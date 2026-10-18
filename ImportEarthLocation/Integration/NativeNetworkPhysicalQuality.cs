using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Game.Common;
using Game.Net;
using Game.Prefabs;
using Game.Tools;
using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Entities;

namespace ImportEarthLocation.Integration;

public static partial class NativeNetworkDiagnostics
{
    private sealed record PhysicalObservation(Entity StartNode,Entity EndNode,Unity.Mathematics.float3 Start,Unity.Mathematics.float3 End,
        Cubic3f Curve,Entity Composition,NativeCompositionEnvelope Envelope,int CensusOffset,int Frame);
    private static readonly Dictionary<Entity,PhysicalObservation> PhysicalCensus=new();
    private static void CapturePhysicalSlice(EntityManager manager,IReadOnlyList<Entity> scope,int offset)
    {
        foreach(var entity in scope)
        {
            if(!manager.HasComponent<Edge>(entity)||!manager.HasComponent<Curve>(entity)||manager.HasComponent<Deleted>(entity)||manager.HasComponent<Temp>(entity)||!manager.HasComponent<Composition>(entity))continue;
            var edge=manager.GetComponentData<Edge>(entity);var composition=manager.GetComponentData<Composition>(entity).m_Edge;
            if(!manager.HasComponent<Node>(edge.m_Start)||!manager.HasComponent<Node>(edge.m_End)||!manager.HasComponent<NetCompositionData>(composition))continue;
            var data=manager.GetComponentData<NetCompositionData>(composition);var c=manager.GetComponentData<Curve>(entity).m_Bezier;
            var mode=(data.m_Flags.m_General&CompositionFlags.General.Tunnel)!=0?NativeStructureMode.Tunnel:
                (data.m_Flags.m_General&CompositionFlags.General.Elevated)!=0?NativeStructureMode.Elevated:NativeStructureMode.Ground;
            var valid=Unity.Mathematics.math.isfinite(data.m_Width)&&data.m_Width>0&&Unity.Mathematics.math.isfinite(data.m_NodeOffset)&&data.m_NodeOffset>=0&&
                Unity.Mathematics.math.isfinite(data.m_SurfaceHeight.min)&&Unity.Mathematics.math.isfinite(data.m_SurfaceHeight.max)&&data.m_SurfaceHeight.min<=data.m_SurfaceHeight.max&&
                Unity.Mathematics.math.isfinite(data.m_HeightRange.min)&&Unity.Mathematics.math.isfinite(data.m_HeightRange.max)&&data.m_HeightRange.min<=data.m_HeightRange.max;
            PhysicalCensus[entity]=new(edge.m_Start,edge.m_End,manager.GetComponentData<Node>(edge.m_Start).m_Position,manager.GetComponentData<Node>(edge.m_End).m_Position,
                new(c.a,c.b,c.c,c.d),composition,new(mode,0,data.m_Width,data.m_SurfaceHeight.min,data.m_SurfaceHeight.max,
                    data.m_HeightRange.min,data.m_HeightRange.max,data.m_NodeOffset,valid),offset,UnityEngine.Time.frameCount);
        }
    }
    public static IEnumerable<int> CapturePhysicalQuality(EntityManager manager,ushort[] raster,float scale,double width,bool registered,ImportLog log)
    {
        var map=_map;var generation=_generation;
        var owners=new Dictionary<Entity,List<PreparedSegment>>();
        foreach(var s in _sections)
        {
            if(!VerifiedPaths.TryGetValue((s.Kind,s.WayId,s.StartNodeId,s.EndNodeId),out var path))continue;
            foreach(var entity in path){if(!owners.TryGetValue(entity,out var list))owners[entity]=list=new();list.Add(s);}
        }
        var models=new List<PreparedSegment>();var assets=new Dictionary<string,NativeAssetContract>(StringComparer.Ordinal);
        var contractKeys=new Dictionary<(Entity,NativeCompositionEnvelope),string>();
        var references=new List<object>(128);var sourceMaps=new List<string>();var unavailable=0;var entities=PhysicalCensus.Keys.OrderBy(e=>e.Index).ThenBy(e=>e.Version).ToArray();
        long Node(Entity entity)=>long.MinValue+((long)(uint)entity.Index<<32)+(uint)entity.Version;
        for(var offset=0;offset<entities.Length;offset+=128)
        {
            if(!ImportSession.Matches(map,generation))yield break;
            references.Clear();
            foreach(var entity in entities.Skip(offset).Take(128))
            {
                if(!owners.TryGetValue(entity,out var source)||source.Select(s=>s.Layer).Distinct().Count()!=1||source.Select(s=>s.Kind).Distinct().Count()!=1)
                {unavailable++;continue;}
                var observation=PhysicalCensus[entity];var data=observation.Envelope;var mode=data.Mode;
                var observedCurve=observation.Curve;
                if(!NetworkDesignChecks.Finite(new(observedCurve.P0,observedCurve.P1,observedCurve.P2,observedCurve.P3))||
                    !Unity.Mathematics.math.all(Unity.Mathematics.math.isfinite(observation.Start))||!Unity.Mathematics.math.all(Unity.Mathematics.math.isfinite(observation.End)))
                {unavailable++;continue;}
                if(!contractKeys.TryGetValue((observation.Composition,data),out var key))
                {
                    key="actual-composition-"+assets.Count;contractKeys.Add((observation.Composition,data),key);
                    assets[key]=new(key,new[]{data},Array.Empty<float>());
                }
                var curve=observation.Curve;var template=source[0];
                var paired=source.Select(s=>(s.PairedProfileId,s.PairedReversed)).Distinct().ToArray();
                models.Add(template with{StartNodeId=Node(observation.StartNode),EndNodeId=Node(observation.EndNode),Start=observation.Start,End=observation.End,
                    FinalDeckCurve=curve,StartCurveOffset=default,EndCurveOffset=default,
                    NativeAssetKey=key,NativeWidth=data.Width,Bridge=mode==NativeStructureMode.Elevated,Tunnel=mode==NativeStructureMode.Tunnel,
                    PairedProfileId=paired.Length==1?paired[0].PairedProfileId:0,PairedReversed=paired.Length==1&&paired[0].PairedReversed,
                    SourceGeometry=Array.Empty<Unity.Mathematics.float3>(),OriginalGeometry=Array.Empty<Unity.Mathematics.float3>(),HorizontalParent=null});
                references.Add(new{section=models.Count-1,entity=entity.ToString(),startNode=observation.StartNode.ToString(),endNode=observation.EndNode.ToString(),
                    composition=key,source=source.Select(s=>new{s.WayId,s.StartNodeId,s.EndNodeId,s.Layer,s.PairedProfileId,s.PairedReversed}).ToArray(),
                    canonicalGeometry="native-owned-final-"+observation.CensusOffset+".ndjson.gz",frame=observation.Frame});
            }
            var name="native-owned-final-physical-source-map-"+offset+".ndjson";
            log.DiagnosticRecordChunks(name,references);sourceMaps.Add(name.Substring(0,name.Length-7)+"-chunks.json");
            yield return 0;
        }
        var contractsCaptured=log.DiagnosticCompressedRecords("native-owned-final-composition-contracts.ndjson.gz",assets.Values.Cast<object>());
        // Capture the exact scalar audit models too. Canonical curves alone do
        // not retain source class, grade policy, offsets and formation metadata;
        // rebuilding those by proximity would not reproduce this calculation.
        var modelsCaptured=log.DiagnosticCompressedRecords("native-owned-final-physical-models.ndjson.gz",DiagnosticArtifacts.ExactNetworkRecords(models));
        log.DiagnosticJson("native-owned-final-physical-manifest.json",new{sourceMaps,contracts="native-owned-final-composition-contracts.ndjson.gz",
            modelFile="native-owned-final-physical-models.ndjson.gz",modelsCaptured,contractsCaptured,nativeRaster="native-observed-heightmap.u16",models=models.Count,unavailable,distributedAcrossFrames=true,singleInstantSnapshot=false});
        log.DiagnosticHeights("native-observed-heightmap.u16",raster);
        log.DiagnosticJson("native-observed-terrain.json",new{samples=raster.Length,scale,width,registered,
            heightFile="native-observed-heightmap.u16",encoding="little-endian-uint16-centred-corner-grid",distributedAcrossFrames=true,singleInstantSnapshot=false});
        void Status(string state,string reason="")
        {
            log.DiagnosticJson("native-physical-audit-status.json",new{state,reason,models=models.Count,unavailable,modelsCaptured,contractsCaptured,
                map,generation,registered,calculationCompletionIsNotReplayCompleteness=true,placementBlocked=false});
            log.Diagnostic($"NATIVE PHYSICAL AUDIT state={state}; sections={models.Count}; modelsCaptured={modelsCaptured}; contractsCaptured={contractsCaptured}; reason={reason}; noPlacementGate=true");
        }
        if(!registered||!(scale>0)||models.Count==0)
        {
            Status("unavailable","terrain-registration-or-source-model-unavailable");
            log.Quality(new("native-physical","clearance-cover-ground-support","final-owned",models.Count,0,Math.Max(1,unavailable),0,0,
                "Actual terrain registration or source/composition contract unavailable; no substituted planned terrain or guessed source layer"));yield break;
        }
        using var cancel=new CancellationTokenSource();var token=cancel.Token;var sections=models.ToArray();
        var task=Task.Run(()=>
        {
            var quality=NetworkTunnelProfile.MeasurePhysical(sections,raster,scale,width,assets,token);
            NetworkQualityFinding? ground=null;
            NetworkTerrainSupport.Validate(sections,raster,scale,width,log,token,scope:"native-observed-ground",stage:"native-physical",result:value=>ground=value,publish:false);
            return (Physical:quality,Ground:ground);
        },token);
        var clock=System.Diagnostics.Stopwatch.StartNew();var next=0d;
        var terminal=false;Status("running");
        try
        {
            while(!task.IsCompleted)
            {
                if(!ImportSession.Matches(map,generation)){cancel.Cancel();yield break;}
                if(clock.Elapsed.TotalSeconds>=next){log.Diagnostic($"NATIVE PHYSICAL AUDIT elapsedSeconds={clock.Elapsed.TotalSeconds:F1}; sections={sections.Length}; pending=true; workerOnly=true; noPlacementGate=true");next=clock.Elapsed.TotalSeconds+5;}
                yield return 0;
            }
            if(task.IsCanceled)yield break;
            if(task.IsFaulted)
            {terminal=true;Status("failed",task.Exception!.GetBaseException().Message);log.Quality(new("native-physical","clearance-cover-ground-support","final-owned",sections.Length,0,sections.Length,0,0,task.Exception!.GetBaseException().Message));yield break;}
            if(!ImportSession.Matches(map,generation))yield break;
            var result=task.Result.Physical;if(task.Result.Ground!=null)log.Quality(task.Result.Ground);
            log.Quality(new("native-physical","full-width-clearance","final-owned",(int)Math.Min(int.MaxValue,result.ClearanceRows),
                (int)Math.Min(int.MaxValue,result.ClearanceFailures),unavailable+result.UncertifiedClearance,result.MaximumClearanceResidual,result.ClearanceAffectedLength,
                "Actual source-qualified native curves/composition envelopes; distributed observation, not one instantaneous snapshot"));
            log.Quality(new("native-physical","continuous-tunnel-cover","final-owned",(int)Math.Min(int.MaxValue,result.CoverRows),
                (int)Math.Min(int.MaxValue,result.CoverFailures),unavailable+result.UncertifiedCover,result.MaximumCoverResidual,result.CoverAffectedLength,
                "Actual captured native encoded terrain and roof envelopes; distributed temporal scope"));
            log.Quality(new("native-physical","same-level-overlap","final-owned",sections.Length,(int)Math.Min(int.MaxValue,result.HorizontalConflictPairs),
                unavailable+result.UncertifiedClearance,result.HorizontalConflictPairs,0,"Actual full-width footprints outside source-qualified formations"));
            terminal=true;Status("completed");
        }
        finally
        {
            if(!terminal)Status("canceled","session-changed-or-readback-disposed");
            if(!task.IsCompleted){cancel.Cancel();_ = task.ContinueWith(t=>{var observed=t.Exception;},TaskContinuationOptions.OnlyOnFaulted);}
        }
    }
}
