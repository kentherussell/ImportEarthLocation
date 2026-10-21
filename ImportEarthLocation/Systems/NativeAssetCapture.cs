using System;
using System.Collections.Generic;
using System.Linq;
using Game.Net;
using Game.Prefabs;
using ImportEarthLocation.Core;
using ImportEarthLocation.Integration;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace ImportEarthLocation.Systems
{
public sealed partial class ImportEarthLocationUISystem
{
    private IReadOnlyList<NativeAssetContract> CaptureNativeAssets(IReadOnlyList<PreparedSegment> segments)
    {
        if(segments.Count==0)return Array.Empty<NativeAssetContract>();
        _log.BeginStage(TelemetryStage.AssetCapture);
        // This method executes on the UI system's main thread. The verified
        // helper contract cannot be silently substituted after an engine update.
        if(typeof(NetCompositionData).Assembly.ManifestModule.ModuleVersionId!=new Guid("e15551e9-3dea-4855-8dde-1da3cea7d7ef"))
            throw new InvalidOperationException("Native composition helper contract differs from the verified engine; asset capture is unavailable.");
        EntityManager.CompleteAllTrackedJobs();
        var subSections=GetBufferLookup<NetSubSection>(true);
        var sectionPieces=GetBufferLookup<NetSectionPiece>(true);
        var pieceData=GetComponentLookup<NetPieceData>(true);
        var matchData=GetComponentLookup<NetVertexMatchData>(true);
        var resolver=new PrefabResolver(World.GetOrCreateSystemManaged<PrefabSystem>(),_log);
        var seen=new HashSet<string>(StringComparer.Ordinal);var output=new List<NativeAssetContract>();
        // Structural intent may select either composition before freeze. Read
        // both palette variants now; an internally selected bridge is not a
        // late asset change requiring another city-wide optimization pass.
        var palette=segments.GroupBy(s=>NativeAssetKeys.Sized(s with{Bridge=false})).SelectMany(g=>new[]{
            g.First() with{Bridge=false,Tunnel=false},g.First() with{Bridge=true,Tunnel=false}});
        foreach(var segment in palette)
        {
            var key=PrefabResolver.SizedPaletteKey(segment);if(!seen.Add(key))continue;
            var prefab=resolver.Resolve(segment);
            if(prefab==Entity.Null||!EntityManager.Exists(prefab)||!EntityManager.HasComponent<PrefabData>(prefab)||
                !EntityManager.HasComponent<NetGeometryData>(prefab)||!EntityManager.HasBuffer<NetGeometrySection>(prefab))
            {_log.Warn("Native composition metadata unavailable for "+key);continue;}
            var sections=EntityManager.GetBuffer<NetGeometrySection>(prefab,true);
            var geometry=EntityManager.GetComponentData<NetGeometryData>(prefab);
            var enabled=EntityManager.IsComponentEnabled<PrefabData>(prefab);
            var envelopes=new NativeCompositionEnvelope[48];var unavailable=0;
            for(var mode=0;mode<3;mode++)for(byte side=0;side<16;side++)
            {
                var flags=new CompositionFlags{m_General=CompositionFlags.General.Edge};
                if(mode==1)flags.m_General|=CompositionFlags.General.Elevated;
                if(mode==2)flags.m_General|=CompositionFlags.General.Tunnel;
                if((side&1)!=0)flags.m_Left|=CompositionFlags.Side.Raised;
                if((side&2)!=0)flags.m_Left|=CompositionFlags.Side.Lowered;
                if((side&4)!=0)flags.m_Right|=CompositionFlags.Side.Raised;
                if((side&8)!=0)flags.m_Right|=CompositionFlags.Side.Lowered;
                using var pieces=new NativeList<NetCompositionPiece>(Allocator.Temp);
                NetCompositionHelpers.GetCompositionPieces(pieces,sections.AsNativeArray(),flags,subSections,sectionPieces);
                var composition=new NetCompositionData{m_Flags=flags};
                NetCompositionHelpers.CalculateCompositionData(ref composition,pieces.AsArray(),pieceData,matchData);
                if(!enabled)
                {
                    composition.m_Width=mode==1?geometry.m_ElevatedWidth:geometry.m_DefaultWidth;
                    composition.m_HeightRange=mode==1?geometry.m_ElevatedHeightRange:geometry.m_DefaultHeightRange;
                    composition.m_SurfaceHeight=geometry.m_DefaultSurfaceHeight;
                }
                var valid=math.isfinite(composition.m_Width)&&composition.m_Width>0&&
                    math.isfinite(composition.m_NodeOffset)&&composition.m_NodeOffset>=0&&
                    math.isfinite(composition.m_SurfaceHeight.min)&&math.isfinite(composition.m_SurfaceHeight.max)&&
                    composition.m_SurfaceHeight.min<=composition.m_SurfaceHeight.max&&
                    math.isfinite(composition.m_HeightRange.min)&&math.isfinite(composition.m_HeightRange.max)&&
                    composition.m_HeightRange.min<=composition.m_HeightRange.max;
                if(!valid)unavailable++;
                envelopes[mode*16+side]=new((NativeStructureMode)mode,side,composition.m_Width,
                    composition.m_SurfaceHeight.min,composition.m_SurfaceHeight.max,
                    composition.m_HeightRange.min,composition.m_HeightRange.max,composition.m_NodeOffset,valid);
            }
            float? trainLimit=null;
            if(segment.Kind==NetworkKind.Rail)
            {
                try {trainLimit=CaptureTrainCurvature(prefab);}
                catch(InvalidOperationException error)
                {_log.Warn($"Native train curvature metadata unavailable for {key}: {error.Message} Rail curvature remains RequiresReview; available roads and rail are not discarded.");}
            }
            PreparedSegment metadata;
            try {metadata=resolver.ResolveMetadata(segment);}
            catch(InvalidOperationException error)
            {
                _log.Warn($"Native composition metadata unavailable for {key}: {error.Message}. This palette remains unavailable; other palettes continue.");
                continue;
            }
            output.Add(new NativeAssetContract(key,envelopes,metadata.NativeLaneOffsets,trainLimit));
            if(_log.DiagnosticsEnabled)_log.Diagnostic($"NATIVE ASSET CONTRACT palette={key}; envelopes=48; unavailable={unavailable}; guessedOffsets=false");
        }
        return output.AsReadOnly();
    }

    private float CaptureTrainCurvature(Entity prefab)
    {
        var prefabs=World.GetOrCreateSystemManaged<PrefabSystem>();var minimum=float.PositiveInfinity;var found=false;
        void Observe(Entity lane)
        {
            if(!EntityManager.HasComponent<TrackLaneData>(lane))return;
            var data=EntityManager.GetComponentData<TrackLaneData>(lane);
            if((data.m_TrackTypes&TrackTypes.Train)==0)return;
            if(!math.isfinite(data.m_MaxCurviness)||data.m_MaxCurviness<=0)
                throw new InvalidOperationException("Selected train lane has no finite positive native curvature contract.");
            minimum=Math.Min(minimum,data.m_MaxCurviness);found=true;
        }
        if(EntityManager.HasBuffer<DefaultNetLane>(prefab))
            foreach(var lane in EntityManager.GetBuffer<DefaultNetLane>(prefab,true))Observe(lane.m_Lane);
        // Top-level DefaultNetLane may be absent. Traverse the selected asset's
        // actual section/piece lane entities; never substitute authoring defaults.
        if(prefabs.TryGetPrefab<TrackPrefab>(prefab,out var track)&&track!=null)
        {
            var visited=new HashSet<NetSectionPrefab>();
            void Visit(NetSectionPrefab section)
            {
                if(section==null||!visited.Add(section))return;
                if(section.m_Pieces!=null)foreach(var piece in section.m_Pieces)
                    if(piece.m_Piece!=null&&piece.m_Piece.TryGet<NetPieceLanes>(out var lanes)&&lanes.m_Lanes!=null)
                        foreach(var lane in lanes.m_Lanes)if(lane.m_Lane!=null)Observe(prefabs.GetEntity(lane.m_Lane));
                if(section.m_SubSections!=null)foreach(var child in section.m_SubSections)Visit(child.m_Section);
            }
            if(track.m_Sections!=null)foreach(var section in track.m_Sections)Visit(section.m_Section);
        }
        if(!found)throw new InvalidOperationException("Selected railway has no available native train-lane curvature metadata; no guessed rail radius is permitted.");
        return minimum;
    }
}
}
