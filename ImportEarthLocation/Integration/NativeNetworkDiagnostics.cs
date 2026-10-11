using System;
using System.Collections.Generic;
using System.Linq;
using Game.Common;
using Game.Net;
using Game.Prefabs;
using Game.Tools;
using ImportEarthLocation.Core;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace ImportEarthLocation.Integration;

// Read the resulting live graph, not the number of definitions submitted to tools.
public static class NativeNetworkDiagnostics
{
    private sealed record Expected(float3 Point,float2 Direction,NetworkKind Kind,bool Structure,long Way);
    private sealed record Actual(float3 Point,float2 Direction,NetworkKind Kind,Entity Node);
    private static Dictionary<string,List<Expected>> _expected=new(StringComparer.Ordinal);
    private static IReadOnlyList<PreparedSegment> _sections=Array.Empty<PreparedSegment>();
    private static Processing.OsmDataset? _evidence;
    public static IReadOnlyDictionary<string,Entity> VerifiedNodes { get; private set; }=new Dictionary<string,Entity>();
    public static IReadOnlyDictionary<(NetworkKind Kind,long WayId,long Start,long End),List<Entity>> VerifiedPaths { get; private set; }=new Dictionary<(NetworkKind,long,long,long),List<Entity>>();
    public static void BindVerified(IReadOnlyDictionary<string,Entity> nodes,IReadOnlyDictionary<(NetworkKind Kind,long WayId,long Start,long End),List<Entity>> paths)
    {VerifiedNodes=nodes;VerifiedPaths=paths;}
    public static void Remember(IReadOnlyList<PreparedSegment> sections,Processing.OsmDataset? evidence=null)
    {
        _sections=sections;_evidence=evidence;
        BindVerified(new Dictionary<string,Entity>(),new Dictionary<(NetworkKind,long,long,long),List<Entity>>());
        _expected=new(StringComparer.Ordinal);
        foreach(var s in sections)foreach(var start in new[]{true,false})
        {
            var point=start?s.Start:s.End;
            var control=start?s.CourseControl1:s.CourseControl2;
            var endpoint=start?s.CourseStart:s.CourseEnd;
            var key=NativeEndpointPolicy.Identity(s,start?s.StartNodeId:s.EndNodeId);
            if(!_expected.TryGetValue(key,out var list))_expected[key]=list=new List<Expected>();
            list.Add(new(point,math.normalizesafe(control.xz-endpoint.xz),s.Kind,NetworkStructure.IsStructure(s),s.WayId));
        }
    }
    public static void Clear(){_expected.Clear();_sections=Array.Empty<PreparedSegment>();_evidence=null;BindVerified(new Dictionary<string,Entity>(),new Dictionary<(NetworkKind,long,long,long),List<Entity>>());}
    public static void Check(EntityManager manager,ImportLog log)
    {
        if(_expected.Count==0)return;
        manager.CompleteAllTrackedJobs();
        using var query=manager.CreateEntityQuery(new EntityQueryDesc{
            All=new[]{ComponentType.ReadOnly<Edge>(),ComponentType.ReadOnly<Curve>(),ComponentType.ReadOnly<PrefabRef>()},
            None=new[]{ComponentType.ReadOnly<Temp>(),ComponentType.ReadOnly<Deleted>()}});
        using var entities=query.ToEntityArray(Allocator.Temp);
        var grid=new Dictionary<(int,int),List<Actual>>();
        (int,int) Cell(float3 p)=>((int)Math.Floor(p.x/4),(int)Math.Floor(p.z/4));
        foreach(var entity in entities)
        {
            var prefab=manager.GetComponentData<PrefabRef>(entity).m_Prefab;
            var road=manager.HasComponent<RoadData>(prefab);var rail=manager.HasComponent<TrackData>(prefab);
            if(!road&&!rail)continue;
            var edge=manager.GetComponentData<Edge>(entity);var curve=manager.GetComponentData<Curve>(entity).m_Bezier;
            Add(curve.a,curve.b,edge.m_Start);Add(curve.d,curve.c,edge.m_End);
            void Add(float3 point,float3 control,Entity node)
            {
                // Native junction geometry trims curve ends and offsets merging
                // lanes. Connectivity belongs to the node anchor, not a trimmed deck end.
                var anchor=manager.HasComponent<Node>(node)?manager.GetComponentData<Node>(node).m_Position:point;
                var cell=Cell(anchor);if(!grid.TryGetValue(cell,out var list))grid[cell]=list=new List<Actual>();
                list.Add(new(anchor,math.normalizesafe(control.xz-point.xz),road?NetworkKind.Road:NetworkKind.Rail,node));
            }
        }
        var checkedJoins=0;var missing=0;var split=0;var portals=0;var badPortals=0;
        foreach(var pair in _expected.Where(p=>p.Value.Count>1))
        {
            checkedJoins++;var portal=pair.Value.Any(e=>e.Structure)&&pair.Value.Any(e=>!e.Structure);if(portal)portals++;
            HashSet<Entity>? common=null;var absent=false;
            foreach(var expected in pair.Value)
            {
                var bound=VerifiedNodes.TryGetValue(pair.Key,out var boundNode);
                var point=bound&&manager.HasComponent<Node>(boundNode)?manager.GetComponentData<Node>(boundNode).m_Position:expected.Point;
                var cell=Cell(point);var candidates=new HashSet<Entity>();
                for(var x=cell.Item1-1;x<=cell.Item1+1;x++)for(var z=cell.Item2-1;z<=cell.Item2+1;z++)
                    if(grid.TryGetValue((x,z),out var list))foreach(var actual in list)
                        if(actual.Kind==expected.Kind&&(bound?actual.Node==boundNode:math.distance(actual.Point,point)<=2)&&math.dot(actual.Direction,expected.Direction)>.7f)candidates.Add(actual.Node);
                if(candidates.Count==0)absent=true;
                if(common==null)common=candidates;else common.IntersectWith(candidates);
            }
            if(!absent&&common!.Count>0)continue;
            if(absent)missing++;else split++;if(portal)badPortals++;
            if(missing+split<=25)log.Diagnostic($"NATIVE CONNECTIVITY mismatch={pair.Key}; ways={string.Join(",",pair.Value.Select(e=>e.Way).Distinct())}; position={pair.Value[0].Point}; missingIncident={absent}; separateNativeNodes={!absent}; structurePortal={portal}; verifiedEntityMatching={VerifiedNodes.ContainsKey(pair.Key)}; geometricFallbackToleranceMeters=2");
        }
        log.Count("Native shared junctions checked",checkedJoins);log.Count("Native junctions missing incident edges",missing);log.Count("Native junctions split across nodes",split);log.Count("Native structure portals mismatched",badPortals);
        log.Diagnostic($"NATIVE CONNECTIVITY SUMMARY liveEdges={entities.Length}; sharedJunctionsChecked={checkedJoins}; missingIncidentEdges={missing}; splitJunctions={split}; structurePortalsChecked={portals}; structurePortalsMismatched={badPortals}; boundSourceNodes={VerifiedNodes.Count}; readOnly=true; verifiedEntityMatchingWhenAvailable=true; geometricFallbackOnlyWhenUnbound=true; saveReloadVerificationRequired=true");
        if(missing+split>0)log.Warn($"Native network check found {missing} junctions with missing/moved edges and {split} with separate nodes, including {badPortals} structure portals. Definition counts are not proof of connectivity; generation is not blocked.");
        NativeNetworkReadback.Check(manager,_sections,_evidence,log);
    }

    public static void ApplySmallRoundabouts(EntityManager manager,IReadOnlyList<PreparedSegment> sections,ImportLog log)
    {
        var planned=new Dictionary<string,(float3 point,float radius)>();
        foreach(var s in sections)
        {
            if(s.StartRoundaboutRadius>0)planned[NativeEndpointPolicy.Identity(s,s.StartNodeId)]=(s.Start,s.StartRoundaboutRadius);
            if(s.EndRoundaboutRadius>0)planned[NativeEndpointPolicy.Identity(s,s.EndNodeId)]=(s.End,s.EndRoundaboutRadius);
        }
        if(planned.Count==0)return;manager.CompleteAllTrackedJobs();
        using var query=manager.CreateEntityQuery(new EntityQueryDesc{All=new[]{ComponentType.ReadOnly<Node>(),ComponentType.ReadOnly<ConnectedEdge>()},None=new[]{ComponentType.ReadOnly<Temp>(),ComponentType.ReadOnly<Deleted>()}});
        using var entities=query.ToEntityArray(Allocator.Temp);
        var grid=new Dictionary<(int,int),List<Entity>>();
        foreach(var entity in entities){var p=manager.GetComponentData<Node>(entity).m_Position;var key=((int)Math.Floor(p.x/4),(int)Math.Floor(p.z/4));if(!grid.TryGetValue(key,out var list))grid[key]=list=new();list.Add(entity);}
        var applied=0;var unverified=0;
        foreach(var pair in planned)
        {
            var plan=pair.Value;
            var x=(int)Math.Floor(plan.point.x/4);var z=(int)Math.Floor(plan.point.z/4);var candidates=new HashSet<Entity>();
            if(VerifiedNodes.TryGetValue(pair.Key,out var boundNode))
            {
                if(manager.HasComponent<Node>(boundNode)&&manager.HasBuffer<ConnectedEdge>(boundNode)&&!manager.HasComponent<Temp>(boundNode)&&!manager.HasComponent<Deleted>(boundNode))candidates.Add(boundNode);
            }
            else for(var dx=-1;dx<=1;dx++)for(var dz=-1;dz<=1;dz++)if(grid.TryGetValue((x+dx,z+dz),out var list))foreach(var entity in list)
                    if(math.distance(manager.GetComponentData<Node>(entity).m_Position,plan.point)<1)candidates.Add(entity);
            if(candidates.Count!=1)
            {unverified++;log.Warn("Small roundabout upgrade could not identify one native junction. Existing approaches retained without guessing a node; the native connectivity audit will report any placement mismatch.");continue;}
            var node=candidates.Single();if(manager.GetBuffer<ConnectedEdge>(node,true).Length<3)
            {unverified++;log.Warn("Small roundabout upgrade found fewer than three native approaches. Junction left unchanged and reported for connectivity review; no road removed by the upgrade.");continue;}
            var upgraded=manager.HasComponent<Upgraded>(node)?manager.GetComponentData<Upgraded>(node):default;
            upgraded.m_Flags.m_General|=CompositionFlags.General.Roundabout;
            if(manager.HasComponent<Upgraded>(node))manager.SetComponentData(node,upgraded);else manager.AddComponentData(node,upgraded);
            if(!manager.HasComponent<Roundabout>(node))manager.AddComponentData(node,new Roundabout{m_Radius=plan.radius});
            if(!manager.HasComponent<Updated>(node))manager.AddComponentData(node,default(Updated));applied++;
        }
        log.Diagnostic($"ROUNDABOUT native nodes applied={applied}; unverifiedJunctions={unverified}; nativeCompositionRebuildRequested=true; upgradedApproachesVerified=true; renderedRadiusRequiresGameReview=true");
    }
}
