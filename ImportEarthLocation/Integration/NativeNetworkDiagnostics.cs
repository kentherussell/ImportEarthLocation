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
    private static readonly Dictionary<Entity,(Entity node,float requested,string prefab)> RoundaboutIslands=new();
    public static IReadOnlyDictionary<string,Entity> VerifiedNodes { get; private set; }=new Dictionary<string,Entity>();
    public static IReadOnlyDictionary<(NetworkKind Kind,long WayId,long Start,long End),List<Entity>> VerifiedPaths { get; private set; }=new Dictionary<(NetworkKind,long,long,long),List<Entity>>();
    public static void BindVerified(IReadOnlyDictionary<string,Entity> nodes,IReadOnlyDictionary<(NetworkKind Kind,long WayId,long Start,long End),List<Entity>> paths)
    {VerifiedNodes=nodes;VerifiedPaths=paths;}
    public static void Remember(IReadOnlyList<PreparedSegment> sections,Processing.OsmDataset? evidence=null)
    {
        _sections=sections;_evidence=evidence;
        RoundaboutIslands.Clear();
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
    public static void Clear(){_expected.Clear();_sections=Array.Empty<PreparedSegment>();_evidence=null;RoundaboutIslands.Clear();BindVerified(new Dictionary<string,Entity>(),new Dictionary<(NetworkKind,long,long,long),List<Entity>>());}
    public static void Check(EntityManager manager,ImportLog log,int sectionLimit=int.MaxValue)
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
        foreach(var pair in _expected.Where(p=>p.Value.Count>1).Take(sectionLimit))
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
        log.Diagnostic($"NATIVE READBACK BUDGET sectionLimit={sectionLimit}; totalPreparedSections={_sections.Count}; sampledReadback={_sections.Count>sectionLimit}; completeConnectivityNotClaimed={_sections.Count>sectionLimit}");
        NativeNetworkReadback.Check(manager,_sections.Take(sectionLimit).ToArray(),_evidence,log,includeMovements:sectionLimit==int.MaxValue);
        var islandRecords=new List<object>();var attachedIslands=0;
        foreach(var pair in RoundaboutIslands)
        {
            var live=manager.HasComponent<Game.Objects.Attached>(pair.Key)&&!manager.HasComponent<Deleted>(pair.Key);
            var parent=live?manager.GetComponentData<Game.Objects.Attached>(pair.Key).m_Parent:Entity.Null;
            var attached=parent!=Entity.Null&&manager.HasComponent<Game.Objects.Attachment>(parent)&&manager.GetComponentData<Game.Objects.Attachment>(parent).m_Attached==pair.Key;
            var radius=parent!=Entity.Null&&manager.HasComponent<Roundabout>(parent)?manager.GetComponentData<Roundabout>(parent).m_Radius:0;
            if(attached&&radius>0)attachedIslands++;
            islandRecords.Add(new{island=$"{pair.Key.Index}:{pair.Key.Version}",pair.Value.prefab,pair.Value.requested,live,parent=$"{parent.Index}:{parent.Version}",attached,nativeRadius=radius,renderingVerified=false});
        }
        log.DiagnosticRecords("native-roundabout-readback.ndjson",islandRecords);
        log.Diagnostic($"ROUNDABOUT READBACK expectedIslands={RoundaboutIslands.Count}; attachedWithNativeRadius={attachedIslands}; appearanceAndTrafficUnverified=true");
    }

    public static void ApplySmallRoundabouts(EntityManager manager,PrefabSystem prefabs,IReadOnlyList<PreparedSegment> sections,ImportLog log)
    {
        // The stock tools place a NetObject island. A manually assigned node
        // radius is recomputed by NetComponentsSystem and is not the tool contract.
        var palette=new List<(Entity prefab,ObjectData data,string name,float radius)>();
        var resolver=new PrefabResolver(prefabs,log);
        var ringPrefab=resolver.Resolve(new PreparedSegment(0,1,2,default,default,NetworkKind.Road,"primary",2,true,false,false,0));
        var ringHalf=ringPrefab==Entity.Null?0:resolver.ReadWidth(ringPrefab,false)/2;
        foreach(var prefab in NativePrefabCatalog.Read<ObjectPrefab>(manager,prefabs))
        {
            var entity=prefabs.GetEntity(prefab);
            if(!prefab.name.Contains("Roundabout")||!manager.HasComponent<NetObjectData>(entity)||
                (manager.GetComponentData<NetObjectData>(entity).m_CompositionFlags.m_General&CompositionFlags.General.Roundabout)==0||
                !manager.HasComponent<ObjectGeometryData>(entity)||!manager.HasComponent<ObjectData>(entity))continue;
            var geometry=manager.GetComponentData<ObjectGeometryData>(entity);
            var radius=math.max(geometry.m_Size.x,geometry.m_Size.z)/2+ringHalf;
            var data=manager.GetComponentData<ObjectData>(entity);
            using var components=data.m_Archetype.GetComponentTypes(Allocator.Temp);
            var types=new HashSet<ComponentType>();foreach(var type in components)types.Add(type);
            if(!types.Contains(ComponentType.ReadWrite<Game.Objects.Attached>())||!types.Contains(ComponentType.ReadWrite<Game.Objects.Transform>())||!types.Contains(ComponentType.ReadWrite<PrefabRef>()))continue;
            if(!math.isfinite(radius)||radius<=0)continue;
            palette.Add((entity,data,prefab.name,radius));
        }
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
            if(manager.HasComponent<Game.Objects.Attachment>(node)&&manager.GetComponentData<Game.Objects.Attachment>(node).m_Attached!=Entity.Null)
            {log.Diagnostic($"ROUNDABOUT existing attachment retained node={node}; requestedRadius={plan.radius}");continue;}
            if(palette.Count==0){unverified++;log.Warn("Stock roundabout object catalog unavailable; junction retained for inspection.");continue;}
            var selected=palette.OrderBy(p=>Math.Abs(p.radius-plan.radius)).ThenByDescending(p=>p.radius).First();
            var island=manager.CreateEntity(selected.data.m_Archetype);
            try
            {
                manager.SetComponentData(island,new PrefabRef{m_Prefab=selected.prefab});
                manager.SetComponentData(island,new Game.Objects.Transform{m_Position=manager.GetComponentData<Node>(node).m_Position,m_Rotation=quaternion.identity});
                manager.SetComponentData(island,new Game.Objects.Attached{m_Parent=node,m_OldParent=Entity.Null,m_CurvePosition=0});
                if(!manager.HasComponent<Updated>(island))manager.AddComponentData(island,default(Updated));
                RoundaboutIslands[island]=(node,plan.radius,selected.name);
                log.Diagnostic($"ROUNDABOUT stock object created prefab={selected.name}; object={island}; node={node}; selectedCenterlineRadius={selected.radius}; requestedRadius={plan.radius}; nativeAttachmentRebuildPending=true");
            }
            catch(Exception error){manager.DestroyEntity(island);unverified++;log.Warn("Roundabout island omitted locally: "+error.Message);continue;}
            var upgraded=manager.HasComponent<Upgraded>(node)?manager.GetComponentData<Upgraded>(node):default;
            upgraded.m_Flags.m_General|=CompositionFlags.General.Roundabout;
            if(manager.HasComponent<Upgraded>(node))manager.SetComponentData(node,upgraded);else manager.AddComponentData(node,upgraded);
            if(!manager.HasComponent<Updated>(node))manager.AddComponentData(node,default(Updated));applied++;
        }
        log.Diagnostic($"ROUNDABOUT stock islands created={applied}; unverifiedJunctions={unverified}; nativeCompositionRebuildRequested=true; nativeAttachmentReadbackPending=true; renderedRadiusRequiresGameReview=true");
    }
}
