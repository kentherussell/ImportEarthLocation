using System;
using System.Collections.Generic;
using System.Linq;
using Game.Common;
using Game.Net;
using Game.Prefabs;
using Game.Tools;
using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using SubLane = Game.Net.SubLane;
using CarLane = Game.Net.CarLane;
using TrackLane = Game.Net.TrackLane;

namespace ImportEarthLocation.Integration;

// Prepared counts are not native acceptance. Follow verified entity/edge-chain
// lineage; only unbound historical sections use geometric diagnostic matching.
// Inspect actual compositions on every live subdivision edge.
public static class NativeNetworkReadback
{
    private sealed record Live(Entity Entity,Entity A,Entity B,float3 Start,float3 End,NetworkKind Kind,Colossal.Mathematics.Bezier4x3 Curve);
    public static void Check(EntityManager manager,IReadOnlyList<PreparedSegment> sections,OsmDataset? evidence,ImportLog log,bool includeMovements=true)
    {
        manager.CompleteAllTrackedJobs();
        using var query=manager.CreateEntityQuery(new EntityQueryDesc{All=new[]{ComponentType.ReadOnly<Edge>(),ComponentType.ReadOnly<Curve>(),ComponentType.ReadOnly<PrefabRef>()},None=new[]{ComponentType.ReadOnly<Temp>(),ComponentType.ReadOnly<Deleted>()}});
        using var entities=query.ToEntityArray(Allocator.Temp);var grid=new Dictionary<(int,int),List<Live>>();
        var byIndex=new Dictionary<int,Live>();var byEntity=new Dictionary<Entity,Live>();var anchors=new Dictionary<(NetworkKind,long),HashSet<Entity>>();
        (int,int) Cell(float3 p)=>((int)Math.Floor(p.x/8),(int)Math.Floor(p.z/8));
        foreach(var entity in entities)
        {
            var prefab=manager.GetComponentData<PrefabRef>(entity).m_Prefab;var road=manager.HasComponent<RoadData>(prefab);if(!road&&!manager.HasComponent<TrackData>(prefab))continue;
            var edge=manager.GetComponentData<Edge>(entity);if(!manager.HasComponent<Node>(edge.m_Start)||!manager.HasComponent<Node>(edge.m_End))continue;
            var live=new Live(entity,edge.m_Start,edge.m_End,manager.GetComponentData<Node>(edge.m_Start).m_Position,manager.GetComponentData<Node>(edge.m_End).m_Position,road?NetworkKind.Road:NetworkKind.Rail,manager.GetComponentData<Curve>(entity).m_Bezier);
            byIndex[entity.Index]=live;
            byEntity[entity]=live;
            foreach(var cell in new[]{Cell(live.Start),Cell(live.End)}.Distinct()){if(!grid.TryGetValue(cell,out var list))grid[cell]=list=new();list.Add(live);}
        }
        var owners=new Dictionary<int,HashSet<long>>();var matched=0;var missing=0;var compositionMissing=0;var exposed=0;var laneMismatch=0;var lineageSections=0;var inspected=new HashSet<Entity>();
        var readbackRecords=new List<object>();
        foreach(var s in sections)
        {
            var chain=new List<Live>();var bound=NativeNetworkDiagnostics.VerifiedPaths.TryGetValue((s.Kind,s.WayId,s.StartNodeId,s.EndNodeId),out var path);
            var valid=true;var a=Entity.Null;var b=Entity.Null;
            if(bound)
            {
                valid=NativeNetworkDiagnostics.VerifiedNodes.TryGetValue(NativeEndpointPolicy.Identity(s,s.StartNodeId),out a)&&
                    NativeNetworkDiagnostics.VerifiedNodes.TryGetValue(NativeEndpointPolicy.Identity(s,s.EndNodeId),out b);
                var cursor=a;
                foreach(var entity in path!)
                {
                    if(!valid||!byEntity.TryGetValue(entity,out var e)||e.Kind!=s.Kind){valid=false;break;}
                    if(e.A==cursor)cursor=e.B;else if(e.B==cursor)cursor=e.A;else{valid=false;break;}
                    chain.Add(e);
                }
                valid=valid&&chain.Count>0&&cursor==b;
                if(valid)lineageSections++;
            }
            var cell=Cell(s.Start);var candidates=new HashSet<Live>();
            if(!bound)for(var dx=-1;dx<=1;dx++)for(var dz=-1;dz<=1;dz++)if(grid.TryGetValue((cell.Item1+dx,cell.Item2+dz),out var list))foreach(var e in list)
                if(e.Kind==s.Kind&&((math.distance(e.Start,s.Start)<2&&math.distance(e.End,s.End)<2)||(math.distance(e.Start,s.End)<2&&math.distance(e.End,s.Start)<2)))candidates.Add(e);
            if(!bound)
            {
                var midpoint=NetworkNormalizer.Evaluate(s,.5f);
                var candidate=candidates.OrderBy(e=>Distance(e.Curve,midpoint)).FirstOrDefault();
                valid=candidate!=null&&Distance(candidate.Curve,midpoint)<=Math.Max(4,s.NativeWidth*.5f);
                if(valid)
                {
                    chain.Add(candidate!);var forward=math.distance(candidate!.Start,s.Start)<=math.distance(candidate.End,s.Start);
                    a=forward?candidate.A:candidate.B;b=forward?candidate.B:candidate.A;
                }
            }
            if(!valid)
            {
                missing++;if(missing<=25)log.Diagnostic($"NATIVE COVERAGE unresolved kind={s.Kind}; way={s.WayId}; nodes={s.StartNodeId},{s.EndNodeId}; sourceIds={string.Join(",",s.SourceWayIds)}; recordedLineage={bound}; matchingBothAnchorsAndInterior={!bound}; legitimateTerminal={s.StartTerminal||s.EndTerminal}; noDuplicateRetry=true");
                if(readbackRecords.Count<8192)readbackRecords.Add(new { outcome="unresolved",s.WayId,s.StartNodeId,s.EndNodeId,kind=s.Kind.ToString(),start=DiagnosticArtifacts.Point(s.Start),end=DiagnosticArtifacts.Point(s.End),s.SourceWayIds,candidateEdges=candidates.Count,recordedLineage=bound });
                continue;
            }
            matched++;Anchor(s.Kind,s.StartNodeId,a);Anchor(s.Kind,s.EndNodeId,b);
            foreach(var live in chain)
            {
            if(!owners.TryGetValue(live.Entity.Index,out var ways))owners[live.Entity.Index]=ways=new();ways.UnionWith(s.SourceWayIds.Count>0?s.SourceWayIds:new[]{s.WayId});
            if(!inspected.Add(live.Entity))continue;
            var flags=default(CompositionFlags);var width=0f;
            if(manager.HasComponent<Composition>(live.Entity))
            {var composition=manager.GetComponentData<Composition>(live.Entity).m_Edge;if(manager.HasComponent<NetCompositionData>(composition)){var data=manager.GetComponentData<NetCompositionData>(composition);flags=data.m_Flags;width=data.m_Width;}else compositionMissing++;}
            else compositionMissing++;
            if(s.Tunnel&&(flags.m_General&CompositionFlags.General.Tunnel)==0)exposed++;
            var cars=0;var tracks=0;var reverse=0;var speeds=new List<float>();
            if(manager.HasBuffer<SubLane>(live.Entity))foreach(var laneRef in manager.GetBuffer<SubLane>(live.Entity,true))
            {
                var lane=laneRef.m_SubLane;
                if(manager.HasComponent<CarLane>(lane)){var car=manager.GetComponentData<CarLane>(lane);if((car.m_Flags&(CarLaneFlags.ParkingLeft|CarLaneFlags.ParkingRight|CarLaneFlags.SideConnection))!=0)continue;cars++;if((car.m_Flags&CarLaneFlags.Invert)!=0)reverse++;speeds.Add(car.m_SpeedLimit);}
                if(manager.HasComponent<TrackLane>(lane))tracks++;
            }
            var requested=s.Kind==NetworkKind.Rail?(s.PhysicalTracks>0?s.PhysicalTracks:s.Lanes):s.Lanes;var count=s.Kind==NetworkKind.Rail?tracks:cars;
            if(count!=requested)laneMismatch++;
            if(readbackRecords.Count<8192&&(inspected.Count<=256||s.Tunnel||count!=requested))readbackRecords.Add(new
            {
                outcome="matched",s.WayId,s.SourceWayIds,s.StartNodeId,s.EndNodeId,kind=s.Kind.ToString(),entity=live.Entity.ToString(),
                startNode=live.A.ToString(),endNode=live.B.ToString(),requestedLanesOrTracks=requested,actualCarLanes=cars,actualTrackLanes=tracks,
                nativeWidth=width,plannedWidth=s.NativeWidth,tunnelRequested=s.Tunnel,nativeGeneralFlags=flags.m_General.ToString(),nativeLeftFlags=flags.m_Left.ToString(),nativeRightFlags=flags.m_Right.ToString(),railAutomaticSidesObserved=s.Kind==NetworkKind.Rail&&(flags.m_Left!=0||flags.m_Right!=0),recordedLineage=bound,sourceChainEdges=chain.Count,
                a=DiagnosticArtifacts.Point(live.Curve.a),b=DiagnosticArtifacts.Point(live.Curve.b),c=DiagnosticArtifacts.Point(live.Curve.c),d=DiagnosticArtifacts.Point(live.Curve.d),
                renderedAcceptanceUnverified=true
            });
            if(inspected.Count<=80||s.Tunnel||count!=requested)log.Diagnostic($"NATIVE NETWORK EDGE entity={live.Entity}; kind={s.Kind}; way={s.WayId}; requestedLanesOrTracks={requested}; actualCarLanes={cars}; actualTrackLanes={tracks}; invertedCarLanes={reverse}; requestedOneWay={s.OneWay}; compositionWidth={width:R}; plannedWidth={s.NativeWidth:R}; tunnelRequested={s.Tunnel}; nativeGeneralFlags={flags.m_General}; nativeLeftFlags={flags.m_Left}; nativeRightFlags={flags.m_Right}; nativeSpeeds={string.Join(",",speeds.Distinct())}; renderedAcceptanceUnverified=true");
            }
        }
        log.DiagnosticRecords("native-network-readback.ndjson",readbackRecords);
        log.Diagnostic($"NATIVE READBACK ARTIFACT records={readbackRecords.Count}; recordLimit=8192; matchesSampled=true; unresolvedTotal={missing}; lineageQualifiedSections={lineageSections}; laneMismatchTotal={laneMismatch}; fileNotFullCoverageProof=true");
        // Junction lanes expose ordered source/target edge owners. This proves
        // movement existence, separately from two edges sharing a node anchor.
        if(!includeMovements)
        {
            log.Diagnostic($"NATIVE ACCEPTANCE SAMPLE matchedSections={matched}/{sections.Count}; unresolvedSections={missing}; uniqueEdgesInspected={inspected.Count}; unavailableCompositions={compositionMissing}; tunnelFlagMismatches={exposed}; fullLaneGraphScanDeferred=true; completeConnectivityNotClaimed=true");
            return;
        }
        var movements=new HashSet<(NetworkKind Kind,long From,long To,Entity Via)>();var nodeLaneCount=0;var unlocatedLanes=0;
        using var lanesQuery=manager.CreateEntityQuery(new EntityQueryDesc{All=new[]{ComponentType.ReadOnly<Lane>()},Any=new[]{ComponentType.ReadOnly<CarLane>(),ComponentType.ReadOnly<TrackLane>()},None=new[]{ComponentType.ReadOnly<Deleted>(),ComponentType.ReadOnly<Temp>()}});
        using var lanes=lanesQuery.ToEntityArray(Allocator.Temp);
        foreach(var entity in lanes)
        {
            var lane=manager.GetComponentData<Lane>(entity);var kind=manager.HasComponent<CarLane>(entity)?NetworkKind.Road:NetworkKind.Rail;
            var invert=false;var twoway=false;
            if(kind==NetworkKind.Road)
            {var car=manager.GetComponentData<CarLane>(entity);if((car.m_Flags&(CarLaneFlags.Forbidden|CarLaneFlags.ParkingLeft|CarLaneFlags.ParkingRight|CarLaneFlags.SideConnection))!=0)continue;invert=(car.m_Flags&CarLaneFlags.Invert)!=0;twoway=(car.m_Flags&CarLaneFlags.Twoway)!=0;}
            else{var track=manager.GetComponentData<TrackLane>(entity);invert=(track.m_Flags&TrackLaneFlags.Invert)!=0;twoway=(track.m_Flags&TrackLaneFlags.Twoway)!=0;}
            var a=lane.m_StartNode.GetOwnerIndex();var b=lane.m_EndNode.GetOwnerIndex();
            if(!owners.TryGetValue(a,out var from)||!owners.TryGetValue(b,out var to)||!byIndex.TryGetValue(a,out var edgeA)||!byIndex.TryGetValue(b,out var edgeB)||edgeA.Kind!=kind||edgeB.Kind!=kind)continue;
            var via=Entity.Null;
            if(manager.HasComponent<Owner>(entity))
            {var owner=manager.GetComponentData<Owner>(entity).m_Owner;if(manager.HasComponent<Node>(owner))via=owner;}
            if(via==Entity.Null&&a!=b)
            {if(edgeA.A==edgeB.A||edgeA.A==edgeB.B)via=edgeA.A;else if(edgeA.B==edgeB.A||edgeA.B==edgeB.B)via=edgeA.B;}
            if(via==Entity.Null){if(a!=b)unlocatedLanes++;continue;}
            nodeLaneCount++;
            foreach(var f in from)foreach(var t in to)
            {movements.Add(invert?(kind,t,f,via):(kind,f,t,via));if(twoway)movements.Add(invert?(kind,f,t,via):(kind,t,f,via));}
        }
        var restrictionChecks=0;var restrictionConflicts=0;var connectivityMissing=0;var unverifiable=0;var representedWays=new HashSet<long>(owners.Values.SelectMany(w=>w));
        var movementsByFrom=movements.GroupBy(m=>(m.Kind,m.From,m.Via)).ToDictionary(g=>g.Key,g=>g.Select(m=>m.To).ToHashSet());
        if(evidence!=null)foreach(var relation in evidence.Relations)
        {
            if(!relation.Tags.TryGetValue("type",out var type)||(type!="restriction"&&type!="connectivity"))continue;
            var from=relation.OrderedMembers.Where(m=>m.Type=="way"&&m.Role=="from").Select(m=>m.Id).ToArray();var to=relation.OrderedMembers.Where(m=>m.Type=="way"&&m.Role=="to").Select(m=>m.Id).ToArray();
            if(from.Length!=1||to.Length!=1||!representedWays.Contains(from[0])||!representedWays.Contains(to[0]))continue;
            var viaNodes=relation.OrderedMembers.Where(m=>m.Type=="node"&&m.Role=="via").Select(m=>m.Id).ToArray();
            var actualVia=new HashSet<Entity>();
            if(viaNodes.Length==1&&anchors.TryGetValue((NetworkKind.Road,viaNodes[0]),out var matches))actualVia.UnionWith(matches);
            // Some connectivity relations use a shared endpoint instead of an
            // explicit via node. Derive it only when the source ways agree.
            if(viaNodes.Length==0&&evidence.Ways.TryGetValue(from[0],out var fromWay)&&evidence.Ways.TryGetValue(to[0],out var toWay))
                foreach(var id in fromWay.NodeIds.Intersect(toWay.NodeIds))if(anchors.TryGetValue((NetworkKind.Road,id),out var common))actualVia.UnionWith(common);
            var exists=actualVia.Any(v=>movements.Contains((NetworkKind.Road,from[0],to[0],v)));relation.Tags.TryGetValue("restriction",out var restriction);restrictionChecks++;
            var forbidden=type=="restriction"&&restriction!=null&&restriction.StartsWith("no_",StringComparison.Ordinal);
            var required=type=="connectivity"||restriction!=null&&restriction.StartsWith("only_",StringComparison.Ordinal);
            var qualified=relation.Tags.ContainsKey("except")||relation.Tags.Keys.Any(k=>k.Contains("conditional"))||relation.OrderedMembers.Any(m=>m.Type=="way"&&m.Role=="via")||actualVia.Count!=1;
            var otherExit=actualVia.Any(v=>movementsByFrom.TryGetValue((NetworkKind.Road,from[0],v),out var exits)&&exits.Any(t=>t!=to[0]));
            if(qualified)unverifiable++;
            if(!qualified&&(forbidden&&exists||restriction!=null&&restriction.StartsWith("only_",StringComparison.Ordinal)&&otherExit))restrictionConflicts++;
            if(!qualified&&required&&!exists)connectivityMissing++;
            log.Diagnostic($"NATIVE MOVEMENT relation={relation.Id}; type={type}; from={from[0]}; to={to[0]}; sourceViaNodes={string.Join(",",viaNodes)}; mappedViaAnchors={actualVia.Count}; actualOrderedNodeLaneExists={exists}; otherExitObserved={otherExit}; policy={restriction}; qualifiedOrUnverifiable={qualified}; nativeInvertAndTwowayHonored=true; exactViaAssociation=true; perLaneConnectivityAndViaWaySequenceNotEnforced=true");
        }
        log.Diagnostic($"NATIVE ACCEPTANCE matchedSections={matched}/{sections.Count}; unresolvedSections={missing}; uniqueEdgesInspected={inspected.Count}; unavailableCompositions={compositionMissing}; tunnelFlagMismatches={exposed}; laneCountApproximations={laneMismatch}; orderedRoadAndRailNodeLanes={nodeLaneCount}; unlocatedNodeLanes={unlocatedLanes}; relationsChecked={restrictionChecks}; relationsUnverifiable={unverifiable}; prohibitedMovementsObserved={restrictionConflicts}; requiredMovementsNotObserved={connectivityMissing}; readOnly=true; definitionCountNotAcceptance=true");
        if(missing+exposed+restrictionConflicts+connectivityMissing>0)log.Warn($"Native acceptance needs review: {missing} unresolved sections, {exposed} tunnel-composition mismatches, {restrictionConflicts} prohibited and {connectivityMissing} unobserved required movements. Import remains available for iteration; no unsafe duplicate edges or transient lane edits were created.");
        void Anchor(NetworkKind kind,long id,Entity entity)
        {if(!anchors.TryGetValue((kind,id),out var values))anchors[(kind,id)]=values=new();values.Add(entity);}
    }
    private static float Distance(Colossal.Mathematics.Bezier4x3 c,float3 point)
    {var distance=float.PositiveInfinity;var previous=c.a;for(var i=1;i<=32;i++){var t=i/32f;var u=1-t;var p=u*u*u*c.a+3*u*u*t*c.b+3*u*t*t*c.c+t*t*t*c.d;var delta=p-previous;var ratio=math.clamp(math.dot(point-previous,delta)/Math.Max(.001f,math.lengthsq(delta)),0,1);distance=Math.Min(distance,math.distance(point,previous+delta*ratio));previous=p;}return distance;}
}
