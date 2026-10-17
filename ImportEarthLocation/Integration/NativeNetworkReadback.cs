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
    private sealed record Port(PreparedSegment Section,Entity Edge,Entity Via,bool Incoming);
    public readonly record struct Report(int Sections,int LineageSections,int Unresolved,int MissingCompositions,int ExposedTunnels,
        int CandidatePortMovements,int UnobservedPortMovements,int ProhibitedMovements,int UnverifiableRelations,
        int RequiredLaneSlots=0,int DisconnectedLaneSlots=0,int UnavailableLaneSlots=0,int RailRepresentationMismatches=0,
        int RailRepresentationsChecked=0,int UnavailableRailRepresentations=0);
    public static Report Check(EntityManager manager,IReadOnlyList<PreparedSegment> sections,OsmDataset? evidence,ImportLog log,bool includeMovements=true,
        IReadOnlyList<Entity>? scope=null,string artifactStem="native-network-readback",bool reportWarnings=true,IReadOnlyList<OsmRelation>? relationScope=null,bool finalCapture=false,bool captureRawLanes=false)
    {
        finalCapture=finalCapture&&log.DiagnosticsEnabled;
        manager.CompleteAllTrackedJobs();
        using var query=manager.CreateEntityQuery(new EntityQueryDesc{All=new[]{ComponentType.ReadOnly<Edge>(),ComponentType.ReadOnly<Curve>(),ComponentType.ReadOnly<PrefabRef>()},None=new[]{ComponentType.ReadOnly<Temp>(),ComponentType.ReadOnly<Deleted>()}});
        using var entities=scope==null?query.ToEntityArray(Allocator.Temp):new NativeArray<Entity>(scope.ToArray(),Allocator.Temp);
        var grid=new Dictionary<(int,int),List<Live>>();
        var byIndex=new Dictionary<int,Live>();var byEntity=new Dictionary<Entity,Live>();var anchors=new Dictionary<(NetworkKind,long),HashSet<Entity>>();
        (int,int) Cell(float3 p)=>((int)Math.Floor(p.x/8),(int)Math.Floor(p.z/8));
        foreach(var entity in entities)
        {
            if(!manager.HasComponent<Edge>(entity)||!manager.HasComponent<Curve>(entity)||!manager.HasComponent<PrefabRef>(entity)||manager.HasComponent<Temp>(entity)||manager.HasComponent<Deleted>(entity))continue;
            var prefab=manager.GetComponentData<PrefabRef>(entity).m_Prefab;var road=manager.HasComponent<RoadData>(prefab);if(!road&&!manager.HasComponent<TrackData>(prefab))continue;
            var edge=manager.GetComponentData<Edge>(entity);if(!manager.HasComponent<Node>(edge.m_Start)||!manager.HasComponent<Node>(edge.m_End))continue;
            var live=new Live(entity,edge.m_Start,edge.m_End,manager.GetComponentData<Node>(edge.m_Start).m_Position,manager.GetComponentData<Node>(edge.m_End).m_Position,road?NetworkKind.Road:NetworkKind.Rail,manager.GetComponentData<Curve>(entity).m_Bezier);
            byIndex[entity.Index]=live;
            byEntity[entity]=live;
            foreach(var cell in new[]{Cell(live.Start),Cell(live.End)}.Distinct()){if(!grid.TryGetValue(cell,out var list))grid[cell]=list=new();list.Add(live);}
        }
        var owners=new Dictionary<int,HashSet<long>>();var matched=0;var missing=0;var compositionMissing=0;var exposed=0;var laneMismatch=0;var lineageSections=0;var inspected=new HashSet<Entity>();
        var readbackRecords=new List<object>();var ports=new Dictionary<SourceNodeKey,List<Port>>();
        var coverage=new List<(string Identity,bool Resolved)>();
        var missingCompositionEdges=new HashSet<Entity>();
        if(scope!=null)foreach(var live in byEntity.Values)
        {
            var composition=manager.HasComponent<Composition>(live.Entity)?manager.GetComponentData<Composition>(live.Entity).m_Edge:Entity.Null;
            var hasComposition=manager.HasComponent<NetCompositionData>(composition);var data=hasComposition?manager.GetComponentData<NetCompositionData>(composition):default;
            if(!hasComposition&&missingCompositionEdges.Add(live.Entity))compositionMissing++;
            var slots=new List<object>();
            if(finalCapture&&manager.HasBuffer<NetCompositionLane>(composition))foreach(var lane in manager.GetBuffer<NetCompositionLane>(composition,true))
                slots.Add(new{index=lane.m_Index,position=DiagnosticArtifacts.Point(lane.m_Position),flags=lane.m_Flags.ToString(),prefab=lane.m_Lane.ToString()});
            readbackRecords.Add(new{outcome="scoped-live-observation-not-source-certification",entity=live.Entity.ToString(),startNode=live.A.ToString(),endNode=live.B.ToString(),kind=live.Kind.ToString(),
                frame=UnityEngine.Time.frameCount,
                prefab=manager.GetComponentData<PrefabRef>(live.Entity).m_Prefab.ToString(),compositionAvailable=hasComposition,width=data.m_Width,
                general=data.m_Flags.m_General.ToString(),left=data.m_Flags.m_Left.ToString(),right=data.m_Flags.m_Right.ToString(),physicalSlots=slots,
                curveQuality=Processing.NetworkCurveObservations.Measure(new(live.Curve.a,live.Curve.b,live.Curve.c,live.Curve.d)),
                a=DiagnosticArtifacts.Point(live.Curve.a),b=DiagnosticArtifacts.Point(live.Curve.b),c=DiagnosticArtifacts.Point(live.Curve.c),d=DiagnosticArtifacts.Point(live.Curve.d)});
        }
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
            if(!bound&&scope==null)for(var dx=-1;dx<=1;dx++)for(var dz=-1;dz<=1;dz++)if(grid.TryGetValue((cell.Item1+dx,cell.Item2+dz),out var list))foreach(var e in list)
                if(e.Kind==s.Kind&&((math.distance(e.Start,s.Start)<2&&math.distance(e.End,s.End)<2)||(math.distance(e.Start,s.End)<2&&math.distance(e.End,s.Start)<2)))candidates.Add(e);
            if(!bound&&scope==null)
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
            if(!bound&&scope!=null)valid=false; // Never certify a local unit by nearest geometry.
            if(!valid)
            {
                if(finalCapture)coverage.Add((DiagnosticArtifacts.SectionIdentity(s),false));
                missing++;if(missing<=25)log.Diagnostic($"NATIVE COVERAGE unresolved kind={s.Kind}; way={s.WayId}; nodes={s.StartNodeId},{s.EndNodeId}; sourceIds={string.Join(",",s.SourceWayIds)}; recordedLineage={bound}; matchingBothAnchorsAndInterior={!bound}; legitimateTerminal={s.StartTerminal||s.EndTerminal}; noDuplicateRetry=true");
                if(finalCapture||readbackRecords.Count<8192)readbackRecords.Add(new { outcome="unresolved",s.WayId,s.StartNodeId,s.EndNodeId,kind=s.Kind.ToString(),start=DiagnosticArtifacts.Point(s.Start),end=DiagnosticArtifacts.Point(s.End),s.SourceWayIds,candidateEdges=candidates.Count,recordedLineage=bound });
                continue;
            }
            matched++;Anchor(s.Kind,s.StartNodeId,a);Anchor(s.Kind,s.EndNodeId,b);
            if(finalCapture)coverage.Add((DiagnosticArtifacts.SectionIdentity(s),bound));
            if(finalCapture)readbackRecords.Add(new{outcome="section-lineage",s.WayId,s.StartNodeId,s.EndNodeId,s.NativeUnitId,kind=s.Kind.ToString(),
                s.SourceWayIds,recordedLineage=bound,edges=chain.Select(e=>e.Entity.ToString()).ToArray(),
                plannedA=DiagnosticArtifacts.Point(s.CourseStart),plannedB=DiagnosticArtifacts.Point(s.CourseControl1),
                plannedC=DiagnosticArtifacts.Point(s.CourseControl2),plannedD=DiagnosticArtifacts.Point(s.CourseEnd)});
            void AddPort(long node,Entity edge,Entity via,bool incoming)
            {var key=new SourceNodeKey(s.Kind,node);if(!ports.TryGetValue(key,out var list))ports[key]=list=new();list.Add(new(s,edge,via,incoming));}
            AddPort(s.StartNodeId,chain[0].Entity,a,false);AddPort(s.EndNodeId,chain[chain.Count-1].Entity,b,true);
            if(!s.OneWay){AddPort(s.StartNodeId,chain[0].Entity,a,true);AddPort(s.EndNodeId,chain[chain.Count-1].Entity,b,false);}
            foreach(var live in chain)
            {
            if(!owners.TryGetValue(live.Entity.Index,out var ways))owners[live.Entity.Index]=ways=new();ways.UnionWith(s.SourceWayIds.Count>0?s.SourceWayIds:new[]{s.WayId});
            if(!inspected.Add(live.Entity))continue;
            var flags=default(CompositionFlags);var width=0f;
            List<object>? physicalSlots=finalCapture?new():null;
            if(manager.HasComponent<Composition>(live.Entity))
            {var composition=manager.GetComponentData<Composition>(live.Entity).m_Edge;if(manager.HasComponent<NetCompositionData>(composition)){var data=manager.GetComponentData<NetCompositionData>(composition);flags=data.m_Flags;width=data.m_Width;}else if(missingCompositionEdges.Add(live.Entity))compositionMissing++;
                if(finalCapture&&manager.HasBuffer<NetCompositionLane>(composition))foreach(var lane in manager.GetBuffer<NetCompositionLane>(composition,true))
                    physicalSlots!.Add(new{index=lane.m_Index,lateral=lane.m_Position.x,flags=lane.m_Flags.ToString(),lanePrefab=lane.m_Lane.ToString()});
            }
            else if(missingCompositionEdges.Add(live.Entity))compositionMissing++;
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
            if(finalCapture&&scope!=null)readbackRecords.Add(new{outcome="matched-source-observation-reference",s.WayId,s.StartNodeId,s.EndNodeId,
                entity=live.Entity.ToString(),canonicalObservation="scoped-live-observation-not-source-certification",canonicalScope=artifactStem,
                sourceQualified=bound,requestedLanesOrTracks=requested,actualCarLanes=cars,actualTrackLanes=tracks,
                payloadRepeated=false,observationsAtDifferentFramesAreNotDeduplicated=true});
            else if(finalCapture||readbackRecords.Count<8192&&(inspected.Count<=256||s.Tunnel||count!=requested))readbackRecords.Add(new
            {
                outcome="matched",s.WayId,s.SourceWayIds,s.StartNodeId,s.EndNodeId,kind=s.Kind.ToString(),entity=live.Entity.ToString(),
                startNode=live.A.ToString(),endNode=live.B.ToString(),requestedLanesOrTracks=requested,actualCarLanes=cars,actualTrackLanes=tracks,
                laneCountSemantics="raw-SubLane-entries-not-certified-composed-lane-count",
                physicalSlots,
                nativeWidth=width,plannedWidth=s.NativeWidth,tunnelRequested=s.Tunnel,nativeGeneralFlags=flags.m_General.ToString(),nativeLeftFlags=flags.m_Left.ToString(),nativeRightFlags=flags.m_Right.ToString(),railAutomaticSidesObserved=s.Kind==NetworkKind.Rail&&(flags.m_Left!=0||flags.m_Right!=0),recordedLineage=bound,sourceChainEdges=chain.Count,
                a=DiagnosticArtifacts.Point(live.Curve.a),b=DiagnosticArtifacts.Point(live.Curve.b),c=DiagnosticArtifacts.Point(live.Curve.c),d=DiagnosticArtifacts.Point(live.Curve.d),
                renderedAcceptanceUnverified=true
            });
            if(!finalCapture&&inspected.Count<=3)log.Diagnostic($"NATIVE NETWORK EDGE entity={live.Entity}; kind={s.Kind}; way={s.WayId}; requestedLanesOrTracks={requested}; actualCarLanes={cars}; actualTrackLanes={tracks}; invertedCarLanes={reverse}; requestedOneWay={s.OneWay}; compositionWidth={width:R}; plannedWidth={s.NativeWidth:R}; tunnelRequested={s.Tunnel}; nativeGeneralFlags={flags.m_General}; nativeLeftFlags={flags.m_Left}; nativeRightFlags={flags.m_Right}; nativeSpeeds={string.Join(",",speeds.Distinct())}; renderedAcceptanceUnverified=true");
            }
        }
        if(finalCapture)log.DiagnosticCompressedRecords(artifactStem+".ndjson.gz",readbackRecords);
        else if(scope==null)log.DiagnosticRecordChunks(artifactStem+".ndjson",readbackRecords);
        else log.DiagnosticRecords(artifactStem+"-readback.ndjson",readbackRecords,1024L*1024);
        log.Diagnostic($"NATIVE READBACK ARTIFACT records={readbackRecords.Count}; finalCapture={finalCapture}; matchesSampled={!finalCapture}; unresolvedTotal={missing}; lineageQualifiedSections={lineageSections}; laneMismatchTotal={laneMismatch}; laneCountIsRawSubLaneApproximation=true; scopedReadback={scope!=null}; checkCaptureManifest=true");
        // Junction lanes expose ordered source/target edge owners. This proves
        // movement existence, separately from two edges sharing a node anchor.
        if(!includeMovements)
        {
            log.Diagnostic($"NATIVE ACCEPTANCE SAMPLE matchedSections={matched}/{sections.Count}; unresolvedSections={missing}; uniqueEdgesInspected={inspected.Count}; unavailableCompositions={compositionMissing}; tunnelFlagMismatches={exposed}; fullLaneGraphScanDeferred=true; completeConnectivityNotClaimed=true");
            return new(sections.Count,lineageSections,missing,compositionMissing,exposed,0,0,0,0);
        }
        var movements=new HashSet<(NetworkKind Kind,long From,long To,Entity Via)>();
        var edgeMovements=new HashSet<(NetworkKind Kind,Entity From,Entity To,Entity Via)>();var nodeLaneCount=0;var unlocatedLanes=0;
        var slotMovements=new HashSet<(Entity From,byte FromSlot,Entity To,byte ToSlot,Entity Via)>();
        var rawLaneRecords=new List<object>();
        var trainCurvatureChecked=0;var trainCurvatureFailed=0;var trainContractUnavailable=0;var worstTrainCurvature=0d;
        using var lanesQuery=manager.CreateEntityQuery(new EntityQueryDesc{All=new[]{ComponentType.ReadOnly<Lane>()},Any=new[]{ComponentType.ReadOnly<CarLane>(),ComponentType.ReadOnly<TrackLane>()},None=new[]{ComponentType.ReadOnly<Deleted>(),ComponentType.ReadOnly<Temp>()}});
        // A settled batch inspects the lane buffers on its actual incident
        // nodes/edges. Do not scan the entire city's lane graph for each batch.
        Entity[] ScopedLanes()
        {
            var ownersToRead=new HashSet<Entity>();foreach(var live in byEntity.Values){ownersToRead.Add(live.Entity);ownersToRead.Add(live.A);ownersToRead.Add(live.B);}
            var found=new HashSet<Entity>();foreach(var owner in ownersToRead)if(manager.HasBuffer<SubLane>(owner))
                foreach(var lane in manager.GetBuffer<SubLane>(owner,true))found.Add(lane.m_SubLane);
            return found.OrderBy(e=>e.Index).ThenBy(e=>e.Version).ToArray();
        }
        using var lanes=scope==null?lanesQuery.ToEntityArray(Allocator.Temp):new NativeArray<Entity>(ScopedLanes(),Allocator.Temp);
        foreach(var entity in lanes)
        {
            if(!manager.HasComponent<Lane>(entity)||manager.HasComponent<Deleted>(entity)||manager.HasComponent<Temp>(entity)||
                !manager.HasComponent<CarLane>(entity)&&!manager.HasComponent<TrackLane>(entity))continue;
            var lane=manager.GetComponentData<Lane>(entity);var kind=manager.HasComponent<CarLane>(entity)?NetworkKind.Road:NetworkKind.Rail;
            if(finalCapture&&captureRawLanes)
            {
                var available=manager.HasComponent<Curve>(entity);var curve=available?manager.GetComponentData<Curve>(entity).m_Bezier:default;
                rawLaneRecords.Add(new{outcome="native-lane-observation-not-source-certification",entity=entity.ToString(),kind=kind.ToString(),
                    frame=UnityEngine.Time.frameCount,
                    startOwnerIndex=lane.m_StartNode.GetOwnerIndex(),endOwnerIndex=lane.m_EndNode.GetOwnerIndex(),
                    startLaneIndex=lane.m_StartNode.GetLaneIndex(),endLaneIndex=lane.m_EndNode.GetLaneIndex(),
                    owner=manager.HasComponent<Owner>(entity)?manager.GetComponentData<Owner>(entity).m_Owner.ToString():null,
                    flags=kind==NetworkKind.Road?manager.GetComponentData<CarLane>(entity).m_Flags.ToString():manager.GetComponentData<TrackLane>(entity).m_Flags.ToString(),
                    curveAvailable=available,a=available?DiagnosticArtifacts.Point(curve.a):null,b=available?DiagnosticArtifacts.Point(curve.b):null,
                    c=available?DiagnosticArtifacts.Point(curve.c):null,d=available?DiagnosticArtifacts.Point(curve.d):null});
            }
            var invert=false;var twoway=false;
            if(kind==NetworkKind.Road)
            {var car=manager.GetComponentData<CarLane>(entity);if((car.m_Flags&(CarLaneFlags.Forbidden|CarLaneFlags.ParkingLeft|CarLaneFlags.ParkingRight|CarLaneFlags.SideConnection))!=0)continue;invert=(car.m_Flags&CarLaneFlags.Invert)!=0;twoway=(car.m_Flags&CarLaneFlags.Twoway)!=0;}
            else
            {
                var track=manager.GetComponentData<TrackLane>(entity);invert=(track.m_Flags&TrackLaneFlags.Invert)!=0;twoway=(track.m_Flags&TrackLaneFlags.Twoway)!=0;
                var prefab=manager.HasComponent<PrefabRef>(entity)?manager.GetComponentData<PrefabRef>(entity).m_Prefab:Entity.Null;
                if(!manager.HasComponent<TrackLaneData>(prefab))trainContractUnavailable++;
                else
                {
                    var contract=manager.GetComponentData<TrackLaneData>(prefab);
                    if((contract.m_TrackTypes&TrackTypes.Train)==0||!math.isfinite(contract.m_MaxCurviness)||contract.m_MaxCurviness<=0)trainContractUnavailable++;
                    else
                    {
                        trainCurvatureChecked++;worstTrainCurvature=Math.Max(worstTrainCurvature,track.m_Curviness);
                        if(!math.isfinite(track.m_Curviness)||track.m_Curviness>contract.m_MaxCurviness)trainCurvatureFailed++;
                    }
                }
            }
            var a=lane.m_StartNode.GetOwnerIndex();var b=lane.m_EndNode.GetOwnerIndex();
            if(!owners.TryGetValue(a,out var from)||!owners.TryGetValue(b,out var to)||!byIndex.TryGetValue(a,out var edgeA)||!byIndex.TryGetValue(b,out var edgeB)||edgeA.Kind!=kind||edgeB.Kind!=kind)continue;
            var via=Entity.Null;
            if(manager.HasComponent<Owner>(entity))
            {var owner=manager.GetComponentData<Owner>(entity).m_Owner;if(manager.HasComponent<Node>(owner))via=owner;}
            if(via==Entity.Null&&a!=b)
            {if(edgeA.A==edgeB.A||edgeA.A==edgeB.B)via=edgeA.A;else if(edgeA.B==edgeB.A||edgeA.B==edgeB.B)via=edgeA.B;}
            if(via==Entity.Null){if(a!=b)unlocatedLanes++;continue;}
            nodeLaneCount++;
            edgeMovements.Add(invert?(kind,edgeB.Entity,edgeA.Entity,via):(kind,edgeA.Entity,edgeB.Entity,via));
            var startSlot=(byte)(lane.m_StartNode.GetLaneIndex()&255);var endSlot=(byte)(lane.m_EndNode.GetLaneIndex()&255);
            slotMovements.Add(invert?(edgeB.Entity,endSlot,edgeA.Entity,startSlot,via):(edgeA.Entity,startSlot,edgeB.Entity,endSlot,via));
            if(twoway)edgeMovements.Add(invert?(kind,edgeA.Entity,edgeB.Entity,via):(kind,edgeB.Entity,edgeA.Entity,via));
            if(twoway)slotMovements.Add(invert?(edgeA.Entity,startSlot,edgeB.Entity,endSlot,via):(edgeB.Entity,endSlot,edgeA.Entity,startSlot,via));
            foreach(var f in from)foreach(var t in to)
            {movements.Add(invert?(kind,t,f,via):(kind,f,t,via));if(twoway)movements.Add(invert?(kind,f,t,via):(kind,t,f,via));}
        }
        var requiredPorts=0;var missingPorts=0;var portRecords=new List<object>();
        var slotPlan=NativeLaneObligations.Build(sections,relationScope??evidence?.Relations);var disconnectedSlots=0;var unavailableSlots=0;var railRepresentationMismatches=0;
        var railChecked=0;var railUnavailable=0;
        Port? Find(SourceLanePort expected,bool incoming)
        {
            var s=sections[expected.Section];var node=new SourceNodeKey(s.Kind,expected.Start?s.StartNodeId:s.EndNodeId);
            return ports.TryGetValue(node,out var candidates)?candidates.FirstOrDefault(p=>ReferenceEquals(p.Section,s)&&p.Incoming==incoming):null;
        }
        byte? Slot(Port p,int rank)
        {
            if(!byEntity.TryGetValue(p.Edge,out var live)||!manager.HasComponent<Composition>(p.Edge))return null;
            var composition=manager.GetComponentData<Composition>(p.Edge).m_Edge;if(!manager.HasBuffer<NetCompositionLane>(composition))return null;
            var forward=p.Incoming?live.B==p.Via:live.A==p.Via;
            var selected=new List<(byte Index,float Position)>();
            foreach(var lane in manager.GetBuffer<NetCompositionLane>(composition,true))
            {
                if((lane.m_Flags&(LaneFlags.Master|LaneFlags.Virtual|LaneFlags.Parking))!=0)continue;
                if(p.Section.Kind==NetworkKind.Road&&!manager.HasComponent<CarLaneData>(lane.m_Lane))continue;
                if(p.Section.Kind==NetworkKind.Rail&&(!manager.HasComponent<TrackLaneData>(lane.m_Lane)||
                    (manager.GetComponentData<TrackLaneData>(lane.m_Lane).m_TrackTypes&TrackTypes.Train)==0))continue;
                var reverse=(lane.m_Flags&LaneFlags.Invert)!=0;var two=(lane.m_Flags&LaneFlags.Twoway)!=0;
                if(!two&&reverse==forward)continue;
                selected.Add((lane.m_Index,forward?lane.m_Position.x:-lane.m_Position.x));
            }
            selected.Sort((a,b)=>a.Position.CompareTo(b.Position));
            var expected=p.Section.Kind==NetworkKind.Rail?1:p.Section.NativeLaneOffsets.Count;
            return selected.Count==expected&&rank<selected.Count?selected[rank].Index:null;
        }
        foreach(var obligation in slotPlan.Movements)
        {
            var a=Find(obligation.From,true);var b=Find(obligation.To,false);byte? ia=a==null?null:Slot(a,obligation.From.Rank);byte? ib=b==null?null:Slot(b,obligation.To.Rank);
            var available=a!=null&&b!=null&&ia.HasValue&&ib.HasValue&&a.Via==b.Via&&manager.HasBuffer<SubLane>(a.Via)&&manager.GetBuffer<SubLane>(a.Via,true).Length>0;
            var connected=available&&(a!.Edge==b!.Edge&&ia==ib||slotMovements.Contains((a!.Edge,ia!.Value,b!.Edge,ib!.Value,a.Via)));
            if(!available)unavailableSlots++;else if(!connected)disconnectedSlots++;
            if(log.DiagnosticsEnabled&&(finalCapture||portRecords.Count<8192))portRecords.Add(new{outcome="required-physical-lane-slot",sourceNode=obligation.Node.Id,
                fromWay=sections[obligation.From.Section].WayId,fromRank=obligation.From.Rank,toWay=sections[obligation.To.Section].WayId,toRank=obligation.To.Rank,
                nativeFromSlot=ia,nativeToSlot=ib,available,connected,obligation.PlannedLateralError,obligation.SourceRelation,sourceSlotFrameIsImporterPolicy=!obligation.SourceRelation.HasValue,nativePathLaneIndexLowByte=true});
        }
        foreach(var live in byEntity.Values.Where(e=>e.Kind==NetworkKind.Rail))
        {
            if(!manager.HasComponent<Composition>(live.Entity)){railUnavailable++;continue;}var composition=manager.GetComponentData<Composition>(live.Entity).m_Edge;
            if(!manager.HasBuffer<NetCompositionLane>(composition)){railUnavailable++;continue;}railChecked++;
            var trackSlots=new List<NetCompositionLane>();
            foreach(var lane in manager.GetBuffer<NetCompositionLane>(composition,true))if((lane.m_Flags&(LaneFlags.Master|LaneFlags.Virtual))==0&&manager.HasComponent<TrackLaneData>(lane.m_Lane)&&
                (manager.GetComponentData<TrackLaneData>(lane.m_Lane).m_TrackTypes&TrackTypes.Train)!=0)trackSlots.Add(lane);
            if(trackSlots.Count!=2||trackSlots.Any(t=>(t.m_Flags&LaneFlags.Twoway)!=0)||trackSlots.Count(t=>(t.m_Flags&LaneFlags.Invert)!=0)!=1)railRepresentationMismatches++;
        }
        foreach(var node in ports)
            foreach(var incoming in node.Value.Where(p=>p.Incoming))foreach(var outgoing in node.Value.Where(p=>!p.Incoming))
            {
                if(ReferenceEquals(incoming.Section,outgoing.Section)||incoming.Via!=outgoing.Via)continue;
                // This observes directional edge-to-edge connectivity, not a
                // proof that each requested physical lane maps to its exact slot.
                requiredPorts++;
                var present=incoming.Edge==outgoing.Edge||edgeMovements.Contains((node.Key.Kind,incoming.Edge,outgoing.Edge,incoming.Via));
                if(!present)missingPorts++;
                if(log.DiagnosticsEnabled&&(finalCapture||portRecords.Count<8192))portRecords.Add(new{outcome="directed-source-port-movement",kind=node.Key.Kind.ToString(),sourceNode=node.Key.Id,
                    fromWay=incoming.Section.WayId,toWay=outgoing.Section.WayId,fromNativeEdge=incoming.Edge.ToString(),toNativeEdge=outgoing.Edge.ToString(),via=incoming.Via.ToString(),present,
                    restrictionsAssessedSeparately=true,perLaneSlotAcceptance=false});
            }
        var restrictionChecks=0;var restrictionConflicts=0;var connectivityMissing=0;var unverifiable=0;var representedWays=new HashSet<long>(owners.Values.SelectMany(w=>w));
        var movementsByFrom=movements.GroupBy(m=>(m.Kind,m.From,m.Via)).ToDictionary(g=>g.Key,g=>g.Select(m=>m.To).ToHashSet());
        if(evidence!=null)foreach(var relation in relationScope??evidence.Relations)
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
        if(finalCapture)log.DiagnosticCompressedRecords(artifactStem+"-port-movements.ndjson.gz",portRecords.Concat(rawLaneRecords));
        else if(log.DiagnosticsEnabled)log.DiagnosticRecords(artifactStem+"-port-movements.ndjson",portRecords,1024L*1024);
        if(finalCapture)log.ObserveNativeCapture(artifactStem,coverage);
        log.Diagnostic($"NATIVE ACCEPTANCE matchedSections={matched}/{sections.Count}; unresolvedSections={missing}; uniqueEdgesInspected={inspected.Count}; unavailableCompositions={compositionMissing}; tunnelFlagMismatches={exposed}; laneCountApproximations={laneMismatch}; orderedRoadAndRailNodeLanes={nodeLaneCount}; unlocatedNodeLanes={unlocatedLanes}; relationsChecked={restrictionChecks}; relationsUnverifiable={unverifiable}; prohibitedMovementsObserved={restrictionConflicts}; requiredMovementsNotObserved={connectivityMissing}; directedPortPairs={requiredPorts}; unobservedPortPairs={missingPorts}; portPairsMayIncludeSourceRestrictedTurns=true; perLaneSlotAcceptance=false; readOnly=true; definitionCountNotAcceptance=true");
        if(reportWarnings&&missing+exposed+restrictionConflicts+connectivityMissing>0)log.Warn($"Native acceptance needs review: {missing} unresolved sections, {exposed} tunnel-composition mismatches, {restrictionConflicts} prohibited and {connectivityMissing} unobserved required movements. Import remains available for iteration; no unsafe duplicate edges or transient lane edits were created.");
        log.Diagnostic($"NATIVE SLOT ACCEPTANCE required={slotPlan.Movements.Count}; disconnected={disconnectedSlots}; unavailable={unavailableSlots}; nodesWithoutCompleteSourceSlotSemantics={slotPlan.UnavailableNodes}; railRepresentationMismatches={railRepresentationMismatches}; railRepresentationsChecked={railChecked}; unavailableRailRepresentations={railUnavailable}; physicalTwoTrackOppositeDirections=true; generalTurnPoliciesNotInvented=true; missingPublicationIsNotDisconnected=true; placementBlocked=false");
        if(finalCapture)
        {
            var stage=artifactStem.StartsWith("native-owned-final-",StringComparison.Ordinal)?"native-census":"native-unit";
            void Quality(string category,int count,int failed,int unknown,double worst,string evidence)
                =>log.Quality(new(stage,category,artifactStem,count,failed,unknown,worst,0,evidence));
            Quality("source-lineage",sections.Count,0,sections.Count-lineageSections,0,"Exact path/owner source qualification; unavailable is not a disconnected observation");
            Quality("composition",sections.Count,exposed,compositionMissing,exposed,"Actual native tunnel/composition readback; source counts are not acceptance");
            Quality("lane-slot-connectivity",slotPlan.Movements.Count,disconnectedSlots,unavailableSlots+slotPlan.UnavailableNodes,0,"Actual ordered lane indices and node-owned SubLane publication; closure observation scopes can overlap");
            Quality("source-turn-restrictions",restrictionChecks,restrictionConflicts+connectivityMissing,unverifiable,0,"Explicit supported relation semantics; conditional/via-way/excepted relations unavailable");
            Quality("rail-representation",railChecked,railRepresentationMismatches,railUnavailable,0,"Two train lanes in opposite directions on one retained axis");
            Quality("actual-train-curvature",trainCurvatureChecked,trainCurvatureFailed,trainContractUnavailable,worstTrainCurvature,
                "Actual TrackLane.m_Curviness versus its actual train TrackLaneData.m_MaxCurviness; includes edge and node-owned lanes");
            var joins=0;var headingFailures=0;var joinUnknown=0;var worstHeading=0d;var gradeJoinFailures=0;var worstGradeJump=0d;
            var junctions=sections.SelectMany(s=>new[]{(Section:s,Start:true),(Section:s,Start:false)})
                .GroupBy(p=>new SourceNodeKey(p.Section.Kind,p.Start?p.Section.StartNodeId:p.Section.EndNodeId));
            foreach(var node in junctions)
            {
                var pair=node.ToArray();if(pair.Length!=2||pair.Any(p=>p.Section.Roundabout))continue;
                double3 Planned((PreparedSegment Section,bool Start) p)=>p.Start?(double3)p.Section.CourseControl1-p.Section.CourseStart:(double3)p.Section.CourseControl2-p.Section.CourseEnd;
                var expectedA=Planned(pair[0]);var expectedB=Planned(pair[1]);
                if(math.lengthsq(expectedA.xz)<=0||math.lengthsq(expectedB.xz)<=0||math.dot(math.normalizesafe(expectedA.xz),math.normalizesafe(expectedB.xz))>-.9)continue;
                joins++;
                double3? Actual((PreparedSegment Section,bool Start) p)
                {
                    var s=p.Section;
                    if(!NativeNetworkDiagnostics.VerifiedPaths.ContainsKey((s.Kind,s.WayId,s.StartNodeId,s.EndNodeId))||!ports.TryGetValue(node.Key,out var candidates))return null;
                    var port=candidates.FirstOrDefault(port=>ReferenceEquals(port.Section,s));
                    if(port==null||!byEntity.TryGetValue(port.Edge,out var live))return null;
                    return live.A==port.Via?(double3)live.Curve.b-live.Curve.a:live.B==port.Via?(double3)live.Curve.c-live.Curve.d:null;
                }
                var actualA=Actual(pair[0]);var actualB=Actual(pair[1]);
                var betaA=NetworkQualityAudit.DirectionRoundingBudget(pair[0].Section,pair[0].Start);
                var betaB=NetworkQualityAudit.DirectionRoundingBudget(pair[1].Section,pair[1].Start);
                if(!actualA.HasValue||!actualB.HasValue||!betaA.HasValue||!betaB.HasValue||betaA+betaB>Math.PI/180*.1)
                {joinUnknown++;continue;}
                var da=actualA.Value;var db=actualB.Value;var la=math.length(da.xz);var lb=math.length(db.xz);
                if(!(la>0&&lb>0)){joinUnknown++;continue;}
                var angle=Math.Atan2(Math.Abs(NetworkDesignChecks.Cross(da.xz,-db.xz)),math.dot(da.xz,-db.xz));
                var gradeJump=Math.Abs(da.y/la+db.y/lb);worstHeading=Math.Max(worstHeading,angle);worstGradeJump=Math.Max(worstGradeJump,gradeJump);
                if(angle>Math.PI/180*.1)headingFailures++;
                if(gradeJump>1e-5)gradeJoinFailures++;
            }
            Quality("continuation-heading",joins,headingFailures,joinUnknown,worstHeading,
                "Source-qualified actual edge-end tangents; native 0.1-degree allowance only with representable planned rounding budget; missing lineage/reference unavailable");
            Quality("continuation-grade",joins,gradeJoinFailures,joinUnknown,worstGradeJump,
                "Source-qualified actual edge-end outward physical-grade sum; through continuations only");
            var gradeFailures=0;var worstGrade=0d;var affectedLength=0d;
            var finiteFailures=0;var regularityUnknown=0;var maximumCurvature=0d;
            foreach(var live in byEntity.Values)
            {
                var curve=new Cubic3d(live.Curve.a,live.Curve.b,live.Curve.c,live.Curve.d);
                var observation=NetworkCurveObservations.Measure(curve);
                if(!observation.Finite)finiteFailures++;
                if(!observation.Finite||NetworkDesignChecks.Certify(curve,double.PositiveInfinity,double.PositiveInfinity,maximumLeaves:4096).Quality!=DesignQuality.Certified)regularityUnknown++;
                maximumCurvature=Math.Max(maximumCurvature,observation.SampledMaximumCurvature);
                worstGrade=Math.Max(worstGrade,observation.SampledMaximumGrade);
                var ceiling=live.Kind==NetworkKind.Rail?.08:NetworkVerticalProfiles.AbsoluteRoadGrade;
                if(observation.SampledMaximumGrade>ceiling){gradeFailures++;affectedLength+=observation.SampledLengthMeters;}
            }
            log.Quality(new(stage,"observed-grade",artifactStem,byEntity.Count,gradeFailures,byEntity.Count-gradeFailures,worstGrade,affectedLength,
                "65-point observations; exceeding the broadest policy is proven failure; a sampled nonfailure does not certify source-specific limits"));
            Quality("finite-native-geometry",byEntity.Count,finiteFailures,0,finiteFailures,"Exact final native curve controls");
            Quality("regularity",byEntity.Count,0,regularityUnknown,0,"Continuous native horizontal derivative certificate; exhausted certificate unavailable");
            Quality("observed-curvature",byEntity.Count,0,byEntity.Count,maximumCurvature,"Sampled centreline curvature only; actual train-lane curvature has its own asset-specific acceptance category");
        }
        return new(sections.Count,lineageSections,missing,compositionMissing,exposed,requiredPorts,missingPorts,restrictionConflicts,unverifiable,slotPlan.Movements.Count,disconnectedSlots,unavailableSlots,railRepresentationMismatches,railChecked,railUnavailable);
        void Anchor(NetworkKind kind,long id,Entity entity)
        {
            if(!anchors.TryGetValue((kind,id),out var values))anchors[(kind,id)]=values=new();values.Add(entity);
            foreach(var section in sections.Where(s=>s.Kind==kind&&(s.StartNodeId==id||s.EndNodeId==id)))
                foreach(var alias in section.SourceNodeAliases.Where(a=>a.CanonicalNodeId==id))
                {var key=(kind,alias.SourceNodeId);if(!anchors.TryGetValue(key,out var mapped))anchors[key]=mapped=new();mapped.Add(entity);}
        }
    }
    private static float Distance(Colossal.Mathematics.Bezier4x3 c,float3 point)
    {var distance=float.PositiveInfinity;var previous=c.a;for(var i=1;i<=32;i++){var t=i/32f;var u=1-t;var p=u*u*u*c.a+3*u*u*t*c.b+3*u*t*t*c.c+t*t*t*c.d;var delta=p-previous;var ratio=math.clamp(math.dot(point-previous,delta)/Math.Max(.001f,math.lengthsq(delta)),0,1);distance=Math.Min(distance,math.distance(point,previous+delta*ratio));previous=p;}return distance;}
}
