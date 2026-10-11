using System;
using System.Collections.Generic;
using System.Diagnostics;
using Colossal.Mathematics;
using Game.Common;
using Game.Net;
using Game.Prefabs;
using Game.Tools;
using ImportEarthLocation.Core;
using ImportEarthLocation.Integration;
using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;

namespace ImportEarthLocation.Systems;

public sealed class NativeNetworkPlacement
{
    private readonly EntityManager _entityManager;
    private readonly PrefabResolver _prefabs;
    private readonly ImportLog _log;
    public int RoadSections { get; private set; }
    public int RailSections { get; private set; }
    public int VerifiedSections => _verifiedPaths.Count;
    private readonly HashSet<string> _isolatedEndpoints = new(StringComparer.Ordinal);
    private Dictionary<string,float> _nodeElevations = new(StringComparer.Ordinal);
    private readonly Dictionary<string, Entity> _committedNodes = new(StringComparer.Ordinal);
    private readonly Dictionary<string,NativeBatchBindings.Anchor<Entity>> _committedAnchors = new(StringComparer.Ordinal);
    private readonly Dictionary<string,float3> _plannedPositions = new(StringComparer.Ordinal);
    private readonly Dictionary<string,float> _centerBudgets = new(StringComparer.Ordinal);
    private readonly List<NativeBatchBindings.Expected<Entity>> _submittedCourses = new();
    private Dictionary<Entity,NativeBatchBindings.Edge<Entity>> _verifiedEdges = new();
    private Dictionary<Entity,NativeBatchBindings.Edge<Entity>>? _stagedEdges;
    private IReadOnlyList<NativeBatchBindings.Edge<Entity>> _touchedEdges=Array.Empty<NativeBatchBindings.Edge<Entity>>();
    private readonly Dictionary<(NetworkKind Kind,long WayId,long Start,long End),List<Entity>> _verifiedPaths=new();
    private Dictionary<Entity,Entity> _stagedEdgeChanges=new();
    private readonly Dictionary<(NetworkKind Kind,long WayId,long Start,long End),List<Entity>> _stagedPaths=new();
    public string ArtifactStem { get; private set; } = "native-preflight";
    private NativeBatchBindings.Batch<Entity>? _generatedBatch;
    private Dictionary<string,Entity>? _stagedCommitted;
    private IReadOnlyList<PreparedSegment>? _source;
    private readonly Dictionary<(NetworkKind Kind,long WayId,long Start,long End),PreparedSegment> _sourceRoutes=new();
    // A shared source node at a bridge/tunnel portal is a real connection. Grade-separated
    // crossings with different source node IDs remain isolated, even at coincident positions.
    private static string Identity(PreparedSegment segment, long node) => $"{segment.Kind}:{node}";

    public void BeginBatch(int startIndex,int attempt,int count,bool preflight=false)
    {
        ArtifactStem=$"native-{(preflight?"preflight":"batch")}-{startIndex:D6}-{count:D3}-attempt-{attempt:D2}";
        _log.Diagnostic($"NATIVE COURSE POLICY artifact={ArtifactStem}; disableOverlapMergeBothEnds=true; subElevationStructuresOnly=true; committedEntityAttachmentRetained=true; preparedSourceTopologyAuthoritative=true; proximityAttachmentsAllowed=false");
        _generatedBatch=null;_stagedCommitted=null;_stagedEdges=null;_touchedEdges=Array.Empty<NativeBatchBindings.Edge<Entity>>();_stagedEdgeChanges.Clear();
        _stagedPaths.Clear();
    }

    public NativeNetworkPlacement(EntityManager entityManager, PrefabSystem prefabSystem, ImportLog log)
    {
        _entityManager = entityManager;
        _prefabs = new PrefabResolver(prefabSystem, log);
        _log = log;
    }

    public static IReadOnlyList<PreparedSegment> Preflight(IReadOnlyList<PreparedSegment> segments, PrefabSystem prefabs, ImportLog log, out bool needsRepair)
    {
        var resolver=new PrefabResolver(prefabs,log);var supported=new List<PreparedSegment>(segments.Count);var rejected=0;needsRepair=false;
        foreach(var s in segments)
        {
            try
            {
                var actual=resolver.ResolveMetadata(s);
                var changed=!math.isfinite(s.NativeWidth)||s.NativeWidth<=0||Math.Abs(actual.NativeWidth-s.NativeWidth)>.01f||!System.Linq.Enumerable.SequenceEqual(actual.NativeLaneOffsets,s.NativeLaneOffsets);
                if(changed)
                {
                    needsRepair=true;log.Count("Native footprint sections reconciled");
                    if(log.Metric("Native footprint sections reconciled")<=25)
                        log.Warn($"Native {s.Kind} way {s.WayId}: prepared width={s.NativeWidth:0.##} m; selected width={actual.NativeWidth:0.##} m; lanes={s.Lanes}. Retained for complete-network repair/re-audit before confirmation.");
                }
                supported.Add(actual);
            }
            catch(Exception error) when(error is InvalidOperationException||error is ArgumentException||error is KeyNotFoundException)
            {
                rejected++;log.Count(s.Kind+" unsupported native direction/width");
                if(rejected<=25)log.Warn($"Native metadata unavailable for {s.Kind} way {s.WayId}: {error.Message}");
            }
        }
        if(rejected>0)throw new InvalidOperationException($"Native network compatibility failed for {rejected}/{segments.Count} sections. Preparation stopped before replacing the map; no disconnected road or rail subset will be applied.");
        return supported;
    }

    public IReadOnlyList<Entity> CreateDefinitions(IReadOnlyList<PreparedSegment> segments, out int skipped,Processing.OsmDataset? evidence=null)
    {
        _entityManager.CompleteAllTrackedJobs();
        if (_source == null) Initialize(segments,evidence);
        var definitions = new List<Entity>(segments.Count);
        _submittedCourses.Clear();
        skipped = 0;
        var roads = Stopwatch.StartNew(); var roadTime = 0d; var railTime = 0d;
        PreparedSegment? active=null;
        try
        {
            foreach (var segment in segments)
            {
                active=segment;
                roads.Restart();
                try
                {
                    var prefab = _prefabs.Resolve(segment);
                    if (prefab == Entity.Null) throw new InvalidOperationException($"Native prefab unavailable for {segment.Kind} way {segment.WayId}.");
                    definitions.Add(CreateDefinition(segment, prefab));
                    _log.Count(segment.Kind + " definitions queued");
                }
                finally { if (segment.Kind == NetworkKind.Road) roadTime += roads.Elapsed.TotalSeconds; else railTime += roads.Elapsed.TotalSeconds; }
            }
        }
        catch(Exception error)
        {
            foreach(var definition in definitions)_entityManager.DestroyEntity(definition);
            if(error is OutOfMemoryException||error is OperationCanceledException)throw;
            throw new InvalidOperationException($"Native definition failed for {active?.Kind} way {active?.WayId}; pending batch rolled back. {error.Message}",error);
        }
        if (roadTime > 0) _log.Time("Road apply", roadTime);
        if (railTime > 0) _log.Time("Rail apply", railTime);
        _log.DiagnosticRecords(ArtifactStem+"-expected.ndjson",ExpectedRecords());
        CapturePlacementEvidence(definitions,"submitted");
        return definitions;
    }

    // Read-only evidence: never use nearby nodes to choose an attachment.
    private void CapturePlacementEvidence(IReadOnlyList<Entity> definitions,string phase)
    {
        try
        {
            var records=new List<object>();
            string Id(Entity e)=>$"{e.Index}:{e.Version}";
            foreach(var definition in definitions)
            {
                var creation=_entityManager.GetComponentData<CreationDefinition>(definition);
                var course=_entityManager.GetComponentData<NetCourse>(definition);
                object Pos(CoursePos p)=>new{entity=Id(p.m_Entity),position=DiagnosticArtifacts.Point(p.m_Position),
                    rotation=new{x=p.m_Rotation.value.x,y=p.m_Rotation.value.y,z=p.m_Rotation.value.z,w=p.m_Rotation.value.w},
                    elevation=new{x=p.m_Elevation.x,y=p.m_Elevation.y},p.m_CourseDelta,p.m_SplitPosition,flags=p.m_Flags.ToString(),p.m_ParentMesh};
                records.Add(new{kind="definition",entity=Id(definition),
                    creation=new{prefab=Id(creation.m_Prefab),original=Id(creation.m_Original),owner=Id(creation.m_Owner),attached=Id(creation.m_Attached),flags=creation.m_Flags.ToString(),creation.m_RandomSeed},
                    course=new{a=DiagnosticArtifacts.Point(course.m_Curve.a),b=DiagnosticArtifacts.Point(course.m_Curve.b),c=DiagnosticArtifacts.Point(course.m_Curve.c),d=DiagnosticArtifacts.Point(course.m_Curve.d),
                        start=Pos(course.m_StartPosition),end=Pos(course.m_EndPosition),elevation=new{x=course.m_Elevation.x,y=course.m_Elevation.y},course.m_Length,course.m_FixedIndex},frame=UnityEngine.Time.frameCount});
            }
            using var query=_entityManager.CreateEntityQuery(ComponentType.ReadOnly<Node>());
            using var nodes=query.ToEntityArray(Allocator.Temp);
            var spatial=new Dictionary<(int X,int Z),List<Entity>>();
            foreach(var node in nodes)
            {
                var p=_entityManager.GetComponentData<Node>(node).m_Position;
                if(!math.all(math.isfinite(p)))continue;
                var cell=((int)math.floor(p.x/64),(int)math.floor(p.z/64));
                if(!spatial.TryGetValue(cell,out var bucket))spatial[cell]=bucket=new List<Entity>();
                bucket.Add(node);
            }
            foreach(var expected in _submittedCourses)
            foreach(var endpoint in new[]{(expected.Section.StartNodeId,expected.Section.Start),(expected.Section.EndNodeId,expected.Section.End)})
            {
                var source=Identity(expected.Section,endpoint.Item1);
                _committedNodes.TryGetValue(source,out var intended);
                var candidates=new List<(Entity Entity,float Distance)>();
                var nearby=new HashSet<Entity>();
                var center=((int)math.floor(endpoint.Item2.x/64),(int)math.floor(endpoint.Item2.z/64));
                for(var x=-1;x<=1;x++)for(var z=-1;z<=1;z++)
                    if(spatial.TryGetValue((center.Item1+x,center.Item2+z),out var bucket))foreach(var node in bucket)nearby.Add(node);
                if(intended!=Entity.Null&&_entityManager.Exists(intended)&&_entityManager.HasComponent<Node>(intended))nearby.Add(intended);
                foreach(var node in nearby)
                {
                    var position=_entityManager.GetComponentData<Node>(node).m_Position;
                    var distance=math.distance(position.xz,endpoint.Item2.xz);
                    if(distance<=64||node==intended)candidates.Add((node,distance));
                }
                candidates.Sort((a,b)=>a.Distance.CompareTo(b.Distance));
                var recorded=0;
                foreach(var candidate in candidates)
                {
                    if(recorded>=64&&candidate.Entity!=intended)continue;
                    var node=candidate.Entity;var incidents=new List<object>();
                    if(_entityManager.HasBuffer<ConnectedEdge>(node))
                    foreach(var link in _entityManager.GetBuffer<ConnectedEdge>(node,true))
                    {
                        if(incidents.Count>=128)break;
                        var edge=link.m_Edge;var exists=_entityManager.Exists(edge);
                        incidents.Add(new{entity=Id(edge),exists,
                            edge=exists&&_entityManager.HasComponent<Edge>(edge)?new{start=Id(_entityManager.GetComponentData<Edge>(edge).m_Start),end=Id(_entityManager.GetComponentData<Edge>(edge).m_End)}:null,
                            curve=exists&&_entityManager.HasComponent<Curve>(edge)?CurveEvidence(_entityManager.GetComponentData<Curve>(edge).m_Bezier):null,
                            prefab=exists&&_entityManager.HasComponent<PrefabRef>(edge)?Id(_entityManager.GetComponentData<PrefabRef>(edge).m_Prefab):null});
                    }
                    records.Add(new{kind="endpoint-neighborhood",source,expected.Section.WayId,intended=Id(intended),entity=Id(node),candidate.Distance,
                        position=DiagnosticArtifacts.Point(_entityManager.GetComponentData<Node>(node).m_Position),deleted=_entityManager.HasComponent<Deleted>(node),
                        temp=_entityManager.HasComponent<Temp>(node)?new{original=Id(_entityManager.GetComponentData<Temp>(node).m_Original),flags=_entityManager.GetComponentData<Temp>(node).m_Flags.ToString()}:null,
                        incidents,candidatesFound=candidates.Count,truncated=candidates.Count>64,searchRadiusMeters=64,attachmentByProximity=false});
                    recorded++;
                }
            }
            _log.DiagnosticRecords(ArtifactStem+"-"+phase+"-placement-evidence.ndjson",records,32L*1024*1024);
        }
        catch(Exception error) when(error is not OutOfMemoryException)
        { _log.Diagnostic($"NATIVE DIAGNOSTIC capture incomplete artifact={ArtifactStem}; phase={phase}; exception={error}"); }
    }

    public void Initialize(IReadOnlyList<PreparedSegment> source, Processing.OsmDataset? evidence)
    {
        _source = source;
        _sourceRoutes.Clear();
        foreach(var section in source)
        {
            var key=(section.Kind,section.WayId,section.StartNodeId,section.EndNodeId);
            if(!_sourceRoutes.ContainsKey(key))_sourceRoutes[key]=section;
            else _log.Count("Duplicate native source route keys observed");
        }
        RoadSections=RailSections=0;
        _stagedPaths.Clear();
        _nodeElevations=NativeEndpointPolicy.Elevations(source);
        _isolatedEndpoints.Clear(); _committedNodes.Clear(); _committedAnchors.Clear();_plannedPositions.Clear();_centerBudgets.Clear(); _verifiedEdges.Clear();_verifiedPaths.Clear();_stagedEdgeChanges.Clear();_stagedEdges=null;_touchedEdges=Array.Empty<NativeBatchBindings.Edge<Entity>>();_generatedBatch=null;_stagedCommitted=null;
        NativeNetworkDiagnostics.Remember(source,evidence);
        var positions = new Dictionary<float3, HashSet<string>>();
        foreach (var segment in source)
        foreach (var endpoint in new[] { (segment.Start, segment.StartNodeId), (segment.End, segment.EndNodeId) })
        {
            if (!positions.TryGetValue(endpoint.Item1, out var identities)) positions[endpoint.Item1] = identities = new HashSet<string>();
            identities.Add(Identity(segment, endpoint.Item2));
            _plannedPositions[Identity(segment,endpoint.Item2)]=endpoint.Item1;
            var key=Identity(segment,endpoint.Item2);var budget=NativeBatchBindings.CenterBudget(segment);
            _centerBudgets[key]=_centerBudgets.TryGetValue(key,out var oldBudget)?Math.Max(oldBudget,budget):budget;
        }
        foreach (var identities in positions.Values) if (identities.Count > 1) foreach (var identity in identities) _isolatedEndpoints.Add(identity);
    }

    // Capture source-to-entity bindings while the submitted graph still has Temp
    // lineage. ApplyNet.Create retains its entity; Update redirects to Original.
    public void CaptureGeneratedNodes(IReadOnlyList<PreparedSegment> batch,IReadOnlyList<Entity> temporary)
    {
        _entityManager.CompleteAllTrackedJobs();
        var actual=new List<NativeBatchBindings.Edge<Entity>>();
        Entity Target(Entity entity)
        {
            if(!_entityManager.HasComponent<Temp>(entity))return entity;
            var temp=_entityManager.GetComponentData<Temp>(entity);
            // Installed ApplyNet: nodes use Replace; edges use Replace|Combine.
            var replace=(temp.m_Flags&TempFlags.Replace)!=0 || (_entityManager.HasComponent<Edge>(entity)&&(temp.m_Flags&TempFlags.Combine)!=0);
            return NativeBatchBindings.Target(entity,temp.m_Original,temp.m_Original!=Entity.Null,replace);
        }
        var previewPositions=new Dictionary<Entity,float3>();
        var replacements=new Dictionary<Entity,Entity>();
        foreach(var entity in temporary)
        {
            if(!_entityManager.HasComponent<Node>(entity))continue;
            if((_entityManager.GetComponentData<Temp>(entity).m_Flags&TempFlags.Delete)!=0)continue;
            var target=Target(entity);var point=_entityManager.GetComponentData<Node>(entity).m_Position;
            if(previewPositions.TryGetValue(target,out var old)&&!math.all(old==point))
                throw new NativePreviewException("Multiple native node updates disagree on their committed anchor.",0);
            previewPositions[target]=point;
            var original=_entityManager.GetComponentData<Temp>(entity).m_Original;
            if(original!=Entity.Null&&target!=original)
            {
                if(replacements.TryGetValue(original,out var other)&&other!=target)
                    throw new NativePreviewException("Multiple native replacements disagree on one source node.",0);
                replacements[original]=target;
            }
        }
        float3 Position(Entity node)=>previewPositions.TryGetValue(Target(node),out var point)?point:_entityManager.GetComponentData<Node>(node).m_Position;
        foreach(var entity in temporary)
        {
            if(!_entityManager.HasComponent<Edge>(entity)||!_entityManager.HasComponent<PrefabRef>(entity))continue;
            var temp=_entityManager.GetComponentData<Temp>(entity);
            if((temp.m_Flags&TempFlags.Delete)!=0)continue;
            var edge=_entityManager.GetComponentData<Edge>(entity);var prefab=_entityManager.GetComponentData<PrefabRef>(entity).m_Prefab;
            var road=_entityManager.HasComponent<RoadData>(prefab);if(!road&&!_entityManager.HasComponent<TrackData>(prefab))continue;
            if(!_entityManager.HasComponent<Curve>(entity))throw new InvalidOperationException("Native generated edge has no curve evidence; unsafe commit refused.");
            var curve=_entityManager.GetComponentData<Curve>(entity).m_Bezier;
            actual.Add(new NativeBatchBindings.Edge<Entity>(Target(entity),Target(edge.m_Start),Target(edge.m_End),
                Position(edge.m_Start),Position(edge.m_End),road?NetworkKind.Road:NetworkKind.Rail,prefab){CurveStart=curve.a,CurveEnd=curve.d,CurveControl1=curve.b,CurveControl2=curve.c});
        }
        if(_submittedCourses.Count!=batch.Count)throw new InvalidOperationException("Submitted native course manifest is incomplete.");
        _stagedCommitted=NativeBatchBindings.RemapCommitted(_committedNodes,replacements);
        // Reconcile only an already-owned node, never a nearby candidate. Native
        // generation may adjust that node again while extending its road.
        for(var i=0;i<_submittedCourses.Count;i++)
        {
            var item=_submittedCourses[i];var section=item.Section;
            NativeBatchBindings.Anchor<Entity>? PreviewAnchor(long id)
            {
                var key=Identity(section,id);
                if(!_stagedCommitted.TryGetValue(key,out var owner)||!previewPositions.TryGetValue(owner,out var position))return null;
                if(!math.all(math.isfinite(position))||math.distance(position.xz,_plannedPositions[key].xz)>_centerBudgets[key])
                    throw new NativePreviewException("Native owned junction moved beyond its original source footprint.",section.WayId,key);
                return new NativeBatchBindings.Anchor<Entity>(owner,_plannedPositions[key],position){CenterLimit=_centerBudgets[key]};
            }
            var start=PreviewAnchor(section.StartNodeId);var end=PreviewAnchor(section.EndNodeId);
            NativeBatchBindings.RebaseOwnedPreview(section,start,end);
            var movement=Math.Max(start==null?0:math.distance(section.Start.xz,start.Generated.xz),end==null?0:math.distance(section.End.xz,end.Generated.xz));
            if(movement>0)
                _log.Diagnostic($"NATIVE OWNED MOVEMENT way={section.WayId}; centerMovementMeters={movement:R}; submittedCoursePreserved=true; artifact={ArtifactStem}; sourceFootprintChecked=true; unrelatedAttachmentAllowed=false");
        }
        var relevant=new HashSet<Entity>();foreach(var edge in actual){relevant.Add(edge.StartNode);relevant.Add(edge.EndNode);}
        foreach(var section in batch)foreach(var id in new[]{section.StartNodeId,section.EndNodeId})
            if(_stagedCommitted.TryGetValue(Identity(section,id),out var known))relevant.Add(known);
        _log.DiagnosticRecords(ArtifactStem+"-owners.ndjson",OwnerRecords(relevant));
        _log.DiagnosticRecords(ArtifactStem+"-graph.ndjson",GraphRecords(actual));
        try{_generatedBatch=NativeBatchBindings.Match(_submittedCourses,actual,_stagedCommitted,MatchRejection);}
        catch(Exception error)
        {
            CapturePlacementEvidence(Array.Empty<Entity>(),"rejected");
            _log.DiagnosticRecords(ArtifactStem+"-rejection-endpoints.ndjson",RejectionEndpoints(actual));
            _log.Checkpoint("native-batch/preview-rejected",new{artifactStem=ArtifactStem,sections=batch.Count,committedOwners=_committedNodes.Count,
                exception=error.ToString(),recoverable=error is NativePreviewException,applied=false,
                way=(error as NativePreviewException)?.WayId,sources=(error as NativePreviewException)?.Sources,
                endpointEvidence=ArtifactStem+"-rejection-endpoints.ndjson",subElevationStructuresOnly=true});
            throw;
        }
        var anchors=new Dictionary<string,NativeBatchBindings.Anchor<Entity>>(StringComparer.Ordinal);
        foreach(var pair in _generatedBatch.Anchors)anchors[pair.Key]=pair.Value with{Planned=_plannedPositions[pair.Key],CenterLimit=_centerBudgets[pair.Key]};
        var required=new Dictionary<Entity,float3>();foreach(var pair in _generatedBatch.Nodes)required[pair.Key]=pair.Value;
        var stagedEdges=new Dictionary<Entity,NativeBatchBindings.Edge<Entity>>(_verifiedEdges);
        var touched=new List<NativeBatchBindings.Edge<Entity>>();
        Entity RemapNode(Entity node)=>replacements.TryGetValue(node,out var next)?next:node;
        var updatedOriginals=new HashSet<Entity>();
        var actualByEntity=new Dictionary<Entity,NativeBatchBindings.Edge<Entity>>();
        foreach(var e in actual)
        {
            if(actualByEntity.TryGetValue(e.Entity,out var previous)&&previous!=e)
                throw new NativePreviewException("Native preview updates disagree on one edge target; regeneration required.",0);
            actualByEntity[e.Entity]=e;
        }
        foreach(var entity in temporary)
        {
            var temp=_entityManager.GetComponentData<Temp>(entity);
            if(_entityManager.HasComponent<Node>(entity)&&(temp.m_Flags&TempFlags.Delete)!=0)
            {
                var original=temp.m_Original!=Entity.Null?temp.m_Original:entity;
                foreach(var owner in _committedNodes)
                    if(owner.Value==original&&!replacements.ContainsKey(original))throw new NativePreviewException($"Preview would delete previously verified junction {owner.Key} without replacement lineage.",0,owner.Key);
            }
            if(!_entityManager.HasComponent<Edge>(entity)||!_verifiedEdges.TryGetValue(temp.m_Original,out var prior))continue;
            if((temp.m_Flags&TempFlags.Delete)!=0)continue;
            if(!actualByEntity.TryGetValue(Target(entity),out var update)||update.Prefab!=prior.Prefab||update.Kind!=prior.Kind)
                throw PriorEdgeRejection(prior,$"Preview changed previously verified edge {temp.m_Original} asset or junction incidence; regeneration required.");
            updatedOriginals.Add(temp.m_Original);stagedEdges.Remove(temp.m_Original);stagedEdges[update.Entity]=update;touched.Add(update);
            if(temp.m_Original!=update.Entity)_stagedEdgeChanges[temp.m_Original]=update.Entity;
            required[update.StartNode]=update.Start;required[update.EndNode]=update.End;
        }
        foreach(var entity in temporary)
        {
            if(!_entityManager.HasComponent<Edge>(entity))continue;
            var temp=_entityManager.GetComponentData<Temp>(entity);
            if((temp.m_Flags&TempFlags.Delete)!=0&&_verifiedEdges.ContainsKey(temp.m_Original)&&!updatedOriginals.Contains(temp.m_Original))
                stagedEdges.Remove(temp.m_Original);
        }
        // Complete overlay: Combine may create a replacement edge without a
        // one-to-one original, including subdivisions not consumed by this batch.
        // Omitting those edges manufactures missing prior routes.
        foreach(var edge in actual)
        {
            stagedEdges[edge.Entity]=edge;
            required[edge.StartNode]=edge.Start;required[edge.EndNode]=edge.End;
            if(!System.Linq.Enumerable.Any(touched,e=>e.Entity==edge.Entity))touched.Add(edge);
        }
        var routeOwners=new Dictionary<string,Entity>(_stagedCommitted,StringComparer.Ordinal);
        foreach(var anchor in _generatedBatch.Anchors)routeOwners[anchor.Key]=anchor.Value.Entity;
        foreach(var key in new List<Entity>(stagedEdges.Keys))
        {
            var edge=stagedEdges[key];var start=RemapNode(edge.StartNode);var end=RemapNode(edge.EndNode);
            stagedEdges[key]=edge with{StartNode=start,EndNode=end,
                Start=previewPositions.TryGetValue(start,out var a)?a:edge.Start,
                End=previewPositions.TryGetValue(end,out var b)?b:edge.End};
        }
        // Prove whole source-route coverage for EVERY changed/deleted original.
        // Native subdivision boundaries may disappear; source junctions may not.
        var changedOriginals=new HashSet<Entity>();
        foreach(var entity in temporary)
            if(_entityManager.HasComponent<Edge>(entity))
            {var original=_entityManager.GetComponentData<Temp>(entity).m_Original;if(_verifiedEdges.ContainsKey(original))changedOriginals.Add(original);}
        var changedNodes=new HashSet<Entity>(replacements.Keys);
        foreach(var edge in _verifiedEdges.Values)
        {
            var start=RemapNode(edge.StartNode);var end=RemapNode(edge.EndNode);
            if(previewPositions.TryGetValue(start,out var a)&&!math.all(a==edge.Start))changedNodes.Add(edge.StartNode);
            if(previewPositions.TryGetValue(end,out var b)&&!math.all(b==edge.End))changedNodes.Add(edge.EndNode);
        }
        foreach(var path in _verifiedPaths)
        {
            if(!System.Linq.Enumerable.Any(path.Value,e=>changedOriginals.Contains(e)||
                _verifiedEdges.TryGetValue(e,out var previous)&&(changedNodes.Contains(previous.StartNode)||changedNodes.Contains(previous.EndNode))))continue;
            if(!_sourceRoutes.TryGetValue(path.Key,out var source))throw PriorEdgeRejection(_verifiedEdges[path.Value[0]],"Prior source route manifest unavailable; combination refused.");
            NativeBatchBindings.Anchor<Entity> Owned(long id,float3 planned)
            {
                var key=Identity(source,id);var owner=_stagedCommitted[key];
                var position=previewPositions.TryGetValue(owner,out var p)?p:_committedAnchors[key].Generated;
                if(math.distance(position.xz,_plannedPositions[key].xz)>_centerBudgets[key])throw new NativePreviewException("Combined route junction exceeded original footprint.",source.WayId,key);
                return new(owner,planned,position){CenterLimit=_centerBudgets[key]};
            }
            // Validate source-owner center movement independently of ports. A
            // junction update may change its native curve while preserving its
            // owned, directed route. Never translate the old prepared evidence.
            Owned(source.StartNodeId,source.Start);Owned(source.EndNodeId,source.End);
            var reconciled=source;
            var tolerance=Math.Max(_centerBudgets[Identity(source,source.StartNodeId)],_centerBudgets[Identity(source,source.EndNodeId)]);
            var prefab=_verifiedEdges[path.Value[0]].Prefab;
            var low=math.min(math.min(reconciled.CourseStart.xz,reconciled.CourseEnd.xz),math.min(reconciled.CourseControl1.xz,reconciled.CourseControl2.xz))-new float2(tolerance+32);
            var high=math.max(math.max(reconciled.CourseStart.xz,reconciled.CourseEnd.xz),math.max(reconciled.CourseControl1.xz,reconciled.CourseControl2.xz))+new float2(tolerance+32);
            var candidates=new List<NativeBatchBindings.Edge<Entity>>();
            foreach(var edge in stagedEdges.Values)
                if(edge.Kind==source.Kind&&edge.Prefab==prefab&&math.all(edge.Start.xz>=low)&&math.all(edge.Start.xz<=high)&&math.all(edge.End.xz>=low)&&math.all(edge.End.xz<=high))candidates.Add(edge);
            NativeBatchBindings.Batch<Entity> route;
            try
            {
                route=NativeBatchBindings.Match(new[]{new NativeBatchBindings.Expected<Entity>(reconciled,prefab){StartCenterBudget=_centerBudgets[Identity(source,source.StartNodeId)],EndCenterBudget=_centerBudgets[Identity(source,source.EndNodeId)],ReconciliationTolerance=tolerance,ValidateCurveCorridor=true,OwnedRoute=true}},candidates,routeOwners,MatchRejection);
            }
            catch(NativePreviewException)
            {
                // Complete candidate controls are needed on failure, not tens
                // of thousands of graph files during successful imports.
                _log.DiagnosticRecords(ArtifactStem+"-prior-route-"+source.WayId+"-"+source.StartNodeId+"-"+source.EndNodeId+".ndjson",GraphRecords(candidates));
                throw;
            }
            _stagedPaths[path.Key]=new List<Entity>(System.Linq.Enumerable.Select(route.Edges,b=>b.Generated.Entity));
            foreach(var pair in route.Nodes)required[pair.Key]=pair.Value;
            foreach(var binding in route.Edges)if(!System.Linq.Enumerable.Any(touched,e=>e.Entity==binding.Generated.Entity))touched.Add(binding.Generated);
            var first=route.Edges[0];var last=route.Edges[route.Edges.Count-1];
            var firstPort=first.Forward?first.Generated.CurveStart??first.Generated.Start:first.Generated.CurveEnd??first.Generated.End;
            var lastPort=last.Forward?last.Generated.CurveEnd??last.Generated.End:last.Generated.CurveStart??last.Generated.Start;
            var portMovement=Math.Max(math.distance(firstPort.xz,source.CourseStart.xz),math.distance(lastPort.xz,source.CourseEnd.xz));
            var heightMovement=Math.Max(Math.Abs(firstPort.y-source.CourseStart.y),Math.Abs(lastPort.y-source.CourseEnd.y));
            _log.Count("Native owned routes requalified");
            _log.Diagnostic($"NATIVE ROUTE REQUALIFIED way={source.WayId}; previousEdges={path.Value.Count}; replacementEdges={route.Edges.Count}; sourceOwnersRetained=true; exactAsset=true; uniqueConnectedCoverage=true; ownedRoute=true; geometricEnvelopeMeters={tolerance:R}; endpointPortMovementMeters={portMovement:R}; endpointHeightMovementMeters={heightMovement:R}; structuralTerrainClearanceCertified=false; immutableSourceEvidence=true; artifact={ArtifactStem}");
        }
        // Retire anonymous subdivision nodes eliminated by an approved Combine.
        var retainedNodes=new HashSet<Entity>();foreach(var e in stagedEdges.Values){retainedNodes.Add(e.StartNode);retainedNodes.Add(e.EndNode);}
        foreach(var node in new List<Entity>(required.Keys))if(!retainedNodes.Contains(node)&&!System.Linq.Enumerable.Contains(_stagedCommitted.Values,node))required.Remove(node);
        _stagedEdges=stagedEdges;_touchedEdges=touched;
        // A preview can regenerate nodes from earlier verified batches. Retain
        // their center evidence too, not just anchors present in this batch.
        foreach(var pair in _stagedCommitted)
        {
            if(!previewPositions.TryGetValue(pair.Value,out var point))continue;
            var prior=_committedAnchors[pair.Key];
            if(!math.all(math.isfinite(point))||math.distance(point.xz,prior.Planned.xz)>_centerBudgets[pair.Key])
                throw new NativePreviewException($"Previously bound junction {pair.Key} moved outside bounded native footprint.",0,pair.Key);
            required[pair.Value]=point;
        }
        _generatedBatch=_generatedBatch with{Anchors=anchors,Nodes=required};
        var changed=0;var maximum=0f;
        foreach(var p in _generatedBatch.Anchors.Values){var delta=Math.Abs(p.Generated.y-p.Planned.y);if(delta>.001f)changed++;maximum=Math.Max(maximum,delta);}
        _log.Count("Native ground anchor height adjustments",changed);
        _log.Diagnostic($"NATIVE BATCH artifact={ArtifactStem}; generatedBindings={_generatedBatch.Anchors.Count}; sourceSectionsMatched={_generatedBatch.SectionCount}/{batch.Count}; nativeEdgesMatched={_generatedBatch.Edges.Count}; nativeSubdivisionNodes={_generatedBatch.Nodes.Count-_generatedBatch.Anchors.Count}; heightAdjusted={changed}; maximumHeightDeltaMeters={maximum:R}; exactCurvePortsAndAsset=true; entityLineage=true; nativeCoverageStillPending=true");
        _log.DiagnosticRecords("native-batch-bindings.ndjson",BindingRecords(_generatedBatch));
        _log.DiagnosticRecords(ArtifactStem+"-bindings.ndjson",BindingRecords(_generatedBatch));
    }

    private IEnumerable<object> OwnerRecords(HashSet<Entity> relevant)
    {
        foreach(var pair in _committedAnchors)
        {
            var entity=_stagedCommitted![pair.Key];if(!relevant.Contains(entity))continue;
            yield return new{source=pair.Key,entity=$"{entity.Index}:{entity.Version}",previousEntity=$"{pair.Value.Entity.Index}:{pair.Value.Entity.Version}",planned=DiagnosticArtifacts.Point(pair.Value.Planned),generated=DiagnosticArtifacts.Point(pair.Value.Generated),pair.Value.CenterLimit};
        }
    }
    private IEnumerable<object> ExpectedRecords()
    {
        foreach(var item in _submittedCourses)
            yield return new{kind="expected-course",prefab=$"{item.Prefab.Index}:{item.Prefab.Version}",item.StartCenterBudget,item.EndCenterBudget,disableOverlapMergeBothEnds=true,
                creationFlags=(NetworkStructure.IsStructure(item.Section)?CreationFlags.SubElevation:0).ToString(),section=System.Linq.Enumerable.First(DiagnosticArtifacts.NetworkRecords(new[]{item.Section}))};
    }
    private IEnumerable<object> RejectionEndpoints(IReadOnlyList<NativeBatchBindings.Edge<Entity>> graph)
    {
        foreach(var item in _submittedCourses)
        foreach(var endpoint in new[]{(id:item.Section.StartNodeId,port:item.Section.CourseStart,center:item.Section.Start,budget:item.StartCenterBudget),
                                      (id:item.Section.EndNodeId,port:item.Section.CourseEnd,center:item.Section.End,budget:item.EndCenterBudget)})
        {
            var source=Identity(item.Section,endpoint.id);
            var known=_stagedCommitted!.TryGetValue(source,out var owner);
            var candidates=new List<object>();
            foreach(var edge in graph)
            {
                if(edge.Kind!=item.Section.Kind||edge.Prefab!=item.Prefab)continue;
                foreach(var end in new[]{(node:edge.StartNode,center:edge.Start,port:edge.CurveStart??edge.Start),(node:edge.EndNode,center:edge.End,port:edge.CurveEnd??edge.End)})
                {
                    // Spatial distance is diagnostic ONLY, never permission to join.
                    var delta=math.distance(end.port.xz,endpoint.port.xz);
                    if((known&&end.node==owner)||delta<=endpoint.budget)
                        candidates.Add(new{node=$"{end.node.Index}:{end.node.Version}",edge=$"{edge.Entity.Index}:{edge.Entity.Version}",ownerMatches=known&&end.node==owner,
                            curvePortDelta=delta,centerDelta=math.distance(end.center.xz,endpoint.center.xz),centerToPortDelta=math.distance(end.center.xz,end.port.xz),
                            center=DiagnosticArtifacts.Point(end.center),port=DiagnosticArtifacts.Point(end.port)});
                    if(candidates.Count>=24)break;
                }
                if(candidates.Count>=24)break;
            }
            yield return new{source,item.Section.WayId,knownOwner=known?$"{owner.Index}:{owner.Version}":null,submittedPort=DiagnosticArtifacts.Point(endpoint.port),
                submittedCenter=DiagnosticArtifacts.Point(endpoint.center),endpoint.budget,candidates,truncated=candidates.Count>=24,proximityAttachmentAllowed=false};
        }
    }
    private static IEnumerable<object> GraphRecords(IReadOnlyList<NativeBatchBindings.Edge<Entity>> graph)
    {
        foreach(var e in graph)yield return new{entity=$"{e.Entity.Index}:{e.Entity.Version}",startNode=$"{e.StartNode.Index}:{e.StartNode.Version}",endNode=$"{e.EndNode.Index}:{e.EndNode.Version}",
            start=DiagnosticArtifacts.Point(e.Start),end=DiagnosticArtifacts.Point(e.End),mode=e.Kind.ToString(),prefab=$"{e.Prefab.Index}:{e.Prefab.Version}",
            curveStart=e.CurveStart.HasValue?DiagnosticArtifacts.Point(e.CurveStart.Value):null,curveEnd=e.CurveEnd.HasValue?DiagnosticArtifacts.Point(e.CurveEnd.Value):null,
            control1=e.CurveControl1.HasValue?DiagnosticArtifacts.Point(e.CurveControl1.Value):null,control2=e.CurveControl2.HasValue?DiagnosticArtifacts.Point(e.CurveControl2.Value):null};
    }

    private static object CurveEvidence(Bezier4x3 c)=>new{a=DiagnosticArtifacts.Point(c.a),b=DiagnosticArtifacts.Point(c.b),c=DiagnosticArtifacts.Point(c.c),d=DiagnosticArtifacts.Point(c.d)};
    private void MatchRejection(object evidence)
    {
        _log.Count("Native matcher rejected attempts",1);
        _log.DiagnosticRecords(ArtifactStem+"-matcher-rejection.ndjson",new[]{evidence});
    }

    private static IEnumerable<object> BindingRecords(NativeBatchBindings.Batch<Entity> batch)
    {
        string Id(Entity e)=>$"{e.Index}:{e.Version}";
        foreach(var pair in batch.Anchors)yield return new { kind="source-anchor",source=pair.Key,entity=Id(pair.Value.Entity),planned=DiagnosticArtifacts.Point(pair.Value.Planned),generated=DiagnosticArtifacts.Point(pair.Value.Generated),heightDelta=pair.Value.Generated.y-pair.Value.Planned.y,horizontalCenterDelta=math.distance(pair.Value.Generated.xz,pair.Value.Planned.xz),pair.Value.CenterLimit };
        foreach(var edge in batch.Edges)yield return new { kind="native-chain-edge",edge.Section.WayId,edge.Section.StartNodeId,edge.Section.EndNodeId,mode=edge.Section.Kind.ToString(),entity=Id(edge.Generated.Entity),start=Id(edge.Generated.StartNode),end=Id(edge.Generated.EndNode),edge.Forward,prefab=Id(edge.Generated.Prefab) };
    }

    public bool TryCaptureCommittedNodes(out string pending)
    {
        _entityManager.CompleteAllTrackedJobs();
        if(_generatedBatch==null)throw new InvalidOperationException("Native source bindings were not captured before Apply.");
        foreach(var pair in _stagedCommitted!)
        {
            if(_committedNodes[pair.Key]==pair.Value)continue;
            if(!_entityManager.HasComponent<Node>(pair.Value)||_entityManager.HasComponent<Temp>(pair.Value)||_entityManager.HasComponent<Deleted>(pair.Value)||!_entityManager.HasBuffer<ConnectedEdge>(pair.Value))
            {pending=$"source={pair.Key}; replacement node not ready";return false;}
        }
        var changedCenters=0;
        var centerChanges=new Dictionary<Entity,(float3 Preview,float3 Live)>();
        foreach(var pair in _generatedBatch.Nodes)
        {
            var entity=pair.Key;
            if(!_entityManager.HasComponent<Node>(entity)||_entityManager.HasComponent<Temp>(entity)||_entityManager.HasComponent<Deleted>(entity)||!_entityManager.HasBuffer<ConnectedEdge>(entity))
            {pending=$"entity={entity}; source/subdivision node/buffer not ready";return false;}
            var point=_entityManager.GetComponentData<Node>(entity).m_Position;
            if(!math.all(point.xz==pair.Value.xz))
            {
                _log.Diagnostic($"NATIVE LIVE CENTER entity={entity}; preview={pair.Value}; live={point}; horizontalDeltaMeters={math.distance(point.xz,pair.Value.xz):R}; numericPlaybackOnly={NativeBatchBindings.PlaybackHorizontal(pair.Value,point)}; ownershipAndIncidenceValidationStillPending=true");
                changedCenters++;
            }
            if(!NativeBatchBindings.PlaybackHorizontal(pair.Value,point))
            {
                centerChanges[entity]=(pair.Value,point);
            }
        }
        foreach(var binding in _generatedBatch.Edges)
        {
            var e=binding.Generated;
            if(!_entityManager.HasComponent<Edge>(e.Entity)||_entityManager.HasComponent<Temp>(e.Entity)||_entityManager.HasComponent<Deleted>(e.Entity))
            {pending=$"way={binding.Section.WayId}; edge={e.Entity}; live edge not ready";return false;}
            if(!_entityManager.HasComponent<PrefabRef>(e.Entity))
            {pending=$"way={binding.Section.WayId}; edge={e.Entity}; live asset not ready";return false;}
            if(_entityManager.GetComponentData<PrefabRef>(e.Entity).m_Prefab!=e.Prefab)
                throw new InvalidOperationException($"Native live asset changed for way {binding.Section.WayId}; unsafe continuation refused.");
            var live=_entityManager.GetComponentData<Edge>(e.Entity);
            if(live.m_Start!=e.StartNode||live.m_End!=e.EndNode)throw new InvalidOperationException($"Native edge lineage changed for way {binding.Section.WayId}; unsafe continuation refused.");
            bool Incident(Entity node){foreach(var c in _entityManager.GetBuffer<ConnectedEdge>(node,true))if(c.m_Edge==e.Entity)return true;return false;}
            if(!Incident(e.StartNode)||!Incident(e.EndNode)){pending=$"way={binding.Section.WayId}; edge={e.Entity}; incident buffers not ready";return false;}
        }
        foreach(var e in _touchedEdges)
        {
            if(!_entityManager.HasComponent<Edge>(e.Entity)||_entityManager.HasComponent<Temp>(e.Entity)||_entityManager.HasComponent<Deleted>(e.Entity)||!_entityManager.HasComponent<PrefabRef>(e.Entity))
            {pending=$"previous edge={e.Entity}; regenerated live edge not ready";return false;}
            var live=_entityManager.GetComponentData<Edge>(e.Entity);
            if(live.m_Start!=e.StartNode||live.m_End!=e.EndNode||_entityManager.GetComponentData<PrefabRef>(e.Entity).m_Prefab!=e.Prefab)
                throw new InvalidOperationException("Previously verified native edge changed lineage or asset during playback.");
            bool Incident(Entity node){foreach(var c in _entityManager.GetBuffer<ConnectedEdge>(node,true))if(c.m_Edge==e.Entity)return true;return false;}
            if(!Incident(e.StartNode)||!Incident(e.EndNode)){pending=$"previous edge={e.Entity}; regenerated incident buffers not ready";return false;}
        }
        foreach(var changed in centerChanges)
        {
            if(!RequalifySourceCenter(changed.Key,changed.Value.Live,out var reason))
            {
                // Buffer/component publication may trail Apply playback. Keep
                // waiting through the existing bounded live-readback phase;
                // never replay Apply to repair an incomplete snapshot.
                if(reason=="missing-live-incidence"||reason=="incident-edge-not-live")
                {pending=$"source center={changed.Key}; {reason}";return false;}
                _log.DiagnosticRecords(ArtifactStem+"-failed-live.ndjson",FailedLiveRecords(changed.Key,changed.Value.Preview,changed.Value.Live));
                throw new InvalidOperationException($"Native live center {changed.Key} could not be requalified: {reason}; actual/preview evidence saved.");
            }
            _log.Diagnostic($"NATIVE LIVE CENTER entity={changed.Key}; decision=source-center-requalified; reason={reason}; actualCenterPublishedAtomically=true; anonymousSubdivisionAllowanceUnchanged=true; noApplyRetry=true");
        }
        // Publish all anchors together, only after every required live edge is present.
        foreach(var pair in _generatedBatch.Anchors)
        {
            var point=_entityManager.GetComponentData<Node>(pair.Value.Entity).m_Position;
            if(!NativeBatchBindings.PlaybackHorizontal(pair.Value.Generated,point)&&!centerChanges.ContainsKey(pair.Value.Entity))throw new InvalidOperationException($"Native source anchor {pair.Key} no longer matches its verified preview center or became non-finite.");
        }
        if(changedCenters>0)
        {
            foreach(var e in System.Linq.Enumerable.Concat(System.Linq.Enumerable.Select(_generatedBatch.Edges,b=>b.Generated),_touchedEdges))
            {
                if(!_entityManager.HasComponent<Curve>(e.Entity))throw new InvalidOperationException("Live roundoff validation lacks native curve evidence.");
                var curve=_entityManager.GetComponentData<Curve>(e.Entity).m_Bezier;
                if(!e.CurveStart.HasValue||!e.CurveEnd.HasValue||!NativeBatchBindings.PlaybackHorizontal(e.CurveStart.Value,curve.a)||!NativeBatchBindings.PlaybackHorizontal(e.CurveEnd.Value,curve.d))
                    throw new InvalidOperationException("Native live curve ports changed beyond numeric playback roundoff; unsafe attachment refused.");
            }
            _log.Diagnostic($"NATIVE PLAYBACK numericRoundedCenters={changedCenters-centerChanges.Count}; sourceRequalifiedCenters={centerChanges.Count}; curvePortAllowance=2-float-ULP-capped-.01m; sourceCenterAllowance=established-preview-footprint; liveCurvePortsVerified=true; assetsAndIncidenceVerified=true; proximityAttachment=false");
        }
        _verifiedEdges=_stagedEdges!;
        foreach(var path in _stagedPaths)_verifiedPaths[path.Key]=path.Value;
        if(_stagedEdgeChanges.Count>0)foreach(var path in _verifiedPaths.Values)
            for(var i=0;i<path.Count;i++)if(_stagedEdgeChanges.TryGetValue(path[i],out var replacement))path[i]=replacement;
        foreach(var group in System.Linq.Enumerable.GroupBy(_generatedBatch.Edges,b=>(b.Section.Kind,b.Section.WayId,b.Section.StartNodeId,b.Section.EndNodeId)))
            _verifiedPaths[group.Key]=new List<Entity>(System.Linq.Enumerable.Select(group,b=>b.Generated.Entity));
        NativeNetworkDiagnostics.BindVerified(_committedNodes,_verifiedPaths);
        foreach(var pair in _stagedCommitted!)
        {
            _committedNodes[pair.Key]=pair.Value;
            var prior=_committedAnchors[pair.Key];
            _committedAnchors[pair.Key]=prior with{Entity=pair.Value,Generated=_entityManager.GetComponentData<Node>(pair.Value).m_Position};
        }
        foreach(var pair in _generatedBatch.Anchors)
        {
            _committedNodes[pair.Key]=pair.Value.Entity;
            _committedAnchors[pair.Key]=pair.Value with{Generated=_entityManager.GetComponentData<Node>(pair.Value.Entity).m_Position};
        }
        foreach(var section in System.Linq.Enumerable.Distinct(System.Linq.Enumerable.Select(_generatedBatch.Edges,b=>b.Section)))
            if(section.Kind==NetworkKind.Road)RoadSections++;else RailSections++;
        _log.Count("Native source sections verified",_generatedBatch.SectionCount);
        _log.Count("Native chain edges verified",_generatedBatch.Edges.Count);
        _log.DiagnosticRecords("native-live-batch-bindings.ndjson",LiveBindingRecords(_generatedBatch));
        _log.DiagnosticRecords(ArtifactStem+"-live.ndjson",LiveBindingRecords(_generatedBatch));
        _log.Diagnostic($"NATIVE BATCH artifact={ArtifactStem}; anchorsVerified={_generatedBatch.Anchors.Count}; sourceSectionsVerified={_generatedBatch.SectionCount}; nativeEdgesVerified={_generatedBatch.Edges.Count}; accumulatedSourceNodes={_committedNodes.Count}; numericPreviewCenterAndExactAsset=true; entityLineage=true; incidentBuffersReady=true");
        _generatedBatch=null;_stagedCommitted=null;_stagedEdges=null;_touchedEdges=Array.Empty<NativeBatchBindings.Edge<Entity>>();pending="";return true;
    }

    private NativePreviewException PriorEdgeRejection(NativeBatchBindings.Edge<Entity> edge,string message)
    {
        var sources=new List<string>();
        foreach(var pair in _committedNodes)if(pair.Value==edge.StartNode||pair.Value==edge.EndNode)sources.Add(pair.Key);
        long way=0;foreach(var pair in _verifiedPaths)if(pair.Value.Contains(edge.Entity)){way=pair.Key.WayId;break;}
        foreach(var path in _verifiedPaths)
            if(path.Value.Contains(edge.Entity))
            {
                var start=$"{path.Key.Kind}:{path.Key.Start}";var end=$"{path.Key.Kind}:{path.Key.End}";
                if(!sources.Contains(start))sources.Add(start);if(!sources.Contains(end))sources.Add(end);
            }
        _log.Diagnostic($"NATIVE PRIOR EDGE rejected={edge.Entity}; way={way}; sources={string.Join(",",sources)}; expectedStart={edge.StartNode}; expectedEnd={edge.EndNode}; expectedPrefab={edge.Prefab}; pendingJunctionRecoveryHasContext={sources.Count>0}");
        return new NativePreviewException(message,way,sources.ToArray());
    }

    private bool RequalifySourceCenter(Entity entity,float3 live,out string reason)
    {
        reason="no-unique-source-owner";
        if(!math.all(math.isfinite(live))) {reason="non-finite-center";return false;}
        var owners=new Dictionary<string,NativeBatchBindings.Anchor<Entity>>(StringComparer.Ordinal);
        foreach(var pair in _stagedCommitted!)if(pair.Value==entity)owners[pair.Key]=_committedAnchors[pair.Key];
        foreach(var pair in _generatedBatch!.Anchors)if(pair.Value.Entity==entity)owners[pair.Key]=pair.Value;
        if(owners.Count!=1)return false;
        NativeBatchBindings.Anchor<Entity>? anchor=null;string source="";
        foreach(var pair in owners){anchor=pair.Value;source=pair.Key;}
        if(anchor==null||!math.isfinite(anchor.CenterLimit)||anchor.CenterLimit<=0||anchor.CenterLimit>32||math.distance(live.xz,anchor.Planned.xz)>anchor.CenterLimit)
        {reason="outside-established-source-footprint";return false;}
        var expected=new Dictionary<Entity,NativeBatchBindings.Edge<Entity>>();
        foreach(var pair in _stagedEdges!)if(pair.Value.StartNode==entity||pair.Value.EndNode==entity)expected[pair.Key]=pair.Value;
        if(expected.Count==0){reason="no-qualified-incident-edges";return false;}
        var observed=new HashSet<Entity>();
        foreach(var link in _entityManager.GetBuffer<ConnectedEdge>(entity,true))
        {
            if(!observed.Add(link.m_Edge)||!expected.TryGetValue(link.m_Edge,out var e)) {reason="extra-or-duplicate-live-incidence";return false;}
            if(!_entityManager.HasComponent<Edge>(e.Entity)||_entityManager.HasComponent<Temp>(e.Entity)||_entityManager.HasComponent<Deleted>(e.Entity)||
                !_entityManager.HasComponent<PrefabRef>(e.Entity)||!_entityManager.HasComponent<Curve>(e.Entity)) {reason="incident-edge-not-live";return false;}
            var edge=_entityManager.GetComponentData<Edge>(e.Entity);var curve=_entityManager.GetComponentData<Curve>(e.Entity).m_Bezier;
            if(edge.m_Start!=e.StartNode||edge.m_End!=e.EndNode||_entityManager.GetComponentData<PrefabRef>(e.Entity).m_Prefab!=e.Prefab)
            {reason="incident-lineage-or-asset-changed";return false;}
            if(!e.CurveStart.HasValue||!e.CurveEnd.HasValue||!NativeBatchBindings.PlaybackHorizontal(e.CurveStart.Value,curve.a)||!NativeBatchBindings.PlaybackHorizontal(e.CurveEnd.Value,curve.d))
            {reason="incident-curve-ports-changed";return false;}
            var port=entity==e.StartNode?curve.a:curve.d;
            if(math.distance(live.xz,port.xz)>anchor.CenterLimit){reason="outside-incident-port-footprint";return false;}
        }
        if(observed.Count!=expected.Count){reason="missing-live-incidence";return false;}
        reason=$"owner={source}; completeIncidentEdges={expected.Count}; establishedCenterLimit={anchor.CenterLimit:R}; curvePortsUnchanged=true";
        return true;
    }

    private IEnumerable<object> FailedLiveRecords(Entity entity,float3 preview,float3 live)
    {
        yield return new{kind="node",entity=$"{entity.Index}:{entity.Version}",preview=DiagnosticArtifacts.Point(preview),live=DiagnosticArtifacts.Point(live),horizontalDelta=math.distance(preview.xz,live.xz)};
        if(_generatedBatch!=null)foreach(var owner in _generatedBatch.Anchors)
            if(owner.Value.Entity==entity)yield return new{kind="source-owner",source=owner.Key,planned=DiagnosticArtifacts.Point(owner.Value.Planned),owner.Value.CenterLimit};
        foreach(var owner in _committedAnchors)
            if(owner.Value.Entity==entity)yield return new{kind="previous-owner",source=owner.Key,planned=DiagnosticArtifacts.Point(owner.Value.Planned),owner.Value.CenterLimit};
        if(!_entityManager.HasBuffer<ConnectedEdge>(entity))yield break;
        foreach(var incident in _entityManager.GetBuffer<ConnectedEdge>(entity,true))
        {
            var e=incident.m_Edge;if(!_entityManager.HasComponent<Edge>(e))continue;
            var edge=_entityManager.GetComponentData<Edge>(e);
            var prefab=_entityManager.HasComponent<PrefabRef>(e)?_entityManager.GetComponentData<PrefabRef>(e).m_Prefab:Entity.Null;
            var curve=_entityManager.HasComponent<Curve>(e)?_entityManager.GetComponentData<Curve>(e).m_Bezier:default;
            yield return new{kind="incident-edge",entity=$"{e.Index}:{e.Version}",start=edge.m_Start.ToString(),end=edge.m_End.ToString(),prefab=prefab.ToString(),curveStart=DiagnosticArtifacts.Point(curve.a),curveEnd=DiagnosticArtifacts.Point(curve.d),curveAvailable=_entityManager.HasComponent<Curve>(e)};
        }
    }

    private IEnumerable<object> LiveBindingRecords(NativeBatchBindings.Batch<Entity> batch)
    {
        foreach(var pair in batch.Anchors)
        {
            var live=_entityManager.GetComponentData<Node>(pair.Value.Entity).m_Position;
            yield return new { source=pair.Key,entity=$"{pair.Value.Entity.Index}:{pair.Value.Entity.Version}",planned=DiagnosticArtifacts.Point(pair.Value.Planned),preview=DiagnosticArtifacts.Point(pair.Value.Generated),live=DiagnosticArtifacts.Point(live),
                plannedHeightDelta=live.y-pair.Value.Planned.y,previewHeightDelta=live.y-pair.Value.Generated.y,horizontalCenterDelta=math.distance(live.xz,pair.Value.Planned.xz),pair.Value.CenterLimit,incidentBufferReady=true };
        }
    }

    private Entity CreateDefinition(PreparedSegment segment, Entity prefab)
    {
        NativeBatchBindings.Anchor<Entity>? Committed(long id)
        {
            if(!_committedNodes.TryGetValue(Identity(segment,id),out var node))return null;
            if(!_entityManager.HasComponent<Node>(node)||_entityManager.HasComponent<Temp>(node)||_entityManager.HasComponent<Deleted>(node))
                throw new InvalidOperationException($"Previously committed node {Identity(segment,id)} is unavailable; unsafe duplicate continuation refused.");
            var anchor=_committedAnchors[Identity(segment,id)];
            var position=_entityManager.GetComponentData<Node>(node).m_Position;
            if(!math.all(position.xz==anchor.Generated.xz)||!math.all(math.isfinite(position)))
                throw new InvalidOperationException($"Previously committed source {Identity(segment,id)} no longer matches its verified live center.");
            return anchor with{Generated=position};
        }
        segment=NativeBatchBindings.RebaseBound(segment,Committed(segment.StartNodeId),Committed(segment.EndNodeId));
        var curve = new Bezier4x3(segment.CourseStart,segment.CourseControl1,segment.CourseControl2,segment.CourseEnd);
        var direction = math.normalizesafe(segment.CourseControl1 - segment.CourseStart, new float3(0f, 0f, 1f));
        var rotation = quaternion.LookRotationSafe(direction, new float3(0f, 1f, 0f));
        var endDirection = math.normalizesafe(segment.CourseEnd - segment.CourseControl2, direction);
        var elevation = NetworkStructure.Offset(segment);
        var structure = NetworkStructure.IsStructure(segment);
        // Ground roads use their solved absolute curve and the prepared roadbed, not
        // an elevated composition merely because the original DEM was a little higher/lower.
        var startElevation = structure ? segment.StartElevation ?? elevation : 0f;
        var endElevation = structure ? segment.EndElevation ?? elevation : 0f;
        var startPosition = CoursePosition(segment.Start, rotation, true, _nodeElevations[Identity(segment,segment.StartNodeId)], segment.Bridge && !segment.Tunnel);
        var endPosition = CoursePosition(segment.End, quaternion.LookRotationSafe(endDirection, new float3(0, 1, 0)), false, _nodeElevations[Identity(segment,segment.EndNodeId)], segment.Bridge && !segment.Tunnel);
        if (_committedNodes.TryGetValue(Identity(segment,segment.StartNodeId),out var existingStart)) startPosition.m_Entity=existingStart;
        if (_committedNodes.TryGetValue(Identity(segment,segment.EndNodeId),out var existingEnd)) endPosition.m_Entity=existingEnd;
        // CourseSplitSystem.FindOverlapsJob checks the AND of both endpoint
        // flags. Protecting only an unbound end does nothing on a continuation:
        // the native tool can extend through/delete an already-owned junction.
        // Our prepared graph already owns all connections; preserve that graph
        // from the first preview, while explicit m_Entity still joins known nodes
        // and GenerateNodes' exact-position keys join new shared source nodes.
        startPosition.m_Flags|=CoursePosFlags.DisableMerge;
        endPosition.m_Flags|=CoursePosFlags.DisableMerge;
        var entity = _entityManager.CreateEntity();
        try
        {
        _entityManager.AddComponentData(entity, new CreationDefinition
        {
            m_Prefab = prefab,
            m_SubPrefab = Entity.Null,
            m_Original = Entity.Null,
            m_Owner = Entity.Null,
            m_Attached = Entity.Null,
            // SubElevation is an elevation-edit contract, not a generic import
            // flag. MergePositions bypasses endpoint separation when it is set.
            // Keep it only where explicit bridge/tunnel elevation is necessary.
            m_Flags = structure ? CreationFlags.SubElevation : 0,
            m_RandomSeed = (int)(math.hash(segment.Start) ^ math.hash(segment.End))
        });
        _entityManager.AddComponentData(entity, new NetCourse
        {
            m_Curve = curve,
            m_StartPosition = startPosition,
            m_EndPosition = endPosition,
            m_Elevation = new float2(startElevation, endElevation),
            m_Length = MathUtils.Length(curve),
            m_FixedIndex = -1
        });
        _entityManager.AddComponentData(entity, default(Updated));
        // GenerateNodes/GenerateEdges consume Upgraded on creation definitions.
        // Negative elevation chooses tunnels; never force a positive elevated node.
        var upgrades=default(CompositionFlags);
        if((segment.RetainingWallSides&NetworkSides.Left)!=0)upgrades.m_Left|=CompositionFlags.Side.Lowered;
        if((segment.RetainingWallSides&NetworkSides.Right)!=0)upgrades.m_Right|=CompositionFlags.Side.Lowered;
        if(upgrades!=default(CompositionFlags))_entityManager.AddComponentData(entity,new Upgraded{m_Flags=upgrades});
        _submittedCourses.Add(new(segment,prefab){StartCenterBudget=_centerBudgets[Identity(segment,segment.StartNodeId)],EndCenterBudget=_centerBudgets[Identity(segment,segment.EndNodeId)]});
        return entity;
        }
        catch { _entityManager.DestroyEntity(entity); throw; }
    }

    private static CoursePos CoursePosition(float3 position, quaternion rotation, bool start, float elevation, bool bridge) => new()
    {
        m_Entity = Entity.Null,
        m_Position = position,
        m_Rotation = rotation,
        m_Elevation = new float2(elevation),
        m_CourseDelta = start ? 0f : 1f,
        m_SplitPosition = 0f,
        m_Flags = (start ? CoursePosFlags.IsFirst : CoursePosFlags.IsLast) | CoursePosFlags.IsLeft | CoursePosFlags.IsRight | CoursePosFlags.FreeHeight |
            (bridge && elevation>.01f ? CoursePosFlags.ForceElevatedNode : 0) | (bridge ? CoursePosFlags.ForceElevatedEdge : 0),
        m_ParentMesh = -1
    };
}
