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
using EdgeStation = ImportEarthLocation.Integration.NativeNetworkDiagnostics.Station;

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
    private Dictionary<string, float> _nodeElevations = new(StringComparer.Ordinal);
    private readonly Dictionary<string, Entity> _committedNodes = new(StringComparer.Ordinal);

    private readonly Dictionary<string, NativeBatchBindings.Anchor<Entity>> _committedAnchors =
        new(StringComparer.Ordinal);

    private readonly Dictionary<string, float3> _plannedPositions = new(StringComparer.Ordinal);
    private readonly Dictionary<string, float> _centerBudgets = new(StringComparer.Ordinal);
    private readonly List<NativeBatchBindings.Expected<Entity>> _submittedCourses = new();
    private Dictionary<Entity, NativeBatchBindings.Edge<Entity>> _verifiedEdges = new();
    // Native incidence observation is not source certification; keep it across
    // advisory batches so later Combine can still resolve absorbed boundaries.
    private readonly Dictionary<Entity, NativeBatchBindings.Edge<Entity>> _observedEdges = new();
    private readonly Dictionary<Entity, NativeBatchBindings.Edge<Entity>> _pendingObservedEdges = new();
    private readonly HashSet<Entity> _advisoryMutationEdges = new();

    public bool LastBatchSourceVerified { get; private set; }

    private Dictionary<Entity, NativeBatchBindings.Edge<Entity>>? _stagedEdges;

    private IReadOnlyList<NativeBatchBindings.Edge<Entity>> _touchedEdges =
        Array.Empty<NativeBatchBindings.Edge<Entity>>();

    private readonly Dictionary<(NetworkKind Kind, long WayId, long Start, long End), List<Entity>> _verifiedPaths =
        new();

    private Dictionary<Entity, Entity> _stagedEdgeChanges = new();

    private readonly Dictionary<(NetworkKind Kind, long WayId, long Start, long End), List<Entity>> _stagedPaths =
        new();

    public string ArtifactStem { get; private set; } = "native-preflight";

    private NativeBatchBindings.Batch<Entity>? _generatedBatch;
    private bool _advisoryBatch;
    private bool _advisoryPublished;
    private readonly Dictionary<string, Entity> _advisoryOwners = new(StringComparer.Ordinal);
    private readonly Dictionary<string, EdgeStation> _edgeStations = new(StringComparer.Ordinal);
    private readonly HashSet<string> _mandatoryPorts = new(StringComparer.Ordinal);
    private readonly Dictionary<string, string> _declaredAliases = new(StringComparer.Ordinal);
    private Dictionary<string, Entity>? _stagedCommitted;
    private IReadOnlyList<PreparedSegment>? _source;
    private readonly HashSet<int> _locallyRefittedUnits = new();
    private readonly HashSet<(NetworkKind, long, long, long)> _revisedRoutes = new();
    private Dictionary<SourceNodeKey, Processing.NativeJunctionReconciliation.Port>? _pendingRefitPorts;

    public bool HasPendingLocalRefit => _pendingRefitPorts != null;

    public IReadOnlyDictionary<SourceNodeKey, Processing.NativeJunctionReconciliation.Port>? ConsumeLocalRefitPorts()
    {
        var ports = _pendingRefitPorts;
        _pendingRefitPorts = null;

        return ports;
    }

    public void RegisterLocalRevision(IReadOnlyList<PreparedSegment> revision)
    {
        foreach (var segment in revision)
        {
            var key = (segment.Kind, segment.WayId, segment.StartNodeId, segment.EndNodeId);
            _sourceRoutes[key] = segment;
            _revisedRoutes.Add(key);
        }

        foreach (var pair in NativeEndpointPolicy.Elevations(revision))
            _nodeElevations[pair.Key] = pair.Value;

        NativeNetworkDiagnostics.ReviseUnissued(revision);
        _log.DiagnosticRecordChunks(
            ArtifactStem + "-local-revision.ndjson",
            DiagnosticArtifacts.NetworkRecords(revision));
    }

    private Processing.OsmDataset? _evidence;

    private readonly Dictionary<(NetworkKind Kind, long WayId, long Start, long End), PreparedSegment> _sourceRoutes =
        new();

    public void BeginBatch(int startIndex, int attempt, int count, bool preflight = false)
    {
        ArtifactStem = $"native-{(preflight ? "preflight" : "batch")}-{startIndex:D6}-{count:D3}-attempt-{attempt:D2}";
        _log.Diagnostic(
            $"NATIVE COURSE POLICY artifact={ArtifactStem}; disableOverlapMergeBothEnds=true; subElevationStructuresOnly=true; committedEntityAttachmentRetained=true; preparedSourceTopologyAuthoritative=true; proximityAttachmentsAllowed=false");

        _generatedBatch = null;
        _stagedCommitted = null;
        _stagedEdges = null;
        _touchedEdges = Array.Empty<NativeBatchBindings.Edge<Entity>>();
        _stagedEdgeChanges.Clear();
        _advisoryBatch = false;
        _advisoryPublished = false;
        _advisoryOwners.Clear();
        LastBatchSourceVerified = false;
        _pendingObservedEdges.Clear();
        _advisoryMutationEdges.Clear();
        _stagedPaths.Clear();
    }

    public NativeNetworkPlacement(EntityManager entityManager, PrefabSystem prefabSystem, ImportLog log)
    {
        _entityManager = entityManager;
        _prefabs = new PrefabResolver(prefabSystem, log);
        _log = log;
    }

    public static IReadOnlyList<PreparedSegment> Preflight(
        IReadOnlyList<PreparedSegment> segments,
        PrefabSystem prefabs,
        ImportLog log,
        out bool needsRepair)
    {
        var resolver = new PrefabResolver(prefabs, log);
        var supported = new List<PreparedSegment>(segments.Count);
        var rejected = 0;
        needsRepair = false;

        foreach (var segment in segments)
        {
            try
            {
                var actual = resolver.ResolveMetadata(segment);
                // A frozen design may intentionally reserve the widest captured
                // side composition. A narrower base prefab is not a change.
                var widthChanged =
                    !math.isfinite(segment.NativeWidth)
                        || segment.NativeWidth <= 0
                        || actual.NativeWidth > segment.NativeWidth + .01f;

                var keyChanged = segment.NativeAssetKey != actual.NativeAssetKey;
                var slotsChanged =
                    !System.Linq.Enumerable.SequenceEqual(actual.NativeLaneOffsets, segment.NativeLaneOffsets);

                var changed = widthChanged || keyChanged || slotsChanged;

                if (changed)
                {
                    needsRepair = true;
                    log.Count("Native footprint sections reconciled");

                    if (widthChanged)
                        log.Count("Preflight change: SelectedModeEnvelope");

                    if (keyChanged)
                        log.Count("Preflight change: AssetKey");

                    if (slotsChanged)
                        log.Count("Preflight change: ComposedLaneOffsets");

                    if (log.Metric("Native footprint sections reconciled") <= 25)
                        log.Warn(
                            $"Native {segment.Kind} way {segment.WayId}: prepared width={segment.NativeWidth:0.##} m; selected width={actual.NativeWidth:0.##} m; lanes={segment.Lanes}. Retained for complete-network repair/re-audit before confirmation.");
                }

                supported.Add(actual with { NativeWidth = Math.Max(segment.NativeWidth, actual.NativeWidth) });
            }
            catch (Exception error) when (error is InvalidOperationException
                || error is ArgumentException
                || error is KeyNotFoundException)
            {
                rejected++;
                log.Count(segment.Kind + " unsupported native direction/width");

                if (rejected <= 25)
                    log.Warn($"Native metadata unavailable for {segment.Kind} way {segment.WayId}: {error.Message}");
            }
        }

        if (rejected > 0)
        {
            needsRepair = true;
            log.Count("Preflight change: SupportedSource", rejected);
            log.Warn(
                $"Native metadata unavailable for {rejected}/{segments.Count} sections; supported sections retained for generation.");
        }

        return supported;
    }

    // Sections omitted before submission, by reason (digits removed), for the placement summary.
    public Dictionary<string, int> SkipReasons { get; } = new();

    public IReadOnlyList<Entity> CreateDefinitions(
        IReadOnlyList<PreparedSegment> segments,
        out int skipped,
        Processing.OsmDataset? evidence = null)
    {
        NetworkCommitSystem.BeginDefinitionBatch(_entityManager, _log, ArtifactStem);
        _entityManager.CompleteAllTrackedJobs();

        if (_source == null)
            Initialize(segments, evidence);

        var definitions = new List<Entity>(segments.Count);
        _submittedCourses.Clear();
        skipped = 0;
        var roads = Stopwatch.StartNew();
        var roadTime = 0d;
        var railTime = 0d;
        PreparedSegment? active = null;

        try
        {
            foreach (var segment in segments)
            {
                active = segment;
                roads.Restart();

                try
                {
                    if (segment.StartNodeId == segment.EndNodeId
                        || !math.all(math.isfinite(segment.Start))
                        || !math.all(math.isfinite(segment.End))
                        || !math.all(math.isfinite(segment.CourseStart))
                            || !math.all(math.isfinite(segment.CourseControl1))
                            || !math.all(math.isfinite(segment.CourseControl2))
                                || !math.all(math.isfinite(segment.CourseEnd)))
                        throw new NativeSafetyException(
                            "Invalid source anchors or nonfinite course cannot be submitted to native generation.");

                    if (!Processing.NetworkDesignChecks.HasHorizontalExtent(
                        new(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd)))
                        throw new NativeSafetyException(
                            "The float-encoded course has no horizontal extent; no longitudinal road/track frame can be represented. Only this definition is omitted.");

                    var prefab = _prefabs.Resolve(segment);

                    if (prefab == Entity.Null)
                        throw new InvalidOperationException(
                            $"Native prefab unavailable for {segment.Kind} way {segment.WayId}.");

                    definitions.Add(CreateDefinition(segment, prefab));
                    _log.Count(segment.Kind + " definitions queued");
                }
                catch (Exception error) when (error is not OutOfMemoryException
                    && error is not OperationCanceledException)
                {
                    skipped++;
                    var message = error.GetBaseException().Message;
                    var kind = new System.Text.StringBuilder();

                    for (var c = 0; c < message.Length && c < 70; c++)
                        if (!char.IsDigit(message[c]))
                            kind.Append(message[c]);

                    var reason = kind.ToString();
                    SkipReasons[reason] = SkipReasons.TryGetValue(reason, out var seen) ? seen + 1 : 1;
                    _log.Warn(
                        $"Network definition unavailable: {segment.Kind} way {segment.WayId} {segment.StartNodeId}->{segment.EndNodeId}; continuing other sections. {error.GetBaseException().Message}");

                    if (_log.DiagnosticsEnabled)
                    {
                        _log.Diagnostic(
                            $"NATIVE DEFINITION OMITTED artifact={ArtifactStem}; way={segment.WayId}; nodes={segment.StartNodeId},{segment.EndNodeId}; exception={error}");

                        _log.DiagnosticRecords(
                            ArtifactStem + $"-definition-omitted-{skipped:D3}.ndjson",
                            DiagnosticArtifacts.NetworkRecords(new[] { segment }));
                    }
                }
                finally
                {
                    if (segment.Kind == NetworkKind.Road)
                        roadTime += roads.Elapsed.TotalSeconds;
                    else
                        railTime += roads.Elapsed.TotalSeconds;
                }
            }
        }
        catch (Exception error)
        {
            foreach (var definition in definitions)
                _entityManager.DestroyEntity(definition);

            if (error is OutOfMemoryException || error is OperationCanceledException)
                throw;

            throw new InvalidOperationException(
                $"Native definition failed for {active?.Kind} way {active?.WayId}; pending batch rolled back. {error.Message}",
                error);
        }

        if (roadTime > 0)
            _log.AddDuration("Road apply", roadTime);

        if (railTime > 0)
            _log.AddDuration("Rail apply", railTime);

        _log.DiagnosticRecords(ArtifactStem + "-expected.ndjson", ExpectedRecords());
        CapturePlacementEvidence(definitions, "submitted");

        if (definitions.Count == 0)
            NetworkCommitSystem.ReleaseEmptyDefinitions(_entityManager);

        return definitions;
    }

    // Read-only evidence: never use nearby nodes to choose an attachment.
    private void CapturePlacementEvidence(IReadOnlyList<Entity> definitions, string phase)
    {
        if (!_log.DiagnosticsEnabled)
            return;

        try
        {
            var records = new List<object>();

            foreach (var definition in definitions)
            {
                var creation = _entityManager.GetComponentData<CreationDefinition>(definition);
                var course = _entityManager.GetComponentData<NetCourse>(definition);

                object DescribePosition(CoursePos otherPosition) => new
                {
                    entity = NetworkCommitSystem.EntityId(otherPosition.m_Entity),
                    position = DiagnosticArtifacts.Point(otherPosition.m_Position),
                    rotation = new
                    {
                        x = otherPosition.m_Rotation.value.x,
                        y = otherPosition.m_Rotation.value.y,
                        z = otherPosition.m_Rotation.value.z,
                        w = otherPosition.m_Rotation.value.w
                    },
                    elevation = new { x = otherPosition.m_Elevation.x, y = otherPosition.m_Elevation.y },
                    otherPosition.m_CourseDelta,
                    otherPosition.m_SplitPosition,
                    flags = otherPosition.m_Flags.ToString(),
                    otherPosition.m_ParentMesh
                };

                records.Add(new
                {
                    kind = "definition",
                    entity = NetworkCommitSystem.EntityId(definition),
                    sourceSection = _submittedCourses.Count == definitions.Count
                        ? DiagnosticArtifacts.SectionIdentity(_submittedCourses[records.Count].Section)
                        : null,
                    sourceAssociationByCompleteDefinitionOrder = _submittedCourses.Count == definitions.Count,
                    creation = new
                    {
                        prefab = NetworkCommitSystem.EntityId(creation.m_Prefab),
                        original = NetworkCommitSystem.EntityId(creation.m_Original),
                        owner = NetworkCommitSystem.EntityId(creation.m_Owner),
                        attached = NetworkCommitSystem.EntityId(creation.m_Attached),
                        flags = creation.m_Flags.ToString(),
                        creation.m_RandomSeed
                    },
                    course = new
                    {
                        a = DiagnosticArtifacts.Point(course.m_Curve.a),
                        b = DiagnosticArtifacts.Point(course.m_Curve.b),
                        c = DiagnosticArtifacts.Point(course.m_Curve.c),
                        d = DiagnosticArtifacts.Point(course.m_Curve.d),
                        start = DescribePosition(course.m_StartPosition),
                        end = DescribePosition(course.m_EndPosition),
                        elevation = new { x = course.m_Elevation.x, y = course.m_Elevation.y },
                        course.m_Length,
                        course.m_FixedIndex
                    },
                    frame = UnityEngine.Time.frameCount
                });
            }

            // Retain every actual submitted curve before the optional neighbourhood
            // examples. These compact records do not contain repeated nearby graphs.
            if (phase == "submitted")
                _log.DiagnosticRecordChunks(ArtifactStem + "-submitted-courses.ndjson", records);

            using var query = _entityManager.CreateEntityQuery(ComponentType.ReadOnly<Node>());
            using var nodes = query.ToEntityArray(Allocator.Temp);
            var spatial = new Dictionary<(int X, int Z), List<Entity>>();

            foreach (var node in nodes)
            {
                var point = _entityManager.GetComponentData<Node>(node).m_Position;

                if (!math.all(math.isfinite(point)))
                    continue;

                var cell = ((int)math.floor(point.x / 64), (int)math.floor(point.z / 64));

                if (!spatial.TryGetValue(cell, out var bucket))
                    spatial[cell] = bucket = new List<Entity>();

                bucket.Add(node);
            }

            foreach (var expected in _submittedCourses)
                foreach (var endpoint in new[] {
                    (expected.Section.StartNodeId, expected.Section.Start),
                    (expected.Section.EndNodeId, expected.Section.End)
                    })
                {
                    var source = NativeEndpointPolicy.Identity(expected.Section, endpoint.Item1);
                    _committedNodes.TryGetValue(source, out var intended);
                    var candidates = new List<(Entity Entity, float Distance)>();
                    var nearby = new HashSet<Entity>();
                    var center = ((int)math.floor(endpoint.Item2.x / 64), (int)math.floor(endpoint.Item2.z / 64));

                    for (var x = -1; x <= 1; x++)
                        for (var z = -1; z <= 1; z++)
                            if (spatial.TryGetValue((center.Item1 + x, center.Item2 + z), out var bucket))
                                foreach (var node in bucket)
                                    nearby.Add(node);

                    if (intended != Entity.Null
                        && _entityManager.Exists(intended)
                        && _entityManager.HasComponent<Node>(intended))
                        nearby.Add(intended);

                    foreach (var node in nearby)
                    {
                        var position = _entityManager.GetComponentData<Node>(node).m_Position;
                        var distance = math.distance(position.xz, endpoint.Item2.xz);

                        if (distance <= 64 || node == intended)
                            candidates.Add((node, distance));
                    }

                    candidates.Sort((firstItem, secondItem) => firstItem.Distance.CompareTo(secondItem.Distance));
                    var recorded = 0;

                    foreach (var candidate in candidates)
                    {
                        if (recorded >= 64 && candidate.Entity != intended)
                            continue;

                        var node = candidate.Entity;
                        var incidents = new List<object>();

                        if (_entityManager.HasBuffer<ConnectedEdge>(node))
                            foreach (var link in _entityManager.GetBuffer<ConnectedEdge>(node, true))
                            {
                                if (incidents.Count >= 128)
                                    break;

                                var edge = link.m_Edge;
                                var exists = _entityManager.Exists(edge);
                                incidents.Add(new
                                {
                                    entity = NetworkCommitSystem.EntityId(edge),
                                    exists,
                                    edge = exists && _entityManager.HasComponent<Edge>(edge)
                                        ? new
                                        {
                                            start = NetworkCommitSystem.EntityId(
                                                _entityManager.GetComponentData<Edge>(edge).m_Start),
                                            end = NetworkCommitSystem.EntityId(
                                                _entityManager.GetComponentData<Edge>(edge).m_End)
                                        }
                                        : null,
                                    curve = exists && _entityManager.HasComponent<Curve>(edge)
                                        ? CurveEvidence(_entityManager.GetComponentData<Curve>(edge).m_Bezier)
                                        : null,
                                    prefab = exists && _entityManager.HasComponent<PrefabRef>(edge)
                                        ? NetworkCommitSystem.EntityId(
                                            _entityManager.GetComponentData<PrefabRef>(edge).m_Prefab)
                                        : null
                                });
                            }

                        records.Add(new
                        {
                            kind = "endpoint-neighborhood",
                            source,
                            expected.Section.WayId,
                            intended = NetworkCommitSystem.EntityId(intended),
                            entity = NetworkCommitSystem.EntityId(node),
                            candidate.Distance,
                            position = DiagnosticArtifacts.Point(
                                _entityManager.GetComponentData<Node>(node).m_Position),
                            deleted = _entityManager.HasComponent<Deleted>(node),
                            temp = _entityManager.HasComponent<Temp>(node)
                                ? new
                                {
                                    original = NetworkCommitSystem.EntityId(
                                        _entityManager.GetComponentData<Temp>(node).m_Original),
                                    flags = _entityManager.GetComponentData<Temp>(node).m_Flags.ToString()
                                }
                                : null,
                            incidents,
                            candidatesFound = candidates.Count,
                            truncated = candidates.Count > 64,
                            searchRadiusMeters = 64,
                            attachmentByProximity = false
                        });

                        recorded++;
                    }
                }

            _log.DiagnosticRecords(
                ArtifactStem + "-" + phase + "-placement-evidence.ndjson",
                records,
                32L * 1024 * 1024);
        }
        catch (Exception error) when (error is not OutOfMemoryException)
        {
            _log.Diagnostic(
                $"NATIVE DIAGNOSTIC capture incomplete artifact={ArtifactStem}; phase={phase}; exception={error}");
        }
    }

    public void Initialize(IReadOnlyList<PreparedSegment> source, Processing.OsmDataset? evidence)
    {
        _source = source;
        _evidence = evidence;
        _locallyRefittedUnits.Clear();
        _revisedRoutes.Clear();
        _pendingRefitPorts = null;
        _observedEdges.Clear();
        _pendingObservedEdges.Clear();
        LastBatchSourceVerified = false;
        _edgeStations.Clear();
        _advisoryOwners.Clear();
        _advisoryBatch = false;
        _mandatoryPorts.Clear();
        _declaredAliases.Clear();
        var incidents = new Dictionary<string, List<PreparedSegment>>(StringComparer.Ordinal);

        foreach (var section in source)
        {
            foreach (var node in new[] { section.StartNodeId, section.EndNodeId })
            {
                var key = NativeEndpointPolicy.Identity(section, node);

                if (!incidents.TryGetValue(key, out var arms))
                    incidents[key] = arms = new();

                arms.Add(section);
            }

            foreach (var alias in section.SourceNodeAliases)
            {
                _declaredAliases[NativeEndpointPolicy.Identity(section, alias.SourceNodeId)] =
                    NativeEndpointPolicy.Identity(section, alias.CanonicalNodeId);

                if (alias.Mandatory)
                    _mandatoryPorts.Add(NativeEndpointPolicy.Identity(section, alias.CanonicalNodeId));
            }
        }

        foreach (var entry in incidents)
        {
            var first = entry.Value[0];

            if (entry.Value.Count != 2
                || entry.Value.Exists(
                    segment => segment.NativeAssetKey != first.NativeAssetKey
                        || segment.Layer != first.Layer
                        || segment.Tunnel != first.Tunnel
                        || segment.Bridge != first.Bridge
                        || segment.StartTerminal
                    || segment.EndTerminal
                    || segment.Roundabout
                    || segment.StartMiniRoundabout
                    || segment.EndMiniRoundabout))
                _mandatoryPorts.Add(entry.Key);
        }

        _sourceRoutes.Clear();

        foreach (var section in source)
        {
            var key = (section.Kind, section.WayId, section.StartNodeId, section.EndNodeId);

            if (!_sourceRoutes.ContainsKey(key))
                _sourceRoutes[key] = section;
            else
                _log.Count("Duplicate native source route keys observed");
        }

        RoadSections = RailSections = 0;
        _stagedPaths.Clear();
        _nodeElevations = NativeEndpointPolicy.Elevations(source);
        _isolatedEndpoints.Clear();
        _committedNodes.Clear();
        _committedAnchors.Clear();
        _plannedPositions.Clear();
        _centerBudgets.Clear();
        _verifiedEdges.Clear();
        _verifiedPaths.Clear();
        _stagedEdgeChanges.Clear();
        _stagedEdges = null;
        _touchedEdges = Array.Empty<NativeBatchBindings.Edge<Entity>>();
        _generatedBatch = null;
        _stagedCommitted = null;
        NativeNetworkDiagnostics.Remember(source, evidence);
        var positions = new Dictionary<float3, HashSet<string>>();

        foreach (var segment in source)
            foreach (var endpoint in new[] { (segment.Start, segment.StartNodeId), (segment.End, segment.EndNodeId) })
            {
                if (!positions.TryGetValue(endpoint.Item1, out var identities))
                    positions[endpoint.Item1] = identities = new HashSet<string>();

                identities.Add(NativeEndpointPolicy.Identity(segment, endpoint.Item2));
                _plannedPositions[NativeEndpointPolicy.Identity(segment, endpoint.Item2)] = endpoint.Item1;
                var key = NativeEndpointPolicy.Identity(segment, endpoint.Item2);
                var budget = NativeBatchBindings.CenterBudget(segment);
                _centerBudgets[key] =
                    _centerBudgets.TryGetValue(key, out var oldBudget) ? Math.Max(oldBudget, budget) : budget;
            }

        foreach (var identities in positions.Values)
            if (identities.Count > 1)
                foreach (var identity in identities)
                    _isolatedEndpoints.Add(identity);
    }

    // Capture source-to-entity bindings while the submitted graph still has Temp
    // lineage. ApplyNet.Create retains its entity; Update redirects to Original.
    public void CaptureGeneratedNodes(IReadOnlyList<PreparedSegment> batch, IReadOnlyList<Entity> temporary)
    {
        _entityManager.CompleteAllTrackedJobs();

        if (StageLocalRefit(batch))
            throw new NativeRefitRequest();

        // Capture the native mutation closure while Original incidence still
        // exists. An advisory batch invalidates this closure, not the entire
        // previously certified city. Entity versions remain part of each key.
        foreach (var entity in temporary)
        {
            if (!_entityManager.HasComponent<Temp>(entity))
                continue;

            var original = _entityManager.GetComponentData<Temp>(entity).m_Original;

            if (original == Entity.Null)
                continue;

            if (_entityManager.HasComponent<Edge>(original))
                _advisoryMutationEdges.Add(original);

            if (_entityManager.HasComponent<Node>(original) && _entityManager.HasBuffer<ConnectedEdge>(original))
                foreach (var incident in _entityManager.GetBuffer<ConnectedEdge>(original, true))
                    _advisoryMutationEdges.Add(incident.m_Edge);
        }

        try
        {
            CaptureQualifiedNodes(batch, temporary);
        }
        catch (Exception error) when (error is not OutOfMemoryException
            && error is not OperationCanceledException
            && error is not NativeSafetyException)
        {
            _log.Count("Native source verification advisories");

            if (_log.Metric("Native source verification advisories") <= 10)
                _log.Warn(
                    $"Native source verification is advisory; applying generated preview for inspection. {error.GetBaseException().Message}");

            _log.Diagnostic(
                $"NATIVE QUALIFICATION ADVISORY artifact={ArtifactStem}; exception={error}; applied=false; sourceVerified=false; duplicateRetry=false");

            CaptureAdvisoryLineage(temporary);
        }
    }

    private bool StageLocalRefit(IReadOnlyList<PreparedSegment> batch)
    {
        if (batch.Count == 0
            || System.Linq.Enumerable.Any(
                batch,
                segment => segment.NativeUnitId < 0 || _locallyRefittedUnits.Contains(segment.NativeUnitId)))
            return false;

        var degrees = new Dictionary<SourceNodeKey, int>();

        foreach (var segment in batch)
            foreach (var id in new[] { segment.StartNodeId, segment.EndNodeId })
            {
                var key = new SourceNodeKey(segment.Kind, id);
                degrees[key] = degrees.TryGetValue(key, out var count) ? count + 1 : 1;
            }

        var ports = new Dictionary<SourceNodeKey, Processing.NativeJunctionReconciliation.Port>();
        var moved = false;

        foreach (var segment in batch)
            foreach (var start in new[] { true, false })
            {
                var id = start ? segment.StartNodeId : segment.EndNodeId;
                var key = new SourceNodeKey(segment.Kind, id);

                if (degrees[key] != 1
                    || !_committedNodes.TryGetValue(NativeEndpointPolicy.Identity(segment, id), out var owner)
                    || !_entityManager.HasComponent<Node>(owner)
                    || !_entityManager.HasBuffer<ConnectedEdge>(owner))
                    continue;

                var desired =
                    math.normalizesafe(
                        (start
                            ? segment.CourseControl1 - segment.CourseStart
                            : segment.CourseEnd - segment.CourseControl2).xz);

                var best = .9f;
                float3 tangent = default;
                var unique = false;

                foreach (var link in _entityManager.GetBuffer<ConnectedEdge>(owner, true))
                {
                    if (!_verifiedEdges.ContainsKey(link.m_Edge)
                        || !_entityManager.HasComponent<Curve>(link.m_Edge)
                        || !_entityManager.HasComponent<Edge>(link.m_Edge))
                        continue;

                    var edge = _entityManager.GetComponentData<Edge>(link.m_Edge);
                    var curve = _entityManager.GetComponentData<Curve>(link.m_Edge).m_Bezier;
                    var derivative = edge.m_Start == owner ? curve.b - curve.a : curve.d - curve.c;
                    var sign = math.dot(derivative.xz, desired) >= 0 ? 1 : -1;
                    derivative *= sign;
                    var score = math.dot(math.normalizesafe(derivative.xz), desired);

                    if (score < best)
                        continue;

                    if (Math.Abs(score - best) < .0001f)
                    {
                        unique = false;
                        continue;
                    }

                    best = score;
                    tangent = derivative;
                    unique = true;
                }

                var reach = math.length(tangent.xz);

                if (!unique || !(reach > 1e-5))
                    continue;

                var position = _entityManager.GetComponentData<Node>(owner).m_Position;
                ports[key] = new(position, tangent.xz / reach, tangent.y / reach, true);
                var originalDerivative =
                    start ? segment.CourseControl1 - segment.CourseStart : segment.CourseEnd - segment.CourseControl2;

                var originalReach = math.length(originalDerivative.xz);
                var originalGrade = originalReach > 1e-8 ? originalDerivative.y / originalReach : 0;

                if (math.distance(position, start ? segment.Start : segment.End) > .1
                    || math.dot(desired, tangent.xz / reach) < .999
                    || Math.Abs(originalGrade - tangent.y / reach) > .001)
                    moved = true;
            }

        if (!moved)
            return false;

        foreach (var segment in batch)
            _locallyRefittedUnits.Add(segment.NativeUnitId);

        _pendingRefitPorts = ports;
        _log.Diagnostic(
            $"LOCAL REFIT requested unappliedSections={batch.Count}; exactBoundaryPoses={ports.Count}; structuralOnly=false; oldPreviewMustRetireBeforeWorker=true; oncePerUnit=true; appliedOwnersWillNotBeMutated=true");

        return true;
    }

    // Playback lineage is independent of whether a source curve fits the engine's
    // regenerated curve. Never turn a missing degree-two boundary into a stale owner.
    private void CaptureAdvisoryLineage(IReadOnlyList<Entity> temporary)
    {
        var captured = _generatedBatch;
        _advisoryBatch = true;
        _generatedBatch = null;
        _advisoryOwners.Clear();
        var replacements = new Dictionary<Entity, Entity>();
        var removed = new HashSet<Entity>();

        foreach (var entity in temporary)
        {
            if (!_entityManager.HasComponent<Temp>(entity))
                continue;

            var otherTemporary = _entityManager.GetComponentData<Temp>(entity);

            if (otherTemporary.m_Original == Entity.Null)
                continue;

            if ((otherTemporary.m_Flags & TempFlags.Delete) != 0)
                removed.Add(otherTemporary.m_Original);
            else if (_entityManager.HasComponent<Node>(entity))
                replacements[otherTemporary.m_Original] =
                    (otherTemporary.m_Flags & TempFlags.Replace) != 0 ? entity : otherTemporary.m_Original;
        }

        foreach (var pair in _committedNodes)
            if (replacements.TryGetValue(pair.Value, out var replacement))
                _advisoryOwners[pair.Key] = replacement;
            else if (!removed.Contains(pair.Value))
                _advisoryOwners[pair.Key] = pair.Value;

        if (captured != null)
            foreach (var pair in captured.Anchors)
                if (!removed.Contains(pair.Value.Entity))
                    _advisoryOwners[pair.Key] = pair.Value.Entity;

        var ownersByNode = new Dictionary<Entity, List<string>>();

        foreach (var owner in _committedNodes)
        {
            if (!ownersByNode.TryGetValue(owner.Value, out var keys))
                ownersByNode[owner.Value] = keys = new();

            keys.Add(owner.Key);
        }

        // Native Combine absorbs degree-two source boundaries into edge interiors.
        // Original edge incidence, never proximity alone, establishes the station.
        foreach (var entity in temporary)
        {
            if (!_entityManager.HasComponent<Edge>(entity)
                || !_entityManager.HasComponent<Curve>(entity)
                || !_entityManager.HasComponent<PrefabRef>(entity))
                continue;

            var otherTemporary = _entityManager.GetComponentData<Temp>(entity);

            if ((otherTemporary.m_Flags & TempFlags.Delete) != 0)
                continue;

            var target =
                (otherTemporary.m_Flags & (TempFlags.Replace | TempFlags.Combine)) != 0
                    ? entity
                    : otherTemporary.m_Original != Entity.Null ? otherTemporary.m_Original : entity;

            var curve = _entityManager.GetComponentData<Curve>(entity).m_Bezier;
            var prefab = _entityManager.GetComponentData<PrefabRef>(entity).m_Prefab;

            foreach (var pair in new List<KeyValuePair<string, EdgeStation>>(_edgeStations))
                if (pair.Value.Edge == otherTemporary.m_Original)
                {
                    var t = CurveParameter(curve, pair.Value.Position);
                    _edgeStations[pair.Key] = new(target, prefab, t, CurvePoint(curve, t));
                }

            if ((otherTemporary.m_Flags & TempFlags.Combine) == 0
                || !_observedEdges.TryGetValue(otherTemporary.m_Original, out var prior)
                    && !_verifiedEdges.TryGetValue(otherTemporary.m_Original, out prior))
                continue;

            foreach (var node in new[] { prior.StartNode, prior.EndNode })
            {
                if (!removed.Contains(node) || !ownersByNode.TryGetValue(node, out var owners))
                    continue;

                foreach (var owner in owners)
                {
                    var point = _committedAnchors[owner].Generated;
                    var t = CurveParameter(curve, point);

                    if (t <= 0 || t >= 1 || !math.all(math.isfinite(CurvePoint(curve, t))))
                        continue;

                    _edgeStations[owner] = new(target, prefab, t, CurvePoint(curve, t));
                    _log.Diagnostic(
                        $"NATIVE COMBINE STATION source={owner}; originalEdge={otherTemporary.m_Original}; replacementEdge={target}; split={t:R}; sourcePosition={point}; nativePosition={CurvePoint(curve, t)}; originalIncidenceProven=true; proximityOwnerGuess=false");
                }
            }
        }

        Entity NodeTarget(Entity node)
        {
            if (!_entityManager.HasComponent<Temp>(node))
                return node;

            var otherTemporary = _entityManager.GetComponentData<Temp>(node);

            return otherTemporary.m_Original != Entity.Null && (otherTemporary.m_Flags & TempFlags.Replace) == 0
                ? otherTemporary.m_Original
                : node;
        }

        foreach (var entity in temporary)
        {
            if (!_entityManager.HasComponent<Edge>(entity))
                continue;

            var otherTemporary = _entityManager.GetComponentData<Temp>(entity);

            if (otherTemporary.m_Original != Entity.Null)
                _observedEdges.Remove(otherTemporary.m_Original);

            if ((otherTemporary.m_Flags & TempFlags.Delete) != 0
                || !_entityManager.HasComponent<Curve>(entity)
                || !_entityManager.HasComponent<PrefabRef>(entity))
                continue;

            var edge = _entityManager.GetComponentData<Edge>(entity);
            var curve = _entityManager.GetComponentData<Curve>(entity).m_Bezier;

            if (!_entityManager.HasComponent<Node>(edge.m_Start) || !_entityManager.HasComponent<Node>(edge.m_End))
                continue;

            var target =
                (otherTemporary.m_Flags & (TempFlags.Replace | TempFlags.Combine)) != 0
                    ? entity
                    : otherTemporary.m_Original != Entity.Null ? otherTemporary.m_Original : entity;

            var kind =
                _entityManager.HasComponent<TrackData>(_entityManager.GetComponentData<PrefabRef>(entity).m_Prefab)
                    ? NetworkKind.Rail
                    : NetworkKind.Road;

            _pendingObservedEdges[target] =
                new(
                    target,
                    NodeTarget(edge.m_Start),
                    NodeTarget(edge.m_End),
                    _entityManager.GetComponentData<Node>(edge.m_Start).m_Position,
                    _entityManager.GetComponentData<Node>(edge.m_End).m_Position,
                    kind,
                    _entityManager.GetComponentData<PrefabRef>(entity).m_Prefab)
                { CurveStart = curve.a, CurveEnd = curve.d, CurveControl1 = curve.b, CurveControl2 = curve.c };
        }

        // Strict matching may have established some batch anchors before a prior
        // route audit disagreed. Retain their entity lineage, not guessed positions.
        if (_stagedCommitted != null)
            foreach (var pair in _stagedCommitted)
                if (!removed.Contains(pair.Value))
                    _advisoryOwners[pair.Key] = pair.Value;

        foreach (var expected in _submittedCourses)
            foreach (var end in new[] {
                (expected.Section.StartNodeId, expected.Section.CourseStart),
                (expected.Section.EndNodeId, expected.Section.CourseEnd)
                })
            {
                var key = NativeEndpointPolicy.Identity(expected.Section, end.Item1);

                if (_edgeStations.ContainsKey(key))
                    continue;

                var candidates = new HashSet<Entity>();

                foreach (var entity in temporary)
                {
                    if (!_entityManager.HasComponent<Edge>(entity)
                        || !_entityManager.HasComponent<Curve>(entity)
                        || !_entityManager.HasComponent<PrefabRef>(entity))
                        continue;

                    if (_entityManager.GetComponentData<PrefabRef>(entity).m_Prefab != expected.Prefab)
                        continue;

                    if ((_entityManager.GetComponentData<Temp>(entity).m_Flags & TempFlags.Delete) != 0)
                        continue;

                    var edge = _entityManager.GetComponentData<Edge>(entity);
                    var curve = _entityManager.GetComponentData<Curve>(entity).m_Bezier;

                    foreach (var port in new[] { (edge.m_Start, curve.a), (edge.m_End, curve.d) })
                    {
                        if (!NativeBatchBindings.PlaybackHorizontal(end.Item2, port.Item2)
                            || !_entityManager.HasComponent<Node>(port.Item1))
                            continue;

                        var target = port.Item1;

                        if (_entityManager.HasComponent<Temp>(target))
                        {
                            var otherTemporary = _entityManager.GetComponentData<Temp>(target);

                            if ((otherTemporary.m_Flags & TempFlags.Delete) != 0)
                                continue;

                            if (otherTemporary.m_Original != Entity.Null
                                && (otherTemporary.m_Flags & TempFlags.Replace) == 0)
                                target = otherTemporary.m_Original;
                        }

                        candidates.Add(target);
                    }
                }

                if (candidates.Count == 1)
                    _advisoryOwners[key] = System.Linq.Enumerable.First(candidates);
                else
                    _log.Diagnostic(
                        $"NATIVE ADVISORY ENDPOINT source={key}; exactPortCandidates={candidates.Count}; explicitAttachmentRetained={_advisoryOwners.ContainsKey(key)}; nearestOwnerGuess=false");
            }

        _log.Diagnostic(
            $"NATIVE ADVISORY LINEAGE artifact={ArtifactStem}; retainedOwners={_advisoryOwners.Count}; deletedOriginals={removed.Count}; replacements={replacements.Count}; sourceRouteVerification=false");
    }

    public void AcceptUnverifiedPlayback(string reason)
    {
        _log.Warn(
            $"Native batch readback remains diagnostic: {reason}. No Apply replay; continuing remaining features.");

        if (!_advisoryBatch)
        {
            _advisoryBatch = true;
            _advisoryOwners.Clear();

            if (_stagedCommitted != null)
                foreach (var pair in _stagedCommitted)
                    _advisoryOwners[pair.Key] = pair.Value;

            if (_generatedBatch != null)
                foreach (var pair in _generatedBatch.Anchors)
                    _advisoryOwners[pair.Key] = pair.Value.Entity;
        }

        PublishAdvisoryOwners();
    }

    private static float3 CurvePoint(Bezier4x3 curve, float t)
    {
        var u = 1 - t;
        return u * u * u * curve.a + 3 * u * u * t * curve.b + 3 * u * t * t * curve.c + t * t * t * curve.d;
    }

    private static float CurveParameter(Bezier4x3 curve, float3 point)
    {
        var best = 0f;
        var distance = float.MaxValue;

        for (var i = 0; i <= 64; i++)
        {
            var t = i / 64f;
            var d = math.distancesq(CurvePoint(curve, t).xz, point.xz);

            if (d < distance)
            {
                distance = d;
                best = t;
            }
        }

        var low = Math.Max(0, best - 1 / 64f);
        var high = Math.Min(1, best + 1 / 64f);

        for (var i = 0; i < 18; i++)
        {
            var a = (2 * low + high) / 3;
            var b = (low + 2 * high) / 3;

            if (math.distancesq(CurvePoint(curve, a).xz, point.xz) < math.distancesq(CurvePoint(curve, b).xz, point.xz))
                high = b;
            else
                low = a;
        }

        var result = (low + high) / 2;
        var interior = math.distancesq(CurvePoint(curve, result).xz, point.xz);

        if (math.distancesq(curve.a.xz, point.xz) <= interior)
            return 0;

        if (math.distancesq(curve.d.xz, point.xz) <= interior)
            return 1;

        return result;
    }

    private void PublishAdvisoryOwners()
    {
        if (_advisoryPublished)
            return;

        ReconcileAdvisoryRoles();

        foreach (var key in new List<string>(_committedNodes.Keys))
            if (!_advisoryOwners.ContainsKey(key))
            {
                _committedNodes.Remove(key);
                _committedAnchors.Remove(key);
            }

        foreach (var pair in _advisoryOwners)
        {
            if (!_entityManager.HasComponent<Node>(pair.Value)
                || _entityManager.HasComponent<Temp>(pair.Value)
                || _entityManager.HasComponent<Deleted>(pair.Value))
            {
                _committedNodes.Remove(pair.Key);
                _committedAnchors.Remove(pair.Key);
                continue;
            }

            var point = _entityManager.GetComponentData<Node>(pair.Value).m_Position;

            if (!math.all(math.isfinite(point)))
            {
                _committedNodes.Remove(pair.Key);
                _committedAnchors.Remove(pair.Key);
                continue;
            }

            _committedNodes[pair.Key] = pair.Value;
            _committedAnchors[pair.Key] =
                new(pair.Value, _plannedPositions[pair.Key], point) { CenterLimit = _centerBudgets[pair.Key] };
        }

        // Paths touched by an unqualified Combine lose certification. Paths
        // outside the captured mutation/incidence closure keep their evidence;
        // this never certifies the newly applied advisory batch.
        foreach (var pair in new List<KeyValuePair<string, EdgeStation>>(_edgeStations))
            if (!_entityManager.HasComponent<Edge>(pair.Value.Edge)
                || _entityManager.HasComponent<Temp>(pair.Value.Edge)
                || _entityManager.HasComponent<Deleted>(pair.Value.Edge)
                || !_entityManager.HasComponent<Curve>(pair.Value.Edge)
                || !_entityManager.HasComponent<PrefabRef>(pair.Value.Edge)
                || _entityManager.GetComponentData<PrefabRef>(pair.Value.Edge).m_Prefab != pair.Value.Prefab)
                _edgeStations.Remove(pair.Key);

        foreach (var pair in _pendingObservedEdges)
            if (_entityManager.HasComponent<Edge>(pair.Key)
                && !_entityManager.HasComponent<Temp>(pair.Key)
                && !_entityManager.HasComponent<Deleted>(pair.Key))
                _observedEdges[pair.Key] = pair.Value;

        var invalidated = new List<(NetworkKind Kind, long WayId, long Start, long End)>();

        foreach (var path in _verifiedPaths)
            if (path.Value.Exists(_advisoryMutationEdges.Contains))
                invalidated.Add(path.Key);

        foreach (var path in invalidated)
            _verifiedPaths.Remove(path);

        foreach (var edge in _advisoryMutationEdges)
            _verifiedEdges.Remove(edge);

        _stagedPaths.Clear();
        NativeNetworkDiagnostics.BindVerified(_committedNodes, _verifiedPaths);
        NativeNetworkDiagnostics.BindObserved(_observedEdges, _edgeStations);
        CaptureSettledReadback(invalidated);
        _generatedBatch = null;
        _stagedCommitted = null;
        _stagedEdges = null;
        _touchedEdges = Array.Empty<NativeBatchBindings.Edge<Entity>>();
        _log.Count("Native batches applied without source certification");
        _advisoryPublished = true;
        _log.Diagnostic(
            $"NATIVE LIVE ADVISORY artifact={ArtifactStem}; liveOwners={_committedNodes.Count}; sourceVerified=false; invalidatedPriorPaths={invalidated.Count}; preservedPriorPaths={_verifiedPaths.Count}; mutationClosureEdges={_advisoryMutationEdges.Count}; newAdvisoryPathsCertified=0; applyReplayed=false");
    }

    private void ReconcileAdvisoryRoles()
    {
        var groups = new Dictionary<Entity, List<string>>();

        foreach (var pair in _advisoryOwners)
        {
            if (!groups.TryGetValue(pair.Value, out var keys))
                groups[pair.Value] = keys = new();

            keys.Add(pair.Key);
        }

        foreach (var group in groups)
        {
            if (group.Value.Count < 2)
                continue;

            foreach (var key in group.Value)
            {
                if (_declaredAliases.TryGetValue(key, out var canonical)
                    && canonical != key
                    && _advisoryOwners.TryGetValue(canonical, out var same)
                    && same == group.Key)
                {
                    _advisoryOwners.Remove(key);
                    continue;
                }

                if (_mandatoryPorts.Contains(key)
                    || !_plannedPositions.TryGetValue(key, out var planned)
                    || !_entityManager.HasBuffer<ConnectedEdge>(group.Key))
                    continue;

                EdgeStation? found = null;
                var ambiguous = false;

                foreach (var incident in _entityManager.GetBuffer<ConnectedEdge>(group.Key, true))
                {
                    var edge = incident.m_Edge;

                    if (!_observedEdges.ContainsKey(edge)
                        && !_pendingObservedEdges.ContainsKey(edge)
                        && !_verifiedEdges.ContainsKey(edge))
                        continue;

                    if (!_entityManager.HasComponent<Curve>(edge)
                        || !_entityManager.HasComponent<PrefabRef>(edge)
                        || _entityManager.HasComponent<Deleted>(edge)
                        || _entityManager.HasComponent<Temp>(edge))
                        continue;

                    var curve = _entityManager.GetComponentData<Curve>(edge).m_Bezier;
                    var t = CurveParameter(curve, planned);
                    var point = CurvePoint(curve, t);

                    if (math.distance(point.xz, planned.xz) > .05f)
                        continue;

                    var station =
                        new EdgeStation(edge, _entityManager.GetComponentData<PrefabRef>(edge).m_Prefab, t, point);

                    if (found != null)
                    {
                        ambiguous = true;
                        break;
                    }

                    found = station;
                }

                // The exact original station and owned incidence, not a nearby
                // node, authorize an absorbed optional station. Ambiguous ones
                // remain unavailable and are never attached to a guessed owner.
                _advisoryOwners.Remove(key);

                if (found != null && !ambiguous)
                    _edgeStations[key] = found;

                _log.Diagnostic(
                    $"NATIVE SOURCE ROLE source={key}; role={(found != null && !ambiguous ? "owned-edge-station" : "unavailable-optional-station")}; mandatory=false; proximityNodeAttachment=false");
            }
        }
    }

    private void CaptureQualifiedNodes(IReadOnlyList<PreparedSegment> batch, IReadOnlyList<Entity> temporary)
    {
        _entityManager.CompleteAllTrackedJobs();
        var actual = new List<NativeBatchBindings.Edge<Entity>>();

        Entity Target(Entity entity)
        {
            if (!_entityManager.HasComponent<Temp>(entity))
                return entity;

            var otherTemporary = _entityManager.GetComponentData<Temp>(entity);
            // Installed ApplyNet: nodes use Replace; edges use Replace|Combine.
            var replace =
                (otherTemporary.m_Flags & TempFlags.Replace) != 0
                    || (_entityManager.HasComponent<Edge>(entity) && (otherTemporary.m_Flags & TempFlags.Combine) != 0);

            return NativeBatchBindings.Target(
                entity,
                otherTemporary.m_Original,
                otherTemporary.m_Original != Entity.Null,
                replace);
        }

        var previewPositions = new Dictionary<Entity, float3>();
        var replacements = new Dictionary<Entity, Entity>();

        foreach (var entity in temporary)
        {
            if (!_entityManager.HasComponent<Node>(entity))
                continue;

            if ((_entityManager.GetComponentData<Temp>(entity).m_Flags & TempFlags.Delete) != 0)
                continue;

            var target = Target(entity);
            var point = _entityManager.GetComponentData<Node>(entity).m_Position;

            if (previewPositions.TryGetValue(target, out var old) && !math.all(old == point))
                throw new NativeSafetyException("Multiple native node updates disagree on their committed anchor.");

            previewPositions[target] = point;
            var original = _entityManager.GetComponentData<Temp>(entity).m_Original;

            if (original != Entity.Null && target != original)
            {
                if (replacements.TryGetValue(original, out var other) && other != target)
                    throw new NativeSafetyException("Multiple native replacements disagree on one source node.");

                replacements[original] = target;
            }
        }

        float3 Position(Entity node)
            => previewPositions.TryGetValue(Target(node), out var point)
                ? point
                : _entityManager.GetComponentData<Node>(node).m_Position;

        foreach (var entity in temporary)
        {
            if (!_entityManager.HasComponent<Edge>(entity) || !_entityManager.HasComponent<PrefabRef>(entity))
                continue;

            var otherTemporary = _entityManager.GetComponentData<Temp>(entity);

            if ((otherTemporary.m_Flags & TempFlags.Delete) != 0)
                continue;

            var edge = _entityManager.GetComponentData<Edge>(entity);
            var prefab = _entityManager.GetComponentData<PrefabRef>(entity).m_Prefab;
            var road = _entityManager.HasComponent<RoadData>(prefab);

            if (!road && !_entityManager.HasComponent<TrackData>(prefab))
                continue;

            if (!_entityManager.HasComponent<Curve>(entity))
                throw new NativeSafetyException("Native generated edge has no required curve contract.");

            var curve = _entityManager.GetComponentData<Curve>(entity).m_Bezier;
            actual.Add(new NativeBatchBindings.Edge<Entity>(Target(entity), Target(edge.m_Start), Target(edge.m_End),
                Position(edge.m_Start), Position(edge.m_End), road ? NetworkKind.Road : NetworkKind.Rail, prefab)
            { CurveStart = curve.a, CurveEnd = curve.d, CurveControl1 = curve.b, CurveControl2 = curve.c });
        }

        if (_submittedCourses.Count != batch.Count)
            throw new InvalidOperationException("Submitted native course manifest is incomplete.");

        _stagedCommitted = NativeBatchBindings.RemapCommitted(_committedNodes, replacements);

        // Reconcile only an already-owned node, never a nearby candidate. Native
        // generation may adjust that node again while extending its road.
        for (var i = 0; i < _submittedCourses.Count; i++)
        {
            var item = _submittedCourses[i];
            var section = item.Section;

            NativeBatchBindings.Anchor<Entity>? PreviewAnchor(long id)
            {
                var key = NativeEndpointPolicy.Identity(section, id);

                if (!_stagedCommitted.TryGetValue(key, out var owner)
                    || !previewPositions.TryGetValue(owner, out var position))
                    return null;

                if (!math.all(math.isfinite(position))
                    || math.distance(position.xz, _plannedPositions[key].xz) > _centerBudgets[key])
                    throw new NativePreviewException(
                        "Native owned junction moved beyond its original source footprint.",
                        section.WayId,
                        key);

                return new NativeBatchBindings.Anchor<Entity>(
                    owner,
                    _plannedPositions[key],
                    position)
                { CenterLimit = _centerBudgets[key] };
            }

            var start = PreviewAnchor(section.StartNodeId);
            var end = PreviewAnchor(section.EndNodeId);
            NativeBatchBindings.RebaseOwnedPreview(section, start, end);
            var movement =
                Math.Max(
                    start == null ? 0 : math.distance(section.Start.xz, start.Generated.xz),
                    end == null ? 0 : math.distance(section.End.xz, end.Generated.xz));

            if (movement > 0)
                _log.Diagnostic(
                    $"NATIVE OWNED MOVEMENT way={section.WayId}; centerMovementMeters={movement:R}; submittedCoursePreserved=true; artifact={ArtifactStem}; sourceFootprintChecked=true; unrelatedAttachmentAllowed=false");
        }

        var relevant = new HashSet<Entity>();

        foreach (var edge in actual)
        {
            relevant.Add(edge.StartNode);
            relevant.Add(edge.EndNode);
        }

        foreach (var section in batch)
            foreach (var id in new[] { section.StartNodeId, section.EndNodeId })
                if (_stagedCommitted.TryGetValue(NativeEndpointPolicy.Identity(section, id), out var known))
                    relevant.Add(known);

        _log.DiagnosticRecords(ArtifactStem + "-owners.ndjson", OwnerRecords(relevant));
        _log.DiagnosticRecords(ArtifactStem + "-graph.ndjson", GraphRecords(actual));

        try
        {
            _generatedBatch = NativeBatchBindings.Match(_submittedCourses, actual, _stagedCommitted, MatchRejection);
        }
        catch (Exception error)
        {
            CapturePlacementEvidence(Array.Empty<Entity>(), "rejected");
            _log.DiagnosticRecords(ArtifactStem + "-rejection-endpoints.ndjson", RejectionEndpoints(actual));
            _log.Checkpoint("native-batch/preview-rejected", new
            {
                artifactStem = ArtifactStem,
                sections = batch.Count,
                committedOwners = _committedNodes.Count,
                exception = error.ToString(),
                recoverable = error is NativePreviewException,
                applied = false,
                way = (error as NativePreviewException)?.WayId,
                sources = (error as NativePreviewException)?.Sources,
                endpointEvidence = ArtifactStem + "-rejection-endpoints.ndjson",
                subElevationStructuresOnly = true
            });

            throw;
        }

        var anchors = new Dictionary<string, NativeBatchBindings.Anchor<Entity>>(StringComparer.Ordinal);

        foreach (var pair in _generatedBatch.Anchors)
            anchors[pair.Key] =
                pair.Value with { Planned = _plannedPositions[pair.Key], CenterLimit = _centerBudgets[pair.Key] };

        var required = new Dictionary<Entity, float3>();

        foreach (var pair in _generatedBatch.Nodes)
            required[pair.Key] = pair.Value;

        var stagedEdges = new Dictionary<Entity, NativeBatchBindings.Edge<Entity>>(_verifiedEdges);
        var touched = new List<NativeBatchBindings.Edge<Entity>>();

        Entity RemapNode(Entity node) => replacements.TryGetValue(node, out var next) ? next : node;

        var updatedOriginals = new HashSet<Entity>();
        var actualByEntity = new Dictionary<Entity, NativeBatchBindings.Edge<Entity>>();

        foreach (var edge in actual)
        {
            if (actualByEntity.TryGetValue(edge.Entity, out var previous) && previous != edge)
                throw new NativePreviewException(
                    "Native preview updates disagree on one edge target; regeneration required.",
                    0);

            actualByEntity[edge.Entity] = edge;
        }

        foreach (var entity in temporary)
        {
            var otherTemporary = _entityManager.GetComponentData<Temp>(entity);

            if (_entityManager.HasComponent<Node>(entity) && (otherTemporary.m_Flags & TempFlags.Delete) != 0)
            {
                var original = otherTemporary.m_Original != Entity.Null ? otherTemporary.m_Original : entity;

                foreach (var owner in _committedNodes)
                    if (owner.Value == original && !replacements.ContainsKey(original))
                        throw new NativePreviewException(
                            $"Preview would delete previously verified junction {owner.Key} without replacement lineage.",
                            0,
                            owner.Key);
            }

            if (!_entityManager.HasComponent<Edge>(entity)
                || !_verifiedEdges.TryGetValue(otherTemporary.m_Original, out var prior))
                continue;

            if ((otherTemporary.m_Flags & TempFlags.Delete) != 0)
                continue;

            if (!actualByEntity.TryGetValue(Target(entity), out var update)
                || update.Prefab != prior.Prefab
                || update.Kind != prior.Kind)
                throw PriorEdgeRejection(
                    prior,
                    $"Preview changed previously verified edge {otherTemporary.m_Original} asset or junction incidence; regeneration required.");

            updatedOriginals.Add(otherTemporary.m_Original);
            stagedEdges.Remove(otherTemporary.m_Original);
            stagedEdges[update.Entity] = update;
            touched.Add(update);

            if (otherTemporary.m_Original != update.Entity)
                _stagedEdgeChanges[otherTemporary.m_Original] = update.Entity;

            required[update.StartNode] = update.Start;
            required[update.EndNode] = update.End;
        }

        foreach (var entity in temporary)
        {
            if (!_entityManager.HasComponent<Edge>(entity))
                continue;

            var otherTemporary = _entityManager.GetComponentData<Temp>(entity);

            if ((otherTemporary.m_Flags & TempFlags.Delete) != 0
                && _verifiedEdges.ContainsKey(otherTemporary.m_Original)
                && !updatedOriginals.Contains(otherTemporary.m_Original))
                stagedEdges.Remove(otherTemporary.m_Original);
        }

        // Complete overlay: Combine may create a replacement edge without a
        // one-to-one original, including subdivisions not consumed by this batch.
        // Omitting those edges manufactures missing prior routes.
        foreach (var edge in actual)
        {
            stagedEdges[edge.Entity] = edge;
            required[edge.StartNode] = edge.Start;
            required[edge.EndNode] = edge.End;

            if (!System.Linq.Enumerable.Any(touched, otherEdge => otherEdge.Entity == edge.Entity))
                touched.Add(edge);
        }

        var routeOwners = new Dictionary<string, Entity>(_stagedCommitted, StringComparer.Ordinal);

        foreach (var anchor in _generatedBatch.Anchors)
            routeOwners[anchor.Key] = anchor.Value.Entity;

        foreach (var key in new List<Entity>(stagedEdges.Keys))
        {
            var edge = stagedEdges[key];
            var start = RemapNode(edge.StartNode);
            var end = RemapNode(edge.EndNode);
            stagedEdges[key] = edge with
            {
                StartNode = start,
                EndNode = end,
                Start = previewPositions.TryGetValue(start, out var vectorA) ? vectorA : edge.Start,
                End = previewPositions.TryGetValue(end, out var vectorB) ? vectorB : edge.End
            };
        }

        // Prove whole source-route coverage for EVERY changed/deleted original.
        // Native subdivision boundaries may disappear; source junctions may not.
        var changedOriginals = new HashSet<Entity>();

        foreach (var entity in temporary)
            if (_entityManager.HasComponent<Edge>(entity))
            {
                var original = _entityManager.GetComponentData<Temp>(entity).m_Original;

                if (_verifiedEdges.ContainsKey(original))
                    changedOriginals.Add(original);
            }

        var changedNodes = new HashSet<Entity>(replacements.Keys);

        foreach (var edge in _verifiedEdges.Values)
        {
            var start = RemapNode(edge.StartNode);
            var end = RemapNode(edge.EndNode);

            if (previewPositions.TryGetValue(start, out var vectorA) && !math.all(vectorA == edge.Start))
                changedNodes.Add(edge.StartNode);

            if (previewPositions.TryGetValue(end, out var vectorB) && !math.all(vectorB == edge.End))
                changedNodes.Add(edge.EndNode);
        }

        foreach (var path in _verifiedPaths)
        {
            if (!System.Linq.Enumerable.Any(path.Value, entity => changedOriginals.Contains(entity)
                || _verifiedEdges.TryGetValue(entity, out var previous)
                    && (changedNodes.Contains(previous.StartNode) || changedNodes.Contains(previous.EndNode))))
                continue;

            if (!_sourceRoutes.TryGetValue(path.Key, out var source))
                throw PriorEdgeRejection(
                    _verifiedEdges[path.Value[0]],
                    "Prior source route manifest unavailable; combination refused.");

            NativeBatchBindings.Anchor<Entity> Owned(long id, float3 planned)
            {
                var key = NativeEndpointPolicy.Identity(source, id);
                var owner = _stagedCommitted[key];
                var position =
                    previewPositions.TryGetValue(owner, out var point) ? point : _committedAnchors[key].Generated;

                if (math.distance(position.xz, _plannedPositions[key].xz) > _centerBudgets[key])
                    throw new NativePreviewException(
                        "Combined route junction exceeded original footprint.",
                        source.WayId,
                        key);

                return new(owner, planned, position) { CenterLimit = _centerBudgets[key] };
            }

            // Validate source-owner center movement independently of ports. A
            // junction update may change its native curve while preserving its
            // owned, directed route. Never translate the old prepared evidence.
            Owned(source.StartNodeId, source.Start);
            Owned(source.EndNodeId, source.End);
            var reconciled = source;
            var tolerance =
                Math.Max(
                    _centerBudgets[NativeEndpointPolicy.Identity(source, source.StartNodeId)],
                    _centerBudgets[NativeEndpointPolicy.Identity(source, source.EndNodeId)]);

            var prefab = _verifiedEdges[path.Value[0]].Prefab;
            var low =
                math.min(
                    math.min(reconciled.CourseStart.xz, reconciled.CourseEnd.xz),
                    math.min(reconciled.CourseControl1.xz, reconciled.CourseControl2.xz))
                    - new float2(tolerance + 32);

            var high =
                math.max(
                    math.max(reconciled.CourseStart.xz, reconciled.CourseEnd.xz),
                    math.max(reconciled.CourseControl1.xz, reconciled.CourseControl2.xz))
                    + new float2(tolerance + 32);

            var candidates = new List<NativeBatchBindings.Edge<Entity>>();

            foreach (var edge in stagedEdges.Values)
                if (edge.Kind == source.Kind
                    && edge.Prefab == prefab
                    && math.all(edge.Start.xz >= low)
                    && math.all(edge.Start.xz <= high)
                    && math.all(edge.End.xz >= low)
                    && math.all(edge.End.xz <= high))
                    candidates.Add(edge);

            NativeBatchBindings.Batch<Entity> route;

            try
            {
                route =
                    NativeBatchBindings.Match(
                        new[] {
                            new NativeBatchBindings.Expected<Entity>(
                                reconciled,
                                prefab) {
                                    StartCenterBudget =
                                        _centerBudgets[NativeEndpointPolicy.Identity(source, source.StartNodeId)],
                                    EndCenterBudget =
                                        _centerBudgets[NativeEndpointPolicy.Identity(source, source.EndNodeId)],
                                    ReconciliationTolerance = tolerance,
                                    ValidateCurveCorridor = true,
                                    OwnedRoute = true
                                    }
                            },
                        candidates,
                        routeOwners,
                        MatchRejection);
            }
            catch (NativePreviewException)
            {
                // Complete candidate controls are needed on failure, not tens
                // of thousands of graph files during successful imports.
                _log.DiagnosticRecords(
                    ArtifactStem
                        + "-prior-route-"
                        + source.WayId
                        + "-"
                        + source.StartNodeId
                        + "-"
                        + source.EndNodeId
                        + ".ndjson",
                    GraphRecords(candidates));

                throw;
            }

            _stagedPaths[path.Key] =
                new List<Entity>(System.Linq.Enumerable.Select(route.Edges, binding => binding.Generated.Entity));

            foreach (var pair in route.Nodes)
                required[pair.Key] = pair.Value;

            foreach (var binding in route.Edges)
                if (!System.Linq.Enumerable.Any(touched, edge => edge.Entity == binding.Generated.Entity))
                    touched.Add(binding.Generated);

            var first = route.Edges[0];
            var last = route.Edges[route.Edges.Count - 1];
            var firstPort =
                first.Forward
                    ? first.Generated.CurveStart ?? first.Generated.Start
                    : first.Generated.CurveEnd ?? first.Generated.End;

            var lastPort =
                last.Forward
                    ? last.Generated.CurveEnd ?? last.Generated.End
                    : last.Generated.CurveStart ?? last.Generated.Start;

            var portMovement =
                Math.Max(
                    math.distance(firstPort.xz, source.CourseStart.xz),
                    math.distance(lastPort.xz, source.CourseEnd.xz));

            var heightMovement =
                Math.Max(Math.Abs(firstPort.y - source.CourseStart.y), Math.Abs(lastPort.y - source.CourseEnd.y));

            _log.Count("Native owned routes requalified");
            _log.Diagnostic(
                $"NATIVE ROUTE REQUALIFIED way={source.WayId}; previousEdges={path.Value.Count}; replacementEdges={route.Edges.Count}; sourceOwnersRetained=true; exactAsset=true; uniqueConnectedCoverage=true; ownedRoute=true; geometricEnvelopeMeters={tolerance:R}; endpointPortMovementMeters={portMovement:R}; endpointHeightMovementMeters={heightMovement:R}; structuralTerrainClearanceCertified=false; immutableSourceEvidence=true; artifact={ArtifactStem}");
        }

        // Retire anonymous subdivision nodes eliminated by an approved Combine.
        var retainedNodes = new HashSet<Entity>();

        foreach (var edge in stagedEdges.Values)
        {
            retainedNodes.Add(edge.StartNode);
            retainedNodes.Add(edge.EndNode);
        }

        foreach (var node in new List<Entity>(required.Keys))
            if (!retainedNodes.Contains(node) && !System.Linq.Enumerable.Contains(_stagedCommitted.Values, node))
                required.Remove(node);

        _stagedEdges = stagedEdges;
        _touchedEdges = touched;

        // A preview can regenerate nodes from earlier verified batches. Retain
        // their center evidence too, not just anchors present in this batch.
        foreach (var pair in _stagedCommitted)
        {
            if (!previewPositions.TryGetValue(pair.Value, out var point))
                continue;

            var prior = _committedAnchors[pair.Key];

            if (!math.all(math.isfinite(point)) || math.distance(point.xz, prior.Planned.xz) > _centerBudgets[pair.Key])
                throw new NativePreviewException(
                    $"Previously bound junction {pair.Key} moved outside bounded native footprint.",
                    0,
                    pair.Key);

            required[pair.Value] = point;
        }

        _generatedBatch = _generatedBatch with { Anchors = anchors, Nodes = required };
        var changed = 0;
        var maximum = 0f;

        foreach (var anchor in _generatedBatch.Anchors.Values)
        {
            var delta = Math.Abs(anchor.Generated.y - anchor.Planned.y);

            if (delta > .001f)
                changed++;

            maximum = Math.Max(maximum, delta);
        }

        _log.Count("Native ground anchor height adjustments", changed);
        _log.Diagnostic(
            $"NATIVE BATCH artifact={ArtifactStem}; generatedBindings={_generatedBatch.Anchors.Count}; sourceSectionsMatched={_generatedBatch.SectionCount}/{batch.Count}; nativeEdgesMatched={_generatedBatch.Edges.Count}; nativeSubdivisionNodes={_generatedBatch.Nodes.Count - _generatedBatch.Anchors.Count}; heightAdjusted={changed}; maximumHeightDeltaMeters={maximum:R}; exactCurvePortsAndAsset=true; entityLineage=true; nativeCoverageStillPending=true");

        _log.DiagnosticRecords("native-batch-bindings.ndjson", BindingRecords(_generatedBatch));
        _log.DiagnosticRecords(ArtifactStem + "-bindings.ndjson", BindingRecords(_generatedBatch));
    }

    private IEnumerable<object> OwnerRecords(HashSet<Entity> relevant)
    {
        foreach (var pair in _committedAnchors)
        {
            var entity = _stagedCommitted![pair.Key];

            if (!relevant.Contains(entity))
                continue;

            yield return new
            {
                source = pair.Key,
                entity = $"{entity.Index}:{entity.Version}",
                previousEntity = $"{pair.Value.Entity.Index}:{pair.Value.Entity.Version}",
                planned = DiagnosticArtifacts.Point(pair.Value.Planned),
                generated = DiagnosticArtifacts.Point(pair.Value.Generated),
                pair.Value.CenterLimit
            };
        }
    }

    private IEnumerable<object> ExpectedRecords()
    {
        foreach (var item in _submittedCourses)
            yield return new
            {
                kind = "expected-course",
                prefab = $"{item.Prefab.Index}:{item.Prefab.Version}",
                item.StartCenterBudget,
                item.EndCenterBudget,
                disableOverlapMergeBothEnds = true,
                creationFlags = (NetworkStructure.IsStructure(item.Section)
                    ? CreationFlags.SubElevation
                    : 0).ToString(),
                section = System.Linq.Enumerable.First(DiagnosticArtifacts.NetworkRecords(new[] { item.Section }))
            };
    }

    private IEnumerable<object> RejectionEndpoints(IReadOnlyList<NativeBatchBindings.Edge<Entity>> graph)
    {
        foreach (var item in _submittedCourses)
            foreach (var endpoint in new[]{
                (
                    id:item.Section.StartNodeId,
                    port:item.Section.CourseStart,
                    center:item.Section.Start,
                    budget:item.StartCenterBudget),
                                      (
                                          id:item.Section.EndNodeId,
                                          port:item.Section.CourseEnd,
                                          center:item.Section.End,
                                          budget:item.EndCenterBudget)})
            {
                var source = NativeEndpointPolicy.Identity(item.Section, endpoint.id);
                var known = _stagedCommitted!.TryGetValue(source, out var owner);
                var candidates = new List<object>();

                foreach (var edge in graph)
                {
                    if (edge.Kind != item.Section.Kind || edge.Prefab != item.Prefab)
                        continue;

                    foreach (var end in new[] {
                        (node: edge.StartNode, center: edge.Start, port: edge.CurveStart ?? edge.Start),
                        (node: edge.EndNode, center: edge.End, port: edge.CurveEnd ?? edge.End)
                        })
                    {
                        // Spatial distance is diagnostic ONLY, never permission to join.
                        var delta = math.distance(end.port.xz, endpoint.port.xz);

                        if ((known && end.node == owner) || delta <= endpoint.budget)
                            candidates.Add(new
                            {
                                node = $"{end.node.Index}:{end.node.Version}",
                                edge = $"{edge.Entity.Index}:{edge.Entity.Version}",
                                ownerMatches = known && end.node == owner,
                                curvePortDelta = delta,
                                centerDelta = math.distance(end.center.xz, endpoint.center.xz),
                                centerToPortDelta = math.distance(end.center.xz, end.port.xz),
                                center = DiagnosticArtifacts.Point(end.center),
                                port = DiagnosticArtifacts.Point(end.port)
                            });

                        if (candidates.Count >= 24)
                            break;
                    }

                    if (candidates.Count >= 24)
                        break;
                }

                yield return new
                {
                    source,
                    item.Section.WayId,
                    knownOwner = known ? $"{owner.Index}:{owner.Version}" : null,
                    submittedPort = DiagnosticArtifacts.Point(endpoint.port),
                    submittedCenter = DiagnosticArtifacts.Point(endpoint.center),
                    endpoint.budget,
                    candidates,
                    truncated = candidates.Count >= 24,
                    proximityAttachmentAllowed = false
                };
            }
    }

    private static IEnumerable<object> GraphRecords(IReadOnlyList<NativeBatchBindings.Edge<Entity>> graph)
    {
        foreach (var edge in graph)
            yield return new
            {
                entity = $"{edge.Entity.Index}:{edge.Entity.Version}",
                startNode = $"{edge.StartNode.Index}:{edge.StartNode.Version}",
                endNode = $"{edge.EndNode.Index}:{edge.EndNode.Version}",
                start = DiagnosticArtifacts.Point(edge.Start),
                end = DiagnosticArtifacts.Point(edge.End),
                mode = edge.Kind.ToString(),
                prefab = $"{edge.Prefab.Index}:{edge.Prefab.Version}",
                curveStart = edge.CurveStart.HasValue ? DiagnosticArtifacts.Point(edge.CurveStart.Value) : null,
                curveEnd = edge.CurveEnd.HasValue ? DiagnosticArtifacts.Point(edge.CurveEnd.Value) : null,
                control1 = edge.CurveControl1.HasValue ? DiagnosticArtifacts.Point(edge.CurveControl1.Value) : null,
                control2 = edge.CurveControl2.HasValue ? DiagnosticArtifacts.Point(edge.CurveControl2.Value) : null
            };
    }

    private static object CurveEvidence(Bezier4x3 curve)
        => new
        {
            a = DiagnosticArtifacts.Point(curve.a),
            b = DiagnosticArtifacts.Point(curve.b),
            c = DiagnosticArtifacts.Point(curve.c),
            d = DiagnosticArtifacts.Point(curve.d)
        };

    private void MatchRejection(object evidence)
    {
        _log.Count("Native matcher rejected attempts", 1);
        _log.DiagnosticRecords(ArtifactStem + "-matcher-rejection.ndjson", new[] { evidence });
    }

    private static IEnumerable<object> BindingRecords(NativeBatchBindings.Batch<Entity> batch)
    {
        foreach (var pair in batch.Anchors)
            yield return new
            {
                kind = "source-anchor",
                source = pair.Key,
                entity = NetworkCommitSystem.EntityId(pair.Value.Entity),
                planned = DiagnosticArtifacts.Point(pair.Value.Planned),
                generated = DiagnosticArtifacts.Point(pair.Value.Generated),
                heightDelta = pair.Value.Generated.y - pair.Value.Planned.y,
                horizontalCenterDelta = math.distance(pair.Value.Generated.xz, pair.Value.Planned.xz),
                pair.Value.CenterLimit
            };

        foreach (var edge in batch.Edges)
            yield return new
            {
                kind = "native-chain-edge",
                edge.Section.WayId,
                edge.Section.StartNodeId,
                edge.Section.EndNodeId,
                mode = edge.Section.Kind.ToString(),
                entity = NetworkCommitSystem.EntityId(edge.Generated.Entity),
                start = NetworkCommitSystem.EntityId(edge.Generated.StartNode),
                end = NetworkCommitSystem.EntityId(edge.Generated.EndNode),
                edge.Forward,
                prefab = NetworkCommitSystem.EntityId(edge.Generated.Prefab)
            };
    }

    public bool TryCaptureCommittedNodes(out string pending)
    {
        if (_advisoryBatch)
        {
            PublishAdvisoryOwners();
            pending = "";

            return true;
        }

        try
        {
            return TryCaptureQualifiedNodes(out pending);
        }
        catch (Exception error) when (error is not OutOfMemoryException && error is not OperationCanceledException)
        {
            AcceptUnverifiedPlayback(error.ToString());
            pending = "";

            return true;
        }
    }

    private bool TryCaptureQualifiedNodes(out string pending)
    {
        _entityManager.CompleteAllTrackedJobs();

        if (_generatedBatch == null)
            throw new InvalidOperationException("Native source bindings were not captured before Apply.");

        foreach (var pair in _stagedCommitted!)
        {
            if (_committedNodes[pair.Key] == pair.Value)
                continue;

            if (!_entityManager.HasComponent<Node>(pair.Value)
                || _entityManager.HasComponent<Temp>(pair.Value)
                || _entityManager.HasComponent<Deleted>(pair.Value)
                || !_entityManager.HasBuffer<ConnectedEdge>(pair.Value))
            {
                pending = $"source={pair.Key}; replacement node not ready";
                return false;
            }
        }

        var changedCenters = 0;
        var centerChanges = new Dictionary<Entity, (float3 Preview, float3 Live)>();

        foreach (var pair in _generatedBatch.Nodes)
        {
            var entity = pair.Key;

            if (!_entityManager.HasComponent<Node>(entity)
                || _entityManager.HasComponent<Temp>(entity)
                || _entityManager.HasComponent<Deleted>(entity)
                || !_entityManager.HasBuffer<ConnectedEdge>(entity))
            {
                pending = $"entity={entity}; source/subdivision node/buffer not ready";
                return false;
            }

            var point = _entityManager.GetComponentData<Node>(entity).m_Position;

            if (!math.all(point.xz == pair.Value.xz))
            {
                _log.Diagnostic(
                    $"NATIVE LIVE CENTER entity={entity}; preview={pair.Value}; live={point}; horizontalDeltaMeters={math.distance(point.xz, pair.Value.xz):R}; numericPlaybackOnly={NativeBatchBindings.PlaybackHorizontal(pair.Value, point)}; ownershipAndIncidenceValidationStillPending=true");

                changedCenters++;
            }

            if (!NativeBatchBindings.PlaybackHorizontal(pair.Value, point))
            {
                centerChanges[entity] = (pair.Value, point);
            }
        }

        foreach (var binding in _generatedBatch.Edges)
        {
            var edge = binding.Generated;

            if (!_entityManager.HasComponent<Edge>(edge.Entity)
                || _entityManager.HasComponent<Temp>(edge.Entity)
                || _entityManager.HasComponent<Deleted>(edge.Entity))
            {
                pending = $"way={binding.Section.WayId}; edge={edge.Entity}; live edge not ready";
                return false;
            }

            if (!_entityManager.HasComponent<PrefabRef>(edge.Entity))
            {
                pending = $"way={binding.Section.WayId}; edge={edge.Entity}; live asset not ready";
                return false;
            }

            if (_entityManager.GetComponentData<PrefabRef>(edge.Entity).m_Prefab != edge.Prefab)
                throw new InvalidOperationException(
                    $"Native live asset changed for way {binding.Section.WayId}; unsafe continuation refused.");

            var live = _entityManager.GetComponentData<Edge>(edge.Entity);

            if (live.m_Start != edge.StartNode || live.m_End != edge.EndNode)
                throw new InvalidOperationException(
                    $"Native edge lineage changed for way {binding.Section.WayId}; unsafe continuation refused.");

            bool Incident(Entity node)
            {
                foreach (var otherEdge in _entityManager.GetBuffer<ConnectedEdge>(node, true))
                    if (otherEdge.m_Edge == edge.Entity)
                        return true;

                return false;
            }

            if (!Incident(edge.StartNode) || !Incident(edge.EndNode))
            {
                pending = $"way={binding.Section.WayId}; edge={edge.Entity}; incident buffers not ready";
                return false;
            }
        }

        foreach (var edge in _touchedEdges)
        {
            if (!_entityManager.HasComponent<Edge>(edge.Entity)
                || _entityManager.HasComponent<Temp>(edge.Entity)
                || _entityManager.HasComponent<Deleted>(edge.Entity)
                || !_entityManager.HasComponent<PrefabRef>(edge.Entity))
            {
                pending = $"previous edge={edge.Entity}; regenerated live edge not ready";
                return false;
            }

            var live = _entityManager.GetComponentData<Edge>(edge.Entity);

            if (live.m_Start != edge.StartNode
                || live.m_End != edge.EndNode
                || _entityManager.GetComponentData<PrefabRef>(edge.Entity).m_Prefab != edge.Prefab)
                throw new InvalidOperationException(
                    "Previously verified native edge changed lineage or asset during playback.");

            bool Incident(Entity node)
            {
                foreach (var otherEdge in _entityManager.GetBuffer<ConnectedEdge>(node, true))
                    if (otherEdge.m_Edge == edge.Entity)
                        return true;

                return false;
            }

            if (!Incident(edge.StartNode) || !Incident(edge.EndNode))
            {
                pending = $"previous edge={edge.Entity}; regenerated incident buffers not ready";
                return false;
            }
        }

        foreach (var changed in centerChanges)
        {
            if (!RequalifySourceCenter(changed.Key, changed.Value.Live, out var reason))
            {
                // Buffer/component publication may trail Apply playback. Keep
                // waiting through the existing bounded live-readback phase;
                // never replay Apply to repair an incomplete snapshot.
                if (reason == "missing-live-incidence" || reason == "incident-edge-not-live")
                {
                    pending = $"source center={changed.Key}; {reason}";
                    return false;
                }

                _log.DiagnosticRecords(
                    ArtifactStem + "-failed-live.ndjson",
                    FailedLiveRecords(changed.Key, changed.Value.Preview, changed.Value.Live));

                throw new InvalidOperationException(
                    $"Native live center {changed.Key} could not be requalified: {reason}; actual/preview evidence saved.");
            }

            _log.Diagnostic(
                $"NATIVE LIVE CENTER entity={changed.Key}; decision=source-center-requalified; reason={reason}; actualCenterPublishedAtomically=true; anonymousSubdivisionAllowanceUnchanged=true; noApplyRetry=true");
        }

        // Publish all anchors together, only after every required live edge is present.
        foreach (var pair in _generatedBatch.Anchors)
        {
            var point = _entityManager.GetComponentData<Node>(pair.Value.Entity).m_Position;

            if (!NativeBatchBindings.PlaybackHorizontal(pair.Value.Generated, point)
                && !centerChanges.ContainsKey(pair.Value.Entity))
                throw new InvalidOperationException(
                    $"Native source anchor {pair.Key} no longer matches its verified preview center or became non-finite.");
        }

        if (changedCenters > 0)
        {
            foreach (var edge in System.Linq.Enumerable.Concat(
                System.Linq.Enumerable.Select(_generatedBatch.Edges, binding => binding.Generated),
                _touchedEdges))
            {
                if (!_entityManager.HasComponent<Curve>(edge.Entity))
                    throw new InvalidOperationException("Live roundoff validation lacks native curve evidence.");

                var curve = _entityManager.GetComponentData<Curve>(edge.Entity).m_Bezier;

                if (!edge.CurveStart.HasValue
                    || !edge.CurveEnd.HasValue
                    || !NativeBatchBindings.PlaybackHorizontal(edge.CurveStart.Value, curve.a)
                    || !NativeBatchBindings.PlaybackHorizontal(edge.CurveEnd.Value, curve.d))
                    throw new InvalidOperationException(
                        "Native live curve ports changed beyond numeric playback roundoff; unsafe attachment refused.");
            }

            _log.Diagnostic(
                $"NATIVE PLAYBACK numericRoundedCenters={changedCenters - centerChanges.Count}; sourceRequalifiedCenters={centerChanges.Count}; curvePortAllowance=2-float-ULP-capped-.01m; sourceCenterAllowance=established-preview-footprint; liveCurvePortsVerified=true; assetsAndIncidenceVerified=true; proximityAttachment=false");
        }

        _verifiedEdges = _stagedEdges!;

        foreach (var pair in _verifiedEdges)
            _observedEdges[pair.Key] = pair.Value;

        LastBatchSourceVerified = true;

        foreach (var path in _stagedPaths)
            _verifiedPaths[path.Key] = path.Value;

        if (_stagedEdgeChanges.Count > 0)
            foreach (var path in _verifiedPaths.Values)
                for (var i = 0; i < path.Count; i++)
                    if (_stagedEdgeChanges.TryGetValue(path[i], out var replacement))
                        path[i] = replacement;

        foreach (var group in System.Linq.Enumerable.GroupBy(
            _generatedBatch.Edges,
            binding => (
                binding.Section.Kind,
                binding.Section.WayId,
                binding.Section.StartNodeId,
                binding.Section.EndNodeId)))
            _verifiedPaths[group.Key] =
                new List<Entity>(System.Linq.Enumerable.Select(group, binding => binding.Generated.Entity));

        NativeNetworkDiagnostics.BindVerified(_committedNodes, _verifiedPaths);
        NativeNetworkDiagnostics.BindObserved(_observedEdges, _edgeStations);

        foreach (var pair in _stagedCommitted!)
        {
            _committedNodes[pair.Key] = pair.Value;
            var prior = _committedAnchors[pair.Key];
            _committedAnchors[pair.Key] =
                prior with
                {
                    Entity = pair.Value,
                    Generated = _entityManager.GetComponentData<Node>(pair.Value).m_Position
                };
        }

        foreach (var pair in _generatedBatch.Anchors)
        {
            _committedNodes[pair.Key] = pair.Value.Entity;
            _committedAnchors[pair.Key] =
                pair.Value with { Generated = _entityManager.GetComponentData<Node>(pair.Value.Entity).m_Position };
        }

        foreach (var section in System.Linq.Enumerable.Distinct(
            System.Linq.Enumerable.Select(_generatedBatch.Edges, binding => binding.Section)))
            if (section.Kind == NetworkKind.Road)
                RoadSections++;
            else
                RailSections++;

        _log.Count("Native source sections verified", _generatedBatch.SectionCount);
        _log.Count("Native chain edges verified", _generatedBatch.Edges.Count);
        _log.DiagnosticRecords("native-live-batch-bindings.ndjson", LiveBindingRecords(_generatedBatch));
        _log.DiagnosticRecords(ArtifactStem + "-live.ndjson", LiveBindingRecords(_generatedBatch));

        if (_log.DiagnosticsEnabled)
            _log.DiagnosticRecords(
                ArtifactStem + "-live-constraint-observations.ndjson",
                LiveConstraintRecords(_generatedBatch));

        _log.Diagnostic(
            $"NATIVE BATCH artifact={ArtifactStem}; anchorsVerified={_generatedBatch.Anchors.Count}; sourceSectionsVerified={_generatedBatch.SectionCount}; nativeEdgesVerified={_generatedBatch.Edges.Count}; accumulatedSourceNodes={_committedNodes.Count}; numericPreviewCenterAndExactAsset=true; entityLineage=true; incidentBuffersReady=true");

        CaptureSettledReadback();
        _generatedBatch = null;
        _stagedCommitted = null;
        _stagedEdges = null;
        _touchedEdges = Array.Empty<NativeBatchBindings.Edge<Entity>>();
        pending = "";

        return true;
    }

    private void CaptureSettledReadback(
        IReadOnlyList<(NetworkKind Kind, long WayId, long Start, long End)>? invalidated = null)
    {
        if (!_log.DiagnosticsEnabled)
            return;

        try
        {
            var scope = new HashSet<Entity>(_pendingObservedEdges.Keys);

            foreach (var edge in _touchedEdges)
                scope.Add(edge.Entity);

            foreach (var edge in _advisoryMutationEdges)
                scope.Add(edge);

            if (_generatedBatch != null)
                foreach (var binding in _generatedBatch.Edges)
                    scope.Add(binding.Generated.Entity);

            var nodes = new HashSet<Entity>();

            foreach (var entity in new List<Entity>(scope))
                if (_entityManager.HasComponent<Edge>(entity))
                {
                    var edge = _entityManager.GetComponentData<Edge>(entity);
                    nodes.Add(edge.m_Start);
                    nodes.Add(edge.m_End);
                }

            foreach (var node in nodes)
                if (_entityManager.HasBuffer<ConnectedEdge>(node))
                    foreach (var edge in _entityManager.GetBuffer<ConnectedEdge>(node, true))
                        scope.Add(edge.m_Edge);

            var sections = new Dictionary<(NetworkKind Kind, long WayId, long Start, long End), PreparedSegment>();

            foreach (var item in _submittedCourses)
            {
                var segment = item.Section;
                sections[(segment.Kind, segment.WayId, segment.StartNodeId, segment.EndNodeId)] = segment;
            }

            foreach (var path in _verifiedPaths)
                if (path.Value.Exists(scope.Contains) && _sourceRoutes.TryGetValue(path.Key, out var segment))
                {
                    sections[path.Key] = segment;

                    foreach (var edge in path.Value)
                        scope.Add(edge);
                }

            if (invalidated != null)
                foreach (var key in invalidated)
                    if (_sourceRoutes.TryGetValue(key, out var segment))
                        sections[key] = segment;

            _log.Diagnostic(
                $"NATIVE SETTLED SCOPE artifact={ArtifactStem}; sourceSections={sections.Count}; scopedEdges={scope.Count}; incidentNodes={nodes.Count}; affectedPriorRoutesIncluded=true; worldEdgeScan=false; worldLaneScan=false; nearestSourceMatching=false; completeWholeJunctionAcceptance=false; qualityOutcome=RequiresReview");

            NativeNetworkReadback.Check(_entityManager, new List<PreparedSegment>(sections.Values), _evidence, _log,
                includeMovements: true, scope: new List<Entity>(scope), artifactStem: ArtifactStem + "-settled");
        }
        catch (Exception error) when (error is not OutOfMemoryException && error is not OperationCanceledException)
        {
            _log.Diagnostic(
                $"NATIVE SETTLED READBACK unavailable artifact={ArtifactStem}; exception={error}; qualityOutcome=RequiresReview; duplicateApply=false");
        }
    }

    private NativePreviewException PriorEdgeRejection(NativeBatchBindings.Edge<Entity> edge, string message)
    {
        var sources = new List<string>();

        foreach (var pair in _committedNodes)
            if (pair.Value == edge.StartNode || pair.Value == edge.EndNode)
                sources.Add(pair.Key);

        long way = 0;

        foreach (var pair in _verifiedPaths)
            if (pair.Value.Contains(edge.Entity))
            {
                way = pair.Key.WayId;
                break;
            }

        foreach (var path in _verifiedPaths)
            if (path.Value.Contains(edge.Entity))
            {
                var start = $"{path.Key.Kind}:{path.Key.Start}";
                var end = $"{path.Key.Kind}:{path.Key.End}";

                if (!sources.Contains(start))
                    sources.Add(start);

                if (!sources.Contains(end))
                    sources.Add(end);
            }

        _log.Diagnostic(
            $"NATIVE PRIOR EDGE rejected={edge.Entity}; way={way}; sources={string.Join(",", sources)}; expectedStart={edge.StartNode}; expectedEnd={edge.EndNode}; expectedPrefab={edge.Prefab}; pendingJunctionRecoveryHasContext={sources.Count > 0}");

        return new NativePreviewException(message, way, sources.ToArray());
    }

    private bool RequalifySourceCenter(Entity entity, float3 live, out string reason)
    {
        reason = "no-unique-source-owner";

        if (!math.all(math.isfinite(live)))
        {
            reason = "non-finite-center";
            return false;
        }

        var owners = new Dictionary<string, NativeBatchBindings.Anchor<Entity>>(StringComparer.Ordinal);

        foreach (var pair in _stagedCommitted!)
            if (pair.Value == entity)
                owners[pair.Key] = _committedAnchors[pair.Key];

        foreach (var pair in _generatedBatch!.Anchors)
            if (pair.Value.Entity == entity)
                owners[pair.Key] = pair.Value;

        if (owners.Count != 1)
            return false;

        NativeBatchBindings.Anchor<Entity>? anchor = null;
        string source = "";

        foreach (var pair in owners)
        {
            anchor = pair.Value;
            source = pair.Key;
        }

        if (anchor == null
            || !math.isfinite(anchor.CenterLimit)
            || anchor.CenterLimit <= 0
            || anchor.CenterLimit > 32
            || math.distance(live.xz, anchor.Planned.xz) > anchor.CenterLimit)
        {
            reason = "outside-established-source-footprint";
            return false;
        }

        var expected = new Dictionary<Entity, NativeBatchBindings.Edge<Entity>>();

        foreach (var pair in _stagedEdges!)
            if (pair.Value.StartNode == entity || pair.Value.EndNode == entity)
                expected[pair.Key] = pair.Value;

        if (expected.Count == 0)
        {
            reason = "no-qualified-incident-edges";
            return false;
        }

        var observed = new HashSet<Entity>();

        foreach (var link in _entityManager.GetBuffer<ConnectedEdge>(entity, true))
        {
            if (!observed.Add(link.m_Edge) || !expected.TryGetValue(link.m_Edge, out var otherEdge))
            {
                reason = "extra-or-duplicate-live-incidence";
                return false;
            }

            if (!_entityManager.HasComponent<Edge>(otherEdge.Entity)
                || _entityManager.HasComponent<Temp>(otherEdge.Entity)
                || _entityManager.HasComponent<Deleted>(otherEdge.Entity)
                || !_entityManager.HasComponent<PrefabRef>(otherEdge.Entity)
                    || !_entityManager.HasComponent<Curve>(otherEdge.Entity))
            {
                reason = "incident-edge-not-live";
                return false;
            }

            var edge = _entityManager.GetComponentData<Edge>(otherEdge.Entity);
            var curve = _entityManager.GetComponentData<Curve>(otherEdge.Entity).m_Bezier;

            if (edge.m_Start != otherEdge.StartNode
                || edge.m_End != otherEdge.EndNode
                || _entityManager.GetComponentData<PrefabRef>(otherEdge.Entity).m_Prefab != otherEdge.Prefab)
            {
                reason = "incident-lineage-or-asset-changed";
                return false;
            }

            if (!otherEdge.CurveStart.HasValue
                || !otherEdge.CurveEnd.HasValue
                || !NativeBatchBindings.PlaybackHorizontal(otherEdge.CurveStart.Value, curve.a)
                || !NativeBatchBindings.PlaybackHorizontal(otherEdge.CurveEnd.Value, curve.d))
            {
                reason = "incident-curve-ports-changed";
                return false;
            }

            var port = entity == otherEdge.StartNode ? curve.a : curve.d;

            if (math.distance(live.xz, port.xz) > anchor.CenterLimit)
            {
                reason = "outside-incident-port-footprint";
                return false;
            }
        }

        if (observed.Count != expected.Count)
        {
            reason = "missing-live-incidence";
            return false;
        }

        reason =
            $"owner={source}; completeIncidentEdges={expected.Count}; establishedCenterLimit={anchor.CenterLimit:R}; curvePortsUnchanged=true";

        return true;
    }

    private IEnumerable<object> FailedLiveRecords(Entity entity, float3 preview, float3 live)
    {
        yield return new
        {
            kind = "node",
            entity = $"{entity.Index}:{entity.Version}",
            preview = DiagnosticArtifacts.Point(preview),
            live = DiagnosticArtifacts.Point(live),
            horizontalDelta = math.distance(preview.xz, live.xz)
        };

        if (_generatedBatch != null)
            foreach (var owner in _generatedBatch.Anchors)
                if (owner.Value.Entity == entity)
                    yield return new
                    {
                        kind = "source-owner",
                        source = owner.Key,
                        planned = DiagnosticArtifacts.Point(owner.Value.Planned),
                        owner.Value.CenterLimit
                    };

        foreach (var owner in _committedAnchors)
            if (owner.Value.Entity == entity)
                yield return new
                {
                    kind = "previous-owner",
                    source = owner.Key,
                    planned = DiagnosticArtifacts.Point(owner.Value.Planned),
                    owner.Value.CenterLimit
                };

        if (!_entityManager.HasBuffer<ConnectedEdge>(entity))
            yield break;

        foreach (var incident in _entityManager.GetBuffer<ConnectedEdge>(entity, true))
        {
            var otherEntity = incident.m_Edge;

            if (!_entityManager.HasComponent<Edge>(otherEntity))
                continue;

            var edge = _entityManager.GetComponentData<Edge>(otherEntity);
            var prefab =
                _entityManager.HasComponent<PrefabRef>(otherEntity)
                    ? _entityManager.GetComponentData<PrefabRef>(otherEntity).m_Prefab
                    : Entity.Null;

            var curve =
                _entityManager.HasComponent<Curve>(otherEntity)
                    ? _entityManager.GetComponentData<Curve>(otherEntity).m_Bezier
                    : default;

            yield return new
            {
                kind = "incident-edge",
                entity = $"{otherEntity.Index}:{otherEntity.Version}",
                start = edge.m_Start.ToString(),
                end = edge.m_End.ToString(),
                prefab = prefab.ToString(),
                curveStart = DiagnosticArtifacts.Point(curve.a),
                curveEnd = DiagnosticArtifacts.Point(curve.d),
                curveAvailable = _entityManager.HasComponent<Curve>(otherEntity)
            };
        }
    }

    private IEnumerable<object> LiveBindingRecords(NativeBatchBindings.Batch<Entity> batch)
    {
        foreach (var pair in batch.Anchors)
        {
            var live = _entityManager.GetComponentData<Node>(pair.Value.Entity).m_Position;
            yield return new
            {
                source = pair.Key,
                entity = $"{pair.Value.Entity.Index}:{pair.Value.Entity.Version}",
                planned = DiagnosticArtifacts.Point(pair.Value.Planned),
                preview = DiagnosticArtifacts.Point(pair.Value.Generated),
                live = DiagnosticArtifacts.Point(live),
                plannedHeightDelta = live.y - pair.Value.Planned.y,
                previewHeightDelta = live.y - pair.Value.Generated.y,
                horizontalCenterDelta = math.distance(live.xz, pair.Value.Planned.xz),
                pair.Value.CenterLimit,
                incidentBufferReady = true
            };
        }
    }

    // Read only inside the existing qualified-batch audit, after its existing job
    // synchronization. No extra synchronization or diagnostic background query.
    private IEnumerable<object> LiveConstraintRecords(NativeBatchBindings.Batch<Entity> batch)
    {
        var count = 0;

        foreach (var binding in batch.Edges)
        {
            if (count++ >= 64)
                yield break;

            var edge = binding.Generated;

            if (!_entityManager.Exists(edge.Entity) || !_entityManager.HasComponent<Curve>(edge.Entity))
            {
                yield return new
                {
                    constraint = "native-edge-geometry",
                    entity = edge.Entity.ToString(),
                    binding.Section.WayId,
                    evidence = "native-edge-unavailable-at-audit",
                    geometryVerified = false,
                    requiresReview = true
                };

                continue;
            }

            var live = _entityManager.GetComponentData<Curve>(edge.Entity).m_Bezier;
            var maximumGrade = 0f;
            var minimumRadius = float.PositiveInfinity;
            var stationary = false;

            for (var i = 0; i <= 32; i++)
            {
                var t = i / 32f;
                var u = 1 - t;
                var derivative =
                    3 * u * u * (live.b - live.a) + 6 * u * t * (live.c - live.b) + 3 * t * t * (live.d - live.c);

                var second = 6 * u * (live.c - 2 * live.b + live.a) + 6 * t * (live.d - 2 * live.c + live.b);
                var speed = math.length(derivative.xz);

                if (speed <= 1e-8f)
                {
                    stationary = true;

                    if (Math.Abs(derivative.y) > 1e-8f)
                        maximumGrade = float.PositiveInfinity;

                    continue;
                }

                maximumGrade = Math.Max(maximumGrade, Math.Abs(derivative.y) / speed);
                var cross = Math.Abs(derivative.x * second.z - derivative.z * second.x);

                if (cross > 1e-8f)
                    minimumRadius = Math.Min(minimumRadius, speed * speed * speed / cross);
            }

            yield return new
            {
                constraint = "native-edge-grade-and-curve-playback",
                policyStatus = "implementation-assumption-awaiting-visual-validation",
                evidence = "live-native-Curve-component",
                geometryVerified = true,
                visualAppearanceVerified = false,
                nativeGeometryFinite = math.all(math.isfinite(live.a))
                    && math.all(math.isfinite(live.b))
                    && math.all(math.isfinite(live.c))
                    && math.all(math.isfinite(live.d)),
                continuousCertificate = false,
                sampledStations = 33,
                stationaryHorizontalDerivativeObserved = stationary,
                entity = edge.Entity.ToString(),
                prefab = edge.Prefab.ToString(),
                binding.Section.WayId,
                binding.Section.StartNodeId,
                binding.Section.EndNodeId,
                kind = binding.Section.Kind.ToString(),
                binding.Section.Lanes,
                binding.Section.OneWay,
                binding.Section.Bridge,
                binding.Section.Tunnel,
                desiredGrade = Processing.NetworkVerticalProfiles.DesiredGrade(binding.Section),
                exceptionalGrade = Processing.NetworkGeometrySafety.GradeLimit(binding.Section),
                sampledMaximumGrade = maximumGrade,
                sampledGradeResidual = maximumGrade - Processing.NetworkGeometrySafety.GradeLimit(binding.Section),
                sampledMinimumRadiusMeters = float.IsInfinity(minimumRadius) ? (float?)null : minimumRadius,
                liveCurve = CurveEvidence(live),
                previewStart = edge.CurveStart.HasValue ? DiagnosticArtifacts.Point(edge.CurveStart.Value) : null,
                previewControl1 = edge.CurveControl1.HasValue
                    ? DiagnosticArtifacts.Point(edge.CurveControl1.Value)
                    : null,
                previewControl2 = edge.CurveControl2.HasValue
                    ? DiagnosticArtifacts.Point(edge.CurveControl2.Value)
                    : null,
                previewEnd = edge.CurveEnd.HasValue ? DiagnosticArtifacts.Point(edge.CurveEnd.Value) : null,
                control1MovementMeters = edge.CurveControl1.HasValue
                    ? (float?)math.distance(live.b, edge.CurveControl1.Value)
                    : null,
                control2MovementMeters = edge.CurveControl2.HasValue
                    ? (float?)math.distance(live.c, edge.CurveControl2.Value)
                    : null,
                requestedNativeEdges = batch.Edges.Count,
                sampleSelection = "first-64-qualified-native-edges-not-complete-coverage"
            };
        }
    }

    private Entity CreateDefinition(PreparedSegment segment, Entity prefab)
    {
        NativeBatchBindings.Anchor<Entity>? Committed(long id)
        {
            if (!_committedNodes.TryGetValue(NativeEndpointPolicy.Identity(segment, id), out var node))
                return null;

            if (!_entityManager.HasComponent<Node>(node)
                || _entityManager.HasComponent<Temp>(node)
                || _entityManager.HasComponent<Deleted>(node))
            {
                _committedNodes.Remove(NativeEndpointPolicy.Identity(segment, id));
                _committedAnchors.Remove(NativeEndpointPolicy.Identity(segment, id));
                _log.Count("Retired native anchors detached");

                return null;
            }

            var anchor = _committedAnchors[NativeEndpointPolicy.Identity(segment, id)];
            var position = _entityManager.GetComponentData<Node>(node).m_Position;

            if (!math.all(math.isfinite(position)))
            {
                _committedNodes.Remove(NativeEndpointPolicy.Identity(segment, id));
                _committedAnchors.Remove(NativeEndpointPolicy.Identity(segment, id));

                return null;
            }

            if (!math.all(position.xz == anchor.Generated.xz))
                _log.Count("Live native anchors rebased after regeneration");

            var current = anchor with { Generated = position };

            if (_revisedRoutes.Contains((segment.Kind, segment.WayId, segment.StartNodeId, segment.EndNodeId)))
                current =
                    NativeBatchBindings.LocalRevisionAnchor(
                        current,
                        id == segment.StartNodeId ? segment.Start : segment.End);

            return current;
        }

        var startAnchor = Committed(segment.StartNodeId);
        var endAnchor = Committed(segment.EndNodeId);
        // Live identity is authoritative for an existing attachment. A source
        // footprint disagreement is a diagnostic, not a reason to omit this course.
        var vectorA = startAnchor?.Generated ?? segment.Start;
        var vectorB = endAnchor?.Generated ?? segment.End;

        EdgeStation? Station(long id)
        {
            var key = NativeEndpointPolicy.Identity(segment, id);

            if (!_edgeStations.TryGetValue(key, out var station))
                return null;

            if (!_entityManager.HasComponent<Edge>(station.Edge)
                || !_entityManager.HasComponent<Curve>(station.Edge)
                || !_entityManager.HasComponent<PrefabRef>(station.Edge)
                || _entityManager.GetComponentData<PrefabRef>(station.Edge).m_Prefab != station.Prefab
                    || _entityManager.HasComponent<Temp>(station.Edge)
                    || _entityManager.HasComponent<Deleted>(station.Edge))
            {
                _edgeStations.Remove(key);
                return null;
            }

            var curve = _entityManager.GetComponentData<Curve>(station.Edge).m_Bezier;

            if (!math.all(math.isfinite(curve.a))
                || !math.all(math.isfinite(curve.b))
                || !math.all(math.isfinite(curve.c))
                || !math.all(math.isfinite(curve.d))
                || !math.all(math.isfinite(station.Position)))
            {
                _edgeStations.Remove(key);
                return null;
            }

            var t = CurveParameter(curve, station.Position);

            return station with { Parameter = t, Position = CurvePoint(curve, t) };
        }

        var startStation = startAnchor == null ? Station(segment.StartNodeId) : null;
        var endStation = endAnchor == null ? Station(segment.EndNodeId) : null;
        vectorA = startStation != null ? startStation.Position - segment.StartCurveOffset : vectorA;
        vectorB = endStation != null ? endStation.Position - segment.EndCurveOffset : vectorB;
        // Native node centres and deck ports are different contracts. Preserve
        // the exact submitted deck; an owner movement must not silently apply
        // an affine correction to its tangents or its vertical profile.
        segment = NativeBatchBindings.RebaseBound(segment, startAnchor, endAnchor);

        if (startStation != null || endStation != null)
            segment = segment with { Start = vectorA, End = vectorB };

        var curve =
            new Bezier4x3(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd);

        var direction = math.normalizesafe(segment.CourseControl1 - segment.CourseStart, new float3(0f, 0f, 1f));
        var rotation = quaternion.LookRotationSafe(direction, new float3(0f, 1f, 0f));
        var endDirection = math.normalizesafe(segment.CourseEnd - segment.CourseControl2, direction);
        var elevation = NetworkStructure.Offset(segment);
        var structure = NetworkStructure.IsStructure(segment);
        // Ground roads use their solved absolute curve and the prepared roadbed, not
        // an elevated composition merely because the original DEM was a little higher/lower.
        var startElevation = structure ? segment.StartElevation ?? elevation : 0f;
        var endElevation = structure ? segment.EndElevation ?? elevation : 0f;

        if (segment.Kind == NetworkKind.Rail && segment.Bridge && !segment.Tunnel)
        {
            startElevation = Math.Max(0, startElevation);
            endElevation = Math.Max(0, endElevation);
        }

        var startPosition =
            CoursePosition(
                segment.Start,
                rotation,
                true,
                _nodeElevations[NativeEndpointPolicy.Identity(segment, segment.StartNodeId)],
                segment.Bridge && !segment.Tunnel);

        var endPosition =
            CoursePosition(
                segment.End,
                quaternion.LookRotationSafe(endDirection, new float3(0, 1, 0)),
                false,
                _nodeElevations[NativeEndpointPolicy.Identity(segment, segment.EndNodeId)],
                segment.Bridge && !segment.Tunnel);

        if (startAnchor != null)
            startPosition.m_Entity = startAnchor.Entity;
        else if (startStation != null)
        {
            startPosition.m_Entity =
                startStation.Parameter == 0
                    ? _entityManager.GetComponentData<Edge>(startStation.Edge).m_Start
                    : startStation.Parameter == 1
                        ? _entityManager.GetComponentData<Edge>(startStation.Edge).m_End
                        : startStation.Edge;

            startPosition.m_SplitPosition = startStation.Parameter;
            startPosition.m_Position = startStation.Position;
        }

        if (endAnchor != null)
            endPosition.m_Entity = endAnchor.Entity;
        else if (endStation != null)
        {
            endPosition.m_Entity =
                endStation.Parameter == 0
                    ? _entityManager.GetComponentData<Edge>(endStation.Edge).m_Start
                    : endStation.Parameter == 1
                        ? _entityManager.GetComponentData<Edge>(endStation.Edge).m_End
                        : endStation.Edge;

            endPosition.m_SplitPosition = endStation.Parameter;
            endPosition.m_Position = endStation.Position;
        }

        // CourseSplitSystem.FindOverlapsJob checks the AND of both endpoint
        // flags. Protecting only an unbound end does nothing on a continuation:
        // the native tool can extend through/delete an already-owned junction.
        // Our prepared graph already owns all connections; preserve that graph
        // from the first preview, while explicit m_Entity still joins known nodes
        // and GenerateNodes' exact-position keys join new shared source nodes.
        startPosition.m_Flags |= CoursePosFlags.DisableMerge;
        endPosition.m_Flags |= CoursePosFlags.DisableMerge;
        var entity = _entityManager.CreateEntity();
        NetworkCommitSystem.RegisterDefinition(entity);

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
            // No manual side upgrades, including stale flags from a prepared object.
            // Native bridge/tunnel composition and terrain handling remain native.
            _submittedCourses.Add(
                new(
                    segment,
                    prefab)
                {
                    StartCenterBudget = _centerBudgets[NativeEndpointPolicy.Identity(segment, segment.StartNodeId)],
                    EndCenterBudget = _centerBudgets[NativeEndpointPolicy.Identity(segment, segment.EndNodeId)]
                });

            return entity;
        }
        catch
        {
            _entityManager.DestroyEntity(entity);
            throw;
        }
    }

    private static CoursePos CoursePosition(
        float3 position,
        quaternion rotation,
        bool start,
        float elevation,
        bool bridge) => new()
        {
            m_Entity = Entity.Null,
            m_Position = position,
            m_Rotation = rotation,
            m_Elevation = new float2(elevation),
            m_CourseDelta = start ? 0f : 1f,
            m_SplitPosition = 0f,
            // The solved absolute profile must survive native splitting. FreeHeight
            // re-samples terrain/water and applies elevation again, changing grades.
            m_Flags =
            (start ? CoursePosFlags.IsFirst : CoursePosFlags.IsLast) | CoursePosFlags.IsLeft | CoursePosFlags.IsRight |
            (bridge && elevation > .01f ? CoursePosFlags.ForceElevatedNode : 0)
                | (bridge ? CoursePosFlags.ForceElevatedEdge : 0),
            m_ParentMesh = -1
        };
}
