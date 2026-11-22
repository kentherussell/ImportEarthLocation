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
public static partial class NativeNetworkDiagnostics
{
    private sealed record Expected(float3 Point, float2 Direction, NetworkKind Kind, bool Structure, long Way);

    private sealed record Actual(float3 Point, float2 Direction, NetworkKind Kind, Entity Node);

    private static Dictionary<string, List<Expected>> _expected = new(StringComparer.Ordinal);
    private static IReadOnlyList<PreparedSegment> _sections = Array.Empty<PreparedSegment>();
    private static Processing.OsmDataset? _evidence;
    private static SourceTopologyManifest? _selectedTopology;

    public static void SetSelectedTopology(SourceTopologyManifest? topology) => _selectedTopology = topology;

    private static long _map, _generation;
    private static readonly Dictionary<SourceNodeKey, List<PreparedSegment>> IncidentSections = new();
    private static readonly Dictionary<long, List<OsmRelation>> RelationsByWay = new();
    private static readonly Dictionary<(NetworkKind, long, long, long), int> SectionIndices = new();
    private static readonly Dictionary<Entity, (Entity node, float requested, string prefab)> RoundaboutIslands = new();

    public static IReadOnlyDictionary<string, Entity> VerifiedNodes { get; private set; } =
        new Dictionary<string, Entity>();

    public static IReadOnlyDictionary<(NetworkKind Kind, long WayId, long Start, long End), List<Entity>> VerifiedPaths { get; private set; } =
        new Dictionary<(NetworkKind, long, long, long), List<Entity>>();

    private static IReadOnlyDictionary<Entity, NativeBatchBindings.Edge<Entity>> _observedEdges =
        new Dictionary<Entity, NativeBatchBindings.Edge<Entity>>();

    public sealed record Station(Entity Edge, Entity Prefab, float Parameter, float3 Position);

    public static IReadOnlyDictionary<string, Station> ObservedStations { get; private set; } =
        new Dictionary<string, Station>();

    public static void BindObserved(
        IReadOnlyDictionary<Entity, NativeBatchBindings.Edge<Entity>> edges,
        IReadOnlyDictionary<string, Station> stations)
    {
        _observedEdges = edges;
        ObservedStations = stations;
    }

    public static void BindVerified(
        IReadOnlyDictionary<string, Entity> nodes,
        IReadOnlyDictionary<(NetworkKind Kind, long WayId, long Start, long End), List<Entity>> paths)
    {
        VerifiedNodes = nodes;
        VerifiedPaths = paths;
    }

    public static void Remember(IReadOnlyList<PreparedSegment> sections, Processing.OsmDataset? evidence = null)
    {
        _sections = sections.ToList();
        _evidence = evidence;
        SectionIndices.Clear();

        for (var i = 0; i < sections.Count; i++)
        {
            var segment = sections[i];
            SectionIndices[(segment.Kind, segment.WayId, segment.StartNodeId, segment.EndNodeId)] = i;
        }

        _map = ImportSession.Map;
        _generation = ImportSession.Generation;
        IncidentSections.Clear();
        RelationsByWay.Clear();

        if (evidence != null)
            foreach (var relation in evidence.Relations)
            {
                if (!relation.Tags.TryGetValue("type", out var type)
                    || (type != "restriction" && type != "connectivity"))
                    continue;

                foreach (var way in relation.OrderedMembers
                    .Where(member => member.Type == "way" && (member.Role == "from" || member.Role == "to"))
                    .Select(member => member.Id)
                    .Distinct())
                {
                    if (!RelationsByWay.TryGetValue(way, out var list))
                        RelationsByWay[way] = list = new();

                    list.Add(relation);
                }
            }

        foreach (var segment in sections)
            foreach (var id in new[] { segment.StartNodeId, segment.EndNodeId })
            {
                var key = new SourceNodeKey(segment.Kind, id);

                if (!IncidentSections.TryGetValue(key, out var list))
                    IncidentSections[key] = list = new();

                list.Add(segment);
            }

        RoundaboutIslands.Clear();
        BindVerified(new Dictionary<string, Entity>(), new Dictionary<(NetworkKind, long, long, long), List<Entity>>());
        _expected = new(StringComparer.Ordinal);

        foreach (var segment in sections)
            foreach (var start in new[] { true, false })
            {
                var point = start ? segment.Start : segment.End;
                var control = start ? segment.CourseControl1 : segment.CourseControl2;
                var endpoint = start ? segment.CourseStart : segment.CourseEnd;
                var key = NativeEndpointPolicy.Identity(segment, start ? segment.StartNodeId : segment.EndNodeId);

                if (!_expected.TryGetValue(key, out var list))
                    _expected[key] = list = new List<Expected>();

                list.Add(
                    new(
                        point,
                        math.normalizesafe(control.xz - endpoint.xz),
                        segment.Kind,
                        NetworkStructure.IsStructure(segment),
                        segment.WayId));
            }
    }

    public static void Clear()
    {
        _expected.Clear();
        IncidentSections.Clear();
        RelationsByWay.Clear();
        SectionIndices.Clear();
        _sections = Array.Empty<PreparedSegment>();
        _evidence = null;
        _selectedTopology = null;
        PhysicalCensus.Clear();
        _map = _generation = 0;
        RoundaboutIslands.Clear();
        _observedEdges = new Dictionary<Entity, NativeBatchBindings.Edge<Entity>>();
        ObservedStations = new Dictionary<string, Station>();
        BindVerified(new Dictionary<string, Entity>(), new Dictionary<(NetworkKind, long, long, long), List<Entity>>());
    }

    public static void Release(long map, long generation)
    {
        if (_map == map && _generation == generation)
            Clear();
    }

    public static void ReviseUnissued(IReadOnlyList<PreparedSegment> revision)
    {
        if (_sections is not List<PreparedSegment> sections)
            return;

        foreach (var segment in revision)
        {
            if (!SectionIndices.TryGetValue(
                (segment.Kind, segment.WayId, segment.StartNodeId, segment.EndNodeId),
                out var index))
                continue;

            var old = sections[index];
            sections[index] = segment;

            foreach (var id in new[] { segment.StartNodeId, segment.EndNodeId })
            {
                var key = new SourceNodeKey(segment.Kind, id);

                if (!IncidentSections.TryGetValue(key, out var arms))
                    continue;

                for (var i = 0; i < arms.Count; i++)
                    if (ReferenceEquals(arms[i], old))
                        arms[i] = segment;

                var expected = new List<Expected>();

                foreach (var arm in arms)
                {
                    var start = arm.StartNodeId == id;
                    var control = start ? arm.CourseControl1 : arm.CourseControl2;
                    var endpoint = start ? arm.CourseStart : arm.CourseEnd;
                    expected.Add(
                        new(
                            start ? arm.Start : arm.End,
                            math.normalizesafe(control.xz - endpoint.xz),
                            arm.Kind,
                            NetworkStructure.IsStructure(arm),
                            arm.WayId));
                }

                _expected[NativeEndpointPolicy.Identity(segment, id)] = expected;
            }
        }
    }

    public static NativeNetworkReadback.Report CheckUnits(
        EntityManager manager,
        IReadOnlyList<PreparedSegment> units,
        ImportLog log,
        int startIndex)
    {
        // Boundary discovery also reads native buffers, before Check itself runs.
        manager.CompleteAllTrackedJobs();
        var closure = new Dictionary<(NetworkKind, long, long, long), PreparedSegment>();

        void Add(PreparedSegment segment)
            => closure[(segment.Kind, segment.WayId, segment.StartNodeId, segment.EndNodeId)] = segment;

        foreach (var segment in units)
        {
            Add(segment);

            foreach (var id in new[] { segment.StartNodeId, segment.EndNodeId })
                if (IncidentSections.TryGetValue(new(segment.Kind, id), out var arms))
                    foreach (var arm in arms)
                        Add(arm);
        }

        var edges = new HashSet<Entity>();

        foreach (var key in closure.Keys)
            if (VerifiedPaths.TryGetValue(key, out var path))
                foreach (var edge in path)
                    edges.Add(edge);

        foreach (var segment in closure.Values)
            foreach (var id in new[] { segment.StartNodeId, segment.EndNodeId })
                if (ObservedStations.TryGetValue(NativeEndpointPolicy.Identity(segment, id), out var station))
                    edges.Add(station.Edge);

        var nodes = new HashSet<Entity>();

        foreach (var segment in closure.Values)
            foreach (var id in new[] { segment.StartNodeId, segment.EndNodeId })
                if (VerifiedNodes.TryGetValue(NativeEndpointPolicy.Identity(segment, id), out var node)
                    && manager.HasComponent<Node>(node))
                    nodes.Add(node);

        foreach (var entity in edges)
            if (manager.HasComponent<Edge>(entity))
            {
                var edge = manager.GetComponentData<Edge>(entity);
                nodes.Add(edge.m_Start);
                nodes.Add(edge.m_End);
            }

        foreach (var node in nodes)
            if (manager.HasBuffer<ConnectedEdge>(node))
                foreach (var edge in manager.GetBuffer<ConnectedEdge>(node, true))
                    edges.Add(edge.m_Edge);

        var relations = new Dictionary<long, OsmRelation>();

        foreach (var segment in closure.Values)
            foreach (var way in segment.SourceWayIds.Count > 0 ? segment.SourceWayIds : new[] { segment.WayId })
                if (RelationsByWay.TryGetValue(way, out var candidates))
                    foreach (var relation in candidates)
                        relations[relation.Id] = relation;

        var stem = "native-unit-final-" + startIndex;
        log.DiagnosticRecords(stem + "-coverage.ndjson", units.Select(segment => (object)new
        {
            segment.WayId,
            segment.StartNodeId,
            segment.EndNodeId,
            segment.NativeUnitId,
            kind = segment.Kind.ToString(),
            hasRecordedPath = VerifiedPaths.ContainsKey(
                (segment.Kind, segment.WayId, segment.StartNodeId, segment.EndNodeId)),
            expectedCoreSection = true
        }), final: true);

        var report =
            NativeNetworkReadback.Check(
                manager,
                closure.Values.ToArray(),
                _evidence,
                log,
                true,
                edges.ToArray(),
                stem,
                reportWarnings: false,
                relationScope: relations.Values.ToArray(),
                finalCapture: true);

        log.Count("Final native unit sections inspected", units.Count);
        log.Count("Final unit unresolved closure observations", report.Unresolved);
        log.Count("Final unit tunnel composition observations", report.ExposedTunnels);
        log.Count("Final unit required lane-slot observations", report.RequiredLaneSlots);
        log.Count("Final unit disconnected lane-slot observations", report.DisconnectedLaneSlots);
        log.Count("Final unit unavailable lane-slot observations", report.UnavailableLaneSlots);
        log.Count("Final unit rail representation observations", report.RailRepresentationMismatches);
        log.Count("Final unit rail representations checked", report.RailRepresentationsChecked);
        log.Count("Final unit unavailable rail representations", report.UnavailableRailRepresentations);
        log.DiagnosticRecords(
            stem + "-result.ndjson",
            new object[]{new{startIndex,coreSections=units.Count,closureSections=closure.Count,report,
            nativeReadbackReturned=true,renderedAcceptanceVerified=false}}, final: true);

        log.Diagnostic(
            $"NATIVE UNIT AUDIT start={startIndex}; coreSections={units.Count}; closureSections={closure.Count}; lineageSections={report.LineageSections}; unresolved={report.Unresolved}; missingCompositions={report.MissingCompositions}; exposedTunnels={report.ExposedTunnels}; candidateDirectionalPortPairs={report.CandidatePortMovements}; unobservedPortPairs={report.UnobservedPortMovements}; allFutureNeighboursIssued=true; perLaneSlotAcceptance=false; quality=RequiresReview; placementReplayed=false");

        return report;
    }

    public static IEnumerable<int> CaptureOwnedCensus(EntityManager manager, ImportLog log)
    {
        if (!log.DiagnosticsEnabled)
            yield break;

        PhysicalCensus.Clear();
        var sourceAnchors = VerifiedNodes.OrderBy(entry => entry.Key, StringComparer.Ordinal).ToArray();
        var sourceStations = ObservedStations.OrderBy(entry => entry.Key, StringComparer.Ordinal).ToArray();
        var entities = _observedEdges.Keys.OrderBy(entity => entity.Index).ThenBy(entity => entity.Version).ToArray();
        var captured = 0;
        var unavailable = 0;
        var railChecked = 0;
        var railUnavailable = 0;
        log.DiagnosticRecords(
            "native-owned-census-status.ndjson",
            new object[]{new{state="started",expectedObservedEntities=entities.Length,
            expectedSourceAnchors=sourceAnchors.Length,expectedSourceStations=sourceStations.Length,
            complete=false,
                scope="exclusive-preview-owned-edges-including-uncertified-batches",
                sourceAcceptanceImplied=false
                }},
                final: true);

        for (var offset = 0; offset < sourceAnchors.Length; offset += 128)
        {
            manager.CompleteAllTrackedJobs();
            log.DiagnosticRecordChunks(
                "native-final-source-anchors-" + offset + ".ndjson",
                sourceAnchors.Skip(offset).Take(128).Select(entry => (object)new
                {
                    source = entry.Key,
                    entity = entry.Value.ToString(),
                    frame = UnityEngine.Time.frameCount,
                    available = manager.HasComponent<Node>(entry.Value),
                    position = manager.HasComponent<Node>(entry.Value)
                    ? DiagnosticArtifacts.Point(manager.GetComponentData<Node>(entry.Value).m_Position)
                    : null,
                    entityLineageOnly = true,
                    geometryAcceptanceImplied = false
                }));

            yield return 0;
        }

        for (var offset = 0; offset < sourceStations.Length; offset += 128)
        {
            manager.CompleteAllTrackedJobs();
            log.DiagnosticRecordChunks(
                "native-final-source-stations-" + offset + ".ndjson",
                sourceStations.Skip(offset).Take(128).Select(entry => (object)new
                {
                    source = entry.Key,
                    edge = entry.Value.Edge.ToString(),
                    frame = UnityEngine.Time.frameCount,
                    prefab = entry.Value.Prefab.ToString(),
                    entry.Value.Parameter,
                    position = DiagnosticArtifacts.Point(entry.Value.Position),
                    available = manager.HasComponent<Edge>(entry.Value.Edge)
                        && !manager.HasComponent<Deleted>(entry.Value.Edge),
                    originalIncidenceLineage = true,
                    geometryAcceptanceImplied = false
                }));

            yield return 0;
        }

        for (var offset = 0; offset < entities.Length; offset += 128)
        {
            manager.CompleteAllTrackedJobs();
            var scope = entities.Skip(offset).Take(128).ToArray();

            try
            {
                var report =
                    NativeNetworkReadback.Check(manager, Array.Empty<PreparedSegment>(), null, log, true, scope,
                    "native-owned-final-" + offset, reportWarnings: false, finalCapture: true, captureRawLanes: true);

                CapturePhysicalSlice(manager, scope, offset);
                railChecked += report.RailRepresentationsChecked;
                railUnavailable += report.UnavailableRailRepresentations;
                captured +=
                    scope.Count(
                        entity => manager.HasComponent<Edge>(entity)
                            && !manager.HasComponent<Deleted>(entity)
                            && !manager.HasComponent<Temp>(entity));
            }
            catch (Exception error) when (error is not OutOfMemoryException && error is not OperationCanceledException)
            {
                unavailable += scope.Length;
                log.Diagnostic("NATIVE OWNED CENSUS unavailable slice=" + offset + "; " + error);
            }

            yield return 0;
        }

        log.DiagnosticRecords(
            "native-owned-census-status.ndjson",
            new object[]{new{state="finished",expectedObservedEntities=entities.Length,
            liveEntitiesObserved=captured,
                unavailableEntities=unavailable,
                retiredOrReplacedEntities=entities.Length-captured-unavailable,
            railRepresentationsChecked=railChecked,unavailableRailRepresentations=railUnavailable,
            iterationComplete=true,
                distributedAcrossFrames=true,
                singleInstantSnapshot=false,
                completeArtifactCaptureNotImplied=true,
                sourceAcceptanceImplied=false,
            scope="exclusive-preview-owned-edges-including-uncertified-batches"}}, final: true);

        var pairedGroups =
            _sections.Where(segment => segment.PairedProfileId != 0).GroupBy(segment => (segment.PairedProfileId,
            Begin: Math.Round(Math.Min(segment.PairedReferenceBegin, segment.PairedReferenceEnd), 6),
                End: Math.Round(Math.Max(segment.PairedReferenceBegin, segment.PairedReferenceEnd), 6)))
                .ToArray();

        var snapshotMap = _map;
        var snapshotGeneration = _generation;

        for (var offset = 0; offset < pairedGroups.Length; offset += 64)
        {
            if (!ImportSession.Matches(snapshotMap, snapshotGeneration))
                yield break;

            manager.CompleteAllTrackedJobs();

            IReadOnlyList<Cubic3d>? Observe(PreparedSegment segment)
            {
                if (!VerifiedPaths.TryGetValue(
                    (segment.Kind, segment.WayId, segment.StartNodeId, segment.EndNodeId),
                    out var path)
                    || !VerifiedNodes.TryGetValue(
                        NativeEndpointPolicy.Identity(segment, segment.StartNodeId),
                        out var cursor)
                        || !VerifiedNodes.TryGetValue(
                            NativeEndpointPolicy.Identity(segment, segment.EndNodeId),
                            out var end))
                    return null;

                var curves = new List<Cubic3d>();

                foreach (var entity in path)
                {
                    if (!manager.HasComponent<Edge>(entity)
                        || !manager.HasComponent<Curve>(entity)
                        || manager.HasComponent<Deleted>(entity)
                        || manager.HasComponent<Temp>(entity))
                        return null;

                    var edge = manager.GetComponentData<Edge>(entity);
                    var curve = manager.GetComponentData<Curve>(entity).m_Bezier;

                    if (edge.m_Start == cursor)
                    {
                        curves.Add(new(curve.a, curve.b, curve.c, curve.d));
                        cursor = edge.m_End;
                    }
                    else if (edge.m_End == cursor)
                    {
                        curves.Add(new(curve.d, curve.c, curve.b, curve.a));
                        cursor = edge.m_Start;
                    }
                    else
                        return null;
                }

                return cursor == end ? curves : null;
            }

            Processing.NetworkObservedQuality.Paired(
                pairedGroups.Skip(offset).Take(64).SelectMany(group => group).ToArray(),
                Observe,
                log,
                System.Threading.CancellationToken.None, "native-pairs-" + offset);

            yield return 0;
        }

        var nodes = IncidentSections.Keys.OrderBy(nodeKey => nodeKey.Kind).ThenBy(nodeKey => nodeKey.Id).ToArray();
        var observedDegrees = new Dictionary<SourceNodeKey, int?>();

        for (var offset = 0; offset < nodes.Length; offset += 128)
        {
            if (!ImportSession.Matches(snapshotMap, snapshotGeneration))
                yield break;

            manager.CompleteAllTrackedJobs();

            foreach (var key in nodes.Skip(offset).Take(128))
            {
                var section = IncidentSections[key][0];

                if (!VerifiedNodes.TryGetValue(NativeEndpointPolicy.Identity(section, key.Id), out var entity)
                    || !manager.HasBuffer<ConnectedEdge>(entity))
                {
                    observedDegrees[key] = null;
                    continue;
                }

                var degree = 0;

                foreach (var edge in manager.GetBuffer<ConnectedEdge>(entity, true))
                    if (manager.HasComponent<Edge>(edge.m_Edge)
                        && !manager.HasComponent<Deleted>(edge.m_Edge)
                        && !manager.HasComponent<Temp>(edge.m_Edge))
                        degree++;

                observedDegrees[key] = degree;
            }

            yield return 0;
        }

        Processing.NetworkObservedQuality.Ports(
            _selectedTopology,
            _sections,
            key => observedDegrees.TryGetValue(key, out var degree) ? degree : null,
            log, System.Threading.CancellationToken.None, "native-ports", "final-owned-ports");

        log.FlushArchive(force: true);
    }

    public static void Check(EntityManager manager, ImportLog log, int sectionLimit = int.MaxValue)
    {
        if (_expected.Count == 0)
            return;

        manager.CompleteAllTrackedJobs();
        using var query = manager.CreateEntityQuery(new EntityQueryDesc
        {
            All =
                new[] {
                    ComponentType.ReadOnly<Edge>(),
                    ComponentType.ReadOnly<Curve>(),
                    ComponentType.ReadOnly<PrefabRef>()
                    },
            None = new[] { ComponentType.ReadOnly<Temp>(), ComponentType.ReadOnly<Deleted>() }
        });

        using var entities = query.ToEntityArray(Allocator.Temp);
        var grid = new Dictionary<(int, int), List<Actual>>();

        (int, int) Cell(float3 point) => ((int)Math.Floor(point.x / 4), (int)Math.Floor(point.z / 4));

        foreach (var entity in entities)
        {
            var prefab = manager.GetComponentData<PrefabRef>(entity).m_Prefab;
            var road = manager.HasComponent<RoadData>(prefab);
            var rail = manager.HasComponent<TrackData>(prefab);

            if (!road && !rail)
                continue;

            var edge = manager.GetComponentData<Edge>(entity);
            var curve = manager.GetComponentData<Curve>(entity).m_Bezier;
            Add(curve.a, curve.b, edge.m_Start);
            Add(curve.d, curve.c, edge.m_End);

            void Add(float3 point, float3 control, Entity node)
            {
                // Native junction geometry trims curve ends and offsets merging
                // lanes. Connectivity belongs to the node anchor, not a trimmed deck end.
                var anchor = manager.HasComponent<Node>(node) ? manager.GetComponentData<Node>(node).m_Position : point;
                var cell = Cell(anchor);

                if (!grid.TryGetValue(cell, out var list))
                    grid[cell] = list = new List<Actual>();

                list.Add(
                    new(
                        anchor,
                        math.normalizesafe(control.xz - point.xz),
                        road ? NetworkKind.Road : NetworkKind.Rail,
                        node));
            }
        }

        var checkedJoins = 0;
        var missing = 0;
        var split = 0;
        var portals = 0;
        var badPortals = 0;

        foreach (var pair in _expected.Where(entry => entry.Value.Count > 1).Take(sectionLimit))
        {
            checkedJoins++;
            var portal =
                pair.Value.Any(expected => expected.Structure) && pair.Value.Any(expected => !expected.Structure);

            if (portal)
                portals++;

            HashSet<Entity>? common = null;
            var absent = false;

            foreach (var expected in pair.Value)
            {
                var bound = VerifiedNodes.TryGetValue(pair.Key, out var boundNode);
                var point =
                    bound && manager.HasComponent<Node>(boundNode)
                        ? manager.GetComponentData<Node>(boundNode).m_Position
                        : expected.Point;

                var cell = Cell(point);
                var candidates = new HashSet<Entity>();

                for (var x = cell.Item1 - 1; x <= cell.Item1 + 1; x++)
                    for (var z = cell.Item2 - 1; z <= cell.Item2 + 1; z++)
                        if (grid.TryGetValue((x, z), out var list))
                            foreach (var actual in list)
                                if (actual.Kind == expected.Kind
                                    && (bound ? actual.Node == boundNode : math.distance(actual.Point, point) <= 2)
                                    && math.dot(actual.Direction, expected.Direction) > .7f)
                                    candidates.Add(actual.Node);

                if (candidates.Count == 0)
                    absent = true;

                if (common == null)
                    common = candidates;
                else
                    common.IntersectWith(candidates);
            }

            if (!absent && common!.Count > 0)
                continue;

            if (absent)
                missing++;
            else
                split++;

            if (portal)
                badPortals++;

            if (missing + split <= 25)
                log.Diagnostic(
                    $"NATIVE CONNECTIVITY mismatch={pair.Key}; ways={string.Join(",", pair.Value.Select(expected => expected.Way).Distinct())}; position={pair.Value[0].Point}; missingIncident={absent}; separateNativeNodes={!absent}; structurePortal={portal}; verifiedEntityMatching={VerifiedNodes.ContainsKey(pair.Key)}; geometricFallbackToleranceMeters=2; nearest={string.Join(" ", pair.Value.Select(Nearest))}");
        }

        // What the live graph holds nearest each expected end, within 32 m on the map:
        // distance on the map, height difference and how well the directions agree. It
        // tells a node the game moved (or built at another height) from an edge that
        // was never built.
        string Nearest(Expected expected)
        {
            const int Reach = 8;
            var cell = Cell(expected.Point);
            Actual? best = null;
            var bestDistance = float.MaxValue;

            for (var x = cell.Item1 - Reach; x <= cell.Item1 + Reach; x++)
                for (var z = cell.Item2 - Reach; z <= cell.Item2 + Reach; z++)
                {
                    if (!grid.TryGetValue((x, z), out var list))
                        continue;

                    foreach (var actual in list)
                    {
                        var distance = math.distance(actual.Point.xz, expected.Point.xz);

                        if (actual.Kind != expected.Kind || distance >= bestDistance)
                            continue;

                        best = actual;
                        bestDistance = distance;
                    }
                }

            if (best == null)
                return "none";

            var height = best.Point.y - expected.Point.y;
            var agreement = math.dot(best.Direction, expected.Direction);

            return FormattableString.Invariant($"{bestDistance:F1}m/{height:+0.0;-0.0}m/dot{agreement:F2}");
        }

        log.Count("Native shared junctions checked", checkedJoins);
        log.Count("Native junctions missing incident edges", missing);
        log.Count("Native junctions split across nodes", split);
        log.Count("Native structure portals mismatched", badPortals);
        log.Diagnostic(
            $"NATIVE CONNECTIVITY SUMMARY liveEdges={entities.Length}; sharedJunctionsChecked={checkedJoins}; missingIncidentEdges={missing}; splitJunctions={split}; structurePortalsChecked={portals}; structurePortalsMismatched={badPortals}; boundSourceNodes={VerifiedNodes.Count}; readOnly=true; verifiedEntityMatchingWhenAvailable=true; geometricFallbackOnlyWhenUnbound=true; saveReloadVerificationRequired=true");

        if (missing + split > 0)
            log.Warn(
                $"Native network check found {missing} junctions with missing/moved edges and {split} with separate nodes, including {badPortals} structure portals. Definition counts are not proof of connectivity; generation is not blocked.");

        log.Diagnostic(
            $"NATIVE READBACK BUDGET sectionLimit={sectionLimit}; totalPreparedSections={_sections.Count}; sampledReadback={_sections.Count > sectionLimit}; completeConnectivityNotClaimed={_sections.Count > sectionLimit}");

        // Stratify the bounded sample: a road-first source array must not hide
        // every railway and tunnel from final native inspection.
        var selected = new List<PreparedSegment>();
        var strata = _sections.GroupBy(segment => (segment.Kind, segment.Tunnel, segment.Bridge)).ToArray();
        var quotas = NativeDiagnosticSampling.Quotas(strata.Select(group => group.Count()).ToArray(), sectionLimit);

        for (var group = 0; group < strata.Length; group++)
        {
            var all = strata[group].OrderBy(segment => segment.WayId).ThenBy(segment => segment.StartNodeId).ToArray();
            var quota = quotas[group];

            for (var j = 0; j < quota; j++)
                selected.Add(all[(int)((long)j * all.Length / quota)]);
        }

        log.Diagnostic(
            $"NATIVE READBACK SAMPLE selected={selected.Count}; strata=kind-tunnel-bridge; sourceOrderBiasRemoved=true; fullMapReadback={selected.Count == _sections.Count}");

        NativeNetworkReadback.Check(manager, selected, _evidence, log, includeMovements: sectionLimit == int.MaxValue);
        var islandRecords = new List<object>();
        var attachedIslands = 0;

        foreach (var pair in RoundaboutIslands)
        {
            var live =
                manager.HasComponent<Game.Objects.Attached>(pair.Key) && !manager.HasComponent<Deleted>(pair.Key);

            var parent = live ? manager.GetComponentData<Game.Objects.Attached>(pair.Key).m_Parent : Entity.Null;

            // The node lists an ordinary attached object in its SubObject buffer; the
            // reciprocal Attachment component is only written for placeholder parents.
            var attached = false;

            if (parent != Entity.Null && manager.HasBuffer<Game.Objects.SubObject>(parent))
            {
                var subObjects = manager.GetBuffer<Game.Objects.SubObject>(parent, true);

                for (var i = 0; i < subObjects.Length; i++)
                    attached |= subObjects[i].m_SubObject == pair.Key;
            }

            var radius =
                parent != Entity.Null && manager.HasComponent<Roundabout>(parent)
                    ? manager.GetComponentData<Roundabout>(parent).m_Radius
                    : 0;

            // Which road ends at the node took a roundabout shape, and which road types
            // cannot take one at all.
            int roads = 0, roundaboutEnds = 0, unsupportedRoads = 0;

            if (parent != Entity.Null && manager.HasBuffer<ConnectedEdge>(parent))
            {
                foreach (var connected in manager.GetBuffer<ConnectedEdge>(parent, true))
                {
                    var road = connected.m_Edge;

                    if (!manager.HasComponent<Edge>(road) || !manager.HasComponent<Composition>(road))
                        continue;

                    roads++;
                    var edge = manager.GetComponentData<Edge>(road);
                    var composition = manager.GetComponentData<Composition>(road);
                    var end = edge.m_Start == parent ? composition.m_StartNode : composition.m_EndNode;

                    if (manager.HasComponent<NetCompositionData>(end)
                        && (manager.GetComponentData<NetCompositionData>(end).m_Flags.m_General
                            & CompositionFlags.General.Roundabout) != 0)
                        roundaboutEnds++;

                    var prefab = manager.GetComponentData<PrefabRef>(road).m_Prefab;

                    if (manager.HasComponent<NetGeometryData>(prefab)
                        && (manager.GetComponentData<NetGeometryData>(prefab).m_Flags & GeometryFlags.SupportRoundabout) == 0)
                        unsupportedRoads++;
                }
            }

            if (attached && radius > 0)
                attachedIslands++;

            islandRecords.Add(
                new
                {
                    island = $"{pair.Key.Index}:{pair.Key.Version}",
                    pair.Value.prefab,
                    pair.Value.requested,
                    live,
                    parent = $"{parent.Index}:{parent.Version}",
                    attached,
                    nativeRadius = radius,
                    roads,
                    roundaboutEnds,
                    unsupportedRoads,
                    renderingVerified = false
                });
        }

        log.DiagnosticRecords("native-roundabout-readback.ndjson", islandRecords);
        log.Diagnostic(
            $"ROUNDABOUT READBACK expectedIslands={RoundaboutIslands.Count}; attachedWithNativeRadius={attachedIslands}; appearanceAndTrafficUnverified=true");
    }

    public static void ApplySmallRoundabouts(
        EntityManager manager,
        PrefabSystem prefabs,
        IReadOnlyList<PreparedSegment> sections,
        ImportLog log)
    {
        // The stock tools place a NetObject island. A manually assigned node
        // radius is recomputed by NetComponentsSystem and is not the tool contract.
        var palette = new PrefabResolver(prefabs, log).ReadRoundaboutIslands();
        var planned = new Dictionary<string, (float3 point, float radius)>();

        foreach (var segment in sections)
        {
            if (segment.StartRoundaboutRadius > 0)
                planned[NativeEndpointPolicy.Identity(segment, segment.StartNodeId)] =
                    (segment.Start, segment.StartRoundaboutRadius);

            if (segment.EndRoundaboutRadius > 0)
                planned[NativeEndpointPolicy.Identity(segment, segment.EndNodeId)] =
                    (segment.End, segment.EndRoundaboutRadius);
        }

        if (planned.Count == 0)
            return;

        manager.CompleteAllTrackedJobs();
        using var query =
            manager.CreateEntityQuery(
                new EntityQueryDesc
                {
                    All = new[] { ComponentType.ReadOnly<Node>(), ComponentType.ReadOnly<ConnectedEdge>() },
                    None = new[] { ComponentType.ReadOnly<Temp>(), ComponentType.ReadOnly<Deleted>() }
                });

        using var entities = query.ToEntityArray(Allocator.Temp);
        var grid = new Dictionary<(int, int), List<Entity>>();

        foreach (var entity in entities)
        {
            var point = manager.GetComponentData<Node>(entity).m_Position;
            var key = ((int)Math.Floor(point.x / 4), (int)Math.Floor(point.z / 4));

            if (!grid.TryGetValue(key, out var list))
                grid[key] = list = new();

            list.Add(entity);
        }

        var applied = 0;
        var unverified = 0;

        foreach (var pair in planned)
        {
            var plan = pair.Value;
            var x = (int)Math.Floor(plan.point.x / 4);
            var z = (int)Math.Floor(plan.point.z / 4);
            var candidates = new HashSet<Entity>();

            if (VerifiedNodes.TryGetValue(pair.Key, out var boundNode))
            {
                if (manager.HasComponent<Node>(boundNode)
                    && manager.HasBuffer<ConnectedEdge>(boundNode)
                    && !manager.HasComponent<Temp>(boundNode)
                    && !manager.HasComponent<Deleted>(boundNode))
                    candidates.Add(boundNode);
            }
            else
                for (var dx = -1; dx <= 1; dx++)
                    for (var dz = -1; dz <= 1; dz++)
                        if (grid.TryGetValue((x + dx, z + dz), out var list))
                            foreach (var entity in list)
                                if (math.distance(manager.GetComponentData<Node>(entity).m_Position, plan.point) < 1)
                                    candidates.Add(entity);

            if (candidates.Count != 1)
            {
                unverified++;
                log.Warn(
                    "Small roundabout upgrade could not identify one native junction. Existing approaches retained without guessing a node; the native connectivity audit will report any placement mismatch.");

                continue;
            }

            var node = candidates.Single();

            // A stock roundabout fits a node with any number of roads, a dead end included.
            if (manager.GetBuffer<ConnectedEdge>(node, true).Length < 1)
            {
                unverified++;
                log.Warn(
                    "Small roundabout upgrade found a node with no roads. Node left unchanged and reported for connectivity review.");

                continue;
            }

            if (manager.HasComponent<Game.Objects.Attachment>(node)
                && manager.GetComponentData<Game.Objects.Attachment>(node).m_Attached != Entity.Null)
            {
                log.Diagnostic($"ROUNDABOUT existing attachment retained node={node}; requestedRadius={plan.radius}");
                continue;
            }

            if (palette.Count == 0)
            {
                unverified++;
                log.Warn("Stock roundabout object catalog unavailable; junction retained for inspection.");
                continue;
            }

            var selected =
                palette
                    .OrderBy(item => Math.Abs(item.Radius - plan.radius))
                    .ThenByDescending(item => item.Radius)
                    .First();

            var island = manager.CreateEntity(selected.Data.m_Archetype);

            try
            {
                manager.SetComponentData(island, new PrefabRef { m_Prefab = selected.Prefab });
                manager.SetComponentData(
                    island,
                    new Game.Objects.Transform
                    {
                        m_Position = manager.GetComponentData<Node>(node).m_Position,
                        m_Rotation = quaternion.identity
                    });

                // The island is attached to its node as the stock tool attaches it; a prefab
                // whose archetype lacks the component gets it here.
                var attached =
                    new Game.Objects.Attached { m_Parent = node, m_OldParent = Entity.Null, m_CurvePosition = 0 };

                if (manager.HasComponent<Game.Objects.Attached>(island))
                    manager.SetComponentData(island, attached);
                else
                    manager.AddComponentData(island, attached);

                if (!manager.HasComponent<Updated>(island))
                    manager.AddComponentData(island, default(Updated));

                // The game takes a node's roundabout from the net objects in its SubObject
                // buffer (CompositionSelectSystem.GetSubObjectFlags). AttachSystem enters an
                // updated attached object there too, but possibly after this frame's
                // composition pass, so the island is entered now; the add is idempotent.
                var subObjects =
                    manager.HasBuffer<Game.Objects.SubObject>(node)
                        ? manager.GetBuffer<Game.Objects.SubObject>(node)
                        : manager.AddBuffer<Game.Objects.SubObject>(node);

                var listed = false;

                for (var i = 0; i < subObjects.Length; i++)
                    listed |= subObjects[i].m_SubObject == island;

                if (!listed)
                    subObjects.Add(new Game.Objects.SubObject(island));

                RoundaboutIslands[island] = (node, plan.radius, selected.Name);
                log.Diagnostic(
                    $"ROUNDABOUT stock object created prefab={selected.Name}; object={island}; node={node}; selectedCenterlineRadius={selected.Radius}; requestedRadius={plan.radius}; nativeAttachmentRebuildPending=true");
            }
            catch (Exception error)
            {
                manager.DestroyEntity(island);
                unverified++;
                log.Warn("Roundabout island omitted locally: " + error.Message);
                continue;
            }

            // The game chooses a junction's shape per road end, and only for roads marked
            // updated (CompositionSelectSystem); the stock tool's apply marks the node's roads,
            // and a marked node alone leaves their old ends and a roundabout radius of zero
            // (report 121: 0 of 210 islands). The roundabout comes from the island itself, not
            // from a hand-set Upgraded flag, so removing the island in the game removes it.
            if (!manager.HasComponent<Updated>(node))
                manager.AddComponentData(node, default(Updated));

            var roads = manager.GetBuffer<ConnectedEdge>(node, true).ToNativeArray(Allocator.Temp);

            foreach (var road in roads)
                if (!manager.HasComponent<Updated>(road.m_Edge))
                    manager.AddComponentData(road.m_Edge, default(Updated));

            roads.Dispose();
            applied++;
        }

        log.Diagnostic(
            $"ROUNDABOUT stock islands created={applied}; unverifiedJunctions={unverified}; nativeCompositionRebuildRequested=true; nativeAttachmentReadbackPending=true; renderedRadiusRequiresGameReview=true");
    }
}
