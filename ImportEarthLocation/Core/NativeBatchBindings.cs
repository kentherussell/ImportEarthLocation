using System;
using System.Collections.Generic;
using Unity.Mathematics;

namespace ImportEarthLocation.Core;

// Bind the generated graph BEFORE Apply. Native ground height is not an OSM
// identity: new attachments require exact submitted ports; regenerated routes
// require established owners, bounded footprints, selected asset/mode and a
// unique directed chain (native generation can subdivide a course).
// Carry actual entity lineage through playback, never nearest-node snapping.
public static class NativeBatchBindings
{
    // Numeric playback roundoff only, never a spatial attachment tolerance.
    // Entity ownership, asset and complete incidence are checked by the caller.
    public static bool PlaybackCoordinate(float expected, float actual)
    {
        if (!math.isfinite(expected) || !math.isfinite(actual))
            return false;

        if (expected == actual)
            return true;

        var magnitude = Math.Max(Math.Abs(expected), Math.Abs(actual));
        var ulp = math.asfloat(math.asint(magnitude) + 1) - magnitude;

        return Math.Abs(expected - actual) <= Math.Min(.01f, ulp * 2);
    }

    public static bool PlaybackHorizontal(float3 expected, float3 actual) =>
        math.all(math.isfinite(actual))
            && PlaybackCoordinate(expected.x, actual.x)
            && PlaybackCoordinate(expected.z, actual.z);

    // ApplyNet keeps preview entities for creates/replacements; ordinary updates
    // are copied to Original. The caller supplies native node/edge flag semantics.
    public static T Target<T>(T preview, T original, bool hasOriginal, bool replaces) where T : notnull =>
        hasOriginal && !replaces ? original : preview;

    public static Dictionary<string, T> RemapCommitted<T>(
        IReadOnlyDictionary<string, T> committed,
        IReadOnlyDictionary<T, T> replacements) where T : notnull
    {
        var mapped = new Dictionary<string, T>(StringComparer.Ordinal);

        foreach (var pair in committed)
            mapped[pair.Key] = replacements.TryGetValue(pair.Value, out var next) ? next : pair.Value;

        return mapped;
    }

    public sealed record Expected<T>(PreparedSegment Section, T Prefab) where T : notnull
    {
        public float StartCenterBudget { get; init; }
        public float EndCenterBudget { get; init; }
        public float ReconciliationTolerance { get; init; }
        public bool ValidateCurveCorridor { get; init; }
        // Only requalification of an already verified route between owned nodes.
        // New source attachment retains the exact submitted-port contract.
        public bool OwnedRoute { get; init; }
    }

    public sealed record Edge<T>(
        T Entity,
        T StartNode,
        T EndNode,
        float3 Start,
        float3 End,
        NetworkKind Kind,
        T Prefab) where T : notnull
    {
        // Node centers may be regenerated inside a junction while the curve
        // still starts/ends at the submitted port. Null retains strict legacy
        // center matching; drift without this native curve evidence is rejected.
        public float3? CurveStart { get; init; }
        public float3? CurveEnd { get; init; }
        public float3? CurveControl1 { get; init; }
        public float3? CurveControl2 { get; init; }
    }

    public sealed record Anchor<T>(T Entity, float3 Planned, float3 Generated) where T : notnull
    {
        public float CenterLimit { get; init; }
    }

    public sealed record Binding<T>(PreparedSegment Section, Edge<T> Generated, bool Forward) where T : notnull;

    public sealed record Batch<T>(IReadOnlyDictionary<string, Anchor<T>> Anchors, IReadOnlyList<Binding<T>> Edges,
        int SectionCount, IReadOnlyDictionary<T, float3> Nodes) where T : notnull;

    public static Batch<T> Match<T>(IReadOnlyList<Expected<T>> expected, IReadOnlyList<Edge<T>> generated,
        IReadOnlyDictionary<string, T> committed, Action<object>? rejectedEvidence = null) where T : notnull
    {
        var starts = new Dictionary<(NetworkKind, T, float2), HashSet<T>>();
        var incident = new Dictionary<T, List<Edge<T>>>();
        var positions = new Dictionary<T, float3>();

        foreach (var edge in generated)
        {
            if (!math.all(math.isfinite(edge.Start)) || !math.all(math.isfinite(edge.End)))
                throw new InvalidOperationException("Native generated graph has a non-finite anchor.");

            if (edge.CurveStart.HasValue != edge.CurveEnd.HasValue)
                throw new InvalidOperationException("Native generated graph has incomplete curve port evidence.");

            Add(edge, edge.StartNode, edge.Start, edge.CurveStart);
            Add(edge, edge.EndNode, edge.End, edge.CurveEnd);
        }

        void Add(Edge<T> edge, T node, float3 point, float3? port)
        {
            if (port.HasValue && !math.all(math.isfinite(port.Value)))
                throw new InvalidOperationException("Native generated graph has a non-finite curve port.");

            if (positions.TryGetValue(node, out var prior) && !math.all(prior == point))
                throw new InvalidOperationException("Native generated entity has conflicting anchors.");

            positions[node] = point;
            var key = (edge.Kind, edge.Prefab, (port ?? point).xz);

            if (!starts.TryGetValue(key, out var nodes))
                starts[key] = nodes = new HashSet<T>();

            nodes.Add(node);

            if (!incident.TryGetValue(node, out var edgesAtNode))
                incident[node] = edgesAtNode = new List<Edge<T>>();

            edgesAtNode.Add(edge);
        }

        // A different submitted source junction is not an anonymous subdivision
        // point. Do not route one source span through its neighbour or a detour.
        var sourcePoints = new HashSet<(NetworkKind, float2)>();

        foreach (var item in expected)
        {
            sourcePoints.Add((item.Section.Kind, item.Section.CourseStart.xz));
            sourcePoints.Add((item.Section.Kind, item.Section.CourseEnd.xz));
        }

        var anchors = new Dictionary<string, Anchor<T>>(StringComparer.Ordinal);
        var owners = new Dictionary<T, string>();
        var ambiguousOwners = new HashSet<T>();
        var aliases = new Dictionary<string, string>(StringComparer.Ordinal);
        var conflictingAliases = new HashSet<string>(StringComparer.Ordinal);

        foreach (var item in expected)
            foreach (var alias in item.Section.SourceNodeAliases)
            {
                var sourceKey = NativeEndpointPolicy.Identity(item.Section, alias.SourceNodeId);
                var canonicalKey = NativeEndpointPolicy.Identity(item.Section, alias.CanonicalNodeId);

                if (aliases.TryGetValue(sourceKey, out var old) && old != canonicalKey)
                    conflictingAliases.Add(sourceKey);
                else
                    aliases[sourceKey] = canonicalKey;
            }

        string Canonical(string key)
        {
            // Transfer publishes resolved canonical aliases. Do not infer an
            // equivalence from coordinates, entity coincidence or alias cycles.
            return !conflictingAliases.Contains(key) && aliases.TryGetValue(key, out var canonical) ? canonical : key;
        }

        var aliasCommitted = new Dictionary<string, T>(StringComparer.Ordinal);

        foreach (var old in committed)
        {
            var key = Canonical(old.Key);

            if (key != old.Key)
            {
                if (aliasCommitted.TryGetValue(key, out var prior)
                    && !EqualityComparer<T>.Default.Equals(prior, old.Value))
                {
                    ambiguousOwners.Add(prior);
                    ambiguousOwners.Add(old.Value);
                }
                else
                    aliasCommitted[key] = old.Value;

                if (committed.TryGetValue(key, out var direct)
                    && !EqualityComparer<T>.Default.Equals(direct, old.Value))
                {
                    ambiguousOwners.Add(direct);
                    ambiguousOwners.Add(old.Value);
                }
            }

            if (owners.TryGetValue(old.Value, out var owner) && owner != key)
                ambiguousOwners.Add(old.Value);
            else
                owners[old.Value] = key;
        }

        bool Known(string key, out T value)
            => committed.TryGetValue(key, out value!) || aliasCommitted.TryGetValue(key, out value!);

        var edges = new List<Binding<T>>(expected.Count);
        var requiredNodes = new Dictionary<T, float3>();
        var used = new HashSet<T>();

        foreach (var item in expected)
        {
            var segment = item.Section;
            List<Binding<T>>? match = null;
            var count = 0;
            var visits = 0;
            var rejected = new Dictionary<string, int>(StringComparer.Ordinal);
            var witnesses = new List<object>();

            void Reject(string reason, float? observed = null, float? limit = null)
            {
                rejected[reason] = rejected.TryGetValue(reason, out var total) ? total + 1 : 1;

                if (witnesses.Count < 32)
                    witnesses.Add(new { reason, observed, limit });
            }

            var aKey = NativeEndpointPolicy.Identity(segment, segment.StartNodeId);
            var bKey = NativeEndpointPolicy.Identity(segment, segment.EndNodeId);
            var knownA = Known(aKey, out var priorA);
            var knownB = Known(bKey, out var priorB);

            // A conflict elsewhere in the city is not a failure of this unit.
            // Using that conflicted owner still requires explicit canonical
            // alias/station reconciliation; never guess an attachment.
            if (knownA && ambiguousOwners.Contains(priorA!) || knownB && ambiguousOwners.Contains(priorB!))
                throw new NativePreviewException(
                    "Required native port has conflicting source identities; unrelated units remain qualifiable.",
                    segment.WayId,
                    aKey,
                    bKey);

            if (anchors.TryGetValue(aKey, out var batchA))
            {
                knownA = true;
                priorA = batchA.Entity;
            }

            if (anchors.TryGetValue(bKey, out var batchB))
            {
                knownB = true;
                priorB = batchB.Entity;
            }

            if (item.OwnedRoute && (!knownA || !knownB))
                throw new NativePreviewException(
                    "Regenerated route requires both previously verified source owners.",
                    segment.WayId,
                    aKey,
                    bKey);

            var progress = new Dictionary<T, float>();
            var path = new List<Binding<T>>();
            var onPath = new HashSet<T>();
            // Legacy callers with no curve evidence still match source centers.
            var startPort = segment.CourseStart.xz;
            var candidates = new HashSet<T>();

            if (item.OwnedRoute && incident.ContainsKey(priorA!))
                candidates.Add(priorA!);

            if (starts.TryGetValue((segment.Kind, item.Prefab, startPort), out var portCandidates))
                candidates.UnionWith(portCandidates);

            if (starts.TryGetValue((segment.Kind, item.Prefab, segment.Start.xz), out var centerCandidates))
                foreach (var node in centerCandidates)
                    if (incident[node].Exists(edge => edge.CurveStart == null && edge.CurveEnd == null))
                        candidates.Add(node);

            foreach (var node in candidates)
            {
                if (knownA && !EqualityComparer<T>.Default.Equals(node, priorA!))
                {
                    Reject("start-owner");
                    continue;
                }

                Walk(node);

                if (count > 1)
                    break;
            }

            void Walk(T start)
            {
                // Use an explicit heap stack: a valid long native subdivision
                // chain must not cause an uncatchable managed stack overflow.
                // List-backed stack also avoids the duplicate Stack<T> facade
                // exported by the game's mixed Unity/.NET Framework references.
                var stack = new List<(T Node, float Progress, int Next)>();
                onPath.Clear();
                path.Clear();
                onPath.Add(start);
                stack.Add((start, 0, 0));

                while (stack.Count > 0 && count < 2)
                {
                    var frame = stack[stack.Count - 1];
                    stack.RemoveAt(stack.Count - 1);
                    var node = frame.Node;
                    var t = frame.Progress;

                    if (frame.Next == 0 && ++visits > 16384)
                        throw new InvalidOperationException(
                            "Native generated chain exceeds bounded matching budget; ambiguous graph refused.");

                    if (frame.Next >= incident[node].Count)
                    {
                        onPath.Remove(node);

                        if (path.Count > 0)
                            path.RemoveAt(path.Count - 1);

                        continue;
                    }

                    stack.Add((node, t, frame.Next + 1));
                    var edge = incident[node][frame.Next];

                    if (edge.Kind != segment.Kind || !EqualityComparer<T>.Default.Equals(edge.Prefab, item.Prefab))
                    {
                        Reject("mode-or-prefab");
                        continue;
                    }

                    var forward = EqualityComparer<T>.Default.Equals(node, edge.StartNode);
                    var next = forward ? edge.EndNode : edge.StartNode;

                    if (segment.OneWay && !forward)
                    {
                        Reject("direction");
                        continue;
                    }

                    if (onPath.Contains(next))
                    {
                        Reject("cycle");
                        continue;
                    }

                    var curveEvidence = edge.CurveStart.HasValue && edge.CurveEnd.HasValue;
                    var from = forward ? edge.CurveStart ?? edge.Start : edge.CurveEnd ?? edge.End;
                    var point = forward ? edge.CurveEnd ?? edge.End : edge.CurveStart ?? edge.Start;
                    var expectedStart = curveEvidence ? segment.CourseStart.xz : segment.Start.xz;
                    var expectedEnd = curveEvidence ? segment.CourseEnd.xz : segment.End.xz;

                    if (!item.OwnedRoute && path.Count == 0 && !math.all(from.xz == expectedStart))
                    {
                        Reject("first-port", math.distance(from.xz, expectedStart), 0);
                        continue;
                    }

                    if (!item.OwnedRoute && path.Count > 0)
                    {
                        var previous = path[path.Count - 1];
                        var previousPort =
                            previous.Forward
                                ? previous.Generated.CurveEnd ?? previous.Generated.End
                                : previous.Generated.CurveStart ?? previous.Generated.Start;

                        if (math.distance(previousPort.xz, from.xz) > .05f)
                        {
                            Reject("subdivision-port-continuity", math.distance(previousPort.xz, from.xz), .05f);
                            continue;
                        }
                    }

                    // Reconciled endpoint movement permits only a bounded
                    // corridor around the intended cubic, not arbitrary detours.
                    if (item.ValidateCurveCorridor || item.ReconciliationTolerance > .05f)
                    {
                        if (!edge.CurveControl1.HasValue
                            || !edge.CurveControl2.HasValue
                            || !math.all(math.isfinite(edge.CurveControl1.Value))
                            || !math.all(math.isfinite(edge.CurveControl2.Value)))
                        {
                            Reject("missing-finite-controls");
                            continue;
                        }

                        var vectorA = edge.CurveStart ?? edge.Start;
                        var d = edge.CurveEnd ?? edge.End;
                        var vectorB = edge.CurveControl1.Value;
                        var c = edge.CurveControl2.Value;
                        var valid = true;

                        for (var sample = 1; sample < 8; sample++)
                        {
                            var u = sample / 8f;
                            var v = 1 - u;
                            var otherPoint =
                                v * v * v * vectorA + 3 * v * v * u * vectorB + 3 * v * u * u * c + u * u * u * d;

                            if (SubdivisionProgress(
                                segment,
                                otherPoint.xz,
                                Math.Max(.1f, Math.Min(32, item.ReconciliationTolerance)))
                                < 0)
                            {
                                valid = false;
                                break;
                            }
                        }

                        if (!valid)
                        {
                            Reject("curve-corridor");
                            continue;
                        }
                    }

                    // This bounded center-to-port allowance is NOT proximity
                    // attachment: new ports match exactly, regenerated endpoints
                    // carry established owners, and all incidence is verified.
                    var end =
                        item.OwnedRoute
                            ? EqualityComparer<T>.Default.Equals(next, priorB!)
                            : math.all(point.xz == expectedEnd);

                    var fromBudget = path.Count == 0 ? Budget(item.StartCenterBudget, segment) : CenterBudget(segment);
                    var toBudget = end ? Budget(item.EndCenterBudget, segment) : CenterBudget(segment);

                    if (curveEvidence)
                    {
                        var fromDistance = math.distance(positions[node].xz, from.xz);
                        var toDistance = math.distance(positions[next].xz, point.xz);

                        if (fromDistance > fromBudget)
                        {
                            Reject("start-center-port-budget", fromDistance, fromBudget);
                            continue;
                        }

                        if (toDistance > toBudget)
                        {
                            Reject("end-center-port-budget", toDistance, toBudget);
                            continue;
                        }
                    }

                    if (end && knownB && !EqualityComparer<T>.Default.Equals(next, priorB!))
                    {
                        Reject("end-owner");
                        continue;
                    }

                    float nextT;

                    if (end)
                        nextT = 1;
                    else
                    {
                        if (sourcePoints.Contains((segment.Kind, point.xz)) || owners.ContainsKey(next))
                        {
                            Reject("foreign-source-junction");
                            continue;
                        }

                        if (!progress.TryGetValue(next, out nextT))
                            progress[next] =
                                nextT =
                                    SubdivisionProgress(
                                        segment,
                                        point.xz,
                                        Math.Max(.05f, Math.Min(32, item.ReconciliationTolerance)));

                        if (nextT < 0)
                        {
                            Reject("subdivision-corridor");
                            continue;
                        }
                    }

                    if (nextT <= t + 1e-6f)
                    {
                        Reject("nonmonotone-course");
                        continue;
                    }

                    path.Add(new Binding<T>(segment, edge, forward));

                    if (end)
                    {
                        count++;
                        match = new List<Binding<T>>(path);
                        path.RemoveAt(path.Count - 1);
                    }
                    else
                    {
                        onPath.Add(next);
                        stack.Add((next, nextT, 0));
                    }
                }
            }

            if (count != 1 || match == null)
            {
                var closest = new List<(float Distance, string Node, float3 Port)>();

                foreach (var edge in generated)
                {
                    if (edge.Kind != segment.Kind || !EqualityComparer<T>.Default.Equals(edge.Prefab, item.Prefab))
                        continue;

                    void Near(T node, float3 port)
                    {
                        closest.Add((math.distance(port.xz, segment.CourseStart.xz), node.ToString() ?? "", port));
                        closest.Sort((firstItem, secondItem) => firstItem.Distance.CompareTo(secondItem.Distance));

                        if (closest.Count > 8)
                            closest.RemoveAt(8);
                    }

                    Near(edge.StartNode, edge.CurveStart ?? edge.Start);
                    Near(edge.EndNode, edge.CurveEnd ?? edge.End);
                }

                var nearestPorts = new List<object>();

                foreach (var port in closest)
                    nearestPorts.Add(
                        new
                        {
                            node = port.Node,
                            horizontalDistanceMeters = port.Distance,
                            position = DiagnosticArtifacts.Point(port.Port)
                        });

                rejectedEvidence?.Invoke(new
                {
                    schemaVersion = 2,
                    way = segment.WayId,
                    start = segment.StartNodeId,
                    end = segment.EndNodeId,
                    kind = segment.Kind.ToString(),
                    qualifiedChains = count,
                    candidateStartNodes = candidates.Count,
                    knownA,
                    knownB,
                    graphEdges = generated.Count,
                    visits,
                    rejected,
                    witnesses,
                    witnessesBoundedAt = 32,
                    nearestPorts,
                    item.ReconciliationTolerance,
                    item.ValidateCurveCorridor,
                    submittedStart = DiagnosticArtifacts.Point(segment.CourseStart),
                    submittedEnd = DiagnosticArtifacts.Point(segment.CourseEnd),
                    control1 = DiagnosticArtifacts.Point(segment.CourseControl1),
                    control2 = DiagnosticArtifacts.Point(segment.CourseControl2),
                    item.StartCenterBudget,
                    item.EndCenterBudget,
                    item.OwnedRoute,
                    reason = candidates.Count == 0 ? "no-qualified-start-owner-or-port" : "missing-or-ambiguous-route"
                });

                throw new NativePreviewException(
                    $"Native generated section {segment.Kind} way {segment.WayId} nodes {segment.StartNodeId},{segment.EndNodeId}: {count} qualified horizontal/asset/incidence chains. Unsafe missing or ambiguous graph; no proximity attachment attempted.",
                    segment.WayId,
                    aKey,
                    bKey);
            }

            foreach (var part in match)
            {
                if (!used.Add(part.Generated.Entity))
                    throw new NativePreviewException(
                        "Multiple source sections resolve to the same native edge; incomplete coverage refused.",
                        segment.WayId,
                        aKey,
                        bKey);

                edges.Add(part);
                requiredNodes[part.Generated.StartNode] = part.Generated.Start;
                requiredNodes[part.Generated.EndNode] = part.Generated.End;
            }

            var first = match[0];
            var last = match[match.Count - 1];
            Bind(
                aKey,
                first.Forward ? first.Generated.StartNode : first.Generated.EndNode,
                segment.Start,
                first.Forward ? first.Generated.Start : first.Generated.End);

            Bind(
                bKey,
                last.Forward ? last.Generated.EndNode : last.Generated.StartNode,
                segment.End,
                last.Forward ? last.Generated.End : last.Generated.Start);
        }

        void Bind(string key, T entity, float3 planned, float3 actual)
        {
            if (anchors.TryGetValue(key, out var previous)
                && (!EqualityComparer<T>.Default.Equals(previous.Entity, entity)
                    || !math.all(previous.Generated == actual)))
                throw new NativePreviewException(
                    $"Native generated source junction {key} is split or has conflicting anchors.",
                    0,
                    key);

            if (owners.TryGetValue(entity, out var owner) && owner != key)
                throw new NativePreviewException(
                    $"Native node merged unrelated source identities {owner} and {key}.",
                    0,
                    owner,
                    key);

            owners[entity] = key;
            anchors[key] = new Anchor<T>(entity, planned, actual);
        }

        return new Batch<T>(anchors, edges, expected.Count, requiredNodes);
    }

    // Tolerance is ONLY for anonymous native subdivision points on a submitted
    // cubic, not source endpoint matching or connecting nearby roads. Native
    // float subdivision introduces small rounding errors at map-scale values.
    private static float SubdivisionProgress(PreparedSegment segment, float2 point, float tolerance = .05f)
    {
        var vectorA = segment.CourseStart.xz;
        var vectorB = segment.CourseControl1.xz;
        var c = segment.CourseControl2.xz;
        var d = segment.CourseEnd.xz;

        float2 At(float t)
        {
            var u = 1 - t;
            return u * u * u * vectorA + 3 * u * u * t * vectorB + 3 * u * t * t * c + t * t * t * d;
        }

        var best = 0f;
        var distance = float.PositiveInfinity;

        for (var i = 0; i <= 64; i++)
        {
            var t = i / 64f;
            var ds = math.lengthsq(At(t) - point);

            if (ds < distance)
            {
                distance = ds;
                best = t;
            }
        }

        for (var i = 0; i < 8; i++)
        {
            var u = 1 - best;
            var otherPoint = At(best) - point;
            var first = 3 * u * u * (vectorB - vectorA) + 6 * u * best * (c - vectorB) + 3 * best * best * (d - c);
            var second = 6 * u * (c - 2 * vectorB + vectorA) + 6 * best * (d - 2 * c + vectorB);
            var denominator = math.dot(first, first) + math.dot(otherPoint, second);

            if (Math.Abs(denominator) < 1e-9f)
                break;

            best = math.clamp(best - math.dot(otherPoint, first) / denominator, 0, 1);
        }

        return math.distance(At(best), point) <= tolerance ? best : -1;
    }

    public static float CenterBudget(PreparedSegment segment)
        => Math.Max(
            2,
            Math.Min(
                32,
                (math.isfinite(segment.NativeWidth) && segment.NativeWidth > 0
                    ? segment.NativeWidth
                    : segment.Kind == NetworkKind.Rail ? 12 : 32)
                    * .5f
                    + 2
                    +
        Math.Max(math.length(segment.StartCurveOffset.xz), math.length(segment.EndCurveOffset.xz))));

    private static float Budget(float value, PreparedSegment segment)
        => math.isfinite(value) && value > 0 ? Math.Min(32, value) : CenterBudget(segment);

    // Only a fully verified source/entity binding authorizes center rebasing.
    // Existing curve evidence may retain a deck port away from the node centre.
    // Rebasing the NODE anchor is not evidence that the deck moved. Preserve its
    // exact float bits until an explicit owned-unit refit replaces the design.
    public static PreparedSegment RebaseBound<T>(
        PreparedSegment segment,
        Anchor<T>? start,
        Anchor<T>? end) where T : notnull
    {
        void Verify(Anchor<T>? anchor, float3 planned)
        {
            if (anchor == null)
                return;

            if (!math.all(math.isfinite(anchor.Generated))
                || !math.isfinite(anchor.CenterLimit)
                || anchor.CenterLimit < 0
                || anchor.CenterLimit > 32
                || !math.all(anchor.Planned.xz == planned.xz)
                || math.distance(anchor.Generated.xz, planned.xz) > Budget(anchor.CenterLimit, segment))
                throw new InvalidOperationException(
                    $"Committed source binding lacks finite bounded center/port provenance; unsafe continuation refused. Live node {math.distance(anchor.Generated.xz, planned.xz):0.###} m from the plan, recorded plan {math.distance(anchor.Planned.xz, planned.xz):0.###} m away, limit {Budget(anchor.CenterLimit, segment):0.###} m.");
        }

        Verify(start, segment.Start);
        Verify(end, segment.End);
        var vectorA = start?.Generated ?? segment.Start;
        var vectorB = end?.Generated ?? segment.End;
        var deck = new Cubic3f(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd);

        return segment with { Start = vectorA, End = vectorB, FinalDeckCurve = deck };
    }

    public static Anchor<T> LocalRevisionAnchor<T>(Anchor<T> original, float3 revised) where T : notnull
    {
        if (!math.all(math.isfinite(original.Generated))
            || !math.all(math.isfinite(revised))
            || !math.all(math.isfinite(original.Planned))
            || !math.isfinite(original.CenterLimit) || original.CenterLimit <= 0 || original.CenterLimit > 32
                || math.distance(original.Generated.xz, original.Planned.xz) > original.CenterLimit
                || math.distance(original.Generated.xz, revised.xz) > original.CenterLimit)
            throw new NativeSafetyException(
                $"Local revision does not retain its owned boundary within original source provenance; live node {math.distance(original.Generated.xz, revised.xz):0.###} m from the revision, {math.distance(original.Generated.xz, original.Planned.xz):0.###} m from the original, limit {original.CenterLimit:0.###} m.");

        // This is an attachment view for one revised course. Never replace the
        // stored original footprint datum with a rolling revision centre.
        return original with { Planned = revised };
    }

    // Preview movement is qualified against ORIGINAL source coordinates, not
    // the previous preview displacement (which could authorize cumulative drift).
    public static PreparedSegment RebaseOwnedPreview<T>(
        PreparedSegment segment,
        Anchor<T>? start,
        Anchor<T>? end) where T : notnull
    {
        foreach (var anchor in new[] { start, end })
            if (anchor != null
                && (!math.all(math.isfinite(anchor.Generated)) || !math.all(math.isfinite(anchor.Planned))
                    || !math.isfinite(anchor.CenterLimit)
                    || anchor.CenterLimit <= 0
                    || anchor.CenterLimit > 32
                    || math.distance(anchor.Generated.xz, anchor.Planned.xz) > anchor.CenterLimit))
                throw new InvalidOperationException("Owned preview lacks original bounded source provenance.");

        // This is evidence of an ALREADY submitted course. A moved owner center
        // does not prove that its native curve port moved (Colorado retained it).
        // Validate the center budget without changing the immutable course.
        return segment;
    }
}
