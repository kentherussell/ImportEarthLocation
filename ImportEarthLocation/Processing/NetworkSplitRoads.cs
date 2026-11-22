using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// OSM often maps a main road as two one-way legs for a short way around a junction
// island or a pocket of median, then back to one road. AvenueConsolidator's parallel
// tests refuse most of them because the legs flare, and in game each split draws as
// two extra junctions a few metres from the crossing (New York Avenue at 395, Rosslyn).
// A pair of opposing legs leaving one node and meeting again within reach becomes one
// two-way road along their middle. Side streets on either leg move onto it.
public static class NetworkSplitRoads
{
    private const float MaximumLeg = 400,
        MaximumGap = 40,
        NearEnds = 30,
        StationGap = 12,
        ShapeSpacing = 30,
        MinimumAlignment = .6f,
        MaximumRatio = 1.6f;

    // Some junctions are mapped as a small box: the road splits into two one-way legs that
    // flare around the crossing street and rejoin just beyond it (Montreal, way 436364814).
    // Legs this short may leave the split node and turn at any angle short of a hairpin; the
    // merge then folds the crossing onto one junction of the two-way road.
    private const float CompactLeg = 50,
        CompactTurn = -.3f,
        Turn = .5f;

    private sealed record Leg(
        List<PreparedSegment> Sections,
        List<long> Nodes,
        List<float3> Points,
        List<float> Stations)
    {
        public float Length => Stations[^1];

        public float3 At(float station)
        {
            var i = Stations.BinarySearch(station);

            if (i >= 0)
                return Points[i];

            i = ~i;

            if (i <= 0)
                return Points[0];

            if (i >= Points.Count)
                return Points[^1];

            return math.lerp(
                Points[i - 1],
                Points[i],
                (station - Stations[i - 1]) / Math.Max(.001f, Stations[i] - Stations[i - 1]));
        }

        public float Project(float3 point)
        {
            var best = float.PositiveInfinity;
            var at = 0f;

            for (var i = 1; i < Points.Count; i++)
            {
                var a = Points[i - 1].xz;
                var d = Points[i].xz - a;
                var t = math.clamp(math.dot(point.xz - a, d) / Math.Max(.001f, math.lengthsq(d)), 0, 1);
                var gap = math.distancesq(point.xz, a + t * d);

                if (gap < best)
                {
                    best = gap;
                    at = math.lerp(Stations[i - 1], Stations[i], t);
                }
            }

            return at;
        }
    }

    private static bool Eligible(PreparedSegment segment)
        => segment.Kind == NetworkKind.Road
            && segment.OneWay
            && segment.Classification is "primary" or "secondary" or "tertiary"
            && !segment.Roundabout && !segment.ProtectedLink && !NetworkStructure.IsStructure(segment);

    public static IReadOnlyList<PreparedSegment> Prepare(
        IReadOnlyList<PreparedSegment> source,
        ImportLog log,
        CancellationToken token)
    {
        var joinClock = System.Diagnostics.Stopwatch.StartNew();
        source = JoinCoincidentStructures(source, log);
        log.AddDuration("Split roads coincident structures", joinClock.Elapsed.TotalSeconds);

        // Ordinary splits settle first; the compact flared ones are swept afterwards, so they
        // never take a leg an ordinary split would have used.
        foreach (var flared in new[] { false, true })
            for (var pass = 0; pass < 4; pass++)
            {
                var before = source.Count;
                var merged = Pass(ref source, flared, log, token);
                log.Count(flared ? "Flared split road junctions merged" : "Split road stretches merged", merged);

                if (merged == 0)
                    break;

                log.Diagnostic(
                    $"SPLIT ROADS pass={pass}; flared={flared}; merged={merged}; sections {before}->{source.Count}");
            }

        return source;
    }

    // The passes below leave structures alone: a bridge or tunnel is not moved onto a middle
    // line. But two opposing one-way legs on a structure can already lie on one course between
    // the same two nodes, when both of their own end junctions were folded into neighbouring
    // avenues (a four-carriageway corridor whose inner pair became one avenue: Kathmandu ways
    // 670112954 and 1091877749 on a bridge, report 117). The game cannot build one course twice:
    // the nodes later split into each leg stood on top of the other's, four dead ends. Such a
    // pair becomes one two-way section where it already is, so nothing moves. Legs further apart
    // than CoincidentTolerance anywhere along them stay as they are.
    private static IReadOnlyList<PreparedSegment> JoinCoincidentStructures(
        IReadOnlyList<PreparedSegment> source,
        ImportLog log)
    {
        const float CoincidentTolerance = .5f;

        static bool Leg(PreparedSegment segment)
            => segment.Kind == NetworkKind.Road
                && segment.OneWay
                && !segment.Roundabout
                && !segment.ProtectedLink
                && NetworkStructure.IsStructure(segment);

        static bool SameCourse(PreparedSegment first, PreparedSegment second)
            => first.Layer == second.Layer
                && first.Bridge == second.Bridge
                && first.Tunnel == second.Tunnel
                && Enumerable
                    .Range(0, 17)
                    .All(
                        j => math.distance(
                                NetworkNormalizer.Evaluate(first, j / 16f).xz,
                                NetworkNormalizer.Evaluate(second, 1 - j / 16f).xz)
                            < CoincidentTolerance);

        var byEnds = new Dictionary<(long Start, long End), int>();
        var result = new List<PreparedSegment>(source.Count);
        var joined = 0;

        foreach (var segment in source)
        {
            if (Leg(segment)
                && byEnds.TryGetValue((segment.EndNodeId, segment.StartNodeId), out var opposite)
                && SameCourse(result[opposite], segment))
            {
                var first = result[opposite];
                var lanes = NetworkSelection.MatchedLanes(first.Lanes + segment.Lanes, false, false);

                result[opposite] = first with
                {
                    SourceWayIds =
                        new[] { first, segment }
                            .SelectMany(
                                leg => leg.SourceWayIds.Count > 0 ? leg.SourceWayIds : new[] { leg.WayId })
                            .Distinct()
                            .ToArray(),
                    SourceNodeAliases =
                        first.SourceNodeAliases.Concat(segment.SourceNodeAliases).Distinct().ToArray(),
                    OriginalGeometry = first.OriginalGeometry.Concat(segment.OriginalGeometry).ToArray(),
                    OneWay = false,
                    Lanes = lanes,
                    ForwardLanes = lanes / 2,
                    BackwardLanes = lanes / 2,
                    BothWaysLanes = 0,
                    SourceWidthMeters = null,
                    ConsolidatedAvenue = lanes > 2,
                    CorridorId = 0,
                    MaximumNativeWidth = float.PositiveInfinity
                };

                byEnds.Remove((segment.EndNodeId, segment.StartNodeId));
                joined++;
                log.Diagnostic(
                    $"SPLIT ROAD coincident structure joined ways={first.WayId},{segment.WayId}; nodes={first.StartNodeId}/{first.EndNodeId}; lanes={lanes}; toleranceMeters={CoincidentTolerance}");

                continue;
            }

            if (Leg(segment))
                byEnds[(segment.StartNodeId, segment.EndNodeId)] = result.Count;

            result.Add(segment);
        }

        if (joined == 0)
            return source;

        log.Count("Coincident structure legs joined", joined);

        return result;
    }

    private static int Pass(
        ref IReadOnlyList<PreparedSegment> source,
        bool flared,
        ImportLog log,
        CancellationToken token)
    {
        var stepClock = System.Diagnostics.Stopwatch.StartNew();
        var roads = new List<PreparedSegment>();
        var incident = new Dictionary<long, List<PreparedSegment>>();
        var rail = new HashSet<long>();
        var outgoing = new Dictionary<long, List<PreparedSegment>>();
        var incoming = new Dictionary<long, List<PreparedSegment>>();

        static void AddTo(Dictionary<long, List<PreparedSegment>> lists, long node, PreparedSegment segment)
        {
            if (!lists.TryGetValue(node, out var segments))
                lists[node] = segments = new();

            segments.Add(segment);
        }

        // One sweep in source order, so every list keeps the order the queries used to give.
        foreach (var segment in source)
        {
            if (segment.Kind == NetworkKind.Rail)
            {
                rail.Add(segment.StartNodeId);
                rail.Add(segment.EndNodeId);
            }

            if (segment.Kind != NetworkKind.Road)
                continue;

            roads.Add(segment);
            AddTo(incident, segment.StartNodeId, segment);
            AddTo(incident, segment.EndNodeId, segment);

            if (!Eligible(segment))
                continue;

            AddTo(outgoing, segment.StartNodeId, segment);
            AddTo(incoming, segment.EndNodeId, segment);
        }

        static float2 Heading(PreparedSegment segment) => math.normalizesafe((segment.End - segment.Start).xz);

        Leg Walk(PreparedSegment first, bool forward)
        {
            // Stations run outward from the seed node either way.
            var sections = new List<PreparedSegment>();
            var nodes = new List<long> { forward ? first.StartNodeId : first.EndNodeId };
            var points = new List<float3> { forward ? first.Start : first.End };
            var stations = new List<float> { 0 };
            var seen = new HashSet<long>(nodes);
            var current = first;

            while (true)
            {
                var length = stations[^1] + math.distance(current.Start.xz, current.End.xz);

                if (length > MaximumLeg)
                    break;

                sections.Add(current);
                var far = forward ? current.EndNodeId : current.StartNodeId;
                nodes.Add(far);
                points.Add(forward ? current.End : current.Start);
                stations.Add(length);

                if (!seen.Add(far))
                    break;

                var options =
                    (forward ? outgoing : incoming).TryGetValue(far, out var next)
                        ? next.Where(segment => !sections.Contains(segment)).ToArray()
                        : Array.Empty<PreparedSegment>();

                if (options.Length != 1)
                    break;

                var turn = math.dot(Heading(current), Heading(options[0]));

                if (turn < Turn && (!flared || turn < CompactTurn || length > CompactLeg))
                    break;

                current = options[0];
            }

            return new(sections, nodes, points, stations);
        }

        static bool Sharp(Leg leg)
        {
            for (var k = 1; k < leg.Sections.Count; k++)
                if (math.dot(Heading(leg.Sections[k - 1]), Heading(leg.Sections[k])) < Turn)
                    return true;

            return false;
        }

        var consumed = new HashSet<PreparedSegment>();
        var moved = new Dictionary<long, (long Node, float3 Point)>();
        var created = new List<PreparedSegment>();
        // Node and way identities in use are needed only once a pass merges, so they are
        // gathered on first use; the source does not change during the pass.
        var sections = source;
        HashSet<long>? ids = null;
        HashSet<long>? ways = null;
        long nextNode = -8_100_000_000_000_000_000;
        long nextWay = -7_800_000_000_000_000_000;

        long NewNode()
        {
            if (ids == null)
            {
                ids = new HashSet<long>();

                foreach (var segment in sections)
                {
                    ids.Add(segment.StartNodeId);
                    ids.Add(segment.EndNodeId);

                    foreach (var alias in segment.SourceNodeAliases)
                    {
                        ids.Add(alias.SourceNodeId);
                        ids.Add(alias.CanonicalNodeId);
                    }
                }
            }

            while (ids.Contains(nextNode))
                nextNode++;

            ids.Add(nextNode);

            return nextNode++;
        }

        var merged = 0;
        // Legs leave one node, or two nodes a short step apart on the road they branch from
        // (Wilson Boulevard between Fort Myer Drive and Lynn Street).
        var seeds =
            outgoing.Keys.Where(incoming.ContainsKey).OrderBy(id => id).Select(id => (Out: id, In: id)).ToList();

        foreach (var segment in roads
            .Where(
                otherSegment => otherSegment.StartNodeId != otherSegment.EndNodeId
                    && math.distance(otherSegment.Start.xz, otherSegment.End.xz) <= NearEnds)
            .OrderBy(otherSegment => otherSegment.StartNodeId)
            .ThenBy(otherSegment => otherSegment.EndNodeId))
            foreach (var (x, y) in new[] {
                (segment.StartNodeId, segment.EndNodeId),
                (segment.EndNodeId, segment.StartNodeId)
                })
                if (outgoing.ContainsKey(x) && incoming.ContainsKey(y))
                    seeds.Add((x, y));

        log.AddDuration("Split roads indexes and seeds", stepClock.Elapsed.TotalSeconds);
        stepClock.Restart();

        foreach (var (seedOut, seedIn) in seeds)
        {
            token.ThrowIfCancellationRequested();

            if (rail.Contains(seedOut) || rail.Contains(seedIn))
                continue;

            foreach (var otherSegment in outgoing[seedOut])
                foreach (var segment in incoming[seedIn])
                {
                    var aligned = math.dot(Heading(otherSegment), -Heading(segment)) >= MinimumAlignment;

                    if (consumed.Contains(otherSegment) || consumed.Contains(segment) || !flared && !aligned)
                        continue;

                    var firstLeg = Walk(otherSegment, true);
                    var secondLeg = Walk(segment, false);

                    if (firstLeg.Sections.Count == 0 || secondLeg.Sections.Count == 0)
                        continue;

                    // The nearest place the two legs meet, on one node or a short step apart.
                    var ia = -1;
                    var ib = -1;
                    var bAt = new Dictionary<long, int>();

                    for (var k = 1; k < secondLeg.Nodes.Count; k++)
                        bAt.TryAdd(secondLeg.Nodes[k], k);

                    for (var k = 1; k < firstLeg.Nodes.Count && ia < 0; k++)
                        if (bAt.TryGetValue(firstLeg.Nodes[k], out var m))
                        {
                            ia = k;
                            ib = m;
                        }

                    if (ia < 0)
                    {
                        var best = NearEnds;

                        for (var k = 1; k < firstLeg.Nodes.Count; k++)
                            for (var m = 1; m < secondLeg.Nodes.Count; m++)
                            {
                                if (!incident.ContainsKey(firstLeg.Nodes[k])
                                    || !incident.ContainsKey(secondLeg.Nodes[m])
                                    || incident[firstLeg.Nodes[k]].Count < 3
                                    || incident[secondLeg.Nodes[m]].Count < 3)
                                    continue;

                                var gap = math.distance(firstLeg.Points[k].xz, secondLeg.Points[m].xz);

                                if (gap < best)
                                {
                                    best = gap;
                                    ia = k;
                                    ib = m;
                                }
                            }
                    }

                    if (ia < 0)
                        continue;

                    var legA =
                        new Leg(
                            firstLeg.Sections.Take(ia).ToList(),
                            firstLeg.Nodes.Take(ia + 1).ToList(),
                            firstLeg.Points.Take(ia + 1).ToList(),
                            firstLeg.Stations.Take(ia + 1).ToList());

                    var legB =
                        new Leg(
                            secondLeg.Sections.Take(ib).ToList(),
                            secondLeg.Nodes.Take(ib + 1).ToList(),
                            secondLeg.Points.Take(ib + 1).ToList(),
                            secondLeg.Stations.Take(ib + 1).ToList());

                    if (legA.Sections.Concat(legB.Sections).Any(consumed.Contains)
                        || legA.Sections
                            .Concat(legB.Sections)
                            .Select(secondSegment => secondSegment.Layer)
                            .Distinct()
                            .Count()
                            != 1)
                        continue;

                    // The flared sweep takes only the shapes the ordinary one could not.
                    if (flared
                        && (aligned && !Sharp(legA) && !Sharp(legB)
                            || legA.Length > CompactLeg
                            || legB.Length > CompactLeg))
                        continue;

                    if (Math.Max(legA.Length, legB.Length) / Math.Max(1, Math.Min(legA.Length, legB.Length))
                        > MaximumRatio
                        && Math.Max(legA.Length, legB.Length) > 60)
                        continue;

                    var wide = false;

                    for (var k = 0; k <= 8 && !wide; k++)
                    {
                        var point = legA.At(legA.Length * k / 8);
                        wide = math.distance(point.xz, legB.At(legB.Project(point)).xz) > MaximumGap;
                    }

                    if (wide)
                        continue;

                    var inner = legA.Nodes.Skip(1).Concat(legB.Nodes.Skip(1)).Distinct().ToArray();
                    var body = new HashSet<PreparedSegment>(legA.Sections.Concat(legB.Sections));

                    // Moved arms must be free to move: no structure, no rail, no motorway.
                    bool Fixed(long node)
                        => rail.Contains(node)
                            || moved.ContainsKey(node)
                            || incident[node].Any(
                                secondSegment => !body.Contains(secondSegment)
                                    && (NetworkStructure.IsStructure(secondSegment)
                                        || secondSegment.Classification == "motorway"
                                        || secondSegment.Roundabout));

                    var endA = legA.Nodes[^1];
                    var endB = legB.Nodes[^1];
                    var innerOfLegs = inner.Where(nodeId => nodeId != endA && nodeId != endB).ToArray();

                    if (innerOfLegs.Any(Fixed) || endA != endB && Fixed(endA) && Fixed(endB))
                        continue;

                    if (seedOut != seedIn && Fixed(seedOut) && Fixed(seedIn))
                        continue;

                    if (moved.ContainsKey(seedOut)
                        || moved.ContainsKey(seedIn)
                        || moved.ContainsKey(endA)
                        || moved.ContainsKey(endB))
                        continue;

                    // The nodes that keep their place at each end.
                    var keep =
                        endA == endB || Fixed(endA)
                            ? endA
                            : Fixed(endB) ? endB : incident[endA].Count >= incident[endB].Count ? endA : endB;

                    var far = keep == endA ? legA.Points[^1] : legB.Points[^1];
                    var seed =
                        seedOut == seedIn || Fixed(seedOut)
                            ? seedOut
                            : Fixed(seedIn)
                                ? seedIn
                                : incident[seedOut].Count >= incident[seedIn].Count ? seedOut : seedIn;

                    var near = seed == seedOut ? legA.Points[0] : legB.Points[0];

                    // A flared pair must split a two-way road that carries on at both ends: not a
                    // turning loop at the end of one, nor one-way ramps around an interchange.
                    bool TwoWayBeyond(long node)
                        => incident[node].Any(arm => !body.Contains(arm) && !arm.OneWay);

                    if (flared && (!TwoWayBeyond(seed) || !TwoWayBeyond(keep)))
                        continue;

                    if (seed == keep
                        || seedOut == endB
                        || seedIn == endA
                        || legA.Length < 8
                        || math.distance(near.xz, far.xz) < 8)
                        continue;

                    float BackOf(float t) => legB.Project(legA.At(t * legA.Length));

                    float3 Middle(float t)
                        => t <= 0 ? near : t >= 1 ? far : (legA.At(t * legA.Length) + legB.At(BackOf(t))) * .5f;

                    // Stations: side-street nodes first, then shape samples clear of them.
                    var knots = new List<(float T, long Node)> { (0, seed), (1, keep) };

                    void Knot(long node, float t)
                    {
                        if (incident[node].Count < 3)
                            return;

                        t = math.clamp(t, .001f, .999f);

                        if (knots.All(item => Math.Abs(item.T - t) * legA.Length >= StationGap))
                            knots.Add((t, node));
                    }

                    for (var k = 1; k < legA.Nodes.Count - 1; k++)
                        Knot(legA.Nodes[k], legA.Stations[k] / legA.Length);

                    for (var k = 1; k < legB.Nodes.Count - 1; k++)
                        Knot(legB.Nodes[k], legA.Project(legB.Points[k]) / legA.Length);

                    var steps = (int)Math.Ceiling(legA.Length / ShapeSpacing);

                    for (var k = 1; k < steps; k++)
                    {
                        var t = k / (float)steps;

                        if (knots.All(item => Math.Abs(item.T - t) * legA.Length >= StationGap))
                            knots.Add((t, NewNode()));
                    }

                    knots.Sort((firstItem, secondItem) => firstItem.T.CompareTo(secondItem.T));
                    var positions =
                        knots
                            .Select(item => item.Node == seed ? near : item.Node == keep ? far : Middle(item.T))
                            .ToArray();

                    // Flared legs make their middle wander; relax each shape sample toward the
                    // chord of its neighbours, as merged trunks do. Junctions stay put.
                    var pinned = knots.Select(item => incident.TryGetValue(item.Node, out var arms)).ToArray();

                    for (var pass = 0; pass < 12; pass++)
                        for (var k = 1; k < positions.Length - 1; k++)
                            if (!pinned[k])
                            {
                                var from = positions[k - 1];
                                var along = positions[k + 1] - from;
                                var t =
                                    math.clamp(
                                        math.dot((positions[k] - from).xz, along.xz)
                                            / Math.Max(.001f, math.lengthsq(along.xz)),
                                        0,
                                        1);

                                positions[k] = math.lerp(positions[k], from + t * along, .5f);
                            }

                    // Every retired node follows the nearest station.
                    (long Node, float3 Point) Nearest(float3 point)
                    {
                        var best = 0;

                        for (var k = 1; k < positions.Length; k++)
                            if (math.distancesq(positions[k].xz, point.xz)
                                < math.distancesq(positions[best].xz, point.xz))
                                best = k;

                        return (knots[best].Node, positions[best]);
                    }

                    for (var k = 1; k < knots.Count - 1; k++)
                        moved[knots[k].Node] = (knots[k].Node, positions[k]);

                    if (seedOut != seedIn)
                        moved[seed == seedOut ? seedIn : seedOut] = (seed, near);

                    foreach (var node in innerOfLegs.Append(endA).Append(endB).Distinct())
                    {
                        if (node == keep || knots.Any(item => item.Node == node))
                            continue;

                        var point =
                            node == endA
                                ? legA.Points[^1]
                                : node == endB
                                    ? legB.Points[^1]
                                    : legA.Nodes.Contains(node)
                                        ? legA.Points[legA.Nodes.IndexOf(node)]
                                        : legB.Points[legB.Nodes.IndexOf(node)];

                        moved[node] = Nearest(point);
                    }

                    if (ways == null)
                    {
                        ways = new HashSet<long>();

                        foreach (var section in sections)
                            ways.Add(section.WayId);
                    }

                    while (ways.Contains(nextWay))
                        nextWay++;

                    var way = nextWay++;
                    ways.Add(way);
                    var template =
                        legA.Sections
                            .OrderByDescending(
                                secondSegment => math.distancesq(secondSegment.Start.xz, secondSegment.End.xz))
                            .First();

                    var lanes =
                        NetworkSelection.MatchedLanes(
                            legA.Sections.Max(secondSegment => secondSegment.Lanes)
                                + legB.Sections.Max(secondSegment => secondSegment.Lanes),
                            false,
                            false);

                    static int Rank(string text) => text == "primary" ? 3 : text == "secondary" ? 2 : 1;

                    var classification =
                        body.OrderByDescending(
                            secondSegment => Rank(secondSegment.Classification)).First().Classification;

                    for (var k = 1; k < knots.Count; k++)
                    {
                        created.Add(template with
                        {
                            WayId = way,
                            StartNodeId = knots[k - 1].Node,
                            EndNodeId = knots[k].Node,
                            Start = positions[k - 1],
                            End = positions[k],
                            SourceWayIds =
                                body
                                    .SelectMany(
                                        secondSegment => secondSegment.SourceWayIds.Count > 0
                                            ? secondSegment.SourceWayIds
                                            : new[] { secondSegment.WayId })
                                    .Distinct()
                                    .ToArray(),
                            OriginalGeometry =
                                body
                                    .SelectMany(
                                        secondSegment => secondSegment.OriginalGeometry.Count > 0
                                            ? secondSegment.OriginalGeometry
                                            : new[] { secondSegment.Start, secondSegment.End })
                                    .ToArray(),
                            Classification = classification,
                            OneWay = false,
                            Lanes = lanes,
                            ForwardLanes = lanes / 2,
                            BackwardLanes = lanes / 2,
                            BothWaysLanes = 0,
                            SourceWidthMeters = null,
                            ConsolidatedAvenue = lanes > 2,
                            CorridorId = 0,
                            MaximumNativeWidth = float.PositiveInfinity,
                            SourceGeometry = new[] { positions[k - 1], positions[k] },
                            Control1 = null,
                            Control2 = null,
                            HorizontalParent = null,
                            ExplicitIncline = null
                        });
                    }

                    foreach (var secondSegment in body)
                        consumed.Add(secondSegment);

                    merged++;
                    log.Diagnostic(
                        $"SPLIT ROAD merged name={template.Name}; ways={string.Join(",", body.Select(secondSegment => secondSegment.WayId).Distinct())}; lengths={legA.Length:0}/{legB.Length:0}; at={legA.Points[0].x:0},{legA.Points[0].z:0}; farNodes={(endA == endB ? 1 : 2)}; stations={knots.Count}");

                    break;
                }
        }

        log.AddDuration("Split roads leg merging", stepClock.Elapsed.TotalSeconds);
        stepClock.Restart();

        if (merged == 0)
            return 0;

        var result = new List<PreparedSegment>(source.Count);

        foreach (var edge in source)
        {
            if (consumed.Contains(edge))
                continue;

            var segment = edge;

            if (segment.Kind == NetworkKind.Road)
            {
                if (moved.TryGetValue(segment.StartNodeId, out var firstItem))
                    segment = segment with { StartNodeId = firstItem.Node, Start = firstItem.Point };

                if (moved.TryGetValue(segment.EndNodeId, out var secondItem))
                    segment = segment with { EndNodeId = secondItem.Node, End = secondItem.Point };

                if (segment.StartNodeId == segment.EndNodeId)
                    continue;

                if (segment != edge)
                    segment =
                        segment with
                        {
                            SourceGeometry = new[] { segment.Start, segment.End },
                            Control1 = null,
                            Control2 = null,
                            HorizontalParent = null
                        };
            }

            result.Add(segment);
        }

        result.AddRange(created);
        var retired =
            moved
                .Where(entry => entry.Key != entry.Value.Node)
                .ToDictionary(entry => entry.Key, entry => entry.Value.Node);

        source = SourceTopologyObligations.Transfer(source, result, NetworkKind.Road, retired);
        log.AddDuration("Split roads rebuild and transfer", stepClock.Elapsed.TotalSeconds);

        return merged;
    }
}
