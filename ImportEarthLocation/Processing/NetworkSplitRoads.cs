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
        for (var pass = 0; pass < 4; pass++)
        {
            var before = source.Count;
            var merged = Pass(ref source, log, token);
            log.Count("Split road stretches merged", merged);

            if (merged == 0)
                break;

            log.Diagnostic($"SPLIT ROADS pass={pass}; merged={merged}; sections {before}->{source.Count}");
        }

        return source;
    }

    private static int Pass(ref IReadOnlyList<PreparedSegment> source, ImportLog log, CancellationToken token)
    {
        var roads = source.Where(segment => segment.Kind == NetworkKind.Road).ToArray();
        var incident = new Dictionary<long, List<PreparedSegment>>();

        foreach (var segment in roads)
            foreach (var id in new[] { segment.StartNodeId, segment.EndNodeId })
            {
                if (!incident.TryGetValue(id, out var segments))
                    incident[id] = segments = new();

                segments.Add(segment);
            }

        var rail =
            new HashSet<long>(
                source
                    .Where(segment => segment.Kind == NetworkKind.Rail)
                    .SelectMany(segment => new[] { segment.StartNodeId, segment.EndNodeId }));

        var outgoing =
            roads
                .Where(Eligible)
                .GroupBy(segment => segment.StartNodeId)
                .ToDictionary(group => group.Key, group => group.ToArray());

        var incoming =
            roads
                .Where(Eligible)
                .GroupBy(segment => segment.EndNodeId)
                .ToDictionary(group => group.Key, group => group.ToArray());

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

                if (options.Length != 1 || math.dot(Heading(current), Heading(options[0])) < .5f)
                    break;

                current = options[0];
            }

            return new(sections, nodes, points, stations);
        }

        var consumed = new HashSet<PreparedSegment>();
        var moved = new Dictionary<long, (long Node, float3 Point)>();
        var created = new List<PreparedSegment>();
        var ids =
            new HashSet<long>(
                source.SelectMany(
                    segment => new[] {
                        segment.StartNodeId,
                        segment.EndNodeId
                        }.Concat(
                            segment.SourceNodeAliases.SelectMany(
                                alias => new[] { alias.SourceNodeId, alias.CanonicalNodeId }))));

        long nextNode = -8_100_000_000_000_000_000;
        long nextWay = -7_800_000_000_000_000_000;
        var ways = new HashSet<long>(source.Select(segment => segment.WayId));

        long NewNode()
        {
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

        foreach (var (seedOut, seedIn) in seeds)
        {
            token.ThrowIfCancellationRequested();

            if (rail.Contains(seedOut) || rail.Contains(seedIn))
                continue;

            foreach (var otherSegment in outgoing[seedOut])
                foreach (var segment in incoming[seedIn])
                {
                    if (consumed.Contains(otherSegment)
                        || consumed.Contains(segment)
                        || math.dot(Heading(otherSegment), -Heading(segment)) < MinimumAlignment)
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

        return merged;
    }
}
