using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

/// <summary>Consolidate ordinary divided streets while retaining attached source junctions.</summary>
public static class AvenueConsolidator
{
    private sealed class Chain
    {
        public PreparedSegment[] Edges = Array.Empty<PreparedSegment>();
        public float[] Distances = Array.Empty<float>();

        public float Length => Distances[Distances.Length - 1];

        public float3 Start => Edges[0].Start;

        public float3 End => Edges[Edges.Length - 1].End;

        public float3 Point(float fraction)
        {
            var d = math.clamp(fraction, 0, 1) * Length;
            var i = Array.BinarySearch(Distances, d);

            if (i < 0)
                i = ~i - 1;

            i = Math.Max(0, Math.Min(Edges.Length - 1, i));
            var edge = Edges[i];
            var length = Distances[i + 1] - Distances[i];

            return math.lerp(edge.Start, edge.End, length <= .001f ? 0 : (d - Distances[i]) / length);
        }

        public PreparedSegment SourceAt(float fraction)
        {
            var i = Array.BinarySearch(Distances, math.clamp(fraction, 0, 1) * Length);

            if (i < 0)
                i = ~i - 1;

            return Edges[Math.Max(0, Math.Min(Edges.Length - 1, i))];
        }

        public int Lanes(float fraction) => SourceAt(fraction).Lanes;

        public PreparedSegment[] SourcesBetween(float begin, float end)
        {
            var low = Math.Min(begin, end) * Length;
            var high = Math.Max(begin, end) * Length;

            return Enumerable
                .Range(0, Edges.Length)
                .Where(i => Distances[i] < high && Distances[i + 1] > low)
                .Select(i => Edges[i])
                .ToArray();
        }

        public float Project(float3 point)
        {
            var best = float.PositiveInfinity;
            var station = 0f;

            for (var i = 0; i < Edges.Length; i++)
            {
                var segment = Edges[i];
                var delta = segment.End.xz - segment.Start.xz;
                var t =
                    math.clamp(
                        math.dot(point.xz - segment.Start.xz, delta) / Math.Max(.001f, math.lengthsq(delta)),
                        0,
                        1);

                var gap = math.distancesq(point.xz, math.lerp(segment.Start.xz, segment.End.xz, t));

                if (gap >= best)
                    continue;

                best = gap;
                station = math.lerp(Distances[i], Distances[i + 1], t);
            }

            return station / Length;
        }
    }

    // Addis Ababa, the longest seen, settles in 14 passes.
    private const int MaximumPasses = 64;

    // A pass that turns less than this share of the remaining one-way road into avenue is the
    // last of its group. Zero restores merging until a pass merges nothing.
    private const double MinimumPassGain = .002;

    // Carriageways farther apart than this (metres) at any point are separate roads in the
    // local pass. The local pass also needs this much common length (metres) and its carriageways
    // within about 18 degrees of parallel (the cosine). Until 2026-11-17 the local pass took
    // 40 m, 8 m and 23 degrees, and paired differently named streets (report 120).
    private const float MaximumSeparation = 32, LocalOverlap = 20, LocalAlignment = .95f;

    // Carriageways sharing a name or ref may stand this far apart (metres) in the named pass: a
    // wide median is still one road, and Quebec City's boulevards run 33 to 40 m apart for
    // kilometres.
    private const float NamedSeparation = 40;

    // A named pair refused for separation or alignment somewhere still merges its longest
    // stretch with carriageways at most NarrowedSeparation metres apart and parallel, if that
    // stretch is at least NarrowedMinimum metres long. The margin under NamedSeparation keeps
    // the carriageways' step onto the merged centre line at each cut short. A named pair at
    // least twice NarrowedMinimum long refused for its junctions is split at its middle node,
    // and each half is matched again on the next pass.
    private const float NarrowedSeparation = 36, NarrowedMinimum = 100;

    private static bool Eligible(PreparedSegment segment) => segment.Kind == NetworkKind.Road && segment.OneWay
        && !segment.Classification.EndsWith("_link", StringComparison.OrdinalIgnoreCase)
            && (segment.Classification == "primary"
            || segment.Classification == "secondary"
            || segment.Classification == "tertiary"
            || segment.Classification == "residential"
            || segment.Classification == "unclassified")
            && !segment.Roundabout && segment.Control1 == null && segment.Control2 == null;

    private static bool TrunkEligible(PreparedSegment segment)
        => segment.Kind == NetworkKind.Road && segment.OneWay && segment.Classification == "trunk"
            && !segment.Bridge
            && !segment.Tunnel
            && !segment.Roundabout
            && segment.Control1 == null
            && segment.Control2 == null;

    // Prefer whole named corridors before the local geometric completion pass.
    private static string Identity(PreparedSegment segment)
        => "street:" + segment.Layer + ":" + segment.Bridge + ":" + segment.Tunnel + ":" +
        (segment.Name.Trim().Length > 0
            ? "name:" + segment.Name.Trim().ToUpperInvariant()
            : segment.Reference.Trim().Length > 0
                ? "ref:" + segment.Reference.Trim().ToUpperInvariant()
                : "way:" + segment.WayId);

    private const string TrunkPairs = "Divided trunk pairs consolidated";
    // Carriageways stay divided this far from an interchange ramp or a trunk
    // structure, so each merged stretch ends with room for its transition
    // (up to 200 m, below).
    private const float InterchangeClearance = 200, StructureClearance = 200, MinimumTrunkStretch = 200;
    // A retained carriageway leaves its half of the merged end at no more than
    // this angle, then bends back onto its own alignment.
    private const float TransitionDegrees = 8;

    // Points around which trunk carriageways are not merged, with their radii.
    // A link belongs to an interchange when its connected links touch a
    // motorway or a structure, or pass near a road structure; median openings
    // and at-grade slip lanes do not.
    private static List<(float2 Point, float Radius)> TrunkClearances(IReadOnlyList<PreparedSegment> source)
    {
        var roads = source.Where(segment => segment.Kind == NetworkKind.Road).ToArray();
        var incident =
            roads
                .SelectMany(
                    segment => new[] {
                        (Id: segment.StartNodeId, Section: segment),
                        (Id: segment.EndNodeId, Section: segment)
                        })
            .GroupBy(item => item.Id)
                .ToDictionary(group => group.Key, group => group.Select(item => item.Section).ToArray());

        var structures =
            roads
                .Where(segment => (segment.Bridge || segment.Tunnel) && !segment.ProtectedLink)
                .SelectMany(
                    segment => new[] {
                        segment.Start.xz,
                        segment.End.xz,
                        math.lerp(segment.Start.xz, segment.End.xz, .5f)
                        })
                .ToArray();

        var result = new List<(float2, float)>();
        var seen = new HashSet<PreparedSegment>();

        foreach (var seed in roads.Where(segment => segment.ProtectedLink))
        {
            if (!seen.Add(seed))
                continue;

            var component = new List<PreparedSegment> { seed };

            for (var k = 0; k < component.Count; k++)
                foreach (var node in new[] { component[k].StartNodeId, component[k].EndNodeId })
                    foreach (var next in incident[node])
                        if (next.ProtectedLink && seen.Add(next))
                            component.Add(next);

            var nodes =
                component.SelectMany(segment => new[] { segment.StartNodeId, segment.EndNodeId }).Distinct().ToArray();

            var interchange =
                component.Any(
                    segment => segment.Classification.StartsWith("motorway", StringComparison.Ordinal)
                        || segment.Bridge
                        || segment.Tunnel)
                    || nodes.Any(nodeId => incident[nodeId].Any(segment => segment.Classification == "motorway"))
                        || nodes
                    .Select(
                        nodeId => incident[nodeId][0].StartNodeId == nodeId
                            ? incident[nodeId][0].Start.xz
                            : incident[nodeId][0].End.xz)
                    .Any(point => structures.Any(otherPoint => math.distancesq(point, otherPoint) <= 100 * 100));

            if (!interchange)
                continue;

            foreach (var nodeId in nodes)
                if (incident[nodeId].Any(segment => segment.Classification == "trunk"))
                    result.Add(
                        (
                            incident[nodeId][0].StartNodeId == nodeId
                                ? incident[nodeId][0].Start.xz
                                : incident[nodeId][0].End.xz,
                            InterchangeClearance));
        }

        foreach (var segment in roads.Where(
            otherSegment => otherSegment.Classification == "trunk" && (otherSegment.Bridge || otherSegment.Tunnel)))
        {
            result.Add((segment.Start.xz, StructureClearance));
            result.Add((segment.End.xz, StructureClearance));
        }

        return result;
    }

    // A retained carriageway's end node moved onto the merged centreline, but
    // its lanes continue the merged road's lanes for its own direction, beside
    // the median. Bring the carriageway onto that half: its vertices move by
    // (T-C)·(1-d/L)², where C is its original end, T the point beside the merged
    // end at the half's lane centre, and d the distance from the end. It leaves
    // the half at no more than TransitionDegrees and meets its own alignment
    // tangentially at d=L, with vertices at most 10 m apart for curve fitting.
    // Only the first piece reaches the shared node; NetworkLaneSlots moves that
    // end onto the half. Any junction passed on the way moves with it; the walk
    // stops at links, structures and other moved ends.
    private static void Transitions(
        List<PreparedSegment> sections,
        List<(int Index, bool AtStart, float3 Delta)> shifted,
        Func<long> newNode,
        ImportLog log,
        CancellationToken token)
    {
        const float spacing = 10, laneSpacing = 4;

        // Nothing moved: no section is replaced or reshaped.
        if (shifted.Count == 0)
            return;

        var incident = new Dictionary<long, List<int>>();

        void AddIncident(long id, int index)
        {
            if (!incident.TryGetValue(id, out var list))
                incident[id] = list = new();

            list.Add(index);
        }

        for (var i = 0; i < sections.Count; i++)
            if (sections[i].Kind == NetworkKind.Road)
            {
                AddIncident(sections[i].StartNodeId, i);
                AddIncident(sections[i].EndNodeId, i);
            }

        var ends =
            new HashSet<long>(
                shifted.Select(
                    item => item.AtStart ? sections[item.Index].StartNodeId : sections[item.Index].EndNodeId));

        var replaced = new Dictionary<int, PreparedSegment[]>();
        var moved = new Dictionary<long, float3>();
        var shortTransitions = 0;

        static bool Carriageway(PreparedSegment segment)
            => segment.Kind == NetworkKind.Road
                && segment.OneWay
                && (segment.Classification == "trunk" || segment.ProtectedLink)
                && !segment.Bridge
                && !segment.Tunnel;

        foreach (var (index, atStart, delta) in shifted)
        {
            token.ThrowIfCancellationRequested();
            var offset = math.length(delta.xz);

            if (offset < 1 || replaced.ContainsKey(index))
                continue;

            var node = atStart ? sections[index].StartNodeId : sections[index].EndNodeId;
            // Two-way lanes are laneSpacing apart about the axis, so one
            // direction's half of an n-lane road is centred n·laneSpacing/4 out.
            var merged =
                incident[node]
                    .Select(j => sections[j])
                    .FirstOrDefault(segment => !segment.OneWay && segment.ConsolidatedAvenue);

            var side = merged == null ? 0 : laneSpacing * (merged.Lanes / 2) / 2f;
            var target = delta * (1 - side / offset);

            if (math.length(target.xz) < 1)
                continue;

            var reach = math.clamp(2 * math.length(target.xz) / math.tan(math.radians(TransitionDegrees)), 50, 200);
            // Sections from the moved end outward; Away: the section's start is the near node.
            var run = new List<(int Index, bool Away)>();
            var current = index;
            var length = 0f;
            var forward = sections[index].StartNodeId == node;
            var stop = "";
            var heading = float2.zero;

            while (true)
            {
                var segment = sections[current];
                var away = segment.StartNodeId == node;
                run.Add((current, away));
                length += math.distance(segment.Start.xz, segment.End.xz);

                // Slicing can leave a sub-metre piece with no direction of its own.
                if (math.distance(segment.Start.xz, segment.End.xz) >= 1)
                    heading = math.normalize((segment.End - segment.Start).xz);

                var far = away ? segment.EndNodeId : segment.StartNodeId;
                stop =
                    length >= reach
                        ? ""
                        : ends.Contains(far)
                            ? "merged-end"
                            : !segment.ProtectedLink && incident[far].Any(j => sections[j].ProtectedLink) ? "link" :
                    incident[far].Any(j => NetworkStructure.IsStructure(sections[j])) ? "structure" : "";

                if (length >= reach || stop.Length > 0)
                    break;

                // A one-way carriageway continues in its own travel direction.
                var next =
                    incident[far]
                        .Where(
                            j => j != current
                                && !replaced.ContainsKey(j)
                                && Carriageway(sections[j])
                                && (sections[j].StartNodeId == far) == forward
                                && (heading.Equals(float2.zero)
                        || math.dot(heading, math.normalizesafe((sections[j].End - sections[j].Start).xz)) > .7f))
                        .ToArray();

                if (next.Length != 1)
                {
                    stop = "branch";
                    break;
                }

                current = next[0];
                node = far;
            }

            if (length < reach)
            {
                var end = atStart ? sections[index].Start : sections[index].End;
                log.Diagnostic(
                    $"TRUNK AVENUE TRANSITION limited way={sections[index].WayId}; at={end.x:0},{end.z:0}; offset={offset:0.0}; needed={reach:0}; available={length:0}; stop={stop}");

                reach = length;
                shortTransitions++;
            }

            var d = 0f;

            foreach (var (j, away) in run)
            {
                var segment = sections[j];
                var near = away ? segment.Start : segment.End;
                var far = away ? segment.End : segment.Start;
                var farNode = away ? segment.EndNodeId : segment.StartNodeId;
                var node0 = near;

                if (j == index)
                    near -= delta;

                var span = math.distance(near.xz, far.xz);

                float3 Shifted(float3 point, float t)
                {
                    var w = t >= reach ? 0 : (1 - t / reach) * (1 - t / reach);
                    return point + target * w;
                }

                var points = new List<float3> { j == index ? node0 : Shifted(near, d) };

                for (var t = spacing; t < span && d + t < reach; t += spacing)
                    points.Add(Shifted(math.lerp(near, far, t / span), d + t));

                points.Add(Shifted(far, d + span));

                if (d + span < reach)
                    moved[farNode] = points[^1];

                var ids = new long[points.Count];
                ids[0] = away ? segment.StartNodeId : segment.EndNodeId;
                ids[^1] = farNode;

                for (var k = 1; k < ids.Length - 1; k++)
                    ids[k] = newNode();

                if (!away)
                {
                    points.Reverse();
                    Array.Reverse(ids);
                }

                replaced[j] = Enumerable.Range(1, points.Count - 1).Select(k => segment with
                {
                    Start = points[k - 1],
                    End = points[k],
                    StartNodeId = ids[k - 1],
                    EndNodeId = ids[k],
                    SourceGeometry = new[] { points[k - 1], points[k] },
                    Control1 = null,
                    Control2 = null,
                    HorizontalParent = null,
                    StartTerminal = k == 1 && segment.StartTerminal,
                    EndTerminal = k == points.Count - 1 && segment.EndTerminal,
                    StartRoundaboutRadius = k == 1 ? segment.StartRoundaboutRadius : 0,
                    EndRoundaboutRadius = k == points.Count - 1 ? segment.EndRoundaboutRadius : 0
                }).ToArray();

                d += span;
            }
        }

        if (replaced.Count == 0)
            return;

        var output = new List<PreparedSegment>(sections.Count + replaced.Values.Sum(segments => segments.Length));

        for (var i = 0; i < sections.Count; i++)
        {
            if (replaced.TryGetValue(i, out var pieces))
            {
                output.AddRange(pieces);
                continue;
            }

            var segment = sections[i];

            if (moved.TryGetValue(segment.StartNodeId, out var point))
                segment =
                    segment with
                    {
                        Start = point,
                        SourceGeometry = new[] { point }.Concat(segment.SourceGeometry.Skip(1)).ToArray()
                    };

            if (moved.TryGetValue(segment.EndNodeId, out var otherPoint))
                segment =
                    segment with
                    {
                        End = otherPoint,
                        SourceGeometry =
                            segment.SourceGeometry
                                .Take(Math.Max(1, segment.SourceGeometry.Count - 1))
                                .Concat(new[] { otherPoint })
                                .ToArray()
                    };

            output.Add(segment);
        }

        sections.Clear();
        sections.AddRange(output);
        log.Count("Divided trunk carriageway sections reshaped", replaced.Count);
        log.Count("Divided trunk transitions limited by nearby junctions or structures", shortTransitions);
    }

    private static float DistanceToSegment(float2 point, float2 vectorA, float2 vectorB)
    {
        var d = vectorB - vectorA;
        var t = math.clamp(math.dot(point - vectorA, d) / Math.Max(.001f, math.lengthsq(d)), 0, 1);

        return math.distance(point, vectorA + t * d);
    }

    // A real merge turns one-way road into avenue, so one-way length must fall. Tails and
    // remapped ends can restore it instead, which on Addis Ababa alternated between two
    // states forever while each pass stacked more avenue, so such a pass is discarded.
    private static IReadOnlyList<PreparedSegment> RepeatPasses(
        IReadOnlyList<PreparedSegment> source,
        ImportLog log,
        CancellationToken token,
        bool localMatching)
    {
        const string Merges = "Opposing road pairs consolidated into avenues";
        const string Junctions = "Avenue median junction connectors canonicalized";

        // Each pass's result is the next pass's source, so its length carries over.
        var after = OneWayLength(source);

        // Nodes where named carriageway chains are cut, kept across passes (NarrowedMinimum).
        var breaks = new HashSet<long>();

        for (var pass = 0; pass < MaximumPasses; pass++)
        {
            var merges = log.Metric(Merges);
            var junctions = log.Metric(Junctions);
            var breaksBefore = breaks.Count;
            var next = PreparePass(source, log, token, localMatching, breaks: breaks);
            var merged = log.Metric(Merges) != merges;
            var bisected = breaks.Count != breaksBefore;
            var before = after;
            after = OneWayLength(next);

            log.Diagnostic(
                $"AVENUE TIMES pass={pass + 1}; local={localMatching}; merges={log.Metric(Merges) - merges:0}; junctionConnectors={log.Metric(Junctions) - junctions:0}; oneWayKilometres={before / 1000:0.0}->{after / 1000:0.0}; sections={source.Count}->{next.Count}");

            if (merged && after >= before)
            {
                log.Warn($"Avenue merging stopped at pass {pass + 1}: the last pass reduced no one-way road.");

                return source;
            }

            source = next;

            if (!merged && !bisected && (!localMatching || log.Metric(Junctions) == junctions))
                return source;

            // Each pass costs the same whatever it merges, and the late ones merge next to
            // nothing: in Paris passes 5 to 7 of each group merged 1.3 km of 2,100 km for 40% of
            // the stage's time (report 119). The pass is kept; the group ends with it, unless it
            // split chains whose halves are still to be matched.
            if (merged && !bisected && before - after < MinimumPassGain * before)
                return source;
        }

        log.Warn($"Avenue merging stopped at the limit of {MaximumPasses} passes.");

        return source;
    }

    // Merged centres are midpoints of two carriageways whose nodes rarely line up, and local
    // merges are often a single section long, so the zigzag runs across many merges. This
    // relaxes each node toward the chord of its avenue neighbours along the whole avenue.
    // Shifts are horizontal and bounded (metres from the merged position). Corners, ends and
    // structures hold, and so do junctions: moving one bends the cross street through it.
    internal static IReadOnlyList<PreparedSegment> SmoothAvenues(
        IReadOnlyList<PreparedSegment> source,
        ImportLog log,
        CancellationToken token)
    {
        const float MaximumCorner = 45, MaximumShift = 3;
        const int Passes = 12;

        var arms = new Dictionary<long, List<PreparedSegment>>();

        foreach (var segment in source)
            foreach (var node in new[] { segment.StartNodeId, segment.EndNodeId })
            {
                if (!arms.TryGetValue(node, out var list))
                    arms[node] = list = new List<PreparedSegment>();

                list.Add(segment);
            }

        var positions = new Dictionary<long, float3>();
        var free = new List<(long Node, long Before, long After, float Limit, float2 Merged)>();

        foreach (var entry in arms.OrderBy(entry => entry.Key))
        {
            var node = entry.Key;
            var list = entry.Value;
            var avenue = list.Where(segment => segment.ConsolidatedAvenue && !segment.OneWay).ToArray();

            if (avenue.Length != 2
                || list.Count != 2
                || list.Any(segment => segment.Kind != NetworkKind.Road
                    || segment.Roundabout
                    || segment.StartNodeId == segment.EndNodeId
                    || NetworkStructure.IsStructure(segment)))
                continue;

            var point = avenue[0].StartNodeId == node ? avenue[0].Start : avenue[0].End;
            var before = avenue[0].StartNodeId == node ? avenue[0].End : avenue[0].Start;
            var after = avenue[1].StartNodeId == node ? avenue[1].End : avenue[1].Start;
            var beforeNode = avenue[0].StartNodeId == node ? avenue[0].EndNodeId : avenue[0].StartNodeId;
            var afterNode = avenue[1].StartNodeId == node ? avenue[1].EndNodeId : avenue[1].StartNodeId;

            if (beforeNode == afterNode || Angle((point - before).xz, (after - point).xz) > MaximumCorner)
                continue;

            positions[node] = point;
            positions[beforeNode] = before;
            positions[afterNode] = after;
            free.Add((node, beforeNode, afterNode, MaximumShift, point.xz));
        }

        for (var pass = 0; pass < Passes; pass++)
        {
            token.ThrowIfCancellationRequested();

            foreach (var item in free)
            {
                var start = positions[item.Before].xz;
                var chord = positions[item.After].xz - start;
                var current = positions[item.Node];
                var along = math.clamp(math.dot(current.xz - start, chord) / Math.Max(.001f, math.lengthsq(chord)), 0, 1);
                var target = math.lerp(current.xz, start + along * chord, .5f);
                var shift = target - item.Merged;
                var distance = math.length(shift);

                if (distance > item.Limit)
                    target = item.Merged + shift * (item.Limit / distance);

                positions[item.Node] = new float3(target.x, current.y, target.y);
            }
        }

        var moved = new Dictionary<long, float3>();

        foreach (var item in free)
            if (math.distance(positions[item.Node].xz, item.Merged) > .01f)
                moved[item.Node] = positions[item.Node];

        if (moved.Count == 0)
            return source;

        log.Count("Avenue nodes smoothed", moved.Count);

        return source
            .Select(
                segment =>
                {
                    var hasStart = moved.TryGetValue(segment.StartNodeId, out var start);
                    var hasEnd = moved.TryGetValue(segment.EndNodeId, out var end);

                    if (!hasStart && !hasEnd)
                        return segment;

                    var first = hasStart ? start : segment.Start;
                    var last = hasEnd ? end : segment.End;

                    return segment with
                    {
                        Start = first,
                        End = last,
                        SourceGeometry = new[] { first, last },
                        Control1 = null,
                        Control2 = null,
                        HorizontalParent = null
                    };
                })
            .ToArray();
    }

    // Two avenue pieces merged separately can end a few metres apart sideways, joined only by
    // what is left of the carriageways: two opposite one-way stubs between the two ends. The
    // design drew that step as an S-bend of 10-20 m radius (Central Avenue in DC; 191 such
    // steps in DC, 130 in Paris, 22 in Marseille; report 120). Where each end node carries
    // one avenue piece (a different one at each end) besides the two stubs, the stubs go and
    // the two ends join at their midpoint, so the avenue continues; SmoothAvenues then relaxes
    // the join. A side street at either end moves with it, by under MaximumStub / 2, and meets
    // the avenue at the one junction instead of two a few metres apart; a bridge or tunnel
    // step joins like any other. Until 11-18 a side street or a structure kept the step, and
    // Quebec City had 26 of these pieces at its boulevards' junctions and bridge ends. The
    // removed node stays known as an alias of the one kept.
    internal static IReadOnlyList<PreparedSegment> JoinAvenueSteps(
        IReadOnlyList<PreparedSegment> source,
        ImportLog log)
    {
        const float MaximumStub = 20;

        var arms = new Dictionary<long, List<int>>();

        for (var i = 0; i < source.Count; i++)
            if (source[i].Kind == NetworkKind.Road)
                foreach (var node in new[] { source[i].StartNodeId, source[i].EndNodeId })
                {
                    if (!arms.TryGetValue(node, out var list))
                        arms[node] = list = new List<int>();

                    list.Add(i);
                }

        bool Stub(PreparedSegment segment)
            => segment.Kind == NetworkKind.Road
                && segment.OneWay
                && !segment.ConsolidatedAvenue
                && !segment.Roundabout
                && math.distance(segment.Start.xz, segment.End.xz) < MaximumStub;

        bool AvenuePiece(PreparedSegment segment) => segment.ConsolidatedAvenue && !segment.OneWay;

        var removed = new HashSet<int>();
        var moved = new Dictionary<long, (long Node, float3 Point)>();

        // A stub may run through nodes with nothing else attached (Quebec City's Chemin des
        // Quatre-Bourgeois: 10 m and 8 m east, one 7 m stub back west). Its sections in order
        // from one in its first, while they stay stubs, under MaximumStub together, and four
        // at most; null where it does not continue as a stub.
        List<int>? Run(int start)
        {
            var run = new List<int> { start };
            var length = math.distance(source[start].Start.xz, source[start].End.xz);
            var at = source[start].EndNodeId;

            while (arms[at].Count == 2)
            {
                var next = arms[at][0] == run[run.Count - 1] ? arms[at][1] : arms[at][0];

                if (removed.Contains(next) || !Stub(source[next]) || source[next].StartNodeId != at || run.Count == 4)
                    return null;

                length += math.distance(source[next].Start.xz, source[next].End.xz);

                if (length >= MaximumStub)
                    return null;

                run.Add(next);
                at = source[next].EndNodeId;
            }

            return run;
        }

        for (var i = 0; i < source.Count; i++)
        {
            var stub = source[i];

            if (removed.Contains(i) || !Stub(stub) || stub.StartNodeId == stub.EndNodeId)
                continue;

            var outward = Run(i);

            if (outward == null)
                continue;

            var first = stub.StartNodeId;
            var second = source[outward[outward.Count - 1]].EndNodeId;

            if (first == second || moved.ContainsKey(first) || moved.ContainsKey(second))
                continue;

            List<int>? inward = null;

            foreach (var j in arms[second])
                if (!outward.Contains(j)
                    && !removed.Contains(j)
                    && Stub(source[j])
                    && source[j].StartNodeId == second
                    && Run(j) is { } candidate
                    && source[candidate[candidate.Count - 1]].EndNodeId == first)
                {
                    inward = candidate;
                    break;
                }

            if (inward == null)
                continue;

            var step = new HashSet<int>(outward.Concat(inward));
            var firstPieces = arms[first].Where(j => !step.Contains(j) && AvenuePiece(source[j])).ToArray();
            var secondPieces = arms[second].Where(j => !step.Contains(j) && AvenuePiece(source[j])).ToArray();

            if (firstPieces.Length != 1
                || secondPieces.Length != 1
                || firstPieces[0] == secondPieces[0]
                || removed.Contains(firstPieces[0])
                || removed.Contains(secondPieces[0]))
                continue;

            var point = (stub.Start + source[outward[outward.Count - 1]].End) * .5f;
            removed.UnionWith(step);

            foreach (var j in step)
            {
                moved[source[j].StartNodeId] = (first, point);
                moved[source[j].EndNodeId] = (first, point);
            }
        }

        if (removed.Count == 0)
            return source;

        log.Count("Avenue steps joined", removed.Count / 2);

        // An end moves with its adjacent curve control, so the piece keeps its direction there.
        var result = new List<PreparedSegment>();

        for (var i = 0; i < source.Count; i++)
        {
            if (removed.Contains(i))
                continue;

            var segment = source[i];

            if (moved.TryGetValue(segment.StartNodeId, out var start))
            {
                var delta = start.Point - segment.Start;
                segment = segment with
                {
                    StartNodeId = start.Node,
                    Start = start.Point,
                    Control1 = segment.Control1 + delta,
                    SourceGeometry = MoveEnd(segment.SourceGeometry, start.Point, true),
                    HorizontalParent = null
                };
            }

            if (moved.TryGetValue(segment.EndNodeId, out var end))
            {
                var delta = end.Point - segment.End;
                segment = segment with
                {
                    EndNodeId = end.Node,
                    End = end.Point,
                    Control2 = segment.Control2 + delta,
                    SourceGeometry = MoveEnd(segment.SourceGeometry, end.Point, false),
                    HorizontalParent = null
                };
            }

            result.Add(segment);
        }

        static IReadOnlyList<float3> MoveEnd(IReadOnlyList<float3> geometry, float3 point, bool atStart)
        {
            if (geometry.Count == 0)
                return geometry;

            var copy = geometry.ToArray();
            copy[atStart ? 0 : copy.Length - 1] = point;

            return copy;
        }

        var aliases = new Dictionary<long, long>();

        foreach (var pair in moved)
            if (pair.Key != pair.Value.Node)
                aliases[pair.Key] = pair.Value.Node;

        return SourceTopologyObligations.Transfer(source, result, NetworkKind.Road, aliases);
    }

    private static double OneWayLength(IReadOnlyList<PreparedSegment> sections)
        => sections.Where(segment => segment.OneWay).Sum(segment => (double)math.distance(segment.Start.xz, segment.End.xz));

    public static IReadOnlyList<PreparedSegment> Prepare(
        IReadOnlyList<PreparedSegment> source,
        ImportLog log,
        CancellationToken token)
    {
        var clock = Stopwatch.StartNew();
        source = StreetIntersectionConsolidator.Prepare(source, log, token);
        log.Diagnostic($"AVENUE TIMES streetIntersections={clock.Elapsed.TotalSeconds:0.00}");

        // One long carriageway may face several disconnected opposing portions.
        // Revisit retained tails after each pass.
        source = RepeatPasses(source, log, token, localMatching: false);

        // Finish locally: an imperfect bend or unequal intersection approach
        // must not veto the otherwise parallel portions of a divided street.
        // Geometry, rather than naming, owns these remaining short matches.
        source = RepeatPasses(source, log, token, localMatching: true);
        source = JoinAvenueSteps(source, log);
        source = SmoothAvenues(source, log, token);

        return source
            .Select(
                segment => segment.ConsolidatedAvenue && !segment.OneWay && segment.Lanes <= 2
                    ? segment with { ConsolidatedAvenue = false }
                    : segment)
            .ToArray();
    }

    private static IReadOnlyList<PreparedSegment> PreparePass(
        IReadOnlyList<PreparedSegment> source,
        ImportLog log,
        CancellationToken token,
        bool localMatching = false,
        bool trunk = false,
        HashSet<long> breaks = null)
    {
        token.ThrowIfCancellationRequested();
        var clock = Stopwatch.StartNew();

        // Merging trunks moves their own junctions; motorways stay fixed.
        bool Fixed(PreparedSegment segment)
            => segment.Kind == NetworkKind.Road
                && (segment.Classification == "motorway"
                    || !trunk && segment.Classification == "trunk"
                    || localMatching && NativeAssetKeys.IsHighway(segment));

        // Plain loops for the lookups built over the whole network on every pass: there are
        // up to 64 passes (report 114).
        var fixedPorts = new HashSet<long>();
        var structureNodes = new HashSet<long>();

        foreach (var segment in source)
        {
            if (Fixed(segment))
            {
                fixedPorts.Add(segment.StartNodeId);
                fixedPorts.Add(segment.EndNodeId);
            }

            if (segment.Kind == NetworkKind.Rail)
            {
                structureNodes.Add(segment.StartNodeId);
                structureNodes.Add(segment.EndNodeId);
            }
        }

        var clearances = trunk ? TrunkClearances(source) : new List<(float2 Point, float Radius)>();

        bool Clear(PreparedSegment segment)
            => clearances.All(item => DistanceToSegment(item.Point, segment.Start.xz, segment.End.xz) > item.Radius);

        var railPorts =
            source
                .Where(segment => segment.Kind == NetworkKind.Rail)
                .SelectMany(
                    segment => new[] {
                        (Node: segment.StartNodeId, Edge: segment),
                        (Node: segment.EndNodeId, Edge: segment)
                        })
            .GroupBy(item => item.Node)
                .ToDictionary(group => group.Key, group => group.Select(item => item.Edge).ToArray());

        bool RailPort(long node)
        {
            if (!localMatching
                || !railPorts.TryGetValue(node, out var edges)
                || edges.Length != 2
                || edges.Any(NetworkStructure.IsStructure)
                || edges[0].Layer != edges[1].Layer)
                return false;

            if (edges[0].Classification != edges[1].Classification
                || edges[0].PhysicalTracks != edges[1].PhysicalTracks
                || edges[0].OneWay != edges[1].OneWay
                || edges[0].ExplicitIncline != edges[1].ExplicitIncline
                || edges.Any(
                    segment => segment.RailService.Length > 0
                        || (segment.StartNodeId == node ? segment.StartTerminal : segment.EndTerminal)))
                return false;

            if (edges[0].OneWay && (edges[0].StartNodeId == node) == (edges[1].StartNodeId == node))
                return false;

            float2 Away(PreparedSegment segment)
                => math.normalizesafe(
                    (segment.StartNodeId == node ? segment.End - segment.Start : segment.Start - segment.End).xz);

            return math.dot(Away(edges[0]), Away(edges[1])) < -.995f;
        }

        // Keep the actual structural tie-in edge, not an entire avenue just because
        // a distant end meets a bridge. The retained edge becomes the transition.
        var unordered =
            source
                .Where(
                    segment => (trunk ? TrunkEligible(segment) && Clear(segment) : Eligible(segment))
                        && (!localMatching || !NetworkStructure.IsStructure(segment))
                        && (!structureNodes.Contains(segment.StartNodeId) || RailPort(segment.StartNodeId))
                && (!structureNodes.Contains(segment.EndNodeId) || RailPort(segment.EndNodeId)))
                .ToArray();

        // Each identity is built once, for the sort and for the corridor groups below.
        var unorderedIdentities = new string[unordered.Length];

        for (var i = 0; i < unordered.Length; i++)
            unorderedIdentities[i] = Identity(unordered[i]);

        var order =
            Enumerable
                .Range(0, unordered.Length)
                .OrderBy(i => unorderedIdentities[i], StringComparer.Ordinal)
                .ThenBy(i => unordered[i].WayId)
                .ThenBy(i => unordered[i].StartNodeId)
                .ToArray();

        var eligible = new PreparedSegment[order.Length];
        var identities = new string[order.Length];

        for (var k = 0; k < order.Length; k++)
        {
            eligible[k] = unordered[order[k]];
            identities[k] = unorderedIdentities[order[k]];
        }

        if (trunk)
            log.Diagnostic(
                $"TRUNK AVENUE CLEARANCES points={clearances.Count}; carriageways={source.Count(TrunkEligible)}; clear={eligible.Length}");

        if (eligible.Length == 0)
            return source;

        // Named corridors stop where a two-way road of their own identity continues them.
        // Only that chaining reads these ports, and only at the nodes of eligible sections,
        // so the identity of any other two-way road is never needed.
        var twoWayPorts = new HashSet<(long Node, string Identity)>();

        if (!localMatching)
        {
            var eligibleNodes = new HashSet<long>();

            foreach (var segment in eligible)
            {
                eligibleNodes.Add(segment.StartNodeId);
                eligibleNodes.Add(segment.EndNodeId);
            }

            foreach (var segment in source)
                if (segment.Kind == NetworkKind.Road
                    && !segment.OneWay
                    && (eligibleNodes.Contains(segment.StartNodeId) || eligibleNodes.Contains(segment.EndNodeId)))
                {
                    var identity = Identity(segment);
                    twoWayPorts.Add((segment.StartNodeId, identity));
                    twoWayPorts.Add((segment.EndNodeId, identity));
                }
        }

        var chains = new List<Chain>();

        if (localMatching)
            foreach (var edge in eligible)
            {
                var length = math.distance(edge.Start.xz, edge.End.xz);

                if (math.isfinite(length) && length >= 8)
                    chains.Add(new Chain { Edges = new[] { edge }, Distances = new[] { 0f, length } });
            }
        else
        {
            // Sorted by identity, so each corridor is one run of equal identities, in the
            // order and with the sections the former GroupBy gave.
            static Dictionary<long, List<PreparedSegment>> ByNode(
                PreparedSegment[] edges,
                Func<PreparedSegment, long> node)
            {
                var result = new Dictionary<long, List<PreparedSegment>>();

                foreach (var edge in edges)
                {
                    if (!result.TryGetValue(node(edge), out var list))
                        result[node(edge)] = list = new List<PreparedSegment>();

                    list.Add(edge);
                }

                return result;
            }

            var groupEnd = 0;

            for (var groupStart = 0; groupStart < eligible.Length; groupStart = groupEnd)
            {
                token.ThrowIfCancellationRequested();
                var identity = identities[groupStart];
                groupEnd = groupStart + 1;

                while (groupEnd < eligible.Length && identities[groupEnd] == identity)
                    groupEnd++;

                var edges = new PreparedSegment[groupEnd - groupStart];
                Array.Copy(eligible, groupStart, edges, 0, edges.Length);
                var outgoing = ByNode(edges, segment => segment.StartNodeId);
                var incoming = ByNode(edges, segment => segment.EndNodeId);
                var visited = new HashSet<PreparedSegment>();

                foreach (var seed in edges
                    .Where(
                        segment => twoWayPorts.Contains((segment.StartNodeId, identity))
                            || breaks != null && breaks.Contains(segment.StartNodeId)
                            || !incoming.TryGetValue(segment.StartNodeId, out var before)
                            || before.Count != 1
                            || outgoing[segment.StartNodeId].Count != 1)
                    .Concat(edges))
                {
                    if (visited.Contains(seed))
                        continue;

                    var list = new List<PreparedSegment>();
                    var current = seed;

                    while (visited.Add(current))
                    {
                        token.ThrowIfCancellationRequested();
                        list.Add(current);

                        if (twoWayPorts.Contains((current.EndNodeId, identity))
                            || breaks != null && breaks.Contains(current.EndNodeId)
                            || !outgoing.TryGetValue(current.EndNodeId, out var next))
                            break;

                        var direction = math.normalizesafe(current.End.xz - current.Start.xz);

                        if (next.Count == 1 && incoming[current.EndNodeId].Count == 1)
                        {
                            if (math.dot(direction, math.normalizesafe(next[0].End.xz - next[0].Start.xz)) < .5f)
                                break;

                            current = next[0];
                            continue;
                        }

                        // Keep a clearly aligned carriageway as one matching
                        // corridor across side-street junctions. Junction stations
                        // remain in SourcesBetween/Stations and are remapped later.
                        var ranked =
                            next
                                .Where(
                                    segment => !visited.Contains(segment)
                                        && segment.Bridge == current.Bridge
                                        && segment.Tunnel == current.Tunnel)
                            .Select(
                                segment => (
                                    edge: segment,
                                    dot: math.dot(direction, math.normalizesafe(segment.End.xz - segment.Start.xz))))
                            .OrderByDescending(item => item.dot).ToArray();

                        if (ranked.Length == 0
                            || ranked[0].dot < .95f
                            || ranked.Length > 1 && ranked[0].dot - ranked[1].dot < .05f)
                            break;

                        current = ranked[0].edge;
                    }

                    if (list.Count == 0 || list[0].StartNodeId == list[list.Count - 1].EndNodeId)
                        continue;

                    var distances = new float[list.Count + 1];

                    for (var i = 0; i < list.Count; i++)
                        distances[i + 1] = distances[i] + math.distance(list[i].Start.xz, list[i].End.xz);

                    if (distances[distances.Length - 1] >= 20 && distances.All(math.isfinite))
                        chains.Add(new Chain { Edges = list.ToArray(), Distances = distances });
                }
            }
        }

        // Search near the opposite chain's end, not all pairs or only their midpoints.
        var grid = new Dictionary<(int, int), List<int>>();

        for (var i = 0; i < chains.Count; i++)
        {
            var steps = Math.Max(1, (int)Math.Ceiling(chains[i].Length / 48));

            for (var k = 0; k <= steps; k++)
            {
                var point = chains[i].Point(k / (float)steps);
                var key = ((int)Math.Floor(point.x / 64), (int)Math.Floor(point.z / 64));

                if (!grid.TryGetValue(key, out var bucket))
                    grid[key] = bucket = new List<int>();

                if (bucket.Count == 0 || bucket[bucket.Count - 1] != i)
                    bucket.Add(i);
            }
        }

        var chainSeconds = clock.Elapsed.TotalSeconds;

        var matches =
            new List<(int other, float score, float a0, float a1, float b0, float b1, bool tapered)>[chains.Count];

        var mirrored =
            new ConcurrentBag<(int Owner, (int other, float score, float a0, float a1, float b0, float b1, bool tapered)
                Match)>();


        var comparisons = 0;
        var rejectedSamples = 0;
        var rejections = new Dictionary<string, int>();

        void Reject(Chain firstChain, Chain secondChain, string reason, float value = 0)
        {
            lock (rejections)
            {
                rejections[reason] = rejections.TryGetValue(reason, out var count) ? count + 1 : 1;

                // Every pair kept apart, by way, so a road reported in game can be looked up.
                if (reason is not ("overlap" or "separation" or "tangent") || rejectedSamples < 60)
                    log.Diagnostic(
                        $"AVENUE REJECT name={firstChain.Edges[0].Name}; sourceWay={firstChain.Edges[0].WayId}; candidateWay={secondChain.Edges[0].WayId}; reason={reason}; value={value:0.###}; lengths={firstChain.Length:0.##}/{secondChain.Length:0.##}; at={firstChain.Point(.5f).x:0},{firstChain.Point(.5f).z:0}");

                rejectedSamples++;
            }
        }

        // Cuts a long named chain refused for its junctions at the node nearest its middle.
        void Bisect(Chain chain)
        {
            if (breaks == null || localMatching || chain.Length < 2 * NarrowedMinimum || chain.Edges.Length < 2)
                return;

            var half = chain.Length / 2;
            var middle = 1;

            for (var k = 2; k < chain.Edges.Length; k++)
                if (Math.Abs(chain.Distances[k] - half) < Math.Abs(chain.Distances[middle] - half))
                    middle = k;

            breaks.Add(chain.Edges[middle].StartNodeId);
        }

        // Each chain's matches depend only on the chains, so they run in parallel; matches[i] is per chain and the result does not depend on the order.
        try
        {
            Parallel.For(0, chains.Count, new ParallelOptions { CancellationToken = token }, i =>
                    {
                        token.ThrowIfCancellationRequested();
                        matches[i] = new();
                        var firstChain = chains[i];
                        var nearby = new HashSet<int>();
                        var searchSteps = Math.Max(1, (int)Math.Ceiling(firstChain.Length / 48));

                        for (var k = 0; k <= searchSteps; k++)
                        {
                            var point = firstChain.Point(k / (float)searchSteps);
                            var x = (int)Math.Floor(point.x / 64);
                            var z = (int)Math.Floor(point.z / 64);

                            for (var dz = -1; dz <= 1; dz++)
                                for (var dx = -1; dx <= 1; dx++)
                                    if (grid.TryGetValue((x + dx, z + dz), out var bucket))
                                        nearby.UnionWith(bucket);
                        }

                        foreach (var j in nearby.OrderBy(v => v))
                        {
                            if (i == j)
                                continue;

                            var secondChain = chains[j];
                            Interlocked.Increment(ref comparisons);
                            var first = firstChain.Edges[0];
                            var second = secondChain.Edges[0];
                            var nameA = first.Name.Trim();
                            var nameB = second.Name.Trim();
                            var sameIdentity = first.Layer == second.Layer && (nameA.Length > 0 && nameB.Length > 0
                    ? nameA.Equals(nameB, StringComparison.OrdinalIgnoreCase)
                    : first.Reference.Trim().Length > 0
                        && first.Reference.Trim().Equals(second.Reference.Trim(), StringComparison.OrdinalIgnoreCase));

                            // The local pass may pair an unnamed carriageway with a named one, but
                            // never two roads named differently: those are two streets, such as a
                            // one-way couplet a block apart, not the halves of one avenue. A shared
                            // ref marks one road whatever its carriageways are called.
                            var referenceA = first.Reference.Trim();
                            var referenceB = second.Reference.Trim();
                            var sharedReference =
                                referenceA.Length > 0
                                    && referenceA.Equals(referenceB, StringComparison.OrdinalIgnoreCase);

                            var conflicting =
                                !sharedReference
                                    && (nameA.Length > 0 && nameB.Length > 0
                                        && !nameA.Equals(nameB, StringComparison.OrdinalIgnoreCase)
                                        || referenceA.Length > 0 && referenceB.Length > 0);

                            if (first.Layer != second.Layer || !sameIdentity && (!localMatching || conflicting))
                                continue;

                            if (firstChain.Edges[0].Bridge != secondChain.Edges[0].Bridge
                                || firstChain.Edges[0].Tunnel != secondChain.Edges[0].Tunnel)
                                continue;

                            var a0 = firstChain.Project(secondChain.End);
                            var a1 = firstChain.Project(secondChain.Start);
                            var b0 = secondChain.Project(firstChain.End);
                            var b1 = secondChain.Project(firstChain.Start);

                            if (localMatching)
                            {
                                // Both straight portions use A's longitudinal axis.
                                // Alternating perpendicular projections trims the ends
                                // twice on angled approaches and leaves parallel tails.
                                var axis = math.normalizesafe((firstChain.End - firstChain.Start).xz);
                                var start = math.dot((secondChain.Start - firstChain.Start).xz, axis);
                                var end = math.dot((secondChain.End - firstChain.Start).xz, axis);
                                var low = Math.Max(0, Math.Min(start, end));
                                var high = Math.Min(firstChain.Length, Math.Max(start, end));

                                if (high <= low || Math.Abs(end - start) < .001f)
                                    continue;

                                a0 = low / firstChain.Length;
                                a1 = high / firstChain.Length;
                                b0 = math.clamp((high - start) / (end - start), 0, 1);
                                b1 = math.clamp((low - start) / (end - start), 0, 1);
                            }
                            else
                            {
                                // Clip the partner to the common interval, not to the ends
                                // of a longer curved road outside that interval. Then bring
                                // the first interval back onto those clipped partner ends.
                                // If B's endpoint projects inside A, B is already covered
                                // to that endpoint. Projecting its perpendicular foot back
                                // again shaves off a tiny tail on non-identical tangents;
                                // moving that tail to the median then creates a sharp stub.
                                b0 = a1 < 1 ? 0 : secondChain.Project(firstChain.Point(a1));
                                b1 = a0 > 0 ? 1 : secondChain.Project(firstChain.Point(a0));

                                if (b1 < 1)
                                    a0 = firstChain.Project(secondChain.Point(b1));

                                if (b0 > 0)
                                    a1 = firstChain.Project(secondChain.Point(b0));
                            }

                            // Keep actual endpoints when projection only shaved a
                            // sub-metre tail, preserving their junction ownership.
                            if (a0 * firstChain.Length < 1)
                                a0 = 0;

                            if ((1 - a1) * firstChain.Length < 1)
                                a1 = 1;

                            if (b0 * secondChain.Length < 1)
                                b0 = 0;

                            if ((1 - b1) * secondChain.Length < 1)
                                b1 = 1;

                            // Distinct ports on a major road keep their short turning
                            // approaches. Merge the divided body behind them instead
                            // of moving the major road or rejecting the entire street.
                            const float transition = 24;

                            if (a0 == 0
                                && b1 == 1
                                && firstChain.Edges[0].StartNodeId != secondChain.Edges[^1].EndNodeId
                                && fixedPorts.Contains(firstChain.Edges[0].StartNodeId)
                        && fixedPorts.Contains(secondChain.Edges[^1].EndNodeId))
                            {
                                a0 = transition / firstChain.Length;
                                b1 = 1 - transition / secondChain.Length;
                            }

                            if (a1 == 1
                                && b0 == 0
                                && firstChain.Edges[^1].EndNodeId != secondChain.Edges[0].StartNodeId
                                && fixedPorts.Contains(firstChain.Edges[^1].EndNodeId)
                        && fixedPorts.Contains(secondChain.Edges[0].StartNodeId))
                            {
                                a1 = 1 - transition / firstChain.Length;
                                b0 = transition / secondChain.Length;
                            }

                            var overlap = (a1 - a0) * firstChain.Length;
                            var reverseOverlap = (b1 - b0) * secondChain.Length;
                            // A short merged trunk between two divided stretches is two
                            // transitions with no avenue between them.
                            var minimumOverlap = localMatching ? LocalOverlap : trunk ? MinimumTrunkStretch : 30;

                            if (overlap < minimumOverlap
                                || reverseOverlap < minimumOverlap
                                || overlap / reverseOverlap < .8f
                                || overlap / reverseOverlap > 1.25f)
                            {
                                Reject(firstChain, secondChain, "overlap", overlap);
                                continue;
                            }

                            var test = Test(a0, a1, b0, b1, overlap, reverseOverlap);

                            // A long named pair that strays apart or out of line somewhere is
                            // not kept divided over that one place: its longest close, parallel
                            // stretch merges now, and the rest is matched again on the next pass
                            // (Quebec City's Route du Président-Kennedy, 7.4 km, was refused for
                            // one point 32.3 m apart).
                            if (!test.Valid
                                && !localMatching
                                && test.Reason is "separation" or "tangent"
                                && Narrow(ref a0, ref a1, ref b0, ref b1))
                            {
                                overlap = (a1 - a0) * firstChain.Length;
                                reverseOverlap = (b1 - b0) * secondChain.Length;

                                if (overlap >= Math.Max(minimumOverlap, NarrowedMinimum)
                                    && reverseOverlap >= Math.Max(minimumOverlap, NarrowedMinimum)
                                    && overlap / reverseOverlap >= .8f
                                    && overlap / reverseOverlap <= 1.25f)
                                {
                                    var narrowed = Test(a0, a1, b0, b1, overlap, reverseOverlap);

                                    if (narrowed.Valid)
                                        test = narrowed;
                                }
                            }

                            if (!test.Valid)
                            {
                                if (test.Reason != null)
                                    Reject(firstChain, secondChain, test.Reason, test.Value);

                                continue;
                            }

                            // The longest run of samples about 8 m apart where the partner lies
                            // within NarrowedSeparation and runs parallel, as new intervals.
                            bool Narrow(ref float from, ref float to, ref float partnerFrom, ref float partnerTo)
                            {
                                var steps = Math.Max(2, (int)Math.Ceiling((to - from) * firstChain.Length / 8));
                                var runStart = -1;
                                var bestStart = 0;
                                var bestEnd = -1;

                                for (var k = 0; k <= steps; k++)
                                {
                                    var t = math.lerp(from, to, k / (float)steps);
                                    var pa = firstChain.Point(t);
                                    var u = math.clamp(secondChain.Project(pa), partnerFrom, partnerTo);
                                    var pb = secondChain.Point(u);

                                    var tangent =
                                        math.normalizesafe(
                                            firstChain.Point(Math.Min(1, t + .01f)).xz
                                                - firstChain.Point(Math.Max(0, t - .01f)).xz);

                                    var reverse =
                                        math.normalizesafe(
                                            secondChain.Point(Math.Max(0, u - .01f)).xz
                                                - secondChain.Point(Math.Min(1, u + .01f)).xz);

                                    if (math.distance(pa.xz, pb.xz) > NarrowedSeparation
                                        || math.dot(tangent, reverse) < .97f)
                                    {
                                        runStart = -1;
                                        continue;
                                    }

                                    if (runStart < 0)
                                        runStart = k;

                                    if (k - runStart > bestEnd - bestStart)
                                    {
                                        bestStart = runStart;
                                        bestEnd = k;
                                    }
                                }

                                if (bestEnd <= bestStart)
                                    return false;

                                var low = math.lerp(from, to, bestStart / (float)steps);
                                var high = math.lerp(from, to, bestEnd / (float)steps);
                                var newPartnerFrom = secondChain.Project(firstChain.Point(high));
                                var newPartnerTo = secondChain.Project(firstChain.Point(low));

                                if (newPartnerTo <= newPartnerFrom)
                                    return false;

                                from = low;
                                to = high;
                                partnerFrom = newPartnerFrom;
                                partnerTo = newPartnerTo;

                                return true;
                            }

                            // Samples the common interval: every point within MaximumSeparation,
                            // parallel, and the partner always on one side. Reason names a failure
                            // worth reporting; a side change is not reported.
                            (bool Valid, string Reason, float Value, float Separation, bool Tapered) Test(
                                float from,
                                float to,
                                float partnerFrom,
                                float partnerTo,
                                float length,
                                float partnerLength)
                            {
                                var separation = 0f;
                                var sign = 0f;
                                var tapered = false;
                                var samples = Math.Max(32, (int)Math.Ceiling(Math.Max(length, partnerLength) / 24));

                                for (var sample = 0; sample <= samples; sample++)
                                {
                                    token.ThrowIfCancellationRequested();
                                    var fraction = sample / (float)samples;
                                    var t = math.lerp(from, to, fraction);
                                    var pa = firstChain.Point(t);
                                    var u =
                                        localMatching
                                            ? math.lerp(partnerTo, partnerFrom, fraction)
                                            : math.clamp(secondChain.Project(pa), partnerFrom, partnerTo);

                                    var pb = secondChain.Point(u);
                                    var distance = math.distance(pa.xz, pb.xz);

                                    if (distance > (localMatching ? MaximumSeparation : NamedSeparation))
                                        return (false, "separation", distance, 0, false);

                                    separation += distance;
                                    var tangent =
                                        math.normalizesafe(
                                            firstChain.Point(Math.Min(1, t + .01f)).xz
                                                - firstChain.Point(Math.Max(0, t - .01f)).xz);

                                    var reverse =
                                        math.normalizesafe(
                                            secondChain.Point(Math.Max(0, u - .01f)).xz
                                                - secondChain.Point(Math.Min(1, u + .01f)).xz);

                                    // At a shared split/join the carriageways converge, so
                                    // their endpoint tangents need not be parallel. Preserve
                                    // that exact connection while testing the divided body.
                                    var sharedStart =
                                        firstChain.Edges[0].StartNodeId
                                            == secondChain.Edges[secondChain.Edges.Length - 1].EndNodeId;

                                    var sharedEnd =
                                        firstChain.Edges[firstChain.Edges.Length - 1].EndNodeId
                                            == secondChain.Edges[0].StartNodeId;

                                    var taper =
                                        sharedStart && t * firstChain.Length < 32 && (1 - u) * secondChain.Length < 32
                                            || sharedEnd && (1 - t) * firstChain.Length < 32 && u * secondChain.Length < 32;

                                    var endpoint =
                                        Math.Min(fraction, 1 - fraction) * Math.Min(length, partnerLength)
                                            < Math.Min(32, .2f * Math.Min(length, partnerLength));

                                    var alignment = math.dot(tangent, reverse);

                                    if (!taper && alignment < (localMatching ? LocalAlignment : endpoint ? .82f : .97f))
                                        return (false, "tangent", alignment, 0, false);

                                    if (!taper && alignment < .97f)
                                        tapered = true;

                                    var delta = pb.xz - pa.xz;
                                    var side = tangent.x * delta.y - tangent.y * delta.x;

                                    if (Math.Abs(side) > 1)
                                    {
                                        if (sign != 0 && Math.Sign(side) != Math.Sign(sign))
                                            return (false, null, 0, 0, false);

                                        sign = side;
                                    }
                                }

                                return (true, null, 0, separation / (samples + 1), tapered);
                            }

                            var unnamed =
                                firstChain.Edges[0].Name.Length == 0 && firstChain.Edges[0].Reference.Length == 0;

                            // Rank coverage as well as separation. A nearby tiny fragment
                            // must not displace a slightly farther full corridor match.
                            var coverage = Math.Min(overlap / firstChain.Length, reverseOverlap / secondChain.Length);

                            if ((localMatching || test.Separation >= 2)
                                && (localMatching || !unnamed || overlap >= 120 && test.Separation <= 18))
                            {
                                var score = test.Separation + 12 * (1 - coverage);
                                matches[i].Add((j, score, a0, a1, b0, b1, test.Tapered));

                                if (!localMatching)
                                    mirrored.Add((j, (i, score, b0, b1, a0, a1, test.Tapered)));
                            }
                        }
                    });
        }
        catch (AggregateException exception) when (exception.InnerExceptions.All(
            inner => inner is OperationCanceledException))
        {
            token.ThrowIfCancellationRequested();
            throw;
        }

        // A named pair is tested from each chain. The two tests can disagree: from the shorter
        // chain, the far end of a longer partner on a bend projects onto the wrong part of it,
        // and narrowing can find a different stretch. Then neither chain was the other's best
        // and neither merged: Boulevard René-Lévesque's 345 m carriageway took its 1.3 km
        // partner's far end as 15 m in. A chain whose own test found no match takes the
        // partner's, mirrored.
        foreach (var (owner, match) in mirrored)
            if (!matches[owner].Any(item => item.other == match.other))
                matches[owner].Add(match);

        // A newly tolerated end taper must not displace an existing fully
        // parallel counterpart merely because its partial interval is closer.
        for (var i = 0; i < chains.Count; i++)
            matches[i] =
                matches[i]
                    .OrderBy(item => item.tapered)
                    .ThenBy(item => item.score)
                    .ThenBy(item => item.other)
                    .ToList();

        var matchSeconds = clock.Elapsed.TotalSeconds - chainSeconds;

        bool ClearBest(int i)
        {
            if (matches[i].Count == 0)
                return false;

            var best = matches[i][0];

            // Disconnected opposing portions occupy different intervals of a
            // long street; they are not competing explanations for one median.
            return localMatching
                || matches[i]
                    .Skip(1)
                    .All(
                        item => item.tapered != best.tapered
                            || Math.Min(item.a1, best.a1) - Math.Max(item.a0, best.a0) <= .00001f
                            || item.score - best.score >= 3);
        }

        var removed = new HashSet<PreparedSegment>();
        var remap = new Dictionary<long, (long node, float3 point)>();
        var created = new List<PreparedSegment>();
        var tails = new List<PreparedSegment>();
        var used = new HashSet<int>();
        var pairs = 0;
        var createdStations = new List<(int Pair, int Index)>();
        var stations = new Dictionary<long, (int Pair, int Index)>();
        // Road sections at each node, in source order (as the former GroupBy kept them).
        var incident = new Dictionary<long, List<PreparedSegment>>(source.Count);

        void AddArm(long node, PreparedSegment segment)
        {
            if (!incident.TryGetValue(node, out var list))
                incident[node] = list = new List<PreparedSegment>();

            list.Add(segment);
        }

        foreach (var segment in source)
            if (segment.Kind == NetworkKind.Road)
            {
                AddArm(segment.StartNodeId, segment);
                AddArm(segment.EndNodeId, segment);
            }

        var junctionAnchors = new Dictionary<long, float3>();

        // Keep mainline junctions fixed. Ramp ends still follow the ordinary
        // road's canonical junction, as other attached arms do; fixing both
        // sides of a divided street at separate ramp ports would prevent its
        // median from having a single junction owner.
        foreach (var segment in source.Where(
            otherSegment => Fixed(otherSegment) && !otherSegment.ProtectedLink
                || otherSegment.Kind == NetworkKind.Road && otherSegment.Roundabout))
        {
            junctionAnchors[segment.StartNodeId] = segment.Start;
            junctionAnchors[segment.EndNodeId] = segment.End;
        }

        long nextNode = -8_000_000_000_000_000_000;
        long nextWay = -7_900_000_000_000_000_000;

        // Retired junction IDs remain live provenance through aliases. Reusing
        // one on a later pass makes that new junction resolve to the old one.
        // New IDs count up from nextNode and nextWay and would need some 8·10^18
        // of them to reach zero, so only IDs in [start, 0) can collide; the
        // positive OSM IDs, nearly all of them, need not be stored.
        var ids = new HashSet<long>();

        void AddId(long id)
        {
            if (id >= nextNode && id < 0)
                ids.Add(id);
        }

        foreach (var segment in source)
        {
            AddId(segment.StartNodeId);
            AddId(segment.EndNodeId);

            foreach (var alias in segment.SourceNodeAliases)
            {
                AddId(alias.SourceNodeId);
                AddId(alias.CanonicalNodeId);
            }
        }

        var wayIds = new HashSet<long>();

        foreach (var segment in source)
            if (segment.WayId >= nextWay && segment.WayId < 0)
                wayIds.Add(segment.WayId);

        long NewNode()
        {
            while (ids.Contains(nextNode))
                nextNode++;

            var nodeId = nextNode++;
            ids.Add(nodeId);

            return nodeId;
        }

        Chain Slice(Chain chain, float begin, float end, List<PreparedSegment> retainedTails)
        {
            var list = new List<PreparedSegment>();
            var low = begin * chain.Length;
            var high = end * chain.Length;
            var startNode = begin == 0 ? chain.Edges[0].StartNodeId : NewNode();
            var endNode = end == 1 ? chain.Edges[chain.Edges.Length - 1].EndNodeId : NewNode();

            for (var k = 0; k < chain.Edges.Length; k++)
            {
                if (Math.Abs(chain.Distances[k] - low) < .01f)
                    startNode = chain.Edges[k].StartNodeId;

                if (Math.Abs(chain.Distances[k] - high) < .01f)
                    endNode = chain.Edges[k].StartNodeId;
            }

            for (var k = 0; k < chain.Edges.Length; k++)
            {
                var segment = chain.Edges[k];
                var a = chain.Distances[k];
                var b = chain.Distances[k + 1];

                if (b <= low || a >= high)
                {
                    retainedTails.Add(segment);
                    continue;
                }

                var start = Math.Max(a, low);
                var finish = Math.Min(b, high);
                var point = math.lerp(segment.Start, segment.End, (start - a) / (b - a));
                var otherPoint = math.lerp(segment.Start, segment.End, (finish - a) / (b - a));
                var first = start == low ? startNode : segment.StartNodeId;
                var last = finish == high ? endNode : segment.EndNodeId;

                if (start > a)
                    retainedTails.Add(segment with { End = point, EndNodeId = first });

                list.Add(segment with { Start = point, End = otherPoint, StartNodeId = first, EndNodeId = last });

                if (finish < b)
                    retainedTails.Add(segment with { Start = otherPoint, StartNodeId = last });
            }

            var distances = new float[list.Count + 1];

            for (var k = 0; k < list.Count; k++)
                distances[k + 1] = distances[k] + math.distance(list[k].Start.xz, list[k].End.xz);

            return new Chain { Edges = list.ToArray(), Distances = distances };
        }

        var attempts =
            Enumerable
                .Range(0, chains.Count)
                .SelectMany(i => matches[i].Select((match, rank) => (Index: i, Rank: rank, Match: match)))
            .Where(item => localMatching || item.Rank == 0)
                .OrderBy(item => localMatching ? item.Match.score : item.Index)
                .ThenBy(item => item.Index)
                .ThenBy(item => item.Match.other);

        foreach (var attempt in attempts)
        {
            token.ThrowIfCancellationRequested();
            var i = attempt.Index;

            if (used.Contains(i) || !localMatching && !ClearBest(i))
                continue;

            var match = attempt.Match;
            var j = match.other;

            if (used.Contains(j)
                || (localMatching
                    ? !matches[j].Any(item => item.other == i)
                    : !ClearBest(j) || matches[j][0].other != i))
                continue;

            var originalA = chains[i];
            var originalB = chains[j];
            var pairTails = new List<PreparedSegment>();
            var firstChain = Slice(originalA, match.a0, match.a1, pairTails);
            var secondChain = Slice(originalB, match.b0, match.b1, pairTails);
            var layer = firstChain.Edges[0].Layer;

            if (firstChain.Edges
                .Concat(secondChain.Edges)
                .Any(
                    segment => structureNodes.Contains(segment.StartNodeId) && !RailPort(segment.StartNodeId)
                        || structureNodes.Contains(segment.EndNodeId) && !RailPort(segment.EndNodeId)))
            {
                log.Count("Avenue pairs retained at structural transitions");
                continue;
            }

            var large = false;

            if (trunk)
            {
                // Only a stretch with an at-grade intersection behaves as an
                // avenue; a divided trunk without one stays a divided highway.
                var body = new HashSet<PreparedSegment>(originalA.Edges.Concat(originalB.Edges));

                bool Crossing(long node)
                    => incident.TryGetValue(node, out var arms)
                        && arms.Any(segment => !body.Contains(segment) && !segment.ProtectedLink);

                if (!firstChain.Edges.Skip(1).Any(segment => Crossing(segment.StartNodeId))
                    && !secondChain.Edges.Skip(1).Any(segment => Crossing(segment.StartNodeId)))
                {
                    log.Count("Divided trunk stretches kept without intersections");
                    Reject(originalA, originalB, "no-intersection");
                    continue;
                }

                // Lane pockets at intersections do not decide the cross section.
                static int Predominant(Chain chain)
                    => Enumerable.Range(0, chain.Edges.Length).GroupBy(k => chain.Edges[k].Lanes)
                    .OrderByDescending(
                        group => group.Sum(
                            k => chain.Distances[k + 1]
                                - chain.Distances[k])).ThenByDescending(group => group.Key).First().Key;

                large = Predominant(firstChain) > 2 || Predominant(secondChain) > 2;
            }

            // Use one longitudinal station system. Equal fractions of unequal
            // carriageway lengths put opposite junctions at different places
            // on bends and pull the median sideways between those stations.
            float MatchOnB(float t)
                => t <= 0 ? 1 : t >= 1 ? 0 : localMatching ? 1 - t : secondChain.Project(firstChain.Point(t));

            float MatchOnA(float t)
                => t <= 0 ? 1 : t >= 1 ? 0 : localMatching ? 1 - t : firstChain.Project(secondChain.Point(t));

            var knots = new List<float> { 0, 1 };

            void Stations(Chain chain, bool reverse)
            {
                for (var k = 1; k < chain.Edges.Length; k++)
                    if (incident.TryGetValue(chain.Edges[k].StartNodeId, out var nodeArms) && nodeArms.Count > 2
                        || chain.Edges[k - 1].Lanes != chain.Edges[k].Lanes)
                    {
                        var t = chain.Distances[k] / chain.Length;
                        knots.Add(reverse ? MatchOnA(t) : t);
                    }
            }

            Stations(firstChain, false);
            Stations(secondChain, true);
            var steps = Math.Max(1, (int)Math.Ceiling(Math.Max(firstChain.Length, secondChain.Length) / 30));
            var fractions = knots.OrderBy(t => t).Aggregate(new List<float>(), (list, t) =>
{
    if (list.Count == 0 || t - list[list.Count - 1] > .00001f)
        list.Add(t);

    return list;
});

            // Opposing OSM node grids rarely line up. Coalesce near-identical
            // stations BEFORE remapping junctions instead of rejecting the avenue.
            for (var k = fractions.Count - 2; k > 0; k--)
                if ((fractions[k] - fractions[k - 1]) * Math.Min(firstChain.Length, secondChain.Length) < 12
                    || (fractions[k + 1] - fractions[k]) * Math.Min(firstChain.Length, secondChain.Length) < 12)
                    fractions.RemoveAt(k);

            // Sampling stations must not displace real side-street junctions.
            // Add regular shape samples only after the protected stations have
            // been reconciled, leaving at least 12 m to either neighbouring pin.
            for (var k = 1; k < steps; k++)
            {
                var t = k / (float)steps;

                if (fractions.All(f => Math.Abs(f - t) * Math.Min(firstChain.Length, secondChain.Length) >= 12))
                    fractions.Add(t);
            }

            fractions.Sort();
            var positions =
                fractions.Select(t => (firstChain.Point(t) + secondChain.Point(MatchOnB(t))) * .5f).ToArray();

            var nodes = positions.Select(_ => NewNode()).ToArray();
            var anchored = new Dictionary<int, (long Node, float3 Point)>();
            var local = new Dictionary<long, (long node, float3 point)>();
            var localStations = new Dictionary<long, int>();
            var conflict = false;

            void Map(Chain chain, bool reversed)
            {
                for (var k = 0; k <= chain.Edges.Length; k++)
                {
                    var original = k == chain.Edges.Length ? chain.Edges[k - 1].EndNodeId : chain.Edges[k].StartNodeId;
                    var t = chain.Distances[k] / chain.Length;

                    if (reversed)
                        t = MatchOnA(t);

                    var closest = 0;

                    for (var n = 1; n < fractions.Count; n++)
                        if (Math.Abs(fractions[n] - t) < Math.Abs(fractions[closest] - t))
                            closest = n;

                    if (junctionAnchors.TryGetValue(original, out var anchor))
                    {
                        if (anchored.TryGetValue(closest, out var priorAnchor)
                            && math.distance(priorAnchor.Point.xz, anchor.xz) > .1f)
                        {
                            conflict = true;
                            return;
                        }

                        anchored[closest] = (original, anchor);
                        positions[closest] = anchor;
                        nodes[closest] = original;
                    }

                    var key = original;
                    var value = (node: nodes[closest], point: positions[closest]);

                    if (remap.TryGetValue(key, out var old))
                    {
                        if (localMatching)
                        {
                            // A second avenue meeting the same source junction
                            // inherits its established centre, rather than
                            // demanding a second independently averaged node.
                            if (math.distance(old.point.xz, value.point.xz) > 32
                                || anchored.TryGetValue(closest, out var owner) && owner.Node != old.node)
                            {
                                conflict = true;
                                return;
                            }

                            anchored[closest] = (old.node, old.point);
                            positions[closest] = old.point;
                            nodes[closest] = old.node;
                            value = (old.node, old.point);
                        }
                        else
                        {
                            if (math.distance(old.point.xz, value.point.xz) > .1f)
                            {
                                conflict = true;
                                return;
                            }

                            nodes[closest] = old.node;
                            value.node = old.node;
                        }
                    }

                    if (local.TryGetValue(key, out var prior) && math.distance(prior.point.xz, value.point.xz) > .1f)
                    {
                        conflict = true;
                        return;
                    }

                    local[key] = value;
                    localStations[key] = closest;
                }
            }

            Map(firstChain, false);
            Map(secondChain, true);

            if (conflict)
            {
                log.Count("Avenue pairs retained for junction conflicts");
                Reject(originalA, originalB, "junction-conflict");
                Bisect(originalA);
                Bisect(originalB);
                continue;
            }

            if (trunk || firstChain.Edges.Concat(secondChain.Edges).Any(NetworkLimitedAccess.Converted))
            {
                // Turn pockets and offset junction nodes on the two mapped
                // carriageways make their midpoint wander sideways. Relax each
                // free station toward the chord of its neighbours; this removes
                // wander shorter than about 100 m and keeps the corridor's
                // bends. Ends, anchors and junctions shared with an earlier
                // merge stay where they are.
                var pinned = new HashSet<int>(anchored.Keys) { 0, positions.Length - 1 };

                foreach (var key in local.Keys)
                    if (remap.ContainsKey(key))
                        pinned.Add(localStations[key]);

                for (var pass = 0; pass < 12; pass++)
                    for (var k = 1; k < positions.Length - 1; k++)
                        if (!pinned.Contains(k))
                        {
                            var point = positions[k - 1];
                            var d = positions[k + 1] - point;
                            var t =
                                math.clamp(
                                    math.dot((positions[k] - point).xz, d.xz) / Math.Max(.001f, math.lengthsq(d.xz)),
                                    0,
                                    1);

                            positions[k] = math.lerp(positions[k], point + t * d, .5f);
                        }
            }

            // A later arm can reuse an existing canonical node at a station.
            // Every earlier source alias at that station follows the final owner.
            foreach (var key in local.Keys.ToArray())
            {
                var station = localStations[key];
                local[key] = (nodes[station], positions[station]);
            }

            // Plain track nodes between two carriageway crossings are part of
            // the same crossing. Retire those interior stations too, otherwise
            // moving only the outer ports would reverse a short track edge.
            foreach (var group in local
                .Where(entry => structureNodes.Contains(entry.Key))
                .GroupBy(entry => entry.Value.node)
                .Where(group => group.Count() > 1)
                .ToArray())
            {
                var gates = new HashSet<long>(group.Select(entry => entry.Key));
                var first = group.First();

                foreach (var gate in gates)
                    foreach (var seed in railPorts[gate])
                    {
                        var path = new List<long>();
                        var seen = new HashSet<long> { gate };
                        var edge = seed;
                        var current = gate;
                        var distance = 0f;

                        while (true)
                        {
                            distance += math.distance(edge.Start.xz, edge.End.xz);
                            var nextRail = edge.StartNodeId == current ? edge.EndNodeId : edge.StartNodeId;

                            if (distance > 40 || !seen.Add(nextRail))
                                break;

                            if (gates.Contains(nextRail))
                            {
                                foreach (var node in path)
                                {
                                    local[node] = first.Value;
                                    localStations[node] = localStations[first.Key];
                                }

                                break;
                            }

                            if (incident.ContainsKey(nextRail) || !RailPort(nextRail))
                                break;

                            path.Add(nextRail);
                            var following = railPorts[nextRail].First(segment => segment != edge);
                            current = nextRail;
                            edge = following;
                        }
                    }
            }

            var railConflict = false;

            foreach (var entry in local.Where(entry => structureNodes.Contains(entry.Key)))
            {
                var edge = railPorts[entry.Key][0];
                var otherPoint = edge.StartNodeId == entry.Key ? edge.Start : edge.End;
                var railDirection = math.normalizesafe((edge.End - edge.Start).xz);
                var delta = entry.Value.point.xz - otherPoint.xz;

                if (edge.Layer != layer
                    || math.length(delta) > 40
                    || Math.Abs(railDirection.x * delta.y - railDirection.y * delta.x) > .5f)
                {
                    railConflict = true;
                    break;
                }
            }

            foreach (var edge in local.Keys
                .Where(railPorts.ContainsKey)
                .SelectMany(nodeId => railPorts[nodeId])
                .Distinct())
            {
                var otherPoint = local.TryGetValue(edge.StartNodeId, out var start) ? start.point : edge.Start;
                var secondPoint = local.TryGetValue(edge.EndNodeId, out var end) ? end.point : edge.End;

                if (math.dot((secondPoint - otherPoint).xz, (edge.End - edge.Start).xz) < -.01f)
                {
                    railConflict = true;
                    break;
                }
            }

            if (railConflict)
            {
                log.Count("Avenue pairs retained for rail alignment conflicts");
                Reject(originalA, originalB, "rail-alignment");
                continue;
            }

            // A taper may explain non-parallel carriageway ends, but cannot
            // justify turning a previously sound side-street connection into
            // an acute junction when both ends move onto the median.
            if (match.tapered || localMatching)
            {
                var body = new HashSet<PreparedSegment>(originalA.Edges.Concat(originalB.Edges));
                var acute = false;

                foreach (var entry in local)
                {
                    if (!incident.TryGetValue(entry.Key, out var attached))
                        continue;

                    var station = localStations[entry.Key];
                    var otherPoint = positions[station];
                    var directions = new List<float2>();

                    if (station > 0)
                        directions.Add((positions[station - 1] - otherPoint).xz);

                    if (station + 1 < positions.Length)
                        directions.Add((positions[station + 1] - otherPoint).xz);

                    var external = new List<(float2 Before, float2 After)>();

                    foreach (var arm in attached.Where(segment => !body.Contains(segment)))
                    {
                        var start = arm.StartNodeId == entry.Key;
                        var old = (start ? arm.End - arm.Start : arm.Start - arm.End).xz;
                        var far = start ? arm.EndNodeId : arm.StartNodeId;
                        var secondPoint =
                            local.TryGetValue(far, out var moved)
                                ? moved.point
                                : remap.TryGetValue(far, out moved) ? moved.point : start ? arm.End : arm.Start;

                        var next = (secondPoint - otherPoint).xz;

                        if (math.lengthsq(next) < .01f)
                            continue;

                        external.Add((old, next));
                        var prior =
                            attached
                                .Where(body.Contains)
                                .Select(
                                    segment => Angle(
                                        old,
                                        (segment.StartNodeId == entry.Key
                                            ? segment.End - segment.Start
                                            : segment.Start - segment.End).xz))
                                .DefaultIfEmpty(0)
                                .Min();

                        if (prior >= 25 && directions.Min(d => Angle(next, d)) < 25)
                        {
                            acute = true;
                            break;
                        }
                    }

                    // Moving one shared anchor also changes the angle between
                    // two attached arms, even when neither is the matched body.
                    for (var x = 0; x < external.Count && !acute; x++)
                        for (var y = x + 1; y < external.Count; y++)
                            if (Angle(external[x].Before, external[y].Before) >= 25
                                && Angle(external[x].After, external[y].After) < 25)
                            {
                                acute = true;
                                break;
                            }

                    if (acute)
                        break;
                }

                if (acute)
                {
                    log.Count("Avenue taper matches retained for acute attachments");
                    Reject(originalA, originalB, "acute-attachment");
                    Bisect(originalA);
                    Bisect(originalB);
                    continue;
                }
            }

            var mappedEvidence =
                Enumerable.Range(1, positions.Length).Select(_ => new List<PreparedSegment>()).ToArray();

            foreach (var edge in firstChain.Edges.Concat(secondChain.Edges))
            {
                var first = localStations[edge.StartNodeId];
                var last = localStations[edge.EndNodeId];

                for (var k = Math.Min(first, last); k < Math.Max(first, last); k++)
                    mappedEvidence[k].Add(edge);
            }

            long Mapped(long id)
                => local.TryGetValue(id, out var mapped)
                    ? mapped.node
                    : remap.TryGetValue(id, out mapped) ? mapped.node : id;

            // Only a structure outside the pair can collapse: the pair's own sections are what the
            // merge replaces. Counting them rejected every divided bridge with a short piece whose
            // two ends land on one station (Quebec City's Route du Président-Kennedy over the
            // autoroute, a 7 m piece of way 13864014), leaving its two carriageways as one-way
            // stubs between merged avenue on both sides.
            var pairSections = new HashSet<PreparedSegment>(originalA.Edges.Concat(originalB.Edges));

            if (local.Keys.Where(incident.ContainsKey).SelectMany(id => incident[id]).Distinct()
                .Any(
                    segment => (segment.Bridge || segment.Tunnel)
                        && !pairSections.Contains(segment)
                        && Mapped(segment.StartNodeId) == Mapped(segment.EndNodeId)))
            {
                log.Count("Avenue pairs retained for collapsed structure connections");
                Reject(originalA, originalB, "collapsed-structure");
                continue;
            }

            // Reject unstable/tiny centerline pieces rather than removing either original carriageway.
            if (positions
                .Zip(positions.Skip(1), (point, otherPoint) => math.distance(point.xz, otherPoint.xz))
                .Any(d => d < .5f))
            {
                log.Count("Avenue pairs retained for tiny junction spacing");
                Reject(originalA, originalB, "tiny-junction-spacing");
                continue;
            }

            while (wayIds.Contains(nextWay))
                nextWay++;

            var way = nextWay++;
            wayIds.Add(way);
            var direction = math.normalizesafe((firstChain.End - firstChain.Start).xz);
            var median = (secondChain.Point(MatchOnB(.5f)) - firstChain.Point(.5f)).xz;
            var bOnLeft = direction.x * median.y - direction.y * median.x > 0;
            var outside = bOnLeft ? NetworkSides.Right : NetworkSides.Left;

            NetworkSides Outer(
                PreparedSegment[] sideA,
                PreparedSegment[] sideB,
                Func<PreparedSegment, NetworkSides> selector)
            {
                var aWall = sideA.Any(segment => (selector(segment) & outside) != 0);
                var bWall = sideB.Any(segment => (selector(segment) & outside) != 0);

                return (aWall ? (bOnLeft ? NetworkSides.Right : NetworkSides.Left) : 0)
                    | (bWall ? (bOnLeft ? NetworkSides.Left : NetworkSides.Right) : 0);
            }

            for (var k = 1; k < positions.Length; k++)
            {
                var middle = (fractions[k - 1] + fractions[k]) * .5f;
                var sideA = firstChain.SourcesBetween(fractions[k - 1], fractions[k]);
                var sideB = secondChain.SourcesBetween(MatchOnB(fractions[k]), MatchOnB(fractions[k - 1]));
                // Coalescing nearby junction stations may move a source port
                // across a sample boundary. Its way owns the intervals between
                // its actual mapped ports as well as its geometric overlap.
                var observations = sideA.Concat(sideB).Concat(mappedEvidence[k - 1]).Distinct().ToArray();

                int Rank(string classification)
                    => classification == "primary"
                        ? 4
                        : classification == "secondary" ? 3 : classification == "tertiary" ? 2 : 1;

                var classification =
                    trunk
                        ? large ? "primary" : "trunk"
                        : observations.OrderByDescending(
                            segment => Rank(segment.Classification)).First().Classification;

                var forward = firstChain.Lanes(middle);
                var backward = secondChain.Lanes(MatchOnB(middle));
                var requested = forward + backward;
                var lanes = trunk ? large ? 6 : 4 : NetworkSelection.MatchedLanes(requested, false, false);
                var speeds =
                    observations
                        .Where(segment => segment.SourceSpeedKph.HasValue)
                        .Select(segment => segment.SourceSpeedKph!.Value)
                        .ToArray();

                // Each output interval owns only overlapping evidence. Assigning
                // every corridor way to every piece creates false native turns.
                createdStations.Add((pairs, k - 1));
                created.Add(firstChain.SourceAt(middle) with
                {
                    WayId = way,
                    StartNodeId = nodes[k - 1],
                    EndNodeId = nodes[k],
                    Start = positions[k - 1],
                    End = positions[k],
                    SourceWayIds =
                        observations
                            .SelectMany(
                                segment => segment.SourceWayIds.Count > 0
                                    ? segment.SourceWayIds
                                    : new[] { segment.WayId })
                            .Distinct()
                            .ToArray(),
                    OriginalGeometry =
                        observations
                            .SelectMany(
                                segment => segment.OriginalGeometry.Count > 0
                                    ? segment.OriginalGeometry
                                    : new[] { segment.Start, segment.End })
                            .ToArray(),
                    Classification = classification,
                    OneWay = false,
                    Lanes = lanes,
                    ForwardLanes = lanes / 2,
                    BackwardLanes = lanes / 2,
                    BothWaysLanes = 0,
                    SourceWidthMeters = null,
                    SourceSpeedKph = speeds.Length > 0 ? speeds.Min() : null,
                    ExplicitLanes = observations.Any(segment => segment.ExplicitLanes),
                    ConsolidatedAvenue = true,
                    CorridorId = 0,
                    MaximumNativeWidth = float.PositiveInfinity,
                    ExplicitCutSides = Outer(sideA, sideB, segment => segment.ExplicitCutSides),
                    ExplicitWallSides = Outer(sideA, sideB, segment => segment.ExplicitWallSides),
                    SourceGeometry = new[] { positions[k - 1], positions[k] },
                    Control1 = null,
                    Control2 = null,
                    HorizontalParent = null,
                    ExplicitIncline = null
                });

                if (requested > 6 || forward != backward)
                    log.Diagnostic(
                        $"AVENUE CROSS SECTION sourceWays={string.Join(",", observations.Select(segment => segment.WayId).Distinct())}; requestedForward={forward}; requestedBackward={backward}; requestedTotal={requested}; supportedTotal={lanes}; directionalApproximationRequired={forward != backward}; intervalEvidenceOnly=true");
            }

            foreach (var entry in local)
                remap[entry.Key] = entry.Value;

            for (var k = 0; k < nodes.Length; k++)
                stations[nodes[k]] = (pairs, k);

            foreach (var edge in originalA.Edges.Concat(originalB.Edges))
                removed.Add(edge);

            tails.AddRange(pairTails);
            used.Add(i);
            used.Add(j);
            pairs++;
            log.Diagnostic(
                $"{(trunk ? "TRUNK AVENUE MERGE" : "AVENUE MERGE")} name={firstChain.Edges[0].Name}; reference={firstChain.Edges[0].Reference}; sourceWays={string.Join(",", firstChain.Edges.Concat(secondChain.Edges).Select(segment => segment.WayId).Distinct().OrderBy(id => id))}; separationMeters={match.score:0.##}; lengthMeters={firstChain.Length:0}; junctionNodesRemapped={local.Count}; outputSections={positions.Length - 1}; vanillaPalette={(trunk ? large ? "Large-Road" : "Highway-Twoway-4" : "Medium-or-Large-Road")}; highwaysAndRampsNotConverted={!trunk}");
        }

        var result = new List<PreparedSegment>();
        var reconnected = 0;
        var shifted = new List<(int Index, bool AtStart, float3 Delta)>();
        var folded = new List<(int Pair, int From, int To, IReadOnlyList<long> Ways)>();

        // An unmatched tail can be an unchanged original edge. Filter removed
        // carriageways before appending tails, otherwise value equality removes
        // those retained edges a second time and severs their exterior routes.
        foreach (var edge in source.Where(segment => !removed.Contains(segment)).Concat(tails))
        {
            token.ThrowIfCancellationRequested();
            var segment = edge;

            // One OSM road node denotes one connection, including a bridge or
            // tunnel portal whose layer differs from the approach. Remap every
            // incident road arm together; never leave a structural arm on the
            // replaced node. Different source node IDs remain independent.
            if (segment.Kind == NetworkKind.Road || localMatching && segment.Kind == NetworkKind.Rail)
            {
                var carriageway =
                    segment.Kind == NetworkKind.Road
                        && segment.OneWay
                        && (segment.Classification == "trunk" || !trunk && segment.ProtectedLink);

                if (remap.TryGetValue(segment.StartNodeId, out var start))
                {
                    if (carriageway)
                        shifted.Add((result.Count, true, start.point - segment.Start));

                    segment = segment with { StartNodeId = start.node, Start = start.point };
                    reconnected++;
                }

                if (remap.TryGetValue(segment.EndNodeId, out var end))
                {
                    if (carriageway)
                        shifted.Add((result.Count, false, end.point - segment.End));

                    segment = segment with { EndNodeId = end.node, End = end.point };
                    reconnected++;
                }

                if (segment.StartNodeId == segment.EndNodeId)
                { // A median crossover has become an internal connection at one avenue junction.
                    log.Count(
                        segment.Kind == NetworkKind.Rail
                            ? "Internal rail crossing spans consolidated"
                            : "Avenue median crossovers consolidated");

                    shifted.RemoveAll(item => item.Index == result.Count);
                    continue;
                }

                // A median opening or U-turn between two stations of one merged
                // trunk would retrace it; its route is carried by the avenue.
                if (trunk
                    && segment.ProtectedLink
                    && stations.TryGetValue(segment.StartNodeId, out var from)
                    && stations.TryGetValue(segment.EndNodeId, out var to)
                    && from.Pair == to.Pair)
                {
                    folded.Add(
                        (
                            from.Pair,
                            Math.Min(from.Index, to.Index),
                            Math.Max(from.Index, to.Index),
                            segment.SourceWayIds.Count > 0 ? segment.SourceWayIds : new[] { segment.WayId }));

                    continue;
                }

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

        for (var k = 0; k < created.Count; k++)
        {
            var (pair, index) = createdStations[k];
            var ways =
                folded
                    .Where(item => item.Pair == pair && item.From <= index && index < item.To)
                    .SelectMany(item => item.Ways)
                    .ToArray();

            if (ways.Length > 0)
                created[k] =
                    created[k] with { SourceWayIds = created[k].SourceWayIds.Concat(ways).Distinct().ToArray() };
        }

        result.AddRange(created);
        var mergeSeconds = clock.Elapsed.TotalSeconds - chainSeconds - matchSeconds;
        Transitions(result, shifted, NewNode, log, token);
        var transitionSeconds = clock.Elapsed.TotalSeconds - chainSeconds - matchSeconds - mergeSeconds;
        log.Count(trunk ? TrunkPairs : "Opposing road pairs consolidated into avenues", pairs);
        log.Count("Avenue attachment endpoints reconnected", reconnected);

        if (trunk)
            log.Count("Median links folded into merged trunks", folded.Count);

        foreach (var i in Enumerable
            .Range(0, chains.Count)
            .Where(i => !used.Contains(i) && matches[i].Count > 0)
            .Take(40))
            log.Diagnostic(
                $"AVENUE UNRESOLVED opposingCandidate=true; sourceWay={chains[i].Edges[0].WayId}; candidateWay={chains[matches[i][0].other].Edges[0].WayId}; separationMeters={matches[i][0].score:0.###}; lengthMeters={chains[i].Length:0.###}; reason={(ClearBest(i) ? "non-reciprocal-or-attachment-transition" : "multiple-near-equal-candidates")}; noBlindDirectionConversion=true");

        log.Diagnostic(
            $"AVENUE TOTAL eligibleSections={eligible.Length}; chains={chains.Count}; comparisons={comparisons}; pairsMerged={pairs}; sourceSectionsReplaced={removed.Count}; outputAvenueSections={created.Count}; attachmentEndpointsReconnected={reconnected}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.###}; ambiguousOrUnmatchedChainsPreserved={chains.Count - used.Count}; structuresPreserved=true");

        log.Diagnostic(
            $"AVENUE REJECTIONS {string.Join("; ", rejections.OrderBy(entry => entry.Key).Select(entry => entry.Key + "=" + entry.Value))}; sampleLimit=60");

        var aliased = SourceTopologyObligations.Transfer(source, result, NetworkKind.Road,
            remap
                .Where(entry => incident.ContainsKey(entry.Key))
                .ToDictionary(entry => entry.Key, entry => entry.Value.node));

        if (localMatching)
            aliased = SourceTopologyObligations.Transfer(source, aliased, NetworkKind.Rail,
                        remap
                            .Where(entry => structureNodes.Contains(entry.Key))
                            .ToDictionary(entry => entry.Key, entry => entry.Value.node));

        var topologySeconds = clock.Elapsed.TotalSeconds;
        var repaired = AvenueJunctionRepair.Prepare(aliased, log, token, completeStreetJunctions: localMatching);

        log.Diagnostic(
            $"AVENUE TIMES local={localMatching}; trunk={trunk}; chains={chainSeconds:0.00}; matching={matchSeconds:0.00}; merge={mergeSeconds:0.00}; transitions={transitionSeconds:0.00}; topology={topologySeconds - chainSeconds - matchSeconds - mergeSeconds - transitionSeconds:0.00}; junctionRepair={clock.Elapsed.TotalSeconds - topologySeconds:0.00}");

        return repaired;
    }

    internal static double Angle(float2 u, float2 v)
        => Math.Acos(math.clamp(math.dot(math.normalizesafe(u), math.normalizesafe(v)), -1, 1)) * 180
                            / Math.PI;
}
