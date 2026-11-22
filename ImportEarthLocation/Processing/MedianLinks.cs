using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// A turn or U-turn link mapped inside the median of a divided road lies between
// two opposing carriageways that the compiler spreads to their native widths, so
// its own native road overlaps both. Where the remaining network still leads
// from the link's start to its end by a short detour (usually through the
// cross street's median crossing), the link is removed and its source way is
// carried by that detour, so every source route stays reachable. A link that is
// the only crossover is kept.
public static class MedianLinks
{
    private const float MaxLength = 200;
    // Lateral search on each side for a carriageway, metres. Divided surface
    // roads whose medians hold turn links have centrelines 10-30 m apart.
    private const float Reach = 25;
    private const float Spacing = 5;
    private const float Cell = 32;

    // A channelized turn (slip) lane mapped as its own one-way way runs a few
    // metres beside the roads it connects, so its native road overlaps theirs
    // and meets them at sharp angles. Within this distance of a parallel road for
    // at least half its length, and with a detour through the junction no longer
    // than SlipDetour, it is folded into that detour like a median link.
    private const float SlipMaxLength = 250;
    private const float SlipBeside = 15;
    private const float SlipShare = .5f;

    public static IReadOnlyList<PreparedSegment> Prepare(
        IReadOnlyList<PreparedSegment> source,
        ImportLog log,
        CancellationToken token) =>
        Slips(Medians(source, log, token), log, token);

    private static float SlipDetour(float length) => 1.5f * length + 60;

    private static IReadOnlyList<PreparedSegment> Slips(
        IReadOnlyList<PreparedSegment> source,
        ImportLog log,
        CancellationToken token)
    {
        // Slips are links or unnamed one-way pieces; named one-way streets and
        // highway ramps are roads of their own.
        static bool Slip(PreparedSegment segment)
            => IsGround(segment) && segment.OneWay && !NativeAssetKeys.IsHighway(segment)
                && (segment.Classification.EndsWith("_link", StringComparison.OrdinalIgnoreCase)
                || string.IsNullOrEmpty(segment.Name))
                && !segment.StartTerminal
                    && !segment.EndTerminal
                    && !segment.StartLevelCrossing
                    && !segment.EndLevelCrossing;

        var incident = Incident(source);

        var grid = new Dictionary<(int, int), List<(int Section, float2 A, float2 B)>>();

        for (var i = 0; i < source.Count; i++)
            if (IsGround(source[i]))
            {
                var points = Points(source[i]);

                for (var k = 1; k < points.Count; k++)
                {
                    var vectorA = points[k - 1].xz;
                    var vectorB = points[k].xz;
                    var (x0, z0) = CellKey(math.min(vectorA, vectorB));
                    var (x1, z1) = CellKey(math.max(vectorA, vectorB));

                    for (var x = x0; x <= x1; x++)
                        for (var z = z0; z <= z1; z++)
                        {
                            if (!grid.TryGetValue((x, z), out var list))
                                grid[(x, z)] = list = new();

                            list.Add((i, vectorA, vectorB));
                        }
                }
            }

        // A parallel road beside the sample. An opposing one-way carriageway
        // means the chain is itself a carriageway of a divided road.
        int Beside(float2 point, float2 t, int layer, HashSet<int> chain)
        {
            var found = 0;
            var (cx, cz) = CellKey(point);
            var r = (int)Math.Ceiling(SlipBeside / Cell);

            for (var x = cx - r; x <= cx + r; x++)
                for (var z = cz - r; z <= cz + r; z++)
                    if (grid.TryGetValue((x, z), out var list))
                        foreach (var (section, vectorA, vectorB) in list)
                        {
                            if (chain.Contains(section) || source[section].Layer != layer)
                                continue;

                            var d = math.normalizesafe(vectorB - vectorA);
                            var dot = math.dot(d, t);

                            if (Math.Abs(dot) < .8f)
                                continue;

                            if (math.distance(point, Nearest(vectorA, vectorB, point)) > SlipBeside)
                                continue;

                            if (dot < 0 && source[section].OneWay)
                                return -1;

                            found = 1;
                        }

            return found;
        }

        var removed = new HashSet<int>();
        var detours = new HashSet<int>();
        var carried = new Dictionary<int, HashSet<long>>();
        var remap = new Dictionary<long, long>();
        var folded = 0;
        var skipped = new Dictionary<string, int>();

        bool Skip(string reason)
        {
            skipped[reason] = skipped.TryGetValue(reason, out var n) ? n + 1 : 1;
            return true;
        }

        for (var i = 0; i < source.Count; i++)
        {
            token.ThrowIfCancellationRequested();
            var first = source[i];

            if (!Slip(first) || incident[first.StartNodeId].Count < 3)
                continue;

            var chain = new List<int> { i };

            while (incident[source[chain[^1]].EndNodeId].Count == 2 && chain.Count < 64)
            {
                var arms = incident[source[chain[^1]].EndNodeId];
                var next = arms[0] == chain[^1] ? arms[1] : arms[0];

                if (chain.Contains(next)
                    || !Slip(source[next])
                    || source[next].StartNodeId != source[chain[^1]].EndNodeId)
                    break;

                chain.Add(next);
            }

            var from = first.StartNodeId;
            var to = source[chain[^1]].EndNodeId;

            if ((from == to || incident[to].Count < 3) && Skip("open end"))
                continue;

            var line = new List<float3>();

            foreach (var k in chain)
                foreach (var point in Points(source[k]))
                    if (line.Count == 0 || math.distancesq(line[^1].xz, point.xz) > 1e-6f)
                        line.Add(point);

            var length = 0f;

            for (var k = 1; k < line.Count; k++)
                length += math.distance(line[k - 1].xz, line[k].xz);

            if ((length > SlipMaxLength || length < 1) && Skip("length"))
                continue;

            if (chain.Any(k => source[k].Layer != first.Layer || detours.Contains(k) || removed.Contains(k))
                && Skip("layer or detour"))
                continue;

            var members = new HashSet<int>(chain);
            var samples = 0;
            var beside = 0;
            var opposed = false;

            for (var k = 1; k < line.Count && !opposed; k++)
            {
                var vectorA = line[k - 1].xz;
                var vectorB = line[k].xz;
                var step = math.distance(vectorA, vectorB);

                if (step < 1e-3f)
                    continue;

                var t = (vectorB - vectorA) / step;

                for (var s = Spacing / 2; s < step; s += Spacing)
                {
                    samples++;
                    var near = Beside(vectorA + t * s, t, first.Layer, members);

                    if (near < 0)
                    {
                        opposed = true;
                        break;
                    }

                    beside += near;
                }
            }

            if (opposed && Skip("carriageway"))
                continue;

            if ((samples == 0 || beside < SlipShare * samples) && Skip("apart"))
                continue;

            var path = Detour(source, incident, removed, members, from, to, SlipDetour(length));

            if (path == null && Skip("no detour"))
                continue;

            var ways =
                chain
                    .SelectMany(
                        k => source[k].SourceWayIds.Count > 0 ? source[k].SourceWayIds : new[] { source[k].WayId })
                    .ToArray();

            foreach (var k in path!)
            {
                if (!carried.TryGetValue(k, out var set))
                    carried[k] = set = new();

                set.UnionWith(ways);
            }

            removed.UnionWith(chain);
            detours.UnionWith(path);
            folded++;

            if (log.DiagnosticsEnabled)
                log.Diagnostic(
                    $"TURN SLIPS fold ways={string.Join(",", ways.Distinct())}; name='{first.Name}'; class={first.Classification}; lengthMeters={length:0}; besideShare={beside / (float)samples:0.##}; detourSections={path.Count}; at=({first.Start.x:0},{first.Start.z:0})");

            var travelled = 0f;

            foreach (var k in chain)
            {
                travelled += math.distance(source[k].Start.xz, source[k].End.xz);

                if (source[k].EndNodeId != to)
                    remap[source[k].EndNodeId] = travelled <= length / 2 ? from : to;
            }
        }

        log.Diagnostic(
            "TURN SLIPS skipped " + string.Join("; ", skipped.Select(entry => $"{entry.Key}={entry.Value}")));

        if (removed.Count == 0)
            return source;

        var result = new List<PreparedSegment>(source.Count - removed.Count);

        for (var i = 0; i < source.Count; i++)
        {
            if (removed.Contains(i))
                continue;

            var segment = source[i];

            if (carried.TryGetValue(i, out var ways))
                segment =
                    segment with
                    {
                        SourceWayIds =
                            (segment.SourceWayIds.Count > 0 ? segment.SourceWayIds : new[] { segment.WayId })
                                .Union(ways.OrderBy(wayId => wayId))
                                .ToArray()
                    };

            result.Add(segment);
        }

        log.Count("Turn slips folded into junctions", folded);
        log.Diagnostic(
            $"TURN SLIPS folded={folded}; removedSections={removed.Count}; carryingSections={carried.Count}; besideMeters={SlipBeside}; routesCarried=true");

        return SourceTopologyObligations.Transfer(source, result, NetworkKind.Road, remap);
    }

    private static IReadOnlyList<PreparedSegment> Medians(
        IReadOnlyList<PreparedSegment> source,
        ImportLog log,
        CancellationToken token)
    {
        static bool IsLink(PreparedSegment segment)
            => segment.Classification.EndsWith("_link", StringComparison.OrdinalIgnoreCase);

        // Motorway links leave the carriageway for an interchange; they are not median turns.
        bool Link(PreparedSegment segment)
            => IsGround(segment)
                && segment.OneWay
                && IsLink(segment)
                && !segment.Classification.StartsWith("motorway", StringComparison.OrdinalIgnoreCase);

        bool Carriageway(PreparedSegment segment) => IsGround(segment) && segment.OneWay && !IsLink(segment);

        var incident = Incident(source);

        var grid = new Dictionary<(int, int), List<(int Section, float2 A, float2 B)>>();

        for (var i = 0; i < source.Count; i++)
            if (Carriageway(source[i]))
            {
                var points = Points(source[i]);

                for (var k = 1; k < points.Count; k++)
                {
                    var vectorA = points[k - 1].xz;
                    var vectorB = points[k].xz;
                    var (x0, z0) = CellKey(math.min(vectorA, vectorB));
                    var (x1, z1) = CellKey(math.max(vectorA, vectorB));

                    for (var x = x0; x <= x1; x++)
                        for (var z = z0; z <= z1; z++)
                        {
                            if (!grid.TryGetValue((x, z), out var list))
                                grid[(x, z)] = list = new();

                            list.Add((i, vectorA, vectorB));
                        }
                }
            }

        // Between opposing carriageways: the nearest parallel carriageway on each
        // side runs in opposite directions, one of them with the link.
        bool Median(float2 point, float2 t, int layer)
        {
            float2? left = null, right = null;
            var leftGap = Reach;
            var rightGap = Reach;
            var (cx, cz) = CellKey(point);
            var r = (int)Math.Ceiling(Reach / Cell);

            for (var x = cx - r; x <= cx + r; x++)
                for (var z = cz - r; z <= cz + r; z++)
                    if (grid.TryGetValue((x, z), out var list))
                        foreach (var (section, vectorA, vectorB) in list)
                        {
                            if (source[section].Layer != layer)
                                continue;

                            var d = math.normalizesafe(vectorB - vectorA);

                            if (Math.Abs(math.dot(d, t)) < .9f)
                                continue;

                            var otherPoint =
                                math.lerp(
                                    vectorA,
                                    vectorB,
                                    math.saturate(
                                        math.dot(point - vectorA, vectorB - vectorA)
                                            / Math.Max(1e-6f, math.lengthsq(vectorB - vectorA))));

                            var gap = math.distance(point, otherPoint);
                            var side = t.x * (otherPoint - point).y - t.y * (otherPoint - point).x;

                            if (side > 0 && gap < leftGap)
                            {
                                leftGap = gap;
                                left = d;
                            }
                            else if (side < 0 && gap < rightGap)
                            {
                                rightGap = gap;
                                right = d;
                            }
                        }

            return left is float2 l && right is float2 m && math.dot(l, m) < -.9f;
        }

        bool Through(long node, PreparedSegment into)
        {
            var arms = incident[node];

            if (arms.Count != 2)
                return false;

            var next = source[arms[0]] == into ? source[arms[1]] : source[arms[0]];

            return Link(next) && next.StartNodeId == node && next.Layer == into.Layer;
        }

        var removed = new HashSet<int>();
        var detours = new HashSet<int>();
        var carried = new Dictionary<int, HashSet<long>>();
        var remap = new Dictionary<long, long>();
        var split = new Dictionary<int, (long Node, float3 Point, long[] Ways)>();
        var openings = new List<PreparedSegment>();
        var folded = 0;
        var ids = new HashSet<long>();
        var nextId = long.MinValue + 46000000;

        foreach (var segment in source)
        {
            ids.Add(segment.StartNodeId);
            ids.Add(segment.EndNodeId);
        }

        foreach (var segment in source)
            foreach (var alias in segment.SourceNodeAliases)
            {
                ids.Add(alias.SourceNodeId);
                ids.Add(alias.CanonicalNodeId);
            }

        long Id()
        {
            while (ids.Contains(nextId))
                nextId++;

            var id = nextId++;
            ids.Add(id);

            return id;
        }

        var skipped = new Dictionary<string, int>();

        bool Skip(string reason)
        {
            skipped[reason] = skipped.TryGetValue(reason, out var n) ? n + 1 : 1;
            return true;
        }

        for (var i = 0; i < source.Count; i++)
        {
            token.ThrowIfCancellationRequested();
            var first = source[i];

            if (!Link(first)
                || incident[first.StartNodeId].Count == 2
                    && incident[first.StartNodeId].Any(
                        j => j != i && Link(source[j]) && source[j].EndNodeId == first.StartNodeId))
                continue;

            var chain = new List<int> { i };

            while (Through(source[chain[^1]].EndNodeId, source[chain[^1]]) && chain.Count < 64)
            {
                var arms = incident[source[chain[^1]].EndNodeId];
                var next = arms[0] == chain[^1] ? arms[1] : arms[0];

                if (chain.Contains(next))
                    break;

                chain.Add(next);
            }

            var from = first.StartNodeId;
            var to = source[chain[^1]].EndNodeId;
            var line = new List<float3>();

            foreach (var k in chain)
                foreach (var point in Points(source[k]))
                    if (line.Count == 0 || math.distancesq(line[^1].xz, point.xz) > 1e-6f)
                        line.Add(point);

            var length = 0f;

            for (var k = 1; k < line.Count; k++)
                length += math.distance(line[k - 1].xz, line[k].xz);

            if (from == to || length > MaxLength || length < 1)
                continue;

            if (chain.Any(k => source[k].Layer != first.Layer || detours.Contains(k)) && Skip("layer or detour"))
                continue;

            var samples = 0;
            var inside = 0;

            for (var k = 1; k < line.Count; k++)
            {
                var vectorA = line[k - 1].xz;
                var vectorB = line[k].xz;
                var step = math.distance(vectorA, vectorB);

                if (step < 1e-3f)
                    continue;

                var t = (vectorB - vectorA) / step;

                for (var s = Spacing / 2; s < step; s += Spacing)
                {
                    samples++;

                    if (Median(vectorA + t * s, t, first.Layer))
                        inside++;
                }
            }

            if ((samples == 0 || inside < .75f * samples) && Skip("not median"))
                continue;

            var ways =
                chain
                    .SelectMany(
                        k => source[k].SourceWayIds.Count > 0 ? source[k].SourceWayIds : new[] { source[k].WayId })
                    .ToArray();

            void Carry(int k)
            {
                if (!carried.TryGetValue(k, out var set))
                    carried[k] = set = new();

                set.UnionWith(ways);
            }

            var path = Detour(source, incident, removed, new HashSet<int>(chain), from, to, 2 * length + 200);

            if (path == null)
            {
                // The only crossover: leave the carriageway where it passes the
                // link's far end and cross the median there, as a median opening.
                var end = source[chain[^1]].End;
                var opening =
                    Opening(
                        source,
                        incident,
                        removed,
                        from,
                        end,
                        math.normalizesafe(end.xz - source[i].Start.xz),
                        length + Reach);

                if (opening == null && Skip("only crossover, no opening"))
                    continue;

                var (walked, point, gap) = opening!.Value;

                if (gap > Reach && Skip("opening too wide"))
                    continue;

                // Within 2 m of an existing node the opening uses it; otherwise
                // the carriageway is split once, its first piece carrying the link.
                var carriageway = source[walked[^1]];
                long node;

                if (math.distance(point.xz, carriageway.Start.xz) < 2)
                {
                    node = carriageway.StartNodeId;
                    walked.RemoveAt(walked.Count - 1);
                }
                else if (math.distance(point.xz, carriageway.End.xz) < 2)
                    node = carriageway.EndNodeId;
                else if (split.ContainsKey(walked[^1]) && Skip("opening shared"))
                    continue;
                else
                {
                    node = Id();
                    split[walked[^1]] = (node, point, ways);
                    walked.RemoveAt(walked.Count - 1);
                }

                if ((node == from || node == to) && Skip("already transverse"))
                    continue;

                removed.UnionWith(chain);

                foreach (var k in walked)
                    Carry(k);

                openings.Add(
                    Refit(
                        source[chain[^1]] with
                        {
                            StartNodeId = node,
                            EndNodeId = to,
                            Start = point,
                            End = end,
                            SourceGeometry = new[] { point, end },
                            SourceWayIds = ways
                        }));
            }
            else
            {
                removed.UnionWith(chain);
                detours.UnionWith(path);

                foreach (var k in path)
                    Carry(k);

                folded++;
            }

            var travelled = 0f;

            foreach (var k in chain)
            {
                travelled += math.distance(source[k].Start.xz, source[k].End.xz);

                if (source[k].EndNodeId != to)
                    remap[source[k].EndNodeId] = travelled <= length / 2 ? from : to;
            }
        }

        log.Diagnostic(
            "MEDIAN LINKS skipped " + string.Join("; ", skipped.Select(entry => $"{entry.Key}={entry.Value}")));

        if (removed.Count == 0)
            return source;

        static PreparedSegment Carrying(PreparedSegment segment, IEnumerable<long> ways) =>
            segment with
            {
                SourceWayIds =
                    (segment.SourceWayIds.Count > 0 ? segment.SourceWayIds : new[] { segment.WayId })
                        .Union(ways.OrderBy(wayId => wayId))
                        .ToArray()
            };

        var result = new List<PreparedSegment>(source.Count - removed.Count + split.Count + openings.Count);

        for (var i = 0; i < source.Count; i++)
        {
            if (removed.Contains(i))
                continue;

            var segment = source[i];

            if (carried.TryGetValue(i, out var ways))
                segment = Carrying(segment, ways);

            if (!split.TryGetValue(i, out var cut))
            {
                result.Add(segment);
                continue;
            }

            var (before, after) = Cut(segment, cut.Point);
            result.Add(
                Refit(
                    Carrying(
                        segment with { EndNodeId = cut.Node, End = cut.Point, SourceGeometry = before },
                        cut.Ways)));

            result.Add(Refit(segment with { StartNodeId = cut.Node, Start = cut.Point, SourceGeometry = after }));
        }

        result.AddRange(openings);
        log.Count("Median turn links folded into crossings", folded);
        log.Count("Median turn links shortened to openings", openings.Count);
        log.Diagnostic(
            $"MEDIAN LINKS folded={folded}; openings={openings.Count}; removedSections={removed.Count}; carryingSections={carried.Count}; routesCarried=true");

        return SourceTopologyObligations.Transfer(source, result, NetworkKind.Road, remap);
    }

    // The road sections at each node, in section order; a section ending where it starts is listed once.
    private static Dictionary<long, List<int>> Incident(IReadOnlyList<PreparedSegment> source)
    {
        var incident = new Dictionary<long, List<int>>();

        void Touch(long id, int index)
        {
            if (!incident.TryGetValue(id, out var arms))
                incident[id] = arms = new();

            arms.Add(index);
        }

        for (var i = 0; i < source.Count; i++)
        {
            if (source[i].Kind != NetworkKind.Road)
                continue;

            Touch(source[i].StartNodeId, i);

            if (source[i].EndNodeId != source[i].StartNodeId)
                Touch(source[i].EndNodeId, i);
        }

        return incident;
    }

    private static IReadOnlyList<float3> Points(PreparedSegment segment)
        => segment.SourceGeometry.Count >= 2 ? segment.SourceGeometry : new[] { segment.Start, segment.End };

    // New ends or polyline: the old curve fit no longer applies.
    private static PreparedSegment Refit(PreparedSegment segment) => segment with
    {
        Control1 = null,
        Control2 = null,
        HorizontalParent = null,
        ParentStartT = 0,
        ParentEndT = 1,
        StartCurveOffset = float3.zero,
        EndCurveOffset = float3.zero
    };

    // The section's polyline before and after the point on it nearest to cut.
    private static (float3[] Before, float3[] After) Cut(PreparedSegment otherSegment, float3 cut)
    {
        var points = Points(otherSegment);
        var segment = 1;
        var best = float.PositiveInfinity;

        for (var k = 1; k < points.Count; k++)
        {
            var gap = math.distancesq(cut.xz, Nearest(points[k - 1].xz, points[k].xz, cut.xz));

            if (gap < best)
            {
                best = gap;
                segment = k;
            }
        }

        return (points.Take(segment).Append(cut).ToArray(), new[] { cut }.Concat(points.Skip(segment)).ToArray());
    }

    private static float2 Nearest(float2 vectorA, float2 vectorB, float2 point)
        => math.lerp(
            vectorA,
            vectorB,
            math.saturate(
                math.dot(point - vectorA, vectorB - vectorA) / Math.Max(1e-6f, math.lengthsq(vectorB - vectorA))));

    // Follow the carriageway leaving from in the link's direction, up to reach
    // metres, and return the sections walked (the last one holding the point)
    // with the point nearest to target and its distance from target.
    private static (List<int> Walked, float3 Point, float Gap)? Opening(
        IReadOnlyList<PreparedSegment> source,
        Dictionary<long, List<int>> incident,
        HashSet<int> removed,
        long from, float3 target, float2 direction, float reach)
    {
        var walked = new List<int>();
        var node = from;
        var heading = direction;
        var travelled = 0f;
        (int Count, float3 Point, float Gap) best = (0, default, float.PositiveInfinity);

        while (travelled < reach)
        {
            var next = -1;
            var alignment = .5f;

            foreach (var i in incident[node])
            {
                var segment = source[i];

                if (removed.Contains(i)
                    || walked.Contains(i)
                    || segment.StartNodeId != node
                    || !segment.OneWay
                    || segment.Bridge
                    || segment.Tunnel
                    || segment.Roundabout
                    || segment.Classification.EndsWith("_link", StringComparison.OrdinalIgnoreCase))
                    continue;

                var dot = math.dot(heading, math.normalizesafe(segment.End.xz - segment.Start.xz));

                if (dot > alignment)
                {
                    alignment = dot;
                    next = i;
                }
            }

            if (next < 0)
                break;

            walked.Add(next);
            var points = Points(source[next]);

            for (var k = 1; k < points.Count; k++)
            {
                var point = Nearest(points[k - 1].xz, points[k].xz, target.xz);
                var gap = math.distance(point, target.xz);
                var t =
                    math.distance(points[k - 1].xz, point)
                        / Math.Max(1e-6f, math.distance(points[k - 1].xz, points[k].xz));

                if (gap < best.Gap)
                    best = (walked.Count, math.lerp(points[k - 1], points[k], t), gap);
            }

            travelled += math.distance(source[next].Start.xz, source[next].End.xz);
            heading = math.normalizesafe(source[next].End.xz - points[^2].xz);
            node = source[next].EndNodeId;
        }

        return best.Count == 0 ? null : (walked.Take(best.Count).ToList(), best.Point, best.Gap);
    }

    // Shortest directed road path that avoids the link and anything already
    // removed, within budget metres; null when the link is the only way.
    private static List<int>? Detour(
        IReadOnlyList<PreparedSegment> source,
        Dictionary<long, List<int>> incident,
        HashSet<int> removed,
        HashSet<int> link,
        long from, long to, float budget)
    {
        var best = new Dictionary<long, float> { [from] = 0 };
        var via = new Dictionary<long, (long Node, int Section)>();
        var queue = new SortedSet<(float Cost, long Node)> { (0, from) };

        while (queue.Count > 0)
        {
            var (cost, node) = queue.Min;
            queue.Remove(queue.Min);

            if (node == to)
            {
                var path = new List<int>();

                for (var at = to; at != from; at = via[at].Node)
                    path.Add(via[at].Section);

                path.Reverse();

                return path;
            }

            foreach (var i in incident[node])
            {
                var segment = source[i];

                if (segment.Kind != NetworkKind.Road || removed.Contains(i) || link.Contains(i))
                    continue;

                long next;

                if (segment.StartNodeId == node)
                    next = segment.EndNodeId;
                else if (!segment.OneWay)
                    next = segment.StartNodeId;
                else
                    continue;

                var total = cost + math.distance(segment.Start.xz, segment.End.xz);

                if (total > budget || best.TryGetValue(next, out var known) && known <= total)
                    continue;

                if (best.ContainsKey(next))
                    queue.Remove((best[next], next));

                best[next] = total;
                via[next] = (node, i);
                queue.Add((total, next));
            }
        }

        return null;
    }

    private static (int, int) CellKey(float2 point)
        => ((int)Math.Floor(point.x / Cell), (int)Math.Floor(point.y / Cell));

    private static bool IsGround(PreparedSegment segment)
        => segment.Kind == NetworkKind.Road && !segment.Bridge && !segment.Tunnel && !segment.Roundabout;
}
