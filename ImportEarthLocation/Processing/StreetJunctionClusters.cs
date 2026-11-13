using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Two ground street junctions joined by a connector shorter than the roads are
// wide cannot both be built: their native intersections overlap. Such pairs are
// usually mapping offsets (two T-junctions a few metres apart). Merge them into
// one junction at the more important node; every route through the connector
// still meets at that node, and the exact remap is transferred as source aliases.
public static class StreetJunctionClusters
{
    // A one-way pair this short and this close between a two-way end and the
    // next junction is too short to consolidate as an avenue, yet each piece is
    // a full native road. Street one-way roads are about 17 m wide, so a pair
    // whose centrelines stay within 20 m leaves at most a sliver of median.
    private const float SplitReach = 60;
    private const float SplitSeparation = 20;

    public static IReadOnlyList<PreparedSegment> Prepare(
        IReadOnlyList<PreparedSegment> source,
        ImportLog log,
        CancellationToken token)
    {
        source = CloseSplits(source, log, token);
        var (incident, railNodes) = Graph(source);

        bool Junction(long id) => incident[id].Count >= 3 && !railNodes.Contains(id) && incident[id].All(Eligible);

        long Other(PreparedSegment segment, long id)
            => segment.StartNodeId == id ? segment.EndNodeId : segment.StartNodeId;

        var remap = new Dictionary<long, (long Target, float3 Position)>();
        var removed = new HashSet<PreparedSegment>();
        var touched = new HashSet<long>();
        var merged = 0;
        // The connector may pass plain nodes where lanes or names change.
        var candidates = new List<(PreparedSegment[] Chain, long From, long To, float Length)>();

        foreach (var from in incident.Keys.Where(Junction))
            foreach (var first in incident[from])
            {
                var chain = new List<PreparedSegment>();
                var node = from;
                var edge = first;
                var length = 0f;

                while (Eligible(edge) && edge.StartNodeId != edge.EndNodeId && !chain.Contains(edge))
                {
                    chain.Add(edge);
                    length += Length(edge);
                    node = Other(edge, node);

                    if (length > 16 || node == from)
                        break;

                    if (incident[node].Count == 2 && !railNodes.Contains(node))
                    {
                        edge = incident[node][0] == edge ? incident[node][1] : incident[node][0];
                        continue;
                    }

                    if (from < node && Junction(node) && length <= Limit(from, node, incident))
                        candidates.Add((chain.ToArray(), from, node, length));

                    break;
                }
            }

        var skipped = new Dictionary<string, int>();

        bool Skip(string reason)
        {
            skipped[reason] = skipped.TryGetValue(reason, out var n) ? n + 1 : 1;
            return true;
        }

        foreach (var (chain, from, to, _) in candidates
            .OrderBy(item => item.Length)
            .ThenBy(item => item.From)
            .ThenBy(item => item.To))
        {
            token.ThrowIfCancellationRequested();
            var interior =
                chain
                    .SelectMany(segment => new[] { segment.StartNodeId, segment.EndNodeId })
                    .Where(id => id != from && id != to)
                    .Distinct()
                    .ToArray();

            if ((touched.Contains(from) || touched.Contains(to) || interior.Any(touched.Contains)) && Skip("chained"))
                continue;

            var firstSegments = incident[from].Except(chain).ToArray();
            var secondSegments = incident[to].Except(chain).ToArray();

            if (firstSegments.Concat(secondSegments).Concat(chain).Any(segment => segment.Layer != chain[0].Layer)
                && Skip("layer"))
                continue;

            // A street crossing a divided road meets each carriageway separately;
            // one node would make the opposing carriageways touch.
            if (firstSegments.Count(segment => segment.OneWay) >= 2
                && secondSegments.Count(segment => segment.OneWay) >= 2
                && Skip("divided"))
                continue;

            // A triangle or a second connector would become a duplicate or a loop.
            var neighbours = new HashSet<long>(firstSegments.Select(segment => Other(segment, from)));

            if (secondSegments.Any(
                segment => neighbours.Contains(Other(segment, to))
                    || Other(segment, to) == from
                    || segment.StartNodeId == segment.EndNodeId)
                && Skip("triangle"))
                continue;

            var (kept, moved, movedArms) =
                Weight(firstSegments) >= Weight(secondSegments)
                    ? (from, to, secondSegments)
                    : (to, from, firstSegments);

            var target =
                chain.First(segment => segment.StartNodeId == kept || segment.EndNodeId == kept) is var end
                    && end.StartNodeId == kept
                    ? end.Start
                    : end.End;

            if (movedArms.Any(
                segment => math.distance(target.xz, (segment.StartNodeId == moved ? segment.End : segment.Start).xz)
                    < 3)
                && Skip("short arm"))
                continue;

            remap[moved] = (kept, target);

            foreach (var id in interior)
                remap[id] = (kept, target);

            removed.UnionWith(chain);
            touched.Add(kept);
            touched.Add(moved);
            touched.UnionWith(interior);
            merged++;
        }

        log.Diagnostic(
            "STREET JUNCTION CLUSTERS skipped "
                + string.Join("; ", skipped.Select(entry => $"{entry.Key}={entry.Value}")));

        if (remap.Count == 0)
            return source;

        var result = new List<PreparedSegment>(source.Count - removed.Count);

        foreach (var segment in source)
        {
            if (removed.Contains(segment))
                continue;

            if (segment.Kind != NetworkKind.Road
                || !remap.ContainsKey(segment.StartNodeId) && !remap.ContainsKey(segment.EndNodeId))
            {
                result.Add(segment);
                continue;
            }

            var first =
                remap.TryGetValue(segment.StartNodeId, out var firstItem)
                    ? firstItem
                    : (Target: segment.StartNodeId, Position: segment.Start);

            var last =
                remap.TryGetValue(segment.EndNodeId, out var secondItem)
                    ? secondItem
                    : (Target: segment.EndNodeId, Position: segment.End);

            var geometry =
                segment.SourceGeometry.Count >= 2
                    ? segment.SourceGeometry.ToArray()
                    : new[] { segment.Start, segment.End };

            geometry[0] = first.Position;
            geometry[^1] = last.Position;
            result.Add(Refit(segment, first.Target, last.Target, geometry));
        }

        log.Count("Street junction clusters merged", merged);
        log.Diagnostic(
            $"STREET JUNCTION CLUSTERS merged={merged}; aliasedNodes={remap.Count}; groundStreetsOnly=true; dividedCrossingsKept=true; exactAliases=true");

        return SourceTopologyObligations.Transfer(
            source,
            result,
            NetworkKind.Road,
            remap.ToDictionary(entry => entry.Key, entry => entry.Value.Target));
    }

    // A two-way street ending at a node where it splits into one-way pieces that
    // rejoin at a single junction within SplitReach becomes one two-way section
    // along their midline, copying the street it continues. Interior nodes alias
    // their nearer end.
    private static IReadOnlyList<PreparedSegment> CloseSplits(
        IReadOnlyList<PreparedSegment> source,
        ImportLog log,
        CancellationToken token)
    {
        var (incident, railNodes) = Graph(source);

        (List<PreparedSegment> Chain, long End)? Trace(long from, PreparedSegment edge, bool outgoing)
        {
            var chain = new List<PreparedSegment>();
            var node = from;
            var length = 0f;

            while (true)
            {
                if (!Eligible(edge)
                    || !edge.OneWay
                    || (outgoing ? edge.StartNodeId : edge.EndNodeId) != node
                    || chain.Contains(edge))
                    return null;

                chain.Add(edge);
                length += Length(edge);

                if (length > SplitReach)
                    return null;

                node = outgoing ? edge.EndNodeId : edge.StartNodeId;

                if (incident[node].Count != 2 || railNodes.Contains(node))
                    return (chain, node);

                edge = incident[node][0] == edge ? incident[node][1] : incident[node][0];
            }
        }

        // Ring pieces by their start node, in travel direction.
        var ringNext =
            source
                .Where(segment => segment.Kind == NetworkKind.Road && segment.Roundabout)
                .GroupBy(segment => segment.StartNodeId)
                .Where(group => group.Count() == 1)
                .ToDictionary(group => group.Key, group => group.First());

        List<PreparedSegment>? Arc(long from, long to)
        {
            var arc = new List<PreparedSegment>();
            var length = 0f;

            for (var node = from; node != to;)
            {
                if (!ringNext.TryGetValue(node, out var piece) || arc.Contains(piece))
                    return null;

                arc.Add(piece);
                length += Length(piece);

                if (length > SplitReach)
                    return null;

                node = piece.EndNodeId;
            }

            return arc;
        }

        var removed = new HashSet<PreparedSegment>();
        var created = new List<PreparedSegment>();
        var remap = new Dictionary<long, long>();
        var used = new HashSet<long>();
        var ringWays = new Dictionary<PreparedSegment, List<long>>();
        var ringApproaches = 0;
        var skipped = new Dictionary<string, int>();

        bool Skip(string reason)
        {
            skipped[reason] = skipped.TryGetValue(reason, out var n) ? n + 1 : 1;
            return true;
        }

        foreach (var x in incident.Keys.OrderBy(id => id))
        {
            token.ThrowIfCancellationRequested();
            var arms = incident[x];

            if (arms.Count != 3 || railNodes.Contains(x) || used.Contains(x))
                continue;

            var street = arms.Where(segment => !segment.OneWay).ToArray();
            var leaving = arms.Where(segment => segment.OneWay && segment.StartNodeId == x).ToArray();
            var entering = arms.Where(segment => segment.OneWay && segment.EndNodeId == x).ToArray();

            if (street.Length != 1
                || leaving.Length != 1
                || entering.Length != 1
                || !Eligible(street[0])
                || street[0].StartNodeId == street[0].EndNodeId)
                continue;

            if (Trace(x, leaving[0], true) is not { } forward || Trace(x, entering[0], false) is not { } backward)
            {
                Skip("trace");
                continue;
            }

            var y = forward.End;
            var pieces = forward.Chain.Concat(backward.Chain).ToArray();
            // A divided street meeting a roundabout leaves the ring just upstream
            // of where it joins it. The pair becomes one two-way approach to the
            // ring node nearest the middle of the arc between them.
            var arc = y != backward.End ? Arc(backward.End, y) : null;

            if ((y == x || y != backward.End && arc == null) && Skip("apart"))
                continue;

            var ends = arc == null ? new[] { y } : arc.Select(segment => segment.StartNodeId).Append(y).ToArray();

            if ((incident[y].Count < 3 || ends.Any(id => railNodes.Contains(id) || used.Contains(id))
                || pieces
                    .Concat(arc ?? new List<PreparedSegment>())
                    .Any(segment => removed.Contains(segment) || segment.Layer != street[0].Layer))
                    && Skip("end"))
                continue;

            // Opposite stations at the same fraction of each carriageway give the
            // midline; a pair farther apart than SplitSeparation is a real median.
            // At a roundabout the flares may spread to the width of the arc, up
            // to twice that.
            var there = Polyline(forward.Chain);
            var back = Polyline(Enumerable.Reverse(backward.Chain));
            back.Reverse();
            var steps = Math.Max(1, (int)Math.Ceiling(Math.Max(PathLength(there), PathLength(back)) / 10));
            var midline =
                Enumerable
                    .Range(0, steps + 1)
                    .Select(k => (A: At(there, k / (float)steps), B: At(back, k / (float)steps)))
                    .ToArray();

            var separation =
                arc == null
                    ? SplitSeparation
                    : Math.Clamp(math.distance(there[^1].xz, back[^1].xz) + 2, SplitSeparation, 2 * SplitSeparation);

            if (midline.Any(item => math.distance(item.A.xz, item.B.xz) > separation) && Skip("median"))
                continue;

            var template = street[0];
            var start = template.StartNodeId == x ? template.Start : template.End;
            var end = there[^1];

            if (arc != null)
            {
                var middle = (there[^1] + back[^1]) / 2;
                var k =
                    Enumerable
                        .Range(0, arc.Count + 1)
                        .OrderBy(i => math.distancesq((i < arc.Count ? arc[i].Start : arc[^1].End).xz, middle.xz))
                        .First();

                y = ends[k];
                end = k < arc.Count ? arc[k].Start : arc[^1].End;

                // Entering traffic now joins at y and circulates to the old entry;
                // exiting traffic circulates from the old exit to y.
                for (var i = 0; i < arc.Count; i++)
                {
                    if (!ringWays.TryGetValue(arc[i], out var ways))
                        ringWays[arc[i]] = ways = new();

                    ways.AddRange(
                        (i < k
                            ? backward.Chain
                            : forward.Chain).SelectMany(
                                segment => segment.SourceWayIds.Count > 0
                                    ? segment.SourceWayIds
                                    : new[] { segment.WayId }));
                }

                used.UnionWith(ends);
                ringApproaches++;
            }

            var geometry = midline.Select(item => (item.A + item.B) / 2).ToArray();
            geometry[0] = start;
            geometry[^1] = end;
            created.Add(Refit(template, x, y, geometry) with
            {
                ExplicitIncline = null,
                SourceWayIds =
                    pieces
                        .SelectMany(
                            segment => segment.SourceWayIds.Count > 0 ? segment.SourceWayIds : new[] { segment.WayId })
                        .Distinct()
                        .ToArray(),
                OriginalGeometry =
                    pieces
                        .SelectMany(
                            segment => segment.OriginalGeometry.Count > 0
                                ? segment.OriginalGeometry
                                : new[] { segment.Start, segment.End })
                        .ToArray()
            });

            foreach (var segment in pieces)
            {
                removed.Add(segment);

                foreach (var (id, point) in new[] {
                    (segment.StartNodeId, segment.Start),
                    (segment.EndNodeId, segment.End)
                    })
                    if (id != x && !ends.Contains(id))
                        remap[id] = math.distancesq(point.xz, start.xz) <= math.distancesq(point.xz, end.xz) ? x : y;
            }

            used.Add(x);
            used.Add(y);
        }

        log.Diagnostic(
            "STREET SPLITS skipped " + string.Join("; ", skipped.Select(entry => $"{entry.Key}={entry.Value}")));

        if (created.Count == 0)
            return source;

        log.Count("Short one-way street splits closed", created.Count);
        log.Count("Divided roundabout approaches joined", ringApproaches);
        log.Diagnostic(
            $"STREET SPLITS closed={created.Count}; roundaboutApproaches={ringApproaches}; reachMeters={SplitReach}; separationMeters={SplitSeparation}; interiorAliases={remap.Count}");

        var result =
            source
                .Where(segment => !removed.Contains(segment))
                .Select(segment => ringWays.TryGetValue(segment, out var ways) ?
            segment with
            {
                SourceWayIds =
                    (segment.SourceWayIds.Count > 0 ? segment.SourceWayIds : new[] { segment.WayId })
                        .Concat(ways)
                        .Distinct()
                        .ToArray()
            }
                : segment)
                .Concat(created)
                .ToArray();

        return SourceTopologyObligations.Transfer(source, result, NetworkKind.Road, remap);
    }

    // Road arms per node. Rail shares node IDs with roads only at level
    // crossings, which keep their shared node.
    private static (Dictionary<long, List<PreparedSegment>> Incident, HashSet<long> RailNodes) Graph(
        IReadOnlyList<PreparedSegment> source)
    {
        var incident = new Dictionary<long, List<PreparedSegment>>();

        foreach (var segment in source.Where(otherSegment => otherSegment.Kind == NetworkKind.Road))
            foreach (var id in new[] { segment.StartNodeId, segment.EndNodeId }.Distinct())
            {
                if (!incident.TryGetValue(id, out var arms))
                    incident[id] = arms = new();

                arms.Add(segment);
            }

        return (
            incident,
            new HashSet<long>(
                source
                    .Where(segment => segment.Kind == NetworkKind.Rail)
                    .SelectMany(segment => new[] { segment.StartNodeId, segment.EndNodeId })));
    }

    // A section with new end nodes and source polyline; the old curve fit no longer applies.
    private static PreparedSegment Refit(PreparedSegment segment, long startNode, long endNode, float3[] geometry)
        => segment with
        {
            StartNodeId = startNode,
            EndNodeId = endNode,
            Start = geometry[0],
            End = geometry[^1],
            SourceGeometry = geometry,
            Control1 = null,
            Control2 = null,
            HorizontalParent = null,
            ParentStartT = 0,
            ParentEndT = 1,
            StartCurveOffset = float3.zero,
            EndCurveOffset = float3.zero
        };

    private static bool Eligible(PreparedSegment segment)
        => segment.Kind == NetworkKind.Road
            && !NativeAssetKeys.IsHighway(segment)
            && !segment.Bridge
            && !segment.Tunnel
            && !segment.Roundabout
            && !segment.StartTerminal
            && !segment.EndTerminal
            && !segment.StartMiniRoundabout
            && !segment.EndMiniRoundabout
            && !segment.StartLevelCrossing
            && !segment.EndLevelCrossing
            && segment.StartRoundaboutRadius == 0 && segment.EndRoundaboutRadius == 0;

    private static float Length(PreparedSegment segment)
        => PathLength(
            segment.SourceGeometry.Count >= 2 ? segment.SourceGeometry : new[] { segment.Start, segment.End });

    private static float PathLength(IReadOnlyList<float3> points)
    {
        var length = 0f;

        for (var i = 1; i < points.Count; i++)
            length += math.distance(points[i - 1].xz, points[i].xz);

        return length;
    }

    // Vertices of consecutive sections, in chain order and travel direction.
    private static List<float3> Polyline(IEnumerable<PreparedSegment> chain)
    {
        var points = new List<float3>();

        foreach (var segment in chain)
            foreach (var point in segment.SourceGeometry.Count >= 2
                ? segment.SourceGeometry
                : new[] { segment.Start, segment.End })
                if (points.Count == 0 || math.distancesq(points[^1].xz, point.xz) > 1e-6f)
                    points.Add(point);

        return points;
    }

    private static float3 At(IReadOnlyList<float3> points, float fraction)
    {
        var target = fraction * PathLength(points);

        for (var i = 1; i < points.Count; i++)
        {
            var step = math.distance(points[i - 1].xz, points[i].xz);

            if (target <= step)
                return math.lerp(points[i - 1], points[i], step > 0 ? target / step : 0);

            target -= step;
        }

        return points[^1];
    }

    // Native widths are not resolved yet; a street's width grows with its lanes
    // (two lanes are about 16 m wide, six about 32 m). The connector must be
    // shorter than half the widest road at either junction.
    private static float Limit(long a, long b, IReadOnlyDictionary<long, List<PreparedSegment>> incident)
    {
        var lanes = incident[a].Concat(incident[b]).Max(segment => segment.OneWay ? segment.Lanes * 2 : segment.Lanes);
        return Math.Min(16, 8 + 2 * lanes);
    }

    private static int Weight(IEnumerable<PreparedSegment> arms)
        => arms.Sum(segment => Rank(segment.Classification) * 10 + segment.Lanes);

    private static int Rank(string classification) => classification switch
    {
        "primary" => 4,
        "secondary" => 3,
        "tertiary" => 2,
        _ => 1
    };
}
