using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// OSM tags a bridge over every ditch, footpath and culvert. Built, a bridge a few
// metres long is a bump in the road with a sliver of deck and wall, and two of them
// a few metres apart are two bumps. Here a run of bridge shorter than ShortRun that
// crosses no imported road or railway and has no modelled water under it stays on
// the ground, and ground shorter than ShortGap between two bridges of one road,
// through plain two-arm nodes, becomes bridge at their layer.
public static class ShortBridges
{
    // Total length of a connected run of bridge that may stay on the ground (metres).
    private const float ShortRun = 16;

    // Ground between two bridges that is built as bridge (metres).
    private const float ShortGap = 40;

    private const int Samples = 16;

    public static IReadOnlyList<PreparedSegment> Prepare(
        IReadOnlyList<PreparedSegment> source,
        Func<float3, float?>? water,
        ImportLog log,
        CancellationToken token)
    {
        var result = source.ToArray();
        var parent = Enumerable.Range(0, source.Count).ToArray();

        int Find(int i)
        {
            while (parent[i] != i)
                i = parent[i] = parent[parent[i]];

            return i;
        }

        bool RoadBridge(int i) => result[i].Kind == NetworkKind.Road && result[i].Bridge && !result[i].Roundabout;

        var atNode = new Dictionary<long, int>();
        var arms = new Dictionary<long, List<int>>();

        for (var i = 0; i < source.Count; i++)
        {
            if (source[i].Kind != NetworkKind.Road)
                continue;

            foreach (var node in new[] { source[i].StartNodeId, source[i].EndNodeId })
            {
                if (!arms.TryGetValue(node, out var list))
                    arms[node] = list = new();

                list.Add(i);

                if (!RoadBridge(i))
                    continue;

                if (atNode.TryGetValue(node, out var other))
                    parent[Find(i)] = Find(other);
                else
                    atNode[node] = i;
            }
        }

        var points = new Dictionary<int, float2[]>();

        float2[] Line(int i)
        {
            if (!points.TryGetValue(i, out var line))
                points[i] =
                    line =
                        Enumerable
                            .Range(0, Samples + 1)
                            .Select(k => NetworkNormalizer.Evaluate(source[i], k / (float)Samples).xz)
                            .ToArray();

            return line;
        }

        var runs =
            Enumerable
                .Range(0, source.Count)
                .Where(RoadBridge)
                .GroupBy(Find)
                .Where(run => run.Sum(i => NetworkVerticalProfiles.ArcLength(source[i])) < ShortRun)
                .Select(run => run.ToArray())
                .ToArray();

        // A roundabout that is part bridge and part ground: its bridge pieces are judged
        // the same way at any length, so a ring is not half deck for no reason. A ring
        // that is all bridge is a raised roundabout and is left alone.
        var ringParent = Enumerable.Range(0, source.Count).ToArray();

        int Ring(int i)
        {
            while (ringParent[i] != i)
                i = ringParent[i] = ringParent[ringParent[i]];

            return i;
        }

        bool RingPiece(int i) => source[i].Kind == NetworkKind.Road && source[i].Roundabout;

        foreach (var list in arms.Values)
        {
            var pieces = list.Where(RingPiece).ToArray();

            for (var k = 1; k < pieces.Length; k++)
                ringParent[Ring(pieces[k])] = Ring(pieces[0]);
        }

        // The ring's bridge pieces, split into groups that touch end to end.
        IEnumerable<int[]> BridgeGroups(int[] pieces)
        {
            var group = Enumerable.Range(0, pieces.Length).ToArray();

            for (var a = 0; a < pieces.Length; a++)
                for (var b = a + 1; b < pieces.Length; b++)
                {
                    var first = source[pieces[a]];
                    var second = source[pieces[b]];

                    if (first.StartNodeId != second.StartNodeId
                        && first.StartNodeId != second.EndNodeId
                        && first.EndNodeId != second.StartNodeId
                        && first.EndNodeId != second.EndNodeId)
                        continue;

                    var from = group[b];

                    for (var k = 0; k < group.Length; k++)
                        if (group[k] == from)
                            group[k] = group[a];
                }

            return Enumerable
                .Range(0, pieces.Length)
                .GroupBy(k => group[k])
                .Select(members => members.Select(k => pieces[k]).ToArray());
        }

        var ringRuns =
            Enumerable
                .Range(0, source.Count)
                .Where(RingPiece)
                .GroupBy(Ring)
                .Where(ring => ring.Any(i => source[i].Bridge) && ring.Any(i => !source[i].Bridge && !source[i].Tunnel))
                .SelectMany(ring => BridgeGroups(ring.Where(i => source[i].Bridge).ToArray()))
                .ToArray();

        var ringGrounded = 0;
        var cells = new Dictionary<(int, int), List<int>>();

        if (runs.Length + ringRuns.Length > 0)
            for (var i = 0; i < source.Count; i++)
                foreach (var key in Line(i).Select(point => ShortUnderpasses.Key(point)).Distinct())
                {
                    if (!cells.TryGetValue(key, out var list))
                        cells[key] = list = new();

                    list.Add(i);
                }

        var grounded = 0;

        foreach (var run in runs.Concat(ringRuns))
        {
            token.ThrowIfCancellationRequested();
            var nodes = new HashSet<long>(run.SelectMany(i => new[] { source[i].StartNodeId, source[i].EndNodeId }));
            var needed = false;

            foreach (var i in run)
            {
                var line = Line(i);

                foreach (var point in line)
                {
                    // The sampler reads the plan position only; heights are not set yet.
                    needed |= water != null && water(new float3(point.x, 0, point.y)).HasValue;

                    var (x, z) = ShortUnderpasses.Key(point);

                    for (var dz = -1; dz <= 1 && !needed; dz++)
                        for (var dx = -1; dx <= 1 && !needed; dx++)
                            if (cells.TryGetValue((x + dx, z + dz), out var list))
                                needed =
                                    list.Any(
                                        candidate => !nodes.Contains(source[candidate].StartNodeId)
                                            && !nodes.Contains(source[candidate].EndNodeId)
                                            && ShortUnderpasses.Crosses(line, Line(candidate)));

                    if (needed)
                        break;
                }

                if (needed)
                    break;
            }

            if (needed)
                continue;

            foreach (var i in run)
            {
                result[i] =
                    result[i] with
                    {
                        Bridge = false,
                        Layer = Math.Min(result[i].Layer, 0),
                        StartElevation = null,
                        EndElevation = null
                    };

                if (source[i].Roundabout)
                    ringGrounded++;
                else
                    grounded++;
            }
        }

        // Ground between two bridges: leave a bridge's end through two-arm nodes.
        var gaps = 0;

        for (var first = 0; first < source.Count; first++)
        {
            if (!RoadBridge(first))
                continue;

            foreach (var start in new[] { source[first].StartNodeId, source[first].EndNodeId })
            {
                token.ThrowIfCancellationRequested();
                var chain = new List<int>();
                var current = first;
                var node = start;
                var length = 0f;
                var last = -1;

                while (arms[node].Count == 2 && length < ShortGap)
                {
                    var next = arms[node][0] == current ? arms[node][1] : arms[node][0];

                    if (next == current
                        || result[next].Roundabout
                        || result[next].Tunnel
                        || result[next].Classification != source[first].Classification
                        || result[next].OneWay != source[first].OneWay)
                        break;

                    if (result[next].Bridge)
                    {
                        last = next;
                        break;
                    }

                    chain.Add(next);
                    length += NetworkVerticalProfiles.ArcLength(source[next]);
                    current = next;
                    node = source[next].StartNodeId == node ? source[next].EndNodeId : source[next].StartNodeId;
                }

                if (last < 0 || chain.Count == 0 || length >= ShortGap)
                    continue;

                var layer = Math.Min(result[first].Layer, result[last].Layer);

                foreach (var i in chain)
                    result[i] = result[i] with { Bridge = true, Layer = Math.Max(result[i].Layer, layer) };

                gaps++;
            }
        }

        log.Count("Short bridge sections over nothing kept on the ground", grounded);
        log.Count("Short ground gaps between bridges built as bridge", gaps);
        log.Count("Roundabout bridge pieces over nothing kept on the ground", ringGrounded);
        log.Diagnostic(
            $"SHORT BRIDGES runs={runs.Length}; groundedSections={grounded}; bridgedGaps={gaps}; "
                + $"ringBridgeGroups={ringRuns.Length}; ringGroundedSections={ringGrounded}; "
                + $"shortRunMeters={ShortRun}; shortGapMeters={ShortGap}");

        return grounded + gaps + ringGrounded == 0 ? source : result;
    }
}
