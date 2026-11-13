using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// A short OSM tunnel is usually a road passing under another road. The game draws a tunnel
// only under about 10.5 m of real ground, so building one sank the interchange around it:
// on 6 Nov, runs under 150 m made up 1.9 km of DC's OSM tunnel and grew 7.1 km of untagged
// tunnel around them (report 110). Here the lower road stays on the ground and the road over
// it becomes a bridge, which is what most of these are. A motorway or trunk over a lesser
// road, a railway, or a roundabout overhead keeps the tunnel: lifting those is worse.
public static class ShortUnderpasses
{
    // Total length of a connected run of tunnel treated as an underpass (metres).
    private const float ShortRun = 150;
    private const int Samples = 16;
    private const float Cell = 64;

    public static IReadOnlyList<PreparedSegment> Prepare(
        IReadOnlyList<PreparedSegment> source,
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

        var atNode = new Dictionary<long, int>();

        for (var i = 0; i < source.Count; i++)
        {
            var segment = source[i];

            if (segment.Kind != NetworkKind.Road || !segment.Tunnel)
                continue;

            foreach (var node in new[] { segment.StartNodeId, segment.EndNodeId })
                if (atNode.TryGetValue(node, out var other))
                    parent[Find(i)] = Find(other);
                else
                    atNode[node] = i;
        }

        var runs =
            Enumerable
                .Range(0, source.Count)
                .Where(i => source[i].Kind == NetworkKind.Road && source[i].Tunnel)
                .GroupBy(Find)
                .Where(run => run.Sum(i => NetworkVerticalProfiles.ArcLength(source[i])) < ShortRun)
                .ToArray();

        if (runs.Length == 0)
            return source;

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

        var cells = new Dictionary<(int, int), List<int>>();

        for (var i = 0; i < source.Count; i++)
        {
            if (source[i].Tunnel)
                continue;

            foreach (var key in Line(i).Select(point => Key(point)).Distinct())
            {
                if (!cells.TryGetValue(key, out var list))
                    cells[key] = list = new();

                list.Add(i);
            }
        }

        var grounded = 0;
        var bridged = 0;
        var kept = 0;

        foreach (var run in runs)
        {
            token.ThrowIfCancellationRequested();
            var nodes = new HashSet<long>(run.SelectMany(i => new[] { source[i].StartNodeId, source[i].EndNodeId }));
            var over = new HashSet<int>();

            foreach (var i in run)
            {
                var line = Line(i);

                foreach (var candidate in line
                    .SelectMany(point => Near(point))
                    .Distinct())
                    if (!nodes.Contains(source[candidate].StartNodeId)
                        && !nodes.Contains(source[candidate].EndNodeId)
                        && Crosses(line, Line(candidate)))
                        over.Add(candidate);
            }

            var lowerIsMajor = run.Any(i => NativeAssetKeys.IsHighway(source[i]));

            if (over.Any(
                i => source[i].Kind != NetworkKind.Road
                    || source[i].Roundabout
                    || NativeAssetKeys.IsHighway(source[i]) && !lowerIsMajor))
            {
                kept++;
                log.Count(
                    "Short underpasses kept, "
                        + (over.Any(i => source[i].Kind != NetworkKind.Road)
                            ? "railway"
                            : over.Any(i => source[i].Roundabout) ? "roundabout" : "major road")
                        + " over");

                var first = NetworkNormalizer.Evaluate(source[run.First()], .5f);
                var overNames = string.Join(",", over.Select(i => $"{source[i].Kind}:{source[i].WayId}:{source[i].Classification}:roundabout={source[i].Roundabout}").Distinct().Take(6));
                log.Diagnostic(
                    $"SHORT UNDERPASS kept x={first.x:0}; z={first.z:0}; lengthMeters={run.Sum(i => NetworkVerticalProfiles.ArcLength(source[i])):0}; lowerWay={source[run.First()].WayId}; lowerIsMajor={lowerIsMajor}; over={overNames}");
                continue;
            }

            foreach (var i in run)
            {
                result[i] =
                    result[i] with { Tunnel = false, Layer = Math.Max(result[i].Layer, 0), StartElevation = null, EndElevation = null };

                grounded++;
            }

            foreach (var i in over)
            {
                if (result[i].Bridge)
                    continue;

                result[i] = result[i] with { Bridge = true, Layer = Math.Max(result[i].Layer, 1) };
                bridged++;
            }
        }

        log.Count("Short underpass sections kept on the ground", grounded);
        log.Count("Sections over a short underpass made bridges", bridged);
        log.Diagnostic(
            $"SHORT UNDERPASSES runs={runs.Length}; keptAsTunnel={kept}; groundedSections={grounded}; bridgedSections={bridged}; shortRunMeters={ShortRun}");

        return result;

        IEnumerable<int> Near(float2 point)
        {
            var (x, z) = Key(point);

            for (var dz = -1; dz <= 1; dz++)
                for (var dx = -1; dx <= 1; dx++)
                    if (cells.TryGetValue((x + dx, z + dz), out var list))
                        foreach (var item in list)
                            yield return item;
        }
    }

    internal static (int, int) Key(float2 point) => ((int)Math.Floor(point.x / Cell), (int)Math.Floor(point.y / Cell));

    internal static bool Crosses(float2[] first, float2[] second)
    {
        for (var a = 1; a < first.Length; a++)
            for (var b = 1; b < second.Length; b++)
                if (Intersects(first[a - 1], first[a], second[b - 1], second[b]))
                    return true;

        return false;
    }

    private static bool Intersects(float2 p, float2 q, float2 r, float2 s)
    {
        var d = q - p;
        var e = s - r;
        var cross = d.x * e.y - d.y * e.x;

        if (Math.Abs(cross) < 1e-6f)
            return false;

        var t = ((r.x - p.x) * e.y - (r.y - p.y) * e.x) / cross;
        var u = ((r.x - p.x) * d.y - (r.y - p.y) * d.x) / cross;

        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }
}
