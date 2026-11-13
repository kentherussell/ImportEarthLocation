using System.Diagnostics;
using ImportEarthLocation.Core;
using Unity.Mathematics;

// Prototype for report 113 (no code in the mod): every road, not only highways, as one
// penalised least-squares fit per stroke. Strokes are fitted in order of importance. A
// junction is placed by the first stroke through it, as an ordinary sample of that fit,
// and is a fixed point for every later stroke. Compares the fitted lines with the map
// lines: lateral error, how far junctions move, turning per kilometre, and the turn
// through each junction.
public static class StreetFit
{
    private const double Spacing = 5;
    private const double CornerTurn = 30;
    private const double ContinueTurn = 35;
    private const double PinWeight = 1e8;

    private static readonly string[] Classes =
    {
        "motorway", "trunk", "primary", "secondary", "tertiary", "unclassified", "residential", "living_street"
    };

    private sealed record Stroke(int Rank, List<long> Nodes);

    public static void Run(string map, double wavelength, double budget)
    {
        var capture = new Cap(map);
        var data = capture.LoadOsm();
        var ways =
            data.Ways.Values
                .Where(
                    way => way.Tags.TryGetValue("highway", out var text)
                        && Array.IndexOf(Classes, text) >= 0
                        && !way.Tags.ContainsKey("area")
                        && !(way.Tags.TryGetValue("junction", out var junction) && junction is "roundabout" or "circular")
                        && way.NodeIds.Length >= 2
                        && way.NodeIds.All(data.Nodes.ContainsKey))
                .OrderBy(way => way.Id)
                .ToList();

        var position = new Dictionary<long, double2>();

        double2 Source(long id)
        {
            if (!position.TryGetValue(id, out var point))
            {
                var local = capture.Local(data.Nodes[id].Latitude, data.Nodes[id].Longitude);
                position[id] = point = new(local.x, local.z);
            }

            return point;
        }

        int Rank(OsmWay way) => Array.IndexOf(Classes, way.Tags["highway"]);

        bool OneWay(OsmWay way) => way.Tags.TryGetValue("oneway", out var text) && text is "yes" or "1" or "true";

        // Arms per node: a way end counts once, a way interior twice.
        var degree = new Dictionary<long, int>();

        foreach (var way in ways)
            for (var k = 0; k < way.NodeIds.Length; k++)
                degree[way.NodeIds[k]] =
                    degree.GetValueOrDefault(way.NodeIds[k]) + (k == 0 || k == way.NodeIds.Length - 1 ? 1 : 2);

        // Pair way ends at each node by least turning, among ways of one class and direction.
        var endsAt = new Dictionary<long, List<(OsmWay Way, bool AtStart)>>();

        foreach (var way in ways)
            foreach (var atStart in new[] { true, false })
            {
                var node = atStart ? way.NodeIds[0] : way.NodeIds[^1];

                if (!endsAt.TryGetValue(node, out var list))
                    endsAt[node] = list = new();

                list.Add((way, atStart));
            }

        double2 Leaving(OsmWay way, bool atStart)
        {
            var ids = way.NodeIds;

            return math.normalizesafe(
                atStart ? Source(ids[1]) - Source(ids[0]) : Source(ids[^2]) - Source(ids[^1]));
        }

        var next = new Dictionary<(long Way, bool AtStart), (OsmWay Way, bool AtStart)>();

        foreach (var node in endsAt.OrderBy(entry => entry.Key))
        {
            var list = node.Value;
            var candidates = new List<(double Turn, int A, int B)>();

            for (var a = 0; a < list.Count; a++)
                for (var b = a + 1; b < list.Count; b++)
                {
                    if (list[a].Way.Id == list[b].Way.Id
                        || Rank(list[a].Way) != Rank(list[b].Way)
                        || OneWay(list[a].Way) != OneWay(list[b].Way)
                        || OneWay(list[a].Way) && list[a].AtStart == list[b].AtStart)
                        continue;

                    var dot = -math.dot(Leaving(list[a].Way, list[a].AtStart), Leaving(list[b].Way, list[b].AtStart));
                    var turn = math.degrees(Math.Acos(math.clamp(dot, -1, 1)));

                    if (turn < (degree[node.Key] > 2 ? ContinueTurn : CornerTurn))
                        candidates.Add((turn, a, b));
                }

            foreach (var (_, a, b) in candidates.OrderBy(item => item.Turn))
                if (!next.ContainsKey((list[a].Way.Id, list[a].AtStart))
                    && !next.ContainsKey((list[b].Way.Id, list[b].AtStart)))
                {
                    next[(list[a].Way.Id, list[a].AtStart)] = list[b];
                    next[(list[b].Way.Id, list[b].AtStart)] = list[a];
                }
        }

        // Strokes: follow the pairing both ways from each unused way, then break at corners.
        var used = new HashSet<long>();
        var strokes = new List<Stroke>();

        foreach (var seed in ways)
        {
            if (used.Contains(seed.Id))
                continue;

            var start = (Way: seed, AtStart: true);
            var guard = 0;

            while (next.TryGetValue((start.Way.Id, start.AtStart), out var before) && guard++ < 10000)
            {
                if (before.Way.Id == seed.Id)
                    break;

                start = (before.Way, !before.AtStart);
            }

            var nodes = new List<long>();
            var current = start;

            while (used.Add(current.Way.Id))
            {
                var ids = current.AtStart ? current.Way.NodeIds : current.Way.NodeIds.Reverse().ToArray();
                nodes.AddRange(nodes.Count == 0 ? ids : ids.Skip(1));

                if (!next.TryGetValue((current.Way.Id, !current.AtStart), out var following))
                    break;

                current = following;
            }

            var first = 0;

            for (var k = 1; k < nodes.Count; k++)
            {
                var corner = false;

                if (k + 1 < nodes.Count)
                {
                    var into = math.normalizesafe(Source(nodes[k]) - Source(nodes[k - 1]));
                    var out2 = math.normalizesafe(Source(nodes[k + 1]) - Source(nodes[k]));
                    corner = math.degrees(Math.Acos(math.clamp(math.dot(into, out2), -1, 1))) >= CornerTurn;
                }

                if (corner || k == nodes.Count - 1)
                {
                    strokes.Add(new(Rank(seed), nodes.GetRange(first, k - first + 1)));
                    first = k;
                }
            }
        }

        double Length(Stroke stroke)
        {
            var length = 0d;

            for (var k = 1; k < stroke.Nodes.Count; k++)
                length += math.distance(Source(stroke.Nodes[k - 1]), Source(stroke.Nodes[k]));

            return length;
        }

        var order =
            strokes
                .Select(stroke => (Stroke: stroke, Length: Length(stroke)))
                .OrderBy(item => item.Stroke.Rank)
                .ThenByDescending(item => item.Length)
                .ThenBy(item => item.Stroke.Nodes[0])
                .ToList();

        var placed = new Dictionary<long, double2>();
        var lateral = new List<double>();
        var moved = new List<double>();
        var turnBefore = new List<double>();
        var turnAfter = new List<double>();
        double kilometres = 0, wobbleBefore = 0, wobbleAfter = 0;
        var samplesTotal = 0;
        var gaps = new List<double>();
        var watch = new Stopwatch();

        foreach (var (stroke, strokeLength) in order)
        {
            var nodes = stroke.Nodes;
            var anchors = new List<int> { 0 };

            for (var k = 1; k < nodes.Count - 1; k++)
                if (degree[nodes[k]] > 2)
                    anchors.Add(k);

            anchors.Add(nodes.Count - 1);

            for (var a = 1; a < anchors.Count; a++)
            {
                var gap = 0d;

                for (var k = anchors[a - 1] + 1; k <= anchors[a]; k++)
                    gap += math.distance(Source(nodes[k - 1]), Source(nodes[k]));

                if (degree[nodes[anchors[a - 1]]] > 2 && degree[nodes[anchors[a]]] > 2)
                    gaps.Add(gap);
            }

            // Samples about every Spacing metres, with every anchor as a sample of its own.
            var samples = new List<double2>();
            var anchorSample = new List<int>();

            for (var a = 0; a + 1 < anchors.Count; a++)
            {
                var line = new List<double2>();

                for (var k = anchors[a]; k <= anchors[a + 1]; k++)
                    line.Add(Source(nodes[k]));

                var length = 0d;

                for (var k = 1; k < line.Count; k++)
                    length += math.distance(line[k - 1], line[k]);

                var count = Math.Max(1, (int)Math.Round(length / Spacing));

                if (a == 0)
                {
                    samples.Add(line[0]);
                    anchorSample.Add(0);
                }

                var walked = 0d;
                var at = 1;

                for (var i = 1; i < count; i++)
                {
                    var target = length * i / count;

                    while (at < line.Count - 1 && walked + math.distance(line[at - 1], line[at]) < target)
                    {
                        walked += math.distance(line[at - 1], line[at]);
                        at++;
                    }

                    var span = math.distance(line[at - 1], line[at]);
                    samples.Add(math.lerp(line[at - 1], line[at], span > 1e-9 ? (target - walked) / span : 0));
                }

                samples.Add(line[^1]);
                anchorSample.Add(samples.Count - 1);
            }

            var total = samples.Count;
            var target2 = samples.ToArray();
            var weights = new double[total];
            Array.Fill(weights, 1d);

            // Stroke ends and junctions an earlier stroke placed are fixed; the rest are free.
            for (var a = 0; a < anchors.Count; a++)
            {
                var node = nodes[anchors[a]];

                if (placed.TryGetValue(node, out var fixedPoint))
                {
                    target2[anchorSample[a]] = fixedPoint;
                    weights[anchorSample[a]] = PinWeight;
                }
                else if (a == 0 || a == anchors.Count - 1)
                    weights[anchorSample[a]] = PinWeight;
            }

            double2[] fitted;

            if (total < 4)
                fitted = target2;
            else
            {
                watch.Start();
                var mu = Math.Pow(wavelength / (2 * Math.PI * Spacing), 6);
                fitted = Solve(target2, weights, mu);

                for (var round = 0; round < 4; round++)
                {
                    var over = false;

                    for (var i = 0; i < total; i++)
                        if (weights[i] < PinWeight && math.distance(fitted[i], samples[i]) > budget)
                        {
                            over = true;

                            for (var j = Math.Max(0, i - 4); j <= Math.Min(total - 1, i + 4); j++)
                                if (weights[j] < PinWeight)
                                    weights[j] = Math.Min(1e4, weights[j] * 4);
                        }

                    if (!over)
                        break;

                    fitted = Solve(target2, weights, mu);
                }

                watch.Stop();
            }

            samplesTotal += total;

            for (var a = 0; a < anchors.Count; a++)
            {
                var node = nodes[anchors[a]];

                if (!placed.ContainsKey(node))
                {
                    placed[node] = fitted[anchorSample[a]];

                    if (degree[node] > 2)
                        moved.Add(math.distance(fitted[anchorSample[a]], Source(node)));
                }
            }

            if (total < 4)
                continue;

            kilometres += strokeLength / 1000;

            for (var i = 0; i < total; i++)
            {
                var best = double.MaxValue;

                for (var j = Math.Max(1, i - 6); j <= Math.Min(total - 1, i + 6); j++)
                    best = Math.Min(best, Hw.DistSeg(fitted[i], samples[j - 1], samples[j]));

                lateral.Add(best);
            }

            double Turn(double2[] points, int i, int reach)
            {
                var into = math.normalizesafe(points[i] - points[i - reach]);
                var out2 = math.normalizesafe(points[i + reach] - points[i]);

                return math.degrees(Math.Acos(math.clamp(math.dot(into, out2), -1, 1)));
            }

            var raw = samples.ToArray();

            for (var i = 1; i + 1 < total; i++)
            {
                wobbleBefore += Turn(raw, i, 1);
                wobbleAfter += Turn(fitted, i, 1);
            }

            for (var a = 1; a + 1 < anchors.Count; a++)
            {
                var i = anchorSample[a];

                if (i < 2 || i + 2 >= total)
                    continue;

                turnBefore.Add(Turn(raw, i, 2));
                turnAfter.Add(Turn(fitted, i, 2));
            }
        }

        lateral.Sort();
        moved.Sort();
        turnBefore.Sort();
        turnAfter.Sort();
        gaps.Sort();

        double At(List<double> list, double q) => list.Count == 0 ? 0 : list[(int)(q * (list.Count - 1))];

        Console.WriteLine(
            $"{map}: wavelength {wavelength:F0} m, budget {budget:F1} m; {ways.Count} ways, {strokes.Count} strokes, {kilometres:F0} km, {samplesTotal} samples, fit {watch.ElapsedMilliseconds} ms");
        Console.WriteLine(
            $"  lateral from map line: p50 {At(lateral, .5):F2} p95 {At(lateral, .95):F2} p99 {At(lateral, .99):F2} max {At(lateral, 1):F1} m");
        Console.WriteLine(
            $"  junctions placed by a fit: {moved.Count}; moved p50 {At(moved, .5):F2} p95 {At(moved, .95):F2} p99 {At(moved, .99):F2} max {At(moved, 1):F1} m; over 2 m {moved.Count(d => d > 2)}");
        Console.WriteLine(
            $"  turning per km: map line {wobbleBefore / kilometres:F0} deg, fitted {wobbleAfter / kilometres:F0} deg");
        Console.WriteLine(
            $"  turn through a junction over 10 m each side ({turnBefore.Count}): map line p50 {At(turnBefore, .5):F1} p95 {At(turnBefore, .95):F1}, over 5 deg {turnBefore.Count(d => d > 5)}, over 10 deg {turnBefore.Count(d => d > 10)}");
        Console.WriteLine(
            $"                                                     fitted   p50 {At(turnAfter, .5):F1} p95 {At(turnAfter, .95):F1}, over 5 deg {turnAfter.Count(d => d > 5)}, over 10 deg {turnAfter.Count(d => d > 10)}");
        Console.WriteLine(
            $"  junction spacing along strokes ({gaps.Count}): under 10 m {gaps.Count(d => d < 10)}, under 15 m {gaps.Count(d => d < 15)}, under 25 m {gaps.Count(d => d < 25)}; p50 {At(gaps, .5):F0} m");
    }

    // Banded Cholesky of (W + mu D3'D3), half bandwidth three.
    private static double2[] Solve(double2[] points, double[] w, double mu)
    {
        var count = points.Length;
        const int b = 3;
        var a = new double[count, b + 1];
        var d = new[] { -1d, 3, -3, 1 };

        for (var i = 0; i < count; i++)
            a[i, 0] = w[i];

        for (var k = 0; k + 3 < count; k++)
            for (var r = 0; r < 4; r++)
                for (var s = r; s < 4; s++)
                    a[k + r, s - r] += mu * d[r] * d[s];

        for (var i = 0; i < count; i++)
            for (var j = 0; j <= b && i + j < count; j++)
            {
                var sum = a[i, j];

                for (var k = 1; k <= b - j && i - k >= 0; k++)
                    sum -= a[i - k, k] * a[i - k, k + j];

                if (j == 0)
                    a[i, 0] = Math.Sqrt(Math.Max(sum, 1e-12));
                else
                    a[i, j] = sum / a[i, 0];
            }

        var result = new double2[count];

        for (var i = 0; i < count; i++)
            result[i] = points[i] * w[i];

        for (var i = 0; i < count; i++)
        {
            for (var k = 1; k <= b && i - k >= 0; k++)
                result[i] -= a[i - k, k] * result[i - k];

            result[i] /= a[i, 0];
        }

        for (var i = count - 1; i >= 0; i--)
        {
            for (var k = 1; k <= b && i + k < count; k++)
                result[i] -= a[i, k] * result[i + k];

            result[i] /= a[i, 0];
        }

        return result;
    }
}
