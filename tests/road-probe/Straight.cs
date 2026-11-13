using Unity.Mathematics;

// Roads that are straight in OSM against what the plan built from them. Offsets are metres
// from the source way's chord, in the horizontal plane.
public static class Straight
{
    const double MinimumLength = 200, SourceTolerance = 1;

    public static void Run(string map)
    {
        var capture = new Cap(map);
        var data = capture.LoadOsm();
        var directory = Environment.GetEnvironmentVariable("PLAN_DIR");
        var plan =
            (string.IsNullOrEmpty(directory) ? Compare.Final(capture) : Hw.Load(Path.Combine(directory, map)))
                .Where(section => section.Kind == "Road")
                .ToList();
        var byWay = new Dictionary<long, List<Sec>>();

        foreach (var section in plan)
            foreach (var way in section.Ways.Append(section.Way).Distinct())
            {
                if (!byWay.TryGetValue(way, out var list))
                    byWay[way] = list = new();

                list.Add(section);
            }

        var degree = new Dictionary<long, int>();

        foreach (var section in plan)
        {
            degree[section.A] = degree.GetValueOrDefault(section.A) + 1;
            degree[section.B] = degree.GetValueOrDefault(section.B) + 1;
        }

        var rows = new List<(string Cls, double Length, double Wobble, double Kink, bool KinkAtJunction, int Sections, int Merged, long Way, double3 At, bool OneWay)>();

        foreach (var way in data.Ways.Values)
        {
            if (!byWay.TryGetValue(way.Id, out var sections)
                || !way.Tags.TryGetValue("highway", out var cls)
                || way.NodeIds.Any(id => !data.Nodes.ContainsKey(id)))
                continue;

            var points = way.NodeIds.Select(id => capture.Local(data.Nodes[id].Latitude, data.Nodes[id].Longitude)).ToArray();
            var a = points[0].xz;
            var b = points[^1].xz;
            var length = math.distance(a, b);

            if (length < MinimumLength)
                continue;

            var along = (b - a) / length;
            var normal = new float2(-along.y, along.x);

            if (points.Any(point => Math.Abs(math.dot(point.xz - a, normal)) > SourceTolerance))
                continue;

            var low = double.MaxValue;
            var high = double.MinValue;

            // Only the plan pieces that run with this way, not links that merely share its id.
            var running =
                sections
                    .Where(
                        section =>
                        {
                            var d = section.P3 - section.P0;
                            var run = Math.Sqrt(d.x * d.x + d.z * d.z);

                            return run > 1 && Math.Abs((d.x * along.x + d.z * along.y) / run) > .9;
                        })
                    .ToList();

            foreach (var section in running)
                for (var k = 0; k <= 16; k++)
                {
                    var point = section.At(k / 16.0);
                    var dx = point.x - a.x;
                    var dz = point.z - a.y;
                    var t = dx * along.x + dz * along.y;

                    if (t < 0 || t > length)
                        continue;

                    var offset = dx * normal.x + dz * normal.y;
                    low = Math.Min(low, offset);
                    high = Math.Max(high, offset);
                }

            if (running.Count == 0 || high < low)
                continue;

            var kink = 0.0;
            var atJunction = false;
            var at = running[0].P0;

            foreach (var first in running)
                foreach (var second in running)
                    if (first != second && first.B == second.A)
                    {
                        var u = first.Derivative(1);
                        var v = second.Derivative(0);
                        var turn = Math.Abs(
                            Math.Atan2(u.x * v.z - u.z * v.x, u.x * v.x + u.z * v.z) * 180 / Math.PI);

                        if (turn > kink)
                        {
                            kink = turn;
                            atJunction = degree.GetValueOrDefault(first.B) > 2;
                            at = first.P3;
                        }
                    }

            rows.Add((cls, length, high - low, kink, atJunction, running.Count, running.Count(s => s.Ways.Length > 1), way.Id, at, running[0].OneWay));
        }

        Console.WriteLine($"{map}: ways straight in OSM (within {SourceTolerance} m over {MinimumLength}+ m): {rows.Count}, {rows.Sum(row => row.Length) / 1000:F0} km");

        foreach (var group in rows.GroupBy(row => row.Cls).OrderByDescending(group => group.Sum(row => row.Length)))
        {
            int Over(double metres) => group.Count(row => row.Wobble > metres);
            Console.WriteLine(
                $"  {group.Key,-14} ways {group.Count(),5} km {group.Sum(row => row.Length) / 1000,6:F0} | sideways >0.5 m {Over(.5),5} >1 m {Over(1),5} >2 m {Over(2),5} >5 m {Over(5),4} | joint turn >1 deg {group.Count(row => row.Kink > 1),5} >3 deg {group.Count(row => row.Kink > 3),5} >8 deg {group.Count(row => row.Kink > 8),4}");
        }

        foreach (var group in rows.Where(row => row.Merged == 0).GroupBy(row => row.OneWay))
            Console.WriteLine(
                $"  single way, {(group.Key ? "one-way" : "two-way")}: {group.Count()} ways, sideways p50 {group.Select(row => row.Wobble).OrderBy(v => v).ElementAt(group.Count() / 2):F2} m, >1 m {group.Count(row => row.Wobble > 1)}, >2 m {group.Count(row => row.Wobble > 2)}, >5 m {group.Count(row => row.Wobble > 5)}");

        Console.WriteLine($"  merged ways: {rows.Count(row => row.Merged > 0)}, >1 m {rows.Count(row => row.Merged > 0 && row.Wobble > 1)}, >2 m {rows.Count(row => row.Merged > 0 && row.Wobble > 2)}");
        // Where the drift sits along each single way: offset at every plan node on it.
        foreach (var row in rows
            .Where(row => row.Merged == 0 && row.Cls is "primary" or "secondary" or "tertiary" && row.Merged == 0 && row.Wobble > 2 && row.Wobble < 30)
            .OrderBy(row => row.Way % 97)
            .Take(14))
        {
            var way = data.Ways[row.Way];
            var points = way.NodeIds.Select(id => capture.Local(data.Nodes[id].Latitude, data.Nodes[id].Longitude)).ToArray();
            var a = points[0].xz;
            var along = math.normalize(points[^1].xz - a);
            var normal = new float2(-along.y, along.x);
            var stops =
                byWay[row.Way]
                    .SelectMany(section => new[] { (section.A, section.P0, section), (section.B, section.P3, section) })
                    .Select(
                        stop => (
                            T: (stop.Item2.x - a.x) * along.x + (stop.Item2.z - a.y) * along.y,
                            Offset: (stop.Item2.x - a.x) * normal.x + (stop.Item2.z - a.y) * normal.y,
                            Degree: degree.GetValueOrDefault(stop.Item1)))
                    .Where(stop => stop.T >= -5 && stop.T <= row.Length + 5)
                    .Distinct()
                    .OrderBy(stop => stop.T)
                    .Select(stop => $"{stop.T:F0}m:{stop.Offset:F1}/d{stop.Degree}");

            var first = byWay[row.Way][0];
            Console.WriteLine($"  way {row.Way} {row.Cls} oneway={row.OneWay} lanes-width {first.Width} sideways {row.Wobble:F1} tags {(way.Tags.TryGetValue("name", out var name) ? name : "")}: {string.Join("  ", stops)}");
        }

        var turned = rows.Where(row => row.Kink > 3).ToList();
        Console.WriteLine($"joint turns over 3 deg: {turned.Count}; at a junction {turned.Count(row => row.KinkAtJunction)}, at a plain node {turned.Count(row => !row.KinkAtJunction)}");
        var wobbly = rows.Where(row => row.Wobble > 2).ToList();
        Console.WriteLine($"sideways over 2 m: {wobbly.Count}; pieces per km {wobbly.Sum(row => row.Sections) / Math.Max(.001, wobbly.Sum(row => row.Length) / 1000):F1}, others {rows.Except(wobbly).Sum(row => row.Sections) / Math.Max(.001, rows.Except(wobbly).Sum(row => row.Length) / 1000):F1}; with merged ways {wobbly.Count(row => row.Merged > 0)}");

        foreach (var row in rows.OrderByDescending(row => row.Wobble).Take(12))
        {
            var place = capture.ToGeographic(row.At.x, row.At.z);
            Console.WriteLine($"  sideways {row.Wobble,5:F1} m turn {row.Kink,5:F1} deg {row.Cls,-10} {row.Length,5:F0} m pieces {row.Sections,2} way {row.Way} near {place.lat:F4},{place.lon:F4}");
        }

        foreach (var row in rows.Where(row => row.Cls is "primary" or "secondary" or "tertiary" or "motorway" or "trunk").OrderByDescending(row => row.Kink).Take(12))
        {
            var place = capture.ToGeographic(row.At.x, row.At.z);
            Console.WriteLine($"  turn {row.Kink,5:F1} deg junction={row.KinkAtJunction} {row.Cls,-10} sideways {row.Wobble,5:F1} m way {row.Way} at {place.lat:F4},{place.lon:F4}");
        }
    }
}
