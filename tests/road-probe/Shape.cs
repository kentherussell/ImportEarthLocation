using Unity.Mathematics;

// Report 113, section 7: how round the plan's roundabouts are, and how even the gap is
// between the two carriageways of a divided road. Distances are metres in the horizontal
// plane. Set PLAN_DIR to read a replayed plan instead of the capture's in-game plan.
public static class Shape
{
    const double Spacing = 5, PairReach = 30, PairAngle = 20, RoundWithin = .25;

    public static void Run(string map)
    {
        var capture = new Cap(map);
        var data = capture.LoadOsm();
        var directory = Environment.GetEnvironmentVariable("PLAN_DIR");
        var plan =
            (string.IsNullOrEmpty(directory) ? Compare.Final(capture) : Hw.Load(Path.Combine(directory, map)))
                .Where(section => section.Kind == "Road")
                .ToList();

        bool Ring(Sec section)
            => section.Ways.Append(section.Way).Any(
                way => data.Ways.TryGetValue(way, out var found)
                    && found.Tags.TryGetValue("junction", out var text)
                    && text is "roundabout" or "circular");

        Rings(map, plan.Where(Ring).ToList());
        Pairs(plan.Where(section => !Ring(section)).ToList());
    }

    static void Rings(string map, List<Sec> ring)
    {
        var parent = new Dictionary<long, long>();

        long Find(long id)
        {
            if (!parent.TryGetValue(id, out var found))
                return parent[id] = id;

            return found == id ? id : parent[id] = Find(found);
        }

        foreach (var section in ring)
            parent[Find(section.A)] = Find(section.B);

        var worst = new List<double>();
        var share = new List<double>();

        foreach (var group in ring.GroupBy(section => Find(section.A)))
        {
            var points = new List<double2>();

            foreach (var section in group)
                for (var k = 0; k <= 8; k++)
                    points.Add(section.At(k / 8d).xz);

            if (!Circle(points, out var centre, out var radius))
                continue;

            var residual = points.Max(point => Math.Abs(math.distance(point, centre) - radius));
            worst.Add(residual);
            share.Add(residual / radius);
        }

        worst.Sort();
        share.Sort();

        if (worst.Count == 0)
        {
            Console.WriteLine($"{map}: no roundabouts");
            return;
        }

        Console.WriteLine(
            $"{map}: roundabouts {worst.Count}; within {RoundWithin} m of a circle "
            + $"{worst.Count(value => value <= RoundWithin)} ({worst.Count(value => value <= RoundWithin) / (double)worst.Count:P0}); "
            + $"off the circle median {worst[worst.Count / 2]:F2} m, p90 {worst[worst.Count * 9 / 10]:F2} m; "
            + $"as a share of the radius median {share[share.Count / 2]:P1}");
    }

    // Least-squares circle (Kasa): x² + y² = 2ax + 2by + c.
    static bool Circle(List<double2> points, out double2 centre, out double radius)
    {
        centre = default;
        radius = 0;

        if (points.Count < 3)
            return false;

        var mean = points.Aggregate(double2.zero, (sum, point) => sum + point) / points.Count;
        double xx = 0, xy = 0, yy = 0, xr = 0, yr = 0, rr = 0;

        foreach (var point in points)
        {
            var p = point - mean;
            var r = math.lengthsq(p);
            xx += p.x * p.x;
            xy += p.x * p.y;
            yy += p.y * p.y;
            xr += p.x * r;
            yr += p.y * r;
            rr += r;
        }

        var determinant = xx * yy - xy * xy;

        if (Math.Abs(determinant) < 1e-9)
            return false;

        var a = (xr * yy - yr * xy) / (2 * determinant);
        var b = (yr * xx - xr * xy) / (2 * determinant);
        centre = mean + new double2(a, b);
        radius = Math.Sqrt(a * a + b * b + rr / points.Count);

        return true;
    }

    static void Pairs(List<Sec> plan)
    {
        var oneWay = plan.Where(section => section.OneWay && !section.Cls.EndsWith("_link")).ToList();
        var samples = new List<(Sec Section, double2 Point, double2 Direction)>();
        var grid = new Dictionary<(int, int), List<int>>();

        foreach (var section in oneWay)
        {
            var count = Math.Max(1, (int)Math.Round(section.ArcLength() / Spacing));

            for (var k = 0; k <= count; k++)
            {
                var point = section.At(k / (double)count).xz;
                var key = ((int)Math.Floor(point.x / PairReach), (int)Math.Floor(point.y / PairReach));

                if (!grid.TryGetValue(key, out var list))
                    grid[key] = list = new();

                list.Add(samples.Count);
                samples.Add((section, point, math.normalizesafe(section.Derivative(k / (double)count).xz)));
            }
        }

        var opposite = -Math.Cos(math.radians(PairAngle));
        var gap = new double[samples.Count];

        for (var i = 0; i < samples.Count; i++)
        {
            var (section, point, direction) = samples[i];
            var nearest = double.MaxValue;
            gap[i] = double.NaN;
            var cx = (int)Math.Floor(point.x / PairReach);
            var cz = (int)Math.Floor(point.y / PairReach);

            for (var dz = -1; dz <= 1; dz++)
                for (var dx = -1; dx <= 1; dx++)
                {
                    if (!grid.TryGetValue((cx + dx, cz + dz), out var list))
                        continue;

                    foreach (var index in list)
                    {
                        var other = samples[index];

                        if (other.Section.Layer != section.Layer
                            || other.Section.Cls != section.Cls
                            || other.Section.A == section.A
                            || other.Section.A == section.B
                            || other.Section.B == section.A
                            || other.Section.B == section.B
                            || math.dot(direction, other.Direction) > opposite)
                            continue;

                        var distance = math.distance(point, other.Point);

                        if (distance >= nearest || distance > PairReach)
                            continue;

                        nearest = distance;
                        var offset = point - other.Point;
                        gap[i] = Math.Abs(other.Direction.x * offset.y - other.Direction.y * offset.x);
                    }
                }
        }

        var steps = new List<double>();
        var paired = 0;

        for (var i = 0; i < samples.Count; i++)
        {
            if (double.IsNaN(gap[i]))
                continue;

            paired++;

            if (i + 1 < samples.Count && samples[i + 1].Section == samples[i].Section && !double.IsNaN(gap[i + 1]))
                steps.Add(Math.Abs(gap[i + 1] - gap[i]));
        }

        steps.Sort();

        if (steps.Count == 0)
        {
            Console.WriteLine("  divided roads: none found");
            return;
        }

        Console.WriteLine(
            $"  divided roads: {paired * Spacing / 1000:F1} km of carriageway beside its opposite; "
            + $"gap change per {Spacing} m: median {steps[steps.Count / 2]:F2} m, p90 {steps[steps.Count * 9 / 10]:F2} m, "
            + $"p99 {steps[steps.Count * 99 / 100]:F2} m; "
            + $"steady (under 0.05 m) {steps.Count(value => value < .05) / (double)steps.Count:P0}, "
            + $"over 0.5 m {steps.Count(value => value > .5) / (double)steps.Count:P1}");
    }
}
