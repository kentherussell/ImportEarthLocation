using Unity.Mathematics;

// Sections that cross in plan without a shared node and close enough in height for the
// game to join them. The game then has to cut both pieces itself while it applies them.
// Given a batch, the crossings that the batch brings are listed and the ones applied
// before it are only counted, so a batch that upset the game can be compared with the rest.
public static class Crossings
{
    // Straight pieces used to stand in for each curve.
    private const int Steps = 8;

    // Side of the search grid cells (metres).
    private const double Cell = 100;

    public static void Run(string map, int first, int count, double heightReach)
    {
        var capture = new Cap(map);
        var plan = Triage.PlanOf(capture);
        var grid = new Dictionary<(long, long), List<Sec>>();

        foreach (var section in plan)
            foreach (var key in Cells(section))
            {
                if (!grid.TryGetValue(key, out var list))
                    grid[key] = list = new();

                list.Add(section);
            }

        var seen = new HashSet<(int, int)>();
        var before = 0;
        var inBatch = 0;
        var after = 0;
        var crossedBefore = new Dictionary<int, int>();
        var crossedInBatch = new Dictionary<int, int>();

        foreach (var section in plan)
            foreach (var key in Cells(section))
                foreach (var other in grid[key])
                {
                    if (other.Index >= section.Index || !seen.Add((other.Index, section.Index)))
                        continue;

                    if (other.A == section.A || other.A == section.B || other.B == section.A || other.B == section.B)
                        continue;

                    if (!Cross(other, section, heightReach, out var point, out var gap))
                        continue;

                    // The later section of the pair is the one whose batch makes the crossing.
                    if (section.Index < first)
                    {
                        before++;
                        crossedBefore[other.Index] = crossedBefore.GetValueOrDefault(other.Index) + 1;

                        continue;
                    }

                    if (section.Index >= first + count)
                    {
                        after++;

                        continue;
                    }

                    inBatch++;
                    crossedInBatch[other.Index] = crossedInBatch.GetValueOrDefault(other.Index) + 1;
                    var geographic = capture.ToGeographic(point.x, point.z);

                    Console.WriteLine(
                        $"  {section.Index,6} {section.Cls,-14} layer={section.Layer} x {other.Index,6} "
                            + $"{other.Cls,-14} layer={other.Layer} "
                            + $"{(other.Index >= first ? "same batch" : "placed earlier")} "
                            + $"height gap {gap:F1} m {geographic.lat:F5},{geographic.lon:F5}");
                }

        Console.WriteLine(
            $"{map}: crossings without a node: before the batch {before}, made by the batch {inBatch}, "
                + $"after {after}");

        Console.WriteLine(
            $"  sections crossed more than once: before {crossedBefore.Values.Count(value => value > 1)}, "
                + $"by the batch {crossedInBatch.Values.Count(value => value > 1)}");
    }

    private static IEnumerable<(long, long)> Cells(Sec section)
    {
        var keys = new HashSet<(long, long)>();

        for (var step = 0; step <= Steps; step++)
        {
            var point = section.At(step / (double)Steps);
            keys.Add(((long)Math.Floor(point.x / Cell), (long)Math.Floor(point.z / Cell)));
        }

        // A straight piece can pass through a cell without a sample in it; the cells
        // between two samples at most 200 m apart are covered by their neighbours.
        foreach (var key in keys.ToArray())
            for (var dx = -1; dx <= 1; dx++)
                for (var dz = -1; dz <= 1; dz++)
                    keys.Add((key.Item1 + dx, key.Item2 + dz));

        return keys;
    }

    private static bool Cross(Sec a, Sec b, double heightReach, out double3 point, out double gap)
    {
        point = default;
        gap = 0;

        for (var i = 0; i < Steps; i++)
        {
            var a0 = a.At(i / (double)Steps);
            var a1 = a.At((i + 1) / (double)Steps);

            for (var k = 0; k < Steps; k++)
            {
                var b0 = b.At(k / (double)Steps);
                var b1 = b.At((k + 1) / (double)Steps);
                var d = a1.xz - a0.xz;
                var e = b1.xz - b0.xz;
                var denominator = d.x * e.y - d.y * e.x;

                if (Math.Abs(denominator) < 1e-9)
                    continue;

                var offset = b0.xz - a0.xz;
                var s = (offset.x * e.y - offset.y * e.x) / denominator;
                var t = (offset.x * d.y - offset.y * d.x) / denominator;

                if (s < 0 || s > 1 || t < 0 || t > 1)
                    continue;

                var onA = math.lerp(a0, a1, s);
                var onB = math.lerp(b0, b1, t);
                gap = Math.Abs(onA.y - onB.y);

                if (gap > heightReach)
                    continue;

                point = onA;

                return true;
            }
        }

        return false;
    }
}
