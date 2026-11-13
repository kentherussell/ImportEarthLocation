using System.Globalization;
using Unity.Mathematics;

// Ground roads that stand in the sea: the deck is no higher than Freeboard over the
// sea surface and the terrain the game ended up with is under water there. Reads a
// replay's plan when PLAN_DIR is set, else what the game received. Land below sea
// level behind a dyke counts too, so on a polder map the figure is an upper bound.
public static class Shoreline
{
    // A deck this far over the sea surface is dry (metres).
    private const double Freeboard = .3;

    // Terrain this far under the sea surface is under water (metres).
    private const double Depth = .3;

    public static void Run(string map, double sea, int examples)
    {
        var capture = new Cap(map);
        var terrain = capture.Heights("native-observed-heightmap.u16");
        var rows = new List<(double Length, string Text)>();
        double ground = 0, wet = 0, causeway = 0;

        foreach (var section in Triage.PlanOf(capture))
        {
            if (section.Kind != "Road" || section.Bridge || section.Tunnel)
                continue;

            var length = section.ArcLength();
            var steps = Math.Max(1, (int)Math.Ceiling(length / 10));
            var under = 0;
            var seabed = 0;
            var lowest = double.MaxValue;
            var where = section.At(.5);

            for (var k = 0; k <= steps; k++)
            {
                var point = section.At(k / (double)steps);

                if (capture.HeightAt(terrain, point.x, point.z) > sea - Depth)
                    continue;

                seabed++;

                if (point.y > sea + Freeboard)
                    continue;

                under++;

                if (point.y < lowest)
                {
                    lowest = point.y;
                    where = point;
                }
            }

            ground += length;
            causeway += length * (seabed - under) / (steps + 1);

            if (under == 0)
                continue;

            var share = length * under / (steps + 1);
            wet += share;
            var geographic = capture.ToGeographic(where.x, where.z);

            rows.Add(
                (share,
                    string.Format(
                        CultureInfo.InvariantCulture,
                        "{0,5:F0} m of {1,5:F0} m, deck {2,5:F1} m over the sea {3,-14} {4:F5},{5:F5} ({6:F0},{7:F0})",
                        share,
                        length,
                        lowest - sea,
                        section.Cls,
                        geographic.lat,
                        geographic.lon,
                        where.x,
                        where.z)));
        }

        Console.WriteLine(
            $"{map}: ground road {ground / 1000:F0} km; in the sea {wet / 1000:F2} km on {rows.Count} sections"
                + $"; dry deck over ground that is under water {causeway / 1000:F2} km (sea {sea:F1} m)");

        foreach (var row in rows.OrderByDescending(row => row.Length).Take(examples))
            Console.WriteLine("  wet " + row.Text);
    }
}
