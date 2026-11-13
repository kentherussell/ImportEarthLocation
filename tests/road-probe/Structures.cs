using Unity.Mathematics;

// Bridge and tunnel runs in a plan: how long each run is, whether anything crosses
// it in plan view, and how short the ground gaps between two runs of one road are.
// Reads a replay's plan when PLAN_DIR is set, else what the game received.
public static class Structures
{
    private const double ShortRun = 40, ShortGap = 40, Cell = 64;

    public static void Run(string map, int examples)
    {
        var capture = new Cap(map);
        var all = Triage.PlanOf(capture).ToList();
        var plan = all.Where(section => section.Kind == "Road").ToList();
        var at = new Dictionary<long, List<Sec>>();

        foreach (var section in plan)
            foreach (var node in new[] { section.A, section.B })
            {
                if (!at.TryGetValue(node, out var list))
                    at[node] = list = new();

                list.Add(section);
            }

        // Sampled courses of every section, road and rail, for plan-view crossings.
        var cells = new Dictionary<(int, int), List<(double2 From, double2 To, Sec Section)>>();

        foreach (var section in all)
            for (var k = 0; k < 8; k++)
            {
                var from = section.At(k / 8d).xz;
                var to = section.At((k + 1) / 8d).xz;
                var key = ((int)Math.Floor(from.x / Cell), (int)Math.Floor(from.y / Cell));

                if (!cells.TryGetValue(key, out var list))
                    cells[key] = list = new();

                list.Add((from, to, section));
            }

        static bool Cross(double2 a, double2 b, double2 c, double2 d)
        {
            double Side(double2 p, double2 q, double2 r) => (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);

            return Side(a, b, c) * Side(a, b, d) < 0 && Side(c, d, a) * Side(c, d, b) < 0;
        }

        bool Crossed(List<Sec> run)
        {
            var nodes = run.SelectMany(section => new[] { section.A, section.B }).ToHashSet();

            foreach (var section in run)
                for (var k = 0; k < 8; k++)
                {
                    var from = section.At(k / 8d).xz;
                    var to = section.At((k + 1) / 8d).xz;
                    var x = (int)Math.Floor(from.x / Cell);
                    var z = (int)Math.Floor(from.y / Cell);

                    for (var dx = -1; dx <= 1; dx++)
                        for (var dz = -1; dz <= 1; dz++)
                        {
                            if (!cells.TryGetValue((x + dx, z + dz), out var list))
                                continue;

                            foreach (var other in list)
                                if (!nodes.Contains(other.Section.A)
                                    && !nodes.Contains(other.Section.B)
                                    && Cross(from, to, other.From, other.To))
                                    return true;
                        }
                }

            return false;
        }

        foreach (var tunnel in new[] { true, false })
        {
            bool Is(Sec section) => tunnel ? section.Tunnel : section.Bridge;

            var seen = new HashSet<Sec>();
            var rows = new List<(double Length, bool Crossed, string Text)>();
            var gaps = new List<(double Length, string Text)>();

            foreach (var first in plan.Where(Is))
            {
                if (!seen.Add(first))
                    continue;

                var run = new List<Sec> { first };
                var queue = new Queue<Sec>();
                queue.Enqueue(first);

                while (queue.Count > 0)
                {
                    var section = queue.Dequeue();

                    foreach (var node in new[] { section.A, section.B })
                        foreach (var next in at[node].Where(Is))
                            if (seen.Add(next))
                            {
                                run.Add(next);
                                queue.Enqueue(next);
                            }
                }

                var length = run.Sum(section => section.ArcLength());
                var crossed = Crossed(run);
                var geographic = capture.ToGeographic(first.P0.x, first.P0.z);

                rows.Add(
                    (length,
                        crossed,
                        $"{length,5:F0} m {first.Cls,-14} pieces {run.Count,2} {(crossed ? "crossed" : "crosses nothing")} "
                            + $"{geographic.lat:F5},{geographic.lon:F5} ({first.P0.x:F0},{first.P0.z:F0})"));
            }

            // Ground between two runs: leave a run's end through two-arm nodes.
            foreach (var section in plan.Where(Is))
                foreach (var start in new[] { section.A, section.B })
                {
                    if (at[start].Count != 2)
                        continue;

                    var current = at[start][0] == section ? at[start][1] : at[start][0];

                    if (Is(current))
                        continue;

                    var node = start;
                    var length = 0d;

                    for (var guard = 0; guard < 50 && length < ShortGap; guard++)
                    {
                        length += current.ArcLength();
                        node = current.A == node ? current.B : current.A;

                        if (at[node].Count != 2)
                            break;

                        var next = at[node][0] == current ? at[node][1] : at[node][0];

                        if (Is(next))
                        {
                            // Each gap is found from both of its ends.
                            if (length < ShortGap && start < node)
                            {
                                var geographic = capture.ToGeographic(current.P0.x, current.P0.z);
                                gaps.Add(
                                    (length,
                                        $"{length,5:F0} m {current.Cls,-14} "
                                            + $"{geographic.lat:F5},{geographic.lon:F5} ({current.P0.x:F0},{current.P0.z:F0})"));
                            }

                            break;
                        }

                        current = next;
                    }
                }

            var name = tunnel ? "tunnel" : "bridge";

            Console.WriteLine(
                $"{map}: {name} runs {rows.Count}; under 15 m {rows.Count(row => row.Length < 15)}, "
                    + $"under {ShortRun} m {rows.Count(row => row.Length < ShortRun)} "
                    + $"(crossing nothing {rows.Count(row => row.Length < ShortRun && !row.Crossed)}); "
                    + $"crossing nothing at any length {rows.Count(row => !row.Crossed)}; "
                    + $"ground gaps under {ShortGap} m between two runs {gaps.Count}");

            foreach (var row in rows.Where(row => !row.Crossed).OrderBy(row => row.Length).Take(examples))
                Console.WriteLine("  short " + row.Text);

            foreach (var gap in gaps.OrderBy(gap => gap.Length).Take(examples))
                Console.WriteLine("  gap   " + gap.Text);
        }
    }
}
