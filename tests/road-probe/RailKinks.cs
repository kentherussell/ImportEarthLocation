using Unity.Mathematics;

// Kinks in planned railway: at a node where exactly two track sections meet, the
// change of heading from one into the other. A train cannot take a sharp turn at a
// joint, and a run of alternating kinks is a zigzag. Reads a replay's plan when
// PLAN_DIR is set, else what the game received.
public static class RailKinks
{
    // A joint turning more than this is a kink (degrees).
    private const double Kink = 5;

    // A joint turning more than this is a sharp kink (degrees).
    private const double Sharp = 15;

    public static void Run(string map, int examples)
    {
        var capture = new Cap(map);
        var plan = Triage.PlanOf(capture).Where(section => section.Kind == "Rail").ToList();
        var at = new Dictionary<long, List<(Sec Section, bool Start)>>();

        foreach (var section in plan)
            foreach (var start in new[] { true, false })
            {
                var node = start ? section.A : section.B;

                if (!at.TryGetValue(node, out var list))
                    at[node] = list = new();

                list.Add((section, start));
            }

        var rows = new List<(double Turn, string Text)>();
        int joints = 0, kinks = 0, sharp = 0;

        foreach (var pair in at)
        {
            if (pair.Value.Count != 2 || pair.Value[0].Section == pair.Value[1].Section)
                continue;

            joints++;

            // Headings leaving the node along each section; straight through is 180 degrees apart.
            var first = Leaving(pair.Value[0].Section, pair.Value[0].Start);
            var second = Leaving(pair.Value[1].Section, pair.Value[1].Start);
            var turn = 180 - math.degrees(math.acos(math.clamp(math.dot(first, second), -1, 1)));

            if (turn < Kink)
                continue;

            kinks++;

            if (turn >= Sharp)
                sharp++;

            var point = pair.Value[0].Start ? pair.Value[0].Section.P0 : pair.Value[0].Section.P3;
            var geographic = capture.ToGeographic(point.x, point.z);

            rows.Add(
                (turn,
                    $"{turn,5:F1} deg between {pair.Value[0].Section.ArcLength(),4:F0} m and "
                        + $"{pair.Value[1].Section.ArcLength(),4:F0} m pieces "
                        + $"{geographic.lat:F5},{geographic.lon:F5} ({point.x:F0},{point.z:F0})"));
        }

        Console.WriteLine(
            $"{map}: rail {plan.Sum(section => section.ArcLength()) / 1000:F0} km; two-piece joints {joints}; "
                + $"turning {Kink} deg or more {kinks}; {Sharp} deg or more {sharp}");

        foreach (var row in rows.OrderByDescending(row => row.Turn).Take(examples))
            Console.WriteLine("  kink " + row.Text);
    }

    private static double2 Leaving(Sec section, bool start)
    {
        return math.normalizesafe(section.Derivative(start ? 0 : 1).xz) * (start ? 1 : -1);
    }
}
