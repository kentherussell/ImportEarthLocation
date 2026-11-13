using Unity.Mathematics;

// Dead-end spurs in a plan: the run of road from an inland dead end back to the first
// junction. A stub is a spur shorter than Stub; a hook is a spur under Hook long whose
// heading turns by more than Turn from the junction to the dead end. Reads a replay's
// plan when PLAN_DIR is set, else what the game received.
public static class Spurs
{
    // A spur shorter than this is a stub (metres).
    private const double Stub = 40;

    // A hook is no longer than this (metres).
    private const double Hook = 150;

    // A hook turns at least this much along its length (degrees).
    private const double Turn = 60;

    public static void Run(string map, int examples)
    {
        var capture = new Cap(map);
        var plan = Triage.PlanOf(capture).Where(section => section.Kind == "Road").ToList();
        var half = capture.Width / 2 - 300;
        var at = new Dictionary<long, List<Sec>>();

        foreach (var section in plan)
            foreach (var node in new[] { section.A, section.B })
            {
                if (!at.TryGetValue(node, out var list))
                    at[node] = list = new();

                list.Add(section);
            }

        var rows = new List<(double Length, string Text)>();
        int spurs = 0, stubs = 0, hooks = 0;

        foreach (var first in plan)
            foreach (var fromStart in new[] { true, false })
            {
                var node = fromStart ? first.A : first.B;
                var end = fromStart ? first.P0 : first.P3;

                if (at[node].Count != 1 || Math.Abs(end.x) > half || Math.Abs(end.z) > half)
                    continue;

                // Walk from the dead end to the first junction.
                var section = first;
                var forward = fromStart;
                double length = 0, turn = 0;
                var heading = Heading(section, forward ? 0 : 1, forward);

                while (true)
                {
                    length += section.ArcLength();

                    for (var k = 1; k <= 4; k++)
                    {
                        var next = Heading(section, forward ? k / 4d : 1 - k / 4d, forward);
                        turn += Angle(heading, next);
                        heading = next;
                    }

                    var far = forward ? section.B : section.A;

                    if (at[far].Count != 2 || length > Hook)
                        break;

                    var following = at[far].First(other => other != section);

                    if (following.A == following.B)
                        break;

                    forward = following.A == far;
                    section = following;
                }

                spurs++;
                var stub = length < Stub;
                var hook = !stub && length <= Hook && Math.Abs(turn) >= Turn;

                if (stub)
                    stubs++;

                if (hook)
                    hooks++;

                if (!stub && !hook)
                    continue;

                var geographic = capture.ToGeographic(end.x, end.z);

                rows.Add(
                    (length,
                        $"{(stub ? "stub" : "hook")} {length,5:F0} m, turns {turn,5:F0} deg {first.Cls,-14} "
                            + $"{geographic.lat:F5},{geographic.lon:F5} ({end.x:F0},{end.z:F0})"));
            }

        Console.WriteLine(
            $"{map}: inland dead-end spurs {spurs}; stubs under {Stub} m {stubs}; "
                + $"hooks (under {Hook} m, turning {Turn} deg or more) {hooks}");

        foreach (var row in rows.OrderBy(row => row.Length).Take(examples))
            Console.WriteLine("  " + row.Text);
    }

    private static double2 Heading(Sec section, double t, bool forward)
    {
        return math.normalizesafe(section.Derivative(t).xz) * (forward ? 1 : -1);
    }

    // Signed angle from one heading to the next, in degrees.
    private static double Angle(double2 from, double2 to)
    {
        return math.degrees(math.atan2(from.x * to.y - from.y * to.x, math.dot(from, to)));
    }
}
