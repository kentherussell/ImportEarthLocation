// Width changes at plain two-arm road nodes in a plan: how many, between which
// kinds of road, and how short the run on the shorter side is. Reads a replay's
// plan when PLAN_DIR is set, else what the game received.
public static class Widths
{
    public static void Run(string map, int examples)
    {
        var capture = new Cap(map);
        var plan = Triage.PlanOf(capture).Where(section => section.Kind == "Road").ToList();
        var at = new Dictionary<long, List<Sec>>();

        foreach (var section in plan)
            foreach (var node in new[] { section.A, section.B })
            {
                if (!at.TryGetValue(node, out var list))
                    at[node] = list = new();

                list.Add(section);
            }

        string Level(Sec section) => section.Bridge ? "bridge" : section.Tunnel ? "tunnel" : "ground";

        // Length of the same-width run that starts with a section and leaves a node,
        // through two-arm nodes, and whether it ends at a junction or a dead end.
        (double Length, int Arms) RunFrom(Sec section, long node)
        {
            var length = 0d;

            for (var guard = 0; guard < 200; guard++)
            {
                length += section.ArcLength();
                node = section.A == node ? section.B : section.A;
                var arms = at[node];

                if (arms.Count != 2)
                    return (length, arms.Count);

                var next = arms[0] == section ? arms[1] : arms[0];

                if (next.Width != section.Width)
                    return (length, 2);

                section = next;
            }

            return (length, 2);
        }

        var rows = new List<(string Group, double Short, int Arms, string Text)>();

        foreach (var node in at.Where(node => node.Value.Count == 2 && node.Value[0].Width != node.Value[1].Width))
        {
            var first = node.Value[0];
            var second = node.Value[1];
            var firstRun = RunFrom(first, node.Key);
            var secondRun = RunFrom(second, node.Key);
            var shorter = firstRun.Length <= secondRun.Length ? firstRun : secondRun;
            var point = first.A == node.Key ? first.P0 : first.P3;
            var geographic = capture.ToGeographic(point.x, point.z);

            var group =
                Level(first) != Level(second)
                    ? "structure boundary"
                    : first.OneWay != second.OneWay
                        ? "one-way meets two-way"
                        : first.Cls != second.Cls
                            ? "class change"
                            : "same class";

            rows.Add(
                (group,
                    shorter.Length,
                    shorter.Arms,
                    $"{first.Cls} {first.Width:F0} {Level(first)} | {second.Cls} {second.Width:F0} {Level(second)} "
                        + $"shorter run {shorter.Length:F0} m ending at {shorter.Arms} arms "
                        + $"{geographic.lat:F5},{geographic.lon:F5} ({point.x:F0},{point.z:F0})"));
        }

        Console.WriteLine($"{map}: width changes at two-arm road nodes {rows.Count}");

        foreach (var group in rows.GroupBy(row => row.Group).OrderByDescending(group => group.Count()))
        {
            Console.WriteLine(
                $"  {group.Key,-22} {group.Count(),5}; shorter run under 30 m {group.Count(row => row.Short < 30),5}, "
                    + $"under 60 m {group.Count(row => row.Short < 60),5}, under 150 m {group.Count(row => row.Short < 150),5}; "
                    + $"shorter run ends at a junction {group.Count(row => row.Arms > 2),5}, "
                    + $"dead end {group.Count(row => row.Arms == 1),4}, another width {group.Count(row => row.Arms == 2),5}");

            foreach (var row in group.OrderBy(row => row.Short).Take(examples))
                Console.WriteLine("    " + row.Text);
        }
    }
}
