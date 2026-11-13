using Unity.Mathematics;

// The two one-way carriageways of a divided road in a plan: whether their tunnels
// and bridges start side by side, and how evenly the two are spaced along the road.
// Reads a replay's plan when PLAN_DIR is set, else what the game received.
public static class Pairs
{
    // Portals further apart than this along the road are staggered (metres).
    private const double Stagger = 10;

    // Carriageways of one road are within this distance of each other (metres).
    private const double Reach = 60;

    // A portal at least this far to the side belongs to the other carriageway (metres).
    private const double Beside = 4;

    private const double Cell = 64;

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

        foreach (var tunnel in new[] { true, false })
        {
            bool Is(Sec section) => tunnel ? section.Tunnel : section.Bridge;

            // A portal: the node where a one-way structure meets a road that is not one,
            // with the direction pointing into the structure.
            var portals = new List<(long Node, double2 Point, double2 Into, string Cls)>();

            foreach (var section in plan.Where(section => Is(section) && section.OneWay && !section.Roundabout))
                foreach (var start in new[] { true, false })
                {
                    var node = start ? section.A : section.B;

                    if (at[node].Count != 2 || at[node].All(Is))
                        continue;

                    var into = math.normalizesafe(section.Derivative(start ? 0 : 1).xz) * (start ? 1 : -1);
                    portals.Add((node, (start ? section.P0 : section.P3).xz, into, Hw.Group(section.Cls)));
                }

            var rows = new List<(double Offset, string Text)>();
            var matched = 0;

            for (var i = 0; i < portals.Count; i++)
            {
                var best = -1;
                var bestDistance = Reach;

                for (var j = 0; j < portals.Count; j++)
                {
                    if (i == j || portals[i].Cls != portals[j].Cls || math.dot(portals[i].Into, portals[j].Into) < .9)
                        continue;

                    var across = portals[j].Point - portals[i].Point;
                    var distance = math.length(across);
                    var along = Math.Abs(math.dot(across, portals[i].Into));

                    // Beside this portal, not further along the same carriageway.
                    if (Math.Sqrt(Math.Max(0, distance * distance - along * along)) < Beside)
                        continue;

                    if (distance < bestDistance)
                    {
                        bestDistance = distance;
                        best = j;
                    }
                }

                // Each pair once, and only when the two chose each other.
                if (best < i)
                    continue;

                matched++;
                var offset = Math.Abs(math.dot(portals[best].Point - portals[i].Point, portals[i].Into));
                var geographic = capture.ToGeographic(portals[i].Point.x, portals[i].Point.y);

                rows.Add(
                    (offset,
                        $"{offset,5:F1} m along, {bestDistance,5:F1} m apart {portals[i].Cls,-14} "
                            + $"{geographic.lat:F5},{geographic.lon:F5} ({portals[i].Point.x:F0},{portals[i].Point.y:F0})"));
            }

            Console.WriteLine(
                $"{map}: one-way {(tunnel ? "tunnel" : "bridge")} ends {portals.Count}; side by side with another {matched}; "
                    + $"staggered over {Stagger} m {rows.Count(row => row.Offset > Stagger)}, "
                    + $"over 25 m {rows.Count(row => row.Offset > 25)}");

            foreach (var row in rows.Where(row => row.Offset > Stagger).OrderByDescending(row => row.Offset).Take(examples))
                Console.WriteLine("  staggered " + row.Text);
        }

        Spacing(map, "map", capture, Compare.Input(capture).Where(section => section.Kind == "Road").ToList(), 0);
        Spacing(map, "plan", capture, plan, examples);
    }

    // The gap between the two carriageways of a divided road should change slowly. For
    // each one-way section, the distance to the nearest carriageway running the other
    // way is taken at nine points; a step where it changes by more than Wobble metres
    // per metre travelled is a wobble. No pair ids are needed.
    private static void Spacing(string map, string label, Cap capture, List<Sec> sections, int examples)
    {
        const double Wobble = .15;
        const double Change = 1.5;
        const double Jump = 8;

        var oneWay =
            sections
                .Where(
                    section => section.OneWay
                        && !section.Roundabout
                        && !section.Cls.Contains("link", StringComparison.OrdinalIgnoreCase))
                .ToList();

        var cells = new Dictionary<(string, int, int), List<(double2 Point, double2 Direction)>>();

        foreach (var section in oneWay)
            for (var k = 0; k <= 8; k++)
            {
                var point = section.At(k / 8d).xz;
                var key = (Hw.Group(section.Cls), (int)Math.Floor(point.x / Cell), (int)Math.Floor(point.y / Cell));

                if (!cells.TryGetValue(key, out var list))
                    cells[key] = list = new();

                list.Add((point, math.normalizesafe(section.Derivative(k / 8d).xz)));
            }

        var degree = new Dictionary<long, int>();

        foreach (var section in sections)
        {
            degree[section.A] = degree.GetValueOrDefault(section.A) + 1;
            degree[section.B] = degree.GetValueOrDefault(section.B) + 1;
        }

        var rows = new List<(double Rate, string Text)>();
        double measured = 0, wobbling = 0;

        foreach (var section in oneWay)
        {
            var step = section.ArcLength() / 8;

            // Between junctions only: at a junction the carriageways part on purpose.
            if (step < 4 || degree[section.A] > 2 || degree[section.B] > 2)
                continue;

            var separation = new double[9];

            for (var k = 0; k <= 8; k++)
            {
                var point = section.At(k / 8d).xz;
                var direction = math.normalizesafe(section.Derivative(k / 8d).xz);
                var x = (int)Math.Floor(point.x / Cell);
                var z = (int)Math.Floor(point.y / Cell);
                var nearest = double.MaxValue;

                for (var dx = -1; dx <= 1; dx++)
                    for (var dz = -1; dz <= 1; dz++)
                    {
                        if (!cells.TryGetValue((Hw.Group(section.Cls), x + dx, z + dz), out var list))
                            continue;

                        foreach (var other in list)
                        {
                            if (math.dot(other.Direction, direction) > -.9)
                                continue;

                            // Across the road only: the other carriageway's sample points
                            // are not opposite this one, so the along part is left out.
                            var across = other.Point - point;
                            var along = math.dot(across, direction);

                            if (Math.Abs(along) > step)
                                continue;

                            nearest = Math.Min(nearest, Math.Abs(across.x * direction.y - across.y * direction.x));
                        }
                    }

                separation[k] = nearest > 30 ? double.NaN : nearest;
            }

            for (var k = 0; k < 8; k++)
            {
                if (double.IsNaN(separation[k]) || double.IsNaN(separation[k + 1]))
                    continue;

                measured += step;
                var change = Math.Abs(separation[k + 1] - separation[k]);

                // A bigger jump is a different road coming into reach, not this pair.
                if (change < Change || change > Jump || change / step < Wobble)
                    continue;

                wobbling += step;
                var point = section.At((k + .5) / 8).xz;
                var geographic = capture.ToGeographic(point.x, point.y);

                rows.Add(
                    (change / step,
                        $"{separation[k],5:F1} to {separation[k + 1],5:F1} m in {step,4:F1} m {section.Cls,-14} "
                            + $"{geographic.lat:F5},{geographic.lon:F5} ({point.x:F0},{point.y:F0})"));
            }
        }

        Console.WriteLine(
            $"{map} {label}: divided road measured {measured / 1000:F1} km; "
                + $"gap wobbles on {wobbling / 1000:F2} km in {rows.Count} places");

        foreach (var row in rows.OrderByDescending(row => row.Rate).Take(examples))
            Console.WriteLine("  wobble " + row.Text);
    }
}
