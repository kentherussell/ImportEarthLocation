using Unity.Mathematics;

// One placement batch of the plan the game received: sections [first, first + count).
// Lists each section and marks what could upset the game: a very short piece, a piece
// whose ends are the same node, a second piece between the same two nodes, a control
// point that is not a number, and a tangent that vanishes at an end.
public static class Batch
{
    // A piece shorter than this is marked short (metres).
    private const double Short = 2;

    // A node where at least this many sections meet is marked busy.
    private const int Busy = 5;

    public static void Run(string map, int first, int count)
    {
        var capture = new Cap(map);
        var plan = Triage.PlanOf(capture);
        var between = new Dictionary<(long, long), int>();
        var degree = new Dictionary<long, int>();

        foreach (var section in plan)
        {
            var key = (Math.Min(section.A, section.B), Math.Max(section.A, section.B));
            between[key] = between.GetValueOrDefault(key) + 1;
            degree[section.A] = degree.GetValueOrDefault(section.A) + 1;
            degree[section.B] = degree.GetValueOrDefault(section.B) + 1;
        }

        var busy = degree.Values.GroupBy(count => count).OrderBy(group => group.Key);
        Console.WriteLine("sections per node: " + string.Join(", ", busy.Select(group => $"{group.Key}:{group.Count()}")));

        Console.WriteLine($"{map}: plan {plan.Count} sections; batch {first}..{first + count - 1}");
        var marked = 0;

        foreach (var section in plan.Skip(first).Take(count))
        {
            var length = section.ArcLength();
            var marks = new List<string>();

            if (length < Short)
                marks.Add("SHORT");

            if (section.A == section.B)
                marks.Add("LOOP");

            if (between[(Math.Min(section.A, section.B), Math.Max(section.A, section.B))] > 1)
                marks.Add("TWIN");

            if (!math.all(math.isfinite(section.P0 + section.P1 + section.P2 + section.P3)))
                marks.Add("NAN");

            if (math.length(section.Derivative(0)) < 1e-3 || math.length(section.Derivative(1)) < 1e-3)
                marks.Add("FLAT-TANGENT");

            var busiest = Math.Max(degree[section.A], degree[section.B]);

            if (busiest >= Busy)
                marks.Add($"DEGREE-{busiest}");

            if (marks.Count > 0)
                marked++;

            var geographic = capture.ToGeographic(section.P0.x, section.P0.z);

            Console.WriteLine(
                $"  {section.Index,6} {section.Kind,-5} {section.Cls,-15} {length,7:F1} m w={section.Width,4:F0} "
                    + $"layer={section.Layer} {(section.Bridge ? "bridge " : "")}{(section.Tunnel ? "tunnel " : "")}"
                    + $"y={section.P0.y:F1}..{section.P3.y:F1} {geographic.lat:F5},{geographic.lon:F5} "
                    + string.Join(" ", marks));
        }

        Console.WriteLine($"marked {marked}");
    }

    // Sections that lie on top of another one, over the whole plan: twins share both
    // nodes; doubles have both ends within reach of another section's ends on the same
    // layer without sharing both nodes.
    public static void Doubles(string map, double reach)
    {
        var capture = new Cap(map);
        var plan = Triage.PlanOf(capture);
        var cell = Math.Max(reach, 1);
        var starts = new Dictionary<(long, long), List<Sec>>();

        foreach (var section in plan)
            foreach (var end in new[] { section.P0, section.P3 })
            {
                var key = ((long)Math.Floor(end.x / cell), (long)Math.Floor(end.z / cell));

                if (!starts.TryGetValue(key, out var list))
                    starts[key] = list = new();

                list.Add(section);
            }

        int twins = 0, doubles = 0;

        foreach (var section in plan)
        {
            var key = ((long)Math.Floor(section.P0.x / cell), (long)Math.Floor(section.P0.z / cell));

            for (var dx = -1; dx <= 1; dx++)
                for (var dz = -1; dz <= 1; dz++)
                {
                    if (!starts.TryGetValue((key.Item1 + dx, key.Item2 + dz), out var list))
                        continue;

                    foreach (var other in list)
                    {
                        if (other.Index <= section.Index || other.Layer != section.Layer)
                            continue;

                        var same = Math.Max(
                            math.distance(other.P0, section.P0),
                            math.distance(other.P3, section.P3));

                        var reversed = Math.Max(
                            math.distance(other.P0, section.P3),
                            math.distance(other.P3, section.P0));

                        if (Math.Min(same, reversed) > reach)
                            continue;

                        var twin = (other.A == section.A && other.B == section.B)
                            || (other.A == section.B && other.B == section.A);

                        if (twin)
                            twins++;
                        else
                            doubles++;

                        var geographic = capture.ToGeographic(section.P0.x, section.P0.z);

                        Console.WriteLine(
                            $"  {(twin ? "twin  " : "double")} {section.Index,6} {section.Cls,-14} "
                                + $"{section.ArcLength(),6:F1} m | {other.Index,6} {other.Cls,-14} "
                                + $"{other.ArcLength(),6:F1} m apart {Math.Min(same, reversed):F1} m "
                                + $"{geographic.lat:F5},{geographic.lon:F5}");
                    }
                }
        }

        Console.WriteLine($"{map}: twins {twins}; doubles {doubles}");
    }

    // The sections of one batch that another capture of the same map does not hold:
    // no section there has both ends within a metre and the same layer and structure.
    public static void Changed(string map, int first, int count, string other)
    {
        var capture = new Cap(map);
        var plan = Triage.PlanOf(capture);
        var earlier = Triage.PlanOf(new Cap(other));

        Console.WriteLine($"{map}: batch {first}..{first + count - 1} against {other}");
        var changed = 0;

        foreach (var section in plan.Skip(first).Take(count))
        {
            // The earlier section whose ends lie nearest this one's, either way round.
            var nearest = double.MaxValue;
            Sec? match = null;

            foreach (var candidate in earlier)
            {
                var same = Math.Max(
                    math.distance(candidate.P0, section.P0),
                    math.distance(candidate.P3, section.P3));

                var reversed = Math.Max(
                    math.distance(candidate.P0, section.P3),
                    math.distance(candidate.P3, section.P0));

                var distance = Math.Min(same, reversed);

                if (distance < nearest)
                {
                    nearest = distance;
                    match = candidate;
                }
            }

            var different = match == null
                || match.Layer != section.Layer
                || match.Bridge != section.Bridge
                || match.Tunnel != section.Tunnel
                || match.Cls != section.Cls
                || Math.Abs(match.Width - section.Width) > 0.5;

            if (nearest < Moved && !different)
                continue;

            changed++;
            var geographic = capture.ToGeographic(section.P0.x, section.P0.z);

            Console.WriteLine(
                $"  {section.Index,6} {section.Cls,-15} {section.ArcLength(),7:F1} m w={section.Width,4:F0} "
                    + $"layer={section.Layer} {(section.Bridge ? "bridge " : "")}{(section.Tunnel ? "tunnel " : "")}"
                    + $"{geographic.lat:F5},{geographic.lon:F5} ends moved {nearest:F1} m"
                    + (match != null && different
                        ? $"; was {match.Cls} w={match.Width:F0} layer={match.Layer} "
                            + $"{(match.Bridge ? "bridge " : "")}{(match.Tunnel ? "tunnel" : "")}"
                        : ""));
        }

        Console.WriteLine($"changed {changed}");
    }

    // Ends further than this from any earlier section's ends count as moved (metres).
    private const double Moved = 3;
}
