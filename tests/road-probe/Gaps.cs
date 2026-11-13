using Unity.Mathematics;

// Breaks in a plan: dead ends that face each other, dead ends that stop just short
// of another road, and small pieces joined to nothing. Reads a replay's plan when
// PLAN_DIR is set, else what the game received.
public static class Gaps
{
    private const double FacingReach = 80, ShortReach = 30, SmallPiece = 150, EdgeMargin = 300;

    private sealed record End(Sec Section, long Node, double3 Point, double2 Outward);

    public static void Run(string map, int examples)
    {
        var capture = new Cap(map);
        var all = Triage.PlanOf(capture);

        foreach (var kind in new[] { "Road", "Rail" })
        {
            var plan = all.Where(section => section.Kind == kind).ToList();

            if (plan.Count == 0)
                continue;

            var degree = new Dictionary<long, int>();

            foreach (var section in plan)
            {
                degree[section.A] = degree.GetValueOrDefault(section.A) + 1;
                degree[section.B] = degree.GetValueOrDefault(section.B) + 1;
            }

            var half = capture.Width / 2 - EdgeMargin;
            var ends = new List<End>();

            foreach (var section in plan)
            {
                if (degree[section.A] == 1 && Inside(section.P0, half))
                    ends.Add(new(section, section.A, section.P0, -Heading(section, 0)));

                if (degree[section.B] == 1 && Inside(section.P3, half))
                    ends.Add(new(section, section.B, section.P3, Heading(section, 1)));
            }

            var cells = new Dictionary<(int, int), List<(double3 Point, Sec Section)>>();

            foreach (var section in plan)
            {
                var steps = Math.Max(2, (int)Math.Ceiling(section.ArcLength() / 4));

                for (var k = 0; k <= steps; k++)
                {
                    var point = section.At(k / (double)steps);
                    var key = ((int)Math.Floor(point.x / 32), (int)Math.Floor(point.z / 32));

                    if (!cells.TryGetValue(key, out var items))
                        cells[key] = items = new();

                    items.Add((point, section));
                }
            }

            var facing = new List<(End First, End Second, double Gap)>();
            var paired = new HashSet<long>();

            foreach (var end in ends)
                foreach (var other in ends)
                {
                    if (end.Node >= other.Node)
                        continue;

                    var offset = other.Point.xz - end.Point.xz;
                    var gap = math.length(offset);

                    if (gap > FacingReach || gap < .01 || math.dot(end.Outward, other.Outward) > -.7)
                        continue;

                    var along = math.dot(offset, end.Outward);
                    var aside = Math.Abs(offset.x * end.Outward.y - offset.y * end.Outward.x);

                    if (along <= 0 || aside > 3 + .25 * along)
                        continue;

                    facing.Add((end, other, gap));
                    paired.Add(end.Node);
                    paired.Add(other.Node);
                }

            var stopsShort = new List<(End End, double Gap, double Rise, Sec Other)>();

            foreach (var end in ends)
            {
                if (paired.Contains(end.Node))
                    continue;

                var best = double.MaxValue;
                var rise = 0d;
                Sec? found = null;
                var x = (int)Math.Floor(end.Point.x / 32);
                var z = (int)Math.Floor(end.Point.z / 32);

                for (var dz = -1; dz <= 1; dz++)
                    for (var dx = -1; dx <= 1; dx++)
                        if (cells.TryGetValue((x + dx, z + dz), out var items))
                            foreach (var item in items)
                            {
                                if (item.Section.A == end.Section.A
                                    || item.Section.A == end.Section.B
                                    || item.Section.B == end.Section.A
                                    || item.Section.B == end.Section.B)
                                    continue;

                                var offset = item.Point.xz - end.Point.xz;
                                var gap = math.length(offset);

                                if (gap >= best || gap > ShortReach || math.dot(offset, end.Outward) < .5 * gap)
                                    continue;

                                best = gap;
                                rise = item.Point.y - end.Point.y;
                                found = item.Section;
                            }

                if (found != null)
                    stopsShort.Add((end, best, rise, found));
            }

            // Connected pieces by total length.
            var parent = new Dictionary<long, long>();

            long Find(long id)
            {
                while (parent.TryGetValue(id, out var next) && next != id)
                    id = parent[id] = parent.GetValueOrDefault(next, next);

                return id;
            }

            foreach (var section in plan)
            {
                parent.TryAdd(section.A, section.A);
                parent.TryAdd(section.B, section.B);
                parent[Find(section.A)] = Find(section.B);
            }

            var pieces =
                plan
                    .GroupBy(section => Find(section.A))
                    .Select(group => (Sections: group.ToList(), Length: group.Sum(section => section.ArcLength())))
                    .ToList();

            var small =
                pieces
                    .Where(piece => piece.Length < SmallPiece && piece.Sections.All(section => Inside(section.P0, half)))
                    .OrderBy(piece => piece.Length)
                    .ToList();

            var level = stopsShort.Where(item => Math.Abs(item.Rise) < 2).ToList();

            Console.WriteLine(
                $"{map} {kind}: dead ends inland {ends.Count}; facing pairs within {FacingReach:F0} m {facing.Count} "
                    + $"(under 10 m {facing.Count(item => item.Gap < 10)}, under 30 m {facing.Count(item => item.Gap < 30)}); "
                    + $"stopping within {ShortReach:F0} m of another {kind.ToLowerInvariant()} {stopsShort.Count} "
                    + $"(same level {level.Count}, of those under 10 m {level.Count(item => item.Gap < 10)}); "
                    + $"pieces {pieces.Count}, under {SmallPiece:F0} m and inland {small.Count}");

            string Place(double3 point)
            {
                var geographic = capture.ToGeographic(point.x, point.z);
                return $"{geographic.lat:F5},{geographic.lon:F5} ({point.x:F0},{point.z:F0})";
            }

            foreach (var item in facing.OrderBy(item => item.Gap).Take(examples))
                Console.WriteLine(
                    $"  facing {item.Gap,5:F1} m rise {item.Second.Point.y - item.First.Point.y,5:F1} {Place(item.First.Point)} "
                        + $"{item.First.Section.Cls} way {item.First.Section.Way} node {item.First.Node} "
                        + $"{Flags(item.First.Section)} | {item.Second.Section.Cls} way {item.Second.Section.Way} "
                        + $"node {item.Second.Node} {Flags(item.Second.Section)}");

            foreach (var item in level.OrderBy(item => item.Gap).Take(examples))
                Console.WriteLine(
                    $"  short  {item.Gap,5:F1} m rise {item.Rise,5:F1} {Place(item.End.Point)} {item.End.Section.Cls} "
                        + $"way {item.End.Section.Way} node {item.End.Node} len {item.End.Section.ArcLength():F0} "
                        + $"{Flags(item.End.Section)} | {item.Other.Cls} way {item.Other.Way} {Flags(item.Other)}");

            foreach (var piece in small.Take(examples))
                Console.WriteLine(
                    $"  piece  {piece.Length,5:F0} m in {piece.Sections.Count} sections {Place(piece.Sections[0].P0)} "
                        + $"{piece.Sections[0].Cls} way {piece.Sections[0].Way} {Flags(piece.Sections[0])}");
        }
    }

    // Every section with an end within the radius of a point, with its node ids.
    public static void Near(string map, double x, double z, double radius, bool source)
    {
        var capture = new Cap(map);
        var all = source ? Compare.Input(capture) : Triage.PlanOf(capture);
        var centre = new double2(x, z);
        var degree = new Dictionary<(string, long), int>();

        foreach (var section in all)
        {
            degree[(section.Kind, section.A)] = degree.GetValueOrDefault((section.Kind, section.A)) + 1;
            degree[(section.Kind, section.B)] = degree.GetValueOrDefault((section.Kind, section.B)) + 1;
        }

        foreach (var section in all
            .Where(
                section => math.distance(section.P0.xz, centre) < radius
                    || math.distance(section.P3.xz, centre) < radius)
            .OrderBy(section => section.Kind)
            .ThenBy(section => section.Way))
            Console.WriteLine(
                $" {section.Kind} {section.Cls,-13} way {section.Way,12} L{section.ArcLength(),5:F0} w{section.Width,4:F0} "
                    + $"{section.A}[{degree[(section.Kind, section.A)]}]@({section.P0.x:F0},{section.P0.z:F0},{section.P0.y:F1}) -> "
                    + $"{section.B}[{degree[(section.Kind, section.B)]}]@({section.P3.x:F0},{section.P3.z:F0},{section.P3.y:F1}) "
                    + $"{Flags(section)} L{section.Layer}");
    }

    // Every map-data way through a node, with the tags that decide whether it is imported.
    public static void OsmNode(string map, IEnumerable<long> nodes)
    {
        var capture = new Cap(map);
        var data = capture.LoadOsm();

        foreach (var node in nodes)
        {
            Console.WriteLine($"node {node}:");

            foreach (var way in data.Ways.Values.Where(way => way.NodeIds.Contains(node)))
            {
                var tags = data.RawWayTags.TryGetValue(way.Id, out var raw) ? raw : way.Tags;
                var ids = way.NodeIds.ToList();
                var index = ids.IndexOf(node);

                var shown =
                    string.Join(
                        " ",
                        tags
                            .Where(
                                tag => tag.Key is "highway" or "railway" or "junction" or "bridge" or "tunnel"
                                    or "layer" or "access" or "area" or "service" or "construction" or "oneway")
                            .Select(tag => $"{tag.Key}={tag.Value}"));

                Console.WriteLine($"  way {way.Id} at {index}/{ids.Count - 1}: {shown}");
            }
        }
    }

    private static string Flags(Sec section) => section.Bridge ? "bridge" : section.Tunnel ? "tunnel" : "ground";

    private static bool Inside(double3 point, double half) => Math.Abs(point.x) < half && Math.Abs(point.z) < half;

    private static double2 Heading(Sec section, double t)
    {
        var direction = section.Derivative(t).xz;

        // A control point on the end leaves no tangent there; the chord stands in.
        if (math.lengthsq(direction) < 1e-9)
            direction = section.P3.xz - section.P0.xz;

        return math.normalizesafe(direction);
    }
}
