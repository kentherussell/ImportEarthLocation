using ImportEarthLocation.Core;
using Unity.Mathematics;

// What the game and the plan do, read from a captured run: patterns rather than single sites.
public static class Patterns
{
    public static void Run(string map)
    {
        var capture = new Cap(map);
        ForcedTunnels(capture, map);
        Walls(capture, map);
        Kinks(capture, map);
    }

    // Plan-forced tunnels: what separates a drawn tunnel from an open cut in the mixed band.
    static void ForcedTunnels(Cap capture, string map)
    {
        var t = capture.Heights("native-observed-heightmap.u16");
        var edges = Live.Edges(capture).Where(edge => edge.Kind == "Road").ToList();
        var rows = new List<(bool Tunnel, double Middle, double Ends, double Length, double Width)>();

        foreach (var edge in edges.Where(edge => edge.Has("Tunnel") || edge.Left.Contains("Lowered") && edge.Right.Contains("Lowered")))
        {
            double Cover(double3 point, double2 side)
                => Math.Min(capture.HeightAt(t, point.x + side.x, point.z + side.y), capture.HeightAt(t, point.x - side.x, point.z - side.y)) - point.y;

            var d = math.normalizesafe(edge.At(.55).xz - edge.At(.45).xz);
            var side = new double2(-d.y, d.x) * edge.Width / 2;
            rows.Add((edge.Has("Tunnel"), Cover(edge.At(.5), side), Math.Min(Cover(edge.P0, side), Cover(edge.P3, side)), edge.Len, edge.Width));
        }

        var mixed = rows.Where(row => row.Middle >= 10 && row.Middle < 14).ToList();
        Console.WriteLine($"{map}: edges with 10-14 m of ground at the middle: {mixed.Count(row => row.Tunnel)} tunnel, {mixed.Count(row => !row.Tunnel)} cut");

        foreach (var (low, high) in new[] { (-99d, 2d), (2d, 5d), (5d, 7.5d), (7.5d, 10d), (10d, 99d) })
        {
            var band = mixed.Where(row => row.Ends >= low && row.Ends < high).ToList();
            Console.WriteLine($"  shallowest end {low,5:F1}..{high,5:F1} m: tunnel {band.Count(row => row.Tunnel),4}  cut {band.Count(row => !row.Tunnel),4}");
        }

        foreach (var (low, high) in new[] { (0d, 25d), (25d, 50d), (50d, 100d), (100d, 999d) })
        {
            var band = mixed.Where(row => row.Length >= low && row.Length < high).ToList();
            Console.WriteLine($"  length {low,3:F0}..{high,3:F0} m: tunnel {band.Count(row => row.Tunnel),4}  cut {band.Count(row => !row.Tunnel),4}");
        }

        foreach (var (low, high) in new[] { (0d, 12d), (12d, 18d), (18d, 30d), (30d, 99d) })
        {
            var band = mixed.Where(row => row.Width >= low && row.Width < high).ToList();
            Console.WriteLine($"  width {low,2:F0}..{high,2:F0} m: tunnel {band.Count(row => row.Tunnel),4}  cut {band.Count(row => !row.Tunnel),4}");
        }
    }

    // Walled ground edges: how far from a bridge they are, and whether fill reached them.
    static void Walls(Cap capture, string map)
    {
        var after = capture.Heights("native-observed-heightmap.u16");
        var before = capture.Heights("offline-design-pass-01-heightmap.u16");
        var edges = Live.Edges(capture).Where(edge => edge.Kind == "Road").ToList();
        var structures = edges.Where(edge => edge.Has("Elevated")).SelectMany(edge => new[] { edge.P0, edge.P3 }).ToList();
        var cells = new HashSet<(int, int)>();
        (int, int) Key(double3 point) => ((int)Math.Floor(point.x / 25), (int)Math.Floor(point.z / 25));

        foreach (var point in structures)
            cells.Add(Key(point));

        bool NearBridge(LiveEdge edge)
        {
            foreach (var point in new[] { edge.P0, edge.P3 })
            {
                var (x, z) = Key(point);

                for (var dz = -1; dz <= 1; dz++)
                    for (var dx = -1; dx <= 1; dx++)
                        if (cells.Contains((x + dx, z + dz)))
                            return true;
            }

            return false;
        }

        var walled = edges.Where(edge => edge.Left.Contains("Raised") && edge.Right.Contains("Raised") && !edge.Has("Elevated")).ToList();
        Console.WriteLine($"{map}: edges walled on both sides {walled.Count}, {walled.Sum(edge => edge.Len) / 1000:F1} km");

        // The plan's own final terrain under the same walls: did the ground or the deck differ?
        var designed = capture.Heights("final-design-heightmap.u16");
        var tall = walled.Where(edge => edge.At(.5).y - capture.HeightAt(after, edge.At(.5).x, edge.At(.5).z) >= 4).ToList();
        var groundDiff = tall.Select(edge => capture.HeightAt(designed, edge.At(.5).x, edge.At(.5).z) - capture.HeightAt(after, edge.At(.5).x, edge.At(.5).z)).OrderBy(v => v).ToList();
        var overDesign = tall.Select(edge => edge.At(.5).y - capture.HeightAt(designed, edge.At(.5).x, edge.At(.5).z)).OrderBy(v => v).ToList();

        if (tall.Count > 0)
            Console.WriteLine(
                $"  * tall walls {tall.Count} edges {tall.Sum(edge => edge.Len) / 1000:F2} km: designed ground minus built ground p10 {groundDiff[tall.Count / 10]:F1} p50 {groundDiff[tall.Count / 2]:F1} p90 {groundDiff[tall.Count * 9 / 10]:F1}; deck over designed ground p10 {overDesign[tall.Count / 10]:F1} p50 {overDesign[tall.Count / 2]:F1} p90 {overDesign[tall.Count * 9 / 10]:F1}");

        var planEnds = new Dictionary<(long, long), double>();

        foreach (var section in Compare.Final(capture).Where(section => section.Kind == "Road"))
        {
            planEnds[End(section.P0)] = section.P0.y;
            planEnds[End(section.P3)] = section.P3.y;
        }

        var deckMoves = tall.SelectMany(edge => new[] { edge.P0, edge.P3 }).Where(point => planEnds.ContainsKey(End(point))).Select(point => point.y - planEnds[End(point)]).OrderBy(v => v).ToList();

        if (deckMoves.Count > 0)
            Console.WriteLine($"  * tall wall ends matched to plan {deckMoves.Count}: built deck minus plan deck p10 {deckMoves[deckMoves.Count / 10]:F1} p50 {deckMoves[deckMoves.Count / 2]:F1} p90 {deckMoves[deckMoves.Count * 9 / 10]:F1}");

        // Sharing an end with a bridge, against merely being near one, and how high it stands.
        (long, long) End(double3 point) => ((long)Math.Round(point.x * 2), (long)Math.Round(point.z * 2));
        var degree = new Dictionary<(long, long), int>();
        var bridgeEnds = new HashSet<(long, long)>();

        foreach (var edge in edges)
            foreach (var point in new[] { edge.P0, edge.P3 })
            {
                degree[End(point)] = degree.GetValueOrDefault(End(point)) + 1;

                if (edge.Has("Elevated"))
                    bridgeEnds.Add(End(point));
            }

        foreach (var group in walled.GroupBy(edge =>
        {
            var attached = bridgeEnds.Contains(End(edge.P0)) || bridgeEnds.Contains(End(edge.P3));
            var junction = degree[End(edge.P0)] > 2 || degree[End(edge.P3)] > 2;
            var m = edge.At(.5);
            var height = m.y - capture.HeightAt(after, m.x, m.z);
            var band = height < 2 ? "under 2 m" : height < 4 ? "2-4 m" : height < 8 ? "4-8 m" : "8+ m";

            return $"{(attached ? "joins a bridge" : "no bridge at its ends"),-22} {(junction ? "junction end" : "plain ends"),-13} deck {band}";
        }).OrderByDescending(group => group.Sum(edge => edge.Len)))
            Console.WriteLine($"  * {group.Key,-56} {group.Count(),4} edges {group.Sum(edge => edge.Len) / 1000,5:F2} km");

        foreach (var group in walled.GroupBy(edge =>
        {
            var m = edge.At(.5);
            var raised = capture.HeightAt(after, m.x, m.z) - capture.HeightAt(before, m.x, m.z);
            var state = raised > 1 ? "ground raised" : raised < -1 ? "ground lowered" : "ground untouched";
            return $"{(NearBridge(edge) ? "touches a bridge" : "away from bridges")}, {state}";
        }).OrderByDescending(group => group.Sum(edge => edge.Len)))
            Console.WriteLine($"  {group.Key,-38} {group.Count(),4} edges {group.Sum(edge => edge.Len) / 1000,5:F1} km, deck over ground p50 {group.Select(edge => edge.At(.5).y - capture.HeightAt(after, edge.At(.5).x, edge.At(.5).z)).OrderBy(v => v).ElementAt(group.Count() / 2):F1} m");
    }

    // A named road through a junction: is its turn in OSM, or did the plan add it?
    static void Kinks(Cap capture, string map)
    {
        var data = capture.LoadOsm();
        var plan = Compare.Final(capture).Where(section => section.Kind == "Road" && !section.Bridge && !section.Tunnel).ToList();
        var ports = new Dictionary<long, List<(Sec Section, bool Start)>>();

        foreach (var section in plan)
        {
            foreach (var (node, start) in new[] { (section.A, true), (section.B, false) })
            {
                if (!ports.TryGetValue(node, out var list))
                    ports[node] = list = new();

                list.Add((section, start));
            }
        }

        var waysAt = new Dictionary<long, List<OsmWay>>();

        foreach (var way in data.Ways.Values.Where(way => way.Tags.ContainsKey("highway")))
            foreach (var node in way.NodeIds)
            {
                if (!waysAt.TryGetValue(node, out var list))
                    waysAt[node] = list = new();

                list.Add(way);
            }

        // Unit direction leaving the node along a source way, over about 40 m.
        double2? Away(OsmWay way, long node, bool forward)
        {
            var index = Array.IndexOf(way.NodeIds, node);

            if (index < 0)
                return null;

            var step = forward ? 1 : -1;
            var from = data.Nodes[node];
            var a = capture.Local(from.Latitude, from.Longitude).xz;

            for (var k = index + step; k >= 0 && k < way.NodeIds.Length; k += step)
            {
                if (!data.Nodes.TryGetValue(way.NodeIds[k], out var at))
                    return null;

                var b = capture.Local(at.Latitude, at.Longitude).xz;

                if (math.distance(a, b) >= 40 || k == 0 || k == way.NodeIds.Length - 1)
                    return (double2)math.normalizesafe(b - a);
            }

            return null;
        }

        static double Turn(double2 u, double2 v)
            => Math.Abs(Math.Atan2(u.x * v.y - u.y * v.x, -(u.x * v.x + u.y * v.y)) * 180 / Math.PI);

        var shown = 0;
        var rows = new List<(double Plan, double Source, double3 At, string Cls, double Moved, int Degree, double FirstLength, double SecondLength)>();

        foreach (var (node, list) in ports.Where(entry => entry.Value.Count >= 3 && waysAt.ContainsKey(entry.Key)))
        {
            for (var a = 0; a < list.Count; a++)
                for (var b = a + 1; b < list.Count; b++)
                {
                    var first = list[a];
                    var second = list[b];

                    string? NameOf(Sec section)
                        => section.Ways.Append(section.Way).Select(way => data.Ways.TryGetValue(way, out var found) && found.Tags.TryGetValue("name", out var name) ? name : null).FirstOrDefault(name => name != null);

                    var name = NameOf(first.Section);

                    if (name == null || name != NameOf(second.Section))
                        continue;

                    double2 Leave((Sec Section, bool Start) port)
                    {
                        var d = port.Start ? port.Section.Derivative(0) : -port.Section.Derivative(1);
                        return math.normalizesafe(new double2(d.x, d.z));
                    }

                    var planTurn = Turn(Leave(first), Leave(second));

                    if (planTurn > 45)
                        continue;

                    var named = waysAt[node].Where(way => way.Tags.TryGetValue("name", out var text) && text == name).ToList();
                    var sourceDirections = named.SelectMany(way => new[] { Away(way, node, true), Away(way, node, false) }).Where(v => v.HasValue).Select(v => v!.Value).ToList();

                    if (sourceDirections.Count < 2)
                        continue;

                    var best = double.MaxValue;

                    for (var i = 0; i < sourceDirections.Count; i++)
                        for (var j = i + 1; j < sourceDirections.Count; j++)
                            best = Math.Min(best, Turn(sourceDirections[i], sourceDirections[j]));

                    var at = first.Start ? first.Section.P0 : first.Section.P3;
                    var osm = capture.Local(data.Nodes[node].Latitude, data.Nodes[node].Longitude);
                    var moved = Math.Sqrt((at.x - osm.x) * (at.x - osm.x) + (at.z - osm.z) * (at.z - osm.z));
                    rows.Add((planTurn, best, at, first.Section.Cls, moved, list.Count, first.Section.ArcLength(), second.Section.ArcLength()));

                    if (planTurn > 38 && best < 2 && shown++ < 3)
                    {
                        Console.WriteLine($"  KINK node {node} arms {list.Count}");

                        foreach (var arm in list)
                        {
                            var s = arm.Section;
                            string R(double3 p) => $"({p.x - at.x:F1},{p.z - at.z:F1})";
                            var tags = data.Ways.TryGetValue(s.Way, out var w) ? string.Join(",", w.Tags.Where(tag => tag.Key is "name" or "oneway" or "lanes" or "highway").Select(tag => tag.Value)) : "";
                            Console.WriteLine($"    way {s.Way} oneway {s.OneWay} w{s.Width} start={arm.Start} {R(s.P0)} {R(s.P1)} {R(s.P2)} {R(s.P3)} {tags}");
                        }

                        foreach (var way in waysAt[node])
                        {
                            var pts = way.NodeIds.Where(data.Nodes.ContainsKey).Select(id => capture.Local(data.Nodes[id].Latitude, data.Nodes[id].Longitude)).Where(p => Math.Abs(p.x - at.x) < 90 && Math.Abs(p.z - at.z) < 90).Select(p => $"({p.x - at.x:F0},{p.z - at.z:F0})");
                            Console.WriteLine($"    osm {way.Id} {(way.Tags.TryGetValue("name", out var wayName) ? wayName : "")}: {string.Join(" ", pts)}");
                        }
                    }
                }
        }

        Console.WriteLine($"{map}: named roads through a junction with OSM directions: {rows.Count}");

        foreach (var (low, high) in new[] { (2d, 5d), (5d, 10d), (10d, 20d), (20d, 45d) })
        {
            var band = rows.Where(row => row.Plan >= low && row.Plan < high).ToList();
            Console.WriteLine($"  plan turn {low,2:F0}-{high,2:F0} deg: {band.Count,5}; OSM turn under 2 deg {band.Count(row => row.Source < 2),5}, 2-5 {band.Count(row => row.Source >= 2 && row.Source < 5),5}, within 3 deg of the plan {band.Count(row => Math.Abs(row.Source - row.Plan) < 3),5}");
        }

        var added = rows.Where(row => row.Source < 2 && row.Plan >= 5).ToList();
        Console.WriteLine($"  added turns over 5 deg: {added.Count}; node moved over 2 m {added.Count(row => row.Moved > 2)}; a piece under 30 m {added.Count(row => Math.Min(row.FirstLength, row.SecondLength) < 30)}; arms 3: {added.Count(row => row.Degree == 3)}, 4: {added.Count(row => row.Degree == 4)}, more: {added.Count(row => row.Degree > 4)}");

        foreach (var row in rows.Where(row => row.Source < 2).OrderByDescending(row => row.Plan).Take(10))
        {
            var place = capture.ToGeographic(row.At.x, row.At.z);
            Console.WriteLine($"  added turn {row.Plan,5:F1} deg (OSM {row.Source:F1}) {row.Cls,-10} {place.lat:F5},{place.lon:F5} node moved {row.Moved:F1} m arms {row.Degree} pieces {row.FirstLength:F0}/{row.SecondLength:F0} m");
        }
    }
}
