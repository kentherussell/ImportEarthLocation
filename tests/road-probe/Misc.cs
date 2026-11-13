using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Newtonsoft.Json.Linq;
using System.Text.RegularExpressions;
using Unity.Mathematics;

public static class Misc
{
    static double Area(GeographicPolygon polygon)
    {
        var a = 0d;

        foreach (var points in polygon.Outer)
        {
            var s = 0d;

            for (var k = 0; k < points.Length; k++)
            {
                var u = points[k];
                var v = points[(k + 1) % points.Length];
                s += (double)u.x * v.z - (double)v.x * u.z;
            }

            a += Math.Abs(s) / 2;
        }

        return a;
    }

    // Mapped lakes whose water was dropped by the containment screen, largest first.
    public static void Lakes(string map)
    {
        var capture = new Cap(map);
        var polys =
            capture
                .Manifest<GeographicPolygon>(capture.InputValue("waterPolygonsManifest"))
                .GroupBy(polygon => polygon.Identity)
                .ToDictionary(group => group.Key, group => group.First());

        var log = Path.Combine(capture.Folder, "events.log");
        var dropped = new Dictionary<long, string>();
        var kept = 0;
        var lowered = 0;

        foreach (var line in File.ReadLines(log))
        {
            var match =
                Regex.Match(
                    line,
                    @"WATER CONTAINMENT body=(-?\d+); plannedLevel=([\d.]+); lowestSpill=([\d.E+\-]+); acceptedLevel=([^;]+); sourceRejected=(True|False)");

            if (!match.Success)
                continue;

            var id = long.Parse(match.Groups[1].Value);

            if (match.Groups[5].Value == "True")
                dropped[id] = $"planned {match.Groups[2].Value} spill {match.Groups[3].Value}";
            else
                kept++;
        }

        var total = dropped.Keys.Where(polys.ContainsKey).Sum(k => Area(polys[k])) / 1e6;
        Console.WriteLine($"{map}: rejected {dropped.Count}, passed {kept}; rejected water area {total:F2} km2");

        foreach (var k in dropped.Keys.Where(polys.ContainsKey).OrderByDescending(k => Area(polys[k])).Take(22))
        {
            var polygon = polys[k];
            var points = polygon.Outer.SelectMany(otherPoints => otherPoints).ToArray();
            var low = points.Select(point => point.xz).Aggregate(math.min);
            var high = points.Select(point => point.xz).Aggregate(math.max);
            var item = capture.ToGeographic((low.x + high.x) / 2, (low.y + high.y) / 2);

            string Tag(string text) => polygon.Tags.TryGetValue(text, out var otherText) ? otherText : "-";

            Console.WriteLine(
                $"  {Area(polygon) / 1e4,8:F1} ha {k,12} {item.lat:F4},{item.lon:F4} water={Tag("water")} name={Tag("name")} {dropped[k]}");
        }
    }

    // Bridge and tunnel length: OSM tags against the finished plan, by class.
    public static void Bridges(string map, string baseDir)
    {
        var capture = new Cap(map);
        var data = capture.LoadOsm();
        var plan = Hw.Load(Path.Combine(baseDir, map));
        var half = capture.Width / 2;
        var input = Compare.Input(capture);
        Console.WriteLine(
            "design-input bridge km by class (after all source steps, before the compiler): "
                + string.Join(
                    ", ",
                    new[] {
                        "motorway",
                        "motorway_link",
                        "trunk",
                        "trunk_link",
                        "primary",
                        "secondary",
                        "tertiary"
                        }.Select(
                            text => $"{text} {input.Where(section => section.Kind == "Road" && Hw.Group(section.Cls) == text && section.Bridge).Sum(section => section.ArcLength()) / 1000:F1}")));

        Console.WriteLine(
            $"{map}: class  osm km | osm bridge km  osm tunnel km | plan km  plan bridge km  plan tunnel km | sections bridge  ground | sections <60m bridge  ground");

        foreach (var text in new[] {
            "motorway",
            "motorway_link",
            "trunk",
            "trunk_link",
            "primary",
            "secondary",
            "tertiary"
            })
        {
            double km = 0, br = 0, tu = 0;

            foreach (var way in data.Ways.Values)
            {
                if (!way.Tags.TryGetValue("highway", out var otherText) || Hw.Group(otherText) != text)
                    continue;

                var l = 0d;
                float3? previous = null;

                foreach (var id in way.NodeIds)
                {
                    if (!data.Nodes.TryGetValue(id, out var node))
                        continue;

                    var point = capture.Local(node.Latitude, node.Longitude);

                    if (previous is float3 otherPoint && Math.Abs(point.x) < half && Math.Abs(point.z) < half)
                        l += math.distance(otherPoint.xz, point.xz);

                    previous = point;
                }

                km += l / 1000;

                if (way.Tags.TryGetValue("bridge", out var secondText) && secondText != "no")
                    br += l / 1000;

                if (way.Tags.TryGetValue("tunnel", out var thirdText) && thirdText != "no")
                    tu += l / 1000;
            }

            var mine = plan.Where(section => section.Kind == "Road" && Hw.Group(section.Cls) == text).ToList();
            Console.WriteLine(
                $"{text,-14} {km,7:F0} | {br,6:F1} {tu,6:F1} | {mine.Sum(section => section.ArcLength()) / 1000,7:F0} {mine.Where(section => section.Bridge).Sum(section => section.ArcLength()) / 1000,6:F1} {mine.Where(section => section.Tunnel).Sum(section => section.ArcLength()) / 1000,6:F1} | {mine.Count(section => section.Bridge),6} {mine.Count(section => !section.Bridge && !section.Tunnel),6} | {mine.Count(section => section.Bridge && section.ArcLength() < 60),6} {mine.Count(section => !section.Bridge && !section.Tunnel && section.ArcLength() < 60),6}");
        }
    }

    // Highway ways that never reach the finished plan.
    public static void Coverage(string map, string baseDir)
    {
        var capture = new Cap(map);
        var data = capture.LoadOsm();
        var plan = Hw.Load(Path.Combine(baseDir, map));
        var input = Compare.Input(capture);
        var inPlan = new HashSet<long>(plan.SelectMany(section => section.Ways.Append(section.Way)));
        var inInput = new HashSet<long>(input.SelectMany(section => section.Ways.Append(section.Way)));
        var half = capture.Width / 2 - 200;

        foreach (var text in new[] { "motorway", "motorway_link", "trunk", "trunk_link" })
        {
            var rows = new List<(double L, OsmWay W, bool Input, float3 P)>();

            foreach (var way in data.Ways.Values)
            {
                if (!way.Tags.TryGetValue("highway", out var otherText)
                    || Hw.Group(otherText) != text
                    || inPlan.Contains(way.Id))
                    continue;

                var points =
                    way.NodeIds
                        .Where(data.Nodes.ContainsKey)
                        .Select(id => capture.Local(data.Nodes[id].Latitude, data.Nodes[id].Longitude))
                        .ToArray();

                if (points.Length < 2)
                    continue;

                if (points.Any(point => Math.Abs(point.x) > half || Math.Abs(point.z) > half))
                    continue;

                var l = 0d;

                for (var i = 1; i < points.Length; i++)
                    l += math.distance(points[i - 1].xz, points[i].xz);

                rows.Add((l, way, inInput.Contains(way.Id), points[points.Length / 2]));
            }

            Console.WriteLine(
                $"{map} {text}: ways missing from plan {rows.Count}, {rows.Sum(item => item.L) / 1000:F1} km ({rows.Where(item => item.W.Tags.ContainsKey("tunnel")).Sum(item => item.L) / 1000:F1} km tagged tunnel; {rows.Where(item => item.Input).Sum(item => item.L) / 1000:F1} km lost inside the compiler)");

            foreach (var item in rows.OrderByDescending(otherItem => otherItem.L).Take(6))
            {
                var otherItem = capture.ToGeographic(item.P.x, item.P.z);

                string Tag(string otherText)
                    => item.W.Tags.TryGetValue(otherText, out var secondText) ? secondText : "-";

                Console.WriteLine(
                    $"    {item.L,6:F0} m way {item.W.Id} {otherItem.lat:F4},{otherItem.lon:F4} name={Tag("name")} ref={Tag("ref")} tunnel={Tag("tunnel")} layer={Tag("layer")} lanes={Tag("lanes")}");
            }
        }
    }

    // In-game source obligations: which road classes the lost connections involve.
    public static void Topo(string map)
    {
        var capture = new Cap(map);
        var data = capture.LoadOsm();

        string Class(long way)
            => data.Ways.TryGetValue(way, out var otherWay) && otherWay.Tags.TryGetValue("highway", out var text)
                ? Hw.Group(text)
                : data.Ways.ContainsKey(way) ? "rail/other" : "?";

        var name = "source-topology-obligations.ndjson";
        Console.WriteLine($"{map}: has {name}: {capture.Has(name)}");

        if (!capture.Has(name))
            return;

        var groups = new Dictionary<string, (int Ok, int Bad)>();
        var sample = new List<string>();

        foreach (var otherText in capture.Lines(capture.Resolve(name)))
        {
            var row = JObject.Parse(otherText);
            var ways = row["Ways"]?.Values<long>().ToArray() ?? Array.Empty<long>();
            var reachable = (bool?)row["reachable"] ?? true;
            var best =
                ways
                    .Select(Class)
                    .OrderBy(
                        secondText => Array.IndexOf(
                            new[] {
                                "motorway",
                                "trunk",
                                "motorway_link",
                                "trunk_link",
                                "primary",
                                "secondary",
                                "tertiary",
                                "street",
                                "other",
                                "rail/other",
                                "?"
                                },
                            secondText))
                    .FirstOrDefault()
                    ?? "?";

            groups.TryGetValue(best, out var item);
            groups[best] = reachable ? (item.Ok + 1, item.Bad) : (item.Ok, item.Bad + 1);

            if (!reachable && best is "motorway" or "trunk" && sample.Count < 8)
                sample.Add(otherText.Length > 260 ? otherText.Substring(0, 260) : otherText);
        }

        foreach (var entry in groups.OrderByDescending(otherEntry => otherEntry.Value.Bad))
            Console.WriteLine($"  {entry.Key,-14} reachable {entry.Value.Ok,6}  unresolved {entry.Value.Bad,6}");

        foreach (var otherText in sample)
            Console.WriteLine("   e.g. " + otherText);
    }
}
