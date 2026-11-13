using Newtonsoft.Json.Linq;
using Unity.Mathematics;

// In-game plan (final-prepared-network) and in-game census of two captures of one city, over the lat/lon box they share.
public static class Compare
{
    public static List<Sec> Final(Cap capture)
    {
        var plan = new List<Sec>();

        foreach (var row in capture.ManifestJ("final-prepared-network-chunks.json"))
        {
            double3 ParsePoint(string text)
            {
                var token = row[text]!;
                return token is JArray array
                    ? new((double)array[0], (double)array[1], (double)array[2])
                    : new((double)token["x"]!, (double)token["y"]!, (double)token["z"]!);
            }

            plan.Add(
                new(
                    plan.Count,
                    (long)row["WayId"]!,
                    (long)row["StartNodeId"]!,
                    (long)row["EndNodeId"]!,
                    (int?)row["Layer"] ?? 0,
                    (row["SourceWayIds"]?.Values<long>() ?? Array.Empty<long>()).ToArray(),
                    (string?)row["kind"] ?? "",
                    (string?)row["Classification"] ?? "",
                    (bool?)row["Bridge"] ?? false,
                    (bool?)row["Tunnel"] ?? false,
                    (bool?)row["OneWay"] ?? false,
                    (float?)row["NativeWidth"] ?? 0,
                    ParsePoint("a"),
                    ParsePoint("b"),
                    ParsePoint("c"),
                    ParsePoint("d")));
        }

        return plan;
    }

    public static List<Sec> Input(Cap capture)
    {
        var plan = new List<Sec>();

        foreach (var segment in capture.Manifest<ImportEarthLocation.Core.PreparedSegment>(
            capture.InputValue("networkManifest")))
            plan.Add(
                new(
                    plan.Count,
                    segment.WayId,
                    segment.StartNodeId,
                    segment.EndNodeId,
                    segment.Layer,
                    segment.SourceWayIds.ToArray(),
                    segment.Kind == ImportEarthLocation.Core.NetworkKind.Rail ? "Rail" : "Road",
                    segment.Classification,
                    segment.Bridge,
                    segment.Tunnel,
                    segment.OneWay,
                    segment.NativeWidth,
                    (double3)(float3)segment.CourseStart,
                    (double3)(float3)segment.CourseControl1,
                    (double3)(float3)segment.CourseControl2,
                    (double3)(float3)segment.CourseEnd));

        return plan;
    }

    public static void Near(string map, string baseDir, double lat, double lon, double radius)
    {
        var capture = new Cap(map);
        var data = capture.LoadOsm();
        var otherPoint = capture.Local(lat, lon);

        void Dump(string label, List<Sec> list)
        {
            Console.WriteLine($"--- {label}");

            foreach (var section in list
                .Where(
                    otherSection => Enumerable
                        .Range(0, 9)
                        .Any(
                            k => math.distance(otherSection.At(k / 8d).xz, new double2(otherPoint.x, otherPoint.z))
                                < radius))
                .OrderBy(otherSection => otherSection.Way))
                Console.WriteLine(
                    $" way {section.Way} {section.A}->{section.B} {section.Cls} L{section.Layer} {(section.Bridge ? "bridge" : section.Tunnel ? "tunnel" : "ground")} oneway={section.OneWay} w={section.Width:F0} len={section.ArcLength():F0} ways=[{string.Join(",", section.Ways.Take(6))}] start=({section.P0.x - otherPoint.x:F0},{section.P0.y:F1},{section.P0.z - otherPoint.z:F0}) end=({section.P3.x - otherPoint.x:F0},{section.P3.y:F1},{section.P3.z - otherPoint.z:F0})");
        }

        Dump("design input", Input(capture));
        Dump("final", Final(capture));
        Console.WriteLine("--- OSM");

        foreach (var way in data.Ways.Values)
        {
            if (!way.Tags.ContainsKey("highway"))
                continue;

            var points =
                way.NodeIds
                    .Where(data.Nodes.ContainsKey)
                    .Select(id => capture.Local(data.Nodes[id].Latitude, data.Nodes[id].Longitude))
                    .ToArray();

            if (!points.Any(point => math.distance(point.xz, otherPoint.xz) < radius))
                continue;

            Console.WriteLine(
                $" way {way.Id} nodes {points.Length} {string.Join(",", way.Tags.Where(entry => entry.Key is "highway" or "name" or "ref" or "layer" or "bridge" or "tunnel" or "oneway" or "lanes" or "covered" or "location" or "indoor" or "level").Select(entry => entry.Key + "=" + entry.Value))}");
        }
    }

    public static void Run(
        string firstText,
        string secondText,
        double latMin,
        double latMax,
        double lonMin,
        double lonMax)
    {
        foreach (var name in new[] { firstText, secondText })
        {
            var capture = new Cap(name);
            var plan = Final(capture);

            bool IsInside(double3 point)
            {
                var item = capture.ToGeographic(point.x, point.z);
                return item.lat >= latMin && item.lat <= latMax && item.lon >= lonMin && item.lon <= lonMax;
            }

            Console.WriteLine($"=== {name} ({capture.Folder.Split('\\')[^1]}): planned {plan.Count}");
            var arms = new Dictionary<(string, long), List<(Sec S, bool Start)>>();

            foreach (var section in plan)
                foreach (var (id, st) in new[] { (section.A, true), (section.B, false) })
                {
                    if (!arms.TryGetValue((section.Kind, id), out var items))
                        arms[(section.Kind, id)] = items = new();

                    items.Add((section, st));
                }

            Console.WriteLine(
                "class          sections km    sec/km <30m | R<floor R<floor/km | deg2>2° thru>2° thru>5° | grade: >4% >6% >8% km-weighted | gradeChange worst>limit | bridge km tunnel km | widths");

            foreach (var text in new[] { "motorway", "motorway_link", "trunk", "trunk_link" })
            {
                var mine =
                    plan
                        .Where(
                            section => section.Kind.Equals("road", StringComparison.OrdinalIgnoreCase)
                                && Hw.Group(section.Cls) == text
                                && IsInside(section.At(.5)))
                        .ToList();

                if (mine.Count == 0)
                    continue;

                var km = mine.Sum(section => section.ArcLength()) / 1000;
                var floor = Hw.FloorOf(text);
                var rf = mine.Count(section =>
{
    var w = 0d;

    for (var k = 0; k <= 16; k++)
        w = Math.Max(w, Math.Abs(section.Curv(k / 16d)));

    return w * floor > 1;
});

                var d2 = new List<double>();
                var thru = new List<double>();

                foreach (var kv in arms)
                {
                    var items = kv.Value;

                    if (!items.Any(
                        item => item.S.Kind.Equals("road", StringComparison.OrdinalIgnoreCase)
                            && Hw.Group(item.S.Cls) == text
                            && IsInside(item.S.At(.5))))
                        continue;

                    if (items.Count == 2)
                        d2.Add(Hw.Deflect(Hw.Away(items[0]), Hw.Away(items[1])));
                    else if (items.Count >= 3)
                    {
                        var same = items.Where(item => Hw.Group(item.S.Cls) == text).ToList();

                        if (same.Count < 2)
                            continue;

                        var best = 180d;

                        for (var p = 0; p < same.Count; p++)
                            for (var q = p + 1; q < same.Count; q++)
                                best = Math.Min(best, Hw.Deflect(Hw.Away(same[p]), Hw.Away(same[q])));

                        thru.Add(best);
                    }
                }

                double grade4 = 0, grade6 = 0, grade8 = 0;
                var rate = 0;

                foreach (var section in mine)
                {
                    var n = Math.Max(2, (int)(section.ArcLength() / 10));
                    var previous = double.NaN;
                    var bad = false;

                    for (var k = 0; k <= n; k++)
                    {
                        var d = section.Derivative(k / (double)n);
                        var otherLength = math.length(d.xz);

                        if (otherLength < 1e-6)
                            continue;

                        var gr = d.y / otherLength;
                        var abs = Math.Abs(gr);
                        var segment = section.ArcLength() / (n + 1);

                        if (abs > .04)
                            grade4 += segment;

                        if (abs > .06)
                            grade6 += segment;

                        if (abs > .08)
                            grade8 += segment;

                        if (!double.IsNaN(previous)
                            && Math.Abs(gr - previous) / (section.ArcLength() / n) * 1000
                                > (text == "motorway" ? .25 : text == "trunk" ? .33 : .67))
                            bad = true;

                        previous = gr;
                    }

                    if (bad)
                        rate++;
                }

                Console.WriteLine(
                    $"{text,-14} {mine.Count,6} {km,6:F0} {mine.Count / km,6:F1} {mine.Count(section => section.ArcLength() < 30),4} | {rf,6} {rf / km,6:F2} | {d2.Count(v => v > 2),5} {thru.Count(v => v > 2),5} {thru.Count(v => v > 5),5} | {grade4 / 1000,5:F1} {grade6 / 1000,5:F1} {grade8 / 1000,5:F1} km | {rate,6} ({rate / km:F2}/km) | {mine.Where(section => section.Bridge).Sum(section => section.ArcLength()) / 1000,5:F0} {mine.Where(section => section.Tunnel).Sum(section => section.ArcLength()) / 1000,5:F0} | {string.Join(",", mine.GroupBy(section => section.Width).OrderByDescending(group => group.Count()).Take(5).Select(group => $"{group.Key:F0}x{group.Count()}"))}");
            }
        }
    }
}
