using Newtonsoft.Json.Linq;
using Unity.Mathematics;
using System.IO.Compression;

public sealed record Sec(
    int Index,
    long Way,
    long A,
    long B,
    int Layer,
    long[] Ways,
    string Kind,
    string Cls,
    bool Bridge,
    bool Tunnel,
    bool OneWay,
    float Width,
    double3 P0,
    double3 P1,
    double3 P2,
    double3 P3)
{
    // Only a replay's plan carries these; other sources leave them unset.
    public bool Roundabout { get; init; }

    public long Pair { get; init; }

    public double3 At(double t)
    {
        var u = 1 - t;
        return u * u * u * P0 + 3 * u * u * t * P1 + 3 * u * t * t * P2 + t * t * t * P3;
    }

    public double3 Derivative(double t)
    {
        var u = 1 - t;
        return 3 * u * u * (P1 - P0) + 6 * u * t * (P2 - P1) + 3 * t * t * (P3 - P2);
    }

    public double3 SecondDerivative(double t)
    {
        var u = 1 - t;
        return 6 * u * (P2 - 2 * P1 + P0) + 6 * t * (P3 - 2 * P2 + P1);
    }

    public double Curv(double t)
    {
        var d = Derivative(t).xz;
        var e = SecondDerivative(t).xz;
        var otherLength = math.length(d);

        if (otherLength < 1e-6)
            return 0;

        return (d.x * e.y - d.y * e.x) / (otherLength * otherLength * otherLength);
    }

    public double ArcLength()
    {
        var l = 0d;
        var point = At(0);

        for (var k = 1; k <= 16; k++)
        {
            var otherPoint = At(k / 16d);
            l += math.distance(point.xz, otherPoint.xz);
            point = otherPoint;
        }

        return l;
    }
}

public static class Hw
{
    public static List<Sec> Load(string folder)
    {
        var list = new List<Sec>();
        using var file = File.OpenRead(Path.Combine(folder, "plot-sections.ndjson.gz"));
        using var zip = new GZipStream(file, CompressionMode.Decompress);
        using var reader = new StreamReader(zip);

        while (reader.ReadLine() is { } line)
        {
            var row = JObject.Parse(line);
            var p = row["p"]!.Values<double>().ToArray();
            list.Add(
                new(
                    list.Count,
                    (long)row["WayId"]!,
                    (long)row["StartNodeId"]!,
                    (long)row["EndNodeId"]!,
                    (int)row["Layer"]!,
                    row["SourceWayIds"]!.Values<long>().ToArray(),
                    (string)row["kind"]!,
                    (string)row["Classification"]!,
                    (bool)row["Bridge"]!,
                    (bool)row["Tunnel"]!,
                    (bool)row["OneWay"]!,
                    (float)row["NativeWidth"]!,
                new(p[0], p[1], p[2]), new(p[3], p[4], p[5]), new(p[6], p[7], p[8]), new(p[9], p[10], p[11]))
                {
                    Roundabout = (bool?)row["Roundabout"] ?? false,
                    Pair = (long?)row["PairedCorridorId"] ?? 0
                });
        }

        return list;
    }

    public static string Group(string classification) => classification switch
    {
        "motorway" => "motorway",
        "motorway_link" => "motorway_link",
        "trunk" => "trunk",
        "trunk_link" => "trunk_link",
        "primary" or "primary_link" => "primary",
        "secondary" or "secondary_link" => "secondary",
        "tertiary" or "tertiary_link" => "tertiary",
        _ => classification is "residential" or "unclassified" or "living_street" or "service" ? "street" : "other"
    };

    static double Quantile(List<double> v, double q) => v.Count == 0 ? 0 : v[(int)(q * (v.Count - 1))];

    public static double DistSeg(double2 point, double2 vectorA, double2 vectorB)
    {
        var d = vectorB - vectorA;
        var l = math.dot(d, d);
        var t = l > 0 ? math.clamp(math.dot(point - vectorA, d) / l, 0, 1) : 0;

        return math.distance(point, vectorA + t * d);
    }

    public static void Run(string map, string baseDir)
    {
        var capture = new Cap(map);
        var data = capture.LoadOsm();
        var sections = Load(Path.Combine(baseDir, map));
        var lines = new Dictionary<long, double2[]>();

        foreach (var way in data.Ways.Values)
        {
            if (!way.Tags.ContainsKey("highway") && !way.Tags.ContainsKey("railway"))
                continue;

            var points = way.NodeIds.Where(data.Nodes.ContainsKey).Select(id =>
{
    var point = capture.Local(data.Nodes[id].Latitude, data.Nodes[id].Longitude);
    return new double2(point.x, point.z);
}).ToArray();

            if (points.Length >= 2)
                lines[way.Id] = points;
        }

        Console.WriteLine(
            $"{map}: sections {sections.Count}; osm road/rail ways {lines.Count}; classes: {string.Join(", ", sections.GroupBy(section => section.Cls).OrderByDescending(group => group.Count()).Select(group => group.Key + "=" + group.Count()))}");

        var arms = new Dictionary<(string, long), List<(Sec S, bool Start)>>();

        foreach (var section in sections)
        {
            foreach (var (id, st) in new[] { (section.A, true), (section.B, false) })
            {
                if (!arms.TryGetValue((section.Kind, id), out var items))
                    arms[(section.Kind, id)] = items = new();

                items.Add((section, st));
            }
        }

        var failures = new List<(string Cat, int Sec, double3 P, double V, double L, double Sev)>();

        using (var file =
            File.OpenRead(
                Path.Combine(
                    baseDir,
                    map,
                    "visual-failures.ndjson.gz"))) using (var zip =
                        new GZipStream(file, CompressionMode.Decompress)) using (var reader = new StreamReader(zip))
            while (reader.ReadLine() is { } line)
            {
                var row = JObject.Parse(line);
                failures.Add(
                    (
                        (string)row["Category"]!,
                        (int)row["Section"]!,
                        new((double)row["x"]!, (double)row["y"]!, (double)row["z"]!),
                        (double)row["Value"]!,
                        (double)row["Limit"]!,
                        (double)row["Severity"]!));
            }

        var groups =
            new[] {
                "motorway",
                "motorway_link",
                "trunk",
                "trunk_link",
                "primary",
                "secondary",
                "tertiary",
                "street",
                "other"
                };

        Console.WriteLine(
            "class          sections   km   medLen  <15m  <30m <60m | lat p50  p95   p99   max | >10m km | defl2:>2 >5 >15 | thru3:>2 >5 >15 | R<floor R<50 | turn/km src/km | Sflip/km src/km");

        var worstLat = new List<(double D, Sec S, double3 P)>();

        foreach (var text in groups)
        {
            var mine = sections.Where(section => section.Kind == "Road" && Group(section.Cls) == text).ToList();

            if (mine.Count == 0)
                continue;

            var lens = mine.Select(section => section.ArcLength()).OrderBy(v => v).ToList();
            var km = lens.Sum() / 1000;
            var lat = new List<double>();
            var over10 = 0d;
            double turn = 0, flips = 0;

            foreach (var section in mine)
            {
                var ways =
                    section.Ways
                        .Append(section.Way)
                        .Distinct()
                        .Where(lines.ContainsKey)
                        .Select(wayId => lines[wayId])
                        .ToArray();

                var length = section.ArcLength();
                var count = Math.Max(2, (int)(length / 5) + 1);
                var prevSign = 0;
                var worst = 0d;
                var worstAt = default(double3);

                for (var k = 0; k < count; k++)
                {
                    var t = k / (count - 1d);
                    var point = section.At(t);

                    if (ways.Length > 0)
                    {
                        var best = double.MaxValue;

                        foreach (var points in ways)
                            for (var j = 1; j < points.Length; j++)
                            {
                                var d = DistSeg(point.xz, points[j - 1], points[j]);

                                if (d < best)
                                    best = d;
                            }

                        lat.Add(best);

                        if (best > 10)
                            over10 += length / count;

                        if (best > worst)
                        {
                            worst = best;
                            worstAt = point;
                        }
                    }

                    var kappa = section.Curv(t);
                    turn += Math.Abs(kappa) * length / count;
                    var sign = Math.Abs(kappa) < 1 / 3000d ? 0 : Math.Sign(kappa);

                    if (sign != 0 && prevSign != 0 && sign != prevSign)
                        flips++;

                    if (sign != 0)
                        prevSign = sign;
                }

                if (worst > 0)
                    worstLat.Add((worst, section, worstAt));
            }

            lat.Sort();
            // Source turning per km and sign flips measured on the OSM ways this class came from.
            double srcTurn = 0, srcKm = 0, srcFlips = 0;

            foreach (var id in mine
                .SelectMany(section => section.Ways.Append(section.Way))
                .Distinct()
                .Where(lines.ContainsKey))
            {
                var points = lines[id];
                var prevSign = 0;

                for (var j = 1; j < points.Length; j++)
                {
                    srcKm += math.distance(points[j - 1], points[j]) / 1000;

                    if (j + 1 >= points.Length)
                        continue;

                    var vectorA = math.normalizesafe(points[j] - points[j - 1]);
                    var vectorB = math.normalizesafe(points[j + 1] - points[j]);
                    var ang = Math.Atan2(vectorA.x * vectorB.y - vectorA.y * vectorB.x, math.dot(vectorA, vectorB));
                    srcTurn += Math.Abs(ang);
                    var sign = Math.Abs(ang) < math.radians(.5) ? 0 : Math.Sign(ang);

                    if (sign != 0 && prevSign != 0 && sign != prevSign)
                        srcFlips++;

                    if (sign != 0)
                        prevSign = sign;
                }
            }

            var d2 = new List<double>();
            var thru = new List<double>();

            foreach (var kv in arms)
            {
                var items = kv.Value;

                if (!items.Any(item => item.S.Kind == "Road" && Group(item.S.Cls) == text))
                    continue;

                if (items.Count == 2)
                    d2.Add(Deflect(Away(items[0]), Away(items[1])));
                else if (items.Count >= 3)
                {
                    var same = items.Where(item => Group(item.S.Cls) == text).ToList();

                    if (same.Count < 2)
                        continue;

                    var bestD = 180d;

                    for (var a = 0; a < same.Count; a++)
                        for (var b = a + 1; b < same.Count; b++)
                            bestD = Math.Min(bestD, Deflect(Away(same[a]), Away(same[b])));

                    thru.Add(bestD);
                }
            }

            var floor = mine.Count(section =>
{
    var floorOf = FloorOf(text);
    var worst = 0d;

    for (var k = 0; k <= 16; k++)
        worst = Math.Max(worst, Math.Abs(section.Curv(k / 16d)));

    return worst * floorOf > 1;
});

            var r50 = mine.Count(section =>
{
    var worst = 0d;

    for (var k = 0; k <= 16; k++)
        worst = Math.Max(worst, Math.Abs(section.Curv(k / 16d)));

    return worst * 50 > 1;
});

            Console.WriteLine(
                $"{text,-14} {mine.Count,7} {km,6:F0} {Quantile(lens, .5),6:F0} {lens.Count(v => v < 15),5} {lens.Count(v => v < 30),5} {lens.Count(v => v < 60),5} | {Quantile(lat, .5),5:F1} {Quantile(lat, .95),5:F1} {Quantile(lat, .99),5:F1} {(lat.Count > 0 ? lat[^1] : 0),6:F1} | {over10 / 1000,6:F1} | {d2.Count(v => v > 2),5} {d2.Count(v => v > 5),4} {d2.Count(v => v > 15),4} | {thru.Count(v => v > 2),5} {thru.Count(v => v > 5),4} {thru.Count(v => v > 15),4} | {floor,6} {r50,5} | {math.degrees(turn) / km,6:F0} {math.degrees(srcTurn) / Math.Max(.001, srcKm),6:F0} | {flips / km,5:F1} {srcFlips / Math.Max(.001, srcKm),5:F1}");
        }

        Console.WriteLine();
        var cats = failures.Select(item => item.Cat).Distinct().OrderBy(text => text).ToArray();
        Console.WriteLine("failures      " + string.Join(" ", cats.Select(text => text.PadLeft(12))));

        foreach (var text in groups)
        {
            var row =
                cats.Select(
                    otherText => failures
                        .Count(
                            item => item.Cat == otherText
                                && sections[item.Sec].Kind == "Road"
                                && Group(sections[item.Sec].Cls) == text)
                        .ToString()
                        .PadLeft(12));

            Console.WriteLine($"{text,-13} " + string.Join(" ", row));
        }

        Console.WriteLine();

        foreach (var text in new[] { "motorway", "trunk", "motorway_link", "trunk_link" })
        {
            Console.WriteLine($"-- worst {text} lateral:");

            foreach (var item in worstLat
                .Where(otherItem => Group(otherItem.S.Cls) == text)
                .OrderByDescending(otherItem => otherItem.D)
                .Take(8))
            {
                var otherItem = capture.ToGeographic(item.P.x, item.P.z);
                Console.WriteLine(
                    $"   {item.D,6:F1} m way {item.S.Way} ({otherItem.lat:F5},{otherItem.lon:F5}) len {item.S.ArcLength():F0} bridge={item.S.Bridge}");
            }

            foreach (var cat in new[] {
                "tangent-break",
                "tight-radius",
                "same-level-overlap",
                "grade-change",
                "crossing-clearance"
                })
            {
                Console.WriteLine($"-- worst {text} {cat}:");

                foreach (var item in failures
                    .Where(
                        otherItem => otherItem.Cat == cat
                            && sections[otherItem.Sec].Kind == "Road"
                            && Group(sections[otherItem.Sec].Cls) == text)
                    .OrderByDescending(otherItem => otherItem.Sev)
                    .Take(6))
                {
                    var otherItem = capture.ToGeographic(item.P.x, item.P.z);
                    Console.WriteLine(
                        $"   sev {item.Sev,6:F1} value {item.V,7:F2} limit {item.L,6:F2} way {sections[item.Sec].Way} ({otherItem.lat:F5},{otherItem.lon:F5})");
                }
            }
        }
    }

    public static double FloorOf(string text) => text switch
    {
        "motorway" => 250,
        "trunk" => 150,
        "primary" => 60,
        "secondary" => 40,
        "tertiary" => 25,
        "motorway_link" or "trunk_link" => 50,
        _ => 12
    };

    internal static double Deflect(double2 point, double2 otherPoint)
        => math.degrees(Math.Acos(math.clamp(-math.dot(point, otherPoint), -1, 1)));

    internal static double2 Away((Sec S, bool Start) firstItem)
    {
        var d = firstItem.Start ? firstItem.S.P1 - firstItem.S.P0 : firstItem.S.P2 - firstItem.S.P3;

        if (math.lengthsq(d.xz) < 1e-6)
            d = firstItem.Start ? firstItem.S.P2 - firstItem.S.P0 : firstItem.S.P1 - firstItem.S.P3;

        return math.normalizesafe(d.xz);
    }

    internal static double TotalLength(List<Sec> sections) => sections.Sum(section => section.ArcLength());
}
