using Newtonsoft.Json.Linq;
using Unity.Mathematics;
using System.Text.RegularExpressions;

public sealed record Edge(
    string Entity,
    string A,
    string B,
    string Kind,
    string Prefab,
    double Width,
    string Flags,
    double3 P0,
    double3 P1,
    double3 P2,
    double3 P3,
    double Len,
    double Grade,
    double Curv)
{
    public double3 At(double t)
    {
        var u = 1 - t;
        return u * u * u * P0 + 3 * u * u * t * P1 + 3 * u * t * t * P2 + t * t * t * P3;
    }
}
public static class Native
{
    public static List<Edge> Census(Cap capture)
    {
        var list = new List<Edge>();
        var seen = new HashSet<string>();

        foreach (var prop in capture.Files.Properties())
        {
            if (!Regex.IsMatch(prop.Name, @"^native-owned-final-\d+\.ndjson\.gz$"))
                continue;

            foreach (var text in capture.Lines(Path.Combine(capture.Folder, (string)prop.Value!)))
            {
                var row = JObject.Parse(text);

                if ((string?)row["kind"] == null || row["a"] == null)
                    continue;

                var id = (string)row["entity"]!;

                if (!seen.Add(id))
                    continue;

                var token = row["curveQuality"]!;
                list.Add(
                    new(
                        id,
                        (string)row["startNode"]!,
                        (string)row["endNode"]!,
                        (string)row["kind"]!,
                        (string)row["prefab"]!,
                        (double?)row["width"] ?? 0,
                        (string?)row["general"] ?? "",
                        Live.ReadVector(row["a"]!),
                        Live.ReadVector(row["b"]!),
                        Live.ReadVector(row["c"]!),
                        Live.ReadVector(row["d"]!),
                        (double?)token["SampledLengthMeters"] ?? 0,
                        (double?)token["SampledMaximumGrade"] ?? 0,
                        (double?)token["SampledMaximumCurvature"] ?? 0));
            }
        }

        return list;
    }

    public static void Run(string map, string baseDir)
    {
        var capture = new Cap(map);
        var edges = Census(capture);
        Console.WriteLine($"{map}: census edges {edges.Count}");

        if (edges.Count == 0)
            return;

        // Planned sections as the game received them.
        var plan = new List<Sec>();

        foreach (var row in capture.ManifestJ("final-prepared-network-chunks.json"))
        {
            if (plan.Count == 0)
                Console.WriteLine(
                    "final record keys: "
                        + string.Join(",", row.Properties().Select(property => property.Name)));

            double3 ParsePoint(string text)
            {
                var token = row[text]!;
                return token is JArray array
                    ? new((double)array[0], (double)array[1], (double)array[2])
                    : new((double)token["x"]!, (double)token["y"]!, (double)token["z"]!);
            }

            try
            {
                var kind = (string?)row["kind"] ?? (string?)row["Kind"] ?? "";
                plan.Add(
                    new(
                        plan.Count,
                        (long)row["WayId"]!,
                        (long)row["StartNodeId"]!,
                        (long)row["EndNodeId"]!,
                        (int?)row["Layer"] ?? 0,
                        Array.Empty<long>(),
                        kind,
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
            catch (Exception exception)
            {
                if (plan.Count == 0)
                    Console.WriteLine("parse " + exception.Message + " " + row.ToString().Substring(0, 600));

                throw;
            }
        }

        Console.WriteLine($"planned {plan.Count}");
        // Spatial index of planned end nodes and centre samples.
        var nodes = new Dictionary<(int, int), List<(double2 P, string Cls)>>();

        (int, int) Cell(double2 point, double s) => ((int)Math.Floor(point.x / s), (int)Math.Floor(point.y / s));

        foreach (var section in plan)
            foreach (var e in new[] { section.P0, section.P3 })
            {
                var item = Cell(e.xz, 4);

                if (!nodes.TryGetValue(item, out var items))
                    nodes[item] = items = new();

                items.Add((e.xz, section.Cls));
            }

        double NearestNode(double2 point)
        {
            var (cx, cz) = Cell(point, 4);
            var best = double.MaxValue;

            for (var dz = -1; dz <= 1; dz++)
                for (var dx = -1; dx <= 1; dx++)
                    if (nodes.TryGetValue((cx + dx, cz + dz), out var items))
                        foreach (var item in items)
                        {
                            var otherDistance = math.distance(item.P, point);

                            if (otherDistance < best)
                                best = otherDistance;
                        }

            return best;
        }

        var samples = new Dictionary<(int, int), List<(double2 P, int Sec)>>();

        foreach (var section in plan)
        {
            var n = Math.Max(2, (int)(section.ArcLength() / 6) + 1);

            for (var k = 0; k <= n; k++)
            {
                var point = section.At(k / (double)n).xz;
                var key = Cell(point, 16);

                if (!samples.TryGetValue(key, out var items))
                    samples[key] = items = new();

                items.Add((point, section.Index));
            }
        }

        (int Sec, double D) NearestSec(double2 point, string kind)
        {
            var (cx, cz) = Cell(point, 16);
            var best = -1;
            var bd = double.MaxValue;

            for (var dz = -2; dz <= 2; dz++)
                for (var dx = -2; dx <= 2; dx++)
                    if (samples.TryGetValue((cx + dx, cz + dz), out var items))
                        foreach (var item in items)
                        {
                            if (!string.Equals(plan[item.Sec].Kind, kind, StringComparison.OrdinalIgnoreCase))
                                continue;

                            var otherDistance = math.distance(item.P, point);

                            if (otherDistance < bd)
                            {
                                bd = otherDistance;
                                best = item.Sec;
                            }
                        }

            return (best, bd);
        }

        var classification = new string[edges.Count];
        var lateral = new double[edges.Count];

        for (var i = 0; i < edges.Count; i++)
        {
            var edge = edges[i];
            var worst = 0d;
            var section = -1;

            foreach (var t in new[] { .25, .5, .75 })
            {
                var item = NearestSec(edge.At(t).xz, edge.Kind);

                if (item.Sec >= 0 && item.D < 500)
                {
                    if (t == .5)
                        section = item.Sec;

                    worst = Math.Max(worst, item.D);
                }
            }

            classification[i] = section >= 0 ? Hw.Group(plan[section].Cls) : "?";
            lateral[i] = worst;
        }

        var byNode = new Dictionary<string, List<int>>();

        for (var i = 0; i < edges.Count; i++)
            foreach (var id in new[] { edges[i].A, edges[i].B })
            {
                if (!byNode.TryGetValue(id, out var l))
                    byNode[id] = l = new();

                l.Add(i);
            }

        double2 Away(int i, string id)
        {
            var edge = edges[i];
            var d = edge.A == id ? edge.P1 - edge.P0 : edge.P2 - edge.P3;

            if (math.lengthsq(d.xz) < 1e-6)
                d = edge.A == id ? edge.P2 - edge.P0 : edge.P1 - edge.P3;

            return math.normalizesafe(d.xz);
        }

        Console.WriteLine(
            "class          edges    km  planned | split-ends  | lateral to plan p95 max | R<floor R<50 | maxGrade>6% >10% | deg2 defl>2 >5 >15 | width values");

        foreach (var text in new[] {
            "motorway",
            "motorway_link",
            "trunk",
            "trunk_link",
            "primary",
            "secondary",
            "tertiary",
            "?"
            })
        {
            var index =
                Enumerable
                    .Range(0, edges.Count)
                    .Where(
                        i => classification[i] == text
                            && edges[i].Kind.Equals("Road", StringComparison.OrdinalIgnoreCase))
                    .ToArray();

            if (index.Length == 0)
                continue;

            var split =
                index.Sum(i => (NearestNode(edges[i].P0.xz) > 1 ? 1 : 0) + (NearestNode(edges[i].P3.xz) > 1 ? 1 : 0));

            var lat = index.Select(i => lateral[i]).OrderBy(v => v).ToArray();
            var floor = Hw.FloorOf(text);
            var d2 = new List<double>();

            foreach (var kv in byNode)
            {
                if (kv.Value.Count != 2 || !kv.Value.All(i => classification[i] == text))
                    continue;

                var vectorA = Away(kv.Value[0], kv.Key);
                var vectorB = Away(kv.Value[1], kv.Key);
                d2.Add(math.degrees(Math.Acos(math.clamp(-math.dot(vectorA, vectorB), -1, 1))));
            }

            Console.WriteLine(
                $"{text,-14} {index.Length,6} {index.Sum(i => edges[i].Len) / 1000,6:F0} {plan.Count(section => section.Kind.Equals("Road", StringComparison.OrdinalIgnoreCase) && Hw.Group(section.Cls) == text),6} | {split,8}    | {lat[(int)(.95 * (lat.Length - 1))],8:F2} {lat[^1],7:F1} | {index.Count(i => edges[i].Curv * floor > 1),6} {index.Count(i => edges[i].Curv * 50 > 1),5} | {index.Count(i => edges[i].Grade > .06),6} {index.Count(i => edges[i].Grade > .1),5} | {d2.Count(v => v > 2),6} {d2.Count(v => v > 5),4} {d2.Count(v => v > 15),4} | {string.Join(",", index.GroupBy(i => edges[i].Width).OrderByDescending(group => group.Count()).Take(6).Select(group => $"{group.Key:F0}x{group.Count()}"))}");
        }

        // Node heights: native edge ends against the plan's node heights (nearest in 3D among nodes within 1.5 m in plan).
        {
            var nodes3 = new Dictionary<(int, int), List<(double3 P, string Cls)>>();

            foreach (var section in plan)
                foreach (var e in new[] { section.P0, section.P3 })
                {
                    var item = Cell(e.xz, 4);

                    if (!nodes3.TryGetValue(item, out var items))
                        nodes3[item] = items = new();

                    items.Add((e, section.Cls));
                }

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
                var dy = new List<double>();

                for (var i = 0; i < edges.Count; i++)
                {
                    if (classification[i] != text)
                        continue;

                    foreach (var e in new[] { edges[i].P0, edges[i].P3 })
                    {
                        var (cx, cz) = Cell(e.xz, 4);
                        var best = double.MaxValue;

                        for (var dz = -1; dz <= 1; dz++)
                            for (var dx = -1; dx <= 1; dx++)
                                if (nodes3.TryGetValue((cx + dx, cz + dz), out var items))
                                    foreach (var item in items)
                                        if (math.distance(item.P.xz, e.xz) < 1.5)
                                            best = Math.Min(best, Math.Abs(item.P.y - e.y));

                        if (best < double.MaxValue)
                            dy.Add(best);
                    }
                }

                dy.Sort();

                if (dy.Count > 0)
                    Console.WriteLine(
                        $"node dy {text,-14} n={dy.Count,6} p50 {dy[dy.Count / 2],5:F3} p95 {dy[(int)(.95 * (dy.Count - 1))],5:F3} p99 {dy[(int)(.99 * (dy.Count - 1))],5:F3} max {dy[^1],6:F2}; over 0.3 m {dy.Count(v => v > .3)}");
            }

            // Edge length distribution: does the game cut long sections?
            foreach (var text in new[] { "motorway", "trunk" })
            {
                var length =
                    Enumerable
                        .Range(0, edges.Count)
                        .Where(i => classification[i] == text)
                        .Select(i => edges[i].Len)
                        .OrderBy(v => v)
                        .ToArray();

                var pl =
                    plan
                        .Where(section => Hw.Group(section.Cls) == text)
                        .Select(section => section.ArcLength())
                        .OrderBy(v => v)
                        .ToArray();

                Console.WriteLine(
                    $"length {text}: native p50 {length[length.Length / 2]:F0} p95 {length[(int)(.95 * length.Length)]:F0} max {length[^1]:F0}; plan p50 {pl[pl.Length / 2]:F0} p95 {pl[(int)(.95 * pl.Length)]:F0} max {pl[^1]:F0}");
            }
        }

        // Height: native centreline against the plan at the same place (projected to the matched section).
        Console.WriteLine(
            "class          n  |dy| p50 p95 p99 max | native grade>limit planned grade>limit (same class, sampled) ");

        double GradeLimit(string text) => text switch
        {
            "motorway" => .06,
            "trunk" => .07,
            "motorway_link" or "trunk_link" => .08,
            _ => .1
        };

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
            var dy = new List<double>();
            var worst = new List<(double D, double3 P)>();

            for (var i = 0; i < edges.Count; i++)
            {
                if (classification[i] != text || !edges[i].Kind.Equals("Road", StringComparison.OrdinalIgnoreCase))
                    continue;

                foreach (var t in new[] { .0, .25, .5, .75, 1.0 })
                {
                    var point = edges[i].At(t);
                    var near = NearestSec(point.xz, "Road");

                    if (near.Sec < 0 || near.D > 6)
                        continue;

                    var section = plan[near.Sec];
                    // project on the matched planned section by dense search
                    var best = double.MaxValue;
                    var by = 0d;

                    for (var k = 0; k <= 32; k++)
                    {
                        var u = section.At(k / 32d);
                        var d = math.distancesq(u.xz, point.xz);

                        if (d < best)
                        {
                            best = d;
                            by = u.y;
                        }
                    }

                    var e = Math.Abs(point.y - by);
                    dy.Add(e);
                    worst.Add((e, point));
                }
            }

            dy.Sort();

            double ProfileGrade(Sec section)
            {
                var w = 0d;

                for (var k = 0; k <= 16; k++)
                {
                    var d = section.Derivative(k / 16d);
                    var otherLength = math.length(d.xz);

                    if (otherLength > 1e-6)
                        w = Math.Max(w, Math.Abs(d.y) / otherLength);
                }

                return w;
            }

            var lim = GradeLimit(text);
            var nat = Enumerable.Range(0, edges.Count).Count(i => classification[i] == text && edges[i].Grade > lim);
            var pl =
                plan.Count(
                    section => section.Kind.Equals("Road", StringComparison.OrdinalIgnoreCase)
                        && Hw.Group(section.Cls) == text
                        && ProfileGrade(section) > lim);

            if (dy.Count > 0)
                Console.WriteLine(
                    $"{text,-14} {dy.Count,6} {dy[dy.Count / 2],6:F2} {dy[(int)(.95 * (dy.Count - 1))],6:F2} {dy[(int)(.99 * (dy.Count - 1))],6:F2} {dy[^1],6:F2} | {nat,6} {pl,6} (limit {lim:P0})");

            if (text is "motorway" or "trunk")
                foreach (var item in worst.OrderByDescending(otherItem => otherItem.D).Take(6))
                {
                    var otherItem = capture.ToGeographic(item.P.x, item.P.z);
                    Console.WriteLine($"     {item.D,6:F2} m at {otherItem.lat:F5},{otherItem.lon:F5} y={item.P.y:F1}");
                }
        }

        // Mixed nodes: highway mainline edge sharing a native node with a non-highway, non-link edge.
        var mixed = new List<(string Id, double3 P, string Classes)>();

        foreach (var kv in byNode)
        {
            var set = kv.Value.Select(i => classification[i]).Distinct().ToArray();

            if (set.Any(text => text is "motorway") && set.Any(text => text is "primary" or "secondary" or "tertiary"))
            {
                var i = kv.Value[0];
                var edge = edges[i];
                mixed.Add(
                    (
                        kv.Key,
                        edge.A == kv.Key ? edge.P0 : edge.P3,
                        string.Join("+", kv.Value.Select(j => classification[j]))));
            }
        }

        Console.WriteLine($"native nodes joining a motorway to a street class: {mixed.Count}");

        foreach (var item in mixed.Take(12))
        {
            var otherItem = capture.ToGeographic(item.P.x, item.P.z);
            Console.WriteLine($"   {otherItem.lat:F5},{otherItem.lon:F5} {item.Classes}");
        }

        // Native split ends on highways: where the game cut an edge the plan did not.
        var splits = new List<(double2 P, string Cls)>();

        for (var i = 0; i < edges.Count; i++)
            if (classification[i] is "motorway" or "trunk")
                foreach (var e in new[] { edges[i].P0, edges[i].P3 })
                    if (NearestNode(e.xz) > 1)
                        splits.Add((e.xz, classification[i]));

        Console.WriteLine($"highway edge ends away from any planned node: {splits.Count}");

        foreach (var item in splits.Take(15))
        {
            var otherItem = capture.ToGeographic(item.P.x, item.P.y);
            Console.WriteLine(
                $"   {otherItem.lat:F5},{otherItem.lon:F5} {item.Cls} nearest planned node {NearestNode(item.P):F1} m");
        }

        var worstLat =
            Enumerable
                .Range(0, edges.Count)
                .Where(i => classification[i] is "motorway" or "trunk" or "motorway_link" or "trunk_link")
                .OrderByDescending(i => lateral[i])
                .Take(12);

        Console.WriteLine("largest native-vs-plan distance on highways:");

        foreach (var i in worstLat)
        {
            var point = edges[i].At(.5);
            var item = capture.ToGeographic(point.x, point.z);
            Console.WriteLine(
                $"   {lateral[i],6:F1} m {classification[i]} {item.lat:F5},{item.lon:F5} len {edges[i].Len:F0} width {edges[i].Width:F0}");
        }
    }
}
