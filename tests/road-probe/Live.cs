using Newtonsoft.Json.Linq;
using System.Text.RegularExpressions;
using Unity.Mathematics;

// What the game actually built, read from a capture's live edges and observed terrain.
public sealed record LiveEdge(
    string Entity,
    string A,
    string B,
    string Kind,
    double Width,
    string General,
    string Left,
    string Right,
    double3 P0,
    double3 P1,
    double3 P2,
    double3 P3,
    double Len,
    double Grade)
{
    public double3 At(double t)
    {
        var u = 1 - t;
        return u * u * u * P0 + 3 * u * u * t * P1 + 3 * u * t * t * P2 + t * t * t * P3;
    }

    public string Flags => Regex.Replace($"{General}|{Left}|{Right}", @"\s", "");

    public bool Has(string flag) => General.Split(", ").Contains(flag);
}

public static class Live
{
    public static List<LiveEdge> Edges(Cap capture)
    {
        var list = new List<LiveEdge>();
        var seen = new HashSet<string>();

        foreach (var prop in capture.Files.Properties())
        {
            if (!Regex.IsMatch(prop.Name, @"^native-owned-final-\d+\.ndjson\.gz$"))
                continue;

            foreach (var text in capture.Lines(Path.Combine(capture.Folder, (string)prop.Value!)))
            {
                if (!text.Contains("\"startNode\""))
                    continue;

                var row = JObject.Parse(text);

                if (row["a"] == null)
                    continue;

                var id = (string)row["entity"]!;

                if (!seen.Add(id))
                    continue;

                var token = row["curveQuality"];
                list.Add(
                    new(
                        id,
                        (string)row["startNode"]!,
                        (string)row["endNode"]!,
                        (string)row["kind"]!,
                        (double?)row["width"] ?? 0,
                        (string?)row["general"] ?? "",
                        (string?)row["left"] ?? "",
                        (string?)row["right"] ?? "",
                        ReadVector(row["a"]!),
                        ReadVector(row["b"]!),
                        ReadVector(row["c"]!),
                        ReadVector(row["d"]!),
                        (double?)token?["SampledLengthMeters"] ?? 0,
                        (double?)token?["SampledMaximumGrade"] ?? 0));
            }
        }

        return list;
    }

    // Composition flags the game gave the live road edges, with length and depth below the terrain.
    public static void Flags(string map)
    {
        var capture = new Cap(map);
        var t = capture.Heights("native-observed-heightmap.u16");
        var edges = Edges(capture).Where(edge => edge.Kind == "Road").ToList();
        Console.WriteLine($"{map}: live road edges {edges.Count}");

        foreach (var group in edges
            .GroupBy(edge => edge.Flags)
            .OrderByDescending(otherGroup => otherGroup.Sum(edge => edge.Len))
            .Take(40))
        {
            var cover = group.SelectMany(edge => new[] { .25, .5, .75 }.Select(u =>
{
    var point = edge.At(u);
    return (double)capture.HeightAt(t, point.x, point.z) - point.y;
})).OrderBy(v => v).ToList();

            Console.WriteLine(
                $"  {group.Count(),6} {group.Sum(edge => edge.Len) / 1000,7:F1} km  ground-deck p10 {cover[cover.Count / 10],6:F1} p50 {cover[cover.Count / 2],6:F1} p90 {cover[cover.Count * 9 / 10],6:F1}  {group.Key}");
        }
    }

    // Live edges around a point, nearest first.
    public static void Site(string map, double lat, double lon, double radius)
    {
        var capture = new Cap(map);
        var otherPoint = capture.Local(lat, lon);
        var t = capture.Heights("native-observed-heightmap.u16");
        var near =
            Edges(capture)
                .Where(
                    edge => edge.Kind == "Road"
                        && math.distance(edge.At(.5).xz, new double2(otherPoint.x, otherPoint.z)) < radius)
                .OrderBy(edge => math.distance(edge.At(.5).xz, new double2(otherPoint.x, otherPoint.z)))
                .ToList();

        Console.WriteLine($"{map} {lat},{lon} r={radius}: {near.Count} live road edges");

        foreach (var edge in near)
        {
            var m = edge.At(.5);
            var item = capture.ToGeographic(m.x, m.z);
            Console.WriteLine(
                $"  {item.lat:F5},{item.lon:F5} L{edge.Len,5:F0} w{edge.Width,3:F0} y {edge.P0.y,6:F1}>{edge.P3.y,6:F1} grade {edge.Grade,4:P0} ground-deck {capture.HeightAt(t, edge.P0.x, edge.P0.z) - edge.P0.y,6:F1} {capture.HeightAt(t, m.x, m.z) - m.y,6:F1} {capture.HeightAt(t, edge.P3.x, edge.P3.z) - edge.P3.y,6:F1}  {edge.Flags}");
        }
    }

    // Open cuts: edges with Lowered walls, by what the plan meant and how deep they run.
    public static void Cuts(string map)
    {
        var capture = new Cap(map);
        var t = capture.Heights("native-observed-heightmap.u16");
        var data = capture.LoadOsm();
        var edges = Edges(capture).Where(edge => edge.Kind == "Road").ToList();
        var plan = Compare.Final(capture).Where(section => section.Kind == "Road").ToList();
        var cells = new Dictionary<(int, int), List<(double2 P, int I)>>();

        for (var i = 0; i < plan.Count; i++)
        {
            var n = Math.Max(2, (int)(plan[i].ArcLength() / 8) + 1);

            for (var k = 0; k <= n; k++)
            {
                var point = plan[i].At(k / (double)n).xz;
                var key = ((int)Math.Floor(point.x / 16), (int)Math.Floor(point.y / 16));

                if (!cells.TryGetValue(key, out var items))
                    cells[key] = items = new();

                items.Add((point, i));
            }
        }

        Sec? Match(double3 point)
        {
            var cx = (int)Math.Floor(point.x / 16);
            var cz = (int)Math.Floor(point.z / 16);
            var best = double.MaxValue;
            Sec? hit = null;

            for (var dz = -1; dz <= 1; dz++)
                for (var dx = -1; dx <= 1; dx++)
                    if (cells.TryGetValue((cx + dx, cz + dz), out var items))
                        foreach (var item in items)
                        {
                            var otherDistance = math.distance(item.P, point.xz);
                            var h = Math.Abs(plan[item.I].At(.5).y - point.y);

                            if (otherDistance + h * .2 < best)
                            {
                                best = otherDistance + h * .2;
                                hit = plan[item.I];
                            }
                        }

            return hit;
        }

        bool Tagged(long way)
            => data.Ways.TryGetValue(way, out var otherWay)
                && (otherWay.Tags.TryGetValue("tunnel", out var text) && text != "no"
                    || otherWay.Tags.TryGetValue("covered", out var otherText) && otherText == "yes");

        double Cover(double3 point) => capture.HeightAt(t, point.x, point.z) - point.y;

        var cuts = edges.Where(edge => edge.Left.Contains("Lowered") && edge.Right.Contains("Lowered")).ToList();
        var rows = new Dictionary<string, (int N, double Km)>();

        foreach (var edge in cuts)
        {
            var section = Match(edge.At(.5));
            var cover = Cover(edge.P0);
            var otherCover = Cover(edge.P3);
            var deep = Math.Max(cover, otherCover);
            var shallow = Math.Min(cover, otherCover);
            var plan2 =
                section == null
                    ? "unmatched"
                    : section.Tunnel
                        ? (section.Ways.Append(section.Way).Any(Tagged)
                            ? "plan tunnel, OSM tunnel"
                            : "plan tunnel, no OSM tag")
                        : (section.Ways.Append(section.Way).Any(Tagged) ? "plan open, OSM tunnel" : "plan open");

            var key =
                $"{plan2,-26} deep end {(deep < 3 ? "<3" : deep < 6 ? "3-6" : deep < 10 ? "6-10" : deep < 14 ? "10-14" : "14+"),-6} shallow end {(shallow < 1 ? "<1" : shallow < 3 ? "1-3" : shallow < 6 ? "3-6" : "6+")}";

            rows.TryGetValue(key, out var item);
            rows[key] = (item.N + 1, item.Km + edge.Len / 1000);
        }

        Console.WriteLine(
            $"{map}: edges Lowered both sides {cuts.Count}, {cuts.Sum(edge => edge.Len) / 1000:F1} km; length p50 {cuts.Select(edge => edge.Len).OrderBy(v => v).ElementAt(cuts.Count / 2):F0} max {cuts.Max(edge => edge.Len):F0}");

        foreach (var entry in rows.OrderBy(otherEntry => otherEntry.Key))
            Console.WriteLine($"  {entry.Value.N,4} {entry.Value.Km,6:F2} km {entry.Key}");

        // Runs of connected cut edges, longest and deepest first.
        var parent = Enumerable.Range(0, cuts.Count).ToArray();

        int Find(int i)
        {
            while (parent[i] != i)
                i = parent[i] = parent[parent[i]];

            return i;
        }

        var at = new Dictionary<string, int>();

        for (var i = 0; i < cuts.Count; i++)
            foreach (var id in new[] { cuts[i].A, cuts[i].B })
            {
                if (at.TryGetValue(id, out var j))
                    parent[Find(i)] = Find(j);
                else
                    at[id] = i;
            }

        var runs =
            Enumerable
                .Range(0, cuts.Count)
                .GroupBy(Find)
                .Select(group => group.Select(i => cuts[i]).ToList())
                .OrderByDescending(otherEdges => otherEdges.Sum(edge => edge.Len))
                .ToList();

        var lens = runs.Select(otherEdges => otherEdges.Sum(edge => edge.Len)).OrderBy(v => v).ToList();
        Console.WriteLine(
            $"runs {runs.Count}: length p50 {lens[lens.Count / 2]:F0} p90 {lens[lens.Count * 9 / 10]:F0} max {lens[^1]:F0}");

        foreach (var otherEdges in runs.Take(24))
        {
            var item = capture.ToGeographic(otherEdges[0].P0.x, otherEdges[0].P0.z);
            var deep = otherEdges.Max(edge => Math.Max(Cover(edge.P0), Math.Max(Cover(edge.At(.5)), Cover(edge.P3))));
            Console.WriteLine(
                $"  {otherEdges.Sum(edge => edge.Len),6:F0} m {otherEdges.Count,3} edges deepest {deep,5:F1} m at {item.lat:F5},{item.lon:F5}");
        }

        // Node depths at the mouth: where a tunnel edge meets something that is not a tunnel.
        var byNode = new Dictionary<string, List<LiveEdge>>();

        foreach (var edge in edges)
            foreach (var id in new[] { edge.A, edge.B })
            {
                if (!byNode.TryGetValue(id, out var otherEdges))
                    byNode[id] = otherEdges = new();

                otherEdges.Add(edge);
            }

        var mouths = new List<double>();

        foreach (var kv in byNode)
        {
            if (!kv.Value.Any(edge => edge.Has("Tunnel")) || kv.Value.All(edge => edge.Has("Tunnel")))
                continue;

            var otherEdge = kv.Value.First(edge => edge.Has("Tunnel"));
            mouths.Add(Cover(otherEdge.A == kv.Key ? otherEdge.P0 : otherEdge.P3));
        }

        mouths.Sort();

        if (mouths.Count > 0)
            Console.WriteLine(
                $"tunnel mouths {mouths.Count}: ground over deck p10 {mouths[mouths.Count / 10]:F1} p50 {mouths[mouths.Count / 2]:F1} p90 {mouths[mouths.Count * 9 / 10]:F1}");
    }

    // One-way main roads with an opposing partner beside them: runs that read as one
    // road drawn twice. Sums by class and run length, longest few sites, and the
    // junctions they crowd.
    public static void Splits(string map)
    {
        var capture = new Cap(map);
        var direction = Environment.GetEnvironmentVariable("PLAN_DIR");
        var plan =
            (string.IsNullOrEmpty(direction) ? Compare.Final(capture) : Hw.Load(Path.Combine(direction, map)))
                .Where(section => section.Kind == "Road")
                .ToList();

        var data = capture.LoadOsm();

        bool Round(Sec section)
            => section.Ways
                .Append(section.Way)
                .Any(
                    wayId => data.Ways.TryGetValue(wayId, out var otherWay)
                        && otherWay.Tags.TryGetValue("junction", out var text)
                        && text is "roundabout" or "circular");

        var mains =
            plan
                .Where(
                    section => section.OneWay
                        && Hw.Group(section.Cls) is "primary" or "secondary" or "tertiary"
                        && !section.Bridge
                        && !section.Tunnel
                        && !Round(section))
                .ToList();

        var cells = new Dictionary<(int, int), List<int>>();

        for (var i = 0; i < mains.Count; i++)
            for (var k = 0; k <= 4; k++)
            {
                var point = mains[i].At(k / 4d).xz;
                var key = ((int)Math.Floor(point.x / 48), (int)Math.Floor(point.y / 48));

                if (!cells.TryGetValue(key, out var l))
                    cells[key] = l = new();

                if (l.Count == 0 || l[^1] != i)
                    l.Add(i);
            }

        bool Paired(int i)
        {
            var section = mains[i];
            var m = section.At(.5);
            var d = math.normalizesafe(section.At(.6).xz - section.At(.4).xz);
            var cx = (int)Math.Floor(m.x / 48);
            var cz = (int)Math.Floor(m.z / 48);

            for (var dz = -1; dz <= 1; dz++)
                for (var dx = -1; dx <= 1; dx++)
                    if (cells.TryGetValue((cx + dx, cz + dz), out var l))
                        foreach (var j in l)
                        {
                            if (j == i)
                                continue;

                            var otherSection = mains[j];

                            for (var k = 1; k < 8; k++)
                            {
                                var point = otherSection.At(k / 8d);

                                if (math.distance(point.xz, m.xz) > 40)
                                    continue;

                                var e =
                                    math.normalizesafe(
                                        otherSection.At(k / 8d + .05).xz - otherSection.At(k / 8d - .05).xz);

                                if (math.dot(d, e) < -.85)
                                    return true;
                            }
                        }

            return false;
        }

        var paired = Enumerable.Range(0, mains.Count).Where(Paired).ToArray();
        var pairedSet = new HashSet<int>(paired);
        // Runs of connected paired sections.
        var parent = Enumerable.Range(0, mains.Count).ToArray();

        int Find(int i)
        {
            while (parent[i] != i)
                i = parent[i] = parent[parent[i]];

            return i;
        }

        var at = new Dictionary<long, int>();

        foreach (var i in paired)
            foreach (var id in new[] { mains[i].A, mains[i].B })
            {
                if (at.TryGetValue(id, out var j))
                    parent[Find(i)] = Find(j);
                else
                    at[id] = i;
            }

        var runs = paired.GroupBy(Find).Select(group => group.Select(i => mains[i]).ToList()).ToList();

        Console.WriteLine(
            $"{map}: one-way main sections {mains.Count} ({mains.Sum(section => section.ArcLength()) / 1000:F0} km), paired {paired.Length} ({paired.Sum(i => mains[i].ArcLength()) / 1000:F0} km) in {runs.Count} runs");

        foreach (var (low, high) in new[] {
            (0d, 60d),
            (60d, 120d),
            (120d, 250d),
            (250d, 500d),
            (500d, 1000d),
            (1000d, 1e9)
            })
        {
            var sections =
                runs
                    .Where(
                        otherSections => Hw.TotalLength(otherSections) >= low && Hw.TotalLength(otherSections) < high)
                    .ToList();

            Console.WriteLine(
                $"  run {low,5:F0}-{(high > 1e8 ? "up" : high.ToString("F0")),-5} m: {sections.Count,5} runs {sections.Sum(Hw.TotalLength) / 1000,6:F1} km");
        }

        foreach (var sections in runs.OrderByDescending(Hw.TotalLength).Take(6))
        {
            var item = capture.ToGeographic(sections[0].P0.x, sections[0].P0.z);
            Console.WriteLine(
                $"  {Hw.TotalLength(sections),6:F0} m {Hw.Group(sections[0].Cls),-10} {item.lat:F5},{item.lon:F5} way {sections[0].Way}");
        }

        // Shape of the short ones: where a one-way run under 250 m begins and ends.
        var all = plan;
        var incident = new Dictionary<long, List<Sec>>();

        foreach (var section in all)
            foreach (var id in new[] { section.A, section.B })
            {
                if (!incident.TryGetValue(id, out var sections))
                    incident[id] = sections = new();

                sections.Add(section);
            }

        var shapes = new Dictionary<string, (int N, double Km, string At)>();

        foreach (var sections in runs.Where(otherSections => Hw.TotalLength(otherSections) < 250))
        {
            var ids =
                sections
                    .SelectMany(section => new[] { section.A, section.B })
                    .GroupBy(id => id)
                    .ToDictionary(group => group.Key, group => group.Count());

            var ends = ids.Where(entry => entry.Value == 1).Select(entry => entry.Key).ToList();

            string End(long id)
            {
                var arms = incident[id].Where(section => !sections.Contains(section)).ToList();
                return arms.Count == 0
                    ? "dead"
                    : arms.Count(section => !section.OneWay) == 1 && arms.Count == 1
                        ? "road"
                        : arms.Any(section => !section.OneWay) ? "junction+twoway" : "junction-oneway";
            }

            var key =
                string.Join(" / ", ends.Select(End).OrderBy(otherText => otherText))
                    + $"  ends={ends.Count} sidearms={sections.SelectMany(section => new[] { section.A, section.B }).Distinct().Count(id => ids[id] == 2 && incident[id].Count > 2)}";

            var item = capture.ToGeographic(sections[0].P0.x, sections[0].P0.z);
            shapes.TryGetValue(key, out var otherItem);
            shapes[key] =
                (
                    otherItem.N + 1,
                    otherItem.Km + Hw.TotalLength(sections) / 1000,
                    otherItem.At ?? $"{item.lat:F5},{item.lon:F5}");
        }

        foreach (var entry in shapes.OrderByDescending(otherEntry => otherEntry.Value.N).Take(14))
            Console.WriteLine($"  {entry.Value.N,4} {entry.Value.Km,5:F1} km {entry.Key}  e.g. {entry.Value.At}");
    }

    // Each live edge against the plan section with the same ends: what was asked and what was drawn.
    public static void Asked(string map)
    {
        var capture = new Cap(map);
        var t = capture.Heights("native-observed-heightmap.u16");
        var edges = Edges(capture).Where(edge => edge.Kind == "Road").ToList();

        var plan = new Dictionary<(int, int, int, int), (bool Tunnel, bool Bridge, float? A, float? B, string Cls)>();

        (int, int) GridKey(double3 point) => ((int)Math.Round(point.x), (int)Math.Round(point.z));

        JObject? first = null;

        foreach (var row in capture.ManifestJ("final-prepared-network-chunks.json"))
        {
            first ??= row;

            if (((string?)row["kind"]
                ?? (string?)row["Kind"])?.Equals("Road", StringComparison.OrdinalIgnoreCase)
                != true)
                continue;

            var firstItem = GridKey(ReadFlexibleVector(row["a"]!));
            var secondItem = GridKey(ReadFlexibleVector(row["d"]!));
            var item =
                (
                    (bool?)row["Tunnel"] ?? false,
                    (bool?)row["Bridge"] ?? false,
                    (float?)row["StartElevation"],
                    (float?)row["EndElevation"],
                    (string?)row["Classification"] ?? "");

            plan[(firstItem.Item1, firstItem.Item2, secondItem.Item1, secondItem.Item2)] = item;
            plan[(secondItem.Item1, secondItem.Item2, firstItem.Item1, firstItem.Item2)] =
                (item.Item1, item.Item2, item.Item4, item.Item3, item.Item5);
        }

        Console.WriteLine(
            "record has StartElevation: " + (first?["StartElevation"] != null) + "; plan keys " + plan.Count);

        var rows = new Dictionary<string, (int N, double Km)>();
        var shown = 0;

        foreach (var edge in edges)
        {
            var drawn =
                edge.Has("Tunnel")
                    ? "TUNNEL"
                    : edge.Has("Elevated")
                        ? "ELEV"
                        : edge.Left.Contains("Lowered") && edge.Right.Contains("Lowered")
                            ? "CUT"
                            : edge.Left.Contains("Raised") && edge.Right.Contains("Raised") ? "WALL" : "ground";

            var firstItem = GridKey(edge.P0);
            var secondItem = GridKey(edge.P3);
            string asked;

            if (!plan.TryGetValue((firstItem.Item1, firstItem.Item2, secondItem.Item1, secondItem.Item2), out var item))
                asked = "no exact plan match";
            else
            {
                string ElevationBand(float? v)
                    => v == null
                        ? "null"
                        : v <= -15 ? "<=-15" : v <= -10 ? "-10..-15" : v < 0 ? "-0..-10" : v == 0 ? "0" : ">0";

                asked =
                    $"plan {(item.Tunnel ? "tunnel" : item.Bridge ? "bridge" : "ground")} elev {ElevationBand(item.A)}/{ElevationBand(item.B)}";
            }

            if (drawn is "ground" or "ELEV" && !asked.Contains("tunnel"))
                continue;

            var key = $"{drawn,-7} {asked}";
            rows.TryGetValue(key, out var otherItem);
            rows[key] = (otherItem.N + 1, otherItem.Km + edge.Len / 1000);

            if (drawn == "CUT" && asked.Contains("<=-15/<=-15") && shown++ < 10)
            {
                var thirdItem = capture.ToGeographic(edge.P0.x, edge.P0.z);
                Console.WriteLine(
                    $"  forced but cut: {thirdItem.lat:F5},{thirdItem.lon:F5} L{edge.Len:F0} y {edge.P0.y:F1}>{edge.P3.y:F1} ground over {capture.HeightAt(t, edge.P0.x, edge.P0.z) - edge.P0.y:F1},{capture.HeightAt(t, edge.P3.x, edge.P3.z) - edge.P3.y:F1}");
            }
        }

        // Sections asked for tunnel at both ends: drawn by real ground over the deck.
        var bins = new[] { 4d, 6, 7, 8, 9, 10, 11, 12, 13, 14, 16, 99 };
        var counts = new int[bins.Length, 2];

        foreach (var edge in edges)
        {
            if (!edge.Has("Tunnel") && !(edge.Left.Contains("Lowered") && edge.Right.Contains("Lowered")))
                continue;

            var firstItem = GridKey(edge.P0);
            var secondItem = GridKey(edge.P3);

            if (!plan.TryGetValue((firstItem.Item1, firstItem.Item2, secondItem.Item1, secondItem.Item2), out var item)
                || !(item.A <= -15 && item.B <= -15))
                continue;

            // The game reads real depth at the middle, at the road's two sides.
            var m = edge.At(.5);
            var d = math.normalizesafe(edge.At(.55).xz - edge.At(.45).xz);
            var side = new double2(-d.y, d.x) * edge.Width / 2;
            var cover =
                Math.Min(
                    capture.HeightAt(t, m.x + side.x, m.z + side.y),
                    capture.HeightAt(t, m.x - side.x, m.z - side.y))
                    - m.y;

            var k = Array.FindIndex(bins, x => cover < x);
            counts[k < 0 ? bins.Length - 1 : k, edge.Has("Tunnel") ? 1 : 0]++;
        }

        for (var k = 0; k < bins.Length; k++)
            Console.WriteLine(
                $"  forced, middle ground under {bins[k],3:F0} m: cut {counts[k, 0],4} tunnel {counts[k, 1],4}");

        // Ground sections drawn as cuts: was the terrain changed, and do they touch a bore?
        var before = capture.Heights("offline-design-pass-01-heightmap.u16");
        var tunnelNodes =
            new HashSet<string>(edges.Where(edge => edge.Has("Tunnel")).SelectMany(edge => new[] { edge.A, edge.B }));

        var kinds = new Dictionary<string, (int N, double Km)>();

        foreach (var edge in edges)
        {
            if (!(edge.Left.Contains("Lowered") && edge.Right.Contains("Lowered")))
                continue;

            var firstItem = GridKey(edge.P0);
            var secondItem = GridKey(edge.P3);

            if (plan.TryGetValue((firstItem.Item1, firstItem.Item2, secondItem.Item1, secondItem.Item2), out var item)
                && item.Tunnel)
                continue;

            var m = edge.At(.5);
            var moved = capture.HeightAt(t, m.x, m.z) - capture.HeightAt(before, m.x, m.z);
            var cover = capture.HeightAt(t, m.x, m.z) - m.y;
            var key =
                $"not bore, deep {(cover < 3 ? "<3" : cover < 6 ? "3-6" : cover < 10 ? "6-10" : "10+"),-5} terrain {(Math.Abs(moved) < .5 ? "untouched" : moved < 0 ? "cut" : "raised"),-9} touches tunnel {tunnelNodes.Contains(edge.A) || tunnelNodes.Contains(edge.B)}";

            kinds.TryGetValue(key, out var otherItem);
            kinds[key] = (otherItem.N + 1, otherItem.Km + edge.Len / 1000);
        }

        foreach (var entry in kinds.OrderBy(otherEntry => otherEntry.Key))
            Console.WriteLine($"  {entry.Value.N,5} {entry.Value.Km,6:F2} km {entry.Key}");

        Console.WriteLine($"{map}:");

        foreach (var entry in rows.OrderBy(otherEntry => otherEntry.Key))
            Console.WriteLine($"  {entry.Value.N,5} {entry.Value.Km,6:F2} km {entry.Key}");
    }

    // Along each planned river: can water stand there on the terrain the game ended up with?
    public static void Reaches(string map, string? name)
    {
        var capture = new Cap(map);
        var t = capture.Heights("native-observed-heightmap.u16");
        var data = capture.LoadOsm();
        var reaches = capture.Manifest<JObject>(capture.InputValue("waterReachesManifest")).ToList();

        if (reaches.Count == 0)
            return;

        Console.WriteLine("fields: " + string.Join(",", reaches[0].Properties().Select(property => property.Name)));

        var rows =
            reaches
                .Where(
                    row => row["PolygonId"] == null
                        || row["PolygonId"]!.Type == JTokenType.Null)
                .Select(row =>
        {
            var vectorA = ReadFlexibleVector(row["Start"]!);
            var vectorB = ReadFlexibleVector(row["End"]!);
            var m = (vectorA + vectorB) / 2;
            var half = (double)row["HalfWidth"]!;
            var ground = (double)capture.HeightAt(t, m.x, m.z);
            var stage = ((double)row["StartLevel"]! + (double)row["EndLevel"]!) / 2;
            var d = math.normalizesafe((vectorB - vectorA).xz);
            var side = new double2(-d.y, d.x) * Math.Max(half, 14);
            var bank =
                Math.Min(
                    (double)capture.HeightAt(t, m.x + side.x, m.z + side.y),
                    (double)capture.HeightAt(t, m.x - side.x, m.z - side.y));

            return (
                M: m,
                Len: math.distance(vectorA.xz, vectorB.xz),
                Half: half,
                Over: stage - ground,
                Bank: bank - stage,
                Way: (long?)row["WayId"] ?? 0);
        }).ToList();

        var km = rows.Sum(item => item.Len) / 1000;
        Console.WriteLine(
            $"{map}: river reaches {rows.Count}, {km:F0} km; half width <7 {rows.Where(item => item.Half < 7).Sum(item => item.Len) / 1000:F0} km, <14 {rows.Where(item => item.Half < 14).Sum(item => item.Len) / 1000:F0} km, <28 {rows.Where(item => item.Half < 28).Sum(item => item.Len) / 1000:F0} km");

        foreach (var (low, high) in new[] { (-1e9, -1d), (-1d, 0d), (0d, .5d), (.5d, 2d), (2d, 4d), (4d, 1e9) })
            Console.WriteLine(
                $"  stage minus final ground at centre {low,5:F1}..{high,4:F1}: {rows.Where(item => item.Over >= low && item.Over < high).Sum(item => item.Len) / 1000,6:F1} km");

        foreach (var (low, high) in new[] { (-1e9, 0d), (0d, .5d), (.5d, 2d), (2d, 1e9) })
            Console.WriteLine(
                $"  lower bank above stage {low,5:F1}..{high,4:F1}: {rows.Where(item => item.Bank >= low && item.Bank < high).Sum(item => item.Len) / 1000,6:F1} km");

        // Named rivers: runs along the way where water can't stand.
        foreach (var group in rows
            .GroupBy(
                item => data.Ways.TryGetValue(item.Way, out var way) && way.Tags.TryGetValue("name", out var text)
                    ? text
                    : "")
            .Where(otherGroup => otherGroup.Key != "")
            .OrderByDescending(otherGroup => otherGroup.Sum(item => item.Len))
            .Take(10))
        {
            var dry = group.Where(item => item.Over < .3).Sum(item => item.Len) / 1000;
            var spill = group.Where(item => item.Bank < 0).Sum(item => item.Len) / 1000;
            var otherItem = group.OrderBy(item => item.Over).First();
            var geo = capture.ToGeographic(otherItem.M.x, otherItem.M.z);
            Console.WriteLine(
                $"  {group.Key,-26} {group.Sum(item => item.Len) / 1000,5:F1} km; ground above surface {dry,5:F1} km; bank under surface {spill,5:F1} km; median half width {group.Select(item => item.Half).OrderBy(v => v).ElementAt(group.Count() / 2):F0} m; worst {otherItem.Over:F1} m at {geo.lat:F4},{geo.lon:F4}");
        }
    }

    // The run's own screenshots and logged counts.
    public static void Shots(string map)
    {
        var capture = new Cap(map);
        var manifest = Path.Combine(capture.Folder, "site-shots.json");

        foreach (var text in File.ReadLines(Path.Combine(capture.Folder, "events.log")))
            if (text.Contains("OUTSIDE CONNECTIONS")
                || text.Contains("DRAWN PREDICTION")
                || text.Contains("SITE SHOTS"))
                Console.WriteLine(text.Substring(Math.Min(text.Length, 9)));

        if (!File.Exists(manifest))
        {
            Console.WriteLine("no site shots in this run");
            return;
        }

        foreach (var token in JArray.Parse(File.ReadAllText(manifest)))
            Console.WriteLine(
                $"  {Path.Combine(capture.Folder, (string)token["file"]!)}  {token["Name"]}  {(double)token["lat"]!:F5},{(double)token["lon"]!:F5} {token["failure"]}");
    }

    // The last wetness reading of each water body from the run's log, worst covered first.
    public static void Rivers(string map)
    {
        var capture = new Cap(map);
        var data = capture.LoadOsm();
        var last =
            new Dictionary<long, (string Kind, int Wet, int All, int Flood, int Banks, double Level, int Controls)>();

        foreach (var text in File.ReadLines(Path.Combine(capture.Folder, "events.log")))
        {
            if (!text.Contains("WATER BODY RESULT"))
                continue;

            var match =
                Regex.Match(
                    text,
                    @"body=(-?\d+); kind=(\w+); controls=(\d+)/\d+;.*wetDownstreamProbes=(\d+)/(\d+); floodedBankProbes=(\d+)/(\d+);.*medianObservedLevel=([\d.NaN]+)");

            if (match.Success)
                last[long.Parse(match.Groups[1].Value)] =
                    (
                        match.Groups[2].Value,
                        int.Parse(match.Groups[4].Value),
                        int.Parse(match.Groups[5].Value),
                        int.Parse(match.Groups[6].Value),
                        int.Parse(match.Groups[7].Value),
                        double.TryParse(match.Groups[8].Value, out var d) ? d : double.NaN,
                        int.Parse(match.Groups[3].Value));
        }

        var rivers = last.Where(entry => entry.Value.Kind == "river").ToList();
        Console.WriteLine(
            $"{map}: bodies {last.Count}; rivers {rivers.Count}: channel probes wet {rivers.Sum(entry => entry.Value.Wet)}/{rivers.Sum(entry => entry.Value.All)}, bank probes flooded {rivers.Sum(entry => entry.Value.Flood)}/{rivers.Sum(entry => entry.Value.Banks)}");

        foreach (var entry in rivers.OrderByDescending(otherEntry => otherEntry.Value.All).Take(24))
        {
            data.Ways.TryGetValue(entry.Key, out var way);

            string Tag(string text) => way != null && way.Tags.TryGetValue(text, out var otherText) ? otherText : "-";

            Console.WriteLine(
                $"  {entry.Key,11} {Tag("name"),-26} {Tag("waterway"),-8} wet {entry.Value.Wet,3}/{entry.Value.All,3} banks wet {entry.Value.Flood,3}/{entry.Value.Banks,3} controls {entry.Value.Controls,3} width={Tag("width")}");
        }
    }

    // Ground roads the game walled: did the terrain try to meet them?
    public static void Walls(string map)
    {
        var capture = new Cap(map);
        var t = capture.Heights("native-observed-heightmap.u16");
        var before = capture.Heights("offline-design-pass-01-heightmap.u16");
        var edges =
            Edges(capture)
                .Where(
                    edge => edge.Kind == "Road"
                        && !edge.Has("Elevated")
                        && !edge.Has("Tunnel")
                        && (edge.Left.Contains("Raised") || edge.Right.Contains("Raised")))
                .ToList();

        var rows = new Dictionary<string, (int N, double Km)>();

        foreach (var edge in edges)
        {
            var m = edge.At(.5);
            var over = m.y - capture.HeightAt(t, m.x, m.z);
            var was = m.y - capture.HeightAt(before, m.x, m.z);
            var moved = capture.HeightAt(t, m.x, m.z) - capture.HeightAt(before, m.x, m.z);
            var key =
                $"{(edge.Left.Contains("Raised") && edge.Right.Contains("Raised") ? "both" : "one "),-4} deck over final ground {(over < 2 ? "<2" : over < 4 ? "2-4" : over < 6 ? "4-6" : over < 8 ? "6-8" : "8+"),-4} over original {(was < 2 ? "<2" : was < 5 ? "2-5" : was < 8 ? "5-8" : "8+"),-4} terrain {(Math.Abs(moved) < .5 ? "untouched" : moved > 0 ? "raised" : "lowered")}";

            rows.TryGetValue(key, out var item);
            rows[key] = (item.N + 1, item.Km + edge.Len / 1000);
        }

        Console.WriteLine($"{map}: walled ground edges {edges.Count}, {edges.Sum(edge => edge.Len) / 1000:F1} km");

        foreach (var entry in rows.OrderBy(otherEntry => otherEntry.Key))
            Console.WriteLine($"  {entry.Value.N,5} {entry.Value.Km,6:F2} km {entry.Key}");

        foreach (var edge in edges
            .Where(otherEdge => otherEdge.Left.Contains("Raised") && otherEdge.Right.Contains("Raised"))
            .OrderByDescending(
                otherEdge => otherEdge.Len
                    * (otherEdge.At(.5).y - capture.HeightAt(t, otherEdge.At(.5).x, otherEdge.At(.5).z)))
            .Take(16))
        {
            var m = edge.At(.5);
            var item = capture.ToGeographic(m.x, m.z);
            Console.WriteLine(
                $"  {item.lat:F5},{item.lon:F5} L{edge.Len,4:F0} w{edge.Width,3:F0} over {m.y - capture.HeightAt(t, m.x, m.z),5:F1} orig {m.y - capture.HeightAt(before, m.x, m.z),5:F1}");
        }
    }

    // Where two named roads come closest, to locate a reported site.
    public static void Find(string map, string nameA, string nameB)
    {
        var capture = new Cap(map);
        var data = capture.LoadOsm();

        IEnumerable<double3> Points(string name)
            => data.Ways.Values
                .Where(
                    way => way.Tags.ContainsKey("highway")
                        && ((way.Tags.TryGetValue("name", out var text)
                            && text.Contains(name, StringComparison.OrdinalIgnoreCase))
                            || (way.Tags.TryGetValue("ref", out var otherText)
                                && otherText.Contains(name, StringComparison.OrdinalIgnoreCase))))
            .SelectMany(way => way.NodeIds).Where(data.Nodes.ContainsKey).Select(id =>
{
    var point = capture.Local(data.Nodes[id].Latitude, data.Nodes[id].Longitude);
    return new double3(point.x, point.y, point.z);
});

        var firstPoints = Points(nameA).ToList();
        var secondPoints = nameB == "" ? firstPoints : Points(nameB).ToList();

        if (firstPoints.Count == 0 || secondPoints.Count == 0)
        {
            Console.WriteLine($"{map}: no match ({firstPoints.Count},{secondPoints.Count})");
            return;
        }

        var best = double.MaxValue;
        var at = firstPoints[0];

        foreach (var point in firstPoints)
            foreach (var otherPoint in secondPoints)
            {
                var otherDistance = math.distance(point.xz, otherPoint.xz);

                if (otherDistance < best)
                {
                    best = otherDistance;
                    at = (point + otherPoint) / 2;
                }
            }

        var item = capture.ToGeographic(at.x, at.z);
        Console.WriteLine(
            $"{map} {nameA} / {nameB}: {item.lat:F5},{item.lon:F5} gap {best:F0} m ({firstPoints.Count},{secondPoints.Count} nodes)");
    }

    internal static double3 ReadFlexibleVector(JToken token) => token is JArray firstArray
                    ? new((double)firstArray[0], (double)firstArray[1], (double)firstArray[2])
                    : new((double)token["x"]!, (double)token["y"]!, (double)token["z"]!);

    internal static double3 ReadVector(JToken token)
        => new((double)token["x"]!, (double)token["y"]!, (double)token["z"]!);
}
