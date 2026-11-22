using ImportEarthLocation.Core;
using Unity.Mathematics;

// Game-pass triage on a capture: what the visible tunnel, junction and sand defects look like in the data.
public static class Triage
{
    // A replay's plan when PLAN_DIR is set (its folder holding <map>/plot-sections.ndjson.gz), else what the game received.
    static readonly string? Dir =
        string.IsNullOrEmpty(Environment.GetEnvironmentVariable("PLAN_DIR"))
            ? null
            : Environment.GetEnvironmentVariable("PLAN_DIR");

    static List<Sec> Plan(Cap capture)
        => Dir == null
            ? Compare.Final(capture)
            : Hw.Load(Path.Combine(Dir, Cap.Ids.First(entry => Path.GetFileName(capture.Folder) == entry.Value).Key));

    public static List<Sec> PlanOf(Cap capture) => Plan(capture);

    static readonly string[] Classes =
        { "motorway", "motorway_link", "trunk", "trunk_link", "primary", "secondary", "tertiary" };

    static float[] Observed(Cap capture)
        => capture.Heights(
            capture.Has("native-observed-heightmap.u16")
                ? "native-observed-heightmap.u16"
                : "final-design-heightmap.u16");

    public static float Sea(Cap capture)
    {
        foreach (var line in File.ReadLines(Path.Combine(capture.Folder, "events.log")))
        {
            var match =
                System.Text.RegularExpressions.Regex.Match(line, @"effective water ([\d.]+) m|seaSurface=([\d.]+)");

            if (match.Success)
                return float.Parse(match.Groups[1].Success ? match.Groups[1].Value : match.Groups[2].Value);
        }

        throw new InvalidOperationException("no sea level in the log");
    }

    // Ends of the plan with no other section: what became of the OSM road that continues there.
    public static void Ends(string map)
    {
        var capture = new Cap(map);
        var data = capture.LoadOsm();
        var plan = Plan(capture).Where(section => section.Kind == "Road").ToList();
        var half = capture.Width / 2 - 300;
        var degree = new Dictionary<long, int>();

        foreach (var section in plan)
        {
            degree[section.A] = degree.GetValueOrDefault(section.A) + 1;
            degree[section.B] = degree.GetValueOrDefault(section.B) + 1;
        }

        var planned = new HashSet<long>(plan.SelectMany(section => section.Ways.Append(section.Way)));
        var byNode = new Dictionary<long, List<OsmWay>>();

        foreach (var way in data.Ways.Values)
        {
            if (!way.Tags.ContainsKey("highway"))
                continue;

            foreach (var id in way.NodeIds)
            {
                if (!byNode.TryGetValue(id, out var waies))
                    byNode[id] = waies = new();

                waies.Add(way);
            }
        }

        var rows = new Dictionary<string, int>();
        var sites = new List<string>();
        var total = 0;
        var edge = 0;

        foreach (var section in plan)
            foreach (var (id, point) in new[] { (section.A, section.P0), (section.B, section.P3) })
            {
                if (degree[id] != 1)
                    continue;

                if (Math.Abs(point.x) > half || Math.Abs(point.z) > half)
                {
                    edge++;
                    continue;
                }

                total++;
                var continuation =
                    byNode.TryGetValue(id, out var ways)
                        ? ways.Where(way => !planned.Contains(way.Id)).ToList()
                        : new();

                string Tag(OsmWay way, string text) => way.Tags.TryGetValue(text, out var otherText) ? otherText : "-";

                var reason =
                    !byNode.ContainsKey(id) ? "synthetic node" : continuation.Count == 0 ? "no dropped way at node" :
                    string.Join(
                        "+",
                        continuation
                            .Select(
                                way => Hw.Group(Tag(way, "highway"))
                                    + (Tag(way, "tunnel") != "-" && Tag(way, "tunnel") != "no" ? ":tunnel" : ""))
                            .Distinct()
                            .OrderBy(text => text));

                var key = $"{Hw.Group(section.Cls),-14} {reason}";
                rows[key] = rows.GetValueOrDefault(key) + 1;

                if ((Hw.Group(section.Cls) is "motorway" or "motorway_link" or "trunk" or "trunk_link"
                        || reason == "synthetic node")
                    && sites.Count < 40)
                {
                    var item = capture.ToGeographic(point.x, point.z);
                    sites.Add(
                        $"  {item.lat:F4},{item.lon:F4} {section.Cls} way {section.Way} tunnel={section.Tunnel} bridge={section.Bridge}: {reason}");
                }
            }

        Console.WriteLine($"{map}: plan dead ends {total} inland ({edge} at map edge)");

        foreach (var entry in rows.OrderByDescending(otherEntry => otherEntry.Value).Take(28))
            Console.WriteLine($"  {entry.Value,5} {entry.Key}");

        foreach (var text in sites)
            Console.WriteLine(text);
    }

    // Connected ground below the water surface that does not reach the map edge
    // or water deeper than the seabed paint: it cannot fill from the sea.
    public static void Sand(string map)
    {
        var capture = new Cap(map);
        var t = capture.Heights("final-design-heightmap.u16");
        var n = (int)Math.Sqrt(t.Length);
        var sea = Sea(capture);
        const int step = 4;
        var m = n / step;
        var scale = capture.Scale;

        float HeightAt(int x, int z) => t[z * step * n + x * step];

        var cell = capture.Width / n * step;
        var seen = new bool[m * m];
        var parts = new List<(int Count, int X, int Z, bool Open, double MaxDepth)>();

        for (var z0 = 0; z0 < m; z0++)
            for (var x0 = 0; x0 < m; x0++)
            {
                if (seen[z0 * m + x0] || HeightAt(x0, z0) >= sea)
                    continue;

                var q = new Queue<int>();
                q.Enqueue(z0 * m + x0);
                seen[z0 * m + x0] = true;
                var count = 0;
                var open = false;
                var sx = 0L;
                var sz = 0L;
                var deepest = 0d;

                while (q.Count > 0)
                {
                    var i = q.Dequeue();
                    var x = i % m;
                    var z = i / m;
                    count++;
                    sx += x;
                    sz += z;
                    deepest = Math.Max(deepest, sea - HeightAt(x, z));

                    if (x == 0 || z == 0 || x == m - 1 || z == m - 1)
                        open = true;

                    foreach (var (a, b) in new[] { (x - 1, z), (x + 1, z), (x, z - 1), (x, z + 1) })
                        if (a >= 0 && b >= 0 && a < m && b < m && !seen[b * m + a] && HeightAt(a, b) < sea)
                        {
                            seen[b * m + a] = true;
                            q.Enqueue(b * m + a);
                        }
                }

                parts.Add((count, (int)(sx / count), (int)(sz / count), open, deepest));
            }

        var area = cell * cell / 1e4;
        Console.WriteLine(
            $"{map}: sea {sea:F2}; cell {cell:F0} m; below-surface groups {parts.Count}; open to edge {parts.Count(item => item.Open)} ({parts.Where(item => item.Open).Sum(item => item.Count) * area:F0} ha); enclosed {parts.Count(item => !item.Open)} ({parts.Where(item => !item.Open).Sum(item => item.Count) * area:F0} ha)");

        foreach (var item in parts.OrderByDescending(otherItem => otherItem.Count).Take(24))
        {
            var otherItem =
                capture.ToGeographic(
                    (item.X + .5) * cell - capture.Width / 2,
                    (item.Z + .5) * cell - capture.Width / 2);

            Console.WriteLine(
                $"  {item.Count * area,8:F1} ha {otherItem.lat:F4},{otherItem.lon:F4} maxDepth {item.MaxDepth,5:F1} {(item.Open ? "open" : "ENCLOSED")}");
        }
    }

    // Tunnel length against OSM, runs by length, and where the plan's tunnels have no OSM tunnel.
    public static void Tunnels(string map)
    {
        var capture = new Cap(map);
        var data = capture.LoadOsm();
        var plan = Plan(capture).Where(section => section.Kind == "Road").ToList();
        var after = Observed(capture);
        var before = capture.Heights("offline-design-pass-01-heightmap.u16");

        bool Tagged(long way)
            => data.Ways.TryGetValue(way, out var otherWay)
                && (otherWay.Tags.TryGetValue("tunnel", out var text) && text != "no"
                    || otherWay.Tags.TryGetValue("covered", out var otherText) && otherText == "yes"
                    || otherWay.Tags.TryGetValue("layer", out var secondText) && secondText.StartsWith("-"));

        Console.WriteLine(
            $"{map}: class  plan km | tunnel km  of which no OSM tag | tunnel sections <60 m | median roof, m | roof<4 m share");

        foreach (var thirdText in Classes)
        {
            var mine = plan.Where(section => Hw.Group(section.Cls) == thirdText).ToList();
            var tubes = mine.Where(section => section.Tunnel).ToList();
            var roofs = new List<double>();

            foreach (var section in tubes)
                for (var k = 1; k < 8; k++)
                {
                    var point = section.At(k / 8d);
                    roofs.Add(capture.HeightAt(after, point.x, point.z) - point.y);
                }

            roofs.Sort();
            Console.WriteLine(
                $"{thirdText,-14} {mine.Sum(section => section.ArcLength()) / 1000,7:F0} | {tubes.Sum(section => section.ArcLength()) / 1000,6:F1} {tubes.Where(section => !section.Ways.Append(section.Way).Any(Tagged)).Sum(section => section.ArcLength()) / 1000,6:F1} | {tubes.Count(section => section.ArcLength() < 60),5} | {(roofs.Count > 0 ? roofs[roofs.Count / 2] : 0),6:F1} | {(roofs.Count > 0 ? roofs.Count(r => r < 4) / (double)roofs.Count : 0),6:P0}");
        }

        // Runs of connected tunnel sections.
        var parent = Enumerable.Range(0, plan.Count).ToArray();

        int Find(int i)
        {
            while (parent[i] != i)
                i = parent[i] = parent[parent[i]];

            return i;
        }

        var at = new Dictionary<long, int>();

        for (var i = 0; i < plan.Count; i++)
        {
            if (!plan[i].Tunnel)
                continue;

            foreach (var id in new[] { plan[i].A, plan[i].B })
            {
                if (at.TryGetValue(id, out var j))
                    parent[Find(i)] = Find(j);
                else
                    at[id] = i;
            }
        }

        var runs =
            Enumerable
                .Range(0, plan.Count)
                .Where(i => plan[i].Tunnel)
                .GroupBy(Find)
                .Select(group => group.Select(i => plan[i]).ToList())
                .ToList();

        Console.WriteLine(
            $"tunnel runs {runs.Count}: <50 m {runs.Count(sections => Hw.TotalLength(sections) < 50)}, 50-100 {runs.Count(sections => Hw.TotalLength(sections) >= 50 && Hw.TotalLength(sections) < 100)}, 100-250 {runs.Count(sections => Hw.TotalLength(sections) >= 100 && Hw.TotalLength(sections) < 250)}, 250+ {runs.Count(sections => Hw.TotalLength(sections) >= 250)}; no OSM tag at all: {runs.Count(sections => !sections.Any(section => section.Ways.Append(section.Way).Any(Tagged)))}");

        // Earthwork around each portal: the final terrain against the terrain before roads.
        var rows = new List<(double Move, double Len, string Cls, double Lat, double Lon, bool Osm)>();

        foreach (var sections in runs)
        {
            var move = 0d;

            foreach (var section in sections)
                foreach (var point in new[] { section.P0, section.P3 })
                    for (var a = 0; a < 8; a++)
                        for (var d = 10; d <= 50; d += 20)
                        {
                            var x = point.x + Math.Cos(a * Math.PI / 4) * d;
                            var z = point.z + Math.Sin(a * Math.PI / 4) * d;
                            move =
                                Math.Max(
                                    move,
                                    Math.Abs(capture.HeightAt(after, x, z) - capture.HeightAt(before, x, z)));
                        }

            var item = capture.ToGeographic(sections[0].P0.x, sections[0].P0.z);
            rows.Add(
                (
                    move,
                    Hw.TotalLength(sections),
                    Hw.Group(sections[0].Cls),
                    item.lat,
                    item.lon,
                    sections.Any(section => section.Ways.Append(section.Way).Any(Tagged))));
        }

        var moved = rows.Select(item => item.Move).OrderBy(x => x).ToList();

        if (moved.Count > 0)
            Console.WriteLine(
                $"terrain moved within 50 m of portals: median {moved[moved.Count / 2]:F1} m, p90 {moved[moved.Count * 9 / 10]:F1} m, max {moved[^1]:F1} m");

        foreach (var firstItem in rows.OrderByDescending(item => item.Move).Take(16))
            Console.WriteLine(
                $"  moved {firstItem.Move,5:F1} m  run {firstItem.Len,6:F0} m {firstItem.Cls,-14} {firstItem.Lat:F4},{firstItem.Lon:F4} osmTag={firstItem.Osm}");
    }

    // Terrain the road design changed: cut and fill over the whole map, largest patches first.
    public static void Earth(string map)
    {
        var capture = new Cap(map);
        var after = capture.Heights("final-design-heightmap.u16");
        var before = capture.Heights("offline-design-pass-01-heightmap.u16");
        var n = (int)Math.Sqrt(after.Length);
        const int step = 4;
        var m = n / step;
        var d = new float[m * m];

        for (var z = 0; z < m; z++)
            for (var x = 0; x < m; x++)
                d[z * m + x] =
                    after[z * step * n + x * step] - before[z * step * n + x * step];

        var cell = capture.Width / n * step;
        var area = cell * cell / 1e4;

        foreach (var limit in new[] { 2f, 5f, 10f, 20f })
            Console.WriteLine(
                $"{map}: |terrain change| > {limit} m: {d.Count(v => Math.Abs(v) > limit) * area:F0} ha (raised {d.Count(v => v > limit) * area:F0}, lowered {d.Count(v => v < -limit) * area:F0})");
    }

    public static void Grades(string map)
    {
        var capture = new Cap(map);
        var plan = Plan(capture).Where(section => section.Kind == "Road").ToList();
        Console.WriteLine($"{map}: class sections | <20 m | <40 m | grade>10% | grade>20% (sections over 15 m)");

        foreach (var text in Classes)
        {
            var mine = plan.Where(section => Hw.Group(section.Cls) == text).ToList();

            double Grade(Sec section) => Math.Abs(section.P3.y - section.P0.y) / Math.Max(section.ArcLength(), 1);

            Console.WriteLine(
                $"{text,-14} {mine.Count,6} | {mine.Count(section => section.ArcLength() < 20),5} | {mine.Count(section => section.ArcLength() < 40),5} | {mine.Count(section => section.ArcLength() > 15 && Grade(section) > .1),5} | {mine.Count(section => section.ArcLength() > 15 && Grade(section) > .2),5}");
        }

        foreach (var section in plan
            .Where(otherSection => otherSection.ArcLength() > 15)
            .OrderByDescending(
                otherSection => Math.Abs(otherSection.P3.y - otherSection.P0.y) / otherSection.ArcLength())
            .Take(14))
        {
            var item = capture.ToGeographic(section.P0.x, section.P0.z);
            Console.WriteLine(
                $"  {Math.Abs(section.P3.y - section.P0.y) / section.ArcLength(),6:P0} over {section.ArcLength(),5:F0} m {section.Cls,-14} {item.lat:F4},{item.lon:F4} way {section.Way} bridge={section.Bridge} tunnel={section.Tunnel}");
        }
    }

    // Short main-road pieces by what sits at their ends, and how far each
    // dead end lies from the nearest other road.
    public static void Pieces(string map)
    {
        var capture = new Cap(map);
        var all = Plan(capture);
        var round = new bool[all.Count];
        var dual = new bool[all.Count];
        var data = capture.LoadOsm();

        for (var k = 0; k < all.Count; k++)
            round[k] =
                all[k].Ways
                    .Append(all[k].Way)
                    .Any(
                        wayId => data.Ways.TryGetValue(wayId, out var otherWay)
                            && otherWay.Tags.TryGetValue("junction", out var text)
                            && text is "roundabout" or "circular");

        var plan = all.Where(section => section.Kind == "Road").ToList();
        var degree = new Dictionary<long, int>();

        foreach (var section in plan)
        {
            degree[section.A] = degree.GetValueOrDefault(section.A) + 1;
            degree[section.B] = degree.GetValueOrDefault(section.B) + 1;
        }

        Console.WriteLine(
            $"{map}: class <20 m total | roundabout | avenue | junction both ends | junction one end | plain/dead");

        foreach (var text in new[] { "primary", "secondary", "tertiary", "motorway_link", "trunk_link" })
        {
            var shorts = plan.Where(section => Hw.Group(section.Cls) == text && section.ArcLength() < 20).ToList();

            int JunctionEnds(Sec section) => (degree[section.A] >= 3 ? 1 : 0) + (degree[section.B] >= 3 ? 1 : 0);

            Console.WriteLine(
                $"{text,-14} {shorts.Count,5} | {shorts.Count(section => round[section.Index]),5} | {shorts.Count(section => dual[section.Index]),5} | {shorts.Count(section => !round[section.Index] && JunctionEnds(section) == 2),5} | {shorts.Count(section => !round[section.Index] && JunctionEnds(section) == 1),5} | {shorts.Count(section => !round[section.Index] && JunctionEnds(section) == 0),5}");
        }

        var cells = new Dictionary<(int, int), List<(double2 P, int Section)>>();

        foreach (var section in plan)
            for (var k = 0; k <= 8; k++)
            {
                var point = section.At(k / 8d).xz;
                var key = ((int)Math.Floor(point.x / 64), (int)Math.Floor(point.y / 64));

                if (!cells.TryGetValue(key, out var items))
                    cells[key] = items = new();

                items.Add((point, section.Index));
            }

        var half = capture.Width / 2 - 300;
        var gaps = new Dictionary<string, List<double>>();
        var examples = new List<string>();

        foreach (var section in plan)
            foreach (var (id, point) in new[] { (section.A, section.P0), (section.B, section.P3) })
            {
                if (degree[id] != 1 || Math.Abs(point.x) > half || Math.Abs(point.z) > half)
                    continue;

                var best = 999d;
                var cx = (int)Math.Floor(point.x / 64);
                var cz = (int)Math.Floor(point.z / 64);

                for (var dz = -2; dz <= 2; dz++)
                    for (var dx = -2; dx <= 2; dx++)
                        if (cells.TryGetValue((cx + dx, cz + dz), out var items))
                            foreach (var item in items)
                                if (item.Section != section.Index && all[item.Section].Way != section.Way)
                                    best = Math.Min(best, math.distance(item.P, point.xz));

                var key = (id < 0 ? "synthetic " : "source    ") + Hw.Group(section.Cls);

                if (!gaps.TryGetValue(key, out var v))
                    gaps[key] = v = new();

                v.Add(best);

                if (id < 0 && best < 40 && examples.Count < 12)
                {
                    var item = capture.ToGeographic(point.x, point.z);
                    examples.Add(
                        $"  {item.lat:F5},{item.lon:F5} {section.Cls} way {section.Way} gap {best:F0} m avenue={dual[section.Index]}");
                }
            }

        foreach (var entry in gaps.OrderBy(otherEntry => otherEntry.Key))
            Console.WriteLine(
                $"  {entry.Key,-24} dead ends {entry.Value.Count,4}: gap<10 {entry.Value.Count(v => v < 10),4}, <25 {entry.Value.Count(v => v < 25),4}, <60 {entry.Value.Count(v => v < 60),4}, <120 {entry.Value.Count(v => v < 120),4}");

        foreach (var text in examples)
            Console.WriteLine(text);
    }

    // Did the plan's dead ends already exist in the design input, or did the compiler make them?
    public static void EndOrigin(string map)
    {
        var capture = new Cap(map);
        var plan = Compare.Final(capture).Where(section => section.Kind == "Road").ToList();
        var input = Compare.Input(capture).Where(section => section.Kind == "Road").ToList();
        var half = capture.Width / 2 - 300;
        var lookup = new Dictionary<long, int>();

        foreach (var section in plan)
        {
            lookup[section.A] = lookup.GetValueOrDefault(section.A) + 1;
            lookup[section.B] = lookup.GetValueOrDefault(section.B) + 1;
        }

        var otherLookup = new Dictionary<long, int>();

        foreach (var section in input)
        {
            otherLookup[section.A] = otherLookup.GetValueOrDefault(section.A) + 1;
            otherLookup[section.B] = otherLookup.GetValueOrDefault(section.B) + 1;
        }

        var rows = new Dictionary<string, int>();
        var shown = 0;

        foreach (var section in plan)
            foreach (var (id, point) in new[] { (section.A, section.P0), (section.B, section.P3) })
            {
                if (lookup[id] != 1 || Math.Abs(point.x) > half || Math.Abs(point.z) > half)
                    continue;

                var was =
                    otherLookup.TryGetValue(id, out var v)
                        ? (v >= 3 ? "junction in input" : v == 2 ? "plain in input" : "dead in input")
                        : "node not in input";

                var key = $"{(id < 0 ? "synthetic" : "source")} {was}";
                rows[key] = rows.GetValueOrDefault(key) + 1;

                if (id < 0 && was != "dead in input" && shown++ < 10)
                {
                    var item = capture.ToGeographic(point.x, point.z);
                    Console.WriteLine(
                        $"  {item.lat:F5},{item.lon:F5} node {id} {section.Cls} way {section.Way} len {section.ArcLength():F0}: {was}");
                }
            }

        Console.WriteLine($"{map}:");

        foreach (var entry in rows.OrderByDescending(otherEntry => otherEntry.Value))
            Console.WriteLine($"  {entry.Value,5} {entry.Key}");
    }

    // Which trunk roads OSM evidence says are limited access. A chain is ways of one ref or name.
    public static void Trunks(string map)
    {
        var capture = new Cap(map);
        var data = capture.LoadOsm();

        IReadOnlyDictionary<string, string> RawTags(OsmWay way)
            => data.RawWayTags.TryGetValue(way.Id, out var lookup) ? lookup : way.Tags;

        var byNode = new Dictionary<long, List<OsmWay>>();

        foreach (var way in data.Ways.Values)
        {
            if (!RawTags(way).ContainsKey("highway"))
                continue;

            foreach (var id in way.NodeIds)
            {
                if (!byNode.TryGetValue(id, out var waies))
                    byNode[id] = waies = new();

                waies.Add(way);
            }
        }

        double Kilometres(OsmWay way)
        {
            var l = 0d;
            float3? previous = null;

            foreach (var id in way.NodeIds)
            {
                if (!data.Nodes.TryGetValue(id, out var node))
                    continue;

                var point = capture.Local(node.Latitude, node.Longitude);

                if (previous is float3 otherPoint)
                    l += math.distance(otherPoint.xz, point.xz);

                previous = point;
            }

            return l / 1000;
        }

        string Tag(OsmWay way, string text) => RawTags(way).TryGetValue(text, out var otherText) ? otherText : "";

        var trunks = data.Ways.Values.Where(way => Tag(way, "highway") == "trunk").ToList();

        foreach (var tag in new[] {
            "control_of_access",
            "expressway",
            "motorroad",
            "access",
            "foot",
            "bicycle",
            "sidewalk",
            "oneway",
            "divided",
            "dual_carriageway"
            })
            Console.WriteLine(
                $"{map} trunk tag {tag}: "
                    + string.Join(
                        ", ",
                        trunks
                            .Where(way => Tag(way, tag) != "")
                            .GroupBy(way => Tag(way, tag))
                            .OrderByDescending(group => group.Sum(Kilometres))
                            .Take(5)
                            .Select(group => $"{group.Key}={group.Sum(Kilometres):F0}km")));

        var rows =
            trunks
                .GroupBy(
                    way => Tag(way, "ref") != ""
                        ? Tag(way, "ref")
                        : Tag(way, "name") != "" ? Tag(way, "name") : "(none)")
                .Select(group =>
        {
            var signals = 0;
            var atGrade = 0;
            var seen = new HashSet<long>();

            foreach (var way in group)
                foreach (var id in way.NodeIds)
                {
                    if (!seen.Add(id))
                        continue;

                    if (data.NodeTags.TryGetValue(id, out var nt)
                        && nt.TryGetValue("highway", out var text)
                        && text == "traffic_signals")
                        signals++;

                    // A road that is not a link or the same road, meeting at grade.
                    if (byNode.TryGetValue(id, out var ways)
                        && ways.Any(
                            otherWay => otherWay.Id != way.Id
                                && !Tag(otherWay, "highway").EndsWith("_link")
                                && Tag(otherWay, "highway") is not ("motorway" or "trunk")
                                && Tag(otherWay, "bridge") == ""
                                && Tag(otherWay, "tunnel") == ""
                                && Tag(way, "bridge") == ""
                                && Tag(way, "tunnel") == ""))
                        atGrade++;
                }

            var km = group.Sum(Kilometres);

            return (
                Name: group.Key,
                Km: km,
                Signals: signals,
                AtGrade: atGrade,
                Coa: string.Join(
                    "/",
                    group.Select(way => Tag(way, "control_of_access")).Where(text => text != "").Distinct()),
                Exp: group.Any(way => Tag(way, "expressway") == "yes" || Tag(way, "motorroad") == "yes"));
        }).OrderByDescending(item => item.Km).ToList();

        double Rate(int n, double km) => n / Math.Max(km, .1);

        Console.WriteLine(
            $"{map}: {trunks.Count} trunk ways, {trunks.Sum(Kilometres):F0} km; route km, signals, at-grade per km, tags");

        foreach (var item in rows.Take(40))
            Console.WriteLine(
                $"  {item.Name,-34} {item.Km,6:F1} km  signals {item.Signals,3}  at-grade {item.AtGrade,3} ({Rate(item.AtGrade, item.Km):F2}/km)  coa={item.Coa} expressway={item.Exp}");

        foreach (var limit in new[] { 0, .25, .5, 1 })
            Console.WriteLine(
                $"  routes with at most {limit}/km at-grade or signal crossings: {rows.Where(item => Rate(item.AtGrade + item.Signals, item.Km) <= limit).Sum(item => item.Km):F0} km");
    }

    // OSM ways by name, and every highway way in a box, with the tags that classify them.
    public static void Ways(string map, string name, double lat0, double lon0, double lat1, double lon1)
    {
        var capture = new Cap(map);
        var data = capture.LoadOsm();
        var planned = new HashSet<long>(Compare.Final(capture).SelectMany(section => section.Ways.Append(section.Way)));

        IReadOnlyDictionary<string, string> RawTags(OsmWay way)
            => data.RawWayTags.TryGetValue(way.Id, out var lookup) ? lookup : way.Tags;

        string Tag(OsmWay way, string text) => RawTags(way).TryGetValue(text, out var otherText) ? otherText : "-";

        bool IsInside(OsmWay way)
            => way.NodeIds.Any(
                id => data.Nodes.TryGetValue(id, out var node)
                    && node.Latitude >= lat0
                    && node.Latitude <= lat1
                    && node.Longitude >= lon0
                    && node.Longitude <= lon1);

        var rows =
            data.Ways.Values
                .Where(
                    way => RawTags(way).ContainsKey("highway")
                        && IsInside(way)
                        && Tag(way, "name").Contains(name, StringComparison.OrdinalIgnoreCase))
                .ToList();

        Console.WriteLine($"{map}: {rows.Count} ways named *{name}* in the box");

        foreach (var group in rows
            .GroupBy(
                way => $"highway={Tag(way, "highway")} service={Tag(way, "service")} oneway={Tag(way, "oneway")} lanes={Tag(way, "lanes")} name={Tag(way, "name")} planned={planned.Contains(way.Id)}")
            .OrderByDescending(otherGroup => otherGroup.Count()))
            Console.WriteLine($"  {group.Count(),3} x {group.Key} e.g. {group.First().Id}");

        Console.WriteLine(
            "all classes in the box: "
                + string.Join(
                    ", ",
                    data.Ways.Values
                        .Where(way => RawTags(way).ContainsKey("highway") && IsInside(way))
                        .GroupBy(way => Tag(way, "highway"))
                        .OrderByDescending(group => group.Count())
                        .Select(group => $"{group.Key}={group.Count()}")));
    }

    // Inland dead ends: one-way or not, and can the data's unimported streets reach a planned road?
    public static void Reach(string map, double limit)
    {
        var capture = new Cap(map);
        var plan = Plan(capture).Where(section => section.Kind == "Road").ToList();
        var half = capture.Width / 2 - 300;
        var data = capture.LoadOsm();
        var lookup = new Dictionary<long, int>();

        foreach (var section in plan)
        {
            lookup[section.A] = lookup.GetValueOrDefault(section.A) + 1;
            lookup[section.B] = lookup.GetValueOrDefault(section.B) + 1;
        }

        var planned = new HashSet<long>(plan.SelectMany(section => section.Ways.Append(section.Way)));
        var cells = new HashSet<(int, int)>();

        foreach (var section in plan)
            for (var k = 0; k <= 8; k++)
            {
                var point = section.At(k / 8d).xz;
                cells.Add(((int)Math.Floor(point.x / 16), (int)Math.Floor(point.y / 16)));
            }

        var osmPlanned =
            new HashSet<long>(
                plan
                    .SelectMany(section => section.Ways.Append(section.Way))
                    .Where(data.Ways.ContainsKey)
                    .SelectMany(wayId => data.Ways[wayId].NodeIds));

        var byNode = new Dictionary<long, List<OsmWay>>();

        foreach (var way in data.Ways.Values)
        {
            if (!way.Tags.TryGetValue("highway", out var text)
                || planned.Contains(way.Id)
                || text is "footway" or "path" or "cycleway" or "steps" or "pedestrian" or "track" or "construction" or "proposed")
                continue;

            foreach (var id in way.NodeIds)
            {
                if (!byNode.TryGetValue(id, out var waies))
                    byNode[id] = waies = new();

                waies.Add(way);
            }
        }

        var rows = new Dictionary<string, int>();

        foreach (var section in plan)
            foreach (var (id, point) in new[] { (section.A, section.P0), (section.B, section.P3) })
            {
                if (lookup[id] != 1 || Math.Abs(point.x) > half || Math.Abs(point.z) > half)
                    continue;

                var result = "no street at node";

                if (byNode.ContainsKey(id))
                {
                    result = "streets, no road within limit";
                    var best = new Dictionary<long, double> { [id] = 0 };
                    var pq = new PriorityQueue<long, double>();
                    pq.Enqueue(id, 0);

                    while (pq.TryDequeue(out var nodeId, out var otherDistance))
                    {
                        if (otherDistance > limit)
                            break;

                        if (nodeId != id && osmPlanned.Contains(nodeId))
                        {
                            result = "streets reach a road";
                            break;
                        }

                        if (!byNode.TryGetValue(nodeId, out var ways) || !data.Nodes.TryGetValue(nodeId, out var nn))
                            continue;

                        var np = capture.Local(nn.Latitude, nn.Longitude);

                        foreach (var way in ways)
                            for (var i = 0; i < way.NodeIds.Length; i++)
                            {
                                if (way.NodeIds[i] != nodeId)
                                    continue;

                                foreach (var j in new[] { i - 1, i + 1 })
                                {
                                    if (j < 0
                                        || j >= way.NodeIds.Length
                                        || !data.Nodes.TryGetValue(way.NodeIds[j], out var mm))
                                        continue;

                                    var v =
                                        otherDistance
                                            + math.distance(np.xz, capture.Local(mm.Latitude, mm.Longitude).xz);

                                    if (!best.TryGetValue(way.NodeIds[j], out var b) || v < b)
                                    {
                                        best[way.NodeIds[j]] = v;
                                        pq.Enqueue(way.NodeIds[j], v);
                                    }
                                }
                            }
                    }
                }

                var key = $"{Hw.Group(section.Cls),-14} {(section.OneWay ? "one-way" : "two-way")} {result}";
                rows[key] = rows.GetValueOrDefault(key) + 1;
            }

        Console.WriteLine($"{map} (street limit {limit} m):");

        foreach (var entry in rows.OrderBy(otherEntry => otherEntry.Key))
            Console.WriteLine($"  {entry.Value,4} {entry.Key}");
    }

    // The game joins new nodes on exact position: do all sections at a plan node agree to the bit?
    public static void Anchors(string map)
    {
        var capture = new Cap(map);
        var first = true;
        var at = new Dictionary<string, List<(string P, long Way, string Cls)>>();

        foreach (var row in capture.ManifestJ("final-prepared-network-chunks.json"))
        {
            if (first)
            {
                Console.WriteLine(
                    "keys: " + string.Join(",", row.Properties().Select(property => property.Name)));

                first = false;
            }

            void Add(string node, string field)
            {
                var token = row[field];

                if (token == null)
                    return;

                var key = ((string?)row["kind"] ?? "") + ":" + node;

                if (!at.TryGetValue(key, out var items))
                    at[key] = items = new();

                items.Add(
                    (
                        token.ToString(Newtonsoft.Json.Formatting.None),
                        (long)row["WayId"]!,
                        (string?)row["Classification"] ?? ""));
            }

            Add((string)row["StartNodeId"]!, "nodeStart");
            Add((string)row["EndNodeId"]!, "nodeEnd");
        }

        var bad = at.Where(entry => entry.Value.Select(item => item.P).Distinct().Count() > 1).ToList();
        Console.WriteLine($"{map}: nodes {at.Count}; with differing anchors {bad.Count}");

        foreach (var entry in bad.Take(10))
            Console.WriteLine(
                $"  {entry.Key}: " + string.Join(" | ", entry.Value.Select(item => $"{item.Way} {item.P}")));
    }

    // Trunk in OSM against the plan: highway or regular road, by route.
    public static void Limited(string map)
    {
        var capture = new Cap(map);
        var data = capture.LoadOsm();
        var plan = Plan(capture).Where(section => section.Kind == "Road").ToList();

        string Tag(OsmWay way, string text) => way.Tags.TryGetValue(text, out var otherText) ? otherText : "";

        var rows = new Dictionary<string, (double High, double Reg)>();

        foreach (var section in plan)
        {
            if (!data.Ways.TryGetValue(section.Way, out var way) || Tag(way, "highway") != "trunk")
                continue;

            var key = Tag(way, "ref") != "" ? Tag(way, "ref") : Tag(way, "name") != "" ? Tag(way, "name") : "(none)";
            rows.TryGetValue(key, out var item);
            rows[key] =
                section.Cls == "trunk"
                    ? (item.High + section.ArcLength() / 1000, item.Reg)
                    : (item.High, item.Reg + section.ArcLength() / 1000);
        }

        Console.WriteLine(
            $"{map}: trunk km as highway {rows.Values.Sum(item => item.High):F0}, as regular {rows.Values.Sum(item => item.Reg):F0}");

        foreach (var entry in rows
            .OrderByDescending(otherEntry => otherEntry.Value.High + otherEntry.Value.Reg)
            .Take(26))
            Console.WriteLine($"  {entry.Key,-34} highway {entry.Value.High,6:F1}  regular {entry.Value.Reg,6:F1}");

        var only = Environment.GetEnvironmentVariable("LIMITED_REF");

        if (only != null)
            foreach (var section in plan
                .Where(
                    otherSection => data.Ways.TryGetValue(otherSection.Way, out var way)
                        && Tag(way, "highway") == "trunk"
                        && Tag(way, "ref").Contains(only))
                .OrderBy(otherSection => otherSection.P0.x)
                .Where((_, i) => i % 6 == 0))
            {
                var item = capture.ToGeographic(section.P0.x, section.P0.z);
                Console.WriteLine(
                    $"    {item.lat:F4},{item.lon:F4} {section.Cls,-8} {Tag(data.Ways[section.Way], "name")}");
            }
    }

    // Two replays of one map: which ways of a class changed section count, and by how much did they move?
    public static void Diff(string map, string dirA, string dirB, string classification)
    {
        var capture = new Cap(map);
        var firstLookup =
            Hw
                .Load(dirA)
                .Where(section => section.Kind == "Road" && section.Cls == classification)
                .GroupBy(section => section.Way)
                .ToDictionary(group => group.Key, group => group.ToList());

        var secondLookup =
            Hw
                .Load(dirB)
                .Where(section => section.Kind == "Road" && section.Cls == classification)
                .GroupBy(section => section.Way)
                .ToDictionary(group => group.Key, group => group.ToList());

        var rows =
            firstLookup.Keys
                .Union(secondLookup.Keys)
                .Select(
                    wayId => (
                        Way: wayId,
                        A: firstLookup.GetValueOrDefault(wayId)?.Count ?? 0,
                        B: secondLookup.GetValueOrDefault(wayId)?.Count ?? 0))
                .Where(item => item.A != item.B)
                .OrderByDescending(item => Math.Abs(item.A - item.B))
                .ToList();

        Console.WriteLine(
            $"{map} {classification}: ways {firstLookup.Count} / {secondLookup.Count}; sections {firstLookup.Values.Sum(sections => sections.Count)} / {secondLookup.Values.Sum(sections => sections.Count)}; ways differing {rows.Count}");

        foreach (var item in rows.Take(16))
        {
            var section = (firstLookup.GetValueOrDefault(item.Way) ?? secondLookup[item.Way])[0];
            var otherItem = capture.ToGeographic(section.P0.x, section.P0.z);
            Console.WriteLine(
                $"  way {item.Way} {item.A} vs {item.B} at {otherItem.lat:F4},{otherItem.lon:F4} width {section.Width}");
        }

        foreach (var item in rows.Take(2))
            foreach (var (label, lookup) in new[] { ("A", firstLookup), ("B", secondLookup) })
                if (lookup.TryGetValue(item.Way, out var sections))
                    Console.WriteLine(
                        $"    {item.Way} {label}: "
                            + string.Join(
                                " ",
                                sections.Select(
                                    section => $"{section.ArcLength():F0}{(section.Bridge ? "B" : section.Tunnel ? "T" : "")}/{section.P0.y:F1}")));

        var same =
            firstLookup.Keys
                .Intersect(secondLookup.Keys)
                .Where(wayId => firstLookup[wayId].Count == secondLookup[wayId].Count)
                .ToList();

        var moved =
            same
                .Select(
                    wayId => firstLookup[wayId]
                        .Zip(
                            secondLookup[wayId],
                            (firstSection, secondSection) => math.distance(firstSection.P0.xz, secondSection.P0.xz))
                        .Max())
                .OrderBy(v => v)
                .ToList();

        if (moved.Count > 0)
            Console.WriteLine(
                $"  same-count ways {moved.Count}: moved p50 {moved[moved.Count / 2]:F2} p90 {moved[moved.Count * 9 / 10]:F2} max {moved[^1]:F1} m; width changes {same.Count(wayId => firstLookup[wayId][0].Width != secondLookup[wayId][0].Width)}");
    }

    // Dead ends by distance to the map edge. The game marks a node an outside
    // connection when it lies within 0.1 m of the terrain bounds.
    public static void Border(string map)
    {
        var capture = new Cap(map);
        var plan = Plan(capture).Where(section => section.Kind == "Road").ToList();
        var half = capture.Width / 2;
        var data = capture.LoadOsm();
        var lookup = new Dictionary<long, int>();

        foreach (var section in plan)
        {
            lookup[section.A] = lookup.GetValueOrDefault(section.A) + 1;
            lookup[section.B] = lookup.GetValueOrDefault(section.B) + 1;
        }

        var bins = new[] { .1, 1, 5, 25, 100, 300, 1000, 1e9 };
        var counts = new int[bins.Length];
        var byClass = new Dictionary<string, int[]>();
        var shown = 0;

        foreach (var section in plan)
            foreach (var (id, point, otherPoint) in new[] {
                (section.A, section.P0, section.P1),
                (section.B, section.P3, section.P2)
                })
            {
                if (lookup[id] != 1)
                    continue;

                var gap = half - Math.Max(Math.Abs(point.x), Math.Abs(point.z));
                var findIndex = Array.FindIndex(bins, v => gap <= v);
                counts[findIndex]++;
                var text = Hw.Group(section.Cls);

                if (!byClass.TryGetValue(text, out var a))
                    byClass[text] = a = new int[bins.Length];

                a[findIndex]++;

                if (gap > .1 && gap <= 300 && shown++ < 14)
                {
                    var item = capture.ToGeographic(point.x, point.z);
                    var tags = data.Ways.TryGetValue(section.Way, out var way);
                    Console.WriteLine(
                        $"  {gap,6:F1} m {item.lat:F5},{item.lon:F5} {section.Cls} way {section.Way} node {id} bridge={section.Bridge} tunnel={section.Tunnel} nodeInOsm={data.Nodes.ContainsKey(id)}");
                }
            }

        Console.WriteLine(
            $"{map}: half {half}; gap to edge <=0.1/1/5/25/100/300/1000/more: {string.Join(" ", counts)}");

        foreach (var entry in byClass.OrderByDescending(otherEntry => otherEntry.Value.Sum()))
            Console.WriteLine($"  {entry.Key,-14} {string.Join(" ", entry.Value)}");

        var both =
            plan.Count(
                section => Math.Max(Math.Abs(section.P0.x), Math.Abs(section.P0.z)) >= half - .1
                    || Math.Max(Math.Abs(section.P3.x), Math.Abs(section.P3.z)) >= half - .1);

        Console.WriteLine($"  sections touching the edge {both}");
    }

    // Everything near a point, for reading one visible defect.
    public static void Site(string map, double lat, double lon, double radius)
    {
        var capture = new Cap(map);
        var data = capture.LoadOsm();
        var otherPoint = capture.Local(lat, lon);
        var after = Observed(capture);
        var before = capture.Heights("offline-design-pass-01-heightmap.u16");
        var near =
            Plan(capture)
                .Where(
                    section => section.Kind == "Road"
                        && math.distance(section.At(.5).xz, new double2(otherPoint.x, otherPoint.z)) < radius)
                .OrderBy(section => section.Way)
                .ToList();

        Console.WriteLine(
            $"{map} {lat},{lon} r={radius}: {near.Count} sections; bridge {near.Count(section => section.Bridge)} tunnel {near.Count(section => section.Tunnel)}; <20 m {near.Count(section => section.ArcLength() < 20)}");

        foreach (var section in near)
        {
            var m = section.At(.5);

            string Tag(string text)
                => data.Ways.TryGetValue(section.Way, out var way) && way.Tags.TryGetValue(text, out var otherText)
                    ? otherText
                    : "-";

            Console.WriteLine(
                $" {section.Way,11} {section.Cls,-13} L{section.ArcLength(),5:F0} y {section.P0.y,6:F1}>{section.P3.y,6:F1} grnd {capture.HeightAt(after, m.x, m.z),6:F1} was {capture.HeightAt(before, m.x, m.z),6:F1} L{section.Layer,2} {(section.Bridge ? "BR" : section.Tunnel ? "TU" : "  ")} osm:t={Tag("tunnel")},b={Tag("bridge")},l={Tag("layer")} {Tag("name")}");
        }
    }
}
