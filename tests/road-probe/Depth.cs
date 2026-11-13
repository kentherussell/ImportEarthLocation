// How far plan roads sit below the original ground, and what OSM says about them there.
// Cover is original terrain minus deck height at a section's middle, in metres.
public static class Depth
{
    static string Band(double cover)
        => cover < 2 ? "<2" : cover < 4 ? "2-4" : cover < 7 ? "4-7" : cover < 11 ? "7-11" : cover < 16 ? "11-16" : "16+";

    public static void Run(string map)
    {
        var capture = new Cap(map);
        var data = capture.LoadOsm();
        var before = capture.Heights("offline-design-pass-01-heightmap.u16");
        var plan = Compare.Final(capture).Where(section => section.Kind == "Road").ToList();
        var byNode = new Dictionary<long, List<Sec>>();

        foreach (var section in plan)
            foreach (var node in new[] { section.A, section.B })
            {
                if (!byNode.TryGetValue(node, out var list))
                    byNode[node] = list = new();

                list.Add(section);
            }

        bool Tagged(Sec section, string key)
            => section.Ways.Append(section.Way).Any(
                way => data.Ways.TryGetValue(way, out var found)
                    && found.Tags.TryGetValue(key, out var value)
                    && value != "no");

        double Cover(Sec section)
        {
            var middle = section.At(.5);
            return capture.HeightAt(before, middle.x, middle.z) - middle.y;
        }

        // Walk outward through the plan until an OSM bridge or tunnel, up to a limit.
        double Distance(Sec start, Func<Sec, bool> goal)
        {
            var seen = new HashSet<Sec> { start };
            var frontier = new PriorityQueue<Sec, double>();
            frontier.Enqueue(start, 0);

            while (frontier.TryDequeue(out var section, out var walked))
            {
                if (goal(section))
                    return walked;

                if (walked > 1500)
                    break;

                foreach (var node in new[] { section.A, section.B })
                    foreach (var next in byNode[node])
                        if (seen.Add(next))
                            frontier.Enqueue(next, walked + next.ArcLength());
            }

            return double.PositiveInfinity;
        }

        var rows =
            plan.Select(
                section => (
                    Section: section,
                    Cover: Cover(section),
                    Length: section.ArcLength(),
                    OsmTunnel: Tagged(section, "tunnel"),
                    OsmBridge: Tagged(section, "bridge"),
                    Cutting: Tagged(section, "cutting"),
                    Under: Tagged(section, "covered")))
                .ToList();

        Console.WriteLine($"{map}: road km by plan type and original cover (ground above deck, m)");

        foreach (var group in rows.GroupBy(row => (row.Section.Tunnel, row.OsmTunnel)).OrderBy(group => group.Key.ToString()))
            Console.WriteLine(
                $"  plan tunnel={group.Key.Tunnel,-5} OSM tunnel={group.Key.OsmTunnel,-5}: "
                    + string.Join(
                        "  ",
                        group.GroupBy(row => Band(row.Cover))
                            .OrderBy(band => band.Key)
                            .Select(band => $"{band.Key}: {band.Sum(row => row.Length) / 1000:F1}")));

        var invented = rows.Where(row => row.Section.Tunnel && !row.OsmTunnel).ToList();
        Console.WriteLine($"tunnels without an OSM tunnel tag: {invented.Count} sections, {invented.Sum(row => row.Length) / 1000:F1} km");

        foreach (var group in invented.GroupBy(row => row.Section.Cls).OrderByDescending(group => group.Sum(row => row.Length)))
            Console.WriteLine($"  {group.Key,-14} {group.Sum(row => row.Length) / 1000,6:F1} km");

        var near = invented.Select(row => Distance(row.Section, section => Tagged(section, "tunnel"))).ToList();
        var nearBridge = invented.Select(row => Distance(row.Section, section => Tagged(section, "bridge"))).ToList();
        string Share(List<double> list, double limit) => $"{100.0 * list.Count(v => v <= limit) / Math.Max(1, list.Count):F0}%";
        Console.WriteLine($"  within road distance of an OSM tunnel: 200 m {Share(near, 200)}, 500 m {Share(near, 500)}, 1500 m {Share(near, 1500)}");
        Console.WriteLine($"  within road distance of an OSM bridge: 200 m {Share(nearBridge, 200)}, 500 m {Share(nearBridge, 500)}, 1500 m {Share(nearBridge, 1500)}");

        // Each run of connected OSM-tagged tunnel, and the untagged tunnel grown from it.
        var tagged = rows.Where(row => row.Section.Tunnel && row.OsmTunnel).Select(row => row.Section).ToHashSet();
        var grown = invented.Select(row => row.Section).ToHashSet();
        var assigned = new HashSet<Sec>();
        var runs = new List<(double Real, double Grown, Sec Seed)>();

        foreach (var seed in tagged)
        {
            if (!assigned.Add(seed))
                continue;

            var stack = new Stack<Sec>();
            stack.Push(seed);
            double real = 0, extra = 0;

            while (stack.Count > 0)
            {
                var section = stack.Pop();

                if (tagged.Contains(section))
                    real += section.ArcLength();
                else
                    extra += section.ArcLength();

                foreach (var node in new[] { section.A, section.B })
                    foreach (var next in byNode[node])
                        if ((tagged.Contains(next) || grown.Contains(next)) && assigned.Add(next))
                            stack.Push(next);
            }

            runs.Add((real, extra, seed));
        }

        Console.WriteLine("connected tunnel systems, by length of real OSM tunnel in them:");

        foreach (var (low, high) in new[] { (0d, 60d), (60d, 150d), (150d, 300d), (300d, 800d), (800d, 1e9) })
        {
            var mine = runs.Where(run => run.Real >= low && run.Real < high).ToList();
            Console.WriteLine($"  OSM {low,4:F0}-{(high > 1e8 ? 9999 : high),4:F0} m: {mine.Count,3} systems, real {mine.Sum(run => run.Real) / 1000,5:F1} km, grown {mine.Sum(run => run.Grown) / 1000,5:F1} km");
        }

        foreach (var run in runs.OrderByDescending(run => run.Grown / Math.Max(50, run.Real)).Take(10))
        {
            var middle = run.Seed.At(.5);
            var place = capture.ToGeographic(middle.x, middle.z);
            var name = data.Ways.TryGetValue(run.Seed.Way, out var found) ? (found.Tags.TryGetValue("name", out var n) ? n : found.Tags.TryGetValue("ref", out var r) ? r : "") : "";
            Console.WriteLine($"  real {run.Real,5:F0} m grew {run.Grown,5:F0} m {run.Seed.Cls,-13} {place.lat:F4},{place.lon:F4} {name}");
        }

        // A deep ground road: is the road low, or the ground high? Compare the ground's
        // relief around it with the deck's own slope.
        var clusters = new Dictionary<(int, int), List<(Sec Section, double Cover, double Length)>>();

        foreach (var row in invented)
        {
            var middle = row.Section.At(.5);
            var key = ((int)Math.Floor(middle.x / 500), (int)Math.Floor(middle.z / 500));

            if (!clusters.TryGetValue(key, out var list))
                clusters[key] = list = new();

            list.Add((row.Section, row.Cover, row.Length));
        }

        Console.WriteLine("largest clusters of such tunnels (500 m cells):");

        foreach (var cluster in clusters.OrderByDescending(entry => entry.Value.Sum(item => item.Length)).Take(14))
        {
            var middle = cluster.Value[0].Section.At(.5);
            var place = capture.ToGeographic(middle.x, middle.z);
            var ways = cluster.Value.SelectMany(item => item.Section.Ways.Append(item.Section.Way)).Distinct().ToArray();
            var layers = ways.Select(way => data.Ways.TryGetValue(way, out var found) && found.Tags.TryGetValue("layer", out var value) ? value : "-").Distinct();
            var cuts = ways.Count(way => data.Ways.TryGetValue(way, out var found) && found.Tags.ContainsKey("cutting"));
            var names = ways.Select(way => data.Ways.TryGetValue(way, out var found) ? (found.Tags.TryGetValue("ref", out var r) ? r : found.Tags.TryGetValue("name", out var n) ? n : "") : "").Where(text => text.Length > 0).Distinct().Take(3);
            Console.WriteLine(
                $"  {place.lat:F4},{place.lon:F4} {cluster.Value.Sum(item => item.Length),5:F0} m cover {cluster.Value.Min(item => item.Cover):F0}..{cluster.Value.Max(item => item.Cover):F0} deckY {cluster.Value.Min(item => item.Section.At(.5).y):F0}..{cluster.Value.Max(item => item.Section.At(.5).y):F0} layers {string.Join(",", layers)} cuttingWays {cuts}/{ways.Length} {string.Join("; ", names)}");
        }
    }
}
