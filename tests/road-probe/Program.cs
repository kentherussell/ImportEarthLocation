using System.Globalization;

CultureInfo.DefaultThreadCurrentCulture =
    CultureInfo.InvariantCulture; CultureInfo.CurrentCulture = CultureInfo.InvariantCulture;
var cmd = args[0];
switch (cmd)
{
    case "pass":
        Live.Cuts(args[1]);
        Live.Walls(args[1]);
        Live.Splits(args[1]);
        Live.Rivers(args[1]);
        Live.Shots(args[1]);
        Triage.Ends(args[1]);
        Triage.Border(args[1]);
        break;

    case "rings":
        Rings.Run(args[1]);
        break;

    case "patterns":
        Patterns.Run(args[1]);
        break;

    case "depth":
        Depth.Run(args[1]);
        break;

    case "straight":
        Straight.Run(args[1]);
        break;

    case "shape":
        Shape.Run(args[1]);
        break;

    case "liveflags":
        Live.Flags(args[1]);
        break;

    case "asked":
        Live.Asked(args[1]);
        break;

    case "reaches":
        Live.Reaches(args[1], args.Length > 2 ? args[2] : null);
        break;

    case "cuts":
        Live.Cuts(args[1]);
        break;

    case "rivers":
        Live.Rivers(args[1]);
        break;

    case "splits":
        Live.Splits(args[1]);
        break;

    case "walls":
        Live.Walls(args[1]);
        break;

    case "livesite":
        Live.Site(args[1], double.Parse(args[2]), double.Parse(args[3]), double.Parse(args[4]));
        break;

    case "find":
        Live.Find(args[1], args[2], args.Length > 3 ? args[3] : "");
        break;

    case "sand":
        Triage.Sand(args[1]);
        break;

    case "ends":
        Triage.Ends(args[1]);
        break;

    case "gaps":
        Gaps.Run(args[1], args.Length > 2 ? int.Parse(args[2]) : 8);
        break;

    case "roundabouts":
        Roundabouts.Run(args[1], args.Length > 2 ? int.Parse(args[2]) : 5);
        break;

    case "pairs":
        Pairs.Run(args[1], args.Length > 2 ? int.Parse(args[2]) : 4);
        break;

    case "shoreline":
        Shoreline.Run(args[1], double.Parse(args[2]), args.Length > 3 ? int.Parse(args[3]) : 4);
        break;

    case "spurs":
        Spurs.Run(args[1], args.Length > 2 ? int.Parse(args[2]) : 6);
        break;

    case "railkinks":
        RailKinks.Run(args[1], args.Length > 2 ? int.Parse(args[2]) : 6);
        break;

    case "batch":
        if (args.Length > 4)
            Batch.Changed(args[1], int.Parse(args[2]), int.Parse(args[3]), args[4]);
        else
            Batch.Run(args[1], int.Parse(args[2]), int.Parse(args[3]));

        break;

    case "doubles":
        Batch.Doubles(args[1], args.Length > 2 ? double.Parse(args[2]) : 3);
        break;

    case "crossings":
        Crossings.Run(args[1], int.Parse(args[2]), int.Parse(args[3]), args.Length > 4 ? double.Parse(args[4]) : 4);
        break;

    case "widths":
        Widths.Run(args[1], args.Length > 2 ? int.Parse(args[2]) : 4);
        break;

    case "structures":
        Structures.Run(args[1], args.Length > 2 ? int.Parse(args[2]) : 4);
        break;

    case "nearplan":
        Gaps.Near(args[1], double.Parse(args[2]), double.Parse(args[3]), double.Parse(args[4]), false);
        break;

    case "osmnode":
        Gaps.OsmNode(args[1], args.Skip(2).Select(long.Parse));
        break;

    case "nearsource":
        Gaps.Near(args[1], double.Parse(args[2]), double.Parse(args[3]), double.Parse(args[4]), true);
        break;

    case "diff":
        Triage.Diff(args[1], args[2], args[3], args[4]);
        break;

    case "limited":
        Triage.Limited(args[1]);
        break;

    case "anchors":
        Triage.Anchors(args[1]);
        break;

    case "reach":
        Triage.Reach(args[1], double.Parse(args[2]));
        break;

    case "border":
        Triage.Border(args[1]);
        break;

    case "ways":
        Triage.Ways(
            args[1],
            args[2],
            double.Parse(args[3]),
            double.Parse(args[4]),
            double.Parse(args[5]),
            double.Parse(args[6]));

        break;

    case "trunks":
        Triage.Trunks(args[1]);
        break;

    case "pieces":
        Triage.Pieces(args[1]);
        break;

    case "endorigin":
        Triage.EndOrigin(args[1]);
        break;

    case "tunnels":
        Triage.Tunnels(args[1]);
        break;

    case "earth":
        Triage.Earth(args[1]);
        break;

    case "grades":
        Triage.Grades(args[1]);
        break;

    case "site":
        Triage.Site(args[1], double.Parse(args[2]), double.Parse(args[3]), double.Parse(args[4]));
        break;

    case "mouth":
        Mouth(args[1], double.Parse(args[2]), double.Parse(args[3]));
        break;

    case "hw":
        Hw.Run(args[1], args[2]);
        break;

    case "plot":
        Plot.Run(
            args[1],
            args[2],
            double.Parse(args[3]),
            double.Parse(args[4]),
            double.Parse(args[5]),
            args[6],
            args.Length > 7 ? args[7] : "");

        break;

    case "water":
        Waters.Stats(args[1]);
        break;

    case "shoreprofile":
        Shore.Run(args[1], float.Parse(args[2]));
        break;

    case "coverage":
        Misc.Coverage(args[1], args[2]);
        break;

    case "fit":
        Fit.Run(args[1]);
        break;

    case "streetfit":
        StreetFit.Run(
            args[1],
            args.Length > 2 ? double.Parse(args[2]) : 100,
            args.Length > 3 ? double.Parse(args[3]) : 1.5);
        break;

    case "bridges":
        Misc.Bridges(args[1], args[2]);
        break;

    case "lakes":
        Misc.Lakes(args[1]);
        break;

    case "topo":
        Misc.Topo(args[1]);
        break;

    case "near":
        Compare.Near(args[1], args[2], double.Parse(args[3]), double.Parse(args[4]), double.Parse(args[5]));
        break;

    case "native":
        Native.Run(args[1], args[2]);
        break;

    case "cmp":
        Compare.Run(
            args[1],
            args[2],
            double.Parse(args[3]),
            double.Parse(args[4]),
            double.Parse(args[5]),
            double.Parse(args[6]));

        break;

    case "gitdiff":
        GitRead.Diff(args[1], args[2], args[3]);
        break;

    case "gitexport":
        GitRead.Export(args[1], args[2], args[3]);
        break;

    case "probe":
        Waters.Probe(
            args[1],
            double.Parse(args[2]),
            double.Parse(args[3]),
            double.Parse(args[4]),
            double.Parse(args[5]),
            double.Parse(args[6]));

        break;
}

// Minimax path from a basin to the open sea on the final terrain.
static void Mouth(string map, double lat, double lon)
{
    var capture = new Cap(map);
    var otherPoint = capture.Local(lat, lon);
    var t = capture.Heights("native-observed-heightmap.u16");
    var n = (int)Math.Sqrt(t.Length);
    var step = (float)capture.Width / (n - 1);

    int GridIndex(double v) => (int)Math.Round((v / capture.Width + .5) * (n - 1));

    float HeightAt(int i) => t[i];

    var start = GridIndex(otherPoint.z) * n + GridIndex(otherPoint.x);
    var best = new float[t.Length];
    Array.Fill(best, float.PositiveInfinity);
    var parent = new int[t.Length];
    var pq = new PriorityQueue<int, float>();
    best[start] = HeightAt(start);
    parent[start] = -1;
    pq.Enqueue(start, best[start]);
    var target = -1;
    var deepest = 304f; // deep water: modelled ocean is at least 4 m below nominal

    while (pq.TryDequeue(out var i, out var cost))
    {
        if (cost > best[i])
            continue;

        if (HeightAt(i) < deepest)
        {
            target = i;
            break;
        }

        var x = i % n;
        var z = i / n;

        for (var dz = -1; dz <= 1; dz++)
            for (var dx = -1; dx <= 1; dx++)
            {
                if (dx == 0 && dz == 0 || x + dx < 0 || z + dz < 0 || x + dx >= n || z + dz >= n)
                    continue;

                var k = (z + dz) * n + x + dx;
                var v = Math.Max(cost, HeightAt(k));

                if (v < best[k])
                {
                    best[k] = v;
                    parent[k] = i;
                    pq.Enqueue(k, v);
                }
            }
    }

    Console.WriteLine($"start height {HeightAt(start):F2}; reached deep water at cost (saddle) {best[target]:F2} m");
    var path = new List<int>();

    for (var i = target; i >= 0; i = parent[i])
        path.Add(i);

    path.Reverse();
    Console.WriteLine($"path cells {path.Count}; length {path.Count * step:F0} m");

    foreach (var i in path.Where((_, k) => k % 6 == 0))
    {
        var x = (i % n) * step - (float)capture.Width / 2;
        var z = (i / n) * step - (float)capture.Width / 2;
        var item = capture.ToGeographic(x, z);
        Console.WriteLine($"  {item.lat:F5},{item.lon:F5} h={HeightAt(i):F2}");
    }

    var otherCount = best.Select((v, i) => (v, i)).Where(item => item.v < 309).Count();
    // Area flooded at the sea surface if connected
    var seen = new bool[t.Length];
    var q = new Queue<int>();
    q.Enqueue(start);
    seen[start] = true;
    var count = 0;
    var level = 309f;

    while (q.Count > 0)
    {
        var i = q.Dequeue();
        count++;
        var x = i % n;
        var z = i / n;

        foreach (var k in new[] {
            x > 0 ? i - 1 : -1,
            x + 1 < n ? i + 1 : -1,
            z > 0 ? i - n : -1,
            z + 1 < n ? i + n : -1
            })
            if (k >= 0 && !seen[k] && HeightAt(k) < level)
            {
                seen[k] = true;
                q.Enqueue(k);
            }
    }

    Console.WriteLine($"cells connected below {level} m from the basin: {count}");
    // Water cell grid view at 28 m: is the channel passable on the coarse grid?
    var w2 = 28f;
    Console.WriteLine("terrain around the path, every 28 m, in the mouth:");
}
