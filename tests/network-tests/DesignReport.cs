using System.IO.Compression;
using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Newtonsoft.Json.Linq;
using Unity.Mathematics;

// What a road replay's design asks of the game, from its plot file: how far decks stand above
// the ground, which decides pillar height, and what lies behind sharp generated corners.
static partial class VisualMetricsReplay
{
    private sealed record PlotSection(
        long Way,
        long Start,
        long End,
        string Kind,
        string Class,
        bool Bridge,
        bool Tunnel,
        bool OneWay,
        double3 P0,
        double3 P1,
        double3 P2,
        double3 P3,
        int SourceWays);

    private static List<PlotSection> ReadPlot(string file)
    {
        var sections = new List<PlotSection>();
        using var stream = File.OpenRead(file);
        using var unzip = new GZipStream(stream, CompressionMode.Decompress);
        using var reader = new StreamReader(unzip);

        while (reader.ReadLine() is { } line)
        {
            var row = JObject.Parse(line);
            var p = row["p"]!.ToObject<double[]>()!;

            if (p.Length != 12)
                continue;

            sections.Add(
                new(
                    (long)row["WayId"]!,
                    (long)row["StartNodeId"]!,
                    (long)row["EndNodeId"]!,
                    (string)row["kind"]!,
                    (string)row["Classification"]!,
                    (bool)row["Bridge"]!,
                    (bool)row["Tunnel"]!,
                    (bool)row["OneWay"]!,
                    new(p[0], p[1], p[2]),
                    new(p[3], p[4], p[5]),
                    new(p[6], p[7], p[8]),
                    new(p[9], p[10], p[11]),
                    ((JArray?)row["SourceWayIds"])?.Count ?? 0));
        }

        return sections;
    }

    private static double3 At(PlotSection s, double t)
    {
        var u = 1 - t;

        return u * u * u * s.P0 + 3 * u * u * t * s.P1 + 3 * u * t * t * s.P2 + t * t * t * s.P3;
    }

    // The heightmap's spread, and cells far from their neighbourhood. Heights are in metres
    // above the map's floor; a spike is over 150 m from the median of the cells 2 to 8 away.
    public static void TerrainReport(string folder)
    {
        var capture = Load(folder);
        var size = (int)Math.Sqrt(capture.Terrain.Length);
        var ordered = capture.Terrain.OrderBy(value => value).ToArray();
        float Percentile(double share) => ordered[(int)Math.Min(ordered.Length - 1, ordered.Length * share)];
        var spikesLow = 0;
        var spikesHigh = 0;
        var around = new float[8];
        var offsets = new[] { (-8, 0), (8, 0), (0, -8), (0, 8), (-3, -3), (3, 3), (-3, 3), (3, -3) };

        for (var z = 8; z < size - 8; z += 2)
            for (var x = 8; x < size - 8; x += 2)
            {
                for (var k = 0; k < 8; k++)
                    around[k] = capture.Terrain[(z + offsets[k].Item2) * size + x + offsets[k].Item1];

                Array.Sort(around);
                var difference = capture.Terrain[z * size + x] - (around[3] + around[4]) / 2f;

                if (difference < -150)
                    spikesLow++;
                else if (difference > 150)
                    spikesHigh++;
            }

        Console.WriteLine(
            $"scale {capture.Scale:F0} | min {ordered[0]:F0} p0.01 {Percentile(.0001):F0} p1 {Percentile(.01):F0} p50 {Percentile(.5):F0} p99 {Percentile(.99):F0} p99.99 {Percentile(.9999):F0} max {ordered[^1]:F0} | sampled cells low {spikesLow} high {spikesHigh}");
    }

    // Tree selection on a capture's roads and terrain and its cached forest data: the time, and
    // a hash of the trees, so a speedup can be shown to leave them unchanged.
    public static void ForestProfile(string folder, string parsedCache)
    {
        var capture = Load(folder);
        var request = capture.Input["request"]!.ToObject<ImportRequest>()!;
        var half = (float)(request.MapSize.WidthMeters / 2);
        OsmDataset? data = null;
        var bestScore = 0;

        foreach (var file in Directory.GetFiles(parsedCache, "*.bin"))
        {
            OsmDataset candidate;

            try
            {
                candidate = OsmParsedCache.LoadSnapshot(file, default);
            }
            catch (Exception error) when (error is InvalidDataException or EndOfStreamException or IOException)
            {
                continue;
            }

            var woods = candidate.Ways.Values.Count(way => way.Tags.ContainsKey("natural") || way.Tags.ContainsKey("landuse"));

            if (woods <= bestScore)
                continue;

            var near =
                candidate.Nodes.Values.Take(5000).Count(
                    node =>
                    {
                        var point = ImportEarthLocation.Geo.GeoMath.ToLocalMeters(node.Latitude, node.Longitude, request.Latitude, request.Longitude);

                        return Math.Abs(point.x) < half && Math.Abs(point.z) < half;
                    });

            if (near > 2500)
            {
                data = candidate;
                bestScore = woods;
            }
        }

        if (data == null)
            throw new InvalidDataException("No cached forest data for this map");

        foreach (var relation in data.Relations
                     .Where(relation => ForestProcessor.Kind(relation.Tags) == ForestProcessor.AreaKind.Woodland)
                     .OrderByDescending(relation => relation.Members.Count)
                     .Take(5))
        {
            var inner = relation.Members.Count(member => member.inner);
            var nodes =
                relation.Members.Sum(
                    member => data.Ways.TryGetValue(member.wayId, out var way) ? way.NodeIds.Length : 0);
            Console.WriteLine($"woodland relation {relation.Id}: members {relation.Members.Count}, inner {inner}, nodes {nodes}");
        }

        var log = new ImportLog();
        var clock = System.Diagnostics.Stopwatch.StartNew();
        var trees =
            ForestProcessor.Prepare(
                data,
                request,
                new(capture.Terrain, capture.Scale, 308, 0, capture.Scale),
                capture.Source,
                log,
                default,
                excludeWater: point => capture.Water(point) is float level && point.y <= level + .5f);

        using var sha = System.Security.Cryptography.SHA256.Create();
        var bytes = new byte[trees.Count * 12];
        Buffer.BlockCopy(trees.SelectMany(tree => new[] { tree.x, tree.y, tree.z }).ToArray(), 0, bytes, 0, bytes.Length);
        Console.WriteLine(
            $"{clock.Elapsed.TotalSeconds:F1} s; forest ways {bestScore}; trees {trees.Count}; hash {Convert.ToHexString(sha.ComputeHash(bytes))[..16]}");

        // Vegetation planning on those trees with the capture's water. The capture has no
        // climate and only the water polygons, so the plan is not the game's, but the work
        // per tree and per bank sample is the same.
        T[] Read<T>(string name) => CorrectiveCapturedReplay.ReadManifest<T>(folder, (string)capture.Input[name]!);

        var water =
            new WaterPlan(false, 0, 0, 0, "captured")
            {
                WaterCellSize = CapturedRunFiles.WaterCellSize(folder, capture.Input),
                AreaStages = Read<WaterAreaStage>("waterAreasManifest"),
                ReachStages = Read<WaterReachStage>("waterReachesManifest")
            };

        clock.Restart();
        var plan =
            VegetationPlanner.Prepare(
                data,
                trees,
                request,
                new(capture.Terrain, capture.Scale, 308, 0, capture.Scale),
                capture.Source,
                Read<GeographicPolygon>("waterPolygonsManifest"),
                water,
                null,
                log,
                default,
                capture.Water);

        var planSeconds = clock.Elapsed.TotalSeconds;
        var values = new float[plan.Count * 5];
        var next = 0;

        foreach (var candidate in plan)
        {
            values[next++] = candidate.Position.x;
            values[next++] = candidate.Position.y;
            values[next++] = candidate.Position.z;
            values[next++] = (float)candidate.Form;
            values[next++] = candidate.Seed;
        }

        var planBytes = new byte[values.Length * 4];
        Buffer.BlockCopy(values, 0, planBytes, 0, planBytes.Length);
        Console.WriteLine(
            $"vegetation plan {planSeconds:F1} s; candidates {plan.Count}; hash {Convert.ToHexString(sha.ComputeHash(planBytes))[..16]}");

        foreach (var line in log.Lines.Where(line => line.Contains("VEGETATION PLAN TIMES")))
            Console.WriteLine(line);
    }

    public static void DesignReport(string folder, string plotFile)
    {
        var capture = Load(folder);
        var sections = ReadPlot(plotFile);
        var bands = new[] { 10, 20, 30, 50, 75, 100, 150 };
        var above = new double[bands.Length];
        var worst = new List<(double Height, PlotSection Section, double3 Point)>();
        var bridgeMeters = 0d;

        foreach (var section in sections.Where(section => section.Bridge && section.Kind == "Road"))
        {
            var length = math.distance(section.P0.xz, section.P3.xz);
            var steps = Math.Max(2, (int)(length / 10));
            var highest = 0d;
            var at = section.P0;

            for (var k = 0; k <= steps; k++)
            {
                var point = At(section, k / (double)steps);
                var ground = NetworkProcessor.Sample((float)point.x, (float)point.z, capture.Terrain, capture.Width);
                var height = point.y - ground;

                for (var b = 0; b < bands.Length; b++)
                    if (height > bands[b])
                        above[b] += length / (steps + 1);

                if (height > highest)
                {
                    highest = height;
                    at = point;
                }
            }

            bridgeMeters += length;
            worst.Add((highest, section, at));
        }

        Console.WriteLine($"road bridge km {bridgeMeters / 1000:F1}; deck over ground km: " + string.Join(" ", bands.Select((band, b) => $">{band}m={above[b] / 1000:F2}")));

        foreach (var item in worst.OrderByDescending(item => item.Height).Take(6))
            Console.WriteLine($"  tall {item.Height:F0} m {item.Section.Class} way {item.Section.Way} at {item.Point.x:F0},{item.Point.z:F0}");

        var size = (int)Math.Sqrt(capture.Terrain.Length);
        var ordered = capture.Terrain.OrderBy(value => value).ToArray();
        var cell = capture.Width / (size - 1);
        Console.WriteLine(
            $"terrain {size}x{size}, {cell:F1} m cells, scale {capture.Scale:F0} m: min {ordered[0]:F0} p0.1 {ordered[ordered.Length / 1000]:F0} median {ordered[ordered.Length / 2]:F0} max {ordered[^1]:F0}; zero cells {ordered.Count(value => value == 0)}");

        foreach (var item in worst.OrderByDescending(item => item.Height).Take(2))
        {
            var point = item.Point;
            Console.WriteLine($"  deck y {point.y:F0}; section ends y {item.Section.P0.y:F0} {item.Section.P3.y:F0}; ground along x:");

            foreach (var distance in new[] { -400, -200, -100, -50, -25, 0, 25, 50, 100, 200, 400 })
                Console.Write($" {NetworkProcessor.Sample((float)point.x + distance, (float)point.z, capture.Terrain, capture.Width):F0}");

            Console.WriteLine();
        }

        var arms = new Dictionary<long, List<(PlotSection Section, bool Start)>>();

        foreach (var section in sections.Where(section => section.Kind == "Road"))
            foreach (var (node, start) in new[] { (section.Start, true), (section.End, false) })
            {
                if (!arms.TryGetValue(node, out var list))
                    arms[node] = list = new();

                list.Add((section, start));
            }

        var kinds = new Dictionary<string, (int Count, List<double> Angles, List<double> Short)>();

        foreach (var (node, list) in arms)
        {
            if (list.Count != 2 || node > long.MinValue + 9000000)
                continue;

            static double3 Away((PlotSection Section, bool Start) arm)
            {
                var s = arm.Section;
                var d = arm.Start ? s.P1 - s.P0 : s.P2 - s.P3;

                if (math.lengthsq(d.xz) < 1e-6)
                    d = arm.Start ? s.P2 - s.P0 : s.P1 - s.P3;

                return d;
            }

            var a = Away(list[0]);
            var b = Away(list[1]);

            if (math.lengthsq(a.xz) < 1e-6 || math.lengthsq(b.xz) < 1e-6)
                continue;

            var deflection = math.degrees(Math.Acos(math.clamp(-math.dot(math.normalize(a.xz), math.normalize(b.xz)), -1, 1)));

            if (deflection <= 2)
                continue;

            var lengths = list.Select(arm => math.distance(arm.Section.P0.xz, arm.Section.P3.xz)).ToArray();
            var chordA = math.normalizesafe(list[0].Start ? list[0].Section.P3.xz - list[0].Section.P0.xz : list[0].Section.P0.xz - list[0].Section.P3.xz);
            var chordB = math.normalizesafe(list[1].Start ? list[1].Section.P3.xz - list[1].Section.P0.xz : list[1].Section.P0.xz - list[1].Section.P3.xz);
            var chordDeflection = math.degrees(Math.Acos(math.clamp(-math.dot(chordA, chordB), -1, 1)));
            var sameWay = list[0].Section.Way == list[1].Section.Way;
            var key =
                (sameWay ? "same-way" : "two-ways")
                + (list[0].Section.Bridge != list[1].Section.Bridge || list[0].Section.Tunnel != list[1].Section.Tunnel ? " structure-end" : "")
                + (list[0].Section.OneWay != list[1].Section.OneWay ? " oneway-change" : "")
                + (chordDeflection < 30 && deflection > 45 ? " controls-disagree" : chordDeflection >= 45 ? " real-corner" : "");

            if (!kinds.TryGetValue(key, out var entry))
                kinds[key] = entry = (0, new(), new());

            entry.Count++;
            entry.Angles.Add(deflection);
            entry.Short.Add(lengths.Min());
            kinds[key] = entry;
        }

        Console.WriteLine("generated corners:");

        foreach (var kind in kinds.OrderByDescending(kind => kind.Value.Count))
        {
            var angles = kind.Value.Angles.OrderBy(v => v).ToArray();
            var shorts = kind.Value.Short.OrderBy(v => v).ToArray();
            Console.WriteLine($"  {kind.Key}: {kind.Value.Count}; median angle {angles[angles.Length / 2]:F0}; median shorter arm {shorts[shorts.Length / 2]:F0} m");
        }
    }
}
