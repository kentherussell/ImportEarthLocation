using ImportEarthLocation.Core;
using ImportEarthLocation.Geo;
using ImportEarthLocation.Processing;
using System.Globalization;
using System.IO.Compression;
using System.Text;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using Unity.Mathematics;

// Report 64 Phases 1–2: visual metrics on a design's final output, after the same
// native units and roadbed the game would receive. The failures file lets
// clustering be revised without repeating the design; the plot file lets two
// pipelines be drawn side by side at fixed locations.
static partial class VisualMetricsReplay
{
    private const string FailuresFile = "visual-failures.ndjson.gz";
    private const string RunFile = "visual-metrics-run.json";
    private const string PlotFile = "plot-sections.ndjson.gz";
    private const double Earth = 6378137;

    private sealed record Capture(
        string Folder,
        JObject Input,
        PreparedSegment[] Source,
        NativeAssetContract[] Contracts,
        Func<float3, float?> Water,
        float3[][] Areas, float3[][] Lines, float[] Terrain, float Scale, double Width);

    private static Capture Load(string folder)
    {
        var input =
            JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder, "offline-design-pass-01-input.json")));

        if ((int?)input["schemaVersion"] != 2 || (string?)input["geometryEncoding"] != "prepared-segment-exact-v1"
            || (string?)input["waterSampler"] is not ("WaterBodyPlanner.SurfaceSampler" or "WaterBodyPlanner.ConstructedSurfaceSampler"))
            throw new InvalidDataException("Unknown full-context replay contract");

        T[] Read<T>(string name) => CorrectiveCapturedReplay.ReadManifest<T>(folder, (string)input[name]!);

        var source = Read<PreparedSegment>("networkManifest");

        if (source.Length != (int)input["sections"]!)
            throw new InvalidDataException("Source count mismatch");

        var cell = CapturedRunFiles.WaterCellSize(folder, input);
        var water =
            new WaterPlan(
                false,
                0,
                0,
                0,
                "captured")
            {
                WaterCellSize = cell,
                AreaStages = Read<WaterAreaStage>("waterAreasManifest"),
                ReachStages = Read<WaterReachStage>("waterReachesManifest")
            };

        var sampler =
            WaterBodyPlanner.ConstructedSurfaceSampler(water, Read<GeographicPolygon>("waterPolygonsManifest"));

        var scale = (float)input["HeightScaleMeters"]!;
        var terrain =
            CapturedRunFiles.Heights(CapturedRunFiles.Resolve(folder, (string)input["terrainFile"]!), scale);

        TerrainSampling.Resolution(terrain);

        return new(
            folder,
            input,
            source,
            Read<NativeAssetContract>("nativeContractsManifest"),
            sampler,
            Read<float3[]>("protectedAreasManifest"),
            Read<float3[]>("protectedLinesManifest"),
                terrain,
                scale,
                (double)input["widthMeters"]!);
    }

    // Lane-count smoothing on a captured network: the summary, and the lane
    // sequence of the named road's sections in travel order before and after.
    public static void LaneRuns(string folder, string name)
    {
        var capture = Load(folder);
        var log = new ImportLog();
        var smoothed = NetworkLaneRuns.Smooth(capture.Source, log);

        foreach (var line in log.Lines.Where(text => text.Contains("Lane counts:", StringComparison.Ordinal)))
            Console.WriteLine("  " + line);

        foreach (var group in Enumerable
            .Range(0, capture.Source.Length)
            .Where(i => capture.Source[i].Name == name)
            .GroupBy(
                i => capture.Source[i].OneWay ? Math.Sign(capture.Source[i].End.x - capture.Source[i].Start.x) : 0))
        {
            var order = group.OrderBy(i => capture.Source[i].Start.x * (group.Key < 0 ? -1 : 1)).ToArray();
            Console.WriteLine(
                $"  direction {group.Key}: before {string.Join("", order.Select(i => capture.Source[i].Lanes))}");

            Console.WriteLine(
                $"  direction {group.Key}: after  {string.Join("", order.Select(i => smoothed[i].Lanes))}");
        }
    }

    // Every plan crossing between a section of the given ways and any other
    // section in a plot file, with both heights at the crossing point.
    public static void Crossings(string plot, IReadOnlyCollection<long> ways)
    {
        var sections = new List<(long Way, string Class, bool Bridge, bool Tunnel, float3[] Points)>();

        using (var reader = new StreamReader(new GZipStream(File.OpenRead(plot), CompressionMode.Decompress)))
            for (string? line; (line = reader.ReadLine()) != null;)
            {
                var row = JObject.Parse(line);
                var p = row["p"]!.Select(token => (float)token).ToArray();
                sections.Add(
                    (
                        (long)row["WayId"]!,
                        (string)row["Classification"]!,
                        (bool)row["Bridge"]!,
                        (bool)row["Tunnel"]!,
                        Enumerable
                            .Range(0, p.Length / 3)
                            .Select(k => new float3(p[3 * k], p[3 * k + 1], p[3 * k + 2]))
                            .ToArray()));
            }

        foreach (var item in sections.Where(otherItem => ways.Contains(otherItem.Way)))
            foreach (var otherItem in sections.Where(secondItem => !ways.Contains(secondItem.Way)))
                for (var i = 1; i < item.Points.Length; i++)
                    for (var j = 1; j < otherItem.Points.Length; j++)
                    {
                        float2 vectorA = item.Points[i - 1].xz,
                            vectorB = item.Points[i].xz,
                            c = otherItem.Points[j - 1].xz,
                            d = otherItem.Points[j].xz;

                        var r = vectorB - vectorA;
                        var point = d - c;
                        var den = r.x * point.y - r.y * point.x;

                        if (Math.Abs(den) < 1e-6f)
                            continue;

                        var t = ((c.x - vectorA.x) * point.y - (c.y - vectorA.y) * point.x) / den;
                        var v = ((c.x - vectorA.x) * r.y - (c.y - vectorA.y) * r.x) / den;

                        if (t < 0 || t > 1 || v < 0 || v > 1)
                            continue;

                        var yu = math.lerp(item.Points[i - 1].y, item.Points[i].y, t);
                        var yo = math.lerp(otherItem.Points[j - 1].y, otherItem.Points[j].y, v);
                        Console.WriteLine(
                            $"  {item.Way}{(item.Bridge ? " bridge" : item.Tunnel ? " tunnel" : "")} y={yu:F1} x {otherItem.Way} {otherItem.Class}{(otherItem.Bridge ? " bridge" : otherItem.Tunnel ? " tunnel" : "")} y={yo:F1}; gap={yo - yu:F1} at ({vectorA.x + t * r.x:F0},{vectorA.y + t * r.y:F0})");
                    }
    }

    // Time the bridge/tunnel interval step (water sampler construction and the
    // per-section water test) on a captured map, as the import runs it.
    public static void IntervalTiming(string folder)
    {
        var watch = System.Diagnostics.Stopwatch.StartNew();
        var capture = Load(folder);
        var load = watch.Elapsed.TotalSeconds;
        var input = capture.Input;

        T[] Read<T>(string name) => CorrectiveCapturedReplay.ReadManifest<T>(folder, (string)input[name]!);

        var water =
            new WaterPlan(
                false,
                0,
                0,
                0,
                "captured")
            {
                AreaStages = Read<WaterAreaStage>("waterAreasManifest"),
                ReachStages = Read<WaterReachStage>("waterReachesManifest")
            };

        var polygons = Read<GeographicPolygon>("waterPolygonsManifest");
        var ground =
            capture.Source
                .Select(segment => segment with { Bridge = false })
                .Where(segment => !segment.Tunnel)
                .ToArray();

        // Unfiltered and prefiltered samplers back to back, with every query counted
        // and every answer compared, so the result is independent of machine load.
        var plain = WaterBodyPlanner.SurfaceSampler(water, polygons, reference: true);
        watch.Restart();
        var sampler = WaterBodyPlanner.SurfaceSampler(water, polygons);
        var build = watch.Elapsed.TotalSeconds;
        long queries = 0, differ = 0;
        var answers = new List<float?>();
        watch.Restart();
        NetworkStructureIntervals.StructureIntervals(
            ground,
            capture.Terrain,
            capture.Width,
            new ImportLog(),
            default,
            point =>
        {
            var v = plain(point);
            answers.Add(v);

            return v;
        }, tunnelIntervalsResolved: true);

        var plainSeconds = watch.Elapsed.TotalSeconds;
        watch.Restart();
        NetworkStructureIntervals.StructureIntervals(
            ground,
            capture.Terrain,
            capture.Width,
            new ImportLog(),
            default,
            point => null,
            tunnelIntervalsResolved: true);

        Console.WriteLine(
            $"  intervals with no water at all {watch.Elapsed.TotalSeconds:F2}s (the cost outside the sampler)");

        var k = 0;
        watch.Restart();
        var result =
            NetworkStructureIntervals.StructureIntervals(
                ground,
                capture.Terrain,
                capture.Width,
                new ImportLog(),
                default,
                point =>
        {
            var v = sampler(point);
            queries++;

            if (k < answers.Count && !Nullable.Equals(answers[k++], v))
                differ++;

            return v;
        }, tunnelIntervalsResolved: true);

        Console.WriteLine(
            $"  load {load:F1}s; sampler build {build:F2}s; intervals unfiltered {plainSeconds:F2}s, prefiltered {watch.Elapsed.TotalSeconds:F2}s; {queries:N0} queries, {differ} answers differ; {ground.Length} sections ({result.Count(segment => segment.Bridge)} over water); areas {water.AreaStages.Count}, reaches {water.ReachStages.Count}, polygons {polygons.Length}");
    }

    // Report 69: rock candidates for a captured map from a rock-query OSM file,
    // against the captured terrain, design roads and water polygons.
    public static void RockPlan(string folder, string osm)
    {
        var capture = Load(folder);
        var request = capture.Input["request"]!.ToObject<ImportRequest>()! with { ImportRocks = true };
        // Several tiles (separated by |) are parsed one by one and combined. A parsed-OSM
        // cache file (.bin) is the exact dataset the game planned from. A folder of them
        // stands for every cache file whose first node lies within 40 km of the map centre.
        var files =
            Directory.Exists(osm)
                ? Directory
                    .GetFiles(osm, "*.bin")
                    .Where(
                        file => WaterReplayMetrics.FirstNode(file) is var (latitude, longitude)
                            && math.length(
                                ImportEarthLocation.Geo.GeoMath
                                    .ToLocalMeters(latitude, longitude, request.Latitude, request.Longitude)
                                    .xz)
                                < 40000)
                    .OrderBy(file => file, StringComparer.Ordinal)
                    .ToArray()
                : osm.Split('|');

        var tiles =
            files
                .Select(
                    text => text.EndsWith(".bin", StringComparison.OrdinalIgnoreCase)
                        ? OsmParsedCache.LoadSnapshot(text, default)
                        : OsmParser.Merge(
                            new[] { File.ReadAllBytes(text) },
                            new ImportLog(),
                            default,
                            wholeMap: true))
                .ToArray();

        var data = tiles.Length == 1 ? tiles[0] : new OsmDataset
        {
            Nodes =
                tiles
                    .SelectMany(dataset => dataset.Nodes)
                    .GroupBy(entry => entry.Key)
                    .ToDictionary(group => group.Key, group => group.First().Value),
            Ways =
                tiles
                    .SelectMany(dataset => dataset.Ways)
                    .GroupBy(entry => entry.Key)
                    .ToDictionary(group => group.Key, group => group.First().Value),
            NodeTags =
                tiles
                    .SelectMany(dataset => dataset.NodeTags)
                    .GroupBy(entry => entry.Key)
                    .ToDictionary(group => group.Key, group => group.First().Value)
        };

        var water =
            capture.Input["waterPolygonsManifest"] != null
                ? CorrectiveCapturedReplay.ReadManifest<GeographicPolygon>(
                    folder,
                    (string)capture.Input["waterPolygonsManifest"]!)
                : Array.Empty<GeographicPolygon>();

        var terrain = new TerrainBuilder.Result(capture.Terrain, capture.Scale, 0, 0, capture.Scale);

        // The captured water plan's stages: the rivers and lakes rock must stay out of. Its
        // sources are not captured, so placement near them is not checked here.
        WaterStage[] Stages<WaterStage>(string name)
            => capture.Input[name] != null
                ? CorrectiveCapturedReplay.ReadManifest<WaterStage>(folder, (string)capture.Input[name]!)
                : Array.Empty<WaterStage>();

        var waterPlan =
            new WaterPlan(false, 0, 0, 0, "replay")
            {
                WaterCellSize = CapturedRunFiles.WaterCellSize(folder, capture.Input),
                AreaStages = Stages<WaterAreaStage>("waterAreasManifest"),
                ReachStages = Stages<WaterReachStage>("waterReachesManifest")
            };

        var log = new ImportLog();
        var watch = System.Diagnostics.Stopwatch.StartNew();
        var rocks =
            RockPlanner.Prepare(
                data,
                request,
                terrain,
                capture.Source,
                water,
                waterPlan,
                log,
                default);

        Console.WriteLine(
            $"  rocks={rocks.Count} in {watch.Elapsed.TotalSeconds:F1}s; small={rocks.Count(candidate => candidate.Form == VegetationForm.RockSmall)}; medium={rocks.Count(candidate => candidate.Form == VegetationForm.RockMedium)}; large={rocks.Count(candidate => candidate.Form == VegetationForm.RockLarge)}");

        foreach (var line in log.Lines.Where(text => text.StartsWith("Rocks prepared", StringComparison.Ordinal)))
            Console.WriteLine("  " + line);

        // Ordered fingerprint of every candidate, to compare implementations exactly.
        var fingerprint = new System.Text.StringBuilder();

        foreach (var candidate in rocks)
            fingerprint.Append(
                FormattableString.Invariant(
                    $"{BitConverter.SingleToInt32Bits(candidate.Position.x)},{BitConverter.SingleToInt32Bits(candidate.Position.y)},{BitConverter.SingleToInt32Bits(candidate.Position.z)},{candidate.Form},{candidate.SourceId},{candidate.Seed},{candidate.MetadataIndex};"));

        Console.WriteLine(
            $"  candidate fingerprint {Convert.ToHexString(System.Security.Cryptography.SHA256.HashData(System.Text.Encoding.UTF8.GetBytes(fingerprint.ToString())))[..16]}");

        var below =
            rocks.Count(candidate => capture.Water(candidate.Position) is float stage && candidate.Position.y < stage);

        Console.WriteLine($"  rocks below a captured water surface: {below}");

        // ROCK_NEAR=lat,lon: rocks within 150 m of a point by reason and size,
        // and the rock-related OSM features covering or passing near it.
        if (Environment.GetEnvironmentVariable("ROCK_NEAR") is string near)
        {
            var parts = near.Split(',').Select(text => double.Parse(text, CultureInfo.InvariantCulture)).ToArray();
            var at =
                ImportEarthLocation.Geo.GeoMath.ToLocalMeters(parts[0], parts[1], request.Latitude, request.Longitude);

            var close = rocks.Where(candidate => math.distance(candidate.Position.xz, at.xz) < 150).ToArray();
            Console.WriteLine(
                $"  within 150 m of {near}: {close.Length} rocks ({close.Count(candidate => candidate.Form == VegetationForm.RockLarge)} boulders); terrain {NetworkProcessor.Sample(at.x, at.z, capture.Terrain, capture.Width):0.0} m");

            foreach (var group in close.GroupBy(candidate => RockPlanner.Reasons[candidate.MetadataIndex]))
                Console.WriteLine($"    {group.Key}: {group.Count()}");

            foreach (var polygon in GeographyPolygons
                .Build(data, request, new ImportLog(), default)
                .Where(otherPolygon => otherPolygon.Contains(at)))
                Console.WriteLine(
                    $"    inside polygon {polygon.Id}: {string.Join(", ", polygon.Tags.Where(entry => entry.Key is "natural" or "surface" or "landuse" or "name").Select(entry => entry.Key + "=" + entry.Value))}");

            foreach (var way in data.Ways.Values.Where(
                otherWay => otherWay.Tags.TryGetValue("natural", out var text) && text == "cliff"))
            {
                var points =
                    way.NodeIds
                        .Where(data.Nodes.ContainsKey)
                        .Select(
                            id => ImportEarthLocation.Geo.GeoMath.ToLocalMeters(
                                data.Nodes[id].Latitude,
                                data.Nodes[id].Longitude,
                                request.Latitude,
                                request.Longitude))
                        .ToArray();

                var d =
                    points
                        .Zip(
                            points.Skip(1),
                            (vectorA, vectorB) => NetworkNormalizer.DistanceToSegment(at, vectorA, vectorB))
                        .DefaultIfEmpty(float.PositiveInfinity)
                        .Min();

                if (d < 300)
                    Console.WriteLine($"    cliff way {way.Id} at {d:0} m");
            }
        }

        // Where they are: the densest 2 km squares, with their centre's latitude/longitude.
        var lat0 = (double)request.Latitude;
        var lon0 = (double)request.Longitude;

        foreach (var group in rocks
            .GroupBy(
                candidate => (
                    (int)Math.Floor(candidate.Position.x / 2000),
                    (int)Math.Floor(candidate.Position.z / 2000)))
            .OrderByDescending(otherGroup => otherGroup.Count())
            .Take(15))
        {
            var x = (group.Key.Item1 + .5) * 2000;
            var z = (group.Key.Item2 + .5) * 2000;
            var lat = lat0 + math.degrees(z / Earth);
            var lon = lon0 + math.degrees(x / (Earth * Math.Cos(math.radians(lat0))));
            Console.WriteLine(
                $"  square {lat:0.000},{lon:0.000}: {group.Count():N0} rocks ({group.Count(candidate => candidate.Form == VegetationForm.RockLarge):N0} boulders)");
        }
    }

    // Beaches against the captured terrain: how much of each mapped beach lies
    // at or below a water level, and how wide it is (twice area over perimeter).
    public static void BeachCheck(string folder, string osm, float waterLevel)
    {
        var capture = Load(folder);
        var request = capture.Input["request"]!.ToObject<ImportRequest>()!;
        // Several tiles (separated by |) are parsed one by one and combined.
        var tiles =
            osm
                .Split('|')
                .Select(
                    text => OsmParser.Merge(
                        new[] { File.ReadAllBytes(text) },
                        new ImportLog(),
                        default,
                        wholeMap: true))
                .ToArray();

        var data = tiles.Length == 1 ? tiles[0] : new OsmDataset
        {
            Nodes =
                tiles
                    .SelectMany(dataset => dataset.Nodes)
                    .GroupBy(entry => entry.Key)
                    .ToDictionary(group => group.Key, group => group.First().Value),
            Ways =
                tiles
                    .SelectMany(dataset => dataset.Ways)
                    .GroupBy(entry => entry.Key)
                    .ToDictionary(group => group.Key, group => group.First().Value),
            NodeTags =
                tiles
                    .SelectMany(dataset => dataset.NodeTags)
                    .GroupBy(entry => entry.Key)
                    .ToDictionary(group => group.Key, group => group.First().Value)
        };

        var beaches =
            GeographyPolygons
                .Build(data, request, new ImportLog(), default)
                .Where(polygon => polygon.Tags.TryGetValue("natural", out var otherText) && otherText == "beach")
                .ToArray();

        if (Environment.GetEnvironmentVariable("BEACH_LIFT") == "1")
        {
            var liftLog = new ImportLog();
            capture =
                capture with
                {
                    Terrain =
                        BeachTerrain.Lift(
                            new TerrainBuilder.Result(
                                capture.Terrain,
                                capture.Scale,
                                waterLevel - WaterLevels.CorrectionMeters,
                                0,
                                capture.Scale),
                            capture.Width,
                            beaches,
                            liftLog,
                            default)
                };

            foreach (var line in liftLog.Lines.Where(text => text.StartsWith("Beaches:", StringComparison.Ordinal)))
                Console.WriteLine("  " + line);
        }

        var widths = new List<double>();
        var under = 0L;
        var total = 0L;
        var drowned = 0;
        var rows = new List<string>();

        foreach (var polygon in beaches)
        {
            var ring = polygon.Outer.OrderByDescending(points => points.Length).First();
            double area = 0, perimeter = 0;

            for (var i = 0; i < ring.Length; i++)
            {
                var point = ring[i];
                var otherPoint = ring[(i + 1) % ring.Length];
                area += point.x * otherPoint.z - otherPoint.x * point.z;
                perimeter += math.distance(point.xz, otherPoint.xz);
            }

            area = Math.Abs(area) / 2;
            var width = perimeter > 0 ? 2 * area / perimeter : 0;
            widths.Add(width);
            var low = ring.Select(point => point.xz).Aggregate(math.min);
            var high = ring.Select(point => point.xz).Aggregate(math.max);
            long nodeId = 0, wet = 0;
            var heights = new List<float>();

            for (var z = low.y; z <= high.y; z += 3)
                for (var x = low.x; x <= high.x; x += 3)
                {
                    var point = new float3(x, 0, z);

                    if (!polygon.Contains(point))
                        continue;

                    nodeId++;
                    var y = NetworkProcessor.Sample(x, z, capture.Terrain, capture.Width);
                    heights.Add(y);

                    if (y <= waterLevel)
                        wet++;
                }

            total += nodeId;
            under += wet;

            if (nodeId > 0 && wet * 2 > nodeId)
                drowned++;

            if (nodeId > 0)
                heights.Sort();

            rows.Add(
                $"{polygon.Id} surface={(polygon.Tags.TryGetValue("surface", out var text) ? text : "-")} width={width:F0}m area={area:F0}m² samples={nodeId} underWater={(nodeId == 0 ? 0 : 100.0 * wet / nodeId):F0}% heightP10={(nodeId == 0 ? 0 : heights[(int)(.1 * (nodeId - 1))]):F1} median={(nodeId == 0 ? 0 : heights[(int)(.5 * (nodeId - 1))]):F1}");
        }

        widths.Sort();

        double Quantile(double q) => widths.Count == 0 ? 0 : widths[(int)(q * (widths.Count - 1))];

        Console.WriteLine(
            $"  beaches={beaches.Length}; widthP10={Quantile(.1):F0} m; median={Quantile(.5):F0} m; P90={Quantile(.9):F0} m; samples under {waterLevel} m: {(total == 0 ? 0 : 100.0 * under / total):F1}%; beaches mostly under water: {drowned}");

        foreach (var row in rows
            .OrderBy(text => text)
            .Take(
                Environment.GetEnvironmentVariable("BEACH_ROWS") is string secondText
                    && int.TryParse(secondText, out var m)
                    ? m
                    : 0))
            Console.WriteLine("  " + row);
    }

    // Shaded relief of a captured heightmap window (centre x,z in local metres),
    // one image pixel per heightmap pixel scaled up, water polygon edges in blue,
    // so stair-stepped banks can be seen. Light from the north-west.
    // Hillshades of one patch at the game's 14 m pixel: zoom-12 DEM bilinear,
    // zoom-12 clamped bicubic, zoom-13 clamped bicubic. Each input is a 768×768
    // float mosaic of the 3×3 Terrarium tiles around (latitude, longitude).
    public static void ResampleDemo(string z12, string z13, double latitude, double longitude, string output)
    {
        const int mosaic = 768;
        const double outputSpacing = 14;
        var size = int.TryParse(Environment.GetEnvironmentVariable("DEMO_SIZE"), out var demoSize) ? demoSize : 192;
        var scale = int.TryParse(Environment.GetEnvironmentVariable("DEMO_SCALE"), out var demoScale) ? demoScale : 3;

        float[] Load(string path)
        {
            var bytes = File.ReadAllBytes(path);
            var f = new float[bytes.Length / 4];
            Buffer.BlockCopy(bytes, 0, f, 0, bytes.Length);

            return f;
        }

        var sources = new[] { (Data: Load(z12), Zoom: 12), (Data: Load(z13), Zoom: 13) };
        var groundPerPixel = 40075016.68557849 * Math.Cos(latitude * Math.PI / 180);
        var panels = new List<float[]>();

        foreach (var (data, zoom, cubic) in new[] {
            (sources[0].Data, 12, false),
            (sources[0].Data, 12, true),
            (sources[1].Data, 13, true)
            })
        {
            var centreX = ImportEarthLocation.Geo.GeoMath.LongitudeToTileX(longitude, zoom) * 256;
            var centreY = ImportEarthLocation.Geo.GeoMath.LatitudeToTileY(latitude, zoom) * 256;
            var originX = (Math.Floor(centreX / 256) - 1) * 256;
            var originY = (Math.Floor(centreY / 256) - 1) * 256;
            var step = outputSpacing / (groundPerPixel / (256d * (1 << zoom)));

            float Read(int x, int y) => data[Math.Clamp(y, 0, mosaic - 1) * mosaic + Math.Clamp(x, 0, mosaic - 1)];

            var heights = new float[size * size];

            for (var j = 0; j < size; j++)
                for (var i = 0; i < size; i++)
                {
                    var px = centreX - originX + (i - size / 2) * step;
                    var py = centreY - originY + (j - size / 2) * step;
                    var x = (int)Math.Floor(px);
                    var y = (int)Math.Floor(py);
                    var fx = (float)(px - x);
                    var fy = (float)(py - y);
                    heights[j * size + i] =
                        cubic
                            ? TerrainSampling.ClampedCubic(
                                Read,
                                x,
                                y,
                                TerrainSampling.CubicWeights(fx),
                                TerrainSampling.CubicWeights(fy))
                            :
                        (Read(x, y) * (1 - fx) + Read(x + 1, y) * fx) * (1 - fy)
                            + (Read(x, y + 1) * (1 - fx) + Read(x + 1, y + 1) * fx) * fy;
                }

            panels.Add(heights);
        }

        var w = (size * scale + 8) * panels.Count;
        var h = size * scale;
        var rgb = new byte[w * h * 3];

        for (var p = 0; p < panels.Count; p++)
            for (var j = 1; j < size - 1; j++)
                for (var i = 1; i < size - 1; i++)
                {
                    var a = panels[p];
                    var dx = (a[j * size + i + 1] - a[j * size + i - 1]) / (2 * outputSpacing);
                    var dz = (a[(j - 1) * size + i] - a[(j + 1) * size + i]) / (2 * outputSpacing);
                    var normal = math.normalize(new double3(-dx, 1, -dz));
                    var shade =
                        (byte)(255 * Math.Clamp(math.dot(normal, math.normalize(new double3(-1, 1.2, 1))) * 1.1, 0, 1));

                    for (var sz = 0; sz < scale; sz++)
                        for (var sx = 0; sx < scale; sx++)
                        {
                            var k = ((j * scale + sz) * w + p * (size * scale + 8) + i * scale + sx) * 3;
                            rgb[k] = rgb[k + 1] = rgb[k + 2] = shade;
                        }
                }

        WritePng(output, w, h, rgb);
        Console.WriteLine(
            $"  resample demo {size}×{size} pixels at {outputSpacing} m: zoom-12 bilinear | zoom-12 bicubic | zoom-13 bicubic → {output}");
    }

    public static void TerrainCrop(string folder, float cx, float cz, float size, string output)
    {
        var capture = Load(folder);

        // TERRAIN_FILE renders another heightmap of the run, such as final-design-heightmap.u16.
        if (Environment.GetEnvironmentVariable("TERRAIN_FILE") is string other && other.Length > 0)
        {
            capture = capture with { Terrain = CapturedRunFiles.Heights(Path.Combine(folder, other), capture.Scale) };
        }

        var n = (int)Math.Sqrt(capture.Terrain.Length);
        var spacing = capture.Width / (n - 1);

        int Index(double v) => (int)Math.Round((v / capture.Width + .5) * (n - 1));

        var x0 = Math.Max(1, Index(cx - size / 2));
        var x1 = Math.Min(n - 2, Index(cx + size / 2));
        var z0 = Math.Max(1, Index(cz - size / 2));
        var z1 = Math.Min(n - 2, Index(cz + size / 2));
        var scale = Math.Max(1, (int)(1024 / (x1 - x0 + 1)));
        var w = (x1 - x0 + 1) * scale;
        var h = (z1 - z0 + 1) * scale;
        var rgb = new byte[w * h * 3];

        float HeightAt(int x, int z) => capture.Terrain[z * n + x];

        var water =
            capture.Input["waterPolygonsManifest"] != null
                ? CorrectiveCapturedReplay.ReadManifest<GeographicPolygon>(
                    folder,
                    (string)capture.Input["waterPolygonsManifest"]!)
                : Array.Empty<GeographicPolygon>();

        for (var z = z0; z <= z1; z++)
            for (var x = x0; x <= x1; x++)
            {
                var dx = (HeightAt(x + 1, z) - HeightAt(x - 1, z)) / (2 * spacing);
                var dz = (HeightAt(x, z + 1) - HeightAt(x, z - 1)) / (2 * spacing);
                var normal = math.normalize(new double3(-dx, 1, -dz));
                var shade = Math.Clamp(math.dot(normal, math.normalize(new double3(-1, 1.2, 1))) * 1.1, 0, 1);
                var y = HeightAt(x, z);
                var tint = (y - HeightAt(x0, z0)) / 40;
                var r = (byte)(255 * shade);
                var g = (byte)(255 * shade);
                var b = (byte)(255 * shade);

                for (var sz = 0; sz < scale; sz++)
                    for (var sx = 0; sx < scale; sx++)
                    {
                        // Image row 0 is north: larger z first.
                        var px = (x - x0) * scale + sx;
                        var py = (z1 - z) * scale + (scale - 1 - sz);
                        var k = (py * w + px) * 3;
                        rgb[k] = r;
                        rgb[k + 1] = g;
                        rgb[k + 2] = b;
                    }
            }

        foreach (var polygon in water)
            foreach (var ring in polygon.Outer.Concat(polygon.Inner))
                for (var i = 0; i < ring.Length; i++)
                {
                    var vectorA = ring[i];
                    var vectorB = ring[(i + 1) % ring.Length];
                    var steps = (int)Math.Ceiling(math.distance(vectorA.xz, vectorB.xz) / (spacing / scale)) + 1;

                    for (var s = 0; s <= steps; s++)
                    {
                        var point = math.lerp(vectorA, vectorB, s / (float)steps);
                        var fx = ((point.x / capture.Width + .5) * (n - 1) - x0 + .5) * scale;
                        var fz = ((point.z / capture.Width + .5) * (n - 1) - z0 + .5) * scale;
                        var px = (int)fx;
                        var py = h - 1 - (int)fz;

                        if (px < 0 || py < 0 || px >= w || py >= h)
                            continue;

                        var k = (py * w + px) * 3;
                        rgb[k] = 40;
                        rgb[k + 1] = 90;
                        rgb[k + 2] = 255;
                    }
                }

        WritePng(output, w, h, rgb);
        Console.WriteLine(
            $"  crop {x1 - x0 + 1}×{z1 - z0 + 1} heightmap pixels ({spacing:F1} m each) at {scale}× → {output}");
    }

    // Pixel "teeth": a grid point standing more than threshold metres off the
    // mean of its four neighbours, the signature of a height step written per
    // pixel along a slanted edge. Compares an earlier and a later heightmap of
    // one run (file names inside the run folder) over a centred square, prints
    // counts and the densest 500 m tiles, and draws the later relief with teeth
    // present in both maps in yellow and teeth new in the later one in red.
    public static void Teeth(string folder, string earlier, string later, float half, float threshold, string output)
    {
        var run =
            JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder, "offline-design-pass-01-input.json")));

        var scale = (float)run["HeightScaleMeters"]!;

        float[] Read(string name) => CapturedRunFiles.Heights(Path.Combine(folder, name), scale);

        var width = (double)run["widthMeters"]!;
        var a = Read(earlier);
        var b = Read(later);
        var n = (int)Math.Sqrt(b.Length);
        var spacing = width / (n - 1);

        int Index(double v) => (int)Math.Round((v / width + .5) * (n - 1));

        var low = Math.Max(1, Index(-half));
        var high = Math.Min(n - 2, Index(half));
        var w = high - low + 1;
        var rgb = new byte[w * w * 3];

        float Laplacian(float[] m, int x, int z)
            => m[z * n + x] - (m[z * n + x - 1] + m[z * n + x + 1] + m[(z - 1) * n + x] + m[(z + 1) * n + x]) / 4f;

        long both = 0, fresh = 0, gone = 0;
        var tiles = new Dictionary<(int, int), (int Both, int Fresh)>();

        for (var z = low; z <= high; z++)
            for (var x = low; x <= high; x++)
            {
                float HeightAt(int xx, int zz) => b[zz * n + xx];

                var dx = (HeightAt(x + 1, z) - HeightAt(x - 1, z)) / (2 * spacing);
                var dz = (HeightAt(x, z + 1) - HeightAt(x, z - 1)) / (2 * spacing);
                var shade =
                    Math.Clamp(
                        math.dot(math.normalize(new double3(-dx, 1, -dz)), math.normalize(new double3(-1, 1.2, 1)))
                            * 1.1,
                        0,
                        1);

                var inA = Math.Abs(Laplacian(a, x, z)) > threshold;
                var inB = Math.Abs(Laplacian(b, x, z)) > threshold;
                var k = ((high - z) * w + (x - low)) * 3;
                var g = (byte)(255 * shade);
                rgb[k] = g;
                rgb[k + 1] = g;
                rgb[k + 2] = g;

                if (inB)
                {
                    var key =
                        (
                            (int)Math.Floor((x * spacing - width / 2) / 500),
                            (int)Math.Floor((z * spacing - width / 2) / 500));

                    tiles.TryGetValue(key, out var item);

                    if (inA)
                    {
                        both++;
                        rgb[k] = 255;
                        rgb[k + 1] = 210;
                        rgb[k + 2] = 0;
                        item.Both++;
                    }
                    else
                    {
                        fresh++;
                        rgb[k] = 255;
                        rgb[k + 1] = 30;
                        rgb[k + 2] = 30;
                        item.Fresh++;
                    }

                    tiles[key] = item;
                }
                else if (inA)
                    gone++;
            }

        WritePng(output, w, w, rgb);
        Console.WriteLine(
            $"  teeth over {threshold} m within ±{half} m: in both {both}; new in {later} {fresh}; removed {gone} → {output}");

        foreach (var entry in tiles
            .OrderByDescending(otherEntry => otherEntry.Value.Both + otherEntry.Value.Fresh)
            .Take(25))
            Console.WriteLine(
                $"    tile x={entry.Key.Item1 * 500 + 250} z={entry.Key.Item2 * 500 + 250}: both {entry.Value.Both}, new {entry.Value.Fresh}");
    }

    // River bank containment on the captured water terrain: for every 192 m of
    // each accepted reach, the planner's two dry bank probes (first point at
    // least two 28 m water cells out whose diagonal footprint is dry) and the
    // stage's margin below the lower one. WaterBodyPlanner.Revalidate rejects a
    // source whose margin is under 0.25 m.
    public static void WaterBanks(string folder)
    {
        var capture = Load(folder);

        float HeightAt(float3 point)
            => NetworkProcessor.Sample(point.x, point.z, capture.Terrain, capture.Width);

        var reaches =
            CorrectiveCapturedReplay.ReadManifest<WaterReachStage>(
                folder,
                (string)capture.Input["waterReachesManifest"]!);

        var polygons =
            CorrectiveCapturedReplay
                .ReadManifest<GeographicPolygon>(folder, (string)capture.Input["waterPolygonsManifest"]!)
                .ToDictionary(polygon => polygon.Identity);

        const float cell = 28;
        var margins = new List<(float Margin, float3 P, long Way, bool Polygon, float Bank, float Stage)>();

        foreach (var reach in reaches)
        {
            var length = math.distance(reach.Start.xz, reach.End.xz);

            if (length < .01f)
                continue;

            var normal = math.normalizesafe(new float3(-(reach.End - reach.Start).z, 0, (reach.End - reach.Start).x));
            polygons.TryGetValue(reach.PolygonId ?? long.MinValue, out var polygon);
            Func<float3, bool> wet =
                polygon != null
                    ? polygon.Contains
                    : point => NetworkNormalizer.DistanceToSegment(point, reach.Start, reach.End) <= reach.HalfWidth;

            bool Dry(float3 point)
                => !wet(point)
                    && !wet(point + new float3(42, 0, 42))
                    && !wet(point + new float3(-42, 0, 42))
                    && !wet(point + new float3(42, 0, -42))
                    && !wet(point + new float3(-42, 0, -42));

            for (var s = 0f; s < length; s += 192)
            {
                var t = s / length;
                var point = math.lerp(reach.Start, reach.End, t);
                var stage = math.lerp(reach.StartLevel, reach.EndLevel, t);
                var bank = float.PositiveInfinity;

                foreach (var side in new[] { -1, 1 })
                {
                    var start = Math.Max(cell, reach.HalfWidth);

                    while (start < 512 && wet(point + normal * (start * side)))
                        start += Math.Max(cell, reach.HalfWidth * .5f);

                    for (var d = Math.Max(start, cell * 2); d <= 1024; d += cell)
                    {
                        var otherPoint = point + normal * (d * side);

                        if (Dry(otherPoint))
                        {
                            bank = Math.Min(bank, HeightAt(otherPoint));
                            break;
                        }
                    }
                }

                if (math.isfinite(bank))
                    margins.Add((bank - stage, point, reach.WayId, polygon != null, bank, stage));
            }
        }

        foreach (var kind in new[] { false, true })
        {
            var m = margins.Where(item => item.Polygon == kind).Select(item => item.Margin).OrderBy(x => x).ToArray();

            if (m.Length == 0)
                continue;

            Console.WriteLine(
                $"  {(kind ? "polygon" : "centerline")} samples {m.Length}: margin < .25 m {m.Count(x => x < .25f)} ({m.Count(x => x < .25f) / (double)m.Length:P0}); < .6 {m.Count(x => x < .6f)}; < 1 {m.Count(x => x < 1)}; median {m[m.Length / 2]:0.00} m; p10 {m[m.Length / 10]:0.00} m");
        }

        // WATER_NEAR=x,z lists water polygons with a vertex within 300 m.
        if (Environment.GetEnvironmentVariable("WATER_NEAR") is string near && near.Length > 0)
        {
            var at = near.Split(',').Select(text => float.Parse(text, CultureInfo.InvariantCulture)).ToArray();
            var otherPoint = new float2(at[0], at[1]);

            foreach (var polygon in polygons.Values.Where(
                otherPolygon => otherPolygon.Outer
                    .Concat(otherPolygon.Inner)
                    .Any(points => points.Any(point => math.distance(point.xz, otherPoint) < 300))))
                Console.WriteLine(
                    $"    near polygon {polygon.Identity} {string.Join(",", polygon.Tags.Select(entry => entry.Key + "=" + entry.Value))}; holes {polygon.Inner.Count}; contains {polygon.Contains(new float3(otherPoint.x, 0, otherPoint.y))}");
        }

        // WATER_NAME=text lists water polygons whose name contains it.
        if (Environment.GetEnvironmentVariable("WATER_NAME") is string named && named.Length > 0)
            foreach (var polygon in polygons.Values.Where(
                otherPolygon => otherPolygon.Tags.TryGetValue("name", out var text)
                    && text.Contains(named, StringComparison.OrdinalIgnoreCase)))
            {
                var all = polygon.Outer.SelectMany(points => points).ToArray();
                Console.WriteLine(
                    $"    polygon {polygon.Identity} {string.Join(",", polygon.Tags.Select(entry => entry.Key + "=" + entry.Value))}: x {all.Min(point => point.x):0}..{all.Max(point => point.x):0} z {all.Min(point => point.z):0}..{all.Max(point => point.z):0}");
            }

        // WATER_OSM=parsed-osm cache file groups the samples by waterway type and
        // prints the traced way's tags.
        if (Environment.GetEnvironmentVariable("WATER_OSM") is string osm && osm.Length > 0)
        {
            var ways = OsmParsedCache.LoadSnapshot(osm, default).Ways;

            string Kind(long id)
                => ways.TryGetValue(id, out var otherWay)
                    ? (otherWay.Tags.TryGetValue("waterway", out var text) ? text : "?")
                        + (otherWay.Tags.ContainsKey("abandoned") || otherWay.Tags.ContainsKey("disused")
                            ? "+abandoned"
                            : "")
                        + (otherWay.Tags.ContainsKey("tunnel") ? "+tunnel" : "")
                    : "missing";

            foreach (var group in margins
                .GroupBy(item => Kind(item.Way))
                .OrderByDescending(otherGroup => otherGroup.Count()))
                Console.WriteLine(
                    $"    {group.Key}: {group.Count()} samples on {group.Select(item => item.Way).Distinct().Count()} ways, margin < .25 m {group.Count(item => item.Margin < .25f)}, < -2 m {group.Count(item => item.Margin < -2)}");

            if (long.TryParse(Environment.GetEnvironmentVariable("WATER_WAY"), out var tagged)
                && ways.TryGetValue(tagged, out var way))
                Console.WriteLine(
                    $"    way {tagged}: {string.Join(", ", way.Tags.Select(entry => entry.Key + "=" + entry.Value))}; {way.NodeIds.Length} nodes");
        }

        // WATER_WAY=id prints that way's reaches: stage, centreline ground and the lower bank.
        if (long.TryParse(Environment.GetEnvironmentVariable("WATER_WAY"), out var traced))
            foreach (var reach in reaches.Where(otherReach => otherReach.WayId == traced))
                Console.WriteLine(
                    $"    ({reach.Start.x:0},{reach.Start.z:0})→({reach.End.x:0},{reach.End.z:0}) stage {reach.StartLevel:0.00}→{reach.EndLevel:0.00} ground {HeightAt(reach.Start):0.00} half {reach.HalfWidth:0} bank {margins.Where(item => item.Way == traced && math.distance(item.P.xz, reach.Start.xz) < 1).Select(item => item.Bank.ToString("0.00")).FirstOrDefault()}");

        foreach (var firstItem in margins.Where(item => item.Margin < .25f).OrderBy(item => item.Margin).Take(40))
            Console.WriteLine(
                $"    way {firstItem.Way} at ({firstItem.P.x:0},{firstItem.P.z:0}) {(firstItem.Polygon ? "polygon" : "line")}: stage {firstItem.Stage:0.00} bank {firstItem.Bank:0.00} margin {firstItem.Margin:0.00}");

        // WATER_SECTION=x,z prints the ground across the reach nearest that point.
        if (Environment.GetEnvironmentVariable("WATER_SECTION") is string section && section.Length > 0)
        {
            var at = section.Split(',').Select(text => float.Parse(text, CultureInfo.InvariantCulture)).ToArray();
            var otherPoint = new float3(at[0], 0, at[1]);
            var reach =
                reaches
                    .OrderBy(
                        otherReach => NetworkNormalizer.DistanceToSegment(otherPoint, otherReach.Start, otherReach.End))
                    .First();

            var normal = math.normalizesafe(new float3(-(reach.End - reach.Start).z, 0, (reach.End - reach.Start).x));
            Console.WriteLine(
                $"    way {reach.WayId} stage {reach.StartLevel:0.00}: "
                    + string.Join(
                        " ",
                        Enumerable
                            .Range(-10, 21)
                            .Select(k => $"{k * 14}:{HeightAt(otherPoint + normal * (k * 14)):0.0}")));
        }
    }

    // Replays WaterBodyPlanner on a capture. The capture keeps no pre-water DEM,
    // so ground within each waterway's carving reach (channel/.6 plus a cell) is
    // restored by harmonic inpainting of the captured terrain around it, outside
    // mapped water polygons. Prints the planner's evidence and counters, the
    // sources Revalidate keeps on the planned terrain alone, and channel
    // continuity: walking each open river, stream or canal downstream every
    // 7 m, a sample is dammed when the bed on its line stands over 0.1 m above
    // the lowest planned water stage upstream on that way (water from upstream
    // cannot pass), and cut depth is the restored ground minus the bed.
    public static void WaterReplay(string folder, string osm, float seaLevel, string? reportPath = null)
    {
        // Water replay does not use native contracts or the historical water
        // sampler. Read its terrain directly instead of building that unrelated
        // index; the optional crossing audit reads the exact source separately.
        var input =
            JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder, "offline-design-pass-01-input.json")));

        if ((int?)input["schemaVersion"] != 2)
            throw new InvalidDataException("Unknown water replay capture");

        var scale = (float)input["HeightScaleMeters"]!;
        var heights =
            CapturedRunFiles.Heights(CapturedRunFiles.Resolve(folder, (string)input["terrainFile"]!), scale);

        var item =
            (
                Input: input,
                Terrain: heights,
                Scale: scale,
                Width: (double)input["widthMeters"]!);

        var resolution = TerrainSampling.Resolution(item.Terrain);
        var width = (float)item.Width;
        var half = width / 2;
        var spacing = width / (resolution - 1);
        var cell = CapturedRunFiles.WaterCellSize(folder, input);
        var request = item.Input["request"]!;
        var importRequest =
            new ImportRequest(
                (double)request["Latitude"]!,
                (double)request["Longitude"]!,
                new MapSizeProfile("replay", "replay", item.Width, false),
                false);

        var phaseWatch = System.Diagnostics.Stopwatch.StartNew();

        void Phase(string name)
        {
            Console.WriteLine($"  TIME {name}: {phaseWatch.Elapsed.TotalSeconds:0.0} s");
            phaseWatch.Restart();
        }

        // A folder of parsed caches: the file with the most waterways on this map. Only
        // files whose first node lies within 150 km of the map centre are loaded in full;
        // the folder holds several gigabytes of other cities.
        if (Directory.Exists(osm))
        {
            bool Near(string file)
            {
                try
                {
                    if (WaterReplayMetrics.FirstNode(file) is not var (latitude, longitude))
                        return false;

                    var local =
                        GeoMath.ToLocalMeters(latitude, longitude, importRequest.Latitude, importRequest.Longitude);

                    return math.length(local.xz) < 150000;
                }
                catch (IOException)
                {
                    return false;
                }
            }

            var files = Directory.GetFiles(osm, "*.bin");
            var near = files.Where(Near).ToArray();

            osm =
                (near.Length > 0 ? near : files)
                    .Select(file =>
                    {
                        try
                        {
                            var candidate = OsmParsedCache.LoadSnapshot(file, default);
                            var count =
                                candidate.Ways.Values.Count(
                                    way => way.Tags.ContainsKey("waterway")
                                        && way.NodeIds.Length > 0
                                        && candidate.Nodes.TryGetValue(way.NodeIds[0], out var node)
                                        && GeoMath.ToLocalMeters(
                                                node.Latitude,
                                                node.Longitude,
                                                importRequest.Latitude,
                                                importRequest.Longitude) is var local
                                        && Math.Abs(local.x) < half
                                        && Math.Abs(local.z) < half);

                            return (File: file, Count: count);
                        }
                        catch (Exception error) when (error is InvalidDataException or EndOfStreamException or IOException)
                        {
                            return (File: file, Count: 0);
                        }
                    })
                    .OrderByDescending(entry => entry.Count)
                    .First()
                    .File;
        }

        Console.WriteLine("water replay OSM: " + Path.GetFileName(osm));
        var data = OsmParsedCache.LoadSnapshot(osm, default);
        var polygons =
            CorrectiveCapturedReplay.ReadManifest<GeographicPolygon>(
                folder,
                (string)item.Input["waterPolygonsManifest"]!);

        float3 Local(long id)
        {
            var node = data.Nodes[id];
            return GeoMath.ToLocalMeters(
                node.Latitude,
                node.Longitude,
                importRequest.Latitude,
                importRequest.Longitude);
        }

        var types = new[] { "river", "stream", "canal", "drain", "ditch" };
        var lines =
            data.Ways.Values
                .Where(
                    way => way.Tags.TryGetValue("waterway", out var text)
                        && types.Contains(text)
                        && way.NodeIds.Length > 1
                        && way.NodeIds.All(data.Nodes.ContainsKey))
                .ToArray();

        var restore = new bool[resolution * resolution];

        foreach (var way in lines)
        {
            var reach =
                (NetworkEvidence.Measure(way.Tags, "width") ?? (way.Tags["waterway"] == "river" ? 32 : 12)) * .5f;

            var radius = Math.Max(reach, cell) / .6f + spacing;

            for (var i = 1; i < way.NodeIds.Length; i++)
            {
                var vectorA = Local(way.NodeIds[i - 1]);
                var vectorB = Local(way.NodeIds[i]);

                int Index(float v) => (int)Math.Round((v / width + .5) * (resolution - 1));

                var x0 = Math.Max(1, Index(Math.Min(vectorA.x, vectorB.x) - radius));
                var x1 = Math.Min(resolution - 2, Index(Math.Max(vectorA.x, vectorB.x) + radius));
                var z0 = Math.Max(1, Index(Math.Min(vectorA.z, vectorB.z) - radius));
                var z1 = Math.Min(resolution - 2, Index(Math.Max(vectorA.z, vectorB.z) + radius));

                for (var z = z0; z <= z1; z++)
                    for (var x = x0; x <= x1; x++)
                        if (NetworkNormalizer.DistanceToSegment(
                            new float3(x * spacing - half, 0, z * spacing - half),
                            vectorA,
                            vectorB)
                            <= radius)
                            restore[z * resolution + x] = true;
            }
        }

        Phase("load");
        var wet = WaterReplayMetrics.Inside(polygons, resolution, spacing, half);
        var cells = new List<int>();

        for (var i = 0; i < restore.Length; i++)
        {
            if (!restore[i])
                continue;

            if (wet[i])
            {
                restore[i] = false;
                continue;
            }

            cells.Add(i);
        }

        var ground = item.Terrain.ToArray();

        // Start from the highest neighbouring untouched ground so the iteration
        // converges from above instead of keeping the carved channel.
        for (var iteration = 0; iteration < 400; iteration++)
            foreach (var i in cells)
                ground[i] = (ground[i - 1] + ground[i + 1] + ground[i - resolution] + ground[i + resolution]) / 4;

        // WATER_DEM=<file>: a float32 grid of the source elevation in sea-relative metres, at the
        // map's resolution. It replaces the whole ground, so lakes and rivers are planned on the
        // ground the game planned on, not on its own water cuts in the captured heightmap.
        if (Environment.GetEnvironmentVariable("WATER_DEM") is { Length: > 0 } demFile && File.Exists(demFile))
        {
            var raw = File.ReadAllBytes(demFile);
            var dem = new float[raw.Length / 4];
            Buffer.BlockCopy(raw, 0, dem, 0, raw.Length);
            var replaced = 0;

            for (var i = 0; i < ground.Length && i < dem.Length; i++)
                if (!float.IsNaN(dem[i]))
                {
                    ground[i] = dem[i] + seaLevel;
                    replaced++;
                }

            Console.WriteLine($"  WATER_DEM replaced {replaced:N0} cells");
        }

        var original = ground;
        Console.WriteLine($"  restored {cells.Count:N0} cells near {lines.Length:N0} waterways");
        Phase("rebuild ground");
        var terrain = new TerrainBuilder.Result(original, item.Scale, seaLevel, 0, item.Scale);

        // Plans on the rebuilt ground with the earlier or the current channel cross-section,
        // so the two are compared like for like (report 117).
        WaterPlan Plan(bool earlierProfile, ImportLog planLog)
        {
            WaterBodyPlanner.EarlierChannelProfile = earlierProfile;

            try
            {
                return WaterBodyPlanner.Prepare(data, polygons, importRequest, terrain, planLog, default, cell);
            }
            finally
            {
                WaterBodyPlanner.EarlierChannelProfile = false;
            }
        }

        // The crossing report only needs the current plan.
        var earlierLog = new ImportLog();
        var earlierPlan = reportPath == null ? Plan(true, earlierLog) : null;

        if (earlierPlan != null)
            Phase("plan, earlier profile");

        var log = new ImportLog();
        var watch = System.Diagnostics.Stopwatch.StartNew();
        var plan = Plan(false, log);
        var traceDams = int.TryParse(Environment.GetEnvironmentVariable("WATER_DAMS"), out var shown) ? shown : 0;

        float Bed(float[] map, float3 point) => NetworkProcessor.Sample(point.x, point.z, map, item.Width);

        Console.WriteLine($"  planned in {watch.Elapsed.TotalSeconds:0.0} s");
        Console.WriteLine("  FINGERPRINT water plan " + PlanHash.Water(plan));

        if (earlierPlan != null)
            Console.WriteLine("  FINGERPRINT water plan, earlier profile " + PlanHash.Water(earlierPlan));

        // WATER_PROBE_LINE=x0,z0,x1,z1[,step]: the rebuilt ground, the planned bed and the
        // planned water surface along a line, in metres above the sea level given, to see
        // what the plan does across a river (Quebec City's St Lawrence, 11-18).
        if (Environment.GetEnvironmentVariable("WATER_PROBE_LINE") is { Length: > 0 } probeLine)
        {
            var values =
                probeLine.Split(',').Select(text => float.Parse(text, CultureInfo.InvariantCulture)).ToArray();

            var surface = WaterBodyPlanner.ConstructedSurfaceSampler(plan, polygons);
            var bed = plan.ConditionedHeights ?? original;
            var from = new float2(values[0], values[1]);
            var to = new float2(values[2], values[3]);
            var stepMeters = values.Length > 4 ? values[4] : 250;
            var length = math.distance(from, to);

            for (var along = 0f; along <= length; along += stepMeters)
            {
                var at = math.lerp(from, to, along / length);
                var point = new float3(at.x, 0, at.y);
                var level = surface(point) is float stage ? (stage - seaLevel).ToString("0.0") : "-";
                var nearest =
                    plan.ReachStages
                        .Select(reach =>
                        {
                            var delta = reach.End.xz - reach.Start.xz;
                            var t = math.clamp(math.dot(at - reach.Start.xz, delta) / Math.Max(1e-6f, math.lengthsq(delta)), 0, 1);
                            return (Reach: reach, Distance: math.distance(at, math.lerp(reach.Start.xz, reach.End.xz, t)), T: t);
                        })
                        .OrderBy(item => item.Distance)
                        .FirstOrDefault();

                var reachText =
                    nearest.Reach == null
                        ? "none"
                        : $"{nearest.Distance:0} m, level {math.lerp(nearest.Reach.StartLevel, nearest.Reach.EndLevel, nearest.T) - seaLevel:0.0}, polygon {nearest.Reach.PolygonId}";

                Console.WriteLine(
                    $"  PROBE x={at.x:0} z={at.y:0} ground={Bed(original, point) - seaLevel:0.0} bed={Bed(bed, point) - seaLevel:0.0} surface={level} nearestReach={reachText}");
            }
        }

        foreach (var name in new[]
            {
                "indexing mapped water",
                "mapped lakes",
                "connecting river reaches",
                "sampling contained river stage bounds",
                "solving directed water stages",
                "conditioning river beds and placing controls",
                "publishing conditioned water terrain",
                "smoothing newly carved banks",
                "checking lake containment"
                })
            Console.WriteLine($"    PHASE {name}: {log.Metric($"Water plan: {name} time (s)"):0.00} s");
        Phase("plan, current profile");

        foreach (var line in log.Lines.Where(
            otherText => otherText.StartsWith("Rivers:") || otherText.StartsWith("Water:")))
            Console.WriteLine("  " + line);

        var planned = plan.ConditionedHeights ?? original;

        // Dry river banks (report 116): across every channel reach, the steepest step
        // between neighbouring grid samples where both stand over half a metre above the
        // water, out to 200 m each side. The capture's own terrain carries the cut it was
        // planned with, so the two lines compare the earlier banks with the current ones.
        void BankSlopes(string label, float[] map)
        {
            var steepest = new List<float>();

            foreach (var reach in plan.ReachStages.Where(reach => reach.PolygonId == null))
            {
                var middle = (reach.Start + reach.End) * .5f;
                var level = (reach.StartLevel + reach.EndLevel) * .5f;
                var along = math.normalizesafe((reach.End - reach.Start).xz);
                var across = new float3(-along.y, 0, along.x);

                foreach (var side in new[] { -1, 1 })
                {
                    var worst = 0f;

                    for (var d = spacing; d <= 200; d += spacing)
                    {
                        var a = Bed(map, middle + across * (side * (d - spacing)));
                        var b = Bed(map, middle + across * (side * d));

                        if (a > level + .5f && b > level + .5f)
                            worst = Math.Max(worst, Math.Abs(b - a) / spacing);
                    }

                    steepest.Add(worst);
                }
            }

            if (steepest.Count == 0)
                return;

            steepest.Sort();
            float Share(float limit) => 100f * steepest.Count(slope => slope > limit) / steepest.Count;

            Console.WriteLine(
                $"  BANKS {label}: steepest dry slope P50 {steepest[steepest.Count / 2]:0.00}, P90 {steepest[steepest.Count * 9 / 10]:0.00}, P99 {steepest[steepest.Count * 99 / 100]:0.00}; over 0.7 (35°) {Share(.7f):0.0}%, over 1.0 (45°) {Share(1f):0.0}% of {steepest.Count} bank sides");
        }

        var earlierPlanned = earlierPlan?.ConditionedHeights;
        BankSlopes("captured (earlier cut, true ground)", item.Terrain);

        if (earlierPlanned != null)
            BankSlopes("earlier profile (rebuilt ground)", earlierPlanned);

        BankSlopes("planned now (rebuilt ground)", planned);

        // Cut-only figures: every waterway reach of the current plan, polygon rivers
        // included, out to 150 m.
        var band = WaterReplayMetrics.NearestLevel(plan.ReachStages, resolution, spacing, half, 200, out var bandDistance);

        if (earlierPlanned != null)
        {
            WaterReplayMetrics.CutBanks("earlier profile", original, earlierPlanned, band, bandDistance, wet, resolution, spacing);
            double outside = 0;

            for (var i = 0; i < planned.Length; i++)
                if (float.IsNaN(band[i]))
                    outside += Math.Abs(earlierPlanned[i] - planned[i]);

            Console.WriteLine(
                FormattableString.Invariant(
                    $"  CUT profiles differ outside the 200 m band by {outside * spacing * spacing / 1e6:0.000} million m³"));
        }

        WaterReplayMetrics.CutBanks("planned now", original, planned, band, bandDistance, wet, resolution, spacing);
        Phase("bank metrics");

        // WATER_RENDER="x,z,half,file.png": shaded relief (3x vertical, 1 m contours) of that window, the capture's
        // terrain (earlier cut) left and the planned terrain right, at 1.5 m a pixel, light
        // from the north-west. Ground below the nearest channel reach's level within 150 m
        // is drawn blue, darker with depth.
        if (Environment.GetEnvironmentVariable("WATER_RENDER")?.Split(',') is { Length: 4 } render)
        {
            var cx = float.Parse(render[0], System.Globalization.CultureInfo.InvariantCulture);
            var cz = float.Parse(render[1], System.Globalization.CultureInfo.InvariantCulture);
            var window = float.Parse(render[2], System.Globalization.CultureInfo.InvariantCulture);
            const float Pixel = 1.5f;
            var size = (int)(2 * window / Pixel);
            var rgb = new byte[size * 2 * size * 3];

            var nearby =
                plan.ReachStages
                    .Where(
                        reach => reach.PolygonId == null
                            && math.distance(((reach.Start + reach.End) * .5f).xz, new float2(cx, cz)) < window + 300)
                    .ToArray();

            float Level(float3 point)
            {
                var best = float.NaN;
                var nearest = 150f;

                foreach (var reach in nearby)
                {
                    var distance = NetworkNormalizer.DistanceToSegment(point, reach.Start, reach.End);

                    if (distance < nearest)
                    {
                        nearest = distance;
                        best = (reach.StartLevel + reach.EndLevel) * .5f;
                    }
                }

                return best;
            }

            for (var panel = 0; panel < 2; panel++)
            {
                var map = panel == 0 ? item.Terrain : planned;

                for (var row = 0; row < size; row++)
                    for (var column = 0; column < size; column++)
                    {
                        var point = new float3(cx - window + column * Pixel, 0, cz + window - row * Pixel);
                        var h = Bed(map, point);
                        var dx = Bed(map, point + new float3(Pixel, 0, 0)) - Bed(map, point - new float3(Pixel, 0, 0));
                        var dz = Bed(map, point + new float3(0, 0, Pixel)) - Bed(map, point - new float3(0, 0, Pixel));
                        // Three times vertical exaggeration, and a darker line at every whole metre.
                        var normal = math.normalize(new float3(-3 * dx, 2 * Pixel, -3 * dz));
                        var light = math.saturate(math.dot(normal, math.normalize(new float3(-1, 1.5f, 1))));
                        var shade = .15f + .85f * light;
                        var colour = new float3(.55f, .6f, .4f) * shade;
                        var right = Bed(map, point + new float3(Pixel, 0, 0));
                        var below = Bed(map, point - new float3(0, 0, Pixel));

                        if (Math.Floor(h) != Math.Floor(right) || Math.Floor(h) != Math.Floor(below))
                            colour *= .55f;
                        var level = Level(point);

                        if (!float.IsNaN(level) && h < level)
                            colour = math.lerp(colour, new float3(.1f, .3f, .55f), math.saturate(.4f + (level - h) / 8));

                        var at = (row * size * 2 + panel * size + column) * 3;
                        rgb[at] = (byte)(255 * math.saturate(colour.x));
                        rgb[at + 1] = (byte)(255 * math.saturate(colour.y));
                        rgb[at + 2] = (byte)(255 * math.saturate(colour.z));
                    }
            }

            // The cross-section through the nearest reach, every 7 m from 100 m on one side to
            // 100 m on the other: earlier cut, planned.
            var closest =
                nearby.OrderBy(reach => NetworkNormalizer.DistanceToSegment(new float3(cx, 0, cz), reach.Start, reach.End))
                    .FirstOrDefault();

            if (closest != null)
            {
                var middle = (closest.Start + closest.End) * .5f;
                var along = math.normalizesafe((closest.End - closest.Start).xz);
                var across = new float3(-along.y, 0, along.x);
                Console.WriteLine($"  SECTION at ({middle.x:0}, {middle.z:0}); water level {(closest.StartLevel + closest.EndLevel) * .5f:0.0}");

                for (var d = -100f; d <= 100f; d += 7)
                {
                    var p = middle + across * d;
                    Console.WriteLine($"  SECTION {d,5:0} m  earlier {Bed(item.Terrain, p),7:0.0}  planned {Bed(planned, p),7:0.0}");
                }
            }

            WritePng(render[3], size * 2, size, rgb);
            Console.WriteLine("  render saved: " + render[3]);
        }

        IReadOnlyList<PreparedSegment>? networks = null;
        object? crossingReport = null;

        if (reportPath != null)
        {
            var captured =
                CorrectiveCapturedReplay.ReadManifest<PreparedSegment>(folder, (string)input["networkManifest"]!);

            if (captured.Length != (int)input["sections"]!)
                throw new InvalidDataException("Captured source count mismatch");

            var channels =
                WaterBodyPlanner
                    .BuriedChannels(data, importRequest, cell)
                    .Concat(WaterBodyPlanner.OpenChannels(data, importRequest, cell, plan.MovedWaterwayNodes))
                    .ToArray();

            networks = NetworkStructureIntervals.CulvertBridges(captured, channels, resolution, width, log, default);
            crossingReport = WaterCrossings(captured, networks, channels, resolution, width);
        }

        var revalidated =
            WaterBodyPlanner.Revalidate(plan, terrain with { Heights = planned }, item.Width, log, default, networks);

        Console.WriteLine("  FINGERPRINT revalidated water plan " + PlanHash.Water(revalidated));

        Console.WriteLine(
            $"  sources {plan.Sources.Count:N0}, kept by Revalidate {revalidated.Sources.Count:N0} (rivers {plan.Sources.Count(river => river.BodyKind == "river"):N0} → {revalidated.Sources.Count(river => river.BodyKind == "river"):N0})");

        Phase("revalidate");

        // River line at risk of running dry in the game (report 117), for each profile with
        // the sources its own Revalidate keeps.
        void Connectivity(string label, WaterPlan kept, float[] map) =>
            WaterReplayMetrics.Connectivity(
                label,
                WaterReplayMetrics.WalkLines(
                    kept.ReachStages,
                    (x, z) => NetworkProcessor.Sample(x, z, map, item.Width),
                    kept.Sources
                        .Select(
                            river => new WaterReplayMetrics.Source(
                                river.Position.xz,
                                river.Radius,
                                river.TargetSurfaceLevel ?? river.Position.y + river.Depth))
                        .ToArray(),
                    cell,
                    WaterReplayMetrics.MinimumGridWeight));

        if (earlierPlan != null && earlierPlanned != null)
            Connectivity(
                "earlier profile",
                WaterBodyPlanner.Revalidate(
                    earlierPlan,
                    terrain with { Heights = earlierPlanned },
                    item.Width,
                    earlierLog,
                    default,
                    networks),
                earlierPlanned);

        Connectivity("planned now", revalidated, planned);
        Phase("connectivity");

        var keptIds =
            revalidated.Sources.Select(river => (river.BodyId, river.Position.x, river.Position.z)).ToHashSet();

        foreach (var river in plan.Sources
            .Where(otherRiver => !keptIds.Contains((otherRiver.BodyId, otherRiver.Position.x, otherRiver.Position.z)))
            .Where(otherRiver => otherRiver.BankProbes.Count > 0)
            .Take(traceDams))
        {
            var low = river.BankProbes.OrderBy(point => Bed(planned, point)).First();
            Console.WriteLine(
                $"    rejected {river.BodyKind} {river.BodyId} at ({river.Position.x:0},{river.Position.z:0}): stage {river.TargetSurfaceLevel:0.00} bank at planning {river.BankLevel:0.00}; lowest probe ({low.x:0},{low.z:0}) planned {Bed(planned, low):0.00} restored {Bed(original, low):0.00}; {river.BankProbes.Count} probes");
        }

        foreach (var kind in new[] { "lake", "river" })
            Console.WriteLine(
                $"    {kind} bodies with a source {plan.Sources.Where(river => river.BodyKind == kind).Select(river => river.BodyId).Distinct().Count():N0}, still with one after Revalidate {revalidated.Sources.Where(river => river.BodyKind == kind).Select(river => river.BodyId).Distinct().Count():N0}");

        foreach (var name in new[]{
            "Water stage bank conflicts",
            "Water source footprint policy rejected",
            "Water lakes left uncarved without a source",
            "Water lakes lowered to their escape",
            "Water lakes lowered to their escape after containment",
            "Water lakes dropped after containment",
            "Water earth dams built",
            "Water terrain cells restored from rejected lakes",
            "Water lakes given a source on a second try",
            "Water sea-level samples left to the sea without a source",
            "Water sources placed in wide river areas",
            "Water locally infeasible reaches",
            "Water final bank conflicts",
            "Water controls with bank probes on planned water",
            "Water levee cells raised",
            "River raster components reseeded",
            "Water stage uphill residual edges",
            "Water endpoint gaps connected",
                "Water line nodes moved to the valley floor",
                "Water buried waterways opened",
                "Water buried channel metres",
                "Lake containment bodies checked",
                "Lake containment levels lowered",
                "Lake containment bodies omitted",
                "Lake containment bodies too large to screen",
                "Water large lakes given wide sources"
                })
            Console.WriteLine($"    {name}: {log.Metric(name)}");

        // WATER_SOURCES_NEAR=x,z,radius: the planned sources inside that circle, and whether
        // each is still there after Revalidate.
        if (Environment.GetEnvironmentVariable("WATER_SOURCES_NEAR") is { Length: > 0 } circle)
        {
            var parts =
                circle.Split(',')
                    .Select(text => float.Parse(text, System.Globalization.CultureInfo.InvariantCulture))
                    .ToArray();

            foreach (var river in plan.Sources.Where(
                river => math.distance(river.Position.xz, new float2(parts[0], parts[1])) <= parts[2]))
                Console.WriteLine(
                    $"    SOURCE way {river.WayId} body {river.BodyId} {river.BodyKind} at ({river.Position.x:0},{river.Position.z:0}) bed {river.Position.y:0.0} depth {river.Depth:0.0} radius {river.Radius:0.0} grid {river.GridSnapped}; kept {revalidated.Sources.Any(other => math.distance(other.Position.xz, river.Position.xz) < .5f)}");
        }

        // WATER_NATIVE_BANKS=1: the apply-time bank check (NativeWaterBridge), replayed on the
        // game's own terrain as this run read it back: how many sources the game would skip
        // because a bank probe stands under their level there, and how many of those a level
        // 0.35 m under that bank would keep.
        if (Environment.GetEnvironmentVariable("WATER_NATIVE_BANKS") == "1")
        {
            var nativeMeta =
                JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder, "native-observed-terrain.json")));

            var native =
                CapturedRunFiles.Heights(
                    CapturedRunFiles.Resolve(folder, (string)nativeMeta["heightFile"]!),
                    (float)nativeMeta["scale"]!);

            var nativeWidth = (double)nativeMeta["width"]!;

            float Native(float3 point) => NetworkProcessor.Sample(point.x, point.z, native, nativeWidth);

            int skipped = 0, rescued = 0, tooFar = 0, tooThin = 0;
            var drops = new List<float>();

            foreach (var river in revalidated.Sources)
            {
                var target = river.TargetSurfaceLevel ?? river.Position.y + river.Depth;
                var bank = river.BankProbes.Aggregate(river.BankLevel, (lowest, probe) => Math.Min(lowest, Native(probe)));

                if (target <= bank - .25f)
                    continue;

                skipped++;
                drops.Add(target - (bank - .35f));

                if (target - (bank - .35f) > 1)
                    tooFar++;
                else if (bank - .35f - Native(river.Position) < .4f)
                    tooThin++;
                else
                    rescued++;
            }

            // Against what the game actually created (the snapshot after apply), so the check
            // above is calibrated: which planned sources are missing, and how many of those it
            // predicts.
            var snapshot =
                JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder, "native-snapshot-after-apply.json")));

            var created =
                (snapshot["water"] as JArray ?? new JArray())
                    .Select(item => new float2((float)item["x"]!, (float)item["z"]!))
                    .ToArray();

            int missing = 0, missingPredicted = 0, presentPredicted = 0;

            foreach (var river in revalidated.Sources)
            {
                var target = river.TargetSurfaceLevel ?? river.Position.y + river.Depth;
                var bank = river.BankProbes.Aggregate(river.BankLevel, (lowest, probe) => Math.Min(lowest, Native(probe)));
                var predicted = target > bank - .25f;
                var present = created.Any(point => math.distance(point, river.Position.xz) < .5f);

                if (!present)
                {
                    missing++;

                    if (predicted)
                        missingPredicted++;

                    if (missing <= 12)
                        Console.WriteLine(
                            $"    MISSING {river.BodyKind} way {river.WayId} at ({river.Position.x:0},{river.Position.z:0}) target {target:0.00} game bank {bank:0.00} game bed {Native(river.Position):0.00} grid {river.GridSnapped} radius {river.Radius:0.0}");
                }
                else if (predicted)
                    presentPredicted++;
            }

            Console.WriteLine(
                $"  NATIVE BANKS vs game: {created.Length:N0} created; {missing:N0} replayed sources missing in the game, {missingPredicted:N0} of them predicted by the bank check; {presentPredicted:N0} predicted but present");

            drops.Sort();
            Console.WriteLine(
                $"  NATIVE BANKS: of {revalidated.Sources.Count:N0} sources, {skipped:N0} stand above a bank on the game's terrain; lowered 0.35 m under it: kept {rescued:N0}, over 1 m lower {tooFar:N0}, under 0.4 m of water {tooThin:N0}; drop needed P50 {(drops.Count > 0 ? drops[drops.Count / 2] : 0):0.00}, P90 {(drops.Count > 0 ? drops[drops.Count * 9 / 10] : 0):0.00} m");
        }

        // WATER_LINES=<text>: the planner's diagnostic lines that contain it.
        if (Environment.GetEnvironmentVariable("WATER_LINES") is { Length: > 0 } wanted)
            foreach (var line in log.Lines.Where(line => line.Contains(wanted)))
                Console.WriteLine($"    LINE {line}");

        // The line as the planner carved it: nodes it moved onto the valley floor stand there.
        float3 Placed(long id) => plan.MovedWaterwayNodes.TryGetValue(id, out var target) ? target : Local(id);

        foreach (var pair in plan.MovedWaterwayNodes
            .OrderBy(pair => Bed(original, pair.Value) - Bed(original, Local(pair.Key)))
            .Take(traceDams))
            Console.WriteLine(
                $"    MOVED {pair.Key} ({Local(pair.Key).x:0},{Local(pair.Key).z:0}) to ({pair.Value.x:0},{pair.Value.z:0}): ground {Bed(original, Local(pair.Key)):0.0} to {Bed(original, pair.Value):0.0}");

        long samples = 0, dammed = 0, dams = 0, damSideLower = 0;
        var damSide = Environment.GetEnvironmentVariable("WATER_DAM_SIDE") == "1";

        // WATER_DAM_WHY=1: each dam (first dammed sample of a run) by what kept the line there:
        // its way moved no node, a node at either end of its segment is shared with another
        // waterway, or the segment is long (over 56 m) so the dam may sit between nodes.
        var damWhy = new Dictionary<string, int>();
        var nodeUse = new Dictionary<long, int>();

        foreach (var otherWay in lines)
            foreach (var id in otherWay.NodeIds.Distinct())
                nodeUse[id] = nodeUse.TryGetValue(id, out var uses) ? uses + 1 : 1;
        var cuts = new List<float>();
        var byType = new Dictionary<string, (long Samples, long Dammed)>();
        var stagesByWay =
            plan.ReachStages.GroupBy(reach => reach.WayId).ToDictionary(group => group.Key, group => group.ToArray());

        foreach (var way in lines.Where(
            otherWay => !(otherWay.Tags.TryGetValue("tunnel", out var otherText) && otherText != "no")
                && !(otherWay.Tags.TryGetValue("intermittent", out var secondText) && secondText == "yes")
                && otherWay.Tags["waterway"] is "river" or "stream"
                && !otherWay.Tags.ContainsKey("abandoned")
                && !otherWay.Tags.ContainsKey("disused")))
        {
            var lowest = float.PositiveInfinity;
            var blocked = false;
            var reaches = stagesByWay.TryGetValue(way.Id, out var list) ? list : Array.Empty<WaterReachStage>();

            for (var i = 1; i < way.NodeIds.Length; i++)
            {
                var vectorA = Placed(way.NodeIds[i - 1]);
                var vectorB = Placed(way.NodeIds[i]);

                if (Math.Max(Math.Abs(vectorA.x), Math.Abs(vectorA.z)) > half - 100
                    || Math.Max(Math.Abs(vectorB.x), Math.Abs(vectorB.z)) > half - 100)
                    continue;

                var steps = Math.Max(1, (int)(math.distance(vectorA.xz, vectorB.xz) / 7));

                for (var k = 0; k < steps; k++)
                {
                    var point = math.lerp(vectorA, vectorB, k / (float)steps);
                    var bed = Bed(planned, point);
                    samples++;
                    cuts.Add(Bed(original, point) - bed);
                    // Upstream water surface: the planned stage of the nearest reach.
                    var reach =
                        reaches
                            .Where(
                                otherReach => NetworkNormalizer.DistanceToSegment(
                                    point,
                                    otherReach.Start,
                                    otherReach.End)
                                    < 30)
                            .OrderBy(
                                otherReach => NetworkNormalizer.DistanceToSegment(
                                    point,
                                    otherReach.Start,
                                    otherReach.End))
                            .FirstOrDefault();

                    var dam = bed > lowest + .1f;
                    byType.TryGetValue(way.Tags["waterway"], out var tally);
                    tally.Samples++;

                    if (dam)
                        tally.Dammed++;

                    byType[way.Tags["waterway"]] = tally;

                    if (dam)
                    {
                        dammed++;

                        // WATER_DAM_SIDE=1: is there ground across the line, within the valley
                        // search's 112 m, low enough to carry this water (original ground no
                        // higher than the upstream low plus the 20 m channel cut)?
                        if (damSide)
                        {
                            var across =
                                math.normalizesafe(new float3(-(vectorB - vectorA).z, 0, (vectorB - vectorA).x));

                            var lower = false;

                            for (var offset = 14; offset <= 112 && !lower; offset += 14)
                                lower =
                                    Bed(original, point + across * offset) - 20 <= lowest + .1f
                                    || Bed(original, point - across * offset) - 20 <= lowest + .1f;

                            if (lower)
                                damSideLower++;
                        }

                        if (!blocked)
                        {
                            dams++;

                            if (Environment.GetEnvironmentVariable("WATER_DAM_WHY") == "1")
                            {
                                var wayMoved = way.NodeIds.Any(plan.MovedWaterwayNodes.ContainsKey);
                                var shared = nodeUse[way.NodeIds[i - 1]] > 1 || nodeUse[way.NodeIds[i]] > 1;
                                var length = math.distance(vectorA.xz, vectorB.xz);

                                var why =
                                    $"{(wayMoved ? "way moved" : "way not moved")}, "
                                        + $"{(shared ? "shared end" : "free ends")}, "
                                        + $"{(length > 56 ? "long segment" : "short segment")}";

                                damWhy[why] = damWhy.TryGetValue(why, out var seen) ? seen + 1 : 1;
                            }

                            if (traceDams-- > 0)
                                Console.WriteLine(
                                    $"    dam on {way.Tags["waterway"]} {way.Id} at ({point.x:0},{point.z:0}): bed {bed:0.0} over upstream low {lowest:0.0}; ground {Bed(original, point):0.0}; reach stage {plan.ReachStages.Where(otherReach => otherReach.WayId == way.Id).OrderBy(otherReach => math.distancesq(otherReach.Start.xz, point.xz)).Select(otherReach => otherReach.StartLevel.ToString("0.0")).FirstOrDefault() ?? "none"}");
                        }
                    }

                    blocked = dam;

                    if (reach != null)
                        lowest = Math.Min(lowest, Math.Min(reach.StartLevel, reach.EndLevel));
                }
            }
        }

        cuts.Sort();

        // A map with no open river or stream inside its edges has nothing to measure.
        if (cuts.Count == 0)
            cuts.Add(0);

        Console.WriteLine(
            $"  open waterways: {samples:N0} samples, dammed {dammed:N0} ({dammed / (double)Math.Max(1, samples):P1}) at {dams:N0} dams; cut depth median {cuts[cuts.Count / 2]:0.0} m, p95 {cuts[(int)(cuts.Count * .95)]:0.0} m, over 10 m {cuts.Count(v => v > 10):N0}");
        Phase("dam walk");

        foreach (var pair in damWhy.OrderByDescending(pair => pair.Value))
            Console.WriteLine($"    dams: {pair.Key}: {pair.Value}");

        if (damSide)
            Console.WriteLine(
                $"  dammed samples with low enough ground within 112 m across the line: {damSideLower:N0} of {dammed:N0}");

        Console.WriteLine(
            "    dammed by type: "
                + string.Join(
                    ", ",
                    byType
                        .OrderByDescending(entry => entry.Value.Samples)
                        .Select(
                            entry => $"{entry.Key} {entry.Value.Dammed / (double)entry.Value.Samples:P1} of {entry.Value.Samples:N0}")));

        // The planned water surface along each way, in the way's order: rises
        // over 5 cm are uphill water; stages under the sea's surface (sea level
        // plus the game's 1 m) would let the sea run up the channel.
        var uphill = 0;
        var reachCount = 0;

        foreach (var group in plan.ReachStages.GroupBy(reach => reach.WayId))
        {
            var previousLevel = float.PositiveInfinity;

            foreach (var reach in group)
            {
                reachCount++;

                if (reach.StartLevel > previousLevel + .05f)
                    uphill++;

                if (reach.EndLevel > reach.StartLevel + .05f)
                    uphill++;

                previousLevel = reach.EndLevel;
            }
        }

        Console.WriteLine(
            $"  water surface: {reachCount:N0} reaches, rising downstream {uphill:N0}; below the sea's surface {plan.ReachStages.Count(reach => Math.Min(reach.StartLevel, reach.EndLevel) < seaLevel + 1 - .05f):N0}");

        var wayType = lines.ToDictionary(way => way.Id, way => way.Tags["waterway"]);

        foreach (var group in plan.ReachStages
            .Where(reach => Bed(original, reach.Start) - reach.StartLevel > 15)
            .GroupBy(reach => reach.WayId)
            .OrderByDescending(otherGroup => otherGroup.Count())
            .Take(shown))
            Console.WriteLine(
                $"    deep: {group.Count()} reaches on way {group.Key}: {string.Join(", ", lines.First(way => way.Id == group.Key).Tags.Select(entry => entry.Key + "=" + entry.Value))}");

        foreach (var group in plan.ReachStages
            .Where(reach => Math.Min(reach.StartLevel, reach.EndLevel) < seaLevel + 1 - .05f)
            .GroupBy(reach => reach.WayId)
            .OrderByDescending(otherGroup => otherGroup.Count())
            .Take(shown))
            Console.WriteLine(
                $"    below sea: {(wayType.TryGetValue(group.Key, out var otherText) ? otherText : "?")} {group.Key} {group.Count()} reaches, lowest {group.Min(reach => Math.Min(reach.StartLevel, reach.EndLevel)):F1} at ({group.First().Start.x:F0},{group.First().Start.z:F0}), polygon {group.First().PolygonId}");

        Console.WriteLine(
            "  deepest cuts by waterway: "
                + string.Join(
                    ", ",
                    lines
                        .Where(way => way.Tags["waterway"] is "river" or "stream" or "canal")
                        .GroupBy(way => way.Tags["waterway"])
                        .Select(group =>
            {
                var ids = group.Select(way => way.Id).ToHashSet();
                return $"{group.Key} {plan.ReachStages.Count(reach => ids.Contains(reach.WayId) && Bed(original, reach.Start) - reach.StartLevel > 15):N0} reaches over 15 m under ground";
            })));

        Console.WriteLine(
            "  river sources by waterway: "
                + string.Join(
                    ", ",
                    plan.Sources
                        .Where(river => river.BodyKind == "river")
                        .GroupBy(river => wayType.TryGetValue(river.WayId, out var thirdText) ? thirdText : "?")
                        .OrderByDescending(group => group.Count())
                        .Select(group => $"{group.Key} {group.Count():N0}")));

        if (reportPath != null)
        {
            var buriedIds =
                lines
                    .Where(way => way.Tags.TryGetValue("tunnel", out var tag) && tag != "no")
                    .Select(way => way.Id)
                    .ToHashSet();

            long buriedSamples = 0, buriedObstructions = 0;

            foreach (var reach in plan.ReachStages.Where(otherReach => buriedIds.Contains(otherReach.WayId)))
            {
                var count = Math.Max(1, (int)Math.Ceiling(math.distance(reach.Start.xz, reach.End.xz) / 7));

                for (var k = 0; k < count; k++)
                {
                    var t = k / (float)count;
                    var point = math.lerp(reach.Start, reach.End, t);
                    buriedSamples++;

                    if (Bed(planned, point) > math.lerp(reach.StartLevel, reach.EndLevel, t) - .1f)
                        buriedObstructions++;
                }
            }

            var report = new
            {
                capture = Path.GetFileName(folder),
                sourceCache = osm,
                waterCellSize = cell,
                terrainReconstructed = true,
                nativeFlowVerified = false,
                sources = revalidated.Sources.Count,
                riverControls = revalidated.Sources.Count(river => river.BodyKind == "river"),
                streamControls = revalidated.Sources.Count(river => wayType.GetValueOrDefault(river.WayId) == "stream"),
                streamReaches = revalidated.ReachStages.Count(
                    reach => wayType.GetValueOrDefault(reach.WayId) == "stream"),
                containment = log.Lines
                    .Where(otherText => otherText.Contains("WATER CONTAINMENT") || otherText.Contains("Lake "))
                    .ToArray(),
                lakes = polygons
                    .Where(
                        polygon => polygon.Outer.Any(points => points.Length > 0)
                            && !polygon.Tags.ContainsKey("waterway"))
                    .Select(polygon =>
                {
                    var vertices = polygon.Outer.SelectMany(points => points).ToArray();
                    var center =
                        vertices.Aggregate(float3.zero, (vectorA, vectorB) => vectorA + vectorB) / vertices.Length;

                    return new
                    {
                        polygon.Id,
                        polygon.Identity,
                        polygon.Tags,
                        latitude = importRequest.Latitude + math.degrees(center.z / Earth),
                        longitude = importRequest.Longitude
                            + math.degrees(center.x / (Earth * Math.Cos(math.radians(importRequest.Latitude)))),
                        controls = revalidated.Sources.Count(
                            river => river.BodyId == polygon.Identity && river.BodyKind == "lake"),
                        stages = revalidated.AreaStages
                            .Where(stage => stage.BodyId == polygon.Identity)
                            .Select(stage => stage.Level)
                            .ToArray()
                    };
                }).ToArray(),
                lakeBodies = revalidated.Sources
                    .Where(river => river.BodyKind == "lake")
                    .Select(river => river.BodyId)
                    .Distinct()
                    .Count(),
                riverBodies = revalidated.Sources
                    .Where(river => river.BodyKind == "river")
                    .Select(river => river.BodyId)
                    .Distinct()
                    .Count(),
                openedBuriedWays = log.Metric("Water buried waterways opened"),
                buriedSamples,
                buriedObstructions,
                naturalOpenSamples = samples,
                naturalOpenObstructions = dammed,
                uphillStageTransitions = uphill,
                crossings = crossingReport
            };

            File.WriteAllText(reportPath, JsonConvert.SerializeObject(report, Formatting.Indented));
            Console.WriteLine("  crossing report: " + JsonConvert.SerializeObject(crossingReport));
        }

        // WATER_REPLAY_GROUND=prefix: the planned ground of each profile as raw float metres,
        // for the road replay's ROAD_REPLAY_TERRAIN (report 117).
        if (Environment.GetEnvironmentVariable("WATER_REPLAY_GROUND") is string groundPrefix && groundPrefix.Length > 0)
        {
            void Save(string name, float[] heights)
            {
                var raw = new byte[heights.Length * sizeof(float)];
                Buffer.BlockCopy(heights, 0, raw, 0, raw.Length);
                File.WriteAllBytes(groundPrefix + "." + name + ".f32", raw);
            }

            Save("current", planned);

            if (earlierPlanned != null)
                Save("earlier", earlierPlanned);
        }

        if (Environment.GetEnvironmentVariable("WATER_REPLAY_OUT") is string output && output.Length > 0)
        {
            var bytes = new byte[planned.Length * 2];
            Buffer.BlockCopy(planned, 0, bytes, 0, bytes.Length);
            File.WriteAllBytes(output, bytes);
            var restored = new byte[original.Length * 2];
            Buffer.BlockCopy(original, 0, restored, 0, restored.Length);
            File.WriteAllBytes(Path.ChangeExtension(output, ".restored.u16"), restored);
        }
    }

    // Top-down road footprints of two plot files side by side at one spot, at
    // native width: grey where one road covers the ground, red where two roads
    // of the same level overlap, blue for bridges and tunnels.
    public static void PlanPng(string before, string after, double lat, double lon, double half, string output)
    {
        var run = JObject.Parse(File.ReadAllText(Path.Combine(before, RunFile)));
        double lat0 = (double)run["latitude"]!, lon0 = (double)run["longitude"]!;
        var cx = math.radians(lon - lon0) * Earth * Math.Cos(math.radians(lat0));
        var cz = math.radians(lat - lat0) * Earth;
        const int size = 700;
        var metres = 2 * half / size;
        var width = size * 2 + 10;
        var rgb = new byte[width * size * 3];
        Array.Fill(rgb, (byte)30);

        for (var side = 0; side < 2; side++)
        {
            var coverage = new byte[size * size];
            var structure = new bool[size * size];

            using (var file =
                File.OpenRead(
                    Path.Combine(
                        side == 0 ? before : after,
                        PlotFile))) using (var zip =
                            new GZipStream(file, CompressionMode.Decompress)) using (var reader = new StreamReader(zip))
                while (reader.ReadLine() is { } line)
                {
                    var row = JObject.Parse(line);

                    if ((string)row["kind"]! != "Road")
                        continue;

                    var p = row["p"]!.Values<double>().ToArray();
                    var radius = (float)row["NativeWidth"]! / 2;
                    var curve =
                        new Cubic3d(
                            new double3(p[0], p[1], p[2]),
                            new double3(p[3], p[4], p[5]),
                            new double3(p[6], p[7], p[8]),
                            new double3(p[9], p[10], p[11]));

                    if (Math.Min(Math.Min(p[0], p[9]), Math.Min(p[3], p[6])) > cx + half + 50
                        || Math.Max(Math.Max(p[0], p[9]), Math.Max(p[3], p[6])) < cx - half - 50
                        || Math.Min(Math.Min(p[2], p[11]), Math.Min(p[5], p[8])) > cz + half + 50
                           || Math.Max(Math.Max(p[2], p[11]), Math.Max(p[5], p[8])) < cz - half - 50)
                        continue;

                    var mine = new HashSet<int>();
                    var steps = Math.Max(2, (int)(NetworkDesignChecks.HorizontalArcLength(curve) / .5));
                    var bridge = (bool)row["Bridge"]! || (bool)row["Tunnel"]!;

                    for (var s = 0; s <= steps; s++)
                    {
                        var point = NetworkDesignChecks.Evaluate(curve, s / (double)steps);
                        var px = (point.x - (cx - half)) / metres;
                        var pz = (cz + half - point.z) / metres;
                        var rr = radius / metres;

                        for (var y = (int)(pz - rr); y <= (int)(pz + rr); y++)
                            for (var x = (int)(px - rr); x <= (int)(px + rr); x++)
                                if (x >= 0
                                    && y >= 0
                                    && x < size
                                    && y < size
                                    && (x - px) * (x - px) + (y - pz) * (y - pz) <= rr * rr)
                                    mine.Add(y * size + x);
                    }

                    foreach (var i in mine)
                    {
                        if (bridge)
                            structure[i] = true;
                        else if (coverage[i] < 255)
                            coverage[i]++;
                    }
                }

            for (var y = 0; y < size; y++)
                for (var x = 0; x < size; x++)
                {
                    var i = y * size + x;
                    var k = (y * width + side * (size + 10) + x) * 3;

                    if (coverage[i] >= 2)
                    {
                        rgb[k] = 220;
                        rgb[k + 1] = 50;
                        rgb[k + 2] = 50;
                    }
                    else if (coverage[i] == 1)
                    {
                        rgb[k] = 150;
                        rgb[k + 1] = 150;
                        rgb[k + 2] = 150;
                    }

                    if (structure[i])
                    {
                        rgb[k] = (byte)(rgb[k] / 2 + 40);
                        rgb[k + 1] = (byte)(rgb[k + 1] / 2 + 70);
                        rgb[k + 2] = (byte)Math.Min(255, rgb[k + 2] / 2 + 150);
                    }
                }
        }

        WritePng(output, width, size, rgb);
        Console.WriteLine($"  plan {2 * half:0} m square, {metres:0.00} m per pixel → {output}");
    }

    private static void WritePng(string path, int width, int height, byte[] rgb)
    {
        using var file = File.Create(path);

        void Chunk(string type, byte[] data)
        {
            var header = Encoding.ASCII.GetBytes(type);
            var length = BitConverter.GetBytes(data.Length);
            Array.Reverse(length);
            file.Write(length);
            file.Write(header);
            file.Write(data);
            var crc = Crc(header.Concat(data));
            var bytes = BitConverter.GetBytes(crc);
            Array.Reverse(bytes);
            file.Write(bytes);
        }

        static uint Crc(IEnumerable<byte> data)
        {
            var c = 0xFFFFFFFFu;

            foreach (var b in data)
            {
                c ^= b;

                for (var k = 0; k < 8; k++)
                    c = (c & 1) != 0 ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            }

            return c ^ 0xFFFFFFFFu;
        }

        file.Write(new byte[] { 137, 80, 78, 71, 13, 10, 26, 10 });
        var ihdr = new byte[13];
        var wb = BitConverter.GetBytes(width);
        var hb = BitConverter.GetBytes(height);
        Array.Reverse(wb);
        Array.Reverse(hb);
        Array.Copy(wb, 0, ihdr, 0, 4);
        Array.Copy(hb, 0, ihdr, 4, 4);
        ihdr[8] = 8;
        ihdr[9] = 2;
        Chunk("IHDR", ihdr);
        using var raw = new MemoryStream();

        using (var stream = new ZLibStream(raw, CompressionLevel.Optimal, true))
            for (var y = 0; y < height; y++)
            {
                stream.WriteByte(0);
                stream.Write(rgb, y * width * 3, width * 3);
            }

        Chunk("IDAT", raw.ToArray());
        Chunk("IEND", Array.Empty<byte>());
    }

    public static void Compiler(string folder, string output)
    {
        var capture = Load(folder);

        if (Environment.GetEnvironmentVariable("VISUAL_RESOLVE_WATER_INTERVALS") == "1")
            capture =
                capture with
                {
                    Source =
                        NetworkStructureIntervals
                            .StructureIntervals(
                                capture.Source,
                                capture.Terrain,
                                capture.Width,
                                new ImportLog(),
                                default,
                                capture.Water)
                            .ToArray()
                };

        Measure(capture, output, "visual-compiler",
        (otherCapture, log) =>
{
    if (Environment.GetEnvironmentVariable("VISUAL_DEBUG_SHORT") == "1")
        ShortSources(otherCapture.Source);

    // Captures predating the step have no bridges over unmapped highway
    // crossings; on newer captures it finds nothing more to do.
    var source = NetworkStructureIntervals.UnmappedCrossingBridges(otherCapture.Source, log, default);

    // The capture is taken after culvert bridges, so re-running them
    // with the planned river reaches standing in for the open channels
    // shows what later changes to that step add.
    if (Environment.GetEnvironmentVariable("VISUAL_CULVERTS") == "1")
    {
        var reaches =
            CorrectiveCapturedReplay.ReadManifest<WaterReachStage>(
                otherCapture.Folder,
                (string)otherCapture.Input["waterReachesManifest"]!);

        source =
            NetworkStructureIntervals.CulvertBridges(
                source,
                reaches
                    .Where(reach => reach.PolygonId == null)
                    .Select(reach => (new[] { reach.Start, reach.End }, reach.HalfWidth))
                    .ToArray(),
                (int)Math.Sqrt(otherCapture.Terrain.Length),
                otherCapture.Width,
                log,
                default);
    }

    if (Environment.GetEnvironmentVariable("VISUAL_PRUNE") == "1")
    {
        var pruneLog = new ImportLog();
        source =
            NetworkStubs.Prune(source, otherCapture.Input["request"]!.ToObject<ImportRequest>()!, pruneLog).ToArray();

        foreach (var line in pruneLog.Lines.Where(text => text.StartsWith("Odd connector", StringComparison.Ordinal)))
            Console.WriteLine("  " + line);
    }

    return VisualNetworkCompiler.Prepare(
        source,
        otherCapture.Terrain,
        otherCapture.Width,
        log,
        default,
        otherCapture.Water);
});
    }

    // Why a ground road misses the stamped terrain: the cut/fill clamp, a
    // structure end nearby, another road at a different height sharing raster
    // cells, or neither (raster resolution).
    private static void TerrainCauses(
        Capture capture,
        IReadOnlyList<PreparedSegment> sections,
        float[] roadbed,
        VisualCompilerMetrics.Measurement measured)
    {
        var points = new Dictionary<(int, int), List<(int Section, double3 P)>>();

        var ends = new List<double3>();

        for (var i = 0; i < sections.Count; i++)
        {
            var segment = sections[i];
            var curve =
                new Cubic3d(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd);

            var n = Math.Max(2, (int)(NetworkDesignChecks.HorizontalArcLength(curve) / 5) + 1);

            for (var k = 0; k < n; k++)
            {
                var point = NetworkDesignChecks.Evaluate(curve, k / (n - 1d));

                if (!points.TryGetValue(VisualCompilerMetrics.GridCell(point), out var items))
                    points[VisualCompilerMetrics.GridCell(point)] = items = new();

                items.Add((i, point));
            }

            if (NetworkStructure.IsStructure(segment))
            {
                ends.Add(curve.P0);
                ends.Add(curve.P3);
            }
        }

        var counts = new Dictionary<string, int>();
        var excess = new Dictionary<string, double>();

        foreach (var failure in measured.Failures.Where(otherFailure => otherFailure.Category == "terrain-mismatch"))
        {
            var segment = sections[failure.Section];
            var point = failure.Position;
            var original =
                NetworkProcessor.Sample((float)point.x, (float)point.z, capture.Terrain, capture.Width);

            var stamped =
                NetworkProcessor.Sample((float)point.x, (float)point.z, roadbed, capture.Width);

            string cause;

            bool NearTunnel()
            {
                var (tx, tz) = VisualCompilerMetrics.GridCell(point);

                for (var x = tx - 1; x <= tx + 1; x++)
                    for (var z = tz - 1; z <= tz + 1; z++)
                        if (points.TryGetValue((x, z), out var list))
                            foreach (var (j, otherPoint) in list)
                                if (sections[j].Tunnel
                                    && math.distance(otherPoint.xz, point.xz) < sections[j].NativeWidth / 2 + 30)
                                    return true;

                return false;
            }

            if (stamped - original > 1 && point.y < stamped - .3 && NearTunnel())
                cause =
                    "roof-cover"
                        + (sections.Any(
                            otherSegment => otherSegment.Tunnel
                                && otherSegment.Kind == segment.Kind
                                && (otherSegment.StartNodeId == segment.StartNodeId
                                    || otherSegment.StartNodeId == segment.EndNodeId
                                    || otherSegment.EndNodeId == segment.StartNodeId
                                    || otherSegment.EndNodeId == segment.EndNodeId))
                            ? " portal"
                            : " beside");
            else if (Math.Abs(point.y - original) > NetworkVerticalProfiles.CutFillBudget(segment) - .1)
                cause = "clamp";
            else if (ends.Any(e => math.distance(e.xz, point.xz) < 60))
                // Approach: this road leads onto the structure (a structure end
                // node on it within 60 m); otherwise it only passes near one.
                cause = "structure-end" + (Environment.GetEnvironmentVariable("VISUAL_DEBUG_TERRAIN") != "2" ? "" :
                    sections.Any(
                        otherSegment => NetworkStructure.IsStructure(otherSegment)
                            && otherSegment.Kind == segment.Kind
                            && new[] {
                                (otherSegment.StartNodeId, otherSegment.CourseStart),
                                (otherSegment.EndNodeId, otherSegment.CourseEnd)
                                }
                        .Any(
                            item => (item.Item1 == segment.StartNodeId || item.Item1 == segment.EndNodeId)
                                && math.distance(((double3)item.Item2).xz, point.xz) < 60))
                            ? " approach"
                            : " beside");
            else
            {
                var (cx, cz) = VisualCompilerMetrics.GridCell(point);
                var shared = false;

                for (var x = cx - 1; x <= cx + 1 && !shared; x++)
                    for (var z = cz - 1; z <= cz + 1 && !shared; z++)
                        if (points.TryGetValue((x, z), out var list))
                            foreach (var (j, otherPoint) in list)
                            {
                                if (j == failure.Section || NetworkStructure.IsStructure(sections[j]))
                                    continue;

                                if (math.distance(otherPoint.xz, point.xz)
                                    < segment.NativeWidth / 2
                                        + sections[j].NativeWidth / 2
                                        + capture.Width / Math.Sqrt(capture.Terrain.Length)
                                    && Math.Abs(otherPoint.y - point.y) > .3)
                                {
                                    shared = true;
                                    break;
                                }
                            }

                cause = shared ? "shared-cells" : "resolution";
            }

            cause =
                (segment.Kind == NetworkKind.Rail ? "rail " : "") + cause + (point.y > stamped ? "/above" : "/below");

            // VISUAL_DEBUG_TERRAIN=2 lists the first structure-end failures.
            if (Environment.GetEnvironmentVariable("VISUAL_DEBUG_TERRAIN") == "2"
                && cause.StartsWith("structure-end")
                && (counts.TryGetValue(cause, out var seen) ? seen : 0) < 5)
            {
                var nearest =
                    sections
                        .Where(NetworkStructure.IsStructure)
                        .Select(
                            otherSegment => (
                                otherSegment,
                                d: Math.Min(
                                    math.distance(((double3)otherSegment.CourseStart).xz, point.xz),
                                    math.distance(((double3)otherSegment.CourseEnd).xz, point.xz))))
                        .OrderBy(item => item.d)
                        .First();

                Console.WriteLine(
                    $"    {cause} way {segment.WayId} {segment.Classification} w{segment.NativeWidth:F0} at ({point.x:F0},{point.z:F0}): road {point.y:F1} stamped {stamped:F1} original {original:F1}; {(nearest.otherSegment.Tunnel ? "tunnel" : "bridge")} end {nearest.d:F0} m (way {nearest.otherSegment.WayId}); section length {NetworkDesignChecks.HorizontalArcLength(new Cubic3d(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd)):F0}");
            }

            counts[cause] = counts.TryGetValue(cause, out var n) ? n + 1 : 1;
            excess[cause] = (excess.TryGetValue(cause, out var e) ? e : 0) + failure.Value;
        }

        foreach (var entry in counts.OrderByDescending(otherEntry => otherEntry.Value))
            Console.WriteLine(
                $"  terrain cause {entry.Key} = {entry.Value}; mean {excess[entry.Key] / entry.Value:F2} m");
    }

    // What a same-level overlap is made of: the nearest other road at the worst
    // point, by direction (opposite carriageway, merge/diverge, crossing) and
    // class pair; tangent breaks and acute junctions by class.
    private static void OverlapCauses(
        IReadOnlyList<PreparedSegment> sections,
        VisualCompilerMetrics.Measurement measured)
    {
        var points = new Dictionary<(int, int), List<(int Section, double3 P, double2 D)>>();

        for (var i = 0; i < sections.Count; i++)
        {
            var segment = sections[i];

            if (segment.Kind != NetworkKind.Road)
                continue;

            var curve =
                new Cubic3d(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd);

            var n = Math.Max(2, (int)(NetworkDesignChecks.HorizontalArcLength(curve) / 2) + 1);

            for (var k = 0; k < n; k++)
            {
                var t = k / (n - 1d);
                var point = NetworkDesignChecks.Evaluate(curve, t);
                var d = math.normalizesafe(NetworkDesignChecks.Derivative(curve, t).xz);

                if (!points.TryGetValue(VisualCompilerMetrics.GridCell(point), out var items))
                    points[VisualCompilerMetrics.GridCell(point)] = items = new();

                items.Add((i, point, d));
            }
        }

        var counts = new Dictionary<string, int>();

        void Add(string key) => counts[key] = counts.TryGetValue(key, out var n) ? n + 1 : 1;

        // As in the metric, sections within two graph steps are connected, not overlapping.
        var incident = new Dictionary<long, List<int>>();

        for (var i = 0; i < sections.Count; i++)
            if (sections[i].Kind == NetworkKind.Road)
                foreach (var id in new[] { sections[i].StartNodeId, sections[i].EndNodeId })
                {
                    if (!incident.TryGetValue(id, out var list))
                        incident[id] = list = new();

                    list.Add(i);
                }

        IEnumerable<int> Touching(int i)
            => new[] { sections[i].StartNodeId, sections[i].EndNodeId }.SelectMany(id => incident[id]);

        foreach (var failure in measured.Failures)
        {
            var segment = sections[failure.Section];

            if (failure.Category == "tangent-break" || failure.Category == "acute-junction")
            {
                Add($"{failure.Category} {ClassOf(segment)}");
                continue;
            }

            if (failure.Category != "same-level-overlap")
                continue;

            var adjacent = new HashSet<int>(Touching(failure.Section).SelectMany(Touching));
            var (cx, cz) = VisualCompilerMetrics.GridCell(failure.Position);
            var best = (Section: -1, Distance: double.PositiveInfinity, D: default(double2));
            var own = default(double2);

            for (var x = cx - 1; x <= cx + 1; x++)
                for (var z = cz - 1; z <= cz + 1; z++)
                    if (points.TryGetValue((x, z), out var list))
                        foreach (var item in list)
                        {
                            var otherDistance = math.distance(item.P.xz, failure.Position.xz);

                            if (item.Section == failure.Section)
                            {
                                if (otherDistance < 3)
                                    own = item.D;

                                continue;
                            }

                            if (adjacent.Contains(item.Section))
                                continue;

                            if (otherDistance < best.Distance && Math.Abs(item.P.y - failure.Position.y) < 3)
                                best = (item.Section, otherDistance, item.D);
                        }

            if (best.Section < 0)
            {
                Add("overlap unknown");
                continue;
            }

            var otherSegment = sections[best.Section];
            var dot = Math.Abs(math.dot(own, best.D)) < 1e-6 ? 0 : math.dot(own, best.D);
            var relation = dot < -.9 ? "opposite" : dot > .9 ? "parallel" : Math.Abs(dot) < .5 ? "crossing" : "angled";
            Add(
                $"overlap {relation} {string.Join("+", new[] { ClassOf(segment), ClassOf(otherSegment) }.OrderBy(text => text))}{(segment.OneWay && otherSegment.OneWay ? "" : " (two-way)")}");

            // VISUAL_DEBUG_OVERLAP=2 also lists each parallel overlap with both roads' ways and end nodes.
            if (Environment.GetEnvironmentVariable("VISUAL_DEBUG_OVERLAP") == "2" && relation == "parallel")
                Console.WriteLine(
                    $"    {ClassOf(segment)} way {segment.WayId} {segment.StartNodeId}->{segment.EndNodeId} L{segment.Lanes} w{segment.NativeWidth:F0} / {ClassOf(otherSegment)} way {otherSegment.WayId} {otherSegment.StartNodeId}->{otherSegment.EndNodeId} L{otherSegment.Lanes} w{otherSegment.NativeWidth:F0} at ({failure.Position.x:F0},{failure.Position.z:F0}) gap {best.Distance:F1} m");

            // VISUAL_DEBUG_OVERLAP=3 lists every overlap with the graph distance between the two sections.
            if (Environment.GetEnvironmentVariable("VISUAL_DEBUG_OVERLAP") == "3")
            {
                var hops = new Dictionary<int, int> { [failure.Section] = 0 };
                var parent = new Dictionary<int, int>();
                var front = new List<int> { failure.Section };

                for (var h = 1; h <= 8 && !hops.ContainsKey(best.Section); h++)
                {
                    var next = new List<int>();

                    foreach (var a in front)
                        foreach (var b in Touching(a))
                            if (hops.TryAdd(b, h))
                            {
                                parent[b] = a;
                                next.Add(b);
                            }

                    front = next;
                }

                var path = new List<int>();

                if (hops.ContainsKey(best.Section))
                    for (var at =
                        parent.GetValueOrDefault(
                            best.Section,
                            -1); at >= 0 && at != failure.Section; at = parent.GetValueOrDefault(at, -1))
                        path.Add(at);

                string Degree(long id) => incident[id].Count.ToString();

                var via = string.Join(" ", path.Select(k =>
{
    var secondSegment = sections[k];
    return $"[{ClassOf(secondSegment)}{(secondSegment.OneWay ? "" : "2")} w{secondSegment.NativeWidth:F0} {NetworkDesignChecks.HorizontalLength(secondSegment):F0}m d{Degree(secondSegment.StartNodeId)}/{Degree(secondSegment.EndNodeId)}]";
}));

                Console.WriteLine(
                    $"    overlap3{(segment.Roundabout || otherSegment.Roundabout || path.Any(k => sections[k].Roundabout) ? " ring" : "")} {relation} hops={(hops.TryGetValue(best.Section, out var hop) ? hop : 99)} depth={failure.Value:F1} {ClassOf(segment)}{(segment.OneWay ? "" : "2")} way {segment.WayId} L{segment.Lanes} w{segment.NativeWidth:F0} len {NetworkDesignChecks.HorizontalLength(segment):F0} {segment.StartNodeId}->{segment.EndNodeId} / {ClassOf(otherSegment)}{(otherSegment.OneWay ? "" : "2")} way {otherSegment.WayId} L{otherSegment.Lanes} w{otherSegment.NativeWidth:F0} len {NetworkDesignChecks.HorizontalLength(otherSegment):F0} {otherSegment.StartNodeId}->{otherSegment.EndNodeId} at ({failure.Position.x:F0},{failure.Position.z:F0}) via {via}");
            }
        }

        // VISUAL_DEBUG_CLEARANCE=1 lists each low crossing with the bridge and the
        // lower road or rail it passes over, and both heights.
        if (Environment.GetEnvironmentVariable("VISUAL_DEBUG_CLEARANCE") == "1")
        {
            var all = new Dictionary<(int, int), List<(int Section, double3 P)>>();

            for (var i = 0; i < sections.Count; i++)
            {
                var segment = sections[i];
                var curve =
                    new Cubic3d(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd);

                var n = Math.Max(2, (int)(NetworkDesignChecks.HorizontalArcLength(curve) / 2) + 1);

                for (var k = 0; k < n; k++)
                {
                    var point = NetworkDesignChecks.Evaluate(curve, k / (n - 1d));

                    if (!all.TryGetValue(VisualCompilerMetrics.GridCell(point), out var items))
                        all[VisualCompilerMetrics.GridCell(point)] = items = new();

                    items.Add((i, point));
                }
            }

            foreach (var failure in measured.Failures
                .Where(otherFailure => otherFailure.Category == "crossing-clearance")
                .OrderBy(otherFailure => otherFailure.Value))
            {
                var segment = sections[failure.Section];
                var (cx, cz) = VisualCompilerMetrics.GridCell(failure.Position);
                var best = (Section: -1, Distance: double.PositiveInfinity, Y: 0d);

                for (var x = cx - 1; x <= cx + 1; x++)
                    for (var z = cz - 1; z <= cz + 1; z++)
                        if (all.TryGetValue((x, z), out var list))
                            foreach (var item in list)
                            {
                                var otherSegment = sections[item.Section];

                                if (item.Section == failure.Section
                                    || !(otherSegment.Layer < segment.Layer || otherSegment.Tunnel))
                                    continue;

                                var otherDistance = math.distance(item.P.xz, failure.Position.xz);

                                if (otherDistance < best.Distance)
                                    best = (item.Section, otherDistance, item.P.y);
                            }

                if (best.Section < 0)
                    continue;

                var secondSegment = sections[best.Section];

                string Describe(PreparedSegment firstSegment)
                    => $"{firstSegment.Kind} {firstSegment.Classification} L{firstSegment.Layer}{(firstSegment.Bridge ? " B" : "")}{(firstSegment.Tunnel ? " T" : "")} way {firstSegment.WayId} len {NetworkDesignChecks.HorizontalLength(firstSegment):F0} ends {firstSegment.CourseStart.y:F1}/{firstSegment.CourseEnd.y:F1}";

                Console.WriteLine(
                    $"    clearance gap={failure.Value:F1} deck {failure.Position.y:F1} over {best.Y:F1} at ({failure.Position.x:F0},{failure.Position.z:F0}): {Describe(segment)} / {Describe(secondSegment)}");
            }
        }

        if (Environment.GetEnvironmentVariable("VISUAL_DEBUG_CROSSINGS") == "1")
            Crossings(sections, "final", true);

        foreach (var entry in counts.OrderByDescending(otherEntry => otherEntry.Value).Take(24))
            Console.WriteLine($"  cause {entry.Key} = {entry.Value}");

        JunctionDistances(sections, measured);
    }

    // VISUAL_DEBUG_CROSSINGS=1: road centrelines that cross without a junction
    // (no shared node within two steps) less than 4 m apart in height, on any
    // layers: roads that visibly pass through each other.
    private static void Crossings(IReadOnlyList<PreparedSegment> sections, string label, bool print)
    {
        var incident = new Dictionary<long, List<int>>();

        for (var i = 0; i < sections.Count; i++)
            if (sections[i].Kind == NetworkKind.Road)
                foreach (var id in new[] { sections[i].StartNodeId, sections[i].EndNodeId })
                {
                    if (!incident.TryGetValue(id, out var list))
                        incident[id] = list = new();

                    list.Add(i);
                }

        IEnumerable<int> Touching(int i)
            => new[] { sections[i].StartNodeId, sections[i].EndNodeId }.SelectMany(id => incident[id]);

        {
            var pieces = new Dictionary<(int, int), List<(int Section, double3 A, double3 B)>>();

            for (var i = 0; i < sections.Count; i++)
            {
                var segment = sections[i];

                if (segment.Kind != NetworkKind.Road)
                    continue;

                var curve =
                    new Cubic3d(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd);

                var n = Math.Max(2, (int)(NetworkDesignChecks.HorizontalArcLength(curve) / 2) + 1);
                var previous = NetworkDesignChecks.Evaluate(curve, 0);

                for (var k = 1; k < n; k++)
                {
                    var point = NetworkDesignChecks.Evaluate(curve, k / (n - 1d));

                    foreach (var item in new[] {
                        VisualCompilerMetrics.GridCell(previous),
                        VisualCompilerMetrics.GridCell(point)
                        }.Distinct())
                    {
                        if (!pieces.TryGetValue(item, out var items))
                            pieces[item] = items = new();

                        items.Add((i, previous, point));
                    }

                    previous = point;
                }
            }

            var seen = new HashSet<(int, int)>();
            var byKind = new Dictionary<string, int>();

            foreach (var cell in pieces.Values)
                for (var x = 0; x < cell.Count; x++)
                    for (var y = x + 1; y < cell.Count; y++)
                    {
                        var (i, vectorA, vectorB) = cell[x];
                        var (j, c, d) = cell[y];

                        if (i == j)
                            continue;

                        var key = (Math.Min(i, j), Math.Max(i, j));

                        if (seen.Contains(key))
                            continue;

                        var r = vectorB.xz - vectorA.xz;
                        var point = d.xz - c.xz;
                        var den = r.x * point.y - r.y * point.x;

                        if (Math.Abs(den) < 1e-9)
                            continue;

                        var t = ((c.x - vectorA.x) * point.y - (c.z - vectorA.z) * point.x) / den;
                        var u = ((c.x - vectorA.x) * r.y - (c.z - vectorA.z) * r.x) / den;

                        if (t < 0 || t > 1 || u < 0 || u > 1)
                            continue;

                        var high = math.lerp(vectorA.y, vectorB.y, t);
                        var low = math.lerp(c.y, d.y, u);

                        if (Math.Abs(high - low) >= 4)
                            continue;

                        var near = new HashSet<int>(Touching(i).SelectMany(Touching));

                        if (near.Contains(j))
                            continue;

                        seen.Add(key);
                        var segment = sections[i];
                        var otherSegment = sections[j];
                        var otherPoint = math.lerp(vectorA, vectorB, t);

                        string Tag(PreparedSegment secondSegment)
                            => $"{ClassOf(secondSegment)}{(secondSegment.Bridge ? "B" : secondSegment.Tunnel ? "T" : "")}L{secondSegment.Layer}";

                        var kind = string.Join("+", new[] { Tag(segment), Tag(otherSegment) }.OrderBy(text => text));
                        byKind[kind] = byKind.GetValueOrDefault(kind) + 1;

                        if (print)
                            Console.WriteLine(
                                $"    crossing dy={high - low:F1} at ({otherPoint.x:F0},{otherPoint.z:F0}) {Tag(segment)} way {segment.WayId} {segment.StartNodeId}->{segment.EndNodeId} / {Tag(otherSegment)} way {otherSegment.WayId} {otherSegment.StartNodeId}->{otherSegment.EndNodeId}");
                    }

            Console.WriteLine(
                $"  {label} crossings without junction: {seen.Count}; "
                    + string.Join(
                        ", ",
                        byKind
                            .OrderByDescending(entry => entry.Value)
                            .Take(12)
                            .Select(entry => $"{entry.Key}={entry.Value}")));
        }
    }

    private static void JunctionDistances(
        IReadOnlyList<PreparedSegment> sections,
        VisualCompilerMetrics.Measurement measured)
    {
        // Distance from each overlap to the nearest road junction (degree three or more).
        var junctions =
            sections
                .Where(segment => segment.Kind == NetworkKind.Road)
                .SelectMany(
                    segment => new[] {
                        (segment.StartNodeId, (double3)segment.CourseStart),
                        (segment.EndNodeId, (double3)segment.CourseEnd)
                        })
            .GroupBy(item => item.Item1)
                .Where(group => group.Count() >= 3)
                .Select(group => group.First().Item2.xz)
                .ToArray();

        var junctionCells =
            junctions
                .GroupBy(point => ((int)Math.Floor(point.x / 100), (int)Math.Floor(point.y / 100)))
                .ToDictionary(group => group.Key, group => group.ToArray());

        var bands = new Dictionary<string, int>();

        foreach (var failure in measured.Failures.Where(otherFailure => otherFailure.Category == "same-level-overlap"))
        {
            var cx = (int)Math.Floor(failure.Position.x / 100);
            var cz = (int)Math.Floor(failure.Position.z / 100);
            var nearest = double.MaxValue;

            for (var dz = -3; dz <= 3; dz++)
                for (var dx = -3; dx <= 3; dx++)
                    if (junctionCells.TryGetValue((cx + dx, cz + dz), out var list))
                        foreach (var point in list)
                            nearest = Math.Min(nearest, math.distance(point, failure.Position.xz));

            var band = nearest < 30 ? "<30 m" : nearest < 100 ? "30-100 m" : nearest < 300 ? "100-300 m" : ">300 m";
            bands[band] = bands.TryGetValue(band, out var n) ? n + 1 : 1;
        }

        Console.WriteLine(
            "  overlap distance to nearest junction: "
                + string.Join("; ", bands.OrderBy(entry => entry.Key).Select(entry => $"{entry.Key}={entry.Value}")));
    }

    // Why a short source road section has kept nodes at both ends.
    private static void ShortSources(IReadOnlyList<PreparedSegment> source)
    {
        var incident =
            source
                .SelectMany(
                    (
                        segment,
                        i) => new[] {
                            (Key: new SourceNodeKey(segment.Kind, segment.StartNodeId), i),
                            (Key: new SourceNodeKey(segment.Kind, segment.EndNodeId), i)
                            })
            .GroupBy(item => item.Key)
                .ToDictionary(group => group.Key, group => group.Select(item => item.i).ToArray());

        string Reason(SourceNodeKey key)
        {
            var arms = incident[key];

            if (arms.Length == 1)
                return "end";

            if (arms.Length > 2)
                return "junction" + Math.Min(arms.Length, 4);

            var firstSegment = source[arms[0]];
            var secondSegment = source[arms[1]];

            if (firstSegment.Bridge != secondSegment.Bridge || firstSegment.Tunnel != secondSegment.Tunnel)
                return "structure";

            if (firstSegment.Layer != secondSegment.Layer)
                return "layer";

            if (firstSegment.OneWay != secondSegment.OneWay)
                return "oneway";

            if (firstSegment.Lanes != secondSegment.Lanes)
                return "lanes";

            if (firstSegment.Classification != secondSegment.Classification)
                return "class";

            if (firstSegment.NativeWidth != secondSegment.NativeWidth
                || firstSegment.NativeAssetKey != secondSegment.NativeAssetKey)
                return "asset";

            if (firstSegment.Reference != secondSegment.Reference)
                return "ref";

            if (firstSegment.ForwardLanes != secondSegment.ForwardLanes
                || firstSegment.BackwardLanes != secondSegment.BackwardLanes)
                return "dir-lanes";

            float2 Away(PreparedSegment segment)
                => math.normalizesafe(
                    (segment.StartNodeId == key.Id
                        ? segment.CourseControl1 - segment.CourseStart
                        : segment.CourseControl2 - segment.CourseEnd).xz);

            if (math.dot(Away(firstSegment), Away(secondSegment)) > -.707f)
                return "bend";

            if (firstSegment.WayId != secondSegment.WayId)
                return "way";

            return "plain";
        }

        var counts = new Dictionary<string, int>();

        foreach (var segment in source.Where(otherSegment => otherSegment.Kind == NetworkKind.Road))
        {
            var length =
                NetworkDesignChecks.HorizontalArcLength(
                    new(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd));

            if (length >= VisualCompilerTable.For(segment).MinimumSegment)
                continue;

            var pair =
                string.Join(
                    "+",
                    new[] {
                        Reason(new(segment.Kind, segment.StartNodeId)),
                        Reason(new(segment.Kind, segment.EndNodeId))
                        }.OrderBy(text => text));

            var key = $"{(segment.ProtectedLink ? "link" : segment.Classification)}:{pair}";
            counts[key] = counts.TryGetValue(key, out var n) ? n + 1 : 1;

            // Between two junctions: is it the gap between a divided cross street's carriageways?
            if (pair.StartsWith("junction") && pair.Contains("+junction"))
            {
                IEnumerable<(string, string)> OneWayNames(long id)
                    => incident[new(segment.Kind, id)]
                        .Select(k => source[k])
                        .Where(
                            otherSegment => otherSegment != segment
                                && otherSegment.OneWay
                                && otherSegment.Name.Length > 0)
                        .Select(otherSegment => (otherSegment.Name, otherSegment.Classification));

                var divided =
                    OneWayNames(segment.StartNodeId)
                        .Intersect(OneWayNames(segment.EndNodeId))
                        .Select(item => item.Item2)
                        .FirstOrDefault();

                var detail =
                    $"{(segment.ProtectedLink ? "link" : segment.Classification)}:junctions {(divided != null ? "across divided " + divided : "other")}";

                counts[detail] = counts.TryGetValue(detail, out var m) ? m + 1 : 1;
            }
        }

        foreach (var entry in counts.OrderByDescending(otherEntry => otherEntry.Value).Take(18))
            Console.WriteLine($"  short source {entry.Key} = {entry.Value}");
    }

    private static void Measure(
        Capture otherCapture,
        string output,
        string pipeline,
        Func<Capture, ImportLog, IReadOnlyList<PreparedSegment>> design)
    {
        var log = new ImportLog { ScalarDiagnostics = Environment.GetEnvironmentVariable("VISUAL_DIAGNOSTICS") == "1" };
        var watch = System.Diagnostics.Stopwatch.StartNew();
        var designed = design(otherCapture, log);

        if (log.DiagnosticsEnabled)
            foreach (var line in log.Lines.Where(
                text => text.Contains("RAIL OVERLAPPING ROUTE", StringComparison.Ordinal)
                    || text.Contains("RAIL PARALLEL ROUTES", StringComparison.Ordinal)
                    || text.Contains("VISUAL DEBUG", StringComparison.Ordinal)
                    || text.Contains("SHORT BRIDGES", StringComparison.Ordinal)
                    || text.Contains("LANE SLOTS", StringComparison.Ordinal)
                    || text.Contains("VISUAL COMPILER crossing", StringComparison.Ordinal)
                    || text.Contains("VISUAL COMPILER vertical", StringComparison.Ordinal)
                    || text.Contains("VISUAL COMPILER profile", StringComparison.Ordinal)
                    || text.Contains("VISUAL COMPILER horizontal", StringComparison.Ordinal)
                    || text.Contains("junction height mismatch", StringComparison.Ordinal)))
                Console.WriteLine("  diag " + line);

        var designSeconds = watch.Elapsed.TotalSeconds;
        var stage = System.Diagnostics.Stopwatch.StartNew();
        var sections =
            NativeDesignUnits.Prepare(
                StarterNetworkPolicy.WithoutSideStructures(designed),
                otherCapture.Contracts,
                log,
                default,
                pipeline.Contains("visual-compiler", StringComparison.Ordinal)
                    ? segment => VisualCompilerTable.For(segment).MinimumSegment
                    : null);

        var unitSeconds = stage.Elapsed.TotalSeconds;
        stage.Restart();
        // The same pre-submission rules NativeNetworkPlacement.CreateDefinitions
        // applies: a section failing one is omitted in game and leaves a gap.
        var sameNode = sections.Count(segment => segment.StartNodeId == segment.EndNodeId);
        var nonfinite =
            sections.Count(
                segment => !new[] {
                    segment.Start,
                    segment.End,
                    segment.CourseStart,
                    segment.CourseControl1,
                    segment.CourseControl2,
                    segment.CourseEnd
                    }.All(point => math.all(math.isfinite(point))));

        var noExtent =
            sections.Count(
                segment => !NetworkDesignChecks.HasHorizontalExtent(
                    new(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd)));

        // The game binds each section end to a live node no further than 32 m from
        // the plan (NativeBatchBindings.CenterBudget); the node sits a curve offset
        // away from the deck end, so a longer offset has the section refused.
        float LargestOffset(PreparedSegment segment)
            => Math.Max(math.length(segment.StartCurveOffset.xz), math.length(segment.EndCurveOffset.xz));

        var offsetOverShift = sections.Count(segment => LargestOffset(segment) > 12);
        var offsetOverBind = sections.Where(segment => LargestOffset(segment) > 32).ToArray();

        // The game's "backward" screen (NetworkVisualQuality): a control leg that
        // runs against the chord, so the piece turns back on itself.
        var backwardPieces =
            sections
                .Where(
                    segment => !segment.Roundabout
                        && NetworkCurveObservations
                            .Measure(
                                new Cubic3d(
                                    segment.CourseStart,
                                    segment.CourseControl1,
                                    segment.CourseControl2,
                                    segment.CourseEnd))
                            .MinimumControlProgressMeters
                        <= 0)
                .ToArray();

        // Lane alignment at one-way merges and splits: planned lateral error
        // between the lanes each movement connects, from the final geometry.
        var lanePlan = NativeLaneObligations.Build(sections);
        var mergeNodes =
            sections
                .SelectMany(segment => new[] { segment.StartNodeId, segment.EndNodeId })
                .GroupBy(id => id)
                .Where(group => group.Count() >= 3)
                .Select(group => group.Key)
                .ToHashSet();

        var laneErrors =
            lanePlan.Movements
                .Where(
                    otherObligation => mergeNodes.Contains(otherObligation.Node.Id)
                        && !double.IsNaN(otherObligation.PlannedLateralError))
                .Select(otherObligation => otherObligation.PlannedLateralError)
                .OrderBy(e => e)
                .ToArray();

        double LaneQ(double q) => laneErrors.Length == 0 ? 0 : laneErrors[(int)(q * (laneErrors.Length - 1))];

        Console.WriteLine(
            $"  lane alignment at merges/splits: {laneErrors.Length} movements; P50 {LaneQ(.5):F2} m, P90 {LaneQ(.9):F2} m, P99 {LaneQ(.99):F2} m; over 0.5 m {laneErrors.Count(e => e > .5)}, over 1.5 m {laneErrors.Count(e => e > 1.5)}");

        // Left-edge continuity where a one-way road changes width at a plain node:
        // the sideways step between the two arms' left edges at the node.
        {
            var oneWay =
                sections
                    .Where(segment => segment.Kind == NetworkKind.Road && segment.OneWay && !segment.Roundabout)
                    .SelectMany(
                        segment => new[] {
                            (Id: segment.StartNodeId, S: segment, Out: true),
                            (Id: segment.EndNodeId, S: segment, Out: false)
                            })
                    .GroupBy(item => item.Id);

            var steps = new List<double>();

            foreach (var group in oneWay)
            {
                var items = group.ToArray();

                if (items.Length != 2 || items[0].Out == items[1].Out)
                    continue;

                var down = items.First(item => item.Out).S;
                var up = items.First(item => !item.Out).S;

                if (Math.Abs(down.NativeWidth - up.NativeWidth) <= .5f)
                    continue;

                var t = math.normalizesafe((down.CourseControl1 - down.CourseStart).xz);
                var right = new float2(t.y, -t.x);

                if (math.dot(t, math.normalizesafe((up.CourseEnd - up.CourseControl2).xz)) < .82f)
                    continue;

                steps.Add(
                    Math.Abs(
                        math.dot(
                            down.CourseStart.xz
                                - right * down.NativeWidth / 2
                                - (up.CourseEnd.xz - right * up.NativeWidth / 2),
                            right)));
            }

            steps.Sort();
            Console.WriteLine(
                $"  left-edge step at width changes: {steps.Count} nodes; P50 {(steps.Count == 0 ? 0 : steps[steps.Count / 2]):F2} m; over 0.5 m {steps.Count(v => v > .5)}");
        }

        var nodeShape =
            sections
                .Where(segment => segment.Kind == NetworkKind.Road)
                .SelectMany(
                    segment => new[] {
                        (Id: segment.StartNodeId, Out: 1, segment.OneWay),
                        (Id: segment.EndNodeId, Out: 0, segment.OneWay)
                        })
                .GroupBy(item => item.Id)
            .ToDictionary(
                group => group.Key,
                group => $"{group.Count(item => item.Out == 0 && item.OneWay)}in/{group.Count(item => item.Out == 1 && item.OneWay)}out/{group.Count(item => !item.OneWay)}twoway");

        foreach (var group in lanePlan.Movements
            .Where(
                otherObligation => mergeNodes.Contains(otherObligation.Node.Id)
                    && otherObligation.PlannedLateralError > .5)
            .GroupBy(otherObligation => otherObligation.Node.Id)
            .GroupBy(otherGroup => nodeShape.TryGetValue(otherGroup.Key, out var shape) ? shape : "?")
            .OrderByDescending(otherGroup => otherGroup.Count())
            .Take(6))
            Console.WriteLine($"  misaligned nodes shaped {group.Key}: {group.Count()}");

        if (Environment.GetEnvironmentVariable("VISUAL_DEBUG_LANES") == "1")
            foreach (var group in lanePlan.Movements
                .Where(
                    otherObligation => mergeNodes.Contains(otherObligation.Node.Id)
                        && otherObligation.PlannedLateralError > .5)
                .GroupBy(otherObligation => otherObligation.Node.Id)
                .Take(8))
            {
                Console.WriteLine(
                    $"  node {group.Key}: worst {group.Max(otherObligation => otherObligation.PlannedLateralError):F2} m");

                foreach (var segment in sections.Where(
                    otherSegment => otherSegment.StartNodeId == group.Key || otherSegment.EndNodeId == group.Key))
                {
                    var start = segment.StartNodeId == group.Key;
                    var point = start ? segment.CourseStart : segment.CourseEnd;
                    var d =
                        start
                            ? segment.CourseControl1 - segment.CourseStart
                            : segment.CourseEnd - segment.CourseControl2;

                    Console.WriteLine(
                        $"    way {segment.WayId} {(start ? "out" : "in")} {segment.Classification} lanes={segment.Lanes} native=[{string.Join(",", segment.NativeLaneOffsets.Select(o => o.ToString("F1")))}] tunnel={segment.Tunnel} bridge={segment.Bridge} at=({point.x:F1},{point.z:F1}) offset=({(start ? segment.StartCurveOffset : segment.EndCurveOffset).x:F2},{(start ? segment.StartCurveOffset : segment.EndCurveOffset).z:F2}) heading=({math.normalizesafe(d.xz).x:F2},{math.normalizesafe(d.xz).y:F2}) len={NetworkDesignChecks.HorizontalArcLength(new(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd)):F0}");
                }
            }

        // Physical breaks: a dead-end node within 3 m of another network end with
        // a different node id; the game builds them as two unconnected nodes.
        foreach (var kind in new[] { NetworkKind.Rail, NetworkKind.Road })
        {
            var ends =
                sections
                    .Where(segment => segment.Kind == kind)
                    .SelectMany(
                        segment => new[] {
                            (Id: segment.StartNodeId, P: segment.Start.xz, Y: segment.Start.y),
                            (Id: segment.EndNodeId, P: segment.End.xz, Y: segment.End.y)
                            })
                    .ToArray();

            var degreeOf = ends.GroupBy(item => item.Id).ToDictionary(group => group.Key, group => group.Count());
            var cells =
                ends
                    .GroupBy(item => ((int)Math.Floor(item.P.x / 4), (int)Math.Floor(item.P.y / 4)))
                    .ToDictionary(group => group.Key, group => group.ToArray());

            var joined =
                sections
                    .Where(segment => segment.Kind == kind)
                    .Select(
                        segment => (
                            Math.Min(segment.StartNodeId, segment.EndNodeId),
                            Math.Max(segment.StartNodeId, segment.EndNodeId)))
                    .ToHashSet();

            var dangling = ends.Where(item => degreeOf[item.Id] == 1).Where(item =>
            {
                var cx = (int)Math.Floor(item.P.x / 4);
                var cz = (int)Math.Floor(item.P.y / 4);

                for (var dz = -1; dz <= 1; dz++)
                    for (var dx = -1; dx <= 1; dx++)
                        if (cells.TryGetValue((cx + dx, cz + dz), out var near))
                            foreach (var otherItem in near)
                                if (otherItem.Id != item.Id
                                    && !joined.Contains(
                                        (Math.Min(otherItem.Id, item.Id), Math.Max(otherItem.Id, item.Id)))
                                    && math.distance(otherItem.P, item.P) < 3
                                    && Math.Abs(otherItem.Y - item.Y) < 3)
                                    return true;

                return false;
            }).ToArray();

            Console.WriteLine(
                $"  physical {kind} breaks (dead end within 3 m of another end): {dangling.Length}"
                    + (dangling.Length == 0
                        ? ""
                        : $"; e.g. {string.Join(" ", dangling.Take(4).Select(item => $"{item.Id}@({item.P.x:F0},{item.P.y:F0})"))}"));

            // Distinct nodes within 3 m of each other (not joined by a section): the
            // game joins nodes at one position, and wired such pairs into dead ends.
            var crowded = ends.GroupBy(item => item.Id).Select(group => group.First()).ToArray();
            var crowdCells =
                crowded
                    .GroupBy(item => ((int)Math.Floor(item.P.x / 3), (int)Math.Floor(item.P.y / 3)))
                    .ToDictionary(group => group.Key, group => group.ToArray());

            var close = crowded.Count(item =>
{
    var cx = (int)Math.Floor(item.P.x / 3);
    var cz = (int)Math.Floor(item.P.y / 3);

    for (var dz = -1; dz <= 1; dz++)
        for (var dx = -1; dx <= 1; dx++)
            if (crowdCells.TryGetValue((cx + dx, cz + dz), out var near)
                && near.Any(
                    otherItem => otherItem.Id != item.Id
                        && !joined.Contains((Math.Min(otherItem.Id, item.Id), Math.Max(otherItem.Id, item.Id)))
                        && math.distance(otherItem.P, item.P) < 3))
                return true;

    return false;
});

            Console.WriteLine($"  {kind} nodes within 3 m of another unjoined node: {close}");
            // Every output dead end with no source dead end within 30 m: a track or
            // road the design left hanging (the game marks it with a dead-end icon).
            var sourceEnds =
                otherCapture.Source
                    .Where(segment => segment.Kind == kind)
                    .SelectMany(
                        segment => new[] {
                            (Id: segment.StartNodeId, P: ((float3)segment.CourseStart).xz),
                            (Id: segment.EndNodeId, P: ((float3)segment.CourseEnd).xz)
                            })
                    .ToArray();

            var sourceDegree =
                sourceEnds.GroupBy(item => item.Id).ToDictionary(group => group.Key, group => group.Count());

            var sourceDead = sourceEnds.Where(item => sourceDegree[item.Id] == 1).Select(item => item.P).ToArray();
            var sourceCells =
                sourceDead
                    .GroupBy(point => ((int)Math.Floor(point.x / 32), (int)Math.Floor(point.y / 32)))
                    .ToDictionary(group => group.Key, group => group.ToArray());

            bool NearSourceEnd(float2 point)
            {
                var cx = (int)Math.Floor(point.x / 32);
                var cz = (int)Math.Floor(point.y / 32);

                for (var dz = -1; dz <= 1; dz++)
                    for (var dx = -1; dx <= 1; dx++)
                        if (sourceCells.TryGetValue((cx + dx, cz + dz), out var near)
                            && near.Any(otherPoint => math.distance(otherPoint, point) < 30))
                            return true;

                return false;
            }

            var hanging = ends.Where(item => degreeOf[item.Id] == 1 && !NearSourceEnd(item.P)).ToArray();
            double lat0 = (double)otherCapture.Input["request"]!["Latitude"]!,
                lon0 = (double)otherCapture.Input["request"]!["Longitude"]!;

            Console.WriteLine(
                $"  new {kind} dead ends (no source dead end within 30 m): {hanging.Length}"
                    + (hanging.Length == 0
                        ? ""
                        : "; e.g. "
                            + string.Join(
                                " ",
                                hanging
                                    .Take(
                                        int.TryParse(
                                            Environment.GetEnvironmentVariable("VISUAL_HANGING"),
                                            out var shown)
                                            ? shown
                                            : 6)
                .Select(
                    item => $"{item.Id}@{lat0 + math.degrees(item.P.y / Earth):F5},{lon0 + math.degrees(item.P.x / (Earth * Math.Cos(math.radians(lat0)))):F5}"))));

            // VISUAL_DEBUG_HANGING=1: what met at each new dead end in the source,
            // and which of those source ways the output still carries anywhere.
            if (Environment.GetEnvironmentVariable("VISUAL_DEBUG_HANGING") == "1")
                foreach (var item in hanging)
                {
                    var here =
                        sections
                            .Where(
                                segment => segment.Kind == kind
                                    && (segment.StartNodeId == item.Id || segment.EndNodeId == item.Id))
                            .ToArray();

                    var source =
                        otherCapture.Source
                            .Where(
                                segment => segment.Kind == kind
                                    && math.distance(((float3)segment.CourseStart).xz, item.P) < 40
                                    || segment.Kind == kind
                                        && math.distance(((float3)segment.CourseEnd).xz, item.P) < 40)
                            .ToArray();

                    var outWays =
                        sections
                            .Where(segment => segment.Kind == kind)
                            .SelectMany(segment => segment.SourceWayIds.Append(segment.WayId))
                            .ToHashSet();

                    Console.WriteLine(
                        $"    hanging {item.Id} at ({item.P.x:F0},{item.P.y:F0}) sourceDegree={(sourceDegree.TryGetValue(item.Id, out var sd) ? sd : 0)} output={string.Join(",", here.Select(segment => $"{segment.WayId}[{segment.Classification} len={NetworkDesignChecks.HorizontalArcLength(new(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd)):F0} other={(segment.StartNodeId == item.Id ? segment.EndNodeId : segment.StartNodeId)} otherDegree={(degreeOf.TryGetValue(segment.StartNodeId == item.Id ? segment.EndNodeId : segment.StartNodeId, out var od) ? od : 0)} service={segment.RailService}]"))}");

                    foreach (var otherItem in ends
                        .Where(secondItem => secondItem.Id != item.Id && math.distance(secondItem.P, item.P) < 60)
                        .GroupBy(secondItem => secondItem.Id)
                        .Select(group => group.First())
                        .OrderBy(secondItem => math.distance(secondItem.P, item.P))
                        .Take(4))
                        Console.WriteLine(
                            $"      near node {otherItem.Id} d={math.distance(otherItem.P, item.P):F1} degree={degreeOf[otherItem.Id]} dy={otherItem.Y - item.Y:F1}");

                    foreach (var segment in source)
                        Console.WriteLine(
                            $"      source way {segment.WayId} {segment.StartNodeId}->{segment.EndNodeId} {segment.Classification} kept={outWays.Contains(segment.WayId)} start=({segment.CourseStart.x:F0},{segment.CourseStart.z:F0}) end=({segment.CourseEnd.x:F0},{segment.CourseEnd.z:F0})");
                }
        }

        Console.WriteLine(
            $"  native omissions: {sameNode + nonfinite + noExtent + offsetOverBind.Length} (same start/end node {sameNode}, nonfinite {nonfinite}, no horizontal extent {noExtent}, curve offset over 32 m {offsetOverBind.Length})");

        Console.WriteLine($"  curve offsets over 12 m: {offsetOverShift}");
        Console.WriteLine($"  backward pieces: {backwardPieces.Length}");

        // One-way regular roads by lane count, against the lanes their source way has.
        {
            var sourceLanes =
                otherCapture.Source
                    .Where(segment => segment.Kind == NetworkKind.Road)
                    .GroupBy(segment => segment.WayId)
                    .ToDictionary(group => group.Key, group => group.First());

            var oneWays =
                sections
                    .Where(segment => segment.Kind == NetworkKind.Road
                        && segment.OneWay
                        && !NativeAssetKeys.IsHighway(segment))
                    .ToArray();

            Console.WriteLine(
                "  one-way regular roads by lanes (designed <- source): "
                    + string.Join(
                        "; ",
                        oneWays
                            .GroupBy(
                                segment => (
                                    Designed: segment.Lanes,
                                    Source: sourceLanes.TryGetValue(segment.WayId, out var origin)
                                        ? $"{(origin.OneWay ? "oneway" : "twoway")} {origin.Lanes}{(origin.ExplicitLanes ? "" : " default")}"
                                        : "no source way"))
                            .OrderByDescending(group => group.Count())
                            .Take(14)
                            .Select(group => $"{group.Key.Designed} <- {group.Key.Source}: {group.Count()}")));

            foreach (var segment in oneWays
                .Where(
                    segment => segment.Lanes >= 3
                        && (!sourceLanes.TryGetValue(segment.WayId, out var origin)
                            || origin.Lanes != segment.Lanes
                            || !origin.OneWay))
                .GroupBy(segment => segment.WayId)
                .Select(group => group.First())
                .Take(12))
            {
                var origin = sourceLanes.GetValueOrDefault(segment.WayId);
                Console.WriteLine(
                    $"    one-way {segment.Lanes} lanes: way {segment.WayId} {segment.Classification} ways=[{string.Join(",", segment.SourceWayIds.Take(4))}] source {(origin == null ? "none" : $"{(origin.OneWay ? "oneway" : "twoway")} lanes {origin.Lanes} explicit {origin.ExplicitLanes} fwd {origin.ForwardLanes} back {origin.BackwardLanes}")} avenue {segment.ConsolidatedAvenue} at ({segment.CourseStart.x:F0},{segment.CourseStart.z:F0})");
            }
        }

        // Short open stretches between two tunnel runs of one road: a chain of
        // ground sections through plain two-arm nodes with a tunnel at each end.
        // Each such gap draws two portal mouths facing each other.
        {
            var roads = sections.Where(segment => segment.Kind == NetworkKind.Road).ToArray();
            var arms =
                roads
                    .SelectMany(segment => new[] { (Node: segment.StartNodeId, Segment: segment), (Node: segment.EndNodeId, Segment: segment) })
                    .GroupBy(item => item.Node)
                    .ToDictionary(group => group.Key, group => group.Select(item => item.Segment).ToArray());

            var seen = new HashSet<PreparedSegment>(ReferenceEqualityComparer.Instance);
            var gaps = new List<(double Length, PreparedSegment First)>();

            foreach (var first in roads.Where(segment => !segment.Tunnel && !segment.Bridge))
            {
                if (!seen.Add(first))
                    continue;

                var length = NetworkDesignChecks.HorizontalArcLength(
                    new(first.CourseStart, first.CourseControl1, first.CourseControl2, first.CourseEnd));

                var tunnelEnds = 0;

                foreach (var startNode in new[] { first.StartNodeId, first.EndNodeId })
                {
                    var node = startNode;
                    var current = first;

                    while (arms[node].Length == 2)
                    {
                        var next = ReferenceEquals(arms[node][0], current) ? arms[node][1] : arms[node][0];

                        if (next.Tunnel)
                        {
                            tunnelEnds++;
                            break;
                        }

                        if (next.Bridge || !seen.Add(next))
                            break;

                        length += NetworkDesignChecks.HorizontalArcLength(
                            new(next.CourseStart, next.CourseControl1, next.CourseControl2, next.CourseEnd));

                        node = next.StartNodeId == node ? next.EndNodeId : next.StartNodeId;
                        current = next;
                    }
                }

                if (tunnelEnds == 2)
                    gaps.Add((length, first));
            }

            Console.WriteLine(
                $"  open gaps between two tunnels: {gaps.Count}; under 30 m {gaps.Count(gap => gap.Length < 30)}; under 60 m {gaps.Count(gap => gap.Length < 60)}; under 120 m {gaps.Count(gap => gap.Length < 120)}");

            foreach (var (length, first) in gaps.Where(gap => gap.Length < 60).OrderBy(gap => gap.Length).Take(12))
                Console.WriteLine(
                    $"    gap {length:F1} m: way {first.WayId} {first.Classification} at ({first.CourseStart.x:F0},{first.CourseStart.y:F1},{first.CourseStart.z:F0})");
        }

        foreach (var segment in backwardPieces.Take(60))
        {
            var chord = math.distance(segment.CourseStart.xz, segment.CourseEnd.xz);
            var turn =
                math.degrees(
                    math.acos(
                        math.clamp(
                            math.dot(
                                math.normalizesafe((segment.CourseControl1 - segment.CourseStart).xz),
                                math.normalizesafe((segment.CourseEnd - segment.CourseControl2).xz)),
                            -1,
                            1)));

            Console.WriteLine(
                $"    backward: way {segment.WayId} {segment.Classification} {segment.StartNodeId}->{segment.EndNodeId} chord {chord:F1} m turn {turn:F0} at ({segment.CourseStart.x:F0},{segment.CourseStart.z:F0})");
        }

        foreach (var segment in sections.OrderByDescending(LargestOffset).Take(5))
            Console.WriteLine(
                $"    largest offset: way {segment.WayId} {segment.Classification} start {math.length(segment.StartCurveOffset.xz):F1} m end {math.length(segment.EndCurveOffset.xz):F1} m");

        Connectivity("source", otherCapture.Source, otherCapture.Source);
        Connectivity("designed", otherCapture.Source, designed);
        Connectivity("emitted", otherCapture.Source, sections);

        if (Environment.GetEnvironmentVariable("VISUAL_DEBUG_OBLIGATION") is { } obligation)
        {
            var parts = obligation.Split(',').Select(long.Parse).ToArray();
            var way = parts[^1];

            foreach (var (label, list) in new[] {
                ("designed", designed),
                ("emitted", (IReadOnlyList<PreparedSegment>)sections)
                })
            {
                foreach (var segment in list.Where(
                    otherSegment => otherSegment.Kind == NetworkKind.Rail
                        && (otherSegment.WayId == way || otherSegment.SourceWayIds.Contains(way))))
                    Console.WriteLine(
                        $"  obligation {label} way {segment.WayId} {segment.StartNodeId}->{segment.EndNodeId} at ({segment.Start.x:F0},{segment.Start.z:F0})");

                foreach (var id in parts[..^1])
                    Console.WriteLine(
                        $"  obligation {label} node {id}: present={list.Any(segment => segment.Kind == NetworkKind.Rail && (segment.StartNodeId == id || segment.EndNodeId == id))} aliases=[{string.Join(",", list.Where(segment => segment.Kind == NetworkKind.Rail).SelectMany(segment => segment.SourceNodeAliases).Where(alias => alias.SourceNodeId == id).Select(alias => alias.CanonicalNodeId).Distinct())}]");
            }
        }

        var continuity = new ImportLog();
        NetworkContinuity.Validate(otherCapture.Source, sections, continuity, default, allowQualityWarnings: true);

        if (Environment.GetEnvironmentVariable("VISUAL_DEBUG_RAIL_OVERLAP") is "1" or "2")
            RailOverlaps(sections);

        Console.WriteLine(
            $"  continuity (as logged in game): unresolved sections={continuity.Metric("Network continuity unresolved sections")}; junction conflicts={continuity.Metric("Network junction position conflicts")}; rail overlaps={continuity.Metric("Rail overlap observations")}");

        if (Environment.GetEnvironmentVariable("VISUAL_DEBUG_CONTINUITY") == "1")
            UnresolvedSections(otherCapture.Source, sections);

        var obligations = new ImportLog { CaptureRecords = true };
        SourceTopologyObligations.Validate(
            SourceTopologyObligations.Capture(otherCapture.Source),
            sections,
            obligations,
            default);

        var topologySeconds = stage.Elapsed.TotalSeconds;
        stage.Restart();
        var roadbed =
            NetworkRoadbed.Prepare(
                otherCapture.Terrain,
                otherCapture.Scale,
                otherCapture.Width,
                sections,
                otherCapture.Areas,
                otherCapture.Lines,
                log,
                default);

        var unitSizes =
            sections.GroupBy(segment => segment.NativeUnitId).Select(group => group.Count()).OrderBy(n => n).ToArray();

        Console.WriteLine(
            $"  stages: design={designSeconds:F1}s; units={unitSeconds:F1}s; topology={topologySeconds:F1}s; roadbed={stage.Elapsed.TotalSeconds:F1}s; "
                +
            $"nativeUnits={unitSizes.Length}; unitP99={unitSizes[(int)(.99 * (unitSizes.Length - 1))]}; largestUnit={unitSizes[^1]}");

        if (Environment.GetEnvironmentVariable("VISUAL_DEBUG_TUNNELS") == "1")
        {
            TunnelCover(otherCapture, sections, roadbed);
        }

        if (Environment.GetEnvironmentVariable("VISUAL_DEBUG_DIPS") == "1")
        {
            BridgeGapDips(otherCapture, sections);
            // Bridge decks sunk in the stamped ground more than 40 m from their ends.
            var buried = 0;
            var total = 0;
            var byRef = new Dictionary<string, int>();

            foreach (var segment in sections.Where(
                otherSegment => otherSegment.Bridge && !otherSegment.Tunnel && otherSegment.Kind == NetworkKind.Road))
            {
                var curve =
                    new Cubic3d(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd);

                var length = NetworkDesignChecks.HorizontalArcLength(curve);
                var n = Math.Max(2, (int)(length / 5));

                for (var k = 0; k <= n; k++)
                {
                    if (Math.Min(k, n - k) * length / n < 40)
                        continue;

                    var point = NetworkDesignChecks.Evaluate(curve, k / (double)n);
                    total++;
                    var stamped =
                        NetworkProcessor.Sample(
                            (float)point.x,
                            (float)point.z,
                            roadbed,
                            otherCapture.Width);

                    if (stamped > point.y - 1)
                    {
                        buried++;
                        var key = segment.Reference.Length > 0 ? segment.Reference : segment.Name;
                        byRef[key] = byRef.TryGetValue(key, out var v) ? v + 1 : 1;

                        if (v == 0)
                        {
                            double lat0 = (double)otherCapture.Input["request"]!["Latitude"]!,
                                lon0 = (double)otherCapture.Input["request"]!["Longitude"]!;

                            Console.WriteLine(
                                $"    buried bridge {key} way {segment.WayId}: deck {point.y:F1}, original {NetworkProcessor.Sample((float)point.x, (float)point.z, otherCapture.Terrain, otherCapture.Width):F1}, stamped {stamped:F1} at {lat0 + math.degrees(point.z / Earth):F5},{lon0 + math.degrees(point.x / (Earth * Math.Cos(math.radians(lat0)))):F5}");
                        }
                    }
                }
            }

            Console.WriteLine(
                $"  bridge deck samples (>40 m from ends) within 1 m of or under the ground: {buried} of {total}; "
                    + string.Join(
                        ", ",
                        byRef
                            .OrderByDescending(entry => entry.Value)
                            .Take(8)
                            .Select(entry => $"{entry.Key} {entry.Value * 5} m")));
        }

        if (Environment.GetEnvironmentVariable("VISUAL_DEBUG_BRIDGES_NEAR") is { } bridgesNear)
        {
            // Each bridge chain within a radius: deck height every ~40 m and its
            // largest rise above (+) or dip below (−) the straight line between ends.
            double lat0 = (double)otherCapture.Input["request"]!["Latitude"]!,
                lon0 = (double)otherCapture.Input["request"]!["Longitude"]!;

            var q = bridgesNear.Split(',').Select(text => double.Parse(text, CultureInfo.InvariantCulture)).ToArray();
            var centre = ImportEarthLocation.Geo.GeoMath.ToLocalMeters(q[0], q[1], lat0, lon0);
            var bridges =
                sections.Where(segment => segment.Bridge && math.distance(segment.Start.xz, centre.xz) < q[2]).ToList();

            var byNode =
                bridges
                    .SelectMany((segment, i) => new[] { (segment.StartNodeId, i), (segment.EndNodeId, i) })
                    .GroupBy(item => item.Item1)
                    .ToDictionary(group => group.Key, group => group.Select(item => item.i).ToList());

            var done = new HashSet<int>();

            for (var i = 0; i < bridges.Count; i++)
            {
                if (done.Contains(i))
                    continue;

                // Walk the chain both ways through nodes shared by exactly two bridge sections.
                var chain = new LinkedList<(int Index, bool Forward)>();
                chain.AddFirst((i, true));
                done.Add(i);

                foreach (var forward in new[] { true, false })
                {
                    var node = forward ? bridges[i].EndNodeId : bridges[i].StartNodeId;

                    while (byNode[node].Count == 2)
                    {
                        var next = byNode[node].First(j => !done.Contains(j) && j != i);

                        if (done.Contains(next))
                            break;

                        done.Add(next);
                        var along = bridges[next].StartNodeId == node;

                        if (forward)
                            chain.AddLast((next, along));
                        else
                            chain.AddFirst((next, !along));

                        node = along ? bridges[next].EndNodeId : bridges[next].StartNodeId;

                        if (!byNode[node].Any(j => !done.Contains(j)))
                            break;
                    }
                }

                var points = new List<double3>();

                foreach (var (index, forward) in chain)
                {
                    var segment = bridges[index];
                    var curve =
                        new Cubic3d(
                            segment.CourseStart,
                            segment.CourseControl1,
                            segment.CourseControl2,
                            segment.CourseEnd);

                    for (var k = 0; k <= 8; k++)
                        points.Add(NetworkDesignChecks.Evaluate(curve, forward ? k / 8d : 1 - k / 8d));
                }

                // The deck over water only: elevated land approaches climbing to it
                // are not part of the span's shape.
                var wet =
                    points
                        .Select((point, k) => (point, k))
                        .Where(
                            item => otherCapture.Water(
                                new float3((float)item.point.x, 0, (float)item.point.z)).HasValue)
                        .Select(item => item.k)
                        .ToArray();

                if (wet.Length > 1)
                    points = points.GetRange(wet[0], wet[^1] - wet[0] + 1);

                var length = 0d;

                for (var k = 1; k < points.Count; k++)
                    length += math.distance(points[k - 1].xz, points[k].xz);

                if (length < 150)
                    continue;

                var walked = 0d;
                var up = 0d;
                var down = 0d;

                for (var k = 1; k < points.Count; k++)
                {
                    walked += math.distance(points[k - 1].xz, points[k].xz);
                    var chord = points[0].y + (points[^1].y - points[0].y) * walked / length;
                    up = Math.Max(up, points[k].y - chord);
                    down = Math.Min(down, points[k].y - chord);
                }

                var first = bridges[chain.First!.Value.Index];
                Console.WriteLine(
                    $"  bridge {first.Reference} {first.Name} way {first.WayId} oneway={first.OneWay}: {length:F0} m, ends {points[0].y:F1}/{points[^1].y:F1} m; rises {up:F1} m above and dips {-down:F1} m below the straight line");

                if (Environment.GetEnvironmentVariable("VISUAL_DEBUG_BRIDGE_HEIGHTS") == "1")
                    Console.WriteLine(
                        "    sections "
                            + chain.Count
                            + "; heights "
                            + string.Join(
                                " ",
                                points
                                    .Where((_, k) => k % 9 == 0)
                                    .Select(point => point.y.ToString("F1", CultureInfo.InvariantCulture)))
                            + "; water "
                            +
                        string.Join(
                            " ",
                            points
                                .Where((_, k) => k % 9 == 0)
                                .Select(
                                    point => otherCapture.Water(
                                        new float3((float)point.x, 0, (float)point.z)) is float w
                                        ? w.ToString("F1", CultureInfo.InvariantCulture)
                                        : "-")));
            }
        }

        if (Environment.GetEnvironmentVariable("VISUAL_DEBUG_WATER_CROSSINGS") == "1")
        {
            // Ground-level roads and rail crossing mapped water (a river reach or
            // polygon with a surface above the deck): each wet run is one crossing.
            double lat0 = (double)otherCapture.Input["request"]!["Latitude"]!,
                lon0 = (double)otherCapture.Input["request"]!["Longitude"]!;

            var crossings = new List<(PreparedSegment S, double Length, double3 P)>();

            foreach (var segment in sections.Where(otherSegment => !otherSegment.Bridge && !otherSegment.Tunnel))
            {
                var curve =
                    new Cubic3d(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd);

                var n = Math.Max(2, (int)(NetworkDesignChecks.HorizontalArcLength(curve) / 3));
                var run = 0;
                double3 at = default;

                for (var k = 0; k <= n; k++)
                {
                    var point = NetworkDesignChecks.Evaluate(curve, k / (double)n);
                    var surface = otherCapture.Water(new float3((float)point.x, 0, (float)point.z));

                    if (surface is float w && w > point.y - 1.5)
                    {
                        if (run == 0)
                            at = point;

                        run++;
                    }
                    else if (run > 0)
                    {
                        crossings.Add((segment, run * 3, at));
                        run = 0;
                    }
                }

                if (run > 0)
                    crossings.Add((segment, run * 3, at));
            }

            Console.WriteLine(
                $"  ground-level crossings of mapped water: {crossings.Count} (roads {crossings.Count(item => item.S.Kind == NetworkKind.Road)}, rail {crossings.Count(item => item.S.Kind == NetworkKind.Rail)}); longer than 6 m: {crossings.Count(item => item.Length > 6)}");

            foreach (var firstItem in crossings
                .Where(item => item.Length > 6)
                .OrderByDescending(item => item.Length)
                .Take(10))
                Console.WriteLine(
                    $"    {firstItem.S.Kind} {firstItem.S.Classification} {firstItem.S.Reference} way {firstItem.S.WayId}: {firstItem.Length:F0} m wet at {lat0 + math.degrees(firstItem.P.z / Earth):F5},{lon0 + math.degrees(firstItem.P.x / (Earth * Math.Cos(math.radians(lat0)))):F5}");
        }

        watch.Restart();
        var measured =
            VisualCompilerMetrics.Measure(sections, roadbed, otherCapture.Width, default);

        var metricSeconds = watch.Elapsed.TotalSeconds;

        if (Environment.GetEnvironmentVariable("VISUAL_DEBUG_TERRAIN") is "1" or "2")
            TerrainCauses(otherCapture, sections, roadbed, measured);

        if (Environment.GetEnvironmentVariable("VISUAL_DEBUG_OVERLAP") is "1" or "2" or "3")
            OverlapCauses(sections, measured);

        // VISUAL_DEBUG_SHORT=2 classifies the finished sections instead of the source.
        if (Environment.GetEnvironmentVariable("VISUAL_DEBUG_SHORT") == "2")
            ShortSources(sections);

        var raw = VisualCompilerMetrics.Measure(designed, roadbed, otherCapture.Width, default);
        Console.WriteLine(
            $"  before native units: sections={designed.Count}; paired failing={raw.Paired.FailedMeters / 1000:F1} km of {raw.Paired.MatchedMeters / 1000:F1}; grade-change={raw.Failures.Count(failure => failure.Category == "grade-change")}; short-segment={raw.Failures.Count(failure => failure.Category == "short-segment")}");

        Directory.CreateDirectory(output);

        using (var file =
            File.Create(
                Path.Combine(
                    output,
                    FailuresFile))) using (var zip =
                        new GZipStream(file, CompressionLevel.Optimal)) using (var writer = new StreamWriter(zip))
            foreach (var failure in measured.Failures)
                writer.WriteLine(JsonConvert.SerializeObject(new
                {
                    failure.Category,
                    failure.Section,
                    failure.WayId,
                    failure.StartNodeId,
                    failure.EndNodeId,
                    x = failure.Position.x,
                    y = failure.Position.y,
                    z = failure.Position.z,
                    failure.Value,
                    failure.Limit,
                    failure.Severity
                }));

        using (var file =
            File.Create(
                Path.Combine(
                    output,
                    PlotFile))) using (var zip =
                        new GZipStream(file, CompressionLevel.Optimal)) using (var writer = new StreamWriter(zip))
            foreach (var segment in sections)
                writer.WriteLine(JsonConvert.SerializeObject(new
                {
                    segment.WayId,
                    segment.StartNodeId,
                    segment.EndNodeId,
                    segment.Layer,
                    segment.SourceWayIds,
                    kind = segment.Kind.ToString(),
                    segment.Classification,
                    segment.Bridge,
                    segment.Tunnel,
                    segment.OneWay,
                    segment.NativeWidth,
                    segment.Roundabout,
                    segment.PairedCorridorId,
                    p = new[] { segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd }
                        .SelectMany(point => new[] { point.x, point.y, point.z })
                        .ToArray()
                }));

        var topology =
            obligations.Lines.Where(text => text.Contains("SOURCE OBLIGATIONS", StringComparison.Ordinal)).ToArray();

        var present =
            new HashSet<SourceNodeKey>(
                sections.SelectMany(
                    segment => new[] {
                        new SourceNodeKey(segment.Kind, segment.StartNodeId),
                        new SourceNodeKey(segment.Kind, segment.EndNodeId)
                        }));

        var aliased =
            new HashSet<SourceNodeKey>(
                sections.SelectMany(
                    segment => segment.SourceNodeAliases.Select(
                        alias => new SourceNodeKey(segment.Kind, alias.SourceNodeId))));

        var missingPorts =
            SourceTopologyObligations.Capture(otherCapture.Source).Ports
                .Where(nodeKey => !present.Contains(nodeKey) && !aliased.Contains(nodeKey))
                .ToArray();

        // What kind of source node each missing port was. A node of two sections where
        // only the tunnel or bridge flag changes is the end of a structure, which the
        // design re-cuts; the others are places where a road met other roads.
        string PortKind(SourceNodeKey nodeKey)
        {
            var touching =
                otherCapture.Source
                    .Where(
                        segment => segment.Kind == nodeKey.Kind
                            && (segment.StartNodeId == nodeKey.Id || segment.EndNodeId == nodeKey.Id))
                    .ToArray();

            if (touching.Length != 2)
                return touching.Length < 2 ? "dead end" : "junction";

            if (touching[0].Tunnel != touching[1].Tunnel)
                return "tunnel portal";

            if (touching[0].Bridge != touching[1].Bridge)
                return "bridge end";

            return "other node of two sections";
        }

        Console.WriteLine(
            $"  missing ports by kind ({missingPorts.Length}): "
                + string.Join(
                    "; ",
                    missingPorts
                        .GroupBy(PortKind)
                        .OrderByDescending(group => group.Count())
                        .Select(group => $"{group.Key} {group.Count()}")));

        var missing =
            missingPorts
                .Take(12)
                .Select(nodeKey =>
        {
            var touching =
                otherCapture.Source
                    .Where(
                        segment => segment.Kind == nodeKey.Kind
                            && (segment.StartNodeId == nodeKey.Id || segment.EndNodeId == nodeKey.Id))
                    .ToArray();

            return $"{nodeKey.Kind}:{nodeKey.Id} degree={touching.Length} classes={string.Join(",", touching.Select(segment => segment.Classification + (segment.Bridge ? "/bridge" : "") + (segment.Tunnel ? "/tunnel" : "") + (segment.Roundabout ? "/roundabout" : "") + (segment.OneWay ? "/oneway" : "") + "/L" + segment.Lanes))}";
        }).ToArray();

        var designedNodes =
            new HashSet<SourceNodeKey>(
                designed.SelectMany(
                    segment => new[] {
                        new SourceNodeKey(segment.Kind, segment.StartNodeId),
                        new SourceNodeKey(segment.Kind, segment.EndNodeId)
                        }));

        foreach (var text in missing)
            Console.WriteLine(
                "  missing port "
                    + text
                    + " inDesignOutput="
                    + designedNodes.Contains(
                        new(
                            Enum.Parse<NetworkKind>(text.Split(':')[0]),
                            long.Parse(text.Split(':')[1].Split(' ')[0]))));

        if (Environment.GetEnvironmentVariable("VISUAL_DEBUG_CROSSINGS") == "1")
            Crossings(otherCapture.Source, "source", false);

        if (long.TryParse(Environment.GetEnvironmentVariable("VISUAL_DEBUG_WAY"), out var debugWay))
            foreach (var (label, list) in new[] {
                ("source", (IReadOnlyList<PreparedSegment>)otherCapture.Source),
                ("designed", designed)
                })
                foreach (var segment in list.Where(
                    otherSegment => otherSegment.WayId == debugWay || otherSegment.SourceWayIds.Contains(debugWay)))
                    Console.WriteLine(
                        $"  {label} way={segment.WayId} ways=[{string.Join(",", segment.SourceWayIds)}] {segment.StartNodeId}->{segment.EndNodeId} oneway={segment.OneWay} class={segment.Classification} start=({segment.Start.x:F1},{segment.Start.z:F1}) end=({segment.End.x:F1},{segment.End.z:F1}) aliases=[{string.Join(",", segment.SourceNodeAliases.Select(alias => alias.SourceNodeId + ">" + alias.CanonicalNodeId))}] "
                            +
                        $"course=[{string.Join(" ", new[] { segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd }.Select(point => $"({point.x:F0},{point.z:F0})"))}] geometry=[{string.Join(" ", segment.SourceGeometry.Take(16).Select(point => $"({point.x:F0},{point.z:F0})"))}] "
                            +
                        $"original=[{string.Join(" ", segment.OriginalGeometry.Take(24).Select(point => $"({point.x:F0},{point.z:F0})"))}] bridge={segment.Bridge} layer={segment.Layer} tags={string.Join(",", segment.RawSourceTags.Where(entry => entry.Key is "bridge" or "layer" or "tunnel" or "railway" or "service" or "tracks").Select(entry => entry.Key + "=" + entry.Value))}");

        foreach (var group in sections
            .SelectMany(
                segment => new[] {
                    (segment.Kind, Id: segment.StartNodeId, P: segment.Start, segment.WayId),
                    (segment.Kind, Id: segment.EndNodeId, P: segment.End, segment.WayId)
                    })
            .GroupBy(item => (item.Kind, item.Id))
            .Where(otherGroup => otherGroup.Any(item => math.distance(item.P, otherGroup.First().P) > .02f)).Take(5))
            Console.WriteLine(
                $"  conflicting node {group.Key.Kind}:{group.Key.Id} "
                    + string.Join(
                        " ",
                        group.Select(item => $"way {item.WayId}@({item.P.x:F2},{item.P.y:F2},{item.P.z:F2})")));

        if (Environment.GetEnvironmentVariable("VISUAL_DEBUG_NEAR") is string near
            && near.Split(',') is [var nx, var nz])
        {
            var centre = new float2(float.Parse(nx), float.Parse(nz));

            foreach (var (label, list) in new[] {
                ("source", (IReadOnlyList<PreparedSegment>)otherCapture.Source),
                ("designed", designed)
                })
                foreach (var segment in list.Where(
                    otherSegment => Enumerable
                        .Range(0, 9)
                        .Any(
                            k => math.distance(
                                NetworkDesignChecks.Evaluate(
                                    new Cubic3d(
                                        otherSegment.CourseStart,
                                        otherSegment.CourseControl1,
                                        otherSegment.CourseControl2,
                                        otherSegment.CourseEnd),
                                    k / 8d).xz,
                                (double2)centre)
                                < 40)))
                    Console.WriteLine(
                        $"  near {label} {segment.Kind} way={segment.WayId} {segment.StartNodeId}->{segment.EndNodeId} class={segment.Classification} service={segment.RailService} bridge={segment.Bridge} tunnel={segment.Tunnel} layer={segment.Layer} start=({segment.CourseStart.x:F1},{segment.CourseStart.y:F1},{segment.CourseStart.z:F1}) end=({segment.CourseEnd.x:F1},{segment.CourseEnd.y:F1},{segment.CourseEnd.z:F1})");
        }

        // VISUAL_DEBUG_REF=A 55: every source section of that route reference,
        // with geographic ends, to find a reported spot in the capture.
        if (Environment.GetEnvironmentVariable("VISUAL_DEBUG_REF") is { } debugRef)
        {
            double lat0 = (double)otherCapture.Input["request"]!["Latitude"]!,
                lon0 = (double)otherCapture.Input["request"]!["Longitude"]!;

            string DescribeLocation(float3 point)
                => $"{lat0 + math.degrees(point.z / Earth):F5},{lon0 + math.degrees(point.x / (Earth * Math.Cos(math.radians(lat0)))):F5}";

            // tag:key=value matches a source tag instead.
            bool Match(PreparedSegment segment)
                => debugRef.StartsWith("tag:", StringComparison.Ordinal)
                    && debugRef.Substring(4).Split('=') is [var key, var value]
                ? segment.SourceTags.TryGetValue(key, out var text) && text == value
                    : segment.Reference.Split(';').Any(otherText => otherText.Trim() == debugRef);

            foreach (var segment in otherCapture.Source.Where(Match))
                Console.WriteLine(
                    $"  ref way={segment.WayId} {segment.StartNodeId}->{segment.EndNodeId} class={segment.Classification} link={segment.ProtectedLink} oneway={segment.OneWay} lanes={segment.Lanes} width={segment.NativeWidth} bridge={segment.Bridge} tunnel={segment.Tunnel} from={DescribeLocation(segment.Start)} to={DescribeLocation(segment.End)} local=({segment.Start.x:F0},{segment.Start.z:F0})->({segment.End.x:F0},{segment.End.z:F0}) y=({segment.CourseStart.y:F1},{segment.CourseEnd.y:F1})");
        }

        // VISUAL_DEBUG_SPLITS=1: source nodes where a one-way road splits in two
        // or two join, with lanes, link flags and the source angle between them.
        if (Environment.GetEnvironmentVariable("VISUAL_DEBUG_SPLITS") == "1")
        {
            double lat0 = (double)otherCapture.Input["request"]!["Latitude"]!,
                lon0 = (double)otherCapture.Input["request"]!["Longitude"]!;

            var ends =
                otherCapture.Source
                    .Where(segment => segment.Kind == NetworkKind.Road && segment.OneWay && !segment.Roundabout)
                    .SelectMany(
                        segment => new[] {
                            (Node: segment.StartNodeId, S: segment, Out: true),
                            (Node: segment.EndNodeId, S: segment, Out: false)
                            })
                    .GroupBy(item => item.Node);

            double2 Away(PreparedSegment segment, bool atStart)
            {
                var points =
                    segment.SourceGeometry.Count > 1 ? segment.SourceGeometry : new[] { segment.Start, segment.End };

                var otherPoint = atStart ? segment.Start : segment.End;

                foreach (var point in atStart ? points : Enumerable.Reverse(points))
                    if (math.distance(point.xz, otherPoint.xz) >= 30)
                        return math.normalizesafe((double2)(point.xz - otherPoint.xz));

                return math.normalizesafe((double2)((atStart ? segment.End : segment.Start).xz - otherPoint.xz));
            }

            foreach (var group in ends)
            {
                var list = group.ToArray();

                if (list.Length != 3)
                    continue;

                var outs = list.Where(item => item.Out).ToArray();
                var ins = list.Where(item => !item.Out).ToArray();
                var (one, two) =
                    outs.Length == 2
                        ? (ins, outs)
                        : ins.Length == 2
                            ? (outs, ins)
                            : (
                                Array.Empty<(long, PreparedSegment, bool)>(),
                                Array.Empty<(long, PreparedSegment, bool)>());

                if (one.Length != 1
                    || two.Length != 2
                    || !two.Any(item => item.Item2.Classification is "motorway" or "trunk"))
                    continue;

                var angle =
                    math.degrees(
                        Math.Acos(
                            math.clamp(
                                math.dot(Away(two[0].Item2, two[0].Item3), Away(two[1].Item2, two[1].Item3)),
                                -1,
                                1)));

                var point = one[0].Item3 ? one[0].Item2.Start : one[0].Item2.End;
                Console.WriteLine(
                    $"  split {(outs.Length == 2 ? "diverge" : "merge")} node={group.Key} at {lat0 + math.degrees(point.z / Earth):F5},{lon0 + math.degrees(point.x / (Earth * Math.Cos(math.radians(lat0)))):F5} local=({point.x:F0},{point.z:F0}) "
                        +
                    $"trunk={one[0].Item2.Classification}/{one[0].Item2.Lanes}L/{one[0].Item2.Reference} branches={string.Join(" + ", two.Select(item => $"{item.Item2.Classification}{(item.Item2.ProtectedLink ? "(link)" : "")}/{item.Item2.Lanes}L/{item.Item2.Reference}"))} angle={angle:F1}");
            }
        }

        // VISUAL_DEBUG_PROFILE=x,z,r: emitted road heights against the original
        // ground at quarter points, for every section within r of a local point.
        if (Environment.GetEnvironmentVariable("VISUAL_DEBUG_PROFILE") is { } profileAt
            && profileAt
                .Split(',')
                .Select(text => double.Parse(text, CultureInfo.InvariantCulture))
                .ToArray() is [var px, var pz, var pr])
            foreach (var segment in sections
                .Where(
                    otherSegment => (otherSegment.Kind == NetworkKind.Road
                        || Environment.GetEnvironmentVariable("VISUAL_DEBUG_PROFILE_RAIL") == "1")
                        && math.distance(((double3)otherSegment.CourseStart).xz, new double2(px, pz)) < pr)
                .OrderBy(otherSegment => otherSegment.WayId)
                .ThenBy(otherSegment => otherSegment.CourseStart.x))
            {
                var curve =
                    new Cubic3d(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd);

                Console.WriteLine(
                    $"  profile way={segment.WayId} {segment.StartNodeId}->{segment.EndNodeId} {segment.Classification} w={segment.NativeWidth} oneway={segment.OneWay} bridge={segment.Bridge} tunnel={segment.Tunnel} ({segment.CourseStart.x:F1},{segment.CourseStart.z:F1})->({segment.CourseEnd.x:F1},{segment.CourseEnd.z:F1}) offsets=({segment.StartCurveOffset.x:F2},{segment.StartCurveOffset.z:F2})/({segment.EndCurveOffset.x:F2},{segment.EndCurveOffset.z:F2}) "
                        +
                    string.Join(" ", Enumerable.Range(0, 5).Select(k =>
{
    var point = NetworkDesignChecks.Evaluate(curve, k / 4d);
    return $"{point.y:F1}/{NetworkProcessor.Sample((float)point.x, (float)point.z, otherCapture.Terrain, otherCapture.Width):F1}";
})));
            }

        if (long.TryParse(Environment.GetEnvironmentVariable("VISUAL_DEBUG_NODE"), out var debugNode))
            foreach (var segment in otherCapture.Source.Where(
                otherSegment => otherSegment.StartNodeId == debugNode || otherSegment.EndNodeId == debugNode))
                Console.WriteLine(
                    $"  node-source way={segment.WayId} {segment.StartNodeId}->{segment.EndNodeId} class={segment.Classification} ref={segment.Reference} lanes={segment.Lanes} link={segment.ProtectedLink} oneway={segment.OneWay} layer={segment.Layer} bridge={segment.Bridge} tunnel={segment.Tunnel} kind={segment.Kind} y=({segment.CourseStart.y:F2},{segment.CourseEnd.y:F2}) roundabout={segment.Roundabout} start=({segment.Start.x:F1},{segment.Start.z:F1}) end=({segment.End.x:F1},{segment.End.z:F1}) course=({segment.CourseStart.x:F1},{segment.CourseStart.z:F1})->({segment.CourseEnd.x:F1},{segment.CourseEnd.z:F1}) len={NetworkDesignChecks.HorizontalArcLength(new(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd)):F1}");

        if (obligations.Records.TryGetValue("source-topology-obligations.ndjson", out var kindRecords))
            foreach (var group in kindRecords
                .Select(JObject.FromObject)
                .Where(row => !(bool)row["reachable"]!)
                .GroupBy(row => (int)row["From"]!["Kind"]!))
                Console.WriteLine($"  unreachable connections {(NetworkKind)group.Key}: {group.Count()}");

        if (obligations.Records.TryGetValue("source-topology-obligations.ndjson", out var connectionRecords))
            foreach (var row in connectionRecords
                .Select(JObject.FromObject)
                .Where(secondRecord => !(bool)secondRecord["reachable"]!)
                .Take(
                    int.TryParse(Environment.GetEnvironmentVariable("VISUAL_UNREACHABLE"), out var shownUnreachable)
                        ? shownUnreachable
                        : 8))
                Console.WriteLine(
                    "  unreachable "
                        + row
                            .ToString(Formatting.None)
                            .Substring(0, Math.Min(300, row.ToString(Formatting.None).Length)));

        var aliasTargets =
            sections
                .SelectMany(segment => segment.SourceNodeAliases.Select(alias => (segment.Kind, alias)))
                .GroupBy(item => new SourceNodeKey(item.Kind, item.alias.SourceNodeId))
            .Where(
                group => group.Select(item => item.alias.CanonicalNodeId).Distinct().Count() != 1
                    || !present.Contains(new(group.Key.Kind, group.First().alias.CanonicalNodeId)))
                .Take(
                    int.TryParse(Environment.GetEnvironmentVariable("VISUAL_BAD_ALIASES"), out var shownAliases)
                        ? shownAliases
                        : 6)
                .ToArray();

        foreach (var group in aliasTargets)
            Console.WriteLine(
                $"  bad alias {group.Key.Kind}:{group.Key.Id} targets={string.Join(",", group.Select(item => item.alias.CanonicalNodeId).Distinct())} targetPresent={group.Select(item => item.alias.CanonicalNodeId).Distinct().Select(t => present.Contains(new(group.Key.Kind, t))).FirstOrDefault()} inSource={otherCapture.Source.Any(segment => segment.Kind == group.Key.Kind && segment.SourceNodeAliases.Any(alias => alias.SourceNodeId == group.Key.Id && alias.CanonicalNodeId == group.First().alias.CanonicalNodeId))} targetIsSourceNode={otherCapture.Source.Any(segment => segment.Kind == group.Key.Kind && (segment.StartNodeId == group.First().alias.CanonicalNodeId || segment.EndNodeId == group.First().alias.CanonicalNodeId))}");

        File.WriteAllText(Path.Combine(output, RunFile), JsonConvert.SerializeObject(new
        {
            capture = Path.GetFileName(otherCapture.Folder),
            pipeline,
            latitude = (double)otherCapture.Input["request"]!["Latitude"]!,
            longitude = (double)otherCapture.Input["request"]!["Longitude"]!,
            sourceSections = otherCapture.Source.Length,
            designedSections = designed.Count,
            measured.Sections,
            measured.Kilometres,
            paired = measured.Paired,
            designSeconds,
            metricSeconds,
            topology,
            compilerLog = log.Lines.Where(text => text.Contains("VISUAL COMPILER", StringComparison.Ordinal)).ToArray()
        }, Formatting.Indented));

        Console.WriteLine(
            $"{pipeline}: designed {designed.Count} sections in {designSeconds:F1} s; emitted {sections.Count}");

        foreach (var line in topology)
            Console.WriteLine("  " + line);

        // Where do tangent breaks sit: generated nodes, source ports, or stroke joins?
        var breaks = measured.Failures.Where(failure => failure.Category == "tangent-break").ToArray();

        string NodeClass(VisualCompilerMetrics.Failure failure)
        {
            var segment = sections[failure.Section];
            var id =
                math.distance((double3)segment.CourseStart, failure.Position)
                    < math.distance((double3)segment.CourseEnd, failure.Position)
                    ? segment.StartNodeId
                    : segment.EndNodeId;

            return id <= long.MinValue + 9000000 ? "generated-compiler" : id < 0 ? "generated-earlier" : "source-node";
        }

        foreach (var group in breaks.GroupBy(NodeClass))
            Console.WriteLine(
                $"  tangent-break at {group.Key}: {group.Count()} (median {group.Select(failure => failure.Value).OrderBy(v => v).ElementAt(group.Count() / 2):F1}°)");

        // VISUAL_DEBUG_GRADES=1: road grade-change failures by where they sit: the
        // worst sample at a section end (mostly a cubic's own curvature, which is
        // largest at an end; a kink between sections is rare) or inside it, near a structure end, a
        // junction or neither, and by class.
        if (Environment.GetEnvironmentVariable("VISUAL_DEBUG_GRADES") == "1")
        {
            var roadDegree =
                sections
                    .Where(segment => segment.Kind == NetworkKind.Road)
                    .SelectMany(segment => new[] { segment.StartNodeId, segment.EndNodeId })
                    .GroupBy(id => id)
                    .ToDictionary(group => group.Key, group => group.Count());

            var structureEnds =
                sections
                    .Where(segment => segment.Kind == NetworkKind.Road && NetworkStructure.IsStructure(segment))
                    .SelectMany(segment => new[] { ((double3)segment.CourseStart).xz, ((double3)segment.CourseEnd).xz })
                    .ToArray();

            var junctions =
                sections
                    .Where(segment => segment.Kind == NetworkKind.Road)
                    .SelectMany(
                        segment => new[] {
                            (segment.StartNodeId, ((double3)segment.CourseStart).xz),
                            (segment.EndNodeId, ((double3)segment.CourseEnd).xz)
                            })
                    .Where(item => roadDegree[item.Item1] >= 3)
                    .Select(item => item.Item2)
                    .ToArray();

            string Where(VisualCompilerMetrics.Failure failure)
            {
                var segment = sections[failure.Section];
                var atNode =
                    math.distance(failure.Position, (double3)segment.CourseStart) < .05
                        || math.distance(failure.Position, (double3)segment.CourseEnd) < .05;

                var near =
                    structureEnds.Any(point => math.distance(point, failure.Position.xz) < 60)
                        ? "structure-end"
                        : junctions.Any(point => math.distance(point, failure.Position.xz) < 60) ? "junction" : "open";

                return $"{(atNode ? "end" : "inside")} {near} {(segment.ProtectedLink ? "link" : segment.Classification)}{(segment.Tunnel ? " tunnel" : segment.Bridge ? " bridge" : "")}";
            }

            foreach (var group in measured.Failures
                .Where(
                    failure => failure.Category == "grade-change" && sections[failure.Section].Kind == NetworkKind.Road)
                .GroupBy(Where)
                .OrderByDescending(otherGroup => otherGroup.Count())
                .Take(16))
                Console.WriteLine(
                    $"  grade-change {group.Key}: {group.Count()} (median {group.Select(failure => failure.Value / failure.Limit).OrderBy(v => v).ElementAt(group.Count() / 2):F1}x)");
        }

        // Rail on its own: failures by category, then (VISUAL_DEBUG_RAIL=n) the
        // n worst of each with location and the section's attributes.
        {
            double lat0 = (double)otherCapture.Input["request"]!["Latitude"]!,
                lon0 = (double)otherCapture.Input["request"]!["Longitude"]!;

            var railKm =
                sections
                    .Where(segment => segment.Kind == NetworkKind.Rail)
                    .Sum(
                        segment => NetworkDesignChecks.HorizontalArcLength(
                            new Cubic3d(
                                segment.CourseStart,
                                segment.CourseControl1,
                                segment.CourseControl2,
                                segment.CourseEnd)))
                    / 1000;

            var rail = measured.Failures.Where(failure => sections[failure.Section].Kind == NetworkKind.Rail).ToArray();
            Console.WriteLine(
                $"  RAIL sections={sections.Count(segment => segment.Kind == NetworkKind.Rail)}; km={railKm:F1}; failures={rail.Length}");

            int.TryParse(Environment.GetEnvironmentVariable("VISUAL_DEBUG_RAIL"), out var shownRail);

            if (shownRail > 0)
            {
                foreach (var group in rail
                    .GroupBy(
                        failure => $"{failure.Category} {(sections[failure.Section].RailService is "" ? "main" : sections[failure.Section].RailService)}")
                    .OrderByDescending(otherGroup => otherGroup.Count())
                    .Take(20))
                    Console.WriteLine($"  rail by service {group.Key}: {group.Count()}");

                // What the failing section's two ends are: a switch (three or more
                // arms), a structure or layer change, a source port, a plain alias
                // target, or a plain joint, and whether the end node came from the
                // source, the compiler or an earlier stage.
                var railDegree =
                    sections
                        .Where(segment => segment.Kind == NetworkKind.Rail)
                        .SelectMany(segment => new[] { segment.StartNodeId, segment.EndNodeId })
                        .GroupBy(id => id)
                        .ToDictionary(group => group.Key, group => group.Count());

                var railPorts =
                    SourceTopologyObligations.Capture(otherCapture.Source).Ports
                        .Where(nodeKey => nodeKey.Kind == NetworkKind.Rail)
                        .Select(nodeKey => nodeKey.Id)
                        .ToHashSet();

                var railAliasTargets =
                    otherCapture.Source
                        .Where(segment => segment.Kind == NetworkKind.Rail)
                        .SelectMany(segment => segment.SourceNodeAliases.Select(alias => alias.CanonicalNodeId))
                        .ToHashSet();

                string RailEnd(PreparedSegment segment, long id)
                {
                    var d = railDegree[id];

                    if (d >= 3)
                        return "switch";

                    if (d == 1)
                        return "end";

                    var otherSegment =
                        sections.First(
                            secondSegment => secondSegment != segment
                                && secondSegment.Kind == NetworkKind.Rail
                                && (secondSegment.StartNodeId == id || secondSegment.EndNodeId == id));

                    if (otherSegment.Bridge != segment.Bridge || otherSegment.Tunnel != segment.Tunnel)
                        return "structure";

                    if (otherSegment.Layer != segment.Layer)
                        return "layer";

                    return railPorts.Contains(id)
                        ? "port"
                        : railAliasTargets.Contains(id)
                            ? "alias"
                            : id >= 0 ? "source-plain" : id <= long.MinValue + 9000000 ? "compiler" : "other";
                }

                foreach (var group in rail.GroupBy(failure =>
{
    var segment = sections[failure.Section];
    return failure.Category
        + " "
        + string.Join(
            "+",
            new[] { RailEnd(segment, segment.StartNodeId), RailEnd(segment, segment.EndNodeId) }.OrderBy(text => text));
})
                    .OrderByDescending(otherGroup => otherGroup.Count()).Take(30))
                    Console.WriteLine($"  rail ends {group.Key}: {group.Count()}");

                string Bucket(VisualCompilerMetrics.Failure failure) => failure.Category switch
                {
                    "tight-radius" => failure.Value < 25
                        ? "<25 m"
                        : failure.Value < 75 ? "25-75 m" : failure.Value < 150 ? "75-150 m" : "150-300 m",
                    "tangent-break" => failure.Value < 5
                        ? "2-5°"
                        : failure.Value < 15 ? "5-15°" : failure.Value < 45 ? "15-45°" : "45°+",
                    "short-segment" => failure.Value < 8
                        ? "<8 m"
                        : failure.Value < 20 ? "8-20 m" : failure.Value < 40 ? "20-40 m" : "40-60 m",
                    "grade-change" or "steep-grade" => failure.Value / failure.Limit < 2
                        ? "<2x"
                        : failure.Value / failure.Limit < 5 ? "2-5x" : "5x+",
                    _ => failure.Value < 1 ? "<1 m" : failure.Value < 3 ? "1-3 m" : "3 m+"
                };

                foreach (var group in rail.GroupBy(failure => failure.Category).OrderBy(otherGroup => otherGroup.Key))
                    Console.WriteLine(
                        $"  rail size {group.Key}: "
                            + string.Join(
                                ", ",
                                group
                                    .GroupBy(Bucket)
                                    .OrderBy(otherGroup => otherGroup.Key)
                                    .Select(otherGroup => $"{otherGroup.Key} {otherGroup.Count()}")));

                // The severe geometry: kinks over 45° and radii under 25 m, with
                // what meets at the node and the source way's own shape there.
                foreach (var failure in rail
                    .Where(
                        otherFailure => otherFailure.Category == "tangent-break" && otherFailure.Value >= 45
                            || otherFailure.Category == "tight-radius" && otherFailure.Value < 25)
                    .OrderByDescending(otherFailure => otherFailure.Severity)
                    .Take(shownRail * 3))
                {
                    var segment = sections[failure.Section];
                    var node =
                        math.distance((double3)segment.CourseStart, failure.Position)
                            < math.distance((double3)segment.CourseEnd, failure.Position)
                            ? segment.StartNodeId
                            : segment.EndNodeId;

                    var arms =
                        sections
                            .Where(
                                otherSegment => otherSegment.Kind == NetworkKind.Rail
                                    && (otherSegment.StartNodeId == node || otherSegment.EndNodeId == node))
                            .ToArray();

                    var source =
                        otherCapture.Source
                            .Where(
                                otherSegment => otherSegment.Kind == NetworkKind.Rail
                                    && (otherSegment.WayId == segment.WayId
                                        || segment.SourceWayIds.Contains(otherSegment.WayId)))
                            .ToArray();

                    Console.WriteLine(
                        $"    severe {failure.Category} {failure.Value:F1} way {failure.WayId} at {lat0 + math.degrees(failure.Position.z / Earth):F5},{lon0 + math.degrees(failure.Position.x / (Earth * Math.Cos(math.radians(lat0)))):F5} node {node} ({RailEnd(segment, node)}) arms={string.Join(";", arms.Select(otherSegment => $"{otherSegment.WayId}/{otherSegment.RailService}/{(otherSegment.Bridge ? "b" : otherSegment.Tunnel ? "t" : "g")}/len{NetworkDesignChecks.HorizontalArcLength(new Cubic3d(otherSegment.CourseStart, otherSegment.CourseControl1, otherSegment.CourseControl2, otherSegment.CourseEnd)):F0}"))} "
                            +
                        $"sourceWays={string.Join(",", segment.SourceWayIds.Take(6))} sourceSections={source.Length} sourceServices={string.Join(",", source.Select(otherSegment => otherSegment.RailService).Distinct())}");
                }

                var sourceRail = otherCapture.Source.Where(segment => segment.Kind == NetworkKind.Rail).ToArray();
                var sourceMeasured =
                    VisualCompilerMetrics.Measure(
                        sourceRail,
                        otherCapture.Terrain,
                        otherCapture.Width,
                        default);

                foreach (var group in sourceMeasured.Failures
                    .Where(failure => failure.Category is "tight-radius" or "tangent-break" or "short-segment")
                    .GroupBy(failure => failure.Category)
                    .OrderBy(otherGroup => otherGroup.Key))
                    Console.WriteLine(
                        $"  source rail size {group.Key}: "
                            + string.Join(
                                ", ",
                                group
                                    .GroupBy(Bucket)
                                    .OrderBy(otherGroup => otherGroup.Key)
                                    .Select(otherGroup => $"{otherGroup.Key} {otherGroup.Count()}")));

                // Hotspots: 1 km cells holding the most rail failures.
                foreach (var group in rail
                    .GroupBy(
                        failure => (
                            (int)Math.Floor(failure.Position.x / 1000),
                            (int)Math.Floor(failure.Position.z / 1000)))
                    .OrderByDescending(otherGroup => otherGroup.Count())
                    .Take(8))
                    Console.WriteLine(
                        $"  rail hotspot ({group.Key.Item1 * 1000 + 500},{group.Key.Item2 * 1000 + 500}) at {lat0 + math.degrees((group.Key.Item2 * 1000 + 500) / Earth):F4},{lon0 + math.degrees((group.Key.Item1 * 1000 + 500) / (Earth * Math.Cos(math.radians(lat0)))):F4}: {group.Count()} "
                            +
                        string.Join(
                            " ",
                            group
                                .GroupBy(failure => failure.Category)
                                .Select(otherGroup => $"{otherGroup.Key}={otherGroup.Count()}")));
            }

            foreach (var group in rail
                .GroupBy(failure => failure.Category)
                .OrderByDescending(otherGroup => otherGroup.Count()))
            {
                var values = group.Select(failure => failure.Value).OrderBy(v => v).ToArray();
                Console.WriteLine(
                    $"  rail {group.Key}: {group.Count()} (median {values[values.Length / 2]:F3}, limit {group.First().Limit:F3})");

                foreach (var failure in group.OrderByDescending(otherFailure => otherFailure.Severity).Take(shownRail))
                {
                    var segment = sections[failure.Section];
                    Console.WriteLine(
                        $"    {failure.Category} way {failure.WayId} {failure.Value:F3}/{failure.Limit:F3} at {lat0 + math.degrees(failure.Position.z / Earth):F5},{lon0 + math.degrees(failure.Position.x / (Earth * Math.Cos(math.radians(lat0)))):F5} local=({failure.Position.x:F0},{failure.Position.z:F0}) "
                            +
                        $"{segment.StartNodeId}->{segment.EndNodeId} service={segment.RailService} bridge={segment.Bridge} tunnel={segment.Tunnel} layer={segment.Layer} len={NetworkDesignChecks.HorizontalArcLength(new Cubic3d(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd)):F1} y=({segment.CourseStart.y:F1},{segment.CourseEnd.y:F1})");
                }
            }
        }

        // VISUAL_DEBUG_SHORT_ENDS=1: short sections by the degree of their end
        // nodes and whether a structure (bridge/tunnel) starts or ends there.
        if (Environment.GetEnvironmentVariable("VISUAL_DEBUG_SHORT_ENDS") == "1")
        {
            var degree =
                sections
                    .SelectMany(
                        segment => new[] { (segment.Kind, segment.StartNodeId), (segment.Kind, segment.EndNodeId) })
                    .GroupBy(item => item)
                    .ToDictionary(group => group.Key, group => group.Count());

            string End(PreparedSegment segment, long node)
            {
                var d = degree[(segment.Kind, node)];

                if (d != 2)
                    return d >= 3 ? "junction" : "dead-end";

                var other =
                    sections.First(
                        otherSegment => otherSegment != segment
                            && otherSegment.Kind == segment.Kind
                            && (otherSegment.StartNodeId == node || otherSegment.EndNodeId == node));

                return other.Bridge != segment.Bridge || other.Tunnel != segment.Tunnel
                    ? "structure"
                    : other.WayId != segment.WayId ? "way-change" : "plain";
            }

            foreach (var group in measured.Failures
                .Where(failure => failure.Category == "short-segment")
                .GroupBy(failure =>
{
    var segment = sections[failure.Section];
    return string.Join(
        "+",
        new[] { End(segment, segment.StartNodeId), End(segment, segment.EndNodeId) }.OrderBy(text => text))
        + " "
        + segment.Kind;
}).OrderByDescending(otherGroup => otherGroup.Count()))
                Console.WriteLine(
                    $"  short-segment ends {group.Key}: {group.Count()} (median {group.Select(failure => failure.Value).OrderBy(v => v).ElementAt(group.Count() / 2):F1} m)");

            // What differs across the ends of short pieces between two other ways.
            string Differs(PreparedSegment segment, long node)
            {
                if (degree[(segment.Kind, node)] > 2)
                    return "junction";

                var otherSegment =
                    sections.FirstOrDefault(
                        secondSegment => secondSegment != segment
                            && secondSegment.Kind == segment.Kind
                            && (secondSegment.StartNodeId == node || secondSegment.EndNodeId == node));

                if (otherSegment == null)
                    return "end";

                var texts = new List<string>();

                if (otherSegment.NativeAssetKey != segment.NativeAssetKey)
                    texts.Add("asset");

                if (otherSegment.Lanes != segment.Lanes)
                    texts.Add("lanes");

                if (otherSegment.Classification != segment.Classification)
                    texts.Add("class");

                if (otherSegment.OneWay != segment.OneWay)
                    texts.Add("oneway");

                if (otherSegment.Bridge != segment.Bridge || otherSegment.Tunnel != segment.Tunnel)
                    texts.Add("structure");

                if (otherSegment.Layer != segment.Layer)
                    texts.Add("layer");

                if (math.dot(
                    math.normalizesafe(
                        (segment.StartNodeId == node
                            ? segment.CourseControl1 - segment.CourseStart
                            : segment.CourseControl2 - segment.CourseEnd).xz),
                    math.normalizesafe(
                        (otherSegment.StartNodeId == node
                            ? otherSegment.CourseControl1 - otherSegment.CourseStart
                            : otherSegment.CourseControl2 - otherSegment.CourseEnd).xz))
                        > -.97)
                    texts.Add("bend");

                return texts.Count == 0 ? "same" : string.Join("/", texts);
            }

            foreach (var group in measured.Failures
                .Where(
                    failure => failure.Category == "short-segment"
                        && sections[failure.Section].Kind == NetworkKind.Road)
                .GroupBy(failure =>
            {
                var segment = sections[failure.Section];
                return string.Join(
                    " + ",
                    new[] {
                        Differs(segment, segment.StartNodeId),
                        Differs(segment, segment.EndNodeId)
                        }.OrderBy(text => text));
            }).OrderByDescending(otherGroup => otherGroup.Count()).Take(12))
                Console.WriteLine($"  short-segment differs {group.Key}: {group.Count()}");

            // Plain-joined pieces: who made the end nodes and whether they are source ports.
            var ports = SourceTopologyObligations.Capture(otherCapture.Source).Ports.ToHashSet();

            var differs = Environment.GetEnvironmentVariable("VISUAL_DEBUG_DIFFERS") ?? "same";
            var same =
                measured.Failures
                    .Where(
                        failure => failure.Category == "short-segment"
                            && sections[failure.Section].Kind == NetworkKind.Road)
                    .Select(failure => sections[failure.Section])
                .Where(
                    segment => Differs(segment, segment.StartNodeId) == differs
                        || Differs(segment, segment.EndNodeId) == differs)
                    .ToArray();

            foreach (var group in same
                .SelectMany(
                    segment => new[] { segment.StartNodeId, segment.EndNodeId }
                        .Where(nodeId => Differs(segment, nodeId) == differs)
                        .Select(
                            nodeId => $"{Origin(nodeId)}{(ports.Contains(new(segment.Kind, nodeId)) ? "/port" : "")} {segment.Classification}{(segment.Bridge ? " bridge" : segment.Tunnel ? " tunnel" : "")}"))
                .GroupBy(text => text).OrderByDescending(otherGroup => otherGroup.Count()).Take(12))
                Console.WriteLine($"  short-segment same-end {group.Key}: {group.Count()}");

            foreach (var segment in same.Take(
                int.TryParse(Environment.GetEnvironmentVariable("VISUAL_DEBUG_SAME"), out var shownSame)
                    ? shownSame
                    : 0))
                Console.WriteLine(
                    $"  same piece way={segment.WayId} {segment.StartNodeId}({Differs(segment, segment.StartNodeId)})->{segment.EndNodeId}({Differs(segment, segment.EndNodeId)}) {segment.Classification} layer={segment.Layer} bridge={segment.Bridge} tunnel={segment.Tunnel} neighbours={string.Join("/", sections.Where(otherSegment => otherSegment != segment && otherSegment.Kind == segment.Kind && (otherSegment.StartNodeId == segment.StartNodeId || otherSegment.EndNodeId == segment.StartNodeId || otherSegment.StartNodeId == segment.EndNodeId || otherSegment.EndNodeId == segment.EndNodeId)).Select(otherSegment => $"L{otherSegment.Layer}{(otherSegment.Bridge ? "b" : otherSegment.Tunnel ? "t" : "")}"))} len={NetworkDesignChecks.HorizontalArcLength(new(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd)):F1} at ({segment.CourseStart.x:F0},{segment.CourseStart.z:F0})");
        }

        if (Environment.GetEnvironmentVariable("VISUAL_DEBUG_KINKS") == "1")
        {
            double lat0 = (double)otherCapture.Input["request"]!["Latitude"]!,
                lon0 = (double)otherCapture.Input["request"]!["Longitude"]!;

            foreach (var group in breaks
                .GroupBy(
                    failure => $"{sections[failure.Section].Kind} {(sections[failure.Section].ProtectedLink ? "link" : sections[failure.Section].Classification)} {NodeClass(failure)} {(failure.Value < 5 ? "2-5°" : failure.Value < 15 ? "5-15°" : "15°+")}")
                .OrderByDescending(otherGroup => otherGroup.Count())
                .Take(14))
                Console.WriteLine($"    kink {group.Key}: {group.Count()}");

            var degreeById =
                sections
                    .SelectMany(
                        segment => new[] { (segment.Kind, segment.StartNodeId), (segment.Kind, segment.EndNodeId) })
                    .GroupBy(item => item)
                    .ToDictionary(group => group.Key, group => group.Count());

            foreach (var group in measured.Failures
                .Where(
                    failure => failure.Category == "short-segment"
                        && sections[failure.Section].Kind == NetworkKind.Road)
                .GroupBy(failure =>
            {
                var segment = sections[failure.Section];

                string End(long id) => Origin(id) + (degreeById[(segment.Kind, id)] > 2 ? "/junction" : "/plain");

                return string.Join(
                    " + ",
                    new[] { End(segment.StartNodeId), End(segment.EndNodeId) }.OrderBy(text => text));
            }).OrderByDescending(otherGroup => otherGroup.Count()).Take(10))
                Console.WriteLine($"    short road pieces between {group.Key}: {group.Count()}");

            foreach (var group in measured.Failures.Where(failure => failure.Category == "short-segment"
                && (Origin(sections[failure.Section].StartNodeId) == "unit"
                    || Origin(sections[failure.Section].EndNodeId) == "unit"))
                    .GroupBy(failure =>
            {
                var segment = sections[failure.Section];
                return $"{segment.Kind} {(segment.ProtectedLink ? "link" : segment.Classification)}{(segment.Bridge ? " bridge" : segment.Tunnel ? " tunnel" : "")}{(segment.PairedProfileId != 0 ? " paired" : "")}";
            }).OrderByDescending(otherGroup => otherGroup.Count()).Take(10))
                Console.WriteLine(
                    $"    short pieces at unit cuts {group.Key}: {group.Count()} (median {group.Select(failure => failure.Value).OrderBy(v => v).ElementAt(group.Count() / 2):F1} m of {group.First().Limit:F0})");

            foreach (var failure in breaks
                .Where(
                    otherFailure => sections[otherFailure.Section].Kind == NetworkKind.Rail
                        || sections[otherFailure.Section].Classification == "motorway")
                .OrderByDescending(otherFailure => otherFailure.Value)
                .Take(8))
                Console.WriteLine(
                    $"    worst kink {sections[failure.Section].Kind} {sections[failure.Section].Reference} way {failure.WayId} {failure.Value:F0}° at {lat0 + math.degrees(failure.Position.z / Earth):F5},{lon0 + math.degrees(failure.Position.x / (Earth * Math.Cos(math.radians(lat0)))):F5} ({NodeClass(failure)})");
        }

        Recluster(output);
    }

    // Tunnel cover on the final stamped terrain: ground (or water surface) above
    // the deck minus the deck, every 5 m, grouped by road reference. A tunnel is
    // exposed in game where its roof (deck + about 6 m) rises above the ground.
    // Also: tunnel and ground-level samples of other roads closer than half their
    // widths at a similar height (tunnels meeting other roads underground).
    // The game's own final check (NetworkGeometrySafety.ValidateTunnelCover): the
    // native tunnel asset's roof across its full width against the stamped
    // terrain. Failing rows are grouped by section; worst shortfall first.
    // Elevated → ground → elevated: a run of ground road sections up to 600 m
    // long between two bridge sections, through plain two-arm nodes, whose deck
    // sags more than 2 m below the straight line between the bridge ends.
    private static void BridgeGapDips(Capture capture, IReadOnlyList<PreparedSegment> sections)
    {
        double lat0 = (double)capture.Input["request"]!["Latitude"]!,
            lon0 = (double)capture.Input["request"]!["Longitude"]!;

        var arms = new Dictionary<long, List<int>>();

        for (var i = 0; i < sections.Count; i++)
            if (sections[i].Kind == NetworkKind.Road)
                foreach (var id in new[] { sections[i].StartNodeId, sections[i].EndNodeId })
                {
                    if (!arms.TryGetValue(id, out var l))
                        arms[id] = l = new();

                    l.Add(i);
                }

        bool Ground(int i) => !sections[i].Bridge && !sections[i].Tunnel;

        var seen = new HashSet<int>();
        var dips = new List<(string Name, double Length, double Sag, double EndsUp, double3 P)>();

        for (var i = 0; i < sections.Count; i++)
        {
            if (sections[i].Kind != NetworkKind.Road || !Ground(i) || !seen.Add(i))
                continue;

            var chain = new List<int> { i };
            var ends = new long[2];
            var outer = new int[2];

            for (var side = 0; side < 2; side++)
            {
                var node = side == 0 ? sections[i].StartNodeId : sections[i].EndNodeId;
                var current = i;
                outer[side] = -1;

                while (true)
                {
                    var list = arms[node];

                    if (list.Count != 2)
                    {
                        ends[side] = node;
                        break;
                    }

                    var next = list[0] == current ? list[1] : list[0];

                    if (!Ground(next))
                    {
                        ends[side] = node;
                        outer[side] = sections[next].Bridge ? next : -1;
                        break;
                    }

                    if (!seen.Add(next))
                    {
                        ends[side] = node;
                        break;
                    }

                    if (side == 0)
                        chain.Insert(0, next);
                    else
                        chain.Add(next);

                    current = next;
                    node = sections[next].StartNodeId == node ? sections[next].EndNodeId : sections[next].StartNodeId;
                }
            }

            if (outer[0] < 0 || outer[1] < 0)
                continue;

            var points = new List<double3>();

            foreach (var k in chain)
            {
                var segment = sections[k];
                var curve =
                    new Cubic3d(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd);

                var otherPoints =
                    Enumerable.Range(0, 9).Select(q => NetworkDesignChecks.Evaluate(curve, q / 8d)).ToList();

                if (points.Count > 0
                    && math.distance(points[^1].xz, otherPoints[^1].xz)
                        < math.distance(points[^1].xz, otherPoints[0].xz))
                    otherPoints.Reverse();

                points.AddRange(otherPoints);
            }

            var at = new double[points.Count];

            for (var k = 1; k < points.Count; k++)
                at[k] = at[k - 1] + math.distance(points[k - 1].xz, points[k].xz);

            var length = at[^1];

            if (length > 600)
                continue;

            var sag = 0d;
            var where = points[0];

            for (var k = 0; k < points.Count; k++)
            {
                var chord = math.lerp(points[0].y, points[^1].y, at[k] / Math.Max(1e-9, length));

                if (chord - points[k].y > sag)
                {
                    sag = chord - points[k].y;
                    where = points[k];
                }
            }

            double HeightAboveGround(double3 point)
                => point.y
                    - NetworkProcessor.Sample(
                        (float)point.x,
                        (float)point.z,
                        capture.Terrain,
                        capture.Width);

            double HeightAt(double2 point)
                => NetworkProcessor.Sample(
                    (float)point.x,
                    (float)point.y,
                    capture.Terrain,
                    capture.Width);

            double BridgeRun(int first, long node)
            {
                var total = 0d;
                var current = first;
                var seenBridge = new HashSet<int>();

                while (seenBridge.Add(current) && sections[current].Bridge)
                {
                    var segment = sections[current];
                    total +=
                        NetworkDesignChecks.HorizontalArcLength(
                            new Cubic3d(
                                segment.CourseStart,
                                segment.CourseControl1,
                                segment.CourseControl2,
                                segment.CourseEnd));

                    node = segment.StartNodeId == node ? segment.EndNodeId : segment.StartNodeId;
                    var list = arms[node];

                    if (list.Count != 2)
                        break;

                    current = list[0] == current ? list[1] : list[0];
                }

                return total;
            }

            var runs = $"{BridgeRun(outer[0], ends[0]):F0}/{BridgeRun(outer[1], ends[1]):F0} m";
            var raised = Enumerable.Range(1, points.Count - 2).Average(k =>
{
    var d = math.normalizesafe(points[k + 1].xz - points[k - 1].xz);
    var side = new double2(-d.y, d.x) * (sections[i].NativeWidth / 2 + 30);

    return HeightAt(points[k].xz) - Math.Max(HeightAt(points[k].xz + side), HeightAt(points[k].xz - side));
});

            if (sections[i].Reference == Environment.GetEnvironmentVariable("VISUAL_DEBUG_DIPS_REF"))
                Console.WriteLine(
                    $"    gap {sections[i].Reference} way {sections[i].WayId} {sections[i].Classification}: bridges {runs}, raised {raised:F1} m, {length:F0} m, {chain.Count} sections, sag {sag:F1} m, deck above ground {points.Min(HeightAboveGround):F1}..{points.Max(HeightAboveGround):F1} m, bridge ends {HeightAboveGround(points[0]):F1}/{HeightAboveGround(points[^1]):F1} m at {lat0 + math.degrees(points[0].z / Earth):F5},{lon0 + math.degrees(points[0].x / (Earth * Math.Cos(math.radians(lat0)))):F5}");

            if (sag > 2)
                dips.Add(
                    (
                        sections[i].Reference.Length > 0
                            ? sections[i].Reference
                            : sections[i].Name.Length > 0 ? sections[i].Name : "way " + sections[i].WayId,
                        length,
                        sag,
                        Math.Min(HeightAboveGround(points[0]), HeightAboveGround(points[^1])),
                        where));
        }

        Console.WriteLine($"  bridge-gap dips (ground run ≤600 m between bridges sagging >2 m): {dips.Count}");

        foreach (var item in dips.OrderByDescending(otherItem => otherItem.Sag).Take(25))
            Console.WriteLine(
                $"    {item.Name}: {item.Length:F0} m sagging {item.Sag:F1} m; ends {item.EndsUp:F1} m above ground; local ({item.P.x:F0},{item.P.z:F0}); at {lat0 + math.degrees(item.P.z / Earth):F5},{lon0 + math.degrees(item.P.x / (Earth * Math.Cos(math.radians(lat0)))):F5}");
    }

    private static void TunnelCover(Capture capture, IReadOnlyList<PreparedSegment> sections, float[] roadbed)
    {
        double lat0 = (double)capture.Input["request"]!["Latitude"]!,
            lon0 = (double)capture.Input["request"]!["Longitude"]!;

        var rows = new List<(string Ref, double Cover, double3 P, bool Wet, long Way)>();

        foreach (var segment in sections.Where(otherSegment => otherSegment.Tunnel))
        {
            var curve =
                new Cubic3d(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd);

            var n = Math.Max(2, (int)(NetworkDesignChecks.HorizontalArcLength(curve) / 5));

            for (var k = 1; k < n; k++)
            {
                var point = NetworkDesignChecks.Evaluate(curve, k / (double)n);
                var ground =
                    NetworkProcessor.Sample((float)point.x, (float)point.z, roadbed, capture.Width);

                var stage = capture.Water(new float3((float)point.x, 0, (float)point.z));
                var wet = stage is float w && w > ground;
                rows.Add(
                    (
                        (segment.Kind == NetworkKind.Rail ? "rail " : "")
                            + (segment.Reference.Length > 0
                                ? segment.Reference
                                : segment.Name.Length > 0 ? segment.Name : "way " + segment.WayId),
                        ground - point.y,
                        point,
                        wet,
                        segment.WayId));
            }
        }

        string Where(double3 point)
            => $"{lat0 + math.degrees(point.z / Earth):F5},{lon0 + math.degrees(point.x / (Earth * Math.Cos(math.radians(lat0)))):F5}";

        if (long.TryParse(Environment.GetEnvironmentVariable("VISUAL_DEBUG_TUNNEL_WAY"), out var tunnelWay))
            foreach (var segment in sections.Where(
                otherSegment => otherSegment.WayId == tunnelWay || otherSegment.SourceWayIds.Contains(tunnelWay)))
            {
                var curve =
                    new Cubic3d(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd);

                var length = NetworkDesignChecks.HorizontalArcLength(curve);
                var cells = string.Join(" ", Enumerable.Range(0, 5).Select(k =>
{
    var point = NetworkDesignChecks.Evaluate(curve, k / 4d);
    return $"[deck {point.y:F1} ground {NetworkProcessor.Sample((float)point.x, (float)point.z, capture.Terrain, capture.Width):F1} stamped {NetworkProcessor.Sample((float)point.x, (float)point.z, roadbed, capture.Width):F1}]";
}));

                Console.WriteLine(
                    $"    way {segment.WayId} {(segment.Tunnel ? "tunnel" : segment.Bridge ? "bridge" : "ground")} {segment.StartNodeId}->{segment.EndNodeId} {length:F0} m: {cells}");
            }

        Console.WriteLine(
            $"  tunnel samples={rows.Count}; cover<6 m={rows.Count(item => item.Cover < 6)}; cover<2 m={rows.Count(item => item.Cover < 2)}; deck above ground={rows.Count(item => item.Cover < 0)}");

        foreach (var group in rows
            .Where(item => item.Cover < 6)
            .GroupBy(item => item.Ref)
            .OrderByDescending(otherGroup => otherGroup.Count())
            .Take(14))
        {
            var worst = group.OrderBy(item => item.Cover).First();
            Console.WriteLine(
                $"    {group.Key}: {group.Count() * 5} m with cover under 6 m ({group.Count(item => item.Wet) * 5} m under water); least {worst.Cover:F1} m at {Where(worst.P)} (way {worst.Way})");
        }
    }

    // Physical connectivity, independent of way provenance: source sections whose
    // two ends both survive (directly or through an alias) but land in different
    // connected pieces of the output graph, and output node ids whose incident
    // ends disagree in position (the game joins a new shared node by exact position).
    private static void Connectivity(
        string label,
        IReadOnlyList<PreparedSegment> source,
        IReadOnlyList<PreparedSegment> output)
    {
        foreach (var kind in new[] { NetworkKind.Rail, NetworkKind.Road })
        {
            var parent = new Dictionary<long, long>();

            long Find(long x)
            {
                if (!parent.TryGetValue(x, out var p))
                {
                    parent[x] = x;
                    return x;
                }

                while (p != x)
                {
                    var g = parent[p];
                    parent[x] = g;
                    x = p;
                    p = g;
                }

                return x;
            }

            var aliases = new Dictionary<long, long>();
            var present = new HashSet<long>();
            var positions = new Dictionary<long, List<float3>>();

            foreach (var segment in output.Where(otherSegment => otherSegment.Kind == kind))
            {
                parent[Find(segment.StartNodeId)] = Find(segment.EndNodeId);
                present.Add(segment.StartNodeId);
                present.Add(segment.EndNodeId);

                foreach (var alias in segment.SourceNodeAliases)
                    aliases[alias.SourceNodeId] = alias.CanonicalNodeId;

                foreach (var (id, point) in new[] {
                    (segment.StartNodeId, segment.Start),
                    (segment.EndNodeId, segment.End)
                    })
                {
                    if (!positions.TryGetValue(id, out var otherPoints))
                        positions[id] = otherPoints = new();

                    otherPoints.Add(point);
                }
            }

            long? Map(long id)
            {
                if (aliases.TryGetValue(id, out var t))
                    id = t;

                return present.Contains(id) ? id : null;
            }

            var broken = new List<PreparedSegment>();
            var lost = 0;

            foreach (var segment in source.Where(otherSegment => otherSegment.Kind == kind))
            {
                var a = Map(segment.StartNodeId);
                var b = Map(segment.EndNodeId);

                if (a == null || b == null)
                {
                    lost++;
                    continue;
                }

                if (Find(a.Value) != Find(b.Value))
                    broken.Add(segment);
            }

            var spread =
                positions.Select(entry => entry.Value.Max(point => math.distance(point, entry.Value[0]))).ToArray();

            var sourceParent = new Dictionary<long, long>();

            long FindSource(long x)
            {
                if (!sourceParent.TryGetValue(x, out var p))
                {
                    sourceParent[x] = x;
                    return x;
                }

                while (p != x)
                {
                    var g = sourceParent[p];
                    sourceParent[x] = g;
                    x = p;
                    p = g;
                }

                return x;
            }

            foreach (var segment in source.Where(otherSegment => otherSegment.Kind == kind))
                sourceParent[FindSource(segment.StartNodeId)] = FindSource(segment.EndNodeId);

            var sourcePieces = sourceParent.Keys.ToArray().Select(FindSource).Distinct().Count();
            // What breaks in game: two different node ids at one place, and two
            // sections joining the same pair of nodes.
            var points = positions.Select(entry => (Id: entry.Key, P: entry.Value[0])).ToArray();
            var grid =
                points
                    .GroupBy(item => ((int)Math.Floor(item.P.x), (int)Math.Floor(item.P.z)))
                    .ToDictionary(group => group.Key, group => group.ToArray());

            var coincident = points.Where(item =>
{
    var cx = (int)Math.Floor(item.P.x);
    var cz = (int)Math.Floor(item.P.z);

    for (var dz = -1; dz <= 1; dz++)
        for (var dx = -1; dx <= 1; dx++)
            if (grid.TryGetValue((cx + dx, cz + dz), out var near))
                foreach (var otherItem in near)
                    if (otherItem.Id != item.Id
                        && math.distance(otherItem.P.xz, item.P.xz) < 1
                        && Math.Abs(otherItem.P.y - item.P.y) < 4)
                        return true;

    return false;
}).ToArray();

            var pairs =
                output
                    .Where(segment => segment.Kind == kind)
                    .GroupBy(
                        segment => (
                            Math.Min(segment.StartNodeId, segment.EndNodeId),
                            Math.Max(segment.StartNodeId, segment.EndNodeId)))
                    .Where(group => group.Count() > 1)
                    .ToArray();

            var lengths =
                output
                    .Where(segment => segment.Kind == kind)
                    .Select(
                        segment => NetworkDesignChecks.HorizontalArcLength(
                            new(
                                segment.CourseStart,
                                segment.CourseControl1,
                                segment.CourseControl2,
                                segment.CourseEnd)))
                    .ToArray();

            Console.WriteLine(
                $"  {label} {kind} sections under 1 m={lengths.Count(l => l < 1)}, under 3 m={lengths.Count(l => l < 3)}, under 8 m={lengths.Count(l => l < 8)}");

            if (Environment.GetEnvironmentVariable("VISUAL_DEBUG_MICRO") == label)
            {
                var degree =
                    output
                        .Where(segment => segment.Kind == kind)
                        .SelectMany(segment => new[] { segment.StartNodeId, segment.EndNodeId })
                        .GroupBy(id => id)
                        .ToDictionary(group => group.Key, group => group.Count());

                foreach (var segment in output
                    .Where(otherSegment => otherSegment.Kind == kind)
                    .Where((otherSegment, i) => lengths[i] < 1)
                    .Take(12))
                    Console.WriteLine(
                        $"    micro way {segment.WayId} {segment.StartNodeId}(deg {degree[segment.StartNodeId]})->{segment.EndNodeId}(deg {degree[segment.EndNodeId]}) br={segment.Bridge} tu={segment.Tunnel} terminal={segment.StartTerminal}/{segment.EndTerminal} at ({segment.Start.x:F1},{segment.Start.z:F1}) len={NetworkDesignChecks.HorizontalArcLength(new(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd)):F2}");
            }

            Console.WriteLine(
                $"  {label} {kind} coincident distinct nodes={coincident.Length}"
                    + (coincident.Length == 0
                        ? ""
                        : $" e.g. {string.Join(" ", coincident.Take(4).Select(item => $"{item.Id}@({item.P.x:F1},{item.P.z:F1})"))}")
                    +
                $"; node pairs joined by several sections={pairs.Length}"
                    + (pairs.Length == 0
                        ? ""
                        : $" e.g. {string.Join(" ", pairs.Take(3).Select(group => $"[{string.Join(",", group.Select(segment => segment.WayId))}]@({group.First().Start.x:F0},{group.First().Start.z:F0})"))}"));

            if (coincident.Length > 0)
            {
                var adjacent =
                    output
                        .Where(segment => segment.Kind == kind)
                        .Select(
                            segment => (
                                Math.Min(segment.StartNodeId, segment.EndNodeId),
                                Math.Max(segment.StartNodeId, segment.EndNodeId)))
                        .ToHashSet();

                var unitOf =
                    output
                        .Where(segment => segment.Kind == kind)
                        .SelectMany(
                            segment => new[] {
                                (segment.StartNodeId, segment.NativeUnitId),
                                (segment.EndNodeId, segment.NativeUnitId)
                                })
                        .GroupBy(item => item.Item1)
                        .ToDictionary(
                            group => group.Key,
                            group => group.Select(item => item.Item2).Distinct().ToArray());

                var degreeOf =
                    output
                        .Where(segment => segment.Kind == kind)
                        .SelectMany(segment => new[] { segment.StartNodeId, segment.EndNodeId })
                        .GroupBy(id => id)
                        .ToDictionary(group => group.Key, group => group.Count());

                var kinds = new Dictionary<string, int>();
                var shown = new List<string>();

                foreach (var item in coincident)
                {
                    var cx = (int)Math.Floor(item.P.x);
                    var cz = (int)Math.Floor(item.P.z);
                    var otherItem = default((long Id, float3 P));

                    for (var dz = -1; dz <= 1; dz++)
                        for (var dx = -1; dx <= 1; dx++)
                            if (grid.TryGetValue((cx + dx, cz + dz), out var near))
                                foreach (var secondItem in near)
                                    if (secondItem.Id != item.Id
                                        && math.distance(secondItem.P.xz, item.P.xz) < 1
                                        && Math.Abs(secondItem.P.y - item.P.y) < 4)
                                        otherItem = secondItem;

                    var linked = adjacent.Contains((Math.Min(item.Id, otherItem.Id), Math.Max(item.Id, otherItem.Id)));
                    var key =
                        (linked ? "joined" : "apart")
                            + $" d={(math.distance(item.P.xz, otherItem.P.xz) < .01 ? "0" : "<1m")} deg={Math.Min(degreeOf[item.Id], 3)}/{Math.Min(degreeOf[otherItem.Id], 3)} {(item.Id <= long.MinValue + 13000000 && item.Id >= long.MinValue + 12000000 ? "unit" : item.Id <= long.MinValue + 9000000 ? "compiler" : item.Id < 0 ? "generated" : "source")}";

                    kinds[key] = kinds.TryGetValue(key, out var n) ? n + 1 : 1;

                    if (!linked && shown.Count < 6)
                        shown.Add(
                            $"{item.Id}/{otherItem.Id}@({item.P.x:F1},{item.P.z:F1}) units {string.Join(",", unitOf[item.Id])}|{string.Join(",", unitOf[otherItem.Id])}");
                }

                Console.WriteLine(
                    $"    coincident kinds: {string.Join("; ", kinds.OrderByDescending(entry => entry.Value).Select(entry => $"{entry.Key}={entry.Value}"))}"
                        + (shown.Count > 0 ? "; apart e.g. " + string.Join(" ", shown) : ""));
            }

            if (Environment.GetEnvironmentVariable("VISUAL_DEBUG_PAIRS") == label)
                foreach (var group in pairs)
                {
                    var firstSegment = group.First();
                    var secondSegment = group.Skip(1).First();

                    double Gap(PreparedSegment otherSegment, PreparedSegment thirdSegment)
                    {
                        var curve =
                            new Cubic3d(
                                otherSegment.CourseStart,
                                otherSegment.CourseControl1,
                                otherSegment.CourseControl2,
                                otherSegment.CourseEnd);

                        var otherCurve =
                            new Cubic3d(
                                thirdSegment.CourseStart,
                                thirdSegment.CourseControl1,
                                thirdSegment.CourseControl2,
                                thirdSegment.CourseEnd);

                        var worst = 0d;

                        for (var k = 0; k <= 20; k++)
                        {
                            var point = NetworkDesignChecks.Evaluate(curve, k / 20d);
                            var best = double.MaxValue;

                            for (var j = 0; j <= 200; j++)
                                best =
                                    Math.Min(
                                        best,
                                        math.distance(point.xz, NetworkDesignChecks.Evaluate(otherCurve, j / 200d).xz));

                            worst = Math.Max(worst, best);
                        }

                        return worst;
                    }

                    string Describe(PreparedSegment segment)
                        => $"way {segment.WayId} {segment.NativeAssetKey} L={NetworkDesignChecks.HorizontalArcLength(new(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd)):F0} br={segment.Bridge} tu={segment.Tunnel} tracks={segment.PhysicalTracks} ways=[{string.Join(",", segment.SourceWayIds)}]";

                    Console.WriteLine(
                        $"    pair x{group.Count()} sep={Gap(firstSegment, secondSegment):F1} m: {Describe(firstSegment)} | {Describe(secondSegment)}");
                }

            Console.WriteLine(
                $"  {label} {kind} connectivity: source sections split across output pieces={broken.Count}; ends without output node={lost}; "
                    +
                $"pieces={present.Select(Find).Distinct().Count()} (source {sourcePieces}); nodes with ends apart >1 cm={spread.Count(d => d > .01f)}, >0.5 m={spread.Count(d => d > .5f)}"
                    +
                (broken.Count == 0
                    ? ""
                    : "; e.g. "
                        + string.Join(
                            " ",
                            broken
                                .Take(5)
                                .Select(
                                    segment => $"way {segment.WayId} {segment.StartNodeId}->{segment.EndNodeId}@({segment.Start.x:F0},{segment.Start.z:F0})"))));
        }
    }

    // The source sections NetworkContinuity counts as unresolved, and why: the way
    // is gone, or its two ends now lie in different connected pieces.
    private static void UnresolvedSections(
        IReadOnlyList<PreparedSegment> source,
        IReadOnlyList<PreparedSegment> sections)
    {
        var neighbours = new Dictionary<(NetworkKind, long), List<long>>();
        var aliases = new Dictionary<(NetworkKind, long), long>();
        var ways = new HashSet<(NetworkKind, long)>();

        foreach (var segment in sections)
        {
            foreach (var (id, next) in new[] {
                (segment.StartNodeId, segment.EndNodeId),
                (segment.EndNodeId, segment.StartNodeId)
                })
            {
                if (!neighbours.TryGetValue((segment.Kind, id), out var list))
                    neighbours[(segment.Kind, id)] = list = new List<long>();

                list.Add(next);
            }

            foreach (var alias in segment.SourceNodeAliases)
                aliases[(segment.Kind, alias.SourceNodeId)] = alias.CanonicalNodeId;

            ways.Add((segment.Kind, segment.WayId));

            foreach (var way in segment.SourceWayIds)
                ways.Add((segment.Kind, way));
        }

        var components = new Dictionary<(NetworkKind, long), int>();
        var sizes = new Dictionary<int, int>();
        var index = 0;

        foreach (var key in neighbours.Keys)
        {
            if (components.ContainsKey(key))
                continue;

            index++;
            var queue = new List<(NetworkKind, long)> { key };
            components[key] = index;

            for (var cursor = 0; cursor < queue.Count; cursor++)
                foreach (var id in neighbours[queue[cursor]])
                    if (components.TryAdd((key.Item1, id), index))
                        queue.Add((key.Item1, id));

            sizes[index] = queue.Count;
        }

        string Where(NetworkKind kind, long id)
        {
            if (components.TryGetValue((kind, id), out var component))
                return $"piece {component} ({sizes[component]} nodes)";

            return aliases.TryGetValue((kind, id), out var canonical)
                ? components.TryGetValue((kind, canonical), out component)
                    ? $"alias {canonical} piece {component} ({sizes[component]} nodes)"
                    : $"alias {canonical} absent"
                : "absent";
        }

        foreach (var segment in source)
        {
            var start = Where(segment.Kind, segment.StartNodeId);
            var end = Where(segment.Kind, segment.EndNodeId);
            var present = !start.EndsWith("absent") && !end.EndsWith("absent");

            if (present
                ? start[start.IndexOf("piece")..] == end[end.IndexOf("piece")..]
                : ways.Contains((segment.Kind, segment.WayId)))
                continue;

            Console.WriteLine(
                $"    unresolved {segment.Kind} way {segment.WayId} {segment.Classification} tunnel={segment.Tunnel} bridge={segment.Bridge} "
                    + $"{segment.StartNodeId}@({segment.Start.x:F0},{segment.Start.z:F0}) {start} -> "
                    + $"{segment.EndNodeId}@({segment.End.x:F0},{segment.End.z:F0}) {end}");
        }
    }

    // NetworkContinuity's rail overlap samples grouped by section pair: how the
    // two tracks relate (shared node, chain ends) and how long they overlap.
    private static void RailOverlaps(IReadOnlyList<PreparedSegment> sections)
    {
        var rails = sections.Where(segment => segment.Kind == NetworkKind.Rail).ToArray();
        var degree =
            rails
                .SelectMany(segment => new[] { segment.StartNodeId, segment.EndNodeId })
                .GroupBy(id => id)
                .ToDictionary(group => group.Key, group => group.Count());

        var samples = new Dictionary<(int, int), List<(int Edge, double3 P, double2 H)>>();
        var pairs = new Dictionary<(int, int), int>();

        for (var e = 0; e < rails.Length; e++)
        {
            var segment = rails[e];
            var curve =
                new Cubic3d(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd);

            var steps = Math.Max(2, (int)Math.Ceiling(NetworkDesignChecks.HorizontalArcLength(curve) / 4));

            for (var i = 1; i < steps; i++)
            {
                var t = i / (double)steps;
                var point = NetworkDesignChecks.Evaluate(curve, t);
                var h = math.normalizesafe(NetworkDesignChecks.Derivative(curve, t).xz);
                var x = (int)Math.Floor(point.x / 4);
                var z = (int)Math.Floor(point.z / 4);

                for (var dz = -1; dz <= 1; dz++)
                    for (var dx = -1; dx <= 1; dx++)
                        if (samples.TryGetValue((x + dx, z + dz), out var list))
                            foreach (var otherItem in list)
                            {
                                if (otherItem.Edge == e)
                                    continue;

                                var otherSegment = rails[otherItem.Edge];

                                if (otherSegment.Bridge != segment.Bridge
                                    || otherSegment.Tunnel != segment.Tunnel
                                    || Math.Abs(point.y - otherItem.P.y) >= 7)
                                    continue;

                                if (math.distance(point.xz, otherItem.P.xz) >= 3
                                    || Math.Abs(math.dot(h, otherItem.H)) < .97)
                                    continue;

                                var shared =
                                    new[] { segment.StartNodeId, segment.EndNodeId }
                                        .Intersect(new[] { otherSegment.StartNodeId, otherSegment.EndNodeId })
                                        .ToArray();

                                if (shared.Any(
                                    id => math.distance(
                                        point.xz,
                                        (id == segment.StartNodeId ? segment.CourseStart : segment.CourseEnd).xz)
                                        < 12))
                                    continue;

                                var key = (Math.Min(e, otherItem.Edge), Math.Max(e, otherItem.Edge));
                                pairs[key] = pairs.TryGetValue(key, out var n) ? n + 1 : 1;
                            }

                if (!samples.TryGetValue((x, z), out var bucket))
                    samples[(x, z)] = bucket = new();

                bucket.Add((e, point, h));
            }
        }

        // Chains between nodes of degree other than two.
        var incident =
            rails
                .Select((segment, i) => (segment, i))
                .SelectMany(
                    item => new[] { (Id: item.segment.StartNodeId, item.i), (Id: item.segment.EndNodeId, item.i) })
                .GroupBy(item => item.Id)
                .ToDictionary(group => group.Key, group => group.Select(item => item.i).ToArray());

        var chainOf = new int[rails.Length];
        Array.Fill(chainOf, -1);
        var chainEnds = new List<(long A, long B)>();

        for (var i = 0; i < rails.Length; i++)
        {
            if (chainOf[i] >= 0)
                continue;

            var id = chainEnds.Count;
            var ends = new List<long>();

            foreach (var forward in new[] { true, false })
            {
                var index = i;
                var node = forward ? rails[i].EndNodeId : rails[i].StartNodeId;
                chainOf[i] = id;

                while (degree[node] == 2)
                {
                    var next = incident[node][0] == index ? incident[node][1] : incident[node][0];

                    if (chainOf[next] >= 0)
                        break;

                    chainOf[next] = id;
                    index = next;
                    node = rails[next].StartNodeId == node ? rails[next].EndNodeId : rails[next].StartNodeId;
                }

                ends.Add(node);
            }

            chainEnds.Add((Math.Min(ends[0], ends[1]), Math.Max(ends[0], ends[1])));
        }

        var chainPairs =
            pairs
                .GroupBy(
                    entry => (
                        Math.Min(chainOf[entry.Key.Item1], chainOf[entry.Key.Item2]),
                        Math.Max(chainOf[entry.Key.Item1], chainOf[entry.Key.Item2])))
                .ToDictionary(group => group.Key, group => group.Sum(entry => entry.Value));

        string ChainShape((int X, int Y) item)
        {
            if (item.X == item.Y)
                return "same chain";

            var firstItem = chainEnds[item.X];
            var secondItem = chainEnds[item.Y];
            var shared = new[] { firstItem.A, firstItem.B }.Intersect(new[] { secondItem.A, secondItem.B }).Count();

            return shared == 2
                ? "duplicate route"
                : shared == 1 ? "branch from shared junction" : "independent parallel";
        }

        foreach (var group in chainPairs
            .GroupBy(entry => ChainShape(entry.Key))
            .OrderByDescending(otherGroup => otherGroup.Sum(entry => entry.Value)))
            Console.WriteLine(
                $"    chains {group.Key}: pairs={group.Count()} overlap≈{4 * group.Sum(entry => entry.Value)} m");

        if (Environment.GetEnvironmentVariable("VISUAL_DEBUG_RAIL_OVERLAP") == "2")
        {
            double ChainLength(int c)
                => rails
                    .Where((segment, i) => chainOf[i] == c)
                    .Sum(
                        segment => NetworkDesignChecks.HorizontalArcLength(
                            new(
                                segment.CourseStart,
                                segment.CourseControl1,
                                segment.CourseControl2,
                                segment.CourseEnd)));

            string Service(int c)
                => string.Join(
                    "+",
                    rails
                        .Where((segment, i) => chainOf[i] == c)
                        .Select(segment => segment.RailService.Length == 0 ? "main" : segment.RailService)
                        .Distinct());

            foreach (var entry in chainPairs.OrderByDescending(otherEntry => otherEntry.Value))
            {
                var at =
                    rails[pairs.Keys.First(
                        item => (
                            Math.Min(chainOf[item.Item1], chainOf[item.Item2]),
                            Math.Max(chainOf[item.Item1], chainOf[item.Item2]))
                            == entry.Key).Item1].Start;

                Console.WriteLine(
                    $"    pair {ChainShape(entry.Key)} overlap≈{4 * entry.Value} m; chain A {ChainLength(entry.Key.Item1):F0} m {Service(entry.Key.Item1)} ends deg {degree[chainEnds[entry.Key.Item1].A]}/{degree[chainEnds[entry.Key.Item1].B]}; "
                        +
                    $"chain B {ChainLength(entry.Key.Item2):F0} m {Service(entry.Key.Item2)} ends deg {degree[chainEnds[entry.Key.Item2].A]}/{degree[chainEnds[entry.Key.Item2].B]}; at ({at.x:F0},{at.z:F0})");
            }
        }

        string Shape(PreparedSegment firstSegment, PreparedSegment secondSegment)
        {
            var shared =
                new[] { firstSegment.StartNodeId, firstSegment.EndNodeId }
                    .Intersect(new[] { secondSegment.StartNodeId, secondSegment.EndNodeId })
                    .Count();

            return $"shared={shared} deg=[{degree[firstSegment.StartNodeId]},{degree[firstSegment.EndNodeId]}|{degree[secondSegment.StartNodeId]},{degree[secondSegment.EndNodeId]}] sameWay={firstSegment.WayId == secondSegment.WayId || firstSegment.SourceWayIds.Intersect(secondSegment.SourceWayIds).Any()}";
        }

        Console.WriteLine($"  rail overlap pairs={pairs.Count}; samples={pairs.Values.Sum()}");

        foreach (var group in pairs
            .GroupBy(entry => Shape(rails[entry.Key.Item1], rails[entry.Key.Item2]))
            .OrderByDescending(otherGroup => otherGroup.Sum(entry => entry.Value))
            .Take(10))
            Console.WriteLine($"    {group.Key}: pairs={group.Count()} samples={group.Sum(entry => entry.Value)}");

        foreach (var entry in pairs.OrderByDescending(otherEntry => otherEntry.Value).Take(8))
        {
            var firstSegment = rails[entry.Key.Item1];
            var secondSegment = rails[entry.Key.Item2];
            Console.WriteLine(
                $"    worst {entry.Value}: way {firstSegment.WayId} {firstSegment.StartNodeId}->{firstSegment.EndNodeId} | way {secondSegment.WayId} {secondSegment.StartNodeId}->{secondSegment.EndNodeId} at ({firstSegment.Start.x:F0},{firstSegment.Start.z:F0}) ways=[{string.Join(",", firstSegment.SourceWayIds)}]|[{string.Join(",", secondSegment.SourceWayIds)}] service={firstSegment.RailService}/{secondSegment.RailService}");
        }
    }

    // Breaks in the live native network a diagnostics run recorded: dead-end
    // nodes lying within 3 m of a different node of the same kind.
    public static void LiveBreaks(string folder)
    {
        var names =
            CapturedRunFiles
                .Names(folder, "native-owned-final-", ".ndjson.gz")
                .Where(
                    text => System.Text.RegularExpressions.Regex.IsMatch(text, @"^native-owned-final-\d+\.ndjson\.gz$"))
                .ToArray();

        var records =
            names.SelectMany(text => CapturedRunFiles.Records(CapturedRunFiles.Resolve(folder, text))).ToArray();

        if (Environment.GetEnvironmentVariable("LIVE_SIDES") == "1")
        {
            // Native side compositions against the deck's height above the captured
            // terrain at mid-span: where the game switches between walls and pillars.
            var capture = Load(folder);

            foreach (var group in records
                .Where(row => (string?)row["kind"] == "Road")
                .GroupBy(
                    row => $"general={(string?)row["general"]} left={(string?)row["left"]} right={(string?)row["right"]}")
                .OrderByDescending(otherGroup => otherGroup.Count())
                .Take(16))
            {
                var heights = group.Select(row =>
{
    var a = ReadPoint(row["a"]!);
    var d = ReadPoint(row["d"]!);
    var m = (a + d) * .5f;

    return (double)m.y - NetworkProcessor.Sample(m.x, m.z, capture.Terrain, capture.Width);
}).OrderBy(v => v).ToArray();

                double Quantile(double q) => heights[(int)(q * (heights.Length - 1))];

                Console.WriteLine(
                    $"  {group.Key}: {group.Count()} edges; deck above terrain P10 {Quantile(.1):F1} m, P50 {Quantile(.5):F1} m, P90 {Quantile(.9):F1} m");
            }

            return;
        }

        if (Environment.GetEnvironmentVariable("LIVE_TUNNELS") == "1")
        {
            // Edges below the captured terrain: how deep each native composition
            // sits, to find the depth at which the game draws a tunnel rather than
            // a lowered (open) road.
            var capture = Load(folder);

            foreach (var group in records
                .Select(
                    row => (
                        row,
                        Depth: (double)NetworkProcessor.Sample(
                            ((ReadPoint(row["a"]!) + ReadPoint(row["d"]!)) * .5f).x,
                            ((ReadPoint(row["a"]!) + ReadPoint(row["d"]!)) * .5f).z,
                            capture.Terrain,
                            capture.Width)
                            - ((ReadPoint(row["a"]!) + ReadPoint(row["d"]!)) * .5f).y))
                .Where(item => item.Depth > 1)
                    .GroupBy(
                        item => $"{(string?)item.row["kind"]} general={(string?)item.row["general"]} left={(string?)item.row["left"]} right={(string?)item.row["right"]}")
                    .OrderByDescending(otherGroup => otherGroup.Count())
                    .Take(14))
            {
                var d = group.Select(item => item.Depth).OrderBy(v => v).ToArray();

                double Quantile(double q) => d[(int)(q * (d.Length - 1))];

                Console.WriteLine(
                    $"  {group.Key}: {group.Count()} edges; depth below terrain min {d[0]:F1}, P10 {Quantile(.1):F1}, P50 {Quantile(.5):F1}, P90 {Quantile(.9):F1} m");
            }

            // Submitted tunnel sections, and whether the live edge over their midpoint is a tunnel.
            var prepared =
                CorrectiveCapturedReplay
                    .ReadManifest<JObject>(folder, "final-prepared-network-chunks.json")
                    .Where(row => (bool)row["Tunnel"]!)
                    .ToArray();

            var cells = records.GroupBy(row =>
{
    var m = (ReadPoint(row["a"]!) + ReadPoint(row["d"]!)) * .5f;
    return ((int)Math.Floor(m.x / 16), (int)Math.Floor(m.z / 16));
}).ToDictionary(group => group.Key, group => group.ToArray());

            var outcome = new List<(bool Tunnel, double Depth, double Elevation, float3 P, string Way)>();

            foreach (var row in prepared)
            {
                var a = ReadPoint(row["a"]!);
                var dd = ReadPoint(row["d"]!);
                var m = (a + dd) * .5f;
                var cx = (int)Math.Floor(m.x / 16);
                var cz = (int)Math.Floor(m.z / 16);
                JToken? live = null;
                var best = 4f;

                for (var z = cz - 1; z <= cz + 1; z++)
                    for (var x = cx - 1; x <= cx + 1; x++)
                        if (cells.TryGetValue((x, z), out var list))
                            foreach (var secondRecord in list)
                            {
                                var em = (ReadPoint(secondRecord["a"]!) + ReadPoint(secondRecord["d"]!)) * .5f;
                                var otherDistance = math.distance(em.xz, m.xz);

                                if (otherDistance < best)
                                {
                                    best = otherDistance;
                                    live = secondRecord;
                                }
                            }

                if (live == null)
                    continue;

                var depth = NetworkProcessor.Sample(m.x, m.z, capture.Terrain, capture.Width) - m.y;
                var elevation = (double?)row["StartElevation"] ?? double.NaN;
                outcome.Add(
                    (
                        ((string?)live["general"] ?? "").Contains("Tunnel"),
                        depth,
                        elevation,
                        m,
                        (string)row["WayId"]!));
            }

            Console.WriteLine(
                $"  submitted tunnel sections matched to live edges: {outcome.Count}; drawn as tunnel {outcome.Count(otherItem => otherItem.Tunnel)}; drawn open {outcome.Count(otherItem => !otherItem.Tunnel)}");

            foreach (var band in new[] { (0d, 5d), (5d, 8d), (8d, 10d), (10d, 12d), (12d, 1000d) })
            {
                var items =
                    outcome.Where(otherItem => otherItem.Depth >= band.Item1 && otherItem.Depth < band.Item2).ToArray();

                if (items.Length > 0)
                    Console.WriteLine(
                        $"    depth {band.Item1}-{band.Item2} m: {items.Length} sections, {items.Count(otherItem => !otherItem.Tunnel)} drawn open; submitted elevation P50 {items.Select(otherItem => otherItem.Elevation).OrderBy(v => v).ElementAt(items.Length / 2):F1}");
            }

            return;
        }

        // LIVE_NEAR=x,z,radius: every live edge with an end inside that circle, as recorded.
        if (Environment.GetEnvironmentVariable("LIVE_NEAR") is { Length: > 0 } circle)
        {
            var parts =
                circle.Split(',')
                    .Select(text => float.Parse(text, System.Globalization.CultureInfo.InvariantCulture))
                    .ToArray();

            var centre = new float2(parts[0], parts[1]);

            foreach (var row in records)
            {
                var a = ReadPoint(row["a"]!);
                var d = ReadPoint(row["d"]!);

                if (math.distance(a.xz, centre) > parts[2] && math.distance(d.xz, centre) > parts[2])
                    continue;

                Console.WriteLine($"  NEAR {row.ToString(Newtonsoft.Json.Formatting.None)}");
            }

            // The sections as they were sent to the game, for the same circle.
            foreach (var row in CorrectiveCapturedReplay.ReadManifest<JObject>(folder, "final-prepared-network-chunks.json"))
            {
                var a = ReadPoint(row["a"]!);
                var d = ReadPoint(row["d"]!);

                if (math.distance(a.xz, centre) > parts[2] && math.distance(d.xz, centre) > parts[2])
                    continue;

                Console.WriteLine(
                    $"  SENT way {row["WayId"]} {row["Classification"]} oneway={row["OneWay"]} bridge={row["Bridge"]} width={row["NativeWidth"]} a({a.x:0.0},{a.y:0.0},{a.z:0.0}) d({d.x:0.0},{d.y:0.0},{d.z:0.0})");
            }

            return;
        }

        foreach (var kind in new[] { "Rail", "Road" })
        {
            var ends =
                records
                    .Where(row => (string?)row["kind"] == kind)
                    .SelectMany(
                        row => new[]{
                            (
                                Node:(string)row["startNode"]!,
                                P:ReadPoint(row["a"]!),
                                Edge:(string)row["entity"]!,
                                Elevated:((string?)row["general"]??"").Contains("Elevated")),
                (
                    Node:(string)row["endNode"]!,
                    P:ReadPoint(row["d"]!),
                    Edge:(string)row["entity"]!,
                    Elevated:((string?)row["general"]??"").Contains("Elevated"))})
                    .ToArray();

            var degree = ends.GroupBy(item => item.Node).ToDictionary(group => group.Key, group => group.Count());
            var cells =
                ends
                    .GroupBy(item => ((int)Math.Floor(item.P.x / 4), (int)Math.Floor(item.P.z / 4)))
                    .ToDictionary(group => group.Key, group => group.ToArray());

            var breaks = new List<(string Node, float3 P, float Gap, float Rise, bool Elevated, int OtherDegree)>();

            foreach (var item in ends.Where(otherItem => degree[otherItem.Node] == 1))
            {
                var best = (Gap: float.MaxValue, Rise: 0f, Degree: 0);
                var cx = (int)Math.Floor(item.P.x / 4);
                var cz = (int)Math.Floor(item.P.z / 4);

                for (var dz = -1; dz <= 1; dz++)
                    for (var dx = -1; dx <= 1; dx++)
                        if (cells.TryGetValue((cx + dx, cz + dz), out var near))
                            foreach (var otherItem in near)
                                if (otherItem.Node != item.Node)
                                {
                                    var otherDistance = math.distance(otherItem.P.xz, item.P.xz);

                                    if (otherDistance < best.Gap)
                                        best = (otherDistance, otherItem.P.y - item.P.y, degree[otherItem.Node]);
                                }

                if (best.Gap < 3)
                    breaks.Add((item.Node, item.P, best.Gap, best.Rise, item.Elevated, best.Degree));
            }

            Console.WriteLine(
                $"live {kind}: edges={ends.Length / 2}; nodes={degree.Count}; dead ends={degree.Count(entry => entry.Value == 1)}; dead ends within 3 m of another node={breaks.Count}");

            if (Environment.GetEnvironmentVariable("LIVE_PLANNED") == "1"
                && CapturedRunFiles.Optional(folder, "final-prepared-network-chunks.json") != null)
            {
                // Each live dead end against the plan: how many submitted section ends
                // met there. Two or more means the game broke a planned connection.
                var plan =
                    CorrectiveCapturedReplay
                        .ReadManifest<JObject>(folder, "final-prepared-network-chunks.json")
                        .Where(row => (string?)row["kind"] == kind)
                    .SelectMany(
                        row => new[] {
                            (
                                P: ReadPoint(row["nodeStart"]!),
                                Way: (string)row["WayId"]!,
                                Tunnel: (bool)row["Tunnel"]!,
                                Bridge: (bool)row["Bridge"]!),
                            (
                                P: ReadPoint(row["nodeEnd"]!),
                                Way: (string)row["WayId"]!,
                                Tunnel: (bool)row["Tunnel"]!,
                                Bridge: (bool)row["Bridge"]!)
                            })
                        .ToArray();

                var planCells =
                    plan
                        .GroupBy(item => ((int)Math.Floor(item.P.x / 8), (int)Math.Floor(item.P.z / 8)))
                        .ToDictionary(group => group.Key, group => group.ToArray());

                var c0 = Load(folder);
                double la = (double)c0.Input["request"]!["Latitude"]!, low = (double)c0.Input["request"]!["Longitude"]!;
                var broken = new List<string>();
                var natural = 0;

                foreach (var item in ends.Where(otherItem => degree[otherItem.Node] == 1))
                {
                    var cx = (int)Math.Floor(item.P.x / 8);
                    var cz = (int)Math.Floor(item.P.z / 8);
                    var met = new List<(float3 P, string Way, bool Tunnel, bool Bridge)>();

                    for (var dz = -1; dz <= 1; dz++)
                        for (var dx = -1; dx <= 1; dx++)
                            if (planCells.TryGetValue((cx + dx, cz + dz), out var near))
                                met.AddRange(near.Where(otherItem => math.distance(otherItem.P.xz, item.P.xz) < 1.5f));

                    if (met.Count < 2)
                    {
                        natural++;
                        continue;
                    }

                    broken.Add(
                        $"{la + math.degrees(item.P.z / Earth):F5},{low + math.degrees(item.P.x / (Earth * Math.Cos(math.radians(la)))):F5} planned ends {met.Count} [{string.Join(",", met.Select(otherItem => otherItem.Way + (otherItem.Tunnel ? "/T" : otherItem.Bridge ? "/B" : "")))}]");
                }

                Console.WriteLine($"  live dead ends the plan connected: {broken.Count}; planned dead ends: {natural}");

                foreach (var text in broken.Take(30))
                    Console.WriteLine("    " + text);
            }

            foreach (var group in breaks.GroupBy(
                item => item.Gap < .01f ? "coincident" : item.Gap < .5f ? "under 0.5 m" : "0.5-3 m"))
                Console.WriteLine(
                    $"  {group.Key}: {group.Count()}; rise P50 {group.Select(item => Math.Abs(item.Rise)).OrderBy(v => v).ElementAt(group.Count() / 2):F2} m; elevated {group.Count(item => item.Elevated)}; partner degree 1/2/3+ = {group.Count(item => item.OtherDegree == 1)}/{group.Count(item => item.OtherDegree == 2)}/{group.Count(item => item.OtherDegree > 2)}; e.g. "
                        +
                    string.Join(
                        " ",
                        group
                            .Take(4)
                            .Select(
                                item => $"{item.Node}@({item.P.x:F1},{item.P.y:F1},{item.P.z:F1}) gap {item.Gap:F2} rise {item.Rise:F2}")));

            if (kind != "Rail" || CapturedRunFiles.Optional(folder, "final-prepared-network-chunks.json") == null)
                continue;

            // The submitted sections meeting at each break, with their placement order.
            var prepared = CorrectiveCapturedReplay.ReadManifest<JObject>(folder, "final-prepared-network-chunks.json");

            const float Radius = 1;

            bool Near(JObject row, float3 spot)
            {
                var curve =
                    new Cubic3d(
                        ReadPoint(row["a"]!),
                        ReadPoint(row["b"]!),
                        ReadPoint(row["c"]!),
                        ReadPoint(row["d"]!));

                for (var k = 0; k <= 200; k++)
                    if (math.distance(NetworkDesignChecks.Evaluate(curve, k / 200d).xz, (double2)spot.xz) < 2)
                        return true;

                return false;
            }

            foreach (var spot in breaks
                .Select(item => item.P)
                .Distinct()
                .Aggregate(new List<float3>(), (points, point) =>
{
    if (!points.Any(otherPoint => math.distance(otherPoint.xz, point.xz) < 5))
        points.Add(point);

    return points;
}).Take(int.TryParse(Environment.GetEnvironmentVariable("LIVE_SPOTS"), out var n) ? n : 6))
            {
                Console.WriteLine($"  at ({spot.x:F1},{spot.y:F1},{spot.z:F1}):");

                foreach (var row in prepared.Where(
                    secondRecord => (string?)secondRecord["kind"] == "Rail"
                        && (math.distance(ReadPoint(secondRecord["nodeStart"]!).xz, spot.xz) < Radius
                            || math.distance(ReadPoint(secondRecord["nodeEnd"]!).xz, spot.xz) < Radius
                            || Near(secondRecord, spot))))
                {
                    var atStart =
                        math.distance(ReadPoint(row["nodeStart"]!).xz, spot.xz)
                            <= math.distance(ReadPoint(row["nodeEnd"]!).xz, spot.xz);

                    var end = atStart ? ReadPoint(row["nodeStart"]!) : ReadPoint(row["nodeEnd"]!);
                    Console.WriteLine(
                        $"    way {row["WayId"]} {(atStart ? "start" : "end")} d={math.distance(end.xz, spot.xz):F1} node {(atStart ? row["StartNodeId"] : row["EndNodeId"])} y={end.y:F2} unit={row["NativeUnitId"]} asset={row["NativeAssetKey"]} bridge={row["Bridge"]} tunnel={row["Tunnel"]} layer={row["Layer"]} class={row["Classification"]} elev={(atStart ? row["StartElevation"] : row["EndElevation"])} aliases={row["SourceNodeAliases"]!.ToString(Formatting.None)}");
                }
            }
        }
    }

    public static void Recluster(string output)
    {
        var run = JObject.Parse(File.ReadAllText(Path.Combine(output, RunFile)));
        var failures = new List<VisualCompilerMetrics.Failure>();

        using (var file =
            File.OpenRead(
                Path.Combine(
                    output,
                    FailuresFile))) using (var zip =
                        new GZipStream(file, CompressionMode.Decompress)) using (var reader = new StreamReader(zip))
            while (reader.ReadLine() is { } line)
            {
                var row = JObject.Parse(line);
                failures.Add(
                    new(
                        (string)row["Category"]!,
                        (int)row["Section"]!,
                        (long)row["WayId"]!,
                        (long)row["StartNodeId"]!,
                        (long)row["EndNodeId"]!,
                    new double3((double)row["x"]!, (double)row["y"]!, (double)row["z"]!),
                        (double)row["Value"]!,
                        (double)row["Limit"]!,
                        (double)row["Severity"]!));
            }

        var issues = VisualCompilerMetrics.Cluster(failures, default);
        var kilometres = (double)run["Kilometres"]!;
        double lat0 = (double)run["latitude"]!, lon0 = (double)run["longitude"]!;

        object Located(VisualCompilerMetrics.Issue issue) => new
        {
            issue.Category,
            issue.Failures,
            issue.Severity,
            issue.Value,
            issue.Limit,
            issue.Ways,
            x = issue.Position.x,
            y = issue.Position.y,
            z = issue.Position.z,
            latitude = lat0 + math.degrees(issue.Position.z / Earth),
            longitude = lon0 + math.degrees(issue.Position.x / (Earth * Math.Cos(math.radians(lat0))))
        };

        var categories =
            failures
                .Select(failure => failure.Category)
                .Concat(issues.Select(issue => issue.Category))
                .Distinct()
                .OrderBy(text => text, StringComparer.Ordinal)
                .Select(text => new
                {
                    category = text,
                    failures = failures.Count(failure => failure.Category == text),
                    issues = issues.Count(issue => issue.Category == text),
                    issuesPer10Km = issues.Count(issue => issue.Category == text) / Math.Max(1e-9, kilometres) * 10
                }).ToArray();

        File.WriteAllText(Path.Combine(output, "visual-metrics.json"), JsonConvert.SerializeObject(new
        {
            run,
            totals = new
            {
                failures = failures.Count,
                issues = issues.Count,
                issuesPer10Km = issues.Count / Math.Max(1e-9, kilometres) * 10
            },
            categories,
            worstIssues = issues.Take(60).Select(Located),
            worstByCategory = categories.ToDictionary(
                c => c.category,
                c => issues.Where(issue => issue.Category == c.category).Take(10).Select(Located))
        }, Formatting.Indented));

        Console.WriteLine(
            $"{run["capture"]} [{run["pipeline"]}]: {run["Sections"]} sections, {kilometres:F1} km, {issues.Count} issues ({issues.Count / Math.Max(1e-9, kilometres) * 10:F2} per 10 km)");

        foreach (var c in categories)
            Console.WriteLine(
                $"  {c.category,-20} failures={c.failures,7} issues={c.issues,6} per10km={c.issuesPer10Km,7:F2}");

        var paired = run["paired"]!;
        Console.WriteLine(
            $"  paired candidates: eligible={(double)paired["EligibleMeters"]! / 1000:F1} km matched={(double)paired["MatchedMeters"]! / 1000:F1} km failing={(double)paired["FailedMeters"]! / 1000:F1} km");
    }

    // Side-by-side plan views (legacy left, compiler right) of fixed spots, each
    // spot "name@latitude,longitude"; 700 m square windows, widths to scale.
    public static void Plot(string legacy, string compiler, string spots, string svgPath)
    {
        var run = JObject.Parse(File.ReadAllText(Path.Combine(legacy, RunFile)));
        double lat0 = (double)run["latitude"]!, lon0 = (double)run["longitude"]!;

        static List<(string Kind, bool Structure, float Width, double[] P, string Label)> Sections(string folder)
        {
            var list = new List<(string, bool, float, double[], string)>();
            using var file = File.OpenRead(Path.Combine(folder, PlotFile));
            using var zip = new GZipStream(file, CompressionMode.Decompress);
            using var reader = new StreamReader(zip);

            while (reader.ReadLine() is { } line)
            {
                var row = JObject.Parse(line);
                list.Add(
                    (
                        (string)row["kind"]!,
                        (bool)row["Bridge"]! || (bool)row["Tunnel"]!,
                        (float)row["NativeWidth"]!,
                        row["p"]!.Values<double>().ToArray(),
                                $"{(long)row["WayId"]!} {(string?)row["Classification"]}{((bool?)row["OneWay"] == true ? "" : "2")} w{(float)row["NativeWidth"]!:F0}"));
            }

            return list;
        }

        var sides = new[] { Sections(legacy), Sections(compiler) };
        var names = spots.Split(';', StringSplitOptions.RemoveEmptyEntries);
        // VISUAL_PLOT_HALF zooms in (half the square's side, metres) and labels each section's way.
        var zoom =
            double.TryParse(
                Environment.GetEnvironmentVariable("VISUAL_PLOT_HALF"),
                NumberStyles.Float,
                CultureInfo.InvariantCulture,
                out var h);

        var half = zoom ? h : 350;
        const int panel = 560;
        var svg = new StringBuilder();
        svg.Append(
            $"<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{panel * 2 + 30}\" height=\"{names.Length * (panel + 40)}\"><rect width=\"100%\" height=\"100%\" fill=\"#1d2428\"/>");

        string FormatNumber(double v) => v.ToString("0.#", CultureInfo.InvariantCulture);

        for (var row = 0; row < names.Length; row++)
        {
            var parts = names[row].Split('@');
            var ll = parts[1].Split(',');
            var lat = double.Parse(ll[0], CultureInfo.InvariantCulture);
            var lon = double.Parse(ll[1], CultureInfo.InvariantCulture);
            var cx = math.radians(lon - lon0) * Earth * Math.Cos(math.radians(lat0));
            var cz = math.radians(lat - lat0) * Earth;
            var top = row * (panel + 40);

            for (var side = 0; side < 2; side++)
            {
                var left = 10 + side * (panel + 10);
                var count = 0;
                svg.Append(
                    $"<text x=\"{left}\" y=\"{top + 20}\" fill=\"#d7e1e7\" font-size=\"14\">{parts[0]} — {(side == 0 ? "legacy" : "visual compiler")}</text>");

                svg.Append(
                    $"<svg x=\"{left}\" y=\"{top + 28}\" width=\"{panel}\" height=\"{panel}\" viewBox=\"{FormatNumber(cx - half)} {FormatNumber(-cz - half)} {FormatNumber(2 * half)} {FormatNumber(2 * half)}\"><rect x=\"{FormatNumber(cx - half)}\" y=\"{FormatNumber(-cz - half)}\" width=\"{FormatNumber(2 * half)}\" height=\"{FormatNumber(2 * half)}\" fill=\"#26333a\"/>");

                foreach (var item in sides[side])
                {
                    var p = item.P;

                    if (Math.Max(Math.Max(p[0], p[3]), Math.Max(p[6], p[9])) < cx - half
                        || Math.Min(Math.Min(p[0], p[3]), Math.Min(p[6], p[9])) > cx + half
                        || Math.Max(Math.Max(p[2], p[5]), Math.Max(p[8], p[11])) < cz - half
                                                || Math.Min(Math.Min(p[2], p[5]), Math.Min(p[8], p[11])) > cz + half)
                        continue;

                    var color = item.Kind == "Rail" ? "#b98a5e" : item.Structure ? "#7fb3d5" : "#c9cfd3";
                    count++;
                    svg.Append(
                        $"<path d=\"M{FormatNumber(p[0])} {FormatNumber(-p[2])} C{FormatNumber(p[3])} {FormatNumber(-p[5])} {FormatNumber(p[6])} {FormatNumber(-p[8])} {FormatNumber(p[9])} {FormatNumber(-p[11])}\" fill=\"none\" stroke=\"{color}\" stroke-opacity=\".75\" stroke-width=\"{FormatNumber(Math.Max(2, item.Width))}\"/>");

                    svg.Append(
                        $"<circle cx=\"{FormatNumber(p[0])}\" cy=\"{FormatNumber(-p[2])}\" r=\"{FormatNumber(zoom ? half / 150 : 2.5)}\" fill=\"#e8a33d\"/>");

                    if (zoom)
                    {
                        var hue = (count * 67) % 360;
                        var m =
                            new[] { (p[0] + 3 * p[3] + 3 * p[6] + p[9]) / 8, (p[2] + 3 * p[5] + 3 * p[8] + p[11]) / 8 };

                        svg.Append(
                            $"<path d=\"M{FormatNumber(p[0])} {FormatNumber(-p[2])} C{FormatNumber(p[3])} {FormatNumber(-p[5])} {FormatNumber(p[6])} {FormatNumber(-p[8])} {FormatNumber(p[9])} {FormatNumber(-p[11])}\" fill=\"none\" stroke=\"hsl({hue},80%,45%)\" stroke-width=\"{FormatNumber(half / 250)}\"/>");

                        svg.Append(
                            $"<text x=\"{FormatNumber(m[0])}\" y=\"{FormatNumber(-m[1])}\" fill=\"hsl({hue},90%,30%)\" font-size=\"{FormatNumber(half / 30)}\">{item.Label}</text>");
                    }
                }

                svg.Append(
                    $"</svg><text x=\"{left + 6}\" y=\"{top + panel + 22}\" fill=\"#93a4ad\" font-size=\"11\">{count} sections (orange dots = section starts); 700 m square; blue = bridge/tunnel</text>");
            }
        }

        svg.Append("</svg>");
        File.WriteAllText(svgPath, svg.ToString());
        Console.WriteLine("Plot: " + svgPath);
    }

    internal static float3 ReadPoint(JToken token) => new((float)token["x"]!, (float)token["y"]!, (float)token["z"]!);

    private static string Origin(long id) => id >= 0
                        ? "source"
                        : id <= long.MinValue + 9000000
                            ? "compiler"
                            : id <= long.MinValue + 13000000
                                ? "unit"
                                : id <= long.MinValue + 44000000 ? "late" : "earlier";

    private static string ClassOf(PreparedSegment segment) => segment.ProtectedLink ? "link" : segment.Classification;
}
