using ImportEarthLocation.Processing;
using Newtonsoft.Json.Linq;
using Unity.Mathematics;

// Terrain through the import, from one capture's three heightmaps: before road design
// (offline-design-pass-01), as designed (final-design) and as the game holds it
// (native-observed). All three are 4096 x 4096 corner grids over the map width, about
// 14 m apart; heights in metres. Prints one JSON line with what
// our design and the game changed, and where the finished terrain looks broken.
static partial class VisualMetricsReplay
{
    private static float[] ReadHeights(string folder, string name, float scale)
        => CapturedRunFiles.Heights(CapturedRunFiles.Resolve(folder, name), scale);

    public static void TerrainAdjustments(string folder, string name)
    {
        var capture = Load(folder);
        var before = capture.Terrain;
        var designed = ReadHeights(folder, "final-design-heightmap.u16", capture.Scale);
        var observed = ReadHeights(folder, "native-observed-heightmap.u16", capture.Scale);
        var size = (int)Math.Sqrt(before.Length);
        // One step of the game's 16-bit height encoding, the grids' precision.
        var encodingStep = capture.Scale / 65535f;
        var spacing = capture.Width / (size - 1);
        var cellArea = spacing * spacing / 1e6;
        var latitude = (double)capture.Input["request"]!["Latitude"]!;
        var longitude = (double)capture.Input["request"]!["Longitude"]!;

        double X(int index) => index * spacing - capture.Width / 2;

        string Place(int x, int z) =>
            $"{latitude + math.degrees(X(z) / Earth):F4},{longitude + math.degrees(X(x) / (Earth * Math.Cos(math.radians(latitude)))):F4}";

        float H(float[] grid, int x, int z) => grid[z * size + x];

        // Registration: the game's grid against ours, shifted by up to one cell.
        var registration = new JObject();

        foreach (var (dx, dz) in new[] { (0, 0), (1, 0), (-1, 0), (0, 1), (0, -1) })
        {
            double sum = 0;
            var count = 0;

            for (var z = 64; z < size - 64; z += 16)
                for (var x = 64; x < size - 64; x += 16)
                {
                    sum += Math.Abs(H(observed, x + dx, z + dz) - H(designed, x, z));
                    count++;
                }

            registration[$"{dx},{dz}"] = Math.Round(sum / count, 3);
        }

        // The game reports its own grid width (native-observed-terrain.json); read the
        // observed grid at that scale instead, bilinearly, and see which fits ours.
        var observedWidth =
            (double?)JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder, "native-observed-terrain.json")))["width"]
            ?? capture.Width;

        float ObservedAt(double worldX, double worldZ)
        {
            var u = (worldX / observedWidth + .5) * (size - 1);
            var v = (worldZ / observedWidth + .5) * (size - 1);
            var x0 = Math.Clamp((int)Math.Floor(u), 0, size - 2);
            var z0 = Math.Clamp((int)Math.Floor(v), 0, size - 2);
            var fx = (float)Math.Clamp(u - x0, 0, 1);
            var fz = (float)Math.Clamp(v - z0, 0, 1);

            return math.lerp(
                math.lerp(H(observed, x0, z0), H(observed, x0 + 1, z0), fx),
                math.lerp(H(observed, x0, z0 + 1), H(observed, x0 + 1, z0 + 1), fx),
                fz);
        }

        {
            double sum = 0;
            var count = 0;

            for (var z = 64; z < size - 64; z += 16)
                for (var x = 64; x < size - 64; x += 16)
                {
                    sum += Math.Abs(ObservedAt(X(x), X(z)) - H(designed, x, z));
                    count++;
                }

            registration[$"gameWidth {observedWidth:F1}"] = Math.Round(sum / count, 3);
        }

        // Steps on the game's grid exactly as it holds it, before any resampling of ours.
        var gameGridSteps = PlanMeasures.Steps(observed, size).Count;

        // Resample the observed grid onto ours when the game's own width fits better.
        if ((double)registration[$"gameWidth {observedWidth:F1}"]! < (double)registration["0,0"]!)
        {
            var resampled = new float[observed.Length];

            for (var z = 0; z < size; z++)
                for (var x = 0; x < size; x++)
                    resampled[z * size + x] = ObservedAt(X(x), X(z));

            observed = resampled;
            registration["used"] = "game width";
        }

        // Control for the game's changes: our designed grid taken onto the game's grid and
        // back with no other change. Whatever differs by over 2 m here is resampling alone.
        var resamplingOnlyOver2 = 0;

        {
            float DesignedAt(double worldX, double worldZ)
            {
                var u = (worldX / capture.Width + .5) * (size - 1);
                var v = (worldZ / capture.Width + .5) * (size - 1);
                var x0 = Math.Clamp((int)Math.Floor(u), 0, size - 2);
                var z0 = Math.Clamp((int)Math.Floor(v), 0, size - 2);
                var fx = (float)Math.Clamp(u - x0, 0, 1);
                var fz = (float)Math.Clamp(v - z0, 0, 1);

                return math.lerp(
                    math.lerp(H(designed, x0, z0), H(designed, x0 + 1, z0), fx),
                    math.lerp(H(designed, x0, z0 + 1), H(designed, x0 + 1, z0 + 1), fx),
                    fz);
            }

            var gameSpacing = observedWidth / (size - 1);
            var onGameGrid = new float[size * size];

            for (var z = 0; z < size; z++)
                for (var x = 0; x < size; x++)
                    onGameGrid[z * size + x] = DesignedAt(x * gameSpacing - observedWidth / 2, z * gameSpacing - observedWidth / 2);

            for (var z = 1; z < size - 1; z++)
                for (var x = 1; x < size - 1; x++)
                {
                    var u = (X(x) / observedWidth + .5) * (size - 1);
                    var v = (X(z) / observedWidth + .5) * (size - 1);
                    var x0 = Math.Clamp((int)Math.Floor(u), 0, size - 2);
                    var z0 = Math.Clamp((int)Math.Floor(v), 0, size - 2);
                    var fx = (float)Math.Clamp(u - x0, 0, 1);
                    var fz = (float)Math.Clamp(v - z0, 0, 1);
                    var back = math.lerp(
                        math.lerp(onGameGrid[z0 * size + x0], onGameGrid[z0 * size + x0 + 1], fx),
                        math.lerp(onGameGrid[(z0 + 1) * size + x0], onGameGrid[(z0 + 1) * size + x0 + 1], fx),
                        fz);

                    if (Math.Abs(back - H(designed, x, z)) > 2)
                        resamplingOnlyOver2++;
                }
        }

        // TERRAIN_PROBE="lat,lon": the three heightmaps on a 7 x 7 window around one place.
        if (Environment.GetEnvironmentVariable("TERRAIN_PROBE") is { } probe)
        {
            var parts = probe.Split(',').Select(double.Parse).ToArray();
            var pz = math.radians(parts[0] - latitude) * Earth;
            var px = math.radians(parts[1] - longitude) * Earth * Math.Cos(math.radians(latitude));
            var cx = (int)Math.Round((px + capture.Width / 2) / spacing);
            var cz = (int)Math.Round((pz + capture.Width / 2) / spacing);

            foreach (var (label, grid) in new[] { ("before", before), ("designed", designed), ("observed", observed) })
            {
                Console.WriteLine(label);

                for (var z = cz + 3; z >= cz - 3; z--)
                    Console.WriteLine("  " + string.Join(" ", Enumerable.Range(cx - 3, 7).Select(x => $"{H(grid, x, z),7:F1}")));
            }

            return;
        }

        // TERRAIN_FLATMAP=1: a 64 x 64 map of where the before grid sits flat at its lowest
        // common level (sea level): '#' mostly at that level, '+' partly, '.' land. North up.
        if (Environment.GetEnvironmentVariable("TERRAIN_FLATMAP") == "1")
        {
            var level =
                Environment.GetEnvironmentVariable("TERRAIN_FLATLEVEL") is { } text
                    ? float.Parse(text)
                    : before.GroupBy(value => value).OrderByDescending(group => group.Count()).First().Key;
            Console.WriteLine($"most common height {level:F2} m, {before.Count(value => value == level) * cellArea:F0} km2");
            var block = size / 64;

            for (var row = 63; row >= 0; row--)
            {
                var line = new char[64];

                for (var column = 0; column < 64; column++)
                {
                    var flat = 0;

                    for (var z = row * block; z < (row + 1) * block; z += 4)
                        for (var x = column * block; x < (column + 1) * block; x += 4)
                            if (Math.Abs(before[z * size + x] - level) <= encodingStep)
                                flat++;

                    var share = flat / (double)(block * block / 16);
                    line[column] = share > .8 ? '#' : share > .2 ? '+' : '.';
                }

                Console.WriteLine("  " + new string(line));
            }

            return;
        }

        // Built roads as a mask: cells within half the road's width plus one cell of a live edge.
        var nearRoad = new bool[size * size];
        var groundRoads = new List<float3[]>();
        var roads = new List<(float3[] Points, string Composition)>();

        foreach (var row in CapturedRunFiles
            .Names(folder, "native-owned-final-", ".ndjson.gz")
            .Where(text => System.Text.RegularExpressions.Regex.IsMatch(text, @"^native-owned-final-\d+\.ndjson\.gz$"))
            .SelectMany(text => CapturedRunFiles.Records(CapturedRunFiles.Resolve(folder, text)))
            .Where(row => row["a"] != null))
        {
            var points = SampleCubic(ReadPoint(row["a"]!), ReadPoint(row["b"]!), ReadPoint(row["c"]!), ReadPoint(row["d"]!), 17);
            var radius = ((float?)row["width"] ?? 12) / 2 + (float)spacing;
            var reach = (int)Math.Ceiling(radius / spacing);

            if ((string?)row["kind"] == "Road" && Composition(row) == "ground")
                groundRoads.Add(points);

            if ((string?)row["kind"] == "Road")
                roads.Add((points, Composition(row)));

            for (var i = 0; i < points.Length; i++)
            {
                var cx = (int)Math.Round((points[i].x + capture.Width / 2) / spacing);
                var cz = (int)Math.Round((points[i].z + capture.Width / 2) / spacing);

                for (var z = Math.Max(0, cz - reach); z <= Math.Min(size - 1, cz + reach); z++)
                    for (var x = Math.Max(0, cx - reach); x <= Math.Min(size - 1, cx + reach); x++)
                        nearRoad[z * size + x] = true;
            }
        }

        // Our design's earthworks and the game's own changes.
        var designRaised = 0;
        var designLowered = 0;
        var designOver5 = 0;
        var designNearRoad = 0;
        double fill = 0;
        double cut = 0;
        var maximumFill = 0f;
        var maximumCut = 0f;
        var gameChanged = 0;
        var gameChangedOver2 = 0;
        var gameOver2NearRoad = 0;
        var gameChangedNearRoad = 0;
        var gameRaised = 0;
        var gameLowered = 0;

        for (var i = 0; i < before.Length; i++)
        {
            var design = designed[i] - before[i];
            var game = observed[i] - designed[i];

            if (design > .5f)
            {
                designRaised++;
                fill += design * cellArea * 1e6;
                maximumFill = Math.Max(maximumFill, design);
            }
            else if (design < -.5f)
            {
                designLowered++;
                cut -= design * cellArea * 1e6;
                maximumCut = Math.Max(maximumCut, -design);
            }

            if (Math.Abs(design) > 5)
                designOver5++;

            if (Math.Abs(design) > .5f && nearRoad[i])
                designNearRoad++;

            if (Math.Abs(game) > .5f)
            {
                gameChanged++;

                if (nearRoad[i])
                    gameChangedNearRoad++;

                if (game > 0)
                    gameRaised++;
                else
                    gameLowered++;
            }

            if (Math.Abs(game) > 2)
            {
                gameChangedOver2++;

                if (nearRoad[i])
                    gameOver2NearRoad++;
            }
        }

        // Defects in the finished terrain, each tagged with where it came from.
        var spikes = new List<(int X, int Z, float Size, string Origin)>();
        var cliffs = new List<(int X, int Z, float Rise, string Origin)>();
        var steps = new List<(int X, int Z, float Rise, string Origin)>();
        var neighbours = new float[8];

        float WindowRise(float[] grid, int x, int z) => PlanMeasures.WindowRise(grid, size, x, z);

        // TERRAIN_ORIGIN_SHARE: how much of the observed rise an earlier grid must already
        // have to take the blame (default 0.75), to test how sensitive origins are to it.
        var originShare =
            Environment.GetEnvironmentVariable("TERRAIN_ORIGIN_SHARE") is { } shareText ? float.Parse(shareText) : .75f;

        string Origin(int x, int z, Func<float[], bool> defect)
        {
            var observedRise = WindowRise(observed, x, z);

            if (defect(before) || WindowRise(before, x, z) >= observedRise * originShare)
                return "source";

            if (defect(designed) || WindowRise(designed, x, z) >= observedRise * originShare)
                return nearRoad[z * size + x] ? "design-road" : "design-other";

            return "game";
        }

        for (var z = 2; z < size - 2; z++)
            for (var x = 2; x < size - 2; x++)
            {
                var here = H(observed, x, z);
                var cx = x;
                var cz = z;

                if (PlanMeasures.IsSpike(observed, size, x, z))
                {
                    var k = 0;

                    for (var dz = -1; dz <= 1; dz++)
                        for (var dx = -1; dx <= 1; dx++)
                            if (dx != 0 || dz != 0)
                                neighbours[k++] = H(observed, x + dx, z + dz);

                    Array.Sort(neighbours);
                    var median = (neighbours[3] + neighbours[4]) / 2;
                    spikes.Add((x, z, here - median, Origin(x, z, grid => PlanMeasures.IsSpike(grid, size, cx, cz))));
                }

                // Cliff: a rise of more than one cell width (over 100%) to the next cell.
                var rise = Math.Max(Math.Abs(H(observed, x + 1, z) - here), Math.Abs(H(observed, x, z + 1) - here));

                if (rise > spacing)
                    cliffs.Add(
                        (x, z, rise, Origin(x, z, grid =>
                            Math.Max(Math.Abs(H(grid, cx + 1, cz) - H(grid, cx, cz)), Math.Abs(H(grid, cx, cz + 1) - H(grid, cx, cz))) > spacing * .5)));

                foreach (var (dx, dz) in new[] { (1, 0), (0, 1) })
                    if (PlanMeasures.IsStep(observed, size, x, z, dx, dz))
                    {
                        var sx = dx;
                        var sz = dz;
                        steps.Add(
                            (x, z, Math.Abs(H(observed, x + dx, z + dz) - here), Origin(x, z, grid =>
                                PlanMeasures.IsStep(grid, size, cx, cz, sx, sz))));
                    }
            }

        // Ground roads against the built terrain at their centre line, every sixteenth of an
        // edge: floating means the deck stands over 1.5 m above the ground, buried below it.
        var floating = new List<(int X, int Z, float Size, string Origin)>();
        var buried = new List<(int X, int Z, float Size, string Origin)>();
        var roadSamples = 0;

        foreach (var points in groundRoads)
            foreach (var point in points.Skip(1).SkipLast(1))
            {
                var x = (int)Math.Round((point.x + capture.Width / 2) / spacing);
                var z = (int)Math.Round((point.z + capture.Width / 2) / spacing);

                if (x < 1 || z < 1 || x >= size - 1 || z >= size - 1)
                    continue;

                roadSamples++;
                var ground = NetworkProcessor.Sample(point.x, point.z, observed, capture.Width);
                var designedGround = NetworkProcessor.Sample(point.x, point.z, designed, capture.Width);
                var gap = point.y - ground;
                var origin = Math.Abs(point.y - designedGround) > 1.5f ? "design" : "game";

                if (gap > 1.5f)
                    floating.Add((x, z, gap, origin));
                else if (gap < -1.5f)
                    buried.Add((x, z, gap, origin));
            }

        // TERRAIN_DEPTH=1: how walls, steps and floating roads grow with depth, to choose the
        // depth past which a road should become a bridge or tunnel instead of an earthwork.
        // "Needed" is the deck against the ground before road design; "earthwork" is what our
        // design actually moved (designed minus before), both at the road's centre line.
        if (Environment.GetEnvironmentVariable("TERRAIN_DEPTH") == "1")
        {
            float[] limits = { 2, 4, 6, 8, 10, 12, 15, 20, float.MaxValue };
            float[] thresholds = { 4, 6, 8, 10, 12, 15, 20 };

            int Bin(float depth)
            {
                var i = 0;

                while (depth >= limits[i])
                    i++;

                return i;
            }

            var edgeCount = new int[limits.Length];
            var edgeKm = new double[limits.Length];
            var walledCount = new int[limits.Length];
            var elevatedCount = new int[limits.Length];
            var tunnelCount = new int[limits.Length];
            var sampleCount = new int[limits.Length];
            var floatingCount = new int[limits.Length];
            var structureKm = new double[thresholds.Length];
            var structureRuns = new int[thresholds.Length];
            var cappedKm = 0.0;
            var groundKm = 0.0;

            foreach (var (points, composition) in roads)
            {
                if (composition is "tunnel")
                    continue;

                var deepest = 0f;
                var crossed = new bool[thresholds.Length];
                var onGround = composition is "ground" or "raised" or "lowered";

                for (var i = 0; i < points.Length; i++)
                {
                    var point = points[i];

                    if (Math.Abs(point.x) > capture.Width / 2 - spacing || Math.Abs(point.z) > capture.Width / 2 - spacing)
                        continue;

                    var original = NetworkProcessor.Sample(point.x, point.z, before, capture.Width);
                    var designedGround = NetworkProcessor.Sample(point.x, point.z, designed, capture.Width);
                    var needed = Math.Abs(point.y - original);
                    deepest = Math.Max(deepest, needed);

                    if (onGround && i > 0)
                    {
                        var segment = math.distance(points[i - 1].xz, point.xz) / 1000;
                        groundKm += segment;

                        // Deck still more than 2 m off our own ground: the earthwork stopped short.
                        if (Math.Abs(point.y - designedGround) > 2)
                            cappedKm += segment;

                        for (var t = 0; t < thresholds.Length; t++)
                            if (needed > thresholds[t])
                            {
                                structureKm[t] += segment;
                                crossed[t] = true;
                            }
                    }

                    if (composition == "ground" && i > 0 && i < points.Length - 1)
                    {
                        var bin = Bin(needed);
                        sampleCount[bin]++;

                        if (point.y - NetworkProcessor.Sample(point.x, point.z, observed, capture.Width) > 1.5f)
                            floatingCount[bin]++;
                    }
                }

                var edgeBin = Bin(deepest);
                edgeCount[edgeBin]++;
                edgeKm[edgeBin] += PolylineLength(points) / 1000;

                if (composition is "raised" or "lowered")
                    walledCount[edgeBin]++;
                else if (composition == "elevated")
                    elevatedCount[edgeBin]++;

                for (var t = 0; t < thresholds.Length; t++)
                    if (crossed[t])
                        structureRuns[t]++;
            }

            // Steps from our road earthworks against the deepest earthwork within three cells,
            // per square kilometre of road earthwork at that depth.
            var earthworkCells = new int[limits.Length];
            var stepCount = new int[limits.Length];

            float DeepestNear(int x, int z)
            {
                var deepest = 0f;

                for (var wz = Math.Max(0, z - 3); wz <= Math.Min(size - 1, z + 3); wz++)
                    for (var wx = Math.Max(0, x - 3); wx <= Math.Min(size - 1, x + 3); wx++)
                        deepest = Math.Max(deepest, Math.Abs(H(designed, wx, wz) - H(before, wx, wz)));

                return deepest;
            }

            for (var i = 0; i < before.Length; i++)
            {
                var depth = Math.Abs(designed[i] - before[i]);

                if (nearRoad[i] && depth > .5f)
                    earthworkCells[Bin(depth)]++;
            }

            foreach (var step in steps.Where(item => item.Origin == "design-road"))
                stepCount[Bin(DeepestNear(step.X, step.Z))]++;

            string Label(int i) => i == limits.Length - 1 ? $"{limits[i - 1]}+" : $"{(i == 0 ? 0 : limits[i - 1])}-{limits[i]}";

            var bins = new JArray();

            for (var i = 0; i < limits.Length; i++)
                bins.Add(new JObject
                {
                    ["metres"] = Label(i),
                    ["edges"] = edgeCount[i],
                    ["km"] = Math.Round(edgeKm[i], 1),
                    ["walled"] = walledCount[i],
                    ["elevated"] = elevatedCount[i],
                    ["groundSamples"] = sampleCount[i],
                    ["floating"] = floatingCount[i],
                    ["earthworkKm2"] = Math.Round(earthworkCells[i] * cellArea, 2),
                    ["earthworkSteps"] = stepCount[i],
                });

            var sweep = new JArray();

            for (var t = 0; t < thresholds.Length; t++)
                sweep.Add(new JObject
                {
                    ["over"] = thresholds[t],
                    ["km"] = Math.Round(structureKm[t], 1),
                    ["edges"] = structureRuns[t],
                });

            Console.WriteLine(new JObject
            {
                ["map"] = name,
                ["groundKm"] = Math.Round(groundKm, 1),
                ["earthworkStoppedShortKm"] = Math.Round(cappedKm, 1),
                ["byNeededDepth"] = bins,
                ["groundKmOverNeededDepth"] = sweep,
            }.ToString(Newtonsoft.Json.Formatting.None));

            return;
        }

        JObject Summary<T>(List<(int X, int Z, float Size, string Origin)> items)
        {
            var result = new JObject { ["count"] = items.Count };

            foreach (var group in items.GroupBy(item => item.Origin))
                result[group.Key] = group.Count();

            // The worst places, one per kilometre square, largest first.
            result["worst"] =
                new JArray(
                    items
                        .GroupBy(item => (item.X * spacing / 1000 >= 0 ? (int)(item.X * spacing / 1000) : 0, (int)(item.Z * spacing / 1000)))
                        .Select(group => (Count: group.Count(), Top: group.OrderByDescending(item => Math.Abs(item.Size)).First()))
                        .OrderByDescending(item => item.Count)
                        .Take(6)
                        .Select(item => $"{Place(item.Top.X, item.Top.Z)} {item.Count} cells, up to {Math.Abs(item.Top.Size):F0} m, {item.Top.Origin}"));

            return result;
        }

        var report = new JObject
        {
            ["map"] = name,
            ["registrationMeanAbsoluteMetres"] = registration,
            ["designChangedKm2"] = Math.Round((designRaised + designLowered) * cellArea, 1),
            ["designRaisedKm2"] = Math.Round(designRaised * cellArea, 1),
            ["designLoweredKm2"] = Math.Round(designLowered * cellArea, 1),
            ["designOver5mKm2"] = Math.Round(designOver5 * cellArea, 2),
            ["designChangedNearRoadShare"] = Math.Round((double)designNearRoad / Math.Max(1, designRaised + designLowered), 3),
            ["fillMillionM3"] = Math.Round(fill / 1e6, 1),
            ["cutMillionM3"] = Math.Round(cut / 1e6, 1),
            ["maximumFillMetres"] = Math.Round(maximumFill, 1),
            ["maximumCutMetres"] = Math.Round(maximumCut, 1),
            ["gameChangedKm2"] = Math.Round(gameChanged * cellArea, 2),
            ["gameChangedOver2mKm2"] = Math.Round(gameChangedOver2 * cellArea, 2),
            ["resamplingOnlyOver2mKm2"] = Math.Round(resamplingOnlyOver2 * cellArea, 2),
            ["gameChangedNearRoadShare"] = Math.Round((double)gameChangedNearRoad / Math.Max(1, gameChanged), 3),
            ["mapNearRoadShare"] = Math.Round((double)nearRoad.Count(value => value) / nearRoad.Length, 4),
            ["gameStepsNearRoad"] = steps.Count(item => item.Origin == "game" && nearRoad[item.Z * size + item.X]),
            ["gameChangedOver2mNearRoadKm2"] = Math.Round(gameOver2NearRoad * cellArea, 2),
            ["gameRaisedCells"] = gameRaised,
            ["gameLoweredCells"] = gameLowered,
            ["spikes"] = Summary<int>(spikes),
            ["cliffs"] = Summary<int>(cliffs),
            ["steps"] = Summary<int>(steps),
            ["groundRoadSamples"] = roadSamples,
            ["gameGridSteps"] = gameGridSteps,
            // The same terrain measures on our submitted grid, unresampled, as the gate computes them.
            ["plan"] = PlanMeasures.Terrain(before, designed, capture.Width, groundRoads),
            ["roadFloating"] = Summary<int>(floating),
            ["roadBuried"] = Summary<int>(buried),
        };

        Console.WriteLine(report.ToString(Newtonsoft.Json.Formatting.None));
    }
}
