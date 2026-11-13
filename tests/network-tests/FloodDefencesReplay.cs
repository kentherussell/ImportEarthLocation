using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Newtonsoft.Json.Linq;
using Unity.Mathematics;

// Flood defences from a capture, terrain only (report 112, section 14). Captures from
// before section 14 have no defence lines or ocean mask; then the defences are built
// from the terrain and mapped water alone, and the report says so.
static partial class VisualMetricsReplay
{
    private sealed record FloodInputs(
        float SeaSurface,
        OceanMask? Ocean,
        float3[][] Lines,
        WaterPlan Water,
        GeographicPolygon[] Polygons);

    // The sea surface (the game's effective water level), the ocean mask and the mapped
    // defence lines. The sea level comes from the capture, or else from the log line the
    // import writes when it applies the terrain.
    private static FloodInputs ReadFloodInputs(Capture capture)
    {
        var sea = float.NaN;

        if (capture.Input["seaLevelMeters"] is JValue level && level.Type != JTokenType.Null)
            sea = WaterLevels.Effective((float)level);
        else
        {
            var events = Path.Combine(capture.Folder, "events.log");

            if (File.Exists(events))
            {
                var match =
                    File.ReadLines(events)
                        .Select(line => System.Text.RegularExpressions.Regex.Match(line, @"Sea level: .*effective water (-?[\d.]+) m"))
                        .FirstOrDefault(found => found.Success);

                if (match != null)
                    sea = float.Parse(match.Groups[1].Value, System.Globalization.CultureInfo.InvariantCulture);
            }
        }

        OceanMask? ocean = null;

        if ((string?)capture.Input["oceanFile"] is { } oceanFile)
        {
            var bytes = File.ReadAllBytes(CapturedRunFiles.Resolve(capture.Folder, oceanFile));
            var values = new ushort[bytes.Length / 2];
            Buffer.BlockCopy(bytes, 0, values, 0, bytes.Length);
            var resolution = (int)capture.Input["oceanResolution"]!;
            ocean = new OceanMask(resolution, capture.Width, values.Select(value => (byte)value).ToArray(), "captured");
        }

        var lines =
            (string?)capture.Input["floodDefenceLinesManifest"] is { } manifest
                ? CorrectiveCapturedReplay.ReadManifest<float3[]>(capture.Folder, manifest)
                : Array.Empty<float3[]>();

        // The lakes' and rivers' planned stages, as the import's water plan holds them.
        T[] Read<T>(string name) => CorrectiveCapturedReplay.ReadManifest<T>(capture.Folder, (string)capture.Input[name]!);

        var water =
            new WaterPlan(false, 0, 0, 0, "captured")
            {
                WaterCellSize = CapturedRunFiles.WaterCellSize(capture.Folder, capture.Input),
                AreaStages = Read<WaterAreaStage>("waterAreasManifest"),
                ReachStages = Read<WaterReachStage>("waterReachesManifest")
            };

        return new(sea, ocean, lines, water, Read<GeographicPolygon>("waterPolygonsManifest"));
    }

    // The capture's water areas. FLOOD_REBUILD_AREAS rebuilds them from the captured water
    // polygons with their holes kept (GeographicPolygon.FilledRings), for runs from before
    // the import kept them; quarries are not among those polygons and drop out.
    private static float3[][] WaterAreas(Capture capture, FloodInputs inputs)
    {
        if (Environment.GetEnvironmentVariable("FLOOD_REBUILD_AREAS") is not { Length: > 0 })
            return capture.Areas.ToArray();

        return inputs.Polygons
            .Where(
                polygon => polygon.Tags.ContainsKey("water")
                    || polygon.Tags.ContainsKey("waterway")
                    || polygon.Tags.TryGetValue("natural", out var natural) && natural == "water")
            .SelectMany(polygon => polygon.FilledRings())
            .ToArray();
    }

    // The defences alone on the capture's grid before road design: land below the sea
    // that the sea reaches, before and after, what was raised, and a check that nothing
    // but defence cells moved.
    public static void FloodDefencesReplay(string folder, string output)
    {
        var capture = Load(folder);
        var inputs = ReadFloodInputs(capture);

        if (float.IsNaN(inputs.SeaSurface))
            throw new InvalidDataException("The capture records no sea level.");

        var areas = WaterAreas(capture, inputs);
        var log = new ImportLog();
        var outcome =
            FloodDefenceData.Apply(
                capture.Terrain,
                capture.Scale,
                capture.Width,
                inputs.SeaSurface,
                inputs.Ocean,
                areas,
                inputs.Lines,
                log,
                default,
                inputs.Water,
                inputs.Polygons);

        var size = (int)Math.Sqrt(capture.Terrain.Length);
        var area = Math.Pow(capture.Width / (size - 1), 2) / 1e6;
        var movedElsewhere = 0;
        var largest = 0f;
        var below = 0;
        var raisedOverHalfMetre = 0;
        double fill = 0;

        for (var id = 0; id < capture.Terrain.Length; id++)
        {
            if (capture.Terrain[id] < inputs.SeaSurface)
                below++;

            if (outcome.Heights[id] == capture.Terrain[id])
                continue;

            var raise = outcome.Heights[id] - capture.Terrain[id];
            raisedOverHalfMetre += raise > .5f ? 1 : 0;
            fill += raise;

            if (!outcome.Defence[id])
                movedElsewhere++;

            largest = Math.Max(largest, (outcome.Heights[id] - capture.Terrain[id]));
        }

        // The game wears 2-3 m off dyke edges (Amsterdam, 9 Nov). Worn here by 3 m: land the
        // water would then stand on, beyond what it stands on behind the unworn defences.
        var size2 = (int)Math.Sqrt(capture.Terrain.Length);
        var worn = (float[])outcome.Heights.Clone();
        const float wear = 3;

        // An edge cell has a neighbour on the map that is not defence; the map border is not an edge.
        bool Open(int x, int z) => x >= 0 && z >= 0 && x < size2 && z < size2 && !outcome.Defence[z * size2 + x];

        for (var id = 0; id < worn.Length; id++)
        {
            int x = id % size2, z = id / size2;

            if (outcome.Defence[id] && (Open(x - 1, z) || Open(x + 1, z) || Open(x, z - 1) || Open(x, z + 1)))
                worn[id] = Math.Max(capture.Terrain[id], worn[id] - wear);
        }

        var inlandForWear = FloodDefenceData.InlandSurface(size2, capture.Width, inputs.Water, inputs.Polygons, default);
        var drownedIntact = FloodDefences.Drowned(outcome.Heights, inputs.SeaSurface, inlandForWear);
        var drownedWorn = FloodDefences.Drowned(worn, inputs.SeaSurface, inlandForWear);
        var extraWorn = 0;

        var wornWindows = new System.Text.StringBuilder();
        var wornShown = 0;

        for (var id = 0; id < worn.Length; id++)
        {
            if (!drownedWorn[id] || drownedIntact[id])
                continue;

            extraWorn++;
            int x = id % size2, z = id / size2;

            // A newly drowned cell right beside a defence: the defence around it, worn.
            if (wornShown >= 4 || extraWorn % 50 != 0 || x < 5 || z < 5 || x >= size2 - 5 || z >= size2 - 5
                || !(outcome.Defence[id - 1] || outcome.Defence[id + 1] || outcome.Defence[id - size2] || outcome.Defence[id + size2]))
                continue;

            wornShown++;
            wornWindows.AppendLine($"cell {x},{z}: worn metres, D defence, * drowned only when worn, ~ drowned anyway, w water");

            for (var dz = 4; dz >= -4; dz--)
            {
                for (var dx = -4; dx <= 4; dx++)
                {
                    var other = (z + dz) * size2 + x + dx;
                    var mark = outcome.Defence[other] ? "D" : drownedWorn[other] && !drownedIntact[other] ? "*" : drownedIntact[other] ? "~" : !float.IsNaN(inlandForWear[other]) ? "w" : " ";
                    wornWindows.Append($"{worn[other],6:0.0}{mark} ");
                }

                wornWindows.AppendLine();
            }
        }

        if (Environment.GetEnvironmentVariable("FLOOD_WINDOWS") is { Length: > 0 } wornFile)
            File.WriteAllText(wornFile + ".worn.txt", wornWindows.ToString());

        // The defended terrain through the terrain bridge onto the game's narrower grid
        // (57,330 m, doc 106), bilinear as before and keeping crests: land newly drowned
        // there by the sea and the lakes, against the defended terrain on our own grid.
        // Lakes and rivers move with the grid: each game cell takes the surface of our
        // nearest cell, and drowned cells are compared at that same cell. A ratio of 1
        // is the control: our own grid, which should drown nothing extra.
        var drownedBySeaIntact = FloodDefences.Drowned(outcome.Heights, inputs.SeaSurface, null);

        double DrownedOnGameGrid(float[]? crests, double ratio, bool seaOnly = false)
        {
            double Ours(int index) => (index / (double)(size2 - 1) - .5) * ratio + .5;

            int OurCell(int x, int z)
                => Math.Clamp((int)Math.Round(Ours(z) * (size2 - 1)), 0, size2 - 1) * size2
                    + Math.Clamp((int)Math.Round(Ours(x) * (size2 - 1)), 0, size2 - 1);

            var grid = new float[outcome.Heights.Length];
            var inland = new float[inlandForWear.Length];

            for (var z = 0; z < size2; z++)
                for (var x = 0; x < size2; x++)
                {
                    grid[z * size2 + x] =
                        (float)TerrainSampling.HeightKeepingCrests(outcome.Heights, crests, size2, Ours(x), Ours(z));

                    inland[z * size2 + x] = inlandForWear[OurCell(x, z)];
                }

            var drowned = FloodDefences.Drowned(grid, inputs.SeaSurface, seaOnly ? null : inland);
            var before = seaOnly ? drownedBySeaIntact : drownedIntact;
            var extra = 0;
            var entries = new System.Text.StringBuilder();
            var shown = 0;

            for (var z = 0; z < size2; z++)
                for (var x = 0; x < size2; x++)
                {
                    var id = z * size2 + x;

                    if (!drowned[id] || before[OurCell(x, z)])
                        continue;

                    extra++;

                    // Where water gets in: a newly drowned cell beside water that was there anyway.
                    if (shown >= 8 || extra % 997 != 0 || x < 5 || z < 5 || x >= size2 - 5 || z >= size2 - 5)
                        continue;

                    shown++;
                    entries.AppendLine($"game cell {x},{z} (ours {OurCell(x, z) % size2},{OurCell(x, z) / size2}): game-grid metres; * newly drowned, ~ drowned anyway, w water, C crest kept");

                    for (var dz = 4; dz >= -4; dz--)
                    {
                        for (var dx = -4; dx <= 4; dx++)
                        {
                            var other = (z + dz) * size2 + x + dx;
                            var ours = OurCell(x + dx, z + dz);
                            var mark = !float.IsNaN(inland[other]) ? "w" : drowned[other] && !drownedIntact[ours] ? "*" : drowned[other] ? "~" : crests != null && !float.IsNaN(crests[ours]) ? "C" : " ";
                            entries.Append($"{grid[other],6:0.0}{mark} ");
                        }

                        entries.AppendLine();
                    }
                }

            if (crests != null && Environment.GetEnvironmentVariable("FLOOD_WINDOWS") is { Length: > 0 } entryFile)
                File.WriteAllText(entryFile + ".entries.txt", entries.ToString());

            return Math.Round(extra * area, 2);
        }

        var gameRatio = 57330d / capture.Width;
        var barriers = FloodDefences.Barriers(outcome.Heights, inputs.SeaSurface, inlandForWear);
        var withBarriers = (float[])barriers.Clone();
        var barrierCells = 0;

        // NaN is "none" in both grids; a cell takes the higher of what it has.
        for (var id = 0; id < barriers.Length; id++)
        {
            var crest = outcome.Crest[id];

            if (float.IsNaN(crest))
                barrierCells += float.IsNaN(barriers[id]) ? 0 : 1;
            else if (float.IsNaN(withBarriers[id]) || crest > withBarriers[id])
                withBarriers[id] = crest;
        }

        var report = new JObject
        {
            ["extraDrownedOnGameGridKm2"] = new JObject
            {
                ["sameGridControl"] = DrownedOnGameGrid(null, 1),
                ["bilinear"] = DrownedOnGameGrid(null, gameRatio),
                ["keepingCrests"] = DrownedOnGameGrid(outcome.Crest, gameRatio),
                ["keepingCrestsAndBarriers"] = DrownedOnGameGrid(withBarriers, gameRatio),
                ["seaOnlyBilinear"] = DrownedOnGameGrid(null, gameRatio, true),
                ["seaOnlyKeepingCrestsAndBarriers"] = DrownedOnGameGrid(withBarriers, gameRatio, true),
            },
            ["barriersBeyondDefencesKm2"] = Math.Round(barrierCells * area, 2),
            ["extraDrownedWithEdgesWorn3mKm2"] = Math.Round(extraWorn * area, 2),
            ["gameBankProbes"] = GameFloodCheck(capture, inputs, outcome),
            ["capture"] = Path.GetFileName(folder.TrimEnd('/', '\\')),
            ["seaSurface"] = Math.Round(inputs.SeaSurface, 2),
            ["oceanMask"] = inputs.Ocean != null,
            ["mappedDefenceLines"] = inputs.Lines.Length,
            ["groundBelowSeaKm2"] = Math.Round(below * area, 2),
            ["landReachedBeforeKm2"] = Math.Round(outcome.LandReachedBefore * area, 2),
            ["landReachedAfterKm2"] = Math.Round(outcome.LandReachedAfter * area, 2),
            ["mappedDefenceKm2"] = Math.Round(outcome.MappedCells * area, 3),
            ["builtDefenceKm2"] = Math.Round(outcome.BuiltCells * area, 3),
            ["raisedOverHalfAMetreKm2"] = Math.Round(raisedOverHalfMetre * area, 2),
            ["fillMillionM3"] = Math.Round(fill * area, 1),
            ["areasDefended"] = outcome.PoldersDefended,
            ["areasTooDeep"] = outcome.AreasTooDeep,
            ["largestRaiseMetres"] = Math.Round(largest, 1),
            ["cellsMovedOutsideDefences"] = movedElsewhere,
        };

        Directory.CreateDirectory(output);
        File.WriteAllText(Path.Combine(output, "flood-defences.json"), report.ToString());
        Console.WriteLine(report.ToString(Newtonsoft.Json.Formatting.None));

        if (movedElsewhere > 0)
            throw new InvalidDataException($"{movedElsewhere} cells outside the defences moved.");
    }

    // The game's own flooding against the model: the bank probes of the run's last water
    // observation (ground beside each water body, depth of water standing on it). A probe
    // under more than 1 m of water is flooded, under less than 0.1 m dry; probes on mapped
    // water or ocean cells are left out. The ground the game ran on is the capture's, with
    // the sea-only defences when the run had flood defences on (the version before lakes
    // were included). For each probe: whether that ground was drowned by the sea alone,
    // by the sea and the lakes and rivers at their stages, and whether it still is behind
    // the defences built now. Null when the run recorded no observations.
    private static JObject? GameFloodCheck(Capture capture, FloodInputs inputs, FloodDefences.Outcome defenceOutcome)
    {
        var files = Directory.GetFiles(capture.Folder, "water-observations-*s-chunk-*.ndjson");

        if (files.Length == 0)
            return null;

        int Seconds(string file)
            => int.Parse(System.Text.RegularExpressions.Regex.Match(Path.GetFileName(file), @"observations-(\d+)s").Groups[1].Value);

        var last = files.Max(Seconds);
        var defended = defenceOutcome.Heights;
        var probes = new List<(int Id, float Depth)>();
        var size = (int)Math.Sqrt(capture.Terrain.Length);
        var switches = (string?)capture.Input["request"]?["DesignSwitches"] ?? "";
        var game = capture.Terrain;

        // Runs from before lakes were included built sea-only defences (their logs say so).
        var seaOnly = File.ReadLines(Path.Combine(capture.Folder, "events.log")).Any(line => line.Contains("Flood defences: land below the sea"));

        if (switches.Contains(DesignSwitches.FloodDefences, StringComparison.Ordinal))
            game = seaOnly
                ? FloodDefenceData.Apply(
                    capture.Terrain,
                    capture.Scale,
                    capture.Width,
                    inputs.SeaSurface,
                    inputs.Ocean,
                    WaterAreas(capture, inputs),
                    inputs.Lines,
                    new ImportLog(),
                    default).Heights
                : defended;

        var wet = TerrainExtremes.Protected(size, capture.Width, WaterAreas(capture, inputs), Array.Empty<float3[]>());

        if (inputs.Ocean != null)
            for (var id = 0; id < wet.Length; id++)
            {
                var ox = (int)Math.Round(id % size * (inputs.Ocean.Resolution - 1) / (double)(size - 1));
                var oz = (int)Math.Round(id / size * (inputs.Ocean.Resolution - 1) / (double)(size - 1));

                if (inputs.Ocean.Classification[oz * inputs.Ocean.Resolution + ox] == 1)
                    wet[id] = 1;
            }

        var inland = FloodDefenceData.InlandSurface(size, capture.Width, inputs.Water, inputs.Polygons, default);
        var bySea = FloodDefences.Drowned(game, inputs.SeaSurface, null);
        var byAll = FloodDefences.Drowned(game, inputs.SeaSurface, inland);
        var behind = FloodDefences.Drowned(defended, inputs.SeaSurface, inland);
        int flooded = 0, dry = 0, floodedBySea = 0, floodedByAll = 0, floodedBehind = 0, dryByAll = 0;

        foreach (var file in files.Where(file => Seconds(file) == last))
            foreach (var line in File.ReadLines(file))
                foreach (var bank in (JArray?)JObject.Parse(line)["banks"] ?? new JArray())
                {
                    var depth = (float)bank["depthMeters"]!;
                    var x = (int)Math.Round(((float)bank["position"]!["x"]! / capture.Width + .5) * (size - 1));
                    var z = (int)Math.Round(((float)bank["position"]!["z"]! / capture.Width + .5) * (size - 1));

                    if (x < 0 || z < 0 || x >= size || z >= size)
                        continue;

                    var id = z * size + x;

                    if (wet[id] != 0 || !float.IsNaN(inland[id]))
                        continue;

                    probes.Add((id, depth));

                    if (depth > 1)
                    {
                        flooded++;
                        floodedBySea += bySea[id] ? 1 : 0;
                        floodedByAll += byAll[id] ? 1 : 0;
                        floodedBehind += behind[id] ? 1 : 0;
                    }
                    else if (depth < .1f)
                    {
                        dry++;
                        dryByAll += byAll[id] ? 1 : 0;
                    }
                }

        // The terrain the game held: as design left it and as read back from the game. On
        // each, how many defence cells stand below their crest, and how many flooded probes
        // that ground drowns.
        var later = new JObject();
        float[]? designedHeights = null;

        foreach (var name in new[] { "final-design-heightmap.u16", "native-observed-heightmap.u16" })
        {
            var file = Path.Combine(capture.Folder, name);

            if (!File.Exists(file) || new FileInfo(file).Length != capture.Terrain.Length * 2L)
                continue;

            var heights = CapturedRunFiles.Heights(file, capture.Scale);
            var built = defenceOutcome;
            int low = 0, lowByMetre = 0, floodedThere = 0;

            for (var id = 0; id < heights.Length; id++)
                if (!float.IsNaN(built.Crest[id]) && heights[id] < built.Crest[id])
                {
                    low++;

                    if (built.Crest[id] - heights[id] > 1)
                        lowByMetre++;
                }

            // Against design's own terrain: how much lower the game holds it, on defences and off.
            if (designedHeights == null)
                designedHeights = heights;
            else
            {
                int onDefence = 0, offDefence = 0, offAll = 0;
                var deepest = 0f;

                for (var id = 0; id < heights.Length; id++)
                {
                    var drop = designedHeights[id] - heights[id];

                    if (drop <= 1)
                        continue;

                    if (!float.IsNaN(built.Crest[id]))
                    {
                        onDefence++;
                        deepest = Math.Max(deepest, drop);
                    }
                    else
                        offDefence++;
                }

                for (var id = 0; id < heights.Length; id++)
                    if (float.IsNaN(built.Crest[id]))
                        offAll++;

                // Roads: every captured section, out to its half width and two cells.
                var road = new bool[heights.Length];
                var cellSize = (float)(capture.Width / (size - 1));

                foreach (var segment in capture.Source)
                {
                    var reach = (segment.NativeWidth > 0 ? segment.NativeWidth * .5f : 6) + 2 * cellSize;
                    var steps = Math.Max(1, (int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(segment) / (cellSize * .5f)));

                    for (var i = 0; i <= steps; i++)
                    {
                        var point = NetworkNormalizer.Evaluate(segment, i / (float)steps);
                        var cells = (int)Math.Ceiling(reach / cellSize);
                        var px = (int)Math.Round((point.x / capture.Width + .5) * (size - 1));
                        var pz = (int)Math.Round((point.z / capture.Width + .5) * (size - 1));

                        for (var z = Math.Max(0, pz - cells); z <= Math.Min(size - 1, pz + cells); z++)
                            for (var x = Math.Max(0, px - cells); x <= Math.Min(size - 1, px + cells); x++)
                                road[z * size + x] = true;
                    }
                }

                int onDefenceUnderRoad = 0, atRidgeEdge = 0, ridgeEdgeCells = 0, central = 0, centralCells = 0;
                var quarter = size / 4;

                for (var id = 0; id < heights.Length; id++)
                {
                    if (float.IsNaN(built.Crest[id]))
                        continue;

                    int x = id % size, z = id / size;
                    var edge = x == 0 || z == 0 || x == size - 1 || z == size - 1
                        || float.IsNaN(built.Crest[id - 1]) || float.IsNaN(built.Crest[id + 1])
                        || float.IsNaN(built.Crest[id - size]) || float.IsNaN(built.Crest[id + size]);
                    var inCentre = Math.Abs(x - size / 2) < size * 0.125 && Math.Abs(z - size / 2) < size * 0.125;
                    var lowered = (designedHeights[id] - heights[id]) > 1;
                    ridgeEdgeCells += edge ? 1 : 0;
                    centralCells += inCentre ? 1 : 0;

                    if (!lowered)
                        continue;

                    onDefenceUnderRoad += road[id] ? 1 : 0;
                    atRidgeEdge += edge ? 1 : 0;
                    central += inCentre ? 1 : 0;
                }

                // Steep steps elsewhere: cells off defences and roads standing over 5 m above a
                // neighbour in design's terrain, and how many of those the game holds lower.
                int steep = 0, steepLowered = 0;
        
                for (var z = 1; z < size - 1; z++)
                    for (var x = 1; x < size - 1; x++)
                    {
                        var id = z * size + x;

                        if (!float.IsNaN(built.Crest[id]) || road[id])
                            continue;

                        var lowest = Math.Min(
                            Math.Min(designedHeights[id - 1], designedHeights[id + 1]),
                            Math.Min(designedHeights[id - size], designedHeights[id + size]));

                        if (designedHeights[id] - lowest <= 5)
                            continue;

                        steep++;

                        if (designedHeights[id] - heights[id] > 1)
                            steepLowered++;
                    }

                later["steepStepsElsewhere"] = new JObject { ["cells"] = steep, ["loweredOverAMetre"] = steepLowered };

                // Every cell off roads, by its step above its lowest neighbour (whole metres,
                // 0 to 12): how many, and how many the game holds over half a metre lower.
                var stepCells = new int[13];
                var stepLowered = new int[13];

                for (var z = 1; z < size - 1; z++)
                    for (var x = 1; x < size - 1; x++)
                    {
                        var id = z * size + x;

                        if (road[id])
                            continue;

                        var lowest = Math.Min(
                            Math.Min(designedHeights[id - 1], designedHeights[id + 1]),
                            Math.Min(designedHeights[id - size], designedHeights[id + size]));
                        var bin = Math.Min(12, Math.Max(0, (int)((designedHeights[id] - lowest))));
                        stepCells[bin]++;

                        if (designedHeights[id] - heights[id] > .5f)
                            stepLowered[bin]++;
                    }

                // Which smoothing reproduces the game's terrain from design's: mean absolute error
                // in metres over cells off roads with a step of 2 m or more.
                var kernels = new (string Name, int Radius, Func<int, int, float> Weight)[]
                {
                    ("none", 0, (dx, dz) => 1),
                    ("box3", 1, (dx, dz) => 1),
                    ("tent3", 1, (dx, dz) => (2 - Math.Abs(dx)) * (2 - Math.Abs(dz))),
                    ("cross5", 1, (dx, dz) => dx == 0 && dz == 0 ? 4 : dx == 0 || dz == 0 ? 1 : 0),
                    ("box5", 2, (dx, dz) => 1),
                    ("tent5", 2, (dx, dz) => (3 - Math.Abs(dx)) * (3 - Math.Abs(dz))),
                };
                var fits = new JObject();

                foreach (var (kernel, radius, weight) in kernels)
                {
                    double error = 0;
                    var count = 0;

                    for (var z = 2; z < size - 2; z += 1)
                        for (var x = 2; x < size - 2; x += 1)
                        {
                            var id = z * size + x;

                            if (road[id])
                                continue;

                            var lowest = Math.Min(
                                Math.Min(designedHeights[id - 1], designedHeights[id + 1]),
                                Math.Min(designedHeights[id - size], designedHeights[id + size]));

                            if (designedHeights[id] - lowest < 2)
                                continue;

                            double sum = 0, total = 0;

                            for (var dz = -radius; dz <= radius; dz++)
                                for (var dx = -radius; dx <= radius; dx++)
                                {
                                    var w = weight(dx, dz);
                                    sum += w * designedHeights[id + dz * size + dx];
                                    total += w;
                                }

                            error += Math.Abs(sum / total - heights[id]);
                            count++;
                        }

                    fits[kernel] = Math.Round(error / Math.Max(1, count), 3);
                }

                later["smoothingFit"] = fits;

                // A few lowered defence cells off roads: design's and the game's metres in the
                // 7 x 7 window around each, written beside the report.
                var windows = new System.Text.StringBuilder();
                var shown = 0;

                for (var id = size * 600; id < heights.Length - size * 600 && shown < 6; id += 997)
                {
                    int x = id % size, z = id / size;

                    if (x < 4 || x >= size - 4 || float.IsNaN(built.Crest[id]) || road[id] || designedHeights[id] - heights[id] < 2)
                        continue;

                    shown++;
                    windows.AppendLine($"cell {x},{z} crest {built.Crest[id]:0.0} m (design / game / D=defence)");

                    for (var dz = 3; dz >= -3; dz--)
                    {
                        for (var dx = -3; dx <= 3; dx++)
                        {
                            var other = (z + dz) * size + x + dx;
                            windows.Append($"{designedHeights[other],6:0.0}/{heights[other],6:0.0}{(!float.IsNaN(built.Crest[other]) ? "D" : " ")} ");
                        }

                        windows.AppendLine();
                    }
                }

                if (Environment.GetEnvironmentVariable("FLOOD_WINDOWS") is { Length: > 0 } windowsFile)
                    File.WriteAllText(windowsFile, windows.ToString());
                later["byStepMetres"] =
                    new JArray(Enumerable.Range(0, 13).Select(bin => $"{bin}: {stepLowered[bin]}/{stepCells[bin]}"));
                later["defenceRidgeEdges"] = new JObject { ["cells"] = ridgeEdgeCells, ["loweredOverAMetre"] = atRidgeEdge };
                later["defenceInCentralQuarter"] = new JObject { ["cells"] = centralCells, ["loweredOverAMetre"] = central };

                later["gameLowerThanDesignByOverAMetre"] = new JObject
                {
                    ["defenceCellsUnderRoads"] = onDefenceUnderRoad,
                    ["defenceCells"] = onDefence,
                    ["otherCells"] = offDefence,
                    ["otherCellsTotal"] = offAll,
                    ["deepestOnDefenceMetres"] = Math.Round(deepest, 1),
                };
            }

            var drowned = FloodDefences.Drowned(heights, inputs.SeaSurface, inland);

            foreach (var (id, depth) in probes)
                if (depth > 1 && drowned[id])
                    floodedThere++;

            // Flooded probes whose water stands within a metre of the sea surface, and how
            // many of those this terrain lets the sea reach from the map edge. Sea water
            // where the sea cannot reach means the game filled the ground some other way.
            var seaReaches = FloodDefences.Drowned(heights, inputs.SeaSurface, null);
            int atSeaLevel = 0, atSeaLevelReached = 0;

            foreach (var (id, depth) in probes)
            {
                var surface = heights[id] + depth;

                if (depth <= 1 || Math.Abs(surface - inputs.SeaSurface) > 1)
                    continue;

                atSeaLevel++;
                atSeaLevelReached += seaReaches[id] ? 1 : 0;
            }

            later[name] = new JObject
            {
                ["floodedAtSeaLevel"] = atSeaLevel,
                ["floodedAtSeaLevelTheSeaReaches"] = atSeaLevelReached,
                ["defenceCellsBelowCrest"] = low,
                ["defenceCellsOverAMetreBelow"] = lowByMetre,
                ["floodedProbesItDrowns"] = floodedThere,
            };
        }

        // The terrain bridge replayed: design's terrain sampled at the game's narrower grid,
        // bilinear as before and keeping defence crests, against what the game held. For
        // each, the flooded probes at sea level that the sea reaches on that grid.
        var bridge = new JObject();
        var observedFile = Path.Combine(capture.Folder, "native-observed-heightmap.u16");
        var terrainFile = Path.Combine(capture.Folder, "native-observed-terrain.json");

        if (designedHeights != null && File.Exists(observedFile) && File.Exists(terrainFile))
        {
            var gameWidth = (double?)JObject.Parse(File.ReadAllText(terrainFile))["width"] ?? capture.Width;
            var observed = CapturedRunFiles.Heights(observedFile, capture.Scale);

            // A probe's cell on the game's grid, from its world position.
            int GameCell(int id)
            {
                var x = (id % size / (double)(size - 1) - .5) * capture.Width;
                var z = (id / size / (double)(size - 1) - .5) * capture.Width;
                var gx = Math.Clamp((int)Math.Round((x / gameWidth + .5) * (size - 1)), 0, size - 1);
                var gz = Math.Clamp((int)Math.Round((z / gameWidth + .5) * (size - 1)), 0, size - 1);

                return gz * size + gx;
            }

            JObject Score(float[] grid)
            {
                var reaches = FloodDefences.Drowned(grid, inputs.SeaSurface, null);
                int atSea = 0, reached = 0;

                foreach (var (id, depth) in probes)
                {
                    var cell = GameCell(id);

                    if (depth <= 1 || Math.Abs(observed[cell] + depth - inputs.SeaSurface) > 1)
                        continue;

                    atSea++;
                    reached += reaches[cell] ? 1 : 0;
                }

                double error = 0;

                for (var id = 0; id < grid.Length; id++)
                    error += Math.Abs(grid[id] - observed[id]);

                return new JObject
                {
                    ["meanDifferenceFromGameMetres"] = Math.Round(error / grid.Length, 3),
                    ["floodedAtSeaLevel"] = atSea,
                    ["seaReaches"] = reached,
                };
            }

            float[] Sample(float[]? crests)
            {
                var grid = new float[designedHeights.Length];
                var ratio = gameWidth / capture.Width;

                for (var z = 0; z < size; z++)
                    for (var x = 0; x < size; x++)
                        grid[z * size + x] =
                            (float)TerrainSampling.HeightKeepingCrests(
                                designedHeights,
                                crests,
                                size,
                                (x / (double)(size - 1) - .5) * ratio + .5,
                                (z / (double)(size - 1) - .5) * ratio + .5);

                return grid;
            }

            bridge["gameAsObserved"] = Score(observed);
            bridge["bilinear"] = Score(Sample(null));
            bridge["keepingCrests"] = Score(Sample(defenceOutcome.Crest));
        }

        return new JObject
        {
            ["wholeMap"] = WholeMapWater(capture, inputs, wet, inland, defenceOutcome),
            ["terrainBridge"] = bridge,
            ["lakeStagesAgainstBanks"] = LakeStagesAgainstBanks(capture, inputs, files.Where(file => Seconds(file) == last)),
            ["laterTerrain"] = later,
            ["observedAtSeconds"] = last,
            ["gameRanWithSeaDefences"] = !ReferenceEquals(game, capture.Terrain),
            ["flooded"] = flooded,
            ["floodedWhereSeaAloneDrowns"] = floodedBySea,
            ["floodedWhereSeaOrLakesDrown"] = floodedByAll,
            ["floodedStillDrownedBehindDefences"] = floodedBehind,
            ["dry"] = dry,
            ["dryWhereSeaOrLakesDrown"] = dryByAll,
        };
    }

    // The game's settled water over the whole map (native-water-depth-cm.u16: centimetres on
    // the game's water grid, row zero south), read at each water cell's centre against the
    // ground the game held. Land is neither ocean nor a planned lake or river; land under more
    // than 1 m of water is flooded, and is split by whether its water stands within a metre of
    // the sea surface and whether the model drowns it behind the built defences. Planned lakes
    // and rivers report how much of them the game left dry (under 0.1 m) or over a metre short
    // of their stage. The largest flooded patches are listed with their centre in world metres.
    // Null when the run saved no water depths.
    private static JObject? WholeMapWater(
        Capture capture,
        FloodInputs inputs,
        byte[] wet,
        float[] inland,
        FloodDefences.Outcome defences)
    {
        var depthFile = Path.Combine(capture.Folder, "native-water-depth-cm.u16");
        var metaFile = Path.Combine(capture.Folder, "native-water-depth.json");

        if (!File.Exists(depthFile) || !File.Exists(metaFile))
            return null;

        var meta = JObject.Parse(File.ReadAllText(metaFile));
        var cells = (int)meta["cells"]!;
        var cellMeters = (double)meta["cellMeters"]!;
        var bytes = File.ReadAllBytes(depthFile);
        var depths = new ushort[bytes.Length / 2];
        Buffer.BlockCopy(bytes, 0, depths, 0, bytes.Length);

        var size = (int)Math.Sqrt(capture.Terrain.Length);
        var groundFile = Path.Combine(capture.Folder, "native-observed-heightmap.u16");
        var ground = File.Exists(groundFile) ? CapturedRunFiles.Heights(groundFile, capture.Scale) : capture.Terrain;
        var cellArea = cellMeters * cellMeters / 1e6;
        var flooded = new bool[depths.Length];
        int land = 0, floodedLand = 0, atSeaLevel = 0, modelDrowns = 0, belowSea = 0;

        // Where the sea reaches: on the defended ground before design, on the ground design
        // sent, and on the ground the game held.
        var designFile = Path.Combine(capture.Folder, "final-design-heightmap.u16");
        var design = File.Exists(designFile) ? CapturedRunFiles.Heights(designFile, capture.Scale) : defences.Heights;
        var seaOnDefended = FloodDefences.Drowned(defences.Heights, inputs.SeaSurface, null);
        var seaOnDesign = FloodDefences.Drowned(design, inputs.SeaSurface, null);
        var seaReaches = FloodDefences.Drowned(ground, inputs.SeaSurface, null);
        var drownedBehindDefences = FloodDefences.Drowned(defences.Heights, inputs.SeaSurface, inland);
        int reached = 0, reachedOnDesign = 0, reachedOnDefended = 0, landModelDrowns = 0;
        int planned = 0, plannedDry = 0, plannedShort = 0;

        for (var row = 0; row < cells; row++)
            for (var column = 0; column < cells; column++)
            {
                var x = (column + .5) * cellMeters - cells * cellMeters / 2;
                var z = (row + .5) * cellMeters - cells * cellMeters / 2;
                var tx = (int)Math.Round((x / capture.Width + .5) * (size - 1));
                var tz = (int)Math.Round((z / capture.Width + .5) * (size - 1));

                if (tx < 0 || tz < 0 || tx >= size || tz >= size)
                    continue;

                var id = tz * size + tx;
                var depth = depths[row * cells + column] / 100f;

                if (!float.IsNaN(inland[id]))
                {
                    planned++;
                    plannedDry += depth < .1f ? 1 : 0;
                    plannedShort += ground[id] + depth < inland[id] - 1 ? 1 : 0;
                    continue;
                }

                if (wet[id] != 0)
                    continue;

                land++;
                landModelDrowns += drownedBehindDefences[id] ? 1 : 0;

                if (depth <= 1)
                    continue;

                flooded[row * cells + column] = true;
                floodedLand++;
                atSeaLevel += Math.Abs(ground[id] + depth - inputs.SeaSurface) <= 1 ? 1 : 0;
                belowSea += ground[id] < inputs.SeaSurface ? 1 : 0;
                modelDrowns += drownedBehindDefences[id] ? 1 : 0;
                reached += seaReaches[id] ? 1 : 0;
                reachedOnDesign += seaOnDesign[id] ? 1 : 0;
                reachedOnDefended += seaOnDefended[id] ? 1 : 0;
            }

        // Breaches: defence cells the sea stands on, on design's ground and the game's, in
        // 8-connected clusters on the terrain grid, largest first, with the deepest cut below
        // the crest.
        JArray Breaches(float[] heights, bool[] sea)
        {
            var marked = new bool[heights.Length];
            var clusters = new List<(int Cells, double X, double Z, float Cut)>();

            for (var start = 0; start < heights.Length; start++)
            {
                if (marked[start] || !sea[start] || float.IsNaN(defences.Crest[start]))
                    continue;

                var members = new List<int> { start };
                marked[start] = true;
                double sumX = 0;
                double sumZ = 0;
                var cut = 0f;

                for (var next = 0; next < members.Count; next++)
                {
                    var index = members[next];
                    int x = index % size, z = index / size;
                    sumX += x;
                    sumZ += z;
                    cut = Math.Max(cut, defences.Crest[index] - heights[index]);

                    for (var dz = -1; dz <= 1; dz++)
                        for (var dx = -1; dx <= 1; dx++)
                        {
                            int nx = x + dx, nz = z + dz;

                            if (nx < 0 || nz < 0 || nx >= size || nz >= size)
                                continue;

                            var other = nz * size + nx;

                            if (marked[other] || !sea[other] || float.IsNaN(defences.Crest[other]))
                                continue;

                            marked[other] = true;
                            members.Add(other);
                        }
                }

                clusters.Add((
                    members.Count,
                    (sumX / members.Count / (size - 1) - .5) * capture.Width,
                    (sumZ / members.Count / (size - 1) - .5) * capture.Width,
                    cut));
            }

            return new JArray(
                new[] { $"{clusters.Count} breaches, {clusters.Sum(cluster => cluster.Cells)} cells" }
                    .Concat(
                        clusters
                            .OrderByDescending(cluster => cluster.Cells)
                            .Take(8)
                            .Select(cluster =>
                                $"{cluster.Cells} cells at x {cluster.X:0}, z {cluster.Z:0}, cut {cluster.Cut:0.0} m below crest")));
        }

        // Flooded land in 4-connected patches, largest first.
        var seen = new bool[flooded.Length];
        var patches = new List<(int Cells, double X, double Z, float Deepest)>();

        for (var start = 0; start < flooded.Length; start++)
        {
            if (!flooded[start] || seen[start])
                continue;

            var queue = new List<int> { start };
            seen[start] = true;
            double sumX = 0;
            double sumZ = 0;
            var deepest = 0f;

            for (var next = 0; next < queue.Count; next++)
            {
                var index = queue[next];
                var column = index % cells;
                var row = index / cells;
                sumX += column;
                sumZ += row;
                deepest = Math.Max(deepest, depths[index] / 100f);

                foreach (var neighbour in new[] { index - 1, index + 1, index - cells, index + cells })
                {
                    if (neighbour < 0 || neighbour >= flooded.Length || seen[neighbour] || !flooded[neighbour])
                        continue;

                    if (Math.Abs(neighbour % cells - column) > 1)
                        continue;

                    seen[neighbour] = true;
                    queue.Add(neighbour);
                }
            }

            patches.Add((
                queue.Count,
                (sumX / queue.Count + .5) * cellMeters - cells * cellMeters / 2,
                (sumZ / queue.Count + .5) * cellMeters - cells * cellMeters / 2,
                deepest));
        }

        if (Environment.GetEnvironmentVariable("FLOOD_MAP") is { Length: > 0 } mapFile)
            WriteFloodMap(mapFile, capture, wet, inland, defences, seaReaches, depths, cells, cellMeters);

        double Km2(int count) => Math.Round(count * cellArea, 2);

        return new JObject
        {
            ["landKm2"] = Km2(land),
            ["landTheModelDrownsKm2"] = Km2(landModelDrowns),
            ["floodedLandKm2"] = Km2(floodedLand),
            ["floodedGroundBelowSeaKm2"] = Km2(belowSea),
            ["floodedAtSeaLevelKm2"] = Km2(atSeaLevel),
            ["floodedTheModelDrownsKm2"] = Km2(modelDrowns),
            ["floodedTheSeaReachesKm2"] = new JObject
            {
                ["defendedBeforeDesign"] = Km2(reachedOnDefended),
                ["asDesignSent"] = Km2(reachedOnDesign),
                ["asTheGameHeld"] = Km2(reached),
            },
            ["breachesAsDesignSent"] = Breaches(design, seaOnDesign),
            ["breachesAsTheGameHeld"] = Breaches(ground, seaReaches),
            ["plannedWaterKm2"] = Km2(planned),
            ["plannedWaterDryKm2"] = Km2(plannedDry),
            ["plannedWaterOverAMetreShortKm2"] = Km2(plannedShort),
            ["largestFloodedPatches"] = new JArray(
                patches
                    .OrderByDescending(patch => patch.Cells)
                    .Take(8)
                    .Select(patch => $"{Km2(patch.Cells)} km² at x {patch.X:0}, z {patch.Z:0}, deepest {patch.Deepest:0.0} m")),
        };
    }

    // A picture of the whole map, north up, one pixel per game water cell: ocean dark, other
    // mapped water teal, planned water light blue, land the game flooded over a metre blue, built defences orange (red
    // where the sea reaches them on the ground the game held), other land grey.
    private static void WriteFloodMap(
        string path,
        Capture capture,
        byte[] wet,
        float[] inland,
        FloodDefences.Outcome defences,
        bool[] seaReaches,
        ushort[] depths,
        int cells,
        double cellMeters)
    {
        var size = (int)Math.Sqrt(capture.Terrain.Length);
        var rgb = new byte[cells * cells * 3];

        for (var row = 0; row < cells; row++)
            for (var column = 0; column < cells; column++)
            {
                var x = (column + .5) * cellMeters - cells * cellMeters / 2;
                var z = (row + .5) * cellMeters - cells * cellMeters / 2;
                var tx = (int)Math.Round((x / capture.Width + .5) * (size - 1));
                var tz = (int)Math.Round((z / capture.Width + .5) * (size - 1));
                (byte R, byte G, byte B) colour = (0, 0, 0);

                if (tx >= 0 && tz >= 0 && tx < size && tz < size)
                {
                    var id = tz * size + tx;
                    var depth = depths[row * cells + column] / 100f;

                    colour = defences.Defence[id] && seaReaches[id] ? ((byte)230, (byte)30, (byte)30)
                        : defences.Defence[id] ? ((byte)240, (byte)150, (byte)20)
                        : !float.IsNaN(inland[id]) ? ((byte)120, (byte)170, (byte)230)
                        : wet[id] == FloodDefences.MappedWater ? ((byte)30, (byte)130, (byte)120)
                        : wet[id] != 0 ? ((byte)40, (byte)60, (byte)100)
                        : depth > 1 ? ((byte)30, (byte)80, (byte)255)
                        : ((byte)190, (byte)190, (byte)180);
                }

                var pixel = ((cells - 1 - row) * cells + column) * 3;
                rgb[pixel] = colour.R;
                rgb[pixel + 1] = colour.G;
                rgb[pixel + 2] = colour.B;
            }

        WritePng(path, cells, cells, rgb);
    }

    // Each lake's planned stage against the ground under its own bank probes, in the
    // terrain before design and as design sent it: whether a flooded bank lies below its
    // lake's stage (the stage floods it directly) or above (the water came another way).
    // Stage minus ground is binned in metres, -3 to 3, for flooded and for dry probes.
    private static JObject LakeStagesAgainstBanks(Capture capture, FloodInputs inputs, IEnumerable<string> files)
    {
        var stages = new Dictionary<long, float>();

        foreach (var stage in inputs.Water.AreaStages)
            stages[stage.BodyId] = stage.Level;

        var size = (int)Math.Sqrt(capture.Terrain.Length);
        var designFile = Path.Combine(capture.Folder, "final-design-heightmap.u16");
        var design = capture.Terrain;

        if (File.Exists(designFile) && new FileInfo(designFile).Length == capture.Terrain.Length * 2L)
        {
            design = CapturedRunFiles.Heights(designFile, capture.Scale);
        }

        var result = new JObject();

        // The game's water surface at flooded banks (design's ground plus the depth),
        // in whole metres against the sea surface.
        var surfaces = new SortedDictionary<int, int>();

        foreach (var file in files)
            foreach (var line in File.ReadLines(file))
                foreach (var bank in (JArray?)JObject.Parse(line)["banks"] ?? new JArray())
                {
                    var depth = (float)bank["depthMeters"]!;
                    var x = (int)Math.Round(((float)bank["position"]!["x"]! / capture.Width + .5) * (size - 1));
                    var z = (int)Math.Round(((float)bank["position"]!["z"]! / capture.Width + .5) * (size - 1));

                    if (x < 0 || z < 0 || x >= size || z >= size || depth <= 1)
                        continue;

                    var surface = design[z * size + x] + depth - inputs.SeaSurface;
                    var key = (int)Math.Floor(surface);
                    surfaces[key] = surfaces.GetValueOrDefault(key) + 1;
                }

        result["floodedSurfaceMinusSea"] = new JArray(surfaces.Select(entry => $"{entry.Key}: {entry.Value}"));

        foreach (var (name, ground) in new[] { ("beforeDesign", capture.Terrain), ("asDesigned", design) })
        {
            int floodedBelowStage = 0, floodedAboveStage = 0, dryBelowStage = 0, dryAboveStage = 0, unmatched = 0;
            var floodedBins = new int[7];
            var dryBins = new int[7];

            foreach (var file in files)
                foreach (var line in File.ReadLines(file))
                {
                    var body = JObject.Parse(line);

                    if ((string?)body["kind"] != "lake")
                        continue;

                    if (!long.TryParse((string?)body["body"], out var id) || !stages.TryGetValue(id, out var level))
                    {
                        unmatched++;
                        continue;
                    }

                    foreach (var bank in (JArray?)body["banks"] ?? new JArray())
                    {
                        var depth = (float)bank["depthMeters"]!;
                        var x = (int)Math.Round(((float)bank["position"]!["x"]! / capture.Width + .5) * (size - 1));
                        var z = (int)Math.Round(((float)bank["position"]!["z"]! / capture.Width + .5) * (size - 1));

                        if (x < 0 || z < 0 || x >= size || z >= size || depth is >= .1f and <= 1)
                            continue;

                        var above = level - ground[z * size + x];
                        var bin = Math.Clamp((int)Math.Floor(above) + 3, 0, 6);

                        if (depth > 1)
                        {
                            floodedBins[bin]++;
                            floodedBelowStage += above > 0 ? 1 : 0;
                            floodedAboveStage += above > 0 ? 0 : 1;
                        }
                        else
                        {
                            dryBins[bin]++;
                            dryBelowStage += above > 0 ? 1 : 0;
                            dryAboveStage += above > 0 ? 0 : 1;
                        }
                    }
                }

            result[name] = new JObject
            {
                ["floodedBankBelowStage"] = floodedBelowStage,
                ["floodedBankAboveStage"] = floodedAboveStage,
                ["dryBankBelowStage"] = dryBelowStage,
                ["dryBankAboveStage"] = dryAboveStage,
                ["unmatchedBodies"] = unmatched,
                ["floodedStageMinusGround"] = new JArray(Enumerable.Range(0, 7).Select(bin => $"{bin - 3}: {floodedBins[bin]}")),
                ["dryStageMinusGround"] = new JArray(Enumerable.Range(0, 7).Select(bin => $"{bin - 3}: {dryBins[bin]}")),
            };
        }

        return result;
    }
}
