using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Newtonsoft.Json.Linq;
using Unity.Mathematics;

// Measurements shared by the water replay and the river connectivity calibration
// (report 117). Everything here reads plans and terrain; nothing is planned or changed.
static class WaterReplayMetrics
{
    // The first node of a parsed OSM cache, read from its header alone: a 32-byte digest,
    // the version, the skipped-feature count and the node count, then the first node's id,
    // latitude and longitude (OsmParsedCache.Write).
    public static (double Latitude, double Longitude)? FirstNode(string file)
    {
        using var stream = File.OpenRead(file);

        if (stream.Length < 32 + 12 + 24)
            return null;

        using var reader = new BinaryReader(stream);
        stream.Position = 32 + 12;
        reader.ReadInt64();

        return (reader.ReadDouble(), reader.ReadDouble());
    }

    // Terrain grid points inside any of the polygons, by scanline. A point is inside a
    // polygon when it is inside one of its outer rings and none of its inner rings, by
    // the same crossing rule as GeographicPolygon.Contains, so the two agree point for
    // point without a polygon query per point.
    public static bool[] Inside(IReadOnlyList<GeographicPolygon> polygons, int resolution, float spacing, float half)
    {
        var inside = new bool[resolution * resolution];
        var scratch = new byte[resolution * resolution];
        var crossings = new List<float>[resolution];

        for (var row = 0; row < resolution; row++)
            crossings[row] = new List<float>();

        int Row(float z) => (int)Math.Ceiling((z + half) / spacing);

        void Fill(float3[] ring, byte value)
        {
            var first = resolution;
            var last = -1;

            for (var i = 0; i < ring.Length; i++)
            {
                var vectorA = ring[i];
                var vectorB = ring[(i + 1) % ring.Length];
                var low = Math.Min(vectorA.z, vectorB.z);
                var high = Math.Max(vectorA.z, vectorB.z);

                for (var row = Math.Max(0, Row(low) - 1); row < resolution; row++)
                {
                    var z = row * spacing - half;

                    if (z < low)
                        continue;

                    if (z >= high)
                        break;

                    crossings[row].Add((vectorB.x - vectorA.x) * (z - vectorA.z) / (vectorB.z - vectorA.z) + vectorA.x);
                    first = Math.Min(first, row);
                    last = Math.Max(last, row);
                }
            }

            for (var row = first; row <= last; row++)
            {
                var list = crossings[row];
                list.Sort();

                for (var k = 0; k + 1 < list.Count; k += 2)
                    for (var column = Math.Max(0, Row(list[k]) - 1); column < resolution; column++)
                    {
                        var x = column * spacing - half;

                        if (x < list[k])
                            continue;

                        if (x >= list[k + 1])
                            break;

                        scratch[row * resolution + column] = value;
                    }

                list.Clear();
            }
        }

        foreach (var polygon in polygons)
        {
            var minimum = new float2(float.MaxValue);
            var maximum = new float2(float.MinValue);

            foreach (var ring in polygon.Outer)
                foreach (var point in ring)
                {
                    minimum = math.min(minimum, point.xz);
                    maximum = math.max(maximum, point.xz);
                }

            foreach (var ring in polygon.Outer)
                Fill(ring, 1);

            foreach (var ring in polygon.Inner)
                Fill(ring, 0);

            var row0 = Math.Clamp(Row(minimum.y) - 1, 0, resolution - 1);
            var row1 = Math.Clamp(Row(maximum.y), 0, resolution - 1);
            var column0 = Math.Clamp(Row(minimum.x) - 1, 0, resolution - 1);
            var column1 = Math.Clamp(Row(maximum.x), 0, resolution - 1);

            for (var row = row0; row <= row1; row++)
                for (var column = column0; column <= column1; column++)
                {
                    var index = row * resolution + column;

                    if (scratch[index] == 0)
                        continue;

                    inside[index] = true;
                    scratch[index] = 0;
                }
        }

        return inside;
    }

    // The water level of the nearest waterway reach within reachMetres of each terrain
    // grid point (NaN beyond that): the band of ground a waterway's cut can touch.
    public static float[] NearestLevel(
        IReadOnlyList<WaterReachStage> reaches,
        int resolution,
        float spacing,
        float half,
        float reachMetres,
        out float[] nearest)
    {
        var level = new float[resolution * resolution];
        nearest = new float[resolution * resolution];
        Array.Fill(level, float.NaN);
        Array.Fill(nearest, float.PositiveInfinity);

        int Index(float v) => (int)Math.Round((v + half) / spacing);

        foreach (var reach in reaches)
        {
            var x0 = Math.Max(0, Index(Math.Min(reach.Start.x, reach.End.x) - reachMetres));
            var x1 = Math.Min(resolution - 1, Index(Math.Max(reach.Start.x, reach.End.x) + reachMetres));
            var z0 = Math.Max(0, Index(Math.Min(reach.Start.z, reach.End.z) - reachMetres));
            var z1 = Math.Min(resolution - 1, Index(Math.Max(reach.Start.z, reach.End.z) + reachMetres));

            for (var z = z0; z <= z1; z++)
                for (var x = x0; x <= x1; x++)
                {
                    var distance =
                        NetworkNormalizer.DistanceToSegment(
                            new float3(x * spacing - half, 0, z * spacing - half),
                            reach.Start,
                            reach.End);

                    if (distance > reachMetres || distance >= nearest[z * resolution + x])
                        continue;

                    nearest[z * resolution + x] = distance;
                    level[z * resolution + x] = (reach.StartLevel + reach.EndLevel) * .5f;
                }
        }

        return level;
    }

    // Bank slopes and earth moved, counted only where a waterway's cut changed dry ground:
    // grid points in the waterway band (level), outside mapped water, lowered by more than
    // 5 cm and standing over half a metre above the water. A point's slope is its steepest
    // step to one of its four neighbours. The ground's own slope at the same point is
    // subtracted, so a natural hillside the cut did not steepen adds nothing.
    public static void CutBanks(
        string label,
        float[] ground,
        float[] planned,
        float[] level,
        float[] distance,
        bool[] wet,
        int resolution,
        float spacing)
    {
        // Newly steep ground (over 35° where the ground was not), by distance from the line
        // and by height above the water.
        var distanceLimits = new[] { 45f, 65, 90, 120, float.PositiveInfinity };
        var heightLimits = new[] { 2f, 5, 10, 20, float.PositiveInfinity };
        var byDistance = new long[distanceLimits.Length];
        var byHeight = new long[heightLimits.Length];
        float Slope(float[] map, int index) =>
            Math.Max(
                Math.Max(Math.Abs(map[index] - map[index - 1]), Math.Abs(map[index] - map[index + 1])),
                Math.Max(
                    Math.Abs(map[index] - map[index - resolution]),
                    Math.Abs(map[index] - map[index + resolution])))
                / spacing;

        var added = new List<float>();
        var steepNatural = new List<float>();
        var steepCut = new List<float>();
        long over35 = 0, over45 = 0, natural35 = 0, natural45 = 0;
        double removed = 0;

        for (var z = 1; z < resolution - 1; z++)
            for (var x = 1; x < resolution - 1; x++)
            {
                var index = z * resolution + x;

                if (float.IsNaN(level[index]) || wet[index])
                    continue;

                var cut = ground[index] - planned[index];

                if (cut > 0)
                    removed += cut;

                if (cut <= .05f || planned[index] <= level[index] + .5f)
                    continue;

                var slope = Slope(planned, index);
                var natural = Slope(ground, index);
                added.Add(slope - natural);

                if (slope > .7f)
                {
                    over35++;

                    if (natural > .7f)
                        natural35++;
                    else
                    {
                        steepNatural.Add(natural);
                        steepCut.Add(cut);
                        byDistance[Array.FindIndex(distanceLimits, limit => distance[index] < limit)]++;
                        byHeight[Array.FindIndex(heightLimits, limit => planned[index] - level[index] < limit)]++;
                    }
                }

                if (slope > 1)
                {
                    over45++;

                    if (natural > 1)
                        natural45++;
                }
            }

        var hectare = spacing * spacing / 10000;
        added.Sort();
        float Percentile(double share) => added.Count == 0 ? 0 : added[(int)Math.Min(added.Count - 1, added.Count * share)];

        Console.WriteLine(
            FormattableString.Invariant(
                $"  CUT {label}: dry ground cut {added.Count * hectare:0.0} ha; steeper than 35° {over35 * hectare:0.00} ha (ground already so {natural35 * hectare:0.00} ha), than 45° {over45 * hectare:0.00} ha (ground already so {natural45 * hectare:0.00} ha); slope added P50 {Percentile(.5):0.00}, P90 {Percentile(.9):0.00}, P99 {Percentile(.99):0.00}; earth removed {removed * spacing * spacing / 1e6:0.00} million m³"));

        static float Share(List<float> values, double share)
        {
            values.Sort();
            return values.Count == 0 ? 0 : values[(int)Math.Min(values.Count - 1, values.Count * share)];
        }

        string Bins(float[] limits, long[] counts) =>
            string.Join(
                ", ",
                limits.Select(
                    (limit, index) => FormattableString.Invariant(
                        $"{(float.IsInfinity(limit) ? $"{limits[index - 1]:0}+" : $"<{limit:0}")} m {counts[index] * hectare:0.00}")));

        Console.WriteLine(
            $"  STEEP {label}: newly over 35°, ha by distance from the line: {Bins(distanceLimits, byDistance)}; by height above the water: {Bins(heightLimits, byHeight)}; there the ground's own slope P10 {Share(steepNatural, .1):0.00}, P50 {Share(steepNatural, .5):0.00}, cut P50 {Share(steepCut, .5):0.0}, P90 {Share(steepCut, .9):0.0} m");
    }

    // A water source as the game's grid sees it: position, radius and the level it holds.
    public readonly record struct Source(float2 Position, float Radius, float Level);

    // How much of a source reaches the game's water grid: the sum over grid nodes (multiples
    // of the cell size) of the game's own weight, 1 − smoothstep(distance² / radius²)
    // (WaterSystem.CalculateSourceMultiplier, report 115). A source between nodes with a
    // radius under about 0.7 cell scores near 0 and adds almost no water.
    public static float GridWeight(Source source, float cell)
    {
        var total = 0f;
        var reach = (int)Math.Ceiling(source.Radius / cell);
        var x0 = (int)Math.Round(source.Position.x / cell);
        var z0 = (int)Math.Round(source.Position.y / cell);

        for (var z = z0 - reach; z <= z0 + reach; z++)
            for (var x = x0 - reach; x <= x0 + reach; x++)
            {
                var ratio = math.distancesq(new float2(x * cell, z * cell), source.Position) / (source.Radius * source.Radius);

                if (ratio < 1)
                    total += 1 - math.smoothstep(0, 1, ratio);
            }

        return total;
    }

    // One sample of a river line: where it is, its length, the static depth there and the
    // distance to the nearest source that reaches the grid.
    public readonly record struct LineSample(float3 Point, long WayId, float Length, float Level, float StaticDepth, float SourceDistance);

    // Walks every channel reach (no polygon, not blocked by a buried inlet) every 7 m.
    // Static depth: the planned level less the terrain at the four surrounding water-grid
    // nodes, never negative, interpolated between them: the depth the game would read back
    // there if the water stood at its planned level everywhere. Source distance: straight
    // to the nearest source whose grid weight is at least minimumWeight.
    public static List<LineSample> WalkLines(
        IEnumerable<WaterReachStage> reaches,
        Func<float, float, float> terrain,
        IReadOnlyList<Source> sources,
        float cell,
        float minimumWeight)
    {
        const float Bucket = 512;
        var buckets = new Dictionary<(int, int), List<Source>>();

        foreach (var source in sources)
        {
            if (GridWeight(source, cell) < minimumWeight)
                continue;

            var key = ((int)Math.Floor(source.Position.x / Bucket), (int)Math.Floor(source.Position.y / Bucket));

            if (!buckets.TryGetValue(key, out var list))
                buckets[key] = list = new List<Source>();

            list.Add(source);
        }

        // Nearest live source within two buckets (at least 1,024 m); beyond that, infinity.
        float Nearest(float3 point)
        {
            var best = float.PositiveInfinity;
            var bx = (int)Math.Floor(point.x / Bucket);
            var bz = (int)Math.Floor(point.z / Bucket);

            for (var z = bz - 2; z <= bz + 2; z++)
                for (var x = bx - 2; x <= bx + 2; x++)
                    if (buckets.TryGetValue((x, z), out var list))
                        foreach (var source in list)
                            best = Math.Min(best, math.distance(source.Position, point.xz));

            return best;
        }

        var samples = new List<LineSample>();

        foreach (var reach in reaches)
        {
            if (reach.PolygonId != null || reach.BlockedByBuriedInlet)
                continue;

            var length = math.distance(reach.Start.xz, reach.End.xz);
            var steps = Math.Max(1, (int)Math.Ceiling(length / 7));

            for (var k = 0; k < steps; k++)
            {
                var t = (k + .5f) / steps;
                var point = math.lerp(reach.Start, reach.End, t);
                var level = math.lerp(reach.StartLevel, reach.EndLevel, t);
                var u = point.x / cell;
                var v = point.z / cell;
                var i = (int)Math.Floor(u);
                var j = (int)Math.Floor(v);

                float Node(int a, int b) => Math.Max(0, level - terrain(a * cell, b * cell));

                var depth =
                    math.lerp(
                        math.lerp(Node(i, j), Node(i + 1, j), u - i),
                        math.lerp(Node(i, j + 1), Node(i + 1, j + 1), u - i),
                        v - j);

                samples.Add(new LineSample(point, reach.WayId, length / steps, level, depth, Nearest(point)));
            }
        }

        return samples;
    }

    // The static rule calibrated in report 117: a sample is expected dry when the grid
    // holds under MinimumStaticDepth of water at the planned level, or no live source
    // stands within MaximumSourceDistance.
    public const float MinimumStaticDepth = .5f, MaximumSourceDistance = 300, MinimumGridWeight = .5f;

    public static bool ExpectedDry(LineSample sample) =>
        sample.StaticDepth < MinimumStaticDepth || sample.SourceDistance > MaximumSourceDistance;

    // Prints the share of river line expected dry and its stretches longer than 56 m.
    public static void Connectivity(string label, List<LineSample> samples)
    {
        double total = 0, dry = 0, shallow = 0, unfed = 0;
        var stretches = 0;
        var run = 0f;
        var previousWay = 0L;

        foreach (var sample in samples)
        {
            total += sample.Length;

            if (sample.WayId != previousWay)
                run = 0;

            previousWay = sample.WayId;

            if (!ExpectedDry(sample))
            {
                run = 0;
                continue;
            }

            dry += sample.Length;

            if (sample.StaticDepth < MinimumStaticDepth)
                shallow += sample.Length;
            else
                unfed += sample.Length;

            if (run <= 56 && run + sample.Length > 56)
                stretches++;

            run += sample.Length;
        }

        var depths = samples.Select(sample => sample.StaticDepth).OrderBy(depth => depth).ToArray();
        float Percentile(double share) => depths.Length == 0 ? 0 : depths[(int)Math.Min(depths.Length - 1, depths.Length * share)];

        Console.WriteLine(
            FormattableString.Invariant(
                $"  CONNECT {label}: {total / 1000:0.0} km of river line; at risk of running dry {dry / 1000:0.00} km ({100 * dry / Math.Max(1, total):0.00}%): too shallow on the grid {shallow / 1000:0.00} km, no source within {MaximumSourceDistance:0} m {unfed / 1000:0.00} km; stretches over 56 m {stretches}; grid depth P10 {Percentile(.1):0.0}, P50 {Percentile(.5):0.0} m"));
    }

    // Calibration against a game capture: the static estimate from the game's own terrain,
    // sources and planned reaches, set against the water depth the game read back.
    // Usage: network-tests --river-connectivity <run folder>
    public static void Calibrate(string folder)
    {
        var meta = JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder, "native-water-depth.json")));
        var cells = (int)meta["cells"]!;
        var cell = (float)meta["cellMeters"]!;
        var bytes = File.ReadAllBytes(CapturedRunFiles.Resolve(folder, (string)meta["file"]!));
        var depth = new ushort[bytes.Length / 2];
        Buffer.BlockCopy(bytes, 0, depth, 0, bytes.Length);
        var half = cells * cell / 2;

        float Depth(float x, float z)
        {
            var u = (x + half) / cell;
            var v = (z + half) / cell;
            var i = (int)Math.Floor(u);
            var j = (int)Math.Floor(v);

            float At(int a, int b) =>
                a < 0 || b < 0 || a >= cells || b >= cells ? 0 : depth[b * cells + a] / 100f;

            return math.lerp(
                math.lerp(At(i, j), At(i + 1, j), u - i),
                math.lerp(At(i, j + 1), At(i + 1, j + 1), u - i),
                v - j);
        }

        var terrainMeta =
            JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder, "native-observed-terrain.json")));

        var ground =
            CapturedRunFiles.Heights(
                CapturedRunFiles.Resolve(folder, (string)terrainMeta["heightFile"]!),
                (float)terrainMeta["scale"]!);

        var n = (int)Math.Sqrt(ground.Length);
        var terrainWidth = (float)terrainMeta["width"]!;

        float Ground(float x, float z)
        {
            var u = Math.Clamp((x / terrainWidth + .5f) * (n - 1), 0, n - 1.001f);
            var v = Math.Clamp((z / terrainWidth + .5f) * (n - 1), 0, n - 1.001f);
            var i = (int)u;
            var j = (int)v;

            return math.lerp(
                math.lerp(ground[j * n + i], ground[j * n + i + 1], u - i),
                math.lerp(ground[(j + 1) * n + i], ground[(j + 1) * n + i + 1], u - i),
                v - j);
        }

        var snapshot =
            JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder, "native-snapshot-after-apply.json")));

        var sources =
            (snapshot["water"] as JArray ?? new JArray())
                .Select(
                    item => new Source(
                        new float2((float)item["x"]!, (float)item["z"]!),
                        (float)item["radius"]!,
                        (float)item["y"]! + (float)item["depth"]!))
                .ToArray();

        var manifest =
            JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder, "offline-design-pass-01-input.json")));

        var reaches =
            CorrectiveCapturedReplay.ReadManifest<WaterReachStage>(folder, (string)manifest["waterReachesManifest"]!);

        var samples = WalkLines(reaches, Ground, sources, cell, MinimumGridWeight);
        var depthBins = new[] { 0f, .25f, .5f, 1, 2, 3.5f, float.PositiveInfinity };
        var distanceBins = new[] { 0f, 100, 200, 300, 500, float.PositiveInfinity };

        Console.WriteLine($"calibration {Path.GetFileName(folder)}: {samples.Sum(sample => sample.Length) / 1000:0.0} km of river line, {sources.Length} sources ({sources.Count(source => GridWeight(source, cell) >= MinimumGridWeight)} reaching the grid)");
        Console.WriteLine("  share dry in the game (km of line), by static grid depth (rows) and distance to the nearest live source (columns)");
        Console.WriteLine("  depth \\ source   " + string.Join("", distanceBins.Skip(1).Select((limit, index) => $"{(float.IsInfinity(limit) ? $"{distanceBins[index]:0}+" : $"<{limit:0}"),16}")));

        for (var row = 0; row + 1 < depthBins.Length; row++)
        {
            var line = $"  {depthBins[row],4:0.00}-{depthBins[row + 1],-5:0.0}      ";

            for (var column = 0; column + 1 < distanceBins.Length; column++)
            {
                var bin =
                    samples
                        .Where(
                            sample => sample.StaticDepth >= depthBins[row]
                                && sample.StaticDepth < depthBins[row + 1]
                                && sample.SourceDistance >= distanceBins[column]
                                && sample.SourceDistance < distanceBins[column + 1])
                        .ToArray();

                var length = bin.Sum(sample => sample.Length);
                var dry = bin.Where(sample => Depth(sample.Point.x, sample.Point.z) < .1f).Sum(sample => sample.Length);
                line += $"{(length == 0 ? "-" : $"{100 * dry / length:0}% ({length / 1000:0.0})"),16}";
            }

            Console.WriteLine(line);
        }

        double hit = 0, miss = 0, falseAlarm = 0, total = 0;

        foreach (var sample in samples)
        {
            var dryInGame = Depth(sample.Point.x, sample.Point.z) < .1f;
            total += sample.Length;

            if (dryInGame && ExpectedDry(sample))
                hit += sample.Length;
            else if (dryInGame)
                miss += sample.Length;
            else if (ExpectedDry(sample))
                falseAlarm += sample.Length;
        }

        Console.WriteLine(
            FormattableString.Invariant(
                $"  rule (depth under {MinimumStaticDepth} m or no live source within {MaximumSourceDistance:0} m): dry in game {(hit + miss) / 1000:0.00} km ({100 * (hit + miss) / Math.Max(1, total):0.0}%); expected dry {(hit + falseAlarm) / 1000:0.00} km; found {hit / 1000:0.00} km of the game's dry line ({100 * hit / Math.Max(.001, hit + miss):0}%), missed {miss / 1000:0.00} km, false alarm {falseAlarm / 1000:0.00} km"));
    }
}
