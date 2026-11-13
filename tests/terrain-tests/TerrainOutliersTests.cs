using ImportEarthLocation.Processing;

// TerrainOutliers on synthetic source grids, about 16.5 m a sample as at zoom 13.
static class TerrainOutliersTests
{
    private const int Size = 256;

    public static void Run()
    {
        // A lake at 0 m with New Orleans-style corrupt patches: out of range, in range
        // but kilometres deep, the -32,768 m NoData colour, and unreadable samples.
        // The first patch has a halo smeared to -300 m around it.
        var lake = Grid((x, z) => 0);
        Paint(lake, 39, 39, 12, 12, -300);
        Paint(lake, 40, 40, 10, 10, -15437);
        Paint(lake, 52, 44, 6, 3, -11989);
        Paint(lake, 120, 90, 12, 40, -32768);
        Paint(lake, 200, 200, 3, 3, float.NaN);
        var fixedLake = TerrainOutliers.Replace(lake, Size, Size, default);
        Check(fixedLake.Samples == 144 + 18 + 480 + 9, $"The lake's corrupt samples were miscounted: {fixedLake.Samples}");
        Check(lake.All(h => Math.Abs(h) < .01f), $"A lake patch was not filled at 0 m: {lake.Min()} to {lake.Max()}");

        // A slope with a corrupt patch is filled on the slope, not flat.
        var slope = Grid((x, z) => 100 + x * 2f);
        var slopeOriginal = (float[])slope.Clone();
        Paint(slope, 100, 100, 8, 8, -20000);
        TerrainOutliers.Replace(slope, Size, Size, default);
        Check(Same(slope, slopeOriginal, 1), "A patch on a slope was not filled along the slope");

        // Real ground never moves: a 900 m gorge 20 samples wide cut into a flat
        // plateau, a 2 km mountain range, a deep ocean beside a steep coast, and a
        // 200 m quarry pit in flat land (a halo only grows from proven corruption).
        var gorge = Grid((x, z) => x is >= 120 and < 140 ? 100 : 1000);
        var quarry = Grid((x, z) => x is >= 60 and < 80 && z is >= 60 and < 80 ? -180 : 20);
        var mountains = Grid((x, z) => 2000 + 1500 * MathF.Sin(x / 20f) * MathF.Cos(z / 25f));
        var coast = Grid((x, z) => x < 128 ? -3000 + x * 5 : 50 + (x - 128) * 20);

        foreach (var (name, ground) in new[] { ("gorge", gorge), ("mountains", mountains), ("coast", coast), ("quarry", quarry) })
        {
            var original = (float[])ground.Clone();
            var outcome = TerrainOutliers.Replace(ground, Size, Size, default);
            Check(outcome.Samples == 0 && Same(ground, original, 0), $"Real ground was changed ({name}): {outcome.Samples} samples");
        }

        // A spike kilometres above rough mountains is corrupt too.
        var spiked = Grid((x, z) => 2000 + 1500 * MathF.Sin(x / 20f) * MathF.Cos(z / 25f));
        Paint(spiked, 60, 60, 2, 2, 9500);
        Check(TerrainOutliers.Replace(spiked, Size, Size, default).Samples == 4 && spiked.Max() < 3600, "A spike in the mountains was kept");

        // Pits and a trench under a kilometre deep in flat shore (Panama City): within the
        // quartile band, but one sample wide across, so no ground.
        var shore = Grid((x, z) => 5 + x * .02f);
        Paint(shore, 100, 100, 1, 1, -997);
        Paint(shore, 150, 150, 1, 2, -650);
        Paint(shore, 40, 200, 10, 1, -993);
        Check(
            TerrainOutliers.Replace(shore, Size, Size, default).Samples == 13 && shore.Min() > 0,
            $"Kilometre pits in a flat shore were kept: lowest {shore.Min():0} m");

        // A whole unreadable tile in a valley is invented across it, from the slopes on
        // both sides; a source with no valid ground at all is refused.
        var valley = Grid((x, z) => 300 + Math.Abs(x - 128) * 4f);
        var valleyOriginal = (float[])valley.Clone();
        Paint(valley, 40, 40, 160, 160, float.NaN);
        TerrainOutliers.Replace(valley, Size, Size, default);
        var valleyFloor = valley[120 * Size + 128];
        Check(
            valley.All(float.IsFinite) && valleyFloor > 300 && valleyFloor < valleyOriginal[120 * Size + 40],
            $"A missing tile in a valley was not spanned from both sides: floor {valleyFloor:0} m");

        var empty = Grid((x, z) => float.NaN);
        Check(Throws(() => TerrainOutliers.Replace(empty, Size, Size, default)), "Ground was invented with nothing to go on");

        // Santa Barbara: a corrupt trench 21 samples wide down the map's west edge, each
        // column one constant: about 1 km a column down to -16 km and back up, so partly
        // inside the valid range. It is filled from the ground beside it.
        const int stripWidth = 1280;
        const int stripHeight = 2048;
        const int trench = 21;
        var strip = new float[stripWidth * stripHeight];

        for (var z = 0; z < stripHeight; z++)
            for (var x = 0; x < stripWidth; x++)
                strip[z * stripWidth + x] =
                    x < 5 ? -12042 - 1017 * x :
                    x < trench ? -392 - 1017 * (trench - 1 - x) :
                    100 + x * .5f + z * .1f;

        var fixedStrip = TerrainOutliers.Replace(strip, stripWidth, stripHeight, default);
        var stripError = 0f;

        for (var z = 0; z < stripHeight; z++)
            for (var x = 0; x < trench; x++)
                stripError = Math.Max(stripError, Math.Abs(strip[z * stripWidth + x] - strip[z * stripWidth + trench]));

        Check(
            fixedStrip.Samples == trench * stripHeight && fixedStrip.Patches == 1 && stripError < 20,
            $"A thin corrupt strip was not filled from the ground beside it: {fixedStrip.Samples} samples, {stripError:0.0} m off");

        // A block as large and thick is filled at the level around it.
        var block = new float[stripWidth * stripHeight];

        for (var z = 0; z < stripHeight; z++)
            for (var x = 0; x < stripWidth; x++)
                block[z * stripWidth + x] = x is >= 200 and < 330 && z is >= 900 and < 1030 ? -16606 : 100;

        TerrainOutliers.Replace(block, stripWidth, stripHeight, default);
        Check(block.All(h => Math.Abs(h - 100) < .01f), $"A thick corrupt block was not filled at 100 m: {block.Min():0.0} to {block.Max():0.0}");

        Console.WriteLine("PASS: terrain outliers: lake patches and their halo filled at the lake's level, slope patch filled on the slope, spike and narrow pits removed; gorge, mountains, steep coast and quarry untouched; a thin strip down the map edge, a thick block and a missing tile in a valley filled; a source with no ground refused.");
    }

    // Runs the pass over a raw little-endian float32 mosaic of decoded source tiles
    // and reports what it replaced and the range before and after.
    public static void Mosaic(string path, int width)
    {
        var bytes = File.ReadAllBytes(path);
        var raster = new float[bytes.Length / 4];
        Buffer.BlockCopy(bytes, 0, raster, 0, bytes.Length);
        var height = raster.Length / width;
        var before = Range(raster);
        var original = (float[])raster.Clone();
        var clock = System.Diagnostics.Stopwatch.StartNew();
        var outcome = TerrainOutliers.Replace(raster, width, height, default);
        var after = Range(raster);
        Console.WriteLine(
            $"{Path.GetFileName(path)}: {width}x{height}; replaced {outcome.Samples:N0} samples in {outcome.Patches} patches (largest {outcome.LargestPatch}), {outcome.Lowest:0} to {outcome.Highest:0} m; range {before.Low:0.0}..{before.High:0.0} m -> {after.Low:0.0}..{after.High:0.0} m; {clock.Elapsed.TotalSeconds:0.0} s");

        DescribePatches(original, raster, width, height);

        var filled = new byte[bytes.Length];
        Buffer.BlockCopy(raster, 0, filled, 0, filled.Length);
        File.WriteAllBytes(Path.ChangeExtension(path, ".filled.f32"), filled);

        var lowest = Array.IndexOf(raster, after.Low);
        var highest = Array.IndexOf(raster, after.High);
        Console.WriteLine(
            $"  after: lowest {after.Low:0} m at x {lowest % width}, z {lowest / width}; highest {after.High:0} m at x {highest % width}, z {highest / width}");
    }

    // Each replaced patch (8-connected changed samples): its box, size, the values it
    // held and the values it was given, largest first.
    private static void DescribePatches(float[] original, float[] replaced, int width, int height)
    {
        var seen = new bool[original.Length];
        var patches = new List<string>();
        var sizes = new List<int>();

        for (var start = 0; start < original.Length; start++)
        {
            if (seen[start] || original[start].Equals(replaced[start]))
                continue;

            var members = new List<int> { start };
            seen[start] = true;

            for (var next = 0; next < members.Count; next++)
            {
                var x = members[next] % width;
                var z = members[next] / width;

                for (var dz = -1; dz <= 1; dz++)
                    for (var dx = -1; dx <= 1; dx++)
                    {
                        var nx = x + dx;
                        var nz = z + dz;

                        if (nx < 0 || nz < 0 || nx >= width || nz >= height)
                            continue;

                        var index = nz * width + nx;

                        if (!seen[index] && !original[index].Equals(replaced[index]))
                        {
                            seen[index] = true;
                            members.Add(index);
                        }
                    }
            }

            var held = members.Select(index => original[index]).ToArray();
            var given = members.Select(index => replaced[index]).ToArray();
            var columns = members.Select(index => index % width).ToArray();
            var rows = members.Select(index => index / width).ToArray();
            sizes.Add(members.Count);
            patches.Add(
                $"  patch {members.Count} samples at x {columns.Min()}..{columns.Max()}, z {rows.Min()}..{rows.Max()}: "
                    + $"held {held.Min():0}..{held.Max():0} m (median {held.OrderBy(value => value).ElementAt(held.Length / 2):0}), "
                    + $"given {given.Min():0}..{given.Max():0} m");
        }

        foreach (var line in patches.Zip(sizes).OrderByDescending(item => item.Second).Take(12).Select(item => item.First))
            Console.WriteLine(line);
    }

    private static (float Low, float High) Range(float[] raster)
    {
        var low = float.PositiveInfinity;
        var high = float.NegativeInfinity;

        foreach (var value in raster)
        {
            if (!float.IsFinite(value))
                continue;

            low = Math.Min(low, value);
            high = Math.Max(high, value);
        }

        return (low, high);
    }

    private static float[] Grid(Func<int, int, float> metres)
    {
        var grid = new float[Size * Size];

        for (var z = 0; z < Size; z++)
            for (var x = 0; x < Size; x++)
                grid[z * Size + x] = metres(x, z);

        return grid;
    }

    private static void Paint(float[] grid, int x0, int z0, int width, int height, float value)
    {
        for (var z = z0; z < z0 + height; z++)
            for (var x = x0; x < x0 + width; x++)
                grid[z * Size + x] = value;
    }

    private static bool Same(float[] a, float[] b, float tolerance)
    {
        for (var i = 0; i < a.Length; i++)
            if (!(Math.Abs(a[i] - b[i]) <= tolerance))
                return false;

        return true;
    }

    private static bool Throws(Action action)
    {
        try
        {
            action();
            return false;
        }
        catch (InvalidOperationException)
        {
            return true;
        }
    }
}
