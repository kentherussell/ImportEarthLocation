using ImportEarthLocation.Processing;

// TerrainExtremes on synthetic grids, terrain only: what is an extreme and is smoothed,
// and what is real terrain and is kept.
static class TerrainExtremesTests
{
    private const int Size = 64;

    public static void Run()
    {
        var none = new byte[Size * Size];

        // A 6 m terrace between two flats becomes a ramp; ground away from it is kept.
        var terrace = Grid((x, z) => x < 32 ? 10 : 16);
        var smoothed = TerrainExtremes.Smooth(terrace, null, null, none, none).Heights;
        Check(LargestRise(smoothed) <= TerrainExtremes.MaxRise + .05f, $"Terrace left a rise of {LargestRise(smoothed):0.00} m");
        Check(Metres(smoothed, 5, 30) == Metres(terrace, 5, 30), "Flat ground far from the terrace moved");
        Check(Metres(smoothed, 60, 30) == Metres(terrace, 60, 30), "Flat ground far from the terrace moved");

        // A steep slope (a mountainside, 5 m per cell) is not an extreme.
        var slope = Grid((x, z) => 100 + 5f * x);
        Check(Same(TerrainExtremes.Smooth(slope, null, null, none, none).Heights, slope), "A slope was smoothed");

        // A water edge is real: the same terrace with water on its low side is kept.
        var water = Mask((x, z) => x < 32);
        Check(Same(TerrainExtremes.Smooth(terrace, null, null, water, none).Heights, terrace), "A water edge moved");

        // A mapped cliff line is real.
        var cliff = Mask((x, z) => x == 31 || x == 32);
        Check(Same(TerrainExtremes.Smooth(terrace, null, null, none, cliff).Heights, terrace), "A mapped cliff moved");

        // A road deck 6 m below the sloping ground beside it (so not a terrace): the wall
        // becomes a bank, the deck stays.
        var deck = new bool[Size * Size];
        var trench = Grid((x, z) => x is >= 30 and <= 33 ? 10 : 16 + .5f * Math.Abs(x - 31.5f));

        for (var z = 0; z < Size; z++)
            for (var x = 30; x <= 33; x++)
                deck[z * Size + x] = true;

        var banked = TerrainExtremes.Smooth(trench, null, deck, none, none);
        Check(banked.Walls > 0, "The wall beside the deck was not found");
        Check(LargestRise(banked.Heights) <= TerrainExtremes.MaxRise + .05f, $"Wall left a rise of {LargestRise(banked.Heights):0.00} m");

        for (var z = 0; z < Size; z++)
            for (var x = 30; x <= 33; x++)
                Check(banked.Heights[z * Size + x] == trench[z * Size + x], "A deck cell moved");

        // A single cell 20 m above flat ground is a spike.
        var spike = Grid((x, z) => x == 20 && z == 20 ? 30 : 10);
        var (despiked, removed) = TerrainExtremes.RemoveSpikes(spike);
        Check(removed == 1 && Metres(despiked, 20, 20) == Metres(spike, 0, 0), "The spike was not removed");

        Console.WriteLine("PASS: terrain extremes: terrace and wall smoothed under 2 m per cell; slope, water edge, cliff and deck kept; spike removed.");
    }

    private static float[] Grid(Func<int, int, float> metres)
    {
        var grid = new float[Size * Size];

        for (var z = 0; z < Size; z++)
            for (var x = 0; x < Size; x++)
                grid[z * Size + x] = metres(x, z);

        return grid;
    }

    private static byte[] Mask(Func<int, int, bool> set)
    {
        var mask = new byte[Size * Size];

        for (var z = 0; z < Size; z++)
            for (var x = 0; x < Size; x++)
                mask[z * Size + x] = set(x, z) ? (byte)1 : (byte)0;

        return mask;
    }

    private static float Metres(float[] grid, int x, int z) => grid[z * Size + x];

    private static float LargestRise(float[] grid)
    {
        var largest = 0f;

        for (var z = 0; z < Size; z++)
            for (var x = 0; x + 1 < Size; x++)
            {
                largest = Math.Max(largest, Math.Abs(Metres(grid, x + 1, z) - Metres(grid, x, z)));
                largest = Math.Max(largest, Math.Abs(Metres(grid, z, x + 1) - Metres(grid, z, x)));
            }

        return largest;
    }

    private static bool Same(float[] a, float[] b) => a.AsSpan().SequenceEqual(b);
}
