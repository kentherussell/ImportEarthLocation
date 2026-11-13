using ImportEarthLocation.Processing;

// FloodDefences on synthetic grids, terrain only. The sea surface is at 10 m; the sea
// fills the west, a dyke at x = 10 and 11 holds it off a polder at 7 m, closed by high
// ground on its other three sides.
static class FloodDefencesTests
{
    private const int Size = 64;
    private const float Scale = 100;
    private const float Sea = 10;
    private const double Width = 20 * (Size - 1);

    public static void Run()
    {
        var sea = Mask((x, z) => x < 10, FloodDefences.OpenWater);
        var canal = Mask((x, z) => x is 40 or 41 && z is > 10 and < 50);
        var wet = Mask((x, z) => sea[z * Size + x] != 0 ? FloodDefences.OpenWater : canal[z * Size + x]);
        var none = new byte[Size * Size];

        float Polder(int x, int z, bool gap)
            => x < 10 ? 5 :
                z <= 1 || z >= Size - 2 || x >= Size - 2 ? 14 :
                x <= 11 ? (gap && z is >= 30 and <= 33 ? 7 : 14) :
                canal[z * Size + x] != 0 ? 6 : 7;

        // A dyke with a gap: the gap is closed, the polder and its canal keep their height.
        var breached = Grid((x, z) => Polder(x, z, true));
        var closed = FloodDefences.Build(breached, Scale, Width, Sea, wet, none);
        Check(closed.LandReachedBefore > 0, "The breached polder was not found");
        Check(closed.LandReachedAfter == 0, $"{closed.LandReachedAfter} polder cells still reached by the sea");
        Check(Metres(closed.Heights, 10, 31) >= Sea + FloodDefences.Freeboard - .01f, "The gap was not closed");
        Check(Same(closed.Heights, breached, x => x >= 14), "Polder land or its canal moved");
        Check(Unreached(closed.Heights), "The sea still reaches land below it");

        // The same gap as a mapped ditch through the dyke: open to the sea, it is the sea's
        // side, so it stays open and gets a dyke along it; the polder stays dry.
        var ditch = Mask((x, z) => sea[z * Size + x] != 0 ? FloodDefences.OpenWater
            : canal[z * Size + x] != 0 || x is 10 or 11 && z is >= 30 and <= 33 ? FloodDefences.MappedWater : (byte)0);
        var inlet = FloodDefences.Build(breached, Scale, Width, Sea, ditch, none);
        Check(inlet.LandReachedAfter == 0 && Unreached(inlet.Heights, ditch), "A ditch through the dyke flooded the polder");

        // A polder's own ditch inside the dyke band, reached only across its land, is filled:
        // left open, it is a hole the game wears the dyke around.
        var inner = Mask((x, z) => sea[z * Size + x] != 0 ? FloodDefences.OpenWater
            : canal[z * Size + x] != 0 || x == 11 && z is 31 or 32 ? FloodDefences.MappedWater : (byte)0);
        var filled = FloodDefences.Build(breached, Scale, Width, Sea, inner, none);
        Check(filled.LandReachedAfter == 0 && Metres(filled.Heights, 11, 31) >= Sea + FloodDefences.Freeboard - .01f, $"A ditch inside the dyke band was left open: after {filled.LandReachedAfter}, ditch {Metres(filled.Heights, 11, 31):0.0} m, beside {Metres(filled.Heights, 10, 31):0.0} m, {Metres(filled.Heights, 12, 31):0.0} m");

        // A polder cut by mapped ditches every fourth column into strips far under
        // SmallestPolder (180 cells against 625) is still one polder: the gap is closed.
        var ditches = Mask((x, z) => sea[z * Size + x] != 0 ? FloodDefences.OpenWater
            : x >= 14 && x < Size - 2 && x % 4 == 0 && z is > 1 and < Size - 2 ? FloodDefences.MappedWater : (byte)0);
        var striped = FloodDefences.Build(breached, Scale, Width, Sea, ditches, none);
        Check(
            striped.LandReachedAfter == 0 && Unreached(striped.Heights, ditches),
            $"A polder cut into strips by its ditches was left to the sea: {striped.LandReachedAfter} cells");

        // A deep harbour (mapped water 200 m by 240 m, bed at -20 m) cut through the dyke into
        // the polder is open water, not a ditch: it stays the sea's, and the polder gets a dyke
        // along it. Taken into the polder, its bed would set the dyke over TallestDyke and the
        // polder would be left to the sea.
        bool Harbour(int x, int z) => x is >= 10 and <= 19 && z is >= 26 and <= 37;
        var harboured = Grid((x, z) => Harbour(x, z) ? -20 : Polder(x, z, false));
        var harbourWater = Mask((x, z) => sea[z * Size + x] != 0 ? FloodDefences.OpenWater
            : Harbour(x, z) || canal[z * Size + x] != 0 ? FloodDefences.MappedWater : (byte)0);
        var harbourDefended = FloodDefences.Build(harboured, Scale, Width, Sea, harbourWater, none);
        Check(
            harbourDefended.LandReachedAfter == 0
                && Unreached(harbourDefended.Heights, harbourWater)
                && Metres(harbourDefended.Heights, 15, 30) == -20,
            $"A polder beside a deep harbour was left to the sea or the harbour filled: {harbourDefended.LandReachedAfter} cells");

        // A whole ring dyke is left alone.
        var whole = Grid((x, z) => Polder(x, z, false));
        var kept = FloodDefences.Build(whole, Scale, Width, Sea, wet, none);
        Check(kept.LandReachedBefore == 0 && Same(kept.Heights, whole, _ => true), "A sound dyke was changed");

        // A mapped dyke over the gap is raised; nothing else needs building.
        var mapped = Mask((x, z) => x == 10 && z is >= 28 and <= 35);
        var raised = FloodDefences.Build(breached, Scale, Width, Sea, wet, mapped);
        Check(raised.MappedCells > 0 && raised.BuiltCells == 0 && raised.LandReachedAfter == 0, "The mapped dyke did not close the gap");

        // A small low patch of shore (well under SmallestPolder) is left to the sea.
        var shore = Grid((x, z) => x < 10 ? 5 : x < 13 && z is >= 20 and < 23 ? 8 : 15);
        Check(Same(FloodDefences.Build(shore, Scale, Width, Sea, sea, none).Heights, shore, _ => true), "A small low shore was dyked");

        // A polder cut by the map edge gets a dyke along the edge.
        var edge = Grid((x, z) => x < 10 ? 5 : x <= 11 ? 14 : z == Size - 1 ? 7 : 7);
        var edged = FloodDefences.Build(edge, Scale, Width, Sea, sea, none);
        Check(edged.LandReachedAfter == 0 && Unreached(edged.Heights), "The map edge of a polder was left open");

        // Earthworks that cut a defence are put back.
        var cut = (float[])closed.Heights.Clone();
        cut[31 * Size + 10] = 0;
        Check(FloodDefences.Restore(cut, closed.Crest) == 1 && Unreached(cut), "A cut defence was not restored");

        // An inland lake held at 12 m beside a polder at 8 m, all inside ground at 20 m the
        // sea never reaches: the lake floods the polder until a dyke 1 m above the lake
        // (not the sea) holds it; the polder's far side and the lake bed never move.
        bool Lake(int x, int z) => x is >= 40 and <= 50 && z is >= 20 and <= 40;
        bool Low(int x, int z) => x is >= 20 and <= 39 && z is >= 10 and <= 50;
        var inland = Grid((x, z) => Lake(x, z) ? 5 : Low(x, z) ? 8 : 20);
        var surface = new float[Size * Size];

        for (var id = 0; id < surface.Length; id++)
            surface[id] = Lake(id % Size, id / Size) ? 12 : float.NaN;

        var held = FloodDefences.Build(inland, Scale, Width, Sea, none, none, default, surface);
        Check(held.LandReachedBefore > 0 && held.LandReachedAfter == 0, $"The lake still floods {held.LandReachedAfter} polder cells");
        Check(Math.Abs(Metres(held.Heights, 39, 30) - (12 + FloodDefences.Freeboard)) < .01f, "The dyke was not built 1 m above the lake");
        Check(Same(held.Heights, inland, x => x < 36 || x >= 40), "The polder's far side or the lake bed moved");
        Check(Same(FloodDefences.Build(inland, Scale, Width, Sea, none, none).Heights, inland, _ => true), "Without the lake's stage the polder was dyked");

        // A river at 30 m falling into a valley that drains off the map edge (ground at 25 m
        // falling to 15 m at the west edge, above the sea): its water runs down the valley
        // and pools nowhere, so nothing is built.
        var valley = Grid((x, z) => z is >= 20 and <= 40 ? 15 + x * 10f / Size : 40);
        var river = new float[Size * Size];

        for (var id = 0; id < river.Length; id++)
            river[id] = id / Size == 30 && id % Size == Size - 1 ? 30 : float.NaN;

        var drained = FloodDefences.Build(valley, Scale, Width, Sea, none, none, default, river);
        Check(drained.LandReachedBefore == 0 && Same(drained.Heights, valley, _ => true), "A draining valley was dyked");

        // On the game's grid, samples fall between ours. A diagonal dyke one cell wide,
        // sampled half a cell off, averages with the sea and polder beside it to 9.5 m,
        // below the sea, and leaks; keeping crests holds it at full height. (At 14 m the
        // average is exactly the sea's 10 m, which leaked only through 16-bit rounding.)
        bool Diagonal(int x, int z) => x + z == 60;
        var diagonal = Grid((x, z) => x + z < 60 ? 5 : Diagonal(x, z) ? 13 : x >= Size - 3 || z >= Size - 3 ? 14 : 7);
        var diagonalCrests = new float[Size * Size];
        Array.Fill(diagonalCrests, float.NaN);

        for (var id = 0; id < diagonalCrests.Length; id++)
            if (Diagonal(id % Size, id / Size))
                diagonalCrests[id] = diagonal[id];

        float[] HalfCellOff(float[]? crests)
        {
            var grid = new float[Size * Size];
            var step = .5 / (Size - 1);

            for (var z = 0; z < Size; z++)
                for (var x = 0; x < Size; x++)
                    grid[z * Size + x] =
                        (float)TerrainSampling.HeightKeepingCrests(
                            diagonal,
                            crests,
                            Size,
                            Math.Min(1, x / (double)(Size - 1) + step),
                            Math.Min(1, z / (double)(Size - 1) + step));

            return grid;
        }

        var diagonalSea = Mask((x, z) => x + z < 60, FloodDefences.OpenWater);
        Check(!Unreached(HalfCellOff(null), diagonalSea), "A thin diagonal dyke held under plain bilinear sampling; the test no longer shows the leak");
        Check(Unreached(HalfCellOff(diagonalCrests), diagonalSea), "A thin diagonal dyke leaked on the game's grid with its crest kept");

        Console.WriteLine("PASS: flood defences: thin diagonal dyke kept whole on the game's grid (leaks without), gap closed, sound dyke kept, mapped dyke raised, small shore left, map edge closed, cut restored, lake above a polder held, draining valley left, ditch inlet walled, inner ditch filled, polder striped by ditches closed, deep harbour kept open; polder and canal never moved.");

        bool Unreached(float[] grid, byte[]? water = null)
        {
            var reached = FloodDefences.Reach(grid, Size, Sea);

            for (var id = 0; id < reached.Length; id++)
                if (reached[id] && (water ?? sea)[id] == 0)
                    return false;

            return true;
        }
    }

    private static float[] Grid(Func<int, int, float> metres)
    {
        var grid = new float[Size * Size];

        for (var z = 0; z < Size; z++)
            for (var x = 0; x < Size; x++)
                grid[z * Size + x] = metres(x, z);

        return grid;
    }

    private static byte[] Mask(Func<int, int, bool> set, byte value = FloodDefences.MappedWater)
    {
        var mask = new byte[Size * Size];

        for (var z = 0; z < Size; z++)
            for (var x = 0; x < Size; x++)
                mask[z * Size + x] = set(x, z) ? value : (byte)0;

        return mask;
    }

    private static byte[] Mask(Func<int, int, byte> value)
    {
        var mask = new byte[Size * Size];

        for (var z = 0; z < Size; z++)
            for (var x = 0; x < Size; x++)
                mask[z * Size + x] = value(x, z);

        return mask;
    }

    private static float Metres(float[] grid, int x, int z) => grid[z * Size + x];

    private static bool Same(float[] a, float[] b, Func<int, bool> column)
    {
        for (var id = 0; id < a.Length; id++)
            if (column(id % Size) && a[id] != b[id])
                return false;

        return true;
    }
}
