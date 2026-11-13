using ImportEarthLocation.Processing;
using Unity.Mathematics;

// Water areas keep their polygons' holes (GeographicPolygon.FilledRings). A ring canal
// mapped as one multipolygon with its polders as inner rings, as around Amsterdam's
// Beemster and Schermer, filled as water must leave the polders as land; with outer rings
// alone the flood defences took them for water and never dyked them.
static class WaterAreaHoleTests
{
    private const int Size = 64;
    private const double Width = 20 * (Size - 1);

    public static void Run()
    {
        float3 At(int x, int z)
            => new((float)((x / (double)(Size - 1) - .5) * Width), 0, (float)((z / (double)(Size - 1) - .5) * Width));

        float3[] Square(int from, int to) => new[] { At(from, from), At(to, from), At(to, to), At(from, to) };

        var water = new Dictionary<string, string> { ["natural"] = "water" };
        var ringCanal = new GeographicPolygon(1, water, new() { Square(10, 50) }, new() { Square(14, 26), Square(32, 46) });
        var mask = TerrainExtremes.Protected(Size, Width, ringCanal.FilledRings(), Array.Empty<float3[]>());

        bool Wet(int x, int z) => mask[z * Size + x] != 0;

        if (Wet(20, 20) || Wet(40, 40))
            throw new InvalidOperationException("A ring canal's polder was filled as water.");

        if (!Wet(12, 30) || !Wet(30, 12) || !Wet(29, 29) || !Wet(48, 48))
            throw new InvalidOperationException("A ring canal's water was left out.");

        var plain = new GeographicPolygon(2, water, new() { Square(10, 50) }, new());

        if (!plain.FilledRings().SequenceEqual(plain.Outer))
            throw new InvalidOperationException("A polygon without holes changed.");

        Console.WriteLine("PASS: water areas keep their holes: a ring canal's two polders stay land, the canal and the water between them stay water.");
    }
}
