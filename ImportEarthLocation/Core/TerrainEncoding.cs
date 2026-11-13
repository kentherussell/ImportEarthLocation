using System;
using System.Threading.Tasks;

namespace ImportEarthLocation.Core;

// The game's terrain is a 16-bit heightmap: a height in metres is value / 65,535 ×
// the height scale, measured from the bottom of the encoding (0 m). The import keeps
// heights in those metres, as floats, from the elevation data to the hand-off; it
// encodes only where the game takes the heightmap and where a run's grids are saved,
// and decodes only where saved grids are read back.
public static class TerrainEncoding
{
    // The nearest encoded value; heights outside 0..scale are held at the limits.
    public static ushort Encode(float metres, float scale)
    {
        var normalized = metres / scale;

        if (!(normalized > 0))
            return 0;

        if (normalized >= 1)
            return ushort.MaxValue;

        return (ushort)Math.Round(normalized * ushort.MaxValue);
    }

    public static float Decode(ushort value, float scale) => value / (float)ushort.MaxValue * scale;

    // In blocks of samples rather than one parallel call per sample: a 4,097² grid is
    // 16.8 million samples, and a delegate call each costs more than the work.
    private const int Block = 16384;

    public static ushort[] Encode(float[] metres, float scale)
    {
        var encoded = new ushort[metres.Length];

        Parallel.For(0, (metres.Length + Block - 1) / Block, block =>
        {
            var end = Math.Min(metres.Length, (block + 1) * Block);

            for (var index = block * Block; index < end; index++)
                encoded[index] = Encode(metres[index], scale);
        });

        return encoded;
    }

    public static float[] Decode(ushort[] encoded, float scale)
    {
        var metres = new float[encoded.Length];

        Parallel.For(0, (encoded.Length + Block - 1) / Block, block =>
        {
            var end = Math.Min(encoded.Length, (block + 1) * Block);

            for (var index = block * Block; index < end; index++)
                metres[index] = Decode(encoded[index], scale);
        });

        return metres;
    }
}
