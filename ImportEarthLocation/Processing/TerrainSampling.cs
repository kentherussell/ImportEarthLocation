using System;

namespace ImportEarthLocation.Processing;

// One corner-aligned interpolation convention for preparation and native uploads.
public static class TerrainSampling
{
    public static int Resolution(ushort[] raster)
    {
        if (raster == null) throw new ArgumentNullException(nameof(raster));
        var size = (int)Math.Sqrt(raster.Length);
        if (size < 2 || size * size != raster.Length)
            throw new ArgumentException("Heightmap must be a nonempty square raster.", nameof(raster));
        return size;
    }

    // u/v are normalized x/z coordinates; the result is in encoded ushort
    // height units, not metres. Raster indexing is row-major z, then x.
    public static double Encoded(ushort[] raster, int size, double u, double v)
        =>Encoded(raster,size,size,u,v);

    public static double Encoded(ushort[] raster,int width,int height,double u,double v)
    {
        var x = Math.Max(0, Math.Min(1, u)) * (width - 1);
        var z = Math.Max(0, Math.Min(1, v)) * (height - 1);
        var x0 = (int)x; var z0 = (int)z;
        var x1 = Math.Min(x0 + 1, width - 1); var z1 = Math.Min(z0 + 1, height - 1);
        var tx = x - x0; var tz = z - z0;
        var a = raster[z0 * width + x0] * (1 - tx) + raster[z0 * width + x1] * tx;
        var b = raster[z1 * width + x0] * (1 - tx) + raster[z1 * width + x1] * tx;
        return a * (1 - tz) + b * tz;
    }

    // Catmull-Rom weights for samples at -1, 0, 1 and 2 around fraction t: the
    // cubic through four samples whose slope at each sample is the central
    // difference, so a surface built from them has continuous slope across
    // source samples. Bilinear interpolation changes slope at every source
    // sample; with output pixels finer than the DEM, lighting shows that as a
    // grid of facets.
    public static (float W0, float W1, float W2, float W3) CubicWeights(float t)
    {
        var t2 = t * t; var t3 = t2 * t;
        return (.5f * (-t3 + 2 * t2 - t), .5f * (3 * t3 - 5 * t2 + 2), .5f * (-3 * t3 + 4 * t2 + t), .5f * (t3 - t2));
    }

    // Bicubic value in source cell (x, y), clamped to the cell's four samples:
    // a cubic can ring past a coast or cliff step, and no output may leave its
    // source cell's range. read(x, y) must cover x-1..x+2 and y-1..y+2.
    public static float ClampedCubic(Func<int, int, float> read, int x, int y,
        (float W0, float W1, float W2, float W3) column, (float W0, float W1, float W2, float W3) row)
    {
        // Each of the 16 neighbours is read once; the clamp reuses the central four.
        float Row(float r0, float r1, float r2, float r3) => column.W0 * r0 + column.W1 * r1 + column.W2 * r2 + column.W3 * r3;
        var top = Row(read(x - 1, y - 1), read(x, y - 1), read(x + 1, y - 1), read(x + 2, y - 1));
        var a = read(x, y); var b = read(x + 1, y);
        var upper = Row(read(x - 1, y), a, b, read(x + 2, y));
        var c = read(x, y + 1); var d = read(x + 1, y + 1);
        var lower = Row(read(x - 1, y + 1), c, d, read(x + 2, y + 1));
        var bottom = Row(read(x - 1, y + 2), read(x, y + 2), read(x + 1, y + 2), read(x + 2, y + 2));
        var value = row.W0 * top + row.W1 * upper + row.W2 * lower + row.W3 * bottom;
        return Math.Min(Math.Max(value, Math.Min(Math.Min(a, b), Math.Min(c, d))), Math.Max(Math.Max(a, b), Math.Max(c, d)));
    }

    public static ushort[] Resample(ushort[] raster, int width, int height)
    {
        var size = Resolution(raster);
        if (width < 1 || height < 1) throw new ArgumentOutOfRangeException(nameof(width));
        if (width == size && height == size) return raster;
        var result = new ushort[checked(width * height)];
        for (var z = 0; z < height; z++)
            for (var x = 0; x < width; x++)
                result[z * width + x] = (ushort)Math.Round(Encoded(raster, size,
                    x / (double)Math.Max(1, width - 1), z / (double)Math.Max(1, height - 1)));
        return result;
    }
}
