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

    public static double Encoded(ushort[] raster, int size, double u, double v)
    {
        var x = Math.Max(0, Math.Min(1, u)) * (size - 1);
        var z = Math.Max(0, Math.Min(1, v)) * (size - 1);
        var x0 = (int)x; var z0 = (int)z;
        var x1 = Math.Min(x0 + 1, size - 1); var z1 = Math.Min(z0 + 1, size - 1);
        var tx = x - x0; var tz = z - z0;
        var a = raster[z0 * size + x0] * (1 - tx) + raster[z0 * size + x1] * tx;
        var b = raster[z1 * size + x0] * (1 - tx) + raster[z1 * size + x1] * tx;
        return a * (1 - tz) + b * tz;
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
