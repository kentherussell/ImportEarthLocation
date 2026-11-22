# Builds the source elevation grid of a map from the cached terrarium tiles, for
# `network-tests --water-replay` with WATER_DEM=<output> (report 122 §13). The captured
# heightmap of a run already holds the game's own water cuts, so lakes planned on it look
# contained; this grid is the ground the game planned on.
# Output: float32 sea-relative metres, row by row from the map's -z edge, n × n samples over
# the map width, bilinear between tile pixels; NaN where a tile is not cached.
# Example: tests/water-dem-grid.ps1 -Output montreal-dem.f32 -Latitude 45.503182 -Longitude -73.569807
param(
    [Parameter(Mandatory)][string]$Output,
    [Parameter(Mandatory)][double]$Latitude,
    [Parameter(Mandatory)][double]$Longitude,
    [double]$Width = 57330,
    [int]$Resolution = 4096,
    [int]$Zoom = 13
)

$ErrorActionPreference = 'Stop'
$cache = Join-Path $env:USERPROFILE 'AppData\LocalLow\Colossal Order\Cities Skylines II\ModsData\ImportEarthLocation\cache\terrain'
$drawing = [System.Drawing.Bitmap].Assembly.Location
Add-Type -AssemblyName System.Drawing

$code = @'
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Runtime.InteropServices;
using System.Security.Cryptography;
using System.Text;

public static class WaterDemGrid
{
    private static readonly Dictionary<string, float[]> Tiles = new Dictionary<string, float[]>();

    // A cached tile is named by the SHA-256 of its key; terrarium encodes metres as
    // red * 256 + green + blue / 256 - 32768.
    private static float[] Tile(string cache, int zoom, int x, int y)
    {
        var key = "terrarium-v1:" + zoom + ":" + x + ":" + y;
        float[] tile;

        if (Tiles.TryGetValue(key, out tile))
            return tile;

        using (var sha = SHA256.Create())
        {
            var hash = BitConverter.ToString(sha.ComputeHash(Encoding.UTF8.GetBytes(key)))
                .Replace("-", "")
                .ToLowerInvariant();

            var file = Path.Combine(cache, hash + ".png");

            if (File.Exists(file))
            {
                using (var bitmap = new Bitmap(file))
                {
                    var data =
                        bitmap.LockBits(
                            new Rectangle(0, 0, 256, 256),
                            ImageLockMode.ReadOnly,
                            PixelFormat.Format24bppRgb);

                    var bytes = new byte[data.Stride * 256];
                    Marshal.Copy(data.Scan0, bytes, 0, bytes.Length);
                    bitmap.UnlockBits(data);
                    tile = new float[256 * 256];

                    for (var row = 0; row < 256; row++)
                        for (var column = 0; column < 256; column++)
                        {
                            var offset = row * data.Stride + column * 3;
                            tile[row * 256 + column] =
                                bytes[offset + 2] * 256f + bytes[offset + 1] + bytes[offset] / 256f - 32768f;
                        }
                }
            }
        }

        Tiles[key] = tile;

        return tile;
    }

    private static float Pixel(string cache, int zoom, long x, long y)
    {
        var tile = Tile(cache, zoom, (int)(x >> 8), (int)(y >> 8));

        return tile == null ? float.NaN : tile[(y & 255) * 256 + (x & 255)];
    }

    public static int Build(
        string cache,
        string output,
        double latitude,
        double longitude,
        double width,
        int resolution,
        int zoom)
    {
        const double EarthRadius = 6378137.0;
        var scale = 256.0 * Math.Pow(2, zoom);
        var result = new float[resolution * resolution];
        var spacing = width / (resolution - 1);
        var missing = 0;

        for (var j = 0; j < resolution; j++)
            for (var i = 0; i < resolution; i++)
            {
                var x = i * spacing - width / 2;
                var z = j * spacing - width / 2;
                var lat = latitude + z / EarthRadius * 180 / Math.PI;
                var lon = longitude + x / (EarthRadius * Math.Cos(latitude * Math.PI / 180)) * 180 / Math.PI;
                var pixelX = (lon + 180) / 360 * scale - .5;
                var tangent = Math.Tan(lat * Math.PI / 180);
                var pixelY = (1 - Math.Log(tangent + Math.Sqrt(tangent * tangent + 1)) / Math.PI) / 2 * scale - .5;
                var x0 = (long)Math.Floor(pixelX);
                var y0 = (long)Math.Floor(pixelY);
                var fx = (float)(pixelX - x0);
                var fy = (float)(pixelY - y0);
                var a = Pixel(cache, zoom, x0, y0);
                var b = Pixel(cache, zoom, x0 + 1, y0);
                var c = Pixel(cache, zoom, x0, y0 + 1);
                var d = Pixel(cache, zoom, x0 + 1, y0 + 1);
                var value = (a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + d * fx) * fy;

                if (float.IsNaN(value))
                    missing++;

                result[j * resolution + i] = value;
            }

        var raw = new byte[result.Length * 4];
        Buffer.BlockCopy(result, 0, raw, 0, raw.Length);
        File.WriteAllBytes(output, raw);

        return missing;
    }
}
'@

$folder = Split-Path $drawing
$references = @(
    $drawing,
    (Join-Path $folder 'System.Private.Windows.GdiPlus.dll'),
    (Join-Path $folder 'System.Private.Windows.Core.dll'),
    'System.Drawing.Primitives',
    'System.Runtime.InteropServices',
    'System.Security.Cryptography',
    'System.Collections',
    'System.Runtime'
)

Add-Type -TypeDefinition $code -ReferencedAssemblies $references
$missing = [WaterDemGrid]::Build($cache, $Output, $Latitude, $Longitude, $Width, $Resolution, $Zoom)
"Wrote $Output; samples without a cached tile: $missing"
