using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using System.Threading;
using ImportEarthLocation.Core;
using ImportEarthLocation.Geo;
using UnityEngine;

namespace ImportEarthLocation.Processing;

public static class TerrainBuilder
{
    public const int Resolution = 4096;

    public sealed record Result(ushort[] Heightmap, float HeightScaleMeters, float SeaLevelMeters, float MinimumElevation, float MaximumElevation)
    { public string ConditioningSummary { get; init; } = ""; }

    public static IReadOnlyList<TerrainTilePixels> DecodeOnMainThread(IReadOnlyList<TerrainTile> tiles, CancellationToken token = default)
    {
        var decoded = new List<TerrainTilePixels>(tiles.Count);
        foreach (var tile in tiles)
        {
            token.ThrowIfCancellationRequested();
            var texture = new Texture2D(2, 2, TextureFormat.RGBA32, false, true);
            try
            {
                if (!ImageConversion.LoadImage(texture, tile.PngBytes, markNonReadable: false))
                    throw new InvalidOperationException($"Could not decode terrain tile {tile.Zoom}/{tile.X}/{tile.Y}.");
                if (texture.width != 256 || texture.height != 256)
                    throw new InvalidOperationException($"Terrain tile {tile.Zoom}/{tile.X}/{tile.Y} was {texture.width}×{texture.height}, expected 256×256.");
                decoded.Add(new TerrainTilePixels
                {
                    Zoom = tile.Zoom,
                    X = tile.X,
                    Y = tile.Y,
                    Pixels = texture.GetPixels32()
                });
            }
            finally
            {
                UnityEngine.Object.Destroy(texture);
            }
        }
        return decoded;
    }

    public static Task<Result> BuildAsync(IReadOnlyList<TerrainTilePixels> tiles, GeoBounds bounds, CancellationToken token = default, ImportLog? log = null, OsmDataset? geography = null) =>
        Task.Run(() =>
        {
            try { return Build(tiles, bounds, token, log, geography); }
            catch (OutOfMemoryException exception) { throw new InvalidOperationException("Terrain preparation: insufficient memory for validated source/working height buffers. Current map unchanged; cached downloads retained.", exception); }
        }, token);

    private static Result Build(IReadOnlyList<TerrainTilePixels> tiles, GeoBounds bounds, CancellationToken token, ImportLog? log, OsmDataset? geography)
    {
        if (tiles.Count == 0) throw new InvalidOperationException("No terrain tiles were supplied.");
        var zoom = tiles[0].Zoom;
        if (tiles.Any(tile => tile.Zoom != zoom)) throw new InvalidOperationException("Terrain tile zoom levels do not match.");
        if (zoom < 0 || zoom > 22 || bounds.North <= bounds.South || bounds.East <= bounds.West ||
            new[] { bounds.North, bounds.South, bounds.East, bounds.West }.Any(v => double.IsNaN(v) || double.IsInfinity(v)))
            throw new InvalidOperationException("DEM mosaic: invalid bounds or zoom.");
        var minX = tiles.Min(t => t.X); var minY = tiles.Min(t => t.Y);
        var maxX = tiles.Max(t => t.X); var maxY = tiles.Max(t => t.Y);
        var mosaicWidth = checked((maxX - minX + 1) * 256); var mosaicHeight = checked((maxY - minY + 1) * 256);
        if ((long)mosaicWidth * mosaicHeight > 32_000_000) throw new InvalidOperationException("DEM mosaic: source buffer exceeds the 32-million-sample safety limit.");
        var source = new float[checked(mosaicWidth * mosaicHeight)];
        var seen = new HashSet<(int, int)>();
        foreach (var tile in tiles)
        {
            token.ThrowIfCancellationRequested();
            if (!seen.Add((tile.X, tile.Y)) || tile.Pixels.Length != 256 * 256 || tile.X < 0 || tile.Y < 0 || tile.X >= (1 << zoom) || tile.Y >= (1 << zoom))
                throw new InvalidOperationException($"DEM mosaic: duplicate/invalid tile {zoom}/{tile.X}/{tile.Y}.");
            for (var y = 0; y < 256; y++) for (var x = 0; x < 256; x++)
            {
                var color = tile.Pixels[(255 - y) * 256 + x];
                source[((tile.Y - minY) * 256 + y) * mosaicWidth + (tile.X - minX) * 256 + x] =
                    color.a == 0 ? float.NaN : color.r * 256f + color.g + color.b / 256f - 32768f;
            }
        }
        for (var y = minY; y <= maxY; y++) for (var x = minX; x <= maxX; x++)
            if (!seen.Contains((x, y))) throw new InvalidOperationException($"DEM mosaic: missing required tile {zoom}/{x}/{y}; refusing zero-filled coverage.");
        var sourceSpacing = (float)(2 * Math.PI * 6378137 * Math.Cos((bounds.North + bounds.South) / 2 * Math.PI / 180) / (256 * Math.Pow(2, zoom)));
        log?.Info($"Terrain conditioning: {tiles.Count} DEM tiles loaded, {mosaicWidth}×{mosaicHeight} source mosaic; source spacing approximately {sourceSpacing:0.0} m. No ocean NoData inference without a verified coastline mask.");
        var protection = geography == null ? null : TerrainGeographyGuard.Build(geography, zoom, minX, minY, mosaicWidth, mosaicHeight, token);
        log?.Info(protection == null ? "Terrain geography protection unavailable; no hydrology or shoreline inference performed." : "Mapped coasts, cliffs, water corridors/rims and quarry envelopes protected before filtering.");
        var conditioned = TerrainConditioner.Condition(source, mosaicWidth, mosaicHeight, sourceSpacing, token,
            protection == null ? null : new Func<int, bool>(i => protection[i]));
        log?.Info(conditioned.Metrics.ToString());
        var working = conditioned.Elevations;
        float ReadRaster(int x, int y)
        {
            x -= minX * 256; y -= minY * 256;
            if (x < 0 || y < 0 || x >= mosaicWidth || y >= mosaicHeight)
                throw new InvalidOperationException($"DEM resampling: missing required source coverage at {x},{y}.");
            return working[y * mosaicWidth + x];
        }
        var elevations = new float[Resolution * Resolution];

        var options = new ParallelOptions { MaxDegreeOfParallelism = 2, CancellationToken = token };
        System.Threading.Tasks.Parallel.For(0, Resolution, options, row =>
        {
            var latitude = bounds.South + (bounds.North - bounds.South) * row / (Resolution - 1d);
            var tileY = GeoMath.LatitudeToTileY(latitude, zoom);
            for (var column = 0; column < Resolution; column++)
            {
                var longitude = bounds.West + (bounds.East - bounds.West) * column / (Resolution - 1d);
                var tileX = GeoMath.LongitudeToTileX(longitude, zoom);
                var px = tileX * 256; var py = tileY * 256;
                var x = (int)Math.Floor(px); var y = (int)Math.Floor(py);
                var fx = (float)(px - x); var fy = (float)(py - y);
                var a = ReadRaster(x, y); var b = ReadRaster(x + 1, y); var c = ReadRaster(x, y + 1); var d = ReadRaster(x + 1, y + 1);
                // Convex bilinear interpolation cannot overshoot its source neighborhood.
                elevations[row * Resolution + column] = (a + (b - a) * fx) * (1 - fy) + (c + (d - c) * fx) * fy;
            }
        });

        var minimum = elevations.Min();
        var maximum = elevations.Max();
        if (float.IsNaN(minimum) || float.IsInfinity(minimum) || float.IsNaN(maximum) || float.IsInfinity(maximum) || maximum - minimum > 12_000f)
            throw new InvalidOperationException($"Terrain elevations were invalid ({minimum:0.0} to {maximum:0.0} m).");

        // CS2's editor imports normalized 16-bit values and applies a height scale in meters.
        // Reserve at least 10 m below real sea level and 10 m over the highest source point.
        var baseElevation = Math.Min(-10f, minimum - 10f);
        var scale = Math.Max(4096f, maximum - baseElevation + 10f);
        var seaLevel = -baseElevation;
        var output = new ushort[elevations.Length];
        System.Threading.Tasks.Parallel.For(0, elevations.Length, options, index =>
        {
            var normalized = (elevations[index] - baseElevation) / scale;
            if (!TerrainConditioner.Finite(normalized) || normalized < 0 || normalized > 1)
                throw new InvalidOperationException($"DEM encoding: elevation outside supported encoding at cell {index}; no silent clamping.");
            output[index] = (ushort)Math.Round(normalized * ushort.MaxValue);
        });
        token.ThrowIfCancellationRequested();
        log?.Info($"DEM resampling/encoding validation passed: {minimum:0.0}–{maximum:0.0} m; no interpolation overshoot or non-finite samples. Terrain sea encoding unchanged.");

        return new Result(output, scale, seaLevel, minimum, maximum) { ConditioningSummary = conditioned.Metrics.ToString() };
    }

    private static float SampleTerrarium(IReadOnlyDictionary<(int x, int y), TerrainTilePixels> tiles, double tileX, double tileY)
    {
        var x0 = (int)Math.Floor(tileX);
        var y0 = (int)Math.Floor(tileY);
        var px = (tileX - x0) * 256d;
        var py = (tileY - y0) * 256d;
        var ix = Numeric.Clamp((int)Math.Floor(px), 0, 255);
        var iy = Numeric.Clamp((int)Math.Floor(py), 0, 255);
        var fx = (float)(px - Math.Floor(px));
        var fy = (float)(py - Math.Floor(py));
        var a = Read(tiles, x0, y0, ix, iy);
        var b = Read(tiles, x0, y0, ix + 1, iy);
        var c = Read(tiles, x0, y0, ix, iy + 1);
        var d = Read(tiles, x0, y0, ix + 1, iy + 1);
        var top = a + (b - a) * fx;
        var bottom = c + (d - c) * fx;
        return top + (bottom - top) * fy;
    }

    private static float Read(IReadOnlyDictionary<(int x, int y), TerrainTilePixels> tiles, int tileX, int tileY, int pixelX, int pixelY)
    {
        if (pixelX > 255) { tileX++; pixelX = 0; }
        if (pixelY > 255) { tileY++; pixelY = 0; }
        if (!tiles.TryGetValue((tileX, tileY), out var tile))
            throw new InvalidOperationException($"Terrain mosaic is missing tile {tileX}/{tileY}.");
        // Unity's pixel array is bottom-up; web tiles are addressed top-down.
        var color = tile.Pixels[(255 - pixelY) * 256 + pixelX];
        return color.r * 256f + color.g + color.b / 256f - 32768f;
    }
}
