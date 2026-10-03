using System;
using System.Diagnostics;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using System.Net.Http;
using System.Threading;
using System.Threading.Tasks;
using ImportEarthLocation.Core;
using ImportEarthLocation.Geo;

namespace ImportEarthLocation.Data;

public sealed class TerrainTileClient
{
    private static readonly string[] Templates =
    {
        "https://elevation-tiles-prod.s3.amazonaws.com/terrarium/{0}/{1}/{2}.png",
        "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{0}/{1}/{2}.png"
    };

    private readonly CacheStore _cache;
    private readonly ImportLog _log;

    public TerrainTileClient(CacheStore cache, ImportLog log)
    {
        _cache = cache;
        _log = log;
    }

    public async Task<IReadOnlyList<TerrainTile>> DownloadAsync(GeoBounds bounds, double widthMeters, CancellationToken token)
    {
        var timer = Stopwatch.StartNew();
        var zoom = GeoMath.SelectTerrainZoom(widthMeters);
        var minX = (int)Math.Floor(GeoMath.LongitudeToTileX(bounds.West, zoom));
        var maxX = (int)Math.Floor(GeoMath.LongitudeToTileX(bounds.East, zoom));
        var minY = (int)Math.Floor(GeoMath.LatitudeToTileY(bounds.North, zoom));
        var maxY = (int)Math.Floor(GeoMath.LatitudeToTileY(bounds.South, zoom));
        var coordinates = new List<(int x, int y)>();
        for (var y = minY; y <= maxY; y++)
        for (var x = minX; x <= maxX; x++)
            coordinates.Add((x, y));

        if (coordinates.Count is 0 or > 100)
            throw new InvalidOperationException($"Terrain plan requested {coordinates.Count} tiles; expected 1-100.");

        _log.Info($"Terrain plan: zoom {zoom}, {coordinates.Count} independently cached tiles.");
        using var gate = new SemaphoreSlim(4);
        var tasks = coordinates.Select(async coordinate =>
        {
            await gate.WaitAsync(token).ConfigureAwait(false);
            try { return await DownloadOneAsync(zoom, coordinate.x, coordinate.y, token).ConfigureAwait(false); }
            finally { gate.Release(); }
        });
        var result = await Task.WhenAll(tasks).ConfigureAwait(false);
        _log.Time("Terrain download", timer.Elapsed.TotalSeconds);
        return result;
    }

    private async Task<TerrainTile> DownloadOneAsync(int zoom, int x, int y, CancellationToken token)
    {
        var key = $"terrarium-v1:{zoom}:{x}:{y}";
        var cached = await _cache.ReadValidAsync("terrain", key, "png", IsPng, token).ConfigureAwait(false);
        if (cached != null) { _log.Count("Terrain cache hits"); return new TerrainTile { Zoom = zoom, X = x, Y = y, PngBytes = cached }; }

        Exception? last = null;
        for (var attempt = 0; attempt < 4; attempt++)
        {
            var url = string.Format(Templates[attempt % Templates.Length], zoom, x, y);
            try
            {
                using var response = await PublicHttpClient.Instance.GetAsync(url, token).ConfigureAwait(false);
                response.EnsureSuccessStatusCode();
                var bytes = await response.Content.ReadAsByteArrayAsync().ConfigureAwait(false);
                if (!IsPng(bytes)) throw new InvalidDataException("Terrain endpoint did not return a PNG.");
                await _cache.WriteAsync("terrain", key, "png", bytes, token).ConfigureAwait(false);
                _log.Count("Terrain downloaded tiles");
                return new TerrainTile { Zoom = zoom, X = x, Y = y, PngBytes = bytes };
            }
            catch (Exception exception) when (exception is not OperationCanceledException)
            {
                last = exception;
                _log.Warn($"Terrain tile {zoom}/{x}/{y} attempt {attempt + 1} failed: {exception.Message}");
                _log.Count("Terrain retries");
                await PublicHttpClient.DelayForRetryAsync(attempt, token).ConfigureAwait(false);
            }
        }
        throw new HttpRequestException($"Terrain tile {zoom}/{x}/{y} failed after retries.", last);
    }

    private static bool IsPng(byte[] bytes) =>
        bytes.Length > 8 && bytes[0] == 137 && bytes[1] == 80 && bytes[2] == 78 && bytes[3] == 71;
}
