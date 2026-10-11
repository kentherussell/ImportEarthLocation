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
    // 256 decoded 256x256 tiles are at most one 4096x4096 output raster's
    // sample count. Keep the preferred detail for the observed 121-tile map.
    private const int MaximumTiles=256;
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
        var preferredZoom=zoom;
        bool Finite(double value)=>!double.IsNaN(value)&&!double.IsInfinity(value);
        if(!Finite(bounds.West)||!Finite(bounds.East)||!Finite(bounds.North)||!Finite(bounds.South)||
            bounds.West>=bounds.East||bounds.South>=bounds.North)
            throw new PreparationDataException(PreparationDataIssue.ElevationUnavailable,"DEM bounds are non-finite or inverted; no recoverable tile footprint exists.");
        // Conditioning and bilinear resampling need real samples beyond the
        // requested boundary, including an exact tile-edge endpoint.
        (int minX,int maxX,int minY,int maxY) Footprint(int level)=>
            (Math.Max(0,(int)Math.Floor(GeoMath.LongitudeToTileX(bounds.West,level)-6d/256)),
             Math.Min((1<<level)-1,(int)Math.Floor(GeoMath.LongitudeToTileX(bounds.East,level)+6d/256)),
             Math.Max(0,(int)Math.Floor(GeoMath.LatitudeToTileY(bounds.North,level)-6d/256)),
             Math.Min((1<<level)-1,(int)Math.Floor(GeoMath.LatitudeToTileY(bounds.South,level)+6d/256)));
        var footprint=Footprint(zoom);
        long Count()=>Math.Max(0L,footprint.maxX-footprint.minX+1L)*Math.Max(0L,footprint.maxY-footprint.minY+1L);
        var preferredCount=Count();
        // Match useful output detail when the SAME provider can fit the entire
        // footprint. Budget includes coast masks, repair clones, decoded tiles and
        // fixed output buffers; finer pixels are not a claim of finer source survey.
        const long scratchBudgetBytes=768L*1024*1024;
        var latitude=(bounds.North+bounds.South)*.5;
        var outputSpacing=widthMeters/4095d;
        double Spacing(int level)=>40075016.68557849*Math.Cos(latitude*Math.PI/180)/(256d*(1<<level));
        long EstimatedBytes((int minX,int maxX,int minY,int maxY) f)=>
            Math.Max(0L,f.maxX-f.minX+1L)*Math.Max(0L,f.maxY-f.minY+1L)*256L*256*40+96L*1024*1024;
        while(zoom<15 && Spacing(zoom)>outputSpacing*1.5)
        {
            var finer=Footprint(zoom+1);
            var finerCount=Math.Max(0L,finer.maxX-finer.minX+1L)*Math.Max(0L,finer.maxY-finer.minY+1L);
            if(finerCount>MaximumTiles || EstimatedBytes(finer)>scratchBudgetBytes) break;
            footprint=finer; zoom++;
        }
        // Mercator ground resolution changes with latitude. Width-only zoom
        // selection must not turn a valid map into an arbitrary 100-tile error.
        // Retain the real boundary halo and choose the next same-provider level.
        while(Count()>MaximumTiles&&zoom>0){token.ThrowIfCancellationRequested();footprint=Footprint(--zoom);}
        _log.Diagnostic($"DEM PLAN preferredZoom={preferredZoom}; preferredTiles={preferredCount}; selectedZoom={zoom}; selectedTiles={Count()}; maximumTiles={MaximumTiles}; zoomReduced={zoom<preferredZoom}; zoomIncreased={zoom>preferredZoom}; sourceSpacingMeters={Spacing(zoom):R}; outputSpacingMeters={outputSpacing:R}; estimatedScratchBytes={EstimatedBytes(footprint)}; scratchBudgetBytes={scratchBudgetBytes}; memoryEstimateNotMeasurement=true; sourceSurveyResolutionUnknown=true; boundaryHaloPixels=6; wholeBoundsRetained=true; newEndpoint=false");
        var (minX,maxX,minY,maxY)=footprint;
        var coordinates = new List<(int x, int y)>();
        for (var y = minY; y <= maxY; y++)
        for (var x = minX; x <= maxX; x++)
            coordinates.Add((x, y));

        if (coordinates.Count==0||coordinates.Count>MaximumTiles)
            throw new PreparationDataException(PreparationDataIssue.ElevationUnavailable,$"DEM bounds have no recoverable bounded tile footprint ({coordinates.Count} tiles at zoom {zoom}).");

        _log.Info($"Terrain plan: zoom {zoom}, {coordinates.Count} independently cached tiles.");
        using var gate = new SemaphoreSlim(4);
        var tasks = coordinates.Select(async coordinate =>
        {
            await gate.WaitAsync(token).ConfigureAwait(false);
            try
            {
                try { return await DownloadOneAsync(zoom, coordinate.x, coordinate.y, token).ConfigureAwait(false); }
                catch(Exception error) when(error is HttpRequestException||error is InvalidDataException)
                {
                    if(zoom==0)throw new PreparationDataException(PreparationDataIssue.ElevationUnavailable,$"DEM tile {zoom}/{coordinate.x}/{coordinate.y}: retries exhausted and no parent exists.",error);
                    _log.Diagnostic($"DEM RECOVERY tile={zoom}/{coordinate.x}/{coordinate.y}; action=unavailable-fine-tile-parent; overlapValidationRequired=true; newEndpoint=false");
                    TerrainTile parent;
                    try { parent=await DownloadOneAsync(zoom-1,coordinate.x/2,coordinate.y/2,token).ConfigureAwait(false); }
                    catch(Exception parentError) when(parentError is HttpRequestException||parentError is InvalidDataException)
                    { throw new PreparationDataException(PreparationDataIssue.ElevationUnavailable,$"DEM tile {zoom}/{coordinate.x}/{coordinate.y}: both fine and parent acquisition exhausted their retry budgets.",parentError); }
                    // Empty bytes decode as missing, never zero. The normal
                    // decoder/semantic validation must accept the parent first.
                    return new TerrainTile{Zoom=zoom,X=coordinate.x,Y=coordinate.y,PngBytes=Array.Empty<byte>(),RecoveryPass=2,RecoveryParent=parent};
                }
            }
            finally { gate.Release(); }
        });
        var result = await Task.WhenAll(tasks).ConfigureAwait(false);
        _log.Time("Terrain download", timer.Elapsed.TotalSeconds);
        return result;
    }

    public async Task<IReadOnlyList<TerrainTile>> RecoverAsync(IReadOnlyList<TerrainTile> tiles, IReadOnlyList<int> invalid, CancellationToken token)
    {
        var result = tiles.ToArray();
        foreach (var index in invalid)
        {
            token.ThrowIfCancellationRequested();
            var tile = tiles[index];
            if (tile.RecoveryPass == 0)
            {
                try{await _cache.QuarantineAsync("terrain",$"terrarium-v1:{tile.Zoom}:{tile.X}:{tile.Y}","png",token).ConfigureAwait(false);}
                catch(Exception error) when(error is IOException||error is UnauthorizedAccessException)
                {_log.Diagnostic($"DEM CACHE quarantine-unavailable tile={tile.Zoom}/{tile.X}/{tile.Y}; reason={error.GetType().Name}; uncachedRetryStillAllowed=true");}
                try
                {
                    var replacement = await DownloadOneAsync(tile.Zoom, tile.X, tile.Y, token, bypassCache: true).ConfigureAwait(false);
                    result[index] = new TerrainTile { Zoom=tile.Zoom, X=tile.X, Y=tile.Y, PngBytes=replacement.PngBytes, RecoveryPass=1 };
                    _log.Diagnostic($"DEM RECOVERY tile={tile.Zoom}/{tile.X}/{tile.Y}; action=uncached-retry; newEndpoint=false");
                }
                catch(Exception error) when(error is HttpRequestException||error is InvalidDataException)
                {
                    result[index]=new TerrainTile{Zoom=tile.Zoom,X=tile.X,Y=tile.Y,PngBytes=tile.PngBytes,RecoveryPass=1};
                    _log.Diagnostic($"DEM RECOVERY tile={tile.Zoom}/{tile.X}/{tile.Y}; action=retry-unavailable; next=same-dataset-parent; reason={error.GetType().Name}; noInvalidElevationAccepted=true");
                }
            }
            else if (tile.RecoveryPass == 1 && tile.Zoom > 0)
            {
                TerrainTile parent;
                try { parent=await DownloadOneAsync(tile.Zoom-1,tile.X/2,tile.Y/2,token).ConfigureAwait(false); }
                catch(Exception error) when(error is HttpRequestException||error is InvalidDataException)
                { throw new PreparationDataException(PreparationDataIssue.ElevationUnavailable,$"DEM tile {tile.Zoom}/{tile.X}/{tile.Y}: parent acquisition exhausted its retry budget.",error); }
                result[index] = new TerrainTile { Zoom=tile.Zoom, X=tile.X, Y=tile.Y, PngBytes=tile.PngBytes, RecoveryPass=2, RecoveryParent=parent };
                _log.Diagnostic($"DEM RECOVERY tile={tile.Zoom}/{tile.X}/{tile.Y}; action=same-dataset-parent; parent={parent.Zoom}/{parent.X}/{parent.Y}; overlapValidationRequired=true");
            }
            else throw new PreparationDataException(PreparationDataIssue.ElevationUnavailable,$"Elevation data for {tile.Zoom}/{tile.X}/{tile.Y} could not be recovered safely after retry/interpolation/parent validation.");
        }
        return result;
    }

    private async Task<TerrainTile> DownloadOneAsync(int zoom, int x, int y, CancellationToken token, bool bypassCache = false)
    {
        var key = $"terrarium-v1:{zoom}:{x}:{y}";
        var cached = bypassCache ? null : await _cache.ReadValidAsync("terrain", key, "png", IsPng, token).ConfigureAwait(false);
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
                token.ThrowIfCancellationRequested();
                if (!IsPng(bytes)) throw new InvalidDataException("Terrain endpoint did not return a PNG.");
                // A semantic retry is not allowed to overwrite the cached tile
                // until the game-thread decoder has validated its elevations.
                if (!bypassCache) await _cache.WriteAsync("terrain", key, "png", bytes, token).ConfigureAwait(false);
                _log.Count("Terrain downloaded tiles");
                return new TerrainTile { Zoom = zoom, X = x, Y = y, PngBytes = bytes };
            }
            catch (Exception exception) when (!token.IsCancellationRequested &&
                (exception is HttpRequestException || exception is InvalidDataException || exception is OperationCanceledException))
            {
                last = exception;
                var reason=exception is OperationCanceledException?"HTTP client timed out; retrying the elevation tile.":exception.Message;
                _log.Warn($"Terrain tile {zoom}/{x}/{y} attempt {attempt + 1} failed: {reason}");
                _log.Count("Terrain retries");
                await PublicHttpClient.DelayForRetryAsync(attempt, token).ConfigureAwait(false);
            }
        }
        throw new HttpRequestException($"Terrain tile {zoom}/{x}/{y} failed after retries.", last);
    }

    private static bool IsPng(byte[] bytes) =>
        bytes.Length > 8 && bytes[0] == 137 && bytes[1] == 80 && bytes[2] == 78 && bytes[3] == 71;
}
