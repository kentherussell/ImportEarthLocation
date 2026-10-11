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
    { public string ConditioningSummary { get; init; } = ""; public OceanMask? Ocean { get; init; } }

    public static IReadOnlyList<TerrainTilePixels> DecodeOnMainThread(IReadOnlyList<TerrainTile> tiles, CancellationToken token = default)
    {
        var decoded = new List<TerrainTilePixels>(tiles.Count);
        foreach (var tile in tiles)
        {
            token.ThrowIfCancellationRequested();
            var texture = new Texture2D(2, 2, TextureFormat.RGBA32, false, true);
            try
            {
                var usable = false;
                try { usable = ImageConversion.LoadImage(texture, tile.PngBytes, markNonReadable: false) && texture.width == 256 && texture.height == 256; }
                catch (Exception error) when (error is UnityException || error is ArgumentException)
                { /* Corrupt image data follows the same bounded retry path as NoData. */ }
                var pixels = usable ? texture.GetPixels32() : new Color32[256 * 256];
                float[]? recovered = null;
                if (tile.RecoveryParent != null)
                {
                    var parent = DecodeOnMainThread(new[] { tile.RecoveryParent }, token)[0];
                    recovered = RecoverFromParent(tile, pixels, parent.Pixels, token);
                }
                else if(tile.RecoveryPass>0)
                {
                    // Retry first. Only then interpolate isolated holes with two
                    // agreeing opposite-side slope estimates, never an empty tile.
                    var raw=new float[65536];for(var y=0;y<256;y++)for(var x=0;x<256;x++)raw[y*256+x]=Decode(pixels[(255-y)*256+x]);
                    if(TryRepairSmallHoles(raw,256,256,token)>0)recovered=raw;
                }
                decoded.Add(new TerrainTilePixels { Zoom=tile.Zoom, X=tile.X, Y=tile.Y, Pixels=pixels, RecoveredElevations=recovered, RecoveryPass=tile.RecoveryPass });
            }
            finally
            {
                UnityEngine.Object.Destroy(texture);
            }
        }
        return decoded;
    }

    public static bool ValidEarthElevation(float value) => TerrainConditioner.ValidEarthElevation(value);
    private static float Decode(Color32 color) => color.a == 0 ? float.NaN : color.r * 256f + color.g + color.b / 256f - 32768f;

    private static int TryRepairSmallHoles(float[] raster,int width,int height,CancellationToken token)
    {
        var invalid=new HashSet<int>(Enumerable.Range(0,raster.Length).Where(i=>!ValidEarthElevation(raster[i])));
        if(invalid.Count==0||invalid.Count>Math.Max(4,raster.Length/10000))return 0;
        var original=(float[])raster.Clone();var changed=0;
        foreach(var seed in invalid.ToArray())
        {
            if(!invalid.Remove(seed))continue;var cluster=new List<int>{seed};
            for(var cursor=0;cursor<cluster.Count;cursor++)
            {
                token.ThrowIfCancellationRequested();var i=cluster[cursor];var x=i%width;var y=i/width;
                foreach(var next in new[]{x>0?i-1:-1,x+1<width?i+1:-1,y>0?i-width:-1,y+1<height?i+width:-1})if(invalid.Remove(next))cluster.Add(next);
            }
            if(cluster.Count>4||cluster.Any(i=>i%width<5||i%width>=width-5||i/width<5||i/width>=height-5))continue;
            var replacements=new Dictionary<int,float>();
            foreach(var i in cluster)
            {
                var estimates=new List<float>();
                foreach(var stride in new[]{1,width})
                {
                    var left=1;var right=1;while(left<=5&&!ValidEarthElevation(original[i-left*stride]))left++;while(right<=5&&!ValidEarthElevation(original[i+right*stride]))right++;
                    if(left<=5&&right<=5)estimates.Add((original[i-left*stride]*right+original[i+right*stride]*left)/(left+right));
                }
                if(estimates.Count==2&&Math.Abs(estimates[0]-estimates[1])<=1)replacements[i]=estimates.Average();
            }
            if(replacements.Count!=cluster.Count)continue;
            foreach(var p in replacements){raster[p.Key]=p.Value;changed++;}
        }
        return changed;
    }

    public static IReadOnlyList<int> InvalidTiles(IReadOnlyList<TerrainTilePixels> tiles, GeoBounds bounds, ImportLog log, CancellationToken token)
    {
        var invalid = new List<int>();
        for (var i=0; i<tiles.Count; i++)
        {
            token.ThrowIfCancellationRequested();
            var tile=tiles[i]; var count=0; var example=-1;
            var left=Math.Max(0,(int)Math.Floor((GeoMath.LongitudeToTileX(bounds.West,tile.Zoom)-tile.X)*256)-3);
            var right=Math.Min(255,(int)Math.Ceiling((GeoMath.LongitudeToTileX(bounds.East,tile.Zoom)-tile.X)*256)+3);
            var top=Math.Max(0,(int)Math.Floor((GeoMath.LatitudeToTileY(bounds.North,tile.Zoom)-tile.Y)*256)-3);
            var bottom=Math.Min(255,(int)Math.Ceiling((GeoMath.LatitudeToTileY(bounds.South,tile.Zoom)-tile.Y)*256)+3);
            for(var y=top;y<=bottom;y++) for(var x=left;x<=right;x++)
            {
                var p=y*256+x;
                var value=tile.RecoveredElevations?[p] ?? Decode(tile.Pixels[(255-y)*256+x]);
                if(!ValidEarthElevation(value)){count++;example=p;}
            }
            if(count==0)continue;
            if (tile.RecoveryPass >= 2 && count <= 64)
            {
                log.Diagnostic($"DEM RECOVERY tile={tile.Zoom}/{tile.X}/{tile.Y}; requiredInvalidSamples={count}; action=defer-small-cluster-to-cross-tile-mosaic; zeroFill=false; boundedNeighborValidationRequired=true");
                continue;
            }
            invalid.Add(i);
            var c=tile.Pixels[(255-example/256)*256+example%256];
            log.Diagnostic($"DEM INVALID tile={tile.Zoom}/{tile.X}/{tile.Y}; requiredInvalidSamples={count}; examplePixel={example%256},{example/256}; rgba={c.r},{c.g},{c.b},{c.a}; decodedMeters={Decode(c):R}; recoverySamplesPresent={tile.RecoveredElevations!=null}");
        }
        return invalid;
    }

    private static float[] RecoverFromParent(TerrainTile tile, Color32[] pixels, Color32[] parent, CancellationToken token)
    {
        float Parent(int x,int y)
        {
            var px=(tile.X%2)*128+(x+.5f)/2-.5f; var py=(tile.Y%2)*128+(y+.5f)/2-.5f;
            var ix=Math.Max(0,Math.Min(254,(int)Math.Floor(px))); var iy=Math.Max(0,Math.Min(254,(int)Math.Floor(py)));
            var fx=Math.Max(0,Math.Min(1,px-ix)); var fy=Math.Max(0,Math.Min(1,py-iy));
            var a=Decode(parent[(255-iy)*256+ix]); var b=Decode(parent[(255-iy)*256+ix+1]);
            var c=Decode(parent[(254-iy)*256+ix]); var d=Decode(parent[(254-iy)*256+ix+1]);
            if(!ValidEarthElevation(a)||!ValidEarthElevation(b)||!ValidEarthElevation(c)||!ValidEarthElevation(d))return float.NaN;
            return (a+(b-a)*fx)*(1-fy)+(c+(d-c)*fx)*fy;
        }
        var overlap=new List<float>();
        for(var y=8;y<256;y+=16)for(var x=8;x<256;x+=16)
        {
            var fine=Decode(pixels[(255-y)*256+x]);var coarse=Parent(x,y);
            if(ValidEarthElevation(fine)&&ValidEarthElevation(coarse))overlap.Add(Math.Abs(fine-coarse));
        }
        overlap.Sort();
        // No fabricated vertical offset and no unchecked substitution across a
        // grossly inconsistent parent. Whole unreadable tiles can use a valid
        // parent; remaining seams are inspected by the terrain conditioner.
        if(overlap.Count>0&&(overlap[overlap.Count/2]>15||overlap[(overlap.Count-1)*95/100]>80))
            throw new PreparationDataException(PreparationDataIssue.ElevationUnavailable,$"Elevation parent for {tile.Zoom}/{tile.X}/{tile.Y} disagrees with valid overlap; safe recovery unavailable.");
        var result=new float[65536];
        for(var y=0;y<256;y++)
        {
            token.ThrowIfCancellationRequested();
            for(var x=0;x<256;x++){var h=Decode(pixels[(255-y)*256+x]);result[y*256+x]=ValidEarthElevation(h)?h:Parent(x,y);}
        }
        return result;
    }

    public static Task<Result> BuildAsync(IReadOnlyList<TerrainTilePixels> tiles, GeoBounds bounds, CancellationToken token = default, ImportLog? log = null, OsmDataset? geography = null) =>
        Task.Run(() =>
        {
            try { return Build(tiles, bounds, token, log, geography); }
            catch (OutOfMemoryException exception) { throw new PreparationDataException(PreparationDataIssue.ResourceLimit,"Terrain preparation: insufficient memory for validated source/working height buffers.",exception); }
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
        if ((long)mosaicWidth * mosaicHeight > 32_000_000) throw new PreparationDataException(PreparationDataIssue.ResourceLimit,"DEM mosaic: source buffer exceeds the 32-million-sample safety limit.");
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
                    tile.RecoveredElevations?[y * 256 + x] ?? Decode(color);
            }
        }
        for (var y = minY; y <= maxY; y++) for (var x = minX; x <= maxX; x++)
            if (!seen.Contains((x, y))) throw new InvalidOperationException($"DEM mosaic: missing required tile {zoom}/{x}/{y}; refusing zero-filled coverage.");
        var sourceSpacing = (float)(2 * Math.PI * 6378137 * Math.Cos((bounds.North + bounds.South) / 2 * Math.PI / 180) / (256 * Math.Pow(2, zoom)));
        // Only the requested map plus its sampling/filter halo is mandatory.
        // Invalid pixels elsewhere in a rectangular downloaded tile are not
        // evidence that the requested map is unusable.
        var cropLeft=Math.Max(0,(int)Math.Floor(GeoMath.LongitudeToTileX(bounds.West,zoom)*256)-minX*256-3);
        var cropTop=Math.Max(0,(int)Math.Floor(GeoMath.LatitudeToTileY(bounds.North,zoom)*256)-minY*256-3);
        var cropRight=Math.Min(mosaicWidth-1,(int)Math.Ceiling(GeoMath.LongitudeToTileX(bounds.East,zoom)*256)-minX*256+3);
        var cropBottom=Math.Min(mosaicHeight-1,(int)Math.Ceiling(GeoMath.LatitudeToTileY(bounds.South,zoom)*256)-minY*256+3);
        var requiredWidth=cropRight-cropLeft+1;var requiredHeight=cropBottom-cropTop+1;
        var required=new float[checked(requiredWidth*requiredHeight)];
        for(var row=0;row<requiredHeight;row++)Array.Copy(source,(row+cropTop)*mosaicWidth+cropLeft,required,row*requiredWidth,requiredWidth);
        var wholeProtection=geography==null?null:TerrainGeographyGuard.Build(geography,zoom,minX,minY,mosaicWidth,mosaicHeight,token,report:message=>log?.Diagnostic(message));
        bool[]? protection=null;
        if(wholeProtection!=null){protection=new bool[required.Length];for(var row=0;row<requiredHeight;row++)Array.Copy(wholeProtection,(row+cropTop)*mosaicWidth+cropLeft,protection,row*requiredWidth,requiredWidth);}
        var landformProtection = geography == null ? null : TerrainGeographyGuard.Build(geography, zoom, minX, minY, mosaicWidth, mosaicHeight, token, landformsOnly: true,report:message=>log?.Diagnostic(message));
        CoastalTerrain.CoastCoverage? coastCoverage=null;
        TerrainArtifactRepair.Trace(required, requiredWidth, "source-before-coastal", message => log?.Diagnostic(message));
        CoastalTerrain.Shape(required, requiredWidth, requiredHeight, zoom, minX * 256 + cropLeft, minY * 256 + cropTop,
            sourceSpacing, geography, token, message => log?.Diagnostic(message),coverage=>coastCoverage=coverage);
        TerrainArtifactRepair.Trace(required, requiredWidth, "after-coastal", message => log?.Diagnostic(message));
        TerrainArtifactRepair.Repair(required, requiredWidth, requiredHeight, sourceSpacing, token, message => log?.Diagnostic(message),
            landformProtection == null ? null : new Func<int, bool>(i => landformProtection[(i / requiredWidth + cropTop) * mosaicWidth + i % requiredWidth + cropLeft]));
        // Keep the original local support scale before the broader rounded-
        // artifact pass; a distant hill must not hide a small local spike.
        TerrainArtifactRepair.Repair(required, requiredWidth, requiredHeight, sourceSpacing, token, message => log?.Diagnostic(message),
            landformProtection == null ? null : new Func<int, bool>(i => landformProtection[(i / requiredWidth + cropTop) * mosaicWidth + i % requiredWidth + cropLeft]), supportMeters: 360);
        TerrainArtifactRepair.Trace(required, requiredWidth, "after-artifact-repair", message => log?.Diagnostic(message));
        log?.Diagnostic($"DEM REQUIRED footprint={requiredWidth}x{requiredHeight}; mosaic={mosaicWidth}x{mosaicHeight}; globalPixelOrigin={minX * 256 + cropLeft},{minY * 256 + cropTop}; zoom={zoom}; recoveredTiles={tiles.Count(t=>t.RecoveredElevations!=null)}; discardedOutsideSamples={source.Length-required.Length}");
        log?.Info($"Terrain conditioning: {tiles.Count} DEM tiles loaded, {mosaicWidth}×{mosaicHeight} source mosaic; source spacing approximately {sourceSpacing:0.0} m. No ocean NoData inference without a verified coastline mask.");
        log?.Diagnostic("PROCESS DEM geography protection begin; mapped coasts/water/cliffs");
        log?.Info(protection == null ? "Terrain geography protection unavailable; no hydrology or shoreline inference performed." : "Mapped coasts, cliffs, water corridors/rims and quarry envelopes protected before filtering.");
        var conditioned = TerrainConditioner.Condition(required, requiredWidth, requiredHeight, sourceSpacing, token,
            protection == null ? null : new Func<int, bool>(i => protection[i]), progress: message => log?.Diagnostic("PROCESS DEM " + message),
            sourceOriginX: cropLeft, sourceOriginY: cropTop);
        log?.Info(conditioned.Metrics.ToString());
        var working = conditioned.Elevations;
        float ReadRaster(int x, int y)
        {
            x -= minX * 256 + cropLeft; y -= minY * 256 + cropTop;
            if (x < 0 || y < 0 || x >= requiredWidth || y >= requiredHeight)
                throw new InvalidOperationException($"DEM resampling: missing required source coverage at {x},{y}.");
            return working[y * requiredWidth + x];
        }
        var elevations = new float[Resolution * Resolution];
        log?.Diagnostic($"PROCESS DEM resampling begin; outputResolution={Resolution}; outputSamples={elevations.Length}");

        var options = new ParallelOptions { MaxDegreeOfParallelism = 2, CancellationToken = token };
        // Longitude sampling is identical on every row. Preserve the exact formula
        // while doing its tile conversion once per column, not once per terrain cell.
        var columns = new (int x, float fraction)[Resolution];
        for (var column = 0; column < Resolution; column++)
        {
            token.ThrowIfCancellationRequested();
            var longitude = bounds.West + (bounds.East - bounds.West) * column / (Resolution - 1d);
            var px = GeoMath.LongitudeToTileX(longitude, zoom) * 256;
            columns[column] = ((int)Math.Floor(px), (float)(px - Math.Floor(px)));
        }
        System.Threading.Tasks.Parallel.For(0, Resolution, options, row =>
        {
            var latitude = bounds.South + (bounds.North - bounds.South) * row / (Resolution - 1d);
            var tileY = GeoMath.LatitudeToTileY(latitude, zoom);
            var py = tileY * 256; var y = (int)Math.Floor(py); var fy = (float)(py - y);
            for (var column = 0; column < Resolution; column++)
            {
                var x = columns[column].x; var fx = columns[column].fraction;
                var a = ReadRaster(x, y); var b = ReadRaster(x + 1, y); var c = ReadRaster(x, y + 1); var d = ReadRaster(x + 1, y + 1);
                // Convex bilinear interpolation cannot overshoot its source neighborhood.
                elevations[row * Resolution + column] = (a + (b - a) * fx) * (1 - fy) + (c + (d - c) * fx) * fy;
            }
        });

        var minimum = elevations.Min();
        var maximum = elevations.Max();
        // CS2's editor imports normalized 16-bit values and applies a height scale in meters.
        // Retain validated source relief and 128m underground headroom rather
        // than rejecting a representable deep-water/land combination.
        var encoding=TerrainConditioner.EncodingFor(minimum,maximum);
        var baseElevation=encoding.BaseElevation;var scale=encoding.Scale;var seaLevel=encoding.SeaLevel;
        log?.Diagnostic($"DEM ENCODING boundsAccepted=true; minimum={minimum:R}; maximum={maximum:R}; span={maximum-minimum:R}; scale={scale:R}; precisionMeters={scale/65535:R}; wideSpan={maximum-minimum>12000}; modeledCoastMayReplaceUnverifiedSourceDepth=true; bathymetryAccuracyNotVerified=true");
        if(maximum-minimum>12000)
        {
            var index=Array.IndexOf(elevations,minimum);var row=index/Resolution;var column=index%Resolution;
            log?.Diagnostic($"DEM EXTREME minimumRasterCell={column},{row}; latitude={bounds.South+(bounds.North-bounds.South)*row/(Resolution-1d):R}; longitude={bounds.West+(bounds.East-bounds.West)*column/(Resolution-1d):R}; zoom={zoom}; sourceTiles={tiles.Count}; validAbsoluteBounds=true; localDepthAccuracyUnknown=true");
        }
        var output = new ushort[elevations.Length];
        System.Threading.Tasks.Parallel.For(0, elevations.Length, options, index =>
        {
            var normalized = (elevations[index] - baseElevation) / scale;
            if (!TerrainConditioner.Finite(normalized) || normalized < 0 || normalized > 1)
                throw new InvalidOperationException($"DEM encoding: elevation outside supported encoding at cell {index}; no silent clamping.");
            output[index] = (ushort)Math.Round(normalized * ushort.MaxValue);
        });
        token.ThrowIfCancellationRequested();
        log?.Info($"DEM resampling/encoding validation passed: {minimum:0.0}–{maximum:0.0} m; Earth datum offset {seaLevel:0.0} m, quantization {scale/65535:0.###} m, underground headroom >=128 m; no interpolation overshoot or non-finite samples.");

        OceanMask? ocean=null;
        if(coastCoverage!=null)
        {
            const int maskSize=512;var labels=new byte[maskSize*maskSize];var accepted=0;var unknown=0;
            for(var row=0;row<maskSize;row++)for(var column=0;column<maskSize;column++)
            {
                token.ThrowIfCancellationRequested();var latitude=bounds.South+(bounds.North-bounds.South)*row/(maskSize-1d);
                var longitude=bounds.West+(bounds.East-bounds.West)*column/(maskSize-1d);
                var x=(int)Math.Floor(GeoMath.LongitudeToTileX(longitude,zoom)*256)-coastCoverage.OriginX;
                var y=(int)Math.Floor(GeoMath.LatitudeToTileY(latitude,zoom)*256)-coastCoverage.OriginY;
                byte value=0;
                if(x<0||y<0||x+1>=coastCoverage.Width||y+1>=coastCoverage.Height)value=3;
                else
                {
                    var allSea=true;var conflicted=false;var unseeded=false;
                    for(var dz=0;dz<=1;dz++)for(var dx=0;dx<=1;dx++)
                    {var index=(y+dz)*coastCoverage.Width+x+dx;allSea&=coastCoverage.Ocean[index];conflicted|=coastCoverage.Ambiguous[index];unseeded|=coastCoverage.Unseeded[index];}
                    value=allSea?(byte)1:conflicted?(byte)2:unseeded?(byte)3:(byte)0;
                }
                labels[row*maskSize+column]=value;if(value==1)accepted++;if(value>=2)unknown++;
            }
            ocean=new OceanMask(maskSize,requestWidth(bounds),labels,"directed-coast-v2");
            log?.Diagnostic($"OCEAN HANDOFF resolution={maskSize}; acceptedSamples={accepted}; unknownSamples={unknown}; rowOrientation=south-to-north; registration=corner-aligned; classificationIsNotNativeNavigability=true");
        }
        return new Result(output, scale, seaLevel, minimum, maximum) { ConditioningSummary = conditioned.Metrics.ToString(),Ocean=ocean };

        double requestWidth(GeoBounds b)=>2*Math.PI*6378137*Math.Cos((b.North+b.South)/2*Math.PI/180)*(b.East-b.West)/360;
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
