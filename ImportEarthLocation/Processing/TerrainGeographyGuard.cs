using System;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using ImportEarthLocation.Geo;

namespace ImportEarthLocation.Processing;

/// <summary>Conservative source-grid protection, not terrain shaping or water-level inference.</summary>
public static class TerrainGeographyGuard
{
    public static bool[] Build(OsmDataset data, int zoom, int minTileX, int minTileY, int width, int height, CancellationToken token)
    {
        var mask = new bool[checked(width * height)];
        foreach (var way in data.Ways.Values.OrderBy(w => w.Id))
        {
            token.ThrowIfCancellationRequested();
            way.Tags.TryGetValue("natural", out var natural); way.Tags.TryGetValue("landuse", out var landuse);
            way.Tags.TryGetValue("waterway", out var waterway);
            var area = natural == "water" || landuse == "reservoir" || landuse == "quarry";
            if (!area && natural != "coastline" && natural != "cliff" && natural != "beach" &&
                waterway != "river" && waterway != "stream" && waterway != "riverbank") continue;
            var points = way.NodeIds.Where(data.Nodes.ContainsKey).Select(id => data.Nodes[id]).Select(n => (
                x: GeoMath.LongitudeToTileX(n.Longitude, zoom) * 256 - minTileX * 256,
                y: GeoMath.LatitudeToTileY(n.Latitude, zoom) * 256 - minTileY * 256)).ToArray();
            if (points.Length == 0 || points.Any(p => double.IsNaN(p.x) || double.IsInfinity(p.x) || double.IsNaN(p.y) || double.IsInfinity(p.y))) continue;
            // Closed known water/quarry geometry protects its entire envelope (including rims).
            // Line segments protect two source cells on either side; do not infer coast from DEM height.
            if (area) Protect(points.Min(p => p.x), points.Min(p => p.y), points.Max(p => p.x), points.Max(p => p.y));
            else for (var i = 1; i < points.Length; i++) Protect(Math.Min(points[i - 1].x, points[i].x), Math.Min(points[i - 1].y, points[i].y), Math.Max(points[i - 1].x, points[i].x), Math.Max(points[i - 1].y, points[i].y));
        }
        return mask;

        void Protect(double left, double top, double right, double bottom)
        {
            if (right < -2 || bottom < -2 || left > width + 2 || top > height + 2) return;
            var x0 = (int)Math.Max(0, Math.Min(width - 1, Math.Floor(left) - 2));
            var y0 = (int)Math.Max(0, Math.Min(height - 1, Math.Floor(top) - 2));
            var x1 = (int)Math.Max(0, Math.Min(width - 1, Math.Ceiling(right) + 2));
            var y1 = (int)Math.Max(0, Math.Min(height - 1, Math.Ceiling(bottom) + 2));
            for (var y = y0; y <= y1; y++)
            { token.ThrowIfCancellationRequested(); for (var x = x0; x <= x1; x++) mask[y * width + x] = true; }
        }
    }
}
