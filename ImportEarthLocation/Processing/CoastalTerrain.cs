using System;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using ImportEarthLocation.Geo;

namespace ImportEarthLocation.Processing;

public static class CoastalTerrain
{
    // OSM coastline direction: land on left, sea on right. Missing or contradictory
    // coast coverage is not permission to flood-fill land from negative DEM height.
    public static void Shape(float[] heights, int width, int height, int zoom, int originX, int originY,
        float spacing, OsmDataset? geography, CancellationToken token, Action<string>? report)
    {
        if (geography == null) return;
        var barrier = new bool[heights.Length]; var seeds = new byte[heights.Length];
        var seedDistance = new double[heights.Length];
        for (var i = 0; i < seedDistance.Length; i++) seedDistance[i] = double.PositiveInfinity;
        var coastEdges = 0;
        foreach (var way in geography.Ways.Values.Where(w => w.Tags.TryGetValue("natural", out var value) && value == "coastline"))
        {
            for (var k = 1; k < way.NodeIds.Length; k++)
            {
                token.ThrowIfCancellationRequested();
                if (!geography.Nodes.TryGetValue(way.NodeIds[k - 1], out var a) || !geography.Nodes.TryGetValue(way.NodeIds[k], out var b)) continue;
                var ax = GeoMath.LongitudeToTileX(a.Longitude, zoom) * 256 - originX;
                var ay = GeoMath.LatitudeToTileY(a.Latitude, zoom) * 256 - originY;
                var bx = GeoMath.LongitudeToTileX(b.Longitude, zoom) * 256 - originX;
                var by = GeoMath.LatitudeToTileY(b.Latitude, zoom) * 256 - originY;
                var dx = bx - ax; var dy = by - ay; var length = Math.Sqrt(dx * dx + dy * dy);
                if (!TerrainConditioner.Finite((float)length) || length < .001 || Math.Max(ax, bx) < -5 || Math.Min(ax, bx) > width + 5 || Math.Max(ay, by) < -5 || Math.Min(ay, by) > height + 5) continue;
                if (length > 100000) continue; // Malformed geography cannot create an unbounded loop.
                coastEdges++;
                var count = (int)Math.Ceiling(length * 2);
                for (var j = 0; j <= count; j++)
                {
                    if ((j & 1023) == 0) token.ThrowIfCancellationRequested();
                    var x = ax + dx * j / count; var y = ay + dy * j / count;
                    var ix = (int)Math.Round(x); var iy = (int)Math.Round(y);
                    for (var yy = iy - 1; yy <= iy + 1; yy++) for (var xx = ix - 1; xx <= ix + 1; xx++)
                        if (xx >= 0 && xx < width && yy >= 0 && yy < height) barrier[yy * width + xx] = true;
                    Mark(x - dy / length * 4, y + dx / length * 4, 1, ax, ay, bx, by); // Sea, right in image coordinates.
                    Mark(x + dy / length * 4, y - dx / length * 4, 2, ax, ay, bx, by); // Land.
                }
            }
        }
        if (coastEdges == 0) return;
        var excluded = new bool[heights.Length];
        foreach (var way in geography.Ways.Values)
        {
            token.ThrowIfCancellationRequested();
            if (!(way.Tags.TryGetValue("natural", out var natural) && natural == "water" || way.Tags.TryGetValue("landuse", out var landuse) && landuse == "reservoir")) continue;
            if (way.Tags.TryGetValue("water", out var water) && (water == "sea" || water == "bay")) continue;
            if (way.NodeIds.Length < 4 || way.NodeIds[0] != way.NodeIds[way.NodeIds.Length - 1]) continue;
            ExcludeEnvelope(way.NodeIds);
        }
        var protectedRelations = 0;
        foreach (var relation in geography.Relations)
        {
            token.ThrowIfCancellationRequested();
            if (!(relation.Tags.TryGetValue("natural", out var natural) && natural == "water" || relation.Tags.TryGetValue("landuse", out var landuse) && landuse == "reservoir")) continue;
            if (relation.Tags.TryGetValue("water", out var water) && (water == "sea" || water == "bay")) continue;
            // Multipolygon tags belong to the relation; outer ways may be open and
            // untagged. Protect their combined envelope, including the interior.
            var outerNodes = relation.Members.Where(m => !m.inner && geography.Ways.ContainsKey(m.wayId))
                .SelectMany(m => geography.Ways[m.wayId].NodeIds);
            ExcludeEnvelope(outerNodes); protectedRelations++;
        }
        void ExcludeEnvelope(System.Collections.Generic.IEnumerable<long> nodeIds)
        {
            var points = nodeIds.Where(geography.Nodes.ContainsKey).Select(id => geography.Nodes[id])
                .Where(p => !double.IsNaN(p.Longitude) && !double.IsInfinity(p.Longitude) && !double.IsNaN(p.Latitude) && !double.IsInfinity(p.Latitude)).ToArray();
            if (points.Length < 3) return;
            var left = Math.Max(0, (int)Math.Floor(points.Min(p => GeoMath.LongitudeToTileX(p.Longitude, zoom) * 256 - originX)));
            var right = Math.Min(width - 1, (int)Math.Ceiling(points.Max(p => GeoMath.LongitudeToTileX(p.Longitude, zoom) * 256 - originX)));
            var top = Math.Max(0, (int)Math.Floor(points.Min(p => GeoMath.LatitudeToTileY(p.Latitude, zoom) * 256 - originY)));
            var bottom = Math.Min(height - 1, (int)Math.Ceiling(points.Max(p => GeoMath.LatitudeToTileY(p.Latitude, zoom) * 256 - originY)));
            for (var y = top; y <= bottom; y++) { token.ThrowIfCancellationRequested(); for (var x = left; x <= right; x++) excluded[y * width + x] = true; }
        }
        var visited = new bool[heights.Length]; var ocean = new bool[heights.Length]; var queue = new int[heights.Length];
        var accepted = 0; var ambiguous = 0;
        for (var seed = 0; seed < heights.Length; seed++)
        {
            if ((seed & 16383) == 0) token.ThrowIfCancellationRequested();
            if (visited[seed] || barrier[seed]) continue;
            var head = 0; var count = 1; byte labels = 0; queue[0] = seed; visited[seed] = true;
            while (head < count)
            {
                if ((head & 4095) == 0) token.ThrowIfCancellationRequested();
                var i = queue[head++]; labels |= seeds[i]; var x = i % width; var y = i / width;
                Add(x > 0 ? i - 1 : -1); Add(x + 1 < width ? i + 1 : -1); Add(y > 0 ? i - width : -1); Add(y + 1 < height ? i + width : -1);
                void Add(int next) { if (next < 0 || visited[next] || barrier[next]) return; visited[next] = true; queue[count++] = next; }
            }
            if (labels == 3) ambiguous++;
            if (labels != 1) continue;
            accepted++; for (var k = 0; k < count; k++) ocean[queue[k]] = true;
        }
        // Two-pass shoreline distance. No artificial slope toward the map's crop edge.
        var distance = new float[heights.Length];
        for (var i = 0; i < distance.Length; i++) distance[i] = !ocean[i] ? 0 : float.PositiveInfinity;
        var diagonal = spacing * 1.41421356f;
        for (var y = 0; y < height; y++)
        {
            token.ThrowIfCancellationRequested();
            for (var x = 0; x < width; x++)
            {
                var i = y * width + x;
                if (x > 0) distance[i] = Math.Min(distance[i], distance[i - 1] + spacing);
                if (y > 0)
                {
                    distance[i] = Math.Min(distance[i], distance[i - width] + spacing);
                    if (x > 0) distance[i] = Math.Min(distance[i], distance[i - width - 1] + diagonal);
                    if (x + 1 < width) distance[i] = Math.Min(distance[i], distance[i - width + 1] + diagonal);
                }
            }
        }
        var changed = 0;
        for (var y = height - 1; y >= 0; y--)
        {
            token.ThrowIfCancellationRequested();
            for (var x = width - 1; x >= 0; x--)
            {
                var i = y * width + x;
                if (x + 1 < width) distance[i] = Math.Min(distance[i], distance[i + 1] + spacing);
                if (y + 1 < height) distance[i] = Math.Min(distance[i], distance[i + width] + spacing);
                if (y + 1 < height && x > 0) distance[i] = Math.Min(distance[i], distance[i + width - 1] + diagonal);
                if (y + 1 < height && x + 1 < width) distance[i] = Math.Min(distance[i], distance[i + width + 1] + diagonal);
                if (!ocean[i] || excluded[i] || !TerrainConditioner.Finite(distance[i])) continue;
                var target = -Math.Min(180, .5f + distance[i] * .015f);
                // Terrarium is not verified bathymetry here. A modeled ocean must
                // not retain a fraction of an 11km corrupt floor at its coastline.
                heights[i] = target; changed++;
            }
        }
        var coastalRimRepairs = 0;
        for (var y = 3; y < height - 3; y++)
        {
            token.ThrowIfCancellationRequested();
            for (var x = 3; x < width - 3; x++)
            {
                var i = y * width + x;
                if (!barrier[i] || excluded[i] || heights[i] >= -.5f) continue;
                var seaNeighbor = false;
                for (var dy = -3; dy <= 3 && !seaNeighbor; dy++) for (var dx = -3; dx <= 3; dx++)
                    if (ocean[(y + dy) * width + x + dx]) { seaNeighbor = true; break; }
                if (!seaNeighbor) continue;
                heights[i] = -.5f; coastalRimRepairs++;
            }
        }
        report?.Invoke($"DEM COAST model=oriented-coastline-offshore-profile; edges={coastEdges}; oceanComponents={accepted}; conflictingComponentsSkipped={ambiguous}; modeledSamples={changed}; coastalRimSamples={coastalRimRepairs}; maximumModeledDepthMeters=180; depthSlope=.015; distance=8-neighbor-chamfer; seedPolicy=nearest-shore-segment; mappedInlandWaterEnvelopesProtected=true; inlandWaterRelations={protectedRelations}; nonOceanLandPreserved=true; incompleteCoastCoverageMayLimitRepair=true; realBathymetryClaimed=false");
        void Mark(double x, double y, byte label, double ax, double ay, double bx, double by)
        {
            var ix = (int)Math.Round(x); var iy = (int)Math.Round(y);
            if (ix < 0 || ix >= width || iy < 0 || iy >= height) return;
            var dx = bx - ax; var dy = by - ay;
            var t = Math.Max(0, Math.Min(1, ((ix - ax) * dx + (iy - ay) * dy) / (dx * dx + dy * dy)));
            var ex = ix - ax - t * dx; var ey = iy - ay - t * dy; var squared = ex * ex + ey * ey;
            var i = iy * width + ix;
            // Offset seeds can overlap on tight bends/inlets. A more distant
            // opposite shoreline must not poison an entire connected ocean.
            if (squared + 1e-6 < seedDistance[i]) { seedDistance[i] = squared; seeds[i] = label; }
            else if (Math.Abs(squared - seedDistance[i]) <= 1e-6) seeds[i] |= label;
        }
    }
}
