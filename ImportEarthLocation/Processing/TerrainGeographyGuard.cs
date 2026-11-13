using System;
using System.IO;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Geo;

namespace ImportEarthLocation.Processing;

/// <summary>Conservative source-grid protection, not terrain shaping or water-level inference.</summary>
public static class TerrainGeographyGuard
{
    public static bool[] Build(
        OsmDataset data,
        int zoom,
        int minTileX,
        int minTileY,
        int width,
        int height,
        CancellationToken token,
        bool landformsOnly = false,
        Action<string>? report = null)
    {
        var mask = new bool[checked(width * height)];
        var holesByOuter =
            new System.Collections.Generic.Dictionary<long, System.Collections.Generic.List<(double x, double y)[]>>();

        foreach (var relation in data.Relations)
        {
            token.ThrowIfCancellationRequested();

            if (!relation.Members.Any(item => item.inner))
                continue;

            var holes = Rings(relation.Members.Where(item => item.inner).Select(item => item.wayId));

            if (holes == null)
                continue;

            foreach (var member in relation.Members.Where(item => !item.inner))
            {
                if (!holesByOuter.TryGetValue(member.wayId, out var existing))
                    holesByOuter[member.wayId] =
                        existing = new System.Collections.Generic.List<(double x, double y)[]>();

                existing.AddRange(holes);
            }
        }

        foreach (var way in data.Ways.Values.OrderBy(way => way.Id))
        {
            token.ThrowIfCancellationRequested();
            way.Tags.TryGetValue("natural", out var natural);
            way.Tags.TryGetValue("landuse", out var landuse);
            way.Tags.TryGetValue("waterway", out var waterway);
            var area = natural == "water" || landuse == "reservoir" || landuse == "quarry";

            // A shoreline is not evidence supporting a corrupt spike. Keep its
            // filtering protection, but allow bounded artifact repair there.
            if (landformsOnly && natural != "cliff" && landuse != "quarry")
                continue;

            if (!area && natural != "coastline" && natural != "cliff" && natural != "beach"
                && waterway != "river" && waterway != "stream" && waterway != "riverbank")
                continue;

            var points = way.NodeIds.Where(data.Nodes.ContainsKey).Select(id => data.Nodes[id]).Select(node => (
                x: GeoMath.LongitudeToTileX(node.Longitude, zoom) * 256 - minTileX * 256,
                y: GeoMath.LatitudeToTileY(node.Latitude, zoom) * 256 - minTileY * 256)).ToArray();

            if (points.Length == 0
                || points.Any(
                    item => double.IsNaN(item.x)
                        || double.IsInfinity(item.x)
                        || double.IsNaN(item.y)
                        || double.IsInfinity(item.y)))
                continue;

            // Closed known water/quarry geometry protects actual interior and rims.
            // Line segments protect two source cells on either side; do not infer coast from DEM height.
            if (area && way.NodeIds.Length >= 4 && way.NodeIds[0] == way.NodeIds[way.NodeIds.Length - 1])
            {
                var holes =
                    holesByOuter.TryGetValue(way.Id, out var knownHoles)
                        ? knownHoles
                        : new System.Collections.Generic.List<(double x, double y)[]>();

                Fill(points, holes);
                Rim(points);
            }
            else
                for (var i = 1; i < points.Length; i++)
                {
                    var firstItem = points[i - 1];
                    var secondItem = points[i];
                    SegmentRim(firstItem, secondItem);
                }
        }

        foreach (var relation in data.Relations.OrderBy(relation => relation.Id))
        {
            token.ThrowIfCancellationRequested();
            relation.Tags.TryGetValue("natural", out var natural);
            relation.Tags.TryGetValue("landuse", out var landuse);

            if (landformsOnly
                ? landuse != "quarry"
                : natural != "water" && landuse != "reservoir" && landuse != "quarry")
                continue;

            var outer = Rings(relation.Members.Where(item => !item.inner).Select(item => item.wayId));
            var inner = Rings(relation.Members.Where(item => item.inner).Select(item => item.wayId));

            if (outer == null || inner == null)
                continue; // Incomplete topology is not an invented filled region.

            foreach (var ring in outer)
            {
                Fill(ring, inner);
                Rim(ring);
            }

            foreach (var hole in inner)
                Rim(hole);
        }

        if (report != null)
        {
            var protectedCells = 0;

            for (var i = 0; i < mask.Length; i++)
            {
                if ((i & 16383) == 0)
                    token.ThrowIfCancellationRequested();

                if (mask[i])
                    protectedCells++;
            }

            report(
                $"DEM PROTECTION policy=scanline-ring-holes-v3; landformsOnly={landformsOnly}; protectedCells={protectedCells}; totalCells={mask.Length}; knownOuterHoleSets={holesByOuter.Count}; sourceZoom={zoom}; width={width}; height={height}; lineBufferCells=2; filledBoundingBoxes=false; perCellPolygonEdgeScan=false");
        }

        return mask;

        System.Collections.Generic.List<(double x, double y)[]>? Rings(System.Collections.Generic.IEnumerable<long> ids)
        {
            var remaining = new System.Collections.Generic.List<System.Collections.Generic.List<long>>();

            foreach (var id in ids)
            {
                if (!data.Ways.TryGetValue(id, out var way)
                    || way.NodeIds.Length < 2
                    || way.NodeIds.Any(nodeId => !data.Nodes.ContainsKey(nodeId)))
                    return null;

                remaining.Add(way.NodeIds.ToList());
            }

            var rings = new System.Collections.Generic.List<(double x, double y)[]>();

            while (remaining.Count > 0)
            {
                token.ThrowIfCancellationRequested();
                var chain = remaining[0];
                remaining.RemoveAt(0);

                while (chain[0] != chain[chain.Count - 1])
                {
                    token.ThrowIfCancellationRequested();
                    var end = chain[chain.Count - 1];
                    var matches = remaining.Where(r => r[0] == end || r[r.Count - 1] == end).ToArray();

                    if (matches.Length != 1)
                        return null;

                    var next = matches[0];
                    remaining.Remove(next);

                    if (next[0] != end)
                        next.Reverse();

                    chain.AddRange(next.Skip(1));
                }

                if (chain.Count < 4)
                    return null;

                var points =
                    chain
                        .Select(id => data.Nodes[id])
                        .Select(
                            node => (
                                x: GeoMath.LongitudeToTileX(node.Longitude, zoom) * 256 - minTileX * 256,
                                y: GeoMath.LatitudeToTileY(node.Latitude, zoom) * 256 - minTileY * 256))
                        .ToArray();

                if (points.Any(
                    item => double.IsNaN(item.x)
                        || double.IsInfinity(item.x)
                        || double.IsNaN(item.y)
                        || double.IsInfinity(item.y)))
                    return null;

                rings.Add(points);
            }

            return rings;
        }

        System.Collections.Generic.List<double>[] Crossings((double x, double y)[] ring)
        {
            var rows = new System.Collections.Generic.List<double>[height];

            for (int k = 0, j = ring.Length - 1; k < ring.Length; j = k++)
            {
                token.ThrowIfCancellationRequested();
                var firstItem = ring[j];
                var secondItem = ring[k];

                if (firstItem.y == secondItem.y)
                    continue;

                var y0 = (int)Math.Max(0, Math.Min(height, Math.Ceiling(Math.Min(firstItem.y, secondItem.y))));
                var y1 = (int)Math.Max(0, Math.Min(height, Math.Ceiling(Math.Max(firstItem.y, secondItem.y))));

                for (var y = y0; y < y1; y++)
                {
                    if ((y & 255) == 0)
                        token.ThrowIfCancellationRequested();

                    (rows[y] ??=
                        new System.Collections.Generic.List<double>()).Add(
                            firstItem.x
                                + (y - firstItem.y) * (secondItem.x - firstItem.x) / (secondItem.y - firstItem.y));
                }
            }

            foreach (var row in rows)
                row?.Sort();

            return rows;
        }

        void Fill((double x, double y)[] ring, System.Collections.Generic.List<(double x, double y)[]> holes)
        {
            var rows = Crossings(ring);
            var exclusions = new System.Collections.Generic.List<(double left, double right)>[height];

            foreach (var hole in holes)
            {
                var holeRows = Crossings(hole);

                for (var y = 0; y < height; y++)
                {
                    if ((y & 255) == 0)
                        token.ThrowIfCancellationRequested();

                    var row = holeRows[y];

                    if (row == null)
                        continue;

                    for (var k = 0; k + 1 < row.Count; k += 2)
                        (exclusions[y] ??=
                            new System.Collections.Generic.List<(double left, double right)>()).Add(
                                (row[k], row[k + 1]));
                }
            }

            for (var y = 0; y < height; y++)
            {
                token.ThrowIfCancellationRequested();
                var row = rows[y];

                if (row == null)
                    continue;

                var omit = exclusions[y];
                omit?.Sort((firstItem, secondItem) => firstItem.left.CompareTo(secondItem.left));

                for (var k = 0; k + 1 < row.Count; k += 2)
                {
                    var left = row[k];
                    var right = row[k + 1];

                    if (omit != null)
                        foreach (var interval in omit)
                        {
                            if (interval.right <= left)
                                continue;

                            if (interval.left >= right)
                                break;

                            Span(left, Math.Min(right, interval.left), y);
                            left = Math.Max(left, interval.right);

                            if (left >= right)
                                break;
                        }

                    Span(left, right, y);
                }
            }
        }

        void Span(double left, double right, int y)
        {
            var x0 = (int)Math.Max(0, Math.Min(width, Math.Ceiling(left)));
            var x1 = (int)Math.Max(0, Math.Min(width, Math.Ceiling(right)));

            for (var x = x0; x < x1; x++)
                mask[y * width + x] = true;
        }

        void Rim((double x, double y)[] ring)
        {
            for (int k = 0, j = ring.Length - 1; k < ring.Length; j = k++)
                SegmentRim(ring[j], ring[k]);
        }

        void SegmentRim((double x, double y) firstItem, (double x, double y) secondItem)
        {
            // Clip the sampling segment to the grid halo before stepping. Exact
            // distance to the original segment filters the small candidate stencil.
            var dx = secondItem.x - firstItem.x;
            var dy = secondItem.y - firstItem.y;
            double first = 0, last = 1;

            bool Clip(double p, double q)
            {
                if (p == 0)
                    return q >= 0;

                var t = q / p;

                if (p < 0)
                {
                    if (t > last)
                        return false;

                    first = Math.Max(first, t);
                }
                else
                {
                    if (t < first)
                        return false;

                    last = Math.Min(last, t);
                }

                return true;
            }

            if (!Clip(-dx, firstItem.x + 3)
                || !Clip(dx, width + 2 - firstItem.x)
                || !Clip(-dy, firstItem.y + 3)
                || !Clip(dy, height + 2 - firstItem.y))
                return;

            var steps = Math.Max(1, (int)Math.Ceiling(Math.Max(Math.Abs(dx), Math.Abs(dy)) * (last - first) * 2));

            for (var step = 0; step <= steps; step++)
            {
                if ((step & 255) == 0)
                    token.ThrowIfCancellationRequested();

                var t = first + (last - first) * step / steps;
                var x = (int)Math.Floor(firstItem.x + dx * t);
                var y = (int)Math.Floor(firstItem.y + dy * t);

                for (var yy = Math.Max(0, y - 3); yy <= Math.Min(height - 1, y + 3); yy++)
                    for (var xx = Math.Max(0, x - 3); xx <= Math.Min(width - 1, x + 3); xx++)
                        if (DistanceSquared(xx, yy, firstItem, secondItem) <= 4)
                            mask[yy * width + xx] = true;
            }
        }
    }

    internal static double DistanceSquared(
        double x,
        double y,
        (double x, double y) firstItem,
        (double x, double y) secondItem)
    {
        var dx = secondItem.x - firstItem.x;
        var dy = secondItem.y - firstItem.y;
        var length = dx * dx + dy * dy;
        var t = length > 0 ? Math.Max(0, Math.Min(1, ((x - firstItem.x) * dx + (y - firstItem.y) * dy) / length)) : 0;
        var exception = x - firstItem.x - t * dx;
        var ey = y - firstItem.y - t * dy;

        return exception * exception + ey * ey;
    }

    internal static bool Inside(double x, double y, (double x, double y)[] points)
    {
        var inside = false;

        for (int k = 0, j = points.Length - 1; k < points.Length; j = k++)
            if ((points[k].y > y) != (points[j].y > y)
                && x < (points[j].x - points[k].x) * (y - points[k].y) / (points[j].y - points[k].y) + points[k].x)
                inside = !inside;

        return inside;
    }
}
