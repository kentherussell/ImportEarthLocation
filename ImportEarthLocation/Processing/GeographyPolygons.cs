using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using ImportEarthLocation.Geo;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

public sealed record GeographicPolygon(
    long Id,
    IReadOnlyDictionary<string, string> Tags,
    List<float3[]> Outer,
    List<float3[]> Inner)
{
    public bool IsRelation { get; init; }

    public long Identity => IsRelation ? -Id : Id;

    public bool Contains(float3 point)
        => Outer.Any(points => ForestProcessor.Inside(point, points))
            && !Inner.Any(points => ForestProcessor.Inside(point, points));

    // Horizontal distance (metres) from a point to the nearest ring edge, outer or inner.
    public float EdgeDistance(float2 point)
    {
        var best = float.PositiveInfinity;

        foreach (var ring in Outer.Concat(Inner))
            for (var i = 0; i < ring.Length; i++)
            {
                var vectorA = ring[i].xz;
                var vectorB = ring[(i + 1) % ring.Length].xz;
                var ab = vectorB - vectorA;
                var t = math.saturate(math.dot(point - vectorA, ab) / math.max(1e-6f, math.lengthsq(ab)));
                best = math.min(best, math.distance(point, vectorA + t * ab));
            }

        return best;
    }

    // The same distance as EdgeDistance(point) wherever it is at most limit; beyond the
    // limit it returns +∞. Callers that ignore distances past a limit use this: it visits
    // only the edges filed near the point instead of every edge of the polygon, and keeps
    // the same per-edge arithmetic, so the nearest distance is bit-identical (report 114 S10).
    public float EdgeDistance(float2 point, float limit)
    {
        var index = Indexes.GetValue(this, polygon => new PolygonIndex(polygon));
        var best = float.PositiveInfinity;

        void Visit(List<(float2 A, float2 B)> edges)
        {
            foreach (var (vectorA, vectorB) in edges)
            {
                var ab = vectorB - vectorA;
                var t = math.saturate(math.dot(point - vectorA, ab) / math.max(1e-6f, math.lengthsq(ab)));
                best = math.min(best, math.distance(point, vectorA + t * ab));
            }
        }

        Visit(index.Long);
        var reach = (int)Math.Ceiling(limit / PolygonIndex.Cell);
        var cx = (int)Math.Floor(point.x / PolygonIndex.Cell);
        var cz = (int)Math.Floor(point.y / PolygonIndex.Cell);

        for (var x = cx - reach; x <= cx + reach; x++)
            for (var z = cz - reach; z <= cz + reach; z++)
                if (index.Edges.TryGetValue((x, z), out var edges))
                    Visit(edges);

        return best <= limit ? best : float.PositiveInfinity;
    }

    // Contains, with each ring's band index (same answer, see ForestProcessor.RingIndex).
    public bool ContainsIndexed(float3 point)
    {
        var index = Indexes.GetValue(this, polygon => new PolygonIndex(polygon));

        return index.Outer.Any(ring => ring.Contains(point)) && !index.Inner.Any(ring => ring.Contains(point));
    }

    private static readonly System.Runtime.CompilerServices.ConditionalWeakTable<GeographicPolygon, PolygonIndex>
        Indexes = new();

    // Edges filed in square cells by their bounding boxes, so every edge that comes within
    // a distance of a point is filed in a cell within that distance. Edges spanning more
    // than 4,096 cells are kept in Long and always visited. Built once per polygon.
    private sealed class PolygonIndex
    {
        public const float Cell = 32;
        public readonly Dictionary<(int, int), List<(float2 A, float2 B)>> Edges = new();
        public readonly List<(float2 A, float2 B)> Long = new();
        public readonly ForestProcessor.RingIndex[] Outer;
        public readonly ForestProcessor.RingIndex[] Inner;

        public PolygonIndex(GeographicPolygon polygon)
        {
            foreach (var ring in polygon.Outer.Concat(polygon.Inner))
                for (var i = 0; i < ring.Length; i++)
                {
                    var vectorA = ring[i].xz;
                    var vectorB = ring[(i + 1) % ring.Length].xz;
                    var low = math.floor(math.min(vectorA, vectorB) / Cell);
                    var high = math.floor(math.max(vectorA, vectorB) / Cell);

                    if ((high.x - low.x + 1) * (high.y - low.y + 1) > 4096)
                    {
                        Long.Add((vectorA, vectorB));
                        continue;
                    }

                    for (var x = (int)low.x; x <= (int)high.x; x++)
                        for (var z = (int)low.y; z <= (int)high.y; z++)
                        {
                            if (!Edges.TryGetValue((x, z), out var list))
                                Edges[(x, z)] = list = new();

                            list.Add((vectorA, vectorB));
                        }
                }

            Outer = polygon.Outer.Select(points => new ForestProcessor.RingIndex(points)).ToArray();
            Inner = polygon.Inner.Select(points => new ForestProcessor.RingIndex(points)).ToArray();
        }
    }

    // The outer rings with each inner ring joined into the outer ring around it by a bridge
    // between their closest vertices (a keyhole), so a mask filled ring by ring, even-odd,
    // leaves the holes out. A ring canal mapped as one multipolygon around its polders
    // (Amsterdam's Beemster and Schermer) would otherwise fill them as water. The bridge
    // edges are walked once each way and add no area; for a ring canal they cross only
    // the canal. The closest pair is searched on at most HoleSamples vertices of the hole
    // and RingSamples of the outer ring, then refined between neighbouring samples.
    public IEnumerable<float3[]> FilledRings()
    {
        if (Inner.Count == 0)
            return Outer;

        const int HoleSamples = 32;
        const int RingSamples = 4096;
        var rings = Outer.Select(ring => ring.ToList()).ToList();

        foreach (var hole in Inner)
        {
            if (hole.Length < 3)
                continue;

            var owner = Outer.FindIndex(ring => ForestProcessor.Inside(hole[0], ring));

            if (owner < 0)
                continue;

            var ring = rings[owner];
            var holeStride = Math.Max(1, hole.Length / HoleSamples);
            var ringStride = Math.Max(1, ring.Count / RingSamples);
            var (holeIndex, ringIndex) = Closest(hole, ring, 0, hole.Length, holeStride, 0, ring.Count, ringStride);
            (holeIndex, ringIndex) =
                Closest(
                    hole,
                    ring,
                    holeIndex - holeStride,
                    holeIndex + holeStride,
                    1,
                    ringIndex - ringStride,
                    ringIndex + ringStride,
                    1);

            var joined = new List<float3>(ring.Count + hole.Length + 2);
            joined.AddRange(ring.Take(ringIndex + 1));

            for (var k = 0; k <= hole.Length; k++)
                joined.Add(hole[(holeIndex + k) % hole.Length]);

            joined.AddRange(ring.Skip(ringIndex));
            rings[owner] = joined;
        }

        return rings.Select(ring => ring.ToArray());
    }

    // The closest pair of a hole vertex and a ring vertex over the given index ranges (ends
    // exclusive, wrapping round each ring) and strides, in the horizontal plane.
    private static (int Hole, int Ring) Closest(
        float3[] hole,
        List<float3> ring,
        int holeFrom,
        int holeTo,
        int holeStride,
        int ringFrom,
        int ringTo,
        int ringStride)
    {
        int Wrap(int index, int count) => (index % count + count) % count;

        var best = float.PositiveInfinity;
        var pair = (Hole: 0, Ring: 0);

        for (var i = holeFrom; i < holeTo; i += holeStride)
        {
            var point = hole[Wrap(i, hole.Length)].xz;

            for (var j = ringFrom; j < ringTo; j += ringStride)
            {
                var distance = math.distancesq(point, ring[Wrap(j, ring.Count)].xz);

                if (distance < best)
                {
                    best = distance;
                    pair = (Wrap(i, hole.Length), Wrap(j, ring.Count));
                }
            }
        }

        return pair;
    }
}

public static class GeographyPolygons
{
    // How far beyond the map an open relation ring is cut and closed (metres).
    private const float OutsideMargin = 300;

    // Closes open chains (each with both ends beyond the square |x|, |z| <= half) into rings:
    // each chain is cut to the square, and each piece's exit joins the next piece's entry
    // along the square's edge, corners included. Walking the edge one way or the other gives
    // the two sides of the chains; the side that holds most of the islands is the inside, or
    // with no islands the smaller side (a river or lake crossing the map is smaller than the
    // land around it). Null when the pieces do not pair up.
    internal static List<float3[]>? CloseAlongBounds(
        IReadOnlyList<float3[]> chains,
        float half,
        IReadOnlyList<float3[]>? islands)
    {
        var perimeter = 8 * half;

        // Position along the square's edge, counter-clockwise from (-half, -half).
        float Along(float2 point)
        {
            if (Math.Abs(point.y + half) < .01f)
                return point.x + half;

            if (Math.Abs(point.x - half) < .01f)
                return 2 * half + point.y + half;

            if (Math.Abs(point.y - half) < .01f)
                return 4 * half + half - point.x;

            return 6 * half + half - point.y;
        }

        float2[] corners = { new(-half, -half), new(half, -half), new(half, half), new(-half, half) };
        float[] cornerAlong = { 0, 2 * half, 4 * half, 6 * half };

        bool Inside(float2 point) => Math.Abs(point.x) <= half && Math.Abs(point.y) <= half;

        // Where the segment from a to b crosses the square's edge, as a fraction of it.
        float Crossing(float2 a, float2 b)
        {
            var low = 0f;
            var high = 1f;
            var d = b - a;
            float[] p = { -d.x, d.x, -d.y, d.y };
            float[] q = { a.x + half, half - a.x, a.y + half, half - a.y };

            for (var k = 0; k < 4; k++)
            {
                if (Math.Abs(p[k]) < 1e-9f)
                    continue;

                var t = q[k] / p[k];

                if (p[k] < 0)
                    low = Math.Max(low, t);
                else
                    high = Math.Min(high, t);
            }

            return Inside(a) ? high : low;
        }

        var pieces = new List<(List<float2> Points, float Entry, float Exit)>();

        foreach (var chain in chains)
        {
            List<float2>? current = null;
            var entry = 0f;

            for (var k = 0; k + 1 < chain.Length; k++)
            {
                var a = chain[k].xz;
                var b = chain[k + 1].xz;
                bool insideA = Inside(a), insideB = Inside(b);

                if (!insideA && !insideB)
                {
                    // A segment can cross the square without either end inside it.
                    var d = b - a;
                    var low = 0f;
                    var high = 1f;
                    float[] p = { -d.x, d.x, -d.y, d.y };
                    float[] q = { a.x + half, half - a.x, a.y + half, half - a.y };
                    var misses = false;

                    for (var m = 0; m < 4 && !misses; m++)
                    {
                        if (Math.Abs(p[m]) < 1e-9f)
                        {
                            misses = q[m] < 0;
                            continue;
                        }

                        var t = q[m] / p[m];

                        if (p[m] < 0)
                            low = Math.Max(low, t);
                        else
                            high = Math.Min(high, t);
                    }

                    if (misses || low >= high)
                        continue;

                    var into = a + d * low;
                    var outOf = a + d * high;
                    pieces.Add((new List<float2> { into, outOf }, Along(into), Along(outOf)));
                    continue;
                }

                if (!insideA)
                {
                    var into = math.lerp(a, b, Crossing(a, b));
                    current = new List<float2> { into };
                    entry = Along(into);
                }

                if (insideB)
                {
                    current?.Add(b);
                    continue;
                }

                var outOfSquare = math.lerp(a, b, Crossing(a, b));
                current?.Add(outOfSquare);

                if (current != null)
                    pieces.Add((current, entry, Along(outOfSquare)));

                current = null;
            }

            if (current != null)
                return null;
        }

        if (pieces.Count == 0)
            return new List<float3[]>();

        // The rings walking the edge one way (forward: counter-clockwise) from each exit.
        List<List<float2>>? Close(bool forward)
        {
            float Gap(float from, float to) => forward ? (to - from + perimeter) % perimeter : (from - to + perimeter) % perimeter;

            var used = new bool[pieces.Count];
            var rings = new List<List<float2>>();

            for (var start = 0; start < pieces.Count; start++)
            {
                if (used[start])
                    continue;

                var ring = new List<float2>();
                var current = start;

                for (var guard = 0; guard <= pieces.Count; guard++)
                {
                    used[current] = true;
                    ring.AddRange(pieces[current].Points);
                    var exit = pieces[current].Exit;
                    var next = -1;
                    var best = float.PositiveInfinity;

                    for (var j = 0; j < pieces.Count; j++)
                    {
                        var gap = Gap(exit, pieces[j].Entry);

                        if ((j == start || !used[j]) && gap < best)
                        {
                            best = gap;
                            next = j;
                        }
                    }

                    if (next < 0)
                        return null;

                    // The corners passed on the way, in walking order.
                    foreach (var k in Enumerable.Range(0, 4)
                        .Where(k => Gap(exit, cornerAlong[k]) > 0 && Gap(exit, cornerAlong[k]) < best)
                        .OrderBy(k => Gap(exit, cornerAlong[k])))
                        ring.Add(corners[k]);

                    if (next == start)
                        break;

                    if (used[next])
                        return null;

                    current = next;
                }

                rings.Add(ring);
            }

            return rings;
        }

        double Area(List<float2> ring)
        {
            var sum = 0d;

            for (var k = 0; k < ring.Count; k++)
            {
                var a = ring[k];
                var b = ring[(k + 1) % ring.Count];
                sum += (double)a.x * b.y - (double)b.x * a.y;
            }

            return Math.Abs(sum / 2);
        }

        static bool Contains(List<float2> ring, float2 point)
        {
            var inside = false;

            for (int k = 0, j = ring.Count - 1; k < ring.Count; j = k++)
                if ((ring[k].y > point.y) != (ring[j].y > point.y)
                    && point.x < (ring[j].x - ring[k].x) * (point.y - ring[k].y) / (ring[j].y - ring[k].y) + ring[k].x)
                    inside = !inside;

            return inside;
        }

        var forwardRings = Close(true);
        var backwardRings = Close(false);

        int Holding(List<List<float2>>? rings)
            => rings == null || islands == null
                ? 0
                : islands.Count(island => rings.Any(ring => Contains(ring, island[0].xz)));

        List<List<float2>>? chosen;

        if (forwardRings == null || backwardRings == null)
            chosen = forwardRings ?? backwardRings;
        else if (Holding(forwardRings) != Holding(backwardRings))
            chosen = Holding(forwardRings) > Holding(backwardRings) ? forwardRings : backwardRings;
        else
            chosen = forwardRings.Sum(Area) <= backwardRings.Sum(Area) ? forwardRings : backwardRings;

        return chosen?.Select(ring => ring.Select(point => new float3(point.x, 0, point.y)).ToArray()).ToList();
    }

    public static List<GeographicPolygon> Build(
        OsmDataset data,
        ImportRequest request,
        ImportLog log,
        CancellationToken token)
    {
        var polygons = new List<GeographicPolygon>();
        var members = new HashSet<long>();
        var fallbackHoles = new Dictionary<long, List<float3[]>>();
        var clipHalf = (float)(request.MapSize.WidthMeters / 2) + OutsideMargin;

        foreach (var relation in data.Relations.OrderBy(relation => relation.Id))
        {
            token.ThrowIfCancellationRequested();

            if (!relation.Tags.TryGetValue("type", out var type) || type != "multipolygon")
                continue;

            try
            {
                var inner = RelationRings(relation.Members.Where(item => item.inner).Select(item => item.wayId), null);
                var outer = RelationRings(relation.Members.Where(item => !item.inner).Select(item => item.wayId), inner);

                if (outer.Count == 0)
                    throw new InvalidOperationException("No outer polygon rings.");

                polygons.Add(new GeographicPolygon(relation.Id, relation.Tags, outer, inner) { IsRelation = true });

                foreach (var member in relation.Members)
                    members.Add(member.wayId);

                log.Count("Geographic relations accepted");
            }
            catch (InvalidOperationException exception)
            {
                log.Count("Geographic relations incomplete");

                // Keep valid, independently tagged outer members available. Never turn a
                // known island/hole into an outer polygon when its parent is incomplete.
                foreach (var member in relation.Members.Where(item => item.inner))
                    members.Add(member.wayId);

                try
                {
                    var holes = Rings(relation.Members.Where(item => item.inner).Select(item => item.wayId));

                    foreach (var member in relation.Members.Where(item => !item.inner))
                    {
                        if (!fallbackHoles.TryGetValue(member.wayId, out var retained))
                            fallbackHoles[member.wayId] = retained = new List<float3[]>();

                        retained.AddRange(holes);
                    }
                }
                catch (InvalidOperationException)
                {
                    // An unknown hole cannot safely be filled by recovering its parent.
                    foreach (var member in relation.Members)
                        members.Add(member.wayId);

                    log.Count("Geographic fallback blocked by unresolved holes");
                }

                log.Warn(
                    $"Incomplete geographic polygon relation {relation.Id}: {exception.Message} Independently tagged closed outer ways remain eligible; no missing ring is invented.");
            }
        }

        foreach (var way in data.Ways.Values.OrderBy(way => way.Id))
        {
            token.ThrowIfCancellationRequested();

            if (members.Contains(way.Id)
                || way.NodeIds.Length < 4
                || way.NodeIds[0] != way.NodeIds[way.NodeIds.Length - 1])
                continue;

            try
            {
                // Untagged relation geometry has no independent semantic meaning.
                if (way.Tags.Count == 0)
                    continue;

                polygons.Add(new GeographicPolygon(way.Id, way.Tags, Rings(new[] { way.Id }),
                    fallbackHoles.TryGetValue(way.Id, out var holes) ? holes : new List<float3[]>()));

                if (fallbackHoles.ContainsKey(way.Id))
                    log.Count("Geographic relation members recovered");

                log.Count("Geographic standalone ways accepted");
            }
            catch (InvalidOperationException exception)
            {
                log.Warn($"Skipped geographic polygon way {way.Id}: {exception.Message}");
            }
        }

        // A large relation (Montreal's St Lawrence) runs far beyond the downloaded area, so its
        // rings arrive open where their outside ways are missing. Open chains whose ends all lie
        // beyond the map are cut to a square OutsideMargin beyond it and closed along that
        // square's edge (CloseAlongBounds); the closing edges lie outside the map. A chain
        // ending on the map is genuinely broken and still rejects the relation.
        float3 Local(long id)
        {
            var node = data.Nodes[id];
            return GeoMath.ToLocalMeters(node.Latitude, node.Longitude, request.Latitude, request.Longitude);
        }

        // The relation's closed rings, plus its open chains closed along the clip square;
        // islands (the hint) choose which side of the square's edge is the inside.
        List<float3[]> RelationRings(IEnumerable<long> ids, IReadOnlyList<float3[]>? islands)
        {
            var open = new List<float3[]>();
            var rings = Rings(ids, open);

            if (open.Count == 0)
                return rings;

            if (open.Any(chain => Math.Max(Math.Abs(chain[0].x), Math.Abs(chain[0].z)) < clipHalf - OutsideMargin
                || Math.Max(Math.Abs(chain[chain.Length - 1].x), Math.Abs(chain[chain.Length - 1].z))
                    < clipHalf - OutsideMargin))
                throw new InvalidOperationException("Open/ambiguous multipolygon ring.");

            var closed = CloseAlongBounds(open, clipHalf, islands);

            if (closed == null)
                throw new InvalidOperationException("Open multipolygon ring could not be closed along the map's edge.");

            log.Count("Geographic relation rings closed along the map's edge", closed.Count);
            rings.AddRange(closed);

            return rings;
        }

        List<float3[]> Rings(IEnumerable<long> ids, List<float3[]>? open = null)
        {
            var remaining = new SortedDictionary<long, long[]>();

            foreach (var id in ids)
            {
                if (!data.Ways.TryGetValue(id, out var way)
                    || way.NodeIds.Length < 2
                    || way.NodeIds.Any(nodeId => !data.Nodes.ContainsKey(nodeId)))
                    throw new InvalidOperationException("Missing polygon geometry.");

                remaining[id] = way.NodeIds;
            }

            var rings = new List<float3[]>();

            while (remaining.Count > 0)
            {
                token.ThrowIfCancellationRequested();
                var first = remaining.First();
                remaining.Remove(first.Key);
                var ring = first.Value.ToList();

                while (ring[0] != ring[ring.Count - 1])
                {
                    var end = ring[ring.Count - 1];
                    var choices =
                        remaining
                            .Where(entry => entry.Value[0] == end || entry.Value[entry.Value.Length - 1] == end)
                            .ToArray();

                    if (choices.Length == 0 && open != null)
                    {
                        // An open end: extend the chain from its start too, then keep it open.
                        while (true)
                        {
                            var start = ring[0];
                            var before =
                                remaining
                                    .Where(entry => entry.Value[0] == start || entry.Value[entry.Value.Length - 1] == start)
                                    .ToArray();

                            if (before.Length != 1)
                                break;

                            remaining.Remove(before[0].Key);
                            var head = before[0].Value[before[0].Value.Length - 1] == start ? before[0].Value : before[0].Value.Reverse().ToArray();
                            ring.InsertRange(0, head.Take(head.Length - 1));
                        }

                        open.Add(ring.Select(Local).ToArray());
                        ring = null;
                        break;
                    }

                    if (choices.Length != 1)
                        throw new InvalidOperationException("Open/ambiguous multipolygon ring.");

                    var next = choices[0];
                    remaining.Remove(next.Key);
                    ring.AddRange((next.Value[0] == end ? next.Value : next.Value.Reverse()).Skip(1));
                }

                if (ring == null)
                    continue;

                if (ring.Count < 4)
                    throw new InvalidOperationException("Degenerate polygon ring.");

                var points = ring.Select(id =>
{
    var node = data.Nodes[id];
    return GeoMath.ToLocalMeters(node.Latitude, node.Longitude, request.Latitude, request.Longitude);
}).ToArray();

                if (points.Any(point => !math.all(math.isfinite(point))))
                    throw new InvalidOperationException("Non-finite polygon geometry.");

                rings.Add(points);
            }

            return rings;
        }

        return polygons;
    }
}
