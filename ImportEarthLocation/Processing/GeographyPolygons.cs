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
    public static List<GeographicPolygon> Build(
        OsmDataset data,
        ImportRequest request,
        ImportLog log,
        CancellationToken token)
    {
        var polygons = new List<GeographicPolygon>();
        var members = new HashSet<long>();
        var fallbackHoles = new Dictionary<long, List<float3[]>>();

        foreach (var relation in data.Relations.OrderBy(relation => relation.Id))
        {
            token.ThrowIfCancellationRequested();

            if (!relation.Tags.TryGetValue("type", out var type) || type != "multipolygon")
                continue;

            try
            {
                var outer = Rings(relation.Members.Where(item => !item.inner).Select(item => item.wayId));

                if (outer.Count == 0)
                    throw new InvalidOperationException("No outer polygon rings.");

                var inner = Rings(relation.Members.Where(item => item.inner).Select(item => item.wayId));
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

        List<float3[]> Rings(IEnumerable<long> ids)
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

                    if (choices.Length != 1)
                        throw new InvalidOperationException("Open/ambiguous multipolygon ring.");

                    var next = choices[0];
                    remaining.Remove(next.Key);
                    ring.AddRange((next.Value[0] == end ? next.Value : next.Value.Reverse()).Skip(1));
                }

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
