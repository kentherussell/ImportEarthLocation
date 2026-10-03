using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using ImportEarthLocation.Geo;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

public sealed record GeographicPolygon(long Id, IReadOnlyDictionary<string, string> Tags, List<float3[]> Outer, List<float3[]> Inner)
{
    public bool Contains(float3 point) => Outer.Any(r => GeographyPolygons.Inside(point, r)) && !Inner.Any(r => GeographyPolygons.Inside(point, r));
}
public static class GeographyPolygons
{
    public static List<GeographicPolygon> Build(OsmDataset data, ImportRequest request, ImportLog log, CancellationToken token)
    {
        var polygons = new List<GeographicPolygon>(); var members = new HashSet<long>();
        foreach (var relation in data.Relations.OrderBy(r => r.Id))
        {
            token.ThrowIfCancellationRequested();
            if (!relation.Tags.TryGetValue("type", out var type) || type != "multipolygon") continue;
            foreach (var member in relation.Members) members.Add(member.wayId);
            try { polygons.Add(new GeographicPolygon(relation.Id, relation.Tags, Rings(relation.Members.Where(m => !m.inner).Select(m => m.wayId)), Rings(relation.Members.Where(m => m.inner).Select(m => m.wayId)))); }
            catch (InvalidOperationException e) { log.Warn($"Skipped geographic polygon relation {relation.Id}: {e.Message}"); }
        }
        foreach (var way in data.Ways.Values.OrderBy(w => w.Id))
        {
            token.ThrowIfCancellationRequested();
            if (members.Contains(way.Id) || way.NodeIds.Length < 4 || way.NodeIds[0] != way.NodeIds[way.NodeIds.Length - 1]) continue;
            try { polygons.Add(new GeographicPolygon(way.Id, way.Tags, Rings(new[] { way.Id }), new List<float3[]>())); }
            catch (InvalidOperationException e) { log.Warn($"Skipped geographic polygon way {way.Id}: {e.Message}"); }
        }
        List<float3[]> Rings(IEnumerable<long> ids)
        {
            var remaining = new SortedDictionary<long, long[]>();
            foreach (var id in ids)
            {
                if (!data.Ways.TryGetValue(id, out var way) || way.NodeIds.Length < 2 || way.NodeIds.Any(n => !data.Nodes.ContainsKey(n))) throw new InvalidOperationException("Missing polygon geometry.");
                remaining[id] = way.NodeIds;
            }
            var rings = new List<float3[]>();
            while (remaining.Count > 0)
            {
                token.ThrowIfCancellationRequested();
                var first = remaining.First(); remaining.Remove(first.Key); var ring = first.Value.ToList();
                while (ring[0] != ring[ring.Count - 1])
                {
                    var end = ring[ring.Count - 1];
                    var choices = remaining.Where(p => p.Value[0] == end || p.Value[p.Value.Length - 1] == end).ToArray();
                    if (choices.Length != 1) throw new InvalidOperationException("Open/ambiguous multipolygon ring.");
                    var next = choices[0]; remaining.Remove(next.Key);
                    ring.AddRange((next.Value[0] == end ? next.Value : next.Value.Reverse()).Skip(1));
                }
                if (ring.Count < 4) throw new InvalidOperationException("Degenerate polygon ring.");
                var points = ring.Select(id => { var n = data.Nodes[id]; return GeoMath.ToLocalMeters(n.Latitude, n.Longitude, request.Latitude, request.Longitude); }).ToArray();
                if (points.Any(p => !math.all(math.isfinite(p)))) throw new InvalidOperationException("Non-finite polygon geometry.");
                rings.Add(points);
            }
            return rings;
        }
        return polygons;
    }
    public static bool Inside(float3 point, IReadOnlyList<float3> ring)
    {
        var inside = false;
        for (var i = 0; i < ring.Count; i++)
        {
            var a = ring[i]; var b = ring[(i + 1) % ring.Count];
            if ((a.z > point.z) != (b.z > point.z) && point.x < (b.x - a.x) * (point.z - a.z) / (b.z - a.z) + a.x) inside = !inside;
        }
        return inside;
    }
}
