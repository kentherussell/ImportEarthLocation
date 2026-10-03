using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using ImportEarthLocation.Geo;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

public static class ForestProcessor
{
    public const int MaximumTrees = 75000;
    public const float MinimumSpacing = 12f;
    private sealed record Mask(List<float3[]> Outer, List<float3[]> Inner);
    private readonly record struct Corridor(float3 A, float3 B, float Radius);
    private static bool Forest(IReadOnlyDictionary<string, string> tags) =>
        tags.TryGetValue("natural", out var natural) && natural == "wood" || tags.TryGetValue("landuse", out var landuse) && landuse == "forest";

    public static IReadOnlyList<float3> Prepare(OsmDataset data, ImportRequest request, TerrainBuilder.Result terrain,
        IReadOnlyList<PreparedSegment> networks, ImportLog log, CancellationToken token)
    {
        token.ThrowIfCancellationRequested();
        var timer = Stopwatch.StartNew(); var masks = new List<Mask>(); var members = new HashSet<long>();
        foreach (var relation in data.Relations.Where(relation => Forest(relation.Tags)).OrderBy(relation => relation.Id))
        {
            token.ThrowIfCancellationRequested();
            foreach (var member in relation.Members) members.Add(member.wayId);
            try
            {
                masks.Add(new Mask(Rings(relation.Members.Where(member => !member.inner).Select(member => member.wayId)),
                    Rings(relation.Members.Where(member => member.inner).Select(member => member.wayId))));
            }
            catch (InvalidOperationException exception) { log.Warn($"Skipped forest relation {relation.Id}: {exception.Message}"); }
        }
        foreach (var way in data.Ways.Values.Where(way => Forest(way.Tags) && !members.Contains(way.Id)).OrderBy(way => way.Id))
        {
            token.ThrowIfCancellationRequested();
            try { masks.Add(new Mask(Rings(new[] { way.Id }), new List<float3[]>())); }
            catch (InvalidOperationException exception) { log.Warn($"Skipped forest way {way.Id}: {exception.Message}"); }
        }
        List<float3[]> Rings(IEnumerable<long> ids)
        {
            var remaining = new SortedDictionary<long, long[]>();
            foreach (var id in ids)
            {
                if (!data.Ways.TryGetValue(id, out var way) || way.NodeIds.Any(node => !data.Nodes.ContainsKey(node)))
                    throw new InvalidOperationException("Incomplete forest polygon membership.");
                remaining[id] = way.NodeIds;
            }
            var rings = new List<float3[]>();
            while (remaining.Count > 0)
            {
                var first = remaining.First(); remaining.Remove(first.Key); var ring = first.Value.ToList();
                while (ring.Count > 1 && ring[0] != ring[ring.Count - 1])
                {
                    token.ThrowIfCancellationRequested();
                    var end = ring[ring.Count - 1];
                    var choices = remaining.Where(item => item.Value[0] == end || item.Value[item.Value.Length - 1] == end).ToArray();
                    if (choices.Length != 1) throw new InvalidOperationException("Open or ambiguous forest ring.");
                    var next = choices[0]; remaining.Remove(next.Key);
                    ring.AddRange((next.Value[0] == end ? next.Value : next.Value.Reverse()).Skip(1));
                }
                if (ring.Count < 4 || ring[0] != ring[ring.Count - 1]) throw new InvalidOperationException("Invalid forest ring.");
                var points = ring.Select(id => { var node = data.Nodes[id]; return GeoMath.ToLocalMeters(node.Latitude, node.Longitude, request.Latitude, request.Longitude); }).ToArray();
                if (points.Any(point => !math.all(math.isfinite(point)))) throw new InvalidOperationException("Invalid forest coordinates.");
                rings.Add(points);
            }
            return rings;
        }
        var half = (float)(request.MapSize.WidthMeters / 2);
        var area = masks.Sum(mask => mask.Outer.Sum(ring => Math.Max(0, Math.Min(half, ring.Max(p => p.x)) - Math.Max(-half, ring.Min(p => p.x))) *
            Math.Max(0, Math.Min(half, ring.Max(p => p.z)) - Math.Max(-half, ring.Min(p => p.z)))));
        var cell = Math.Max(30f, (float)Math.Sqrt(area / MaximumTrees));
        if (cell > 30) log.Info($"Forest density reduced deterministically to respect the {MaximumTrees:N0}-tree limit.");
        var corridors = new Dictionary<(int, int), List<Corridor>>();
        foreach (var segment in networks)
        {
            token.ThrowIfCancellationRequested();
            var radius = segment.Kind == NetworkKind.Rail ? 12f : Math.Max(15f, segment.Lanes * 1.75f + 7f);
            var sections = Math.Max(1, (int)Math.Ceiling(math.distance(segment.Start, segment.End) / 10f));
            for (var i = 0; i < sections; i++)
            {
                var a = NetworkNormalizer.Evaluate(segment, i / (float)sections); var b = NetworkNormalizer.Evaluate(segment, (i + 1) / (float)sections);
                var corridor = new Corridor(a, b, radius + 2f);
                for (var x = (int)Math.Floor((Math.Min(a.x, b.x) - radius - 2) / 100); x <= (int)Math.Floor((Math.Max(a.x, b.x) + radius + 2) / 100); x++)
                for (var z = (int)Math.Floor((Math.Min(a.z, b.z) - radius - 2) / 100); z <= (int)Math.Floor((Math.Max(a.z, b.z) + radius + 2) / 100); z++)
                { if (!corridors.TryGetValue((x, z), out var list)) corridors[(x, z)] = list = new List<Corridor>(); list.Add(corridor); }
            }
        }
        var output = new List<float3>(); var visited = new HashSet<(int, int)>();
        var spacing = new Dictionary<(int, int), List<float3>>();
        foreach (var mask in masks)
        foreach (var ring in mask.Outer)
        {
            var minX = (int)Math.Floor(Math.Max(-half, ring.Min(p => p.x)) / cell); var maxX = (int)Math.Floor(Math.Min(half, ring.Max(p => p.x)) / cell);
            var minZ = (int)Math.Floor(Math.Max(-half, ring.Min(p => p.z)) / cell); var maxZ = (int)Math.Floor(Math.Min(half, ring.Max(p => p.z)) / cell);
            for (var z = minZ; z <= maxZ && output.Count < MaximumTrees; z++)
            for (var x = minX; x <= maxX && output.Count < MaximumTrees; x++)
            {
                token.ThrowIfCancellationRequested();
                // Global cell hashing gives reproducible, non-regular points, independent of tile order.
                var seed = Hash(unchecked((uint)x * 73856093u ^ (uint)z * 19349663u));
                var point = new float3((x + (seed & 65535) / 65536f) * cell, 0, (z + (Hash(seed) & 65535) / 65536f) * cell);
                if (Math.Abs(point.x) > half || Math.Abs(point.z) > half || !Inside(point, ring) || mask.Inner.Any(hole => Inside(point, hole))) continue;
                if (!visited.Add((x, z))) continue;
                point.y = NetworkProcessor.Sample(point.x, point.z, terrain.Heightmap, terrain.HeightScaleMeters, request.MapSize.WidthMeters);
                if (point.y <= WaterLevels.Effective(terrain.SeaLevelMeters) + 0.5f) continue;
                if (corridors.TryGetValue(((int)Math.Floor(point.x / 100), (int)Math.Floor(point.z / 100)), out var nearby) &&
                    nearby.Any(corridor => NetworkNormalizer.DistanceToSegment(point, corridor.A, corridor.B) <= corridor.Radius)) continue;
                var bucket = ((int)Math.Floor(point.x / MinimumSpacing), (int)Math.Floor(point.z / MinimumSpacing)); var tooClose = false;
                for (var dx = -1; dx <= 1; dx++) for (var dz = -1; dz <= 1; dz++)
                    if (spacing.TryGetValue((bucket.Item1 + dx, bucket.Item2 + dz), out var neighbors) && neighbors.Any(other => math.distance(other.xz, point.xz) < MinimumSpacing)) tooClose = true;
                if (tooClose) continue;
                if (!spacing.TryGetValue(bucket, out var trees)) spacing[bucket] = trees = new List<float3>();
                trees.Add(point); output.Add(point);
            }
        }
        log.Count("Trees prepared", output.Count); log.Time("Vegetation preparation", timer.Elapsed.TotalSeconds);
        return output;
    }

    private static uint Hash(uint value) { unchecked { value ^= value >> 16; value *= 0x7feb352d; value ^= value >> 15; value *= 0x846ca68b; return value ^ (value >> 16); } }
    private static bool Inside(float3 point, IReadOnlyList<float3> ring)
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
