using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

public static class SurfacePlanner
{
    public static SurfaceMask Prepare(IReadOnlyList<GeographicPolygon> polygons, ImportRequest request, TerrainBuilder.Result terrain, ImportLog log, CancellationToken token)
    {
        const int resolution = 2048;
        var material = new byte[resolution * resolution]; var confidence = new byte[material.Length];
        var half = (float)(request.MapSize.WidthMeters / 2); var cell = (float)(request.MapSize.WidthMeters / resolution);
        var classified = 0;
        foreach (var polygon in polygons.OrderBy(p => p.Id))
        {
            var (kind, priority) = Classify(polygon.Tags);
            if (kind == GroundMaterial.Base || polygon.Outer.Count == 0) continue;
            foreach (var ring in polygon.Outer)
            {
                var minX = Math.Max(0, (int)Math.Floor((ring.Min(p => p.x) + half) / cell));
                var maxX = Math.Min(resolution - 1, (int)Math.Ceiling((ring.Max(p => p.x) + half) / cell));
                var minZ = Math.Max(0, (int)Math.Floor((ring.Min(p => p.z) + half) / cell));
                var maxZ = Math.Min(resolution - 1, (int)Math.Ceiling((ring.Max(p => p.z) + half) / cell));
                for (var z = minZ; z <= maxZ; z++)
                {
                    token.ThrowIfCancellationRequested();
                    for (var x = minX; x <= maxX; x++)
                    {
                        var i = z * resolution + x; if (confidence[i] >= priority) continue;
                        var point = new float3((x + .5f) * cell - half, 0, (z + .5f) * cell - half);
                        if (!polygon.Contains(point)) continue;
                        var elevation = NetworkProcessor.Sample(point.x, point.z, terrain.Heightmap, terrain.HeightScaleMeters, request.MapSize.WidthMeters);
                        if (elevation < WaterLevels.Effective(terrain.SeaLevelMeters)) continue;
                        material[i] = (byte)kind; confidence[i] = priority; classified++;
                    }
                }
            }
        }
        // Complete exclusive mask first; two-pass chamfer distance to any material boundary.
        var distance = new float[material.Length];
        for (var z = 0; z < resolution; z++)
        {
            token.ThrowIfCancellationRequested();
            for (var x = 0; x < resolution; x++)
            {
                var i = z * resolution + x;
                distance[i] = material[i] == 0 || x == 0 || z == 0 || x == resolution - 1 || z == resolution - 1 ||
                    material[i - 1] != material[i] || material[i + 1] != material[i] || material[i - resolution] != material[i] || material[i + resolution] != material[i] ? 0 : 100000;
            }
        }
        for (var z = 1; z < resolution; z++)
        {
            token.ThrowIfCancellationRequested();
            for (var x = 1; x < resolution; x++)
            {
                var i = z * resolution + x;
                distance[i] = Math.Min(distance[i], Math.Min(distance[i - 1] + cell, Math.Min(distance[i - resolution] + cell, distance[i - resolution - 1] + cell * 1.414214f)));
            }
        }
        for (var z = resolution - 2; z >= 0; z--)
        {
            token.ThrowIfCancellationRequested();
            for (var x = resolution - 2; x >= 0; x--)
            {
                var i = z * resolution + x;
                distance[i] = Math.Min(distance[i], Math.Min(distance[i + 1] + cell, Math.Min(distance[i + resolution] + cell, distance[i + resolution + 1] + cell * 1.414214f)));
            }
        }
        var intensity = new byte[material.Length];
        for (var z = 0; z < resolution; z++)
        {
            token.ThrowIfCancellationRequested();
            for (var x = 0; x < resolution; x++)
            {
                var i = z * resolution + x;
                if (material[i] == 0) continue;
                var transition = material[i] == (byte)GroundMaterial.DryGrass || material[i] == (byte)GroundMaterial.Grass ? 100 : 40;
                var width = Math.Max(transition, cell * 2) * (1 + .1 * Math.Sin(x * cell / 180) * Math.Cos(z * cell / 220));
                var fraction = Math.Min(1, (distance[i] + cell * .5) / width);
                intensity[i] = (byte)Math.Round(255 * fraction * fraction * (3 - 2 * fraction));
            }
        }
        var report = $"Ground surfaces: exclusive material + intensity feathering; {material.Count(m => m != 0):N0} mapped samples; unknown areas remain base. Explicit sand/beaches only; no coastline-only sand inference.";
        log.Info(report);
        return new SurfaceMask(resolution, request.MapSize.WidthMeters, material, intensity, report)
        {
            Confidence = confidence,
            Transitions = material.Select(m => m == (byte)GroundMaterial.Grass || m == (byte)GroundMaterial.DryGrass ? SurfaceTransition.Soft : SurfaceTransition.Medium).ToArray()
        };
    }
    private static (GroundMaterial material, byte confidence) Classify(IReadOnlyDictionary<string, string> tags)
    {
        tags.TryGetValue("natural", out var natural); tags.TryGetValue("landuse", out var landuse);
        if (natural == "sand") return (GroundMaterial.Sand, 250);
        if (natural == "beach") return (GroundMaterial.Sand, 240);
        if (landuse == "quarry") return (GroundMaterial.Dirt, 250);
        if (landuse == "grass" || landuse == "meadow" || natural == "grassland") return (GroundMaterial.Grass, 220);
        // Scrub does not reliably mean dry soil; leave it unclassified.
        return (GroundMaterial.Base, 0);
    }
}
