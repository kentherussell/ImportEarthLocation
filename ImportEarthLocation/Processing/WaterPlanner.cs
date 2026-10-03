using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using ImportEarthLocation.Geo;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

public static class WaterPlanner
{
    public static WaterPlan Prepare(OsmDataset? data, IReadOnlyList<GeographicPolygon> polygons, ImportRequest request, TerrainBuilder.Result terrain, ImportLog log, CancellationToken token)
    {
        if (data == null) return new WaterPlan(false, 0, 0, 0, "Water geography unavailable; inland sources skipped. Sea level assigned through existing native path, coastal coverage not verified.");
        var half = request.MapSize.WidthMeters / 2;
        bool Inside(float3 point) => Math.Abs(point.x) <= half && Math.Abs(point.z) <= half;
        bool Tag(OsmWay way, string key, string value) => way.Tags.TryGetValue(key, out var found) && found == value;
        var coastal = data.Ways.Values.Where(w => Tag(w, "natural", "coastline")).Any(w => w.NodeIds.Any(id => data.Nodes.TryGetValue(id, out var n) && Inside(GeoMath.ToLocalMeters(n.Latitude, n.Longitude, request.Latitude, request.Longitude))));
        var rivers = 0; var skipped = 0;
        var inDegree = new Dictionary<long, int>(); var outDegree = new Dictionary<long, int>();
        var waterways = data.Ways.Values.Where(w => Tag(w, "waterway", "river") || Tag(w, "waterway", "stream")).OrderBy(w => w.Id).ToArray();
        foreach (var way in waterways)
        {
            if (way.NodeIds.Length < 2) continue;
            var first = way.NodeIds[0]; var last = way.NodeIds[way.NodeIds.Length - 1];
            outDegree.TryGetValue(first, out var outgoing); outDegree[first] = outgoing + 1;
            inDegree.TryGetValue(last, out var incoming); inDegree[last] = incoming + 1;
        }
        foreach (var way in waterways)
        {
            token.ThrowIfCancellationRequested();
            if (way.NodeIds.Length < 2 || way.NodeIds.Any(id => !data.Nodes.ContainsKey(id)) || Tag(way, "intermittent", "yes"))
            { if (++skipped <= 25) log.Warn($"Waterway {way.Id} skipped: intermittent, malformed or missing geometry."); continue; }
            var points = way.NodeIds.Select(id => { var n = data.Nodes[id]; return GeoMath.ToLocalMeters(n.Latitude, n.Longitude, request.Latitude, request.Longitude); }).ToArray();
            var inside = points.Where(Inside).ToArray();
            if (inside.Length < 2) continue;
            var heights = inside.Select(p => NetworkProcessor.Sample(p.x, p.z, terrain.Heightmap, terrain.HeightScaleMeters, request.MapSize.WidthMeters)).ToArray();
            // Broad downhill test: tolerate minor rises; never demand strict monotonicity.
            if (heights.Any(h => !TerrainConditioner.Finite(h)) || heights[heights.Length - 1] > heights[0] + 5)
            { if (++skipped <= 25) log.Warn($"Waterway {way.Id} skipped: direction/elevation inconsistent (upstream {heights[0]:0.0}, downstream {heights[heights.Length - 1]:0.0} m)."); continue; }
            rivers++;
        }
        var lakes = polygons.Count(p => p.Tags.TryGetValue("natural", out var natural) && natural == "water" && (!p.Tags.TryGetValue("water", out var type) || type == "lake" || type == "reservoir") || p.Tags.TryGetValue("landuse", out var landuse) && landuse == "reservoir");
        // Current 1.6 editor source path ignores legacy ConstantDepth; GPU source level/flow mapping is not verified.
        // Stop only this affected stage, not the terrain/network import. No guessed pumping or terrain carving.
        var report = $"Water: {(coastal ? "mapped coast found; native sea plane +1 m" : "no mapped coast confirmed; native sea plane retained")}; inland sources NOT created ({rivers} river/stream candidates, {lakes} lakes/reservoirs, {skipped} invalid features). Current source level/flow semantics could not be safely verified; hydrology corrections skipped.";
        log.Warn(report);
        return new WaterPlan(coastal, rivers, lakes, skipped, report);
    }
}
