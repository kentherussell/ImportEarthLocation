using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using ImportEarthLocation.Geo;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Mapped flood defences, taken out of the geography download (report 112, section 14).
// The geography query fetches them alongside water and land cover; they are split off
// right after parsing, so every stage that read the geography before still sees exactly
// what it did (a dam or lock gate is tagged waterway, which water planning and the
// roadbed's guarded lines would otherwise pick up).
public static class FloodDefenceData
{
    public const float FallbackOceanDepth = 10;

    public static bool IsDefence(IReadOnlyDictionary<string, string> tags)
    {
        string Tag(string key) => tags.TryGetValue(key, out var value) ? value : "";

        return Tag("man_made") is "dyke" or "flood_wall"
            || Tag("wall") == "flood_wall"
            || Tag("waterway") is "dam" or "lock_gate" or "sluice_gate";
    }

    // Defence lines in local metres, and the geography without the ways that only the
    // defence query fetched.
    public static (OsmDataset Geography, float3[][] Lines) Split(OsmDataset geography, double latitude, double longitude)
    {
        var lines = new List<float3[]>();
        var ways = new Dictionary<long, OsmWay>();
        // Relation members were fetched with their relation whatever their own tags.
        var members = new HashSet<long>(geography.Relations.SelectMany(relation => relation.Members.Select(member => member.wayId)));

        foreach (var (id, way) in geography.Ways)
        {
            if (!IsDefence(way.Tags))
            {
                ways[id] = way;
                continue;
            }

            lines.Add(
                way.NodeIds
                    .Where(geography.Nodes.ContainsKey)
                    .Select(node => GeoMath.ToLocalMeters(geography.Nodes[node].Latitude, geography.Nodes[node].Longitude, latitude, longitude))
                    .ToArray());

            if (FetchedOtherwise(way.Tags) || members.Contains(id))
                ways[id] = way;
        }

        var kept =
            new OsmDataset
            {
                MalformedFeaturesSkipped = geography.MalformedFeaturesSkipped,
                Nodes = geography.Nodes,
                NodeTags = geography.NodeTags,
                Ways = ways,
                Relations = geography.Relations,
                SupportWayIds = geography.SupportWayIds,
                RawWayTags = geography.RawWayTags
            };

        return (kept, lines.Where(line => line.Length > 1).ToArray());
    }

    // FloodDefences on an import's grid. Wet cells are the coast model's ocean
    // (classification 1, a coarser grid over the same corners, rows south to north) and
    // mapped water and quarries; ocean is null when the capture has none. With the water
    // plan and its polygons, lakes and rivers flood at their planned stages as well.
    public static FloodDefences.Outcome Apply(
        float[] heights,
        float scale,
        double width,
        float seaSurface,
        OceanMask? ocean,
        IEnumerable<float3[]> waterAreas,
        IEnumerable<float3[]> defenceLines,
        ImportLog log,
        CancellationToken token,
        WaterPlan? water = null,
        IReadOnlyList<GeographicPolygon>? polygons = null)
    {
        var clock = System.Diagnostics.Stopwatch.StartNew();
        var n = (int)Math.Sqrt(heights.Length);
        var cell = width / (n - 1);
        var wet = TerrainExtremes.Protected(n, width, waterAreas, Array.Empty<float3[]>());

        if (ocean != null && ocean.Classification.Length == ocean.Resolution * ocean.Resolution)
            for (var z = 0; z < n; z++)
                for (var x = 0; x < n; x++)
                {
                    var ox = (int)Math.Round(x * (ocean.Resolution - 1) / (double)(n - 1));
                    var oz = (int)Math.Round(z * (ocean.Resolution - 1) / (double)(n - 1));

                    if (ocean.Classification[oz * ocean.Resolution + ox] == 1)
                        wet[z * n + x] = FloodDefences.OpenWater;
                }

        // A capture from before the ocean mask was recorded: ground more than
        // FallbackOceanDepth below the sea surface is taken as sea (no polder is as deep).
        if (ocean == null)
        {
            var deep = seaSurface - FallbackOceanDepth;

            for (var id = 0; id < heights.Length; id++)
                if (heights[id] < deep)
                    wet[id] = FloodDefences.OpenWater;
        }

        var mapped = TerrainExtremes.Protected(n, width, Array.Empty<float3[]>(), defenceLines);
        var inland = water == null || polygons == null ? null : InlandSurface(n, width, water, polygons, token);
        var outcome = FloodDefences.Build(heights, scale, width, seaSurface, wet, mapped, token, inland);
        var area = cell * cell / 1e6;
        var staged = 0;

        if (inland != null)
            foreach (var level in inland)
                if (!float.IsNaN(level))
                    staged++;

        log.Count("Flood defence cells from mapped dykes", outcome.MappedCells);
        log.Count("Flood defence cells built", outcome.BuiltCells);
        log.Evidence(
            $"Flood defences: land below a water surface that the water reaches {outcome.LandReachedBefore * area:0.00} km² before, {outcome.LandReachedAfter * area:0.00} km² after; {outcome.MappedCells * area:0.00} km² of mapped dykes and {outcome.BuiltCells * area:0.00} km² of built ones raised to {FloodDefences.Freeboard:0} m above the water they hold; {outcome.PoldersDefended:N0} areas defended, {outcome.AreasTooDeep:N0} left that would need dykes over {FloodDefences.TallestDyke:0} m; ocean mask {(ocean == null ? "absent" : "used")}; lakes and rivers {(inland == null ? "not included" : $"at their stages over {staged * area:0.00} km²")}; {clock.Elapsed.TotalSeconds:0.0} s.");

        return outcome;
    }

    // The planned water surface of each grid cell in metres, NaN where none: lakes over
    // their mapped polygons at their stage (even-odd over outer and inner rings, so an
    // island stays land), and river reaches along their line out to the widened half
    // width the constructed surface sampler uses, at the stage interpolated along the
    // reach. Where two overlap the higher wins. The polygons are those the surface sampler
    // treats as water (WaterBodyPlanner.SurfaceSampler). Rasterised directly: the sampler
    // itself costs too much in game to ask at every cell.
    public static float[] InlandSurface(
        int n,
        double width,
        WaterPlan water,
        IReadOnlyList<GeographicPolygon> polygons,
        CancellationToken token)
    {
        var surface = new float[n * n];
        Array.Fill(surface, float.NaN);
        var cell = (float)(width / (n - 1));

        double Coordinate(int i) => (i / (double)(n - 1) - .5) * width;

        int Pixel(double value) => (int)Math.Round((value / width + .5) * (n - 1));

        void Set(int id, float level)
        {
            if (float.IsNaN(surface[id]) || surface[id] < level)
                surface[id] = level;
        }

        var stages =
            water.AreaStages
                .GroupBy(stage => stage.BodyId)
                .ToDictionary(group => group.Key, group => group.Min(stage => stage.Level));

        var crossings = new List<double>();

        foreach (var polygon in polygons)
        {
            token.ThrowIfCancellationRequested();

            if (!stages.TryGetValue(polygon.Identity, out var level) || WaterBodyPlanner.ExcludedChannel(polygon.Tags))
                continue;

            var tags = polygon.Tags;

            if (!(tags.TryGetValue("natural", out var natural) && natural == "water"
                || tags.TryGetValue("waterway", out var waterway) && waterway == "riverbank"
                || tags.TryGetValue("landuse", out var landuse) && landuse == "reservoir"))
                continue;

            var rings = polygon.Outer.Concat(polygon.Inner).Where(ring => ring.Length >= 3).ToArray();

            if (rings.Length == 0)
                continue;

            var low = Math.Max(0, rings.Min(ring => ring.Min(point => Pixel(point.z))));
            var high = Math.Min(n - 1, rings.Max(ring => ring.Max(point => Pixel(point.z))));

            for (var z = low; z <= high; z++)
            {
                var y = Coordinate(z);
                crossings.Clear();

                foreach (var ring in rings)
                    for (var i = 0; i < ring.Length; i++)
                    {
                        var a = ring[i];
                        var b = ring[(i + 1) % ring.Length];

                        if (a.z <= y && b.z > y || b.z <= y && a.z > y)
                            crossings.Add(a.x + (y - a.z) / (b.z - a.z) * (b.x - a.x));
                    }

                crossings.Sort();

                for (var i = 0; i + 1 < crossings.Count; i += 2)
                    for (var x = Math.Max(0, (int)Math.Ceiling((crossings[i] / width + .5) * (n - 1)));
                        x <= Math.Min(n - 1, (int)Math.Floor((crossings[i + 1] / width + .5) * (n - 1)));
                        x++)
                        Set(z * n + x, level);
            }
        }

        foreach (var reach in water.ReachStages)
        {
            token.ThrowIfCancellationRequested();
            var halfWidth = Math.Max(reach.HalfWidth, water.WaterCellSize) / .6f;
            var reachCells = (int)Math.Ceiling(halfWidth / cell);
            var steps = Math.Max(1, (int)Math.Ceiling(math.distance(reach.Start.xz, reach.End.xz) / (cell * .5f)));

            for (var step = 0; step <= steps; step++)
            {
                var t = step / (float)steps;
                var point = math.lerp(reach.Start, reach.End, t);
                var level = math.lerp(reach.StartLevel, reach.EndLevel, t);
                int px = Pixel(point.x), pz = Pixel(point.z);

                for (var z = Math.Max(0, pz - reachCells); z <= Math.Min(n - 1, pz + reachCells); z++)
                    for (var x = Math.Max(0, px - reachCells); x <= Math.Min(n - 1, px + reachCells); x++)
                    {
                        var dx = Coordinate(x) - point.x;
                        var dz = Coordinate(z) - point.z;

                        if (dx * dx + dz * dz <= halfWidth * halfWidth)
                            Set(z * n + x, level);
                    }
            }
        }

        return surface;
    }

    // Tags under which the geography query fetched a way before defences were added
    // (OverpassClient.BuildGeographyQuery).
    private static bool FetchedOtherwise(IReadOnlyDictionary<string, string> tags)
    {
        string Tag(string key) => tags.TryGetValue(key, out var value) ? value : "";

        return tags.ContainsKey("natural")
            || tags.ContainsKey("landuse")
            || tags.ContainsKey("wetland")
            || tags.ContainsKey("landcover")
            || Tag("area") == "yes"
            || Tag("waterway") is "river" or "stream" or "riverbank" or "canal" or "drain" or "ditch";
    }
}
