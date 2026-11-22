using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// A terrain-only spill screen, in local map metres and absolute encoded heights.
// It does not certify native fluid behaviour. No terrain is raised or cut here.
public static class WaterContainment
{
    private const int MaximumCells = 1048576;
    private const float Freeboard = .35f;

    // Warnings go to the log, or into warnings when given: the planner checks a plan several
    // times and logs only its last check's warnings, once per lake.
    public static WaterPlan Validate(WaterPlan plan, TerrainBuilder.Result terrain, double width,
        IReadOnlyList<GeographicPolygon> polygons, ImportLog log, CancellationToken token,
        ICollection<string>? warnings = null)
    {
        token.ThrowIfCancellationRequested();

        void Warn(string message)
        {
            if (warnings == null)
                log.Warn(message);
            else
                warnings.Add(message);
        }
        var lakes = plan.Sources.Where(river => river.BodyKind == "lake").GroupBy(river => river.BodyId).ToArray();

        if (lakes.Length == 0)
            return plan;

        var outlines =
            polygons.GroupBy(polygon => polygon.Identity).ToDictionary(group => group.Key, group => group.First());

        // A neighbouring standing-water body may itself fail this screen, so at first only
        // mapped flowing reaches can establish an intended outlet here. A lake that escapes
        // is checked again below with the neighbours that passed as outlets too.
        // Channel cuts feather to zero over their outer 40 percent. Their wet
        // sloping rim may extend beyond the nominal one-cell flat core. Test
        // that rim against actual terrain below the reach's level; treating it
        // as dry exterior rejects otherwise contained lakes with stream outlets.
        var surface = PlannedSurface(plan with { AreaStages = Array.Empty<WaterAreaStage>() }, polygons);

        var levels = new Dictionary<long, float>();
        var rejected = new HashSet<long>();
        var checkedBodies = 0;
        var lowered = 0;
        var unscreened = 0;
        var escaped =
            new List<(long Body, GeographicPolygon Polygon, RiverSourcePlan[] Controls, float Stage, string Warning)>();

        bool Wet(RiverSourcePlan[] controls, float level)
            => controls.Any(
                river => level
                    - NetworkProcessor.Sample(
                        river.Position.x,
                        river.Position.z,
                        terrain.Heights,
                        width)
                    >= .4f);

        foreach (var lake in lakes)
        {
            token.ThrowIfCancellationRequested();

            if (!outlines.TryGetValue(lake.Key, out var polygon))
            {
                rejected.Add(lake.Key);
                Warn($"Lake {lake.Key}: containment outline unavailable; its controls were omitted.");
                continue;
            }

            if (plan.Coastal
                && (polygon.Tags.TryGetValue("water", out var kind) && kind == "lagoon"
                    || polygon.Tags.TryGetValue("salt", out var salt) && salt == "yes"))
                continue;

            var controls = lake.ToArray();
            var stage = controls.Min(river => river.TargetSurfaceLevel ?? river.Position.y + river.Depth);

            // A lake larger than the bounded screen cannot be checked, so it keeps the level the
            // planner set from its own shore (the lowest shore probe less a margin). Omitted, it
            // lost every control and its ground was restored: Lac Saint-Louis (1.9 million screen
            // cells) stayed dry but for the river's channel (Montreal, report 122 §11). A lake
            // planned below the sea's level stands among polders (Amsterdam's Markermeer at
            // −3.4 m from shore probes on polder land) and is still omitted, as before.
            var cells = ScreenCells(polygon, terrain.Heights, width, plan.WaterCellSize);

            if (cells > MaximumCells && stage >= terrain.SeaLevelMeters)
            {
                levels[lake.Key] = stage;
                unscreened++;
                Warn(
                    $"Lake {lake.Key}: too large for the containment screen ({cells:N0} cells); kept at its planned level {stage:0.##} m.");

                continue;
            }

            var spill =
                Spill(
                    polygon,
                    controls.Select(river => river.Position),
                    stage,
                    terrain.Heights,
                    width,
                    plan.WaterCellSize,
                    surface,
                    token);
            checkedBodies++;

            if (!spill.HasValue)
            {
                rejected.Add(lake.Key);
                Warn(
                    $"Lake {lake.Key}: containment coverage exceeded its bounded terrain screen; its controls were omitted.");

                continue;
            }

            var safe = Math.Min(stage, spill.Value - Freeboard);

            if (!Wet(controls, safe) || !math.isfinite(safe))
            {
                rejected.Add(lake.Key);

                escaped.Add(
                    (lake.Key,
                        polygon,
                        controls,
                        stage,
                        $"Lake {lake.Key}: its planned level {stage:0.##} m has an uncontained escape at {spill.Value:0.##} m; its controls were omitted."));
            }
            else
            {
                levels[lake.Key] = safe;

                if (safe < stage - .01f)
                    lowered++;
            }

            log.Diagnostic(
                $"WATER CONTAINMENT body={lake.Key}; plannedLevel={stage:R}; lowestSpill={spill.Value:R}; acceptedLevel={(rejected.Contains(lake.Key) ? float.NaN : safe):R}; sourceRejected={rejected.Contains(lake.Key)}; terrainOnly=true; nativeFlowVerified=false; bankRaising=false");
        }

        // Water passing into a neighbouring lake that passed stays in mapped water. Counted as
        // an escape over that lake's cut bed, it dropped lakes sharing a shore with another
        // (100 of Quebec City's 255 dropped lakes, report 122 §13). Only neighbours that
        // passed count, at their accepted levels: one that failed does not hold water.
        var standing =
            plan.AreaStages
                .Where(stage => levels.ContainsKey(stage.BodyId))
                .Select(stage => stage with { Level = levels[stage.BodyId] })
                .ToArray();

        var shared = standing.Length > 0 && escaped.Count > 0
            ? PlannedSurface(plan with { AreaStages = standing }, polygons)
            : null;

        var joined = 0;

        foreach (var (body, polygon, controls, stage, warning) in escaped)
        {
            token.ThrowIfCancellationRequested();

            var spill =
                shared == null
                    ? null
                    : Spill(
                        polygon,
                        controls.Select(river => river.Position),
                        stage,
                        terrain.Heights,
                        width,
                        plan.WaterCellSize,
                        shared,
                        token);

            var safe = spill.HasValue ? Math.Min(stage, spill.Value - Freeboard) : float.NaN;

            if (!math.isfinite(safe) || !Wet(controls, safe))
            {
                Warn(warning);
                continue;
            }

            rejected.Remove(body);
            levels[body] = safe;
            joined++;

            if (safe < stage - .01f)
                lowered++;
        }

        var sources = new List<RiverSourcePlan>();

        foreach (var source in plan.Sources)
        {
            if (rejected.Contains(source.BodyId))
                continue;

            if (source.BodyKind != "lake" || !levels.TryGetValue(source.BodyId, out var level))
            {
                sources.Add(source);
                continue;
            }

            var position = source.Position;
            position.y =
                NetworkProcessor.Sample(position.x, position.z, terrain.Heights, width);

            if (level - position.y < .4f)
                continue;

            sources.Add(source with { Position = position, TargetSurfaceLevel = level, Depth = level - position.y });
        }

        log.Count("Lake containment bodies checked", checkedBodies);
        log.Count("Lake containment levels lowered", lowered);
        log.Count("Lake containment bodies omitted", rejected.Count);
        log.Count("Lake containment bodies too large to screen", unscreened);
        log.Count("Lake containment bodies held by a neighbouring lake", joined);

        return plan with
        {
            Sources = sources,
            AreaStages = plan.AreaStages.Where(stage => !rejected.Contains(stage.BodyId))
            .Select(stage => levels.TryGetValue(stage.BodyId, out var level) ? stage with { Level = level } : stage)
                .ToArray(),
            Skipped = plan.Skipped + plan.Sources.Count - sources.Count,
            Report =
                plan.Report
                    + $" Lake containment: {checkedBodies} checked, {lowered} lowered, {rejected.Count} omitted; native flow unverified."
        };
    }

    // The planned water surface that ends a containment path: flowing reaches that no
    // buried inlet blocks, and the plan's area levels.
    public static Func<float3, float?> PlannedSurface(WaterPlan plan, IReadOnlyList<GeographicPolygon> polygons)
    {
        var flowing = plan.ReachStages.Where(reach => !reach.BlockedByBuriedInlet).ToArray();

        return WaterBodyPlanner.ConstructedSurfaceSampler(plan with { ReachStages = flowing }, polygons);
    }

    // The terrain cells Spill would hold for a polygon: its outer rings' bounds and margin,
    // clipped to the map.
    private static long ScreenCells(GeographicPolygon polygon, float[] heights, double width, float waterCell)
    {
        var resolution = TerrainSampling.Resolution(heights);
        var step = (float)width / (resolution - 1);
        var half = (float)width * .5f;
        var vertices = polygon.Outer.SelectMany(points => points).ToArray();

        if (vertices.Length < 3)
            return 0;

        var margin = Math.Max(step * 3, waterCell * 2);
        var low = vertices.Select(point => point.xz).Aggregate(math.min) - margin;
        var high = vertices.Select(point => point.xz).Aggregate(math.max) + margin;

        int Pixel(float value) => (int)Math.Floor((value + half) / step);

        var columns = Math.Min(resolution - 1, Pixel(high.x) + 1) - Math.Max(0, Pixel(low.x)) + 1;
        var rows = Math.Min(resolution - 1, Pixel(high.y) + 1) - Math.Max(0, Pixel(low.y)) + 1;

        return (long)columns * rows;
    }

    // Minimax Dijkstra: a path costs its highest terrain sample. The first dry
    // exterior point popped identifies the lowest escape saddle. Planned flowing
    // outlets terminate their own branch; other dry exterior land remains tested.
    // The lowest level at which water standing in the polygon escapes over the terrain, or
    // null when the polygon is too large for the bounded screen. Seeds are points of its
    // water; the surface gives the level of planned flowing water at a point, if any.
    public static float? Spill(
        GeographicPolygon polygon,
        IEnumerable<float3> seeds,
        float stage,
        float[] heights,
        double width,
        float waterCell,
        Func<float3, float?> surface,
        CancellationToken token)
    {
        var resolution = TerrainSampling.Resolution(heights);
        var step = (float)width / (resolution - 1);
        var half = (float)width * .5f;
        var vertices = polygon.Outer.SelectMany(points => points).ToArray();

        if (vertices.Length < 3)
            return null;

        var margin = Math.Max(step * 3, waterCell * 2);
        var low = vertices.Select(point => point.xz).Aggregate(math.min) - margin;
        var high = vertices.Select(point => point.xz).Aggregate(math.max) + margin;

        int Pixel(float value) => (int)Math.Floor((value + half) / step);

        var x0 = Math.Max(0, Pixel(low.x));
        var z0 = Math.Max(0, Pixel(low.y));
        var x1 = Math.Min(resolution - 1, Pixel(high.x) + 1);
        var z1 = Math.Min(resolution - 1, Pixel(high.y) + 1);
        var nx = x1 - x0 + 1;
        var nz = z1 - z0 + 1;
        var count = (long)nx * nz;

        if (nx < 2 || nz < 2 || count > MaximumCells)
            return null;

        var inside = new byte[(int)count];
        var crossings = new List<float>();
        void Fill(float3[] ring, byte value)
        {
            if (ring.Length < 3)
                return;

            for (var z = 0; z < nz; z++)
            {
                token.ThrowIfCancellationRequested();
                var worldZ = (z + z0) * step - half;
                crossings.Clear();

                for (var k = 0; k < ring.Length; k++)
                {
                    var vectorA = ring[k];
                    var vectorB = ring[(k + 1) % ring.Length];

                    if ((vectorA.z > worldZ) != (vectorB.z > worldZ))
                        crossings.Add(
                            vectorA.x + (worldZ - vectorA.z) * (vectorB.x - vectorA.x) / (vectorB.z - vectorA.z));
                }

                crossings.Sort();

                for (var k = 1; k < crossings.Count; k += 2)
                {
                    var first = Math.Max(0, (int)Math.Ceiling((crossings[k - 1] + half) / step) - x0);
                    var last = Math.Min(nx - 1, Pixel(crossings[k]) - x0);

                    for (var x = first; x <= last; x++)
                        inside[z * nx + x] = value;
                }
            }
        }

        foreach (var ring in polygon.Outer)
            Fill(ring, 1);

        foreach (var ring in polygon.Inner)
            Fill(ring, 2);

        // A one-water-cell shoreline collar accommodates the sloping bank, but
        // cannot conceal a path continuing into surrounding downhill terrain.
        var allowed = (byte[])inside.Clone();
        var collar = Math.Max(1, (int)Math.Ceiling(waterCell / step));

        for (var z = 0; z < nz; z++)
            for (var x = 0; x < nx; x++)
                if (inside[z * nx + x] == 1)
                {
                    if (x > 0
                        && x < nx - 1
                        && z > 0
                        && z < nz - 1
                        && inside[z * nx + x - 1] != 0
                        && inside[z * nx + x + 1] != 0
                        && inside[(z - 1) * nx + x] != 0
                        && inside[(z + 1) * nx + x] != 0)
                        continue;

                    for (var dz = -collar; dz <= collar; dz++)
                        for (var dx = -collar; dx <= collar; dx++)
                            if (x + dx >= 0
                                && x + dx < nx
                                && z + dz >= 0
                                && z + dz < nz
                                && inside[(z + dz) * nx + x + dx] != 2)
                                allowed[(z + dz) * nx + x + dx] = 1;
                }

        // The shoreline collar must not erase small mapped islands. Their
        // terrain, like the downhill exterior, must stand above the lake.
        for (var i = 0; i < inside.Length; i++)
            if (inside[i] == 2)
                allowed[i] = 0;

        var best = new float[(int)count];

        for (var k = 0; k < best.Length; k++)
            best[k] = float.PositiveInfinity;

        var heap = new List<(int Index, float Cost)>();
        // A sub-cell ridge can hold water on the DEM yet disappear on the
        // coarser fluid grid. Require the escape barrier to survive a half-cell
        // neighbourhood, rather than accepting a single high terrain vertex.
        var bankHeights = GridBankHeights(heights, resolution, x0, z0, nx, nz,
            Math.Max(0, (int)Math.Ceiling(waterCell / (2 * step))), token);

        float Height(int index)
            => (inside[index] == 2
                ? heights[(index / nx + z0) * resolution + index % nx + x0]
                : bankHeights[index]);

        void Push(int index, float cost)
        {
            if (cost >= best[index])
                return;

            best[index] = cost;
            heap.Add((index, cost));
            var k = heap.Count - 1;

            while (k > 0)
            {
                var parent = (k - 1) / 2;

                if (heap[parent].Cost <= cost)
                    break;

                heap[k] = heap[parent];
                k = parent;
            }

            heap[k] = (index, cost);
        }

        (int Index, float Cost) Pop()
        {
            var first = heap[0];
            var last = heap[heap.Count - 1];
            heap.RemoveAt(heap.Count - 1);

            if (heap.Count == 0)
                return first;

            var k = 0;

            while (k * 2 + 1 < heap.Count)
            {
                var child = k * 2 + 1;

                if (child + 1 < heap.Count && heap[child + 1].Cost < heap[child].Cost)
                    child++;

                if (heap[child].Cost >= last.Cost)
                    break;

                heap[k] = heap[child];
                k = child;
            }

            heap[k] = last;

            return first;
        }

        // Check every wet part of the mapped basin, including disconnected
        // outer rings and shallow shelves without a nearby circular control.
        // Seeding only controls can hide a low escape behind an uncut DEM ridge.
        for (var i = 0; i < inside.Length; i++)
        {
            if ((i & 16383) == 0)
                token.ThrowIfCancellationRequested();

            var x = i % nx;
            var z = i / nx;

            if (inside[i] == 1 && Height(i) < stage && (x == 0 || z == 0 || x == nx - 1 || z == nz - 1
                || inside[i - 1] != 1 || inside[i + 1] != 1 || inside[i - nx] != 1 || inside[i + nx] != 1))
                Push(i, Height(i));
        }

        foreach (var seed in seeds)
        {
            var x = (int)Math.Round((seed.x + half) / step) - x0;
            var z = (int)Math.Round((seed.z + half) / step) - z0;

            if (x >= 0 && x < nx && z >= 0 && z < nz)
                Push(z * nx + x, Height(z * nx + x));
        }

        if (heap.Count == 0)
            return null;

        while (heap.Count > 0)
        {
            token.ThrowIfCancellationRequested();
            var (index, cost) = Pop();

            if (cost != best[index])
                continue;

            var x = index % nx;
            var z = index / nx;

            if (allowed[index] == 0
                || x + x0 == 0
                || z + z0 == 0
                || x + x0 == resolution - 1
                || z + z0 == resolution - 1)
            {
                var point = new float3((x + x0) * step - half, Height(index), (z + z0) * step - half);
                var outlet = surface(point);

                // Ground under planned flowing water ends the path: the lake drains into a
                // stream lower than it, and cannot drain into one standing higher, which flows
                // in. Counting an inlet as an escape dropped lakes whose inlet channel, cut
                // below the stream's water, reached under the lake's level (report 122 §13).
                if (inside[index] != 2
                    && allowed[index] == 0
                    && outlet.HasValue
                    && point.y < outlet.Value + .1f)
                    continue;

                return cost;
            }

            for (var dz = -1; dz <= 1; dz++)
                for (var dx = -1; dx <= 1; dx++)
                {
                    if (dx == 0 && dz == 0 || x + dx < 0 || x + dx >= nx || z + dz < 0 || z + dz >= nz)
                        continue;

                    var next = (z + dz) * nx + x + dx;
                    Push(next, Math.Max(cost, Height(next)));
                }
        }

        // Every exit was an evidenced planned water connection.
        return float.MaxValue;
    }

    private static float[] GridBankHeights(
        float[] terrain,
        int n,
        int x0,
        int z0,
        int nx,
        int nz,
        int radius,
        CancellationToken token)
    {
        var first = Math.Max(0, z0 - radius);
        var last = Math.Min(n - 1, z0 + nz - 1 + radius);
        var rows = last - first + 1;
        var horizontal = new float[rows * nx];
        var result = new float[nx * nz];
        var queue = new int[Math.Max(n, rows)];

        for (var z = first; z <= last; z++)
        {
            token.ThrowIfCancellationRequested();
            var head = 0;
            var tail = 0;
            var added = Math.Max(0, x0 - radius) - 1;

            for (var x = 0; x < nx; x++)
            {
                var right = Math.Min(n - 1, x0 + x + radius);

                while (added < right)
                {
                    added++;

                    while (tail > head && terrain[z * n + queue[tail - 1]] >= terrain[z * n + added])
                        tail--;

                    queue[tail++] = added;
                }

                while (queue[head] < x0 + x - radius)
                    head++;

                horizontal[(z - first) * nx + x] = terrain[z * n + queue[head]];
            }
        }

        for (var x = 0; x < nx; x++)
        {
            token.ThrowIfCancellationRequested();
            var head = 0;
            var tail = 0;
            var added = -1;

            for (var z = 0; z < nz; z++)
            {
                var bottom = Math.Min(rows - 1, z0 + z + radius - first);

                while (added < bottom)
                {
                    added++;

                    while (tail > head && horizontal[queue[tail - 1] * nx + x] >= horizontal[added * nx + x])
                        tail--;

                    queue[tail++] = added;
                }

                while (queue[head] < z0 + z - radius - first)
                    head++;

                result[z * nx + x] = horizontal[queue[head] * nx + x];
            }
        }

        return result;
    }
}
