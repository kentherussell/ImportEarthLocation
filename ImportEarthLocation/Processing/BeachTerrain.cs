using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Mapped beaches are shaped as beaches: just above the water at their edge,
// rising gently inland, and continuing under the water as a shallow shelf. The
// DEM near a shore is coarse (Marseille: a 14 m grid against a median beach
// width of 10 m) and the game's water is raised 1 m above nominal sea level
// (WaterLevels.Effective), so mapped beaches were partly drowned, or stood high
// and dropped steeply into deep water, which the game draws as a brown band of
// bare slope between the sand and the sea.
//
// Every height is a continuous function of the exact signed distance s to the
// beach outline, never of which grid points fall inside it: a height that
// steps at the outline draws each 14 m grid point of a slanted edge as a
// tooth. With E = effective water + EdgeHeight:
//   inside:  E + Slope·s, at most InlandRise above E, raised or lowered (by at
//            most MaximumLift / MaximumCut);
//   inland:  outside on dry ground, the land blends from E at the outline back
//            to its own height over InlandBlend metres (smoothstep);
//   seaward: outside under water, a shelf falls from E to ShelfDepth below
//            the water over Shelf metres, steepest at the outline (so the
//            waterline lies a few metres beyond it), and blends into the
//            existing sea floor towards its end. The shelf only ever raises,
//            by at most MaximumLift, so a deep shore keeps its depth.
public static class BeachTerrain
{
    public const float EdgeHeight = .25f, Slope = .05f, InlandRise = 1, InlandBlend = 30;
    public const float Shelf = 160, ShelfDepth = 4;
    public const float MaximumLift = 3, MaximumCut = 3;

    public static float[] Lift(
        TerrainBuilder.Result terrain,
        double width,
        IReadOnlyList<GeographicPolygon> polygons,
        ImportLog log,
        CancellationToken token)
    {
        var beaches =
            polygons
                .Where(
                    polygon => polygon.Tags.TryGetValue("natural", out var text)
                        && text == "beach"
                        && polygon.Outer.Count > 0)
                .ToArray();

        if (beaches.Length == 0)
            return terrain.Heights;

        var n = (int)Math.Sqrt(terrain.Heights.Length);

        if ((long)n * n != terrain.Heights.Length || n < 2)
            return terrain.Heights;

        var surface = WaterLevels.Effective(terrain.SeaLevelMeters);
        var scale = terrain.HeightScaleMeters;
        var edge = surface + EdgeHeight;

        // Grid point i sits at (i/(n-1) − ½)·width, as NetworkProcessor.Sample reads it.
        float Coordinate(int i) => (i / (float)(n - 1) - .5f) * (float)width;

        int Index(float v) => (int)Math.Floor((v / width + .5) * (n - 1));

        var result = (float[])terrain.Heights.Clone();
        var raised = 0;
        var lowered = 0;
        var shelf = 0;
        var shaped = 0;
        var maximum = 0f;

        foreach (var beach in beaches)
        {
            token.ThrowIfCancellationRequested();
            var touched = false;
            var points = beach.Outer.SelectMany(otherPoints => otherPoints).ToArray();
            var reach = Math.Max(Shelf, InlandBlend);
            var low = points.Select(point => point.xz).Aggregate(math.min) - reach;
            var high = points.Select(point => point.xz).Aggregate(math.max) + reach;

            for (var z = Math.Max(0, Index(low.y)); z <= Math.Min(n - 1, Index(high.y) + 1); z++)
                for (var x = Math.Max(0, Index(low.x)); x <= Math.Min(n - 1, Index(high.x) + 1); x++)
                {
                    var i = z * n + x;
                    var current = result[i];
                    var point = new float3(Coordinate(x), 0, Coordinate(z));
                    var inside = beach.ContainsIndexed(point);

                    // No height below depends on a distance past the shelf.
                    var distance = beach.EdgeDistance(point.xz, Math.Max(Shelf, InlandBlend));
                    float target;

                    if (inside)
                        target =
                            math.clamp(
                                edge + Math.Min(InlandRise, Slope * distance),
                                current - MaximumCut,
                                current + MaximumLift);
                    else if (current >= surface)
                    {
                        // Ground standing well above the beach (a cliff, a seawall)
                        // keeps its height: the blend fades out between MaximumCut
                        // and twice that above the edge.
                        if (distance >= InlandBlend)
                            continue;

                        var weight =
                            (1 - Numeric.SmoothStep(distance / InlandBlend))
                                * Numeric.SmoothStep((2 * MaximumCut - (current - edge)) / MaximumCut);

                        target =
                            math.lerp(current, math.clamp(edge, current - MaximumCut, current + MaximumLift), weight);
                    }
                    else
                    {
                        if (distance >= Shelf)
                            continue;

                        var t = distance / Shelf;
                        var profile = edge - (EdgeHeight + ShelfDepth) * (1 - (1 - t) * (1 - t));
                        target =
                            Math.Min(
                                current + MaximumLift,
                                Math.Max(current, math.lerp(profile, current, Numeric.SmoothStep(t))));

                        if (target > current)
                            shelf++;
                    }

                    var height = math.clamp(target, 0, scale);

                    if (Math.Abs(height - current) < 1e-3f)
                        continue;

                    if (height > current)
                        raised++;
                    else
                        lowered++;

                    maximum = Math.Max(maximum, Math.Abs(target - current));
                    result[i] = height;
                    touched = true;
                }

            if (touched)
                shaped++;
        }

        if (raised + lowered == 0)
            return terrain.Heights;

        log.Count("Beach terrain pixels raised", raised);
        log.Count("Beach terrain pixels lowered", lowered);
        log.Evidence(
            $"Beaches: {beaches.Length:N0} mapped; {shaped:N0} shaped by distance to their outline: {raised:N0} terrain points raised and {lowered:N0} lowered, by at most {maximum:0.0} m; edge {edge:0.0} m (effective water {surface:0.0} m + {EdgeHeight}), rising inland at 1:{1 / Slope:0}, {shelf:N0} points of an underwater shelf reaching {ShelfDepth:0} m deep over {Shelf:0} m.");

        return result;
    }
}
