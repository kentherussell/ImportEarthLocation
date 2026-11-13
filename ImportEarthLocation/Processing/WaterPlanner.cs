using System;
using System.Collections.Generic;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

public static class WaterPlanner
{
    public static WaterPlan Prepare(
        OsmDataset? data,
        IReadOnlyList<GeographicPolygon> polygons,
        ImportRequest request,
        TerrainBuilder.Result terrain,
        ImportLog log,
        CancellationToken token,
        float waterCellSize = 7,
        Action<string>? progress = null)
        => WaterBodyPlanner.Prepare(data, polygons, request, terrain, log, token, waterCellSize, progress);

    // Clipping also finds crossings with both endpoints outside the map.
    internal static bool Clip(float3 vectorA, float3 vectorB, float half, out float entry, out float exit)
    {
        var low = 0f;
        var high = 1f;

        bool Axis(float start, float delta)
        {
            if (Math.Abs(delta) < .0001f)
                return Math.Abs(start) <= half;

            var t0 = (-half - start) / delta;
            var t1 = (half - start) / delta;
            low = Math.Max(low, Math.Min(t0, t1));
            high = Math.Min(high, Math.Max(t0, t1));

            return low <= high;
        }

        var hit = Axis(vectorA.x, vectorB.x - vectorA.x) && Axis(vectorA.z, vectorB.z - vectorA.z);
        entry = low;
        exit = high;

        return hit && high > low;
    }
}
