using System;
using System.Collections.Generic;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Geo;

public static class GeoMath
{
    private const double EarthRadius = 6_378_137d;

    public static GeoBounds BoundsFromCenter(double latitude, double longitude, double widthMeters)
    {
        var half = widthMeters / 2d;
        var latDelta = RadiansToDegrees(half / EarthRadius);
        var cosLat = Math.Cos(DegreesToRadians(latitude));
        var lonDelta = RadiansToDegrees(half / (EarthRadius * Math.Max(0.01d, cosLat)));
        return new GeoBounds(latitude - latDelta, longitude - lonDelta, latitude + latDelta, longitude + lonDelta);
    }

    public static IReadOnlyList<GeoBounds> Split4x4(GeoBounds bounds)
    {
        var result = new List<GeoBounds>(16);
        var latStep = (bounds.North - bounds.South) / 4d;
        var lonStep = (bounds.East - bounds.West) / 4d;
        for (var row = 0; row < 4; row++)
        for (var column = 0; column < 4; column++)
        {
            var south = bounds.South + row * latStep;
            var west = bounds.West + column * lonStep;
            result.Add(new GeoBounds(south, west, south + latStep, west + lonStep));
        }
        return result;
    }

    public static float3 ToLocalMeters(double latitude, double longitude, double centerLatitude, double centerLongitude)
    {
        var lat0 = DegreesToRadians(centerLatitude);
        var x = EarthRadius * DegreesToRadians(longitude - centerLongitude) * Math.Cos(lat0);
        var z = EarthRadius * DegreesToRadians(latitude - centerLatitude);
        return new float3((float)x, 0f, (float)z);
    }

    public static double LongitudeToTileX(double longitude, int zoom) =>
        (longitude + 180d) / 360d * (1 << zoom);

    public static double LatitudeToTileY(double latitude, int zoom)
    {
        var lat = DegreesToRadians(Numeric.Clamp(latitude, -85.05112878d, 85.05112878d));
        var tangent = Math.Tan(lat);
        var asinh = Math.Log(tangent + Math.Sqrt(tangent * tangent + 1d));
        return (1d - asinh / Math.PI) / 2d * (1 << zoom);
    }

    public static int SelectTerrainZoom(double widthMeters) => widthMeters switch
    {
        <= 14_336d => 14,
        <= 28_672d => 13,
        _ => 12
    };

    private static double DegreesToRadians(double value) => value * Math.PI / 180d;
    private static double RadiansToDegrees(double value) => value * 180d / Math.PI;
}
