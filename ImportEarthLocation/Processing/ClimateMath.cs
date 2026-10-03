using System;

namespace ImportEarthLocation.Processing;

public static class ClimateMath
{
    // Meteorological 'from' bearings become east/north horizontal flow vectors.
    public static (float east, float north, float speed) PrevailingWind(float[] from, float[] speeds)
    {
        if (from.Length != 12 || speeds.Length != 12) throw new ArgumentException("Twelve monthly wind normals are required.");
        double east = 0, north = 0, speed = 0; var days = new[] { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
        for (var i = 0; i < 12; i++)
        {
            if (!TerrainConditioner.Finite(from[i]) || !TerrainConditioner.Finite(speeds[i]) || speeds[i] < 0 || speeds[i] > 60 || from[i] < 0 || from[i] > 360) throw new ArgumentException("Invalid wind normal.");
            var bearing = from[i] * Math.PI / 180;
            east -= Math.Sin(bearing) * speeds[i] * days[i]; north -= Math.Cos(bearing) * speeds[i] * days[i]; speed += speeds[i] * days[i];
        }
        return ((float)(east / 365), (float)(north / 365), (float)(speed / 365));
    }
}
