using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using ImportEarthLocation.Core;
using ImportEarthLocation.Geo;
using Newtonsoft.Json.Linq;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

public static class AirPlanner
{
    public const string EligibilityVersion = "nearest-exterior-v2";
    public static IReadOnlyList<AirDestination> Select(ImportRequest request, ImportLog log)
    {
        using var stream = Assembly.GetExecutingAssembly().GetManifestResourceStream("ImportEarthLocation.Data.cities.json") ?? throw new InvalidOperationException("Bundled city dataset is unavailable.");
        using var reader = new StreamReader(stream);
        var root = JObject.Parse(reader.ReadToEnd());
        var bounds = GeoMath.BoundsFromCenter(request.Latitude, request.Longitude, request.MapSize.WidthMeters);
        var candidates = new List<(string name, string country, string region, double distance, double bearing, bool primary)>();
        var rejected = 0;
        foreach (var row in (JArray)root["cities"]!)
        {
            if (!(row is JArray record) || record.Count != 9 || string.IsNullOrWhiteSpace((string?)row[0]) || string.IsNullOrWhiteSpace((string?)row[1])) { rejected++; continue; }
            if (record[3].Type != JTokenType.Float && record[3].Type != JTokenType.Integer || record[4].Type != JTokenType.Float && record[4].Type != JTokenType.Integer || record[5].Type != JTokenType.Integer || record[6].Type != JTokenType.Integer || record[7].Type != JTokenType.Boolean)
            { rejected++; continue; }
            var lat = (double)record[3]; var lon = (double)record[4]; var populationValue = (double)record[5]; var rankValue = (double)record[6]; var capital = (bool)record[7];
            if (populationValue < 0 || populationValue > int.MaxValue || rankValue < 0 || rankValue > 20) { rejected++; continue; }
            var population = (int)populationValue; var rank = (int)rankValue;
            if (!Data.LocationServices.Valid(lat, lon) || population < 0 || rank < 0) { rejected++; continue; }
            var longitudeDelta = ((lon - request.Longitude + 540) % 360) - 180;
            var halfLongitudeSpan = (bounds.East - bounds.West) / 2;
            if (lat >= bounds.South && lat <= bounds.North && Math.Abs(longitudeDelta) <= halfLongitudeSpan) continue;
            var primary = capital || population >= 500000 || rank <= 3;
            var a = request.Latitude * Math.PI / 180; var b = lat * Math.PI / 180; var dl = (lon - request.Longitude) * Math.PI / 180;
            var h = Math.Pow(Math.Sin((b - a) / 2), 2) + Math.Cos(a) * Math.Cos(b) * Math.Pow(Math.Sin(dl / 2), 2);
            var distance = 6371.0088 * 2 * Math.Asin(Math.Sqrt(Math.Min(1, Math.Max(0, h))));
            var bearing = (Math.Atan2(Math.Sin(dl) * Math.Cos(b), Math.Cos(a) * Math.Sin(b) - Math.Sin(a) * Math.Cos(b) * Math.Cos(dl)) * 180 / Math.PI + 360) % 360;
            candidates.Add(((string)row[0]!, (string)row[1]!, (string?)row[2] ?? "", distance, bearing, primary));
        }
        var eligible = candidates.OrderBy(c => c.distance).ThenBy(c => c.name, StringComparer.Ordinal).ThenBy(c => c.country, StringComparer.Ordinal).ThenBy(c => c.region, StringComparer.Ordinal).Take(20).ToArray();
        var duplicatedNames = new HashSet<string>(eligible.GroupBy(c => c.name).Where(g => g.Count() > 1).Select(g => g.Key));
        var result = eligible.Select(c => new AirDestination(duplicatedNames.Contains(c.name) ? string.Join(", ", new[] { c.name, c.region, c.country }.Where(s => s.Length > 0).Distinct()) : c.name, c.distance, c.bearing, float3.zero)).ToArray();
        log.Info($"Air destinations: {result.Length} eligible cities; {rejected} invalid records; {root["version"]}; {EligibilityVersion}. No runtime city lookup.");
        log.Count("Air city records invalid", rejected);
        log.Count("Air exterior cities eligible", candidates.Count);
        log.Count("Air destinations selected", result.Length);
        log.DiagnosticRecords("air-destination-selection.ndjson", eligible.Select((c, rank) => (object)new
        { selectionVersion = EligibilityVersion, rank = rank + 1, c.name, c.country, c.region, c.distance, c.bearing }));
        return result;
    }

    // Continuous square perimeter. Keep nearest-first acceptance, but enforce circular ordering of accepted bearings.
    // Reject congestion instead of displacing a destination to an unrelated side.
    public static IReadOnlyList<AirDestination> Place(IReadOnlyList<AirDestination> selected, double halfWidth, float height, double minimumSpacing = 200)
    {
        if (halfWidth <= 0 || double.IsNaN(halfWidth) || double.IsInfinity(halfWidth) || !TerrainConditioner.Finite(height) || minimumSpacing <= 0 || double.IsNaN(minimumSpacing) || double.IsInfinity(minimumSpacing)) throw new ArgumentException("Invalid native airway perimeter/spacing/altitude.");
        var perimeter = halfWidth * 8;
        var placed = new List<(AirDestination destination, double coordinate)>();
        foreach (var city in selected)
        {
            if (string.IsNullOrWhiteSpace(city.Name) || double.IsNaN(city.Bearing) || double.IsInfinity(city.Bearing) || city.Bearing < 0 || city.Bearing >= 360) continue;
            var angle = city.Bearing * Math.PI / 180;
            var x = Math.Sin(angle); var z = Math.Cos(angle); var scale = halfWidth / Math.Max(Math.Abs(x), Math.Abs(z));
            x *= scale; z *= scale;
            var ideal = z >= halfWidth - .001 ? x + halfWidth : x >= halfWidth - .001 ? halfWidth * 2 + halfWidth - z : z <= -halfWidth + .001 ? halfWidth * 4 + halfWidth - x : halfWidth * 6 + z + halfWidth;
            bool Free(double s) => placed.All(p => CircularDistance(p.coordinate, s, perimeter) >= minimumSpacing);
            var chosen = ideal;
            if (!Free(chosen))
            {
                // Zero displacement keeps exact city bearing/order. Prefer fewer cities to congestion.
                continue;
            }
            var side = (int)(chosen / (halfWidth * 2)); var offset = chosen - side * halfWidth * 2;
            var position = side switch { 0 => new float3((float)(offset - halfWidth), height, (float)halfWidth),
                1 => new float3((float)halfWidth, height, (float)(halfWidth - offset)),
                2 => new float3((float)(halfWidth - offset), height, (float)-halfWidth),
                _ => new float3((float)-halfWidth, height, (float)(offset - halfWidth)) };
            placed.Add((city with { Position = position }, chosen));
        }
        return placed.Select(p => p.destination).ToArray();
    }
    private static double CircularDistance(double a, double b, double circumference) => Math.Min(Math.Abs(a - b), circumference - Math.Abs(a - b));
}
