using System;
using System.Linq;
using System.Reflection;
using Game.Prefabs;
using Game.Prefabs.Climate;
using Game.Simulation;
using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

namespace ImportEarthLocation.Integration;

public static class NativeClimateBridge
{
    public static string Apply(PreparedImport prepared, Unity.Entities.EntityManager manager, PrefabSystem prefabs, ClimateSystem climate, WindSimulationSystem wind, ImportLog log)
    {
        var normals = prepared.Climate;
        log.Diagnostic($"CLIMATE BEGIN normalsAvailable={normals!=null}; stage=source-and-prefab-selection");
        if (normals == null) return "Climate: existing/default preserved; climatology unavailable.";
        var centerHeight = NetworkProcessor.Sample(0, 0, prepared.Heightmap, prepared.HeightScaleMeters, prepared.Request.MapSize.WidthMeters) - prepared.SeaLevelMeters;
        // Limited altitude adjustment: 6.5 C/km, at most 4 C; never a full atmospheric model.
        var correction = (float)Numeric.Clamp((centerHeight - normals.SourceElevation) * -.0065, -4d, 4d);
        log.Diagnostic($"CLIMATE SOURCE temperatureC=[{string.Join(",",normals.Temperatures.Select(v=>v.ToString("R",System.Globalization.CultureInfo.InvariantCulture)))}]; precipitationMmPerDay=[{string.Join(",",normals.Precipitation.Select(v=>v.ToString("R",System.Globalization.CultureInfo.InvariantCulture)))}]; windSpeedMps=[{string.Join(",",normals.WindSpeed.Select(v=>v.ToString("R",System.Globalization.CultureInfo.InvariantCulture)))}]; windFromDegrees=[{string.Join(",",normals.WindFrom.Select(v=>v.ToString("R",System.Globalization.CultureInfo.InvariantCulture)))}]; centerHeight={centerHeight:R}; sourceElevation={normals.SourceElevation:R}; selectionAltitudeAdjustmentC={correction:R}; source={normals.SourceVersion}");
        var candidates = NativePrefabCatalog.Read<ClimatePrefab>(manager, prefabs).Where(p => p.m_Seasons.Length > 0).ToArray();
        var scores = candidates.Select(p => (prefab: p, score: Score(p, normals, correction))).Where(p => !double.IsNaN(p.score) && !double.IsInfinity(p.score)).OrderBy(p => p.score).ThenBy(p => p.prefab.name, StringComparer.Ordinal).ToArray();
        if (scores.Length == 0) return "Climate: existing/default preserved; no compatible native climate prefab.";
        var selected = scores[0].prefab;
        log.Diagnostic($"CLIMATE SELECT prefab={selected.name}; numericalScore={scores[0].score:R}; compatibleCandidates={scores.Length}; stage=native-assignment; temperaturesNotDirectlyOverwritten=true");
        var previousClimate = climate.currentClimate; var previousWind = wind.constantWind;
        var pressureProperty = typeof(WindSimulationSystem).GetProperty("m_ConstantPressure", BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
        var previousPressure = pressureProperty?.GetValue(wind) is float pressure && TerrainConditioner.Finite(pressure) ? (float?)pressure : null;
        var windChanged = false;
        try
        {
            // Same public setter used by ClimatePanelSystem. Never mutate a shared prefab/season/weather asset.
            climate.currentClimate = prefabs.GetEntity(selected);
            NativeDiagnostics.ClimateApplied=climate.currentClimate==prefabs.GetEntity(selected);
            NativeDiagnostics.RememberClimate(selected.name);
            log.Diagnostic($"CLIMATE READBACK prefabMatch={NativeDiagnostics.ClimateApplied}; requestedPrefab={selected.name}");
            var (east, north, speed) = PrevailingWind(normals.WindFrom, normals.WindSpeed);
            var vector = new float2(east, north);
            if (math.lengthsq(vector) > .01f)
            {
                // Native editor direction assigns a unit horizontal flow vector and pressure 40.
                // There is no verified m/s -> simulation-strength mapping, so retain native magnitude/pressure.
                var magnitude = math.length(previousWind);
                if (math.isfinite(magnitude) && magnitude > 0 && magnitude <= 1.5f && previousPressure.HasValue)
                {
                    var expected=math.normalize(vector)*magnitude;
                    wind.SetWind(expected, previousPressure.Value); windChanged = true;
                    log.Diagnostic($"CLIMATE WIND READBACK expectedVector={expected}; actualVector={wind.constantWind}; vectorMatch={math.distance(expected,wind.constantWind)<.0001f}; expectedNativeMagnitude={magnitude:R}; actualNativeMagnitude={math.length(wind.constantWind):R}; expectedPressure={previousPressure.Value:R}; actualPressure={pressureProperty?.GetValue(wind)}; physicalMpsNotAssigned=true");
                }
                else log.Warn("Wind direction preserved: native wind magnitude was outside the verified existing state.");
            }
            var from = (Math.Atan2(-east, -north) * 180 / Math.PI + 360) % 360;
            var windReport = windChanged ? $"Applied prevailing wind direction from {from:0}°" : "Existing wind direction preserved (calm/cancelling normals or unverified native state)";
            var report = $"Climate prefab: {selected.name}; closest numerical seasonal match (temperature + precipitation), altitude adjustment {correction:0.0}°C. {windReport}; {speed:0.0} m/s climatology; native wind strength retained (physical speed mapping unverified). Source: {normals.SourceVersion}.";
            log.Info(report); return report;
        }
        catch
        {
            if (previousClimate != Unity.Entities.Entity.Null) climate.currentClimate = previousClimate;
            if (windChanged) wind.SetWind(previousWind, previousPressure!.Value);
            throw;
        }
    }

    private static double Score(ClimatePrefab prefab, ClimateNormals normals, float correction)
    {
        var curves = prefab.CreateSeasonTemperatureCurves(); var rain = prefab.CreateSeasonPrecipCurves();
        var total = 0d;
        for (var month = 0; month < 12; month++)
        {
            var time = month + .5f; // Verified native curves use a 12-month year, not normalized 0..1.
            var temperature = (curves.nightMin.Evaluate(time) + curves.nightMax.Evaluate(time) + curves.dayMin.Evaluate(time) + curves.dayMax.Evaluate(time)) * .25f;
            var nativeRain = rain.precipChance.Evaluate(time) * (rain.precipAmountMin.Evaluate(time) + rain.precipAmountMax.Evaluate(time)) * .5f;
            if (!TerrainConditioner.Finite(temperature) || !TerrainConditioner.Finite(nativeRain) || nativeRain < 0 || nativeRain > 1) return double.PositiveInfinity;
            var difference = temperature - normals.Temperatures[month] - correction;
            // Rank prefab wetness/seasonality; mm/day is NOT assigned as a native rain parameter.
            var targetRain = 1 - Math.Exp(-normals.Precipitation[month] / 4);
            total += difference * difference + 50 * (nativeRain - targetRain) * (nativeRain - targetRain);
        }
        return total;
    }
    public static (float east, float north, float speed) PrevailingWind(float[] from, float[] speeds)
        => ClimateMath.PrevailingWind(from, speeds);
}
