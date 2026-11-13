using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Game.SceneFlow;
using Game.Rendering;
using Game.Simulation;
using ImportEarthLocation.Core;
using ImportEarthLocation.Geo;
using ImportEarthLocation.Processing;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace ImportEarthLocation.Integration;

// With Diagnostics on, after an import: moves the camera to each site and saves what
// the screen shows into the run folder, so a run can be read without someone flying
// there. Sites are the plan's worst pieces plus any in diagnostics/sites.json inside
// the map. Takes about 3 s a site and puts the camera back. A failure here only
// loses the pictures.
public static class SiteShots
{
    private const float SettleSeconds = 2.5f, DefaultZoom = 450, Pitch = 50;

    // A site's piece fills about half the view: the zoom is this many times its length,
    // kept between the closest and widest views that still read as one place.
    private const float ZoomPerMetre = 2, ClosestZoom = 250, WidestZoom = 900;
    private const float MiddayHour = 12;
    private const int MaximumSites = 200, RiverSites = 15;

    private sealed record Site(string Name, float3 Point, float Zoom, float Heading);

    private static (bool State, float Value)? s_HeldDate;

    public static bool Active { get; private set; }

    private static ClimateSystem? Climate
        => World.DefaultGameObjectInjectionWorld?.GetExistingSystemManaged<ClimateSystem>();

    // Snow hides the terrain in the pictures, so a Diagnostics import runs in the map's
    // warmest month, set the way the editor's date slider sets it (0-1 through the year).
    // Snow that has already fallen needs time to go, so this starts with the import.
    public static void HoldWarmestMonth(PreparedImport prepared, ImportLog log)
    {
        if (!log.DiagnosticsEnabled || s_HeldDate != null)
            return;

        try
        {
            var climate = Climate;

            if (climate == null)
                return;

            var temperatures = prepared.Climate?.Temperatures;
            var month =
                temperatures is { Length: 12 }
                    ? Array.IndexOf(temperatures, temperatures.Max())
                    : prepared.Request.Latitude < 0 ? 0 : 6;

            var date = climate.currentDate;
            s_HeldDate = (date.overrideState, date.overrideValue);
            date.overrideValue = (month + .5f) / 12;
            log.Diagnostic($"SITE SHOTS holding date at month {month + 1} for the run");
        }
        catch (Exception exception)
        {
            log.Diagnostic("SITE SHOTS date not held: " + exception.GetBaseException().Message);
        }
    }

    // Puts back whatever date override stood before, including the editor slider's.
    public static void ReleaseDate()
    {
        if (s_HeldDate is not { } held)
            return;

        s_HeldDate = null;
        var climate = Climate;

        if (climate == null)
            return;

        climate.currentDate.overrideValue = held.Value;
        climate.currentDate.overrideState = held.State;
    }

    public static void Begin(PreparedImport prepared, ImportLog log)
    {
        if (!log.DiagnosticsEnabled || log.RunFolder == null)
        {
            ReleaseDate();
            return;
        }

        var started = false;

        try
        {
            var sites =
                Listed(prepared, log, Path.GetDirectoryName(Path.GetDirectoryName(log.RunFolder))!)
                    .Concat(
                        NetworkDrawnStructures
                            .Worst(
                                prepared.Segments,
                                prepared.Heights,
                                prepared.Request.MapSize.WidthMeters)
                            .Select(item => new Site(
                                item.Name,
                                item.Point,
                                math.clamp(item.Span * ZoomPerMetre, ClosestZoom, WidestZoom),
                                0))
                            .Concat(
                                NetworkDrawnStructures
                                    .Crowded(prepared.Segments, prepared.Request.MapSize.WidthMeters)
                                    .Select(item => new Site(item.Name, item.Point, DefaultZoom, 0))))
                    .Concat(Rivers(prepared))
                    .Take(MaximumSites)
                    .ToArray();

            if (sites.Length > 0)
            {
                Active = true;
                GameManager.instance.StartCoroutine(Run(sites, prepared, log.RunFolder, log));
                started = true;
            }
        }
        catch (Exception exception)
        {
            log.Diagnostic("SITE SHOTS unavailable: " + exception.GetBaseException().Message);
        }

        if (!started)
            ReleaseDate();
    }

    // Spots along the rivers, spread out, close enough to read channel against pool.
    private static IEnumerable<Site> Rivers(PreparedImport prepared)
    {
        var chosen = new List<float3>();

        foreach (var reach in (prepared.Water?.ReachStages ?? Array.Empty<WaterReachStage>())
            .Where(otherReach => otherReach.PolygonId == null)
            .OrderBy(otherReach => otherReach.BodyId)
            .ThenBy(otherReach => otherReach.Start.x))
        {
            var point = (reach.Start + reach.End) * .5f;

            if (chosen.Any(otherPoint => math.distance(otherPoint.xz, point.xz) < 2500))
                continue;

            chosen.Add(point);

            if (chosen.Count > RiverSites)
                yield break;

            yield return new Site($"river {chosen.Count}", point, 900, 0);
        }
    }

    private static IEnumerable<Site> Listed(PreparedImport prepared, ImportLog log, string diagnostics)
    {
        var file = Path.Combine(diagnostics, "sites.json");

        if (!File.Exists(file))
            yield break;

        var half = (float)prepared.Request.MapSize.WidthMeters / 2;
        JArray rows;

        try
        {
            rows = JArray.Parse(File.ReadAllText(file));
        }
        catch (Exception exception)
        {
            log.Diagnostic("SITE SHOTS sites.json unreadable: " + exception.Message);
            yield break;
        }

        foreach (var row in rows)
        {
            var point =
                GeoMath.ToLocalMeters(
                    (double)row["lat"]!,
                    (double)row["lon"]!,
                    prepared.Request.Latitude,
                    prepared.Request.Longitude);

            if (Math.Abs(point.x) > half || Math.Abs(point.z) > half)
                continue;

            point.y =
                NetworkProcessor.Sample(
                    point.x,
                    point.z,
                    prepared.Heights,
                    prepared.Request.MapSize.WidthMeters);

            yield return new Site(
                (string?)row["name"] ?? "site",
                point,
                (float?)row["zoom"] ?? DefaultZoom,
                (float?)row["heading"] ?? 0);
        }
    }

    private static IEnumerator Run(Site[] sites, PreparedImport prepared, string folder, ImportLog log)
    {
        // The import's closing work runs first; it uses none of the camera.
        yield return new WaitForSecondsRealtime(20);
        var system = World.DefaultGameObjectInjectionWorld?.GetExistingSystemManaged<CameraUpdateSystem>();
        var camera = system?.activeCameraController;

        if (camera == null)
        {
            ReleaseDate();
            Active = false;
            log.Diagnostic("SITE SHOTS no active camera");
            yield break;
        }

        var pivot = camera.pivot;
        var zoom = camera.zoom;
        var rotation = camera.rotation;
        var input = camera.inputEnabled;
        var manifest = new List<object>();
        var taken = 0;
        var cloudiness = Climate?.cloudiness;
        var cloudState = cloudiness?.overrideState ?? false;
        var cloudValue = cloudiness?.overrideValue ?? 0;
        var precipitation = Climate?.precipitation;
        var rainState = precipitation?.overrideState ?? false;
        var rainValue = precipitation?.overrideValue ?? 0;
        var planetary = World.DefaultGameObjectInjectionWorld?.GetExistingSystemManaged<PlanetarySystem>();
        var timeOverride = planetary?.overrideTime ?? false;
        var timeValue = planetary?.time ?? 0;

        try
        {
            camera.inputEnabled = false;
            BeachSurfaceProbe.LogSettings(log);

            if (cloudiness != null)
                cloudiness.overrideValue = 0;

            if (precipitation != null)
                precipitation.overrideValue = 0;

            // Clear midday: low sun's long shadows and rain streaks hid the roads.
            if (planetary != null)
            {
                planetary.overrideTime = true;
                planetary.time = MiddayHour;
            }

            for (var i = 0; i < sites.Length; i++)
            {
                var site = sites[i];
                var name = $"site-{i:D2}.jpg";
                var failure = "";

                try
                {
                    // No panel covered the right of the screen in the 11-10 runs' shots, so
                    // the site sits in the middle.
                    camera.pivot = site.Point;
                    camera.zoom = site.Zoom;
                    camera.rotation = new Vector3(Pitch, site.Heading, 0);
                }
                catch (Exception exception)
                {
                    failure = exception.GetBaseException().Message;
                }

                if (failure.Length == 0)
                {
                    yield return new WaitForSecondsRealtime(SettleSeconds);
                    yield return new WaitForEndOfFrame();

                    failure = Capture(Path.Combine(folder, name));

                    if (failure.Length == 0)
                        taken++;
                }

                if (failure.Length == 0 && BeachSurfaceProbe.IsBeach(site.Name))
                {
                    try
                    {
                        BeachSurfaceProbe.Sample(prepared, site.Point, site.Name, log);
                    }
                    catch (Exception exception)
                    {
                        log.Diagnostic($"BEACH PROBE site=\"{site.Name}\" failed: {exception.GetBaseException().Message}");
                    }

                    // Also pictured with Sand's dirt override at zero, in case full-strength
                    // paint alone does not show under water.
                    var restore = BeachSurfaceProbe.ClearSandDirtOverride();

                    try
                    {
                        yield return new WaitForSecondsRealtime(SettleSeconds);
                        yield return new WaitForEndOfFrame();
                        var variant = $"site-{i:D2}-no-dirt-override.jpg";
                        var variantFailure = Capture(Path.Combine(folder, variant));
                        log.Diagnostic($"BEACH PROBE site=\"{site.Name}\" variant={variant} {(variantFailure.Length == 0 ? "taken" : variantFailure)}");
                    }
                    finally
                    {
                        restore();
                    }
                }

                var geo =
                    (
                        lat: prepared.Request.Latitude + site.Point.z / 6378137 * 180 / Math.PI,
                        lon: prepared.Request.Longitude
                            + site.Point.x / (6378137 * Math.Cos(prepared.Request.Latitude * Math.PI / 180)) * 180
                                / Math.PI);

                manifest.Add(
                    new
                    {
                        file = name,
                        site.Name,
                        geo.lat,
                        geo.lon,
                        x = site.Point.x,
                        y = site.Point.y,
                        z = site.Point.z,
                        site.Zoom,
                        site.Heading,
                        // The on-screen clock follows the simulation, not this override.
                        sunHour = planetary?.time,
                        failure
                    });
            }
        }
        finally
        {
            camera.pivot = pivot;
            camera.zoom = zoom;
            camera.rotation = rotation;
            camera.inputEnabled = input;

            if (cloudiness != null)
            {
                cloudiness.overrideValue = cloudValue;
                cloudiness.overrideState = cloudState;
            }

            if (precipitation != null)
            {
                precipitation.overrideValue = rainValue;
                precipitation.overrideState = rainState;
            }

            if (planetary != null)
            {
                planetary.time = timeValue;
                planetary.overrideTime = timeOverride;
            }

            ReleaseDate();

            try
            {
                File.WriteAllText(
                    Path.Combine(folder, "site-shots.json"),
                    JsonConvert.SerializeObject(manifest, Formatting.Indented));
            }
            catch (Exception) { }

            log.Diagnostic($"SITE SHOTS taken={taken}/{sites.Length}");
            Active = false;
        }
    }

    // Saves what the screen shows; returns the failure, or "" when saved.
    private static string Capture(string path)
    {
        try
        {
            var shot = ScreenCapture.CaptureScreenshotAsTexture();

            try
            {
                File.WriteAllBytes(path, shot.EncodeToJPG(80));
                return "";
            }
            finally
            {
                UnityEngine.Object.Destroy(shot);
            }
        }
        catch (Exception exception)
        {
            return exception.GetBaseException().Message;
        }
    }
}
