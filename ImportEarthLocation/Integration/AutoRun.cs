using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using Game;
using Game.SceneFlow;
using Game.UI.Menu;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using Unity.Entities;
using UnityEngine;

namespace ImportEarthLocation.Integration;

// Unattended test runs, started by tests/autorun.ps1. When diagnostics/autorun-request.json
// exists at startup, the mod opens the editor and imports each listed city on a new map, with
// Diagnostics on, and reports through autorun-status.json. The request is deleted as soon as it
// is read, so a crash cannot restart a run during normal play. Without it, nothing here acts.
public static class AutoRun
{
    public enum Phase
    {
        Idle,
        WaitingForMap,
        Preparing,
        Applying,
        Settling,
    }

    private const float MapSettleSeconds = 20, RunSettleSeconds = 30, HeartbeatSeconds = 15;
    private const float DiagnosticsLimitSeconds = 1800;

    private sealed class CityRun
    {
        public string Name = "";
        public string Latitude = "";
        public string Longitude = "";
        public string Switches = "";
        public string Result = "pending";
        public string? RunFolder;
        public string? Error;
        public DateTime? StartedUtc;
        public DateTime? FinishedUtc;
    }

    private static readonly List<CityRun> s_Runs = new();
    private static int s_Index;
    private static long s_PreviousMapSession = -1;
    private static float s_WaitUntil;
    private static float s_NextHeartbeat;
    private static string s_Step = "";

    public static Phase Current { get; private set; }

    public static string Latitude => s_Runs[s_Index].Latitude;

    public static string Longitude => s_Runs[s_Index].Longitude;

    public static string Switches => s_Runs[s_Index].Switches;

    private static string Root => Path.Combine(Application.persistentDataPath, "ModsData", Mod.Id, "diagnostics");

    public static void Begin()
    {
        var file = Path.Combine(Root, "autorun-request.json");

        if (!File.Exists(file))
            return;

        try
        {
            var rows = JArray.Parse(File.ReadAllText(file));
            File.Delete(file);

            foreach (var row in rows)
                s_Runs.Add(
                    new CityRun
                    {
                        Name = (string?)row["name"] ?? "city",
                        Latitude = (string)row["lat"]!,
                        Longitude = (string)row["lon"]!,
                        Switches = (string?)row["switches"] ?? ""
                    });

            if (s_Runs.Count == 0)
                return;

            Current = Phase.WaitingForMap;
            WriteStatus("starting");
            GameManager.instance.StartCoroutine(OpenEditorFromMenu());
        }
        catch (Exception exception)
        {
            Current = Phase.Idle;
            Mod.Log.Error("AUTORUN request unreadable: " + exception.Message);
        }
    }

    // The import panel's system only updates in the editor, so this first step runs apart from it.
    private static IEnumerator OpenEditorFromMenu()
    {
        while (GameManager.instance.gameMode != GameMode.MainMenu || GameManager.instance.isGameLoading)
            yield return new WaitForSecondsRealtime(1);

        yield return new WaitForSecondsRealtime(MapSettleSeconds);

        try
        {
            StartNewMap(-1);
        }
        catch (Exception exception)
        {
            Current = Phase.Idle;
            Mod.Log.Error("AUTORUN could not open the editor: " + exception);
            WriteStatus("failed");
        }
    }

    // The same call as the menu's Editor button and, once confirmed, the editor's New Map.
    private static void StartNewMap(long mapSession)
    {
        s_PreviousMapSession = mapSession;
        s_WaitUntil = 0;
        Current = Phase.WaitingForMap;
        World.DefaultGameObjectInjectionWorld.GetExistingSystemManaged<MenuUISystem>().SafeStartEditor();
    }

    // True once a different map than the last one has been loaded and left to settle.
    public static bool MapIsReady(long mapSession)
    {
        if (mapSession == s_PreviousMapSession)
            return false;

        if (s_WaitUntil == 0)
            s_WaitUntil = Time.realtimeSinceStartup + MapSettleSeconds;

        return Time.realtimeSinceStartup >= s_WaitUntil;
    }

    public static void Started()
    {
        s_Runs[s_Index].StartedUtc = DateTime.UtcNow;
        s_Runs[s_Index].Result = "running";
        Current = Phase.Preparing;
        WriteStatus("running");
    }

    public static void Applying(string? runFolder)
    {
        s_Runs[s_Index].RunFolder = runFolder;
        Current = Phase.Applying;
        WriteStatus("running");
    }

    public static void Finished(string? error)
    {
        var run = s_Runs[s_Index];
        run.Result = error == null ? "completed" : "failed";
        run.Error = error;
        run.FinishedUtc = DateTime.UtcNow;
        s_WaitUntil = Time.realtimeSinceStartup + RunSettleSeconds;
        Current = Phase.Settling;
        WriteStatus("running");
    }

    // A new map discards queued diagnostics, which ran over 7 minutes past the last shot on
    // DC, so this waits for them to drain. The limit keeps a stuck check from ending the run.
    public static void AdvanceWhenSettled(long mapSession)
    {
        if (Time.realtimeSinceStartup < s_WaitUntil)
            return;

        if (NativeDiagnostics.Busy && Time.realtimeSinceStartup < s_WaitUntil + DiagnosticsLimitSeconds)
            return;

        if (++s_Index < s_Runs.Count)
        {
            WriteStatus("running");
            StartNewMap(mapSession);
            return;
        }

        s_Index = s_Runs.Count - 1;
        Current = Phase.Idle;
        WriteStatus("done");
    }

    // The script treats a stale heartbeat as a hung game.
    public static void Heartbeat(string step)
    {
        s_Step = step;

        if (Time.realtimeSinceStartup < s_NextHeartbeat)
            return;

        s_NextHeartbeat = Time.realtimeSinceStartup + HeartbeatSeconds;
        WriteStatus("running");
    }

    private static void WriteStatus(string state)
    {
        try
        {
            var file = Path.Combine(Root, "autorun-status.json");
            var temporary = file + ".tmp";
            var status =
                new
                {
                    state,
                    phase = Current.ToString(),
                    city = s_Index,
                    step = s_Step,
                    heartbeatUtc = DateTime.UtcNow,
                    runs = s_Runs
                };
            File.WriteAllText(temporary, JsonConvert.SerializeObject(status, Formatting.Indented));
            File.Delete(file);
            File.Move(temporary, file);
        }
        catch (Exception exception)
        {
            Mod.Log.Warn("AUTORUN status not written: " + exception.Message);
        }
    }
}
