using System;
using System.Reflection;
using Game;
using Game.Common;
using Game.Net;
using Game.Tools;
using Unity.Entities;
using ImportEarthLocation.Core;

namespace ImportEarthLocation.Systems
{

[UpdateAfter(typeof(GenerateEdgesSystem))]
[UpdateBefore(typeof(ToolOutputSystem))]
public sealed partial class NetworkCommitSystem : GameSystemBase
{
    private static bool s_Requested;
    private static int s_WaitFrames;
    private static string? s_Failure;
    private static long s_Map, s_Generation;
    private ToolSystem _toolSystem = null!;
    private FieldInfo? _lastToolField;
    private MethodInfo? _applyModeSetter;

    public static void RequestCommit(long map, long generation)
    {
        if (!ImportSession.Matches(map, generation)) return;
        s_Map = map; s_Generation = generation;
        s_Requested = true;
        s_WaitFrames = 0;
        s_Failure = null;
    }

    public static void CancelPending() { s_Requested = false; s_WaitFrames = 0; s_Failure = null; s_Map = s_Generation = 0; }

    protected override void OnGamePreload(Colossal.Serialization.Entities.Purpose purpose, GameMode mode)
    { base.OnGamePreload(purpose, mode); CancelPending(); }
    protected override void OnDestroy() { CancelPending(); base.OnDestroy(); }

    public static bool TryConsumeResult(out bool complete, out string? failure)
    {
        complete = !s_Requested && (s_WaitFrames < 0 || s_Failure != null);
        failure = s_Failure;
        if (!complete) return false;
        s_WaitFrames = 0;
        s_Failure = null;
        return true;
    }

    public static bool CanCommit(out string reason)
    {
        var property = typeof(ToolBaseSystem).GetProperty("applyMode", BindingFlags.Instance | BindingFlags.Public);
        var field = typeof(ToolSystem).GetField("m_LastTool", BindingFlags.Instance | BindingFlags.NonPublic);
        if (property?.GetSetMethod(nonPublic: true) == null || field == null)
        {
            reason = "The current CS2 tool pipeline no longer exposes the expected native network commit hook.";
            return false;
        }
        reason = "";
        return true;
    }

    protected override void OnCreate()
    {
        base.OnCreate();
        _toolSystem = World.GetOrCreateSystemManaged<ToolSystem>();
        _lastToolField = typeof(ToolSystem).GetField("m_LastTool", BindingFlags.Instance | BindingFlags.NonPublic);
        _applyModeSetter = typeof(ToolBaseSystem).GetProperty("applyMode", BindingFlags.Instance | BindingFlags.Public)?.GetSetMethod(nonPublic: true);
    }

    protected override void OnUpdate()
    {
        if (!s_Requested) return;
        if (!ImportSession.Matches(s_Map, s_Generation) || Game.SceneFlow.GameManager.instance.isGameLoading) { CancelPending(); return; }
        EntityManager.CompleteAllTrackedJobs();
        using var query = EntityManager.CreateEntityQuery(new EntityQueryDesc
        {
            All = new[] { ComponentType.ReadOnly<Temp>() },
            Any = new[] { ComponentType.ReadOnly<Edge>(), ComponentType.ReadOnly<Game.Objects.Tree>() }
        });
        if (query.CalculateEntityCount() == 0)
        {
            if (++s_WaitFrames <= 8) return;
            s_Failure = "CS2 did not generate temporary network/tree entities from the prepared native definitions.";
            s_Requested = false;
            return;
        }

        try
        {
            var lastTool = _lastToolField?.GetValue(_toolSystem) as ToolBaseSystem;
            if (lastTool == null || _applyModeSetter == null)
                throw new InvalidOperationException("The native tool commit hook was unavailable.");
            _applyModeSetter.Invoke(lastTool, new object[] { ApplyMode.Apply });
            s_Requested = false;
            s_WaitFrames = -1;
        }
        catch (Exception exception)
        {
            s_Failure = $"Native network commit failed: {exception.GetBaseException().Message}";
            s_Requested = false;
        }
    }
}
}
