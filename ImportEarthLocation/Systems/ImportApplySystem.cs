using System;
using System.Collections.Generic;
using System.Diagnostics;
using Game;
using Game.Prefabs;
using Game.Simulation;
using Game.SceneFlow;
using ImportEarthLocation.Core;
using ImportEarthLocation.Integration;
using Unity.Entities;

namespace ImportEarthLocation.Systems
{

public sealed partial class ImportApplySystem : Game.GameSystemBase
{
    private static PreparedImport? s_Pending;
    private static ImportLog? s_Log;
    private static readonly object Gate = new();
    private static ImportApplySystem? s_Instance;

    private PrefabSystem _prefabSystem = null!;
    private TerrainBridge _terrain = null!;
    private IReadOnlyList<Entity>? _definitions;
    private int _phase;
    private int _created;
    private int _skipped;
    private Entity[] _treePrefabs = Array.Empty<Entity>();
    private int _treeIndex;
    private int _roadSections, _railSections;
    private readonly Stopwatch _vegetationClock = new();

    public static event Action<string>? Progress;
    public static event Action<int, int>? Completed;
    public static event Action<string>? Failed;

    public static bool TryRequest(PreparedImport prepared, ImportLog log)
    {
        lock (Gate)
        {
            if (s_Pending != null || !ImportSession.Matches(prepared.Request.MapSessionId, prepared.Request.Generation)) return false;
            s_Pending = prepared;
            s_Log = log;
            return true;
        }
    }

    protected override void OnCreate()
    {
        base.OnCreate();
        s_Instance = this;
        _prefabSystem = World.GetOrCreateSystemManaged<PrefabSystem>();
        _terrain = new TerrainBridge(
            World.GetOrCreateSystemManaged<TerrainSystem>(),
            World.GetOrCreateSystemManaged<WaterSystem>());
    }

    protected override void OnUpdate()
    {
        PreparedImport? prepared;
        lock (Gate) prepared = s_Pending;
        if (prepared == null) return;
        if (!ImportSession.Matches(prepared.Request.MapSessionId, prepared.Request.Generation) || GameManager.instance.isGameLoading || GameManager.instance.gameMode != GameMode.Editor)
        { s_Log?.Warn("Stale previous-map placement rejected."); AbortMapWork(); return; }

        try
        {
            if (_phase == 0)
            {
                Progress?.Invoke("Replacing terrain and clearing existing networks/water sources…");
                if (GameManager.instance.gameMode != GameMode.Editor)
                    throw new InvalidOperationException("The game left Map Editor mode before confirmation.");
                if (!MapExtDetector.TryGetCurrentMapWidth(out var actualWidth) ||
                    Math.Abs(actualWidth - prepared.Request.MapSize.WidthMeters) > 2d)
                    throw new InvalidOperationException("The loaded map size changed or could not be verified after preflight.");
                if (!_terrain.CanApply(out var terrainReason)) throw new InvalidOperationException(terrainReason);
                if (!NetworkCommitSystem.CanCommit(out var networkReason) && prepared.Segments.Count > 0)
                    throw new InvalidOperationException(networkReason);

                var resolver = new PrefabResolver(_prefabSystem, s_Log!);
                if (System.Linq.Enumerable.Any(prepared.Segments, segment => segment.Kind == NetworkKind.Road) &&
                    !resolver.CanResolveRoad(out var roadReason))
                    throw new InvalidOperationException(roadReason);
                if (System.Linq.Enumerable.Any(prepared.Segments, segment => segment.Kind == NetworkKind.Rail) &&
                    !resolver.CanResolveRail(out var railReason))
                    throw new InvalidOperationException(railReason);

                var deleted = MapContentCleaner.MarkForDeletion(EntityManager);
                s_Log?.Info($"Marked {deleted.networks} existing network entities and {deleted.waterSources} water sources for deletion.");
                var terrainClock = Stopwatch.StartNew();
                _terrain.Apply(prepared);
                s_Log?.Time("Terrain apply", terrainClock.Elapsed.TotalSeconds);
                s_Log?.Info($"Sea level: nominal {prepared.SeaLevelMeters:0.0} m; effective water {WaterLevels.Effective(prepared.SeaLevelMeters):0.0} m (+{WaterLevels.CorrectionMeters:0.0} m).");

                var placement = new NativeNetworkPlacement(EntityManager, _prefabSystem, s_Log!);
                _definitions = placement.CreateDefinitions(prepared.Segments, out _skipped);
                _created = _definitions.Count;
                _roadSections = placement.RoadSections; _railSections = placement.RailSections;
                s_Log?.Info($"Queued {_created} native NetCourse definitions; {_skipped} segments skipped safely.");
                if (_created == 0) { _phase = 3; return; }
                _phase = 1;
                return;
            }

            if (_phase == 1)
            {
                // Definition entities must survive one GenerateNodes/GenerateEdges pass, then be removed
                // before the second pass. Temp network entities remain for ApplyNetSystem.
                // NOTE: EntityManager.Exists() is intentionally not used here: with the
                // stock CS2 + Entities toolchain it breaks the UpdateAt<SystemType>
                // generic constraint (CS0311 on Mod.cs). DestroyEntity(Entity) compiles
                // and suffices because phase 1 runs exactly once per import.
                foreach (var definition in _definitions!)
                    EntityManager.DestroyEntity(definition);
                NetworkCommitSystem.RequestCommit(prepared.Request.MapSessionId, prepared.Request.Generation);
                Progress?.Invoke("Committing roads and rail through the native network systems…");
                _phase = 2;
                return;
            }

            if (_phase == 2 && NetworkCommitSystem.TryConsumeResult(out var complete, out var failure) && complete)
            {
                if (failure != null) throw new InvalidOperationException(failure);
                if (prepared.Request.RoadLevel != RoadImportLevel.None) s_Log?.Count("Road sections committed", _roadSections);
                if (prepared.Request.ImportRail) s_Log?.Count("Rail sections committed", _railSections);
                _phase = 3;
                return;
            }

            if (_phase == 3)
            {
                if (_treeIndex >= prepared.Trees.Count) { Finish(); return; }
                if (_treeIndex == 0)
                {
                    _vegetationClock.Restart();
                    _treePrefabs = NativeVegetationPlacement.ResolvePrefabs(EntityManager, _prefabSystem);
                    if (_treePrefabs.Length == 0) { s_Log?.Warn("Optional vegetation skipped: no tree prefabs are available."); s_Log?.Count("Trees skipped", prepared.Trees.Count); Finish(); return; }
                }
                Progress?.Invoke($"Applying optional forest trees: {_treeIndex:N0}/{prepared.Trees.Count:N0} candidates…");
                var count = Math.Min(512, prepared.Trees.Count - _treeIndex);
                _definitions = NativeVegetationPlacement.CreateBatch(EntityManager, prepared.Trees, _treeIndex, count, _treePrefabs, s_Log!);
                _treeIndex += count;
                if (count >= 32 && _definitions.Count < count / 2)
                {
                    foreach (var definition in _definitions) EntityManager.DestroyEntity(definition);
                    s_Log?.Count("Trees skipped", _definitions.Count + prepared.Trees.Count - _treeIndex);
                    s_Log?.Warn("Optional vegetation stopped because most candidates in a batch failed placement. Terrain and networks remain imported.");
                    Finish(); return;
                }
                _phase = _definitions.Count == 0 ? 3 : 4;
                return;
            }
            if (_phase == 4)
            {
                foreach (var definition in _definitions!) EntityManager.DestroyEntity(definition);
                NetworkCommitSystem.RequestCommit(prepared.Request.MapSessionId, prepared.Request.Generation); _phase = 5;
                return;
            }
            if (_phase == 5 && NetworkCommitSystem.TryConsumeResult(out var treesComplete, out var treeFailure) && treesComplete)
            {
                if (treeFailure != null) throw new InvalidOperationException(treeFailure);
                s_Log?.Count("Tree definitions committed", _definitions!.Count);
                _phase = 3;
            }
        }
        catch (Exception exception)
        {
            var message = exception.GetBaseException().Message;
            if (_phase >= 3)
            {
                s_Log?.Warn($"Optional vegetation application stopped: {message}. Terrain and networks remain imported.");
                s_Log?.Count("Trees skipped", Math.Max(0, prepared.Trees.Count - _treeIndex));
                s_Log?.Count("Unconfirmed tree definitions", _definitions?.Count ?? 0);
                Finish(); return;
            }
            s_Log?.Error(message);
            Failed?.Invoke(message);
            Reset();
        }
    }

    private void Finish()
    {
        var prepared = s_Pending;
        if (prepared != null && !ImportSession.Matches(prepared.Request.MapSessionId, prepared.Request.Generation)) { AbortMapWork(); return; }
        if (prepared != null)
        {
            var reports = new List<string>
            {
                $"Location: {prepared.Request.Latitude:0.######}, {prepared.Request.Longitude:0.######}; {prepared.Request.MapSize.Label}.",
                prepared.Water?.Report ?? "Water: native sea level set; inland sources not verified."
            };
            if (prepared.Request.ApplyGroundSurfaces)
            {
                Progress?.Invoke("Applying ground surfaces…");
                try
                {
                    reports.Add(prepared.SurfaceMask == null ? "Ground surfaces skipped: classification unavailable." :
                        NativeSurfaceBridge.Apply(prepared.SurfaceMask, EntityManager, _prefabSystem, World.GetOrCreateSystemManaged<Game.Rendering.TerrainMaterialSystem>(), World.GetOrCreateSystemManaged<TerrainSystem>(), s_Log!));
                }
                catch (Exception exception) { reports.Add("Ground surfaces stopped safely: " + exception.GetBaseException().Message); }
            }
            else reports.Add("Ground surfaces: not requested.");
            if (prepared.Request.CreateAirConnections)
            {
                Progress?.Invoke("Creating air connections…");
                try { reports.Add(NativeAirBridge.Apply(prepared, EntityManager, _prefabSystem, World.GetOrCreateSystemManaged<Game.Net.AirwaySystem>(), World.GetOrCreateSystemManaged<Game.UI.NameSystem>(), s_Log!)); }
                catch (Exception exception) { reports.Add("Air connections skipped: " + exception.GetBaseException().Message); }
            }
            else reports.Add("Air connections: not requested.");
            Progress?.Invoke("Assigning climate…");
            try { reports.Add(NativeClimateBridge.Apply(prepared, EntityManager, _prefabSystem, World.GetOrCreateSystemManaged<ClimateSystem>(), World.GetOrCreateSystemManaged<WindSimulationSystem>(), s_Log!)); }
            catch (Exception exception) { reports.Add("Climate assignment failed; existing/default preserved: " + exception.GetBaseException().Message); }
            prepared.FeatureReport = string.Join("\n", reports);
            foreach (var report in reports)
                if (report.IndexOf("skipped", StringComparison.OrdinalIgnoreCase) >= 0 || report.IndexOf("failed", StringComparison.OrdinalIgnoreCase) >= 0 || report.IndexOf("stopped", StringComparison.OrdinalIgnoreCase) >= 0) s_Log?.Warn(report);
                else s_Log?.Info(report);
        }
        if (_vegetationClock.IsRunning) { _vegetationClock.Stop(); s_Log?.Time("Vegetation apply", _vegetationClock.Elapsed.TotalSeconds); }
        s_Log?.Info($"Import complete. Native definitions: {_created}; skipped: {_skipped}.");
        Completed?.Invoke(_created, _skipped);
        Reset();
    }

    private void Reset()
    {
        lock (Gate) { s_Pending = null; s_Log = null; }
        _definitions = null;
        _phase = 0;
        _created = 0;
        _skipped = 0;
        _treeIndex = 0; _treePrefabs = Array.Empty<Entity>(); _vegetationClock.Reset();
        _roadSections = _railSections = 0;
    }

    protected override void OnGamePreload(Colossal.Serialization.Entities.Purpose purpose, GameMode mode)
    { base.OnGamePreload(purpose, mode); AbortMapWork(); }

    protected override void OnDestroy()
    { AbortMapWork(); if(s_Instance==this)s_Instance=null; base.OnDestroy(); }

    public static void AbortPending() { if(s_Instance!=null)s_Instance.AbortMapWork(); else { lock(Gate){s_Pending=null;s_Log=null;} NetworkCommitSystem.CancelPending(); } }

    public void AbortMapWork()
    {
        // Clear static ownership FIRST. Only touch our recorded helper definitions; never
        // query/delete trees or networks belonging to a newly loaded map.
        var definitions = _definitions; var log = s_Log;
        Reset(); NetworkCommitSystem.CancelPending();
        if (definitions == null) return;
        try
        {
            EntityManager.CompleteAllTrackedJobs();
            foreach (var entity in definitions)
                if (EntityManager.HasComponent<Game.Tools.CreationDefinition>(entity)) EntityManager.DestroyEntity(entity);
        }
        catch (Exception exception) { log?.Warn("Owned temporary-definition cleanup failed; session already invalidated: " + exception.GetBaseException().Message); }
    }
}
}
