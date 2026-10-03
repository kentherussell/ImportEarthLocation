using System;
using System.Collections.Generic;
using System.Globalization;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Colossal.UI.Binding;
using Game;
using Game.Prefabs;
using Game.SceneFlow;
using Game.Simulation;
using Game.UI;
using ImportEarthLocation.Core;
using ImportEarthLocation.Data;
using ImportEarthLocation.Geo;
using ImportEarthLocation.Integration;
using ImportEarthLocation.Processing;
using UnityEngine.Scripting;
using Newtonsoft.Json.Linq;

namespace ImportEarthLocation.Systems
{

public sealed partial class ImportEarthLocationUISystem : UISystemBase
{
    public const string BindingGroup = Mod.Id;

    private ImportLog _log = new();
    private ValueBinding<bool> _isEditor = null!;
    private ValueBinding<bool> _mapExtDetected = null!;
    private ValueBinding<string> _selectedMapSize = null!;
    private ValueBinding<bool> _panelOpen = null!;
    private ValueBinding<bool> _busy = null!;
    private ValueBinding<bool> _ready = null!;
    private ValueBinding<int> _stage = null!;
    private ValueBinding<string> _status = null!;
    private ValueBinding<string> _error = null!;
    private ValueBinding<string> _summary = null!;
    private ValueBinding<string> _logText = null!;
    private ValueBinding<string> _location = null!;
    private ValueBinding<string> _placeResults = null!;
    private ValueBinding<string> _displayTiles = null!;
    private ValueBinding<string> _roadLevelBinding = null!;
    private ValueBinding<bool> _railBinding = null!;
    private ValueBinding<string> _mapSessionBinding = null!;
    private ValueBinding<string> _actionRevision = null!;
    private long _actionCounter;
    private RoadImportLevel _roadLevel = RoadImportLevel.Primary;
    private bool _importRail = true;
    private long _mapSession;
    private Task<string>? _searchTask;
    private Task<string>? _tilesTask;
    private long _searchMapSession, _tilesMapSession;
    private CancellationTokenSource? _searchCancellation;
    private CancellationTokenSource? _tilesCancellation;
    private DateTime _nextSearch;

    private string _latitude = "";
    private string _longitude = "";
    private string _mapSizeId = MapSizeProfile.Vanilla.Id;
    private bool _includeSecondary;
    private bool _applyGroundSurfaces;
    private bool _createAirConnections;
    private double _lastMapWidth;
    private CancellationTokenSource? _cancellation;
    private Task<DownloadBundle>? _downloadTask;
    private Task<ProcessedBundle>? _processTask;
    private PreparedImport? _prepared;
    private bool _downloadConsumed;
    private bool _processConsumed;
    private volatile bool _logDirty;

    public override GameMode gameMode => GameMode.Editor;

    [Preserve]
    protected override void OnCreate()
    {
        base.OnCreate();
        var preferences = NetworkPreferences.Load(_log); _roadLevel = preferences.Roads; _importRail = preferences.Rail;
        AddBinding(_isEditor = new ValueBinding<bool>(BindingGroup, "isEditor", false));
        AddBinding(_mapExtDetected = new ValueBinding<bool>(BindingGroup, "mapExtDetected", MapExtDetector.HasVerifiedSizeApi));
        AddBinding(_selectedMapSize = new ValueBinding<string>(BindingGroup, "mapSize", _mapSizeId));
        AddBinding(_panelOpen = new ValueBinding<bool>(BindingGroup, "panelOpen", false));
        AddBinding(_busy = new ValueBinding<bool>(BindingGroup, "busy", false));
        AddBinding(_ready = new ValueBinding<bool>(BindingGroup, "ready", false));
        AddBinding(_stage = new ValueBinding<int>(BindingGroup, "stage", (int)ImportStage.Idle));
        AddBinding(_status = new ValueBinding<string>(BindingGroup, "status", "Open a map in the editor to begin."));
        AddBinding(_error = new ValueBinding<string>(BindingGroup, "error", ""));
        AddBinding(_summary = new ValueBinding<string>(BindingGroup, "summary", ""));
        AddBinding(_logText = new ValueBinding<string>(BindingGroup, "log", ""));
        AddBinding(_location = new ValueBinding<string>(BindingGroup, "location", "{}"));
        AddBinding(_placeResults = new ValueBinding<string>(BindingGroup, "placeResults", "{}"));
        AddBinding(_displayTiles = new ValueBinding<string>(BindingGroup, "displayTiles", "{}"));
        AddBinding(_roadLevelBinding = new ValueBinding<string>(BindingGroup, "roadLevel", NetworkSelection.Id(_roadLevel)));
        AddBinding(_railBinding = new ValueBinding<bool>(BindingGroup, "importRail", _importRail));
        _mapSession = ImportSession.Map;
        AddBinding(_mapSessionBinding = new ValueBinding<string>(BindingGroup, "mapSession", _mapSession.ToString(CultureInfo.InvariantCulture)));
        AddBinding(_actionRevision = new ValueBinding<string>(BindingGroup, "actionRevision", "0"));
        AddBinding(new TriggerBinding<string>(BindingGroup, "setRoadLevel", value => { if (_busy.value) return; _roadLevel = NetworkSelection.Parse(value); _roadLevelBinding.Update(NetworkSelection.Id(_roadLevel)); InvalidatePrepared(); new NetworkPreferences(_roadLevel,_importRail).Save(_log); }));
        AddBinding(new TriggerBinding<bool>(BindingGroup, "setImportRail", value => { if (_busy.value) return; _importRail = value; _railBinding.Update(value); InvalidatePrepared(); new NetworkPreferences(_roadLevel,_importRail).Save(_log); }));
        AddBinding(new TriggerBinding<string>(BindingGroup, "commitLocation", CommitLocation));
        AddBinding(new TriggerBinding<string>(BindingGroup, "searchPlace", SearchPlace));
        AddBinding(new TriggerBinding<string>(BindingGroup, "previewTiles", PreviewTiles));
        AddBinding(new TriggerBinding(BindingGroup, "cancelLocationServices", CancelLocationServices));

        AddBinding(new TriggerBinding<bool>(BindingGroup, "setPanelOpen", value => { _panelOpen.Update(value); if (!value) CancelLocationServices(); }));
        AddBinding(new TriggerBinding<string>(BindingGroup, "setLatitude", value => SetInput(ref _latitude, value)));
        AddBinding(new TriggerBinding<string>(BindingGroup, "setLongitude", value => SetInput(ref _longitude, value)));
        AddBinding(new TriggerBinding<string>(BindingGroup, "setMapSize", value => { if (_busy.value) return; SetInput(ref _mapSizeId, value); _selectedMapSize.Update(_mapSizeId); }));
        AddBinding(new TriggerBinding<bool>(BindingGroup, "setIncludeSecondary", value => { if (_busy.value) return; _includeSecondary = value; _roadLevel = value ? RoadImportLevel.Secondary : RoadImportLevel.Primary; _roadLevelBinding.Update(NetworkSelection.Id(_roadLevel)); InvalidatePrepared(); new NetworkPreferences(_roadLevel,_importRail).Save(_log); }));
        AddBinding(new TriggerBinding<bool>(BindingGroup, "setGroundSurfaces", value => { if (_busy.value) return; _applyGroundSurfaces = value; InvalidatePrepared(); }));
        AddBinding(new TriggerBinding<bool>(BindingGroup, "setAirConnections", value => { if (_busy.value) return; _createAirConnections = value; InvalidatePrepared(); }));
        AddBinding(new TriggerBinding(BindingGroup, "prepare", StartPrepare));
        AddBinding(new TriggerBinding(BindingGroup, "cancel", CancelPrepare));
        AddBinding(new TriggerBinding(BindingGroup, "confirm", ConfirmApply));

        _log.Changed += OnLogChanged;
        ImportApplySystem.Progress += OnApplyProgress;
        ImportApplySystem.Completed += OnApplyCompleted;
        ImportApplySystem.Failed += OnApplyFailed;
    }

    protected override void OnDestroy()
    {
        ImportSession.ChangeMap(false);
        Observe(_downloadTask); Observe(_processTask); Observe(_searchTask); Observe(_tilesTask);
        _log.Changed -= OnLogChanged;
        _cancellation?.Cancel();
        _searchCancellation?.Cancel();
        _tilesCancellation?.Cancel();
        ImportApplySystem.Progress -= OnApplyProgress;
        ImportApplySystem.Completed -= OnApplyCompleted;
        ImportApplySystem.Failed -= OnApplyFailed;
        base.OnDestroy();
    }

    protected override void OnGamePreload(Colossal.Serialization.Entities.Purpose purpose, GameMode mode)
    { base.OnGamePreload(purpose, mode); ImportSession.ChangeMap(false); ResetMapState(); }

    protected override void OnGameLoadingComplete(Colossal.Serialization.Entities.Purpose purpose, GameMode mode)
    { base.OnGameLoadingComplete(purpose, mode); ImportSession.ChangeMap(mode == GameMode.Editor); ResetMapState(); }

    private static void Observe(Task? task)
    { if (task != null) _ = task.ContinueWith(t => { _ = t.Exception; }, CancellationToken.None, TaskContinuationOptions.OnlyOnFaulted, TaskScheduler.Default); }
    private void OnLogChanged() => _logDirty = true;

    private void ResetMapState()
    {
        _mapSession = ImportSession.Map; // Invalidate first, even if subsystem cleanup fails.
        try { _cancellation?.Cancel(); CancelLocationServices(); World.GetOrCreateSystemManaged<ImportApplySystem>().AbortMapWork(); }
        catch (Exception exception) { _log.Warn("Map-session cleanup failed; stale work remains fenced: " + exception.GetBaseException().Message); }
        Observe(_downloadTask); Observe(_processTask); Observe(_searchTask); Observe(_tilesTask);
        _downloadTask = null; _processTask = null; _searchTask = null; _tilesTask = null; _prepared = null;
        _downloadConsumed = _processConsumed = true; _lastMapWidth = 0;
        _busy.Update(false); _ready.Update(false); _panelOpen.Update(false); _stage.Update((int)ImportStage.Idle);
        _summary.Update(""); _error.Update(""); _status.Update("Map session reset. Prepare an import for the current map.");
        _placeResults.Update("{}"); _displayTiles.Update("{}"); _logText.Update("");
        _log.Changed -= OnLogChanged; _log = new ImportLog(); _log.Changed += OnLogChanged;
        _mapSessionBinding.Update(_mapSession.ToString(CultureInfo.InvariantCulture));
        _log.Info($"Editor map session {_mapSession}: old preparation/placement queues invalidated; downloaded caches retained.");
    }

    private void CancelLocationServices()
    {
        _searchCancellation?.Cancel(); _tilesCancellation?.Cancel();
    }

    protected override void OnUpdate()
    {
        base.OnUpdate();
        if (_mapSession != ImportSession.Map) ResetMapState();
        var editor = GameManager.instance.gameMode == GameMode.Editor;
        if (_isEditor.value != editor) _isEditor.Update(editor);
        var mapExt = MapExtDetector.HasVerifiedSizeApi;
        if (_mapExtDetected.value != mapExt) _mapExtDetected.Update(mapExt);
        if (editor && !_busy.value && MapExtDetector.TryGetCurrentMapWidth(out var width) && Math.Abs(width - _lastMapWidth) > 2d)
        {
            _lastMapWidth = width;
            _mapSizeId = width > 40_000d ? MapSizeProfile.MapExt57.Id : width > 20_000d ? MapSizeProfile.MapExt28.Id : MapSizeProfile.Vanilla.Id;
            _selectedMapSize.Update(_mapSizeId);
            InvalidatePrepared();
            _error.Update("");
            _status.Update($"Editor map: {width / 1000d:0.###} km. Enter coordinates, then prepare the import.");
        }

        if (_logDirty)
        {
            _logDirty = false;
            _logText.Update(_log.Snapshot());
        }
        if (_searchTask is { IsCompleted: true })
        {
            if (_searchTask.Status == TaskStatus.RanToCompletion && ImportSession.Ready && _searchMapSession == ImportSession.Map && _searchCancellation?.IsCancellationRequested == false) _placeResults.Update(_searchTask.Result);
            else { _ = _searchTask.Exception; }
            _searchTask = null;
        }
        if (_tilesTask is { IsCompleted: true })
        {
            if (_tilesTask.Status == TaskStatus.RanToCompletion && ImportSession.Ready && _tilesMapSession == ImportSession.Map && _tilesCancellation?.IsCancellationRequested == false) _displayTiles.Update(_tilesTask.Result);
            else { _ = _tilesTask.Exception; }
            _tilesTask = null;
        }

        if (_downloadTask is { IsCompleted: true } && !_downloadConsumed)
        {
            _downloadConsumed = true;
            if (!CompleteDownload()) return;
        }

        if (_processTask is { IsCompleted: true } && !_processConsumed)
        {
            _processConsumed = true;
            CompleteProcessing();
        }
    }

    private void StartPrepare()
    {
        if (_busy.value || !ImportSession.Ready || GameManager.instance.isGameLoading) return;
        _actionRevision.Update((++_actionCounter).ToString(CultureInfo.InvariantCulture));
        InvalidatePrepared();
        _error.Update("");
        _log.ResetMetrics();
        if (GameManager.instance.gameMode != GameMode.Editor) { Fail("Import Earth Location is available only in the Map Editor."); return; }
        if (!TryNumber(_latitude, out var latitude) || !TryNumber(_longitude, out var longitude))
        { Fail("Enter valid numeric latitude and longitude values."); return; }

        var request = new ImportRequest(latitude, longitude, MapSizeProfile.FromId(_mapSizeId), _includeSecondary)
        { RoadLevel = _roadLevel, ImportRail = _importRail, MapSessionId = ImportSession.Map, Generation = ImportSession.Begin(), ApplyGroundSurfaces = _applyGroundSurfaces, CreateAirConnections = _createAirConnections };
        var errors = InputValidator.Validate(request, MapExtDetector.HasVerifiedSizeApi).ToList();
        if (!MapExtDetector.TryGetCurrentMapWidth(out var actualWidth))
            errors.Add("The active MapExt map size could not be read; import is disabled to prevent a scale mismatch.");
        else if (Math.Abs(actualWidth - request.MapSize.WidthMeters) > 2d)
            errors.Add($"Selected map size is {request.MapSize.WidthMeters / 1000d:0.###} km, but the loaded editor map is {actualWidth / 1000d:0.###} km.");
        if (errors.Count > 0) { Fail(string.Join(" ", errors)); return; }

        var terrainBridge = new TerrainBridge(
            World.GetOrCreateSystemManaged<TerrainSystem>(),
            World.GetOrCreateSystemManaged<WaterSystem>());
        if (!terrainBridge.CanApply(out var readinessReason)) { Fail(readinessReason); return; }

        _cancellation?.Dispose();
        _cancellation = new CancellationTokenSource();
        _processTask = null;
        _busy.Update(true);
        _stage.Update((int)ImportStage.DownloadingTerrain);
        _status.Update("Downloading and validating public terrain and OSM tiles. Nothing in the map has changed.");
        _log.Info($"Preparing {request.MapSize.Label} centered at {latitude:0.######}, {longitude:0.######}.");
        _log.Info(NetworkSelection.Summary(request.RoadLevel, request.ImportRail));
        _downloadConsumed = false;
        _processConsumed = false;
        _downloadTask = DownloadAsync(request, _cancellation.Token);
    }

    private void CommitLocation(string payload)
    {
        if (_busy.value) return;
        try
        {
            var input = JObject.Parse(payload);
            var latitude = (double)input["latitude"]!;
            var longitude = (double)input["longitude"]!;
            var id = (string?)input["mapSize"];
            if (!LocationServices.Valid(latitude, longitude)) throw new ArgumentException("Enter valid coordinates: latitude −90 to 90; longitude −180 to 180.");
            if (id != "vanilla" && id != "mapext28" && id != "mapext57") throw new ArgumentException("Unsupported map size.");
            if (id != "vanilla" && !MapExtDetector.HasVerifiedSizeApi) throw new ArgumentException("MapExt size API unavailable; only vanilla is supported.");
            _latitude = latitude.ToString("R", CultureInfo.InvariantCulture);
            _longitude = longitude.ToString("R", CultureInfo.InvariantCulture);
            _mapSizeId = id!;
            InvalidatePrepared();
            _selectedMapSize.Update(_mapSizeId);
            _location.Update(new JObject { ["latitude"] = latitude, ["longitude"] = longitude, ["mapSize"] = id }.ToString(Newtonsoft.Json.Formatting.None));
            _error.Update("");
            _status.Update("Location selected. Prepare import when ready. No map data changed.");
        }
        catch (Exception exception) { _error.Update(exception.Message); }
    }

    private void SearchPlace(string payload)
    {
        if (!_panelOpen.value) return;
        _searchCancellation?.Cancel();
        if (_searchTask != null) _ = _searchTask.ContinueWith(t => { _ = t.Exception; }, TaskContinuationOptions.OnlyOnFaulted);
        _searchTask = null;
        try
        {
            var input = JObject.Parse(payload);
            var revision = (int?)input["revision"] ?? 0;
            if (DateTime.UtcNow < _nextSearch)
            { _placeResults.Update(new JObject { ["revision"] = revision, ["results"] = new JArray(), ["message"] = "Please wait a moment before searching again." }.ToString(Newtonsoft.Json.Formatting.None)); return; }
            _nextSearch = DateTime.UtcNow.AddSeconds(2);
            _searchCancellation = new CancellationTokenSource();
            _searchMapSession = ImportSession.Map;
            _searchTask = SearchWithRevision((string?)input["query"] ?? "", revision, _searchCancellation.Token);
        }
        catch { }
    }

    private static async Task<string> SearchWithRevision(string query, int revision, CancellationToken token)
    {
        var result = JObject.Parse(await LocationServices.SearchAsync(query, token).ConfigureAwait(false));
        result["revision"] = revision;
        return result.ToString(Newtonsoft.Json.Formatting.None);
    }

    private void PreviewTiles(string payload)
    {
        if (!_panelOpen.value) return;
        _tilesCancellation?.Cancel();
        if (_tilesTask != null) _ = _tilesTask.ContinueWith(t => { _ = t.Exception; }, TaskContinuationOptions.OnlyOnFaulted);
        _tilesCancellation = new CancellationTokenSource();
        _tilesMapSession = ImportSession.Map;
        _tilesTask = LocationServices.TilesAsync(payload, _tilesCancellation.Token);
    }

    private async Task<DownloadBundle> DownloadAsync(ImportRequest request, CancellationToken token)
    {
        var log = _log; // Capture the current session log; old tasks must not write into a new map's panel.

        var bounds = GeoMath.BoundsFromCenter(request.Latitude, request.Longitude, request.MapSize.WidthMeters);
        var cache = new CacheStore();
        using var downloads = CancellationTokenSource.CreateLinkedTokenSource(token);
        var climateTask = ClimateClient.LoadAsync(request, log, downloads.Token);
        var terrainTask = new TerrainTileClient(cache, log).DownloadAsync(bounds, request.MapSize.WidthMeters, downloads.Token);
        var osmTask = new OverpassClient(cache, log, roadLevel: request.RoadLevel, importRail: request.ImportRail).Download16Async(bounds, request.IncludeSecondaryRoads, downloads.Token);
        try
        {
            // A failed tile aborts preparation instead of waiting for the other download.
            var first = await Task.WhenAny(terrainTask, osmTask).ConfigureAwait(false);
            await first.ConfigureAwait(false);
            await Task.WhenAll(terrainTask, osmTask).ConfigureAwait(false);
            IReadOnlyList<byte[]>? geography = null;
            using var geographyCancellation = CancellationTokenSource.CreateLinkedTokenSource(downloads.Token);
            geographyCancellation.CancelAfter(TimeSpan.FromMinutes(3));
            try
            {
                log.Info("Preparing water geography" + (request.ApplyGroundSurfaces ? " and ground-surface data." : "."));
                geography = await new OverpassClient(cache, log, geography: true).Download16Async(bounds, false, geographyCancellation.Token).ConfigureAwait(false);
            }
            catch (Exception exception) when (!token.IsCancellationRequested)
            { log.Warn("Water/surface geography unavailable; affected stages skipped: " + (geographyCancellation.IsCancellationRequested ? "optional geography exceeded its three-minute deadline; core downloaded data/cache retained." : exception.Message)); }
            var climate = await climateTask.ConfigureAwait(false);
            return new DownloadBundle(request, bounds, await terrainTask.ConfigureAwait(false), await osmTask.ConfigureAwait(false), geography, climate);
        }
        catch
        {
            downloads.Cancel();
            try { await Task.WhenAll(terrainTask, osmTask, climateTask).ConfigureAwait(false); } catch { /* Observe cancelled/faulted sibling. */ }
            throw;
        }
    }

    private bool CompleteDownload()
    {
        if (_downloadTask!.IsCanceled) { ReturnToIdle("Preparation cancelled. Nothing changed."); return false; }
        if (_downloadTask.IsFaulted) { Fail(_downloadTask.Exception!.GetBaseException().Message); return false; }
        try
        {
            _stage.Update((int)ImportStage.Processing);
            _status.Update("Decoding terrain and merging/deduplicating OSM data. Nothing in the map has changed.");
            var bundle = _downloadTask.Result;
            if (!ImportSession.Matches(bundle.Request.MapSessionId, bundle.Request.Generation)) { _log.Warn("Discarded stale previous-map/download result."); ReturnToIdle("Stale preparation discarded."); return false; }
            var pixels = TerrainBuilder.DecodeOnMainThread(bundle.TerrainTiles, _cancellation!.Token);
            _processTask = ProcessAsync(bundle, pixels, _cancellation.Token);
            return true;
        }
        catch (Exception exception) { Fail(exception.GetBaseException().Message); return false; }
    }

    private async Task<ProcessedBundle> ProcessAsync(DownloadBundle bundle, IReadOnlyList<TerrainTilePixels> pixels, CancellationToken token)
    {
        var log = _log; // Capture the current session log; old tasks must not write into a new map's panel.

        var osmTask = Task.Run(() => { var clock = Stopwatch.StartNew(); var result = OsmParser.Merge(bundle.OsmDocuments, log, token); log.Time("OSM parse", clock.Elapsed.TotalSeconds); return result; }, token);
        OsmDataset? geography = null;
        IReadOnlyList<GeographicPolygon> polygons = Array.Empty<GeographicPolygon>();
        if (bundle.GeographyDocuments != null)
        {
            try { geography = await Task.Run(() => OsmParser.Merge(bundle.GeographyDocuments, log, token), token).ConfigureAwait(false); polygons = GeographyPolygons.Build(geography, bundle.Request, log, token); }
            catch (Exception exception) when (exception is not OperationCanceledException) { log.Warn("Water/surface classification skipped: " + exception.Message); }
        }
        var terrainTask = TerrainBuilder.BuildAsync(pixels, bundle.Bounds, token, log, geography);
        await Task.WhenAll(terrainTask, osmTask).ConfigureAwait(false);
        var dataset = await osmTask.ConfigureAwait(false);
        var terrain = await terrainTask.ConfigureAwait(false);
        var water = WaterPlanner.Prepare(geography, polygons, bundle.Request, terrain, log, token);
        SurfaceMask? surfaces = null;
        if (bundle.Request.ApplyGroundSurfaces && geography != null)
        {
            try { log.Info("Preparing surfaces."); surfaces = SurfacePlanner.Prepare(polygons, bundle.Request, terrain, log, token); }
            catch (Exception exception) when (exception is not OperationCanceledException) { log.Warn("Surface preparation skipped: " + exception.Message); }
        }
        IReadOnlyList<AirDestination> destinations = Array.Empty<AirDestination>();
        if (bundle.Request.CreateAirConnections)
        {
            try { destinations = AirPlanner.Select(bundle.Request, log); }
            catch (Exception exception) when (exception is not OperationCanceledException) { log.Warn("Air destination preparation skipped: " + exception.Message); }
        }
        token.ThrowIfCancellationRequested();
        log.Info("Building OSM topology and cleaning network geometry.");
        var clock = Stopwatch.StartNew();
        var rawSegments = NetworkProcessor.BuildSegments(dataset, bundle.Request, log, token);
        log.Time("Network cleanup", clock.Elapsed.TotalSeconds);
        log.Info("Simplifying protected network chains and fitting bounded-error curves.");
        var normalized = rawSegments.Count == 0 ? Array.Empty<PreparedSegment>() : NetworkNormalizer.Normalize(rawSegments, log, token, bundle.Request.MapSize.WidthMeters);
        var hardened = NetworkHardener.Prepare(normalized, log, token, bundle.Request.ImportRail);
        var initialAlignment = NetworkProcessor.AlignToTerrain(hardened, terrain.Heightmap, terrain.HeightScaleMeters, bundle.Request.MapSize.WidthMeters, token);
        var aligned = NetworkVerticalProfiles.Prepare(initialAlignment, terrain.Heightmap, terrain.HeightScaleMeters, bundle.Request.MapSize.WidthMeters, log, token);
        clock.Restart();
        log.Info("Validating normalized road and rail coverage/geometry.");
        NetworkNormalizer.Validate(aligned, bundle.Request.MapSize.WidthMeters, token);
        log.Time("Geometry validation", clock.Elapsed.TotalSeconds);
        IReadOnlyList<Unity.Mathematics.float3> trees = Array.Empty<Unity.Mathematics.float3>();
        try { log.Info("Preparing optional forest masks and tree candidates."); trees = ForestProcessor.Prepare(dataset, bundle.Request, terrain, aligned, log, token); }
        catch (Exception exception) when (exception is not OperationCanceledException) { log.Warn($"Vegetation preparation skipped: {exception.Message}"); }
        token.ThrowIfCancellationRequested();
        return new ProcessedBundle(bundle, terrain, aligned, trees, dataset.Ways.Count, surfaces, destinations, water);
    }

    private void CompleteProcessing()
    {
        if (_processTask!.IsCanceled) { ReturnToIdle("Preparation cancelled. Nothing changed."); return; }
        if (_processTask.IsFaulted) { Fail(_processTask.Exception!.GetBaseException().Message); return; }
        try
        {
            var result = _processTask.Result;
            if (!ImportSession.Matches(result.Download.Request.MapSessionId, result.Download.Request.Generation)) { _log.Warn("Discarded stale previous-map/processing result."); ReturnToIdle("Stale preparation discarded."); return; }
            var errors = new List<string>();
            var warnings = new List<string>();
            var trees = result.Trees;
            var validationClock = Stopwatch.StartNew();
            var terrainBridge = new TerrainBridge(
                World.GetOrCreateSystemManaged<TerrainSystem>(),
                World.GetOrCreateSystemManaged<WaterSystem>());
            if (!terrainBridge.CanApply(out var terrainReason)) errors.Add(terrainReason);
            if (result.Segments.Count > 0 && !NetworkCommitSystem.CanCommit(out var commitReason)) errors.Add(commitReason);

            var resolver = new PrefabResolver(World.GetOrCreateSystemManaged<PrefabSystem>(), _log);
            if (result.Segments.Any(segment => segment.Kind == NetworkKind.Road) && !resolver.CanResolveRoad(out var roadReason)) errors.Add(roadReason);
            if (result.Segments.Any(segment => segment.Kind == NetworkKind.Rail) && !resolver.CanResolveRail(out var railReason)) errors.Add(railReason);
            if (result.Download.Request.ImportRail && !result.Segments.Any(segment => segment.Kind == NetworkKind.Rail)) warnings.Add("No selected heavy-rail segments remain inside these bounds; see the processing log.");
            if (result.Download.Request.RoadLevel != RoadImportLevel.None && !result.Segments.Any(segment => segment.Kind == NetworkKind.Road)) warnings.Add("No selected road sections remain inside these bounds; see the processing log.");
            var geometrySkips = _log.Metric("Unsafe carriageway sections omitted") + _log.Metric("Roads rejected for impossible grade/profile");
            if (geometrySkips > 0) warnings.Add($"Partial road import: {geometrySkips:0} sections omitted after horizontal/vertical safety checks; review the log before replacement.");
            if (trees.Count > 0)
            {
                try
                {
                    if (NativeVegetationPlacement.ResolvePrefabs(EntityManager, World.GetOrCreateSystemManaged<PrefabSystem>()).Length == 0)
                    { warnings.Add("Optional forest trees will be skipped: no tree prefab fallback exists."); trees = Array.Empty<Unity.Mathematics.float3>(); }
                }
                catch (Exception exception) { warnings.Add($"Optional forest trees will be skipped: {exception.Message}"); trees = Array.Empty<Unity.Mathematics.float3>(); }
            }
            if (errors.Count > 0) { Fail(string.Join(" ", errors)); return; }
            var supported = NativeNetworkPlacement.Preflight(result.Segments, World.GetOrCreateSystemManaged<PrefabSystem>(), _log);
            if(supported.Count < result.Segments.Count) warnings.Add($"Partial network import: {result.Segments.Count-supported.Count} sections omitted for native direction/width compatibility.");
            result = result with { Segments = supported };

            _prepared = new PreparedImport
            {
                Request = result.Download.Request,
                Bounds = result.Download.Bounds,
                Heightmap = result.Terrain.Heightmap,
                HeightScaleMeters = result.Terrain.HeightScaleMeters,
                SeaLevelMeters = result.Terrain.SeaLevelMeters,
                Segments = result.Segments,
                Trees = trees,
                SurfaceMask = result.Surfaces,
                AirDestinations = result.Destinations,
                Climate = result.Download.Climate,
                Water = result.Water,
                Warnings = warnings,
                SourceWayCount = result.SourceWayCount
            };
            var roads = result.Segments.Count(segment => segment.Kind == NetworkKind.Road);
            var rail = result.Segments.Count(segment => segment.Kind == NetworkKind.Rail);
            _log.Time("Runtime validation", validationClock.Elapsed.TotalSeconds);
            _summary.Update(NetworkSelection.Summary(result.Download.Request.RoadLevel, result.Download.Request.ImportRail) + $"\nTerrain {result.Terrain.MinimumElevation:0}–{result.Terrain.MaximumElevation:0} m • {roads:N0} road sections" + (result.Download.Request.ImportRail ? $" • {rail:N0} rail sections" : "") + $" • {trees.Count:N0} optional trees • {warnings.Count} warnings\n" + result.Terrain.ConditioningSummary + "\n" + result.Water.Report);
            foreach (var warning in warnings) _log.Warn(warning);
            _ready.Update(true);
            _busy.Update(false);
            _stage.Update((int)ImportStage.ReadyForConfirmation);
            _status.Update("Core preflight passed. Review the summary for skipped or unverified optional stages. Save a backup, then confirm replacement.");
            _log.Info("Preflight complete. No editor entities have been modified.");
            _log.Info("Preparation statistics: " + _log.Metrics());
        }
        catch (Exception exception) { Fail(exception.GetBaseException().Message); }
    }

    private void ConfirmApply()
    {
        if (!_busy.value) _actionRevision.Update((++_actionCounter).ToString(CultureInfo.InvariantCulture));
        if (_prepared == null || !_ready.value || _busy.value) return;
        if (!ImportSession.Matches(_prepared.Request.MapSessionId, _prepared.Request.Generation) || _prepared.Request.RoadLevel != _roadLevel || _prepared.Request.ImportRail != _importRail)
        { InvalidatePrepared(); _status.Update("Prepared network settings/map are stale. Prepare again."); return; }
        if (!ImportApplySystem.TryRequest(_prepared, _log)) { Fail("Another import is already applying."); return; }
        _busy.Update(true);
        _ready.Update(false);
        _stage.Update((int)ImportStage.Applying);
        _status.Update("Applying import. Cancellation is disabled during native map mutation.");
    }

    private void CancelPrepare()
    {
        if (_stage.value == (int)ImportStage.Applying) return;
        _cancellation?.Cancel();
        ImportSession.InvalidateOperation(); Observe(_downloadTask); Observe(_processTask);
        _cancellation = null;
        _downloadTask = null;
        _processTask = null;
        _downloadConsumed = true;
        _processConsumed = true;
        ReturnToIdle("Preparation cancelled. Nothing changed.");
    }

    private void OnApplyProgress(string message) => _status.Update(message);

    private void OnApplyCompleted(int created, int skipped)
    {
        _busy.Update(false);
        _stage.Update((int)ImportStage.Complete);
        _status.Update($"Import complete: {created:N0} native network segments queued, {skipped:N0} skipped. Save the map normally in CS2.");
        _summary.Update(_summary.value + $" • applied {created:N0} • skipped {skipped:N0}");
        if (_prepared != null) _summary.Update(_summary.value + "\n" + _prepared.FeatureReport);
        _log.Info("Completion statistics: " + _log.Metrics());
        _summary.Update(_summary.value + " • detailed totals/timings are in the log");
        _prepared = null;
    }

    private void OnApplyFailed(string message) => Fail(message);

    private void SetInput(ref string field, string value)
    {
        if (_busy.value) return;
        field = value?.Trim() ?? "";
        InvalidatePrepared();
    }

    private void InvalidatePrepared()
    {
        if (_stage.value == (int)ImportStage.Applying) return;
        ImportSession.InvalidateOperation();
        _prepared = null;
        _ready.Update(false);
        _summary.Update("");
        if (!_busy.value) _stage.Update((int)ImportStage.Idle);
    }

    private void Fail(string message)
    {
        var applying = _stage.value == (int)ImportStage.Applying;
        _cancellation?.Cancel();
        _log.Error(message);
        _prepared = null;
        _busy.Update(false);
        _ready.Update(false);
        _summary.Update("");
        _stage.Update((int)(applying ? ImportStage.Failed : ImportStage.Idle));
        _status.Update(applying
            ? "Application stopped; the map may have changed. Review the log before saving."
            : "Preparation stopped. Nothing in the map changed. Review the log, then try again.");
        _error.Update(message);
    }

    private void ReturnToIdle(string message)
    {
        _busy.Update(false);
        _ready.Update(false);
        _stage.Update((int)ImportStage.Idle);
        _status.Update(message);
    }

    private static bool TryNumber(string value, out double number) =>
        double.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out number) ||
        double.TryParse(value, NumberStyles.Float, CultureInfo.CurrentCulture, out number);

    private sealed record DownloadBundle(
        ImportRequest Request,
        GeoBounds Bounds,
        IReadOnlyList<TerrainTile> TerrainTiles,
        IReadOnlyList<byte[]> OsmDocuments,
        IReadOnlyList<byte[]>? GeographyDocuments,
        ClimateNormals? Climate);

    private sealed record ProcessedBundle(
        DownloadBundle Download,
        TerrainBuilder.Result Terrain,
        IReadOnlyList<PreparedSegment> Segments,
        IReadOnlyList<Unity.Mathematics.float3> Trees,
        int SourceWayCount,
        SurfaceMask? Surfaces,
        IReadOnlyList<AirDestination> Destinations,
        WaterPlan Water);
}
}
