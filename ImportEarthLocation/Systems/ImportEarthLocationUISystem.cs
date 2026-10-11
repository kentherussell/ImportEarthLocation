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
    private ValueBinding<bool> _forestBinding = null!;
    private ValueBinding<string> _mapSessionBinding = null!;
    private ValueBinding<string> _actionRevision = null!;
    private ValueBinding<string> _serviceUrl = null!;
    private ValueBinding<string> _serviceStatus = null!;
    private ValueBinding<int> _serviceRevision = null!;
    private long _actionCounter;
    private RoadImportLevel _roadLevel = RoadImportLevel.Primary;
    private bool _importRail = true;
    private bool _importForests = true;
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
    private PreparationProgress _progress = new();
    private DateTime _nextProgressUpdate;
    private DateTime _nextArchiveFlush;

    public override GameMode gameMode => GameMode.Editor;

    [Preserve]
    protected override void OnCreate()
    {
        base.OnCreate();
        var preferences = NetworkPreferences.Load(_log); _roadLevel = preferences.Roads; _importRail = preferences.Rail; _importForests = preferences.Forests;
        AddBinding(_serviceUrl = new ValueBinding<string>(BindingGroup, "serviceUrl", ServicePreferences.Load(_log)));
        AddBinding(_serviceStatus = new ValueBinding<string>(BindingGroup, "serviceStatus", ""));
        AddBinding(_serviceRevision = new ValueBinding<int>(BindingGroup, "serviceRevision", 0));
        AddBinding(new TriggerBinding<string>(BindingGroup, "saveServiceUrl", value =>
        {
            if (_busy.value) return;
            try
            {
                var normalized = ServicePreferences.Normalize(value);
                if (!ServicePreferences.Save(normalized, _log)) { _serviceStatus.Update("Could not save. Check access to the mod settings folder and try again."); return; }
                _serviceUrl.Update(normalized); _serviceStatus.Update("Saved.");
                _serviceRevision.Update(_serviceRevision.value + 1);
            }
            catch (ArgumentException e) { _serviceStatus.Update(e.Message); }
        }));
        AddBinding(_isEditor = new ValueBinding<bool>(BindingGroup, "isEditor", GameManager.instance.gameMode==GameMode.Editor&&!GameManager.instance.isGameLoading));
        AddBinding(new TriggerBinding<string>(BindingGroup,"frontendState",NativeUiBootstrap.Frontend));
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
        AddBinding(_forestBinding = new ValueBinding<bool>(BindingGroup, "importForests", _importForests));
        _mapSession = ImportSession.Map;
        AddBinding(_mapSessionBinding = new ValueBinding<string>(BindingGroup, "mapSession", _mapSession.ToString(CultureInfo.InvariantCulture)));
        AddBinding(_actionRevision = new ValueBinding<string>(BindingGroup, "actionRevision", "0"));
        AddBinding(new TriggerBinding<string>(BindingGroup, "setRoadLevel", value => { if (_busy.value) return; _roadLevel = NetworkSelection.Parse(value); _roadLevelBinding.Update(NetworkSelection.Id(_roadLevel)); InvalidatePrepared(); new NetworkPreferences(_roadLevel,_importRail,_importForests).Save(_log); }));
        AddBinding(new TriggerBinding<bool>(BindingGroup, "setImportRail", value => { if (_busy.value) return; _importRail = value; _railBinding.Update(value); InvalidatePrepared(); new NetworkPreferences(_roadLevel,_importRail,_importForests).Save(_log); }));
        AddBinding(new TriggerBinding<bool>(BindingGroup, "setImportForests", value => { if (_busy.value) return; _importForests = value; _forestBinding.Update(value); InvalidatePrepared(); new NetworkPreferences(_roadLevel,_importRail,_importForests).Save(_log); }));
        AddBinding(new TriggerBinding<string>(BindingGroup, "commitLocation", CommitLocation));
        AddBinding(new TriggerBinding<string>(BindingGroup, "searchPlace", SearchPlace));
        AddBinding(new TriggerBinding<string>(BindingGroup, "previewTiles", PreviewTiles));
        AddBinding(new TriggerBinding(BindingGroup, "cancelLocationServices", CancelLocationServices));

        AddBinding(new TriggerBinding<bool>(BindingGroup, "setPanelOpen", value => { _panelOpen.Update(value); if (!value) CancelLocationServices(); }));
        AddBinding(new TriggerBinding<string>(BindingGroup, "setLatitude", value => SetInput(ref _latitude, value)));
        AddBinding(new TriggerBinding<string>(BindingGroup, "setLongitude", value => SetInput(ref _longitude, value)));
        AddBinding(new TriggerBinding<string>(BindingGroup, "setMapSize", value => { if (_busy.value) return; SetInput(ref _mapSizeId, value); _selectedMapSize.Update(_mapSizeId); }));
        AddBinding(new TriggerBinding<bool>(BindingGroup, "setIncludeSecondary", value => { if (_busy.value) return; _includeSecondary = value; _roadLevel = value ? RoadImportLevel.Secondary : RoadImportLevel.Primary; _roadLevelBinding.Update(NetworkSelection.Id(_roadLevel)); InvalidatePrepared(); new NetworkPreferences(_roadLevel,_importRail,_importForests).Save(_log); }));
        AddBinding(new TriggerBinding<bool>(BindingGroup, "setGroundSurfaces", value => { if (_busy.value) return; _applyGroundSurfaces = value; InvalidatePrepared(); }));
        AddBinding(new TriggerBinding<bool>(BindingGroup, "setAirConnections", value => { if (_busy.value) return; _createAirConnections = value; InvalidatePrepared(); }));
        AddBinding(new TriggerBinding(BindingGroup, "prepare", StartPrepare));
        AddBinding(new TriggerBinding(BindingGroup, "cancel", CancelPrepare));
        AddBinding(new TriggerBinding(BindingGroup, "confirm", ConfirmApply));

        _log.Changed += OnLogChanged;
        ImportApplySystem.Progress += OnApplyProgress;
        ImportApplySystem.Completed += OnApplyCompleted;
        ImportApplySystem.Failed += OnApplyFailed;
        NativeUiBootstrap.Ensure();
        Mod.Log.Info($"UI bindings created; editor={_isEditor.value}; session={_mapSession}; otherModsRequired=false");
    }

    protected override void OnDestroy()
    {
        _log.CloseArchive(_busy.value ? "interrupted-on-dispose" : null);
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
    { base.OnGamePreload(purpose, mode); _isEditor.Update(false); ImportSession.ChangeMap(false); ResetMapState(); }

    protected override void OnGameLoadingComplete(Colossal.Serialization.Entities.Purpose purpose, GameMode mode)
    { base.OnGameLoadingComplete(purpose, mode); ImportSession.ChangeMap(mode == GameMode.Editor); ResetMapState(); _isEditor.Update(mode==GameMode.Editor); NativeUiBootstrap.Ensure(); Mod.Log.Info($"UI map loaded; mode={mode}; editor={_isEditor.value}; session={_mapSession}"); if(mode==GameMode.Editor)NativeDiagnostics.Queue("after-load",_log); }

    private static void Observe(Task? task)
    { if (task != null) _ = task.ContinueWith(t => { _ = t.Exception; }, CancellationToken.None, TaskContinuationOptions.OnlyOnFaulted, TaskScheduler.Default); }
    private void OnLogChanged() => _logDirty = true;

    private void ResetMapState()
    {
        _log.CloseArchive(_busy.value ? "interrupted-by-map-change" : null);
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
        NativeUiBootstrap.Ensure();
        if (_mapSession != ImportSession.Map) ResetMapState();
        var editor = GameManager.instance.gameMode == GameMode.Editor&&!GameManager.instance.isGameLoading;
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
        if (DateTime.UtcNow >= _nextArchiveFlush)
        {
            _log.FlushArchive();
            _nextArchiveFlush = DateTime.UtcNow.AddSeconds(5);
        }
        if (_busy.value && DateTime.UtcNow >= _nextProgressUpdate)
        {
            _status.Update(_progress.Snapshot(_stage.value != (int)ImportStage.Applying));
            _nextProgressUpdate = DateTime.UtcNow.AddSeconds(1);
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
        _log.CloseArchive();
        _log.ResetMetrics();
        if (GameManager.instance.gameMode != GameMode.Editor) { Fail("Import Earth Location is available only in the Map Editor."); return; }
        if (!TryNumber(_latitude, out var latitude) || !TryNumber(_longitude, out var longitude))
        { Fail("Enter valid numeric latitude and longitude values."); return; }

        var request = new ImportRequest(latitude, longitude, MapSizeProfile.FromId(_mapSizeId), _includeSecondary)
        { RoadLevel = _roadLevel, ImportRail = _importRail, ImportForests = _importForests, MapSessionId = ImportSession.Map, Generation = ImportSession.Begin(), ApplyGroundSurfaces = _applyGroundSurfaces, CreateAirConnections = _createAirConnections };
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

        _log.BeginArchive(request);

        _cancellation?.Dispose();
        _cancellation = new CancellationTokenSource();
        _processTask = null;
        _busy.Update(true);
        _stage.Update((int)ImportStage.DownloadingTerrain);
        _progress = new PreparationProgress();
        _progress.Begin("Downloading terrain and road information");
        _nextProgressUpdate = DateTime.MinValue;
        _status.Update(_progress.Snapshot());
        _log.Info($"Preparing {request.MapSize.Label} centered at {latitude:0.######}, {longitude:0.######}.");
        _log.Info(NetworkSelection.Summary(request.RoadLevel, request.ImportRail));
        _log.Diagnostic($"PREPARE mapSession={request.MapSessionId}; generation={request.Generation}; {NetworkSelection.Summary(request.RoadLevel, request.ImportRail)}; forests={request.ImportForests}; mapWidth={request.MapSize.WidthMeters:R}; surfaces={request.ApplyGroundSurfaces}; air={request.CreateAirConnections}");
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
        var progress = _progress;

        var bounds = GeoMath.BoundsFromCenter(request.Latitude, request.Longitude, request.MapSize.WidthMeters);
        var cache = new CacheStore(log);
        using var downloads = CancellationTokenSource.CreateLinkedTokenSource(token);
        var climateTask = ClimateClient.LoadAsync(request, log, downloads.Token);
        var terrainTask = new TerrainTileClient(cache, log).DownloadAsync(bounds, request.MapSize.WidthMeters, downloads.Token);
        downloads.CancelAfter(TimeSpan.FromMinutes(30));
        var osmTask = DownloadNetworksAsync();
        try
        {
            // Both terrain and selected networks are required. Optional
            // geography/vegetation may degrade, never required infrastructure.
            var first = await Task.WhenAny(terrainTask, osmTask).ConfigureAwait(false);
            await first.ConfigureAwait(false);
            await Task.WhenAll(terrainTask, osmTask).ConfigureAwait(false);
            downloads.CancelAfter(Timeout.Infinite); // Optional datasets have their own safety/active-request budgets.
            progress.Begin("Downloading water, landscape and selected forest information");
            // Independent optional datasets overlap, but share the same global HTTP budget.
            // Neither dataset can invalidate successful terrain/network coverage.
            var geographyTask=DownloadOptionalAsync("geography",new OverpassClient(cache,log,geography:true,includeGroundSurfaces:request.ApplyGroundSurfaces));
            var forestTask=request.ImportForests ? DownloadOptionalAsync("forests",new OverpassClient(cache,log,forestOnly:true)) : Task.FromResult<IReadOnlyList<byte[]>?>(null);
            if (!request.ImportForests) log.Diagnostic("OSM OPTIONAL SKIP category=forests; reason=user-disabled; httpRequests=0");
            await Task.WhenAll(geographyTask,forestTask).ConfigureAwait(false);
            var geography=await geographyTask.ConfigureAwait(false);
            var forests=await forestTask.ConfigureAwait(false);
            var climate = await climateTask.ConfigureAwait(false);
            return new DownloadBundle(request, bounds, await terrainTask.ConfigureAwait(false), await osmTask.ConfigureAwait(false), forests, geography, climate);
        }
        catch (Exception exception)
        {
            downloads.Cancel();
            try { await Task.WhenAll(terrainTask, osmTask, climateTask).ConfigureAwait(false); } catch { /* Observe cancelled/faulted sibling. */ }
            if (exception is OperationCanceledException && !token.IsCancellationRequested) throw new TimeoutException("Core downloads exceeded their 30-minute safety deadline. Completed downloads remain cached.", exception);
            throw;
        }

        async Task<IReadOnlyList<byte[]>> DownloadNetworksAsync()
        {
            try{return await new OverpassClient(cache,log,roadLevel:request.RoadLevel,importRail:request.ImportRail).DownloadMapAsync(bounds,request.IncludeSecondaryRoads,downloads.Token,allowPartial:false).ConfigureAwait(false);}
            catch(Exception error) when(!token.IsCancellationRequested && (OsmRecovery.Recoverable(error)||error is OperationCanceledException))
            {
                log.Count("OSM network downloads unavailable");
                throw new PreparationDataException(PreparationDataIssue.NetworkIncomplete,"Required road/rail data could not be downloaded completely after the existing-provider recovery budget.",error);
            }
        }

        async Task<IReadOnlyList<byte[]>?> DownloadOptionalAsync(string category,OverpassClient client)
        {
            using var optionalCancellation=CancellationTokenSource.CreateLinkedTokenSource(downloads.Token);
            optionalCancellation.CancelAfter(TimeSpan.FromMinutes(15));
            var optionalClock=Stopwatch.StartNew();
            try
            {
                log.Diagnostic($"OSM OPTIONAL BEGIN category={category}; overallSafetySeconds=900; activeHttpBudgetSeconds=600; queueTimeExcludedFromActiveBudget=true; sharedConcurrencyLimit=2; perProviderLimit=1");
                return await client.DownloadMapAsync(bounds,false,optionalCancellation.Token, activeBudgetSeconds:600,allowPartial:true).ConfigureAwait(false);
            }
            catch(Exception exception) when(!token.IsCancellationRequested && (OsmRecovery.Recoverable(exception)||exception is OperationCanceledException))
            {
                var reason=optionalCancellation.IsCancellationRequested?"optional dataset exceeded its 15-minute overall safety deadline":exception.Message;
                log.Warn($"Optional OSM {category} unavailable; dependent stage skipped: {reason}. Core downloaded data/cache retained.");
                log.Diagnostic($"OSM OPTIONAL SKIP category={category}; elapsedSeconds={optionalClock.Elapsed.TotalSeconds:0.###}; reason={reason}; coreImportPreserved=true");
                return null;
            }
        }
    }

    private bool CompleteDownload()
    {
        if (_downloadTask!.IsCanceled) { ReturnToIdle("Preparation cancelled. Nothing changed."); return false; }
        if (_downloadTask.IsFaulted) { Fail(_downloadTask.Exception!, "downloads"); return false; }
        try
        {
            _stage.Update((int)ImportStage.Processing);
            _progress.Begin("Reading downloaded terrain");
            _status.Update(_progress.Snapshot());
            var bundle = _downloadTask.Result;
            if (!ImportSession.Matches(bundle.Request.MapSessionId, bundle.Request.Generation)) { _log.Warn("Discarded stale previous-map/download result."); ReturnToIdle("Stale preparation discarded."); return false; }
            _log.Diagnostic($"PROCESS terrain-decode begin; tiles={bundle.TerrainTiles.Count}");
            var pixels = TerrainBuilder.DecodeOnMainThread(bundle.TerrainTiles, _cancellation!.Token);
            var invalidTiles=TerrainBuilder.InvalidTiles(pixels,bundle.Bounds,_log,_cancellation.Token);
            if(invalidTiles.Count>0)
            {
                _progress.Begin("Recovering invalid elevation tiles");
                _downloadTask=RecoverTerrainAsync(bundle,invalidTiles,_cancellation.Token);
                _downloadConsumed=false;
                return false;
            }
            _log.Diagnostic("PROCESS terrain-decode complete; starting cached OSM parsing and DEM preparation");
            // Snapshot native capabilities on the game thread. Processing receives
            // plain numbers only, never an EntityManager or native prefab object.
            var metadataResolver=new PrefabResolver(World.GetOrCreateSystemManaged<PrefabSystem>(),_log);
            var widths=metadataResolver.CaptureMinimumWidths();
            var laneOffsets=metadataResolver.LaneOffsets;
            var waterCellSize=World.GetOrCreateSystemManaged<WaterSystem>().CellSize;
            _processTask = ProcessAsync(bundle, pixels, widths, laneOffsets, waterCellSize, _cancellation.Token);
            return true;
        }
        catch (Exception exception) { Fail(exception, "terrain-decode/start-processing"); return false; }
    }

    private async Task<ProcessedBundle> ProcessAsync(DownloadBundle bundle, IReadOnlyList<TerrainTilePixels> pixels, IReadOnlyDictionary<string,float> widths, IReadOnlyDictionary<string,float[]> laneOffsets, float waterCellSize, CancellationToken token)
    {
        var log = _log; // Capture the current session log; old tasks must not write into a new map's panel.
        var progress = _progress;
        progress.Begin("Reading downloaded map information");

        var validatedCache=new CacheStore(log);
        var parsedCacheRoot = validatedCache.Root;
        for(var i=0;i<bundle.TerrainTiles.Count;i++)
        {
            var tile=bundle.TerrainTiles[i];
            // Small-hole interpolation lives in decoded samples, not in these
            // original PNG bytes. Never recache a semantically invalid PNG.
            if(tile.RecoveryPass==1&&pixels[i].RecoveredElevations==null)
                await validatedCache.WriteAsync("terrain",$"terrarium-v1:{tile.Zoom}:{tile.X}:{tile.Y}","png",tile.PngBytes,token).ConfigureAwait(false);
        }
        log.Diagnostic("PROCESS OSM parsing begin; networks and geography; cached binary data preferred");
        var osmTask = Task.Run(() => { var clock = Stopwatch.StartNew(); var result = bundle.Request.RoadLevel == RoadImportLevel.None && !bundle.Request.ImportRail
            ? new OsmDataset() : OsmRecovery.Parse(bundle.OsmDocuments, parsedCacheRoot, "networks", log, token); log.Time("OSM parse", clock.Elapsed.TotalSeconds); return result; }, token);
        OsmDataset? geography = null;
        IReadOnlyList<GeographicPolygon> polygons = Array.Empty<GeographicPolygon>();
        if (bundle.GeographyDocuments != null)
        {
            try { geography = await Task.Run(() => OsmRecovery.Parse(bundle.GeographyDocuments, parsedCacheRoot, "geography", log, token), token).ConfigureAwait(false); polygons = GeographyPolygons.Build(geography, bundle.Request, log, token); }
            catch (Exception exception) when (exception is not OperationCanceledException) { log.Warn("Water/surface classification skipped: " + exception.Message); }
        }
        var terrainTask = TerrainBuilder.BuildAsync(pixels, bundle.Bounds, token, log, geography);
        progress.Begin("Shaping and smoothing the terrain");
        log.Diagnostic("PROCESS terrain-conditioning/resampling begin; awaiting terrain and network parsing");
        await Task.WhenAll(terrainTask, osmTask).ConfigureAwait(false);
        OsmDataset networkSource;
        try
        {
            networkSource=await new OverpassClient(validatedCache,log,roadLevel:bundle.Request.RoadLevel,importRail:bundle.Request.ImportRail)
                .RecoverDependenciesAsync(await osmTask.ConfigureAwait(false),bundle.Bounds,token).ConfigureAwait(false);
        }
        catch(Exception error) when(!token.IsCancellationRequested&&OsmRecovery.Recoverable(error))
        {
            log.Diagnostic($"OSM DEPENDENCY unavailable exception={error.GetType().Name}; reason={error.Message}; requiredInfrastructureRetained=false; applicationNotStarted=true; cacheRetained=true");
            throw new PreparationDataException(PreparationDataIssue.NetworkIncomplete,"Required network references could not be recovered completely from the configured OSM providers. No disconnected subset will be applied.",error);
        }
        var dataset = NetworkEvidence.Normalize(networkSource,bundle.Request,log,token);
        var terrain = await terrainTask.ConfigureAwait(false);
        log.Diagnostic($"PROCESS terrain/parsing complete; networkWays={dataset.Ways.Count}; heightSamples={terrain.Heightmap.Length}; starting water classification");
        SurfaceMask? surfaces = null;
        // River shaping changes the ground beneath crossings and tunnels. Plan
        // it before network grading so every final cover/clearance constraint is
        // solved against the terrain that will actually be applied.
        progress.Begin("Planning rivers and lakes");
        var water=OsmRecovery.Run("water-planning",()=>WaterPlanner.Prepare(geography,polygons,bundle.Request,terrain,log,token,waterCellSize,progress.Detail),new WaterPlan(false,0,0,0,"Water planning unavailable; original terrain retained. See OSM recovery diagnostics."),log,token);
        if(water.ConditionedHeightmap!=null)terrain=terrain with{Heightmap=water.ConditionedHeightmap};
        IReadOnlyList<AirDestination> destinations = Array.Empty<AirDestination>();
        if (bundle.Request.CreateAirConnections)
        {
            try { progress.Begin("Planning airplane connections"); destinations = AirPlanner.Select(bundle.Request, log); }
            catch (Exception exception) when (exception is not OperationCanceledException) { log.Warn("Air destination preparation skipped: " + exception.Message); }
        }
        token.ThrowIfCancellationRequested();
        log.Info("Building OSM topology and cleaning network geometry.");
        progress.Begin("Connecting roads and simplifying nearby routes");
        var clock = Stopwatch.StartNew();
        // Core network construction cannot fall back to an empty import: retain
        // useful stage context, but never turn a processing fault into missing roads.
        var rawSegments = NetworkProcessor.BuildSegments(dataset, bundle.Request, log, token);
        rawSegments = OsmRecovery.Run("avenue-consolidation",()=>AvenueConsolidator.Prepare(rawSegments, log, token),rawSegments,log,token);
        rawSegments = RoundaboutPlanner.Prepare(rawSegments,log,token);
        if(bundle.Request.ImportRail) { progress.Begin("Combining nearby railway tracks"); rawSegments = OsmRecovery.Run("rail-consolidation",()=>RailCorridorConsolidator.Prepare(rawSegments, log, token),rawSegments,log,token); }
        log.Diagnostic($"PROCESS source-topology complete; sections={rawSegments.Count}; starting normalization");
        log.Time("Network cleanup", clock.Elapsed.TotalSeconds);
        log.Info("Simplifying protected network chains and fitting bounded-error curves.");
        progress.Begin("Smoothing road and railway curves");
        var normalized = OsmRecovery.Run("normalization",()=>NetworkNormalizer.Normalize(rawSegments, log, token, bundle.Request.MapSize.WidthMeters),rawSegments,log,token);
        var hardened = OsmRecovery.Run("network-hardening",()=>NetworkHardener.Prepare(normalized, log, token, bundle.Request.ImportRail),normalized,log,token);
        hardened=NetworkWidthRepair.Prepare(hardened,s=>widths.TryGetValue(PrefabResolver.SizedPaletteKey(s),out var sized)?sized:widths.TryGetValue(PrefabResolver.PaletteKey(s),out var value)?value:float.PositiveInfinity,bundle.Request.MapSize.WidthMeters,log,token);
        hardened=OsmRecovery.RepairGeometry(hardened,bundle.Request.MapSize.WidthMeters,log,token);
        hardened=hardened.Select(s=>PrefabResolver.ApplyMetadata(s,widths,laneOffsets)).ToArray();
        hardened=OsmRecovery.Run("final-road-horizontal-joins",()=>NetworkNormalizer.RefineFinalRoadJoins(hardened,log,token,bundle.Request.MapSize.WidthMeters),hardened,log,token);
        hardened=NativeJunctionSimplifier.Prepare(hardened,log,token);
        hardened=NativeChainCompactor.Prepare(hardened,log,token);
        log.Diagnostic($"PROCESS terrain-alignment begin; sections={hardened.Count}");
        progress.Begin("Smoothing road and railway slopes");
        var initialAlignment = NetworkProcessor.AlignToTerrain(hardened, terrain.Heightmap, terrain.HeightScaleMeters, bundle.Request.MapSize.WidthMeters, token);
        var aligned = OsmRecovery.Run("rail-grading",()=>RailVerticalProfiles.Prepare(initialAlignment,terrain.Heightmap,terrain.HeightScaleMeters,bundle.Request.MapSize.WidthMeters,log,token),initialAlignment,log,token);
        aligned=OsmRecovery.Run("road-grading",()=>NetworkVerticalProfiles.Prepare(aligned, terrain.Heightmap, terrain.HeightScaleMeters, bundle.Request.MapSize.WidthMeters, log, token),aligned,log,token);
        aligned=TunnelCorridorPlanner.Prepare(aligned,terrain.Heightmap,terrain.HeightScaleMeters,bundle.Request.MapSize.WidthMeters,log,token);
        aligned=OsmRecovery.RepairGeometry(aligned,bundle.Request.MapSize.WidthMeters,log,token);
        aligned=aligned.Select(s=>PrefabResolver.ApplyMetadata(s,widths,laneOffsets)).ToArray();
        aligned=NetworkLaneAlignment.Prepare(aligned,log,token,bundle.Request.MapSize.WidthMeters);
        // Lane offsets change the actual mesh footprint. Recheck spacing after
        // alignment, rather than solving clearance against obsolete centrelines.
        aligned=NetworkWidthRepair.Prepare(aligned,s=>s.NativeWidth>0?s.NativeWidth:s.Kind==NetworkKind.Rail?12:float.PositiveInfinity,bundle.Request.MapSize.WidthMeters,log,token);
        aligned=CorridorProfiles.Prepare(aligned,log,token,WaterBodyPlanner.SurfaceSampler(water,polygons));
        // This iteration retains the best bounded repair and reports remaining quality
        // defects. Invalid coordinates, exposed tunnels and broken topology still block.
        aligned=NetworkGeometrySafety.Repair(aligned,terrain.Heightmap,terrain.HeightScaleMeters,bundle.Request.MapSize.WidthMeters,log,token,allowQualityWarnings:true,waterLevel:WaterBodyPlanner.SurfaceSampler(water,polygons));
        clock.Restart();
        log.Info("Validating normalized road and rail coverage/geometry.");
        progress.Begin("Checking road and railway connections");
        log.Diagnostic($"PROCESS geometry-validation begin; sections={aligned.Count}");
        NetworkGeometrySafety.ValidateEnvelope(aligned,bundle.Request.MapSize.WidthMeters,terrain.HeightScaleMeters,token);
        progress.Begin("Blending the ground beneath roads and railways");
        var networkTerrain=terrain;
        var protectedAreas=polygons.Where(p=>p.Tags.ContainsKey("water")||p.Tags.ContainsKey("waterway")||p.Tags.TryGetValue("natural",out var natural)&&natural=="water"||p.Tags.TryGetValue("landuse",out var landuse)&&landuse=="quarry").SelectMany(p=>p.Outer).ToArray();
        var protectedLines=geography==null?Array.Empty<Unity.Mathematics.float3[]>():geography.Ways.Values.Where(w=>w.Tags.ContainsKey("waterway")||w.Tags.TryGetValue("natural",out var natural)&&(natural=="cliff"||natural=="coastline"))
            .Select(w=>w.NodeIds.Where(geography.Nodes.ContainsKey).Select(id=>{var node=geography.Nodes[id];return GeoMath.ToLocalMeters(node.Latitude,node.Longitude,bundle.Request.Latitude,bundle.Request.Longitude);}).ToArray()).ToArray();
        var wallLines=dataset.Ways.Values.Where(w=>w.Tags.TryGetValue("barrier",out var barrier)&&barrier=="retaining_wall")
            .Select(w=>w.NodeIds.Where(dataset.Nodes.ContainsKey).Select(id=>{var node=dataset.Nodes[id];return GeoMath.ToLocalMeters(node.Latitude,node.Longitude,bundle.Request.Latitude,bundle.Request.Longitude);}).ToArray()).ToArray();
        aligned=NetworkCutPlanner.Prepare(aligned,terrain.Heightmap,terrain.HeightScaleMeters,bundle.Request.MapSize.WidthMeters,protectedAreas,protectedLines,wallLines,log,token);
        var roadbedProfiles=aligned;
        var roadbedRaster=NetworkRoadbed.Prepare(terrain.Heightmap,terrain.HeightScaleMeters,bundle.Request.MapSize.WidthMeters,aligned,protectedAreas,protectedLines,log,token);
        terrain=terrain with{Heightmap=roadbedRaster};
        log.CaptureGeometry("before-cover",aligned,terrain.Heightmap,terrain.HeightScaleMeters,bundle.Request.MapSize.WidthMeters);
        aligned=NetworkGeometrySafety.RepairFinalTunnelCover(aligned,terrain.Heightmap,terrain.HeightScaleMeters,bundle.Request.MapSize.WidthMeters,log,token);
        water=WaterBodyPlanner.Revalidate(water,terrain,bundle.Request.MapSize.WidthMeters,log,token,aligned,progress.Detail);
        if(water.ConditionedHeightmap!=null)terrain=terrain with{Heightmap=water.ConditionedHeightmap};
        // Water bed revalidation can lower the roof after the first tunnel audit.
        // Recover against the actual final raster, then grade all incident approaches.
        aligned=NetworkGeometrySafety.RepairFinalTunnelCover(aligned,terrain.Heightmap,terrain.HeightScaleMeters,bundle.Request.MapSize.WidthMeters,log,token);
        aligned=CorridorProfiles.Prepare(aligned,log,token,WaterBodyPlanner.SurfaceSampler(water,polygons),finishOnly:true);
        aligned=NetworkGeometrySafety.RepairFinalTunnelCover(aligned,terrain.Heightmap,terrain.HeightScaleMeters,bundle.Request.MapSize.WidthMeters,log,token);
        terrain=terrain with{Heightmap=NetworkRoadbed.Refresh(networkTerrain.Heightmap,roadbedRaster,terrain.Heightmap,terrain.HeightScaleMeters,bundle.Request.MapSize.WidthMeters,roadbedProfiles,aligned,protectedAreas,protectedLines,log,token)};
        NetworkGeometrySafety.ValidateEnvelope(aligned,bundle.Request.MapSize.WidthMeters,terrain.HeightScaleMeters,token);
        NetworkGeometrySafety.Validate(aligned,log,token,allowQualityWarnings:true);
        NetworkGeometrySafety.ValidateTunnelCover(aligned,terrain.Heightmap,terrain.HeightScaleMeters,bundle.Request.MapSize.WidthMeters,token);
        NetworkContinuity.Validate(hardened,aligned,log,token,allowQualityWarnings:true);
        if(bundle.Request.ApplyGroundSurfaces&&geography!=null)
        {
            try{progress.Begin("Planning ground textures");surfaces=SurfacePlanner.Prepare(polygons,bundle.Request,terrain,log,token,geography,climate:bundle.Climate);}
            catch(Exception exception) when(exception is not OperationCanceledException){log.Warn("Surface preparation skipped: "+exception.Message);}
        }
        log.Diagnostic($"PROCESS geometry-validation complete; elapsedSeconds={clock.Elapsed.TotalSeconds:0.0}; starting optional forests");
        log.Time("Geometry validation", clock.Elapsed.TotalSeconds);
        IReadOnlyList<Unity.Mathematics.float3> trees = Array.Empty<Unity.Mathematics.float3>();
        OsmDataset? forestData=null;
        try
        {
            if(bundle.ForestDocuments!=null)
            {
                log.Info("Preparing independently cached forest masks and tree candidates.");
                progress.Begin("Choosing tree locations");
                var forestClock=Stopwatch.StartNew();
                forestData=OsmRecovery.Parse(bundle.ForestDocuments,parsedCacheRoot,"forests",log,token);
                log.Time("Forest OSM parse",forestClock.Elapsed.TotalSeconds);
                trees=ForestProcessor.Prepare(forestData,bundle.Request,terrain,aligned,log,token);
            }
            else log.Info(bundle.Request.ImportForests ? "Optional vegetation skipped: independent forest data unavailable; core terrain/networks remain valid." : "Optional vegetation skipped: forestry disabled by user.");
        }
        catch (Exception exception) when (exception is not OperationCanceledException) { log.Warn($"Vegetation preparation skipped: {exception.Message}"); }
        token.ThrowIfCancellationRequested();
        var vegetation=VegetationPlanner.Prepare(forestData,trees,bundle.Request,terrain,aligned,polygons,water,bundle.Climate,log,token);
        return new ProcessedBundle(bundle, terrain, aligned, trees, dataset.Ways.Count, surfaces, destinations, water)
            {Vegetation=vegetation,HabitatPolygons=polygons,NetworkData=dataset,NetworkTerrain=networkTerrain,ProtectedAreas=protectedAreas,ProtectedLines=protectedLines,ExplicitWallLines=wallLines,ForestData=forestData,SurfaceGeography=bundle.Request.ApplyGroundSurfaces?geography:null,SurfacePolygons=bundle.Request.ApplyGroundSurfaces?polygons:Array.Empty<GeographicPolygon>()};
    }

    private async Task<DownloadBundle> RecoverTerrainAsync(DownloadBundle bundle,IReadOnlyList<int> invalid,CancellationToken token)
    {
        var client=new TerrainTileClient(new CacheStore(_log),_log);
        var tiles=await client.RecoverAsync(bundle.TerrainTiles,invalid,token).ConfigureAwait(false);
        return bundle with{TerrainTiles=tiles};
    }

    private Task<ProcessedBundle> RepairNativeFootprintsAsync(ProcessedBundle result,CancellationToken token)
    {
        // Only immutable native metadata is used off-thread. The retained terrain
        // predates roadbed work, so the retry does not accumulate cuts/fills.
        var log=_log;var progress=_progress;
        return Task.Run(()=>
        {
            token.ThrowIfCancellationRequested();var clock=Stopwatch.StartNew();
            var terrain=result.NetworkTerrain??result.Terrain;var request=result.Download.Request;
            progress.Begin("Repairing actual road and railway footprints");
            var aligned=NetworkLaneAlignment.Prepare(result.Segments,log,token,request.MapSize.WidthMeters);
            var sections=NetworkWidthRepair.Prepare(aligned,s=>s.NativeWidth,request.MapSize.WidthMeters,log,token);
            sections=CorridorProfiles.Prepare(sections,log,token,WaterBodyPlanner.SurfaceSampler(result.Water,result.HabitatPolygons));
            sections=NetworkGeometrySafety.Repair(sections,terrain.Heightmap,terrain.HeightScaleMeters,request.MapSize.WidthMeters,log,token,allowQualityWarnings:true,waterLevel:WaterBodyPlanner.SurfaceSampler(result.Water,result.HabitatPolygons));
            sections=NetworkCutPlanner.Prepare(sections,terrain.Heightmap,terrain.HeightScaleMeters,request.MapSize.WidthMeters,result.ProtectedAreas,result.ProtectedLines,result.ExplicitWallLines,log,token);
            var roadbedBaseline=terrain.Heightmap;var roadbedProfiles=sections;
            var roadbedRaster=NetworkRoadbed.Prepare(terrain.Heightmap,terrain.HeightScaleMeters,request.MapSize.WidthMeters,sections,result.ProtectedAreas,result.ProtectedLines,log,token);
            log.CaptureGeometry("native-before-cover",sections,roadbedRaster,terrain.HeightScaleMeters,request.MapSize.WidthMeters);
            terrain=terrain with{Heightmap=roadbedRaster};
            sections=NetworkGeometrySafety.RepairFinalTunnelCover(sections,terrain.Heightmap,terrain.HeightScaleMeters,request.MapSize.WidthMeters,log,token);
            var water=WaterBodyPlanner.Revalidate(result.Water,terrain,request.MapSize.WidthMeters,log,token,sections);
            if(water.ConditionedHeightmap!=null)terrain=terrain with{Heightmap=water.ConditionedHeightmap};
            sections=NetworkGeometrySafety.RepairFinalTunnelCover(sections,terrain.Heightmap,terrain.HeightScaleMeters,request.MapSize.WidthMeters,log,token);
            sections=CorridorProfiles.Prepare(sections,log,token,WaterBodyPlanner.SurfaceSampler(water,result.HabitatPolygons),finishOnly:true);
            sections=NetworkGeometrySafety.RepairFinalTunnelCover(sections,terrain.Heightmap,terrain.HeightScaleMeters,request.MapSize.WidthMeters,log,token);
            terrain=terrain with{Heightmap=NetworkRoadbed.Refresh(roadbedBaseline,roadbedRaster,terrain.Heightmap,terrain.HeightScaleMeters,request.MapSize.WidthMeters,roadbedProfiles,sections,result.ProtectedAreas,result.ProtectedLines,log,token)};
            NetworkGeometrySafety.ValidateEnvelope(sections,request.MapSize.WidthMeters,terrain.HeightScaleMeters,token);
            NetworkGeometrySafety.Validate(sections,log,token,allowQualityWarnings:true);
            NetworkGeometrySafety.ValidateTunnelCover(sections,terrain.Heightmap,terrain.HeightScaleMeters,request.MapSize.WidthMeters,token);
            NetworkContinuity.Validate(result.Segments,sections,log,token,allowQualityWarnings:true);
            if(sections.Count!=result.Segments.Count)throw new InvalidOperationException("Native footprint repair changed section coverage; complete network retained instead of applying a subset.");
            var trees=result.ForestData==null?result.Trees:ForestProcessor.Prepare(result.ForestData,request,terrain,sections,log,token);
            var vegetation=VegetationPlanner.Prepare(result.ForestData,trees,request,terrain,sections,result.HabitatPolygons,water,result.Download.Climate,log,token);
            var surfaces=result.Surfaces;
            if(request.ApplyGroundSurfaces&&result.SurfaceGeography!=null)
            {
                try { progress.Begin("Refreshing ground textures for the final terrain"); surfaces=SurfacePlanner.Prepare(result.SurfacePolygons,request,terrain,log,token,result.SurfaceGeography,climate:result.Download.Climate); }
                catch(Exception exception) when(exception is not OperationCanceledException) { surfaces=null; log.Warn("Surface refresh unavailable: "+exception.Message); }
            }
            log.Time("Native footprint reconciliation",clock.Elapsed.TotalSeconds);
            log.Diagnostic($"PREFLIGHT repaired actual native footprints; sections={sections.Count}; deletedSections=0; repeatLimit=1; bounds/continuity/tunnelCoverRechecked=true");
            return result with{Terrain=terrain,Segments=sections,Trees=trees,Vegetation=vegetation,Surfaces=surfaces,Water=water,NativeFootprintsRepaired=true};
        },token);
    }

    private void CompleteProcessing()
    {
        if (_processTask!.IsCanceled) { ReturnToIdle("Preparation cancelled. Nothing changed."); return; }
        if (_processTask.IsFaulted) { Fail(_processTask.Exception!, "background-processing"); return; }
        var validationPhase = "native-terrain-validation";
        try
        {
            var result = _processTask.Result;
            if (!ImportSession.Matches(result.Download.Request.MapSessionId, result.Download.Request.Generation)) { _log.Warn("Discarded stale previous-map/processing result."); ReturnToIdle("Stale preparation discarded."); return; }
            var errors = new List<string>();
            var warnings = new List<string>();
            if(_log.Metric("Network repair quality observations")>0)warnings.Add("Test iteration: some network overlaps or grades remain after bounded repair. Use a disposable map and inspect interchanges; this is not production acceptance.");
            var recoveryCount=_log.Metric("OSM recovery stages")+_log.Metric("Road profiles retained with relaxed limits")+_log.Metric("Rail profiles retained with relaxed limits")+_log.Metric("OSM unreadable tiles")+_log.Metric("OSM unavailable download tiles")+_log.Metric("OSM network downloads unavailable")+_log.Metric("OSM irrecoverable geometry sections omitted")+_log.Metric("Carriageway conflicts retained for review")+_log.Metric("Network continuity unresolved sections")+_log.Metric("Rail overlap observations");
            if(recoveryCount>0)warnings.Add("Some map data required recovery or relaxed quality limits. Generation can continue, but road/rail quality or coverage may be incomplete; review the log.");
            var trees = result.Trees;
            var validationClock = Stopwatch.StartNew();
            _progress.Begin("Checking compatibility with the game");
            _status.Update(_progress.Snapshot());
            var terrainBridge = new TerrainBridge(
                World.GetOrCreateSystemManaged<TerrainSystem>(),
                World.GetOrCreateSystemManaged<WaterSystem>());
            if (!terrainBridge.CanApply(out var terrainReason)) errors.Add(terrainReason);
            if (result.Segments.Count > 0 && !NetworkCommitSystem.CanCommit(out var commitReason)) errors.Add(commitReason);

            validationPhase = "native-prefab-availability";
            var resolver = new PrefabResolver(World.GetOrCreateSystemManaged<PrefabSystem>(), _log);
            if (result.Segments.Any(segment => segment.Kind == NetworkKind.Road) && !resolver.CanResolveRoad(out var roadReason)) errors.Add(roadReason);
            if (result.Segments.Any(segment => segment.Kind == NetworkKind.Rail) && !resolver.CanResolveRail(out var railReason)) errors.Add(railReason);
            if (result.Download.Request.ImportRail && !result.Segments.Any(segment => segment.Kind == NetworkKind.Rail)) warnings.Add("No selected heavy-rail segments remain inside these bounds; see the processing log.");
            if (result.Download.Request.RoadLevel != RoadImportLevel.None && !result.Segments.Any(segment => segment.Kind == NetworkKind.Road)) warnings.Add("No selected road sections remain inside these bounds; see the processing log.");
            var geometrySkips = _log.Metric("Unsafe carriageway sections omitted") + _log.Metric("Roads rejected for impossible grade/profile") + _log.Metric("OSM irrecoverable geometry sections omitted");
            if (geometrySkips > 0) errors.Add($"Network preparation lost {geometrySkips:0} sections. Application blocked to avoid disconnected roads; see the log.");
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
            validationPhase = "native-network-preflight";
            _log.Diagnostic($"PREFLIGHT BEGIN phase={validationPhase}; segments={result.Segments.Count}");
            var supported = NativeNetworkPlacement.Preflight(result.Segments, World.GetOrCreateSystemManaged<PrefabSystem>(), _log,out var needsRepair);
            result = result with { Segments = supported };
            if(needsRepair)
            {
                if(result.NativeFootprintsRepaired)throw new InvalidOperationException("Native prefab metadata changed again after complete-network repair. Nothing replaced; prepare again with a stable asset catalog.");
                _log.Warn("Actual game footprint differs from preparation metadata. Repairing and re-auditing every retained network section; no segments skipped.");
                _progress.Begin("Repairing actual road and railway footprints");
                _status.Update(_progress.Snapshot());
                _processConsumed=false;
                _processTask=RepairNativeFootprintsAsync(result,_cancellation!.Token);
                return;
            }

            validationPhase = "prepared-result/summary";
            _prepared = new PreparedImport
            {
                Request = result.Download.Request,
                Bounds = result.Download.Bounds,
                Heightmap = result.Terrain.Heightmap,
                HeightScaleMeters = result.Terrain.HeightScaleMeters,
                SeaLevelMeters = result.Terrain.SeaLevelMeters,
                Segments = result.Segments,
                Trees = trees,
                Vegetation=result.Vegetation,
                SurfaceMask = result.Surfaces,
                AirDestinations = result.Destinations,
                Climate = result.Download.Climate,
                Water = result.Water,
                NetworkEvidence=result.NetworkData,
                Warnings = warnings,
                SourceWayCount = result.SourceWayCount
            };
            _log.CapturePrepared(_prepared);
            _log.Checkpoint("preparation/ready",new { segments=_prepared.Segments.Count, vegetation=_prepared.Vegetation.Count, mapSession=_prepared.Request.MapSessionId, generation=_prepared.Request.Generation, mapUnchanged=true });
            var roads = result.Segments.Count(segment => segment.Kind == NetworkKind.Road);
            var rail = result.Segments.Count(segment => segment.Kind == NetworkKind.Rail);
            _log.Time("Runtime validation", validationClock.Elapsed.TotalSeconds);
            _summary.Update("");
            _log.Info("Preparation summary: " + NetworkSelection.Summary(result.Download.Request.RoadLevel, result.Download.Request.ImportRail) + $"\nTerrain {result.Terrain.MinimumElevation:0}–{result.Terrain.MaximumElevation:0} m • {roads:N0} road sections" + (result.Download.Request.ImportRail ? $" • {rail:N0} rail sections" : "") + $" • {trees.Count:N0} optional trees • {warnings.Count} warnings\n" + result.Terrain.ConditioningSummary + "\n" + result.Water.Report);
            foreach (var warning in warnings) _log.Warn(warning);
            _ready.Update(true);
            _busy.Update(false);
            _stage.Update((int)ImportStage.ReadyForConfirmation);
            _status.Update("Preparation complete. Save a backup, then choose Replace current map." + (warnings.Count > 0 ? " Review warnings in View log." : ""));
            _log.Info("Preflight complete. No editor entities have been modified.");
            _log.Info("Preparation statistics: " + _log.Metrics());
            _log.FlushArchive("prepared");
        }
        catch (Exception exception) { Fail(exception, validationPhase); }
    }

    private void ConfirmApply()
    {
        if (!_busy.value) _actionRevision.Update((++_actionCounter).ToString(CultureInfo.InvariantCulture));
        if (_prepared == null || !_ready.value || _busy.value) return;
        if (!ImportSession.Matches(_prepared.Request.MapSessionId, _prepared.Request.Generation) || _prepared.Request.RoadLevel != _roadLevel || _prepared.Request.ImportRail != _importRail || _prepared.Request.ImportForests != _importForests)
        { InvalidatePrepared(); _status.Update("Prepared network settings/map are stale. Prepare again."); return; }
        if (!ImportApplySystem.TryRequest(_prepared, _log)) { Fail("Another import is already applying."); return; }
        _busy.Update(true);
        _ready.Update(false);
        _stage.Update((int)ImportStage.Applying);
        _progress.Begin("Replacing your map");
        _nextProgressUpdate = DateTime.UtcNow.AddSeconds(1);
        _status.Update("Replacing your map. This may take up to 5 minutes; please wait. This step cannot be cancelled.");
        _log.FlushArchive("applying");
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

    private void OnApplyProgress(string message)
    {
        _progress.Begin(message.TrimEnd('…', '.', ' '));
        _status.Update(_progress.Snapshot(false));
        _nextProgressUpdate = DateTime.UtcNow.AddSeconds(1);
    }

    private void OnApplyCompleted(int created, int skipped)
    {
        _busy.Update(false);
        _stage.Update((int)ImportStage.Complete);
        _status.Update("Import complete. Save the map normally in Cities: Skylines II. Details are in View log.");
        _summary.Update("");
        _log.Info($"Import complete: {created:N0} native network segments queued, {skipped:N0} skipped.");
        if (_prepared != null) _log.Info(_prepared.FeatureReport);
        _log.Info("Completion statistics: " + _log.Metrics());
        _log.Diagnostic("IMPORT TOTALS (definition counts, not final live-entity proof): " + _log.Metrics());
        _log.FlushArchive("completed");
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

    private void Fail(Exception exception, string phase)
    {
        // Preserve typed recovery outcomes through task/reflection wrappers;
        // GetBaseException would discard them when they retain a source cause.
        var cause=exception;
        while(cause is not PreparationDataException&&cause.InnerException!=null)cause=cause.InnerException;
        if(cause is PreparationDataException data&&_stage.value!=(int)ImportStage.Applying)
        {
            _cancellation?.Cancel();_prepared=null;_busy.Update(false);_ready.Update(false);_summary.Update("");
            _stage.Update((int)ImportStage.Idle);_error.Update("");_status.Update(data.UserMessage);
            _log.Warn(data.UserMessage);_log.Diagnostic($"PREPARATION DATA UNAVAILABLE phase={phase}; code={data.Issue}; detail={data.Message}; mapUnchanged=true; userInputError=false; emptyGeometrySubstitution=false");
            _log.FlushArchive("preparation-data-unavailable");return;
        }
        _log.Diagnostic($"FAILURE phase={phase}; mapSession={ImportSession.Map}; exception={exception}");
        Fail(exception.GetBaseException().Message);
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
        _log.FlushArchive(applying ? "application-failed" : "preparation-failed");
    }

    private void ReturnToIdle(string message)
    {
        _busy.Update(false);
        _ready.Update(false);
        _stage.Update((int)ImportStage.Idle);
        _status.Update(message);
        _log.FlushArchive("stopped-before-application");
    }

    private static bool TryNumber(string value, out double number) =>
        double.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out number) ||
        double.TryParse(value, NumberStyles.Float, CultureInfo.CurrentCulture, out number);

    private sealed record DownloadBundle(
        ImportRequest Request,
        GeoBounds Bounds,
        IReadOnlyList<TerrainTile> TerrainTiles,
        IReadOnlyList<byte[]> OsmDocuments,
        IReadOnlyList<byte[]>? ForestDocuments,
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
        WaterPlan Water)
    {
        public TerrainBuilder.Result? NetworkTerrain { get; init; }
        public OsmDataset? NetworkData { get; init; }
        public IReadOnlyList<VegetationCandidate> Vegetation { get; init; }=Array.Empty<VegetationCandidate>();
        public IReadOnlyList<GeographicPolygon> HabitatPolygons { get; init; }=Array.Empty<GeographicPolygon>();
        public Unity.Mathematics.float3[][] ProtectedAreas { get; init; }=Array.Empty<Unity.Mathematics.float3[]>();
        public Unity.Mathematics.float3[][] ProtectedLines { get; init; }=Array.Empty<Unity.Mathematics.float3[]>();
        public Unity.Mathematics.float3[][] ExplicitWallLines { get; init; }=Array.Empty<Unity.Mathematics.float3[]>();
        public OsmDataset? ForestData { get; init; }
        public OsmDataset? SurfaceGeography { get; init; }
        public IReadOnlyList<GeographicPolygon> SurfacePolygons { get; init; }=Array.Empty<GeographicPolygon>();
        public bool NativeFootprintsRepaired { get; init; }
    }
}
}
