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
    private GameManager? _saveManager;
    private TerrainBridge _terrain = null!;
    private IReadOnlyList<Entity>? _definitions;
    private int _phase;
    private int _created;
    private int _skipped;
    private NativeVegetationPlacement.StockPrefab[] _treePrefabs = Array.Empty<NativeVegetationPlacement.StockPrefab>();
    private int _treeIndex;
    private int _roadSections, _railSections;
    private NativeNetworkPlacement? _networkPlacement;
    private NativeSubmissionPlan? _submissionPlan;
    private int _networkIndex;
    private IReadOnlyList<PreparedSegment> _networkBatch = Array.Empty<PreparedSegment>();
    private Entity[] _oldNetworks = Array.Empty<Entity>();
    private int _retirementFrames;
    private int _vegetationRetirementFrames;
    private int _readbackFrames;
    private int _readbackStartFrame;
    private int _recoveryAttempts;
    private int _recoveryWaitFrames;
    private int _batchSizeLimit=NetworkBatchSize;
    private int _recoveryUntilIndex;
    private bool _mapChanged;
    private List<IReadOnlyList<PreparedSegment>> _preflightBatches=new();
    private int _preflightIndex;
    private readonly HashSet<string> _attemptedInputs=new(StringComparer.Ordinal);
    private int _preferredBatchSize=NetworkBatchSize;
    // Limit command-buffer pressure, not map coverage. Every section is submitted.
    private const int NetworkBatchSize = 32;
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
        _saveManager=GameManager.instance;
        _saveManager.onGameSaveLoad+=OnSaveDiagnostic;
        _prefabSystem = World.GetOrCreateSystemManaged<PrefabSystem>();
        _terrain = new TerrainBridge(
            World.GetOrCreateSystemManaged<TerrainSystem>(),
            World.GetOrCreateSystemManaged<WaterSystem>());
    }

    protected override void OnUpdate()
    {
        NativeDiagnostics.Poll(World,EntityManager);
        PreparedImport? prepared;
        lock (Gate) prepared = s_Pending;
        if (prepared == null) return;
        if (!ImportSession.Matches(prepared.Request.MapSessionId, prepared.Request.Generation) || GameManager.instance.isGameLoading || GameManager.instance.gameMode != GameMode.Editor)
        { s_Log?.Warn("Stale previous-map placement rejected."); AbortMapWork(); return; }

        try
        {
            if (_phase == 0)
            {
                Progress?.Invoke("Checking native road and railway generation before replacing the map…");
                if (GameManager.instance.gameMode != GameMode.Editor)
                    throw new InvalidOperationException("The game left Map Editor mode before confirmation.");
                if (!MapExtDetector.TryGetCurrentMapWidth(out var actualWidth) ||
                    Math.Abs(actualWidth - prepared.Request.MapSize.WidthMeters) > 2d)
                    throw new InvalidOperationException("The loaded map size changed or could not be verified after preflight.");
                if (!_terrain.CanApply(out var terrainReason)) throw new InvalidOperationException(terrainReason);
                if (!NetworkCommitSystem.CanCommit(out var networkReason) && prepared.Segments.Count > 0)
                    throw new InvalidOperationException(networkReason);
                if((prepared.Segments.Count>0||prepared.Vegetation.Count>0)&&!NetworkCommitSystem.CanStartExclusive(EntityManager,out var previewReason))
                    throw new InvalidOperationException(previewReason);

                var resolver = new PrefabResolver(_prefabSystem, s_Log!);
                if (System.Linq.Enumerable.Any(prepared.Segments, segment => segment.Kind == NetworkKind.Road) &&
                    !resolver.CanResolveRoad(out var roadReason))
                    throw new InvalidOperationException(roadReason + " Application stopped before replacing the map.");
                if (System.Linq.Enumerable.Any(prepared.Segments, segment => segment.Kind == NetworkKind.Rail) &&
                    !resolver.CanResolveRail(out var railReason))
                    throw new InvalidOperationException(railReason + " Application stopped before replacing the map.");

                Processing.NetworkGeometrySafety.Validate(prepared.Segments,s_Log!,default,allowQualityWarnings:true);
                Processing.NetworkGeometrySafety.ValidateEnvelope(prepared.Segments,prepared.Request.MapSize.WidthMeters,prepared.HeightScaleMeters,default);
                Processing.NetworkGeometrySafety.ValidateTunnelCover(prepared.Segments,prepared.Heightmap,prepared.HeightScaleMeters,prepared.Request.MapSize.WidthMeters,default);
                foreach(var segment in prepared.Segments)
                {
                    var asset=resolver.Resolve(segment);
                    if(asset==Entity.Null)
                        throw new InvalidOperationException($"Network prefab missing for {segment.Kind} way {segment.WayId}; application stopped before replacing the map.");
                    var width=resolver.ReadWidth(asset,segment.Bridge);
                    if(float.IsNaN(width)||float.IsInfinity(width)||width<=0||segment.NativeWidth<=0||width>segment.NativeWidth+.01f)
                        throw new InvalidOperationException($"Network asset width changed for {segment.Kind} way {segment.WayId}; application stopped before replacing the map. Audited={segment.NativeWidth:0.##} m, selected={width:0.##} m.");
                }

                _networkPlacement = new NativeNetworkPlacement(EntityManager, _prefabSystem, s_Log!);
                _networkPlacement!.Initialize(prepared.Segments,prepared.NetworkEvidence);
                _submissionPlan=new NativeSubmissionPlan(prepared.Segments);
                s_Log?.DiagnosticRecords("native-submission-order.ndjson",DiagnosticArtifacts.NetworkRecords(_submissionPlan.Sections));
                s_Log?.Diagnostic($"NATIVE SCHEDULE sections={_submissionPlan.Sections.Count}; connectedSourceOrder=true; curvesChanged=false; omittedSections=0; maximumBatch={NetworkBatchSize}");
                _preflightBatches=_submissionPlan.RiskPreflight(NetworkBatchSize);_preflightIndex=0;
                _networkBatch = _preflightBatches.Count>0?_preflightBatches[0]:Array.Empty<PreparedSegment>();
                s_Log?.Diagnostic($"NATIVE RISK PREFLIGHT clusters={_preflightBatches.Count}; originalMapUnchanged=true; nativeInteractionWithFutureCommittedGraphNotGuaranteed=true");
                NetworkBatchCheckpoint("apply/definition-preflight",prepared,0);
                _definitions = _networkPlacement.CreateDefinitions(_networkBatch, out _skipped);
                if(_skipped>0)
                {
                    foreach(var definition in _definitions)EntityManager.DestroyEntity(definition);
                    _definitions=null;
                    throw new InvalidOperationException($"Native network preflight failed for {_skipped} sections. Application stopped before replacing the map; no disconnected subset retained.");
                }
                _created = prepared.Segments.Count;
                if(_created>0){_phase=9;return;}
                _phase=11;
            }
            if(_phase==9)
            {
                EntityManager.CompleteAllTrackedJobs();
                foreach(var definition in _definitions!)EntityManager.DestroyEntity(definition);
                _definitions=null;
                NetworkCommitSystem.RequestCommit(prepared.Request.MapSessionId,prepared.Request.Generation,s_Log,
                NetworkBatchContext(prepared,_preflightIndex*NetworkBatchSize),temporary=>_networkPlacement!.CaptureGeneratedNodes(_networkBatch,temporary),
                    _networkPlacement!.ArtifactStem,previewOnly:true);
                _phase=10;return;
            }
            if(_phase==10)
            {
                if(!NetworkCommitSystem.TryConsumeResult(out var preflightComplete,out var preflightFailure,out var preflightRecoverable)||!preflightComplete)return;
                if(preflightFailure!=null)
                {
                    if(!PrepareRecovery(prepared,preflightFailure,preflightRecoverable,preflight:true))
                        throw new InvalidOperationException(preflightFailure+" Native preflight stopped before replacing the map.");
                    return;
                }
                _readbackStartFrame=UnityEngine.Time.frameCount;_recoveryWaitFrames=0;
                _phase=14;return;
            }
            if(_phase==14)
            {
                if(UnityEngine.Time.frameCount-_readbackStartFrame<4)return;
                if(!NetworkCommitSystem.CanStartExclusive(EntityManager,out var clearedReason))
                {
                    if(++_recoveryWaitFrames==1||_recoveryWaitFrames%30==0)
                        NetworkCommitSystem.LogPendingRetirement(EntityManager,s_Log,"native-preflight-clear",_recoveryWaitFrames);
                    if(_recoveryWaitFrames>=180)throw new InvalidOperationException("Native preflight downstream retirement did not settle; map unchanged. "+clearedReason);
                    return;
                }
                _recoveryAttempts=0;_batchSizeLimit=NetworkBatchSize;_recoveryUntilIndex=0;
                _attemptedInputs.Clear();
                if(++_preflightIndex<_preflightBatches.Count)
                {
                    _networkBatch=_preflightBatches[_preflightIndex];
                    NetworkBatchCheckpoint("apply/definition-preflight",prepared,_preflightIndex*NetworkBatchSize);
                    _definitions=_networkPlacement!.CreateDefinitions(_networkBatch,out _skipped);
                    if(_skipped!=0)throw new InvalidOperationException("Risk preflight omitted required sections; original map retained.");
                    _phase=9;return;
                }
                // Preview anchors were never committed. Discard all staged bindings.
                _networkBatch=System.Linq.Enumerable.ToArray(System.Linq.Enumerable.Take(_submissionPlan!.Sections,NetworkBatchSize));
                _phase=11;
            }
            if(_phase==11)
            {
                Progress?.Invoke("Replacing terrain and removing the old roads, railways and water sources…");
                _definitions=null;
                _networkPlacement!.Initialize(prepared.Segments,prepared.NetworkEvidence);
                _oldNetworks=MapContentCleaner.SnapshotNetworks(EntityManager);

                var terrainClock = Stopwatch.StartNew();
                s_Log?.Checkpoint("apply/terrain-replacement",new { prepared.Request.MapSessionId, prepared.Request.Generation, segments=prepared.Segments.Count, oldNetworkEntities=_oldNetworks.Length, mapChanged=false });
                _terrain.Apply(prepared,s_Log);
                _mapChanged=true;
                s_Log?.Time("Terrain apply", terrainClock.Elapsed.TotalSeconds);
                s_Log?.Checkpoint("apply/old-network-deletion",new { oldNetworkEntities=_oldNetworks.Length, mapChanged=true });
                var deleted = MapContentCleaner.MarkForDeletion(EntityManager);
                s_Log?.Info($"Marked {deleted.networks} existing network entities and {deleted.waterSources} water sources for deletion after terrain replacement succeeded.");
                s_Log?.Info($"Sea level: nominal {prepared.SeaLevelMeters:0.0} m; effective water {WaterLevels.Effective(prepared.SeaLevelMeters):0.0} m (+{WaterLevels.CorrectionMeters:0.0} m).");

                s_Log?.Info($"Prepared {_created} native network sections; applying in batches of at most {NetworkBatchSize}, with verified source-node attachment between batches.");
                if (_created == 0) { _phase = 3; return; }
                _phase = 6;
                return;
            }

            if(_phase==6)
            {
                // Native deletion/creation playback must not share a city-sized frame.
                var remaining=MapContentCleaner.RemainingNetworks(EntityManager,_oldNetworks);
                if(++_retirementFrames<4||remaining>0)
                {
                    if(_retirementFrames<=180)return;
                    throw new InvalidOperationException($"Old native network teardown did not settle ({remaining} entities). New-network application stopped safely; reload the original map before retrying.");
                }
                if(!NetworkCommitSystem.CanStartExclusive(EntityManager,out var retiredPreviewReason))
                {
                    if(_retirementFrames==4||_retirementFrames%30==0)
                        NetworkCommitSystem.LogPendingRetirement(EntityManager,s_Log,"native-replacement",_retirementFrames);
                    if(_retirementFrames>=180)throw new InvalidOperationException("Native replacement downstream retirement did not settle. "+retiredPreviewReason);
                    return;
                }
                s_Log?.Diagnostic($"NATIVE REPLACEMENT oldEntities={_oldNetworks.Length}; retired=true; waitFrames={_retirementFrames}; newDefinitionsDuringTeardown=0");
                _oldNetworks=Array.Empty<Entity>();
                NetworkBatchCheckpoint("network/definitions",prepared,0);
                _definitions=_networkPlacement!.CreateDefinitions(_networkBatch,out _skipped);
                _networkIndex=_networkBatch.Count;
                _phase=1;return;
            }

            if (_phase == 1)
            {
                EntityManager.CompleteAllTrackedJobs();
                // Definition entities must survive one GenerateNodes/GenerateEdges pass, then be removed
                // before the second pass. Temp network entities remain for ApplyNetSystem.
                // NOTE: EntityManager.Exists() is intentionally not used here: with the
                // stock CS2 + Entities toolchain it breaks the UpdateAt<SystemType>
                // generic constraint (CS0311 on Mod.cs). DestroyEntity(Entity) compiles
                // and suffices because phase 1 runs exactly once per import.
                foreach (var definition in _definitions!)
                    EntityManager.DestroyEntity(definition);
                _definitions=null;
                NetworkCommitSystem.RequestCommit(prepared.Request.MapSessionId, prepared.Request.Generation,s_Log,NetworkBatchContext(prepared,_networkIndex-_networkBatch.Count),temporary=>_networkPlacement!.CaptureGeneratedNodes(_networkBatch,temporary),_networkPlacement!.ArtifactStem);
                Progress?.Invoke("Building roads and railways in your map…");
                _phase = 2;
                return;
            }

            if (_phase == 2 && NetworkCommitSystem.TryConsumeResult(out var complete, out var failure,out var recoverable) && complete)
            {
                if (failure != null)
                {
                    if(!PrepareRecovery(prepared,failure,recoverable,preflight:false))
                        throw new InvalidOperationException(failure+" Changed-input pre-commit recovery was unavailable or exhausted; no unsafe preview was applied.");
                    return;
                }
                _readbackFrames=0;_readbackStartFrame=UnityEngine.Time.frameCount;_phase=7;
            }
            if(_phase==8||_phase==12)
            {
                // Clear is asynchronous. Never generate a retry over its stale
                // temporary entities or another editor tool's live preview.
                if(++_recoveryWaitFrames<3||!NetworkCommitSystem.CanStartExclusive(EntityManager,out _))
                {
                    if(_recoveryWaitFrames<180)return;
                    throw new InvalidOperationException("Rejected native preview did not clear; regeneration stopped without another Apply.");
                }
                NetworkBatchCheckpoint("network/recovered-definitions",prepared,_networkIndex);
                _definitions=_networkPlacement!.CreateDefinitions(_networkBatch,out var retrySkipped);
                if(retrySkipped!=0)throw new InvalidOperationException("Recovery omitted a required network section.");
                if(_phase==12){_phase=9;return;}
                _networkIndex+=_networkBatch.Count;_phase=1;return;
            }
            if(_phase==7)
            {
                // Temp node/edge retirement is earlier than downstream lane
                // generation. Let modification barriers run before publishing
                // owners and generating another batch against those entities.
                var elapsedFrames=UnityEngine.Time.frameCount-_readbackStartFrame;
                if(elapsedFrames<4)return;
                if(!NetworkCommitSystem.CanStartExclusive(EntityManager,out var downstreamReason))
                {
                    if(++_readbackFrames==1||_readbackFrames%30==0)
                    {
                        s_Log?.Diagnostic($"NATIVE BATCH downstreamPending frames={elapsedFrames}; {downstreamReason}; duplicateApply=false; nextDefinitionsCreated=false");
                        NetworkCommitSystem.LogPendingRetirement(EntityManager,s_Log,_networkPlacement!.ArtifactStem,_readbackFrames);
                    }
                    if(_readbackFrames>=180)throw new InvalidOperationException("Native downstream preview retirement did not settle after committed playback: "+downstreamReason);
                    return;
                }
                if(!_networkPlacement!.TryCaptureCommittedNodes(out var pending))
                {
                    if(++_readbackFrames==1||_readbackFrames%30==0)s_Log?.Diagnostic($"NATIVE BATCH readbackPending frames={_readbackFrames}; {pending}; duplicateApply=false");
                    if(_readbackFrames>=180)throw new InvalidOperationException("Native graph did not become ready after committed playback: "+pending);
                    return;
                }
                s_Log?.Checkpoint("network/live-batch-verified",new { progress=_networkIndex,total=prepared.Segments.Count,waitFrames=_readbackFrames,downstreamElapsedFrames=elapsedFrames,sourceEntityLineage=true });
                _recoveryAttempts=0;
                _attemptedInputs.Clear();
                var workload=NetworkCommitSystem.LastMutationTargets;
                var suggested=workload>0?Math.Max(1,Math.Min(NetworkBatchSize,_networkBatch.Count*48/workload)):NetworkBatchSize;
                _preferredBatchSize=Math.Min(suggested,Math.Min(NetworkBatchSize,_preferredBatchSize+4));
                s_Log?.Diagnostic($"NATIVE ADAPTIVE BATCH verifiedSections={_networkBatch.Count}; distinctMutations={workload}; targetMutations=48; nextPreferredSections={_preferredBatchSize}; hardMutationGuard=64");
                if(_networkIndex>=_recoveryUntilIndex){_batchSizeLimit=_preferredBatchSize;_recoveryUntilIndex=0;}
                if (_networkIndex < prepared.Segments.Count)
                {
                    if(!NetworkCommitSystem.CanStartExclusive(EntityManager,out var batchPreviewReason))throw new InvalidOperationException(batchPreviewReason);
                    _networkBatch=System.Linq.Enumerable.ToArray(System.Linq.Enumerable.Take(System.Linq.Enumerable.Skip(_submissionPlan!.Sections,_networkIndex),_batchSizeLimit));
                    NetworkBatchCheckpoint("network/definitions",prepared,_networkIndex);
                    _definitions=_networkPlacement.CreateDefinitions(_networkBatch,out var batchSkipped);
                    if(batchSkipped!=0)throw new InvalidOperationException("Native batch omitted a required section.");
                    _networkIndex+=_networkBatch.Count;
                    s_Log?.Diagnostic($"NATIVE BATCH queued sections={_networkBatch.Count}; progress={_networkIndex}/{prepared.Segments.Count}; noWholeCityBuffer=true");
                    Progress?.Invoke($"Building roads and railways… {100L*_networkIndex/Math.Max(1,prepared.Segments.Count)}%");
                    _phase=1;return;
                }
                _roadSections=_networkPlacement.RoadSections;_railSections=_networkPlacement.RailSections;
                s_Log?.Checkpoint("network/roundabout-upgrades",new { _roadSections, _railSections, mapChanged=_mapChanged });
                NativeNetworkDiagnostics.ApplySmallRoundabouts(EntityManager,prepared.Segments,s_Log!);
                if (prepared.Request.RoadLevel != RoadImportLevel.None) s_Log?.Count("Road sections committed", _roadSections);
                if (prepared.Request.ImportRail) s_Log?.Count("Rail sections committed", _railSections);
                _definitions=null;
                _phase = 3;
                return;
            }

            if (_phase == 3)
            {
                var vegetation=prepared.Vegetation;
                if (_treeIndex >= vegetation.Count) { Finish(); return; }
                if (_treeIndex == 0)
                {
                    _vegetationClock.Restart();
                    _treePrefabs = NativeVegetationPlacement.ResolveCatalog(EntityManager, _prefabSystem,s_Log);
                    if (_treePrefabs.Length == 0) { s_Log?.Warn("Optional vegetation skipped: no verified stock trees or bushes are available."); s_Log?.Count("Vegetation skipped", vegetation.Count); Finish(); return; }
                }
                Progress?.Invoke($"Planting trees and bushes… {100L * _treeIndex / Math.Max(1, vegetation.Count)}% of locations processed.");
                var count = Math.Min(512, vegetation.Count - _treeIndex);
                var failuresBefore=s_Log!.Metric("Vegetation definition failures");
                if(!NetworkCommitSystem.CanStartExclusive(EntityManager,out var vegetationPreviewReason))
                {
                    if(++_vegetationRetirementFrames==1||_vegetationRetirementFrames%30==0)
                        s_Log.Diagnostic($"VEGETATION RETIREMENT pendingUpdates={_vegetationRetirementFrames}; candidateIndex={_treeIndex}; {vegetationPreviewReason}; definitionsNotResubmitted=true");
                    if(_vegetationRetirementFrames>=180)throw new InvalidOperationException("Vegetation preview retirement exceeded bounded wait: "+vegetationPreviewReason);
                    return;
                }
                _vegetationRetirementFrames=0;
                s_Log.Checkpoint("vegetation/definitions",new { startIndex=_treeIndex, count, total=vegetation.Count, mapChanged=_mapChanged });
                _definitions = NativeVegetationPlacement.CreateBatch(EntityManager, vegetation, _treeIndex, count, _treePrefabs, s_Log!);
                _treeIndex += count;
                if (count >= 32 && s_Log.Metric("Vegetation definition failures")-failuresBefore > count / 2)
                {
                    foreach (var definition in _definitions) EntityManager.DestroyEntity(definition);
                    s_Log?.Count("Vegetation skipped", _definitions.Count + vegetation.Count - _treeIndex);
                    s_Log?.Warn("Optional vegetation stopped because most candidates in a batch failed placement. Terrain and networks remain imported.");
                    _definitions=null;
                    Finish(); return;
                }
                _phase = _definitions.Count == 0 ? 3 : 4;
                return;
            }
            if (_phase == 4)
            {
                EntityManager.CompleteAllTrackedJobs();
                foreach (var definition in _definitions!) EntityManager.DestroyEntity(definition);
                NetworkCommitSystem.RequestCommit(prepared.Request.MapSessionId, prepared.Request.Generation,s_Log,new { kind="vegetation", candidatesProcessed=_treeIndex, definitions=_definitions.Count }); _phase = 5;
                return;
            }
            if (_phase == 5 && NetworkCommitSystem.TryConsumeResult(out var treesComplete, out var treeFailure) && treesComplete)
            {
                if (treeFailure != null) throw new InvalidOperationException(treeFailure);
                s_Log?.Count("Tree definitions committed", _definitions!.Count);
                _definitions=null;
                _phase = 3;
            }
        }
        catch (Exception exception)
        {
            s_Log?.Diagnostic($"APPLICATION FAILURE phase={_phase}; networkProgress={_networkIndex}/{prepared.Segments.Count}; vegetationProgress={_treeIndex}/{prepared.Vegetation.Count}; mapChanged={_mapChanged}; exception={exception}");
            s_Log?.Checkpoint("apply/managed-failure",new { phase=_phase, networkProgress=_networkIndex, vegetationProgress=_treeIndex, mapChanged=_mapChanged, exception=exception.ToString() });
            var message = exception.GetBaseException().Message;
            if (_phase >= 3 && _phase <= 5)
            {
                s_Log?.Warn($"Optional vegetation application stopped: {message}. Terrain and networks remain imported.");
                s_Log?.Count("Vegetation skipped", Math.Max(0, prepared.Vegetation.Count - _treeIndex));
                s_Log?.Count("Unconfirmed tree definitions", _definitions?.Count ?? 0);
                CleanupOwnedPreview(prepared);
                _definitions=null;
                Finish(); return;
            }
            if(_mapChanged)
            {
                s_Log?.Checkpoint("apply/partial-outcome",new{verifiedSourceSections=_networkPlacement?.VerifiedSections??0,requestedSourceSections=prepared.Segments.Count,
                    queuedProgressNotVerification=_networkIndex,phase=_phase,rejectedArtifact=_networkPlacement?.ArtifactStem,
                    rollbackAvailable=false,automaticApplyRetry=false,completionReported=false});
                message += $" Replacement had already started; {_networkPlacement?.VerifiedSections??0}/{prepared.Segments.Count} source sections were verified before this failure. The map is partial, not complete. Reload the original saved map before retrying. Do not save this partial import.";
            }
            s_Log?.Error(message);
            Failed?.Invoke(message);
            AbortMapWork();
        }
    }

    private object NetworkBatchContext(PreparedImport prepared,int startIndex) => new
    {
        kind="network", startIndex, count=_networkBatch.Count, total=prepared.Segments.Count,
        recoveryAttempt=_recoveryAttempts,verifiedPrefix=startIndex,artifactStem=_networkPlacement?.ArtifactStem,
        batchSizeLimit=_batchSizeLimit,recoveryUntilIndex=_recoveryUntilIndex,
        snapshot="native-submission-order.ndjson", originalSnapshot="native-plan-network.ndjson", mapChanged=_mapChanged,
        sourceEndpoints=System.Linq.Enumerable.ToArray(System.Linq.Enumerable.Select(_networkBatch,s=>new { kind=s.Kind.ToString(),s.WayId,s.StartNodeId,s.EndNodeId }))
    };

    private bool PrepareRecovery(PreparedImport prepared,string failure,bool recoverable,bool preflight)
    {
        if(!recoverable||_recoveryAttempts>=NativePreviewRecovery.MaximumAttempts)return false;
        var start=preflight?0:_networkIndex-_networkBatch.Count;
        var details=NetworkCommitSystem.PreviewFailure;
        var gathered=preflight?0:_submissionPlan!.GatherPendingJunction(start,_networkBatch.Count,NetworkBatchSize,details?.Sources??Array.Empty<string>());
        var retryCount=gathered>0?gathered:NativePreviewRecovery.NextBatchSize(_networkBatch.Count,_recoveryAttempts,applied:false,recoverable:true);
        if(gathered==0&&NetworkCommitSystem.LastMutationTargets>64&&_networkBatch.Count>1)
            retryCount=Math.Max(1,Math.Min(_networkBatch.Count-1,_networkBatch.Count*48/NetworkCommitSystem.LastMutationTargets));
        if(retryCount==0)return false;
        _recoveryUntilIndex=Math.Max(_recoveryUntilIndex,start+_networkBatch.Count);
        _batchSizeLimit=retryCount;
        if(!preflight)_networkIndex-=_networkBatch.Count;
        if(preflight)
        {
            // Partition the whole selected risk cluster; never silently skip its tail.
            var original=_networkBatch;
            _networkBatch=System.Linq.Enumerable.ToArray(System.Linq.Enumerable.Take(original,retryCount));
            _preflightBatches[_preflightIndex]=_networkBatch;
            if(retryCount<original.Count)_preflightBatches.Insert(_preflightIndex+1,System.Linq.Enumerable.ToArray(System.Linq.Enumerable.Skip(original,retryCount)));
        }
        else _networkBatch=System.Linq.Enumerable.ToArray(System.Linq.Enumerable.Take(System.Linq.Enumerable.Skip(_submissionPlan!.Sections,start),retryCount));
        if(_attemptedInputs.Contains(InputSignature(_networkBatch)))
        {
            if(!preflight&&retryCount>1)
            {_networkBatch=new[]{_submissionPlan!.Sections[start]};retryCount=1;_batchSizeLimit=1;}
            if(_attemptedInputs.Contains(InputSignature(_networkBatch)))
            {s_Log?.Diagnostic($"NATIVE RECOVERY no novel input remains index={start}; way={details?.WayId}; sources={string.Join(",",details?.Sources??Array.Empty<string>())}; identicalPreviewNotRetried=true; applied=false");return false;}
        }
        _recoveryAttempts++;_recoveryWaitFrames=0;
        var strategy=gathered>0?"pending-source-junction-arms":"smaller-peer-context";
        s_Log?.Warn($"Native preview rejected before commit; regenerating changed input, not reporting repair success: strategy={strategy}, attempt={_recoveryAttempts}, index={start}, sections={retryCount}. {failure}");
        s_Log?.DiagnosticRecords("native-submission-order.ndjson",DiagnosticArtifacts.NetworkRecords(_submissionPlan!.Sections));
        s_Log?.Checkpoint("network/recovery-choice",new{strategy,start,retryCount,preflight,way=details?.WayId,sources=details?.Sources,geometryChanged=false,applied=false,successNotYetVerified=true});
        NetworkBatchCheckpoint("network/pre-commit-recovery",prepared,start);
        Progress?.Invoke("Regenerating the affected road/rail junction before committing…");
        _phase=preflight?12:8;return true;
    }

    private void NetworkBatchCheckpoint(string phase,PreparedImport prepared,int startIndex)
    {
        _attemptedInputs.Add(InputSignature(_networkBatch));
        _networkPlacement?.BeginBatch(startIndex,_recoveryAttempts,_networkBatch.Count,
            phase=="apply/definition-preflight"||_phase==12||phase=="network/pre-commit-recovery"&&_phase==10);
        s_Log?.Checkpoint(phase,NetworkBatchContext(prepared,startIndex));
    }

    private static string InputSignature(IReadOnlyList<PreparedSegment> sections)=>string.Join("|",System.Linq.Enumerable.Select(sections,s=>$"{s.Kind}:{s.WayId}:{s.StartNodeId}:{s.EndNodeId}:{s.CourseStart}:{s.CourseEnd}:{s.CourseControl1}:{s.CourseControl2}"));

    private void Finish()
    {
        var prepared = s_Pending;
        if (prepared != null && !ImportSession.Matches(prepared.Request.MapSessionId, prepared.Request.Generation)) { AbortMapWork(); return; }
        if (prepared != null)
        {
            NativeDiagnostics.Begin(prepared,s_Log!);
            var reports = new List<string>
            {
                $"Location: {prepared.Request.Latitude:0.######}, {prepared.Request.Longitude:0.######}; {prepared.Request.MapSize.Label}."
            };
            Progress?.Invoke("Adding river water sources…");
            s_Log?.Checkpoint("environment/water",new { plannedControls=prepared.Water?.Sources.Count??0, mapChanged=_mapChanged });
            try { reports.Add(NativeWaterBridge.Apply(prepared, EntityManager, World.GetOrCreateSystemManaged<TerrainSystem>(), World.GetOrCreateSystemManaged<WaterSystem>(), s_Log!)); }
            catch (Exception exception) { s_Log?.Diagnostic("WATER FAILURE: " + exception); reports.Add("River water stopped safely: " + exception.GetBaseException().Message); }
            if (prepared.Request.ApplyGroundSurfaces)
            {
                Progress?.Invoke("Applying ground surfaces…");
                try
                {
                    s_Log?.Checkpoint("environment/surface",new { maskAvailable=prepared.SurfaceMask!=null, mapChanged=_mapChanged });
                    reports.Add(prepared.SurfaceMask == null ? "Ground surfaces skipped: classification unavailable." :
                        NativeSurfaceBridge.Apply(prepared.SurfaceMask, EntityManager, _prefabSystem, World.GetOrCreateSystemManaged<Game.Rendering.TerrainMaterialSystem>(), World.GetOrCreateSystemManaged<TerrainSystem>(), s_Log!));
                }
                catch (Exception exception) { s_Log?.Diagnostic("SURFACE FAILURE: " + exception); reports.Add("Ground surfaces stopped safely: " + exception.GetBaseException().Message); }
            }
            else reports.Add("Ground surfaces: not requested.");
            if (prepared.Request.CreateAirConnections)
            {
                Progress?.Invoke("Creating air connections…");
                s_Log?.Checkpoint("environment/air",new { mapChanged=_mapChanged });
                try { reports.Add(NativeAirBridge.Apply(prepared, EntityManager, _prefabSystem, World.GetOrCreateSystemManaged<Game.Net.AirwaySystem>(), World.GetOrCreateSystemManaged<Game.UI.NameSystem>(), s_Log!)); }
                catch (Exception exception) { s_Log?.Diagnostic("AIR FAILURE: " + exception); reports.Add("Air connections skipped: " + exception.GetBaseException().Message); }
            }
            else reports.Add("Air connections: not requested.");
            if(prepared.Request.CreateSeaConnections)
            {
                try { reports.Add(NativeSeaBridge.Assess(prepared,EntityManager,_prefabSystem,s_Log!)); }
                catch(Exception exception) { s_Log?.Diagnostic("SEA ASSESSMENT FAILURE: "+exception); reports.Add("Sea assessment skipped safely; no shipping entities changed."); }
            }
            Progress?.Invoke("Setting the map's climate…");
            s_Log?.Checkpoint("environment/climate",new { available=prepared.Climate!=null, mapChanged=_mapChanged });
            try { reports.Add(NativeClimateBridge.Apply(prepared, EntityManager, _prefabSystem, World.GetOrCreateSystemManaged<ClimateSystem>(), World.GetOrCreateSystemManaged<WindSimulationSystem>(), s_Log!)); }
            catch (Exception exception) { NativeDiagnostics.ClimateApplied=false; s_Log?.Diagnostic("CLIMATE FAILURE: " + exception); reports.Add("Climate assignment failed; existing/default preserved: " + exception.GetBaseException().Message); }
            prepared.FeatureReport = string.Join("\n", reports);
            foreach (var report in reports)
                if (report.IndexOf("skipped", StringComparison.OrdinalIgnoreCase) >= 0 || report.IndexOf("failed", StringComparison.OrdinalIgnoreCase) >= 0 || report.IndexOf("stopped", StringComparison.OrdinalIgnoreCase) >= 0) s_Log?.Warn(report);
                else s_Log?.Info(report);
            foreach(var report in reports)s_Log?.Diagnostic("FEATURE RESULT " + report);
            NativeDiagnostics.Queue("after-apply",s_Log!);
        }
        if (_vegetationClock.IsRunning) { _vegetationClock.Stop(); s_Log?.Time("Vegetation apply", _vegetationClock.Elapsed.TotalSeconds); }
        s_Log?.Info($"Import complete. Native definitions: {_created}; skipped: {_skipped}.");
        s_Log?.Checkpoint("apply/completed",new { definitionsSubmitted=_created, skipped=_skipped, vegetationCandidatesProcessed=_treeIndex, delayedReadbackPending=true, nativeRenderingNotVerified=true });
        Completed?.Invoke(_created, _skipped);
        Reset();
    }

    private void Reset()
    {
        lock (Gate) { s_Pending = null; s_Log = null; }
        _definitions = null;
        _networkPlacement=null;_submissionPlan=null;_networkIndex=0;_networkBatch=Array.Empty<PreparedSegment>();
        _oldNetworks=Array.Empty<Entity>();_retirementFrames=0;_readbackFrames=0;_readbackStartFrame=0;_mapChanged=false;
        _recoveryAttempts=0;_recoveryWaitFrames=0;
        _preflightBatches.Clear();_preflightIndex=0;_attemptedInputs.Clear();
        _batchSizeLimit=NetworkBatchSize;_recoveryUntilIndex=0;
        _preferredBatchSize=NetworkBatchSize;
        _phase = 0;
        _created = 0;
        _skipped = 0;
        _treeIndex = 0; _vegetationRetirementFrames=0; _treePrefabs = Array.Empty<NativeVegetationPlacement.StockPrefab>(); _vegetationClock.Reset();
        _roadSections = _railSections = 0;
    }

    protected override void OnGamePreload(Colossal.Serialization.Entities.Purpose purpose, GameMode mode)
    { base.OnGamePreload(purpose, mode); AbortMapWork(); }

    protected override void OnDestroy()
    { if(_saveManager!=null)_saveManager.onGameSaveLoad-=OnSaveDiagnostic; AbortMapWork(); if(s_Instance==this)s_Instance=null; base.OnDestroy(); }

    private void OnSaveDiagnostic(string saveName,string previewUri,bool start,bool success)
    {
        if(GameManager.instance.gameMode!=GameMode.Editor)return;
        (s_Log??new ImportLog()).Diagnostic($"SAVE name={saveName}; start={start}; success={success}; mapSession={ImportSession.Map}; persistenceStillRequiresReloadComparison=true");
    }

    public static void AbortPending() { if(s_Instance!=null)s_Instance.AbortMapWork(); else { lock(Gate){s_Pending=null;s_Log=null;} NetworkCommitSystem.CancelPending(); } }

    public void AbortMapWork()
    {
        // Clear static ownership FIRST. Only touch our recorded helper definitions; never
        // query/delete trees or networks belonging to a newly loaded map.
        var definitions = _definitions; var log = s_Log;
        if(s_Pending!=null)CleanupOwnedPreview(s_Pending);
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

    private void CleanupOwnedPreview(PreparedImport prepared)
    {
        if(!ImportSession.Matches(prepared.Request.MapSessionId,prepared.Request.Generation)||GameManager.instance.isGameLoading)return;
        // Exclusive preview ownership was established before creating any definitions.
        if(_definitions==null && _phase!=2 && _phase!=5 && _phase!=10 && _phase!=8 && _phase!=12)return;
        World.GetOrCreateSystemManaged<NetworkCommitSystem>().CancelOwnedPreview(s_Log);
    }
}
}
