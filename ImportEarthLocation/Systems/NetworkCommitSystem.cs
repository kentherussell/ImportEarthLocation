using System;
using System.Collections.Generic;
using System.Reflection;
using Game;
using Game.Common;
using Game.Net;
using Game.Tools;
using Unity.Entities;
using Unity.Collections;
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
    private static bool s_Recoverable;
    private static string? s_ArtifactStem;
    private static long s_Map, s_Generation;
    private static readonly HashSet<Entity> s_Submitted = new();
    private static bool s_Applied;
    private static bool s_PreviewOnly, s_ClearingPreview;
    private static NativePreviewException? s_PreviewFailure;
    private static int s_SettledFrames;
    private static int s_CommitSequence;
    private static bool s_DeepCapture;
    public static int LastMutationTargets { get; private set; }
    private static ImportLog? s_Log;
    private static object? s_Context;
    private static Action<IReadOnlyList<Entity>>? s_BeforeApply;
    private ToolSystem _toolSystem = null!;
    private FieldInfo? _lastToolField;
    private MethodInfo? _applyModeSetter;

    public static void RequestCommit(long map, long generation, ImportLog? log = null, object? context = null, Action<IReadOnlyList<Entity>>? beforeApply = null, string? artifactStem = null, bool previewOnly = false)
    {
        if (!ImportSession.Matches(map, generation)) return;
        if(s_Map!=map||s_Generation!=generation)s_CommitSequence=0;
        s_DeepCapture=previewOnly||++s_CommitSequence%16==1;
        s_Map = map; s_Generation = generation;
        s_Requested = true;
        LastMutationTargets=0;
        s_WaitFrames = 0;
        s_Failure = null;
        s_Recoverable=false;s_ArtifactStem=artifactStem;
        s_Submitted.Clear(); s_Applied=false; s_SettledFrames=0; s_Log=log; s_Context=context;
        s_PreviewOnly=previewOnly;s_ClearingPreview=false;s_PreviewFailure=null;
        s_BeforeApply=beforeApply;
    }

    public static void CancelPending() { s_Requested = false; s_WaitFrames = 0; s_Failure = null; s_Map = s_Generation = 0; s_Submitted.Clear(); s_Applied=false; s_SettledFrames=0; s_Log=null; s_Context=null; s_BeforeApply=null;s_Recoverable=false;s_ArtifactStem=null;s_PreviewOnly=s_ClearingPreview=false;s_PreviewFailure=null; }

    public static NativePreviewException? PreviewFailure => s_PreviewFailure;
    // An issued Apply must settle before a user stop touches remaining previews.
    public static bool ApplicationInFlight => s_Requested && s_Applied;
    public static bool HasIssuedApply => s_Applied;

    protected override void OnGamePreload(Colossal.Serialization.Entities.Purpose purpose, GameMode mode)
    { base.OnGamePreload(purpose, mode); CancelPending(); }
    protected override void OnDestroy() { CancelPending(); base.OnDestroy(); }

    public static bool TryConsumeResult(out bool complete, out string? failure)
        => TryConsumeResult(out complete,out failure,out _);

    public static bool TryConsumeResult(out bool complete, out string? failure,out bool recoverable)
    {
        complete = !s_Requested && (s_WaitFrames < 0 || s_Failure != null);
        failure = s_Failure;
        recoverable=s_Recoverable&&!s_Applied;
        if (!complete) return false;
        s_WaitFrames = 0;
        s_Failure = null;
        s_Recoverable=false;
        s_Submitted.Clear(); s_Log=null; s_Context=null; s_BeforeApply=null;
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

    public static bool CanStartExclusive(EntityManager manager,out string reason)
    {
        // ApplyMode is global to the native tool output pipeline. Never submit
        // an import while unrelated editing previews could be committed with it.
        manager.CompleteAllTrackedJobs();
        using var query=manager.CreateEntityQuery(new EntityQueryDesc
        {All=new[]{ComponentType.ReadOnly<Temp>()},None=new[]{ComponentType.ReadOnly<Deleted>()}});
        using var entities=query.ToEntityArray(Allocator.Temp);
        var count=0;
        foreach(var entity in entities)
        {
            var temp=manager.GetComponentData<Temp>(entity);
            // Selection-only bookkeeping is not a placement preview. Native
            // selection markers can remain indefinitely after a committed batch.
            // Never relax this for a geometry/object/lane or an editing flag.
            var selectionOnly=(temp.m_Flags&TempFlags.Select)!=0&&
                (temp.m_Flags&~(TempFlags.Select|TempFlags.Essential))==0&&
                temp.m_Original!=Entity.Null&&manager.Exists(temp.m_Original)&&
                !manager.HasComponent<Edge>(entity)&&!manager.HasComponent<Node>(entity)&&
                !manager.HasComponent<Lane>(entity)&&!manager.HasComponent<Game.Objects.Tree>(entity)&&
                !manager.HasComponent<Game.Objects.Plant>(entity)&&!manager.HasComponent<CreationDefinition>(entity)&&
                (!manager.HasComponent<Owner>(entity)||manager.GetComponentData<Owner>(entity).m_Owner==Entity.Null);
            if(!selectionOnly)count++;
        }
        reason=count==0?"":$"Editor-tool/native preview retirement pending ({count} temporary entities still present). Finish or cancel an unrelated editing preview before importing.";
        return count==0;
    }

    public static void LogPendingRetirement(EntityManager manager,ImportLog? log,string stem,int frames)
    {
        using var query=manager.CreateEntityQuery(new EntityQueryDesc
        {All=new[]{ComponentType.ReadOnly<Temp>()},None=new[]{ComponentType.ReadOnly<Deleted>()}});
        using var entities=query.ToEntityArray(Allocator.Temp);
        var records=new List<object>();var laneCount=0;
        foreach(var entity in entities)
        {
            if(manager.HasComponent<Lane>(entity))laneCount++;
            if(records.Count>=256)continue;
            var temp=manager.GetComponentData<Temp>(entity);
            var owner=manager.HasComponent<Owner>(entity)?manager.GetComponentData<Owner>(entity).m_Owner:Entity.Null;
            using var types=manager.GetComponentTypes(entity,Allocator.Temp);
            var components=new List<string>();foreach(var type in types)components.Add(type.ToString());
            records.Add(new{entity=$"{entity.Index}:{entity.Version}",original=$"{temp.m_Original.Index}:{temp.m_Original.Version}",
                owner=$"{owner.Index}:{owner.Version}",components,flags=temp.m_Flags.ToString(),lane=manager.HasComponent<Lane>(entity),
                edge=manager.HasComponent<Edge>(entity),node=manager.HasComponent<Node>(entity),frames,deletedEntitiesExcluded=true});
        }
        log?.Diagnostic($"NATIVE DOWNSTREAM RETIREMENT pendingTemps={entities.Length}; pendingLanes={laneCount}; recorded={records.Count}; truncated={entities.Length>records.Count}; frames={frames}; readOnly=true; clearingUnrelatedPreview=false");
        log?.DiagnosticRecords(stem+"-pending-retirement.ndjson",records);
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
            Any = new[] { ComponentType.ReadOnly<Edge>(), ComponentType.ReadOnly<Node>(), ComponentType.ReadOnly<Game.Objects.Tree>(), ComponentType.ReadOnly<Game.Objects.Plant>() },
            None = new[] { ComponentType.ReadOnly<Deleted>() }
        });
        using var temporary=query.ToEntityArray(Allocator.Temp);
        if(s_Applied||s_ClearingPreview)
        {
            // Setting ApplyMode only requests native application. Wait until the
            // exact submitted Temp entities retire; do not mistake it for commit.
            var remaining=0;
            foreach(var entity in temporary)if(s_Submitted.Contains(entity))remaining++;
            s_SettledFrames=remaining==0?s_SettledFrames+1:0;
            if(s_SettledFrames>=2)
            {s_Log?.Diagnostic($"NATIVE {(s_ClearingPreview?"PREFLIGHT":"COMMIT")} settled submitted={s_Submitted.Count}; pendingTemps=0; frames={s_WaitFrames}; definitionCountNotAcceptance=true");s_Log?.Checkpoint(s_ClearingPreview?"native-preflight/cleared":"native-commit/settled",new { context=s_Context, submitted=s_Submitted.Count, frames=s_WaitFrames, applied=s_Applied, liveRenderingNotVerified=true });s_Requested=false;s_WaitFrames=-1;return;}
            if(++s_WaitFrames<180)return;
            s_Failure=$"Native {(s_ClearingPreview?"preview clearing":"application")} did not settle: {remaining} of {s_Submitted.Count} submitted temporary entities remain. No duplicate retry was attempted.";
            s_Log?.Diagnostic("NATIVE COMMIT timeout "+s_Failure);s_Requested=false;return;
        }
        if (temporary.Length == 0)
        {
            if (++s_WaitFrames <= 32) return;
            s_Failure = "CS2 did not generate temporary network/vegetation entities from the prepared native definitions.";
            s_Requested = false;
            return;
        }

        try
        {
            var lastTool = _lastToolField?.GetValue(_toolSystem) as ToolBaseSystem;
            if (lastTool == null || _applyModeSetter == null)
                throw new InvalidOperationException("The native tool commit hook was unavailable.");
            // Native ECB faults are outside managed exception recovery. Reject an
            // oversized or malformed temporary graph BEFORE setting ApplyMode.
            if(temporary.Length>8192)s_Log?.Warn($"Large native preview: {temporary.Length} temporary entities; diagnostic only, not a generation gate.");
            // One batch-local graph instead of two identical serialized copies.
            // Vegetation has no road graph to diagnose; retain sampled captures.
            // Retain the actual last object batch too, not just every sixteenth
            // batch: native faults cannot be reconstructed after process exit.
            var batchArtifact=(s_ArtifactStem??"last-native-temp-batch")+(s_ArtifactStem==null?".ndjson":"-temp.ndjson");
            if(s_Log?.DiagnosticsEnabled==true)s_Log.DiagnosticRecords(batchArtifact,TempRecords(EntityManager,temporary.ToArray(),World.GetExistingSystemManaged<Game.Prefabs.PrefabSystem>()));
            var originalTargets=new HashSet<Entity>();var mutationTargets=new HashSet<Entity>();var originalMutations=0;
            foreach(var entity in temporary)
            {
                var temp=EntityManager.GetComponentData<Temp>(entity);
                if(temp.m_Original==Entity.Null)continue;
                if(mutationTargets.Add(temp.m_Original))originalMutations++;
                // A delete companion to a qualified replacement is allowed;
                // CaptureGeneratedNodes proves its replacement coverage.
                if((temp.m_Flags&TempFlags.Delete)!=0)continue;
                if(!originalTargets.Add(temp.m_Original))
                    s_Log?.Warn("Native preview contains duplicate original mutation targets; native companion updates retained for inspection.");
                if(EntityManager.HasComponent<Deleted>(temp.m_Original)||
                    (EntityManager.HasComponent<Node>(entity)&&!EntityManager.HasComponent<Node>(temp.m_Original))||
                    (EntityManager.HasComponent<Edge>(entity)&&!EntityManager.HasComponent<Edge>(temp.m_Original)))
                    s_Log?.Warn("Native preview original has retired or changed component kind; native lineage discrepancy recorded.");
            }
            s_Log?.Diagnostic($"NATIVE APPLY WORKLOAD temporaryEntities={temporary.Length}; originalMutationTargets={originalMutations}; originalMutationLimit=disabled; rejectedBeforeApply=false; downstreamLaneAcceptanceUnverified=true");
            LastMutationTargets=originalMutations;
            // Existing graph matching proves ports/ownership, not downstream
            // native ECB capacity. Shrink an uncommitted regeneration burst,
            // including flags=0 updates, rather than Apply then retry it.
            if(originalMutations>64)s_Log?.Count("Native batches above former mutation guard");
            if(s_ArtifactStem!=null)s_Log?.DiagnosticRecords(s_ArtifactStem+"-mutation-contracts.ndjson",MutationRecords(EntityManager,temporary.ToArray()));
            if(s_ArtifactStem!=null)
            {
                using var allPreviewQuery=EntityManager.CreateEntityQuery(ComponentType.ReadOnly<Temp>());
                using var allPreview=allPreviewQuery.ToEntityArray(Allocator.Temp);
                if(s_DeepCapture)s_Log?.DiagnosticRecords(s_ArtifactStem+"-downstream-contracts.ndjson",MutationRecords(EntityManager,allPreview.ToArray()),32L*1024*1024);
                s_Log?.Diagnostic($"NATIVE DOWNSTREAM CAPTURE artifact={s_ArtifactStem}; allTempsIncludingDeletedAndLanes={allPreview.Length}; deepCapture={s_DeepCapture}; samplingEveryBatches=16; frame={UnityEngine.Time.frameCount}; readOnly=true; nativeCommandBufferContentsUnavailable=true");
                AuditLaneContracts(EntityManager,allPreview.ToArray(),temporary.ToArray(),s_Log);
            }
            var invalidNativeGraph=false;
            foreach(var entity in temporary)
            {
                if(!EntityManager.HasComponent<Edge>(entity))continue;
                if((EntityManager.GetComponentData<Temp>(entity).m_Flags&TempFlags.Delete)!=0)continue;
                var edge=EntityManager.GetComponentData<Edge>(entity);
                if(edge.m_Start==Entity.Null||edge.m_End==Entity.Null||edge.m_Start==edge.m_End||
                    !EntityManager.HasComponent<Node>(edge.m_Start)||!EntityManager.HasComponent<Node>(edge.m_End))
                {invalidNativeGraph=true;s_Log?.Warn($"Native temporary edge {entity} has incomplete node references; this native preview will be cleared, not applied to an invalid engine graph.");}
                if(EntityManager.HasComponent<Curve>(entity))
                {
                    var curve=EntityManager.GetComponentData<Curve>(entity).m_Bezier;
                    if(!Unity.Mathematics.math.all(Unity.Mathematics.math.isfinite(curve.a))||!Unity.Mathematics.math.all(Unity.Mathematics.math.isfinite(curve.b))||
                       !Unity.Mathematics.math.all(Unity.Mathematics.math.isfinite(curve.c))||!Unity.Mathematics.math.all(Unity.Mathematics.math.isfinite(curve.d)))
                    {invalidNativeGraph=true;s_Log?.Warn($"Native temporary edge {entity} has a non-finite curve; native preview defect recorded.");}
                }
            }
            if(invalidNativeGraph)
            {
                if(s_ArtifactStem!=null)s_Log?.DiagnosticRecords(s_ArtifactStem+"-invalid-graph.ndjson",MutationRecords(EntityManager,temporary.ToArray()));
                s_Failure="Native preview contained invalid engine references or non-finite coordinates. Preview cleared locally; subsequent import batches remain eligible.";
                s_BeforeApply=null;foreach(var entity in temporary)s_Submitted.Add(entity);
                _applyModeSetter.Invoke(lastTool,new object[]{ApplyMode.Clear});
                s_ClearingPreview=true;s_WaitFrames=0;return;
            }
            s_BeforeApply?.Invoke(temporary.ToArray());
            s_BeforeApply=null;
            foreach(var entity in temporary)s_Submitted.Add(entity);
            if(s_PreviewOnly)
            {
                s_Log?.Checkpoint("native-preflight/validated",new { context=s_Context, temporaryEntities=s_Submitted.Count, applied=false, wholeMapVerified=false });
                _applyModeSetter.Invoke(lastTool,new object[]{ApplyMode.Clear});
                s_ClearingPreview=true;s_WaitFrames=0;return;
            }
            s_Log?.Checkpoint("native-commit/before-apply",new { context=s_Context, mapSession=s_Map, generation=s_Generation, temporaryEntities=s_Submitted.Count, frame=UnityEngine.Time.frameCount, lastTool=lastTool.GetType().FullName, managedMemoryBytes=GC.GetTotalMemory(false), graph=batchArtifact });
            _applyModeSetter.Invoke(lastTool, new object[] { ApplyMode.Apply });
            s_Applied=true;s_WaitFrames=0;
            s_Log?.Diagnostic($"NATIVE COMMIT requested temporaryEntities={s_Submitted.Count}; waitingForRetirement=true; includesBushPlants=true");
        }
        catch (Exception exception)
        {
            try { if(s_ArtifactStem!=null)s_Log?.DiagnosticRecords(s_ArtifactStem+"-failure-contracts.ndjson",MutationRecords(EntityManager,temporary.ToArray()),32L*1024*1024); }
            catch(Exception captureError){s_Log?.Warn("Failure contract capture unavailable: "+captureError.Message);}
            // Recovery is limited to typed graph rejection BEFORE Apply. Native
            // hook/playback failures are never blindly retried or mislabeled.
            s_Recoverable=!s_Applied&&exception is NativePreviewException;
            s_PreviewFailure=exception as NativePreviewException;
            s_Log?.Diagnostic($"NATIVE COMMIT FAILURE context={Newtonsoft.Json.JsonConvert.SerializeObject(s_Context)}; exception={exception}");
            s_Log?.Checkpoint("native-commit/managed-failure",new { context=s_Context, exception=exception.ToString(), submitted=s_Submitted.Count, recoverable=s_Recoverable, applied=s_Applied,artifactStem=s_ArtifactStem });
            CancelOwnedPreview(s_Log);
            s_Failure = $"Native network commit failed: {exception.GetBaseException().Message}";
            s_Requested = false;
        }
    }

    private static IEnumerable<object> TempRecords(EntityManager manager,IReadOnlyList<Entity> entities,Game.Prefabs.PrefabSystem? prefabs)
    {
        string Id(Entity e)=>$"{e.Index}:{e.Version}";
        foreach(var entity in entities)
        {
            object? edgeData=null,curveData=null,nodeData=null;
            var temp=manager.GetComponentData<Temp>(entity);
            var prefab=manager.HasComponent<Game.Prefabs.PrefabRef>(entity)?manager.GetComponentData<Game.Prefabs.PrefabRef>(entity).m_Prefab:Entity.Null;
            string? prefabName=null;
            if(prefab!=Entity.Null&&prefabs!=null){try{prefabName=prefabs.GetPrefabName(prefab);}catch{prefabName="unavailable";}}
            if(manager.HasComponent<Edge>(entity))
            {
                var edge=manager.GetComponentData<Edge>(entity);
                edgeData=new { start=Id(edge.m_Start),end=Id(edge.m_End), startPosition=manager.HasComponent<Node>(edge.m_Start)?DiagnosticArtifacts.Point(manager.GetComponentData<Node>(edge.m_Start).m_Position):null,
                    endPosition=manager.HasComponent<Node>(edge.m_End)?DiagnosticArtifacts.Point(manager.GetComponentData<Node>(edge.m_End).m_Position):null };
            }
            if(manager.HasComponent<Curve>(entity))
            {var c=manager.GetComponentData<Curve>(entity).m_Bezier;curveData=new { a=DiagnosticArtifacts.Point(c.a),b=DiagnosticArtifacts.Point(c.b),c=DiagnosticArtifacts.Point(c.c),d=DiagnosticArtifacts.Point(c.d) };}
            if(manager.HasComponent<Node>(entity))nodeData=DiagnosticArtifacts.Point(manager.GetComponentData<Node>(entity).m_Position);
            yield return new { entity=Id(entity),edge=edgeData,curve=curveData,nodePosition=nodeData,tree=manager.HasComponent<Game.Objects.Tree>(entity),plant=manager.HasComponent<Game.Objects.Plant>(entity),
                prefab=prefab==Entity.Null?null:Id(prefab),prefabName,original=Id(temp.m_Original),flags=temp.m_Flags.ToString(),tempCurvePosition=temp.m_CurvePosition };
        }
    }

    private static void AuditLaneContracts(EntityManager manager,IReadOnlyList<Entity> allPreview,IReadOnlyList<Entity> networks,ImportLog? log)
    {
        // Installed LaneSystem reads Lane/PrefabRef for each nonsecondary
        // SubLane. Reject broken references before requesting native playback.
        void RequireLane(Entity lane,string context)
        {
            if(manager.HasComponent<SecondaryLane>(lane))return;
            if(!manager.Exists(lane)||!manager.HasComponent<Lane>(lane)||!manager.HasComponent<Game.Prefabs.PrefabRef>(lane))
                log?.Warn($"Native lane contract incomplete ({context}, lane={lane}); advisory only.");
        }
        foreach(var preview in allPreview)
        {
            if(!manager.HasComponent<Lane>(preview))continue;
            var temp=manager.GetComponentData<Temp>(preview);
            if(manager.HasComponent<Deleted>(preview)||(temp.m_Flags&TempFlags.Cancel)!=0)continue;
            RequireLane(preview,"preview");
            if(temp.m_Original!=Entity.Null)RequireLane(temp.m_Original,"original");
            if(!manager.HasComponent<Owner>(preview)){log?.Warn("Native lane has no owner contract; advisory only.");continue;}
            var owner=manager.GetComponentData<Owner>(preview).m_Owner;
            if(!manager.Exists(owner))log?.Warn($"Native lane owner {owner} is unavailable; advisory only.");
        }
        var visited=new HashSet<Entity>();
        foreach(var preview in networks)
        foreach(var owner in new[]{preview,manager.GetComponentData<Temp>(preview).m_Original})
        {
            if(manager.HasComponent<Deleted>(preview)||(manager.GetComponentData<Temp>(preview).m_Flags&TempFlags.Cancel)!=0)continue;
            if(owner==Entity.Null||!visited.Add(owner)||!manager.HasBuffer<SubLane>(owner))continue;
            foreach(var sub in manager.GetBuffer<SubLane>(owner,true))RequireLane(sub.m_SubLane,"owner-buffer");
        }
    }

    private static IEnumerable<object> MutationRecords(EntityManager manager,IReadOnlyList<Entity> temporary)
    {
        string Id(Entity e)=>$"{e.Index}:{e.Version}";
        var seen=new HashSet<Entity>();
        foreach(var preview in temporary)
        foreach(var entity in new[]{preview,manager.GetComponentData<Temp>(preview).m_Original})
        {
            if(entity==Entity.Null||!seen.Add(entity))continue;
            if(!manager.Exists(entity))
            {yield return new{entity=Id(entity),preview=Id(preview),exists=false,frame=UnityEngine.Time.frameCount};continue;}
            var connected=new List<string>();var lanes=new List<object>();
            if(manager.HasBuffer<ConnectedEdge>(entity))
                foreach(var edge in manager.GetBuffer<ConnectedEdge>(entity,true))connected.Add(Id(edge.m_Edge));
            if(manager.HasBuffer<SubLane>(entity))
                foreach(var lane in manager.GetBuffer<SubLane>(entity,true))
                    lanes.Add(new{entity=Id(lane.m_SubLane),hasLane=manager.HasComponent<Lane>(lane.m_SubLane),
                        temporary=manager.HasComponent<Temp>(lane.m_SubLane),deleted=manager.HasComponent<Deleted>(lane.m_SubLane)});
            using var types=manager.GetComponentTypes(entity,Allocator.Temp);
            var components=new List<string>();foreach(var type in types)components.Add(type.ToString());
            object? composition=null;
            if(manager.HasComponent<Composition>(entity))
            {var c=manager.GetComponentData<Composition>(entity);composition=new{edge=Id(c.m_Edge),start=Id(c.m_StartNode),end=Id(c.m_EndNode)};}
            yield return new{entity=Id(entity),preview=Id(preview),components,connectedEdges=connected,subLanes=lanes,composition,
                exists=true,owner=manager.HasComponent<Owner>(entity)?Id(manager.GetComponentData<Owner>(entity).m_Owner):null,
                temp=manager.HasComponent<Temp>(entity)?(object)manager.GetComponentData<Temp>(entity):null,
                lane=manager.HasComponent<Lane>(entity)?(object)manager.GetComponentData<Lane>(entity):null,
                deleted=manager.HasComponent<Deleted>(entity),frame=UnityEngine.Time.frameCount,downstreamLanePlaybackNotProven=true};
        }
    }

    public void CancelOwnedPreview(ImportLog? log)
    {
        if(Game.SceneFlow.GameManager.instance.isGameLoading)return;
        try
        {
            EntityManager.CompleteAllTrackedJobs();
            var lastTool=_lastToolField?.GetValue(_toolSystem) as ToolBaseSystem;
            if(lastTool!=null && _applyModeSetter!=null)
            {
                _applyModeSetter.Invoke(lastTool,new object[]{ApplyMode.Clear});
                log?.Diagnostic("NATIVE PREVIEW cancelled through native Clear; no failed batch was re-applied");
            }
        }
        catch(Exception error){log?.Warn("Native preview cancellation failed: "+error.GetBaseException().Message);}
    }
}
}
