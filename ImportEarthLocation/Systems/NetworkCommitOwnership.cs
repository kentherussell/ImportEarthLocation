using System;
using System.Collections.Generic;
using Game.Common;
using Game.Net;
using Game.Prefabs;
using Game.Tools;
using ImportEarthLocation.Core;
using Unity.Collections;
using Unity.Entities;

namespace ImportEarthLocation.Systems
{

public enum NativeTransactionState
{
    Idle, LeaseAcquired, DefinitionsLive, PreviewGenerated, ApplyIssued,
    AwaitingRetirement, Settled, ClearIssued, AwaitingClearRetirement, Cleared,
    SettlementOverdue, AwaitingExclusiveSafety, PreviewLostUnissued,
    MapRetiring, MapRetired, UnresolvedMapRetirement, WorldRetired
}

// The managed partial belongs to the existing commit system. No new game system,
// tool, asynchronous service or UI interaction is introduced.
public sealed partial class NetworkCommitSystem
{
    private static NetworkCommitSystem? s_Instance;
    private static ulong s_DetachedWorld;
    private static bool s_DetachedRetirement;
    private readonly HashSet<Entity> _ownedDefinitions = new();
    private int _foliageRequested=-1,_foliageDefinitions=-1;
    private readonly HashSet<Entity> _baselineTemps = new();
    private readonly HashSet<Entity> _claimedTemps = new();
    private readonly NativeEntityOwnership<Entity> _ownedEntities = new();
    private readonly NativeOwnerLineage<Entity> _ownerLineage = new();
    private readonly NativeOwnerLineage<Entity> _retiringLineage = new();
    private readonly Dictionary<Entity,(Entity Prefab,bool Tree)> _foliageCandidates=new();
    private ToolBaseSystem? _capturedActive, _capturedLast;
    private long _toolSerial, _capturedSerial;
    private ImportEpoch _leaseEpoch;
    private bool _hasLease, _generationExclusive, _allocationUnknown;
    private long _batchId, _retirementId;
    private bool _retirementCovered, _retirementNotificationSafe;
    private ImportLog? _leaseLog;
    public NativeTransactionState TransactionState { get; private set; }
    public static bool MapRetirementInProgress => s_Instance?._retirementId != null && s_Instance._retirementId != 0;

    private void ToolChanged(ToolBaseSystem tool) { _toolSerial++; RecordToolChange(tool); }

    private void AttachOwner()
    {
        if(s_DetachedRetirement)
        {
            if(s_DetachedWorld==World.SequenceNumber)
            {_retirementId=++_batchId;TransactionState=NativeTransactionState.UnresolvedMapRetirement;}
            else {s_Requested=false;s_Applied=false;s_ClearingPreview=false;CancelPending();s_DetachedRetirement=false;}
        }
        s_Instance=this;
    }
    private void DetachOwner()
    {
        s_DetachedWorld=World.SequenceNumber;s_DetachedRetirement=_hasLease||(_retirementId!=0&&(!_retirementCovered||!_retirementNotificationSafe));
        if(s_Instance==this)s_Instance=null;
    }

    public static void ReleaseEmptyDefinitions(EntityManager manager)
    {
        var owner=s_Instance;if(owner==null||!owner._hasLease||owner._allocationUnknown||s_Applied||s_ClearingPreview)return;
        foreach(var id in owner._ownedDefinitions)if(manager.Exists(id))return;
        using var query=manager.CreateEntityQuery(ComponentType.ReadOnly<Temp>());
        using var ids=query.ToEntityArray(Allocator.Temp);
        foreach(var id in ids)if(!owner._baselineTemps.Contains(id))return;
        owner._hasLease=false;owner._ownedDefinitions.Clear();owner._baselineTemps.Clear();owner._leaseLog=null;
        owner._ownedEntities.DiscardBatch();
        owner.TransactionState=NativeTransactionState.Cleared;
    }

    public static void BeginDefinitionBatch(EntityManager manager, ImportLog log, string? artifactStem = null)
    {
        var owner = s_Instance ?? manager.World.GetOrCreateSystemManaged<NetworkCommitSystem>();
        if (owner._hasLease || owner._retirementId != 0)
            throw new InvalidOperationException("The previous native lease has not retired.");
        if (!CanStartExclusive(manager, out var reason)) throw new InvalidOperationException(reason);
        using var definitions = manager.CreateEntityQuery(new EntityQueryDesc
        { All=new[]{ComponentType.ReadOnly<CreationDefinition>(),ComponentType.ReadOnly<Updated>()},None=new[]{ComponentType.ReadOnly<Deleted>()} });
        if (!definitions.IsEmptyIgnoreFilter) throw new InvalidOperationException("An unrelated native definition producer is active.");
        var last = owner._lastToolField?.GetValue(owner._toolSystem) as ToolBaseSystem;
        if (last == null)
            throw new InvalidOperationException("The engine's last tool is unavailable for native placement.");
        // ToolOutput reads this mode; it does not acknowledge/reset it. The
        // default tool may request Clear on an empty preview every frame.
        // CanStartExclusive already excluded unrelated definitions/edit Temps.
        // Do not clear foreign state or treat the mode as an unconsumed message.
        log.Diagnostic($"NATIVE TOOL READINESS active={owner._toolSystem.activeTool?.GetType().FullName}; last={last.GetType().FullName}; mode={last.applyMode}; serial={owner._toolSerial}; exclusiveEntitiesChecked=true; modeAloneBlocks=false; frame={UnityEngine.Time.frameCount}");
        if(last.applyMode!=ApplyMode.None)log.Count("Native idle tool modes accepted");
        if(owner._leaseEpoch.MapSession!=ImportSession.Map||owner._leaseEpoch.Generation!=ImportSession.Generation)
            owner._ownedEntities.Clear();
        owner._leaseEpoch = new ImportEpoch(ImportSession.Map,ImportSession.Generation,manager.World.SequenceNumber);
        s_Applied=false;s_ClearingPreview=false;s_ApplyInvocations=0;s_Submitted.Clear();
        owner._capturedActive=owner._toolSystem.activeTool;owner._capturedLast=last;owner._capturedSerial=owner._toolSerial;
        owner._ownedDefinitions.Clear();owner._baselineTemps.Clear();owner._claimedTemps.Clear();
        owner._foliageRequested=owner._foliageDefinitions=-1;
        owner._foliageCandidates.Clear();
        owner._ownedEntities.DiscardBatch();
        using var temps = manager.CreateEntityQuery(ComponentType.ReadOnly<Temp>());
        using var ids = temps.ToEntityArray(Allocator.Temp);
        foreach(var id in ids) owner._baselineTemps.Add(id);
        owner._batchId++;owner._leaseLog=log;owner._hasLease=true;owner._generationExclusive=true;owner._allocationUnknown=false;
        owner.ResetSuspensionDiagnostics(artifactStem);
        owner._ownerLineage.BeginObservation();
        owner.TransactionState=NativeTransactionState.LeaseAcquired;
    }

    public static void RegisterDefinition(Entity entity)
    {
        var owner=s_Instance;
        if(owner==null||!owner._hasLease) throw new InvalidOperationException("No owner for the native allocation.");
        try { owner._ownedDefinitions.Add(entity);owner.TransactionState=NativeTransactionState.DefinitionsLive; }
        catch { AllocationUncertain(); throw; }
    }

    public static void DescribeFoliageDefinitions(int requested,int definitions)
    {
        var owner=s_Instance;
        if(owner==null||!owner._hasLease||requested<0||definitions<0||definitions>requested)
            throw new InvalidOperationException("Foliage definition counts require the current owned native lease.");
        owner._foliageRequested=requested;owner._foliageDefinitions=definitions;
    }

    public static void AllocationUncertain()
    {
        if(s_Instance==null)return;
        s_Instance._allocationUnknown=true;s_Instance.TransactionState=NativeTransactionState.AwaitingExclusiveSafety;
        s_Instance._leaseLog?.Event(TelemetryEvent.Failure,TelemetryReason.AllocationUnknown,"Native allocation threw before complete entity registration. Ownership retained; no retry.");
    }

    private bool ObserveExclusiveWindow(bool claimOutputs)
    {
        if(!_hasLease)return false;
        if(_retirementId!=0||_allocationUnknown||!World.IsCreated||!ImportSession.Matches(_leaseEpoch,World.SequenceNumber))
        {ReportSuspension($"Lease unavailable: retirement={_retirementId!=0}, allocationUnknown={_allocationUnknown}, worldCreated={World.IsCreated}, epochMatches={World.IsCreated&&ImportSession.Matches(_leaseEpoch,World.SequenceNumber)}");return false;}
        var serialChanged=_toolSerial!=_capturedSerial;
        var activeChanged=!ReferenceEquals(_capturedActive,_toolSystem.activeTool);
        var lastChanged=!ReferenceEquals(_capturedLast,_lastToolField?.GetValue(_toolSystem));
        if(serialChanged||activeChanged||lastChanged) _generationExclusive=false;
        using var definitions=EntityManager.CreateEntityQuery(new EntityQueryDesc
        {All=new[]{ComponentType.ReadOnly<CreationDefinition>(),ComponentType.ReadOnly<Updated>()},None=new[]{ComponentType.ReadOnly<Deleted>()}});
        using var ids=definitions.ToEntityArray(Allocator.Temp);
        var unexpectedCount=0;
        List<Entity>? unexpected=null;
        foreach(var id in ids) if(!_ownedDefinitions.Contains(id))
        {
            _generationExclusive=false;unexpectedCount++;
            if(_leaseLog?.DiagnosticsEnabled==true&&!_suspensionActive)
            {unexpected??=new List<Entity>();if(unexpected.Count<32)unexpected.Add(id);}
        }
        if(!_generationExclusive)
        {
            TransactionState=NativeTransactionState.AwaitingExclusiveSafety;
            var reason=$"serialChanged={serialChanged}, activeToolChanged={activeChanged}, lastToolChanged={lastChanged}, unexpectedDefinitions={unexpectedCount}";
            if(!serialChanged&&!activeChanged&&!lastChanged&&unexpectedCount==0)reason="Tool/definition checks unchanged; original/owner checks not rerun because prior suspension remains unresolved";
            ReportSuspension(reason,unexpected:unexpected,unexpectedCount:unexpectedCount,
                comparisons:_leaseLog?.DiagnosticsEnabled==true&&!_suspensionActive ? new{claimOutputs,serialChanged,activeChanged,lastChanged,unexpectedCount} : null);
            return false;
        }
        if(!claimOutputs) return true;
        _ownerLineage.BeginObservation();
        _retiringLineage.BeginObservation();
        using var tempQuery=EntityManager.CreateEntityQuery(ComponentType.ReadOnly<Temp>());
        using var temps=tempQuery.ToEntityArray(Allocator.Temp);
        foreach(var id in temps)
        {
            if(!_baselineTemps.Contains(id)){_claimedTemps.Add(id);continue;}
            if(EntityManager.HasComponent<Deleted>(id))continue;
            var temp=EntityManager.GetComponentData<Temp>(id);
            var selectionOnly=(temp.m_Flags&TempFlags.Select)!=0&&
                (temp.m_Flags&~(TempFlags.Select|TempFlags.Essential))==0&&
                temp.m_Original!=Entity.Null&&EntityManager.Exists(temp.m_Original)&&
                !EntityManager.HasComponent<Edge>(id)&&!EntityManager.HasComponent<Node>(id)&&!EntityManager.HasComponent<Lane>(id)&&
                !EntityManager.HasComponent<Game.Objects.Tree>(id)&&!EntityManager.HasComponent<Game.Objects.Plant>(id)&&
                !EntityManager.HasComponent<CreationDefinition>(id)&&
                (!EntityManager.HasComponent<Owner>(id)||EntityManager.GetComponentData<Owner>(id).m_Owner==Entity.Null);
            if(!selectionOnly)return SuspendUncertain("Baseline preview is not selection-only",id,temp.m_Original);
        }
        foreach(var id in _claimedTemps)
        {
            if(!EntityManager.HasComponent<Temp>(id)||EntityManager.HasComponent<Deleted>(id))continue;
            var temp=EntityManager.GetComponentData<Temp>(id);
            if((temp.m_Flags&TempFlags.Cancel)!=0)continue;
            var removing=(temp.m_Flags&TempFlags.Delete)!=0;
            var original=temp.m_Original;
            if(original!=Entity.Null&&!Known(original,removing)) return SuspendUncertain("Preview original has no valid imported owner lineage",id,original);
            if(EntityManager.HasComponent<Owner>(id))
            {var parent=EntityManager.GetComponentData<Owner>(id).m_Owner;if(parent!=Entity.Null&&!Known(parent,removing))return SuspendUncertain("Preview parent has no valid imported owner lineage",id,parent);}
            if(!removing&&EntityManager.HasComponent<Edge>(id))
            {var edge=EntityManager.GetComponentData<Edge>(id);if(!Known(edge.m_Start))return SuspendUncertain("Edge references an unknown start node",id,edge.m_Start);if(!Known(edge.m_End))return SuspendUncertain("Edge references an unknown end node",id,edge.m_End);}
        }
        ResumeOwnershipObservation();
        return true;
    }
    private NativeOwnerLink<Entity> InspectOwnership(Entity entity) => InspectOwnership(entity,false);
    private NativeOwnerLink<Entity> InspectRetiringOwnership(Entity entity) => InspectOwnership(entity,true);
    private NativeOwnerLink<Entity> InspectOwnership(Entity entity,bool removing)
    {
        if(entity==Entity.Null||!EntityManager.Exists(entity)||(!removing&&EntityManager.HasComponent<Deleted>(entity))||
            EntityManager.HasComponent<PrefabData>(entity)||EntityManager.HasComponent<NetCompositionData>(entity)||
            EntityManager.HasComponent<CreationDefinition>(entity))return default;
        var temporary=EntityManager.HasComponent<Temp>(entity);
        var root=_claimedTemps.Contains(entity)||(!temporary&&_ownedEntities.Contains(entity));
        var parent=EntityManager.HasComponent<Owner>(entity)?EntityManager.GetComponentData<Owner>(entity).m_Owner:Entity.Null;
        return new NativeOwnerLink<Entity>(true,root,parent!=Entity.Null,parent);
    }
    private bool Known(Entity entity,bool removing=false)
    {
        var lineage=removing?_retiringLineage:_ownerLineage;
        if(!lineage.Resolve(entity,removing?InspectRetiringOwnership:InspectOwnership,out var root,out var reason))return false;
        if(!_ownedEntities.Contains(entity)&&!_claimedTemps.Contains(entity))
        {
            _leaseLog?.Count("Native downstream owner-lineage references accepted");
            if(_leaseLog?.DiagnosticsEnabled==true&&_leaseLog.Metric("Native downstream owner-lineage references accepted")<=64)
                _leaseLog.Diagnostic($"NATIVE OWNER LINEAGE entity={entity}; root={root}; proof={reason}; batch={_diagnosticBatchStem}; prefabOrProximityGuess=false; geometryCertificationRequired=false");
        }
        return true;
    }
    // The Resume button. Only a batch whose Apply and Clear were never issued
    // qualifies: nothing of it reached the map, and its definitions are still
    // the importer's alone. They are destroyed, the tool active when the batch
    // began is restored, and once the game has retired the batch's preview
    // (CanStartExclusive) the batch reports Interrupted so the importer
    // submits the same sections again. Nothing is applied or cleared here.
    public static bool CanResume => s_Instance is { } owner&&owner._hasLease&&owner._suspensionActive&&
        !s_Applied&&!s_ClearingPreview&&!owner._allocationUnknown&&owner._retirementId==0&&!s_Resuming;

    public static bool Resume(out string message)
    {
        var owner=s_Instance;
        if(owner==null||!CanResume)
        {message="Nothing can be resumed: no unissued native batch is suspended.";return false;}
        var manager=owner.EntityManager;var destroyed=0;
        foreach(var id in owner._ownedDefinitions)if(manager.Exists(id)&&manager.HasComponent<CreationDefinition>(id)){manager.DestroyEntity(id);destroyed++;}
        var restored=owner._capturedActive!=null&&!ReferenceEquals(owner._toolSystem.activeTool,owner._capturedActive);
        if(restored)owner._toolSystem.activeTool=owner._capturedActive;
        owner._leaseLog?.Warn($"Native placement resumed by the user after {(DateTime.UtcNow-owner._suspensionUtc).TotalSeconds:0} s: {destroyed} unapplied definitions of batch {owner._diagnosticBatchStem} discarded; {(restored?"restored "+ToolName(owner._capturedActive):"tool unchanged")}; the batch is submitted again once its preview retires.");
        owner._hasLease=false;owner._ownedDefinitions.Clear();owner._baselineTemps.Clear();owner._claimedTemps.Clear();
        owner._ownedEntities.DiscardBatch();owner._suspensionActive=false;owner.TransactionState=NativeTransactionState.Cleared;
        s_Resuming=true;s_ResumeFrames=0;
        message="Resuming: the interrupted batch will be placed again.";return true;
    }

    private bool SuspendUncertain(string reason,Entity culprit,Entity related)
    { TransactionState=NativeTransactionState.AwaitingExclusiveSafety;ReportSuspension(reason,culprit,related);return false; }

    private void IssueOwnedClear(ToolBaseSystem tool)
    {
        if(s_Applied||!ObserveExclusiveWindow(true))throw new InvalidOperationException("Native Clear has no verified unissued exclusive lease.");
        if(_leaseLog?.DiagnosticsEnabled==true)_leaseLog.Checkpoint("native/before-clear",
            new RunArchive.NativeOperationIntent(_leaseEpoch.MapSession,_leaseEpoch.Generation,_leaseEpoch.WorldSequenceNumber,"PreviewGenerated",_ownedDefinitions.Count,_claimedTemps.Count,s_ApplyInvocations));
        foreach(var id in _claimedTemps)s_Submitted.Add(id);
        s_Requested=true;s_ClearingPreview=true;s_WaitFrames=0;TransactionState=NativeTransactionState.ClearIssued;
        _applyModeSetter!.Invoke(tool,new object[]{ApplyMode.Clear});
        TransactionState=NativeTransactionState.AwaitingClearRetirement;
    }

    private void RecordSettlement()
    {
        if(s_Applied&&_foliageRequested>=0)
        {
            var accepted=0;var unverified=0;
            foreach(var pair in _foliageCandidates)
            {
                var id=pair.Key;
                var valid=EntityManager.Exists(id)&&!EntityManager.HasComponent<Temp>(id)&&!EntityManager.HasComponent<Deleted>(id)&&
                    EntityManager.HasComponent<PrefabRef>(id)&&EntityManager.GetComponentData<PrefabRef>(id).m_Prefab==pair.Value.Prefab&&
                    EntityManager.HasComponent<Game.Objects.Transform>(id)&&
                    (EntityManager.HasComponent<Game.Objects.Plant>(id)||EntityManager.HasComponent<Game.Objects.Tree>(id))&&
                    (!pair.Value.Tree||EntityManager.HasComponent<Game.Objects.Tree>(id));
                if(valid)accepted++;else unverified++;
            }
            _leaseLog?.Count("Vegetation accepted native objects",accepted);
            _leaseLog?.Count("Vegetation native outcomes unverified",unverified);
            _leaseLog?.Diagnostic($"VEGETATION OUTCOME batch={_diagnosticBatchStem}; candidatesProcessed={_foliageRequested}; definitionsSubmitted={_foliageDefinitions}; capturedNewPlantTemps={_foliageCandidates.Count}; acceptedCapturedObjects={accepted}; unverifiedCapturedObjects={unverified}; definitionToObjectMatching=false; definitionMinusCapturedDifference={_foliageDefinitions-_foliageCandidates.Count}; differenceIsNotProvenLoss=true; mapSession={_leaseEpoch.MapSession}; generation={_leaseEpoch.Generation}");
        }
        _foliageCandidates.Clear();
        if(s_Applied)
        {
            _ownedEntities.SettleApplied(entity => EntityManager.Exists(entity)&&
                !EntityManager.HasComponent<Deleted>(entity)&&!EntityManager.HasComponent<Temp>(entity)&&
                !EntityManager.HasComponent<CreationDefinition>(entity)&&!EntityManager.HasComponent<PrefabData>(entity)&&
                !EntityManager.HasComponent<NetCompositionData>(entity));
            _leaseLog?.Diagnostic($"NATIVE OWNERSHIP SETTLED batch={_diagnosticBatchStem}; retainedEntities={_ownedEntities.Count}; includesGeneratedObjects=true; candidatesFromExclusivePreviewOnly=true; worldScan=false; geometryCertificationRequired=false");
        }
        else _ownedEntities.DiscardBatch();
        _hasLease=false;_ownedDefinitions.Clear();_baselineTemps.Clear();_claimedTemps.Clear();_leaseLog=null;
        TransactionState=s_Applied?NativeTransactionState.Settled:NativeTransactionState.Cleared;
    }

    private void CaptureFoliageCandidates()
    {
        _foliageCandidates.Clear();
        var missingContract=0;var retiredOrCancelled=0;var originalUpdates=0;var nonPlant=0;var wrongPrefab=0;
        foreach(var id in _claimedTemps)
        {
            if(!EntityManager.HasComponent<Temp>(id)||!EntityManager.HasComponent<PrefabRef>(id)){missingContract++;continue;}
            var temp=EntityManager.GetComponentData<Temp>(id);
            if(EntityManager.HasComponent<Deleted>(id)||(temp.m_Flags&(TempFlags.Delete|TempFlags.Cancel))!=0){retiredOrCancelled++;continue;}
            if(temp.m_Original!=Entity.Null){originalUpdates++;continue;}
            if(!EntityManager.HasComponent<Game.Objects.Plant>(id)&&!EntityManager.HasComponent<Game.Objects.Tree>(id)){nonPlant++;continue;}
            var prefab=EntityManager.GetComponentData<PrefabRef>(id).m_Prefab;
            if(EntityManager.HasComponent<PlantData>(prefab))_foliageCandidates.Add(id,(prefab,EntityManager.HasComponent<TreeData>(prefab)));
            else wrongPrefab++;
        }
        if(_foliageRequested>=0)_leaseLog?.Diagnostic($"VEGETATION PREVIEW FUNNEL batch={_diagnosticBatchStem}; candidatesProcessed={_foliageRequested}; definitionsSubmitted={_foliageDefinitions}; claimedTemps={_claimedTemps.Count}; eligibleNewPlantTemps={_foliageCandidates.Count}; missingTempOrPrefabRef={missingContract}; deletedOrCancelled={retiredOrCancelled}; originalUpdates={originalUpdates}; nonPlantObjects={nonPlant}; missingPlantPrefabData={wrongPrefab}; filtersPartitionClaimedTemps=true; definitionToObjectMatching=false");
    }

    private void CaptureOwnershipCandidates()
    {
        // Called only after the complete original/owner checks and immediately
        // before Apply. Clear/preflight never promotes candidates to ownership.
        _ownedEntities.DiscardBatch();
        foreach(var id in _claimedTemps)
        {
            if(!EntityManager.HasComponent<Temp>(id)||EntityManager.HasComponent<Deleted>(id))continue;
            var temp=EntityManager.GetComponentData<Temp>(id);
            _ownedEntities.Capture(id,temp.m_Original,temp.m_Original!=Entity.Null,
                (temp.m_Flags&TempFlags.Delete)!=0,(temp.m_Flags&TempFlags.Cancel)!=0);
        }
        _leaseLog?.Diagnostic($"NATIVE OWNERSHIP CAPTURE batch={_diagnosticBatchStem}; claimedPreviewEntities={_claimedTemps.Count}; retainedBeforeApply={_ownedEntities.Count}; includesGeneratedObjects=true; applyIssued=false");
    }

    public static long BeginMapRetirement()
    {
        var owner=s_Instance;if(owner==null)return 0;
        if(owner._retirementId!=0)return owner._retirementId;
        owner._retirementId=++owner._batchId;
        owner.TransactionState=NativeTransactionState.MapRetiring;
        owner._retirementCovered=!owner._allocationUnknown;
        owner._retirementNotificationSafe=!owner._hasLease||(!s_Applied&&!s_ClearingPreview);
        if(!owner._hasLease)owner._leaseEpoch=new ImportEpoch(ImportSession.Map,ImportSession.Generation,owner.World.SequenceNumber);
        if(owner.World.IsCreated&&owner.World.SequenceNumber==owner._leaseEpoch.WorldSequenceNumber)
        {
            try
            {
                owner.EntityManager.CompleteAllTrackedJobs();
                bool Covered(Entity id) => !owner.EntityManager.Exists(id)||
                    ((owner.EntityManager.HasComponent<CreationDefinition>(id)||owner.EntityManager.HasComponent<PrefabRef>(id))&&
                     !owner.EntityManager.HasComponent<PrefabData>(id)&&!owner.EntityManager.HasComponent<NetCompositionData>(id));
                foreach(var id in owner._ownedDefinitions)owner._retirementCovered&=Covered(id);
                foreach(var id in owner._claimedTemps)owner._retirementCovered&=Covered(id);
                if(owner._hasLease&&owner._generationExclusive&&owner._capturedSerial==owner._toolSerial&&
                   ReferenceEquals(owner._capturedActive,owner._toolSystem.activeTool)&&
                   ReferenceEquals(owner._capturedLast,owner._lastToolField?.GetValue(owner._toolSystem)))
                {
                    var expected=s_Applied?ApplyMode.Apply:s_ClearingPreview?ApplyMode.Clear:ApplyMode.None;
                    if(owner._capturedLast!=null&&owner._capturedLast.applyMode==expected)
                    {
                        if(expected!=ApplyMode.None)owner._applyModeSetter!.Invoke(owner._capturedLast,new object[]{ApplyMode.None});
                        owner._retirementNotificationSafe=true;
                    }
                }
            }
            catch(Exception error)
            {owner._retirementCovered=false;owner._leaseLog?.Event(TelemetryEvent.Failure,TelemetryReason.OwnerUncertain,"Native map retirement is uncertain: "+error.GetBaseException().Message);}
        }
        return owner._retirementId;
    }

    public static void FinishMapRetirement()
    {
        var owner=s_Instance;if(owner==null||owner._retirementId==0)return;
        var worldChanged=!owner.World.IsCreated||owner.World.SequenceNumber!=owner._leaseEpoch.WorldSequenceNumber;
        if(!worldChanged&&(!owner._retirementCovered||!owner._retirementNotificationSafe))
        {owner.TransactionState=NativeTransactionState.UnresolvedMapRetirement;return;}
        // Loading-complete is the verified engine clear boundary. NEVER inspect
        // or delete old IDs in the new map, even when the World is reused.
        owner._hasLease=false;owner._retirementId=0;owner._ownedDefinitions.Clear();owner._baselineTemps.Clear();
        owner._claimedTemps.Clear();owner._ownedEntities.Clear();owner._leaseLog=null;
        owner._ownerLineage.BeginObservation();
        owner._foliageCandidates.Clear();
        owner.TransactionState=worldChanged?NativeTransactionState.WorldRetired:NativeTransactionState.MapRetired;
        s_Requested=false;s_Applied=false;s_ClearingPreview=false;s_Submitted.Clear();CancelPending();
    }
}
}
