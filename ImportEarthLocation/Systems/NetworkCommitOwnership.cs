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
    private readonly HashSet<Entity> _baselineTemps = new();
    private readonly HashSet<Entity> _claimedTemps = new();
    private readonly HashSet<Entity> _ownedNetworks = new();
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

    private void ToolChanged(ToolBaseSystem tool) { _toolSerial++; }

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
        var owner=s_Instance;if(owner==null||!owner._hasLease||owner._allocationUnknown)return;
        foreach(var id in owner._ownedDefinitions)if(manager.Exists(id))return;
        using var query=manager.CreateEntityQuery(ComponentType.ReadOnly<Temp>());
        using var ids=query.ToEntityArray(Allocator.Temp);
        foreach(var id in ids)if(!owner._baselineTemps.Contains(id))return;
        owner._hasLease=false;owner._ownedDefinitions.Clear();owner._baselineTemps.Clear();owner._leaseLog=null;
        owner.TransactionState=NativeTransactionState.Cleared;
    }

    public static void BeginDefinitionBatch(EntityManager manager, ImportLog log)
    {
        var owner = s_Instance ?? manager.World.GetOrCreateSystemManaged<NetworkCommitSystem>();
        if (owner._hasLease || owner._retirementId != 0)
            throw new InvalidOperationException("The previous native lease has not retired.");
        if (!CanStartExclusive(manager, out var reason)) throw new InvalidOperationException(reason);
        using var definitions = manager.CreateEntityQuery(new EntityQueryDesc
        { All=new[]{ComponentType.ReadOnly<CreationDefinition>(),ComponentType.ReadOnly<Updated>()},None=new[]{ComponentType.ReadOnly<Deleted>()} });
        if (!definitions.IsEmptyIgnoreFilter) throw new InvalidOperationException("An unrelated native definition producer is active.");
        var last = owner._lastToolField?.GetValue(owner._toolSystem) as ToolBaseSystem;
        if (last == null || last.applyMode != ApplyMode.None)
            throw new InvalidOperationException("The native tool has an unconsumed notification.");
        if(owner._leaseEpoch.MapSession!=ImportSession.Map||owner._leaseEpoch.Generation!=ImportSession.Generation)
            owner._ownedNetworks.Clear();
        owner._leaseEpoch = new ImportEpoch(ImportSession.Map,ImportSession.Generation,manager.World.SequenceNumber);
        s_Applied=false;s_ClearingPreview=false;s_ApplyInvocations=0;s_Submitted.Clear();
        owner._capturedActive=owner._toolSystem.activeTool;owner._capturedLast=last;owner._capturedSerial=owner._toolSerial;
        owner._ownedDefinitions.Clear();owner._baselineTemps.Clear();owner._claimedTemps.Clear();
        owner._foliageCandidates.Clear();
        using var temps = manager.CreateEntityQuery(ComponentType.ReadOnly<Temp>());
        using var ids = temps.ToEntityArray(Allocator.Temp);
        foreach(var id in ids) owner._baselineTemps.Add(id);
        owner._batchId++;owner._leaseLog=log;owner._hasLease=true;owner._generationExclusive=true;owner._allocationUnknown=false;
        owner.TransactionState=NativeTransactionState.LeaseAcquired;
    }

    public static void RegisterDefinition(Entity entity)
    {
        var owner=s_Instance;
        if(owner==null||!owner._hasLease) throw new InvalidOperationException("No owner for the native allocation.");
        try { owner._ownedDefinitions.Add(entity);owner.TransactionState=NativeTransactionState.DefinitionsLive; }
        catch { AllocationUncertain(); throw; }
    }

    public static void AllocationUncertain()
    {
        if(s_Instance==null)return;
        s_Instance._allocationUnknown=true;s_Instance.TransactionState=NativeTransactionState.AwaitingExclusiveSafety;
        s_Instance._leaseLog?.Event(TelemetryEvent.Failure,TelemetryReason.AllocationUnknown,"Native allocation threw before complete entity registration. Ownership retained; no retry.");
    }

    private bool ObserveExclusiveWindow(bool claimOutputs)
    {
        if(!_hasLease||_retirementId!=0||_allocationUnknown||!World.IsCreated||
           !ImportSession.Matches(_leaseEpoch,World.SequenceNumber)) return false;
        if(_toolSerial!=_capturedSerial||!ReferenceEquals(_capturedActive,_toolSystem.activeTool)||
           !ReferenceEquals(_capturedLast,_lastToolField?.GetValue(_toolSystem))) _generationExclusive=false;
        using var definitions=EntityManager.CreateEntityQuery(new EntityQueryDesc
        {All=new[]{ComponentType.ReadOnly<CreationDefinition>(),ComponentType.ReadOnly<Updated>()},None=new[]{ComponentType.ReadOnly<Deleted>()}});
        using var ids=definitions.ToEntityArray(Allocator.Temp);
        foreach(var id in ids) if(!_ownedDefinitions.Contains(id)) _generationExclusive=false;
        if(!_generationExclusive)
        {
            TransactionState=NativeTransactionState.AwaitingExclusiveSafety;
            _leaseLog?.Event(TelemetryEvent.Failure,TelemetryReason.OwnerUncertain,"The native producer/tool changed during generation. Apply and Clear suspended; ownership is not inferred from source geometry.");
            return false;
        }
        if(!claimOutputs) return true;
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
            if(!selectionOnly)return SuspendUncertain();
        }
        foreach(var id in _claimedTemps)
        {
            if(!EntityManager.HasComponent<Temp>(id))continue;
            var original=EntityManager.GetComponentData<Temp>(id).m_Original;
            if(original!=Entity.Null&&!_ownedNetworks.Contains(original)&&!_claimedTemps.Contains(original)) return SuspendUncertain();
            if(EntityManager.HasComponent<Owner>(id))
            {var parent=EntityManager.GetComponentData<Owner>(id).m_Owner;if(parent!=Entity.Null&&!_ownedNetworks.Contains(parent)&&!_claimedTemps.Contains(parent))return SuspendUncertain();}
            if(EntityManager.HasComponent<Edge>(id))
            {var edge=EntityManager.GetComponentData<Edge>(id);if(!Known(edge.m_Start)||!Known(edge.m_End))return SuspendUncertain();}
        }
        return true;
    }
    private bool Known(Entity entity) => entity!=Entity.Null&&(_claimedTemps.Contains(entity)||_ownedNetworks.Contains(entity));
    private bool SuspendUncertain()
    { _generationExclusive=false;TransactionState=NativeTransactionState.AwaitingExclusiveSafety;return false; }

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
        if(s_Applied&&_foliageCandidates.Count>0)
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
        }
        _foliageCandidates.Clear();
        foreach(var entity in _claimedTemps)
        {
            if(!EntityManager.Exists(entity)||EntityManager.HasComponent<Deleted>(entity))continue;
            if(EntityManager.HasComponent<Node>(entity)||EntityManager.HasComponent<Edge>(entity)||EntityManager.HasComponent<Lane>(entity)) _ownedNetworks.Add(entity);
        }
        _hasLease=false;_ownedDefinitions.Clear();_baselineTemps.Clear();_claimedTemps.Clear();_leaseLog=null;
        TransactionState=s_Applied?NativeTransactionState.Settled:NativeTransactionState.Cleared;
    }

    private void CaptureFoliageCandidates()
    {
        _foliageCandidates.Clear();
        foreach(var id in _claimedTemps)
        {
            if(!EntityManager.HasComponent<Temp>(id)||EntityManager.HasComponent<Deleted>(id)||!EntityManager.HasComponent<PrefabRef>(id))continue;
            var temp=EntityManager.GetComponentData<Temp>(id);
            if(temp.m_Original!=Entity.Null||(temp.m_Flags&(TempFlags.Delete|TempFlags.Cancel))!=0)continue;
            if(!EntityManager.HasComponent<Game.Objects.Plant>(id)&&!EntityManager.HasComponent<Game.Objects.Tree>(id))continue;
            var prefab=EntityManager.GetComponentData<PrefabRef>(id).m_Prefab;
            if(EntityManager.HasComponent<PlantData>(prefab))_foliageCandidates.Add(id,(prefab,EntityManager.HasComponent<TreeData>(prefab)));
        }
    }

    public static long BeginMapRetirement()
    {
        var owner=s_Instance;if(owner==null)return 0;
        if(owner._retirementId!=0)return owner._retirementId;
        owner._retirementId=++owner._batchId;
        owner.TransactionState=NativeTransactionState.MapRetiring;
        owner._retirementCovered=!owner._allocationUnknown;
        owner._retirementNotificationSafe=!owner._hasLease;
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
        owner._claimedTemps.Clear();owner._ownedNetworks.Clear();owner._leaseLog=null;
        owner._foliageCandidates.Clear();
        owner.TransactionState=worldChanged?NativeTransactionState.WorldRetired:NativeTransactionState.MapRetired;
        s_Requested=false;s_Applied=false;s_ClearingPreview=false;s_Submitted.Clear();CancelPending();
    }
}
}
