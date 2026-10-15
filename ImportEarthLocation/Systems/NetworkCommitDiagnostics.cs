using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Game.Common;
using Game.Prefabs;
using Game.Tools;
using ImportEarthLocation.Core;
using Unity.Collections;
using Unity.Entities;

namespace ImportEarthLocation.Systems
{

public sealed partial class NetworkCommitSystem
{
    private sealed record ToolChangeRecord(int Frame, long Serial, string? ToolType, int Instance);
    private readonly List<ToolChangeRecord> _toolChangeHistory = new(8);
    private string? _suspensionReason, _currentSuspensionReason, _diagnosticBatchStem;
    private DateTime _suspensionUtc, _nextSuspensionDiagnosticUtc;
    private bool _suspensionActive;
    private int _suspensionSequence;

    private void RecordToolChange(ToolBaseSystem tool)
    {
        if (_leaseLog?.DiagnosticsEnabled != true) return;
        if (_toolChangeHistory.Count == 8) _toolChangeHistory.RemoveAt(0);
        _toolChangeHistory.Add(new ToolChangeRecord(UnityEngine.Time.frameCount, _toolSerial,
            tool?.GetType().FullName, tool == null ? 0 : RuntimeHelpers.GetHashCode(tool)));
    }

    private void ResetSuspensionDiagnostics(string? artifactStem)
    {
        _suspensionReason = _currentSuspensionReason = null;
        _suspensionUtc = _nextSuspensionDiagnosticUtc = default;
        _diagnosticBatchStem = artifactStem ?? $"native-lease-{_batchId:D6}";
        _toolChangeHistory.Clear();
        _suspensionActive=false;_suspensionSequence=0;
    }

    // Observational only: never reacquire ownership or clear an ambiguous preview.
    // Preserve the first trigger even if later polls find that it disappeared.
    private void ReportSuspension(string currentReason, Entity culprit = default, Entity related = default,
        IReadOnlyList<Entity>? unexpected = null, int unexpectedCount = 0, object? comparisons = null)
    {
        var now = DateTime.UtcNow;
        _currentSuspensionReason = currentReason;
        if (!_suspensionActive)
        {
            _suspensionActive=true;_suspensionSequence++;
            _suspensionReason = currentReason;
            _suspensionUtc = now;
            _nextSuspensionDiagnosticUtc = now.AddSeconds(10);
            _leaseLog?.Warn($"Native placement suspended: {currentReason}; batch={_diagnosticBatchStem}; Apply issued={s_Applied}; Clear issued={s_ClearingPreview}. Ownership remains unresolved; no automatic Apply/Clear or replay.");
            if (_leaseLog?.DiagnosticsEnabled != true) return;
            try
            {
                var records = new List<object>
                {
                    new { kind="suspension", reason=currentReason, batch=_diagnosticBatchStem,
                        frame=UnityEngine.Time.frameCount, utc=now, state=TransactionState.ToString(),
                        comparisons, capturedSerial=_capturedSerial, currentSerial=_toolSerial,
                        capturedActive=ToolEvidence(_capturedActive), currentActive=ToolEvidence(_toolSystem.activeTool),
                        capturedLast=ToolEvidence(_capturedLast), currentLast=ToolEvidence(_lastToolField?.GetValue(_toolSystem) as ToolBaseSystem),
                        ownedDefinitions=_ownedDefinitions.Count, baselineTemps=_baselineTemps.Count,
                        claimedTemps=_claimedTemps.Count, ownedEntities=_ownedEntities.Count, submitted=s_Submitted.Count,
                        applyIssued=s_Applied, clearIssued=s_ClearingPreview, applyInvocations=s_ApplyInvocations,
                        epoch=_leaseEpoch, retirementId=_retirementId, allocationUnknown=_allocationUnknown,
                        unexpectedDefinitions=unexpectedCount, recordedUnexpectedDefinitions=unexpected?.Count ?? 0,
                        truncated=unexpectedCount>(unexpected?.Count ?? 0), producerAttribution="not-known-from-entity-data", readOnly=true }
                };
                foreach (var change in _toolChangeHistory) records.Add(new { kind="tool-change", change });
                var recorded = new HashSet<Entity>();
                void Capture(Entity id)
                {
                    if (id != Entity.Null && recorded.Add(id)) records.Add(SuspensionEntityEvidence(id));
                }
                Capture(culprit); Capture(related);
                if (unexpected != null) foreach (var id in unexpected) Capture(id);
                var suffix=_suspensionSequence==1?"":$"-{_suspensionSequence:D2}";
                _leaseLog.DiagnosticRecords(_diagnosticBatchStem+"-ownership-suspension"+suffix+".ndjson", records, 1024L*1024,critical:true);
            }
            catch (Exception error)
            {
                // A diagnostic capture must never create a new placement failure.
                _leaseLog.Diagnostic("NATIVE SUSPENSION CAPTURE unavailable: "+error.GetBaseException().Message);
            }
            return;
        }
        if (_leaseLog?.DiagnosticsEnabled != true || now < _nextSuspensionDiagnosticUtc) return;
        _nextSuspensionDiagnosticUtc = now.AddSeconds(10);
        _leaseLog.Diagnostic($"NATIVE SUSPENSION WAIT batch={_diagnosticBatchStem}; originalReason={_suspensionReason}; currentReason={currentReason}; elapsedSeconds={(now-_suspensionUtc).TotalSeconds:0.0}; frame={UnityEngine.Time.frameCount}; applyIssued={s_Applied}; clearIssued={s_ClearingPreview}; ownershipReacquired=false; newSnapshot=false");
    }

    private void ResumeOwnershipObservation()
    {
        if(!_suspensionActive)return;
        _leaseLog?.Diagnostic($"NATIVE OWNERSHIP RESUMED batch={_diagnosticBatchStem}; previousReason={_suspensionReason}; elapsedSeconds={(DateTime.UtcNow-_suspensionUtc).TotalSeconds:0.0}; fullReferenceChecksPassed=true; applyReplayed=false");
        _suspensionActive=false;
        TransactionState=s_Requested?NativeTransactionState.PreviewGenerated:NativeTransactionState.DefinitionsLive;
    }

    private static object ToolEvidence(ToolBaseSystem? tool) => new
    {
        type=tool?.GetType().FullName, instance=tool == null ? 0 : RuntimeHelpers.GetHashCode(tool),
        mode=tool?.applyMode.ToString()
    };

    private object SuspensionEntityEvidence(Entity id)
    {
        string Id(Entity entity) => $"{entity.Index}:{entity.Version}";
        if (!EntityManager.Exists(id)) return new { kind="entity", entity=Id(id), exists=false };
        using var types = EntityManager.GetComponentTypes(id, Allocator.Temp);
        var components = new List<string>();
        foreach (var type in types) components.Add(type.GetManagedType()?.FullName ?? type.ToString());
        object? creation = null, temp = null;
        if (EntityManager.HasComponent<CreationDefinition>(id))
        {
            var value = EntityManager.GetComponentData<CreationDefinition>(id);
            creation = new { prefab=Id(value.m_Prefab), original=Id(value.m_Original), owner=Id(value.m_Owner),
                attached=Id(value.m_Attached), flags=value.m_Flags.ToString() };
        }
        if (EntityManager.HasComponent<Temp>(id))
        {
            var value = EntityManager.GetComponentData<Temp>(id);
            temp = new { original=Id(value.m_Original), flags=value.m_Flags.ToString() };
        }
        return new { kind="entity", entity=Id(id), exists=true, components, creation, temp,
            updated=EntityManager.HasComponent<Updated>(id), deleted=EntityManager.HasComponent<Deleted>(id),
            importerDefinition=_ownedDefinitions.Contains(id), importerTemp=_claimedTemps.Contains(id),
            baselineTemp=_baselineTemps.Contains(id), importerOwnedEntity=_ownedEntities.Contains(id),
            owner=EntityManager.HasComponent<Owner>(id) ? Id(EntityManager.GetComponentData<Owner>(id).m_Owner) : null,
            prefab=EntityManager.HasComponent<PrefabRef>(id) ? Id(EntityManager.GetComponentData<PrefabRef>(id).m_Prefab) : null };
    }

    public static string PlacementProgress
    {
        get
        {
            var owner = s_Instance;
            if (owner?._hasLease != true) return "native batch settled; checking live entities";
            if (owner._suspensionActive)
                return $"SUSPENDED: {owner._suspensionReason}; {(DateTime.UtcNow-owner._suspensionUtc).TotalSeconds:0}s; ownership unresolved (not building)";
            if (s_ClearingPreview) return $"waiting for preview cleanup ({s_Submitted.Count} submitted entities)";
            if (s_Applied) return $"waiting for native application ({s_Submitted.Count} submitted entities)";
            return s_Requested ? "waiting for native preview and source qualification" : "generating native nodes, edges and lanes";
        }
    }
}
}
