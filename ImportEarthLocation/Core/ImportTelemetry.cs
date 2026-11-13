using System;
using System.Diagnostics;

namespace ImportEarthLocation.Core;

// Fixed vocabulary and storage: never keyed by an entity, source ID or message.
public enum TelemetryStage
{
    Validate,
    Download,
    Source,
    AssetCapture,
    Design,
    Earthwork,
    Forest,
    ReadyWait,
    NetworkApply,
    VegetationApply,
    Environment,
    FinalAudit,
    Retirement
}

public enum TelemetryEvent
{
    RunStart,
    StageStart,
    StageEnd,
    Progress,
    UnitOutcome,
    BatchOutcome,
    TransactionState,
    ResourceSample,
    Failure,
    RunEnd
}

public enum TelemetryReason
{
    None,
    ClassOff,
    ServiceOmitted,
    MissingSource,
    OutsideMap,
    NonFinite,
    InvalidAsset,
    WidthChanged,
    AmbiguousPair,
    SourceFitExceeded,
    GradeResidual,
    ClearanceResidual,
    CoverResidual,
    LaneDisconnected,
    PendingPublication,
    PendingNeighbor,
    OwnerUncertain,
    AllocationUnknown,
    Cancelled,
    StaleEpoch,
    WaitOverdue,
    DiskUnavailable,
    ResourceUnavailable,
    LegacyOther
}

/// <summary>Run-owned fixed-size statistics; no timers, queries or background work.</summary>
public sealed class ImportTelemetry
{
    private const int Stages = 13, Events = 11, Reasons = 25;
    private readonly object _gate = new();
    private readonly long[] _repeats = new long[Stages * Events * Reasons];
    private long _lastProgress;

    public ImportEpoch Epoch { get; }
    public string RunId { get; } = Guid.NewGuid().ToString("N");
    public bool Detailed { get; }
    public TelemetryStage Stage { get; private set; }

    public ImportTelemetry(ImportEpoch epoch, bool detailed)
    {
        Epoch = epoch;
        Detailed = detailed;
    }

    public void SetStage(TelemetryStage stage)
    {
        lock (_gate)
            Stage = stage;
    }

    public bool First(TelemetryEvent kind, TelemetryReason reason)
    {
        lock (_gate)
            return _repeats[((int)Stage * Events + (int)kind) * Reasons + (int)reason]++ == 0;
    }

    public bool ProgressDue()
    {
        var now = Stopwatch.GetTimestamp();

        lock (_gate)
        {
            if (_lastProgress != 0 && (now - _lastProgress) / (double)Stopwatch.Frequency < 10)
                return false;

            _lastProgress = now;

            return true;
        }
    }
}
