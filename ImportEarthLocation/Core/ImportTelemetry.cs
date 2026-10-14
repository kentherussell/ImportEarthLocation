using System;
using System.Diagnostics;

namespace ImportEarthLocation.Core;

// Fixed vocabulary and storage: never keyed by an entity, source ID or message.
public enum TelemetryStage { Validate, Download, Source, AssetCapture, Design, Earthwork, Forest, ReadyWait, NetworkApply, VegetationApply, Environment, FinalAudit, Retirement }
public enum TelemetryEvent { RunStart, StageStart, StageEnd, Progress, UnitOutcome, BatchOutcome, TransactionState, ResourceSample, Failure, RunEnd }
public enum TelemetryOperation { SourceSelection, Projection, CorridorMatch, HorizontalFit, JunctionFrames, ContactBuild, PortalDistance, WitnessSearch, ProfileSolve, EarthworkStamp, ForestPrepare, VegetationPlan, CandidateSort, AssetCapture, NetworkCreate, VegetationCreate, TempCensus, NativeApply, NativeClear, NativeAudit, SyncWait, ArchiveCapture, ArchiveWrite }
public enum TelemetryReason { None, ClassOff, ServiceOmitted, MissingSource, OutsideMap, NonFinite, InvalidAsset, WidthChanged, AmbiguousPair, SourceFitExceeded, GradeResidual, ClearanceResidual, CoverResidual, LaneDisconnected, PendingPublication, PendingNeighbor, OwnerUncertain, AllocationUnknown, Cancelled, StaleEpoch, WaitOverdue, DiskUnavailable, ResourceUnavailable, LegacyOther }
public enum RunOutcome { Success, PartialRequiresReview, Stopped, Cancelled, PreparationUnavailable, Failed, RetirementUncertain }

/// <summary>Run-owned fixed-size statistics; no timers, queries or background work.</summary>
public sealed class ImportTelemetry
{
    private const int Stages = 13, Operations = 24, Events = 11, Reasons = 25;
    private readonly object _gate = new();
    private readonly long[] _counts = new long[Stages * Operations];
    private readonly double[] _seconds = new double[Stages * Operations];
    private readonly double[] _maxSeconds = new double[Stages * Operations];
    private readonly long[] _zeros = new long[Stages * Operations];
    private readonly long[]? _histograms;
    private readonly long[] _repeats = new long[Stages * Events * Reasons];
    private long _lastProgress;
    public ImportEpoch Epoch { get; }
    public string RunId { get; } = Guid.NewGuid().ToString("N");
    public bool Detailed { get; }
    public TelemetryStage Stage { get; private set; }

    public ImportTelemetry(ImportEpoch epoch, bool detailed)
    { Epoch = epoch; Detailed = detailed; if (detailed) _histograms = new long[Stages * Operations * 32]; }

    public void SetStage(TelemetryStage stage) { lock (_gate) Stage = stage; }
    public bool First(TelemetryEvent kind, TelemetryReason reason)
    {
        lock (_gate) return _repeats[((int)Stage * Events + (int)kind) * Reasons + (int)reason]++ == 0;
    }
    public bool ProgressDue()
    {
        var now = Stopwatch.GetTimestamp();
        lock (_gate)
        {
            if (_lastProgress != 0 && (now - _lastProgress) / (double)Stopwatch.Frequency < 10) return false;
            _lastProgress = now; return true;
        }
    }
    public void Observe(TelemetryStage stage, TelemetryOperation operation, long count, double seconds)
    {
        if (seconds < 0 || double.IsNaN(seconds) || double.IsInfinity(seconds)) return;
        var index = (int)stage * Operations + (int)operation;
        lock (_gate)
        {
            _counts[index] += count; _seconds[index] += seconds;
            _maxSeconds[index] = Math.Max(_maxSeconds[index], seconds);
            if (seconds == 0) _zeros[index]++;
            else if (_histograms != null)
            {
                var ms = seconds * 1000; var bucket = 0;
                while (ms >= 2 && bucket < 31) { ms *= .5; bucket++; }
                _histograms[index * 32 + bucket]++;
            }
        }
    }
    public (long Count, double Seconds, double Maximum, long ZeroDurations) Read(TelemetryStage stage, TelemetryOperation operation)
    {
        var index = (int)stage * Operations + (int)operation;
        lock (_gate) return (_counts[index], _seconds[index], _maxSeconds[index], _zeros[index]);
    }
}
