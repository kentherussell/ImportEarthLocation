using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;

namespace ImportEarthLocation.Core;

public sealed class ImportLog
{
    // NOTE: List-based ring buffer on purpose. Unity's mscorlib.dll (referenced by
    // the official CS2 toolchain) also defines Queue<T>, which collides with the
    // .NET Framework System.dll Queue<T> (CS0433). List<T> lives only in mscorlib
    // and compiles cleanly under net48 + Unity mscorlib.
    private readonly List<string> _lines = new();
    private readonly object _gate = new();
    private readonly object _archiveGate = new();
    private int _lineHead;
    private readonly SortedDictionary<string, double> _metrics = new(StringComparer.Ordinal);
    private readonly Dictionary<string, NetworkQualityFinding> _quality = new(StringComparer.Ordinal);

    public void Quality(NetworkQualityFinding finding)
    {
        lock (_gate)
            _quality[finding.Stage + "/" + finding.Category + "/" + finding.Scope] = finding;
    }

    public void PublishQualityLedger()
    {
        NetworkQualityFinding[] findings;

        lock (_gate)
            findings =
                _quality.Values
                    .OrderBy(finding => finding.Stage)
                    .ThenBy(finding => finding.Category)
                    .ThenBy(finding => finding.Scope)
                    .ToArray();

        Artifact("quality-ledger", archive => archive.WriteJson("quality-ledger.json", new
        {
            schemaVersion = 1,
            releaseState = "RequiresReview",
            humanVisualAndTrafficAcceptance = "Unavailable",
            findings,
            summary = findings.GroupBy(finding => (finding.Stage, finding.Category)).Select(group => new
            {
                stage = group.Key.Stage,
                category = group.Key.Category,
                scopes = group.Count(),
                checkedObservations = group.Sum(finding => (long)finding.Checked),
                failedObservations = group.Sum(finding => (long)finding.Failed),
                unavailableObservations = group.Sum(finding => (long)finding.Unavailable),
                expectedObservations = group.Sum(finding => (long)finding.Expected),
                intentionallyExcluded = group.Sum(finding => (long)finding.IntentionallyExcluded),
                notApplicable = group.Sum(finding => (long)finding.NotApplicable),
                worst = group.Max(finding => finding.Worst),
                affectedLengthMeters = group.Sum(finding => finding.AffectedLengthMeters),
                observationScopesMayOverlap = group.Key.Stage == "native-unit",
                subdivisionCountIsNotAffectedLength = true
            }).ToArray(),
            evidenceCompleteness = archive.CaptureStatus(),
            generationCompletionIsNotAcceptance = true
        }));
    }

    private RunArchive? _archive;
    private readonly List<string> _archivePending = new();
    // Lines waiting for the main thread to pass them to the game's log (DrainToGameLog).
    // Shared by every ImportLog, so a log the UI system doesn't own still reaches the game.
    private static readonly List<(string Level, string Line)> _gameLogPending = new();
    private static readonly object _gameLogGate = new();
    private static int _gameLogFailures;
    private const int GameLogPendingLimit = 500;
    private const int GameLogLinesPerFrame = 20;
    private string? _archiveFailure;
    private bool _artifactWarning;
    private bool _finalCapture;
    private double _artifactSeconds;
    private volatile bool _diagnosticsEnabled;

    // Diagnostic files are written in call order by one background thread, so the game's
    // main thread and the preparation pay only for collecting the records (report 114 S1).
    // Callers still enumerate their records themselves: some count while enumerating, and
    // ECS reads must stay on the main thread. A list, not Queue<T>, for the reason above.
    private readonly object _writerGate = new();
    private readonly List<(string Label, Action<RunArchive> Write)> _writes = new();
    private int _writeHead;
    private Thread? _writer;
    private const int MaximumQueuedWrites = 64;

    public bool DiagnosticsEnabled
    {
        get => _diagnosticsEnabled;
        set => _diagnosticsEnabled = value;
    }

    public string? RunFolder => _archive?.DirectoryPath;

    // Replays and the gate fail on an error a normal import would recover from.
    public bool StrictRecovery { get; init; }

    private readonly int[] _legacyRepeats = new int[3];
    private const int MaximumRepeatKinds = 256;
    private readonly Dictionary<string, int> _repeatKinds = new();
    private int _suppressedLines;
    private long _lastArchiveFlush;

    public ImportTelemetry? Telemetry { get; private set; }

    public void CloseArchive(string? state = null)
    {
        DrainWrites();

        lock (_archiveGate)
        {
            // At once, not queued: the archive closes below.
            if (DiagnosticsEnabled)
                WriteArchiveNow(state);
            else
                FlushArchive(state);

            lock (_gate)
            {
                _archive = null;
                _archivePending.Clear();
            }
        }
    }

    public void BeginArchive(ImportRequest request)
    {
        DiagnosticsEnabled = request.DiagnosticsEnabled;
        Telemetry =
            new ImportTelemetry(
                new ImportEpoch(request.MapSessionId, request.Generation, request.WorldSequenceNumber),
                DiagnosticsEnabled);

        Event(TelemetryEvent.RunStart, TelemetryReason.None, "Import started.");

        // An earlier run's files finish before this run's archive replaces it.
        DrainWrites();

        // Called on the game thread: workers never read Unity's persistent-data path.
        lock (_archiveGate)
            lock (_gate)
            {
                _archive = null;
                _archivePending.Clear();
                _archiveFailure = null;
                _artifactWarning = false;
                _finalCapture = false;
                _artifactSeconds = 0;
                _quality.Clear();

                if (!DiagnosticsEnabled)
                    return;

                try
                {
                    var root =
                        System.IO.Path.Combine(
                            UnityEngine.Application.persistentDataPath,
                            "ModsData",
                            Mod.Id,
                            "diagnostics",
                            "runs");

                    var loaded =
                        AppDomain.CurrentDomain
                            .GetAssemblies()
                            .Where(assembly => !assembly.IsDynamic)
                            .Select(assembly => assembly.GetName())
                            .OrderBy(name => name.Name, StringComparer.Ordinal)
                            .ToArray();

                    string assemblyPath;

                    try
                    {
                        assemblyPath = typeof(ImportLog).Assembly.Location;
                    }
                    catch
                    {
                        assemblyPath = "";
                    }

                    _archive = new RunArchive(root, request, assemblyPath, new
                    {
                        modBuildMvid = typeof(ImportLog).Assembly.ManifestModule.ModuleVersionId,
                        runId = Telemetry.RunId,
                        epoch = Telemetry.Epoch,
                        engineBuildMvid = typeof(Game.Net.Node).Assembly.ManifestModule.ModuleVersionId,
                        gameVersion = UnityEngine.Application.version,
                        unityVersion = UnityEngine.Application.unityVersion,
                        operatingSystem = Environment.OSVersion.VersionString,
                        managedRuntime = Environment.Version.ToString(),
                        systemMemoryMb = UnityEngine.SystemInfo.systemMemorySize,
                        loadedAssemblyCount = loaded.Length,
                        loadedAssemblies = loaded
                            .Take(512)
                            .Select(otherName => new { name = otherName.Name, version = otherName.Version?.ToString() })
                            .ToArray(),
                        assembliesTruncated = loaded.Length > 512,
                        assemblyInventoryNotModProvenance = true
                    });
                }
                catch (Exception exception)
                {
                    _archiveFailure = exception.Message;
                }
            }

        if (_archive != null)
            Info("Diagnostic history: " + _archive.DirectoryPath);

        FlushArchive("preparing");
    }

    public void FlushArchive(string? state = null, bool force = false)
    {
        if (!DiagnosticsEnabled)
        {
            if (state != null
                && (state == "completed"
                    || state.Contains("failed")
                    || state.Contains("stopped")
                    || state.Contains("unavailable")
                    || state.Contains("interrupted")))
                ReportSuppressed();

            return;
        }

        var now = System.Diagnostics.Stopwatch.GetTimestamp();

        lock (_gate)
        {
            if (!force
                && state == null
                && _lastArchiveFlush != 0
                && (now - _lastArchiveFlush) / (double)System.Diagnostics.Stopwatch.Frequency < 10)
                return;

            _lastArchiveFlush = now;
        }

        // A periodic or forced flush, or a step on the way (preparing, prepared, applying), is
        // queued behind the files before it: written after them, in order, without the import
        // waiting. A final state waits for every queued file and then records the run.
        if (state is null or "preparing" or "prepared" or "applying")
        {
            string[] queuedLines;
            SortedDictionary<string, double> queuedMetrics;

            string? unavailable;

            lock (_gate)
            {
                queuedLines = _archivePending.ToArray();
                _archivePending.Clear();
                queuedMetrics = new SortedDictionary<string, double>(_metrics, StringComparer.Ordinal);
                unavailable = _archiveFailure;
                _archiveFailure = null;
            }

            Artifact("archive-checkpoint", archive => archive.Write(queuedLines, queuedMetrics, state));

            if (unavailable != null)
                Warn("Diagnostic history unavailable; import continues normally: " + unavailable);

            return;
        }

        DrainWrites();
        WriteArchiveNow(state);
    }

    // Records the run's events, metrics and state at once. Serialized with the archive's
    // other writes; disk I/O never holds the metrics/progress lock.
    private void WriteArchiveNow(string? state)
    {
        string? failure = null;

        lock (_archiveGate)
        {
            RunArchive? archive;
            string[] lines;
            SortedDictionary<string, double> metrics;

            lock (_gate)
            {
                archive = _archive;
                lines = _archivePending.ToArray();
                _archivePending.Clear();
                metrics = new SortedDictionary<string, double>(_metrics, StringComparer.Ordinal);
                failure = _archiveFailure;
                _archiveFailure = null;
            }

            try
            {
                archive?.Write(lines, metrics, state);
            }
            catch (Exception exception)
            {
                archive?.RecordFailure("archive-checkpoint", exception);
                Count("Diagnostic archive flush failures");

                if (!_artifactWarning)
                {
                    _artifactWarning = true;
                    failure = exception.Message;
                }
            }
        }

        if (failure != null)
            Warn("Diagnostic history unavailable; import continues normally: " + failure);
    }

    private void Artifact(string label, Action<RunArchive> write)
    {
        if (!DiagnosticsEnabled)
            return;

        // A file written from within another file's write goes straight to disk.
        if (Thread.CurrentThread == _writer)
        {
            WriteArtifact(label, write);
            return;
        }

        var waited = System.Diagnostics.Stopwatch.StartNew();

        lock (_writerGate)
        {
            // Bounded, so a slow disk holds the caller back instead of memory growing.
            while (_writes.Count - _writeHead >= MaximumQueuedWrites)
                Monitor.Wait(_writerGate);

            _writes.Add((label, write));

            if (_writer == null)
            {
                _writer = new Thread(WriteArtifacts) { IsBackground = true, Name = "Import Earth diagnostics" };
                _writer.Start();
            }

            Monitor.PulseAll(_writerGate);
        }

        if (waited.Elapsed.TotalSeconds > .01)
            AddWriterWait(waited.Elapsed.TotalSeconds);
    }

    private void WriteArtifacts()
    {
        while (true)
        {
            (string Label, Action<RunArchive> Write) next;

            lock (_writerGate)
            {
                while (_writes.Count == _writeHead)
                    Monitor.Wait(_writerGate);

                next = _writes[_writeHead];
            }

            WriteArtifact(next.Label, next.Write);

            lock (_writerGate)
            {
                _writes[_writeHead++] = default;

                if (_writeHead == _writes.Count)
                {
                    _writes.Clear();
                    _writeHead = 0;
                }

                Monitor.PulseAll(_writerGate);
            }
        }
    }

    // Waits until every queued diagnostic file is written. The writer itself never waits.
    // Time spent here and in a full queue is reported, since it is time the import waited.
    private void DrainWrites()
    {
        var clock = System.Diagnostics.Stopwatch.StartNew();

        lock (_writerGate)
        {
            if (Thread.CurrentThread == _writer)
                return;

            while (_writes.Count > _writeHead)
                Monitor.Wait(_writerGate);
        }

        AddWriterWait(clock.Elapsed.TotalSeconds);
    }

    private double _writerWaitSeconds;

    private void AddWriterWait(double seconds)
    {
        if (seconds < .001)
            return;

        double total;

        lock (_writerGate)
            total = _writerWaitSeconds += seconds;

        Time("Diagnostic writer waits", total);
    }

    // Records are collected on the calling thread; a failure is recorded in order.
    private object[]? Collect(string label, IEnumerable<object> records)
    {
        lock (_gate)
            if (_archive == null)
                return null;

        try
        {
            return records.ToArray();
        }
        catch (Exception error)
        {
            Artifact(label, _ => System.Runtime.ExceptionServices.ExceptionDispatchInfo.Capture(error).Throw());

            return null;
        }
    }

    // Whether an intermediate capture is sampled, decided at the call so suppressed
    // captures are never enumerated.
    private bool Sampled(string name)
    {
        RunArchive? archive;

        lock (_gate)
            archive = _archive;

        return archive != null && archive.CaptureIntermediate(name);
    }

    private void WriteArtifact(string label, Action<RunArchive> write)
    {
        string? failure = null;
        var clock = System.Diagnostics.Stopwatch.StartNew();
        double seconds;

        lock (_archiveGate)
        {
            RunArchive? archive;

            lock (_gate)
                archive = _archive;

            if (archive == null)
                return;

            try
            {
                write(archive);
            }
            catch (Exception error)
            {
                if (!_artifactWarning)
                {
                    _artifactWarning = true;
                    failure = error.GetBaseException().Message;
                }

                archive.RecordFailure(label, error);
                Count("Diagnostic artifact failures");
            }

            _artifactSeconds += clock.Elapsed.TotalSeconds;
            seconds = _artifactSeconds;
        }

        Time("Diagnostic artifacts total", seconds);

        if (failure != null)
            Warn(
                $"Diagnostic artifact unavailable ({label}): {failure}. Import rules unchanged; disk logging is not required for generation.");
    }

    public void Checkpoint(string phase, object details)
    {
        if (!DiagnosticsEnabled)
            return;

        // Never reflect/serialize legacy anonymous graphs. Preserve epoch and
        // stage even for not-yet-migrated callers; -1 means unknown, not zero.
        var epoch = Telemetry?.Epoch ?? default;
        var intent = details as RunArchive.NativeOperationIntent ?? new RunArchive.NativeOperationIntent(
            epoch.MapSession, epoch.Generation, epoch.WorldSequenceNumber, "StageIntentOnly", -1, -1, -1);

        // Written at once, not queued: it must be on disk before the native operation starts.
        // It has its own lock rather than the archive's, so the game thread never waits
        // behind a large file the background writer is saving (Terrain apply rose from
        // 2.6 s to 14-22 s on 11-13 while it did).
        RunArchive? archive;

        lock (_gate)
            archive = _archive;

        if (archive == null)
            return;

        try
        {
            lock (_checkpointGate)
                archive.Checkpoint(phase, intent);
        }
        catch (Exception error)
        {
            // Recorded in order with the other files; the import continues either way.
            Artifact("checkpoint", _ => System.Runtime.ExceptionServices.ExceptionDispatchInfo.Capture(error).Throw());
        }
    }

    private readonly object _checkpointGate = new();

    public void DiagnosticRecords(
        string name,
        IEnumerable<object> records,
        long maximumBytes = 8L * 1024 * 1024,
        bool critical = false,
        bool final = false)
    {
        if (!DiagnosticsEnabled || !critical && !final && !Sampled(name))
            return;

        var collected = Collect(name, records);

        if (collected == null)
            return;

        Artifact(name, archive =>
        {
            var result = archive.WriteRecords(name, collected, maximumBytes, critical: critical, final: final);
            Diagnostic(
                $"ARTIFACT file={result.File}; attemptFile={result.AttemptFile}; records={result.Records}; bytes={result.Bytes}; truncated={result.Truncated}; preservedPrevious={result.PreservedPrevious}; criticalReserve={critical}; localOnly=true");
        });
    }

    public void DiagnosticRecordChunks(string name, IEnumerable<object> records)
    {
        if (!DiagnosticsEnabled || !Sampled(name))
            return;

        var collected = Collect(name, records);

        if (collected == null)
            return;

        Artifact(name, archive =>
        {
            var result = archive.WriteRecordChunks(name, collected, final: true);
            Diagnostic(
                $"ARTIFACT CHUNKS manifest={result.Manifest}; attemptManifest={result.AttemptManifest}; preservedPrevious={result.PreservedPrevious}; chunks={result.Chunks.Length}; records={result.Records}; truncated={result.Truncated}; boundedRunByteBudget=true; localOnly=true");
        });
    }

    public void ExpectNativeCapture(IReadOnlyList<PreparedSegment> sections)
    {
        if (!DiagnosticsEnabled)
            return;

        var identities = sections.Select(DiagnosticArtifacts.SectionIdentity).ToArray();
        Artifact("native-coverage-expectations", archive => archive.ExpectNativeSections(identities));
    }

    // True when the records were collected and queued; a later write failure is recorded
    // in capture-completeness.json like any other.
    public bool DiagnosticCompressedRecords(string name, IEnumerable<object> records)
    {
        if (!DiagnosticsEnabled)
            return false;

        var collected = Collect(name, records);

        if (collected == null)
            return false;

        Artifact(name, archive =>
        {
            var result = archive.WriteCompressedRecords(name, collected);
            Diagnostic(
                $"ARTIFACT REPLAY file={result.File}; records={result.Records}; compressedBytes={result.Bytes}; complete=true; encoding=gzip-ndjson; finalBudget=true");
        });

        return true;
    }

    // Arrays are copied: the pipeline may change its own grids after this call.
    public void DiagnosticHeights(string name, ushort[] heights)
    {
        if (!DiagnosticsEnabled)
            return;

        var copy = (ushort[])heights.Clone();
        Artifact(name, archive => archive.WriteHeights(name, copy));
    }

    public void DiagnosticHeights(string name, float[] heights, float scale)
    {
        if (!DiagnosticsEnabled)
            return;

        var copy = (float[])heights.Clone();
        Artifact(name, archive => archive.WriteHeights(name, copy, scale));
    }

    public void DiagnosticBytes(string name, byte[] values, int maximumBytes)
    {
        if (!DiagnosticsEnabled)
            return;

        var copy = (byte[])values.Clone();
        Artifact(name, archive => archive.WriteBytes(name, copy, maximumBytes));
    }

    public void DiagnosticJson(string name, object value) => Artifact(name, archive => archive.WriteJson(name, value));

    public void DiagnosticSvg(string name, string svg) => Artifact(name, archive => archive.WriteSvg(name, svg));

    // Exact replay records use the protected, bounded replay reserve. Capture
    // completeness remains in each manifest; a partial archive is never a fixture.
    public void CaptureReplayBoundary(string name, IEnumerable<object> records)
    {
        if (!DiagnosticsEnabled)
            return;

        var collected = Collect(name, records);

        if (collected != null)
            Artifact(
                name,
                archive => archive.WriteRecordChunks("offline-" + name + ".ndjson", collected, final: true, compress: true));
    }

    public void ObserveNativeCapture(
        string stem,
        IEnumerable<(string Identity, bool Resolved)> sections,
        bool requireGeometryOutcomes = false)
    {
        if (!DiagnosticsEnabled)
            return;

        var observed = sections.ToArray();
        Artifact(
            "native-coverage-observation",
            archive => archive.ObserveNativeSections(stem, observed, requireGeometryOutcomes));
    }

    public void CapturePrepared(PreparedImport prepared)
    {
        if (!DiagnosticsEnabled)
            return;

        lock (_archiveGate)
        {
            if (_finalCapture)
                return;

            _finalCapture = true;
        }

        var heights = (float[])prepared.Heights.Clone();

        Artifact("prepared-plan", archive =>
        {
            var records = prepared.Segments.Select(segment => (object)new
            {
                segment.WayId,
                segment.StartNodeId,
                segment.EndNodeId,
                segment.SourceWayIds,
                segment.SourceNodeAliases,
                kind = segment.Kind.ToString(),
                segment.Classification,
                segment.Reference,
                segment.CorridorId,
                segment.Lanes,
                segment.OneWay,
                segment.Bridge,
                segment.Tunnel,
                segment.Layer,
                segment.NativeWidth,
                segment.NativeAssetKey,
                segment.NativeUnitId,
                segment.NativeCenterlineCurvatureLimit,
                a = DiagnosticArtifacts.Point(segment.CourseStart),
                b = DiagnosticArtifacts.Point(segment.CourseControl1),
                c = DiagnosticArtifacts.Point(segment.CourseControl2),
                d = DiagnosticArtifacts.Point(segment.CourseEnd),
                horizontalEmissionReference = DiagnosticArtifacts.Curve(segment.HorizontalEmissionReference),
                nodeStart = DiagnosticArtifacts.Point(segment.Start),
                nodeEnd = DiagnosticArtifacts.Point(segment.End),
                segment.StartElevation,
                segment.EndElevation
            });

            var result =
                archive.WriteRecordChunks("final-prepared-network.ndjson", records, final: true, compress: true);

            var assets =
                archive.WriteRecords(
                    "final-native-asset-contracts.ndjson",
                    prepared.NativeAssets.Cast<object>(),
                    1024L * 1024,
                    final: true);

            if (prepared.Design?.Source != null)
                archive.WriteRecordChunks(
                    "immutable-source-network.ndjson",
                    DiagnosticArtifacts.SourceRecords(prepared.Design.Source),
                    final: true,
                    compress: true);

            archive.WriteHeights("final-design-heightmap.u16", heights, prepared.HeightScaleMeters);
            archive.WriteJson("final-design.json", new
            {
                schemaVersion = 1,
                fingerprint = prepared.Design?.Fingerprint,
                quality = prepared.Design?.Quality.ToString() ?? "RequiresReview",
                sections = prepared.Segments.Count,
                nativeAssets = prepared.NativeAssets.Count,
                sourceChains = prepared.Design?.Source?.Chains.Count,
                fingerprintScope = "final-section-anchors-curves-provenance-and-native-asset-contracts",
                networkManifest = result.Manifest,
                assetsFile = assets.File,
                networkTruncated = result.Truncated,
                assetsTruncated = assets.Truncated,
                exactFloatDeckFrozen = prepared.Design != null,
                nativeAcceptanceVerified = false,
                terrainFile = "final-design-heightmap.u16",
                heightSamples = prepared.Heights.Length,
                heightScaleMeters = prepared.HeightScaleMeters,
                mapWidthMeters = prepared.Request.MapSize.WidthMeters,
                terrainEncoding = "little-endian uint16, row-major z/x; local centred metres; metres=value/65535*heightScaleMeters"
            });

            Diagnostic(
                $"PREPARED ARTIFACT manifest={result.Manifest}; records={result.Records}/{prepared.Segments.Count}; truncated={result.Truncated}; compactFinalCurves=true; reservedFinalBudget=true; nativeAppearanceNotCertified=true");
        });
    }

    public void ResetMetrics()
    {
        lock (_gate)
        {
            _metrics.Clear();
            Array.Clear(_legacyRepeats, 0, _legacyRepeats.Length);
            _repeatKinds.Clear();
            _suppressedLines = 0;
            _lastArchiveFlush = 0;
        }
    }

    public void Count(string name, int count = 1)
    {
        lock (_gate)
        {
            _metrics.TryGetValue(name, out var value);
            _metrics[name] = value + count;
        }
    }

    public void Time(string name, double seconds)
    {
        lock (_gate)
            _metrics[name + " time (s)"] = seconds;
    }

    public void AddDuration(string name, double seconds)
    {
        lock (_gate)
        {
            var key = name + " time (s)";
            _metrics.TryGetValue(key, out var old);
            _metrics[key] = old + seconds;
        }
    }

    public double Metric(string name)
    {
        lock (_gate)
            return _metrics.TryGetValue(name, out var value) ? value : 0;
    }

    public string Metrics()
    {
        lock (_gate)
            return string.Join(" • ", _metrics.Select(item => $"{item.Key}: {item.Value:0.##}"));
    }

    public event Action? Changed;

    public void Info(string message) => Add("INFO", message);

    // Bounded per-import evidence for verifying a run from logs alone: exempt from
    // the normal-mode repeat filter and mirrored to Player.log, which can omit the
    // Colossal logger's INFO level. Callers keep the line count small.
    public void Evidence(string message, bool warning = false)
    {
        Add(warning ? "WARN" : "INFO", message, structured: true);
        UnityEngine.Debug.Log("[IEL] " + message);
    }

    public void Warn(string message) => Add("WARN", message);

    public void Error(string message) => Add("ERROR", message);

    public void Event(TelemetryEvent kind, TelemetryReason reason, string message)
    {
        if (kind == TelemetryEvent.Progress && Telemetry?.ProgressDue() == false)
            return;

        if (kind == TelemetryEvent.Failure && Telemetry?.First(kind, reason) == false)
            return;

        Add(kind == TelemetryEvent.Failure ? "WARN" : "INFO", message, structured: true);
    }

    public void BeginStage(TelemetryStage stage)
    {
        if (Telemetry == null || Telemetry.Stage == stage)
            return;

        Event(TelemetryEvent.StageEnd, TelemetryReason.None, "Stage complete: " + Telemetry.Stage);
        Telemetry.SetStage(stage);
        Event(TelemetryEvent.StageStart, TelemetryReason.None, "Stage: " + stage);
        FlushArchive();
    }

    // Player.log can omit the Colossal logger's INFO level. Mirror only bounded
    // diagnostic summaries, not every download or geometry message.
    public void Diagnostic(string message)
    {
        if (!DiagnosticsEnabled)
            return;

        var tagged = "[IEL-DIAG] " + message;
        Info(tagged);
        UnityEngine.Debug.Log(tagged);
    }

    public string Snapshot(int maximumLines = 120)
    {
        lock (_gate)
        {
            var count = Math.Min(_lines.Count, Math.Max(0, maximumLines));
            var snapshot = new string[count];

            for (var i = 0; i < count; i++)
                snapshot[i] = _lines[(_lineHead + _lines.Count - count + i) % _lines.Count];

            return string.Join("\n", snapshot);
        }
    }

    private void Add(string level, string message, bool structured = false)
    {
        // Keep important warnings/errors, but show only a few examples of a
        // repeated per-source condition in normal mode. Full mode is unfiltered.
        if (!DiagnosticsEnabled && !structured && level != "ERROR")
        {
            // A message keeps three examples per kind, its text with digits
            // removed (so "way 123: …" and "way 456: …" are one kind). One
            // global count let three startup notices hide every later warning,
            // including why vegetation stopped. Kinds are bounded; beyond the
            // bound, messages share the fixed overflow buckets.
            var index = level == "WARN" ? 1 : 0;
            var kind = level + ":" + new string(message.Take(60).Where(c => !char.IsDigit(c)).ToArray());

            lock (_gate)
            {
                if (_repeatKinds.TryGetValue(kind, out var seen) || _repeatKinds.Count < MaximumRepeatKinds)
                {
                    _repeatKinds[kind] = seen + 1;

                    if (seen >= 3)
                    {
                        _suppressedLines++;
                        return;
                    }
                }
                else if (_legacyRepeats[index]++ >= 3)
                {
                    _suppressedLines++;
                    return;
                }
            }
        }

        var line =
            $"{DateTime.Now:HH:mm:ss} [{level}]"
                + (Telemetry == null ? "" : " [RunId=" + Telemetry.RunId + "]")
                + " "
                + message;

        lock (_gate)
        {
            if (_lines.Count < 500)
                _lines.Add(line);
            else
            {
                _lines[_lineHead] = line;
                _lineHead = (_lineHead + 1) % 500;
            }

            if (_archive != null && _archivePending.Count < 500)
                _archivePending.Add(line);

            // With diagnostics on, every line is already in events.log; the game's log gets
            // errors only. Its logger failed even on the main thread (one warning on DC,
            // 9 Nov), raising the error dialog, so the fewer lines it writes the better.
            if (level == "ERROR" || !DiagnosticsEnabled)
            {
                bool queued;

                lock (_gameLogGate)
                {
                    queued = _gameLogPending.Count < GameLogPendingLimit;

                    if (queued)
                        _gameLogPending.Add((level, line));
                }

                if (!queued)
                {
                    _metrics.TryGetValue("Game log lines dropped", out var dropped);
                    _metrics["Game log lines dropped"] = dropped + 1;
                }
            }

            // Failures from earlier drains, reported in this run's metrics.
            var failures = Interlocked.Exchange(ref _gameLogFailures, 0);

            if (failures > 0)
            {
                _metrics.TryGetValue("Game log writes failed", out var failed);
                _metrics["Game log writes failed"] = failed + failures;
            }
        }

        Changed?.Invoke();
    }

    // The game's logger is not safe to call from worker threads. Imports log from
    // many, and those calls ended in a NullReferenceException inside
    // UnityLogger.Internal_WriteStream that raised the game's error dialog over
    // every site shot (report 112). Lines wait in _gameLogPending for the main
    // thread, which calls this once per frame.
    public static void DrainToGameLog()
    {
        (string Level, string Line)[] lines;

        lock (_gameLogGate)
        {
            if (_gameLogPending.Count == 0)
                return;

            var count = Math.Min(GameLogLinesPerFrame, _gameLogPending.Count);
            lines = _gameLogPending.GetRange(0, count).ToArray();
            _gameLogPending.RemoveRange(0, count);
        }

        var failures = 0;

        foreach (var (level, line) in lines)
        {
            try
            {
                if (level == "ERROR")
                    Mod.Log.Error(line);
                else if (level == "WARN")
                    Mod.Log.Warn(line);
                else
                    Mod.Log.Info(line);
            }
            catch (Exception)
            {
                // A logging failure must never surface as a game error; the line
                // is still in the panel and events.log.
                failures++;
            }
        }

        if (failures > 0)
            Interlocked.Add(ref _gameLogFailures, failures);
    }

    public void ReportSuppressed()
    {
        int count;

        lock (_gate)
        {
            count = _suppressedLines;
            _suppressedLines = 0;
        }

        if (count > 0)
            Event(
                TelemetryEvent.Progress,
                TelemetryReason.LegacyOther,
                $"Omitted {count} repeated log entries. Enable diagnostics before preparation for full per-source details.");
    }
}
