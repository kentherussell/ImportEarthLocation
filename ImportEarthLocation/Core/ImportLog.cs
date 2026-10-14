using System;
using System.Collections.Generic;
using System.Linq;

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
    private RunArchive? _archive;
    private readonly List<string> _archivePending = new();
    private string? _archiveFailure;
    private bool _artifactWarning;
    private bool _finalCapture;
    private bool _failureCapture;
    private double _artifactSeconds;
    private volatile bool _diagnosticsEnabled;
    public bool DiagnosticsEnabled { get=>_diagnosticsEnabled; set=>_diagnosticsEnabled=value; }
    private readonly int[] _legacyRepeats = new int[3];
    private int _suppressedLines;
    private long _archiveDrops;
    private long _lastArchiveFlush;
    public ImportTelemetry? Telemetry { get; private set; }

    public void CloseArchive(string? state = null)
    {
        lock (_archiveGate)
        {
            FlushArchive(state);
            lock (_gate) { _archive = null; _archivePending.Clear(); }
        }
    }

    public void BeginArchive(ImportRequest request)
    {
        DiagnosticsEnabled=request.DiagnosticsEnabled;
        Telemetry = new ImportTelemetry(new ImportEpoch(request.MapSessionId, request.Generation, request.WorldSequenceNumber), DiagnosticsEnabled);
        Event(TelemetryEvent.RunStart, TelemetryReason.None, "Import started.");
        // Called on the game thread: workers never read Unity's persistent-data path.
        lock (_archiveGate) lock (_gate)
        {
            _archive = null; _archivePending.Clear(); _archiveFailure = null; _artifactWarning=false; _finalCapture=false; _failureCapture=false; _artifactSeconds=0;
            if(!DiagnosticsEnabled)return;
            try
            {
                var root = System.IO.Path.Combine(UnityEngine.Application.persistentDataPath, "ModsData", Mod.Id, "diagnostics", "runs");
                var loaded=AppDomain.CurrentDomain.GetAssemblies().Where(a=>!a.IsDynamic).Select(a=>a.GetName()).OrderBy(a=>a.Name,StringComparer.Ordinal).ToArray();
                string assemblyPath;try{assemblyPath=typeof(ImportLog).Assembly.Location;}catch{assemblyPath="";}
                _archive = new RunArchive(root, request, assemblyPath,new
                {
                    modBuildMvid=typeof(ImportLog).Assembly.ManifestModule.ModuleVersionId,
                    runId=Telemetry.RunId, epoch=Telemetry.Epoch,
                    engineBuildMvid=typeof(Game.Net.Node).Assembly.ManifestModule.ModuleVersionId,
                    gameVersion=UnityEngine.Application.version,unityVersion=UnityEngine.Application.unityVersion,
                    operatingSystem=Environment.OSVersion.VersionString,managedRuntime=Environment.Version.ToString(),
                    systemMemoryMb=UnityEngine.SystemInfo.systemMemorySize,loadedAssemblyCount=loaded.Length,
                    loadedAssemblies=loaded.Take(512).Select(a=>new {name=a.Name,version=a.Version?.ToString()}).ToArray(),
                    assembliesTruncated=loaded.Length>512,assemblyInventoryNotModProvenance=true
                });
            }
            catch (Exception exception) { _archiveFailure = exception.Message; }
        }
        if (_archive != null) Info("Diagnostic history: " + _archive.DirectoryPath);
        FlushArchive("preparing");
    }

    public void FlushArchive(string? state = null)
    {
        if(!DiagnosticsEnabled)
        {
            if(state!=null&&(state=="completed"||state.Contains("failed")||state.Contains("stopped")||state.Contains("unavailable")||state.Contains("interrupted")))ReportSuppressed();
            return;
        }
        var now = System.Diagnostics.Stopwatch.GetTimestamp();
        lock (_gate)
        {
            if (state == null && _lastArchiveFlush != 0 && (now - _lastArchiveFlush) / (double)System.Diagnostics.Stopwatch.Frequency < 10) return;
            _lastArchiveFlush = now;
        }
        string? failure=null;
        // Serialize archive lifecycle/writes separately from worker logging.
        // Disk I/O must not hold the metrics/progress lock.
        lock (_archiveGate)
        {
            RunArchive? archive;string[] lines;SortedDictionary<string,double> metrics;
            lock (_gate)
            {
                archive=_archive;lines=_archivePending.ToArray();_archivePending.Clear();
                metrics=new SortedDictionary<string,double>(_metrics,StringComparer.Ordinal);
                failure=_archiveFailure;_archiveFailure=null;
            }
            try
            {
                archive?.Write(lines, metrics, state);
            }
            catch (Exception exception)
            {failure=exception.Message;lock (_gate){if(ReferenceEquals(_archive,archive)){_archive=null;_archivePending.Clear();}}}
        }
        if (failure != null) Warn("Diagnostic history unavailable; import continues normally: " + failure);
    }

    private void Artifact(string label, Action<RunArchive> write)
    {
        if(!DiagnosticsEnabled)return;
        string? failure = null;
        var clock=System.Diagnostics.Stopwatch.StartNew();double seconds;
        lock (_archiveGate)
        {
            RunArchive? archive; lock (_gate) archive = _archive;
            if (archive == null) return;
            try { write(archive); }
            catch (Exception error)
            {
                if (!_artifactWarning) { _artifactWarning=true; failure=error.GetBaseException().Message; }
                lock (_gate) { _archive=null; _archivePending.Clear(); }
            }
            _artifactSeconds+=clock.Elapsed.TotalSeconds;seconds=_artifactSeconds;
        }
        Time("Diagnostic artifacts total",seconds);
        if (failure != null) Warn($"Diagnostic artifact unavailable ({label}): {failure}. Import rules unchanged; disk logging is not required for generation.");
    }

    public void Checkpoint(string phase, object details)
    {
        if(!DiagnosticsEnabled)return;
        // Never reflect/serialize legacy anonymous graphs. Preserve epoch and
        // stage even for not-yet-migrated callers; -1 means unknown, not zero.
        var epoch=Telemetry?.Epoch??default;
        var intent=details as RunArchive.NativeOperationIntent??new RunArchive.NativeOperationIntent(
            epoch.MapSession,epoch.Generation,epoch.WorldSequenceNumber,"StageIntentOnly",-1,-1,-1);
        Artifact("checkpoint", archive => archive.Checkpoint(phase, intent));
    }

    public void DiagnosticRecords(string name, IEnumerable<object> records, long maximumBytes = 8L * 1024 * 1024)
        => Artifact(name, archive =>
        {
            var result = archive.WriteRecords(name, records, maximumBytes);
            Diagnostic($"ARTIFACT file={result.File}; records={result.Records}; bytes={result.Bytes}; truncated={result.Truncated}; localOnly=true");
        });

    public void CaptureGeometry(string stage, IReadOnlyList<PreparedSegment> sections, ushort[] heights, float scale, double width)
    {
        // Intermediate whole-network/raster dumps are intentionally removed.
        if (!DiagnosticsEnabled || stage.IndexOf("failure", StringComparison.OrdinalIgnoreCase)<0) return;
        lock (_archiveGate) { if (_failureCapture) return; _failureCapture=true; }
        Artifact(stage, archive =>
        {
            archive.WriteRecords("first-failure-network-sample.ndjson", DiagnosticArtifacts.NetworkRecords(sections).Take(64), 8L*1024*1024, 64);
        });
    }

    public void CapturePrepared(PreparedImport prepared)
    {
        if (!DiagnosticsEnabled) return;
        lock (_archiveGate) { if (_finalCapture) return; _finalCapture=true; }
        Artifact("prepared-plan", archive =>
        {
            archive.WriteRecords("final-network-sample.ndjson", DiagnosticArtifacts.NetworkRecords(prepared.Segments).Take(64), 8L*1024*1024, 64);
            Diagnostic("PREPARED ARTIFACT bounded network sample; not replay-complete; raster and per-site transcripts omitted.");
        });
    }

    public void ResetMetrics() { lock (_gate) { _metrics.Clear();Array.Clear(_legacyRepeats,0,_legacyRepeats.Length);_suppressedLines=0;_archiveDrops=0;_lastArchiveFlush=0; } }
    public void Clear() { lock (_gate) { _lines.Clear(); _lineHead=0; _metrics.Clear(); } Changed?.Invoke(); }
    public void Count(string name, int count = 1) { lock (_gate) { _metrics.TryGetValue(name, out var value); _metrics[name] = value + count; } }
    public void Time(string name, double seconds) { lock (_gate) _metrics[name + " time (s)"] = seconds; }
    public void AddDuration(string name, double seconds) { lock (_gate) { var key=name+" time (s)";_metrics.TryGetValue(key,out var old);_metrics[key]=old+seconds; } }
    public void SetGauge(string name, double value) { lock (_gate) _metrics[name]=value; }
    public void AddCounter(string name, long delta) { lock (_gate) { _metrics.TryGetValue(name,out var old);_metrics[name]=old+delta; } }
    public double Metric(string name) { lock (_gate) return _metrics.TryGetValue(name, out var value) ? value : 0; }
    public string Metrics() { lock (_gate) return string.Join(" • ", _metrics.Select(item => $"{item.Key}: {item.Value:0.##}")); }

    public event Action? Changed;

    public void Info(string message) => Add("INFO", message);
    public void Warn(string message) => Add("WARN", message);
    public void Error(string message) => Add("ERROR", message);
    public void Event(TelemetryEvent kind, TelemetryReason reason, string message)
    {
        if (kind == TelemetryEvent.Progress && Telemetry?.ProgressDue() == false) return;
        if (kind == TelemetryEvent.Failure && Telemetry?.First(kind, reason) == false) return;
        Add(kind == TelemetryEvent.Failure ? "WARN" : "INFO", message, structured:true);
    }
    public void BeginStage(TelemetryStage stage)
    {
        if (Telemetry == null || Telemetry.Stage == stage) return;
        Event(TelemetryEvent.StageEnd, TelemetryReason.None, "Stage complete: "+Telemetry.Stage);
        Telemetry.SetStage(stage);
        Event(TelemetryEvent.StageStart, TelemetryReason.None, "Stage: "+stage);
        FlushArchive();
    }

    // Player.log can omit the Colossal logger's INFO level. Mirror only bounded
    // diagnostic summaries, not every download or geometry message.
    public void Diagnostic(string message)
    {
        if(!DiagnosticsEnabled)return;
        var tagged = "[IEL-DIAG] " + message;
        Info(tagged);
        UnityEngine.Debug.Log(tagged);
    }

    public string Snapshot(int maximumLines = 120)
    {
        lock (_gate)
        {
            var count=Math.Min(_lines.Count,Math.Max(0,maximumLines));
            var snapshot=new string[count];for(var i=0;i<count;i++)snapshot[i]=_lines[(_lineHead+_lines.Count-count+i)%_lines.Count];
            return string.Join("\n", snapshot);
        }
    }

    private void Add(string level, string message, bool structured = false)
    {
        // Keep important warnings/errors, but show only a few examples of a
        // repeated per-source condition in normal mode. Full mode is unfiltered.
        if(!DiagnosticsEnabled&&!structured&&level!="ERROR")
        {
            // Unknown legacy messages share fixed overflow buckets. Typed callers
            // use (stage,event,reason), not a growing normalized-message dictionary.
            var index = level == "WARN" ? 1 : 0;
            lock(_gate){if(_legacyRepeats[index]++>=3){_suppressedLines++;return;}}
        }
        var line = $"{DateTime.Now:HH:mm:ss} [{level}]"+(Telemetry==null?"":" [RunId="+Telemetry.RunId+"]")+" "+message;
        lock (_gate)
        {
            if(_lines.Count<500)_lines.Add(line);
            else{_lines[_lineHead]=line;_lineHead=(_lineHead+1)%500;}
            if (_archive != null) { if(_archivePending.Count<500)_archivePending.Add(line);else _archiveDrops++; }
        }

        if (level == "ERROR") Mod.Log.Error(line);
        else if (level == "WARN") Mod.Log.Warn(line);
        else Mod.Log.Info(line);
        Changed?.Invoke();
    }

    public void ReportSuppressed()
    {
        int count;lock(_gate){count=_suppressedLines;_suppressedLines=0;}
        if(count>0)Event(TelemetryEvent.Progress,TelemetryReason.LegacyOther,$"Omitted {count} repeated log entries. Enable diagnostics before preparation for full per-source details.");
    }
}
