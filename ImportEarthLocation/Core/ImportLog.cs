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
    private readonly HashSet<string> _artifactWarnings = new(StringComparer.Ordinal);
    private double _artifactSeconds;

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
        // Called on the game thread: workers never read Unity's persistent-data path.
        lock (_archiveGate) lock (_gate)
        {
            _archive = null; _archivePending.Clear(); _archiveFailure = null; _artifactWarnings.Clear(); _artifactSeconds=0;
            try
            {
                var root = System.IO.Path.Combine(UnityEngine.Application.persistentDataPath, "ModsData", Mod.Id, "diagnostics", "runs");
                var loaded=AppDomain.CurrentDomain.GetAssemblies().Where(a=>!a.IsDynamic).Select(a=>a.GetName()).OrderBy(a=>a.Name,StringComparer.Ordinal).ToArray();
                _archive = new RunArchive(root, request, typeof(ImportLog).Assembly.Location,new
                {
                    modBuildMvid=typeof(ImportLog).Assembly.ManifestModule.ModuleVersionId,
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
        string? failure = null;
        var clock=System.Diagnostics.Stopwatch.StartNew();double seconds;
        lock (_archiveGate)
        {
            RunArchive? archive; lock (_gate) archive = _archive;
            if (archive == null) return;
            try { write(archive); }
            catch (Exception error) { if (_artifactWarnings.Add(label)) failure = error.GetBaseException().Message; }
            _artifactSeconds+=clock.Elapsed.TotalSeconds;seconds=_artifactSeconds;
        }
        Time("Diagnostic artifacts total",seconds);
        if (failure != null) Warn($"Diagnostic artifact unavailable ({label}): {failure}. Import rules unchanged; disk logging is not required for generation.");
    }

    public void Checkpoint(string phase, object details)
    {
        Artifact("checkpoint", archive => archive.Checkpoint(phase, details));
        FlushArchive();
    }

    public void DiagnosticRecords(string name, IEnumerable<object> records, long maximumBytes = 8L * 1024 * 1024)
        => Artifact(name, archive =>
        {
            var result = archive.WriteRecords(name, records, maximumBytes);
            Diagnostic($"ARTIFACT file={result.File}; records={result.Records}; bytes={result.Bytes}; truncated={result.Truncated}; localOnly=true");
        });

    public void CaptureGeometry(string stage, IReadOnlyList<PreparedSegment> sections, ushort[] heights, float scale, double width)
        => Artifact(stage, archive =>
        {
            var result = DiagnosticArtifacts.CaptureGeometry(archive, stage, sections, heights, scale, width);
            Diagnostic($"GEOMETRY ARTIFACT stage={stage}; sections={result.Records}/{sections.Count}; truncated={result.Truncated}; actualHeightRaster=true; nativeRenderingNotCaptured=true");
        });

    public void CapturePrepared(PreparedImport prepared)
        => Artifact("prepared-plan", archive => { DiagnosticArtifacts.CapturePrepared(archive, prepared); Diagnostic("PREPARED ARTIFACTS network/height/water/vegetation/surface; see per-file truncation metadata; localOnly=true"); });

    public void ResetMetrics() { lock (_gate) _metrics.Clear(); }
    public void Clear() { lock (_gate) { _lines.Clear(); _lineHead=0; _metrics.Clear(); } Changed?.Invoke(); }
    public void Count(string name, int count = 1) { lock (_gate) { _metrics.TryGetValue(name, out var value); _metrics[name] = value + count; } }
    public void Time(string name, double seconds) { lock (_gate) _metrics[name + " time (s)"] = seconds; }
    public double Metric(string name) { lock (_gate) return _metrics.TryGetValue(name, out var value) ? value : 0; }
    public string Metrics() { lock (_gate) return string.Join(" • ", _metrics.Select(item => $"{item.Key}: {item.Value:0.##}")); }

    public event Action? Changed;

    public void Info(string message) => Add("INFO", message);
    public void Warn(string message) => Add("WARN", message);
    public void Error(string message) => Add("ERROR", message);

    // Player.log can omit the Colossal logger's INFO level. Mirror only bounded
    // diagnostic summaries, not every download or geometry message.
    public void Diagnostic(string message)
    {
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

    private void Add(string level, string message)
    {
        var line = $"{DateTime.Now:HH:mm:ss} [{level}] {message}";
        lock (_gate)
        {
            if(_lines.Count<500)_lines.Add(line);
            else{_lines[_lineHead]=line;_lineHead=(_lineHead+1)%500;}
            if (_archive != null) _archivePending.Add(line);
        }

        if (level == "ERROR") Mod.Log.Error(line);
        else if (level == "WARN") Mod.Log.Warn(line);
        else Mod.Log.Info(line);
        Changed?.Invoke();
    }
}
