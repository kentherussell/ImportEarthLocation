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
    private readonly Dictionary<string,NetworkQualityFinding> _quality=new(StringComparer.Ordinal);
    public void Quality(NetworkQualityFinding finding)
    {lock(_gate)_quality[finding.Stage+"/"+finding.Category+"/"+finding.Scope]=finding;}
    public void PublishQualityLedger()
    {
        NetworkQualityFinding[] findings;lock(_gate)findings=_quality.Values.OrderBy(f=>f.Stage).ThenBy(f=>f.Category).ThenBy(f=>f.Scope).ToArray();
        Artifact("quality-ledger",archive=>archive.WriteJson("quality-ledger.json",new
        {
            schemaVersion=1,releaseState="RequiresReview",humanVisualAndTrafficAcceptance="Unavailable",
            findings,summary=findings.GroupBy(f=>(f.Stage,f.Category)).Select(g=>new
            {
                stage=g.Key.Stage,category=g.Key.Category,scopes=g.Count(),checkedObservations=g.Sum(f=>(long)f.Checked),
                failedObservations=g.Sum(f=>(long)f.Failed),unavailableObservations=g.Sum(f=>(long)f.Unavailable),
                expectedObservations=g.Sum(f=>(long)f.Expected),intentionallyExcluded=g.Sum(f=>(long)f.IntentionallyExcluded),notApplicable=g.Sum(f=>(long)f.NotApplicable),
                worst=g.Max(f=>f.Worst),affectedLengthMeters=g.Sum(f=>f.AffectedLengthMeters),
                observationScopesMayOverlap=g.Key.Stage=="native-unit",subdivisionCountIsNotAffectedLength=true
            }).ToArray(),
            evidenceCompleteness=archive.CaptureStatus(),generationCompletionIsNotAcceptance=true
        }));
    }
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
            _archive = null; _archivePending.Clear(); _archiveFailure = null; _artifactWarning=false; _finalCapture=false; _failureCapture=false; _artifactSeconds=0;_quality.Clear();
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

    public void FlushArchive(string? state = null,bool force=false)
    {
        if(!DiagnosticsEnabled)
        {
            if(state!=null&&(state=="completed"||state.Contains("failed")||state.Contains("stopped")||state.Contains("unavailable")||state.Contains("interrupted")))ReportSuppressed();
            return;
        }
        var now = System.Diagnostics.Stopwatch.GetTimestamp();
        lock (_gate)
        {
            if (!force&&state == null && _lastArchiveFlush != 0 && (now - _lastArchiveFlush) / (double)System.Diagnostics.Stopwatch.Frequency < 10) return;
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
            {
                archive?.RecordFailure("archive-checkpoint",exception);
                Count("Diagnostic archive flush failures");
                if(!_artifactWarning){_artifactWarning=true;failure=exception.Message;}
            }
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
                archive.RecordFailure(label,error);
                Count("Diagnostic artifact failures");
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

    public void DiagnosticRecords(string name, IEnumerable<object> records, long maximumBytes = 8L * 1024 * 1024, bool critical = false, bool final = false)
        => Artifact(name, archive =>
        {
            if(!critical&&!final&&!archive.CaptureIntermediate(name))return;
            var result = archive.WriteRecords(name, records, maximumBytes,critical:critical,final:final);
            Diagnostic($"ARTIFACT file={result.File}; attemptFile={result.AttemptFile}; records={result.Records}; bytes={result.Bytes}; truncated={result.Truncated}; preservedPrevious={result.PreservedPrevious}; criticalReserve={critical}; localOnly=true");
        });

    public void DiagnosticRecordChunks(string name,IEnumerable<object> records)
        => Artifact(name,archive=>
        {
            if(!archive.CaptureIntermediate(name))return;
            var result=archive.WriteRecordChunks(name,records,final:true);
            Diagnostic($"ARTIFACT CHUNKS manifest={result.Manifest}; attemptManifest={result.AttemptManifest}; preservedPrevious={result.PreservedPrevious}; chunks={result.Chunks.Length}; records={result.Records}; truncated={result.Truncated}; boundedRunByteBudget=true; localOnly=true");
        });

    public void ExpectNativeCapture(IReadOnlyList<PreparedSegment> sections)
        =>Artifact("native-coverage-expectations",archive=>archive.ExpectNativeSections(sections.Select(DiagnosticArtifacts.SectionIdentity)));

    public bool DiagnosticCompressedRecords(string name,IEnumerable<object> records)
    {
        var captured=false;
        Artifact(name,archive=>
        {
            var result=archive.WriteCompressedRecords(name,records);
            captured=true;
            Diagnostic($"ARTIFACT REPLAY file={result.File}; records={result.Records}; compressedBytes={result.Bytes}; complete=true; encoding=gzip-ndjson; finalBudget=true");
        });
        return captured;
    }

    public void DiagnosticHeights(string name,ushort[] heights)=>Artifact(name,archive=>archive.WriteHeights(name,heights));
    public void DiagnosticJson(string name,object value)=>Artifact(name,archive=>archive.WriteJson(name,value));
    public void DiagnosticSvg(string name,string svg)=>Artifact(name,archive=>archive.WriteSvg(name,svg));

    // Exact replay records use the protected, bounded replay reserve. Capture
    // completeness remains in each manifest; a partial archive is never a fixture.
    public void CaptureReplayBoundary(string name,IEnumerable<object> records)
        =>Artifact(name,archive=>archive.WriteRecordChunks("offline-"+name+".ndjson",records,final:true,compress:true));

    public void ObserveNativeCapture(string stem,IEnumerable<(string Identity,bool Resolved)> sections,bool requireGeometryOutcomes=false)
        =>Artifact("native-coverage-observation",archive=>archive.ObserveNativeSections(stem,sections,requireGeometryOutcomes));

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
            var records=prepared.Segments.Select(s=>(object)new
            {
                s.WayId,s.StartNodeId,s.EndNodeId,s.SourceWayIds,s.SourceNodeAliases,kind=s.Kind.ToString(),s.Classification,s.Reference,s.CorridorId,s.Lanes,s.OneWay,s.Bridge,s.Tunnel,s.Layer,s.NativeWidth,s.NativeAssetKey,s.NativeUnitId,s.NativeCenterlineCurvatureLimit,
                a=DiagnosticArtifacts.Point(s.CourseStart),b=DiagnosticArtifacts.Point(s.CourseControl1),c=DiagnosticArtifacts.Point(s.CourseControl2),d=DiagnosticArtifacts.Point(s.CourseEnd),
                horizontalEmissionReference=DiagnosticArtifacts.Curve(s.HorizontalEmissionReference),
                nodeStart=DiagnosticArtifacts.Point(s.Start),nodeEnd=DiagnosticArtifacts.Point(s.End),s.StartElevation,s.EndElevation
            });
            var result=archive.WriteRecordChunks("final-prepared-network.ndjson",records,final:true,compress:true);
            var assets=archive.WriteRecords("final-native-asset-contracts.ndjson",prepared.NativeAssets.Cast<object>(),1024L*1024,final:true);
            if(prepared.Design?.Source!=null)
                archive.WriteRecordChunks("immutable-source-network.ndjson",DiagnosticArtifacts.SourceRecords(prepared.Design.Source),final:true,compress:true);
            archive.WriteHeights("final-design-heightmap.u16",prepared.Heightmap);
            archive.WriteJson("final-design.json",new{schemaVersion=1,fingerprint=prepared.Design?.Fingerprint,quality=prepared.Design?.Quality.ToString()??"RequiresReview",
                sections=prepared.Segments.Count,nativeAssets=prepared.NativeAssets.Count,sourceChains=prepared.Design?.Source?.Chains.Count,
                fingerprintScope="final-section-anchors-curves-provenance-and-native-asset-contracts",networkManifest=result.Manifest,assetsFile=assets.File,
                networkTruncated=result.Truncated,assetsTruncated=assets.Truncated,exactFloatDeckFrozen=prepared.Design!=null,nativeAcceptanceVerified=false,
                terrainFile="final-design-heightmap.u16",heightSamples=prepared.Heightmap.Length,heightScaleMeters=prepared.HeightScaleMeters,
                mapWidthMeters=prepared.Request.MapSize.WidthMeters,terrainEncoding="little-endian uint16, row-major z/x; local centred metres; metres=value/65535*heightScaleMeters"});
            Diagnostic($"PREPARED ARTIFACT manifest={result.Manifest}; records={result.Records}/{prepared.Segments.Count}; truncated={result.Truncated}; compactFinalCurves=true; reservedFinalBudget=true; nativeAppearanceNotCertified=true");
        });
    }

    public void CaptureProfileInput(IReadOnlyList<PreparedSegment> sections,ushort[] heights,float scale,double width,
        IReadOnlyDictionary<string,NativeAssetContract>? assets)
        =>Artifact("profile-input",archive=>
        {
            var geometry=archive.WriteRecordChunks("profile-horizontal-input.ndjson",DiagnosticArtifacts.ExactNetworkRecords(sections),final:true,compress:true);
            var contracts=archive.WriteRecords("profile-native-contracts.ndjson",assets==null?Array.Empty<object>():assets.Values.Cast<object>(),final:true);
            archive.WriteHeights("profile-baseline-heightmap.u16",heights);
            archive.WriteJson("profile-input.json",new{schemaVersion=1,networkManifest=geometry.Manifest,assetsFile=contracts.File,
                terrainFile="profile-baseline-heightmap.u16",heightScaleMeters=scale,mapWidthMeters=width,sections=sections.Count,
                geometryTruncated=geometry.Truncated,assetsTruncated=contracts.Truncated,geometryEncoding="prepared-segment-exact-v1",
                scope="frozen-horizontal-profile-input-before-roadbed-stamp",heightEncoding="little-endian-uint16-corner-grid"});
        });

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
