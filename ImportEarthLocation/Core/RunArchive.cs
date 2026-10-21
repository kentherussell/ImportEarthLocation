using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Compression;
using System.Security.Cryptography;
using System.Text;
using Newtonsoft.Json;
using Unity.Mathematics;

namespace ImportEarthLocation.Core;

// Local, append-only run history. No uploads, background services, or native game access.
public sealed class RunArchive
{
    public string DirectoryPath { get; }
    private readonly DateTime _started = DateTime.UtcNow;
    private string _state = "preparing";
    private long _checkpointSequence;
    private long _recordBytes;
    private long _criticalRecordBytes;
    private long _finalRecordBytes;
    private long _censusRecordBytes;
    private long _replayRecordBytes;
    private readonly HashSet<string> _exhaustedFinalBudgets=new(StringComparer.Ordinal);
    private static string BudgetKey(bool census,bool replay)=>census?"census":replay?"replay":"details";
    private static bool IsCensus(string name)=>name.StartsWith("native-owned-final-",StringComparison.Ordinal)||
        name.StartsWith("native-final-source-",StringComparison.Ordinal)||name.StartsWith("native-owned-census-",StringComparison.Ordinal);
    private static bool IsReplay(string name)=>name.StartsWith("offline-",StringComparison.Ordinal)||name=="profile-problem.ndjson.gz"||name=="profile-result.ndjson.gz"||
        name=="profile-native-variable-bindings.ndjson.gz"||name.StartsWith("profile-variable-map",StringComparison.Ordinal)||
        name.StartsWith("profile-horizontal-input",StringComparison.Ordinal)||name.StartsWith("profile-native-contracts",StringComparison.Ordinal)||
        name.StartsWith("final-prepared-network",StringComparison.Ordinal)||name.StartsWith("immutable-source-network",StringComparison.Ordinal)||name.StartsWith("final-native-asset-contracts",StringComparison.Ordinal);
    private long FinalRemaining(bool census,bool replay)=>_exhaustedFinalBudgets.Contains(BudgetKey(census,replay))?0:Math.Max(0,Math.Min(512L*1024*1024-_finalRecordBytes,
        census?256L*1024*1024-_censusRecordBytes:replay?192L*1024*1024-_replayRecordBytes:
        64L*1024*1024-(_finalRecordBytes-_censusRecordBytes-_replayRecordBytes)));
    private long _artifactSequence;
    private readonly Dictionary<string,ArtifactResult> _captures=new(StringComparer.Ordinal);
    private readonly Dictionary<string,ChunkedArtifactResult> _chunkCaptures=new(StringComparer.Ordinal);
    private readonly HashSet<string> _nativeExpected=new(StringComparer.Ordinal);
    private readonly Dictionary<string,string> _nativeObserved=new(StringComparer.Ordinal);
    private readonly Dictionary<string,string> _nativeCaptureStems=new(StringComparer.Ordinal);
    private readonly HashSet<string> _nativeGeometryOutcomeStems=new(StringComparer.Ordinal);
    private readonly Dictionary<string,int> _sampledFamilies=new(StringComparer.Ordinal);
    private int _suppressedCaptures;
    private readonly Dictionary<string,string> _fileNames=new(StringComparer.Ordinal);
    private readonly Dictionary<string,string> _failedArtifacts=new(StringComparer.Ordinal);
    public object CaptureStatus()=>new
    {
        expectedSections=_nativeExpected.Count,observedSections=System.Linq.Enumerable.Count(_nativeExpected,k=>ObservationState(k)!=null),
        completeNativeSectionCapture=_nativeExpected.Count>0&&System.Linq.Enumerable.All(_nativeExpected,k=>ObservationState(k) is "lineage-observed" or "unresolved"),
        nativeCaptureApplicable=_nativeExpected.Count>0,
        sourceQualifiedSections=System.Linq.Enumerable.Count(_nativeExpected,k=>ObservationState(k)=="lineage-observed"),
        unresolvedSections=System.Linq.Enumerable.Count(_nativeExpected,k=>ObservationState(k)=="unresolved"),
        incompleteArtifacts=System.Linq.Enumerable.Count(_captures.Values,c=>c.Truncated),
        incompleteChunkCaptures=System.Linq.Enumerable.Count(_chunkCaptures.Values,c=>c.Truncated),artifactFailures=_artifactFailures,
        currentFailedArtifacts=_failedArtifacts.Count,
        sourceQualificationImplied=false
    };
    private int _artifactFailures;
    public sealed record NativeOperationIntent(long MapSession, long Generation, ulong WorldSequenceNumber,
        string State, int Definitions, int Submitted, int ApplyInvocations,
        int SourceStartIndex = -1, int SourceSections = -1);

    public sealed record ArtifactResult(string File, int Records, long Bytes, bool Truncated)
    { public bool PreservedPrevious { get; init; } public string? AttemptFile { get; init; }
      public string? LimitReason { get; init; } public string Budget { get; init; } = "ordinary"; public string Encoding { get; init; }="ndjson"; }
    public sealed record ChunkedArtifactResult(string Manifest, ArtifactResult[] Chunks, int Records, bool Truncated)
    { public bool PreservedPrevious { get; init; } public string? AttemptManifest { get; init; } }

    // JavaScript Number cannot round-trip generated Int64 node identities.
    // Keep identifiers exact in record artifacts; ordinary counts remain Int32.
    private sealed class ExactIntegerConverter : JsonConverter
    {
        public override bool CanConvert(Type type)=>type==typeof(long)||type==typeof(ulong);
        public override bool CanRead=>false;
        public override void WriteJson(JsonWriter writer,object? value,JsonSerializer serializer)
            =>writer.WriteValue(Convert.ToString(value,System.Globalization.CultureInfo.InvariantCulture));
        public override object ReadJson(JsonReader reader,Type type,object? existing,JsonSerializer serializer)=>throw new NotSupportedException();
    }
    private static readonly JsonSerializerSettings RecordSettings=new()
    { FloatFormatHandling=FloatFormatHandling.String,Converters={new ExactIntegerConverter(),new ScalarVectorConverter()} };

    // Unity vectors expose recursive computed swizzles. Exact replay records
    // must serialize scalar coordinates only, not their public property graph.
    private sealed class ScalarVectorConverter : JsonConverter
    {
        public override bool CanRead=>false;
        public override bool CanConvert(Type type)
        {
            type=Nullable.GetUnderlyingType(type)??type;
            return type==typeof(float2)||type==typeof(float3)||type==typeof(float4)||
                type==typeof(double2)||type==typeof(double3)||type==typeof(int3);
        }
        public override void WriteJson(JsonWriter writer,object? value,JsonSerializer serializer)
        {
            if(value==null){writer.WriteNull();return;}
            writer.WriteStartObject();
            void Number(string name,double coordinate){writer.WritePropertyName(name);writer.WriteValue(coordinate);}
            switch(value)
            {
                case float2 p:Number("x",p.x);Number("y",p.y);break;
                case float3 p:Number("x",p.x);Number("y",p.y);Number("z",p.z);break;
                case float4 p:Number("x",p.x);Number("y",p.y);Number("z",p.z);Number("w",p.w);break;
                case double2 p:Number("x",p.x);Number("y",p.y);break;
                case double3 p:Number("x",p.x);Number("y",p.y);Number("z",p.z);break;
                case int3 p:Number("x",p.x);Number("y",p.y);Number("z",p.z);break;
                default:throw new JsonSerializationException("Unsupported replay vector");
            }
            writer.WriteEndObject();
        }
        public override object ReadJson(JsonReader reader,Type type,object? existing,JsonSerializer serializer)=>throw new NotSupportedException();
    }

    public RunArchive(string root, object settings, string assemblyPath, object? environment = null)
    {
        // Leave room for metadata and atomic-write suffixes on Unity's Windows
        // runtime, which cannot be assumed to support extended-length paths.
        DirectoryPath = Path.Combine(root, "r-" + Guid.NewGuid().ToString("N"));
        if(DirectoryPath.Length+48>240)throw new IOException("Diagnostic root is too long for safe Windows artifact paths.");
        Directory.CreateDirectory(DirectoryPath);
        string? fingerprint = null;
        var fingerprintStatus = "assembly-location-unavailable";
        if (File.Exists(assemblyPath))
        {
            // Fingerprinting must not abort the complete diagnostic archive if
            // a loader exposes an inaccessible or transient assembly location.
            try
            {
                using var sha = SHA256.Create(); using var stream = File.OpenRead(assemblyPath);
                fingerprint = BitConverter.ToString(sha.ComputeHash(stream)).Replace("-", "").ToLowerInvariant();
                fingerprintStatus = "loaded-assembly-file-hashed";
            }
            catch(IOException){fingerprintStatus="assembly-file-unreadable";}
            catch(UnauthorizedAccessException){fingerprintStatus="assembly-file-access-denied";}
        }
        File.WriteAllText(Path.Combine(DirectoryPath, "run.json"), JsonConvert.SerializeObject(new
        {
            schemaVersion = 4, pipelineVersion = "native-acceptance-replay-v4", startedUtc = _started,
            artifactPathPolicy="short-physical-names-with-logical-file-index; maximum240-characters-including-temp-suffix",
            recordIntegerEncoding = "Int64 and UInt64 are decimal JSON strings; Int32 counts remain numbers",
            assemblySha256 = fingerprint, assemblyFingerprintStatus = fingerprintStatus,
            assemblyFileName = string.IsNullOrEmpty(assemblyPath)?null:Path.GetFileName(assemblyPath), settings, environment,
            evidence = "Recorded operations and counts are not proof of final native rendering or persistence. Read native readback messages separately."
        }, Formatting.Indented), Encoding.UTF8);
    }

    public void Write(IReadOnlyList<string> lines, IReadOnlyDictionary<string, double> metrics, string? state)
    {
        if (state != null) _state = state;
        if (lines.Count > 0) File.AppendAllText(Path.Combine(DirectoryPath, "events.log"), string.Join(Environment.NewLine, lines) + Environment.NewLine, Encoding.UTF8);
        var path = Path.Combine(DirectoryPath, "metrics.json");
        var temporary = path + ".tmp";
        File.WriteAllText(temporary, JsonConvert.SerializeObject(new
        {
            schemaVersion = 1, startedUtc = _started, updatedUtc = DateTime.UtcNow, state = _state,
            elapsedSeconds = (DateTime.UtcNow - _started).TotalSeconds, metrics
        }, Formatting.Indented), Encoding.UTF8);
        if (File.Exists(path)) File.Replace(temporary, path, null); else File.Move(temporary, path);
        WriteJson("capture-completeness.json",new
        {
            schemaVersion=1,state=_state,updatedUtc=DateTime.UtcNow,
            ordinaryBytes=_recordBytes,finalBytes=_finalRecordBytes,criticalBytes=_criticalRecordBytes,
            nativeCensusBytes=_censusRecordBytes,nativeCensusReservedBytes=256L*1024*1024,
            inputReplayBytes=_replayRecordBytes,inputReplayLimitBytes=192L*1024*1024,optionalFinalDetailsLimitBytes=64L*1024*1024,
            ordinaryLimitBytes=32L*1024*1024,finalLimitBytes=512L*1024*1024,criticalLimitBytes=4L*1024*1024,
            suppressedIntermediateCaptures=_suppressedCaptures,
            artifactFailures=_artifactFailures,failedArtifacts=_failedArtifacts,
            intermediateFamilies=_sampledFamilies,
            nativeCoverage=new
            {
                expectedSections=_nativeExpected.Count,
                capturedSections=System.Linq.Enumerable.Count(_nativeExpected,k=>ObservationState(k) is "lineage-observed" or "unresolved"),
                unresolvedSections=System.Linq.Enumerable.Where(_nativeExpected,k=>ObservationState(k)=="unresolved"),
                missingOrIncompleteSections=System.Linq.Enumerable.Where(_nativeExpected,k=>ObservationState(k)==null||ObservationState(k)=="artifact-incomplete"),
                complete=_nativeExpected.Count>0&&System.Linq.Enumerable.All(_nativeExpected,k=>ObservationState(k) is "lineage-observed" or "unresolved"),
                observationsAreNotGeometricAcceptance=true
            },
            incompleteArtifacts=new List<ArtifactResult>(_captures.Values).FindAll(c=>c.Truncated),
            incompleteChunkCaptures=new List<ChunkedArtifactResult>(_chunkCaptures.Values).FindAll(c=>c.Truncated),
            artifacts=new List<ArtifactResult>(_captures.Values),
            evidence="Artifact completeness only. Native unit coverage and unavailable observations must also be checked; generation completion is not native acceptance."
        });
        WriteJson("artifact-files.json",new{schemaVersion=1,files=_fileNames});
    }

    // A failed optional artifact must not disable later final-state evidence.
    // Keep a bounded failure ledger; the aggregate count remains exact.
    public void RecordFailure(string name,Exception error)
    {
        _artifactFailures++;
        // Invalidation must not depend on whether the bounded message ledger
        // still has room. Old files remain recoverable, but are not this attempt.
        var recordsName=name.EndsWith(".gz",StringComparison.Ordinal)?name.Substring(0,name.Length-3):name;
        _captures.Remove(recordsName);_captures.Remove(recordsName+".gz");_chunkCaptures.Remove(recordsName);
        // Optional per-unit detail can fill the ordinary message ledger before
        // final replay capture. Reserve 64 additional entries for essential
        // inputs/census so their actual failure reason does not disappear.
        var essential=IsCensus(name)||IsReplay(name)||name=="profile-input"||name.StartsWith("design-pass-",StringComparison.Ordinal);
        if(_failedArtifacts.Count<(essential?192:128)||_failedArtifacts.ContainsKey(name))
            _failedArtifacts[name]=error.GetType().Name+": "+error.GetBaseException().Message;
        _chunkCaptures.Remove(name);
    }

    public void ExpectNativeSections(IEnumerable<string> identities)
    { _nativeExpected.Clear();foreach(var identity in identities)_nativeExpected.Add(identity); }

    public void ObserveNativeSections(string stem,IEnumerable<(string Identity,bool Resolved)> sections,bool requireGeometryOutcomes=false)
    {
        if(requireGeometryOutcomes)_nativeGeometryOutcomeStems.Add(stem);
        foreach(var section in sections)
        {_nativeCaptureStems[section.Identity]=stem;_nativeObserved[section.Identity]=section.Resolved?"lineage-observed":"unresolved";}
    }

    private bool CompleteRecords(string name)=>!_failedArtifacts.ContainsKey(name)&&
        (_chunkCaptures.TryGetValue(name,out var chunks)&&!chunks.Truncated||
         !_failedArtifacts.ContainsKey(name+".gz")&&_captures.TryGetValue(name+".gz",out var compressed)&&!compressed.Truncated);

    private string? ObservationState(string identity)
    {
        if(!_nativeObserved.TryGetValue(identity,out var state)||!_nativeCaptureStems.TryGetValue(identity,out var stem))return null;
        // Re-evaluate the latest attempt. A later failed recapture must not
        // leave the earlier successful observation falsely marked complete.
        return CompleteRecords(stem+".ndjson")&&CompleteRecords(stem+"-port-movements.ndjson")&&
            (!_nativeGeometryOutcomeStems.Contains(stem)||CompleteRecords(stem+"-connector-endpoints.ndjson")&&CompleteRecords(stem+"-junction-quality.ndjson"))?state:"artifact-incomplete";
    }

    // Detailed preview graphs are examples, not final-state coverage. Reserve disk
    // and serialization work for the post-neighbour readback of every native unit.
    public bool CaptureIntermediate(string name)
    {
        if(name=="native-live-batch-bindings.ndjson")
        {
            _sampledFamilies.TryGetValue(name,out var legacy);_sampledFamilies[name]=legacy+1;
            if(legacy<3)return true;_suppressedCaptures++;return false;
        }
        if(!name.StartsWith("native-batch-",StringComparison.Ordinal))return true;
        var suffixes=new[]{"-expected.ndjson","-submitted-placement-evidence.ndjson","-temp.ndjson",
            "-mutation-contracts.ndjson","-owners.ndjson","-graph.ndjson","-bindings.ndjson",
            "-live.ndjson","-settled-readback.ndjson","-settled-port-movements.ndjson",
            "-rejected-placement-evidence.ndjson","-matcher-rejection.ndjson","-rejection-endpoints.ndjson",
            "-downstream-contracts.ndjson","-submitted-courses.ndjson","-live-constraint-observations.ndjson"};
        foreach(var suffix in suffixes)if(name.EndsWith(suffix,StringComparison.Ordinal))
        {
            _sampledFamilies.TryGetValue(suffix,out var count);_sampledFamilies[suffix]=count+1;
            if(count<3)return true;
            _suppressedCaptures++;return false;
        }
        return true;
    }

    // Written BEFORE entering a native operation, rather than relying on the
    // periodic event buffer surviving an abrupt process exit. Not a crash diagnosis.
    public void Checkpoint(string phase, object details)
    {
        var sequence = ++_checkpointSequence;
        // Legacy anonymous payloads can contain entire native graphs. Only the
        // primitive intent contract belongs in the durable last-operation record.
        var record = new
        {
            schemaVersion = 1, sequence, recordedUtc = DateTime.UtcNow,
            phase = phase.Length<=160?phase:phase.Substring(0,160), state = _state,
            details = details as NativeOperationIntent,
            evidence = "Intent only; asynchronous native completion/fault is not proven."
        };
        if (Encoding.UTF8.GetByteCount(JsonConvert.SerializeObject(record))>4096)
            throw new InvalidOperationException("Native intent exceeded the 4 KiB diagnostic budget.");
        WriteJson("last-operation.json", record, durable: true);
    }

    public void WriteJson(string name, object value, bool durable = false)
    {
        var path = ArtifactPath(name); var temporary = path + ".tmp";
        using (var stream = new FileStream(temporary, FileMode.Create, FileAccess.Write, FileShare.Read))
        {
            using (var writer = new StreamWriter(stream, new UTF8Encoding(false), 4096, leaveOpen: true))
            {
                new JsonSerializer { FloatFormatHandling = FloatFormatHandling.String }.Serialize(writer, value);
                writer.Flush();
            }
            stream.Flush(durable);
        }
        Publish(temporary, path);
    }

    public void WriteSvg(string name,string svg)
    {
        if(!name.EndsWith(".svg",StringComparison.OrdinalIgnoreCase)||Encoding.UTF8.GetByteCount(svg)>1024*1024)
            throw new ArgumentException("Diagnostic SVG must have a bounded .svg payload.");
        var path=ArtifactPath(name);var temporary=path+".tmp";
        File.WriteAllText(temporary,svg,new UTF8Encoding(false));Publish(temporary,path);
    }

    public ArtifactResult WriteRecords(string name, IEnumerable<object> records, long maximumBytes = 64L * 1024 * 1024, int maximumRecords = 250000, bool critical = false,bool final = false,bool census = false,bool replay=false)
    {
        if (maximumBytes < 1 || maximumRecords < 1) throw new ArgumentOutOfRangeException(nameof(maximumBytes));
        // Keep a separate bounded reserve so ordinary preview captures cannot
        // consume the evidence budget for a later ownership failure.
        census|=IsCensus(name);
        replay|=IsReplay(name);
        var remaining=critical?4L*1024*1024-_criticalRecordBytes:final?FinalRemaining(census,replay):32L*1024*1024-_recordBytes;
        maximumBytes = Math.Min(maximumBytes, Math.Min(critical?1024L*1024:8L*1024*1024, Math.Max(0,remaining)));
        maximumRecords = Math.Min(maximumRecords,250000);
        var path = ArtifactPath(name); var temporary = path + ".tmp";
        var count = 0; long bytes = 0; var truncated = false; string? limitReason=null; var encoding = new UTF8Encoding(false);
        using (var stream = new FileStream(temporary, FileMode.Create, FileAccess.Write, FileShare.Read))
        {
            foreach (var record in records)
            {
                var line = JsonConvert.SerializeObject(record, RecordSettings) + "\n";
                var size = encoding.GetByteCount(line);
                if (count >= maximumRecords || bytes + size > maximumBytes)
                { truncated = true;limitReason=count>=maximumRecords?"record-limit":remaining<=bytes+size?"run-budget":"file-byte-limit";break; }
                var buffer = encoding.GetBytes(line); stream.Write(buffer, 0, buffer.Length);
                count++; bytes += buffer.Length;
            }
        }
        // A depleted budget must never erase earlier valid evidence. Keep its
        // metadata intact and describe this unsuccessful attempt separately.
        var preserve=truncated&&File.Exists(path)&&new FileInfo(path).Length>0;
        string? attempt=null;
        if(preserve)
        {
            attempt=$"{Path.GetFileNameWithoutExtension(name)}-attempt-{++_artifactSequence:D6}{Path.GetExtension(name)}";
            Publish(temporary,ArtifactPath(attempt));
        }
        else Publish(temporary, path);
        if(critical)_criticalRecordBytes+=bytes;else if(final){_finalRecordBytes+=bytes;if(census)_censusRecordBytes+=bytes;else if(replay)_replayRecordBytes+=bytes;}else _recordBytes+=bytes;
        var result = new ArtifactResult(Path.GetFileName(path), count, bytes, truncated){PreservedPrevious=preserve,AttemptFile=attempt==null?null:Path.GetFileName(ArtifactPath(attempt)),
            LimitReason=limitReason,Budget=critical?"critical":final?"final":"ordinary"};
        _captures[attempt??name]=result;
        if(!truncated)_failedArtifacts.Remove(name);
        if(attempt!=null)WriteJson(attempt+".meta.json",result);
        WriteJson(name + (preserve?".skipped.meta.json":".meta.json"), result);
        return result;
    }

    public ChunkedArtifactResult WriteRecordChunks(string name,IEnumerable<object> records,int chunkRecords=256,bool final=false,bool compress=false)
    {
        if(chunkRecords<1||chunkRecords>1024)throw new ArgumentOutOfRangeException(nameof(chunkRecords));
        ArtifactPath(name); // Validate the caller's name before deriving part names.
        var stem=Path.GetFileNameWithoutExtension(name);
        var capture=$"c{++_artifactSequence:D6}";
        var chunks=new List<ArtifactResult>();var batch=new List<object>(chunkRecords);
        var total=0;var truncated=false;
        using var enumerator=records.GetEnumerator();
        var exhausted=false;
        while(!exhausted)
        {
            batch.Clear();
            while(batch.Count<chunkRecords)
            {if(!enumerator.MoveNext()){exhausted=true;break;}batch.Add(enumerator.Current);}
            if(batch.Count==0)break;
            var part=$"{capture}-part-{chunks.Count:D4}.ndjson";
            var result=compress?WriteCompressedRecords(part+".gz",batch,census:IsCensus(name),replay:IsReplay(name)):
                WriteRecords(part,batch,maximumRecords:chunkRecords,final:final,census:IsCensus(name),replay:IsReplay(name));
            chunks.Add(result);total+=result.Records;
            if(result.Truncated){truncated=true;break;}
        }
        var manifest=stem+"-chunks.json";
        var attemptManifest=capture+"-chunks.json";
        var preserve=truncated&&File.Exists(ArtifactPath(manifest));
        var output=new ChunkedArtifactResult(Path.GetFileName(ArtifactPath(preserve?attemptManifest:manifest)),chunks.ToArray(),total,truncated)
            {PreservedPrevious=preserve,AttemptManifest=Path.GetFileName(ArtifactPath(attemptManifest))};
        // Publish immutable parts first, then the attempt manifest, and replace
        // the canonical pointer only after a complete generation is available.
        WriteJson(attemptManifest,output);
        if(!preserve)WriteJson(manifest,output);
        else WriteJson(stem+"-latest-attempt.json",output);
        _chunkCaptures[name]=output;
        if(!truncated)_failedArtifacts.Remove(name);
        return output;
    }

    public void WriteBytes(string name, byte[] values, int maximumBytes = 32 * 1024 * 1024)
    {
        if (values.Length > maximumBytes) throw new InvalidOperationException("Diagnostic raster exceeds its byte budget.");
        var path = ArtifactPath(name); var temporary = path + ".tmp";
        using (var stream = new FileStream(temporary, FileMode.Create, FileAccess.Write, FileShare.Read)) stream.Write(values, 0, values.Length);
        Publish(temporary, path);
    }

    // Complete numerical replay inputs need compression, not a first-megabyte
    // sample. Stream records without retaining a second city-sized object graph.
    public ArtifactResult WriteCompressedRecords(string name,IEnumerable<object> records,bool census=false,bool replay=false)
    {
        var path=ArtifactPath(name);var temporary=path+".tmp";var count=0;
        census|=IsCensus(name);replay|=IsReplay(name);var remaining=FinalRemaining(census,replay);
        // Once a protected bucket has overflowed, do not serialize/compress
        // another graph only to fail at the same byte limit. Other reserves
        // remain independent, and the failed artifact remains explicit.
        if(remaining<=0)throw new IOException("Diagnostic budget exhausted before enumeration: "+BudgetKey(census,replay));
        BoundedWriteStream? budgetStream=null;
        try
        {
            using var file=new FileStream(temporary,FileMode.Create,FileAccess.Write,FileShare.Read);
            using(var bounded=budgetStream=new BoundedWriteStream(file,remaining))
            using(var gzip=new GZipStream(bounded,CompressionLevel.Fastest,leaveOpen:true))
            using(var writer=new StreamWriter(gzip,new UTF8Encoding(false),65536))
            {
                var serializer=JsonSerializer.Create(RecordSettings);
                foreach(var record in records){serializer.Serialize(writer,record);writer.Write('\n');count=checked(count+1);}
            }
        }
        catch
        {
            if(budgetStream?.Exhausted==true)_exhaustedFinalBudgets.Add(BudgetKey(census,replay));
            // The unpublished temp belongs to this write. Preserve the prior
            // complete capture, charge partial I/O to the run budget, and do
            // not leave unreplayable gzip fragments occupying the reserve.
            try{if(File.Exists(temporary)){var partial=new FileInfo(temporary).Length;_finalRecordBytes+=partial;if(census)_censusRecordBytes+=partial;else if(replay)_replayRecordBytes+=partial;File.Delete(temporary);}}
            catch(IOException){}catch(UnauthorizedAccessException){}
            throw;
        }
        var bytes=new FileInfo(temporary).Length;_finalRecordBytes+=bytes;if(census)_censusRecordBytes+=bytes;else if(replay)_replayRecordBytes+=bytes;
        Publish(temporary,path);
        var result=new ArtifactResult(Path.GetFileName(path),count,bytes,false){Budget="final",Encoding="gzip-ndjson"};
        _captures[name]=result;_failedArtifacts.Remove(name);WriteJson(name+".meta.json",result);return result;
    }

    private sealed class BoundedWriteStream : Stream
    {
        private readonly Stream _inner;private readonly long _limit;private long _written;
        public bool Exhausted { get; private set; }
        public BoundedWriteStream(Stream inner,long limit){_inner=inner;_limit=limit;}
        public override void Write(byte[] buffer,int offset,int count)
        {
            if(count>_limit-_written){Exhausted=true;throw new IOException("Compressed replay exceeded the remaining protected diagnostic budget; capture is incomplete.");}
            _inner.Write(buffer,offset,count);_written+=count;
        }
        public override void Flush()=>_inner.Flush();
        public override bool CanRead=>false;public override bool CanSeek=>false;public override bool CanWrite=>true;
        public override long Length=>_written;public override long Position{get=>_written;set=>throw new NotSupportedException();}
        public override int Read(byte[] buffer,int offset,int count)=>throw new NotSupportedException();
        public override long Seek(long offset,SeekOrigin origin)=>throw new NotSupportedException();
        public override void SetLength(long length)=>throw new NotSupportedException();
    }

    public void WriteHeights(string name, ushort[] heights)
    {
        if (heights.Length > 16_777_216) throw new InvalidOperationException("Diagnostic height raster exceeds its byte budget.");
        var path = ArtifactPath(name); var temporary = path + ".tmp";
        using (var stream = new FileStream(temporary, FileMode.Create, FileAccess.Write, FileShare.Read))
        {
            if (BitConverter.IsLittleEndian)
            {
                var buffer = new byte[65536]; var total = heights.Length * 2;
                for (var offset = 0; offset < total; offset += buffer.Length)
                { var length = Math.Min(buffer.Length, total - offset); Buffer.BlockCopy(heights, offset, buffer, 0, length); stream.Write(buffer, 0, length); }
            }
            else { using var writer = new BinaryWriter(stream); foreach (var value in heights) writer.Write(value); }
        }
        Publish(temporary, path);
    }

    private string ArtifactPath(string name)
    {
        if (string.IsNullOrEmpty(name) || name.Contains("..") || name != Path.GetFileName(name)) throw new ArgumentException("Diagnostic filename must stay inside its run directory.");
        foreach (var c in name) if (!char.IsLetterOrDigit(c) && c != '-' && c != '_' && c != '.') throw new ArgumentException("Invalid diagnostic filename.");
        if(!_fileNames.TryGetValue(name,out var physical))
        {
            physical=name;
            // Reserve the longest metadata/temp suffix before choosing a name.
            if(DirectoryPath.Length+1+name.Length+22>240)
            {
                using var hash=SHA256.Create();
                var extension=Path.GetExtension(name);if(extension.Length>8)extension=".bin";
                physical="h-"+BitConverter.ToString(hash.ComputeHash(Encoding.UTF8.GetBytes(name))).Replace("-","").Substring(0,32).ToLowerInvariant()+extension;
            }
            _fileNames.Add(name,physical);
        }
        return Path.Combine(DirectoryPath, physical);
    }

    private static void Publish(string temporary, string path)
    { if (File.Exists(path)) File.Replace(temporary, path, null); else File.Move(temporary, path); }
}
