using ImportEarthLocation.Core;
using Newtonsoft.Json.Linq;
using Unity.Mathematics;

static class ArchiveTests
{
    public static void Run()
    {
        var root = Path.Combine(Path.GetTempPath(), "iel-history-" + Guid.NewGuid().ToString("N"));
        try
        {
            var archive = new RunArchive(root, new { latitude = 38.9, roads = "primary" }, typeof(ArchiveTests).Assembly.Location);
            var second = new RunArchive(root, new { latitude = 38.9 }, "missing.dll");
            if (archive.DirectoryPath == second.DirectoryPath) throw new Exception("Runs overwrite each other");
            var metrics = new SortedDictionary<string,double> { ["Trees prepared"] = 60000, ["Network cleanup time (s)"] = 1.55 };
            archive.Write(new[] { "PREPARE", "WATER native readback" }, metrics, "prepared");
            archive.Write(new[] { "IMPORT TOTALS" }, metrics, "completed");
            archive.Write(Array.Empty<string>(), metrics, null);
            var result = JObject.Parse(File.ReadAllText(Path.Combine(archive.DirectoryPath,"metrics.json")));
            if ((string?)result["state"] != "completed" || (double?)result["metrics"]?["Trees prepared"] != 60000) throw new Exception("Archived metrics/outcome lost");
            if (File.ReadAllLines(Path.Combine(archive.DirectoryPath,"events.log")).Length != 3) throw new Exception("Events truncated or duplicated");
            var metadata=JObject.Parse(File.ReadAllText(Path.Combine(archive.DirectoryPath,"run.json")));
            if (((string?)metadata["assemblySha256"])?.Length != 64 || metadata["settings"]?["latitude"] == null) throw new Exception("Build/settings provenance missing");
            archive.Checkpoint("before-native-apply", new RunArchive.NativeOperationIntent(7,8,9,"Prepared",128,0,0));
            var checkpoint=JObject.Parse(File.ReadAllText(Path.Combine(archive.DirectoryPath,"last-operation.json")));
            if((string?)checkpoint["phase"]!="before-native-apply"||(int?)checkpoint["details"]?["Definitions"]!=128||(long?)checkpoint["details"]?["MapSession"]!=7)throw new Exception("Pre-native crash checkpoint lost batch/epoch identity");
            archive.Checkpoint("settled", new RunArchive.NativeOperationIntent(7,8,9,"Settled",128,128,1));
            checkpoint=JObject.Parse(File.ReadAllText(Path.Combine(archive.DirectoryPath,"last-operation.json")));
            if((long?)checkpoint["sequence"]!=2||(int?)checkpoint["details"]?["Submitted"]!=128||File.Exists(Path.Combine(archive.DirectoryPath,"checkpoints.ndjson")))throw new Exception("Bounded last-operation replacement lost intent or created prohibited journal");
            archive.Checkpoint("legacy-payload",new { count=128, unboundedGraph="must-not-be-serialized" });
            checkpoint=JObject.Parse(File.ReadAllText(Path.Combine(archive.DirectoryPath,"last-operation.json")));
            if((string?)checkpoint["phase"]!="legacy-payload"||checkpoint["details"]?.Type!=JTokenType.Null||new FileInfo(Path.Combine(archive.DirectoryPath,"last-operation.json")).Length>4096)throw new Exception("Checkpoint serialized unbounded legacy payload or exceeded fixed budget");
            var bounded=archive.WriteRecords("bounded.ndjson", Enumerable.Range(0,20).Select(i=>(object)new {i,text="test"}), maximumRecords:2);
            if(!bounded.Truncated||bounded.Records!=2||File.ReadAllLines(Path.Combine(archive.DirectoryPath,"bounded.ndjson")).Length!=2)throw new Exception("Diagnostic record bound silently discarded/truncated invalid JSON");
            var previousRecords=File.ReadAllText(Path.Combine(archive.DirectoryPath,"bounded.ndjson"));
            IEnumerable<object> InterruptedRecords(){yield return new {i=99};throw new IOException("Simulated diagnostic write failure");}
            try{archive.WriteRecords("bounded.ndjson",InterruptedRecords());throw new Exception("Interrupted artifact unexpectedly completed");}catch(IOException){}
            if(File.ReadAllText(Path.Combine(archive.DirectoryPath,"bounded.ndjson"))!=previousRecords)throw new Exception("Failed artifact overwrite destroyed previously recorded evidence");
            var byteBound=archive.WriteRecords("byte-bound.ndjson",new object[]{new {text=new string('x',100)}},maximumBytes:8);
            if(!byteBound.Truncated||byteBound.Records!=0||byteBound.Bytes!=0)throw new Exception("Diagnostic byte budget not enforced");
            try {archive.WriteJson("../escape.json",new{});throw new Exception("Diagnostic path escaped run directory");} catch(ArgumentException) {}
            var originalHeights=new ushort[]{0,1,256,65535};var heights=(ushort[])originalHeights.Clone();
            var section=new PreparedSegment(10,11,12,new(0,5,0),new(100,6,0),NetworkKind.Road,"primary",4,false,false,false,0)
                {Control1=new float3(20,5,0),Control2=new float3(80,6,0),StartCurveOffset=new float3(0,0,2),NativeWidth=24,SourceWayIds=new long[]{10,20},SourceTags=new Dictionary<string,string>{{"lanes","2"},{"website","secret-endpoint"}}};
            var geometry=DiagnosticArtifacts.CaptureGeometry(archive,"fixture",new[]{section},heights,100,1000);
            var bytes=File.ReadAllBytes(Path.Combine(archive.DirectoryPath,"fixture-heightmap.u16"));
            if(geometry.Truncated||bytes.Length!=8||bytes[4]!=0||bytes[5]!=1||!heights.SequenceEqual(originalHeights))throw new Exception("Diagnostic height encoding changed input or byte order");
            var record=JObject.Parse(File.ReadAllLines(Path.Combine(archive.DirectoryPath,"fixture-network.ndjson"))[0]);
            if((long?)record["WayId"]!=10||(float?)record["Control1"]?["x"]!=20||(float?)record["StartCurveOffset"]?["z"]!=2||record["SourceTags"]?["website"]!=null)throw new Exception("Curve/offset/source provenance missing or unrelated URL leaked");
            var prepared=new PreparedImport {Segments=new[]{section},Heightmap=heights,HeightScaleMeters=100,Vegetation=new[]{new VegetationCandidate(new(0,5,20),VegetationForm.Palm,10,42,0)},VegetationMetadata=new[]{new VegetationMetadata("warm-arid-existing-tree-site","")},SurfaceMask=new SurfaceMask(2,1000,new byte[]{1,0,0,0},new byte[]{255,0,0,0},"fixture")};
            DiagnosticArtifacts.CapturePrepared(archive,prepared);
            var environment=JObject.Parse(File.ReadAllText(Path.Combine(archive.DirectoryPath,"environment-plan.json")));
            if((int?)environment["vegetationCandidates"]!=1||File.ReadAllBytes(Path.Combine(archive.DirectoryPath,"surface-material.u8"))[0]!=1||prepared.Segments[0]!=section)throw new Exception("Environment evidence missing or generation inputs changed");
            Console.WriteLine("PASS: synchronous bounded native intent replacement without history journal, bounded complete JSON records, path containment, real height/curve/offset snapshots, URL whitelist and immutable environment artifacts.");
            Console.WriteLine("PASS: independent run archives, append-only events, numerical metrics, persisted outcome, build fingerprint/settings.");
            var budgetArchive=new RunArchive(root,new{},"missing.dll");
            var payload=new {text=new string('x',1024*1024)};
            for(var i=0;i<6;i++)budgetArchive.WriteRecords("preview-"+i+".ndjson",Enumerable.Repeat<object>(payload,8));
            var exhausted=budgetArchive.WriteRecords("ordinary-exhausted.ndjson",new object[]{payload});
            if(!exhausted.Truncated||exhausted.LimitReason!="run-budget")throw new Exception("Exhausted ordinary budget was not explained");
            var final=budgetArchive.WriteRecordChunks("final-evidence.ndjson",Enumerable.Repeat<object>(payload,20),chunkRecords:4,final:true);
            if(final.Truncated||final.Records!=20)throw new Exception("Ordinary captures exhausted protected final evidence or old 16MiB limit remained");
            for(var i=0;i<3;i++)if(!budgetArchive.CaptureIntermediate($"native-batch-{i}-expected.ndjson"))throw new Exception("Initial preview examples not retained");
            if(budgetArchive.CaptureIntermediate("native-batch-99-expected.ndjson"))throw new Exception("Repeated preview graph not suppressed");
            if(!budgetArchive.CaptureIntermediate("native-batch-99-failed-live.ndjson"))throw new Exception("Failure evidence was sampled as routine preview");
            budgetArchive.Write(Array.Empty<string>(),metrics,"completed");
            var completeness=JObject.Parse(File.ReadAllText(Path.Combine(budgetArchive.DirectoryPath,"capture-completeness.json")));
            if((int?)completeness["suppressedIntermediateCaptures"]!=1||!completeness["incompleteArtifacts"]!.Any()||
                !completeness["artifacts"]!.Any(a=>(string?)a["Budget"]=="final"))throw new Exception("Capture completeness omitted loss, suppression or protected final evidence");
            Console.WriteLine("PASS: ordinary-budget exhaustion preserves >16MiB final capture, explicit limit reasons, routine sampling and completeness manifest.");
            budgetArchive.ExpectNativeSections(new[]{"Road:1:2:3","Rail:4:5:6","Road:7:8:9"});
            budgetArchive.WriteRecordChunks("unit.ndjson",new object[]{new{identity="Road:1:2:3"}},final:true);
            budgetArchive.WriteRecordChunks("unit-port-movements.ndjson",Array.Empty<object>(),final:true);
            budgetArchive.ObserveNativeSections("unit",new[]{("Road:1:2:3",true),("Rail:4:5:6",false)});
            budgetArchive.Write(Array.Empty<string>(),metrics,"completed");
            var coverage=JObject.Parse(File.ReadAllText(Path.Combine(budgetArchive.DirectoryPath,"capture-completeness.json")))["nativeCoverage"]!;
            if((int?)coverage["capturedSections"]!=2||(bool?)coverage["complete"]!=false||coverage["missingOrIncompleteSections"]!.Count()!=1||coverage["unresolvedSections"]!.Count()!=1)
                throw new Exception("Completed generation hid missing or unresolved native coverage");
            budgetArchive.ObserveNativeSections("unit",new[]{("Road:7:8:9",true)});
            budgetArchive.Write(Array.Empty<string>(),metrics,"completed");
            coverage=JObject.Parse(File.ReadAllText(Path.Combine(budgetArchive.DirectoryPath,"capture-completeness.json")))["nativeCoverage"]!;
            if((bool?)coverage["complete"]!=true)throw new Exception("Complete captured native coverage remained unavailable");
            budgetArchive.ObserveNativeSections("missing-artifact",new[]{("Road:1:2:3",true)});
            budgetArchive.Write(Array.Empty<string>(),metrics,"completed");
            coverage=JObject.Parse(File.ReadAllText(Path.Combine(budgetArchive.DirectoryPath,"capture-completeness.json")))["nativeCoverage"]!;
            if((bool?)coverage["complete"]!=false)throw new Exception("Unwritten native artifact was counted as complete");
            Console.WriteLine("PASS: expected native identities, unresolved observations and missing artifacts are distinguished from generation completion.");
            var longRoot=Path.Combine(root,new string('a',Math.Max(1,140-root.Length)));
            var longArchive=new RunArchive(longRoot,new{},"missing.dll");
            var longName="native-batch-000000-030-attempt-00-submitted-courses";
            var longCapture=longArchive.WriteRecordChunks(longName+".ndjson",new object[]{new{value=1}},final:true);
            var forcedName=new string('b',150)+".ndjson";
            var hashed=longArchive.WriteRecords(forcedName,new object[]{new{value=2}},final:true);
            longArchive.RecordFailure("failed-fixture",new IOException("simulated single-artifact failure"));
            longArchive.WriteRecordChunks("after-failure.ndjson",new object[]{new{value=3}},final:true);
            var compressed=longArchive.WriteCompressedRecords("replay.ndjson.gz",Enumerable.Range(0,8000).Select(i=>(object)new{ordinal=i,coefficient=1.1234567890123456,record=i==7999?"complete":"row"}));
            var priorCompressed=File.ReadAllBytes(Path.Combine(longArchive.DirectoryPath,compressed.File));
            try{longArchive.WriteCompressedRecords("replay.ndjson.gz",InterruptedRecords());throw new Exception("Interrupted compressed capture unexpectedly completed");}catch(IOException){}
            if(!File.ReadAllBytes(Path.Combine(longArchive.DirectoryPath,compressed.File)).SequenceEqual(priorCompressed)||File.Exists(Path.Combine(longArchive.DirectoryPath,compressed.File)+".tmp"))
                throw new Exception("Failed compressed overwrite destroyed prior evidence or retained its unpublished fragment");
            using(var stream=File.OpenRead(Path.Combine(longArchive.DirectoryPath,compressed.File)))
            using(var gzip=new System.IO.Compression.GZipStream(stream,System.IO.Compression.CompressionMode.Decompress))
            using(var reader=new StreamReader(gzip))
            {
                var lines=reader.ReadToEnd().Split('\n',StringSplitOptions.RemoveEmptyEntries);
                if(lines.Length!=8000||(string?)JObject.Parse(lines[^1])["record"]!="complete"||
                    (double?)JObject.Parse(lines[0])["coefficient"]!=1.1234567890123456)throw new Exception("Compressed replay lost its trailer or exact coefficient");
            }
            longArchive.Write(Array.Empty<string>(),metrics,"completed");
            var files=Directory.GetFiles(longArchive.DirectoryPath);
            if(files.Any(p=>p.Length>240)||!File.Exists(Path.Combine(longArchive.DirectoryPath,longCapture.Manifest))||
                longCapture.Chunks.Any(c=>!File.Exists(Path.Combine(longArchive.DirectoryPath,c.File)))||!File.Exists(Path.Combine(longArchive.DirectoryPath,hashed.File)))
                throw new Exception("Diagnostic paths exceed the legacy runtime bound or manifest references are not physical filenames");
            var failures=JObject.Parse(File.ReadAllText(Path.Combine(longArchive.DirectoryPath,"capture-completeness.json")));
            if((int?)failures["artifactFailures"]!=1||failures["failedArtifacts"]?["failed-fixture"]==null||
                !File.Exists(Path.Combine(longArchive.DirectoryPath,"after-failure-chunks.json")))throw new Exception("One artifact failure prevented later evidence or disappeared from completeness");
            Console.WriteLine("PASS: bounded Windows paths, physical chunk references, isolated failure ledger and full compressed numerical replay.");
            // A failed optional detail must not repeatedly walk another graph
            // or spend the independent census/replay reserves.
            var exhaustedArchive=new RunArchive(root,new{},"missing.dll");
            typeof(RunArchive).GetField("_finalRecordBytes",System.Reflection.BindingFlags.Instance|System.Reflection.BindingFlags.NonPublic)!.SetValue(exhaustedArchive,64L*1024*1024);
            var enumerated=false;
            IEnumerable<object> Expensive(){enumerated=true;yield return new{value="must-not-enumerate"};}
            try{exhaustedArchive.WriteCompressedRecords("optional-detail.ndjson.gz",Expensive());throw new Exception("Exhausted detail reserve accepted capture");}catch(IOException){}
            if(enumerated)throw new Exception("Exhausted diagnostics still enumerated expensive records");
            var reserved=exhaustedArchive.WriteCompressedRecords("profile-result.ndjson.gz",new object[]{new{record="complete"}});
            if(reserved.Records!=1)throw new Exception("Optional detail exhaustion consumed independent replay reserve");
            var overflowed=new RunArchive(root,new{},"missing.dll");
            typeof(RunArchive).GetField("_finalRecordBytes",System.Reflection.BindingFlags.Instance|System.Reflection.BindingFlags.NonPublic)!.SetValue(overflowed,64L*1024*1024-64);
            var noise=new byte[4096];new System.Random(123).NextBytes(noise);
            try{overflowed.WriteCompressedRecords("oversized-detail.ndjson.gz",new object[]{new{noise=Convert.ToBase64String(noise)}});throw new Exception("Tiny remaining reserve accepted oversized capture");}catch(IOException){}
            enumerated=false;
            try{overflowed.WriteCompressedRecords("next-detail.ndjson.gz",Expensive());throw new Exception("Overflowed bucket was reused");}catch(IOException){}
            if(enumerated)throw new Exception("Repeated compression continued after a bucket overflow");
            var model=new PreparedSegment(123,1,2,new(12345.125f,45,32),new(12445.5f,47,36),NetworkKind.Road,"motorway",2,true,false,false,0)
                {SourceGeometry=Enumerable.Range(0,600).Select(i=>new float3(i,1,2)).ToArray(),
                    HorizontalEmissionReference=new(new(1,2,3),new(4,5,6),new(7,8,9),new(10,11,12))};
            var modelFile=archive.WriteCompressedRecords("offline-roundtrip.ndjson.gz",DiagnosticArtifacts.ExactNetworkRecords(new[]{model with{HorizontalParent=model}}));
            using(var stream=File.OpenRead(Path.Combine(archive.DirectoryPath,modelFile.File)))
            using(var gzip=new System.IO.Compression.GZipStream(stream,System.IO.Compression.CompressionMode.Decompress))
            using(var reader=new StreamReader(gzip))
            {
                var modelRecord=JObject.Parse(reader.ReadLine()!);var recovered=modelRecord.ToObject<PreparedSegment>()!;
                if(modelRecord["Start"]!.Children().Count()!=3||modelRecord["HorizontalParent"]!.Type!=JTokenType.Null||
                    recovered.SourceGeometry.Count!=600||!math.all(recovered.CourseStart==model.CourseStart)||
                    recovered.HorizontalEmissionReference!=model.HorizontalEmissionReference)
                    throw new Exception("Exact replay lost coordinates/source points or serialized recursive vector/parent properties");
            }
        }
        finally { if (Directory.Exists(root)) Directory.Delete(root, true); }
    }
}
