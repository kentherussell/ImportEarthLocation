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
            archive.Checkpoint("before-native-apply", new { batchStart=128, count=128 });
            var checkpoint=JObject.Parse(File.ReadAllText(Path.Combine(archive.DirectoryPath,"last-operation.json")));
            if((string?)checkpoint["phase"]!="before-native-apply"||(int?)checkpoint["details"]?["batchStart"]!=128)throw new Exception("Pre-native crash checkpoint lost batch identity");
            archive.Checkpoint("settled", new { submitted=128 });
            checkpoint=JObject.Parse(File.ReadAllText(Path.Combine(archive.DirectoryPath,"last-operation.json")));
            if((long?)checkpoint["sequence"]!=2||File.ReadAllLines(Path.Combine(archive.DirectoryPath,"checkpoints.ndjson")).Length!=2)throw new Exception("Checkpoint replacement/history lost");
            File.WriteAllBytes(Path.Combine(archive.DirectoryPath,"checkpoints.ndjson"),new byte[8*1024*1024-1]);
            archive.Checkpoint("journal-at-limit",new { count=128 });
            checkpoint=JObject.Parse(File.ReadAllText(Path.Combine(archive.DirectoryPath,"last-operation.json")));
            if((bool?)checkpoint["checkpointJournalTruncated"]!=true||(string?)checkpoint["phase"]!="journal-at-limit"||new FileInfo(Path.Combine(archive.DirectoryPath,"checkpoints.ndjson")).Length!=8*1024*1024-1)throw new Exception("Full journal corrupted checkpoint schema or exceeded budget");
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
            var prepared=new PreparedImport {Segments=new[]{section},Heightmap=heights,HeightScaleMeters=100,Vegetation=new[]{new VegetationCandidate(new(0,5,20),VegetationForm.Palm,42,10,"warm-arid-existing-tree-site","")},SurfaceMask=new SurfaceMask(2,1000,new byte[]{1,0,0,0},new byte[]{255,0,0,0},"fixture")};
            DiagnosticArtifacts.CapturePrepared(archive,prepared);
            var environment=JObject.Parse(File.ReadAllText(Path.Combine(archive.DirectoryPath,"environment-plan.json")));
            if((int?)environment["vegetationCandidates"]!=1||File.ReadAllBytes(Path.Combine(archive.DirectoryPath,"surface-material.u8"))[0]!=1||prepared.Segments[0]!=section)throw new Exception("Environment evidence missing or generation inputs changed");
            Console.WriteLine("PASS: synchronous pre-native checkpoints/history, bounded complete JSON records, path containment, real height/curve/offset snapshots, URL whitelist and immutable environment artifacts.");
            Console.WriteLine("PASS: independent run archives, append-only events, numerical metrics, persisted outcome, build fingerprint/settings.");
        }
        finally { if (Directory.Exists(root)) Directory.Delete(root, true); }
    }
}
