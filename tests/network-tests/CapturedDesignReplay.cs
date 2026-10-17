using System.IO.Compression;
using System.Security.Cryptography;
using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using Unity.Mathematics;

static class CapturedDesignReplay
{
    static string Child(string folder,string name)
    {
        if(name!=Path.GetFileName(name))throw new InvalidDataException("Capture references a file outside its run");
        return Path.Combine(folder,name);
    }
    static IEnumerable<JObject> Lines(string path)
    {
        using var file=File.OpenRead(path);
        using var stream=path.EndsWith(".gz",StringComparison.Ordinal)?new GZipStream(file,CompressionMode.Decompress):(Stream)file;
        using var reader=new StreamReader(stream);while(reader.ReadLine() is {} line)yield return JObject.Parse(line);
    }
    public static void Run(string folder,string output,bool full,bool feasibilityOnly=false)
    {
        var input=JObject.Parse(File.ReadAllText(Path.Combine(folder,"profile-input.json")));
        if((bool?)input["geometryTruncated"]!=false||(bool?)input["assetsTruncated"]!=false)throw new InvalidDataException("Incomplete captured design input");
        var manifest=JObject.Parse(File.ReadAllText(Child(folder,(string)input["networkManifest"]!)));
        if((bool?)manifest["Truncated"]!=false)throw new InvalidDataException("Incomplete geometry manifest");
        var source=new List<PreparedSegment>();var truncatedSourceReferences=0;
        foreach(var chunk in manifest["Chunks"]!)
        {
            if((bool?)chunk["Truncated"]!=false)throw new InvalidDataException("Incomplete geometry chunk");
            var count=0;
            foreach(var record in Lines(Child(folder,(string)chunk["File"]!)))
            {
                count++;var s=record.ToObject<PreparedSegment>()??throw new InvalidDataException("Unreadable captured section");
                float3 Point(string key)=>record[key]!.ToObject<float3>();
                // Legacy records expose Deck* observations; exact model records
                // retain offsets, policy fields and complete source references.
                if(record["DeckStart"]==null&&((string?)input["geometryEncoding"]!="prepared-segment-exact-v1"||record["Start"]==null||record["End"]==null))
                    throw new InvalidDataException("Unknown/incomplete captured geometry encoding");
                source.Add(record["DeckStart"]!=null?s with{FinalDeckCurve=new(Point("DeckStart"),Point("DeckControl1"),Point("DeckControl2"),Point("DeckEnd"))}:s);
                if((bool?)record["SourcePointsTruncated"]==true)truncatedSourceReferences++;
            }
            if(count!=(int)chunk["Records"]!)throw new InvalidDataException("Geometry chunk record count mismatch");
        }
        if(source.Count!=(int)manifest["Records"]!||source.Count!=(int)input["sections"]!)throw new InvalidDataException("Geometry manifest coverage mismatch");
        var assets=Lines(Child(folder,(string)input["assetsFile"]!)).Select(r=>r.ToObject<NativeAssetContract>()!).ToDictionary(a=>a.PaletteKey);
        var bytes=File.ReadAllBytes(Child(folder,(string)input["terrainFile"]!));var terrain=new ushort[bytes.Length/2];
        if(bytes.Length%2!=0||!BitConverter.IsLittleEndian)throw new InvalidDataException("Unsupported terrain encoding");
        Buffer.BlockCopy(bytes,0,terrain,0,bytes.Length);TerrainSampling.Resolution(terrain);
        var scale=(float)input["heightScaleMeters"]!;var width=(double)input["mapWidthMeters"]!;
        if(feasibilityOnly)
        {
            var witnesses=new List<object>();var clock=System.Diagnostics.Stopwatch.StartNew();
            var feasibility=NetworkGroundFeasibility.Check(source,terrain,scale,width,default,value=>{if(witnesses.Count<128)witnesses.Add(value);});
            File.WriteAllText(output,JsonConvert.SerializeObject(new{capture=Path.GetFileName(folder),scope="captured-fixed-horizontal-ground-mode",
                feasibility,witnesses,elapsedSeconds=clock.Elapsed.TotalSeconds,necessaryConditionOnly=true,solverRerun=false,
                rawOsmReplay=false,truncatedSourceReferences},Formatting.Indented));
            Console.WriteLine($"Ground feasibility: {feasibility.Conflicts}/{feasibility.Checked} endpoint incompatibilities; report={output}");return;
        }
        var reports=new List<object>();
        var ways=full?new[]{0L}:new[]{4352998L,180832046L,220548747L,1025720515L};
        foreach(var way in ways)
        {
            var selected=source.Where(s=>way==0||s.WayId==way||s.SourceWayIds.Contains(way)).ToArray();
            if(selected.Length==0){reports.Add(new{way,state="not-in-this-capture"});continue;}
            var low=new float2(float.PositiveInfinity);var high=new float2(float.NegativeInfinity);
            foreach(var s in selected)foreach(var p in new[]{s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd}){low=math.min(low,p.xz);high=math.max(high,p.xz);}
            var subset=full?source.ToArray():source.Where(s=>
                math.all(math.max(math.max(s.CourseStart.xz,s.CourseControl1.xz),math.max(s.CourseControl2.xz,s.CourseEnd.xz))>=low-200)&&
                math.all(math.min(math.min(s.CourseStart.xz,s.CourseControl1.xz),math.min(s.CourseControl2.xz,s.CourseEnd.xz))<=high+200)).ToArray();
            var log=new ImportLog();var watch=System.Diagnostics.Stopwatch.StartNew();var nextProgress=0d;
            // This is a rebuild from the captured horizontal boundary, not raw
            // OSM acquisition. Re-derive pairing/profile families rather than
            // accidentally aliasing old family numbers into the new formulation.
            var rebuild=subset.Select(s=>s with{PairedSpanId=0,PairedProfileId=0,PairedCorridorId=0,StartHeightFamily=0,EndHeightFamily=0,ReferenceSpan=0}).ToArray();
            var result=CoordinatedNetworkDesign.Prepare(rebuild,terrain,scale,width,log,default,structureIntervalsResolved:true,assets:assets,
                progress:message=>{if(watch.Elapsed.TotalSeconds>=nextProgress){Console.WriteLine($"CAPTURE {way}: {message}; elapsed={watch.Elapsed.TotalSeconds:F1}s");nextProgress=watch.Elapsed.TotalSeconds+10;}});
            NetworkQualityAudit.Planned(result,log,default,false);
            var physical=NetworkTunnelProfile.MeasurePhysical(result,terrain,scale,width,assets,default);
            reports.Add(new{way,inputSections=subset.Length,targetSections=selected.Length,outputSections=result.Count,elapsedSeconds=watch.Elapsed.TotalSeconds,
                finite=result.All(s=>NetworkDesignChecks.Finite(new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd))),
                findings=log.Findings,physical,diagnostics=log.Lines.Where(l=>l.StartsWith("PROFILE DESIGN")||l.StartsWith("PHYSICAL RECONSTRUCTION")||l.StartsWith("COORDINATED DESIGN")||l.StartsWith("V-PROFILE JOINT")).ToArray()});
            Console.WriteLine($"CAPTURE {way}: completed; sections {subset.Length}->{result.Count}; {watch.Elapsed.TotalSeconds:F1}s; grade failures={physical.ExceptionalGradeFailures}; clearance failures={physical.ClearanceFailures}; cover failures={physical.CoverFailures}");
            Save();
        }
        Save();
        void Save()
        {
            using var hash=SHA256.Create();using var assembly=File.OpenRead(typeof(CapturedDesignReplay).Assembly.Location);
            File.WriteAllText(output,JsonConvert.SerializeObject(new{capture=Path.GetFileName(folder),scope=full?"whole-captured-horizontal-rebuild":"named-ways-with-200m-captured-neighbours",
                inputSections=source.Count,truncatedSourceReferences,rawOsmReplay=false,omittedWaterSnapshot=true,nativeRenderingVerified=false,
                rebuiltPairFamilies=true,fixtureNeighbourBoundaryHeightsNotCommitted=true,
                assemblySha256=Convert.ToHexString(hash.ComputeHash(assembly)),reports},Formatting.Indented));
        }
    }
}
