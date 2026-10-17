using System.Security.Cryptography;
using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

static class CapturedSourceCleanup
{
    static void NormalizeIds(JToken token)
    {
        if(token is JObject obj)foreach(var property in obj.Properties().ToArray())
        {
            if(property.Name is "chainId" or "nodeId" or "WayId")property.Value=property.Value.ToString();
            else NormalizeIds(property.Value);
        }
        else if(token is JArray array)foreach(var child in array)NormalizeIds(child);
    }
    public static void Run(string folder,string cachePath,string output)
    {
        var watch=System.Diagnostics.Stopwatch.StartNew();var raw=OsmParsedCache.LoadSnapshot(cachePath,default);
        var run=JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder,"run.json")));
        var request=run["settings"]!.ToObject<ImportRequest>()!;
        var immutable=NetworkSourceManifest.Build(raw,request,default);
        var manifest=JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder,"immutable-source-network-chunks.json")));
        if((bool?)manifest["Truncated"]!=false)throw new InvalidDataException("Incomplete immutable source capture");
        IEnumerable<JObject> Expected()
        {
            var total=0;
            foreach(var chunk in manifest["Chunks"]!)
            {
                if((bool?)chunk["Truncated"]!=false)throw new InvalidDataException("Incomplete immutable source chunk");
                var count=0;foreach(var record in CapturedRunFiles.Records(CapturedRunFiles.Resolve(folder,(string)chunk["File"]!)))
                {count++;total++;yield return record;}
                if(count!=(int)chunk["Records"]!)throw new InvalidDataException("Immutable source chunk count mismatch");
            }
            if(total!=(int)manifest["Records"]!)throw new InvalidDataException("Immutable source coverage mismatch");
        }
        using(var expected=Expected().GetEnumerator())using(var actual=DiagnosticArtifacts.SourceRecords(immutable).GetEnumerator())
        {
            var ordinal=0;
            while(expected.MoveNext())
            {
                if(!actual.MoveNext())throw new InvalidDataException("Candidate cache has fewer source records");
                var left=expected.Current;var right=JObject.FromObject(actual.Current);NormalizeIds(left);NormalizeIds(right);
                if(!JToken.DeepEquals(left,right))throw new InvalidDataException("Candidate cache differs from immutable source record "+ordinal);
                ordinal++;
            }
            if(actual.MoveNext())throw new InvalidDataException("Candidate cache has extra source records");
        }
        var log=new ImportLog();var stages=new List<object>();SourceTopologyManifest? topology=null;
        IReadOnlyList<PreparedSegment> segments=NetworkProcessor.BuildSegments(NetworkEvidence.Normalize(raw,request,log,default),request,log);
        void Stage(string name,Action action)
        {
            var timer=System.Diagnostics.Stopwatch.StartNew();action();var seconds=timer.Elapsed.TotalSeconds;
            var audit=new ImportLog{CaptureRecords=true};
            if(topology!=null)SourceTopologyObligations.Validate(topology,segments,audit,default);
            var witnesses=audit.Records.Values.SelectMany(r=>r).Select(JObject.FromObject).Where(r=>(bool?)r["reachable"]==false).Take(16).ToArray();
            var witnessWays=witnesses.SelectMany(r=>r["Ways"]!.Values<long>()).ToHashSet();
            var witnessSections=segments.Where(s=>s.Kind==NetworkKind.Rail&&(witnessWays.Contains(s.WayId)||s.SourceWayIds.Any(witnessWays.Contains)))
                .Select(s=>new{s.WayId,s.StartNodeId,s.EndNodeId,s.SourceWayIds,s.SourceNodeAliases}).ToArray();
            stages.Add(new{name,seconds,sections=segments.Count,railSections=segments.Count(s=>s.Kind==NetworkKind.Rail),
                missingPorts=audit.Metric("Missing mandatory selected-source ports"),unresolvedRoutes=audit.Metric("Unresolved selected-source reachability obligations"),witnesses});
            if(witnessSections.Length>0)File.WriteAllText(output+"."+name+"-witnesses.json",JsonConvert.SerializeObject(witnessSections,Formatting.Indented));
        }
        Stage("tunnel-intervals",()=>segments=CoordinatedNetworkDesign.TunnelIntervals(segments,log,default));
        topology=SourceTopologyObligations.Capture(segments);
        Stage("overlap-consolidation",()=>segments=NetworkOverlapConsolidator.Prepare(segments,log,default));
        Stage("avenue-consolidation",()=>segments=AvenueConsolidator.Prepare(segments,log,default));
        Stage("roundabouts",()=>segments=RoundaboutPlanner.Prepare(segments,log,default));
        var flags=System.Reflection.BindingFlags.Static|System.Reflection.BindingFlags.NonPublic;
        var demands=typeof(RailCorridorConsolidator).GetMethod("ExactPhysicalDemands",flags)!;
        var pass=typeof(RailCorridorConsolidator).GetMethod("PreparePass",flags)!;
        Stage("rail-physical-demands",()=>
        {
            segments=segments.Select(s=>s.Kind==NetworkKind.Rail?s with{Lanes=2,PhysicalTracks=2,OneWay=false,ExplicitWallSides=NetworkSides.None,RetainingWallSides=NetworkSides.None}:s).ToArray();
            segments=(IReadOnlyList<PreparedSegment>)demands.Invoke(null,new object[]{segments,log,default(CancellationToken)})!;
        });
        for(var iteration=0;iteration<8;iteration++)
        {
            var before=segments;
            Stage("rail-pass-"+iteration,()=>segments=(IReadOnlyList<PreparedSegment>)pass.Invoke(null,new object[]{segments,log,default(CancellationToken)})!);
            if(segments.SequenceEqual(before))break;
        }
        Stage("selected-rail-policy",()=>segments=StarterNetworkPolicy.Rail(segments,log,default));
        Stage("normalization",()=>segments=NetworkNormalizer.Normalize(segments,log,default,request.MapSize.WidthMeters));
        Stage("hardening",()=>segments=NetworkHardener.Prepare(segments,log,default,request.ImportRail));
        Stage("geometry-safety",()=>segments=OsmRecovery.RepairGeometry(segments,request.MapSize.WidthMeters,log,default));
        SourceTopologyObligations.Validate(topology,segments,log,default);
        using var sha=SHA256.Create();using var cache=File.OpenRead(cachePath);
        File.WriteAllText(output,JsonConvert.SerializeObject(new{capture=Path.GetFileName(folder),cacheSha256=Convert.ToHexString(sha.ComputeHash(cache)),
            immutableSourceMatchedExactly=true,scope="live-source-cleanup-and-normalization-before-native-metadata",stages,
            metrics=new{missingPorts=log.Metric("Missing mandatory selected-source ports"),unresolvedRoutes=log.Metric("Unresolved selected-source reachability obligations"),
                consolidatedRailSpans=log.Metric("Rail parallel spans consolidated")},
            sourceDiagnostics=log.Lines.Where(l=>l.StartsWith("RAIL TOTAL")||l.StartsWith("SOURCE OBLIGATIONS")).ToArray(),
            elapsedSeconds=watch.Elapsed.TotalSeconds,profileSolved=false,terrainOrWaterReplayed=false,nativeOutcomePredicted=false},Formatting.Indented));
        Console.WriteLine("Source cleanup replay saved: "+output);
    }
}
