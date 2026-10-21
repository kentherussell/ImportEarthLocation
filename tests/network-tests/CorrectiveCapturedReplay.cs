using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using System.IO.Compression;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using Unity.Mathematics;

static class CorrectiveCapturedReplay
{
    private sealed class HorizontalComplete : Exception { }
    private static T[] ReadManifest<T>(string folder,string logical)
    {
        var manifest=JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder,logical)));
        if((bool?)manifest["Truncated"]!=false)throw new InvalidDataException("Incomplete captured manifest: "+logical);
        var output=new List<T>();
        foreach(var chunk in manifest["Chunks"]!)
        {
            if((bool?)chunk["Truncated"]!=false)throw new InvalidDataException("Incomplete captured chunk: "+logical);
            var count=0;
            var path=CapturedRunFiles.Resolve(folder,(string)chunk["File"]!);
            using var file=File.OpenRead(path);using var stream=path.EndsWith(".gz",StringComparison.Ordinal)?new GZipStream(file,CompressionMode.Decompress):(Stream)file;
            using var reader=new StreamReader(stream);
            while(reader.ReadLine() is {} line){output.Add(JsonConvert.DeserializeObject<T>(line)!);count++;}
            if(count!=(int)chunk["Records"]!)throw new InvalidDataException("Captured chunk count mismatch: "+logical);
        }
        if(output.Count!=(int)manifest["Records"]!)throw new InvalidDataException("Captured manifest count mismatch: "+logical);
        return output.ToArray();
    }
    public static void Benchmark(string folder,string output)
    {
        var reports=new List<JObject>();
        for(var iteration=0;iteration<4;iteration++)
        {
            Console.WriteLine($"Captured benchmark {(iteration==0?"warmup":iteration+"/3")}");
            var file=output+".run-"+iteration+".json";
            // Exclude the warmup from timing and use it to verify every stage's
            // source connectivity and canonical anchors on the same full input.
            Design(folder,file,benchmark:iteration!=0);
            if(iteration>0)reports.Add(JObject.Parse(File.ReadAllText(file)));
        }
        double Median(IEnumerable<double> values)=>values.OrderBy(v=>v).ElementAt(1);
        var stages=reports.SelectMany(r=>r["stageSeconds"]!.Children<JProperty>().Select(p=>p.Name)).Distinct()
            .ToDictionary(name=>name,name=>Median(reports.Select(r=>(double)r["stageSeconds"]![name]!)));
        File.WriteAllText(output,JsonConvert.SerializeObject(new{warmRepetitions=3,
            medianDesignSeconds=Median(reports.Select(r=>(double)r["designSeconds"]!)),stageMediansSeconds=stages,
            peakProcessWorkingSetBytes=System.Diagnostics.Process.GetCurrentProcess().PeakWorkingSet64,
            diagnosticsMode="scalar findings enabled; archive serialization not reproduced by this offline harness",
            nativeLifecycleWallTimeVerified=false,archiveBytesVerified=false,runs=reports},Formatting.Indented));
        Console.WriteLine("Three-warm-repetition benchmark report: "+output);
    }
    public static void Design(string folder,string output,bool horizontalOnly=false,bool benchmark=false,bool feasibilityOnly=false,bool profileInspection=false,
        NetworkWriterTrace? writerTrace=null)
    {
        if(benchmark&&writerTrace!=null)throw new ArgumentException("Writer inspection cannot qualify performance measurements");
        var input=JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder,"offline-design-pass-01-input.json")));
        if((int?)input["schemaVersion"]!=2||(string?)input["geometryEncoding"]!="prepared-segment-exact-v1"||
            (string?)input["waterSampler"]!="WaterBodyPlanner.SurfaceSampler")throw new InvalidDataException("Unknown full-context replay contract");
        T[] Read<T>(string name)=>ReadManifest<T>(folder,(string)input[name]!);
        var source=Read<PreparedSegment>("networkManifest");if(source.Length!=(int)input["sections"]!)throw new InvalidDataException("Source count mismatch");
        var assets=Read<NativeAssetContract>("nativeContractsManifest").ToDictionary(a=>a.PaletteKey);
        var polygons=Read<GeographicPolygon>("waterPolygonsManifest");
        var water=new WaterPlan(false,0,0,0,"captured"){
            AreaStages=Read<WaterAreaStage>("waterAreasManifest"),ReachStages=Read<WaterReachStage>("waterReachesManifest")};
        var areas=Read<float3[]>("protectedAreasManifest");var lines=Read<float3[]>("protectedLinesManifest");
        var bytes=File.ReadAllBytes(CapturedRunFiles.Resolve(folder,(string)input["terrainFile"]!));
        if(bytes.Length%2!=0||!BitConverter.IsLittleEndian)throw new InvalidDataException("Unsupported terrain encoding");
        var terrain=new ushort[bytes.Length/2];Buffer.BlockCopy(bytes,0,terrain,0,bytes.Length);TerrainSampling.Resolution(terrain);
        var scale=(float)input["HeightScaleMeters"]!;var width=(double)input["widthMeters"]!;
        var log=new ImportLog{ScalarDiagnostics=true};var watch=System.Diagnostics.Stopwatch.StartNew();var nextProgress=0d;var stages=new List<object>();
        var groundContradictions=new List<object>();var rowCensus=new List<object>();var seamWitnesses=new List<object>();var seamCount=0;
        var causal=new List<object>();var originalPorts=source.SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}).ToHashSet();
        var previousJunctions=new Dictionary<(long Node,long Way,string Category),NetworkJunctionQuality.Observation>();
        void JunctionStage(string name,IReadOnlyList<PreparedSegment> sections)
        {
            var current=new Dictionary<(long Node,long Way,string Category),NetworkJunctionQuality.Observation>();
            NetworkJunctionQuality.Audit(sections,(s,start)=>new(start?s.CourseStart:s.CourseEnd,
                math.normalizesafe((double2)(start?s.CourseControl1-s.CourseStart:s.CourseEnd-s.CourseControl2).xz),s.NativeLaneOffsets),
                new ImportLog(),default,"planned",name,value=>
                {
                    if(!originalPorts.Contains(value.Node))return;var key=(value.Node,value.ApproachWay,value.Category);
                    if(current.TryGetValue(key,out var old))value=value with{Value=old.Value.HasValue&&value.Value.HasValue?Math.Max(old.Value.Value,value.Value.Value):null};
                    current[key]=value;
                });
            foreach(var key in previousJunctions.Keys.Concat(current.Keys).Distinct().OrderBy(k=>k.Node).ThenBy(k=>k.Way).ThenBy(k=>k.Category))
            {
                var had=previousJunctions.TryGetValue(key,out var before);var has=current.TryGetValue(key,out var after);
                var failedBefore=had&&before.Value.HasValue&&before.Value.Value>before.Limit;
                var failedAfter=has&&after.Value.HasValue&&after.Value.Value>after.Limit;
                if(!failedBefore&&!failedAfter&&(had==has)&&(!had||before.Value.HasValue==after.Value.HasValue))continue;
                causal.Add(new{stage=name,node=key.Node,approachWay=key.Way,category=key.Category,
                    before=had?before.Value:null,after=has?after.Value:null,
                    classification=!has?"contract-not-observed":!after.Value.HasValue?"unavailable":failedAfter?(failedBefore?"persisting":"introduced-or-inherited-at-input"):"resolved",
                    sourcePortIdentity=true,missingIsNotPassed=true});
            }
            previousJunctions=current;
        }
        if(!benchmark)JunctionStage("selected-input",source);
        var obligations=SourceTopologyObligations.Capture(source);
        var baselineLog=new ImportLog{CaptureRecords=true};SourceTopologyObligations.Validate(obligations,source,baselineLog,default);
        var baselineConnections=baselineLog.Records["source-topology-obligations.ndjson"].Select(JObject.FromObject).ToArray();
        var sampler=WaterBodyPlanner.SurfaceSampler(water,polygons);
        IReadOnlyList<PreparedSegment> result=source;IReadOnlyList<PreparedSegment>? horizontal=null;
        try {result=CoordinatedNetworkDesign.Prepare(source,terrain,scale,width,log,default,sampler,true,assets,
            progress:message=>{if(watch.Elapsed.TotalSeconds>=nextProgress){Console.WriteLine(message);nextProgress=watch.Elapsed.TotalSeconds+10;}},
            protectedAreas:areas,protectedLines:lines,sourceTopology:obligations,inspectStage:benchmark?null:(name,sections)=>
            {
                JunctionStage(name,sections);
                var check=new ImportLog{CaptureRecords=true};SourceTopologyObligations.Validate(obligations,sections,check,default);
                var mismatches=sections.SelectMany(s=>new[]{(Key:new SourceNodeKey(s.Kind,s.StartNodeId),Point:s.Start),(Key:new SourceNodeKey(s.Kind,s.EndNodeId),Point:s.End)})
                    .GroupBy(p=>p.Key).Count(g=>g.Any(p=>!math.all(p.Point==g.First().Point)));
                var connections=check.Records["source-topology-obligations.ndjson"].Select(JObject.FromObject).ToArray();
                var introduced=baselineConnections.Zip(connections).Where(p=>(bool)p.First["reachable"]!&&!(bool)p.Second["reachable"]!).Select(p=>p.Second).ToArray();
                var witnessWays=introduced.Take(4).SelectMany(p=>p["Ways"]!.Values<long>()).ToHashSet();
                stages.Add(new{name,sections=sections.Count,canonicalMismatches=mismatches,source=check.Lines,introducedTopologyFailures=introduced,
                    namedFixtures=NamedFixtures(source,sections),
                    topologySectionWitnesses=sections.Where(s=>witnessWays.Contains(s.WayId)||s.SourceWayIds.Any(witnessWays.Contains)).Select(s=>new{
                        s.WayId,s.SourceWayIds,s.StartNodeId,s.EndNodeId,s.SourceNodeAliases,s.PairedProfileId,s.PairedReferenceBegin,s.PairedReferenceEnd}).ToArray()});
                if(name=="profile-input-and-cover")NetworkGroundFeasibility.Check(sections,terrain,scale,width,default,
                    witness=>{if(groundContradictions.Count<64)groundContradictions.Add(witness);});
                if(horizontalOnly&&name=="paired-corridors"||feasibilityOnly&&name=="profile-input-and-cover")
                {horizontal=sections;throw new HorizontalComplete();}
            },inspectProfileRows:profileInspection?(scope,rows)=>rowCensus.Add(new {scope,report=ProfileRowCensus.Inspect(rows)}):null,
            inspectProfileSeam:profileInspection?witness=>{seamCount++;if(seamWitnesses.Count<128)seamWitnesses.Add(witness);}:null,
            inspectWriter:writerTrace==null?null:writerTrace.Observe);}
        catch(HorizontalComplete) when((horizontalOnly||feasibilityOnly)&&horizontal!=null){result=horizontal;}
        var designSeconds=watch.Elapsed.TotalSeconds;
        SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(source),result,log,default);
        NetworkQualityAudit.Planned(result,log,default,true);
        // Original source ports are stable across changed native subdivisions.
        // Keep unmapped generated ports separate; this is not an all-map issue
        // total while continuous contact/cover intervals lack exact mappings.
        var localizedJunctions=new Dictionary<(long Node,long Way,string Category),NetworkJunctionQuality.Observation>();
        NetworkJunctionQuality.Audit(result,(s,start)=>new(start?s.CourseStart:s.CourseEnd,
            math.normalizesafe((double2)(start?s.CourseControl1-s.CourseStart:s.CourseEnd-s.CourseControl2).xz),s.NativeLaneOffsets),
            new ImportLog(),default,"planned","issue-inventory",value=>
            {
                var key=(value.Node,value.ApproachWay,value.Category);
                if(localizedJunctions.TryGetValue(key,out var old)&&old.Value.HasValue&&value.Value.HasValue&&old.Value.Value>value.Value.Value)return;
                localizedJunctions[key]=value;
            });
        var localizedIssues=localizedJunctions.Values.Where(v=>!v.Value.HasValue||v.Value.Value>v.Limit)
            .OrderBy(v=>v.Node).ThenBy(v=>v.ApproachWay).ThenBy(v=>v.Category).Select(v=>new
            {v.Node,v.ApproachWay,v.Category,v.Value,v.Limit,mappingState=originalPorts.Contains(v.Node)?"ExactSourcePort":"UnmappedGeneratedPort",
                evidenceState=v.Value.HasValue?"Failed":"Unavailable"}).ToArray();
        var physicalIntervals=profileInspection&&!horizontalOnly&&!feasibilityOnly?
            PhysicalIssueInventory.Capture(source,result,terrain,scale,width,assets):null;
        File.WriteAllText(output,JsonConvert.SerializeObject(new{capture=Path.GetFileName(folder),sourceBoundary=(string?)input["sourceBoundary"],
            completeWaterAndProtectionContext=true,sourceSections=source.Length,finalSections=result.Count,
            findings=log.Findings,diagnostics=log.Lines,stages,groundContradictions,junctionCausalTrace=causal,performanceBenchmark=benchmark,nativeAcceptanceVerified=false,designSeconds,
            rowCensus,seamWitnesses,seamCount,seamWitnessCoverage=seamCount<=seamWitnesses.Count?"complete":"bounded",
            localizedIssues,physicalIntervals,writerTrace=writerTrace?.Stages,completeMapIssueInventory=false,
            physicalIntervalMapping=physicalIntervals==null?"NotRequested":"FinalSectionParameters; exact selected-source parameters unavailable",
            offlineAssemblyMvid=typeof(CoordinatedNetworkDesign).Assembly.ManifestModule.ModuleVersionId,
            peakProcessWorkingSetBytes=System.Diagnostics.Process.GetCurrentProcess().PeakWorkingSet64,
            runtime=Environment.Version.ToString(),logicalProcessors=Environment.ProcessorCount,
            namedFixtures=NamedFixtures(source,result),
            pairedConstruction=log.Records.TryGetValue("paired-construction",out var construction)?construction:Array.Empty<object>(),
            stageSeconds=log.Lines.Where(l=>l.StartsWith("DESIGN STAGE name=",StringComparison.Ordinal)).Select(l=>l.Split(';'))
                .ToDictionary(parts=>parts[0].Substring("DESIGN STAGE name=".Length),parts=>double.Parse(parts[1].Split('=')[1],System.Globalization.CultureInfo.InvariantCulture)),
            verticalSolveReplayed=!horizontalOnly&&!feasibilityOnly,
            finite=result.All(s=>NetworkDesignChecks.Finite(new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd)))},Formatting.Indented));
        Console.WriteLine("Full-context corrective correctness report: "+output);
    }
    private static object[] NamedFixtures(IReadOnlyList<PreparedSegment> input,IReadOnlyList<PreparedSegment> output)
    {
        var paired=output.Where(s=>s.PairedProfileId!=0).GroupBy(s=>(s.PairedProfileId,
            Begin:Math.Round(Math.Min(s.PairedReferenceBegin,s.PairedReferenceEnd),6),End:Math.Round(Math.Max(s.PairedReferenceBegin,s.PairedReferenceEnd),6)))
            .ToDictionary(g=>g.Key,g=>g.ToArray());
        return new[]{71335603L,27774024L,4352239L,755326574L,112835797L,6310467L,755328910L}.Select(way=>
        {
            bool Contains(PreparedSegment s)=>s.WayId==way||s.SourceWayIds.Contains(way);
            var sections=output.Where(Contains).ToArray();var checkedPairs=0;var unknownPairs=0;var maximumHeight=0d;var minimumMedian=double.PositiveInfinity;
            foreach(var key in sections.Where(s=>s.PairedProfileId!=0).Select(s=>(s.PairedProfileId,
                Begin:Math.Round(Math.Min(s.PairedReferenceBegin,s.PairedReferenceEnd),6),End:Math.Round(Math.Max(s.PairedReferenceBegin,s.PairedReferenceEnd),6))).Distinct())
            {
                var pair=paired[key];if(pair.Length!=2){unknownPairs++;continue;}checkedPairs++;
                var a=pair[0];var b=pair[1];
                var same=Math.Sign(a.PairedReferenceEnd-a.PairedReferenceBegin)==Math.Sign(b.PairedReferenceEnd-b.PairedReferenceBegin);
                for(var j=0;j<=64;j++)
                {
                    var pa=NetworkNormalizer.Evaluate(a,j/64f);var pb=NetworkNormalizer.Evaluate(b,same?j/64f:1-j/64f);
                    maximumHeight=Math.Max(maximumHeight,Math.Abs((double)pa.y-pb.y));
                    minimumMedian=Math.Min(minimumMedian,math.distance(pa.xz,pb.xz)-(a.NativeWidth+b.NativeWidth)/2);
                }
            }
            return (object)new{way,inputSections=input.Count(Contains),outputSections=sections.Length,checkedPairs,unknownPairs,
                confirmedSections=sections.Count(s=>s.PairedProfileId!=0),spacingCertifiedSections=sections.Count(s=>s.PairedProfileId!=0&&!s.PairedProfileOnly),
                pairedObservationState=checkedPairs==0?"Unavailable":"Observed-not-native-accepted",maximumSampledPairedHeightDifferenceMeters=maximumHeight,
                minimumSampledMedianMeters=SparseProfileRows.Finite(minimumMedian)?(double?)minimumMedian:null,
                sampledIsNotContinuousCertification=true,nativeAcceptanceVerified=false};
        }).ToArray();
    }
    public static void Run(string folder,string output)
    {
        IEnumerable<PreparedSegment> Read(string logical)
        {
            var manifest=JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder,logical+"-chunks.json")));
            if((bool?)manifest["Truncated"]!=false)throw new InvalidDataException("Truncated corrective replay source");
            var count=0;
            foreach(var chunk in manifest["Chunks"]!)
            {
                if((bool?)chunk["Truncated"]!=false)throw new InvalidDataException("Truncated corrective replay chunk");
                foreach(var record in CapturedRunFiles.Records(CapturedRunFiles.Resolve(folder,(string)chunk["File"]!)))
                {count++;yield return record.ToObject<PreparedSegment>()??throw new InvalidDataException("Missing prepared segment");}
            }
            if(count!=(int)manifest["Records"]!)throw new InvalidDataException("Corrective source count mismatch");
        }
        var source=Read("offline-design-pass-01-entry").ToArray();
        var baseline=new ImportLog{CaptureRecords=true};
        var obligations=SourceTopologyObligations.Capture(source);SourceTopologyObligations.Validate(obligations,source,baseline,default);
        var log=new ImportLog{CaptureRecords=true};
        var terminal=NetworkRailTerminalFormations.Prepare(source,log,default);
        SourceTopologyObligations.Validate(obligations,terminal,log,default);
        var budgeted=NetworkVisualDesign.FitBudgets(terminal,57344,Array.Empty<float3[]>(),Array.Empty<float3[]>(),null,log,default);
        var paired=NetworkPairedCorridors.Prepare(budgeted,log,default);
        var aliasConflicts=terminal.SelectMany(s=>s.SourceNodeAliases.Select(a=>new{kind=s.Kind,a.SourceNodeId,a.CanonicalNodeId}))
            .GroupBy(a=>(a.kind,a.SourceNodeId)).Where(g=>g.Select(a=>a.CanonicalNodeId).Distinct().Count()>1)
            .Select(g=>new{g.Key,canonical=g.Select(a=>a.CanonicalNodeId).Distinct().ToArray()}).ToArray();
        var formations=terminal.Where(s=>s.RailTerminalFormationId!=0).GroupBy(s=>s.RailTerminalFormationId)
            .Select(g=>new{id=g.Key,sections=g.Count(),junctionDegree=g.Count(s=>s.StartNodeId==g.Key||s.EndNodeId==g.Key),ways=g.SelectMany(s=>s.SourceWayIds).Distinct().Count()}).ToArray();
        var ways=new[]{71335603L,27774024L,4352239L,755326574L,112835797L,6310467L,755328910L};
        var partners=ways.Select(way=>new{way,sourceSections=source.Count(s=>s.WayId==way||s.SourceWayIds.Contains(way)),
            sections=paired.Where(s=>s.WayId==way||s.SourceWayIds.Contains(way)).Select(s=>new{s.StartNodeId,s.EndNodeId,s.PairedProfileId,s.PairedProfileOnly,s.PairedReferenceBegin,s.PairedReferenceEnd}).ToArray()}).ToArray();
        File.WriteAllText(output,JsonConvert.SerializeObject(new{capture=Path.GetFileName(folder),input=source.Length,output=paired.Count,formations,partners,
            findings=log.Findings,diagnostics=log.Lines,baselineDiagnostics=baseline.Lines,aliasConflicts,
            scope="complete-captured-pre-design-source-formation-correctness",
            protectedGeographyNotReplayed=true,verticalSolveNotReplayed=true,nativeAcceptanceVerified=false,performanceBenchmark=false},Formatting.Indented));
        foreach(var formation in terminal.Where(s=>s.RailTerminalFormationId!=0).GroupBy(s=>s.RailTerminalFormationId))
            if(formation.SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}).GroupBy(n=>n).Any(g=>g.Count()>3))
                throw new Exception("Terminal macrograph contains a non-simple junction");
        var previous=baseline.Records["source-topology-obligations.ndjson"].Select(JObject.FromObject).ToArray();
        var next=log.Records["source-topology-obligations.ndjson"].Select(JObject.FromObject).ToArray();
        if(previous.Length!=next.Length||previous.Zip(next).Any(p=>(bool)p.First["reachable"]!&&!(bool)p.Second["reachable"]!)||
            log.Metric("Missing mandatory selected-source ports")>baseline.Metric("Missing mandatory selected-source ports"))
            throw new Exception("Terminal contraction introduced a new captured source connectivity failure; see "+output);
        Console.WriteLine($"Captured formation correctness: {source.Length}->{paired.Count}; terminal formations={formations.Length}; report={output}");
    }
}
