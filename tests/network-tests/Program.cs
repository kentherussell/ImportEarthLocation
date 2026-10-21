using ImportEarthLocation.Core;
using ImportEarthLocation.Data;
using ImportEarthLocation.Geo;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

static class Program
{
    static void Check(bool value,string message){if(!value)throw new Exception(message);}
    static PreparedSegment S(long way,long start,long end,float x1,float z1,float x2,float z2,NetworkKind kind=NetworkKind.Road,bool oneWay=true)=>new(way,start,end,new(x1,0,z1),new(x2,0,z2),kind,"primary",2,oneWay,false,false,0){Name="Main",SourceGeometry=new[]{new float3(x1,0,z1),new float3(x2,0,z2)}};
    static int Main(string[] args)
    {
        try { Run(args); return 0; }
        catch(Exception e) { Console.Error.WriteLine(e); return 1; }
    }
    static void Run(string[] args)
    {
        if(args.Length==1&&args[0]=="--writer-trace-only"){NetworkWriterTrace.Test();PairedPublicationTraceTests.Run();return;}
        if(args.Length==4&&args[0] is "--corrective-writer-trace" or "--corrective-writer-trace-horizontal")
        {
            var ways=args[3].Split(',').Select(value=>long.Parse(value,System.Globalization.CultureInfo.InvariantCulture));
            CorrectiveCapturedReplay.Design(args[1],args[2],horizontalOnly:args[0].EndsWith("-horizontal",StringComparison.Ordinal),
                writerTrace:new NetworkWriterTrace(ways));return;
        }
        if(args.Length==1&&args[0]=="--profile-census-only"){ProfileRowCensus.Test();ProfileCouplingCensus.Test();return;}
        if(args.Length==3&&args[0]=="--profile-coupling-census")
        {
            ProfileCouplingCensus.Test();
            ProfileReplay.Inspect(args[1],(rows,prior,stars,scale)=>File.WriteAllText(args[2],
                Newtonsoft.Json.JsonConvert.SerializeObject(ProfileCouplingCensus.Analyze(rows,prior,stars),Newtonsoft.Json.Formatting.Indented)));
            return;
        }
        if(args.Length==3&&args[0]=="--profile-evaluation-kernels"){ProfileReplay.BenchmarkEvaluation(args[1],args[2]);return;}
        if(args.Length==3&&args[0]=="--corrective-census"){CorrectiveCapturedReplay.Design(args[1],args[2],profileInspection:true);return;}
        if(args.Length==1&&args[0]=="--corrective-plan-only"){CorrectivePlanTests.Run();return;}
        if(args.Length==3&&args[0]=="--corrective-capture"){CorrectiveCapturedReplay.Run(args[1],args[2]);return;}
        if(args.Length==3&&args[0]=="--corrective-design"){CorrectiveCapturedReplay.Design(args[1],args[2]);return;}
        if(args.Length==3&&args[0]=="--corrective-horizontal"){CorrectiveCapturedReplay.Design(args[1],args[2],true);return;}
        if(args.Length==3&&args[0]=="--corrective-benchmark"){CorrectiveCapturedReplay.Benchmark(args[1],args[2]);return;}
        if(args.Length==3&&args[0]=="--corrective-timing"){CorrectiveCapturedReplay.Design(args[1],args[2],benchmark:true);return;}
        if(args.Length==3&&args[0]=="--corrective-feasibility"){CorrectiveCapturedReplay.Design(args[1],args[2],feasibilityOnly:true);return;}
        if(args.Length==1&&args[0]=="--visual-design-only"){VisualDesignTests.Run();return;}
        if(args.Length==4&&args[0]=="--compare-run-quality"){NetworkQualityComparison.Run(args[1],args[2],args[3]);return;}
        if(args.Length==1&&args[0]=="--paired-corridors-only"){PairedCorridorTests.Run();return;}
        if(args.Length==3&&args[0]=="--captured-problem-audit"){CapturedProblemAudit.Run(args[1],args[2]);return;}
        if(args.Length==3&&args[0]=="--profile-kernels"){ProfileReplay.BenchmarkKernels(args[1],args[2]);return;}
        if(args.Length==3&&args[0]=="--native-physical-replay"){CapturedNativePhysicalReplay.Run(args[1],args[2]);return;}
        if(args.Length==3&&args[0]=="--captured-feasibility"){CapturedDesignReplay.Run(args[1],args[2],true,true);return;}
        if(args.Length==4&&args[0]=="--source-cleanup-replay"){CapturedSourceCleanup.Run(args[1],args[2],args[3]);return;}
        if(args.Length==4&&args[0]=="--route-cleanup-replay"){CapturedSourceCleanup.Run(args[1],args[2],args[3],true);return;}
        if(args.Length==3&&(args[0]=="--captured-fixtures"||args[0]=="--captured-design"))
        {CapturedDesignReplay.Run(args[1],args[2],args[0]=="--captured-design");return;}
        if(args.Length==1&&args[0]=="--critical-rework-only"){CriticalReworkTests.Run();return;}
        if(args.Length==1&&args[0]=="--offline-completion-only"){OfflineCompletionTests.Run();return;}
        ProfileCouplingCensus.Test();
        NetworkWriterTrace.Test();
        PairedPublicationTraceTests.Run();
        CorrectivePlanTests.Run();
        CriticalReworkTests.Run();
        VisualDesignTests.Run();
        OfflineCompletionTests.Run();
        if(args.Length==1&&args[0]=="--corridor-quality-only"){CorridorQualityTests.Run();return;}
        if(args.Length==2&&args[0]=="--corridor-quality-replay"){CorridorQualityTests.Replay(args[1]);return;}
        if(args.Length==1&&args[0]=="--design-math-only"){DesignMathTests.Run();return;}
        if(args.Length==1&&args[0]=="--profile-math-only"){ProfileMathTests.Run();return;}
        if(args.Length==2&&args[0]=="--profile-replay"){ProfileReplay.Run(args[1]);return;}
        if(args.Length==2&&args[0]=="--audit-local-compaction")
        {NativeImportRegressionTests.ReplayLocalCompaction(args[1]);return;}
        if(args.Length>=2&&args[0]=="--audit-compaction")
        {
            foreach(var folder in args.Skip(1))NativeImportRegressionTests.ReplayCompaction(folder);
            return;
        }
        if(args.Length>=2&&args[0]=="--audit-junction-plans")
        {
            foreach(var folder in args.Skip(1))NativeBatchTests.ReplayJunctionPlan(folder);
            return;
        }
        if(args.Length==2&&args[0]=="--audit-native-batch"){NativeBatchTests.Replay(args[1]);return;}
        if(args.Length>=2&&args[0]=="--audit-native-batches")
        {
            foreach(var folder in args.Skip(1))NativeBatchTests.Replay(folder);
            return;
        }
        if(args.Length>0&&(args[0]=="--audit-native-batch"||args[0]=="--audit-native-batches"||args[0]=="--audit-junction-plans"||args[0]=="--native-bindings-only")&&!(args.Length==1&&args[0]=="--native-bindings-only"))
            throw new ArgumentException("Native replay needs a captured folder; --native-bindings-only takes no additional arguments.");
        NativeBatchTests.Run();
        NativeImportRegressionTests.Run();
        if(args.Length==1&&args[0]=="--native-bindings-only")return;
        RepairTests.Run();
        SafetyTests.Run();
        if(args.Length==3&&args[0]=="--audit-diagnostic"){CorridorTests.AuditDiagnostic(args[1],args[2]);return;}
        if(args.Length==2&&args[0]=="--cached-osm"){CorridorTests.CachedMap(args[1]);return;}
        if(args.Length==2&&args[0]=="--cached-tiles"){CorridorTests.CachedMap(args[1],cachedTiles:true);return;}
        if(args.Length==4&&args[0]=="--cached-location"){CorridorTests.CachedMap(args[1],cachedTiles:true,latitude:double.Parse(args[2],System.Globalization.CultureInfo.InvariantCulture),longitude:double.Parse(args[3],System.Globalization.CultureInfo.InvariantCulture),level:RoadImportLevel.Secondary);return;}
        if(args.Length==4&&args[0]=="--cached-osm"&&args[2]=="--snapshot"){CorridorTests.CachedMap(args[1],args[3]);return;}
        if(args.Length==2&&args[0]=="--audit-snapshot"){CorridorTests.AuditSnapshot(args[1]);return;}
        if(args.Length==2&&args[0]=="--repair-snapshot"){CorridorTests.AuditSnapshot(args[1],true);return;}
        var log=new ImportLog();var request=new ImportRequest(0,0,MapSizeProfile.Vanilla,false);
        var nodes=new Dictionary<long,OsmNode>();var ways=new Dictionary<long,OsmWay>();
        // Explicit fixture includes excluded street links as negative cases.
        // Road levels are now independent flags, not the old cumulative enum.
        var classes=new[]{"motorway","motorway_link","trunk","trunk_link","primary","primary_link","secondary","secondary_link","tertiary"};
        for(var i=0;i<=classes.Length;i++){var id=i*2L+1;nodes[id]=new(id,0,i*.001);nodes[id+1]=new(id+1,.0001,i*.001);ways[i+1]=new(i+1,new[]{id,id+1},new Dictionary<string,string>{{i==classes.Length?"railway":"highway",i==classes.Length?"rail":classes[i]}});}
        var data=new OsmDataset{Nodes=nodes,Ways=ways};
        foreach(var level in Enum.GetValues<RoadImportLevel>())foreach(var rail in new[]{false,true})
        {
            var selected=request with{RoadLevel=level,ImportRail=rail};var result=NetworkProcessor.BuildSegments(data,selected,log);
            var expected=((level&RoadImportLevel.Highways)!=0?2:0)+((level&RoadImportLevel.Routes)!=0?((level&RoadImportLevel.Streets)!=0?2:1):0)+((level&RoadImportLevel.Streets)!=0?3:0);
            Check(result.Count(s=>s.Kind==NetworkKind.Road)==expected&&result.Count(s=>s.Kind==NetworkKind.Rail)==(rail?1:0),"Road/rail independence failed");
            var query=OverpassClient.BuildNetworkQuery(new(-1,-1,1,1),level,rail);
            Check(query.Contains("way[\"highway\"~\"^("+NetworkSelection.Classes(level))== (level!=RoadImportLevel.None)&&query.Contains("way[\"railway\"=\"rail\"]")==rail,"Disabled selected network type fetched");
            Check(!query.Contains("forest")&&!query.Contains("wood"),"Optional forest acquisition leaked into required networks");
            var forestQuery=OverpassClient.BuildForestQuery(new(-1,-1,1,1));
            Check(forestQuery.Contains("forest")&&forestQuery.Contains("wood")&&!forestQuery.Contains("highway")&&!forestQuery.Contains("railway"),"Forest query includes networks or loses cover");
        }
        Check(new ImportRequest(0,0,MapSizeProfile.Vanilla,true).RoadLevel==RoadImportLevel.None,"Legacy constructor enabled independent groups without an explicit selection");
        Check(NetworkPreferences.Parse("{}")==NetworkPreferences.Defaults&&NetworkPreferences.Parse("broken")==NetworkPreferences.Defaults,"Corrupt/missing preferences unsafe");
        Check(NetworkPreferences.Parse("{\"includeSecondaryRoads\":true}").Roads==RoadImportLevel.None,"Legacy persisted secondary enabled independent groups without an explicit selection");
        Check(NetworkPreferences.Parse("{\"roadLevel\":\"none\",\"importRail\":false}")==new NetworkPreferences(RoadImportLevel.None,false),"Explicit none/off not persisted");
        Check(!NetworkPreferences.Parse("{\"importForests\":false}").Forests,"Independent forestry preference not preserved");
        Check(NetworkPreferences.Parse("{\"roadLevel\":123,\"importRail\":\"false\"}")==NetworkPreferences.Defaults,"Corrupt typed preference did not default");
        Check(!NetworkSelection.NativeDirectionMatches(true,false,false)&&NetworkSelection.NativeDirectionMatches(true,true,false)&&!NetworkSelection.NativeDirectionMatches(true,false,true),"One-way can resolve to two-way/backward prefab");
        Check(NetworkSelection.BasicRoadName(4)=="Medium Road"&&NetworkSelection.BasicRoadName(6)=="Large Road","Standard road lane mapping inconsistent");
        Check(NetworkSelection.IsBasicVanillaRoad("Small Road")&&NetworkSelection.IsBasicVanillaRoad("Highway Oneway - 3 lanes"),"Basic vanilla palette lost");
        Check(!NetworkSelection.IsBasicVanillaRoad("Small Road with Trees")&&!NetworkSelection.IsBasicVanillaRoad("Custom Boulevard")&&!NetworkSelection.IsBasicVanillaRoad("Small Road Oneway - 2 lanes Bus"),"Decorated/custom road allowed into basic palette");
        var pair=new[]{S(1,1,2,0,0,100,0),S(2,3,4,100,20,0,20)};
        var paired=NetworkHardener.Prepare(pair,log,default,false);
        Check(paired.Count==2&&paired.All(s=>s.CorridorId==1&&Math.Abs(s.MaximumNativeWidth-19)<.01),"Legitimate carriageway pair lost/not width-constrained");
        Check(NetworkHardener.Prepare(new[]{pair[0],pair[1] with{Name="Unrelated"}},log,default,false).All(s=>s.CorridorId==0),"Unrelated named parallel roads paired");
        var junctionPair=new[]{pair[0] with{CorridorId=1},pair[1] with{CorridorId=1,EndNodeId=1,End=pair[0].Start}};
        Check(NetworkHardener.Prepare(junctionPair,log,default,false).All(s=>float.IsPositiveInfinity(s.MaximumNativeWidth)),"Genuine shared junction treated as zero-width carriageway");
        var duplicated=new[]{pair[0],pair[1],pair[0] with{WayId=3,StartNodeId=10,EndNodeId=11},pair[1] with{WayId=4,StartNodeId=12,EndNodeId=13}};
        Check(NetworkHardener.Prepare(duplicated,log,default,false).Count==2,"Double boulevard not removed conservatively");
        var bridge=S(20,20,21,100,0,150,0) with{Bridge=true,Layer=1};
        var approach=S(21,22,23,0,0,90,0);var exit=S(22,24,25,160,0,250,0);var perpendicular=S(23,26,27,96,-20,96,20);
        var repaired=NetworkHardener.Prepare(new[]{bridge,approach,exit,perpendicular},log,default,false);
        Check(repaired.Count==6 && repaired.Any(s=>s.StartNodeId==23&&s.EndNodeId==20)&&repaired.Any(s=>s.StartNodeId==21&&s.EndNodeId==24),"Parallel bridge approaches not resolved by corridor evidence");
        Check(!repaired.Any(s=>s.WayId<0&&(s.StartNodeId==26||s.EndNodeId==27)),"Bridge tied to perpendicular road");
        var links=new[]{S(30,30,31,0,0,1,0) with{Classification="motorway_link"},S(30,31,32,1,0,2,0) with{Classification="motorway_link"}};
        Check(NetworkNormalizer.Normalize(links,log,default).Count==2,"Short topology-critical ramp consolidated");
        var steep=S(40,40,41,0,0,100,0) with{End=new float3(100,34,0)};
        Check(Math.Abs(NetworkVerticalProfiles.MaximumGrade(steep)-.34)<.001,"Steep valid road grade wrong");
        Check(NetworkVerticalProfiles.WithinGradeCeiling(steep),"Conservative grade guard rejected real steep road");
        Check(NetworkVerticalProfiles.MaximumGrade(steep with{End=new float3(100,35,0)})>.348,"Absolute ceiling not detected");
        Check(!NetworkVerticalProfiles.WithinGradeCeiling(steep with{End=new float3(100,35,0)}),"Continuous grade guard missed excessive grade");
        var loop=S(41,42,43,0,0,10,0) with{End=new float3(10,20,0),Control1=new float3(0,6,100),Control2=new float3(10,14,100)};
        Check(NetworkVerticalProfiles.ArcLength(loop)>150,"Curved path length replaced by endpoint chord");
        var raster=Enumerable.Repeat((ushort)32768,128*128).ToArray();
        var flat=NetworkVerticalProfiles.Prepare(new[]{S(50,50,51,-100,0,100,0)},raster,100,1000,log,default);
        Check(flat.Count==1&&NetworkVerticalProfiles.MaximumGrade(flat[0])<.0001,"Flat road gained microrelief");
        // Match the existing physical policies explicitly retained in ROAD-005:
        // motorway exceptional9%, protected-link13%, not the older8%/12% draft.
        Check(NetworkVerticalProfiles.GradeLimit(S(1,1,2,0,0,100,0) with{Classification="motorway"})==.09f,"Motorway specified exceptional limit missing");
        Check(NetworkVerticalProfiles.GradeLimit(S(1,1,2,0,0,100,0) with{Classification="motorway_link"})==.13f,"Ramp specified exceptional limit missing");
        var rolling=new ushort[128*128];
        for(var z=0;z<128;z++)for(var x=0;x<128;x++)rolling[z*128+x]=(ushort)Math.Round((100+((x/127f-.5f)*1000)*.12f)/200*65535);
        var gradingSource=new[]{S(70,70,71,-100,0,0,0) with{Classification="motorway"},S(70,71,72,0,0,100,0) with{Classification="motorway"},S(71,71,73,0,0,0,100)};
        var grading=NetworkVerticalProfiles.Prepare(gradingSource,rolling,200,1000,log,default);
        Check(grading.Count>=3&&grading.Where(s=>s.Classification=="motorway").All(s=>NetworkVerticalProfiles.WithinGradeCeiling(s,.09f)),"Repairable highway grades not reduced to specified exceptional limit");
        var junctionHeights=grading.SelectMany(s=>new[]{(id:s.StartNodeId,y:s.Start.y),(id:s.EndNodeId,y:s.End.y)}).Where(p=>p.id==71).ToArray();
        Check(junctionHeights.Length==3&&junctionHeights.All(p=>Math.Abs(p.y-junctionHeights[0].y)<.001f),"Grade smoothing disconnected a shared junction");
        Check(gradingSource.SelectMany(s=>new[]{s.Start,s.End}).All(p=>p.y==0),"Grading mutated source geometry");
        Check(log.Lines.Any(s=>s.Contains("PROCESS road-grading begin"))&&log.Lines.Any(s=>s.Contains("PROCESS road-grading complete")),"Post-download grading diagnostics missing");
        var explicitUrban=new OsmDataset{Nodes=nodes,Ways=new Dictionary<long,OsmWay>{{100,new(100,new long[]{1,2},new Dictionary<string,string>{{"highway","trunk"},{"motorroad","no"}})}}};
        var explicitRoute=NetworkProcessor.BuildSegments(explicitUrban,request with{RoadLevel=RoadImportLevel.Routes},log).Single();
        Check(explicitRoute.Classification=="trunk"&&explicitRoute.SourceTags["motorroad"]=="no","Selected source class was overwritten by an expressway inference instead of preserving the explicit road-group policy");
        var cliff=new ushort[128*128];for(var z=0;z<128;z++)for(var x=0;x<128;x++)cliff[z*128+x]=(ushort)(x<64?0:65535);
        var relaxed=NetworkVerticalProfiles.Prepare(new[]{S(51,52,53,-10,0,10,0)},cliff,1000,1000,log,default);
        Check(relaxed.Count==1&&relaxed[0].StartNodeId==52&&relaxed[0].EndNodeId==53&&!relaxed[0].Bridge,"Difficult road aborted or lost connectivity/changed to bridge");
        Check(log.Lines.Any(s=>s.Contains("relaxed quality limits"))&&log.Lines.Any(s=>s.Contains("practicalLimitsEnforced=False")),"Relaxed grades incorrectly reported as validated success");
        var profiled=NetworkVerticalProfiles.Prepare(NetworkProcessor.AlignToTerrain(repaired,raster,100,1000),raster,100,1000,log,default);
        Check(profiled.Count>=6 && profiled.All(s=>NetworkVerticalProfiles.MaximumGrade(s)<=.34801f),"Bridge tie-in profile unsafe or valid approaches removed");
        NetworkNormalizer.Validate(profiled,1000,default);
        var tunnel=S(60,60,61,-200,0,200,0) with{Tunnel=true,Layer=-1};
        var mountain=new ushort[128*128];for(var z=0;z<128;z++)for(var x=0;x<128;x++)mountain[z*128+x]=(ushort)(10000+30000*Math.Max(0,1-Math.Abs(x-63.5)/25));
        var tunneled=NetworkVerticalProfiles.Prepare(new[]{tunnel},mountain,200,1000,log,default);
        Check(tunneled.Count>0&&NetworkNormalizer.Evaluate(tunneled[0],.5f).y<60,"Tunnel climbed toward mountain summit");
        var tracks=new List<PreparedSegment>();for(var i=0;i<7;i++)tracks.Add(S(100+i,1000+i*2,1001+i*2,0,i*2,100,i*2,NetworkKind.Rail,false) with{Classification="rail",RailService="yard"});
        var simplified=NetworkHardener.Prepare(NetworkNormalizer.Normalize(RailCorridorConsolidator.Prepare(tracks,log,default),log,default),log,default,true);
        Check(simplified.Count==1&&simplified.All(s=>s.Lanes==2&&s.PhysicalTracks==2&&!s.OneWay)&&
            simplified.SelectMany(s=>s.SourceWayIds).Distinct().OrderBy(id=>id).SequenceEqual(tracks.Select(s=>s.WayId)),
            "Starter-corridor yard simplification must use one double-track segment and retain every folded source way");
        Check(NetworkHardener.Prepare(tracks,log,default,true).Count==tracks.Count,"Hardening unexpectedly repeated rail consolidation or dropped an independent supplied track");
        var distantRail=S(999,9990,9991,0,100,100,100,NetworkKind.Rail,false) with{Classification="rail"};
        var pairedRails=RailCorridorConsolidator.Prepare(tracks.Select(s=>s with{RailService="",Lanes=1}).Append(distantRail).ToArray(),log,default);
        Check(pairedRails.Count==2&&pairedRails.All(s=>s.Lanes==2&&s.PhysicalTracks==2&&!s.OneWay)&&
            pairedRails.Any(s=>s.WayId==999||s.SourceWayIds.Contains(999)),"Mainline pairing lost the one-double-track-axis policy or consolidated a track outside the evidence radius");
        Check(NetworkHardener.Prepare(tracks,log,default,false).Count==0,"Disabled rail processed/retained");
        ImportSession.ChangeMap(true);var map=ImportSession.Map;var generation=ImportSession.Begin();Check(ImportSession.Matches(map,generation),"New session not accepted");
        ImportSession.InvalidateOperation();Check(!ImportSession.Matches(map,generation),"Changed configuration retained stale preparation");
        generation=ImportSession.Begin();ImportSession.ChangeMap(false);Check(!ImportSession.Matches(map,generation),"Map A work valid while loading B");ImportSession.ChangeMap(true);Check(!ImportSession.Matches(map,generation),"Map A callback valid in map B");
        var current=ImportSession.Map;generation=ImportSession.Begin();ImportSession.ChangeMap(true);Check(!ImportSession.Matches(current,generation),"Returning to same map reused old session");
        using(var cancellation=new CancellationTokenSource()){cancellation.Cancel();try{NetworkHardener.Prepare(pair,log,cancellation.Token,false);throw new Exception("Cancellation ignored");}catch(OperationCanceledException){}}
        AvenueTests.Run();
        CorridorTests.Run();
        Console.WriteLine("PASS: all 8 independent road/rail combinations, query omissions, preference defaults/migration, strict native direction, divided/duplicate corridors, bridge approaches/perpendicular rejection, short links, path-length grades/ceiling, flat/bridge/tunnel profiles, single-axis yard/mainline consolidation, cancellation and stale configuration/map sessions.");
    }
}
namespace ImportEarthLocation { public static class Mod { public const string Id="ImportEarthLocation"; } }
namespace ImportEarthLocation.Core
{
    public sealed class ImportLog
    {
        public Dictionary<string,object> Artifacts { get; }=new();
        public void DiagnosticJson(string name,object value)=>Artifacts[name]=value;
        public void DiagnosticSvg(string name,string svg)=>Artifacts[name]=svg;
        public readonly List<string> Lines=new();
        public readonly List<NetworkQualityFinding> Findings=new();
        private readonly Dictionary<string,double> _metrics=new();
        public bool CaptureRecords { get; init; }
        public bool ScalarDiagnostics { get; init; }
        public readonly Dictionary<string,object[]> Records=new();
        public bool DiagnosticsEnabled=>CaptureRecords||ScalarDiagnostics;
        public void Info(string s)=>Lines.Add(s);
        public void Warn(string s)=>Lines.Add(s);
        public void Diagnostic(string s)=>Lines.Add(s);
        public void Quality(NetworkQualityFinding finding)=>Findings.Add(finding);
        public void Count(string s,int count=1){_metrics.TryGetValue(s,out var old);_metrics[s]=old+count;}
        public void AddCounter(string s,long count){_metrics.TryGetValue(s,out var old);_metrics[s]=old+count;}
        public void AddDuration(string s,double seconds){_metrics.TryGetValue(s+" time (s)",out var old);_metrics[s+" time (s)"]=old+seconds;}
        public void DiagnosticRecords(string s,IEnumerable<object> records,long maximumBytes=8L*1024*1024,bool critical=false){}
        public void DiagnosticCompressedRecords(string s,IEnumerable<object> records){}
        public void DiagnosticRecordChunks(string s,IEnumerable<object> records){if(CaptureRecords||ScalarDiagnostics&&s=="paired-construction")Records[s]=records.ToArray();}
        public void CaptureProfileInput(IReadOnlyList<PreparedSegment> s,ushort[] heights,float scale,double width,IReadOnlyDictionary<string,NativeAssetContract>? assets){}
        public void Time(string s,double seconds){_metrics[s+" time (s)"]=seconds;}
        public double Metric(string s)=>_metrics.TryGetValue(s,out var value)?value:0;
    }
}
namespace ImportEarthLocation.Processing { public static class TerrainBuilder { public const int Resolution=128; public sealed record Result(ushort[] Heightmap,float HeightScaleMeters,float SeaLevelMeters,float MinimumElevation,float MaximumElevation); } }
namespace ImportEarthLocation.Data
{
    public static class PublicHttpClient { public static HttpClient Instance=new(); }
    public sealed class CacheStore { public Task<byte[]?> ReadValidAsync(string group,string key,string extension,Func<byte[],bool> valid,CancellationToken token)=>Task.FromResult<byte[]?>(null);public Task WriteAsync(string group,string key,string extension,byte[] bytes,CancellationToken token)=>Task.CompletedTask; }
}
