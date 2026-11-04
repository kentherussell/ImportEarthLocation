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
        if(args.Length==1&&args[0]=="--avenues-only"){AvenueTests.Run();return;}
        if(args.Length==1&&args[0]=="--rail-crossings-only"){RailCrossingTests.Run();return;}
        if(args.Length==1&&args[0]=="--street-smoothing-only"){StreetSmoothingTests.Run();return;}
        if(args.Length==3&&args[0]=="--rail-crossings-capture"){VisualMetricsReplay.RailCrossings(args[1],args[2]);return;}
        if(args.Length==3&&args[0]=="--street-smoothing-capture"){VisualMetricsReplay.StreetSmoothing(args[1],args[2]);return;}
        if(args.Length==4&&args[0]=="--street-smoothing-capture"){VisualMetricsReplay.StreetSmoothing(args[1],args[2],args[3]);return;}
        if(args.Length==1&&args[0]=="--connectivity-only"){ConnectivityTests.Run();return;}
        if(args.Length==3&&args[0] is "--street-capture" or "--street-design-capture"){VisualMetricsReplay.Streets(args[1],args[2],args[0]=="--street-design-capture");return;}
        if(args.Length==4&&args[0]=="--road-water-capture"){VisualMetricsReplay.RoadWater(args[1],args[2],args[3]);return;}
        if(args.Length==1&&args[0]=="--water-roads-only"){WaterRoadTests.Run();return;}
        if(args.Length==5&&args[0]=="--water-landscape-capture"){VisualMetricsReplay.WaterLandscape(args[1],args[2],args[3],args[4]);return;}
        if(args.Length==1&&args[0]=="--rebalance-anchors-only"){RebalanceRegressionTests.Run();return;}
        if(args.Length==1&&args[0]=="--writer-trace-only"){NetworkWriterTrace.Test();PairedPublicationTraceTests.Run();return;}
        if(args.Length==4&&args[0] is "--corrective-writer-trace" or "--corrective-writer-trace-horizontal")
        {
            var traceWays=args[3].Split(',').Select(value=>long.Parse(value,System.Globalization.CultureInfo.InvariantCulture));
            CorrectiveCapturedReplay.Design(args[1],args[2],horizontalOnly:args[0].EndsWith("-horizontal",StringComparison.Ordinal),
                writerTrace:new NetworkWriterTrace(traceWays));return;
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
        if(args.Length==3&&args[0]=="--visual-metrics"){VisualMetricsReplay.Run(args[1],args[2]);return;}
        if(args.Length==2&&args[0]=="--visual-metrics-recluster"){VisualMetricsReplay.Recluster(args[1]);return;}
        if(args.Length==2&&args[0]=="--live-breaks"){VisualMetricsReplay.LiveBreaks(args[1]);return;}
        if(args.Length==6&&args[0]=="--resample-demo"){var f=System.Globalization.CultureInfo.InvariantCulture;VisualMetricsReplay.ResampleDemo(args[1],args[2],double.Parse(args[3],f),double.Parse(args[4],f),args[5]);return;}
        if(args.Length==3&&args[0]=="--visual-compiler"){VisualMetricsReplay.Compiler(args[1],args[2]);return;}
        if(args.Length==3&&args[0]=="--rock-plan"){VisualMetricsReplay.RockPlan(args[1],args[2]);return;}
        if(args.Length==3&&args[0]=="--lane-runs"){VisualMetricsReplay.LaneRuns(args[1],args[2]);return;}
        if(args.Length==2&&args[0]=="--interval-timing"){VisualMetricsReplay.IntervalTiming(args[1]);return;}
        if(args.Length>=3&&args[0]=="--crossings"){VisualMetricsReplay.Crossings(args[1],args.Skip(2).Select(long.Parse).ToHashSet());return;}
        if(args.Length==7&&args[0]=="--plan-png"){var f=System.Globalization.CultureInfo.InvariantCulture;VisualMetricsReplay.PlanPng(args[1],args[2],double.Parse(args[3],f),double.Parse(args[4],f),double.Parse(args[5],f),args[6]);return;}
        if(args.Length==6&&args[0]=="--terrain-crop"){var f=System.Globalization.CultureInfo.InvariantCulture;VisualMetricsReplay.TerrainCrop(args[1],float.Parse(args[2],f),float.Parse(args[3],f),float.Parse(args[4],f),args[5]);return;}
        if(args.Length==7&&args[0]=="--terrain-teeth"){var f=System.Globalization.CultureInfo.InvariantCulture;VisualMetricsReplay.Teeth(args[1],args[2],args[3],float.Parse(args[4],f),float.Parse(args[5],f),args[6]);return;}
        if(args.Length==4&&args[0]=="--water-replay"){VisualMetricsReplay.WaterReplay(args[1],args[2],float.Parse(args[3],System.Globalization.CultureInfo.InvariantCulture));return;}
        if(args.Length==5&&args[0]=="--water-replay"){VisualMetricsReplay.WaterReplay(args[1],args[2],float.Parse(args[3],System.Globalization.CultureInfo.InvariantCulture),args[4]);return;}
        if(args.Length==4&&args[0]=="--channel-crossings-capture"){VisualMetricsReplay.ChannelCrossings(args[1],args[2],args[3]);return;}
        if(args.Length==2&&args[0]=="--water-banks"){VisualMetricsReplay.WaterBanks(args[1]);return;}
        if(args.Length==4&&args[0]=="--beach-check"){VisualMetricsReplay.BeachCheck(args[1],args[2],float.Parse(args[3],System.Globalization.CultureInfo.InvariantCulture));return;}
        if(args.Length==5&&args[0]=="--visual-plot"){VisualMetricsReplay.Plot(args[1],args[2],args[3],args[4]);return;}
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
        RailCrossingTests.Run();
        StreetSmoothingTests.Run();
        WaterRoadTests.Run();
        RebalanceRegressionTests.Run();
        ConnectivityTests.Run();
        TerrainSamplingTests.Run();
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
            // Main Roads takes trunk links with it, with or without Streets.
            var expected=((level&RoadImportLevel.Highways)!=0?2:0)+((level&RoadImportLevel.Routes)!=0?2:0)+((level&RoadImportLevel.Streets)!=0?3:0);
            Check(result.Count(s=>s.Kind==NetworkKind.Road)==expected&&result.Count(s=>s.Kind==NetworkKind.Rail)==(rail?1:0),"Road/rail independence failed");
            var query=OverpassClient.BuildNetworkQuery(new(-1,-1,1,1),level,rail);
            Check(query.Contains("way[\"highway\"~\"^("+NetworkSelection.Classes(level))== (level!=RoadImportLevel.None)&&query.Contains("way[\"railway\"=\"rail\"]")==rail,"Disabled selected network type fetched");
            Check(!query.Contains("forest")&&!query.Contains("wood"),"Optional forest acquisition leaked into required networks");
            var forestQuery=OverpassClient.BuildForestQuery(new(-1,-1,1,1));
            Check(forestQuery.Contains("forest")&&forestQuery.Contains("wood")&&!forestQuery.Contains("highway")&&!forestQuery.Contains("railway"),"Forest query includes networks or loses cover");
        }
        Check(new ImportRequest(0,0,MapSizeProfile.Vanilla,true).RoadLevel==RoadImportLevel.None,"Legacy constructor enabled independent groups without an explicit selection");
        // Lane runs: a 100 m 4-lane pocket between 3-lane stretches takes 3 lanes;
        // a 500 m 4-lane stretch is a real change and stays.
        PreparedSegment Way(long id,float x0,float x1,int lanes)=>new(id,id,id+1,new float3(x0,0,0),new float3(x1,0,0),NetworkKind.Road,"trunk",lanes,true,false,false,0){Name="Avenue"};
        var pocket=NetworkLaneRuns.Smooth(new[]{Way(1,0,400,3),Way(2,400,500,4),Way(3,500,900,3),Way(4,900,1400,4),Way(5,1400,1800,3)},new ImportLog());
        Check(pocket.Select(s=>s.Lanes).SequenceEqual(new[]{3,3,3,4,3}),"Short lane pocket not smoothed, or a long lane change removed: "+string.Join(",",pocket.Select(s=>s.Lanes)));
        // Lane slots: two 2-lane roads merging into 3 lanes land on the left two
        // and right two lanes (±2 m), not both on the centre.
        PreparedSegment Lane(long id,long a,long b,float3 p0,float3 p1,float[] offsets)=>new(id,a,b,p0,p1,NetworkKind.Road,"motorway",offsets.Length,true,false,false,0){NativeLaneOffsets=offsets};
        var merged=NetworkLaneSlots.Align(new[]{Lane(11,100,200,new float3(-6,0,-200),new float3(0,0,0),new[]{-2f,2f}),Lane(12,101,200,new float3(6,0,-200),new float3(0,0,0),new[]{-2f,2f}),
            Lane(13,200,300,new float3(0,0,0),new float3(0,0,200),new[]{-4f,0f,4f})},new ImportLog());
        var plan=NativeLaneObligations.Build(merged);
        Check(plan.Movements.Count>0&&plan.Movements.All(m=>m.PlannedLateralError<.05),"Merge lanes not aligned to the receiving lanes: "+string.Join(",",plan.Movements.Select(m=>m.PlannedLateralError.ToString("F2"))));
        // A one-way road widening from 2 to 3 lanes at a plain node keeps its left
        // edge: the wider side starts 2 m to the right; the 40 m section moves
        // whole and the next, long enough, bends back to its mapped line.
        var widened=NetworkLaneSlots.Align(new[]{Lane(21,400,401,new float3(0,0,-200),new float3(0,0,0),new[]{-2f,2f}) with{NativeWidth=14},
            Lane(22,401,402,new float3(0,0,0),new float3(0,0,40),new[]{-4f,0f,4f}) with{NativeWidth=18},Lane(23,402,403,new float3(0,0,40),new float3(0,0,300),new[]{-4f,0f,4f}) with{NativeWidth=18}},new ImportLog());
        Check(Math.Abs(widened[1].StartCurveOffset.x-2)<1e-3&&Math.Abs(widened[1].EndCurveOffset.x-2)<1e-3&&Math.Abs(widened[2].StartCurveOffset.x-widened[1].EndCurveOffset.x)<1e-4&&
            math.lengthsq(widened[2].EndCurveOffset)<1e-6&&math.lengthsq(widened[0].EndCurveOffset)<1e-6,
            $"Lane gain not left-aligned: {widened[1].StartCurveOffset} {widened[1].EndCurveOffset} {widened[2].StartCurveOffset} {widened[2].EndCurveOffset}");
        Check(NetworkSelection.Parse("highways")== (RoadImportLevel.Highways|RoadImportLevel.Routes)&&
            NetworkSelection.Parse("routes")== (RoadImportLevel.Highways|RoadImportLevel.Routes)&&
            NetworkSelection.Parse("streets")==RoadImportLevel.Streets,"Combined highway group or legacy selection migration is incorrect");
        Check(NetworkSelection.Label(RoadImportLevel.Primary)=="Highways, Main Roads","Road group labels do not match the two-option UI");
        Check(NetworkPreferences.Parse("{}")==NetworkPreferences.Defaults&&NetworkPreferences.Parse("broken")==NetworkPreferences.Defaults,"Corrupt/missing preferences unsafe");
        Check(NetworkPreferences.Parse("{\"includeSecondaryRoads\":true}").Roads==RoadImportLevel.None,"Legacy persisted secondary enabled independent groups without an explicit selection");
        Check(NetworkPreferences.Parse("{\"roadLevel\":\"none\",\"importRail\":false}")==new NetworkPreferences(RoadImportLevel.None,false),"Explicit none/off not persisted");
        Check(!NetworkPreferences.Parse("{\"importForests\":false}").Forests,"Independent forestry preference not preserved");
        // Density sliders: older files migrate from their on/off value; 0 is off.
        var legacy=NetworkPreferences.Parse("{\"importForests\":true,\"importRocks\":false}");
        Check(legacy.VegetationDensity==1&&legacy.RockDensity==0,"Legacy on/off preferences did not migrate to full/zero density");
        var densities=NetworkPreferences.Parse("{\"importForests\":true,\"importRocks\":true,\"vegetationDensity\":0.44,\"rockDensity\":0}");
        Check(densities.VegetationDensity==.4f&&densities.Forests&&densities.RockDensity==0&&!densities.Rocks,"Density preferences not stepped to tenths or zero not treated as off");
        var routes=NetworkPreferences.Parse("{\"applyGroundSurfaces\":false,\"createAirConnections\":true,\"createSeaConnections\":true}");
        Check(!routes.Surfaces&&routes.AirRoutes&&routes.ShipRoutes&&NetworkPreferences.Defaults.Surfaces&&!NetworkPreferences.Defaults.AirRoutes&&!NetworkPreferences.Defaults.ShipRoutes,
            "Surfaces, air and ship route choices not remembered, or their defaults changed");
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
        public void Evidence(string s,bool warning=false)=>Lines.Add(s);
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
