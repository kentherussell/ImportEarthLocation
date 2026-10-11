using ImportEarthLocation.Core;
using ImportEarthLocation.Geo;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

static class WaterTests
{
    static void Check(bool b,string s) { if(!b)throw new Exception(s); }
    public static void Run()
    {
        var request=new ImportRequest(0,0,new("test","test",1000,false),false);
        var heights=Enumerable.Repeat((ushort)(30f/100*65535),64*64).ToArray();
        var original=heights.ToArray();
        var terrain=new TerrainBuilder.Result(heights,100,0,0,100);
        OsmNode Node(long id,float x,float z)=>new(id,z/6378137d*180/Math.PI,x/6378137d*180/Math.PI);
        var nodes=new Dictionary<long,OsmNode>{{1,Node(1,0,-600)},{2,Node(2,0,0)},{3,Node(3,0,600)}};
        var way=new OsmWay(1,new long[]{1,2,3},new Dictionary<string,string>{{"waterway","river"}});
        var dataset=new OsmDataset{Nodes=nodes,Ways=new Dictionary<long,OsmWay>{{1,way}}};
        float3[] Ring(float l,float b,float r,float t)=>new[]{new float3(l,0,b),new float3(r,0,b),new float3(r,0,t),new float3(l,0,t)};
        var river=new GeographicPolygon(99,new Dictionary<string,string>{{"natural","water"},{"water","river"}},new(){Ring(-160,-480,160,480)},new());
        WaterPlan Plan(GeographicPolygon[] areas,TerrainBuilder.Result? input=null,float cell=28,OsmDataset? data=null)=>
            WaterBodyPlanner.Prepare(data??dataset,areas,request,input??terrain,new ImportLog(),default,cell);
        var plan=Plan(new[]{river});
        // 2026-10-09: indexed predicates must retain exact legacy geometry,
        // including negative bands, implicit closure, holes and large fallback rings.
        IndexedMaskChecks();
        var pulseLog=new ImportLog();var details=new List<string>();
        WaterBodyPlanner.Prepare(dataset,new[]{river},request,terrain,pulseLog,default,28,details.Add);
        foreach(var phase in new[]{"indexing mapped water","mapped lakes","connecting river reaches","solving river levels pass","conditioning river beds and placing controls","publishing conditioned water terrain"})
            Check(details.Any(s=>s.Contains(phase,StringComparison.Ordinal)),"Water phase missing from UI progress: "+phase);
        Check(pulseLog.Lines.Any(s=>s.Contains("WATER PLAN PROGRESS")&&s.Contains("bedCellsVisited=")&&s.Contains("reachSamples=")),"Water diagnostics omit work counters");
        Check(plan.Sources.Count>1&&plan.Sources.Select(s=>s.BodyId).Distinct().Count()==1,"Connected split river lost its durable body identity or distributed controls");
        var splitWays=new[]{way with{Id=50,NodeIds=new long[]{1,2}},way with{Id=4,NodeIds=new long[]{2,3}}};
        WaterPlan Split(bool reverse)=>Plan(new[]{river},data:new OsmDataset{Nodes=nodes,Ways=(reverse?splitWays.Reverse():splitWays).ToDictionary(w=>w.Id)});
        var splitForward=Split(false);var splitReverse=Split(true);
        Check(splitForward.Sources.Count>0&&splitReverse.Sources.Count>0&&splitForward.Sources.Concat(splitReverse.Sources).All(s=>s.BodyId==4),"Ranked reach connectivity lost minimum source-way body identity");
        Check(splitForward.ConditionedHeightmap!.SequenceEqual(splitReverse.ConditionedHeightmap!),"Water bed conditioning changed with dictionary insertion order");
        Check(plan.Sources.All(s=>s.Radius>=28&&!s.BoundaryDriven&&river.Contains(s.Position)&&s.TargetSurfaceLevel.HasValue&&Math.Abs(s.Position.y+s.Depth-s.TargetSurfaceLevel.Value)<.001&&s.TargetSurfaceLevel<=s.BankLevel-.35),"Control violates bank-limited absolute stage or mapped footprint");
        Check(plan.ConditionedHeightmap!=null&&heights.SequenceEqual(original),"Water conditioning must not mutate the input DEM");
        int Pixel(float x,float z)=>(int)Math.Round((z/1000+.5)*63)*64+(int)Math.Round((x/1000+.5)*63);
        Check(plan.ConditionedHeightmap![Pixel(80,0)]<original[Pixel(80,0)],"Wide mapped river was carved as a thin centerline ribbon");
        Check(plan.ConditionedHeightmap[Pixel(300,0)]==original[Pixel(300,0)],"Mapped river carving touched dry land");
        for(var i=0;i<heights.Length;i++)Check(plan.ConditionedHeightmap[i]<=heights[i]&&(heights[i]-plan.ConditionedHeightmap[i])/65535f*100<=12.001,"Water cut raised terrain or exceeded its budget");
        var island=river with{Inner=new(){Ring(-40,-40,40,40)}};
        var islandPlan=Plan(new[]{island});
        Check(islandPlan.ConditionedHeightmap![Pixel(0,0)]==original[Pixel(0,0)]&&islandPlan.Sources.All(s=>island.Contains(s.Position)),"Mapped island was carved or received a source");
        var sample=WaterBodyPlanner.SurfaceSampler(plan,new[]{river});
        Check(sample(new float3(80,0,0)).HasValue&&!sample(new float3(300,0,0)).HasValue,"Bridge water sampling ignores the actual mapped wet mask");
        Check(Plan(new[]{river},cell:float.NaN).Sources.Count==0,"Unknown native water-grid size should not create controls");
        var intermittent=new OsmDataset{Nodes=nodes,Ways=new Dictionary<long,OsmWay>{{1,way with{Tags=new Dictionary<string,string>{{"waterway","river"},{"intermittent","yes"}}}}}};
        Check(Plan(new[]{river},data:intermittent).Sources.Count==0,"Intermittent river received permanent sources");
        Check(Plan(new[]{river},terrain with{SeaLevelMeters=50}).Sources.Count==0,"A high receiving level bypassed bank containment");
        var narrow=new OsmDataset{Nodes=nodes,Ways=new Dictionary<long,OsmWay>{{1,way with{Tags=new Dictionary<string,string>{{"waterway","canal"},{"width","4"}}}}}};
        Check(Plan(Array.Empty<GeographicPolygon>(),data:narrow).Sources.Count==0,"Sub-grid canal received a source spilling beyond its known width");
        var earthworks=plan.ConditionedHeightmap.ToArray();var source=plan.Sources.First();
        earthworks[Pixel(source.Position.x,source.Position.z)]=(ushort)((source.TargetSurfaceLevel!.Value+.5f)/100*65535);
        var revisionLog=new ImportLog();var revised=WaterBodyPlanner.Revalidate(plan,terrain with{Heightmap=earthworks},1000,revisionLog,default);
        Check(revised.Sources.Count==plan.Sources.Count&&revised.ConditionedHeightmap!=null,"Small final-earthworks obstruction was not recovered: "+string.Join(" | ",revisionLog.Lines));
        var protectedRoad=new PreparedSegment(5,10,11,source.Position-new float3(100,0,0),source.Position+new float3(100,0,0),NetworkKind.Road,"primary",2,false,false,false,0){NativeWidth=16};
        var noControls=plan with{Sources=Array.Empty<RiverSourcePlan>()};var emptyLog=new ImportLog();
        var beforeFinal=earthworks.ToArray();
        var noControlRevision=WaterBodyPlanner.Revalidate(noControls,terrain with{Heightmap=earthworks},1000,emptyLog,default,new[]{protectedRoad});
        Check(noControlRevision.ConditionedHeightmap==null&&earthworks.SequenceEqual(beforeFinal),"No-control revalidation restored stale preparation terrain over newer earthworks");
        Check(emptyLog.Lines.Any(s=>s.Contains("networkProtectionRasterizationSkipped=true")),"Empty-control fast path is not diagnosed");
        var protectedPlan=WaterBodyPlanner.Revalidate(plan,terrain with{Heightmap=earthworks},1000,new ImportLog(),default,new[]{protectedRoad});
        Check(protectedPlan.Sources.Count<plan.Sources.Count,"A blocked source was retained by cutting through a protected roadbed");
        Check(protectedPlan.ConditionedHeightmap==null||protectedPlan.ConditionedHeightmap[Pixel(source.Position.x,source.Position.z)]==earthworks[Pixel(source.Position.x,source.Position.z)],"Recovery modified infrastructure-owned terrain");
        var lake=river with{Id=100,Tags=new Dictionary<string,string>{{"natural","water"},{"water","lake"}},Outer=new(){Ring(-300,-300,300,300)},Inner=new(){Ring(-40,-40,40,40)}};
        var lakePlan=Plan(new[]{lake},data:new OsmDataset());
        Check(lakePlan.Sources.Count>0&&lakePlan.Sources.All(s=>s.BodyKind=="lake"&&lake.Contains(s.Position))&&lakePlan.Sources.Select(s=>s.TargetSurfaceLevel).Distinct().Count()==1,"Lake lost a single body stage or island protection");
        using var cancellation=new CancellationTokenSource();cancellation.Cancel();
        try{WaterBodyPlanner.Prepare(dataset,new[]{river},request,terrain,new ImportLog(),cancellation.Token,28);throw new Exception("Cancellation ignored");}catch(OperationCanceledException){}
        try{WaterBodyPlanner.Revalidate(noControls,terrain,1000,new ImportLog(),cancellation.Token);throw new Exception("Empty-control fast path swallowed cancellation");}catch(OperationCanceledException){}
        using var duringPlanning=new CancellationTokenSource();var reachedRiverPhase=false;
        try
        {
            WaterBodyPlanner.Prepare(dataset,new[]{river},request,terrain,new ImportLog(),duringPlanning.Token,28,detail=>
            {if(detail.Contains("connecting river reaches")){reachedRiverPhase=true;duringPlanning.Cancel();}});
            throw new Exception("Cancellation during river planning ignored");
        }
        catch(OperationCanceledException){Check(reachedRiverPhase,"Cancellation case never reached the formerly silent river phase");}
        Check(WaterPlanner.Clip(new float3(-600,0,0),new float3(600,0,0),500,out var entry,out var exit)&&Math.Abs(entry-1f/12)<.0001&&Math.Abs(exit-11f/12)<.0001,"Water boundary clipping changed");
        Console.WriteLine("PASS: exact indexed/local/global water masks and precedence, deterministic split-body identity, phase/work diagnostics, bank-limited stages, immutable bounded beds/islands, final-earthworks recovery, no-control terrain preservation, infrastructure protection, intermittent/sub-grid rejection and mid-planning cancellation; native wetness or timed throughput not claimed.");
    }

    static void IndexedMaskChecks()
    {
        // Compare the real indexed predicate against the existing exact public
        // predicate, not a hand-written replacement. Reflection avoids changing
        // production visibility solely for tests. No wall-clock timing assertions.
        var areaType=typeof(WaterBodyPlanner).GetNestedType("Area",System.Reflection.BindingFlags.NonPublic)!;
        var contains=areaType.GetMethod("Contains")!;
        float3[] Rect(float lo,float hi)=>new[]{new float3(lo,0,lo),new float3(hi,0,lo),new float3(hi,0,hi),new float3(lo,0,hi)};
        var tags=new Dictionary<string,string>{{"natural","water"},{"water","lake"}};
        var local=new GeographicPolygon(501,tags,new(){Rect(-400,400)},new(){Rect(-128,128)});
        var huge=local with{Id=502,Outer=new(){Rect(-100000,100000)},Inner=new(){Rect(-128,128)}};
        var concave=local with{Id=503,Outer=new(){new[]{new float3(-500,0,-256),new float3(400,0,-256),new float3(400,0,384),new float3(0,0,0),new float3(-500,0,384)},Rect(700,900)}};
        var queries=new List<float3>();
        foreach(var z in new[]{-100000f,-512,-256,-128,-.01f,0,128,256,384,512,100000})
            foreach(var x in new[]{-100000f,-500,-400,-128,-.01f,0,128,400,700,900,100000})queries.Add(new float3(x,0,z));
        var random=new System.Random(20261009);
        for(var i=0;i<300;i++)queries.Add(new float3((float)(random.NextDouble()*2400-1200),0,(float)(random.NextDouble()*2400-1200)));
        foreach(var polygon in new[]{local,huge,concave,local with{Outer=new(){Rect(-400,400).Reverse().ToArray()}}})
        {
            var indexed=Activator.CreateInstance(areaType,new object?[]{polygon,0,CancellationToken.None,null})!;
            foreach(var p in queries)Check((bool)contains.Invoke(indexed,new object[]{p})! == polygon.Contains(p),$"Indexed water predicate differs for body {polygon.Id} at {p}");
        }
        // Large/global and local area buckets must merge in original input order.
        // Co-located controls make a precedence error observable as a wrong level.
        var broad=huge with{Inner=new()};var small=local with{Inner=new()};
        RiverSourcePlan Source(long body,float stage)=>new(body,"test","fixture",float3.zero,28,1,100,float3.zero){BodyKind="lake",BodyId=body,TargetSurfaceLevel=stage};
        var water=new WaterPlan(false,0,2,0,"fixture"){Sources=new[]{Source(broad.Id,42),Source(small.Id,19)}};
        Check(WaterBodyPlanner.SurfaceSampler(water,new[]{broad,small})(float3.zero)==42,"Global area lost precedence over later local area");
        Check(WaterBodyPlanner.SurfaceSampler(water,new[]{small,broad})(float3.zero)==19,"Local area lost precedence over later global area");
    }
}
