using ImportEarthLocation.Core;
using ImportEarthLocation.Geo;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

static class WaterTests
{
    static void Check(bool b,string s) { if(!b)throw new Exception(s); }
    public static void Run()
    {
        WaterCutTests.Run();
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
        foreach(var phase in new[]{"indexing mapped water","mapped lakes","connecting river reaches","sampling contained river stage bounds","solving directed water stages","conditioning river beds and placing controls","publishing conditioned water terrain"})
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
        // Both short culverts and long buried rivers become connected open
        // channels. Their crossings must use the full constructed footprint.
        OsmDataset Culverted(float halfLength)
        {
            var culvertNodes=new Dictionary<long,OsmNode>(nodes){{7,Node(7,0,-halfLength)},{8,Node(8,0,halfLength)}};
            var tags=new Dictionary<string,string>{{"waterway","river"},{"tunnel","culvert"}};
            return new OsmDataset{Nodes=culvertNodes,Ways=new Dictionary<long,OsmWay>{{61,way with{Id=61,NodeIds=new long[]{1,7}}},{62,way with{Id=62,NodeIds=new long[]{7,8},Tags=tags}},{63,way with{Id=63,NodeIds=new long[]{8,3}}}}};
        }
        Check(Plan(new[]{river},data:Culverted(20)).ReachStages.Any(r=>r.WayId==62)&&WaterBodyPlanner.BuriedChannels(Culverted(20),request).Count==1,"A short culvert broke the river");
        // A 200 m culvert under an interchange still carries the river.
        Check(Plan(new[]{river},data:Culverted(100)).ReachStages.Any(r=>r.WayId==62)&&WaterBodyPlanner.BuriedChannels(Culverted(100),request).Count==1,"An interchange culvert broke the river");
        // A river without a polygon is carved a water cell (28 m) either side of
        // its line, so the simulation's cells can carry it; a ditch stays narrow.
        OsmDataset Stream(string type)=>new OsmDataset{Nodes=nodes,Ways=new Dictionary<long,OsmWay>{{70,way with{Id=70,Tags=new Dictionary<string,string>{{"waterway",type}}}}}};
        var stream=Plan(Array.Empty<GeographicPolygon>(),data:Stream("river")).ConditionedHeightmap!;
        Check(stream[Pixel(16,0)]<original[Pixel(16,0)]&&stream[Pixel(32,0)]<original[Pixel(32,0)],"A river was carved narrower than a water cell");
        Check(stream[Pixel(0,0)]<=stream[Pixel(32,0)],"A river channel is not deepest along its line");
        var terrace=heights.ToArray();
        for(var z=0;z<64;z++)for(var x=0;x<64;x++)
        {
            var px=x*1000f/63-500;
            if(px>=48&&px<=72)terrace[z*64+x]=(ushort)(20f/100*65535);
        }
        var terracePlan=Plan(Array.Empty<GeographicPolygon>(),terrain with{Heightmap=terrace},data:Stream("river"));
        Check(terracePlan.Sources.Count>0&&terracePlan.ReachStages.All(r=>Math.Max(r.StartLevel,r.EndLevel)<22),"A high distant terrace concealed a low bank beside the native channel");
        var junctionNodes=new Dictionary<long,OsmNode>{{101,Node(101,-420,0)},{102,Node(102,0,0)},{103,Node(103,420,0)},{104,Node(104,0,420)}};
        var junctionWays=new[]{new OsmWay(101,new long[]{101,102,103},new Dictionary<string,string>{{"waterway","river"}}),
            new OsmWay(102,new long[]{102,104},new Dictionary<string,string>{{"waterway","river"}})};
        var junctionHeights=heights.ToArray();
        for(var z=0;z<64;z++)for(var x=0;x<64;x++)
        {
            var px=x*1000f/63-500;var pz=z*1000f/63-500;
            if(px>=-320&&px<=-160&&Math.Abs(pz)>=48&&Math.Abs(pz)<=104)junctionHeights[z*64+x]=(ushort)(20f/100*65535);
        }
        var junction=Plan(Array.Empty<GeographicPolygon>(),terrain with{Heightmap=junctionHeights},data:new OsmDataset{Nodes=junctionNodes,Ways=junctionWays.ToDictionary(w=>w.Id)});
        var arriving=junction.ReachStages.Where(r=>r.WayId==101&&r.Start.x<=0&&r.End.x>=0)
            .Min(r=>math.lerp(r.StartLevel,r.EndLevel,-r.Start.x/Math.Max(.01f,r.End.x-r.Start.x)));
        Check(junction.ReachStages.Where(r=>r.WayId==102).All(r=>r.StartLevel<=arriving+.01f&&r.EndLevel<=arriving+.01f),
            "A branch beginning at an interior way node reset to a higher water level");
        foreach(var group in junction.ReachStages.GroupBy(r=>r.WayId))
        {
            var previous=float.PositiveInfinity;
            foreach(var reach in group)
            {
                Check(reach.StartLevel<=previous+.01f&&reach.EndLevel<=reach.StartLevel+.01f,"Junction splitting raised water along one source way");
                previous=reach.EndLevel;
            }
        }
        var ditch=Plan(Array.Empty<GeographicPolygon>(),data:Stream("ditch")).ConditionedHeightmap;
        Check(ditch==null||ditch[Pixel(32,0)]==original[Pixel(32,0)],"A ditch was widened like a stream");
        var openedPlan=Plan(new[]{river},data:Culverted(300));
        Check(new long[]{61,62,63}.All(id=>openedPlan.ReachStages.Any(r=>r.WayId==id))&&WaterBodyPlanner.BuriedChannels(Culverted(300),request).Count==1,
            "A long buried river still broke at its portals");
        Check(openedPlan.Sources.Any(s=>s.WayId==61)&&openedPlan.Sources.Any(s=>s.WayId==62)&&
            openedPlan.Sources.Select(s=>s.BodyId).Distinct().Count()==1&&openedPlan.ReachStages.All(r=>!r.BlockedByBuriedInlet),
            "Opening a buried river withheld its upstream sources or split its hydraulic identity");
        Check(WaterBodyPlanner.BuriedChannels(Culverted(300),request,28).Single().HalfWidth>46&&
            WaterBodyPlanner.BuriedChannels(Culverted(300),request,56).Single().HalfWidth>93,
            "Opened-channel bridge footprints do not follow the actual native grid");
        Check(WaterBodyPlanner.BuriedChannels(Culverted(300),request,float.NaN).Count==0,"An unknown grid generated guessed buried-channel footprints");
        var outsideData=Culverted(300);var outsideNodes=outsideData.Nodes.ToDictionary(p=>p.Key,p=>p.Value);
        outsideNodes[7]=Node(7,0,600);outsideNodes[8]=Node(8,0,900);outsideNodes[3]=Node(3,0,1200);
        Check(Plan(new[]{river},data:new OsmDataset{Nodes=outsideNodes,Ways=outsideData.Ways}).Sources.Any(s=>s.WayId==61),
            "A buried inlet outside the map disabled a valid map-edge river outlet");
        var bypassData=Culverted(300);var bypassWays=bypassData.Ways.ToDictionary(p=>p.Key,p=>p.Value);
        bypassWays[64]=way with{Id=64,NodeIds=new long[]{7,3}};
        Check(Plan(new[]{river},data:new OsmDataset{Nodes=bypassData.Nodes,Ways=bypassWays}).Sources.Any(s=>s.WayId==61),"A real open alternative was disabled by a buried side branch");
        // A short geometric gap in the same named downstream river is repairable.
        // Diverging, ambiguous and differently named streetside channels are not.
        var gapNodes=new Dictionary<long,OsmNode>(nodes){{7,Node(7,0,-10)},{8,Node(8,0,10)}};
        var named=new Dictionary<string,string>{{"waterway","river"},{"name","Test Creek"}};
        var gapWays=new[]{way with{Id=81,NodeIds=new long[]{1,7},Tags=named},way with{Id=82,NodeIds=new long[]{8,3},Tags=named}};
        var gapData=new OsmDataset{Nodes=gapNodes,Ways=gapWays.ToDictionary(w=>w.Id)};
        var joined=Plan(Array.Empty<GeographicPolygon>(),data:gapData);
        Check(joined.Sources.Select(s=>s.BodyId).Distinct().Count()==1&&gapData.Ways[81].NodeIds.Length==2,"A supported river gap stayed disconnected or mutated shared source evidence");
        Check(WaterBodyPlanner.OpenChannels(gapData,request).Any(c=>c.Line.Length==3),"Road crossings did not receive the same repaired channel geometry");
        var otherName=gapWays[1] with{Tags=new Dictionary<string,string>{{"waterway","river"},{"name","Different Creek"}}};
        Check(WaterBodyPlanner.OpenChannels(new OsmDataset{Nodes=gapNodes,Ways=new[]{gapWays[0],otherName}.ToDictionary(w=>w.Id)},request).All(c=>c.Line.Length==2),"Different named channels were joined");
        var ambiguousWays=gapWays.Concat(new[]{gapWays[1] with{Id=83}}).ToDictionary(w=>w.Id);
        Check(WaterBodyPlanner.OpenChannels(new OsmDataset{Nodes=gapNodes,Ways=ambiguousWays},request).All(c=>c.Line.Length==2),"An ambiguous urban waterway connection was guessed");
        // A dry abandoned canal and a ditch narrower than a water cell are not
        // carved: one was a trench across a hillside, the other a row of pits.
        OsmDataset Tagged(params (string Key,string Value)[] tags)=>new OsmDataset{Nodes=nodes,Ways=new Dictionary<long,OsmWay>{{71,way with{Id=71,Tags=tags.ToDictionary(t=>t.Key,t=>t.Value)}}}};
        Check(Plan(Array.Empty<GeographicPolygon>(),data:Tagged(("waterway","canal"),("abandoned","yes"))).ConditionedHeightmap==null,"An abandoned canal was carved");
        Check(Plan(Array.Empty<GeographicPolygon>(),data:Tagged(("waterway","ditch"))).ConditionedHeightmap==null,"A sub-grid ditch was carved");
        var excludedStream=Tagged(("waterway","stream"),("width","50"));
        var streamPlan=Plan(Array.Empty<GeographicPolygon>(),data:excludedStream);
        Check(streamPlan.Sources.Count==0&&streamPlan.ReachStages.Count==0&&streamPlan.ConditionedHeightmap==null&&
            WaterBodyPlanner.OpenChannels(excludedStream,request).Count==0,"An excluded stream generated water, terrain cuts or road bridges");
        var streamArea=river with{Tags=new Dictionary<string,string>{{"natural","water"},{"water","stream"}}};
        Check(Plan(new[]{streamArea},data:new OsmDataset()).AreaStages.Count==0,"A stream polygon was imported as a lake");
        Check(WaterBodyPlanner.OpenChannels(Tagged(("waterway","canal"),("abandoned","yes")),request).Count==0,"An abandoned canal still forces culvert bridges");
        // A river polygon narrower than the grid still gets a channel along its
        // line below its stage; masked to the polygon, the Huveaune's bed stood
        // near ground level between the few grid points inside it.
        var thin=river with{Id=98,Outer=new(){Ring(-8,-480,8,480)}};var thinPlan=Plan(new[]{thin});
        var thinSurface=WaterBodyPlanner.SurfaceSampler(thinPlan,new[]{thin});
        Check(Enumerable.Range(-4,9).All(k=>thinSurface(new float3(0,0,k*20)) is float stage&&NetworkProcessor.Sample(0,k*20,thinPlan.ConditionedHeightmap!,100,1000)<stage-1),"A river polygon narrower than the grid was left without a channel below its stage");
        // The bed never rises downstream. A 10 m hollow beside the stream caps
        // its stage there, below the floor to which the reach solve may lower
        // the stream's nodes; downstream, under a higher stage, the bed stays
        // as deep.
        var dipped=heights.ToArray();for(var z=11;z<=15;z++)for(var x=35;x<=37;x++)dipped[z*64+x]=(ushort)(20f/100*65535);
        var dipPlan=Plan(Array.Empty<GeographicPolygon>(),terrain with{Heightmap=dipped},data:Stream("river"));var dipBed=dipPlan.ConditionedHeightmap!;
        float LineBed(float z)=>NetworkProcessor.Sample(0,z,dipBed,100,1000);
        Check(LineBed(-300)<LineBed(-450)-1,"Bank-dip fixture does not lower the stage midway");
        Check(Enumerable.Range(-12,33).All(k=>LineBed(k*20)<=LineBed(-300)+.1f),"The river bed rose downstream of a bank dip, damming the water above it");
        // A lake's outlet river runs through one of its bank probes (each
        // shore vertex probes east first). Water reaching the stream's own
        // channel is not a spill: the lake keeps its control instead of
        // staying a dry pit.
        var pond=river with{Id=97,Tags=new Dictionary<string,string>{{"natural","water"},{"water","lake"}},
            Outer=new(){new[]{new float3(-200,0,-200),new float3(200,0,-200),new float3(200,0,0),new float3(200,0,200),new float3(-200,0,200)}}};
        var outletNodes=new Dictionary<long,OsmNode>{{20,Node(20,150,0)},{21,Node(21,480,0)}};
        var outlet=new OsmDataset{Nodes=outletNodes,Ways=new Dictionary<long,OsmWay>{{72,new OsmWay(72,new long[]{20,21},new Dictionary<string,string>{{"waterway","river"}})}}};
        var pondPlan=Plan(new[]{pond},data:outlet);
        Check(pondPlan.Sources.Any(s=>s.BodyKind=="lake")&&NetworkProcessor.Sample(256,0,pondPlan.ConditionedHeightmap!,100,1000)<pondPlan.AreaStages.Single(a=>a.BodyId==pond.Identity).Level-1,"Outlet fixture has no lake control or no channel through its east probe");
        var pondFinal=WaterBodyPlanner.Revalidate(pondPlan,terrain with{Heightmap=pondPlan.ConditionedHeightmap!},1000,new ImportLog(),default);
        Check(pondFinal.Sources.Count(s=>s.BodyKind=="lake")==pondPlan.Sources.Count(s=>s.BodyKind=="lake"),"A lake lost its control to a bank probe in its own outlet channel");
        // Nominal sea metadata is not an evidenced receiving boundary for an
        // inland/no-coast fixture. It must neither raise stages nor relax banks.
        var highNominalSea=Plan(new[]{river},terrain with{SeaLevelMeters=50});
        Check(!highNominalSea.Coastal&&highNominalSea.Sources.Count==plan.Sources.Count&&
            highNominalSea.Sources.Select(s=>s.TargetSurfaceLevel).SequenceEqual(plan.Sources.Select(s=>s.TargetSurfaceLevel))&&
            highNominalSea.Sources.All(s=>s.TargetSurfaceLevel<=s.BankLevel-.35f),"Nominal sea metadata changed an unconnected river stage or bypassed bank containment");
        // Classification selects natural waterways independently of mapped width.
        // The simulated channel still needs enough room for the native water grid.
        foreach(var type in new[]{"river"})foreach(var mappedWidth in new[]{"0.5","4"})
        {
            var narrow=Tagged(("waterway",type),("width",mappedWidth));var narrowPlan=Plan(Array.Empty<GeographicPolygon>(),data:narrow);
            Check(narrowPlan.Sources.Count>0&&narrowPlan.ReachStages.Any(r=>r.WayId==71)&&
                narrowPlan.Sources.All(s=>Math.Abs(s.Position.x)+s.Radius<=28.01f),"A narrow natural waterway was excluded or spilled outside its simulated channel");
            Check(WaterBodyPlanner.OpenChannels(narrow,request).Count==1,"A narrow natural waterway lost its road-crossing protection");
        }
        foreach(var mappedWidth in new[]{"0.5","4","100"})
        {
            var canal=Tagged(("waterway","canal"),("width",mappedWidth));var excluded=Plan(Array.Empty<GeographicPolygon>(),data:canal);
            Check(excluded.Sources.Count==0&&excluded.ReachStages.Count==0&&excluded.ConditionedHeightmap==null,"A canal received water controls or terrain cuts");
            Check(WaterBodyPlanner.OpenChannels(canal,request).Count==0,"An excluded canal forced a road bridge");
        }
        var connectedNodes=new Dictionary<long,OsmNode>(nodes){{4,Node(4,100,600)}};
        var natural=way with{NodeIds=new long[]{1,2}};
        var canalLink=way with{Id=73,NodeIds=new long[]{2,3},Tags=new Dictionary<string,string>{{"waterway","canal"}}};
        var remoteDrain=way with{Id=74,NodeIds=new long[]{3,4},Tags=new Dictionary<string,string>{{"waterway","drain"}}};
        var connected=new OsmDataset{Nodes=connectedNodes,Ways=new[]{natural,canalLink,remoteDrain}.ToDictionary(w=>w.Id)};
        Check(WaterBodyPlanner.OpenChannels(connected,request).Count==1,"A canal admitted its connected drainage network into the natural river network");
        var directDrain=new OsmDataset{Nodes=connectedNodes,Ways=new[]{natural,remoteDrain with{NodeIds=new long[]{2,4}}}.ToDictionary(w=>w.Id)};
        Check(WaterBodyPlanner.OpenChannels(directDrain,request).Count==2,"Excluding canals broke a narrow drain directly connected to a river");
        var canalCulvert=Culverted(20);var culvertWays=canalCulvert.Ways.ToDictionary(p=>p.Key,p=>p.Value);
        culvertWays[62]=culvertWays[62] with{Tags=new Dictionary<string,string>{{"waterway","canal"},{"tunnel","culvert"}}};
        Check(WaterBodyPlanner.BuriedChannels(new OsmDataset{Nodes=canalCulvert.Nodes,Ways=culvertWays},request).Count==0,"An excluded canal culvert forced a road bridge");
        foreach(var canalTags in new[]{new Dictionary<string,string>{{"natural","water"},{"water","canal"}},
            new Dictionary<string,string>{{"natural","water"},{"waterway","canal"}},
            new Dictionary<string,string>{{"waterway","riverbank"},{"water","canal"}}})
        {
            var canalArea=river with{Tags=canalTags};var excluded=Plan(new[]{canalArea},data:new OsmDataset());
            Check(excluded.Sources.Count==0&&excluded.AreaStages.Count==0&&excluded.ConditionedHeightmap==null,"A canal polygon was carved or treated as a lake");
            var oldStage=excluded with{AreaStages=new[]{new WaterAreaStage(canalArea.Identity,"river",30)}};
            Check(WaterBodyPlanner.SurfaceSampler(oldStage,new[]{canalArea})(float3.zero)==null,"An excluded canal polygon supplied a water surface");
        }
        var earthworks=plan.ConditionedHeightmap.ToArray();var source=plan.Sources.First();
        var obstruction=Pixel(source.Position.x,source.Position.z);
        // Raise the complete bilinear neighbourhood. One nearest raster sample
        // did not actually obstruct the off-grid source's interpolated bed.
        for(var z=Math.Max(0,obstruction/64-1);z<=Math.Min(63,obstruction/64+1);z++)
            for(var x=Math.Max(0,obstruction%64-1);x<=Math.Min(63,obstruction%64+1);x++)
                earthworks[z*64+x]=(ushort)((source.TargetSurfaceLevel!.Value+.5f)/100*65535);
        Check(NetworkProcessor.Sample(source.Position.x,source.Position.z,earthworks,100,1000)>source.TargetSurfaceLevel!.Value,"Protected-source fixture does not actually obstruct the bilinear bed");
        var revisionLog=new ImportLog();var revised=WaterBodyPlanner.Revalidate(plan,terrain with{Heightmap=earthworks},1000,revisionLog,default);
        Check(revised.Sources.Count==plan.Sources.Count&&revised.ConditionedHeightmap!=null,"Small final-earthworks obstruction was not recovered: "+string.Join(" | ",revisionLog.Lines));
        var protectedRoad=new PreparedSegment(5,10,11,source.Position-new float3(100,0,0),source.Position+new float3(100,0,0),NetworkKind.Road,"primary",2,false,false,false,0){NativeWidth=16};
        var ownedWithoutControls=plan with{Sources=Array.Empty<RiverSourcePlan>()};
        var restoredOwnedBed=WaterBodyPlanner.Revalidate(ownedWithoutControls,terrain with{Heightmap=earthworks},1000,new ImportLog(),default);
        Check(restoredOwnedBed.ConditionedHeightmap!=null&&restoredOwnedBed.ConditionedHeightmap[Pixel(source.Position.x,source.Position.z)]==plan.PlannedBedHeightmap![Pixel(source.Position.x,source.Position.z)],"Accepted water-owned bed depended on a native regulator being present");
        var noControls=ownedWithoutControls with{BedEdits=null,PlannedBedHeightmap=null};var emptyLog=new ImportLog();
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
        // A shore at an angle to the grid must not shade as a staircase. That
        // staircase is a bank whose steepness jumps between neighbouring terrain
        // vertices (one on a one-pixel cliff, the next flat), so shading flips
        // along a zigzag. Measured as the 95th-percentile steepness contrast
        // between adjacent grid points within 40 m of a 30° edge, relative to
        // the peak steepness there.
        var shoreA=new float2(-350,-250);var shoreB=new float2(350,154);
        var slanted=new GeographicPolygon(101,new Dictionary<string,string>{{"natural","water"},{"water","lake"}},new(){new[]{new float3(shoreA.x,0,-400),new float3(shoreB.x,0,-400),new float3(shoreB.x,0,shoreB.y),new float3(shoreA.x,0,shoreA.y)}},new());
        var slantedPlan=Plan(new[]{slanted},data:new OsmDataset());var bed=slantedPlan.ConditionedHeightmap!;
        var along=math.normalize(shoreB-shoreA);var normal=new float2(-along.y,along.x);var grid=1000/63f;
        var steepness=new float[64*64];var near=new bool[64*64];
        for(var z=1;z<63;z++)for(var x=1;x<63;x++)
        {
            var p=new float2(x*grid-500,z*grid-500);var t=math.dot(p-shoreA,along)/math.distance(shoreA,shoreB);
            near[z*64+x]=t>=.15f&&t<=.85f&&Math.Abs(math.dot(p-shoreA,normal))<=40;
            float H(int i,int k)=>bed[k*64+i]/65535f*100;
            steepness[z*64+x]=math.length(new float2(H(x+1,z)-H(x-1,z),H(x,z+1)-H(x,z-1))/(2*grid));
        }
        var contrasts=new List<float>();
        for(var z=1;z<62;z++)for(var x=1;x<62;x++)if(near[z*64+x])
            foreach(var k in new[]{z*64+x+1,(z+1)*64+x})if(near[k])contrasts.Add(Math.Abs(steepness[z*64+x]-steepness[k]));
        contrasts.Sort();var peakSteep=Enumerable.Range(0,64*64).Where(i=>near[i]).Max(i=>steepness[i]);
        var contrast=contrasts[(int)(.95*(contrasts.Count-1))]/Math.Max(1e-6f,peakSteep);
        Console.WriteLine($"Shoreline: neighbouring-vertex steepness contrast near a 30° shore {contrast:P0} of peak steepness {peakSteep:0.000} (grid 15.9 m).");
        Check(contrast<.5,$"Slanted shore shades as a staircase: neighbouring vertices differ by {contrast:P0} of the peak steepness");
        // The waterline itself: where the bilinear terrain crosses the lake's
        // stage, walking in from the land along the shore's normal, every 2 m
        // along the shore. A height that steps at the mapped edge put that
        // crossing wherever each grid point fell and drew the shore as teeth.
        var stageLevel=slantedPlan.AreaStages.Single(s=>s.BodyId==slanted.Identity).Level;var crossings=new List<float>();
        for(var s=.2f;s<=.8f;s+=2/math.distance(shoreA,shoreB))
        {
            var o=math.lerp(shoreA,shoreB,s);float previous=float.NaN;
            for(var d=40f;d>=-40f;d-=.25f)
            {
                var p=o+normal*d;var h=NetworkProcessor.Sample(p.x,p.y,bed,100,1000);
                if(!float.IsNaN(previous)&&previous>=stageLevel&&h<stageLevel){crossings.Add(d+.25f*(previous-stageLevel)/(previous-h));break;}
                previous=h;
            }
        }
        var mean=crossings.Average();var wander=crossings.Select(c=>Math.Abs(c-mean)).OrderBy(c=>c).ElementAt((int)(.95*(crossings.Count-1)));
        Console.WriteLine($"Shoreline: waterline of a 30° shore {mean:0.0} m from the mapped edge, wandering by {wander:0.00} m (95th percentile) over {crossings.Count} samples.");
        Check(crossings.Count>100&&Math.Abs(mean)<3&&wander<.5,$"Slanted shore's waterline is toothed or off its mapped edge: mean {mean:0.0} m, wander {wander:0.00} m");
        var diamondPlan=slantedPlan;
        // Low bank land may rise to the waterline lip (at most 0.8 m), never more.
        for(var i=0;i<heights.Length;i++)Check((diamondPlan.ConditionedHeightmap![i]-heights[i])/65535f*100<=.801&&(heights[i]-diamondPlan.ConditionedHeightmap[i])/65535f*100<=8.001,"Shore shaping raised terrain past the lip or exceeded its budget");
        Check(diamondPlan.ConditionedHeightmap![Pixel(0,350)]==original[Pixel(0,350)],"Shore shaping touched land beyond the shore band");
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
        // Surface sampling consumes canonical typed area stages, not the
        // optional native control list. Retain both products in this fixture.
        var water=new WaterPlan(false,0,2,0,"fixture")
        {Sources=new[]{Source(broad.Identity,42),Source(small.Identity,19)},
            AreaStages=new[]{new WaterAreaStage(broad.Identity,"lake",42),new WaterAreaStage(small.Identity,"lake",19)}};
        Check(WaterBodyPlanner.SurfaceSampler(water,new[]{broad,small})(float3.zero)==42,"Global area lost precedence over later local area");
        Check(WaterBodyPlanner.SurfaceSampler(water,new[]{small,broad})(float3.zero)==19,"Local area lost precedence over later global area");
    }
}
