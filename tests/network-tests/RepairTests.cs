using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

internal static class RepairTests
{
    private static void Check(bool pass,string message){if(!pass)throw new Exception(message);}
    private static PreparedSegment S(long way,long a,long b,float x,float z,float endX,float endZ,NetworkKind kind=NetworkKind.Road)=>
        new(way,a,b,new(x,50,z),new(endX,50,endZ),kind,"primary",2,true,false,false,0){SourceGeometry=new[]{new float3(x,50,z),new float3(endX,50,endZ)}};
    public static void Run()
    {
        var log=new ImportLog();var flat=Enumerable.Repeat((ushort)32768,128*128).ToArray();
        var ground=S(1,1,2,-100,0,100,0) with{Layer=2};
        var bridgeJoin=S(2,2,3,100.00001f,0,200,0) with{Bridge=true,StartElevation=8,EndElevation=8};
        var joined=NativeEndpointPolicy.Canonicalize(new[]{ground,bridgeJoin},log);
        Check(math.all(joined[0].End==joined[1].Start),"Native exact-position keys still split a shared bridge portal");
        var nodeElevations=NativeEndpointPolicy.Elevations(joined);
        Check(nodeElevations[NativeEndpointPolicy.Identity(ground,2)]==8,"Bridge portal composition not shared on the ground course");
        var independent=bridgeJoin with{StartNodeId=99};
        Check(NativeEndpointPolicy.Canonicalize(new[]{ground,independent},log)[1].Start.x==independent.Start.x,"Nearby unrelated road snapped by native canonicalization");
        Check(bridgeJoin.Start.x==100.00001f,"Native canonicalization mutated source geometry");
        var alignment=NetworkProcessor.AlignToTerrain(new[]{ground},flat,100,1000,default);
        Check(Math.Abs(alignment[0].Start.y-50)<.01&&NetworkStructure.Offset(ground)==0,"OSM layer became artificial ground-road elevation");
        var pair=new[]{S(1,1,2,-100,0,100,0) with{CorridorId=1,MaximumNativeWidth=5},S(2,3,4,100,6,-100,6) with{CorridorId=1,MaximumNativeWidth=5},S(3,2,5,100,0,150,-50)};
        var repaired=NetworkWidthRepair.Prepare(pair,_=>16,1000,log,default);
        Check(repaired.Count==pair.Length&&repaired.Take(2).All(s=>s.MaximumNativeWidth>=16),"Vanilla width repair deleted roads or failed separation");
        Check(math.distance(repaired[0].End,repaired[2].Start)<.001,"Width repair detached a shared branch");
        NetworkContinuity.Validate(pair,repaired,log,default);
        var tinyPair=new[]{S(401,401,402,-100,0,100,0),S(402,403,404,100,16.999f,-100,10),S(403,402,405,100,0,180,-60)};
        var tinyRepair=NetworkWidthRepair.Prepare(tinyPair,_=>16,1000,new ImportLog(),default);
        Check(math.all(tinyRepair.Single(s=>s.WayId==401).End==tinyRepair.Single(s=>s.WayId==403).Start),"Repeated sub-millimetre moves split a native shared junction");
        var fork=new[]{S(451,451,452,0,0,220,0),S(452,451,453,0,0,220,5)};
        var separatedFork=NetworkWidthRepair.Prepare(fork,_=>16,1000,new ImportLog(),default);
        Check(math.all(separatedFork[0].Start==separatedFork[1].Start),"Remote fork separation broke its shared merge anchor");
        Check(math.distance(separatedFork[0].End.xz,separatedFork[1].End.xz)>=16,"Sharing a merge node exempted the entire overlapping remote fork");
        foreach(var side in new[]{-1,1})
        {
            var boundaryPair=new[]{S(21,101,102,-100,side*499,100,side*499) with{CorridorId=21,MaximumNativeWidth=5},S(22,103,104,100,side*493,-100,side*493) with{CorridorId=21,MaximumNativeWidth=5},S(23,102,105,100,side*499,180,side*450)};
            var before=boundaryPair.ToArray();
            var bounded=NetworkWidthRepair.Prepare(boundaryPair,_=>16,1000,log,default);
            Check(bounded.Count==boundaryPair.Length&&boundaryPair.SequenceEqual(before),"Boundary repair deleted roads or mutated input");
            Check(math.distance(bounded.First(s=>s.WayId==21).End,bounded.First(s=>s.WayId==23).Start)<.001,"Boundary constraint detached a shared junction");
            Check(bounded.SelectMany(s=>Enumerable.Range(0,65).Select(i=>NetworkNormalizer.Evaluate(s,i/64f))).All(p=>Math.Abs(p.x)<=500&&Math.Abs(p.z)<=500),"Boundary adjustment left the map");
            NetworkContinuity.Validate(boundaryPair,bounded,log,default);
            var eastPair=boundaryPair.Select(s=>s with{Start=new(s.Start.z,s.Start.y,s.Start.x),End=new(s.End.z,s.End.y,s.End.x),SourceGeometry=s.SourceGeometry.Select(p=>new float3(p.z,p.y,p.x)).ToArray()}).ToArray();
            var eastBounded=NetworkWidthRepair.Prepare(eastPair,_=>16,1000,log,default);
            Check(eastBounded.SelectMany(s=>Enumerable.Range(0,65).Select(i=>NetworkNormalizer.Evaluate(s,i/64f))).All(p=>Math.Abs(p.x)<=500&&Math.Abs(p.z)<=500),"East/west boundary adjustment left the map");
            NetworkContinuity.Validate(eastPair,eastBounded,log,default);
        }
        var curvePair=new[]{S(31,201,202,-100,490,100,490) with{CorridorId=31,MaximumNativeWidth=5,Control1=new float3(-35,50,499),Control2=new float3(35,50,499)},S(32,203,204,100,484,-100,484) with{CorridorId=31,MaximumNativeWidth=5},S(33,202,205,100,490,180,450)};
        var curveBounded=NetworkWidthRepair.Prepare(curvePair,_=>16,1000,log,default);
        Check(math.all(curveBounded.First(s=>s.WayId==31).Start==curvePair[0].Start)&&math.all(curveBounded.First(s=>s.WayId==31).Control1!.Value==curvePair[0].Control1!.Value),"Unsafe curve repair must preserve its original geometry");
        Check(curveBounded.Count==3&&math.distance(curveBounded.First(s=>s.WayId==31).End,curveBounded.First(s=>s.WayId==33).Start)<.001,"Curve rollback deleted a road or detached its branch");
        NetworkContinuity.Validate(curvePair,curveBounded,log,default);
        Check(log.Lines.Any(s=>s.Contains("movement or map boundary limit")&&s.Contains("final safety audit is still required")),"Boundary fallback must explain why repair was limited and must not imply acceptance before the final audit");
        var lifted=ground with{Start=ground.Start+new float3(0,3,0),End=ground.End+new float3(0,3,0)};
        var original=(ushort[])flat.Clone();
        var protectedRing=new[]{new float3(-10,0,-10),new float3(10,0,-10),new float3(10,0,10),new float3(-10,0,10)};
        var bed=NetworkRoadbed.Prepare(flat,100,1000,new[]{lifted},new[]{protectedRing},Array.Empty<float3[]>(),log,default);
        Check(flat.SequenceEqual(original)&&bed.Where((p,i)=>p!=flat[i]).Any(),"Roadbed mutated its input or changed nothing");
        Check(bed[64*128+64]==flat[64*128+64]&&bed[0]==flat[0],"Roadbed modified protected water or distant terrain");
        Check(bed.Zip(flat,(a,b)=>Math.Abs((int)a-b)*100/65535f).Max()<=6.001f,"Roadbed exceeded bounded cut/fill");
        var structural=NetworkRoadbed.Prepare(flat,100,1000,new[]{lifted with{Bridge=true}},Array.Empty<float3[]>(),Array.Empty<float3[]>(),log,default);
        Check(structural.SequenceEqual(flat),"Bridge raised terrain into a long embankment");
        var laterWater=(ushort[])bed.Clone();laterWater[0]=1234;
        var lateProfile=lifted with{Start=lifted.Start-new float3(0,2,0),End=lifted.End-new float3(0,2,0)};
        var refreshedBed=NetworkRoadbed.Refresh(flat,bed,laterWater,100,1000,new[]{lifted},new[]{lateProfile},new[]{protectedRing},Array.Empty<float3[]>(),log,default);
        var expectedBed=NetworkRoadbed.Prepare(flat,100,1000,new[]{lateProfile},new[]{protectedRing},Array.Empty<float3[]>(),new ImportLog(),default);
        Check(refreshedBed[0]==1234&&refreshedBed.Skip(1).SequenceEqual(expectedBed.Skip(1)),"Late roadbed refresh lost water edits or accumulated a second cut/fill budget");
        Check(flat.SequenceEqual(original)&&laterWater[0]==1234,"Late roadbed refresh mutated its input raster");
        var triangularWater=new[]{new float3(-200,0,-200),new float3(200,0,-200),new float3(-200,0,200)};
        var dryRoad=S(501,501,502,80,140,160,140) with{Start=new(80,53,140),End=new(160,53,140)};
        var nearWaterBed=NetworkRoadbed.Prepare(flat,100,1000,new[]{dryRoad},new[]{triangularWater},Array.Empty<float3[]>(),new ImportLog(),default);
        Check(nearWaterBed[82*128+80]>flat[82*128+80],"Dry road inside the water polygon bounding box was incorrectly protected from cut/fill");
        var tunnel=new[]{S(5,10,11,-300,0,-100,0,NetworkKind.Rail),S(6,11,12,-100,0,100,0,NetworkKind.Rail) with{Tunnel=true},S(7,12,13,100,0,300,0,NetworkKind.Rail)};
        var profiled=RailVerticalProfiles.Prepare(tunnel,flat,100,1000,log,default);
        NetworkContinuity.Validate(tunnel,profiled,log,default);
        Check(profiled.Where(s=>s.Tunnel).Min(s=>s.End.y)<47&&profiled.Where(s=>s.Tunnel).Any(s=>s.StartElevation<0&&s.EndElevation<0)&&profiled.All(s=>s.Lanes==2),"Rail tunnel remained on the surface or retained multiple tracks");
        NetworkNormalizer.Validate(profiled,1000,default);
        Check(profiled.Where(s=>s.Tunnel).Min(s=>s.EndElevation)<-7,"Tunnel interior lacks native underground cover");
        var entry=profiled.First(s=>s.Tunnel);var exit=profiled.Last(s=>s.Tunnel);
        Check(Math.Abs(entry.Control1!.Value.y-entry.Start.y)<.001&&Math.Abs(exit.End.y-exit.Control2!.Value.y)<.001,"Tunnel portals introduce a vertical tangent kink into level approach tracks");
        for(var i=1;i<profiled.Count;i++)Check(math.distance(profiled[i-1].End,profiled[i].Start)<.001,"Rail portal/subdivision has a position gap");
        var covered=new[]{S(61,61,62,-300,0,-100,0,NetworkKind.Rail) with{Tunnel=true},S(62,62,63,-100,0,100,0,NetworkKind.Rail) with{Tunnel=true},S(63,63,64,100,0,300,0,NetworkKind.Rail) with{Tunnel=true}};
        var buried=RailVerticalProfiles.Prepare(covered,flat,100,1000,new ImportLog(),default);
        Check(buried.All(s=>s.StartElevation<=-7.99f&&s.EndElevation<=-7.99f),"Grade projection lifted an entirely underground rail corridor onto the surface");
        var highRail=S(71,71,72,0,-200,0,200,NetworkKind.Rail) with{Start=new float3(0,60,-200),End=new float3(0,60,200)};
        var overRail=S(72,73,74,-200,0,200,0) with{Bridge=true};
        var crossing=NetworkVerticalProfiles.Prepare(new[]{overRail,highRail},flat,100,1000,new ImportLog(),default);
        Check(NetworkNormalizer.Evaluate(crossing.First(s=>s.Kind==NetworkKind.Road),.5f).y>=67,"Road-over-rail clearance omitted the rail deck height");
        Check(crossing.Single(s=>s.Kind==NetworkKind.Rail)==highRail,"Road clearance solver moved a fixed rail obstacle");
        try{NetworkContinuity.Validate(new[]{ground},Array.Empty<PreparedSegment>(),log,default);throw new Exception("Lost route passed continuity gate");}catch(InvalidOperationException){}
        Check(log.Lines.Any(s=>s.Contains("continuity not verified")),"Missing continuity must be diagnosed and blocked");
        var rail=S(8,20,21,-100,0,100,0,NetworkKind.Rail);
        try{NetworkContinuity.Validate(new[]{rail,rail with{WayId=9,StartNodeId=22,EndNodeId=23}},new[]{rail,rail with{WayId=9,StartNodeId=22,EndNodeId=23}},log,default);throw new Exception("Duplicate rails passed continuity gate");}catch(InvalidOperationException){}
        Check(log.Lines.Any(s=>s.Contains("Rail overlap remains")),"Unresolved rail overlap must be diagnosed and blocked");
        RecoveryTests.Run();
        using(var cancellation=new CancellationTokenSource()){cancellation.Cancel();try{NetworkRoadbed.Prepare(flat,100,1000,new[]{lifted},Array.Empty<float3[]>(),Array.Empty<float3[]>(),log,cancellation.Token);throw new Exception("Roadbed ignored cancellation");}catch(OperationCanceledException){}}
        Console.WriteLine("PASS: vanilla-width separation, all four map boundaries, safe curve rollback with shared junctions/no deletions/immutable input, layer semantics, protected roadbed, connected rail portals and overlap/continuity gates.");
    }
}
