using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

static class AvenueTests
{
    static void Check(bool ok,string message){if(!ok)throw new Exception(message);}
    static PreparedSegment S(long way,long from,long to,float x,float z,float endX,float endZ)=>new(way,from,to,new(x,0,z),new(endX,0,endZ),NetworkKind.Road,"primary",2,true,false,false,0){Name="Main"};
    public static void Run()
    {
        var a=S(1,1,2,0,0,100,0);var b=S(2,3,4,100,10,0,10);
        IReadOnlyList<PreparedSegment> Run(params PreparedSegment[] edges)=>AvenueConsolidator.Prepare(edges,new ImportLog(),default);
        var pair=Run(a,b);
        Check(Run(a with{Lanes=1},b with{Lanes=1}).All(e=>e.ConsolidatedAvenue&&e.Lanes==4&&e.ForwardLanes==2&&e.BackwardLanes==2&&!e.OneWay),"Approved minimum 2+2 boulevard layout missing");
        Check(pair.Count>0&&pair.All(e=>e.ConsolidatedAvenue&&!e.OneWay&&e.Lanes==4&&Math.Abs(e.Start.z-5)<.001&&Math.Abs(e.End.z-5)<.001),"Opposing carriageways did not become a four-lane centerline avenue");
        Check(math.distance(pair[0].Start.xz,new float2(0,5))<.001&&math.distance(pair[pair.Count-1].End.xz,new float2(100,5))<.001,"Avenue discarded corridor endpoints");
        Check(Run(b,a).Select(e=>(e.WayId,e.StartNodeId,e.EndNodeId,e.Start,e.End,e.Lanes,e.OneWay)).SequenceEqual(pair.Select(e=>(e.WayId,e.StartNodeId,e.EndNodeId,e.Start,e.End,e.Lanes,e.OneWay))),"Input ordering changes consolidation");
        Check(Run(a,b with{Name="Different"}).Count==2,"Unrelated named streets merged");
        Check(Run(a,b with{Start=b.End,End=b.Start,StartNodeId=4,EndNodeId=3}).Count==2,"Same-direction carriageways merged");
        Check(Run(a with{Classification="motorway"},b with{Classification="motorway"}).Count==2,"Highways converted into avenues");
        Check(Run(a with{Classification="trunk"},b with{Classification="trunk"}).Count==2,"Trunk highways converted into avenues");
        Check(Run(a with{Classification="primary_link"},b with{Classification="primary_link"}).Count==2,"Ramps converted into avenues");
        Check(Run(a with{Bridge=true},b with{Bridge=true}).Count==2,"Elevated structures unexpectedly rewritten");
        Check(Run(a,b with{Layer=1}).Count==2,"Different elevations merged");
        Check(Run(a,b,S(9,2,20,100,0,150,0) with{Bridge=true}).All(e=>!e.ConsolidatedAvenue),"Structural transition node disconnected by avenue remapping");
        Check(Run(a,b,S(9,2,20,100,0,150,0) with{Kind=NetworkKind.Rail}).All(e=>!e.ConsolidatedAvenue),"Rail crossing remapped without a verified shared transition");
        Check(Run(a,b with{Start=new(100,0,50),End=new(0,0,50)}).All(e=>!e.ConsolidatedAvenue),"Widely separated roads merged");
        var ambiguous=S(3,10,11,0,20,100,20);
        Check(Run(a,b,ambiguous).All(e=>!e.ConsolidatedAvenue),"Non-mutual ambiguous pair merged");
        Check(Run(a,b with{Name=""},a with{WayId=3,StartNodeId=5,EndNodeId=6,Start=new(0,0,20),End=new(100,0,20)}).All(e=>!e.ConsolidatedAvenue),"Unnamed/ambiguous neighboring streets incorrectly converted");
        var first=S(1,1,5,0,0,50,0);var second=S(3,5,2,50,0,100,0);
        var side=S(9,8,5,50,-50,50,0) with{Name="Side",OneWay=false};
        var connected=Run(first,second,b,side);
        var attachment=connected.Single(e=>e.WayId==9);
        Check(math.distance(attachment.End.xz,new float2(50,5))<.001&&connected.Any(e=>e.ConsolidatedAvenue&&(e.StartNodeId==attachment.EndNodeId||e.EndNodeId==attachment.EndNodeId)),"Side street lost connection at an internal corridor junction");
        var entry=S(8,100,1,-50,0,0,0) with{Name="Entry",OneWay=false};
        var exit=S(9,2,101,100,0,150,0) with{Name="Exit",OneWay=false};
        var tied=Run(a,b,entry,exit);var from=tied.Single(e=>e.WayId==8).EndNodeId;var to=tied.Single(e=>e.WayId==9).StartNodeId;
        var seen=new HashSet<long>{from};var queue=new Queue<long>();queue.Enqueue(from);
        while(queue.Count>0){var n=queue.Dequeue();foreach(var e in tied.Where(e=>e.StartNodeId==n||e.EndNodeId==n)){var next=e.StartNodeId==n?e.EndNodeId:e.StartNodeId;if(seen.Add(next))queue.Enqueue(next);}}
        Check(seen.Contains(to),"Consolidation severed route between original attachments");
        var reverseFirst=S(2,3,6,100,10,65,10);var reverseSecond=S(4,6,4,65,10,0,10);
        var mismatch=Run(first,second,reverseFirst,reverseSecond,side);
        Check(mismatch.Count(e=>e.ConsolidatedAvenue)>0&&mismatch.Single(e=>e.WayId==9).End.x==50,"Unequal segmentation lost a side-road junction");
        var median=S(10,5,6,50,0,50,10) with{Classification="primary_link",Name="Median"};
        var alignedReverseFirst=S(2,3,6,100,10,50,10);var alignedReverseSecond=S(4,6,4,50,10,0,10);
        var medianResult=Run(first,second,alignedReverseFirst,alignedReverseSecond,median);
        Check(medianResult.Any(e=>e.ConsolidatedAvenue)&&medianResult.All(e=>e.StartNodeId!=e.EndNodeId),"Median crossing consolidation created a self-loop");
        JunctionRegressions();
        var normalized=NetworkNormalizer.Normalize(connected,new ImportLog(),default,1000);
        NetworkNormalizer.Validate(normalized,1000,default);
        var terrain=Enumerable.Repeat((ushort)32768,TerrainBuilder.Resolution*TerrainBuilder.Resolution).ToArray();
        var aligned=NetworkProcessor.AlignToTerrain(normalized,terrain,100,1000,default);
        Check(NetworkVerticalProfiles.Prepare(aligned,terrain,100,1000,new ImportLog(),default).Any(e=>e.ConsolidatedAvenue),"Avenue cannot pass normal grading pipeline");
        using var cancel=new CancellationTokenSource();cancel.Cancel();
        try{AvenueConsolidator.Prepare(new[]{a,b},new ImportLog(),cancel.Token);throw new Exception("Cancellation ignored");}catch(OperationCanceledException){}
        Console.WriteLine("PASS: opposing one-ways to vanilla avenue centerlines, cross-way chains, junction/end reconnection, route reachability, unmatched/highway/ramp/structure/level guards, determinism and grading integration.");
    }

    static void JunctionRegressions()
    {
        // Exact dimensions from the Palm Springs failure: two projected
        // stations 16.6 m apart and a transverse source median connection.
        var left=S(-100,-1,-2,15202.5f,-11504.86f,15099.667f,-11503.515f) with{ConsolidatedAvenue=true,OneWay=false,Lanes=6,NativeWidth=32};
        var internalPiece=S(-100,-2,-3,15099.667f,-11503.515f,15083.043f,-11503.26f) with{ConsolidatedAvenue=true,OneWay=false,Lanes=6,NativeWidth=32};
        var right=S(-100,-3,-4,15083.043f,-11503.26f,14863.046f,-11500.373f) with{ConsolidatedAvenue=true,OneWay=false,Lanes=6,NativeWidth=32};
        var median=S(1220190819,-3,-2,15083.043f,-11503.26f,15099.667f,-11503.515f) with{OneWay=false,OriginalGeometry=new[]{new float3(15090.907f,0,-11494.583f),new float3(15090.842f,0,-11512.228f)}};
        var result=AvenueJunctionRepair.Prepare(new[]{left,internalPiece,right,median},new ImportLog(),default);
        Check(result.Count==2&&result[0].EndNodeId==result[1].StartNodeId&&math.all(result[0].End==result[1].Start),"Palm Springs median link still duplicates a tiny avenue interval");
        Check(math.all(result[0].Start==left.Start)&&math.all(result[1].End==right.End)&&result.All(s=>s.SourceGeometry.Count==2&&s.Control1==null),"Canonical median changed distant endpoints or left stale curve geometry");
        Check(AvenueJunctionRepair.Prepare(new[]{median,right,internalPiece,left},new ImportLog(),default).OrderBy(s=>s.StartNodeId).Select(s=>(s.StartNodeId,s.EndNodeId,s.Start,s.End)).SequenceEqual(result.OrderBy(s=>s.StartNodeId).Select(s=>(s.StartNodeId,s.EndNodeId,s.Start,s.End))),"Median canonicalization depends on input ordering");
        Check(AvenueJunctionRepair.Prepare(result,new ImportLog(),default).SequenceEqual(result),"Median repair is not idempotent");
        var unchanged=median with{OriginalGeometry=new[]{median.Start,median.End}};
        Check(AvenueJunctionRepair.Prepare(new[]{left,internalPiece,right,unchanged},new ImportLog(),default).Count==4,"A real longitudinal short street was consumed as a median");
        Check(AvenueJunctionRepair.Prepare(new[]{left,internalPiece,right,median with{Layer=1}},new ImportLog(),default).Count==4,"Different layers canonicalized");
        Check(AvenueJunctionRepair.Prepare(new[]{left,internalPiece,right,median with{Bridge=true}},new ImportLog(),default).Count==4,"Bridge portal consumed");
        var rail=S(45,-2,50,left.End.x,left.End.z,left.End.x+50,left.End.z) with{Kind=NetworkKind.Rail};
        Check(AvenueJunctionRepair.Prepare(new[]{left,internalPiece,right,median,rail},new ImportLog(),default).Count==5,"Road/rail junction canonicalized without source-mode proof");
        // Oman: two distinct consolidated corridors connected by an explicit
        // ~1 m source link (not an arbitrary nearby node).
        var old=S(-200,-10,-11,-11536.171f,18037.46f,-11456.347f,18008.092f) with{ConsolidatedAvenue=true,OneWay=false,NativeWidth=24,SourceWayIds=new long[]{36064432,183177670}};
        var approach=S(-201,-12,-13,-11394.675f,17985.45f,-11455.377f,18007.832f) with{ConsolidatedAvenue=true,OneWay=false,Lanes=6,NativeWidth=32};
        var tie=S(183177670,-13,-11,approach.End.x,approach.End.z,old.End.x,old.End.z);
        var oman=AvenueJunctionRepair.Prepare(new[]{old,approach,tie},new ImportLog(),default);
        Check(oman.Count==2&&oman[0].EndNodeId==oman[1].EndNodeId&&math.all(oman[0].End==oman[1].End),"Oman explicit short tie-in still has two competing junction owners");
        Check(AvenueJunctionRepair.Prepare(new[]{old,approach},new ImportLog(),default).Count==2&&old.EndNodeId!=approach.EndNodeId,"Unconnected nearby roads were proximity-joined");
        Check(AvenueJunctionRepair.Prepare(new[]{old with{SourceWayIds=Array.Empty<long>()},approach,tie},new ImportLog(),default).Count==3,"Short connection lacking consolidation provenance was removed");
        using var cancel=new CancellationTokenSource();cancel.Cancel();
        try{AvenueJunctionRepair.Prepare(new[]{left,internalPiece,right,median},new ImportLog(),cancel.Token);throw new Exception("Junction cancellation ignored");}catch(OperationCanceledException){}
        Console.WriteLine("PASS: captured-dimension Palm Springs/Oman median topology recovery, deterministic/idempotent canonical junctions, unchanged distant arms, and unconnected/longitudinal/structure/layer/rail/provenance guards.");
    }
}
