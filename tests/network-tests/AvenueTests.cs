using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;
using Newtonsoft.Json.Linq;

static class AvenueTests
{
    static void Check(bool ok,string message){if(!ok)throw new Exception(message);}
    static PreparedSegment S(long way,long from,long to,float x,float z,float endX,float endZ)=>new(way,from,to,new(x,0,z),new(endX,0,endZ),NetworkKind.Road,"primary",2,true,false,false,0){Name="Main"};
    public static void Run()
    {
        FragmentedCounterparts();
        DividedCrossing();
        TaperAttachmentGuard();
        StrictLocalMatches();
        StrictJunctionCompletion();
        var a=S(1,1,2,0,0,100,0);var b=S(2,3,4,100,10,0,10);
        IReadOnlyList<PreparedSegment> Run(params PreparedSegment[] edges)=>AvenueConsolidator.Prepare(edges,new ImportLog(),default);
        var pair=Run(a,b);
        var longRoad=new[]{S(20,20,21,-100,0,0,0),a,S(21,2,22,100,0,200,0)};
        var partial=Run(longRoad.Concat(new[]{b}).ToArray());
        Check(partial.Any(s=>s.StartNodeId==20)&&partial.Any(s=>s.EndNodeId==22),"Partial avenue matching removed unchanged exterior carriageway edges");
        var partialLog=new ImportLog();SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(longRoad.Concat(new[]{b}).ToArray()),partial,partialLog,default);
        Check(partialLog.Metric("Missing mandatory selected-source ports")==0&&partialLog.Metric("Unresolved selected-source reachability obligations")==0,
            "Partial avenue matching lost a required exterior source connection");
        Check(Run(a with{Lanes=1},b with{Lanes=1}).All(e=>!e.ConsolidatedAvenue&&e.Lanes==2&&e.ForwardLanes==1&&e.BackwardLanes==1&&!e.OneWay),"A 1+1 corridor did not become a regular two-way two-lane road");
        Check(Run(a with{Name=""},b with{Name=""}).All(e=>!e.OneWay),"Anonymous opposing streets escaped geometric consolidation");
        Check(Run(a,b with{Classification="residential"}).All(e=>e.ConsolidatedAvenue&&e.Classification=="primary"),"A street-class discrepancy prevented consolidation or lost the through-road class");
        Check(Run(a,b with{Start=new(100,0,25),End=new(0,0,25)}).All(e=>e.ConsolidatedAvenue),"A wide same-street median prevented consolidation");
        Check(Run(a,b with{Start=new(100,0,40),End=new(0,0,40)}).All(e=>!e.OneWay),"A 40 m divided street escaped strict consolidation");
        Check(pair.Count>0&&pair.All(e=>e.ConsolidatedAvenue&&!e.OneWay&&e.Lanes==4&&Math.Abs(e.Start.z-5)<.001&&Math.Abs(e.End.z-5)<.001),"Opposing carriageways did not become a four-lane centerline avenue");
        Check(math.distance(pair[0].Start.xz,new float2(0,5))<.001&&math.distance(pair[pair.Count-1].End.xz,new float2(100,5))<.001,"Avenue discarded corridor endpoints");
        Check(Run(b,a).Select(e=>(e.WayId,e.StartNodeId,e.EndNodeId,e.Start,e.End,e.Lanes,e.OneWay)).SequenceEqual(pair.Select(e=>(e.WayId,e.StartNodeId,e.EndNodeId,e.Start,e.End,e.Lanes,e.OneWay))),"Input ordering changes consolidation");
        Check(Run(a,b with{Name="Different"}).All(e=>!e.OneWay),"Opposing geometric counterparts remained split because their names differ");
        Check(Run(a,b with{Start=a.End,End=a.Start}).All(e=>!e.OneWay),"Coincident opposing streets did not become one two-way road");
        Check(Run(a with{Reference="R1"},b with{Name="",Reference="R1"}).All(e=>!e.OneWay),"A missing name on one carriageway hid an otherwise identified divided street");
        Check(Run(a with{Reference="R1"},b with{Name="Different",Reference="R1"}).All(e=>!e.OneWay),"Naming prevented a geometrically supported avenue");
        var highway=S(400,400,1,0,-150,0,0) with{Classification="trunk",Name="Highway"};
        var anchored=Run(a,b,highway);
        Check(math.all(anchored.Single(e=>e.WayId==400).Start==highway.Start)&&math.all(anchored.Single(e=>e.WayId==400).End==highway.End),
            "Avenue midpoint placement moved an attached trunk road away from its surveyed alignment");
        Check(Run(a,b with{Start=b.End,End=b.Start,StartNodeId=4,EndNodeId=3}).Count==2,"Same-direction carriageways merged");
        Check(Run(a with{Classification="motorway"},b with{Classification="motorway"}).Count==2,"Highways converted into avenues");
        Check(Run(a with{Classification="trunk"},b with{Classification="trunk"}).Count==2,"Trunk highways converted into avenues");
        Check(Run(a with{Classification="primary_link"},b with{Classification="primary_link"}).Count==2,"Ramps converted into avenues");
        Check(Run(a with{Bridge=true},b with{Bridge=true}).All(e=>e.ConsolidatedAvenue&&e.Bridge&&e.Layer==0),"Same-structure avenue consolidation lost the elevated mode or source layer");
        Check(Run(a,b with{Layer=1}).Count==2,"Different elevations merged");
        var structural=Run(a,b,S(9,2,20,100,0,150,0) with{Bridge=true});var tie=structural.Single(e=>e.WayId==9);
        Check(tie.Bridge&&math.all(tie.End==new float3(150,0,0))&&structural.Any(e=>e.ConsolidatedAvenue&&
            (e.StartNodeId==tie.StartNodeId&&math.all(e.Start==tie.Start)||e.EndNodeId==tie.StartNodeId&&math.all(e.End==tie.Start))),
            "Avenue consolidation must preserve the actual bridge tie-in and exact shared transition rather than banning the entire corridor");
        var differentLayer=Run(a,b,S(9,2,20,100,0,150,0) with{Bridge=true,Layer=1});var bridgePort=differentLayer.Single(e=>e.WayId==9);
        Check(bridgePort.Bridge&&bridgePort.Layer==1&&differentLayer.Any(e=>e.ConsolidatedAvenue&&
            (e.StartNodeId==bridgePort.StartNodeId&&math.all(e.Start==bridgePort.Start)||e.EndNodeId==bridgePort.StartNodeId&&math.all(e.End==bridgePort.Start))),
            "An explicit shared bridge portal lost its canonical source connection because its layer differs from the road");
        var acrossMedian=Run(a,b,S(9,2,3,100,0,100,10) with{Bridge=true,Layer=1});
        Check(acrossMedian.All(e=>e.StartNodeId!=e.EndNodeId)&&acrossMedian.Any(e=>e.WayId==9&&e.Bridge&&e.StartNodeId==2&&e.EndNodeId==3),
            "Avenue remapping collapsed a real structure into a self-loop or removed it");
        Check(Run(a,b,S(9,2,20,100,0,150,0) with{Kind=NetworkKind.Rail}).All(e=>!e.ConsolidatedAvenue),"Rail crossing remapped without a verified shared transition");
        Check(Run(a,b with{Start=new(100,0,50),End=new(0,0,50)}).All(e=>!e.ConsolidatedAvenue),"Widely separated roads merged");
        var ambiguous=S(3,10,11,0,20,100,20);
        Check(Run(a,b,ambiguous).Any(e=>e.ConsolidatedAvenue),"A nearby third route vetoed every ordinary-street pair");
        Check(Run(a,b with{Name=""},a with{WayId=3,StartNodeId=5,EndNodeId=6,Start=new(0,0,20),End=new(100,0,20)}).Any(e=>e.ConsolidatedAvenue),"Unnamed counterparts remained split in a three-route formation");
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
        CapturedBoiseStreet();
        var normalized=NetworkNormalizer.Normalize(connected,new ImportLog(),default,1000);
        NetworkNormalizer.Validate(normalized,1000,default);
        var terrain=Enumerable.Repeat((ushort)32768,TerrainBuilder.Resolution*TerrainBuilder.Resolution).ToArray();
        var aligned=NetworkProcessor.AlignToTerrain(normalized,terrain,100,1000,default);
        Check(NetworkVerticalProfiles.Prepare(aligned,terrain,100,1000,new ImportLog(),default).Any(e=>e.ConsolidatedAvenue),"Avenue cannot pass normal grading pipeline");
        using var cancel=new CancellationTokenSource();cancel.Cancel();
        try{AvenueConsolidator.Prepare(new[]{a,b},new ImportLog(),cancel.Token);throw new Exception("Cancellation ignored");}catch(OperationCanceledException){}
        Console.WriteLine("PASS: opposing one-ways to vanilla avenue centerlines, cross-way chains, junction/end reconnection, route reachability, unmatched/highway/ramp/structure/level guards, determinism and grading integration.");
    }

    static void StrictLocalMatches()
    {
        var source=new[]{S(900,1,101,0,0,50,0),S(900,101,2,50,0,100,0),
            S(901,3,102,100,12,50,12),S(901,102,4,50,12,0,12),
            S(902,5,101,50,-100,50,0) with{Kind=NetworkKind.Rail,Classification="rail",OneWay=false},
            S(902,101,103,50,0,50,5) with{Kind=NetworkKind.Rail,Classification="rail",OneWay=false},
            S(902,103,102,50,5,50,12) with{Kind=NetworkKind.Rail,Classification="rail",OneWay=false},
            S(902,102,6,50,12,50,100) with{Kind=NetworkKind.Rail,Classification="rail",OneWay=false}};
        var log=new ImportLog();var result=AvenueConsolidator.Prepare(source,log,default);
        Check(result.Where(s=>s.Kind==NetworkKind.Road).All(s=>!s.OneWay),"Straight track crossings left the street as parallel one-ways");
        Check(result.Where(s=>s.Kind==NetworkKind.Rail).All(s=>Math.Abs(s.Start.x-50)<.001&&Math.Abs(s.End.x-50)<.001&&s.End.z>s.Start.z),
            "Street consolidation bent or reversed the through railway");
        Check(NativeEndpointPolicy.LevelCrossings(result).Count==1&&result.All(s=>s.StartNodeId!=s.EndNodeId),
            "The consolidated avenue did not have one shared road/rail crossing");
        SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(source),result,log,default);
        Check(log.Metric("Missing mandatory selected-source ports")==0&&log.Metric("Unresolved selected-source reachability obligations")==0,
            "Consolidating a divided railway crossing lost a road or rail route");
        var road=new[]{S(910,10,11,0,0,80,0),S(910,11,12,80,0,100,60),
            S(911,13,14,100,70,80,10),S(911,14,15,80,10,0,10),
            S(912,11,16,80,0,80,-100) with{Name="Side",OneWay=false}};
        var locally=AvenueConsolidator.Prepare(road,new ImportLog(),default);
        Check(locally.Any(s=>!s.OneWay&&s.SourceWayIds.Contains(910)&&s.SourceWayIds.Contains(911)),
            "A bend vetoed every parallel local interval");
    }

    static void StrictJunctionCompletion()
    {
        var left=S(100,1,2,-120,0,0,0) with{OneWay=false,ConsolidatedAvenue=true,SourceWayIds=new long[]{10,20}};
        var right=S(200,3,4,8,0,120,0) with{OneWay=false,ConsolidatedAvenue=true,SourceWayIds=new long[]{10,20}};
        var tie=S(10,2,3,0,0,8,0);var source=new[]{left,right,tie};var log=new ImportLog();
        var result=AvenueJunctionRepair.Prepare(source,log,default,completeStreetJunctions:true);
        Check(result.Count==2&&result[0].EndNodeId==result[1].StartNodeId&&math.all(result[0].End==result[1].Start),
            "A short evidenced one-way tie remained between two consolidated avenues");
        SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(source),result,log,default);
        Check(log.Metric("Missing mandatory selected-source ports")==0&&log.Metric("Unresolved selected-source reachability obligations")==0,
            "Completing a street junction lost source reachability");
        var highway=S(300,5,2,0,-100,0,0) with{Classification="trunk"};
        Check(AvenueJunctionRepair.Prepare(source.Append(highway).ToArray(),new ImportLog(),default,completeStreetJunctions:true).Count==4,
            "Completing a street junction consumed an attached trunk-road port");
        var side=S(400,3,6,8,0,20,6) with{OneWay=false,Name="Side"};
        Check(AvenueJunctionRepair.Prepare(source.Append(side).ToArray(),new ImportLog(),default,completeStreetJunctions:true).Count==4,
            "Completing a street junction introduced an acute side-street opening");
    }

    static void FragmentedCounterparts()
    {
        var source=new List<PreparedSegment>{S(300,300,301,0,0,2400,0) with{Lanes=1}};
        for(var i=0;i<12;i++)source.Add(S(301+i,400+2*i,401+2*i,150+200*i,12,20+200*i,12) with{Lanes=1});
        var log=new ImportLog();var result=AvenueConsolidator.Prepare(source,log,default);
        Check(Enumerable.Range(301,12).All(way=>!result.Any(s=>s.OneWay&&s.WayId==way)),
            "A long street retained parallel opposing fragments or tiny projected tails");
        SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(source),result,log,default);
        Check(log.Metric("Missing mandatory selected-source ports")==0&&log.Metric("Unresolved selected-source reachability obligations")==0,
            "Consolidating fragmented opposing streets lost an exterior route or source port");
        Check(result.Where(s=>!s.OneWay).All(s=>s.Lanes==2&&!s.ConsolidatedAvenue),"Fragmented 1+1 lanes did not become regular two-lane roads");
    }

    static void DividedCrossing()
    {
        var box=new[]{S(600,600,601,-8,8,8,8),S(601,601,602,8,8,8,-8),
            S(602,602,603,8,-8,-8,-8),S(603,603,600,-8,-8,-8,8)};
        var arms=new[]{S(610,610,600,-300,8,-8,8),S(611,603,611,-8,-8,-300,-8),
            S(612,601,612,8,8,300,8),S(613,613,602,300,-8,8,-8),
            S(614,600,614,-8,8,-8,300) with{Name="Cross"},S(615,615,601,8,300,8,8) with{Name="Cross"},
            S(616,602,616,8,-8,8,-300) with{Name="Cross"},S(617,617,603,-8,-300,-8,-8) with{Name="Cross"}}
            .Select(s=>s with{Lanes=3}).ToArray();
        var source=box.Concat(arms).ToArray();var log=new ImportLog();var result=AvenueConsolidator.Prepare(source,log,default);
        Check(log.Metric("Divided street intersection boxes consolidated")==1&&result.All(s=>!s.OneWay&&s.Lanes==6),
            "A surface crossing retained split three-lane approaches instead of continuous six-lane avenues");
        var at=result.SelectMany(s=>new[]{(s.StartNodeId,s.Start),(s.EndNodeId,s.End)}).Where(p=>math.length(p.Item2.xz)<1).ToArray();
        Check(at.Length==4&&at.Select(p=>p.Item1).Distinct().Count()==1,"The four approaches do not share one central crossing");
        SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(source),result,log,default);
        Check(log.Metric("Missing mandatory selected-source ports")==0&&log.Metric("Unresolved selected-source reachability obligations")==0,"Street crossing consolidation lost source connections");
        var protectedSource=source.Append(S(620,600,620,-8,8,-100,100) with{Classification="trunk"}).ToArray();
        Check(StreetIntersectionConsolidator.Prepare(protectedSource,new ImportLog(),default).SequenceEqual(protectedSource),"A street crossing moved an attached highway");
        var separate=source.Select(s=>s.WayId==613?s with{Name="Different"}:s).ToArray();
        Check(StreetIntersectionConsolidator.Prepare(separate,new ImportLog(),default).SequenceEqual(separate),"Separate named streets were treated as a divided crossing");
    }

    static void TaperAttachmentGuard()
    {
        var body=new[]{S(500,500,501,0,0,50,0),S(500,501,502,50,0,100,0),
            S(501,503,504,100,12,90,6),S(501,504,505,90,6,0,10)};
        Check(AvenueConsolidator.Prepare(body,new ImportLog(),default).Any(s=>s.ConsolidatedAvenue),
            "An identified divided street's short end taper prevented consolidation");
        var arm=S(502,501,506,50,0,70,11) with{Name="Side",OneWay=false};var log=new ImportLog();
        var result=AvenueConsolidator.Prepare(body.Append(arm).ToArray(),log,default);
        var side=result.Single(s=>s.WayId==502);var heading=(side.End-side.Start).xz;
        var opening=result.Where(s=>s.WayId!=502&&(s.StartNodeId==side.StartNodeId||s.EndNodeId==side.StartNodeId))
            .Select(s=>Math.Acos(math.clamp(math.dot(math.normalizesafe(heading),math.normalizesafe((s.StartNodeId==side.StartNodeId?s.End-s.Start:s.Start-s.End).xz)),-1,1))*180/Math.PI).Min();
        if(opening<25)
        {
            foreach(var s in result)Console.WriteLine($"TAPER {s.WayId} {s.StartNodeId}->{s.EndNodeId} {s.Start.x},{s.Start.z} -> {s.End.x},{s.End.z}");
            foreach(var line in log.Lines.Where(l=>l.StartsWith("AVENUE MERGE",StringComparison.Ordinal)))Console.WriteLine(line);
        }
        Check(opening>=25&&log.Metric("Avenue taper matches retained for acute attachments")>0,
            $"A relaxed end taper introduced an acute side-street attachment ({opening:0.##} degrees)");
        var trunk=S(503,502,503,100,0,100,12) with{Classification="trunk",Name="Major"};
        var tied=AvenueConsolidator.Prepare(body.Append(trunk).ToArray(),new ImportLog(),default);
        Check(tied.Any(s=>s.ConsolidatedAvenue)&&tied.Single(s=>s.WayId==503)==trunk,
            "Consolidation failed to retain short turning approaches behind distinct trunk-road ports");
        var obligations=new ImportLog();SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(body.Append(trunk).ToArray()),tied,obligations,default);
        Check(obligations.Metric("Missing mandatory selected-source ports")==0&&obligations.Metric("Unresolved selected-source reachability obligations")==0,
            "Keeping separate trunk turning ports lost a selected-source route");
        var ring=new[]{S(504,502,507,100,0,106,6),S(504,507,503,106,6,100,12),
            S(504,503,508,100,12,94,6),S(504,508,502,94,6,100,0)}.Select(s=>s with{Name="",Roundabout=true}).ToArray();
        var roundabout=AvenueConsolidator.Prepare(body.Concat(ring).ToArray(),new ImportLog(),default);
        Check(roundabout.Where(s=>s.Roundabout).SequenceEqual(ring),
            "Avenue consolidation distorted a roundabout's ring");
        var competition=new[]{S(510,510,511,0,0,300,0),S(511,512,513,300,10,0,10),
            S(512,514,515,300,8,285,2),S(512,515,516,285,2,0,2)};
        var selected=AvenueConsolidator.Prepare(competition,new ImportLog(),default);
        Check(selected.Any(s=>s.ConsolidatedAvenue&&s.SourceWayIds.Contains(510)&&s.SourceWayIds.Contains(511))&&
            !selected.Any(s=>s.ConsolidatedAvenue&&s.SourceWayIds.Contains(512)),
            "A closer end-taper candidate displaced an established fully parallel counterpart");
    }

    static void CapturedBoiseStreet()
    {
        // Northside Road and its incident arms, from the complete Boise source
        // capture. Coalesced stations must still carry the short source way
        // 1572423297 between both of its mapped junction ports.
        var source=File.ReadLines(Path.Combine(AppContext.BaseDirectory,"fixtures","avenue-boise-northside.ndjson")).Select(line=>
        {
            var r=Newtonsoft.Json.Linq.JObject.Parse(line);
            float3 P(string key){var p=r[key]!.Values<float>().ToArray();return new(p[0],p[1],p[2]);}
            return new PreparedSegment((long)r["WayId"]!,(long)r["StartNodeId"]!,(long)r["EndNodeId"]!,P("start"),P("end"),
                NetworkKind.Road,(string)r["Classification"]!,(int)r["Lanes"]!,(bool)r["OneWay"]!,(bool)r["Bridge"]!,(bool)r["Tunnel"]!,(int)r["Layer"]!)
                {Name=(string)r["Name"]!,Reference=(string)r["Reference"]!,SourceWayIds=Array.AsReadOnly(r["SourceWayIds"]!.Select(value=>(long)value).ToArray()),SourceGeometry=new[]{P("start"),P("end")}};
        }).ToArray();
        var log=new ImportLog();var result=AvenueConsolidator.Prepare(source,log,default);
        Check(log.Metric("Opposing road pairs consolidated into avenues")>0,"Captured Boise street did not consolidate");
        SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(source),result,log,default);
        SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(result),result,log,default);
        Check(log.Metric("Missing mandatory selected-source ports")==0&&log.Metric("Unresolved selected-source reachability obligations")==0,
            "Captured Boise avenue lost an attached street or a short source way's directional route");
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
