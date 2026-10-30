using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

// Shapes that built as disconnected or unusable pieces in game: two rail copies
// between one node pair, sub-lane unit pieces beside a node, and continuity
// counting merged interior nodes as losses.
static class ConnectivityTests
{
    static void Check(bool value,string message){if(!value)throw new Exception(message);}
    static PreparedSegment Rail(long way,long a,long b,float3 p,float3 q)=>
        new(way,a,b,p,q,NetworkKind.Rail,"rail",2,false,false,false,0){NativeWidth=11,NativeAssetKey="rail",PhysicalTracks=2,SourceGeometry=new[]{p,q},OriginalGeometry=new[]{p,q}};
    static PreparedSegment Road(long way,long a,long b,float3 p,float3 q)=>
        new(way,a,b,p,q,NetworkKind.Road,"trunk",2,false,false,false,0){NativeWidth=12,NativeAssetKey="road",SourceGeometry=new[]{p,q},OriginalGeometry=new[]{p,q}};

    public static void Run()
    {
        ReservedRailIdentity();
        OverlappingHighwayExtensions();
        ReservedHighwayIdentity();
        var ground=Rail(1,10,11,new(0,50,0),new(40,50,0));
        var bridge=Rail(2,11,10,new(40,50,.5f),new(0,50,.5f)) with{Bridge=true};
        var loop=Rail(3,10,11,new(0,50,0),new(40,50,30)) with{Control1=new float3(10,50,30),Control2=new float3(30,50,30)};
        var onward=Rail(4,11,12,new(40,50,0),new(90,50,0));
        var kept=RailCorridorConsolidator.RedundantParallelSpans(new[]{ground,bridge,loop,onward},new ImportLog(),default);
        Check(kept.Count==3&&kept.Count(s=>s.WayId==3)==1,"Rail copies between one node pair were not reduced to one, or a separate loop was dropped");
        var survivor=kept.Single(s=>s.WayId is 1 or 2);
        Check(survivor.Bridge&&survivor.SourceWayIds.Contains(1)&&survivor.SourceWayIds.Contains(2),"The kept copy lost its structure or the dropped copy's provenance");
        Check(survivor.SourceNodeAliases.Any(a=>a.SourceNodeId==10&&a.CanonicalNodeId==10),"A junction made only by the copy was not kept as a port");

        // Whole routes through degree-two nodes, not just single sections. An
        // alias aimed at the dropped route's interior follows it to an end; a
        // route crossing a road at grade is never dropped.
        var spur=Rail(9,40,44,new(0,50,0),new(-60,50,0));var tail=Rail(10,43,45,new(100,50,0),new(160,50,0));
        var routeA=new[]{Rail(5,40,41,new(0,50,0),new(50,50,0)),Rail(6,41,43,new(50,50,0),new(100,50,0))};
        var routeB=new[]{Rail(7,40,42,new(0,50,.6f),new(45,50,.6f)),Rail(8,42,43,new(45,50,.6f),new(100,50,.6f)) with{SourceNodeAliases=new[]{new SourceNodeAlias(99,42,true)}}};
        var routes=RailCorridorConsolidator.RedundantParallelSpans(routeA.Concat(routeB).Append(spur).Append(tail).ToArray(),new ImportLog(),default);
        Check(routes.Count==4&&routes.All(s=>s.WayId is not (7 or 8)),"A duplicate two-section rail route between one junction pair survived");
        var routeAliases=routes.SelectMany(s=>s.SourceNodeAliases).ToArray();
        Check(routeAliases.Any(a=>a.SourceNodeId==99&&a.CanonicalNodeId is 40 or 43)&&routeAliases.Any(a=>a.SourceNodeId==42&&a.CanonicalNodeId==40),
            "The dropped route's interior node or an alias aimed at it was not carried to an end");
        Check(routes.Where(s=>s.WayId is 5 or 6).All(s=>s.SourceWayIds.Contains(7)&&s.SourceWayIds.Contains(8)),"The kept route did not carry the dropped route's ways");
        var crossing=Road(30,42,46,new(45,50,-20),new(45,50,20));
        var crossed=RailCorridorConsolidator.RedundantParallelSpans(routeA.Concat(routeB).Append(spur).Append(tail).Append(crossing).ToArray(),new ImportLog(),default);
        Check(crossed.Count(s=>s.Kind==NetworkKind.Rail)==6,"A rail route with a level crossing was dropped");

        // A connector lying inside another line's footprint goes when its ends
        // stay connected nearby; its way rides on that path. With a level
        // crossing on it, it stays.
        PreparedSegment Narrow(PreparedSegment s)=>s with{NativeWidth=4};
        var stub=Narrow(Rail(70,79,71,new(-100,50,0),new(0,50,0)));var main=Narrow(Rail(71,71,72,new(0,50,0),new(200,50,0)));
        var link=Narrow(Rail(72,72,75,new(200,50,0),new(300,50,6)));var lineIn=Narrow(Rail(73,76,74,new(-100,50,6),new(150,50,6)));
        var lineOut=Narrow(Rail(74,74,75,new(150,50,6),new(300,50,6)));var tailOut=Narrow(Rail(76,75,77,new(300,50,6),new(400,50,6)));
        var connector=Narrow(Rail(75,71,78,new(0,50,0),new(75,50,3)));var connectorEnd=Narrow(Rail(77,78,74,new(75,50,3),new(150,50,6)));
        var yard=new[]{stub,main,link,lineIn,lineOut,tailOut,connector,connectorEnd};
        var cleared=RailCorridorConsolidator.RedundantOverlappingRoutes(yard,new ImportLog(),default,35);
        Check(cleared.All(s=>s.WayId is not (75 or 77))&&cleared.Count==6,"A connector inside another line's footprint was not removed");
        Check(cleared.Single(s=>s.WayId==71).SourceWayIds.Contains(75)&&cleared.Single(s=>s.WayId==74).SourceWayIds.Contains(77),"The removed connector's ways did not move onto the alternative path");
        Check(cleared.SelectMany(s=>s.SourceNodeAliases).Any(a=>a.SourceNodeId==78&&a.CanonicalNodeId is 71 or 74),"The connector's interior node was not aliased to an end");
        var guarded=RailCorridorConsolidator.RedundantOverlappingRoutes(yard.Append(Road(80,78,90,new(75,50,3),new(75,50,40))).ToArray(),new ImportLog(),default,35);
        Check(guarded.Count(s=>s.Kind==NetworkKind.Rail)==8,"A connector with a level crossing was removed");

        // Wherever a junction core ends along the arm, no unit cut may leave a
        // piece shorter than a lane beside an existing node.
        foreach(var minimum in new Func<PreparedSegment,double>?[]{null,_=>20})
            for(var first=14f;first<=40;first+=.1f)
            {
                var arms=new[]{Road(10,1,2,new(0,0,0),new(first,0,0)),Road(11,2,3,new(first,0,0),new(400,0,0)),
                    Road(12,1,4,new(0,0,0),new(-300,0,0)),Road(13,1,5,new(0,0,0),new(0,0,300))};
                var units=NativeDesignUnits.Prepare(arms,Array.Empty<NativeAssetContract>(),new ImportLog(),default,minimum);
                var shortest=units.Where(s=>s.StartNodeId<long.MinValue+13000000||s.EndNodeId<long.MinValue+13000000)
                    .Select(s=>math.distance(s.CourseStart.xz,s.CourseEnd.xz)).DefaultIfEmpty(float.MaxValue).Min();
                Check(shortest>=3-1e-3,$"A native unit cut left a {shortest:F2} m piece beside a node (first span {first:F1} m)");
                var junction=units.Where(s=>s.StartNodeId==1||s.EndNodeId==1).Select(s=>s.NativeUnitId).Distinct().Count();
                Check(junction==1,"A junction's arms were split across native units");
            }

        // A merged interior node is not a loss while its way survives; a way
        // with no surviving section still is.
        var a=Road(20,30,31,new(0,0,0),new(50,0,0));var b=Road(21,31,32,new(50,0,0),new(100,0,0));
        var merged=a with{EndNodeId=32,End=new float3(100,0,0),SourceWayIds=new long[]{20,21}};
        var log=new ImportLog();NetworkContinuity.Validate(new[]{a,b},new[]{merged},log,default,allowQualityWarnings:true);
        Check(log.Metric("Network continuity unresolved sections")==0,"Continuity counted a merged interior node as a lost section");
        log=new ImportLog();NetworkContinuity.Validate(new[]{a,b},new[]{a},log,default,allowQualityWarnings:true);
        Check(log.Metric("Network continuity unresolved sections")==1,"Continuity missed a section whose way disappeared");

        // A second track whose ends were merged onto the first leaves a lens: a
        // detour turning 90° between the same two nodes. It goes, whatever its overlap.
        var straight=Narrow(Rail(90,91,92,new(0,50,0),new(50,50,0)));
        var detour=new[]{Narrow(Rail(91,91,93,new(0,50,0),new(25,50,22))),Narrow(Rail(92,93,92,new(25,50,22),new(50,50,0)))};
        var beyond=new[]{Narrow(Rail(93,94,91,new(-100,50,0),new(0,50,0))),Narrow(Rail(94,92,95,new(50,50,0),new(150,50,0)))};
        var lens=RailCorridorConsolidator.RedundantOverlappingRoutes(beyond.Append(straight).Concat(detour).ToArray(),new ImportLog(),default,35);
        Check(lens.Count==3&&lens.All(s=>s.WayId is not (91 or 92))&&lens.Single(s=>s.WayId==90).SourceWayIds.Contains(91),"A 90° rail lens beside a straight track survived or lost its ways");

        // A 15 m spur from a junction to nowhere is a consolidated crossover's
        // remnant and goes, even where the through track bends; a 60 m spur stays.
        var through=new[]{Narrow(Rail(200,201,202,new(0,50,0),new(100,50,0))),Narrow(Rail(201,202,203,new(100,50,0),new(200,50,8))),Narrow(Rail(203,203,205,new(200,50,8),new(300,50,8)))};
        var shortSpur=Narrow(Rail(202,202,204,new(100,50,0),new(114,50,-5)));var longSpur=Narrow(Rail(204,203,206,new(200,50,8),new(256,50,28)));
        var spurs=RailCorridorConsolidator.RedundantOverlappingRoutes(through.Append(shortSpur).Append(longSpur).ToArray(),new ImportLog(),default,2);
        Check(spurs.All(s=>s.WayId!=202)&&spurs.Any(s=>s.WayId==204)&&spurs.Count==4,"A 15 m rail stub was kept, or a 60 m spur was removed");
        Check(spurs.SelectMany(s=>s.SourceNodeAliases).Any(a=>a.SourceNodeId==204&&a.CanonicalNodeId==202)&&spurs.Any(s=>s.SourceWayIds.Contains(202)),"The removed stub's dead end or way was not carried to its junction");

        // A road over a short culvert becomes a bridge across it, split out of
        // the section; a road beside it is untouched.
        var culvert=new[]{(new[]{new float3(100,0,-10),new float3(100,0,10)},3f)};
        var over=Road(100,101,102,new(0,20,0),new(300,20,0));var beside=Road(101,103,104,new(0,20,40),new(300,20,40));
        var bridged=CoordinatedNetworkDesign.CulvertBridges(new[]{over,beside},culvert,4097,57344,new ImportLog(),default);
        var deck=bridged.Where(s=>s.WayId==100&&s.Bridge).ToArray();
        Check(bridged.Count(s=>s.WayId==100)==3&&deck.Length==1&&bridged.Single(s=>s.WayId==101)==beside,"A road over a culvert was not split around one bridge, or a road beside it changed");
        // Half the road (6 m) plus the channel (3 m) and two 14 m raster cells.
        Check(Math.Abs(deck[0].CourseStart.x-63)<.5&&Math.Abs(deck[0].CourseEnd.x-137)<.5,$"Culvert bridge spans {deck[0].CourseStart.x:F1}–{deck[0].CourseEnd.x:F1} m, not 63–137 m");
        Check(bridged.Where(s=>s.WayId==100).SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}).GroupBy(id=>id).Count(g=>g.Count()==2)==2,"Culvert bridge pieces do not share their split nodes");
        // A long open stream, one piece spanning many index cells, is found the same way.
        var stream=new[]{(new[]{new float3(100,0,-1000),new float3(100,0,1000),new float3(900,0,1000)},3f)};
        var spanned=CoordinatedNetworkDesign.CulvertBridges(new[]{over,beside},stream,4097,57344,new ImportLog(),default);
        Check(spanned.Count(s=>s.WayId==100&&s.Bridge)==1&&spanned.Count(s=>s.WayId==101&&s.Bridge)==1,"A road crossing a long open stream was not bridged once");
        Console.WriteLine("PASS: one rail course per node pair, overlapping rail connectors and lenses removed with provenance, no sub-lane unit pieces beside nodes, alias-aware continuity, and roads bridged over short culverts.");
    }

    static void OverlappingHighwayExtensions()
    {
        PreparedSegment Highway(long way,long a,long b,float startX,float endX)=>Road(way,a,b,new(startX,50,0),new(endX,50,0))
            with{Classification="motorway",OneWay=true,Reference="M1"};
        var roads=new[]{Highway(1000,1000,1001,0,1000),Highway(1001,1002,1003,800,1800),Highway(1002,1004,1005,1200,2000),
            Road(1003,1006,1007,new(2000,50,20),new(0,50,20)) with{Classification="motorway",OneWay=true,Reference="M1"},
            Road(1004,1004,1008,new(1200,50,0),new(1200,50,-100)),
            Road(1005,1009,1010,new(0,50,4),new(2000,50,4)) with{Classification="motorway",OneWay=true,Reference="M1"}};
        foreach(var input in new[]{roads,roads.Reverse().ToArray()})
        {
            var output=NetworkOverlapConsolidator.Prepare(input,new ImportLog(),default);
            var forward=output.Where(s=>s.Kind==NetworkKind.Road&&s.OneWay&&s.End.x>s.Start.x&&Math.Abs(s.Start.z)<.001).OrderBy(s=>s.Start.x).ToArray();
            Check(Math.Abs(forward[0].Start.x)<.01&&Math.Abs(forward[^1].End.x-2000)<.01&&Math.Abs(forward.Sum(s=>s.End.x-s.Start.x)-2000)<.01,
                "A retained extension still overlaps another motorway or lost its unique length");
            Check(forward.Zip(forward.Skip(1)).All(p=>p.First.EndNodeId==p.Second.StartNodeId&&math.distance(p.First.End,p.Second.Start)<.001),
                "Closing duplicate motorway extensions broke their shared junctions");
            Check(forward.Single(s=>s.Start.x<1500&&s.End.x>1500).SourceWayIds.OrderBy(w=>w).SequenceEqual(new long[]{1001,1002}),
                "The shared motorway interval lost one source way or inherited unrelated provenance");
            var again=NetworkOverlapConsolidator.Prepare(output,new ImportLog(),default);
            Check(again.Count==output.Count&&again.Zip(output).All(p=>p.First.StartNodeId==p.Second.StartNodeId&&p.First.EndNodeId==p.Second.EndNodeId)&&
                Math.Abs(again.Sum(s=>math.distance(s.Start.xz,s.End.xz))-output.Sum(s=>math.distance(s.Start.xz,s.End.xz)))<.01,
                "Highway consolidation left more coincident intervals for the next import stage");
            Check(output.Count(s=>s.WayId==1003)==1,"Consolidation removed a separate opposing motorway");
            Check(output.Count(s=>s.WayId==1005)==1,"Consolidation removed a separate parallel motorway");
            var audit=new ImportLog();SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(input),output,audit,default);
            Check(audit.Metric("Missing mandatory selected-source ports")==0&&audit.Metric("Unresolved selected-source reachability obligations")==0,
                "Closing duplicate motorway extensions lost a source route or incident street");
        }
    }

    static void ReservedHighwayIdentity()
    {
        const long retired=long.MinValue+7000000;
        var a=Road(1100,1100,1101,new(0,50,0),new(1000,50,0)) with{Classification="motorway",OneWay=true,Reference="M2",
            SourceNodeAliases=new[]{new SourceNodeAlias(retired,1100,true)}};
        var b=Road(1101,1102,1103,new(800,50,0),new(1800,50,0)) with{Classification="motorway",OneWay=true,Reference="M2"};
        var output=NetworkOverlapConsolidator.Prepare(new[]{a,b},new ImportLog(),default);
        Check(output.All(s=>s.StartNodeId!=retired&&s.EndNodeId!=retired),"A new motorway junction reused a retired source alias identity");
        Check(output.SelectMany(s=>s.SourceNodeAliases).Where(n=>n.SourceNodeId==retired).All(n=>n.CanonicalNodeId==1100),
            "A new motorway junction redirected an unrelated retained source alias");
    }

    static void ReservedRailIdentity()
    {
        const long retired=-7_800_000_000_000_000_000;
        var axis=Rail(900,900,901,new(0,50,0),new(200,50,0)) with{SourceNodeAliases=new[]{new SourceNodeAlias(retired,900,true)}};
        var parallel=Rail(901,902,903,new(50,50,8),new(150,50,8));
        var branch=Rail(902,903,904,new(150,50,8),new(150,50,100));
        var output=RailCorridorConsolidator.Prepare(new[]{axis,parallel,branch},new ImportLog(),default);
        Check(output.Any(s=>s.StartNodeId<0||s.EndNodeId<0),"Rail identity fixture did not transfer an interior junction");
        Check(output.All(s=>s.StartNodeId!=retired&&s.EndNodeId!=retired),"A new rail junction reused a retired source alias identity");
        Check(output.SelectMany(s=>s.SourceNodeAliases).Where(a=>a.SourceNodeId==retired).All(a=>a.CanonicalNodeId==900),
            "Rail junction generation redirected an unrelated retained source alias");
    }
}
