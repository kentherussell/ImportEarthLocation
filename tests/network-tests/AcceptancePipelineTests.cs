using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

static class AcceptancePipelineTests
{
    static void Check(bool value,string message){if(!value)throw new Exception(message);}
    static PreparedSegment Line(long way,long a,long b,float3 p,float3 q)=>new(way,a,b,p,q,NetworkKind.Road,"motorway",2,true,false,false,0)
        {NativeWidth=16,NativeAssetKey="fixture",SourceWayIds=new[]{way},NativeLaneOffsets=new[]{-2f,2f}};
    public static void Run()
    {
        var a=Line(1,1,2,new(-100,100,0),new(0,100,0));var b=Line(1,2,3,a.End,new(100,100,0));
        var branch=Line(2,2,4,a.End,new(100,100,30));var initial=new[]{a,b,branch};
        var topology=SourceTopologyObligations.Capture(initial);
        var mapped=initial.Select(s=>s with{StartNodeId=s.StartNodeId==2?20:s.StartNodeId,EndNodeId=s.EndNodeId==2?20:s.EndNodeId}).ToArray();
        var aliased=SourceTopologyObligations.Transfer(initial,mapped,NetworkKind.Road,new Dictionary<long,long>{{2,20}});
        Check(aliased.All(s=>s.SourceNodeAliases.Any(alias=>alias.SourceNodeId==2&&alias.CanonicalNodeId==20&&alias.Mandatory)),"Consolidation lost an explicit mandatory source alias");
        var log=new ImportLog();SourceTopologyObligations.Validate(topology,aliased,log,default);
        Check(log.Metric("Missing mandatory selected-source ports")==0&&log.Metric("Unresolved selected-source reachability obligations")==0,"Exact alias transfer broke the directional source graph");
        var missing=new ImportLog();SourceTopologyObligations.Validate(topology,aliased.Take(2).ToArray(),missing,default);
        Check(missing.Metric("Unresolved selected-source reachability obligations")>0,"Lost mandatory branch was hidden by a different reachable road");
        var aliasJoin=aliased.Take(2).ToArray();
        Check(NativeChainCompactor.Prepare(aliasJoin,new ImportLog(),default).Count==2,"Compaction erased a transferred mandatory source junction");

        var envelope=new NativeCompositionEnvelope(NativeStructureMode.Ground,0,16,0,1,-2,3,4,true);
        var contract=new NativeAssetContract("fixture",new[]{envelope,envelope with{Mode=NativeStructureMode.Elevated}},Array.Empty<float>());
        var shortRoad=Line(3,30,31,new(0,100,0),new(60,100,0));
        var units=NativeDesignUnits.Prepare(new[]{shortRoad},new[]{contract},new ImportLog(),default);
        Check(units.Count==2&&units.Select(s=>s.NativeUnitId).Distinct().Count()==2&&
            units[0].EndNodeId==units[1].StartNodeId&&units[0].StartNodeId==shortRoad.StartNodeId&&units[1].EndNodeId==shortRoad.EndNodeId&&
            math.distance(units[0].CourseEnd,shortRoad.CourseStart)>envelope.NodeOffset&&
            math.distance(units[1].CourseStart,shortRoad.CourseEnd)>envelope.NodeOffset,
            "A single fitted span lost separate native core ownership or used redundant corridor cuts");
        var tooShort=NativeDesignUnits.Prepare(new[]{shortRoad with{End=new(10,100,0)}},new[]{contract},new ImportLog(),default);
        Check(tooShort.Select(s=>s.NativeUnitId).Distinct().Count()==1,"Genuinely overlapping central native formations were split");
        var paired=new[]{Line(4,40,41,new(10000,100,0),new(10500,100,0)) with{PairedProfileId=10,PairedReferenceEnd=1},
            Line(5,50,51,new(10500,100,30),new(10000,100,30)) with{PairedProfileId=10,PairedReferenceBegin=1,PairedReferenceEnd=0,NativeWidth=16.001f}};
        var cuts=NativeDesignUnits.Prepare(paired,Array.Empty<NativeAssetContract>(),new ImportLog(),default);
        Check(cuts.All(s=>math.distance(s.CourseStart.xz,s.CourseEnd.xz)>.049&&s.CourseStart.y==100&&s.CourseControl1.y==100&&s.CourseControl2.y==100&&s.CourseEnd.y==100),
            "Near-coincident synchronized optional cuts emitted a collapsed/sliver or changed the frozen profile");
        var broken=a with{Control1=a.Start};var regular=NetworkHorizontalRegularity.Prepare(new[]{broken},new ImportLog(),default)[0];
        Check(regular.StartNodeId==a.StartNodeId&&regular.EndNodeId==a.EndNodeId&&math.distance(regular.CourseStart.xz,regular.CourseControl1.xz)>1,
            "Stationary endpoint regularization removed topology or left a free vertical handle");
        var terrain=Enumerable.Repeat((ushort)32768,17*17).ToArray();
        var offset=a with{StartCurveOffset=new(0,7,0),EndCurveOffset=new(0,5,0)};
        var solved=NetworkTunnelProfile.Solve(new[]{offset},200,Array.Empty<(int,float,float)>(),new ImportLog(),default,design:new(terrain,2000))[0];
        Check(math.distance(solved.Start+solved.StartCurveOffset,solved.CourseStart)<1e-5&&math.distance(solved.End+solved.EndCurveOffset,solved.CourseEnd)<1e-5,
            "Profile publication applied a vertical deck offset twice");
        var collapsed=a with{End=a.Start,Control1=a.Start+new float3(0,100,0),Control2=a.Start-new float3(0,50,0)};
        var pointProfile=NetworkTunnelProfile.Solve(new[]{collapsed},200,Array.Empty<(int,float,float)>(),new ImportLog(),default,design:new(terrain,2000))[0];
        Check(pointProfile.CourseControl1.y==pointProfile.CourseStart.y&&pointProfile.CourseControl2.y==pointProfile.CourseEnd.y&&
            !NetworkDesignChecks.HasHorizontalExtent(new(pointProfile.CourseStart,pointProfile.CourseControl1,pointProfile.CourseControl2,pointProfile.CourseEnd)),
            "A collapsed horizontal span retained independent vertical handles or became a representable native road");

        var turn=b with{End=new(0,100,100)};
        var relation=new OsmRelation(100,Array.Empty<(long,bool)>(),new Dictionary<string,string>{{"type","connectivity"},{"connectivity","1:2|2:1"}})
            {OrderedMembers=new[]{new OsmMember("way",a.WayId,"from"),new OsmMember("node",2,"via"),new OsmMember("way",turn.WayId,"to")}};
        var movement=NativeLaneObligations.Build(new[]{a,turn},new[]{relation});
        Check(movement.Movements.Count(m=>m.SourceRelation==100)==2&&movement.Movements.Where(m=>m.SourceRelation==100).All(m=>m.From.Rank!=m.To.Rank),
            "Explicit non-collinear connectivity relation lost its physical lane mapping");
        var malformed=relation with{Tags=new Dictionary<string,string>{{"type","connectivity"},{"connectivity","1:(2)"}}};
        Check(NativeLaneObligations.Build(new[]{a,turn},new[]{malformed}).Movements.All(m=>m.SourceRelation==null),"Unsupported conditional/priority grammar fabricated a source certificate");
        var assets=new Dictionary<string,NativeAssetContract>{{"fixture",contract}};
        var parallel=Line(6,60,61,new(0,100,8),new(100,100,8)) with{Bridge=true};
        var ground=Line(7,70,71,new(0,100,0),new(100,100,0));
        var topologyWitness=new NetworkContactTopology(new[]{ground,parallel},new[]{envelope,envelope with{Mode=NativeStructureMode.Elevated}},default);
        Check(double.IsPositiveInfinity(topologyWitness.RiseBudget(0,.5,1,.5,default)),"Unconnected footprint acquired a fabricated canonical connection");
        var transitionA=Line(8,80,81,new(-10,100,0),new(0,100,0));var transitionB=Line(9,81,82,new(0,100,0),new(10,100,0)) with{Bridge=true,Layer=1};
        var local=new NetworkContactTopology(new[]{transitionA,transitionB},new[]{envelope,envelope with{Mode=NativeStructureMode.Elevated}},default);
        var leaves=new List<FootprintLeaf>();NetworkFootprintMath.BuildLeaves(new(transitionA.CourseStart,transitionA.CourseControl1,transitionA.CourseControl2,transitionA.CourseEnd),8,0,leaves,default);
        NetworkFootprintMath.BuildLeaves(new(transitionB.CourseStart,transitionB.CourseControl1,transitionB.CourseControl2,transitionB.CourseEnd),8,1,leaves,default);
        Check(local.LocalTransition(leaves[0],leaves[1]),"Unique local bridge transition was treated as an independent obstacle");
        Check(local.RiseBudget(0,.9,1,.1,default)<1,"Virtual contact stations ignored partial arc lengths in the rise witness");
        var bankTerrain=Enumerable.Range(0,33*33).Select(i=>(ushort)(i/33<16?49152:32768)).ToArray();
        var quay=Line(200,2000,2001,new(-300,110,0),new(300,110,0)) with{PairedProfileId=200,PairedReferenceBegin=10,PairedReferenceEnd=20};
        quay=quay with{FinalDeckCurve=new(quay.CourseStart,quay.CourseControl1,quay.CourseControl2,quay.CourseEnd)};
        var quays=StarterNetworkPolicy.Quays(new[]{quay},bankTerrain,200,2000,new ImportLog(),default);
        Check(quays.Count==3&&quays.Count(s=>s.QuaySides!=NetworkSides.None)==1,"Quay setback fixture did not exercise exact mode subdivision");
        foreach(var child in quays)
        {
            Check(child.PairedReferenceEnd>child.PairedReferenceBegin&&child.PairedReferenceEnd-child.PairedReferenceBegin<10,
                "Quay child inherited the whole parent's paired-reference interval");
            for(var sample=0;sample<=8;sample++)
            {
                var t=sample/8d;var parentT=(math.lerp(child.PairedReferenceBegin,child.PairedReferenceEnd,t)-10)/10;
                var expected=NetworkDesignChecks.Evaluate(new(quay.CourseStart,quay.CourseControl1,quay.CourseControl2,quay.CourseEnd),parentT);
                var actual=NetworkDesignChecks.Evaluate(new(child.CourseStart,child.CourseControl1,child.CourseControl2,child.CourseEnd),t);
                Check(math.distance(expected,actual)<.0001,"Quay child reused the parent's frozen deck or reshaped its profile");
            }
        }
        var railA=a with{Kind=NetworkKind.Rail,OneWay=false,Start=new(0,100,0),End=new(100,100,10),
            Control1=new(100f/3,100,10f/3),Control2=new(200f/3,100,5)};
        var railB=b with{Kind=NetworkKind.Rail,OneWay=false,Start=railA.End,End=new(200,100,30),
            Control1=new(400f/3,100,15),Control2=new(500f/3,100,70f/3)};
        var approach=NetworkApproachTapers.Materialize(new[]{railA,railB},Array.Empty<NetworkApproachTapers.Shift>(),new ImportLog(),default,
            new[]{new NetworkApproachTapers.DirectionRequest(0,true,new(1,0))},includeLaneOffsets:false);
        var firstPort=approach.Single(s=>s.StartNodeId==railA.StartNodeId);
        Check(math.dot(math.normalizesafe((firstPort.CourseControl1-firstPort.CourseStart).xz),new float2(1,0))>1-1e-6,
            "Whole rail approach failed its exact canonical port direction");
        Check(approach.Single(s=>s.EndNodeId==railB.EndNodeId).End.Equals(railB.End),"Rail direction field moved the fixed far boundary");
        foreach(var node in approach.SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}).Distinct())
        {
            var ports=approach.Where(s=>s.StartNodeId==node||s.EndNodeId==node).ToArray();if(ports.Length!=2)continue;
            float2 Outward(PreparedSegment s)=>math.normalizesafe((s.StartNodeId==node?s.CourseControl1-s.CourseStart:s.CourseControl2-s.CourseEnd).xz);
            Check(math.dot(Outward(ports[0]),Outward(ports[1]))<-.99999,"Whole-approach emission introduced a new tangent kink");
        }
        Console.WriteLine("PASS: mandatory source aliases/reachability, protected transferred joins, actual-vs-heuristic cores, float-safe optional cuts, stationary handles, exact deck publication, explicit turn-slot relations and bounded local contact witnesses.");
    }
}
