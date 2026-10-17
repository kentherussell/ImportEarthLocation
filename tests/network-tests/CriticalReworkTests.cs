using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

static class CriticalReworkTests
{
    static void Check(bool condition,string message){if(!condition)throw new Exception(message);}
    static PreparedSegment Line(long way,long a,long b,float3 start,float3 end,NetworkKind kind=NetworkKind.Road)=>
        new(way,a,b,start,end,kind,kind==NetworkKind.Rail?"rail":"primary",2,kind==NetworkKind.Road,false,false,0)
        {SourceGeometry=new[]{start,end},NativeWidth=12,NativeAssetKey="fixture"};
    public static void Run()
    {
        var points=new[]{new float3(0,50,0),new float3(1,50,0),new float3(7,50,0),new float3(100,50,0)};
        Check(NetworkSourceFit.TryFit(points,points[0],points[3],new(1,0),new(1,0),2,default,out var straight,out _),"Bounded source fit rejected a nonuniformly sampled straight line");
        for(var i=0;i<=20;i++)Check(math.distance(NetworkDesignChecks.Evaluate(straight,i/20d),new double3(5*i,50,0))<1e-5,"Physical-support weighting distorted the analytical straight optimum");
        Check(!NetworkSourceFit.TryFit(new[]{points[0],new float3(50,50,100),points[3]},points[0],points[3],new(1,0),new(1,0),2,default,out _,out _),"A hidden hairpin was certified by endpoint agreement");
        var cuts=NetworkEmissionStations.Cuts(new(new(10000,0,0),new(10100,0,0),new(10200,0,0),new(10300,0,0)),new[]{0d,1e-6,.5,1-1e-6,1});
        Check(cuts.SequenceEqual(new[]{0d,.5,1}),"Optional A7-like sub-millimetre taper cuts survived float-grid emission");

        var railA=Line(2,10,11,new(0,50,0),new(200,50,10),NetworkKind.Rail) with{NativeCenterlineCurvatureLimit=.03};
        var railB=Line(3,11,12,railA.End,new(400,50,20),NetworkKind.Rail) with{NativeCenterlineCurvatureLimit=.03};
        var framed=NetworkApproachFrames.Prepare(new[]{railA,railB},new[]{new NetworkApproachTapers.DirectionRequest(0,true,new(1,0)),
            new NetworkApproachTapers.DirectionRequest(1,false,new(-1,0))},new ImportLog(),default);
        Check(math.dot(math.normalizesafe((framed[0].CourseControl1-framed[0].CourseStart).xz),new float2(1,0))>1-1e-6&&
            math.dot(math.normalizesafe((framed[1].CourseEnd-framed[1].CourseControl2).xz),new float2(1,0))>1-1e-6,"Joint rail frame did not satisfy both requested port directions");
        Check(math.all(framed[0].End==framed[1].Start)&&math.all(framed[0].Start==railA.Start)&&math.all(framed[1].End==railB.End),"Whole rail approach moved an outer boundary or split its canonical internal node");
        Check(framed.All(s=>NetworkDesignChecks.Certify(new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd),.03,double.PositiveInfinity,maximumLeaves:4096).Quality==DesignQuality.Certified),"Rail approach failed actual fixture curvature contract");

        var terrain=Enumerable.Repeat((ushort)16384,17*17).ToArray();
        var frozen=Line(4,20,21,new(-200,100,100),new(200,100,100));
        var free=Line(5,30,31,new(-200,100,-100),new(200,100,-100));
        var solved=NetworkTunnelProfile.Solve(new[]{frozen,free},200,Array.Empty<(int,float,float)>(),new ImportLog(),default,
            design:new(terrain,2000),frozenSections:new HashSet<int>{0});
        Check(solved[0].CourseStart.y==100&&solved[0].CourseControl1.y==100&&solved[0].CourseControl2.y==100&&solved[0].CourseEnd.y==100,
            "Local physical context reoptimized an unaffected neighbour");
        Check(solved[1].CourseStart.y<99,"Frozen context accidentally froze an unrelated free approach");

        var a=Line(6,40,41,new(-100,50,0),new(0,50,0));var b=Line(6,41,42,a.End,new(100,50,0));
        var badA=a with{End=new(0,80,0),Control1=new(-200f/3,50,0),Control2=new(-100f/3,80,0)};
        var badB=b with{Start=new(0,80,0),Control1=new(100f/3,80,0),Control2=new(200f/3,50,0)};
        var log=new ImportLog();var repaired=NetworkPhysicalReconstruction.Once(new[]{a,b},new[]{badA,badB},terrain,200,2000,null,null,log,default);
        Check(repaired.All(s=>NetworkVerticalProfiles.WithinGradeCeiling(s,NetworkGeometrySafety.GradeLimit(s))),"One-shot reconstruction did not remove a repairable interior profile spike");
        Check(math.all(repaired[0].Start==a.Start)&&math.all(repaired[1].End==b.End)&&badA.End.y==80,"Physical reconstruction moved outer anchors or mutated its immutable input");
        Check(log.Lines.Count(line=>line.StartsWith("PHYSICAL RECONSTRUCTION attempts=1"))==1,"Exceptional-grade fallback repeated its local optimizer");
        NetworkQualityAudit.Planned(new[]{badA,badB},log,default,false);
        Check(log.Findings.Any(f=>f.Category=="exceptional-grade"&&f.Failed==2&&f.AffectedLengthMeters>0),"Quality ledger hid a known bumpy profile or lost affected length");
        Check(log.Findings.Single(f=>f.Category=="source-topology").State=="Unavailable","Missing source evidence was called a passed topology check");
        var compacted=NativeChainCompactor.Prepare(new[]{a,b},new ImportLog(),default);
        Check(compacted.Count==1&&compacted[0].StartNodeId==40&&compacted[0].EndNodeId==42&&compacted[0].SourceWayIds.Contains(6),
            "Shared bounded compactor failed a straight pair or lost source identity");
        var reach=math.distance(compacted[0].CourseStart.xz,compacted[0].CourseControl1.xz);
        Check(reach>=10&&reach<=90&&compacted[0].HorizontalEmissionReference.HasValue,"Compaction bypassed shared handle bounds or lost emission evidence");
        var branch=Line(7,41,43,a.End,new(0,50,100));
        Check(NativeChainCompactor.Prepare(new[]{a,b,branch},new ImportLog(),default).Count==3,"Compaction removed a real three-arm junction");
        var origin=new double3(20000,50,20000);var axis=math.normalizesafe(new double3(1,0,.1234));
        var refA=new Cubic3d(origin-100*axis,origin-70*axis,origin-37*axis,origin);
        var refB=new Cubic3d(origin,origin+61*axis,origin+80*axis,origin+120*axis);
        PreparedSegment Emitted(Cubic3d curve,long start,long end)=>Line(8,start,end,(float3)curve.P0,(float3)curve.P3) with
            {Control1=(float3)curve.P1,Control2=(float3)curve.P2,HorizontalEmissionReference=curve};
        var emittedA=Emitted(refA,50,51);var emittedB=Emitted(refB,51,52);var joinLog=new ImportLog();
        NetworkQualityAudit.Planned(new[]{emittedA,emittedB},joinLog,default);
        Check(joinLog.Findings.Single(f=>f.Category=="continuation-heading").State=="Passed"&&
            joinLog.Findings.Single(f=>f.Category=="float-direction-representability").State=="Passed",
            "A common double direction within its measured float rounding budget failed the junction audit");
        Check(NetworkNormalizer.Reverse(emittedA).HorizontalEmissionReference!.Value.P0.Equals(refA.P3),"Reversing a source span left its emission reference backward");
        var pieces=NativeDesignUnits.Split(emittedA,.37,53);
        Check(pieces.Left.HorizontalEmissionReference.HasValue&&pieces.Right.HorizontalEmissionReference.HasValue&&
            pieces.Left.HorizontalEmissionReference.Value.P3.Equals(pieces.Right.HorizontalEmissionReference.Value.P0),"Exact native cuts discarded their common double emission port");
        var kink=emittedB with{Control1=emittedB.CourseControl1+new float3(0,0,.03f),HorizontalEmissionReference=null};
        var unknownJoin=new ImportLog();NetworkQualityAudit.Planned(new[]{emittedA,kink},unknownJoin,default);
        Check(unknownJoin.Findings.Single(f=>f.Category=="continuation-heading").State=="Unavailable","A sub-native-tolerance kink without a rounding reference falsely passed planned G1");
        var railSlots=NativeLaneObligations.Build(new[]{railA,railB});
        Check(railSlots.Movements.Count==2&&railSlots.Movements.All(m=>m.From.Rank==0&&m.To.Rank==0),"Two-track continuation lost a required travel direction or demanded two lanes per direction");
        var crossing=new[]{Line(10,90,100,new(-100,50,0),new(0,50,0)),Line(10,100,91,new(0,50,0),new(100,50,0)),
            Line(11,92,100,new(0,60,-100),new(0,60,0),NetworkKind.Rail),Line(11,100,93,new(0,60,0),new(0,60,100),NetworkKind.Rail)};
        var level=NetworkTunnelProfile.Solve(crossing,200,Array.Empty<(int,float,float)>(),new ImportLog(),default,design:new(terrain,2000));
        var crossingHeights=level.SelectMany(s=>new[]{(Node:s.StartNodeId,Y:s.CourseStart.y),(Node:s.EndNodeId,Y:s.CourseEnd.y)}).Where(p=>p.Node==100).Select(p=>p.Y).ToArray();
        Check(crossingHeights.Distinct().Count()==1&&level.Select(s=>s.Kind).Distinct().Count()==2,
            "An explicit ground road/rail crossing split its height or welded network kinds");
        var fixedCrossing=NetworkTunnelProfile.Solve(crossing,200,Array.Empty<(int,float,float)>(),new ImportLog(),default,
            verifiedBoundaryHeights:new Dictionary<SourceNodeKey,double>{{new(NetworkKind.Road,100),50},{new(NetworkKind.Rail,100),60}},design:new(terrain,2000));
        Check(fixedCrossing.Where(s=>s.Kind==NetworkKind.Road).All(s=>(s.StartNodeId==100?s.CourseStart.y:s.CourseEnd.y)==50)&&
            fixedCrossing.Where(s=>s.Kind==NetworkKind.Rail).All(s=>(s.StartNodeId==100?s.CourseStart.y:s.CourseEnd.y)==60),
            "Incompatible committed road/rail heights were merged or recursively retried");
        var missingContracts=NetworkTunnelProfile.MeasurePhysical(new[]{a},terrain,200,2000,new Dictionary<string,NativeAssetContract>(),default);
        Check(missingContracts.UncertifiedClearance>0,"Missing native envelopes silently became complete clearance coverage");
        var support=new ImportLog();NetworkTerrainSupport.Validate(new[]{a,b},terrain,200,2000,support,default);
        Check(support.Findings.Single(f=>f.Category=="encoded-ground-support").State=="Passed","Flat encoded support did not receive a continuous platform certificate");
        var floating=new ImportLog();NetworkTerrainSupport.Validate(new[]{a with{Start=a.Start+new float3(0,10,0),End=a.End+new float3(0,10,0)}},terrain,200,2000,floating,default);
        Check(floating.Findings.Single(f=>f.Category=="encoded-ground-support").Failed==1,"A requested but unrealized ground platform was called supported");
        var originalRows=new SparseProfileRows(2);var mergedRows=new SparseProfileRows(2);
        foreach(var weight in new[]{2d,3d})
        {
            originalRows.Append(new[]{0,1},new[]{1d,-2d},2,3,weight,ProfileRowKind.UpperHingeCost);
            mergedRows.Append(new[]{0,1},new[]{1d,-2d},2,3,weight,ProfileRowKind.UpperHingeCost,coalescePrevious:true);
        }
        originalRows.Freeze();mergedRows.Freeze();var gradientA=new double[2];var gradientB=new double[2];
        var costA=originalRows.Evaluate(new[]{9d,1d},gradientA,default);var costB=mergedRows.Evaluate(new[]{9d,1d},gradientB,default);
        Check(mergedRows.Count==1&&costA==costB&&gradientA.SequenceEqual(gradientB),"Exact equivalent-row coalescing changed the objective or gradient");
        Console.WriteLine("PASS: source support/certification, optional float-grid cuts, joint native-curvature rail frames, frozen neighbours, one-shot physical reconstruction, honest quality states and both train travel slots.");
    }
}
