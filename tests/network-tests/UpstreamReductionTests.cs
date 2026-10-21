using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

static class UpstreamReductionTests
{
    static void Check(bool condition,string message){if(!condition)throw new Exception(message);}
    static PreparedSegment Road(long way,long a,long b,float start,float end)=>
        new(way,a,b,new(start,50,0),new(end,50,0),NetworkKind.Road,"motorway",2,true,false,false,0)
        {NativeWidth=8,NativeAssetKey="road",NativeLaneOffsets=new[]{-2f,2f}};
    public static void Run()
    {
        var censusSource=new[]{Road(900,900,901,-100,0),Road(901,901,902,0,100),Road(902,902,903,100,200) with{Bridge=true}};
        var censusSnapshot=censusSource.ToArray();
        var census=NetworkBoundaryPolicy.Survey(censusSource,SourceTopologyObligations.Capture(censusSource),new ImportLog(),"fixture");
        Check(census.Nodes==4&&census.Ends==2&&census.DegreeTwo==2&&census.ProfileBoundaries==1&&census.SameSemanticDegreeTwo==1&&
            censusSource.SequenceEqual(censusSnapshot),"Boundary census changed input or confused a semantic transition with an observation cut");
        var censusLog=new ImportLog();var reversedPort=censusSource[1] with{StartNodeId=902,EndNodeId=901,Start=censusSource[1].End,End=censusSource[1].Start};
        var incompatible=NetworkBoundaryPolicy.Survey(new[]{censusSource[0],reversedPort},null,censusLog,"reversed");
        Check(incompatible.ProfileBoundaries==1&&incompatible.SameSemanticDegreeTwo==0&&censusLog.Metric("Boundary census reversed: nodes")==3,
            "Read-only census lost scalar totals or treated opposed one-way source ports as shareable");
        var changedPair=NetworkBoundaryPolicy.Survey(new[]{censusSource[0],censusSource[1] with{PairedCorridorId=99}},null,new ImportLog(),"pair-change");
        Check(changedPair.ProfileBoundaries==1&&changedPair.SameSemanticDegreeTwo==0,"Pairing change disappeared from semantic cut census");
        var requiredAlias=censusSource[0] with{SourceNodeAliases=new[]{new SourceNodeAlias(999,901,true)}};
        Check(NetworkBoundaryPolicy.Survey(new[]{requiredAlias,censusSource[1]},null,new ImportLog(),"alias").RequiredPorts==1,
            "Mandatory alias did not retain its canonical native-port census");
        var compositionAssets=new Dictionary<string,NativeAssetContract>{{"road",new("road",
            new[]{new NativeCompositionEnvelope(NativeStructureMode.Ground,0,8,0,1,-2,2,8,true)},new[]{-2f,2f})}};
        var nominal=censusSource[1] with{Lanes=3};
        Check((NetworkBoundaryPolicy.Classify(censusSource[0],false,nominal,true)&NetworkBoundaryKind.Composition)!=0&&
            (NetworkBoundaryPolicy.Classify(censusSource[0],false,nominal,true,compositionAssets)&NetworkBoundaryKind.Composition)!=0&&
            NetworkBoundaryPolicy.SameCapturedComposition(censusSource[0],nominal,compositionAssets),
            "Captured composition evidence must not alone remove a required profile freedom");
        Check((NetworkBoundaryPolicy.Classify(censusSource[0],false,nominal with{NativeLaneOffsets=new[]{-4f,0,4}},true,compositionAssets)&NetworkBoundaryKind.Composition)!=0&&nominal.Lanes==3,
            "Actual lane transition or original source metadata was lost by profile simplification");
        var parentRows=new SparseProfileRows(7);
        for(var i=0;i<7;i++)parentRows.Append(new[]{i},new[]{1d},1,50,1,ProfileRowKind.EqualityCost);parentRows.Freeze();
        var parentStars=new[]{new ProfileStar(0,new[]{1},new[]{30d},new[]{0}),
            new ProfileStar(3,new[]{2,4},new[]{-30d,30d},new[]{0,0}),new ProfileStar(6,new[]{5},new[]{-30d},new[]{0})};
        PreparedSegment Handles(PreparedSegment s)=>s with{Control1=s.Start+new float3(30,0,0),Control2=s.End-new float3(30,0,0)};
        NetworkProfileParents.Reduction? ParentFor(PreparedSegment end)=>NetworkProfileParents.Reduce(new[]{Handles(requiredAlias),Handles(end)},new[]{0,3},new[]{1,4},new[]{2,5},new[]{3,6},
            parentRows,Enumerable.Repeat(50d,7).ToArray(),parentStars,100,new ImportLog(),default,compositionAssets);
        Check(ParentFor(censusSource[1])?.Prior.Length==4&&ParentFor(nominal)==null&&requiredAlias.SourceNodeAliases.Single().Mandatory&&nominal.Lanes==3,
            "Compatible observations did not share a parent or nominal transition/source/native alias metadata changed");
        Check(ParentFor(nominal with{SourceTags=new Dictionary<string,string>{{"turn:lanes","left|through|right"}}})==null&&
            ParentFor(nominal with{RawSourceTags=new Dictionary<string,string>{{"access","private"}}})==null&&
            ParentFor(nominal with{ForwardLanes=3})==null&&ParentFor(nominal with{NativeLaneOffsets=new[]{-4f,0,4}})==null,
            "Routing, directional or actual native lane transition was incorrectly removed from parent input");
        Check(!NetworkBoundaryPolicy.SameCapturedComposition(requiredAlias with{NativeWidth=float.PositiveInfinity},nominal with{NativeWidth=float.PositiveInfinity},compositionAssets),
            "Nonfinite native width was accepted as a captured-composition proof");
        var localAllowances=Enumerable.Range(0,4).Select(i=>Road(930,9300+i,9301+i,-300+150*i,-150+150*i) with
            {SourceFitToleranceMeters=i==0?16:2,SourceGeometry=new[]{new float3(-300+150*i,50,0),new float3(-150+150*i,50,0)}}).ToArray();
        var requested=new double2(Math.Cos(Math.PI/9),Math.Sin(Math.PI/9));var approachLog=new ImportLog();
        var localFit=NetworkApproachFrames.Prepare(localAllowances,new[]{new NetworkApproachTapers.DirectionRequest(0,true,requested)},approachLog,default);
        Check(approachLog.Metric("Whole approach frames reconstructed")>0&&
            math.dot(math.normalizesafe((double2)(localFit[0].CourseControl1-localFit[0].CourseStart).xz),requested)>1-1e-7&&
            localFit.Select((s,i)=>NetworkNormalizer.WithinTolerance(s,i==0?16:2)).All(v=>v),
            "Approach partitioning ignored a local allowance or failed to retain the requested tangent inside every original envelope");
        var offsetMain=new[]{Road(940,9400,9401,-200,0),Road(941,9401,9402,0,200)}
            .Select(s=>s with{StartCurveOffset=new(0,0,10),EndCurveOffset=new(0,0,10),PairedCorridorId=42}).ToArray();
        var exit=Road(942,9401,9403,0,200) with{Classification="motorway_link",SourceGeometry=new[]{new float3(0,50,0),new float3(200,50,0)}};
        var selectedFrames=offsetMain.Concat(new[]{exit}).ToArray();var finalLinks=NetworkLaneAlignment.MaterializeLinkApproaches(selectedFrames,new ImportLog(),default);
        Check(finalLinks.Take(2).SequenceEqual(offsetMain)&&finalLinks.Any(s=>s.WayId==exit.WayId&&s.StartNodeId==exit.StartNodeId),
            "Link handoff rewrote selected paired courses or lost an exterior source port");
        var finalExit=finalLinks.First(s=>s.WayId==exit.WayId&&s.StartNodeId==exit.StartNodeId);
        var exitHeading=math.normalizesafe((finalExit.CourseControl1-finalExit.CourseStart).xz);
        Check(Math.Abs(Math.Acos(math.clamp(exitHeading.x,-1,1))*180/Math.PI-20)<.01&&
            math.distance(finalExit.CourseStart,offsetMain[0].CourseEnd)<.001,
            "Final link did not consume the selected mainline course pose and no-added-lane angle");
        var fractional=selectedFrames.Select(s=>s with{Start=s.Start+new float3(.01f,0,0),End=s.End+new float3(.01f,0,0),
            StartCurveOffset=s.StartCurveOffset+(s.ProtectedLink?default:new float3(10,0,0)),
            EndCurveOffset=s.EndCurveOffset+(s.ProtectedLink?default:new float3(10,0,0))}).ToArray();
        var fractionalExit=NetworkLaneAlignment.MaterializeLinkApproaches(fractional,new ImportLog(),default).First(s=>s.WayId==exit.WayId);
        Check(math.all(fractionalExit.Start==fractional[2].Start)&&math.all(fractionalExit.CourseStart==fractional[0].CourseEnd),
            "Selected-course publication reconstructed a canonical anchor through a lossy float subtraction");
        var points=Enumerable.Range(0,101).Select(i=>new float3(i,20,i<=50?0:i-50)).ToArray();
        var inclinedRail=Enumerable.Range(0,21).Select(i=>new float3(i*10,20+i,0)).ToArray();
        Check(NetworkSourceFit.TryFit(inclinedRail,inclinedRail[0],inclinedRail[^1],requested,new(1,0),16,default,
            out var balancedRail,out _,balancedHandles:true)&&
            Math.Abs(balancedRail.P1.y-math.lerp(20d,40d,1d/3))<1e-10&&
            Math.Abs(balancedRail.P2.y-math.lerp(20d,40d,2d/3))<1e-10,
            "Bounded curvature alternative lost the longitudinal intent or original geographic certificate");
        Check(NetworkDesignChecks.Certify(balancedRail,.02,double.PositiveInfinity,token:default,maximumLeaves:4096).Quality==DesignQuality.Certified,
            "Balanced rail candidate did not meet the native curvature fixture");
        Check(!NetworkSourceFit.TryFit(inclinedRail,inclinedRail[0],inclinedRail[^1],requested,new(1,0),.01,default,
            out _,out _,balancedHandles:true),"Curvature alternative bypassed the original source envelope");
        var tinyJoin=new[]{Road(950,9500,9501,0,.1f),Road(951,9501,9502,.1f,.2f)}.Select((s,i)=>s with
            {Start=new(i*.1f,299.99902f+i*.001f,0),End=new((i+1)*.1f,300.00002f+i*.001f,0)}).ToArray();
        tinyJoin[0]=tinyJoin[0] with{Control2=new(2f/30,299.9997f,0)};
        tinyJoin[1]=tinyJoin[1] with{Control1=new(4f/30,300.0004f,0)};
        var publicationLog=new ImportLog();
        var published=NetworkTunnelProfile.Solve(tinyJoin,1000,new[]{(0,.5f,1000f)},publicationLog,default,
            new Dictionary<SourceNodeKey,double>{{new(NetworkKind.Road,9500),299.999014},{new(NetworkKind.Road,9501),300.000014},{new(NetworkKind.Road,9502),300.001014}});
        var incoming=published[0].CourseControl2-published[0].CourseEnd;var outgoing=published[1].CourseControl1-published[1].CourseStart;
        Console.WriteLine("FLOAT PUBLICATION FIXTURE "+string.Join(" | ",publicationLog.Lines.Where(l=>l.Contains("PROFILE FLOAT")||l.Contains("PROFILE SEAMS")))+
            $"; jump={Math.Abs(incoming.y/math.length(incoming.xz)+outgoing.y/math.length(outgoing.xz)):R}; height={published[0].CourseEnd.y:R}");
        Check(Math.Abs(incoming.y/math.length(incoming.xz)+outgoing.y/math.length(outgoing.xz))<=1e-5&&
            published[0].CourseEnd.y==(float)300.000014&&publicationLog.Lines.Any(l=>l.Contains("candidateSeams=1; accepted=True")),
            "Single float publication did not retain the solved through seam and fixed node height");
        var snapshot=points.ToArray();var station=new double[points.Length];
        for(var i=1;i<points.Length;i++)station[i]=station[i-1]+math.distance(points[i-1].xz,points[i].xz);
        var selected=NetworkWorkingPolyline.Select(points,station,.05,default);
        Check(selected!=null&&selected.Length==3&&selected.Contains(50)&&selected[0]==0&&selected[^1]==100,
            "Working geometry failed to retain endpoints/a genuine corner while removing redundant shape points");
        var duplicate=Enumerable.Range(0,101).Select(i=>new float3(i/2,20,0)).ToArray();
        var duplicateStations=duplicate.Select(p=>(double)p.x).ToArray();
        Check(NetworkWorkingPolyline.Select(duplicate,duplicateStations,.05,default)!.Length==2,
            "Duplicate fitting observations remained independent design decisions");
        Check(points.SequenceEqual(snapshot),"Working-point cleanup mutated immutable source evidence");
        var invalid=points.ToArray();invalid[50]=new(float.NaN,20,0);
        Check(!NetworkSourceFit.TryFit(invalid,points[0],points[^1],new(1,0),new(1,1),2,default,out _,out _),
            "Nonfinite source evidence entered recursive fitting/certification");
        var arc=Enumerable.Range(0,101).Select(i=>{var t=i*Math.PI/200;return new float3((float)(100*Math.Cos(t)),20,(float)(100*Math.Sin(t)));}).ToArray();
        var raw=NetworkSourceFit.TryFit(arc,arc[0],arc[^1],new(0,1),new(-1,0),2,default,out _,out _,false);
        var reduced=NetworkSourceFit.TryFit(arc,arc[0],arc[^1],new(0,1),new(-1,0),2,default,out var fit,out _);
        Check(raw&&reduced,"Working observations rejected a previously certifiable curved source");
        var curve=Road(1,1,2,arc[0].x,arc[^1].x) with{Start=arc[0],End=arc[^1],Control1=(float3)fit.P1,Control2=(float3)fit.P2,SourceGeometry=arc};
        Check(NetworkNormalizer.WithinTolerance(curve,2),"Reduced observations hid a full-source fitting failure");

        var full=new SparseProfileRows(3);var compact=new SparseProfileRows(3,coalesceInterleaved:true);
        for(var i=0;i<9000;i++)
        {
            var indices=new[]{i%3,(i+1)%3};var coefficients=new[]{1d,2d};var kind=i%2==0?ProfileRowKind.EqualityCost:ProfileRowKind.UpperHingeCost;
            full.Append(indices,coefficients,2,i%7-3,1+i%3,kind,false);
            compact.Append(indices,coefficients,2,i%7-3,1+i%3,kind);
        }
        full.Freeze();compact.Freeze();var x=new[]{2d,-1,4};var ga=new double[3];var gb=new double[3];
        Check(compact.Count<1000&&compact.CoalescedRows>8000,"Nonadjacent equivalent rows were not reduced");
        Check(full.Evaluate(x,ga,default)==compact.Evaluate(x,gb,default)&&ga.SequenceEqual(gb),
            "Equivalent-row reduction changed objective or gradient");
        var distinct=new SparseProfileRows(1,coalesceInterleaved:true);
        distinct.Append(new[]{0},new[]{1d},1,2,1,ProfileRowKind.EqualityCost);
        distinct.Append(new[]{0},new[]{1d},1,3,1,ProfileRowKind.EqualityCost);
        distinct.Append(new[]{0},new[]{1d},1,2,1,ProfileRowKind.UpperHingeCost);
        distinct.Append(new[]{0},new[]{1d+1e-10},1,2,1,ProfileRowKind.EqualityCost);
        Check(distinct.Count==4,"A distinct physical requirement was discarded as approximately equivalent");
        var indexed=new SparseProfileRows(2);var expectedCost=0d;var expectedVariableCost=0d;var expectedGradient=new double[2];var vector=new[]{.25,-.75};
        for(var i=0;i<12000;i++)
        {
            var variable=i<4096&&i%4==0||i>=8192&&i%4==0;var id=(i/4)%2;var coefficient=i%8==0?2d:-3d;var target=(i%7)-3d;var weight=1+i%3;
            var kind=i%3==0?ProfileRowKind.UpperHingeCost:ProfileRowKind.EqualityCost;
            indexed.Append(new[]{id},new[]{coefficient},variable?1:0,target,weight,kind,false);
            var residual=(variable?coefficient*vector[id]:0)-target;if(kind==ProfileRowKind.UpperHingeCost)residual=Math.Max(0,residual);
            var cost=.5*weight*residual*residual;expectedCost+=cost;
            if(variable){expectedVariableCost+=cost;expectedGradient[id]+=weight*residual*coefficient;}
        }
        indexed.Freeze();var indexedGradient=new double[2];
        Check(indexed.IndexedVariableRows&&indexed.Evaluate(vector,indexedGradient,default,includeConstantObjective:false)==expectedVariableCost&&
            indexedGradient.SequenceEqual(expectedGradient)&&indexed.Evaluate(vector,null,default)==expectedCost,
            "Frozen-row indexing changed original row evidence, variable summation order, objective or gradient");
        var conditioning=indexed.Conditioning(default);var direction=Enumerable.Range(0,2).Select(i=>expectedGradient[i]/conditioning.D[i]).ToArray();
        var denominator=Enumerable.Range(0,2).Sum(i=>expectedGradient[i]*direction[i]);var numerator=0d;
        for(long i=0;i<indexed.Count;i++)
        {
            var row=indexed.Row(i);var derivative=0d;
            for(var j=0;j<row.Count;j++)derivative+=indexed.Coefficient(row.Offset+j)*direction[indexed.Index(row.Offset+j)];
            numerator+=row.Weight*derivative*derivative;
        }
        Check(indexed.DirectionalLipschitz(expectedGradient,conditioning.D,conditioning.L,new double[2],default)==Math.Min(conditioning.L,1.01*(numerator/denominator)),
            "Indexed directional curvature differs from the complete original-row reference");
        using(var cancelled=new CancellationTokenSource())
        {cancelled.Cancel();try{indexed.Evaluate(vector,null,cancelled.Token,includeConstantObjective:false);throw new Exception("Indexed sweep ignored cancellation");}catch(OperationCanceledException){}}
        for(var degree=0;degree<=7;degree++)
        {
            var integral=Enumerable.Range(0,4).Sum(i=>ProfileIntentQuadrature.Weights[i]*Math.Pow(ProfileIntentQuadrature.Nodes[i],degree));
            Check(Math.Abs(integral-1d/(degree+1))<1e-13,"Cubic-intent quadrature is not exact through degree seven");
        }

        var left=Road(1,1,2,-10,0) with{Tunnel=true,Layer=-1};var middle=Road(2,2,3,0,100);
        var right=Road(3,3,4,100,110) with{Tunnel=true,Layer=-1};
        Check(CoordinatedNetworkDesign.TunnelIntervals(new[]{left,middle,right},new ImportLog(),default)[1].Tunnel,
            "An unsupported short interruption between compatible tunnels was not resolved");
        var explicitGround=middle with{RawSourceTags=new Dictionary<string,string>{{"tunnel","no"}}};
        Check(!CoordinatedNetworkDesign.TunnelIntervals(new[]{left,explicitGround,right},new ImportLog(),default)[1].Tunnel,
            "Structural simplification overwrote explicit ground policy");
        Check(!CoordinatedNetworkDesign.TunnelIntervals(new[]{left,middle,right with{Layer=-2}},new ImportLog(),default)[1].Tunnel,
            "Structural simplification joined incompatible levels");
        var branch=Road(4,2,5,0,30) with{End=new(30,50,30)};
        Check(!CoordinatedNetworkDesign.TunnelIntervals(new[]{left,middle,right,branch},new ImportLog(),default)[1].Tunnel,
            "Structural simplification crossed a real junction");
        var terrain=Enumerable.Repeat((ushort)32768,17*17).ToArray();
        var waterPolicy=CoordinatedNetworkDesign.StructureIntervals(new[]{explicitGround with{RawSourceTags=new Dictionary<string,string>{{"bridge","no"}}}},
            terrain,100,1000,new ImportLog(),default,_=>60,true);
        Check(!waterPolicy[0].Bridge,"Water inference overrode an explicit bridge policy");
        var route=new[]{Road(100,10,11,0,100),Road(101,11,12,100,200),Road(102,12,13,200,300)};
        var log=new ImportLog{ScalarDiagnostics=true};
        CoordinatedNetworkDesign.Prepare(route,terrain,100,1000,log,default,structureIntervalsResolved:true);
        Check(log.Lines.Any(l=>l.StartsWith("DESIGN INPUT routes=1;",StringComparison.Ordinal)),
            "Unnamed compatible source-way boundaries split one logical route");
        Console.WriteLine("PASS: upstream working points, exact sparse-row reduction, cubic intent quadrature, structural evidence guards and unnamed route assembly.");
    }
}
