using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

static class CorrectivePlanTests
{
    static void Check(bool condition,string message){if(!condition)throw new Exception(message);}
    static PreparedSegment Road(long way,long a,long b,float x,float end,int lanes)=>
        new(way,a,b,new(x,50,0),new(end,50,0),NetworkKind.Road,"motorway",lanes,true,false,false,0)
        {NativeWidth=lanes*4,NativeAssetKey="road",Reference="A1",SourceFitToleranceMeters=24,
            NativeLaneOffsets=Enumerable.Range(0,lanes).Select(i=>(i-(lanes-1)*.5f)*4).ToArray()};
    public static void Run()
    {
        var envelopes=new[]{new NativeCompositionEnvelope(NativeStructureMode.Ground,0,12,0,1,-2,2,8,true),
            new NativeCompositionEnvelope(NativeStructureMode.Elevated,0,12,0,1,-2,2,8,true)};
        var first=new NativeAssetContract("road",envelopes,new[]{-4f,0,4});
        var reordered=new NativeAssetContract("road",envelopes.Reverse().ToArray(),new[]{4f,-4,0});
        Check(NativeAssetContract.Fingerprint(new[]{first})==NativeAssetContract.Fingerprint(new[]{reordered}),"Contract order invalidates unchanged effective design");
        var sameMode=envelopes.Append(envelopes[0] with{Width=14}).ToArray();
        Check(NativeAssetContract.Fingerprint(new[]{new NativeAssetContract("road",sameMode,new[]{0f})})==
            NativeAssetContract.Fingerprint(new[]{new NativeAssetContract("road",sameMode.Reverse().ToArray(),new[]{0f})}),
            "Same-mode envelope order invalidates an unchanged native contract");
        var changed=new NativeAssetContract("road",envelopes,new[]{-3f,0,4});
        Check(NativeAssetContract.Fingerprint(new[]{first})!=NativeAssetContract.Fingerprint(new[]{changed}),"Changed native lane offset reused design signature");
        var rows=new SparseProfileRows(2);var original=new SparseProfileRows(2);
        foreach(var kind in new[]{ProfileRowKind.EqualityCost,ProfileRowKind.UpperHingeCost})
        {
            foreach(var weight in new[]{2d,3d})
            {rows.Append(new[]{1,0},new[]{-2d,1d},2,3,weight,kind);original.Append(new[]{0,1},new[]{1d,-2d},2,3,weight,kind,false);}
            rows.Append(Array.Empty<int>(),Array.Empty<double>(),0,-4,2,kind);
            original.Append(Array.Empty<int>(),Array.Empty<double>(),0,-4,2,kind,false);
        }
        rows.Freeze();original.Freeze();Check(rows.Count==4&&original.Count==6,"Exact adjacent duplicates did not coalesce");
        foreach(var x in new[]{new[]{0d,0d},new[]{10d,-7d},new[]{-1d,2d}})
        {
            var ga=new double[2];var gb=new double[2];
            Check(Math.Abs(rows.Evaluate(x,ga,default)-original.Evaluate(x,gb,default))<1e-10&&ga.Zip(gb).All(p=>Math.Abs(p.First-p.Second)<1e-10),
                "Canonical coalescing changed the objective, constant contribution or gradient");
        }
        var constant=new SparseProfileRows(1);constant.Append(new[]{0},new[]{1d},1,1,1,ProfileRowKind.EqualityCost);
        constant.Append(Array.Empty<int>(),Array.Empty<double>(),0,-1e12,1,ProfileRowKind.UpperHingeCost);constant.Freeze();
        var constantResult=NetworkProfileSolve.SolveComponents(constant,new[]{0d},10,Array.Empty<ProfileStar>());
        Check(constantResult.Report.Outcome==ProfileSolveOutcome.Converged&&Math.Abs(constantResult.Heights[0]-1)<1e-8&&
            constantResult.Report.Objective==5e23&&constant.Count==2,"Constant contradiction changed convergence or disappeared from reported evidence");
        var route=new[]{Road(1,1,2,-600,0,3),Road(2,2,3,0,600,2),Road(3,3,4,600,1200,2)};
        // Actual native slots, not this deliberately incorrect nominal count.
        route[1]=route[1] with{Lanes=4};route[2]=route[2] with{Lanes=4};
        var aligned=NetworkLaneAlignment.Prepare(route,new ImportLog(),default);
        var byWay=aligned.GroupBy(s=>s.WayId).ToDictionary(g=>g.Key,g=>g.OrderBy(s=>s.CourseStart.x).ToArray());
        float Datum(PreparedSegment s,bool start)=>-(start?s.CourseStart.z:s.CourseEnd.z)+s.NativeLaneOffsets.Min();
        Check(Math.Abs(Datum(byWay[1][^1],false)-Datum(byWay[2][0],true))<=.05,"3-to-2 continuing lane datum is discontinuous");
        Check(Math.Abs(Datum(byWay[2][^1],false)-Datum(byWay[3][0],true))<=.05&&Math.Abs(Datum(byWay[2][0],true)-Datum(byWay[2][^1],false))<=.05,
            "Continuing lane shift decays back to the source centreline");
        var opposite=Road(4,40,41,1200,-600,2) with{Start=new(1200,50,-12),End=new(-600,50,-12)};
        var coupled=NetworkPairedCorridors.Prepare(aligned.Concat(new[]{opposite}).ToArray(),new ImportLog(),default);
        foreach(var node in coupled.Where(s=>s.WayId is 1 or 2 or 3).SelectMany(s=>new[]{(Id:s.StartNodeId,Point:s.Start),(Id:s.EndNodeId,Point:s.End)}).GroupBy(p=>p.Id))
            Check(node.All(p=>math.all(p.Point==node.First().Point)),"Pair formation overwrote canonical lane-transition anchors");
        var audit=new ImportLog();NetworkJunctionQuality.Audit(coupled,(s,start)=>new(start?s.CourseStart:s.CourseEnd,
            math.normalizesafe((double2)(start?s.CourseControl1-s.CourseStart:s.CourseEnd-s.CourseControl2).xz),s.NativeLaneOffsets),audit,default,"planned","fixture");
        Check(audit.Findings.Where(f=>f.Category=="continuing-lane-datum").All(f=>f.Failed==0),"Pairing fought the persistent continuing-lane datum");
        var formationLog=new ImportLog();
        var repairedFormation=NetworkPairedCorridors.Prepare(aligned.Concat(new[]{opposite with{Start=new(1200,65,-4),End=new(-600,65,-4)}}).ToArray(),formationLog,default,
            width:10000,protectedAreas:Array.Empty<float3[]>(),protectedLines:Array.Empty<float3[]>());
        var repairedAudit=new ImportLog();NetworkJunctionQuality.Audit(repairedFormation,(s,start)=>new(start?s.CourseStart:s.CourseEnd,
            math.normalizesafe((double2)(start?s.CourseControl1-s.CourseStart:s.CourseEnd-s.CourseControl2).xz),s.NativeLaneOffsets),repairedAudit,default,"planned","fixture");
        Check(repairedAudit.Findings.Where(f=>f.Category=="continuing-lane-datum").All(f=>f.Failed==0),
            "Coordinated paired repair fought a width-transition continuing lane");
        Check(repairedFormation.Any(s=>s.PairedProfileId!=0)&&repairedFormation.Where(s=>s.PairedProfileId!=0).All(s=>!s.PairedProfileOnly),
            "Width-transition repair passed lane continuity only by retaining the overlapping source curves: "+string.Join(";",formationLog.Lines));
        var a=Road(10,10,11,-200,200,2);var b=Road(11,12,13,200,-200,2) with{Start=new(200,65,-4),End=new(-200,65,-4)};
        var guarded=NetworkPairedCorridors.Prepare(new[]{a with{SourceFitToleranceMeters=.01f},b with{SourceFitToleranceMeters=.01f}},new ImportLog(),default);
        Check(guarded.All(s=>s.PairedProfileId!=0&&s.PairedProfileOnly),"Spacing rejection discarded the independently confirmed height relationship");
        var uneven=NetworkPairedCorridors.Prepare(new[]{a with{Control1=new(-199,50,0),Control2=new(199,50,0),SourceFitToleranceMeters=.01f},
            b with{SourceFitToleranceMeters=.01f}},new ImportLog(),default);
        Check(uneven.Any(s=>s.PairedProfileOnly),"Straight profile metric fixture has no confirmed paired spans");
        foreach(var pair in uneven.Where(s=>s.PairedProfileOnly).GroupBy(s=>s.PairedSpanId))
        {
            var forward=pair.Single(s=>!s.PairedReversed);var reverse=pair.Single(s=>s.PairedReversed);
            Check(Math.Abs(math.distance(forward.CourseStart.xz,forward.CourseControl1.xz)-math.distance(reverse.CourseEnd.xz,reverse.CourseControl2.xz))<1e-3&&
                Math.Abs(math.distance(forward.CourseEnd.xz,forward.CourseControl2.xz)-math.distance(reverse.CourseStart.xz,reverse.CourseControl1.xz))<1e-3,
                "Straight profile-only pair retained incompatible physical handle metrics");
            Check(Math.Abs(forward.CourseControl1.z)<1e-6&&Math.Abs(reverse.CourseControl1.z+4)<1e-6,
                "Profile-only parameter normalization moved a straight source locus");
        }
        var changingPartner=new[]{Road(50,5000,5001,0,1000,2),
            Road(51,5100,5101,500,0,2) with{Start=new(500,50,-20),End=new(0,50,-20)},
            Road(52,5200,5201,1000,500,2) with{Start=new(1000,50,-20),End=new(500,50,-20)}};
        var cuts=NetworkPairedCorridors.Prepare(changingPartner,new ImportLog(),default).Where(s=>s.WayId==50).ToArray();
        Check(cuts.Select(s=>s.PairedCorridorId).Where(id=>id!=0).Distinct().Count()==2,"Partner-change fixture did not exercise two paired intervals");
        var reached=new HashSet<long>{5000};
        for(var pass=0;pass<cuts.Length;pass++)foreach(var cut in cuts)if(reached.Contains(cut.StartNodeId))reached.Add(cut.EndNodeId);
        Check(reached.Contains(5001),"Adjacent pair intervals allocated different identities at the same original source cut");
        var empty=new NetworkQualityFinding("native","lanes","fixture",0,0,1,0,0,"Expected lanes not published");
        Check(empty.State=="Unavailable","Empty expected lane coverage became passed");
        Check((empty with{Unavailable=0,Expected=5}).State=="Unavailable","Missing expected observations became not applicable");
        PreparedSegment Rail(long way,long aNode,long bNode,float3 p,float3 q)=>new(way,aNode,bNode,p,q,NetworkKind.Rail,"rail",2,false,false,false,0)
            {NativeWidth=12,NativeAssetKey="rail",NativeCenterlineCurvatureLimit=.03,SourceGeometry=new[]{p,q},SourceWayIds=new[]{way}};
        var fan=Enumerable.Range(0,4).Select(i=>Rail(100+i,10+i,100,new(0,50,(i-1.5f)*8),new(500,50,0)))
            .Concat(new[]{Rail(110,100,200,new(500,50,0),new(1000,50,-200)),Rail(111,100,201,new(500,50,0),new(1000,50,200)),
                Rail(112,200,300,new(1000,50,-200),new(3500,50,-200)),Rail(113,201,301,new(1000,50,200),new(3500,50,200))}).ToArray();
        var terminalLog=new ImportLog();var reduced=NetworkRailTerminalFormations.Prepare(fan,terminalLog,default);
        Check(reduced.Count(s=>s.RailTerminalFormationId!=0)==3&&terminalLog.Metric("Functional rail terminal formations")==1,
            "Proven two-exterior-group terminal did not produce one degree-three Y");
        SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(fan),reduced,terminalLog,default);
        Check(terminalLog.Metric("Missing mandatory selected-source ports")==0&&terminalLog.Metric("Unresolved selected-source reachability obligations")==0,
            "Terminal macrograph lost original external connectivity or contraction aliases");
        var oldAlias=fan.Select(s=>s.WayId==112?s with{SourceNodeAliases=new[]{new SourceNodeAlias(10,200,true)}}:s).ToArray();
        var replacedAlias=NetworkRailTerminalFormations.Prepare(oldAlias,new ImportLog(),default);
        Check(replacedAlias.SelectMany(s=>s.SourceNodeAliases).Where(a=>a.SourceNodeId==10).All(a=>a.CanonicalNodeId==10),
            "A surviving exterior section retained a stale alias after an intentional contraction");
        var unknownRail=fan.Select(s=>s with{NativeCenterlineCurvatureLimit=null}).ToArray();var unknownRailLog=new ImportLog();
        Check(ReferenceEquals(NetworkRailTerminalFormations.Prepare(unknownRail,unknownRailLog,default),unknownRail)&&
            unknownRailLog.Findings.Single(f=>f.Category=="rail-terminal-contraction").State=="Unavailable",
            "Terminal contraction invented an unbounded native rail curvature contract");
        var originalPort=Road(800,8000,8001,0,100,2);var ghostLog=new ImportLog();
        var ghostPort=originalPort with{StartNodeId=8100,EndNodeId=8101,
            SourceNodeAliases=new[]{new SourceNodeAlias(8000,8999,true),new SourceNodeAlias(8001,8999,true)}};
        SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(new[]{originalPort}),new[]{ghostPort},ghostLog,default);
        Check(ghostLog.Metric("Missing mandatory selected-source ports")==2&&ghostLog.Metric("Unresolved selected-source reachability obligations")==1,
            "Two aliases pointing to an absent canonical node fabricated a passed source connection");
        var multi=Enumerable.Range(0,4).Select(i=>Rail(300+i,10+i,100,new(0,50,(i-1.5f)*8),new(500,50,0)))
            .Concat(Enumerable.Range(0,4).SelectMany(i=>new[]{
                Rail(310+i*3,100,150+i,new(500,50,0),new(800,50,(i-1.5f)*100)),
                Rail(311+i*3,150+i,200+i,new(800,50,(i-1.5f)*100),new(1200,50,(i-1.5f)*200)),
                Rail(312+i*3,200+i,300+i,new(1200,50,(i-1.5f)*200),new(4000,50,(i-1.5f)*200)) with{Bridge=i==3}})).ToArray();
        var multiLog=new ImportLog();var multiReduced=NetworkRailTerminalFormations.Prepare(multi,multiLog,default);
        Check(multiLog.Metric("Functional rail terminal formations")==1,"Multi-gate terminal was not reconstructed");
        Check(multiReduced.Where(s=>s.RailTerminalFormationId!=0).SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId})
            .GroupBy(n=>n).All(g=>g.Count()<=3),"Multi-route reconstruction created a high-degree switch");
        SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(multi),multiReduced,multiLog,default);
        Check(multiLog.Metric("Missing mandatory selected-source ports")==0&&multiLog.Metric("Unresolved selected-source reachability obligations")==0,
            "Multi-gate terminal lost an exterior route");
        Check(multiReduced.Any(s=>s.StartNodeId==203&&s.EndNodeId==303&&s.Bridge),"Terminal reconstruction changed an exterior bridge contract");
        var forward1=Road(200,500,501,-200,0,2) with{PairedSpanId=1,PairedProfileId=1,StartHeightFamily=1,EndHeightFamily=2,ReferenceSpan=200};
        var forward2=Road(201,501,502,0,200,2) with{PairedSpanId=2,PairedProfileId=2,StartHeightFamily=3,EndHeightFamily=4,ReferenceSpan=200};
        PreparedSegment Partner(PreparedSegment s,long start,long end)=>s with{WayId=s.WayId+10,StartNodeId=start,EndNodeId=end,
            Start=s.End+new float3(0,0,-20),End=s.Start+new float3(0,0,-20),PairedReversed=true,
            StartHeightFamily=s.EndHeightFamily,EndHeightFamily=s.StartHeightFamily};
        var adjacent=new[]{forward1,forward2,Partner(forward1,601,600),Partner(forward2,602,601)};
        var terrain=Enumerable.Repeat((ushort)32768,17*17).ToArray();
        var shared=NetworkTunnelProfile.Solve(adjacent,100,Array.Empty<(int,float,float)>(),new ImportLog(),default,design:new(terrain,2000));
        Check(Math.Abs(shared[0].CourseEnd.y-shared[2].CourseStart.y)<1e-5&&Math.Abs(shared[1].CourseStart.y-shared[3].CourseEnd.y)<1e-5,
            "Adjacent local station families did not union their exact shared source ports before allocating projection stars");
        var changingMetric=adjacent.Select(s=>s with{ReferenceSpan=s.PairedSpanId==2?1000:200,
            StartReferenceSpeed=s.PairedSpanId==2?.2:1,EndReferenceSpeed=s.PairedSpanId==2?.2:1}).ToArray();
        var metricLog=new ImportLog();
        var metricProfile=NetworkTunnelProfile.Solve(changingMetric,100,Array.Empty<(int,float,float)>(),metricLog,default,
            new Dictionary<SourceNodeKey,double>{{new(NetworkKind.Road,500),50},{new(NetworkKind.Road,502),54}},new(terrain,2000),
            new Dictionary<SourceNodeKey,double>{{new(NetworkKind.Road,500),.01},{new(NetworkKind.Road,600),-.01}});
        double StartGrade(PreparedSegment s)=>(s.CourseControl1.y-s.CourseStart.y)/math.length((double2)(s.CourseControl1-s.CourseStart).xz);
        double EndGrade(PreparedSegment s)=>(s.CourseEnd.y-s.CourseControl2.y)/math.length((double2)(s.CourseEnd-s.CourseControl2).xz);
        Check(Math.Abs(EndGrade(metricProfile[0])-StartGrade(metricProfile[1]))<=1e-5&&Math.Abs(StartGrade(metricProfile[0])-.01)<1e-6,
            "Local reference units or opposing source directions changed a shared physical boundary grade");
        var bridge=Road(700,7000,7001,-100,100,2) with{Bridge=true,Layer=1,PairedProfileId=77,PairedCorridorId=77};
        var ground=Road(701,7100,7101,100,-100,2) with{Start=new(100,50,-20),End=new(-100,50,-20),PairedReversed=true,PairedProfileId=77,PairedCorridorId=77};
        var mode=NetworkVisualDesign.StructureIntent(new[]{bridge,ground},terrain,100,2000,
            new Dictionary<string,NativeAssetContract>{{"road",first}},new ImportLog(),default);
        Check(mode[1].Bridge&&mode[1].Layer==1,"Confirmed paired bridge intent remained contradictory despite an available native contract");
        var stockGround=NativeAssetKeys.Sized(ground);var stockBridge=NativeAssetKeys.Sized(ground with{Bridge=true});
        var stockAssets=new Dictionary<string,NativeAssetContract>{
            {stockGround,new(stockGround,envelopes,new[]{-2f,2f})},
            {stockBridge,new(stockBridge,envelopes,new[]{-3f,3f})}};
        var stockSource=new[]{bridge with{NativeAssetKey=stockBridge},ground with{NativeAssetKey=stockGround}};
        mode=NetworkVisualDesign.StructureIntent(stockSource,terrain,100,2000,stockAssets,new ImportLog(),default);
        Check(mode[1].Bridge&&mode[1].NativeAssetKey==stockBridge&&mode[1].NativeLaneOffsets.SequenceEqual(new[]{-3f,3f}),
            "Inferred mode retained a stale native palette or lane contract");
        stockAssets.Remove(stockBridge);
        mode=NetworkVisualDesign.StructureIntent(stockSource,terrain,100,2000,stockAssets,new ImportLog(),default);
        Check(!mode[1].Bridge&&mode[1].NativeAssetKey==stockGround,"Missing stock mode contract was silently substituted");
        mode=NetworkVisualDesign.StructureIntent(new[]{bridge,ground with{RawSourceTags=new Dictionary<string,string>{{"bridge","no"}}}},terrain,100,2000,
            new Dictionary<string,NativeAssetContract>{{"road",first}},new ImportLog(),default);
        Check(!mode[1].Bridge,"Paired structure inference silently overrode an explicit source ground policy");
        var finiteRoute=new[]{Road(900,9000,9001,-600,-200,2),Road(901,9001,9002,-200,200,2),Road(902,9002,9003,200,600,2)};
        var badSource=Road(903,9030,9031,0,100,2) with{Start=new(float.NaN,50,500)};var protectedObserved=false;
        var safeRemainder=CoordinatedNetworkDesign.Prepare(finiteRoute.Append(badSource).ToArray(),terrain,100,2000,new ImportLog(),default,
            assets:new Dictionary<string,NativeAssetContract>{{"road",first}},
            protectedAreas:new[]{new[]{new float3(-50,0,-20),new float3(50,0,20)}},protectedLines:Array.Empty<float3[]>(),
            sourceTopology:SourceTopologyObligations.Capture(finiteRoute),inspectStage:(name,segments)=>
            {if(name=="regularity-lane-alignment-portals")protectedObserved=segments.Any(s=>s.WayId==901&&s.SourceFitToleranceMeters==2);});
        Check(protectedObserved&&safeRemainder.All(s=>NetworkDesignChecks.Finite(new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd))),
            "Excluding a nonfinite source section dropped the other roads' protection or evidence context");
        ParentRestriction();
        EmissionCompactionTests.Run();
        UpstreamReductionTests.Run();
        Console.WriteLine("PASS: corrective contract signatures, exact-row equivalence, native-slot persistent datum and guarded shared profiles.");
    }
    private static void ParentRestriction()
    {
        var sections=new List<PreparedSegment>();var from=new List<int>();var c1=new List<int>();var c2=new List<int>();var to=new List<int>();
        for(var span=0;span<6;span++)foreach(var reverse in new[]{false,true})
        {
            var a=span*60f;var b=a+60;
            sections.Add(Road(400+span,1000+span,1001+span,reverse?b:a,reverse?a:b,2) with{
                PairedCorridorId=1,PairedSpanId=span+1,PairedReversed=reverse,ReferenceSpan=60,
                PairedReferenceBegin=reverse?b:a,PairedReferenceEnd=reverse?a:b});
            from.Add(reverse?span+1:span);to.Add(reverse?span:span+1);
            c1.Add(7+span*2+(reverse?1:0));c2.Add(7+span*2+(reverse?0:1));
        }
        var raw=new SparseProfileRows(19);for(var i=0;i<19;i++)raw.Append(new[]{i},new[]{1d},1,50+i,1,ProfileRowKind.EqualityCost);raw.Freeze();
        var stars=new List<ProfileStar>();
        for(var node=0;node<=6;node++)
        {
            var controls=new List<int>();var alpha=new List<double>();
            if(node>0){controls.Add(8+(node-1)*2);alpha.Add(-60d/3);}
            if(node<6){controls.Add(7+node*2);alpha.Add(60d/3);}
            stars.Add(new(node,controls.ToArray(),alpha.ToArray(),new int[controls.Count]));
        }
        var reduced=NetworkProfileParents.Reduce(sections,from.ToArray(),c1.ToArray(),c2.ToArray(),to.ToArray(),raw,
            Enumerable.Repeat(50d,19).ToArray(),stars.ToArray(),100,new ImportLog(),default);
        Check(reduced!=null&&reduced.Prior.Length<19,"Native emission knots remained independent height unknowns");
        Check(reduced!.Prior.Length==4&&reduced.Bindings[3].Variables.Length>1,
            "Artificial cuts split a compatible paired profile within the parent span budget");
        var values=Enumerable.Range(0,reduced!.Prior.Length).Select(i=>20d+i*2).ToArray();
        Check(Math.Abs(raw.Evaluate(reduced.Expand(values),null,default)-reduced.Rows.Evaluate(values,null,default))<1e-8,
            "Cubic parent substitution changed the original physical/observation objective");
        Check(new ProfileHardProjector(values.Length,100,reduced.Stars).Project(values,Enumerable.Repeat(1d,values.Length).ToArray()),"Parent hard set is unavailable");
        var expanded=reduced.Expand(values);var projected=(double[])expanded.Clone();
        Check(new ProfileHardProjector(19,100,stars.ToArray()).Project(projected,Enumerable.Repeat(1d,19).ToArray())&&
            expanded.Zip(projected).All(p=>Math.Abs(p.First-p.Second)<1e-8),"Parent substitution lost a native-knot G1 hard constraint");
        var ordinary=Enumerable.Range(0,6).Select(k=>sections[k*2] with{PairedCorridorId=0,PairedSpanId=0,PairedProfileId=0}).ToArray();
        var ordinaryStars=new List<ProfileStar>();
        for(var node=0;node<=6;node++)
        {
            var controls=new List<int>();var alpha=new List<double>();
            if(node>0){controls.Add(8+(node-1)*2);alpha.Add(-math.length((double2)(ordinary[node-1].CourseEnd-ordinary[node-1].CourseControl2).xz));}
            if(node<6){controls.Add(7+node*2);alpha.Add(math.length((double2)(ordinary[node].CourseControl1-ordinary[node].CourseStart).xz));}
            ordinaryStars.Add(new(node,controls.ToArray(),alpha.ToArray(),new int[controls.Count]));
        }
        var nativeFrom=from.Where((_,i)=>i%2==0).ToArray();var nativeTo=to.Where((_,i)=>i%2==0).ToArray();
        var nativeFirst=c1.Where((_,i)=>i%2==0).ToArray();var nativeSecond=c2.Where((_,i)=>i%2==0).ToArray();
        var ordinaryReduction=NetworkProfileParents.Reduce(ordinary,nativeFrom,nativeFirst,nativeSecond,nativeTo,raw,
            Enumerable.Repeat(50d,19).ToArray(),ordinaryStars.ToArray(),100,new ImportLog(),default);
        Check(ordinaryReduction!=null&&ordinaryReduction.Prior.Length<19,"Ordinary emission cuts remained independent profile knots");
        Check(ordinaryReduction!.Bindings[3].Variables.Length>1,
            "An artificial mainline window retained an independent interior height");
        var ordinaryValues=Enumerable.Range(0,ordinaryReduction!.Prior.Length).Select(i=>30d+i).ToArray();
        Check(new ProfileHardProjector(ordinaryValues.Length,100,ordinaryReduction.Stars).Project(ordinaryValues,Enumerable.Repeat(1d,ordinaryValues.Length).ToArray()),"Ordinary parent hard set unavailable");
        var physical=ordinaryReduction.Expand(ordinaryValues);var physicalProjection=(double[])physical.Clone();
        Check(new ProfileHardProjector(19,100,ordinaryStars.ToArray()).Project(physicalProjection,Enumerable.Repeat(1d,19).ToArray())&&
            physical.Zip(physicalProjection).All(p=>Math.Abs(p.First-p.Second)<1e-8),"Unequal native handle reaches changed the physical G1 constraint");
        var parentEnd=ordinaryReduction.Bindings[6].Variables.Single();
        double Parameter(int node)
        {
            var binding=ordinaryReduction.Bindings[node];var end=Array.IndexOf(binding.Variables,parentEnd);
            return end<0?0:Math.Pow(binding.Coefficients[end],1d/3);
        }
        for(var node=1;node<6;node++)
        {
            // Float-grid handle reaches make even these straight cubics' local
            // parameter speeds slightly unequal. Compare in the common parent
            // parameter, not by assuming every native span has equal dt.
            var leftSpan=Parameter(node)-Parameter(node-1);var rightSpan=Parameter(node+1)-Parameter(node);
            var left=(physical[node]-2*physical[8+(node-1)*2]+physical[7+(node-1)*2])/(leftSpan*leftSpan);
            var right=(physical[8+node*2]-2*physical[7+node*2]+physical[node])/(rightSpan*rightSpan);
            Check(Math.Abs(left-right)<1e-8,"An artificial cut introduced a second-derivative jump in the common parent");
        }
        NetworkProfileParents.Reduction? ReduceOrdinary(PreparedSegment[] segments,ProfileStar[] constraints)=>
            NetworkProfileParents.Reduce(segments,nativeFrom,nativeFirst,nativeSecond,nativeTo,raw,
                Enumerable.Repeat(50d,19).ToArray(),constraints,100,new ImportLog(),default);
        var markedNode=ordinary[2].EndNodeId;
        var marked=ordinary.Select((s,i)=>i==2?s with{SourceNodeAliases=new[]{new SourceNodeAlias(90003,markedNode,true)}}:s).ToArray();
        var markedKind=NetworkBoundaryPolicy.Classify(marked[2],false,marked[3],true);
        Check(markedKind==NetworkBoundaryKind.SourcePort&&!NetworkBoundaryPolicy.RequiresProfileBoundary(markedKind),
            "Native provenance alone became a mandatory vertical bend");
        var markedReduction=ReduceOrdinary(marked,ordinaryStars.ToArray());
        Check(markedReduction!=null&&markedReduction.Bindings[3].Variables.Length>1&&marked[2].SourceNodeAliases[0].Mandatory&&marked[2].EndNodeId==markedNode,
            "A topology-only source port either split the shared profile or lost its native identity");
        var compactedMarker=NativeChainCompactor.Prepare(marked,new ImportLog(),default);
        Check(compactedMarker.Any(s=>s.EndNodeId==markedNode)&&compactedMarker.Any(s=>s.StartNodeId==markedNode),
            "A profile-shareable source port was incorrectly removed by native compaction");
        var renamed=ordinary.Select((s,i)=>s with{WayId=900+i,Name="section "+i,Reference="ref "+i,SourceWayIds=new[]{900L+i}}).ToArray();
        Check(NetworkBoundaryPolicy.Classify(renamed[2],false,renamed[3],true)==NetworkBoundaryKind.None&&
            ReduceOrdinary(renamed,ordinaryStars.ToArray())!.Bindings[3].Variables.Length>1,
            "Bookkeeping names or source-way boundaries split a compatible design profile");
        foreach(var (segment,expected) in new[]{
            (ordinary[3] with{Tunnel=true},NetworkBoundaryKind.Structure),
            (ordinary[3] with{Layer=1},NetworkBoundaryKind.Structure),
            (ordinary[3] with{NativeAssetKey="different"},NetworkBoundaryKind.Composition),
            (ordinary[3] with{PhysicalTracks=3},NetworkBoundaryKind.Composition),
            (ordinary[3] with{OneWay=false},NetworkBoundaryKind.Direction),
            (ordinary[3] with{ExplicitIncline=.02f},NetworkBoundaryKind.ProfilePrior),
            (ordinary[3] with{StartTerminal=true},NetworkBoundaryKind.EndpointMarker)})
        {
            var kind=NetworkBoundaryPolicy.Classify(marked[2],false,segment,true);
            Check((kind&expected)!=0&&NetworkBoundaryPolicy.RequiresProfileBoundary(kind),
                "A source port concealed a design-boundary reason: "+expected);
        }
        var pairedMarked=sections.Select((s,i)=>i==4?s with{SourceNodeAliases=new[]{new SourceNodeAlias(90003,s.EndNodeId,true)}}:s).ToArray();
        var pairedMarkedReduction=NetworkProfileParents.Reduce(pairedMarked,from.ToArray(),c1.ToArray(),c2.ToArray(),to.ToArray(),raw,
            Enumerable.Repeat(50d,19).ToArray(),stars.ToArray(),100,new ImportLog(),default);
        Check(pairedMarkedReduction!=null&&pairedMarkedReduction.Bindings[3].Variables.Length>1,
            "A paired topology-only cut acquired an independent profile bend");
        void BoundaryRetained(PreparedSegment[] segments,string message)
        {
            var result=ReduceOrdinary(segments,ordinaryStars.ToArray());
            Check(result!=null&&result.Bindings[3].Variables.Length==1,message);
        }
        BoundaryRetained(ordinary.Select((s,i)=>i>=3?s with{NativeAssetKey="other"}:s).ToArray(),"Native recipe boundary was absorbed");
        BoundaryRetained(ordinary.Select((s,i)=>i>=3?s with{Lanes=3,NativeLaneOffsets=new[]{-4f,0,4}}:s).ToArray(),"Lane composition boundary was absorbed");
        BoundaryRetained(ordinary.Select((s,i)=>i==2?s with{EndTerminal=true}:s).ToArray(),"Terminal marker was absorbed");
        var fixedStars=ordinaryStars.Select((s,i)=>i==3?new ProfileStar(s.Height,s.Controls,s.Alpha,s.Groups,57,new double?[]{.02}):s).ToArray();
        var fixedReduction=ReduceOrdinary(ordinary,fixedStars);
        Check(fixedReduction!=null&&fixedReduction.Bindings[3].Variables.Length==1,"Fixed native height became an interior parent observation");
        Check(ReduceOrdinary(marked,fixedStars)!.Bindings[3].Variables.Length==1,
            "A topology-only marker overrode a fixed native profile constraint");
        var fixedValues=(double[])fixedReduction!.Prior.Clone();
        Check(new ProfileHardProjector(fixedValues.Length,100,fixedReduction.Stars).Project(fixedValues,Enumerable.Repeat(1d,fixedValues.Length).ToArray()),
            "Fixed parent endpoint constraints were unavailable");
        var fixedExpanded=fixedReduction.Expand(fixedValues);var fixedProjected=(double[])fixedExpanded.Clone();
        Check(new ProfileHardProjector(19,100,fixedStars).Project(fixedProjected,Enumerable.Repeat(1d,19).ToArray())&&
            Math.Abs(fixedExpanded[3]-57)<1e-10&&fixedExpanded.Zip(fixedProjected).All(p=>Math.Abs(p.First-p.Second)<1e-8),
            "Parent expansion discarded a verified height or slope");
        var pairedFixed=stars.Select((s,i)=>i==3?new ProfileStar(s.Height,s.Controls,s.Alpha,s.Groups,57,new double?[]{.02}):s).ToArray();
        var pairedFixedReduction=NetworkProfileParents.Reduce(sections,from.ToArray(),c1.ToArray(),c2.ToArray(),to.ToArray(),raw,
            Enumerable.Repeat(50d,19).ToArray(),pairedFixed,100,new ImportLog(),default);
        Check(pairedFixedReduction!=null&&pairedFixedReduction.Bindings[3].Variables.Length==1&&pairedFixedReduction.Stars.Any(s=>s.FixedHeight==57),
            "A fixed paired boundary was hidden inside a common parent");
        var branchRows=new SparseProfileRows(20);for(var i=0;i<20;i++)branchRows.Append(new[]{i},new[]{1d},1,50,1,ProfileRowKind.EqualityCost);branchRows.Freeze();
        var branchStars=stars.Select((s,i)=>i==3?new ProfileStar(s.Height,s.Controls.Append(19).ToArray(),s.Alpha.Append(20).ToArray(),s.Groups.Append(0).ToArray()):s).ToArray();
        var branched=NetworkProfileParents.Reduce(sections,from.ToArray(),c1.ToArray(),c2.ToArray(),to.ToArray(),branchRows,
            Enumerable.Repeat(50d,20).ToArray(),branchStars,100,new ImportLog(),default);
        Check(branched!=null&&branched.Bindings[3].Variables.Length==1,"An attached branch was absorbed into a paired profile parent");
        var mixed=ordinary.Select((s,i)=>i==2?s with{Tunnel=true}:s).ToArray();
        var mixedReduction=NetworkProfileParents.Reduce(mixed,nativeFrom,nativeFirst,nativeSecond,nativeTo,raw,
            Enumerable.Repeat(50d,19).ToArray(),ordinaryStars.ToArray(),100,new ImportLog(),default);
        Check(mixedReduction!=null&&mixedReduction.Bindings[2].Variables.Length==1&&mixedReduction.Bindings[3].Variables.Length==1,
            "A structure-mode boundary was absorbed into an ordinary profile parent");
    }
}
