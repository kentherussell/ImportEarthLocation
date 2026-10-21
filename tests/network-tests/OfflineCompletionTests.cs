using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

static class OfflineCompletionTests
{
    static void Check(bool value,string message){if(!value)throw new Exception(message);}
    static PreparedSegment Line(long way,long a,long b,float3 p,float3 q,NetworkKind kind=NetworkKind.Road)=>
        new(way,a,b,p,q,kind,kind==NetworkKind.Rail?"rail":"primary",2,kind==NetworkKind.Road,false,false,0)
        {NativeWidth=12,NativeAssetKey="fixture",SourceGeometry=new[]{p,q},NativeCenterlineCurvatureLimit=kind==NetworkKind.Rail?.03:null};
    static Cubic3d Curve(PreparedSegment s)=>new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);
    public static void Run()
    {
        CapturedProblemAudit.SelfTest();
        CapturedNativePhysicalReplay.SelfTest();
        var remap=new Dictionary<(int,long),(long node,float3 point)>
        {
            [(0,1)]=(2,new(2,0,0)),[(0,2)]=(3,new(3,0,0)),
            [(0,4)]=(5,new(5,0,0)),[(0,5)]=(4,new(4,0,0)),[(0,6)]=(5,new(5,0,0)),
            [(1,1)]=(7,new(7,0,0))
        };
        RailCorridorConsolidator.ResolveJunctions(remap);
        Check(remap[(0,1)].node==3&&remap[(0,2)].node==3&&remap[(0,4)].node==4&&remap[(0,5)].node==4&&
            remap[(0,6)].node==4&&math.all(remap[(0,6)].point==new float3(4,0,0))&&remap[(1,1)].node==7,
            "Rail geometry remapping disagreed across a chain, equivalence cycle, or layer boundary");
        var frozenFirst=Line(899,8990,8991,new(-100,50,0),new(0,50,0));
        var frozenSecond=Line(899,8991,8992,frozenFirst.End,new(100,50,0));
        frozenFirst=frozenFirst with{FinalDeckCurve=new(frozenFirst.CourseStart,frozenFirst.CourseControl1,frozenFirst.CourseControl2,frozenFirst.CourseEnd)};
        frozenSecond=frozenSecond with{FinalDeckCurve=new(frozenSecond.CourseStart,frozenSecond.CourseControl1,frozenSecond.CourseControl2,frozenSecond.CourseEnd)};
        var compactFrozen=NativeChainCompactor.Prepare(new[]{frozenFirst,frozenSecond},new ImportLog(),default);
        Check(compactFrozen.Count==1&&math.all(compactFrozen[0].CourseEnd==frozenSecond.CourseEnd),
            "Shared fitter inherited the first frozen deck instead of emitting the merged span");
        var steepTerrain=Enumerable.Range(0,17*17).Select(i=>(ushort)Math.Round((i%17)/16d*65535)).ToArray();
        var steepGround=Line(900,9000,9001,new(-100,50,0),new(100,50,0));
        var infeasible=NetworkGroundFeasibility.Check(new[]{steepGround},steepTerrain,200,200,default);
        Check(infeasible.Conflicts==1&&infeasible.MaximumDeficitMeters>100&&
            NetworkGroundFeasibility.Check(new[]{steepGround with{Bridge=true}},steepTerrain,200,200,default).Checked==0,
            "Necessary ground/grade preflight missed an independently impossible span or treated a bridge as ground earthwork");
        var groundChain=Enumerable.Range(0,6).Select(i=>Line(901,9100+i,9101+i,new(-30+10*i,50,0),new(-20+10*i,50,0))).ToArray();
        Check(groundChain.All(s=>NetworkGroundFeasibility.Check(new[]{s},steepTerrain,200,200,default).Conflicts==0),
            "Chain fixture must not already fail the single-span endpoint condition");
        var chainConflict=NetworkGroundFeasibility.Check(groundChain,steepTerrain,200,200,default);
        var reversedConflict=NetworkGroundFeasibility.Check(groundChain.Reverse().Select(NetworkNormalizer.Reverse).ToArray(),steepTerrain,200,200,default);
        Check(chainConflict.Conflicts==2&&chainConflict.MaximumDeficitMeters>5&&chainConflict.MaximumDeficitMeters<5.2&&reversedConflict.Conflicts==chainConflict.Conflicts&&
            Math.Abs(reversedConflict.MaximumDeficitMeters-chainConflict.MaximumDeficitMeters)<1e-5,
            $"Bounded chain reachability missed accumulated incompatibility or depended on direction/enumeration: {chainConflict} / {reversedConflict}");
        var flatScreenTerrain=Enumerable.Repeat((ushort)32768,17*17).ToArray();
        var offsetChain=new[]{groundChain[0] with{StartCurveOffset=new(0,50,0),EndCurveOffset=new(0,50,0)},
            groundChain[1] with{StartCurveOffset=new(0,-50,0),EndCurveOffset=new(0,-50,0)}};
        Check(offsetChain.All(s=>NetworkGroundFeasibility.Check(new[]{s},flatScreenTerrain,200,200,default).Conflicts==0)&&
            NetworkGroundFeasibility.Check(offsetChain,flatScreenTerrain,200,200,default).Conflicts>0,
            "Ground intervals ignored distinct course offsets on a common node anchor");
        var cycle=new[]{Line(901,9200,9201,new(0,50,0),new(10,50,0)),Line(901,9201,9202,new(10,50,0),new(10,50,10)),
            Line(901,9202,9200,new(10,50,10),new(0,50,0))};
        var cycleScreen=NetworkGroundFeasibility.Check(cycle,flatScreenTerrain,200,200,default);
        var reversedCycle=NetworkGroundFeasibility.Check(cycle.Reverse().Select(NetworkNormalizer.Reverse).ToArray(),flatScreenTerrain,200,200,default);
        Check(cycleScreen.Conflicts==0&&cycleScreen.Unavailable==3&&cycleScreen==reversedCycle,
            "Cycle screening used an arbitrary seam or hid unresolved coverage");
        var monotoneProfile=NetworkCurveObservations.Measure(new(new(-120,0,0),new(-40,4,0),new(40,8,0),new(120,12,0)));
        var oscillatingProfile=NetworkCurveObservations.Measure(new(new(-120,0,0),new(-40,20,0),new(40,-20,0),new(120,0,0)));
        Check(monotoneProfile.SampledGradeSignChanges==0&&monotoneProfile.SampledSignedGradeVariation<1e-12&&
            oscillatingProfile.SampledGradeSignChanges==2&&oscillatingProfile.SampledSignedGradeVariation>.1&&
            oscillatingProfile.SampledMaximumGradeChangePerMeter>monotoneProfile.SampledMaximumGradeChangePerMeter,
            "Profile shape observations lost oscillation or flagged constant-grade variation");
        var shortRailA=Line(902,9020,9022,new(18,50,0),new(0,50,0),NetworkKind.Rail) with{Bridge=true,Layer=1,Reference="930000"};
        var shortRailB=Line(903,9021,9022,new(18,50,4),new(0,50,0),NetworkKind.Rail) with{Bridge=true,Layer=1,Reference="930000"};
        var shortRailLog=new ImportLog();var shortRail=RailCorridorConsolidator.Prepare(new[]{shortRailA,shortRailB},shortRailLog,default);
        Check(shortRail.Count==1&&shortRailLog.Metric("Rail parallel spans consolidated")>0,
            "A source-connected short structural pair remained two overlapping native double-track corridors");
        var unrelated=RailCorridorConsolidator.Prepare(new[]{shortRailA,shortRailB with{Reference="other"}},new ImportLog(),default);
        Check(unrelated.Count==2,"Short-pair consolidation used proximity without matching route evidence");
        var tinySwitch=new[]{
            Line(940,9400,9401,new(-100,50,0),new(0,50,0),NetworkKind.Rail),
            Line(941,9401,9402,new(0,50,0),new(.25f,50,0),NetworkKind.Rail),
            Line(942,9402,9403,new(.25f,50,0),new(100,50,0),NetworkKind.Rail),
            Line(943,9401,9404,new(0,50,0),new(-50,50,100),NetworkKind.Rail),
            Line(944,9402,9405,new(.25f,50,0),new(50,50,-100),NetworkKind.Rail)};
        var switchLog=new ImportLog();var switchResult=RailCorridorConsolidator.Prepare(tinySwitch,switchLog,default);
        var switchTopology=new ImportLog();SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(tinySwitch),switchResult,switchTopology,default);
        Check(switchLog.Metric("Sub-footprint connected rail links consolidated")==1&&switchResult.All(s=>math.distance(s.Start,s.End)>3)&&
            switchTopology.Metric("Missing mandatory selected-source ports")==0&&switchTopology.Metric("Unresolved selected-source reachability obligations")==0,
            "Connected sub-footprint rail preprocessing lost a switch port/route or retained the tiny solver edge");
        var portalTiny=tinySwitch.Select(s=>s.WayId==942?s with{Bridge=true}:s).ToArray();
        var portalTinyLog=new ImportLog();RailCorridorConsolidator.Prepare(portalTiny,portalTinyLog,default);
        Check(portalTinyLog.Metric("Sub-footprint connected rail links consolidated")==0,"Short-link preprocessing collapsed a structural portal");
        var isolatedTiny=RailCorridorConsolidator.Prepare(new[]{tinySwitch[1]},new ImportLog(),default);
        Check(isolatedTiny.Count==1,"Short-link preprocessing deleted an isolated selected source component");
        var untouchedRail=Line(945,9450,9451,new(1000,50,0),new(1100,50,0),NetworkKind.Rail) with
            {Control1=new float3(1030,50,10),SourceGeometry=new[]{new float3(1000,50,0),new float3(1050,50,5),new float3(1100,50,0)}};
        var preservedRail=RailCorridorConsolidator.Prepare(tinySwitch.Append(untouchedRail).ToArray(),new ImportLog(),default).Single(s=>s.WayId==945);
        Check(preservedRail.SourceGeometry.SequenceEqual(untouchedRail.SourceGeometry),"Short-link preprocessing replaced unaffected source geometry");
        var portalFan=new[]{
            Line(910,9100,9101,new(0,50,0),new(100,50,0),NetworkKind.Rail),
            Line(911,9110,9111,new(0,50,4),new(100,50,4),NetworkKind.Rail),
            Line(912,9120,9121,new(100,50,8),new(200,50,8),NetworkKind.Rail) with{Bridge=true,Layer=1},
            Line(913,9111,9131,new(100,50,4),new(200,50,4),NetworkKind.Rail) with{Bridge=true,Layer=1}
        };
        var portalResult=RailCorridorConsolidator.Prepare(portalFan,new ImportLog(),default);
        var portalAudit=new ImportLog();SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(portalFan),portalResult,portalAudit,default);
        Check(portalAudit.Metric("Missing mandatory selected-source ports")==0&&portalAudit.Metric("Unresolved selected-source reachability obligations")==0,
            "Independent mode/layer folds split one shared source portal into incompatible canonical junctions");
        var equivalentA=Line(920,9200,9201,new(0,50,0),new(100,50,0),NetworkKind.Rail) with{SourceWayIds=new[]{920L}};
        var equivalentB=equivalentA with{WayId=921,SourceWayIds=new[]{921L},SourceNodeAliases=new[]{new SourceNodeAlias(9210,9200,true)}};
        var equivalentResult=NetworkNormalizer.Normalize(new[]{equivalentA,equivalentB},new ImportLog(),default);
        Check(equivalentResult.Count==1&&equivalentResult[0].SourceWayIds.OrderBy(id=>id).SequenceEqual(new[]{920L,921L})&&
            equivalentResult[0].SourceNodeAliases.Contains(new SourceNodeAlias(9210,9200,true))&&Curve(equivalentResult[0])==Curve(equivalentA),
            "Normalization discarded equivalent-course route provenance or changed the retained curve");
        var bendA=Line(901,9010,9011,new(-200,50,0),new(0,50,5),NetworkKind.Rail);
        var bendB=Line(901,9011,9012,bendA.End,new(200,50,0),NetworkKind.Rail);
        var bendTerrain=Enumerable.Repeat((ushort)16384,17*17).ToArray();
        var bend=CoordinatedNetworkDesign.Prepare(new[]{bendA,bendB},bendTerrain,200,1000,new ImportLog(),default,structureIntervalsResolved:true);
        Check(bend.SelectMany(s=>new[]{s.CourseStart.z,s.CourseControl1.z,s.CourseControl2.z,s.CourseEnd.z}).Max()>=2,
            "Route straightening exceeded the shared rail source-fit budget before later fitting");
        var railIn=Line(1,1,2,new(-200,50,0),new(0,50,0),NetworkKind.Rail);
        var railOut=Line(1,2,3,railIn.End,new(200,50,10),NetworkKind.Rail);
        var railBranch=Line(2,2,4,railIn.End,new(10,50,200),NetworkKind.Rail);
        var fan=new[]{railIn,railOut,railBranch};var failed=new HashSet<int>();
        var retained=NetworkApproachFrames.Prepare(fan,new[]{new NetworkApproachTapers.DirectionRequest(0,false,new(-1,0)),
            new NetworkApproachTapers.DirectionRequest(1,true,new(1,0)),new NetworkApproachTapers.DirectionRequest(2,true,new(1,0))},new ImportLog(),default,failed);
        Check(failed.Count==3&&retained.Zip(fan,(a,b)=>Curve(a)==Curve(b)).All(v=>v),"One failed turnout arm left a partially published fan");

        var terrain=Enumerable.Repeat((ushort)16384,17*17).ToArray();
        var frozenA=Line(3,10,11,new(-100,50,0),new(0,50,0)) with{Control2=new(-33,49,0)};
        var frozenB=Line(3,11,12,new(0,50,0),new(100,50,0)) with{Control1=new(33,52,0)};
        var free=Line(4,20,21,new(-100,100,100),new(100,100,100));
        var solved=NetworkTunnelProfile.Solve(new[]{frozenA,frozenB,free},200,Array.Empty<(int,float,float)>(),new ImportLog(),default,
            design:new(terrain,2000),frozenSections:new HashSet<int>{0,1});
        Check(Curve(solved[0])==Curve(frozenA)&&Curve(solved[1])==Curve(frozenB)&&solved[2].Start.y<99,
            "A pre-existing wholly frozen grade disagreement prevented unrelated reconstruction or moved frozen controls");
        var roundedA=frozenA with{Start=new(-100,495,0),End=new(0,489.89538f,0),Control2=new(-36.99385f,491.47461f,0)};
        var roundedB=frozenB with{Start=roundedA.End,End=new(100,485,0),Control1=new(36.99385f,488.3161f,0)};
        var freeArm=roundedB with{WayId=41,EndNodeId=22};var roundedLog=new ImportLog();
        var rounded=NetworkTunnelProfile.Solve(new[]{roundedA,roundedB,freeArm},1000,Array.Empty<(int,float,float)>(),roundedLog,default,
            design:new(terrain,2000),frozenSections:new HashSet<int>{0,1});
        Check(Curve(rounded[0])==Curve(roundedA)&&Curve(rounded[1])==Curve(roundedB)&&
            roundedLog.Metric("Frozen float-resolution grade observations retained independently")>0,
            "Float-rounded frozen grades refused local reconstruction or immutable controls moved");
        try
        {
            NetworkTunnelProfile.Solve(new[]{roundedA,roundedB with{Control1=new(36.99385f,480,0)},freeArm},1000,
                Array.Empty<(int,float,float)>(),new ImportLog(),default,design:new(terrain,2000),frozenSections:new HashSet<int>{0,1});
            throw new Exception("A genuine frozen grade conflict was treated as float rounding");
        }
        catch(InvalidOperationException e) when(e.Message.Contains("conflicts with a shared grade group")){}

        var a=Line(5,30,31,new(-100,55,0),new(100,55,0)) with{PairedProfileId=1,PairedReferenceBegin=0,PairedReferenceEnd=200};
        var b=Line(6,32,33,new(100,55,40),new(-100,55,40)) with{PairedProfileId=1,PairedReversed=true,PairedReferenceBegin=200,PairedReferenceEnd=0,Bridge=true};
        var raster=Enumerable.Repeat((ushort)16384,257*257).ToArray();var formationLog=new ImportLog();
        var stamped=NetworkRoadbed.Prepare(raster,200,512,new[]{a,b},Array.Empty<float3[]>(),Array.Empty<float3[]>(),formationLog,default);
        Check(Math.Abs(NetworkProcessor.Sample(0,0,stamped,200,512)-55)<.02&&Math.Abs(NetworkProcessor.Sample(0,16,stamped,200,512)-55)<.02,
            "Mixed formation failed to support its ground core and shared median half");
        Check(NetworkProcessor.Sample(0,40,stamped,200,512)==NetworkProcessor.Sample(0,40,raster,200,512),"Mixed ground formation filled beneath its structural carriageway");
        Check(formationLog.Lines.Any(l=>l.Contains("sharedMixedModeHalfFormations=1")),"Mixed formation was silently treated as independent ground stamps");

        var pairLog=new ImportLog();
        NetworkObservedQuality.Paired(new[]{a,b},s=>new[]{s==b?Curve(s) with{P0=Curve(s).P0+new double3(0,3,0),P1=Curve(s).P1+new double3(0,3,0),
            P2=Curve(s).P2+new double3(0,3,0),P3=Curve(s).P3+new double3(0,3,0)}:Curve(s)},pairLog,default,"fixture");
        Check(pairLog.Findings.Single().Failed==1&&pairLog.Findings.Single().Worst>2.99,"Qualified native paired-height drift was not measured");
        var missing=new ImportLog();NetworkObservedQuality.Paired(new[]{a,b},s=>s==b?null:new[]{Curve(s)},missing,default,"fixture");
        Check(missing.Findings.Single().State=="Unavailable","Missing native pairing evidence passed acceptance");

        var root=Line(7,40,41,new(-100,50,0),new(0,50,0)) with{StartTerminal=true};
        var arm1=Line(8,41,42,root.End,new(100,50,30));var arm2=Line(9,41,43,root.End,new(100,50,-30));
        var topology=SourceTopologyObligations.Capture(new[]{root,arm1,arm2});var ports=new ImportLog();
        NetworkObservedQuality.Ports(topology,new[]{root},_=>1,ports,default,"planned","fixture");
        Check(ports.Findings.Single(f=>f.Category=="nonterminal-orphan-ports").Failed==1&&ports.Findings.Single(f=>f.Category=="declared-terminal-ports").Checked==1,
            "Lost selected junction and legitimate terminal were conflated");

        var rowObservations=new List<double>();var values=new[]{9d,1d};
        var measured=SparseProfileRows.ForMeasurement(2,(indices,coefficients,count,target)=>
        {var residual=-target;for(var j=0;j<count;j++)residual+=coefficients[j]*values[indices[j]];rowObservations.Add(residual);});
        var packed=new SparseProfileRows(2);
        foreach(var weight in new[]{2d,3d})foreach(var rows in new[]{measured,packed})
            rows.Append(new[]{0,1},new[]{1d,-2d},2,3,weight,ProfileRowKind.UpperHingeCost,coalescePrevious:true);
        Check(measured.Count==packed.Count&&measured.Terms==packed.Terms&&measured.CapacityBytes==0&&rowObservations.SequenceEqual(new[]{4d}),
            "Streaming fixed-row measurement changed canonical coverage or retained a second packed matrix");
        measured.Freeze();var refused=false;try{measured.Evaluate(values,null,default);}catch(InvalidOperationException){refused=true;}
        Check(refused,"Fixed-row observation storage was accepted as an alternative optimizer input");

        var lower=Line(10,50,51,new(-100,50,0),new(100,50,0));
        var upper=Line(11,52,53,new(0,59,-100),new(0,59,100)) with{Bridge=true,Layer=1};
        var contract=new NativeAssetContract("fixture",new[]{new NativeCompositionEnvelope(NativeStructureMode.Ground,0,12,0,0,0,0,4,true),
            new NativeCompositionEnvelope(NativeStructureMode.Elevated,0,12,0,0,-2,0,4,true)},Array.Empty<float>());
        // Derived from Marseille ways 417386065/080/073/038: short parallel
        // embankment/bridge intervals converge at the same canonical rail node.
        PreparedSegment Rail(long way,long start,long end,float2 p,float2 q,bool bridge)=>
            Line(way,start,end,new(p.x,50,p.y),new(q.x,50,q.y),NetworkKind.Rail) with{Bridge=bridge,Layer=1,Reference="930000"};
        var abutment=new[]{
            Rail(417386065,4179435005,4179435015,new(8658.923f,-7133.16357f),new(8629.792f,-7120.23975f),false),
            Rail(417386080,4179435007,4179435018,new(8660.178f,-7129.468f),new(8630.763f,-7116.577f),false),
            Rail(417386073,4179435015,4179435022,new(8629.792f,-7120.23975f),new(8617.088f,-7110.265f),true),
            Rail(417386038,4179435018,4179435022,new(8630.763f,-7116.577f),new(8617.088f,-7110.265f),true)};
        var abutmentBefore=NetworkTunnelProfile.MeasurePhysical(abutment,terrain,200,57344,new Dictionary<string,NativeAssetContract>{{"fixture",contract}},default);
        var abutmentAfter=RailCorridorConsolidator.Prepare(abutment,new ImportLog(),default);
        var abutmentQuality=NetworkTunnelProfile.MeasurePhysical(abutmentAfter,terrain,200,57344,new Dictionary<string,NativeAssetContract>{{"fixture",contract}},default);
        Check(abutmentBefore.ClearanceFailures>0&&abutmentAfter.Count==2&&abutmentQuality.ClearanceFailures==0,
            "Source-connected short rail formation still demanded artificial vertical stacking at its abutment");
        var abutmentTopology=new ImportLog();SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(abutment),abutmentAfter,abutmentTopology,default);
        Check(abutmentTopology.Metric("Missing mandatory selected-source ports")==0&&abutmentTopology.Metric("Unresolved selected-source reachability obligations")==0,
            "Short rail formation removed a required source port or route obligation");
        var evidence=new List<NetworkPhysicalContacts.ContactEvidence>();var physicalRows=new SparseProfileRows(8);
        var report=NetworkPhysicalContacts.Append(new[]{lower,upper},new Dictionary<string,NativeAssetContract>{{"fixture",contract}},new[]{0,4},new[]{1,5},new[]{2,6},new[]{3,7},physicalRows,.05,default,evidence:evidence.Add);
        var mainlineA=lower with{Classification="motorway",Reference="I-95"};
        var mainlineB=Line(930,9300,9301,new(100,59,4),new(-100,59,4)) with{Classification="motorway",Reference="I-95",Bridge=true};
        NetworkPhysicalContacts.Report Contact(PreparedSegment other)
        {
            var rows=new SparseProfileRows(8);
            return NetworkPhysicalContacts.Append(new[]{mainlineA,other},new Dictionary<string,NativeAssetContract>{{"fixture",contract}},
                new[]{0,4},new[]{1,5},new[]{2,6},new[]{3,7},rows,.05,default);
        }
        var parallelContact=Contact(mainlineB);
        Check(parallelContact.Rows==0&&parallelContact.HorizontalConflictPairs>0,
            "Unpaired same-layer opposing mainlines were vertically stacked across a composition boundary");
        Check(Contact(mainlineB with{Layer=1}).Rows>0&&Contact(mainlineB with{Reference="other"}).Rows>0&&
            Contact(mainlineB with{Start=new(100,59,8),End=new(-100,59,-8)}).Rows>0,
            "The opposing-mainline rule removed explicit-level, unrelated-route or shallow true-crossing clearance");
        var sideFlags=mainlineA with{ExplicitWallSides=NetworkSides.Left,RetainingWallSides=NetworkSides.Right,QuaySides=NetworkSides.Left};
        var noSides=StarterNetworkPolicy.WithoutSideStructures(new[]{sideFlags}).Single();
        Check(noSides.ExplicitWallSides==NetworkSides.None&&noSides.RetainingWallSides==NetworkSides.None&&noSides.QuaySides==NetworkSides.None&&
            Curve(noSides)==Curve(sideFlags)&&noSides.StartNodeId==sideFlags.StartNodeId&&noSides.EndNodeId==sideFlags.EndNodeId,
            "Disabling manual walls/quays reshaped or subdivided the solved deck");
        Check(report.Rows>0&&evidence.Sum(e=>e.RowCount)==report.Rows&&evidence.Where(e=>e.RowCount>0).All(e=>e.FirstLeaf>=0&&e.SecondLeaf>=0&&e.Classification=="independent-source-ordered-levels"),
            "Physical row ranges lost their frozen contact/source identity");
        var measuredOrdinal=0L;
        var fixedRows=SparseProfileRows.ForMeasurement(8,(indices,coefficients,count,target)=>
        {
            var expected=physicalRows.Row(measuredOrdinal++);
            Check(count==expected.Count&&target==expected.Target,"One-pass physical measurement changed a canonical inequality");
            for(var j=0;j<count;j++)Check(indices[j]==physicalRows.Index(expected.Offset+j)&&coefficients[j]==physicalRows.Coefficient(expected.Offset+j),
                "One-pass physical measurement changed coefficient order or value");
        });
        var fixedReport=NetworkPhysicalContacts.Append(new[]{lower,upper},new Dictionary<string,NativeAssetContract>{{"fixture",contract}},
            new[]{0,4},new[]{1,5},new[]{2,6},new[]{3,7},fixedRows,.05,default);
        Check(measuredOrdinal==physicalRows.Count&&fixedReport.Rows==report.Rows&&fixedReport.Regions==report.Regions&&fixedRows.CapacityBytes==0,
            "One-pass physical measurement omitted coverage or allocated a packed matrix");
        var sameLevel=new[]{lower,upper with{Bridge=false,Layer=0}};var observedRows=SparseProfileRows.ForMeasurement(8,(_,_,_,_)=>{});
        var overlapReport=NetworkPhysicalContacts.Append(sameLevel,new Dictionary<string,NativeAssetContract>{{"fixture",contract}},
            new[]{0,4},new[]{1,5},new[]{2,6},new[]{3,7},observedRows,.05,default);
        Check(overlapReport.HorizontalConflictPairs>0&&overlapReport.Rows==0,"One-pass certification lost same-level conflict observations");
        // Curved, clamped and locally connected footprints exercise clipping
        // and formation subtraction, not just one straight crossing polygon.
        foreach(var curved in new[]{false,true})foreach(var connected in new[]{false,true})
        {
            var lo=curved?lower with{Control1=new(-33,50,18),Control2=new(33,50,-18)}:lower;
            var hi=curved?upper with{Control1=new(-18,59,-33),Control2=new(18,59,33)}:upper;
            if(connected)hi=hi with{StartNodeId=lo.StartNodeId,Start=lo.Start,Control1=lo.Start+new float3(25,3,8)};
            var members=new[]{lo,hi};var contracts=new Dictionary<string,NativeAssetContract>{{"fixture",contract}};
            var stored=new SparseProfileRows(8);var storedReport=NetworkPhysicalContacts.Append(members,contracts,new[]{0,4},new[]{1,5},new[]{2,6},new[]{3,7},stored,.05,default);
            long ordinal=0;
            var observation=SparseProfileRows.ForMeasurement(8,(indices,coefficients,count,target)=>
            {
                var row=stored.Row(ordinal++);Check(row.Count==count&&row.Target==target,"Curved/formation measurement changed its target");
                for(var j=0;j<count;j++)Check(indices[j]==stored.Index(row.Offset+j)&&coefficients[j]==stored.Coefficient(row.Offset+j),"Curved/formation measurement changed a coefficient");
            });
            var observed=NetworkPhysicalContacts.Append(members,contracts,new[]{0,4},new[]{1,5},new[]{2,6},new[]{3,7},observation,.05,default);
            Check(ordinal==stored.Count&&observed.Rows==storedReport.Rows&&observed.HorizontalConflictPairs==storedReport.HorizontalConflictPairs&&
                observed.GradeContradictionPairs==storedReport.GradeContradictionPairs&&observed.CandidateTests*2==storedReport.CandidateTests,
                "One-pass certification changed physical coverage/classification or retained the redundant candidate pass");
        }
        Console.WriteLine("PASS: atomic turnout publication, immutable inconsistent context, mixed formation support, actual paired drift, orphan/terminal classification, streaming physical rows and complete contact row provenance.");
    }
}
