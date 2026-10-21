using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

static class PairedCorridorTests
{
    static void Check(bool value,string message){if(!value)throw new Exception(message);}
    static PreparedSegment Road(long way,long start,long end,float3 a,float3 b)=>new(way,start,end,a,b,NetworkKind.Road,"motorway",2,true,false,false,0){NativeWidth=16,CorridorId=50,NativeAssetKey="two"};
    public static void Run()
    {
        var a=Road(1,1,2,new(-200,90,4),new(200,110,4));var b=Road(2,3,4,new(200,120,-4),new(-200,100,-4));
        var log=new ImportLog();var paired=NetworkPairedCorridors.Prepare(new[]{a,b},log,default);
        var course=new Cubic3d(new(0,0,0),new(100d/3,0,0),new(200d/3,0,0),new(100,0,0));
        Cubic3d Translate(double z){var shift=new double3(2,0,z);return new(course.P0+shift,course.P1+shift,course.P2+shift,course.P3+shift);}
        Check(!NetworkPairedCorridors.Separated(course,Translate(2),2.5,default)&&NetworkPairedCorridors.Separated(course,Translate(3),2.5,default),
            "Spacing certificate checked only equal stations and missed a diagonal nearest-point overlap");
        var repairLog=new ImportLog();
        var repaired=NetworkPairedCorridors.Prepare(new[]{a with{SourceFitToleranceMeters=2},b with{SourceFitToleranceMeters=2}},repairLog,default,
            width:2000,protectedAreas:Array.Empty<float3[]>(),protectedLines:Array.Empty<float3[]>());
        Check(repaired.All(s=>s.PairedSpanId>0&&!s.PairedProfileOnly),"Coordinated repair retained a known source overlap");
        var localA=a with{NativeLaneOffsets=new[]{-4f,4f}};var localB=b with{NativeLaneOffsets=new[]{-4f,4f}};
        var distant=localA with{StartNodeId=localA.EndNodeId,EndNodeId=9,Start=localA.End,End=new(600,110,4),
            NativeWidth=48,NativeLaneOffsets=new[]{-20f,-12f,-4f,4f,12f,20f}};
        var localPair=NetworkPairedCorridors.Prepare(new[]{localA,localB},new ImportLog(),default,width:2000,
            protectedAreas:Array.Empty<float3[]>(),protectedLines:Array.Empty<float3[]>());
        var extendedPair=NetworkPairedCorridors.Prepare(new[]{localA,localB,distant},new ImportLog(),default,width:2000,
            protectedAreas:Array.Empty<float3[]>(),protectedLines:Array.Empty<float3[]>());
        var distantReverse=localB with{StartNodeId=localB.EndNodeId,EndNodeId=10,Start=localB.End,End=new(-600,100,-4),
            NativeWidth=48,NativeLaneOffsets=new[]{-20f,-12f,-4f,4f,12f,20f}};
        var reverseExtendedPair=NetworkPairedCorridors.Prepare(new[]{distantReverse,localB,localA},new ImportLog(),default,width:2000,
            protectedAreas:Array.Empty<float3[]>(),protectedLines:Array.Empty<float3[]>());
        Check(localPair.All(s=>s.PairedProfileId!=0&&!s.PairedProfileOnly)&&
            extendedPair.Where(s=>s.PairedProfileId!=0).All(s=>!s.PairedProfileOnly)&&
            reverseExtendedPair.Where(s=>s.PairedProfileId!=0).All(s=>!s.PairedProfileOnly),
            "Interval-local datum isolation retained an uncertified pair instead of its separated formation");
        double2 PairedAtX(IReadOnlyList<PreparedSegment> segments,long way)
        {
            var section=segments.First(s=>s.WayId==way&&s.PairedProfileId!=0&&
                Math.Min(s.CourseStart.x,s.CourseEnd.x)<=0&&Math.Max(s.CourseStart.x,s.CourseEnd.x)>=0);
            var t=-section.CourseStart.x/(section.CourseEnd.x-section.CourseStart.x);
            return NetworkDesignChecks.Evaluate(new(section.CourseStart,section.CourseControl1,section.CourseControl2,section.CourseEnd),t).xz;
        }
        foreach(var road in new[]{localA,localB})
        {
            Check(math.distance(PairedAtX(localPair,road.WayId),PairedAtX(extendedPair,road.WayId))<.001,
                "A lane-width event outside the reciprocal interval displaced its local paired datum");
            Check(math.distance(PairedAtX(localPair,road.WayId),PairedAtX(reverseExtendedPair,road.WayId))<.001,
                "A reversed partner's distant lane-width event displaced the local paired datum");
        }
        var widthEvent=new[]{
            localA with{EndNodeId=11,End=new(-5,100,4)},
            localA with{StartNodeId=11,EndNodeId=12,Start=new(-5,100,4),End=new(5,100,4),
                NativeWidth=24,NativeLaneOffsets=new[]{-8f,-4f,0f,4f,8f}},
            localA with{StartNodeId=12,Start=new(5,100,4)},localB};
        var widthPair=NetworkPairedCorridors.Prepare(widthEvent,new ImportLog(),default,width:2000,
            protectedAreas:Array.Empty<float3[]>(),protectedLines:Array.Empty<float3[]>());
        Check(widthPair.All(s=>s.PairedProfileId!=0&&!s.PairedProfileOnly)&&widthPair.Any(s=>s.NativeWidth==24),
            "A short real width event inside the pair was ignored or lost its certified formation");
        var through=widthPair.Where(s=>s.WayId==localA.WayId).ToArray();
        foreach(var incoming in through)foreach(var outgoing in through.Where(s=>s.StartNodeId==incoming.EndNodeId))
        {
            var aLane=incoming.CourseEnd.z-incoming.NativeLaneOffsets.Min();
            var bLane=outgoing.CourseStart.z-outgoing.NativeLaneOffsets.Min();
            Check(Math.Abs(aLane-bLane)<=.05&&math.all(incoming.End==outgoing.Start),
                "An in-interval width event broke the continuing lane or canonical anchor");
        }
        var longFormation=widthEvent.Select((s,i)=>i==0?s with{Start=new(-1000,100,4)}:
            i==2?s with{End=new(1000,100,4)}:i==3?s with{Start=new(1000,100,-4),End=new(-1000,100,-4)}:s).ToArray();
        var freeFormation=NetworkPairedCorridors.Prepare(longFormation,new ImportLog(),default,width:4000,
            protectedAreas:Array.Empty<float3[]>(),protectedLines:Array.Empty<float3[]>());
        var branches=new[]{
            new PreparedSegment(100,localA.StartNodeId,101,longFormation[0].Start,new(-1200,100,100),NetworkKind.Road,"primary",2,false,false,false,0),
            new PreparedSegment(102,localA.StartNodeId,103,longFormation[0].Start,new(-1200,100,-100),NetworkKind.Road,"primary",2,false,false,false,0)};
        var joinedFormation=NetworkPairedCorridors.Prepare(longFormation.Concat(branches).ToArray(),new ImportLog(),default,width:4000,
            protectedAreas:Array.Empty<float3[]>(),protectedLines:Array.Empty<float3[]>());
        foreach(var road in new[]{localA,localB})
            Check(math.distance(PairedAtX(freeFormation,road.WayId),PairedAtX(joinedFormation,road.WayId))<.05,
                "A generated outer junction reapplied an independent correction to the selected paired lane datum");
        foreach(var ports in joinedFormation.SelectMany(s=>new[]{(s.StartNodeId,s.Start),(s.EndNodeId,s.End)}).GroupBy(p=>p.Item1))
            Check(ports.All(p=>math.all(p.Item2==ports.First().Item2)),
                "The paired owner did not publish one canonical anchor to its attached source remainder");
        Check(repairLog.Findings.Single(f=>f.Category=="paired-source-fidelity").Failed==1,
            "Formation repair silently certified the original two-metre source budget");
        var tails=NetworkPairedCorridors.Prepare(new[]{a with{Start=new(-400,90,4),End=new(400,110,4),SourceFitToleranceMeters=2},
            b with{SourceFitToleranceMeters=2}},new ImportLog(),default,width:2000,protectedAreas:Array.Empty<float3[]>(),protectedLines:Array.Empty<float3[]>());
        Check(tails.Count(s=>s.PairedProfileId==0)>=2&&tails.Where(s=>s.PairedProfileId!=0).All(s=>!s.PairedProfileOnly),
            "Independent source remainders pulled a repaired synthetic cut back into overlap");
        foreach(var port in tails.SelectMany(s=>new[]{(Node:s.StartNodeId,Point:s.Start),(Node:s.EndNodeId,Point:s.End)}).GroupBy(p=>p.Node))
            Check(port.All(p=>math.all(p.Point==port.First().Point)),"Paired cut and source remainder have inconsistent canonical anchors");
        var modeContract=new NativeAssetContract("two",new[]{
            new NativeCompositionEnvelope(NativeStructureMode.Ground,0,16,0,1,-2,2,8,true),
            new NativeCompositionEnvelope(NativeStructureMode.Elevated,0,24,0,1,-2,2,8,true)},new[]{-4f,0,4});
        var contracts=new Dictionary<string,NativeAssetContract>{{"two",modeContract}};
        var groundPair=NetworkPairedCorridors.Prepare(new[]{localA,localB},new ImportLog(),default,width:2000,
            protectedAreas:Array.Empty<float3[]>(),protectedLines:Array.Empty<float3[]>(),assets:contracts);
        var remoteBridgePair=NetworkPairedCorridors.Prepare(new[]{localA,localB,distantReverse with{Bridge=true,NativeWidth=16}},
            new ImportLog(),default,width:2000,protectedAreas:Array.Empty<float3[]>(),protectedLines:Array.Empty<float3[]>(),assets:contracts);
        foreach(var road in new[]{localA,localB})
            Check(math.distance(PairedAtX(groundPair,road.WayId),PairedAtX(remoteBridgePair,road.WayId))<.001,
                "A bridge composition outside the paired interval displaced a ground formation's lane datum");
        var modes=NetworkPairedCorridors.Prepare(new[]{a,b with{Bridge=true}},new ImportLog(),default,width:2000,
            protectedAreas:Array.Empty<float3[]>(),protectedLines:Array.Empty<float3[]>(),assets:new Dictionary<string,NativeAssetContract>{{"two",modeContract}});
        Check(modes.Where(s=>s.PairedProfileId!=0).All(s=>s.Bridge&&s.NativeWidth==24&&!s.PairedProfileOnly),
            "Paired native composition widened after the spacing decision or bridge policy was not reconciled");
        foreach(var group in repaired.GroupBy(s=>s.PairedProfileId))
        {
            var sides=group.ToArray();var forward=sides.Single(s=>!s.PairedReversed);var reverse=sides.Single(s=>s.PairedReversed);
            Check(NetworkPairedCorridors.Separated(new(forward.CourseStart,forward.CourseControl1,forward.CourseControl2,forward.CourseEnd),
                new(reverse.CourseEnd,reverse.CourseControl2,reverse.CourseControl1,reverse.CourseStart),18,default),
                "Published float pair spent the physical two-metre median");
        }
        var cliff=new[]{new float3(-500,0,0),new float3(500,0,0)};
        var protection=new NetworkFormationProtection(2000,Array.Empty<float3[]>(),new[]{cliff},default);
        Cubic3d Straight(double z)=>new(new(-200,0,z),new(-70,0,z),new(70,0,z),new(200,0,z));
        Check(protection.Clear(Straight(50),8)&&!protection.Clear(Straight(10),8),
            "Actual protected-footprint certificate waived its native half-width plus twelve-metre margin");
        var protectedLog=new ImportLog();var inspected=NetworkPairedCorridors.Prepare(new[]{a with{SourceFitToleranceMeters=2},b with{SourceFitToleranceMeters=2}},protectedLog,default,
            width:2000,protectedAreas:Array.Empty<float3[]>(),protectedLines:new[]{cliff});
        Check(inspected.All(s=>!s.PairedProfileOnly)&&protectedLog.Findings.Single(f=>f.Category=="paired-protected-footprint").Failed==1,
            "Protected quality failure either hid the overlap or was silently certified");
        Check(log.Metric("Trusted paired intervals")==1&&paired.Count==2&&paired.All(s=>s.PairedSpanId>0),"A unique straight pair was not emitted as one synchronized shared family: "+string.Join(";",log.Lines));
        Check(paired[0].StartHeightFamily==paired[1].EndHeightFamily&&paired[0].EndHeightFamily==paired[1].StartHeightFamily&&paired[0].StartNodeId!=paired[1].EndNodeId,"Height-family sharing merged topology IDs or ignored reverse station orientation");
        var unitCuts=NativeDesignUnits.Prepare(new[]{paired[0],paired[1] with{NativeWidth=24}},Array.Empty<NativeAssetContract>(),new ImportLog(),default);
        double[] Stations(long way)=>unitCuts.Where(s=>s.WayId==way).SelectMany(s=>new[]{s.PairedReferenceBegin,s.PairedReferenceEnd}).Distinct().Order().ToArray();
        Check(Stations(1).Length>=3&&Stations(1).Zip(Stations(2),(x,y)=>Math.Abs(x-y)<1e-7).All(x=>x)&&Stations(1).Length==Stations(2).Length,
            "Asymmetric native core cuts desynchronized the shared reference profile: "+string.Join(",",Stations(1))+" / "+string.Join(",",Stations(2)));
        var reversed=NetworkNormalizer.Reverse(paired[0]);
        Check(math.all(reversed.CourseStart==paired[0].CourseEnd)&&math.all(reversed.CourseControl1==paired[0].CourseControl2)&&reversed.StartHeightFamily==paired[0].EndHeightFamily,"Reversal ignored final float curve or paired station lineage");
        var terrain=Enumerable.Repeat((ushort)32768,17*17).ToArray();
        var solved=NetworkTunnelProfile.Solve(paired,200,Array.Empty<(int,float,float)>(),new ImportLog(),default,design:new(terrain,2000));
        Check(solved[0].CourseStart.y==solved[1].CourseEnd.y&&solved[0].CourseControl1.y==solved[1].CourseControl2.y&&
            solved[0].CourseControl2.y==solved[1].CourseControl1.y&&solved[0].CourseEnd.y==solved[1].CourseStart.y,"Shared profile controls were not bit-identical after float publication");
        for(var i=0;i<=64;i++)
        {
            double Height(PreparedSegment s,double t)=>NetworkDesignChecks.Evaluate(new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd),t).y;
            Check(Math.Abs(Height(solved[0],i/64d)-Height(solved[1],1-i/64d))<1e-10,"Shared height cubics disagree in reference station");
        }
        var third=b with{WayId=3,StartNodeId=5,EndNodeId=6,Start=b.Start+new float3(0,0,-1),End=b.End+new float3(0,0,-1)};
        var ambiguous=NetworkPairedCorridors.Prepare(new[]{a,b,third},new ImportLog(),default);
        Check(ambiguous.All(s=>s.PairedSpanId==0)&&ambiguous.Count==3,"A stable sort fabricated a partner in an ambiguous collector corridor");
        var separated=NetworkPairedCorridors.Prepare(new[]{a,b with{Layer=1}},new ImportLog(),default);
        Check(separated.All(s=>s.PairedSpanId==0),"Different levels were paired into one height family");
        var splitA=new[]{a with{EndNodeId=7,End=new(0,100,4)},a with{StartNodeId=7,Start=new(0,100,4)}};
        var synchronized=NetworkPairedCorridors.Prepare(splitA.Concat(new[]{b}).ToArray(),new ImportLog(),default);
        Check(synchronized.Count(s=>s.PairedSpanId>0)==4&&synchronized.Where(s=>s.PairedSpanId>0).GroupBy(s=>s.PairedSpanId).All(g=>g.Count()==2),"Unequal source boundaries were not synchronized across the pair");
        // A shallow, constant-separation curved pair exercises the clamped
        // reference spline and analytical normal derivative, not chord matching.
        var ca=a with{Control1=new(-200f/3,90,20),Control2=new(200f/3,110,20)};
        var cb=b with{Control1=new(200f/3,120,12),Control2=new(-200f/3,100,12)};
        var curvedLog=new ImportLog();var curved=NetworkPairedCorridors.Prepare(new[]{ca,cb},curvedLog,default);
        Check(curvedLog.Metric("Trusted paired intervals")>0&&curved.Where(s=>s.PairedSpanId>0).GroupBy(s=>s.PairedSpanId).All(g=>g.Count()==2),"Valid shallow curves fell back to whole-edge chord pairing");
        Check(curved.Count<40,"Analysis samples leaked into native emission as a dense node chain");
        foreach(var span in curved.Where(s=>s.PairedSpanId>0).GroupBy(s=>s.PairedSpanId))
        {
            var members=span.ToArray();
            for(var j=0;j<=64;j++)
            {
                var distance=math.distance(NetworkNormalizer.Evaluate(members[0],j/64f).xz,NetworkNormalizer.Evaluate(members[1],1-j/64f).xz);
                Check(distance>=(members[0].NativeWidth+members[1].NativeWidth)/2+2-.001,
                    "Coarser internal paired fitting spent the physical two-metre median: "+string.Join(";",curvedLog.Lines));
            }
        }
        Check(curvedLog.Metric("Paired continuous projections excluded by hull bounds")>0,"Conservative pairing broad phase was not exercised");
        Console.WriteLine($"PAIR FIXTURE shallowCurveSections={curved.Count}; trustedIntervals={curvedLog.Metric("Trusted paired intervals")}; projectionCandidates={curvedLog.Metric("Paired continuous projection candidates")}; hullExcluded={curvedLog.Metric("Paired continuous projections excluded by hull bounds")}");
        var reverseInput=NetworkPairedCorridors.Prepare(new[]{b,a},new ImportLog(),default).OrderBy(s=>s.WayId).ThenBy(s=>s.PairedReferenceBegin).ToArray();
        var ordered=paired.OrderBy(s=>s.WayId).ThenBy(s=>s.PairedReferenceBegin).ToArray();
        Check(reverseInput.Length==ordered.Length&&reverseInput.Zip(ordered,(x,y)=>math.all(x.CourseStart==y.CourseStart)&&math.all(x.CourseEnd==y.CourseEnd)).All(x=>x),"Source enumeration order changed the trusted corridor geometry");
        var nearBoundary=b with{Start=b.Start+new float3(.005f,0,0)};
        var bounded=NetworkPairedCorridors.Prepare(new[]{a,nearBoundary},new ImportLog(),default);
        Check(bounded.All(s=>math.distance(s.CourseStart.xz,s.CourseEnd.xz)>=.05f),"Optional paired correspondence manufactured a sub-grid tail");
        Check(bounded.Any(s=>s.StartNodeId==a.StartNodeId)&&bounded.Any(s=>s.EndNodeId==a.EndNodeId)&&
            bounded.Any(s=>s.StartNodeId==nearBoundary.StartNodeId)&&bounded.Any(s=>s.EndNodeId==nearBoundary.EndNodeId),
            "Sub-grid pair rejection deleted a source endpoint");
        Console.WriteLine("PASS: reciprocal ambiguity-aware pairing, C2 curved reference, synchronized source cuts, distinct topology/shared height cubics and level exclusion.");
    }
}
