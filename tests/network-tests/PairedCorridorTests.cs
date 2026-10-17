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
        Check(log.Metric("Trusted paired intervals")==1&&paired.Count==2&&paired.All(s=>s.PairedSpanId>0),"A unique straight pair was not emitted as one synchronized shared family: "+string.Join(";",log.Lines));
        Check(paired[0].StartHeightFamily==paired[1].EndHeightFamily&&paired[0].EndHeightFamily==paired[1].StartHeightFamily&&paired[0].StartNodeId!=paired[1].EndNodeId,"Height-family sharing merged topology IDs or ignored reverse station orientation");
        var unitCuts=NativeDesignUnits.Prepare(new[]{paired[0],paired[1] with{NativeWidth=24}},Array.Empty<NativeAssetContract>(),new ImportLog(),default);
        double[] Stations(long way)=>unitCuts.Where(s=>s.WayId==way).SelectMany(s=>new[]{s.PairedReferenceBegin,s.PairedReferenceEnd}).Distinct().Order().ToArray();
        Check(Stations(1).Length>3&&Stations(1).Zip(Stations(2),(x,y)=>Math.Abs(x-y)<1e-7).All(x=>x)&&Stations(1).Length==Stations(2).Length,"Asymmetric native core cuts desynchronized the shared reference profile");
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
