using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

static class EmissionCompactionTests
{
    static void Check(bool condition,string message){if(!condition)throw new Exception(message);}
    static PreparedSegment Road(long way,long a,long b,float x,float end)=>
        new(way,a,b,new(x,50,0),new(end,50,0),NetworkKind.Road,"motorway",2,true,false,false,0)
        {NativeWidth=8,NativeAssetKey="road",Reference="A1",SourceFitToleranceMeters=24,NativeLaneOffsets=new[]{-2f,2f}};
    static List<PreparedSegment> Pair(int count)
    {
        var spans=new List<PreparedSegment>();
        for(var i=0;i<count;i++)foreach(var reverse in new[]{false,true})
        {
            var begin=i*100f;var end=begin+100;var z=reverse?-20f:0;
            var a=new float3(reverse?end:begin,50,z);var b=new float3(reverse?begin:end,50,z);
            spans.Add(Road(100+i,reverse?201+i:100+i,reverse?200+i:101+i,a.x,b.x) with{
                Start=a,End=b,SourceGeometry=new[]{a,b},OriginalGeometry=new[]{a,b},
                PairedCorridorId=1,PairedSpanId=i+1,PairedProfileId=i+1,PairedReversed=reverse,ReferenceSpan=100,
                PairedReferenceBegin=reverse?end:begin,PairedReferenceEnd=reverse?begin:end,
                StartHeightFamily=reverse?10001+i:10000+i,EndHeightFamily=reverse?10000+i:10001+i,
                StartReferenceSpeed=1,EndReferenceSpeed=1});
        }
        return spans;
    }
    public static void Run()
    {
        var source=Pair(3);var snapshot=source.ToArray();
        var compact=PairedEmissionCompaction.Prepare(source,source,new ImportLog(),default,out var profiles);
        Check(compact.Count==2&&profiles.Count==2&&compact.Select(s=>s.PairedProfileId).Distinct().Count()==1,
            "Redundant paired source cuts were not removed synchronously");
        var forward=compact.Single(s=>!s.PairedReversed);var reverse=compact.Single(s=>s.PairedReversed);
        Check(forward.StartHeightFamily==reverse.EndHeightFamily&&forward.EndHeightFamily==reverse.StartHeightFamily&&
            forward.SourceWayIds.Count==3&&reverse.SourceWayIds.Count==3&&forward.SourceGeometry.Count==6,
            "Paired coalescence lost source evidence or shared family orientation");
        Check(NetworkPairedCorridors.Separated(new(forward.CourseStart,forward.CourseControl1,forward.CourseControl2,forward.CourseEnd),
            new(reverse.CourseStart,reverse.CourseControl1,reverse.CourseControl2,reverse.CourseEnd),10,default),
            "Coalesced final float pair did not preserve independent-parameter spacing");
        var required=source.Select((s,i)=>i==0?s with{SourceNodeAliases=new[]{new SourceNodeAlias(101,101,true)}}:s).ToList();
        Check(PairedEmissionCompaction.Prepare(required,required,new ImportLog(),default,out _).Count==4,
            "A mandatory source port was removed from a paired formation");
        var branched=source.Append(Road(900,101,999,100,150) with{End=new(150,50,100)}).ToList();
        var branchOutput=PairedEmissionCompaction.Prepare(branched,branched,new ImportLog(),default,out _);
        Check(branchOutput.Count==5&&branchOutput.Any(s=>s.EndNodeId==101)&&branchOutput.Any(s=>s.StartNodeId==101),
            "An attached branch was contracted during paired coalescence");
        var transition=source.Select((s,i)=>i>=2?s with{Bridge=true}:s).ToList();
        Check(PairedEmissionCompaction.Prepare(transition,transition,new ImportLog(),default,out _).Count==4,
            "A structural transition was erased during paired coalescence");
        var fixedUnits=source.Select(s=>s with{NativeUnitId=7}).ToList();
        Check(PairedEmissionCompaction.Prepare(fixedUnits,fixedUnits,new ImportLog(),default,out _).Count==source.Count,
            "Paired coalescence rewrote established native units");
        Check(PairedEmissionCompaction.Prepare(source,source,new ImportLog(),default,out _,width:250).Count==source.Count,
            "Paired coalescence published controls outside the map envelope");
        var offset=source.Select(s=>s with{Start=s.Start-new float3(0,0,1),End=s.End-new float3(0,0,1),
            StartCurveOffset=new(0,0,1),EndCurveOffset=new(0,0,1)}).ToList();
        var offsetResult=PairedEmissionCompaction.Prepare(offset,offset,new ImportLog(),default,out _);
        Check(offsetResult.Count==2&&offsetResult.All(s=>s.StartCurveOffset.z==1&&s.EndCurveOffset.z==1),
            "Course offsets unnecessarily split a continuous pair or were lost during publication");
        var longPair=Pair(10);var bounded=PairedEmissionCompaction.Prepare(longPair,longPair,new ImportLog(),default,out _);
        Check(bounded.Count==6&&bounded.All(s=>s.ReferenceSpan<=400),"Paired simplification raised the native span limit");
        var curved=Pair(3).Select(s=>s with{Control1=s.CourseControl1+new float3(0,0,.02f),
            Control2=s.CourseControl2+new float3(0,0,.02f)}).ToList();
        var curvedResult=PairedEmissionCompaction.Prepare(curved,curved,new ImportLog(),default,out _);
        Check(curvedResult.Count<curved.Count,"The bounded nonstraight paired fit was never exercised");
        foreach(var s in curved)for(var k=0;k<=64;k++)
        {
            var t=k/64d;var station=math.lerp(s.PairedReferenceBegin,s.PairedReferenceEnd,t);
            var result=curvedResult.First(r=>r.PairedReversed==s.PairedReversed&&
                station>=Math.Min(r.PairedReferenceBegin,r.PairedReferenceEnd)-1e-7&&station<=Math.Max(r.PairedReferenceBegin,r.PairedReferenceEnd)+1e-7);
            var u=(station-result.PairedReferenceBegin)/(result.PairedReferenceEnd-result.PairedReferenceBegin);
            var before=NetworkDesignChecks.Evaluate(new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd),t);
            var after=NetworkDesignChecks.Evaluate(new(result.CourseStart,result.CourseControl1,result.CourseControl2,result.CourseEnd),u);
            Check(math.distance(before.xz,after.xz)<=.250001,"Repeated paired merges accumulated beyond their total deviation budget");
        }
        var ordinary=Enumerable.Range(0,10).Select(i=>Road(900+i,900+i,901+i,i*100,(i+1)*100)).ToArray();
        var adaptive=NativeChainCompactor.Prepare(ordinary,new ImportLog(),default);
        Check(adaptive.Count==3&&adaptive.All(s=>NetworkDesignChecks.HorizontalArcLength(new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd))<=400.001),
            "Adaptive fitting added avoidable short windows or exceeded the native cap");
        Check(source.SequenceEqual(snapshot),"Paired compaction mutated its input");
    }
}
