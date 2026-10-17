using ImportEarthLocation.Core;
using Unity.Mathematics;

static class LaneObligationTests
{
    static void Check(bool value,string message){if(!value)throw new Exception(message);}
    public static void Run()
    {
        PreparedSegment Road(long way,long a,long b,float3 start,float3 end,int lanes,float offset)=>
            new(way,a,b,start,end,NetworkKind.Road,"motorway",lanes,true,false,false,0)
            {StartCurveOffset=new(0,0,offset),EndCurveOffset=new(0,0,offset),NativeLaneOffsets=Enumerable.Range(0,lanes).Select(i=>(i-(lanes-1)*.5f)*4).ToArray()};
        var left=Road(1,1,3,new(-100,20,0),new(0,20,0),2,4);
        var right=Road(2,2,3,new(-100,20,0),new(0,20,0),2,-4);
        var receiver=Road(3,3,4,new(0,20,0),new(100,20,0),4,0);
        var merge=NativeLaneObligations.Build(new[]{left,right,receiver});
        Check(merge.Movements.Count==4&&merge.UnavailableNodes==0,"2+2 to4 did not publish four exact slot obligations");
        Check(merge.Movements.Where(m=>m.From.Section==0).Select(m=>m.To.Rank).Order().SequenceEqual(new[]{0,1})&&
            merge.Movements.Where(m=>m.From.Section==1).Select(m=>m.To.Rank).Order().SequenceEqual(new[]{2,3}),"Two inputs mapped to the wrong half of the four-lane receiver");
        Check(merge.Movements.All(m=>m.PlannedLateralError<1e-6),"Aligned lane slots acquired a planned lateral mismatch");
        var split=NativeLaneObligations.Build(new[]{left with{StartNodeId=3,EndNodeId=1,Start=new(0,20,0),End=new(100,20,0)},right with{StartNodeId=3,EndNodeId=2,Start=new(0,20,0),End=new(100,20,0)},receiver with{StartNodeId=4,EndNodeId=3,Start=new(-100,20,0),End=new(0,20,0)}});
        Check(split.Movements.Count==4&&split.Movements.Select(m=>m.From.Rank).Distinct().Count()==4,"Reverse split dropped physical lane obligations");
        var unavailable=NativeLaneObligations.Build(new[]{left with{NativeLaneOffsets=Array.Empty<float>()},receiver});
        Check(unavailable.UnavailableNodes==1&&unavailable.Movements.Count==0,"Missing metadata became an invented or disconnected lane graph");
        var turn=NativeLaneObligations.Build(new[]{left,receiver with{End=new(0,20,100)}});
        Check(turn.UnavailableNodes==1&&turn.Movements.Count==0,"General turn permissions were silently invented from geometry");
        Console.WriteLine("PASS: immutable 2+2->4 and reverse physical lane-slot obligations, exact lane-half mapping and unavailable metadata/turn semantics.");
    }
}
