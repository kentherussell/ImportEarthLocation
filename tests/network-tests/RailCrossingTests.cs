using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

static class RailCrossingTests
{
    static void Check(bool condition,string message){if(!condition)throw new Exception(message);}
    static PreparedSegment Line(long way,long a,long b,float3 p,float3 q,NetworkKind kind)=>
        new(way,a,b,p,q,kind,kind==NetworkKind.Rail?"rail":"secondary",2,false,false,false,0)
        {NativeWidth=kind==NetworkKind.Rail?10:24,SourceGeometry=new[]{p,q},OriginalGeometry=new[]{p,q}};

    public static void Run()
    {
        const double width=2000;const float scale=300;const int size=257;
        var terrain=new ushort[size*size];
        for(var z=0;z<size;z++)for(var x=0;x<size;x++)
        {
            var px=(x/(double)(size-1)-.5)*width;var pz=(z/(double)(size-1)-.5)*width;
            terrain[z*size+x]=(ushort)Math.Round((100+.025*px+.015*pz+3*Math.Exp(-px*px/2500))/scale*65535);
        }
        float3 P(float x,float z)=>new(x,100,z);
        var source=new[]{Line(1,1,50,P(-503,0),P(0,0),NetworkKind.Road),Line(1,50,2,P(0,0),P(487,0),NetworkKind.Road),
            Line(2,3,4,P(0,-501),P(3,-18),NetworkKind.Rail),Line(2,4,50,P(3,-18),P(0,0),NetworkKind.Rail),
            Line(2,50,5,P(0,0),P(-2,21),NetworkKind.Rail),Line(2,5,6,P(-2,21),P(0,499),NetworkKind.Rail)};
        var log=new ImportLog();
        var compiled=VisualNetworkCompiler.Prepare(source,terrain,scale,width,log,default);
        var output=NativeEndpointPolicy.PrepareLevelCrossings(compiled,log);
        var ports=output.SelectMany(s=>new[]{(Section:s,Node:s.StartNodeId,Point:s.CourseStart),(Section:s,Node:s.EndNodeId,Point:s.CourseEnd)})
            .Where(p=>p.Node==50).ToArray();
        Check(ports.Length==4,"The road/rail crossing lost one of its four source approaches");
        Check(ports.Select(p=>p.Point).Distinct().Count()==1&&math.length(ports[0].Point.xz)<.001f,"Crossing approaches moved apart horizontally or vertically");
        Check(ports.All(p=>NativeEndpointPolicy.Identity(p.Section,50)=="Crossing:50"),"The native crossing retained separate road/rail owners");
        Check(log.Metric("Visual compiler shared road rail crossing heights")==1,"The visual compiler did not couple the crossing height");
        SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(source),output,log,default);
        Check(log.Metric("Missing mandatory selected-source ports")==0&&log.Metric("Unresolved selected-source reachability obligations")==0,"Crossing shaping lost a source route");
        foreach(var port in ports)
        {
            var s=port.Section;var start=s.StartNodeId==50;
            var tangent=start?s.CourseControl1-s.CourseStart:s.CourseEnd-s.CourseControl2;
            Check(Math.Abs(tangent.y)/Math.Max(.01,math.length(tangent.xz))<.02,"The crossing has a steep warped entry tangent");
        }
        var railDirections=ports.Where(p=>p.Section.Kind==NetworkKind.Rail).Select(p=>
        {
            var s=p.Section;return math.normalizesafe(p.Node==s.StartNodeId?(s.CourseControl1-s.CourseStart).xz:(s.CourseControl2-s.CourseEnd).xz);
        }).ToArray();
        Check(math.dot(railDirections[0],railDirections[1])<-.999f,"The railway has a horizontal kink at its road crossing");
        foreach(var rail in new[]{source[2] with{StartNodeId=50,Start=P(0,0),EndNodeId=99,End=P(0,100),Bridge=true,Layer=1},
            source[2] with{StartNodeId=50,Start=P(0,0),EndNodeId=99,End=P(0,100),Tunnel=true},
            source[2] with{StartNodeId=50,Start=P(0,0),EndNodeId=99,End=P(0,100),Layer=1}})
            Check(NativeEndpointPolicy.LevelCrossings(new[]{source[0],rail}).Count==0,"A separate-level railway became a level crossing");
        var unrelated=source[3] with{EndNodeId=500};
        Check(NativeEndpointPolicy.LevelCrossings(new[]{source[0],unrelated}).Count==0,"Coincident unrelated endpoints became a crossing");
        NativeBinding();
        var marked=source[0] with{EndLevelCrossing=true};
        var reversed=NetworkNormalizer.Reverse(marked);
        Check(reversed.StartLevelCrossing&&!reversed.EndLevelCrossing,"Reversing a road moved its crossing identity to the wrong end");
        var split=NativeDesignUnits.Split(marked,.5,-500);
        Check(!split.Left.EndLevelCrossing&&!split.Right.StartLevelCrossing&&split.Right.EndLevelCrossing,
            "A generated partition inherited the source crossing identity");
        var railPort=source[4];var shifted=source[1] with{Start=new(0,100.1f,0),Control1=new(100,100.1f,0),Control2=new(200,100,0),
            FinalDeckCurve=new(new(0,100.1f,0),new(100,100.1f,0),new(200,100,0),new(487,100,0))};
        var canonical=NativeEndpointPolicy.PrepareLevelCrossings(new[]{shifted,railPort},new ImportLog());
        Check(canonical[0].CourseStart.y==canonical[1].CourseStart.y&&canonical[0].CourseControl1.y==100,
            "Crossing canonicalization did not update the frozen native deck and its adjacent handle");
        var conflict=shifted with{FinalDeckCurve=null,Start=new(0,101,0)};
        Check(NativeEndpointPolicy.PrepareLevelCrossings(new[]{conflict,railPort},new ImportLog()).All(s=>!s.StartLevelCrossing),
            "A large crossing height disagreement was hidden by snapping the road");
        Console.WriteLine("PASS: shared road/rail crossing position, height, shallow approach tangents, native ownership across batches, source routes and separate-level guards.");
    }

    static void NativeBinding()
    {
        var road=Line(10,1,50,new(-50,100,0),new(0,100,0),NetworkKind.Road) with{EndLevelCrossing=true};
        var rail=Line(11,50,2,new(0,100,0),new(0,100,50),NetworkKind.Rail) with{StartLevelCrossing=true};
        NativeBatchBindings.Edge<string> Edge(PreparedSegment s,string id,string a,string b)=>new(id,a,b,s.Start,s.End,s.Kind,"stock");
        var re=Edge(road,"road","a","crossing");var te=Edge(rail,"rail","crossing","b");
        NativeBatchBindings.Expected<string> Expected(PreparedSegment s)=>new(s,"stock");
        var empty=new Dictionary<string,string>();
        var together=NativeBatchBindings.Match(new[]{Expected(road),Expected(rail)},new[]{re,te},empty);
        Check(together.Anchors.Count==3&&together.Anchors["Crossing:50"].Entity=="crossing","Native mixed-mode crossing did not share its owner");
        var first=NativeBatchBindings.Match(new[]{Expected(road)},new[]{re},empty);
        var committed=first.Anchors.ToDictionary(p=>p.Key,p=>p.Value.Entity);
        var second=NativeBatchBindings.Match(new[]{Expected(rail)},new[]{te},committed);
        Check(second.Anchors["Crossing:50"].Entity=="crossing","A later rail batch did not attach to the existing road crossing");
    }
}
