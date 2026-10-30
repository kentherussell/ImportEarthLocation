using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

// Reproduction of repeated redistribution extending a fit beyond its interval.
// Shared endpoints, outer anchors and tangents must survive every redistribution.
static class RebalanceRegressionTests
{
    static void Check(bool value,string message){if(!value)throw new Exception(message);}
    static PreparedSegment Road(long way,long a,long b,float3 p,float3 q)=>
        new(way,a,b,p,q,NetworkKind.Road,"motorway",2,true,false,false,0)
        {NativeWidth=12,NativeAssetKey="visual",Reference="V1",SourceGeometry=new[]{p,q},OriginalGeometry=new[]{p,q}};

    // Rebalancing adjacent short sections twice used an untrimmed source piece
    // on the second fit, moving one end while its neighbour kept the old end.
    public static void Run()
    {
        float3[] points={new(-1000,50,0),new(-873,50,27),new(-722,50,16),new(-645,50,35),new(-560,50,70),
            new(-493,50,52),new(-395,50,81),new(-327,50,91),new(-214,50,57),new(-55,50,43),new(57,50,58),
            new(141,50,49),new(281,50,49),new(436,50,73),new(549,50,51),new(675,50,45),new(828,50,12),new(985,50,16),new(1066,50,6)};
        var terrain=Enumerable.Repeat((ushort)32768,33*33).ToArray();
        foreach(var reverse in new[]{false,true})
        {
            var path=reverse?points.Reverse().ToArray():points;
            var roads=Enumerable.Range(0,path.Length-1).Select(i=>Road(i+1,i+1,i+2,path[i],path[i+1]) with{NativeWidth=14}).ToArray();
            var output=VisualNetworkCompiler.Prepare(roads,terrain,100,8000,new ImportLog(),default);
            Check(output.Count>1&&output.SelectMany(s=>s.SourceWayIds).Distinct().Count()==roads.Length,
                "Short-section fitting lost the continuous road or its source sections");
            foreach(var node in output.SelectMany(s=>new[]{(s.StartNodeId,s.Start),(s.EndNodeId,s.End)}).GroupBy(p=>p.Item1))
                Check(node.All(p=>math.distance(p.Item2,node.First().Item2)<.001f),
                    $"Rebalancing left the pieces at shared node {node.Key} with different endpoints");
            Check(output.All(s=>NetworkDesignChecks.Finite(new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd))),
                "Short-section fitting emitted nonfinite geometry");
            foreach(var node in output.SelectMany(s=>new[]{(Id:s.StartNodeId,Start:true,Section:s),(Id:s.EndNodeId,Start:false,Section:s)}).GroupBy(p=>p.Id))
            {
                if(node.Count()!=2)continue;
                var headings=node.Select(p=>math.normalizesafe((p.Start?p.Section.CourseControl1-p.Section.CourseStart:p.Section.CourseControl2-p.Section.CourseEnd).xz)).ToArray();
                Check(math.dot(headings[0],headings[1])<-.9999f,"Rebalancing introduced a tangent kink at a smooth internal join");
            }
            Check(output.All(s=>NetworkVerticalProfiles.MaximumGrade(s)<1e-5),"Rebalancing introduced a bump on flat terrain");
            Check(math.distance(output.Single(s=>s.StartNodeId==1).Start.xz,path[0].xz)<.001f&&
                math.distance(output.Single(s=>s.EndNodeId==path.Length).End.xz,path[^1].xz)<.001f,
                "Short-section fitting moved an outer source anchor");
        }
    }

}
