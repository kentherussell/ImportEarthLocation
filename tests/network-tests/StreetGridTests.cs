using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

static class StreetGridTests
{
    static void Check(bool value,string message){if(!value)throw new Exception(message);}
    public static void Run()
    {
        var angle=23*Math.PI/180;var e=new double2(Math.Cos(angle),Math.Sin(angle));var n=new double2(-e.y,e.x);
        SourceNodeKey Node(int x,int y)=>new(NetworkKind.Road,100+x*10+y);
        double2 Point(int x,int y)=>e*(x*180+(y==0?0:.6*Math.Sin(x+2*y)))+n*(y*180+(x==0?0:.7*Math.Cos(2*x+y)));
        var streets=new List<StreetGridAlignment.Street>();
        for(var y=0;y<3;y++)streets.Add(new(y,Enumerable.Range(0,3).Select(x=>Node(x,y)).ToArray(),new[]{0,1,2},Enumerable.Range(0,3).Select(x=>Point(x,y)).ToArray(),12));
        for(var x=0;x<3;x++)streets.Add(new(3+x,Enumerable.Range(0,3).Select(y=>Node(x,y)).ToArray(),new[]{0,1,2},Enumerable.Range(0,3).Select(y=>Point(x,y)).ToArray(),12));
        var curve=new StreetGridAlignment.Street(9,new[]{Node(9,0),Node(9,1),Node(9,2)},new[]{0,1,2},new[]{new double2(1400,0),new double2(1420,180),new double2(1400,360)},12);
        streets.Add(curve);var fixedNodes=new HashSet<SourceNodeKey>{Node(0,0)};
        var result=StreetGridAlignment.Prepare(streets,fixedNodes,new ImportLog(),default);
        Check(Enumerable.Range(0,6).All(result.Lines.ContainsKey),"An evidenced rotated street grid was not aligned");
        Check(!result.Lines.ContainsKey(9),"An intentionally curved road was flattened into the nearby grid");
        for(var y=0;y<3;y++)for(var x=0;x<3;x++)
        {
            var point=result.Nodes.TryGetValue(Node(x,y),out var moved)?moved:Point(x,y);
            Check(Math.Abs(math.dot(point,result.Lines[y].Normal)-result.Lines[y].Offset)<1e-6&&
                Math.Abs(math.dot(point,result.Lines[3+x].Normal)-result.Lines[3+x].Offset)<1e-6,
                "A grid junction does not lie on both of its aligned streets");
            Check(math.distance(point,Point(x,y))<2,"Grid cleanup needlessly moved a surveyed junction");
        }
        Check(!result.Nodes.ContainsKey(Node(0,0)),"Grid cleanup displaced a protected highway/structure junction");
        Check(Math.Abs(math.dot(result.Lines[0].Direction,result.Lines[3].Direction))<1e-12&&
            Math.Abs(NetworkDesignChecks.Cross(result.Lines[0].Direction,result.Lines[1].Direction))<1e-12,
            "Aligned grid streets are not perpendicular and parallel");
        var reversed=StreetGridAlignment.Prepare(streets.AsEnumerable().Reverse().ToArray(),fixedNodes,new ImportLog(),default);
        Check(result.Nodes.Count==reversed.Nodes.Count&&result.Nodes.All(p=>math.distance(p.Value,reversed.Nodes[p.Key])<1e-8),
            "The inferred grid depends on source enumeration order");
        var oblique=new[]{new StreetGridAlignment.Street(20,new[]{Node(20,0),Node(20,1)},new[]{0,1},new[]{new double2(0,0),new double2(400,0)},12),
            new StreetGridAlignment.Street(21,new[]{Node(21,0),Node(21,1)},new[]{0,1},new[]{new double2(0,0),new double2(200,350)},12)};
        var isolated=StreetGridAlignment.Prepare(oblique,new(),new ImportLog(),default);
        Check(Math.Abs(math.dot(isolated.Lines[20].Direction,isolated.Lines[21].Direction))>.4,
            "An isolated oblique intersection was forced into a rectangular grid");
        Console.WriteLine("PASS: rotated local street grids, exact parallel/perpendicular axes, shared junctions, protected anchors and curved-road exceptions.");
    }
}
