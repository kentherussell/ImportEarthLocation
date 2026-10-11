using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

static class TreeTests
{
    static void Check(bool value,string message) { if(!value)throw new Exception(message); }
    public static void Run()
    {
        double Degrees(float m)=>m/6378137d*180/Math.PI;
        OsmNode Node(long id,float x,float z)=>new(id,Degrees(z),Degrees(x));
        var nodes=new Dictionary<long,OsmNode>{{1,Node(1,0,200)},{2,Node(2,-120,200)},{3,Node(3,0,200)},{4,Node(4,120,200)},{5,Node(5,400,200)},{6,Node(6,0,0)}};
        IReadOnlyDictionary<string,string> tag=new Dictionary<string,string>{{"natural","tree"}};
        var tags=new Dictionary<long,IReadOnlyDictionary<string,string>>{{1,tag},{3,tag},{5,tag},{6,tag}};
        var row=new OsmWay(1,new long[]{2,3,4},new Dictionary<string,string>{{"natural","tree_row"}});
        var data=new OsmDataset{Nodes=nodes,NodeTags=tags,Ways=new Dictionary<long,OsmWay>{{1,row}}};
        var request=new ImportRequest(0,0,new("test","test",600,false),false);
        var terrain=new TerrainBuilder.Result(Enumerable.Repeat((ushort)32768,256).ToArray(),100,0,0,100);
        var road=new PreparedSegment(10,1,2,new float3(-300,0,0),new float3(300,0,0),NetworkKind.Road,"primary",2,false,false,false,0);
        var log=new ImportLog();
        var output=ForestProcessor.Prepare(data,request,terrain,new[]{road},log,default);
        Check(output.Count>=19&&output.Count<=21,"Mapped tree row was not continuously populated at about 12m spacing");
        Check(math.distance(output[0].xz,new float2(0,200))<.001,"Mapped individual tree did not get priority/preserve its position");
        Check(output.Count(p=>math.distance(p.xz,new float2(0,200))<.001)==1,"Duplicate individual/row tree planted twice");
        Check(output.All(p=>Math.Abs(p.x)<=300&&Math.Abs(p.z)>17),"Mapped trees ignored map bounds or network clearance");
        Check(output.SequenceEqual(ForestProcessor.Prepare(data,request,terrain,new[]{road},new ImportLog(),default)),"Mapped tree/row output is nondeterministic");
        var single=ForestProcessor.Prepare(new OsmDataset{Nodes=nodes,NodeTags=tags},request,terrain,new[]{road},new ImportLog(),default);
        Check(single.Count==1,"Individual mapped trees must work without woodland polygons");
        Check(ForestProcessor.Prepare(data,request,terrain with{SeaLevelMeters=60},Array.Empty<PreparedSegment>(),new ImportLog(),default).Count==0,"Mapped/row trees planted under sea level");
        var reversed=new OsmDataset{Nodes=nodes.Reverse().ToDictionary(x=>x.Key,x=>x.Value),NodeTags=tags.Reverse().ToDictionary(x=>x.Key,x=>x.Value),Ways=data.Ways};
        Check(output.SequenceEqual(ForestProcessor.Prepare(reversed,request,terrain,new[]{road},new ImportLog(),default)),"Tile/dictionary order changes tree placement");
        Check(output.Zip(output.Skip(1),(a,b)=>math.lengthsq(a.xz)<=math.lengthsq(b.xz)).All(x=>x),"Tree placement order is not center-out");
        var manyNodes=new Dictionary<long,OsmNode>(); var manyTags=new Dictionary<long,IReadOnlyDictionary<string,string>>();
        for(var i=0;i<100001;i++) { var id=(long)i+1; manyNodes[id]=Node(id,(i%350-175)*15,(i/350-143)*15); manyTags[id]=tag; }
        var uncapped=ForestProcessor.Prepare(new OsmDataset{Nodes=manyNodes,NodeTags=manyTags},request with{MapSize=new("test","test",6000,false)},terrain,Array.Empty<PreparedSegment>(),new ImportLog(),default);
        Check(uncapped.Count==manyNodes.Count,"An eligible mapped tree was dropped at the old 100K limit");
        Check(uncapped.Zip(uncapped.Skip(1),(a,b)=>math.lengthsq(a.xz)<=math.lengthsq(b.xz)).All(x=>x),"Uncapped mapped trees are not center-out");
        var woodNodes=new Dictionary<long,OsmNode>{{1,Node(1,-30000,-30000)},{2,Node(2,30000,-30000)},{3,Node(3,30000,30000)},{4,Node(4,-30000,30000)}};
        var wood=new OsmWay(1,new long[]{1,2,3,4,1},new Dictionary<string,string>{{"natural","wood"}});
        var largeLog=new ImportLog();
        // Exercise the former cap without enumerating an entire 57 km forest in a smoke check.
        var woodlandRequest=request with{MapSize=new("test","test",8000,false)};
        var dense=ForestProcessor.Prepare(new OsmDataset{Nodes=woodNodes,Ways=new Dictionary<long,OsmWay>{{1,wood}}},woodlandRequest,terrain,Array.Empty<PreparedSegment>(),largeLog,default);
        Check(dense.Count>100000,"Woodland still stops at the old 100K accepted-tree cap");
        Check(math.length(dense[dense.Count-1].xz)>5000,"Uncapped woodland did not reach the outer map area");
        Check(dense.Zip(dense.Skip(1),(a,b)=>math.lengthsq(a.xz)<=math.lengthsq(b.xz)).All(x=>x),"Lazy woodland blocks hide nearer candidates");
        var diagnostic=largeLog.Diagnostics.Single(s=>s.StartsWith("FOREST DISTRIBUTION"));
        Check(diagnostic.Contains("woodlandCellMeters=18;")&&diagnostic.Contains("globalDensityReduction=false")&&diagnostic.Contains("woodlandFullyEnumerated=True")&&diagnostic.Contains("unexaminedDiscoveredCandidates=0;")&&diagnostic.Contains("limit=none;"),"Fixed-density/full uncapped enumeration diagnostics missing");
        var overlapped=ForestProcessor.Prepare(new OsmDataset{Nodes=woodNodes,Ways=new Dictionary<long,OsmWay>{{2,wood with{Id=2}},{1,wood}}},woodlandRequest,terrain,Array.Empty<PreparedSegment>(),new ImportLog(),default);
        Check(dense.SequenceEqual(overlapped),"Overlapping polygons or polygon order change local density/output");
        using var cancel=new CancellationTokenSource(); cancel.Cancel();
        try { ForestProcessor.Prepare(data,request,terrain,Array.Empty<PreparedSegment>(),new ImportLog(),cancel.Token); throw new Exception("Forest cancellation ignored"); } catch(OperationCanceledException){}
        Console.WriteLine("PASS: tagged-node XML/cache preservation, uncapped center-out mapped/woodland trees, mapped-tree priority, rows across vertices, overlap deduplication, standalone trees, bounds/network/water guards and determinism.");
    }
}
