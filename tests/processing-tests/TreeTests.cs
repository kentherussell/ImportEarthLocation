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
        var dryOnly=ForestProcessor.Prepare(data,request,terrain,new[]{road},new ImportLog(),default,excludeWater:p=>Math.Abs(p.x)<30);
        Check(dryOnly.Count<output.Count&&dryOnly.All(p=>Math.Abs(p.x)>=30),"Forest candidates were retained inside a supplied continuous water mask");
        Check(output.Count>=39&&output.Count<=41,"Mapped tree row was not continuously populated at about 6m spacing");
        for(var i=0;i<output.Count;i++)for(var j=i+1;j<output.Count;j++)Check(math.distance(output[i].xz,output[j].xz)>=ForestProcessor.MappedSpacing-.001f,"Mapped tree spacing violated");
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
        Check(diagnostic.Contains("woodlandCellMeters=15;")&&diagnostic.Contains("globalDensityReduction=false")&&diagnostic.Contains("woodlandFullyEnumerated=True")&&diagnostic.Contains("unexaminedDiscoveredCandidates=0;")&&diagnostic.Contains("limit=none;"),"Fixed-density/full uncapped enumeration diagnostics missing");
        // Natural thinning: interior gaps and density variation remove roughly a
        // tenth to a third, and the woodland edge is sparser than its interior.
        int Field(string text,string key){var i=text.IndexOf(key+"=",StringComparison.Ordinal)+key.Length+1;return int.Parse(new string(text.Skip(i).TakeWhile(char.IsDigit).ToArray()));}
        var thinnedShare=Field(diagnostic,"woodlandThinned")/(double)Field(diagnostic,"woodlandCandidates");
        Check(thinnedShare is >.08 and <.35,$"Woodland thinning removed {thinnedShare:P0}, outside the intended natural range");
        var square=new Dictionary<long,OsmNode>{{1,Node(1,-1000,-1000)},{2,Node(2,1000,-1000)},{3,Node(3,1000,1000)},{4,Node(4,-1000,1000)}};
        var block=ForestProcessor.Prepare(new OsmDataset{Nodes=square,Ways=new Dictionary<long,OsmWay>{{1,wood}}},woodlandRequest,terrain,Array.Empty<PreparedSegment>(),new ImportLog(),default);
        double Inset(float3 p)=>1000-Math.Max(Math.Abs(p.x),Math.Abs(p.z));
        var rimDensity=block.Count(p=>Inset(p)<30)/(4*2000*30d);var coreDensity=block.Count(p=>Inset(p)>=60)/(1880*1880d);
        Check(rimDensity<.92*coreDensity,$"Woodland edges are not feathered: rim {rimDensity*1e4:0.0} vs core {coreDensity*1e4:0.0} trees/ha");
        var overlapped=ForestProcessor.Prepare(new OsmDataset{Nodes=woodNodes,Ways=new Dictionary<long,OsmWay>{{2,wood with{Id=2}},{1,wood}}},woodlandRequest,terrain,Array.Empty<PreparedSegment>(),new ImportLog(),default);
        Check(dense.SequenceEqual(overlapped),"Overlapping polygons or polygon order change local density/output");
        if(ForestProcessor.PlantInferredTrees)Inferred(Node,request,terrain);
        using var cancel=new CancellationTokenSource(); cancel.Cancel();
        try { ForestProcessor.Prepare(data,request,terrain,Array.Empty<PreparedSegment>(),new ImportLog(),cancel.Token); throw new Exception("Forest cancellation ignored"); } catch(OperationCanceledException){}
        Console.WriteLine("PASS: tagged-node XML/cache preservation, uncapped center-out mapped/woodland trees, mapped-tree priority, rows across vertices, overlap deduplication, standalone trees, bounds/network/water guards and determinism.");
    }

    // Trees inferred from land use: orchard rows, parks with open pitches,
    // swamps as woodland, residential gardens and street trees, and gaps
    // filled between mapped street trees.
    static void Inferred(Func<long,float,float,OsmNode> node,ImportRequest request,TerrainBuilder.Result terrain)
    {
        var nodes=new Dictionary<long,OsmNode>();var ways=new Dictionary<long,OsmWay>();long next=1;
        void Area(long id,float x0,float z0,float x1,float z1,params (string,string)[] tags)
        {
            var ids=new long[5];foreach(var (i,x,z) in new[]{(0,x0,z0),(1,x1,z0),(2,x1,z1),(3,x0,z1)}){ids[i]=next;nodes[next]=node(next,x,z);next++;}
            ids[4]=ids[0];ways[id]=new OsmWay(id,ids,tags.ToDictionary(t=>t.Item1,t=>t.Item2));
        }
        IReadOnlyList<float3> Plant(IReadOnlyList<PreparedSegment>? roads=null,Dictionary<long,IReadOnlyDictionary<string,string>>? trees=null)=>
            ForestProcessor.Prepare(new OsmDataset{Nodes=nodes,Ways=ways,NodeTags=trees??new()},request,terrain,roads??Array.Empty<PreparedSegment>(),new ImportLog(),default);
        bool Spaced(IReadOnlyList<float3> trees)
        {
            for(var i=0;i<trees.Count;i++)for(var j=i+1;j<trees.Count;j++)if(math.distance(trees[i].xz,trees[j].xz)<ForestProcessor.MappedSpacing-.001f)return false;
            return true;
        }

        Area(1,-50,-35,50,35,("landuse","orchard"));
        var orchard=Plant();
        Check(orchard.Count is >=140 and <=180&&orchard.All(p=>Math.Abs(p.x)<=50&&Math.Abs(p.z)<=35)&&Spaced(orchard),$"Orchard was not planted in 6 × 7 m rows ({orchard.Count} trees)");
        Check(orchard.Select(p=>Math.Round(p.z,1)).Distinct().Count()==10,"Orchard rows do not follow the orchard's longest edge");

        ways.Clear();Area(1,-200,-200,200,200,("leisure","park"));Area(2,-40,-30,40,30,("leisure","pitch"));
        var park=Plant();var hectares=16-.48;
        Check(park.Count/hectares is >4 and <30,$"Park density {park.Count/hectares:0.0} trees/ha is outside the open parkland range");
        Check(!park.Any(p=>Math.Abs(p.x)<40&&Math.Abs(p.z)<30),"Trees planted on a mapped pitch");

        ways.Clear();Area(1,-200,-200,200,200,("natural","wetland"),("wetland","swamp"));
        var swamp=Plant();ways.Clear();Area(1,-200,-200,200,200,("natural","wood"));
        Check(swamp.SequenceEqual(Plant()),"A swamp is not planted as woodland");

        ways.Clear();Area(1,-280,-280,280,280,("landuse","residential"));
        var street=new PreparedSegment(10,1,2,new float3(-280,0,0),new float3(280,0,0),NetworkKind.Road,"residential",2,false,false,false,0){NativeWidth=12};
        var homes=Plant(new[]{street});
        Check(homes.Count(p=>Math.Abs(Math.Abs(p.z)-10)<.5f)>=60,"Residential street is missing its street trees");
        Check(homes.All(p=>Math.Abs(p.z)>9),"Street trees were planted inside the road's verge clearance");
        Check(homes.Count(p=>Math.Abs(p.z)>=14)/(56*56*.01)>4,"Residential land got no garden trees");
        Check(homes.SequenceEqual(Plant(new[]{street})),"Inferred trees are nondeterministic");

        ways.Clear();
        var mapped=new Dictionary<long,IReadOnlyDictionary<string,string>>();IReadOnlyDictionary<string,string> tree=new Dictionary<string,string>{{"natural","tree"}};
        foreach(var x in new[]{-100f,-90,-80,0}){nodes[next]=node(next,x,8);mapped[next++]=tree;}
        var avenue=Plant(new[]{street with{Classification="tertiary",NativeWidth=8}},mapped);
        Check(avenue.Count(p=>p.x>-80&&p.x<0&&Math.Abs(p.z-8)<.5f)==7,$"Gap between mapped street trees was not filled at their 10 m interval ({avenue.Count} trees)");
        Check(avenue.Count==11,"Gap filling or street trees added trees outside mapped gaps or residential land");
    }
}
