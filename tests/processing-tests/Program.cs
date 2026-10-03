using System.Text;
using ImportEarthLocation.Core;
using ImportEarthLocation.Geo;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

static class Program
{
    static void Check(bool value, string message) { if (!value) throw new Exception(message); }
    static PreparedSegment Segment(long way, long start, long end, float x1, float z1, float x2, float z2, NetworkKind kind = NetworkKind.Road) => new(way, start, end, new float3(x1,0,z1), new float3(x2,0,z2),kind,"primary",2,false,false,false,0);
    static void Main()
    {
        var log = new ImportLog(); var source = new List<PreparedSegment>();
        for (var i = 0; i < 10; i++) source.Add(Segment(1,i+1,i+2,i*10,0,(i+1)*10,0));
        var normalized = NetworkNormalizer.Normalize(source,log,default);
        Check(normalized.Count == 1 && normalized[0].StartNodeId == 1 && normalized[0].EndNodeId == 11,"Straight geometry not merged/preserved");
        NetworkNormalizer.Validate(normalized,14336,default);
        source.Add(Segment(2,6,100,50,0,50,50));
        normalized = NetworkNormalizer.Normalize(source,log,default);
        Check(normalized.Count == 3 && normalized.Any(s => s.EndNodeId == 6) && normalized.Any(s => s.StartNodeId == 6),"Junction moved/lost");
        var hairpin = new[] { Segment(1,1,2,0,0,10,0),Segment(1,2,3,10,0,10,10),Segment(1,3,4,10,10,0,10) };
        Check(NetworkNormalizer.Normalize(hairpin,log,default).Count == 3,"Sharp corners rounded");
        source = new(); for(var i=0;i<16;i++) source.Add(Segment(1,i+1,i+2,i*10,20*MathF.Sin(i/10f),(i+1)*10,20*MathF.Sin((i+1)/10f)));
        normalized=NetworkNormalizer.Normalize(source,log,default); Check(normalized.Count < source.Count,"Gentle curve not fitted"); NetworkNormalizer.Validate(normalized,14336,default);
        var parallel = new[] { Segment(1,1,2,0,0,100,0,NetworkKind.Rail), Segment(2,3,4,0,3,100,3,NetworkKind.Rail) };
        Check(NetworkNormalizer.Normalize(parallel,log,default).Count==2,"Parallel rail removed");
        var reverseDuplicates = new[] { Segment(1,1,2,0,0,100,0), Segment(2,2,1,100,0,0,0) };
        Check(NetworkNormalizer.Normalize(reverseDuplicates,log,default).Count == 1,"Reversed equivalent duplicate not removed");
        var separated = new[] { Segment(1,1,2,0,0,100,0), Segment(2,1,2,0,0,100,0) with { Bridge=true, Layer=1 } };
        Check(NetworkNormalizer.Normalize(separated,log,default).Count==2,"Bridge-separated infrastructure removed");
        using(var cancelled=new CancellationTokenSource()) { cancelled.Cancel(); try { NetworkNormalizer.Normalize(source,log,cancelled.Token); throw new Exception("Cancellation ignored"); } catch(OperationCanceledException) {} }
        var xml=Encoding.UTF8.GetBytes("<osm><node id='1' lat='0' lon='0'/><node id='bad' lat='0' lon='0'/><node id='2' lat='0' lon='.01'/><way id='1'><nd ref='1'/><nd ref='2'/><tag k='highway' v='primary'/></way></osm>");
        var dataset=OsmParser.Merge(Enumerable.Repeat(xml,16).ToArray(),log); Check(dataset.Nodes.Count==2 && dataset.Ways.Count==1,"Malformed feature prevented valid parsing");
        try { OsmParser.Merge(new[]{xml},log); throw new Exception("Incomplete coverage accepted"); } catch(System.IO.InvalidDataException) {}
        Check(WaterLevels.Effective(40)==41 && WaterLevels.Effective(10)==11,"Water correction not exactly +1m");
        var nodes = new Dictionary<long,OsmNode>();
        double Degrees(float meters) => meters / 6378137d * 180 / Math.PI;
        void Ring(long first,float min,float max) { nodes[first]=new(first,Degrees(min),Degrees(min)); nodes[first+1]=new(first+1,Degrees(min),Degrees(max)); nodes[first+2]=new(first+2,Degrees(max),Degrees(max)); nodes[first+3]=new(first+3,Degrees(max),Degrees(min)); }
        Ring(1,-500,500); Ring(5,-100,100);
        var tags = new Dictionary<string,string>{{"landuse","forest"}};
        var ways=new Dictionary<long,OsmWay>{{1,new(1,new long[]{1,2,3,4,1},new Dictionary<string,string>())},{2,new(2,new long[]{5,6,7,8,5},new Dictionary<string,string>())}};
        var forest=new OsmDataset{Nodes=nodes,Ways=ways,Relations=new[]{new OsmRelation(1,new (long,bool)[]{(1,false),(2,true)},tags)}};
        var terrain=new TerrainBuilder.Result(Enumerable.Repeat((ushort)32768,16*16).ToArray(),100,40,0,100);
        var request=new ImportRequest(0,0,MapSizeProfile.Vanilla,false);
        var corridor=new[]{Segment(1,1,2,-500,0,500,0)};
        var trees=ForestProcessor.Prepare(forest,request,terrain,corridor,log,default);
        Check(trees.Count>100 && trees.Count<=ForestProcessor.MaximumTrees,"Forest candidates not bounded/generated");
        Check(trees.All(p => Math.Abs(p.x)>100 || Math.Abs(p.z)>100),"Forest hole filled");
        Check(trees.All(p=>Math.Abs(p.z)>17),"Road corridor planted");
        Check(trees.SequenceEqual(ForestProcessor.Prepare(forest,request,terrain,corridor,log,default)),"Tree generation nondeterministic");
        var smallMap=request with{MapSize=new MapSizeProfile("test","test",600,false)};
        Check(ForestProcessor.Prepare(forest,smallMap,terrain,corridor,log,default).All(p=>Math.Abs(p.x)<=300 && Math.Abs(p.z)<=300),"Trees not clipped to map bounds");
        for(var i=0;i<trees.Count;i++) for(var j=i+1;j<trees.Count;j++) Check(math.distance(trees[i].xz,trees[j].xz)>=ForestProcessor.MinimumSpacing,"Tree spacing violated");
        Check(ForestProcessor.Prepare(forest,request,terrain with{SeaLevelMeters=60},corridor,log,default).Count==0,"Underwater forest planted");
        Console.WriteLine("PASS: straight/gentle curves, strict deviation, protected junctions/sharp bends, parallel rail, cancellation, malformed features, complete coverage, deterministic forest masks/holes/corridors/spacing/water, exact +1m correction");
    }
}
namespace ImportEarthLocation.Core
{
    public sealed class ImportLog { public void Info(string x){} public void Warn(string x){} public void Count(string x,int count=1){} public void Time(string x,double seconds){} }
}
namespace ImportEarthLocation.Processing
{
    public static class TerrainBuilder { public const int Resolution=16; public sealed record Result(ushort[] Heightmap,float HeightScaleMeters,float SeaLevelMeters,float MinimumElevation,float MaximumElevation); }
}
