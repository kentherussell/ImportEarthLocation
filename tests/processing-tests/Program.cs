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
        var progress = new PreparationProgress();
        progress.Begin("Smoothing roads");
        Check(progress.Snapshot().Contains("Smoothing roads") && progress.Snapshot().Contains("seconds in this step"), "Plain-language stage/elapsed progress missing");
        Task.WaitAll(Enumerable.Range(0, 4).Select(i => Task.Run(() => { for(var j=0;j<100;j++) { progress.Begin("Checking connections"); Check(progress.Snapshot().Contains("Your map has not changed"), "Concurrent progress snapshot corrupted"); } })).ToArray());
        progress.Begin("Choosing tree locations");
        Check(progress.Snapshot().Contains("Choosing tree locations") && !progress.Snapshot().Contains("Smoothing roads"), "Previous stage retained");
        Check(!progress.Snapshot(false).Contains("map has not changed"), "Application falsely claims the map is unchanged");
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
        var ramps=new List<PreparedSegment>();
        for(var i=0;i<20;i++)
        {
            var a=i*.035f;var b=(i+1)*.035f;
            ramps.Add(Segment(500,500+i,501+i,150*MathF.Cos(a),150*MathF.Sin(a),150*MathF.Cos(b),150*MathF.Sin(b)) with{Classification="motorway_link",OneWay=true});
        }
        var smoothRamps=NetworkNormalizer.Normalize(ramps,log,default);
        Check(smoothRamps.Count<ramps.Count&&smoothRamps[0].StartNodeId==500&&smoothRamps[^1].EndNodeId==520,"Ramp chain stayed jagged or lost endpoints");
        NetworkNormalizer.Validate(smoothRamps,14336,default);
        var rampJunction=ramps.Concat(new[]{Segment(501,510,900,ramps[10].Start.x,ramps[10].Start.z,200,200)}).ToArray();
        var junctionRamps=NetworkNormalizer.Normalize(rampJunction,log,default);
        Check(junctionRamps.Any(s=>s.WayId==500&&s.EndNodeId==510)&&junctionRamps.Any(s=>s.WayId==500&&s.StartNodeId==510),"Ramp smoothing erased an exit junction");
        Check(NetworkNormalizer.Normalize(hairpin.Select(s=>s with{Classification="motorway_link",OneWay=true}).ToArray(),log,default).Count==3,"Ramp smoothing rounded an intentional corner");
        var parallel = new[] { Segment(1,1,2,0,0,100,0,NetworkKind.Rail), Segment(2,3,4,0,3,100,3,NetworkKind.Rail) };
        Check(NetworkNormalizer.Normalize(parallel,log,default).Count==2,"Parallel rail removed");
        var reverseDuplicates = new[] { Segment(1,1,2,0,0,100,0), Segment(2,2,1,100,0,0,0) };
        Check(NetworkNormalizer.Normalize(reverseDuplicates,log,default).Count == 1,"Reversed equivalent duplicate not removed");
        var separated = new[] { Segment(1,1,2,0,0,100,0), Segment(2,1,2,0,0,100,0) with { Bridge=true, Layer=1 } };
        Check(NetworkNormalizer.Normalize(separated,log,default).Count==2,"Bridge-separated infrastructure removed");
        using(var cancelled=new CancellationTokenSource()) { cancelled.Cancel(); try { NetworkNormalizer.Normalize(source,log,cancelled.Token); throw new Exception("Cancellation ignored"); } catch(OperationCanceledException) {} }
        // Mixed Overpass output emits tagged ways/trees before supporting skeleton
        // nodes, in quadtile order rather than ID order. Parsing must not depend on order.
        var xml=Encoding.UTF8.GetBytes("<osm><way id='1'><nd ref='1'/><nd ref='2'/><tag k='highway' v='primary'/><tag k='lanes' v='4'/><tag k='bridge' v='yes'/><tag k='layer' v='1'/></way><node id='1' lat='0' lon='0'><tag k='natural' v='tree'/><tag k='diameter_crown' v='8'/></node><node id='bad' lat='0' lon='0'/><node id='2' lat='0' lon='.01'/></osm>");
        var dataset=OsmParser.Merge(Enumerable.Repeat(xml,16).ToArray(),log); Check(dataset.Nodes.Count==2 && dataset.Ways.Count==1,"Malformed feature prevented valid parsing");
        Check(dataset.Ways[1].NodeIds.All(dataset.Nodes.ContainsKey)&&dataset.Ways[1].Tags["lanes"]=="4"&&dataset.Ways[1].Tags["bridge"]=="yes"&&dataset.Ways[1].Tags["layer"]=="1"&&dataset.NodeTags[1]["natural"]=="tree","Mixed tagged/skeleton output lost geometry, structure or tree metadata");
        try { OsmParser.Merge(new[]{xml},log); throw new Exception("Incomplete coverage accepted"); } catch(System.IO.InvalidDataException) {}
        var cacheRoot=System.IO.Path.Combine(System.IO.Path.GetTempPath(),"iel-parsed-test-"+Guid.NewGuid().ToString("N"));
        try
        {
            var docs=Enumerable.Repeat(xml,16).ToArray();
            var first=OsmParsedCache.Merge(docs,cacheRoot,log,default);
            var second=OsmParsedCache.Merge(docs,cacheRoot,log,default);
            Check(log.Counts["Parsed OSM cache hits"]==1&&log.Counts["Parsed OSM cache writes"]==1,"Second parse did not use its binary cache");
            Check(first.Nodes.OrderBy(p=>p.Key).SequenceEqual(second.Nodes.OrderBy(p=>p.Key)),"Cached node coordinates changed");
            Check(first.NodeTags[1]["natural"]=="tree"&&second.NodeTags[1]["NATURAL"]=="tree"&&second.NodeTags[1]["diameter_crown"]=="8","Mapped tree tags lost during XML/cache round trip");
            var untaggedLast=docs.ToArray(); untaggedLast[15]=Encoding.UTF8.GetBytes("<osm><node id='1' lat='0' lon='0'/></osm>");
            Check(OsmParser.Merge(untaggedLast,log).NodeTags[1]["natural"]=="tree","Untagged duplicate erased mapped-tree tags");
            Check(first.Ways[1].NodeIds.SequenceEqual(second.Ways[1].NodeIds)&&second.Ways[1].Tags["HIGHWAY"]=="primary","Cached references/tags changed");
            Check(first.MalformedFeaturesSkipped==second.MalformedFeaturesSkipped,"Cache lost malformed feature diagnostics");
            var cacheFile=System.IO.Directory.GetFiles(System.IO.Path.Combine(cacheRoot,"parsed-osm")).Single();
            var wholeParsed=OsmParsedCache.Merge(new[]{xml},cacheRoot,log,default,wholeMap:true);
            Check(wholeParsed.Nodes.Count==2&&wholeParsed.Ways.Count==1,"Whole-map coverage was not parsed correctly");
            System.IO.File.WriteAllBytes(cacheFile,new byte[]{1,2,3});
            Check(OsmParsedCache.Merge(docs,cacheRoot,log,default).Nodes.Count==2,"Corrupt parsed cache did not fall back to XML");
            using var cancelledCache=new CancellationTokenSource();cancelledCache.Cancel();
            try { OsmParsedCache.Merge(docs,cacheRoot,log,cancelledCache.Token);throw new Exception("Cached parse ignored cancellation"); } catch(OperationCanceledException) {}
            var changed=docs.ToArray();changed[15]=Encoding.UTF8.GetBytes("<osm><node id='3' lat='0' lon='0'/></osm>");
            Check(OsmParsedCache.Merge(changed,cacheRoot,log,default).Nodes.Count==3,"Changed XML reused stale parsed cache");
            var relationsXml=Encoding.UTF8.GetBytes("<osm><relation id='7'><member type='way' ref='1' role='outer'/><member type='way' ref='2' role='inner'/><tag k='natural' v='wood'/></relation></osm>");
            var relationsDocs=Enumerable.Repeat(relationsXml,16).ToArray();
            OsmParsedCache.Merge(relationsDocs,cacheRoot,log,default);
            var relation=OsmParsedCache.Merge(relationsDocs,cacheRoot,log,default).Relations.Single();
            Check(relation.Id==7&&relation.Members.SequenceEqual(new (long,bool)[]{(1,false),(2,true)})&&relation.Tags["NATURAL"]=="wood","Cached polygon membership/roles changed");
        }
        finally { if(System.IO.Directory.Exists(cacheRoot))System.IO.Directory.Delete(cacheRoot,true); }
        var index=new ForestProcessor.CorridorIndex();
        var indexedSegments=new[]{(new float3(-100,0,-50),new float3(100,0,50),17f),(new float3(0,0,0),new float3(0,0,0),14f)};
        foreach(var section in indexedSegments)index.Add(section.Item1,section.Item2,section.Item3,default);
        for(var x=-150;x<=150;x+=5)for(var z=-150;z<=150;z+=5)
        {
            var p=new float3(x,0,z);Check(index.Contains(p)==indexedSegments.Any(s=>NetworkNormalizer.DistanceToSegment(p,s.Item1,s.Item2)<=s.Item3),"Spatial clearance differs from exhaustive check");
        }
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
        Check(trees.Count>100 && trees.All(p=>Math.Abs(p.x)<=request.MapSize.WidthMeters/2 && Math.Abs(p.z)<=request.MapSize.WidthMeters/2),"Forest candidates not generated within map bounds");
        Check(trees.Count>1500,"Woodland density was not substantially increased from 30m cells");
        Check(trees.All(p => Math.Abs(p.x)>100 || Math.Abs(p.z)>100),"Forest hole filled");
        Check(trees.All(p=>Math.Abs(p.z)>17),"Road corridor planted");
        Check(trees.SequenceEqual(ForestProcessor.Prepare(forest,request,terrain,corridor,log,default)),"Tree generation nondeterministic");
        var smallMap=request with{MapSize=new MapSizeProfile("test","test",600,false)};
        Check(ForestProcessor.Prepare(forest,smallMap,terrain,corridor,log,default).All(p=>Math.Abs(p.x)<=300 && Math.Abs(p.z)<=300),"Trees not clipped to map bounds");
        for(var i=0;i<trees.Count;i++) for(var j=i+1;j<trees.Count;j++) Check(math.distance(trees[i].xz,trees[j].xz)>=ForestProcessor.MinimumSpacing,"Tree spacing violated");
        Check(ForestProcessor.Prepare(forest,request,terrain with{SeaLevelMeters=60},corridor,log,default).Count==0,"Underwater forest planted");
        TreeTests.Run();
        Console.WriteLine("PASS: straight/gentle curves, strict deviation, protected junctions/sharp bends, parallel rail, cancellation, malformed features, complete coverage, deterministic forest masks/holes/corridors/spacing/water, exact +1m correction");
    }
}
namespace ImportEarthLocation.Core
{
    public sealed class ImportLog { public Dictionary<string,int> Counts=new(); public List<string> Diagnostics=new(); public void Info(string x){} public void Warn(string x){} public void Diagnostic(string x){Diagnostics.Add(x);} public void Count(string x,int count=1){Counts.TryGetValue(x,out var old);Counts[x]=old+count;} public void Time(string x,double seconds){} }
}
namespace ImportEarthLocation.Processing
{
    public static class TerrainBuilder { public const int Resolution=16; public sealed record Result(ushort[] Heightmap,float HeightScaleMeters,float SeaLevelMeters,float MinimumElevation,float MaximumElevation); }
}
