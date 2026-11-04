using ImportEarthLocation.Core;
using ImportEarthLocation.Geo;
using ImportEarthLocation.Processing;
using System.Globalization;
using System.IO.Compression;
using System.Text;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using Unity.Mathematics;

// Report 64 Phases 1–2: visual metrics on a design's final output, after the same
// native units and roadbed the game would receive. The failures file lets
// clustering be revised without repeating the design; the plot file lets two
// pipelines be drawn side by side at fixed locations.
static partial class VisualMetricsReplay
{
    private const string FailuresFile="visual-failures.ndjson.gz";
    private const string RunFile="visual-metrics-run.json";
    private const string PlotFile="plot-sections.ndjson.gz";
    private const double Earth=6378137;

    private sealed record Capture(string Folder,JObject Input,PreparedSegment[] Source,NativeAssetContract[] Contracts,Func<float3,float?> Water,
        float3[][] Areas,float3[][] Lines,ushort[] Terrain,float Scale,double Width);

    private static Capture Load(string folder)
    {
        var input=JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder,"offline-design-pass-01-input.json")));
        if((int?)input["schemaVersion"]!=2||(string?)input["geometryEncoding"]!="prepared-segment-exact-v1"||
            (string?)input["waterSampler"] is not ("WaterBodyPlanner.SurfaceSampler" or "WaterBodyPlanner.ConstructedSurfaceSampler"))throw new InvalidDataException("Unknown full-context replay contract");
        T[] Read<T>(string name)=>CorrectiveCapturedReplay.ReadManifest<T>(folder,(string)input[name]!);
        var source=Read<PreparedSegment>("networkManifest");if(source.Length!=(int)input["sections"]!)throw new InvalidDataException("Source count mismatch");
        var cell=CapturedRunFiles.WaterCellSize(folder,input);
        var water=new WaterPlan(false,0,0,0,"captured"){WaterCellSize=cell,AreaStages=Read<WaterAreaStage>("waterAreasManifest"),ReachStages=Read<WaterReachStage>("waterReachesManifest")};
        var sampler=WaterBodyPlanner.ConstructedSurfaceSampler(water,Read<GeographicPolygon>("waterPolygonsManifest"));
        var bytes=File.ReadAllBytes(CapturedRunFiles.Resolve(folder,(string)input["terrainFile"]!));
        if(bytes.Length%2!=0||!BitConverter.IsLittleEndian)throw new InvalidDataException("Unsupported terrain encoding");
        var terrain=new ushort[bytes.Length/2];Buffer.BlockCopy(bytes,0,terrain,0,bytes.Length);TerrainSampling.Resolution(terrain);
        return new(folder,input,source,Read<NativeAssetContract>("nativeContractsManifest"),sampler,Read<float3[]>("protectedAreasManifest"),
            Read<float3[]>("protectedLinesManifest"),terrain,(float)input["HeightScaleMeters"]!,(double)input["widthMeters"]!);
    }

    public static void Run(string folder,string output)=>Measure(Load(folder),output,"legacy-coordinated",(c,log)=>
    {
        var watch=System.Diagnostics.Stopwatch.StartNew();var nextProgress=0d;
        return CoordinatedNetworkDesign.Prepare(c.Source,c.Terrain,c.Scale,c.Width,log,default,c.Water,true,c.Contracts.ToDictionary(a=>a.PaletteKey),
            progress:message=>{if(watch.Elapsed.TotalSeconds>=nextProgress){Console.WriteLine(message);nextProgress=watch.Elapsed.TotalSeconds+10;}},
            protectedAreas:c.Areas,protectedLines:c.Lines,sourceTopology:SourceTopologyObligations.Capture(c.Source));
    });

    // Lane-count smoothing on a captured network: the summary, and the lane
    // sequence of the named road's sections in travel order before and after.
    public static void LaneRuns(string folder,string name)
    {
        var c=Load(folder);var log=new ImportLog();var smoothed=NetworkLaneRuns.Smooth(c.Source,log);
        foreach(var line in log.Lines.Where(l=>l.Contains("Lane counts:",StringComparison.Ordinal)))Console.WriteLine("  "+line);
        foreach(var group in Enumerable.Range(0,c.Source.Length).Where(i=>c.Source[i].Name==name).GroupBy(i=>c.Source[i].OneWay?Math.Sign(c.Source[i].End.x-c.Source[i].Start.x):0))
        {
            var order=group.OrderBy(i=>c.Source[i].Start.x*(group.Key<0?-1:1)).ToArray();
            Console.WriteLine($"  direction {group.Key}: before {string.Join("",order.Select(i=>c.Source[i].Lanes))}");
            Console.WriteLine($"  direction {group.Key}: after  {string.Join("",order.Select(i=>smoothed[i].Lanes))}");
        }
    }

    // Every plan crossing between a section of the given ways and any other
    // section in a plot file, with both heights at the crossing point.
    public static void Crossings(string plot,IReadOnlyCollection<long> ways)
    {
        var sections=new List<(long Way,string Class,bool Bridge,bool Tunnel,float3[] Points)>();
        using(var reader=new StreamReader(new GZipStream(File.OpenRead(plot),CompressionMode.Decompress)))
            for(string? line;(line=reader.ReadLine())!=null;)
            {
                var o=JObject.Parse(line);var p=o["p"]!.Select(v=>(float)v).ToArray();
                sections.Add(((long)o["WayId"]!,(string)o["Classification"]!,(bool)o["Bridge"]!,(bool)o["Tunnel"]!,Enumerable.Range(0,p.Length/3).Select(k=>new float3(p[3*k],p[3*k+1],p[3*k+2])).ToArray()));
            }
        foreach(var u in sections.Where(s=>ways.Contains(s.Way)))
            foreach(var o in sections.Where(s=>!ways.Contains(s.Way)))
                for(var i=1;i<u.Points.Length;i++)for(var j=1;j<o.Points.Length;j++)
                {
                    float2 a=u.Points[i-1].xz,b=u.Points[i].xz,c=o.Points[j-1].xz,d=o.Points[j].xz;var r=b-a;var q=d-c;var den=r.x*q.y-r.y*q.x;
                    if(Math.Abs(den)<1e-6f)continue;var t=((c.x-a.x)*q.y-(c.y-a.y)*q.x)/den;var v=((c.x-a.x)*r.y-(c.y-a.y)*r.x)/den;
                    if(t<0||t>1||v<0||v>1)continue;
                    var yu=math.lerp(u.Points[i-1].y,u.Points[i].y,t);var yo=math.lerp(o.Points[j-1].y,o.Points[j].y,v);
                    Console.WriteLine($"  {u.Way}{(u.Bridge?" bridge":u.Tunnel?" tunnel":"")} y={yu:F1} x {o.Way} {o.Class}{(o.Bridge?" bridge":o.Tunnel?" tunnel":"")} y={yo:F1}; gap={yo-yu:F1} at ({a.x+t*r.x:F0},{a.y+t*r.y:F0})");
                }
    }

    // Time the bridge/tunnel interval step (water sampler construction and the
    // per-section water test) on a captured map, as the import runs it.
    public static void IntervalTiming(string folder)
    {
        var watch=System.Diagnostics.Stopwatch.StartNew();var c=Load(folder);var load=watch.Elapsed.TotalSeconds;
        var input=c.Input;T[] Read<T>(string name)=>CorrectiveCapturedReplay.ReadManifest<T>(folder,(string)input[name]!);
        var water=new WaterPlan(false,0,0,0,"captured"){AreaStages=Read<WaterAreaStage>("waterAreasManifest"),ReachStages=Read<WaterReachStage>("waterReachesManifest")};
        var polygons=Read<GeographicPolygon>("waterPolygonsManifest");
        var ground=c.Source.Select(s=>s with{Bridge=false}).Where(s=>!s.Tunnel).ToArray();
        // Unfiltered and prefiltered samplers back to back, with every query counted
        // and every answer compared, so the result is independent of machine load.
        var plain=WaterBodyPlanner.SurfaceSampler(water,polygons,reference:true);
        watch.Restart();var sampler=WaterBodyPlanner.SurfaceSampler(water,polygons);var build=watch.Elapsed.TotalSeconds;
        long queries=0,differ=0;var answers=new List<float?>();
        watch.Restart();CoordinatedNetworkDesign.StructureIntervals(ground,c.Terrain,c.Scale,c.Width,new ImportLog(),default,p=>{var v=plain(p);answers.Add(v);return v;},tunnelIntervalsResolved:true);var plainSeconds=watch.Elapsed.TotalSeconds;
        watch.Restart();CoordinatedNetworkDesign.StructureIntervals(ground,c.Terrain,c.Scale,c.Width,new ImportLog(),default,p=>null,tunnelIntervalsResolved:true);
        Console.WriteLine($"  intervals with no water at all {watch.Elapsed.TotalSeconds:F2}s (the cost outside the sampler)");
        var k=0;watch.Restart();var result=CoordinatedNetworkDesign.StructureIntervals(ground,c.Terrain,c.Scale,c.Width,new ImportLog(),default,p=>{var v=sampler(p);queries++;if(k<answers.Count&&!Nullable.Equals(answers[k++],v))differ++;return v;},tunnelIntervalsResolved:true);
        Console.WriteLine($"  load {load:F1}s; sampler build {build:F2}s; intervals unfiltered {plainSeconds:F2}s, prefiltered {watch.Elapsed.TotalSeconds:F2}s; {queries:N0} queries, {differ} answers differ; {ground.Length} sections ({result.Count(s=>s.Bridge)} over water); areas {water.AreaStages.Count}, reaches {water.ReachStages.Count}, polygons {polygons.Length}");
    }

    // Report 69: rock candidates for a captured map from a rock-query OSM file,
    // against the captured terrain, design roads and water polygons.
    public static void RockPlan(string folder,string osm)
    {
        var c=Load(folder);var request=c.Input["request"]!.ToObject<ImportRequest>()! with{ImportRocks=true};
        // Several tiles (separated by |) are parsed one by one and combined.
        var tiles=osm.Split('|').Select(f=>OsmParser.Merge(new[]{File.ReadAllBytes(f)},new ImportLog(),default,wholeMap:true)).ToArray();
        var data=tiles.Length==1?tiles[0]:new OsmDataset{Nodes=tiles.SelectMany(d=>d.Nodes).GroupBy(n=>n.Key).ToDictionary(g=>g.Key,g=>g.First().Value),
            Ways=tiles.SelectMany(d=>d.Ways).GroupBy(w=>w.Key).ToDictionary(g=>g.Key,g=>g.First().Value),
            NodeTags=tiles.SelectMany(d=>d.NodeTags).GroupBy(n=>n.Key).ToDictionary(g=>g.Key,g=>g.First().Value)};
        var water=c.Input["waterPolygonsManifest"]!=null?CorrectiveCapturedReplay.ReadManifest<GeographicPolygon>(folder,(string)c.Input["waterPolygonsManifest"]!):Array.Empty<GeographicPolygon>();
        var terrain=new TerrainBuilder.Result(c.Terrain,c.Scale,0,0,c.Scale);
        var log=new ImportLog();var watch=System.Diagnostics.Stopwatch.StartNew();
        var rocks=RockPlanner.Prepare(data,request,terrain,c.Source,water,new WaterPlan(false,0,0,0,"replay"),log,default);
        Console.WriteLine($"  rocks={rocks.Count} in {watch.Elapsed.TotalSeconds:F1}s; small={rocks.Count(r=>r.Form==VegetationForm.RockSmall)}; medium={rocks.Count(r=>r.Form==VegetationForm.RockMedium)}; large={rocks.Count(r=>r.Form==VegetationForm.RockLarge)}");
        foreach(var line in log.Lines.Where(l=>l.StartsWith("Rocks prepared",StringComparison.Ordinal)))Console.WriteLine("  "+line);
        // Ordered fingerprint of every candidate, to compare implementations exactly.
        var fingerprint=new System.Text.StringBuilder();
        foreach(var r in rocks)fingerprint.Append(FormattableString.Invariant($"{BitConverter.SingleToInt32Bits(r.Position.x)},{BitConverter.SingleToInt32Bits(r.Position.y)},{BitConverter.SingleToInt32Bits(r.Position.z)},{r.Form},{r.SourceId},{r.Seed},{r.MetadataIndex};"));
        Console.WriteLine($"  candidate fingerprint {Convert.ToHexString(System.Security.Cryptography.SHA256.HashData(System.Text.Encoding.UTF8.GetBytes(fingerprint.ToString())))[..16]}");
        var below=rocks.Count(r=>c.Water(r.Position) is float stage&&r.Position.y<stage);
        Console.WriteLine($"  rocks below a captured water surface: {below}");
        // ROCK_NEAR=lat,lon: rocks within 150 m of a point by reason and size,
        // and the rock-related OSM features covering or passing near it.
        if(Environment.GetEnvironmentVariable("ROCK_NEAR") is string near)
        {
            var parts=near.Split(',').Select(v=>double.Parse(v,CultureInfo.InvariantCulture)).ToArray();
            var at=ImportEarthLocation.Geo.GeoMath.ToLocalMeters(parts[0],parts[1],request.Latitude,request.Longitude);
            var close=rocks.Where(r=>math.distance(r.Position.xz,at.xz)<150).ToArray();
            Console.WriteLine($"  within 150 m of {near}: {close.Length} rocks ({close.Count(r=>r.Form==VegetationForm.RockLarge)} boulders); terrain {NetworkProcessor.Sample(at.x,at.z,c.Terrain,c.Scale,c.Width):0.0} m");
            foreach(var g in close.GroupBy(r=>RockPlanner.Reasons[r.MetadataIndex]))Console.WriteLine($"    {g.Key}: {g.Count()}");
            foreach(var p in GeographyPolygons.Build(data,request,new ImportLog(),default).Where(p=>p.Contains(at)))
                Console.WriteLine($"    inside polygon {p.Id}: {string.Join(", ",p.Tags.Where(t=>t.Key is "natural" or "surface" or "landuse" or "name").Select(t=>t.Key+"="+t.Value))}");
            foreach(var w in data.Ways.Values.Where(w=>w.Tags.TryGetValue("natural",out var n)&&n=="cliff"))
            {
                var pts=w.NodeIds.Where(data.Nodes.ContainsKey).Select(id=>ImportEarthLocation.Geo.GeoMath.ToLocalMeters(data.Nodes[id].Latitude,data.Nodes[id].Longitude,request.Latitude,request.Longitude)).ToArray();
                var d=pts.Zip(pts.Skip(1),(a,b)=>NetworkNormalizer.DistanceToSegment(at,a,b)).DefaultIfEmpty(float.PositiveInfinity).Min();
                if(d<300)Console.WriteLine($"    cliff way {w.Id} at {d:0} m");
            }
        }
        // Where they are: the densest 2 km squares, with their centre's latitude/longitude.
        var lat0=(double)request.Latitude;var lon0=(double)request.Longitude;
        foreach(var g in rocks.GroupBy(r=>((int)Math.Floor(r.Position.x/2000),(int)Math.Floor(r.Position.z/2000))).OrderByDescending(g=>g.Count()).Take(15))
        {
            var x=(g.Key.Item1+.5)*2000;var z=(g.Key.Item2+.5)*2000;
            var lat=lat0+math.degrees(z/Earth);var lon=lon0+math.degrees(x/(Earth*Math.Cos(math.radians(lat0))));
            Console.WriteLine($"  square {lat:0.000},{lon:0.000}: {g.Count():N0} rocks ({g.Count(r=>r.Form==VegetationForm.RockLarge):N0} boulders)");
        }
    }

    // Beaches against the captured terrain: how much of each mapped beach lies
    // at or below a water level, and how wide it is (twice area over perimeter).
    public static void BeachCheck(string folder,string osm,float waterLevel)
    {
        var c=Load(folder);var request=c.Input["request"]!.ToObject<ImportRequest>()!;
        // Several tiles (separated by |) are parsed one by one and combined.
        var tiles=osm.Split('|').Select(f=>OsmParser.Merge(new[]{File.ReadAllBytes(f)},new ImportLog(),default,wholeMap:true)).ToArray();
        var data=tiles.Length==1?tiles[0]:new OsmDataset{Nodes=tiles.SelectMany(d=>d.Nodes).GroupBy(n=>n.Key).ToDictionary(g=>g.Key,g=>g.First().Value),
            Ways=tiles.SelectMany(d=>d.Ways).GroupBy(w=>w.Key).ToDictionary(g=>g.Key,g=>g.First().Value),
            NodeTags=tiles.SelectMany(d=>d.NodeTags).GroupBy(n=>n.Key).ToDictionary(g=>g.Key,g=>g.First().Value)};
        var beaches=GeographyPolygons.Build(data,request,new ImportLog(),default).Where(p=>p.Tags.TryGetValue("natural",out var n)&&n=="beach").ToArray();
        if(Environment.GetEnvironmentVariable("BEACH_LIFT")=="1")
        {
            var liftLog=new ImportLog();
            c=c with{Terrain=BeachTerrain.Lift(new TerrainBuilder.Result(c.Terrain,c.Scale,waterLevel-WaterLevels.CorrectionMeters,0,c.Scale),c.Width,beaches,liftLog,default)};
            foreach(var line in liftLog.Lines.Where(l=>l.StartsWith("Beaches:",StringComparison.Ordinal)))Console.WriteLine("  "+line);
        }
        var widths=new List<double>();var under=0L;var total=0L;var drowned=0;var rows=new List<string>();
        foreach(var b in beaches)
        {
            var ring=b.Outer.OrderByDescending(r=>r.Length).First();
            double area=0,perimeter=0;for(var i=0;i<ring.Length;i++){var p=ring[i];var q=ring[(i+1)%ring.Length];area+=p.x*q.z-q.x*p.z;perimeter+=math.distance(p.xz,q.xz);}
            area=Math.Abs(area)/2;var width=perimeter>0?2*area/perimeter:0;widths.Add(width);
            var lo=ring.Select(p=>p.xz).Aggregate(math.min);var hi=ring.Select(p=>p.xz).Aggregate(math.max);long n=0,wet=0;var heights=new List<float>();
            for(var z=lo.y;z<=hi.y;z+=3)for(var x=lo.x;x<=hi.x;x+=3)
            {
                var p=new float3(x,0,z);if(!b.Contains(p))continue;n++;var y=NetworkProcessor.Sample(x,z,c.Terrain,c.Scale,c.Width);heights.Add(y);if(y<=waterLevel)wet++;
            }
            total+=n;under+=wet;if(n>0&&wet*2>n)drowned++;
            if(n>0)heights.Sort();
            rows.Add($"{b.Id} surface={(b.Tags.TryGetValue("surface",out var s)?s:"-")} width={width:F0}m area={area:F0}m² samples={n} underWater={(n==0?0:100.0*wet/n):F0}% heightP10={(n==0?0:heights[(int)(.1*(n-1))]):F1} median={(n==0?0:heights[(int)(.5*(n-1))]):F1}");
        }
        widths.Sort();double Q(double q)=>widths.Count==0?0:widths[(int)(q*(widths.Count-1))];
        Console.WriteLine($"  beaches={beaches.Length}; widthP10={Q(.1):F0} m; median={Q(.5):F0} m; P90={Q(.9):F0} m; samples under {waterLevel} m: {(total==0?0:100.0*under/total):F1}%; beaches mostly under water: {drowned}");
        foreach(var row in rows.OrderBy(r=>r).Take(Environment.GetEnvironmentVariable("BEACH_ROWS") is string k&&int.TryParse(k,out var m)?m:0))Console.WriteLine("  "+row);
    }

    // Shaded relief of a captured heightmap window (centre x,z in local metres),
    // one image pixel per heightmap pixel scaled up, water polygon edges in blue,
    // so stair-stepped banks can be seen. Light from the north-west.
    // Hillshades of one patch at the game's 14 m pixel: zoom-12 DEM bilinear,
    // zoom-12 clamped bicubic, zoom-13 clamped bicubic. Each input is a 768×768
    // float mosaic of the 3×3 Terrarium tiles around (latitude, longitude).
    public static void ResampleDemo(string z12,string z13,double latitude,double longitude,string output)
    {
        const int mosaic=768;const double outputSpacing=14;
        var size=int.TryParse(Environment.GetEnvironmentVariable("DEMO_SIZE"),out var demoSize)?demoSize:192;var scale=int.TryParse(Environment.GetEnvironmentVariable("DEMO_SCALE"),out var demoScale)?demoScale:3;
        float[] Load(string path){var bytes=File.ReadAllBytes(path);var f=new float[bytes.Length/4];Buffer.BlockCopy(bytes,0,f,0,bytes.Length);return f;}
        var sources=new[]{(Data:Load(z12),Zoom:12),(Data:Load(z13),Zoom:13)};
        var groundPerPixel=40075016.68557849*Math.Cos(latitude*Math.PI/180);
        var panels=new List<float[]>();
        foreach(var (data,zoom,cubic) in new[]{(sources[0].Data,12,false),(sources[0].Data,12,true),(sources[1].Data,13,true)})
        {
            var centreX=ImportEarthLocation.Geo.GeoMath.LongitudeToTileX(longitude,zoom)*256;var centreY=ImportEarthLocation.Geo.GeoMath.LatitudeToTileY(latitude,zoom)*256;
            var originX=(Math.Floor(centreX/256)-1)*256;var originY=(Math.Floor(centreY/256)-1)*256;
            var step=outputSpacing/(groundPerPixel/(256d*(1<<zoom)));
            float Read(int x,int y)=>data[Math.Clamp(y,0,mosaic-1)*mosaic+Math.Clamp(x,0,mosaic-1)];
            var heights=new float[size*size];
            for(var j=0;j<size;j++)for(var i=0;i<size;i++)
            {
                var px=centreX-originX+(i-size/2)*step;var py=centreY-originY+(j-size/2)*step;var x=(int)Math.Floor(px);var y=(int)Math.Floor(py);var fx=(float)(px-x);var fy=(float)(py-y);
                heights[j*size+i]=cubic?TerrainSampling.ClampedCubic(Read,x,y,TerrainSampling.CubicWeights(fx),TerrainSampling.CubicWeights(fy)):
                    (Read(x,y)*(1-fx)+Read(x+1,y)*fx)*(1-fy)+(Read(x,y+1)*(1-fx)+Read(x+1,y+1)*fx)*fy;
            }
            panels.Add(heights);
        }
        var w=(size*scale+8)*panels.Count;var h=size*scale;var rgb=new byte[w*h*3];
        for(var p=0;p<panels.Count;p++)for(var j=1;j<size-1;j++)for(var i=1;i<size-1;i++)
        {
            var a=panels[p];var dx=(a[j*size+i+1]-a[j*size+i-1])/(2*outputSpacing);var dz=(a[(j-1)*size+i]-a[(j+1)*size+i])/(2*outputSpacing);
            var normal=math.normalize(new double3(-dx,1,-dz));var shade=(byte)(255*Math.Clamp(math.dot(normal,math.normalize(new double3(-1,1.2,1)))*1.1,0,1));
            for(var sz=0;sz<scale;sz++)for(var sx=0;sx<scale;sx++){var k=((j*scale+sz)*w+p*(size*scale+8)+i*scale+sx)*3;rgb[k]=rgb[k+1]=rgb[k+2]=shade;}
        }
        WritePng(output,w,h,rgb);
        Console.WriteLine($"  resample demo {size}×{size} pixels at {outputSpacing} m: zoom-12 bilinear | zoom-12 bicubic | zoom-13 bicubic → {output}");
    }

    public static void TerrainCrop(string folder,float cx,float cz,float size,string output)
    {
        var c=Load(folder);
        // TERRAIN_FILE renders another heightmap of the run, such as final-design-heightmap.u16.
        if(Environment.GetEnvironmentVariable("TERRAIN_FILE") is string other&&other.Length>0)
        {var raw=File.ReadAllBytes(Path.Combine(folder,other));var t=new ushort[raw.Length/2];Buffer.BlockCopy(raw,0,t,0,raw.Length);c=c with{Terrain=t};}
        var n=(int)Math.Sqrt(c.Terrain.Length);var spacing=c.Width/(n-1);
        int Index(double v)=>(int)Math.Round((v/c.Width+.5)*(n-1));
        var x0=Math.Max(1,Index(cx-size/2));var x1=Math.Min(n-2,Index(cx+size/2));var z0=Math.Max(1,Index(cz-size/2));var z1=Math.Min(n-2,Index(cz+size/2));
        var scale=Math.Max(1,(int)(1024/(x1-x0+1)));var w=(x1-x0+1)*scale;var h=(z1-z0+1)*scale;var rgb=new byte[w*h*3];
        float H(int x,int z)=>c.Terrain[z*n+x]/65535f*c.Scale;
        var water=c.Input["waterPolygonsManifest"]!=null?CorrectiveCapturedReplay.ReadManifest<GeographicPolygon>(folder,(string)c.Input["waterPolygonsManifest"]!):Array.Empty<GeographicPolygon>();
        for(var z=z0;z<=z1;z++)for(var x=x0;x<=x1;x++)
        {
            var dx=(H(x+1,z)-H(x-1,z))/(2*spacing);var dz=(H(x,z+1)-H(x,z-1))/(2*spacing);
            var normal=math.normalize(new double3(-dx,1,-dz));var shade=Math.Clamp(math.dot(normal,math.normalize(new double3(-1,1.2,1)))*1.1,0,1);
            var y=H(x,z);var tint=(y-H(x0,z0))/40;
            var r=(byte)(255*shade);var g=(byte)(255*shade);var b=(byte)(255*shade);
            for(var sz=0;sz<scale;sz++)for(var sx=0;sx<scale;sx++)
            {
                // Image row 0 is north: larger z first.
                var px=(x-x0)*scale+sx;var py=(z1-z)*scale+(scale-1-sz);var k=(py*w+px)*3;rgb[k]=r;rgb[k+1]=g;rgb[k+2]=b;
            }
        }
        foreach(var polygon in water)foreach(var ring in polygon.Outer.Concat(polygon.Inner))for(var i=0;i<ring.Length;i++)
        {
            var a=ring[i];var b=ring[(i+1)%ring.Length];var steps=(int)Math.Ceiling(math.distance(a.xz,b.xz)/(spacing/scale))+1;
            for(var s=0;s<=steps;s++)
            {
                var p=math.lerp(a,b,s/(float)steps);var fx=((p.x/c.Width+.5)*(n-1)-x0+.5)*scale;var fz=((p.z/c.Width+.5)*(n-1)-z0+.5)*scale;
                var px=(int)fx;var py=h-1-(int)fz;if(px<0||py<0||px>=w||py>=h)continue;var k=(py*w+px)*3;rgb[k]=40;rgb[k+1]=90;rgb[k+2]=255;
            }
        }
        WritePng(output,w,h,rgb);
        Console.WriteLine($"  crop {x1-x0+1}×{z1-z0+1} heightmap pixels ({spacing:F1} m each) at {scale}× → {output}");
    }

    // Pixel "teeth": a grid point standing more than threshold metres off the
    // mean of its four neighbours, the signature of a height step written per
    // pixel along a slanted edge. Compares an earlier and a later heightmap of
    // one run (file names inside the run folder) over a centred square, prints
    // counts and the densest 500 m tiles, and draws the later relief with teeth
    // present in both maps in yellow and teeth new in the later one in red.
    public static void Teeth(string folder,string earlier,string later,float half,float threshold,string output)
    {
        ushort[] Read(string name){var raw=File.ReadAllBytes(Path.Combine(folder,name));var t=new ushort[raw.Length/2];Buffer.BlockCopy(raw,0,t,0,raw.Length);return t;}
        var run=JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder,"offline-design-pass-01-input.json")));
        var scale=(float)run["HeightScaleMeters"]!;var width=(double)run["widthMeters"]!;
        var a=Read(earlier);var b=Read(later);var n=(int)Math.Sqrt(b.Length);var spacing=width/(n-1);
        int Index(double v)=>(int)Math.Round((v/width+.5)*(n-1));
        var lo=Math.Max(1,Index(-half));var hi=Math.Min(n-2,Index(half));var w=hi-lo+1;var rgb=new byte[w*w*3];
        float Lap(ushort[] m,int x,int z)=>(m[z*n+x]-(m[z*n+x-1]+m[z*n+x+1]+m[(z-1)*n+x]+m[(z+1)*n+x])/4f)/65535f*scale;
        long both=0,fresh=0,gone=0;var tiles=new Dictionary<(int,int),(int Both,int Fresh)>();
        for(var z=lo;z<=hi;z++)for(var x=lo;x<=hi;x++)
        {
            float H(int xx,int zz)=>b[zz*n+xx]/65535f*scale;
            var dx=(H(x+1,z)-H(x-1,z))/(2*spacing);var dz=(H(x,z+1)-H(x,z-1))/(2*spacing);
            var shade=Math.Clamp(math.dot(math.normalize(new double3(-dx,1,-dz)),math.normalize(new double3(-1,1.2,1)))*1.1,0,1);
            var inA=Math.Abs(Lap(a,x,z))>threshold;var inB=Math.Abs(Lap(b,x,z))>threshold;
            var k=((hi-z)*w+(x-lo))*3;var g=(byte)(255*shade);rgb[k]=g;rgb[k+1]=g;rgb[k+2]=g;
            if(inB){var key=((int)Math.Floor((x*spacing-width/2)/500),(int)Math.Floor((z*spacing-width/2)/500));tiles.TryGetValue(key,out var c);
                if(inA){both++;rgb[k]=255;rgb[k+1]=210;rgb[k+2]=0;c.Both++;}else{fresh++;rgb[k]=255;rgb[k+1]=30;rgb[k+2]=30;c.Fresh++;}tiles[key]=c;}
            else if(inA)gone++;
        }
        WritePng(output,w,w,rgb);
        Console.WriteLine($"  teeth over {threshold} m within ±{half} m: in both {both}; new in {later} {fresh}; removed {gone} → {output}");
        foreach(var t in tiles.OrderByDescending(t=>t.Value.Both+t.Value.Fresh).Take(25))
            Console.WriteLine($"    tile x={t.Key.Item1*500+250} z={t.Key.Item2*500+250}: both {t.Value.Both}, new {t.Value.Fresh}");
    }

    // River bank containment on the captured water terrain: for every 192 m of
    // each accepted reach, the planner's two dry bank probes (first point at
    // least two 28 m water cells out whose diagonal footprint is dry) and the
    // stage's margin below the lower one. WaterBodyPlanner.Revalidate rejects a
    // source whose margin is under 0.25 m.
    public static void WaterBanks(string folder)
    {
        var c=Load(folder);
        float H(float3 p)=>NetworkProcessor.Sample(p.x,p.z,c.Terrain,c.Scale,c.Width);
        var reaches=CorrectiveCapturedReplay.ReadManifest<WaterReachStage>(folder,(string)c.Input["waterReachesManifest"]!);
        var polygons=CorrectiveCapturedReplay.ReadManifest<GeographicPolygon>(folder,(string)c.Input["waterPolygonsManifest"]!).ToDictionary(p=>p.Identity);
        const float cell=28;var margins=new List<(float Margin,float3 P,long Way,bool Polygon,float Bank,float Stage)>();
        foreach(var r in reaches)
        {
            var length=math.distance(r.Start.xz,r.End.xz);if(length<.01f)continue;
            var normal=math.normalizesafe(new float3(-(r.End-r.Start).z,0,(r.End-r.Start).x));
            polygons.TryGetValue(r.PolygonId??long.MinValue,out var polygon);
            Func<float3,bool> wet=polygon!=null?polygon.Contains:q=>NetworkNormalizer.DistanceToSegment(q,r.Start,r.End)<=r.HalfWidth;
            bool Dry(float3 q)=>!wet(q)&&!wet(q+new float3(42,0,42))&&!wet(q+new float3(-42,0,42))&&!wet(q+new float3(42,0,-42))&&!wet(q+new float3(-42,0,-42));
            for(var s=0f;s<length;s+=192)
            {
                var t=s/length;var p=math.lerp(r.Start,r.End,t);var stage=math.lerp(r.StartLevel,r.EndLevel,t);var bank=float.PositiveInfinity;
                foreach(var side in new[]{-1,1})
                {
                    var start=Math.Max(cell,r.HalfWidth);
                    while(start<512&&wet(p+normal*(start*side)))start+=Math.Max(cell,r.HalfWidth*.5f);
                    for(var d=Math.Max(start,cell*2);d<=1024;d+=cell){var q=p+normal*(d*side);if(Dry(q)){bank=Math.Min(bank,H(q));break;}}
                }
                if(math.isfinite(bank))margins.Add((bank-stage,p,r.WayId,polygon!=null,bank,stage));
            }
        }
        foreach(var kind in new[]{false,true})
        {
            var m=margins.Where(x=>x.Polygon==kind).Select(x=>x.Margin).OrderBy(x=>x).ToArray();if(m.Length==0)continue;
            Console.WriteLine($"  {(kind?"polygon":"centerline")} samples {m.Length}: margin < .25 m {m.Count(x=>x<.25f)} ({m.Count(x=>x<.25f)/(double)m.Length:P0}); < .6 {m.Count(x=>x<.6f)}; < 1 {m.Count(x=>x<1)}; median {m[m.Length/2]:0.00} m; p10 {m[m.Length/10]:0.00} m");
        }
        // WATER_NEAR=x,z lists water polygons with a vertex within 300 m.
        if(Environment.GetEnvironmentVariable("WATER_NEAR") is string near&&near.Length>0)
        {
            var at=near.Split(',').Select(v=>float.Parse(v,CultureInfo.InvariantCulture)).ToArray();var o=new float2(at[0],at[1]);
            foreach(var p in polygons.Values.Where(p=>p.Outer.Concat(p.Inner).Any(r=>r.Any(q=>math.distance(q.xz,o)<300))))
                Console.WriteLine($"    near polygon {p.Identity} {string.Join(",",p.Tags.Select(t=>t.Key+"="+t.Value))}; holes {p.Inner.Count}; contains {p.Contains(new float3(o.x,0,o.y))}");
        }
        // WATER_NAME=text lists water polygons whose name contains it.
        if(Environment.GetEnvironmentVariable("WATER_NAME") is string named&&named.Length>0)
            foreach(var p in polygons.Values.Where(p=>p.Tags.TryGetValue("name",out var v)&&v.Contains(named,StringComparison.OrdinalIgnoreCase)))
            {var all=p.Outer.SelectMany(r=>r).ToArray();Console.WriteLine($"    polygon {p.Identity} {string.Join(",",p.Tags.Select(t=>t.Key+"="+t.Value))}: x {all.Min(q=>q.x):0}..{all.Max(q=>q.x):0} z {all.Min(q=>q.z):0}..{all.Max(q=>q.z):0}");}
        // WATER_OSM=parsed-osm cache file groups the samples by waterway type and
        // prints the traced way's tags.
        if(Environment.GetEnvironmentVariable("WATER_OSM") is string osm&&osm.Length>0)
        {
            var ways=OsmParsedCache.LoadSnapshot(osm,default).Ways;
            string Kind(long id)=>ways.TryGetValue(id,out var w)?(w.Tags.TryGetValue("waterway",out var t)?t:"?")+(w.Tags.ContainsKey("abandoned")||w.Tags.ContainsKey("disused")?"+abandoned":"")+(w.Tags.ContainsKey("tunnel")?"+tunnel":""):"missing";
            foreach(var g in margins.GroupBy(x=>Kind(x.Way)).OrderByDescending(g=>g.Count()))
                Console.WriteLine($"    {g.Key}: {g.Count()} samples on {g.Select(x=>x.Way).Distinct().Count()} ways, margin < .25 m {g.Count(x=>x.Margin<.25f)}, < -2 m {g.Count(x=>x.Margin<-2)}");
            if(long.TryParse(Environment.GetEnvironmentVariable("WATER_WAY"),out var tagged)&&ways.TryGetValue(tagged,out var way))
                Console.WriteLine($"    way {tagged}: {string.Join(", ",way.Tags.Select(t=>t.Key+"="+t.Value))}; {way.NodeIds.Length} nodes");
        }
        // WATER_WAY=id prints that way's reaches: stage, centreline ground and the lower bank.
        if(long.TryParse(Environment.GetEnvironmentVariable("WATER_WAY"),out var traced))
            foreach(var r in reaches.Where(r=>r.WayId==traced))
                Console.WriteLine($"    ({r.Start.x:0},{r.Start.z:0})→({r.End.x:0},{r.End.z:0}) stage {r.StartLevel:0.00}→{r.EndLevel:0.00} ground {H(r.Start):0.00} half {r.HalfWidth:0} bank {margins.Where(x=>x.Way==traced&&math.distance(x.P.xz,r.Start.xz)<1).Select(x=>x.Bank.ToString("0.00")).FirstOrDefault()}");
        foreach(var x in margins.Where(x=>x.Margin<.25f).OrderBy(x=>x.Margin).Take(40))
            Console.WriteLine($"    way {x.Way} at ({x.P.x:0},{x.P.z:0}) {(x.Polygon?"polygon":"line")}: stage {x.Stage:0.00} bank {x.Bank:0.00} margin {x.Margin:0.00}");
        // WATER_SECTION=x,z prints the ground across the reach nearest that point.
        if(Environment.GetEnvironmentVariable("WATER_SECTION") is string section&&section.Length>0)
        {
            var at=section.Split(',').Select(v=>float.Parse(v,CultureInfo.InvariantCulture)).ToArray();var o=new float3(at[0],0,at[1]);
            var r=reaches.OrderBy(r=>NetworkNormalizer.DistanceToSegment(o,r.Start,r.End)).First();var normal=math.normalizesafe(new float3(-(r.End-r.Start).z,0,(r.End-r.Start).x));
            Console.WriteLine($"    way {r.WayId} stage {r.StartLevel:0.00}: "+string.Join(" ",Enumerable.Range(-10,21).Select(k=>$"{k*14}:{H(o+normal*(k*14)):0.0}")));
        }
    }

    // Replays WaterBodyPlanner on a capture. The capture keeps no pre-water DEM,
    // so ground within each waterway's carving reach (channel/.6 plus a cell) is
    // restored by harmonic inpainting of the captured terrain around it, outside
    // mapped water polygons. Prints the planner's evidence and counters, the
    // sources Revalidate keeps on the planned terrain alone, and channel
    // continuity: walking each open river, stream or canal downstream every
    // 7 m, a sample is dammed when the bed on its line stands over 0.1 m above
    // the lowest planned water stage upstream on that way (water from upstream
    // cannot pass), and cut depth is the restored ground minus the bed.
    public static void WaterReplay(string folder,string osm,float seaLevel,string? reportPath=null)
    {
        // Water replay does not use native contracts or the historical water
        // sampler. Read its terrain directly instead of building that unrelated
        // index; the optional crossing audit reads the exact source separately.
        var input=JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder,"offline-design-pass-01-input.json")));
        if((int?)input["schemaVersion"]!=2)throw new InvalidDataException("Unknown water replay capture");
        var terrainBytes=File.ReadAllBytes(CapturedRunFiles.Resolve(folder,(string)input["terrainFile"]!));
        if(terrainBytes.Length%2!=0||!BitConverter.IsLittleEndian)throw new InvalidDataException("Unsupported terrain encoding");
        var heights=new ushort[terrainBytes.Length/2];Buffer.BlockCopy(terrainBytes,0,heights,0,terrainBytes.Length);
        var c=(Input:input,Terrain:heights,Scale:(float)input["HeightScaleMeters"]!,Width:(double)input["widthMeters"]!);
        var n=TerrainSampling.Resolution(c.Terrain);var width=(float)c.Width;var half=width/2;var spacing=width/(n-1);
        var cell=CapturedRunFiles.WaterCellSize(folder,input);
        var request=c.Input["request"]!;
        var importRequest=new ImportRequest((double)request["Latitude"]!,(double)request["Longitude"]!,new MapSizeProfile("replay","replay",c.Width,false),false);
        var data=OsmParsedCache.LoadSnapshot(osm,default);
        var polygons=CorrectiveCapturedReplay.ReadManifest<GeographicPolygon>(folder,(string)c.Input["waterPolygonsManifest"]!);
        float3 Local(long id){var p=data.Nodes[id];return GeoMath.ToLocalMeters(p.Latitude,p.Longitude,importRequest.Latitude,importRequest.Longitude);}
        var types=new[]{"river","stream","canal","drain","ditch"};
        var lines=data.Ways.Values.Where(w=>w.Tags.TryGetValue("waterway",out var t)&&types.Contains(t)&&w.NodeIds.Length>1&&w.NodeIds.All(data.Nodes.ContainsKey)).ToArray();
        var restore=new bool[n*n];
        foreach(var w in lines)
        {
            var reach=(NetworkEvidence.Measure(w.Tags,"width")??(w.Tags["waterway"]=="river"?32:12))*.5f;var radius=Math.Max(reach,cell)/.6f+spacing;
            for(var i=1;i<w.NodeIds.Length;i++)
            {
                var a=Local(w.NodeIds[i-1]);var b=Local(w.NodeIds[i]);
                int Index(float v)=>(int)Math.Round((v/width+.5)*(n-1));
                var x0=Math.Max(1,Index(Math.Min(a.x,b.x)-radius));var x1=Math.Min(n-2,Index(Math.Max(a.x,b.x)+radius));
                var z0=Math.Max(1,Index(Math.Min(a.z,b.z)-radius));var z1=Math.Min(n-2,Index(Math.Max(a.z,b.z)+radius));
                for(var z=z0;z<=z1;z++)for(var x=x0;x<=x1;x++)
                    if(NetworkNormalizer.DistanceToSegment(new float3(x*spacing-half,0,z*spacing-half),a,b)<=radius)restore[z*n+x]=true;
            }
        }
        var wetIndex=polygons.Select(p=>(Polygon:p,Min:p.Outer.SelectMany(r=>r).Aggregate(new float2(float.MaxValue),(m,q)=>math.min(m,q.xz)),Max:p.Outer.SelectMany(r=>r).Aggregate(new float2(float.MinValue),(m,q)=>math.max(m,q.xz)))).ToArray();
        var cells=new List<int>();
        for(var i=0;i<restore.Length;i++)
        {
            if(!restore[i])continue;var q=new float3(i%n*spacing-half,0,i/n*spacing-half);
            if(wetIndex.Any(p=>q.x>=p.Min.x&&q.z>=p.Min.y&&q.x<=p.Max.x&&q.z<=p.Max.y&&p.Polygon.Contains(q))){restore[i]=false;continue;}
            cells.Add(i);
        }
        var ground=c.Terrain.Select(v=>(float)v).ToArray();
        // Start from the highest neighbouring untouched ground so the iteration
        // converges from above instead of keeping the carved channel.
        for(var iteration=0;iteration<400;iteration++)
            foreach(var i in cells)ground[i]=(ground[i-1]+ground[i+1]+ground[i-n]+ground[i+n])/4;
        var original=ground.Select(v=>(ushort)Math.Clamp(Math.Round(v),0,65535)).ToArray();
        Console.WriteLine($"  restored {cells.Count:N0} cells near {lines.Length:N0} waterways");
        var terrain=new TerrainBuilder.Result(original,c.Scale,seaLevel,0,c.Scale);
        var log=new ImportLog();var watch=System.Diagnostics.Stopwatch.StartNew();
        var plan=WaterBodyPlanner.Prepare(data,polygons,importRequest,terrain,log,default,cell);
        var traceDams=int.TryParse(Environment.GetEnvironmentVariable("WATER_DAMS"),out var shown)?shown:0;
        float Bed(ushort[] map,float3 p)=>NetworkProcessor.Sample(p.x,p.z,map,c.Scale,c.Width);
        Console.WriteLine($"  planned in {watch.Elapsed.TotalSeconds:0.0} s");
        foreach(var line in log.Lines.Where(l=>l.StartsWith("Rivers:")||l.StartsWith("Water:")))Console.WriteLine("  "+line);
        var planned=plan.ConditionedHeightmap??original;
        IReadOnlyList<PreparedSegment>? networks=null;object? crossingReport=null;
        if(reportPath!=null)
        {
            var captured=CorrectiveCapturedReplay.ReadManifest<PreparedSegment>(folder,(string)input["networkManifest"]!);
            if(captured.Length!=(int)input["sections"]!)throw new InvalidDataException("Captured source count mismatch");
            var channels=WaterBodyPlanner.BuriedChannels(data,importRequest,cell).Concat(WaterBodyPlanner.OpenChannels(data,importRequest,cell)).ToArray();
            networks=CoordinatedNetworkDesign.CulvertBridges(captured,channels,n,width,log,default);
            crossingReport=WaterCrossings(captured,networks,channels,n,width);
        }
        var revalidated=WaterBodyPlanner.Revalidate(plan,terrain with{Heightmap=planned},c.Width,log,default,networks);
        Console.WriteLine($"  sources {plan.Sources.Count:N0}, kept by Revalidate {revalidated.Sources.Count:N0} (rivers {plan.Sources.Count(s=>s.BodyKind=="river"):N0} → {revalidated.Sources.Count(s=>s.BodyKind=="river"):N0})");
        var keptIds=revalidated.Sources.Select(s=>(s.BodyId,s.Position.x,s.Position.z)).ToHashSet();
        foreach(var s in plan.Sources.Where(s=>!keptIds.Contains((s.BodyId,s.Position.x,s.Position.z))).Take(traceDams))
        {
            var low=s.BankProbes.OrderBy(q=>Bed(planned,q)).First();
            Console.WriteLine($"    rejected {s.BodyKind} {s.BodyId} at ({s.Position.x:0},{s.Position.z:0}): stage {s.TargetSurfaceLevel:0.00} bank at planning {s.BankLevel:0.00}; lowest probe ({low.x:0},{low.z:0}) planned {Bed(planned,low):0.00} restored {Bed(original,low):0.00}; {s.BankProbes.Count} probes");
        }
        foreach(var kind in new[]{"lake","river"})
            Console.WriteLine($"    {kind} bodies with a source {plan.Sources.Where(s=>s.BodyKind==kind).Select(s=>s.BodyId).Distinct().Count():N0}, still with one after Revalidate {revalidated.Sources.Where(s=>s.BodyKind==kind).Select(s=>s.BodyId).Distinct().Count():N0}");
        foreach(var name in new[]{"Water stage bank conflicts","Water source footprint policy rejected","Water locally infeasible reaches","Water final bank conflicts","River raster components reseeded","Water stage uphill residual edges",
            "Water endpoint gaps connected","Water buried waterways opened","Water buried channel metres","Lake containment bodies checked","Lake containment levels lowered","Lake containment bodies omitted"})
            Console.WriteLine($"    {name}: {log.Metric(name)}");
        long samples=0,dammed=0,dams=0;var cuts=new List<float>();var byType=new Dictionary<string,(long Samples,long Dammed)>();
        var stagesByWay=plan.ReachStages.GroupBy(r=>r.WayId).ToDictionary(g=>g.Key,g=>g.ToArray());
        foreach(var w in lines.Where(w=>!(w.Tags.TryGetValue("tunnel",out var t)&&t!="no")&&!(w.Tags.TryGetValue("intermittent",out var i)&&i=="yes")&&
            w.Tags["waterway"] is "river" or "stream"&&!w.Tags.ContainsKey("abandoned")&&!w.Tags.ContainsKey("disused")))
        {
            var lowest=float.PositiveInfinity;var blocked=false;var reaches=stagesByWay.TryGetValue(w.Id,out var list)?list:Array.Empty<WaterReachStage>();
            for(var i=1;i<w.NodeIds.Length;i++)
            {
                var a=Local(w.NodeIds[i-1]);var b=Local(w.NodeIds[i]);if(Math.Max(Math.Abs(a.x),Math.Abs(a.z))>half-100||Math.Max(Math.Abs(b.x),Math.Abs(b.z))>half-100)continue;
                var steps=Math.Max(1,(int)(math.distance(a.xz,b.xz)/7));
                for(var k=0;k<steps;k++)
                {
                    var p=math.lerp(a,b,k/(float)steps);var bed=Bed(planned,p);samples++;cuts.Add(Bed(original,p)-bed);
                    // Upstream water surface: the planned stage of the nearest reach.
                    var reach=reaches.Where(r=>NetworkNormalizer.DistanceToSegment(p,r.Start,r.End)<30).OrderBy(r=>NetworkNormalizer.DistanceToSegment(p,r.Start,r.End)).FirstOrDefault();
                    var dam=bed>lowest+.1f;byType.TryGetValue(w.Tags["waterway"],out var tally);tally.Samples++;if(dam)tally.Dammed++;byType[w.Tags["waterway"]]=tally;
                    if(dam){dammed++;if(!blocked){dams++;if(traceDams-->0)
                        Console.WriteLine($"    dam on {w.Tags["waterway"]} {w.Id} at ({p.x:0},{p.z:0}): bed {bed:0.0} over upstream low {lowest:0.0}; ground {Bed(original,p):0.0}; reach stage {plan.ReachStages.Where(r=>r.WayId==w.Id).OrderBy(r=>math.distancesq(r.Start.xz,p.xz)).Select(r=>r.StartLevel.ToString("0.0")).FirstOrDefault()??"none"}");}}
                    blocked=dam;if(reach!=null)lowest=Math.Min(lowest,Math.Min(reach.StartLevel,reach.EndLevel));
                }
            }
        }
        cuts.Sort();
        Console.WriteLine($"  open waterways: {samples:N0} samples, dammed {dammed:N0} ({dammed/(double)Math.Max(1,samples):P1}) at {dams:N0} dams; cut depth median {cuts[cuts.Count/2]:0.0} m, p95 {cuts[(int)(cuts.Count*.95)]:0.0} m, over 10 m {cuts.Count(v=>v>10):N0}");
        Console.WriteLine("    dammed by type: "+string.Join(", ",byType.OrderByDescending(t=>t.Value.Samples).Select(t=>$"{t.Key} {t.Value.Dammed/(double)t.Value.Samples:P1} of {t.Value.Samples:N0}")));
        // The planned water surface along each way, in the way's order: rises
        // over 5 cm are uphill water; stages under the sea's surface (sea level
        // plus the game's 1 m) would let the sea run up the channel.
        var uphill=0;var reachCount=0;
        foreach(var g in plan.ReachStages.GroupBy(r=>r.WayId))
        {var previousLevel=float.PositiveInfinity;foreach(var r in g){reachCount++;if(r.StartLevel>previousLevel+.05f)uphill++;if(r.EndLevel>r.StartLevel+.05f)uphill++;previousLevel=r.EndLevel;}}
        Console.WriteLine($"  water surface: {reachCount:N0} reaches, rising downstream {uphill:N0}; below the sea's surface {plan.ReachStages.Count(r=>Math.Min(r.StartLevel,r.EndLevel)<seaLevel+1-.05f):N0}");
        var wayType=lines.ToDictionary(w=>w.Id,w=>w.Tags["waterway"]);
        foreach(var g in plan.ReachStages.Where(r=>Bed(original,r.Start)-r.StartLevel>15).GroupBy(r=>r.WayId).OrderByDescending(g=>g.Count()).Take(shown))
            Console.WriteLine($"    deep: {g.Count()} reaches on way {g.Key}: {string.Join(", ",lines.First(w=>w.Id==g.Key).Tags.Select(t=>t.Key+"="+t.Value))}");
        foreach(var g in plan.ReachStages.Where(r=>Math.Min(r.StartLevel,r.EndLevel)<seaLevel+1-.05f).GroupBy(r=>r.WayId).OrderByDescending(g=>g.Count()).Take(shown))
            Console.WriteLine($"    below sea: {(wayType.TryGetValue(g.Key,out var t)?t:"?")} {g.Key} {g.Count()} reaches, lowest {g.Min(r=>Math.Min(r.StartLevel,r.EndLevel)):F1} at ({g.First().Start.x:F0},{g.First().Start.z:F0}), polygon {g.First().PolygonId}");
        Console.WriteLine("  deepest cuts by waterway: "+string.Join(", ",lines.Where(w=>w.Tags["waterway"] is "river" or "stream" or "canal").GroupBy(w=>w.Tags["waterway"]).Select(g=>
            {var ids=g.Select(w=>w.Id).ToHashSet();return $"{g.Key} {plan.ReachStages.Count(r=>ids.Contains(r.WayId)&&Bed(original,r.Start)-r.StartLevel>15):N0} reaches over 15 m under ground";})));
        Console.WriteLine("  river sources by waterway: "+string.Join(", ",plan.Sources.Where(s=>s.BodyKind=="river").GroupBy(s=>wayType.TryGetValue(s.WayId,out var t)?t:"?").OrderByDescending(g=>g.Count()).Select(g=>$"{g.Key} {g.Count():N0}")));
        if(reportPath!=null)
        {
            var buriedIds=lines.Where(w=>w.Tags.TryGetValue("tunnel",out var tag)&&tag!="no").Select(w=>w.Id).ToHashSet();
            long buriedSamples=0,buriedObstructions=0;
            foreach(var r in plan.ReachStages.Where(r=>buriedIds.Contains(r.WayId)))
            {
                var count=Math.Max(1,(int)Math.Ceiling(math.distance(r.Start.xz,r.End.xz)/7));
                for(var k=0;k<count;k++)
                {
                    var t=k/(float)count;var p=math.lerp(r.Start,r.End,t);buriedSamples++;
                    if(Bed(planned,p)>math.lerp(r.StartLevel,r.EndLevel,t)-.1f)buriedObstructions++;
                }
            }
            var report=new{capture=Path.GetFileName(folder),sourceCache=osm,waterCellSize=cell,terrainReconstructed=true,nativeFlowVerified=false,
                sources=revalidated.Sources.Count,riverControls=revalidated.Sources.Count(s=>s.BodyKind=="river"),
                streamControls=revalidated.Sources.Count(s=>wayType.GetValueOrDefault(s.WayId)=="stream"),
                streamReaches=revalidated.ReachStages.Count(s=>wayType.GetValueOrDefault(s.WayId)=="stream"),
                containment=log.Lines.Where(s=>s.Contains("WATER CONTAINMENT")||s.Contains("Lake ")).ToArray(),
                lakes=polygons.Where(p=>p.Outer.Any(r=>r.Length>0)&&!p.Tags.ContainsKey("waterway")).Select(p=>
                {
                    var vertices=p.Outer.SelectMany(r=>r).ToArray();var center=vertices.Aggregate(float3.zero,(a,b)=>a+b)/vertices.Length;
                    return new{p.Id,p.Identity,p.Tags,latitude=importRequest.Latitude+math.degrees(center.z/Earth),
                        longitude=importRequest.Longitude+math.degrees(center.x/(Earth*Math.Cos(math.radians(importRequest.Latitude)))),
                        controls=revalidated.Sources.Count(s=>s.BodyId==p.Identity&&s.BodyKind=="lake"),
                        stages=revalidated.AreaStages.Where(s=>s.BodyId==p.Identity).Select(s=>s.Level).ToArray()};
                }).ToArray(),
                lakeBodies=revalidated.Sources.Where(s=>s.BodyKind=="lake").Select(s=>s.BodyId).Distinct().Count(),
                riverBodies=revalidated.Sources.Where(s=>s.BodyKind=="river").Select(s=>s.BodyId).Distinct().Count(),
                openedBuriedWays=log.Metric("Water buried waterways opened"),buriedSamples,buriedObstructions,
                naturalOpenSamples=samples,naturalOpenObstructions=dammed,uphillStageTransitions=uphill,crossings=crossingReport};
            File.WriteAllText(reportPath,JsonConvert.SerializeObject(report,Formatting.Indented));
            Console.WriteLine("  crossing report: "+JsonConvert.SerializeObject(crossingReport));
        }
        if(Environment.GetEnvironmentVariable("WATER_REPLAY_OUT") is string output&&output.Length>0)
        {
            var bytes=new byte[planned.Length*2];Buffer.BlockCopy(planned,0,bytes,0,bytes.Length);File.WriteAllBytes(output,bytes);
            var restored=new byte[original.Length*2];Buffer.BlockCopy(original,0,restored,0,restored.Length);File.WriteAllBytes(Path.ChangeExtension(output,".restored.u16"),restored);
        }
    }

    // Top-down road footprints of two plot files side by side at one spot, at
    // native width: grey where one road covers the ground, red where two roads
    // of the same level overlap, blue for bridges and tunnels.
    public static void PlanPng(string before,string after,double lat,double lon,double half,string output)
    {
        var run=JObject.Parse(File.ReadAllText(Path.Combine(before,RunFile)));double lat0=(double)run["latitude"]!,lon0=(double)run["longitude"]!;
        var cx=math.radians(lon-lon0)*Earth*Math.Cos(math.radians(lat0));var cz=math.radians(lat-lat0)*Earth;
        const int size=700;var metres=2*half/size;var width=size*2+10;var rgb=new byte[width*size*3];Array.Fill(rgb,(byte)30);
        for(var side=0;side<2;side++)
        {
            var coverage=new byte[size*size];var structure=new bool[size*size];
            using(var file=File.OpenRead(Path.Combine(side==0?before:after,PlotFile)))using(var zip=new GZipStream(file,CompressionMode.Decompress))using(var reader=new StreamReader(zip))
            while(reader.ReadLine() is {} line)
            {
                var r=JObject.Parse(line);if((string)r["kind"]!!="Road")continue;var p=r["p"]!.Values<double>().ToArray();var radius=(float)r["NativeWidth"]!/2;
                var curve=new Cubic3d(new double3(p[0],p[1],p[2]),new double3(p[3],p[4],p[5]),new double3(p[6],p[7],p[8]),new double3(p[9],p[10],p[11]));
                if(Math.Min(Math.Min(p[0],p[9]),Math.Min(p[3],p[6]))>cx+half+50||Math.Max(Math.Max(p[0],p[9]),Math.Max(p[3],p[6]))<cx-half-50||
                   Math.Min(Math.Min(p[2],p[11]),Math.Min(p[5],p[8]))>cz+half+50||Math.Max(Math.Max(p[2],p[11]),Math.Max(p[5],p[8]))<cz-half-50)continue;
                var mine=new HashSet<int>();var steps=Math.Max(2,(int)(NetworkDesignChecks.HorizontalArcLength(curve)/.5));var bridge=(bool)r["Bridge"]!||(bool)r["Tunnel"]!;
                for(var s=0;s<=steps;s++)
                {
                    var q=NetworkDesignChecks.Evaluate(curve,s/(double)steps);var px=(q.x-(cx-half))/metres;var pz=(cz+half-q.z)/metres;var rr=radius/metres;
                    for(var y=(int)(pz-rr);y<=(int)(pz+rr);y++)for(var x=(int)(px-rr);x<=(int)(px+rr);x++)
                        if(x>=0&&y>=0&&x<size&&y<size&&(x-px)*(x-px)+(y-pz)*(y-pz)<=rr*rr)mine.Add(y*size+x);
                }
                foreach(var i in mine){if(bridge)structure[i]=true;else if(coverage[i]<255)coverage[i]++;}
            }
            for(var y=0;y<size;y++)for(var x=0;x<size;x++)
            {
                var i=y*size+x;var k=(y*width+side*(size+10)+x)*3;
                if(coverage[i]>=2){rgb[k]=220;rgb[k+1]=50;rgb[k+2]=50;}else if(coverage[i]==1){rgb[k]=150;rgb[k+1]=150;rgb[k+2]=150;}
                if(structure[i]){rgb[k]=(byte)(rgb[k]/2+40);rgb[k+1]=(byte)(rgb[k+1]/2+70);rgb[k+2]=(byte)Math.Min(255,rgb[k+2]/2+150);}
            }
        }
        WritePng(output,width,size,rgb);Console.WriteLine($"  plan {2*half:0} m square, {metres:0.00} m per pixel → {output}");
    }

    private static void WritePng(string path,int width,int height,byte[] rgb)
    {
        using var file=File.Create(path);
        void Chunk(string type,byte[] data)
        {
            var header=Encoding.ASCII.GetBytes(type);var length=BitConverter.GetBytes(data.Length);Array.Reverse(length);file.Write(length);
            file.Write(header);file.Write(data);var crc=Crc(header.Concat(data));var bytes=BitConverter.GetBytes(crc);Array.Reverse(bytes);file.Write(bytes);
        }
        static uint Crc(IEnumerable<byte> data){var c=0xFFFFFFFFu;foreach(var b in data){c^=b;for(var k=0;k<8;k++)c=(c&1)!=0?0xEDB88320u^(c>>1):c>>1;}return c^0xFFFFFFFFu;}
        file.Write(new byte[]{137,80,78,71,13,10,26,10});
        var ihdr=new byte[13];var wb=BitConverter.GetBytes(width);var hb=BitConverter.GetBytes(height);Array.Reverse(wb);Array.Reverse(hb);
        Array.Copy(wb,0,ihdr,0,4);Array.Copy(hb,0,ihdr,4,4);ihdr[8]=8;ihdr[9]=2;Chunk("IHDR",ihdr);
        using var raw=new MemoryStream();
        using(var z=new ZLibStream(raw,CompressionLevel.Optimal,true))for(var y=0;y<height;y++){z.WriteByte(0);z.Write(rgb,y*width*3,width*3);}
        Chunk("IDAT",raw.ToArray());Chunk("IEND",Array.Empty<byte>());
    }

    public static void Compiler(string folder,string output)
    {
        var capture=Load(folder);
        if(Environment.GetEnvironmentVariable("VISUAL_RESOLVE_WATER_INTERVALS")=="1")
            capture=capture with{Source=CoordinatedNetworkDesign.StructureIntervals(capture.Source,capture.Terrain,capture.Scale,capture.Width,new ImportLog(),default,capture.Water).ToArray()};
        Measure(capture,output,"visual-compiler",
        (c,log)=>{if(Environment.GetEnvironmentVariable("VISUAL_DEBUG_SHORT")=="1")ShortSources(c.Source);
            // Captures predating the step have no bridges over unmapped highway
            // crossings; on newer captures it finds nothing more to do.
            var source=CoordinatedNetworkDesign.UnmappedCrossingBridges(c.Source,log,default);
            // The capture is taken after culvert bridges, so re-running them
            // with the planned river reaches standing in for the open channels
            // shows what later changes to that step add.
            if(Environment.GetEnvironmentVariable("VISUAL_CULVERTS")=="1")
            {
                var reaches=CorrectiveCapturedReplay.ReadManifest<WaterReachStage>(c.Folder,(string)c.Input["waterReachesManifest"]!);
                source=CoordinatedNetworkDesign.CulvertBridges(source,reaches.Where(r=>r.PolygonId==null).Select(r=>(new[]{r.Start,r.End},r.HalfWidth)).ToArray(),(int)Math.Sqrt(c.Terrain.Length),c.Width,log,default);
            }
            if(Environment.GetEnvironmentVariable("VISUAL_PRUNE")=="1")
            {
                var pruneLog=new ImportLog();source=NetworkStubs.Prune(source,c.Input["request"]!.ToObject<ImportRequest>()!,pruneLog).ToArray();
                foreach(var line in pruneLog.Lines.Where(l=>l.StartsWith("Odd connector",StringComparison.Ordinal)))Console.WriteLine("  "+line);
            }
            return VisualNetworkCompiler.Prepare(source,c.Terrain,c.Scale,c.Width,log,default,c.Water);});
    }

    // Why a ground road misses the stamped terrain: the cut/fill clamp, a
    // structure end nearby, another road at a different height sharing raster
    // cells, or neither (raster resolution).
    private static void TerrainCauses(Capture c,IReadOnlyList<PreparedSegment> sections,ushort[] roadbed,VisualCompilerMetrics.Measurement measured)
    {
        var points=new Dictionary<(int,int),List<(int Section,double3 P)>>();
        (int,int) Cell(double3 p)=>((int)Math.Floor(p.x/32),(int)Math.Floor(p.z/32));
        var ends=new List<double3>();
        for(var i=0;i<sections.Count;i++)
        {
            var s=sections[i];var curve=new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);
            var n=Math.Max(2,(int)(NetworkDesignChecks.HorizontalArcLength(curve)/5)+1);
            for(var k=0;k<n;k++){var p=NetworkDesignChecks.Evaluate(curve,k/(n-1d));if(!points.TryGetValue(Cell(p),out var l))points[Cell(p)]=l=new();l.Add((i,p));}
            if(NetworkStructure.IsStructure(s)){ends.Add(curve.P0);ends.Add(curve.P3);}
        }
        var counts=new Dictionary<string,int>();var excess=new Dictionary<string,double>();
        foreach(var f in measured.Failures.Where(f=>f.Category=="terrain-mismatch"))
        {
            var s=sections[f.Section];var p=f.Position;
            var original=NetworkProcessor.Sample((float)p.x,(float)p.z,c.Terrain,c.Scale,c.Width);
            var stamped=NetworkProcessor.Sample((float)p.x,(float)p.z,roadbed,c.Scale,c.Width);
            string cause;
            bool NearTunnel()
            {
                var (tx,tz)=Cell(p);
                for(var x=tx-1;x<=tx+1;x++)for(var z=tz-1;z<=tz+1;z++)if(points.TryGetValue((x,z),out var list))
                    foreach(var (j,q) in list)if(sections[j].Tunnel&&math.distance(q.xz,p.xz)<sections[j].NativeWidth/2+30)return true;
                return false;
            }
            if(stamped-original>1&&p.y<stamped-.3&&NearTunnel())
                cause="roof-cover"+(sections.Any(o=>o.Tunnel&&o.Kind==s.Kind&&(o.StartNodeId==s.StartNodeId||o.StartNodeId==s.EndNodeId||o.EndNodeId==s.StartNodeId||o.EndNodeId==s.EndNodeId))?" portal":" beside");
            else if(Math.Abs(p.y-original)>NetworkVerticalProfiles.CutFillBudget(s)-.1)cause="clamp";
            else if(ends.Any(e=>math.distance(e.xz,p.xz)<60))
                // Approach: this road leads onto the structure (a structure end
                // node on it within 60 m); otherwise it only passes near one.
                cause="structure-end"+(Environment.GetEnvironmentVariable("VISUAL_DEBUG_TERRAIN")!="2"?"":
                    sections.Any(o=>NetworkStructure.IsStructure(o)&&o.Kind==s.Kind&&new[]{(o.StartNodeId,o.CourseStart),(o.EndNodeId,o.CourseEnd)}
                        .Any(e=>(e.Item1==s.StartNodeId||e.Item1==s.EndNodeId)&&math.distance(((double3)e.Item2).xz,p.xz)<60))?" approach":" beside");
            else
            {
                var (cx,cz)=Cell(p);var shared=false;
                for(var x=cx-1;x<=cx+1&&!shared;x++)for(var z=cz-1;z<=cz+1&&!shared;z++)if(points.TryGetValue((x,z),out var list))
                    foreach(var (j,q) in list)
                    {
                        if(j==f.Section||NetworkStructure.IsStructure(sections[j]))continue;
                        if(math.distance(q.xz,p.xz)<s.NativeWidth/2+sections[j].NativeWidth/2+c.Width/Math.Sqrt(c.Terrain.Length)&&Math.Abs(q.y-p.y)>.3){shared=true;break;}
                    }
                cause=shared?"shared-cells":"resolution";
            }
            cause=(s.Kind==NetworkKind.Rail?"rail ":"")+cause+(p.y>stamped?"/above":"/below");
            // VISUAL_DEBUG_TERRAIN=2 lists the first structure-end failures.
            if(Environment.GetEnvironmentVariable("VISUAL_DEBUG_TERRAIN")=="2"&&cause.StartsWith("structure-end")&&(counts.TryGetValue(cause,out var seen)?seen:0)<5)
            {
                var nearest=sections.Where(NetworkStructure.IsStructure).Select(o=>(o,d:Math.Min(math.distance(((double3)o.CourseStart).xz,p.xz),math.distance(((double3)o.CourseEnd).xz,p.xz)))).OrderBy(x=>x.d).First();
                Console.WriteLine($"    {cause} way {s.WayId} {s.Classification} w{s.NativeWidth:F0} at ({p.x:F0},{p.z:F0}): road {p.y:F1} stamped {stamped:F1} original {original:F1}; {(nearest.o.Tunnel?"tunnel":"bridge")} end {nearest.d:F0} m (way {nearest.o.WayId}); section length {NetworkDesignChecks.HorizontalArcLength(new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd)):F0}");
            }
            counts[cause]=counts.TryGetValue(cause,out var n)?n+1:1;excess[cause]=(excess.TryGetValue(cause,out var e)?e:0)+f.Value;
        }
        foreach(var p in counts.OrderByDescending(p=>p.Value))Console.WriteLine($"  terrain cause {p.Key} = {p.Value}; mean {excess[p.Key]/p.Value:F2} m");
    }

    // What a same-level overlap is made of: the nearest other road at the worst
    // point, by direction (opposite carriageway, merge/diverge, crossing) and
    // class pair; tangent breaks and acute junctions by class.
    private static void OverlapCauses(IReadOnlyList<PreparedSegment> sections,VisualCompilerMetrics.Measurement measured)
    {
        var points=new Dictionary<(int,int),List<(int Section,double3 P,double2 D)>>();
        (int,int) Cell(double3 p)=>((int)Math.Floor(p.x/32),(int)Math.Floor(p.z/32));
        for(var i=0;i<sections.Count;i++)
        {
            var s=sections[i];if(s.Kind!=NetworkKind.Road)continue;var curve=new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);
            var n=Math.Max(2,(int)(NetworkDesignChecks.HorizontalArcLength(curve)/2)+1);
            for(var k=0;k<n;k++){var t=k/(n-1d);var p=NetworkDesignChecks.Evaluate(curve,t);var d=math.normalizesafe(NetworkDesignChecks.Derivative(curve,t).xz);
                if(!points.TryGetValue(Cell(p),out var l))points[Cell(p)]=l=new();l.Add((i,p,d));}
        }
        var counts=new Dictionary<string,int>();
        void Add(string key)=>counts[key]=counts.TryGetValue(key,out var n)?n+1:1;
        string Class(PreparedSegment s)=>s.ProtectedLink?"link":s.Classification;
        // As in the metric, sections within two graph steps are connected, not overlapping.
        var incident=new Dictionary<long,List<int>>();
        for(var i=0;i<sections.Count;i++)if(sections[i].Kind==NetworkKind.Road)foreach(var id in new[]{sections[i].StartNodeId,sections[i].EndNodeId})
        {if(!incident.TryGetValue(id,out var list))incident[id]=list=new();list.Add(i);}
        IEnumerable<int> Touching(int i)=>new[]{sections[i].StartNodeId,sections[i].EndNodeId}.SelectMany(id=>incident[id]);
        foreach(var f in measured.Failures)
        {
            var s=sections[f.Section];
            if(f.Category=="tangent-break"||f.Category=="acute-junction"){Add($"{f.Category} {Class(s)}");continue;}
            if(f.Category!="same-level-overlap")continue;
            var adjacent=new HashSet<int>(Touching(f.Section).SelectMany(Touching));
            var (cx,cz)=Cell(f.Position);var best=(Section:-1,Distance:double.PositiveInfinity,D:default(double2));var own=default(double2);
            for(var x=cx-1;x<=cx+1;x++)for(var z=cz-1;z<=cz+1;z++)if(points.TryGetValue((x,z),out var list))foreach(var q in list)
            {
                var d=math.distance(q.P.xz,f.Position.xz);
                if(q.Section==f.Section){if(d<3)own=q.D;continue;}
                if(adjacent.Contains(q.Section))continue;
                if(d<best.Distance&&Math.Abs(q.P.y-f.Position.y)<3)best=(q.Section,d,q.D);
            }
            if(best.Section<0){Add("overlap unknown");continue;}
            var o=sections[best.Section];var dot=Math.Abs(math.dot(own,best.D))<1e-6?0:math.dot(own,best.D);
            var relation=dot<-.9?"opposite":dot>.9?"parallel":Math.Abs(dot)<.5?"crossing":"angled";
            Add($"overlap {relation} {string.Join("+",new[]{Class(s),Class(o)}.OrderBy(c=>c))}{(s.OneWay&&o.OneWay?"":" (two-way)")}");
            // VISUAL_DEBUG_OVERLAP=2 also lists each parallel overlap with both roads' ways and end nodes.
            if(Environment.GetEnvironmentVariable("VISUAL_DEBUG_OVERLAP")=="2"&&relation=="parallel")
                Console.WriteLine($"    {Class(s)} way {s.WayId} {s.StartNodeId}->{s.EndNodeId} L{s.Lanes} w{s.NativeWidth:F0} / {Class(o)} way {o.WayId} {o.StartNodeId}->{o.EndNodeId} L{o.Lanes} w{o.NativeWidth:F0} at ({f.Position.x:F0},{f.Position.z:F0}) gap {best.Distance:F1} m");
            // VISUAL_DEBUG_OVERLAP=3 lists every overlap with the graph distance between the two sections.
            if(Environment.GetEnvironmentVariable("VISUAL_DEBUG_OVERLAP")=="3")
            {
                var hops=new Dictionary<int,int>{[f.Section]=0};var parent=new Dictionary<int,int>();var front=new List<int>{f.Section};
                for(var h=1;h<=8&&!hops.ContainsKey(best.Section);h++){var next=new List<int>();foreach(var a in front)foreach(var b in Touching(a))if(hops.TryAdd(b,h)){parent[b]=a;next.Add(b);}front=next;}
                double Length(PreparedSegment x)=>NetworkDesignChecks.HorizontalArcLength(new(x.CourseStart,x.CourseControl1,x.CourseControl2,x.CourseEnd));
                var path=new List<int>();if(hops.ContainsKey(best.Section))for(var at=parent.GetValueOrDefault(best.Section,-1);at>=0&&at!=f.Section;at=parent.GetValueOrDefault(at,-1))path.Add(at);
                string Degree(long id)=>incident[id].Count.ToString();
                var via=string.Join(" ",path.Select(k=>{var x=sections[k];return $"[{Class(x)}{(x.OneWay?"":"2")} w{x.NativeWidth:F0} {Length(x):F0}m d{Degree(x.StartNodeId)}/{Degree(x.EndNodeId)}]";}));
                Console.WriteLine($"    overlap3{(s.Roundabout||o.Roundabout||path.Any(k=>sections[k].Roundabout)?" ring":"")} {relation} hops={(hops.TryGetValue(best.Section,out var hop)?hop:99)} depth={f.Value:F1} {Class(s)}{(s.OneWay?"":"2")} way {s.WayId} L{s.Lanes} w{s.NativeWidth:F0} len {Length(s):F0} {s.StartNodeId}->{s.EndNodeId} / {Class(o)}{(o.OneWay?"":"2")} way {o.WayId} L{o.Lanes} w{o.NativeWidth:F0} len {Length(o):F0} {o.StartNodeId}->{o.EndNodeId} at ({f.Position.x:F0},{f.Position.z:F0}) via {via}");
            }
        }
        // VISUAL_DEBUG_CLEARANCE=1 lists each low crossing with the bridge and the
        // lower road or rail it passes over, and both heights.
        if(Environment.GetEnvironmentVariable("VISUAL_DEBUG_CLEARANCE")=="1")
        {
            var all=new Dictionary<(int,int),List<(int Section,double3 P)>>();
            for(var i=0;i<sections.Count;i++)
            {
                var s=sections[i];var curve=new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);var n=Math.Max(2,(int)(NetworkDesignChecks.HorizontalArcLength(curve)/2)+1);
                for(var k=0;k<n;k++){var p=NetworkDesignChecks.Evaluate(curve,k/(n-1d));if(!all.TryGetValue(Cell(p),out var l))all[Cell(p)]=l=new();l.Add((i,p));}
            }
            double Arc(PreparedSegment x)=>NetworkDesignChecks.HorizontalArcLength(new(x.CourseStart,x.CourseControl1,x.CourseControl2,x.CourseEnd));
            foreach(var f in measured.Failures.Where(f=>f.Category=="crossing-clearance").OrderBy(f=>f.Value))
            {
                var s=sections[f.Section];var (cx,cz)=Cell(f.Position);var best=(Section:-1,Distance:double.PositiveInfinity,Y:0d);
                for(var x=cx-1;x<=cx+1;x++)for(var z=cz-1;z<=cz+1;z++)if(all.TryGetValue((x,z),out var list))foreach(var q in list)
                {
                    var o=sections[q.Section];if(q.Section==f.Section||!(o.Layer<s.Layer||o.Tunnel))continue;
                    var d=math.distance(q.P.xz,f.Position.xz);if(d<best.Distance)best=(q.Section,d,q.P.y);
                }
                if(best.Section<0)continue;var b=sections[best.Section];
                string Describe(PreparedSegment x)=>$"{x.Kind} {x.Classification} L{x.Layer}{(x.Bridge?" B":"")}{(x.Tunnel?" T":"")} way {x.WayId} len {Arc(x):F0} ends {x.CourseStart.y:F1}/{x.CourseEnd.y:F1}";
                Console.WriteLine($"    clearance gap={f.Value:F1} deck {f.Position.y:F1} over {best.Y:F1} at ({f.Position.x:F0},{f.Position.z:F0}): {Describe(s)} / {Describe(b)}");
            }
        }
        if(Environment.GetEnvironmentVariable("VISUAL_DEBUG_CROSSINGS")=="1")Crossings(sections,"final",true);
        foreach(var p in counts.OrderByDescending(p=>p.Value).Take(24))Console.WriteLine($"  cause {p.Key} = {p.Value}");
        JunctionDistances(sections,measured);
    }

    // VISUAL_DEBUG_CROSSINGS=1: road centrelines that cross without a junction
    // (no shared node within two steps) less than 4 m apart in height, on any
    // layers: roads that visibly pass through each other.
    private static void Crossings(IReadOnlyList<PreparedSegment> sections,string label,bool print)
    {
        (int,int) Cell(double3 p)=>((int)Math.Floor(p.x/32),(int)Math.Floor(p.z/32));
        string Class(PreparedSegment s)=>s.ProtectedLink?"link":s.Classification;
        var incident=new Dictionary<long,List<int>>();
        for(var i=0;i<sections.Count;i++)if(sections[i].Kind==NetworkKind.Road)foreach(var id in new[]{sections[i].StartNodeId,sections[i].EndNodeId})
        {if(!incident.TryGetValue(id,out var list))incident[id]=list=new();list.Add(i);}
        IEnumerable<int> Touching(int i)=>new[]{sections[i].StartNodeId,sections[i].EndNodeId}.SelectMany(id=>incident[id]);
        {
            var pieces=new Dictionary<(int,int),List<(int Section,double3 A,double3 B)>>();
            for(var i=0;i<sections.Count;i++)
            {
                var s=sections[i];if(s.Kind!=NetworkKind.Road)continue;var curve=new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);
                var n=Math.Max(2,(int)(NetworkDesignChecks.HorizontalArcLength(curve)/2)+1);var prev=NetworkDesignChecks.Evaluate(curve,0);
                for(var k=1;k<n;k++){var p=NetworkDesignChecks.Evaluate(curve,k/(n-1d));foreach(var c in new[]{Cell(prev),Cell(p)}.Distinct()){if(!pieces.TryGetValue(c,out var l))pieces[c]=l=new();l.Add((i,prev,p));}prev=p;}
            }
            var seen=new HashSet<(int,int)>();var byKind=new Dictionary<string,int>();
            foreach(var cell in pieces.Values)for(var x=0;x<cell.Count;x++)for(var y=x+1;y<cell.Count;y++)
            {
                var (i,a,b)=cell[x];var (j,c,d)=cell[y];if(i==j)continue;var key=(Math.Min(i,j),Math.Max(i,j));if(seen.Contains(key))continue;
                var r=b.xz-a.xz;var q=d.xz-c.xz;var den=r.x*q.y-r.y*q.x;if(Math.Abs(den)<1e-9)continue;
                var t=((c.x-a.x)*q.y-(c.z-a.z)*q.x)/den;var u=((c.x-a.x)*r.y-(c.z-a.z)*r.x)/den;if(t<0||t>1||u<0||u>1)continue;
                var hi=math.lerp(a.y,b.y,t);var lo=math.lerp(c.y,d.y,u);if(Math.Abs(hi-lo)>=4)continue;
                var near=new HashSet<int>(Touching(i).SelectMany(Touching));if(near.Contains(j))continue;
                seen.Add(key);var s=sections[i];var o=sections[j];var p=math.lerp(a,b,t);
                string Tag(PreparedSegment z)=>$"{Class(z)}{(z.Bridge?"B":z.Tunnel?"T":"")}L{z.Layer}";
                var kind=string.Join("+",new[]{Tag(s),Tag(o)}.OrderBy(v=>v));byKind[kind]=byKind.GetValueOrDefault(kind)+1;
                if(print)Console.WriteLine($"    crossing dy={hi-lo:F1} at ({p.x:F0},{p.z:F0}) {Tag(s)} way {s.WayId} {s.StartNodeId}->{s.EndNodeId} / {Tag(o)} way {o.WayId} {o.StartNodeId}->{o.EndNodeId}");
            }
            Console.WriteLine($"  {label} crossings without junction: {seen.Count}; "+string.Join(", ",byKind.OrderByDescending(p=>p.Value).Take(12).Select(p=>$"{p.Key}={p.Value}")));
        }
    }

    private static void JunctionDistances(IReadOnlyList<PreparedSegment> sections,VisualCompilerMetrics.Measurement measured)
    {
        // Distance from each overlap to the nearest road junction (degree three or more).
        var junctions=sections.Where(s=>s.Kind==NetworkKind.Road).SelectMany(s=>new[]{(s.StartNodeId,(double3)s.CourseStart),(s.EndNodeId,(double3)s.CourseEnd)})
            .GroupBy(e=>e.Item1).Where(g=>g.Count()>=3).Select(g=>g.First().Item2.xz).ToArray();
        var junctionCells=junctions.GroupBy(p=>((int)Math.Floor(p.x/100),(int)Math.Floor(p.y/100))).ToDictionary(g=>g.Key,g=>g.ToArray());
        var bands=new Dictionary<string,int>();
        foreach(var f in measured.Failures.Where(f=>f.Category=="same-level-overlap"))
        {
            var cx=(int)Math.Floor(f.Position.x/100);var cz=(int)Math.Floor(f.Position.z/100);var nearest=double.MaxValue;
            for(var dz=-3;dz<=3;dz++)for(var dx=-3;dx<=3;dx++)if(junctionCells.TryGetValue((cx+dx,cz+dz),out var list))foreach(var p in list)nearest=Math.Min(nearest,math.distance(p,f.Position.xz));
            var band=nearest<30?"<30 m":nearest<100?"30-100 m":nearest<300?"100-300 m":">300 m";bands[band]=bands.TryGetValue(band,out var n)?n+1:1;
        }
        Console.WriteLine("  overlap distance to nearest junction: "+string.Join("; ",bands.OrderBy(b=>b.Key).Select(b=>$"{b.Key}={b.Value}")));
    }

    // Why a short source road section has kept nodes at both ends.
    private static void ShortSources(IReadOnlyList<PreparedSegment> source)
    {
        var incident=source.SelectMany((s,i)=>new[]{(Key:new SourceNodeKey(s.Kind,s.StartNodeId),i),(Key:new SourceNodeKey(s.Kind,s.EndNodeId),i)})
            .GroupBy(x=>x.Key).ToDictionary(g=>g.Key,g=>g.Select(x=>x.i).ToArray());
        string Reason(SourceNodeKey key)
        {
            var arms=incident[key];if(arms.Length==1)return "end";if(arms.Length>2)return "junction"+Math.Min(arms.Length,4);
            var a=source[arms[0]];var b=source[arms[1]];
            if(a.Bridge!=b.Bridge||a.Tunnel!=b.Tunnel)return "structure";if(a.Layer!=b.Layer)return "layer";
            if(a.OneWay!=b.OneWay)return "oneway";if(a.Lanes!=b.Lanes)return "lanes";if(a.Classification!=b.Classification)return "class";
            if(a.NativeWidth!=b.NativeWidth||a.NativeAssetKey!=b.NativeAssetKey)return "asset";
            if(a.Reference!=b.Reference)return "ref";if(a.ForwardLanes!=b.ForwardLanes||a.BackwardLanes!=b.BackwardLanes)return "dir-lanes";
            float2 Away(PreparedSegment s)=>math.normalizesafe((s.StartNodeId==key.Id?s.CourseControl1-s.CourseStart:s.CourseControl2-s.CourseEnd).xz);
            if(math.dot(Away(a),Away(b))>-.707f)return "bend";if(a.WayId!=b.WayId)return "way";
            return "plain";
        }
        var counts=new Dictionary<string,int>();
        foreach(var s in source.Where(s=>s.Kind==NetworkKind.Road))
        {
            var length=NetworkDesignChecks.HorizontalArcLength(new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd));
            if(length>=VisualCompilerTable.For(s).MinimumSegment)continue;
            var pair=string.Join("+",new[]{Reason(new(s.Kind,s.StartNodeId)),Reason(new(s.Kind,s.EndNodeId))}.OrderBy(r=>r));
            var key=$"{(s.ProtectedLink?"link":s.Classification)}:{pair}";counts[key]=counts.TryGetValue(key,out var n)?n+1:1;
            // Between two junctions: is it the gap between a divided cross street's carriageways?
            if(pair.StartsWith("junction")&&pair.Contains("+junction"))
            {
                IEnumerable<(string,string)> OneWayNames(long id)=>incident[new(s.Kind,id)].Select(k=>source[k]).Where(o=>o!=s&&o.OneWay&&o.Name.Length>0).Select(o=>(o.Name,o.Classification));
                var divided=OneWayNames(s.StartNodeId).Intersect(OneWayNames(s.EndNodeId)).Select(p=>p.Item2).FirstOrDefault();
                var detail=$"{(s.ProtectedLink?"link":s.Classification)}:junctions {(divided!=null?"across divided "+divided:"other")}";counts[detail]=counts.TryGetValue(detail,out var m)?m+1:1;
            }
        }
        foreach(var p in counts.OrderByDescending(p=>p.Value).Take(18))Console.WriteLine($"  short source {p.Key} = {p.Value}");
    }

    private static void Measure(Capture c,string output,string pipeline,Func<Capture,ImportLog,IReadOnlyList<PreparedSegment>> design)
    {
        var log=new ImportLog{ScalarDiagnostics=Environment.GetEnvironmentVariable("VISUAL_DIAGNOSTICS")=="1"};var watch=System.Diagnostics.Stopwatch.StartNew();
        var designed=design(c,log);
        if(log.DiagnosticsEnabled)foreach(var line in log.Lines.Where(l=>l.Contains("RAIL OVERLAPPING ROUTE",StringComparison.Ordinal)||l.Contains("RAIL PARALLEL ROUTES",StringComparison.Ordinal)||l.Contains("VISUAL DEBUG",StringComparison.Ordinal)||l.Contains("LANE SLOTS",StringComparison.Ordinal)||l.Contains("VISUAL COMPILER crossing",StringComparison.Ordinal)||l.Contains("VISUAL COMPILER vertical",StringComparison.Ordinal)||l.Contains("VISUAL COMPILER profile",StringComparison.Ordinal)||l.Contains("VISUAL COMPILER horizontal",StringComparison.Ordinal)||l.Contains("junction height mismatch",StringComparison.Ordinal)))Console.WriteLine("  diag "+line);var designSeconds=watch.Elapsed.TotalSeconds;
        var stage=System.Diagnostics.Stopwatch.StartNew();
        var sections=NativeDesignUnits.Prepare(StarterNetworkPolicy.WithoutSideStructures(designed),c.Contracts,log,default,pipeline.Contains("visual-compiler",StringComparison.Ordinal)?s=>VisualCompilerTable.For(s).MinimumSegment:null);
        var unitSeconds=stage.Elapsed.TotalSeconds;stage.Restart();
        // The same pre-submission rules NativeNetworkPlacement.CreateDefinitions
        // applies: a section failing one is omitted in game and leaves a gap.
        var sameNode=sections.Count(s=>s.StartNodeId==s.EndNodeId);
        var nonfinite=sections.Count(s=>!new[]{s.Start,s.End,s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd}.All(p=>math.all(math.isfinite(p))));
        var noExtent=sections.Count(s=>!NetworkDesignChecks.HasHorizontalExtent(new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd)));
        // Lane alignment at one-way merges and splits: planned lateral error
        // between the lanes each movement connects, from the final geometry.
        var lanePlan=NativeLaneObligations.Build(sections);
        var mergeNodes=sections.SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}).GroupBy(id=>id).Where(g=>g.Count()>=3).Select(g=>g.Key).ToHashSet();
        var laneErrors=lanePlan.Movements.Where(m=>mergeNodes.Contains(m.Node.Id)&&!double.IsNaN(m.PlannedLateralError)).Select(m=>m.PlannedLateralError).OrderBy(e=>e).ToArray();
        double LaneQ(double q)=>laneErrors.Length==0?0:laneErrors[(int)(q*(laneErrors.Length-1))];
        Console.WriteLine($"  lane alignment at merges/splits: {laneErrors.Length} movements; P50 {LaneQ(.5):F2} m, P90 {LaneQ(.9):F2} m, P99 {LaneQ(.99):F2} m; over 0.5 m {laneErrors.Count(e=>e>.5)}, over 1.5 m {laneErrors.Count(e=>e>1.5)}");
        // Left-edge continuity where a one-way road changes width at a plain node:
        // the sideways step between the two arms' left edges at the node.
        {
            var oneWay=sections.Where(s=>s.Kind==NetworkKind.Road&&s.OneWay&&!s.Roundabout).SelectMany(s=>new[]{(Id:s.StartNodeId,S:s,Out:true),(Id:s.EndNodeId,S:s,Out:false)}).GroupBy(e=>e.Id);
            var steps=new List<double>();
            foreach(var g in oneWay)
            {
                var e=g.ToArray();if(e.Length!=2||e[0].Out==e[1].Out)continue;var down=e.First(x=>x.Out).S;var up=e.First(x=>!x.Out).S;
                if(Math.Abs(down.NativeWidth-up.NativeWidth)<=.5f)continue;
                var t=math.normalizesafe((down.CourseControl1-down.CourseStart).xz);var right=new float2(t.y,-t.x);
                if(math.dot(t,math.normalizesafe((up.CourseEnd-up.CourseControl2).xz))<.82f)continue;
                steps.Add(Math.Abs(math.dot(down.CourseStart.xz-right*down.NativeWidth/2-(up.CourseEnd.xz-right*up.NativeWidth/2),right)));
            }
            steps.Sort();
            Console.WriteLine($"  left-edge step at width changes: {steps.Count} nodes; P50 {(steps.Count==0?0:steps[steps.Count/2]):F2} m; over 0.5 m {steps.Count(v=>v>.5)}");
        }
        var nodeShape=sections.Where(s=>s.Kind==NetworkKind.Road).SelectMany(s=>new[]{(Id:s.StartNodeId,Out:1,s.OneWay),(Id:s.EndNodeId,Out:0,s.OneWay)}).GroupBy(x=>x.Id)
            .ToDictionary(g=>g.Key,g=>$"{g.Count(x=>x.Out==0&&x.OneWay)}in/{g.Count(x=>x.Out==1&&x.OneWay)}out/{g.Count(x=>!x.OneWay)}twoway");
        foreach(var g in lanePlan.Movements.Where(m=>mergeNodes.Contains(m.Node.Id)&&m.PlannedLateralError>.5).GroupBy(m=>m.Node.Id).GroupBy(g=>nodeShape.TryGetValue(g.Key,out var shape)?shape:"?").OrderByDescending(g=>g.Count()).Take(6))
            Console.WriteLine($"  misaligned nodes shaped {g.Key}: {g.Count()}");
        if(Environment.GetEnvironmentVariable("VISUAL_DEBUG_LANES")=="1")
            foreach(var g in lanePlan.Movements.Where(m=>mergeNodes.Contains(m.Node.Id)&&m.PlannedLateralError>.5).GroupBy(m=>m.Node.Id).Take(8))
            {
                Console.WriteLine($"  node {g.Key}: worst {g.Max(m=>m.PlannedLateralError):F2} m");
                foreach(var s in sections.Where(s=>s.StartNodeId==g.Key||s.EndNodeId==g.Key))
                {
                    var start=s.StartNodeId==g.Key;var p=start?s.CourseStart:s.CourseEnd;var d=start?s.CourseControl1-s.CourseStart:s.CourseEnd-s.CourseControl2;
                    Console.WriteLine($"    way {s.WayId} {(start?"out":"in")} {s.Classification} lanes={s.Lanes} native=[{string.Join(",",s.NativeLaneOffsets.Select(o=>o.ToString("F1")))}] tunnel={s.Tunnel} bridge={s.Bridge} at=({p.x:F1},{p.z:F1}) offset=({(start?s.StartCurveOffset:s.EndCurveOffset).x:F2},{(start?s.StartCurveOffset:s.EndCurveOffset).z:F2}) heading=({math.normalizesafe(d.xz).x:F2},{math.normalizesafe(d.xz).y:F2}) len={NetworkDesignChecks.HorizontalArcLength(new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd)):F0}");
                }
            }
        // Physical breaks: a dead-end node within 3 m of another network end with
        // a different node id; the game builds them as two unconnected nodes.
        foreach(var kind in new[]{NetworkKind.Rail,NetworkKind.Road})
        {
            var ends=sections.Where(s=>s.Kind==kind).SelectMany(s=>new[]{(Id:s.StartNodeId,P:s.Start.xz,Y:s.Start.y),(Id:s.EndNodeId,P:s.End.xz,Y:s.End.y)}).ToArray();
            var degreeOf=ends.GroupBy(e=>e.Id).ToDictionary(g=>g.Key,g=>g.Count());
            var cells=ends.GroupBy(e=>((int)Math.Floor(e.P.x/4),(int)Math.Floor(e.P.y/4))).ToDictionary(g=>g.Key,g=>g.ToArray());
            var joined=sections.Where(s=>s.Kind==kind).Select(s=>(Math.Min(s.StartNodeId,s.EndNodeId),Math.Max(s.StartNodeId,s.EndNodeId))).ToHashSet();
            var dangling=ends.Where(e=>degreeOf[e.Id]==1).Where(e=>
            {
                var cx=(int)Math.Floor(e.P.x/4);var cz=(int)Math.Floor(e.P.y/4);
                for(var dz=-1;dz<=1;dz++)for(var dx=-1;dx<=1;dx++)if(cells.TryGetValue((cx+dx,cz+dz),out var near))
                    foreach(var o in near)if(o.Id!=e.Id&&!joined.Contains((Math.Min(o.Id,e.Id),Math.Max(o.Id,e.Id)))&&math.distance(o.P,e.P)<3&&Math.Abs(o.Y-e.Y)<3)return true;
                return false;
            }).ToArray();
            Console.WriteLine($"  physical {kind} breaks (dead end within 3 m of another end): {dangling.Length}"+(dangling.Length==0?"":$"; e.g. {string.Join(" ",dangling.Take(4).Select(b=>$"{b.Id}@({b.P.x:F0},{b.P.y:F0})"))}"));
            // Distinct nodes within 3 m of each other (not joined by a section): the
            // game joins nodes at one position, and wired such pairs into dead ends.
            var crowded=ends.GroupBy(e=>e.Id).Select(g=>g.First()).ToArray();var crowdCells=crowded.GroupBy(e=>((int)Math.Floor(e.P.x/3),(int)Math.Floor(e.P.y/3))).ToDictionary(g=>g.Key,g=>g.ToArray());
            var close=crowded.Count(e=>{var cx=(int)Math.Floor(e.P.x/3);var cz=(int)Math.Floor(e.P.y/3);for(var dz=-1;dz<=1;dz++)for(var dx=-1;dx<=1;dx++)if(crowdCells.TryGetValue((cx+dx,cz+dz),out var near)&&near.Any(o=>o.Id!=e.Id&&!joined.Contains((Math.Min(o.Id,e.Id),Math.Max(o.Id,e.Id)))&&math.distance(o.P,e.P)<3))return true;return false;});
            Console.WriteLine($"  {kind} nodes within 3 m of another unjoined node: {close}");
            // Every output dead end with no source dead end within 30 m: a track or
            // road the design left hanging (the game marks it with a dead-end icon).
            var sourceEnds=c.Source.Where(s=>s.Kind==kind).SelectMany(s=>new[]{(Id:s.StartNodeId,P:((float3)s.CourseStart).xz),(Id:s.EndNodeId,P:((float3)s.CourseEnd).xz)}).ToArray();
            var sourceDegree=sourceEnds.GroupBy(e=>e.Id).ToDictionary(g=>g.Key,g=>g.Count());
            var sourceDead=sourceEnds.Where(e=>sourceDegree[e.Id]==1).Select(e=>e.P).ToArray();
            var sourceCells=sourceDead.GroupBy(p=>((int)Math.Floor(p.x/32),(int)Math.Floor(p.y/32))).ToDictionary(g=>g.Key,g=>g.ToArray());
            bool NearSourceEnd(float2 p){var cx=(int)Math.Floor(p.x/32);var cz=(int)Math.Floor(p.y/32);for(var dz=-1;dz<=1;dz++)for(var dx=-1;dx<=1;dx++)if(sourceCells.TryGetValue((cx+dx,cz+dz),out var near)&&near.Any(q=>math.distance(q,p)<30))return true;return false;}
            var hanging=ends.Where(e=>degreeOf[e.Id]==1&&!NearSourceEnd(e.P)).ToArray();
            double lat0=(double)c.Input["request"]!["Latitude"]!,lon0=(double)c.Input["request"]!["Longitude"]!;
            Console.WriteLine($"  new {kind} dead ends (no source dead end within 30 m): {hanging.Length}"+(hanging.Length==0?"":"; e.g. "+string.Join(" ",hanging.Take(int.TryParse(Environment.GetEnvironmentVariable("VISUAL_HANGING"),out var shown)?shown:6)
                .Select(b=>$"{b.Id}@{lat0+math.degrees(b.P.y/Earth):F5},{lon0+math.degrees(b.P.x/(Earth*Math.Cos(math.radians(lat0)))):F5}"))));
            // VISUAL_DEBUG_HANGING=1: what met at each new dead end in the source,
            // and which of those source ways the output still carries anywhere.
            if(Environment.GetEnvironmentVariable("VISUAL_DEBUG_HANGING")=="1")foreach(var b in hanging)
            {
                var here=sections.Where(s=>s.Kind==kind&&(s.StartNodeId==b.Id||s.EndNodeId==b.Id)).ToArray();
                var src=c.Source.Where(s=>s.Kind==kind&&math.distance(((float3)s.CourseStart).xz,b.P)<40||s.Kind==kind&&math.distance(((float3)s.CourseEnd).xz,b.P)<40).ToArray();
                var outWays=sections.Where(s=>s.Kind==kind).SelectMany(s=>s.SourceWayIds.Append(s.WayId)).ToHashSet();
                Console.WriteLine($"    hanging {b.Id} at ({b.P.x:F0},{b.P.y:F0}) sourceDegree={(sourceDegree.TryGetValue(b.Id,out var sd)?sd:0)} output={string.Join(",",here.Select(s=>$"{s.WayId}[{s.Classification} len={NetworkDesignChecks.HorizontalArcLength(new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd)):F0} other={(s.StartNodeId==b.Id?s.EndNodeId:s.StartNodeId)} otherDegree={(degreeOf.TryGetValue(s.StartNodeId==b.Id?s.EndNodeId:s.StartNodeId,out var od)?od:0)} service={s.RailService}]"))}");
                foreach(var n in ends.Where(e=>e.Id!=b.Id&&math.distance(e.P,b.P)<60).GroupBy(e=>e.Id).Select(g=>g.First()).OrderBy(e=>math.distance(e.P,b.P)).Take(4))Console.WriteLine($"      near node {n.Id} d={math.distance(n.P,b.P):F1} degree={degreeOf[n.Id]} dy={n.Y-b.Y:F1}");
                foreach(var s in src)Console.WriteLine($"      source way {s.WayId} {s.StartNodeId}->{s.EndNodeId} {s.Classification} kept={outWays.Contains(s.WayId)} start=({s.CourseStart.x:F0},{s.CourseStart.z:F0}) end=({s.CourseEnd.x:F0},{s.CourseEnd.z:F0})");
            }
        }
        Console.WriteLine($"  native omissions: {sameNode+nonfinite+noExtent} (same start/end node {sameNode}, nonfinite {nonfinite}, no horizontal extent {noExtent})");
        Connectivity("source",c.Source,c.Source);Connectivity("designed",c.Source,designed);Connectivity("emitted",c.Source,sections);
        if(Environment.GetEnvironmentVariable("VISUAL_DEBUG_OBLIGATION") is {} obligation)
        {
            var parts=obligation.Split(',').Select(long.Parse).ToArray();var way=parts[^1];
            foreach(var (label,list) in new[]{("designed",designed),("emitted",(IReadOnlyList<PreparedSegment>)sections)})
            {
                foreach(var s in list.Where(s=>s.Kind==NetworkKind.Rail&&(s.WayId==way||s.SourceWayIds.Contains(way))))
                    Console.WriteLine($"  obligation {label} way {s.WayId} {s.StartNodeId}->{s.EndNodeId} at ({s.Start.x:F0},{s.Start.z:F0})");
                foreach(var id in parts[..^1])
                    Console.WriteLine($"  obligation {label} node {id}: present={list.Any(s=>s.Kind==NetworkKind.Rail&&(s.StartNodeId==id||s.EndNodeId==id))} aliases=[{string.Join(",",list.Where(s=>s.Kind==NetworkKind.Rail).SelectMany(s=>s.SourceNodeAliases).Where(a=>a.SourceNodeId==id).Select(a=>a.CanonicalNodeId).Distinct())}]");
            }
        }
        var continuity=new ImportLog();NetworkContinuity.Validate(c.Source,sections,continuity,default,allowQualityWarnings:true);
        if(Environment.GetEnvironmentVariable("VISUAL_DEBUG_RAIL_OVERLAP") is "1" or "2")RailOverlaps(sections);
        Console.WriteLine($"  continuity (as logged in game): unresolved sections={continuity.Metric("Network continuity unresolved sections")}; junction conflicts={continuity.Metric("Network junction position conflicts")}; rail overlaps={continuity.Metric("Rail overlap observations")}");
        var obligations=new ImportLog{CaptureRecords=true};SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(c.Source),sections,obligations,default);
        var topologySeconds=stage.Elapsed.TotalSeconds;stage.Restart();
        var roadbed=NetworkRoadbed.Prepare(c.Terrain,c.Scale,c.Width,sections,c.Areas,c.Lines,log,default);
        var unitSizes=sections.GroupBy(s=>s.NativeUnitId).Select(g=>g.Count()).OrderBy(n=>n).ToArray();
        Console.WriteLine($"  stages: design={designSeconds:F1}s; units={unitSeconds:F1}s; topology={topologySeconds:F1}s; roadbed={stage.Elapsed.TotalSeconds:F1}s; "+
            $"nativeUnits={unitSizes.Length}; unitP99={unitSizes[(int)(.99*(unitSizes.Length-1))]}; largestUnit={unitSizes[^1]}");
        if(Environment.GetEnvironmentVariable("VISUAL_DEBUG_TUNNELS")=="1"){TunnelCover(c,sections,roadbed);FullWidthCover(c,sections,roadbed);}
        if(Environment.GetEnvironmentVariable("VISUAL_DEBUG_DIPS")=="1")
        {
            BridgeGapDips(c,sections);
            // Bridge decks sunk in the stamped ground more than 40 m from their ends.
            var buried=0;var total=0;var byRef=new Dictionary<string,int>();
            foreach(var s in sections.Where(s=>s.Bridge&&!s.Tunnel&&s.Kind==NetworkKind.Road))
            {
                var curve=new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);var length=NetworkDesignChecks.HorizontalArcLength(curve);var n=Math.Max(2,(int)(length/5));
                for(var k=0;k<=n;k++)
                {
                    if(Math.Min(k,n-k)*length/n<40)continue;var p=NetworkDesignChecks.Evaluate(curve,k/(double)n);total++;
                    var stamped=NetworkProcessor.Sample((float)p.x,(float)p.z,roadbed,c.Scale,c.Width);
                    if(stamped>p.y-1)
                    {
                        buried++;var key=s.Reference.Length>0?s.Reference:s.Name;byRef[key]=byRef.TryGetValue(key,out var v)?v+1:1;
                        if(v==0){double lat0=(double)c.Input["request"]!["Latitude"]!,lon0=(double)c.Input["request"]!["Longitude"]!;
                            Console.WriteLine($"    buried bridge {key} way {s.WayId}: deck {p.y:F1}, original {NetworkProcessor.Sample((float)p.x,(float)p.z,c.Terrain,c.Scale,c.Width):F1}, stamped {stamped:F1} at {lat0+math.degrees(p.z/Earth):F5},{lon0+math.degrees(p.x/(Earth*Math.Cos(math.radians(lat0)))):F5}");}
                    }
                }
            }
            Console.WriteLine($"  bridge deck samples (>40 m from ends) within 1 m of or under the ground: {buried} of {total}; "+string.Join(", ",byRef.OrderByDescending(p=>p.Value).Take(8).Select(p=>$"{p.Key} {p.Value*5} m")));
        }
        if(Environment.GetEnvironmentVariable("VISUAL_DEBUG_BRIDGES_NEAR") is {} bridgesNear)
        {
            // Each bridge chain within a radius: deck height every ~40 m and its
            // largest rise above (+) or dip below (−) the straight line between ends.
            double lat0=(double)c.Input["request"]!["Latitude"]!,lon0=(double)c.Input["request"]!["Longitude"]!;
            var q=bridgesNear.Split(',').Select(v=>double.Parse(v,CultureInfo.InvariantCulture)).ToArray();
            var centre=ImportEarthLocation.Geo.GeoMath.ToLocalMeters(q[0],q[1],lat0,lon0);
            var bridges=sections.Where(s=>s.Bridge&&math.distance(s.Start.xz,centre.xz)<q[2]).ToList();
            var byNode=bridges.SelectMany((s,i)=>new[]{(s.StartNodeId,i),(s.EndNodeId,i)}).GroupBy(x=>x.Item1).ToDictionary(g=>g.Key,g=>g.Select(x=>x.i).ToList());
            var done=new HashSet<int>();
            for(var i=0;i<bridges.Count;i++)
            {
                if(done.Contains(i))continue;
                // Walk the chain both ways through nodes shared by exactly two bridge sections.
                var chain=new LinkedList<(int Index,bool Forward)>();chain.AddFirst((i,true));done.Add(i);
                foreach(var forward in new[]{true,false})
                {
                    var node=forward?bridges[i].EndNodeId:bridges[i].StartNodeId;
                    while(byNode[node].Count==2)
                    {
                        var next=byNode[node].First(j=>!done.Contains(j)&&j!=i);if(done.Contains(next))break;done.Add(next);
                        var along=bridges[next].StartNodeId==node;
                        if(forward)chain.AddLast((next,along));else chain.AddFirst((next,!along));
                        node=along?bridges[next].EndNodeId:bridges[next].StartNodeId;
                        if(!byNode[node].Any(j=>!done.Contains(j)))break;
                    }
                }
                var points=new List<double3>();
                foreach(var (index,forward) in chain)
                {
                    var s=bridges[index];var curve=new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);
                    for(var k=0;k<=8;k++)points.Add(NetworkDesignChecks.Evaluate(curve,forward?k/8d:1-k/8d));
                }
                // The deck over water only: elevated land approaches climbing to it
                // are not part of the span's shape.
                var wet=points.Select((p,k)=>(p,k)).Where(x=>c.Water(new float3((float)x.p.x,0,(float)x.p.z)).HasValue).Select(x=>x.k).ToArray();
                if(wet.Length>1)points=points.GetRange(wet[0],wet[^1]-wet[0]+1);
                var length=0d;for(var k=1;k<points.Count;k++)length+=math.distance(points[k-1].xz,points[k].xz);if(length<150)continue;
                var walked=0d;var up=0d;var down=0d;
                for(var k=1;k<points.Count;k++){walked+=math.distance(points[k-1].xz,points[k].xz);var chord=points[0].y+(points[^1].y-points[0].y)*walked/length;up=Math.Max(up,points[k].y-chord);down=Math.Min(down,points[k].y-chord);}
                var first=bridges[chain.First!.Value.Index];
                Console.WriteLine($"  bridge {first.Reference} {first.Name} way {first.WayId} oneway={first.OneWay}: {length:F0} m, ends {points[0].y:F1}/{points[^1].y:F1} m; rises {up:F1} m above and dips {-down:F1} m below the straight line");
                if(Environment.GetEnvironmentVariable("VISUAL_DEBUG_BRIDGE_HEIGHTS")=="1")
                    Console.WriteLine("    sections "+chain.Count+"; heights "+string.Join(" ",points.Where((_,k)=>k%9==0).Select(p=>p.y.ToString("F1",CultureInfo.InvariantCulture)))+"; water "+
                        string.Join(" ",points.Where((_,k)=>k%9==0).Select(p=>c.Water(new float3((float)p.x,0,(float)p.z)) is float w?w.ToString("F1",CultureInfo.InvariantCulture):"-")));
            }
        }
        if(Environment.GetEnvironmentVariable("VISUAL_DEBUG_WATER_CROSSINGS")=="1")
        {
            // Ground-level roads and rail crossing mapped water (a river reach or
            // polygon with a surface above the deck): each wet run is one crossing.
            double lat0=(double)c.Input["request"]!["Latitude"]!,lon0=(double)c.Input["request"]!["Longitude"]!;
            var crossings=new List<(PreparedSegment S,double Length,double3 P)>();
            foreach(var s in sections.Where(s=>!s.Bridge&&!s.Tunnel))
            {
                var curve=new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);var n=Math.Max(2,(int)(NetworkDesignChecks.HorizontalArcLength(curve)/3));
                var run=0;double3 at=default;
                for(var k=0;k<=n;k++)
                {
                    var p=NetworkDesignChecks.Evaluate(curve,k/(double)n);var surface=c.Water(new float3((float)p.x,0,(float)p.z));
                    if(surface is float w&&w>p.y-1.5){if(run==0)at=p;run++;}
                    else if(run>0){crossings.Add((s,run*3,at));run=0;}
                }
                if(run>0)crossings.Add((s,run*3,at));
            }
            Console.WriteLine($"  ground-level crossings of mapped water: {crossings.Count} (roads {crossings.Count(x=>x.S.Kind==NetworkKind.Road)}, rail {crossings.Count(x=>x.S.Kind==NetworkKind.Rail)}); longer than 6 m: {crossings.Count(x=>x.Length>6)}");
            foreach(var x in crossings.Where(x=>x.Length>6).OrderByDescending(x=>x.Length).Take(10))
                Console.WriteLine($"    {x.S.Kind} {x.S.Classification} {x.S.Reference} way {x.S.WayId}: {x.Length:F0} m wet at {lat0+math.degrees(x.P.z/Earth):F5},{lon0+math.degrees(x.P.x/(Earth*Math.Cos(math.radians(lat0)))):F5}");
        }
        watch.Restart();var measured=VisualCompilerMetrics.Measure(sections,roadbed,c.Scale,c.Width,default);var metricSeconds=watch.Elapsed.TotalSeconds;
        if(Environment.GetEnvironmentVariable("VISUAL_DEBUG_TERRAIN") is "1" or "2")TerrainCauses(c,sections,roadbed,measured);
        if(Environment.GetEnvironmentVariable("VISUAL_DEBUG_OVERLAP") is "1" or "2" or "3")OverlapCauses(sections,measured);
        // VISUAL_DEBUG_SHORT=2 classifies the finished sections instead of the source.
        if(Environment.GetEnvironmentVariable("VISUAL_DEBUG_SHORT")=="2")ShortSources(sections);
        var raw=VisualCompilerMetrics.Measure(designed,roadbed,c.Scale,c.Width,default);
        Console.WriteLine($"  before native units: sections={designed.Count}; paired failing={raw.Paired.FailedMeters/1000:F1} km of {raw.Paired.MatchedMeters/1000:F1}; grade-change={raw.Failures.Count(f=>f.Category=="grade-change")}; short-segment={raw.Failures.Count(f=>f.Category=="short-segment")}");
        Directory.CreateDirectory(output);
        using(var file=File.Create(Path.Combine(output,FailuresFile)))using(var zip=new GZipStream(file,CompressionLevel.Optimal))using(var writer=new StreamWriter(zip))
            foreach(var f in measured.Failures)writer.WriteLine(JsonConvert.SerializeObject(new{f.Category,f.Section,f.WayId,f.StartNodeId,f.EndNodeId,
                x=f.Position.x,y=f.Position.y,z=f.Position.z,f.Value,f.Limit,f.Severity}));
        using(var file=File.Create(Path.Combine(output,PlotFile)))using(var zip=new GZipStream(file,CompressionLevel.Optimal))using(var writer=new StreamWriter(zip))
            foreach(var s in sections)writer.WriteLine(JsonConvert.SerializeObject(new{s.WayId,s.StartNodeId,s.EndNodeId,s.Layer,s.SourceWayIds,kind=s.Kind.ToString(),s.Classification,s.Bridge,s.Tunnel,s.OneWay,s.NativeWidth,
                p=new[]{s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd}.SelectMany(p=>new[]{p.x,p.y,p.z}).ToArray()}));
        var topology=obligations.Lines.Where(l=>l.Contains("SOURCE OBLIGATIONS",StringComparison.Ordinal)).ToArray();
        var present=new HashSet<SourceNodeKey>(sections.SelectMany(s=>new[]{new SourceNodeKey(s.Kind,s.StartNodeId),new SourceNodeKey(s.Kind,s.EndNodeId)}));
        var aliased=new HashSet<SourceNodeKey>(sections.SelectMany(s=>s.SourceNodeAliases.Select(a=>new SourceNodeKey(s.Kind,a.SourceNodeId))));
        var missing=SourceTopologyObligations.Capture(c.Source).Ports.Where(p=>!present.Contains(p)&&!aliased.Contains(p)).Take(12).Select(p=>
        {
            var touching=c.Source.Where(s=>s.Kind==p.Kind&&(s.StartNodeId==p.Id||s.EndNodeId==p.Id)).ToArray();
            return $"{p.Kind}:{p.Id} degree={touching.Length} classes={string.Join(",",touching.Select(s=>s.Classification+(s.Bridge?"/bridge":"")+(s.Tunnel?"/tunnel":"")+(s.Roundabout?"/roundabout":"")+(s.OneWay?"/oneway":"")+"/L"+s.Lanes))}";
        }).ToArray();
        var designedNodes=new HashSet<SourceNodeKey>(designed.SelectMany(s=>new[]{new SourceNodeKey(s.Kind,s.StartNodeId),new SourceNodeKey(s.Kind,s.EndNodeId)}));
        foreach(var m in missing)Console.WriteLine("  missing port "+m+" inDesignOutput="+designedNodes.Contains(new(Enum.Parse<NetworkKind>(m.Split(':')[0]),long.Parse(m.Split(':')[1].Split(' ')[0]))));
        if(Environment.GetEnvironmentVariable("VISUAL_DEBUG_CROSSINGS")=="1")Crossings(c.Source,"source",false);
        if(long.TryParse(Environment.GetEnvironmentVariable("VISUAL_DEBUG_WAY"),out var debugWay))
            foreach(var (label,list) in new[]{("source",(IReadOnlyList<PreparedSegment>)c.Source),("designed",designed)})
                foreach(var s in list.Where(s=>s.WayId==debugWay||s.SourceWayIds.Contains(debugWay)))
                    Console.WriteLine($"  {label} way={s.WayId} ways=[{string.Join(",",s.SourceWayIds)}] {s.StartNodeId}->{s.EndNodeId} oneway={s.OneWay} class={s.Classification} start=({s.Start.x:F1},{s.Start.z:F1}) end=({s.End.x:F1},{s.End.z:F1}) aliases=[{string.Join(",",s.SourceNodeAliases.Select(a=>a.SourceNodeId+">"+a.CanonicalNodeId))}] "+
                        $"course=[{string.Join(" ",new[]{s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd}.Select(p=>$"({p.x:F0},{p.z:F0})"))}] geometry=[{string.Join(" ",s.SourceGeometry.Take(16).Select(p=>$"({p.x:F0},{p.z:F0})"))}] "+
                        $"original=[{string.Join(" ",s.OriginalGeometry.Take(24).Select(p=>$"({p.x:F0},{p.z:F0})"))}] bridge={s.Bridge} layer={s.Layer} tags={string.Join(",",s.RawSourceTags.Where(t=>t.Key is "bridge" or "layer" or "tunnel" or "railway" or "service" or "tracks").Select(t=>t.Key+"="+t.Value))}");
        foreach(var g in sections.SelectMany(s=>new[]{(s.Kind,Id:s.StartNodeId,P:s.Start,s.WayId),(s.Kind,Id:s.EndNodeId,P:s.End,s.WayId)}).GroupBy(e=>(e.Kind,e.Id))
            .Where(g=>g.Any(e=>math.distance(e.P,g.First().P)>.02f)).Take(5))
            Console.WriteLine($"  conflicting node {g.Key.Kind}:{g.Key.Id} "+string.Join(" ",g.Select(e=>$"way {e.WayId}@({e.P.x:F2},{e.P.y:F2},{e.P.z:F2})")));
        if(Environment.GetEnvironmentVariable("VISUAL_DEBUG_NEAR") is string near&&near.Split(',') is [var nx,var nz])
        {
            var centre=new float2(float.Parse(nx),float.Parse(nz));
            foreach(var (label,list) in new[]{("source",(IReadOnlyList<PreparedSegment>)c.Source),("designed",designed)})
                foreach(var s in list.Where(s=>Enumerable.Range(0,9).Any(k=>math.distance(NetworkDesignChecks.Evaluate(new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd),k/8d).xz,(double2)centre)<40)))
                    Console.WriteLine($"  near {label} {s.Kind} way={s.WayId} {s.StartNodeId}->{s.EndNodeId} class={s.Classification} service={s.RailService} bridge={s.Bridge} tunnel={s.Tunnel} layer={s.Layer} start=({s.CourseStart.x:F1},{s.CourseStart.y:F1},{s.CourseStart.z:F1}) end=({s.CourseEnd.x:F1},{s.CourseEnd.y:F1},{s.CourseEnd.z:F1})");
        }
        // VISUAL_DEBUG_REF=A 55: every source section of that route reference,
        // with geographic ends, to find a reported spot in the capture.
        if(Environment.GetEnvironmentVariable("VISUAL_DEBUG_REF") is {} debugRef)
        {
            double lat0=(double)c.Input["request"]!["Latitude"]!,lon0=(double)c.Input["request"]!["Longitude"]!;
            string Geo(float3 p)=>$"{lat0+math.degrees(p.z/Earth):F5},{lon0+math.degrees(p.x/(Earth*Math.Cos(math.radians(lat0)))):F5}";
            // tag:key=value matches a source tag instead.
            bool Match(PreparedSegment s)=>debugRef.StartsWith("tag:",StringComparison.Ordinal)&&debugRef.Substring(4).Split('=') is [var key,var value]
                ?s.SourceTags.TryGetValue(key,out var v)&&v==value:s.Reference.Split(';').Any(r=>r.Trim()==debugRef);
            foreach(var s in c.Source.Where(Match))
                Console.WriteLine($"  ref way={s.WayId} {s.StartNodeId}->{s.EndNodeId} class={s.Classification} link={s.ProtectedLink} oneway={s.OneWay} lanes={s.Lanes} width={s.NativeWidth} bridge={s.Bridge} tunnel={s.Tunnel} from={Geo(s.Start)} to={Geo(s.End)} local=({s.Start.x:F0},{s.Start.z:F0})->({s.End.x:F0},{s.End.z:F0}) y=({s.CourseStart.y:F1},{s.CourseEnd.y:F1})");
        }
        // VISUAL_DEBUG_SPLITS=1: source nodes where a one-way road splits in two
        // or two join, with lanes, link flags and the source angle between them.
        if(Environment.GetEnvironmentVariable("VISUAL_DEBUG_SPLITS")=="1")
        {
            double lat0=(double)c.Input["request"]!["Latitude"]!,lon0=(double)c.Input["request"]!["Longitude"]!;
            var ends=c.Source.Where(s=>s.Kind==NetworkKind.Road&&s.OneWay&&!s.Roundabout).SelectMany(s=>new[]{(Node:s.StartNodeId,S:s,Out:true),(Node:s.EndNodeId,S:s,Out:false)}).GroupBy(e=>e.Node);
            double2 Away(PreparedSegment s,bool atStart)
            {
                var g=s.SourceGeometry.Count>1?s.SourceGeometry:new[]{s.Start,s.End};var o=atStart?s.Start:s.End;
                foreach(var p in atStart?g:Enumerable.Reverse(g))if(math.distance(p.xz,o.xz)>=30)return math.normalizesafe((double2)(p.xz-o.xz));
                return math.normalizesafe((double2)((atStart?s.End:s.Start).xz-o.xz));
            }
            foreach(var g in ends)
            {
                var list=g.ToArray();if(list.Length!=3)continue;var outs=list.Where(e=>e.Out).ToArray();var ins=list.Where(e=>!e.Out).ToArray();
                var (one,two)=outs.Length==2?(ins,outs):ins.Length==2?(outs,ins):(Array.Empty<(long,PreparedSegment,bool)>(),Array.Empty<(long,PreparedSegment,bool)>());
                if(one.Length!=1||two.Length!=2||!two.Any(e=>e.Item2.Classification is "motorway" or "trunk"))continue;
                var angle=math.degrees(Math.Acos(math.clamp(math.dot(Away(two[0].Item2,two[0].Item3),Away(two[1].Item2,two[1].Item3)),-1,1)));
                var p=one[0].Item3?one[0].Item2.Start:one[0].Item2.End;
                Console.WriteLine($"  split {(outs.Length==2?"diverge":"merge")} node={g.Key} at {lat0+math.degrees(p.z/Earth):F5},{lon0+math.degrees(p.x/(Earth*Math.Cos(math.radians(lat0)))):F5} local=({p.x:F0},{p.z:F0}) "+
                    $"trunk={one[0].Item2.Classification}/{one[0].Item2.Lanes}L/{one[0].Item2.Reference} branches={string.Join(" + ",two.Select(e=>$"{e.Item2.Classification}{(e.Item2.ProtectedLink?"(link)":"")}/{e.Item2.Lanes}L/{e.Item2.Reference}"))} angle={angle:F1}");
            }
        }
        // VISUAL_DEBUG_PROFILE=x,z,r: emitted road heights against the original
        // ground at quarter points, for every section within r of a local point.
        if(Environment.GetEnvironmentVariable("VISUAL_DEBUG_PROFILE") is {} profileAt&&profileAt.Split(',').Select(v=>double.Parse(v,CultureInfo.InvariantCulture)).ToArray() is [var px,var pz,var pr])
            foreach(var s in sections.Where(s=>(s.Kind==NetworkKind.Road||Environment.GetEnvironmentVariable("VISUAL_DEBUG_PROFILE_RAIL")=="1")&&math.distance(((double3)s.CourseStart).xz,new double2(px,pz))<pr).OrderBy(s=>s.WayId).ThenBy(s=>s.CourseStart.x))
            {
                var curve=new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);
                Console.WriteLine($"  profile way={s.WayId} {s.StartNodeId}->{s.EndNodeId} {s.Classification} w={s.NativeWidth} oneway={s.OneWay} bridge={s.Bridge} tunnel={s.Tunnel} ({s.CourseStart.x:F1},{s.CourseStart.z:F1})->({s.CourseEnd.x:F1},{s.CourseEnd.z:F1}) offsets=({s.StartCurveOffset.x:F2},{s.StartCurveOffset.z:F2})/({s.EndCurveOffset.x:F2},{s.EndCurveOffset.z:F2}) "+
                    string.Join(" ",Enumerable.Range(0,5).Select(k=>{var p=NetworkDesignChecks.Evaluate(curve,k/4d);return $"{p.y:F1}/{NetworkProcessor.Sample((float)p.x,(float)p.z,c.Terrain,c.Scale,c.Width):F1}";})));
            }
        if(long.TryParse(Environment.GetEnvironmentVariable("VISUAL_DEBUG_NODE"),out var debugNode))
            foreach(var s in c.Source.Where(s=>s.StartNodeId==debugNode||s.EndNodeId==debugNode))
                Console.WriteLine($"  node-source way={s.WayId} {s.StartNodeId}->{s.EndNodeId} class={s.Classification} ref={s.Reference} lanes={s.Lanes} link={s.ProtectedLink} oneway={s.OneWay} layer={s.Layer} bridge={s.Bridge} tunnel={s.Tunnel} kind={s.Kind} y=({s.CourseStart.y:F2},{s.CourseEnd.y:F2}) roundabout={s.Roundabout} start=({s.Start.x:F1},{s.Start.z:F1}) end=({s.End.x:F1},{s.End.z:F1}) course=({s.CourseStart.x:F1},{s.CourseStart.z:F1})->({s.CourseEnd.x:F1},{s.CourseEnd.z:F1}) len={NetworkDesignChecks.HorizontalArcLength(new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd)):F1}");
        if(obligations.Records.TryGetValue("source-topology-obligations.ndjson",out var kindRecords))
            foreach(var g in kindRecords.Select(JObject.FromObject).Where(r=>!(bool)r["reachable"]!).GroupBy(r=>(int)r["From"]!["Kind"]!))
                Console.WriteLine($"  unreachable connections {(NetworkKind)g.Key}: {g.Count()}");
        if(obligations.Records.TryGetValue("source-topology-obligations.ndjson",out var connectionRecords))
            foreach(var r in connectionRecords.Select(JObject.FromObject).Where(r=>!(bool)r["reachable"]!).Take(int.TryParse(Environment.GetEnvironmentVariable("VISUAL_UNREACHABLE"),out var shownUnreachable)?shownUnreachable:8))
                Console.WriteLine("  unreachable "+r.ToString(Formatting.None).Substring(0,Math.Min(300,r.ToString(Formatting.None).Length)));
        var aliasTargets=sections.SelectMany(s=>s.SourceNodeAliases.Select(a=>(s.Kind,a))).GroupBy(x=>new SourceNodeKey(x.Kind,x.a.SourceNodeId))
            .Where(g=>g.Select(x=>x.a.CanonicalNodeId).Distinct().Count()!=1||!present.Contains(new(g.Key.Kind,g.First().a.CanonicalNodeId))).Take(int.TryParse(Environment.GetEnvironmentVariable("VISUAL_BAD_ALIASES"),out var shownAliases)?shownAliases:6).ToArray();
        foreach(var g in aliasTargets)Console.WriteLine($"  bad alias {g.Key.Kind}:{g.Key.Id} targets={string.Join(",",g.Select(x=>x.a.CanonicalNodeId).Distinct())} targetPresent={g.Select(x=>x.a.CanonicalNodeId).Distinct().Select(t=>present.Contains(new(g.Key.Kind,t))).FirstOrDefault()} inSource={c.Source.Any(s=>s.Kind==g.Key.Kind&&s.SourceNodeAliases.Any(a=>a.SourceNodeId==g.Key.Id&&a.CanonicalNodeId==g.First().a.CanonicalNodeId))} targetIsSourceNode={c.Source.Any(s=>s.Kind==g.Key.Kind&&(s.StartNodeId==g.First().a.CanonicalNodeId||s.EndNodeId==g.First().a.CanonicalNodeId))}");
        File.WriteAllText(Path.Combine(output,RunFile),JsonConvert.SerializeObject(new{capture=Path.GetFileName(c.Folder),pipeline,
            latitude=(double)c.Input["request"]!["Latitude"]!,longitude=(double)c.Input["request"]!["Longitude"]!,
            sourceSections=c.Source.Length,designedSections=designed.Count,measured.Sections,measured.Kilometres,paired=measured.Paired,designSeconds,metricSeconds,
            topology,compilerLog=log.Lines.Where(l=>l.Contains("VISUAL COMPILER",StringComparison.Ordinal)).ToArray()},Formatting.Indented));
        Console.WriteLine($"{pipeline}: designed {designed.Count} sections in {designSeconds:F1} s; emitted {sections.Count}");
        foreach(var line in topology)Console.WriteLine("  "+line);
        // Where do tangent breaks sit: generated nodes, source ports, or stroke joins?
        var breaks=measured.Failures.Where(f=>f.Category=="tangent-break").ToArray();
        string NodeClass(VisualCompilerMetrics.Failure f)
        {
            var s=sections[f.Section];var id=math.distance((double3)s.CourseStart,f.Position)<math.distance((double3)s.CourseEnd,f.Position)?s.StartNodeId:s.EndNodeId;
            return id<=long.MinValue+9000000?"generated-compiler":id<0?"generated-earlier":"source-node";
        }
        foreach(var g in breaks.GroupBy(NodeClass))Console.WriteLine($"  tangent-break at {g.Key}: {g.Count()} (median {g.Select(f=>f.Value).OrderBy(v=>v).ElementAt(g.Count()/2):F1}°)");
        // VISUAL_DEBUG_GRADES=1: road grade-change failures by where they sit: the
        // worst sample at a section end (mostly a cubic's own curvature, which is
        // largest at an end; a kink between sections is rare) or inside it, near a structure end, a
        // junction or neither, and by class.
        if(Environment.GetEnvironmentVariable("VISUAL_DEBUG_GRADES")=="1")
        {
            var roadDegree=sections.Where(s=>s.Kind==NetworkKind.Road).SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}).GroupBy(id=>id).ToDictionary(g=>g.Key,g=>g.Count());
            var structureEnds=sections.Where(s=>s.Kind==NetworkKind.Road&&NetworkStructure.IsStructure(s)).SelectMany(s=>new[]{((double3)s.CourseStart).xz,((double3)s.CourseEnd).xz}).ToArray();
            var junctions=sections.Where(s=>s.Kind==NetworkKind.Road).SelectMany(s=>new[]{(s.StartNodeId,((double3)s.CourseStart).xz),(s.EndNodeId,((double3)s.CourseEnd).xz)}).Where(e=>roadDegree[e.Item1]>=3).Select(e=>e.Item2).ToArray();
            string Where(VisualCompilerMetrics.Failure f)
            {
                var s=sections[f.Section];var atNode=math.distance(f.Position,(double3)s.CourseStart)<.05||math.distance(f.Position,(double3)s.CourseEnd)<.05;
                var near=structureEnds.Any(p=>math.distance(p,f.Position.xz)<60)?"structure-end":junctions.Any(p=>math.distance(p,f.Position.xz)<60)?"junction":"open";
                return $"{(atNode?"end":"inside")} {near} {(s.ProtectedLink?"link":s.Classification)}{(s.Tunnel?" tunnel":s.Bridge?" bridge":"")}";
            }
            foreach(var g in measured.Failures.Where(f=>f.Category=="grade-change"&&sections[f.Section].Kind==NetworkKind.Road).GroupBy(Where).OrderByDescending(g=>g.Count()).Take(16))
                Console.WriteLine($"  grade-change {g.Key}: {g.Count()} (median {g.Select(f=>f.Value/f.Limit).OrderBy(v=>v).ElementAt(g.Count()/2):F1}x)");
        }
        // Rail on its own: failures by category, then (VISUAL_DEBUG_RAIL=n) the
        // n worst of each with location and the section's attributes.
        {
            double lat0=(double)c.Input["request"]!["Latitude"]!,lon0=(double)c.Input["request"]!["Longitude"]!;
            var railKm=sections.Where(s=>s.Kind==NetworkKind.Rail).Sum(s=>NetworkDesignChecks.HorizontalArcLength(new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd)))/1000;
            var rail=measured.Failures.Where(f=>sections[f.Section].Kind==NetworkKind.Rail).ToArray();
            Console.WriteLine($"  RAIL sections={sections.Count(s=>s.Kind==NetworkKind.Rail)}; km={railKm:F1}; failures={rail.Length}");
            int.TryParse(Environment.GetEnvironmentVariable("VISUAL_DEBUG_RAIL"),out var shownRail);
            if(shownRail>0)
            {
                foreach(var g in rail.GroupBy(f=>$"{f.Category} {(sections[f.Section].RailService is ""?"main":sections[f.Section].RailService)}").OrderByDescending(g=>g.Count()).Take(20))
                    Console.WriteLine($"  rail by service {g.Key}: {g.Count()}");
                // What the failing section's two ends are: a switch (three or more
                // arms), a structure or layer change, a source port, a plain alias
                // target, or a plain joint, and whether the end node came from the
                // source, the compiler or an earlier stage.
                var railDegree=sections.Where(s=>s.Kind==NetworkKind.Rail).SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}).GroupBy(id=>id).ToDictionary(g=>g.Key,g=>g.Count());
                var railPorts=SourceTopologyObligations.Capture(c.Source).Ports.Where(p=>p.Kind==NetworkKind.Rail).Select(p=>p.Id).ToHashSet();
                var railAliasTargets=c.Source.Where(s=>s.Kind==NetworkKind.Rail).SelectMany(s=>s.SourceNodeAliases.Select(a=>a.CanonicalNodeId)).ToHashSet();
                string RailEnd(PreparedSegment s,long id)
                {
                    var d=railDegree[id];if(d>=3)return "switch";if(d==1)return "end";
                    var o=sections.First(x=>x!=s&&x.Kind==NetworkKind.Rail&&(x.StartNodeId==id||x.EndNodeId==id));
                    if(o.Bridge!=s.Bridge||o.Tunnel!=s.Tunnel)return "structure";if(o.Layer!=s.Layer)return "layer";
                    return railPorts.Contains(id)?"port":railAliasTargets.Contains(id)?"alias":id>=0?"source-plain":id<=long.MinValue+9000000?"compiler":"other";
                }
                foreach(var g in rail.GroupBy(f=>{var s=sections[f.Section];return f.Category+" "+string.Join("+",new[]{RailEnd(s,s.StartNodeId),RailEnd(s,s.EndNodeId)}.OrderBy(x=>x));})
                    .OrderByDescending(g=>g.Count()).Take(30))
                    Console.WriteLine($"  rail ends {g.Key}: {g.Count()}");
                string Bucket(VisualCompilerMetrics.Failure f)=>f.Category switch
                {
                    "tight-radius"=>f.Value<25?"<25 m":f.Value<75?"25-75 m":f.Value<150?"75-150 m":"150-300 m",
                    "tangent-break"=>f.Value<5?"2-5°":f.Value<15?"5-15°":f.Value<45?"15-45°":"45°+",
                    "short-segment"=>f.Value<8?"<8 m":f.Value<20?"8-20 m":f.Value<40?"20-40 m":"40-60 m",
                    "grade-change" or "steep-grade"=>f.Value/f.Limit<2?"<2x":f.Value/f.Limit<5?"2-5x":"5x+",
                    _=>f.Value<1?"<1 m":f.Value<3?"1-3 m":"3 m+"
                };
                foreach(var g in rail.GroupBy(f=>f.Category).OrderBy(g=>g.Key))
                    Console.WriteLine($"  rail size {g.Key}: "+string.Join(", ",g.GroupBy(Bucket).OrderBy(x=>x.Key).Select(x=>$"{x.Key} {x.Count()}")));
                // The severe geometry: kinks over 45° and radii under 25 m, with
                // what meets at the node and the source way's own shape there.
                foreach(var f in rail.Where(f=>f.Category=="tangent-break"&&f.Value>=45||f.Category=="tight-radius"&&f.Value<25).OrderByDescending(f=>f.Severity).Take(shownRail*3))
                {
                    var s=sections[f.Section];var node=math.distance((double3)s.CourseStart,f.Position)<math.distance((double3)s.CourseEnd,f.Position)?s.StartNodeId:s.EndNodeId;
                    var arms=sections.Where(o=>o.Kind==NetworkKind.Rail&&(o.StartNodeId==node||o.EndNodeId==node)).ToArray();
                    var src=c.Source.Where(o=>o.Kind==NetworkKind.Rail&&(o.WayId==s.WayId||s.SourceWayIds.Contains(o.WayId))).ToArray();
                    Console.WriteLine($"    severe {f.Category} {f.Value:F1} way {f.WayId} at {lat0+math.degrees(f.Position.z/Earth):F5},{lon0+math.degrees(f.Position.x/(Earth*Math.Cos(math.radians(lat0)))):F5} node {node} ({RailEnd(s,node)}) arms={string.Join(";",arms.Select(o=>$"{o.WayId}/{o.RailService}/{(o.Bridge?"b":o.Tunnel?"t":"g")}/len{NetworkDesignChecks.HorizontalArcLength(new Cubic3d(o.CourseStart,o.CourseControl1,o.CourseControl2,o.CourseEnd)):F0}"))} "+
                        $"sourceWays={string.Join(",",s.SourceWayIds.Take(6))} sourceSections={src.Length} sourceServices={string.Join(",",src.Select(o=>o.RailService).Distinct())}");
                }
                var sourceRail=c.Source.Where(s=>s.Kind==NetworkKind.Rail).ToArray();
                var sourceMeasured=VisualCompilerMetrics.Measure(sourceRail,c.Terrain,c.Scale,c.Width,default);
                foreach(var g in sourceMeasured.Failures.Where(f=>f.Category is "tight-radius" or "tangent-break" or "short-segment").GroupBy(f=>f.Category).OrderBy(g=>g.Key))
                    Console.WriteLine($"  source rail size {g.Key}: "+string.Join(", ",g.GroupBy(Bucket).OrderBy(x=>x.Key).Select(x=>$"{x.Key} {x.Count()}")));
                // Hotspots: 1 km cells holding the most rail failures.
                foreach(var g in rail.GroupBy(f=>((int)Math.Floor(f.Position.x/1000),(int)Math.Floor(f.Position.z/1000))).OrderByDescending(g=>g.Count()).Take(8))
                    Console.WriteLine($"  rail hotspot ({g.Key.Item1*1000+500},{g.Key.Item2*1000+500}) at {lat0+math.degrees((g.Key.Item2*1000+500)/Earth):F4},{lon0+math.degrees((g.Key.Item1*1000+500)/(Earth*Math.Cos(math.radians(lat0)))):F4}: {g.Count()} "+
                        string.Join(" ",g.GroupBy(f=>f.Category).Select(x=>$"{x.Key}={x.Count()}")));
            }
            foreach(var g in rail.GroupBy(f=>f.Category).OrderByDescending(g=>g.Count()))
            {
                var values=g.Select(f=>f.Value).OrderBy(v=>v).ToArray();
                Console.WriteLine($"  rail {g.Key}: {g.Count()} (median {values[values.Length/2]:F3}, limit {g.First().Limit:F3})");
                foreach(var f in g.OrderByDescending(f=>f.Severity).Take(shownRail))
                {
                    var s=sections[f.Section];
                    Console.WriteLine($"    {f.Category} way {f.WayId} {f.Value:F3}/{f.Limit:F3} at {lat0+math.degrees(f.Position.z/Earth):F5},{lon0+math.degrees(f.Position.x/(Earth*Math.Cos(math.radians(lat0)))):F5} local=({f.Position.x:F0},{f.Position.z:F0}) "+
                        $"{s.StartNodeId}->{s.EndNodeId} service={s.RailService} bridge={s.Bridge} tunnel={s.Tunnel} layer={s.Layer} len={NetworkDesignChecks.HorizontalArcLength(new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd)):F1} y=({s.CourseStart.y:F1},{s.CourseEnd.y:F1})");
                }
            }
        }
        // VISUAL_DEBUG_SHORT_ENDS=1: short sections by the degree of their end
        // nodes and whether a structure (bridge/tunnel) starts or ends there.
        if(Environment.GetEnvironmentVariable("VISUAL_DEBUG_SHORT_ENDS")=="1")
        {
            var degree=sections.SelectMany(s=>new[]{(s.Kind,s.StartNodeId),(s.Kind,s.EndNodeId)}).GroupBy(k=>k).ToDictionary(g=>g.Key,g=>g.Count());
            string End(PreparedSegment s,long node)
            {
                var d=degree[(s.Kind,node)];if(d!=2)return d>=3?"junction":"dead-end";
                var other=sections.First(o=>o!=s&&o.Kind==s.Kind&&(o.StartNodeId==node||o.EndNodeId==node));
                return other.Bridge!=s.Bridge||other.Tunnel!=s.Tunnel?"structure":other.WayId!=s.WayId?"way-change":"plain";
            }
            foreach(var g in measured.Failures.Where(f=>f.Category=="short-segment").GroupBy(f=>{var s=sections[f.Section];return string.Join("+",new[]{End(s,s.StartNodeId),End(s,s.EndNodeId)}.OrderBy(x=>x))+" "+s.Kind;}).OrderByDescending(g=>g.Count()))
                Console.WriteLine($"  short-segment ends {g.Key}: {g.Count()} (median {g.Select(f=>f.Value).OrderBy(v=>v).ElementAt(g.Count()/2):F1} m)");
            // What differs across the ends of short pieces between two other ways.
            string Differs(PreparedSegment s,long node)
            {
                if(degree[(s.Kind,node)]>2)return "junction";
                var o=sections.FirstOrDefault(o=>o!=s&&o.Kind==s.Kind&&(o.StartNodeId==node||o.EndNodeId==node));if(o==null)return "end";
                var d=new List<string>();if(o.NativeAssetKey!=s.NativeAssetKey)d.Add("asset");if(o.Lanes!=s.Lanes)d.Add("lanes");if(o.Classification!=s.Classification)d.Add("class");
                if(o.OneWay!=s.OneWay)d.Add("oneway");if(o.Bridge!=s.Bridge||o.Tunnel!=s.Tunnel)d.Add("structure");if(o.Layer!=s.Layer)d.Add("layer");
                if(math.dot(math.normalizesafe((s.StartNodeId==node?s.CourseControl1-s.CourseStart:s.CourseControl2-s.CourseEnd).xz),
                    math.normalizesafe((o.StartNodeId==node?o.CourseControl1-o.CourseStart:o.CourseControl2-o.CourseEnd).xz))>-.97)d.Add("bend");
                return d.Count==0?"same":string.Join("/",d);
            }
            foreach(var g in measured.Failures.Where(f=>f.Category=="short-segment"&&sections[f.Section].Kind==NetworkKind.Road).GroupBy(f=>
            {var s=sections[f.Section];return string.Join(" + ",new[]{Differs(s,s.StartNodeId),Differs(s,s.EndNodeId)}.OrderBy(x=>x));}).OrderByDescending(g=>g.Count()).Take(12))
                Console.WriteLine($"  short-segment differs {g.Key}: {g.Count()}");
            // Plain-joined pieces: who made the end nodes and whether they are source ports.
            var ports=SourceTopologyObligations.Capture(c.Source).Ports.ToHashSet();
            string Origin(long id)=>id>=0?"source":id<=long.MinValue+9000000?"compiler":id<=long.MinValue+13000000?"unit":id<=long.MinValue+44000000?"late":"earlier";
            var differs=Environment.GetEnvironmentVariable("VISUAL_DEBUG_DIFFERS")??"same";
            var same=measured.Failures.Where(f=>f.Category=="short-segment"&&sections[f.Section].Kind==NetworkKind.Road).Select(f=>sections[f.Section])
                .Where(s=>Differs(s,s.StartNodeId)==differs||Differs(s,s.EndNodeId)==differs).ToArray();
            foreach(var g in same.SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}.Where(n=>Differs(s,n)==differs).Select(n=>$"{Origin(n)}{(ports.Contains(new(s.Kind,n))?"/port":"")} {s.Classification}{(s.Bridge?" bridge":s.Tunnel?" tunnel":"")}"))
                .GroupBy(x=>x).OrderByDescending(g=>g.Count()).Take(12))
                Console.WriteLine($"  short-segment same-end {g.Key}: {g.Count()}");
            foreach(var s in same.Take(int.TryParse(Environment.GetEnvironmentVariable("VISUAL_DEBUG_SAME"),out var shownSame)?shownSame:0))
                Console.WriteLine($"  same piece way={s.WayId} {s.StartNodeId}({Differs(s,s.StartNodeId)})->{s.EndNodeId}({Differs(s,s.EndNodeId)}) {s.Classification} layer={s.Layer} bridge={s.Bridge} tunnel={s.Tunnel} neighbours={string.Join("/",sections.Where(o=>o!=s&&o.Kind==s.Kind&&(o.StartNodeId==s.StartNodeId||o.EndNodeId==s.StartNodeId||o.StartNodeId==s.EndNodeId||o.EndNodeId==s.EndNodeId)).Select(o=>$"L{o.Layer}{(o.Bridge?"b":o.Tunnel?"t":"")}"))} len={NetworkDesignChecks.HorizontalArcLength(new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd)):F1} at ({s.CourseStart.x:F0},{s.CourseStart.z:F0})");
        }
        if(Environment.GetEnvironmentVariable("VISUAL_DEBUG_KINKS")=="1")
        {
            double lat0=(double)c.Input["request"]!["Latitude"]!,lon0=(double)c.Input["request"]!["Longitude"]!;
            foreach(var g in breaks.GroupBy(f=>$"{sections[f.Section].Kind} {(sections[f.Section].ProtectedLink?"link":sections[f.Section].Classification)} {NodeClass(f)} {(f.Value<5?"2-5°":f.Value<15?"5-15°":"15°+")}").OrderByDescending(g=>g.Count()).Take(14))
                Console.WriteLine($"    kink {g.Key}: {g.Count()}");
            // Short pieces by what made their end nodes, and whether those nodes are junctions.
            string Origin(long id)=>id>=0?"source":id<=long.MinValue+9000000?"compiler":id<=long.MinValue+13000000?"unit":id<=long.MinValue+44000000?"late":"earlier";
            var degreeById=sections.SelectMany(s=>new[]{(s.Kind,s.StartNodeId),(s.Kind,s.EndNodeId)}).GroupBy(k=>k).ToDictionary(g=>g.Key,g=>g.Count());
            foreach(var g in measured.Failures.Where(f=>f.Category=="short-segment"&&sections[f.Section].Kind==NetworkKind.Road).GroupBy(f=>
            {var s=sections[f.Section];string End(long id)=>Origin(id)+(degreeById[(s.Kind,id)]>2?"/junction":"/plain");return string.Join(" + ",new[]{End(s.StartNodeId),End(s.EndNodeId)}.OrderBy(x=>x));}).OrderByDescending(g=>g.Count()).Take(10))
                Console.WriteLine($"    short road pieces between {g.Key}: {g.Count()}");
            foreach(var g in measured.Failures.Where(f=>f.Category=="short-segment"&&
                (Origin(sections[f.Section].StartNodeId)=="unit"||Origin(sections[f.Section].EndNodeId)=="unit")).GroupBy(f=>
            {var s=sections[f.Section];return $"{s.Kind} {(s.ProtectedLink?"link":s.Classification)}{(s.Bridge?" bridge":s.Tunnel?" tunnel":"")}{(s.PairedProfileId!=0?" paired":"")}";}).OrderByDescending(g=>g.Count()).Take(10))
                Console.WriteLine($"    short pieces at unit cuts {g.Key}: {g.Count()} (median {g.Select(f=>f.Value).OrderBy(v=>v).ElementAt(g.Count()/2):F1} m of {g.First().Limit:F0})");
            foreach(var f in breaks.Where(f=>sections[f.Section].Kind==NetworkKind.Rail||sections[f.Section].Classification=="motorway").OrderByDescending(f=>f.Value).Take(8))
                Console.WriteLine($"    worst kink {sections[f.Section].Kind} {sections[f.Section].Reference} way {f.WayId} {f.Value:F0}° at {lat0+math.degrees(f.Position.z/Earth):F5},{lon0+math.degrees(f.Position.x/(Earth*Math.Cos(math.radians(lat0)))):F5} ({NodeClass(f)})");
        }
        Recluster(output);
    }

    // Tunnel cover on the final stamped terrain: ground (or water surface) above
    // the deck minus the deck, every 5 m, grouped by road reference. A tunnel is
    // exposed in game where its roof (deck + about 6 m) rises above the ground.
    // Also: tunnel and ground-level samples of other roads closer than half their
    // widths at a similar height (tunnels meeting other roads underground).
    // The game's own final check (NetworkGeometrySafety.ValidateTunnelCover): the
    // native tunnel asset's roof across its full width against the stamped
    // terrain. Failing rows are grouped by section; worst shortfall first.
    private static void FullWidthCover(Capture c,IReadOnlyList<PreparedSegment> sections,ushort[] roadbed)
    {
        double lat0=(double)c.Input["request"]!["Latitude"]!,lon0=(double)c.Input["request"]!["Longitude"]!;
        var assets=c.Contracts.ToDictionary(a=>a.PaletteKey);var count=sections.Count;
        var a=new int[count];var b=new int[count];var cc=new int[count];var d=new int[count];var heights=new double[count*4];
        for(var i=0;i<count;i++){a[i]=4*i;b[i]=4*i+1;cc[i]=4*i+2;d[i]=4*i+3;var s=sections[i];heights[a[i]]=s.CourseStart.y;heights[b[i]]=s.CourseControl1.y;heights[cc[i]]=s.CourseControl2.y;heights[d[i]]=s.CourseEnd.y;}
        var rows=new SparseProfileRows(heights.Length);var eps=Math.Max(.05,2*c.Scale/65535d);var evidence=new List<NetworkTunnelContacts.CoverEvidence>();
        var report=NetworkTunnelContacts.Append(sections,roadbed,c.Scale,c.Width,assets,a,b,cc,d,rows,eps,default,evidence:evidence.Add);rows.Freeze();
        double Residual(long r){var row=rows.Row(r);var value=-row.Target;for(var j=0;j<row.Count;j++)value+=rows.Coefficient(row.Offset+j)*heights[rows.Index(row.Offset+j)];return value*eps;}
        var worst=new Dictionary<int,double>();
        foreach(var e in evidence.Where(e=>e.FirstRow>=0))for(var r=e.FirstRow;r<e.FirstRow+e.RowCount;r++)
        {var v=Residual(r);if(v>eps&&(!worst.TryGetValue(e.Section,out var w)||v>w))worst[e.Section]=v;}
        Console.WriteLine($"  full-width cover: rows={rows.Count}; failing sections={worst.Count}; uncertified={report.Uncertified}; missingAssets={report.MissingAssets}");
        // The game's own rule from the 21 October readbacks: a tunnel section is
        // drawn as a tunnel only when its deck lies about 12 m below the ground.
        // Per section at mid-span against the lowest ground across the width.
        var open=new List<(string Name,double Depth,double Length,double3 P)>();var tunnelSections=0;
        foreach(var s in sections.Where(s=>s.Tunnel))
        {
            tunnelSections++;var curve=new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);var p=NetworkDesignChecks.Evaluate(curve,.5);
            var dir=math.normalizesafe(NetworkDesignChecks.Derivative(curve,.5).xz);var side=new double2(-dir.y,dir.x)*Math.Max(4,s.NativeWidth/2);
            var low=new[]{p.xz,p.xz+side,p.xz-side}.Min(q=>NetworkProcessor.Sample((float)q.x,(float)q.y,roadbed,c.Scale,c.Width));
            if(low-p.y<12)open.Add(((s.Kind==NetworkKind.Rail?"rail ":"")+(s.Reference.Length>0?s.Reference:s.Name.Length>0?s.Name:"way "+s.WayId),low-p.y,NetworkDesignChecks.HorizontalArcLength(curve),p));
        }
        Console.WriteLine($"  tunnel sections drawn open by the game's 12 m rule: {open.Count} of {tunnelSections} ({open.Sum(o=>o.Length):F0} m)");
        foreach(var g in open.GroupBy(o=>o.Name).OrderByDescending(g=>g.Sum(o=>o.Length)).Take(12))
        {
            var w=g.OrderBy(o=>o.Depth).First();Console.WriteLine($"    {g.Key}: {g.Sum(o=>o.Length):F0} m in {g.Count()} sections; shallowest {w.Depth:F1} m at {lat0+math.degrees(w.P.z/Earth):F5},{lon0+math.degrees(w.P.x/(Earth*Math.Cos(math.radians(lat0)))):F5}");
            if(g.Key==Environment.GetEnvironmentVariable("VISUAL_DEBUG_OPEN_REF"))
                foreach(var o in g)Console.WriteLine($"      {o.Depth:F1} m deep (deck {o.P.y:F1}, original ground {NetworkProcessor.Sample((float)o.P.x,(float)o.P.z,c.Terrain,c.Scale,c.Width):F1}), {o.Length:F0} m at {lat0+math.degrees(o.P.z/Earth):F5},{lon0+math.degrees(o.P.x/(Earth*Math.Cos(math.radians(lat0)))):F5}");
        }
        // Visible bore: anywhere across the asset's full width, every 5 m, the
        // stamped ground lower than the roof (deck + the envelope's top).
        var exposed=new List<(string Name,double Depth,double3 P,long Way)>();var checkedSamples=0;
        foreach(var s in sections.Where(s=>s.Tunnel))
        {
            if(!assets.TryGetValue(s.NativeAssetKey,out var asset)||!asset.TryEnvelope(NativeStructureMode.Tunnel,out var envelope))continue;
            var curve=new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);var n=Math.Max(2,(int)(NetworkDesignChecks.HorizontalArcLength(curve)/5));
            var halfWidth=Math.Max(s.NativeWidth,envelope.Width)/2;
            for(var k=0;k<=n;k++)
            {
                var t=k/(double)n;var p=NetworkDesignChecks.Evaluate(curve,t);var dir=math.normalizesafe(NetworkDesignChecks.Derivative(curve,t).xz);var normal=new double2(-dir.y,dir.x);
                var roof=p.y+envelope.RangeMax;var depth=0d;
                var detail="";
                foreach(var side in new[]{-1d,-.5,0,.5,1})
                {
                    var q=p.xz+normal*halfWidth*side;var built=NetworkProcessor.Sample((float)q.x,(float)q.y,roadbed,c.Scale,c.Width);
                    if(roof-built>depth){depth=roof-built;detail=$"side {side:+0.0;-0.0;0}: original ground {NetworkProcessor.Sample((float)q.x,(float)q.y,c.Terrain,c.Scale,c.Width):F1}, built {built:F1}, roof {roof:F1}, water {(c.Water(new float3((float)q.x,0,(float)q.y)) is float w?w.ToString("F1"):"-")}, from ends {t*NetworkDesignChecks.HorizontalArcLength(curve):F0}/{(1-t)*NetworkDesignChecks.HorizontalArcLength(curve):F0} m";}
                }
                checkedSamples++;if(depth>0)exposed.Add(((s.Kind==NetworkKind.Rail?"rail ":"")+(s.Reference.Length>0?s.Reference:s.Name.Length>0?s.Name:"way "+s.WayId)+" — "+detail,depth,p,s.WayId));
            }
        }
        Console.WriteLine($"  visible bore: {exposed.Count} of {checkedSamples} tunnel samples have ground below the roof somewhere across the width");
        foreach(var key in sections.Where(s=>s.Tunnel).Select(s=>s.NativeAssetKey).Distinct())
            if(assets.TryGetValue(key,out var asset)&&asset.TryEnvelope(NativeStructureMode.Tunnel,out var envelope))Console.WriteLine($"    tunnel asset {key}: roof {envelope.RangeMax:F1} m above deck, width {envelope.Width:F1} m");
        foreach(var g in exposed.GroupBy(e=>e.Way).OrderByDescending(g=>g.Max(e=>e.Depth)).Take(20))
        {var w=g.OrderByDescending(e=>e.Depth).First();Console.WriteLine($"    {w.Name} (way {w.Way}): {g.Count()*5} m exposed, roof up to {w.Depth:F1} m above ground at {lat0+math.degrees(w.P.z/Earth):F5},{lon0+math.degrees(w.P.x/(Earth*Math.Cos(math.radians(lat0)))):F5}");}
        foreach(var (i,v) in worst.OrderByDescending(p=>p.Value).Take(15))
        {
            var s=sections[i];var p=(s.CourseStart+s.CourseEnd)/2;
            Console.WriteLine($"    {(s.Reference.Length>0?s.Reference:s.Name.Length>0?s.Name:"way "+s.WayId)} (way {s.WayId}, {s.Kind}): {v:F1} m short of the required roof cover at {lat0+math.degrees(p.z/Earth):F5},{lon0+math.degrees(p.x/(Earth*Math.Cos(math.radians(lat0)))):F5}");
        }
    }

    // Elevated → ground → elevated: a run of ground road sections up to 600 m
    // long between two bridge sections, through plain two-arm nodes, whose deck
    // sags more than 2 m below the straight line between the bridge ends.
    private static void BridgeGapDips(Capture c,IReadOnlyList<PreparedSegment> sections)
    {
        double lat0=(double)c.Input["request"]!["Latitude"]!,lon0=(double)c.Input["request"]!["Longitude"]!;
        var arms=new Dictionary<long,List<int>>();
        for(var i=0;i<sections.Count;i++)if(sections[i].Kind==NetworkKind.Road)foreach(var id in new[]{sections[i].StartNodeId,sections[i].EndNodeId})
        {if(!arms.TryGetValue(id,out var l))arms[id]=l=new();l.Add(i);}
        bool Ground(int i)=>!sections[i].Bridge&&!sections[i].Tunnel;
        var seen=new HashSet<int>();var dips=new List<(string Name,double Length,double Sag,double EndsUp,double3 P)>();
        for(var i=0;i<sections.Count;i++)
        {
            if(sections[i].Kind!=NetworkKind.Road||!Ground(i)||!seen.Add(i))continue;
            var chain=new List<int>{i};var ends=new long[2];var outer=new int[2];
            for(var side=0;side<2;side++)
            {
                var node=side==0?sections[i].StartNodeId:sections[i].EndNodeId;var current=i;outer[side]=-1;
                while(true)
                {
                    var list=arms[node];if(list.Count!=2){ends[side]=node;break;}
                    var next=list[0]==current?list[1]:list[0];
                    if(!Ground(next)){ends[side]=node;outer[side]=sections[next].Bridge?next:-1;break;}
                    if(!seen.Add(next)){ends[side]=node;break;}
                    if(side==0)chain.Insert(0,next);else chain.Add(next);current=next;
                    node=sections[next].StartNodeId==node?sections[next].EndNodeId:sections[next].StartNodeId;
                }
            }
            if(outer[0]<0||outer[1]<0)continue;
            var points=new List<double3>();
            foreach(var k in chain)
            {
                var s=sections[k];var curve=new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);
                var pts=Enumerable.Range(0,9).Select(q=>NetworkDesignChecks.Evaluate(curve,q/8d)).ToList();
                if(points.Count>0&&math.distance(points[^1].xz,pts[^1].xz)<math.distance(points[^1].xz,pts[0].xz))pts.Reverse();
                points.AddRange(pts);
            }
            var at=new double[points.Count];for(var k=1;k<points.Count;k++)at[k]=at[k-1]+math.distance(points[k-1].xz,points[k].xz);
            var length=at[^1];if(length>600)continue;
            var sag=0d;var where=points[0];
            for(var k=0;k<points.Count;k++){var chord=math.lerp(points[0].y,points[^1].y,at[k]/Math.Max(1e-9,length));if(chord-points[k].y>sag){sag=chord-points[k].y;where=points[k];}}
            double Up(double3 p)=>p.y-NetworkProcessor.Sample((float)p.x,(float)p.z,c.Terrain,c.Scale,c.Width);
            double H(double2 q)=>NetworkProcessor.Sample((float)q.x,(float)q.y,c.Terrain,c.Scale,c.Width);
            double BridgeRun(int first,long node)
            {
                var total=0d;var current=first;var seenBridge=new HashSet<int>();
                while(seenBridge.Add(current)&&sections[current].Bridge)
                {
                    var s=sections[current];total+=NetworkDesignChecks.HorizontalArcLength(new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd));
                    node=s.StartNodeId==node?s.EndNodeId:s.StartNodeId;var list=arms[node];if(list.Count!=2)break;current=list[0]==current?list[1]:list[0];
                }
                return total;
            }
            var runs=$"{BridgeRun(outer[0],ends[0]):F0}/{BridgeRun(outer[1],ends[1]):F0} m";
            var raised=Enumerable.Range(1,points.Count-2).Average(k=>{var d=math.normalizesafe(points[k+1].xz-points[k-1].xz);var side=new double2(-d.y,d.x)*(sections[i].NativeWidth/2+30);return H(points[k].xz)-Math.Max(H(points[k].xz+side),H(points[k].xz-side));});
            if(sections[i].Reference==Environment.GetEnvironmentVariable("VISUAL_DEBUG_DIPS_REF"))
                Console.WriteLine($"    gap {sections[i].Reference} way {sections[i].WayId} {sections[i].Classification}: bridges {runs}, raised {raised:F1} m, {length:F0} m, {chain.Count} sections, sag {sag:F1} m, deck above ground {points.Min(Up):F1}..{points.Max(Up):F1} m, bridge ends {Up(points[0]):F1}/{Up(points[^1]):F1} m at {lat0+math.degrees(points[0].z/Earth):F5},{lon0+math.degrees(points[0].x/(Earth*Math.Cos(math.radians(lat0)))):F5}");
            if(sag>2)dips.Add((sections[i].Reference.Length>0?sections[i].Reference:sections[i].Name.Length>0?sections[i].Name:"way "+sections[i].WayId,length,sag,Math.Min(Up(points[0]),Up(points[^1])),where));
        }
        Console.WriteLine($"  bridge-gap dips (ground run ≤600 m between bridges sagging >2 m): {dips.Count}");
        foreach(var d in dips.OrderByDescending(d=>d.Sag).Take(25))
            Console.WriteLine($"    {d.Name}: {d.Length:F0} m sagging {d.Sag:F1} m; ends {d.EndsUp:F1} m above ground; local ({d.P.x:F0},{d.P.z:F0}); at {lat0+math.degrees(d.P.z/Earth):F5},{lon0+math.degrees(d.P.x/(Earth*Math.Cos(math.radians(lat0)))):F5}");
    }

    private static void TunnelCover(Capture c,IReadOnlyList<PreparedSegment> sections,ushort[] roadbed)
    {
        double lat0=(double)c.Input["request"]!["Latitude"]!,lon0=(double)c.Input["request"]!["Longitude"]!;
        var rows=new List<(string Ref,double Cover,double3 P,bool Wet,long Way)>();
        foreach(var s in sections.Where(s=>s.Tunnel))
        {
            var curve=new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);var n=Math.Max(2,(int)(NetworkDesignChecks.HorizontalArcLength(curve)/5));
            for(var k=1;k<n;k++)
            {
                var p=NetworkDesignChecks.Evaluate(curve,k/(double)n);var ground=NetworkProcessor.Sample((float)p.x,(float)p.z,roadbed,c.Scale,c.Width);
                var stage=c.Water(new float3((float)p.x,0,(float)p.z));var wet=stage is float w&&w>ground;
                rows.Add(((s.Kind==NetworkKind.Rail?"rail ":"")+(s.Reference.Length>0?s.Reference:s.Name.Length>0?s.Name:"way "+s.WayId),ground-p.y,p,wet,s.WayId));
            }
        }
        string Where(double3 p)=>$"{lat0+math.degrees(p.z/Earth):F5},{lon0+math.degrees(p.x/(Earth*Math.Cos(math.radians(lat0)))):F5}";
        if(long.TryParse(Environment.GetEnvironmentVariable("VISUAL_DEBUG_TUNNEL_WAY"),out var tunnelWay))
            foreach(var s in sections.Where(s=>s.WayId==tunnelWay||s.SourceWayIds.Contains(tunnelWay)))
            {
                var curve=new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);var length=NetworkDesignChecks.HorizontalArcLength(curve);
                var cells=string.Join(" ",Enumerable.Range(0,5).Select(k=>{var p=NetworkDesignChecks.Evaluate(curve,k/4d);
                    return $"[deck {p.y:F1} ground {NetworkProcessor.Sample((float)p.x,(float)p.z,c.Terrain,c.Scale,c.Width):F1} stamped {NetworkProcessor.Sample((float)p.x,(float)p.z,roadbed,c.Scale,c.Width):F1}]";}));
                Console.WriteLine($"    way {s.WayId} {(s.Tunnel?"tunnel":s.Bridge?"bridge":"ground")} {s.StartNodeId}->{s.EndNodeId} {length:F0} m: {cells}");
            }
        Console.WriteLine($"  tunnel samples={rows.Count}; cover<6 m={rows.Count(r=>r.Cover<6)}; cover<2 m={rows.Count(r=>r.Cover<2)}; deck above ground={rows.Count(r=>r.Cover<0)}");
        foreach(var g in rows.Where(r=>r.Cover<6).GroupBy(r=>r.Ref).OrderByDescending(g=>g.Count()).Take(14))
        {
            var worst=g.OrderBy(r=>r.Cover).First();
            Console.WriteLine($"    {g.Key}: {g.Count()*5} m with cover under 6 m ({g.Count(r=>r.Wet)*5} m under water); least {worst.Cover:F1} m at {Where(worst.P)} (way {worst.Way})");
        }
    }

    // Physical connectivity, independent of way provenance: source sections whose
    // two ends both survive (directly or through an alias) but land in different
    // connected pieces of the output graph, and output node ids whose incident
    // ends disagree in position (the game joins a new shared node by exact position).
    private static void Connectivity(string label,IReadOnlyList<PreparedSegment> source,IReadOnlyList<PreparedSegment> output)
    {
        foreach(var kind in new[]{NetworkKind.Rail,NetworkKind.Road})
        {
            var parent=new Dictionary<long,long>();
            long Find(long x){if(!parent.TryGetValue(x,out var p)){parent[x]=x;return x;}while(p!=x){var g=parent[p];parent[x]=g;x=p;p=g;}return x;}
            var aliases=new Dictionary<long,long>();var present=new HashSet<long>();var positions=new Dictionary<long,List<float3>>();
            foreach(var s in output.Where(s=>s.Kind==kind))
            {
                parent[Find(s.StartNodeId)]=Find(s.EndNodeId);present.Add(s.StartNodeId);present.Add(s.EndNodeId);
                foreach(var a in s.SourceNodeAliases)aliases[a.SourceNodeId]=a.CanonicalNodeId;
                foreach(var (id,p) in new[]{(s.StartNodeId,s.Start),(s.EndNodeId,s.End)}){if(!positions.TryGetValue(id,out var l))positions[id]=l=new();l.Add(p);}
            }
            long? Map(long id){if(aliases.TryGetValue(id,out var t))id=t;return present.Contains(id)?id:null;}
            var broken=new List<PreparedSegment>();var lost=0;
            foreach(var s in source.Where(s=>s.Kind==kind))
            {
                var a=Map(s.StartNodeId);var b=Map(s.EndNodeId);
                if(a==null||b==null){lost++;continue;}
                if(Find(a.Value)!=Find(b.Value))broken.Add(s);
            }
            var spread=positions.Select(p=>p.Value.Max(q=>math.distance(q,p.Value[0]))).ToArray();
            var sourceParent=new Dictionary<long,long>();
            long FindSource(long x){if(!sourceParent.TryGetValue(x,out var p)){sourceParent[x]=x;return x;}while(p!=x){var g=sourceParent[p];sourceParent[x]=g;x=p;p=g;}return x;}
            foreach(var s in source.Where(s=>s.Kind==kind))sourceParent[FindSource(s.StartNodeId)]=FindSource(s.EndNodeId);
            var sourcePieces=sourceParent.Keys.ToArray().Select(FindSource).Distinct().Count();
            // What breaks in game: two different node ids at one place, and two
            // sections joining the same pair of nodes.
            var points=positions.Select(p=>(Id:p.Key,P:p.Value[0])).ToArray();
            var grid=points.GroupBy(p=>((int)Math.Floor(p.P.x),(int)Math.Floor(p.P.z))).ToDictionary(g=>g.Key,g=>g.ToArray());
            var coincident=points.Where(p=>{var cx=(int)Math.Floor(p.P.x);var cz=(int)Math.Floor(p.P.z);
                for(var dz=-1;dz<=1;dz++)for(var dx=-1;dx<=1;dx++)if(grid.TryGetValue((cx+dx,cz+dz),out var near))
                    foreach(var o in near)if(o.Id!=p.Id&&math.distance(o.P.xz,p.P.xz)<1&&Math.Abs(o.P.y-p.P.y)<4)return true;return false;}).ToArray();
            var pairs=output.Where(s=>s.Kind==kind).GroupBy(s=>(Math.Min(s.StartNodeId,s.EndNodeId),Math.Max(s.StartNodeId,s.EndNodeId))).Where(g=>g.Count()>1).ToArray();
            var lengths=output.Where(s=>s.Kind==kind).Select(s=>NetworkDesignChecks.HorizontalArcLength(new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd))).ToArray();
            Console.WriteLine($"  {label} {kind} sections under 1 m={lengths.Count(l=>l<1)}, under 3 m={lengths.Count(l=>l<3)}, under 8 m={lengths.Count(l=>l<8)}");
            if(Environment.GetEnvironmentVariable("VISUAL_DEBUG_MICRO")==label)
            {
                var degree=output.Where(s=>s.Kind==kind).SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}).GroupBy(id=>id).ToDictionary(g=>g.Key,g=>g.Count());
                foreach(var s in output.Where(s=>s.Kind==kind).Where((s,i)=>lengths[i]<1).Take(12))
                    Console.WriteLine($"    micro way {s.WayId} {s.StartNodeId}(deg {degree[s.StartNodeId]})->{s.EndNodeId}(deg {degree[s.EndNodeId]}) br={s.Bridge} tu={s.Tunnel} terminal={s.StartTerminal}/{s.EndTerminal} at ({s.Start.x:F1},{s.Start.z:F1}) len={NetworkDesignChecks.HorizontalArcLength(new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd)):F2}");
            }
            Console.WriteLine($"  {label} {kind} coincident distinct nodes={coincident.Length}"+(coincident.Length==0?"":$" e.g. {string.Join(" ",coincident.Take(4).Select(p=>$"{p.Id}@({p.P.x:F1},{p.P.z:F1})"))}")+
                $"; node pairs joined by several sections={pairs.Length}"+(pairs.Length==0?"":$" e.g. {string.Join(" ",pairs.Take(3).Select(g=>$"[{string.Join(",",g.Select(s=>s.WayId))}]@({g.First().Start.x:F0},{g.First().Start.z:F0})"))}"));
            if(coincident.Length>0)
            {
                var adjacent=output.Where(s=>s.Kind==kind).Select(s=>(Math.Min(s.StartNodeId,s.EndNodeId),Math.Max(s.StartNodeId,s.EndNodeId))).ToHashSet();
                var unitOf=output.Where(s=>s.Kind==kind).SelectMany(s=>new[]{(s.StartNodeId,s.NativeUnitId),(s.EndNodeId,s.NativeUnitId)}).GroupBy(p=>p.Item1).ToDictionary(g=>g.Key,g=>g.Select(p=>p.Item2).Distinct().ToArray());
                var degreeOf=output.Where(s=>s.Kind==kind).SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}).GroupBy(id=>id).ToDictionary(g=>g.Key,g=>g.Count());
                var kinds=new Dictionary<string,int>();var shown=new List<string>();
                foreach(var p in coincident)
                {
                    var cx=(int)Math.Floor(p.P.x);var cz=(int)Math.Floor(p.P.z);var o=default((long Id,float3 P));
                    for(var dz=-1;dz<=1;dz++)for(var dx=-1;dx<=1;dx++)if(grid.TryGetValue((cx+dx,cz+dz),out var near))
                        foreach(var q in near)if(q.Id!=p.Id&&math.distance(q.P.xz,p.P.xz)<1&&Math.Abs(q.P.y-p.P.y)<4)o=q;
                    var linked=adjacent.Contains((Math.Min(p.Id,o.Id),Math.Max(p.Id,o.Id)));
                    var key=(linked?"joined":"apart")+$" d={(math.distance(p.P.xz,o.P.xz)<.01?"0":"<1m")} deg={Math.Min(degreeOf[p.Id],3)}/{Math.Min(degreeOf[o.Id],3)} {(p.Id<=long.MinValue+13000000&&p.Id>=long.MinValue+12000000?"unit":p.Id<=long.MinValue+9000000?"compiler":p.Id<0?"generated":"source")}";
                    kinds[key]=kinds.TryGetValue(key,out var n)?n+1:1;
                    if(!linked&&shown.Count<6)shown.Add($"{p.Id}/{o.Id}@({p.P.x:F1},{p.P.z:F1}) units {string.Join(",",unitOf[p.Id])}|{string.Join(",",unitOf[o.Id])}");
                }
                Console.WriteLine($"    coincident kinds: {string.Join("; ",kinds.OrderByDescending(k=>k.Value).Select(k=>$"{k.Key}={k.Value}"))}"+(shown.Count>0?"; apart e.g. "+string.Join(" ",shown):""));
            }
            if(Environment.GetEnvironmentVariable("VISUAL_DEBUG_PAIRS")==label)
                foreach(var g in pairs)
                {
                    var a=g.First();var b=g.Skip(1).First();
                    double Gap(PreparedSegment x,PreparedSegment y)
                    {
                        var cx=new Cubic3d(x.CourseStart,x.CourseControl1,x.CourseControl2,x.CourseEnd);var cy=new Cubic3d(y.CourseStart,y.CourseControl1,y.CourseControl2,y.CourseEnd);
                        var worst=0d;for(var k=0;k<=20;k++){var p=NetworkDesignChecks.Evaluate(cx,k/20d);var best=double.MaxValue;for(var j=0;j<=200;j++)best=Math.Min(best,math.distance(p.xz,NetworkDesignChecks.Evaluate(cy,j/200d).xz));worst=Math.Max(worst,best);}
                        return worst;
                    }
                    string D(PreparedSegment s)=>$"way {s.WayId} {s.NativeAssetKey} L={NetworkDesignChecks.HorizontalArcLength(new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd)):F0} br={s.Bridge} tu={s.Tunnel} tracks={s.PhysicalTracks} ways=[{string.Join(",",s.SourceWayIds)}]";
                    Console.WriteLine($"    pair x{g.Count()} sep={Gap(a,b):F1} m: {D(a)} | {D(b)}");
                }
            Console.WriteLine($"  {label} {kind} connectivity: source sections split across output pieces={broken.Count}; ends without output node={lost}; "+
                $"pieces={present.Select(Find).Distinct().Count()} (source {sourcePieces}); nodes with ends apart >1 cm={spread.Count(d=>d>.01f)}, >0.5 m={spread.Count(d=>d>.5f)}"+
                (broken.Count==0?"":"; e.g. "+string.Join(" ",broken.Take(5).Select(s=>$"way {s.WayId} {s.StartNodeId}->{s.EndNodeId}@({s.Start.x:F0},{s.Start.z:F0})"))));
        }
    }

    // NetworkContinuity's rail overlap samples grouped by section pair: how the
    // two tracks relate (shared node, chain ends) and how long they overlap.
    private static void RailOverlaps(IReadOnlyList<PreparedSegment> sections)
    {
        var rails=sections.Where(s=>s.Kind==NetworkKind.Rail).ToArray();
        var degree=rails.SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}).GroupBy(id=>id).ToDictionary(g=>g.Key,g=>g.Count());
        var samples=new Dictionary<(int,int),List<(int Edge,double3 P,double2 H)>>();var pairs=new Dictionary<(int,int),int>();
        for(var e=0;e<rails.Length;e++)
        {
            var s=rails[e];var curve=new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);var steps=Math.Max(2,(int)Math.Ceiling(NetworkDesignChecks.HorizontalArcLength(curve)/4));
            for(var i=1;i<steps;i++)
            {
                var t=i/(double)steps;var p=NetworkDesignChecks.Evaluate(curve,t);var h=math.normalizesafe(NetworkDesignChecks.Derivative(curve,t).xz);
                var x=(int)Math.Floor(p.x/4);var z=(int)Math.Floor(p.z/4);
                for(var dz=-1;dz<=1;dz++)for(var dx=-1;dx<=1;dx++)if(samples.TryGetValue((x+dx,z+dz),out var list))foreach(var o in list)
                {
                    if(o.Edge==e)continue;var r=rails[o.Edge];if(r.Bridge!=s.Bridge||r.Tunnel!=s.Tunnel||Math.Abs(p.y-o.P.y)>=7)continue;
                    if(math.distance(p.xz,o.P.xz)>=3||Math.Abs(math.dot(h,o.H))<.97)continue;
                    var shared=new[]{s.StartNodeId,s.EndNodeId}.Intersect(new[]{r.StartNodeId,r.EndNodeId}).ToArray();
                    if(shared.Any(id=>math.distance(p.xz,(id==s.StartNodeId?s.CourseStart:s.CourseEnd).xz)<12))continue;
                    var key=(Math.Min(e,o.Edge),Math.Max(e,o.Edge));pairs[key]=pairs.TryGetValue(key,out var n)?n+1:1;
                }
                if(!samples.TryGetValue((x,z),out var bucket))samples[(x,z)]=bucket=new();bucket.Add((e,p,h));
            }
        }
        // Chains between nodes of degree other than two.
        var incident=rails.Select((s,i)=>(s,i)).SelectMany(p=>new[]{(Id:p.s.StartNodeId,p.i),(Id:p.s.EndNodeId,p.i)}).GroupBy(x=>x.Id).ToDictionary(g=>g.Key,g=>g.Select(x=>x.i).ToArray());
        var chainOf=new int[rails.Length];Array.Fill(chainOf,-1);var chainEnds=new List<(long A,long B)>();
        for(var i=0;i<rails.Length;i++)
        {
            if(chainOf[i]>=0)continue;var id=chainEnds.Count;var ends=new List<long>();
            foreach(var forward in new[]{true,false})
            {
                var index=i;var node=forward?rails[i].EndNodeId:rails[i].StartNodeId;chainOf[i]=id;
                while(degree[node]==2)
                {
                    var next=incident[node][0]==index?incident[node][1]:incident[node][0];if(chainOf[next]>=0)break;
                    chainOf[next]=id;index=next;node=rails[next].StartNodeId==node?rails[next].EndNodeId:rails[next].StartNodeId;
                }
                ends.Add(node);
            }
            chainEnds.Add((Math.Min(ends[0],ends[1]),Math.Max(ends[0],ends[1])));
        }
        var chainPairs=pairs.GroupBy(p=>(Math.Min(chainOf[p.Key.Item1],chainOf[p.Key.Item2]),Math.Max(chainOf[p.Key.Item1],chainOf[p.Key.Item2]))).ToDictionary(g=>g.Key,g=>g.Sum(p=>p.Value));
        string ChainShape((int X,int Y) c)
        {
            if(c.X==c.Y)return "same chain";var x=chainEnds[c.X];var y=chainEnds[c.Y];
            var shared=new[]{x.A,x.B}.Intersect(new[]{y.A,y.B}).Count();
            return shared==2?"duplicate route":shared==1?"branch from shared junction":"independent parallel";
        }
        foreach(var g in chainPairs.GroupBy(p=>ChainShape(p.Key)).OrderByDescending(g=>g.Sum(p=>p.Value)))
            Console.WriteLine($"    chains {g.Key}: pairs={g.Count()} overlap≈{4*g.Sum(p=>p.Value)} m");
        if(Environment.GetEnvironmentVariable("VISUAL_DEBUG_RAIL_OVERLAP")=="2")
        {
            double ChainLength(int c)=>rails.Where((s,i)=>chainOf[i]==c).Sum(s=>NetworkDesignChecks.HorizontalArcLength(new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd)));
            string Service(int c)=>string.Join("+",rails.Where((s,i)=>chainOf[i]==c).Select(s=>s.RailService.Length==0?"main":s.RailService).Distinct());
            foreach(var p in chainPairs.OrderByDescending(p=>p.Value))
            {
                var at=rails[pairs.Keys.First(k=>(Math.Min(chainOf[k.Item1],chainOf[k.Item2]),Math.Max(chainOf[k.Item1],chainOf[k.Item2]))==p.Key).Item1].Start;
                Console.WriteLine($"    pair {ChainShape(p.Key)} overlap≈{4*p.Value} m; chain A {ChainLength(p.Key.Item1):F0} m {Service(p.Key.Item1)} ends deg {degree[chainEnds[p.Key.Item1].A]}/{degree[chainEnds[p.Key.Item1].B]}; "+
                    $"chain B {ChainLength(p.Key.Item2):F0} m {Service(p.Key.Item2)} ends deg {degree[chainEnds[p.Key.Item2].A]}/{degree[chainEnds[p.Key.Item2].B]}; at ({at.x:F0},{at.z:F0})");
            }
        }
        string Shape(PreparedSegment a,PreparedSegment b)
        {
            var shared=new[]{a.StartNodeId,a.EndNodeId}.Intersect(new[]{b.StartNodeId,b.EndNodeId}).Count();
            return $"shared={shared} deg=[{degree[a.StartNodeId]},{degree[a.EndNodeId]}|{degree[b.StartNodeId]},{degree[b.EndNodeId]}] sameWay={a.WayId==b.WayId||a.SourceWayIds.Intersect(b.SourceWayIds).Any()}";
        }
        Console.WriteLine($"  rail overlap pairs={pairs.Count}; samples={pairs.Values.Sum()}");
        foreach(var g in pairs.GroupBy(p=>Shape(rails[p.Key.Item1],rails[p.Key.Item2])).OrderByDescending(g=>g.Sum(p=>p.Value)).Take(10))
            Console.WriteLine($"    {g.Key}: pairs={g.Count()} samples={g.Sum(p=>p.Value)}");
        foreach(var p in pairs.OrderByDescending(p=>p.Value).Take(8))
        {
            var a=rails[p.Key.Item1];var b=rails[p.Key.Item2];
            Console.WriteLine($"    worst {p.Value}: way {a.WayId} {a.StartNodeId}->{a.EndNodeId} | way {b.WayId} {b.StartNodeId}->{b.EndNodeId} at ({a.Start.x:F0},{a.Start.z:F0}) ways=[{string.Join(",",a.SourceWayIds)}]|[{string.Join(",",b.SourceWayIds)}] service={a.RailService}/{b.RailService}");
        }
    }

    // Breaks in the live native network a diagnostics run recorded: dead-end
    // nodes lying within 3 m of a different node of the same kind.
    public static void LiveBreaks(string folder)
    {
        var names=CapturedRunFiles.Names(folder,"native-owned-final-",".ndjson.gz").Where(n=>System.Text.RegularExpressions.Regex.IsMatch(n,@"^native-owned-final-\d+\.ndjson\.gz$")).ToArray();
        var records=names.SelectMany(n=>CapturedRunFiles.Records(CapturedRunFiles.Resolve(folder,n))).ToArray();
        float3 P(JToken t)=>new((float)t["x"]!,(float)t["y"]!,(float)t["z"]!);
        if(Environment.GetEnvironmentVariable("LIVE_SIDES")=="1")
        {
            // Native side compositions against the deck's height above the captured
            // terrain at mid-span: where the game switches between walls and pillars.
            var c=Load(folder);
            foreach(var g in records.Where(r=>(string?)r["kind"]=="Road").GroupBy(r=>$"general={(string?)r["general"]} left={(string?)r["left"]} right={(string?)r["right"]}").OrderByDescending(g=>g.Count()).Take(16))
            {
                var heights=g.Select(r=>{var a=P(r["a"]!);var d=P(r["d"]!);var m=(a+d)*.5f;return (double)m.y-NetworkProcessor.Sample(m.x,m.z,c.Terrain,c.Scale,c.Width);}).OrderBy(v=>v).ToArray();
                double Q(double q)=>heights[(int)(q*(heights.Length-1))];
                Console.WriteLine($"  {g.Key}: {g.Count()} edges; deck above terrain P10 {Q(.1):F1} m, P50 {Q(.5):F1} m, P90 {Q(.9):F1} m");
            }
            return;
        }
        if(Environment.GetEnvironmentVariable("LIVE_TUNNELS")=="1")
        {
            // Edges below the captured terrain: how deep each native composition
            // sits, to find the depth at which the game draws a tunnel rather than
            // a lowered (open) road.
            var c=Load(folder);
            foreach(var g in records.Select(r=>(r,Depth:(double)NetworkProcessor.Sample(((P(r["a"]!)+P(r["d"]!))*.5f).x,((P(r["a"]!)+P(r["d"]!))*.5f).z,c.Terrain,c.Scale,c.Width)-((P(r["a"]!)+P(r["d"]!))*.5f).y))
                .Where(x=>x.Depth>1).GroupBy(x=>$"{(string?)x.r["kind"]} general={(string?)x.r["general"]} left={(string?)x.r["left"]} right={(string?)x.r["right"]}").OrderByDescending(g=>g.Count()).Take(14))
            {
                var d=g.Select(x=>x.Depth).OrderBy(v=>v).ToArray();double Q(double q)=>d[(int)(q*(d.Length-1))];
                Console.WriteLine($"  {g.Key}: {g.Count()} edges; depth below terrain min {d[0]:F1}, P10 {Q(.1):F1}, P50 {Q(.5):F1}, P90 {Q(.9):F1} m");
            }
            // Submitted tunnel sections, and whether the live edge over their midpoint is a tunnel.
            var prepared=CorrectiveCapturedReplay.ReadManifest<JObject>(folder,"final-prepared-network-chunks.json").Where(r=>(bool)r["Tunnel"]!).ToArray();
            var cells=records.GroupBy(r=>{var m=(P(r["a"]!)+P(r["d"]!))*.5f;return((int)Math.Floor(m.x/16),(int)Math.Floor(m.z/16));}).ToDictionary(g=>g.Key,g=>g.ToArray());
            var outcome=new List<(bool Tunnel,double Depth,double Elevation,float3 P,string Way)>();
            foreach(var r in prepared)
            {
                var a=P(r["a"]!);var dd=P(r["d"]!);var m=(a+dd)*.5f;var cx=(int)Math.Floor(m.x/16);var cz=(int)Math.Floor(m.z/16);
                JToken? live=null;var best=4f;
                for(var z=cz-1;z<=cz+1;z++)for(var x=cx-1;x<=cx+1;x++)if(cells.TryGetValue((x,z),out var list))foreach(var e in list)
                {var em=(P(e["a"]!)+P(e["d"]!))*.5f;var dist=math.distance(em.xz,m.xz);if(dist<best){best=dist;live=e;}}
                if(live==null)continue;
                var depth=NetworkProcessor.Sample(m.x,m.z,c.Terrain,c.Scale,c.Width)-m.y;
                var elevation=(double?)r["StartElevation"]??double.NaN;
                outcome.Add((((string?)live["general"]??"").Contains("Tunnel"),depth,elevation,m,(string)r["WayId"]!));
            }
            Console.WriteLine($"  submitted tunnel sections matched to live edges: {outcome.Count}; drawn as tunnel {outcome.Count(o=>o.Tunnel)}; drawn open {outcome.Count(o=>!o.Tunnel)}");
            foreach(var band in new[]{(0d,5d),(5d,8d),(8d,10d),(10d,12d),(12d,1000d)})
            {var g=outcome.Where(o=>o.Depth>=band.Item1&&o.Depth<band.Item2).ToArray();if(g.Length>0)Console.WriteLine($"    depth {band.Item1}-{band.Item2} m: {g.Length} sections, {g.Count(o=>!o.Tunnel)} drawn open; submitted elevation P50 {g.Select(o=>o.Elevation).OrderBy(v=>v).ElementAt(g.Length/2):F1}");}
            return;
        }
        foreach(var kind in new[]{"Rail","Road"})
        {
            var ends=records.Where(r=>(string?)r["kind"]==kind).SelectMany(r=>new[]{(Node:(string)r["startNode"]!,P:P(r["a"]!),Edge:(string)r["entity"]!,Elevated:((string?)r["general"]??"").Contains("Elevated")),
                (Node:(string)r["endNode"]!,P:P(r["d"]!),Edge:(string)r["entity"]!,Elevated:((string?)r["general"]??"").Contains("Elevated"))}).ToArray();
            var degree=ends.GroupBy(e=>e.Node).ToDictionary(g=>g.Key,g=>g.Count());
            var cells=ends.GroupBy(e=>((int)Math.Floor(e.P.x/4),(int)Math.Floor(e.P.z/4))).ToDictionary(g=>g.Key,g=>g.ToArray());
            var breaks=new List<(string Node,float3 P,float Gap,float Rise,bool Elevated,int OtherDegree)>();
            foreach(var e in ends.Where(e=>degree[e.Node]==1))
            {
                var best=(Gap:float.MaxValue,Rise:0f,Degree:0);
                var cx=(int)Math.Floor(e.P.x/4);var cz=(int)Math.Floor(e.P.z/4);
                for(var dz=-1;dz<=1;dz++)for(var dx=-1;dx<=1;dx++)if(cells.TryGetValue((cx+dx,cz+dz),out var near))
                    foreach(var o in near)if(o.Node!=e.Node){var d=math.distance(o.P.xz,e.P.xz);if(d<best.Gap)best=(d,o.P.y-e.P.y,degree[o.Node]);}
                if(best.Gap<3)breaks.Add((e.Node,e.P,best.Gap,best.Rise,e.Elevated,best.Degree));
            }
            Console.WriteLine($"live {kind}: edges={ends.Length/2}; nodes={degree.Count}; dead ends={degree.Count(d=>d.Value==1)}; dead ends within 3 m of another node={breaks.Count}");
            if(Environment.GetEnvironmentVariable("LIVE_PLANNED")=="1"&&CapturedRunFiles.Optional(folder,"final-prepared-network-chunks.json")!=null)
            {
                // Each live dead end against the plan: how many submitted section ends
                // met there. Two or more means the game broke a planned connection.
                var plan=CorrectiveCapturedReplay.ReadManifest<JObject>(folder,"final-prepared-network-chunks.json").Where(r=>(string?)r["kind"]==kind)
                    .SelectMany(r=>new[]{(P:P(r["nodeStart"]!),Way:(string)r["WayId"]!,Tunnel:(bool)r["Tunnel"]!,Bridge:(bool)r["Bridge"]!),(P:P(r["nodeEnd"]!),Way:(string)r["WayId"]!,Tunnel:(bool)r["Tunnel"]!,Bridge:(bool)r["Bridge"]!)}).ToArray();
                var planCells=plan.GroupBy(e=>((int)Math.Floor(e.P.x/8),(int)Math.Floor(e.P.z/8))).ToDictionary(g=>g.Key,g=>g.ToArray());
                var c0=Load(folder);double la=(double)c0.Input["request"]!["Latitude"]!,lo=(double)c0.Input["request"]!["Longitude"]!;
                var broken=new List<string>();var natural=0;
                foreach(var e in ends.Where(e=>degree[e.Node]==1))
                {
                    var cx=(int)Math.Floor(e.P.x/8);var cz=(int)Math.Floor(e.P.z/8);var met=new List<(float3 P,string Way,bool Tunnel,bool Bridge)>();
                    for(var dz=-1;dz<=1;dz++)for(var dx=-1;dx<=1;dx++)if(planCells.TryGetValue((cx+dx,cz+dz),out var near))met.AddRange(near.Where(q=>math.distance(q.P.xz,e.P.xz)<1.5f));
                    if(met.Count<2){natural++;continue;}
                    broken.Add($"{la+math.degrees(e.P.z/Earth):F5},{lo+math.degrees(e.P.x/(Earth*Math.Cos(math.radians(la)))):F5} planned ends {met.Count} [{string.Join(",",met.Select(m=>m.Way+(m.Tunnel?"/T":m.Bridge?"/B":"")))}]");
                }
                Console.WriteLine($"  live dead ends the plan connected: {broken.Count}; planned dead ends: {natural}");
                foreach(var b in broken.Take(30))Console.WriteLine("    "+b);
            }
            foreach(var g in breaks.GroupBy(b=>b.Gap<.01f?"coincident":b.Gap<.5f?"under 0.5 m":"0.5-3 m"))
                Console.WriteLine($"  {g.Key}: {g.Count()}; rise P50 {g.Select(b=>Math.Abs(b.Rise)).OrderBy(v=>v).ElementAt(g.Count()/2):F2} m; elevated {g.Count(b=>b.Elevated)}; partner degree 1/2/3+ = {g.Count(b=>b.OtherDegree==1)}/{g.Count(b=>b.OtherDegree==2)}/{g.Count(b=>b.OtherDegree>2)}; e.g. "+
                    string.Join(" ",g.Take(4).Select(b=>$"{b.Node}@({b.P.x:F1},{b.P.y:F1},{b.P.z:F1}) gap {b.Gap:F2} rise {b.Rise:F2}")));
            if(kind!="Rail"||CapturedRunFiles.Optional(folder,"final-prepared-network-chunks.json")==null)continue;
            // The submitted sections meeting at each break, with their placement order.
            var prepared=CorrectiveCapturedReplay.ReadManifest<JObject>(folder,"final-prepared-network-chunks.json");
            float3 Q(JToken t)=>new((float)t["x"]!,(float)t["y"]!,(float)t["z"]!);
            const float Radius=1;
            bool Near(JObject r,float3 spot)
            {
                var curve=new Cubic3d(Q(r["a"]!),Q(r["b"]!),Q(r["c"]!),Q(r["d"]!));
                for(var k=0;k<=200;k++)if(math.distance(NetworkDesignChecks.Evaluate(curve,k/200d).xz,(double2)spot.xz)<2)return true;
                return false;
            }
            foreach(var spot in breaks.Select(b=>b.P).Distinct().Aggregate(new List<float3>(),(l,p)=>{if(!l.Any(q=>math.distance(q.xz,p.xz)<5))l.Add(p);return l;}).Take(int.TryParse(Environment.GetEnvironmentVariable("LIVE_SPOTS"),out var n)?n:6))
            {
                Console.WriteLine($"  at ({spot.x:F1},{spot.y:F1},{spot.z:F1}):");
                foreach(var r in prepared.Where(r=>(string?)r["kind"]=="Rail"&&(math.distance(Q(r["nodeStart"]!).xz,spot.xz)<Radius||math.distance(Q(r["nodeEnd"]!).xz,spot.xz)<Radius||Near(r,spot))))
                {
                    var atStart=math.distance(Q(r["nodeStart"]!).xz,spot.xz)<=math.distance(Q(r["nodeEnd"]!).xz,spot.xz);var end=atStart?Q(r["nodeStart"]!):Q(r["nodeEnd"]!);
                    Console.WriteLine($"    way {r["WayId"]} {(atStart?"start":"end")} d={math.distance(end.xz,spot.xz):F1} node {(atStart?r["StartNodeId"]:r["EndNodeId"])} y={end.y:F2} unit={r["NativeUnitId"]} asset={r["NativeAssetKey"]} bridge={r["Bridge"]} tunnel={r["Tunnel"]} layer={r["Layer"]} class={r["Classification"]} elev={(atStart?r["StartElevation"]:r["EndElevation"])} aliases={r["SourceNodeAliases"]!.ToString(Formatting.None)}");
                }
            }
        }
    }

    public static void Recluster(string output)
    {
        var run=JObject.Parse(File.ReadAllText(Path.Combine(output,RunFile)));var failures=new List<VisualCompilerMetrics.Failure>();
        using(var file=File.OpenRead(Path.Combine(output,FailuresFile)))using(var zip=new GZipStream(file,CompressionMode.Decompress))using(var reader=new StreamReader(zip))
            while(reader.ReadLine() is {} line)
            {
                var r=JObject.Parse(line);
                failures.Add(new((string)r["Category"]!,(int)r["Section"]!,(long)r["WayId"]!,(long)r["StartNodeId"]!,(long)r["EndNodeId"]!,
                    new double3((double)r["x"]!,(double)r["y"]!,(double)r["z"]!),(double)r["Value"]!,(double)r["Limit"]!,(double)r["Severity"]!));
            }
        var issues=VisualCompilerMetrics.Cluster(failures,default);var kilometres=(double)run["Kilometres"]!;
        double lat0=(double)run["latitude"]!,lon0=(double)run["longitude"]!;
        object Located(VisualCompilerMetrics.Issue i)=>new{i.Category,i.Failures,i.Severity,i.Value,i.Limit,i.Ways,
            x=i.Position.x,y=i.Position.y,z=i.Position.z,
            latitude=lat0+math.degrees(i.Position.z/Earth),longitude=lon0+math.degrees(i.Position.x/(Earth*Math.Cos(math.radians(lat0))))};
        var categories=failures.Select(f=>f.Category).Concat(issues.Select(i=>i.Category)).Distinct().OrderBy(c=>c,StringComparer.Ordinal).Select(c=>new
        {
            category=c,failures=failures.Count(f=>f.Category==c),issues=issues.Count(i=>i.Category==c),
            issuesPer10Km=issues.Count(i=>i.Category==c)/Math.Max(1e-9,kilometres)*10
        }).ToArray();
        File.WriteAllText(Path.Combine(output,"visual-metrics.json"),JsonConvert.SerializeObject(new{run,
            totals=new{failures=failures.Count,issues=issues.Count,issuesPer10Km=issues.Count/Math.Max(1e-9,kilometres)*10},categories,
            worstIssues=issues.Take(60).Select(Located),
            worstByCategory=categories.ToDictionary(c=>c.category,c=>issues.Where(i=>i.Category==c.category).Take(10).Select(Located))},Formatting.Indented));
        Console.WriteLine($"{run["capture"]} [{run["pipeline"]}]: {run["Sections"]} sections, {kilometres:F1} km, {issues.Count} issues ({issues.Count/Math.Max(1e-9,kilometres)*10:F2} per 10 km)");
        foreach(var c in categories)Console.WriteLine($"  {c.category,-20} failures={c.failures,7} issues={c.issues,6} per10km={c.issuesPer10Km,7:F2}");
        var paired=run["paired"]!;
        Console.WriteLine($"  paired candidates: eligible={(double)paired["EligibleMeters"]!/1000:F1} km matched={(double)paired["MatchedMeters"]!/1000:F1} km failing={(double)paired["FailedMeters"]!/1000:F1} km");
    }

    // Side-by-side plan views (legacy left, compiler right) of fixed spots, each
    // spot "name@latitude,longitude"; 700 m square windows, widths to scale.
    public static void Plot(string legacy,string compiler,string spots,string svgPath)
    {
        var run=JObject.Parse(File.ReadAllText(Path.Combine(legacy,RunFile)));double lat0=(double)run["latitude"]!,lon0=(double)run["longitude"]!;
        static List<(string Kind,bool Structure,float Width,double[] P,string Label)> Sections(string folder)
        {
            var list=new List<(string,bool,float,double[],string)>();
            using var file=File.OpenRead(Path.Combine(folder,PlotFile));using var zip=new GZipStream(file,CompressionMode.Decompress);using var reader=new StreamReader(zip);
            while(reader.ReadLine() is {} line){var r=JObject.Parse(line);list.Add(((string)r["kind"]!,(bool)r["Bridge"]!||(bool)r["Tunnel"]!,(float)r["NativeWidth"]!,r["p"]!.Values<double>().ToArray(),
                $"{(long)r["WayId"]!} {(string?)r["Classification"]}{((bool?)r["OneWay"]==true?"":"2")} w{(float)r["NativeWidth"]!:F0}"));}
            return list;
        }
        var sides=new[]{Sections(legacy),Sections(compiler)};var names=spots.Split(';',StringSplitOptions.RemoveEmptyEntries);
        // VISUAL_PLOT_HALF zooms in (half the square's side, metres) and labels each section's way.
        var zoom=double.TryParse(Environment.GetEnvironmentVariable("VISUAL_PLOT_HALF"),NumberStyles.Float,CultureInfo.InvariantCulture,out var h);
        var half=zoom?h:350;const int panel=560;var svg=new StringBuilder();
        svg.Append($"<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{panel*2+30}\" height=\"{names.Length*(panel+40)}\"><rect width=\"100%\" height=\"100%\" fill=\"#1d2428\"/>");
        string N(double v)=>v.ToString("0.#",CultureInfo.InvariantCulture);
        for(var row=0;row<names.Length;row++)
        {
            var parts=names[row].Split('@');var ll=parts[1].Split(',');var lat=double.Parse(ll[0],CultureInfo.InvariantCulture);var lon=double.Parse(ll[1],CultureInfo.InvariantCulture);
            var cx=math.radians(lon-lon0)*Earth*Math.Cos(math.radians(lat0));var cz=math.radians(lat-lat0)*Earth;var top=row*(panel+40);
            for(var side=0;side<2;side++)
            {
                var left=10+side*(panel+10);var count=0;
                svg.Append($"<text x=\"{left}\" y=\"{top+20}\" fill=\"#d7e1e7\" font-size=\"14\">{parts[0]} — {(side==0?"legacy":"visual compiler")}</text>");
                svg.Append($"<svg x=\"{left}\" y=\"{top+28}\" width=\"{panel}\" height=\"{panel}\" viewBox=\"{N(cx-half)} {N(-cz-half)} {N(2*half)} {N(2*half)}\"><rect x=\"{N(cx-half)}\" y=\"{N(-cz-half)}\" width=\"{N(2*half)}\" height=\"{N(2*half)}\" fill=\"#26333a\"/>");
                foreach(var s in sides[side])
                {
                    var p=s.P;if(Math.Max(Math.Max(p[0],p[3]),Math.Max(p[6],p[9]))<cx-half||Math.Min(Math.Min(p[0],p[3]),Math.Min(p[6],p[9]))>cx+half||
                        Math.Max(Math.Max(p[2],p[5]),Math.Max(p[8],p[11]))<cz-half||Math.Min(Math.Min(p[2],p[5]),Math.Min(p[8],p[11]))>cz+half)continue;
                    var color=s.Kind=="Rail"?"#b98a5e":s.Structure?"#7fb3d5":"#c9cfd3";count++;
                    svg.Append($"<path d=\"M{N(p[0])} {N(-p[2])} C{N(p[3])} {N(-p[5])} {N(p[6])} {N(-p[8])} {N(p[9])} {N(-p[11])}\" fill=\"none\" stroke=\"{color}\" stroke-opacity=\".75\" stroke-width=\"{N(Math.Max(2,s.Width))}\"/>");
                    svg.Append($"<circle cx=\"{N(p[0])}\" cy=\"{N(-p[2])}\" r=\"{N(zoom?half/150:2.5)}\" fill=\"#e8a33d\"/>");
                    if(zoom)
                    {
                        var hue=(count*67)%360;var m=new[]{(p[0]+3*p[3]+3*p[6]+p[9])/8,(p[2]+3*p[5]+3*p[8]+p[11])/8};
                        svg.Append($"<path d=\"M{N(p[0])} {N(-p[2])} C{N(p[3])} {N(-p[5])} {N(p[6])} {N(-p[8])} {N(p[9])} {N(-p[11])}\" fill=\"none\" stroke=\"hsl({hue},80%,45%)\" stroke-width=\"{N(half/250)}\"/>");
                        svg.Append($"<text x=\"{N(m[0])}\" y=\"{N(-m[1])}\" fill=\"hsl({hue},90%,30%)\" font-size=\"{N(half/30)}\">{s.Label}</text>");
                    }
                }
                svg.Append($"</svg><text x=\"{left+6}\" y=\"{top+panel+22}\" fill=\"#93a4ad\" font-size=\"11\">{count} sections (orange dots = section starts); 700 m square; blue = bridge/tunnel</text>");
            }
        }
        svg.Append("</svg>");File.WriteAllText(svgPath,svg.ToString());Console.WriteLine("Plot: "+svgPath);
    }
}
