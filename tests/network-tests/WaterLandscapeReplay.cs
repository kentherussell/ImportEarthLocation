using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using System.Globalization;
using System.IO.Compression;
using System.Text.RegularExpressions;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using Unity.Mathematics;

static partial class VisualMetricsReplay
{
    // Partial historical evidence: logged native lake controls, captured stages,
    // and final terrain. The source boundary is already past avenue merging and
    // water planning; this cannot reconstruct either upstream decision in full.
    public static void WaterLandscape(string folder,string before,string after,string output)
    {
        var c=Load(folder);var request=c.Input["request"]!.ToObject<ImportRequest>()!;
        T[] Read<T>(string name)=>CorrectiveCapturedReplay.ReadManifest<T>(folder,(string)c.Input[name]!);
        var polygons=Read<GeographicPolygon>("waterPolygonsManifest");
        var controlPattern=new Regex(@"WATER CREATED way=(?<way>-?\d+); name=lake;.*?position=float3\((?<x>[^f]+)f, (?<y>[^f]+)f, (?<z>[^f]+)f\); radius=(?<radius>[^;]+); targetDepth=(?<depth>[^;]+); targetLevel=(?<level>[^;]+);");
        var lines=File.ReadAllLines(CapturedRunFiles.Resolve(folder,"events.log"));
        var byId=polygons.GroupBy(p=>p.Id).ToDictionary(g=>g.Key,g=>g.First().Identity);
        var sources=lines.Select(l=>controlPattern.Match(l)).Where(m=>m.Success).Select(m=>
        {
            float F(string key)=>float.Parse(m.Groups[key].Value,CultureInfo.InvariantCulture);
            var way=long.Parse(m.Groups["way"].Value,CultureInfo.InvariantCulture);var p=new float3(F("x"),F("y"),F("z"));
            return new RiverSourcePlan(way,"lake","logged-native-control",p,F("radius"),F("depth"),F("level")+.35f,p)
                {BodyId=byId.GetValueOrDefault(way,way),BodyKind="lake",TargetSurfaceLevel=F("level")};
        }).ToArray();
        var cell=CapturedRunFiles.WaterCellSize(folder,c.Input);
        var water=new WaterPlan(false,0,0,0,"partial historical replay"){WaterCellSize=cell,Sources=sources,
            AreaStages=Read<WaterAreaStage>("waterAreasManifest"),ReachStages=Read<WaterReachStage>("waterReachesManifest")};
        var terrainBytes=File.ReadAllBytes(CapturedRunFiles.Resolve(folder,"native-observed-heightmap.u16"));
        var heights=new ushort[terrainBytes.Length/2];Buffer.BlockCopy(terrainBytes,0,heights,0,terrainBytes.Length);
        var terrain=new TerrainBuilder.Result(heights,c.Scale,0,0,c.Scale);var log=new ImportLog{ScalarDiagnostics=true};
        var checkedWater=WaterContainment.Validate(water,terrain,c.Width,polygons,log,default);
        var surface=WaterBodyPlanner.ConstructedSurfaceSampler(water,polygons);
        object Roads(string directory)
        {
            var wet=0;var under=0;var low=0;var sections=0;var tunnelSections=0;var maximumDepth=0d;
            var worst=new Dictionary<long,(double Depth,float3 Point,float Level,bool Bridge)>();
            using var file=File.OpenRead(Path.Combine(directory,PlotFile));using var zip=new GZipStream(file,CompressionMode.Decompress);using var reader=new StreamReader(zip);
            while(reader.ReadLine() is {} line)
            {
                var row=JObject.Parse(line);if((string?)row["kind"]!="Road")continue;
                if((bool)row["Tunnel"]!){tunnelSections++;continue;}sections++;
                var p=row["p"]!.Values<double>().ToArray();var curve=new Cubic3d(new(p[0],p[1],p[2]),new(p[3],p[4],p[5]),new(p[6],p[7],p[8]),new(p[9],p[10],p[11]));
                var count=Math.Max(1,(int)Math.Ceiling(NetworkDesignChecks.HorizontalArcLength(curve)/5));
                for(var k=0;k<=count;k++)
                {
                    var point=NetworkDesignChecks.Evaluate(curve,k/(double)count);
                    var tangent=NetworkDesignChecks.Evaluate(curve,Math.Min(1,k/(double)count+.001))-NetworkDesignChecks.Evaluate(curve,Math.Max(0,k/(double)count-.001));
                    var normal=math.normalizesafe(new double2(-tangent.z,tangent.x))*(double)row["NativeWidth"]!*.5;
                    foreach(var side in new[]{-1,0,1})
                    {
                        var q=new float3((float)(point.x+normal.x*side),(float)point.y,(float)(point.z+normal.y*side));var level=surface(q);if(!level.HasValue)continue;
                        wet++;var depth=level.Value-point.y;if(depth>0){under++;maximumDepth=Math.Max(maximumDepth,depth);
                            var way=(long)row["WayId"]!;if(!worst.TryGetValue(way,out var old)||depth>old.Depth)worst[way]=(depth,q,level.Value,(bool)row["Bridge"]!);}
                        if(depth>-.25)low++;
                    }
                }
            }
            return new{sections,tunnelSections,wetSamples=wet,underwaterSamples=under,samplesWithLessThan25cmFreeboard=low,maximumDepth,
                worstWays=worst.OrderByDescending(p=>p.Value.Depth).Take(12).Select(p=>new{way=p.Key,depth=p.Value.Depth,x=p.Value.Point.x,y=p.Value.Point.y,z=p.Value.Point.z,level=p.Value.Level,bridge=p.Value.Bridge}).ToArray()};
        }
        // Rainfall is retained as a scalar log, not a complete climate capture.
        // Use it only for density; omit temperature priors and existing tree data.
        var rainMatch=lines.Select(l=>Regex.Match(l,@"VEGETATION POLICY.*annualPrecipitationMm=([\d.]+)")).FirstOrDefault(m=>m.Success);
        var rain=rainMatch!=null?float.Parse(rainMatch.Groups[1].Value,CultureInfo.InvariantCulture):float.NaN;
        ClimateNormals? climate=float.IsFinite(rain)?new(Array.Empty<float>(),Enumerable.Repeat(rain/365,12).ToArray(),new float[12],new float[12],0,"captured annual rain only; temperatures unavailable"):null;
        var skipPlants=Environment.GetEnvironmentVariable("WATER_REPLAY_SKIP_PLANTS")=="1";
        var plants=skipPlants?VegetationPlan.Empty:VegetationPlanner.Prepare(new OsmDataset(),Array.Empty<float3>(),request,terrain,c.Source,polygons,checkedWater,climate,log,default);
        var actualSurface=WaterBodyPlanner.ConstructedSurfaceSampler(checkedWater,polygons);
        var plantedWater=plants.Count(p=>actualSurface(p.Position) is float level&&p.Position.y<=level+.5f);
        var bank=plants.Where(p=>plants.Metadata[p.MetadataIndex].Evidence.StartsWith("riparian-bank",StringComparison.Ordinal)).ToArray();
        var report=new{capture=Path.GetFileName(folder),partialEvidence=true,nativeFlowVerified=false,avenuesReplayed=false,
            roadComparisonUsesSameHistoricalStages=true,historicalStagesPredateCanalExclusion=true,before=Roads(before),after=Roads(after),
            lakes=new{loggedControls=sources.Length,loggedBodies=sources.Select(s=>s.BodyId).Distinct().Count(),acceptedControls=checkedWater.Sources.Count,
                acceptedBodies=checkedWater.Sources.Select(s=>s.BodyId).Distinct().Count(),details=log.Lines.Where(l=>l.Contains("WATER CONTAINMENT",StringComparison.Ordinal)).ToArray()},
            planting=new{skipped=skipPlants,existingTreesReplayed=false,fullClimateAvailable=false,count=bank.Length,shrubs=bank.Count(p=>p.Form==VegetationForm.Shrub),
                trees=bank.Count(p=>p.Form!=VegetationForm.Shrub),inPlannedWater=plantedWater},caveat="No pre-water DEM, full source control list, full forest/climate capture or new native water simulation is available."};
        File.WriteAllText(output,JsonConvert.SerializeObject(report,Formatting.Indented));Console.WriteLine(JsonConvert.SerializeObject(report,Formatting.Indented));
        if(plantedWater!=0)throw new Exception("Bank inference planted inside the constructed water footprint");
    }
}
