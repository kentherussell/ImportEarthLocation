#if !SURFACE_SMOKE_ONLY
using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using System.IO.Compression;
using System.Security.Cryptography;

static class SurfaceClimateReplay
{
    public static void Run(string folder,string climateFile,string osmFile,string output)
    {
        var input=JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder,"offline-design-pass-01-input.json")));
        var manifest=JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder,(string)input["waterPolygonsManifest"]!)));
        if((bool?)manifest["Truncated"]!=false)throw new InvalidDataException("Incomplete geography manifest");
        var polygons=new List<GeographicPolygon>();
        foreach(var chunk in manifest["Chunks"]!)
        {
            if((bool?)chunk["Truncated"]!=false)throw new InvalidDataException("Incomplete geography chunk");
            var path=CapturedRunFiles.Resolve(folder,(string)chunk["File"]!);var count=0;
            using var file=File.OpenRead(path);using var stream=path.EndsWith(".gz")?new GZipStream(file,CompressionMode.Decompress):(Stream)file;
            using var reader=new StreamReader(stream);while(reader.ReadLine() is {} line){polygons.Add(JsonConvert.DeserializeObject<GeographicPolygon>(line)!);count++;}
            if(count!=(int)chunk["Records"]!)throw new InvalidDataException("Geography chunk count mismatch");
        }
        if(polygons.Count!=(int)manifest["Records"]!)throw new InvalidDataException("Geography manifest count mismatch");
        var bytes=File.ReadAllBytes(CapturedRunFiles.Resolve(folder,(string)input["terrainFile"]!));var heights=new ushort[bytes.Length/2];Buffer.BlockCopy(bytes,0,heights,0,bytes.Length);
        var request=input["request"]!.ToObject<ImportRequest>()!;var scale=(float)input["HeightScaleMeters"]!;
        var climateJson=JObject.Parse(File.ReadAllText(climateFile))["properties"]!["parameter"]!;
        var months=new[]{"JAN","FEB","MAR","APR","MAY","JUN","JUL","AUG","SEP","OCT","NOV","DEC"};
        var climate=new ClimateNormals(months.Select(m=>(float)climateJson["T2M"]![m]!).ToArray(),months.Select(m=>(float)climateJson["PRECTOTCORR"]![m]!).ToArray(),new float[12],new float[12],0,"cached NASA normals");
        var data=OsmParsedCache.LoadSnapshot(osmFile,default);var log=new ImportLog();
        var events=File.ReadAllText(CapturedRunFiles.Resolve(folder,"events.log"));
        var seaMatch=System.Text.RegularExpressions.Regex.Match(events,@"Sea level: nominal ([\d.]+) m");
        if(!seaMatch.Success)throw new InvalidDataException("Captured nominal sea level unavailable");
        var sea=float.Parse(seaMatch.Groups[1].Value,System.Globalization.CultureInfo.InvariantCulture);
        var mask=SurfacePlanner.Prepare(polygons,request,new(heights,scale,sea,0,scale),log,default,data,1024,climate);
        var packed=mask.Material.Concat(mask.Intensity).ToArray();File.WriteAllBytes(output+".mask",packed);
        File.WriteAllText(output,JsonConvert.SerializeObject(new{request.Latitude,request.Longitude,annualRain=ClimateMath.AnnualPrecipitation(climate.Precipitation),
            resolution=mask.Resolution,seaLevelUsed=sea,terrainBoundary="captured pre-design conditioned terrain; no native simulation",sha256=Convert.ToHexString(SHA256.HashData(packed)),
            brushes=mask.Material.GroupBy(m=>m).ToDictionary(g=>((GroundMaterial)g.Key).ToString(),g=>g.Count()),
            climatePolicy=log.Lines.Where(l=>l.Contains("DRY GRASS",StringComparison.Ordinal)).ToArray()},Formatting.Indented));
        Console.WriteLine(File.ReadAllText(output));
    }
}
#endif
