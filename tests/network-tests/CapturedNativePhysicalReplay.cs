using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using Unity.Mathematics;

static class CapturedNativePhysicalReplay
{
    public static void SelfTest()
    {
        var root=Path.Combine(Path.GetTempPath(),"iel-native-replay-"+Guid.NewGuid().ToString("N"));
        try
        {
            var archive=new RunArchive(root,new{},"missing.dll");var raster=Enumerable.Repeat((ushort)16384,17*17).ToArray();var height=16384/65535f*200;
            var ground=new PreparedSegment(1,1,2,new(-100,height,0),new(100,height,0),NetworkKind.Road,"primary",2,false,false,false,0){NativeWidth=12,NativeAssetKey="fixture"};
            var bridge=new PreparedSegment(2,3,4,new(0,height+9,-100),new(0,height+9,100),NetworkKind.Road,"primary",2,false,true,false,1){NativeWidth=12,NativeAssetKey="fixture"};
            archive.WriteCompressedRecords("models.ndjson.gz",DiagnosticArtifacts.ExactNetworkRecords(new[]{ground,bridge}));
            var contract=new NativeAssetContract("fixture",new[]{new NativeCompositionEnvelope(NativeStructureMode.Ground,0,12,0,0,0,0,4,true),
                new NativeCompositionEnvelope(NativeStructureMode.Elevated,0,12,0,0,-2,0,4,true)},Array.Empty<float>());
            archive.WriteCompressedRecords("contracts.ndjson.gz",new object[]{contract});archive.WriteHeights("native.u16",raster);
            archive.WriteJson("native-owned-final-physical-manifest.json",new{modelFile="models.ndjson.gz",contracts="contracts.ndjson.gz",models=2,unavailable=5});
            archive.WriteJson("native-observed-terrain.json",new{registered=true,heightFile="native.u16",samples=raster.Length,scale=200,width=1000});
            var output=Path.Combine(root,"report.json");Run(archive.DirectoryPath,output);var report=JObject.Parse(File.ReadAllText(output));
            if((int?)report["sections"]!=2||(int?)report["outsideSubsetUnavailable"]!=5||(long?)report["physical"]?["ClearanceFailures"]!=0||
                (int?)report["support"]?["Failed"]!=0||(bool?)report["wholeNetworkAcceptance"]!=false)throw new Exception("Native replay changed geometry/support or certified outside its qualified subset");
            archive.WriteJson("native-owned-final-physical-manifest.json",new{models=2});var refused=false;
            try{Run(archive.DirectoryPath,output);}catch(InvalidDataException){refused=true;}
            if(!refused)throw new Exception("Native replay guessed absent source policy metadata");
            Console.WriteLine("PASS: exact native model/terrain/composition replay, subset scope and missing-policy refusal");
        }
        finally{if(Directory.Exists(root))Directory.Delete(root,true);}
    }
    public static void Run(string folder,string output)
    {
        var manifest=JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder,"native-owned-final-physical-manifest.json")));
        var modelFile=(string?)manifest["modelFile"]??throw new InvalidDataException("Historical capture lacks exact native audit models; canonical curves do not establish source policies. No guessed replay permitted.");
        var metadata=JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder,modelFile+".meta.json")));
        if((bool?)metadata["Truncated"]!=false)throw new InvalidDataException("Incomplete native audit model capture");
        var sections=CapturedRunFiles.Records(CapturedRunFiles.Resolve(folder,modelFile)).Select(r=>r.ToObject<PreparedSegment>()!).ToArray();
        if(sections.Length!=(int)metadata["Records"]!||sections.Length!=(int)manifest["models"]!)throw new InvalidDataException("Native model coverage mismatch");
        var contracts=(string)manifest["contracts"]!;
        var contractMeta=JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder,contracts+".meta.json")));
        if((bool?)contractMeta["Truncated"]!=false)throw new InvalidDataException("Incomplete native composition capture");
        var assets=CapturedRunFiles.Records(CapturedRunFiles.Resolve(folder,contracts)).Select(r=>r.ToObject<NativeAssetContract>()!).ToDictionary(a=>a.PaletteKey);
        if(assets.Count!=(int)contractMeta["Records"]!)throw new InvalidDataException("Native contract count mismatch");
        var terrain=JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder,"native-observed-terrain.json")));
        if((bool?)terrain["registered"]!=true)throw new InvalidDataException("Unregistered native terrain cannot certify support");
        var bytes=File.ReadAllBytes(CapturedRunFiles.Resolve(folder,(string)terrain["heightFile"]!));
        if(!BitConverter.IsLittleEndian||bytes.Length%2!=0||bytes.Length/2!=(int)terrain["samples"]!)throw new InvalidDataException("Native terrain encoding/coverage mismatch");
        var raster=new ushort[bytes.Length/2];Buffer.BlockCopy(bytes,0,raster,0,bytes.Length);TerrainSampling.Resolution(raster);
        var scale=(float)terrain["scale"]!;var width=(double)terrain["width"]!;
        if(!float.IsFinite(scale)||scale<=0||!double.IsFinite(width)||width<=0)throw new InvalidDataException("Invalid native terrain units");
        var watch=System.Diagnostics.Stopwatch.StartNew();var physical=NetworkTunnelProfile.MeasurePhysical(sections,raster,scale,width,assets,default);
        NetworkQualityFinding? support=null;
        NetworkTerrainSupport.Validate(sections,raster,scale,width,new ImportLog(),default,stage:"native-physical",result:value=>support=value,publish:false);
        File.WriteAllText(output,JsonConvert.SerializeObject(new{scope="captured-source-qualified-native-subset",sections=sections.Length,
            outsideSubsetUnavailable=(int)manifest["unavailable"]!,physical,support,elapsedSeconds=watch.Elapsed.TotalSeconds,
            distributedAcrossFrames=true,singleInstantSnapshot=false,wholeNetworkAcceptance=false,gameRerun=false},Formatting.Indented));
        Console.WriteLine("Native physical replay saved: "+output);
    }
}
