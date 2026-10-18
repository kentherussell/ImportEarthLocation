using System.Security.Cryptography;
using System.Text;
using ImportEarthLocation.Core;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

static class NetworkQualityComparison
{
    static JObject Read(string folder,string name)=>JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder,name)));
    static string SourceHash(string folder)
    {
        var manifest=Read(folder,"immutable-source-network-chunks.json");
        if((bool?)manifest["Truncated"]!=false)throw new InvalidDataException("Source manifest incomplete");
        using var sha=IncrementalHash.CreateHash(HashAlgorithmName.SHA256);var total=0;
        foreach(var chunk in manifest["Chunks"]!)
        {
            if((bool?)chunk["Truncated"]!=false)throw new InvalidDataException("Source chunk incomplete");var count=0;
            foreach(var record in CapturedRunFiles.Records(CapturedRunFiles.Resolve(folder,(string)chunk["File"]!)))
            {sha.AppendData(Encoding.UTF8.GetBytes(record.ToString(Formatting.None)+"\n"));count++;total++;}
            if(count!=(int)chunk["Records"]!)throw new InvalidDataException("Source chunk count mismatch");
        }
        if(total!=(int)manifest["Records"]!)throw new InvalidDataException("Source manifest count mismatch");
        return Convert.ToHexString(sha.GetHashAndReset());
    }
    public static void Run(string baseline,string candidate,string output)
    {
        var a=Read(baseline,"run.json");var b=Read(candidate,"run.json");
        foreach(var key in new[]{"Latitude","Longitude","MapSize","RoadLevel","ImportRail","IncludeSecondaryRoads","ApplyGroundSurfaces","ImportForests","CreateAirConnections","CreateSeaConnections"})
            if(a["settings"]?[key]==null||b["settings"]?[key]==null||!JToken.DeepEquals(a["settings"]?[key],b["settings"]?[key]))
                throw new InvalidDataException("Different or missing import settings: "+key);
        var sourceEqual=false;string? sourceUnavailable=null;
        try{sourceEqual=SourceHash(baseline)==SourceHash(candidate);}catch(Exception e)when(e is IOException or InvalidDataException){sourceUnavailable=e.Message;}
        var left=Read(baseline,"quality-ledger.json")["findings"]!.ToObject<NetworkQualityFinding[]>()!;
        var right=Read(candidate,"quality-ledger.json")["findings"]!.ToObject<NetworkQualityFinding[]>()!;
        string Key(NetworkQualityFinding f)=>f.Stage+"/"+f.Category+"/"+f.Scope;
        var aa=left.ToDictionary(Key);var bb=right.ToDictionary(Key);
        var findings=aa.Keys.Union(bb.Keys).Order().Select(key=>
        {
            aa.TryGetValue(key,out var before);bb.TryGetValue(key,out var after);
            double? Rate(NetworkQualityFinding? f)=>f==null||f.Checked==0?null:f.Failed/(double)f.Checked;
            return new{key,before,after,beforeFailureRate=Rate(before),afterFailureRate=Rate(after),
                regression=before!=null&&after!=null&&(after.Failed>before.Failed||after.Unavailable>before.Unavailable||after.Worst>before.Worst),
                changedDenominator=before?.Checked!=after?.Checked,missingCategoryIsNotPassed=before==null||after==null};
        }).ToArray();
        var am=(JObject)Read(baseline,"metrics.json")["metrics"]!;var bm=(JObject)Read(candidate,"metrics.json")["metrics"]!;
        var timings=am.Properties().Select(p=>p.Name).Union(bm.Properties().Select(p=>p.Name)).Where(n=>n.EndsWith(" time (s)",StringComparison.Ordinal)).Order()
            .Select(name=>new{name,beforeSeconds=(double?)am[name],afterSeconds=(double?)bm[name],
                changePercent=(double?)am[name]>0&&bm[name]!=null?100*((double)bm[name]!/(double)am[name]!-1):(double?)null}).ToArray();
        File.WriteAllText(output,JsonConvert.SerializeObject(new{baseline,candidate,sourceEqual,sourceUnavailable,
            directlyComparable=sourceEqual,findings,timings,
            timingSettingsComparable=sourceEqual&&JToken.DeepEquals(a["settings"]?["DiagnosticsEnabled"],b["settings"]?["DiagnosticsEnabled"]),
            nativeMeshAcceptance="Unavailable",interpretation="Observation counts depend on segmentation; compare failure rates, affected length, worst values and coverage together. No automatic production pass."},Formatting.Indented));
        Console.WriteLine($"Quality comparison saved: {output}; identical complete source={sourceEqual}");
    }
}
