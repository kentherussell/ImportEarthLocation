using System.IO.Compression;
using Newtonsoft.Json.Linq;

static class CapturedRunFiles
{
    public static string Resolve(string folder,string name)
        =>Optional(folder,name)??throw new InvalidDataException("Artifact unavailable: "+name);
    public static string? Optional(string folder,string name)
    {
        if(name!=Path.GetFileName(name)||name.Contains("..",StringComparison.Ordinal))throw new InvalidDataException("Artifact reference escapes capture");
        var direct=Path.Combine(folder,name);if(File.Exists(direct))return direct;
        var index=Path.Combine(folder,"artifact-files.json");
        var physical=File.Exists(index)?(string?)JObject.Parse(File.ReadAllText(index))["files"]?[name]:null;
        if(physical==null)return null;
        if(physical!=Path.GetFileName(physical)||physical.Contains("..",StringComparison.Ordinal))throw new InvalidDataException("Invalid artifact mapping: "+name);
        var path=Path.Combine(folder,physical);if(!File.Exists(path))throw new InvalidDataException("Mapped artifact unavailable: "+name);return path;
    }
    public static string[] Names(string folder,string prefix,string suffix)
    {
        var names=new HashSet<string>(Directory.EnumerateFiles(folder).Select(Path.GetFileName).OfType<string>(),StringComparer.Ordinal);
        var index=Path.Combine(folder,"artifact-files.json");
        if(File.Exists(index)&&JObject.Parse(File.ReadAllText(index))["files"] is JObject files)
            foreach(var property in files.Properties())names.Add(property.Name);
        return names.Where(name=>name.StartsWith(prefix,StringComparison.Ordinal)&&name.EndsWith(suffix,StringComparison.Ordinal))
            .OrderBy(name=>name,StringComparer.Ordinal).ToArray();
    }
    public static IEnumerable<JObject> Records(string path)
    {
        using var file=File.OpenRead(path);
        using var stream=path.EndsWith(".gz",StringComparison.Ordinal)?new GZipStream(file,CompressionMode.Decompress):(Stream)file;
        using var reader=new StreamReader(stream);while(reader.ReadLine() is {} line)yield return JObject.Parse(line);
    }
}
