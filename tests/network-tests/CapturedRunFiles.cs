using System.IO.Compression;
using Newtonsoft.Json.Linq;

static class CapturedRunFiles
{
    public static string Resolve(string folder,string name)
    {
        if(name!=Path.GetFileName(name)||name.Contains("..",StringComparison.Ordinal))throw new InvalidDataException("Artifact reference escapes capture");
        var direct=Path.Combine(folder,name);if(File.Exists(direct))return direct;
        var index=Path.Combine(folder,"artifact-files.json");
        var physical=File.Exists(index)?(string?)JObject.Parse(File.ReadAllText(index))["files"]?[name]:null;
        if(physical==null||physical!=Path.GetFileName(physical)||physical.Contains("..",StringComparison.Ordinal))throw new InvalidDataException("Artifact unavailable: "+name);
        var path=Path.Combine(folder,physical);if(!File.Exists(path))throw new InvalidDataException("Mapped artifact unavailable: "+name);return path;
    }
    public static IEnumerable<JObject> Records(string path)
    {
        using var file=File.OpenRead(path);
        using var stream=path.EndsWith(".gz",StringComparison.Ordinal)?new GZipStream(file,CompressionMode.Decompress):(Stream)file;
        using var reader=new StreamReader(stream);while(reader.ReadLine() is {} line)yield return JObject.Parse(line);
    }
}
