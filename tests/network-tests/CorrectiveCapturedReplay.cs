using System.IO.Compression;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

static class CorrectiveCapturedReplay
{
    internal static T[] ReadManifest<T>(string folder, string logical)
    {
        var manifest = JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder, logical)));

        if ((bool?)manifest["Truncated"] != false)
            throw new InvalidDataException("Incomplete captured manifest: " + logical);

        var output = new List<T>();

        foreach (var chunk in manifest["Chunks"]!)
        {
            if ((bool?)chunk["Truncated"] != false)
                throw new InvalidDataException("Incomplete captured chunk: " + logical);

            var count = 0;
            var path = CapturedRunFiles.Resolve(folder, (string)chunk["File"]!);
            using var file = File.OpenRead(path);
            using var stream =
                path.EndsWith(".gz", StringComparison.Ordinal)
                    ? new GZipStream(file, CompressionMode.Decompress)
                    : (Stream)file;

            using var reader = new StreamReader(stream);

            while (reader.ReadLine() is { } line)
            {
                output.Add(JsonConvert.DeserializeObject<T>(line)!);
                count++;
            }

            if (count != (int)chunk["Records"]!)
                throw new InvalidDataException("Captured chunk count mismatch: " + logical);
        }

        if (output.Count != (int)manifest["Records"]!)
            throw new InvalidDataException("Captured manifest count mismatch: " + logical);

        return output.ToArray();
    }
}
