using System.IO.Compression;
using ImportEarthLocation.Core;
using ImportEarthLocation.Geo;
using ImportEarthLocation.Processing;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using Unity.Mathematics;

// Read-only capture helpers for analysis. Mirrors tests/network-tests/CapturedRunFiles + ReadManifest.
public sealed class Cap
{
    public string Folder;
    public JObject Files = new();
    public JObject Input = new();
    public double Lat, Lon, Width;
    public float Scale;

    public static readonly string Root =
        Path.Combine(
            Environment.GetEnvironmentVariable("USERPROFILE")!,
            @"AppData\LocalLow\Colossal Order\Cities Skylines II\ModsData\ImportEarthLocation\diagnostics\runs");

    public static readonly Dictionary<string, string> Ids =
        new()
        {
            ["marseille"] = "r-f4992aedcb9847b384a41d75be492d0d",
            ["dc"] = "r-ccae1b9bd8e54ebb9775d8251c360f0c",
            ["cairo"] = "r-0210a563b9c44d478dece34aae9c805a",
            ["marseille-smooth"] = "r-41e2716355104aa18649d6cadb78bc1c",
            ["dc-smooth"] = "r-81cf4313af284ca2877ffb5cf280aa39",
            ["cairo-smooth"] = "r-468549fc09b8491c85ecf4601cfebbf9",
            ["amsterdam-smooth"] = "r-a6e40917b09941b2add4076715d85b8e",
            ["barcelona-smooth"] = "r-bcca1359a77242bf87720c36cad5e26f",
            ["buenos-aires-smooth"] = "r-f78c170eb5e24901a3cc70c8636acd07",
            ["edinburgh-smooth"] = "r-f31e58dcc17b4d768ae58cc97b5b72ef",
            ["marseille-crash"] = "r-2e1325eec7ef48e9914043f4d35a1a58",
            ["amsterdam-crash"] = "r-3299c0a184074b51ba50cfdca3c3aee7",
            ["dc-round2"] = "r-1570a7d487e543efbd376be9ab027e4b",
            ["cairo-round2"] = "r-2033698695a84127aecd5ddc52ab55b4",
            ["buenos-aires-round2"] = "r-6dccaf9a150e48bca914ee8698a52e8e",
            ["edinburgh-round2"] = "r-3ad836bf57624b7b8aab677f8449c596",
            ["barcelona-crash"] = "r-a1019eb33f944d63a8f7c239dba063f7",
            ["marseille-2"] ="r-885c4f326af741d995d138afba63911b",
            ["dc-2"] = "r-323f791af09e4e248fae3521351ca21b",
            ["marseille-7"] = "r-f50299dc9dca4086a90e424e50f36515",
            ["marseille-6"] ="r-9f8b43b30c724812a798ecf9d6b71e3e",
            ["dc-6"] = "r-31002766855d481f96c8bad38c621813",
            ["marseille-5"] ="r-431f3dc544af4f1f8865b72b78760ffe",
            ["dc-5"] = "r-558e532cf3fb464298c9acca4dbf3900",
            ["marseille-4"] = "r-e7d86dc9d1f54ee3b75ef096fdb0acba",
            ["dc-4"] = "r-1afe331761804375bf1eaaedfe1f5120",
            ["marseille-3"] = "r-be1533b07a1a40bab380a34d35481251",
            ["dc-3"] = "r-574671f365014256828f11ccfacf3683",
            ["marseille-pass"] = "r-b1449004d9524bbab1bf78fb6f78ce9b",
            ["dc-pass"] = "r-2d44ca7420014dbabc8b9945ae10c8f0",
            ["cairo-pass"] = "r-e4b6c718994f43b4b3bbe0a66870d84a",
            ["marseille-old"] = "r-f348dcdac0c54ac593fe8dbc7e358f8e",
            ["dc-old"] = "r-2374715603534270941ff92d69c05d95",
            ["marseille-overnight"] = "r-818f67aed2564a5ba691d3010cd95fb8",
            ["dc-overnight"] = "r-e49b36048e7549248a469d502cee8cd5",
            ["cairo-overnight"] = "r-f2d3e4424c9a4c4f8dadb8839ee06a27",
            ["dhaka-overnight"] = "r-0b76fbb6bc484bbb9966576ded8e859d",
            ["marseille-1112"] = "r-0f29d8bb226d48648d0f868fbc377296",
            ["dc-1112"] = "r-42d2444ffe6a499da617320c9ba8d5af",
            ["cairo-1112"] = "r-dd40aaca9b8241a8a07b9b0ba6d47d63",
            ["dhaka-1112"] = "r-dde5403afa2543dcbb55581e5d348589",
            ["santa-barbara-1112"] = "r-bc495b956bf94ad29cc1f0aed65f644f",
        ["marseille-1113"] = "r-5de037b82aa54303bd6d7a6827715629",
        ["dc-1113"] = "r-6e95bb8a3bdc48d38cfa35ba76e8ad31",
        ["cairo-1113"] = "r-88a9da46935b43b6bc7058d1dcab1938",
        ["marseille-1113b"] = "r-2b1c4a88de614b32a150c0df7749fa5f",
        ["dc-1113b"] = "r-7993ba96373a4ac8afd5fee7906cab41",
        ["cairo-1113b"] = "r-53b8b90288444fe287759b439e540292",
        ["marseille-1113d"] = "r-d63b4b30392843a894ea9c9f9fe1a090",
        ["dc-1113d"] = "r-9dc3b4d084254135b338a29ac3557fd7",
        ["cairo-1113d"] = "r-00b4dfc7377842b4bddee066c0a2c78f",
        ["montreal-1119"] = "r-16aea08148814e68a4afeb2b38c141a3",
        ["quebec-1119"] = "r-ed736ed2132b4680a5037e2160ba69c2",
        ["montreal-1120"] = "r-636d89f8b19142139e6383a053862e43",
        ["quebec-1120"] = "r-f1d692b5007c4e5c9216db47c8b51500"
        };

    public Cap(string map)
    {
        Folder = Path.Combine(Root, Ids[map]);
        var index = Path.Combine(Folder, "artifact-files.json");

        if (File.Exists(index))
            Files = (JObject)JObject.Parse(File.ReadAllText(index))["files"]!;

        Input = JObject.Parse(File.ReadAllText(Resolve("offline-design-pass-01-input.json")));
        Lat = (double)Input["request"]!["Latitude"]!;
        Lon = (double)Input["request"]!["Longitude"]!;
        Width = (double)Input["widthMeters"]!;
        Scale = (float)Input["HeightScaleMeters"]!;
    }

    public string Resolve(string name)
    {
        var text = Path.Combine(Folder, name);

        if (File.Exists(text))
            return text;

        var otherText = (string?)Files[name];

        if (otherText == null)
            throw new FileNotFoundException(name);

        return Path.Combine(Folder, otherText);
    }

    public bool Has(string name) => File.Exists(Path.Combine(Folder, name)) || Files[name] != null;

    public IEnumerable<string> Lines(string path)
    {
        using var file = File.OpenRead(path);
        using var stream =
            path.EndsWith(".gz", StringComparison.Ordinal)
                ? new GZipStream(file, CompressionMode.Decompress)
                : (Stream)file;

        using var reader = new StreamReader(stream);

        while (reader.ReadLine() is { } line)
            yield return line;
    }

    public IEnumerable<T> Manifest<T>(string logical)
    {
        var row = JObject.Parse(File.ReadAllText(Resolve(logical)));

        foreach (var chunk in row["Chunks"]!)
            foreach (var text in Lines(Resolve((string)chunk["File"]!)))
                yield return JsonConvert.DeserializeObject<T>(text)!;
    }

    public IEnumerable<JObject> ManifestJ(string logical)
    {
        var row = JObject.Parse(File.ReadAllText(Resolve(logical)));

        foreach (var chunk in row["Chunks"]!)
            foreach (var text in Lines(Resolve((string)chunk["File"]!)))
                yield return JObject.Parse(text);
    }

    // A saved 16-bit height grid, decoded to metres once as it is read.
    public float[] Heights(string name)
    {
        var b = File.ReadAllBytes(Resolve(name));
        var t = new ushort[b.Length / 2];
        Buffer.BlockCopy(b, 0, t, 0, b.Length);

        return TerrainEncoding.Decode(t, Scale);
    }

    public float3 Local(double lat, double lon) => GeoMath.ToLocalMeters(lat, lon, Lat, Lon);

    public (double lat, double lon) ToGeographic(double x, double z)
        => (Lat + z / 6378137 * 180 / Math.PI, Lon + x / (6378137 * Math.Cos(Lat * Math.PI / 180)) * 180 / Math.PI);

    public OsmDataset LoadOsm()
    {
        Dictionary<long, T> Read<T>(string name)
            => Manifest<KeyValuePair<long, T>>(
                "offline-osm-" + name + "-chunks.json").ToDictionary(entry => entry.Key, entry => entry.Value);

        var metadata = JObject.Parse(File.ReadAllText(Resolve("offline-osm-metadata.json")));

        return new OsmDataset
        {
            Nodes = Read<OsmNode>("nodes"),
            Ways = Read<OsmWay>("ways"),
            NodeTags = Read<IReadOnlyDictionary<string, string>>("node-tags"),
            RawWayTags = Read<IReadOnlyDictionary<string, string>>("raw-way-tags"),
            Relations = Manifest<OsmRelation>("offline-osm-relations-chunks.json").ToArray(),
            SupportWayIds = metadata["SupportWayIds"]!.ToObject<long[]>()!,
            MalformedFeaturesSkipped = (int)metadata["MalformedFeaturesSkipped"]!
        };
    }

    public string InputValue(string key) => (string)Input[key]!;

    public float HeightAt(float[] t, double x, double z)
        => NetworkProcessor.Sample((float)x, (float)z, t, Width);
}
