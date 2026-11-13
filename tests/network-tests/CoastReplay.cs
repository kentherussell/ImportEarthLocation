using System.IO.Compression;
using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Newtonsoft.Json.Linq;

// Replays the coast model (CoastalTerrain.Shape) from a run's saved coast inputs
// (CoastInput: coast-input.json, the source grid and the coast geography) and reports
// what it modelled: ocean cells, the deepest modelled floor, and thin straight runs of
// ocean through land (a row or column the scanline classification got wrong, as the
// trenches across New Orleans). Writes the shaped grid beside the report as raw
// little-endian float32 (coast-shaped.f32, same rows as the input).
static class CoastReplay
{
    // A run of ocean at least this long, with land a few cells either side across it.
    private const int SliverLength = 64;
    private const int SliverClearance = 3;

    public static void Run(string folder, string output)
    {
        var meta = JObject.Parse(File.ReadAllText(Path.Combine(folder, "coast-input.json")));
        var width = (int)meta["width"]!;
        var height = (int)meta["height"]!;
        var low = (float)meta["low"]!;
        var step = (float)meta["step"]!;
        var bytes = File.ReadAllBytes(Path.Combine(folder, (string)meta["heightsFile"]!));
        var heights = new float[width * height];

        for (var i = 0; i < heights.Length; i++)
            heights[i] = low + BitConverter.ToUInt16(bytes, 2 * i) * step;

        var geography = ReadGeography(Path.Combine(folder, (string)meta["geographyFile"]!));
        var messages = new List<string>();
        CoastalTerrain.CoastCoverage? coverage = null;

        CoastalTerrain.Shape(
            heights,
            width,
            height,
            (int)meta["zoom"]!,
            (int)meta["originX"]!,
            (int)meta["originY"]!,
            (float)meta["spacing"]!,
            geography,
            default,
            messages.Add,
            captured => coverage = captured);

        Directory.CreateDirectory(output);
        var shaped = new byte[heights.Length * 4];
        Buffer.BlockCopy(heights, 0, shaped, 0, shaped.Length);
        File.WriteAllBytes(Path.Combine(output, "coast-shaped.f32"), shaped);

        var report = new JObject
        {
            ["width"] = width,
            ["height"] = height,
            ["coastlineWays"] = geography.Ways.Values.Count(CoastalTerrain.IsCoastline),
            ["lowestMetres"] = Math.Round(heights.Min(), 1),
            ["messages"] = new JArray(messages),
        };

        if (coverage != null)
        {
            report["oceanCells"] = coverage.Ocean.Count(ocean => ocean);
            report["rowSlivers"] = Slivers(coverage.Ocean, width, height, alongRows: true);
            report["columnSlivers"] = Slivers(coverage.Ocean, width, height, alongRows: false);
        }

        File.WriteAllText(Path.Combine(output, "coast-replay.json"), report.ToString());
        Console.WriteLine(report.ToString(Newtonsoft.Json.Formatting.None));
    }

    // Runs of ocean along a row (or column) at least SliverLength long whose cells have
    // land SliverClearance cells away on both sides across the run.
    private static JArray Slivers(bool[] ocean, int width, int height, bool alongRows)
    {
        var lines = alongRows ? height : width;
        var length = alongRows ? width : height;
        var found = new JArray();

        bool Ocean(int line, int position)
            => line >= 0 && line < lines && (alongRows ? ocean[line * width + position] : ocean[position * width + line]);

        for (var line = 0; line < lines; line++)
        {
            var start = -1;

            for (var position = 0; position <= length; position++)
            {
                var thin = position < length
                    && Ocean(line, position)
                    && !Ocean(line - SliverClearance, position)
                    && !Ocean(line + SliverClearance, position);

                if (thin && start < 0)
                    start = position;

                if (thin || start < 0)
                    continue;

                if (position - start >= SliverLength)
                    found.Add($"{(alongRows ? "row" : "column")} {line}: {start}..{position - 1}");

                start = -1;
            }
        }

        return found;
    }

    private static OsmDataset ReadGeography(string path)
    {
        var nodes = new Dictionary<long, OsmNode>();
        var ways = new Dictionary<long, OsmWay>();
        var relations = new List<OsmRelation>();

        using var file = File.OpenRead(path);
        using var gzip = new GZipStream(file, CompressionMode.Decompress);
        using var reader = new StreamReader(gzip);

        while (reader.ReadLine() is { } line)
        {
            if (line.Length == 0)
                continue;

            var record = JObject.Parse(line);
            var id = (long)record["id"]!;
            var tags = (record["tags"] as JObject)?.Properties().ToDictionary(tag => tag.Name, tag => (string)tag.Value!)
                ?? new Dictionary<string, string>();

            switch ((string)record["type"]!)
            {
                case "node":
                    nodes[id] = new OsmNode(id, (double)record["lat"]!, (double)record["lon"]!);
                    break;

                case "way":
                    ways[id] = new OsmWay(id, record["nodes"]!.Select(node => (long)node).ToArray(), tags);
                    break;

                case "relation":
                    var members = record["members"]!
                        .Select(member => ((long)member["way"]!, (bool)member["inner"]!))
                        .ToArray();
                    relations.Add(new OsmRelation(id, members, tags));
                    break;
            }
        }

        return new OsmDataset { Nodes = nodes, Ways = ways, Relations = relations };
    }
}
