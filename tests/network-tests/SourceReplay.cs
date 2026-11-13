using System.Diagnostics;
using System.Reflection;
using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Newtonsoft.Json.Linq;
using Unity.Mathematics;

// Road preparation from a run's saved map data alone. It needs no design input, so it also
// works on a run that never got past preparation, and a time limit turns a hang into a failure.
static class SourceReplay
{
    private static OsmDataset LoadOsm(string folder)
    {
        Dictionary<long, T> Read<T>(string name)
            => CorrectiveCapturedReplay
                .ReadManifest<KeyValuePair<long, T>>(folder, "offline-osm-" + name + "-chunks.json")
                .ToDictionary(entry => entry.Key, entry => entry.Value);

        var metadata = JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder, "offline-osm-metadata.json")));

        return new OsmDataset
        {
            Nodes = Read<OsmNode>("nodes"),
            Ways = Read<OsmWay>("ways"),
            NodeTags = Read<IReadOnlyDictionary<string, string>>("node-tags"),
            RawWayTags = Read<IReadOnlyDictionary<string, string>>("raw-way-tags"),
            Relations = CorrectiveCapturedReplay.ReadManifest<OsmRelation>(folder, "offline-osm-relations-chunks.json"),
            SupportWayIds = metadata["SupportWayIds"]!.ToObject<long[]>()!,
            MalformedFeaturesSkipped = (int)metadata["MalformedFeaturesSkipped"]!
        };
    }

    private static ImportRequest LoadRequest(string folder)
        => JObject
            .Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder, "offline-request.json")))["request"]!
            .ToObject<ImportRequest>()!;

    public static void Run(string folder, int seconds)
    {
        var palette = JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder, "offline-palette-input.json")));
        var widths = palette["widths"]!.ToObject<Dictionary<string, float>>()!;
        var laneOffsets = palette["laneOffsets"]?.ToObject<Dictionary<string, float[]>>() ?? new();
        var log = new ImportLog();
        using var limit = new CancellationTokenSource(TimeSpan.FromSeconds(seconds));
        var clock = Stopwatch.StartNew();
        var dataset = LoadOsm(folder);
        var request = LoadRequest(folder);

        var result =
            NetworkSourcePipeline.Prepare(
                dataset,
                request,
                widths,
                laneOffsets,
                log,
                limit.Token,
                new SourceOptions
                {
                    BeforeStage = (stage, sections) => Console.WriteLine($"{clock.Elapsed.TotalSeconds,7:0.0} s  {stage}: {sections:N0} sections"),
                    AfterStage = (stage, sections, stageSeconds) =>
                    {
                        if (stageSeconds >= 1)
                            Console.WriteLine($"          {stage} took {stageSeconds:0.0} s");
                    }
                });

        Console.WriteLine($"{clock.Elapsed.TotalSeconds,7:0.0} s  done: {result.Sections.Count:N0} sections");

        foreach (var line in log.Lines.Where(text => text.StartsWith("Avenue ", StringComparison.Ordinal)))
            Console.WriteLine("  " + line);

        PrintAvenueKinks(result.Sections);
    }

    // How sharply a merged avenue changes direction where two of its sections meet, measured
    // on section ends. Turns above 45 degrees count as real corners, not jaggedness.
    private static void PrintAvenueKinks(IReadOnlyList<PreparedSegment> sections)
    {
        var arms = new Dictionary<long, List<PreparedSegment>>();

        foreach (var section in sections.Where(section => section.Kind == NetworkKind.Road))
            foreach (var node in new[] { section.StartNodeId, section.EndNodeId })
            {
                if (!arms.TryGetValue(node, out var list))
                    arms[node] = list = new();

                list.Add(section);
            }

        foreach (var (label, junction) in new[] { ("plain joins", false), ("junctions", true) })
        {
            var turns = new List<double>();

            foreach (var (node, list) in arms)
            {
                var avenue = list.Where(section => section.ConsolidatedAvenue && !section.OneWay).ToArray();

                if (avenue.Length != 2 || list.Count > 2 != junction)
                    continue;

                var into = Away(avenue[0], node);
                var outOf = Away(avenue[1], node);
                var turn = AvenueConsolidator.Angle(-into, outOf);

                if (turn <= 45)
                    turns.Add(turn);
            }

            turns.Sort();
            var median = turns.Count > 0 ? turns[turns.Count / 2] : 0;
            Console.WriteLine(
                $"  kinks {label}: {turns.Count:N0} joins; median {median:0.0} deg; over 5: {turns.Count(turn => turn > 5):N0}; over 10: {turns.Count(turn => turn > 10):N0}; over 20: {turns.Count(turn => turn > 20):N0}");
        }

        static float2 Away(PreparedSegment section, long node)
            => section.StartNodeId == node ? (section.End - section.Start).xz : (section.Start - section.End).xz;
    }

    // Each avenue pass's effect, to show whether the passes are converging.
    public static void AvenueTrace(string folder, int maximumPasses)
    {
        var log = new ImportLog();
        var dataset = LoadOsm(folder);
        var request = LoadRequest(folder);
        IReadOnlyList<PreparedSegment>? input = null;
        using var stop = new CancellationTokenSource();

        try
        {
            NetworkSourcePipeline.Prepare(
                dataset,
                request,
                new Dictionary<string, float>(),
                new Dictionary<string, float[]>(),
                log,
                stop.Token,
                new SourceOptions
                {
                    AfterStage = (stage, sections, _) =>
                    {
                        if (stage == "median-links")
                        {
                            input = sections;
                            stop.Cancel();
                        }
                    }
                });
        }
        catch (OperationCanceledException)
        {
        }

        var pass =
            typeof(AvenueConsolidator).GetMethod("PreparePass", BindingFlags.NonPublic | BindingFlags.Static)!;

        var source = StreetIntersectionConsolidator.Prepare(input!, log, CancellationToken.None);
        const string Merges = "Opposing road pairs consolidated into avenues";
        var local = false;

        for (var i = 0; i < maximumPasses; i++)
        {
            var before = log.Metric(Merges);
            source =
                (IReadOnlyList<PreparedSegment>)pass.Invoke(
                    null,
                    new object[] { source, log, CancellationToken.None, local, false })!;

            var oneWayMeters = source.Where(segment => segment.OneWay).Sum(Length);
            var avenueMeters = source.Where(segment => segment.ConsolidatedAvenue).Sum(Length);
            var merged = log.Metric(Merges) - before;
            Console.WriteLine(
                $"pass {i,4} local={local,-5} merged={merged,3} sections={source.Count,6} oneWayKm={oneWayMeters / 1000:0.000} avenueKm={avenueMeters / 1000:0.000}");

            if (merged == 0 && local)
                break;

            local |= merged == 0;
        }

        static double Length(PreparedSegment segment) => math.distance(segment.Start.xz, segment.End.xz);
    }
}
