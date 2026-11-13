using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using Unity.Mathematics;

static partial class VisualMetricsReplay
{
    public static void ChannelCrossings(string folder, string osm, string output)
    {
        var input =
            JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder, "offline-design-pass-01-input.json")));

        var request = input["request"]!.ToObject<ImportRequest>()!;
        var cell = CapturedRunFiles.WaterCellSize(folder, input);
        var bytes = new FileInfo(CapturedRunFiles.Resolve(folder, (string)input["terrainFile"]!)).Length;
        var resolution = (int)Math.Sqrt(bytes / 2);

        if ((long)resolution * resolution * 2 != bytes)
            throw new InvalidDataException("Invalid captured terrain dimensions");

        var source = CorrectiveCapturedReplay.ReadManifest<PreparedSegment>(folder, (string)input["networkManifest"]!);

        if (source.Length != (int)input["sections"]!)
            throw new InvalidDataException("Captured source count mismatch");

        var data = OsmParsedCache.LoadSnapshot(osm, default);
        var channels =
            WaterBodyPlanner
                .BuriedChannels(data, request, cell)
                .Concat(WaterBodyPlanner.OpenChannels(data, request, cell))
                .ToArray();

        var after =
            NetworkStructureIntervals.CulvertBridges(
                source,
                channels,
                resolution,
                request.MapSize.WidthMeters,
                new ImportLog(),
                default);

        var report = WaterCrossings(source, after, channels, resolution, request.MapSize.WidthMeters);
        File.WriteAllText(output, JsonConvert.SerializeObject(report, Formatting.Indented));
        Console.WriteLine(JsonConvert.SerializeObject(report));
    }

    // Independently sample the remaining ground courses against Euclidean
    // channel distances. This checks classification and support envelopes;
    // native deck profiles and fluid simulation are not replayed here.
    static object WaterCrossings(IReadOnlyList<PreparedSegment> before, IReadOnlyList<PreparedSegment> after,
        IReadOnlyList<(float3[] Line, float HalfWidth)> channels, int resolution, double width)
    {
        const float size = 128;
        var cell = (float)width / (resolution - 1);
        var pieces = new List<(float3 A, float3 B, float Radius)>();
        var grid = new Dictionary<(int, int), List<int>>();

        foreach (var channel in channels)
            for (var k = 1; k < channel.Line.Length; k++)
            {
                var vectorA = channel.Line[k - 1];
                var vectorB = channel.Line[k];
                var radius = channel.HalfWidth + cell * 2;
                var id = pieces.Count;
                pieces.Add((vectorA, vectorB, radius));
                var low = (int2)math.floor((math.min(vectorA.xz, vectorB.xz) - radius) / size);
                var high = (int2)math.floor((math.max(vectorA.xz, vectorB.xz) + radius) / size);

                for (var x = low.x; x <= high.x; x++)
                    for (var z = low.y; z <= high.y; z++)
                    {
                        if (!grid.TryGetValue((x, z), out var list))
                            grid[(x, z)] = list = new();

                        list.Add(id);
                    }
            }

        long Overlaps(IReadOnlyList<PreparedSegment> source)
        {
            long wet = 0;
            var candidates = new HashSet<int>();

            foreach (var segment in source.Where(otherSegment => !otherSegment.Bridge && !otherSegment.Tunnel))
            {
                var roadHalf = Math.Max(4, segment.NativeWidth * .5f);
                var reach = (int)Math.Ceiling(roadHalf / size) + 1;
                var count =
                    Math.Max(
                        1,
                        (int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(segment) / Math.Max(2, cell * .5f)));

                for (var k = 0; k <= count; k++)
                {
                    var point = NetworkNormalizer.Evaluate(segment, k / (float)count);
                    var key = (int2)math.floor(point.xz / size);
                    candidates.Clear();

                    for (var dx = -reach; dx <= reach; dx++)
                        for (var dz = -reach; dz <= reach; dz++)
                            if (grid.TryGetValue((key.x + dx, key.y + dz), out var list))
                                candidates.UnionWith(list);

                    if (candidates.Any(
                        id => NetworkNormalizer.DistanceToSegment(point, pieces[id].A, pieces[id].B)
                            < pieces[id].Radius + roadHalf - .1f))
                        wet++;
                }
            }

            return wet;
        }

        var expected = SourceTopologyObligations.Capture(before);
        var baseline = new ImportLog { CaptureRecords = true };
        var audit = new ImportLog { CaptureRecords = true };
        SourceTopologyObligations.Validate(expected, before, baseline, default);
        SourceTopologyObligations.Validate(expected, after, audit, default);

        HashSet<string> Failed(ImportLog log)
            => log.Records.Values
                .SelectMany(r => r)
                .Select(JObject.FromObject)
                .Where(row => (bool?)row["reachable"] == false)
            .Select(
                row => string.Join(
                    "|",
                    new[] {
                        row["From"],
                        row["To"],
                        row["Ways"]
                        }.Select(token => token!.ToString(Formatting.None))))
                .ToHashSet();

        var newFailures = Failed(audit).Except(Failed(baseline)).Count();
        var newMissing =
            Math.Max(
                0,
                audit.Metric("Missing mandatory selected-source ports")
                    - baseline.Metric("Missing mandatory selected-source ports"));

        if (newFailures > 0 || newMissing > 0)
            throw new Exception(
                $"Channel bridge splitting introduced {newMissing} missing ports and {newFailures} new unresolved routes");

        double BridgeLength(IReadOnlyList<PreparedSegment> source)
            => source.Where(segment => segment.Bridge).Sum(segment => NetworkVerticalProfiles.ArcLength(segment));

        return new
        {
            inputSections = before.Count,
            outputSections = after.Count,
            bridgeSectionsBefore = before.Count(segment => segment.Bridge),
            bridgeSectionsAfter = after.Count(segment => segment.Bridge),
            bridgeMetresBefore = BridgeLength(before),
            bridgeMetresAfter = BridgeLength(after),
            groundSupportOverlapSamplesBefore = Overlaps(before),
            groundSupportOverlapSamplesAfter = Overlaps(after),
            missingPorts = audit.Metric("Missing mandatory selected-source ports"),
            unresolvedRoutes = audit.Metric("Unresolved selected-source reachability obligations"),
            baselineMissingPorts = baseline.Metric("Missing mandatory selected-source ports"),
            baselineUnresolvedRoutes = baseline.Metric("Unresolved selected-source reachability obligations"),
            newMissingPorts = newMissing,
            newUnresolvedRoutes = newFailures,
            deckProfilesReplayed = false,
            nativePlacementVerified = false
        };
    }
}
