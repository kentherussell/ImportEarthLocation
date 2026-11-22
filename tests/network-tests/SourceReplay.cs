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
        // IEL_REPLAY_LINES=<text>[|<text>...] turns diagnostics on and prints every log line
        // containing any of the texts.
        var linesWanted = Environment.GetEnvironmentVariable("IEL_REPLAY_LINES");
        var log = new ImportLog { ScalarDiagnostics = !string.IsNullOrEmpty(linesWanted) };
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

                        // TRACE_NEAR=x,z,radius: the road sections with an end that close to the
                        // point after every stage, to see which stage removes or moves one.
                        if (Environment.GetEnvironmentVariable("TRACE_NEAR") is { Length: > 0 } near)
                        {
                            var parts = near.Split(',').Select(value => float.Parse(value, System.Globalization.CultureInfo.InvariantCulture)).ToArray();
                            var centre = new float2(parts[0], parts[1]);

                            foreach (var section in sections.Where(
                                section => section.Kind == NetworkKind.Road
                                    && Math.Min(
                                        math.distance(section.Start.xz, centre),
                                        math.distance(section.End.xz, centre)) <= parts[2]))
                                Console.WriteLine(
                                    $"    NEAR {stage}: way {section.WayId} {section.Classification} oneWay={section.OneWay} '{section.Name}' {section.StartNodeId}->{section.EndNodeId} ({section.Start.x:0},{section.Start.z:0})->({section.End.x:0},{section.End.z:0})");
                        }
                    }
                });

        Console.WriteLine($"{clock.Elapsed.TotalSeconds,7:0.0} s  done: {result.Sections.Count:N0} sections");

        // A fingerprint of the cleaned sections (ids, ends and course by their bits), to show a
        // speed change leaves the output identical (report 118).
        var fingerprint = new System.Text.StringBuilder();

        foreach (var segment in result.Sections)
            fingerprint.Append(
                FormattableString.Invariant(
                    $"{segment.WayId},{segment.StartNodeId},{segment.EndNodeId},{BitConverter.SingleToInt32Bits(segment.CourseStart.x)},{BitConverter.SingleToInt32Bits(segment.CourseStart.z)},{BitConverter.SingleToInt32Bits(segment.CourseEnd.x)},{BitConverter.SingleToInt32Bits(segment.CourseEnd.z)},{segment.Lanes},{segment.OneWay};"));

        Console.WriteLine(
            $"  source fingerprint {Convert.ToHexString(System.Security.Cryptography.SHA256.HashData(System.Text.Encoding.UTF8.GetBytes(fingerprint.ToString())))[..16]}");

        Console.WriteLine($"  full fingerprint {FullFingerprint(result.Sections)}");

        // The cleanup stages' part times.
        foreach (var metric in log.Metrics().Where(
            metric => metric.Key.StartsWith("Normalization ", StringComparison.Ordinal)
                || metric.Key.StartsWith("Rail consolidation ", StringComparison.Ordinal)
                || metric.Key.StartsWith("Split roads ", StringComparison.Ordinal)
                || metric.Key.StartsWith("Reversible ", StringComparison.Ordinal)))
            Console.WriteLine($"  {metric.Key}: {metric.Value:0.00}");

        foreach (var line in log.Lines.Where(text => text.StartsWith("Avenue ", StringComparison.Ordinal) || text.StartsWith("AVENUE TIMES", StringComparison.Ordinal)))
            Console.WriteLine("  " + line);

        PrintAvenueKinks(result.Sections);

        // SOURCE_DUMP=<folder>: the cleaned sections as a gzip ndjson chunk and manifest, in the
        // final prepared network's field names, so a picture tool can draw them.
        if (Environment.GetEnvironmentVariable("SOURCE_DUMP") is { Length: > 0 } dump)
        {
            Directory.CreateDirectory(dump);

            using (var file = File.Create(Path.Combine(dump, "source-dump-part-0000.ndjson.gz")))
            using (var zip = new System.IO.Compression.GZipStream(file, System.IO.Compression.CompressionLevel.Fastest))
            using (var writer = new StreamWriter(zip))
                foreach (var segment in result.Sections)
                {
                    object Point(float3 point) => new { point.x, point.y, point.z };

                    writer.WriteLine(
                        Newtonsoft.Json.JsonConvert.SerializeObject(
                            new
                            {
                                segment.WayId,
                                segment.StartNodeId,
                                segment.EndNodeId,
                                kind = segment.Kind.ToString(),
                                segment.Classification,
                                segment.OneWay,
                                segment.NativeWidth,
                                segment.Name,
                                segment.Layer,
                                segment.Bridge,
                                segment.Tunnel,
                                a = Point(segment.CourseStart),
                                b = Point(segment.CourseControl1),
                                c = Point(segment.CourseControl2),
                                d = Point(segment.CourseEnd)
                            }));
                }

            File.WriteAllText(
                Path.Combine(dump, "source-dump-chunks.json"),
                "{\"Chunks\":[{\"File\":\"source-dump-part-0000.ndjson.gz\"}]}");
        }

        if (!string.IsNullOrEmpty(linesWanted))
        {
            var patterns = linesWanted.Split('|', StringSplitOptions.RemoveEmptyEntries);

            bool Wanted(string text) => patterns.Any(pattern => text.Contains(pattern, StringComparison.Ordinal));

            foreach (var line in log.Lines.Where(Wanted))
                Console.WriteLine("  " + line);
        }
    }

    // A fingerprint over far more of each section than the source fingerprint: identity, kind,
    // structure, every course point and source list, all floats by their bits.
    private static string FullFingerprint(IReadOnlyList<PreparedSegment> sections)
    {
        var text = new System.Text.StringBuilder();

        void Bits(float value) => text.Append(BitConverter.SingleToInt32Bits(value)).Append(',');

        void Point(float3 point)
        {
            Bits(point.x);
            Bits(point.y);
            Bits(point.z);
        }

        void OptionalPoint(float3? point)
        {
            if (point.HasValue)
                Point(point.Value);
            else
                text.Append("none,");
        }

        foreach (var section in sections)
        {
            text.Append(FormattableString.Invariant($"{section.WayId},{section.StartNodeId},{section.EndNodeId},"));
            text.Append(FormattableString.Invariant($"{section.Kind},{section.Classification},"));
            text.Append(
                FormattableString.Invariant(
                    $"{section.Lanes},{section.OneWay},{section.Layer},{section.Bridge},{section.Tunnel},"));
            text.Append(
                FormattableString.Invariant(
                    $"{section.Name}|{section.Reference}|{section.RailService}|{section.PhysicalTracks},"));
            text.Append(
                FormattableString.Invariant(
                    $"{section.BothWaysLanes},{section.ForwardLanes},{section.BackwardLanes},{section.Roundabout},"));
            text.Append(FormattableString.Invariant($"{section.StartTerminal},{section.EndTerminal},"));
            text.Append(FormattableString.Invariant($"{section.StartLevelCrossing},{section.EndLevelCrossing},"));
            text.Append(FormattableString.Invariant($"{section.CorridorId},{section.ConsolidatedAvenue},"));
            text.Append(FormattableString.Invariant($"{section.StartMiniRoundabout},{section.EndMiniRoundabout},"));
            Point(section.Start);
            Point(section.End);
            Point(section.CourseStart);
            Point(section.CourseControl1);
            Point(section.CourseControl2);
            Point(section.CourseEnd);
            OptionalPoint(section.Control1);
            OptionalPoint(section.Control2);
            Bits(section.NativeWidth);
            Bits(section.StartRoundaboutRadius);
            Bits(section.EndRoundaboutRadius);
            text.Append("ways:");

            foreach (var way in section.SourceWayIds)
                text.Append(way).Append(',');

            text.Append("geometry:");

            foreach (var point in section.SourceGeometry)
                Point(point);

            text.Append("original:");

            foreach (var point in section.OriginalGeometry)
                Point(point);

            text.Append("aliases:");

            foreach (var alias in section.SourceNodeAliases)
                text.Append(
                    FormattableString.Invariant($"{alias.SourceNodeId}>{alias.CanonicalNodeId}:{alias.Mandatory},"));

            text.Append("tags:").Append(section.SourceTags.Count).Append(';');
        }

        var hash = System.Security.Cryptography.SHA256.HashData(System.Text.Encoding.UTF8.GetBytes(text.ToString()));

        return Convert.ToHexString(hash)[..16];
    }

    // How sharply a merged avenue changes direction where two of its sections meet, measured
    // on section ends. Turns above 45 degrees count as real corners, not jaggedness.
    internal static void PrintAvenueKinks(IReadOnlyList<PreparedSegment> sections, string stage = "")
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
            var examples = new List<string>();

            foreach (var (node, list) in arms)
            {
                var avenue = list.Where(section => section.ConsolidatedAvenue && !section.OneWay).ToArray();

                if (avenue.Length != 2 || list.Count > 2 != junction)
                    continue;

                var into = Away(avenue[0], node);
                var outOf = Away(avenue[1], node);
                var turn = AvenueConsolidator.Angle(-into, outOf);

                if (turn > 45)
                    continue;

                turns.Add(turn);

                if (turn > 20 && examples.Count < 6)
                {
                    var point = avenue[0].StartNodeId == node ? avenue[0].Start : avenue[0].End;
                    examples.Add($"{turn:0}deg@({point.x:0},{point.z:0}) arms={list.Count} {avenue[0].Name}");
                }
            }

            turns.Sort();
            var median = turns.Count > 0 ? turns[turns.Count / 2] : 0;
            Console.WriteLine(
                $"  kinks{stage} {label}: {turns.Count:N0} joins; median {median:0.0} deg; over 5: {turns.Count(turn => turn > 5):N0}; over 10: {turns.Count(turn => turn > 10):N0}; over 20: {turns.Count(turn => turn > 20):N0}");

            if (examples.Count > 0)
                Console.WriteLine($"    e.g. {string.Join("; ", examples)}");
        }

        // Steps between two avenue pieces: a pair of opposite one-way sections, both under
        // 20 m, joining two nodes that each carry an avenue section.
        var steps = 0;
        var stepExamples = new List<string>();
        var seenPairs = new HashSet<(long, long)>();

        foreach (var section in sections.Where(
            section => section.Kind == NetworkKind.Road && section.OneWay && !section.ConsolidatedAvenue))
        {
            var key =
                (Math.Min(section.StartNodeId, section.EndNodeId), Math.Max(section.StartNodeId, section.EndNodeId));

            if (math.distance(section.Start.xz, section.End.xz) >= 20 || !seenPairs.Add(key))
                continue;

            var opposite =
                arms[section.EndNodeId].Any(
                    other => other.OneWay
                        && other.StartNodeId == section.EndNodeId
                        && other.EndNodeId == section.StartNodeId);

            bool AvenueAt(long node) => arms[node].Any(other => other.ConsolidatedAvenue && !other.OneWay);

            if (opposite && AvenueAt(section.StartNodeId) && AvenueAt(section.EndNodeId))
            {
                steps++;

                if (stepExamples.Count < 12)
                    stepExamples.Add($"way {section.WayId} {section.Name} {math.distance(section.Start.xz, section.End.xz):0} m at {section.Start.x:0},{section.Start.z:0}");
            }
        }

        Console.WriteLine($"  avenue steps{stage}: {steps:N0} pairs of short opposite one-way stubs between avenue ends");

        foreach (var example in stepExamples)
            Console.WriteLine("    step " + example);

        // Wiggles inside a curve: the tightest radius along each two-way avenue section
        // that has curve controls, from the cubic's curvature at nine interior points.
        var tight30 = 0;
        var tight60 = 0;
        var curved = 0;
        var tightExamples = new List<string>();

        // IEL_BEND_EXAMPLES=<n> lists up to n tight sections (default 10).
        var exampleLimit =
            int.TryParse(Environment.GetEnvironmentVariable("IEL_BEND_EXAMPLES"), out var limit) ? limit : 10;

        foreach (var section in sections.Where(
            section => section.Kind == NetworkKind.Road
                && section.ConsolidatedAvenue
                && !section.OneWay
                && section.Control1.HasValue
                && section.Control2.HasValue))
        {
            curved++;
            var p0 = section.Start.xz;
            var p1 = section.Control1!.Value.xz;
            var p2 = section.Control2!.Value.xz;
            var p3 = section.End.xz;
            var tightest = double.PositiveInfinity;

            for (var k = 1; k <= 9; k++)
            {
                var t = k / 10f;
                var u = 1 - t;
                var first = 3 * u * u * (p1 - p0) + 6 * u * t * (p2 - p1) + 3 * t * t * (p3 - p2);
                var second = 6 * u * (p2 - 2 * p1 + p0) + 6 * t * (p3 - 2 * p2 + p1);
                var speed = math.length(first);

                if (speed < 1e-3f)
                    continue;

                var curvature = Math.Abs(first.x * second.y - first.y * second.x) / (speed * speed * speed);

                if (curvature > 1e-9)
                    tightest = Math.Min(tightest, 1 / curvature);
            }

            if (tightest < 60)
                tight60++;

            if (tightest < 30)
            {
                tight30++;

                if (tightExamples.Count < exampleLimit)
                {
                    var arcLength = math.distance(p0, p3);
                    var startArms = arms.TryGetValue(section.StartNodeId, out var a) ? a.Count : 0;
                    var endArms = arms.TryGetValue(section.EndNodeId, out var b) ? b.Count : 0;
                    tightExamples.Add(
                        $"r{tightest:0}m len{arcLength:0} arms{startArms}/{endArms} @({p0.x:0},{p0.y:0})-({p3.x:0},{p3.y:0}) {section.Name}");
                }
            }
        }

        if (curved > 0)
            Console.WriteLine(
                $"  bends{stage}: {curved:N0} curved avenue sections; tightest radius under 60 m: {tight60:N0}; under 30 m: {tight30:N0}");

        foreach (var example in tightExamples)
            Console.WriteLine("    " + example);

        // The section's direction leaving the node: its curve tangent where it has one, so a
        // fitted curve is measured as drawn, else the chord.
        static float2 Away(PreparedSegment section, long node)
        {
            if (section.StartNodeId == node)
            {
                var tangent = (section.Control1 ?? section.End) - section.Start;

                return math.lengthsq(tangent.xz) > 1e-6f ? tangent.xz : (section.End - section.Start).xz;
            }

            var reverse = (section.Control2 ?? section.Start) - section.End;

            return math.lengthsq(reverse.xz) > 1e-6f ? reverse.xz : (section.Start - section.End).xz;
        }
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
