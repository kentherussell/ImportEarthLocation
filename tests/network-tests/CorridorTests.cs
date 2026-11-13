using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

internal static class CorridorTests
{
    private static PreparedSegment MakeSegment(
        long way,
        long a,
        long b,
        float x1,
        float z1,
        float x2,
        float z2,
        NetworkKind kind = NetworkKind.Rail) =>
        new(
            way,
            a,
            b,
            new float3(x1, 0, z1),
            new float3(x2, 0, z2),
            kind,
            kind == NetworkKind.Rail ? "rail" : "primary",
            kind == NetworkKind.Rail ? 1 : 2,
            false,
            false,
            false,
            0)
        { Name = "Route", SourceGeometry = new[] { new float3(x1, 0, z1), new float3(x2, 0, z2) } };

    private static readonly System.Text.Json.JsonSerializerOptions SnapshotOptions =
        new()
        {
            IncludeFields = true,
            NumberHandling =
                System.Text.Json.Serialization.JsonNumberHandling.AllowNamedFloatingPointLiterals
                    | System.Text.Json.Serialization.JsonNumberHandling.AllowReadingFromString,
            Converters = { new Float3Converter() }
        };

    private sealed class Float3Converter : System.Text.Json.Serialization.JsonConverter<float3>
    {
        public override float3 Read(
            ref System.Text.Json.Utf8JsonReader reader,
            Type type,
            System.Text.Json.JsonSerializerOptions options)
        {
            if (reader.TokenType == System.Text.Json.JsonTokenType.StartArray)
            {
                reader.Read();
                var x = reader.GetSingle();
                reader.Read();
                var y = reader.GetSingle();
                reader.Read();
                var z = reader.GetSingle();
                reader.Read();

                return new(x, y, z);
            }

            if (reader.TokenType != System.Text.Json.JsonTokenType.StartObject)
                throw new System.Text.Json.JsonException("Expected diagnostic vector.");

            var point = new float3();
            var components = 0;

            while (reader.Read() && reader.TokenType != System.Text.Json.JsonTokenType.EndObject)
            {
                var key = reader.GetString();
                reader.Read();
                var value = reader.GetSingle();

                if (key == "x")
                    point.x = value;
                else if (key == "y")
                    point.y = value;
                else if (key == "z")
                    point.z = value;
                else
                    throw new System.Text.Json.JsonException("Unexpected vector member.");

                components++;
            }

            if (components != 3)
                throw new System.Text.Json.JsonException("Incomplete diagnostic vector.");

            return point;
        }

        public override void Write(
            System.Text.Json.Utf8JsonWriter writer,
            float3 value,
            System.Text.Json.JsonSerializerOptions options)
        {
            writer.WriteStartArray();
            writer.WriteNumberValue(value.x);
            writer.WriteNumberValue(value.y);
            writer.WriteNumberValue(value.z);
            writer.WriteEndArray();
        }
    }

    public static void AuditDiagnostic(string directory, string stage)
    {
        if (stage != "before-cover" && stage != "native-before-cover" && stage != "native-plan")
            throw new ArgumentException("Use a recorded geometry stage.");

        using var metadata =
            System.Text.Json.JsonDocument.Parse(File.ReadAllText(Path.Combine(directory, stage + "-geometry.json")));

        var root = metadata.RootElement;

        if (root.GetProperty("network").GetProperty("Truncated").GetBoolean())
            throw new InvalidOperationException(
                "Network artifact is truncated; do not treat a partial replay as full coverage.");

        var options = new System.Text.Json.JsonSerializerOptions(SnapshotOptions);
        options.Converters.Add(new System.Text.Json.Serialization.JsonStringEnumConverter());
        var sections = new List<PreparedSegment>();

        foreach (var line in File.ReadLines(Path.Combine(directory, stage + "-network.ndjson")))
        {
            using var record = System.Text.Json.JsonDocument.Parse(line);

            if (record.RootElement.GetProperty("SourcePointsTruncated").GetBoolean())
                throw new InvalidOperationException(
                    "Source geometry is truncated; only a limited regional audit is possible.");

            sections.Add(System.Text.Json.JsonSerializer.Deserialize<PreparedSegment>(line, options)!);
        }

        if (sections.Count != root.GetProperty("sectionsRequested").GetInt32())
            throw new InvalidOperationException("Recorded section count differs from geometry metadata.");

        var bytes = File.ReadAllBytes(Path.Combine(directory, stage + "-heightmap.u16"));

        if (bytes.Length != root.GetProperty("heightSamples").GetInt32() * 2)
            throw new InvalidOperationException("Recorded terrain size differs from metadata.");

        var encoded = new ushort[bytes.Length / 2];

        for (var i = 0; i < encoded.Length; i++)
            encoded[i] = (ushort)(bytes[i * 2] | bytes[i * 2 + 1] << 8);

        var scale = root.GetProperty("heightScaleMeters").GetSingle();
        var heights = TerrainEncoding.Decode(encoded, scale);
        var width = root.GetProperty("mapWidthMeters").GetDouble();
        var log = new ImportLog();
        NetworkGeometrySafety.ValidateEnvelope(sections, width, scale, default);
        NetworkGeometrySafety.Validate(sections, log, default, allowQualityWarnings: true);

        try
        {
            NetworkGeometrySafety.ValidateTunnelCover(sections, heights, scale, width, default);
            Console.WriteLine("Diagnostic actual-raster roof audit passed.");
        }
        finally
        {
            foreach (var line in log.Lines)
                Console.WriteLine(line);
        }

        Console.WriteLine(
            $"Diagnostic replay: stage={stage}, sections={sections.Count}, terrain={heights.Length}; original raster, no download, no native game mutation. This is not rendered acceptance.");
    }

    private static void DiagnosticRoundtrip()
    {
        var root = Path.Combine(Path.GetTempPath(), "iel-diagnostic-replay-" + Guid.NewGuid().ToString("N"));

        try
        {
            var archive =
                new RunArchive(root, new { latitude = 0, longitude = 0 }, typeof(CorridorTests).Assembly.Location);

            var tunnel =
                MakeSegment(
                    77,
                    701,
                    702,
                    -100,
                    0,
                    100,
                    0) with
                {
                    Tunnel = true,
                    Start = new float3(-100, 15, 0),
                    End = new float3(100, 15, 0),
                    NativeWidth = 4,
                    SourceGeometry = new[] { new float3(-100, 15, 0), new float3(100, 15, 0) }
                };

            var heights = Enumerable.Repeat(50f, 16 * 16).ToArray();
            DiagnosticArtifacts.CaptureGeometry(archive, "native-plan", new[] { tunnel }, heights, 100, 1000);
            AuditDiagnostic(archive.DirectoryPath, "native-plan");
            DiagnosticArtifacts.CaptureGeometry(
                archive,
                "before-cover",
                new[] { tunnel },
                new float[16 * 16],
                100,
                1000);

            var rejected = false;

            try
            {
                AuditDiagnostic(archive.DirectoryPath, "before-cover");
            }
            catch (InvalidOperationException exception)
            {
                rejected = exception.Message.Contains("Strict offline tunnel cover audit failed for way 77");
            }

            Check(
                rejected,
                "Diagnostic replay substituted flat terrain or missed an exposed roof in the original raster");

            var advisory = new ImportLog();
            NetworkGeometrySafety.ValidateTunnelCover(
                new[] { tunnel },
                new float[16 * 16],
                100,
                1000,
                default,
                advisory);

            Check(
                advisory.Metric("Final exposed tunnel observations") > 0,
                "Production tunnel-quality path neither reported the exposure nor retained advisory inspectability");

            Console.WriteLine(
                "PASS: diagnostic original-raster replay preserves controls/offsets and distinguishes covered from exposed tunnels without a download or game mutation.");
        }
        finally
        {
            if (Directory.Exists(root))
                Directory.Delete(root, true);
        }
    }

    public static void Run()
    {
        DiagnosticRoundtrip();
        RoadDesignChecks();
        RailSwitchChecks();
        var triangle =
            new[] {
                MakeSegment(901, 9001, 9003, 0, 0, 80, 0),
                MakeSegment(902, 9001, 9002, 0, 0, 55, 1),
                MakeSegment(903, 9002, 9003, 55, 1, 80, 0),
                MakeSegment(903, 9003, 9004, 80, 0, 200, 0)
                };

        var triangleResult = RailCorridorConsolidator.Prepare(triangle, new ImportLog(), default);
        NetworkContinuity.Validate(
            triangleResult,
            triangleResult,
            new ImportLog(),
            default,
            allowQualityWarnings: true);

        Check(
            triangleResult.Any(segment => segment.EndNodeId == 9004),
            "Consolidating a short redundant switch triangle deleted its unique continuation");

        var log = new ImportLog();
        var main = MakeSegment(1, 1, 2, 0, 0, 300, 0);
        var parallel = new[] { MakeSegment(2, 3, 4, 0, 6, 150, 6), MakeSegment(2, 4, 5, 150, 6, 300, 6) };
        var branch = MakeSegment(3, 4, 6, 150, 6, 150, 100);
        var source = new[] { main, parallel[0], parallel[1], branch };
        var result = RailCorridorConsolidator.Prepare(source, log, default);
        Check(
            result.Count == 3 && result.Count(segment => segment.WayId == 1) == 2,
            "Parallel rails were not folded into a single split axis");

        var transferred = result.Single(segment => segment.WayId == 3);
        Check(
            transferred.Start.z == 0
                && result.Count(
                    segment => segment.StartNodeId == transferred.StartNodeId
                        || segment.EndNodeId == transferred.StartNodeId)
                    == 3,
            "Rail branch lost its native junction");

        var reverse =
            RailCorridorConsolidator.Prepare(new[] { main, MakeSegment(2, 5, 3, 300, 6, 0, 6) }, log, default);

        Check(reverse.Count == 1, "Reverse-oriented parallel rail retained");
        var dividedAxis =
            new[] {
                MakeSegment(1, 1, 2, 0, 0, 100, 0),
                MakeSegment(1, 2, 7, 100, 0, 200, 0),
                MakeSegment(1, 7, 6, 200, 0, 300, 0),
                MakeSegment(2, 3, 5, 300, 6, 0, 6)
                };

        Check(
            RailCorridorConsolidator.Prepare(dividedAxis, log, default).Count == 3,
            "Different raw vertex spacing prevented rail consolidation");

        var partial =
            RailCorridorConsolidator.Prepare(
                new[] {
                    main,
                    MakeSegment(2, 3, 4, 0, 6, 200, 6) with { RailService = "siding" },
                    MakeSegment(2, 4, 5, 200, 6, 400, 6) with { RailService = "siding" }
                    },
                log,
                default);

        Check(
            partial.Any(segment => segment.WayId == 2 && segment.End.x == 400)
                && partial.Any(segment => segment.SourceWayIds.Contains(2))
                && partial
                .Where(segment => segment.WayId == 2)
                .Any(
                    tail => partial.Any(
                        axis => axis.WayId == 1
                            && (axis.StartNodeId == tail.StartNodeId || axis.EndNodeId == tail.StartNodeId))),
            "Siding consolidation lost its unique tail, source provenance, or exact attachment to the retained axis");

        var dense = new List<PreparedSegment> { main };

        for (var i = 0; i < 8; i++)
            dense.Add(
                MakeSegment(
                    10 + i,
                    100 + i * 2,
                    101 + i * 2,
                    0,
                    2 + i * 2,
                    300,
                    2 + i * 2) with
                { RailService = "yard" });

        var denseResult = RailCorridorConsolidator.Prepare(dense, log, default);
        Check(
            denseResult.Count == 1
                && denseResult[0].Lanes == 2
                && denseResult[0].PhysicalTracks == 2
                && !denseResult[0].OneWay
                && denseResult[0].SourceWayIds
                .OrderBy(id => id)
                .SequenceEqual(dense.Select(segment => segment.WayId).OrderBy(id => id)),
                "Starter rail corridor must fold the parallel yard ladder without losing source provenance or creating extra physical axes");

        Check(
            RailCorridorConsolidator.Prepare(
                new[] { main, MakeSegment(2, 3, 4, 0, 6, 300, 6) with { Layer = 1 } },
                log,
                default).Count
                == 2,
            "Different rail levels were destructively consolidated");

        foreach (var guarded in new[] {
            MakeSegment(
                2,
                3,
                4,
                0,
                RailCorridorConsolidator.CorridorDistance + 5,
                300,
                RailCorridorConsolidator.CorridorDistance + 5),
            MakeSegment(2, 3, 4, 0, 6, 300, 6) with { Bridge = true },
            MakeSegment(2, 3, 4, 0, 6, 300, 6) with { Tunnel = true },
            MakeSegment(2, 3, 4, 150, -100, 150, 100)
            })
            Check(
                RailCorridorConsolidator.Prepare(new[] { main, guarded }, log, default).Count == 2,
                "Rail crossing/level/structure guard failed");

        var bridge = MakeSegment(3, 5, 6, 300, 6, 400, 6) with { Bridge = true, Layer = 1 };
        var protectedResult =
            RailCorridorConsolidator.Prepare(new[] { main, parallel[0], parallel[1], bridge }, log, default);

        var portal = protectedResult.Single(segment => segment.WayId == 3);
        Check(
            portal.Start.z == 0
                && protectedResult.Any(segment => segment.WayId == 1 && segment.EndNodeId == portal.StartNodeId),
            "Rail structural transition disconnected");

        foreach (var tunnel in new[] { false, true })
        {
            var structural =
                RailCorridorConsolidator.Prepare(
                    new[] {
                        main with { Bridge = !tunnel, Tunnel = tunnel },
                        MakeSegment(2, 3, 4, 0, 6, 300, 6) with { Bridge = !tunnel, Tunnel = tunnel }
                        },
                    log,
                    default);

            Check(structural.Count == 1 && structural[0].Lanes == 2, "Parallel structural rail was not consolidated");
        }

        Check(
            RailCorridorConsolidator
                .Prepare(source, log, default)
                .Select(segment => (segment.WayId, segment.StartNodeId, segment.EndNodeId, segment.Start, segment.End))
                .SequenceEqual(
                    result.Select(
                        segment => (
                            segment.WayId,
                            segment.StartNodeId,
                            segment.EndNodeId,
                            segment.Start,
                            segment.End))),
            "Rail consolidation is not deterministic");

        NetworkNormalizer.Validate(NetworkNormalizer.Normalize(result, log, default, 1000), 1000, default);

        using (var cancellation = new CancellationTokenSource())
        {
            cancellation.Cancel();

            try
            {
                RailCorridorConsolidator.Prepare(source, log, cancellation.Token);
                throw new Exception("Rail cancellation ignored");
            }
            catch (OperationCanceledException) { }
        }

        var avenue =
            new[] {
                MakeSegment(50, -100, -101, 0, 0, 25, .4f, NetworkKind.Road),
                MakeSegment(50, -101, -102, 25, .4f, 50, 0, NetworkKind.Road),
                MakeSegment(50, -102, -103, 50, 0, 75, .4f, NetworkKind.Road)
                };

        Check(
            NetworkNormalizer.Normalize(avenue, log, default, 1000).Count < avenue.Length,
            "Synthetic avenue nodes still block smoothing");

        var sharp =
            new[] {
                MakeSegment(51, -110, -111, 0, 0, 30, 0, NetworkKind.Road),
                MakeSegment(51, -111, -112, 30, 0, 30, 30, NetworkKind.Road)
                };

        Check(
            NetworkNormalizer.Normalize(sharp, log, default, 1000).Count == 2,
            "Intentional corner rounded across topology");

        var divided =
            new[] {
                MakeSegment(80, 300, 301, 0, 0, 100, 0, NetworkKind.Road) with { OneWay = true },
                MakeSegment(80, 301, 302, 100, 0, 200, 0, NetworkKind.Road) with { OneWay = true },
                MakeSegment(80, 302, 303, 200, 0, 300, 0, NetworkKind.Road) with { OneWay = true },
                MakeSegment(81, 304, 305, 300, 10, 200, 10, NetworkKind.Road) with { OneWay = true },
                MakeSegment(81, 305, 306, 200, 10, 100, 10, NetworkKind.Road) with { OneWay = true },
                MakeSegment(81, 306, 307, 100, 10, 0, 10, NetworkKind.Road) with { OneWay = true },
                MakeSegment(82, 299, 300, -100, 0, 0, 0, NetworkKind.Road) with { Bridge = true },
                MakeSegment(83, 307, 308, 0, 10, -100, 10, NetworkKind.Road) with { Bridge = true }
                };

        var dividedResult = AvenueConsolidator.Prepare(divided, log, default);
        var entryBridge = dividedResult.Single(segment => segment.WayId == 82);
        var exitBridge = dividedResult.Single(segment => segment.WayId == 83);

        bool HasAvenuePort(long node, float3 point) => dividedResult.Any(segment => segment.ConsolidatedAvenue
            && (segment.StartNodeId == node && math.all(segment.Start == point)
                || segment.EndNodeId == node && math.all(segment.End == point)));

        Check(dividedResult.Any(segment => segment.ConsolidatedAvenue) && entryBridge.Bridge && exitBridge.Bridge
            && math.all(entryBridge.Start == divided[6].Start) && math.all(exitBridge.End == divided[7].End)
                && HasAvenuePort(entryBridge.EndNodeId, entryBridge.End)
                && HasAvenuePort(exitBridge.StartNodeId, exitBridge.Start),
            "Interior avenue simplification lost a structural approach, its distant endpoint, or the exact shared portal connection");

        Check(log.Lines.Any(text => text.Contains("RAIL TOTAL")), "Corridor diagnostics missing");

        IndexedGeometryChecks();
        Console.WriteLine(
            "PASS: bounded two-track mainline pairing, independent yards/sidings/levels, reversed tracks, transferred branches, unique tails, source-evidenced switches, determinism, cancellation, synthetic-node smoothing and sharp corners.");
    }

    private static void RailSwitchChecks()
    {
        var axis = MakeSegment(901, 901, 902, -100, 0, 100, 0);
        var spur = MakeSegment(902, 903, 904, 0, 60, 0, 5);
        var continuation = MakeSegment(903, 904, 905, 0, 5, 60, 45);
        var unchanged = RailJunctionRepair.Prepare(new[] { axis, spur, continuation }, new ImportLog(), default);
        Check(
            unchanged.Count == 3 && unchanged[1].EndNodeId == 904 && unchanged[1].End.Equals(spur.End),
            "Nearby rails were joined without source-node switch evidence");

        var incoming = MakeSegment(901, 901, 904, -100, 0, 0, 0);
        var outgoing = MakeSegment(904, 904, 906, 0, 0, 100, 0);
        var connected =
            RailJunctionRepair.Prepare(new[] { incoming, outgoing, spur, continuation }, new ImportLog(), default);

        Check(
            connected.Count == 4
                && connected.Count(segment => segment.StartNodeId == 904 || segment.EndNodeId == 904) == 4,
            "Mapped switch lost an incident edge");

        Check(
            connected.Where(segment => segment.StartNodeId == 904).All(segment => segment.Start.Equals(float3.zero))
                && connected.Where(segment => segment.EndNodeId == 904).All(segment => segment.End.Equals(float3.zero)),
            "Mapped switch was not canonicalized to one native anchor");

        Check(spur.End.z == 5 && axis.Start.x == -100, "Rail repair mutated its input");
    }

    private static void RoadDesignChecks()
    {
        var log = new ImportLog();

        PreparedSegment Road(long way, long from, long to, float x1, float z1, float x2, float z2, int lanes = 3)
            => MakeSegment(
                way,
                from,
                to,
                x1,
                z1,
                x2,
                z2,
                NetworkKind.Road) with
            { Classification = "motorway", OneWay = true, Lanes = lanes, Reference = "TEST" };

        var laneRun =
            new[] {
                Road(701, 1, 2, -300, 0, -50, 0),
                Road(702, 2, 3, -50, 0, 50, 0, 2),
                Road(703, 3, 4, 50, 0, 300, 0)
                };

        var normalized = NetworkNormalizer.Normalize(laneRun, log, default, 1000);
        Check(
            normalized.All(segment => segment.Lanes == 3)
                && normalized.First().StartNodeId == 1
                && normalized.Last().EndNodeId == 4,
            "Short 3-2-3 mainline lane blip not removed or unique terminal topology changed");

        var explicitLanes =
            NetworkNormalizer.Normalize(
                laneRun
                    .Select(segment => segment.WayId == 702 ? segment with { ExplicitLanes = true } : segment)
                    .ToArray(),
                log,
                default,
                1000);

        Check(
            explicitLanes.Single(segment => segment.WayId == 702).Lanes == 2,
            "Inferred lane smoothing overwrote explicit source lane evidence");

        var narrowBridge =
            NetworkNormalizer.Normalize(
                laneRun
                    .Select(segment => segment.WayId == 702 ? segment with { Bridge = true, Layer = 1 } : segment)
                    .ToArray(),
                log,
                default,
                1000);

        Check(
            narrowBridge.Single(segment => segment.WayId == 702).Lanes == 2,
            "Lane smoothing erased a real bridge cross-section boundary");

        var junction = Road(704, 2, 5, -50, 0, -50, 100, 1) with { Classification = "motorway_link" };
        Check(
            NetworkNormalizer.Normalize(
                laneRun.Concat(new[] { junction }).ToArray(),
                log,
                default,
                1000).Single(segment => segment.WayId == 702).Lanes
                == 2,
            "Meaningful merge lane change was erased");

        var merge =
            new[] {
                Road(720, 30, 31, -300, 0, 0, 0),
                Road(721, 31, 32, 0, 0, 250, 0),
                Road(722, 32, 33, 250, 0, 450, 0),
                Road(723, 34, 31, -120, -80, 0, 0, 1) with { Classification = "motorway_link" }
                };

        var merging = NetworkNormalizer.Normalize(merge, log, default, 1000);
        Check(
            merging.Where(segment => segment.Classification == "motorway").All(segment => segment.Lanes == 3)
                && merging.Any(segment => segment.StartNodeId == 31),
            "Synthetic entrance widening introduced a lane-count blip or erased the merge junction");

        var avenueRun =
            new[] {
                Road(730, 50, 51, -400, 200, -50, 200, 6),
                Road(731, 51, 52, -50, 200, 0, 200, 4),
                Road(732, 52, 53, 0, 200, 50, 200, 4),
                Road(733, 53, 54, 50, 200, 400, 200, 6)
                }
            .Select(segment => segment with { Classification = "primary", OneWay = false }).ToArray();

        Check(
            NetworkNormalizer.Normalize(avenueRun, log, default, 1000).All(segment => segment.Lanes == 6),
            "Multi-segment ordinary-road width fluctuation was not normalized");

        Console.WriteLine(
            "ROAD DESIGN FIXTURE shortLaneBlipsNormalized=true; mergeLaneChangesPreserved=true; offlineSynthetic=true");
    }

    private static void IndexedGeometryChecks()
    {
        bool Reference(PreparedSegment segment)
        {
            var tolerance = NetworkNormalizer.Tolerance(segment.Kind);
            var vectorA = segment.Start;
            var vectorB = segment.Control1!.Value;
            var c = segment.Control2!.Value;
            var d = segment.End;
            var bound = math.distance(vectorA, vectorB) + math.distance(vectorB, c) + math.distance(c, d);
            var count = Math.Min(2048, Math.Max(32, (int)Math.Ceiling(6 * bound / tolerance)));
            var samples =
                Enumerable
                    .Range(0, count + 1)
                    .Select(i => NetworkNormalizer.Evaluate(segment, i / (float)count))
                    .ToArray();

            var allowance = tolerance - 3 * bound / (2 * count);

            float Distance(float3 point, IReadOnlyList<float3> line)
            {
                var result = float.MaxValue;

                for (var i = 1; i < line.Count; i++)
                    result = Math.Min(result, NetworkNormalizer.DistanceToSegment(point, line[i - 1], line[i]));

                return result;
            }

            if (allowance <= 0 || samples.Any(point => Distance(point, segment.SourceGeometry) > allowance))
                return false;

            for (var i = 1; i < segment.SourceGeometry.Count; i++)
            {
                var first = segment.SourceGeometry[i - 1];
                var last = segment.SourceGeometry[i];
                var steps = Math.Max(1, (int)Math.Ceiling(math.distance(first, last) / (tolerance / 2)));

                for (var j = 0; j <= steps; j++)
                    if (Distance(math.lerp(first, last, j / (float)steps), samples) > tolerance * .75f)
                        return false;
            }

            return true;
        }

        foreach (var kind in new[] { NetworkKind.Road, NetworkKind.Rail })
            foreach (var shift in new[] { 0f, .3f, 2f, 8f })
            {
                var curve =
                    MakeSegment(
                        500,
                        1,
                        2,
                        0,
                        0,
                        150,
                        35,
                        kind) with
                    { Control1 = new float3(50, 0, 0), Control2 = new float3(100, 0, 35) };

                var points = Enumerable.Range(0, 65).Select(i => NetworkNormalizer.Evaluate(curve, i / 64f)).ToArray();
                var candidate =
                    curve with
                    {
                        SourceGeometry = points,
                        Control1 = curve.Control1 + new float3(0, 0, shift),
                        Control2 = curve.Control2 + new float3(0, 0, shift)
                    };

                Check(
                    NetworkNormalizer.WithinTolerance(candidate) == Reference(candidate),
                    "Indexed nearest-segment validation disagrees with the original exhaustive test");
            }
    }
}
