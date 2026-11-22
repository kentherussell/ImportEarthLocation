using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Newtonsoft.Json;
using Unity.Mathematics;

static partial class VisualMetricsReplay
{
    public static void StreetSmoothing(string folder, string output, string? baselineAssembly = null)
    {
        var otherCapture = Load(folder);
        var log = new ImportLog();
        var compiled =
            VisualNetworkCompiler.Prepare(
                otherCapture.Source,
                otherCapture.Terrain,
                otherCapture.Width,
                log,
                default,
                otherCapture.Water);

        var sections =
            NativeDesignUnits.Prepare(
                StarterNetworkPolicy.WithoutSideStructures(compiled),
                otherCapture.Contracts,
                log,
                default,
                segment => VisualCompilerTable.For(segment).MinimumSegment);

        sections = NativeEndpointPolicy.PrepareLevelCrossings(sections, log);

        bool Street(PreparedSegment segment)
            => segment.Kind == NetworkKind.Road
                && !NativeAssetKeys.IsHighway(segment)
                && !segment.ProtectedLink
                && !segment.Bridge
                && !segment.Tunnel
                && !segment.Roundabout;

        float[] Point(float3 point) => new[] { point.x, point.y, point.z };

        var ports =
            sections
                .SelectMany(
                    (
                        segment,
                        i) => new[] {
                            (Index: i, Section: segment, Node: segment.StartNodeId, Start: true),
                            (Index: i, Section: segment, Node: segment.EndNodeId, Start: false)
                            })
            .GroupBy(item => new SourceNodeKey(item.Section.Kind, item.Node))
                .ToDictionary(group => group.Key, group => group.ToArray());

        var rows = sections.Select((segment, i) => (segment, i)).Where(item => Street(item.segment)).Select(item =>
        {
            var segment = item.segment;
            var length =
                NetworkDesignChecks.HorizontalArcLength(
                    new(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd));

            return new
            {
                index = item.i,
                segment.WayId,
                segment.StartNodeId,
                segment.EndNodeId,
                segment.Name,
                segment.Classification,
                length,
                minimum = VisualCompilerTable.For(segment).MinimumSegment,
                startDegree = ports[new(segment.Kind, segment.StartNodeId)].Length,
                endDegree = ports[new(segment.Kind, segment.EndNodeId)].Length,
                start = new[] { segment.CourseStart.x, segment.CourseStart.y, segment.CourseStart.z },
                end = new[] { segment.CourseEnd.x, segment.CourseEnd.y, segment.CourseEnd.z }
            };
        }).ToArray();

        var kinks =
            ports
                .Where(entry => entry.Value.Length == 2 && entry.Value.All(item => Street(item.Section)))
                .Select(entry =>
        {
            var arms = entry.Value.Select(item =>
            {
                var segment = item.Section;
                var d =
                    item.Start
                        ? segment.CourseControl1 - segment.CourseStart
                        : segment.CourseControl2 - segment.CourseEnd;

                return (
                    Direction: math.normalizesafe(d.xz),
                    Grade: d.y / Math.Max(.001, math.length(d.xz)),
                    Degenerate: math.length(d.xz) < 1e-6);
            }).ToArray();

            var first = entry.Value[0].Section;
            var second = entry.Value[1].Section;

            return new
            {
                node = entry.Key.Id,
                ways = entry.Value.Select(item => item.Section.WayId).ToArray(),
                // A zero horizontal tangent normalizes to zero and reads as 90°; it is its own
                // finding (report 121 §24).
                zeroTangent = arms.Any(arm => arm.Degenerate),
                // Both arms join the same two nodes, one reversed: an out-and-back pair, which a
                // 180° reading does not by itself make wrong or redundant (report 121 §24).
                reversedPair =
                    first.StartNodeId == second.EndNodeId && first.EndNodeId == second.StartNodeId,
                angle = math.degrees(Math.Acos(math.clamp(-math.dot(arms[0].Direction, arms[1].Direction), -1, 1))),
                gradeJump = Math.Abs(arms[0].Grade + arms[1].Grade),
                position = Point(
                    entry.Value[0].Start ? entry.Value[0].Section.CourseStart : entry.Value[0].Section.CourseEnd)
            };
        }).ToArray();

        var topology = new ImportLog { CaptureRecords = true };
        SourceTopologyObligations.Validate(
            SourceTopologyObligations.Capture(otherCapture.Source),
            sections,
            topology,
            default);

        string GeometryHash(IEnumerable<PreparedSegment> values)
        {
            var lines = values.Select(segment => JsonConvert.SerializeObject(new
            {
                segment.Kind,
                segment.WayId,
                segment.Classification,
                segment.Lanes,
                segment.OneWay,
                segment.Bridge,
                segment.Tunnel,
                segment.Layer,
                a = Point(segment.CourseStart),
                b = Point(segment.CourseControl1),
                c = Point(segment.CourseControl2),
                d = Point(segment.CourseEnd)
            })).OrderBy(text => text, StringComparer.Ordinal);

            return Convert.ToHexString(
                System.Security.Cryptography.SHA256.HashData(
                    System.Text.Encoding.UTF8.GetBytes(string.Join("\n", lines))));
        }

        string? baselineHighways = null;
        string? baselineRail = null;

        if (baselineAssembly != null)
        {
            // Load the frozen pre-edit harness in its own context. Its private
            // capture loader and compiler supply the actual baseline geometry;
            // no candidate code reconstructs or approximates that baseline.
            var assembly = System.Reflection.Assembly.LoadFile(Path.GetFullPath(baselineAssembly));
            var capture =
                assembly.GetType(
                    "VisualMetricsReplay")!.GetMethod(
                        "Load",
                        System.Reflection.BindingFlags.NonPublic
                            | System.Reflection.BindingFlags.Static)!.Invoke(null, new object[] { folder })!;

            object Get(string name) => capture.GetType().GetProperty(name)!.GetValue(capture)!;

            var baselineLog = Activator.CreateInstance(assembly.GetType("ImportEarthLocation.Core.ImportLog")!)!;
            var result =
                (System.Collections.IEnumerable)assembly.GetType(
                    "ImportEarthLocation.Processing.VisualNetworkCompiler")!.GetMethod("Prepare")!
                .Invoke(
                    null,
                    new[] {
                        Get("Source"),
                        Get("Terrain"),
                        Get("Scale"),
                        Get("Width"),
                        baselineLog,
                        (object)default(System.Threading.CancellationToken),
                        Get("Water"),
                        (object)float.NaN
                        })!;

            var converted = result.Cast<object>().Select(s =>
            {
                object Value(string name) => s.GetType().GetProperty(name)!.GetValue(s)!;

                float3 Position(string name)
                {
                    var p = Value(name);

                    float Field(string key) => Convert.ToSingle(p.GetType().GetField(key)!.GetValue(p));

                    return new(Field("x"), Field("y"), Field("z"));
                }

                return new PreparedSegment(
                    Convert.ToInt64(Value("WayId")),
                    0,
                    0,
                    Position("CourseStart"),
                    Position("CourseEnd"),
                    Enum.Parse<NetworkKind>(Value("Kind").ToString()!),
                        (string)Value("Classification"),
                        Convert.ToInt32(Value("Lanes")),
                    (bool)Value("OneWay"),
                        (bool)Value("Bridge"),
                        (bool)Value("Tunnel"),
                        Convert.ToInt32(Value("Layer")))
                { Control1 = Position("CourseControl1"), Control2 = Position("CourseControl2") };
            }).ToArray();

            baselineHighways =
                GeometryHash(
                    converted.Where(segment => segment.Kind == NetworkKind.Road && NativeAssetKeys.IsHighway(segment)));

            baselineRail = GeometryHash(converted.Where(segment => segment.Kind == NetworkKind.Rail));
        }

        var report = new
        {
            capture = Path.GetFileName(folder),
            streetSections = rows.Length,
            shortSections = rows.Count(r => r.length < r.minimum),
            artificialShortSections = rows.Count(r => r.length < r.minimum && (r.startDegree == 2 || r.endDegree == 2)),
            below30Metres = rows.Count(r => r.length < 30),
            plainNodeKinksOver1Degree = kinks.Count(k => k.angle > 1),
            plainNodeKinksOver5Degrees = kinks.Count(k => k.angle > 5),
            plainNodeGradeJumpsOver1Percent = kinks.Count(k => k.gradeJump > .01),
            plainNodeKinksWithZeroTangent = kinks.Count(k => k.zeroTangent),
            plainNodeKinksOver5DegreesReversedPairs = kinks.Count(k => k.angle > 5 && k.reversedPair),
            highwayGeometryHash = GeometryHash(
                sections.Where(segment => segment.Kind == NetworkKind.Road && NativeAssetKeys.IsHighway(segment))),
            railGeometryHash = GeometryHash(sections.Where(segment => segment.Kind == NetworkKind.Rail)),
            compiledHighwayGeometryHash = GeometryHash(
                compiled.Where(segment => segment.Kind == NetworkKind.Road && NativeAssetKeys.IsHighway(segment))),
            compiledRailGeometryHash = GeometryHash(compiled.Where(segment => segment.Kind == NetworkKind.Rail)),
            baselineHighwayGeometryHash = baselineHighways,
            baselineRailGeometryHash = baselineRail,
            missingPorts = topology.Metric("Missing mandatory selected-source ports"),
            unresolvedRoutes = topology.Metric("Unresolved selected-source reachability obligations"),
            rows,
            kinks = kinks.OrderByDescending(k => k.angle).ToArray(),
            diagnostics = log.Lines
                .Where(
                    text => text.StartsWith("VISUAL COMPILER horizontal", StringComparison.Ordinal)
                        || text.StartsWith("NATIVE UNITS", StringComparison.Ordinal))
                .ToArray()
        };

        File.WriteAllText(output, JsonConvert.SerializeObject(report, Formatting.Indented));
        Console.WriteLine(
            $"Street smoothing {report.capture}: sections={rows.Length}; short={report.shortSections}; artificialShort={report.artificialShortSections}; kinks>1deg={report.plainNodeKinksOver1Degree}; report={output}");
    }
}
