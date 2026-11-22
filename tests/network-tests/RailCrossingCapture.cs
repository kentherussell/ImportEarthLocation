using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Newtonsoft.Json;
using Unity.Mathematics;

static partial class VisualMetricsReplay
{
    public static void RailCrossings(string folder, string output)
    {
        var otherCapture = Load(folder);
        var log = new ImportLog();
        var expected = NativeEndpointPolicy.LevelCrossings(otherCapture.Source);
        var compiled =
            VisualNetworkCompiler.Prepare(
                otherCapture.Source,
                otherCapture.Terrain,
                otherCapture.Width,
                log,
                default,
                otherCapture.Water);

        var sections = NativeEndpointPolicy.PrepareLevelCrossings(compiled, log);
        var actual = NativeEndpointPolicy.LevelCrossings(sections);
        var rows =
            sections
                .SelectMany(
                    segment => new[] {
                        (Section: segment, Node: segment.StartNodeId, Point: segment.CourseStart),
                        (Section: segment, Node: segment.EndNodeId, Point: segment.CourseEnd)
                        })
            .Where(item => expected.Contains(item.Node)).GroupBy(item => item.Node).Select(group =>
            {
                var ports = group.ToArray();
                var road = ports.Where(item => item.Section.Kind == NetworkKind.Road).ToArray();
                var rail = ports.Where(item => item.Section.Kind == NetworkKind.Rail).ToArray();
                var horizontal =
                    road.Length == 0 || rail.Length == 0
                        ? double.PositiveInfinity
                        : road.Max(
                            firstItem => rail.Max(
                                secondItem => (double)math.distance(firstItem.Point.xz, secondItem.Point.xz)));

                var vertical =
                    road.Length == 0 || rail.Length == 0
                        ? double.PositiveInfinity
                        : road.Max(
                            firstItem => rail.Max(
                                secondItem => (double)Math.Abs(firstItem.Point.y - secondItem.Point.y)));

                var grades = ports.Select(item =>
                {
                    var segment = item.Section;
                    var d =
                        item.Node == segment.StartNodeId
                            ? segment.CourseControl1 - segment.CourseStart
                            : segment.CourseEnd - segment.CourseControl2;

                    return Math.Abs(d.y) / Math.Max(.001, math.length(d.xz));
                }).ToArray();

                return new
                {
                    node = group.Key,
                    roadPorts = road.Length,
                    railPorts = rail.Length,
                    horizontalGap = horizontal,
                    verticalGap = vertical,
                    maxEntryGrade = grades.Max(),
                    sharedNativeOwner = ports
                        .Select(item => NativeEndpointPolicy.Identity(item.Section, item.Node))
                        .Distinct()
                        .Count()
                        == 1,
                    structures = ports.Count(item => NetworkStructure.IsStructure(item.Section))
                };
            }).ToArray();

        File.WriteAllText(output, JsonConvert.SerializeObject(new
        {
            capture = Path.GetFileName(folder),
            sourceCrossings = expected.Count,
            finalCrossings = actual.Count,
            missingNodes = expected.Where(nodeId => !rows.Any(r => r.node == nodeId)).ToArray(),
            rows,
            metrics = log.Lines
                .Where(
                    text => text.Contains("crossing", StringComparison.OrdinalIgnoreCase)
                        || text.Contains("junction height mismatch", StringComparison.Ordinal))
                .ToArray()
        }, Formatting.Indented));

        Console.WriteLine(
            $"Rail crossings: source={expected.Count}; final={actual.Count}; exactSharedNativeOwners={rows.Count(r => r.sharedNativeOwner)}; report={output}");

        // Each crossing whose arms came apart: where every arm's end is in the source and
        // after design, to find the step that moved it.
        foreach (var row in rows.Where(item => item.horizontalGap > .001))
        {
            Console.WriteLine($"  crossing {row.node}: horizontal gap {row.horizontalGap:F3} m");

            foreach (var (stage, list) in new[] { ("source", otherCapture.Source), ("design", sections) })
                foreach (var segment in list.Where(item => item.StartNodeId == row.node || item.EndNodeId == row.node))
                {
                    var atStart = segment.StartNodeId == row.node;
                    var point = atStart ? segment.CourseStart : segment.CourseEnd;
                    Console.WriteLine(
                        FormattableString.Invariant(
                            $"    {stage,-6} {segment.Kind} way={segment.WayId} {segment.Classification} lanes={segment.Lanes} oneWay={segment.OneWay} end={(atStart ? "start" : "end")} at={point.x:F2},{point.z:F2}"));
                }
        }
    }
}
