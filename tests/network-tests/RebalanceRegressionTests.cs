using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

// Reproduction of repeated redistribution extending a fit beyond its interval.
// Shared endpoints, outer anchors and tangents must survive every redistribution.
static class RebalanceRegressionTests
{
    // Rebalancing adjacent short sections twice used an untrimmed source piece
    // on the second fit, moving one end while its neighbour kept the old end.
    public static void Run()
    {
        float3[] points ={new(-1000,50,0),new(-873,50,27),new(-722,50,16),new(-645,50,35),new(-560,50,70),
            new(-493,50,52),new(-395,50,81),new(-327,50,91),new(-214,50,57),new(-55,50,43),new(57,50,58),
            new(141,50,49),
                new(281,50,49),
                new(436,50,73),
                new(549,50,51),
                new(675,50,45),
                new(828,50,12),
                new(985,50,16),
                new(1066,50,6)
                };

        var terrain = Enumerable.Repeat(50f, 33 * 33).ToArray();

        foreach (var reverse in new[] { false, true })
        {
            var path = reverse ? points.Reverse().ToArray() : points;
            var roads =
                Enumerable
                    .Range(0, path.Length - 1)
                    .Select(
                        i => VisualDesignTests.Road(
                            i + 1,
                            i + 1,
                            i + 2,
                            path[i],
                            path[i + 1]) with
                        { NativeWidth = 14 })
                    .ToArray();

            var output = VisualNetworkCompiler.Prepare(roads, terrain, 8000, new ImportLog(), default);
            Check(
                output.Count > 1
                    && output.SelectMany(segment => segment.SourceWayIds).Distinct().Count() == roads.Length,
                "Short-section fitting lost the continuous road or its source sections");

            foreach (var node in output
                .SelectMany(segment => new[] { (segment.StartNodeId, segment.Start), (segment.EndNodeId, segment.End) })
                .GroupBy(item => item.Item1))
                Check(node.All(item => math.distance(item.Item2, node.First().Item2) < .001f),
                    $"Rebalancing left the pieces at shared node {node.Key} with different endpoints");

            Check(
                output.All(
                    segment => NetworkDesignChecks.Finite(
                        new(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd))),
                "Short-section fitting emitted nonfinite geometry");

            foreach (var node in output
                .SelectMany(
                    segment => new[] {
                        (Id: segment.StartNodeId, Start: true, Section: segment),
                        (Id: segment.EndNodeId, Start: false, Section: segment)
                        })
                .GroupBy(item => item.Id))
            {
                if (node.Count() != 2)
                    continue;

                var headings =
                    node
                        .Select(
                            item => math.normalizesafe(
                                (item.Start
                                    ? item.Section.CourseControl1 - item.Section.CourseStart
                                    : item.Section.CourseControl2 - item.Section.CourseEnd).xz))
                        .ToArray();

                Check(
                    math.dot(headings[0], headings[1]) < -.9999f,
                    "Rebalancing introduced a tangent kink at a smooth internal join");
            }

            Check(
                output.All(segment => NetworkVerticalProfiles.MaximumGrade(segment) < 1e-5),
                "Rebalancing introduced a bump on flat terrain");

            Check(math.distance(output.Single(segment => segment.StartNodeId == 1).Start.xz, path[0].xz) < .001f
                && math.distance(output.Single(segment => segment.EndNodeId == path.Length).End.xz, path[^1].xz)
                    < .001f,
                "Short-section fitting moved an outer source anchor");
        }
    }
}
