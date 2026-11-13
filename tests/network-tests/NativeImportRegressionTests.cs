using ImportEarthLocation.Core;
using Unity.Mathematics;

// Shared production decisions. These do not execute the game's network generator.
internal static class NativeImportRegressionTests
{
    private static PreparedSegment Line(long way, long a, long b, float x, float end) =>
        new(way, a, b, new(x, 0, 0), new(end, 0, 0), NetworkKind.Road, "secondary", 2, false, false, false, 0)
        {
            Name = "Route",
            SourceWayIds = new[] { way },
            SourceGeometry = new[] { new float3(x, 0, 0), new float3(end, 0, 0) }
        };

    public static void Run()
    {
        var firstSegment = Line(10, 1, 2, 0, 40);
        var secondSegment = Line(11, 2, 3, 40, 41.6f) with { ExplicitLanes = true };
        var branch =
            Line(12, 2, 4, 40, 40) with { End = new float3(40, 0, 30), SourceGeometry = Array.Empty<float3>() };

        var disconnected = Line(99, 90, 91, 900, 950);
        var otherSegment = Line(13, 3, 5, 41.6f, 80);
        var plan = new NativeSubmissionPlan(new[] { firstSegment, disconnected, otherSegment, secondSegment, branch });
        Check(
            plan.Sections.Count == 5
                && plan.Sections[0] == firstSegment
                && plan.Sections[1] == secondSegment
                && plan.Sections[2] == branch,
            "Connected submission order lost identity frontier");

        Check(
            plan.Sections
                .ToHashSet()
                .SetEquals(new[] { firstSegment, secondSegment, otherSegment, branch, disconnected }),
            "Submission scheduling removed/changed geometry");

        Check(
            plan.GatherPendingJunction(1, 1, 128, new[] { "Road:2" }) == 2,
            "Recovery did not gather pending incident arms");

        Check(
            plan.Sections[0] == firstSegment && plan.Sections.Count == 5,
            "Recovery replayed committed prefix or dropped a section");

        Check(
            plan.GatherPendingJunction(-1, 1, 128, new[] { "Road:2" }) == 0
                && plan.GatherPendingJunction(1, 1, 1, new[] { "Road:2" }) == 0,
            "Invalid or oversized recovery star accepted");

        Console.WriteLine(
            "PASS: connected submission order, unchanged geometry and source-identity pending-only recovery.");
    }
}
