using System;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

public static class NetworkVerticalProfiles
{
    public const float MinimumBridgeClearance = 5f;
    public const float AbsoluteRoadGrade = .348f;

    // Practical import limits, not claims about the maximum grade of real roads.
    public static float GradeLimit(PreparedSegment segment)
        => segment.SourceSpeedKph >= 80 && !segment.ProtectedLink ? .09f : segment.ProtectedLink ? .13f :
        segment.Classification == "motorway" ? .09f : segment.Classification == "trunk" ? .11f : .16f;

    public static float DesiredGrade(PreparedSegment segment)
        => segment.Kind == NetworkKind.Rail ? .02f : segment.ProtectedLink ? .06f :
        segment.Classification == "motorway" || segment.Classification == "trunk" || segment.SourceSpeedKph >= 80
            ? .035f
            :
        segment.Classification == "primary" || segment.Classification == "secondary" ? .06f : .08f;

    // Depth past which a road or railway is a bridge or tunnel rather than an earthwork,
    // for every class: walls and floating roads in game rise sharply past 12 m on the
    // 28 maps of 8 November (report 112, 10.2). Used when the structure-depth switch is on.
    public const float StructureDepth = 12f;

    // Track follows the ground only at long wavelengths, so it runs in deeper
    // cuttings and on higher banks than a street: at 6 m Marseille's smoothed
    // lines missed the stamped ground by up to 12 m where the budget ran out.
    public static float CutFillBudget(PreparedSegment segment)
        => DesignSwitches.On(DesignSwitches.StructureDepth) ? StructureDepth :
        segment.Kind == NetworkKind.Rail ? 12f :
        segment.Classification == "motorway" || segment.Classification == "trunk" || segment.ProtectedLink ? 20f : 10f;

    public static float MaximumGrade(PreparedSegment segment)
    {
        var vectorA = segment.CourseStart;
        var vectorB = segment.CourseControl1;
        var c = segment.CourseControl2;
        var d = segment.CourseEnd;
        var maximum = 0f;

        for (var i = 0; i <= 256; i++)
        {
            var t = i / 256f;
            var u = 1 - t;
            var derivative = 3 * u * u * (vectorB - vectorA) + 6 * u * t * (c - vectorB) + 3 * t * t * (d - c);
            var horizontal = math.length(derivative.xz);
            var grade =
                horizontal < .00001f
                    ? (Math.Abs(derivative.y) < .00001f ? 0 : float.PositiveInfinity)
                    : Math.Abs(derivative.y) / horizontal;

            maximum = Math.Max(maximum, grade);
        }

        return maximum;
    }

    // Quadratic Bezier derivative controls bound every derivative in an interval.
    // Subdivision tightens the bounds; an uncertain interval is rejected, never
    // accepted on the strength of discrete samples that might miss a sharp peak.
    public static bool WithinGradeCeiling(PreparedSegment segment, float ceiling = AbsoluteRoadGrade)
    {
        var vectorA = segment.CourseStart;
        var vectorB = segment.CourseControl1;
        var c = segment.CourseControl2;
        var d = segment.CourseEnd;

        return Bound(3 * (vectorB - vectorA), 3 * (c - vectorB), 3 * (d - c), 0);

        bool Bound(float3 point, float3 otherPoint, float3 r, int depth)
        {
            if (!math.all(math.isfinite(point)) || !math.all(math.isfinite(otherPoint)) || !math.all(math.isfinite(r)))
                return false;

            var min = math.min(point, math.min(otherPoint, r));
            var max = math.max(point, math.max(otherPoint, r));
            var vertical = Math.Max(Math.Abs(min.y), Math.Abs(max.y));

            if (vertical < .000001f)
                return true;

            var dx = min.x > 0 ? min.x : max.x < 0 ? -max.x : 0;
            var dz = min.z > 0 ? min.z : max.z < 0 ? -max.z : 0;
            var horizontal = (float)Math.Sqrt(dx * dx + dz * dz);

            if (vertical <= ceiling * horizontal + .000001f)
                return true;

            var pq = (point + otherPoint) * .5f;
            var qr = (otherPoint + r) * .5f;
            var middle = (pq + qr) * .5f;

            foreach (var derivative in new[] { point, middle, r })
                if (Math.Abs(derivative.y) > ceiling * math.length(derivative.xz) + .000001f)
                    return false;

            if (depth >= 14)
                return false;

            return Bound(point, pq, middle, depth + 1) && Bound(middle, qr, r, depth + 1);
        }
    }

    public static float ArcLength(PreparedSegment segment)
    {
        float length = 0;
        var previous = NetworkNormalizer.Evaluate(segment, 0);

        for (var i = 1; i <= 64; i++)
        {
            var next = NetworkNormalizer.Evaluate(segment, i / 64f);
            length += math.distance(previous.xz, next.xz);
            previous = next;
        }

        return length;
    }
}
