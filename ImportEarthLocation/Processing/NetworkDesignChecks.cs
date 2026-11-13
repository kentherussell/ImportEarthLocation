using System;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

/// <summary>Double geometry and continuous certificates. Quality is not native safety.</summary>
public static class NetworkDesignChecks
{
    public static double3 Evaluate(Cubic3d curve, double t)
    {
        var u = 1 - t;
        return u * u * u * curve.P0 + 3 * u * u * t * curve.P1 + 3 * u * t * t * curve.P2 + t * t * t * curve.P3;
    }

    public static double3 Derivative(Cubic3d curve, double t)
    {
        var u = 1 - t;
        return 3 * (u * u * (curve.P1 - curve.P0) + 2 * u * t * (curve.P2 - curve.P1) + t * t * (curve.P3 - curve.P2));
    }

    public static double3 SecondDerivative(Cubic3d curve, double t)
        => 6 * ((1 - t) * (curve.P2 - 2 * curve.P1 + curve.P0) + t * (curve.P3 - 2 * curve.P2 + curve.P1));

    public static (Cubic3d Left, Cubic3d Right) Split(Cubic3d curve, double t)
    {
        var vectorA = math.lerp(curve.P0, curve.P1, t);
        var vectorB = math.lerp(curve.P1, curve.P2, t);
        var d = math.lerp(curve.P2, curve.P3, t);
        var e = math.lerp(vectorA, vectorB, t);
        var f = math.lerp(vectorB, d, t);
        var point = math.lerp(e, f, t);

        return (new(curve.P0, vectorA, e, point), new(point, f, d, curve.P3));
    }

    public static double Curvature(Cubic3d curve, double t)
    {
        var d = Derivative(curve, t).xz;
        var a = SecondDerivative(curve, t).xz;
        var speed = math.length(d);

        return speed > 0 ? Cross(d, a) / (speed * speed * speed) : double.NaN;
    }

    public static double Cross(double2 vectorA, double2 vectorB) => vectorA.x * vectorB.y - vectorA.y * vectorB.x;

    public static double HorizontalArcLength(Cubic3d curve, double begin = 0, double end = 1)
    {
        if (!Finite(curve))
            return double.NaN;

        if (end < 1)
            curve = Split(curve, end).Left;

        if (begin > 0)
            curve = Split(curve, begin / end).Right;

        double Length(Cubic3d otherCurve, double tolerance, int depth)
        {
            var chord = math.distance(otherCurve.P0.xz, otherCurve.P3.xz);
            var polygon =
                math.distance(otherCurve.P0.xz, otherCurve.P1.xz)
                    + math.distance(otherCurve.P1.xz, otherCurve.P2.xz)
                    + math.distance(otherCurve.P2.xz, otherCurve.P3.xz);

            if (polygon - chord <= tolerance || depth == 18)
                return (chord + polygon) / 2;

            var pair = Split(otherCurve, .5);

            return Length(pair.Left, tolerance / 2, depth + 1) + Length(pair.Right, tolerance / 2, depth + 1);
        }

        return Length(curve, .001, 0);
    }

    // Exact two-variable box-constrained least squares. Stable candidate order,
    // no iterative fitter or unconstrained handle that reverses progression.
    public static Cubic3d FitHandles(double3 start, double3 end, double2 startTangent, double2 endTangent,
        double3[] observations, double[] stations, double[] support, double epsWorld)
    {
        if (observations.Length != stations.Length || stations.Length != support.Length)
            throw new ArgumentException("Source observation dimensions differ.");

        var chord = math.distance(start.xz, end.xz);
        var lower = Math.Max(.05 * chord, 10 * epsWorld);
        var upper = .45 * chord;

        if (!(chord > 0) || !(lower <= upper) || math.lengthsq(startTangent) == 0 || math.lengthsq(endTangent) == 0)
            throw new ArgumentException("Source cluster has no representable bounded handles.");

        startTangent = math.normalize(startTangent);
        endTangent = math.normalize(endTangent);
        double a11 = 0, a12 = 0, a22 = 0, c1 = 0, c2 = 0;

        for (var i = 0; i < observations.Length; i++)
        {
            var t = stations[i];
            var u = 1 - t;
            var b0 = u * u * u;
            var b1 = 3 * u * u * t;
            var b2 = 3 * u * t * t;
            var b3 = t * t * t;
            var v1 = b1 * startTangent;
            var v2 = -b2 * endTangent;
            var residual = observations[i].xz - ((b0 + b1) * start.xz + (b2 + b3) * end.xz);
            var w = support[i];

            if (double.IsNaN(w) || double.IsInfinity(w) || w < 0 || t < 0 || t > 1)
                throw new ArgumentException("Invalid physical observation support.");

            a11 += w * math.dot(v1, v1);
            a12 += w * math.dot(v1, v2);
            a22 += w * math.dot(v2, v2);
            c1 += w * math.dot(v1, residual);
            c2 += w * math.dot(v2, residual);
        }

        var bestA = math.clamp(chord / 3, lower, upper);
        var bestB = bestA;

        double Cost(double a, double b) => a11 * a * a + 2 * a12 * a * b + a22 * b * b - 2 * c1 * a - 2 * c2 * b;

        var best = Cost(bestA, bestB);

        void Candidate(double a, double b)
        {
            a = math.clamp(a, lower, upper);
            b = math.clamp(b, lower, upper);
            var cost = Cost(a, b);

            if (cost < best)
            {
                best = cost;
                bestA = a;
                bestB = b;
            }
        }

        var determinant = a11 * a22 - a12 * a12;
        var trace = a11 + a22;

        if (determinant > 1e-12 * trace * trace)
        {
            var a = (c1 * a22 - c2 * a12) / determinant;
            var b = (c2 * a11 - c1 * a12) / determinant;

            if (a >= lower && a <= upper && b >= lower && b <= upper)
                Candidate(a, b);
        }

        Candidate(lower, a22 > 0 ? (c2 - a12 * lower) / a22 : bestB);
        Candidate(upper, a22 > 0 ? (c2 - a12 * upper) / a22 : bestB);
        Candidate(a11 > 0 ? (c1 - a12 * lower) / a11 : bestA, lower);
        Candidate(a11 > 0 ? (c1 - a12 * upper) / a11 : bestA, upper);

        return new(
            start,
            new(start.x + bestA * startTangent.x, math.lerp(start.y, end.y, 1 / 3d), start.z + bestA * startTangent.y),
            new(end.x - bestB * endTangent.x, math.lerp(start.y, end.y, 2 / 3d), end.z - bestB * endTangent.y), end);
    }

    public static bool Finite(Cubic3d curve)
        => math.all(math.isfinite(curve.P0))
            && math.all(math.isfinite(curve.P1))
            && math.all(math.isfinite(curve.P2))
            && math.all(math.isfinite(curve.P3));

    public static bool HasHorizontalExtent(Cubic3d curve) =>
        math.lengthsq(curve.P1.xz - curve.P0.xz)
            + math.lengthsq(curve.P2.xz - curve.P0.xz)
            + math.lengthsq(curve.P3.xz - curve.P0.xz)
            > 0;

    public static CurveCertificate Certify(
        Cubic3d curve,
        double maximumCurvature,
        double maximumGrade,
        double numericalSpeed = 1e-10,
        CancellationToken token = default,
        int maximumLeaves = int.MaxValue)
    {
        if (!Finite(curve))
            return new(DesignQuality.RequiresReview, double.PositiveInfinity, double.PositiveInfinity, 0, true);

        var curvature = 0d;
        var grade = 0d;
        var leaves = 0;
        var certified = true;
        var visits = 0;

        void Visit(Cubic3d otherCurve, int depth)
        {
            // Exhausting analysis effort means unknown quality, not a proof,
            // a geometry deletion, or another chance to reshape the curve.
            if (leaves >= maximumLeaves)
            {
                certified = false;
                curvature = grade = double.PositiveInfinity;

                return;
            }

            if ((visits++ & 1023) == 0)
                token.ThrowIfCancellationRequested();

            var d0 = 3 * (otherCurve.P1 - otherCurve.P0);
            var d1 = 3 * (otherCurve.P2 - otherCurve.P1);
            var d2 = 3 * (otherCurve.P3 - otherCurve.P2);

            if (math.lengthsq(d0.xz) + math.lengthsq(d1.xz) + math.lengthsq(d2.xz) == 0)
            {
                certified = false;
                leaves++;
                curvature = grade = double.PositiveInfinity;

                return;
            }

            var chord = otherCurve.P3.xz - otherCurve.P0.xz;
            var length = math.length(chord);
            var direction = length > 0 ? chord / length : double2.zero;
            var lower =
                Math.Min(math.dot(d0.xz, direction), Math.Min(math.dot(d1.xz, direction), math.dot(d2.xz, direction)));

            var boundK = double.PositiveInfinity;
            var boundG = double.PositiveInfinity;

            if (lower > numericalSpeed)
            {
                var a0 = 2 * (d1.xz - d0.xz);
                var a1 = 2 * (d2.xz - d1.xz);
                var n0 = Cross(d0.xz, a0);
                var n1 = (Cross(d0.xz, a1) + 2 * Cross(d1.xz, a0)) / 3;
                var n2 = (2 * Cross(d1.xz, a1) + Cross(d2.xz, a0)) / 3;
                var n3 = Cross(d2.xz, a1);
                boundK =
                    Math.Max(Math.Max(Math.Abs(n0), Math.Abs(n1)), Math.Max(Math.Abs(n2), Math.Abs(n3)))
                        / (lower * lower * lower);

                boundG = Math.Max(Math.Abs(d0.y), Math.Max(Math.Abs(d1.y), Math.Abs(d2.y))) / lower;
            }

            if (lower > numericalSpeed && boundK <= maximumCurvature && boundG <= maximumGrade)
            {
                leaves++;
                curvature = Math.Max(curvature, boundK);
                grade = Math.Max(grade, boundG);

                return;
            }

            if (depth == 16)
            {
                certified = false;
                leaves++;
                curvature = Math.Max(curvature, boundK);
                grade = Math.Max(grade, boundG);

                return;
            }

            var split = Split(otherCurve, .5);
            Visit(split.Left, depth + 1);
            Visit(split.Right, depth + 1);
        }

        Visit(curve, 0);

        return new(certified ? DesignQuality.Certified : DesignQuality.RequiresReview, curvature, grade, leaves, false);
    }

    // Conservative horizontal proof that two curves stay at least minimum apart.
    public static bool Separated(Cubic3d firstCurve, Cubic3d secondCurve, double minimum, CancellationToken token)
    {
        var visits = 0;

        bool Prove(Cubic3d first, Cubic3d second, int firstDepth, int secondDepth)
        {
            token.ThrowIfCancellationRequested();

            if (++visits > 32768)
                return false;

            var lowA = math.min(math.min(first.P0.xz, first.P1.xz), math.min(first.P2.xz, first.P3.xz));
            var highA = math.max(math.max(first.P0.xz, first.P1.xz), math.max(first.P2.xz, first.P3.xz));
            var lowB = math.min(math.min(second.P0.xz, second.P1.xz), math.min(second.P2.xz, second.P3.xz));
            var highB = math.max(math.max(second.P0.xz, second.P1.xz), math.max(second.P2.xz, second.P3.xz));

            if (math.length(math.max(double2.zero, math.max(lowA - highB, lowB - highA))) >= minimum - 1e-7)
                return true;

            var delta = Evaluate(first, .5).xz - Evaluate(second, .5).xz;
            var distance = math.length(delta);

            if (!(distance >= minimum - 1e-7))
                return false;

            var axis = delta / distance;
            var lowerA = Math.Min(Math.Min(math.dot(first.P0.xz, axis), math.dot(first.P1.xz, axis)),
                Math.Min(math.dot(first.P2.xz, axis), math.dot(first.P3.xz, axis)));

            var upperB = Math.Max(Math.Max(math.dot(second.P0.xz, axis), math.dot(second.P1.xz, axis)),
                Math.Max(math.dot(second.P2.xz, axis), math.dot(second.P3.xz, axis)));

            if (lowerA - upperB >= minimum - 1e-7)
                return true;

            // Independent curve parameters: equal-station distance alone can
            // miss a diagonal overlap. Split one hull and cover both products,
            // including cross-boundary parameter combinations.
            if (firstDepth == 16 && secondDepth == 16)
                return false;

            if (secondDepth == 16 || firstDepth < 16 && math.lengthsq(highA - lowA) >= math.lengthsq(highB - lowB))
            {
                var split = Split(first, .5);
                return Prove(split.Left, second, firstDepth + 1, secondDepth)
                    && Prove(split.Right, second, firstDepth + 1, secondDepth);
            }

            var other = Split(second, .5);

            return Prove(first, other.Left, firstDepth, secondDepth + 1)
                && Prove(first, other.Right, firstDepth, secondDepth + 1);
        }

        return Finite(firstCurve) && Finite(secondCurve) && minimum > 0 && Prove(firstCurve, secondCurve, 0, 0);
    }

    public static Cubic3d Course(PreparedSegment segment)
        => new(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd);

    public static double HorizontalLength(PreparedSegment segment) => NetworkDesignChecks.HorizontalArcLength(
                new(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd));
}
