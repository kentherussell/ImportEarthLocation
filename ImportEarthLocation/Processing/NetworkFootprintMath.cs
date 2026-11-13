using System;
using System.Collections.Generic;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

public readonly record struct FootprintLeaf(int Curve, int Ordinal, double Begin, double End,
    double2 A, double2 B, double2 Tangent, double2 Normal, double Length, double Error, double MaximumSpeed,
    double2 Min, double2 Max, bool Certified);

/// <summary>Correlated full-width contact geometry; analysis leaves never create native nodes.</summary>
public static class NetworkFootprintMath
{
    public static void BuildLeaves(
        Cubic3d curve,
        double halfWidth,
        int curveIndex,
        List<FootprintLeaf> output,
        CancellationToken token)
    {
        void Visit(Cubic3d otherCurve, double first, double last, int depth)
        {
            token.ThrowIfCancellationRequested();
            var vectorA = otherCurve.P0.xz;
            var vectorB = otherCurve.P3.xz;
            var chord = vectorB - vectorA;
            var length = math.length(chord);
            var tangent = length > 0 ? chord / length : double2.zero;
            var normal = new double2(-tangent.y, tangent.x);
            var d0 = 3 * (otherCurve.P1 - otherCurve.P0).xz;
            var d1 = 3 * (otherCurve.P2 - otherCurve.P1).xz;
            var d2 = 3 * (otherCurve.P3 - otherCurve.P2).xz;

            if (math.lengthsq(d0) + math.lengthsq(d1) + math.lengthsq(d2) == 0)
                depth = 16;

            var minimum = Math.Min(math.dot(d0, tangent), Math.Min(math.dot(d1, tangent), math.dot(d2, tangent)));
            var across =
                Math.Max(
                    Math.Abs(math.dot(d0, normal)),
                    Math.Max(Math.Abs(math.dot(d1, normal)), Math.Abs(math.dot(d2, normal))));

            var parameterError =
                Math.Max(
                    math.distance(otherCurve.P1.xz, (2 * vectorA + vectorB) / 3),
                    math.distance(otherCurve.P2.xz, (vectorA + 2 * vectorB) / 3));

            var error =
                minimum > 0
                    ? parameterError + halfWidth * 2 * Math.Sin(Math.Atan(across / minimum) / 2)
                    : double.PositiveInfinity;

            var valid = length > 0 && minimum > 0 && error <= .10;

            if (!valid && depth < 16)
            {
                var split = NetworkDesignChecks.Split(otherCurve, .5);
                var middle = (first + last) / 2;
                Visit(split.Left, first, middle, depth + 1);
                Visit(split.Right, middle, last, depth + 1);

                return;
            }

            // Uncertified finite leaves retain a conservative control-hull box;
            // consumers cannot use them as a clearance/cover certificate.
            var extent = valid ? halfWidth + error : halfWidth;
            var min = valid ? math.min(vectorA, vectorB) - math.abs(tangent) * error - math.abs(normal) * extent :
                math.min(math.min(otherCurve.P0.xz, otherCurve.P1.xz), math.min(otherCurve.P2.xz, otherCurve.P3.xz))
                    - extent;

            var max = valid ? math.max(vectorA, vectorB) + math.abs(tangent) * error + math.abs(normal) * extent :
                math.max(math.max(otherCurve.P0.xz, otherCurve.P1.xz), math.max(otherCurve.P2.xz, otherCurve.P3.xz))
                    + extent;

            output.Add(new(curveIndex, output.Count, first, last, vectorA, vectorB, tangent, normal, length, error,
                Math.Max(math.length(d0), Math.Max(math.length(d1), math.length(d2))), min, max, valid));
        }

        if (!NetworkDesignChecks.Finite(curve) || !math.isfinite(halfWidth) || halfWidth <= 0)
            throw new ArgumentException("Nonfinite or invalid native footprint.");

        Visit(curve, 0, 1, 0);
    }

    public static int Rectangle(FootprintLeaf leaf, double halfWidth, double2 origin, double2[] output)
    {
        if (!leaf.Certified)
            throw new InvalidOperationException("Uncertified footprint has no affine contact certificate.");

        var vectorA = leaf.A - origin - leaf.Tangent * leaf.Error;
        var vectorB = leaf.B - origin + leaf.Tangent * leaf.Error;
        var lateral = leaf.Normal * (halfWidth + leaf.Error);
        output[0] = vectorA - lateral;
        output[1] = vectorB - lateral;
        output[2] = vectorB + lateral;
        output[3] = vectorA + lateral;

        return 4;
    }

    // Clip in a contact-local frame. The caller owns/reuses alternating16-slot
    // buffers. Boundaries are inside; zero-area contacts are retained.
    public static int Clip(double2[] input, int count, double2[] output, double2 normal, double threshold)
    {
        var used = 0;

        if (count == 0)
            return 0;

        var previous = input[count - 1];
        var dp = math.dot(previous, normal) - threshold;

        for (var i = 0; i < count; i++)
        {
            var current = input[i];
            var dc = math.dot(current, normal) - threshold;

            if ((dp >= 0) != (dc >= 0))
                Add(math.lerp(previous, current, dp / (dp - dc)));

            if (dc >= 0)
                Add(current);

            previous = current;
            dp = dc;
        }

        if (used > 1 && math.distance(output[0], output[used - 1]) <= 1e-10)
            used--;

        return used;

        void Add(double2 point)
        {
            if (used > 0 && math.distance(output[used - 1], point) <= 1e-10)
                return;

            if (used == output.Length)
                throw new InvalidOperationException(
                    "Convex clipping exceeded its representable buffer; coverage was not truncated.");

            output[used++] = point;
        }
    }

    public static void Blossom(double p, double q, double r, double[] weights)
    {
        weights[0] = (1 - p) * (1 - q) * (1 - r);
        weights[1] = p * (1 - q) * (1 - r) + (1 - p) * q * (1 - r) + (1 - p) * (1 - q) * r;
        weights[2] = p * q * (1 - r) + p * (1 - q) * r + (1 - p) * q * r;
        weights[3] = p * q * r;
    }

    public static void TriangleBlossom(double p, double q, double r, int coefficient, double[] weights)
    {
        switch (coefficient)
        {
            case 0:
                Blossom(p, p, p, weights);
                break;

            case 1:
                Blossom(p, p, q, weights);
                break;

            case 2:
                Blossom(p, p, r, weights);
                break;

            case 3:
                Blossom(p, q, q, weights);
                break;

            case 4:
                Blossom(p, q, r, weights);
                break;

            case 5:
                Blossom(p, r, r, weights);
                break;

            case 6:
                Blossom(q, q, q, weights);
                break;

            case 7:
                Blossom(q, q, r, weights);
                break;

            case 8:
                Blossom(q, r, r, weights);
                break;

            case 9:
                Blossom(r, r, r, weights);
                break;

            default:
                throw new ArgumentOutOfRangeException(nameof(coefficient));
        }
    }

    public static double Parameter(FootprintLeaf leaf, double2 world)
        => math.dot(world - leaf.A, leaf.Tangent) / leaf.Length;

    public static double TriangleArea(double2 vectorA, double2 vectorB, double2 c)
        => Math.Abs(NetworkDesignChecks.Cross(vectorB - vectorA, c - vectorA)) * .5;
}
