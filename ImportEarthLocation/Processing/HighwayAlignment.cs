using System;
using System.Collections.Generic;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Smooth alignment of one highway stroke: penalised least squares on samples every
// Spacing metres, minimising sum w(x-p)^2 + mu sum (third difference of x)^2 per axis,
// with mu=(wavelength/(2 pi Spacing))^6, so detail shorter than the wavelength is
// removed. Junction nodes are pinned by heavy weights and the curve runs smoothly
// through them, so a through road has no corner at a ramp. Local XZ metres.
public static class HighwayAlignment
{
    public const double Spacing = 5;
    private const double PinWeight = 1e8;
    private const int BudgetRounds = 3;

    // Returns the fitted samples and, per pin, its sample index. Null if too short to fit.
    public static (double2[] Points, int[] Pins)? Fit(
        IReadOnlyList<double2> line,
        IReadOnlyList<int> pins,
        double wavelength,
        double budget)
    {
        var samples = new List<double2>();
        var pinAt = new List<int>();

        for (var p = 0; p + 1 < pins.Count; p++)
        {
            var length = 0d;

            for (var k = pins[p] + 1; k <= pins[p + 1]; k++)
                length += math.distance(line[k - 1], line[k]);

            var count = Math.Max(1, (int)Math.Round(length / Spacing));

            if (p == 0)
            {
                samples.Add(line[pins[0]]);
                pinAt.Add(0);
            }

            var step = length / count;
            var walked = 0d;
            var k2 = pins[p] + 1;

            for (var i = 1; i < count; i++)
            {
                var target = step * i;

                while (k2 < pins[p + 1] && walked + math.distance(line[k2 - 1], line[k2]) < target)
                {
                    walked += math.distance(line[k2 - 1], line[k2]);
                    k2++;
                }

                var otherDistance = math.distance(line[k2 - 1], line[k2]);
                samples.Add(
                    math.lerp(line[k2 - 1], line[k2], otherDistance > 1e-9 ? (target - walked) / otherDistance : 0));
            }

            samples.Add(line[pins[p + 1]]);
            pinAt.Add(samples.Count - 1);
        }

        var otherCount = samples.Count;

        if (otherCount < 8)
            return null;

        var weights = new double[otherCount];
        Array.Fill(weights, 1d);

        foreach (var pin in pinAt)
            weights[pin] = PinWeight;

        var result = Solve(samples, weights, Math.Pow(wavelength / (2 * Math.PI * Spacing), 6));

        for (var round = 0; round < BudgetRounds; round++)
        {
            var over = false;

            for (var i = 0; i < otherCount; i++)
                if (weights[i] < PinWeight && math.distance(result[i], samples[i]) > budget)
                {
                    over = true;

                    for (var j = Math.Max(0, i - 4); j <= Math.Min(otherCount - 1, i + 4); j++)
                        if (weights[j] < PinWeight)
                            weights[j] = Math.Min(1e4, weights[j] * 2);
                }

            if (!over)
                break;

            result = Solve(samples, weights, Math.Pow(wavelength / (2 * Math.PI * Spacing), 6));
        }

        return (result, pinAt.ToArray());
    }

    // Banded Cholesky of (W + mu D3'D3), half bandwidth three.
    private static double2[] Solve(IReadOnlyList<double2> points, double[] w, double mu)
    {
        var count = points.Count;
        const int b = 3;
        var a = new double[count, b + 1];
        var d = new[] { -1d, 3, -3, 1 };

        for (var i = 0; i < count; i++)
            a[i, 0] = w[i];

        for (var k = 0; k + 3 < count; k++)
            for (var r = 0; r < 4; r++)
                for (var s = r; s < 4; s++)
                    a[k + r, s - r] += mu * d[r] * d[s];

        for (var i = 0; i < count; i++)
            for (var j = 0; j <= b && i + j < count; j++)
            {
                var sum = a[i, j];

                for (var k = 1; k <= b - j && i - k >= 0; k++)
                    sum -= a[i - k, k] * a[i - k, k + j];

                if (j == 0)
                    a[i, 0] = Math.Sqrt(Math.Max(sum, 1e-12));
                else
                    a[i, j] = sum / a[i, 0];
            }

        var otherPoints = new double2[count];

        for (var i = 0; i < count; i++)
            otherPoints[i] = points[i] * w[i];

        for (var i = 0; i < count; i++)
        {
            for (var k = 1; k <= b && i - k >= 0; k++)
                otherPoints[i] -= a[i - k, k] * otherPoints[i - k];

            otherPoints[i] /= a[i, 0];
        }

        for (var i = count - 1; i >= 0; i--)
        {
            for (var k = 1; k <= b && i + k < count; k++)
                otherPoints[i] -= a[i, k] * otherPoints[i + k];

            otherPoints[i] /= a[i, 0];
        }

        return otherPoints;
    }
}
