using System;
using System.Collections.Generic;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Smooth alignment of one road stroke (report 113, 5.2): penalised least squares on
// samples every Spacing metres, minimising sum w(x-p)^2 + mu sum (third difference of x)^2
// per axis, with mu=(wavelength/(2 pi Spacing))^6, so detail shorter than the wavelength
// is removed. A sample's weight says how firmly it is held: PinWeight fixes a node that
// is already placed, HoldWeight keeps a deliberate shape such as a fillet, and one leaves
// the sample, junctions included, to the fit. Local XZ metres.
//
// The buffers are kept between strokes, so one instance serves one design on one thread.
public sealed class RoadAlignment
{
    public const double Spacing = 5;
    public const double PinWeight = 1e8;
    public const double HoldWeight = 1e4;

    // A range shorter than this has too few samples to shape and stays straight.
    private const int MinimumSamples = 4;
    private const int BudgetRounds = 4;
    private const int HalfBandwidth = 3;

    private static readonly double[] s_ThirdDifference = { -1, 3, -3, 1 };

    private double[] m_Band = Array.Empty<double>();
    private double[] m_Weights = Array.Empty<double>();

    // Samples about Spacing apart along the line, with one exactly on every anchor.
    // The anchors are ascending line indices that start at zero and end at the last
    // point. Origin receives, per sample, the line index at or just after it.
    public static int[] Resample(
        IReadOnlyList<double2> line,
        IReadOnlyList<int> anchors,
        List<double2> samples,
        List<int> origin)
    {
        var anchorAt = new int[anchors.Count];
        samples.Add(line[anchors[0]]);
        origin.Add(anchors[0]);

        for (var a = 0; a + 1 < anchors.Count; a++)
        {
            var length = 0d;

            for (var k = anchors[a] + 1; k <= anchors[a + 1]; k++)
                length += math.distance(line[k - 1], line[k]);

            var count = Math.Max(1, (int)Math.Round(length / Spacing));
            var step = length / count;
            var walked = 0d;
            var index = anchors[a] + 1;

            for (var i = 1; i < count; i++)
            {
                var target = step * i;

                while (index < anchors[a + 1] && walked + math.distance(line[index - 1], line[index]) < target)
                {
                    walked += math.distance(line[index - 1], line[index]);
                    index++;
                }

                var segment = math.distance(line[index - 1], line[index]);
                samples.Add(math.lerp(line[index - 1], line[index], segment > 1e-9 ? (target - walked) / segment : 0));
                origin.Add(index);
            }

            samples.Add(line[anchors[a + 1]]);
            origin.Add(anchors[a + 1]);
            anchorAt[a + 1] = samples.Count - 1;
        }

        return anchorAt;
    }

    // The fitted samples. The curve is smooth through every sample except the breaks
    // (ascending interior sample indices), where it may turn: each side is fitted on
    // its own and both meet at the break's target. Free samples left more than the
    // budget from their targets are weighted up, with their neighbours, and solved again.
    public double2[] Fit(
        IReadOnlyList<double2> targets,
        IReadOnlyList<double> weights,
        IReadOnlyList<int> breaks,
        double wavelength,
        double budget)
    {
        var count = targets.Count;
        var result = new double2[count];

        if (m_Weights.Length < count)
        {
            m_Weights = new double[count];
            m_Band = new double[count * (HalfBandwidth + 1)];
        }

        for (var i = 0; i < count; i++)
            m_Weights[i] = weights[i];

        var mu = Math.Pow(wavelength / (2 * Math.PI * Spacing), 6);
        var from = 0;

        for (var b = 0; b <= breaks.Count; b++)
        {
            var to = b < breaks.Count ? breaks[b] : count - 1;

            if (to > from)
                FitRange(targets, result, from, to, mu, budget);

            from = to;
        }

        if (count == 1)
            result[0] = targets[0];

        return result;
    }

    private void FitRange(IReadOnlyList<double2> targets, double2[] result, int from, int to, double mu, double budget)
    {
        var count = to - from + 1;

        if (count < MinimumSamples)
        {
            for (var i = from; i <= to; i++)
                result[i] = math.lerp(targets[from], targets[to], (i - from) / (double)(to - from));

            return;
        }

        Solve(targets, result, from, count, mu);

        for (var round = 0; round < BudgetRounds; round++)
        {
            var over = false;

            for (var i = from; i <= to; i++)
            {
                if (m_Weights[i] >= HoldWeight || math.distance(result[i], targets[i]) <= budget)
                    continue;

                over = true;

                for (var j = Math.Max(from, i - 4); j <= Math.Min(to, i + 4); j++)
                    if (m_Weights[j] < HoldWeight)
                        m_Weights[j] = Math.Min(HoldWeight, m_Weights[j] * 4);
            }

            if (!over)
                break;

            Solve(targets, result, from, count, mu);
        }
    }

    // Banded Cholesky of (W + mu D3'D3), half bandwidth three, on count samples from 'from'.
    private void Solve(IReadOnlyList<double2> targets, double2[] result, int from, int count, double mu)
    {
        const int b = HalfBandwidth;
        const int stride = b + 1;
        var a = m_Band;
        Array.Clear(a, 0, count * stride);

        for (var i = 0; i < count; i++)
            a[i * stride] = m_Weights[from + i];

        for (var k = 0; k + 3 < count; k++)
            for (var r = 0; r < 4; r++)
                for (var s = r; s < 4; s++)
                    a[(k + r) * stride + s - r] += mu * s_ThirdDifference[r] * s_ThirdDifference[s];

        for (var i = 0; i < count; i++)
            for (var j = 0; j <= b && i + j < count; j++)
            {
                var sum = a[i * stride + j];

                for (var k = 1; k <= b - j && i - k >= 0; k++)
                    sum -= a[(i - k) * stride + k] * a[(i - k) * stride + k + j];

                if (j == 0)
                    a[i * stride] = Math.Sqrt(Math.Max(sum, 1e-12));
                else
                    a[i * stride + j] = sum / a[i * stride];
            }

        for (var i = 0; i < count; i++)
            result[from + i] = targets[from + i] * m_Weights[from + i];

        for (var i = 0; i < count; i++)
        {
            for (var k = 1; k <= b && i - k >= 0; k++)
                result[from + i] -= a[(i - k) * stride + k] * result[from + i - k];

            result[from + i] /= a[i * stride];
        }

        for (var i = count - 1; i >= 0; i--)
        {
            for (var k = 1; k <= b && i + k < count; k++)
                result[from + i] -= a[i * stride + k] * result[from + i + k];

            result[from + i] /= a[i * stride];
        }
    }
}
