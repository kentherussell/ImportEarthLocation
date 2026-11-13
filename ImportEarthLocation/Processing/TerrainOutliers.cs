using System;
using System.Collections.Generic;
using System.Threading;

namespace ImportEarthLocation.Processing;

/// <summary>
/// Replaces source elevations that cannot be real ground before anything else reads them.
/// </summary>
/// <remarks>
/// Public elevation tiles hold patches of corrupt pixels: New Orleans' lakes decode to
/// −8 to −23 km beside water at 0 m, in the tile and its parent alike. A sample is
/// corrupt when it is no valid Earth elevation, or when it stands further from its
/// neighbourhood than real ground can: more than <see cref="Reach"/> below the
/// neighbourhood's lower quartile or above its upper quartile, widened to
/// <see cref="Spreads"/> times the quartile spread where the ground itself is rough.
/// The neighbourhood is <see cref="Window"/> blocks of <see cref="Block"/> samples a
/// side (about 1.5 km at zoom 13), so a corrupt patch is a small share of it and the
/// quartiles stay on real ground. Real cliffs, canyons and quarries keep their depth:
/// rough ground widens the band, and even a narrow gorge cut into flat ground, a
/// fifth of the neighbourhood, would need walls over a kilometre high to leave it.
/// Corrupt samples seen so far lie 8 km or more from the ground around them.
/// Corruption smears into the samples around it (New Orleans keeps a halo down to
/// −995 m), so a patch then grows into neighbouring samples outside a much tighter
/// band, <see cref="HaloReach"/>: beside proven corruption, that is no real pit.
/// A sample more than <see cref="StepLimit"/> off both its neighbours along either axis
/// (7 to 20 m away at zoom 13) is corrupt too, wherever it stands: no pit, spike, trench
/// or ridge is one sample wide and that deep or tall. That catches pits and trenches a
/// kilometre deep in flat ground (Panama City, Darwin) that stay within <see cref="Reach"/>
/// of their neighbourhood. Larger corrupt
/// plateaus whose rims pass the quartile test (Reykjavík) are not caught: growing a region
/// from a drop and keeping the side that closes also flattened real hills (Mexico City) and
/// filled a real lowland ringed by a corrupt plateau (Iquitos).
/// Every corrupt sample, however large its patch, is then invented from the valid ground
/// around it as a smooth (harmonic) surface, so a patch in a lake becomes the lake, one
/// on a slope continues the slope, and one across a valley spans it from both sides.
/// Only a source with no valid ground at all is refused.
/// </remarks>
public static class TerrainOutliers
{
    public const float Reach = 1000;
    public const float HaloReach = 50;
    public const float StepLimit = 300;
    public const float Spreads = 3;
    public const int Block = 32;
    public const int Window = 3;

    private const int Stride = 4;

    // Gauss-Seidel sweeps over the gaps at each level of the fill. Each level starts
    // from the coarser level's surface, so the sweeps only need to smooth a few samples.
    private const int Relaxations = 40;

    public sealed record Outcome(int Samples, int Patches, int LargestPatch, float Lowest, float Highest);

    public static Outcome Replace(float[] raster, int width, int height, CancellationToken token)
    {
        var corrupt = Find(raster, width, height, token);
        var count = 0;

        foreach (var flag in corrupt)
            if (flag)
                count++;

        if (count == 0)
            return new Outcome(0, 0, 0, float.NaN, float.NaN);

        if (count == raster.Length)
            throw new InvalidOperationException("Corrupt elevation: the source holds no valid ground to fill from.");

        var lowest = float.PositiveInfinity;
        var highest = float.NegativeInfinity;

        for (var i = 0; i < raster.Length; i++)
        {
            if (!corrupt[i] || !TerrainConditioner.Finite(raster[i]))
                continue;

            lowest = Math.Min(lowest, raster[i]);
            highest = Math.Max(highest, raster[i]);
        }

        var patches = 0;
        var largest = 0;
        var seen = new bool[raster.Length];

        for (var seed = 0; seed < raster.Length; seed++)
        {
            if (!corrupt[seed] || seen[seed])
                continue;

            token.ThrowIfCancellationRequested();
            var patch = Patch(corrupt, seen, width, height, seed);
            patches++;
            largest = Math.Max(largest, patch.Count);
        }

        Fill(raster, corrupt, width, height, token);

        return new Outcome(count, patches, largest, lowest, highest);
    }

    // Marks every sample that is no valid Earth elevation or lies outside its
    // neighbourhood's band, then grows those patches through the halo. Quartiles
    // come from every Stride-th sample of the Window × Window blocks around each
    // block, skipping invalid samples.
    private static bool[] Find(float[] raster, int width, int height, CancellationToken token)
    {
        var corrupt = new bool[raster.Length];
        var blocksX = (width + Block - 1) / Block;
        var blocksY = (height + Block - 1) / Block;
        var haloFloor = new float[blocksX * blocksY];
        var haloCeiling = new float[blocksX * blocksY];
        var reach = Window / 2 * Block;
        var samples = new List<float>((2 * reach + Block) * (2 * reach + Block) / (Stride * Stride) + 1);
        var halo = new List<int>();

        for (var blockY = 0; blockY < blocksY; blockY++)
        {
            token.ThrowIfCancellationRequested();

            for (var blockX = 0; blockX < blocksX; blockX++)
            {
                var x0 = blockX * Block;
                var y0 = blockY * Block;
                var x1 = Math.Min(width, x0 + Block);
                var y1 = Math.Min(height, y0 + Block);
                samples.Clear();

                for (var y = Math.Max(0, y0 - reach); y < Math.Min(height, y1 + reach); y += Stride)
                    for (var x = Math.Max(0, x0 - reach); x < Math.Min(width, x1 + reach); x += Stride)
                    {
                        var value = raster[y * width + x];

                        if (TerrainConditioner.ValidEarthElevation(value))
                            samples.Add(value);
                    }

                var block = blockY * blocksX + blockX;
                var floor = float.NegativeInfinity;
                var ceiling = float.PositiveInfinity;
                haloFloor[block] = float.NegativeInfinity;
                haloCeiling[block] = float.PositiveInfinity;

                if (samples.Count >= 16)
                {
                    samples.Sort();
                    var lower = samples[samples.Count / 4];
                    var upper = samples[samples.Count * 3 / 4];
                    var spread = Spreads * (upper - lower);
                    floor = lower - Math.Max(Reach, spread);
                    ceiling = upper + Math.Max(Reach, spread);
                    haloFloor[block] = lower - Math.Max(HaloReach, spread);
                    haloCeiling[block] = upper + Math.Max(HaloReach, spread);
                }

                for (var y = y0; y < y1; y++)
                    for (var x = x0; x < x1; x++)
                    {
                        var index = y * width + x;
                        var value = raster[index];
                        corrupt[index] =
                            !TerrainConditioner.ValidEarthElevation(value) || value < floor || value > ceiling;

                        if (corrupt[index])
                            halo.Add(index);
                    }
            }
        }

        for (var k = 0; k < halo.Count; k++)
        {
            var index = halo[k];
            var x = index % width;
            var y = index / width;

            void Grow(int next)
            {
                if (corrupt[next])
                    return;

                var block = next / width / Block * blocksX + next % width / Block;
                var value = raster[next];

                if (value >= haloFloor[block] && value <= haloCeiling[block])
                    return;

                corrupt[next] = true;
                halo.Add(next);
            }

            if (x > 0)
                Grow(index - 1);

            if (x + 1 < width)
                Grow(index + 1);

            if (y > 0)
                Grow(index - width);

            if (y + 1 < height)
                Grow(index + width);
        }

        Pits(raster, corrupt, width, height, token);

        return corrupt;
    }

    // Marks samples more than StepLimit off both neighbours along either axis: a pit,
    // spike, trench or ridge one sample wide.
    private static void Pits(float[] raster, bool[] corrupt, int width, int height, CancellationToken token)
    {
        for (var y = 1; y < height - 1; y++)
        {
            if ((y & 255) == 0)
                token.ThrowIfCancellationRequested();

            for (var x = 1; x < width - 1; x++)
            {
                var index = y * width + x;

                if (corrupt[index])
                    continue;

                var value = raster[index];
                var acrossX = Far(raster[index - 1], value) && Far(raster[index + 1], value);
                var acrossZ = Far(raster[index - width], value) && Far(raster[index + width], value);

                if (acrossX || acrossZ)
                    corrupt[index] = true;
            }
        }
    }

    private static bool Far(float neighbour, float value) => Math.Abs(neighbour - value) > StepLimit;

    private static List<int> Patch(bool[] corrupt, bool[] seen, int width, int height, int seed)
    {
        var patch = new List<int> { seed };
        seen[seed] = true;

        for (var k = 0; k < patch.Count; k++)
        {
            var index = patch[k];
            var x = index % width;
            var y = index / width;

            void Add(int next)
            {
                if (corrupt[next] && !seen[next])
                {
                    seen[next] = true;
                    patch.Add(next);
                }
            }

            if (x > 0)
                Add(index - 1);

            if (x + 1 < width)
                Add(index + 1);

            if (y > 0)
                Add(index - width);

            if (y + 1 < height)
                Add(index + width);
        }

        return patch;
    }

    // Fills every corrupt sample from the valid ground around it, coarse to fine. A
    // grid half as wide (each 2 × 2 block the mean of its valid samples, a gap when
    // it has none) is filled first by the same method, until a level has no gaps;
    // the last level always has one valid sample, since the caller found one. Each
    // gap then starts from the coarser surface and is relaxed towards the harmonic
    // surface its valid neighbours define: each gap the mean of its four neighbours,
    // with the map edge holding no value. The work is about a third more than one
    // pass over the grid plus the sweeps, whatever the depth of a patch.
    private static void Fill(float[] raster, bool[] corrupt, int width, int height, CancellationToken token)
    {
        token.ThrowIfCancellationRequested();
        var gaps = new List<int>();

        for (var index = 0; index < raster.Length; index++)
            if (corrupt[index])
                gaps.Add(index);

        if (gaps.Count == 0)
            return;

        var coarseWidth = (width + 1) / 2;
        var coarseHeight = (height + 1) / 2;
        var coarse = new float[coarseWidth * coarseHeight];
        var coarseGaps = new bool[coarse.Length];

        for (var y = 0; y < coarseHeight; y++)
            for (var x = 0; x < coarseWidth; x++)
            {
                var sum = 0f;
                var count = 0;

                for (var fineY = 2 * y; fineY < Math.Min(height, 2 * y + 2); fineY++)
                    for (var fineX = 2 * x; fineX < Math.Min(width, 2 * x + 2); fineX++)
                    {
                        var index = fineY * width + fineX;

                        if (corrupt[index])
                            continue;

                        sum += raster[index];
                        count++;
                    }

                coarse[y * coarseWidth + x] = count > 0 ? sum / count : 0;
                coarseGaps[y * coarseWidth + x] = count == 0;
            }

        Fill(coarse, coarseGaps, coarseWidth, coarseHeight, token);

        foreach (var index in gaps)
            raster[index] = coarse[index / width / 2 * coarseWidth + index % width / 2];

        for (var sweep = 0; sweep < Relaxations; sweep++)
            foreach (var index in gaps)
                raster[index] = Neighbours(raster, width, height, index);

        foreach (var index in gaps)
            corrupt[index] = false;
    }

    private static float Neighbours(float[] raster, int width, int height, int index)
    {
        var x = index % width;
        var y = index / width;
        var sum = 0f;
        var count = 0;

        void Add(int next)
        {
            sum += raster[next];
            count++;
        }

        if (x > 0)
            Add(index - 1);

        if (x + 1 < width)
            Add(index + 1);

        if (y > 0)
            Add(index - width);

        if (y + 1 < height)
            Add(index + width);

        return count == 0 ? float.NaN : sum / count;
    }
}
