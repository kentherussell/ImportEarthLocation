using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;

namespace ImportEarthLocation.Processing;

/// <summary>Source-grid operations only. Never mutates the source reference or calls Unity.</summary>
public static class TerrainConditioner
{
    public const string Version = "conservative-v1";
    public const float MaximumGeneralCorrection = 50f;
    public static bool ValidEarthElevation(float value) => Finite(value) && value >= -12000 && value <= 10000;
    public sealed record HeightEncoding(float BaseElevation, float Scale, float SeaLevel);
    public static HeightEncoding EncodingFor(float minimum,float maximum)
    {
        // Absolute sample bounds, not an unrelated 12km total-span cutoff.
        // Deep water plus land can legitimately span more than that cutoff.
        if(!ValidEarthElevation(minimum)||!ValidEarthElevation(maximum)||maximum<minimum)
            throw new InvalidOperationException($"DEM encoding: invalid validated elevation bounds ({minimum} to {maximum} m).");
        var floor=Math.Min(-128f,minimum-128f);
        return new HeightEncoding(floor,Math.Max(4096f,maximum-floor+10f),-floor);
    }
    public sealed record Statistics(int Repaired, int Spikes, int Pits, int Noise, int SeamCandidates,
        int SeamsCorrected, double MeanAbsoluteChange, float MaximumChange, double MaterialPercent,
        double Bias, float Percentile95, double Seconds)
    {
        public string ValidationNote { get; init; } = "validation passed";
        public override string ToString() => $"{Version}: NoData repaired {Repaired}, unresolved 0; ambiguous seam candidates {SeamCandidates}, corrected {SeamsCorrected}; spikes {Spikes}, pits {Pits}; noise samples {Noise}; hydrology corrections 0 (max 0 m); mean |change| {MeanAbsoluteChange:0.####} m, maximum general {MaximumChange:0.###} m, p95 of changed samples {Percentile95:0.###} m; materially changed {MaterialPercent:0.####}%; bias {Bias:0.####} m; {Seconds:0.00}s; {ValidationNote}.";
    }
    public sealed record Result(float[] Elevations, Statistics Metrics);

    public static Result Condition(float[] source, int width, int height, float metersPerCell,
        CancellationToken token, Func<int, bool>? protectedSample = null, int tileSize = 256, Action<string>? progress = null,
        int sourceOriginX = 0, int sourceOriginY = 0)
    {
        if (width < 3 || height < 3 || tileSize < 1 || source.Length != checked(width * height) || !Finite(metersPerCell) || metersPerCell <= 0)
            throw new InvalidOperationException("Terrain conditioning: invalid raster dimensions or spacing.");
        var clock = Stopwatch.StartNew();
        progress?.Invoke($"source-validation begin; samples={source.Length}");
        var work = (float[])source.Clone();
        var missing = new List<int>();
        for (var i = 0; i < source.Length; i++)
        {
            if ((i & 16383) == 0) token.ThrowIfCancellationRequested();
            if (float.IsNaN(source[i])) missing.Add(i);
            else if (!ValidEarthElevation(source[i]))
                throw new InvalidOperationException($"DEM validation: invalid Earth elevation at raster cell {i} ({source[i]} m).");
        }
        // NoData is NOT zero. Limit each connected hole to four source cells and total missing to 0.01%.
        if (missing.Count > Math.Max(4, source.Length / 10000))
            throw new InvalidOperationException($"NoData repair: {missing.Count} unknown source samples exceed the bounded repair budget; ocean absence cannot be assumed without a water mask.");
        var missingSet = new HashSet<int>(missing);
        progress?.Invoke($"NoData-repair begin; missingSamples={missing.Count}");
        foreach (var seed in missing)
        {
            if (!missingSet.Remove(seed)) continue;
            var cluster = new List<int> { seed };
            for (var p = 0; p < cluster.Count; p++)
            {
                token.ThrowIfCancellationRequested();
                var index = cluster[p]; var x = index % width; var y = index / width;
                foreach (var next in new[] { x > 0 ? index - 1 : -1, x + 1 < width ? index + 1 : -1, y > 0 ? index - width : -1, y + 1 < height ? index + width : -1 })
                    if (missingSet.Remove(next)) cluster.Add(next);
                if (cluster.Count > 4) throw new InvalidOperationException($"NoData repair: contiguous missing area near {seed % width},{seed / width} exceeds four source cells.");
            }
            foreach (var index in cluster)
            {
                if (protectedSample?.Invoke(index) == true)
                    throw new InvalidOperationException("NoData repair: missing data intersects protected shoreline/water/landform geometry; refusing to invent terrain.");
                var x = index % width; var y = index / width;
                var estimates = new List<float>();
                foreach (var delta in new[] { 1, width })
                {
                    var a = 1; var b = 1;
                    while (a <= 5 && Inside(x, y, delta, -a, width, height) && !Finite(source[index - delta * a])) a++;
                    while (b <= 5 && Inside(x, y, delta, b, width, height) && !Finite(source[index + delta * b])) b++;
                    if (a <= 5 && b <= 5 && Inside(x, y, delta, -a, width, height) && Inside(x, y, delta, b, width, height))
                        estimates.Add((source[index - delta * a] * b + source[index + delta * b] * a) / (a + b));
                }
                if (estimates.Count == 0 || estimates.Max() - estimates.Min() > 3)
                    throw new InvalidOperationException($"NoData repair: insufficient consistent surrounding slope at {x},{y}.");
                work[index] = estimates.Average();
            }
        }
        // Whole-tile offsets are corruption, not terrain to beautify. A coherent real cliff is preserved.
        var seams = 0;
        progress?.Invoke("tile-seam inspection begin");
        // Cropping the mandatory footprint must not move the original tile grid.
        // Inspect actual source tile seams, not every 256th cell of the crop.
        var firstX = tileSize - ((sourceOriginX % tileSize + tileSize) % tileSize);
        var firstY = tileSize - ((sourceOriginY % tileSize + tileSize) % tileSize);
        for (var boundary = firstX; boundary < width; boundary += tileSize)
            seams += InspectSeam(source, width, height, boundary, true, token);
        for (var boundary = firstY; boundary < height; boundary += tileSize)
            seams += InspectSeam(source, width, height, boundary, false, token);
        var spikes = 0; var pits = 0; var noise = 0;
        var residuals = new float[24];
        progress?.Invoke($"noise/spike conditioning begin; rows={height}");
        var nextProgress=clock.Elapsed.TotalSeconds+5;
        for (var y = 2; y < height - 2; y++)
        {
            token.ThrowIfCancellationRequested();
            if(clock.Elapsed.TotalSeconds>=nextProgress){progress?.Invoke($"noise/spike conditioning rows={y}/{height}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.0}");nextProgress=clock.Elapsed.TotalSeconds+5;}
            for (var x = 2; x < width - 2; x++)
            {
                var index = y * width + x; var original = source[index];
                // Preserve uncertain coasts/islands, protected mapped water/landforms, and repaired NoData neighborhoods.
                if (!Finite(original) || Math.Abs(original) < 5 || protectedSample?.Invoke(index) == true) continue;
                var sum = 0d; var sx = 0d; var sy = 0d; var count = 0; var valid = true;
                for (var dy = -2; dy <= 2; dy++) for (var dx = -2; dx <= 2; dx++)
                {
                    if (dx == 0 && dy == 0) continue;
                    var h = source[(y + dy) * width + x + dx];
                    if (!Finite(h)) { valid = false; continue; }
                    sum += h; sx += dx * h; sy += dy * h; count++;
                }
                if (!valid || count != 24) continue;
                // Symmetric least-squares plane preserves slope instead of flattening sloped surfaces.
                var estimate = (float)(sum / 24); var slopeX = sx / 50; var slopeY = sy / 50;
                if (Math.Abs(estimate) < 5 || Math.Sign(estimate) != Math.Sign(original)) continue; // Never erase an island or fill an ocean hole without coastline evidence.
                var n = 0; float maxResidual = 0;
                for (var dy = -2; dy <= 2; dy++) for (var dx = -2; dx <= 2; dx++)
                {
                    if (dx == 0 && dy == 0) continue;
                    var r = (float)Math.Abs(source[(y + dy) * width + x + dx] - estimate - slopeX * dx - slopeY * dy);
                    residuals[n++] = r; maxResidual = Math.Max(maxResidual, r);
                }
                Array.Sort(residuals);
                var deviation = original - estimate;
                // A single sharp residual in a very coherent low-slope 5x5 neighborhood only.
                if (Math.Abs(deviation) >= 8 && Math.Abs(deviation) <= MaximumGeneralCorrection &&
                    residuals[12] <= 0.5 && maxResidual <= 1.5 && Math.Abs(slopeX) + Math.Abs(slopeY) <= 3)
                {
                    work[index] = estimate;
                    if (deviation > 0) spikes++; else pits++;
                }
                else if (Math.Abs(deviation) >= 0.3 && Math.Abs(deviation) <= 1 && maxResidual <= 0.6 &&
                    Math.Abs(slopeX) + Math.Abs(slopeY) <= 0.5)
                {
                    work[index] = original - deviation * 0.15f; noise++;
                }
            }
        }
        var changes = new List<float>(); double absolute = 0, bias = 0; float maximum = 0; var material = 0;
        for (var i = 0; i < work.Length; i++)
        {
            if ((i & 16383) == 0) token.ThrowIfCancellationRequested();
            if (!Finite(work[i])) throw new InvalidOperationException($"Terrain validation: unresolved non-finite cell {i}.");
            if (!Finite(source[i])) continue;
            var delta = work[i] - source[i]; var magnitude = Math.Abs(delta);
            if (magnitude > MaximumGeneralCorrection + .001f) throw new InvalidOperationException("Terrain validation: correction exceeded general safety limit.");
            if (magnitude > 0) changes.Add(magnitude);
            if (magnitude >= 0.25f) material++;
            absolute += magnitude; bias += delta; maximum = Math.Max(maximum, magnitude);
        }
        if (material > work.Length * .02 || Math.Abs(bias / work.Length) > .05)
        {
            // Optional cosmetic filtering is not allowed to invalidate otherwise usable source data.
            // Keep validated hole repairs, but revert every finite source sample atomically in this buffer.
            for (var i = 0; i < work.Length; i++) if (Finite(source[i])) work[i] = source[i];
            token.ThrowIfCancellationRequested();
            return new Result(work, new Statistics(missing.Count, 0, 0, 0, seams, 0, 0, 0, 0, 0, 0, clock.Elapsed.TotalSeconds)
            { ValidationNote = "optional artifact/noise filters reverted: change budget exceeded; validated source and bounded NoData repairs preserved" });
        }
        changes.Sort();
        return new Result(work, new Statistics(missing.Count, spikes, pits, noise, seams, 0,
            absolute / work.Length, maximum, material * 100d / work.Length, bias / work.Length,
            changes.Count == 0 ? 0 : changes[(int)((changes.Count - 1) * .95)], clock.Elapsed.TotalSeconds));
    }

    private static int InspectSeam(float[] source, int width, int height, int boundary, bool vertical, CancellationToken token)
    {
        if (boundary < 2 || boundary + 1 >= (vertical ? width : height)) return 0;
        var jumps = new List<float>();
        for (var p = 2; p < (vertical ? height : width) - 2; p++)
        {
            if ((p & 255) == 0) token.ThrowIfCancellationRequested();
            var index = vertical ? p * width + boundary : boundary * width + p;
            var step = vertical ? 1 : width;
            var a = source[index - step * 2]; var b = source[index - step]; var c = source[index]; var d = source[index + step];
            if (!Finite(a) || !Finite(b) || !Finite(c) || !Finite(d)) continue;
            var residual = (c - b) - ((b - a) + (d - c)) * .5f;
            if (Math.Abs(residual) > 10 && Math.Abs(b - a) < 2 && Math.Abs(d - c) < 2) jumps.Add(residual);
        }
        if (jumps.Count < (vertical ? height : width) * .75) return 0;
        // Even a large coherent step may be a real cliff. The tile boundary alone is not evidence
        // of corruption: retain it rather than blending or rejecting a legitimate landform.
        return 1;
    }
    private static bool Inside(int x, int y, int delta, int distance, int width, int height) =>
        delta == 1 ? x + distance >= 0 && x + distance < width : y + distance >= 0 && y + distance < height;
    public static bool Finite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);

    public sealed record HydrologyCorrection(int Index, float TargetElevation, string Reason);
    public sealed record HydrologyResult(int Changed, float MaximumChange, double MeanAbsoluteChange);
    public static HydrologyResult ApplyHydrologyCorrections(float[] original, float[] working, bool[] verifiedCorridor,
        bool[] protectedRim, IReadOnlyList<HydrologyCorrection> proposals, CancellationToken token)
    {
        if (original.Length != working.Length || original.Length != verifiedCorridor.Length || original.Length != protectedRim.Length)
            throw new ArgumentException("Hydrology correction masks do not match the source raster.");
        // Explicit geography only. No watershed discovery and no corrections without a bounded mapped corridor.
        var validated = new SortedDictionary<int, float>();
        foreach (var proposal in proposals)
        {
            token.ThrowIfCancellationRequested();
            var i = proposal.Index;
            if (i < 0 || i >= working.Length || !verifiedCorridor[i] || protectedRim[i] || !Finite(original[i]) || !Finite(working[i]) || !Finite(proposal.TargetElevation))
                throw new InvalidOperationException("Hydrology correction: invalid or unverified corridor/rim proposal.");
            var change = proposal.TargetElevation - original[i];
            if (change > .001f || change < -3f || Math.Abs(proposal.TargetElevation - working[i]) > 3f)
                throw new InvalidOperationException("Hydrology correction exceeds the separate 3 m lowering limit; no canyon carving.");
            if (validated.TryGetValue(i, out var value) && Math.Abs(value - proposal.TargetElevation) > .001f)
                throw new InvalidOperationException("Hydrology correction: conflicting proposals at one source sample.");
            validated[i] = proposal.TargetElevation;
        }
        if (validated.Count > Math.Max(4, working.Length / 1000)) throw new InvalidOperationException("Hydrology correction area exceeds the 0.1% source-raster budget.");
        token.ThrowIfCancellationRequested();
        var maximum = 0f; var sum = 0d;
        foreach (var item in validated)
        {
            var difference = Math.Abs(item.Value - original[item.Key]); maximum = Math.Max(maximum, difference); sum += difference;
            working[item.Key] = item.Value;
        }
        return new HydrologyResult(validated.Count, maximum, validated.Count == 0 ? 0 : sum / validated.Count);
    }
}
