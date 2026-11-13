using System;
using System.Collections.Generic;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Source cleaning (report 112, 11.4 B1). The elevation tiles are a surface model in
// many cities, so building blocks, tree canopy and elevated structures stand in them
// as bumps; upsampled, their edges become the terrace steps found before road design.
//
// A grey-scale opening (a running minimum, then a running maximum, over a window)
// removes every bump narrower than the window and never raises a cell, so rivers and
// valleys are untouched. A square window would also shave narrow ridge crests, so
// away from structures the opening is taken along four lines (rows, columns and both
// diagonals) and the highest of the four kept: a ridge survives the line that runs
// along it, while a compact block is removed by all four. Within StructureReach of a
// bridge or raised road or rail, which is itself long and narrow, the square opening
// is used so a baked-in viaduct goes too.
//
// Cleaning is applied near the road and rail network only (the built-up proxy the
// design capture can reproduce), never within WaterMargin of mapped water, quarries,
// waterways, cliffs or coast, and lowers a cell by at most MaximumLowering. Its weight
// is the share of near-network cells in a window around each cell, so it fades in
// over one window instead of starting at a step.
//
// Heights are metres (TerrainBuilder.Result.Heights). The grid is a square of corner
// samples across the map width, centred on the map.
public static class TerrainCleaning
{
    public const float WindowMeters = 60;
    public const float MaximumLowering = 20;
    public const float NearNetwork = 60;
    public const float WaterMargin = 45;
    public const float StructureReach = 30;

    public static float[] Apply(
        float[] heights,
        double width,
        IReadOnlyList<PreparedSegment> network,
        IEnumerable<float3[]> protectedAreas,
        IEnumerable<float3[]> protectedLines,
        ImportLog log,
        CancellationToken token,
        float window = WindowMeters)
    {
        try
        {
            return Clean(heights, width, network, protectedAreas, protectedLines, log, token, window);
        }
        catch (Exception exception) when (exception is not OperationCanceledException)
        {
            log.Warn("Source cleaning skipped: " + exception.Message);
            log.Count("Design switch failures");
            return heights;
        }
    }

    private static float[] Clean(
        float[] heights,
        double width,
        IReadOnlyList<PreparedSegment> network,
        IEnumerable<float3[]> protectedAreas,
        IEnumerable<float3[]> protectedLines,
        ImportLog log,
        CancellationToken token,
        float window)
    {
        var clock = System.Diagnostics.Stopwatch.StartNew();
        var n = (int)Math.Sqrt(heights.Length);

        if (n * n != heights.Length || n < 8 || width <= 0)
            throw new InvalidOperationException("Invalid terrain grid.");

        var cell = (float)(width / (n - 1));
        var radius = Math.Max(1, (int)Math.Round(window / 2 / cell));

        int Cells(float metres) => Math.Max(1, (int)Math.Ceiling(metres / cell));

        int Pixel(float value) => (int)Math.Round((value / width + .5) * (n - 1));

        void Mark(byte[] mask, float3 point)
        {
            var x = Pixel(point.x);
            var z = Pixel(point.z);

            if (x >= 0 && z >= 0 && x < n && z < n)
                mask[z * n + x] = 1;
        }

        // Network and structures, sampled every half cell. Tunnels lie under real ground.
        var near = new byte[n * n];
        var structures = new byte[n * n];

        foreach (var segment in network)
        {
            token.ThrowIfCancellationRequested();

            if (segment.Tunnel)
                continue;

            var raised = segment.Bridge || segment.Layer > 0;
            var length = Math.Max(1, NetworkVerticalProfiles.ArcLength(segment));
            var steps = Math.Max(1, (int)Math.Ceiling(length / (cell * .5f)));

            for (var i = 0; i <= steps; i++)
            {
                var point = NetworkNormalizer.Evaluate(segment, i / (float)steps);
                Mark(near, point);

                if (raised)
                    Mark(structures, point);
            }
        }

        var kept = TerrainExtremes.Protected(n, width, protectedAreas, protectedLines);
        token.ThrowIfCancellationRequested();

        // The weight below spreads one window radius past the near-network mask, so
        // the mask stops that far short of the water margin.
        var builtUp = Dilate(near, n, Cells(NearNetwork));
        var excluded = Dilate(kept, n, Cells(WaterMargin) + radius);
        var nearCells = 0;
        var excludedCells = 0;

        for (var i = 0; i < builtUp.Length; i++)
        {
            nearCells += builtUp[i];

            if (excluded[i] != 0 && builtUp[i] != 0)
            {
                builtUp[i] = 0;
                excludedCells++;
            }
        }

        var nearStructure = Dilate(structures, n, Cells(StructureReach));
        token.ThrowIfCancellationRequested();

        var square = (float[])heights.Clone();
        Square(square, n, radius);
        token.ThrowIfCancellationRequested();

        var opened = new float[n * n];
        var diagonal = Math.Max(1, (int)Math.Round(radius / Math.Sqrt(2)));

        OpenAlong(heights, opened, n, 1, 0, radius);
        OpenAlong(heights, opened, n, 0, 1, radius);
        OpenAlong(heights, opened, n, 1, 1, diagonal);
        OpenAlong(heights, opened, n, 1, -1, diagonal);
        token.ThrowIfCancellationRequested();

        // Weight: the share of built-up cells in the window, by row counts then columns.
        var rowCounts = new byte[n * n];

        for (var z = 0; z < n; z++)
        {
            var count = 0;

            for (var x = 0; x < Math.Min(n, radius); x++)
                count += builtUp[z * n + x];

            for (var x = 0; x < n; x++)
            {
                if (x + radius < n)
                    count += builtUp[z * n + x + radius];

                if (x - radius - 1 >= 0)
                    count -= builtUp[z * n + x - radius - 1];

                rowCounts[z * n + x] = (byte)count;
            }
        }

        var result = (float[])heights.Clone();
        var area = (2 * radius + 1) * (2 * radius + 1);
        var lowered = 0;
        var largest = 0f;
        double volume = 0;

        for (var x = 0; x < n; x++)
        {
            token.ThrowIfCancellationRequested();
            var count = 0;

            for (var z = 0; z < Math.Min(n, radius); z++)
                count += rowCounts[z * n + x];

            for (var z = 0; z < n; z++)
            {
                if (z + radius < n)
                    count += rowCounts[(z + radius) * n + x];

                if (z - radius - 1 >= 0)
                    count -= rowCounts[(z - radius - 1) * n + x];

                if (count == 0)
                    continue;

                var id = z * n + x;
                var floor = nearStructure[id] != 0 ? square[id] : opened[id];

                if (floor >= heights[id])
                    continue;

                var metres = Math.Min(MaximumLowering, heights[id] - floor) * count / (float)area;
                result[id] = heights[id] - metres;
                volume += metres * cell * cell;
                largest = Math.Max(largest, metres);

                if (metres >= .5f)
                    lowered++;
            }
        }

        log.Count("Source cleaning cells lowered over 0.5 m", lowered);
        log.Evidence(
            $"Source cleaning: {lowered * cell * cell / 1e6:0.00} km² near roads lowered by more than 0.5 m, at most {largest:0.0} m, {volume / 1e6:0.0} million m³ in all; {nearCells * cell * cell / 1e6:0.0} km² near the network, {excludedCells * cell * cell / 1e6:0.0} km² of it kept for water; window {window:0} m, cell {cell:0.0} m; {clock.Elapsed.TotalSeconds:0.0} s.");

        return result;
    }

    // Binary dilation by a square of the given radius: rows, then columns.
    private static byte[] Dilate(byte[] mask, int n, int radius)
    {
        var rows = new byte[n * n];
        var result = new byte[n * n];

        for (var z = 0; z < n; z++)
            DilateLine(mask, rows, z * n, 1, n, radius);

        for (var x = 0; x < n; x++)
            DilateLine(rows, result, x, n, n, radius);

        return result;
    }

    // A cell is set when any cell within radius along the line is set: the distance
    // to the last set cell, scanned both ways.
    private static void DilateLine(byte[] source, byte[] target, int start, int stride, int count, int radius)
    {
        var since = int.MaxValue / 2;

        for (var i = 0; i < count; i++)
        {
            since = source[start + i * stride] != 0 ? 0 : since + 1;

            if (since <= radius)
                target[start + i * stride] = 1;
        }

        since = int.MaxValue / 2;

        for (var i = count - 1; i >= 0; i--)
        {
            since = source[start + i * stride] != 0 ? 0 : since + 1;

            if (since <= radius)
                target[start + i * stride] = 1;
        }
    }

    // Square opening in place: erode rows and columns, then dilate rows and columns.
    private static void Square(float[] grid, int n, int radius)
    {
        var line = new float[n];
        var output = new float[n];
        var deque = new int[n];

        foreach (var minimum in new[] { true, false })
        {
            for (var z = 0; z < n; z++)
            {
                Array.Copy(grid, z * n, line, 0, n);
                Slide(line, n, radius, minimum, output, deque);
                Array.Copy(output, 0, grid, z * n, n);
            }

            for (var x = 0; x < n; x++)
            {
                for (var z = 0; z < n; z++)
                    line[z] = grid[z * n + x];

                Slide(line, n, radius, minimum, output, deque);

                for (var z = 0; z < n; z++)
                    grid[z * n + x] = output[z];
            }
        }
    }

    // Opening along every line of direction (dx, dz), keeping the highest result so far.
    private static void OpenAlong(float[] grid, float[] best, int n, int dx, int dz, int radius)
    {
        var line = new float[n];
        var eroded = new float[n];
        var opened = new float[n];
        var deque = new int[n];
        var starts = new List<(int X, int Z)>();

        // A line starts at a border cell whose previous cell along it is off the grid.
        for (var z = 0; z < n; z++)
            for (var x = 0; x < n; x++)
            {
                if (z > 0 && z < n - 1 && x > 0 && x < n - 1)
                    continue;

                var px = x - dx;
                var pz = z - dz;

                if (px < 0 || pz < 0 || px >= n || pz >= n)
                    starts.Add((x, z));
            }

        foreach (var (x0, z0) in starts)
        {
            var count = 0;

            for (int x = x0, z = z0; x >= 0 && z >= 0 && x < n && z < n; x += dx, z += dz)
                line[count++] = grid[z * n + x];

            Slide(line, count, radius, true, eroded, deque);
            Slide(eroded, count, radius, false, opened, deque);

            var k = 0;

            for (int x = x0, z = z0; x >= 0 && z >= 0 && x < n && z < n; x += dx, z += dz, k++)
                best[z * n + x] = Math.Max(best[z * n + x], opened[k]);
        }
    }

    // Running minimum or maximum over [i - radius, i + radius], clipped at the ends,
    // in linear time with a monotonic queue of indices.
    private static void Slide(float[] input, int count, int radius, bool minimum, float[] output, int[] deque)
    {
        var head = 0;
        var tail = 0;

        for (var i = 0; i < count + radius; i++)
        {
            if (i < count)
            {
                while (tail > head
                    && (minimum ? input[deque[tail - 1]] >= input[i] : input[deque[tail - 1]] <= input[i]))
                    tail--;

                deque[tail++] = i;
            }

            var centre = i - radius;

            if (centre < 0)
                continue;

            while (deque[head] < centre - radius)
                head++;

            output[centre] = input[deque[head]];
        }
    }
}
