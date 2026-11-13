using System;
using System.Collections.Generic;
using System.Threading;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Terrain extremes (report 112, section 13): rises between neighbouring cells that are
// not real terrain, and single-cell spikes. This class works on the grid alone; the
// road network reaches it only as masks (NetworkTerrainExtremes), so it is tested on
// terrain without roads.
//
// Two kinds of rise over MaxRise are extremes:
// - a terrace step: between two cells that are each flat on their far side (the shape
//   left by upsampled elevation data, levelled areas and earthwork edges);
// - a wall: from a deck cell (ground the game reads under a road) to a cell beside it.
// A slope is not an extreme, so mountains keep their shape. Rises at water and on
// waterway, cliff and coast lines are real edges and are kept.
//
// Each extreme up to MaxRise * MaxRamp (12 m) gets a band of cells around it,
// ceil(rise / MaxRise) cells wide; larger rises are kept. Inside the band the ground is replaced by the smooth surface between the
// band's edges (a harmonic fill, solved by over-relaxed Gauss-Seidel), which turns a
// step into a ramp. Fixed cells form part of the edge and never move.
//
// Heights are metres (TerrainBuilder.Result.Heights) on a square grid of corner samples
// across the map width, centred on the map.
public static class TerrainExtremes
{
    public const float MaxRise = 2;
    public const int MaxRamp = 6;
    public const float SpikeHeight = 8;
    private const float Flat = .25f;
    private const int Sweeps = 80;
    private const float Relaxation = 1.8f;

    public sealed record Outcome(float[] Heights, int Terraces, int Walls, int Moved, float LargestMetres);

    // fixedCells: never moved (decks, tunnels, water banks); deck: cells under ground
    // roads, whose rise to a free neighbour is a wall; water and lines: real edges (see
    // Protected). Fixed cells within two cells of water or a line are added here.
    public static Outcome Smooth(
        float[] heights,
        bool[]? fixedCells,
        bool[]? deck,
        byte[] water,
        byte[] lines,
        CancellationToken token = default)
    {
        var n = (int)Math.Sqrt(heights.Length);

        if (n * n != heights.Length || n < 8)
            throw new InvalidOperationException("Invalid terrain grid.");

        var held = fixedCells != null ? (bool[])fixedCells.Clone() : new bool[n * n];
        deck ??= new bool[n * n];

        for (var z = 0; z < n; z++)
            for (var x = 0; x < n; x++)
            {
                var id = z * n + x;

                if (deck[id])
                    held[id] = true;

                if (water[id] == 0 && lines[id] == 0)
                    continue;

                for (var dz = -2; dz <= 2; dz++)
                    for (var dx = -2; dx <= 2; dx++)
                        if (x + dx >= 0 && z + dz >= 0 && x + dx < n && z + dz < n)
                            held[(z + dz) * n + x + dx] = true;
            }

        token.ThrowIfCancellationRequested();

        float H(int x, int z) => heights[z * n + x];

        // A cell of a band may move by at most the largest rise that marked it (metres;
        // zero outside every band), so a ramp never reshapes the hill around it.
        var band = new float[n * n];
        var terraces = 0;
        var walls = 0;

        void Mark(int x, int z, float rise)
        {
            var reach = Math.Min(MaxRamp, (int)Math.Ceiling(rise / MaxRise));

            for (var dz = -reach; dz <= reach; dz++)
                for (var dx = -reach; dx <= reach; dx++)
                    if (x + dx >= 0 && z + dz >= 0 && x + dx < n && z + dz < n)
                    {
                        var id = (z + dz) * n + x + dx;
                        band[id] = Math.Max(band[id], rise);
                    }
        }

        for (var z = 2; z < n - 2; z++)
            for (var x = 2; x < n - 2; x++)
                foreach (var (dx, dz) in Directions)
                {
                    var here = H(x, z);
                    var there = H(x + dx, z + dz);
                    var rise = Math.Abs(there - here);

                    // Over MaxRise * MaxRamp the ramp could not bring it under MaxRise, and
                    // such rises are mostly real (terraced fields, plateau edges): kept.
                    if (rise <= MaxRise || rise > MaxRise * MaxRamp)
                        continue;

                    var a = z * n + x;
                    var b = (z + dz) * n + x + dx;

                    // Both cells held: nothing here may move.
                    if (held[a] && held[b])
                        continue;

                    if (water[a] != 0 || water[b] != 0 || lines[a] != 0 || lines[b] != 0)
                        continue;

                    var terrace =
                        Math.Abs(H(x - dx, z - dz) - here) < Flat
                        && Math.Abs(H(x + 2 * dx, z + 2 * dz) - there) < Flat;

                    var wall = deck[a] != deck[b];

                    if (!terrace && !wall)
                        continue;

                    if (terrace)
                        terraces++;
                    else
                        walls++;

                    Mark(x, z, rise);
                    Mark(x + dx, z + dz, rise);
                }

        // The free cells of the band, with each one's four neighbours: another free cell
        // (by index) or a fixed height on the band's edge.
        var index = new Dictionary<int, int>();
        var ids = new List<int>();

        for (var id = 0; id < band.Length; id++)
            if (band[id] > 0 && !held[id])
            {
                index[id] = ids.Count;
                ids.Add(id);
            }

        var count = ids.Count;
        var values = new float[count];
        var neighbour = new int[count * 4];
        var edge = new float[count * 4];

        for (var k = 0; k < count; k++)
        {
            var id = ids[k];
            int x = id % n, z = id / n;
            values[k] = heights[id];
            var j = 0;

            foreach (var (dx, dz) in Neighbours)
            {
                // Off the grid: the cell itself, so the map edge adds no pull.
                var other = Math.Min(n - 1, Math.Max(0, z + dz)) * n + Math.Min(n - 1, Math.Max(0, x + dx));

                if (index.TryGetValue(other, out var free))
                    neighbour[k * 4 + j] = free;
                else
                {
                    neighbour[k * 4 + j] = -1;
                    edge[k * 4 + j] = heights[other];
                }

                j++;
            }
        }

        for (var sweep = 0; sweep < Sweeps; sweep++)
        {
            token.ThrowIfCancellationRequested();

            for (var k = 0; k < count; k++)
            {
                var sum = 0f;

                for (var j = 0; j < 4; j++)
                {
                    var other = neighbour[k * 4 + j];
                    sum += other >= 0 ? values[other] : edge[k * 4 + j];
                }

                values[k] += Relaxation * (sum / 4 - values[k]);
            }
        }

        var result = (float[])heights.Clone();
        var moved = 0;
        var largest = 0f;

        for (var k = 0; k < count; k++)
        {
            var original = heights[ids[k]];
            var limit = band[ids[k]];
            var value = math.clamp(values[k], original - limit, original + limit);
            var change = Math.Abs(value - original);

            if (change < .05f)
                continue;

            result[ids[k]] = value;
            moved++;
            largest = Math.Max(largest, change);
        }

        return new(result, terraces, walls, moved, largest);
    }

    // Single-cell spikes and pits: a cell more than SpikeHeight off every one of its
    // eight neighbours takes their median. Cells within two cells of water or a mapped
    // edge (see Protected) are kept, and so are cells below the sea surface (metres, in
    // the grid's frame): a seabed cell beside a cliff is not a pit, and lifting one stood
    // 85 m out of Marseille's sea (9 Nov).
    public static (float[] Heights, int Removed) RemoveSpikes(
        float[] heights,
        byte[]? water = null,
        byte[]? lines = null,
        float seaSurface = float.NegativeInfinity)
    {
        var n = (int)Math.Sqrt(heights.Length);
        var result = (float[])heights.Clone();
        var around = new float[8];
        var removed = 0;

        for (var z = 1; z < n - 1; z++)
            for (var x = 1; x < n - 1; x++)
            {
                var here = heights[z * n + x];
                var k = 0;
                var low = float.PositiveInfinity;
                var high = float.NegativeInfinity;

                for (var dz = -1; dz <= 1; dz++)
                    for (var dx = -1; dx <= 1; dx++)
                        if (dx != 0 || dz != 0)
                        {
                            var value = heights[(z + dz) * n + x + dx];
                            around[k++] = value;
                            low = Math.Min(low, value);
                            high = Math.Max(high, value);
                        }

                if (here - high <= SpikeHeight && low - here <= SpikeHeight)
                    continue;

                if (here < seaSurface || NearEdge(water, lines, n, x, z))
                    continue;

                Array.Sort(around);
                result[z * n + x] = (around[3] + around[4]) / 2;
                removed++;
            }

        return (result, removed);
    }

    // Cells of mapped water and quarry interiors (areas), or on waterway, cliff and coast
    // lines (lines), as the roadbed protects them.
    public static byte[] Protected(
        int n,
        double width,
        IEnumerable<float3[]> protectedAreas,
        IEnumerable<float3[]> protectedLines)
    {
        var cell = (float)(width / (n - 1));
        var kept = new byte[n * n];

        int Pixel(float value) => (int)Math.Round((value / width + .5) * (n - 1));

        void MarkLine(float3 a, float3 b)
        {
            var steps = Math.Max(1, (int)Math.Ceiling(math.distance(a.xz, b.xz) / (cell * .5f)));

            for (var i = 0; i <= steps; i++)
            {
                var point = math.lerp(a, b, i / (float)steps);
                var x = Pixel(point.x);
                var z = Pixel(point.z);

                if (x >= 0 && z >= 0 && x < n && z < n)
                    kept[z * n + x] = 1;
            }
        }

        foreach (var ring in protectedAreas)
        {
            if (ring.Length < 3)
                continue;

            FillPolygon(kept, n, ring, Pixel, width);

            for (var i = 0; i < ring.Length; i++)
                MarkLine(ring[i], ring[(i + 1) % ring.Length]);
        }

        foreach (var line in protectedLines)
            for (var i = 1; i < line.Length; i++)
                MarkLine(line[i - 1], line[i]);

        return kept;
    }

    private static bool NearEdge(byte[]? water, byte[]? lines, int n, int x, int z)
    {
        for (var dz = -2; dz <= 2; dz++)
            for (var dx = -2; dx <= 2; dx++)
            {
                var nx = x + dx;
                var nz = z + dz;

                if (nx < 0 || nz < 0 || nx >= n || nz >= n)
                    continue;

                if (water?[nz * n + nx] > 0 || lines?[nz * n + nx] > 0)
                    return true;
            }

        return false;
    }

    private static readonly (int, int)[] Directions = { (1, 0), (0, 1) };
    private static readonly (int, int)[] Neighbours = { (1, 0), (-1, 0), (0, 1), (0, -1) };

    private static void FillPolygon(byte[] mask, int n, float3[] ring, Func<float, int> pixel, double width)
    {
        var crossings = new List<float>(ring.Length);
        var low = int.MaxValue;
        var high = int.MinValue;

        foreach (var point in ring)
        {
            low = Math.Min(low, pixel(point.z));
            high = Math.Max(high, pixel(point.z));
        }

        for (var z = Math.Max(0, low); z <= Math.Min(n - 1, high); z++)
        {
            var y = (float)((z / (double)(n - 1) - .5) * width);
            crossings.Clear();

            for (var i = 0; i < ring.Length; i++)
            {
                var a = ring[i];
                var b = ring[(i + 1) % ring.Length];

                if ((a.z > y) == (b.z > y))
                    continue;

                crossings.Add(a.x + (y - a.z) * (b.x - a.x) / (b.z - a.z));
            }

            crossings.Sort();

            for (var i = 1; i < crossings.Count; i += 2)
                for (var x = Math.Max(0, pixel(crossings[i - 1])); x <= Math.Min(n - 1, pixel(crossings[i])); x++)
                    mask[z * n + x] = 1;
        }
    }
}
