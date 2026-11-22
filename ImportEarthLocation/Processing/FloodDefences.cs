using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;

namespace ImportEarthLocation.Processing;

// Flood defences for land below a water surface (report 112, section 14). The game has one
// sea surface for the whole map; it fills any ground below that surface it can reach from
// the map edge. The import's lakes and rivers are held at their planned stages by water
// sources, and their water spreads the same way into any lower ground it can reach. Ground
// below a surface stays dry only behind something higher (doc 106). The elevation data is
// too coarse to carry dykes and levees whole, so Amsterdam's polders and New Orleans lie
// open to the sea and to their own lakes in our terrain.
//
// 1. Mapped defences (dykes, levees, flood walls, dams, lock gates) are raised to the
//    sea's crest, Freeboard above the sea surface, at least CrestWidth wide: the game's
//    water is simulated on a coarser grid than the terrain, and a one-cell wall leaks.
// 2. Water is flooded in from every surface: the sea from the map edge, each lake and
//    river from its own cells at its stage. It moves into any lower ground but stands
//    there no higher than that ground's spill height (where water standing on it would
//    overflow towards the map edge), so it runs on down a valley that drains and fills a
//    basin only to its rim. A cell two waters reach takes the higher. Land where water
//    stands is land it would drown (not ocean, mapped water or a planned stage). Each
//    connected piece of it, joined across its mapped ditches, of at least SmallestPolder
//    gets a dyke where it meets the
//    water's side (what the water reaches without crossing such a piece: its own cells
//    and low patches too small to defend) and along the map edge, Freeboard above the
//    level of the water it meets. Smaller pieces (a low strip of shore) are left to the
//    water, as is a piece whose dyke would stand over TallestDyke above most of its
//    shore. This repeats until no such piece is reached.
// Land behind the defences is never moved, so polders keep their own height and their
// canals their own lower water levels.
//
// Heights are metres (TerrainBuilder.Result.Heights) on a square grid of corner samples
// across the map width, rows south to north. The sea surface and the inland surfaces are
// in the same metres: the sea at the game's effective water level (WaterLevels.Effective).
public static class FloodDefences
{
    public const float Freeboard = 1;
    // The game lowers a dyke's edge cells (its grid is one cell narrower, so ours is
    // resampled between samples; report 112, 14.1). 60 m was tried and built a third more
    // dyke without keeping more of it whole.
    public const float CrestWidth = 30;
    public const byte OpenWater = 2;
    public const byte MappedWater = 1;
    private const byte StagedWater = 3;
    public const float SmallestPolder = 250_000;
    // Real dykes and levees stand well under this above the land they protect.
    public const float TallestDyke = 15;
    // Mapped water at most this wide between land on both sides, across a row or a column,
    // is a ditch and joins the land beside it. Polder ditches and drains are a few metres to
    // about 20 m wide; harbours and canals wider than this keep pieces apart, so a polder
    // never takes in their deep beds.
    public const float DitchWidth = 30;
    private const int Rounds = 4;

    // Crest: the height each defence cell was raised to (NaN where none), so earthworks
    // that cut it can be put back.
    public sealed record Outcome(
        float[] Heights,
        bool[] Defence,
        float[] Crest,
        int MappedCells,
        int BuiltCells,
        int PoldersDefended,
        int AreasTooDeep,
        int LandReachedBefore,
        int LandReachedAfter);

    // wet: OpenWater (ocean, sea) or MappedWater (mapped water and quarries; any other
    // non-zero value counts as open); mapped: cells on mapped defence lines;
    // inlandSurface: the planned water surface of each lake or river cell in metres, NaN
    // elsewhere (null for the sea alone).
    // scale: the top of the game's height encoding; no crest is built above it.
    public static Outcome Build(
        float[] heights,
        float scale,
        double width,
        float seaSurface,
        byte[] wet,
        byte[] mapped,
        CancellationToken token = default,
        float[]? inlandSurface = null)
    {
        var n = (int)Math.Sqrt(heights.Length);

        if (n * n != heights.Length || n < 8 || scale <= 0 || width <= 0)
            throw new InvalidOperationException("Invalid terrain grid.");

        float CrestFor(float level) => Math.Min(scale, level + Freeboard);

        var cell = (float)(width / (n - 1));
        var seaCrest = CrestFor(seaSurface);
        var skipped = new HashSet<int>();
        var radius = Math.Max(1, (int)Math.Ceiling(CrestWidth / cell / 2));
        var smallest = (int)Math.Ceiling(SmallestPolder / (cell * cell));
        var result = (float[])heights.Clone();
        var defence = new bool[n * n];
        var crests = new float[n * n];
        Array.Fill(crests, float.NaN);
        var water = (byte[])wet.Clone();
        var inland = inlandSurface;
        var mappedCells = 0;
        var builtCells = 0;

        // Inland surface cells are water, never land to defend.
        if (inland != null)
            for (var id = 0; id < inland.Length; id++)
                if (!float.IsNaN(inland[id]) && water[id] != OpenWater)
                    water[id] = StagedWater;

        var ditch = Ditches(water, n, (int)Math.Floor(DitchWidth / cell));

        // This round's water side (what the water reaches without crossing a piece of land to
        // be defended); null for the mapped defences, raised before any round.
        float[]? side = null;

        void Raise(int x, int z, bool[]? within, float crest)
        {
            for (var dz = -radius; dz <= radius; dz++)
                for (var dx = -radius; dx <= radius; dx++)
                {
                    int nx = x + dx, nz = z + dz;

                    if (nx < 0 || nz < 0 || nx >= n || nz >= n)
                        continue;

                    var id = nz * n + nx;

                    // The sea and ocean are never filled, nor the water being held back. Other
                    // water within the band (a polder's ditches, mapped or with their own stage)
                    // is closed as a sluice would close it: left open, each is a hole the game
                    // wears the dyke around until the water runs through. A mapped defence
                    // closes mapped water only, never a planned stage (a river across a dam).
                    var open = water[id] switch
                    {
                        0 => within != null && !within[id],
                        MappedWater => side != null && side[id] > float.NegativeInfinity,
                        StagedWater => side == null || side[id] > float.NegativeInfinity,
                        _ => true,
                    };

                    if (open)
                        continue;

                    // A cell already in a defence is raised again only by a higher water.
                    if (defence[id] && crests[id] >= crest)
                        continue;

                    if (!defence[id])
                    {
                        if (within == null)
                            mappedCells++;
                        else
                            builtCells++;
                    }

                    defence[id] = true;
                    crests[id] = crest;

                    if (result[id] < crest)
                        result[id] = crest;
                }
        }

        for (var id = 0; id < mapped.Length; id++)
            if (mapped[id] != 0)
                Raise(id % n, id / n, null, seaCrest);

        token.ThrowIfCancellationRequested();
        var landBefore = -1;
        var landAfter = 0;
        var defended = 0;

        for (var round = 0; round <= Rounds; round++)
        {
            var flooding = new Flooding(result, n, seaSurface, inland, token);
            var reached = flooding.Flood(token);
            var leaking = new bool[n * n];

            // A polder's mapped ditches (narrow water with no planned stage) belong to its
            // piece: Amsterdam's ditches every 100 to 200 m would otherwise cut each polder
            // into strips under SmallestPolder, all left to the sea (report 112, 14.8).
            for (var id = 0; id < leaking.Length; id++)
                leaking[id] = (water[id] == 0 || ditch[id]) && !defence[id] && flooding.Pools(reached[id], id);

            var pieces = Pieces(leaking, n, smallest);
            // The water's side: what the water reaches without crossing a piece of land to
            // be defended (its own cells, and low patches too small to defend). A polder's
            // own canals, reached only across its land, are not on the water's side.
            var passable = new byte[n * n];

            for (var id = 0; id < passable.Length; id++)
                passable[id] = reached[id] >= 0 ? (byte)1 : (byte)0;

            foreach (var piece in pieces)
                foreach (var id in piece)
                    passable[id] = 0;

            side = flooding.Metres(flooding.Flood(token, passable));
            landAfter = 0;

            foreach (var piece in pieces)
                foreach (var id in piece)
                    landAfter += water[id] == 0 ? 1 : 0;

            if (landBefore < 0)
                landBefore = landAfter;

            if (pieces.Count == 0 || round == Rounds)
                break;

            // A dyke where the water meets each piece: beside reached water at that water's
            // level, or on the map edge at the sea's.
            foreach (var piece in pieces)
            {
                if (skipped.Contains(piece[0]))
                    continue;

                var inside = new bool[n * n];
                var shore = new List<(int Id, float Crest)>();

                foreach (var id in piece)
                    inside[id] = true;

                foreach (var id in piece)
                {
                    int x = id % n, z = id / n;
                    var level = x == 0 || z == 0 || x == n - 1 || z == n - 1 ? seaSurface : float.NegativeInfinity;

                    foreach (var (dx, dz) in Neighbours)
                    {
                        int nx = x + dx, nz = z + dz;

                        if (nx >= 0 && nz >= 0 && nx < n && nz < n)
                            level = Math.Max(level, side[nz * n + nx]);
                    }

                    if (level > float.NegativeInfinity)
                        shore.Add((id, CrestFor(level)));
                }

                // A dyke that would have to stand TallestDyke over most of its shore holds
                // back water running down a valley, not a polder's: the piece is left. Only
                // its land counts; a ditch's bed is not where the dyke stands.
                var raises =
                    shore
                        .Where(cell => water[cell.Id] == 0)
                        .Select(cell => cell.Crest - result[cell.Id])
                        .OrderBy(raise => raise)
                        .ToArray();

                if (raises.Length == 0 || raises[raises.Length / 2] > TallestDyke)
                {
                    skipped.Add(piece[0]);
                    continue;
                }

                defended++;

                foreach (var (id, crest) in shore)
                    Raise(id % n, id / n, inside, crest);
            }
        }

        return new(result, defence, crests, mappedCells, builtCells, defended, skipped.Count, landBefore, landAfter);
    }

    // Defence cells the earthworks lowered below their crest go back up to it (a NaN
    // crest, none, never compares greater).
    public static int Restore(float[] heights, float[] crests)
    {
        var restored = 0;

        for (var id = 0; id < heights.Length; id++)
            if (heights[id] < crests[id])
            {
                heights[id] = crests[id];
                restored++;
            }

        return restored;
    }

    // Restore on a window of the map: `heights` holds `width` columns per row, its first cell
    // at column x and row z of the crests' grid, `resolution` cells square.
    public static int Restore(float[] heights, float[] crests, int x, int z, int width, int resolution)
    {
        var restored = 0;

        for (var index = 0; index < heights.Length; index++)
        {
            var id = (z + index / width) * resolution + x + index % width;

            if (heights[index] < crests[id])
            {
                heights[index] = crests[id];
                restored++;
            }
        }

        return restored;
    }

    // Land cells some water reaches: cells below the sea or an inland surface (metres, NaN
    // where none) that the water floods into and that are not themselves that water.
    public static bool[] Drowned(float[] heights, float seaSurface, float[]? inlandSurface)
    {
        var n = (int)Math.Sqrt(heights.Length);
        var flooding = new Flooding(heights, n, seaSurface, inlandSurface, default);
        var levels = flooding.Flood(default);
        var drowned = new bool[n * n];

        for (var id = 0; id < drowned.Length; id++)
            drowned[id] =
                (inlandSurface == null || float.IsNaN(inlandSurface[id]))
                    && flooding.Pools(levels[id], id);

        return drowned;
    }

    // Ground that holds water back from lower land, its height in metres (NaN elsewhere):
    // a dry cell beside standing water (the sea, or a lake or river at its stage) with dry
    // land below that water within BarrierDepth cells beyond it, straight across. Built
    // dykes are not the only such ground: a polder's own dykes in the source data are as
    // thin, and the terrain bridge must keep both whole (TerrainSampling.HeightKeepingCrests).
    public const int BarrierDepth = 3;

    public static float[] Barriers(float[] heights, float seaSurface, float[]? inlandSurface)
    {
        var n = (int)Math.Sqrt(heights.Length);
        var flooding = new Flooding(heights, n, seaSurface, inlandSurface, default);
        var levels = flooding.Flood(default);
        var ground = flooding.Ground;

        // The level water stands at on each cell, in steps (-1 where none stands).
        var standing = new int[n * n];

        for (var id = 0; id < standing.Length; id++)
            standing[id] = flooding.Pools(levels[id], id) ? Math.Min(levels[id], flooding.Spill[id]) : -1;

        var barriers = new float[n * n];
        Array.Fill(barriers, float.NaN);

        for (var z = 0; z < n; z++)
            for (var x = 0; x < n; x++)
            {
                var id = z * n + x;

                if (standing[id] >= 0)
                    continue;

                foreach (var (dx, dz) in Neighbours)
                {
                    int wx = x + dx, wz = z + dz;

                    if (wx < 0 || wz < 0 || wx >= n || wz >= n || standing[wz * n + wx] < 0)
                        continue;

                    var water = standing[wz * n + wx];

                    if (ground[id] < water)
                        continue;

                    for (var step = 1; step <= BarrierDepth; step++)
                    {
                        int bx = x - dx * step, bz = z - dz * step;

                        if (bx < 0 || bz < 0 || bx >= n || bz >= n)
                            break;

                        var beyond = bz * n + bx;

                        if (standing[beyond] < 0 && ground[beyond] < water)
                        {
                            barriers[id] = heights[id];
                            break;
                        }
                    }
                }
            }

        return barriers;
    }

    // Cells the sea reaches: from every map-edge cell below its surface, through cells
    // below its surface, four-connected (as water flows between grid cells). With
    // through, it spreads only into cells marked there.
    public static bool[] Reach(
        float[] heights,
        int n,
        float sea,
        CancellationToken token = default,
        byte[]? through = null)
    {
        var levels = new Flooding(heights, n, sea, null, token).Flood(token, through);
        var reached = new bool[n * n];

        for (var id = 0; id < reached.Length; id++)
            reached[id] = levels[id] >= 0;

        return reached;
    }

    // The two floods, on heights in whole steps. Heights stay metres everywhere else; here
    // each is a 16-bit step count above the lowest height present, steps of a centimetre
    // (wider when the heights span more than 65,536 of them). The step count is then both
    // the height and the bucket a cell is queued in, as with the game's own 16-bit codes
    // before: a cell's bucket needs no arithmetic and no second read, and the grids are
    // half the size of metres. Timed on Amsterdam, metres throughout ran about 1.6 times
    // slower for the same 30 million queue operations.
    //
    // Ground rounds down to its step and water levels round up, so ground is below a level
    // exactly when its step is lower: a cell a few millimetres under the sea still floods.
    private sealed class Flooding
    {
        private const int Steps = 65536;
        private const double SmallestStep = .01;
        private readonly int _n;
        private readonly double _low;
        private readonly double _step;
        private readonly int _sea;
        private readonly float[]? _inland;

        // Each cell's ground, and its spill height: the lowest level water standing there
        // must rise to before it can run off the map (the highest point on the lowest path
        // to the map edge, where the sea holds it at the sea's level).
        public readonly ushort[] Ground;
        public readonly ushort[] Spill;

        public Flooding(float[] heights, int n, float sea, float[]? inland, CancellationToken token)
        {
            var low = (double)sea;
            var high = (double)sea;

            foreach (var height in heights)
            {
                low = Math.Min(low, height);
                high = Math.Max(high, height);
            }

            if (inland != null)
                foreach (var level in inland)
                    if (!float.IsNaN(level))
                        high = Math.Max(high, level);

            _n = n;
            _low = low;
            _step = Math.Max(SmallestStep, (high - low) / (Steps - 1));
            _inland = inland;
            Ground = new ushort[heights.Length];
            var ground = Ground;
            var lowest = (float)_low;
            var perMetre = (float)(1 / _step);

            // Every height is at least the lowest, so truncation is the floor.
            Parallel.For(
                0,
                heights.Length,
                id => ground[id] = (ushort)Math.Min(Steps - 1, (int)((heights[id] - lowest) * perMetre)));

            _sea = LevelStep(sea);
            Spill = SpillSteps(token);
        }

        // Water stands on a cell when water reaches it (level, in steps; -1 for none) and
        // cannot drain away below that level: it pools to the lower of the level and the
        // spill height, which must be above the ground. Water flowing down a valley that
        // drains to the map edge reaches every cell below its source but pools on none.
        public bool Pools(int level, int id) => level >= 0 && Math.Min(level, Spill[id]) > Ground[id];

        // Levels in metres, negative infinity where none.
        public float[] Metres(int[] levels)
        {
            var metres = new float[levels.Length];

            for (var id = 0; id < levels.Length; id++)
                metres[id] = levels[id] < 0 ? float.NegativeInfinity : (float)(_low + levels[id] * _step);

            return metres;
        }

        private int LevelStep(float metres) => (int)Math.Min(Steps - 1, Math.Max(0, Math.Ceiling((metres - _low) / _step)));

        // A priority flood from the map edge, one bucket per step.
        private ushort[] SpillSteps(CancellationToken token)
        {
            var n = _n;
            var spill = new ushort[n * n];
            var done = new bool[n * n];
            var buckets = new List<int>?[Steps];
            var seaFloor = (ushort)_sea;

            void Push(int id, ushort level)
            {
                spill[id] = level;
                done[id] = true;
                (buckets[level] ??= new List<int>()).Add(id);
            }

            for (var i = 0; i < n; i++)
                foreach (var id in new[] { i, (n - 1) * n + i, i * n, i * n + n - 1 })
                    if (!done[id])
                        Push(id, Math.Max(Ground[id], seaFloor));

            for (var level = 0; level < buckets.Length; level++)
            {
                // Cells pushed at this level while it is processed join the same list.
                for (var index = 0; buckets[level] is { } bucket && index < bucket.Count; index++)
                {
                    var id = bucket[index];
                    int x = id % n, z = id / n;

                    foreach (var (dx, dz) in Neighbours)
                    {
                        int nx = x + dx, nz = z + dz;

                        if (nx < 0 || nz < 0 || nx >= n || nz >= n)
                            continue;

                        var other = nz * n + nx;

                        if (!done[other])
                            Push(other, Math.Max(Ground[other], (ushort)level));
                    }
                }

                buckets[level] = null;

                if ((level & 1023) == 0)
                    token.ThrowIfCancellationRequested();
            }

            return spill;
        }

        // The level, in steps, water stands at on each cell it reaches (-1 where none does).
        // Seeds: map-edge cells below the sea at the sea's level, and inland cells below their
        // own surface at that surface (their sources hold it). Water moves, four-connected,
        // into any neighbour whose ground is below its level, and stands there no higher than
        // that neighbour's spill height: it runs on down a valley that drains, and fills a
        // basin only to where the basin overflows. Each cell keeps the highest level that
        // reaches it, taken highest first (levels only fall as water moves on). Seeds ignore
        // through; it limits the spread only.
        public int[] Flood(CancellationToken token, byte[]? through = null)
        {
            var n = _n;
            var levels = new int[n * n];
            Array.Fill(levels, -1);
            var buckets = new List<int>?[Steps];

            void Arrive(int id, int level)
            {
                if (level <= levels[id])
                    return;

                levels[id] = level;
                (buckets[level] ??= new List<int>()).Add(id);
            }

            void Seed(int id, int level)
            {
                if (Ground[id] < level)
                    Arrive(id, level);
            }

            for (var i = 0; i < n; i++)
            {
                Seed(i, _sea);
                Seed((n - 1) * n + i, _sea);
                Seed(i * n, _sea);
                Seed(i * n + n - 1, _sea);
            }

            if (_inland != null)
                for (var id = 0; id < _inland.Length; id++)
                    if (!float.IsNaN(_inland[id]))
                        Seed(id, LevelStep(_inland[id]));

            for (var level = buckets.Length - 1; level >= 0; level--)
            {
                // A cell reached again at this same level joins the list being read.
                for (var index = 0; buckets[level] is { } bucket && index < bucket.Count; index++)
                {
                    var id = bucket[index];

                    // Superseded: reached again from a higher level after being queued here.
                    if (levels[id] != level)
                        continue;

                    int x = id % n, z = id / n;

                    foreach (var (dx, dz) in Neighbours)
                    {
                        int nx = x + dx, nz = z + dz;

                        if (nx < 0 || nz < 0 || nx >= n || nz >= n)
                            continue;

                        var other = nz * n + nx;

                        if (Ground[other] >= level || through != null && through[other] == 0)
                            continue;

                        Arrive(other, Math.Min(level, (int)Spill[other]));
                    }
                }

                buckets[level] = null;

                if ((level & 1023) == 0)
                    token.ThrowIfCancellationRequested();
            }

            return levels;
        }
    }

    // Four-connected pieces of the mask with at least smallest cells.
    // Mapped water cells in a run of mapped water at most widest cells long, along a row or a
    // column, with land (no water) at both ends.
    private static bool[] Ditches(byte[] water, int n, int widest)
    {
        var ditch = new bool[n * n];
        widest = Math.Max(1, widest);

        void Scan(int first, int step)
        {
            var start = -1;

            for (var i = 0; i < n; i++)
            {
                var id = first + i * step;

                if (water[id] == MappedWater)
                {
                    if (start < 0)
                        start = i;

                    continue;
                }

                var bounded = start > 0 && water[id] == 0 && water[first + (start - 1) * step] == 0;

                if (bounded && i - start <= widest)
                    for (var j = start; j < i; j++)
                        ditch[first + j * step] = true;

                start = -1;
            }
        }

        for (var line = 0; line < n; line++)
        {
            Scan(line * n, 1);
            Scan(line, n);
        }

        return ditch;
    }

    private static List<List<int>> Pieces(bool[] mask, int n, int smallest)
    {
        var seen = new bool[mask.Length];
        var pieces = new List<List<int>>();
        var pending = new List<int>();

        for (var start = 0; start < mask.Length; start++)
        {
            if (!mask[start] || seen[start])
                continue;

            var piece = new List<int>();
            seen[start] = true;
            pending.Add(start);

            while (pending.Count > 0)
            {
                var id = pending[^1];
                pending.RemoveAt(pending.Count - 1);
                piece.Add(id);
                int x = id % n, z = id / n;

                foreach (var (dx, dz) in Neighbours)
                {
                    int nx = x + dx, nz = z + dz;

                    if (nx < 0 || nz < 0 || nx >= n || nz >= n)
                        continue;

                    var other = nz * n + nx;

                    if (mask[other] && !seen[other])
                    {
                        seen[other] = true;
                        pending.Add(other);
                    }
                }
            }

            if (piece.Count >= smallest)
                pieces.Add(piece);
        }

        return pieces;
    }

    private static readonly (int, int)[] Neighbours = { (1, 0), (-1, 0), (0, 1), (0, -1) };
}
