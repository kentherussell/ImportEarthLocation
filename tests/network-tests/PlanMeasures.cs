using ImportEarthLocation.Processing;
using Newtonsoft.Json.Linq;
using Unity.Mathematics;

// One road edge as the measures see it, whether planned (a prepared section) or built
// (a live edge read back from the game). Structure is "ground", "bridge" or "tunnel".
internal sealed record MeasureEdge(string Start, string End, float3[] Points, string Structure, string Asset, bool OneWay);

// The measures of report 112 (11.4, A1), computed the same way for the gate's replays
// and for both analysis tools, so a plan and the game's result are judged alike.
// Distances are metres in map coordinates; heightmaps are square corner grids over
// the map width, heights in metres.
internal static class PlanMeasures
{
    // Junctions closer than this are drawn as back-to-back crossings (report 112, R9).
    public const float CloseJunction = 15;

    // Depth past which a road is a bridge or tunnel rather than an earthwork (report 112, 10.2).
    public const float StructureDepth = 12;

    public static float Length(float3[] points)
    {
        var length = 0f;

        for (var i = 1; i < points.Length; i++)
            length += math.distance(points[i - 1].xz, points[i].xz);

        return length;
    }

    public static JObject Roads(IReadOnlyList<MeasureEdge> roads)
    {
        var arms = new Dictionary<string, List<(MeasureEdge Edge, bool AtStart)>>();
        var position = new Dictionary<string, float3>();

        foreach (var edge in roads)
            foreach (var (node, atStart) in new[] { (edge.Start, true), (edge.End, false) })
            {
                if (!arms.TryGetValue(node, out var list))
                    arms[node] = list = new();

                list.Add((edge, atStart));
                position[node] = atStart ? edge.Points[0] : edge.Points[^1];
            }

        var kilometres = roads.Sum(edge => Length(edge.Points)) / 1000;
        var per100Km = (double count) => Math.Round(count / Math.Max(.001, kilometres) * 100, 2);

        // Close junction pairs, by what a merge could do with them: joined by one edge,
        // joined through one middle node, or apart; level (within 1 m) or stacked.
        var junctions = arms.Where(entry => entry.Value.Count >= 3).Select(entry => entry.Key).ToArray();
        var cells = junctions
            .GroupBy(node => ((int)Math.Floor(position[node].x / CloseJunction), (int)Math.Floor(position[node].z / CloseJunction)))
            .ToDictionary(group => group.Key, group => group.ToArray());

        var neighbours = arms.ToDictionary(
            entry => entry.Key,
            entry => new HashSet<string>(entry.Value.Select(arm => arm.AtStart ? arm.Edge.End : arm.Edge.Start)));

        var pairs = new Dictionary<string, int>();
        var inPair = new HashSet<string>();

        foreach (var node in junctions)
        {
            var p = position[node];
            var cx = (int)Math.Floor(p.x / CloseJunction);
            var cz = (int)Math.Floor(p.z / CloseJunction);

            for (var dz = -1; dz <= 1; dz++)
                for (var dx = -1; dx <= 1; dx++)
                {
                    if (!cells.TryGetValue((cx + dx, cz + dz), out var near))
                        continue;

                    foreach (var other in near)
                    {
                        if (string.CompareOrdinal(other, node) <= 0 || math.distance(position[other].xz, p.xz) >= CloseJunction)
                            continue;

                        var link =
                            neighbours[node].Contains(other)
                                ? "joined"
                                : neighbours[node].Overlaps(neighbours[other]) ? "twoSteps" : "apart";

                        var level = Math.Abs(position[other].y - p.y) < 1 ? "level" : "stacked";
                        var label = link + "-" + level;
                        pairs[label] = pairs.GetValueOrDefault(label) + 1;
                        inPair.Add(node);
                        inPair.Add(other);
                    }
                }
        }

        // Through nodes: two arms. Which of them the game combines isn't predictable from
        // the plan: on Cairo 84% join two sides with the same asset and structure under a
        // 5 degree turn, yet the game combined 12% (report 112, section 12). Only the
        // game's readback (--game-adjustments) says which were combined.
        var through = arms.Count(entry => entry.Value.Count == 2);

        var result = new JObject
        {
            ["roadKm"] = Math.Round(kilometres, 1),
            ["edges"] = roads.Count,
            ["junctions"] = junctions.Length,
            ["junctionsInClosePairShare"] = Math.Round(inPair.Count / (double)Math.Max(1, junctions.Length), 4),
            ["edgesUnder10mPer100Km"] = per100Km(roads.Count(edge => Length(edge.Points) < 10)),
            ["throughNodesPer100Km"] = per100Km(through),
        };

        var closePairs = new JObject();

        foreach (var (label, count) in pairs.OrderBy(entry => entry.Key))
            closePairs[label] = count;

        result["closeJunctionPairs"] = closePairs;

        return result;
    }

    // Terrace step: a rise over 2 m between two cells that are each flat (under 0.25 m)
    // on their far side, the staircase left when a coarse or clamped surface is upsampled.
    public static bool IsStep(float[] grid, int size, int x, int z, int dx, int dz)
    {
        float H(int i, int j) => grid[j * size + i];

        var here = H(x, z);

        return Math.Abs(H(x + dx, z + dz) - here) > 2
            && Math.Abs(H(x - dx, z - dz) - here) < .25f
            && Math.Abs(H(x + 2 * dx, z + 2 * dz) - H(x + dx, z + dz)) < .25f;
    }

    // Spike or pit: one cell more than 8 m off every one of its eight neighbours.
    public static bool IsSpike(float[] grid, int size, int x, int z)
    {
        var here = grid[z * size + x];
        var low = float.MaxValue;
        var high = float.MinValue;

        for (var dz = -1; dz <= 1; dz++)
            for (var dx = -1; dx <= 1; dx++)
                if (dx != 0 || dz != 0)
                {
                    var value = grid[(z + dz) * size + x + dx];
                    low = Math.Min(low, value);
                    high = Math.Max(high, value);
                }

        return here - high > 8 || low - here > 8;
    }

    // Every step in a grid: (x, z, direction, rise).
    public static List<(int X, int Z, int Dx, int Dz, float Rise)> Steps(float[] grid, int size)
    {
        var steps = new List<(int, int, int, int, float)>();

        for (var z = 2; z < size - 2; z++)
            for (var x = 2; x < size - 2; x++)
                foreach (var (dx, dz) in new[] { (1, 0), (0, 1) })
                    if (IsStep(grid, size, x, z, dx, dz))
                        steps.Add((x, z, dx, dz, Math.Abs(grid[(z + dz) * size + x + dx] - grid[z * size + x])));

        return steps;
    }

    // The largest rise between neighbouring cells in a 4 x 4 window around (x, z). A defect
    // is traced to an earlier grid that already had a comparable rise anywhere in the window,
    // which absorbs half-cell differences in registration between grids.
    public static float WindowRise(float[] grid, int size, int x, int z)
    {
        var largest = 0f;

        for (var wz = Math.Max(0, z - 1); wz <= Math.Min(size - 2, z + 2); wz++)
            for (var wx = Math.Max(0, x - 1); wx <= Math.Min(size - 2, x + 2); wx++)
            {
                var here = grid[wz * size + wx];
                largest = Math.Max(largest, Math.Abs(grid[wz * size + wx + 1] - here));
                largest = Math.Max(largest, Math.Abs(grid[(wz + 1) * size + wx] - here));
            }

        return largest;
    }

    // Terrain of a plan: steps and spikes in the designed grid, each traced to the grid
    // before road design ("source") or to the design; and the ground roads needing more
    // than StructureDepth, or left more than 2 m off the designed ground. The depth is
    // measured from designBaseline when given: the grid the design started from, which
    // differs from before when source cleaning ran.
    public static JObject Terrain(
        float[] before,
        float[] designed,
        double width,
        IEnumerable<float3[]> groundRoads,
        float originShare = .75f,
        float[]? designBaseline = null)
    {
        var baseline = designBaseline ?? before;
        var size = (int)Math.Sqrt(designed.Length);

        string Origin(int x, int z, bool earlier)
            => earlier || WindowRise(before, size, x, z) >= WindowRise(designed, size, x, z) * originShare
                ? "source"
                : "design";

        var designedSteps = Steps(designed, size);
        var steps = designedSteps
            .Select(step => Origin(step.X, step.Z, IsStep(before, size, step.X, step.Z, step.Dx, step.Dz)))
            .ToList();

        // Where the design's steps are: cells within one cell of a ground road's line
        // stand under a deck. A step from deck to deck lies between two roads at
        // different heights; deck to ground is the edge of an earthwork.
        var roadList = groundRoads.ToList();
        var deck = new bool[designed.Length];

        foreach (var points in roadList)
            for (var i = 1; i < points.Length; i++)
            {
                var count = (int)Math.Ceiling(math.distance(points[i - 1].xz, points[i].xz) / (width / (size - 1) * .5));

                for (var k = 0; k <= count; k++)
                {
                    var point = math.lerp(points[i - 1], points[i], k / (float)Math.Max(1, count));
                    var cx = (int)Math.Round((point.x / width + .5) * (size - 1));
                    var cz = (int)Math.Round((point.z / width + .5) * (size - 1));

                    for (var dz = -1; dz <= 1; dz++)
                        for (var dx = -1; dx <= 1; dx++)
                            if (cx + dx >= 0 && cz + dz >= 0 && cx + dx < size && cz + dz < size)
                                deck[(cz + dz) * size + cx + dx] = true;
                }
            }

        var stepsAt = new JObject { ["deckToDeck"] = 0, ["deckToGround"] = 0, ["awayFromRoads"] = 0 };

        for (var i = 0; i < designedSteps.Count; i++)
        {
            if (steps[i] != "design")
                continue;

            var step = designedSteps[i];
            var here = deck[step.Z * size + step.X];
            var there = deck[(step.Z + step.Dz) * size + step.X + step.Dx];
            var key = here && there ? "deckToDeck" : here || there ? "deckToGround" : "awayFromRoads";
            stepsAt[key] = (int)stepsAt[key]! + 1;
        }

        var spikes = new List<string>();

        for (var z = 1; z < size - 1; z++)
            for (var x = 1; x < size - 1; x++)
                if (IsSpike(designed, size, x, z))
                    spikes.Add(Origin(x, z, IsSpike(before, size, x, z)));

        double groundKm = 0;
        double deepKm = 0;
        double shortKm = 0;
        var half = width / 2;

        foreach (var points in roadList)
            for (var i = 1; i < points.Length; i++)
            {
                var point = points[i];

                if (Math.Abs(point.x) > half || Math.Abs(point.z) > half)
                    continue;

                var segment = math.distance(points[i - 1].xz, point.xz) / 1000;
                groundKm += segment;

                if (Math.Abs(point.y - NetworkProcessor.Sample(point.x, point.z, baseline, width)) > StructureDepth)
                    deepKm += segment;

                if (Math.Abs(point.y - NetworkProcessor.Sample(point.x, point.z, designed, width)) > 2)
                    shortKm += segment;
            }

        JObject ByOrigin(List<string> items)
        {
            var result = new JObject { ["count"] = items.Count };

            foreach (var group in items.GroupBy(item => item).OrderBy(group => group.Key))
                result[group.Key] = group.Count();

            return result;
        }

        return new JObject
        {
            ["steps"] = ByOrigin(steps),
            ["designStepsAt"] = stepsAt,
            ["spikes"] = ByOrigin(spikes),
            ["groundRoadKm"] = Math.Round(groundKm, 1),
            ["groundKmNeedingOver12m"] = Math.Round(deepKm, 2),
            ["groundKmStoppedShort"] = Math.Round(shortKm, 2),
        };
    }
}
