#if !SURFACE_SMOKE_ONLY
using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

// The raise-banks rules (report 112, 14.4): a lake stands at the elevation data's
// flat reading over its water, held by shore levees; a river stretch at the sea's
// level is left to the sea without a source.
static class RaiseBanksTests
{
    // 64 × 64 samples over 1000 m; heights in metres over a 100 m scale.
    private const int Size = 64;
    private const float Width = 1000;
    private const float Scale = 100;
    private const float LakeHalf = 200;

    public static void Run()
    {
        var previous = DesignSwitches.Active;

        try
        {
            LakeLevels();
            SeaLevelRivers();
        }
        finally
        {
            DesignSwitches.Set(previous);
        }

        Console.WriteLine(
            "PASS: raise-banks lake levels from flat water readings (capped at 5 m, refused when uneven, off by "
                + "default), shore levees 1 m above the lake, and no river sources at the sea's level.");
    }

    private static void LakeLevels()
    {
        var lake =
            new GeographicPolygon(
                300,
                new Dictionary<string, string> { { "natural", "water" }, { "water", "lake" } },
                new() { Ring(-LakeHalf, -LakeHalf, LakeHalf, LakeHalf) },
                new());

        // Shore land at 28 m all round, so the shore rule sets 27.5 m.
        float Stage(Func<float, float, float> water, string switches, out float[] conditioned)
        {
            DesignSwitches.Set(switches);
            var terrain = Terrain((x, z) => Math.Abs(x) <= LakeHalf && Math.Abs(z) <= LakeHalf ? water(x, z) : 28);
            var plan = Plan(terrain, new[] { lake }, new OsmDataset());
            conditioned = plan.ConditionedHeights!;

            return plan.AreaStages.Single(stage => stage.BodyId == lake.Identity).Level;
        }

        var shoreRule = Stage((x, z) => 29.5f, "", out _);
        Check(Math.Abs(shoreRule - 27.5f) < .01f, $"Lake level without raise-banks moved: {shoreRule}");

        // Containment would lower the level to the lowest spill if a shore were left
        // without its levee (an open ring's closing side once was).
        var raised = Stage((x, z) => 29.5f, DesignSwitches.RaiseBanks, out var raisedHeights);
        Check(Math.Abs(raised - 29.5f) < .01f, $"Flat lake reading did not set the lake level: {raised}");

        // Just beyond each shore the levee stands 1 m above the lake, at least.
        foreach (var (x, z) in new[] { (LakeHalf + 12, 0f), (-LakeHalf - 12, 0f), (0f, LakeHalf + 12), (0f, -LakeHalf - 12) })
        {
            var shore = Metres(raisedHeights, x, z);
            Check(shore >= raised + .99f, $"Shore at ({x}, {z}) stands {shore:F2} m beside a {raised:F2} m lake");
        }

        // Far from the lake the land is untouched.
        Check(Math.Abs(Metres(raisedHeights, 450, 450) - 28) < .01f, "Shore levee reached land far from the lake");

        var capped = Stage((x, z) => 35, DesignSwitches.RaiseBanks, out _);
        Check(Math.Abs(capped - 32.5f) < .01f, $"Lake level rose more than 5 m above the shore rule: {capped}");

        // A reading spread over 6 m is old ground or land beside a ditch, not a surface.
        var uneven = Stage((x, z) => 30 + 6 * (x + LakeHalf) / (2 * LakeHalf), DesignSwitches.RaiseBanks, out _);
        Check(Math.Abs(uneven - 27.5f) < .01f, $"Uneven lake reading set the lake level: {uneven}");
    }

    private static void SeaLevelRivers()
    {
        // Sea level 10 m (11 m effective); land at 11.2 m puts the river's surface
        // below the sea, so a sea-bound river is held at the sea's level.
        var nodes =
            new Dictionary<long, OsmNode>
            {
                { 11, Node(11, 0, -400) },
                { 12, Node(12, 0, 0) },
                { 13, Node(13, 0, 400) },
                { 21, Node(21, -500, 420) },
                { 22, Node(22, 500, 420) }
            };

        var river = new OsmWay(1, new long[] { 11, 12, 13 }, new Dictionary<string, string> { { "waterway", "river" } });
        var coast = new OsmWay(2, new long[] { 21, 22 }, new Dictionary<string, string> { { "natural", "coastline" } });

        var channel =
            new GeographicPolygon(
                301,
                new Dictionary<string, string> { { "natural", "water" }, { "water", "river" } },
                new() { Ring(-60, -400, 60, 400) },
                new());

        var terrain = Terrain((x, z) => 11.2f) with { SeaLevelMeters = 10 };

        WaterPlan Run(params OsmWay[] ways)
        {
            DesignSwitches.Set(DesignSwitches.RaiseBanks);
            var dataset = new OsmDataset { Nodes = nodes, Ways = ways.ToDictionary(way => way.Id) };

            return Plan(terrain, new[] { channel }, dataset);
        }

        var inland = Run(river);
        Check(inland.Sources.Count > 0, "Control river without a coast received no sources");

        var seaBound = Run(river, coast);
        Check(seaBound.Sources.Count == 0, $"River at the sea's level kept {seaBound.Sources.Count} sources");
    }

    private static WaterPlan Plan(TerrainBuilder.Result terrain, GeographicPolygon[] areas, OsmDataset dataset)
    {
        var request = new ImportRequest(0, 0, new("test", "test", Width, false), false);

        return WaterBodyPlanner.Prepare(dataset, areas, request, terrain, new ImportLog(), default, 28);
    }

    private static TerrainBuilder.Result Terrain(Func<float, float, float> height)
    {
        var heights = new float[Size * Size];

        for (var z = 0; z < Size; z++)
            for (var x = 0; x < Size; x++)
            {
                var px = (x / (float)(Size - 1) - .5f) * Width;
                var pz = (z / (float)(Size - 1) - .5f) * Width;
                heights[z * Size + x] = height(px, pz);
            }

        return new(heights, Scale, 0, 0, Scale);
    }

    // Height in metres at the sample nearest a point.
    private static float Metres(float[] heights, float x, float z)
    {
        var column = (int)Math.Round((x / Width + .5f) * (Size - 1));
        var row = (int)Math.Round((z / Width + .5f) * (Size - 1));

        return heights[row * Size + column];
    }

    private static float3[] Ring(float left, float bottom, float right, float top)
        => new[] { new float3(left, 0, bottom), new float3(right, 0, bottom), new float3(right, 0, top), new float3(left, 0, top) };

    private static OsmNode Node(long id, float x, float z)
        => new(id, z / 6378137d * 180 / Math.PI, x / 6378137d * 180 / Math.PI);
}
#endif
