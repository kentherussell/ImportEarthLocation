#if !SURFACE_SMOKE_ONLY
using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

static class WaterContainmentTests
{
    public static void Run()
    {
        const int n = 81;
        const double width = 400;
        const float scale = 100;
        var polygon =
            new GeographicPolygon(100, new Dictionary<string, string> { { "natural", "water" }, { "water", "pond" } },
            new() {
                new[] { new float3(-60, 0, -60), new float3(60, 0, -60), new float3(60, 0, 60), new float3(-60, 0, 60) }
                },
                new());

        TerrainBuilder.Result Terrain(float? escape)
        {
            var heights = new float[n * n];

            for (var z = 0; z < n; z++)
                for (var x = 0; x < n; x++)
                {
                    var px = (x / (double)(n - 1) - .5) * width;
                    var pz = (z / (double)(n - 1) - .5) * width;
                    var height = Math.Abs(px) <= 60 && Math.Abs(pz) <= 60 ? 45f : 55f;

                    if (escape.HasValue && px >= 55 && Math.Abs(pz) <= 10)
                        height = escape.Value;

                    heights[z * n + x] = height;
                }

            return new(heights, scale, 0, 0, 100);
        }

        var source =
            new RiverSourcePlan(
                100,
                "pond",
                "fixture",
                new(0, 45, 0),
                14,
                5,
                55,
                new(0, 45, 0))
            { BodyId = 100, BodyKind = "lake", TargetSurfaceLevel = 50 };

        var plan =
            new WaterPlan(
                false,
                0,
                1,
                0,
                "fixture")
            {
                WaterCellSize = 14,
                Sources = new[] { source },
                AreaStages = new[] { new WaterAreaStage(100, "lake", 50) }
            };

        WaterPlan Apply(TerrainBuilder.Result terrain, WaterPlan? input = null)
            => WaterContainment.Validate(input ?? plan, terrain, width, new[] { polygon }, new ImportLog(), default);

        var closed = Terrain(null);
        var before = closed.Heights.ToArray();
        var valid = Apply(closed);
        Check(
            valid.Sources.Count == 1
                && valid.Sources[0].TargetSurfaceLevel == 50
                && closed.Heights.SequenceEqual(before),
            "A closed pond was changed or containment modified terrain");

        var broken = Apply(Terrain(40));
        Check(
            broken.Sources.Count == 0 && broken.AreaStages.Count == 0,
            "An open downhill escape retained its flooding source or canonical stage");

        var narrowRim = Terrain(40);
        var narrowHeights = narrowRim.Heights.ToArray();

        for (var z = 0; z < n; z++)
        {
            var pz = z * 5 - 200;

            if (Math.Abs(pz) <= 10)
                narrowHeights[z * n + 52] = 55f;
        }

        Check(Apply(narrowRim with { Heights = narrowHeights }).Sources.Count == 0,
            "A single DEM ridge narrower than the fluid grid concealed a downhill lake escape");

        var lowered = Apply(Terrain(49.5f));
        Check(
            lowered.Sources.Count == 1
                && lowered.Sources[0].TargetSurfaceLevel < 49.5f
                && lowered.AreaStages[0].Level == lowered.Sources[0].TargetSurfaceLevel,
            "A viable lower pond level was not shared by controls and canonical surface evidence");

        var outlet =
            plan with
            {
                ReachStages = new[] { new WaterReachStage(20, 30, new(0, 0, 0), new(200, 0, 0), 50, 50, 20, null) }
            };

        Check(
            Apply(Terrain(48), outlet).Sources.Count == 1,
            "A mapped flowing outlet was treated as an unintended land spill");

        var blocked =
            outlet with
            {
                ReachStages = outlet.ReachStages.Select(reach => reach with { BlockedByBuriedInlet = true }).ToArray()
            };

        Check(
            Apply(Terrain(40), blocked).Sources.Count == 0,
            "An unsimulated buried inlet was accepted as a lake's safe flowing outlet");

        var neighbour =
            new GeographicPolygon(
                200,
                polygon.Tags,
                new() {
                    new[] {
                        new float3(60, 0, -60),
                        new float3(200, 0, -60),
                        new float3(200, 0, 60),
                        new float3(60, 0, 60)
                        }
                    },
                new());

        var adjacent =
            plan with { AreaStages = plan.AreaStages.Concat(new[] { new WaterAreaStage(200, "lake", 50) }).ToArray() };

        var checkedAdjacent =
            WaterContainment.Validate(
                adjacent,
                Terrain(40),
                width,
                new[] { polygon, neighbour },
                new ImportLog(),
                default);

        Check(checkedAdjacent.Sources.Count == 0, "A neighbouring unverified lake concealed a downhill escape");
        // A second wet component of one mapped basin has no regulator. A high
        // DEM strip between it and the regulator must not conceal its spill.
        var splitTerrain = Terrain(40);
        var splitHeights = splitTerrain.Heights.ToArray();

        for (var z = 0; z < n; z++)
            for (var x = 0; x < n; x++)
            {
                var px = x * 5 - 200;
                var pz = z * 5 - 200;

                if (px >= 15 && px <= 25 && Math.Abs(pz) <= 60)
                    splitHeights[z * n + x] = 60f;
            }

        Check(
            Apply(splitTerrain with { Heights = splitHeights }).Sources.Count == 0,
            "A high interior ridge concealed an unsourced downhill part of the lake");

        var island =
            polygon with
            {
                Inner =
                    new() {
                        new[] {
                            new float3(20, 0, -5),
                            new float3(30, 0, -5),
                            new float3(30, 0, 5),
                            new float3(20, 0, 5)
                            }
                        }
            };

        var islandResult =
            WaterContainment.Validate(plan, Terrain(null), width, new[] { island }, new ImportLog(), default);

        Check(
            islandResult.Sources.Count == 0,
            "The shoreline collar concealed a submerged mapped island smaller than a water cell");

        Check(
            WaterContainment.Validate(
                outlet,
                Terrain(null),
                width,
                new[] { island },
                new ImportLog(),
                default).Sources.Count
                == 0,
            "A flowing reach concealed a submerged mapped island");

        LargeLake();

        using var cancellation = new CancellationTokenSource();
        cancellation.Cancel();

        try
        {
            WaterContainment.Validate(plan, closed, width, new[] { polygon }, new ImportLog(), cancellation.Token);
            throw new Exception("Containment ignored cancellation");
        }
        catch (OperationCanceledException) { }

        Console.WriteLine(
            "PASS: whole-basin escape saddle, safe lower shared level, unsafe source omission, intended river outlet, unchanged terrain, too-large lakes and cancellation.");
    }

    // A lake larger than the bounded screen keeps its planned level when that level is above
    // the sea (Lac Saint-Louis, Montreal); below the sea it is still omitted (Amsterdam's
    // Markermeer, among polders).
    static void LargeLake()
    {
        const int n = 2049;
        const double width = 57344;
        var heights = new float[n * n];

        for (var z = 0; z < n; z++)
            for (var x = 0; x < n; x++)
            {
                var px = (x / (double)(n - 1) - .5) * width;
                var pz = (z / (double)(n - 1) - .5) * width;
                heights[z * n + x] = Math.Abs(px) <= 20000 && Math.Abs(pz) <= 20000 ? 45f : 55f;
            }

        var lake =
            new GeographicPolygon(
                300,
                new Dictionary<string, string> { { "natural", "water" }, { "water", "lake" } },
                new() {
                    new[] {
                        new float3(-20000, 0, -20000),
                        new float3(20000, 0, -20000),
                        new float3(20000, 0, 20000),
                        new float3(-20000, 0, 20000)
                        }
                    },
                new());

        var source =
            new RiverSourcePlan(300, "lake", "fixture", new(0, 45, 0), 25, 5, 55, new(0, 45, 0))
            {
                BodyId = 300,
                BodyKind = "lake",
                TargetSurfaceLevel = 50
            };

        var plan =
            new WaterPlan(false, 0, 1, 0, "fixture")
            {
                WaterCellSize = 14,
                Sources = new[] { source },
                AreaStages = new[] { new WaterAreaStage(300, "lake", 50) }
            };

        var log = new ImportLog();
        var aboveSea =
            WaterContainment.Validate(plan, new(heights, 100, 0, 0, 100), width, new[] { lake }, log, default);

        Check(
            aboveSea.Sources.Count == 1
                && aboveSea.Sources[0].TargetSurfaceLevel == 50
                && log.Metric("Lake containment bodies too large to screen") == 1,
            "A lake too large to screen lost its controls");

        var belowSea =
            WaterContainment.Validate(
                plan,
                new(heights, 100, 60, 0, 100),
                width,
                new[] { lake },
                new ImportLog(),
                default);

        Check(
            belowSea.Sources.Count == 0 && belowSea.AreaStages.Count == 0,
            "A lake too large to screen, planned below the sea, kept its controls");
    }
}
#endif
