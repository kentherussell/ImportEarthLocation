using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

static class WaterRoadTests
{
    public static void Run()
    {
        RoundaboutClearance();
        DryShoreJunction();
        OpenedRiverDeck();
        var terrain = Enumerable.Repeat(100f, 129 * 129).ToArray();
        var road =
            new PreparedSegment(
                1,
                1,
                2,
                new(-600, 100, 0),
                new(600, 100, 0),
                NetworkKind.Road,
                "primary",
                2,
                false,
                false,
                false,
                0)
            { NativeWidth = 12, SourceGeometry = new[] { new float3(-600, 100, 0), new float3(600, 100, 0) } };

        foreach (var depth in new[] { 1f, 6f })
        {
            float? Water(float3 point) => Math.Abs(point.x) < 150 && Math.Abs(point.z) < 100 ? 100 + depth : null;

            var selected =
                NetworkStructureIntervals.StructureIntervals(
                    new[] { road },
                    terrain,
                    1600,
                    new ImportLog(),
                    default,
                    Water);

            Check(
                selected[0].Bridge == (depth >= WaterLevels.RoadBridgeDepthMeters),
                "Water depth did not distinguish a slight ground-road lift from a bridge");

            var result = VisualNetworkCompiler.Prepare(selected, terrain, 1600, new ImportLog(), default, Water);
            var points =
                result
                    .SelectMany(
                        segment => Enumerable.Range(0, 101).Select(k => NetworkNormalizer.Evaluate(segment, k / 100f)))
                    .Where(point => Water(point).HasValue)
                    .ToArray();

            Check(
                points.Length > 0 && points.All(point => point.y > 100 + depth + .25f),
                "A designed road remains below its planned water surface");

            Check(
                result.All(
                    segment => NetworkDesignChecks.Finite(
                        new(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd))),
                "Water clearance emitted nonfinite road controls");

            var grade = result.SelectMany(segment => Enumerable.Range(0, 100).Select(k =>
            {
                var vectorA = NetworkNormalizer.Evaluate(segment, k / 100f);
                var vectorB = NetworkNormalizer.Evaluate(segment, (k + 1) / 100f);

                return Math.Abs(vectorB.y - vectorA.y) / Math.Max(.01f, math.distance(vectorA.xz, vectorB.xz));
            })).Max();

            Check(grade < .08f, "A water lift created an abrupt road approach");
        }

        float? Shoulder(float3 point) => Math.Abs(point.x) < 150 && point.z > 4 ? 106 : null;

        Check(
            NetworkStructureIntervals.StructureIntervals(
                new[] { road },
                terrain,
                1600,
                new ImportLog(),
                default,
                Shoulder)[0].Bridge,
            "A deeply submerged road edge was missed by the bridge decision");

        var junction =
            new[]{
                road with{
                    EndNodeId=3,
                    End=new(0,100,0),
                    SourceGeometry=new[]{new float3(-600,100,0),new float3(0,100,0)}
                    },
            road with{
                WayId=2,
                StartNodeId=3,
                EndNodeId=4,
                Start=new(0,100,0),
                SourceGeometry=new[]{new float3(0,100,0),new float3(600,100,0)}
                },
            road with{
                WayId=3,
                StartNodeId=3,
                EndNodeId=5,
                Start=new(0,100,0),
                End=new(0,100,600),
                SourceGeometry=new[]{new float3(0,100,0),new float3(0,100,600)}
                }};

        float? JunctionWater(float3 point) => Math.Abs(point.x) < 120 && Math.Abs(point.z) < 120 ? 101 : null;

        var joined =
            VisualNetworkCompiler.Prepare(junction, terrain, 1600, new ImportLog(), default, JunctionWater);

        var shared =
            joined
                .SelectMany(
                    segment => new[] {
                        (Id: segment.StartNodeId, Point: segment.CourseStart),
                        (Id: segment.EndNodeId, Point: segment.CourseEnd)
                        })
                .Where(item => item.Id == 3)
                .ToArray();

        Check(
            shared.Length >= 3
                && shared.All(item => item.Point.y > 101.25)
                && shared.All(item => math.distance(item.Point, shared[0].Point) < .001),
            "A shared wet junction settled below its water floor or submitted different arm heights");

        var tunnel = road with { Tunnel = true, Layer = -1 };
        Check(
            NetworkStructureIntervals.StructureIntervals(
                new[] { tunnel },
                terrain,
                1600,
                new ImportLog(),
                default,
                _ => 110)[0]
                == tunnel,
            "An intentionally submerged tunnel was converted into a bridge");

        var dry = VisualNetworkCompiler.Prepare(new[] { road }, terrain, 1600, new ImportLog(), default);
        var absent =
            VisualNetworkCompiler.Prepare(new[] { road }, terrain, 1600, new ImportLog(), default, _ => null);

        Check(
            dry
                .Select(
                    segment => (
                        segment.StartNodeId,
                        segment.EndNodeId,
                        segment.CourseStart,
                        segment.CourseControl1,
                        segment.CourseControl2,
                        segment.CourseEnd,
                        segment.Bridge,
                        segment.Tunnel,
                        segment.Lanes))
            .SequenceEqual(
                absent.Select(
                    segment => (
                        segment.StartNodeId,
                        segment.EndNodeId,
                        segment.CourseStart,
                        segment.CourseControl1,
                        segment.CourseControl2,
                        segment.CourseEnd,
                        segment.Bridge,
                        segment.Tunnel,
                        segment.Lanes))),
            "Unavailable water evidence changed a dry road");

        Console.WriteLine(
            "PASS: shallow-water ground-road freeboard, deep-water bridge selection, finite curves, tunnel preservation and dry-road equivalence.");
    }

    static void OpenedRiverDeck()
    {
        var terrain = Enumerable.Repeat(100f, 129 * 129).ToArray();
        var line = new[] { new float3(-700, 0, 0), new float3(700, 0, 0) };
        var water = new WaterPlan(false, 1, 0, 0, "opened river fixture")
        {
            WaterCellSize = 28,
            ReachStages = new[] { new WaterReachStage(10, 10, line[0], line[1], 106, 106, 6, null) }
        };

        var surface = WaterBodyPlanner.ConstructedSurfaceSampler(water, Array.Empty<GeographicPolygon>());
        Check(
            surface(new(0, 0, 35)) == 106 && !surface(new(0, 0, 60)).HasValue,
            "The feathered wet cut does not match the road-clearance sampler");

        var road =
            new PreparedSegment(
                10,
                11,
                12,
                new(-600, 100, 35),
                new(600, 100, 35),
                NetworkKind.Road,
                "primary",
                6,
                false,
                false,
                false,
                0)
            { NativeWidth = 32, SourceGeometry = new[] { new float3(-600, 100, 35), new float3(600, 100, 35) } };

        var selected =
            NetworkStructureIntervals.CulvertBridges(
                new[] { road },
                new[] { (line, 28 / .6f) },
                129,
                1600,
                new ImportLog(),
                default);

        Check(
            selected.All(segment => segment.Bridge && segment.Lanes == 6 && !segment.OneWay),
            "A boulevard following an opened river was not retained as a two-way bridge");

        var result = VisualNetworkCompiler.Prepare(selected, terrain, 1600, new ImportLog(), default, surface);
        Check(
            result
                .SelectMany(
                    segment => Enumerable.Range(0, 81).Select(k => NetworkNormalizer.Evaluate(segment, k / 80f)))
            .All(point => point.y >= 106 + WaterLevels.RoadBridgeClearanceMeters - .01f),
                "A bridge alongside the river's feathered bank remained underwater");

        var rail =
            new PreparedSegment(
                20,
                21,
                22,
                new(-600, 100, 49),
                new(600, 100, 49),
                NetworkKind.Rail,
                "rail",
                2,
                false,
                true,
                false,
                0)
            {
                NativeWidth = 11,
                PhysicalTracks = 2,
                NativeAssetKey = "rail",
                SourceGeometry = new[] { new float3(-600, 100, 49), new float3(600, 100, 49) }
            };

        var railResult =
            VisualNetworkCompiler.Prepare(new[] { rail }, terrain, 1600, new ImportLog(), default, surface);

        Check(
            railResult
                .SelectMany(
                    segment => Enumerable.Range(0, 81).Select(k => NetworkNormalizer.Evaluate(segment, k / 80f)))
            .All(point => point.y >= 106 + WaterLevels.RoadBridgeClearanceMeters - .01f),
                "A railway bridge missed water beneath its outer track");
    }

    static void DryShoreJunction()
    {
        var terrain = Enumerable.Repeat(100f, 129 * 129).ToArray();

        PreparedSegment Road(long way, long a, long b, float3 start, float3 end) =>
            new(way, a, b, start, end, NetworkKind.Road, "secondary", 2, false, false, false, 0)
            { NativeWidth = 12, SourceGeometry = new[] { start, end } };

        var roads = new[]{Road(201,201,202,new(-500,100,0),new(0,100,0)),Road(202,202,203,new(0,100,0),new(500,100,0)),
            Road(203,202,204,new(0,100,0),new(0,100,300))};

        float? Water(float3 point) => point.z >= 30 && point.z <= 90 && Math.Abs(point.x) < 30 ? 106 : null;

        var output = VisualNetworkCompiler.Prepare(roads, terrain, 1600, new ImportLog(), default, Water);
        var wet =
            output
                .SelectMany(
                    segment => Enumerable.Range(0, 201).Select(k => NetworkNormalizer.Evaluate(segment, k / 200f)))
                .Where(point => Water(point).HasValue)
                .ToArray();

        Check(wet.Length > 0 && wet.All(point => point.y >= 106.25f),
            "A dry shore junction overrode the connected road's water approach and submerged its emitted curve");

        var junction =
            output
                .SelectMany(
                    segment => new[] {
                        (Id: segment.StartNodeId, P: segment.CourseStart),
                        (Id: segment.EndNodeId, P: segment.CourseEnd)
                        })
                .Where(item => item.Id == 202)
                .ToArray();

        Check(junction.Length >= 3 && junction.All(item => math.distance(item.P, junction[0].P) < .001),
            "A water approach submitted inconsistent heights at its shared dry shore junction");

        var log = new ImportLog();
        SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(roads), output, log, default);
        Check(
            log.Metric("Missing mandatory selected-source ports") == 0
                && log.Metric("Unresolved selected-source reachability obligations") == 0,
            "A shore water approach lost a source port or route");
    }

    static void RoundaboutClearance()
    {
        var terrain = Enumerable.Repeat(100f, 129 * 129).ToArray();
        var points =
            new[] { new float3(20, 100, 0), new float3(0, 100, 20), new float3(-20, 100, 0), new float3(0, 100, -20) };

        const float handle = 11.045695f;
        var ring = Enumerable.Range(0, 4).Select(i =>
        {
            var vectorA = points[i];
            var vectorB = points[(i + 1) % 4];
            var first = new float3(-vectorA.z, 0, vectorA.x) / 20;
            var last = new float3(-vectorB.z, 0, vectorB.x) / 20;

            return new PreparedSegment(
                50,
                20 + i,
                20 + (i + 1) % 4,
                vectorA,
                vectorB,
                NetworkKind.Road,
                "primary",
                2,
                true,
                true,
                false,
                0)
            {
                Roundabout = true,
                NativeWidth = 12,
                Control1 = vectorA + first * handle,
                Control2 = vectorB - last * handle,
                SourceGeometry = new[] { vectorA, vectorB }
            };
        }).ToArray();

        var approach =
            new PreparedSegment(
                51,
                100,
                20,
                new(600, 100, 0),
                points[0],
                NetworkKind.Road,
                "primary",
                2,
                false,
                false,
                false,
                0)
            { NativeWidth = 12, SourceGeometry = new[] { new float3(600, 100, 0), points[0] } };

        float? Water(float3 point) => Math.Abs(point.x) < 40 && Math.Abs(point.z) < 40 ? 106 : null;

        var output =
            VisualNetworkCompiler.Prepare(
                ring.Append(approach).ToArray(),
                terrain,
                1600,
                new ImportLog(),
                default,
                Water);

        foreach (var original in ring)
        {
            var section = output.Single(segment => segment.Roundabout && segment.StartNodeId == original.StartNodeId);
            Check(
                math.all(section.CourseStart.xz == original.CourseStart.xz)
                    && math.all(section.CourseControl1.xz == original.CourseControl1.xz)
                    && math.all(section.CourseControl2.xz == original.CourseControl2.xz)
                    && math.all(section.CourseEnd.xz == original.CourseEnd.xz),
                "Roundabout water clearance changed the circular plan geometry");

            Check(
                Enumerable
                    .Range(0, 41)
                    .All(
                        k => NetworkNormalizer.Evaluate(section, k / 40f).y
                            >= 106 + WaterLevels.RoadBridgeClearanceMeters - .01f),
                "A roundabout bypassed bridge water clearance");
        }

        var arms = output.Where(segment => !segment.Roundabout).ToArray();
        Check(
            arms.Any(segment => segment.StartNodeId == 20 || segment.EndNodeId == 20)
                && arms.All(segment => NetworkVerticalProfiles.MaximumGrade(segment) < .08),
            "A raised roundabout disconnected or sharply bent its approach");

        var dry =
            VisualNetworkCompiler.Prepare(
                ring.Append(approach).ToArray(),
                terrain,
                1600,
                new ImportLog(),
                default);

        var unavailable =
            VisualNetworkCompiler.Prepare(
                ring.Append(approach).ToArray(),
                terrain,
                1600,
                new ImportLog(),
                default,
                _ => null);

        Check(
            dry
                .Select(
                    segment => (segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd))
                .SequenceEqual(
            unavailable.Select(
                segment => (segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd))),
                "Absent water changed a dry roundabout or approach");
    }
}
