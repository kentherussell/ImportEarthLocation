using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

static class RailCrossingTests
{
    static PreparedSegment Line(long way, long a, long b, float3 point, float3 otherPoint, NetworkKind kind) =>
        new(
            way,
            a,
            b,
            point,
            otherPoint,
            kind,
            kind == NetworkKind.Rail ? "rail" : "secondary",
            2,
            false,
            false,
            false,
            0)
        {
            NativeWidth = kind == NetworkKind.Rail ? 10 : 24,
            SourceGeometry = new[] { point, otherPoint },
            OriginalGeometry = new[] { point, otherPoint }
        };

    public static void Run()
    {
        const double width = 2000;
        const float scale = 300;
        const int size = 257;
        var terrain = new float[size * size];

        for (var z = 0; z < size; z++)
            for (var x = 0; x < size; x++)
            {
                var px = (x / (double)(size - 1) - .5) * width;
                var pz = (z / (double)(size - 1) - .5) * width;
                terrain[z * size + x] =
                    (float)(100 + .025 * px + .015 * pz + 3 * Math.Exp(-px * px / 2500));
            }

        var source =
            new[]{
                Line(1,1,50,VisualDesignTests.FlatPoint(-503,0),VisualDesignTests.FlatPoint(0,0),NetworkKind.Road),
                Line(1,50,2,VisualDesignTests.FlatPoint(0,0),VisualDesignTests.FlatPoint(487,0),NetworkKind.Road),
            Line(2,3,4,VisualDesignTests.FlatPoint(0,-501),VisualDesignTests.FlatPoint(3,-18),NetworkKind.Rail),
                Line(2,4,50,VisualDesignTests.FlatPoint(3,-18),VisualDesignTests.FlatPoint(0,0),NetworkKind.Rail),
            Line(2,50,5,VisualDesignTests.FlatPoint(0,0),VisualDesignTests.FlatPoint(-2,21),NetworkKind.Rail),
                Line(2,5,6,VisualDesignTests.FlatPoint(-2,21),VisualDesignTests.FlatPoint(0,499),NetworkKind.Rail)
                };

        var log = new ImportLog();
        var compiled = VisualNetworkCompiler.Prepare(source, terrain, width, log, default);
        var output = NativeEndpointPolicy.PrepareLevelCrossings(compiled, log);
        var ports =
            output
                .SelectMany(
                    segment => new[] {
                        (Section: segment, Node: segment.StartNodeId, Point: segment.CourseStart),
                        (Section: segment, Node: segment.EndNodeId, Point: segment.CourseEnd)
                        })
            .Where(item => item.Node == 50).ToArray();

        Check(ports.Length == 4, "The road/rail crossing lost one of its four source approaches");
        Check(
            ports.Select(item => item.Point).Distinct().Count() == 1 && math.length(ports[0].Point.xz) < .001f,
            "Crossing approaches moved apart horizontally or vertically");

        Check(
            ports.All(item => NativeEndpointPolicy.Identity(item.Section, 50) == "Crossing:50"),
            "The native crossing retained separate road/rail owners");

        Check(
            log.Metric("Visual compiler shared road rail crossing heights") == 1,
            "The visual compiler did not couple the crossing height");

        // Under smooth-roads the road fit placed the crossing's road end a rounding residual
        // (-4.5e-20 m) from the rail's, so the crossing was not recognised (report 114).
        var switches = DesignSwitches.Active;
        DesignSwitches.Set(DesignSwitches.SmoothRoads);

        try
        {
            var smoothLog = new ImportLog();
            var smooth =
                NativeEndpointPolicy.PrepareLevelCrossings(
                    VisualNetworkCompiler.Prepare(source, terrain, width, smoothLog, default),
                    smoothLog);

            var smoothPorts =
                smooth
                    .SelectMany(
                        segment => new[] {
                            (Section: segment, Node: segment.StartNodeId, Point: segment.CourseStart),
                            (Section: segment, Node: segment.EndNodeId, Point: segment.CourseEnd)
                            })
                    .Where(item => item.Node == 50)
                    .ToArray();

            Check(
                smoothPorts.Length == 4
                    && smoothPorts.Select(item => item.Point).Distinct().Count() == 1
                    && smoothPorts.All(item => NativeEndpointPolicy.Identity(item.Section, 50) == "Crossing:50"),
                "Smooth-roads level crossing ends not shared exactly");
        }
        finally
        {
            DesignSwitches.Set(switches);
        }

        SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(source), output, log, default);
        Check(
            log.Metric("Missing mandatory selected-source ports") == 0
                && log.Metric("Unresolved selected-source reachability obligations") == 0,
            "Crossing shaping lost a source route");

        foreach (var port in ports)
        {
            var segment = port.Section;
            var start = segment.StartNodeId == 50;
            var tangent =
                start ? segment.CourseControl1 - segment.CourseStart : segment.CourseEnd - segment.CourseControl2;

            Check(
                Math.Abs(tangent.y) / Math.Max(.01, math.length(tangent.xz)) < .02,
                "The crossing has a steep warped entry tangent");
        }

        var railDirections = ports.Where(item => item.Section.Kind == NetworkKind.Rail).Select(item =>
        {
            var segment = item.Section;
            return math.normalizesafe(
                item.Node == segment.StartNodeId
                    ? (segment.CourseControl1 - segment.CourseStart).xz
                    : (segment.CourseControl2 - segment.CourseEnd).xz);
        }).ToArray();

        Check(
            math.dot(railDirections[0], railDirections[1]) < -.999f,
            "The railway has a horizontal kink at its road crossing");

        foreach (var rail in new[]{
            source[2] with{
                StartNodeId=50,
                Start=VisualDesignTests.FlatPoint(0,0),
                EndNodeId=99,
                End=VisualDesignTests.FlatPoint(0,100),
                Bridge=true,
                Layer=1
                },
            source[2] with{
                StartNodeId=50,
                Start=VisualDesignTests.FlatPoint(0,0),
                EndNodeId=99,
                End=VisualDesignTests.FlatPoint(0,100),
                Tunnel=true
                },
            source[2] with{
                StartNodeId=50,
                Start=VisualDesignTests.FlatPoint(0,0),
                EndNodeId=99,
                End=VisualDesignTests.FlatPoint(0,100),
                Layer=1
                }})
            Check(
                NativeEndpointPolicy.LevelCrossings(new[] { source[0], rail }).Count == 0,
                "A separate-level railway became a level crossing");

        var unrelated = source[3] with { EndNodeId = 500 };
        Check(
            NativeEndpointPolicy.LevelCrossings(new[] { source[0], unrelated }).Count == 0,
            "Coincident unrelated endpoints became a crossing");

        NativeBinding();
        var marked = source[0] with { EndLevelCrossing = true };
        var reversed = NetworkNormalizer.Reverse(marked);
        Check(
            reversed.StartLevelCrossing && !reversed.EndLevelCrossing,
            "Reversing a road moved its crossing identity to the wrong end");

        var split = NativeDesignUnits.Split(marked, .5, -500);
        Check(!split.Left.EndLevelCrossing && !split.Right.StartLevelCrossing && split.Right.EndLevelCrossing,
            "A generated partition inherited the source crossing identity");

        var railPort = source[4];
        var shifted = source[1] with
        {
            Start = new(0, 100.1f, 0),
            Control1 = new(100, 100.1f, 0),
            Control2 = new(200, 100, 0),
            FinalDeckCurve = new(new(0, 100.1f, 0), new(100, 100.1f, 0), new(200, 100, 0), new(487, 100, 0))
        };

        var canonical = NativeEndpointPolicy.PrepareLevelCrossings(new[] { shifted, railPort }, new ImportLog());
        Check(canonical[0].CourseStart.y == canonical[1].CourseStart.y && canonical[0].CourseControl1.y == 100,
            "Crossing canonicalization did not update the frozen native deck and its adjacent handle");

        var conflict = shifted with { FinalDeckCurve = null, Start = new(0, 101, 0) };
        Check(
            NativeEndpointPolicy
                .PrepareLevelCrossings(new[] { conflict, railPort }, new ImportLog())
                .All(segment => !segment.StartLevelCrossing),
            "A large crossing height disagreement was hidden by snapping the road");

        Console.WriteLine(
            "PASS: shared road/rail crossing position, height, shallow approach tangents, native ownership across batches, source routes and separate-level guards.");
    }

    static void NativeBinding()
    {
        var road = Line(10, 1, 50, new(-50, 100, 0), new(0, 100, 0), NetworkKind.Road) with { EndLevelCrossing = true };
        var rail =
            Line(11, 50, 2, new(0, 100, 0), new(0, 100, 50), NetworkKind.Rail) with { StartLevelCrossing = true };

        NativeBatchBindings.Edge<string> Edge(PreparedSegment segment, string id, string firstText, string secondText)
            => new(id, firstText, secondText, segment.Start, segment.End, segment.Kind, "stock");

        var re = Edge(road, "road", "a", "crossing");
        var te = Edge(rail, "rail", "crossing", "b");

        NativeBatchBindings.Expected<string> Expected(PreparedSegment segment) => new(segment, "stock");

        var empty = new Dictionary<string, string>();
        var together = NativeBatchBindings.Match(new[] { Expected(road), Expected(rail) }, new[] { re, te }, empty);
        Check(
            together.Anchors.Count == 3 && together.Anchors["Crossing:50"].Entity == "crossing",
            "Native mixed-mode crossing did not share its owner");

        var first = NativeBatchBindings.Match(new[] { Expected(road) }, new[] { re }, empty);
        var committed = first.Anchors.ToDictionary(entry => entry.Key, entry => entry.Value.Entity);
        var second = NativeBatchBindings.Match(new[] { Expected(rail) }, new[] { te }, committed);
        Check(
            second.Anchors["Crossing:50"].Entity == "crossing",
            "A later rail batch did not attach to the existing road crossing");
    }
}
