using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

static class VisualDesignTests
{
    // A motorway over a crest with a ramp leaving it 60 m before a bridge over a
    // crossing road. Levelling the through carriageway at the diverge, and a node
    // height the lifted deck could not reach, left the ramp 1–2 m off the node
    // (Marseille and DC motorway junctions); arms must meet the node they share.
    static void JunctionBesideBridge()
    {
        const int size = 513;
        const double width = 4000;
        const float scale = 1000;
        var worst = 0d;

        foreach (var kink in new[] { -60f, -30f, 15f, 40f, 70f })
        {
            double Ground(double x) => x < kink ? 200 + .08 * x : 200 + .08 * kink - .06 * (x - kink);

            var terrain = new float[size * size];

            for (var z = 0; z < size; z++)
                for (var x = 0; x < size; x++)
                    terrain[z * size + x] =
                        (float)Ground((x / (double)(size - 1) - .5) * width);

            float3 Point(float x, float z) => new(x, (float)Ground(x), z);

            var roads =
                new[]{
                    Road(1,1,2,Point(-800,0),Point(0,0)),
                    Road(2,2,8,Point(0,0),Point(40,0)),
                    Road(6,8,9,Point(40,0),Point(80,0)) with{Bridge=true,Layer=1},
                    Road(7,9,3,Point(80,0),Point(800,0)),
                Road(8,20,21,Point(60,-300),Point(60,300)) with{Classification="primary",OneWay=false}};

            var ramp = Road(3, 2, 4, Point(0, 0), Point(400, -90)) with { Classification = "motorway_link" };
            var log = new ImportLog { ScalarDiagnostics = true };
            VisualNetworkCompiler.Prepare(roads.Append(ramp).ToArray(), terrain, width, log, default);
            var line =
                log.Lines.LastOrDefault(text => text.Contains("junction height mismatch", StringComparison.Ordinal));

            Check(line != null, "The compiler did not report junction height mismatch");
            var max =
                double.Parse(line!.Split("max=")[1].Split(';')[0], System.Globalization.CultureInfo.InvariantCulture);

            worst = Math.Max(worst, max);
        }

        Check(worst < .3, $"A ramp leaving a motorway beside a bridge met its node {worst:F2} m off");
    }

    // A ramp drawn hugging the motorway for 300 m before it diverges. It must
    // turn out at the merge angle and run parallel at its clearance until its
    // own line is wider: the old approach line swung it about 35 m out and back
    // (an S, A55 Marseille). The motorway keeps its line whether the branch is a
    // link or another motorway (A7 bowing toward a diverge).
    static void RampLeavesWithoutSwingingOut()
    {
        const int size = 257;
        const double width = 4000;
        const float scale = 1000;
        var terrain = new float[size * size];
        Array.Fill(terrain, 100f);

        var drawn =
            new[] {
                FlatPoint(0, 0),
                FlatPoint(150, -6),
                FlatPoint(300, -8),
                FlatPoint(450, -30),
                FlatPoint(700, -150)
                };

        foreach (var branch in new[] { "motorway_link", "motorway" })
        {
            var roads =
                new[] {
                    Road(1, 1, 2, FlatPoint(-800, 0), FlatPoint(0, 0)),
                    Road(2, 2, 3, FlatPoint(0, 0), FlatPoint(1200, 0))
                    };

            var ramp =
                Enumerable
                    .Range(0, drawn.Length - 1)
                    .Select(
                        k => Road(
                            3,
                            k == 0 ? 2 : 40 + k,
                            41 + k,
                            drawn[k],
                            drawn[k + 1]) with
                        { Classification = branch, Reference = branch == "motorway" ? "V2" : "" });

            var output =
                VisualNetworkCompiler.Prepare(
                    roads.Concat(ramp).ToArray(),
                    terrain,
                    width,
                    new ImportLog(),
                    default);

            IEnumerable<double3> Samples(long way)
                => output
                    .Where(segment => segment.WayId == way || segment.SourceWayIds.Contains(way))
                    .SelectMany(segment => Enumerable.Range(0, 33).Select(k =>
                NetworkDesignChecks.Evaluate(
                    new Cubic3d(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd),
                    k / 32d)));

            var mainline = Samples(1).Concat(Samples(2)).Max(point => Math.Abs(point.z));
            Check(mainline < .3, $"The motorway bowed {mainline:F2} m toward a {branch} leaving it");
            var rampLine = Samples(3).Where(point => point.x > 0 && point.x < 350).OrderBy(point => point.x).ToArray();
            var widest = rampLine.Max(point => -point.z);
            Check(widest < 22, $"A {branch} hugging the motorway swung {widest:F1} m out before following its line");
            var back = rampLine.Zip(rampLine.Skip(1), (vectorA, vectorB) => vectorB.z - vectorA.z).Max();
            Check(back < .5, $"A {branch} leaving the motorway turned back toward it by {back:F2} m between samples");
        }
    }

    // A 30 m motorway or track piece that differs from its neighbours only by
    // its OSM layer, on the ground and on a continuous bridge (a short source bridge
    // whose approaches were raised). Both layer changes are source ports, so
    // each bounded its own 30 m run, under the 60 m motorway minimum. The piece
    // joins a neighbour and the ports stay as mandatory aliases of a kept node.
    static void ShortLayerPieceJoined()
    {
        var terrain = Enumerable.Repeat(50f, 33 * 33).ToArray();

        foreach (var (bridge, rail) in new[] { (false, false), (true, false), (false, true) })
        {
            var source =
                new[]{
                    Road(901,910,911,new(-300,50,0),new(-15,50,0)),
                    Road(902,911,912,new(-15,50,0),new(15,50,0)) with{Layer=1},
                Road(903,912,913,new(15,50,0),new(300,50,0))}
                    .Select(segment => segment with { Bridge = bridge, Layer = segment.Layer + (bridge ? 1 : 0) })
                .Select(
                    segment => rail
                        ? segment with
                        {
                            Kind = NetworkKind.Rail,
                            Classification = "rail",
                            OneWay = false,
                            NativeAssetKey = "rail",
                            PhysicalTracks = 2
                        }
                        : segment)
                    .ToArray();

            var log = new ImportLog();
            var output = VisualNetworkCompiler.Prepare(source, terrain, 2000, log, default);
            SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(source), output, log, default);
            Check(
                log.Metric("Missing mandatory selected-source ports") == 0
                    && log.Metric("Unresolved selected-source reachability obligations") == 0,
                $"Joining a short layer piece (bridge={bridge}, rail={rail}) lost a source port or directed connection");

            var shortest =
                output.Min(
                    segment => NetworkDesignChecks.HorizontalArcLength(
                        new Cubic3d(
                            segment.CourseStart,
                            segment.CourseControl1,
                            segment.CourseControl2,
                            segment.CourseEnd)));

            Check(shortest >= 60, $"A layer-only boundary (bridge={bridge}, rail={rail}) left a {shortest:F1} m piece");
            var nodes = output.SelectMany(segment => new[] { segment.StartNodeId, segment.EndNodeId }).ToHashSet();
            Check(
                output
                    .SelectMany(segment => segment.SourceNodeAliases)
                    .Where(alias => alias.SourceNodeId is 911 or 912 && !nodes.Contains(alias.SourceNodeId))
                    .All(alias => alias.Mandatory && nodes.Contains(alias.CanonicalNodeId)),
                $"A joined layer port (bridge={bridge}, rail={rail}) is not a mandatory alias of an emitted node");

            Check(
                !bridge || output.All(segment => segment.Bridge && segment.Layer >= 1),
                "A joined bridge piece lost its structure or its higher layer");
        }
    }

    // Track on flat ground at 200 m crossing a 40 m deep, 300 m wide valley on a
    // bridge. Rail targets are a 150 m moving average of the ground; with the
    // valley floor under the deck averaged in, the bank targets sank and the
    // deck with them (DC's Long Bridge: 7 m below the switches at its end).
    static void RailBridgeOverValley()
    {
        const int size = 513;
        const double width = 4000;
        const float scale = 1000;

        static double Ground(double x) => Math.Abs(x) < 150 ? 160 : 200;

        var terrain = new float[size * size];

        for (var z = 0; z < size; z++)
            for (var x = 0; x < size; x++)
                terrain[z * size + x] =
                    (float)Ground((x / (double)(size - 1) - .5) * width);

        static PreparedSegment Track(long way, long a, long b, float x0, float x1, bool bridge) =>
            new(
                way,
                a,
                b,
                new(x0, (float)(bridge ? 200 : Ground(x0)), 0),
                new(x1, (float)(bridge ? 200 : Ground(x1)), 0),
                NetworkKind.Rail,
                "rail",
                2,
                false,
                bridge,
                false,
                bridge ? 1 : 0)
            {
                NativeWidth = 12,
                NativeAssetKey = "rail",
                PhysicalTracks = 2,
                SourceGeometry = new[] { new float3(x0, 200, 0), new float3(x1, 200, 0) },
                OriginalGeometry = new[] { new float3(x0, 200, 0), new float3(x1, 200, 0) }
            };

        var line =
            new[] {
                Track(1, 1, 2, -1200, -150, false),
                Track(2, 2, 3, -150, 150, true),
                Track(3, 3, 4, 150, 1200, false)
                };

        var output = VisualNetworkCompiler.Prepare(line, terrain, width, new ImportLog(), default);
        var deck =
            output
                .Where(segment => segment.Bridge)
                .SelectMany(
                    segment => Enumerable
                        .Range(0, 17)
                        .Select(
                            k => NetworkDesignChecks.Evaluate(
                                new Cubic3d(
                                    segment.CourseStart,
                                    segment.CourseControl1,
                                    segment.CourseControl2,
                                    segment.CourseEnd),
                                k / 16d).y))
                .ToArray();

        Check(
            deck.Length > 0 && deck.All(y => Math.Abs(y - 200) < 2),
            $"A rail bridge over a valley sank to {deck.DefaultIfEmpty(0).Min():F1} m between banks at 200 m");
    }

    // A motorway on flat ground with ramps leaving it 100 m before and 100 m after
    // a bridge over a crossing road. Grade edges alone let the
    // coupled solve hold both junctions near the ground, so the deck had to
    // climb its 6.5 m clearance and come down again between them, a vertical
    // curve several times sharper than the motorway K. Both junctions must rise.
    static void BridgeBetweenJunctions()
    {
        const int size = 257;
        const double width = 4000;
        const float scale = 1000;
        var terrain = new float[size * size];
        Array.Fill(terrain, 100f);

        var source =
            new[]{
                Road(1,1,2,FlatPoint(-1500,0),FlatPoint(0,0)),
                Road(2,2,3,FlatPoint(0,0),FlatPoint(50,0)),
                Road(3,3,4,FlatPoint(50,0),FlatPoint(150,0)) with{Bridge=true,Layer=1},
            Road(4,4,5,FlatPoint(150,0),FlatPoint(200,0)),Road(7,5,8,FlatPoint(200,0),FlatPoint(1500,0)),
            Road(5,2,6,FlatPoint(0,0),FlatPoint(400,-150)) with{Classification="motorway_link"},
            Road(6,5,9,FlatPoint(200,0),FlatPoint(700,-150)) with{Classification="motorway_link"},
            Road(8,20,21,FlatPoint(100,-400),FlatPoint(100,400)) with{Classification="primary",OneWay=false}};

        var output = VisualNetworkCompiler.Prepare(source, terrain, width, new ImportLog(), default);
        var motorway = output.Where(segment => segment.Classification == "motorway").ToArray();
        var bends =
            VisualCompilerMetrics.Measure(motorway, terrain, width, default).Failures
                .Where(failure => failure.Category == "grade-change")
                .ToArray();

        Check(
            bends.Length == 0,
            $"A motorway bridge between two junctions 200 m apart bent {bends.Select(failure => failure.Severity).DefaultIfEmpty(0).Max():F2}× its K");
    }

    static void RampOutsideLoopFootprint()
    {
        // Translated plan samples of the DC ramp whose neighbouring through
        // road loops back. Its far side is outside the road's actual footprint.
        var fixture =
            Newtonsoft.Json.Linq.JObject.Parse(
                File.ReadAllText(Path.Combine(AppContext.BaseDirectory, "fixtures", "ramp-dc-loop.json")));

        float3[] Points(string key) => fixture[key]!.Select(row => FlatPoint((float)row[0]!, (float)row[1]!)).ToArray();

        var partner = Points("through");
        var branch = Points("branch");
        var through =
            new[] { FlatPoint(-200 * (partner[1].x / 12), -200 * (partner[1].z / 12)) }.Concat(partner).ToArray();

        var source =
            Enumerable
                .Range(1, through.Length - 1)
                .Select(k => Road(600, 6000 + k - 1, 6000 + k, through[k - 1], through[k]))
            .Concat(
                Enumerable
                    .Range(1, branch.Length - 1)
                    .Select(
                        k => Road(
                            601,
                            k == 1 ? 6001 : 6100 + k - 1,
                            6100 + k,
                            branch[k - 1],
                            branch[k]) with
                        { Classification = "motorway_link", Reference = "" }))
                .ToArray();

        var terrain = Enumerable.Repeat(100f, 33 * 33).ToArray();
        var log = new ImportLog();
        var result = VisualNetworkCompiler.Prepare(source, terrain, 4000, log, default);

        double Distance(double2 point)
        {
            var best = double.PositiveInfinity;

            for (var k = 1; k < branch.Length; k++)
            {
                var a = (double2)branch[k - 1].xz;
                var d = (double2)branch[k].xz - a;
                best =
                    Math.Min(
                        best,
                        math.distance(point, a + d * math.clamp(math.dot(point - a, d) / math.lengthsq(d), 0, 1)));
            }

            return best;
        }

        var deviation =
            result
                .Where(segment => segment.WayId == 601 || segment.SourceWayIds.Contains(601))
                .SelectMany(segment => Enumerable.Range(0, 33)
            .Select(
                k => NetworkDesignChecks.Evaluate(
                    new Cubic3d(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd),
                    k / 32d).xz))
            .Where(point => math.length(point) > 100).Max(Distance);

        Check(deviation < 40, $"A ramp outside the loop's footprint was pushed {deviation:0.##} m across empty space");
        SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(source), result, log, default);
        Check(
            log.Metric("Missing mandatory selected-source ports") == 0
                && log.Metric("Unresolved selected-source reachability obligations") == 0,
            "Loop approach shaping lost a ramp's directed route");
    }

    public static void Run()
    {
        StreetGridTests.Run();
        ShortLayerPieceJoined();
        RailBridgeOverValley();
        BridgeBetweenJunctions();
        RampOutsideLoopFootprint();
        HighwayPriorityAndAvenueJunction();
        var bridgeRoad = new[]{Road(801,810,811,new(-250,50,0),new(-50,50,0)) with{Bridge=true,Layer=1},
            Road(802,811,812,new(-50,50,0),new(50,50,0)),
            Road(803,812,813,new(50,50,0),new(250,50,0)) with{Bridge=true,Layer=1}};

        var bridgeLog = new ImportLog();
        var continuousBridge =
            VisualNetworkCompiler.Prepare(
                bridgeRoad,
                Enumerable.Repeat(50f, 33 * 33).ToArray(),
                2000,
                bridgeLog,
                default);

        SourceTopologyObligations.Validate(
            SourceTopologyObligations.Capture(bridgeRoad),
            continuousBridge,
            bridgeLog,
            default);

        Check(
            bridgeLog.Metric("Missing mandatory selected-source ports") == 0
                && bridgeLog.Metric("Unresolved selected-source reachability obligations") == 0,
            "Filling a bridge gap lost its former boundary ports or directed source connections");

        Check(continuousBridge.All(segment => segment.Bridge && math.distance(segment.Start.xz, segment.End.xz) >= 100),
            "Retaining bridge-boundary identity reintroduced the ground gap or extra short sections");

        Check(
            continuousBridge
                .SelectMany(segment => segment.SourceNodeAliases)
                .Where(alias => alias.SourceNodeId is 811 or 812)
                .Select(alias => alias.SourceNodeId)
                .Distinct()
                .Count()
                == 2,
            "Former bridge boundaries lack explicit source aliases");

        JunctionBesideBridge();
        RampLeavesWithoutSwingingOut();
        var crossing = Road(300, 3000, 3001, new(-200, 50, 0), new(200, 50, 0));

        PreparedSegment Rail(long way, long a, long b, float3 point, float3 otherPoint)
            => Road(
                way,
                a,
                b,
                point,
                otherPoint) with
            { Kind = NetworkKind.Rail, Classification = "rail", OneWay = false };

        var fan = new[]{Rail(401,1,2,new(-100,50,0),new(0,50,0)),Rail(402,2,3,new(0,50,0),new(8,50,0)),
            Rail(403,3,4,new(8,50,0),new(100,50,0)),
                Rail(404,2,5,new(0,50,0),new(-100,50,100)),
                Rail(405,3,6,new(8,50,0),new(100,50,-100))
                };

        var nativeFan = RailCorridorConsolidator.NativeJunctionFootprints(fan, new ImportLog(), default);
        var fanLog = new ImportLog();
        SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(fan), nativeFan, fanLog, default);
        Check(
            nativeFan.Count == 4
                && fanLog.Metric("Missing mandatory selected-source ports") == 0
                && fanLog.Metric("Unresolved selected-source reachability obligations") == 0,
            "Native-width rail footprint reconstruction lost external ports or movements");

        var oscillating = crossing with { Control1 = new(-65, 80, 0), Control2 = new(65, 20, 0) };
        var visualLog = new ImportLog { CaptureRecords = true };
        NetworkQualityAudit.Planned(new[] { oscillating }, visualLog, default);
        Check(
            visualLog.Findings.Any(finding => finding.Category == "visual-oscillation" && finding.Failed > 0),
            "Visual ledger missed a full hump-and-dip profile");

        var svg = (string)visualLog.Artifacts["visual-quality-worst.svg"];
        Check(
            svg.Contains("<polyline")
                && svg.Contains("sampled, not rendered")
                && System.Text.Encoding.UTF8.GetByteCount(svg) < 1024 * 1024,
            "Visual evidence diagram missing, misleading, or unbounded");

        var document = new System.Xml.XmlDocument();
        document.LoadXml(svg);
        Console.WriteLine(
            "PASS: guarded budgets; bridge/tunnel intent; footprint cache invalidation; visual ledger and SVG.");
    }

    static void HighwayPriorityAndAvenueJunction()
    {
        var terrain = Enumerable.Repeat(50f, 129 * 129).ToArray();

        foreach (var classification in new[] { "motorway", "trunk" })
        {
            var highway =
                Road(60001, 60001, 60002, new(-500, 50, 0), new(500, 50, 0)) with { Classification = classification };

            var street =
                Road(
                    60002,
                    60003,
                    60004,
                    new(-500, 50, 10),
                    new(500, 50, 10)) with
                { Classification = "primary", Reference = "Street" };

            var result =
                VisualNetworkCompiler.Prepare(new[] { highway, street }, terrain, 2000, new ImportLog(), default);

            Check(
                result
                    .Where(segment => segment.WayId == highway.WayId)
                    .SelectMany(
                        segment => Enumerable.Range(0, 41).Select(k => NetworkNormalizer.Evaluate(segment, k / 40f)))
                .All(point => Math.Abs(point.z) < .001),
                    "A street moved the motorway/trunk alignment to share its clearance correction");

            Check(
                result
                    .Where(segment => segment.WayId == street.WayId)
                    .Any(
                        segment => Enumerable
                            .Range(0, 41)
                            .Any(k => NetworkNormalizer.Evaluate(segment, k / 40f).z > 11)),
                "Highway priority left the overlapping street without a clearance correction");
        }

        var firstSegment =
            Road(
                60010,
                60010,
                60011,
                new(-500, 50, 0),
                new(0, 50, 0)) with
            { Classification = "primary", OneWay = false };

        var secondSegment =
            Road(
                60011,
                60011,
                60012,
                new(0, 50, 0),
                new(500, 50, 0)) with
            { Classification = "primary", OneWay = false };

        var branch =
            Road(
                60012,
                60011,
                60013,
                new(0, 50, 0),
                new(350, 50, 18)) with
            { Classification = "secondary", OneWay = false, Reference = "Branch" };

        var output =
            VisualNetworkCompiler.Prepare(
                new[] { firstSegment, secondSegment, branch },
                terrain,
                2000,
                new ImportLog(),
                default);

        var arm = output.Single(segment => segment.WayId == branch.WayId && segment.StartNodeId == 60011);
        var heading = math.normalizesafe((arm.CourseControl1 - arm.CourseStart).xz);
        var angle = math.degrees(Math.Acos(math.clamp(heading.x, -1, 1)));
        Check(angle >= 25, $"An ordinary two-way avenue retained its acute side-street junction ({angle:F2} degrees)");
        Check(
            output
                .Where(segment => segment.WayId != branch.WayId)
                .All(segment => Math.Abs(segment.CourseControl1.z) < .001 && Math.Abs(segment.CourseControl2.z) < .001),
            "Correcting an acute avenue branch moved the through street");

        var audit = new ImportLog();
        SourceTopologyObligations.Validate(
            SourceTopologyObligations.Capture(new[] { firstSegment, secondSegment, branch }),
            output,
            audit,
            default);

        Check(
            audit.Metric("Missing mandatory selected-source ports") == 0
                && audit.Metric("Unresolved selected-source reachability obligations") == 0,
            "Avenue approach correction lost the side street's source route");
    }

    internal static PreparedSegment Road(long way, long a, long b, float3 point, float3 otherPoint) =>
            new(way, a, b, point, otherPoint, NetworkKind.Road, "motorway", 2, true, false, false, 0)
            {
                NativeWidth = 12,
                NativeAssetKey = "visual",
                Reference = "V1",
                SourceGeometry = new[] { point, otherPoint },
                OriginalGeometry = new[] { point, otherPoint }
            };

    internal static float3 FlatPoint(float x, float z) => new(x, 100, z);
}
