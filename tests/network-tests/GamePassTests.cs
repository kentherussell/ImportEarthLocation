using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

// Defects seen in the 5 November game pass: broken ends and buried roads at tunnels.
static class GamePassTests
{
    static PreparedSegment Road(
        long way,
        long a,
        long b,
        float x1,
        float z1,
        float x2,
        float z2,
        string classification = "primary") =>
        new(
            way,
            a,
            b,
            new(x1, 100, z1),
            new(x2, 100, z2),
            NetworkKind.Road,
            classification,
            2,
            false,
            false,
            false,
            0)
        { NativeWidth = 12, SourceGeometry = new[] { new float3(x1, 100, z1), new float3(x2, 100, z2) } };

    // A straight street keeps its line across a crossing. Its continuation past the junction
    // was read as a road alongside it, which pushed it about 4 m sideways 60 m out.
    static void StraightThroughCrossing()
    {
        PreparedSegment Street(long way, long a, long b, float x1, float z1, float x2, float z2, string name)
            => Road(way, a, b, x1, z1, x2, z2) with { Name = name, NativeWidth = 16 };

        var streets =
            new[] {
                Street(1, 1, 2, -600, 0, 0, 0, "West") with { OneWay = true },
                Street(2, 2, 3, 0, 0, 600, 0, "East") with { Classification = "secondary", OneWay = true },
                Street(3, 4, 2, 0, -600, 0, 0, "South"),
                Street(4, 2, 5, 0, 0, 0, 600, "North")
                };

        var flat = Enumerable.Repeat(100f, 128 * 128).ToArray();
        var output = VisualNetworkCompiler.Prepare(streets, flat, 4000, new ImportLog(), default);
        var drift = 0f;

        foreach (var segment in output.Where(segment => segment.WayId is 1 or 2))
            for (var k = 0; k <= 16; k++)
                drift = Math.Max(drift, Math.Abs(NetworkNormalizer.Evaluate(segment, k / 16f).z));

        Check(drift < .5f, $"A straight street drifted {drift:F1} m sideways approaching a crossing");
    }

    // A short tunnel under a road stays on the ground and the road over it becomes a bridge.
    static void ShortUnderpass(ImportLog log)
    {
        var under = Road(1, 1, 2, -60, 0, 60, 0) with { Tunnel = true, Layer = -1 };
        var over = Road(2, 3, 4, 0, -100, 0, 100);
        var opened = ShortUnderpasses.Prepare(new[] { under, over }, log, default);
        Check(!opened[0].Tunnel && opened[1].Bridge, "Short underpass kept as a tunnel");

        var longBore = Road(1, 1, 2, -200, 0, 200, 0) with { Tunnel = true, Layer = -1 };
        Check(
            ShortUnderpasses.Prepare(new[] { longBore, over }, log, default)[0].Tunnel,
            "A long tunnel was brought to the ground");

        var motorway = over with { Classification = "motorway", OneWay = true };
        Check(
            ShortUnderpasses.Prepare(new[] { under, motorway }, log, default)[0].Tunnel,
            "A minor road under a motorway lifted the motorway");
    }

    // A compact roundabout is level even on a slope. Each node used to take the height of
    // the road reaching it, which tilted rings up to 100% or more.
    static void LevelRoundabout()
    {
        PreparedSegment Ring(long way, long a, long b, float x1, float z1, float x2, float z2)
            => Road(way, a, b, x1, z1, x2, z2) with { Roundabout = true, OneWay = true, NativeWidth = 10 };

        var streets =
            new[] {
                Ring(1, 1, 2, -30, 0, 0, 30),
                Ring(2, 2, 3, 0, 30, 30, 0),
                Ring(3, 3, 4, 30, 0, 0, -30),
                Ring(4, 4, 1, 0, -30, -30, 0),
                Road(5, 5, 1, -500, 0, -30, 0) with { NativeWidth = 16 },
                Road(6, 3, 6, 30, 0, 500, 0) with { NativeWidth = 16 }
                };

        // Ground rising 15 m per 100 m along x.
        var slope = new float[128 * 128];

        for (var z = 0; z < 128; z++)
            for (var x = 0; x < 128; x++)
                slope[z * 128 + x] = (100 + (x / 127f - .5f) * 4000 * .15f);

        var output = VisualNetworkCompiler.Prepare(streets, slope, 4000, new ImportLog(), default);
        var ring = output.Where(segment => segment.Roundabout).SelectMany(segment => new[] { segment.Start.y, segment.End.y }).ToArray();
        Check(ring.Length > 0 && ring.Max() - ring.Min() < .01f, $"A roundabout's nodes differ by {ring.Max() - ring.Min():F1} m");
    }

    public static void Run()
    {
        LevelRoundabout();
        StraightThroughCrossing();
        ShortUnderpass(new ImportLog());
        var log = new ImportLog();
        var request =
            new ImportRequest(0, 0, new("test", "test", 4000, false), false) { RoadLevel = RoadImportLevel.Primary };

        // A side-street arm left on a merged avenue goes, and its port stays an alias.
        var avenue = new[] { Road(1, 1, 2, -500, 0, 0, 0), Road(2, 2, 3, 0, 0, 500, 0), Road(3, 2, 4, 0, 0, 0, 10) };
        var pruned = NetworkDeadEnds.PruneStubs(avenue, request, log);
        Check(
            pruned.Count == 2 && pruned.All(segment => segment.StartNodeId != 4 && segment.EndNodeId != 4),
            "Merged avenue's side arm survived");

        // An end stopping short of a road ahead splits that road and joins it.
        var crossing = new[] { Road(1, 1, 2, -500, 0, 500, 0), Road(2, 3, 4, 100, -500, 100, -20) };
        var closed = NetworkDeadEnds.Close(crossing, 4000, log, default);
        Check(closed.Count == 3 && closed.Count(segment => segment.WayId == 1) == 2, "Road ahead was not split");
        var joined = closed.Single(segment => segment.WayId == 2);
        Check(
            closed.Where(segment => segment.WayId == 1).Any(segment => segment.EndNodeId == joined.EndNodeId)
                && Math.Abs(joined.End.z) < .01f,
            "Dead end did not reach the road ahead");

        // A node close to the meeting point is reused rather than cutting a stub.
        var nearNode =
            NetworkDeadEnds.Close(
                new[] { Road(1, 1, 2, -500, 0, 0, 0), Road(2, 2, 5, 0, 0, 500, 0), Road(3, 3, 4, 2, -500, 2, -20) },
                4000,
                log,
                default);

        Check(
            nearNode.Count == 3 && nearNode.Single(segment => segment.WayId == 3).EndNodeId == 2,
            "Dead end cut a stub next to an existing node");

        // Not joined: a road beside it, a road on a structure, a mainline for a main road, one out of reach.
        Check(
            NetworkDeadEnds.Close(
                new[] { Road(1, 1, 2, -500, 0, 500, 0), Road(2, 3, 4, -200, 15, 200, 15) },
                4000,
                log,
                default).Count
                == 2,
            "Parallel road joined sideways");

        Check(
            NetworkDeadEnds.Close(
                new[] { Road(1, 1, 2, -500, 0, 500, 0) with { Bridge = true }, Road(2, 3, 4, 100, -500, 100, -20) },
                4000,
                log,
                default).Single(segment => segment.WayId == 2).EndNodeId
                == 4,
            "Dead end joined a bridge");

        Check(
            NetworkDeadEnds.Close(
                new[] { Road(1, 1, 2, -500, 0, 500, 0, "motorway"), Road(2, 3, 4, 100, -500, 100, -20) },
                4000,
                log,
                default).Single(segment => segment.WayId == 2).EndNodeId
                == 4,
            "Main road joined a motorway");

        Check(
            NetworkDeadEnds.Close(
                new[] { Road(1, 1, 2, -500, 0, 500, 0), Road(2, 3, 4, 100, -500, 100, -40) },
                4000,
                log,
                default).Single(segment => segment.WayId == 2).EndNodeId
                == 4,
            "Avenue reached too far");

        var ramp =
            NetworkDeadEnds.Close(
                new[] { Road(1, 1, 2, -500, 0, 500, 0), Road(2, 3, 4, 100, -500, 100, -40, "motorway_link") },
                4000,
                log,
                default);

        Check(ramp.Single(segment => segment.WayId == 2).EndNodeId != 4, "Ramp stopped short of the road ahead");
        Roadbed();
        Border(log);
        Exact(log);
        TunnelElevation(log);
        Drawn(log);
        SplitRoads(log);
        StrayingCarriageways();
        ConflictingJunctionHalves();
        ReversibleCentreLane(request);
        BlindOneWay(log);
        LimitedAccess(log);
        Streets(log, request);
        Console.WriteLine("Game pass tests passed");
    }

    // The game makes a node an outside connection within 0.1 m of the terrain bounds.
    static void Border(ImportLog log)
    {
        const float half = 2000;
        var near = Road(1, 1, 2, 0, 0, half - 3, 0);
        Check(
            NativeEndpointPolicy.SnapToBorder(new[] { near }, 4000, log)[0].End.x == half,
            "Clipped end left short of the edge");

        var joined =
            NativeEndpointPolicy.SnapToBorder(new[] { near, Road(2, 2, 3, half - 3, 0, half - 3, 50) }, 4000, log);

        Check(joined[0].End.x == half - 3, "A node with two roads moved to the edge");
        var heading = NetworkDeadEnds.ReachBorder(new[] { Road(1, 1, 2, 0, 0, half - 200, 0) }, 4000, log);
        Check(Math.Abs(heading[0].End.x - half) < .01f, "Road heading off the map did not reach the edge");
        Check(
            NetworkDeadEnds.ReachBorder(new[] { Road(1, 1, 2, 0, 0, half - 400, 0) }, 4000, log)[0].End.x == half - 400,
            "Road too far from the edge was extended");

        Check(
            NetworkDeadEnds.ReachBorder(new[] { Road(1, 1, 2, half - 100, -500, half - 100, 0) }, 4000, log)[0].End.x
                == half - 100,
            "Road running beside the edge was turned to it");
    }

    // Unit cuts round each side on its own; the game joins new nodes only on an exact position.
    static void Exact(ImportLog log)
    {
        var firstSegment = Road(1, 1, 2, 0, 0, 100, 0);
        var secondSegment = Road(1, 2, 3, 100.0001f, 0, 200, 0);
        var fixedUp = NativeEndpointPolicy.Canonicalize(new[] { firstSegment, secondSegment }, log);
        Check(math.all(fixedUp[0].End == fixedUp[1].Start), "Shared node left with two positions");
    }

    // A section inside a bore draws as tunnel only at -10 m or below, with -15 m somewhere.
    static void TunnelElevation(ImportLog log)
    {
        const int n = 257;
        const float scale = 200, width = 2000;
        var flat = Enumerable.Repeat(100f, n * n).ToArray();

        PreparedSegment At(long a, long b, float x1, float y1, float x2, float y2, bool tunnel)
            => new PreparedSegment(
                a,
                a,
                b,
                new(x1, y1, 0),
                new(x2, y2, 0),
                NetworkKind.Road,
                "primary",
                2,
                false,
                false,
                tunnel,
                tunnel ? -1 : 0)
            { NativeWidth = 12, StartElevation = tunnel ? y1 - 100 : null, EndElevation = tunnel ? y2 - 100 : null };

        var plan =
            new[] {
                At(1, 2, -100, 100, 0, 92, false),
                At(2, 3, 0, 92, 300, 92, true),
                At(3, 4, 300, 92, 600, 92, true),
                At(4, 5, 600, 97, 900, 100, false)
                };

        var drawn = NativeEndpointPolicy.InteriorTunnelElevations(plan, flat, width, log);
        Check(drawn[1].StartElevation == -8, "Portal end's elevation changed");
        Check(
            drawn[1].EndElevation == -16 && drawn[2].StartElevation == -16,
            "Interior node left too shallow to draw as tunnel");

        Check(drawn[1].End.y == 92 && drawn[2].Start.y == 92, "Elevation change moved the deck");
        var shallow =
            NativeEndpointPolicy.InteriorTunnelElevations(
                new[] { At(2, 3, 0, 98, 300, 98, true), At(3, 4, 300, 98, 600, 98, true) },
                flat,
                width,
                log);

        Check(shallow[0].EndElevation == -2, "Bore without ground over it forced to tunnel");
    }

    // The game walls a ground section off the terrain and draws a whole edge as one
    // composition, so pieces are told what to be: on flat ground at 100 m.
    static void Drawn(ImportLog log)
    {
        const int n = 257;
        const float scale = 200, width = 2000;
        var flat = Enumerable.Repeat(100f, n * n).ToArray();

        PreparedSegment At(
            long a,
            long b,
            float x1,
            float y1,
            float x2,
            float y2,
            bool tunnel = false,
            bool bridge = false,
            float z = 0)
                => new PreparedSegment(
                    a,
                    a,
                    b,
                    new(x1, y1, z),
                    new(x2, y2, z),
                    NetworkKind.Road,
                    "primary",
                    2,
                    false,
                    bridge,
                    tunnel,
                    tunnel ? -1 : bridge ? 1 : 0)
                { NativeWidth = 12 };

        var deck = At(1, 2, 0, 108, 100, 108, bridge: true);
        Check(
            NetworkDrawnStructures.Prepare(
                new[] { deck, At(2, 3, 100, 108, 130, 106) },
                flat,
                width,
                log)[1].Bridge,
            "Ground piece standing clear beside a bridge stayed ground");

        Check(
            !NetworkDrawnStructures.Prepare(
                new[] { deck, At(2, 3, 100, 102, 130, 101) },
                flat,
                width,
                log)[1].Bridge,
            "Ground piece near the terrain became a bridge");

        Check(
            !NetworkDrawnStructures.Prepare(new[] { At(2, 3, 100, 108, 130, 106) }, flat, width, log)[0].Bridge,
            "Ground piece with no bridge became one");

        Check(
            NetworkDrawnStructures.Finish(new[] { At(2, 3, 100, 108, 130, 106) }, flat, width, log)[0].Bridge,
            "Section clear of the finished ground stayed ground");

        Check(
            !NetworkDrawnStructures.Finish(new[] { At(2, 3, 100, 108, 110, 108) }, flat, width, log)[0].Bridge,
            "Very short section became a viaduct");

        var between =
            NetworkDrawnStructures.Prepare(
                new[] {
                    At(1, 2, 0, 108, 100, 108, bridge: true),
                    At(2, 3, 100, 108, 130, 108),
                    At(3, 4, 130, 108, 230, 108, bridge: true)
                    },
                flat,
                width,
                log);

        Check(between[1].Bridge, "Ground piece between two bridges stayed ground");

        var bore = At(1, 2, 0, 90, 100, 90, tunnel: true);
        Check(
            NetworkDrawnStructures.Prepare(
                new[] { bore, At(2, 3, 100, 90, 150, 90) },
                flat,
                width,
                log)[1].Tunnel,
            "Section deep under the terrain beside a bore stayed open");

        Check(
            !NetworkDrawnStructures.Prepare(
                new[] { bore, At(2, 3, 100, 90, 150, 99) },
                flat,
                width,
                log)[1].Tunnel,
            "Section rising to the surface became a tunnel");

        // A section from the surface to deep ground is cut where the ground covers it.
        var portal =
            new[] {
                At(1, 2, -100, 100, 0, 100),
                At(2, 3, 0, 100, 200, 86, tunnel: true),
                At(3, 4, 200, 86, 400, 86, tunnel: true)
                };

        var cut = NetworkDrawnStructures.Finish(portal, flat, width, log);
        Check(cut.Count == 4, "Portal section not cut");
        var mouth = cut.Single(segment => segment.Tunnel && segment.StartNodeId == 2);
        var inside = cut.Single(segment => segment.Tunnel && segment.StartNodeId == mouth.EndNodeId);
        Check(
            mouth.EndElevation == -16 && inside.StartElevation == -16 && inside.EndElevation == -16,
            "Bore past the mouth not drawn as tunnel");

        Check(math.distance(mouth.CourseEnd, inside.CourseStart) < .001f, "Cut left two nodes");
        var along = mouth.CourseEnd.x;
        // 14 m of fall over 200 m gives the 10 m a cut's end needs at 143 m.
        Check(along >= 143 && along < 160, "Cut not where the ground first covers the bore: " + along);
        Check(math.abs(mouth.CourseEnd.y - (100 - 14 * along / 200)) < .01f, "Cut moved the deck");
        // A covered part of a ground section joins the bore; the rest stays open.
        var partial =
            NetworkDrawnStructures.Prepare(new[] { bore, At(2, 3, 100, 90, 300, 100) }, flat, width, log);

        Check(
            partial.Count == 3 && partial.Count(segment => segment.Tunnel) == 2,
            "Covered part of a ground section not joined to the bore");

        // Ground too thin for the game to draw a tunnel stays predicted open however it is forced.
        var thin =
            NativeEndpointPolicy.InteriorTunnelElevations(
                new[] {
                    At(1, 2, 0, 91, 100, 91, tunnel: true),
                    At(2, 3, 100, 91, 200, 91, tunnel: true),
                    At(3, 4, 200, 91, 300, 91, tunnel: true)
                    },
                flat,
                width,
                log);

        Check(
            NetworkDrawnStructures.Predict(thin, flat, width).TunnelKm == 0,
            "Bore with 9 m of ground predicted as tunnel");

        // Too short to leave two pieces.
        Check(
            NetworkDrawnStructures.Finish(
                new[] {
                    At(1, 2, -100, 100, 0, 100),
                    At(2, 3, 0, 90, 30, 90, tunnel: true),
                    At(3, 4, 30, 90, 300, 90, tunnel: true)
                    },
                flat,
                width,
                log).Count
                == 3,
            "Short portal section cut");

        // Two bores side by side, the second covered sooner (10 m of ground at 95 m, not
        // 143 m): under smooth-roads its mouth moves in beside the first one's.
        var twin =
            new[] {
                At(1, 2, -100, 100, 0, 100),
                At(2, 3, 0, 100, 200, 86, tunnel: true),
                At(3, 4, 200, 86, 400, 86, tunnel: true),
                At(11, 12, -100, 100, 0, 100, z: 14),
                At(12, 13, 0, 100, 200, 79, tunnel: true, z: 14),
                At(13, 14, 200, 79, 400, 79, tunnel: true, z: 14)
                };

        float Mouth(IReadOnlyList<PreparedSegment> sections, long node) =>
            sections.Single(segment => segment.Tunnel && segment.StartNodeId == node).CourseEnd.x;

        var apart = NetworkDrawnStructures.Finish(twin, flat, width, log);
        Check(Mouth(apart, 2) - Mouth(apart, 12) > 30, "Twin bores cut at the same point without smooth-roads");

        var switches = DesignSwitches.Active;
        DesignSwitches.Set(DesignSwitches.SmoothRoads);
        var abreast = NetworkDrawnStructures.Finish(twin, flat, width, log);
        DesignSwitches.Set(switches);

        Check(
            math.abs(Mouth(abreast, 2) - Mouth(abreast, 12)) < 6 && Mouth(abreast, 2) == Mouth(apart, 2),
            "Twin tunnel mouths not side by side: " + Mouth(abreast, 2) + " and " + Mouth(abreast, 12));

        var predicted = NetworkDrawnStructures.Predict(cut, flat, width);
        Check(predicted.TunnelKm > .2 && predicted.OpenBoreKm < .17, "Prediction of what draws as tunnel");
    }

    // Two opposing one-way legs leaving a node and meeting again are one road.
    static PreparedSegment Carriageway(long way, long a, long b, float x1, float z1, float x2, float z2)
        => Road(way, a, b, x1, z1, x2, z2) with { OneWay = true, Name = "Boulevard" };

    static float OneWayLength(IEnumerable<PreparedSegment> sections)
        => sections
            .Where(segment => segment.OneWay)
            .Sum(segment => math.distance(segment.Start.xz, segment.End.xz));

    // Two 1.2 km carriageways of one named road, eastbound on z = 0 and westbound at the
    // given offsets, in 100 m sections. Nodes 100-112 and 200-212 sit at x = 100 k.
    static List<PreparedSegment> NamedPair(Func<int, float> offset)
    {
        var source = new List<PreparedSegment>();

        for (var k = 0; k < 12; k++)
            source.Add(Carriageway(10 + k, 100 + k, 101 + k, k * 100, 0, (k + 1) * 100, 0));

        for (var k = 12; k > 0; k--)
            source.Add(Carriageway(40 + k, 200 + k, 199 + k, k * 100, offset(k), (k - 1) * 100, offset(k - 1)));

        return source;
    }

    // A named pair is not kept divided over one place where its carriageways stray apart
    // (Quebec City's boulevards). These run 34 m apart, past the local pass's 32 m, and 70 m
    // apart at x = 600: the stretches either side of that bulge still merge.
    static void StrayingCarriageways()
    {
        var source = NamedPair(k => k == 6 ? 70 : 34);
        var before = OneWayLength(source);
        var oneWay = OneWayLength(AvenueConsolidator.Prepare(source, new ImportLog(), default));

        Check(
            oneWay < before / 3,
            $"Carriageways straying apart at one place kept {oneWay:F0} of {before:F0} m divided");
    }

    // A long named pair refused for one junction conflict is split and its halves matched
    // again. Motorway ramps fixed on both carriageways at x = 300 conflict on the merged road.
    static void ConflictingJunctionHalves()
    {
        var source = NamedPair(_ => 34);
        source.Add(Road(90, 300, 103, 300, -300, 300, 0, "motorway"));
        source.Add(Road(91, 203, 301, 300, 34, 300, 300, "motorway"));
        var log = new ImportLog();
        var result = AvenueConsolidator.Prepare(source, log, default);

        Check(
            log.Metric("Avenue pairs retained for junction conflicts") > 0,
            "The junction conflict fixture no longer conflicts");

        Check(
            !result.Any(segment => segment.OneWay && Math.Min(segment.Start.x, segment.End.x) >= 600),
            "A junction conflict at one end kept a whole named pair divided");
    }

    // A reversible centre lane between the one-way carriageways of the same street is left
    // out (Montreal's Rue Bridge); a reversible street with no such companions stays.
    static void ReversibleCentreLane(ImportRequest request)
    {
        var nodes = new Dictionary<long, OsmNode>();
        var ways = new Dictionary<long, OsmWay>();

        OsmWay Way(long id, double latitude, string name, string oneWay, bool reverse = false)
        {
            var first = id * 10;
            nodes[first] = new(first, latitude, reverse ? .004 : 0);
            nodes[first + 1] = new(first + 1, latitude, .002);
            nodes[first + 2] = new(first + 2, latitude, reverse ? 0 : .004);

            return ways[id] =
                new(
                    id,
                    new[] { first, first + 1, first + 2 },
                    new Dictionary<string, string> { { "highway", "primary" }, { "name", name }, { "oneway", oneWay } });
        }

        Way(1, 0, "Bridge", "yes");
        Way(2, .0002, "Bridge", "yes", reverse: true);
        Way(3, .0001, "Bridge", "reversible");
        Way(4, .01, "Lone", "reversible");

        var log = new ImportLog();
        var segments = NetworkProcessor.BuildSegments(new OsmDataset { Nodes = nodes, Ways = ways }, request, log);

        Check(
            !segments.Any(segment => segment.WayId == 3)
                && segments.Count(segment => segment.WayId == 4) == 2
                && segments.Count(segment => segment.WayId is 1 or 2) == 4,
            "A reversible centre lane was kept, or a reversible street without companions was dropped");
    }

    static void SplitRoads(ImportLog log)
    {
        var approach = Road(1, 1, 2, -300, 0, 0, 0);
        var leave = Road(2, 3, 4, 200, 0, 500, 0);
        var eye =
            new[] {
                approach,
                leave,
                OneWayRoad(3, 2, 5, 0, 0, 100, 12),
                OneWayRoad(3, 5, 3, 100, 12, 200, 0),
                OneWayRoad(4, 3, 6, 200, 0, 100, -12),
                OneWayRoad(4, 6, 2, 100, -12, 0, 0)
                };

        var merged = NetworkSplitRoads.Prepare(eye, log, default);
        Check(merged.All(segment => !segment.OneWay), "Eye left as one-way legs");
        Check(
            merged.Sum(segment => math.distance(segment.Start.xz, segment.End.xz)) < 801
                && merged.Any(segment => segment.StartNodeId == 2)
                && merged.Any(segment => segment.EndNodeId == 3),
            "Eye not joined node to node");

        Check(
            merged
                .Where(segment => segment.WayId != 1 && segment.WayId != 2)
                .All(segment => Math.Abs(segment.Start.z) < .5f && Math.Abs(segment.End.z) < .5f),
            "Merged road not on the legs' middle");

        // A side street on one leg moves onto the merged road and stays connected.
        var side =
            new[] {
                approach,
                leave,
                OneWayRoad(3, 2, 5, 0, 0, 100, 12),
                OneWayRoad(3, 5, 3, 100, 12, 200, 0),
                OneWayRoad(4, 3, 6, 200, 0, 100, -12),
                OneWayRoad(4, 6, 2, 100, -12, 0, 0),
                Road(5, 5, 7, 100, 12, 100, 200, "tertiary")
                };

        var moved = NetworkSplitRoads.Prepare(side, log, default);
        var arm = moved.Single(segment => segment.WayId == 5);
        Check(
            Math.Abs(arm.Start.z) < .5f
                && moved.Any(segment => segment.WayId != 5 && segment.EndNodeId == arm.StartNodeId),
            "Side street left behind by the merge");

        // Not merged: a real wide median, a through one-way street, a motorway's ramps.
        var wide =
            new[] {
                approach,
                leave,
                OneWayRoad(3, 2, 5, 0, 0, 100, 70),
                OneWayRoad(3, 5, 3, 100, 70, 200, 0),
                OneWayRoad(4, 3, 6, 200, 0, 100, -70),
                OneWayRoad(4, 6, 2, 100, -70, 0, 0)
                };

        Check(
            NetworkSplitRoads.Prepare(wide, log, default).Count(segment => segment.OneWay) == 4,
            "Wide median merged");

        var through = new[] { OneWayRoad(1, 1, 2, -300, 0, 0, 0), OneWayRoad(2, 2, 3, 0, 0, 300, 0) };
        Check(
            NetworkSplitRoads.Prepare(through, log, default).Count(segment => segment.OneWay) == 2,
            "Through one-way street merged");

        var ramps =
            new[] {
                approach,
                leave,
                OneWayRoad(3, 2, 5, 0, 0, 100, 12) with { Classification = "motorway_link" },
                OneWayRoad(3, 5, 3, 100, 12, 200, 0) with { Classification = "motorway_link" },
                OneWayRoad(4, 3, 6, 200, 0, 100, -12) with { Classification = "motorway_link" },
                OneWayRoad(4, 6, 2, 100, -12, 0, 0) with { Classification = "motorway_link" }
                };

        Check(NetworkSplitRoads.Prepare(ramps, log, default).Count(segment => segment.OneWay) == 4, "Ramps merged");
        // Legs branching from two nodes a short step apart on a cross street.
        var cross =
            new[] {
                Road(6, 10, 2, 0, -200, 0, 0, "secondary"),
                Road(6, 2, 8, 0, 0, 0, 20, "secondary"),
                Road(6, 8, 11, 0, 20, 0, 200, "secondary"),
                OneWayRoad(3, 2, 5, 0, 0, 100, 8),
                OneWayRoad(3, 5, 3, 100, 8, 200, 8),
                OneWayRoad(4, 4, 6, 200, 28, 100, 28),
                OneWayRoad(4, 6, 8, 100, 28, 0, 20),
                Road(7, 3, 4, 200, 8, 200, 28, "secondary"),
                Road(8, 3, 12, 200, 8, 400, 8),
                Road(8, 12, 13, 400, 8, 700, 8)
                };

        var near = NetworkSplitRoads.Prepare(cross, log, default);
        Check(near.Count(segment => segment.OneWay) == 0, "Pair between two cross-street nodes left as one-way");
        Check(
            near.Where(segment => segment.WayId == 6).All(segment => segment.StartNodeId != segment.EndNodeId),
            "Cross street collapsed on itself");

        // A junction mapped as a small box (Montreal, way 436364814): the legs flare around the
        // cross street and rejoin 30 m on. The crossing becomes one junction on a two-way road.
        var box =
            new[] {
                Road(1, 1, 2, -300, 0, 0, 0, "secondary"),
                Road(2, 3, 4, 30, 0, 300, 0, "secondary"),
                OneWayRoad(3, 2, 5, 0, 0, 8, -12),
                OneWayRoad(3, 5, 6, 8, -12, 22, -10),
                OneWayRoad(3, 6, 3, 22, -10, 30, 0),
                OneWayRoad(4, 3, 7, 30, 0, 22, 10),
                OneWayRoad(4, 7, 8, 22, 10, 8, 12),
                OneWayRoad(4, 8, 2, 8, 12, 0, 0),
                Road(5, 9, 6, 22, -100, 22, -10, "tertiary"),
                Road(5, 6, 7, 22, -10, 22, 10, "tertiary"),
                Road(5, 7, 10, 22, 10, 22, 100, "tertiary")
                };

        var crossing = NetworkSplitRoads.Prepare(box, log, default);
        var south = crossing.Single(segment => segment.StartNodeId == 9);
        var north = crossing.Single(segment => segment.EndNodeId == 10);
        Check(crossing.All(segment => !segment.OneWay), "Box junction left as one-way legs");
        Check(
            south.EndNodeId == north.StartNodeId && crossing.Count(segment => segment.WayId == 5) == 2,
            "Cross street does not meet the box's road at one junction");

        // Without a road beyond the far end the same shape is a turning loop and stays.
        var loop = box.Where(segment => segment.WayId != 2).ToArray();
        Check(
            NetworkSplitRoads.Prepare(loop, log, default).Count(segment => segment.OneWay) == 6,
            "Turning loop merged as a box junction");
    }

    static void BlindOneWay(ImportLog log)
    {
        var kept =
            NetworkDeadEnds.RemoveBlindOneWays(
                new[] {
                    Road(1, 1, 2, -500, 0, 0, 0),
                    Road(2, 2, 3, 0, 0, 500, 0),
                    OneWayRoad(3, 2, 4, 0, 0, 0, 100),
                    OneWayRoad(3, 4, 5, 0, 100, 0, 200)
                    },
                4000,
                log);

        Check(kept.Count == 2 && kept.All(segment => segment.WayId != 3), "Blind one-way run survived");
        Check(
            NetworkDeadEnds.RemoveBlindOneWays(
                new[] { Road(1, 1, 2, -500, 0, 0, 0), Road(2, 2, 3, 0, 0, 500, 0), Road(3, 2, 4, 0, 0, 0, 100) },
                4000,
                log).Count
                == 3,
            "Two-way dead end removed");

        Check(
            NetworkDeadEnds.RemoveBlindOneWays(new[] { OneWayRoad(1, 1, 2, 0, 0, 0, 100) }, 4000, log).Count == 1,
            "Isolated one-way removed with nothing to alias to");

        Check(
            NetworkDeadEnds.RemoveBlindOneWays(
                new[] { Road(1, 1, 2, -500, 0, 0, 0), Road(2, 2, 3, 0, 0, 500, 0), OneWayRoad(3, 2, 4, 0, 0, 1995, 0) },
                4000,
                log).Count
                == 3,
            "One-way reaching the edge removed");
    }

    static void LimitedAccess(ImportLog log)
    {
        OsmDataset Data(IEnumerable<OsmWay> ways, params long[] signals)
            => new()
            {
                Ways = ways.ToDictionary(way => way.Id),
                NodeTags =
                    signals.ToDictionary(
                        id => id,
                        id => (IReadOnlyDictionary<string, string>)new Dictionary<string, string>
                        {
                            ["highway"] = "traffic_signals"
                        })
            };

        PreparedSegment Trunk(long a, long b, float x1, float x2) => Road(a, a, b, x1, 0, x2, 0, "trunk");

        // An arterial: 3 km of trunk with one signal in the middle and no ramps.
        var arterial = new[] { Trunk(1, 2, 0, 1500), Trunk(2, 3, 1500, 3000) };
        Check(
            NetworkLimitedAccess
                .Apply(arterial, Data(Array.Empty<OsmWay>(), 2), log)
                .All(segment => segment.Classification == "primary"),
            "Arterial stayed a highway");

        // Interchanges at both ends of an unbroken stretch.
        var ramp1 = Road(10, 2, 20, 1500, 0, 1500, 100, "trunk_link");
        var ramp2 = Road(11, 3, 21, 3000, 0, 3000, 100, "trunk_link");
        var freeway =
            NetworkLimitedAccess.Apply(
                new[] { Trunk(1, 2, 0, 1500), Trunk(2, 3, 1500, 3000), Trunk(3, 4, 3000, 4000), ramp1, ramp2 },
                Data(Array.Empty<OsmWay>()),
                log);

        Check(
            freeway.Where(segment => !segment.ProtectedLink).All(segment => segment.Classification == "trunk"),
            "Freeway converted");

        Check(
            freeway.Where(segment => segment.ProtectedLink).All(segment => segment.Classification == "trunk_link"),
            "Ramp changed class");

        // A side street at grade cuts the stretch; one passing over on a bridge does not.
        var side = TaggedWay(50, "residential", 2, 99);
        Check(
            NetworkLimitedAccess
                .Apply(arterial, Data(new[] { side }), log)
                .All(segment => segment.Classification == "primary"),
            "At-grade side street ignored");

        var over =
            side with { Tags = new Dictionary<string, string> { ["highway"] = "residential", ["bridge"] = "yes" } };

        Check(
            NetworkLimitedAccess
                .Apply(new[] { Trunk(1, 2, 0, 1500), Trunk(2, 3, 1500, 3000), ramp1, ramp2 }, Data(new[] { over }), log)
                .Where(segment => !segment.ProtectedLink)
                .All(segment => segment.Classification == "trunk"),
            "Overpass counted as at grade");

        // A long bridge or tunnel stays, with no ramps.
        Check(
            NetworkLimitedAccess.Apply(
                new[] { Trunk(1, 2, 0, 800) with { Tunnel = true } },
                Data(Array.Empty<OsmWay>()),
                log)[0].Classification
                == "trunk",
            "Tunnel converted");

        Check(
            NetworkLimitedAccess.Converted(
                arterial[0] with
                {
                    Classification = "primary",
                    RawSourceTags = new Dictionary<string, string> { ["highway"] = "trunk" }
                }),
            "Converted trunk not recognised");
    }

    static void Streets(ImportLog log, ImportRequest request)
    {
        OsmDataset Data(params OsmWay[] ways)
        {
            var nodes = new Dictionary<long, OsmNode>();

            foreach (var (id, lat, lon) in new (long, double, double)[] {
                (1, 0, 0),
                (2, 0, .001),
                (3, 0, .002),
                (4, 0, .003),
                (5, 0, .009)
                })
                nodes[id] = new OsmNode(id, lat, lon);

            return new OsmDataset
            {
                Nodes = nodes,
                Ways = ways.ToDictionary(way => way.Id),
                RawWayTags = ways.ToDictionary(way => way.Id, way => way.Tags)
            };
        }

        var widths = new Dictionary<string, float>();
        var offsets = new Dictionary<string, float[]>();
        var ramp = Road(1, 1, 2, 0, 0, 111, 0, "motorway_link") with { OneWay = true, SourceWayIds = new long[] { 1 } };
        var target = Road(2, 4, 6, 334, 0, 334, 500);
        var reachable =
            NetworkDeadEnds.ConnectThroughStreets(
                new[] { ramp, target },
                Data(
                    TaggedWay(1, "motorway_link", 1, 2),
                    TaggedWay(2, "primary", 4, 6),
                    TaggedWay(3, "residential", 2, 3, 4)),
                request,
                widths,
                offsets,
                log,
                default);

        Check(
            reachable.Count(segment => segment.WayId == 3) == 2
                && reachable
                    .Where(segment => segment.WayId == 3)
                    .All(segment => !segment.OneWay && segment.Classification == "tertiary"),
            "Street between a ramp and a road not imported");

        var far =
            NetworkDeadEnds.ConnectThroughStreets(
                new[] { ramp, Road(2, 5, 6, 1000, 0, 1000, 500) },
                Data(
                    TaggedWay(1, "motorway_link", 1, 2),
                    TaggedWay(2, "primary", 5, 6),
                    TaggedWay(3, "residential", 2, 5)),
                request,
                widths,
                offsets,
                log,
                default);

        Check(far.Count == 2, "Street beyond its reach imported");
        var path =
            NetworkDeadEnds.ConnectThroughStreets(
                new[] { ramp, target },
                Data(
                    TaggedWay(1, "motorway_link", 1, 2),
                    TaggedWay(2, "primary", 4, 6),
                    TaggedWay(3, "footway", 2, 3, 4)),
                request,
                widths,
                offsets,
                log,
                default);

        Check(path.Count == 2, "Footway imported as a road");

        // The road's node 4 was replaced by a merge (node 99, as an avenue merge aliases it): the
        // street must end on the live node, not on the node no section uses any more.
        var merged =
            Road(2, 99, 6, 334, 0, 334, 500) with
            {
                SourceWayIds = new long[] { 2 },
                SourceNodeAliases = new[] { new SourceNodeAlias(4, 99, false) }
            };

        var rejoined =
            NetworkDeadEnds.ConnectThroughStreets(
                new[] { ramp, merged },
                Data(
                    TaggedWay(1, "motorway_link", 1, 2),
                    TaggedWay(2, "primary", 4, 6),
                    TaggedWay(3, "residential", 2, 3, 4)),
                request,
                widths,
                offsets,
                log,
                default);

        Check(
            rejoined.Any(segment => segment.WayId == 3 && (segment.StartNodeId == 99 || segment.EndNodeId == 99))
                && rejoined.All(segment => segment.StartNodeId != 4 && segment.EndNodeId != 4),
            "Street connector ended on a node a merge had replaced");
    }

    // Flat ground at 100 m; a bore 8 m down from x=0 to x=600 with open approaches.
    static void Roadbed()
    {
        const int n = 257;
        const float scale = 200, width = 2000;
        var flat = Enumerable.Repeat(100f, n * n).ToArray();

        float Read(float[] t, float x, float z)
            => t[(int)Math.Round((z / width + .5) * (n - 1)) * n + (int)Math.Round((x / width + .5) * (n - 1))];

        PreparedSegment At(
            long way,
            long a,
            long b,
            float x1,
            float y1,
            float x2,
            float y2,
            float z = 0,
            bool tunnel = false) =>
            new PreparedSegment(
                way,
                a,
                b,
                new(x1, y1, z),
                new(x2, y2, z),
                NetworkKind.Road,
                "primary",
                2,
                false,
                false,
                tunnel,
                tunnel ? -1 : 0)
            { NativeWidth = 12 };

        var bore = At(1, 2, 3, 0, 92, 600, 92, 0, true);
        var approach = At(2, 1, 2, -300, 100, 0, 92);
        var exit = At(3, 3, 4, 600, 92, 900, 100);
        var plain =
            NetworkRoadbed.Prepare(
                flat,
                scale,
                width,
                new[] { bore, approach, exit },
                Array.Empty<float3[]>(),
                Array.Empty<float3[]>(),
                new ImportLog(),
                default);

        Check(Read(plain, 300, 0) > 101.5f, "Bore's middle lost its cover");
        Check(Read(plain, 8, 0) < 101, "Cover stood as a block over the portal");
        Check(Read(plain, -8, 0) < 97, "Approach cutting not stamped up to the portal");
        // A ground road beside the bore keeps its level; cover falls to it.
        var beside = At(4, 5, 6, 100, 100, 500, 100, 30);
        var withRoad =
            NetworkRoadbed.Prepare(
                flat,
                scale,
                width,
                new[] { bore, approach, exit, beside },
                Array.Empty<float3[]>(),
                Array.Empty<float3[]>(),
                new ImportLog(),
                default);

        Check(Read(withRoad, 300, 30) <= 100.5f, "Tunnel cover buried a ground road beside it");
    }

    private static PreparedSegment OneWayRoad(long way, long a, long b, float x1, float z1, float x2, float z2)
        => Road(way, a, b, x1, z1, x2, z2) with { OneWay = true };

    private static OsmWay TaggedWay(long id, string highway, params long[] nodes)
        => new(id, nodes, new Dictionary<string, string> { ["highway"] = highway });
}
