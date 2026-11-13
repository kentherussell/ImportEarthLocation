using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

// Shapes that built as disconnected or unusable pieces in game: two rail copies
// between one node pair, sub-lane unit pieces beside a node, and continuity
// counting merged interior nodes as losses.
static class ConnectivityTests
{
    static PreparedSegment Rail(long way, long a, long b, float3 point, float3 otherPoint) =>
        new(
            way,
            a,
            b,
            point,
            otherPoint,
            NetworkKind.Rail,
            "rail",
            2,
            false,
            false,
            false,
            0)
        {
            NativeWidth = 11,
            NativeAssetKey = "rail",
            PhysicalTracks = 2,
            SourceGeometry = new[] { point, otherPoint },
            OriginalGeometry = new[] { point, otherPoint }
        };

    static PreparedSegment Road(long way, long a, long b, float3 point, float3 otherPoint) =>
        new(
            way,
            a,
            b,
            point,
            otherPoint,
            NetworkKind.Road,
            "trunk",
            2,
            false,
            false,
            false,
            0)
        {
            NativeWidth = 12,
            NativeAssetKey = "road",
            SourceGeometry = new[] { point, otherPoint },
            OriginalGeometry = new[] { point, otherPoint }
        };

    public static void Run()
    {
        ReservedRailIdentity();
        var ground = Rail(1, 10, 11, new(0, 50, 0), new(40, 50, 0));
        var bridge = Rail(2, 11, 10, new(40, 50, .5f), new(0, 50, .5f)) with { Bridge = true };
        var loop =
            Rail(
                3,
                10,
                11,
                new(0, 50, 0),
                new(40, 50, 30)) with
            { Control1 = new float3(10, 50, 30), Control2 = new float3(30, 50, 30) };

        var onward = Rail(4, 11, 12, new(40, 50, 0), new(90, 50, 0));
        var kept =
            RailCorridorConsolidator.RedundantParallelSpans(
                new[] { ground, bridge, loop, onward },
                new ImportLog(),
                default);

        Check(
            kept.Count == 3 && kept.Count(segment => segment.WayId == 3) == 1,
            "Rail copies between one node pair were not reduced to one, or a separate loop was dropped");

        var survivor = kept.Single(segment => segment.WayId is 1 or 2);
        Check(
            survivor.Bridge && survivor.SourceWayIds.Contains(1) && survivor.SourceWayIds.Contains(2),
            "The kept copy lost its structure or the dropped copy's provenance");

        Check(
            survivor.SourceNodeAliases.Any(alias => alias.SourceNodeId == 10 && alias.CanonicalNodeId == 10),
            "A junction made only by the copy was not kept as a port");

        // Whole routes through degree-two nodes, not just single sections. An
        // alias aimed at the dropped route's interior follows it to an end; a
        // route crossing a road at grade is never dropped.
        var spur = Rail(9, 40, 44, new(0, 50, 0), new(-60, 50, 0));
        var tail = Rail(10, 43, 45, new(100, 50, 0), new(160, 50, 0));
        var routeA =
            new[] { Rail(5, 40, 41, new(0, 50, 0), new(50, 50, 0)), Rail(6, 41, 43, new(50, 50, 0), new(100, 50, 0)) };

        var routeB =
            new[] {
                Rail(7, 40, 42, new(0, 50, .6f), new(45, 50, .6f)),
                Rail(
                    8,
                    42,
                    43,
                    new(45, 50, .6f),
                    new(100, 50, .6f)) with { SourceNodeAliases = new[] { new SourceNodeAlias(99, 42, true) } }
                };

        var routes =
            RailCorridorConsolidator.RedundantParallelSpans(
                routeA.Concat(routeB).Append(spur).Append(tail).ToArray(),
                new ImportLog(),
                default);

        Check(
            routes.Count == 4 && routes.All(segment => segment.WayId is not (7 or 8)),
            "A duplicate two-section rail route between one junction pair survived");

        var routeAliases = routes.SelectMany(segment => segment.SourceNodeAliases).ToArray();
        Check(
            routeAliases.Any(alias => alias.SourceNodeId == 99 && alias.CanonicalNodeId is 40 or 43)
                && routeAliases.Any(alias => alias.SourceNodeId == 42 && alias.CanonicalNodeId == 40),
            "The dropped route's interior node or an alias aimed at it was not carried to an end");

        Check(
            routes
                .Where(segment => segment.WayId is 5 or 6)
                .All(segment => segment.SourceWayIds.Contains(7) && segment.SourceWayIds.Contains(8)),
            "The kept route did not carry the dropped route's ways");

        var crossing = Road(30, 42, 46, new(45, 50, -20), new(45, 50, 20));
        var crossed =
            RailCorridorConsolidator.RedundantParallelSpans(
                routeA.Concat(routeB).Append(spur).Append(tail).Append(crossing).ToArray(),
                new ImportLog(),
                default);

        Check(
            crossed.Count(segment => segment.Kind == NetworkKind.Rail) == 6,
            "A rail route with a level crossing was dropped");

        // A connector lying inside another line's footprint goes when its ends
        // stay connected nearby; its way rides on that path. With a level
        // crossing on it, it stays.
        PreparedSegment Narrow(PreparedSegment segment) => segment with { NativeWidth = 4 };

        var stub = Narrow(Rail(70, 79, 71, new(-100, 50, 0), new(0, 50, 0)));
        var main = Narrow(Rail(71, 71, 72, new(0, 50, 0), new(200, 50, 0)));
        var link = Narrow(Rail(72, 72, 75, new(200, 50, 0), new(300, 50, 6)));
        var lineIn = Narrow(Rail(73, 76, 74, new(-100, 50, 6), new(150, 50, 6)));
        var lineOut = Narrow(Rail(74, 74, 75, new(150, 50, 6), new(300, 50, 6)));
        var tailOut = Narrow(Rail(76, 75, 77, new(300, 50, 6), new(400, 50, 6)));
        var connector = Narrow(Rail(75, 71, 78, new(0, 50, 0), new(75, 50, 3)));
        var connectorEnd = Narrow(Rail(77, 78, 74, new(75, 50, 3), new(150, 50, 6)));
        var yard = new[] { stub, main, link, lineIn, lineOut, tailOut, connector, connectorEnd };
        var cleared = RailCorridorConsolidator.RedundantOverlappingRoutes(yard, new ImportLog(), default, 35);
        Check(
            cleared.All(segment => segment.WayId is not (75 or 77)) && cleared.Count == 6,
            "A connector inside another line's footprint was not removed");

        Check(
            cleared.Single(segment => segment.WayId == 71).SourceWayIds.Contains(75)
                && cleared.Single(segment => segment.WayId == 74).SourceWayIds.Contains(77),
            "The removed connector's ways did not move onto the alternative path");

        Check(
            cleared
                .SelectMany(segment => segment.SourceNodeAliases)
                .Any(alias => alias.SourceNodeId == 78 && alias.CanonicalNodeId is 71 or 74),
            "The connector's interior node was not aliased to an end");

        var guarded =
            RailCorridorConsolidator.RedundantOverlappingRoutes(
                yard.Append(Road(80, 78, 90, new(75, 50, 3), new(75, 50, 40))).ToArray(),
                new ImportLog(),
                default,
                35);

        Check(
            guarded.Count(segment => segment.Kind == NetworkKind.Rail) == 8,
            "A connector with a level crossing was removed");

        // Wherever a junction core ends along the arm, no unit cut may leave a
        // piece shorter than a lane beside an existing node.
        foreach (var minimum in new Func<PreparedSegment, double>?[] { null, _ => 20 })
            for (var first = 14f; first <= 40; first += .1f)
            {
                var arms = new[]{Road(10,1,2,new(0,0,0),new(first,0,0)),Road(11,2,3,new(first,0,0),new(400,0,0)),
                    Road(12,1,4,new(0,0,0),new(-300,0,0)),Road(13,1,5,new(0,0,0),new(0,0,300))};

                var units =
                    NativeDesignUnits.Prepare(
                        arms,
                        Array.Empty<NativeAssetContract>(),
                        new ImportLog(),
                        default,
                        minimum);

                var shortest =
                    units
                        .Where(
                            segment => segment.StartNodeId < long.MinValue + 13000000
                                || segment.EndNodeId < long.MinValue + 13000000)
                    .Select(segment => math.distance(segment.CourseStart.xz, segment.CourseEnd.xz))
                        .DefaultIfEmpty(float.MaxValue)
                        .Min();

                Check(
                    shortest >= 3 - 1e-3,
                    $"A native unit cut left a {shortest:F2} m piece beside a node (first span {first:F1} m)");

                var junction =
                    units
                        .Where(segment => segment.StartNodeId == 1 || segment.EndNodeId == 1)
                        .Select(segment => segment.NativeUnitId)
                        .Distinct()
                        .Count();

                Check(junction == 1, "A junction's arms were split across native units");
            }

        // A merged interior node is not a loss while its way survives; a way
        // with no surviving section still is.
        var firstSegment = Road(20, 30, 31, new(0, 0, 0), new(50, 0, 0));
        var secondSegment = Road(21, 31, 32, new(50, 0, 0), new(100, 0, 0));
        var merged =
            firstSegment with { EndNodeId = 32, End = new float3(100, 0, 0), SourceWayIds = new long[] { 20, 21 } };

        var log = new ImportLog();
        NetworkContinuity.Validate(
            new[] { firstSegment, secondSegment },
            new[] { merged },
            log,
            default,
            allowQualityWarnings: true);

        Check(
            log.Metric("Network continuity unresolved sections") == 0,
            "Continuity counted a merged interior node as a lost section");

        log = new ImportLog();
        NetworkContinuity.Validate(
            new[] { firstSegment, secondSegment },
            new[] { firstSegment },
            log,
            default,
            allowQualityWarnings: true);

        Check(
            log.Metric("Network continuity unresolved sections") == 1,
            "Continuity missed a section whose way disappeared");

        // A second track whose ends were merged onto the first leaves a lens: a
        // detour turning 90° between the same two nodes. It goes, whatever its overlap.
        var straight = Narrow(Rail(90, 91, 92, new(0, 50, 0), new(50, 50, 0)));
        var detour =
            new[] {
                Narrow(Rail(91, 91, 93, new(0, 50, 0), new(25, 50, 22))),
                Narrow(Rail(92, 93, 92, new(25, 50, 22), new(50, 50, 0)))
                };

        var beyond =
            new[] {
                Narrow(Rail(93, 94, 91, new(-100, 50, 0), new(0, 50, 0))),
                Narrow(Rail(94, 92, 95, new(50, 50, 0), new(150, 50, 0)))
                };

        var lens =
            RailCorridorConsolidator.RedundantOverlappingRoutes(
                beyond.Append(straight).Concat(detour).ToArray(),
                new ImportLog(),
                default,
                35);

        Check(
            lens.Count == 3
                && lens.All(segment => segment.WayId is not (91 or 92))
                && lens.Single(segment => segment.WayId == 90).SourceWayIds.Contains(91),
            "A 90° rail lens beside a straight track survived or lost its ways");

        // A 15 m spur from a junction to nowhere is a consolidated crossover's
        // remnant and goes, even where the through track bends; a 60 m spur stays.
        var through =
            new[] {
                Narrow(Rail(200, 201, 202, new(0, 50, 0), new(100, 50, 0))),
                Narrow(Rail(201, 202, 203, new(100, 50, 0), new(200, 50, 8))),
                Narrow(Rail(203, 203, 205, new(200, 50, 8), new(300, 50, 8)))
                };

        var shortSpur = Narrow(Rail(202, 202, 204, new(100, 50, 0), new(114, 50, -5)));
        var longSpur = Narrow(Rail(204, 203, 206, new(200, 50, 8), new(256, 50, 28)));
        var spurs =
            RailCorridorConsolidator.RedundantOverlappingRoutes(
                through.Append(shortSpur).Append(longSpur).ToArray(),
                new ImportLog(),
                default,
                2);

        Check(
            spurs.All(segment => segment.WayId != 202)
                && spurs.Any(segment => segment.WayId == 204)
                && spurs.Count == 4,
            "A 15 m rail stub was kept, or a 60 m spur was removed");

        Check(
            spurs
                .SelectMany(segment => segment.SourceNodeAliases)
                .Any(alias => alias.SourceNodeId == 204 && alias.CanonicalNodeId == 202)
                && spurs.Any(segment => segment.SourceWayIds.Contains(202)),
            "The removed stub's dead end or way was not carried to its junction");

        // A switch N meets a 32 m track to switch S and two short connectors,
        // each lying inside a curved line from S. The connectors go one after
        // the other; the second used to leave N a dead end of the 32 m track
        // (Marseille Saint-Charles, DC Union Station). That track is now a spur
        // of S and goes with it; S keeps its other lines.
        var throat = new[]
        {
            Narrow(Rail(300,310,311,new(0,50,0),new(-32,50,0))),Narrow(Rail(301,311,312,new(-32,50,0),new(-200,50,0))),
            Narrow(Rail(302,310,313,new(0,50,0),new(15,50,1))),Narrow(Rail(303,310,314,new(0,50,0),new(23,50,-1))),
            Narrow(Rail(304,311,315,new(-32,50,0),new(-16,50,10))),
                Narrow(Rail(305,315,316,new(-16,50,10),new(0,50,3))),
                Narrow(Rail(306,316,313,new(0,50,3),new(15,50,1))),
            Narrow(Rail(307,311,317,new(-32,50,0),new(-16,50,-10))),
                Narrow(Rail(308,317,318,new(-16,50,-10),new(0,50,-3))),
                Narrow(Rail(309,318,314,new(0,50,-3),new(23,50,-1))),
            Narrow(Rail(310,313,319,new(15,50,1),new(150,50,1))),Narrow(Rail(311,314,320,new(23,50,-1),new(150,50,-30)))
        };

        int Degree(IEnumerable<PreparedSegment> list, long id)
            => list.Count(segment => segment.StartNodeId == id || segment.EndNodeId == id);

        var switches =
            throat
                .SelectMany(segment => new[] { segment.StartNodeId, segment.EndNodeId })
                .Distinct()
                .Where(id => Degree(throat, id) >= 3)
                .ToArray();

        var thinned = RailCorridorConsolidator.RedundantOverlappingRoutes(throat, new ImportLog(), default, 35);
        var stranded = switches.Where(id => Degree(thinned, id) == 1).ToArray();
        Check(
            thinned.All(segment => segment.WayId is not (302 or 303)) && stranded.Length == 0,
            $"Overlapping route removal kept a connector or left former switches {string.Join(",", stranded)} as dead ends");

        Check(
            Degree(thinned, 311) >= 3
                && thinned
                    .SelectMany(segment => segment.SourceNodeAliases)
                    .Any(alias => alias.SourceNodeId == 310 && alias.CanonicalNodeId == 311),
            "The stranded track's switch was not carried to the switch it led to");

        // A road over a short culvert becomes a bridge across it, split out of
        // the section; a road beside it is untouched.
        var culvert = new[] { (new[] { new float3(100, 0, -10), new float3(100, 0, 10) }, 3f) };
        var over = Road(100, 101, 102, new(0, 20, 0), new(300, 20, 0));
        var beside = Road(101, 103, 104, new(0, 20, 60), new(300, 20, 60));
        var bridged =
            NetworkStructureIntervals.CulvertBridges(
                new[] { over, beside },
                culvert,
                4097,
                57344,
                new ImportLog(),
                default);

        var deck = bridged.Where(segment => segment.WayId == 100 && segment.Bridge).ToArray();
        Check(
            bridged.Count(segment => segment.WayId == 100) == 3
                && deck.Length == 1
                && bridged.Single(segment => segment.WayId == 101) == beside,
            "A road over a culvert was not split around one bridge, or a road beside it changed");

        // Half the road (6 m) plus the channel (3 m) and two 14 m raster cells.
        Check(
            Math.Abs(deck[0].CourseStart.x - 63) < .5 && Math.Abs(deck[0].CourseEnd.x - 137) < .5,
            $"Culvert bridge spans {deck[0].CourseStart.x:F1}–{deck[0].CourseEnd.x:F1} m, not 63–137 m");

        Check(
            bridged
                .Where(segment => segment.WayId == 100)
                .SelectMany(segment => new[] { segment.StartNodeId, segment.EndNodeId })
                .GroupBy(id => id)
                .Count(group => group.Count() == 2)
                == 2,
            "Culvert bridge pieces do not share their split nodes");

        // A long open stream, one piece spanning many index cells, is found the same way.
        var stream =
            new[] { (new[] { new float3(100, 0, -1000), new float3(100, 0, 1000), new float3(900, 0, 1000) }, 3f) };

        var spanned =
            NetworkStructureIntervals.CulvertBridges(
                new[] { over, beside },
                stream,
                4097,
                57344,
                new ImportLog(),
                default);

        Check(
            spanned.Count(segment => segment.WayId == 100 && segment.Bridge) == 1
                && spanned.Count(segment => segment.WayId == 101 && segment.Bridge) == 1,
            "A road crossing a long open stream was not bridged once");

        OpenChannelSpans();
        Console.WriteLine(
            "PASS: one rail course per node pair, overlapping rail connectors and lenses removed with provenance, no sub-lane unit pieces beside nodes, alias-aware continuity, and roads bridged over short culverts.");
    }

    static void OpenChannelSpans()
    {
        const float extent = 28 / .6f;
        var channel = new[] { (new[] { new float3(-2000, 0, 0), new float3(2000, 0, 0) }, extent) };
        var across = Road(301, 301, 302, new(0, 100, -300), new(0, 100, 300));
        var along = Road(302, 303, 304, new(-1000, 100, 20), new(1000, 100, 20));
        // These bounds occupy different raw index cells; padded indexing must
        // still find the road whose support overlaps the channel near z=128.
        var edgeChannel = new[] { (new[] { new float3(-1000, 0, 120), new float3(1000, 0, 120) }, extent) };
        var edgeRoad = Road(303, 305, 306, new(-500, 100, 150), new(500, 100, 150));
        var dry = Road(304, 307, 308, new(-500, 100, 240), new(500, 100, 240));
        var rail = Rail(305, 309, 310, new(200, 100, -300), new(200, 100, 300));
        var diagonal = Road(306, 311, 312, new(-500, 100, -100), new(500, 100, 100));
        var input = new[] { across, along, rail, diagonal };
        var output = NetworkStructureIntervals.CulvertBridges(input, channel, 4097, 57344, new ImportLog(), default);
        var crossing = output.Single(segment => segment.WayId == 301 && segment.Bridge);
        var margin = extent + 6 + 28;
        Check(
            crossing.CourseStart.z <= -margin + .01 && crossing.CourseEnd.z >= margin - .01,
            "A wide constructed channel retained the old 50 m bridge cap");

        Check(
            output.Where(segment => segment.WayId == 302).All(segment => segment.Bridge),
            "A road following an opened river was left on a damming embankment");

        Check(
            output.Any(segment => segment.WayId == 305 && segment.Bridge)
                && output.Any(segment => segment.WayId == 306 && segment.Bridge),
            "Rail or an oblique crossing did not span the opened channel");

        foreach (var segment in output.Where(otherSegment => !otherSegment.Bridge))
            Check(
                Enumerable
                    .Range(0, 21)
                    .All(
                        k => Math.Abs(NetworkNormalizer.Evaluate(segment, k / 20f).z)
                            >= extent + Math.Max(4, segment.NativeWidth * .5f) + 28 - .1f),
                "An oblique bridge ended inside the channel's protected ground footprint");

        var edge =
            NetworkStructureIntervals.CulvertBridges(
                new[] { edgeRoad, dry },
                edgeChannel,
                4097,
                57344,
                new ImportLog(),
                default);

        Check(
            edge.Where(segment => segment.WayId == 303).All(segment => segment.Bridge)
                && edge.Single(segment => segment.WayId == 304) == dry,
            "Padded indexing missed an adjacent-cell overlap or changed a dry road");

        var audit = new ImportLog();
        SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(input), output, audit, default);
        Check(
            audit.Metric("Missing mandatory selected-source ports") == 0
                && audit.Metric("Unresolved selected-source reachability obligations") == 0,
            "Opened-channel bridge splits lost a selected-source endpoint or route");

        using var cancellation = new CancellationTokenSource();
        cancellation.Cancel();

        try
        {
            NetworkStructureIntervals.CulvertBridges(input, channel, 4097, 57344, new ImportLog(), cancellation.Token);
            throw new Exception("Channel indexing ignored cancellation");
        }
        catch (OperationCanceledException) { }
    }

    static void ReservedRailIdentity()
    {
        const long retired = -7_800_000_000_000_000_000;
        var axis =
            Rail(
                900,
                900,
                901,
                new(0, 50, 0),
                new(200, 50, 0)) with
            { SourceNodeAliases = new[] { new SourceNodeAlias(retired, 900, true) } };

        var parallel = Rail(901, 902, 903, new(50, 50, 8), new(150, 50, 8));
        var branch = Rail(902, 903, 904, new(150, 50, 8), new(150, 50, 100));
        var output = RailCorridorConsolidator.Prepare(new[] { axis, parallel, branch }, new ImportLog(), default);
        Check(
            output.Any(segment => segment.StartNodeId < 0 || segment.EndNodeId < 0),
            "Rail identity fixture did not transfer an interior junction");

        Check(
            output.All(segment => segment.StartNodeId != retired && segment.EndNodeId != retired),
            "A new rail junction reused a retired source alias identity");

        Check(
            output
                .SelectMany(segment => segment.SourceNodeAliases)
                .Where(alias => alias.SourceNodeId == retired)
                .All(alias => alias.CanonicalNodeId == 900),
            "Rail junction generation redirected an unrelated retained source alias");
    }
}
