using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

static class AvenueTests
{
    static PreparedSegment MakeSegment(long way, long from, long to, float x, float z, float endX, float endZ)
        => new(
            way,
            from,
            to,
            new(x, 0, z),
            new(endX, 0, endZ),
            NetworkKind.Road,
            "primary",
            2,
            true,
            false,
            false,
            0)
        { Name = "Main" };

    public static void Run()
    {
        FragmentedCounterparts();
        DividedCrossing();
        TaperAttachmentGuard();
        StrictLocalMatches();
        StrictJunctionCompletion();
        var firstSegment = MakeSegment(1, 1, 2, 0, 0, 100, 0);
        var secondSegment = MakeSegment(2, 3, 4, 100, 10, 0, 10);

        IReadOnlyList<PreparedSegment> Run(params PreparedSegment[] edges)
            => AvenueConsolidator.Prepare(edges, new ImportLog(), default);

        var pair = Run(firstSegment, secondSegment);
        var longRoad =
            new[] { MakeSegment(20, 20, 21, -100, 0, 0, 0), firstSegment, MakeSegment(21, 2, 22, 100, 0, 200, 0) };

        var partial = Run(longRoad.Concat(new[] { secondSegment }).ToArray());
        Check(
            partial.Any(segment => segment.StartNodeId == 20) && partial.Any(segment => segment.EndNodeId == 22),
            "Partial avenue matching removed unchanged exterior carriageway edges");

        var partialLog = new ImportLog();
        SourceTopologyObligations.Validate(
            SourceTopologyObligations.Capture(longRoad.Concat(new[] { secondSegment }).ToArray()),
            partial,
            partialLog,
            default);

        Check(
            partialLog.Metric("Missing mandatory selected-source ports") == 0
                && partialLog.Metric("Unresolved selected-source reachability obligations") == 0,
            "Partial avenue matching lost a required exterior source connection");

        Check(
            Run(
                firstSegment with { Lanes = 1 },
                secondSegment with
                {
                    Lanes = 1
                }).All(
                    segment => !segment.ConsolidatedAvenue
                        && segment.Lanes == 2
                        && segment.ForwardLanes == 1
                        && segment.BackwardLanes == 1
                        && !segment.OneWay),
            "A 1+1 corridor did not become a regular two-way two-lane road");

        Check(
            Run(firstSegment with { Name = "" }, secondSegment with { Name = "" }).All(segment => !segment.OneWay),
            "Anonymous opposing streets escaped geometric consolidation");

        Check(
            Run(
                firstSegment,
                secondSegment with
                {
                    Classification = "residential"
                }).All(segment => segment.ConsolidatedAvenue && segment.Classification == "primary"),
            "A street-class discrepancy prevented consolidation or lost the through-road class");

        Check(
            Run(
                firstSegment,
                secondSegment with
                {
                    Start = new(100, 0, 25),
                    End = new(0, 0, 25)
                }).All(segment => segment.ConsolidatedAvenue),
            "A wide same-street median prevented consolidation");

        Check(
            Run(
                firstSegment,
                secondSegment with { Start = new(100, 0, 40), End = new(0, 0, 40) }).All(segment => !segment.OneWay),
            "A 40 m divided street escaped strict consolidation");

        Check(
            pair.Count > 0
                && pair.All(
                    segment => segment.ConsolidatedAvenue
                        && !segment.OneWay
                        && segment.Lanes == 4
                        && Math.Abs(segment.Start.z - 5) < .001
                        && Math.Abs(segment.End.z - 5) < .001),
            "Opposing carriageways did not become a four-lane centerline avenue");

        Check(
            math.distance(pair[0].Start.xz, new float2(0, 5)) < .001
                && math.distance(pair[pair.Count - 1].End.xz, new float2(100, 5)) < .001,
            "Avenue discarded corridor endpoints");

        Check(
            Run(secondSegment, firstSegment)
                .Select(
                    segment => (
                        segment.WayId,
                        segment.StartNodeId,
                        segment.EndNodeId,
                        segment.Start,
                        segment.End,
                        segment.Lanes,
                        segment.OneWay))
                .SequenceEqual(
                    pair.Select(
                        segment => (
                            segment.WayId,
                            segment.StartNodeId,
                            segment.EndNodeId,
                            segment.Start,
                            segment.End,
                            segment.Lanes,
                            segment.OneWay))),
            "Input ordering changes consolidation");

        Check(
            Run(firstSegment, secondSegment with { Name = "Different" }).All(segment => !segment.OneWay),
            "Opposing geometric counterparts remained split because their names differ");

        Check(
            Run(
                firstSegment,
                secondSegment with
                {
                    Start = firstSegment.End,
                    End = firstSegment.Start
                }).All(segment => !segment.OneWay),
            "Coincident opposing streets did not become one two-way road");

        Check(
            Run(
                firstSegment with { Reference = "R1" },
                secondSegment with { Name = "", Reference = "R1" }).All(segment => !segment.OneWay),
            "A missing name on one carriageway hid an otherwise identified divided street");

        Check(
            Run(
                firstSegment with { Reference = "R1" },
                secondSegment with { Name = "Different", Reference = "R1" }).All(segment => !segment.OneWay),
            "Naming prevented a geometrically supported avenue");

        var highway = MakeSegment(400, 400, 1, 0, -150, 0, 0) with { Classification = "trunk", Name = "Highway" };
        var anchored = Run(firstSegment, secondSegment, highway);
        Check(
            math.all(anchored.Single(segment => segment.WayId == 400).Start == highway.Start)
                && math.all(anchored.Single(segment => segment.WayId == 400).End == highway.End),
            "Avenue midpoint placement moved an attached trunk road away from its surveyed alignment");

        Check(
            Run(
                firstSegment,
                secondSegment with
                {
                    Start = secondSegment.End,
                    End = secondSegment.Start,
                    StartNodeId = 4,
                    EndNodeId = 3
                }).Count
                == 2,
            "Same-direction carriageways merged");

        Check(
            Run(
                firstSegment with { Classification = "motorway" },
                secondSegment with { Classification = "motorway" }).Count
                == 2,
            "Highways converted into avenues");

        Check(
            Run(firstSegment with { Classification = "trunk" }, secondSegment with { Classification = "trunk" }).Count
                == 2,
            "Trunk highways converted into avenues");

        Check(
            Run(
                firstSegment with { Classification = "primary_link" },
                secondSegment with { Classification = "primary_link" }).Count
                == 2,
            "Ramps converted into avenues");

        Check(
            Run(
                firstSegment with { Bridge = true },
                secondSegment with
                {
                    Bridge = true
                }).All(segment => segment.ConsolidatedAvenue && segment.Bridge && segment.Layer == 0),
            "Same-structure avenue consolidation lost the elevated mode or source layer");

        Check(Run(firstSegment, secondSegment with { Layer = 1 }).Count == 2, "Different elevations merged");
        var structural = Run(firstSegment, secondSegment, MakeSegment(9, 2, 20, 100, 0, 150, 0) with { Bridge = true });
        var tie = structural.Single(segment => segment.WayId == 9);
        Check(
            tie.Bridge
                && math.all(tie.End == new float3(150, 0, 0))
                && structural.Any(segment => segment.ConsolidatedAvenue
                    && (segment.StartNodeId == tie.StartNodeId && math.all(segment.Start == tie.Start)
                || segment.EndNodeId == tie.StartNodeId && math.all(segment.End == tie.Start))),
            "Avenue consolidation must preserve the actual bridge tie-in and exact shared transition rather than banning the entire corridor");

        var differentLayer =
            Run(firstSegment, secondSegment, MakeSegment(9, 2, 20, 100, 0, 150, 0) with { Bridge = true, Layer = 1 });

        var bridgePort = differentLayer.Single(segment => segment.WayId == 9);
        Check(bridgePort.Bridge && bridgePort.Layer == 1 && differentLayer.Any(segment => segment.ConsolidatedAvenue
            && (segment.StartNodeId == bridgePort.StartNodeId && math.all(segment.Start == bridgePort.Start)
                || segment.EndNodeId == bridgePort.StartNodeId && math.all(segment.End == bridgePort.Start))),
            "An explicit shared bridge portal lost its canonical source connection because its layer differs from the road");

        var acrossMedian =
            Run(firstSegment, secondSegment, MakeSegment(9, 2, 3, 100, 0, 100, 10) with { Bridge = true, Layer = 1 });

        Check(
            acrossMedian.All(segment => segment.StartNodeId != segment.EndNodeId)
                && acrossMedian.Any(
                    segment => segment.WayId == 9
                        && segment.Bridge
                        && segment.StartNodeId == 2
                        && segment.EndNodeId == 3),
            "Avenue remapping collapsed a real structure into a self-loop or removed it");

        Check(
            Run(
                firstSegment,
                secondSegment,
                MakeSegment(
                    9,
                    2,
                    20,
                    100,
                    0,
                    150,
                    0) with
                { Kind = NetworkKind.Rail }).All(segment => !segment.ConsolidatedAvenue),
            "Rail crossing remapped without a verified shared transition");

        Check(
            Run(
                firstSegment,
                secondSegment with
                {
                    Start = new(100, 0, 50),
                    End = new(0, 0, 50)
                }).All(segment => !segment.ConsolidatedAvenue),
            "Widely separated roads merged");

        var ambiguous = MakeSegment(3, 10, 11, 0, 20, 100, 20);
        Check(
            Run(firstSegment, secondSegment, ambiguous).Any(segment => segment.ConsolidatedAvenue),
            "A nearby third route vetoed every ordinary-street pair");

        Check(
            Run(
                firstSegment,
                secondSegment with { Name = "" },
                firstSegment with
                {
                    WayId = 3,
                    StartNodeId = 5,
                    EndNodeId = 6,
                    Start = new(0, 0, 20),
                    End = new(100, 0, 20)
                }).Any(segment => segment.ConsolidatedAvenue),
            "Unnamed counterparts remained split in a three-route formation");

        var first = MakeSegment(1, 1, 5, 0, 0, 50, 0);
        var second = MakeSegment(3, 5, 2, 50, 0, 100, 0);
        var side = MakeSegment(9, 8, 5, 50, -50, 50, 0) with { Name = "Side", OneWay = false };
        var connected = Run(first, second, secondSegment, side);
        var attachment = connected.Single(segment => segment.WayId == 9);
        Check(
            math.distance(attachment.End.xz, new float2(50, 5)) < .001
                && connected.Any(
                    segment => segment.ConsolidatedAvenue
                        && (segment.StartNodeId == attachment.EndNodeId || segment.EndNodeId == attachment.EndNodeId)),
            "Side street lost connection at an internal corridor junction");

        var entry = MakeSegment(8, 100, 1, -50, 0, 0, 0) with { Name = "Entry", OneWay = false };
        var exit = MakeSegment(9, 2, 101, 100, 0, 150, 0) with { Name = "Exit", OneWay = false };
        var tied = Run(firstSegment, secondSegment, entry, exit);
        var from = tied.Single(segment => segment.WayId == 8).EndNodeId;
        var to = tied.Single(segment => segment.WayId == 9).StartNodeId;
        var seen = new HashSet<long> { from };
        var queue = new Queue<long>();
        queue.Enqueue(from);

        while (queue.Count > 0)
        {
            var nodeId = queue.Dequeue();

            foreach (var segment in tied.Where(
                otherSegment => otherSegment.StartNodeId == nodeId || otherSegment.EndNodeId == nodeId))
            {
                var next = segment.StartNodeId == nodeId ? segment.EndNodeId : segment.StartNodeId;

                if (seen.Add(next))
                    queue.Enqueue(next);
            }
        }

        Check(seen.Contains(to), "Consolidation severed route between original attachments");
        var reverseFirst = MakeSegment(2, 3, 6, 100, 10, 65, 10);
        var reverseSecond = MakeSegment(4, 6, 4, 65, 10, 0, 10);
        var mismatch = Run(first, second, reverseFirst, reverseSecond, side);
        Check(
            mismatch.Count(segment => segment.ConsolidatedAvenue) > 0
                && mismatch.Single(segment => segment.WayId == 9).End.x == 50,
            "Unequal segmentation lost a side-road junction");

        var median = MakeSegment(10, 5, 6, 50, 0, 50, 10) with { Classification = "primary_link", Name = "Median" };
        var alignedReverseFirst = MakeSegment(2, 3, 6, 100, 10, 50, 10);
        var alignedReverseSecond = MakeSegment(4, 6, 4, 50, 10, 0, 10);
        var medianResult = Run(first, second, alignedReverseFirst, alignedReverseSecond, median);
        Check(
            medianResult.Any(segment => segment.ConsolidatedAvenue)
                && medianResult.All(segment => segment.StartNodeId != segment.EndNodeId),
            "Median crossing consolidation created a self-loop");

        JunctionRegressions();
        CapturedBoiseStreet();
        var normalized = NetworkNormalizer.Normalize(connected, new ImportLog(), default, 1000);
        NetworkNormalizer.Validate(normalized, 1000, default);
        using var cancel = new CancellationTokenSource();
        cancel.Cancel();

        try
        {
            AvenueConsolidator.Prepare(new[] { firstSegment, secondSegment }, new ImportLog(), cancel.Token);
            throw new Exception("Cancellation ignored");
        }
        catch (OperationCanceledException) { }

        Console.WriteLine(
            "PASS: opposing one-ways to vanilla avenue centerlines, cross-way chains, junction/end reconnection, route reachability, unmatched/highway/ramp/structure/level guards and determinism.");
    }

    static void StrictLocalMatches()
    {
        var source = new[]{MakeSegment(900,1,101,0,0,50,0),MakeSegment(900,101,2,50,0,100,0),
            MakeSegment(901,3,102,100,12,50,12),MakeSegment(901,102,4,50,12,0,12),
            MakeSegment(902,5,101,50,-100,50,0) with{Kind=NetworkKind.Rail,Classification="rail",OneWay=false},
            MakeSegment(902,101,103,50,0,50,5) with{Kind=NetworkKind.Rail,Classification="rail",OneWay=false},
            MakeSegment(902,103,102,50,5,50,12) with{Kind=NetworkKind.Rail,Classification="rail",OneWay=false},
            MakeSegment(902,102,6,50,12,50,100) with{Kind=NetworkKind.Rail,Classification="rail",OneWay=false}};

        var log = new ImportLog();
        var result = AvenueConsolidator.Prepare(source, log, default);
        Check(
            result.Where(segment => segment.Kind == NetworkKind.Road).All(segment => !segment.OneWay),
            "Straight track crossings left the street as parallel one-ways");

        Check(
            result
                .Where(segment => segment.Kind == NetworkKind.Rail)
                .All(
                    segment => Math.Abs(segment.Start.x - 50) < .001
                        && Math.Abs(segment.End.x - 50) < .001
                        && segment.End.z > segment.Start.z),
            "Street consolidation bent or reversed the through railway");

        Check(
            NativeEndpointPolicy.LevelCrossings(result).Count == 1
                && result.All(segment => segment.StartNodeId != segment.EndNodeId),
            "The consolidated avenue did not have one shared road/rail crossing");

        SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(source), result, log, default);
        Check(
            log.Metric("Missing mandatory selected-source ports") == 0
                && log.Metric("Unresolved selected-source reachability obligations") == 0,
            "Consolidating a divided railway crossing lost a road or rail route");

        var road = new[]{MakeSegment(910,10,11,0,0,80,0),MakeSegment(910,11,12,80,0,100,60),
            MakeSegment(911,13,14,100,70,80,10),MakeSegment(911,14,15,80,10,0,10),
            MakeSegment(912,11,16,80,0,80,-100) with{Name="Side",OneWay=false}};

        var locally = AvenueConsolidator.Prepare(road, new ImportLog(), default);
        Check(
            locally.Any(
                segment => !segment.OneWay && segment.SourceWayIds.Contains(910) && segment.SourceWayIds.Contains(911)),
            "A bend vetoed every parallel local interval");
    }

    static void StrictJunctionCompletion()
    {
        var left =
            MakeSegment(
                100,
                1,
                2,
                -120,
                0,
                0,
                0) with
            { OneWay = false, ConsolidatedAvenue = true, SourceWayIds = new long[] { 10, 20 } };

        var right =
            MakeSegment(
                200,
                3,
                4,
                8,
                0,
                120,
                0) with
            { OneWay = false, ConsolidatedAvenue = true, SourceWayIds = new long[] { 10, 20 } };

        var tie = MakeSegment(10, 2, 3, 0, 0, 8, 0);
        var source = new[] { left, right, tie };
        var log = new ImportLog();
        var result = AvenueJunctionRepair.Prepare(source, log, default, completeStreetJunctions: true);
        Check(
            result.Count == 2
                && result[0].EndNodeId == result[1].StartNodeId
                && math.all(result[0].End == result[1].Start),
            "A short evidenced one-way tie remained between two consolidated avenues");

        SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(source), result, log, default);
        Check(
            log.Metric("Missing mandatory selected-source ports") == 0
                && log.Metric("Unresolved selected-source reachability obligations") == 0,
            "Completing a street junction lost source reachability");

        var highway = MakeSegment(300, 5, 2, 0, -100, 0, 0) with { Classification = "trunk" };
        Check(
            AvenueJunctionRepair.Prepare(
                source.Append(highway).ToArray(),
                new ImportLog(),
                default,
                completeStreetJunctions: true).Count
                == 4,
            "Completing a street junction consumed an attached trunk-road port");

        var side = MakeSegment(400, 3, 6, 8, 0, 20, 6) with { OneWay = false, Name = "Side" };
        Check(
            AvenueJunctionRepair.Prepare(
                source.Append(side).ToArray(),
                new ImportLog(),
                default,
                completeStreetJunctions: true).Count
                == 4,
            "Completing a street junction introduced an acute side-street opening");
    }

    static void FragmentedCounterparts()
    {
        var source = new List<PreparedSegment> { MakeSegment(300, 300, 301, 0, 0, 2400, 0) with { Lanes = 1 } };

        for (var i = 0; i < 12; i++)
            source.Add(
                MakeSegment(301 + i, 400 + 2 * i, 401 + 2 * i, 150 + 200 * i, 12, 20 + 200 * i, 12) with { Lanes = 1 });

        var log = new ImportLog();
        var result = AvenueConsolidator.Prepare(source, log, default);
        Check(Enumerable.Range(301, 12).All(way => !result.Any(segment => segment.OneWay && segment.WayId == way)),
            "A long street retained parallel opposing fragments or tiny projected tails");

        SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(source), result, log, default);
        Check(
            log.Metric("Missing mandatory selected-source ports") == 0
                && log.Metric("Unresolved selected-source reachability obligations") == 0,
            "Consolidating fragmented opposing streets lost an exterior route or source port");

        Check(
            result.Where(segment => !segment.OneWay).All(segment => segment.Lanes == 2 && !segment.ConsolidatedAvenue),
            "Fragmented 1+1 lanes did not become regular two-lane roads");
    }

    static void DividedCrossing()
    {
        var box = new[]{MakeSegment(600,600,601,-8,8,8,8),MakeSegment(601,601,602,8,8,8,-8),
            MakeSegment(602,602,603,8,-8,-8,-8),MakeSegment(603,603,600,-8,-8,-8,8)};

        var arms = new[]{MakeSegment(610,610,600,-300,8,-8,8),MakeSegment(611,603,611,-8,-8,-300,-8),
            MakeSegment(612,601,612,8,8,300,8),MakeSegment(613,613,602,300,-8,8,-8),
            MakeSegment(614,600,614,-8,8,-8,300) with{Name="Cross"},
                MakeSegment(615,615,601,8,300,8,8) with{Name="Cross"},
            MakeSegment(616,602,616,8,-8,8,-300) with{Name="Cross"},
                MakeSegment(617,617,603,-8,-300,-8,-8) with{Name="Cross"}
                }
            .Select(segment => segment with { Lanes = 3 }).ToArray();

        var source = box.Concat(arms).ToArray();
        var log = new ImportLog();
        var result = AvenueConsolidator.Prepare(source, log, default);
        Check(
            log.Metric("Divided street intersection boxes consolidated") == 1
                && result.All(segment => !segment.OneWay && segment.Lanes == 6),
            "A surface crossing retained split three-lane approaches instead of continuous six-lane avenues");

        var at =
            result
                .SelectMany(segment => new[] { (segment.StartNodeId, segment.Start), (segment.EndNodeId, segment.End) })
                .Where(item => math.length(item.Item2.xz) < 1)
                .ToArray();

        Check(
            at.Length == 4 && at.Select(item => item.Item1).Distinct().Count() == 1,
            "The four approaches do not share one central crossing");

        SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(source), result, log, default);
        Check(
            log.Metric("Missing mandatory selected-source ports") == 0
                && log.Metric("Unresolved selected-source reachability obligations") == 0,
            "Street crossing consolidation lost source connections");

        var protectedSource =
            source.Append(MakeSegment(620, 600, 620, -8, 8, -100, 100) with { Classification = "trunk" }).ToArray();

        Check(
            StreetIntersectionConsolidator
                .Prepare(protectedSource, new ImportLog(), default)
                .SequenceEqual(protectedSource),
            "A street crossing moved an attached highway");

        var separate =
            source.Select(segment => segment.WayId == 613 ? segment with { Name = "Different" } : segment).ToArray();

        Check(
            StreetIntersectionConsolidator.Prepare(separate, new ImportLog(), default).SequenceEqual(separate),
            "Separate named streets were treated as a divided crossing");
    }

    static void TaperAttachmentGuard()
    {
        var body = new[]{MakeSegment(500,500,501,0,0,50,0),MakeSegment(500,501,502,50,0,100,0),
            MakeSegment(501,503,504,100,12,90,6),MakeSegment(501,504,505,90,6,0,10)};

        Check(AvenueConsolidator.Prepare(body, new ImportLog(), default).Any(segment => segment.ConsolidatedAvenue),
            "An identified divided street's short end taper prevented consolidation");

        var arm = MakeSegment(502, 501, 506, 50, 0, 70, 11) with { Name = "Side", OneWay = false };
        var log = new ImportLog();
        var result = AvenueConsolidator.Prepare(body.Append(arm).ToArray(), log, default);
        var side = result.Single(segment => segment.WayId == 502);
        var heading = (side.End - side.Start).xz;
        var opening =
            result
                .Where(
                    segment => segment.WayId != 502
                        && (segment.StartNodeId == side.StartNodeId || segment.EndNodeId == side.StartNodeId))
            .Select(
                segment => Math.Acos(
                    math.clamp(
                        math.dot(
                            math.normalizesafe(heading),
                            math.normalizesafe(
                                (segment.StartNodeId == side.StartNodeId
                                    ? segment.End - segment.Start
                                    : segment.Start - segment.End).xz)),
                        -1,
                        1))
                    * 180
                    / Math.PI)
                .Min();

        if (opening < 25)
        {
            foreach (var segment in result)
                Console.WriteLine(
                    $"TAPER {segment.WayId} {segment.StartNodeId}->{segment.EndNodeId} {segment.Start.x},{segment.Start.z} -> {segment.End.x},{segment.End.z}");

            foreach (var line in log.Lines.Where(text => text.StartsWith("AVENUE MERGE", StringComparison.Ordinal)))
                Console.WriteLine(line);
        }

        Check(opening >= 25 && log.Metric("Avenue taper matches retained for acute attachments") > 0,
            $"A relaxed end taper introduced an acute side-street attachment ({opening:0.##} degrees)");

        var trunk = MakeSegment(503, 502, 503, 100, 0, 100, 12) with { Classification = "trunk", Name = "Major" };
        var tied = AvenueConsolidator.Prepare(body.Append(trunk).ToArray(), new ImportLog(), default);
        Check(tied.Any(segment => segment.ConsolidatedAvenue) && tied.Single(segment => segment.WayId == 503) == trunk,
            "Consolidation failed to retain short turning approaches behind distinct trunk-road ports");

        var obligations = new ImportLog();
        SourceTopologyObligations.Validate(
            SourceTopologyObligations.Capture(body.Append(trunk).ToArray()),
            tied,
            obligations,
            default);

        Check(
            obligations.Metric("Missing mandatory selected-source ports") == 0
                && obligations.Metric("Unresolved selected-source reachability obligations") == 0,
            "Keeping separate trunk turning ports lost a selected-source route");

        var ring = new[]{MakeSegment(504,502,507,100,0,106,6),MakeSegment(504,507,503,106,6,100,12),
            MakeSegment(504,503,508,100,12,94,6),MakeSegment(504,508,502,94,6,100,0)}
                .Select(segment => segment with { Name = "", Roundabout = true })
                .ToArray();

        var roundabout = AvenueConsolidator.Prepare(body.Concat(ring).ToArray(), new ImportLog(), default);
        Check(roundabout.Where(segment => segment.Roundabout).SequenceEqual(ring),
            "Avenue consolidation distorted a roundabout's ring");

        var competition = new[]{MakeSegment(510,510,511,0,0,300,0),MakeSegment(511,512,513,300,10,0,10),
            MakeSegment(512,514,515,300,8,285,2),MakeSegment(512,515,516,285,2,0,2)};

        var selected = AvenueConsolidator.Prepare(competition, new ImportLog(), default);
        Check(
            selected.Any(
                segment => segment.ConsolidatedAvenue
                    && segment.SourceWayIds.Contains(510)
                    && segment.SourceWayIds.Contains(511))
                && !selected.Any(segment => segment.ConsolidatedAvenue && segment.SourceWayIds.Contains(512)),
            "A closer end-taper candidate displaced an established fully parallel counterpart");
    }

    static void CapturedBoiseStreet()
    {
        // Northside Road and its incident arms, from the complete Boise source
        // capture. Coalesced stations must still carry the short source way
        // 1572423297 between both of its mapped junction ports.
        var source =
            File
                .ReadLines(Path.Combine(AppContext.BaseDirectory, "fixtures", "avenue-boise-northside.ndjson"))
                .Select(line =>
        {
            var row = Newtonsoft.Json.Linq.JObject.Parse(line);

            float3 Point(string key)
            {
                var p = row[key]!.Values<float>().ToArray();
                return new(p[0], p[1], p[2]);
            }

            return new PreparedSegment(
                (long)row["WayId"]!,
                (long)row["StartNodeId"]!,
                (long)row["EndNodeId"]!,
                Point("start"),
                Point("end"),
                NetworkKind.Road,
                    (string)row["Classification"]!,
                    (int)row["Lanes"]!,
                    (bool)row["OneWay"]!,
                    (bool)row["Bridge"]!,
                    (bool)row["Tunnel"]!,
                    (int)row["Layer"]!)
            {
                Name = (string)row["Name"]!,
                Reference = (string)row["Reference"]!,
                SourceWayIds = Array.AsReadOnly(row["SourceWayIds"]!.Select(value => (long)value).ToArray()),
                SourceGeometry = new[] { Point("start"), Point("end") }
            };
        }).ToArray();

        var log = new ImportLog();
        var result = AvenueConsolidator.Prepare(source, log, default);
        Check(
            log.Metric("Opposing road pairs consolidated into avenues") > 0,
            "Captured Boise street did not consolidate");

        SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(source), result, log, default);
        SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(result), result, log, default);
        Check(
            log.Metric("Missing mandatory selected-source ports") == 0
                && log.Metric("Unresolved selected-source reachability obligations") == 0,
            "Captured Boise avenue lost an attached street or a short source way's directional route");
    }

    static void JunctionRegressions()
    {
        // Exact dimensions from the Palm Springs failure: two projected
        // stations 16.6 m apart and a transverse source median connection.
        var left =
            MakeSegment(
                -100,
                -1,
                -2,
                15202.5f,
                -11504.86f,
                15099.667f,
                -11503.515f) with
            { ConsolidatedAvenue = true, OneWay = false, Lanes = 6, NativeWidth = 32 };

        var internalPiece =
            MakeSegment(
                -100,
                -2,
                -3,
                15099.667f,
                -11503.515f,
                15083.043f,
                -11503.26f) with
            { ConsolidatedAvenue = true, OneWay = false, Lanes = 6, NativeWidth = 32 };

        var right =
            MakeSegment(
                -100,
                -3,
                -4,
                15083.043f,
                -11503.26f,
                14863.046f,
                -11500.373f) with
            { ConsolidatedAvenue = true, OneWay = false, Lanes = 6, NativeWidth = 32 };

        var median =
            MakeSegment(
                1220190819,
                -3,
                -2,
                15083.043f,
                -11503.26f,
                15099.667f,
                -11503.515f) with
            {
                OneWay = false,
                OriginalGeometry =
                    new[] { new float3(15090.907f, 0, -11494.583f), new float3(15090.842f, 0, -11512.228f) }
            };

        var result =
            AvenueJunctionRepair.Prepare(new[] { left, internalPiece, right, median }, new ImportLog(), default);

        Check(
            result.Count == 2
                && result[0].EndNodeId == result[1].StartNodeId
                && math.all(result[0].End == result[1].Start),
            "Palm Springs median link still duplicates a tiny avenue interval");

        Check(
            math.all(result[0].Start == left.Start)
                && math.all(result[1].End == right.End)
                && result.All(segment => segment.SourceGeometry.Count == 2 && segment.Control1 == null),
            "Canonical median changed distant endpoints or left stale curve geometry");

        Check(
            AvenueJunctionRepair
                .Prepare(new[] { median, right, internalPiece, left }, new ImportLog(), default)
                .OrderBy(segment => segment.StartNodeId)
                .Select(segment => (segment.StartNodeId, segment.EndNodeId, segment.Start, segment.End))
                .SequenceEqual(
                    result
                        .OrderBy(segment => segment.StartNodeId)
                        .Select(segment => (segment.StartNodeId, segment.EndNodeId, segment.Start, segment.End))),
            "Median canonicalization depends on input ordering");

        Check(
            AvenueJunctionRepair.Prepare(result, new ImportLog(), default).SequenceEqual(result),
            "Median repair is not idempotent");

        var unchanged = median with { OriginalGeometry = new[] { median.Start, median.End } };
        Check(
            AvenueJunctionRepair.Prepare(
                new[] { left, internalPiece, right, unchanged },
                new ImportLog(),
                default).Count
                == 4,
            "A real longitudinal short street was consumed as a median");

        Check(
            AvenueJunctionRepair.Prepare(
                new[] { left, internalPiece, right, median with { Layer = 1 } },
                new ImportLog(),
                default).Count
                == 4,
            "Different layers canonicalized");

        Check(
            AvenueJunctionRepair.Prepare(
                new[] { left, internalPiece, right, median with { Bridge = true } },
                new ImportLog(),
                default).Count
                == 4,
            "Bridge portal consumed");

        var rail =
            MakeSegment(
                45,
                -2,
                50,
                left.End.x,
                left.End.z,
                left.End.x + 50,
                left.End.z) with
            { Kind = NetworkKind.Rail };

        Check(
            AvenueJunctionRepair.Prepare(
                new[] { left, internalPiece, right, median, rail },
                new ImportLog(),
                default).Count
                == 5,
            "Road/rail junction canonicalized without source-mode proof");

        // Oman: two distinct consolidated corridors connected by an explicit
        // ~1 m source link (not an arbitrary nearby node).
        var old =
            MakeSegment(
                -200,
                -10,
                -11,
                -11536.171f,
                18037.46f,
                -11456.347f,
                18008.092f) with
            {
                ConsolidatedAvenue = true,
                OneWay = false,
                NativeWidth = 24,
                SourceWayIds = new long[] { 36064432, 183177670 }
            };

        var approach =
            MakeSegment(
                -201,
                -12,
                -13,
                -11394.675f,
                17985.45f,
                -11455.377f,
                18007.832f) with
            { ConsolidatedAvenue = true, OneWay = false, Lanes = 6, NativeWidth = 32 };

        var tie = MakeSegment(183177670, -13, -11, approach.End.x, approach.End.z, old.End.x, old.End.z);
        var oman = AvenueJunctionRepair.Prepare(new[] { old, approach, tie }, new ImportLog(), default);
        Check(
            oman.Count == 2 && oman[0].EndNodeId == oman[1].EndNodeId && math.all(oman[0].End == oman[1].End),
            "Oman explicit short tie-in still has two competing junction owners");

        Check(
            AvenueJunctionRepair.Prepare(new[] { old, approach }, new ImportLog(), default).Count == 2
                && old.EndNodeId != approach.EndNodeId,
            "Unconnected nearby roads were proximity-joined");

        Check(
            AvenueJunctionRepair.Prepare(
                new[] { old with { SourceWayIds = Array.Empty<long>() }, approach, tie },
                new ImportLog(),
                default).Count
                == 3,
            "Short connection lacking consolidation provenance was removed");

        using var cancel = new CancellationTokenSource();
        cancel.Cancel();

        try
        {
            AvenueJunctionRepair.Prepare(new[] { left, internalPiece, right, median }, new ImportLog(), cancel.Token);
            throw new Exception("Junction cancellation ignored");
        }
        catch (OperationCanceledException) { }

        Console.WriteLine(
            "PASS: captured-dimension Palm Springs/Oman median topology recovery, deterministic/idempotent canonical junctions, unchanged distant arms, and unconnected/longitudinal/structure/layer/rail/provenance guards.");
    }
}
