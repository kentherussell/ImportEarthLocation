using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Visual network compiler, horizontal stage (research report 64 §5.1–5.2, §5.5).
// Builds each perceived road once: short junction connectors are contracted,
// node-to-node sections are chained into strokes by good continuation, each
// stroke is simplified (Douglas–Peucker) and rebuilt from straights and circular
// fillets, then emitted as merged cubics. Junction nodes and topology ports keep
// their source identities; absorbed nodes become aliases. Coordinates are local
// map metres (XZ). Emission waits for VisualCompilerProfile to solve heights for
// every stroke together.
public static class VisualNetworkCompiler
{
    // The game cuts an edge longer than 200 m in two; plan no node it does not know.
    private const double MaximumSpan = 200;
    // How far along a stroke end the fork clearance acts (metres).
    private const double ForkReach = 300, ForkFade = 150;
    private const long GeneratedNodeBase = long.MinValue + 8000000;
    // Shortest section the compiler keeps between two distinct nodes (metres).
    private const double MicroLength = 1.5;
    // Longest section contracted when it doubles back at a through node (metres).
    private const double FoldLength = 15;
    // Angle (degrees) at which a one-way road without lanes of its own joins or leaves another.
    private const double MergeAngle = 30;
    // Highway fit: shortest wavelength kept, and how far a sample may leave its line.
    private const double MainlineWavelength = 200, LinkWavelength = 150, MainlineBudget = 3, LinkBudget = 4;
    // The same for every other road under smooth-roads (report 113, 5.2).
    private const double StreetWavelength = 100, StreetBudget = 1.5;
    // A merged avenue's line is the midpoint of its two carriageways, and where two merges meet
    // or close junctions were joined it steps 5-7 m sideways. Within 1.5 m the fit could only
    // follow the step, as a 10-30 m radius S-bend (report 120). A stroke that is mostly avenue
    // may leave its line by this much (metres), inside the median it replaced. Of 3, 4 and 6 m,
    // 6 left the fewest tight bends (DC 135, 120, 99 under 30 m) with the same junction kinks.
    private const double AvenueBudget = 6;
    // Turn (degrees) at one point from which a road has a corner rather than a curve.
    private const double CornerAngle = 30;
    // Smallest fillet radius drawn as a curve (metres); a tighter corner is a node.
    private const double CornerRadius = 8;
    // A corner this near (metres) a node that cannot move is a merged junction's jog.
    private const double JogReach = 12;
    // Length (metres) over which a road eases onto a node that stands off its line.
    private const double NodeEase = 40;

    // A section shorter than this (metres) that returns to the node it left lies inside a merged junction.
    private const double MergedLoop = 30;
    // Smooth roads: a loop off one junction shorter than JunctionLoop (metres) whose nodes
    // all lie within JunctionLoopReach (metres) of the junction becomes part of it.
    private const double JunctionLoop = 60;
    private const double JunctionLoopReach = 20;

    // Smooth roads: how far (metres) a junction joined from close junctions may move from
    // their centre onto the line of its through road (report 120).
    private const double JoinedJunctionShift = 15;
    // Most a planned piece of a fitted road may turn (degrees). On slopes the game refits a
    // piece from its ends and end directions, so a small turn bounds what that can change.
    private const double PartTurn = 30;
    // How many more equal parts than the fewest possible a fitted run is tried with.
    private const int EvenTries = 3;
    // Gap (metres) a fitted road keeps from the edge of a road fitted before it, the
    // sideways slope at which that move eases off, and the slope at which the need
    // grows away from a node both roads share (about 5 and 8.5 degrees).
    private const double FitClearance = 2, ClearanceTaper = .0875, ClearanceFade = .15;
    // Weight of a sample moved for clearance: a hundred times a free sample, so the
    // fit keeps the move but still rounds its ends.
    private const double ClearanceWeight = 100;
    // Most (metres) a road is moved sideways for clearance. Two roads mapped closer than
    // this to each other's line are one over the other on the map, and pushing one a full
    // road width aside put a bulge in it that the map does not have.
    private const double ClearanceLimit = 10;
    // An arm within this angle (degrees) of straight ahead is the same road continuing.
    private const double ThroughAngle = 30;
    // A closed roundabout ring is drawn as a circle when its radius (metres) is within
    // these limits and its map line strays from the fitted circle by no more than a
    // metre or RingRoundness of the radius. An arm within RingArmAngle (degrees) of the
    // radius at its ring node is pointed at the centre.
    private const double RingMinimumRadius = 5;
    private const double RingIsland = 1;

    // Pieces shorter than this that pair up between the same two nodes are one junction.
    private const double TwinLength = 8;
    private const double RingMaximumRadius = 120;
    private const double RingRoundness = .12;
    private const double RingArmAngle = 10;
    // A two-way street that ends within AcuteAngle (degrees) of another arm of its node
    // is turned away from that arm, toward SquareAngle, over its last SquareReach metres.
    private const double AcuteAngle = 45;
    private const double SquareAngle = 60;
    private const double SquareReach = 20;
    // A fitted road that stays within StraightReach (metres) of a line for StraightLength
    // (metres) or more is set on that line, easing in over StraightEase (metres) at each
    // free end. A node that cannot move may turn the line by at most StraightTurn (degrees).
    private const double StraightReach = 1;
    private const double StraightLength = 150;
    private const double StraightEase = 20;
    private const double StraightTurn = 1;
    // A one-way road follows an opposite one-way road of the same name that runs within
    // PairReach (metres) for PairLength or more, at the median gap over PairWindow
    // either side, easing in over PairEase at each end (report 113, 7.1).
    private const double PairReach = 30;
    private const double PairLength = 60;
    private const double PairWindow = 50;
    private const double PairEase = 20;
    // A run of road shorter than this (metres) between two stretches that share one
    // asset takes that asset (report 113, 7.4).
    private const double AssetRunLength = 150;
    // A two-way street branch needs room for curve fitting while retaining
    // at least the junction's 25-degree opening after its approach is rounded.
    private const double StreetBranchAngle = 35;
    // Angle for a branch of two or more lanes (3 lanes → 2 + 2, as the A55
    // splits into its 2-lane link before its tunnel): at 30° the two branches
    // still read as one widened road with an artifact at the split. Applied
    // to single-lane ramps too, 40° added 390 tight radii in DC.
    private const double SplitAngle = 40;

    private static bool Splits(PreparedSegment segment) => segment.Lanes >= 2;

    // Largest radius (metres) of a ramp's turn back to parallel at its
    // clearance. At a motorway's 250 m floor a branch at SplitAngle needed
    // 75 m of offset to straighten, veering out and back in an S before the
    // A55's tunnel. The turn is fitted inside the clearance: a corner set
    // wider (to keep a 20 m straight start) was a hump once the ramp came
    // back in to its clearance.
    private const double TurnBackRadius = 50;
    // How far a merging ramp is held at its clearance beside the through
    // road while its source stays inside it (metres), and the length over
    // which a hold cut short by a fixed point fades out.
    private const double HoldReach = 600;
    private const double HoldFade = 60;
    // Longest rail section on either side of a plain node straightened as a jog (metres).
    private const double JogLength = 30;
    // Widest map-data break between two dead ends that is joined (metres).
    private const double MappingGap = 3;
    // Smooth roads: widest break between two dead ends in line with each other that is
    // joined (metres), and how far to the side the far end may lie: InlineAside (metres)
    // plus InlineDrift for each metre of gap.
    private const double InlineGap = 45;
    private const double InlineAside = 1.5;
    private const double InlineDrift = .1;
    // Height above the original ground (metres) beyond which a bridge approach
    // is built elevated rather than as a ground piece.
    private const float ElevateHeight = 8;
    // Longest roundabout held level (metres of ring).
    private const double MaximumLevelRing = 300;

    // A motorway or trunk mainline keeps its own line at a fork or merge. Bending it
    // to an approach angle drew jogs tens of metres off the map (reports 77, 103).
    private static bool TakesApproach(PreparedSegment segment)
        => !(segment.Kind == NetworkKind.Road && !segment.ProtectedLink && NativeAssetKeys.IsHighway(segment));

    private readonly record struct Arm(int Section, bool AtStart);

    private readonly record struct Piece(double2 P0, double2 P1, double2 P2, double2 P3);

    private readonly record struct Cut(int Piece, long Node, int StrokeNode);

    // water: the water planner's surface sampler; sea: effective sea level (NaN
    // when unknown). Both keep bridge decks clear of the water they cross.
    // defenceCrests: flood-defence crests on the terrain's grid (0 where none). A
    // ground road crossing one keeps its deck at the crest: the game levels the
    // ground beside a road to the road, so a deck below the crest cuts a notch the
    // water comes through (report 112, 14.1: a third of the lowered dyke cells).
    public static IReadOnlyList<PreparedSegment> Prepare(
        IReadOnlyList<PreparedSegment> source,
        float[] terrain,
        double width,
        ImportLog log, CancellationToken token, Func<float3, float?>? water = null, float sea = float.NaN,
        float[]? defenceCrests = null)
    {
        // Each phase's own time, so the design stage can be split (report 118).
        var phaseClock = System.Diagnostics.Stopwatch.StartNew();

        // Start of the current part of a phase, as a Stopwatch timestamp.
        var partStarted = System.Diagnostics.Stopwatch.GetTimestamp();

        void Phase(string name)
        {
            log.Time("Visual compiler " + name, phaseClock.Elapsed.TotalSeconds);
            phaseClock.Restart();
            partStarted = System.Diagnostics.Stopwatch.GetTimestamp();
        }

        // A part of the current phase: the time since the previous part or the phase start.
        void PhasePart(string name)
        {
            var now = System.Diagnostics.Stopwatch.GetTimestamp();
            log.Time("Visual compiler " + name, Seconds(now - partStarted));
            partStarted = now;
        }

        static double Seconds(long ticks) => ticks / (double)System.Diagnostics.Stopwatch.Frequency;

        // Time inside the line fit of every stroke, reported with the "ordering" phase.
        var strokeFitTicks = 0L;

        // Time in the other parts of every stroke (EmitStroke), also reported with "ordering".
        var strokeLineTicks = 0L;
        var strokePushTicks = 0L;
        var strokePieceTicks = 0L;
        var strokeStationTicks = 0L;
        var strokeRunTicks = 0L;
        var strokePlanTicks = 0L;

        var seaCells =
            (DesignSwitches.On(DesignSwitches.SeaFromEdge) || DesignSwitches.On(DesignSwitches.FloodDefences))
                && !float.IsNaN(sea)
                ? SeaFromEdge(terrain, sea, log)
                : null;

        PhasePart("setup, of which sea from edge");

        // Same rail formation and turnout-footprint owners as the legacy design;
        // they resolve terminal aliases before strokes are built.
        source = NetworkRailTerminalFormations.Prepare(source, log, token);
        source = RailCorridorConsolidator.NativeJunctionFootprints(source, log, token);
        PhasePart("setup, of which rail formations and footprints");

        source = ShortUnderpasses.Prepare(source, log, token);
        var structureSource = source;
        var structurePorts = SourceTopologyObligations.Capture(source).Ports;
        source = BridgeGaps(source, log, token);

        // After the capture, so a bridge end that stops being a port is kept as an alias.
        if (DesignSwitches.On(DesignSwitches.SmoothRoads))
            source = ShortBridges.Prepare(source, water, log, token);

        PhasePart("setup, of which underpasses and bridges");

        var tunnelPortals = defenceCrests != null ? TunnelPortals(source) : null;
        // Topology ports and every mandatory alias target must survive as nodes.
        // A plain road alias (a source node that was never a port, such as a way
        // boundary inside a merged avenue) may be joined into its run. Rail keeps
        // every alias target: its formation and turnout nodes are aliases, and
        // joining past them steepened a DC rail tunnel from 8.2% to 9.5%.
        var ports = new HashSet<SourceNodeKey>(SourceTopologyObligations.Capture(source).Ports);

        foreach (var segment in source)
            foreach (var alias in segment.SourceNodeAliases)
                if (alias.Mandatory || segment.Kind == NetworkKind.Rail)
                    ports.Add(new(segment.Kind, alias.CanonicalNodeId));

        var retiredStructurePorts =
            new HashSet<SourceNodeKey>(
                structurePorts.Where(nodeKey => nodeKey.Kind == NetworkKind.Road && !ports.Contains(nodeKey)));

        var structureAliases = new Dictionary<long, long>();
        var plainPorts = 0;

        var position = new Dictionary<SourceNodeKey, double2>();
        var degree = new Dictionary<SourceNodeKey, int>();
        var fixedNodes = new HashSet<SourceNodeKey>();

        foreach (var segment in source)
        {
            position[SourceNodeKey.Start(segment)] = ((float3)segment.CourseStart).xz;
            position[SourceNodeKey.End(segment)] = ((float3)segment.CourseEnd).xz;

            foreach (var key in new[] { SourceNodeKey.Start(segment), SourceNodeKey.End(segment) })
            {
                degree[key] = degree.TryGetValue(key, out var d) ? d + 1 : 1;

                if (segment.Roundabout)
                    fixedNodes.Add(key);
            }
        }

        var levelCrossings = NativeEndpointPolicy.LevelCrossings(source);

        foreach (var node in levelCrossings)
        {
            fixedNodes.Add(new(NetworkKind.Road, node));
            fixedNodes.Add(new(NetworkKind.Rail, node));
        }

        // Contract short connectors between junctions (report 64 V1 step 2).
        var parent = new Dictionary<SourceNodeKey, SourceNodeKey>();

        SourceNodeKey Root(SourceNodeKey nodeKey)
        {
            while (parent.TryGetValue(nodeKey, out var otherNodeKey) && otherNodeKey != nodeKey)
                nodeKey =
                    parent[nodeKey] =
                        parent.TryGetValue(otherNodeKey, out var secondNodeKey) ? secondNodeKey : otherNodeKey;

            return nodeKey;
        }

        var contracted = new bool[source.Count];
        PhasePart("setup, of which ports and nodes");

        // A section shorter than MicroLength is one node drawn twice (a portal or
        // tag change beside a junction). The game cannot build a usable edge that
        // short, so its ends become one node, unless another section joins the
        // same two (merged) nodes and would become a loop.
        Phase("setup");

        var micro =
            Enumerable
                .Range(0, source.Count)
                .Where(
                    i => !source[i].Roundabout
                        && SourceNodeKey.Start(source[i]) != SourceNodeKey.End(source[i])
                        && !fixedNodes.Contains(SourceNodeKey.Start(source[i]))
                        && !fixedNodes.Contains(SourceNodeKey.End(source[i]))
                && NetworkDesignChecks.HorizontalArcLength(NetworkDesignChecks.Course(source[i])) < MicroLength)
                .ToArray();

        // Whether a section other than i, not yet contracted, already joins the merged nodes
        // first and second, so that joining them would make it a loop. Only the sections at
        // the nodes merged into first are checked, not every section of the map: that scan,
        // once per short section, took 8 s of Paris's design offline (report 118). mergedNodes
        // holds each merged node's original nodes and is rebuilt before each loop that uses
        // it, since other steps also merge nodes.
        var incident = new Dictionary<SourceNodeKey, List<int>>();

        for (var i = 0; i < source.Count; i++)
            foreach (var key in new[] { SourceNodeKey.Start(source[i]), SourceNodeKey.End(source[i]) })
            {
                if (!incident.TryGetValue(key, out var list))
                    incident[key] = list = new();

                list.Add(i);
            }

        var mergedNodes = new Dictionary<SourceNodeKey, List<SourceNodeKey>>();

        void IndexMembers()
        {
            mergedNodes.Clear();

            foreach (var key in incident.Keys)
            {
                var root = Root(key);

                if (!mergedNodes.TryGetValue(root, out var list))
                    mergedNodes[root] = list = new();

                list.Add(key);
            }
        }

        void Merge(SourceNodeKey first, SourceNodeKey second)
        {
            mergedNodes[first].AddRange(mergedNodes[second]);
            mergedNodes.Remove(second);
        }

        bool Joined(int i, SourceNodeKey first, SourceNodeKey second)
        {
            foreach (var key in mergedNodes[first])
                foreach (var j in incident[key])
                {
                    if (j == i || contracted[j])
                        continue;

                    var start = Root(SourceNodeKey.Start(source[j]));
                    var end = Root(SourceNodeKey.End(source[j]));

                    if (start == first && end == second || start == second && end == first)
                        return true;
                }

            return false;
        }

        IndexMembers();

        foreach (var i in micro)
        {
            token.ThrowIfCancellationRequested();
            var firstNodeKey = Root(SourceNodeKey.Start(source[i]));
            var secondNodeKey = Root(SourceNodeKey.End(source[i]));

            if (firstNodeKey == secondNodeKey)
                continue;

            if (Joined(i, firstNodeKey, secondNodeKey))
                continue;

            parent[secondNodeKey] = firstNodeKey;
            Merge(firstNodeKey, secondNodeKey);
            contracted[i] = true;
        }

        // A short section that doubles back at a plain through node (its neighbour
        // there leaves within 60° of it) is a fold, not a route: rail
        // consolidation re-attaching a tail can end a line 4 m past where the
        // next piece starts, a 178° kink that reads as a break. Its ends become
        // one node, under the same loop guard.
        var folds = 0;

        double2 Leaving(PreparedSegment segment, bool start)
        {
            var curve = NetworkDesignChecks.Course(segment);
            var point = start ? curve.P0 : curve.P3;

            return math.normalizesafe(NetworkDesignChecks.Evaluate(curve, start ? .25 : .75).xz - point.xz);
        }

        var ends = new Dictionary<SourceNodeKey, List<(int Section, bool Start)>>();

        for (var i = 0; i < source.Count; i++)
            foreach (var start in new[] { true, false })
            {
                var key = start ? SourceNodeKey.Start(source[i]) : SourceNodeKey.End(source[i]);

                if (!ends.TryGetValue(key, out var list))
                    ends[key] = list = new();

                list.Add((i, start));
            }

        var streetTags = 0;
        PreparedSegment[]? streetSource = null;

        foreach (var i in Enumerable
            .Range(0, source.Count)
            .OrderBy(i => source[i].WayId)
            .ThenBy(i => source[i].StartNodeId)
            .ThenBy(i => source[i].EndNodeId))
        {
            token.ThrowIfCancellationRequested();
            var segment = source[i];

            if (contracted[i]
                || SourceNodeKey.Start(segment) == SourceNodeKey.End(segment)
                || !IsGroundStreet(segment)
                || fixedNodes.Contains(SourceNodeKey.Start(segment))
                || fixedNodes.Contains(SourceNodeKey.End(segment))
                || degree[SourceNodeKey.Start(segment)] < 2
                    || degree[SourceNodeKey.End(segment)] < 2
                    || NetworkBoundaryPolicy.EndpointMarker(segment, true)
                    || NetworkBoundaryPolicy.EndpointMarker(segment, false)
                    || NetworkDesignChecks.HorizontalArcLength(NetworkDesignChecks.Course(segment)) > 10)
                continue;

            foreach (var start in new[] { true, false })
            {
                var key = start ? SourceNodeKey.Start(segment) : SourceNodeKey.End(segment);

                if (degree[key] != 2)
                    continue;

                var other = ends[key].First(item => item.Section != i);
                var streetNeighbor = source[other.Section];

                if (contracted[other.Section]
                    || !IsGroundStreet(streetNeighbor)
                    || segment.Lanes != streetNeighbor.Lanes
                    || segment.OneWay != streetNeighbor.OneWay
                    || segment.Layer != streetNeighbor.Layer
                    || segment.NativeWidth != streetNeighbor.NativeWidth
                        || segment.NativeAssetKey != streetNeighbor.NativeAssetKey
                        || NetworkBoundaryPolicy.EndpointMarker(streetNeighbor, other.Start)
                        || segment.OneWay && start == other.Start
                        || math.dot(Leaving(segment, start), Leaving(streetNeighbor, other.Start))
                            > -Math.Cos(math.radians(10)))
                    continue;

                var far = start ? SourceNodeKey.End(segment) : SourceNodeKey.Start(segment);
                var nextFar = other.Start ? SourceNodeKey.End(streetNeighbor) : SourceNodeKey.Start(streetNeighbor);

                if (Root(nextFar) == Root(far)
                    || Root(SourceNodeKey.Start(segment)) == Root(SourceNodeKey.End(segment)))
                    continue;

                var delta = position[SourceNodeKey.Start(segment)] - position[SourceNodeKey.End(segment)];
                var heading = Leaving(streetNeighbor, other.Start);

                if (Math.Abs(NetworkDesignChecks.Cross(delta, heading))
                    > Math.Min(2, VisualCompilerTable.For(segment).Deviation))
                    continue;

                // Retire only a plain, nearly straight tag/alias station. Keep
                // its way evidence and aliases on the surviving continuation.
                streetSource ??= source.ToArray();
                streetSource[other.Section] = streetNeighbor with
                {
                    SourceWayIds = streetNeighbor.SourceWayIds.Append(streetNeighbor.WayId)
                        .Concat(segment.SourceWayIds).Append(segment.WayId).Distinct().ToArray(),
                    SourceNodeAliases =
                        streetNeighbor.SourceNodeAliases.Concat(segment.SourceNodeAliases).Distinct().ToArray()
                };

                source = streetSource;
                parent[Root(SourceNodeKey.End(segment))] = Root(SourceNodeKey.Start(segment));
                contracted[i] = true;
                streetTags++;
                break;
            }
        }

        log.Count("Short plain street stations absorbed", streetTags);
        IndexMembers();

        for (var i = 0; i < source.Count; i++)
        {
            var segment = source[i];

            if (contracted[i]
                || segment.Roundabout
                || SourceNodeKey.Start(segment) == SourceNodeKey.End(segment)
                || fixedNodes.Contains(SourceNodeKey.Start(segment))
                || fixedNodes.Contains(SourceNodeKey.End(segment))
                || NetworkDesignChecks.HorizontalArcLength(NetworkDesignChecks.Course(segment)) >= FoldLength)
                continue;

            var folded = false;

            foreach (var start in new[] { true, false })
            {
                var key = start ? SourceNodeKey.Start(segment) : SourceNodeKey.End(segment);

                if (degree[key] != 2)
                    continue;

                foreach (var (j, atStart) in ends[key])
                    if (j != i && !contracted[j] && math.dot(Leaving(source[j], atStart), Leaving(segment, start)) > .5)
                        folded = true;

                if (folded)
                    break;
            }

            if (!folded)
                continue;

            token.ThrowIfCancellationRequested();
            var firstNodeKey = Root(SourceNodeKey.Start(segment));
            var secondNodeKey = Root(SourceNodeKey.End(segment));

            if (firstNodeKey == secondNodeKey)
                continue;

            if (Joined(i, firstNodeKey, secondNodeKey))
                continue;

            parent[secondNodeKey] = firstNodeKey;
            Merge(firstNodeKey, secondNodeKey);
            contracted[i] = true;
            folds++;
        }

        log.Count("Folded short sections contracted", folds);
        // Two dead ends of one kind and level under MappingGap apart, facing each
        // other (turning under 90°) with any one-way flow running through, are one
        // road broken in the map data; their nodes become one. Nothing else is
        // joined by proximity.
        var deadEnds = new List<(SourceNodeKey Key, int Section, bool AtStart, double3 P, double2 Away)>();

        for (var i = 0; i < source.Count; i++)
            foreach (var start in new[] { true, false })
            {
                var segment = source[i];
                var key = start ? SourceNodeKey.Start(segment) : SourceNodeKey.End(segment);

                if (degree[key] != 1 || segment.Roundabout || fixedNodes.Contains(key))
                    continue;

                var curve = NetworkDesignChecks.Course(segment);
                var point = start ? curve.P0 : curve.P3;
                var otherPoint = NetworkDesignChecks.Evaluate(curve, start ? .25 : .75);
                deadEnds.Add((key, i, start, point, math.normalizesafe(otherPoint.xz - point.xz)));
            }

        Phase("micro");

        var gapCells =
            deadEnds
                .Select((item, k) => (item, k))
                .GroupBy(
                    item => ((int)Math.Floor(item.item.P.x / MappingGap), (int)Math.Floor(item.item.P.z / MappingGap)))
                .ToDictionary(group => group.Key, group => group.Select(item => item.k).ToArray());

        var gapsJoined = 0;

        for (var k = 0; k < deadEnds.Count; k++)
        {
            var item = deadEnds[k];
            var cx = (int)Math.Floor(item.P.x / MappingGap);
            var cz = (int)Math.Floor(item.P.z / MappingGap);

            for (var dz = -1; dz <= 1; dz++)
                for (var dx = -1; dx <= 1; dx++)
                    if (gapCells.TryGetValue((cx + dx, cz + dz), out var near))
                        foreach (var m in near)
                        {
                            if (m <= k)
                                continue;

                            var otherItem = deadEnds[m];
                            var firstSegment = source[item.Section];
                            var secondSegment = source[otherItem.Section];

                            if (firstSegment.Kind != secondSegment.Kind
                                || firstSegment.Layer != secondSegment.Layer
                                || firstSegment.Bridge != secondSegment.Bridge
                                || firstSegment.Tunnel != secondSegment.Tunnel
                                || firstSegment.OneWay != secondSegment.OneWay
                                || (firstSegment.OneWay && item.AtStart == otherItem.AtStart))
                                continue;

                            if (math.distance(item.P.xz, otherItem.P.xz) >= MappingGap
                                || Math.Abs(item.P.y - otherItem.P.y) >= 1
                                || math.dot(item.Away, otherItem.Away) >= 0)
                                continue;

                            var firstNodeKey = Root(item.Key);
                            var secondNodeKey = Root(otherItem.Key);

                            if (firstNodeKey == secondNodeKey)
                                continue;

                            parent[secondNodeKey] = firstNodeKey;
                            gapsJoined++;
                        }
        }

        log.Count("Map-data breaks joined between facing dead ends", gapsJoined);

        // Report 113: every road is one fitted curve, and the first stroke through a
        // junction places it. The fit works on 5 m samples, so the map lines are read
        // at that spacing too.
        var smoothRoads = DesignSwitches.On(DesignSwitches.SmoothRoads);

        // A closed ground roundabout whose radius is under half its own road width plus
        // RingIsland has no room for an island: the game draws it as a knot of
        // overlapping pieces a few metres long. Its nodes become one plain junction at
        // the ring's centre. A ring is left alone when a road joins it twice or two of
        // its arms come from one node, which would leave a loop or a pair of roads
        // between two nodes.
        if (smoothRoads)
        {
            var ringOf = new Dictionary<SourceNodeKey, SourceNodeKey>();

            SourceNodeKey Ring(SourceNodeKey key)
            {
                if (!ringOf.TryGetValue(key, out var up))
                    return ringOf[key] = key;

                return up == key ? key : ringOf[key] = Ring(up);
            }

            var ringSections =
                Enumerable
                    .Range(0, source.Count)
                    .Where(i => !contracted[i] && source[i].Kind == NetworkKind.Road && source[i].Roundabout)
                    .ToArray();

            foreach (var i in ringSections)
                ringOf[Ring(SourceNodeKey.Start(source[i]))] = Ring(SourceNodeKey.End(source[i]));

            var collapsed = 0;

            foreach (var ring in ringSections.GroupBy(i => Ring(SourceNodeKey.Start(source[i]))))
            {
                token.ThrowIfCancellationRequested();
                var members = ring.ToHashSet();
                var passes = new Dictionary<SourceNodeKey, int>();
                var sampled = new List<double2>();

                foreach (var i in members)
                {
                    foreach (var key in new[] { SourceNodeKey.Start(source[i]), SourceNodeKey.End(source[i]) })
                        passes[key] = passes.GetValueOrDefault(key) + 1;

                    var curve = NetworkDesignChecks.Course(source[i]);

                    for (var k = 0; k < 4; k++)
                        sampled.Add(NetworkDesignChecks.Evaluate(curve, k / 4d).xz);
                }

                if (members.Any(i => NetworkStructure.IsStructure(source[i]))
                    || passes.Values.Any(count => count != 2)
                    || passes.Keys.Any(key => Root(key) != key))
                    continue;

                var centre = double2.zero;

                foreach (var point in sampled)
                    centre += point;

                centre /= sampled.Count;
                var radius = sampled.Average(point => math.distance(point, centre));

                if (radius >= members.Max(i => source[i].NativeWidth) / 2 + RingIsland)
                    continue;

                var farEnds = new HashSet<SourceNodeKey>();
                var clear = true;

                foreach (var key in passes.Keys)
                    foreach (var (section, start) in ends[key])
                    {
                        if (members.Contains(section) || contracted[section])
                            continue;

                        var far =
                            Root(start ? SourceNodeKey.End(source[section]) : SourceNodeKey.Start(source[section]));

                        clear &= !passes.ContainsKey(far) && farEnds.Add(far);
                    }

                if (!clear)
                    continue;

                var kept = passes.Keys.OrderBy(key => key.Id).First();

                foreach (var key in passes.Keys)
                    if (key != kept)
                        parent[key] = kept;

                foreach (var i in members)
                    contracted[i] = true;

                position[kept] = centre;
                collapsed++;
            }

            log.Count("Smooth roads roundabouts too small for an island made plain junctions", collapsed);
        }

        // Where a road drawn as one line stops and starts again, its two carriageways
        // can be left as a pair of pieces a few metres long between the same two
        // junctions, sometimes one of them flagged as a bridge. The game draws that as
        // a bump or a sliver of wall. When every road piece between two nodes is
        // shorter than TwinLength, the two nodes are one junction.
        if (smoothRoads)
        {
            var between = new Dictionary<(SourceNodeKey, SourceNodeKey), List<int>>();

            for (var i = 0; i < source.Count; i++)
            {
                var segment = source[i];

                if (contracted[i] || segment.Kind != NetworkKind.Road || segment.Roundabout)
                    continue;

                var first = Root(SourceNodeKey.Start(segment));
                var second = Root(SourceNodeKey.End(segment));

                if (first == second)
                    continue;

                var pair = first.Id < second.Id ? (first, second) : (second, first);

                if (!between.TryGetValue(pair, out var list))
                    between[pair] = list = new();

                list.Add(i);
            }

            var twins = 0;

            foreach (var item in between.OrderBy(item => item.Key.Item1.Id).ThenBy(item => item.Key.Item2.Id))
            {
                var first = Root(item.Key.Item1);
                var second = Root(item.Key.Item2);

                if (item.Value.Count < 2
                    || first == second
                    || item.Value.Any(
                        i => fixedNodes.Contains(SourceNodeKey.Start(source[i]))
                            || fixedNodes.Contains(SourceNodeKey.End(source[i]))
                            || NetworkDesignChecks.HorizontalArcLength(NetworkDesignChecks.Course(source[i]))
                                >= TwinLength))
                    continue;

                token.ThrowIfCancellationRequested();
                parent[second] = first;

                foreach (var i in item.Value)
                    contracted[i] = true;

                twins++;
            }

            log.Count("Smooth roads pairs of very short pieces between two junctions made one junction", twins);
        }

        // A road or railway the map data breaks for a few dozen metres leaves two dead
        // ends facing each other. Where each lies straight ahead of the other, on the
        // same level and of the same kind, they meet halfway and become one node. Heights
        // are not known yet, so the short reach is what keeps the join gentle.
        if (smoothRoads)
        {
            var joined = new HashSet<SourceNodeKey>(parent.Keys.Concat(parent.Values));
            var inline = new List<(double Gap, int First, int Second)>();

            bool Ahead(double2 offset, double2 away)
            {
                var along = -math.dot(offset, away);
                var aside = Math.Abs(offset.x * away.y - offset.y * away.x);

                return along > 0 && aside <= InlineAside + InlineDrift * along;
            }

            for (var k = 0; k < deadEnds.Count; k++)
                for (var m = k + 1; m < deadEnds.Count; m++)
                {
                    var item = deadEnds[k];
                    var otherItem = deadEnds[m];
                    var offset = otherItem.P.xz - item.P.xz;
                    var gap = math.length(offset);

                    if (gap >= InlineGap || joined.Contains(item.Key) || joined.Contains(otherItem.Key))
                        continue;

                    var firstSegment = source[item.Section];
                    var secondSegment = source[otherItem.Section];

                    if (firstSegment.Kind != secondSegment.Kind
                        || firstSegment.Layer != secondSegment.Layer
                        || firstSegment.Bridge != secondSegment.Bridge
                        || firstSegment.Tunnel != secondSegment.Tunnel
                        || firstSegment.OneWay != secondSegment.OneWay
                        || (firstSegment.OneWay && item.AtStart == otherItem.AtStart)
                        || math.dot(item.Away, otherItem.Away) > -.8
                        || !Ahead(offset, item.Away)
                        || !Ahead(-offset, otherItem.Away))
                        continue;

                    inline.Add((gap, k, m));
                }

            var inlineJoined = 0;

            foreach (var (_, k, m) in inline.OrderBy(pair => pair.Gap))
            {
                var firstNodeKey = deadEnds[k].Key;
                var secondNodeKey = deadEnds[m].Key;

                if (joined.Contains(firstNodeKey) || joined.Contains(secondNodeKey))
                    continue;

                joined.Add(firstNodeKey);
                joined.Add(secondNodeKey);
                parent[secondNodeKey] = firstNodeKey;
                position[firstNodeKey] = position[secondNodeKey] = (deadEnds[k].P.xz + deadEnds[m].P.xz) / 2;
                inlineJoined++;
            }

            log.Count("Smooth roads breaks joined between dead ends in line", inlineJoined);
        }

        if (smoothRoads)
            CloseJunctions();

        // Report 113, 7.2: a roundabout drawn as a true circle. The circle is fitted to
        // the ring's map line; the ring's nodes and every point read from its sections
        // are put on it. An arm that already ends within RingArmAngle of the radius is
        // held on the radius there, so it points at the centre.
        var ringCircles = new Dictionary<int, (double2 Centre, double Radius)>();
        var ringCentres = new Dictionary<SourceNodeKey, double2>();

        if (smoothRoads)
            FitRings();

        void FitRings()
        {
            var ringOf = new Dictionary<SourceNodeKey, SourceNodeKey>();

            SourceNodeKey Ring(SourceNodeKey key)
            {
                if (!ringOf.TryGetValue(key, out var up))
                    return ringOf[key] = key;

                return up == key ? key : ringOf[key] = Ring(up);
            }

            var sections =
                Enumerable
                    .Range(0, source.Count)
                    .Where(i => !contracted[i] && source[i].Kind == NetworkKind.Road && source[i].Roundabout)
                    .ToArray();

            foreach (var i in sections)
                ringOf[Ring(Root(SourceNodeKey.Start(source[i])))] = Ring(Root(SourceNodeKey.End(source[i])));

            foreach (var ring in sections.GroupBy(i => Ring(Root(SourceNodeKey.Start(source[i])))))
            {
                var ends = new Dictionary<SourceNodeKey, int>();
                var sampled = new List<double2>();

                foreach (var i in ring)
                {
                    foreach (var key in new[] { Root(SourceNodeKey.Start(source[i])), Root(SourceNodeKey.End(source[i])) })
                        ends[key] = ends.GetValueOrDefault(key) + 1;

                    var curve = NetworkDesignChecks.Course(source[i]);
                    var parts = Math.Max(2, (int)Math.Ceiling(NetworkDesignChecks.HorizontalArcLength(curve) / 3));

                    for (var k = 0; k < parts; k++)
                        sampled.Add(NetworkDesignChecks.Evaluate(curve, k / (double)parts).xz);
                }

                // A closed ring passes each of its nodes exactly once.
                if (sampled.Count < 6 || ends.Values.Any(count => count != 2))
                    continue;

                // Least-squares circle through the samples (Kasa): x² + z² + a·x + b·z + c = 0
                // about the samples' mean, solved from its 3×3 normal equations.
                var mean = double2.zero;

                foreach (var point in sampled)
                    mean += point;

                mean /= sampled.Count;
                double sxx = 0, sxz = 0, szz = 0, sxr = 0, szr = 0, sr = 0;

                foreach (var point in sampled)
                {
                    var d = point - mean;
                    var r = math.lengthsq(d);
                    sxx += d.x * d.x;
                    sxz += d.x * d.y;
                    szz += d.y * d.y;
                    sxr += d.x * r;
                    szr += d.y * r;
                    sr += r;
                }

                var determinant = sxx * szz - sxz * sxz;

                if (Math.Abs(determinant) < 1e-6)
                    continue;

                var a = (sxr * szz - szr * sxz) / determinant;
                var b = (szr * sxx - sxr * sxz) / determinant;
                var centre = mean + new double2(a, b) / 2;
                var radius = Math.Sqrt(sr / sampled.Count + (a * a + b * b) / 4);
                var worst = sampled.Max(point => Math.Abs(math.distance(point, centre) - radius));

                if (radius < RingMinimumRadius || radius > RingMaximumRadius || worst > Math.Max(1, RingRoundness * radius))
                {
                    log.Count("Smooth roads roundabouts left as mapped (not a circle)");
                    continue;
                }

                foreach (var i in ring)
                    ringCircles[i] = (centre, radius);

                foreach (var key in ends.Keys)
                {
                    position[key] = centre + radius * math.normalizesafe(position[key] - centre);
                    ringCentres[key] = centre;
                }

                log.Count("Smooth roads roundabouts set on a circle");
            }
        }
        // Report 113, 5.1: one rule for close junctions on ground roads, in place of
        // the class cluster pass below. Two junctions nearer than the half widths of
        // their widest roads overlap in the game, so the limit comes from the widths.
        void CloseJunctions()
        {
            var members =
                position.Keys.GroupBy(Root).ToDictionary(group => group.Key, group => group.ToList());

            double SectionLength(int i)
                => NetworkDesignChecks.HorizontalArcLength(NetworkDesignChecks.Course(source[i]));

            double Widest(SourceNodeKey key) => ends[key].Max(item => source[item.Section].NativeWidth);

            bool Held(SourceNodeKey key)
                => fixedNodes.Contains(key)
                    || ends[key].Any(item => NetworkBoundaryPolicy.EndpointMarker(source[item.Section], item.Start));

            // The direction of travel of a one-way section where it meets a node.
            double2 Flow(int i, bool start) => Leaving(source[i], start) * (start ? 1 : -1);

            // True when the section joins the two one-way carriageways of one road.
            // Contracting it would pinch the median.
            bool CrossesMedian(int i)
            {
                foreach (var (a, startOfA) in ends[SourceNodeKey.Start(source[i])])
                    foreach (var (b, startOfB) in ends[SourceNodeKey.End(source[i])])
                    {
                        var first = source[a];
                        var second = source[b];

                        if (a == i || b == i || a == b || !first.OneWay || !second.OneWay)
                            continue;

                        var sameRoad =
                            first.PairedCorridorId != 0 && first.PairedCorridorId == second.PairedCorridorId
                            || first.Name.Length > 0 && first.Name == second.Name
                            || first.Reference.Length > 0 && first.Reference == second.Reference;

                        if (sameRoad && math.dot(Flow(a, startOfA), Flow(b, startOfB)) < 0)
                            return true;
                    }

                return false;
            }

            int Importance(PreparedSegment segment)
                => 10 * (segment.Classification switch
                {
                    "primary" => 4,
                    "secondary" => 3,
                    "tertiary" => 2,
                    _ => 1
                }) + segment.Lanes;

            // Dead ends shorter than the class minimum length join their junction.
            var stubs = 0;

            for (var i = 0; i < source.Count; i++)
            {
                var segment = source[i];
                var start = SourceNodeKey.Start(segment);
                var end = SourceNodeKey.End(segment);

                if (contracted[i]
                    || !IsGroundStreet(segment)
                    || degree[start] == 1 == (degree[end] == 1)
                    || Held(start)
                    || Held(end)
                    || SectionLength(i) >= VisualCompilerTable.For(segment).MinimumSegment)
                    continue;

                var deadEnd = degree[start] == 1 ? start : end;
                var junction = degree[start] == 1 ? end : start;

                if (degree[junction] < 3 || members[Root(deadEnd)].Count != 1)
                    continue;

                var root = Root(junction);
                parent[deadEnd] = root;
                members[root].Add(deadEnd);
                members.Remove(deadEnd);
                contracted[i] = true;
                stubs++;
            }

            log.Count("Smooth roads dead ends under the minimum length removed", stubs);

            // Node pairs an earlier stage already declared one junction: a source alias
            // from one node onto the other. A section still joining such a pair is
            // contracted whatever its length, or the alias would point across it.
            var aliased = new HashSet<(long, long)>();

            foreach (var segment in source.Where(item => item.Kind == NetworkKind.Road))
                foreach (var alias in segment.SourceNodeAliases)
                    if (alias.SourceNodeId != alias.CanonicalNodeId)
                        aliased.Add((alias.SourceNodeId, alias.CanonicalNodeId));

            var candidates = new List<(double Length, int Section, double Spacing)>();

            for (var i = 0; i < source.Count; i++)
            {
                var segment = source[i];
                var start = SourceNodeKey.Start(segment);
                var end = SourceNodeKey.End(segment);
                var sameJunction = aliased.Contains((start.Id, end.Id)) || aliased.Contains((end.Id, start.Id));

                if (contracted[i]
                    || !IsGroundStreet(segment)
                    || start == end
                    || degree[start] < 3
                    || degree[end] < 3
                    || Held(start)
                    || Held(end)
                    || Math.Abs(((float3)segment.CourseStart).y - ((float3)segment.CourseEnd).y) > 1)
                    continue;

                var spacing = Math.Max(15, (Widest(start) + Widest(end)) / 2);
                var length = SectionLength(i);

                if (sameJunction || length < spacing && !CrossesMedian(i))
                    candidates.Add((length, i, spacing));
            }

            var joined = 0;
            var legs = 0;
            PreparedSegment[]? edited = null;

            foreach (var (_, i, spacing) in candidates
                .OrderBy(item => item.Length)
                .ThenBy(item => source[item.Section].WayId)
                .ThenBy(item => source[item.Section].StartNodeId)
                .ThenBy(item => source[item.Section].EndNodeId))
            {
                token.ThrowIfCancellationRequested();

                if (contracted[i])
                    continue;

                var first = Root(SourceNodeKey.Start(source[i]));
                var second = Root(SourceNodeKey.End(source[i]));

                if (first == second)
                {
                    contracted[i] = true;
                    continue;
                }

                var group = members[first].Concat(members[second]).ToList();
                var tooWide = false;

                for (var a = 0; a < group.Count && !tooWide; a++)
                    for (var b = a + 1; b < group.Count && !tooWide; b++)
                        tooWide = math.distance(position[group[a]], position[group[b]]) > 2 * spacing;

                if (tooWide)
                    continue;

                // Other sections on the two junctions: those joining the same two
                // junctions, and those running from either one to a third node.
                var parallel = new List<int>();
                var toThird = new Dictionary<SourceNodeKey, List<(int Section, bool FromFirst)>>();

                foreach (var node in group)
                    foreach (var (j, _) in ends[node])
                    {
                        if (j == i || contracted[j])
                            continue;

                        var from = Root(SourceNodeKey.Start(source[j]));
                        var to = Root(SourceNodeKey.End(source[j]));
                        var fromInside = from == first || from == second;
                        var toInside = to == first || to == second;

                        if (fromInside && toInside)
                        {
                            if (from != to && !parallel.Contains(j))
                                parallel.Add(j);

                            continue;
                        }

                        var third = fromInside ? to : from;
                        var inside = fromInside ? from : to;

                        if (!toThird.TryGetValue(third, out var list))
                            toThird[third] = list = new();

                        if (!list.Contains((j, inside == first)))
                            list.Add((j, inside == first));
                    }

                bool Removable(int j) => IsGroundStreet(source[j]) && SectionLength(j) < 2 * spacing;

                // A one-way section seen from the third node: true when it leads away from it.
                bool LeavesThird(int j, SourceNodeKey third) => Root(SourceNodeKey.Start(source[j])) == third;

                var remove = new List<(int Section, int Kept)>();
                var blocked = parallel.Any(j => !Removable(j));

                foreach (var pair in toThird)
                {
                    if (blocked)
                        break;

                    if (pair.Value.All(item => item.FromFirst) || pair.Value.All(item => !item.FromFirst))
                        continue;

                    // The two junctions and this third node make a triangle. The most
                    // important leg stays; a leg from the other junction is removed
                    // if it is short and the leg that stays carries its direction.
                    var kept =
                        pair.Value
                            .OrderByDescending(item => Importance(source[item.Section]))
                            .ThenBy(item => SectionLength(item.Section))
                            .ThenBy(item => source[item.Section].WayId)
                            .First();

                    foreach (var leg in pair.Value.Where(item => item.FromFirst != kept.FromFirst))
                    {
                        var carried =
                            !source[kept.Section].OneWay
                            || source[leg.Section].OneWay
                                && LeavesThird(leg.Section, pair.Key) == LeavesThird(kept.Section, pair.Key);

                        if (!carried || !Removable(leg.Section))
                        {
                            blocked = true;
                            break;
                        }

                        remove.Add((leg.Section, kept.Section));
                    }
                }

                if (blocked)
                    continue;

                foreach (var (leg, kept) in remove)
                {
                    // The removed leg's ways and aliases stay checkable on the leg that is kept.
                    edited ??= source.ToArray();
                    edited[kept] = edited[kept] with
                    {
                        SourceWayIds = edited[kept].SourceWayIds.Append(edited[kept].WayId)
                            .Concat(edited[leg].SourceWayIds).Append(edited[leg].WayId).Distinct().ToArray(),
                        SourceNodeAliases =
                            edited[kept].SourceNodeAliases.Concat(edited[leg].SourceNodeAliases).Distinct().ToArray()
                    };

                    source = edited;
                    contracted[leg] = true;
                    legs++;
                }

                foreach (var j in parallel)
                    contracted[j] = true;

                parent[second] = first;
                members[first].AddRange(members[second]);
                members.Remove(second);
                contracted[i] = true;
                joined++;
            }

            // A loop shorter than JunctionLoop that leaves a junction and comes back to it
            // through plain nodes no further than JunctionLoopReach away (a turning
            // circle, a U-turn pocket, or what is left of a small triangle once two of
            // its corners are joined) is too small to build as roads of its own. Its
            // nodes join the junction, so they stay known as aliases of it.
            var loops = 0;

            List<(int Section, bool Start)> Live(SourceNodeKey root)
                => members[root].SelectMany(node => ends[node]).Where(item => !contracted[item.Section]).ToList();

            foreach (var root in members.Keys.OrderBy(key => key.Kind).ThenBy(key => key.Id).ToList())
            {
                token.ThrowIfCancellationRequested();

                if (root.Kind != NetworkKind.Road || !members.ContainsKey(root) || members[root].Any(Held))
                    continue;

                var leaving = Live(root);

                if (leaving.Count < 3)
                    continue;

                foreach (var (first, firstStart) in leaving)
                {
                    if (contracted[first])
                        continue;

                    var chain = new List<int>();
                    var passed = new List<SourceNodeKey>();
                    var length = 0d;
                    var section = first;
                    var fromStart = firstStart;
                    var closed = false;

                    while (!source[section].Roundabout)
                    {
                        chain.Add(section);
                        length += SectionLength(section);

                        if (length >= JunctionLoop)
                            break;

                        var far = Root(fromStart ? SourceNodeKey.End(source[section]) : SourceNodeKey.Start(source[section]));

                        if (far == root)
                        {
                            closed = true;
                            break;
                        }

                        if (members[far].Any(Held) || math.distance(position[far], position[root]) > JunctionLoopReach)
                            break;

                        var onward = Live(far).Where(item => item.Section != section).ToList();

                        if (onward.Count != 1)
                            break;

                        passed.Add(far);
                        (section, fromStart) = onward[0];
                    }

                    if (!closed || passed.Count == 0)
                        continue;

                    foreach (var j in chain)
                        contracted[j] = true;

                    foreach (var node in passed)
                    {
                        parent[node] = root;
                        members[root].AddRange(members[node]);
                        members.Remove(node);
                    }

                    loops++;
                }
            }

            // Joining two junctions can leave the leg that was kept as a dead end: the
            // node it ran to had only the removed leg besides. Such an end was not a dead
            // end in the map, so it goes when it is no longer than a removed leg may be
            // (twice the junction spacing). A real dead end still goes only under the
            // class minimum length. Either way its node joins the junction as an alias.
            var leftovers = 0;

            for (var i = 0; i < source.Count; i++)
            {
                var segment = source[i];

                if (contracted[i] || !IsGroundStreet(segment))
                    continue;

                var start = Root(SourceNodeKey.Start(segment));
                var end = Root(SourceNodeKey.End(segment));
                var length = SectionLength(i);

                if (start == end || length >= 2 * Math.Max(15, segment.NativeWidth))
                    continue;

                var atStart = Live(start).Count;
                var atEnd = Live(end).Count;

                if (atStart == 1 == (atEnd == 1))
                    continue;

                var deadEnd = atStart == 1 ? start : end;
                var junction = atStart == 1 ? end : start;
                var mapped = members[deadEnd].Any(node => degree[node] == 1);

                if (Math.Max(atStart, atEnd) < 3
                    || members[deadEnd].Any(Held)
                    || members[junction].Any(Held)
                    || mapped && length >= VisualCompilerTable.For(segment).MinimumSegment)
                    continue;

                parent[deadEnd] = junction;
                members[junction].AddRange(members[deadEnd]);
                members.Remove(deadEnd);
                contracted[i] = true;
                leftovers++;
            }

            log.Count("Smooth roads dead ends left by joined junctions removed", leftovers);
            log.Count("Smooth roads close junctions joined", joined);
            log.Count("Smooth roads triangle legs removed at joined junctions", legs);
            log.Count("Smooth roads short loops joined into their junction", loops);
            log.Diagnostic(
                $"SMOOTH ROADS junction rule candidates={candidates.Count}; joined={joined}; triangleLegsRemoved={legs}; deadEndsRemoved={stubs}");
        }

        for (var i = 0; i < source.Count; i++)
        {
            var segment = source[i];

            if (contracted[i] || smoothRoads && segment.Kind == NetworkKind.Road)
                continue;

            var limit = VisualCompilerTable.For(segment).ClusterLength;

            if (limit <= 0
                || segment.Roundabout
                || NetworkStructure.IsStructure(segment)
                || degree[SourceNodeKey.Start(segment)] < 3
                || degree[SourceNodeKey.End(segment)] < 3
                || fixedNodes.Contains(SourceNodeKey.Start(segment))
                || fixedNodes.Contains(SourceNodeKey.End(segment)))
                continue;

            // A main road never joins two highway nodes; with one, the highway node survives.
            if (!NativeAssetKeys.IsHighway(segment)
                && (ends[SourceNodeKey.Start(segment)].Any(item => NativeAssetKeys.IsHighway(source[item.Section]))
                    && ends[SourceNodeKey.End(segment)].Any(item => NativeAssetKeys.IsHighway(source[item.Section]))))
                continue;

            if (NetworkDesignChecks.HorizontalArcLength(NetworkDesignChecks.Course(segment)) >= limit)
                continue;

            var firstNodeKey = Root(SourceNodeKey.Start(segment));
            var secondNodeKey = Root(SourceNodeKey.End(segment));

            if (firstNodeKey != secondNodeKey)
                parent[secondNodeKey] = firstNodeKey;

            contracted[i] = true;
        }

        var majorArms = new Dictionary<SourceNodeKey, int>();

        foreach (var segment in source.Where(otherSegment => !otherSegment.ProtectedLink))
            foreach (var key in new[] { SourceNodeKey.Start(segment), SourceNodeKey.End(segment) })
                majorArms[key] = majorArms.TryGetValue(key, out var c) ? c + 1 : 1;

        var canonical = new Dictionary<SourceNodeKey, SourceNodeKey>();

        foreach (var group in position.Keys.GroupBy(Root))
        {
            var best =
                group
                    .OrderByDescending(
                        nodeKey => ends[nodeKey].Any(item => NativeAssetKeys.IsHighway(source[item.Section])))
                .ThenByDescending(nodeKey => majorArms.TryGetValue(nodeKey, out var c) ? c : 0)
                    .ThenBy(nodeKey => nodeKey.Id)
                    .First();

            foreach (var key in group)
                canonical[key] = best;
        }

        SourceNodeKey Canon(SourceNodeKey key) => canonical[key];

        if (smoothRoads)
            PlaceJoinedJunctions();

        // A junction joined from two or more close junctions sat where its canonical member
        // was, so the road through it jogged sideways to reach it: on avenues past a
        // staggered crossroads or a divided cross road, a hook of 10-30 m radius in the last
        // section (report 120). The joined node goes instead onto the line of its most
        // important road through it, at the point nearest the old junctions' centre, and the
        // minor arms bend. A group with a fixed or held node, a highway, a roundabout or a
        // structure keeps its position, as does one that would move more than
        // JoinedJunctionShift.
        void PlaceJoinedJunctions()
        {
            static int Importance(PreparedSegment segment)
                => 10 * (segment.Classification switch
                {
                    "primary" => 4,
                    "secondary" => 3,
                    "tertiary" => 2,
                    _ => 1
                }) + segment.Lanes;

            bool Holds((int Section, bool Start) item)
            {
                var segment = source[item.Section];

                return NativeAssetKeys.IsHighway(segment)
                    || segment.Roundabout
                    || NetworkStructure.IsStructure(segment)
                    || (item.Start ? segment.StartRoundaboutRadius : segment.EndRoundaboutRadius) > 0
                    || NetworkBoundaryPolicy.EndpointMarker(segment, item.Start);
            }

            // A point about 30 m into the arm, at most halfway along it.
            double2 Inward((int Section, bool Start) arm)
            {
                var curve = NetworkDesignChecks.Course(source[arm.Section]);
                var fraction = Math.Min(.5, 30 / Math.Max(1, NetworkDesignChecks.HorizontalArcLength(curve)));

                return NetworkDesignChecks.Evaluate(curve, arm.Start ? fraction : 1 - fraction).xz;
            }

            var placed = 0;

            foreach (var group in position.Keys.Where(key => key.Kind == NetworkKind.Road).GroupBy(Canon))
            {
                var junctions = group.Where(key => degree[key] >= 3).ToList();

                if (junctions.Count < 2
                    || group.Any(key => fixedNodes.Contains(key) || ends[key].Any(Holds)))
                    continue;

                var arms = new List<(int Section, bool Start)>();

                foreach (var key in group)
                    foreach (var item in ends[key])
                    {
                        var far =
                            item.Start
                                ? SourceNodeKey.End(source[item.Section])
                                : SourceNodeKey.Start(source[item.Section]);

                        if (!contracted[item.Section] && Canon(far) != group.Key)
                            arms.Add(item);
                    }

                (int Section, bool Start)? throughA = null;
                (int Section, bool Start)? throughB = null;
                var best = -1;

                for (var a = 0; a < arms.Count; a++)
                    for (var b = a + 1; b < arms.Count; b++)
                    {
                        var importance = Importance(source[arms[a].Section]) + Importance(source[arms[b].Section]);
                        var opposite =
                            math.dot(
                                Leaving(source[arms[a].Section], arms[a].Start),
                                Leaving(source[arms[b].Section], arms[b].Start))
                                < -.87;

                        if (opposite && importance > best)
                        {
                            best = importance;
                            throughA = arms[a];
                            throughB = arms[b];
                        }
                    }

                if (throughA == null || throughB == null)
                    continue;

                var centre = default(double2);

                foreach (var key in junctions)
                    centre += position[key];

                centre /= junctions.Count;
                var from = Inward(throughA.Value);
                var line = Inward(throughB.Value) - from;
                var along = math.clamp(math.dot(centre - from, line) / Math.Max(1e-6, math.lengthsq(line)), 0, 1);
                var target = from + along * line;

                if (math.distance(target, centre) > JoinedJunctionShift)
                    continue;

                position[group.Key] = target;
                placed++;
            }

            log.Count("Smooth roads joined junctions placed on their through road", placed);
        }

        var roundaboutRadii = new Dictionary<SourceNodeKey, float>();

        foreach (var segment in source.Where(otherSegment => otherSegment.Kind == NetworkKind.Road))
        {
            if (segment.StartRoundaboutRadius > 0)
                roundaboutRadii[Canon(SourceNodeKey.Start(segment))] = segment.StartRoundaboutRadius;

            if (segment.EndRoundaboutRadius > 0)
                roundaboutRadii[Canon(SourceNodeKey.End(segment))] = segment.EndRoundaboutRadius;
        }

        Phase("gap cells");

        var absorbed =
            canonical.Where(entry => entry.Key != entry.Value).ToDictionary(entry => entry.Key, entry => entry.Value);

        // The merged nodes of each node that absorbed others, in the order of absorbed,
        // which is not changed after this, and each merged node's place in that order.
        // The strokes ask for them at every node, and Emit at both ends of every piece.
        var absorbedBy = new Dictionary<SourceNodeKey, List<SourceNodeKey>>();
        var absorbedOrder = new Dictionary<SourceNodeKey, int>();

        foreach (var entry in absorbed)
        {
            if (!absorbedBy.TryGetValue(entry.Value, out var merged))
                absorbedBy[entry.Value] = merged = new();

            merged.Add(entry.Key);
            absorbedOrder[entry.Key] = absorbedOrder.Count;
        }

        var arms = new Dictionary<SourceNodeKey, List<Arm>>();

        for (var i = 0; i < source.Count; i++)
            if (!contracted[i])
                foreach (var start in new[] { true, false })
                {
                    var key = Canon(start ? SourceNodeKey.Start(source[i]) : SourceNodeKey.End(source[i]));

                    if (!arms.TryGetValue(key, out var list))
                        arms[key] = list = new();

                    list.Add(new(i, start));
                }

        var points = new List<double2>[source.Count];
        var gridShift = new Dictionary<SourceNodeKey, double2>();

        var alignment = new RoadAlignment();
        var placed = new HashSet<SourceNodeKey>();

        // Every fitted road's samples by SideCell, with the nodes of each fitted stroke.
        var placedSamples = new List<(int Stroke, int Section, double2 P, double2 D)>();
        var placedGrid = new Dictionary<(int, int), List<int>>();
        var placedNodes = new List<HashSet<SourceNodeKey>>();

        // Where each section's own curve ends, before its ends are put on their nodes.
        var sourceEnds = new (double2 Start, double2 End)[source.Count];

        // The fitted direction in which a section leaves its start or end node.
        var placedArms = new Dictionary<(int Section, bool AtStart), double2>();
        var sampleStep = smoothRoads ? RoadAlignment.Spacing : 15;
        var sampleLimit = smoothRoads ? 256 : 64;

        List<double2> Curve2(int i)
        {
            var curve = NetworkDesignChecks.Course(source[i]);
            var list = new List<double2>();
            var n =
                Math.Max(
                    1,
                    Math.Min(
                        sampleLimit,
                        (int)Math.Ceiling(NetworkDesignChecks.HorizontalArcLength(curve) / sampleStep)));

            for (var k = 0; k <= n; k++)
                list.Add(NetworkDesignChecks.Evaluate(curve, k / (double)n).xz);

            if (ringCircles.TryGetValue(i, out var circle))
                for (var k = 0; k < list.Count; k++)
                    list[k] = circle.Centre + circle.Radius * math.normalizesafe(list[k] - circle.Centre);

            return list;
        }

        Phase("absorbed nodes");

        var shift = Separate(source, contracted, Curve2, Canon, levelCrossings, token);

        foreach (var pair in shift)
            position[pair.Key] += pair.Value;

        // Track cannot bend at a point. A plain main-line rail node beside a short
        // section (under JogLength) that turns the line more than 30°, which no
        // railway curve does within 30 m, is a jog left by consolidation: a 4 m
        // zigzag or hook, or a 13 m detour through a node of a removed parallel
        // track. It moves onto the chord between its neighbours (at most
        // JogLength / 2), over three passes so a detour's shoulders follow its
        // apex. The shift carries linearly along both sections, as for
        // carriageway separation.
        var jogs = 0;

        for (var pass = 0; pass < 3; pass++)
            foreach (var node in arms)
            {
                var list = node.Value;

                if (node.Key.Kind != NetworkKind.Rail
                    || list.Count != 2
                    || fixedNodes.Contains(node.Key)
                    || list[0].Section == list[1].Section
                    || list.Any(arm => source[arm.Section].Classification != "rail"))
                    continue;

                var point = position[node.Key];

                // A short section's far node, or the point JogLength along a longer one.
                (double2 Point, bool Short) Toward(Arm arm)
                {
                    var far =
                        position[Canon(
                            arm.AtStart
                                ? SourceNodeKey.End(source[arm.Section])
                                : SourceNodeKey.Start(source[arm.Section]))];

                    if (math.distance(point, far) < JogLength)
                        return (far, true);

                    var line = Curve2(arm.Section);

                    if (!arm.AtStart)
                        line.Reverse();

                    var walked = 0d;

                    for (var k = 1; k < line.Count; k++)
                    {
                        walked += math.distance(line[k - 1], line[k]);

                        if (walked >= JogLength)
                            return (point + line[k] - line[0], false);
                    }

                    return (far, false);
                }

                var (vectorA, shortA) = Toward(list[0]);
                var (vectorB, shortB) = Toward(list[1]);

                if (!shortA && !shortB)
                    continue;

                var u = math.normalizesafe(vectorA - point);
                var v = math.normalizesafe(vectorB - point);

                if (math.dot(u, v) <= -Math.Cos(math.radians(30)))
                    continue;

                // A node projecting near a chord end would land on its neighbour: two
                // distinct nodes 0.4 m apart, which the game joins wrongly and leaves
                // as a break. Such jogs, and moves ending within 2 m of any other rail
                // node, are left as they are.
                var ab = vectorB - vectorA;
                var t = math.dot(point - vectorA, ab) / Math.Max(1e-9, math.lengthsq(ab));

                if (t < .15 || t > .85)
                    continue;

                var move = vectorA + ab * t - point;

                if (math.length(move) > JogLength / 2)
                    continue;

                var target = point + move;
                var crowded = false;

                foreach (var other in arms.Keys)
                    if (other.Kind == NetworkKind.Rail
                        && other != node.Key
                        && math.distancesq(position[other], target) < 4)
                    {
                        crowded = true;
                        break;
                    }

                if (crowded)
                    continue;

                position[node.Key] += move;
                shift[node.Key] = shift.TryGetValue(node.Key, out var old) ? old + move : move;
                jogs++;
            }

        log.Count("Rail jogs straightened", jogs);

        List<double2> Points(int i)
        {
            // The section's own curve, already fitted to OSM by the preparation
            // stages (SourceGeometry can span a whole way, beyond this section),
            // carrying its end nodes' carriageway separation shifts linearly.
            if (points[i] != null)
                return points[i];

            var segment = source[i];
            var list = Curve2(i);
            shift.TryGetValue(Canon(SourceNodeKey.Start(segment)), out var vectorA);
            shift.TryGetValue(Canon(SourceNodeKey.End(segment)), out var vectorB);

            for (var k = 0; k < list.Count; k++)
                list[k] += math.lerp(vectorA, vectorB, k / (double)Math.Max(1, list.Count - 1));

            if (gridShift.Count > 0)
            {
                gridShift.TryGetValue(Canon(SourceNodeKey.Start(segment)), out var gridA);
                gridShift.TryGetValue(Canon(SourceNodeKey.End(segment)), out var gridB);

                for (var k = 0; k < list.Count; k++)
                {
                    var t = k / (double)Math.Max(1, list.Count - 1);
                    list[k] += math.lerp(gridA, gridB, t * t * (3 - 2 * t));
                }
            }

            sourceEnds[i] = (list[0], list[list.Count - 1]);
            list[0] = position[Canon(SourceNodeKey.Start(segment))];
            list[list.Count - 1] = position[Canon(SourceNodeKey.End(segment))];

            return points[i] = list;
        }

        double2 Away(Arm arm)
        {
            var otherPoints = Points(arm.Section);

            if (!arm.AtStart)
                otherPoints = Enumerable.Reverse(otherPoints).ToList();

            var walked = 0d;

            for (var k = 1; k < otherPoints.Count; k++)
            {
                walked += math.distance(otherPoints[k - 1], otherPoints[k]);

                if (walked >= 30 || k == otherPoints.Count - 1)
                    return math.normalizesafe(otherPoints[k] - otherPoints[0]);
            }

            return default;
        }

        var next = new Dictionary<Arm, Arm>();

        foreach (var node in arms.OrderBy(entry => entry.Key.Kind).ThenBy(entry => entry.Key.Id))
        {
            token.ThrowIfCancellationRequested();
            var list = node.Value;

            if (fixedNodes.Contains(node.Key))
                continue;

            var candidates = new List<(double Score, Arm X, Arm Y)>();

            for (var a = 0; a < list.Count; a++)
                for (var b = a + 1; b < list.Count; b++)
                {
                    var firstSegment = source[list[a].Section];
                    var secondSegment = source[list[b].Section];
                    // At a node of degree two the road goes on whatever its tags say
                    // next: the A551 becomes a motorway_link 68 m before it joins the
                    // A7, 3 m from its centreline, and ending the stroke there fixed
                    // that point, so the merge pushed the link out and back (a hump).
                    var plain = list.Count == 2;

                    if (list[a].Section == list[b].Section
                        || firstSegment.Kind != secondSegment.Kind
                        || !plain && firstSegment.ProtectedLink != secondSegment.ProtectedLink
                        || firstSegment.OneWay != secondSegment.OneWay
                        || firstSegment.Roundabout
                        || secondSegment.Roundabout)
                        continue;

                    if (!plain
                        && firstSegment.Kind == NetworkKind.Road
                        && firstSegment.Classification != secondSegment.Classification
                        && !(IsMainline(firstSegment) && IsMainline(secondSegment)))
                        continue;

                    if (firstSegment.OneWay && list[a].AtStart == list[b].AtStart)
                        continue;

                    // At a junction only a gentle continuation is the same road; at a
                    // degree-two node a sharp bend is still one road and gets a fillet.
                    var deflection =
                        math.degrees(Math.Acos(math.clamp(-math.dot(Away(list[a]), Away(list[b])), -1, 1)));

                    if (deflection >= (list.Count > 2 ? 35 : 150))
                        continue;

                    var bonus =
                        firstSegment.Reference.Length > 0 && firstSegment.Reference == secondSegment.Reference
                            ? 10
                            : firstSegment.Name.Length > 0 && firstSegment.Name == secondSegment.Name ? 5 : 0;

                    candidates.Add((deflection - bonus, list[a], list[b]));
                }

            foreach (var item in candidates
                .OrderBy(otherItem => otherItem.Score)
                .ThenBy(otherItem => otherItem.X.Section)
                .ThenBy(otherItem => otherItem.Y.Section))
                if (!next.ContainsKey(item.X) && !next.ContainsKey(item.Y))
                {
                    next[item.X] = item.Y;
                    next[item.Y] = item.X;
                }
        }

        // The polyline leaving a node along an arm, followed through the
        // sections that continue it, at least length long where it goes on;
        // sections, if given, receives the section of every point.
        List<double2> Chain(Arm arm, double length, List<int>? sections = null)
        {
            var chain = new List<double2>();
            var current = arm;
            var walked = 0d;
            var guard = 0;

            while (guard++ < 64)
            {
                var otherPoints = Points(current.Section);

                if (!current.AtStart)
                    otherPoints = Enumerable.Reverse(otherPoints).ToList();

                foreach (var point in chain.Count == 0 ? otherPoints : otherPoints.Skip(1))
                {
                    if (chain.Count > 0)
                        walked += math.distance(chain[chain.Count - 1], point);

                    chain.Add(point);
                    sections?.Add(current.Section);
                }

                if (walked >= length
                    || !next.TryGetValue(new Arm(current.Section, !current.AtStart), out var following))
                    break;

                current = following;
            }

            return chain;
        }

        var output = new List<PreparedSegment>();
        var visited = new bool[source.Count];
        long generated = GeneratedNodeBase;
        var nodeY = new Dictionary<SourceNodeKey, float>();
        var exact = new Dictionary<SourceNodeKey, float3>();

        float JunctionHeight(SourceNodeKey key)
        {
            if (!nodeY.TryGetValue(key, out var y))
            {
                var point = position[key];
                nodeY[key] = y = NetworkProcessor.Sample((float)point.x, (float)point.y, terrain, width);
            }

            return y;
        }

        // Side-by-side clearance samples, built on first use (SideClear).
        List<(int Section, double2 P, double2 D)>? sideSamples = null;
        Dictionary<(int, int), List<int>>? sideGrid = null;
        Phase("separation");

        var counts =
            (
                Strokes: 0,
                Pieces: 0,
                Corners: 0,
                Merged: 0,
                Projected: 0,
                ProjectionMiss: 0,
                VerticalMisfit: 0,
                Merges: 0,
                Absorbed: 0,
                Rebalanced: 0,
                ShortRuns: 0,
                SidePushes: 0,
                CornersDropped: 0,
                GuardedPoints: 0);

        var plans = new List<VisualCompilerProfile.Stroke>();
        var deferred = new List<Action>();
        var passthrough = new List<int>();
        var paths = new List<List<(int Section, bool Forward)>>();

        for (var i = 0; i < source.Count; i++)
        {
            if (contracted[i] || visited[i])
                continue;

            token.ThrowIfCancellationRequested();

            if (source[i].Roundabout)
            {
                visited[i] = true;
                passthrough.Add(i);
                continue;
            }

            // Walk back to the stroke start, then forward through paired arms.
            var start = new Arm(i, true);
            var guard = 0;

            while (next.TryGetValue(start, out var previous) && previous.Section != i && guard++ < source.Count)
                start = new(previous.Section, !previous.AtStart);

            var path = new List<(int Section, bool Forward)>();
            var entry = start;

            while (!visited[entry.Section])
            {
                visited[entry.Section] = true;
                path.Add((entry.Section, entry.AtStart));
                var exit = new Arm(entry.Section, !entry.AtStart);

                if (!next.TryGetValue(exit, out var paired) || visited[paired.Section])
                    break;

                entry = paired;
            }

            counts.Strokes++;
            paths.Add(path);
        }

        if (smoothRoads)
            OneAssetPerStretch();

        // Report 113, 7.4: the map changes lane count for a block and back (a turn lane,
        // a mapping slip), and each change is a visible step in the game. Along a road
        // stroke, a run shorter than AssetRunLength between two stretches that share one
        // asset takes their lane counts, width and asset. Bridge, tunnel, layer,
        // direction and road family never change. A neighbour with unequal lane counts
        // each way gives them only to sections drawn in its own direction, because its
        // lane sides follow the section direction.
        void OneAssetPerStretch()
        {
            var result = source.ToArray();
            var changed = 0;

            bool SameAsset(PreparedSegment segment, PreparedSegment otherSegment)
                => NativeAssetKeys.Sized(segment) == NativeAssetKeys.Sized(otherSegment)
                    && segment.NativeWidth == otherSegment.NativeWidth
                    && segment.Tunnel == otherSegment.Tunnel
                    && segment.Layer == otherSegment.Layer;

            foreach (var path in paths)
            {
                if (source[path[0].Section].Kind != NetworkKind.Road)
                    continue;

                var from = 0;

                while (from < path.Count)
                {
                    var to = from;

                    while (to + 1 < path.Count && SameAsset(source[path[from].Section], source[path[to + 1].Section]))
                        to++;

                    if (from > 0 && to + 1 < path.Count)
                    {
                        var before = path[from - 1];
                        var neighbour = result[before.Section];
                        var after = source[path[to + 1].Section];
                        var first = source[path[from].Section];
                        var length = 0d;

                        for (var k = from; k <= to; k++)
                            length +=
                                NetworkDesignChecks.HorizontalArcLength(
                                    NetworkDesignChecks.Course(source[path[k].Section]));

                        var even = neighbour.OneWay || neighbour.ForwardLanes == neighbour.BackwardLanes;

                        if (length < AssetRunLength
                            && SameAsset(neighbour, after)
                            && NativeAssetKeys.Family(neighbour) == NativeAssetKeys.Family(first)
                            && neighbour.PairedSpanId == 0
                            && Enumerable.Range(from, to - from + 1).All(
                                k => source[path[k].Section].PairedSpanId == 0
                                    && (even || path[k].Forward == before.Forward)))
                        {
                            for (var k = from; k <= to; k++)
                            {
                                result[path[k].Section] = source[path[k].Section] with
                                {
                                    Lanes = neighbour.Lanes,
                                    ForwardLanes = neighbour.ForwardLanes,
                                    BackwardLanes = neighbour.BackwardLanes,
                                    BothWaysLanes = neighbour.BothWaysLanes,
                                    NativeWidth = neighbour.NativeWidth,
                                    NativeAssetKey = neighbour.NativeAssetKey,
                                    NativeLaneOffsets = neighbour.NativeLaneOffsets
                                };

                                changed++;
                            }
                        }
                    }

                    from = to + 1;
                }
            }

            source = result;
            log.Count("Smooth roads short runs given their neighbours' asset", changed);
        }
        // Each section's stroke, and the nodes each stroke passes through (not its
        // ends): a stroke starting or ending on another's interior branches off it.
        var strokeOf = new int[source.Count];
        Array.Fill(strokeOf, -1);
        var interior = new HashSet<SourceNodeKey>[paths.Count];
        var strokeEnds = new (SourceNodeKey Start, SourceNodeKey End)[paths.Count];

        for (var p = 0; p < paths.Count; p++)
        {
            var path = paths[p];
            interior[p] = new();

            foreach (var (section, _) in path)
                strokeOf[section] = p;

            for (var k = 1; k < path.Count; k++)
            {
                var segment = source[path[k].Section];
                interior[p].Add(Canon(path[k].Forward ? SourceNodeKey.Start(segment) : SourceNodeKey.End(segment)));
            }

            var first = source[path[0].Section];
            var last = source[path[^1].Section];
            strokeEnds[p] =
                (
                    Canon(path[0].Forward ? SourceNodeKey.Start(first) : SourceNodeKey.End(first)),
                    Canon(path[^1].Forward ? SourceNodeKey.End(last) : SourceNodeKey.Start(last)));
        }

        bool BranchOf(int t, int s)
            => t != s && (interior[s].Contains(strokeEnds[t].Start) || interior[s].Contains(strokeEnds[t].End));

        Phase("strokes and profiles");

        var laneShifted = LaneDatum();
        var dryStreets = new HashSet<int>();
        Phase("lane datum");

        var streetLines = AlignStreets();
        var twoWayApproaches = 0;
        var carriagewaySplits = 0;
        Phase("street alignment");

        var approach = MergeApproaches();
        var crossingDirections = new Dictionary<Arm, double2>();

        foreach (var node in arms.Where(entry => levelCrossings.Contains(entry.Key.Id) && entry.Value.Count == 2))
        {
            double2 Heading(Arm arm)
            {
                var chain = Chain(arm, 100);
                var walked = 0d;

                for (var k = 1; k < chain.Count; k++)
                {
                    var length = math.distance(chain[k - 1], chain[k]);

                    if (walked + length >= 100)
                        return math.normalizesafe(
                            math.lerp(chain[k - 1], chain[k], (100 - walked) / length) - chain[0]);

                    walked += length;
                }

                return math.normalizesafe(chain[^1] - chain[0]);
            }

            var firstArm = node.Value[0];
            var secondArm = node.Value[1];
            var da = Heading(firstArm);
            var db = Heading(secondArm);

            if (math.dot(da, db) > -Math.Cos(math.radians(35)))
                continue;

            var axis = math.normalizesafe(da - db);
            crossingDirections[firstArm] = axis;
            crossingDirections[secondArm] = -axis;
        }

        Phase("merge approaches");

        var forks = ForkClearance();
        var forkPushes = 0;

        // Motorways and trunks are fitted first and frozen. Everything else measures
        // its clearance from those finished curves, not from their map lines.
        bool FrozenFirst(List<(int Section, bool Forward)> path)
        {
            var segment = source[path[0].Section];
            return segment.Kind == NetworkKind.Road
                && NativeAssetKeys.IsHighway(segment)
                && !segment.ProtectedLink
                && !segment.Roundabout;
        }

        // Under smooth-roads the first stroke through a junction places it, so the order
        // decides who keeps its line: motorways, trunks, links, then each lower class,
        // the longest first, ties by way id (report 113, 5.2). Rail keeps its order.
        int FitRank(List<(int Section, bool Forward)> path)
        {
            var segment = source[path[0].Section];

            if (segment.Kind != NetworkKind.Road)
                return 7;

            if (segment.ProtectedLink)
                return 2;

            return segment.Classification switch
            {
                "motorway" => 0,
                "trunk" => 1,
                "primary" => 3,
                "secondary" => 4,
                "tertiary" => 5,
                _ => 6
            };
        }

        double FitLength(List<(int Section, bool Forward)> path)
            => source[path[0].Section].Kind == NetworkKind.Road
                ? path.Sum(
                    item => NetworkDesignChecks.HorizontalArcLength(NetworkDesignChecks.Course(source[item.Section])))
                : 0;

        Phase("fork clearance");

        var rankingStarted = System.Diagnostics.Stopwatch.GetTimestamp();

        var ordered =
            smoothRoads
                ? paths
                    .Select(path => (Path: path, Rank: FitRank(path), Length: FitLength(path)))
                    .OrderBy(item => item.Rank)
                    .ThenByDescending(item => item.Length)
                    .ThenBy(item => item.Rank == 7 ? 0 : source[item.Path[0].Section].WayId)
                    .Select(item => item.Path)
                    .ToList()
                : paths;

        var rankingTicks = System.Diagnostics.Stopwatch.GetTimestamp() - rankingStarted;

        foreach (var path in ordered.Where(FrozenFirst))
            EmitStroke(path);

        var clearanceStarted = System.Diagnostics.Stopwatch.GetTimestamp();
        approach = MergeApproaches();
        forks = ForkClearance();
        var clearanceTicks = System.Diagnostics.Stopwatch.GetTimestamp() - clearanceStarted;
        sideSamples = null;
        sideGrid = null;

        foreach (var path in ordered.Where(items => !FrozenFirst(items)))
            EmitStroke(path);

        log.Time(
            "Visual compiler ordering, of which line fitting",
            strokeFitTicks / (double)System.Diagnostics.Stopwatch.Frequency);

        log.Time("Visual compiler ordering, of which stroke ranking", Seconds(rankingTicks));
        log.Time("Visual compiler ordering, of which approaches and forks after frozen roads", Seconds(clearanceTicks));
        log.Time("Visual compiler ordering, of which stroke lines", Seconds(strokeLineTicks));
        log.Time("Visual compiler ordering, of which pushes and approaches", Seconds(strokePushTicks));
        log.Time("Visual compiler ordering, of which building pieces", Seconds(strokePieceTicks));
        log.Time("Visual compiler ordering, of which node stations", Seconds(strokeStationTicks));
        log.Time("Visual compiler ordering, of which runs and port aliases", Seconds(strokeRunTicks));
        log.Time("Visual compiler ordering, of which vertical plans", Seconds(strokePlanTicks));

        Phase("ordering");

        var roundaboutFloors = RoundaboutWaterFloors();
        Phase("roundabout water floors");

        var levelRings = LevelRings();
        VisualCompilerProfile.Solve(plans, terrain, width, log, token, roundaboutFloors, levelRings);

        // Timed apart: the "level rings" phase used to include this solve, which is most of
        // it (Barcelona: about 7 of its 7.4 s), and read as if the rings were slow.
        Phase("profile solve");

        // A shared node takes the height its first stroke solved; every stroke was
        // pinned to the same coupled value at the node's own station, so the others
        // agree closely; the mismatch diagnostic reports where they do not.
        foreach (var plan in plans)
            foreach (var pin in plan.Pins)
                if (!nodeY.ContainsKey(pin.Node))
                {
                    var height = plan.HeightAt(pin.Station);

                    if (plan.WaterNodeFloors != null && plan.WaterNodeFloors.TryGetValue(pin.Node, out var floor))
                        height = Math.Max(height, floor);

                    nodeY[pin.Node] = (float)height;
                }

        foreach (var pair in roundaboutFloors)
            nodeY[pair.Key] = (float)Math.Max(JunctionHeight(pair.Key), pair.Value);

        // Every node of a level ring takes one height, including nodes no arm reaches.
        // A ring a railway crosses takes the track's height: the track keeps its own.
        foreach (var ring in levelRings.GroupBy(pair => pair.Value, pair => pair.Key))
        {
            var track = ring.Where(key => key.Kind == NetworkKind.Rail && nodeY.ContainsKey(key)).ToArray();
            var heights = ring.Where(nodeY.ContainsKey).Select(key => nodeY[key]).ToArray();

            if (heights.Length > 0)
            {
                var height = track.Length > 0 ? nodeY[track[0]] : heights.Max();

                foreach (var key in ring)
                    if (key.Kind == NetworkKind.Road)
                        nodeY[key] = height;
            }
        }

        foreach (var node in levelCrossings)
        {
            var rail = new SourceNodeKey(NetworkKind.Rail, node);
            var road = new SourceNodeKey(NetworkKind.Road, node);

            if (nodeY.TryGetValue(rail, out var height)
                && nodeY.TryGetValue(road, out var roadHeight)
                && Math.Abs(roadHeight - height) <= .25f)
                nodeY[road] = height;
        }

        Phase("level rings");

        var mismatch =
            plans
                .SelectMany(
                    stroke => stroke.Pins.Select(pin => Math.Abs(stroke.HeightAt(pin.Station) - nodeY[pin.Node])))
                .OrderBy(d => d)
                .ToArray();

        double MismatchAt(double q) => mismatch.Length == 0 ? 0 : mismatch[(int)(q * (mismatch.Length - 1))];

        log.Diagnostic(
            $"VISUAL COMPILER junction height mismatch pins={mismatch.Length}; p50={MismatchAt(.5):F3}; p90={MismatchAt(.9):F3}; p99={MismatchAt(.99):F3}; max={MismatchAt(1):F2}; over10cm={mismatch.Count(d => d > .1)}; over30cm={mismatch.Count(d => d > .3)}");

        foreach (var emit in deferred)
            emit();

        foreach (var i in passthrough)
            output.Add(Passthrough(source[i]));

        log.Count("Visual compiler strokes", counts.Strokes);
        log.Count("Visual compiler junction nodes absorbed", absorbed.Count);
        log.Diagnostic(
            $"VISUAL COMPILER horizontal inputSections={source.Count}; contracted={contracted.Count(c => c)}; absorbedNodes={absorbed.Count}; strokes={counts.Strokes}; laneDatumNodesShifted={laneShifted}; rampApproachesAtMergeAngle={counts.Merges}; twoWayApproaches={twoWayApproaches}; carriagewaySplits={carriagewaySplits}; tightForkArms={forks.Count}; forkClearancePointsMoved={forkPushes}; sideClearancePointsMoved={counts.SidePushes}; pushesGuardedFromThirdRoads={counts.GuardedPoints}; pieces={counts.Pieces}; corners={counts.Corners}; cornerVerticesDropped={counts.CornersDropped}; mergedSections={counts.Merged}; shortPiecesAbsorbed={counts.Absorbed}; shortPiecesRebalanced={counts.Rebalanced}; shortRunsBetweenKeptNodes={counts.ShortRuns};projectedPorts={counts.Projected}; projectionMisses={counts.ProjectionMiss}; verticalMisfits={counts.VerticalMisfit}; outputSections={output.Count}");

        PhasePart("height mismatch, of which statistics and deferred emits");

        var laneAligned = NetworkLaneSlots.Align(output, log);
        PhasePart("height mismatch, of which lane slots");

        // Contracting a short connector can leave two rail routes between the
        // same pair of nodes, rebuilt here onto one stroke; crossovers and
        // connectors inside another route's footprint would overlap it in game.
        var aligned = RailCorridorConsolidator.RedundantParallelSpans(laneAligned, log, token);
        PhasePart("height mismatch, of which redundant parallel spans");

        Phase("height mismatch");

        var approaches =
            ElevateApproaches(
                RailCorridorConsolidator.RedundantOverlappingRoutes(aligned, log, token, 2),
                terrain,
                width,
                log,
                token);

        Phase("approaches");

        var finished =
            GroundAtJunctions(ElevateApproaches(approaches, terrain, width, log, token, sink: true), log, token);

        log.Count("Former bridge-boundary ports retained as aliases", structureAliases.Count);
        log.Count("Smooth roads plain ports kept as aliases", plainPorts);

        if (smoothRoads)
            log.Diagnostic($"SMOOTH ROADS plain ports kept as aliases={plainPorts}");

        var required = new HashSet<SourceNodeKey>(ports);
        required.UnionWith(structurePorts);
        Phase("finishing");

        var transferred =
            SourceTopologyObligations.Transfer(structureSource, finished, NetworkKind.Road, structureAliases);

        Phase("transfer");

        var joinedPieces = JoinShortPieces(transferred, required, levelCrossings, smoothRoads, log, token);

        var retargeted = RetargetPlainAliases(joinedPieces, position, log);
        Phase("join short pieces and retarget aliases");

        return retargeted;

        // Travel-left lane continuity (report 64 V2 step 4). Along a one-way stroke
        // the left edge stays continuous where the native width changes, so a lane
        // drop removes the travel-right lane instead of narrowing both sides. Each
        // section's centreline moves left by half the width it differs from the
        // stroke's first section, eased in over max(60 m, 30·Δ) after each change.
        // The node itself stays put; NetworkLaneSlots offsets the arms there.
        // Right-hand traffic is assumed (both benchmark cities); shifts above 6 m
        // are skipped. Mainlines go first and claim shared junction nodes, so
        // attached ramps follow them.
        int LaneDatum()
        {
            var lane = new Dictionary<SourceNodeKey, double2>();

            foreach (var path in paths.OrderBy(items => source[items[0].Section].ProtectedLink ? 1 : 0))
            {
                var first = source[path[0].Section];

                if (first.Kind != NetworkKind.Road || !first.OneWay || path.Count < 2)
                    continue;

                var changes = new List<(double Station, double Offset, double Taper)>();
                var station = 0d;
                var previous = 0d;
                var stations = new double[path.Count + 1];

                for (var k = 0; k < path.Count; k++)
                {
                    var segment = source[path[k].Section];
                    var target = (first.NativeWidth - segment.NativeWidth) / 2d;

                    if (Math.Abs(target) > 6)
                        target = previous;

                    if (Math.Abs(target - previous) > .25)
                        changes.Add((station, target, Math.Max(60, 30 * Math.Abs(target - previous))));

                    previous = target;
                    station += NetworkDesignChecks.HorizontalArcLength(NetworkDesignChecks.Course(segment));
                    stations[k + 1] = station;
                }

                if (changes.Count == 0)
                    continue;

                double Offset(double at)
                {
                    var value = 0d;
                    var from = 0d;

                    foreach (var (start, target, taper) in changes)
                    {
                        if (at <= start)
                            break;

                        value = at >= start + taper ? target : math.lerp(from, target, (at - start) / taper);
                        from = target;
                    }

                    return value;
                }

                for (var k = 0; k <= path.Count; k++)
                {
                    var segment = source[path[Math.Min(k, path.Count - 1)].Section];
                    var key =
                        Canon(
                            k == path.Count
                                ? (path[k - 1].Forward
                                    ? SourceNodeKey.End(source[path[k - 1].Section])
                                    : SourceNodeKey.Start(source[path[k - 1].Section]))
                                :
                                            (path[k].Forward
                                                ? SourceNodeKey.Start(segment)
                                                : SourceNodeKey.End(segment)));

                    var offset = Offset(stations[k]);

                    if (Math.Abs(offset) < .01 || lane.ContainsKey(key) || fixedNodes.Contains(key))
                        continue;

                    var travel = math.normalizesafe(((float3)segment.CourseEnd).xz - ((float3)segment.CourseStart).xz);
                    lane[key] = new double2(-travel.y, travel.x) * offset;
                }
            }

            foreach (var pair in lane)
            {
                position[pair.Key] += pair.Value;
                shift[pair.Key] = shift.TryGetValue(pair.Key, out var old) ? old + pair.Value : pair.Value;
            }

            Array.Clear(points, 0, points.Length);

            return lane.Count;
        }

        IReadOnlyDictionary<int, StreetGridAlignment.Line> AlignStreets()
        {
            var highways = new Dictionary<(int, int), List<double2>>();

            (int, int) HighwayCell(double2 point) => ((int)Math.Floor(point.x / 128), (int)Math.Floor(point.y / 128));

            foreach (var segment in source.Where(
                otherSegment => otherSegment.Kind == NetworkKind.Road && NativeAssetKeys.IsHighway(otherSegment)))
            {
                token.ThrowIfCancellationRequested();
                var curve = NetworkDesignChecks.Course(segment);
                var count = Math.Max(1, (int)Math.Ceiling(NetworkDesignChecks.HorizontalArcLength(curve) / 20));

                for (var k = 0; k <= count; k++)
                {
                    var point = NetworkDesignChecks.Evaluate(curve, k / (double)count).xz;
                    var key = HighwayCell(point);

                    if (!highways.TryGetValue(key, out var nearby))
                        highways[key] = nearby = new();

                    nearby.Add(point);
                }
            }

            bool NearHighway(double2 point)
            {
                var (x, z) = HighwayCell(point);

                for (var dz = -1; dz <= 1; dz++)
                    for (var dx = -1; dx <= 1; dx++)
                        if (highways.TryGetValue((x + dx, z + dz), out var nearby)
                            && nearby.Any(otherPoint => math.distancesq(point, otherPoint) < 100 * 100))
                            return true;

                return false;
            }

            var protectedNodes = new HashSet<SourceNodeKey>(arms.Where(entry => entry.Value.Any(arm =>
                source[arm.Section].Kind != NetworkKind.Road || NativeAssetKeys.IsHighway(source[arm.Section])
                    || source[arm.Section].Roundabout || NetworkStructure.IsStructure(source[arm.Section])))
                    .Select(entry => entry.Key));

            protectedNodes.UnionWith(fixedNodes);

            foreach (var node in position.Keys)
                if (math.cmax(math.abs(position[node])) >= width / 2 - 16)
                    protectedNodes.Add(node);

            var streets = new List<StreetGridAlignment.Street>();

            for (var id = 0; id < paths.Count; id++)
            {
                var path = paths[id];
                var first = source[path[0].Section];

                if (first.Kind != NetworkKind.Road
                    || path.Any(
                        item => NativeAssetKeys.IsHighway(source[item.Section]) || source[item.Section].ProtectedLink
                            || source[item.Section].Roundabout
                        || NetworkStructure.IsStructure(source[item.Section])
                        || Math.Abs(source[item.Section].NativeWidth - first.NativeWidth) > .5))
                    continue;

                var line = new List<double2>();
                var nodes = new List<SourceNodeKey>();
                var at = new List<int>();
                var wet = false;

                foreach (var (section, forward) in path)
                {
                    var segment = source[section];
                    var otherPoints = Points(section);

                    if (!forward)
                        otherPoints = otherPoints.AsEnumerable().Reverse().ToList();

                    if (line.Count == 0)
                    {
                        line.Add(otherPoints[0]);
                        nodes.Add(Canon(forward ? SourceNodeKey.Start(segment) : SourceNodeKey.End(segment)));
                        at.Add(0);
                    }

                    line.AddRange(otherPoints.Skip(1));
                    nodes.Add(Canon(forward ? SourceNodeKey.End(segment) : SourceNodeKey.Start(segment)));
                    at.Add(line.Count - 1);
                }

                if (water != null)
                    for (var k = 0; k < line.Count && !wet; k++)
                    {
                        var d = math.normalizesafe(line[Math.Min(k + 1, line.Count - 1)] - line[Math.Max(0, k - 1)]);
                        var n = new double2(-d.y, d.x) * (first.NativeWidth / 2 + 12);

                        for (var side = -1; side <= 1; side++)
                            if (water(
                                new((float)(line[k].x + n.x * side), 0, (float)(line[k].y + n.y * side))).HasValue)
                            {
                                wet = true;
                                break;
                            }
                    }

                if (!wet)
                {
                    streets.Add(new(id, nodes.ToArray(), at.ToArray(), line.ToArray(), first.NativeWidth));

                    if (!line.Any(NearHighway))
                        dryStreets.Add(id);
                }
                else
                    foreach (var node in nodes)
                        protectedNodes.Add(node);
            }

            var result = StreetGridAlignment.Prepare(streets, protectedNodes, log, token);

            foreach (var node in result.Nodes)
            {
                gridShift[node.Key] = node.Value - position[node.Key];
                position[node.Key] = node.Value;
            }

            if (result.Nodes.Count > 0)
                Array.Clear(points, 0, points.Length);

            return result.Lines;
        }

        // A one-way road joining or leaving another one-way road meets it at
        // MergeAngle (SplitAngle for a branch of two or more lanes; report 43
        // COR-003, report 64 R3), not tangentially, unless
        // every one of its lanes continues in a lane of its own: lanes after a
        // merge (before a diverge) at least both roads' lanes together. Shared
        // lanes (2 + 2 → 3), a lane drop (2 + 1 → 2) or one lane splitting into
        // two need the angle whatever the roads' classes; at 20° the pieces still
        // looked disconnected in game. The joining road's away direction is the
        // through arm on its side rotated MergeAngle toward it; Clearance is the
        // centreline spacing at which the two roads' edges are 1 m apart.
        Dictionary<Arm, (double2 Direction, double2 Through, double Clearance, List<(double2 Point, double Width)>? Partner)> MergeApproaches()
        {
            var result = new Dictionary<Arm, (double2, double2, double, List<(double2, double)>?)>();
            var angle = math.radians(MergeAngle);

            double2 Rotate(double2 v, double radians)
            {
                var c = Math.Cos(radians);
                var s = Math.Sin(radians);

                return new(v.x * c - v.y * s, v.x * s + v.y * c);
            }

            foreach (var node in arms)
            {
                var list = node.Value;

                if (list.Count < 3 || fixedNodes.Contains(node.Key))
                    continue;

                var through =
                    list
                        .Where(
                            firstArm => source[firstArm.Section].Kind == NetworkKind.Road
                                && source[firstArm.Section].OneWay
                                && !source[firstArm.Section].Roundabout
                                && next.TryGetValue(firstArm, out var secondArm)
                                && list.Contains(secondArm))
                        .ToArray();

                var twoWayThrough =
                    list
                        .Where(
                            firstArm => source[firstArm.Section].Kind == NetworkKind.Road
                                && !source[firstArm.Section].OneWay
                                && !source[firstArm.Section].Roundabout
                                && next.TryGetValue(firstArm, out var otherArm)
                                && list.Contains(otherArm))
                        .ToArray();

                // Preserve the established one-way highway merge when a second
                // two-way road also runs through the junction. Use an avenue
                // pair as the fallback, or when it is the highway of the two.
                if (through.Length == 0
                    || through.All(arm => !NativeAssetKeys.IsHighway(source[arm.Section]))
                        && twoWayThrough.Length == 2
                        && twoWayThrough.Any(arm => NativeAssetKeys.IsHighway(source[arm.Section])))
                    through = twoWayThrough;

                // A two-way road becoming two one-way carriageways (a reversible
                // express road joining both directions, or a divided road
                // starting): OSM often splits them at a sliver angle, which the
                // native node draws as a pinch. The two carriageways open
                // symmetrically about their bisector to MergeAngle between them.
                if (through.Length == 0 && list.Count == 3)
                {
                    var single =
                        list
                            .Where(
                                arm => source[arm.Section].Kind == NetworkKind.Road
                                    && !source[arm.Section].OneWay
                                    && !source[arm.Section].Roundabout)
                            .ToArray();

                    var pair =
                        list
                            .Where(
                                arm => source[arm.Section].Kind == NetworkKind.Road
                                    && source[arm.Section].OneWay
                                    && !source[arm.Section].Roundabout)
                            .ToArray();

                    if (single.Length != 1 || pair.Length != 2 || pair[0].AtStart == pair[1].AtStart)
                        continue;

                    var u = Away(pair[0]);
                    var v = Away(pair[1]);

                    if (math.degrees(Math.Acos(math.clamp(math.dot(u, v), -1, 1))) >= MergeAngle)
                        continue;

                    var bisector = math.normalizesafe(u + v);
                    var half = (source[pair[0].Section].NativeWidth + source[pair[1].Section].NativeWidth) / 4d + .5;

                    foreach (var arm in pair)
                    {
                        var side = Math.Sign(NetworkDesignChecks.Cross(bisector, Away(arm)));

                        if (side == 0)
                            side = arm == pair[0] ? 1 : -1;

                        result[arm] = (Rotate(bisector, side * angle / 2), bisector, half, null);
                    }

                    carriagewaySplits++;
                    continue;
                }

                if (through.Length != 2)
                    continue;

                if (!source[through[0].Section].OneWay)
                {
                    foreach (var branch in list.Where(arm => !through.Contains(arm)))
                    {
                        var segment = source[branch.Section];

                        if (segment.Kind != NetworkKind.Road || segment.Roundabout || next.ContainsKey(branch))
                            continue;

                        if (NativeAssetKeys.IsHighway(segment)
                            && !NativeAssetKeys.IsHighway(source[through[0].Section]))
                            continue;

                        var direction = Away(branch);
                        var partner =
                            math.dot(Away(through[0]), direction) > math.dot(Away(through[1]), direction)
                                ? through[0]
                                : through[1];

                        var m = Away(partner);

                        if (math.dot(m, direction) <= Math.Cos(angle))
                            continue;

                        var side = Math.Sign(NetworkDesignChecks.Cross(m, direction));

                        if (side == 0)
                        {
                            var line = Chain(branch, 100);

                            foreach (var point in line.Skip(1))
                            {
                                side = Math.Sign(NetworkDesignChecks.Cross(m, point - line[0]));

                                if (side != 0)
                                    break;
                            }
                        }

                        if (side == 0)
                            continue;

                        var sections = new List<int>();
                        var lineOfPartner = Chain(partner, HoldReach + 100, sections)
                                                    .Select(
                                                        (point, k) => (point, (double)source[sections[k]].NativeWidth))
                                                        .ToList();

                        result[branch] =
                            (
                                Rotate(m, side * math.radians(StreetBranchAngle)),
                                m,
                                (source[partner.Section].NativeWidth + segment.NativeWidth) / 2d + 1,
                                lineOfPartner);

                        twoWayApproaches++;
                    }

                    continue;
                }

                var up = through.FirstOrDefault(arm => !arm.AtStart);
                var down = through.FirstOrDefault(arm => arm.AtStart);

                if (up == default || down == default || up.Section == down.Section)
                    continue;

                foreach (var ramp in list)
                {
                    var segment = source[ramp.Section];

                    if (ramp == up
                        || ramp == down
                        || segment.Kind != NetworkKind.Road
                        || segment.Roundabout
                        || !segment.OneWay && !IsMainline(segment) && !segment.ProtectedLink)
                        continue;

                    var l = Away(ramp);
                    // A two-way road here is usually a reversible carriageway
                    // (express lanes), which parses as two-way: it meets the through
                    // arm it lies alongside, and always at the full angle.
                    var merge = segment.OneWay ? !ramp.AtStart : math.dot(Away(up), l) > math.dot(Away(down), l);
                    var lanesUp = source[up.Section].Lanes;
                    var lanesDown = source[down.Section].Lanes;

                    if (segment.OneWay
                        && (merge ? lanesDown >= lanesUp + segment.Lanes : lanesUp >= lanesDown + segment.Lanes))
                        continue;

                    var m = Away(merge ? up : down);

                    if (math.degrees(Math.Acos(math.clamp(math.dot(m, l), -1, 1))) >= MergeAngle)
                        continue;

                    var side = Math.Sign(NetworkDesignChecks.Cross(m, l));

                    if (side == 0)
                        continue;

                    if (!segment.OneWay)
                        twoWayApproaches++;

                    var turn = Splits(segment) ? math.radians(SplitAngle) : angle;
                    var sections = new List<int>();
                    var partner =
                        Chain(merge ? up : down, HoldReach + 100, sections)
                            .Select((point, k) => (point, (double)source[sections[k]].NativeWidth))
                            .ToList();

                    result[ramp] =
                        (
                            Rotate(m, side * turn),
                            m,
                            (source[(merge ? up : down).Section].NativeWidth + segment.NativeWidth) / 2d + 1,
                            partner);
                }
            }

            return result;
        }

        // Tight forks: two same-level one-way roads that both leave, or both join,
        // a junction within 35° of each other. OSM draws a ramp leaving the
        // motorway's centreline and staying a few metres away for 100 m or more,
        // but the native motorway is 15–20 m wide, so the ramp was drawn on top
        // of it until it finally cleared. Each such end gets its partner's
        // polyline (followed through continuing sections) and a clearance of half
        // both widths plus 2.5 m; the margin covers the partner's own smoothing
        // drift from its OSM line. A ramp gives way fully to a mainline, and a
        // branch to the road continuing through (a motorway splitting in two);
        // only two branches of equal standing each take half.
        Dictionary<Arm, List<(List<double2> Partner, double Clearance, double Share, double Taper)>> ForkClearance()
        {
            var result = new Dictionary<Arm, List<(List<double2>, double, double, double)>>();

            foreach (var node in arms)
            {
                var list = node.Value;

                if (list.Count < 3 || fixedNodes.Contains(node.Key))
                    continue;

                for (var i = 0; i < list.Count; i++)
                    for (var j = i + 1; j < list.Count; j++)
                    {
                        var firstArm = list[i];
                        var secondArm = list[j];
                        var firstSegment = source[firstArm.Section];
                        var secondSegment = source[secondArm.Section];

                        if (firstSegment.Kind != secondSegment.Kind
                            || firstSegment.Roundabout
                            || secondSegment.Roundabout
                            || firstSegment.Layer != secondSegment.Layer)
                            continue;

                        // Rail diverges both ways at a switch; roads only where both
                        // one-way ends leave, or both join.
                        var rail = firstSegment.Kind == NetworkKind.Rail;

                        if (!rail
                            && (!firstSegment.OneWay || !secondSegment.OneWay || firstArm.AtStart != secondArm.AtStart))
                            continue;

                        if (next.TryGetValue(firstArm, out var paired) && paired == secondArm)
                            continue;

                        if (math.degrees(Math.Acos(math.clamp(math.dot(Away(firstArm), Away(secondArm)), -1, 1))) >= 35)
                            continue;

                        var clearance = (firstSegment.NativeWidth + secondSegment.NativeWidth) / 2d + (rail ? 1 : 2.5);

                        // A rail branch, siding or yard track gives way to the through line.
                        // A road that continues through the junction keeps its line
                        // and the branch leaving it moves, as a link does for a
                        // mainline; only two equal branches share the push.
                        bool Branch(Arm arm, PreparedSegment segment)
                            => rail ? segment.RailService.Length > 0 || !next.ContainsKey(arm) : segment.ProtectedLink;

                        var shareA =
                            !rail && NativeAssetKeys.IsHighway(firstSegment) != NativeAssetKeys.IsHighway(secondSegment)
                                ? (NativeAssetKeys.IsHighway(firstSegment) ? 0 : 1)
                                :
                            Branch(firstArm, firstSegment) && !Branch(secondArm, secondSegment)
                                ? 1
                                : !Branch(firstArm, firstSegment) && Branch(secondArm, secondSegment) ? 0 :
                            rail || next.ContainsKey(firstArm) == next.ContainsKey(secondArm)
                                ? .5
                                : next.ContainsKey(firstArm) ? 0 : 1;

                        void Add(Arm arm, Arm partner, double share)
                        {
                            if (share <= 0)
                                return;

                            if (!result.TryGetValue(arm, out var forks))
                                result[arm] = forks = new();

                            forks.Add((Chain(partner, ForkReach + 100), clearance, share, rail ? 12 : 8));
                        }

                        Add(firstArm, secondArm, shareA);
                        Add(secondArm, firstArm, 1 - shareA);
                    }
            }

            return result;
        }

        // Push a stroke end clear of a fork partner. The separation it needs grows
        // along the fork's taper (8° for roads, 12° for a rail turnout) from the
        // junction up to the clearance (a gentle merge,
        // not an S-bend), and the push fades out over the last ForkFade of ForkReach.
        // The push is to one consistent side, the side where the end is farthest
        // from its partner: OSM often draws a ramp slightly inside the motorway's
        // centreline first, and pushing away from the nearest point would send it
        // the wrong way. Interior junctions stay put, as in Approach.
        void Clear(
            List<double2> line,
            bool atStart,
            (List<double2> Partner, double Clearance, double Share, double Taper) fork,
            HashSet<int> junctions)
        {
            var count = line.Count;

            if (count < 2 || fork.Partner.Count < 2)
                return;

            var taper = Math.Tan(math.radians(fork.Taper));

            (double Signed, double2 Normal) Offset(double2 point)
            {
                var best = double.PositiveInfinity;
                var closest = point;
                var direction = new double2(1, 0);

                for (var k = 1; k < fork.Partner.Count; k++)
                {
                    var a = fork.Partner[k - 1];
                    var ab = fork.Partner[k] - a;
                    var t = math.clamp(math.dot(point - a, ab) / Math.Max(1e-9, math.lengthsq(ab)), 0, 1);
                    var otherPoint = a + ab * t;
                    var otherDistance = math.distance(point, otherPoint);

                    if (otherDistance < best)
                    {
                        best = otherDistance;
                        closest = otherPoint;
                        direction = math.normalizesafe(ab);
                    }
                }

                var normal = new double2(-direction.y, direction.x);

                return (math.dot(point - closest, normal), normal);
            }

            var reach = new List<int>();
            var walked = 0d;
            var previous = atStart ? line[0] : line[count - 1];
            var side = 0d;
            var widest = 0d;

            for (var i = 1; i < count; i++)
            {
                var index = atStart ? i : count - 1 - i;
                walked += math.distance(previous, line[index]);
                previous = line[index];

                if (walked >= ForkReach || junctions.Contains(index))
                    break;

                reach.Add(index);
                var signed = Offset(line[index]).Signed;

                if (Math.Abs(signed) > widest)
                {
                    widest = Math.Abs(signed);
                    side = Math.Sign(signed);
                }
            }

            if (side == 0)
                return;

            walked = 0;
            previous = atStart ? line[0] : line[count - 1];

            foreach (var index in reach)
            {
                var point = line[index];
                walked += math.distance(previous, point);
                previous = point;
                var required = Math.Min(fork.Clearance, walked * taper);
                var (signedOffset, normal) = Offset(point);
                var current = signedOffset * side;

                if (current >= required)
                    continue;

                var fadeT = math.saturate((ForkReach - walked) / ForkFade);
                var fade = fadeT * fadeT * (3 - 2 * fadeT);
                line[index] = point + normal * side * (required - current) * fork.Share * fade;
                forkPushes++;
            }
        }

        // Bend a stroke end onto its required approach. In a frame along the
        // through road (along, lateral) the end leaves along the approach
        // direction until it is Clearance beside the through road, then heads
        // straight for its own line and follows it: the required lateral is the
        // upper concave hull of the node, that corner, the source line ahead
        // and the plain clearance at the end of the reach (the corner may stand
        // wider, to fit the straight start and the turn). A ramp drawn close beside
        // the motorway therefore turns out sharply and straightens, and one
        // whose real line diverges meets it along a tangent, rather than
        // running parallel at the clearance and turning out again (a hump), or
        // swinging out past its real line and back (an S). It is never moved
        // inward. The turn at the corner is a circular arc at the class's radius
        // floor up to TurnBackRadius, or the largest that fits; the meeting with the source line is
        // rounded by a smooth maximum. Points within half the reach take
        // the target, and the next two reaches blend back to the source by a
        // smoothstep, which has no tangent break at either end of the blend. The
        // reach is 40 m, or the arc that turns MergeAngle (SplitAngle) at the
        // class's radius floor (131 m for a motorway). It stops at any interior junction, whose
        // shared position must not move.
        static double ApproachReach(PreparedSegment segment)
            => Math.Max(
                40,
                VisualCompilerTable.For(segment).RadiusFloor * math.radians(Splits(segment) ? SplitAngle : MergeAngle));

        void Approach(
            List<double2> line,
            bool atStart,
            (double2 Direction, double2 Through, double Clearance, List<(double2 Point, double Width)>? Partner) approach,
            double reach,
            double radius,
            HashSet<int> junctions,
            Func<int, double> width)
        {
            var count = line.Count;

            if (count < 2)
                return;

            var node = atStart ? line[0] : line[count - 1];
            var m = approach.Through;
            var normal = math.normalizesafe(approach.Direction - m * math.dot(approach.Direction, m));
            var theta = Math.Acos(math.clamp(math.dot(approach.Direction, m), -1, 1));
            radius = Math.Min(radius, TurnBackRadius);
            var clearance = approach.Clearance;
            var corner = new double2(clearance / Math.Tan(theta), clearance);
            // The frame follows the through road's own polyline where it is
            // known (along it, and out from it on the approach side), so the
            // approach and the hold beyond it measure one offset: in a straight
            // frame from the node, a gently curving motorway put them a metre
            // apart, a bump where one handed over to the other. Needed is the
            // clearance there, half the widths side by side plus 1 m; a point
            // past the polyline's end is not alongside it.
            var partner = approach.Partner;
            var side = Math.Sign(math.dot(new double2(-m.y, m.x), normal));
            var lengths = new double[partner?.Count ?? 0];

            for (var k = 1; k < lengths.Length; k++)
                lengths[k] = lengths[k - 1] + math.distance(partner![k - 1].Point, partner[k].Point);

            (double Along, double Lateral, double2 Normal, double Needed, bool Beyond, double Distance) Frame(int index)
            {
                var point = line[index];

                if (partner == null || partner.Count < 2 || side == 0)
                    return (
                        math.dot(point - node, m),
                        math.dot(point - node, normal),
                        normal,
                        clearance,
                        true,
                        math.distance(point, node));

                var best = double.PositiveInfinity;
                var nearest = 1;
                var at = 0d;

                for (var k = 1; k < partner.Count; k++)
                {
                    var a = partner[k - 1].Point;
                    var ab = partner[k].Point - a;
                    var t = math.dot(point - a, ab) / Math.Max(1e-9, math.lengthsq(ab));
                    var otherDistance = math.distance(point, a + ab * math.clamp(t, 0, 1));

                    if (otherDistance < best)
                    {
                        best = otherDistance;
                        nearest = k;
                        at = t;
                    }
                }

                var start = partner[nearest - 1].Point;
                var segment = partner[nearest].Point - start;
                var direction = math.normalizesafe(segment);
                var n = new double2(-direction.y, direction.x) * side;
                var beyond = nearest == partner.Count - 1 && at > 1;

                if (!beyond)
                    at = math.clamp(at, 0, 1);

                return (
                    lengths[nearest - 1] + at * math.length(segment),
                    math.dot(point - (start + segment * at), n),
                    n,
                    (partner[nearest].Width + width(index)) / 2 + 1,
                    beyond,
                    best);
            }

            // The points this end may move, and the source line in the frame.
            var moved = new List<(int Index, double Walked)>();
            var source = new List<double2>();
            var walked = 0d;
            var previous = node;

            for (var i = 1; i < count - 1; i++)
            {
                var index = atStart ? i : count - 1 - i;
                var point = line[index];
                walked += math.distance(previous, point);
                previous = point;

                if (walked >= 2.5 * reach || junctions.Contains(index))
                    break;

                var frame = Frame(index);
                moved.Add((index, walked));
                source.Add(new(frame.Along, frame.Lateral));
            }

            if (moved.Count == 0)
                return;

            // Upper concave hull, by a monotone chain, of the corner, the source
            // line ahead of it while it keeps advancing along the through road
            // (a loop ramp curling back is not a line ahead), and the clearance
            // at the end. The required lateral is the lower of the approach line
            // and that hull, so it is concave and never exceeds the approach.
            var ahead = new List<double2>();
            var furthest = 0d;

            foreach (var point in source)
            {
                if (point.x < furthest - 1)
                    break;

                furthest = Math.Max(furthest, point.x);

                if (point.x > corner.x)
                    ahead.Add(point);
            }

            var candidates = new List<double2> { corner };
            candidates.AddRange(ahead);
            candidates.Add(new(Math.Max(corner.x, furthest), clearance));
            candidates.Sort((vectorA, vectorB) => vectorA.x.CompareTo(vectorB.x));
            var hull = new List<double2>();

            foreach (var point in candidates)
            {
                while (hull.Count >= 2
                    && NetworkDesignChecks.Cross(
                        hull[hull.Count - 1] - hull[hull.Count - 2],
                        point - hull[hull.Count - 2])
                        >= 0)
                    hull.RemoveAt(hull.Count - 1);

                hull.Add(point);
            }

            double Hull(double along)
            {
                if (along <= hull[0].x)
                    return hull[0].y;

                for (var k = 1; k < hull.Count; k++)
                    if (along <= hull[k].x || k == hull.Count - 1)
                    {
                        var vectorA = hull[k - 1];
                        var vectorB = hull[k];

                        return vectorB.x - vectorA.x < 1e-9
                            ? Math.Max(vectorA.y, vectorB.y)
                            : vectorA.y
                                + (vectorB.y - vectorA.y)
                                    * math.clamp((along - vectorA.x) / (vectorB.x - vectorA.x), 0, 1);
                    }

                return clearance;
            }

            // Round the corner between the approach line and the hull's first
            // edge with an arc tangent to both, at distance d either side.
            var arc = (Start: double.NaN, End: double.NaN, Centre: default(double2), Radius: 0d);
            var u1 = new double2(Math.Cos(theta), Math.Sin(theta));
            var u2 = hull.Count > 1 ? math.normalizesafe(hull[1] - hull[0]) : new double2(1, 0);

            if (u2.y / Math.Max(1e-9, u2.x) < Math.Tan(theta))
            {
                var turn = Math.Acos(math.clamp(math.dot(u1, u2), -1, 1));

                if (turn > 1e-4)
                {
                    var d =
                        Math.Min(
                            radius * Math.Tan(turn / 2),
                            Math.Min(
                                math.length(corner),
                                hull.Count > 1 ? math.distance(corner, hull[1]) : double.PositiveInfinity));

                    var r = d / Math.Tan(turn / 2);
                    var t1 = corner - u1 * d;
                    var t2 = corner + u2 * d;
                    arc = (t1.x, t2.x, t1 + new double2(u1.y, -u1.x) * r, r);
                }
            }

            double Required(double along)
            {
                if (along >= arc.Start && along <= arc.End)
                    return arc.Centre.y
                        + Math.Sqrt(
                            Math.Max(0, arc.Radius * arc.Radius - (along - arc.Centre.x) * (along - arc.Centre.x)));

                return Math.Min(along * Math.Tan(theta), Hull(along));
            }

            static double SmoothMax(double a, double b, double k)
            {
                var h = math.clamp(.5 + .5 * (a - b) / k, 0, 1);
                return math.lerp(b, a, h) + k * h * (1 - h);
            }

            // Beyond the blend the end keeps being held at the clearance while
            // its source stays inside it, up to HoldReach; a hold cut short by a
            // junction, the stroke's other end or HoldReach fades out over the
            // last HoldFade metres before it.
            // Signed lateral position describes a side, not a footprint: a
            // loop ramp can be hundreds of metres away on the opposite side.
            // Use the actual distance to the partner, including at its bends.
            bool Inside(int index)
            {
                var frame = Frame(index);
                return !frame.Beyond && frame.Distance < frame.Needed;
            }

            var hold = new List<(int Index, double Walked)>();
            var fixedAt = double.NaN;
            walked = 0;
            previous = node;

            for (var i = 1; i < count; i++)
            {
                var index = atStart ? i : count - 1 - i;
                var point = line[index];
                walked += math.distance(previous, point);
                previous = point;

                if (i == count - 1 || junctions.Contains(index) || walked >= Math.Max(2.5 * reach, HoldReach))
                {
                    fixedAt = walked;
                    break;
                }

                if (walked >= 2.5 * reach && !Inside(index))
                    break;

                hold.Add((index, walked));
            }

            if (hold.Count > 0 && !double.IsNaN(fixedAt) && !Inside(hold[^1].Index))
                fixedAt = double.NaN;

            for (var k = 0; k < hold.Count; k++)
            {
                var (index, distance) = hold[k];
                var frame = Frame(index);
                var lateral = frame.Lateral;

                // A returning through-road segment can put an already clear
                // ramp on its negative side. That half-plane is not occupied
                // pavement: do not push the ramp across the intervening land.
                if (lateral < 0 && frame.Distance >= frame.Needed)
                    continue;

                var fade = double.IsNaN(fixedAt) ? 1 : math.clamp((fixedAt - distance) / HoldFade, 0, 1);
                var held = frame.Beyond ? lateral : lateral + Math.Max(0, frame.Needed - lateral) * fade;
                var target = held;

                if (k < moved.Count)
                {
                    var required = Required(Math.Max(0, frame.Along));
                    var u = Math.Max(0, (distance / reach - .5) / 2);
                    var weight = 1 - u * u * (3 - 2 * u);
                    target = math.lerp(held, SmoothMax(lateral, required, math.clamp(required, .1, 2)), weight);
                }

                line[index] += frame.Normal * (target - lateral);
            }
        }

        // Side-by-side clearance. Two same-level strokes of one kind running within
        // 20° of each other (express and local lanes, a collector beside a
        // motorway, opposite carriageways drawn too close, parallel rail lines)
        // overlap wherever their centrelines are closer than half both native
        // widths plus 1 m. Each stroke moves away by its share of that shortfall:
        // a ramp gives way to a mainline and a siding or yard track to a main
        // line; equal routes each take half. The push rises and falls along the
        // same 8° taper as fork clearance (5° for a through road) and is zero at
        // the stroke's ends and interior junctions, so no node moves. Pairs
        // sharing a node are included rather than left to fork clearance,
        // which stops 300 m from the node; with each partner cleared to one
        // side (below), that removed 228 of DC's 1,195 overlaps.
        // The taper keeps the push at zero at the shared node. Partners are
        // measured at their source positions; each partner moves its own share.
        // Every section's source points in SideCell squares, built once.
        void EnsureSideGrid()
        {
            if (sideSamples != null)
                return;

            sideSamples = new();
            sideGrid = new();

            for (var i = 0; i < source.Count; i++)
            {
                if (contracted[i])
                    continue;

                var otherPoints = Points(i);

                for (var k = 0; k < otherPoints.Count; k++)
                {
                    var d =
                        math.normalizesafe(
                            otherPoints[Math.Min(k + 1, otherPoints.Count - 1)] - otherPoints[Math.Max(k - 1, 0)]);

                    var key =
                        ((int)Math.Floor(otherPoints[k].x / SideCell), (int)Math.Floor(otherPoints[k].y / SideCell));

                    if (!sideGrid.TryGetValue(key, out var list))
                        sideGrid[key] = list = new();

                    list.Add(sideSamples.Count);
                    sideSamples.Add((i, otherPoints[k], d));
                }
            }
        }

        // The approach, fork and side pushes each look at one partner. Packed
        // carriageways (the Dulles Toll Road, Access Road and their ramps, four
        // one-way roads within 40 m) let a ramp pushed off one partner land on a
        // third road 25 m away. Wherever a moved point ends up closer to another
        // same-level road of its kind (not one meeting this stroke) than it
        // started, and inside their clearance, its displacement shrinks until it
        // does not; the shrinking eases back in over 2.5% per metre either side
        // so the line keeps its curvature.
        void GuardPushes(
            List<double2> line,
            IReadOnlyList<double2> original,
            List<int> sectionAt,
            List<(int Section, bool Forward)> path,
            HashSet<int> junctions)
        {
            var count = line.Count;

            if (count < 3)
                return;

            EnsureSideGrid();
            var own = new HashSet<int>(path.Select(item => item.Section));
            var ownNodes =
                new HashSet<SourceNodeKey>(
                    path.SelectMany(
                        item => new[] {
                            Canon(SourceNodeKey.Start(source[item.Section])),
                            Canon(SourceNodeKey.End(source[item.Section]))
                            }));

            var factor = new double[count];
            var limited = new List<int>();

            for (var k = 0; k < count; k++)
            {
                // A move under 0.5 m cannot carry a road into another.
                factor[k] = 1;

                if (math.distancesq(line[k], original[k]) < .25)
                    continue;

                var segment = source[sectionAt[k]];
                var cx = (int)Math.Floor(line[k].x / SideCell);
                var cz = (int)Math.Floor(line[k].y / SideCell);

                double Allowed(double f)
                {
                    var point = math.lerp(original[k], line[k], f);
                    var worst = double.PositiveInfinity;

                    for (var dz = -1; dz <= 1; dz++)
                        for (var dx = -1; dx <= 1; dx++)
                            if (sideGrid!.TryGetValue((cx + dx, cz + dz), out var near))
                                foreach (var index in near)
                                {
                                    var item = sideSamples![index];

                                    if (own.Contains(item.Section))
                                        continue;

                                    var otherSegment = source[item.Section];

                                    if (otherSegment.Kind != segment.Kind
                                        || otherSegment.Layer != segment.Layer
                                        || ownNodes.Contains(Canon(SourceNodeKey.Start(otherSegment)))
                                        || ownNodes.Contains(Canon(SourceNodeKey.End(otherSegment))))
                                        continue;

                                    var clearance = (segment.NativeWidth + otherSegment.NativeWidth) / 2d + 1;
                                    var after = math.distance(point, item.P);

                                    if (after >= clearance)
                                        continue;

                                    worst =
                                        Math.Min(
                                            worst,
                                            after - Math.Min(clearance, math.distance(original[k], item.P)));
                                }

                    return worst;
                }

                if (Allowed(1) >= -.05)
                    continue;

                var low = 0d;
                var high = 1d;

                for (var step = 0; step < 8; step++)
                {
                    var middle = (low + high) / 2;

                    if (Allowed(middle) >= -.05)
                        low = middle;
                    else
                        high = middle;
                }

                factor[k] = low;
                limited.Add(k);
            }

            if (limited.Count == 0)
                return;

            var station = new double[count];

            for (var k = 1; k < count; k++)
                station[k] = station[k - 1] + math.distance(original[k - 1], original[k]);

            var eased = new double[count];

            for (var k = 0; k < count; k++)
            {
                var f = 1d;

                foreach (var j in limited)
                    f = Math.Min(f, factor[j] + Math.Abs(station[k] - station[j]) * .025);

                eased[k] = f;
            }

            for (var k = 1; k < count - 1; k++)
                if (!junctions.Contains(k) && eased[k] < 1)
                {
                    line[k] = math.lerp(original[k], line[k], eased[k]);
                    counts.GuardedPoints++;
                }
        }

        void SideClear(
            List<double2> line,
            List<int> sectionAt,
            List<(int Section, bool Forward)> path,
            HashSet<int> junctions)
        {
            // A through road that moves (an equal pair) tapers at 5°: at 8° its
            // pushes bent motorways below their radius floor.
            var first = source[path[0].Section];
            const double maximumPush = 12;
            var taper = Math.Tan(math.radians(first.ProtectedLink || first.Kind == NetworkKind.Rail ? 8 : 5));
            var parallel = Math.Cos(math.radians(20));
            EnsureSideGrid();
            var count = line.Count;

            if (count < 3)
                return;

            var own = new HashSet<int>(path.Select(item => item.Section));

            bool Minor(PreparedSegment segment)
                => segment.Kind == NetworkKind.Rail ? segment.RailService.Length > 0 : segment.ProtectedLink;

            var station = new double[count];

            for (var k = 1; k < count; k++)
                station[k] = station[k - 1] + math.distance(line[k - 1], line[k]);

            var pushes = new List<(double S, double Need, double2 Direction)>();
            // Each partner section is cleared to one side: the side where this
            // stroke stands farthest from it. A ramp drawn from inside a
            // motorway's centreline crosses it, and pushing each point to the
            // side it happened to be on bent the ramp into a zigzag.
            var sideOf = new Dictionary<int, (double Farthest, double Side)>();
            var ownNodes =
                new HashSet<SourceNodeKey>(
                    path.SelectMany(
                        item => new[] {
                            Canon(SourceNodeKey.Start(source[item.Section])),
                            Canon(SourceNodeKey.End(source[item.Section]))
                            }));

            // A road's continuation across a junction is ahead of it, not beside it. Counted
            // as alongside, it pushed straight streets about 4 m off their line some 60 m
            // either side of every crossing (report 110). A fork's arm stays on the same side
            // of the shared node, so it still counts.
            bool BeyondSharedNode(double2 point, double2 partnerPoint, PreparedSegment partner)
            {
                bool Beyond(SourceNodeKey key)
                {
                    var node = Canon(key);

                    return ownNodes.Contains(node)
                        && math.dot(point - position[node], partnerPoint - position[node]) <= 0;
                }

                return Beyond(SourceNodeKey.Start(partner)) || Beyond(SourceNodeKey.End(partner));
            }

            void Partners(
                Action<int, double2, (int Section, double2 P, double2 D), PreparedSegment, PreparedSegment> visit)
            {
                for (var k = 1; k < count - 1; k++)
                {
                    if (junctions.Contains(k))
                        continue;

                    var segment = source[sectionAt[k]];
                    var direction = math.normalizesafe(line[k + 1] - line[k - 1]);
                    var normal = new double2(-direction.y, direction.x);
                    var cx = (int)Math.Floor(line[k].x / SideCell);
                    var cz = (int)Math.Floor(line[k].y / SideCell);

                    for (var dz = -1; dz <= 1; dz++)
                        for (var dx = -1; dx <= 1; dx++)
                            if (sideGrid!.TryGetValue((cx + dx, cz + dz), out var near))
                                foreach (var index in near)
                                {
                                    var item = sideSamples![index];

                                    if (own.Contains(item.Section))
                                        continue;

                                    var otherSegment = source[item.Section];

                                    if (otherSegment.Kind != segment.Kind
                                        || otherSegment.Layer != segment.Layer
                                        || otherSegment.Roundabout
                                        || segment.Roundabout
                                        || Math.Abs(math.dot(direction, item.D)) < parallel)
                                        continue;

                                    if (math.distance(line[k], item.P)
                                        >= (segment.NativeWidth + otherSegment.NativeWidth) / 2d + 1)
                                        continue;

                                    if (BeyondSharedNode(line[k], item.P, otherSegment))
                                        continue;

                                    visit(k, normal, item, segment, otherSegment);
                                }
                }
            }

            Partners((k, normal, item, segment, otherSegment) =>
            {
                var signed = math.dot(line[k] - item.P, normal);

                if (!sideOf.TryGetValue(item.Section, out var known) || Math.Abs(signed) > known.Farthest)
                    sideOf[item.Section] = (Math.Abs(signed), signed < 0 ? -1 : 1);
            });

            var best = new (double Need, double2 Direction)[count];
            Partners((k, normal, item, segment, otherSegment) =>
            {
                // Equal road pairs keep their designed spacing: only a ramp
                // gives way, and only to a road it runs alongside. Between
                // equal roads, one branching off the other gives way to it.
                if (segment.Kind == NetworkKind.Road && !Minor(segment) && Minor(otherSegment))
                    return;

                var self = strokeOf[path[0].Section];
                var other = strokeOf[item.Section];
                var share =
                    segment.Kind == NetworkKind.Road
                        && NativeAssetKeys.IsHighway(segment) != NativeAssetKeys.IsHighway(otherSegment)
                        ? (NativeAssetKeys.IsHighway(segment) ? 0 : 1)
                        :
                    Minor(segment) && !Minor(otherSegment) ? 1 : !Minor(segment) && Minor(otherSegment) ? 0 :
                    segment.Kind != NetworkKind.Road || other < 0
                        ? .5
                        : BranchOf(other, self) ? 0 : BranchOf(self, other) ? 1 : .5;

                if (share == 0)
                    return;

                var side = sideOf[item.Section].Side;
                var current = math.dot(line[k] - item.P, normal) * side;
                var clearance = (segment.NativeWidth + otherSegment.NativeWidth) / 2d + 1;

                if (current >= clearance)
                    return;

                var need = Math.Min(maximumPush, (clearance - current) * share);

                if (need > best[k].Need)
                    best[k] = (need, normal * side);
            });

            for (var k = 1; k < count - 1; k++)
                if (best[k].Need > .05)
                    pushes.Add((station[k], best[k].Need, best[k].Direction));

            if (pushes.Count == 0)
                return;

            var anchors = new List<double> { 0, station[count - 1] };

            foreach (var j in junctions)
                if (j > 0 && j < count - 1)
                    anchors.Add(station[j]);

            double Limit(int k)
            {
                var limit = double.PositiveInfinity;

                foreach (var a in anchors)
                    limit = Math.Min(limit, Math.Abs(station[k] - a) * taper);

                return limit;
            }

            var push = new double2[count];

            // The pushes are in station order, and none needs more than maximumPush, so
            // one at maximumPush / taper or farther gives a magnitude of zero or less and
            // can never win. Those before the k being placed are skipped for good (the
            // stations only grow), and the scan stops at the first one beyond.
            var firstPush = 0;

            for (var k = 1; k < count - 1; k++)
            {
                if (junctions.Contains(k))
                    continue;

                while (firstPush < pushes.Count && (station[k] - pushes[firstPush].S) * taper >= maximumPush)
                    firstPush++;

                var magnitude = 0d;
                var direction = default(double2);

                for (var p = firstPush; p < pushes.Count; p++)
                {
                    var item = pushes[p];

                    if ((item.S - station[k]) * taper >= maximumPush)
                        break;

                    var m = item.Need - Math.Abs(station[k] - item.S) * taper;

                    if (m > magnitude)
                    {
                        magnitude = m;
                        direction = item.Direction;
                    }
                }

                push[k] = direction * Math.Min(magnitude, Limit(k));
            }

            // A triangular 40 m average rounds the taper's corners, so the moved
            // alignment needs no tight fillet; junction limits apply again after.
            const double window = 40;

            for (var k = 1; k < count - 1; k++)
            {
                if (junctions.Contains(k))
                    continue;

                var sum = default(double2);
                var weight = 0d;

                // Only points within the window of k weigh in, and the stations only grow,
                // so they are the run around k; the sum still goes in the same order.
                var low = k;

                while (low > 0 && window - Math.Abs(station[low - 1] - station[k]) > 0)
                    low--;

                var high = k;

                while (high < count - 1 && window - Math.Abs(station[high + 1] - station[k]) > 0)
                    high++;

                for (var j = low; j <= high; j++)
                {
                    var w = window - Math.Abs(station[j] - station[k]);

                    if (w > 0)
                    {
                        sum += push[j] * w;
                        weight += w;
                    }
                }

                var smooth = weight > 0 ? sum / weight : default;
                var length = math.length(smooth);
                var limit = Limit(k);

                if (length > limit)
                    smooth *= limit / length;

                if (length > .01)
                {
                    line[k] += smooth;
                    counts.SidePushes++;
                }
            }
        }

        PreparedSegment Passthrough(PreparedSegment segment)
        {
            var vectorA = Node(SourceNodeKey.Start(segment));
            var vectorB = Node(SourceNodeKey.End(segment));
            var curve = NetworkDesignChecks.Course(segment);

            float3 Lift(double3 point, float y) => new((float)point.x, y, (float)point.z);

            return segment with
            {
                Start = vectorA,
                End = vectorB,
                Control1 = Lift(curve.P1, math.lerp(vectorA.y, vectorB.y, 1f / 3)),
                Control2 = Lift(curve.P2, math.lerp(vectorA.y, vectorB.y, 2f / 3)),
                StartCurveOffset = default,
                EndCurveOffset = default,
                FinalDeckCurve = null,
                HorizontalParent = null,
                HorizontalEmissionReference = null,
                NativeUnitId = -1
            };
        }

        float3 Node(SourceNodeKey key)
        {
            var nodeKey = Canon(key);
            var point = position[nodeKey];

            return new((float)point.x, JunctionHeight(nodeKey), (float)point.y);
        }

        // Rings skip the height solve, and each node took the height of the road reaching
        // it, so a bridge arm tilted the ring: on 6 Nov 138 of Marseille's 740 rings had a
        // piece over 15%, some over 100% (report 110). Longer rings can follow the ground.
        Dictionary<SourceNodeKey, SourceNodeKey> LevelRings()
        {
            var parent = new Dictionary<SourceNodeKey, SourceNodeKey>();
            var lengths = new Dictionary<SourceNodeKey, double>();

            SourceNodeKey Find(SourceNodeKey key)
            {
                if (!parent.TryGetValue(key, out var up))
                    return parent[key] = key;

                return up == key ? key : parent[key] = Find(up);
            }

            var rings = source.Where(segment => segment.Kind == NetworkKind.Road && segment.Roundabout).ToArray();

            foreach (var segment in rings)
                parent[Find(Canon(SourceNodeKey.Start(segment)))] = Find(Canon(SourceNodeKey.End(segment)));

            foreach (var segment in rings)
            {
                var root = Find(Canon(SourceNodeKey.Start(segment)));
                lengths[root] =
                    lengths.GetValueOrDefault(root)
                        + NetworkDesignChecks.HorizontalArcLength(NetworkDesignChecks.Course(segment));
            }

            var level =
                parent.Keys
                    .ToArray()
                    .Where(key => lengths.GetValueOrDefault(Find(key)) <= MaximumLevelRing)
                    .ToDictionary(key => key, Find);

            // A railway crossing a roundabout at grade (Barcelona node 553061190, report 121
            // §19) shares one height with the ring, so the rail's node joins the ring's level.
            foreach (var node in levelCrossings)
                if (level.TryGetValue(Canon(new SourceNodeKey(NetworkKind.Road, node)), out var ring))
                    level[new SourceNodeKey(NetworkKind.Rail, node)] = ring;

            return level;
        }

        Dictionary<SourceNodeKey, double> RoundaboutWaterFloors()
        {
            var floors = new Dictionary<SourceNodeKey, double>();

            if (water == null)
                return floors;

            var seen = new HashSet<int>();
            var rings = 0;

            foreach (var seed in passthrough)
            {
                if (source[seed].Kind != NetworkKind.Road || source[seed].Tunnel || !seen.Add(seed))
                    continue;

                var ring = new List<int> { seed };
                var floor = double.NegativeInfinity;

                for (var j = 0; j < ring.Count; j++)
                {
                    token.ThrowIfCancellationRequested();
                    var segment = source[ring[j]];

                    foreach (var node in new[] { SourceNodeKey.Start(segment), SourceNodeKey.End(segment) })
                        foreach (var arm in arms[node])
                            if (source[arm.Section].Roundabout
                                && source[arm.Section].Kind == NetworkKind.Road
                                && !source[arm.Section].Tunnel
                                && seen.Add(arm.Section))
                                ring.Add(arm.Section);

                    var curve = NetworkDesignChecks.Course(segment);
                    var count = Math.Max(1, (int)Math.Ceiling(NetworkDesignChecks.HorizontalArcLength(curve) / 5));

                    for (var k = 0; k <= count; k++)
                    {
                        var t = k / (double)count;
                        var point = NetworkDesignChecks.Evaluate(curve, t);
                        var d = math.normalizesafe(NetworkDesignChecks.Derivative(curve, t).xz);
                        var normal = new double2(-d.y, d.x) * Math.Max(4, segment.NativeWidth / 2);

                        for (var side = -1; side <= 1; side++)
                        {
                            var level =
                                water(
                                    new(
                                        (float)(point.x + normal.x * side),
                                        (float)point.y,
                                        (float)(point.z + normal.y * side)));

                            if (level.HasValue && math.isfinite(level.Value))
                                floor =
                                    Math.Max(
                                        floor,
                                        level.Value
                                            + (segment.Bridge
                                                ? WaterLevels.RoadBridgeClearanceMeters
                                                : WaterLevels.RoadFreeboardMeters));
                        }
                    }
                }

                if (double.IsNegativeInfinity(floor))
                    continue;

                rings++;

                foreach (var index in ring)
                    foreach (var node in new[] {
                        Canon(SourceNodeKey.Start(source[index])),
                        Canon(SourceNodeKey.End(source[index]))
                        })
                        floors[node] = floor;
            }

            log.Diagnostic(
                $"VISUAL COMPILER roundabout water floors rings={rings}; nodes={floors.Count}; circularGeometryPreserved=true; approachFloorsCoupled=true");

            return floors;
        }

        void EmitStroke(List<(int Section, bool Forward)> path)
        {
            // Each part of the stroke is timed from the end of the previous one, so a
            // return between parts loses no time already measured.
            var lapStarted = System.Diagnostics.Stopwatch.GetTimestamp();

            long Lap()
            {
                var now = System.Diagnostics.Stopwatch.GetTimestamp();
                var elapsed = now - lapStarted;
                lapStarted = now;

                return elapsed;
            }

            // Under smooth-roads a short section whose two ends were merged into one node
            // is the inside of that junction. It is left out; the old path builds no
            // pieces for it either. The stroke stays whole, since both ends are one node.
            if (smoothRoads && source[path[0].Section].Kind == NetworkKind.Road)
            {
                bool InsideJunction(int section)
                {
                    var segment = source[section];

                    if (!Canon(SourceNodeKey.Start(segment)).Equals(Canon(SourceNodeKey.End(segment))))
                        return false;

                    var course = Points(section);
                    var length = 0d;

                    for (var j = 1; j < course.Count; j++)
                        length += math.distance(course[j - 1], course[j]);

                    return length < MergedLoop;
                }

                var outsideJunctions = new List<(int Section, bool Forward)>();

                foreach (var item in path)
                    if (!InsideJunction(item.Section))
                        outsideJunctions.Add(item);

                path = outsideJunctions;

                if (path.Count == 0)
                {
                    strokeLineTicks += Lap();
                    return;
                }
            }

            // Stroke polyline with the polyline index of every stroke node.
            var line = new List<double2>();
            var nodes = new List<SourceNodeKey>();
            var at = new List<int>();
            var sectionAt = new List<int>();

            foreach (var (section, forward) in path)
            {
                // Read backwards in place when the stroke runs against the section.
                var otherPoints = Points(section);
                var lastPoint = otherPoints.Count - 1;
                var segment = source[section];

                if (line.Count == 0)
                {
                    line.Add(otherPoints[forward ? 0 : lastPoint]);
                    sectionAt.Add(section);
                    nodes.Add(Canon(forward ? SourceNodeKey.Start(segment) : SourceNodeKey.End(segment)));
                    at.Add(0);
                }

                for (var j = 1; j <= lastPoint; j++)
                {
                    line.Add(otherPoints[forward ? j : lastPoint - j]);
                    sectionAt.Add(section);
                }

                nodes.Add(Canon(forward ? SourceNodeKey.End(segment) : SourceNodeKey.Start(segment)));
                at.Add(line.Count - 1);
            }

            if (streetLines.TryGetValue(strokeOf[path[0].Section], out var streetLine))
            {
                for (var k = 0; k < line.Count; k++)
                    line[k] = streetLine.Project(line[k]);

                for (var k = 0; k < nodes.Count; k++)
                    line[at[k]] = position[nodes[k]];
            }

            var tolerance = VisualCompilerTable.For(source[path[0].Section]);
            // Reshaping a stroke containing a bridge or tunnel also changes its
            // profile stations and approach grades. Preserve its established
            // corners until horizontal and vertical structure fitting agree.
            var optionalCornerRadius =
                path.Any(item => NetworkStructure.IsStructure(source[item.Section]))
                    ? 8
                    : Math.Max(8, tolerance.RadiusFloor);

            var smoothStreet = dryStreets.Contains(strokeOf[path[0].Section]);
            var smoothMajor = IsMainline(source[path[0].Section]);
            var interiorJunctions = new HashSet<int>();

            for (var k = 1; k < nodes.Count - 1; k++)
                if (Junction(k))
                    interiorJunctions.Add(at[k]);

            var unpushed = line.ToList();
            // Report 113, 5.2: under smooth-roads the line becomes one fitted curve on
            // 5 m samples, and from here on the line is that curve. The fit keeps its
            // own clearance from the roads placed before it, so the pushes are not run:
            // they bent the 5 m samples by metres and the fit then followed the bends.
            var smooth = smoothRoads && source[path[0].Section].Kind == NetworkKind.Road;
            var pushes = !smooth;
            strokeLineTicks += Lap();

            // Side clearance first, so an approach (which only ever moves a point
            // outward) has the last word at a merging end; there it also holds
            // the end at its clearance, so fork clearance, with its wider margin,
            // would only add a step and leaves such an end alone.
            if (pushes)
                SideClear(line, sectionAt, path, interiorJunctions);

            if (pushes && TakesApproach(source[path[0].Section])
                && approach.TryGetValue(new Arm(path[0].Section, path[0].Forward), out var startDirection))
            {
                Approach(
                    line,
                    true,
                    startDirection,
                    ApproachReach(source[path[0].Section]),
                    VisualCompilerTable.For(source[path[0].Section]).RadiusFloor,
                    interiorJunctions,
                    k => source[sectionAt[k]].NativeWidth);

                counts.Merges++;
            }

            var last = path[path.Count - 1];

            if (pushes && TakesApproach(source[last.Section])
                && approach.TryGetValue(new Arm(last.Section, !last.Forward), out var endDirection))
            {
                Approach(
                    line,
                    false,
                    endDirection,
                    ApproachReach(source[last.Section]),
                    VisualCompilerTable.For(source[last.Section]).RadiusFloor,
                    interiorJunctions,
                    k => source[sectionAt[k]].NativeWidth);

                counts.Merges++;
            }

            if (pushes && !approach.ContainsKey(new Arm(path[0].Section, path[0].Forward))
                && forks.TryGetValue(new Arm(path[0].Section, path[0].Forward), out var startForks))
                foreach (var fork in startForks)
                    Clear(line, true, fork, interiorJunctions);

            if (pushes && !approach.ContainsKey(new Arm(last.Section, !last.Forward))
                && forks.TryGetValue(new Arm(last.Section, !last.Forward), out var endForks))
                foreach (var fork in endForks)
                    Clear(line, false, fork, interiorJunctions);

            if (pushes)
                GuardPushes(line, unpushed, sectionAt, path, interiorJunctions);

            var crossingKeep = new HashSet<int>();

            void CrossingApproach(Arm arm, bool start)
            {
                if (!crossingDirections.TryGetValue(arm, out var axis))
                    return;

                var node = start ? line[0] : line[^1];
                var moved = new List<(int Index, double2 Point)>();
                var walked = 0d;
                var previous = node;
                // A short straight through the footprint, then a smooth return
                // over 80 m. Never move another junction or exceed the existing
                // class allowance merely to straighten a level crossing.
                var straight = Math.Max(12, source[arm.Section].NativeWidth);
                var reach = straight + 80;

                for (var j = 1; j < line.Count - 1; j++)
                {
                    var k = start ? j : line.Count - 1 - j;
                    var point = line[k];
                    walked += math.distance(previous, point);
                    previous = point;

                    if (walked >= reach || interiorJunctions.Contains(k))
                        break;

                    var u = math.clamp((walked - straight) / (reach - straight), 0, 1);
                    var weight = 1 - u * u * (3 - 2 * u);
                    var projected = node + axis * math.dot(point - node, axis);
                    var otherPoint = math.lerp(point, projected, weight);

                    if (math.distance(otherPoint, point) > tolerance.Deviation)
                    {
                        log.Count("Level crossing approach alignments requiring review");
                        return;
                    }

                    moved.Add((k, otherPoint));
                }

                foreach (var (k, point) in moved)
                    line[k] = point;

                if (moved.Count > 0)
                    crossingKeep.Add(moved[0].Index);
            }

            CrossingApproach(new(path[0].Section, path[0].Forward), true);
            CrossingApproach(new(last.Section, !last.Forward), false);
            strokePushTicks += Lap();

            var corners = new HashSet<int>();

            bool Junction(int k)
                => k == 0 || k == nodes.Count - 1 || arms[nodes[k]].Count >= 3 || fixedNodes.Contains(nodes[k]);

            // Report 113, 5.1: on a fitted road a port with the same road on both sides
            // needs no node in the game. It is kept as an alias on its run instead.
            bool PlainPort(int k)
            {
                if (!smooth
                    || Junction(k)
                    || arms[nodes[k]].Count != 2
                    || corners.Contains(at[k])
                    || !ports.Contains(nodes[k]) && !absorbedBy.ContainsKey(nodes[k]))
                    return false;

                var before = source[path[k - 1].Section];
                var after = source[path[k].Section];

                return before.NativeAssetKey == after.NativeAssetKey
                    && before.NativeWidth == after.NativeWidth
                    && before.Lanes == after.Lanes
                    && before.OneWay == after.OneWay
                    && (!before.OneWay || path[k - 1].Forward == path[k].Forward)
                    && before.Bridge == after.Bridge
                    && before.Tunnel == after.Tunnel
                    && before.Layer == after.Layer
                    && !NetworkBoundaryPolicy.EndpointMarker(before, !path[k - 1].Forward)
                    && !NetworkBoundaryPolicy.EndpointMarker(after, path[k].Forward);
            }

            bool Port(int k)
                => !Junction(k)
                    && (ports.Contains(nodes[k]) || absorbedBy.ContainsKey(nodes[k]))
                    && !PlainPort(k);

            double Stub(int k)
                => arms[nodes[k]].Count >= 3
                    ? Math.Max(10, .75 * arms[nodes[k]].Max(arm => source[arm.Section].NativeWidth))
                    : 0;

            if (smooth)
            {
                var fitStarted = System.Diagnostics.Stopwatch.GetTimestamp();
                FitLine();
                strokeFitTicks += System.Diagnostics.Stopwatch.GetTimestamp() - fitStarted;
            }

            // The line fit is counted on its own above, so its lap is left out.
            Lap();

            void FitLine()
            {
                var first = source[path[0].Section];
                var mainline = IsMainline(first);
                var wavelength = first.ProtectedLink
                    ? LinkWavelength
                    : mainline ? MainlineWavelength : StreetWavelength;
                var budget = first.ProtectedLink ? LinkBudget : mainline ? MainlineBudget : StreetBudget;

                if (budget == StreetBudget)
                {
                    var avenueLength = 0d;
                    var totalLength = 0d;

                    foreach (var item in path)
                    {
                        var section = source[item.Section];
                        var length = math.distance(((float3)section.CourseStart).xz, ((float3)section.CourseEnd).xz);
                        totalLength += length;

                        if (section.ConsolidatedAvenue && !section.OneWay)
                            avenueLength += length;
                    }

                    if (avenueLength >= totalLength / 2)
                        budget = AvenueBudget;
                }

                // A node stays where it is if it is fixed, if an earlier stroke placed it,
                // if it is a dead end, or if this stroke meets it twice (a loop), since
                // one node cannot be placed at two samples.
                var seen = new HashSet<SourceNodeKey>();
                var repeated = new HashSet<SourceNodeKey>();

                foreach (var node in nodes)
                    if (!seen.Add(node))
                        repeated.Add(node);

                var pinned = new bool[nodes.Count];
                var nodeAtLine = new Dictionary<int, int>();

                for (var k = 0; k < nodes.Count; k++)
                {
                    pinned[k] =
                        fixedNodes.Contains(nodes[k])
                        || placed.Contains(nodes[k])
                        || arms[nodes[k]].Count == 1
                        || repeated.Contains(nodes[k]);

                    nodeAtLine[at[k]] = k;
                }

                // A merged junction's node stands in the middle of its group, and the
                // ends of its roads were moved there, which leaves a spike in each road
                // that passes through. At a node that cannot move, the line takes this
                // road's own point instead: where its own sections ended before they were
                // moved. The node keeps its place, and the targets ease onto it
                // (EaseOntoNodes). A node this stroke is free to place stays where the
                // merge put it, and the fit moves it onto the curve.
                var nodeOffset = new double2[nodes.Count];

                for (var k = 0; k < nodes.Count; k++)
                {
                    if (!pinned[k] || repeated.Contains(nodes[k]))
                        continue;

                    var own = double2.zero;
                    var ends = 0;

                    if (k > 0)
                    {
                        var before = sourceEnds[path[k - 1].Section];
                        own += path[k - 1].Forward ? before.End : before.Start;
                        ends++;
                    }

                    if (k < path.Count)
                    {
                        var after = sourceEnds[path[k].Section];
                        own += path[k].Forward ? after.Start : after.End;
                        ends++;
                    }

                    own /= ends;
                    var offset = line[at[k]] - own;
                    var size = math.length(offset);

                    if (size < .25 || size > NodeEase / 2)
                        continue;

                    line[at[k]] = own;
                    nodeOffset[k] = offset;
                }

                var station = new double[line.Count];

                for (var j = 1; j < line.Count; j++)
                    station[j] = station[j - 1] + math.distance(line[j - 1], line[j]);

                // The turn at each point, between the directions from 2.5 m before it
                // and to 2.5 m after it, so a short jog between two samples is not a corner.
                const double look = 2.5;
                var turns = new List<(int Index, double Turn, double2 In, double2 Out)>();

                for (var j = 1; j < line.Count - 1; j++)
                {
                    var before = j - 1;

                    while (before > 0 && station[j] - station[before] < look)
                        before--;

                    var after = j + 1;

                    while (after < line.Count - 1 && station[after] - station[j] < look)
                        after++;

                    var directionIn = math.normalizesafe(line[j] - line[before]);
                    var directionOut = math.normalizesafe(line[after] - line[j]);

                    // A node that cannot move may have been placed off this line by the
                    // road that crosses it. Its turn is taken between the line before and
                    // the line after it, so that sideways step is not read as a corner.
                    if (nodeAtLine.TryGetValue(j, out var turnNode)
                        && pinned[turnNode]
                        && before > 0
                        && after < line.Count - 1)
                    {
                        directionIn = math.normalizesafe(line[before] - line[before - 1]);
                        directionOut = math.normalizesafe(line[after + 1] - line[after]);
                    }

                    var turn =
                        Math.Atan2(
                            NetworkDesignChecks.Cross(directionIn, directionOut),
                            math.dot(directionIn, directionOut));

                    if (Math.Abs(turn) >= math.radians(CornerAngle))
                        turns.Add((j, turn, directionIn, directionOut));
                }

                // One corner per 2.5 m, the sharpest. Corners within 10 m of each other
                // are a tight curve, not a corner: its points are held so the fit follows it.
                var sharp = new List<(int Index, double Turn, double2 In, double2 Out)>();

                foreach (var item in turns.OrderByDescending(item => Math.Abs(item.Turn)).ThenBy(item => item.Index))
                    if (sharp.All(other => Math.Abs(station[other.Index] - station[item.Index]) >= look))
                        sharp.Add(item);

                // A corner within JogReach of a node that cannot move is the jog left where
                // a merged junction's node went to the middle of its group. Its point is
                // dropped, so the line runs from the node to the point after the jog.
                var dropped = new HashSet<int>();

                for (var k = 0; k < nodes.Count; k++)
                {
                    if (!pinned[k])
                        continue;

                    foreach (var item in sharp)
                        if (!nodeAtLine.ContainsKey(item.Index)
                            && Math.Abs(station[item.Index] - station[at[k]]) < JogReach)
                            dropped.Add(item.Index);
                }

                sharp.RemoveAll(item => dropped.Contains(item.Index));
                sharp.Sort((a, b) => a.Index.CompareTo(b.Index));
                var heldLine = new bool[line.Count];
                var single = new List<(int Index, double Turn, double2 In, double2 Out)>();

                for (var c = 0; c < sharp.Count; c++)
                {
                    var crowded =
                        c > 0 && station[sharp[c].Index] - station[sharp[c - 1].Index] < 10
                        || c + 1 < sharp.Count && station[sharp[c + 1].Index] - station[sharp[c].Index] < 10;

                    if (crowded)
                        heldLine[sharp[c].Index] = true;
                    else
                        single.Add(sharp[c]);
                }

                // A corner keeps its fillet where the fillet is a curve (radius of
                // CornerRadius or more within the class deviation and half the way to
                // the next node or corner); a tighter corner, or one at a node that
                // cannot move, is a break in the curve.
                var features = nodeAtLine.Keys
                    .Concat(single.Select(item => item.Index))
                    .Distinct()
                    .OrderBy(j => j)
                    .ToArray();
                var arcs = new List<(int Index, double Tangent, double Radius, double Turn, double2 In, double2 Out)>();
                var breakLines = new HashSet<int>();

                foreach (var item in single)
                {
                    var half = Math.Abs(item.Turn) / 2;
                    var f = Array.BinarySearch(features, item.Index);
                    var room =
                        Math.Min(
                            station[item.Index] - station[features[Math.Max(0, f - 1)]],
                            station[features[Math.Min(features.Length - 1, f + 1)]] - station[item.Index]) / 2;

                    var radius =
                        Math.Min(
                            tolerance.RadiusTarget,
                            Math.Min(room / Math.Tan(half), tolerance.Deviation / (1 / Math.Cos(half) - 1)));

                    if (nodeAtLine.TryGetValue(item.Index, out var cornerNode) && pinned[cornerNode]
                        || radius < CornerRadius
                        || half > math.radians(85))
                        breakLines.Add(item.Index);
                    else
                        arcs.Add((item.Index, radius * Math.Tan(half), radius, item.Turn, item.In, item.Out));
                }

                // The line with every kept fillet drawn in place of its corner.
                var shaped = new List<double2>(line.Count);
                var shapedSection = new List<int>(line.Count);
                var held = new List<bool>(line.Count);
                var nodeAtShaped = new int[nodes.Count];
                var breakShaped = new List<int>();
                var nextArc = 0;

                for (var j = 0; j < line.Count; j++)
                {
                    if (dropped.Contains(j))
                        continue;

                    if (nextArc < arcs.Count && station[j] > station[arcs[nextArc].Index] - arcs[nextArc].Tangent)
                    {
                        var arc = arcs[nextArc++];
                        var corner = line[arc.Index];
                        var start = corner - arc.In * arc.Tangent;
                        var side = arc.Turn > 0 ? 1 : -1;
                        var center = start + new double2(-arc.In.y, arc.In.x) * side * arc.Radius;
                        var angle = Math.Atan2(start.y - center.y, start.x - center.x);
                        var parts = 2 * Math.Max(1, (int)Math.Ceiling(arc.Radius * Math.Abs(arc.Turn) / 5));

                        if (nodeAtLine.TryGetValue(arc.Index, out var arcNode))
                            nodeAtShaped[arcNode] = shaped.Count + parts / 2;

                        for (var q = 0; q <= parts; q++)
                        {
                            var a = angle + arc.Turn * q / parts;
                            shaped.Add(center + arc.Radius * new double2(Math.Cos(a), Math.Sin(a)));
                            shapedSection.Add(sectionAt[arc.Index]);
                            held.Add(true);
                        }

                        while (j + 1 < line.Count && station[j + 1] < station[arc.Index] + arc.Tangent)
                            j++;

                        continue;
                    }

                    if (nodeAtLine.TryGetValue(j, out var k))
                        nodeAtShaped[k] = shaped.Count;

                    if (breakLines.Contains(j))
                        breakShaped.Add(shaped.Count);

                    shaped.Add(line[j]);
                    shapedSection.Add(sectionAt[j]);
                    held.Add(heldLine[j]);
                }

                var anchors = nodeAtShaped.Concat(breakShaped).Distinct().OrderBy(j => j).ToArray();
                var samples = new List<double2>();
                var origin = new List<int>();
                var anchorAt = RoadAlignment.Resample(shaped, anchors, samples, origin);
                var sampleOf = new Dictionary<int, int>();

                for (var a = 0; a < anchors.Length; a++)
                    sampleOf[anchors[a]] = anchorAt[a];

                var weights = new double[samples.Count];

                for (var i = 0; i < weights.Length; i++)
                    weights[i] = held[origin[i]] && held[Math.Max(0, origin[i] - 1)] ? RoadAlignment.HoldWeight : 1;

                // The samples beside a held shape are held less and less firmly, so the
                // curve does not swing wide on its way into a tight fillet.
                for (var i = 0; i < weights.Length; i++)
                {
                    if (weights[i] != RoadAlignment.HoldWeight)
                        continue;

                    var eased = RoadAlignment.HoldWeight;

                    for (var d = 1; d <= 3; d++)
                    {
                        eased /= 10;

                        if (i - d >= 0 && weights[i - d] < eased)
                            weights[i - d] = eased;

                        if (i + d < weights.Length && weights[i + d] < eased)
                            weights[i + d] = eased;
                    }
                }

                for (var k = 0; k < nodes.Count; k++)
                    if (pinned[k])
                        weights[sampleOf[nodeAtShaped[k]]] = RoadAlignment.PinWeight;

                var breakList = new List<int>();

                foreach (var j in breakShaped)
                    if (sampleOf[j] > 0 && sampleOf[j] < samples.Count - 1)
                        breakList.Add(sampleOf[j]);

                var breaks = breakList.ToArray();

                foreach (var i in breaks)
                    weights[i] = RoadAlignment.PinWeight;

                EaseOntoNodes();
                HoldThrough(0, 0, 1);
                HoldThrough(nodes.Count - 1, samples.Count - 1, samples.Count - 2);

                // The line was drawn through this road's own point at every node. A node
                // that cannot move may stand metres off it. The targets move sideways by
                // each such node's distance from the road's own point, easing to nothing
                // over NodeEase metres or up to the next node that cannot move or corner,
                // so the curve reaches the node in one gentle shift, not a step beside it.
                void EaseOntoNodes()
                {
                    var offsetAt = new SortedDictionary<int, double2>();

                    foreach (var i in breaks)
                        offsetAt[i] = double2.zero;

                    var any = false;

                    for (var k = 0; k < nodes.Count; k++)
                    {
                        if (!pinned[k])
                            continue;

                        var i = sampleOf[nodeAtShaped[k]];
                        offsetAt[i] = nodeOffset[k];
                        any |= !nodeOffset[k].Equals(double2.zero);
                    }

                    if (!any)
                        return;

                    var sampleStation = new double[samples.Count];

                    for (var i = 1; i < samples.Count; i++)
                        sampleStation[i] = sampleStation[i - 1] + math.distance(samples[i - 1], samples[i]);

                    var anchorSamples = offsetAt.Keys.ToArray();
                    var next = 0;

                    for (var i = 0; i < samples.Count; i++)
                    {
                        while (next < anchorSamples.Length && anchorSamples[next] < i)
                            next++;

                        var shift = double2.zero;

                        if (next < anchorSamples.Length)
                            shift += Eased(anchorSamples[next], next > 0 ? anchorSamples[next - 1] : -1);

                        if (next > 0 && (next == anchorSamples.Length || anchorSamples[next] != i))
                            shift += Eased(anchorSamples[next - 1], next < anchorSamples.Length ? anchorSamples[next] : -1);

                        // The share of one node's offset at sample i; other is the
                        // node or corner on the far side of i, if there is one.
                        double2 Eased(int anchor, int other)
                        {
                            var reach = NodeEase;

                            if (other >= 0)
                                reach = Math.Min(reach, Math.Abs(sampleStation[other] - sampleStation[anchor]));

                            var share = 1 - Math.Abs(sampleStation[i] - sampleStation[anchor]) / Math.Max(1e-9, reach);

                            return offsetAt[anchor] * Math.Max(0, share);
                        }

                        samples[i] += shift;
                    }

                    log.Count("Smooth roads eased onto a node off their line");
                }

                // One alignment through a node (report 113, 5.2). Where this stroke ends at
                // a node already placed and an arm fitted earlier leaves that node within
                // ThroughAngle of straight ahead, the sample next to the node is held on
                // that arm's line, so the two curves meet there without a kink.
                void HoldThrough(int strokeNode, int nodeSample, int nextSample)
                {
                    if (!pinned[strokeNode] || samples.Count < 4 || weights[nextSample] >= RoadAlignment.PinWeight)
                        return;

                    var step = math.distance(samples[nodeSample], samples[nextSample]);
                    var ahead = math.normalizesafe(samples[nextSample] - samples[nodeSample]);

                    if (ringCentres.TryGetValue(nodes[strokeNode], out var ringCentre))
                    {
                        var radial = math.normalizesafe(samples[nodeSample] - ringCentre);

                        if (!first.Roundabout && math.dot(ahead, radial) >= Math.Cos(math.radians(RingArmAngle)))
                        {
                            samples[nextSample] = samples[nodeSample] + radial * step;
                            weights[nextSample] = Math.Max(weights[nextSample], RoadAlignment.HoldWeight);
                        }

                        return;
                    }
                    if (!first.OneWay && !first.ProtectedLink && !mainline && MeetSquarely())
                        return;

                    // Report 113, 7.5: a side road meets a road squarely. Where this
                    // street ends within AcuteAngle of an arm already fitted at the node,
                    // the game would draw a thin sliver between the two. Its last
                    // SquareReach metres turn away from that arm, by the full shortfall
                    // to SquareAngle at the node and by nothing at the far end. One-way
                    // roads, links and highways are left alone: they merge at a slant.
                    bool MeetSquarely()
                    {
                        var closest = Math.Cos(math.radians(AcuteAngle));
                        var nearest = double2.zero;

                        foreach (var arm in arms[nodes[strokeNode]])
                        {
                            if (!placedArms.TryGetValue((arm.Section, arm.AtStart), out var armDirection))
                                continue;

                            var along = math.dot(armDirection, ahead);

                            if (along > closest)
                            {
                                closest = along;
                                nearest = armDirection;
                            }
                        }

                        if (nearest.Equals(double2.zero))
                            return false;

                        var side = NetworkDesignChecks.Cross(nearest, ahead) < 0 ? -1 : 1;
                        var turn = side * (math.radians(SquareAngle) - Math.Acos(Math.Min(1, closest)));
                        var turned = Turned(ahead, turn);

                        // Turning must not bring the street as near to another arm.
                        foreach (var arm in arms[nodes[strokeNode]])
                            if (placedArms.TryGetValue((arm.Section, arm.AtStart), out var armDirection)
                                && !armDirection.Equals(nearest)
                                && math.dot(armDirection, turned) > Math.Cos(math.radians(AcuteAngle)))
                                return false;

                        var node = samples[nodeSample];
                        var direction = nextSample - nodeSample;
                        var previous = node;
                        var walked = 0d;

                        for (var i = nextSample; i >= 0 && i < samples.Count; i += direction)
                        {
                            walked += math.distance(previous, samples[i]);
                            previous = samples[i];

                            if (walked >= SquareReach || weights[i] >= RoadAlignment.PinWeight)
                                break;

                            samples[i] = node + Turned(samples[i] - node, turn * (1 - walked / SquareReach));
                        }

                        weights[nextSample] = Math.Max(weights[nextSample], RoadAlignment.HoldWeight);
                        log.Count("Smooth roads side roads turned to meet a road squarely");
                        return true;
                    }

                    double2 Turned(double2 vector, double angle)
                    {
                        var cos = Math.Cos(angle);
                        var sin = Math.Sin(angle);

                        return new double2(vector.x * cos - vector.y * sin, vector.x * sin + vector.y * cos);
                    }
                    var best = -Math.Cos(math.radians(ThroughAngle));
                    var through = double2.zero;

                    foreach (var arm in arms[nodes[strokeNode]])
                    {
                        if (!placedArms.TryGetValue((arm.Section, arm.AtStart), out var armDirection))
                            continue;

                        var along = math.dot(armDirection, ahead);

                        if (along < best)
                        {
                            best = along;
                            through = armDirection;
                        }
                    }

                    if (through.Equals(double2.zero))
                        return;

                    samples[nextSample] = samples[nodeSample] - through * step;
                    weights[nextSample] = Math.Max(weights[nextSample], RoadAlignment.HoldWeight);
                }

                var fit = alignment.Fit(samples, weights, breaks, wavelength, budget);
                if (RunBeside())
                {
                    fit = alignment.Fit(samples, weights, breaks, wavelength, budget);
                }

                // Report 113, 7.1: the second carriageway of a divided road follows the
                // first. Where this one-way road runs against an earlier one-way road of
                // the same name or reference, within PairReach, its targets are set at a
                // steady distance from that road's fitted curve: the median of the gap
                // the map draws over PairWindow either side. The gap then changes only
                // where the map's median really widens, and the two run parallel. The
                // earlier road never moves. True when any target moved.
                bool RunBeside()
                {
                    var count = samples.Count;

                    if (count < 3 || !first.OneWay || first.ProtectedLink || first.Roundabout)
                        return false;

                    var fitStation = new double[count];

                    for (var i = 1; i < count; i++)
                        fitStation[i] = fitStation[i - 1] + math.distance(fit[i - 1], fit[i]);

                    var opposite = -Math.Cos(math.radians(20));
                    var partner = new int[count];
                    var gap = new double[count];
                    var foot = new double2[count];
                    var across = new double2[count];

                    for (var i = 0; i < count; i++)
                    {
                        partner[i] = -1;

                        if (i == 0 || i == count - 1)
                            continue;

                        var segment = source[shapedSection[origin[i]]];
                        var direction = math.normalizesafe(fit[i + 1] - fit[i - 1]);
                        var nearest = PairReach;
                        var cx = (int)Math.Floor(fit[i].x / SideCell);
                        var cz = (int)Math.Floor(fit[i].y / SideCell);

                        for (var dz = -1; dz <= 1; dz++)
                            for (var dx = -1; dx <= 1; dx++)
                            {
                                if (!placedGrid.TryGetValue((cx + dx, cz + dz), out var near))
                                    continue;

                                foreach (var index in near)
                                {
                                    var item = placedSamples[index];
                                    var otherSegment = source[item.Section];
                                    var distance = math.distance(fit[i], item.P);

                                    if (distance >= nearest
                                        || math.dot(direction, item.D) > opposite
                                        || !otherSegment.OneWay
                                        || otherSegment.ProtectedLink
                                        || otherSegment.Roundabout
                                        || otherSegment.Layer != segment.Layer
                                        || !SameRoad(segment, otherSegment))
                                        continue;

                                    nearest = distance;
                                    partner[i] = item.Stroke;
                                    across[i] = new double2(-item.D.y, item.D.x);
                                    foot[i] = item.P + item.D * math.dot(fit[i] - item.P, item.D);
                                    gap[i] = math.dot(fit[i] - item.P, across[i]);
                                }
                            }
                    }

                    var moved = false;
                    var from = 0;

                    while (from < count)
                    {
                        var to = from;

                        while (to + 1 < count && partner[to + 1] == partner[from])
                            to++;

                        if (partner[from] >= 0 && fitStation[to] - fitStation[from] >= PairLength)
                        {
                            for (var i = from; i <= to; i++)
                            {
                                // A node that cannot move keeps its place.
                                if (weights[i] >= RoadAlignment.PinWeight)
                                    continue;

                                var low = i;
                                var high = i;

                                while (low > from && fitStation[i] - fitStation[low - 1] <= PairWindow)
                                    low--;

                                while (high < to && fitStation[high + 1] - fitStation[i] <= PairWindow)
                                    high++;

                                var window = new double[high - low + 1];
                                Array.Copy(gap, low, window, 0, window.Length);
                                Array.Sort(window);

                                var share =
                                    Math.Min(
                                        1,
                                        Math.Min(fitStation[i] - fitStation[from], fitStation[to] - fitStation[i])
                                            / PairEase);

                                share = share * share * (3 - 2 * share);
                                var target = foot[i] + across[i] * window[window.Length / 2];
                                samples[i] = math.lerp(fit[i], target, share);

                                if (share >= 1)
                                    weights[i] = Math.Max(weights[i], ClearanceWeight);
                            }

                            moved = true;
                            log.Count("Smooth roads carriageways set beside their opposite");
                        }

                        from = to + 1;
                    }

                    return moved;

                    bool SameRoad(PreparedSegment segment, PreparedSegment otherSegment)
                        => NativeAssetKeys.IsHighway(segment) == NativeAssetKeys.IsHighway(otherSegment)
                            && (segment.Name.Length > 0 && segment.Name == otherSegment.Name
                                || segment.Reference.Length > 0 && segment.Reference == otherSegment.Reference
                                || segment.PairedCorridorId != 0
                                    && segment.PairedCorridorId == otherSegment.PairedCorridorId);
                }

                if (KeepClear())
                {
                    fit = alignment.Fit(samples, weights, breaks, wavelength, budget);
                }

                // Report 113, 5.3: clearance from the roads fitted before this one. Where
                // the curve runs alongside one of them, closer than both half widths plus
                // FitClearance, its targets move sideways by the shortfall and are held
                // there; the move eases off at ClearanceTaper either side. The need fades
                // to nothing toward a node both roads share, where they must meet. The
                // earlier road never moves. True when any target moved.
                bool KeepClear()
                {
                    var count = samples.Count;

                    if (count < 3)
                        return false;

                    var fitStation = new double[count];

                    for (var i = 1; i < count; i++)
                        fitStation[i] = fitStation[i - 1] + math.distance(fit[i - 1], fit[i]);

                    var parallel = Math.Cos(math.radians(20));
                    var normals = new double2[count];
                    var sharedStations = new Dictionary<int, double[]>();
                    var sideSum = new Dictionary<int, double>();
                    var found = new List<(int Sample, int Stroke, double Shortfall)>();

                    for (var i = 1; i < count - 1; i++)
                    {
                        var direction = math.normalizesafe(fit[i + 1] - fit[i - 1]);
                        normals[i] = new double2(-direction.y, direction.x);

                        if (weights[i] >= RoadAlignment.PinWeight)
                            continue;

                        var segment = source[shapedSection[origin[i]]];

                        if (segment.Roundabout)
                            continue;

                        var cx = (int)Math.Floor(fit[i].x / SideCell);
                        var cz = (int)Math.Floor(fit[i].y / SideCell);

                        for (var dz = -1; dz <= 1; dz++)
                            for (var dx = -1; dx <= 1; dx++)
                            {
                                if (!placedGrid.TryGetValue((cx + dx, cz + dz), out var near))
                                    continue;

                                foreach (var index in near)
                                {
                                    var item = placedSamples[index];
                                    var otherSegment = source[item.Section];

                                    if (otherSegment.Layer != segment.Layer
                                        || otherSegment.Roundabout
                                        || Math.Abs(math.dot(direction, item.D)) < parallel)
                                        continue;

                                    var need = (segment.NativeWidth + otherSegment.NativeWidth) / 2d + FitClearance;
                                    var distance = math.distance(fit[i], item.P);

                                    if (distance >= need)
                                        continue;

                                    if (!sharedStations.TryGetValue(item.Stroke, out var shared))
                                    {
                                        var stationList = new List<double>();

                                        for (var k = 0; k < nodes.Count; k++)
                                            if (placedNodes[item.Stroke].Contains(nodes[k]))
                                                stationList.Add(fitStation[sampleOf[nodeAtShaped[k]]]);

                                        shared = stationList.ToArray();
                                        sharedStations[item.Stroke] = shared;
                                    }

                                    foreach (var s in shared)
                                        need = Math.Min(need, Math.Abs(fitStation[i] - s) * ClearanceFade);

                                    if (distance >= need)
                                        continue;

                                    sideSum.TryGetValue(item.Stroke, out var sum);
                                    sideSum[item.Stroke] = sum + math.dot(fit[i] - item.P, normals[i]);
                                    found.Add((i, item.Stroke, need - distance));
                                }
                            }
                    }

                    if (found.Count == 0)
                        return false;

                    // Each earlier road is cleared to one side, the side most of this
                    // curve is already on, so a curve that crosses it is not bent both ways.
                    // The needs to either side (along or against the normal) are kept
                    // apart, eased off at ClearanceTaper, and the move is their
                    // difference. A street between the two carriageways of a viaduct
                    // mapped at its level is pushed both ways, and the pushes cancel;
                    // when the larger push won sample by sample, the side alternated
                    // and the street was drawn as a staircase (Dhaka, report 113).
                    var shortfall = new double[count];
                    var alongNeed = new double[count];
                    var againstNeed = new double[count];

                    var limited = false;

                    foreach (var (i, stroke, amount) in found)
                    {
                        limited |= amount > ClearanceLimit;

                        var held = Math.Min(amount, ClearanceLimit);
                        var need = sideSum[stroke] < 0 ? againstNeed : alongNeed;
                        need[i] = Math.Max(need[i], held);
                        shortfall[i] = Math.Max(shortfall[i], held);
                    }

                    if (limited)
                        log.Count("Smooth roads moves for clearance held to the limit");

                    var along = Eased(alongNeed);
                    var against = Eased(againstNeed);

                    double[] Eased(double[] need)
                    {
                        var eased = new double[count];

                        for (var j = 1; j < count - 1; j++)
                        {
                            if (need[j] <= 0)
                                continue;

                            for (var i = j; i >= 1; i--)
                            {
                                var value = need[j] - (fitStation[j] - fitStation[i]) * ClearanceTaper;

                                if (value <= 0)
                                    break;

                                eased[i] = Math.Max(eased[i], value);
                            }

                            for (var i = j + 1; i < count - 1; i++)
                            {
                                var value = need[j] - (fitStation[i] - fitStation[j]) * ClearanceTaper;

                                if (value <= 0)
                                    break;

                                eased[i] = Math.Max(eased[i], value);
                            }
                        }

                        return eased;
                    }

                    // A sample that cannot move (a pinned node, or either end of the
                    // curve) holds the move at zero there, and the move eases away from
                    // it at ClearanceTaper too, instead of bending in and out sharply at
                    // each pinned junction.
                    var room = new double[count];

                    for (var i = 0; i < count; i++)
                        room[i] =
                            i == 0 || i == count - 1 || weights[i] >= RoadAlignment.PinWeight ? 0 : double.MaxValue;

                    for (var i = 1; i < count; i++)
                        room[i] = Math.Min(room[i], room[i - 1] + (fitStation[i] - fitStation[i - 1]) * ClearanceTaper);

                    for (var i = count - 2; i >= 0; i--)
                        room[i] = Math.Min(room[i], room[i + 1] + (fitStation[i + 1] - fitStation[i]) * ClearanceTaper);

                    for (var i = 1; i < count - 1; i++)
                    {
                        var move = along[i] - against[i];
                        var amount = Math.Min(Math.Abs(move), room[i]);

                        if (amount <= 0)
                            continue;

                        samples[i] = fit[i] + normals[i] * Math.Sign(move) * amount;

                        if (shortfall[i] > 0)
                            weights[i] = Math.Max(weights[i], ClearanceWeight);
                    }

                    log.Count("Smooth roads moved clear of an earlier road");
                    return true;
                }

                SetStraight();

                // Report 113, 7.3: exactly straight where the map is straight. A stretch
                // of the curve that stays within StraightReach of its own chord for
                // StraightLength or more is moved onto straight lines between its ends
                // and the nodes inside it that cannot move. Those nodes keep their place,
                // so the stretch is left alone where one would turn the line too far.
                void SetStraight()
                {
                    var count = fit.Length;

                    if (count < 3 || first.Roundabout)
                        return;

                    var fitStation = new double[count];

                    for (var i = 1; i < count; i++)
                        fitStation[i] = fitStation[i - 1] + math.distance(fit[i - 1], fit[i]);

                    if (fitStation[count - 1] < StraightLength)
                        return;

                    var a = 0;

                    while (a < count - 2)
                    {
                        var b = a + 1;

                        while (b + 1 < count && NearChord(a, b + 1))
                            b++;

                        if (fitStation[b] - fitStation[a] < StraightLength)
                        {
                            a++;
                            continue;
                        }

                        Straighten(a, b);
                        a = b;
                    }

                    bool NearChord(int from, int to)
                    {
                        var along = math.normalizesafe(fit[to] - fit[from]);

                        for (var i = from + 1; i < to; i++)
                            if (Math.Abs(NetworkDesignChecks.Cross(along, fit[i] - fit[from])) > StraightReach)
                                return false;

                        return true;
                    }

                    void Straighten(int from, int to)
                    {
                        var stops = new List<int> { from };

                        for (var i = from + 1; i < to; i++)
                            if (weights[i] >= RoadAlignment.PinWeight)
                                stops.Add(i);

                        stops.Add(to);

                        for (var s = 1; s < stops.Count - 1; s++)
                        {
                            var before = math.normalizesafe(fit[stops[s]] - fit[stops[s - 1]]);
                            var after = math.normalizesafe(fit[stops[s + 1]] - fit[stops[s]]);

                            if (math.dot(before, after) < Math.Cos(math.radians(StraightTurn)))
                            {
                                log.Count("Smooth roads straight stretches left for a node off the line");
                                return;
                            }
                        }

                        var easeIn = from > 0 && weights[from] < RoadAlignment.PinWeight;
                        var easeOut = to < count - 1 && weights[to] < RoadAlignment.PinWeight;
                        var ends = new double2[stops.Count];

                        for (var s = 0; s < stops.Count; s++)
                            ends[s] = fit[stops[s]];

                        for (var s = 0; s + 1 < stops.Count; s++)
                        {
                            var span = Math.Max(1e-9, fitStation[stops[s + 1]] - fitStation[stops[s]]);

                            for (var i = stops[s] + 1; i < stops[s + 1]; i++)
                            {
                                var target = math.lerp(ends[s], ends[s + 1], (fitStation[i] - fitStation[stops[s]]) / span);
                                var share = 1d;

                                if (easeIn)
                                    share = Math.Min(share, (fitStation[i] - fitStation[from]) / StraightEase);

                                if (easeOut)
                                    share = Math.Min(share, (fitStation[to] - fitStation[i]) / StraightEase);

                                share = share * share * (3 - 2 * share);
                                fit[i] = math.lerp(fit[i], target, share);
                            }
                        }

                        log.Count("Smooth roads stretches set exactly straight");
                    }
                }

                // This road's curve, for the clearance of the roads fitted after it.
                var strokeIndex = placedNodes.Count;
                placedNodes.Add(new HashSet<SourceNodeKey>(nodes));

                for (var i = 0; i < fit.Length; i++)
                {
                    var key = ((int)Math.Floor(fit[i].x / SideCell), (int)Math.Floor(fit[i].y / SideCell));

                    if (!placedGrid.TryGetValue(key, out var list))
                        placedGrid[key] = list = new();

                    list.Add(placedSamples.Count);

                    placedSamples.Add((
                        strokeIndex,
                        shapedSection[origin[i]],
                        fit[i],
                        math.normalizesafe(fit[Math.Min(i + 1, fit.Length - 1)] - fit[Math.Max(i - 1, 0)])));
                }

                // The direction in which each arm leaves each node of this stroke.
                for (var k = 0; k < nodes.Count; k++)
                {
                    var nodeSample = sampleOf[nodeAtShaped[k]];

                    if (k > 0 && nodeSample > 0)
                        placedArms[(path[k - 1].Section, !path[k - 1].Forward)] =
                            math.normalizesafe(fit[nodeSample - 1] - fit[nodeSample]);

                    if (k < path.Count && nodeSample + 1 < fit.Length)
                        placedArms[(path[k].Section, path[k].Forward)] =
                            math.normalizesafe(fit[nodeSample + 1] - fit[nodeSample]);
                }

                line.Clear();
                line.AddRange(fit);
                sectionAt.Clear();

                foreach (var j in origin)
                    sectionAt.Add(shapedSection[j]);

                corners.UnionWith(breaks);
                crossingKeep.Clear();

                // The fit places every node it was free to move. Sections of other
                // strokes that were already read end at the node's new place.
                for (var k = 0; k < nodes.Count; k++)
                {
                    at[k] = sampleOf[nodeAtShaped[k]];

                    if (!placed.Add(nodes[k]) || pinned[k])
                        continue;

                    position[nodes[k]] = line[at[k]];

                    foreach (var arm in arms[nodes[k]])
                    {
                        var cached = points[arm.Section];

                        if (cached != null)
                            cached[arm.AtStart ? 0 : cached.Count - 1] = line[at[k]];
                    }
                }
            }

            // A port that the alignment cannot pass near is pinned like a junction
            // on a second build, so no topology port is ever dropped.
            var forced = new HashSet<int>();
            var lastResort = false;
            var built = Build();

            for (var attempt = 0; built.Missed.Count > 0; attempt++)
            {
                if (attempt == 3)
                    lastResort = true;

                forced.UnionWith(built.Missed);
                built = Build();

                if (lastResort)
                    break;
            }

            var (pieces, allCuts, _) = built;
            strokePieceTicks += Lap();

            if (pieces.Count == 0)
                return;

            counts.Pieces += pieces.Count;
            counts.ProjectionMiss += forced.Count;
            var stations = Stations(pieces);
            var freeze = FrozenFirst(path) || smooth;

            // Station of every stroke node on the new alignment, searched forward;
            // a cubic's provenance is every source section its interval overlaps.
            var nodeStation = new double[nodes.Count];
            var cursor = 0;
            var cutStation = new Dictionary<int, double>();

            // The first cut at each stroke node gives its station.
            foreach (var cut in allCuts)
                if (cut.StrokeNode >= 0 && !cutStation.ContainsKey(cut.StrokeNode))
                    cutStation[cut.StrokeNode] = stations[cut.Piece + 1];

            for (var k = 0; k < nodes.Count; k++)
            {
                if (k == 0 || cutStation.ContainsKey(k))
                {
                    nodeStation[k] = k == 0 ? 0 : cutStation[k];

                    if (k > 0)
                        nodeStation[k] = Math.Max(nodeStation[k], nodeStation[k - 1]);

                    var s = Array.BinarySearch(stations, nodeStation[k]);
                    cursor = Math.Max(0, Math.Min(pieces.Count - 1, s < 0 ? ~s - 1 : s));
                    continue;
                }

                var target = k == 0 ? pieces[0].P0 : k == nodes.Count - 1 ? pieces[pieces.Count - 1].P3 : line[at[k]];
                var best = (Piece: cursor, T: 0d, Distance: double.PositiveInfinity);

                for (var p = cursor; p < pieces.Count; p++)
                {
                    for (var q = 0; q <= 16; q++)
                    {
                        var otherDistance = math.distance(Evaluate(pieces[p], q / 16d), target);

                        if (otherDistance < best.Distance)
                            best = (p, q / 16d, otherDistance);
                    }

                    if (stations[p] - stations[best.Piece + 1] > 200)
                        break;
                }

                cursor = best.Piece;
                nodeStation[k] = stations[best.Piece] + best.T * (stations[best.Piece + 1] - stations[best.Piece]);

                if (k > 0)
                    nodeStation[k] = Math.Max(nodeStation[k], nodeStation[k - 1]);
            }

            if (freeze)
                for (var k = 0; k < path.Count; k++)
                {
                    var frozen = new List<double2>();
                    var count =
                        Math.Max(
                            1,
                            (int)Math.Ceiling((nodeStation[k + 1] - nodeStation[k]) / HighwayAlignment.Spacing));

                    for (var q = 0; q <= count; q++)
                        frozen.Add(
                            At(pieces, stations, math.lerp(nodeStation[k], nodeStation[k + 1], q / (double)count)));

                    if (!path[k].Forward)
                        frozen.Reverse();

                    points[path[k].Section] = frozen;
                }

            strokeStationTicks += Lap();

            List<int> Covered(double s0, double s1)
            {
                var list = new List<int>();
                var middle = (s0 + s1) / 2;
                var nearest = 0;

                for (var k = 0; k < path.Count; k++)
                {
                    if (Math.Min(s1, nodeStation[k + 1]) - Math.Max(s0, nodeStation[k]) > .5)
                        list.Add(k);

                    if (nodeStation[k] <= middle)
                        nearest = k;
                }

                if (list.Count == 0)
                    list.Add(nearest);

                return list;
            }

            // Runs between cuts. Provenance comes from the stroke nodes bounding the
            // run, so every source section between two kept nodes is represented once.
            var runs =
                new List<(int First, int Last, long StartNode, long EndNode, List<int> Path, PreparedSegment Template)>();

            var startPiece = 0;
            var startNode = nodes[0].Id;
            var startStroke = 0;

            foreach (var cut in allCuts)
            {
                var run = startStroke >= 0 && cut.StrokeNode >= 0 && cut.StrokeNode != startStroke
                    ? Enumerable
                        .Range(Math.Min(startStroke, cut.StrokeNode), Math.Abs(cut.StrokeNode - startStroke))
                        .ToList()
                    : Covered(stations[startPiece], stations[cut.Piece + 1]);

                var template =
                    source[path[run
                        .OrderByDescending(
                            k => NetworkDesignChecks.HorizontalArcLength(
                                NetworkDesignChecks.Course(source[path[k].Section])))
                        .First()].Section];

                runs.Add((startPiece, cut.Piece, startNode, cut.Node, run, template));
                startPiece = cut.Piece + 1;
                startNode = cut.Node;
                startStroke = cut.StrokeNode;
            }

            // Filling a bridge gap removes its old structure boundary, not its
            // source identity. A former port absorbed inside one emitted run
            // aliases that run's start; no extra node or short edge is needed.
            var cutNodes = new HashSet<long>();

            foreach (var cut in allCuts)
                cutNodes.Add(cut.Node);

            for (var k = 1; k < nodes.Count - 1; k++)
            {
                var plain = PlainPort(k);

                if (!plain && !retiredStructurePorts.Contains(nodes[k]) || cutNodes.Contains(nodes[k].Id))
                    continue;

                var run =
                    runs.First(
                        item => nodeStation[k] >= stations[item.First] - 1e-6
                            && nodeStation[k] <= stations[item.Last + 1] + 1e-6);

                if (!plain)
                {
                    structureAliases[nodes[k].Id] = run.StartNode;
                    continue;
                }

                // A port at the very end of its run takes that end's node, which the
                // next run shares, so the ways on both sides of the port still reach it.
                var target = stations[run.Last + 1] - nodeStation[k] < .5 ? run.EndNode : run.StartNode;
                structureAliases[nodes[k].Id] = target;

                // Nodes merged into this one earlier follow it onto the run.
                if (absorbedBy.TryGetValue(nodes[k], out var merged))
                    foreach (var mergedNode in merged)
                        structureAliases[mergedNode.Id] = target;

                plainPorts++;
            }

            strokeRunTicks += Lap();

            // Vertical plan: samples every 10 m with the attributes of their run.
            var length = stations[stations.Length - 1];
            var n = Math.Max(2, (int)Math.Ceiling(length / VisualCompilerProfile.Spacing) + 1);
            var plan = new VisualCompilerProfile.Stroke
            {
                Kind = runs[0].Template.Kind,
                Class = tolerance,
                OneWay = runs[0].Template.OneWay,
                Classification = runs[0].Template.Classification,
                Station = new double[n],
                Point = new double2[n],
                Direction = new double2[n],
                Mode = new sbyte[n],
                Layer = new int[n],
                HalfWidth = new float[n],
                Grade = new float[n],
                CutFill = new float[n],
                SampleClass = new string[n],
                PairBand = new float[n],
                Water = new float[n]
            };

            for (var k = 0; k < n; k++)
            {
                var s = length * k / (n - 1);
                // The first run that reaches s.
                var findIndex = -1;

                for (var r = 0; r < runs.Count; r++)
                    if (stations[runs[r].Last + 1] >= s - 1e-6)
                    {
                        findIndex = r;
                        break;
                    }

                var segment = runs[findIndex < 0 ? runs.Count - 1 : findIndex].Template;
                plan.Station[k] = s;
                plan.Point[k] = At(pieces, stations, s);
                plan.Direction[k] =
                    math.normalizesafe(
                        At(pieces, stations, Math.Min(length, s + .5)) - At(pieces, stations, Math.Max(0, s - .5)));

                plan.Mode[k] = (sbyte)(segment.Tunnel ? -1 : segment.Bridge ? 1 : 0);
                plan.Layer[k] = segment.Layer;
                plan.HalfWidth[k] = Math.Max(4, segment.NativeWidth / 2);
                plan.Grade[k] = NetworkGeometrySafety.GradeLimit(segment);
                plan.CutFill[k] = NetworkVerticalProfiles.CutFillBudget(segment);
                plan.SampleClass[k] = segment.Classification;
                plan.PairBand[k] =
                    segment.OneWay && !segment.Roundabout ? (float)VisualCompilerTable.For(segment).PairSeparation : 0;

                plan.Water[k] = float.NaN;

                if (!segment.Tunnel && (segment.Bridge || segment.Kind == NetworkKind.Road))
                {
                    var point = new float3((float)plan.Point[k].x, 0, (float)plan.Point[k].y);
                    var surface = water?.Invoke(point) ?? float.NaN;

                    if ((segment.Kind == NetworkKind.Road || segment.Bridge) && water != null)
                    {
                        // Cover the gap between profile samples and small lateral
                        // differences between the polyline and its emitted cubic.
                        var direction = new float3((float)plan.Direction[k].x, 0, (float)plan.Direction[k].y);
                        var normal =
                            new float3(-direction.z, 0, direction.x)
                                * (plan.HalfWidth[k] + (float)VisualCompilerProfile.Spacing * .5f);

                        for (var along = -1; along <= 1; along++)
                            for (var side = -1; side <= 1; side++)
                            {
                                if (along == 0 && side == 0)
                                    continue;

                                var edge =
                                    water(
                                        point
                                            + normal * side
                                            + direction * (along * (float)VisualCompilerProfile.Spacing * .5f));

                                if (edge.HasValue && math.isfinite(edge.Value))
                                    surface = float.IsNaN(surface) ? edge.Value : Math.Max(surface, edge.Value);
                            }
                    }

                    if (!float.IsNaN(sea)
                        && NetworkProcessor.Sample(point.x, point.z, terrain, width) < sea
                        && (seaCells == null || IsSea(seaCells, point, width)))
                        surface = float.IsNaN(surface) ? sea : Math.Max(surface, sea);

                    // A ground road over a dyke: its floor (water + RoadFreeboard) is the
                    // highest crest under its width, so the deck rides over the dyke.
                    // Near a tunnel portal the road dives under the dyke instead.
                    var overDyke =
                        defenceCrests != null
                            && !segment.Bridge
                            && segment.Kind == NetworkKind.Road
                            && !NearTunnelPortal(tunnelPortals!, plan.Point[k]);

                    if (overDyke)
                    {
                        var crest = HighestCrest(defenceCrests!, point, plan.HalfWidth[k], width);

                        if (!float.IsNaN(crest))
                        {
                            var floor = crest - WaterLevels.RoadFreeboardMeters;
                            surface = float.IsNaN(surface) ? floor : Math.Max(surface, floor);
                        }
                    }

                    plan.Water[k] = surface;
                }
            }

            plan.Pins.Add((0, nodes[0], arms[nodes[0]].Count >= 3));

            foreach (var cut in allCuts)
                if (cut.StrokeNode > 0 && Junction(cut.StrokeNode))
                    plan.Pins.Add(
                        (stations[cut.Piece + 1], nodes[cut.StrokeNode], arms[nodes[cut.StrokeNode]].Count >= 3));

            foreach (var pin in plan.Pins)
                if (levelCrossings.Contains(pin.Node.Id))
                    plan.LevelCrossingNodes.Add(pin.Node.Id);

            plans.Add(plan);
            strokePlanTicks += Lap();

            // Emission waits for the vertical stage; cubics merge greedily up to 400 m.
            deferred.Add(() =>
            {
                // A cubic's height is a Hermite curve between its end heights and
                // grades; reject merges whose height strays more than 0.3 m from
                // the solved profile (report 64 V5). At 0.15 m, and the class merge
                // tolerance alone in plan, straight roads were cut into many short
                // pieces whose joints read as bumps; both are now doubled.
                bool Vertical(Piece piece, double s0, double s1, double limit = .3, bool bend = true)
                {
                    var y0 = plan.HeightAt(s0);
                    var y3 = plan.HeightAt(s1);
                    var y1 = y0 + math.distance(piece.P0, piece.P1) * plan.GradeAt(s0);
                    var y2 = y3 - math.distance(piece.P2, piece.P3) * plan.GradeAt(s1);

                    for (var q = 1; q < 4; q++)
                    {
                        var t = q / 4d;
                        var u = 1 - t;
                        var y = u * u * u * y0 + 3 * u * u * t * y1 + 3 * u * t * t * y2 + t * t * t * y3;

                        if (Math.Abs(y - plan.HeightAt(s0 + t * (s1 - s0))) > limit)
                            return false;
                    }

                    // A Hermite height has a linear second derivative, so a vertical
                    // curve filling only part of the cubic is drawn up to twice as
                    // sharp at one end (DC motorway bridge ends: 1.6–2.1× the class
                    // K). The cubic may bend no more than the class allows, or than
                    // the solved profile itself does over the same stretch. Joining a
                    // piece shorter than the class minimum ignores this: a short
                    // section reads as a kink itself, and the check added 2–3% more.
                    if (!bend || plan.Kind != NetworkKind.Road)
                        return true;

                    var rate = HermiteRate(piece, y0, y1, y2, y3);

                    return rate <= RateLimit(s0, s1) || rate <= 1.05 * plan.GradeChangeRate(s0, s1);
                }

                // Steepest grade change along a Hermite height curve, in grade per metre.
                static double HermiteRate(Piece piece, double y0, double y1, double y2, double y3)
                {
                    const int n = 16;
                    var worst = 0d;
                    var previousGrade = 0d;
                    var walked = 0d;
                    var previousStation = 0d;
                    var previous = piece.P0;

                    for (var q = 0; q <= n; q++)
                    {
                        var t = q / (double)n;
                        var u = 1 - t;
                        var point = Evaluate(piece, t);
                        walked += math.distance(previous, point);
                        previous = point;
                        var dxz =
                            3
                                * (u * u * (piece.P1 - piece.P0)
                                    + 2 * u * t * (piece.P2 - piece.P1)
                                    + t * t * (piece.P3 - piece.P2));

                        var dy = 3 * (u * u * (y1 - y0) + 2 * u * t * (y2 - y1) + t * t * (y3 - y2));
                        var grade = dy / Math.Max(1e-9, math.length(dxz));

                        if (q > 0 && walked - previousStation > 1e-6)
                            worst = Math.Max(worst, Math.Abs(grade - previousGrade) / (walked - previousStation));

                        previousGrade = grade;
                        previousStation = walked;
                    }

                    return worst;
                }

                // The class rate: 1% of grade per K metres, looser in a tunnel.
                double RateLimit(double s0, double s1)
                    => .01 / (tolerance.K * (plan.AllTunnel(s0, s1) ? VisualCompilerTable.TunnelKShare : 1));

                foreach (var run in runs)
                {
                    // A fitted loop inside a merged junction can shrink to a point at its
                    // node. A run from a node back to itself with no length is not a road.
                    if (smooth && run.StartNode == run.EndNode && stations[run.Last + 1] - stations[run.First] < 1)
                    {
                        log.Count("Smooth roads with no length left at a merged junction");
                        continue;
                    }

                    var merge = 2 * (run.Template.Kind == NetworkKind.Rail ? .3 : tolerance.MergeTolerance);
                    var runStartNode = run.StartNode;
                    // Long straights are cut into equal parts of at most 200 m, so no
                    // cubic exceeds the native span and each can follow the profile.
                    var queue = new List<(Piece Piece, double S0, double S1)>();

                    for (var p = run.First; p <= run.Last; p++)
                    {
                        var parts = (int)Math.Ceiling((stations[p + 1] - stations[p]) / 200);
                        var rest = pieces[p];

                        for (var q = parts; q > 1; q--)
                        {
                            var (left, right) = Split(rest, 1d / q);
                            var s = stations[p] + (stations[p + 1] - stations[p]) * (parts - q) / parts;
                            queue.Add((left, s, s + (stations[p + 1] - stations[p]) / parts));
                            rest = right;
                        }

                        queue.Add(
                            (
                                rest,
                                stations[p] + (stations[p + 1] - stations[p]) * (parts - 1) / parts,
                                stations[p + 1]));
                    }

                    var list = PiecesOf(queue);
                    var groups = new List<(int A, int B, Piece Piece)>();

                    // Report 113, 5.3: a fitted run is cut into equal parts, as few as follow
                    // the fitted line and the height profile. Equal parts leave no short piece
                    // over, and parts of one curve share their direction where they meet. A run
                    // that a few more equal parts do not follow has one hard place, a tight
                    // bend or a step in height; the greedy merge below spends its pieces there.
                    var even = false;

                    if (smooth)
                    {
                        var runLength = queue[queue.Count - 1].S1 - queue[0].S0;
                        var shortest = VisualCompilerTable.For(run.Template).MinimumSegment;
                        var most = Math.Max(1, (int)Math.Floor(runLength / shortest));
                        var fewest = Math.Min(most, Math.Max(1, (int)Math.Ceiling(runLength / MaximumSpan)));
                        var cutForHeight = false;

                        for (var parts = fewest; !even && parts <= Math.Min(most, fewest + EvenTries); parts++)
                        {
                            var ends = new List<int>();
                            var evenQueue = CutEvenly(queue, parts, ends);
                            var evenPieces = PiecesOf(evenQueue);
                            var evenGroups = new List<(int A, int B, Piece Piece)>();
                            var a = 0;

                            foreach (var b in ends)
                            {
                                if (!Fit(evenPieces, a, b, merge, out var cubic)
                                    || Turning(evenPieces, a, b) > math.radians(PartTurn))
                                    break;

                                if (!Vertical(cubic, evenQueue[a].S0, evenQueue[b].S1))
                                {
                                    cutForHeight = true;
                                    break;
                                }

                                evenGroups.Add((a, b, cubic));
                                a = b + 1;
                            }

                            if (evenGroups.Count < parts)
                                continue;

                            queue = evenQueue;
                            list = evenPieces;
                            groups = evenGroups;
                            even = true;
                        }

                        if (cutForHeight)
                            log.Count("Smooth road runs cut into more pieces for the height profile");
                    }

                    for (var a = 0; !even && a < queue.Count;)
                    {
                        var b = a;
                        var best = queue[a].Piece;

                        while (b + 1 < queue.Count && queue[b + 1].S1 - queue[a].S0 <= MaximumSpan
                            && Fit(list, a, b + 1, merge, out var merged)
                                && Vertical(merged, queue[a].S0, queue[b + 1].S1)
                                && (!smooth || Turning(list, a, b + 1) <= math.radians(PartTurn)))
                        {
                            b++;
                            best = merged;
                        }

                        groups.Add((a, b, best));
                        a = b + 1;
                    }

                    // A cubic shorter than the class minimum joins a neighbour when one
                    // cubic still fits within twice the merge tolerance and 0.6 m of the
                    // profile: a short piece reads as a kink, a decimetre of drift does not.
                    var minimum = Math.Max(smoothStreet ? 30 : 0, VisualCompilerTable.For(run.Template).MinimumSegment);

                    for (var g = 0; !even && g < groups.Count; g++)
                    {
                        var (a, b, _) = groups[g];

                        if (queue[b].S1 - queue[a].S0 >= minimum)
                            continue;

                        foreach (var other in new[] { g - 1, g + 1 })
                        {
                            if (other < 0 || other >= groups.Count)
                                continue;

                            var low = Math.Min(a, groups[other].A);
                            var high = Math.Max(b, groups[other].B);

                            if (queue[high].S1 - queue[low].S0 > MaximumSpan
                                || !Fit(list, low, high, 2 * merge, out var merged)
                                || !Vertical(merged, queue[low].S0, queue[high].S1, .6, bend: false))
                                continue;

                            groups[Math.Min(g, other)] = (low, high, merged);
                            groups.RemoveAt(Math.Max(g, other));
                            counts.Absorbed++;
                            g = Math.Min(g, other) - 1;
                            break;
                        }
                    }

                    if (groups.Count == 1 && queue[queue.Count - 1].S1 - queue[0].S0 < minimum)
                        counts.ShortRuns++;

                    // A short cubic that cannot join a neighbour whole shares its
                    // length instead: the two are cut at the middle of their union
                    // and refitted as two equal cubics, each at least the minimum.
                    // Short pieces between two plain joints of one road were
                    // Marseille's most common short sections.
                    var cubics =
                        groups
                            .Select(
                                item => (
                                    S0: queue[item.A].S0,
                                    S1: queue[item.B].S1,
                                    Piece: item.Piece,
                                    Merged: item.B > item.A))
                            .ToList();

                    for (var g = 0; !even && g < cubics.Count; g++)
                    {
                        if (cubics[g].S1 - cubics[g].S0 >= minimum)
                            continue;

                        foreach (var other in new[] { g + 1, g - 1 }
                            .Where(o => o >= 0 && o < cubics.Count)
                            .OrderByDescending(o => cubics[o].S1 - cubics[o].S0))
                        {
                            var u0 = Math.Min(cubics[g].S0, cubics[other].S0);
                            var u1 = Math.Max(cubics[g].S1, cubics[other].S1);
                            var middle = (u0 + u1) / 2;

                            if (u1 - u0 < 2 * minimum)
                                continue;

                            var left = new List<Piece>();
                            var right = new List<Piece>();
                            var clipped = false;

                            foreach (var item in queue.Where(item => item.S1 > u0 + 1e-6 && item.S0 < u1 - 1e-6))
                            {
                                // A previous redistribution can put this pair's
                                // outer boundary inside an original source piece.
                                // Trim that piece before fitting; including its
                                // exterior moves the shared node away from the
                                // neighbouring cubic that still owns that node.
                                var low = Math.Max(u0, item.S0);
                                var high = Math.Min(u1, item.S1);
                                var piece = item.Piece;
                                clipped |= low > item.S0 + 1e-6 || high < item.S1 - 1e-6;

                                if (high < item.S1)
                                    piece = Split(piece, (high - item.S0) / (item.S1 - item.S0)).Left;

                                if (low > item.S0)
                                    piece = Split(piece, (low - item.S0) / (high - item.S0)).Right;

                                if (high <= middle)
                                    left.Add(piece);
                                else if (low >= middle)
                                    right.Add(piece);
                                else
                                {
                                    var (otherPiece, secondPiece) = Split(piece, (middle - low) / (high - low));
                                    left.Add(otherPiece);
                                    right.Add(secondPiece);
                                }
                            }

                            if (!Fit(left, 0, left.Count - 1, 2 * merge, out var first)
                                || !Vertical(first, u0, middle, .6, bend: false)
                                || !Fit(right, 0, right.Count - 1, 2 * merge, out var second)
                                   || !Vertical(second, middle, u1, .6, bend: false))
                                continue;

                            // A repeated redistribution must not introduce or
                            // deepen a water intrusion while repairing the join.
                            // Inspect only these clipped pairs, on their actual
                            // Hermite curves and across the full road width.
                            if (clipped
                                && water != null
                                && run.Template.Kind == NetworkKind.Road
                                && !run.Template.Tunnel)
                            {
                                double Freeboard(Piece piece, double s0, double s1)
                                {
                                    var y0 = plan.HeightAt(s0);
                                    var y3 = plan.HeightAt(s1);

                                    double3 ToPoint(double2 point, double y) => new(point.x, y, point.y);

                                    var curve =
                                        new Cubic3d(
                                            ToPoint(piece.P0, y0),
                                            ToPoint(
                                                piece.P1,
                                                y0 + math.distance(piece.P0, piece.P1) * plan.GradeAt(s0)),
                                        ToPoint(piece.P2, y3 - math.distance(piece.P2, piece.P3) * plan.GradeAt(s1)),
                                            ToPoint(piece.P3, y3));

                                    var freeboard = double.PositiveInfinity;
                                    var count = Math.Max(1, (int)Math.Ceiling(Length(piece) / 5));

                                    for (var k = 0; k <= count; k++)
                                    {
                                        var t = k / (double)count;
                                        var point = NetworkDesignChecks.Evaluate(curve, t);
                                        var d = math.normalizesafe(NetworkDesignChecks.Derivative(curve, t).xz);
                                        var normal = new double2(-d.y, d.x) * Math.Max(4, run.Template.NativeWidth / 2);

                                        for (var side = -1; side <= 1; side++)
                                        {
                                            var level =
                                                water(
                                                    new(
                                                        (float)(point.x + normal.x * side),
                                                        (float)point.y,
                                                        (float)(point.z + normal.y * side)));

                                            if (level.HasValue && math.isfinite(level.Value))
                                                freeboard = Math.Min(freeboard, point.y - level.Value);
                                        }
                                    }

                                    return freeboard;
                                }

                                var prior =
                                    Math.Min(
                                        Freeboard(cubics[g].Piece, cubics[g].S0, cubics[g].S1),
                                        Freeboard(cubics[other].Piece, cubics[other].S0, cubics[other].S1));

                                var revised = Math.Min(Freeboard(first, u0, middle), Freeboard(second, middle, u1));
                                var required =
                                    run.Template.Bridge
                                        ? WaterLevels.RoadBridgeClearanceMeters
                                        : WaterLevels.RoadFreeboardMeters;

                                if (revised < Math.Min(prior, required) - .01)
                                    continue;
                            }

                            var at = Math.Min(g, other);
                            cubics[at] = (u0, middle, first, true);
                            cubics[at + 1] = (middle, u1, second, true);
                            counts.Rebalanced++;
                            break;
                        }
                    }

                    // The run's source sections; Emit only reads them.
                    var runSections = new List<int>();

                    foreach (var strokeSection in run.Path)
                        runSections.Add(path[strokeSection].Section);

                    for (var g = 0; g < cubics.Count; g++)
                    {
                        var (s0, s1, best, merged) = cubics[g];
                        var endNode = g == cubics.Count - 1 ? run.EndNode : generated++;

                        if (merged)
                            counts.Merged++;

                        if (!Vertical(best, s0, s1))
                            counts.VerticalMisfit++;

                        output.Add(
                            Emit(
                                run.Template,
                                runSections,
                                best,
                                runStartNode,
                                endNode,
                                plan,
                                s0,
                                s1,
                                !path[run.Path[0]].Forward));
                        runStartNode = endNode;
                    }
                }
            });

            (List<Piece> Pieces, List<Cut> Cuts, List<int> Missed) Build(bool smoothTangents = true)
            {
                // Douglas–Peucker between pinned nodes; other ports are projected later.
                var keep = new bool[line.Count];
                var pinnedAt = new Dictionary<int, int>();

                foreach (var k in crossingKeep)
                    keep[k] = true;

                // A fitted line passes through every node exactly, so its ports are cut
                // where they are, like junctions, and its corners are cut too.
                for (var k = 0; k < nodes.Count; k++)
                    if (Junction(k) || forced.Contains(k) || smooth && Port(k))
                    {
                        keep[at[k]] = true;
                        pinnedAt[at[k]] = k;
                    }

                foreach (var corner in corners)
                    keep[corner] = true;

                // The indices of the kept points, in order.
                int[] Kept()
                {
                    var kept = new List<int>();

                    for (var j = 0; j < line.Count; j++)
                        if (keep[j])
                            kept.Add(j);

                    return kept.ToArray();
                }

                var anchors = Kept();

                if (!smooth)
                    for (var j = 1; j < anchors.Length; j++)
                        Simplify(line, anchors[j - 1], anchors[j], tolerance.Epsilon, keep);

                var vertices = Kept();
                const double cornerRadius = 8;

                // A vertex whose fillet falls below the class radius floor
                // leaves an unnecessarily tight bend beside a short tangent.
                // Where the line without it stays within
                // the class deviation of every source point it spans, the vertex
                // is dropped instead; three passes let neighbours follow.
                double FilletRadius(IReadOnlyList<int> list, int v)
                {
                    var previous = line[list[v - 1]];
                    var point = line[list[v]];
                    var following = line[list[v + 1]];
                    var inLength = math.distance(previous, point);
                    var outLength = math.distance(point, following);
                    var dirIn = (point - previous) / Math.Max(1e-9, inLength);
                    var dirOut = (following - point) / Math.Max(1e-9, outLength);
                    var half =
                        Math.Abs(Math.Atan2(NetworkDesignChecks.Cross(dirIn, dirOut), math.dot(dirIn, dirOut))) / 2;

                    if (half < math.radians(.1))
                        return double.PositiveInfinity;

                    if (half > math.radians(85))
                        return 0;

                    double Available(int neighbour, double length)
                        => pinnedAt.TryGetValue(list[neighbour], out var k) ? length - Stub(k) : length / 2;

                    var tangent = Math.Max(0, Math.Min(Available(v - 1, inLength), Available(v + 1, outLength)));

                    return Math.Min(
                        tolerance.RadiusTarget,
                        Math.Min(tangent / Math.Tan(half), tolerance.Deviation / (1 / Math.Cos(half) - 1)));
                }

                var kept = vertices.ToList();

                for (var pass = 0; pass < 3; pass++)
                {
                    var dropped = false;

                    for (var v = 1; v < kept.Count - 1; v++)
                    {
                        if (smooth || pinnedAt.ContainsKey(kept[v]) || FilletRadius(kept, v) >= optionalCornerRadius)
                            continue;

                        var vectorA = line[kept[v - 1]];
                        var vectorB = line[kept[v + 1]];
                        var ab = vectorB - vectorA;
                        var worst = 0d;

                        for (var j = kept[v - 1] + 1; j < kept[v + 1]; j++)
                        {
                            var t =
                                math.clamp(math.dot(line[j] - vectorA, ab) / Math.Max(1e-9, math.lengthsq(ab)), 0, 1);

                            worst = Math.Max(worst, math.distance(line[j], vectorA + ab * t));
                        }

                        if (worst > tolerance.Deviation)
                            continue;

                        kept.RemoveAt(v);
                        v--;
                        dropped = true;
                        counts.CornersDropped++;
                    }

                    if (!dropped)
                        break;
                }

                vertices = kept.ToArray();

                // Straights joined by circular fillets (report 64 V2 step 2, rules R1/R5).
                // A curve tighter than the class floor is still drawn as an arc; only
                // a radius too small to read as a curve becomes a node corner.
                var pieces = new List<Piece>();
                var cuts = new List<Cut>();
                var current = line[vertices[0]];
                // Motorways, trunks and their links: one smooth curve through the
                // junctions instead of straights and fillets (report 103).
                var highway = source[path[0].Section];
                var pins = new List<int>();

                for (var j = 0; j < line.Count; j++)
                    if (j == 0 || j == line.Count - 1 || pinnedAt.ContainsKey(j) || corners.Contains(j))
                        pins.Add(j);

                var pinList = pins.ToArray();

                // Under smooth-roads the line is already the fitted curve (FitLine).
                (double2[] Points, int[] Pins)? fitted =
                    smooth
                        ? (line.ToArray(), pinList)
                        : highway.Kind == NetworkKind.Road && NativeAssetKeys.IsHighway(highway) && !highway.Roundabout
                            ? HighwayAlignment.Fit(
                                line,
                                pinList,
                                highway.ProtectedLink ? LinkWavelength : MainlineWavelength,
                                highway.ProtectedLink ? LinkBudget : MainlineBudget)
                            : null;

                if (fitted.HasValue)
                {
                    for (var p = 0; p + 1 < fitted.Value.Pins.Length; p++)
                    {
                        var (curve, curvePins) = fitted.Value;
                        var from2 = curvePins[p];
                        var to = curvePins[p + 1];
                        var spans = Math.Max(1, (to - from2) / 4);

                        for (var q = 0; q < spans; q++)
                        {
                            var a = from2 + (to - from2) * q / spans;
                            var b = from2 + (to - from2) * (q + 1) / spans;

                            // The curve's direction at a sample; at a corner, the
                            // direction of the side the piece lies on.
                            double2 Tangent(int i, bool leaving)
                            {
                                var corner = corners.Contains(i);
                                var before = corner && leaving ? i : Math.Max(0, i - 1);
                                var after = corner && !leaving ? i : Math.Min(curve.Length - 1, i + 1);

                                return math.normalizesafe(curve[after] - curve[before]);
                            }

                            var reach = math.distance(curve[a], curve[b]) / 3;
                            pieces.Add(
                                new(
                                    curve[a],
                                    curve[a] + Tangent(a, true) * reach,
                                    curve[b] - Tangent(b, false) * reach,
                                    curve[b]));
                        }

                        if (pinnedAt.TryGetValue(pinList[p + 1], out var pinNode))
                            Boundary(nodes[pinNode].Id, pinNode);
                        else if (corners.Contains(pinList[p + 1]))
                        {
                            // A corner that falls on a source node keeps that node's identity.
                            var cornerNode = at.IndexOf(pinList[p + 1], 1);

                            if (cornerNode > 0 && cornerNode < nodes.Count - 1)
                                Boundary(nodes[cornerNode].Id, cornerNode);
                            else
                                Boundary(generated++, -1);

                            counts.Corners++;
                        }
                    }
                }

                void Straight(double2 vectorA, double2 vectorB)
                {
                    if (math.distancesq(vectorA, vectorB) > 1e-6)
                        pieces.Add(
                            new(
                                vectorA,
                                math.lerp(vectorA, vectorB, 1 / 3d),
                                math.lerp(vectorA, vectorB, 2 / 3d),
                                vectorB));
                }

                void Boundary(long node, int strokeNode)
                {
                    if (pieces.Count > 0)
                        cuts.Add(new(pieces.Count - 1, node, strokeNode));
                }

                if (fitted == null)
                    for (var v = 1; v < vertices.Length; v++)
                    {
                        var j = vertices[v];
                        var point = line[j];

                        if (pinnedAt.TryGetValue(j, out var pinnedNode))
                        {
                            Straight(current, point);
                            current = point;
                            Boundary(nodes[pinnedNode].Id, pinnedNode);
                            continue;
                        }

                        var previous = line[vertices[v - 1]];
                        var following = line[vertices[v + 1]];
                        var inLength = math.distance(previous, point);
                        var outLength = math.distance(point, following);
                        var dirIn = (point - previous) / Math.Max(1e-9, inLength);
                        var dirOut = (following - point) / Math.Max(1e-9, outLength);
                        var turn = Math.Atan2(NetworkDesignChecks.Cross(dirIn, dirOut), math.dot(dirIn, dirOut));
                        var half = Math.Abs(turn) / 2;

                        if (half < math.radians(.1))
                        {
                            Straight(current, point);
                            current = point;
                            continue;
                        }

                        double Available(int neighbour, double length)
                            => pinnedAt.TryGetValue(vertices[neighbour], out var k) ? length - Stub(k) : length / 2;

                        var tangent = Math.Max(0, Math.Min(Available(v - 1, inLength), Available(v + 1, outLength)));
                        var radius =
                            Math.Min(
                                tolerance.RadiusTarget,
                                Math.Min(tangent / Math.Tan(half), tolerance.Deviation / (1 / Math.Cos(half) - 1)));

                        if (radius < cornerRadius || half > math.radians(85))
                        {
                            Straight(current, point);
                            current = point;
                            Boundary(generated++, -1);
                            counts.Corners++;
                            continue;
                        }

                        var t = radius * Math.Tan(half);
                        var t1 = point - dirIn * t;
                        var t2 = point + dirOut * t;
                        Straight(current, t1);
                        Arc(pieces, t1, dirIn, radius, turn);
                        current = t2;
                    }

                var missed = new List<int>();

                if (pieces.Count == 0)
                    return (pieces, cuts, missed);

                // A retained street junction is a position constraint, not a
                // corner. Keep one tangent through a gently continuing route;
                // the old pair of straight chords left a kink at every small
                // survey offset. Endpoints and connecting side streets stay put.
                if (fitted == null && (smoothStreet || smoothMajor) && smoothTangents)
                    foreach (var cut in cuts)
                    {
                        var k = cut.StrokeNode;
                        var piece = cut.Piece;

                        if (piece + 1 >= pieces.Count
                            || k >= 0
                                && (k == 0 || k >= nodes.Count - 1 || fixedNodes.Contains(nodes[k]) || !smoothMajor
                                    && arms[nodes[k]].Any(
                                arm => source[arm.Section].Kind != NetworkKind.Road
                                    || NativeAssetKeys.IsHighway(source[arm.Section])
                                    || source[arm.Section].ProtectedLink
                                    || source[arm.Section].Roundabout
                                    || NetworkStructure.IsStructure(source[arm.Section]))))
                            continue;

                        var firstPiece = pieces[piece];
                        var secondPiece = pieces[piece + 1];
                        var da = math.normalizesafe(firstPiece.P3 - firstPiece.P2);
                        var db = math.normalizesafe(secondPiece.P1 - secondPiece.P0);

                        if (math.dot(da, db) < Math.Cos(math.radians(!smoothStreet ? 20 : k < 0 ? 120 : 35)))
                            continue;

                        var direction = math.normalizesafe(da + db);
                        var c2 = firstPiece.P3 - direction * math.distance(firstPiece.P2, firstPiece.P3);
                        var c1 = secondPiece.P0 + direction * math.distance(secondPiece.P0, secondPiece.P1);

                        if (math.distance(c2, firstPiece.P2) > tolerance.Deviation
                            || math.distance(c1, secondPiece.P1) > tolerance.Deviation)
                            continue;

                        pieces[piece] = firstPiece with { P2 = c2 };
                        pieces[piece + 1] = secondPiece with { P1 = c1 };
                    }

                // Degree-two ports (attribute changes, absorbed targets) keep their
                // identity: split the alignment at their nearest point, in order. A
                // port landing on an existing cut moves to the middle of its piece.
                var from = 0;

                for (var k = 1; k < nodes.Count - 1; k++)
                    if (Port(k) && !forced.Contains(k) && !smooth)
                    {
                        var target = line[at[k]];
                        var best = (Piece: -1, T: 0d, Distance: double.PositiveInfinity);

                        void Search(int first)
                        {
                            for (var p = first; p < pieces.Count; p++)
                            {
                                // A piece with no point nearer than the best so far cannot replace it.
                                if (PieceBeyond(pieces[p], target, best.Distance))
                                    continue;

                                for (var q = 0; q <= 32; q++)
                                {
                                    var otherDistance = math.distance(Evaluate(pieces[p], q / 32d), target);

                                    if (otherDistance < best.Distance)
                                        best = (p, q / 32d, otherDistance);
                                }
                            }
                        }

                        Search(from);

                        if (best.Distance > 2 * tolerance.Deviation)
                            Search(0);

                        if (best.Piece < 0 || best.Distance > 2 * tolerance.Deviation && !lastResort)
                        {
                            missed.Add(k);
                            continue;
                        }

                        best = Refine(pieces, target, best.Piece, best.T);
                        counts.Projected++;
                        from = best.Piece;
                        var boundary = best.T <= 1e-4 ? best.Piece - 1 : best.T >= 1 - 1e-4 ? best.Piece : -2;

                        if (boundary >= 0 && cuts.All(cut => cut.Piece != boundary))
                        {
                            cuts.Add(new(boundary, nodes[k].Id, k));
                            continue;
                        }

                        // A corner node carries no identity: the port takes its place
                        // rather than leaving a 0.5 m piece beside it.
                        var corner =
                            boundary >= 0 ? cuts.FindIndex(cut => cut.Piece == boundary && cut.StrokeNode < 0) : -1;

                        if (corner >= 0)
                        {
                            cuts[corner] = new(boundary, nodes[k].Id, k);
                            continue;
                        }

                        // On an occupied boundary, split just after it so stroke order holds.
                        var piece = boundary >= 0 ? boundary + 1 : best.Piece;

                        if (piece >= pieces.Count)
                        {
                            missed.Add(k);
                            continue;
                        }

                        var split = boundary == -2 ? best.T : Math.Min(.5, .5 / Math.Max(1, Length(pieces[piece])));
                        var (left, right) = Split(pieces[piece], split);
                        pieces[piece] = left;
                        pieces.Insert(piece + 1, right);

                        for (var c = 0; c < cuts.Count; c++)
                            if (cuts[c].Piece >= piece)
                                cuts[c] = cuts[c] with { Piece = cuts[c].Piece + 1 };

                        cuts.Add(new(piece, nodes[k].Id, k));
                    }

                cuts = cuts.OrderBy(cut => cut.Piece).ToList();

                if (cuts.Count == 0 || cuts[cuts.Count - 1].Piece != pieces.Count - 1)
                    cuts.Add(new(pieces.Count - 1, nodes[nodes.Count - 1].Id, nodes.Count - 1));

                if (smoothStreet && smoothTangents && pieces.Any(piece => Length(piece) < 3))
                {
                    log.Count("Street tangent smoothing sliver fallbacks");
                    return Build(false);
                }

                return (pieces, cuts, missed);
            }
        }

        PreparedSegment Emit(
            PreparedSegment template,
            List<int> covered,
            Piece piece,
            long startNode,
            long endNode,
            VisualCompilerProfile.Stroke plan,
            double s0,
            double s1,
            bool backwards)
        {
            // Heights and handle slopes come from the solved profile, so adjacent
            // cubics share both height and grade at every joint.
            var kind = template.Kind;
            var y0 = NodeHeight(kind, startNode, (float)plan.HeightAt(s0));
            var y3 = NodeHeight(kind, endNode, (float)plan.HeightAt(s1));

            float3 ToPoint(double2 point, float y) => new((float)point.x, y, (float)point.y);

            var p0 = Exact(kind, startNode, ToPoint(piece.P0, y0));
            var p3 = Exact(kind, endNode, ToPoint(piece.P3, y3));
            var p1 = ToPoint(piece.P1, (float)(y0 + math.distance(piece.P0, piece.P1) * plan.GradeAt(s0)));
            var p2 = ToPoint(piece.P2, (float)(y3 - math.distance(piece.P2, piece.P3) * plan.GradeAt(s1)));

            if (backwards)
            {
                (p0, p3) = (p3, p0);
                (p1, p2) = (p2, p1);
                (startNode, endNode) = (endNode, startNode);
            }

            var ways =
                covered
                    .SelectMany(i => source[i].SourceWayIds.Append(source[i].WayId))
                    .Distinct()
                    .OrderBy(wayId => wayId)
                    .ToArray();

            // The nodes absorbed into either end, in the order of absorbed.
            var mergedAtEnds = new List<SourceNodeKey>();

            if (absorbedBy.TryGetValue(new(kind, startNode), out var mergedAtStart))
                mergedAtEnds.AddRange(mergedAtStart);

            if (endNode != startNode && absorbedBy.TryGetValue(new(kind, endNode), out var mergedAtEnd))
                mergedAtEnds.AddRange(mergedAtEnd);

            mergedAtEnds.Sort((first, second) => absorbedOrder[first].CompareTo(absorbedOrder[second]));

            // An earlier alias onto a node this compiler absorbed follows it.
            var aliases = covered.SelectMany(i => source[i].SourceNodeAliases)
                .Select(
                    alias => absorbed.TryGetValue(new(kind, alias.CanonicalNodeId), out var merged)
                        ? alias with { CanonicalNodeId = merged.Id }
                        : alias)
                .Concat(
                    mergedAtEnds.Select(
                        key => new SourceNodeAlias(key.Id, absorbed[key].Id, ports.Contains(key))))
                    .GroupBy(alias => alias.SourceNodeId)
                    .Select(group => group.Last())
                    .OrderBy(alias => alias.SourceNodeId)
                    .ToArray();

            var geometry = covered.SelectMany(i => source[i].SourceGeometry).ToArray();

            // A tunnel's native elevation is its solved depth below the ground, not
            // an earlier stage's value: the game draws it open unless it is deep.
            float Depth(float3 point) => point.y - NetworkProcessor.Sample(point.x, point.z, terrain, width);

            var startElevation = template.Tunnel ? Depth(p0) : template.StartElevation;
            var endElevation = template.Tunnel ? Depth(p3) : template.EndElevation;

            return template with
            {
                StartNodeId = startNode,
                EndNodeId = endNode,
                Start = p0,
                End = p3,
                Control1 = p1,
                Control2 = p2,
                StartElevation = startElevation,
                EndElevation = endElevation,
                StartCurveOffset = default,
                EndCurveOffset = default,
                FinalDeckCurve = null,
                HorizontalParent = null,
                HorizontalEmissionReference = null,
                SourceWayIds = ways,
                SourceNodeAliases = aliases,
                SourceGeometry = geometry,
                NativeUnitId = -1,
                StartTerminal = template.StartTerminal && startNode == template.StartNodeId,
                EndTerminal = template.EndTerminal && endNode == template.EndNodeId,
                StartMiniRoundabout = template.StartMiniRoundabout && startNode == template.StartNodeId,
                EndMiniRoundabout = template.EndMiniRoundabout && endNode == template.EndNodeId,
                StartRoundaboutRadius = roundaboutRadii.GetValueOrDefault(new(kind, startNode)),
                EndRoundaboutRadius = roundaboutRadii.GetValueOrDefault(new(kind, endNode))
            };
        }

        float NodeHeight(NetworkKind kind, long node, float fallback)
            => nodeY.TryGetValue(new(kind, node), out var y) ? y : fallback;

        float3 Exact(NetworkKind kind, long node, float3 point)
        {
            // Every section meeting at a shared node must submit the identical position.
            var key = new SourceNodeKey(kind, node);

            if (!arms.ContainsKey(key))
                return point;

            if (!exact.TryGetValue(key, out var value))
                exact[key] = value = point;

            return value;
        }
    }

    // A motorway or trunk mapped as separate bridges with short ground ways
    // between (the A7 into Marseille, whose embankment the elevation data
    // carries) was drawn viaduct, embankment, viaduct at about one height. A
    // ground run of at most BridgeGap metres of the same class between two of
    // its bridges, through plain two-arm nodes, becomes bridge at their layer:
    // one straight elevated run. Only between viaducts, bridge runs of at
    // least ViaductRun metres on both sides (the A7's are 109–1,002 m): a
    // motorway at grade over a few 20–40 m underpass bridges stays on the ground.
    private const double BridgeGap = 120, ViaductRun = 100;

    private static IReadOnlyList<PreparedSegment> BridgeGaps(
        IReadOnlyList<PreparedSegment> source,
        ImportLog log,
        CancellationToken token)
    {
        static bool Major(PreparedSegment segment)
            => segment.Kind == NetworkKind.Road
                && !segment.Roundabout
                && segment.Classification is "motorway" or "trunk";

        var arms = new Dictionary<SourceNodeKey, List<int>>();

        for (var i = 0; i < source.Count; i++)
            foreach (var id in new[] { source[i].StartNodeId, source[i].EndNodeId })
            {
                var key = new SourceNodeKey(source[i].Kind, id);

                if (!arms.TryGetValue(key, out var list))
                    arms[key] = list = new();

                list.Add(i);
            }

        bool Ground(int i) => !source[i].Bridge && !source[i].Tunnel;

        // Length of the bridge run starting with section first, entered at node.
        double Viaduct(int first, SourceNodeKey node)
        {
            var total = 0d;
            var current = first;
            var visited = new HashSet<int>();

            while (source[current].Bridge && visited.Add(current) && total < ViaductRun)
            {
                total += NetworkDesignChecks.HorizontalArcLength(NetworkDesignChecks.Course(source[current]));
                node =
                    new SourceNodeKey(
                        node.Kind,
                        source[current].StartNodeId == node.Id
                            ? source[current].EndNodeId
                            : source[current].StartNodeId);

                if (arms[node].Count != 2)
                    break;

                current = arms[node][0] == current ? arms[node][1] : arms[node][0];
            }

            return total;
        }

        PreparedSegment[]? result = null;
        var seen = new bool[source.Count];
        var runs = 0;

        for (var i = 0; i < source.Count; i++)
        {
            if (seen[i] || !Major(source[i]) || !Ground(i))
                continue;

            token.ThrowIfCancellationRequested();
            var chain = new List<int> { i };
            seen[i] = true;
            var outer = new[] { -1, -1 };
            var entry = new SourceNodeKey[2];

            for (var side = 0; side < 2; side++)
            {
                var node = new SourceNodeKey(source[i].Kind, side == 0 ? source[i].StartNodeId : source[i].EndNodeId);
                var current = i;

                while (arms[node].Count == 2)
                {
                    var next = arms[node][0] == current ? arms[node][1] : arms[node][0];

                    if (!Major(source[next]) || source[next].Classification != source[i].Classification)
                        break;

                    if (!Ground(next))
                    {
                        if (source[next].Bridge)
                        {
                            outer[side] = next;
                            entry[side] = node;
                        }

                        break;
                    }

                    if (seen[next])
                        break;

                    seen[next] = true;
                    chain.Add(next);
                    current = next;
                    node =
                        new SourceNodeKey(
                            source[next].Kind,
                            source[next].StartNodeId == node.Id ? source[next].EndNodeId : source[next].StartNodeId);
                }
            }

            if (outer[0] < 0
                || outer[1] < 0
                || chain.Sum(k => NetworkDesignChecks.HorizontalArcLength(NetworkDesignChecks.Course(source[k])))
                    > BridgeGap
                || Viaduct(outer[0], entry[0]) < ViaductRun || Viaduct(outer[1], entry[1]) < ViaductRun)
                continue;

            result ??= source.ToArray();
            var layer = Math.Min(source[outer[0]].Layer, source[outer[1]].Layer);

            foreach (var k in chain)
                result[k] = source[k] with { Bridge = true, Layer = Math.Max(source[k].Layer, layer) };

            runs++;
        }

        log.Count("Ground gaps between bridges built as bridge", runs);

        return result ?? source;
    }

    // A road or track piece shorter than its class minimum, left between two
    // emitted pieces that the game builds identically (the same asset, lanes,
    // direction and structure; for track also service, track count and
    // terminal formation), joins its neighbour as one cubic. Approach elevation
    // is the usual cause on roads: it turns the ground either side of a short
    // source bridge into bridge, and the source's bridge and layer boundaries
    // become plain joints. On track the removed nodes are mostly aliases left
    // by consolidating parallel tracks. A port on the removed node becomes a
    // mandatory alias of the short piece's far end, which lies less than the
    // minimum away. The join must stay within the compiler's short-piece
    // allowance: twice the doubled merge tolerance across, 0.6 m in height. A
    // joined track may curve no tighter than the class radius floor or the
    // tighter of its two pieces, and must pass the native train-curvature check.
    private static IReadOnlyList<PreparedSegment> JoinShortPieces(
        IReadOnlyList<PreparedSegment> source,
        HashSet<SourceNodeKey> required,
        HashSet<long> levelCrossings,
        bool joinRings,
        ImportLog log,
        CancellationToken token)
    {
        var list = source.ToList<PreparedSegment?>();
        var ends = new Dictionary<SourceNodeKey, List<int>>();

        void Attach(int i, long id)
        {
            var key = new SourceNodeKey(list[i]!.Kind, id);

            if (!ends.TryGetValue(key, out var arms))
                ends[key] = arms = new();

            arms.Add(i);
        }

        for (var i = 0; i < list.Count; i++)
        {
            Attach(i, list[i]!.StartNodeId);
            Attach(i, list[i]!.EndNodeId);
        }

        double Length(PreparedSegment segment)
            => NetworkDesignChecks.HorizontalArcLength(NetworkDesignChecks.Course(segment));

        // With joinRings every roundabout piece is tried, so a ring keeps one piece
        // between neighbouring arms. TryJoin refuses a joined piece that turns more
        // than a quarter circle, which one cubic cannot follow.
        bool Short(PreparedSegment segment)
            => segment.Roundabout
                ? joinRings
                : Length(segment) < VisualCompilerTable.For(segment).MinimumSegment;

        double Curvature(Cubic3d curve)
            => Enumerable.Range(0, 33).Max(k => Math.Abs(NetworkDesignChecks.Curvature(curve, k / 32d)));

        bool Same(PreparedSegment firstSegment, PreparedSegment secondSegment)
            => firstSegment.Kind == secondSegment.Kind
                && firstSegment.Classification == secondSegment.Classification
                && firstSegment.NativeAssetKey == secondSegment.NativeAssetKey
                && firstSegment.NativeWidth == secondSegment.NativeWidth
                && firstSegment.Lanes == secondSegment.Lanes
                && firstSegment.ForwardLanes == secondSegment.ForwardLanes
                && firstSegment.BackwardLanes == secondSegment.BackwardLanes
                && firstSegment.BothWaysLanes == secondSegment.BothWaysLanes
                && firstSegment.OneWay == secondSegment.OneWay
                && firstSegment.Bridge == secondSegment.Bridge
                && firstSegment.Tunnel == secondSegment.Tunnel
                && firstSegment.Roundabout == secondSegment.Roundabout
                && (joinRings || !firstSegment.Roundabout)
                && firstSegment.NativeLaneOffsets.SequenceEqual(secondSegment.NativeLaneOffsets)
                && firstSegment.PairedProfileId == secondSegment.PairedProfileId
                && firstSegment.PairedSpanId == 0
                && secondSegment.PairedSpanId == 0
                && firstSegment.ConsolidatedAvenue == secondSegment.ConsolidatedAvenue
                && firstSegment.ExplicitCutSides == secondSegment.ExplicitCutSides
                && firstSegment.ExplicitWallSides == secondSegment.ExplicitWallSides
                && firstSegment.RetainingWallSides == secondSegment.RetainingWallSides
                && firstSegment.QuaySides == secondSegment.QuaySides
                && firstSegment.FinalDeckCurve == null
                && secondSegment.FinalDeckCurve == null
                && firstSegment.RailService == secondSegment.RailService
                && firstSegment.PhysicalTracks == secondSegment.PhysicalTracks
                && firstSegment.RailTerminalFormationId == secondSegment.RailTerminalFormationId
                && firstSegment.NativeCenterlineCurvatureLimit == secondSegment.NativeCenterlineCurvatureLimit;

        bool MainJoin(PreparedSegment firstSegment, PreparedSegment secondSegment)
            => IsGroundStreet(firstSegment)
                && IsGroundStreet(secondSegment)
                && firstSegment.OneWay == secondSegment.OneWay
                && firstSegment.ConsolidatedAvenue == secondSegment.ConsolidatedAvenue
                && firstSegment.PairedProfileId == secondSegment.PairedProfileId
                && firstSegment.PairedSpanId == 0
                && secondSegment.PairedSpanId == 0
                && firstSegment.FinalDeckCurve == null
                && secondSegment.FinalDeckCurve == null
                && firstSegment.ExplicitCutSides == secondSegment.ExplicitCutSides
                && firstSegment.ExplicitWallSides == secondSegment.ExplicitWallSides
                && firstSegment.RetainingWallSides == secondSegment.RetainingWallSides
                && firstSegment.QuaySides == secondSegment.QuaySides;

        const double MainJoinAcross = 4, MainJoinHeight = 1;
        var removed = new Dictionary<SourceNodeKey, (long Keep, bool Mandatory)>();
        var joined = 0;
        var queue = Enumerable.Range(0, list.Count).Where(i => Short(list[i]!)).ToList();

        for (var q = 0; q < queue.Count; q++)
        {
            token.ThrowIfCancellationRequested();
            var i = queue[q];
            var segment = list[i];

            if (segment == null || !Short(segment))
                continue;

            (PreparedSegment Merged, int Other, SourceNodeKey Node, double Error)? best = null;

            foreach (var atStart in new[] { true, false })
            {
                var node = new SourceNodeKey(segment.Kind, atStart ? segment.StartNodeId : segment.EndNodeId);

                if (!ends.TryGetValue(node, out var arms) || arms.Count != 2 || levelCrossings.Contains(node.Id))
                    continue;

                var j = arms[0] == i ? arms[1] : arms[0];
                var otherSegment = list[j];

                if (j == i || otherSegment == null)
                    continue;

                // Orient as a then b through the node; a two-way piece may be reversed.
                var firstSegment = atStart ? otherSegment : segment;
                var secondSegment = atStart ? segment : otherSegment;

                if (firstSegment.EndNodeId != node.Id)
                {
                    if (firstSegment.OneWay)
                        continue;

                    firstSegment = NetworkNormalizer.Reverse(firstSegment);
                }

                if (secondSegment.StartNodeId != node.Id)
                {
                    if (secondSegment.OneWay)
                        continue;

                    secondSegment = NetworkNormalizer.Reverse(secondSegment);
                }

                if (firstSegment.EndNodeId != node.Id
                    || secondSegment.StartNodeId != node.Id
                    || firstSegment.StartNodeId == secondSegment.EndNodeId
                    || !Same(firstSegment, secondSegment) && !MainJoin(firstSegment, secondSegment)
                    || math.any(firstSegment.EndCurveOffset != 0)
                       || math.any(secondSegment.StartCurveOffset != 0)
                       || firstSegment.EndTerminal
                       || secondSegment.StartTerminal
                       || firstSegment.EndMiniRoundabout
                       || secondSegment.StartMiniRoundabout
                       || firstSegment.EndLevelCrossing
                       || secondSegment.StartLevelCrossing
                       || firstSegment.EndRoundaboutRadius > 0
                       || secondSegment.StartRoundaboutRadius > 0)
                    continue;

                if (Length(firstSegment) + Length(secondSegment) > MaximumSpan
                    || !TryJoin(firstSegment, secondSegment, out var merged, out var error))
                    continue;

                var tolerance = VisualCompilerTable.For(segment);
                var mainJoin = MainJoin(firstSegment, secondSegment);

                if (error.Across > (mainJoin ? MainJoinAcross : 4 * tolerance.MergeTolerance)
                    || error.Height > (mainJoin ? MainJoinHeight : .6))
                    continue;

                if (mainJoin && !Same(firstSegment, secondSegment))
                {
                    var longer = Length(firstSegment) >= Length(secondSegment) ? firstSegment : secondSegment;

                    // The far end's lane offset was worked out for the short piece's own width.
                    if (longer == secondSegment
                        ? math.any(firstSegment.StartCurveOffset != 0)
                        : math.any(secondSegment.EndCurveOffset != 0))
                        continue;

                    merged = merged with
                    {
                        Classification = longer.Classification,
                        NativeAssetKey = longer.NativeAssetKey,
                        NativeWidth = longer.NativeWidth,
                        Lanes = longer.Lanes,
                        ForwardLanes = longer.ForwardLanes,
                        BackwardLanes = longer.BackwardLanes,
                        BothWaysLanes = longer.BothWaysLanes,
                        NativeLaneOffsets = longer.NativeLaneOffsets
                    };
                }

                if (segment.Kind == NetworkKind.Rail)
                {
                    var curve = NetworkDesignChecks.Course(merged);

                    if (Curvature(curve)
                        > Math.Max(
                            1 / tolerance.RadiusFloor,
                            Math.Max(
                                Curvature(NetworkDesignChecks.Course(firstSegment)),
                                Curvature(NetworkDesignChecks.Course(secondSegment))))
                            + 1e-9)
                        continue;

                    if (merged.NativeCenterlineCurvatureLimit is double k
                        && NetworkDesignChecks.Certify(
                            curve,
                            k,
                            double.PositiveInfinity,
                            token: token,
                            maximumLeaves: 4096).Quality
                            != DesignQuality.Certified)
                        continue;
                }

                // A structure keeps its highest (bridge) or lowest (tunnel) layer.
                merged =
                    merged with
                    {
                        Layer =
                            segment.Tunnel
                                ? Math.Min(firstSegment.Layer, secondSegment.Layer)
                                : segment.Bridge
                                    ? Math.Max(firstSegment.Layer, secondSegment.Layer)
                                    : (Length(firstSegment) >= Length(secondSegment)
                                        ? firstSegment
                                        : secondSegment).Layer
                    };

                if (best == null || error.Across < best.Value.Error)
                    best = (merged, j, node, error.Across);
            }

            if (best == null)
                continue;

            var (result, other, removedNode, _) = best.Value;
            var keep = removedNode.Id == segment.StartNodeId ? segment.EndNodeId : segment.StartNodeId;
            removed[removedNode] = (keep, required.Contains(removedNode));
            list[i] = result;
            list[other] = null;
            ends.Remove(removedNode);
            joined++;
            var far =
                new SourceNodeKey(segment.Kind, result.StartNodeId == keep ? result.EndNodeId : result.StartNodeId);

            var farArms = ends[far];
            farArms[farArms.IndexOf(other)] = i;

            if (Short(result))
                queue.Add(i);
        }

        if (joined == 0)
            return source;

        log.Count("Short pieces joined across plain joints after approach elevation", joined);

        long Resolve(SourceNodeKey key)
        {
            while (removed.TryGetValue(key, out var item))
                key = key with { Id = item.Keep };

            return key.Id;
        }

        var byKeep =
            removed
                .GroupBy(entry => new SourceNodeKey(entry.Key.Kind, Resolve(entry.Key)))
                .ToDictionary(
                    group => group.Key,
                    group => group
                        .Select(entry => new SourceNodeAlias(entry.Key.Id, group.Key.Id, entry.Value.Mandatory))
                        .ToArray());

        return list.OfType<PreparedSegment>().Select(segment =>
        {
            var start =
                byKeep.TryGetValue(new(segment.Kind, segment.StartNodeId), out var firstAlias)
                    ? firstAlias
                    : Array.Empty<SourceNodeAlias>();

            var end =
                byKeep.TryGetValue(new(segment.Kind, segment.EndNodeId), out var secondAlias)
                    ? secondAlias
                    : Array.Empty<SourceNodeAlias>();

            if (start.Length == 0
                && end.Length == 0
                && !segment.SourceNodeAliases.Any(
                    alias => removed.ContainsKey(new(segment.Kind, alias.CanonicalNodeId))))
                return segment;

            var aliases =
                segment.SourceNodeAliases
                    .Select(
                        alias => removed.ContainsKey(new(segment.Kind, alias.CanonicalNodeId))
                            ? alias with { CanonicalNodeId = Resolve(new(segment.Kind, alias.CanonicalNodeId)) }
                            : alias)
                .Concat(start)
                    .Concat(end)
                    .GroupBy(alias => alias.SourceNodeId)
                    .Select(group => group.OrderByDescending(alias => alias.Mandatory).First())
                    .OrderBy(alias => alias.SourceNodeId)
                    .ToArray();

            return segment with { SourceNodeAliases = aliases };
        }).ToArray();
    }

    // One cubic through a then b: the outer ends and tangent directions stay,
    // and the two handle lengths are a least-squares fit to points along both
    // pieces at chord-length parameters. Error is the largest horizontal and
    // vertical departure of those points from the joined curve.
    private static bool TryJoin(
        PreparedSegment firstSegment,
        PreparedSegment secondSegment,
        out PreparedSegment merged,
        out (double Across, double Height) error)
    {
        merged = firstSegment;
        error = (double.PositiveInfinity, double.PositiveInfinity);
        var ca = NetworkDesignChecks.Course(firstSegment);
        var callback = NetworkDesignChecks.Course(secondSegment);
        var p0 = ca.P0;
        var p3 = callback.P3;
        var ta = math.normalizesafe(ca.P1 - ca.P0);
        var tb = math.normalizesafe(callback.P3 - callback.P2);

        if (math.lengthsq(ta) < .5 || math.lengthsq(tb) < .5 || math.dot(ta.xz, tb.xz) < 0)
            return false;

        var points = new List<double3>();

        for (var k = 0; k <= 8; k++)
            points.Add(NetworkDesignChecks.Evaluate(ca, k / 8d));

        for (var k = 1; k <= 8; k++)
            points.Add(NetworkDesignChecks.Evaluate(callback, k / 8d));

        var u = new double[points.Count];

        for (var k = 1; k < points.Count; k++)
            u[k] = u[k - 1] + math.distance(points[k - 1], points[k]);

        var total = u[^1];

        if (total < 1e-3)
            return false;

        double m11 = 0, m12 = 0, m22 = 0, r1 = 0, r2 = 0;

        for (var k = 0; k < points.Count; k++)
        {
            var t = u[k] / total;
            var w = 1 - t;
            var b0 = w * w * w;
            var b1 = 3 * w * w * t;
            var b2 = 3 * w * t * t;
            var b3 = t * t * t;
            var r = points[k] - (b0 + b1) * p0 - (b2 + b3) * p3;
            m11 += b1 * b1;
            m12 -= b1 * b2 * math.dot(ta, tb);
            m22 += b2 * b2;
            r1 += b1 * math.dot(ta, r);
            r2 -= b2 * math.dot(tb, r);
        }

        var det = m11 * m22 - m12 * m12;

        if (Math.Abs(det) < 1e-12)
            return false;

        var alpha = (r1 * m22 - m12 * r2) / det;
        var beta = (m11 * r2 - m12 * r1) / det;

        if (!(alpha > 0 && beta > 0) || alpha > total || beta > total)
            return false;

        var curve = new Cubic3d(p0, p0 + alpha * ta, p3 - beta * tb, p3);
        var samples = Enumerable.Range(0, 129).Select(k => NetworkDesignChecks.Evaluate(curve, k / 128d)).ToArray();
        double across = 0, height = 0;

        foreach (var point in points)
        {
            // Nearest point of the sampled joined curve, by projection onto its chords.
            var best = (Distance: double.PositiveInfinity, Y: 0d);

            for (var k = 1; k < samples.Length; k++)
            {
                var otherPoint = samples[k - 1];
                var d = samples[k] - otherPoint;
                var t =
                    math.clamp(math.dot(point.xz - otherPoint.xz, d.xz) / Math.Max(1e-12, math.lengthsq(d.xz)), 0, 1);

                var secondPoint = otherPoint + d * t;
                var distance = math.distance(secondPoint.xz, point.xz);

                if (distance < best.Distance)
                    best = (distance, secondPoint.y);
            }

            across = Math.Max(across, best.Distance);
            height = Math.Max(height, Math.Abs(best.Y - point.y));
        }

        error = (across, height);
        merged = firstSegment with
        {
            EndNodeId = secondSegment.EndNodeId,
            End = secondSegment.End,
            Control1 = (float3)curve.P1 - firstSegment.StartCurveOffset,
            Control2 = (float3)curve.P2 - secondSegment.EndCurveOffset,
            EndCurveOffset = secondSegment.EndCurveOffset,
            EndElevation = secondSegment.EndElevation,
            EndTerminal = secondSegment.EndTerminal,
            EndMiniRoundabout = secondSegment.EndMiniRoundabout,
            EndRoundaboutRadius = secondSegment.EndRoundaboutRadius,
            EndLevelCrossing = secondSegment.EndLevelCrossing,
            EndHeightFamily = secondSegment.EndHeightFamily,
            HorizontalEmissionReference = null,
            HorizontalParent = null,
            ParentStartT = 0,
            ParentEndT = 1,
            SourceWayIds =
                firstSegment.SourceWayIds
                    .Append(firstSegment.WayId)
                    .Concat(secondSegment.SourceWayIds.Append(secondSegment.WayId))
                    .Distinct()
                    .OrderBy(wayId => wayId)
                    .ToArray(),
            SourceNodeAliases =
                firstSegment.SourceNodeAliases.Concat(secondSegment.SourceNodeAliases).Distinct().ToArray(),
            SourceGeometry = firstSegment.SourceGeometry.Concat(secondSegment.SourceGeometry).ToArray(),
            OriginalGeometry = firstSegment.OriginalGeometry.Concat(secondSegment.OriginalGeometry).ToArray(),
            MaximumNativeWidth = Math.Min(firstSegment.MaximumNativeWidth, secondSegment.MaximumNativeWidth),
            ExplicitLanes = firstSegment.ExplicitLanes || secondSegment.ExplicitLanes
        };

        return true;
    }

    // A plain alias whose canonical node was joined into a run follows the
    // nearest emitted end, judged from the old node's position, so every alias
    // names a node that exists. Mandatory aliases are ports and never move.
    private static IReadOnlyList<PreparedSegment> RetargetPlainAliases(
        IReadOnlyList<PreparedSegment> source,
        IReadOnlyDictionary<SourceNodeKey, double2> position,
        ImportLog log)
    {
        var present = new HashSet<SourceNodeKey>();

        foreach (var segment in source)
        {
            present.Add(new(segment.Kind, segment.StartNodeId));
            present.Add(new(segment.Kind, segment.EndNodeId));
        }

        // A target still present on another section wins (distance -1), so one
        // source node never names two canonical nodes.
        var target = new Dictionary<SourceNodeKey, (long Id, double Distance)>();
        var stale = 0;

        foreach (var segment in source)
            foreach (var alias in segment.SourceNodeAliases)
            {
                var old = new SourceNodeKey(segment.Kind, alias.CanonicalNodeId);
                var key = new SourceNodeKey(segment.Kind, alias.SourceNodeId);

                if (alias.Mandatory)
                    continue;

                if (present.Contains(old))
                {
                    target[key] = (old.Id, -1);
                    continue;
                }

                stale++;
                var at = position.TryGetValue(old, out var point) ? point : ((float3)segment.CourseStart).xz;

                foreach (var (id, end) in new[] {
                    (segment.StartNodeId, ((float3)segment.CourseStart).xz),
                    (segment.EndNodeId, ((float3)segment.CourseEnd).xz)
                    })
                {
                    var otherDistance = math.distance(at, (double2)end);

                    if (!target.TryGetValue(key, out var best) || otherDistance < best.Distance)
                        target[key] = (id, otherDistance);
                }
            }

        if (stale == 0)
            return source;

        log.Count("Plain aliases moved to the nearest emitted node", stale);

        return source
            .Select(
                segment => !segment.SourceNodeAliases.Any(
                    alias => target.ContainsKey(new(segment.Kind, alias.SourceNodeId))
                        && !present.Contains(new(segment.Kind, alias.CanonicalNodeId)))
                    ? segment
                    :
            segment with
            {
                SourceNodeAliases =
                    segment.SourceNodeAliases
                        .Select(
                            alias => !alias.Mandatory
                                && !present.Contains(new(segment.Kind, alias.CanonicalNodeId))
                                && target.TryGetValue(new(segment.Kind, alias.SourceNodeId), out var item)
                ? alias with { CanonicalNodeId = item.Id } : alias).ToArray()
            }).ToArray();
    }

    // Where ground and elevated road pieces meet at one junction (an exit leaving
    // a ground motorway straight onto a viaduct, one ground lane splitting into
    // two elevated ones), the elevated pieces' railings and piers warp the
    // junction. Each elevated arm there keeps its first JunctionGround metres as
    // a ground piece, on its own slope, when at least that much bridge remains
    // beyond. Junctions of only elevated (or only ground) arms are unchanged.
    private const double JunctionGround = 24;

    private static IReadOnlyList<PreparedSegment> GroundAtJunctions(
        IReadOnlyList<PreparedSegment> source,
        ImportLog log,
        CancellationToken token)
    {
        var arms = new Dictionary<SourceNodeKey, List<(int Section, bool Start)>>();

        for (var i = 0; i < source.Count; i++)
            foreach (var start in new[] { true, false })
            {
                var key = new SourceNodeKey(source[i].Kind, start ? source[i].StartNodeId : source[i].EndNodeId);

                if (!arms.TryGetValue(key, out var list))
                    arms[key] = list = new();

                list.Add((i, start));
            }

        var cuts = new Dictionary<int, (bool AtStart, bool AtEnd)>();

        foreach (var node in arms)
        {
            token.ThrowIfCancellationRequested();
            var list = node.Value;

            if (node.Key.Kind != NetworkKind.Road || list.Count < 3 || list.Any(item => source[item.Section].Tunnel))
                continue;

            if (!list.Any(item => !source[item.Section].Bridge) || !list.Any(item => source[item.Section].Bridge))
                continue;

            foreach (var (i, start) in list)
            {
                var segment = source[i];
                var length = NetworkDesignChecks.HorizontalArcLength(NetworkDesignChecks.Course(segment));

                if (!segment.Bridge || length < 2 * JunctionGround)
                    continue;

                // Both ends need three ground lengths, or the two cuts nearly meet.
                cuts.TryGetValue(i, out var item);
                var both = start ? item.AtEnd : item.AtStart;

                if (both && length < 3 * JunctionGround)
                    continue;

                cuts[i] = start ? (true, item.AtEnd) : (item.AtStart, true);
            }
        }

        if (cuts.Count == 0)
            return source;

        var occupied =
            new HashSet<long>(source.SelectMany(segment => new[] { segment.StartNodeId, segment.EndNodeId }));

        var next = long.MinValue + 43000000;

        long Id()
        {
            while (occupied.Contains(next))
                next++;

            occupied.Add(next);

            return next++;
        }

        double At(PreparedSegment segment, double metres)
        {
            var curve = NetworkDesignChecks.Course(segment);
            var total = NetworkDesignChecks.HorizontalArcLength(curve);
            var walked = 0d;
            var previous = curve.P0.xz;

            for (var k = 1; k <= 200; k++)
            {
                var point = NetworkDesignChecks.Evaluate(curve, k / 200d).xz;
                walked += math.distance(previous, point);
                previous = point;

                if (walked >= metres)
                    return k / 200d;
            }

            return metres / Math.Max(1e-9, total);
        }

        var result = new List<PreparedSegment>();
        var grounded = 0;

        for (var i = 0; i < source.Count; i++)
        {
            var segment = source[i];

            if (!cuts.TryGetValue(i, out var item))
            {
                result.Add(segment);
                continue;
            }

            var rest = segment;

            if (item.AtStart)
            {
                var (left, right) = NativeDesignUnits.Split(rest, At(rest, JunctionGround), Id());
                result.Add(
                    left with { Bridge = false, FinalDeckCurve = null, StartElevation = null, EndElevation = null });

                rest = right with { FinalDeckCurve = null };
                grounded++;
            }

            if (item.AtEnd)
            {
                var keep = NetworkDesignChecks.HorizontalArcLength(NetworkDesignChecks.Course(rest)) - JunctionGround;
                var (left, right) = NativeDesignUnits.Split(rest, At(rest, keep), Id());
                result.Add(left with { FinalDeckCurve = null });
                rest = right with { Bridge = false, FinalDeckCurve = null, StartElevation = null, EndElevation = null };
                grounded++;
            }

            result.Add(rest);
        }

        log.Count("Elevated junction arms given a ground first piece", grounded);

        return result;
    }

    // An approach raised for clearance stayed a ground piece, and the game walls
    // in a ground road held well above the ground beside it (DC's walled edges
    // sat a median 7.3 m up, under low overpasses). From each bridge end
    // through plain two-arm nodes, the approach is built elevated while it
    // stands more than ElevateHeight above the original ground: whole sections,
    // and the last one split where it comes down (an approach above it for
    // under 5 m stays ground). Junctions, roundabouts and tunnels stop it.
    // With sink, the same from each tunnel end: an approach lowered into a
    // cutting before its portal (VisualCompilerProfile.PortalCut) is built as
    // tunnel while it lies more than SinkDepth below the original ground. Beside
    // the other carriageway's tube the roadbed may not cut, and the Vieux-Port
    // tunnel's northbound approach ran 15 m under the ground.
    private const float SinkDepth = 4;

    // Structure-depth seeds: sampled every SeedSpacing metres; deep runs closer than
    // SeedGap are joined, and a run shorter than SeedRun stays earthwork.
    private const double SeedSpacing = 5, SeedGap = 60, SeedRun = 40;

    // Square size (metres) of the side-by-side clearance grid in Prepare.
    private const double SideCell = 32;

    private static IReadOnlyList<PreparedSegment> ElevateApproaches(
        IReadOnlyList<PreparedSegment> source,
        float[] terrain,
        double width,
        ImportLog log, CancellationToken token, bool sink = false)
    {
        var list = source.ToList();
        var ends = new Dictionary<SourceNodeKey, List<int>>();

        for (var i = 0; i < list.Count; i++)
            foreach (var id in new[] { list[i].StartNodeId, list[i].EndNodeId })
            {
                var key = new SourceNodeKey(list[i].Kind, id);

                if (!ends.TryGetValue(key, out var arms))
                    ends[key] = arms = new();

                arms.Add(i);
            }

        float Ground(double3 point) => NetworkProcessor.Sample((float)point.x, (float)point.z, terrain, width);

        float Above(double3 point) => (float)point.y - Ground(point);

        // Sunk: below the lowest ground around the road out to its half width
        // plus 6 m (a tunnel asset is wider than its road), so a hillside
        // cutting does not become a tube with its downhill side open.
        float Clearance(PreparedSegment segment, double3 point)
        {
            if (!sink)
                return Above(point);

            var r = segment.NativeWidth / 2 + 6d;
            var low = Ground(point);

            for (var a = 0; a < 8; a++)
                low =
                    Math.Min(
                        low,
                        Ground(point + new double3(Math.Cos(a * Math.PI / 4) * r, 0, Math.Sin(a * Math.PI / 4) * r)));

            return low - (float)point.y;
        }

        var structureDepth = DesignSwitches.On(DesignSwitches.StructureDepth);
        var threshold = structureDepth ? NetworkVerticalProfiles.StructureDepth : sink ? SinkDepth : ElevateHeight;

        PreparedSegment Elevated(PreparedSegment segment)
            => sink ? segment with { Tunnel = true, Layer = Math.Min(segment.Layer, -1), FinalDeckCurve = null } :
            segment with
            {
                Bridge = true,
                FinalDeckCurve = null,
                StartElevation = Above(segment.CourseStart),
                EndElevation = Above(segment.CourseEnd)
            };

        var occupied = new HashSet<long>(list.SelectMany(segment => new[] { segment.StartNodeId, segment.EndNodeId }));
        var next = long.MinValue + (sink ? 44000000 : 41000000);
        var queue =
            list
                .Where(segment => sink ? segment.Tunnel : segment.Bridge && !segment.Tunnel)
                .SelectMany(
                    segment => new[] {
                        new SourceNodeKey(segment.Kind, segment.StartNodeId),
                        new SourceNodeKey(segment.Kind, segment.EndNodeId)
                        })
                .Distinct()
                .ToList();

        var queued = new HashSet<SourceNodeKey>(queue);
        var whole = 0;
        var split = 0;

        void Queue(NetworkKind kind, long id)
        {
            var key = new SourceNodeKey(kind, id);

            if (queued.Add(key))
                queue.Add(key);
        }

        long NewNode()
        {
            while (occupied.Contains(next))
                next++;

            occupied.Add(next);
            return next++;
        }

        // Structure depth (report 112, 11.4 B3): a ground stretch deeper than the
        // threshold for at least SeedRun metres becomes a structure wherever it is, not
        // only beside a mapped bridge or tunnel. Each section's longest deep run, after
        // joining runs less than SeedGap apart, is split out (no piece shorter than the
        // class minimum) and its ends queued, so the walk below carries it on through
        // plain nodes.
        var seeded = 0;

        if (structureDepth)
            for (int i = 0, count = list.Count; i < count; i++)
            {
                token.ThrowIfCancellationRequested();
                var segment = list[i];

                if (segment.Bridge || segment.Tunnel || segment.Roundabout || segment.StartNodeId == segment.EndNodeId)
                    continue;

                var curve = NetworkDesignChecks.Course(segment);
                var length = NetworkDesignChecks.HorizontalArcLength(curve);

                if (length < SeedRun)
                    continue;

                var samples = Math.Max(2, (int)Math.Ceiling(length / SeedSpacing));
                var runs = new List<(int First, int Last)>();

                for (var k = 0; k <= samples; k++)
                {
                    if (Clearance(segment, NetworkDesignChecks.Evaluate(curve, k / (double)samples)) <= threshold)
                        continue;

                    if (runs.Count > 0 && (k - runs[^1].Last) * length / samples < SeedGap)
                        runs[^1] = (runs[^1].First, k);
                    else
                        runs.Add((k, k));
                }

                if (runs.Count == 0)
                    continue;

                var (first, last) = runs.OrderByDescending(run => run.Last - run.First).First();
                var begin = first / (double)samples;
                var end = last / (double)samples;

                if ((end - begin) * length < SeedRun)
                    continue;

                var minimum = VisualCompilerTable.For(segment).MinimumSegment;

                if (begin * length < minimum)
                    begin = 0;

                if ((1 - end) * length < minimum)
                    end = 1;

                var endKey = new SourceNodeKey(segment.Kind, segment.EndNodeId);
                var endArms = ends[endKey];
                seeded++;

                if (begin == 0 && end == 1)
                {
                    list[i] = Elevated(segment);
                    Queue(segment.Kind, segment.StartNodeId);
                    Queue(segment.Kind, segment.EndNodeId);
                    continue;
                }

                if (begin == 0)
                {
                    var cut = NewNode();
                    var (raised, rest) = NativeDesignUnits.Split(segment, end, cut);
                    list[i] = Elevated(raised);
                    list.Add(rest with { FinalDeckCurve = null });
                    endArms[endArms.IndexOf(i)] = list.Count - 1;
                    ends[new SourceNodeKey(segment.Kind, cut)] = new List<int> { i, list.Count - 1 };
                    Queue(segment.Kind, segment.StartNodeId);
                    Queue(segment.Kind, cut);
                    continue;
                }

                var entry = NewNode();
                var (head, remainder) = NativeDesignUnits.Split(segment, begin, entry);
                list[i] = head with { FinalDeckCurve = null };
                ends[new SourceNodeKey(segment.Kind, entry)] = new List<int> { i, list.Count };

                if (end == 1)
                {
                    list.Add(Elevated(remainder));
                    endArms[endArms.IndexOf(i)] = list.Count - 1;
                    Queue(segment.Kind, entry);
                    Queue(segment.Kind, segment.EndNodeId);
                    continue;
                }

                // The remainder's parameter u covers [begin, 1] of the original curve.
                var exit = NewNode();
                var (middle, tail) = NativeDesignUnits.Split(remainder, (end - begin) / (1 - begin), exit);
                list.Add(Elevated(middle));
                list.Add(tail with { FinalDeckCurve = null });
                endArms[endArms.IndexOf(i)] = list.Count - 1;
                ends[new SourceNodeKey(segment.Kind, exit)] = new List<int> { list.Count - 2, list.Count - 1 };
                Queue(segment.Kind, entry);
                Queue(segment.Kind, exit);
            }

        for (var q = 0; q < queue.Count; q++)
        {
            token.ThrowIfCancellationRequested();
            var node = queue[q];

            if (ends[node].Count != 2)
                continue;

            foreach (var i in ends[node].ToArray())
            {
                var segment = list[i];

                if (segment.Bridge || segment.Tunnel || segment.Roundabout)
                    continue;

                var fromStart = segment.StartNodeId == node.Id;
                var curve = NetworkDesignChecks.Course(segment);
                var length = NetworkDesignChecks.HorizontalArcLength(curve);
                var n = Math.Max(2, (int)Math.Ceiling(length / 2));
                var down = -1;

                for (var k = 0; k <= n; k++)
                    if (Clearance(
                        segment,
                        NetworkDesignChecks.Evaluate(curve, fromStart ? k / (double)n : 1 - k / (double)n))
                        < threshold)
                    {
                        down = k;
                        break;
                    }

                if (down == 0 || down > 0 && down * length / n < 5)
                    continue;

                // Neither piece shorter than the class's shortest section: the
                // elevated piece runs on a little lower, or takes the whole section.
                // A tube is never stretched past where the ground covers it: a
                // sunk piece too short, or leaving too short a remainder, stays ground.
                var minimum = VisualCompilerTable.For(segment).MinimumSegment;

                if (sink && down > 0 && (down * length / n < minimum || (n - down) * length / n < minimum))
                    continue;

                if (down > 0)
                    down = Math.Max(down, (int)Math.Ceiling(minimum / length * n));

                if (down < 0 || down >= n || (n - down) * length / n < minimum)
                {
                    list[i] = Elevated(segment);
                    whole++;
                    var far = new SourceNodeKey(segment.Kind, fromStart ? segment.EndNodeId : segment.StartNodeId);

                    if (queued.Add(far))
                        queue.Add(far);

                    continue;
                }

                while (occupied.Contains(next))
                    next++;

                var cut = next++;
                occupied.Add(cut);
                var (left, right) =
                    NativeDesignUnits.Split(segment, fromStart ? down / (double)n : 1 - down / (double)n, cut);

                var (raised, ground) = fromStart ? (left, right) : (right, left);
                list[i] = Elevated(raised);
                list.Add(ground with { FinalDeckCurve = null });
                split++;
                var farNode = new SourceNodeKey(segment.Kind, fromStart ? segment.EndNodeId : segment.StartNodeId);
                var farArms = ends[farNode];
                farArms[farArms.IndexOf(i)] = list.Count - 1;
                ends[new SourceNodeKey(segment.Kind, cut)] = new List<int> { i, list.Count - 1 };
            }
        }

        if (structureDepth)
            log.Count(sink ? "Deep cuttings built as tunnel" : "High fills built as bridge", seeded);

        if (sink)
        {
            log.Count("Tunnel approach sections sunk", whole);
            log.Count("Tunnel approaches split where they come up", split);
        }
        else
        {
            log.Count("Bridge approach sections elevated", whole);
            log.Count("Bridge approaches split where they come down", split);
        }

        return list;
    }

    // Metres from a road tunnel's portal within which ground samples take no crest
    // floor: the road dives under the dyke there. Lifting the approach ramps onto the
    // crest dragged the whole tunnel up with them (Amsterdam: 10 m).
    private const double TunnelApproachMeters = 100;

    // Road tunnel ends, bucketed by TunnelApproachMeters squares.
    private static Dictionary<(int, int), List<double2>> TunnelPortals(IReadOnlyList<PreparedSegment> segments)
    {
        var portals = new Dictionary<(int, int), List<double2>>();

        foreach (var segment in segments)
            if (segment.Tunnel && segment.Kind == NetworkKind.Road)
                foreach (var end in new[] { ((float3)segment.CourseStart).xz, ((float3)segment.CourseEnd).xz })
                {
                    var cell = PortalCell(end);

                    if (!portals.TryGetValue(cell, out var list))
                        portals[cell] = list = new();

                    list.Add(end);
                }

        return portals;
    }

    private static (int, int) PortalCell(double2 point)
        => ((int)Math.Floor(point.x / TunnelApproachMeters), (int)Math.Floor(point.y / TunnelApproachMeters));

    private static bool NearTunnelPortal(Dictionary<(int, int), List<double2>> portals, double2 point)
    {
        var (cx, cz) = PortalCell(point);

        for (var dz = -1; dz <= 1; dz++)
            for (var dx = -1; dx <= 1; dx++)
                if (portals.TryGetValue((cx + dx, cz + dz), out var list)
                    && list.Any(portal => math.distance(portal, point) <= TunnelApproachMeters))
                    return true;

        return false;
    }

    // The highest flood-defence crest (metres) under a road's footprint at a point: cells
    // whose centres lie within the half width plus half a cell. NaN when it crosses none.
    // A wider square reach lifted roads and tunnel portals running beside a dyke onto
    // its crest (Amsterdam: 10 m tunnel lifts, +10 km of paired-carriageway failures).
    internal static float HighestCrest(float[] crests, float3 point, float halfWidth, double width)
    {
        var n = (int)Math.Sqrt(crests.Length);
        var cell = width / (n - 1);
        var radius = halfWidth + cell / 2;
        var reach = (int)Math.Ceiling(radius / cell);
        var gridX = (point.x / width + .5) * (n - 1);
        var gridZ = (point.z / width + .5) * (n - 1);
        var cx = (int)Math.Round(gridX);
        var cz = (int)Math.Round(gridZ);
        var limit = radius / cell * (radius / cell);
        var highest = float.NaN;

        for (var z = Math.Max(0, cz - reach); z <= Math.Min(n - 1, cz + reach); z++)
            for (var x = Math.Max(0, cx - reach); x <= Math.Min(n - 1, cx + reach); x++)
            {
                var dx = x - gridX;
                var dz = z - gridZ;
                var crest = crests[z * n + x];

                // A NaN crest (none) never compares greater, so it never replaces one.
                if (dx * dx + dz * dz <= limit && (float.IsNaN(highest) || crest > highest))
                    highest = crest;
            }

        return highest;
    }

    // The sea (report 112, 11.4 W1): ground below sea level that is connected to the
    // map edge through ground below sea level. Polders lie below the sea too, but
    // behind dykes above it, so they are not reached; their roads were lifted onto
    // embankments as if they stood in the sea (Amsterdam's 418 million m³ of fill).
    // Mapped rivers and lakes come through the water sampler, not this.
    private static bool[] SeaFromEdge(float[] terrain, float sea, ImportLog log)
    {
        var n = (int)Math.Sqrt(terrain.Length);
        var result = new bool[terrain.Length];
        // Cells still to spread from, taken from the end.
        var pending = new List<int>();

        void Visit(int x, int z)
        {
            var id = z * n + x;

            if (!result[id] && terrain[id] < sea)
            {
                result[id] = true;
                pending.Add(id);
            }
        }

        for (var i = 0; i < n; i++)
        {
            Visit(i, 0);
            Visit(i, n - 1);
            Visit(0, i);
            Visit(n - 1, i);
        }

        while (pending.Count > 0)
        {
            var id = pending[^1];
            pending.RemoveAt(pending.Count - 1);
            int x = id % n, z = id / n;

            if (x > 0)
                Visit(x - 1, z);

            if (x < n - 1)
                Visit(x + 1, z);

            if (z > 0)
                Visit(x, z - 1);

            if (z < n - 1)
                Visit(x, z + 1);
        }

        var below = terrain.Count(value => value < sea);
        log.Diagnostic(
            $"VISUAL COMPILER sea from edge: {result.Count(cell => cell):N0} of {below:N0} cells below sea level reach the map edge");

        return result;
    }

    private static bool IsSea(bool[] cells, float3 point, double width)
    {
        var n = (int)Math.Sqrt(cells.Length);
        var x = (int)math.clamp(Math.Round((point.x / width + .5) * (n - 1)), 0, n - 1);
        var z = (int)math.clamp(Math.Round((point.z / width + .5) * (n - 1)), 0, n - 1);

        return cells[z * n + x];
    }

    // Opposite carriageways of one divided road closer than half their widths
    // plus a 2 m median are pushed apart symmetrically, each by half the
    // shortfall (at most 4 m), at their nodes. A node closer than 1 m to its
    // partner is where the carriageways meet and stays put, as does a level
    // crossing node, which the track holds (Amsterdam, report 121 §19).
    private static Dictionary<SourceNodeKey, double2> Separate(
        IReadOnlyList<PreparedSegment> source,
        bool[] contracted,
        Func<int, List<double2>> curve,
        Func<SourceNodeKey, SourceNodeKey> canon,
        HashSet<long> levelCrossings,
        CancellationToken token)
    {
        bool Candidate(PreparedSegment segment)
            => segment.Kind == NetworkKind.Road
                && segment.OneWay
                && !segment.ProtectedLink
                && !segment.Roundabout
                && VisualCompilerTable.For(segment).PairSeparation > 0;

        var segments = new Dictionary<(int, int), List<(int Section, double2 A, double2 B)>>();

        for (var i = 0; i < source.Count; i++)
            if (!contracted[i] && Candidate(source[i]))
            {
                var points = curve(i);

                for (var k = 1; k < points.Count; k++)
                {
                    var item = PairCell((points[k - 1] + points[k]) / 2);

                    if (!segments.TryGetValue(item, out var items))
                        segments[item] = items = new();

                    items.Add((i, points[k - 1], points[k]));
                }
            }

        var shift = new Dictionary<SourceNodeKey, double2>();

        for (var i = 0; i < source.Count; i++)
        {
            var segment = source[i];

            if (contracted[i] || !Candidate(segment))
                continue;

            token.ThrowIfCancellationRequested();
            var direction = math.normalizesafe(((float3)segment.CourseEnd).xz - ((float3)segment.CourseStart).xz);

            foreach (var (node, point) in new[] {
                (canon(new(segment.Kind, segment.StartNodeId)), (double2)((float3)segment.CourseStart).xz),
                (canon(new(segment.Kind, segment.EndNodeId)), (double2)((float3)segment.CourseEnd).xz)
                })
            {
                if (levelCrossings.Contains(node.Id))
                    continue;

                var best = (Distance: double.PositiveInfinity, Closest: default(double2), Width: 0f);
                var item = PairCell(point);

                for (var x = item.Item1 - 1; x <= item.Item1 + 1; x++)
                    for (var z = item.Item2 - 1; z <= item.Item2 + 1; z++)
                        if (segments.TryGetValue((x, z), out var list))
                            foreach (var (j, vectorA, vectorB) in list)
                            {
                                var otherSegment = source[j];

                                if (j == i
                                    || otherSegment.Classification != segment.Classification
                                    || otherSegment.Layer != segment.Layer
                                    || math.dot(math.normalizesafe(vectorB - vectorA), direction) > -.9)
                                    continue;

                                var t =
                                    math.clamp(
                                        math.dot(point - vectorA, vectorB - vectorA)
                                            / Math.Max(1e-9, math.lengthsq(vectorB - vectorA)),
                                        0,
                                        1);

                                var otherPoint = vectorA + (vectorB - vectorA) * t;
                                var otherDistance = math.distance(point, otherPoint);

                                if (otherDistance < best.Distance)
                                    best = (otherDistance, otherPoint, otherSegment.NativeWidth);
                            }

                var required = (segment.NativeWidth + best.Width) / 2 + 2;

                if (best.Distance < 1
                    || best.Distance >= required
                    || best.Distance > VisualCompilerTable.For(segment).PairSeparation)
                    continue;

                var push = math.normalizesafe(point - best.Closest) * Math.Min(4, (required - best.Distance) / 2);

                if (!shift.TryGetValue(node, out var old) || math.lengthsq(push) > math.lengthsq(old))
                    shift[node] = push;
            }
        }

        return shift;
    }

    private static double2 Evaluate(Piece piece, double t)
    {
        var u = 1 - t;
        return u * u * u * piece.P0 + 3 * u * u * t * piece.P1 + 3 * u * t * t * piece.P2 + t * t * t * piece.P3;
    }

    // Whether no point of a piece can be nearer a target than distance. A cubic lies in
    // the convex hull of its control points, so in their bounding box; the millimetre
    // allows for rounding in Evaluate, which is many orders of magnitude smaller.
    private static bool PieceBeyond(Piece piece, double2 target, double distance)
    {
        var low = math.min(math.min(piece.P0, piece.P1), math.min(piece.P2, piece.P3));
        var high = math.max(math.max(piece.P0, piece.P1), math.max(piece.P2, piece.P3));
        var gap = math.max(double2.zero, math.max(low - target, target - high));

        return math.length(gap) >= distance + .001;
    }

    private static (Piece Left, Piece Right) Split(Piece piece, double t)
    {
        var vectorA = math.lerp(piece.P0, piece.P1, t);
        var vectorB = math.lerp(piece.P1, piece.P2, t);
        var d = math.lerp(piece.P2, piece.P3, t);
        var e = math.lerp(vectorA, vectorB, t);
        var f = math.lerp(vectorB, d, t);
        var m = math.lerp(e, f, t);

        return (new(piece.P0, vectorA, e, m), new(m, f, d, piece.P3));
    }

    // Closest point to a target near a coarse sample: golden-section search on
    // the sampled piece and its neighbours, which handles boundary minima.
    private static (int Piece, double T, double Distance) Refine(
        List<Piece> pieces,
        double2 target,
        int piece,
        double t)
    {
        var best = (Piece: piece, T: t, Distance: math.distance(Evaluate(pieces[piece], t), target));

        for (var p = Math.Max(0, piece - 1); p <= Math.Min(pieces.Count - 1, piece + 1); p++)
        {
            double low = p == piece ? Math.Max(0, t - 1 / 32d) : 0, high = p == piece ? Math.Min(1, t + 1 / 32d) : 1;
            const double g = .6180339887498949;

            for (var i = 0; i < 60; i++)
            {
                var a = high - g * (high - low);
                var b = low + g * (high - low);

                if (math.distance(Evaluate(pieces[p], a), target) < math.distance(Evaluate(pieces[p], b), target))
                    high = b;
                else
                    low = a;
            }

            var u = (low + high) / 2;
            var otherDistance = math.distance(Evaluate(pieces[p], u), target);

            if (otherDistance < best.Distance)
                best = (p, u, otherDistance);
        }

        return best;
    }

    private static double Length(Piece piece)
    {
        var l = 0d;
        var point = piece.P0;

        for (var k = 1; k <= 16; k++)
        {
            var otherPoint = Evaluate(piece, k / 16d);
            l += math.distance(point, otherPoint);
            point = otherPoint;
        }

        return l;
    }

    private static double[] Stations(List<Piece> pieces)
    {
        var s = new double[pieces.Count + 1];

        for (var k = 0; k < pieces.Count; k++)
            s[k + 1] = s[k] + Length(pieces[k]);

        return s;
    }

    private static double2 At(List<Piece> pieces, double[] stations, double s)
    {
        var k = Array.BinarySearch(stations, s);

        if (k < 0)
            k = ~k - 1;

        k = Math.Max(0, Math.Min(pieces.Count - 1, k));

        return Evaluate(pieces[k], math.clamp((s - stations[k]) / Math.Max(1e-9, stations[k + 1] - stations[k]), 0, 1));
    }

    // Circular arc from a tangent point, split into parts of at most 90 degrees;
    // handle length (4/3)·tan(φ/4)·R gives radial error ≈ 2.7e-4·R at 90°.
    private static void Arc(List<Piece> pieces, double2 start, double2 direction, double radius, double turn)
    {
        var side = turn > 0 ? 1 : -1;
        var normal = new double2(-direction.y, direction.x) * side;
        var center = start + normal * radius;
        var parts = Math.Max(1, (int)Math.Ceiling(Math.Abs(turn) / (Math.PI / 2)));
        var step = turn / parts;
        var angle = Math.Atan2(start.y - center.y, start.x - center.x);
        var k = 4 / 3d * Math.Tan(Math.Abs(step) / 4) * radius;

        for (var p = 0; p < parts; p++)
        {
            var a0 = angle + p * step;
            var a1 = a0 + step;
            var q0 = center + radius * new double2(Math.Cos(a0), Math.Sin(a0));
            var q3 = center + radius * new double2(Math.Cos(a1), Math.Sin(a1));
            var t0 = side * new double2(-Math.Sin(a0), Math.Cos(a0));
            var t3 = side * new double2(-Math.Sin(a1), Math.Cos(a1));
            pieces.Add(new(q0, q0 + t0 * k, q3 - t3 * k, q3));
        }
    }

    // The pieces cut again at the stations that divide their whole length into equal
    // parts. `ends` receives the index of the last piece of each part.
    private static List<(Piece Piece, double S0, double S1)> CutEvenly(
        List<(Piece Piece, double S0, double S1)> pieces,
        int parts,
        List<int> ends)
    {
        var result = new List<(Piece Piece, double S0, double S1)>();
        var start = pieces[0].S0;
        var step = (pieces[pieces.Count - 1].S1 - start) / parts;
        var next = 1;

        foreach (var item in pieces)
        {
            var rest = item;

            while (next < parts && start + next * step < rest.S1 - .01)
            {
                var boundary = start + next * step;

                if (boundary > rest.S0 + .01)
                {
                    var (left, right) = Split(rest.Piece, (boundary - rest.S0) / (rest.S1 - rest.S0));
                    result.Add((left, rest.S0, boundary));
                    rest = (right, boundary, rest.S1);
                }

                ends.Add(result.Count - 1);
                next++;
            }

            result.Add(rest);
        }

        ends.Add(result.Count - 1);

        return result;
    }

    // The pieces of a queue, in order, without their stations.
    private static List<Piece> PiecesOf(List<(Piece Piece, double S0, double S1)> queue)
    {
        var pieces = new List<Piece>(queue.Count);

        foreach (var item in queue)
            pieces.Add(item.Piece);

        return pieces;
    }

    // Total turn (radians) along pieces [a,b] that join without a kink.
    private static double Turning(List<Piece> pieces, int a, int b)
    {
        var total = 0d;

        for (var p = a; p <= b; p++)
        {
            var leaving = math.normalizesafe(pieces[p].P1 - pieces[p].P0);
            var arriving = math.normalizesafe(pieces[p].P3 - pieces[p].P2);
            total += Math.Abs(Math.Atan2(NetworkDesignChecks.Cross(leaving, arriving), math.dot(leaving, arriving)));
        }

        return total;
    }

    // One cubic over pieces [a,b] with the end points and end tangents fixed;
    // handle lengths by least squares on arc-length samples (Schneider 1990).
    private static bool Fit(List<Piece> pieces, int a, int b, double tolerance, out Piece result)
    {
        result = default;
        var samples = new List<double2>();

        for (var p = a; p <= b; p++)
            for (var q = p == a ? 0 : 1; q <= 12; q++)
                samples.Add(Evaluate(pieces[p], q / 12d));

        var u = new double[samples.Count];

        for (var k = 1; k < samples.Count; k++)
            u[k] = u[k - 1] + math.distance(samples[k - 1], samples[k]);

        var total = u[u.Length - 1];

        if (total <= 1e-6)
            return false;

        for (var k = 0; k < u.Length; k++)
            u[k] /= total;

        var p0 = pieces[a].P0;
        var p3 = pieces[b].P3;
        var t0 = math.normalizesafe(pieces[a].P1 - pieces[a].P0);
        var t3 = math.normalizesafe(pieces[b].P3 - pieces[b].P2);
        double c11 = 0, c12 = 0, c22 = 0, x1 = 0, x2 = 0;

        for (var k = 0; k < samples.Count; k++)
        {
            var s = u[k];
            var m = 1 - s;
            var b0 = m * m * m;
            var b1 = 3 * m * m * s;
            var b2 = 3 * m * s * s;
            var b3 = s * s * s;
            var a1 = b1 * t0;
            var a2 = -b2 * t3;
            var r = samples[k] - (b0 + b1) * p0 - (b2 + b3) * p3;
            c11 += math.dot(a1, a1);
            c12 += math.dot(a1, a2);
            c22 += math.dot(a2, a2);
            x1 += math.dot(a1, r);
            x2 += math.dot(a2, r);
        }

        var det = c11 * c22 - c12 * c12;

        if (Math.Abs(det) < 1e-12)
            return false;

        var alpha = (x1 * c22 - x2 * c12) / det;
        var beta = (c11 * x2 - c12 * x1) / det;
        var chord = math.distance(p0, p3);

        if (!(alpha > chord * .05 && beta > chord * .05 && alpha < chord && beta < chord))
            return false;

        var fitted = new Piece(p0, p0 + alpha * t0, p3 - beta * t3, p3);

        for (var k = 0; k < samples.Count; k++)
            if (math.distance(Evaluate(fitted, u[k]), samples[k]) > tolerance)
                return false;

        result = fitted;

        return true;
    }

    private static void Simplify(List<double2> line, int first, int last, double epsilon, bool[] keep)
    {
        var stack = new List<(int A, int B)> { (first, last) };

        while (stack.Count > 0)
        {
            var (a, b) = stack[stack.Count - 1];
            stack.RemoveAt(stack.Count - 1);

            if (b <= a + 1)
                continue;

            var worst = -1;
            var distance = epsilon;
            var d = line[b] - line[a];
            var length = math.length(d);

            for (var k = a + 1; k < b; k++)
            {
                var v = line[k] - line[a];
                var e = length < 1e-9 ? math.length(v) : Math.Abs(NetworkDesignChecks.Cross(d, v)) / length;

                if (e > distance)
                {
                    distance = e;
                    worst = k;
                }
            }

            if (worst < 0)
                continue;

            keep[worst] = true;
            stack.Add((a, worst));
            stack.Add((worst, b));
        }
    }

    private static bool IsGroundStreet(PreparedSegment segment) => segment.Kind == NetworkKind.Road
                    && !NativeAssetKeys.IsHighway(segment)
                    && !segment.ProtectedLink
                    && !segment.Roundabout
                    && !NetworkStructure.IsStructure(segment);

    internal static bool IsMainline(PreparedSegment segment) => segment.Classification is "motorway" or "trunk";

    internal static (int, int) PairCell(double2 point)
        => ((int)Math.Floor(point.x / 64), (int)Math.Floor(point.y / 64));
}
