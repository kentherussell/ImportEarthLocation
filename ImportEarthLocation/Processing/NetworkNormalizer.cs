using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Collections.Concurrent;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

public static class NetworkNormalizer
{
    // Run after all consolidation/roundabout fitting: distinct OSM ways can
    // become the same native course only after those transformations.
    public static IReadOnlyList<PreparedSegment> RemoveCoincidentCourses(
        IReadOnlyList<PreparedSegment> source,
        ImportLog log,
        CancellationToken token)
    {
        var result = new List<PreparedSegment>();
        var buckets = new Dictionary<string, List<int>>();

        foreach (var segment in source)
        {
            token.ThrowIfCancellationRequested();
            var a = segment.OneWay ? segment.StartNodeId : Math.Min(segment.StartNodeId, segment.EndNodeId);
            var b = segment.OneWay ? segment.EndNodeId : Math.Max(segment.StartNodeId, segment.EndNodeId);
            var family =
                segment.Classification is "motorway" or "motorway_link" or "trunk" or "trunk_link"
                    ? "highway"
                    : segment.Kind == NetworkKind.Rail ? segment.Classification : "street";

            var key =
                $"{segment.Kind}:{family}:{segment.Lanes}:{segment.OneWay}:{segment.Layer}:{segment.Bridge}:{segment.Tunnel}:{a}:{b}";

            if (!buckets.TryGetValue(key, out var candidates))
                buckets[key] = candidates = new();

            var match = -1;

            foreach (var index in candidates)
            {
                var other = result[index];
                var reversed = segment.StartNodeId != other.StartNodeId;

                if (Math.Abs(segment.NativeWidth - other.NativeWidth) > .05f)
                    continue;

                if (Enumerable
                    .Range(0, 33)
                    .All(
                        j => math.distance(Evaluate(segment, j / 32f), Evaluate(other, (reversed ? 32 - j : j) / 32f))
                            < .5f))
                {
                    match = index;
                    break;
                }
            }

            if (match < 0)
            {
                candidates.Add(result.Count);
                result.Add(segment);
                continue;
            }

            var old = result[match];
            var keeper = segment.Roundabout && !old.Roundabout ? segment : old;
            result[match] = keeper with
            {
                SourceWayIds =
                    old.SourceWayIds
                        .Concat(segment.SourceWayIds)
                        .Append(old.WayId)
                        .Append(segment.WayId)
                        .Distinct()
                        .ToArray(),
                SourceNodeAliases = old.SourceNodeAliases.Concat(segment.SourceNodeAliases).Distinct().ToArray()
            };

            log.Count("Final coincident courses coalesced");
            log.Diagnostic(
                $"NETWORK COINCIDENT COURSE retainedWay={keeper.WayId}; aliasWay={segment.WayId}; ports={a}/{b}; toleranceMeters=0.5; sourceAliasesRetained=true");
        }

        return result;
    }

    public static float Tolerance(NetworkKind kind) => kind == NetworkKind.Rail ? 3f : 2f;

    // Native placement shares a real source node through bridge/tunnel portals.
    // Count all incident same-kind edges, including other structure groups, so
    // merging a ground continuation cannot erase a structural branch junction.
    private static SourceNodeKey NodeKey(PreparedSegment segment, long node) => new(segment.Kind, node);

    public static IReadOnlyList<PreparedSegment> Normalize(
        IReadOnlyList<PreparedSegment> input,
        ImportLog log,
        CancellationToken token,
        double mapWidth = double.PositiveInfinity,
        IDictionary<SourceNodeKey, long>? merged = null)
    {
        // Each step's time, from the end of the previous step, as a Stopwatch timestamp.
        var stepStarted = Stopwatch.GetTimestamp();

        void Step(string name)
        {
            var now = Stopwatch.GetTimestamp();
            log.AddDuration("Normalization " + name, (now - stepStarted) / (double)Stopwatch.Frequency);
            stepStarted = now;
        }

        input = NormalizeLaneRuns(input, log, token);
        Step("lane runs");

        var clock = Stopwatch.StartNew();
        log.Diagnostic(
            $"PROCESS normalization begin; sourceSections={input.Count}; junctions/sharp-turns/levels protected=true");

        var degree = new Dictionary<SourceNodeKey, int>();

        foreach (var segment in input)
        {
            foreach (var id in new[] { segment.StartNodeId, segment.EndNodeId })
            {
                var key = NodeKey(segment, id);
                degree.TryGetValue(key, out var count);
                degree[key] = count + 1;
            }
        }

        foreach (var segment in input)
        {
            foreach (var alias in segment.SourceNodeAliases.Where(alias => alias.Mandatory))
            {
                var key = NodeKey(segment, alias.CanonicalNodeId);

                if (degree.ContainsKey(key))
                    degree[key] += 100;
            }

            if (segment.StartMiniRoundabout)
                degree[NodeKey(segment, segment.StartNodeId)] += 100;

            if (segment.EndMiniRoundabout)
                degree[NodeKey(segment, segment.EndNodeId)] += 100;
        }

        log.Time("Topology/cleanup", clock.Elapsed.TotalSeconds);
        Step("node degrees");

        clock.Restart();
        // OSM way boundaries are metadata boundaries, not mandatory native nodes.
        // Merge compatible degree-two continuations, retaining every real junction.
        var groups = input.GroupBy(segment => (Route: segment.Reference,
            segment.Kind,
                segment.Classification,
                segment.Lanes,
                segment.OneWay,
                segment.Bridge,
                segment.Tunnel,
                segment.Layer,
            segment.Roundabout,
                segment.ExplicitCutSides,
                segment.ExplicitWallSides,
                segment.RailService,
                segment.ForwardLanes,
                segment.BackwardLanes))
                .ToArray();

        // Groups share no sections and are concatenated in order, so the result
        // does not depend on scheduling. A node merged away is recorded, so a source
        // port there keeps its identity as an alias of the merged section's start.
        // Each group either passes its sections on in outputs or lists the pieces to fit.
        var outputs = new List<PreparedSegment>[groups.Length];
        var groupPieces = new List<(List<float3> Points, List<PreparedSegment> Sections)>[groups.Length];
        var mergedNodes = new ConcurrentDictionary<SourceNodeKey, long>();
        var protectedRamps = 0;
        var options = new ParallelOptions { CancellationToken = token };
        var fitted = Array.Empty<List<PreparedSegment>>();

        try
        {
            Parallel.For(0, groups.Length, options, g =>
                    {
                        var group = groups[g];
                        var output = outputs[g] = new List<PreparedSegment>();
                        var pieces = groupPieces[g] = new List<(List<float3>, List<PreparedSegment>)>();

                        // RoundaboutPlanner has already fitted approved circular arcs. Endpoint-only
                        // refitting would erase their handles and turn the ring into a polygon.
                        if (group.First().Roundabout
                            && group.All(segment => segment.Control1.HasValue && segment.Control2.HasValue))
                        {
                            output.AddRange(group);
                            return;
                        }

                        // A ramp is protected topology, not necessarily a sequence of straight edges.
                        // Retain tiny links; longer chains use the same junction-aware bounded fit.
                        if (group.First().ProtectedLink
                            && group.Sum(segment => math.distance(segment.Start.xz, segment.End.xz)) < 12)
                        {
                            foreach (var segment in group)
                            {
                                token.ThrowIfCancellationRequested();
                                output.Add(segment with { SourceGeometry = new[] { segment.Start, segment.End } });
                            }

                            Interlocked.Add(ref protectedRamps, group.Count());

                            return;
                        }

                        var chain = new List<PreparedSegment>();

                        void Flush()
                        {
                            if (chain.Count == 0)
                                return;

                            var piece = new List<PreparedSegment>();
                            var length = 0f;

                            void Emit()
                            {
                                if (piece.Count == 0)
                                    return;

                                var points = new List<float3> { piece[0].Start };
                                points.AddRange(piece.Select(segment => segment.End));

                                for (var k = 1; k < piece.Count; k++)
                                    mergedNodes[new(piece[k].Kind, piece[k].StartNodeId)] = piece[0].StartNodeId;

                                pieces.Add((points, piece));
                                piece = new List<PreparedSegment>();
                                length = 0;
                            }

                            foreach (var segment in chain)
                            {
                                if (piece.Count > 0)
                                {
                                    var previous = piece[piece.Count - 1];

                                    if (degree[NodeKey(segment, segment.StartNodeId)] != 2
                                        || math.dot(
                                            math.normalizesafe(previous.End - previous.Start),
                                            math.normalizesafe(segment.End - segment.Start))
                                            < (segment.Kind == NetworkKind.Rail
                                                || segment.Classification == "motorway"
                                                || segment.Classification == "trunk"
                                                ? .5f
                                                : .707f)
                                            || length
                                            > (segment.Kind == NetworkKind.Rail
                                                || segment.Classification == "motorway"
                                                || segment.Classification == "trunk"
                                                ? 400f
                                                : 200f)
                                            || piece.Count >= 256)
                                        Emit();
                                }

                                piece.Add(segment);
                                length += math.distance(segment.Start, segment.End);
                            }

                            Emit();
                            chain.Clear();
                        }

                        foreach (var segment in OrderSections(group, degree))
                        {
                            token.ThrowIfCancellationRequested();

                            if (chain.Count > 0 && chain[0].StartNodeId == segment.EndNodeId && segment.OneWay)
                                chain.Insert(0, segment);
                            else if (chain.Count == 0 || chain[chain.Count - 1].EndNodeId == segment.StartNodeId)
                                chain.Add(segment);
                            else
                            {
                                Flush();
                                chain.Add(segment);
                            }
                        }

                        Flush();
                    });

            // Fitting is most of the work and one group can hold most of the network, so the
            // pieces of all groups are fitted in parallel. Fit reads only its own piece, and
            // each piece keeps its own output, joined below in group and piece order.
            var allPieces = groupPieces.SelectMany(pieces => pieces).ToArray();
            fitted = new List<PreparedSegment>[allPieces.Length];

            Parallel.For(0, allPieces.Length, options, p =>
            {
                fitted[p] = new List<PreparedSegment>();
                Fit(allPieces[p].Points, allPieces[p].Sections, fitted[p], token, (float)(mapWidth / 2));
            });
        }
        catch (AggregateException exception) when (exception.InnerExceptions.All(
            inner => inner is OperationCanceledException))
        {
            token.ThrowIfCancellationRequested();
            throw;
        }

        var output = new List<PreparedSegment>();
        var fittedPiece = 0;

        for (var g = 0; g < groups.Length; g++)
        {
            output.AddRange(outputs[g]);

            for (var k = 0; k < groupPieces[g].Count; k++)
                output.AddRange(fitted[fittedPiece++]);
        }

        Step("merging and fitting");

        log.Count("Protected ramps preserved", protectedRamps);
        log.Count("Segments merged", Math.Max(0, input.Count - output.Count));
        var unique = new List<PreparedSegment>();
        var equivalents = new Dictionary<string, List<int>>(StringComparer.Ordinal);

        foreach (var segment in output)
        {
            token.ThrowIfCancellationRequested();
            var a = segment.OneWay ? segment.StartNodeId : Math.Min(segment.StartNodeId, segment.EndNodeId);
            var b = segment.OneWay ? segment.EndNodeId : Math.Max(segment.StartNodeId, segment.EndNodeId);
            var key =
                $"{segment.Kind}:{segment.Classification}:{segment.Lanes}:{segment.OneWay}:{segment.Bridge}:{segment.Tunnel}:{segment.Layer}:{a}:{b}";

            if (!equivalents.TryGetValue(key, out var candidates))
                equivalents[key] = candidates = new List<int>();

            // Only equivalent topology endpoints AND nearly coincident curves qualify.
            // Parallel tracks/carriageways with different OSM node IDs never qualify.
            var duplicate = -1;

            foreach (var index in candidates)
            {
                var other = unique[index];

                if (Enumerable.Range(0, 33).All(i => math.distance(Evaluate(segment, i / 32f).xz,
                    Evaluate(other, (segment.StartNodeId == other.StartNodeId ? i : 32 - i) / 32f).xz) < .05f))
                {
                    duplicate = index;
                    break;
                }
            }

            if (duplicate >= 0)
            {
                // Equivalent geometry is one physical demand, not permission
                // to erase the source routes which the retained course carries.
                var retained = unique[duplicate];
                unique[duplicate] = retained with
                {
                    SourceWayIds =
                        retained.SourceWayIds
                            .Concat(segment.SourceWayIds)
                            .Concat(new[] { retained.WayId, segment.WayId })
                            .Distinct()
                            .ToArray(),
                    SourceNodeAliases =
                        retained.SourceNodeAliases.Concat(segment.SourceNodeAliases).Distinct().ToArray(),
                    OriginalGeometry = retained.OriginalGeometry.Concat(segment.OriginalGeometry).ToArray()
                };

                log.Count("Duplicates/artifacts removed");
                continue;
            }

            candidates.Add(unique.Count);
            unique.Add(segment);
        }

        Step("duplicates");

        log.Time("Smoothing", clock.Elapsed.TotalSeconds);
        SmoothJoins(unique, log, token, (float)(mapWidth / 2));
        Step("tangent continuity");

        log.Diagnostic(
            $"PROCESS normalization complete; sourceSections={input.Count}; outputSections={unique.Count}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.0}");

        if (merged != null)
            foreach (var pair in mergedNodes)
                merged[pair.Key] = pair.Value;

        return unique;
    }

    private static IEnumerable<PreparedSegment> OrderSections(
        IEnumerable<PreparedSegment> source,
        Dictionary<SourceNodeKey, int> degree)
    {
        var items =
            source
                .OrderBy(
                    segment => segment.OneWay ? segment.StartNodeId : Math.Min(segment.StartNodeId, segment.EndNodeId))
                .ThenBy(segment => segment.WayId)
            .ThenBy(segment => segment.StartNodeId).ThenBy(segment => segment.EndNodeId).ToArray();

        // Items are only ever taken, so the lowest remaining index never moves back: a cursor
        // finds the next seed rather than a search of every remaining item for each chain.
        var taken = new bool[items.Length];
        var remaining = items.Length;
        var seed = 0;
        var nodes = new Dictionary<SourceNodeKey, List<int>>();

        void AddNode(int index, long id)
        {
            var key = NodeKey(items[index], id);

            if (!nodes.TryGetValue(key, out var list))
                nodes[key] = list = new();

            list.Add(index);
        }

        for (var i = 0; i < items.Length; i++)
        {
            AddNode(i, items[i].StartNodeId);
            AddNode(i, items[i].EndNodeId);
        }

        while (remaining > 0)
        {
            while (taken[seed])
                seed++;

            taken[seed] = true;
            remaining--;
            var chain = new LinkedList<PreparedSegment>();
            var first = items[seed];

            if (!first.OneWay && first.StartNodeId > first.EndNodeId)
                first = Reverse(first);

            chain.AddLast(first);

            foreach (var prepend in new[] { false, true })
                while (true)
                {
                    var tip = prepend ? chain.First!.Value : chain.Last!.Value;
                    var id = prepend ? tip.StartNodeId : tip.EndNodeId;
                    var key = NodeKey(tip, id);

                    if (degree[key] != 2)
                        break;

                    // A closed section is listed twice at its node and so counts twice here.
                    var candidates = 0;
                    var index = -1;

                    foreach (var item in nodes[key])
                        if (!taken[item])
                        {
                            candidates++;
                            index = item;
                        }

                    if (candidates != 1)
                        break;

                    var segment = items[index];
                    var matches = prepend ? segment.EndNodeId == id : segment.StartNodeId == id;

                    if (!matches)
                    {
                        if (segment.OneWay)
                            break;

                        segment = Reverse(segment);
                    }

                    taken[index] = true;
                    remaining--;

                    if (prepend)
                        chain.AddFirst(segment);
                    else
                        chain.AddLast(segment);
                }

            foreach (var item in chain)
                yield return item;
        }
    }

    public static PreparedSegment Reverse(PreparedSegment segment)
    {
        NetworkSides Swap(NetworkSides value)
            => ((value & NetworkSides.Left) != 0 ? NetworkSides.Right : 0)
                | ((value & NetworkSides.Right) != 0 ? NetworkSides.Left : 0);

        return segment with
        {
            StartNodeId = segment.EndNodeId,
            EndNodeId = segment.StartNodeId,
            Start = segment.End,
            End = segment.Start,
            Control1 = segment.Control2,
            Control2 = segment.Control1,
            FinalDeckCurve =
                segment.FinalDeckCurve.HasValue
                    ? new(
                        segment.FinalDeckCurve.Value.P3,
                        segment.FinalDeckCurve.Value.P2,
                        segment.FinalDeckCurve.Value.P1,
                        segment.FinalDeckCurve.Value.P0)
                    : null,
            HorizontalEmissionReference =
                segment.HorizontalEmissionReference.HasValue
                    ? new(
                        segment.HorizontalEmissionReference.Value.P3,
                        segment.HorizontalEmissionReference.Value.P2,
                        segment.HorizontalEmissionReference.Value.P1,
                        segment.HorizontalEmissionReference.Value.P0)
                    : null,
            StartHeightFamily = segment.EndHeightFamily,
            EndHeightFamily = segment.StartHeightFamily,
            PairedReversed = segment.PairedSpanId != 0 ? !segment.PairedReversed : segment.PairedReversed,
            PairedReferenceBegin = segment.PairedReferenceEnd,
            PairedReferenceEnd = segment.PairedReferenceBegin,
            StartReferenceSpeed = segment.EndReferenceSpeed,
            EndReferenceSpeed = segment.StartReferenceSpeed,
            StartCurveOffset = segment.EndCurveOffset,
            EndCurveOffset = segment.StartCurveOffset,
            StartElevation = segment.EndElevation,
            EndElevation = segment.StartElevation,
            ForwardLanes = segment.BackwardLanes,
            BackwardLanes = segment.ForwardLanes,
            ExplicitIncline = segment.ExplicitIncline.HasValue ? -segment.ExplicitIncline.Value : null,
            ExplicitCutSides = Swap(segment.ExplicitCutSides),
            ExplicitWallSides = Swap(segment.ExplicitWallSides),
            RetainingWallSides = Swap(segment.RetainingWallSides),
            StartRoundaboutRadius = segment.EndRoundaboutRadius,
            EndRoundaboutRadius = segment.StartRoundaboutRadius,
            StartMiniRoundabout = segment.EndMiniRoundabout,
            EndMiniRoundabout = segment.StartMiniRoundabout,
            StartTerminal = segment.EndTerminal,
            EndTerminal = segment.StartTerminal,
            StartLevelCrossing = segment.EndLevelCrossing,
            EndLevelCrossing = segment.StartLevelCrossing,
            OriginalGeometry = segment.OriginalGeometry.Reverse().ToArray(),
            SourceGeometry = segment.SourceGeometry.Reverse().ToArray(),
            HorizontalParent = null
        };
    }

    private static void SmoothJoins(
        List<PreparedSegment> sections,
        ImportLog log,
        CancellationToken token,
        float halfWidth)
    {
        var endpoints = new Dictionary<SourceNodeKey, List<(int index, bool start)>>();

        for (var i = 0; i < sections.Count; i++)
        {
            var segment = sections[i];

            foreach (var start in new[] { true, false })
            {
                var node = start ? segment.StartNodeId : segment.EndNodeId;
                var key = NodeKey(segment, node);

                if (!endpoints.TryGetValue(key, out var entries))
                    endpoints[key] = entries = new List<(int, bool)>();

                entries.Add((i, start));
            }
        }

        var clock = Stopwatch.StartNew();
        var smoothed = 0;

        // Joins that share no section are independent: a join reads and changes only its two
        // sections. Sections joined through two-section nodes form components; each component
        // keeps the original node order and the components run in parallel, so the result is
        // the same as one pass in order (report 114 S4). The game's runtime took 28-83 s here.
        var nodes = endpoints.Values.ToArray();
        var component = Enumerable.Range(0, sections.Count).ToArray();

        int Root(int i)
        {
            while (component[i] != i)
                i = component[i] = component[component[i]];

            return i;
        }

        foreach (var entries in nodes)
            if (entries.Count == 2)
                component[Root(entries[0].index)] = Root(entries[1].index);

        var groups = new Dictionary<int, List<int>>();

        for (var n = 0; n < nodes.Length; n++)
        {
            if (nodes[n].Count != 2)
                continue;

            var root = Root(nodes[n][0].index);

            if (!groups.TryGetValue(root, out var members))
                groups[root] = members = new List<int>();

            members.Add(n);
        }

        var current = sections.ToArray();

        void Join(List<(int index, bool start)> entries)
        {
            if (entries.Count != 2 || entries[0].index == entries[1].index)
                return;

            var left = entries[0];
            var right = entries[1];
            var firstSegment = current[left.index];
            var secondSegment = current[right.index];
            var sameRoute =
                firstSegment.WayId == secondSegment.WayId
                    || firstSegment.Reference.Length > 0
                        && firstSegment.Reference.Equals(secondSegment.Reference, StringComparison.OrdinalIgnoreCase)
                    || firstSegment.Name.Length > 0
                        && firstSegment.Name.Equals(secondSegment.Name, StringComparison.OrdinalIgnoreCase);

            // A name boundary is not a sharp bend. Degree-two continuations with
            // compatible roles/widths may share a tangent across source ways.
            if (firstSegment.Classification != secondSegment.Classification
                || firstSegment.Lanes != secondSegment.Lanes
                || firstSegment.OneWay != secondSegment.OneWay
                || !sameRoute
                    && (firstSegment.ProtectedLink != secondSegment.ProtectedLink
                        || firstSegment.Reference.Length > 0
                            && secondSegment.Reference.Length > 0
                            && !firstSegment.Reference.Equals(
                                secondSegment.Reference,
                                StringComparison.OrdinalIgnoreCase)))
                return;

            if (firstSegment.OneWay && left.start == right.start)
                return;

            // A curve joining a straight section must not bow that straight section.
            bool Straight(PreparedSegment segment)
                => segment.SourceGeometry.Count >= 2
                    && segment.SourceGeometry.All(
                        otherPoint => DistanceToSegment(otherPoint, segment.Start, segment.End) < .05f);

            var point = left.start ? firstSegment.Start : firstSegment.End;
            var other = right.start ? secondSegment.Start : secondSegment.End;

            if (math.distance(point, other) > .01f)
                return;

            var ca =
                left.start
                    ? firstSegment.Control1 ?? math.lerp(firstSegment.Start, firstSegment.End, 1f / 3)
                    : firstSegment.Control2 ?? math.lerp(firstSegment.Start, firstSegment.End, 2f / 3);

            var callback =
                right.start
                    ? secondSegment.Control1 ?? math.lerp(secondSegment.Start, secondSegment.End, 1f / 3)
                    : secondSegment.Control2 ?? math.lerp(secondSegment.Start, secondSegment.End, 2f / 3);

            var incoming = math.normalizesafe(point - ca);
            var outgoing = math.normalizesafe(callback - point);

            if (math.dot(incoming, outgoing) < .866f)
                return; // Intentional corners stay sharp.

            // An individual source chord is not necessarily an intentionally
            // straight road. Only long straight runs anchor their own direction.
            var anchorA = Straight(firstSegment) && math.distance(firstSegment.Start.xz, firstSegment.End.xz) > 150;
            var anchorB = Straight(secondSegment) && math.distance(secondSegment.Start.xz, secondSegment.End.xz) > 150;

            if (anchorA && anchorB)
                return;

            var tangent = anchorA ? incoming : anchorB ? outgoing : math.normalizesafe(incoming + outgoing);
            var newA = point - tangent * math.distance(point, ca);
            var newB = point + tangent * math.distance(point, callback);

            if (Math.Abs(newA.x) > halfWidth
                || Math.Abs(newA.z) > halfWidth
                || Math.Abs(newB.x) > halfWidth
                || Math.Abs(newB.z) > halfWidth)
                return;

            var candidateA = left.start ? firstSegment with { Control1 = newA } : firstSegment with { Control2 = newA };
            var candidateB =
                right.start ? secondSegment with { Control1 = newB } : secondSegment with { Control2 = newB };

            if (!WithinTolerance(candidateA) || !WithinTolerance(candidateB))
                return;

            current[left.index] = candidateA;
            current[right.index] = candidateB;
            Interlocked.Increment(ref smoothed);
        }

        Parallel.ForEach(
            groups.Values,
            new ParallelOptions { CancellationToken = token },
            members =>
            {
                foreach (var n in members)
                    Join(nodes[n]);
            });

        for (var i = 0; i < current.Length; i++)
            sections[i] = current[i];

        log.Count("Road curve joins smoothed", smoothed);
        log.Diagnostic(
            $"PROCESS tangent-continuity complete; joinsSmoothed={smoothed}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.00}; junctions/sharp-corners unchanged=true");

        SmoothRampAttachments(sections, log, token, halfWidth);
    }

    private static IReadOnlyList<PreparedSegment> NormalizeLaneRuns(
        IReadOnlyList<PreparedSegment> input,
        ImportLog log,
        CancellationToken token)
    {
        var result = input.ToArray();
        var nodes = new Dictionary<long, List<int>>();

        for (var i = 0; i < input.Count; i++)
            if (input[i].Kind == NetworkKind.Road)
                foreach (var id in new[] { input[i].StartNodeId, input[i].EndNodeId })
                {
                    if (!nodes.TryGetValue(id, out var list))
                        nodes[id] = list = new List<int>();

                    list.Add(i);
                }

        var changed = 0;

        for (var i = 0; i < input.Count; i++)
        {
            token.ThrowIfCancellationRequested();
            var segment = input[i];

            if (segment.Kind != NetworkKind.Road
                || segment.ExplicitLanes
                || segment.ProtectedLink
                || !segment.OneWay
                || !(segment.Classification == "motorway" || segment.Classification == "trunk")
                || math.distance(segment.Start.xz, segment.End.xz) > 180)
                continue;

            int Neighbor(long id)
            {
                if (nodes[id].Count != 2)
                    return -1;

                return nodes[id][0] == i ? nodes[id][1] : nodes[id][0];
            }

            var ai = Neighbor(segment.StartNodeId);
            var bi = Neighbor(segment.EndNodeId);

            if (ai < 0 || bi < 0 || ai == bi)
                continue;

            var firstSegment = input[ai];
            var secondSegment = input[bi];

            bool Compatible(PreparedSegment otherSegment)
                => !otherSegment.ProtectedLink
                    && !otherSegment.Roundabout
                    && otherSegment.Classification == segment.Classification
                    && otherSegment.OneWay == segment.OneWay
                    && otherSegment.Layer == segment.Layer
                    && otherSegment.Tunnel == segment.Tunnel
                    && otherSegment.Bridge == segment.Bridge;

            if (!Compatible(firstSegment)
                || !Compatible(secondSegment)
                || firstSegment.Lanes != secondSegment.Lanes
                || firstSegment.Lanes == segment.Lanes)
                continue;

            result[i] = segment with { Lanes = Numeric.Clamp(firstSegment.Lanes, 1, 5) };
            changed++;
        }

        log.Count("Short highway lane fluctuations normalized", changed);
        // Remove short lane-count runs bounded on both sides by the same size.
        // A real branch can explain a lane addition/drop. Stop there rather than
        // following the through route and erasing a legitimate exit/merge lane.
        var runChanged = 0;
        var seenRuns = new HashSet<int>();

        bool SameRoute(PreparedSegment firstSegment, PreparedSegment secondSegment)
            => !secondSegment.ProtectedLink
                && !secondSegment.Roundabout
                && firstSegment.Kind == secondSegment.Kind
                && firstSegment.Classification == secondSegment.Classification
                && firstSegment.OneWay == secondSegment.OneWay
                && firstSegment.Layer == secondSegment.Layer
                && firstSegment.Tunnel == secondSegment.Tunnel
                && firstSegment.Bridge == secondSegment.Bridge
                && (firstSegment.WayId == secondSegment.WayId
                || firstSegment.CorridorId != 0 && firstSegment.CorridorId == secondSegment.CorridorId
                || firstSegment.Reference.Length > 0 && firstSegment.Reference == secondSegment.Reference
                || firstSegment.Name.Length > 0 && firstSegment.Name == secondSegment.Name);

        int Continuation(int index, long node)
        {
            if (nodes[node].Count != 2)
                return -1;

            var candidates = nodes[node].Where(i => i != index && SameRoute(result[index], result[i])).ToArray();

            return candidates.Length == 1 ? candidates[0] : -1;
        }

        for (var seed = 0; seed < result.Length; seed++)
        {
            token.ThrowIfCancellationRequested();
            var segment = result[seed];

            if (segment.Kind != NetworkKind.Road
                || segment.ExplicitLanes
                || segment.ProtectedLink
                || segment.Roundabout
                || !(segment.Classification is "motorway" or "trunk" or "primary" or "secondary" or "tertiary")
                    || seenRuns.Contains(seed))
                continue;

            var run = new HashSet<int> { seed };
            var ends = new List<int>();
            var length = math.distance(segment.Start.xz, segment.End.xz);

            foreach (var startNode in new[] { segment.StartNodeId, segment.EndNodeId })
            {
                var current = seed;
                var node = startNode;

                while (run.Count < 64 && length <= 350)
                {
                    var next = Continuation(current, node);

                    if (next < 0 || run.Contains(next))
                        break;

                    var road = result[next];

                    if (road.ExplicitLanes || road.Lanes != segment.Lanes)
                    {
                        ends.Add(next);
                        break;
                    }

                    run.Add(next);
                    length += math.distance(road.Start.xz, road.End.xz);
                    current = next;
                    node = road.StartNodeId == node ? road.EndNodeId : road.StartNodeId;
                }
            }

            foreach (var index in run)
                seenRuns.Add(index);

            if (length > 350 || ends.Count != 2 || result[ends[0]].Lanes != result[ends[1]].Lanes)
                continue;

            foreach (var index in run)
            {
                result[index] = result[index] with { Lanes = result[ends[0]].Lanes };
                runChanged++;
            }
        }

        log.Count("Corridor lane fluctuation sections normalized", runChanged);
        log.Diagnostic(
            "LANE POLICY syntheticAuxiliaryWidening=false; sustainedSourceLaneChangesPreserved=true; maximumNormalizedRunMeters=350; rampsNotWidened=true; realBranchesAndStructureBoundariesStopNormalization=true; explicitLaneEvidenceUnchanged=true");

        return result;
    }

    private static void SmoothRampAttachments(
        List<PreparedSegment> sections,
        ImportLog log,
        CancellationToken token,
        float halfWidth)
    {
        var nodes = new Dictionary<long, List<(int index, bool start)>>();

        for (var i = 0; i < sections.Count; i++)
            if (sections[i].Kind == NetworkKind.Road)
                foreach (var start in new[] { true, false })
                {
                    var id = start ? sections[i].StartNodeId : sections[i].EndNodeId;

                    if (!nodes.TryGetValue(id, out var list))
                        nodes[id] = list = new List<(int, bool)>();

                    list.Add((i, start));
                }

        var changed = 0;

        foreach (var entries in nodes.Values.Where(items => items.Count >= 3))
            foreach (var rampEnd in entries)
            {
                token.ThrowIfCancellationRequested();
                var ramp = sections[rampEnd.index];

                if (!ramp.ProtectedLink || !ramp.OneWay)
                    continue;

                var point = rampEnd.start ? ramp.Start : ramp.End;
                var control =
                    rampEnd.start
                        ? ramp.Control1 ?? math.lerp(ramp.Start, ramp.End, 1f / 3)
                        : ramp.Control2 ?? math.lerp(ramp.Start, ramp.End, 2f / 3);

                var direction = math.normalizesafe((rampEnd.start ? control - point : point - control).xz);
                var best = direction;
                var score = .5f;
                PreparedSegment? bestMain = null;

                foreach (var entry in entries)
                {
                    var main = sections[entry.index];

                    if (main.ProtectedLink || !main.OneWay || main.Tunnel != ramp.Tunnel)
                        continue;

                    var handle =
                        entry.start
                            ? main.Control1 ?? math.lerp(main.Start, main.End, 1f / 3)
                            : main.Control2 ?? math.lerp(main.Start, main.End, 2f / 3);

                    var heading = math.normalizesafe((entry.start ? handle - main.Start : main.End - handle).xz);
                    var dot = math.dot(direction, heading);

                    if (dot > score)
                    {
                        score = dot;
                        best = heading;
                        bestMain = main;
                    }
                }

                if (score <= .5f)
                    continue;

                var length = Math.Min(60, math.distance(point.xz, control.xz));
                var revised = control;
                revised.x = point.x + best.x * length * (rampEnd.start ? 1 : -1);
                revised.z = point.z + best.y * length * (rampEnd.start ? 1 : -1);

                // Lateral lane alignment belongs to NetworkLaneAlignment only.
                // Applying it here too doubled offsets before the final width pass.
                if (math.distance(revised.xz, control.xz) > 12
                    || Math.Abs(revised.x) > halfWidth
                    || Math.Abs(revised.z) > halfWidth)
                    continue;

                if (bestMain != null && bestMain.NativeLaneOffsets.Count > 0 && ramp.NativeLaneOffsets.Count > 0)
                    log.Count("Ramp outer-lane approaches aligned");

                var candidate = rampEnd.start ? ramp with { Control1 = revised } : ramp with { Control2 = revised };

                // Direction-only smoothing must obey the existing source envelope.
                // Lane-aware geometry is an intentional bounded lateral correction;
                // retain its approved alignment as the subsequent grading reference.
                if (!WithinTolerance(candidate))
                    continue;

                sections[rampEnd.index] = candidate;
                changed++;
            }

        log.Count("Ramp mainline tangents aligned", changed);
    }

    public static IReadOnlyList<PreparedSegment> Refit(
        PreparedSegment section,
        CancellationToken token,
        float tolerance = .5f)
    {
        var points = section.SourceGeometry.ToList();

        if (points.Count < 3)
            return new[] { section };

        var edges = new List<PreparedSegment>();

        long Id(float3 point)
        {
            unchecked
            {
                return -1 - (long)(((ulong)section.WayId * 104729UL ^ math.hash(point.xz)) % (ulong)long.MaxValue);
            }
        }

        for (var i = 1; i < points.Count; i++)
            edges.Add(section with
            {
                Start = points[i - 1],
                End = points[i],
                StartNodeId = i == 1 ? section.StartNodeId : Id(points[i - 1]),
                EndNodeId = i == points.Count - 1 ? section.EndNodeId : Id(points[i]),
                Control1 = null,
                Control2 = null,
                HorizontalParent = null,
                StartRoundaboutRadius = i == 1 ? section.StartRoundaboutRadius : 0,
                EndRoundaboutRadius = i == points.Count - 1 ? section.EndRoundaboutRadius : 0
            });

        var output = new List<PreparedSegment>();
        Fit(points, edges, output, token, float.PositiveInfinity, tolerance);

        return output;
    }

    private static void Fit(
        List<float3> points,
        IReadOnlyList<PreparedSegment> source,
        List<PreparedSegment> output,
        CancellationToken token,
        float halfWidth,
        float? tolerance = null)
    {
        token.ThrowIfCancellationRequested();

        if (source.Count == 1)
        {
            output.Add(source[0] with { SourceGeometry = points.ToArray() });
            return;
        }

        var first = source[0];
        var last = source[source.Count - 1];
        var distance = math.distance(points[0].xz, points[points.Count - 1].xz);
        // Estimate direction over a short arc, rather than trusting one noisy OSM edge.
        var reach =
            Math.Min(
                first.Kind == NetworkKind.Rail
                    ? 80f
                    : first.Classification == "motorway" || first.Classification == "trunk" ? 60f : 35f,
                distance * .2f);

        float3 Tangent(bool end)
        {
            var origin = end ? points[points.Count - 1] : points[0];
            var length = 0f;
            var previous = origin;

            for (var i = 1; i < points.Count; i++)
            {
                var point = points[end ? points.Count - 1 - i : i];
                length += math.distance(previous.xz, point.xz);
                previous = point;

                if (length >= reach || i == points.Count - 1)
                    return math.normalizesafe(end ? origin - point : point - origin);
            }

            return math.normalizesafe(last.End - first.Start);
        }

        var fitValid =
            NetworkSourceFit.TryFit(points, points[0], points[points.Count - 1], Tangent(false).xz, Tangent(true).xz,
            tolerance ?? Tolerance(first.Kind), token, out var fitted, out var worstStation);

        var c1 = (float3)fitted.P1;
        var c2 = (float3)fitted.P2;
        var candidate = first with
        {
            End = last.End,
            EndNodeId = last.EndNodeId,
            EndRoundaboutRadius = last.EndRoundaboutRadius,
            EndMiniRoundabout = last.EndMiniRoundabout,
            EndTerminal = last.EndTerminal,
            EndLevelCrossing = last.EndLevelCrossing,
            SourceWayIds =
                source
                    .SelectMany(
                        segment => segment.SourceWayIds.Count > 0 ? segment.SourceWayIds : new[] { segment.WayId })
                    .Distinct()
                    .ToArray(),
            SourceNodeAliases = source.SelectMany(segment => segment.SourceNodeAliases).Distinct().ToArray(),
            OriginalGeometry =
                source
                    .SelectMany(
                        segment => segment.OriginalGeometry.Count > 0
                            ? segment.OriginalGeometry
                            : new[] { segment.Start, segment.End })
                    .ToArray(),
            ExplicitLanes = source.Any(segment => segment.ExplicitLanes),
            Control1 = c1,
            Control2 = c2,
            SourceGeometry = points.ToArray()
        };

        if (fitValid && distance > 0.05f && Math.Abs(c1.x) <= halfWidth && Math.Abs(c1.z) <= halfWidth
            && Math.Abs(c2.x) <= halfWidth && Math.Abs(c2.z) <= halfWidth && WithinTolerance(candidate, tolerance))
        {
            output.Add(candidate);
            return;
        }

        // A rejected fit increases the section count, never deletes source geometry.
        var split = Math.Max(1, Math.Min(source.Count - 1, worstStation));
        Fit(points.Take(split + 1).ToList(), source.Take(split).ToArray(), output, token, halfWidth, tolerance);
        Fit(points.Skip(split).ToList(), source.Skip(split).ToArray(), output, token, halfWidth, tolerance);
    }

    public static float3 Evaluate(PreparedSegment segment, float t)
    {
        var c1 = segment.CourseControl1;
        var c2 = segment.CourseControl2;
        var u = 1 - t;

        return u * u * u * segment.CourseStart
            + 3 * u * u * t * c1
            + 3 * u * t * t * c2
            + t * t * t * segment.CourseEnd;
    }

    public static float DistanceToSegment(float3 point, float3 start, float3 end)
    {
        point.y = start.y = end.y = 0;
        var delta = end - start;
        var t = math.lengthsq(delta) > 0 ? math.clamp(math.dot(point - start, delta) / math.lengthsq(delta), 0, 1) : 0;

        return math.distance(point, start + t * delta);
    }

    public static bool WithinTolerance(PreparedSegment segment, float? requestedTolerance = null)
    {
        if (segment.SourceGeometry.Count < 2)
            return false;

        // Exact straight horizontal geometry remains exact even when only its vertical
        // controls change. Do not reject long straight sections on a sampling allowance.
        if (segment.SourceGeometry.All(point => DistanceToSegment(point, segment.Start, segment.End) < .01f)
            && DistanceToSegment(segment.Control1 ?? segment.Start, segment.Start, segment.End) < .01f
                && DistanceToSegment(segment.Control2 ?? segment.End, segment.Start, segment.End) < .01f)
            return true;

        var c1 = segment.Control1 ?? math.lerp(segment.Start, segment.End, 1f / 3f);
        var c2 = segment.Control2 ?? math.lerp(segment.Start, segment.End, 2f / 3f);
        var a = segment.Start;
        var d = segment.End;
        a.y = c1.y = c2.y = d.y = 0;
        var arcBound = math.distance(a, c1) + math.distance(c1, c2) + math.distance(c2, d);
        var tolerance = requestedTolerance ?? segment.SourceFitToleranceMeters ?? Tolerance(segment.Kind);
        // Adapt sampling to curve length while preserving the same continuous-error
        // reserve. Short curves no longer need 256 samples; long curves get more.
        var sampleCount = Math.Min(2048, Math.Max(32, (int)Math.Ceiling(6 * arcBound / tolerance)));
        var samples = new float3[sampleCount + 1];

        for (var i = 0; i <= sampleCount; i++)
            samples[i] = Evaluate(segment, i / (float)sampleCount);

        // Control-polygon length bounds between-sample movement; reserve that error
        // instead of trusting vertex-only comparisons which can cut a hairpin.
        var allowance = tolerance - 3 * arcBound / (2 * sampleCount);

        if (allowance <= 0)
            return false;

        // The largest coordinate either line holds, for the rounding margin (RoundingMargin).
        var extent = 0d;

        foreach (var sample in samples)
            extent = Math.Max(extent, Math.Max(Math.Abs((double)sample.x), Math.Abs((double)sample.z)));

        foreach (var sourcePoint in segment.SourceGeometry)
            extent = Math.Max(extent, Math.Max(Math.Abs((double)sourcePoint.x), Math.Abs((double)sourcePoint.z)));

        // Consecutive samples are usually near the same source piece. A piece near the cursor
        // that lies within the allowance by more than the rounding margin settles a sample:
        // the index could only answer at least as near (see CertainlyNear). The index decides
        // every other sample, so the answer is the index's for every sample.
        LineDistanceIndex? sourceIndex = null;
        var sourceCursor = 0;

        foreach (var point in samples)
        {
            if (CertainlyNear(segment.SourceGeometry, point, allowance, extent, ref sourceCursor))
                continue;

            sourceIndex ??= new LineDistanceIndex(segment.SourceGeometry);

            if (sourceIndex.Distance(point) > allowance)
                return false;
        }

        // Every source point must lie within 0.75 tolerance of the sampled curve. The curve
        // runs the same way as the source, so the nearest sample piece is almost always next
        // to the previous one: a short window settles most points without building an index.
        // A point fails only if no piece at all is close enough, which the index then decides.
        LineDistanceIndex? curveIndex = null;
        var limit = tolerance * 0.75f;
        var cursor = 0;

        for (var i = 1; i < segment.SourceGeometry.Count; i++)
        {
            var start = segment.SourceGeometry[i - 1];
            var end = segment.SourceGeometry[i];
            var steps = Math.Max(1, (int)Math.Ceiling(math.distance(start, end) / (tolerance / 2)));

            for (var j = 0; j <= steps; j++)
            {
                var point = math.lerp(start, end, j / (float)steps);

                if (NearPiece(samples, point, limit, extent, ref cursor))
                    continue;

                curveIndex ??= new LineDistanceIndex(samples);

                if (curveIndex.Distance(point) > limit)
                    return false;
            }
        }

        return true;
    }

    // Whether a piece of the sampled line within a few pieces of the cursor lies within
    // the limit of the point, using the same distance as LineDistanceIndex. Moves the
    // cursor to the nearest such piece. Extent bounds the size of every coordinate involved.
    private static bool NearPiece(float3[] samples, float3 point, float limit, double extent, ref int cursor)
    {
        const int Window = 24;
        var first = Math.Max(0, cursor - Window);
        var last = Math.Min(samples.Length - 2, cursor + Window);
        var best = float.PositiveInfinity;
        var bestPiece = -1;

        // The result is the nearest piece (the first of equally near ones) and its distance.
        // The cursor's own piece, inside the window, bounds that distance, so a piece lying
        // further off than reach along x or z is further than it even after rounding: it
        // can be neither and is skipped. A NaN or infinite coordinate makes reach NaN or
        // infinite, and then nothing is skipped.
        var cursorDistance = (double)DistanceToSegment(point, samples[cursor], samples[cursor + 1]);
        var reach = cursorDistance + RoundingMargin(extent, cursorDistance);

        for (var piece = first; piece <= last; piece++)
        {
            var start = samples[piece];
            var end = samples[piece + 1];

            if (Math.Min(start.x, end.x) - (double)point.x > reach
                || point.x - (double)Math.Max(start.x, end.x) > reach
                || Math.Min(start.z, end.z) - (double)point.z > reach
                || point.z - (double)Math.Max(start.z, end.z) > reach)
                continue;

            var distance = DistanceToSegment(point, start, end);

            if (distance < best)
            {
                best = distance;
                bestPiece = piece;
            }
        }

        if (best > limit)
            return false;

        cursor = bestPiece;

        return true;
    }

    // How far a float distance from DistanceToSegment or LineDistanceIndex.Lower can fall short
    // of the true one, with ten times to spare. Float rounding there stays under about
    // 1e-6 x the largest coordinate involved (extent) + 5e-7 x the distance itself.
    private static double RoundingMargin(double extent, double distance) => 1e-5 * (extent + distance) + 1e-3;

    // Whether LineDistanceIndex.Distance(point) over this line is certainly within the limit,
    // judged on the pieces near the cursor; moves the cursor to the piece that settled it.
    // The index returns the least distance among the pieces its search reaches. It passes a
    // piece over only when a box around it lies further off than the best distance found so
    // far, and the box is no further than the piece. So with a piece within the limit by more
    // than the rounding margin, the index's answer is within the limit too.
    private static bool CertainlyNear(
        IReadOnlyList<float3> line,
        float3 point,
        float limit,
        double extent,
        ref int cursor)
    {
        const int Window = 4;

        bool Settles(int piece)
        {
            var distance = (double)DistanceToSegment(point, line[piece], line[piece + 1]);

            return distance <= limit - RoundingMargin(extent, distance);
        }

        if (Settles(cursor))
            return true;

        var first = Math.Max(0, cursor - Window);
        var last = Math.Min(line.Count - 2, cursor + Window);

        for (var piece = first; piece <= last; piece++)
            if (piece != cursor && Settles(piece))
            {
                cursor = piece;

                return true;
            }

        return false;
    }

    // Exact nearest-segment queries with bounding-box pruning. This retains the
    // same two-sided tolerance test without scanning thousands of samples per query.
    private sealed class LineDistanceIndex
    {
        private sealed class Node
        {
            public float2 Min, Max;
            public int First, Count;
            public Node? Left, Right;
        }

        private readonly IReadOnlyList<float3> points;
        private readonly int[] indices;
        private readonly Node root;

        public LineDistanceIndex(IReadOnlyList<float3> line)
        {
            points = line;
            indices = Enumerable.Range(0, line.Count - 1).ToArray();
            root = Build(0, indices.Length);
        }

        private Node Build(int first, int count)
        {
            var node =
                new Node
                {
                    First = first,
                    Count = count,
                    Min = new float2(float.PositiveInfinity),
                    Max = new float2(float.NegativeInfinity)
                };

            for (var i = first; i < first + count; i++)
            {
                var index = indices[i];
                node.Min = math.min(node.Min, math.min(points[index].xz, points[index + 1].xz));
                node.Max = math.max(node.Max, math.max(points[index].xz, points[index + 1].xz));
            }

            if (count <= 8)
                return node;

            var x = node.Max.x - node.Min.x >= node.Max.y - node.Min.y;
            Array.Sort(indices, first, count, Comparer<int>.Create((otherFirst, second) =>
                (x
                    ? points[otherFirst].x + points[otherFirst + 1].x
                    : points[otherFirst].z
                        + points[otherFirst
                            + 1].z).CompareTo(
                                x
                                    ? points[second].x + points[second + 1].x
                                    : points[second].z + points[second + 1].z)));

            var half = count / 2;
            node.Left = Build(first, half);
            node.Right = Build(first + half, count - half);

            return node;
        }

        public float Distance(float3 point)
        {
            var best = float.PositiveInfinity;
            Search(root, point, Lower(root, point), ref best);

            return best;
        }

        private static float Lower(Node node, float3 point)
            => math.distance(point.xz, math.clamp(point.xz, node.Min, node.Max));

        // Plain methods rather than local functions: this runs for every sample of every
        // fitted curve, and a captured local allocates on each query.
        private void Search(Node node, float3 point, float lower, ref float best)
        {
            if (lower > best)
                return;

            if (node.Left == null)
            {
                for (var i = node.First; i < node.First + node.Count; i++)
                {
                    var index = indices[i];
                    best = Math.Min(best, DistanceToSegment(point, points[index], points[index + 1]));
                }

                return;
            }

            var left = node.Left!;
            var right = node.Right!;
            var lowerLeft = Lower(left, point);
            var lowerRight = Lower(right, point);

            if (lowerLeft <= lowerRight)
            {
                Search(left, point, lowerLeft, ref best);
                Search(right, point, lowerRight, ref best);
            }
            else
            {
                Search(right, point, lowerRight, ref best);
                Search(left, point, lowerLeft, ref best);
            }
        }
    }

    public static void Validate(
        IReadOnlyList<PreparedSegment> segments,
        double width,
        CancellationToken token,
        ImportLog? log = null)
    {
        var clock = Stopwatch.StartNew();
        var nextProgress = 5d;
        var processed = 0;
        var parents = new HashSet<PreparedSegment>();

        foreach (var segment in segments)
        {
            token.ThrowIfCancellationRequested();
            processed++;

            if (clock.Elapsed.TotalSeconds >= nextProgress)
            {
                log?.Diagnostic(
                    $"PROCESS geometry-validation sections={processed}/{segments.Count}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.0}");

                nextProgress = clock.Elapsed.TotalSeconds + 5;
            }

            if (!math.all(math.isfinite(segment.Start))
                || !math.all(math.isfinite(segment.End))
                || math.distance(segment.Start, segment.End) < 0.01f)
                log?.Warn(
                    $"Normalized way {segment.WayId} has invalid geometry; retained for local repair, not an import gate.");

            for (var i = 0; i <= 32; i++)
            {
                var point = Evaluate(segment, i / 32f);

                if (!math.all(math.isfinite(point))
                    || Math.Abs(point.x) > width / 2 + 0.02
                    || Math.Abs(point.z) > width / 2 + 0.02)
                    log?.Count("Normalized network envelope observations");
            }

            if (segment.HorizontalParent != null)
            {
                // Profile refinement subdivides an already validated horizontal curve exactly.
                if (parents.Add(segment.HorizontalParent) && !WithinTolerance(segment.HorizontalParent))
                    log?.Count("Profile parent geographic tolerance observations");

                for (var i = 0; i <= 32; i++)
                    if (math.distance(
                        Evaluate(segment, i / 32f).xz,
                        Evaluate(
                            segment.HorizontalParent,
                            math.lerp(segment.ParentStartT, segment.ParentEndT, i / 32f)).xz)
                        > .02f)
                        log?.Count("Vertical refinement horizontal alignment observations");
            }
            else if (segment.Control1.HasValue && !WithinTolerance(segment))
                log?.Count("Fitted network geographic tolerance observations");
        }
    }
}
