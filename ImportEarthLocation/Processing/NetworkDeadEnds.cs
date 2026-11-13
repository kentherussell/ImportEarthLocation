using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Merged avenues stop between the carriageways of the road they met, and ramps
// stop where an unimported street carried on: in the 5 November game captures
// nearly all of these ends lay within 25 m (avenues) or 60 m (ramps) of another
// road, and each drew a dead-end warning. Such an end is carried on to that road,
// along its own heading unless the gap is under TightGap. It joins a node within
// NodeSnap of the meeting point, or else splits the road it meets. Bridges and
// tunnels are never joined, and only a ramp may join a motorway or trunk.
public static class NetworkDeadEnds
{
    private const double AvenueReach = 25,
        RampReach = 60,
        TightGap = 3,
        NodeSnap = 8,
        BorderMargin = 50,
        MaximumRise = 3,
        MinimumHeading = .5,
        MaximumAlong = .9,
        RelaxedReach = 80,
        RelaxedHeading = 0,
        SampleSpacing = 8,
        CellSize = 32,
        // A merged avenue's centreline lies up to half the avenue's width from each carriageway.
        MergedReach = 40;

    private readonly record struct End(int Section, bool AtStart, double3 Point, double2 Outward, long Node);

    private readonly record struct Sample(int Section, double T, double2 Point, double Y);

    // Merging a divided road keeps a short arm out to each node where an unimported
    // side street met a carriageway. The first stub pass runs before the merge and
    // never sees them; left in, each is a dead end and cuts the avenue into pieces.
    // A removed port becomes an alias of the node its stub hung from.
    public static IReadOnlyList<PreparedSegment> PruneStubs(
        IReadOnlyList<PreparedSegment> source,
        ImportRequest request,
        ImportLog log)
    {
        var kept = NetworkStubs.Prune(source, request, log);

        if (kept.Count == source.Count)
            return kept;

        var alive =
            new HashSet<long>(
                kept
                    .Where(segment => segment.Kind == NetworkKind.Road)
                    .SelectMany(segment => new[] { segment.StartNodeId, segment.EndNodeId }));

        var parent = new Dictionary<long, long>();

        long Root(long id)
        {
            while (parent.TryGetValue(id, out var up) && up != id)
                id = up;

            return id;
        }

        // Prune keeps its survivors in order.
        var next = 0;

        foreach (var segment in source)
        {
            if (next < kept.Count && ReferenceEquals(kept[next], segment))
            {
                next++;
                continue;
            }

            if (segment.Kind != NetworkKind.Road)
                continue;

            var root = Root(segment.StartNodeId);
            var otherRoot = Root(segment.EndNodeId);

            if (root == otherRoot)
                continue;

            if (alive.Contains(root))
                parent[otherRoot] = root;
            else
                parent[root] = otherRoot;
        }

        var remap =
            parent.Keys.Where(id => !alive.Contains(id) && alive.Contains(Root(id))).ToDictionary(id => id, Root);

        return SourceTopologyObligations.Transfer(source, kept, NetworkKind.Road, remap);
    }

    // A ramp mostly ends where an unimported street carries on, and the game warns
    // at any one-way lane with nowhere to go. Where the capture's unimported ways lead
    // from a dead end to a planned road within StreetReach, that path is imported as a
    // two-way regular road. Close then joins its far end if it stops beside the road.
    public static IReadOnlyList<PreparedSegment> ConnectThroughStreets(
        IReadOnlyList<PreparedSegment> source,
        OsmDataset dataset,
        ImportRequest request,
        IReadOnlyDictionary<string, float> widths,
            IReadOnlyDictionary<string, float[]> laneOffsets,
            ImportLog log,
            CancellationToken token)
    {
        const double StreetReach = 800;

        static bool Drivable(OsmWay way)
            => way.Tags.TryGetValue("highway", out var text)
                && text is "residential" or "unclassified" or "tertiary" or "secondary" or "primary" or "living_street" or "road" or "primary_link" or "secondary_link" or "tertiary_link"
                && !(way.Tags.TryGetValue("access", out var firstText) && firstText is "no" or "private")
                    && !(way.Tags.TryGetValue("area", out var area) && area == "yes")
                && !way.Tags.ContainsKey("bridge")
                && !way.Tags.ContainsKey("tunnel");

        var half = request.MapSize.WidthMeters / 2 - BorderMargin;
        var imported =
            new HashSet<long>(
                source
                    .Where(segment => segment.Kind == NetworkKind.Road)
                    .SelectMany(segment => segment.SourceWayIds.Append(segment.WayId)));

        var planned =
            new HashSet<long>(
                source
                    .Where(segment => segment.Kind == NetworkKind.Road)
                    .SelectMany(segment => new[] { segment.StartNodeId, segment.EndNodeId }));

        foreach (var id in imported)
            if (dataset.Ways.TryGetValue(id, out var way))
                planned.UnionWith(way.NodeIds);

        var degree = new Dictionary<long, int>();

        foreach (var segment in source)
            if (segment.Kind == NetworkKind.Road)
            {
                degree[segment.StartNodeId] = degree.GetValueOrDefault(segment.StartNodeId) + 1;
                degree[segment.EndNodeId] = degree.GetValueOrDefault(segment.EndNodeId) + 1;
            }

        var neighbours = new Dictionary<long, List<(long Node, OsmWay Way)>>();

        foreach (var way in dataset.Ways.Values)
        {
            if (imported.Contains(way.Id) || !Drivable(way))
                continue;

            for (var i = 1; i < way.NodeIds.Length; i++)
            {
                void Add(long a, long b)
                {
                    if (!neighbours.TryGetValue(a, out var items))
                        neighbours[a] = items = new();

                    items.Add((b, way));
                }

                Add(way.NodeIds[i - 1], way.NodeIds[i]);
                Add(way.NodeIds[i], way.NodeIds[i - 1]);
            }
        }

        float3? Local(long id)
            => dataset.Nodes.TryGetValue(id, out var otherNode)
                ? ImportEarthLocation.Geo.GeoMath.ToLocalMeters(
                    otherNode.Latitude,
                    otherNode.Longitude,
                    request.Latitude,
                    request.Longitude)
                : null;

        var added = new List<PreparedSegment>();
        var seen = new HashSet<(long, long)>();
        var connected = 0;

        foreach (var start in degree.Where(entry => entry.Value == 1).Select(entry => entry.Key).OrderBy(id => id))
        {
            token.ThrowIfCancellationRequested();

            if (!neighbours.ContainsKey(start)
                || !(Local(start) is float3 origin)
                || Math.Abs(origin.x) > half
                || Math.Abs(origin.z) > half)
                continue;

            var best = new Dictionary<long, double> { [start] = 0 };
            var from = new Dictionary<long, (long Node, OsmWay Way)>();
            var pending = new SortedSet<(double, long)> { (0, start) };
            long? reached = null;

            while (pending.Count > 0)
            {
                var (distance, node) = pending.Min;
                pending.Remove((distance, node));

                if (distance > StreetReach)
                    break;

                if (node != start && planned.Contains(node))
                {
                    reached = node;
                    break;
                }

                if (!neighbours.TryGetValue(node, out var next) || !(Local(node) is float3 point))
                    continue;

                foreach (var (to, way) in next)
                {
                    if (!(Local(to) is float3 otherPoint)
                        || Math.Abs(otherPoint.x) > half
                        || Math.Abs(otherPoint.z) > half)
                        continue;

                    var d = distance + math.distance(point.xz, otherPoint.xz);

                    if (best.TryGetValue(to, out var old) && old <= d)
                        continue;

                    pending.Remove((old, to));
                    best[to] = d;
                    from[to] = (node, way);
                    pending.Add((d, to));
                }
            }

            if (reached == null)
                continue;

            connected++;

            for (var node = reached.Value; node != start; node = from[node].Node)
            {
                var (previous, way) = from[node];
                planned.Add(node);
                planned.Add(previous);

                if (!seen.Add((Math.Min(previous, node), Math.Max(previous, node))))
                    continue;

                var vectorA = Local(previous)!.Value;
                var vectorB = Local(node)!.Value;
                way.Tags.TryGetValue("name", out var name);
                added.Add(
                    NativeAssetKeys.ApplyMetadata(
                        new PreparedSegment(
                            way.Id,
                            previous,
                            node,
                            vectorA,
                            vectorB,
                            NetworkKind.Road,
                            "tertiary",
                            2,
                            false,
                            false,
                            false,
                            0)
                        {
                            Name = name ?? "",
                            SourceWayIds = new[] { way.Id },
                            OriginalGeometry = new[] { vectorA, vectorB },
                            SourceGeometry = new[] { vectorA, vectorB },
                            SourceTags = way.Tags,
                            RawSourceTags = dataset.RawWayTags.TryGetValue(way.Id, out var raw) ? raw : way.Tags
                        }, widths, laneOffsets));
            }
        }

        log.Count("Dead ends connected through unimported streets", connected);
        log.Count("Street sections imported as connectors", added.Count);
        log.Diagnostic($"STREET CONNECTORS connected={connected}; sections={added.Count}; reachMeters={StreetReach}");

        return added.Count == 0 ? source : source.Concat(added).ToArray();
    }

    // The game makes any node within 0.1 m of the terrain bounds an outside connection
    // (GenerateNodesSystem.CreateNodesJob). An end heading off the map within BorderReach
    // is carried to the exact edge, for roads and track alike.
    public static IReadOnlyList<PreparedSegment> ReachBorder(
        IReadOnlyList<PreparedSegment> source,
        double width,
        ImportLog log)
    {
        const double BorderReach = 300;
        var half = width / 2;
        var result = source.ToArray();
        var extended = 0;
        var degree = new Dictionary<(NetworkKind, long), int>();

        foreach (var segment in source)
        {
            degree[(segment.Kind, segment.StartNodeId)] =
                degree.GetValueOrDefault((segment.Kind, segment.StartNodeId)) + 1;

            degree[(segment.Kind, segment.EndNodeId)] = degree.GetValueOrDefault((segment.Kind, segment.EndNodeId)) + 1;
        }

        for (var i = 0; i < result.Length; i++)
            foreach (var atStart in new[] { true, false })
            {
                var segment = result[i];
                var node = atStart ? segment.StartNodeId : segment.EndNodeId;

                if (degree[(segment.Kind, node)] != 1
                    || NetworkStructure.IsStructure(segment)
                    || segment.FinalDeckCurve != null
                    || (atStart ? segment.StartTerminal : segment.EndTerminal))
                    continue;

                var point = (double3)(atStart ? segment.CourseStart : segment.CourseEnd);
                var inner = (double3)(atStart ? segment.CourseControl1 : segment.CourseControl2);
                var outward = math.normalizesafe(point.xz - inner.xz);
                var gap = half - Math.Max(Math.Abs(point.x), Math.Abs(point.z));

                if (gap <= 0 || gap > BorderReach)
                    continue;

                var run = double.PositiveInfinity;

                if (Math.Abs(outward.x) > .5)
                    run = Math.Min(run, ((outward.x > 0 ? half : -half) - point.x) / outward.x);

                if (Math.Abs(outward.y) > .5)
                    run = Math.Min(run, ((outward.y > 0 ? half : -half) - point.z) / outward.y);

                if (run <= 0 || run > BorderReach)
                    continue;

                var delta = (float3)new double3(outward.x * run, 0, outward.y * run);
                result[i] =
                    atStart
                        ? segment with
                        {
                            Start = segment.Start + delta,
                            Control1 = (segment.Control1 ?? math.lerp(segment.Start, segment.End, 1f / 3)) + delta,
                            Control2 = segment.Control2 ?? math.lerp(segment.Start, segment.End, 2f / 3)
                        }
                        :
                    segment with
                    {
                        End = segment.End + delta,
                        Control1 = segment.Control1 ?? math.lerp(segment.Start, segment.End, 1f / 3),
                        Control2 = (segment.Control2 ?? math.lerp(segment.Start, segment.End, 2f / 3)) + delta
                    };

                extended++;
            }

        log.Count("Dead ends carried to the map edge", extended);

        return extended == 0 ? source : result;
    }

    // A one-way road still ending inland leads nowhere or comes from nowhere, and the
    // game warns at it. Its run back to the first junction goes, up to BlindLength,
    // with its ports aliased to that junction.
    public static IReadOnlyList<PreparedSegment> RemoveBlindOneWays(
        IReadOnlyList<PreparedSegment> source,
        double width,
        ImportLog log)
    {
        const double BlindLength = 800;
        var half = width / 2 - BorderMargin;
        var arms = new Dictionary<long, List<int>>();

        for (var i = 0; i < source.Count; i++)
            if (source[i].Kind == NetworkKind.Road)
                foreach (var id in new[] { source[i].StartNodeId, source[i].EndNodeId })
                {
                    if (!arms.TryGetValue(id, out var l))
                        arms[id] = l = new();

                    l.Add(i);
                }

        var removed = new HashSet<int>();
        var remap = new Dictionary<long, long>();

        foreach (var (id, list) in arms.OrderBy(entry => entry.Key))
        {
            if (list.Count != 1 || !source[list[0]].OneWay || removed.Contains(list[0]))
                continue;

            var first = source[list[0]];
            var at = first.StartNodeId == id ? first.CourseStart : first.CourseEnd;

            if (Math.Abs(at.x) > half
                || Math.Abs(at.z) > half
                || (first.StartNodeId == id ? first.StartTerminal : first.EndTerminal))
                continue;

            var run = new List<int>();
            var nodes = new List<long> { id };
            var node = id;
            var current = list[0];
            var length = 0d;
            var keep = -1L;

            while (true)
            {
                var segment = source[current];

                if (!segment.OneWay || segment.Roundabout || run.Contains(current))
                    break;

                run.Add(current);
                length += NetworkVerticalProfiles.ArcLength(segment);

                if (length > BlindLength)
                    break;

                node = segment.StartNodeId == node ? segment.EndNodeId : segment.StartNodeId;
                var next = arms[node].Where(a => a != current).ToArray();

                if (next.Length != 1)
                {
                    keep = next.Length > 1 ? node : -1;
                    break;
                }

                nodes.Add(node);
                current = next[0];
            }

            if (keep == -1 || length > BlindLength)
                continue;

            removed.UnionWith(run);

            foreach (var nodeId in nodes)
                remap[nodeId] = keep;
        }

        log.Count("Blind one-way roads removed", removed.Count);

        if (removed.Count == 0)
            return source;

        return SourceTopologyObligations.Transfer(
            source,
            source.Where((_, i) => !removed.Contains(i)).ToArray(),
            NetworkKind.Road,
            remap);
    }

    // A road or railway under LoosePiece long in total and joined to nothing else of
    // its kind is a leftover: the map data mapped a fragment, or the rest of it was
    // not imported. It carries no traffic and reads as litter, so it goes. A piece
    // that reaches the map border continues outside the map and stays.
    public static IReadOnlyList<PreparedSegment> RemoveLoosePieces(
        IReadOnlyList<PreparedSegment> source,
        double width,
        ImportLog log)
    {
        const double LoosePiece = 100;
        var half = width / 2 - BorderMargin;
        var parent = new Dictionary<SourceNodeKey, SourceNodeKey>();

        SourceNodeKey Root(SourceNodeKey key)
        {
            while (parent.TryGetValue(key, out var up) && up != key)
                key = up;

            return key;
        }

        foreach (var segment in source)
        {
            var first = Root(SourceNodeKey.Start(segment));
            var second = Root(SourceNodeKey.End(segment));

            if (first != second)
                parent[second] = first;
        }

        var length = new Dictionary<SourceNodeKey, double>();
        var kept = new HashSet<SourceNodeKey>();

        foreach (var segment in source)
        {
            var root = Root(SourceNodeKey.Start(segment));
            length[root] = length.GetValueOrDefault(root) + NetworkVerticalProfiles.ArcLength(segment);

            foreach (var point in new[] { segment.CourseStart, segment.CourseEnd })
                if (Math.Abs(point.x) > half || Math.Abs(point.z) > half)
                    kept.Add(root);
        }

        var result =
            source
                .Where(
                    segment =>
                    {
                        var root = Root(SourceNodeKey.Start(segment));

                        return kept.Contains(root) || length[root] >= LoosePiece;
                    })
                .ToArray();

        log.Count("Sections removed in short pieces joined to nothing", source.Count - result.Length);

        return result.Length == source.Count ? source : result;
    }

    // A first pass on heading, then one for one-way ends only, which the game
    // warns about: any road not behind them, though never a motorway or trunk.
    public static IReadOnlyList<PreparedSegment> Close(
        IReadOnlyList<PreparedSegment> source,
        double width,
        ImportLog log,
        CancellationToken token) =>
        Join(Join(source, width, log, token, false), width, log, token, true);

    private static IReadOnlyList<PreparedSegment> Join(
        IReadOnlyList<PreparedSegment> source,
        double width,
        ImportLog log,
        CancellationToken token,
        bool relaxed)
    {
        var degree = new Dictionary<long, int>();
        var used = new HashSet<long>();

        for (var i = 0; i < source.Count; i++)
        {
            used.Add(source[i].StartNodeId);
            used.Add(source[i].EndNodeId);

            if (source[i].Kind != NetworkKind.Road)
                continue;

            degree[source[i].StartNodeId] = degree.GetValueOrDefault(source[i].StartNodeId) + 1;
            degree[source[i].EndNodeId] = degree.GetValueOrDefault(source[i].EndNodeId) + 1;
        }

        var cells = new Dictionary<(int, int), List<Sample>>();
        var sampleStep = new double[source.Count];
        // Sections that record where an earlier merge moved a map-data node.
        var carriers = new Dictionary<long, List<(int Section, long Canonical)>>();

        (int, int) Cell(double2 point) => ((int)Math.Floor(point.x / CellSize), (int)Math.Floor(point.y / CellSize));

        for (var i = 0; i < source.Count; i++)
        {
            var segment = source[i];

            if (segment.Kind != NetworkKind.Road || NetworkStructure.IsStructure(segment))
                continue;

            token.ThrowIfCancellationRequested();

            foreach (var alias in segment.SourceNodeAliases)
            {
                if (!carriers.TryGetValue(alias.SourceNodeId, out var sections))
                    carriers[alias.SourceNodeId] = sections = new();

                sections.Add((i, alias.CanonicalNodeId));
            }

            var curve = NetworkDesignChecks.Course(segment);
            var n = Math.Max(1, (int)Math.Ceiling(NetworkDesignChecks.HorizontalArcLength(curve) / SampleSpacing));
            sampleStep[i] = 1d / n;

            for (var k = 0; k <= n; k++)
            {
                var t = k / (double)n;
                var point = NetworkDesignChecks.Evaluate(curve, t);
                var key = Cell(point.xz);

                if (!cells.TryGetValue(key, out var list))
                    cells[key] = list = new();

                list.Add(new(i, t, point.xz, point.y));
            }
        }

        // The nearest point of a section to a point, searched between two parameters.
        Sample Nearest(int section, double2 point, double from, double to)
        {
            var curve = NetworkDesignChecks.Course(source[section]);
            var best = from;
            var bestDistance = double.MaxValue;

            for (var round = 0; round < 3; round++)
            {
                var step = (to - from) / 8;

                for (var k = 0; k <= 8; k++)
                {
                    var t = from + k * step;
                    var distance = math.distance(NetworkDesignChecks.Evaluate(curve, t).xz, point);

                    if (distance < bestDistance)
                    {
                        bestDistance = distance;
                        best = t;
                    }
                }

                from = Math.Max(from, best - step);
                to = Math.Min(to, best + step);
            }

            var meet = NetworkDesignChecks.Evaluate(curve, best);

            return new(section, best, meet.xz, meet.y);
        }

        // Merging an avenue moves the node its carriageways shared onto the centreline
        // and records the move on the merged sections. A side road that ended at that
        // node is left behind at the old place, and belongs where the node went: the
        // merged node itself if a section still ends there, else the nearest point of
        // the merged road.
        Sample? Rejoined(int section, long node, double3 point)
        {
            if (!carriers.TryGetValue(node, out var sections))
                return null;

            Sample? found = null;
            var nearest = MergedReach;

            foreach (var atNode in new[] { true, false })
            {
                foreach (var (carrier, canonical) in sections)
                {
                    if (carrier == section)
                        continue;

                    var other = source[carrier];
                    var endsThere = other.StartNodeId == canonical || other.EndNodeId == canonical;

                    if (endsThere != atNode)
                        continue;

                    var end = other.StartNodeId == canonical ? 0d : 1d;
                    var sample = endsThere ? Nearest(carrier, point.xz, end, end) : Nearest(carrier, point.xz, 0, 1);

                    var distance = math.distance(sample.Point, point.xz);

                    if (distance >= nearest || Math.Abs(sample.Y - point.y) > MaximumRise)
                        continue;

                    nearest = distance;
                    found = sample;
                }

                if (found != null)
                    break;
            }

            return found;
        }

        var half = width / 2 - BorderMargin;
        var rejoined = 0;
        var ends = new List<(End End, Sample Target, double Gap)>();

        for (var i = 0; i < source.Count; i++)
        {
            var segment = source[i];

            if (segment.Kind != NetworkKind.Road
                || NetworkStructure.IsStructure(segment)
                || segment.FinalDeckCurve != null)
                continue;

            if (relaxed && !segment.OneWay)
                continue;

            foreach (var atStart in new[] { true, false })
            {
                var node = atStart ? segment.StartNodeId : segment.EndNodeId;

                if (degree[node] != 1
                    || (atStart ? segment.StartTerminal : segment.EndTerminal)
                    || math.any((atStart ? segment.StartCurveOffset : segment.EndCurveOffset) != 0))
                    continue;

                var curve = NetworkDesignChecks.Course(segment);
                var point = atStart ? curve.P0 : curve.P3;

                if (Math.Abs(point.x) > half || Math.Abs(point.z) > half)
                    continue;

                var inner = atStart ? curve.P1 : curve.P2;
                var outward = math.normalizesafe(point.xz - inner.xz);
                var reach = relaxed ? RelaxedReach : segment.ProtectedLink ? RampReach : AvenueReach;
                var r = (int)Math.Ceiling(reach / CellSize);
                var (cx, cz) = Cell(point.xz);
                Sample? best = null;
                var gap = reach;

                if (!relaxed && Rejoined(i, node, point) is Sample merged)
                {
                    ends.Add((new(i, atStart, point, outward, node), merged, math.distance(merged.Point, point.xz)));
                    rejoined++;

                    continue;
                }

                for (var dz = -r; dz <= r; dz++)
                    for (var dx = -r; dx <= r; dx++)
                    {
                        if (!cells.TryGetValue((cx + dx, cz + dz), out var list))
                            continue;

                        foreach (var coarse in list)
                        {
                            if (coarse.Section == i || math.distance(coarse.Point, point.xz) >= gap + SampleSpacing)
                                continue;

                            // The road may pass much nearer than its nearest sample.
                            var sample =
                                Nearest(
                                    coarse.Section,
                                    point.xz,
                                    Math.Max(0, coarse.T - sampleStep[coarse.Section]),
                                    Math.Min(1, coarse.T + sampleStep[coarse.Section]));

                            var otherSegment = source[sample.Section];
                            var otherDistance = math.distance(sample.Point, point.xz);

                            if (otherDistance >= gap
                                || otherSegment.WayId == segment.WayId && otherDistance > TightGap
                                || VisualNetworkCompiler.IsMainline(otherSegment) && (relaxed || !segment.ProtectedLink)
                                || Math.Abs(sample.Y - point.y) > MaximumRise)
                                continue;

                            if (relaxed)
                            {
                                if (math.dot(math.normalizesafe(sample.Point - point.xz), outward) >= RelaxedHeading)
                                {
                                    gap = otherDistance;
                                    best = sample;
                                }

                                continue;
                            }

                            if (otherDistance > TightGap
                                && math.dot(math.normalizesafe(sample.Point - point.xz), outward) < MinimumHeading)
                                continue;

                            // A road running alongside is not one this end was heading for.
                            if (otherDistance > TightGap
                                && Math.Abs(
                                    math.dot(
                                        math.normalizesafe(
                                            NetworkDesignChecks.Derivative(
                                                NetworkDesignChecks.Course(otherSegment),
                                                sample.T).xz),
                                        outward))
                                    > MaximumAlong)
                                continue;

                            gap = otherDistance;
                            best = sample;
                        }
                    }

                if (best != null)
                    ends.Add((new(i, atStart, point, outward, node), best.Value, gap));
            }
        }

        long next = long.MinValue + 47000000;

        long Fresh()
        {
            while (used.Contains(next))
                next++;

            used.Add(next);

            return next;
        }

        var moves = new Dictionary<(int Section, bool AtStart), (long Node, double3 Point)>();
        var splits = new Dictionary<int, List<(double T, long Node)>>();
        var welded = 0;
        var split = 0;

        foreach (var (end, target, _) in ends.OrderBy(item => item.Gap).ThenBy(item => item.End.Node))
        {
            token.ThrowIfCancellationRequested();

            if (degree[end.Node] != 1)
                continue;

            var otherSegment = source[target.Section];
            var curve = NetworkDesignChecks.Course(otherSegment);
            var length = NetworkDesignChecks.HorizontalArcLength(curve);
            var meet = NetworkDesignChecks.Evaluate(curve, target.T);
            // A road this end already moved onto carries its new node, not its old one.
            var startMoved = moves.TryGetValue((target.Section, true), out var movedStart);
            var endMoved = moves.TryGetValue((target.Section, false), out var movedEnd);
            var startNode = startMoved ? movedStart.Node : otherSegment.StartNodeId;
            var endNode = endMoved ? movedEnd.Node : otherSegment.EndNodeId;
            long node;
            double3 at;
            var alongStart = target.T * length;
            var alongEnd = (1 - target.T) * length;

            if (alongStart <= NodeSnap && alongStart <= alongEnd && !startMoved)
            {
                node = startNode;
                at = curve.P0;
                welded++;
            }
            else if (alongEnd <= NodeSnap && !endMoved)
            {
                node = endNode;
                at = curve.P3;
                welded++;
            }
            else if (alongStart <= NodeSnap || alongEnd <= NodeSnap)
                continue;
            else
            {
                if (!splits.TryGetValue(target.Section, out var cuts))
                    splits[target.Section] = cuts = new();

                var existing =
                    cuts
                        .Where(item => Math.Abs(item.T - target.T) * length <= NodeSnap)
                        .Select(item => (long?)item.Node)
                        .FirstOrDefault();

                if (existing is long shared)
                {
                    node = shared;
                    at = NetworkDesignChecks.Evaluate(curve, cuts.First(item => item.Node == shared).T);
                    welded++;
                }
                else
                {
                    node = Fresh();
                    at = meet;
                    cuts.Add((target.T, node));
                    degree[node] = 2;
                    split++;
                }
            }

            if (node == (end.AtStart ? source[end.Section].EndNodeId : source[end.Section].StartNodeId))
                continue;

            moves[(end.Section, end.AtStart)] = (node, at);
            degree[end.Node] = 0;
            degree[node] = degree.GetValueOrDefault(node) + 1;
        }

        log.Count("Dead ends carried on to the road they stopped short of", moves.Count);
        log.Count("Roads split to take a closed dead end", split);
        log.Count("Dead ends taken to where a merge moved their junction", rejoined);
        log.Diagnostic(
            $"DEAD ENDS candidates={ends.Count}; closed={moves.Count}; atExistingNodes={welded}; splits={split}; avenueReach={AvenueReach}; rampReach={RampReach}");

        if (moves.Count == 0)
            return source;

        var output = new List<PreparedSegment>(source.Count + split);

        for (var i = 0; i < source.Count; i++)
        {
            var segment = source[i];

            if (moves.TryGetValue((i, true), out var firstItem))
                segment = Moved(segment, true, firstItem.Node, firstItem.Point);

            if (moves.TryGetValue((i, false), out var secondItem))
                segment = Moved(segment, false, secondItem.Node, secondItem.Point);

            if (!splits.TryGetValue(i, out var cuts))
            {
                output.Add(segment);
                continue;
            }

            var consumed = 0d;

            foreach (var (t, node) in cuts.OrderBy(item => item.T))
            {
                var (left, right) = NativeDesignUnits.Split(segment, (t - consumed) / (1 - consumed), node);
                consumed = t;
                output.Add(Plain(left));
                segment = Plain(right);
            }

            output.Add(segment);
        }

        return output;
    }

    // Split leaves a final deck for the compiler's stage; here the anchors and controls are the geometry.
    private static PreparedSegment Plain(PreparedSegment segment) => segment with
    {
        FinalDeckCurve = null,
        HorizontalEmissionReference = null,
        SourceNodeAliases =
            segment.SourceNodeAliases
                .Where(
                    alias => alias.CanonicalNodeId == segment.StartNodeId || alias.CanonicalNodeId == segment.EndNodeId)
                .ToArray()
    };

    private static PreparedSegment Moved(PreparedSegment segment, bool atStart, long node, double3 point)
    {
        var otherPoint = (float3)point;
        var old = atStart ? segment.Start : segment.End;
        var delta = otherPoint - old;
        var alias = new SourceNodeAlias(atStart ? segment.StartNodeId : segment.EndNodeId, node, false);

        IReadOnlyList<float3> Extended(IReadOnlyList<float3> line)
            => line.Count == 0
                ? line
                : atStart ? line.Prepend(otherPoint).ToArray() : line.Append(otherPoint).ToArray();

        var moved =
            segment with
            {
                SourceGeometry = Extended(segment.SourceGeometry),
                OriginalGeometry = Extended(segment.OriginalGeometry),
                SourceNodeAliases = segment.SourceNodeAliases.Append(alias).ToArray()
            };

        return atStart
            ? moved with
            {
                StartNodeId = node,
                Start = otherPoint,
                Control1 = (segment.Control1 ?? math.lerp(segment.Start, segment.End, 1f / 3)) + delta,
                Control2 = segment.Control2 ?? math.lerp(segment.Start, segment.End, 2f / 3)
            }
            :
            moved with
            {
                EndNodeId = node,
                End = otherPoint,
                Control1 = segment.Control1 ?? math.lerp(segment.Start, segment.End, 1f / 3),
                Control2 = (segment.Control2 ?? math.lerp(segment.Start, segment.End, 2f / 3)) + delta
            };
    }
}
