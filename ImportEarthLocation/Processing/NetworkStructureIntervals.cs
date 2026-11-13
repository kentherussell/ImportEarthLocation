using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Structure decisions on source sections: tunnel intervals, inferred water and
// highway bridges. Runs before VisualNetworkCompiler, which owns all geometry.
public static class NetworkStructureIntervals
{
    private static bool IsMainline(PreparedSegment segment)
        => segment.Kind == NetworkKind.Rail || segment.Classification is "motorway" or "trunk";

    private static bool ExplicitStructure(PreparedSegment segment)
        => segment.RawSourceTags.ContainsKey("bridge") || segment.RawSourceTags.ContainsKey("tunnel")
            || segment.RawSourceTags.ContainsKey("layer")
            || segment.SourceTags.ContainsKey("bridge")
            || segment.SourceTags.ContainsKey("tunnel")
            || segment.SourceTags.ContainsKey("layer");

    public static IReadOnlyList<PreparedSegment> TunnelIntervals(
        IReadOnlyList<PreparedSegment> source,
        ImportLog log,
        CancellationToken token)
    {
        var result = source.ToArray();
        var nodes = new Dictionary<(NetworkKind, long), List<int>>();

        for (var i = 0; i < result.Length; i++)
            foreach (var id in new[] { result[i].StartNodeId, result[i].EndNodeId })
            {
                var key = (result[i].Kind, id);

                if (!nodes.TryGetValue(key, out var list))
                    nodes[key] = list = new();

                list.Add(i);
            }

        var filled = 0;
        var visited = new HashSet<int>();

        for (var seed = 0; seed < result.Length; seed++)
        {
            var first = result[seed];

            if (first.Tunnel
                || first.Bridge
                || first.ProtectedLink
                || !IsMainline(first)
                || ExplicitStructure(first)
                || visited.Contains(seed))
                continue;

            token.ThrowIfCancellationRequested();
            var run = new HashSet<int> { seed };
            var boundary = new List<int>();
            var length = NetworkVerticalProfiles.ArcLength(first);

            foreach (var end in new[] { first.StartNodeId, first.EndNodeId })
            {
                var current = seed;
                var node = end;

                while (length <= 220)
                {
                    if (nodes[(first.Kind, node)].Count != 2)
                        break;

                    var options =
                        nodes[(first.Kind, node)]
                            .Where(
                                i => i != current
                                    && !result[i].ProtectedLink
                                    && result[i].OneWay == first.OneWay
                                    && result[i].Classification == first.Classification)
                            .ToArray();

                    if (options.Length != 1)
                        break;

                    var next = options[0];

                    if (run.Contains(next))
                        break;

                    var firstSegment = result[current];
                    var secondSegment = result[next];

                    if (firstSegment.OneWay && (firstSegment.StartNodeId == node) == (secondSegment.StartNodeId == node)
                        || NetworkBoundaryPolicy.EndpointMarker(firstSegment, firstSegment.StartNodeId == node)
                            || NetworkBoundaryPolicy.EndpointMarker(secondSegment, secondSegment.StartNodeId == node))
                        break;

                    if (result[next].Tunnel)
                    {
                        boundary.Add(next);
                        break;
                    }

                    if (result[next].Bridge)
                        break;

                    if (ExplicitStructure(result[next]))
                        break;

                    run.Add(next);
                    length += NetworkVerticalProfiles.ArcLength(result[next]);
                    current = next;
                    node = result[next].StartNodeId == node ? result[next].EndNodeId : result[next].StartNodeId;
                }
            }

            visited.UnionWith(run);

            if (length > 220 || boundary.Count != 2 || result[boundary[0]].Layer != result[boundary[1]].Layer)
                continue;

            foreach (var index in run)
            {
                result[index] =
                    result[index] with
                    {
                        Tunnel = true,
                        Bridge = false,
                        Layer = Math.Min(result[boundary[0]].Layer, result[boundary[1]].Layer)
                    };

                filled++;
            }
        }

        log.Count("Short tunnel surface interruptions absorbed", filled);
        log.Diagnostic(
            $"TUNNEL INTERVALS surfaceSectionsAbsorbed={filled}; maximumInterruptionMeters=220; geographicOverrides=false");

        return result;
    }

    public static IReadOnlyList<PreparedSegment> StructureIntervals(
        IReadOnlyList<PreparedSegment> source,
        float[] raster,
        double width,
        ImportLog log,
            CancellationToken token,
            Func<float3, float?>? water = null,
            bool tunnelIntervalsResolved = false)
    {
        var edges = (tunnelIntervalsResolved ? source : TunnelIntervals(source, log, token)).ToArray();
        var waterBridges = 0;

        if (water != null)
            for (var i = 0; i < edges.Length; i++)
            {
                var segment = edges[i];

                if (segment.Tunnel || segment.Bridge || ExplicitStructure(segment))
                    continue;

                var samples =
                    segment.Kind == NetworkKind.Road
                        ? Math.Max(8, (int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(segment) / 10))
                        : 8;

                for (var j = 0; j <= samples; j++)
                {
                    var t = j / (float)samples;
                    var point = NetworkNormalizer.Evaluate(segment, t);
                    var direction =
                        NetworkNormalizer.Evaluate(segment, Math.Min(1, t + .001f))
                            - NetworkNormalizer.Evaluate(segment, Math.Max(0, t - .001f));

                    var normal =
                        math.normalizesafe(new float3(-direction.z, 0, direction.x))
                            * Math.Max(4, segment.NativeWidth * .5f);

                    var crossing = false;

                    for (var side =
                        segment.Kind == NetworkKind.Road
                            ? -1
                            : 0; side <= (segment.Kind == NetworkKind.Road ? 1 : 0); side++)
                    {
                        var otherPoint = point + normal * side;
                        var stage = water(otherPoint);

                        if (!stage.HasValue || !math.isfinite(stage.Value))
                            continue;

                        var ground = NetworkProcessor.Sample(otherPoint.x, otherPoint.z, raster, width);

                        if (segment.Kind == NetworkKind.Road
                            ? stage.Value - ground < WaterLevels.RoadBridgeDepthMeters
                            : ground > stage.Value + .5f)
                            continue;

                        crossing = true;
                        break;
                    }

                    if (crossing)
                    {
                        edges[i] = segment with { Bridge = true };

                        if (segment.Kind == NetworkKind.Road)
                            waterBridges++;

                        break;
                    }
                }
            }

        if (water != null)
        {
            log.Count("Road bridges inferred from planned water depth", waterBridges);
            log.Diagnostic(
                $"ROAD WATER POLICY bridgeDepthMeters={WaterLevels.RoadBridgeDepthMeters}; groundFreeboardMeters={WaterLevels.RoadFreeboardMeters}; inferredBridges={waterBridges}; roadEdgesSampled=true; explicitStructuresRetained=true; nativeWaterObserved=false");
        }

        return edges;
    }

    // A ground road or railway over an opened buried waterway (WaterBodyPlanner.BuriedChannels)
    // or an untagged crossing of an open channel (WaterBodyPlanner.OpenChannels)
    // becomes a bridge across it. Otherwise water revalidation, which never
    // carves beside a ground road, leaves the river dammed by the embankment.
    // Span the complete overlap with the constructed cut, including parallel
    // and oblique approaches. Road half width and the two raster cells protected
    // by revalidation keep ground support outside the channel. A span ending
    // within 10 m of a section end takes the rest of the section.
    public static IReadOnlyList<PreparedSegment> CulvertBridges(
        IReadOnlyList<PreparedSegment> source,
        IReadOnlyList<(float3[] Line, float HalfWidth)> culverts,
        int resolution, double width, ImportLog log, CancellationToken token)
    {
        if (culverts.Count == 0)
            return source;

        var cell = width / (resolution - 1);
        // Channel pieces indexed by the 128 m grid cells their bounds touch, so
        // thousands of open waterways cost a lookup per section, not a scan.
        const double GridCell = 128;
        var pieces = new List<(int Culvert, double2 A, double2 B)>();
        var grid = new Dictionary<(int, int), List<int>>();

        void Cells(double2 vectorA, double2 vectorB, double margin, Action<(int, int)> visit)
        {
            for (var x =
                (int)Math.Floor(
                    (Math.Min(vectorA.x, vectorB.x) - margin)
                        / GridCell); x <= (int)Math.Floor((Math.Max(vectorA.x, vectorB.x) + margin) / GridCell); x++)
            {
                token.ThrowIfCancellationRequested();

                for (var z =
                    (int)Math.Floor(
                        (Math.Min(vectorA.y, vectorB.y) - margin)
                            / GridCell); z
                                <= (int)Math.Floor((Math.Max(vectorA.y, vectorB.y) + margin) / GridCell); z++)
                    visit((x, z));
            }
        }

        for (var c = 0; c < culverts.Count; c++)
            for (var m = 1; m < culverts[c].Line.Length; m++)
            {
                var vectorA = (double2)culverts[c].Line[m - 1].xz;
                var vectorB = (double2)culverts[c].Line[m].xz;
                var index = pieces.Count;
                pieces.Add((c, vectorA, vectorB));
                Cells(vectorA, vectorB, culverts[c].HalfWidth + 2 * cell, key =>
    {
        if (!grid.TryGetValue(key, out var list))
            grid[key] = list = new List<int>();

        list.Add(index);
    });
            }

        var occupied =
            new HashSet<long>(source.SelectMany(segment => new[] { segment.StartNodeId, segment.EndNodeId }));

        var next = long.MinValue + 40000000;

        long Id()
        {
            while (occupied.Contains(next))
                next++;

            occupied.Add(next);

            return next++;
        }

        var result = new List<PreparedSegment>();
        var bridged = 0;
        var candidates = new HashSet<int>();

        foreach (var segment in source)
        {
            token.ThrowIfCancellationRequested();

            if (segment.Bridge || segment.Tunnel)
            {
                result.Add(segment);
                continue;
            }

            var (points, at) = Polyline(segment);
            var n = points.Length - 1;
            var roadHalf = Math.Max(4, segment.NativeWidth * .5);
            candidates.Clear();

            for (var k = 1; k <= n; k++)
                Cells(points[k - 1], points[k], roadHalf, key =>
                {
                    if (grid.TryGetValue(key, out var list))
                        candidates.UnionWith(list);
                });

            if (candidates.Count == 0)
            {
                result.Add(segment);
                continue;
            }

            var spans = new List<(double From, double To)>();

            foreach (var piece in candidates.OrderBy(i => i))
            {
                var (c, c0, c1) = pieces[piece];

                for (var k = 1; k <= n; k++)
                {
                    var radius = culverts[c].HalfWidth + roadHalf + 2 * cell;

                    if (!ChannelSpan(points[k - 1], points[k], c0, c1, radius, out var entry, out var exit))
                        continue;

                    spans.Add((math.lerp(at[k - 1], at[k], entry), math.lerp(at[k - 1], at[k], exit)));
                }
            }

            if (spans.Count == 0)
            {
                result.Add(segment);
                continue;
            }

            Bridged(segment, at, spans, segment.Layer, Id, result);
            bridged++;
        }

        log.Count("Sections bridged over short culverts", bridged);
        log.Count("Sections bridged over water channel footprints", bridged);

        return result;
    }

    // Intersection of a road segment with a channel segment's capsule, in XZ
    // metres. A rectangle and its two circular caps cover the entire capsule;
    // its convexity makes the union of their parameter intervals continuous.
    private static bool ChannelSpan(
        double2 vectorA,
        double2 vectorB,
        double2 c,
        double2 d,
        double radius,
        out double entry,
        out double exit)
    {
        var delta = vectorB - vectorA;
        var squared = math.lengthsq(delta);
        var axis = d - c;
        var length = math.length(axis);
        var first = double.PositiveInfinity;
        var last = double.NegativeInfinity;

        void Include(double low, double high)
        {
            low = Math.Max(0, low);
            high = Math.Min(1, high);

            if (low <= high)
            {
                first = Math.Min(first, low);
                last = Math.Max(last, high);
            }
        }

        void Disk(double2 center)
        {
            if (squared < 1e-12)
            {
                if (math.distancesq(vectorA, center) <= radius * radius)
                    Include(0, 1);

                return;
            }

            var t = math.dot(center - vectorA, delta) / squared;
            var reach = (radius * radius - math.distancesq(vectorA + delta * t, center)) / squared;

            if (reach >= 0)
            {
                reach = Math.Sqrt(reach);
                Include(t - reach, t + reach);
            }
        }

        bool Slab(double origin, double slope, double low, double high, ref double from, ref double to)
        {
            if (Math.Abs(slope) < 1e-12)
                return origin >= low && origin <= high;

            var x = (low - origin) / slope;
            var y = (high - origin) / slope;
            from = Math.Max(from, Math.Min(x, y));
            to = Math.Min(to, Math.Max(x, y));

            return from <= to;
        }

        if (length > 1e-8)
        {
            axis /= length;
            var offset = vectorA - c;
            var low = 0d;
            var high = 1d;

            if (Slab(math.dot(offset, axis), math.dot(delta, axis), 0, length, ref low, ref high)
                && Slab(
                    NetworkDesignChecks.Cross(axis, offset),
                    NetworkDesignChecks.Cross(axis, delta),
                    -radius,
                    radius,
                    ref low,
                    ref high))
                Include(low, high);
        }

        Disk(c);
        Disk(d);
        entry = first;
        exit = last;

        return first <= last;
    }

    // Two ground highway-class roads (motorway, trunk or their links) crossing at
    // more than 10° without a shared node are grade separated; the map left
    // the upper one's bridge untagged (the I-495 express ramps over the
    // highway). At grade the game draws one through the other. The minor road
    // (a link before a mainline, then fewer lanes, then the higher way id)
    // becomes a bridge one layer above the other, over the other's half width
    // plus ClimbRoom either side, so the profile solver can lift it clear.
    public static IReadOnlyList<PreparedSegment> UnmappedCrossingBridges(
        IReadOnlyList<PreparedSegment> source,
        ImportLog log,
        CancellationToken token)
    {
        const double ClimbRoom = 30, Cell = 64;

        static bool Highway(PreparedSegment segment)
            => segment.Kind == NetworkKind.Road
                && !segment.Bridge
                && !segment.Tunnel
                && (segment.ProtectedLink
                    || segment.Classification is "motorway" or "trunk" or "motorway_link" or "trunk_link"
                    || NetworkLimitedAccess.Converted(segment));

        var candidates = Enumerable.Range(0, source.Count).Where(i => Highway(source[i])).ToArray();

        if (candidates.Length < 2)
            return source;

        var lines = new Dictionary<int, (double2[] Points, double[] At)>();

        foreach (var i in candidates)
            lines[i] = Polyline(source[i]);

        var grid = new Dictionary<(int, int), List<(int Section, int K)>>();

        foreach (var i in candidates)
            for (var k = 1; k < lines[i].Points.Length; k++)
            {
                var point = lines[i].Points[k];
                var key = ((int)Math.Floor(point.x / Cell), (int)Math.Floor(point.y / Cell));

                if (!grid.TryGetValue(key, out var list))
                    grid[key] = list = new();

                list.Add((i, k));
            }

        var spans = new Dictionary<int, List<(double From, double To)>>();
        var layers = new Dictionary<int, int>();
        var crossings = 0;

        foreach (var cell in grid)
            foreach (var (i, k) in cell.Value)
            {
                token.ThrowIfCancellationRequested();

                for (var dz = -1; dz <= 1; dz++)
                    for (var dx = -1; dx <= 1; dx++)
                        if (grid.TryGetValue((cell.Key.Item1 + dx, cell.Key.Item2 + dz), out var near))
                            foreach (var (j, m) in near)
                            {
                                // Each sample belongs to one cell, so a pair of samples is met once,
                                // from the lower section; a set of visited pairs grew to hundreds of
                                // millions in DC and Houston and ran out of memory.
                                if (j <= i)
                                    continue;

                                var firstSegment = source[i];
                                var secondSegment = source[j];

                                if (firstSegment.Layer != secondSegment.Layer
                                    || firstSegment.StartNodeId == secondSegment.StartNodeId
                                    || firstSegment.StartNodeId == secondSegment.EndNodeId
                                    || firstSegment.EndNodeId == secondSegment.StartNodeId
                                    || firstSegment.EndNodeId == secondSegment.EndNodeId)
                                    continue;

                                var (pa, sa) = lines[i];
                                var (pb, builder) = lines[j];

                                if (!Cross(pa[k - 1], pa[k], pb[m - 1], pb[m], out var u, out var v))
                                    continue;

                                var da = math.normalizesafe(pa[k] - pa[k - 1]);
                                var db = math.normalizesafe(pb[m] - pb[m - 1]);

                                if (Math.Abs(NetworkDesignChecks.Cross(da, db)) < Math.Sin(math.radians(10)))
                                    continue;

                                // A crossing within 5 m of a section end is a junction mapped with separate nodes.
                                var station = math.lerp(sa[k - 1], sa[k], u);
                                var t = math.lerp(builder[m - 1], builder[m], v);

                                if (station < 5
                                    || station > sa[sa.Length - 1] - 5
                                    || t < 5
                                    || t > builder[builder.Length - 1] - 5)
                                    continue;

                                var minorIsA =
                                    firstSegment.ProtectedLink != secondSegment.ProtectedLink
                                        ? firstSegment.ProtectedLink
                                        : firstSegment.Lanes != secondSegment.Lanes
                                            ? firstSegment.Lanes < secondSegment.Lanes
                                            : firstSegment.WayId > secondSegment.WayId;

                                var (minor, major, at) = minorIsA ? (i, secondSegment, station) : (j, firstSegment, t);
                                var reach =
                                    Math.Max(4, major.NativeWidth * .5)
                                        / Math.Max(.5, Math.Abs(NetworkDesignChecks.Cross(da, db)))
                                        + ClimbRoom;

                                if (!spans.TryGetValue(minor, out var list))
                                    spans[minor] = list = new();

                                list.Add((at - reach, at + reach));
                                layers[minor] =
                                    Math.Max(
                                        layers.TryGetValue(minor, out var l) ? l : source[minor].Layer,
                                        major.Layer + 1);

                                crossings++;
                            }
            }

        if (crossings == 0)
            return source;

        var occupied =
            new HashSet<long>(source.SelectMany(segment => new[] { segment.StartNodeId, segment.EndNodeId }));

        var next = long.MinValue + 42000000;

        long Id()
        {
            while (occupied.Contains(next))
                next++;

            occupied.Add(next);

            return next++;
        }

        var result = new List<PreparedSegment>();

        for (var i = 0; i < source.Count; i++)
            if (spans.TryGetValue(i, out var list))
                Bridged(source[i], lines[i].At, list, layers[i], Id, result);
            else
                result.Add(source[i]);

        log.Count("Unmapped highway crossings bridged", crossings);

        return result;
    }

    // Segments ab and cd intersect at a + u(b − a) = c + v(d − c).
    private static bool Cross(double2 vectorA, double2 vectorB, double2 c, double2 d, out double u, out double v)
    {
        var r = vectorB - vectorA;
        var point = d - c;
        var den = r.x * point.y - r.y * point.x;
        u = v = 0;

        if (Math.Abs(den) < 1e-12)
            return false;

        var w = c - vectorA;
        u = (w.x * point.y - w.y * point.x) / den;
        v = (w.x * r.y - w.y * r.x) / den;

        return u >= 0 && u <= 1 && v >= 0 && v <= 1;
    }

    // A section's course sampled every 2 m, with arc-length stations.
    private static (double2[] Points, double[] At) Polyline(PreparedSegment segment)
    {
        var curve = new Cubic3d(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd);
        var n = Math.Max(2, (int)Math.Ceiling(NetworkDesignChecks.HorizontalArcLength(curve) / 2));
        var points = new double2[n + 1];
        var at = new double[n + 1];

        for (var k = 0; k <= n; k++)
        {
            points[k] = NetworkDesignChecks.Evaluate(curve, k / (double)n).xz;

            if (k > 0)
                at[k] = at[k - 1] + math.distance(points[k - 1], points[k]);
        }

        return (points, at);
    }

    // Splits a section (stations from Polyline) so the spans become bridge
    // pieces at the given layer; spans are merged when within 10 m, and a span
    // ending within 10 m of a section end takes the rest of the section.
    private static void Bridged(
        PreparedSegment segment,
        double[] at,
        List<(double From, double To)> spans,
        int layer,
        Func<long> id,
        List<PreparedSegment> result)
    {
        var n = at.Length - 1;
        var total = at[n];
        var merged = new List<(double From, double To)>();

        foreach (var span in spans.OrderBy(item => item.From))
        {
            var from = span.From < 10 ? 0 : span.From;
            var to = span.To > total - 10 ? total : span.To;

            if (merged.Count > 0 && from <= merged[merged.Count - 1].To + 10)
                merged[merged.Count - 1] = (merged[merged.Count - 1].From, Math.Max(merged[merged.Count - 1].To, to));
            else
                merged.Add((from, to));
        }

        double FractionAt(double station)
        {
            var k = Array.BinarySearch(at, station);

            if (k >= 0)
                return k / (double)n;

            k = ~k;

            return (k - 1 + (station - at[k - 1]) / Math.Max(1e-9, at[k] - at[k - 1])) / n;
        }

        var stations =
            new[] { 0d }
                .Concat(merged.SelectMany(item => new[] { item.From, item.To }).Where(x => x > 0 && x < total))
                .Append(total)
                .ToArray();

        bool Over(int piece)
            => merged.Any(item => item.From <= stations[piece] + 1e-6 && item.To >= stations[piece + 1] - 1e-6);

        PreparedSegment Mark(PreparedSegment otherSegment, int piece)
            => Over(piece) ? otherSegment with { Bridge = true, Layer = layer } : otherSegment;

        var rest = segment;
        var consumed = 0d;

        for (var piece = 0; piece < stations.Length - 2; piece++)
        {
            var t = FractionAt(stations[piece + 1]);
            var (left, right) = NativeDesignUnits.Split(rest, (t - consumed) / (1 - consumed), id());
            result.Add(Mark(left with { FinalDeckCurve = null }, piece));
            rest = right with { FinalDeckCurve = null };
            consumed = t;
        }

        result.Add(Mark(rest, stations.Length - 2));
    }
}
