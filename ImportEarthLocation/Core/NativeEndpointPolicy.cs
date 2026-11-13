using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Unity.Mathematics;

namespace ImportEarthLocation.Core;

// The native node map compares float3 positions exactly (not within a tolerance).
// Canonicalize only already-shared topology; never snap nearby unrelated roads.
public static class NativeEndpointPolicy
{
    public static string Identity(PreparedSegment segment, long node) =>
        node == segment.StartNodeId && segment.StartLevelCrossing
            || node == segment.EndNodeId && segment.EndLevelCrossing
            ? $"Crossing:{node}"
            : $"{segment.Kind}:{node}";

    // A shared OSM node is evidence; mere geometric intersection is not. Reject
    // mixed structures/layers and conflicting horizontal positions as a whole.
    public static HashSet<long> LevelCrossings(IReadOnlyList<PreparedSegment> source)
    {
        var ports =
            source.SelectMany(
                segment => new[] {
                    (Section: segment, Node: segment.StartNodeId, Point: segment.CourseStart, Anchor: segment.Start),
                    (Section: segment, Node: segment.EndNodeId, Point: segment.CourseEnd, Anchor: segment.End)
                    });

        return ports
            .GroupBy(item => item.Node)
            .Where(
                group => group.Any(item => item.Section.Kind == NetworkKind.Road)
                    && group.Any(item => item.Section.Kind == NetworkKind.Rail)
                    && group.All(
                item => !NetworkStructure.IsStructure(item.Section)
                    && item.Section.Layer == group.First().Section.Layer
                    && math.all(item.Point.xz == group.First().Point.xz)
                    && math.all(item.Anchor.xz == group.First().Anchor.xz)
                    && math.all(math.isfinite(item.Point))
                    && math.all(math.isfinite(item.Anchor))))
            .Select(group => group.Key).ToHashSet();
    }

    // A level crossing needs its road and rail ends at exactly one horizontal position.
    // The road fit computes its nodes in doubles and can leave a crossing's road end a
    // rounding residual away from the rail's (-4.5e-20 m against 0 in RailCrossingTests),
    // which LevelCrossings, comparing exactly, then rejects. Road ends within a millimetre
    // of the rail end at a shared, non-structure node on the same layer take the rail's
    // horizontal position; heights are reconciled after, as before.
    private static IReadOnlyList<PreparedSegment> AlignCrossingPorts(
        IReadOnlyList<PreparedSegment> source,
        ImportLog log)
    {
        const float Residual = .001f;
        var railEnds = new Dictionary<long, (float3 Point, float3 Anchor, int Layer)>();

        foreach (var segment in source)
            if (segment.Kind == NetworkKind.Rail && !NetworkStructure.IsStructure(segment))
            {
                railEnds[segment.StartNodeId] = (segment.CourseStart, segment.Start, segment.Layer);
                railEnds[segment.EndNodeId] = (segment.CourseEnd, segment.End, segment.Layer);
            }

        if (railEnds.Count == 0)
            return source;

        bool Close(float3 point, float3 target)
            => !math.all(point.xz == target.xz) && math.distance(point.xz, target.xz) <= Residual;

        var aligned = 0;

        var result = source.Select(segment =>
        {
            if (segment.Kind != NetworkKind.Road || NetworkStructure.IsStructure(segment))
                return segment;

            var start =
                railEnds.TryGetValue(segment.StartNodeId, out var startRail)
                    && startRail.Layer == segment.Layer
                    && (Close(segment.CourseStart, startRail.Point) || Close(segment.Start, startRail.Anchor));

            var end =
                railEnds.TryGetValue(segment.EndNodeId, out var endRail)
                    && endRail.Layer == segment.Layer
                    && (Close(segment.CourseEnd, endRail.Point) || Close(segment.End, endRail.Anchor));

            if (!start && !end)
                return segment;

            float3 Onto(float3 point, float3 target) => new(target.x, point.y, target.z);

            aligned += (start ? 1 : 0) + (end ? 1 : 0);

            return segment with
            {
                Start = start ? Onto(segment.Start, startRail.Anchor) : segment.Start,
                End = end ? Onto(segment.End, endRail.Anchor) : segment.End,
                FinalDeckCurve =
                    segment.FinalDeckCurve is { } deck
                        ? new(
                            start ? Onto(deck.P0, startRail.Point) : deck.P0,
                            deck.P1,
                            deck.P2,
                            end ? Onto(deck.P3, endRail.Point) : deck.P3)
                        : null
            };
        }).ToArray();

        log.Count("Level crossing road ends aligned to the rail", aligned);

        return result;
    }

    public static IReadOnlyList<PreparedSegment> PrepareLevelCrossings(
        IReadOnlyList<PreparedSegment> source,
        ImportLog log)
    {
        source = AlignCrossingPorts(source, log);
        var crossings = LevelCrossings(source);
        var heights =
            source
                .Where(segment => segment.Kind == NetworkKind.Rail)
                .SelectMany(
                    segment => new[] {
                        (Node: segment.StartNodeId, Point: segment.CourseStart),
                        (Node: segment.EndNodeId, Point: segment.CourseEnd)
                        })
            .Where(item => crossings.Contains(item.Node))
                .GroupBy(item => item.Node)
                .ToDictionary(group => group.Key, group => group.First().Point.y);

        // Only rounding-size residuals may be canonicalized here. Larger profile
        // failures remain separate, reported crossings rather than warped decks.
        foreach (var segment in source)
            foreach (var item in new[] {
                (Node: segment.StartNodeId, Point: segment.CourseStart),
                (Node: segment.EndNodeId, Point: segment.CourseEnd)
                })
                if (heights.TryGetValue(item.Node, out var y) && Math.Abs(item.Point.y - y) > .25f)
                    crossings.Remove(item.Node);

        var result = source.Select(segment =>
        {
            var contains = crossings.Contains(segment.StartNodeId);
            var otherContains = crossings.Contains(segment.EndNodeId);

            if (!contains && !otherContains)
                return segment.StartLevelCrossing || segment.EndLevelCrossing
                    ? segment with { StartLevelCrossing = false, EndLevelCrossing = false }
                    : segment;

            var da = contains ? new float3(0, heights[segment.StartNodeId] - segment.CourseStart.y, 0) : float3.zero;
            var db = otherContains ? new float3(0, heights[segment.EndNodeId] - segment.CourseEnd.y, 0) : float3.zero;

            return segment with
            {
                Start = segment.Start + da,
                End = segment.End + db,
                Control1 = contains ? segment.CourseControl1 - segment.StartCurveOffset + da : segment.Control1,
                Control2 = otherContains ? segment.CourseControl2 - segment.EndCurveOffset + db : segment.Control2,
                FinalDeckCurve =
                    segment.FinalDeckCurve.HasValue
                        ? new(
                            segment.CourseStart + da,
                            segment.CourseControl1 + da,
                            segment.CourseControl2 + db,
                            segment.CourseEnd + db)
                        : null,
                StartLevelCrossing = contains,
                EndLevelCrossing = otherContains
            };
        }).ToArray();

        log.Count("Native level crossings prepared", crossings.Count);
        log.Count("Native level crossings rejected for height mismatch", heights.Count - crossings.Count);

        return result;
    }

    public static IReadOnlyList<PreparedSegment> Canonicalize(IReadOnlyList<PreparedSegment> source, ImportLog log)
    {
        var positions = new Dictionary<string, float3>(StringComparer.Ordinal);
        var changed = 0;
        var conflicts = 0;
        var result = new List<PreparedSegment>(source.Count);

        float3 Point(PreparedSegment segment, long node, float3 point)
        {
            var key = Identity(segment, node);

            if (!positions.TryGetValue(key, out var canonical))
            {
                positions[key] = point;
                return point;
            }

            if (math.distance(canonical, point) > .25f)
            {
                if (conflicts++ < 25)
                    log.Warn(
                        $"Native endpoint conflict {key}: {math.distance(canonical, point):0.###} m; not snapping unrelated/conflicting geometry.");

                return point;
            }

            if (!math.all(canonical == point))
                changed++;

            return canonical;
        }

        foreach (var segment in source)
        {
            var vectorA = Point(segment, segment.StartNodeId, segment.Start);
            var vectorB = Point(segment, segment.EndNodeId, segment.End);
            result.Add(
                segment with
                {
                    Start = vectorA,
                    End = vectorB,
                    Control1 = segment.Control1.HasValue ? segment.Control1 + (vectorA - segment.Start) : null,
                    Control2 = segment.Control2.HasValue ? segment.Control2 + (vectorB - segment.End) : null
                });
        }

        log.Count("Native endpoints canonicalized", changed);
        log.Count("Native endpoint position conflicts", conflicts);
        log.Diagnostic(
            $"NATIVE ENDPOINTS canonicalized={changed}; conflicts={conflicts}; toleranceMeters=0.25; exactNativeKeys=true; proximityConnectionsCreated=false; sectionsDeleted=0");

        return result;
    }

    // The game makes a node within 0.1 m of the terrain bounds an outside connection.
    // Design can leave a clipped end a few metres short (lane datum, smoothing), and
    // then it is a dead end: anything that close goes to the exact edge.
    public static IReadOnlyList<PreparedSegment> SnapToBorder(
        IReadOnlyList<PreparedSegment> source,
        double width,
        ImportLog log)
    {
        const float Reach = 5;
        var half = (float)(width / 2);
        var result = source.ToArray();
        var snapped = 0;
        var degree = new Dictionary<string, int>(StringComparer.Ordinal);

        foreach (var segment in source)
            foreach (var node in new[] { segment.StartNodeId, segment.EndNodeId })
            {
                var key = Identity(segment, node);
                degree[key] = degree.GetValueOrDefault(key) + 1;
            }

        float3 Edge(float3 point)
            => Math.Abs(point.x) >= Math.Abs(point.z)
                ? new float3(point.x >= 0 ? half : -half, point.y, point.z)
                : new float3(point.x, point.y, point.z >= 0 ? half : -half);

        for (var i = 0; i < result.Length; i++)
        {
            var segment = result[i];

            if (degree[Identity(segment, segment.StartNodeId)] == 1
                && half - Math.Max(Math.Abs(segment.Start.x), Math.Abs(segment.Start.z)) is var a
                && a > 0
                && a <= Reach)
            {
                var d = Edge(segment.Start) - segment.Start;
                var deck = segment.FinalDeckCurve;
                segment =
                    segment with
                    {
                        Start = segment.Start + d,
                        Control1 = segment.Control1.HasValue ? segment.Control1 + d : null,
                        FinalDeckCurve =
                            deck.HasValue
                                ? new(deck.Value.P0 + d, deck.Value.P1 + d, deck.Value.P2, deck.Value.P3)
                                : null
                    };

                snapped++;
            }

            if (degree[Identity(segment, segment.EndNodeId)] == 1
                && half - Math.Max(Math.Abs(segment.End.x), Math.Abs(segment.End.z)) is var b
                && b > 0
                && b <= Reach)
            {
                var d = Edge(segment.End) - segment.End;
                var deck = segment.FinalDeckCurve;
                segment =
                    segment with
                    {
                        End = segment.End + d,
                        Control2 = segment.Control2.HasValue ? segment.Control2 + d : null,
                        FinalDeckCurve =
                            deck.HasValue
                                ? new(deck.Value.P0, deck.Value.P1, deck.Value.P2 + d, deck.Value.P3 + d)
                                : null
                    };

                snapped++;
            }

            result[i] = segment;
        }

        log.Count("Road ends snapped to the map edge", snapped);

        return snapped == 0 ? source : result;
    }

    // The game draws a tunnel only when its edge and node elevations are all at most
    // -10 m and the deepest -15 m or lower (NetCompositionHelpers.GetElevationFlags,
    // at the road assets' 5 m limit); anything shallower is an open cut, whatever the
    // section's flag. A section inside a bore, with ground enough to hold it, takes an
    // elevation that draws as tunnel, though its deck stays where it was designed.
    public static IReadOnlyList<PreparedSegment> InteriorTunnelElevations(
        IReadOnlyList<PreparedSegment> source,
        float[] terrain,
        double width,
        ImportLog log)
    {
        const float Drawn = -16, MinimumDepth = Processing.NetworkDrawnStructures.TunnelCover;
        var open = new HashSet<string>(StringComparer.Ordinal);
        var changed = 0;

        foreach (var segment in source)
            if (!segment.Tunnel)
            {
                open.Add(Identity(segment, segment.StartNodeId));
                open.Add(Identity(segment, segment.EndNodeId));
            }

        var result = source.Select(segment =>
        {
            if (!segment.Tunnel)
                return segment;

            float? Value(long node, float3 point, float? elevation)
            {
                if (open.Contains(Identity(segment, node))
                    || Processing.NetworkProcessor.Sample(point.x, point.z, terrain, width) - point.y
                        < MinimumDepth
                    || (elevation ?? 0) <= Drawn)
                    return elevation;

                changed++;

                return Drawn;
            }

            return segment with
            {
                StartElevation = Value(segment.StartNodeId, segment.CourseStart, segment.StartElevation),
                EndElevation = Value(segment.EndNodeId, segment.CourseEnd, segment.EndElevation)
            };
        }).ToArray();

        log.Count("Tunnel ends given an elevation that draws as tunnel", changed);

        return result;
    }

    public static Dictionary<string, float> Elevations(IReadOnlyList<PreparedSegment> source)
    {
        var values = new Dictionary<string, List<float>>(StringComparer.Ordinal);

        foreach (var segment in source)
            foreach (var start in new[] { true, false })
            {
                var key = Identity(segment, start ? segment.StartNodeId : segment.EndNodeId);

                if (!values.TryGetValue(key, out var list))
                    values[key] = list = new List<float>();

                var elevation =
                    NetworkStructure.IsStructure(segment)
                        ? (start ? segment.StartElevation : segment.EndElevation) ?? NetworkStructure.Offset(segment)
                        : 0;

                // A rail bridge abutment can be ground-supported, never a negative
                // cutting composition. Tunnel elevations remain genuinely negative.
                if (segment.Kind == NetworkKind.Rail && segment.Bridge && !segment.Tunnel)
                    elevation = Math.Max(0, elevation);

                list.Add(elevation);
            }

        // Mixed ground/structure portals use one node composition on both courses;
        // edge structure flags remain per-edge. Native collection otherwise keeps
        // the first endpoint's flags/elevation, making mixed portals order-dependent.
        return values.ToDictionary(
            entry => entry.Key,
            entry => entry.Value.Any(v => v > 0) ? entry.Value.Max() : entry.Value.Min(),
            StringComparer.Ordinal);
    }
}
