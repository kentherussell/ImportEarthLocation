using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

/// <summary>Conservative network-only topology repairs. Never moves terrain or merges carriageways.</summary>
public static class NetworkHardener
{
    private static bool SameLevel(PreparedSegment firstSegment, PreparedSegment secondSegment)
        => firstSegment.Kind == secondSegment.Kind
            && firstSegment.Bridge == secondSegment.Bridge
            && firstSegment.Tunnel == secondSegment.Tunnel
            && firstSegment.Layer == secondSegment.Layer;

    private static bool ConflictingNames(PreparedSegment firstSegment, PreparedSegment secondSegment) =>
        firstSegment.Reference.Length > 0
            && secondSegment.Reference.Length > 0
            && !firstSegment.Reference.Equals(secondSegment.Reference, StringComparison.OrdinalIgnoreCase)
            || firstSegment.Name.Length > 0
            && secondSegment.Name.Length > 0
            && !firstSegment.Name.Equals(secondSegment.Name, StringComparison.OrdinalIgnoreCase)
            && !(firstSegment.Reference.Length > 0
                && firstSegment.Reference.Equals(secondSegment.Reference, StringComparison.OrdinalIgnoreCase));

    private static bool NamedMatch(PreparedSegment firstSegment, PreparedSegment secondSegment) =>
        firstSegment.Reference.Length > 0
            && firstSegment.Reference.Equals(secondSegment.Reference, StringComparison.OrdinalIgnoreCase)
            || firstSegment.Name.Length > 0
                && firstSegment.Name.Equals(secondSegment.Name, StringComparison.OrdinalIgnoreCase);

    private static float2 Heading(PreparedSegment segment) => math.normalizesafe(segment.End.xz - segment.Start.xz);

    private static float Length(PreparedSegment segment) => math.distance(segment.Start.xz, segment.End.xz);

    public static IReadOnlyList<PreparedSegment> Prepare(
        IReadOnlyList<PreparedSegment> source,
        ImportLog log,
        CancellationToken token,
        bool rail)
    {
        var roads = source.Where(segment => segment.Kind == NetworkKind.Road).ToList();
        var tracks = source.Where(segment => segment.Kind == NetworkKind.Rail).ToList();

        if (roads.Count > 0)
        {
            log.Diagnostic($"PROCESS road-consolidation duplicate-removal; sections={roads.Count}");
            roads = RemoveCoincident(roads, log, token);
            log.Diagnostic($"PROCESS road-consolidation divided-corridor-association; sections={roads.Count}");
            roads = AssociateCorridors(roads, log, token);
            log.Diagnostic($"PROCESS road-consolidation crossing-safety; sections={roads.Count}");
            roads = ProtectCarriageways(roads, log, token);
            log.Diagnostic($"PROCESS road-consolidation carriageway-widths; sections={roads.Count}");
            roads = ConstrainWidths(roads, token);
            log.Diagnostic($"PROCESS road-consolidation bridge-approaches; sections={roads.Count}");
            roads = ReconnectBridges(roads, log, token);
            log.Diagnostic($"PROCESS road-consolidation complete; sections={roads.Count}");
        }

        // Rail corridors are consolidated on raw topology before curve fitting.
        // Do not run the old all-pairs yard deletion pass on fitted sections.
        return roads.Concat(rail ? tracks : Enumerable.Empty<PreparedSegment>()).ToArray();
    }

    private static List<PreparedSegment> RemoveCoincident(
        List<PreparedSegment> source,
        ImportLog log,
        CancellationToken token)
    {
        var buckets = new Dictionary<(int, int, int, int), List<PreparedSegment>>();
        var kept = new List<PreparedSegment>();
        var remap = new Dictionary<(int, bool, bool, long), long>();

        foreach (var segment in source
            .OrderBy(otherSegment => otherSegment.WayId)
            .ThenBy(otherSegment => otherSegment.StartNodeId))
        {
            token.ThrowIfCancellationRequested();
            var vectorA = segment.Start.xz;
            var vectorB = segment.End.xz;

            if (!segment.OneWay && (vectorA.x > vectorB.x || vectorA.x == vectorB.x && vectorA.y > vectorB.y))
            {
                var swap = vectorA;
                vectorA = vectorB;
                vectorB = swap;
            }

            var key =
                (
                    (int)Math.Round(vectorA.x * 10),
                    (int)Math.Round(vectorA.y * 10),
                    (int)Math.Round(vectorB.x * 10),
                    (int)Math.Round(vectorB.y * 10));

            if (!buckets.TryGetValue(key, out var candidates))
                buckets[key] = candidates = new List<PreparedSegment>();

            PreparedSegment? duplicate = null;

            foreach (var other in candidates)
            {
                if (!SameLevel(segment, other)
                    || segment.Classification != other.Classification
                    || segment.Lanes != other.Lanes
                    || segment.OneWay != other.OneWay
                    || ConflictingNames(segment, other))
                    continue;

                var reversed = math.distance(segment.Start.xz, other.End.xz) < .05f;

                if (Enumerable
                    .Range(0, 33)
                    .All(
                        i => math.distance(
                            NetworkNormalizer.Evaluate(segment, i / 32f).xz,
                            NetworkNormalizer.Evaluate(other, (reversed ? 32 - i : i) / 32f).xz)
                            < .05f))
                {
                    duplicate = other;
                    break;
                }
            }

            if (duplicate == null)
            {
                kept.Add(segment);
                candidates.Add(segment);
                continue;
            }

            var reverse = math.distance(segment.Start.xz, duplicate.End.xz) < .05f;
            remap[(segment.Layer, segment.Bridge, segment.Tunnel, segment.StartNodeId)] =
                reverse ? duplicate.EndNodeId : duplicate.StartNodeId;

            remap[(segment.Layer, segment.Bridge, segment.Tunnel, segment.EndNodeId)] =
                reverse ? duplicate.StartNodeId : duplicate.EndNodeId;

            log.Count("Coincident road representations removed");
        }

        long Map(PreparedSegment segment, long node)
        {
            for (var i =
                0; i < 16
                    && remap.TryGetValue((segment.Layer, segment.Bridge, segment.Tunnel, node), out var next)
                    && next != node; i++)
                node = next;

            return node;
        }

        return kept
            .Select(
                segment => segment with
                {
                    StartNodeId = Map(segment, segment.StartNodeId),
                    EndNodeId = Map(segment, segment.EndNodeId)
                })
            .ToList();
    }

    private static List<PreparedSegment> AssociateCorridors(
        List<PreparedSegment> source,
        ImportLog log,
        CancellationToken token)
    {
        var grid = Index(source, 128);
        var evidence = new Dictionary<(long, long), (float overlap, int count, bool named)>();

        foreach (var firstSegment in source.Where(segment => segment.OneWay && !segment.ProtectedLink))
        {
            token.ThrowIfCancellationRequested();

            foreach (var j in Neighbors(grid, (firstSegment.Start.xz + firstSegment.End.xz) * .5f, 128))
            {
                var secondSegment = source[j];

                if (firstSegment.WayId >= secondSegment.WayId
                    || !secondSegment.OneWay
                    || secondSegment.ProtectedLink
                    || firstSegment.Classification != secondSegment.Classification
                    || !SameLevel(firstSegment, secondSegment)
                    || ConflictingNames(firstSegment, secondSegment))
                    continue;

                var ha = Heading(firstSegment);
                var hb = Heading(secondSegment);

                if (math.dot(ha, hb) > -.97f)
                    continue;

                var length = Length(firstSegment);

                if (length < 10 || Length(secondSegment) < 10)
                    continue;

                var relative = secondSegment.Start.xz - firstSegment.Start.xz;
                var separation = Math.Abs(ha.x * relative.y - ha.y * relative.x);

                if (separation < 2 || separation > 40)
                    continue;

                var endSeparation =
                    Math.Abs(
                        ha.x * (secondSegment.End.z - firstSegment.Start.z)
                            - ha.y * (secondSegment.End.x - firstSegment.Start.x));

                if (Math.Abs(endSeparation - separation) > 3)
                    continue;

                var p = math.dot(relative, ha);
                var q = math.dot(secondSegment.End.xz - firstSegment.Start.xz, ha);
                var overlap = Math.Min(length, Math.Max(p, q)) - Math.Max(0, Math.Min(p, q));

                if (overlap < Math.Min(length, Length(secondSegment)) * .7f)
                    continue;

                var key = (firstSegment.WayId, secondSegment.WayId);
                evidence.TryGetValue(key, out var old);
                evidence[key] =
                    (old.overlap + overlap, old.count + 1, old.named || NamedMatch(firstSegment, secondSegment));
            }
        }

        var parent = new Dictionary<long, long>();

        long Root(long id)
        {
            if (!parent.TryGetValue(id, out var p))
                return id;

            while (parent.TryGetValue(p, out var next) && next != p)
                p = next;

            return p;
        }

        foreach (var pair in evidence.OrderBy(entry => entry.Key.Item1).ThenBy(entry => entry.Key.Item2))
        {
            if (!(pair.Value.named && pair.Value.overlap >= 30 || pair.Value.count >= 3 && pair.Value.overlap >= 150))
            {
                log.Count("Ambiguous parallel road pairs preserved");
                continue;
            }

            var root = Root(pair.Key.Item1);
            var otherRoot = Root(pair.Key.Item2);

            if (root == otherRoot)
                continue;

            parent[Math.Max(root, otherRoot)] = Math.Min(root, otherRoot);
            parent[Math.Min(root, otherRoot)] = Math.Min(root, otherRoot);
            log.Count("Divided corridors detected");
        }

        var paired =
            source
                .Select(
                    segment => parent.ContainsKey(segment.WayId)
                        ? segment with { CorridorId = Root(segment.WayId) }
                        : segment)
                .ToList();

        return paired;
    }

    public static List<PreparedSegment> ConstrainWidths(List<PreparedSegment> paired, CancellationToken token)
    {
        var pairedGrid = Index(paired, 128);
        var curveSamples = new float3[paired.Count][];
        var headings = new float2[paired.Count];

        for (var i = 0; i < paired.Count; i++)
        {
            token.ThrowIfCancellationRequested();
            headings[i] = Heading(paired[i]);

            if (paired[i].CorridorId == 0)
                continue;

            curveSamples[i] = new float3[17];

            for (var sample = 0; sample <= 16; sample++)
                curveSamples[i][sample] = NetworkNormalizer.Evaluate(paired[i], sample / 16f);
        }

        for (var i = 0; i < paired.Count; i++)
        {
            token.ThrowIfCancellationRequested();
            var firstSegment = paired[i];

            if (firstSegment.CorridorId == 0)
                continue;

            var limit = float.PositiveInfinity;

            foreach (var j in Neighbors(pairedGrid, (firstSegment.Start.xz + firstSegment.End.xz) * .5f, 128))
            {
                var secondSegment = paired[j];

                if (firstSegment.WayId == secondSegment.WayId
                    || firstSegment.CorridorId != secondSegment.CorridorId
                    || !SameLevel(firstSegment, secondSegment)
                    || math.dot(headings[i], headings[j]) > -.9f)
                    continue;

                // A genuine shared source junction is handled by the native junction
                // builder; its zero separation is not a carriageway width failure.
                if (firstSegment.StartNodeId == secondSegment.StartNodeId
                    || firstSegment.StartNodeId == secondSegment.EndNodeId
                    || firstSegment.EndNodeId == secondSegment.StartNodeId
                    || firstSegment.EndNodeId == secondSegment.EndNodeId)
                    continue;

                for (var sample = 0; sample <= 8; sample++)
                {
                    var point = curveSamples[i][sample * 2];
                    var closest = float.PositiveInfinity;

                    for (var step = 1; step <= 16; step++)
                        closest =
                            Math.Min(
                                closest,
                                NetworkNormalizer.DistanceToSegment(
                                    point,
                                    curveSamples[j][step - 1],
                                    curveSamples[j][step]));

                    if (closest < 40)
                        limit = Math.Min(limit, Math.Max(0, closest - 1));
                }
            }

            paired[i] = firstSegment with { MaximumNativeWidth = limit };
        }

        return paired;
    }

    private static List<PreparedSegment> ProtectCarriageways(
        List<PreparedSegment> source,
        ImportLog log,
        CancellationToken token)
    {
        HashSet<int> Crossed(List<PreparedSegment> items)
        {
            var crossed = new HashSet<int>();
            var grid = Index(items, 128);

            for (var i = 0; i < items.Count; i++)
            {
                token.ThrowIfCancellationRequested();
                var firstSegment = items[i];

                if (firstSegment.CorridorId == 0)
                    continue;

                foreach (var j in Neighbors(grid, (firstSegment.Start.xz + firstSegment.End.xz) * .5f, 128))
                {
                    var secondSegment = items[j];

                    if (i >= j
                        || firstSegment.WayId == secondSegment.WayId
                        || firstSegment.CorridorId != secondSegment.CorridorId
                        || !SameLevel(firstSegment, secondSegment)
                        || firstSegment.StartNodeId == secondSegment.StartNodeId
                        || firstSegment.StartNodeId == secondSegment.EndNodeId
                        || firstSegment.EndNodeId == secondSegment.StartNodeId
                        || firstSegment.EndNodeId == secondSegment.EndNodeId)
                        continue;

                    bool collision = false;

                    for (var p = 1; p <= 16 && !collision; p++)
                        for (var q = 1; q <= 16; q++)
                            if (Intersects(
                                NetworkNormalizer.Evaluate(firstSegment, (p - 1) / 16f).xz,
                                NetworkNormalizer.Evaluate(firstSegment, p / 16f).xz,
                                NetworkNormalizer.Evaluate(secondSegment, (q - 1) / 16f).xz,
                                NetworkNormalizer.Evaluate(secondSegment, q / 16f).xz))
                            {
                                collision = true;
                                break;
                            }

                    if (collision)
                    {
                        crossed.Add(i);
                        crossed.Add(j);
                    }
                }
            }

            return crossed;
        }

        var collisions = Crossed(source);

        if (collisions.Count == 0)
            return source;

        var restored = new List<PreparedSegment>();

        for (var index = 0; index < source.Count; index++)
        {
            var segment = source[index];

            if (!collisions.Contains(index) || segment.SourceGeometry.Count < 2)
            {
                restored.Add(segment);
                continue;
            }

            var refitted = NetworkNormalizer.Refit(segment, token, .5f);
            restored.AddRange(refitted);
            log.Count("Carriageway curves locally refitted");
            log.Diagnostic(
                $"CURVE RECOVERY way={segment.WayId}; inputObservations={segment.SourceGeometry.Count}; fittedSections={refitted.Count}; toleranceMeters=0.5; rawChordFallback=false; sharedOuterEndpointsRetained=true");
        }

        var unsafeIndices = Crossed(restored);

        if (unsafeIndices.Count > 0)
        {
            log.Count("Carriageway conflicts retained for review", unsafeIndices.Count);
            log.Warn(
                $"{unsafeIndices.Count} carriageway sections still conflict after source-geometry repair; source topology retained, no sections deleted. Preparation continues; overlaps require review.");
        }

        return restored;
    }

    private static bool Intersects(float2 vectorA, float2 vectorB, float2 c, float2 d)
    {
        float Cross(float2 otherVectorA, float2 otherVectorB)
            => otherVectorA.x * otherVectorB.y - otherVectorA.y * otherVectorB.x;

        var r = vectorB - vectorA;
        var s = d - c;
        var divisor = Cross(r, s);

        if (Math.Abs(divisor) < .00001f)
            return false;

        var t = Cross(c - vectorA, s) / divisor;
        var u = Cross(c - vectorA, r) / divisor;

        return t >= -.0001f && t <= 1.0001f && u >= -.0001f && u <= 1.0001f;
    }

    private static List<PreparedSegment> ReconnectBridges(
        List<PreparedSegment> source,
        ImportLog log,
        CancellationToken token)
    {
        var degree = new Dictionary<long, int>();

        foreach (var segment in source)
            foreach (var nodeId in new[] { segment.StartNodeId, segment.EndNodeId })
            {
                degree.TryGetValue(nodeId, out var d);
                degree[nodeId] = d + 1;
            }

        var grid = Index(source, 128);
        var result = new List<PreparedSegment>(source);
        var used = new HashSet<long>();
        long synthetic = -9_000_000_000_000_000_000;

        foreach (var bridge in source.Where(segment => segment.Bridge && !segment.Tunnel))
            foreach (var atStart in new[] { true, false })
            {
                token.ThrowIfCancellationRequested();
                var node = atStart ? bridge.StartNodeId : bridge.EndNodeId;

                if (degree[node] > 1 || used.Contains(node))
                    continue;

                var position = atStart ? bridge.Start : bridge.End;
                var outward = Heading(bridge) * (atStart ? -1 : 1);
                var candidates = new List<(PreparedSegment road, bool start, float distance)>();

                foreach (var i in Neighbors(grid, position.xz, 128))
                {
                    var road = source[i];

                    if (road.WayId == bridge.WayId && road.Bridge
                        || road.Tunnel
                        || road.Bridge
                        || road.Classification != bridge.Classification
                        || road.OneWay != bridge.OneWay
                        || ConflictingNames(bridge, road))
                        continue;

                    if (!(road.WayId == bridge.WayId
                        || NamedMatch(bridge, road)
                        || road.CorridorId != 0 && road.CorridorId == bridge.CorridorId))
                        continue;

                    foreach (var roadStart in new[] { true, false })
                    {
                        var roadNode = roadStart ? road.StartNodeId : road.EndNodeId;

                        if (degree[roadNode] != 1 || used.Contains(roadNode))
                            continue;

                        if (bridge.OneWay && roadStart == atStart)
                            continue; // incoming end -> bridge start, bridge end -> outgoing start

                        var endpoint = roadStart ? road.Start : road.End;
                        var delta = endpoint.xz - position.xz;
                        var distance = math.length(delta);

                        if (distance < .05f || distance > 30 || math.dot(math.normalizesafe(delta), outward) < .97f)
                            continue;

                        var heading = Heading(road) * (roadStart ? 1 : -1);

                        if (math.dot(heading, outward) < .97f)
                            continue;

                        if (Math.Abs(outward.x * delta.y - outward.y * delta.x) > 1)
                            continue; // never hop sideways onto the paired deck

                        candidates.Add((road, roadStart, distance));
                    }
                }

                candidates = candidates.OrderBy(item => item.distance).ThenBy(item => item.road.WayId).ToList();

                if (candidates.Count == 0
                    || candidates.Count > 1 && candidates[1].distance - candidates[0].distance < 3)
                {
                    log.Count("Bridge approaches unresolved");
                    continue;
                }

                var best = candidates[0];
                var roadPoint = best.start ? best.road.Start : best.road.End;
                var roadId = best.start ? best.road.StartNodeId : best.road.EndNodeId;
                var connector =
                    best.road with
                    {
                        WayId = synthetic++,
                        Start = atStart ? roadPoint : position,
                        End = atStart ? position : roadPoint,
                        StartNodeId = atStart ? roadId : node,
                        EndNodeId = atStart ? node : roadId,
                        Control1 = null,
                        Control2 = null,
                        SourceGeometry = new[] { atStart ? roadPoint : position, atStart ? position : roadPoint },
                        CorridorId = bridge.CorridorId
                    };

                result.Add(connector);
                used.Add(node);
                used.Add(roadId);
                log.Count("Bridge approaches resolved");
            }

        return result;
    }

    private static Dictionary<(int, int), List<int>> Index(List<PreparedSegment> source, float cell)
    {
        var index = new Dictionary<(int, int), List<int>>();

        for (var i = 0; i < source.Count; i++)
        {
            var segment = source[i];
            var point = (segment.Start.xz + segment.End.xz) * .5f;
            var key = ((int)Math.Floor(point.x / cell), (int)Math.Floor(point.y / cell));

            if (!index.TryGetValue(key, out var list))
                index[key] = list = new List<int>();

            list.Add(i);
        }

        return index;
    }

    private static IEnumerable<int> Neighbors(Dictionary<(int, int), List<int>> grid, float2 point, float cell)
    {
        var x = (int)Math.Floor(point.x / cell);
        var y = (int)Math.Floor(point.y / cell);

        for (var dy = -2; dy <= 2; dy++)
            for (var dx = -2; dx <= 2; dx++)
                if (grid.TryGetValue((x + dx, y + dy), out var found))
                    foreach (var i in found)
                        yield return i;
    }
}
