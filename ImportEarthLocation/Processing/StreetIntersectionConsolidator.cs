using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// A divided surface crossing is sometimes mapped as a small directed box.
// Recover its single junction before pairing the approaching carriageways.
public static class StreetIntersectionConsolidator
{
    public static IReadOnlyList<PreparedSegment> Prepare(
        IReadOnlyList<PreparedSegment> source,
        ImportLog log,
        CancellationToken token)
    {
        bool Street(PreparedSegment segment)
            => segment.Kind == NetworkKind.Road
                && !NativeAssetKeys.IsHighway(segment)
                && !segment.Bridge
                && !segment.Tunnel
                && !segment.Roundabout
                && !segment.StartMiniRoundabout && !segment.EndMiniRoundabout;

        string Identity(PreparedSegment segment)
            => segment.Name.Trim().Length > 0
                ? segment.Name.Trim().ToUpperInvariant()
                : segment.Reference.Trim().ToUpperInvariant();

        var incident =
            source
                .SelectMany(
                    segment => new[] {
                        (Id: segment.StartNodeId, Edge: segment),
                        (Id: segment.EndNodeId, Edge: segment)
                        })
            .GroupBy(item => item.Id)
                .ToDictionary(group => group.Key, group => group.Select(item => item.Edge).ToArray());

        var small =
            source
                .Where(
                    segment => Street(segment)
                        && segment.OneWay
                        && math.distance(segment.Start.xz, segment.End.xz) <= 48
                        && math.distance(segment.Start.xz, segment.End.xz) >= 2)
                .ToArray();

        var outgoing =
            small.GroupBy(segment => segment.StartNodeId).ToDictionary(group => group.Key, group => group.ToArray());

        var remap = new Dictionary<long, long>();
        var positions = new Dictionary<long, float3>();
        var removed = new HashSet<PreparedSegment>();
        var count = 0;

        foreach (var firstSegment in small.OrderBy(segment => segment.StartNodeId).ThenBy(segment => segment.EndNodeId))
        {
            token.ThrowIfCancellationRequested();

            if (remap.ContainsKey(firstSegment.StartNodeId)
                || !outgoing.TryGetValue(firstSegment.EndNodeId, out var second))
                continue;

            foreach (var secondSegment in second)
                foreach (var segment in outgoing.TryGetValue(secondSegment.EndNodeId, out var third)
                    ? third
                    : Array.Empty<PreparedSegment>())
                    foreach (var otherSegment in outgoing.TryGetValue(segment.EndNodeId, out var fourth)
                        ? fourth
                        : Array.Empty<PreparedSegment>())
                    {
                        if (otherSegment.EndNodeId != firstSegment.StartNodeId)
                            continue;

                        var box = new[] { firstSegment, secondSegment, segment, otherSegment };
                        var nodes = box.Select(thirdSegment => thirdSegment.StartNodeId).ToArray();

                        if (nodes.Distinct().Count() != 4
                            || nodes.Any(remap.ContainsKey)
                            || nodes.Min() != firstSegment.StartNodeId)
                            continue;

                        var all = nodes.SelectMany(id => incident[id]).Distinct().ToArray();

                        if (all.Any(thirdSegment => !Street(thirdSegment) || thirdSegment.Layer != firstSegment.Layer))
                            continue;

                        var center =
                            box.Aggregate(float3.zero, (point, thirdSegment) => point + thirdSegment.Start) / 4;

                        if (box.Any(thirdSegment => math.distance(thirdSegment.Start.xz, center.xz) > 32))
                            continue;

                        var rectangular = true;

                        for (var k = 0; k < 4; k++)
                            if (Math.Abs(math.dot(math.normalizesafe((box[k].End - box[k].Start).xz),
                                                math.normalizesafe((box[(k + 1) % 4].End - box[(k + 1) % 4].Start).xz)))
                                                    > .5f)
                                rectangular = false;

                        if (!rectangular)
                            continue;

                        var internalEdges =
                            all
                                .Where(
                                    thirdSegment => nodes.Contains(thirdSegment.StartNodeId)
                                        && nodes.Contains(thirdSegment.EndNodeId))
                                .ToArray();

                        if (internalEdges.Any(thirdSegment => !box.Contains(thirdSegment)))
                            continue;

                        var external = all.Except(box).ToArray();

                        if (external.Length < 6)
                            continue;

                        var axis = math.normalizesafe((firstSegment.End - firstSegment.Start).xz);
                        var normal = new float2(-axis.y, axis.x);
                        var groups = new List<(PreparedSegment Edge, bool Outgoing)>[4];

                        for (var k = 0; k < 4; k++)
                            groups[k] = new();

                        var valid = true;

                        foreach (var arm in external)
                        {
                            var atStart = nodes.Contains(arm.StartNodeId);
                            var away = math.normalizesafe((atStart ? arm.End - arm.Start : arm.Start - arm.End).xz);
                            var x = math.dot(away, axis);
                            var y = math.dot(away, normal);
                            var parallel = Math.Abs(x) >= Math.Abs(y);

                            if (!arm.OneWay
                                || Identity(arm).Length == 0
                                || Math.Max(Math.Abs(x), Math.Abs(y)) < Math.Cos(20 * Math.PI / 180))
                            {
                                valid = false;
                                break;
                            }

                            groups[parallel ? (x > 0 ? 0 : 2) : (y > 0 ? 1 : 3)].Add((arm, atStart));
                        }

                        if (!valid || groups.Count(items => items.Count > 0) < 3)
                            continue;

                        if (groups
                            .Where(items => items.Count > 0)
                            .Any(
                                items => items.Count != 2
                                    || items[0].Outgoing == items[1].Outgoing
                                    || Identity(items[0].Edge) != Identity(items[1].Edge)))
                            continue;

                        for (var k = 0; k < 2; k++)
                            if (groups[k].Count > 0
                                && groups[k + 2].Count > 0
                                && Identity(groups[k][0].Edge) != Identity(groups[k + 2][0].Edge))
                                valid = false;

                        if (!valid)
                            continue;

                        foreach (var id in nodes)
                        {
                            remap[id] = firstSegment.StartNodeId;
                            positions[id] = center;
                        }

                        foreach (var edge in box)
                            removed.Add(edge);

                        count++;
                    }
        }

        if (count == 0)
            return source;

        var result = new List<PreparedSegment>(source.Count - removed.Count);

        foreach (var segment in source)
        {
            if (removed.Contains(segment))
                continue;

            if (!remap.ContainsKey(segment.StartNodeId) && !remap.ContainsKey(segment.EndNodeId))
            {
                result.Add(segment);
                continue;
            }

            var point = positions.TryGetValue(segment.StartNodeId, out var start) ? start : segment.Start;
            var otherPoint = positions.TryGetValue(segment.EndNodeId, out var end) ? end : segment.End;
            result.Add(segment with
            {
                StartNodeId = remap.TryGetValue(segment.StartNodeId, out var first) ? first : segment.StartNodeId,
                EndNodeId = remap.TryGetValue(segment.EndNodeId, out var last) ? last : segment.EndNodeId,
                Start = point,
                End = otherPoint,
                SourceGeometry = new[] { point, otherPoint },
                Control1 = null,
                Control2 = null,
                HorizontalParent = null,
                ParentStartT = 0,
                ParentEndT = 1,
                StartCurveOffset = float3.zero,
                EndCurveOffset = float3.zero
            });
        }

        log.Count("Divided street intersection boxes consolidated", count);
        log.Diagnostic(
            $"STREET INTERSECTIONS boxes={count}; explicitDirectedCycle=true; opposingNamedApproaches=true; majorRoadsAndStructuresProtected=true");

        return SourceTopologyObligations.Transfer(source, result, NetworkKind.Road, remap);
    }
}
