using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

public static class RoundaboutPlanner
{
    private static float[] _stockRadii = Array.Empty<float>();

    public static void ConfigureStockRadii(IEnumerable<float> radii)
        => _stockRadii = radii.Where(r => math.isfinite(r) && r > 0).Distinct().OrderBy(r => r).ToArray();

    public static IReadOnlyList<PreparedSegment> Prepare(
        IReadOnlyList<PreparedSegment> source,
        ImportLog log,
        CancellationToken token)
    {
        log.Diagnostic(
            $"ROUNDABOUT native palette sizes={string.Join(",", _stockRadii)}; maximumCenterlineRadiusMeters={(_stockRadii.Length > 0 ? _stockRadii[_stockRadii.Length - 1] : 0)}; irregularRingsRetained=true");

        var rings =
            source
                .Where(
                    segment => segment.Roundabout
                        && segment.Kind == NetworkKind.Road
                        && !segment.Bridge
                        && !segment.Tunnel)
                .ToArray();

        var incident = new Dictionary<long, List<PreparedSegment>>();

        foreach (var segment in source)
            foreach (var id in new[] { segment.StartNodeId, segment.EndNodeId })
            {
                if (!incident.TryGetValue(id, out var list))
                    incident[id] = list = new();

                list.Add(segment);
            }

        var byNode = new Dictionary<long, List<PreparedSegment>>();

        foreach (var segment in rings)
            foreach (var id in new[] { segment.StartNodeId, segment.EndNodeId })
            {
                if (!byNode.TryGetValue(id, out var list))
                    byNode[id] = list = new();

                list.Add(segment);
            }

        var visited = new HashSet<PreparedSegment>();
        var replacements = new Dictionary<PreparedSegment, PreparedSegment>();
        var extraArcs = new Dictionary<PreparedSegment, List<PreparedSegment>>();
        var removed = new HashSet<PreparedSegment>();
        var remap = new Dictionary<long, (long id, float3 point, float radius)>();
        var ringPositions = new Dictionary<long, float3>();
        var circles = new Dictionary<long, (float3 center, float radius)>();
        var allIds = new HashSet<long>(source.SelectMany(segment => new[] { segment.StartNodeId, segment.EndNodeId }));
        var next = long.MinValue + 2000000;
        var smooth = 0;
        var compact = 0;

        foreach (var seed in rings)
        {
            token.ThrowIfCancellationRequested();

            if (visited.Contains(seed))
                continue;

            var group = new HashSet<PreparedSegment>();
            var queue = new List<PreparedSegment> { seed };

            for (var q = 0; q < queue.Count; q++)
            {
                var segment = queue[q];

                if (!group.Add(segment))
                    continue;

                visited.Add(segment);

                foreach (var id in new[] { segment.StartNodeId, segment.EndNodeId })
                    foreach (var otherSegment in byNode[id])
                        if (!group.Contains(otherSegment))
                            queue.Add(otherSegment);
            }

            var edges =
                group
                    .OrderBy(segment => segment.WayId)
                    .ThenBy(segment => segment.StartNodeId)
                    .ThenBy(segment => segment.EndNodeId)
                    .ToArray();

            var ids = new HashSet<long>(edges.SelectMany(segment => new[] { segment.StartNodeId, segment.EndNodeId }));

            if (ids.Any(id => byNode[id].Count != 2) || edges.Length < 3)
                continue; // Incomplete rings must not collapse.

            var length = edges.Sum(segment => math.distance(segment.Start.xz, segment.End.xz));

            if (length < 10)
                continue;

            var center = default(float3);

            foreach (var segment in edges)
                center +=
                    (segment.Start + segment.End) * .5f * math.distance(segment.Start.xz, segment.End.xz) / length;

            var radius =
                edges.Sum(
                    segment => (math.distance(segment.Start.xz, center.xz) + math.distance(segment.End.xz, center.xz))
                        * .5f
                        * math.distance(segment.Start.xz, segment.End.xz))
                    / length;

            if (radius < 3)
                continue;

            var circular =
                edges.All(
                    segment => Math.Abs(math.distance(segment.Start.xz, center.xz) - radius)
                        <= Math.Max(2f, radius * .2f));

            var approaches =
                ids.SelectMany(id => incident[id]).Where(segment => !group.Contains(segment)).Distinct().ToArray();

            var stockRadii = _stockRadii;
            var stockRadius =
                stockRadii.Length == 0
                    ? 0
                    : stockRadii.OrderBy(r => Math.Abs(r - radius)).ThenByDescending(r => r).First();

            var entries = approaches.Count(segment => !segment.OneWay || ids.Contains(segment.EndNodeId));
            var exits = approaches.Count(segment => !segment.OneWay || ids.Contains(segment.StartNodeId));

            // A ring no larger than the game's largest roundabout becomes one junction node that
            // carries a stock roundabout, whatever its number of roads: the game places one on
            // any node, a dead end included.
            if (circular
                && stockRadius > 0
                && radius <= stockRadii[stockRadii.Length - 1]
                && approaches.Length >= 1
                && entries > 0
                && exits > 0
                && ids.All(
                    id => edges.Count(segment => segment.StartNodeId == id) == 1
                        && edges.Count(segment => segment.EndNodeId == id) == 1)
                    && edges.All(segment => segment.Layer == edges[0].Layer)
                    && approaches.All(
                        segment => segment.Kind == NetworkKind.Road
                            && !segment.Bridge
                            && !segment.Tunnel
                            && segment.Layer == edges[0].Layer
                            && !(ids.Contains(segment.StartNodeId) && ids.Contains(segment.EndNodeId))
                        && math.distance(segment.Start.xz, segment.End.xz) > stockRadius + 12))
            {
                while (allIds.Contains(next))
                    next++;

                var node = next++;
                allIds.Add(node);

                foreach (var id in ids)
                    remap[id] = (node, center, stockRadius);

                removed.UnionWith(group);
                compact++;
                log.Diagnostic(
                    $"ROUNDABOUT native-node planned; sourceWays={string.Join(",", edges.Select(segment => segment.WayId).Distinct())}; sourceRadiusMeters={radius:0.##}; selectedStockCenterlineRadiusMeters={stockRadius}; approaches={approaches.Length}; nativeRadiusDerivedFromStockGeometry=true");

                continue;
            }

            foreach (var segment in edges)
            {
                replacements[segment] =
                    segment with
                    {
                        Classification = "primary",
                        OneWay = true,
                        Lanes = 2,
                        Control1 = math.lerp(segment.Start, segment.End, 1f / 3),
                        Control2 = math.lerp(segment.Start, segment.End, 2f / 3)
                    };

                if (!circular)
                {
                    var before = byNode[segment.StartNodeId].First(otherSegment => otherSegment != segment);
                    var after = byNode[segment.EndNodeId].First(otherSegment => otherSegment != segment);
                    var point = before.StartNodeId == segment.StartNodeId ? before.End : before.Start;
                    var otherPoint = after.StartNodeId == segment.EndNodeId ? after.End : after.Start;
                    var irregularReach = math.distance(segment.Start.xz, segment.End.xz) / 3;
                    var irregularStart = math.normalizesafe((segment.End - point).xz);
                    var irregularEnd = math.normalizesafe((otherPoint - segment.Start).xz);
                    replacements[segment] =
                        segment with
                        {
                            Classification = "primary",
                            OneWay = true,
                            Lanes = 2,
                            Control1 =
                                segment.Start + new float3(irregularStart.x, 0, irregularStart.y) * irregularReach,
                            Control2 = segment.End - new float3(irregularEnd.x, 0, irregularEnd.y) * irregularReach,
                            SourceGeometry = new[] { segment.Start, segment.End }
                        };

                    smooth++;
                    continue;
                }

                var vectorA = segment.Start.xz - center.xz;
                var vectorB = segment.End.xz - center.xz;
                var cross = vectorA.x * vectorB.y - vectorA.y * vectorB.x;
                var angle =
                    (float)Math.Acos(
                        math.clamp(math.dot(math.normalizesafe(vectorA), math.normalizesafe(vectorB)), -1, 1));

                var start =
                    center
                        + new float3(
                            math.normalizesafe(vectorA).x * radius,
                            segment.Start.y - center.y,
                            math.normalizesafe(vectorA).y * radius);

                var end =
                    center
                        + new float3(
                            math.normalizesafe(vectorB).x * radius,
                            segment.End.y - center.y,
                            math.normalizesafe(vectorB).y * radius);

                ringPositions[segment.StartNodeId] = start;
                ringPositions[segment.EndNodeId] = end;
                circles[segment.StartNodeId] = (center, radius);
                circles[segment.EndNodeId] = (center, radius);

                if (angle < .001f)
                {
                    replacements[segment] =
                        replacements[segment] with
                        {
                            Start = start,
                            End = end,
                            Control1 = math.lerp(start, end, 1f / 3),
                            Control2 = math.lerp(start, end, 2f / 3)
                        };

                    continue;
                }

                if (angle > (float)Math.PI / 2 + .001f)
                {
                    var ringSign =
                        edges.Sum(
                            otherSegment => (otherSegment.Start.x - center.x) * (otherSegment.End.z - center.z)
                                - (otherSegment.Start.z - center.z) * (otherSegment.End.x - center.x))
                            >= 0
                            ? 1f
                            : -1f;

                    var arcSign = Math.Abs(cross) < .01f ? ringSign : cross >= 0 ? 1f : -1f;
                    var beginAngle = (float)Math.Atan2(vectorA.y, vectorA.x);
                    var arcCount = (int)Math.Ceiling(angle / ((float)Math.PI / 2));
                    var arcStart = start;
                    var arcStartId = segment.StartNodeId;
                    var extras = new List<PreparedSegment>();

                    for (var k = 1; k <= arcCount; k++)
                    {
                        var theta = beginAngle + arcSign * angle * k / arcCount;
                        var arcEnd =
                            k == arcCount
                                ? end
                                : center
                                    + new float3(
                                        (float)Math.Cos(theta) * radius,
                                        math.lerp(start.y, end.y, k / (float)arcCount) - center.y,
                                        (float)Math.Sin(theta) * radius);

                        var arcEndId = segment.EndNodeId;

                        if (k < arcCount)
                        {
                            while (allIds.Contains(next))
                                next++;

                            arcEndId = next++;
                            allIds.Add(arcEndId);
                            circles[arcEndId] = (center, radius);
                        }

                        var va = arcStart.xz - center.xz;
                        var vb = arcEnd.xz - center.xz;
                        var at = math.normalizesafe(new float2(-va.y, va.x)) * arcSign;
                        var bt = math.normalizesafe(new float2(-vb.y, vb.x)) * arcSign;
                        var arcReach = 4f / 3 * radius * (float)Math.Tan(angle / arcCount / 4);
                        var part = segment with
                        {
                            Classification = "primary",
                            OneWay = true,
                            Lanes = 2,
                            Start = arcStart,
                            End = arcEnd,
                            StartNodeId = arcStartId,
                            EndNodeId = arcEndId,
                            Control1 = arcStart + new float3(at.x, 0, at.y) * arcReach,
                            Control2 = arcEnd - new float3(bt.x, 0, bt.y) * arcReach,
                            SourceGeometry = new[] { arcStart, arcEnd }
                        };

                        if (k == 1)
                            replacements[segment] = part;
                        else
                            extras.Add(part);

                        arcStart = arcEnd;
                        arcStartId = arcEndId;
                        smooth++;
                    }

                    extraArcs[segment] = extras;
                    continue;
                }

                var sign = cross >= 0 ? 1f : -1f;
                var ta = math.normalizesafe(new float2(-vectorA.y, vectorA.x)) * sign;
                var tb = math.normalizesafe(new float2(-vectorB.y, vectorB.x)) * sign;
                var reach = 4f / 3 * radius * (float)Math.Tan(angle / 4);
                var c1 = start + new float3(ta.x, 0, ta.y) * reach;
                var c2 = end - new float3(tb.x, 0, tb.y) * reach;
                var candidate =
                    segment with
                    {
                        Classification = "primary",
                        OneWay = true,
                        Lanes = 2,
                        Start = start,
                        End = end,
                        Control1 = c1,
                        Control2 = c2,
                        SourceGeometry = new[] { start, end }
                    };

                replacements[segment] = candidate;
                smooth++;
            }
        }

        var result = new List<PreparedSegment>();

        foreach (var original in source)
        {
            if (removed.Contains(original))
                continue;

            var segment = replacements.TryGetValue(original, out var fitted) ? fitted : original;

            if (!original.Roundabout)
            {
                if (ringPositions.TryGetValue(segment.StartNodeId, out var ringStart))
                    segment = segment with { Start = ringStart, Control1 = null, Control2 = null };

                if (ringPositions.TryGetValue(segment.EndNodeId, out var ringEnd))
                    segment = segment with { End = ringEnd, Control1 = null, Control2 = null };
            }

            if (remap.TryGetValue(segment.StartNodeId, out var firstItem))
                segment =
                    segment with
                    {
                        StartNodeId = firstItem.id,
                        Start = firstItem.point,
                        StartRoundaboutRadius = firstItem.radius,
                        Control1 = null,
                        Control2 = null,
                        SourceGeometry = Array.Empty<float3>()
                    };

            if (remap.TryGetValue(segment.EndNodeId, out var secondItem))
                segment =
                    segment with
                    {
                        EndNodeId = secondItem.id,
                        End = secondItem.point,
                        EndRoundaboutRadius = secondItem.radius,
                        Control1 = null,
                        Control2 = null,
                        SourceGeometry = Array.Empty<float3>()
                    };

            if (segment.Kind == NetworkKind.Road && !segment.Bridge && !segment.Tunnel)
            {
                if (segment.StartMiniRoundabout
                    && incident.TryGetValue(segment.StartNodeId, out var startArms)
                    && startArms.Count >= 3)
                    segment = segment with { StartRoundaboutRadius = Math.Max(segment.StartRoundaboutRadius, 6) };

                if (segment.EndMiniRoundabout
                    && incident.TryGetValue(segment.EndNodeId, out var endArms)
                    && endArms.Count >= 3)
                    segment = segment with { EndRoundaboutRadius = Math.Max(segment.EndRoundaboutRadius, 6) };
            }

            result.Add(segment);

            if (extraArcs.TryGetValue(original, out var extras))
                result.AddRange(extras);
        }

        // Compact only degree-two circle nodes; never remove an approach attachment.
        var compactedArcs = 0;
        var compactedIrregular = 0;
        var arcRemoved = new HashSet<PreparedSegment>();
        var ringMerged = new Dictionary<long, long>();
        var degree =
            result
                .SelectMany(segment => new[] { segment.StartNodeId, segment.EndNodeId })
                .GroupBy(nodeId => nodeId)
                .ToDictionary(group => group.Key, group => group.Count());

        var outgoing =
            result
                .Where(segment => segment.Roundabout)
                .GroupBy(segment => segment.StartNodeId)
                .ToDictionary(group => group.Key, group => group.ToArray());

        for (var i = 0; i < result.Count; i++)
        {
            token.ThrowIfCancellationRequested();

            if (arcRemoved.Contains(result[i]))
                continue;

            while (true)
            {
                var firstSegment = result[i];

                if (!firstSegment.Roundabout
                    || degree[firstSegment.EndNodeId] != 2
                    || !outgoing.TryGetValue(firstSegment.EndNodeId, out var nextArcs)
                    || nextArcs.Length != 1)
                    break;

                var secondSegment = nextArcs[0];

                if (firstSegment == secondSegment
                    || arcRemoved.Contains(secondSegment)
                    || firstSegment.StartNodeId == secondSegment.EndNodeId)
                    break;

                if (!circles.TryGetValue(firstSegment.EndNodeId, out var circle))
                {
                    if (!firstSegment.Control1.HasValue
                        || !secondSegment.Control2.HasValue
                        || math.distance(firstSegment.Start.xz, secondSegment.End.xz) > 160)
                        break;

                    var taIrregular = math.normalizesafe((firstSegment.Control1.Value - firstSegment.Start).xz);
                    var tbIrregular = math.normalizesafe((secondSegment.End - secondSegment.Control2.Value).xz);

                    if (math.dot(taIrregular, tbIrregular) < .6f)
                        break;

                    var irregularReach = math.distance(firstSegment.Start.xz, secondSegment.End.xz) / 3;
                    var originalPoints =
                        firstSegment.SourceGeometry.Concat(secondSegment.SourceGeometry.Skip(1)).ToArray();

                    var irregular = firstSegment with
                    {
                        End = secondSegment.End,
                        EndNodeId = secondSegment.EndNodeId,
                        Control1 = firstSegment.Start + new float3(taIrregular.x, 0, taIrregular.y) * irregularReach,
                        Control2 = secondSegment.End - new float3(tbIrregular.x, 0, tbIrregular.y) * irregularReach,
                        SourceWayIds =
                            firstSegment.SourceWayIds.Concat(secondSegment.SourceWayIds).Distinct().ToArray(),
                        SourceNodeAliases =
                            firstSegment.SourceNodeAliases.Concat(secondSegment.SourceNodeAliases).Distinct().ToArray(),
                        OriginalGeometry =
                            firstSegment.OriginalGeometry.Concat(secondSegment.OriginalGeometry).ToArray(),
                        SourceGeometry = originalPoints
                    };

                    if (!NetworkNormalizer.WithinTolerance(irregular, 3f))
                        break;

                    ringMerged[firstSegment.EndNodeId] = firstSegment.StartNodeId;
                    result[i] = irregular;
                    outgoing[firstSegment.StartNodeId] = new[] { irregular };
                    outgoing.Remove(secondSegment.StartNodeId);
                    arcRemoved.Add(secondSegment);
                    compactedIrregular++;
                    continue;
                }

                var startVector = firstSegment.Start.xz - circle.center.xz;
                var endVector = secondSegment.End.xz - circle.center.xz;
                var startAngle = (float)Math.Atan2(startVector.y, startVector.x);
                var endAngle = (float)Math.Atan2(endVector.y, endVector.x);
                var firstDelta = firstSegment.End.xz - firstSegment.Start.xz;
                var sign = startVector.x * firstDelta.y - startVector.y * firstDelta.x >= 0 ? 1f : -1f;
                var sweep = (endAngle - startAngle) * sign;

                while (sweep < 0)
                    sweep += 2 * (float)Math.PI;

                if (sweep > (float)Math.PI / 2 + .001f)
                    break;

                var reach = 4f / 3 * circle.radius * (float)Math.Tan(sweep / 4);
                var ta = math.normalizesafe(new float2(-startVector.y, startVector.x)) * sign;
                var tb = math.normalizesafe(new float2(-endVector.y, endVector.x)) * sign;
                var merged = firstSegment with
                {
                    End = secondSegment.End,
                    EndNodeId = secondSegment.EndNodeId,
                    Control1 = firstSegment.Start + new float3(ta.x, 0, ta.y) * reach,
                    Control2 = secondSegment.End - new float3(tb.x, 0, tb.y) * reach,
                    SourceWayIds = firstSegment.SourceWayIds.Concat(secondSegment.SourceWayIds).Distinct().ToArray(),
                    SourceNodeAliases =
                        firstSegment.SourceNodeAliases.Concat(secondSegment.SourceNodeAliases).Distinct().ToArray(),
                    OriginalGeometry = firstSegment.OriginalGeometry.Concat(secondSegment.OriginalGeometry).ToArray(),
                    SourceGeometry = new[] { firstSegment.Start, secondSegment.End }
                };

                ringMerged[firstSegment.EndNodeId] = firstSegment.StartNodeId;
                result[i] = merged;
                outgoing[firstSegment.StartNodeId] = new[] { merged };
                outgoing.Remove(secondSegment.StartNodeId);
                arcRemoved.Add(secondSegment);
                compactedArcs++;
            }
        }

        result.RemoveAll(arcRemoved.Contains);
        log.Count("Roundabout degree-two nodes simplified", compactedArcs);
        log.Count("Irregular roundabout degree-two nodes simplified", compactedIrregular);
        log.Diagnostic(
            $"ROUNDABOUT PREPARATION curvedRingSections={smooth}; nativeSmallJunctions={compact}; circularizedRingNodes={ringPositions.Count}; taggedMiniApproaches={result.Count(segment => segment.StartMiniRoundabout || segment.EndMiniRoundabout)}; incompleteRingsPreserved=true; oversizedRingPalette=regular-oneway-two-lanes");

        // A ring node merged into its arc keeps its identity as an alias of the arc's start.
        foreach (var pair in remap)
            ringMerged[pair.Key] = pair.Value.id;

        return SourceTopologyObligations.Transfer(source, result, NetworkKind.Road, ringMerged);
    }
}
