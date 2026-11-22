using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Consolidation can project the two ends of ONE median junction onto different
// centerline stations. Repair that explicit connection, never nearest neighbours.
// Run before curve fitting/grading/roadbed generation so all downstream geometry
// is solved against the same canonical topology.
public static class AvenueJunctionRepair
{
    public static IReadOnlyList<PreparedSegment> Prepare(
        IReadOnlyList<PreparedSegment> source,
        ImportLog log,
        CancellationToken token,
        bool completeStreetJunctions = false)
    {
        token.ThrowIfCancellationRequested();
        var parent = new Dictionary<long, long>();
        var points = new Dictionary<long, float3>();
        // The IDs in points by their root, so a cluster's members need no scan of every point.
        var clusters = new Dictionary<long, List<long>>();

        long Root(long id)
        {
            while (parent.TryGetValue(id, out var next) && next != id)
                id = next;

            return id;
        }

        bool Ground(PreparedSegment segment)
            => segment.Kind == NetworkKind.Road
                && !segment.Bridge
                && !segment.Tunnel
                && !segment.Roundabout
                && !segment.StartMiniRoundabout
                && !segment.EndMiniRoundabout;

        bool Avenue(PreparedSegment segment)
            => segment.ConsolidatedAvenue
                || completeStreetJunctions && !segment.OneWay && segment.SourceWayIds.Count > 1;

        float Width(PreparedSegment segment)
            => math.isfinite(segment.NativeWidth) && segment.NativeWidth > 0
                ? segment.NativeWidth
                : segment.Lanes > 4 ? 32 : 24;

        bool Continuation(PreparedSegment link, PreparedSegment segment, long node)
        {
            if (Avenue(segment))
                return true;

            if (!completeStreetJunctions || segment.OneWay || segment == link)
                return false;

            var away = (segment.StartNodeId == node ? segment.End - segment.Start : segment.Start - segment.End).xz;
            var toward = (link.StartNodeId == node ? link.End - link.Start : link.Start - link.End).xz;

            // The direction test is cheap and rejects nearly every arm before the names are compared.
            if (!(math.dot(math.normalizesafe(away), math.normalizesafe(toward)) < -.9f))
                return false;

            return link.Name.Trim().Length > 0
                    && link.Name.Trim().Equals(segment.Name.Trim(), StringComparison.OrdinalIgnoreCase)
                || link.Reference.Trim().Length > 0
                    && link.Reference.Trim().Equals(segment.Reference.Trim(), StringComparison.OrdinalIgnoreCase);
        }

        bool Transverse(PreparedSegment link, float distance)
        {
            var original = link.OriginalGeometry;

            return original.Count >= 2 && distance > .01f
                && Math.Abs(
                    math.dot(
                        math.normalizesafe(original[original.Count - 1].xz - original[0].xz),
                        math.normalizesafe(link.End.xz - link.Start.xz)))
                    < .5f;
        }

        bool Duplicate(PreparedSegment link, PreparedSegment segment)
            => segment.ConsolidatedAvenue
                && ((segment.StartNodeId == link.StartNodeId && segment.EndNodeId == link.EndNodeId)
                    || (segment.EndNodeId == link.StartNodeId && segment.StartNodeId == link.EndNodeId));

        bool SourceEvidence(PreparedSegment link, PreparedSegment segment)
            => Avenue(segment) && segment.SourceWayIds.Contains(link.WayId);

        // The loop below collapses a link only as a transverse duplicate interval, or with
        // source evidence when at most 2 m long or a one-way approach within half its bound;
        // the bound is never above 32 m. A link that fails this on its own fields is skipped
        // whatever its arms, and that is nearly every road of the map on every pass.
        bool ShortLink(PreparedSegment link)
        {
            if (link.ConsolidatedAvenue || !Ground(link) || link.StartNodeId == link.EndNodeId)
                return false;

            var length = math.distance(link.Start.xz, link.End.xz);

            return math.isfinite(length)
                && length <= 32
                && (Transverse(link, length)
                    || length <= 2
                    || completeStreetJunctions && link.OneWay && length <= 16);
        }

        var shortLinks = new List<PreparedSegment>();
        var linkNodes = new HashSet<long>();

        foreach (var segment in source)
            if (ShortLink(segment))
            {
                shortLinks.Add(segment);
                linkNodes.Add(segment.StartNodeId);
                linkNodes.Add(segment.EndNodeId);
            }

        // Every node looked up below is an end of one of those links: the link's own, or
        // a cluster member taken from an earlier link. Arms stay in source order.
        var incident = new Dictionary<long, List<PreparedSegment>>();

        void AddIncident(long id, PreparedSegment segment)
        {
            if (!incident.TryGetValue(id, out var list))
                incident[id] = list = new();

            list.Add(segment);
        }

        foreach (var segment in source)
        {
            if (linkNodes.Contains(segment.StartNodeId))
                AddIncident(segment.StartNodeId, segment);

            if (linkNodes.Contains(segment.EndNodeId))
                AddIncident(segment.EndNodeId, segment);
        }

        var collapsed = new HashSet<PreparedSegment>();

        // The rest of the loop's tests that read only the link and its arms, which the loop
        // does not change: the evidence above, and a continuation at both ends. Filtering
        // before the (stable) sort leaves the same links in the same order (report 114).
        bool Candidate(PreparedSegment link)
        {
            var startArms = incident[link.StartNodeId];
            var endArms = incident[link.EndNodeId];
            var distance = math.distance(link.Start.xz, link.End.xz);
            var evidenceCounts = distance <= 2 || completeStreetJunctions && link.OneWay && distance <= 16;

            var collapsible =
                Transverse(link, distance)
                    && (startArms.Any(segment => Duplicate(link, segment))
                        || endArms.Any(segment => Duplicate(link, segment)))
                || evidenceCounts
                    && (startArms.Any(segment => SourceEvidence(link, segment))
                        || endArms.Any(segment => SourceEvidence(link, segment)));

            return collapsible
                && startArms.Any(segment => Continuation(link, segment, link.StartNodeId))
                && endArms.Any(segment => Continuation(link, segment, link.EndNodeId));
        }

        foreach (var link in shortLinks
            .Where(Candidate)
            .OrderBy(segment => segment.WayId)
            .ThenBy(segment => segment.StartNodeId)
            .ThenBy(segment => segment.EndNodeId))
        {
            token.ThrowIfCancellationRequested();
            var distance = math.distance(link.Start.xz, link.End.xz);

            var arms = incident[link.StartNodeId].Concat(incident[link.EndNodeId]).Distinct().ToArray();

            // A structure, rail crossing, roundabout or another layer is not a
            // recoverable median junction merely because it is close by.
            if (arms.Any(
                segment => !Ground(segment)
                    || segment.Layer != link.Layer
                    || completeStreetJunctions && NativeAssetKeys.IsHighway(segment)))
                continue;

            var firstSegments =
                incident[link.StartNodeId].Where(segment => Continuation(link, segment, link.StartNodeId)).ToArray();

            var secondSegments =
                incident[link.EndNodeId].Where(segment => Continuation(link, segment, link.EndNodeId)).ToArray();

            if (firstSegments.Length == 0 || secondSegments.Length == 0)
                continue;

            var bound = Math.Min(32, Math.Min(firstSegments.Min(Width), secondSegments.Min(Width)));

            if (!math.isfinite(distance) || distance > bound)
                continue;

            var duplicate = arms.Any(segment => Duplicate(link, segment));
            var transverse = Transverse(link, distance);
            var sourceEvidence = arms.Any(segment => SourceEvidence(link, segment));
            // Palm Springs: a transverse median link became a duplicate
            // longitudinal avenue interval. Oman: an explicit ~1 m tie-in
            // between remapped junctions retained its own carriageway evidence.
            var shortApproach = completeStreetJunctions && link.OneWay && sourceEvidence && distance <= bound / 2;

            if (!(duplicate && transverse) && !(distance <= 2 && sourceEvidence) && !shortApproach)
                continue;

            var first = Root(link.StartNodeId);
            var last = Root(link.EndNodeId);

            if (first == last)
            {
                collapsed.Add(link);
                continue;
            }

            var memberSet = new HashSet<long> { link.StartNodeId, link.EndNodeId };

            if (clusters.TryGetValue(first, out var firstCluster))
                memberSet.UnionWith(firstCluster);

            if (clusters.TryGetValue(last, out var lastCluster))
                memberSet.UnionWith(lastCluster);

            var members = memberSet.OrderBy(id => id).ToArray();

            float3 Point(long id)
                => points.TryGetValue(id, out var point)
                    ? point
                    : incident[id][0].StartNodeId == id ? incident[id][0].Start : incident[id][0].End;

            var origin = Point(members[0]);
            var center = origin;
            var sum = float3.zero;

            foreach (var id in members)
                sum += Point(id) - origin;

            center += sum / members.Length;

            if (members.Any(id => math.distance(Point(id).xz, center.xz) > bound * .5f))
                continue;

            var set = new HashSet<long>(members);
            // Do not accidentally consume a real third street inside a cluster.
            var internalRoads =
                members
                    .SelectMany(id => incident[id])
                    .Where(
                        segment => set.Contains(segment.StartNodeId)
                            && set.Contains(segment.EndNodeId)
                            && !collapsed.Contains(segment)
                            && segment != link)
                    .Distinct()
                    .ToArray();

            if (internalRoads.Any(segment => !Avenue(segment)) && !shortApproach)
                continue;

            if (shortApproach)
            {
                var acute = false;

                foreach (var id in members)
                {
                    var outside =
                        incident[id]
                            .Where(
                                segment => segment != link
                                    && !(set.Contains(segment.StartNodeId) && set.Contains(segment.EndNodeId)))
                            .ToArray();

                    float2 Away(PreparedSegment segment, float3 node)
                        => (segment.StartNodeId == id ? segment.End - node : segment.Start - node).xz;

                    for (var x = 0; x < outside.Length; x++)
                        for (var y = x + 1; y < outside.Length; y++)
                            if (AvenueConsolidator.Angle(Away(outside[x], Point(id)), Away(outside[y], Point(id))) >= 25
                                && AvenueConsolidator.Angle(Away(outside[x], center), Away(outside[y], center)) < 25)
                                acute = true;
                }

                if (acute)
                    continue;
            }

            var canonical = Math.Min(first, last);
            parent[first] = canonical;
            parent[last] = canonical;

            foreach (var id in members)
                points[id] = Point(id);

            // Every point whose root was first or last is a member, and now has the canonical root.
            clusters.Remove(first);
            clusters.Remove(last);
            clusters[canonical] = new List<long>(members);
            collapsed.Add(link);

            if (shortApproach)
                foreach (var edge in internalRoads)
                    collapsed.Add(edge);

            log.Diagnostic(
                $"AVENUE JUNCTION RECOVERY connectorWay={link.WayId}; sourceNodes={string.Join(",", members)}; canonical={canonical}; lengthMeters={distance:R}; duplicateInterval={duplicate}; transverseOriginal={transverse}; sourceEvidence={sourceEvidence}; center=({center.x:R},{center.y:R},{center.z:R}); explicitConnectionOnly=true; requiresDownstreamReaudit=true");
        }

        if (collapsed.Count == 0)
            return source;

        var centers = points.Keys.GroupBy(Root).ToDictionary(group => group.Key, group =>
        {
            var point = points[group.First()];
            var sum = float3.zero;

            foreach (var id in group)
                sum += points[id] - point;

            return point + sum / group.Count();
        });

        var result = new List<PreparedSegment>(source.Count);
        var internalIntervals = 0;
        var reconnected = 0;

        foreach (var segment in source)
        {
            token.ThrowIfCancellationRequested();

            if (collapsed.Contains(segment))
                continue;

            var root = Root(segment.StartNodeId);
            var otherRoot = Root(segment.EndNodeId);

            if (root == otherRoot && segment.ConsolidatedAvenue)
            {
                internalIntervals++;
                continue;
            }

            if (!points.ContainsKey(segment.StartNodeId) && !points.ContainsKey(segment.EndNodeId))
            {
                result.Add(segment);
                continue;
            }

            var start = points.ContainsKey(segment.StartNodeId) ? centers[root] : segment.Start;
            var end = points.ContainsKey(segment.EndNodeId) ? centers[otherRoot] : segment.End;
            result.Add(segment with
            {
                StartNodeId = root,
                EndNodeId = otherRoot,
                Start = start,
                End = end,
                SourceGeometry = new[] { start, end },
                Control1 = null,
                Control2 = null,
                HorizontalParent = null,
                ParentStartT = 0,
                ParentEndT = 1,
                StartCurveOffset = float3.zero,
                EndCurveOffset = float3.zero
            });

            reconnected++;
        }

        log.Count("Avenue median junction connectors canonicalized", collapsed.Count);
        log.Count("Avenue internal duplicate intervals collapsed", internalIntervals);
        log.Diagnostic(
            $"AVENUE JUNCTION RECOVERY complete; connectors={collapsed.Count}; internalIntervals={internalIntervals}; armsReconnected={reconnected}; outputSections={result.Count}; nearbyUnconnectedRoadsUntouched=true");

        var transferred = SourceTopologyObligations.Transfer(
            source,
            result,
            NetworkKind.Road,
            parent.Keys.ToDictionary(id => id, Root));
        return transferred;
    }
}
