using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Only remap-created return loops inside an explicit consolidated junction.
// No proximity joining, real branch contraction, rail or roundabout deletion.
public static class NativeJunctionSimplifier
{
    public static IReadOnlyList<PreparedSegment> Prepare(
        IReadOnlyList<PreparedSegment> source,
        ImportLog log,
        CancellationToken token)
    {
        var incident = new Dictionary<long, List<PreparedSegment>>();

        foreach (var segment in source.Where(otherSegment => otherSegment.Kind == NetworkKind.Road))
            foreach (var id in new[] { segment.StartNodeId, segment.EndNodeId })
            {
                if (!incident.TryGetValue(id, out var arms))
                    incident[id] = arms = new List<PreparedSegment>();

                arms.Add(segment);
            }

        var removed = new HashSet<PreparedSegment>();

        foreach (var firstSegment in source)
        {
            token.ThrowIfCancellationRequested();

            if (removed.Contains(firstSegment)
                || !Eligible(firstSegment)
                || !firstSegment.OneWay
                || firstSegment.StartNodeId == firstSegment.EndNodeId)
                continue;

            if (!incident.TryGetValue(firstSegment.EndNodeId, out var atEnd) || atEnd.Count != 2)
                continue;

            var secondSegment = atEnd.FirstOrDefault(segment => segment != firstSegment);

            if (secondSegment == null
                || !Eligible(secondSegment)
                || secondSegment.StartNodeId != firstSegment.EndNodeId
                || secondSegment.EndNodeId != firstSegment.StartNodeId
                || secondSegment.WayId != firstSegment.WayId)
                continue;

            if (firstSegment.Classification != secondSegment.Classification
                || firstSegment.Layer != secondSegment.Layer
                || firstSegment.Lanes != secondSegment.Lanes
                || firstSegment.OneWay != secondSegment.OneWay
                || firstSegment.NativeWidth != secondSegment.NativeWidth
                    || firstSegment.SourceSpeedKph != secondSegment.SourceSpeedKph
                    || firstSegment.ExplicitIncline != secondSegment.ExplicitIncline
                    || !firstSegment.SourceTags
                    .OrderBy(entry => entry.Key)
                    .SequenceEqual(secondSegment.SourceTags.OrderBy(entry => entry.Key))
                    || !firstSegment.RawSourceTags
                    .OrderBy(entry => entry.Key)
                    .SequenceEqual(secondSegment.RawSourceTags.OrderBy(entry => entry.Key)))
                continue;

            if (!incident.TryGetValue(firstSegment.StartNodeId, out var anchorArms)
                || !anchorArms.Any(segment => segment.ConsolidatedAvenue))
                continue;

            if (anchorArms.Any(segment => !Eligible(segment) || segment.Layer != firstSegment.Layer))
                continue;

            // Original endpoint no longer equals the remapped avenue center.
            if (firstSegment.OriginalGeometry.Count < 2 || secondSegment.OriginalGeometry.Count < 2
                || math.distance(firstSegment.OriginalGeometry[0].xz, firstSegment.Start.xz) < .5f
                    || math.distance(
                    secondSegment.OriginalGeometry[secondSegment.OriginalGeometry.Count - 1].xz,
                    secondSegment.End.xz)
                    < .5f
                    || math.distance(
                    firstSegment.OriginalGeometry[0].xz,
                    secondSegment.OriginalGeometry[secondSegment.OriginalGeometry.Count - 1].xz)
                    <= .5f
                    || math.distance(
                    firstSegment.OriginalGeometry[firstSegment.OriginalGeometry.Count - 1].xz,
                    secondSegment.OriginalGeometry[0].xz)
                    > .05f)
                continue;

            var bound =
                Math.Min(12, Math.Max(4, (firstSegment.NativeWidth > 0 ? firstSegment.NativeWidth * .5f : 8) + 2));

            if (NetworkVerticalProfiles.ArcLength(firstSegment) + NetworkVerticalProfiles.ArcLength(secondSegment) > 24)
                continue;

            var inside = true;

            foreach (var segment in new[] { firstSegment, secondSegment })
                for (var i = 0; i <= 16; i++)
                    if (math.distance(NetworkNormalizer.Evaluate(segment, i / 16f).xz, firstSegment.Start.xz) > bound)
                        inside = false;

            if (!inside)
                continue;

            removed.Add(firstSegment);
            removed.Add(secondSegment);
            log.Diagnostic(
                $"NATIVE MICROCLUSTER normalized=true; way={firstSegment.WayId}; retainedAnchor={firstSegment.StartNodeId}; removedInterior={firstSegment.EndNodeId}; spans=2; interiorDegree=2; radiusBoundMeters={bound}; originalRemapEvidence=true; realBranchesRemoved=0; regradePending=true");
        }

        log.Count("Remap-created return loop sections normalized", removed.Count);

        if (removed.Count > 0)
            log.DiagnosticRecords(
                "native-normalized-return-loops.ndjson",
                DiagnosticArtifacts.NetworkRecords(source.Where(segment => removed.Contains(segment)).ToArray()));

        return removed.Count == 0 ? source : source.Where(segment => !removed.Contains(segment)).ToArray();
    }

    private static bool Eligible(PreparedSegment segment)
        => segment.Kind == NetworkKind.Road && !segment.Bridge && !segment.Tunnel && !segment.Roundabout
            && !segment.StartTerminal
                && !segment.EndTerminal
                && !segment.StartMiniRoundabout
                && !segment.EndMiniRoundabout
                && segment.StartRoundaboutRadius == 0
            && segment.EndRoundaboutRadius == 0
            && segment.ExplicitCutSides == 0
            && segment.ExplicitWallSides == 0
            && segment.RetainingWallSides == 0;
}
