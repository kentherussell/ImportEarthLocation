using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Pure numeric work only. Callers must first Clear the owned, unapplied preview
// and observe exact retirement. This method cannot mutate an applied ECS owner.
// Each touched section moves to its observed port and is refitted alone; the end
// the port does not name stays where it is, so no neighbour changes.
public static class NativeJunctionReconciliation
{
    public readonly record struct Port(float3 Position, float2 TravelTangent, double Grade, bool AppliedBoundary);

    public sealed record Result(
        IReadOnlyList<PreparedSegment> Sections,
        bool NativeSafe,
        bool NeedsTerrainRestamp,
        int ChangedSections,
        DesignQuality Quality)
    {
        public NetworkRoadbed.Patch? TerrainPatch { get; init; }
    }

    public static Result Refit(
        IReadOnlyList<PreparedSegment> immutableUnit,
        IReadOnlyDictionary<SourceNodeKey, Port> observed,
        float scale,
        double width,
        ImportLog log,
        CancellationToken token)
    {
        var result = immutableUnit.ToArray();
        var changed = 0;
        var restamp = false;

        foreach (var pair in observed)
            if (!math.all(math.isfinite(pair.Value.Position))
                || !math.all(math.isfinite(pair.Value.TravelTangent))
                || !math.isfinite(pair.Value.Grade))
                return new(immutableUnit, false, false, 0, DesignQuality.RequiresReview);

        for (var i = 0; i < result.Length; i++)
        {
            token.ThrowIfCancellationRequested();
            var segment = immutableUnit[i];
            var vectorA = segment.CourseStart;
            var vectorB = segment.CourseEnd;
            var ta = math.normalizesafe((segment.CourseControl1 - vectorA).xz);
            var tb = math.normalizesafe((vectorB - segment.CourseControl2).xz);
            var aPose = observed.TryGetValue(new(segment.Kind, segment.StartNodeId), out var ap);
            var bPose = observed.TryGetValue(new(segment.Kind, segment.EndNodeId), out var bp);
            var slopeA =
                math.length((segment.CourseControl1 - vectorA).xz) > 1e-8
                    ? (segment.CourseControl1.y - vectorA.y) / math.length((segment.CourseControl1 - vectorA).xz)
                    : 0;

            var slopeB =
                math.length((vectorB - segment.CourseControl2).xz) > 1e-8
                    ? (vectorB.y - segment.CourseControl2.y) / math.length((vectorB - segment.CourseControl2).xz)
                    : 0;

            if (aPose)
            {
                vectorA = ap.Position + segment.StartCurveOffset;
                ta = ap.TravelTangent;
            }

            if (bPose)
            {
                vectorB = bp.Position + segment.EndCurveOffset;
                tb = bp.TravelTangent;
            }

            var da = vectorA - segment.CourseStart;
            var db = vectorB - segment.CourseEnd;
            var tangentChanged =
                math.dot(ta, math.normalizesafe((segment.CourseControl1 - segment.CourseStart).xz)) < 1 - 1e-8
                    || math.dot(tb, math.normalizesafe((segment.CourseEnd - segment.CourseControl2).xz)) < 1 - 1e-8;

            var gradeChanged =
                aPose && Math.Abs(ap.Grade - slopeA) > 1e-6 || bPose && Math.Abs(bp.Grade - slopeB) > 1e-6;

            var sourceCurve =
                new Cubic3d(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd);

            Cubic3d curve;

            if (Math.Max(math.length(da), math.length(db)) <= .1 && !tangentChanged)
                curve =
                    new(vectorA, (double3)segment.CourseControl1 + da, (double3)segment.CourseControl2 + db, vectorB);
            else
            {
                var raw =
                    segment.SourceGeometry.Count >= 2
                        ? segment.SourceGeometry.ToArray()
                        : Enumerable
                            .Range(0, 33)
                            .Select(j => (float3)NetworkDesignChecks.Evaluate(sourceCurve, j / 32d))
                            .ToArray();

                var observations = new double3[raw.Length];
                var stations = new double[raw.Length];
                var support = new double[raw.Length];

                for (var j = 0; j < raw.Length; j++)
                {
                    observations[j] = raw[j];

                    if (j > 0)
                        stations[j] = stations[j - 1] + math.distance(raw[j - 1].xz, raw[j].xz);
                }

                var total = stations[stations.Length - 1];

                for (var j = 0; j < raw.Length; j++)
                    support[j] =
                        ((j > 0 ? stations[j] - stations[j - 1] : 0)
                            + (j + 1 < raw.Length ? stations[j + 1] - stations[j] : 0))
                            * .5;

                for (var j = 0; j < raw.Length; j++)
                    stations[j] = total > 0 ? stations[j] / total : j / (double)(raw.Length - 1);

                try
                {
                    curve =
                        NetworkDesignChecks.FitHandles(vectorA, vectorB, ta, tb, observations, stations, support, 1e-5);
                }
                catch (ArgumentException)
                {
                    log.Diagnostic(
                        $"LOCAL REFIT unavailable way={segment.WayId}; unrepresentable fixed endpoint/tangent frame; unchanged unit retained");

                    return new(immutableUnit, true, false, 0, DesignQuality.RequiresReview);
                }
            }

            // Each end keeps its grade unless a port fixes a new one.
            var ca = (float3)curve.P1;
            var callback = (float3)curve.P2;
            ca.y = vectorA.y + (float)((aPose ? ap.Grade : slopeA) * math.length((ca - vectorA).xz));
            callback.y = vectorB.y - (float)((bPose ? bp.Grade : slopeB) * math.length((vectorB - callback).xz));
            var controls = new[] { vectorA, ca, callback, vectorB };

            if (controls.Any(
                point => !math.all(math.isfinite(point))
                    || math.any(math.abs(point.xz) > (float)(width / 2 + .02))
                    || point.y < 0
                    || point.y > scale))
                return new(immutableUnit, false, false, 0, DesignQuality.RequiresReview);

            result[i] = segment with
            {
                Start = vectorA - segment.StartCurveOffset,
                End = vectorB - segment.EndCurveOffset,
                Control1 = ca - segment.StartCurveOffset,
                Control2 = callback - segment.EndCurveOffset,
                FinalDeckCurve = null,
                HorizontalParent = null,
                HorizontalEmissionReference = new(vectorA, ca, callback, vectorB),
                PairedSpanId = 0,
                StartHeightFamily = 0,
                EndHeightFamily = 0,
                ReferenceSpan = 0
            };

            if (gradeChanged
                || tangentChanged
                || math.distance(vectorA, segment.CourseStart) > .001
                || math.distance(vectorB, segment.CourseEnd) > .001
                || math.distance(ca, segment.CourseControl1) > .001
                || math.distance(callback, segment.CourseControl2) > .001)
            {
                changed++;

                if (!NetworkStructure.IsStructure(segment))
                    restamp = true;
            }
        }

        log.Diagnostic(
            $"LOCAL REFIT designed sections={result.Length}; changed={changed}; needsOwnedTerrainRestamp={restamp}; nativeApplyInvocations=0; quality=RequiresReview");

        return changed == 0
            ? new(immutableUnit, true, false, 0, DesignQuality.RequiresReview)
            : new(result, true, restamp, changed, DesignQuality.RequiresReview);
    }
}
