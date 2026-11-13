using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

public static class NetworkQualityAudit
{
    public static double? DirectionRoundingBudget(PreparedSegment section, bool start)
    {
        var reference = section.HorizontalEmissionReference;

        if (!reference.HasValue)
            return null;

        var intended = reference.Value;
        var point = start ? intended.P0 : intended.P3;
        var h = start ? intended.P1 : intended.P2;
        var actual = start ? section.CourseStart : section.CourseEnd;
        var actualHandle = start ? section.CourseControl1 : section.CourseControl2;
        var error = math.distance(point.xz, ((double3)actual).xz) + math.distance(h.xz, ((double3)actualHandle).xz);
        var reach = math.distance(point.xz, h.xz);

        if (!math.isfinite(error) || !math.isfinite(reach) || reach <= 0)
            return null;

        return Math.Asin(Math.Min(1, error / Math.Max(reach - error, 1e-5)));
    }

    public static void Planned(
        IReadOnlyList<PreparedSegment> sections,
        ImportLog log,
        CancellationToken token,
        bool sourceTopologyAvailable = true)
    {
        var gradeFailures = 0;
        var curvatureFailures = 0;
        var curvatureUnknown = 0;
        var finiteFailures = 0;
        var regularityUnknown = 0;
        var gradeLength = 0d;
        var curvatureLength = 0d;
        var maximumGrade = 0d;
        var maximumCurvature = 0d;
        var incident = new Dictionary<SourceNodeKey, List<(PreparedSegment Section, bool Start)>>();
        var observations = new NetworkCurveObservations.Observation[sections.Count];
        var ordinal = 0;

        foreach (var segment in sections)
        {
            token.ThrowIfCancellationRequested();
            var curve =
                new Cubic3d(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd);

            var observation = NetworkCurveObservations.Measure(curve);
            var finite = observation.Finite;
            observations[ordinal++] = observation;

            if (!finite)
                finiteFailures++;

            maximumGrade = Math.Max(maximumGrade, observation.SampledMaximumGrade);

            if (segment.Kind == NetworkKind.Rail)
                maximumCurvature = Math.Max(maximumCurvature, observation.SampledMaximumCurvature);

            if (!NetworkVerticalProfiles.WithinGradeCeiling(segment, NetworkGeometrySafety.GradeLimit(segment)))
            {
                gradeFailures++;
                gradeLength += observation.SampledLengthMeters;
            }

            var limit = segment.NativeCenterlineCurvatureLimit;

            if (segment.Kind == NetworkKind.Rail && !limit.HasValue)
                curvatureUnknown++;

            if (finite)
            {
                var certificate =
                    NetworkDesignChecks.Certify(
                        curve,
                        limit ?? double.PositiveInfinity,
                        double.PositiveInfinity,
                        token: token,
                        maximumLeaves: 4096);

                if (certificate.Quality != DesignQuality.Certified)
                {
                    regularityUnknown++;

                    if (segment.Kind == NetworkKind.Rail && limit.HasValue)
                    {
                        curvatureFailures++;
                        curvatureLength += observation.SampledLengthMeters;
                    }
                }
            }
            else
                regularityUnknown++;

            foreach (var start in new[] { true, false })
            {
                var key = new SourceNodeKey(segment.Kind, start ? segment.StartNodeId : segment.EndNodeId);

                if (!incident.TryGetValue(key, out var list))
                    incident[key] = list = new();

                list.Add((segment, start));
            }
        }

        var anchorFailures = 0;
        var headingFailures = 0;
        var headingUnknown = 0;
        var gradeJoinFailures = 0;
        var gradeJoinUnknown = 0;
        var joins = 0;
        var headingWorst = 0d;
        var gradeWorst = 0d;
        var roundingFailures = 0;
        var roundingUnknown = 0;
        var roundingWorst = 0d;
        var joinWitnesses = new List<object>();

        static double Angle(double2 vectorA, double2 vectorB)
            => Math.Atan2(Math.Abs(NetworkDesignChecks.Cross(vectorA, vectorB)), math.dot(vectorA, vectorB));

        foreach (var node in incident.Values)
        {
            token.ThrowIfCancellationRequested();
            var point = node[0].Start ? node[0].Section.Start : node[0].Section.End;

            if (node.Any(item => !math.all((item.Start ? item.Section.Start : item.Section.End) == point)))
                anchorFailures++;

            if (node.Count != 2 || node.Any(item => item.Section.Roundabout))
                continue;

            var firstItem = node[0];
            var secondItem = node[1];

            double3 Away((PreparedSegment Section, bool Start) item)
                => item.Start
                    ? (double3)(item.Section.CourseControl1 - item.Section.CourseStart)
                    : (double3)(item.Section.CourseControl2 - item.Section.CourseEnd);

            var da = Away(firstItem);
            var db = Away(secondItem);
            var la = math.length(da.xz);
            var lb = math.length(db.xz);

            if (!(la > 0 && lb > 0))
            {
                joins++;
                headingUnknown++;
                gradeJoinUnknown++;
                roundingUnknown++;
                continue;
            }

            // Only designed through continuations have this contract. A real
            // corner is not assigned an invented common grade or travel axis.
            var dot = math.dot(da.xz / la, db.xz / lb);

            if (dot > -.9)
                continue;

            joins++;
            var angle = Angle(da.xz, -db.xz);
            var jump = Math.Abs(da.y / la + db.y / lb);
            headingWorst = Math.Max(headingWorst, angle);
            gradeWorst = Math.Max(gradeWorst, jump);
            var betaA = DirectionRoundingBudget(firstItem.Section, firstItem.Start);
            var betaB = DirectionRoundingBudget(secondItem.Section, secondItem.Start);
            double? allowance = null;

            if (betaA.HasValue && betaB.HasValue)
            {
                allowance = Math.Max(1e-5, betaA.Value + betaB.Value);
                roundingWorst = Math.Max(roundingWorst, allowance.Value);

                if (allowance.Value > Math.PI / 180 * .1)
                    roundingFailures++;

                // An excessive budget is a representability failure, not
                // permission to accept any sharp join it could explain.
                if (angle > Math.Min(allowance.Value, Math.PI / 180 * .1))
                    headingFailures++;
            }
            else
            {
                roundingUnknown++;

                if (angle > Math.PI / 180 * .1)
                    headingFailures++;
                else if (angle > 1e-5)
                    headingUnknown++;
            }

            if (jump > 1e-5)
                gradeJoinFailures++;

            if (log.DiagnosticsEnabled
                && joinWitnesses.Count < 128
                && (angle > 1e-5 || jump > 1e-5 || !allowance.HasValue))
                joinWitnesses.Add(new
                {
                    firstItem.Section.Kind,
                    node = firstItem.Start ? firstItem.Section.StartNodeId : firstItem.Section.EndNodeId,
                    wayA = firstItem.Section.WayId,
                    wayB = secondItem.Section.WayId,
                    headingRadians = angle,
                    gradeJump = jump,
                    betaA,
                    betaB,
                    allowance,
                    emittedA = DiagnosticArtifacts.Point(da),
                    emittedB = DiagnosticArtifacts.Point(db),
                    referenceA = DiagnosticArtifacts.Curve(firstItem.Section.HorizontalEmissionReference),
                    referenceB = DiagnosticArtifacts.Curve(secondItem.Section.HorizontalEmissionReference)
                });
        }

        var paired = 0;
        var pairedFailures = 0;
        var pairedUnknown = 0;
        var pairedWorst = 0d;
        var separationFailures = 0;

        foreach (var group in sections
            .Where(segment => segment.PairedProfileId != 0)
            .GroupBy(segment => (segment.PairedProfileId,
            Begin: Math.Round(Math.Min(segment.PairedReferenceBegin, segment.PairedReferenceEnd), 6),
                End: Math.Round(Math.Max(segment.PairedReferenceBegin, segment.PairedReferenceEnd), 6))))
        {
            var pair = group.ToArray();

            if (pair.Length != 2)
            {
                pairedUnknown++;
                continue;
            }

            paired++;
            var firstSegment = pair[0];
            var secondSegment = pair[1];
            var ah =
                new[] {
                    firstSegment.CourseStart.y,
                    firstSegment.CourseControl1.y,
                    firstSegment.CourseControl2.y,
                    firstSegment.CourseEnd.y
                    };

            var ac =
                new Cubic3d(
                    firstSegment.CourseStart,
                    firstSegment.CourseControl1,
                    firstSegment.CourseControl2,
                    firstSegment.CourseEnd);

            var bc =
                new Cubic3d(
                    secondSegment.CourseStart,
                    secondSegment.CourseControl1,
                    secondSegment.CourseControl2,
                    secondSegment.CourseEnd);

            if (Math.Sign(firstSegment.PairedReferenceEnd - firstSegment.PairedReferenceBegin)
                != Math.Sign(secondSegment.PairedReferenceEnd - secondSegment.PairedReferenceBegin))
                bc = new(bc.P3, bc.P2, bc.P1, bc.P0);

            if (!NetworkDesignChecks.Separated(
                ac,
                bc,
                (firstSegment.NativeWidth + secondSegment.NativeWidth) / 2 + 2,
                token))
                separationFailures++;

            var bh =
                new[] {
                    secondSegment.CourseStart.y,
                    secondSegment.CourseControl1.y,
                    secondSegment.CourseControl2.y,
                    secondSegment.CourseEnd.y
                    };

            if (Math.Sign(firstSegment.PairedReferenceEnd - firstSegment.PairedReferenceBegin)
                != Math.Sign(secondSegment.PairedReferenceEnd - secondSegment.PairedReferenceBegin))
                Array.Reverse(bh);

            var difference = Enumerable.Range(0, 4).Max(i => Math.Abs((double)ah[i] - bh[i]));
            pairedWorst = Math.Max(pairedWorst, difference);

            if (difference > .05)
                pairedFailures++;
        }

        void Put(
            string category,
            int count,
            int failed,
            int unknown,
            double worst,
            double length,
            string evidence,
            bool countIncludesUnknown = true)
            => log.Quality(
                new(
                    "planned",
                    category,
                    "final-design",
                    countIncludesUnknown ? Math.Max(failed, count - unknown) : count,
                    failed,
                    unknown,
                    worst,
                    length,
                    evidence)
                { Expected = countIncludesUnknown ? count : count + unknown });

        Put(
            "native-safety-finite",
            sections.Count,
            finiteFailures,
            0,
            finiteFailures,
            0,
            "Exact finite controls; ECS safety remains a separate native observation");

        Put(
            "regularity",
            sections.Count,
            0,
            regularityUnknown,
            0,
            0,
            "Bounded continuous derivative certificate; exhausted/uncertified is unavailable, not passed");

        Put(
            "exceptional-grade",
            sections.Count,
            gradeFailures,
            0,
            maximumGrade,
            gradeLength,
            "Continuous grade ceiling; worst value and affected length are sampled observations");

        Put(
            "rail-curvature",
            sections.Count(segment => segment.Kind == NetworkKind.Rail),
            curvatureFailures,
            curvatureUnknown,
            maximumCurvature,
            curvatureLength,
            "Actual selected rail contract; absent contracts unavailable");

        Put(
            "canonical-anchors",
            incident.Count,
            anchorFailures,
            0,
            anchorFailures,
            0,
            "Bit-identical source node anchors; deck offsets are separate");

        Put(
            "continuation-heading",
            joins,
            headingFailures,
            headingUnknown,
            headingWorst,
            0,
            "Final float through-join radians; max(1e-5,betaA+betaB); absent references cannot justify the native 0.1-degree allowance");

        Put(
            "float-direction-representability",
            joins,
            roundingFailures,
            roundingUnknown,
            roundingWorst,
            0,
            "Actual intended/emitted XZ endpoint and handle errors; budget above 0.1 degree requires review, never a relaxed heading limit");

        Put(
            "continuation-grade",
            joins,
            gradeJoinFailures,
            gradeJoinUnknown,
            gradeWorst,
            0,
            "Outward physical-grade sum; through continuations only; stationary ports unavailable");

        Put(
            "paired-height",
            paired,
            pairedFailures,
            pairedUnknown,
            pairedWorst,
            0,
            "Maximum corresponding height-control difference <=0.05m; exact common reference interval including separation-uncertified confirmed pairs",
            false);

        Put("paired-lateral-separation", paired, separationFailures, pairedUnknown, 0, 0,
            "Independent continuous two-metre median check on actual final float curves; sharing height variables never certifies separation",
                false);

        if (!sourceTopologyAvailable || log.Metric("Selected-source topology validations") == 0)
            Put(
                "source-topology",
                0,
                0,
                1,
                0,
                0,
                "Selected-source obligation snapshot or completed exact validation unavailable",
                false);

        if (log.DiagnosticsEnabled)
            log.DiagnosticRecords("planned-continuation-witnesses.ndjson", joinWitnesses, critical: true);

        NetworkVisualQuality.Audit(sections, observations, log, token);
        NetworkJunctionQuality.Audit(sections, (segment, start) => new(start ? segment.CourseStart : segment.CourseEnd,
            math.normalizesafe(
                (double2)(start
                    ? segment.CourseControl1 - segment.CourseStart
                    : segment.CourseEnd - segment.CourseControl2).xz),
                segment.NativeLaneOffsets),
                log,
                token,
                "planned",
                "final-design");
    }
}
