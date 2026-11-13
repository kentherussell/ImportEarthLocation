using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// One set of numeric junction checks, used for final design and qualified
// native readback. Observations never grant ownership or rewrite a junction.
public static class NetworkJunctionQuality
{
    public sealed record Pose(double3 Point, double2 Travel, IReadOnlyList<float> LaneOffsets);

    public readonly record struct Observation(
        long Node,
        long ApproachWay,
        string Category,
        double? Value,
        double Limit);

    public static void Audit(IReadOnlyList<PreparedSegment> sections, Func<PreparedSegment, bool, Pose?> observe,
        ImportLog log, CancellationToken token, string stage, string scope, Action<Observation>? observation = null)
    {
        var nodes = sections.Where(segment => segment.Kind == NetworkKind.Road && segment.OneWay)
            .SelectMany(segment => new[] { (Section: segment, Start: true), (Section: segment, Start: false) })
            .GroupBy(item => item.Start ? item.Section.StartNodeId : item.Section.EndNodeId);

        var expected = 0;
        var checkedJoins = 0;
        var missing = 0;
        var datumFailed = 0;
        var headingFailed = 0;
        var rampExpected = 0;
        var rampChecked = 0;
        var rampMissing = 0;
        var rampFailed = 0;
        var worstDatum = 0d;
        var worstHeading = 0d;
        var worstAngleError = 0d;
        var records = log.DiagnosticsEnabled ? new List<object>() : null;
        var positionTolerance = stage == "planned" ? .05 : .25;

        foreach (var node in nodes)
        {
            token.ThrowIfCancellationRequested();
            var entries = node.ToArray();
            var main =
                entries
                    .Where(
                        item => !item.Section.ProtectedLink && (item.Section.Classification is "motorway" or "trunk"))
                    .ToArray();

            if (main.Length != 2 || main.Count(item => item.Start) != 1)
                continue;

            expected++;
            var incoming = main.First(item => !item.Start);
            var outgoing = main.First(item => item.Start);
            var firstPose = observe(incoming.Section, false);
            var secondPose = observe(outgoing.Section, true);
            var available =
                firstPose != null
                    && secondPose != null
                    && firstPose.LaneOffsets.Count > 0
                    && secondPose.LaneOffsets.Count > 0
                    && math.lengthsq(firstPose.Travel) > .5
                    && math.lengthsq(secondPose.Travel) > .5;

            var heading = available ? math.normalizesafe(firstPose!.Travel + secondPose!.Travel) : default;
            available = available && math.lengthsq(heading) > .5;

            if (!available)
            {
                missing++;
                observation?.Invoke(new(node.Key, 0, "continuing-lane-datum", null, positionTolerance));
                observation?.Invoke(new(node.Key, 0, "mainline-junction-heading", null, 1));
            }
            else
            {
                checkedJoins++;
                var normal = new double2(heading.y, -heading.x);
                var leftA =
                    firstPose!.Point.xz
                        + new double2(firstPose.Travel.y, -firstPose.Travel.x) * firstPose.LaneOffsets.Min();

                var leftB =
                    secondPose!.Point.xz
                        + new double2(secondPose.Travel.y, -secondPose.Travel.x) * secondPose.LaneOffsets.Min();

                var mismatch =
                    Math.Max(
                        Math.Abs(math.dot(leftA - leftB, normal)),
                        Math.Abs(firstPose.Point.y - secondPose.Point.y));

                var angle = Math.Acos(math.clamp(math.dot(firstPose.Travel, secondPose.Travel), -1, 1)) * 180 / Math.PI;
                worstDatum = Math.Max(worstDatum, mismatch);
                worstHeading = Math.Max(worstHeading, angle);
                observation?.Invoke(new(node.Key, 0, "continuing-lane-datum", mismatch, positionTolerance));
                observation?.Invoke(new(node.Key, 0, "mainline-junction-heading", angle, 1));

                if (mismatch > positionTolerance)
                    datumFailed++;

                if (angle > 1)
                    headingFailed++;

                records?.Add(new
                {
                    node = node.Key,
                    incoming = incoming.Section.WayId,
                    outgoing = outgoing.Section.WayId,
                    continuingLaneMismatchMeters = mismatch,
                    throughHeadingDegrees = angle,
                    nativeSlots = true
                });
            }

            foreach (var ramp in entries.Where(item => item.Section.ProtectedLink))
            {
                // Without actual slot counts, it is unknown whether a new lane
                // exists. Do not infer the merge contract from nominal lanes.
                if (!available)
                {
                    rampExpected++;
                    rampMissing++;
                    observation?.Invoke(new(node.Key, ramp.Section.WayId, "no-added-lane-approach", null, 1));
                    continue;
                }

                var far = ramp.Start ? ramp.Section.End : ramp.Section.Start;
                var anchor = ramp.Start ? ramp.Section.Start : ramp.Section.End;
                var rightSide = math.dot((double2)(far - anchor).xz, new double2(heading.y, -heading.x)) > 0;
                var added =
                    rightSide
                        && (ramp.Start
                            ? firstPose!.LaneOffsets.Count > secondPose!.LaneOffsets.Count
                            : secondPose!.LaneOffsets.Count > firstPose!.LaneOffsets.Count);

                if (added)
                    continue;

                rampExpected++;
                var pose = observe(ramp.Section, ramp.Start);

                if (pose == null || math.lengthsq(pose.Travel) < .5)
                {
                    rampMissing++;
                    observation?.Invoke(new(node.Key, ramp.Section.WayId, "no-added-lane-approach", null, 1));
                    continue;
                }

                rampChecked++;
                var angle = Math.Acos(math.clamp(math.dot(heading, pose.Travel), -1, 1)) * 180 / Math.PI;
                var error = Math.Abs(angle - 20);
                worstAngleError = Math.Max(worstAngleError, error);

                if (error > 1)
                    rampFailed++;

                observation?.Invoke(new(node.Key, ramp.Section.WayId, "no-added-lane-approach", error, 1));
                records?.Add(
                    new
                    {
                        node = node.Key,
                        ramp = ramp.Section.WayId,
                        noAddedLane = true,
                        approachDegrees = angle,
                        errorDegrees = error
                    });
            }
        }

        void Put(string category, int count, int failures, int unavailable, int total, double worst, string meaning)
            => log.Quality(
                new(stage, category, scope, count, failures, unavailable, worst, 0, meaning) { Expected = total });

        Put(
            "continuing-lane-datum",
            checkedJoins,
            datumFailed,
            missing,
            expected,
            worstDatum,
            "Travel-left continuing native slot; lateral and height mismatch; longitudinal native junction trimming excluded");

        Put(
            "mainline-junction-heading",
            checkedJoins,
            headingFailed,
            missing,
            expected,
            worstHeading,
            "Through mainline heading difference <=1 degree");

        Put(
            "no-added-lane-approach",
            rampChecked,
            rampFailed,
            rampMissing,
            rampExpected,
            worstAngleError,
            "Actual slot counts select no-added-lane 20 +/-1 degree contract; missing lineage/palette unavailable");

        if (records != null)
            log.DiagnosticCompressedRecords(scope + "-junction-quality.ndjson.gz", records);
    }
}
