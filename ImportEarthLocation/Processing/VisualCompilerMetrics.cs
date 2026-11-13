using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Visual screening of a finished network against the per-class tolerances in
// research report 64 (§5.6, §7). Units are metres and grade fractions in local
// map XZ; heights are absolute. This is a comparison metric for pipelines, not
// a certificate or a placement veto. Each section contributes at most one
// failure per category, located at its worst sample, so long and short
// sections weigh equally and clustering decides what one visible issue is.
public static class VisualCompilerMetrics
{
    public const double IssueRadius = 100;
    private const double Spacing = 5;
    private const double TerrainTolerance = .3;
    private const double PairedTolerance = .3;

    public readonly record struct Tolerance(double RadiusFloor, double K, double MinimumSegment, double PairSeparation);

    public readonly record struct Failure(string Category, int Section, long WayId, long StartNodeId, long EndNodeId,
        double3 Position, double Value, double Limit, double Severity);

    public sealed record Issue(
        string Category,
        int Failures,
        double Severity,
        double Value,
        double Limit,
        double3 Position,
        long[] Ways);

    public readonly record struct PairedCoverage(double EligibleMeters, double MatchedMeters, double FailedMeters);

    public sealed record Measurement(
        int Sections,
        double Kilometres,
        PairedCoverage Paired,
        IReadOnlyList<Failure> Failures);

    private readonly record struct Sample(int Section, double3 Position, double2 Direction, double Grade, double Step);

    public static Tolerance For(PreparedSegment segment)
    {
        var otherClass = VisualCompilerTable.For(segment);
        return new(otherClass.RadiusFloor, otherClass.K, otherClass.MinimumSegment, otherClass.PairSeparation);
    }

    public static Measurement Measure(
        IReadOnlyList<PreparedSegment> sections,
        float[] terrain,
        double width,
        CancellationToken token)
    {
        var failures = new List<Failure>();
        var samples = new List<Sample>();
        var first = new int[sections.Count + 1];
        var kilometres = 0d;

        void Fail(string category, int section, double3 position, double value, double limit, double severity)
        {
            var segment = sections[section];
            failures.Add(
                new(
                    category,
                    section,
                    segment.WayId,
                    segment.StartNodeId,
                    segment.EndNodeId,
                    position,
                    value,
                    limit,
                    severity));
        }

        for (var i = 0; i < sections.Count; i++)
        {
            token.ThrowIfCancellationRequested();
            first[i] = samples.Count;
            var segment = sections[i];
            var curve = NetworkDesignChecks.Course(segment);

            if (!NetworkDesignChecks.Finite(curve))
                continue;

            var length = NetworkDesignChecks.HorizontalArcLength(curve);

            if (!(length > 0))
                continue;

            kilometres += length / 1000;
            var tolerance = For(segment);
            var count = Math.Max(2, (int)Math.Ceiling(length / Spacing) + 1);
            var previous = curve.P0;

            for (var k = 0; k < count; k++)
            {
                var t = k / (count - 1d);
                var point = NetworkDesignChecks.Evaluate(curve, t);
                var d = NetworkDesignChecks.Derivative(curve, t);
                var speed = math.length(d.xz);
                samples.Add(
                    new(
                        i,
                        point,
                        speed > 1e-9 ? d.xz / speed : default,
                        speed > 1e-9 ? d.y / speed : 0,
                        math.distance(previous.xz, point.xz)));

                previous = point;
            }

            var own = samples.GetRange(first[i], count);
            var steepest = own.OrderByDescending(sample => Math.Abs(sample.Grade)).First();
            var gradeLimit = NetworkGeometrySafety.GradeLimit(segment);

            if (Math.Abs(steepest.Grade) > gradeLimit)
                Fail(
                    "steep-grade",
                    i,
                    steepest.Position,
                    Math.Abs(steepest.Grade),
                    gradeLimit,
                    Math.Abs(steepest.Grade) / gradeLimit);

            // Grade change reported in percentage points per 10 m; the limit is 10/K.
            var rateLimit = 10 / (tolerance.K * (segment.Tunnel ? VisualCompilerTable.TunnelKShare : 1));
            var worstRate = 0d;
            var rateAt = own[0].Position;

            for (var k = 1; k < own.Count; k++)
                if (own[k].Step >= 1)
                {
                    var rate = Math.Abs(own[k].Grade - own[k - 1].Grade) / own[k].Step * 1000;

                    if (rate > worstRate)
                    {
                        worstRate = rate;
                        rateAt = own[k].Position;
                    }
                }

            if (worstRate > rateLimit)
                Fail("grade-change", i, rateAt, worstRate, rateLimit, worstRate / rateLimit);

            if (!segment.Roundabout)
            {
                var curvature = 0d;
                var curvatureAt = own[0].Position;

                for (var k = 0; k < count; k++)
                {
                    var value = Math.Abs(NetworkDesignChecks.Curvature(curve, k / (count - 1d)));

                    if (value > curvature)
                    {
                        curvature = value;
                        curvatureAt = own[k].Position;
                    }
                }

                if (curvature * tolerance.RadiusFloor > 1)
                    Fail(
                        "tight-radius",
                        i,
                        curvatureAt,
                        1 / curvature,
                        tolerance.RadiusFloor,
                        curvature * tolerance.RadiusFloor);

                if (length < tolerance.MinimumSegment)
                    Fail(
                        "short-segment",
                        i,
                        own[count / 2].Position,
                        length,
                        tolerance.MinimumSegment,
                        tolerance.MinimumSegment / Math.Max(1, length));
            }

            if (!NetworkStructure.IsStructure(segment))
            {
                var worst =
                    own
                        .Select(
                            sample => (
                                sample.Position,
                                Delta: Math.Abs(
                                    sample.Position.y
                                        - NetworkProcessor.Sample(
                                            (float)sample.Position.x,
                                            (float)sample.Position.z,
                                            terrain,
                                            width))))
                    .OrderByDescending(item => item.Delta).First();

                if (worst.Delta > TerrainTolerance)
                    Fail(
                        "terrain-mismatch",
                        i,
                        worst.Position,
                        worst.Delta,
                        TerrainTolerance,
                        worst.Delta / TerrainTolerance);
            }
        }

        first[sections.Count] = samples.Count;
        Nodes(sections, Fail, token);
        var grid = new Dictionary<(int, int), List<int>>();

        for (var k = 0; k < samples.Count; k++)
        {
            var cell = GridCell(samples[k].Position);

            if (!grid.TryGetValue(cell, out var list))
                grid[cell] = list = new();

            list.Add(k);
        }

        IEnumerable<int> Near(double3 point, double radius)
        {
            var r = (int)Math.Ceiling(radius / 32);
            var (cx, cz) = GridCell(point);

            for (var x = cx - r; x <= cx + r; x++)
                for (var z = cz - r; z <= cz + r; z++)
                    if (grid.TryGetValue((x, z), out var list))
                        foreach (var k in list)
                            yield return k;
        }

        // Height of b's centreline at a's position, extrapolated along b's own grade.
        double HeightAt(Sample sample, double3 point)
            => sample.Position.y + sample.Grade * math.dot(point.xz - sample.Position.xz, sample.Direction);

        var paired = Paired(sections, samples, first, Near, HeightAt, Fail, token);
        Clearance(sections, samples, first, Near, HeightAt, Fail, token);

        return new(sections.Count, kilometres, paired, failures);
    }

    // Groups failures of one category into visible issues. Candidates are already
    // the same category and at most IssueRadius apart; SameIssue decides the rest.
    public static IReadOnlyList<Issue> Cluster(IReadOnlyList<Failure> failures, CancellationToken token)
    {
        var parent = Enumerable.Range(0, failures.Count).ToArray();

        int Root(int i)
        {
            while (parent[i] != i)
                i = parent[i] = parent[parent[i]];

            return i;
        }

        var grid = new Dictionary<(string, int, int), List<int>>();

        for (var i = 0; i < failures.Count; i++)
        {
            token.ThrowIfCancellationRequested();
            var failure = failures[i];
            var cx = (int)Math.Floor(failure.Position.x / IssueRadius);
            var cz = (int)Math.Floor(failure.Position.z / IssueRadius);

            for (var x = cx - 1; x <= cx + 1; x++)
                for (var z = cz - 1; z <= cz + 1; z++)
                    if (grid.TryGetValue((failure.Category, x, z), out var list))
                        foreach (var j in list)
                            if (math.distance(failures[j].Position.xz, failure.Position.xz) <= IssueRadius
                                && SameIssue(failures[j], failure))
                                parent[Root(i)] = Root(j);

            if (!grid.TryGetValue((failure.Category, cx, cz), out var own))
                grid[(failure.Category, cx, cz)] = own = new();

            own.Add(i);
        }

        return Enumerable.Range(0, failures.Count).GroupBy(Root).Select(group =>
        {
            var members = group.Select(i => failures[i]).ToArray();
            var worst =
                members.OrderByDescending(failure => failure.Severity).ThenBy(failure => failure.Section).First();

            return new Issue(
                worst.Category,
                members.Length,
                worst.Severity,
                worst.Value,
                worst.Limit,
                worst.Position,
                members.Select(failure => failure.WayId).Distinct().OrderBy(wayId => wayId).ToArray());
        }).OrderByDescending(issue => issue.Severity).ThenBy(issue => issue.Category, StringComparer.Ordinal).ToArray();
    }

    // One visible issue follows one road: the same OSM way or a section touching
    // it end to end. Mere proximity would chain across dense grids and hide
    // distinct defects. Paired heights are the exception: the opposite
    // carriageway is another way, side by side, and shows the same defect.
    private static bool SameIssue(in Failure firstFailure, in Failure secondFailure) =>
        firstFailure.WayId == secondFailure.WayId
            || firstFailure.StartNodeId == secondFailure.StartNodeId
            || firstFailure.StartNodeId == secondFailure.EndNodeId
            || firstFailure.EndNodeId == secondFailure.StartNodeId
            || firstFailure.EndNodeId == secondFailure.EndNodeId
            || firstFailure.Category == "paired-height"
            && math.distance(firstFailure.Position.xz, secondFailure.Position.xz) <= 60;

    private static void Nodes(
        IReadOnlyList<PreparedSegment> sections,
        Action<string, int, double3, double, double, double> fail,
        CancellationToken token)
    {
        var arms = new Dictionary<SourceNodeKey, List<(int Section, bool Start)>>();

        for (var i = 0; i < sections.Count; i++)
            foreach (var start in new[] { true, false })
            {
                var key = new SourceNodeKey(sections[i].Kind, start ? sections[i].StartNodeId : sections[i].EndNodeId);

                if (!arms.TryGetValue(key, out var list))
                    arms[key] = list = new();

                list.Add((i, start));
            }

        foreach (var node in arms.OrderBy(entry => entry.Key.Kind).ThenBy(entry => entry.Key.Id))
        {
            token.ThrowIfCancellationRequested();
            var list = node.Value;
            var away = list.Select(item => Away(sections[item.Section], item.Start)).ToArray();

            if (away.Any(item => !item.Valid))
                continue;

            var position =
                (double3)(list[0].Start ? sections[list[0].Section].CourseStart : sections[list[0].Section].CourseEnd);

            if (list.Count == 2)
            {
                // A continuous road leaves a degree-two node opposite to how it arrived.
                var deflection =
                    math.degrees(Math.Acos(math.clamp(-math.dot(away[0].Direction, away[1].Direction), -1, 1)));

                if (deflection > 2)
                    fail("tangent-break", list[1].Section, position, deflection, 2, deflection / 2);

                var kink = Math.Abs(away[0].Grade + away[1].Grade) * 100;
                var limit = Math.Min(10 / For(sections[list[0].Section]).K, 10 / For(sections[list[1].Section]).K);

                if (sections[list[0].Section].Tunnel && sections[list[1].Section].Tunnel)
                    limit /= VisualCompilerTable.TunnelKShare;

                if (kink > limit)
                    fail("grade-change", list[1].Section, position, kink, limit, kink / limit);

                continue;
            }

            if (node.Key.Kind != NetworkKind.Road)
                continue;

            for (var a = 0; a < list.Count; a++)
                for (var b = a + 1; b < list.Count; b++)
                {
                    var firstSegment = sections[list[a].Section];
                    var secondSegment = sections[list[b].Section];

                    if (firstSegment.ProtectedLink || secondSegment.ProtectedLink)
                        continue;

                    // A two-way road splitting into its one-way carriageways diverges at a small angle by design.
                    if (firstSegment.OneWay && secondSegment.OneWay && list[a].Start != list[b].Start)
                        continue;

                    var angle =
                        math.degrees(Math.Acos(math.clamp(math.dot(away[a].Direction, away[b].Direction), -1, 1)));

                    if (angle < 25)
                        fail("acute-junction", list[b].Section, position, angle, 25, 25 / Math.Max(1, angle));
                }
        }
    }

    // Opposite one-way carriageways of one class within that class's separation
    // band, regardless of whether the design paired them (report 43: 4% coverage).
    private static PairedCoverage Paired(IReadOnlyList<PreparedSegment> sections, List<Sample> samples, int[] first,
        Func<double3, double, IEnumerable<int>> near,
            Func<Sample, double3, double> heightAt,
            Action<string, int, double3, double, double, double> fail,
            CancellationToken token)
    {
        bool Candidate(PreparedSegment segment)
            => segment.Kind == NetworkKind.Road
                && segment.OneWay
                && !segment.ProtectedLink
                && !segment.Roundabout
                && For(segment).PairSeparation > 0;

        var eligible = 0d;
        var matched = 0d;
        var failed = 0d;

        for (var i = 0; i < sections.Count; i++)
        {
            var segment = sections[i];

            if (!Candidate(segment))
                continue;

            token.ThrowIfCancellationRequested();
            var band = For(segment).PairSeparation;
            var worst = 0d;
            var worstAt = default(double3);

            for (var k = first[i]; k < first[i + 1]; k++)
            {
                var firstSample = samples[k];
                eligible += firstSample.Step;
                var best = double.PositiveInfinity;
                var delta = 0d;

                foreach (var j in near(firstSample.Position, band))
                {
                    var secondSample = samples[j];
                    var otherSegment = sections[secondSample.Section];

                    if (secondSample.Section == i
                        || !Candidate(otherSegment)
                        || otherSegment.Classification != segment.Classification
                        || otherSegment.Layer != segment.Layer
                        || math.dot(firstSample.Direction, secondSample.Direction) > -.966)
                        continue;

                    var offset = secondSample.Position.xz - firstSample.Position.xz;
                    var along = math.dot(offset, firstSample.Direction);
                    var lateral = Math.Abs(NetworkDesignChecks.Cross(firstSample.Direction, offset));

                    if (Math.Abs(along) > Spacing || lateral < 5 || lateral > band || lateral >= best)
                        continue;

                    best = lateral;
                    delta = Math.Abs(firstSample.Position.y - heightAt(secondSample, firstSample.Position));
                }

                if (double.IsPositiveInfinity(best))
                    continue;

                matched += firstSample.Step;

                if (delta > PairedTolerance)
                {
                    failed += firstSample.Step;

                    if (delta > worst)
                    {
                        worst = delta;
                        worstAt = firstSample.Position;
                    }
                }
            }

            if (worst > 0)
                fail("paired-height", i, worstAt, worst, PairedTolerance, worst / PairedTolerance);
        }

        return new(eligible, matched, failed);
    }

    // Deck-to-deck height over a lower-layer or tunnel section whose footprint the
    // bridge overlaps. Clearance includes deck depth: 5.5 m over road, 7 m over rail.
    private static void Clearance(IReadOnlyList<PreparedSegment> sections, List<Sample> samples, int[] first,
        Func<double3, double, IEnumerable<int>> near,
            Func<Sample, double3, double> heightAt,
            Action<string, int, double3, double, double, double> fail,
            CancellationToken token)
    {
        static double Half(PreparedSegment segment) => (segment.NativeWidth > 0 ? segment.NativeWidth : 8) / 2;

        // Sections within two graph steps of the bridge connect to it (a ramp
        // leaving at its abutment); their overlap is a junction, not a crossing.
        var incident = new Dictionary<SourceNodeKey, List<int>>();

        for (var i = 0; i < sections.Count; i++)
            foreach (var key in new SourceNodeKey[] {
                new(sections[i].Kind, sections[i].StartNodeId),
                new(sections[i].Kind, sections[i].EndNodeId)
                })
            {
                if (!incident.TryGetValue(key, out var list))
                    incident[key] = list = new();

                list.Add(i);
            }

        IEnumerable<int> Touching(int i)
            => new SourceNodeKey[] {
                new(sections[i].Kind, sections[i].StartNodeId),
                new(sections[i].Kind, sections[i].EndNodeId)
                }.SelectMany(nodeKey => incident[nodeKey]);

        for (var i = 0; i < sections.Count; i++)
        {
            var segment = sections[i];

            if (!segment.Bridge || segment.Tunnel)
                continue;

            token.ThrowIfCancellationRequested();
            var adjacent = new HashSet<int>(Touching(i).SelectMany(Touching));
            var ends =
                new HashSet<SourceNodeKey> {
                    new(segment.Kind, segment.StartNodeId),
                    new(segment.Kind, segment.EndNodeId)
                    };

            var worst = double.PositiveInfinity;
            var worstAt = default(double3);
            var worstLimit = 0d;

            for (var k = first[i]; k < first[i + 1]; k++)
            {
                var firstSample = samples[k];

                foreach (var j in near(firstSample.Position, Half(segment) + 15))
                {
                    var secondSample = samples[j];
                    var otherSegment = sections[secondSample.Section];

                    if (secondSample.Section == i || adjacent.Contains(secondSample.Section))
                        continue;

                    if (!(otherSegment.Layer < segment.Layer || otherSegment.Tunnel)
                        || ends.Contains(new(otherSegment.Kind, otherSegment.StartNodeId))
                        || ends.Contains(new(otherSegment.Kind, otherSegment.EndNodeId)))
                        continue;

                    if (math.distance(firstSample.Position.xz, secondSample.Position.xz)
                        > Half(segment) + Half(otherSegment) + Spacing / 2)
                        continue;

                    var gap = firstSample.Position.y - heightAt(secondSample, firstSample.Position);
                    var limit = (otherSegment.Kind == NetworkKind.Rail ? 7 : 5.5) - .5;

                    if (gap < limit && gap < worst)
                    {
                        worst = gap;
                        worstAt = firstSample.Position;
                        worstLimit = limit;
                    }
                }
            }

            if (!double.IsPositiveInfinity(worst))
                fail("crossing-clearance", i, worstAt, worst, worstLimit, worstLimit / Math.Max(.5, worst));
        }

        // Same-level overlap: road footprints of unconnected sections on one layer
        // overlapping by more than 0.5 m within 3 m of height, e.g. carriageways
        // drawn on top of each other. Rail is excluded: parallel tracks touch.
        // Each sample is a disc of half the road width, except that a section ends
        // flat where it continues within 30° into exactly one other road: there
        // the disc is clipped at the end line. Junction, dead-end and sharp-bend
        // discs stay whole, standing in for node pavement, end caps and mitres.
        var along = new double[samples.Count];

        for (var i = 0; i < sections.Count; i++)
            for (var k = first[i] + 1; k < first[i + 1]; k++)
                along[k] = along[k - 1] + samples[k].Step;

        double2 Outward(int section, long id)
            => sections[section].StartNodeId == id
                ? -samples[first[section]].Direction
                : samples[first[section + 1] - 1].Direction;

        bool Plain(int section, long id)
        {
            var arms = incident[new(sections[section].Kind, id)];

            if (arms.Count != 2
                || arms[0] == arms[1]
                || first[arms[0]] == first[arms[0] + 1]
                || first[arms[1]] == first[arms[1] + 1])
                return false;

            var other = arms[0] == section ? arms[1] : arms[0];

            return math.dot(Outward(section, id), -Outward(other, id)) > Math.Cos(Math.PI / 6);
        }

        var plainStart = new bool[sections.Count];
        var plainEnd = new bool[sections.Count];

        for (var i = 0; i < sections.Count; i++)
            if (sections[i].Kind == NetworkKind.Road && first[i] < first[i + 1])
            {
                plainStart[i] = Plain(i, sections[i].StartNodeId);
                plainEnd[i] = Plain(i, sections[i].EndNodeId);
            }

        double Reach(int k, double2 u)
        {
            var sample = samples[k];
            var segment = sections[sample.Section];
            var otherHalf = Half(segment);
            var reach = otherHalf;

            void Clip(double e, double2 outward)
            {
                var c = math.dot(u, outward);

                if (e < otherHalf && otherHalf * c > e)
                    reach =
                        Math.Min(
                            reach,
                            e * c + Math.Sqrt(otherHalf * otherHalf - e * e) * Math.Sqrt(Math.Max(0, 1 - c * c)));
            }

            if (plainStart[sample.Section])
                Clip(along[k], Outward(sample.Section, segment.StartNodeId));

            if (plainEnd[sample.Section])
                Clip(along[first[sample.Section + 1] - 1] - along[k], Outward(sample.Section, segment.EndNodeId));

            return reach;
        }

        for (var i = 0; i < sections.Count; i++)
        {
            var segment = sections[i];

            if (segment.Kind != NetworkKind.Road)
                continue;

            token.ThrowIfCancellationRequested();
            var adjacent = new HashSet<int>(Touching(i).SelectMany(Touching));
            var worst = 0d;
            var worstAt = default(double3);

            for (var k = first[i]; k < first[i + 1]; k++)
            {
                var firstSample = samples[k];

                foreach (var j in near(firstSample.Position, Half(segment) + 15))
                {
                    var secondSample = samples[j];
                    var otherSegment = sections[secondSample.Section];

                    if (secondSample.Section <= i
                        || otherSegment.Kind != NetworkKind.Road
                        || otherSegment.Layer != segment.Layer
                        || adjacent.Contains(secondSample.Section))
                        continue;

                    var distance = math.distance(firstSample.Position.xz, secondSample.Position.xz);

                    if (Half(segment) + Half(otherSegment) - distance <= .5)
                        continue;

                    var u = distance > 1e-6 ? (secondSample.Position.xz - firstSample.Position.xz) / distance : default;
                    var overlap =
                        distance > 1e-6 ? Reach(k, u) + Reach(j, -u) - distance : Half(segment) + Half(otherSegment);

                    if (overlap > .5
                        && Math.Abs(firstSample.Position.y - heightAt(secondSample, firstSample.Position)) < 3
                        && overlap > worst)
                    {
                        worst = overlap;
                        worstAt = firstSample.Position;
                    }
                }
            }

            if (worst > 0)
                fail("same-level-overlap", i, worstAt, worst, .5, worst / .5);
        }
    }

    private static (bool Valid, double2 Direction, double Grade) Away(PreparedSegment segment, bool start)
    {
        var curve = NetworkDesignChecks.Course(segment);
        var d = start ? (curve.P1 - curve.P0) : (curve.P2 - curve.P3);

        if (math.lengthsq(d.xz) < 1e-6)
            d = start ? (curve.P2 - curve.P0) : (curve.P1 - curve.P3);

        var horizontal = math.length(d.xz);

        return horizontal < 1e-6 ? (false, default, 0) : (true, d.xz / horizontal, d.y / horizontal);
    }

    internal static (int, int) GridCell(double3 point)
        => ((int)Math.Floor(point.x / 32), (int)Math.Floor(point.z / 32));
}
