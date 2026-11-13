using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using ImportEarthLocation.Geo;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// What a finished plan looks like against the map data, by road class. Distances
// are metres in local XZ. Lateral error is measured to the OSM ways a section
// claims, so a merged avenue reads half its carriageway spacing and a separated
// carriageway reads its offset. This is a comparison measure, not a placement veto.
public static class RoadQualityReport
{
    public sealed record ClassRow(
        string Class,
        int Sections,
        double Kilometres,
        double LateralP50,
        double LateralP95,
        double LateralMaximum,
        double KilometresOver10,
        int Jogs,
            int KinksAtPlainNodes,
            int KinksThroughJunctions,
            int UnderRadiusFloor,
            int UnderRadius50,
            int UnderMinimumLength,
            double SectionsPerKilometre,
        double BridgeKilometres,
            double SourceBridgeKilometres,
            double BridgeRatio,
            double SourceKilometres,
            double MissingSourceKilometres,
            int MissingSourceWays);

    public sealed record Report(
        int SchemaVersion,
        IReadOnlyList<ClassRow> Classes,
        int OverallSections,
        double OverallKilometres);

    private const double Spacing = 5, KinkDegrees = 2, JogMeters = 10, LinkClearanceJogMeters = 20;

    private static readonly string[] Groups =
        { "motorway", "motorway_link", "trunk", "trunk_link", "primary", "secondary", "tertiary", "street" };

    public static string Group(string classification) => classification switch
    {
        "motorway" or "motorway_link" or "trunk" or "trunk_link" => classification,
        "primary" or "primary_link" => "primary",
        "secondary" or "secondary_link" => "secondary",
        "tertiary" or "tertiary_link" => "tertiary",
        _ => "street"
    };

    public static Report Measure(
        IReadOnlyList<PreparedSegment> sections,
        OsmDataset dataset,
        ImportRequest request,
        CancellationToken token)
    {
        var half = request.MapSize.WidthMeters / 2;
        var lines = new Dictionary<long, double2[]>();
        var sourceKm = new Dictionary<string, double>();
        var sourceBridge = new Dictionary<string, double>();
        var sourceWays = new Dictionary<long, (string Group, double Km)>();

        foreach (var way in dataset.Ways.Values)
        {
            if (!way.Tags.TryGetValue("highway", out var highway))
                continue;

            var points = way.NodeIds.Where(dataset.Nodes.ContainsKey).Select(id =>
{
    var node = dataset.Nodes[id];
    var point = GeoMath.ToLocalMeters(node.Latitude, node.Longitude, request.Latitude, request.Longitude);

    return new double2(point.x, point.z);
}).ToArray();

            if (points.Length < 2)
                continue;

            lines[way.Id] = points;
            // Ways reaching the map edge are clipped, not missing.
            var inside = points.All(point => Math.Abs(point.x) < half - 200 && Math.Abs(point.y) < half - 200);
            var km = 0d;

            for (var k = 1; k < points.Length; k++)
                if (Math.Abs(points[k].x) < half && Math.Abs(points[k].y) < half)
                    km += math.distance(points[k - 1], points[k]) / 1000;

            var group = Group(highway);
            Add(sourceKm, group, km);

            if (way.Tags.TryGetValue("bridge", out var bridge) && bridge != "no")
                Add(sourceBridge, group, km);

            if (inside)
                sourceWays[way.Id] = (group, km);
        }

        var claimed = new HashSet<long>();

        foreach (var segment in sections)
        {
            claimed.Add(segment.WayId);

            foreach (var wayId in segment.SourceWayIds)
                claimed.Add(wayId);
        }

        var arms = new Dictionary<SourceNodeKey, List<(PreparedSegment S, bool Start)>>();

        foreach (var segment in sections)
            foreach (var (id, start) in new[] { (segment.StartNodeId, true), (segment.EndNodeId, false) })
            {
                var key = new SourceNodeKey(segment.Kind, id);

                if (!arms.TryGetValue(key, out var list))
                    arms[key] = list = new();

                list.Add((segment, start));
            }

        var rows = new List<ClassRow>();

        foreach (var group in Groups)
        {
            var mine =
                sections
                    .Where(segment => segment.Kind == NetworkKind.Road && Group(segment.Classification) == group)
                    .ToArray();

            if (mine.Length == 0 && !sourceKm.ContainsKey(group))
                continue;

            var lateral = new List<double>();
            var over10 = 0d;
            var jogs = 0;
            var km = 0d;
            var tight = 0;
            var tight50 = 0;
            var shortPieces = 0;
            var bridgeKm = 0d;

            foreach (var segment in mine)
            {
                token.ThrowIfCancellationRequested();
                var curve =
                    new Cubic3d(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd);

                var length = NetworkDesignChecks.HorizontalArcLength(curve);
                km += length / 1000;

                if (segment.Bridge)
                    bridgeKm += length / 1000;

                var table = VisualCompilerTable.For(segment);

                if (length < table.MinimumSegment && !segment.Roundabout)
                    shortPieces++;

                var ways =
                    segment.SourceWayIds
                        .Append(segment.WayId)
                        .Distinct()
                        .Where(lines.ContainsKey)
                        .Select(wayId => lines[wayId])
                        .ToArray();

                var count = Math.Max(2, (int)(length / Spacing) + 1);
                var low = double.PositiveInfinity;
                var high = 0d;
                var worstCurvature = 0d;

                for (var k = 0; k < count; k++)
                {
                    var t = k / (count - 1d);
                    worstCurvature = Math.Max(worstCurvature, Math.Abs(NetworkDesignChecks.Curvature(curve, t)));

                    if (ways.Length == 0)
                        continue;

                    var point = NetworkDesignChecks.Evaluate(curve, t).xz;
                    var best = double.PositiveInfinity;

                    foreach (var way in ways)
                        for (var j = 1; j < way.Length; j++)
                            best = Math.Min(best, Distance(point, way[j - 1], way[j]));

                    lateral.Add(best);

                    if (best > 10)
                        over10 += length / count / 1000;

                    low = Math.Min(low, best);
                    high = Math.Max(high, best);
                }

                // A link leaving a highway must move half both widths off its mapped line; that is
                // the design, not a jog, so only a larger swing counts there.
                bool OnHighway(long id)
                    => arms[new(
                        segment.Kind,
                        id)].Any(
                            item => item.S != segment && NativeAssetKeys.IsHighway(item.S) && !item.S.ProtectedLink);

                if (ways.Length > 0
                    && high - low
                        > (segment.ProtectedLink && (OnHighway(segment.StartNodeId) || OnHighway(segment.EndNodeId))
                            ? LinkClearanceJogMeters
                            : JogMeters))
                    jogs++;

                if (!segment.Roundabout)
                {
                    if (worstCurvature * table.RadiusFloor > 1)
                        tight++;

                    if (worstCurvature * 50 > 1)
                        tight50++;
                }
            }

            lateral.Sort();

            double Quantile(double q) => lateral.Count == 0 ? 0 : lateral[(int)(q * (lateral.Count - 1))];

            var plain = 0;
            var through = 0;

            foreach (var node in arms.Values)
            {
                if (!node.Any(item => item.S.Kind == NetworkKind.Road && Group(item.S.Classification) == group))
                    continue;

                if (node.Count == 2)
                {
                    if (Deflection(node[0], node[1]) > KinkDegrees)
                        plain++;

                    continue;
                }

                var same = node.Where(item => Group(item.S.Classification) == group).ToArray();

                if (same.Length < 2)
                    continue;

                var least = 180d;

                for (var a = 0; a < same.Length; a++)
                    for (var b = a + 1; b < same.Length; b++)
                        least = Math.Min(least, Deflection(same[a], same[b]));

                if (least > KinkDegrees)
                    through++;
            }

            var missing =
                sourceWays.Where(entry => entry.Value.Group == group && !claimed.Contains(entry.Key)).ToArray();

            sourceBridge.TryGetValue(group, out var osmBridge);
            sourceKm.TryGetValue(group, out var osmKm);
            rows.Add(
                new(
                    group,
                    mine.Length,
                    km,
                    Quantile(.5),
                    Quantile(.95),
                    lateral.Count == 0 ? 0 : lateral[lateral.Count - 1],
                    over10,
                    jogs,
                    plain,
                    through,
                    tight,
                    tight50,
                    shortPieces,
                    km > 0 ? mine.Length / km : 0,
                bridgeKm,
                    osmBridge,
                    osmBridge > .01 ? bridgeKm / osmBridge : 0,
                    osmKm,
                    missing.Sum(entry => entry.Value.Km),
                    missing.Length));
        }

        return new(1, rows, sections.Count, rows.Sum(row => row.Kilometres));
    }

    private static void Add(Dictionary<string, double> totals, string key, double value)
    {
        totals.TryGetValue(key, out var old);
        totals[key] = old + value;
    }

    private static double Distance(double2 point, double2 vectorA, double2 vectorB)
    {
        var d = vectorB - vectorA;
        var l = math.dot(d, d);
        var t = l > 0 ? math.clamp(math.dot(point - vectorA, d) / l, 0, 1) : 0;

        return math.distance(point, vectorA + t * d);
    }

    private static double Deflection(
        (PreparedSegment S, bool Start) firstItem,
        (PreparedSegment S, bool Start) secondItem) =>
        math.degrees(Math.Acos(math.clamp(-math.dot(Away(firstItem), Away(secondItem)), -1, 1)));

    private static double2 Away((PreparedSegment S, bool Start) arm)
    {
        var segment = arm.S;
        double3 p0 = segment.CourseStart,
            p1 = segment.CourseControl1,
            p2 = segment.CourseControl2,
            p3 = segment.CourseEnd;

        var d = arm.Start ? p1 - p0 : p2 - p3;

        if (math.lengthsq(d.xz) < 1e-6)
            d = arm.Start ? p2 - p0 : p1 - p3;

        return math.normalizesafe(d.xz);
    }
}
