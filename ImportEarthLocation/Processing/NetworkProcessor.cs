using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using ImportEarthLocation.Geo;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

public static class NetworkProcessor
{
    public static IReadOnlyList<PreparedSegment> BuildSegments(
        OsmDataset dataset,
        ImportRequest request,
        ImportLog log,
        CancellationToken token = default)
    {
        var segments = new List<PreparedSegment>();
        var dedupe = new HashSet<string>(StringComparer.Ordinal);
        var centreLanes = ReversibleCentreLanes(dataset, request);

        // Underground-link omission belongs to TunnelIntervals on the selected,
        // clipped segment graph. Whole-way flood filling here crossed branches
        // and discarded surface continuations before that classifier could see them.
        foreach (var way in dataset.Ways.Values.OrderBy(way => way.Id))
        {
            token.ThrowIfCancellationRequested();

            if (way.Tags.TryGetValue("highway", out var excluded) && NetworkSelection.IsExcludedStreetLink(excluded))
            {
                log.Count("Street links explicitly excluded");
                continue;
            }

            // Support ways supply context only; they never override the selected road families.
            if (!TryClassify(way, request, out var kind, out var classification))
            {
                if (request.RoadLevel != RoadImportLevel.None && way.Tags.ContainsKey("highway"))
                    log.Count("Roads excluded by selected tier");

                continue;
            }

            if (centreLanes.Contains(way.Id))
            {
                log.Count("Reversible centre lanes beside one-way carriageways omitted");
                continue;
            }

            var oneWay = kind == NetworkKind.Road && ParseOneWay(way.Tags, classification);
            var lanes = ParseLanes(way.Tags, kind, oneWay, classification);

            if (kind == NetworkKind.Rail
                && way.Tags.TryGetValue("service", out var railService)
                && railService is "yard" or "siding" or "spur")
            {
                log.Count("Rail service branches intentionally excluded");
                continue;
            }

            if (kind == NetworkKind.Road && Reversible(way.Tags, classification))
            {
                oneWay = false;
                lanes = 2;
                log.Count("Reversible highways normalized to two way two lanes");
            }

            var bridge = IsYes(way.Tags, "bridge");
            var tunnel =
                IsYes(way.Tags, "tunnel")
                    && (!way.Tags.TryGetValue("tunnel", out var tunnelType) || tunnelType != "building_passage");

            var layer = ParseLayer(way.Tags, bridge, tunnel);

            // Asset family is now an explicit user policy, not inferred from expressway tags.
            if (kind == NetworkKind.Road)
                log.Count(
                    classification.StartsWith("motorway") || classification.StartsWith("trunk")
                        ? "Highway-family source ways"
                        : "Street-family source ways");

            for (var index = 0; index < way.NodeIds.Length - 1; index++)
            {
                var startId = way.NodeIds[index];
                var endId = way.NodeIds[index + 1];

                if (!dataset.Nodes.TryGetValue(startId, out var startNode)
                    || !dataset.Nodes.TryGetValue(endId, out var endNode))
                {
                    log.Count("Network segments with missing endpoints omitted");

                    if (log.Metric("Network segments with missing endpoints omitted") <= 25)
                        log.Warn(
                            $"OSM way {way.Id} segment {startId}->{endId}: unavailable endpoint; adjacent valid segments retained, gap not fabricated.");

                    continue;
                }

                var low = Math.Min(startId, endId);
                var high = Math.Max(startId, endId);
                var key =
                    $"{kind}:{classification}:{lanes}:{oneWay}:{bridge}:{tunnel}:{layer}:{(oneWay ? startId : low)}:{(oneWay ? endId : high)}";

                if (!dedupe.Add(key))
                {
                    log.Count("Duplicates/artifacts removed");
                    continue;
                }

                var start =
                    GeoMath.ToLocalMeters(startNode.Latitude, startNode.Longitude, request.Latitude, request.Longitude);

                var end =
                    GeoMath.ToLocalMeters(endNode.Latitude, endNode.Longitude, request.Latitude, request.Longitude);

                if (!math.all(math.isfinite(start)) || !math.all(math.isfinite(end)))
                {
                    log.Count("Invalid network segments skipped");
                    continue;
                }

                var originalStart = start;
                var originalEnd = end;

                if (!ClipToMap(ref start, ref end, (float)(request.MapSize.WidthMeters / 2d)))
                    continue;

                if (math.distance(start, end) < 0.05f)
                {
                    log.Count("Duplicates/artifacts removed");
                    continue;
                }

                // Clipped ends must not reuse an out-of-map topology node shared by other edges.
                if (math.distance(originalStart, start) > 0.001f)
                    startId = ClippedId(low, high, start);

                if (math.distance(originalEnd, end) > 0.001f)
                    endId = ClippedId(low, high, end);

                if (oneWay && way.Tags.TryGetValue("oneway", out var direction) && direction == "-1")
                {
                    var position = start;
                    start = end;
                    end = position;
                    var nodeId = startId;
                    startId = endId;
                    endId = nodeId;
                }

                way.Tags.TryGetValue("name", out var name);
                way.Tags.TryGetValue("ref", out var reference);
                way.Tags.TryGetValue("service", out var service);
                var incline = ParseIncline(way.Tags);

                if (oneWay && way.Tags.TryGetValue("oneway", out var reverse) && reverse == "-1" && incline.HasValue)
                    incline = -incline.Value;

                segments.Add(
                    new PreparedSegment(
                        way.Id,
                        startId,
                        endId,
                        start,
                        end,
                        kind,
                        classification,
                        lanes,
                        oneWay,
                        bridge,
                        tunnel,
                        layer)
                    {
                        Name = name ?? "",
                        Reference = reference ?? "",
                        RailService = service ?? "",
                        ExplicitIncline = incline,
                        SourceWayIds = new[] { way.Id },
                        OriginalGeometry = new[] { originalStart, originalEnd },
                        SourceTags = way.Tags,
                        RawSourceTags = dataset.RawWayTags.TryGetValue(way.Id, out var rawTags) ? rawTags : way.Tags,
                        PhysicalTracks = kind == NetworkKind.Rail ? lanes : 0,
                        BothWaysLanes = LaneCount(way.Tags, "lanes:both_ways"),
                        SourceWidthMeters =
                        NetworkEvidence.Measure(way.Tags, "width") ?? NetworkEvidence.Measure(way.Tags, "est_width"),
                        SourceSpeedKph = NetworkEvidence.Measure(way.Tags, "maxspeed", true),
                        StartMiniRoundabout = NetworkEvidence.NodeIs(dataset, startId, "highway", "mini_roundabout"),
                        EndMiniRoundabout = NetworkEvidence.NodeIs(dataset, endId, "highway", "mini_roundabout"),
                        StartTerminal =
                        NetworkEvidence.NodeIs(dataset, startId, "railway", "buffer_stop")
                            || NetworkEvidence.NodeIs(dataset, startId, "highway", "turning_circle"),
                        EndTerminal =
                        NetworkEvidence.NodeIs(dataset, endId, "railway", "buffer_stop")
                            || NetworkEvidence.NodeIs(dataset, endId, "highway", "turning_circle"),
                        ExplicitLanes =
                        LaneCount(way.Tags, "lanes") > 0
                            || LaneCount(way.Tags, "lanes:forward") > 0
                            || LaneCount(way.Tags, "lanes:backward") > 0
                            || way.Tags.Any(
                            entry => entry.Key.StartsWith("turn:lanes", StringComparison.Ordinal)
                                && !string.IsNullOrWhiteSpace(entry.Value)),
                        ForwardLanes =
                        LaneCount(
                            way.Tags,
                            oneWay && way.Tags.TryGetValue("oneway", out var otherDirection) && otherDirection == "-1"
                                ? "lanes:backward"
                                : "lanes:forward"),
                        BackwardLanes = oneWay ? 0 : LaneCount(way.Tags, "lanes:backward"),
                        Roundabout = way.Tags.TryGetValue("junction", out var junction) && junction == "roundabout",
                        ExplicitCutSides =
                        Sides(
                            way.Tags,
                            "cutting",
                            oneWay && way.Tags.TryGetValue("oneway", out var cutDirection) && cutDirection == "-1"),
                        ExplicitWallSides =
                        Sides(
                            way.Tags,
                            "retaining_wall",
                            oneWay && way.Tags.TryGetValue("oneway", out var wallDirection) && wallDirection == "-1")
                    });
            }
        }

        log.Diagnostic(
            $"NETWORK SEMANTICS sections={segments.Count}; explicitLaneSections={segments.Count(segment => segment.ExplicitLanes)}; widthEvidence={segments.Count(segment => segment.SourceWidthMeters.HasValue)}; speedEvidence={segments.Count(segment => segment.SourceSpeedKph.HasValue)}; miniRoundaboutEnds={segments.Count(segment => segment.StartMiniRoundabout || segment.EndMiniRoundabout)}; validTerminalEnds={segments.Count(segment => segment.StartTerminal || segment.EndTerminal)}; undergroundSections={segments.Count(segment => segment.Tunnel)}");

        return segments;
    }

    private static long ClippedId(long low, long high, float3 position)
    {
        unchecked
        {
            return -1L - (long)(((ulong)(low * 397 ^ high) ^ math.hash(position)) % (ulong)long.MaxValue);
        }
    }

    private static bool ClipToMap(ref float3 start, ref float3 end, float halfWidth)
    {
        var delta = end - start;
        var enter = 0f;
        var exit = 1f;

        if (!ClipAxis(-delta.x, start.x + halfWidth, ref enter, ref exit)
            || !ClipAxis(delta.x, halfWidth - start.x, ref enter, ref exit)
                || !ClipAxis(-delta.z, start.z + halfWidth, ref enter, ref exit)
                || !ClipAxis(delta.z, halfWidth - start.z, ref enter, ref exit))
            return false;

        end = start + delta * exit;
        start += delta * enter;

        return true;
    }

    private static bool ClipAxis(float direction, float distance, ref float enter, ref float exit)
    {
        if (direction == 0f)
            return distance >= 0f;

        var ratio = distance / direction;

        if (direction < 0f)
            enter = Math.Max(enter, ratio);
        else
            exit = Math.Min(exit, ratio);

        return enter <= exit;
    }

    // Ground height in metres at a map point (x, z in metres from the map's centre).
    public static float Sample(float x, float z, float[] heights, double width)
    {
        var u = Numeric.Clamp(x / width + 0.5d, 0d, 1d);
        var v = Numeric.Clamp(z / width + 0.5d, 0d, 1d);
        var resolution = TerrainSampling.Resolution(heights);

        return (float)TerrainSampling.Height(heights, resolution, u, v);
    }

    private static bool TryClassify(OsmWay way, ImportRequest request, out NetworkKind kind, out string classification)
    {
        if (way.Tags.TryGetValue("railway", out var railway)
            && railway.Equals("rail", StringComparison.OrdinalIgnoreCase))
        {
            if (!request.ImportRail || IsYes(way.Tags, "abandoned") || IsYes(way.Tags, "disused"))
            {
                kind = default;
                classification = "";

                return false;
            }

            kind = NetworkKind.Rail;
            classification = "rail";

            return true;
        }

        if (way.Tags.TryGetValue("highway", out var highway)
            && NetworkSelection.Includes(request.RoadLevel, highway))
        {
            kind = NetworkKind.Road;
            classification = highway;

            return true;
        }

        kind = default;
        classification = "";

        return false;
    }

    private static int ParseLanes(
        IReadOnlyDictionary<string, string> tags,
        NetworkKind kind,
        bool oneWay,
        string classification)
    {
        if (Reversible(tags, classification))
            return 2;

        if (kind == NetworkKind.Rail)
            return tags.TryGetValue("tracks", out var tracks) && int.TryParse(tracks, out var count)
                ? Numeric.Clamp(count, 1, 2)
                : 1;

        var lanes = LaneCount(tags, "lanes");
        var forward = LaneCount(tags, "lanes:forward");
        var backward = LaneCount(tags, "lanes:backward");

        if (oneWay)
        {
            var reverse = tags.TryGetValue("oneway", out var direction) && direction == "-1";
            var directional = reverse ? backward : forward;

            if (directional > 0)
                lanes = directional;
        }
        else if (lanes == 0 && forward > 0 && backward > 0)
            lanes = forward + backward + LaneCount(tags, "lanes:both_ways");

        if (lanes == 0)
        {
            // The number of directional lane entries is useful when the simple
            // count is absent. Do not count conditional alternatives as lanes.
            var reverse = oneWay && tags.TryGetValue("oneway", out var direction) && direction == "-1";
            var laneKey = oneWay ? (reverse ? "turn:lanes:backward" : "turn:lanes:forward") : "turn:lanes";

            if (!tags.TryGetValue(laneKey, out var entries))
                tags.TryGetValue("turn:lanes", out entries);

            if (!string.IsNullOrWhiteSpace(entries))
                lanes = entries.Split('|').Length;

            if (lanes == 0
                && (classification == "motorway" || classification == "trunk")
                && NetworkEvidence.Measure(tags, "width") is float width)
                lanes = Numeric.Clamp((int)Math.Round((width - 2) / 3.5f), 1, 6);
        }

        return lanes > 0
            ? Numeric.Clamp(lanes, 1, 32)
            : classification.EndsWith("_link", StringComparison.OrdinalIgnoreCase) ? 1 : 2;
    }

    private static int LaneCount(IReadOnlyDictionary<string, string> tags, string key)
        => tags.TryGetValue(key, out var value)
            && int.TryParse(value.Trim(), NumberStyles.Integer, CultureInfo.InvariantCulture, out var count)
                && count > 0
            ? count
            : 0;

    private static NetworkSides Sides(IReadOnlyDictionary<string, string> tags, string key, bool reverse)
    {
        var result = NetworkSides.None;

        if (tags.TryGetValue(key, out var value))
            result = value switch
            {
                "yes" or "both" or "1" or "true" => NetworkSides.Both,
                "left" => NetworkSides.Left,
                "right" => NetworkSides.Right,
                _ => NetworkSides.None
            };

        if (IsYes(tags, key + ":left"))
            result |= NetworkSides.Left;

        if (IsYes(tags, key + ":right"))
            result |= NetworkSides.Right;

        return reverse
            ? ((result & NetworkSides.Left) != 0 ? NetworkSides.Right : 0)
                | ((result & NetworkSides.Right) != 0 ? NetworkSides.Left : 0)
            : result;
    }

    private static float? ParseIncline(IReadOnlyDictionary<string, string> tags)
    {
        if (!tags.TryGetValue("incline", out var value))
            return null;

        value = value.Trim();
        var degrees = value.EndsWith("°", StringComparison.Ordinal);

        if (!float.TryParse(value.TrimEnd('%', '°'), NumberStyles.Float, CultureInfo.InvariantCulture, out var parsed)
            || float.IsNaN(parsed)
            || float.IsInfinity(parsed))
            return null;

        var grade = degrees ? (float)Math.Tan(parsed * Math.PI / 180) : parsed / 100f;

        return Math.Abs(grade) <= .348f ? grade : null;
    }

    private static bool ParseOneWay(IReadOnlyDictionary<string, string> tags, string classification)
    {
        if (Reversible(tags, classification))
            return false;

        if (tags.TryGetValue("oneway", out var value))
            return value is "yes" or "1" or "true" or "-1";

        // Selected motorway/trunk links default to one-way; explicit oneway=no wins.
        return classification.Equals("motorway", StringComparison.OrdinalIgnoreCase)
            || classification is "motorway_link" or "trunk_link"
            || (tags.TryGetValue("junction", out var junction) && junction == "roundabout");
    }

    private static int ParseLayer(IReadOnlyDictionary<string, string> tags, bool bridge, bool tunnel)
    {
        if (tags.TryGetValue("layer", out var value)
            && int.TryParse(value, NumberStyles.Integer, CultureInfo.InvariantCulture, out var layer))
            return Numeric.Clamp(layer, -5, 5);

        if (bridge)
            return 1;

        if (tunnel)
            return -1;

        return 0;
    }

    private static bool IsYes(IReadOnlyDictionary<string, string> tags, string key) =>
        tags.TryGetValue(key, out var value) && value is not ("no" or "0" or "false");

    // A reversible lane on an ordinary street (Montreal's Rue Bridge) is a
    // centre carriageway whose direction changes with the time of day. The game
    // has no such lanes, and imported as a two-way road it lands a few metres
    // from the one-way carriageways on either side. It is left out when every
    // node and segment midpoint lies within this many metres of a one-way way
    // with the same name or ref. A reversible street without such companions
    // stays a two-way road; motorway and trunk lanes keep the Reversible policy.
    private const float CentreLaneCompanionDistance = 30;

    private static HashSet<long> ReversibleCentreLanes(OsmDataset dataset, ImportRequest request)
    {
        var centreLanes = new HashSet<long>();

        var reversible = dataset.Ways.Values
            .Where(way => way.Tags.TryGetValue("highway", out var highway)
                && !highway.StartsWith("motorway", StringComparison.Ordinal)
                && !highway.StartsWith("trunk", StringComparison.Ordinal)
                && way.Tags.TryGetValue("oneway", out var direction)
                && direction == "reversible"
                && RoadLabel(way) != null)
            .ToList();

        if (reversible.Count == 0)
            return centreLanes;

        var labels = new HashSet<string>(reversible.Select(RoadLabel), StringComparer.Ordinal);

        var companions = dataset.Ways.Values
            .Where(way => way.Tags.ContainsKey("highway")
                && way.Tags.TryGetValue("oneway", out var direction)
                && direction is "yes" or "1" or "true" or "-1"
                && labels.Contains(RoadLabel(way)))
            .GroupBy(RoadLabel)
            .ToDictionary(
                group => group.Key,
                group => group.Select(Points).Where(points => points != null).ToList());

        float3[] Points(OsmWay way)
        {
            var points = new float3[way.NodeIds.Length];

            for (var index = 0; index < points.Length; index++)
            {
                if (!dataset.Nodes.TryGetValue(way.NodeIds[index], out var node))
                    return null;

                points[index] = GeoMath.ToLocalMeters(node.Latitude, node.Longitude, request.Latitude, request.Longitude);
            }

            return points;
        }

        bool Accompanied(float3 point, List<float3[]> lines)
        {
            foreach (var line in lines)
            {
                for (var index = 0; index < line.Length - 1; index++)
                {
                    var start = line[index].xz;
                    var along = line[index + 1].xz - start;
                    var length = math.lengthsq(along);
                    var t = length > 0 ? math.saturate(math.dot(point.xz - start, along) / length) : 0;

                    if (math.distance(point.xz, start + t * along) <= CentreLaneCompanionDistance)
                        return true;
                }
            }

            return false;
        }

        foreach (var way in reversible)
        {
            var points = Points(way);

            if (points == null || points.Length < 2 || !companions.TryGetValue(RoadLabel(way), out var lines))
                continue;

            var accompanied = true;

            for (var index = 0; index < points.Length && accompanied; index++)
            {
                accompanied = Accompanied(points[index], lines)
                    && (index == 0 || Accompanied((points[index - 1] + points[index]) / 2, lines));
            }

            if (accompanied)
                centreLanes.Add(way.Id);
        }

        return centreLanes;
    }

    private static string RoadLabel(OsmWay way)
    {
        if (way.Tags.TryGetValue("name", out var name))
            return name;

        return way.Tags.TryGetValue("ref", out var reference) ? reference : null;
    }

    private static bool Reversible(IReadOnlyDictionary<string, string> tags, string classification) =>
        (classification.StartsWith("motorway", StringComparison.Ordinal)
            || classification.StartsWith("trunk", StringComparison.Ordinal))
            && (tags.TryGetValue("oneway", out var direction) && direction is "reversible" or "alternating"
                || LaneCount(tags, "lanes:reversible") > 0);
}
