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
    private static readonly HashSet<string> MajorRoads = new(StringComparer.OrdinalIgnoreCase)
    {
        "motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link"
    };

    public static IReadOnlyList<PreparedSegment> BuildSegments(OsmDataset dataset, ImportRequest request, ImportLog log, CancellationToken token = default)
    {
        var segments = new List<PreparedSegment>();
        var dedupe = new HashSet<string>(StringComparer.Ordinal);
        foreach (var way in dataset.Ways.Values.OrderBy(way => way.Id))
        {
            token.ThrowIfCancellationRequested();
            if (!TryClassify(way, request, out var kind, out var classification))
            { if(request.RoadLevel != RoadImportLevel.None && way.Tags.ContainsKey("highway")) log.Count("Roads excluded by selected tier"); continue; }
            var lanes = ParseLanes(way.Tags, kind);
            var oneWay = kind == NetworkKind.Road && ParseOneWay(way.Tags, classification);
            var bridge = IsYes(way.Tags, "bridge");
            var tunnel = IsYes(way.Tags, "tunnel");
            var layer = ParseLayer(way.Tags, bridge, tunnel);

            for (var index = 0; index < way.NodeIds.Length - 1; index++)
            {
                var startId = way.NodeIds[index];
                var endId = way.NodeIds[index + 1];
                if (!dataset.Nodes.TryGetValue(startId, out var startNode) || !dataset.Nodes.TryGetValue(endId, out var endNode))
                {
                    log.Warn($"OSM way {way.Id} referenced a missing node; one segment was skipped.");
                    continue;
                }

                var low = Math.Min(startId, endId);
                var high = Math.Max(startId, endId);
                var key = $"{kind}:{classification}:{lanes}:{oneWay}:{bridge}:{tunnel}:{layer}:{(oneWay ? startId : low)}:{(oneWay ? endId : high)}";
                if (!dedupe.Add(key)) { log.Count("Duplicates/artifacts removed"); continue; }

                var start = GeoMath.ToLocalMeters(startNode.Latitude, startNode.Longitude, request.Latitude, request.Longitude);
                var end = GeoMath.ToLocalMeters(endNode.Latitude, endNode.Longitude, request.Latitude, request.Longitude);
                if (!math.all(math.isfinite(start)) || !math.all(math.isfinite(end))) { log.Count("Invalid network segments skipped"); continue; }
                var originalStart = start; var originalEnd = end;
                if (!ClipToMap(ref start, ref end, (float)(request.MapSize.WidthMeters / 2d))) continue;
                if (math.distance(start, end) < 0.05f) { log.Count("Duplicates/artifacts removed"); continue; }
                // Clipped ends must not reuse an out-of-map topology node shared by other edges.
                if (math.distance(originalStart, start) > 0.001f) startId = ClippedId(low, high, start);
                if (math.distance(originalEnd, end) > 0.001f) endId = ClippedId(low, high, end);
                if (oneWay && way.Tags.TryGetValue("oneway", out var direction) && direction == "-1")
                {
                    var position = start; start = end; end = position;
                    var nodeId = startId; startId = endId; endId = nodeId;
                }
                way.Tags.TryGetValue("name", out var name); way.Tags.TryGetValue("ref", out var reference); way.Tags.TryGetValue("service", out var service);
                var incline = ParseIncline(way.Tags);
                if (oneWay && way.Tags.TryGetValue("oneway", out var reverse) && reverse == "-1" && incline.HasValue) incline = -incline.Value;
                segments.Add(new PreparedSegment(way.Id, startId, endId, start, end, kind, classification, lanes, oneWay, bridge, tunnel, layer)
                { Name = name ?? "", Reference = reference ?? "", RailService = service ?? "", ExplicitIncline = incline });
            }
        }
        return segments;
    }

    private static long ClippedId(long low, long high, float3 position)
    {
        unchecked { return -1L - (long)(((ulong)(low * 397 ^ high) ^ math.hash(position)) % (ulong)long.MaxValue); }
    }

    private static bool ClipToMap(ref float3 start, ref float3 end, float halfWidth)
    {
        var delta = end - start;
        var enter = 0f;
        var exit = 1f;
        if (!ClipAxis(-delta.x, start.x + halfWidth, ref enter, ref exit) ||
            !ClipAxis(delta.x, halfWidth - start.x, ref enter, ref exit) ||
            !ClipAxis(-delta.z, start.z + halfWidth, ref enter, ref exit) ||
            !ClipAxis(delta.z, halfWidth - start.z, ref enter, ref exit)) return false;
        end = start + delta * exit;
        start += delta * enter;
        return true;
    }

    private static bool ClipAxis(float direction, float distance, ref float enter, ref float exit)
    {
        if (direction == 0f) return distance >= 0f;
        var ratio = distance / direction;
        if (direction < 0f) enter = Math.Max(enter, ratio);
        else exit = Math.Min(exit, ratio);
        return enter <= exit;
    }

    public static IReadOnlyList<PreparedSegment> AlignToTerrain(
        IReadOnlyList<PreparedSegment> segments,
        ushort[] heightmap,
        float heightScale,
        double mapWidth, CancellationToken token = default)
    {
        var aligned = new List<PreparedSegment>(segments.Count);
        foreach (var segment in segments)
        {
            token.ThrowIfCancellationRequested();
            var offset = segment.Bridge ? Math.Max(8f, segment.Layer * 6f) : segment.Tunnel ? Math.Min(-8f, segment.Layer * 6f) : segment.Layer * 6f;
            var start = segment.Start;
            var end = segment.End;
            start.y = Sample(start.x, start.z, heightmap, heightScale, mapWidth) + offset;
            end.y = Sample(end.x, end.z, heightmap, heightScale, mapWidth) + offset;
            float3? control1 = segment.Control1, control2 = segment.Control2;
            if (control1.HasValue) { var point = control1.Value; point.y = Sample(point.x, point.z, heightmap, heightScale, mapWidth) + offset; control1 = point; }
            if (control2.HasValue) { var point = control2.Value; point.y = Sample(point.x, point.z, heightmap, heightScale, mapWidth) + offset; control2 = point; }
            aligned.Add(segment with { Start = start, End = end, Control1 = control1, Control2 = control2 });
        }
        return aligned;
    }

    public static float Sample(float x, float z, ushort[] heightmap, float scale, double width)
    {
        var u = Numeric.Clamp(x / width + 0.5d, 0d, 1d);
        var v = Numeric.Clamp(z / width + 0.5d, 0d, 1d);
        var px = (int)Math.Round(u * (TerrainBuilder.Resolution - 1));
        var py = (int)Math.Round(v * (TerrainBuilder.Resolution - 1));
        return heightmap[py * TerrainBuilder.Resolution + px] / (float)ushort.MaxValue * scale;
    }

    private static bool TryClassify(OsmWay way, ImportRequest request, out NetworkKind kind, out string classification)
    {
        if (way.Tags.TryGetValue("railway", out var railway) && railway.Equals("rail", StringComparison.OrdinalIgnoreCase))
        {
            if (!request.ImportRail || IsYes(way.Tags, "abandoned") || IsYes(way.Tags, "disused")) { kind = default; classification = ""; return false; }
            kind = NetworkKind.Rail;
            classification = "rail";
            return true;
        }

        if (way.Tags.TryGetValue("highway", out var highway) &&
            NetworkSelection.Includes(request.RoadLevel, highway))
        {
            kind = NetworkKind.Road;
            classification = highway;
            return true;
        }
        kind = default;
        classification = "";
        return false;
    }

    private static int ParseLanes(IReadOnlyDictionary<string, string> tags, NetworkKind kind)
    {
        if (kind == NetworkKind.Rail) return tags.TryGetValue("tracks", out var tracks) && int.TryParse(tracks, out var count) ? Numeric.Clamp(count, 1, 2) : 1;
        if (!tags.TryGetValue("lanes", out var raw)) return 2;
        var first = raw.Split(new[] { ';' })[0];
        return int.TryParse(first, NumberStyles.Integer, CultureInfo.InvariantCulture, out var lanes) ? Numeric.Clamp(lanes, 1, 6) : 2;
    }

    private static float? ParseIncline(IReadOnlyDictionary<string,string> tags)
    {
        if(!tags.TryGetValue("incline",out var value)) return null;
        value = value.Trim(); var degrees = value.EndsWith("°",StringComparison.Ordinal);
        if(!float.TryParse(value.TrimEnd('%','°'),NumberStyles.Float,CultureInfo.InvariantCulture,out var parsed) || float.IsNaN(parsed) || float.IsInfinity(parsed)) return null;
        var grade = degrees ? (float)Math.Tan(parsed * Math.PI / 180) : parsed / 100f;
        return Math.Abs(grade) <= .348f ? grade : null;
    }

    private static bool ParseOneWay(IReadOnlyDictionary<string, string> tags, string classification)
    {
        if (tags.TryGetValue("oneway", out var value)) return value is "yes" or "1" or "true" or "-1";
        return classification.StartsWith("motorway", StringComparison.OrdinalIgnoreCase) || (tags.TryGetValue("junction", out var junction) && junction == "roundabout");
    }

    private static int ParseLayer(IReadOnlyDictionary<string, string> tags, bool bridge, bool tunnel)
    {
        if (tags.TryGetValue("layer", out var value) && int.TryParse(value, NumberStyles.Integer, CultureInfo.InvariantCulture, out var layer))
            return Numeric.Clamp(layer, -5, 5);
        if (bridge) return 1;
        if (tunnel) return -1;
        return 0;
    }

    private static bool IsYes(IReadOnlyDictionary<string, string> tags, string key) =>
        tags.TryGetValue(key, out var value) && value is not ("no" or "0" or "false");
}
