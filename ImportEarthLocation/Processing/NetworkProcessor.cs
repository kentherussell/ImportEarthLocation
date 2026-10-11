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
            if (!TryClassify(way, request, out var kind, out var classification) && !(dataset.SupportWayIds.Contains(way.Id) && way.Tags.TryGetValue("highway",out classification)))
            { if(request.RoadLevel != RoadImportLevel.None && way.Tags.ContainsKey("highway")) log.Count("Roads excluded by selected tier"); continue; }
            var oneWay = kind == NetworkKind.Road && ParseOneWay(way.Tags, classification);
            var lanes = ParseLanes(way.Tags, kind, oneWay, classification);
            var bridge = IsYes(way.Tags, "bridge");
            var tunnel = IsYes(way.Tags, "tunnel") && (!way.Tags.TryGetValue("tunnel",out var tunnelType)||tunnelType!="building_passage");
            var layer = ParseLayer(way.Tags, bridge, tunnel);
            // A trunk classification does not by itself prove limited-access operation.
            // Explicit ordinary-road tags override it; never classify by route number.
            if (kind == NetworkKind.Road && classification.StartsWith("trunk", StringComparison.OrdinalIgnoreCase) &&
                ((way.Tags.TryGetValue("motorroad", out var motorroad) && motorroad == "no") ||
                 (way.Tags.TryGetValue("expressway", out var expressway) && expressway == "no")))
            { classification = classification.EndsWith("_link", StringComparison.OrdinalIgnoreCase) ? "primary_link" : "primary"; log.Count("Trunk sections mapped to ordinary roads by explicit tags"); }

            for (var index = 0; index < way.NodeIds.Length - 1; index++)
            {
                var startId = way.NodeIds[index];
                var endId = way.NodeIds[index + 1];
                if (!dataset.Nodes.TryGetValue(startId, out var startNode) || !dataset.Nodes.TryGetValue(endId, out var endNode))
                {
                    throw new InvalidOperationException($"Required OSM way {way.Id} is missing node {(!dataset.Nodes.ContainsKey(startId)?startId:endId)}; map unchanged, complete dependency recovery required.");
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
                {
                    Name = name ?? "", Reference = reference ?? "", RailService = service ?? "", ExplicitIncline = incline,
                    SourceWayIds=new[]{way.Id}, OriginalGeometry=new[]{originalStart,originalEnd}, SourceTags=way.Tags,
                    RawSourceTags=dataset.RawWayTags.TryGetValue(way.Id,out var rawTags)?rawTags:way.Tags,
                    PhysicalTracks=kind==NetworkKind.Rail?lanes:0, BothWaysLanes=LaneCount(way.Tags,"lanes:both_ways"),
                    SourceWidthMeters=NetworkEvidence.Measure(way.Tags,"width")??NetworkEvidence.Measure(way.Tags,"est_width"),
                    SourceSpeedKph=NetworkEvidence.Measure(way.Tags,"maxspeed",true),
                    StartMiniRoundabout=NetworkEvidence.NodeIs(dataset,startId,"highway","mini_roundabout"),
                    EndMiniRoundabout=NetworkEvidence.NodeIs(dataset,endId,"highway","mini_roundabout"),
                    StartTerminal=NetworkEvidence.NodeIs(dataset,startId,"railway","buffer_stop")||NetworkEvidence.NodeIs(dataset,startId,"highway","turning_circle"),
                    EndTerminal=NetworkEvidence.NodeIs(dataset,endId,"railway","buffer_stop")||NetworkEvidence.NodeIs(dataset,endId,"highway","turning_circle"),
                    ExplicitLanes = LaneCount(way.Tags,"lanes") > 0 || LaneCount(way.Tags,"lanes:forward") > 0 || LaneCount(way.Tags,"lanes:backward") > 0 ||
                        way.Tags.Any(t=>t.Key.StartsWith("turn:lanes",StringComparison.Ordinal)&&!string.IsNullOrWhiteSpace(t.Value)),
                    ForwardLanes = LaneCount(way.Tags,oneWay && way.Tags.TryGetValue("oneway",out var dir) && dir == "-1" ? "lanes:backward" : "lanes:forward"),
                    BackwardLanes = oneWay ? 0 : LaneCount(way.Tags,"lanes:backward"),
                    Roundabout = way.Tags.TryGetValue("junction",out var junction) && junction == "roundabout",
                    ExplicitCutSides = Sides(way.Tags,"cutting",oneWay && way.Tags.TryGetValue("oneway",out var cutDir) && cutDir == "-1"),
                    ExplicitWallSides = Sides(way.Tags,"retaining_wall",oneWay && way.Tags.TryGetValue("oneway",out var wallDir) && wallDir == "-1")
                });
            }
        }
        log.Diagnostic($"NETWORK SEMANTICS sections={segments.Count}; explicitLaneSections={segments.Count(s=>s.ExplicitLanes)}; widthEvidence={segments.Count(s=>s.SourceWidthMeters.HasValue)}; speedEvidence={segments.Count(s=>s.SourceSpeedKph.HasValue)}; miniRoundaboutEnds={segments.Count(s=>s.StartMiniRoundabout||s.EndMiniRoundabout)}; validTerminalEnds={segments.Count(s=>s.StartTerminal||s.EndTerminal)}; undergroundSections={segments.Count(s=>s.Tunnel)}");
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
            var offset = NetworkStructure.Offset(segment);
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
        var resolution = TerrainSampling.Resolution(heightmap);
        return (float)(TerrainSampling.Encoded(heightmap, resolution, u, v) / ushort.MaxValue * scale);
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

    private static int ParseLanes(IReadOnlyDictionary<string, string> tags, NetworkKind kind, bool oneWay, string classification)
    {
        if (kind == NetworkKind.Rail) return tags.TryGetValue("tracks", out var tracks) && int.TryParse(tracks, out var count) ? Numeric.Clamp(count, 1, 2) : 1;
        var lanes=LaneCount(tags,"lanes");
        var forward=LaneCount(tags,"lanes:forward");var backward=LaneCount(tags,"lanes:backward");
        if(oneWay)
        {
            var reverse=tags.TryGetValue("oneway",out var direction)&&direction=="-1";
            var directional=reverse?backward:forward;
            if(directional>0)lanes=directional;
        }
        else if(lanes==0 && forward>0 && backward>0)lanes=forward+backward+LaneCount(tags,"lanes:both_ways");
        if(lanes==0)
        {
            // The number of directional lane entries is useful when the simple
            // count is absent. Do not count conditional alternatives as lanes.
            var reverse=oneWay&&tags.TryGetValue("oneway",out var direction)&&direction=="-1";
            var laneKey=oneWay?(reverse?"turn:lanes:backward":"turn:lanes:forward"):"turn:lanes";
            if(!tags.TryGetValue(laneKey,out var entries))tags.TryGetValue("turn:lanes",out entries);
            if(!string.IsNullOrWhiteSpace(entries))lanes=entries.Split('|').Length;
            if(lanes==0&&(classification=="motorway"||classification=="trunk")&&NetworkEvidence.Measure(tags,"width") is float width)
                lanes=Numeric.Clamp((int)Math.Round((width-2)/3.5f),1,6);
        }
        return lanes>0?Numeric.Clamp(lanes,1,6):classification.EndsWith("_link",StringComparison.OrdinalIgnoreCase)?1:2;
    }

    private static int LaneCount(IReadOnlyDictionary<string,string> tags,string key) => tags.TryGetValue(key,out var value) &&
        int.TryParse(value.Trim(),NumberStyles.Integer,CultureInfo.InvariantCulture,out var count) && count>0 ? count : 0;

    private static NetworkSides Sides(IReadOnlyDictionary<string,string> tags,string key,bool reverse)
    {
        var result=NetworkSides.None;
        if(tags.TryGetValue(key,out var value))result=value switch { "yes" or "both" or "1" or "true" => NetworkSides.Both, "left" => NetworkSides.Left, "right" => NetworkSides.Right, _ => NetworkSides.None };
        if(IsYes(tags,key+":left"))result|=NetworkSides.Left;
        if(IsYes(tags,key+":right"))result|=NetworkSides.Right;
        return reverse ? ((result&NetworkSides.Left)!=0?NetworkSides.Right:0)|((result&NetworkSides.Right)!=0?NetworkSides.Left:0) : result;
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
        // A motorway_link can be a two-way connector; only motorway itself
        // implies one-way traffic. Explicit oneway always takes precedence.
        return classification.Equals("motorway", StringComparison.OrdinalIgnoreCase) || (tags.TryGetValue("junction", out var junction) && junction == "roundabout");
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
