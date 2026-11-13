using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using ImportEarthLocation.Geo;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

/// <summary>One normalized evidence snapshot, independent of mutable designed geometry.</summary>
public static class NetworkEvidence
{
    private static readonly HashSet<string> SemanticKeys = new(StringComparer.OrdinalIgnoreCase)
    {
        "highway",
        "railway",
        "oneway",
        "bridge",
        "tunnel",
        "covered",
        "location",
        "junction",
        "cutting",
        "retaining_wall",
        "motorroad",
        "expressway",
        "access",
        "service",
        "noexit",
        "leaf_type",
        "leaf_cycle",
        "type",
        "restriction"
        };

    public static OsmDataset Normalize(OsmDataset source, ImportRequest request, ImportLog log, CancellationToken token)
    {
        IReadOnlyDictionary<string, string> Tags(IReadOnlyDictionary<string, string> raw)
            => raw
                .GroupBy(entry => entry.Key.Trim(), StringComparer.OrdinalIgnoreCase)
                .ToDictionary(
                    group => group.Key,
                    group => SemanticKeys.Contains(group.Key)
                        ? group.Last().Value.Trim().ToLowerInvariant()
                        : group.Last().Value.Trim(),
                    StringComparer.OrdinalIgnoreCase);

        var ways = new Dictionary<long, OsmWay>();

        foreach (var way in source.Ways.Values)
        {
            token.ThrowIfCancellationRequested();
            ways[way.Id] = way with { Tags = Tags(way.Tags) };
        }

        var selected =
            new HashSet<long>(
                ways.Values
                    .Where(
                        way => way.Tags.TryGetValue("highway", out var text)
                            && NetworkSelection.Includes(request.RoadLevel, text)
                            || request.ImportRail
                                && way.Tags.TryGetValue("railway", out var otherText)
                                && otherText == "rail")
                    .Select(way => way.Id));

        // Support connectors are a road-selection feature, never rail dependency closure.
        var selectedNodes =
            new HashSet<long>(
                selected
                    .Where(id => ways[id].Tags.ContainsKey("highway") && request.RoadLevel != RoadImportLevel.None)
                    .SelectMany(id => ways[id].NodeIds));

        var support = new HashSet<long>();
        var relations =
            source.Relations
                .Select(
                    relation => relation with
                    {
                        Tags = Tags(relation.Tags),
                        OrderedMembers =
                            relation.OrderedMembers
                                .Select(
                                    member => member with
                                    {
                                        Type = member.Type.Trim().ToLowerInvariant(),
                                        Role = member.Role.Trim().ToLowerInvariant()
                                    })
                                .ToArray()
                    })
                .ToArray();

        foreach (var relation in relations)
        {
            if (request.RoadLevel == RoadImportLevel.None)
                break;

            var members = relation.OrderedMembers;

            if (!members.Any(member => member.Type == "way" && member.Role == "from" && selected.Contains(member.Id))
                || !members.Any(member => member.Type == "way" && member.Role == "to" && selected.Contains(member.Id)))
                continue;

            foreach (var via in members.Where(member => member.Type == "way" && member.Role == "via"))
                if (ways.TryGetValue(via.Id, out var viaWay)
                    && (!viaWay.Tags.TryGetValue("highway", out var viaClass)
                        || !NetworkSelection.IsExcludedStreetLink(viaClass)))
                    support.Add(via.Id);
        }

        foreach (var way in ways.Values.Where(
            otherWay => request.RoadLevel != RoadImportLevel.None
                && !selected.Contains(otherWay.Id)
                && otherWay.Tags.ContainsKey("highway")))
        {
            token.ThrowIfCancellationRequested();

            if (NetworkSelection.IsExcludedStreetLink(way.Tags["highway"]))
                continue;

            var joins = way.NodeIds.Count(selectedNodes.Contains);
            var ring = way.Tags.TryGetValue("junction", out var secondText) && secondText == "roundabout";

            if (joins == 0 || !ring && joins < 2)
                continue;

            var length = 0f;

            for (var i = 1; i < way.NodeIds.Length; i++)
                if (source.Nodes.TryGetValue(way.NodeIds[i - 1], out var firstNode)
                    && source.Nodes.TryGetValue(way.NodeIds[i], out var secondNode))
                    length +=
                        math.distance(
                            GeoMath.ToLocalMeters(
                                firstNode.Latitude,
                                firstNode.Longitude,
                                request.Latitude,
                                request.Longitude).xz,
                            GeoMath.ToLocalMeters(
                                secondNode.Latitude,
                                secondNode.Longitude,
                                request.Latitude,
                                request.Longitude).xz);

            if (ring || length <= 500)
                support.Add(way.Id);
        }

        var required = selected.Concat(support).Distinct().ToArray();
        // Complete split roundabout components, not just the arc touching a selected
        // road. The downloaded query is bounded; this walks only its ring subgraph.
        var ringNodes =
            new HashSet<long>(
                selected
                    .Concat(support)
                    .Where(id => ways[id].Tags.TryGetValue("junction", out var thirdText) && thirdText == "roundabout")
                    .SelectMany(id => ways[id].NodeIds));

        var pending =
            ways.Values
                .Where(
                    way => request.RoadLevel != RoadImportLevel.None
                        && !selected.Contains(way.Id)
                        && !support.Contains(way.Id)
                        && (!way.Tags.TryGetValue("highway", out var fourthText)
                            || !NetworkSelection.IsExcludedStreetLink(fourthText))
                        && way.Tags.TryGetValue("junction", out var fifthText)
                        && fifthText == "roundabout")
                .ToList();

        bool changed;

        do
        {
            changed = false;

            for (var i = pending.Count - 1; i >= 0; i--)
            {
                token.ThrowIfCancellationRequested();
                var way = pending[i];

                if (!way.NodeIds.Any(ringNodes.Contains))
                    continue;

                support.Add(way.Id);
                ringNodes.UnionWith(way.NodeIds);
                pending.RemoveAt(i);
                changed = true;
            }
        }
        while (changed);

        required = selected.Concat(support).Distinct().ToArray();
        var missing =
            required
                .SelectMany(id => ways[id].NodeIds)
                .Where(id => !source.Nodes.ContainsKey(id))
                .Distinct()
                .Take(20)
                .ToArray();

        if (missing.Length > 0)
            log.Warn(
                $"Network dependency recovery left missing node references ({string.Join(",", missing)}). Only segments with unavailable endpoints will be omitted; remaining networks retained.");

        log.Count("Junction support ways admitted", support.Count);
        log.Diagnostic(
            $"NETWORK EVIDENCE selectedWays={selected.Count}; supportWays={support.Count}; taggedNodes={source.NodeTags.Count}; restrictions={source.Relations.Count(relation => relation.Tags.TryGetValue("type", out var t) && t == "restriction")}; connectivity={source.Relations.Count(relation => relation.Tags.TryGetValue("type", out var t) && t == "connectivity")}; dependencyClosure=true; interpretationVersion=3; newEndpoints=0");

        foreach (var relation in source.Relations
            .Where(
                otherRelation => otherRelation.Tags.TryGetValue("type", out var secondText)
                    && (secondText == "restriction" || secondText == "connectivity"))
            .Take(40))
            log.Diagnostic(
                $"MOVEMENT EVIDENCE relation={relation.Id}; members={string.Join("|", relation.OrderedMembers.Select(member => $"{member.Type}:{member.Id}:{member.Role}"))}; policy={string.Join("|", relation.Tags.Where(entry => entry.Key.StartsWith("restriction") || entry.Key == "connectivity" || entry.Key == "except").Select(entry => entry.Key + "=" + entry.Value))}; nativeEnforcementRequiresReadback=true");

        return new OsmDataset
        {
            Nodes = source.Nodes,
            NodeTags = source.NodeTags.ToDictionary(entry => entry.Key, entry => Tags(entry.Value)),
            Ways = ways,
            Relations = relations,
            SupportWayIds = support,
            RawWayTags = source.Ways.ToDictionary(entry => entry.Key, entry => entry.Value.Tags),
            MalformedFeaturesSkipped = source.MalformedFeaturesSkipped
        };
    }

    public static bool NodeIs(OsmDataset data, long id, string key, string value)
        => data.NodeTags.TryGetValue(id, out var tags) && tags.TryGetValue(key, out var actual) && actual == value;

    public static float? Measure(IReadOnlyDictionary<string, string> tags, string key, bool speed = false)
    {
        if (!tags.TryGetValue(key, out var text))
            return null;

        text = text.Trim().ToLowerInvariant();
        var factor = 1f;

        if (speed && text.EndsWith("mph"))
        {
            factor = 1.609344f;
            text = text.Substring(0, text.Length - 3).Trim();
        }
        else if (!speed && (text.EndsWith("ft") || text.EndsWith("feet")))
        {
            factor = .3048f;
            text = text.Substring(0, text.Length - (text.EndsWith("ft") ? 2 : 4)).Trim();
        }
        else
            text = text.Replace(speed ? "km/h" : "m", "").Trim();

        return float.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out var number)
            && TerrainConditioner.Finite(number)
            && number > 0
            && number * factor <= (speed ? 250 : 100)
            ? number * factor
            : null;
    }
}
