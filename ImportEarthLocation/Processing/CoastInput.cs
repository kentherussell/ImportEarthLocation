using System;
using System.Collections.Generic;
using System.Linq;
using ImportEarthLocation.Core;

namespace ImportEarthLocation.Processing;

// Saves what the coast model (CoastalTerrain.Shape) reads, so it can be replayed offline
// (network-tests --coast-replay). Written only when the map has a coastline:
//   coast-input.json             grid size, tile zoom and origin, spacing, and the height
//                                encoding below
//   coast-source-heights.u16     the source grid as Shape receives it, row 0 north (tile
//                                rows), each height (metres) = Low + code * Step; Step is
//                                the grid's own range over 65,535, under a centimetre for
//                                most maps
//   coast-geography.ndjson.gz    coastline ways, water and reservoir ways and relations,
//                                the relations' member ways, and every node they use
public static class CoastInput
{
    public const string HeightsFile = "coast-source-heights.u16";
    public const string GeographyFile = "coast-geography.ndjson.gz";
    private const int MaximumBytes = 64 * 1024 * 1024;

    public static void Capture(
        ImportLog log,
        float[] heights,
        int width,
        int height,
        int zoom,
        int originX,
        int originY,
        float spacing,
        OsmDataset? geography)
    {
        if (geography == null || !geography.Ways.Values.Any(CoastalTerrain.IsCoastline))
            return;

        var low = float.PositiveInfinity;
        var high = float.NegativeInfinity;

        foreach (var value in heights)
        {
            if (!TerrainConditioner.Finite(value))
                continue;

            low = Math.Min(low, value);
            high = Math.Max(high, value);
        }

        if (!(high >= low))
            return;

        var step = Math.Max(1e-4f, (high - low) / 65535);
        var bytes = new byte[heights.Length * 2];

        for (var i = 0; i < heights.Length; i++)
        {
            var code = TerrainConditioner.Finite(heights[i])
                ? (ushort)Math.Min(65535, Math.Max(0, Math.Round((heights[i] - low) / step)))
                : (ushort)0;
            bytes[2 * i] = (byte)code;
            bytes[2 * i + 1] = (byte)(code >> 8);
        }

        log.DiagnosticBytes(HeightsFile, bytes, MaximumBytes);
        log.DiagnosticCompressedRecords(GeographyFile, Records(geography));
        log.DiagnosticJson(
            "coast-input.json",
            new
            {
                width,
                height,
                zoom,
                originX,
                originY,
                spacing,
                low,
                step,
                heightsFile = HeightsFile,
                geographyFile = GeographyFile,
            });
    }

    private static IEnumerable<object> Records(OsmDataset geography)
    {
        var relations = geography.Relations.Where(relation => CoastalTerrain.IsWaterArea(relation.Tags)).ToArray();
        var memberWays = new HashSet<long>(relations.SelectMany(relation => relation.Members.Select(member => member.wayId)));
        var ways =
            geography.Ways.Values
                .Where(way => CoastalTerrain.IsCoastline(way) || CoastalTerrain.IsWaterArea(way.Tags) || memberWays.Contains(way.Id))
                .ToArray();
        var nodes = new HashSet<long>(ways.SelectMany(way => way.NodeIds));

        foreach (var id in nodes)
            if (geography.Nodes.TryGetValue(id, out var node))
                yield return new { type = "node", id, lat = node.Latitude, lon = node.Longitude };

        foreach (var way in ways)
            yield return new { type = "way", id = way.Id, nodes = way.NodeIds, tags = way.Tags };

        foreach (var relation in relations)
            yield return new
            {
                type = "relation",
                id = relation.Id,
                members = relation.Members.Select(member => new { way = member.wayId, member.inner }).ToArray(),
                tags = relation.Tags,
            };
    }
}
