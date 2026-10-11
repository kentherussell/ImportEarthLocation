using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Xml.Linq;
using System.Threading;
using ImportEarthLocation.Core;

namespace ImportEarthLocation.Processing;

public sealed class OsmDataset
{
    public int MalformedFeaturesSkipped { get; init; }
    public IReadOnlyDictionary<long, OsmNode> Nodes { get; init; } = new Dictionary<long, OsmNode>();
    public IReadOnlyDictionary<long, IReadOnlyDictionary<string, string>> NodeTags { get; init; } = new Dictionary<long, IReadOnlyDictionary<string, string>>();
    public IReadOnlyDictionary<long, OsmWay> Ways { get; init; } = new Dictionary<long, OsmWay>();
    public IReadOnlyList<OsmRelation> Relations { get; init; } = Array.Empty<OsmRelation>();
    public IReadOnlyCollection<long> SupportWayIds { get; init; } = Array.Empty<long>();
    public IReadOnlyDictionary<long,IReadOnlyDictionary<string,string>> RawWayTags { get; init; } = new Dictionary<long,IReadOnlyDictionary<string,string>>();
}

public static class OsmParser
{
    public static OsmDataset Merge(IReadOnlyList<byte[]> documents, ImportLog? log = null, CancellationToken token = default, bool wholeMap = false)
    {
        var nodes = new Dictionary<long, OsmNode>();
        var nodeTags = new Dictionary<long, IReadOnlyDictionary<string, string>>();
        var ways = new Dictionary<long, OsmWay>();
        var relations = new SortedDictionary<long, OsmRelation>();
        var skipped = 0;
        if (documents.Count != (wholeMap ? 1 : 16)) throw new InvalidDataException("Complete whole-map or 16-tile OSM coverage is required.");
        foreach (var bytes in documents)
        {
            token.ThrowIfCancellationRequested();
            using var stream = new MemoryStream(bytes, writable: false);
            var document = XDocument.Load(stream, LoadOptions.None);
            if (document.Root?.Name.LocalName != "osm") throw new InvalidDataException("OSM document root was not <osm>.");
            if (document.Root.Elements("remark").Any()) throw new InvalidDataException("An OSM tile contained a server error/incomplete response.");

            foreach (var element in document.Root.Elements("node"))
            {
                token.ThrowIfCancellationRequested();
                try
                {
                    var id = Long(element, "id");
                    var latitude = Double(element, "lat"); var longitude = Double(element, "lon");
                    if (double.IsNaN(latitude) || double.IsInfinity(latitude) || Math.Abs(latitude) > 90 ||
                        double.IsNaN(longitude) || double.IsInfinity(longitude) || Math.Abs(longitude) > 180)
                        throw new InvalidDataException("Invalid node coordinates.");
                    nodes[id] = new OsmNode(id, latitude, longitude);
                    if (element.Elements("tag").Any())
                    {
                        var tags = element.Elements("tag").Where(t => t.Attribute("k") != null && t.Attribute("v") != null)
                            .GroupBy(t => t.Attribute("k")!.Value,StringComparer.OrdinalIgnoreCase).ToDictionary(g => g.Key, g => g.Last().Attribute("v")!.Value, StringComparer.OrdinalIgnoreCase);
                        if (tags.Count > 0) nodeTags[id] = tags;
                    }
                }
                catch (Exception exception) when (exception is FormatException || exception is OverflowException || exception is InvalidDataException)
                { if (++skipped <= 25) log?.Warn($"Skipped malformed OSM node: {exception.Message}"); }
            }

            foreach (var element in document.Root.Elements("way"))
            {
                token.ThrowIfCancellationRequested();
                try
                {
                var id = Long(element, "id");
                var references = element.Elements("nd").Select(nd => Long(nd, "ref")).ToArray();
                var tags = element.Elements("tag")
                    .Where(tag => tag.Attribute("k") != null && tag.Attribute("v") != null)
                    .GroupBy(tag => tag.Attribute("k")!.Value,StringComparer.OrdinalIgnoreCase)
                    .ToDictionary(group => group.Key, group => group.Last().Attribute("v")!.Value, StringComparer.OrdinalIgnoreCase);
                if (references.Length >= 2) ways[id] = new OsmWay(id, references, tags);
                }
                catch (Exception exception) when (exception is FormatException || exception is OverflowException || exception is InvalidDataException)
                { if (++skipped <= 25) log?.Warn($"Skipped malformed OSM way: {exception.Message}"); }
            }
            foreach (var element in document.Root.Elements("relation"))
            {
                token.ThrowIfCancellationRequested();
                try
                {
                    var ordered = element.Elements("member")
                        .Select(member => new OsmMember((string?)member.Attribute("type")??"",Long(member,"ref"),(string?)member.Attribute("role")??"")).ToArray();
                    var members=ordered.Where(m=>m.Type=="way").Select(m=>(m.Id,m.Role=="inner")).ToArray();
                    var tags = element.Elements("tag").Where(tag => tag.Attribute("k") != null && tag.Attribute("v") != null)
                        .GroupBy(tag => tag.Attribute("k")!.Value,StringComparer.OrdinalIgnoreCase).ToDictionary(group => group.Key, group => group.Last().Attribute("v")!.Value, StringComparer.OrdinalIgnoreCase);
                    relations[Long(element, "id")] = new OsmRelation(Long(element, "id"), members, tags) { OrderedMembers=ordered };
                }
                catch (Exception exception) when (exception is FormatException || exception is OverflowException || exception is InvalidDataException)
                { if (++skipped <= 25) log?.Warn($"Skipped malformed OSM relation: {exception.Message}"); }
            }
        }
        log?.Count("Malformed OSM features skipped", skipped);
        return new OsmDataset { Nodes = nodes, NodeTags = nodeTags, Ways = ways, Relations = relations.Values.ToArray(), MalformedFeaturesSkipped = skipped };
    }

    private static long Long(XElement element, string name) =>
        long.Parse(element.Attribute(name)?.Value ?? throw new InvalidDataException($"Missing {name}."), CultureInfo.InvariantCulture);

    private static double Double(XElement element, string name) =>
        double.Parse(element.Attribute(name)?.Value ?? throw new InvalidDataException($"Missing {name}."), CultureInfo.InvariantCulture);
}
