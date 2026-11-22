using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Threading;
using System.Xml;
using ImportEarthLocation.Core;

namespace ImportEarthLocation.Processing;

public sealed class OsmDataset
{
    public int MalformedFeaturesSkipped { get; init; }
    public IReadOnlyDictionary<long, OsmNode> Nodes { get; init; } = new Dictionary<long, OsmNode>();

    public IReadOnlyDictionary<long, IReadOnlyDictionary<string, string>> NodeTags { get; init; } =
        new Dictionary<long, IReadOnlyDictionary<string, string>>();

    public IReadOnlyDictionary<long, OsmWay> Ways { get; init; } = new Dictionary<long, OsmWay>();
    public IReadOnlyList<OsmRelation> Relations { get; init; } = Array.Empty<OsmRelation>();
    public IReadOnlyCollection<long> SupportWayIds { get; init; } = Array.Empty<long>();

    public IReadOnlyDictionary<long, IReadOnlyDictionary<string, string>> RawWayTags { get; init; } =
        new Dictionary<long, IReadOnlyDictionary<string, string>>();
}

public static class OsmParser
{
    // The child elements of one node, way or relation that the parser reads, as text.
    private sealed class Children
    {
        public readonly List<(string? Key, string? Value)> Tags = new();
        public readonly List<string?> NodeReferences = new();
        public readonly List<(string? Type, string? Reference, string? Role)> Members = new();

        public void Clear()
        {
            Tags.Clear();
            NodeReferences.Clear();
            Members.Clear();
        }
    }

    // Each tile is read once, front to back. Loading it whole first built every element and
    // attribute of up to 70 MB of text in memory and then walked the result three times; a
    // city's first import parses all of its tiles (report 119). The dataset is the same:
    // elements and attributes without a namespace, each kind in document order, a repeated id
    // keeping its first place and its last value. Whitespace is ignored and a document type is
    // parsed, as when the document was loaded whole.
    public static OsmDataset Merge(
        IReadOnlyList<byte[]> documents,
        ImportLog? log = null,
        CancellationToken token = default,
        bool wholeMap = false)
    {
        var nodes = new Dictionary<long, OsmNode>();
        var nodeTags = new Dictionary<long, IReadOnlyDictionary<string, string>>();
        var ways = new Dictionary<long, OsmWay>();
        var relations = new SortedDictionary<long, OsmRelation>();
        var skipped = 0;

        if (documents.Count != (wholeMap ? 1 : 16))
            throw new InvalidDataException("Complete whole-map or 16-tile OSM coverage is required.");

        var children = new Children();

        foreach (var bytes in documents)
        {
            token.ThrowIfCancellationRequested();
            using var stream = new MemoryStream(bytes, writable: false);
            using var reader =
                XmlReader.Create(
                    stream,
                    new XmlReaderSettings
                    {
                        IgnoreWhitespace = true,
                        DtdProcessing = DtdProcessing.Parse,
                        MaxCharactersFromEntities = 10_000_000
                    });

            var rootSeen = false;

            while (reader.Read())
            {
                if (reader.NodeType != XmlNodeType.Element)
                    continue;

                if (!rootSeen)
                {
                    if (reader.LocalName != "osm")
                        throw new InvalidDataException("OSM document root was not <osm>.");

                    rootSeen = true;
                    continue;
                }

                // Only the root's own children are features; ReadChildren consumes theirs.
                if (reader.Depth != 1 || reader.NamespaceURI.Length != 0)
                    continue;

                switch (reader.LocalName)
                {
                    case "remark":
                        throw new InvalidDataException(
                            "An OSM tile contained a server error/incomplete response.");

                    case "node":
                    {
                        token.ThrowIfCancellationRequested();
                        var idText = reader.GetAttribute("id");
                        var latitudeText = reader.GetAttribute("lat");
                        var longitudeText = reader.GetAttribute("lon");
                        ReadChildren(reader, children);

                        try
                        {
                            var id = Long(idText, "id");
                            var latitude = Double(latitudeText, "lat");
                            var longitude = Double(longitudeText, "lon");

                            if (double.IsNaN(latitude) || double.IsInfinity(latitude) || Math.Abs(latitude) > 90
                                || double.IsNaN(longitude) || double.IsInfinity(longitude) || Math.Abs(longitude) > 180)
                                throw new InvalidDataException("Invalid node coordinates.");

                            nodes[id] = new OsmNode(id, latitude, longitude);

                            if (children.Tags.Count > 0)
                            {
                                var tags = Tags(children);

                                if (tags.Count > 0)
                                    nodeTags[id] = tags;
                            }
                        }
                        catch (Exception exception) when (exception is FormatException
                            || exception is OverflowException
                            || exception is InvalidDataException)
                        {
                            if (++skipped <= 25)
                                log?.Warn($"Skipped malformed OSM node: {exception.Message}");
                        }

                        break;
                    }

                    case "way":
                    {
                        token.ThrowIfCancellationRequested();
                        var idText = reader.GetAttribute("id");
                        ReadChildren(reader, children);

                        try
                        {
                            var id = Long(idText, "id");
                            var references = new long[children.NodeReferences.Count];

                            for (var i = 0; i < references.Length; i++)
                                references[i] = Long(children.NodeReferences[i], "ref");

                            var tags = Tags(children);

                            if (references.Length >= 2)
                                ways[id] = new OsmWay(id, references, tags);
                        }
                        catch (Exception exception) when (exception is FormatException
                            || exception is OverflowException
                            || exception is InvalidDataException)
                        {
                            if (++skipped <= 25)
                                log?.Warn($"Skipped malformed OSM way: {exception.Message}");
                        }

                        break;
                    }

                    case "relation":
                    {
                        token.ThrowIfCancellationRequested();
                        var idText = reader.GetAttribute("id");
                        ReadChildren(reader, children);

                        try
                        {
                            var ordered = new OsmMember[children.Members.Count];
                            var members = new List<(long, bool)>();

                            for (var i = 0; i < ordered.Length; i++)
                            {
                                var member = children.Members[i];
                                ordered[i] =
                                    new OsmMember(
                                        member.Type ?? "",
                                        Long(member.Reference, "ref"),
                                        member.Role ?? "");

                                if (ordered[i].Type == "way")
                                    members.Add((ordered[i].Id, ordered[i].Role == "inner"));
                            }

                            var tags = Tags(children);
                            var id = Long(idText, "id");
                            relations[id] = new OsmRelation(id, members.ToArray(), tags) { OrderedMembers = ordered };
                        }
                        catch (Exception exception) when (exception is FormatException
                            || exception is OverflowException
                            || exception is InvalidDataException)
                        {
                            if (++skipped <= 25)
                                log?.Warn($"Skipped malformed OSM relation: {exception.Message}");
                        }

                        break;
                    }
                }
            }
        }

        log?.Count("Malformed OSM features skipped", skipped);

        return new OsmDataset
        {
            Nodes = nodes,
            NodeTags = nodeTags,
            Ways = ways,
            Relations = relations.Values.ToArray(),
            MalformedFeaturesSkipped = skipped
        };
    }

    // Reads the direct children of the element the reader stands on and leaves the reader on
    // that element's end, so the caller's next read moves past it.
    private static void ReadChildren(XmlReader reader, Children children)
    {
        children.Clear();

        if (reader.IsEmptyElement)
            return;

        var depth = reader.Depth;

        while (reader.Read())
        {
            if (reader.NodeType == XmlNodeType.EndElement && reader.Depth == depth)
                return;

            if (reader.NodeType != XmlNodeType.Element
                || reader.Depth != depth + 1
                || reader.NamespaceURI.Length != 0)
                continue;

            switch (reader.LocalName)
            {
                case "tag":
                    children.Tags.Add((reader.GetAttribute("k"), reader.GetAttribute("v")));
                    break;

                case "nd":
                    children.NodeReferences.Add(reader.GetAttribute("ref"));
                    break;

                case "member":
                    children.Members.Add(
                        (reader.GetAttribute("type"), reader.GetAttribute("ref"), reader.GetAttribute("role")));

                    break;
            }
        }
    }

    // Keys compare without case. A repeated key keeps the spelling and the place of its first
    // tag and the value of its last.
    private static Dictionary<string, string> Tags(Children children)
    {
        var tags = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);

        foreach (var (key, value) in children.Tags)
            if (key != null && value != null)
                tags[key] = value;

        return tags;
    }

    private static long Long(string? value, string name) =>
        long.Parse(value ?? throw new InvalidDataException($"Missing {name}."), CultureInfo.InvariantCulture);

    private static double Double(string? value, string name) =>
        double.Parse(value ?? throw new InvalidDataException($"Missing {name}."), CultureInfo.InvariantCulture);
}
