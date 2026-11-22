using System.Globalization;
using System.Text;
using System.Xml.Linq;
using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;

// The streaming OSM parser against the parser it replaced, which loaded each document whole
// (report 119). Both must give the same dataset, or fail with the same kind of exception, on
// documents built to reach every rule of the old one, and on real tiles when a folder of
// cached tiles is given.
static class OsmParserEquivalence
{
    public static void Run()
    {
        const string Head = "<?xml version=\"1.0\" encoding=\"UTF-8\"?><osm version=\"0.6\">";

        var ordinary =
            Head
                + "<note>ignored</note><meta osm_base=\"x\"/>"
                + "<node id=\"1\" lat=\"43.1\" lon=\"5.2\"/>"
                + "<node id=\"2\" lat=\"43.2\" lon=\"5.3\"><tag k=\"natural\" v=\"tree\"/></node>"
                + "<node id=\"3\" lat=\"43.3\" lon=\"5.4\"><tag k=\"Name\" v=\"first\"/><tag k=\"name\" v=\"last\"/>"
                + "<tag k=\"ref\" v=\"A &amp; B\"/><tag k=\"empty\" v=\"\"/></node>"
                + "<way id=\"10\"><nd ref=\"1\"/><nd ref=\"2\"/><nd ref=\"3\"/><tag k=\"highway\" v=\"primary\"/>"
                + "<tag k=\"HIGHWAY\" v=\"secondary\"/></way>"
                + "<way id=\"11\"><nd ref=\"1\"/></way>"
                + "<way id=\"12\"><nd ref=\"2\"/><tag k=\"oneway\" v=\"yes\"/><nd ref=\"3\"/></way>"
                + "<relation id=\"20\"><member type=\"way\" ref=\"10\" role=\"outer\"/>"
                + "<member type=\"way\" ref=\"12\" role=\"inner\"/><member type=\"node\" ref=\"1\" role=\"via\"/>"
                + "<member ref=\"11\"/><tag k=\"type\" v=\"multipolygon\"/></relation>"
                + "</osm>";

        var malformed =
            Head
                + "<node id=\"x\" lat=\"1\" lon=\"1\"><tag k=\"a\" v=\"b\"/></node>"
                + "<node lat=\"1\" lon=\"1\"/>"
                + "<node id=\"4\" lat=\"91\" lon=\"1\"/>"
                + "<node id=\"5\" lat=\"1\" lon=\"NaN\"/>"
                + "<node id=\"6\" lat=\"1\"/>"
                + "<node id=\"99999999999999999999\" lat=\"1\" lon=\"1\"/>"
                + "<node id=\"7\" lat=\" 1.5 \" lon=\"1e1\"><tag k=\"a\"/><tag v=\"b\"/><other><tag k=\"deep\" v=\"x\"/></other></node>"
                + "<node id=\"8\" lat=\"2\" lon=\"2\"><tag k=\"a\"/></node>"
                + "<way><nd ref=\"7\"/><nd ref=\"8\"/></way>"
                + "<way id=\"30\"><nd ref=\"7\"/><nd/><nd ref=\"8\"/></way>"
                + "<way id=\"31\"><nd ref=\"7\"/><nd ref=\"q\"/></way>"
                + "<way id=\"bad\"><nd ref=\"q\"/></way>"
                + "<way id=\"32\"><nd ref=\"7\"/><nd ref=\"8\"/><extra><nd ref=\"6\"/></extra></way>"
                + "<relation id=\"40\"><member type=\"way\" ref=\"oops\"/></relation>"
                + "<relation><member type=\"way\" ref=\"32\"/></relation>"
                + "<relation id=\"zz\"><member type=\"way\"/></relation>"
                + "<relation id=\"41\"/>"
                + "</osm>";

        // Kinds out of the usual order, an id repeated within and across documents, and
        // elements in a namespace, which neither parser reads.
        var interleaved =
            Head
                + "<way id=\"10\"><nd ref=\"2\"/><nd ref=\"1\"/></way>"
                + "<relation id=\"20\"><member type=\"way\" ref=\"10\" role=\"\"/></relation>"
                + "<node id=\"2\" lat=\"1\" lon=\"1\"><tag k=\"second\" v=\"copy\"/></node>"
                + "<node id=\"50\" lat=\"3\" lon=\"3\"/>"
                + "<node id=\"2\" lat=\"2\" lon=\"2\"/>"
                + "<x:node xmlns:x=\"urn:x\" id=\"60\" lat=\"1\" lon=\"1\"/>"
                + "<node xmlns=\"urn:y\" id=\"61\" lat=\"1\" lon=\"1\"/>"
                + "<node id=\"62\" x:id=\"63\" xmlns:x=\"urn:x\" lat=\"4\" lon=\"4\"><x:tag k=\"a\" v=\"b\"/></node>"
                + "</osm>";

        var documents = new[]
        {
            ("ordinary", ordinary),
            ("malformed features", malformed),
            ("interleaved and repeated", interleaved),
            ("empty", Head + "</osm>"),
            ("self-closed root", "<osm/>"),
            ("remark", Head + "<node id=\"1\" lat=\"1\" lon=\"1\"/><remark>runtime error</remark></osm>"),
            ("nested remark", Head + "<note><remark>fine</remark></note><node id=\"1\" lat=\"1\" lon=\"1\"/></osm>"),
            ("wrong root", "<html><body>busy</body></html>"),
            ("truncated", ordinary.Substring(0, ordinary.Length / 2)),
            ("trailing text", ordinary + "x"),
            ("not xml", "rate limited"),
            ("document type", "<!DOCTYPE osm [<!ENTITY t \"tree\">]><osm><node id=\"1\" lat=\"1\" lon=\"1\">"
                + "<tag k=\"natural\" v=\"&t;\"/></node></osm>")
        };

        foreach (var (name, text) in documents)
            Compare(name, new[] { Encoding.UTF8.GetBytes(text) }, wholeMap: true);

        // Sixteen tiles, as an import merges them, in two different orders.
        var sixteen =
            Enumerable
                .Range(0, 16)
                .Select(i => Encoding.UTF8.GetBytes(i % 3 == 0 ? ordinary : i % 3 == 1 ? interleaved : malformed))
                .ToArray();

        Compare("sixteen tiles", sixteen, wholeMap: false);
        Compare("sixteen tiles reversed", sixteen.Reverse().ToArray(), wholeMap: false);

        // More than 25 malformed features: the count goes on, the warnings stop.
        var many =
            Head
                + string.Concat(Enumerable.Range(0, 40).Select(i => "<node id=\"n" + i + "\" lat=\"1\" lon=\"1\"/>"))
                + "</osm>";

        Compare("many malformed", new[] { Encoding.UTF8.GetBytes(many) }, wholeMap: true);
        Console.WriteLine("PASS: streaming OSM parser equals the whole-document parser on built documents.");
    }

    // Real tiles from the tile cache, largest first, in groups of sixteen.
    public static void RunOnTiles(string folder, int groups)
    {
        var files =
            new DirectoryInfo(folder)
                .GetFiles("*.osm")
                .OrderByDescending(file => file.Length)
                .Take(16 * groups)
                .ToArray();

        double oldSeconds = 0, newSeconds = 0;
        long bytes = 0;

        for (var group = 0; group + 16 <= files.Length; group += 16)
        {
            var documents = files.Skip(group).Take(16).Select(file => File.ReadAllBytes(file.FullName)).ToArray();
            bytes += documents.Sum(document => (long)document.Length);
            var (first, second) = Compare("tiles from " + files[group].Name, documents, wholeMap: false);
            oldSeconds += first;
            newSeconds += second;
        }

        Console.WriteLine(
            $"PASS: {files.Length / 16 * 16} real tiles, {bytes / 1e6:0} MB; whole-document {oldSeconds:0.0} s, streaming {newSeconds:0.0} s.");
    }

    private static (double Old, double New) Compare(string name, byte[][] documents, bool wholeMap)
    {
        var oldLog = new ImportLog();
        var newLog = new ImportLog();
        OsmDataset? expected = null, actual = null;
        Exception? expectedError = null, actualError = null;
        var clock = System.Diagnostics.Stopwatch.StartNew();

        try
        {
            expected = Reference(documents, oldLog, wholeMap);
        }
        catch (Exception error)
        {
            expectedError = error;
        }

        var oldSeconds = clock.Elapsed.TotalSeconds;
        clock.Restart();

        try
        {
            actual = OsmParser.Merge(documents, newLog, default, wholeMap);
        }
        catch (Exception error)
        {
            actualError = error;
        }

        var newSeconds = clock.Elapsed.TotalSeconds;

        Check(
            expectedError?.GetType() == actualError?.GetType(),
            $"{name}: whole-document parser {Describe(expectedError)}, streaming parser {Describe(actualError)}");

        if (expected == null || actual == null)
            return (oldSeconds, newSeconds);

        Check(expected.MalformedFeaturesSkipped == actual.MalformedFeaturesSkipped, name + ": skipped count differs");
        Check(
            oldLog.Metric("Malformed OSM features skipped") == newLog.Metric("Malformed OSM features skipped"),
            name + ": skipped metric differs");

        Check(expected.Nodes.SequenceEqual(actual.Nodes), name + ": nodes differ");
        Check(expected.NodeTags.Keys.SequenceEqual(actual.NodeTags.Keys), name + ": tagged nodes differ");

        foreach (var id in expected.NodeTags.Keys)
            Check(SameTags(expected.NodeTags[id], actual.NodeTags[id]), $"{name}: tags of node {id} differ");

        Check(expected.Ways.Keys.SequenceEqual(actual.Ways.Keys), name + ": ways differ");

        foreach (var id in expected.Ways.Keys)
        {
            Check(expected.Ways[id].Id == actual.Ways[id].Id, $"{name}: way {id} id differs");
            Check(expected.Ways[id].NodeIds.SequenceEqual(actual.Ways[id].NodeIds), $"{name}: nodes of way {id} differ");
            Check(SameTags(expected.Ways[id].Tags, actual.Ways[id].Tags), $"{name}: tags of way {id} differ");
        }

        Check(expected.Relations.Count == actual.Relations.Count, name + ": relation count differs");

        for (var i = 0; i < expected.Relations.Count; i++)
        {
            var first = expected.Relations[i];
            var second = actual.Relations[i];
            Check(first.Id == second.Id, $"{name}: relation {i} id differs");
            Check(first.Members.SequenceEqual(second.Members), $"{name}: members of relation {first.Id} differ");
            Check(
                first.OrderedMembers.SequenceEqual(second.OrderedMembers),
                $"{name}: ordered members of relation {first.Id} differ");

            Check(SameTags(first.Tags, second.Tags), $"{name}: tags of relation {first.Id} differ");
        }

        return (oldSeconds, newSeconds);
    }

    private static string Describe(Exception? error) => error == null ? "succeeded" : "threw " + error.GetType().Name;

    // The same keys in the same order with the same spelling and values, and the same answer
    // to a lookup in another case.
    private static bool SameTags(IReadOnlyDictionary<string, string> first, IReadOnlyDictionary<string, string> second)
        => first.Count == second.Count
            && first.Zip(second, (a, b) => a.Key == b.Key && a.Value == b.Value).All(same => same)
            && first.Keys.All(
                key => second.TryGetValue(key.ToUpperInvariant(), out var value) && value == first[key]);

    // The parser as it was before report 119, kept as the reference.
    private static OsmDataset Reference(IReadOnlyList<byte[]> documents, ImportLog log, bool wholeMap)
    {
        var nodes = new Dictionary<long, OsmNode>();
        var nodeTags = new Dictionary<long, IReadOnlyDictionary<string, string>>();
        var ways = new Dictionary<long, OsmWay>();
        var relations = new SortedDictionary<long, OsmRelation>();
        var skipped = 0;

        if (documents.Count != (wholeMap ? 1 : 16))
            throw new InvalidDataException("Complete whole-map or 16-tile OSM coverage is required.");

        foreach (var bytes in documents)
        {
            using var stream = new MemoryStream(bytes, writable: false);
            var document = XDocument.Load(stream, LoadOptions.None);

            if (document.Root?.Name.LocalName != "osm")
                throw new InvalidDataException("OSM document root was not <osm>.");

            if (document.Root.Elements("remark").Any())
                throw new InvalidDataException("An OSM tile contained a server error/incomplete response.");

            foreach (var element in document.Root.Elements("node"))
            {
                try
                {
                    var id = Long(element, "id");
                    var latitude = Double(element, "lat");
                    var longitude = Double(element, "lon");

                    if (double.IsNaN(latitude) || double.IsInfinity(latitude) || Math.Abs(latitude) > 90
                        || double.IsNaN(longitude) || double.IsInfinity(longitude) || Math.Abs(longitude) > 180)
                        throw new InvalidDataException("Invalid node coordinates.");

                    nodes[id] = new OsmNode(id, latitude, longitude);

                    if (element.Elements("tag").Any())
                    {
                        var tags = Tags(element);

                        if (tags.Count > 0)
                            nodeTags[id] = tags;
                    }
                }
                catch (Exception exception) when (exception is FormatException
                    || exception is OverflowException
                    || exception is InvalidDataException)
                {
                    skipped++;
                }
            }

            foreach (var element in document.Root.Elements("way"))
            {
                try
                {
                    var id = Long(element, "id");
                    var references = element.Elements("nd").Select(nd => Long(nd, "ref")).ToArray();
                    var tags = Tags(element);

                    if (references.Length >= 2)
                        ways[id] = new OsmWay(id, references, tags);
                }
                catch (Exception exception) when (exception is FormatException
                    || exception is OverflowException
                    || exception is InvalidDataException)
                {
                    skipped++;
                }
            }

            foreach (var element in document.Root.Elements("relation"))
            {
                try
                {
                    var ordered = element.Elements("member")
                        .Select(
                            member => new OsmMember(
                                (string?)member.Attribute("type") ?? "",
                                Long(member, "ref"),
                                (string?)member.Attribute("role") ?? ""))
                        .ToArray();

                    var members =
                        ordered
                            .Where(member => member.Type == "way")
                            .Select(member => (member.Id, member.Role == "inner"))
                            .ToArray();

                    var tags = Tags(element);

                    relations[Long(element, "id")] =
                        new OsmRelation(Long(element, "id"), members, tags) { OrderedMembers = ordered };
                }
                catch (Exception exception) when (exception is FormatException
                    || exception is OverflowException
                    || exception is InvalidDataException)
                {
                    skipped++;
                }
            }
        }

        log.Count("Malformed OSM features skipped", skipped);

        return new OsmDataset
        {
            Nodes = nodes,
            NodeTags = nodeTags,
            Ways = ways,
            Relations = relations.Values.ToArray(),
            MalformedFeaturesSkipped = skipped
        };
    }

    private static Dictionary<string, string> Tags(XElement element)
        => element
            .Elements("tag")
            .Where(tag => tag.Attribute("k") != null && tag.Attribute("v") != null)
            .GroupBy(tag => tag.Attribute("k")!.Value, StringComparer.OrdinalIgnoreCase)
            .ToDictionary(
                group => group.Key,
                group => group.Last().Attribute("v")!.Value,
                StringComparer.OrdinalIgnoreCase);

    private static long Long(XElement element, string name) =>
        long.Parse(
            element.Attribute(name)?.Value ?? throw new InvalidDataException($"Missing {name}."),
            CultureInfo.InvariantCulture);

    private static double Double(XElement element, string name) =>
        double.Parse(
            element.Attribute(name)?.Value ?? throw new InvalidDataException($"Missing {name}."),
            CultureInfo.InvariantCulture);
}
