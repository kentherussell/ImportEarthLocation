using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

// Slip lanes at regular junctions are removed only when every driving movement they made
// survives without them (report 121 §12.1): in its legal direction, through turns the OSM
// restrictions allow, and on the network left after all removals.
static class RegularLinkTests
{
    // Road X runs west-east through junction J (node 1) at z = 0; road Y runs south-north
    // through J at x = 0. The slip lane joins A on X (node 10) to B on Y (node 20).
    const long RoadX = 100, RoadY = 200, Slip = 300;

    static PreparedSegment Road(long way, long a, long b, float3 start, float3 end, string classification, bool oneWay)
        => new(way, a, b, start, end, NetworkKind.Road, classification, 2, oneWay, false, false, 0)
        {
            SourceGeometry = new[] { start, end },
            OriginalGeometry = new[] { start, end }
        };

    static List<PreparedSegment> Junction(bool roadYOneWaySouth)
    {
        var junction = new float3(0, 0, 0);
        var a = new float3(-30, 0, 0);
        var b = new float3(0, 0, 30);

        var sections = new List<PreparedSegment>
        {
            Road(RoadX, 2, 10, new(-200, 0, 0), a, "secondary", false),
            Road(RoadX, 10, 1, a, junction, "secondary", false),
            Road(RoadX, 1, 3, junction, new(200, 0, 0), "secondary", false),
            Road(Slip, 10, 20, a, b, "secondary_link", true)
        };

        if (roadYOneWaySouth)
        {
            sections.Add(Road(RoadY, 5, 20, new(0, 0, 200), b, "secondary", true));
            sections.Add(Road(RoadY, 20, 1, b, junction, "secondary", true));
            sections.Add(Road(RoadY, 1, 4, junction, new(0, 0, -200), "secondary", true));
        }
        else
        {
            sections.Add(Road(RoadY, 4, 1, new(0, 0, -200), junction, "secondary", false));
            sections.Add(Road(RoadY, 1, 20, junction, b, "secondary", false));
            sections.Add(Road(RoadY, 20, 5, b, new(0, 0, 200), "secondary", false));
        }

        return sections;
    }

    static OsmDataset Restrictions(params OsmRelation[] relations) => new() { Relations = relations };

    static OsmRelation NoTurn(long from, long via, long to, string restriction = "no_left_turn")
        => new(
            1,
            Array.Empty<(long, bool)>(),
            new Dictionary<string, string> { ["type"] = "restriction", ["restriction"] = restriction })
        {
            OrderedMembers = new[]
            {
                new OsmMember("way", from, "from"),
                new OsmMember("node", via, "via"),
                new OsmMember("way", to, "to")
            }
        };

    public static void Run()
    {
        var plain = NetworkLimitedAccess.RegularLinks(Junction(false), Restrictions(), new ImportLog());
        Check(
            plain.All(segment => segment.WayId != Slip),
            "A slip lane whose turn the junction makes was not removed");

        var restricted =
            NetworkLimitedAccess.RegularLinks(Junction(false), Restrictions(NoTurn(RoadX, 1, RoadY)), new ImportLog());

        Check(
            restricted.Any(segment => segment.WayId == Slip && segment.Classification == "secondary"),
            "A slip lane was removed although the junction's turn it replaces is forbidden");

        var conditional =
            NetworkLimitedAccess.RegularLinks(
                Junction(false),
                Restrictions(
                    new OsmRelation(
                        2,
                        Array.Empty<(long, bool)>(),
                        new Dictionary<string, string>
                        {
                            ["type"] = "restriction",
                            ["restriction:conditional"] = "no_left_turn @ (Mo-Fr 07:00-09:00)"
                        })
                    {
                        OrderedMembers = NoTurn(RoadX, 1, RoadY).OrderedMembers
                    }),
                new ImportLog());

        Check(
            conditional.Any(segment => segment.WayId == Slip),
            "A slip lane was removed although its replacement turn is forbidden at times");

        var exceptCars =
            NetworkLimitedAccess.RegularLinks(
                Junction(false),
                Restrictions(
                    NoTurn(RoadX, 1, RoadY) with
                    {
                        Tags = new Dictionary<string, string>
                        {
                            ["type"] = "restriction",
                            ["restriction"] = "no_left_turn",
                            ["except"] = "motorcar"
                        }
                    }),
                new ImportLog());

        Check(
            exceptCars.All(segment => segment.WayId != Slip),
            "A restriction that excepts cars kept a slip lane");

        var oneWay = NetworkLimitedAccess.RegularLinks(Junction(true), Restrictions(), new ImportLog());
        Check(
            oneWay.Any(segment => segment.WayId == Slip),
            "A slip lane into a one-way road was removed although the junction cannot reach it legally");

        MutualDetours();
    }

    // Two links on a loop, each the other's only detour: removing both would cut the loop.
    static void MutualDetours()
    {
        var a = new float3(0, 0, 0);
        var b = new float3(100, 0, 0);
        var c = new float3(0, 0, 100);
        var d = new float3(100, 0, 100);

        var sections = new List<PreparedSegment>
        {
            Road(400, 1, 2, a, b, "tertiary_link", false),
            Road(401, 1, 3, a, c, "tertiary", false),
            Road(402, 3, 4, c, d, "tertiary_link", false),
            Road(403, 4, 2, d, b, "tertiary", false)
        };

        var result = NetworkLimitedAccess.RegularLinks(sections, Restrictions(), new ImportLog());
        Check(
            result.Count == 4 && result.All(segment => segment.Classification == "tertiary"),
            "Two links that were each the other's detour were both removed");
    }
}
