using System;
using System.IO;
using System.Linq;

namespace ImportEarthLocation.Core;

// Design changes under trial (report 112, 11.4). Each is off unless named in the
// import request's DesignSwitches (set by the autorun) or in the replay's --with,
// and off reproduces the design as it was. One import designs at a time, so the
// switches for the running design are held here rather than passed through every
// stage; the import sets them before design starts.
public static class DesignSwitches
{
    // B1: lower building and canopy bumps in the elevation data near roads.
    public const string SourceCleaning = "source-cleaning";

    // B2: bridge and tunnel approaches keep full earthwork banks instead of narrowed ones.
    // The planned fill/cut envelope was tried and dropped: it doubled the replayed terrain
    // steps on Cairo and DC (report 112, section 12).
    public const string WideApproaches = "wide-approaches";

    // B3: one 12 m structure depth for profiles, bridges and tunnels.
    public const string StructureDepth = "structure-depth";

    // W1: the sea lifts roads only on below-sea ground connected to the map edge.
    public const string SeaFromEdge = "sea-from-edge";

    // Terrain: remove spikes and smooth terraces and walls beside roads on the finished grid.
    public const string SmoothExtremes = "smooth-extremes";

    // Terrain: dykes and levees hold the sea off land below it (FloodDefences); roads are
    // lifted for the sea only where it can still reach them, as with SeaFromEdge.
    public const string FloodDefences = "flood-defences";

    // Water: lakes and rivers are written into the game's water grid at their planned
    // levels when the sources are created, instead of filling from the sources.
    public const string PlaceWater = "place-water";

    // Water: a river keeps its level over longer and deeper bank dips, with a levee
    // raised beside it there, instead of dropping its water to the lowest bank.
    public const string RaiseBanks = "raise-banks";

    // Roads: one junction rule, one fitted curve for every road with its junctions placed
    // by the fit, and planned cuts (report 113).
    public const string SmoothRoads = "smooth-roads";

    public static readonly string[] All =
    {
        SourceCleaning,
        WideApproaches,
        StructureDepth,
        SeaFromEdge,
        SmoothExtremes,
        FloodDefences,
        PlaceWater,
        RaiseBanks,
        SmoothRoads
    };

    private static volatile string[] s_Active = Array.Empty<string>();

    public static string Active => string.Join(",", s_Active);

    public static bool On(string name) => Array.IndexOf(s_Active, name) >= 0;

    // A comma-separated list; unknown names are ignored, so an old request still loads.
    public static void Set(string? names)
        => s_Active =
            (names ?? "")
                .Split(',')
                .Select(name => name.Trim())
                .Where(name => Array.IndexOf(All, name) >= 0)
                .Distinct()
                .ToArray();
}
