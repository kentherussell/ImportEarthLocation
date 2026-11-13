using System;
using System.Collections.Generic;

namespace ImportEarthLocation.Core;

[Flags]
public enum RoadImportLevel
{
    None = 0,
    Highways = 1,
    Routes = 2,
    Streets = 4,
    Primary = Highways | Routes | Streets,
    Secondary = Primary
}

public static class NetworkSelection
{
    public static RoadImportLevel Parse(string? value)
    {
        var result = RoadImportLevel.None;

        foreach (var item in (value ?? "").ToLowerInvariant().Split('|'))
            result |= item switch
            {
                "highways" => RoadImportLevel.Highways,
                "routes" => RoadImportLevel.Routes,
                "streets" => RoadImportLevel.Streets,
                _ => RoadImportLevel.None
            };

        // Keep historical flag values readable while presenting one highway group.
        if ((result & (RoadImportLevel.Highways | RoadImportLevel.Routes)) != 0)
            result |= RoadImportLevel.Highways | RoadImportLevel.Routes;

        return result;
    }

    public static string Id(RoadImportLevel value)
    {
        var names = new List<string>();

        if ((value & RoadImportLevel.Highways) != 0)
            names.Add("highways");

        if ((value & RoadImportLevel.Routes) != 0)
            names.Add("routes");

        if ((value & RoadImportLevel.Streets) != 0)
            names.Add("streets");

        return names.Count == 0 ? "none" : string.Join("|", names);
    }

    public static string Label(RoadImportLevel value)
    {
        var names = new List<string>();

        if ((value & (RoadImportLevel.Highways | RoadImportLevel.Routes)) != 0)
            names.Add("Highways");

        if ((value & RoadImportLevel.Streets) != 0)
            names.Add("Main Roads");

        return names.Count == 0 ? "None" : string.Join(", ", names);
    }

    public static string Classes(RoadImportLevel value)
    {
        var names = new List<string>();

        if ((value & RoadImportLevel.Highways) != 0)
            names.Add("motorway|motorway_link");

        if ((value & RoadImportLevel.Routes) != 0)
            names.Add("trunk|trunk_link");

        if ((value & RoadImportLevel.Streets) != 0)
            names.Add("primary|secondary|tertiary");

        return string.Join("|", names);
    }

    public static bool IsExcludedStreetLink(string classification)
        => classification is "primary_link" or "secondary_link" or "tertiary_link";

    public static int MatchedLanes(int requested, bool highway, bool oneWay)
    {
        var choices =
            highway
                ? (oneWay ? new[] { 1, 2, 3, 4, 5 } : new[] { 2, 3, 4, 8 })
                : (oneWay ? new[] { 1, 2, 3, 4, 5, 6, 7 } : new[] { 2, 4, 6 });

        // Asset representation is a ceiling, not nearest-neighbour rounding:
        // five through seven two-way highway lanes must not become four.
        if (highway)
        {
            foreach (var lanes in choices)
                if (lanes >= requested)
                    return lanes;

            return choices[choices.Length - 1];
        }

        var best = choices[0];

        foreach (var lanes in choices)
            if (Math.Abs(lanes - requested) <= Math.Abs(best - requested))
                best = lanes;

        return best;
    }

    public static bool Includes(RoadImportLevel value, string classification)
    {
        foreach (var candidate in Classes(value).Split('|'))
            if (candidate.Length > 0 && candidate.Equals(classification, StringComparison.OrdinalIgnoreCase))
                return true;

        return false;
    }

    public static string Summary(RoadImportLevel roads, bool rail)
        => $"Network import: Roads: {Label(roads)}; Rail: {(rail ? "Enabled" : "Disabled")}.";

    public static bool NativeDirectionMatches(bool oneWay, bool forwardOnly, bool backwardOnly)
        => oneWay ? forwardOnly && !backwardOnly : !forwardOnly && !backwardOnly;

    public static string BasicRoadName(int lanes)
        => lanes >= 5 ? "Large Road" : lanes >= 3 ? "Medium Road" : "Small Road";

    // Most car lanes a regular one-way road asset has.
    public const int OneWayRoadLanes = 7;

    // The stock regular one-way roads by car lanes, as named in the game's locale
    // (Assets.NAME[...]). The two-, four- and six-lane roads carry no count in their
    // names; asking for "Small Road Oneway - 2 lanes" found nothing, and every
    // two-lane one-way street fell back to the three-lane road.
    public static string OneWayRoadName(int lanes)
        => Math.Clamp(lanes, 1, OneWayRoadLanes) switch
        {
            1 => "Small Road Oneway - 1 lane",
            2 => "Small Road Oneway",
            3 => "Small Road Oneway - 3 lanes",
            4 => "Medium Road Oneway",
            5 => "Medium Road Oneway - 5 lanes",
            6 => "Large Road Oneway",
            _ => "Large Road Oneway - 7 lanes"
        };

    public static bool HasTag(IReadOnlyDictionary<string, string> tags, string key, string value)
        => tags.TryGetValue(key, out var actual) && actual == value;
}
