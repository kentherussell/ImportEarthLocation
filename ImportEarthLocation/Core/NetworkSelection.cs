using System;
using System.Collections.Generic;

namespace ImportEarthLocation.Core;

public enum RoadImportLevel { None, Highways, Primary, Secondary }
public static class NetworkSelection
{
    public static RoadImportLevel Parse(string? value, bool legacySecondary = false) => value?.ToLowerInvariant() switch
    { "none" => RoadImportLevel.None, "highways" => RoadImportLevel.Highways, "primary" => RoadImportLevel.Primary, "secondary" => RoadImportLevel.Secondary,
      _ => legacySecondary ? RoadImportLevel.Secondary : RoadImportLevel.Primary };
    public static bool ParseRail(string? value) => bool.TryParse(value, out var enabled) ? enabled : true;
    public static string Id(RoadImportLevel value) => Enum.IsDefined(typeof(RoadImportLevel), value) ? value.ToString().ToLowerInvariant() : "primary";
    public static string Label(RoadImportLevel value) => value switch
    { RoadImportLevel.None => "None", RoadImportLevel.Highways => "Highways only", RoadImportLevel.Secondary => "Secondary roads & above", _ => "Primary roads & above" };
    public static string Classes(RoadImportLevel value) => value switch
    { RoadImportLevel.None => "", RoadImportLevel.Highways => "motorway|motorway_link|trunk|trunk_link",
      RoadImportLevel.Secondary => "motorway|motorway_link|trunk|trunk_link|primary|primary_link|secondary|secondary_link",
      _ => "motorway|motorway_link|trunk|trunk_link|primary|primary_link" };
    public static bool Includes(RoadImportLevel value, string classification)
    {
        foreach(var candidate in Classes(value).Split('|')) if(candidate.Length > 0 && candidate.Equals(classification,StringComparison.OrdinalIgnoreCase)) return true;
        return false;
    }
    public static string Summary(RoadImportLevel roads, bool rail) => $"Network import: Roads: {Label(roads)}; Rail: {(rail ? "Enabled" : "Disabled")}.";
    public static bool NativeDirectionMatches(bool oneWay, bool forwardOnly, bool backwardOnly) => oneWay ? forwardOnly && !backwardOnly : !forwardOnly && !backwardOnly;
    public static string BasicRoadName(int lanes) => lanes >= 5 ? "Large Road" : lanes >= 3 ? "Medium Road" : "Small Road";
    public static bool IsBasicVanillaRoad(string name)
    {
        if(name=="Small Road"||name=="Medium Road"||name=="Large Road"||name=="Highway")return true;
        foreach(var prefix in new[]{"Small Road Oneway - ","Medium Road Oneway - ","Large Road Oneway - ","Highway Oneway - ","Highway Twoway - "})
            for(var lanes=1;lanes<=6;lanes++)if(name==prefix+lanes+" lane"+(lanes==1?"":"s"))return true;
        return false;
    }
}
