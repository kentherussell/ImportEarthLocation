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
}
