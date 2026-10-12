using System;
using System.Collections.Generic;

namespace ImportEarthLocation.Core;

[Flags]
public enum RoadImportLevel { None=0, Highways=1, Routes=2, Streets=4, Primary=Highways|Routes|Streets, Secondary=Primary }
public static class NetworkSelection
{
    public static RoadImportLevel Parse(string? value, bool legacySecondary = false)
    {
        var result=RoadImportLevel.None;
        foreach(var item in (value??"").ToLowerInvariant().Split('|'))
            result |= item switch { "highways"=>RoadImportLevel.Highways,"routes"=>RoadImportLevel.Routes,"streets"=>RoadImportLevel.Streets,_=>RoadImportLevel.None };
        return result;
    }
    public static bool ParseRail(string? value) => bool.TryParse(value, out var enabled) ? enabled : true;
    public static string Id(RoadImportLevel value) {var names=new List<string>();if((value&RoadImportLevel.Highways)!=0)names.Add("highways");if((value&RoadImportLevel.Routes)!=0)names.Add("routes");if((value&RoadImportLevel.Streets)!=0)names.Add("streets");return names.Count==0?"none":string.Join("|",names);}
    public static string Label(RoadImportLevel value) => Id(value).Replace("|",", ");
    public static string Classes(RoadImportLevel value) {var names=new List<string>();if((value&RoadImportLevel.Highways)!=0)names.Add("motorway|motorway_link");if((value&RoadImportLevel.Routes)!=0)names.Add((value&RoadImportLevel.Streets)!=0?"trunk|trunk_link":"trunk");if((value&RoadImportLevel.Streets)!=0)names.Add("primary|secondary|tertiary");return string.Join("|",names);}
    public static bool IsExcludedStreetLink(string classification) => classification is "primary_link" or "secondary_link" or "tertiary_link";
    public static int MatchedLanes(int requested,bool highway,bool oneWay) {var choices=highway?(oneWay?new[]{1,2,3,4,5}:new[]{2,3,4,8}):(oneWay?new[]{1,2,3}:new[]{2,4,6});var best=choices[0];foreach(var lanes in choices)if(Math.Abs(lanes-requested)<=Math.Abs(best-requested))best=lanes;return best;}
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
            for(var lanes=1;lanes<=8;lanes++)if(name==prefix+lanes+" lane"+(lanes==1?"":"s"))return true;
        return false;
    }
}
