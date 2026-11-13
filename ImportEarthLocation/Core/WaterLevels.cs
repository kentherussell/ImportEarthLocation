namespace ImportEarthLocation.Core;

public static class WaterLevels
{
    public const float CorrectionMeters = 1f;
    public const float RoadFreeboardMeters = .75f;
    public const float RoadBridgeDepthMeters = 3f;
    public const float RoadBridgeClearanceMeters = 3.5f;

    // Mapped water that is part of the sea: it shares the sea's surface and bed and
    // never gets a lake level or banks of its own. A harbour planned as a lake stood
    // above the sea behind its own raised banks (Marseille's Old Port, report 103).
    public static bool SeaTagged(System.Collections.Generic.IReadOnlyDictionary<string, string> tags)
    {
        string Tag(string key) => tags.TryGetValue(key, out var value) ? value : "";

        var water = Tag("water");

        return water is "sea" or "ocean" or "bay" or "harbour" or "dock"
            || Tag("natural") == "bay"
            || Tag("harbour") == "yes"
            || Tag("seamark:type") == "harbour_basin"
            || Tag("tidal") == "yes";
    }

    public static float Effective(float nominalEncodedSeaLevel) => nominalEncodedSeaLevel + CorrectionMeters;
}
