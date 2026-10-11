namespace ImportEarthLocation.Core;

public static class RailPrefabPolicy
{
    // Names are an allowlist, not proof that a bidirectional rail has two tracks.
    public static bool IsBaseCorridor(string name) =>
        name == "Twoway Train Track" || name == "Double Train Track" || name == "Train Track";

    public static bool CanUse(string name, int physicalTracks) =>
        IsBaseCorridor(name) && (physicalTracks == 1 || physicalTracks == 2);
}
