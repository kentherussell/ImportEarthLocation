namespace ImportEarthLocation.Core;

public static class WaterLevels
{
    public const float CorrectionMeters = 1f;
    public const float RoadFreeboardMeters = .75f;
    public const float RoadBridgeDepthMeters = 3f;
    public const float RoadBridgeClearanceMeters = 3.5f;
    public static float Effective(float nominalEncodedSeaLevel) => nominalEncodedSeaLevel + CorrectionMeters;
}
