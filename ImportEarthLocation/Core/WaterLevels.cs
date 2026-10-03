namespace ImportEarthLocation.Core;

public static class WaterLevels
{
    public const float CorrectionMeters = 1f;
    public static float Effective(float nominalEncodedSeaLevel) => nominalEncodedSeaLevel + CorrectionMeters;
}
