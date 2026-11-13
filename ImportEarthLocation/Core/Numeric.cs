using Unity.Mathematics;

namespace ImportEarthLocation.Core;

public static class Numeric
{
    public static int Clamp(int value, int minimum, int maximum)
        => value < minimum ? minimum : value > maximum ? maximum : value;

    public static double Clamp(double value, double minimum, double maximum)
        => value < minimum ? minimum : value > maximum ? maximum : value;

    public static float SmoothStep(float x)
    {
        x = math.clamp(x, 0, 1);
        return x * x * (3 - 2 * x);
    }

    public static bool IsFinite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
}
