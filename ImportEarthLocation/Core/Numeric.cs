namespace ImportEarthLocation.Core;

public static class Numeric
{
    public static int Clamp(int value, int minimum, int maximum) => value < minimum ? minimum : value > maximum ? maximum : value;
    public static float Clamp(float value, float minimum, float maximum) => value < minimum ? minimum : value > maximum ? maximum : value;
    public static double Clamp(double value, double minimum, double maximum) => value < minimum ? minimum : value > maximum ? maximum : value;
}
