using System;
using System.Linq;
using System.Reflection;

namespace ImportEarthLocation.Integration;

public static class MapExtDetector
{
    private static Type? PatchManager => AppDomain.CurrentDomain.GetAssemblies()
        .Select(assembly => assembly.GetType("MapExtPDX.MapExt.Core.PatchManager", throwOnError: false))
        .FirstOrDefault(type => type != null);

    public static bool IsInstalled => AppDomain.CurrentDomain.GetAssemblies().Any(assembly =>
        assembly.GetName().Name?.IndexOf("MapExt", StringComparison.OrdinalIgnoreCase) >= 0) || PatchManager != null;

    public static bool HasVerifiedSizeApi
    {
        get
        {
            try { return PatchManager?.GetProperty("CurrentMapSize", BindingFlags.Public | BindingFlags.Static)?.GetValue(null) is IConvertible value && IsSupported(value.ToDouble(System.Globalization.CultureInfo.InvariantCulture)); }
            catch { return false; }
        }
    }

    private static bool IsSupported(double width) => new[] { 14_336d, 28_672d, 57_344d }.Any(size => Math.Abs(size - width) < 2d);

    public static bool TryGetCurrentMapWidth(out double widthMeters)
    {
        widthMeters = 14_336d;
        var patchManager = PatchManager;
        if (patchManager == null && !IsInstalled) return true;

        try
        {
            var property = patchManager?.GetProperty("CurrentMapSize", BindingFlags.Public | BindingFlags.Static);
            if (property?.GetValue(null) is IConvertible value)
            {
                widthMeters = value.ToDouble(System.Globalization.CultureInfo.InvariantCulture);
                if (IsSupported(widthMeters)) return true;
            }
        }
        catch { }
        // A loaded but incompatible MapExt must never be assumed to be 57 km.
        widthMeters = 14_336d;
        return false;
    }
}
