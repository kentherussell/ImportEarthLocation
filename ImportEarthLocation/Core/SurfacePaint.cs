using System;

namespace ImportEarthLocation.Core;

/// <summary>One brush per sample. The game's paint shaders own native storage encoding; Default is implicit.</summary>
public static class SurfacePaint
{
    // Installed vanilla prefab identities and TerrainMaterialProperties.m_Type,
    // corroborated by the bundled English tool descriptions and paint-system IL.
    public static GroundMaterial VanillaMaterial(string name, int nativeType) => (name, nativeType) switch
    {
        ("Terrain Material Extra 1", 4) => GroundMaterial.DryGrass,
        ("Terrain Material Extra 2", 5) => GroundMaterial.Grass,
        ("Terrain Material Extra 3", 6) => GroundMaterial.Dirt,
        ("Terrain Material Extra 4", 7) => GroundMaterial.Sand,
        _ => GroundMaterial.Base
    };

    public static void Validate(SurfaceMask mask)
    {
        if (mask.Resolution < 2 || mask.Resolution > 4096 || double.IsNaN(mask.WidthMeters) ||
            double.IsInfinity(mask.WidthMeters) || mask.WidthMeters <= 0 ||
            mask.Material.Length != (long)mask.Resolution * mask.Resolution || mask.Intensity.Length != mask.Material.Length)
            throw new ArgumentException("Invalid surface-mask dimensions or bounds.");
        for (var i = 0; i < mask.Material.Length; i++)
            if (mask.Material[i] > (byte)GroundMaterial.Dirt || mask.Material[i] == 0 && mask.Intensity[i] != 0)
                throw new ArgumentException("Surface mask contains an invalid material or painted Default sample.");
    }

    public static (GroundMaterial material, byte intensity) Sample(SurfaceMask mask, double x, double z)
    {
        var n = mask.Resolution;
        var px = Math.Max(0, Math.Min(n - 1, (x / mask.WidthMeters + .5) * n - .5));
        var pz = Math.Max(0, Math.Min(n - 1, (z / mask.WidthMeters + .5) * n - .5));
        var nearest = (int)Math.Round(pz) * n + (int)Math.Round(px);
        var material = mask.Material[nearest];
        if (material == 0) return (GroundMaterial.Base, 0);
        var x0 = (int)px; var z0 = (int)pz; var x1 = Math.Min(n - 1, x0 + 1); var z1 = Math.Min(n - 1, z0 + 1);
        var fx = px - x0; var fz = pz - z0;
        // Interpolate only the winning brush. Never mix two materials or normalize
        // away the Default remainder at a boundary.
        double Value(int i) => mask.Material[i] == material ? mask.Intensity[i] : 0;
        var value = (Value(z0 * n + x0) * (1 - fx) + Value(z0 * n + x1) * fx) * (1 - fz) +
                    (Value(z1 * n + x0) * (1 - fx) + Value(z1 * n + x1) * fx) * fz;
        return ((GroundMaterial)material, (byte)Math.Round(value));
    }
}
