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
        var x0 = (int)px; var z0 = (int)pz; var x1 = Math.Min(n - 1, x0 + 1); var z1 = Math.Min(n - 1, z0 + 1);
        var fx = px - x0; var fz = pz - z0;
        int a = z0 * n + x0, b = z0 * n + x1, c = z1 * n + x0, d = z1 * n + x1;
        double wa = (1 - fx) * (1 - fz), wb = fx * (1 - fz), wc = (1 - fx) * fz, wd = fx * fz;
        byte ma = mask.Material[a], mb = mask.Material[b], mc = mask.Material[c], md = mask.Material[d];
        double Share(byte m) => (ma == m ? wa : 0) + (mb == m ? wb : 0) + (mc == m ? wc : 0) + (md == m ? wd : 0);
        // The material with the largest bilinear share wins. Ties keep the
        // nearest cell's material.
        var material = mask.Material[(int)Math.Round(pz) * n + (int)Math.Round(px)]; var best = Share(material);
        void Consider(byte candidate) { var share = Share(candidate); if (share > best + 1e-9) { best = share; material = candidate; } }
        Consider(ma); Consider(mb); Consider(mc); Consider(md);
        if (material == 0) return (GroundMaterial.Base, 0);
        // Interpolate only the winning brush; never mix two materials. Each
        // native texel holds one material, so wherever the winner changes the
        // texel grid shows: a material meeting another one, or bare ground, at
        // any strength drew a 14 m staircase along every slanted border (the
        // inland edge of a beach, a park against a lawn). The winner's strength
        // therefore falls to zero where the runner-up's share reaches its own,
        // and the border passes through unpainted ground instead of a step.
        double Value(int i, byte m, double w) => m == material ? w * mask.Intensity[i] : 0;
        var value = Value(a, ma, wa) + Value(b, mb, wb) + Value(c, mc, wc) + Value(d, md, wd);
        var second = 0d;
        void Runner(byte candidate) { if (candidate != material) second = Math.Max(second, Share(candidate)); }
        Runner(ma); Runner(mb); Runner(mc); Runner(md);
        var margin = Math.Max(0, Math.Min(1, 1 - second / best)); margin = margin * margin * (3 - 2 * margin);
        return ((GroundMaterial)material, (byte)Math.Round(Math.Min(255, value / best * margin)));
    }
}
