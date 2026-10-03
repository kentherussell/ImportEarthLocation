using System;
using System.Collections.Generic;
using System.Linq;
using Game.Prefabs;
using Game.Rendering;
using Game.Simulation;
using ImportEarthLocation.Core;
using Unity.Mathematics;
using UnityEngine;

namespace ImportEarthLocation.Integration;

public static class NativeSurfaceBridge
{
    public static string Apply(SurfaceMask mask, Unity.Entities.EntityManager manager, PrefabSystem prefabs, TerrainMaterialSystem materials, TerrainSystem terrain, ImportLog log)
    {
        var channels = new Dictionary<GroundMaterial, int>();
        foreach (var prefab in NativePrefabCatalog.Read<TerraformingPrefab>(manager, prefabs))
        {
            if (!prefab.TryGet<TerrainMaterialProperties>(out var property)) continue;
            var channel = (int)property.m_Type - (int)TerrainMaterialType.Extra1;
            if (channel < 0 || channel > 3) continue;
            var name = prefab.name.Replace(" ", "").Replace("_", "").ToLowerInvariant();
            GroundMaterial kind;
            if (name.Contains("sand")) kind = GroundMaterial.Sand;
            else if (name.Contains("drygrass")) kind = GroundMaterial.DryGrass;
            else if (name.Contains("fertile") || name.Contains("grass")) kind = GroundMaterial.Grass;
            else if (name.Contains("dirt")) kind = GroundMaterial.Dirt;
            else continue;
            if (!channels.ContainsKey(kind)) channels[kind] = channel;
        }
        var used = mask.Material.Where(m => m != 0).Distinct().Select(m => (GroundMaterial)m).ToArray();
        if (used.Length == 0) return "Ground surfaces: no confidently mapped cover; base terrain preserved.";
        if (used.Any(m => !channels.ContainsKey(m)))
            return "Ground surfaces skipped: native paint channels could not be verified for " + string.Join(", ", used.Where(m => !channels.ContainsKey(m))) + ".";
        if (!(materials.UserSplatmap is RenderTexture local) || !(materials.UserSplatmapWorld is RenderTexture world))
            return "Ground surfaces skipped: current native splat textures are unavailable.";
        if (local.format != RenderTextureFormat.ARGB32 || world.format != RenderTextureFormat.ARGB32)
            return "Ground surfaces skipped: native paint texture format changed; refusing an unverified copy.";
        // The engine serializes these two native user splatmaps. RGBA channels are verified in ApplyToSplatmap.
        // Resolve exclusively before this point; never blend/stack incompatible material writes.
        ApplyTexture(local, terrain.playableArea, terrain.playableOffset);
        ApplyTexture(world, terrain.worldSize, terrain.worldOffset);
        materials.ForceUpdateWholeSplatmap();
        log.Info("Native ground surfaces applied using exclusive RGBA channel + normalized intensity. Native splatmaps will be included in map saves.");
        return "Ground surfaces applied: exclusive + intensity feathering.";

        void ApplyTexture(RenderTexture target, float2 area, float2 offset)
        {
            if (area.x <= 0 || area.y <= 0 || target.width <= 0 || target.height <= 0 || (long)target.width * target.height > 16_777_216)
                throw new InvalidOperationException("Unsupported native paint texture size/bounds.");
            var texture = new Texture2D(target.width, target.height, TextureFormat.RGBA32, false, true);
            var previous = RenderTexture.active;
            try
            {
                if (!target.IsCreated()) target.Create();
                RenderTexture.active = target;
                texture.ReadPixels(new Rect(0, 0, target.width, target.height), 0, 0, false);
                var pixels = texture.GetPixels32();
                var half = (float)(mask.WidthMeters / 2);
                for (var y = 0; y < target.height; y++) for (var x = 0; x < target.width; x++)
                {
                    var position = new float2((x + .5f) / target.width, (y + .5f) / target.height) * area + offset;
                    if (math.any(math.abs(position) > half)) continue;
                    var mx = Math.Min(mask.Resolution - 1, Math.Max(0, (int)((position.x + half) / mask.WidthMeters * mask.Resolution)));
                    var my = Math.Min(mask.Resolution - 1, Math.Max(0, (int)((position.y + half) / mask.WidthMeters * mask.Resolution)));
                    var i = my * mask.Resolution + mx; var color = new Color32(0, 0, 0, 0);
                    if (mask.Material[i] != 0)
                    {
                        var intensity = mask.Intensity[i];
                        switch (channels[(GroundMaterial)mask.Material[i]]) { case 0: color.r = intensity; break; case 1: color.g = intensity; break; case 2: color.b = intensity; break; case 3: color.a = intensity; break; }
                    }
                    pixels[y * target.width + x] = color;
                }
                texture.SetPixels32(pixels); texture.Apply(false, false);
                if (target.format != RenderTextureFormat.ARGB32)
                    throw new InvalidOperationException("Native paint texture format changed; refusing an unverified copy.");
                Graphics.CopyTexture(texture, target);
            }
            finally { RenderTexture.active = previous; UnityEngine.Object.Destroy(texture); }
        }
    }
}
