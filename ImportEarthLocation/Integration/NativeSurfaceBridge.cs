using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using Game.Prefabs;
using Game.Rendering;
using Game.Simulation;
using Game.Tools;
using ImportEarthLocation.Core;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Experimental.Rendering;

namespace ImportEarthLocation.Integration;

public static class NativeSurfaceBridge
{
    internal static RenderTexture? NativeTexture(TerrainMaterialSystem materials, bool world)
    {
        // Public getters return black fallback textures until editor painting starts.
        var field = typeof(TerrainMaterialSystem).GetField(world ? "m_UserSplatmapWorld" : "m_UserSplatmap", BindingFlags.Instance | BindingFlags.NonPublic);
        return field?.GetValue(materials) as RenderTexture;
    }

    public static string Apply(SurfaceMask mask, EntityManager manager, PrefabSystem prefabs,
        TerrainMaterialSystem materials, TerrainSystem terrain, ImportLog log)
    {
        SurfacePaint.Validate(mask);
        NativeDiagnostics.SurfaceApplied = false;
        var tools = new Dictionary<GroundMaterial, Entity>(); var erase = Entity.Null;
        foreach (var prefab in NativePrefabCatalog.Read<TerraformingPrefab>(manager, prefabs))
        {
            if (!prefab.TryGet<TerrainMaterialProperties>(out var property)) continue;
            // The native erase kernel is selected by None, not a localized name.
            if (property.m_Type == TerrainMaterialType.None && erase == Entity.Null)
            {
                erase = prefabs.GetEntity(prefab);
                log.Diagnostic($"SURFACE ERASER prefab={prefab.name}; nativeType=None; encoding=nativeRG16");
            }
            var kind = SurfacePaint.VanillaMaterial(prefab.name, (int)property.m_Type);
            if (kind == GroundMaterial.Base) continue;
            tools[kind] = prefabs.GetEntity(prefab);
            log.Diagnostic($"SURFACE BRUSH prefab={prefab.name}; nativeType={property.m_Type}; recognized=true; storageEncoding=nativeRG16; brushSelectionIsNotTextureChannels=true");
        }
        var used = mask.Material.Where(m => m != 0).Distinct().OrderBy(m => m).Select(m => (GroundMaterial)m).ToArray();
        log.Diagnostic($"SURFACE maskResolution={mask.Resolution}; widthMeters={mask.WidthMeters:0.###}; mappedSamples={mask.Material.Count(m => m != 0)}; materials=" +
            string.Join(",", used.Select(m => $"{m}:{mask.Material.Count(v => v == (byte)m)}")));
        if (erase == Entity.Null || used.Any(m => !tools.ContainsKey(m) || tools[m] == Entity.Null))
            throw new InvalidOperationException("Native terrain paint/eraser tools could not be resolved.");

        // Four brush selections are NOT four stored RGBA weights. The game's
        // PaintDither/EraseDither kernels own the R8G8_UNorm encoding and opacity.
        var paint = typeof(TerrainMaterialSystem).GetMethod("ApplyToSplatmap", BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.Public,
            null, new[] { typeof(RenderTexture), typeof(Brush), typeof(Texture), typeof(float2), typeof(float2) }, null)
            ?? throw new InvalidOperationException("Native terrain brush application contract changed.");
        var local = NativeTexture(materials, false); var world = NativeTexture(materials, true);
        log.Diagnostic($"SURFACE TARGETS local={Describe(local)}; world={Describe(world)}; encoding=nativeRG16");
        CheckTarget(local, terrain.playableArea, terrain.playableOffset);
        CheckTarget(world, terrain.worldSize, terrain.worldOffset);
        var verified = 0;
        try
        {
            ApplyTexture(local!, terrain.playableArea, terrain.playableOffset, "local");
            ApplyTexture(world!, terrain.worldSize, terrain.worldOffset, "world");
        }
        finally { materials.ForceUpdateWholeSplatmap(); }
        NativeDiagnostics.SurfaceApplied = verified == 2;
        return "Ground surfaces applied through native paint shaders; both encoded texture copies verified. Appearance and save/reload still require game checks.";

        void ApplyTexture(RenderTexture target, float2 area, float2 offset, string label)
        {
            var scratch = new RenderTexture(target.descriptor) { name = "IEL native surface staging", enableRandomWrite = true };
            var brushTexture = new Texture2D(target.width, target.height, TextureFormat.RGBA32, false, true)
                { filterMode = FilterMode.Point, wrapMode = TextureWrapMode.Clamp };
            var readback = new Texture2D(target.width, target.height, TextureFormat.RG16, false, true);
            var previous = RenderTexture.active; var previousSrgb = GL.sRGBWrite;
            try
            {
                if (!scratch.Create()) throw new InvalidOperationException("Native paint staging allocation failed.");
                GL.sRGBWrite = false;
                if (target.IsCreated()) Graphics.CopyTexture(target, scratch);
                else { RenderTexture.active = scratch; GL.Clear(false, true, Color.clear); }
                byte[] Read(RenderTexture source)
                {
                    RenderTexture.active = source;
                    readback.ReadPixels(new Rect(0, 0, source.width, source.height), 0, 0, false);
                    var bytes = readback.GetRawTextureData();
                    if (bytes.Length != (long)source.width * source.height * 2)
                        throw new InvalidOperationException("Native RG16 readback has an unexpected byte count.");
                    return bytes;
                }
                var before = Read(scratch);
                var pixels = new Color32[target.width * target.height];
                var selection = new byte[pixels.Length]; var strength = new byte[pixels.Length];
                var half = mask.WidthMeters * .5; var affected = 0; var requested = 0;
                for (var y = 0; y < target.height; y++) for (var x = 0; x < target.width; x++)
                {
                    var i = y * target.width + x;
                    var position = new float2((x + .5f) / target.width, (y + .5f) / target.height) * area + offset;
                    if (math.any(math.abs(position) > (float)half)) { selection[i] = byte.MaxValue; continue; }
                    affected++;
                    var sample = SurfacePaint.Sample(mask, position.x, position.y);
                    selection[i] = (byte)sample.material; strength[i] = sample.intensity;
                    if (sample.intensity > 0) requested++;
                }
                var center = offset + area * .5f;
                var brush = new Brush { m_Position = new float3(center.x, 0, center.y), m_Size = area.x, m_Angle = 0, m_Strength = 1, m_Opacity = 1 };
                void Dispatch(Entity tool, GroundMaterial? material)
                {
                    // Full native-resolution, point-filtered, mutually exclusive masks:
                    // every pixel receives at most one painted material after erasing.
                    for (var i = 0; i < pixels.Length; i++)
                    {
                        var value = material.HasValue ? selection[i] == (byte)material.Value ? strength[i] : (byte)0 :
                            selection[i] != byte.MaxValue ? (byte)255 : (byte)0;
                        pixels[i] = new Color32(value, value, value, value);
                    }
                    brushTexture.SetPixels32(pixels); brushTexture.Apply(false, false); brush.m_Tool = tool;
                    try { paint.Invoke(materials, new object[] { scratch, brush, brushTexture, area, offset }); }
                    catch (TargetInvocationException exception) { throw new InvalidOperationException("Native terrain brush failed.", exception.InnerException ?? exception); }
                }
                Dispatch(erase, null);
                foreach (var material in used) Dispatch(tools[material], material);
                var expected = Read(scratch); var changed = 0; var painted = 0; var outsideChanges = 0; var defaultResidual = 0;
                for (var i = 0; i < selection.Length; i++)
                {
                    var r = i * 2; var g = r + 1;
                    var different = expected[r] != before[r] || expected[g] != before[g];
                    if (different) changed++;
                    if (selection[i] == byte.MaxValue) { if (different) outsideChanges++; }
                    else if (selection[i] == 0 && (expected[r] != 0 || expected[g] != 0)) defaultResidual++;
                    else if (expected[r] != 0 || expected[g] != 0) painted++;
                }
                if (outsideChanges != 0 || defaultResidual != 0 || requested > 100 && painted == 0)
                    throw new InvalidOperationException($"Native surface staging failed verification: outsideChanges={outsideChanges}, DefaultResidual={defaultResidual}, requested={requested}, painted={painted}. Native target not replaced.");
                if (!target.IsCreated() && !target.Create()) throw new InvalidOperationException("Native paint target allocation failed.");
                // Identical RG16 GPU formats: copy shader-encoded data without conversion.
                Graphics.CopyTexture(scratch, target);
                var actual = Read(target); var mismatched = 0;
                for (var i = 0; i < selection.Length; i++)
                    if (expected[i * 2] != actual[i * 2] || expected[i * 2 + 1] != actual[i * 2 + 1]) mismatched++;
                var hash = NativeDiagnostics.PixelHash(actual);
                log.Diagnostic($"SURFACE COPY target={label}; api=nativePaintDither+EraseDither/Graphics.CopyTexture; encoding=nativeRG16; dimensions={target.width}x{target.height}; format={target.format}; gpuFormat={target.graphicsFormat}; visitedPixels={affected}; requestedPaintPixels={requested}; encodedNonzeroPixels={painted}; changedPixels={changed}; outsideChanges={outsideChanges}; DefaultResidual={defaultResidual}; exclusiveMasks=true; area={area}; offset={offset}");
                log.Diagnostic($"SURFACE READBACK target={label}; comparedPixels={selection.Length}; mismatchedPixels={mismatched}; match={mismatched == 0}; gpuBytesHash={hash}; nativeShaderEncoding=true; rendererAppearanceVerified=false");
                if (mismatched != 0) throw new InvalidOperationException("Native paint texture copy readback differs from the native shader result.");
                NativeDiagnostics.RememberSurface(label, hash); verified++;
            }
            finally
            {
                GL.sRGBWrite = previousSrgb; RenderTexture.active = previous;
                scratch.Release(); UnityEngine.Object.Destroy(scratch); UnityEngine.Object.Destroy(brushTexture); UnityEngine.Object.Destroy(readback);
            }
        }
    }
    private static void CheckTarget(RenderTexture? target, float2 area, float2 offset)
    {
        if (target == null || target.graphicsFormat != GraphicsFormat.R8G8_UNorm || !target.enableRandomWrite ||
            !math.all(math.isfinite(area)) || !math.all(math.isfinite(offset)) || math.any(area <= 0) || Math.Abs(area.x - area.y) > .01f ||
            target.width <= 0 || target.height <= 0 || (long)target.width * target.height > 16_777_216 || target.sRGB)
            throw new InvalidOperationException("Native terrain paint format, square bounds or compute-write contract changed.");
    }
    private static string Describe(RenderTexture? texture) => texture == null ? "missing" : $"{texture.width}x{texture.height},format={texture.format},gpuFormat={texture.graphicsFormat},created={texture.IsCreated()}";
}
