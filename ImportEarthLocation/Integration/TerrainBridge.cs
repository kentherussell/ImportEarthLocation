using System;
using System.Linq;
using System.Reflection;
using Game.Simulation;
using ImportEarthLocation.Core;
using Unity.Mathematics;
using UnityEngine;

namespace ImportEarthLocation.Integration;

public sealed class TerrainBridge
{
    private readonly TerrainSystem _terrainSystem;
    private readonly WaterSystem _waterSystem;
    private readonly MethodInfo? _setTerrainProperties;
    private readonly MethodInfo? _replaceHeightmap;

    public TerrainBridge(TerrainSystem terrainSystem, WaterSystem waterSystem)
    {
        _terrainSystem = terrainSystem;
        _waterSystem = waterSystem;
        _setTerrainProperties = typeof(TerrainSystem)
            .GetMethods(BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic)
            .FirstOrDefault(method =>
                method.Name.IndexOf("SetTerrainProperties", StringComparison.OrdinalIgnoreCase) >= 0 &&
                method.GetParameters().Length == 1 &&
                method.GetParameters()[0].ParameterType == typeof(float2));
        _replaceHeightmap = typeof(TerrainSystem).GetMethod(
            "ReplaceHeightmap",
            BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic,
            binder: null,
            types: new[] { typeof(Texture2D) },
            modifiers: null);
    }

    public bool CanApply(out string reason)
    {
        if (!IsTerrainLoaded()) { reason = "Editor terrain resources are not ready. Finish loading a map before preparing an import."; return false; }
        if (!_waterSystem.Loaded) { reason = "WaterSystem is not loaded; open a map in the editor first."; return false; }
        if (_replaceHeightmap == null) { reason = "The current CS2 TerrainSystem no longer exposes the expected ReplaceHeightmap hook."; return false; }
        if (_setTerrainProperties == null) { reason = "The current CS2 TerrainSystem no longer exposes the expected terrain-property hook."; return false; }
        reason = "";
        return true;
    }

    private bool IsTerrainLoaded()
    {
        // m_Loaded is a one-update cascade-refresh flag, NOT persistent map readiness.
        // Verify the render heightmap and the public CPU terrain data instead.
        var heightmap = _terrainSystem.heightmap;
        if (heightmap == null || heightmap.width <= 0 || heightmap.height <= 0) return false;
        var data = _terrainSystem.GetHeightData(false);
        return data.heights.IsCreated && data.heights.Length > 0 && data.resolution.x > 2 && data.resolution.z > 2;
    }

    public void Apply(PreparedImport prepared)
    {
        if (!CanApply(out var reason)) throw new InvalidOperationException(reason);
        if (prepared.Heightmap.Length != Processing.TerrainBuilder.Resolution * Processing.TerrainBuilder.Resolution)
            throw new InvalidOperationException("Prepared heightmap dimensions are invalid.");

        var texture = new Texture2D(
            Processing.TerrainBuilder.Resolution,
            Processing.TerrainBuilder.Resolution,
            TextureFormat.R16,
            mipChain: false,
            linear: true)
        { name = $"{Mod.Id}_Heightmap" };

        var bytes = new byte[prepared.Heightmap.Length * sizeof(ushort)];
        Buffer.BlockCopy(prepared.Heightmap, 0, bytes, 0, bytes.Length);
        texture.LoadRawTextureData(bytes);
        // ReplaceHeightmap calls GetRawTextureData<ushort> to populate CPU heights.
        texture.Apply(updateMipmaps: false, makeNoLongerReadable: false);

        // The private hook name and float2 payload are preflighted because this API is not public/stable.
        var parameters = _setTerrainProperties!.GetParameters();
        var arguments = parameters.Select(parameter =>
            parameter.ParameterType == typeof(float2)
                ? (object)new float2(prepared.HeightScaleMeters, 0f)
                : parameter.HasDefaultValue ? parameter.DefaultValue : Activator.CreateInstance(parameter.ParameterType)).ToArray();
        try
        {
            _setTerrainProperties.Invoke(_terrainSystem, arguments);
            _terrainSystem.ReplaceHeightmap(texture);
            _waterSystem.SeaLevel = WaterLevels.Effective(prepared.SeaLevelMeters);
        }
        finally
        {
            UnityEngine.Object.Destroy(texture);
        }
    }
}
