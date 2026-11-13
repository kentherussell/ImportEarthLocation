using System;
using System.Linq;
using Game.Rendering;
using Game.Simulation;
using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace ImportEarthLocation.Integration;

// Diagnostics for sand at the waterline (report 113, §16). The game makes its final
// ground splatmap from the heightmap, the water texture and our painted user splatmap
// (TerrainMaterialSystem.UpdateSplatmap), and its terrain shader then blends each Extra
// layer by four global vectors set in ApplyRenderSettings. Sand is Extra 4, so it is
// component w of each. At a beach site this logs those vectors and both splatmaps
// around the site by depth below the planned sea. Where the game's water lies, the
// final splatmap's G channel is 0 and, with our faded paint, the ground drawn is base Dirt.
public static class BeachSurfaceProbe
{
    private static readonly int DirtOverride = Shader.PropertyToID("_TerrainExtraDirtOverride");
    private static readonly int BlurDepth = Shader.PropertyToID("_TerrainExtraBlurDepth");
    private static readonly int SplatRange = Shader.PropertyToID("_TerrainExtraSplatRange");
    private static readonly int HeightOffset = Shader.PropertyToID("_TerrainExtraHeightOffset");
    private static readonly int SplatMultiplier = Shader.PropertyToID("_SplatMultiplier");
    private static readonly int SplatPower = Shader.PropertyToID("_SplatPower");

    // Half the sampled square, in splatmap pixels (about 14 m each on a 57 km map).
    private const int Reach = 12;

    // Depth bins below the planned water surface, in metres; negative is dry land.
    private static readonly float[] DepthEdges = { -1, 0, 1, 3, 6, 12 };

    public static bool IsBeach(string siteName) => siteName.IndexOf("beach", StringComparison.OrdinalIgnoreCase) >= 0;

    // Sets Sand's dirt override to zero; returns what puts it back.
    public static Action ClearSandDirtOverride()
    {
        var previous = Shader.GetGlobalVector(DirtOverride);
        var cleared = previous;
        cleared.w = 0;
        Shader.SetGlobalVector(DirtOverride, cleared);

        return () => Shader.SetGlobalVector(DirtOverride, previous);
    }

    public static void LogSettings(ImportLog log)
    {
        string Text(int id) => Shader.GetGlobalVector(id).ToString("0.###");

        log.Diagnostic(
            $"BEACH PROBE shaderExtras dirtOverride={Text(DirtOverride)}; blurDepth={Text(BlurDepth)}; splatRange={Text(SplatRange)}; heightOffset={Text(HeightOffset)}; splatMultiplier={Shader.GetGlobalFloat(SplatMultiplier):0.###}; splatPower={Shader.GetGlobalFloat(SplatPower):0.###}; sandComponent=w");
    }

    // Per depth bin: pixel count, planned Sand share and mean strength, and the mean
    // channels of the user splatmap (RG) and of the final splatmap (RGBA), 0-255.
    public static void Sample(PreparedImport prepared, float3 point, string name, ImportLog log)
    {
        var world = World.DefaultGameObjectInjectionWorld;
        var materials = world?.GetExistingSystemManaged<TerrainMaterialSystem>();
        var terrain = world?.GetExistingSystemManaged<TerrainSystem>();
        var final = materials?.splatmap as RenderTexture;
        var user = materials == null ? null : NativeSurfaceBridge.NativeTexture(materials, false);

        if (terrain == null || final == null || user == null || prepared.SurfaceMask == null)
        {
            log.Diagnostic($"BEACH PROBE site=\"{name}\" unavailable");
            return;
        }

        var area = terrain.playableArea;
        var offset = terrain.playableOffset;
        var centre = (int2)math.floor((point.xz - offset) / area * new float2(final.width, final.height));
        var low = math.max(centre - Reach, 0);
        var high = math.min(centre + Reach, new int2(final.width, final.height) - 1);
        var size = high - low + 1;
        var finalPixels = Read(final, low, size, TextureFormat.RGBA32, 4);
        var userPixels = Read(user, (int2)math.floor((float2)low / final.width * user.width), size, TextureFormat.RG16, 2);
        var surface = WaterLevels.Effective(prepared.SeaLevelMeters);
        var bins = DepthEdges.Length + 1;
        var count = new int[bins];
        var sand = new int[bins];
        var strength = new double[bins];
        var userSum = new double[bins, 2];
        var finalSum = new double[bins, 4];

        for (var y = 0; y < size.y; y++)
            for (var x = 0; x < size.x; x++)
            {
                var position = (new float2(low.x + x, low.y + y) + .5f) / new float2(final.width, final.height) * area + offset;
                var height =
                    NetworkProcessor.Sample(position.x, position.y, prepared.Heights, prepared.Request.MapSize.WidthMeters);

                var depth = surface - height;
                var bin = Array.FindIndex(DepthEdges, edge => depth < edge);
                bin = bin < 0 ? bins - 1 : bin;
                var planned = SurfacePaint.Sample(prepared.SurfaceMask, position.x, position.y);
                var i = y * size.x + x;
                count[bin]++;

                if (planned.material == GroundMaterial.Sand && planned.intensity > 0)
                {
                    sand[bin]++;
                    strength[bin] += planned.intensity;
                }

                for (var c = 0; c < 2; c++)
                    userSum[bin, c] += userPixels[i * 2 + c];

                for (var c = 0; c < 4; c++)
                    finalSum[bin, c] += finalPixels[i * 4 + c];
            }

        var rows =
            Enumerable
                .Range(0, bins)
                .Where(bin => count[bin] > 0)
                .Select(
                    bin =>
                    {
                        var label =
                            bin == 0 ? $"<{DepthEdges[0]}" :
                            bin == bins - 1 ? $">={DepthEdges[bins - 2]}" : $"{DepthEdges[bin - 1]}..{DepthEdges[bin]}";

                        string Mean(double sum) => (sum / count[bin]).ToString("0");

                        return $"depth {label} m: n={count[bin]} sand={sand[bin]} strength={(sand[bin] == 0 ? 0 : strength[bin] / sand[bin]):0} "
                            + $"user=({Mean(userSum[bin, 0])},{Mean(userSum[bin, 1])}) "
                            + $"final=({Mean(finalSum[bin, 0])},{Mean(finalSum[bin, 1])},{Mean(finalSum[bin, 2])},{Mean(finalSum[bin, 3])})";
                    });

        log.Diagnostic($"BEACH PROBE site=\"{name}\" pixels={size.x}x{size.y}; " + string.Join("; ", rows));
    }

    private static byte[] Read(RenderTexture source, int2 origin, int2 size, TextureFormat format, int channels)
    {
        var previous = RenderTexture.active;
        var readback = new Texture2D(size.x, size.y, format, false, true);

        try
        {
            RenderTexture.active = source;
            readback.ReadPixels(new Rect(origin.x, origin.y, size.x, size.y), 0, 0, false);
            var bytes = readback.GetRawTextureData();

            if (bytes.Length != size.x * size.y * channels)
                throw new InvalidOperationException("Splatmap readback has an unexpected byte count.");

            return bytes;
        }
        finally
        {
            RenderTexture.active = previous;
            UnityEngine.Object.Destroy(readback);
        }
    }
}
