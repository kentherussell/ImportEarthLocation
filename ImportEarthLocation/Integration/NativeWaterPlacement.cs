using System;
using System.Threading.Tasks;
using Game.Simulation;
using ImportEarthLocation.Core;
using UnityEngine;
using UnityEngine.Experimental.Rendering;

namespace ImportEarthLocation.Integration;

// Puts the planned lakes and rivers straight into the game's water simulation, so they
// are full at once instead of filling from their sources (idea from the Real World Maps
// mod, MIT). The water texture holds one cell per WaterSystem.CellSize over the playable
// area, four floats each: depth, velocity x, velocity z, pollution. The current texture
// is read back first and only depth is raised, never lowered: the sea, anything the
// sources already filled and every velocity stay as they are. Only planned lake and
// river surfaces are placed (NetworkWaterSurface has no sea), so land below the sea
// gets no water from here. Sources still hold the levels afterwards.
public static class NativeWaterPlacement
{
    // Once placed, the water runs this much faster for this many seconds of running
    // simulation, so it settles soon. The game sets the speed to 0 while terrain
    // changes; that pause is left alone and does not count.
    private const int SettleSpeed = 4;
    private const float SettleSeconds = 12;

    private static float s_SettleLeft;
    private static int s_SettleRestore;

    public static void Settle(WaterSystem water, SimulationSystem simulation)
    {
        if (s_SettleRestore == 0)
            return;

        if (simulation.selectedSpeed <= 0 || water.WaterSimSpeed == 0)
            return;

        s_SettleLeft -= Time.unscaledDeltaTime;

        if (s_SettleLeft > 0)
        {
            water.WaterSimSpeed = SettleSpeed;
            return;
        }

        water.WaterSimSpeed = s_SettleRestore;
        s_SettleRestore = 0;
    }

    public static string Apply(PreparedImport prepared, WaterSystem water, ImportLog log)
    {
        var surface = prepared.NetworkWaterSurface;

        if (surface == null || prepared.Heights.Length == 0)
            return "Water placement skipped: no planned water surface.";

        if (Texture(water) is not RenderTexture target
            || Processing.TerrainBuilder.Resolution % target.width != 0
            || Math.Abs(target.width * water.CellSize - prepared.Request.MapSize.PlayableMeters) > water.CellSize)
        {
            log.Diagnostic(
                $"WATER PLACEMENT skipped; texture={water.WaterTexture?.GetType().Name}; size={water.WaterTexture?.width}x{water.WaterTexture?.height}; format={(water.WaterTexture as RenderTexture)?.graphicsFormat}");

            return "Water placement skipped: the water texture is not the expected format; the sources fill the water.";
        }

        var clock = System.Diagnostics.Stopwatch.StartNew();
        var cells = target.width;
        var block = Processing.TerrainBuilder.Resolution / cells;
        var resolution = Processing.TerrainBuilder.Resolution;
        var playable = prepared.Request.MapSize.PlayableMeters;
        var cell = playable / cells;

        // The game's heightmap samples sit at pixel centres (MapSizeProfile), so water
        // cell (col, row) covers samples col·block .. col·block + block - 1.
        var current = Read(target);
        var data = current.GetPixelData<float>(0).ToArray();
        UnityEngine.Object.Destroy(current);
        var placed = 0;
        var raised = 0;

        Parallel.For(
            0,
            cells,
            () => (Placed: 0, Raised: 0),
            (row, _, counts) =>
            {
                for (var col = 0; col < cells; col++)
                {
                    var centre =
                        new Unity.Mathematics.float3(
                            (float)(-playable / 2 + (col + .5) * cell),
                            0,
                            (float)(-playable / 2 + (row + .5) * cell));

                    if (surface(centre) is not float level || !float.IsFinite(level))
                        continue;

                    var ground = 0d;

                    for (var z = row * block; z < (row + 1) * block; z++)
                        for (var x = col * block; x < (col + 1) * block; x++)
                            ground += prepared.Heights[z * resolution + x];

                    var depth = level - (float)(ground / (block * block));
                    var index = (row * cells + col) * 4;

                    if (depth <= 0)
                        continue;

                    counts.Placed++;

                    if (depth > data[index])
                    {
                        data[index] = depth;
                        counts.Raised++;
                    }
                }

                return counts;
            },
            counts =>
            {
                System.Threading.Interlocked.Add(ref placed, counts.Placed);
                System.Threading.Interlocked.Add(ref raised, counts.Raised);
            });

        var upload = new Texture2D(cells, cells, GraphicsFormat.R32G32B32A32_SFloat, TextureCreationFlags.None);

        try
        {
            upload.SetPixelData(data, 0);
            upload.Apply(false, false);
            Graphics.CopyTexture(upload, target);
        }
        finally
        {
            UnityEngine.Object.Destroy(upload);
        }

        s_SettleRestore = Math.Max(1, water.WaterSimSpeed);
        s_SettleLeft = SettleSeconds;
        log.Diagnostic(
            $"WATER PLACEMENT cells={cells}x{cells}; cellMeters={cell:R}; nativeCellSize={water.CellSize:R}; plannedWetCells={placed}; raisedCells={raised}; seaUntouched=true; settleSpeed={SettleSpeed}; settleSeconds={SettleSeconds}; seconds={clock.Elapsed.TotalSeconds:0.00}");

        return $"Water placed directly in {raised:N0} of {placed:N0} planned lake and river cells; the sources hold their levels.";
    }

    // The game's water depth over the whole map, in centimetres (0 dry, 65,535 for
    // 655 m or more), one value per water cell, row 0 at the map's south edge. Null
    // when the texture is not the expected format.
    public static ushort[]? DepthCentimetres(WaterSystem water, out int cells)
    {
        cells = 0;

        if (Texture(water) is not RenderTexture target)
            return null;

        cells = target.width;
        var copy = Read(target);

        try
        {
            var data = copy.GetPixelData<float>(0);
            var depths = new ushort[cells * cells];

            for (var i = 0; i < depths.Length; i++)
                depths[i] = (ushort)Math.Round(Math.Max(0, Math.Min(65535, data[i * 4] * 100)));

            return depths;
        }
        finally
        {
            UnityEngine.Object.Destroy(copy);
        }
    }

    // The water texture when it is the square four-float texture this class reads and
    // writes; null otherwise.
    private static RenderTexture? Texture(WaterSystem water)
        => water.WaterTexture is RenderTexture texture
            && texture.width == texture.height
            && texture.graphicsFormat == GraphicsFormat.R32G32B32A32_SFloat
                ? texture
                : null;

    private static Texture2D Read(RenderTexture source)
    {
        var copy = new Texture2D(source.width, source.height, GraphicsFormat.R32G32B32A32_SFloat, TextureCreationFlags.None);
        var active = RenderTexture.active;

        try
        {
            RenderTexture.active = source;
            copy.ReadPixels(new Rect(0, 0, source.width, source.height), 0, 0, false);
            copy.Apply(false, false);
        }
        finally
        {
            RenderTexture.active = active;
        }

        return copy;
    }
}
