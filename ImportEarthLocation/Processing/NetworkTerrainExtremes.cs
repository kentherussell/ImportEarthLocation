using System;
using System.Collections.Generic;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// The finished grid's extremes (TerrainExtremes), with the road network as fixed cells:
// ground road and rail decks out to one cell past their edge (the ground the game reads
// under them) and tunnels with two cells around them (their roof). Bridges stand clear
// of the ground and fix nothing. Spikes are removed after the smoothing.
public static class NetworkTerrainExtremes
{
    public static float[] Prepare(
        float[] heights,
        double width,
        IReadOnlyList<PreparedSegment> segments,
        IEnumerable<float3[]> protectedAreas,
        IEnumerable<float3[]> protectedLines,
        ImportLog log,
        CancellationToken token,
        bool[]? held = null,
        float seaSurface = float.NegativeInfinity)
    {
        try
        {
            var clock = System.Diagnostics.Stopwatch.StartNew();
            var n = (int)Math.Sqrt(heights.Length);
            var cell = (float)(width / (n - 1));
            // Held cells (flood defences) never move.
            var fixedCells = held != null ? (bool[])held.Clone() : new bool[n * n];
            var deck = new bool[n * n];

            int Pixel(float value) => (int)Math.Round((value / width + .5) * (n - 1));

            void Disc(bool[] mask, float3 point, float radius)
            {
                var reach = (int)Math.Ceiling(radius / cell);
                var px = Pixel(point.x);
                var pz = Pixel(point.z);

                for (var z = Math.Max(0, pz - reach); z <= Math.Min(n - 1, pz + reach); z++)
                    for (var x = Math.Max(0, px - reach); x <= Math.Min(n - 1, px + reach); x++)
                    {
                        var dx = (x - px) * cell;
                        var dz = (z - pz) * cell;

                        if (dx * dx + dz * dz <= radius * radius)
                            mask[z * n + x] = true;
                    }
            }

            foreach (var segment in segments)
            {
                token.ThrowIfCancellationRequested();

                if (segment.Bridge && !segment.Tunnel)
                    continue;

                var half = segment.NativeWidth > 0 ? segment.NativeWidth * .5f : 6;
                var steps =
                    Math.Max(1, (int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(segment) / (cell * .5f)));

                for (var i = 0; i <= steps; i++)
                {
                    var point = NetworkNormalizer.Evaluate(segment, i / (float)steps);

                    if (segment.Tunnel)
                        Disc(fixedCells, point, half + 2 * cell);
                    else
                        Disc(deck, point, half + cell);
                }
            }

            var water = TerrainExtremes.Protected(n, width, protectedAreas, Array.Empty<float3[]>());
            var lines = TerrainExtremes.Protected(n, width, Array.Empty<float3[]>(), protectedLines);
            var outcome = TerrainExtremes.Smooth(heights, fixedCells, deck, water, lines, token);
            // After smoothing: a held cell can be left standing above smoothed cells around it.
            var (finished, spikes) = TerrainExtremes.RemoveSpikes(outcome.Heights, water, lines, seaSurface);

            log.Count("Terrain spikes and pits removed", spikes);
            log.Count("Terrain terraces smoothed", outcome.Terraces);
            log.Count("Terrain walls beside roads smoothed", outcome.Walls);
            log.Evidence(
                $"Terrain extremes: {spikes:N0} spikes removed; {outcome.Terraces:N0} terrace steps and {outcome.Walls:N0} walls beside roads over {TerrainExtremes.MaxRise:0} m smoothed; {outcome.Moved:N0} cells moved ({outcome.Moved * cell * cell / 1e6:0.00} km²), at most {outcome.LargestMetres:0.0} m; {clock.Elapsed.TotalSeconds:0.0} s.");

            return finished;
        }
        catch (Exception exception) when (exception is not OperationCanceledException)
        {
            log.Warn("Terrain extremes not smoothed: " + exception.Message);
            log.Count("Design switch failures");
            return heights;
        }
    }
}
