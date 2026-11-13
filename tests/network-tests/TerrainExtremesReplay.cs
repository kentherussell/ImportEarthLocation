using ImportEarthLocation.Processing;
using Newtonsoft.Json.Linq;

// TerrainExtremes on a capture's grid before road design, with no road network: the
// terrain alone. Reports the extremes before and after, what moved, and checks that
// nothing beside water, waterways, cliffs or coast changed and that slopes are kept.
static partial class VisualMetricsReplay
{
    public static void TerrainExtremesReplay(string folder, string output)
    {
        var capture = Load(folder);
        var size = (int)Math.Sqrt(capture.Terrain.Length);
        var water = TerrainExtremes.Protected(size, capture.Width, capture.Areas, Array.Empty<Unity.Mathematics.float3[]>());
        var lines = TerrainExtremes.Protected(size, capture.Width, Array.Empty<Unity.Mathematics.float3[]>(), capture.Lines);
        var clock = System.Diagnostics.Stopwatch.StartNew();
        var smoothed = TerrainExtremes.Smooth(capture.Terrain, null, null, water, lines);
        var (finished, spikes) = TerrainExtremes.RemoveSpikes(smoothed.Heights, water, lines, ReadFloodInputs(capture).SeaSurface is var sea && !float.IsNaN(sea) ? sea : float.NegativeInfinity);
        var outcome = smoothed with { Heights = finished };
        var seconds = clock.Elapsed.TotalSeconds;

        bool NearEdge(int x, int z)
        {
            for (var dz = -2; dz <= 2; dz++)
                for (var dx = -2; dx <= 2; dx++)
                {
                    var nx = x + dx;
                    var nz = z + dz;

                    if (nx >= 0 && nz >= 0 && nx < size && nz < size
                        && (water[nz * size + nx] != 0 || lines[nz * size + nx] != 0))
                        return true;
                }

            return false;
        }

        // Rises over 2 m between neighbours: terraces (flat on both far sides) and the
        // rest (slopes), away from water and mapped edges; and terraces at those edges.
        JObject Rises(float[] grid)
        {
            int terraces = 0, slopes = 0, edges = 0;

            foreach (var step in PlanMeasures.Steps(grid, size))
                if (NearEdge(step.X, step.Z))
                    edges++;
                else
                    terraces++;

            for (var z = 0; z < size; z++)
                for (var x = 0; x + 1 < size; x++)
                {
                    if (Math.Abs(grid[z * size + x + 1] - grid[z * size + x]) > TerrainExtremes.MaxRise)
                        slopes++;

                    if (Math.Abs(grid[(x + 1) * size + z] - grid[x * size + z]) > TerrainExtremes.MaxRise)
                        slopes++;
                }

            var spikeCount = 0;

            for (var z = 1; z < size - 1; z++)
                for (var x = 1; x < size - 1; x++)
                    if (PlanMeasures.IsSpike(grid, size, x, z))
                        spikeCount++;

            return new JObject
            {
                ["terracesAwayFromEdges"] = terraces,
                ["terracesAtWaterAndCliffs"] = edges,
                ["spikes"] = spikeCount,
                ["risesOver2m"] = slopes,
            };
        }

        var changedNearEdge = 0;
        var changed = 0;
        double volume = 0;
        var cellArea = Math.Pow(capture.Width / (size - 1), 2);

        for (var id = 0; id < capture.Terrain.Length; id++)
        {
            var change = Math.Abs(outcome.Heights[id] - capture.Terrain[id]);

            if (change < .05f)
                continue;

            changed++;
            volume += change * cellArea;

            if (NearEdge(id % size, id / size))
                changedNearEdge++;
        }

        var report = new JObject
        {
            ["capture"] = Path.GetFileName(folder.TrimEnd('/', '\\')),
            ["cellMetres"] = Math.Round(capture.Width / (size - 1), 2),
            ["before"] = Rises(capture.Terrain),
            ["after"] = Rises(outcome.Heights),
            ["spikesRemoved"] = spikes,
            ["terracesSmoothed"] = outcome.Terraces,
            ["cellsMoved"] = changed,
            ["areaMovedKm2"] = Math.Round(changed * cellArea / 1e6, 2),
            ["largestMoveMetres"] = Math.Round(outcome.LargestMetres, 1),
            ["volumeMovedMillionM3"] = Math.Round(volume / 1e6, 2),
            ["cellsMovedWithin2CellsOfWaterOrMappedEdges"] = changedNearEdge,
            ["seconds"] = Math.Round(seconds, 1),
        };

        Directory.CreateDirectory(output);
        File.WriteAllText(Path.Combine(output, "terrain-extremes.json"), report.ToString());
        Console.WriteLine(report.ToString(Newtonsoft.Json.Formatting.None));

        if (changedNearEdge > 0)
            throw new InvalidDataException($"{changedNearEdge} cells on water or mapped edges moved.");
    }
}
