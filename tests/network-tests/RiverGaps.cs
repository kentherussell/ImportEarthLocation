using ImportEarthLocation.Core;
using Newtonsoft.Json.Linq;
using Unity.Mathematics;

// Where planned rivers are dry in the game: walks every planned reach of a run and reads
// the game's own water depth (native-water-depth-cm.u16, read back after placement) every
// 7 m along it. A dry stretch is a gap in a river the player sees as detached.
// Usage: network-tests --river-gaps <run folder> [minimum wet depth in metres, default 0.1]
static class RiverGaps
{
    public static void Run(string folder, float minimumDepth)
    {
        var meta = JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder, "native-water-depth.json")));
        var cells = (int)meta["cells"]!;
        var cellMeters = (float)meta["cellMeters"]!;
        var bytes = File.ReadAllBytes(CapturedRunFiles.Resolve(folder, (string)meta["file"]!));
        var depth = new ushort[bytes.Length / 2];
        Buffer.BlockCopy(bytes, 0, depth, 0, bytes.Length);
        var half = cells * cellMeters / 2;

        // Texel i holds the water at grid node i·cell from the south-west corner, the node
        // the game itself uses (WaterSystem.CalculateSourceMultiplier); bilinear between nodes.
        float Depth(float x, float z)
        {
            var u = (x + half) / cellMeters;
            var v = (z + half) / cellMeters;
            var i = (int)Math.Floor(u);
            var j = (int)Math.Floor(v);
            var fu = u - i;
            var fv = v - j;

            float At(int a, int b) =>
                a < 0 || b < 0 || a >= cells || b >= cells ? 0 : depth[b * cells + a] / 100f;

            return math.lerp(
                math.lerp(At(i, j), At(i + 1, j), fu),
                math.lerp(At(i, j + 1), At(i + 1, j + 1), fu),
                fv);
        }

        // The game's own terrain, read back after placement (a centred corner grid).
        var terrainMeta =
            JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder, "native-observed-terrain.json")));

        var ground =
            CapturedRunFiles.Heights(
                CapturedRunFiles.Resolve(folder, (string)terrainMeta["heightFile"]!),
                (float)terrainMeta["scale"]!);

        var n = (int)Math.Sqrt(ground.Length);
        var terrainWidth = (float)terrainMeta["width"]!;

        float Ground(float x, float z)
        {
            var u = Math.Clamp((x / terrainWidth + .5f) * (n - 1), 0, n - 1.001f);
            var v = Math.Clamp((z / terrainWidth + .5f) * (n - 1), 0, n - 1.001f);
            var i = (int)u;
            var j = (int)v;

            return math.lerp(
                math.lerp(ground[j * n + i], ground[j * n + i + 1], u - i),
                math.lerp(ground[(j + 1) * n + i], ground[(j + 1) * n + i + 1], u - i),
                v - j);
        }

        // Water sources as the game holds them after placement: position and top level.
        var snapshot = JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder, "native-snapshot-after-apply.json")));
        var sources =
            (snapshot["water"] as JArray ?? new JArray())
                .Select(item => new float3((float)item["x"]!, (float)item["y"]!, (float)item["z"]!))
                .ToArray();

        var sourceLevels =
            (snapshot["water"] as JArray ?? new JArray())
                .Select(item => (float)item["y"]! + (float)item["depth"]!)
                .ToArray();

        // A constant-level source holds water at its own position; one standing dry adds nothing.
        var radii =
            (snapshot["water"] as JArray ?? new JArray()).Select(item => (float)item["radius"]!).ToArray();

        foreach (var (label, wide) in new[] { ("channel (radius under 25 m)", false), ("polygon (radius 25 m or more)", true) })
        {
            var group = Enumerable.Range(0, sources.Length).Where(i => radii[i] >= 25 == wide).ToArray();
            var dead = group.Count(i => Depth(sources[i].x, sources[i].z) < minimumDepth);
            var shallow = group.Count(i => Depth(sources[i].x, sources[i].z) is var d && d >= minimumDepth && d < 1);

            Console.WriteLine(
                $"sources, {label}: {group.Length}; dry at the source {dead}; under 1 m {shallow}; median depth there {(group.Length == 0 ? 0 : group.Select(i => Depth(sources[i].x, sources[i].z)).OrderBy(d => d).ElementAt(group.Length / 2)):0.0} m; median planned depth {(group.Length == 0 ? 0 : group.Select(i => (float)snapshot["water"]![i]!["depth"]!).OrderBy(d => d).ElementAt(group.Length / 2)):0.0} m");
        }

        var manifest =
            JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder, "offline-design-pass-01-input.json")));

        var reaches =
            CorrectiveCapturedReplay.ReadManifest<WaterReachStage>(folder, (string)manifest["waterReachesManifest"]!);

        Console.WriteLine(
            $"reaches: {reaches.Length}; in polygons {reaches.Count(reach => reach.PolygonId != null)}; blocked by a buried inlet {reaches.Count(reach => reach.BlockedByBuriedInlet)} ({reaches.Where(reach => reach.BlockedByBuriedInlet).Sum(reach => math.distance(reach.Start.xz, reach.End.xz)) / 1000:0.0} km)");

        // Only channel reaches (no polygon): those are the river lines drawn by sources.
        // RIVER_PROFILE=<way id>: every 28 m along that way, with sources within 40 m of it.
        if (long.TryParse(Environment.GetEnvironmentVariable("RIVER_PROFILE"), out var profileWay))
        {
            var along = 0f;

            foreach (var reach in reaches.Where(reach => reach.WayId == profileWay))
            {
                var length = math.distance(reach.Start.xz, reach.End.xz);
                var steps = Math.Max(1, (int)Math.Ceiling(length / 28));

                for (var k = 0; k < steps; k++)
                {
                    var t = (k + .5f) / steps;
                    var point = math.lerp(reach.Start, reach.End, t);
                    var near =
                        Enumerable.Range(0, sources.Length)
                            .Where(i => math.distance(sources[i].xz, point.xz) < 20)
                            .Select(i => $"source bed {sources[i].y:0.0} level {sourceLevels[i]:0.0}")
                            .FirstOrDefault() ?? "";

                    Console.WriteLine(
                        $"PROFILE {along + t * length,7:0} m level {math.lerp(reach.StartLevel, reach.EndLevel, t),7:0.0} ground {Ground(point.x, point.z),7:0.0} water {Depth(point.x, point.z),5:0.0} {near}");
                }

                along += length;
            }

            return;
        }

        var ways =
            reaches
                .Where(reach => (Environment.GetEnvironmentVariable("RIVER_POLYGONS") == "1") == (reach.PolygonId != null) && !reach.BlockedByBuriedInlet)
                .GroupBy(reach => reach.WayId)
                .ToArray();

        double total = 0, dry = 0;
        var gaps = new List<(long Way, long Body, float Length, float3 At, float Level, float HalfWidth, float GroundAbove, float NearestSource)>();

        foreach (var way in ways)
        {
            var run = 0f;
            var runStart = float3.zero;
            var runAbove = float.NegativeInfinity;

            float Nearest(float3 point)
                => sources.Length == 0 ? float.NaN : sources.Min(source => math.distance(source.xz, point.xz));

            foreach (var reach in way)
            {
                var length = math.distance(reach.Start.xz, reach.End.xz);
                var steps = Math.Max(1, (int)Math.Ceiling(length / 7));

                for (var k = 0; k < steps; k++)
                {
                    var point = math.lerp(reach.Start, reach.End, (k + .5f) / steps);
                    var step = length / steps;
                    total += step;

                    var level = math.lerp(reach.StartLevel, reach.EndLevel, (k + .5f) / steps);

                    if (Depth(point.x, point.z) >= minimumDepth)
                    {
                        if (run > 0)
                            gaps.Add((way.Key, reach.BodyId, run, runStart, level, reach.HalfWidth, runAbove, Nearest(runStart)));

                        run = 0;
                        continue;
                    }

                    dry += step;

                    if (run == 0)
                    {
                        runStart = point;
                        runAbove = float.NegativeInfinity;
                    }

                    // How far the game's ground stands above the planned water level.
                    runAbove = Math.Max(runAbove, Ground(point.x, point.z) - level);
                    run += step;
                }
            }

            if (run > 0)
                gaps.Add((way.Key, way.First().BodyId, run, runStart, way.Last().EndLevel, way.Last().HalfWidth, runAbove, Nearest(runStart)));
        }

        Console.WriteLine(
            $"rivers: {ways.Length} ways, {total / 1000:0.0} km of channel; dry in game {dry / 1000:0.0} km ({100 * dry / Math.Max(1, total):0.0}%) at depth < {minimumDepth} m");

        foreach (var limit in new[] { 28f, 56f, 112f, 280f })
            Console.WriteLine(
                $"  gaps over {limit} m: {gaps.Count(gap => gap.Length > limit)}; {gaps.Where(gap => gap.Length > limit).Sum(gap => gap.Length) / 1000:0.0} km");

        var dryWays = gaps.GroupBy(gap => gap.Way).Count(group => group.Sum(gap => gap.Length) > 0);
        Console.WriteLine($"  ways with any gap: {dryWays} of {ways.Length}");

        var long_ = gaps.Where(gap => gap.Length > 28).ToArray();
        Console.WriteLine(
            $"  gaps over 28 m by highest ground above the planned level: under 0 m {long_.Count(gap => gap.GroundAbove < 0)}, 0-1 m {long_.Count(gap => gap.GroundAbove is >= 0 and < 1)}, 1-3 m {long_.Count(gap => gap.GroundAbove is >= 1 and < 3)}, 3 m or more {long_.Count(gap => gap.GroundAbove >= 3)}");

        Console.WriteLine(
            $"  gaps over 28 m by nearest source: under 200 m {long_.Count(gap => gap.NearestSource < 200)}, 200-400 m {long_.Count(gap => gap.NearestSource is >= 200 and < 400)}, 400 m or more {long_.Count(gap => gap.NearestSource >= 400)}; sources {sources.Length}");

        foreach (var gap in gaps.OrderByDescending(gap => gap.Length).Take(25))
            Console.WriteLine(
                $"  GAP way {gap.Way} body {gap.Body}: {gap.Length:0} m from ({gap.At.x:0}, {gap.At.z:0}); planned level {gap.Level:0.0}; ground up to {gap.GroundAbove:0.0} m above it; nearest source {gap.NearestSource:0} m; half width {gap.HalfWidth:0}");
    }
}
