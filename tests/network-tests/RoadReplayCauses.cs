using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Newtonsoft.Json.Linq;
using Unity.Mathematics;

// Why a replayed plan has open tunnels, more bridge than the map, roads bridged after
// earthworks and roads in water (report 113, 11.2). These only sort what the pipeline
// already produced; they change nothing. Distances are metres in map coordinates,
// lengths are reported in kilometres.
static partial class VisualMetricsReplay
{
    // A road or railway passes over a tunnel section when one of its samples lies within
    // this distance, on the map, of a sample of the section.
    private const float OverheadReach = 12;

    private const float OverheadSpacing = 8;

    private static double RoadBridgeKilometres(IEnumerable<PreparedSegment> sections)
        => sections
            .Where(segment => segment.Kind == NetworkKind.Road && segment.Bridge)
            .Sum(segment => NetworkVerticalProfiles.ArcLength(segment)) / 1000;

    private static bool NearMask(byte[] mask, double width, float3 point, int reach)
    {
        var size = (int)Math.Sqrt(mask.Length);
        var cx = (int)Math.Round((point.x / width + .5) * (size - 1));
        var cz = (int)Math.Round((point.z / width + .5) * (size - 1));

        for (var dz = -reach; dz <= reach; dz++)
            for (var dx = -reach; dx <= reach; dx++)
                if (cx + dx >= 0 && cz + dz >= 0 && cx + dx < size && cz + dz < size
                    && mask[(cz + dz) * size + cx + dx] != 0)
                    return true;

        return false;
    }

    // Why the earthworks left a road point off the ground: in mapped water or a quarry,
    // beside a waterway, cliff or coast line (the roadbed guards both), deeper than the
    // class budget against the ground the design started from, or none of these.
    private static string OffGroundReason(
        PreparedSegment segment,
        float3 point,
        byte[] areas,
        byte[] lines,
        float[] designTerrain,
        double width)
    {
        if (NearMask(areas, width, point, 1))
            return "water or quarry";

        if (NearMask(lines, width, point, 2))
            return "beside waterway, cliff or coast";

        var before = NetworkProcessor.Sample(point.x, point.z, designTerrain, width);

        return Math.Abs(point.y - before) > NetworkVerticalProfiles.CutFillBudget(segment)
            ? "deeper than budget"
            : "other";
    }

    private static float3[] OverheadSamples(PreparedSegment segment)
    {
        var count = Math.Max(2, (int)(NetworkVerticalProfiles.ArcLength(segment) / OverheadSpacing) + 1);

        return Enumerable.Range(0, count)
            .Select(i => NetworkNormalizer.Evaluate(segment, i / (count - 1f)))
            .ToArray();
    }

    private static void AddKilometres(JObject totals, string key, double kilometres)
        => totals[key] = Math.Round(((double?)totals[key] ?? 0) + kilometres, 3);

    private static JObject Causes(
        IReadOnlyList<PreparedSegment> beforeEarthworks,
        IReadOnlyList<PreparedSegment> final,
        IReadOnlyList<PreparedSegment> openBores,
        IReadOnlyList<(string Step, double Kilometres)> bridgeSteps,
        float[] roadbed,
        float[] designTerrain,
        double width,
        byte[] areas,
        byte[] lines,
        float sea)
    {
        // Bridge length after each step that can add bridges, and what the step added.
        var bridges = new JArray();
        var last = 0d;

        foreach (var (step, kilometres) in bridgeSteps)
        {
            bridges.Add(
                new JObject
                {
                    ["step"] = step,
                    ["bridgeKm"] = Math.Round(kilometres, 2),
                    ["addedKm"] = Math.Round(kilometres - last, 2)
                });

            last = kilometres;
        }

        Console.WriteLine(
            "bridge km added by step: "
                + string.Join("; ", bridges.Select(row => $"{row["step"]} {(double)row["addedKm"]!:+0.0;-0.0}")));

        // Samples of everything that is not in a tunnel, to find what passes over a bore.
        var overhead = new Dictionary<(int, int), List<(float3 Point, PreparedSegment Section)>>();

        foreach (var segment in final.Where(segment => !segment.Tunnel))
            foreach (var point in OverheadSamples(segment))
            {
                var cell = ((int)Math.Floor(point.x / OverheadReach), (int)Math.Floor(point.z / OverheadReach));

                if (!overhead.TryGetValue(cell, out var list))
                    overhead[cell] = list = new();

                list.Add((point, segment));
            }

        string Over(PreparedSegment bore)
        {
            var railway = false;
            var road = false;
            var water = false;

            foreach (var point in OverheadSamples(bore))
            {
                water |= NearMask(areas, width, point, 0);
                var cx = (int)Math.Floor(point.x / OverheadReach);
                var cz = (int)Math.Floor(point.z / OverheadReach);

                for (var dz = -1; dz <= 1; dz++)
                    for (var dx = -1; dx <= 1; dx++)
                    {
                        if (!overhead.TryGetValue((cx + dx, cz + dz), out var list))
                            continue;

                        foreach (var (other, section) in list)
                        {
                            var joined =
                                section.Kind == bore.Kind
                                    && (section.StartNodeId == bore.StartNodeId
                                        || section.StartNodeId == bore.EndNodeId
                                        || section.EndNodeId == bore.StartNodeId
                                        || section.EndNodeId == bore.EndNodeId);

                            if (joined || math.distance(other.xz, point.xz) > OverheadReach)
                                continue;

                            railway |= section.Kind != NetworkKind.Road;
                            road |= section.Kind == NetworkKind.Road;
                        }
                    }
            }

            return railway ? "railway over" : road ? "road over" : water ? "mapped water over" : "nothing mapped over";
        }

        // An open bore is a mouth when one end meets open road, a whole short underpass
        // when both do, and inside a longer tunnel when neither does.
        var openArm =
            new HashSet<long>(
                final
                    .Where(segment => segment.Kind == NetworkKind.Road && !segment.Tunnel)
                    .SelectMany(segment => new[] { segment.StartNodeId, segment.EndNodeId }));

        var bores = new JObject();
        var boreCounts = new JObject();

        foreach (var bore in openBores)
        {
            var ends = (openArm.Contains(bore.StartNodeId) ? 1 : 0) + (openArm.Contains(bore.EndNodeId) ? 1 : 0);
            var place = ends == 2 ? "whole underpass" : ends == 1 ? "mouth piece" : "inside a tunnel";
            var key = place + ", " + Over(bore);
            AddKilometres(bores, key, NetworkVerticalProfiles.ArcLength(bore) / 1000);
            boreCounts[key] = ((int?)boreCounts[key] ?? 0) + 1;
        }

        Console.WriteLine(
            "open bores by place (km, sections): "
                + string.Join(
                    "; ",
                    bores.Properties()
                        .OrderByDescending(entry => (double)entry.Value)
                        .Select(entry => $"{entry.Name} {(double)entry.Value:0.00} x{boreCounts[entry.Name]}")));

        // Road sections that became bridges after the earthworks, by why the ground at
        // their middle could not be brought to the deck.
        var bridgedBefore =
            new HashSet<(long, long, long)>(
                beforeEarthworks
                    .Where(segment => segment.Bridge)
                    .Select(segment => (segment.WayId, segment.StartNodeId, segment.EndNodeId)));

        var late = new JObject();
        var lateClear = new List<float>();

        foreach (var segment in final.Where(
            segment => segment.Kind == NetworkKind.Road
                && segment.Bridge
                && !bridgedBefore.Contains((segment.WayId, segment.StartNodeId, segment.EndNodeId))))
        {
            var middle = NetworkNormalizer.Evaluate(segment, .5f);
            lateClear.Add(middle.y - NetworkProcessor.Sample(middle.x, middle.z, roadbed, width));
            AddKilometres(
                late,
                OffGroundReason(segment, middle, areas, lines, designTerrain, width),
                NetworkVerticalProfiles.ArcLength(segment) / 1000);
        }

        lateClear.Sort();

        Console.WriteLine(
            $"bridged after earthworks: {lateClear.Count} sections; deck over the ground at the middle, median "
                + $"{(lateClear.Count > 0 ? lateClear[lateClear.Count / 2] : 0):0.0} m, most "
                + $"{(lateClear.Count > 0 ? lateClear[^1] : 0):0.0} m; km by reason: "
                + string.Join(
                    "; ",
                    late.Properties()
                        .OrderByDescending(entry => (double)entry.Value)
                        .Select(entry => $"{entry.Name} {(double)entry.Value:0.00}")));

        // Ground road in water: on a cell of mapped water or quarry, with its deck below
        // the sea surface, or with the finished ground under it below the sea surface.
        // Land behind a dyke is rightly below the sea, so the last two are not faults alone.
        double inWater = 0, deckBelowSea = 0, groundBelowSea = 0, inWaterBelowSea = 0;

        foreach (var segment in final.Where(
            segment => segment.Kind == NetworkKind.Road && !NetworkStructure.IsStructure(segment)))
        {
            var piece = NetworkVerticalProfiles.ArcLength(segment) / 8 / 1000;

            for (var i = 1; i <= 8; i++)
            {
                var point = NetworkNormalizer.Evaluate(segment, i / 8f);
                var wet = NearMask(areas, width, point, 0);
                var low = !float.IsNaN(sea) && point.y < sea;
                inWater += wet ? piece : 0;
                deckBelowSea += low ? piece : 0;
                inWaterBelowSea += wet && low ? piece : 0;

                if (!float.IsNaN(sea) && NetworkProcessor.Sample(point.x, point.z, roadbed, width) < sea)
                    groundBelowSea += piece;
            }
        }

        Console.WriteLine(
            $"ground road in water (km): on mapped water or quarry {inWater:0.00}; deck below the sea surface "
                + $"{deckBelowSea:0.00}; both {inWaterBelowSea:0.00}; ground under it below the sea surface "
                + $"{groundBelowSea:0.00}");

        return new JObject
        {
            ["bridgeKmByStep"] = bridges,
            ["openBoreKm"] = bores,
            ["openBoreSections"] = boreCounts,
            ["bridgedAfterEarthworks"] =
                new JObject
                {
                    ["sections"] = lateClear.Count,
                    ["medianClearMetres"] = lateClear.Count > 0 ? Math.Round(lateClear[lateClear.Count / 2], 1) : 0,
                    ["kmByReason"] = late
                },
            ["groundRoadInWater"] =
                new JObject
                {
                    ["onMappedWaterOrQuarryKm"] = Math.Round(inWater, 2),
                    ["deckBelowSeaKm"] = Math.Round(deckBelowSea, 2),
                    ["onWaterAndDeckBelowSeaKm"] = Math.Round(inWaterBelowSea, 2),
                    ["groundBelowSeaKm"] = Math.Round(groundBelowSea, 2)
                }
        };
    }
}
