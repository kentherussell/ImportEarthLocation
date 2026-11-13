using System.Reflection;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

public static class Waters
{
    public static OsmDataset? LoadCache(string path)
    {
        var type = typeof(OsmParsedCache);
        var info = type.GetMethod("LoadSnapshot", BindingFlags.Static | BindingFlags.NonPublic | BindingFlags.Public)!;

        return (OsmDataset?)info.Invoke(null, new object[] { path, default(CancellationToken) });
    }

    public static string CacheDir
        => Path.Combine(
            Environment.GetEnvironmentVariable("USERPROFILE")!,
            @"AppData\LocalLow\Colossal Order\Cities Skylines II\ModsData\ImportEarthLocation\cache\parsed-osm");

    // Find the geography snapshot for a capture: the parsed-osm file holding coastline/water ways near the map centre.
    public static string? FindGeography(Cap capture)
    {
        foreach (var info in new DirectoryInfo(CacheDir)
            .GetFiles("*.bin")
            .OrderByDescending(otherInfo => otherInfo.LastWriteTime)
            .Take(60))
        {
            if (info.Length < 1_000_000)
                continue;

            try
            {
                var dataset = LoadCache(info.FullName)!;

                if (dataset.Ways.Count == 0)
                    continue;

                var near =
                    dataset.Ways.Values
                        .Where(
                            way => way.Tags.TryGetValue("natural", out var text) && text == "water"
                                || way.Tags.TryGetValue("natural", out var otherText) && otherText == "coastline"
                                || way.Tags.ContainsKey("waterway"))
                        .Take(4000)
                        .Count(
                            way => way.NodeIds.Length > 0
                                && dataset.Nodes.TryGetValue(way.NodeIds[0], out var node)
                                && Math.Abs(node.Latitude - capture.Lat) < .25
                                && Math.Abs(node.Longitude - capture.Lon) < .35);

                if (near > 100)
                    return info.FullName;
            }
            catch { }
        }

        return null;
    }

    public static void Stats(string map)
    {
        var capture = new Cap(map);
        var polys = capture.Manifest<GeographicPolygon>(capture.InputValue("waterPolygonsManifest")).ToArray();
        var water =
            polys.Where(polygon => polygon.Tags.TryGetValue("natural", out var text) && text == "water").ToArray();

        Console.WriteLine($"{map}: {polys.Length} polygons; natural=water {water.Length}");

        foreach (var group in water
            .GroupBy(polygon => polygon.Tags.TryGetValue("water", out var otherText) ? otherText : "(none)")
            .OrderByDescending(otherGroup => otherGroup.Count()))
            Console.WriteLine($"  water={group.Key}: {group.Count()}");

        var seaish =
            water
                .Where(
                    polygon => polygon.Tags.TryGetValue("water", out var secondText)
                        && secondText is "harbour" or "dock" or "marina" or "port" or "basin" or "lagoon" or "sea" or "bay"
                        || polygon.Tags.ContainsKey("harbour")
                        || polygon.Tags.Any(entry => entry.Key.StartsWith("seamark")))
                .OrderByDescending(polygon => polygon.Outer.SelectMany(points => points).Count())
                .Take(40);

        foreach (var polygon in seaish)
        {
            var points = polygon.Outer.SelectMany(otherPoints => otherPoints).ToArray();
            var low = points.Select(point => point.xz).Aggregate(math.min);
            var high = points.Select(point => point.xz).Aggregate(math.max);
            var item = capture.ToGeographic((low.x + high.x) / 2, (low.y + high.y) / 2);
            Console.WriteLine(
                $"  {polygon.Identity} size {high.x - low.x:F0}x{high.y - low.y:F0} at {item.lat:F4},{item.lon:F4} water={(polygon.Tags.TryGetValue("water", out var otherText) ? otherText : "-")} name={(polygon.Tags.TryGetValue("name", out var nm) ? nm : "-")}");
        }
    }

    // Everything mapped around one point: coastline ways, waterways, barriers, and a terrain cross-section.
    public static void Probe(string map, double lat, double lon, double radius, double lat2, double lon2)
    {
        var capture = new Cap(map);
        var otherPoint = capture.Local(lat, lon);
        var path = FindGeography(capture);
        Console.WriteLine("geography cache: " + path);

        if (path != null)
        {
            var dataset = LoadCache(path)!;
            Console.WriteLine(
                $"geography ways {dataset.Ways.Count}, coastline {dataset.Ways.Values.Count(way => way.Tags.TryGetValue("natural", out var text) && text == "coastline")}");

            foreach (var way in dataset.Ways.Values)
            {
                var points =
                    way.NodeIds
                        .Where(dataset.Nodes.ContainsKey)
                        .Select(id => capture.Local(dataset.Nodes[id].Latitude, dataset.Nodes[id].Longitude))
                        .ToArray();

                if (points.Length < 2)
                    continue;

                var close = false;

                for (var j = 1; j < points.Length; j++)
                    if (NetworkNormalizer.DistanceToSegment(otherPoint, points[j - 1], points[j]) < radius)
                    {
                        close = true;
                        break;
                    }

                if (!close)
                    continue;

                if (!way.Tags.Any(
                    entry => entry.Key is "natural" or "waterway" or "water" or "man_made" or "barrier" or "harbour" or "seamark:type"))
                    continue;

                Console.WriteLine(
                    $" way {way.Id} nodes {points.Length} closed={way.NodeIds[0] == way.NodeIds[^1]}: {string.Join(",", way.Tags.Where(entry => entry.Key is "natural" or "waterway" or "water" or "man_made" or "barrier" or "harbour" or "name" or "seamark:type").Select(entry => entry.Key + "=" + entry.Value))}");

                if (way.Tags.TryGetValue("natural", out var otherText) && otherText == "coastline")
                {
                    var near = new List<string>();

                    for (var j = 0; j < points.Length; j++)
                        if (math.distance(points[j].xz, otherPoint.xz) < radius * 1.5)
                        {
                            var item = capture.ToGeographic(points[j].x, points[j].z);
                            near.Add($"{item.lat:F5},{item.lon:F5}");
                        }

                    Console.WriteLine("    coast nodes near: " + string.Join(" ", near.Take(30)));
                }
            }

            foreach (var relation in dataset.Relations)
            {
                if (!relation.Tags.Any(entry => entry.Key is "natural" or "waterway" or "water"))
                    continue;

                var succeeded = false;

                foreach (var item in relation.Members)
                    if (dataset.Ways.TryGetValue(item.wayId, out var way))
                        foreach (var id in way.NodeIds)
                            if (dataset.Nodes.TryGetValue(id, out var node))
                            {
                                var point = capture.Local(node.Latitude, node.Longitude);

                                if (math.distance(point.xz, otherPoint.xz) < radius)
                                {
                                    succeeded = true;
                                    break;
                                }
                            }

                if (succeeded)
                    Console.WriteLine(
                        $" relation {relation.Id}: {string.Join(",", relation.Tags.Select(entry => entry.Key + "=" + entry.Value).Take(8))}");
            }
        }

        var vectorA = otherPoint;
        var vectorB = capture.Local(lat2, lon2);

        foreach (var name in new[] { capture.InputValue("terrainFile"), "native-observed-heightmap.u16" })
        {
            var t = capture.Heights(name);
            Console.WriteLine("-- " + name + " section:");
            var line = new System.Text.StringBuilder();
            var length = math.distance(vectorA.xz, vectorB.xz);

            for (var s = 0f; s <= length; s += 14)
            {
                var point = math.lerp(vectorA, vectorB, s / length);
                line.Append($"{capture.HeightAt(t, point.x, point.z):F1} ");
            }

            Console.WriteLine(line);
        }
    }
}
