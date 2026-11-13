using ImportEarthLocation.Processing;
using Unity.Mathematics;

// Median terrain profile across mapped beaches and across coast with no beach, on the terrain the game received.
public static class Shore
{
    public static void Run(string map, float sea)
    {
        var capture = new Cap(map);
        var t =
            capture.Heights(
                capture.Has("native-observed-heightmap.u16")
                    ? "native-observed-heightmap.u16"
                    : "final-design-heightmap.u16");

        var polys = capture.Manifest<GeographicPolygon>(capture.InputValue("waterPolygonsManifest")).ToArray();
        var beaches =
            polys
                .Where(
                    polygon => polygon.Tags.TryGetValue("natural", out var text)
                        && text == "beach"
                        && polygon.Outer.Count > 0)
                .ToArray();

        Console.WriteLine($"{map}: beaches {beaches.Length}; surface {sea}");
        var geo = Waters.FindGeography(capture);
        var dataset = Waters.LoadCache(geo!)!;
        var coast = new List<(float3 A, float3 B)>();

        foreach (var way in dataset.Ways.Values)
            if (way.Tags.TryGetValue("natural", out var otherText) && otherText == "coastline")
            {
                var points =
                    way.NodeIds
                        .Where(dataset.Nodes.ContainsKey)
                        .Select(id => capture.Local(dataset.Nodes[id].Latitude, dataset.Nodes[id].Longitude))
                        .ToArray();

                for (var i = 1; i < points.Length; i++)
                    coast.Add((points[i - 1], points[i]));
            }

        Console.WriteLine($"coast segments {coast.Count}");
        var half = (float)capture.Width / 2;

        bool Near(float3 point, float r)
            => beaches.Any(
                polygon => math.distance(polygon.Outer[0][0].xz, point.xz) < r + 800
                    && polygon.EdgeDistance(point.xz) < r);

        // Sample along the coast every 300 m; normal points to the sea (right of travel).
        var distances = new[] { -60, -45, -30, -20, -14, -7, 0, 7, 14, 21, 28, 42, 56, 84, 112, 168, 224, 336 };
        var beachRows = new List<float[]>();
        var plainRows = new List<float[]>();
        var walked = 0f;

        foreach (var (vectorA, vectorB) in coast)
        {
            var length = math.distance(vectorA.xz, vectorB.xz);

            if (length < 1)
                continue;

            var direction = math.normalizesafe(vectorB - vectorA);

            for (var s = 0f; s < length; s += 25)
            {
                walked += 25;

                if (walked < 300)
                    continue;

                walked = 0;
                var point = vectorA + direction * s;

                if (Math.Abs(point.x) > half - 500 || Math.Abs(point.z) > half - 500)
                    continue;

                var normal =
                    new float3(direction.z, 0, -direction.x); // right of travel in x,z = sea side for OSM coastlines

                var row =
                    distances
                        .Select(k => capture.HeightAt(t, point.x + normal.x * k, point.z + normal.z * k) - sea)
                        .ToArray();

                (Near(point, 40) ? beachRows : plainRows).Add(row);
            }
        }

        void Print(string label, List<float[]> rows)
        {
            Console.WriteLine(
                $"{label}: {rows.Count} profiles; sea side = positive distance. height above water, median (p10..p90):");

            for (var k = 0; k < distances.Length; k++)
            {
                var v = rows.Select(r => r[k]).OrderBy(x => x).ToArray();

                if (v.Length == 0)
                    break;

                Console.WriteLine(
                    $"  {distances[k],5} m: {v[v.Length / 2],7:F2} ({v[v.Length / 10],7:F2} .. {v[v.Length * 9 / 10],7:F2})");
            }

            // Width of the shallow band: first distance at which ground is 1, 2, 4 m under water.
            foreach (var depth in new[] { .5f, 1f, 2f, 4f })
            {
                var w = rows.Select(r =>
{
    for (var k = 6; k < distances.Length; k++)
        if (r[k] < -depth)
            return (float)distances[k];

    return 999f;
}).OrderBy(x => x).ToArray();

                if (w.Length > 0)
                    Console.WriteLine(
                        $"  deeper than {depth} m at: median {w[w.Length / 2]} m (p10 {w[w.Length / 10]}, p90 {w[w.Length * 9 / 10]})");
            }

            var slopes = rows.Select(r =>
{
    var i0 = Array.IndexOf(distances, -14);
    var i1 = Array.IndexOf(distances, 14);

    return Math.Abs(r[i0] - r[i1]) / 28f;
}).OrderBy(x => x).ToArray();

            if (slopes.Length > 0)
                Console.WriteLine(
                    $"  slope across the waterline (±14 m): median {slopes[slopes.Length / 2]:P1}, p90 {slopes[slopes.Length * 9 / 10]:P1}");
        }

        Print("mapped-beach coast", beachRows);
        Print("coast with no beach", plainRows);
    }
}
