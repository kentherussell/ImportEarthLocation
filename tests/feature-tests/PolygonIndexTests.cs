using ImportEarthLocation.Processing;
using Unity.Mathematics;

// The indexed polygon queries used by beach paint and beach terrain (report 114 S10) give
// the same answers as the plain ones: containment exactly, and the edge distance
// bit-identically wherever it is within the limit, +∞ beyond it.
static class PolygonIndexTests
{
    public static void Run()
    {
        var random = new System.Random(114);

        float3[] Ring(float2 centre, float radius, int count, float wobble)
        {
            var ring = new float3[count];

            for (var i = 0; i < count; i++)
            {
                var angle = i * 2 * Math.PI / count;
                var r = radius * (1 + wobble * (float)(random.NextDouble() - .5));
                ring[i] = new float3(centre.x + r * (float)Math.Cos(angle), 0, centre.y + r * (float)Math.Sin(angle));
            }

            return ring;
        }

        var polygons = new[]
        {
            new GeographicPolygon(1, new Dictionary<string, string>(), new() { Ring(new float2(0, 0), 400, 40, .3f) }, new()),
            new GeographicPolygon(
                2,
                new Dictionary<string, string>(),
                new() { Ring(new float2(1000, -500), 900, 1500, .2f) },
                new() { Ring(new float2(1000, -500), 200, 300, .2f) }),
            new GeographicPolygon(
                3,
                new Dictionary<string, string>(),
                new() { new[] { new float3(-5000, 0, -20), new float3(5000, 0, -20), new float3(5000, 0, 20), new float3(-5000, 0, 20) } },
                new())
        };

        var checkedPoints = 0;

        foreach (var polygon in polygons)
            foreach (var limit in new[] { 20f, 160f, 421f })
                for (var k = 0; k < 4000; k++)
                {
                    var point =
                        new float3(
                            (float)(random.NextDouble() * 6000 - 2500),
                            0,
                            (float)(random.NextDouble() * 3000 - 1800));

                    if (polygon.Contains(point) != polygon.ContainsIndexed(point))
                        throw new Exception($"Indexed containment differs for polygon {polygon.Id} at {point}");

                    var exact = polygon.EdgeDistance(point.xz);
                    var indexed = polygon.EdgeDistance(point.xz, limit);

                    if (exact <= limit
                        ? BitConverter.SingleToInt32Bits(exact) != BitConverter.SingleToInt32Bits(indexed)
                        : !float.IsPositiveInfinity(indexed))
                        throw new Exception($"Indexed edge distance differs for polygon {polygon.Id} at {point}: {exact} vs {indexed}");

                    checkedPoints++;
                }

        Console.WriteLine(
            $"PASS: indexed beach polygons match containment and bit-identical edge distance within the limit ({checkedPoints} points).");
    }
}
