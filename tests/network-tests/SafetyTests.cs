using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

static class SafetyTests
{
    static PreparedSegment MakeSegment(
        long way,
        long a,
        long b,
        float3 start,
        float3 end,
        bool bridge = false,
        int layer = 0,
        NetworkKind kind = NetworkKind.Road) =>
        new(
            way,
            a,
            b,
            start,
            end,
            kind,
            kind == NetworkKind.Rail ? "rail" : "motorway",
            kind == NetworkKind.Rail ? 2 : 3,
            kind == NetworkKind.Road,
            bridge,
            false,
            layer)
        { NativeWidth = 12, SourceGeometry = new[] { start, end } };

    public static void Run()
    {
        var raster = Enumerable.Repeat(100f, 256).ToArray();
        var offset =
            new[] {
                MakeSegment(10, 10, 11, new(-200, 100, 0), new(200, 100, 0), true, 2),
                MakeSegment(11, 12, 13, new(-200, 100, 10), new(200, 100, 10), true, 1)
                };

        var log = new ImportLog();

        try
        {
            NetworkGeometrySafety.Validate(offset, log, default);
            throw new Exception("Colliding prepared map passed final audit");
        }
        catch (InvalidOperationException exception)
        {
            Check(exception.Message.Contains("prevented application"), "Safety gate lost useful failure message");
        }

        var sky = new[] { MakeSegment(30, 30, 31, new(-10, 100, 0), new(10, 190, 0)) };

        try
        {
            NetworkGeometrySafety.Validate(sky, new ImportLog(), default);
            throw new Exception("Steep profile passed audit");
        }
        catch (InvalidOperationException) { }

        var tied =
            new[] {
                MakeSegment(40, 40, 41, new(-20, 100, 0), new(0, 100, 0), true),
                MakeSegment(41, 41, 42, new(0, 100, 0), new(20, 100, 0))
                };

        NetworkGeometrySafety.Validate(tied, new ImportLog(), default);
        var loop =
            MakeSegment(
                45,
                45,
                46,
                new(0, 100, 0),
                new(100, 100, 0),
                true,
                1) with
            { Control1 = new(500, 100, 500), Control2 = new(500, 100, -500) };

        var joinedStraight = MakeSegment(46, 45, 47, new(0, 100, 0), new(800, 100, 0), true, 1);

        try
        {
            NetworkGeometrySafety.Validate(new[] { loop, joinedStraight }, new ImportLog(), default);
            throw new Exception("Remote crossing bypassed clearance because the curves share a far-away endpoint");
        }
        catch (InvalidOperationException) { }

        var tunnels =
            new[] {
                MakeSegment(
                    60,
                    60,
                    61,
                    new(-200, 100, 0),
                    new(0, 100, 0),
                    kind: NetworkKind.Rail) with { Tunnel = true },
                MakeSegment(61, 61, 62, new(0, 100, 0), new(200, 100, 0), kind: NetworkKind.Rail) with { Tunnel = true }
                };

        // 20 m under the 100 m ground clears the 15 m cover the audit asks of an interior
        // bore; once the ground drops to 85 m the same bore has only 5 m.
        var buried =
            tunnels
                .Select(
                    segment => segment with
                    {
                        Start = segment.Start - new float3(0, 20, 0),
                        End = segment.End - new float3(0, 20, 0)
                    })
                .ToArray();

        NetworkGeometrySafety.ValidateTunnelCover(buried, raster, 200, 1000, default);

        try
        {
            NetworkGeometrySafety.ValidateTunnelCover(tunnels, raster, 200, 1000, default);
            throw new Exception("Surface-level tunnel passed final terrain audit");
        }
        catch (InvalidOperationException) { }

        var surfaced = (float[])raster.Clone();
        Array.Fill(surfaced, 85f);

        try
        {
            NetworkGeometrySafety.ValidateTunnelCover(buried, surfaced, 200, 1000, default);
            throw new Exception("Later terrain carving exposed a tunnel without detection");
        }
        catch (InvalidOperationException) { }

        var outside =
            new[] {
                MakeSegment(70, 70, 71, new(450, 100, 0), new(480, 100, 0)) with { EndCurveOffset = new(25, 0, 0) }
                };

        try
        {
            NetworkGeometrySafety.ValidateEnvelope(outside, 1000, 200, default);
            throw new Exception("Offset native curve outside the map passed audit");
        }
        catch (InvalidOperationException) { }

        Console.WriteLine(
            "PASS: final audit rejects colliding decks, steep profiles, remote crossings, exposed tunnels and off-map curves, and accepts native junction exclusions.");
    }
}
