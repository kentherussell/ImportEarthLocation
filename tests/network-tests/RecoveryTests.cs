using System.Text;
using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

internal static class RecoveryTests
{
    public static void Run()
    {
        var log = new ImportLog();
        Check(
            OsmRecovery.Run<int>("test", () => throw new InvalidOperationException("bad OSM"), 17, log, default) == 17,
            "Data-stage fallback failed");

        try
        {
            OsmRecovery.Run<int>("test", () => throw new OutOfMemoryException(), 17, log, default);
            throw new Exception("Memory failure swallowed");
        }
        catch (OutOfMemoryException) { }

        using (var cancellation = new CancellationTokenSource())
        {
            cancellation.Cancel();

            try
            {
                OsmRecovery.Run("test", () => 1, 17, log, cancellation.Token);
                throw new Exception("Cancellation swallowed");
            }
            catch (OperationCanceledException) { }
        }

        PreparedSegment MakeSegment(long way, long a, long b, float3 start, float3 end)
            => new(way, a, b, start, end, NetworkKind.Road, "primary", 2, false, false, false, 0);

        var original =
            new[] {
                MakeSegment(1, 1, 2, new(-600, 50, 0), new(0, 50, 0)) with { Control1 = new float3(float.NaN, 0, 0) },
                MakeSegment(2, 2, 3, new(0, 55, 0), new(100, 50, 0)),
                MakeSegment(
                    3,
                    4,
                    5,
                    new(float.NaN, 0, 0),
                    new(
                        10,
                        50,
                        100)) with { SourceGeometry = new[] { new float3(-50, 50, 100), new float3(10, 50, 100) } }
                };

        var repaired = OsmRecovery.RepairGeometry(original, 1000, log, default);
        Check(
            repaired.Count == 3
                && repaired[0].Start.x == -500
                && !repaired[0].Control1.HasValue
                && repaired[2].Start.x == -50,
            "Local geometry repair omitted a recoverable original anchor");

        Check(
            math.distance(repaired[0].End, repaired[1].Start) < .001f && original[1].Start.y == 55,
            "Shared endpoint repair broke topology/mutated source");

        var missing = MakeSegment(4, 2, 6, new(float.NaN, 0, 0), new(40, 50, -100));
        Check(
            OsmRecovery.RepairGeometry(new[] { missing, original[0] }, 1000, log, default).Count == 2,
            "Healthy later incident node cannot recover a missing shared anchor");

        try
        {
            OsmRecovery.RepairGeometry(new[] { missing with { StartNodeId = 99 } }, 1000, log, default);
            throw new Exception("Unknown malformed anchor silently omitted");
        }
        catch (InvalidOperationException) { }

        var root = Path.Combine(Path.GetTempPath(), "iel-recovery-" + Guid.NewGuid().ToString("N"));
        var valid =
            Encoding.UTF8.GetBytes(
                "<osm><node id='1' lat='38' lon='-77'/><node id='2' lat='38.001' lon='-77'/><way id='4'><nd ref='1'/><nd ref='2'/><tag k='highway' v='primary'/></way></osm>");

        var partial =
            OsmRecovery.Parse(
                new[] { valid, Encoding.UTF8.GetBytes("<osm><remark>timeout</remark></osm>") },
                root,
                "test",
                log,
                default);

        Check(
            partial.Nodes.Count == 2
                && partial.Ways.Count == 1
                && log.Lines.Any(text => text.Contains("coverageComplete=false")),
            "Unreadable tile discarded healthy OSM coverage");

        Check(!Directory.Exists(root), "Partial parsing should not persist an incomplete full-coverage cache");
        Console.WriteLine(
            "PASS: OSM recovery, partial XML coverage, immutable bounded geometry/shared joins, cancellation and memory failure propagation.");
    }
}
