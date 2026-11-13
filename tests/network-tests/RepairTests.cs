using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

internal static class RepairTests
{
    private static PreparedSegment MakeSegment(
        long way,
        long a,
        long b,
        float x,
        float z,
        float endX,
        float endZ,
        NetworkKind kind = NetworkKind.Road) =>
        new(
            way,
            a,
            b,
            new(x, 50, z),
            new(endX, 50, endZ),
            kind,
            "primary",
            2,
            true,
            false,
            false,
            0)
        { SourceGeometry = new[] { new float3(x, 50, z), new float3(endX, 50, endZ) } };

    public static void Run()
    {
        var log = new ImportLog();
        var flat = Enumerable.Repeat(50f, 128 * 128).ToArray();
        var ground = MakeSegment(1, 1, 2, -100, 0, 100, 0) with { Layer = 2 };
        var bridgeJoin =
            MakeSegment(2, 2, 3, 100.00001f, 0, 200, 0) with { Bridge = true, StartElevation = 8, EndElevation = 8 };

        var joined = NativeEndpointPolicy.Canonicalize(new[] { ground, bridgeJoin }, log);
        Check(
            math.all(joined[0].End == joined[1].Start),
            "Native exact-position keys still split a shared bridge portal");

        var nodeElevations = NativeEndpointPolicy.Elevations(joined);
        Check(
            nodeElevations[NativeEndpointPolicy.Identity(ground, 2)] == 8,
            "Bridge portal composition not shared on the ground course");

        var independent = bridgeJoin with { StartNodeId = 99 };
        Check(
            NativeEndpointPolicy.Canonicalize(new[] { ground, independent }, log)[1].Start.x == independent.Start.x,
            "Nearby unrelated road snapped by native canonicalization");

        Check(bridgeJoin.Start.x == 100.00001f, "Native canonicalization mutated source geometry");
        Check(NetworkStructure.Offset(ground) == 0, "OSM layer became artificial ground-road elevation");

        var lifted = ground with { Start = ground.Start + new float3(0, 3, 0), End = ground.End + new float3(0, 3, 0) };
        var original = (float[])flat.Clone();
        var protectedRing =
            new[] { new float3(-10, 0, -10), new float3(10, 0, -10), new float3(10, 0, 10), new float3(-10, 0, 10) };

        var bed =
            NetworkRoadbed.Prepare(
                flat,
                100,
                1000,
                new[] { lifted },
                new[] { protectedRing },
                Array.Empty<float3[]>(),
                log,
                default);

        Check(
            flat.SequenceEqual(original) && bed.Where((p, i) => p != flat[i]).Any(),
            "Roadbed mutated its input or changed nothing");

        Check(
            bed[64 * 128 + 64] == flat[64 * 128 + 64] && bed[0] == flat[0],
            "Roadbed modified protected water or distant terrain");

        Check(
            bed.Zip(flat, (a, b) => Math.Abs(a - b)).Max() <= 6.001f,
            "Roadbed exceeded bounded cut/fill");

        var structural =
            NetworkRoadbed.Prepare(
                flat,
                100,
                1000,
                new[] { lifted with { Bridge = true } },
                Array.Empty<float3[]>(),
                Array.Empty<float3[]>(),
                log,
                default);

        Check(structural.SequenceEqual(flat), "Bridge raised terrain into a long embankment");
        // A deck sunk in the ground (city elevation data carrying the viaduct)
        // gets the ground lowered beneath it, never raised.
        var sunk =
            NetworkRoadbed.Prepare(
                flat,
                100,
                1000,
                new[] {
                    ground with {
                        Start = ground.Start - new float3(0, 1, 0),
                        End = ground.End - new float3(0, 1, 0),
                        Bridge = true
                        }
                    },
                Array.Empty<float3[]>(),
                Array.Empty<float3[]>(),
                log,
                default);

        Check(
            sunk.Zip(flat, (a, b) => a < b).Any(x => x) && sunk.Zip(flat, (a, b) => a <= b).All(x => x),
            "Ground under a sunk bridge deck was not lowered, or was raised");

        var unsupported =
            ground with { Start = ground.Start + new float3(0, 30, 0), End = ground.End + new float3(0, 30, 0) };

        var higherUnsupported =
            unsupported with
            {
                Start = unsupported.Start + new float3(0, 15, 0),
                End = unsupported.End + new float3(0, 15, 0)
            };

        var unsupportedLog = new ImportLog();
        var boundedBank =
            NetworkRoadbed.Prepare(
                flat,
                100,
                1000,
                new[] { unsupported },
                Array.Empty<float3[]>(),
                Array.Empty<float3[]>(),
                unsupportedLog,
                default);

        var higherBank =
            NetworkRoadbed.Prepare(
                flat,
                100,
                1000,
                new[] { higherUnsupported },
                Array.Empty<float3[]>(),
                Array.Empty<float3[]>(),
                new ImportLog(),
                default);

        Check(
            boundedBank.SequenceEqual(higherBank),
            "Unsupported road height enlarged or flattened earthworks beyond the same cut/fill budget");

        Check(
            unsupportedLog.Metric("Roadbed ground formations beyond support budget") == 1,
            "Bounded terrain edit concealed an unsupported road profile");

        var bankRow =
            Enumerable.Range(64, 20).Select(z => (int)boundedBank[z * 128 + 64] - flat[z * 128 + 64]).ToArray();

        Check(
            bankRow.Zip(bankRow.Skip(1), (first, second) => first >= second).All(v => v) && bankRow.Last() == 0,
            "Bounded earthwork feather is not monotone or has no unchanged toe");

        var deepCut =
            ground with { Start = ground.Start - new float3(0, 30, 0), End = ground.End - new float3(0, 30, 0) };

        var deeperCut =
            deepCut with { Start = deepCut.Start - new float3(0, 15, 0), End = deepCut.End - new float3(0, 15, 0) };

        Check(
            NetworkRoadbed
                .Prepare(
                    flat,
                    100,
                    1000,
                    new[] { deepCut },
                    Array.Empty<float3[]>(),
                    Array.Empty<float3[]>(),
                    new ImportLog(),
                    default)
            .SequenceEqual(
                NetworkRoadbed.Prepare(
                    flat,
                    100,
                    1000,
                    new[] { deeperCut },
                    Array.Empty<float3[]>(),
                    Array.Empty<float3[]>(),
                    new ImportLog(),
                    default)),
            "Unsupported underground height enlarged cut earthworks beyond the same budget");

        var triangularWater = new[] { new float3(-200, 0, -200), new float3(200, 0, -200), new float3(-200, 0, 200) };
        var dryRoad =
            MakeSegment(501, 501, 502, 80, 140, 160, 140) with { Start = new(80, 53, 140), End = new(160, 53, 140) };

        var nearWaterBed =
            NetworkRoadbed.Prepare(
                flat,
                100,
                1000,
                new[] { dryRoad },
                new[] { triangularWater },
                Array.Empty<float3[]>(),
                new ImportLog(),
                default);

        Check(
            nearWaterBed[82 * 128 + 80] > flat[82 * 128 + 80],
            "Dry road inside the water polygon bounding box was incorrectly protected from cut/fill");

        // The game reads the ground under a road bilinearly from the four grid
        // samples around it (7.9 m apart here). Two parallel roads 10 m apart at
        // heights 2 m apart share those samples, and an approach rising 5% to a
        // bridge reads samples under the deck; each must read its own deck.
        float Misread(float[] bed, PreparedSegment segment, float from, float to)
            => Enumerable.Range(0, 41).Select(k =>
        {
            var point = NetworkNormalizer.Evaluate(segment, from + (to - from) * k / 40f);
            return Math.Abs(point.y - NetworkProcessor.Sample(point.x, point.z, bed, 1000));
        }).Max();

        var lower =
            MakeSegment(511, 511, 512, -150, 0, 150, 0) with { Start = new(-150, 51, 0), End = new(150, 51, 0) };

        var upper =
            MakeSegment(513, 513, 514, -150, 10, 150, 10) with { Start = new(-150, 53, 10), End = new(150, 53, 10) };

        var sharedBed =
            NetworkRoadbed.Prepare(
                flat,
                100,
                1000,
                new[] { lower, upper },
                Array.Empty<float3[]>(),
                Array.Empty<float3[]>(),
                new ImportLog(),
                default);

        var sharedMisread = Math.Max(Misread(sharedBed, lower, .3f, .7f), Misread(sharedBed, upper, .3f, .7f));
        Check(
            sharedMisread < .3f,
            $"Roads sharing terrain samples read the ground {sharedMisread:F2} m off their decks");

        var approach = MakeSegment(515, 515, 516, -200, 0, 0, 0) with { Start = new(-200, 46, 0), End = new(0, 56, 0) };
        var deck =
            MakeSegment(
                517,
                516,
                518,
                0,
                0,
                100,
                0) with
            { Start = new(0, 56, 0), End = new(100, 61, 0), Bridge = true };

        var abutmentBed =
            NetworkRoadbed.Prepare(
                flat,
                100,
                1000,
                new[] { approach, deck },
                Array.Empty<float3[]>(),
                Array.Empty<float3[]>(),
                new ImportLog(),
                default);

        var abutmentMisread = Misread(abutmentBed, approach, .95f, 1);
        Check(
            abutmentMisread < .05f,
            $"The ground under an approach's last metres before a bridge is {abutmentMisread:F2} m off its deck");

        try
        {
            NetworkContinuity.Validate(new[] { ground }, Array.Empty<PreparedSegment>(), log, default);
            throw new Exception("Lost route passed continuity gate");
        }
        catch (InvalidOperationException) { }

        Check(
            log.Lines.Any(text => text.Contains("continuity not verified")),
            "Missing continuity must be diagnosed and blocked");

        var rail = MakeSegment(8, 20, 21, -100, 0, 100, 0, NetworkKind.Rail);

        try
        {
            NetworkContinuity.Validate(
                new[] { rail, rail with { WayId = 9, StartNodeId = 22, EndNodeId = 23 } },
                new[] { rail, rail with { WayId = 9, StartNodeId = 22, EndNodeId = 23 } },
                log,
                default);

            throw new Exception("Duplicate rails passed continuity gate");
        }
        catch (InvalidOperationException) { }

        Check(
            log.Lines.Any(text => text.Contains("Rail overlap remains")),
            "Unresolved rail overlap must be diagnosed and blocked");

        RecoveryTests.Run();

        using (var cancellation = new CancellationTokenSource())
        {
            cancellation.Cancel();

            try
            {
                NetworkRoadbed.Prepare(
                    flat,
                    100,
                    1000,
                    new[] { lifted },
                    Array.Empty<float3[]>(),
                    Array.Empty<float3[]>(),
                    log,
                    cancellation.Token);

                throw new Exception("Roadbed ignored cancellation");
            }
            catch (OperationCanceledException) { }
        }

        Console.WriteLine(
            "PASS: vanilla-width separation, all four map boundaries, safe curve rollback with shared junctions/no deletions/immutable input, layer semantics, protected roadbed, connected rail portals and overlap/continuity gates.");
    }
}
