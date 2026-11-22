using System.Diagnostics;
using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;
using Random = System.Random;

// NetworkRoadbed.BuildLocalPatch and Prepare against the implementations they replaced, kept
// below as the reference. On a synthetic map the game's size (4,096 x 4,096 cells, 57,344 m,
// hilly) with a few thousand sections, lakes, a river, streams, a dyke and later water edits,
// every local patch must match the reference bit for bit (rectangle, heights, crests, published
// cells and their order, log lines, counts and findings), and so must the whole-map roadbed.
static class LocalPatchEquivalence
{
    private const int Resolution = 4096;
    private const double MapWidth = 57344;
    private const float Scale = 3000;
    private const float Half = (float)(MapWidth / 2);

    private static int Main()
    {
        try
        {
            Run();
        }
        catch (Exception exception)
        {
            Console.WriteLine("FAIL: " + exception.Message);

            return 1;
        }

        return 0;
    }

    private static void Run()
    {
        var clock = Stopwatch.StartNew();
        var baseline = Relief();
        var plan = Network(baseline);
        var areas = Areas();
        var lines = Lines();
        var crests = Crests(baseline);
        var edits = Edits();
        var current = Current(baseline);

        var prepared =
            new PreparedImport
            {
                Request = new ImportRequest(0, 0, new MapSizeProfile("test", "Test", MapWidth, false), false),
                Heights = baseline,
                RoadbedBaseline = baseline,
                DefenceCrests = crests,
                RoadbedLaterEdits = edits,
                RoadbedProtectedAreas = areas,
                RoadbedProtectedLines = lines,
                HeightScaleMeters = Scale
            };

        var bare =
            new PreparedImport
            {
                Request = prepared.Request,
                Heights = baseline,
                RoadbedBaseline = baseline,
                RoadbedProtectedAreas = areas,
                RoadbedProtectedLines = lines,
                HeightScaleMeters = Scale
            };

        Console.WriteLine(
            $"Synthetic map built in {clock.Elapsed.TotalSeconds:0.0} s: {plan.Count} sections, {areas.Length} areas, "
                + $"{lines.Length} lines.");

        var scattered = Unit(plan, segment => Inside(segment, -26000, -6000, 26000, 24000), 40, 7);
        var corridor = plan.Where(segment => segment.WayId is 21 or 22).ToArray();
        var edge = plan.Where(segment => segment.WayId is 1 or 2).ToArray();
        var small = Unit(plan, segment => Inside(segment, 4000, 4000, 9000, 9000), 40, 3);

        // Another design's crests over the same baseline: the count of the baseline's cells
        // below their crest outside the region must be taken afresh, not from the last design.
        var raised = (float[])crests.Clone();

        for (var i = 0; i < raised.Length; i += 7)
            raised[i] = baseline[i] + 2;

        var otherCrests =
            new PreparedImport
            {
                Request = prepared.Request,
                Heights = baseline,
                RoadbedBaseline = baseline,
                DefenceCrests = raised,
                RoadbedLaterEdits = edits,
                RoadbedProtectedAreas = areas,
                RoadbedProtectedLines = lines,
                HeightScaleMeters = Scale
            };

        var cases =
            new (string Name, PreparedImport Prepared, PreparedSegment[] Unit, bool Moved, bool Diagnostics)[]
            {
                ("scattered over half the map", prepared, scattered, true, false),
                ("one corridor, several kilometres", prepared, corridor, true, false),
                ("along the map edge", prepared, edge, true, false),
                ("refit that changes nothing", prepared, corridor, false, false),
                ("diagnostics on", prepared, corridor, true, true),
                ("no crests or later edits", bare, small, true, false),
                ("another design's crests", otherCrests, corridor, true, false),
                ("first design's crests again", prepared, small, true, false)
            };

        // Both implementations once before timing, so neither pays the JIT.
        Compare(cases[1].Name, cases[1].Prepared, current, plan, cases[1].Unit, true, false, timed: false);

        foreach (var (name, casePrepared, unit, moved, diagnostics) in cases)
            Compare(name, casePrepared, current, plan, unit, moved, diagnostics, timed: true);

        CompareWholeMap(baseline, plan, areas, lines);
        CompareRefitSequence(prepared, current, Network(baseline, 2400));

        Console.WriteLine(
            "PASS: local roadbed patches identical to the reference bit for bit (rectangle, heights, crests, "
                + "published cells, log), and the whole-map roadbed too.");
    }

    private static void Compare(
        string name,
        PreparedImport prepared,
        float[] current,
        IReadOnlyList<PreparedSegment> network,
        PreparedSegment[] unit,
        bool moved,
        bool diagnostics,
        bool timed)
    {
        const int Start = 1500;

        var members = new HashSet<PreparedSegment>(unit);
        var plan = network.Where(segment => !members.Contains(segment)).ToList();
        plan.InsertRange(Start, unit);
        var revision = unit.Select((segment, index) => moved ? Moved(segment, index) : segment with { }).ToArray();

        // The fastest of three alternating runs of each, so allocation and caches weigh alike.
        var referenceLog = new ImportLog();
        var newLog = new ImportLog();
        NetworkRoadbed.Patch reference = null!;
        NetworkRoadbed.Patch patch = null!;
        var referenceSeconds = double.MaxValue;
        var newSeconds = double.MaxValue;

        for (var run = 0; run < (timed ? 3 : 1); run++)
        {
            referenceLog = new ImportLog { CaptureRecords = diagnostics };
            newLog = new ImportLog { CaptureRecords = diagnostics };

            GC.Collect();
            var clock = Stopwatch.StartNew();
            reference = ReferenceBuildLocalPatch(prepared, current, plan, Start, revision, referenceLog, default);
            referenceSeconds = Math.Min(referenceSeconds, clock.Elapsed.TotalSeconds);

            GC.Collect();
            clock.Restart();
            patch = NetworkRoadbed.BuildLocalPatch(prepared, current, plan, Start, revision, newLog, default);
            newSeconds = Math.Min(newSeconds, clock.Elapsed.TotalSeconds);
        }

        Check(
            patch.X == reference.X
                && patch.Z == reference.Z
                && patch.Width == reference.Width
                && patch.Height == reference.Height
                && patch.Resolution == reference.Resolution
                && patch.MapWidth == reference.MapWidth,
            name + ": rectangle differs");

        Same(reference.Samples, patch.Samples, name + ": heights");
        Same(reference.Crests, patch.Crests, name + ": crests");
        Check(
            reference.Published == null == (patch.Published == null)
                && (reference.Published == null || reference.Published.SequenceEqual(patch.Published!)),
            name + ": published cells differ");

        SameLog(referenceLog, newLog, name);

        var referenceGround = (float[])current.Clone();
        var newGround = (float[])current.Clone();
        reference.CopyTo(referenceGround);
        patch.CopyTo(newGround);
        Same(referenceGround, newGround, name + ": ground after CopyTo");

        if (timed)
            Console.WriteLine(
                $"  {name}: {reference.Width} x {reference.Height} cells, "
                    + $"{reference.Published?.Length ?? 0} published; "
                    + $"reference {referenceSeconds:0.000} s, new {newSeconds:0.000} s");
    }

    private static void CompareWholeMap(
        float[] baseline,
        IReadOnlyList<PreparedSegment> plan,
        float3[][] areas,
        float3[][] lines)
    {
        var referenceLog = new ImportLog();
        var newLog = new ImportLog();

        GC.Collect();
        var clock = Stopwatch.StartNew();
        var reference =
            ReferencePrepare(baseline, Scale, MapWidth, plan, areas, lines, referenceLog, default, waterSurface: 1300);

        var referenceSeconds = clock.Elapsed.TotalSeconds;

        GC.Collect();
        clock.Restart();
        var result =
            NetworkRoadbed.Prepare(baseline, Scale, MapWidth, plan, areas, lines, newLog, default, waterSurface: 1300);

        var newSeconds = clock.Elapsed.TotalSeconds;

        Same(reference, result, "whole-map roadbed");
        SameLog(referenceLog, newLog, "whole-map roadbed");
        Console.WriteLine($"  whole-map roadbed: reference {referenceSeconds:0.000} s, new {newSeconds:0.000} s");

        // Prepare with a dirty region still returns the whole grid, the source outside it.
        var region = new int4(900, 1300, 2600, 2900);
        var regionReferenceLog = new ImportLog();
        var regionLog = new ImportLog();
        var regionReference =
            ReferencePrepare(baseline, Scale, MapWidth, plan, areas, lines, regionReferenceLog, default, region, 1300);

        var regionResult =
            NetworkRoadbed.Prepare(baseline, Scale, MapWidth, plan, areas, lines, regionLog, default, region, 1300);

        Same(regionReference, regionResult, "roadbed in a dirty region");
        SameLog(regionReferenceLog, regionLog, "roadbed in a dirty region");
        Console.WriteLine("  roadbed in a dirty region: identical");
    }

    // An import's refits one after another on a plan the size of Kathmandu's (about 53,000
    // sections), with one cache across them, as ImportApplySystem keeps it. Between calls the
    // plan changes as a placement changes it: an accepted refit's revision replaces its unit
    // (ReplaceUnissued), a rejected one leaves it, and other sections are replaced elsewhere.
    // One unit is refitted twice in a row, with diagnostics on both times, so the support
    // check's whole-map ground must come back clean after each. Every call must match the
    // reference called fresh, and so must the same call made without a cache.
    private static void CompareRefitSequence(PreparedImport prepared, float[] current, List<PreparedSegment> plan)
    {
        var cache = new NetworkRoadbed.LocalPatchCache();
        var random = new Random(31);
        var start = 1200;

        Console.WriteLine($"  refit sequence on {plan.Count} sections, one cache:");

        for (var call = 0; call < 10; call++)
        {
            var name = $"refit sequence call {call + 1}";
            var count = 20 + call % 3 * 10;
            var diagnostics = call is 4 or 5;
            var unit = plan.GetRange(start, count).ToArray();
            var revision = unit.Select((segment, index) => Moved(segment, index + call)).ToArray();
            // ImportApplySystem hands each refit a copy of the plan as it stands.
            var snapshot = plan.ToArray();

            var referenceLog = new ImportLog { CaptureRecords = diagnostics };
            GC.Collect();
            var clock = Stopwatch.StartNew();
            var reference = ReferenceBuildLocalPatch(prepared, current, snapshot, start, revision, referenceLog, default);
            var referenceSeconds = clock.Elapsed.TotalSeconds;

            var freshLog = new ImportLog { CaptureRecords = diagnostics };
            GC.Collect();
            clock.Restart();
            var fresh = NetworkRoadbed.BuildLocalPatch(prepared, current, snapshot, start, revision, freshLog, default);
            var freshSeconds = clock.Elapsed.TotalSeconds;

            var cachedLog = new ImportLog { CaptureRecords = diagnostics };
            GC.Collect();
            clock.Restart();
            var cached =
                NetworkRoadbed.BuildLocalPatch(prepared, current, snapshot, start, revision, cachedLog, default, cache);

            var cachedSeconds = clock.Elapsed.TotalSeconds;

            SamePatch(reference, fresh, name + " without a cache");
            SameLog(referenceLog, freshLog, name + " without a cache");
            SamePatch(reference, cached, name + " with the cache");
            SameLog(referenceLog, cachedLog, name + " with the cache");

            static double Part(ImportLog log, string part)
                => log.Durations.TryGetValue("Local refit patch " + part, out var seconds) ? seconds : 0;

            Console.WriteLine(
                $"    call {call + 1}: {reference.Width} x {reference.Height} cells, {count} sections"
                    + (diagnostics ? ", diagnostics" : "")
                    + $"; reference {referenceSeconds:0.000} s, new without cache {freshSeconds:0.000} s "
                    + $"(neighbours {Part(freshLog, "neighbours"):0.0000}, "
                    + $"protection {Part(freshLog, "protection"):0.0000}, "
                    + $"support check {Part(freshLog, "support check"):0.000}), "
                    + $"with cache {cachedSeconds:0.000} s "
                    + $"(neighbours {Part(cachedLog, "neighbours"):0.0000}, "
                    + $"protection {Part(cachedLog, "protection"):0.0000}, "
                    + $"support check {Part(cachedLog, "support check"):0.000})");

            // Every refit but the third is accepted; the fifth and sixth refit the same unit.
            if (call != 2)
                for (var index = 0; index < count; index++)
                    plan[start + index] = revision[index];

            // Other units revised elsewhere in the plan, before and after this one.
            for (var other = 0; other < 25; other++)
            {
                var index = random.Next(plan.Count);
                plan[index] = Moved(plan[index], other);
            }

            if (call != 4)
                start += count + random.Next(1000, 5000);
        }

        // The support check's ground went back to the baseline with its defences restored.
        var expected = (float[])prepared.RoadbedBaseline.Clone();
        FloodDefences.Restore(expected, prepared.DefenceCrests!);
        Same(expected, cache.Ground(prepared.RoadbedBaseline, prepared.DefenceCrests), "support-check ground");
    }

    private static void SamePatch(NetworkRoadbed.Patch reference, NetworkRoadbed.Patch patch, string name)
    {
        Check(
            patch.X == reference.X
                && patch.Z == reference.Z
                && patch.Width == reference.Width
                && patch.Height == reference.Height
                && patch.Resolution == reference.Resolution
                && patch.MapWidth == reference.MapWidth,
            name + ": rectangle differs");

        Same(reference.Samples, patch.Samples, name + ": heights");
        Same(reference.Crests, patch.Crests, name + ": crests");
        Check(
            reference.Published == null == (patch.Published == null)
                && (reference.Published == null || reference.Published.SequenceEqual(patch.Published!)),
            name + ": published cells differ");
    }

    private static void Same(float[]? expected, float[]? actual, string what)
    {
        Check(expected == null == (actual == null), what + ": one is missing");

        if (expected == null)
            return;

        Check(expected.Length == actual!.Length, what + ": lengths differ");

        for (var i = 0; i < expected.Length; i++)
            if (BitConverter.SingleToInt32Bits(expected[i]) != BitConverter.SingleToInt32Bits(actual[i]))
                throw new Exception($"{what}: cell {i} differs ({expected[i]:R} against {actual[i]:R})");
    }

    // Elapsed times, and the progress lines written every five seconds, depend on speed alone.
    private static void SameLog(ImportLog expected, ImportLog actual, string name)
    {
        static IEnumerable<string> Timeless(IEnumerable<string> lines)
            => lines
                .Where(line => !line.StartsWith("PROCESS roadbed sections=", StringComparison.Ordinal))
                .Select(line => System.Text.RegularExpressions.Regex.Replace(line, @"elapsedSeconds=[0-9.]+", ""));

        Check(Timeless(expected.Lines).SequenceEqual(Timeless(actual.Lines)), name + ": log lines differ");
        Check(expected.Counts.SequenceEqual(actual.Counts), name + ": counts differ");
        Check(expected.Findings.SequenceEqual(actual.Findings), name + ": findings differ");
    }

    private static void Check(bool condition, string message)
    {
        if (!condition)
            throw new Exception(message);
    }

    private static float Coordinate(int index) => (float)((index / (double)(Resolution - 1) - .5) * MapWidth);

    // Hills of several wavelengths, 700 to 2,100 m above the bottom of the encoding.
    private static float[] Relief()
    {
        var heights = new float[Resolution * Resolution];

        for (var z = 0; z < Resolution; z++)
            for (var x = 0; x < Resolution; x++)
                heights[z * Resolution + x] =
                    (float)(1400
                        + 500 * Math.Sin(x * .004) * Math.Cos(z * .003)
                        + 200 * Math.Sin(x * .021 + z * .017)
                        + 25 * Math.Sin(x * .13) * Math.Sin(z * .11));

        return heights;
    }

    // The published ground: the baseline with earlier earthworks in a checkerboard of blocks.
    private static float[] Current(float[] baseline)
    {
        var current = (float[])baseline.Clone();

        for (var z = 0; z < Resolution; z++)
            for (var x = 0; x < Resolution; x++)
                if ((x / 64 + z / 64) % 5 == 0)
                    current[z * Resolution + x] += .75f;

        return current;
    }

    // Wandering roads and railways (150 unless told) of 20 sections each, with bridges, tunnels,
    // walls and paired carriageways; roads 1 and 2 run along the west and north edges of the map.
    private static List<PreparedSegment> Network(float[] baseline, int roads = 150)
    {
        var random = new Random(118);
        var sections = new List<PreparedSegment>();
        long node = 1;

        for (var road = 1; road <= roads; road++)
        {
            var position =
                road == 1 ? new float2(-Half + 4, -20000)
                : road == 2 ? new float2(-15000, Half - 4)
                : new float2((float)(random.NextDouble() * 50000 - 25000), (float)(random.NextDouble() * 50000 - 25000));

            var heading = road == 1 ? Math.PI / 2 : road == 2 ? 0 : random.NextDouble() * Math.PI * 2;
            var rail = road % 7 == 0;
            var paired = road % 10 == 3;
            var width = 8 + road % 4 * 6;
            var previousNode = node++;

            for (var k = 0; k < 20; k++)
            {
                if (road > 2)
                    heading += random.NextDouble() * .6 - .3;

                var length = (float)(180 + random.NextDouble() * 120);
                var direction = new float2((float)Math.Cos(heading), (float)Math.Sin(heading));
                var end = math.clamp(position + direction * length, -Half + 2, Half - 2);
                var side = new float2(-direction.y, direction.x);
                var control1 = math.lerp(position, end, 1f / 3) + side * (float)(random.NextDouble() * 20 - 10);
                var control2 = math.lerp(position, end, 2f / 3) + side * (float)(random.NextDouble() * 20 - 10);
                var bridge = k % 23 == 7 || road % 11 == 4 && k == 12;
                var tunnel = !bridge && k % 29 == 13;
                var lift = bridge ? 9f : tunnel ? -18f : (float)(4 * Math.Sin(k * .7 + road));

                float3 Point(float2 at) => new(at.x, NetworkProcessor.Sample(at.x, at.y, baseline, MapWidth) + lift, at.y);

                var section =
                    new PreparedSegment(
                        road,
                        previousNode,
                        node,
                        Point(position),
                        Point(end),
                        rail ? NetworkKind.Rail : NetworkKind.Road,
                        rail ? "rail" : "primary",
                        2,
                        false,
                        bridge,
                        tunnel,
                        0)
                    {
                        Control1 = Point(control1),
                        Control2 = Point(control2),
                        NativeWidth = width,
                        ExplicitWallSides = k % 17 == 5 ? NetworkSides.Left : NetworkSides.None,
                        NativeUnitId = road,
                        PairedProfileId = paired ? road * 100 + k : 0,
                        PairedReferenceBegin = k,
                        PairedReferenceEnd = k + 1
                    };

                sections.Add(section);

                // The opposite carriageway, 25 m to the side, drawn the other way at the same
                // heights; on one section it is a bridge where its pair is on the ground.
                if (paired)
                {
                    var offset = new float3(side.x * 25, 0, side.y * 25);

                    sections.Add(
                        section with
                        {
                            WayId = road + 1000,
                            StartNodeId = node + 100000,
                            EndNodeId = previousNode + 100000,
                            Start = section.End + offset,
                            End = section.Start + offset,
                            Control1 = section.Control2!.Value + offset,
                            Control2 = section.Control1!.Value + offset,
                            Bridge = bridge || k == 9,
                            PairedReversed = true
                        });
                }

                previousNode = node++;
                position = end;
            }
        }

        return sections;
    }

    // Count sections inside a window, spread evenly over all the sections that lie in it.
    private static PreparedSegment[] Unit(
        IReadOnlyList<PreparedSegment> plan,
        Func<PreparedSegment, bool> inside,
        int count,
        int seed)
    {
        var candidates = plan.Where(inside).ToArray();
        var random = new Random(seed);

        return candidates.OrderBy(_ => random.Next()).Take(count).ToArray();
    }

    private static bool Inside(PreparedSegment segment, float minimumX, float minimumZ, float maximumX, float maximumZ)
        => segment.Start.x >= minimumX
            && segment.Start.x <= maximumX
            && segment.Start.z >= minimumZ
            && segment.Start.z <= maximumZ;

    // A refit's revision: the section's ends and controls moved a few metres and its heights changed.
    private static PreparedSegment Moved(PreparedSegment segment, int index)
    {
        var shift = new float3(2.5f, index % 3 == 0 ? 1.2f : -.8f, -1.8f);

        return segment with
        {
            Start = segment.Start + (index % 2 == 0 ? shift : default),
            Control1 = segment.CourseControl1 + shift,
            Control2 = segment.CourseControl2 - shift * .5f,
            End = segment.End + shift
        };
    }

    // Sixty lakes and one long winding river across the whole map.
    private static float3[][] Areas()
    {
        var random = new Random(5);
        var areas = new List<float3[]>();

        for (var lake = 0; lake < 60; lake++)
        {
            var centre = new float2((float)(random.NextDouble() * 52000 - 26000), (float)(random.NextDouble() * 52000 - 26000));
            var radius = (float)(200 + random.NextDouble() * 1000);
            var ring = new float3[48];

            for (var i = 0; i < ring.Length; i++)
            {
                var angle = i * Math.PI * 2 / ring.Length;
                var wobble = 1 + .2f * (float)Math.Sin(angle * 5 + lake);
                ring[i] =
                    new float3(
                        centre.x + radius * wobble * (float)Math.Cos(angle),
                        0,
                        centre.y + radius * wobble * (float)Math.Sin(angle));
            }

            areas.Add(ring);
        }

        const int Points = 2000;
        var river = new float3[Points * 2];

        for (var i = 0; i < Points; i++)
        {
            var z = -Half + 100 + i * (MapWidth - 200) / (Points - 1);
            var x = -5000 + 3000 * Math.Sin(z * .0003) + 400 * Math.Sin(z * .003);
            river[i] = new float3((float)(x - 40), 0, (float)z);
            river[river.Length - 1 - i] = new float3((float)(x + 40), 0, (float)z);
        }

        areas.Add(river);

        return areas.ToArray();
    }

    // 400 streams of 30 segments each.
    private static float3[][] Lines()
    {
        var random = new Random(9);
        var lines = new float3[400][];

        for (var line = 0; line < lines.Length; line++)
        {
            var at = new float2((float)(random.NextDouble() * 54000 - 27000), (float)(random.NextDouble() * 54000 - 27000));
            var heading = random.NextDouble() * Math.PI * 2;
            lines[line] = new float3[31];

            for (var i = 0; i < lines[line].Length; i++)
            {
                lines[line][i] = new float3(at.x, 0, at.y);
                heading += random.NextDouble() * .8 - .4;
                at += new float2((float)Math.Cos(heading), (float)Math.Sin(heading)) * 120;
            }
        }

        return lines;
    }

    // A dyke along the diagonal, 1.5 m above the ground (none elsewhere).
    private static float[] Crests(float[] baseline)
    {
        var crests = new float[baseline.Length];
        Array.Fill(crests, float.NaN);

        for (var i = 0; i < Resolution; i++)
        {
            crests[i * Resolution + i] = baseline[i * Resolution + i] + 1.5f;
            crests[i * Resolution + Resolution - 1 - i] = baseline[i * Resolution + Resolution - 1 - i] - 1;
        }

        return crests;
    }

    // Thirty blocks of 40 x 40 cells that later water work edits.
    private static TerrainEditSet Edits()
    {
        var random = new Random(13);
        var mask = new byte[Resolution * Resolution];

        for (var block = 0; block < 30; block++)
        {
            var left = random.Next(Resolution - 40);
            var top = random.Next(Resolution - 40);

            for (var z = top; z < top + 40; z++)
                for (var x = left; x < left + 40; x++)
                    mask[z * Resolution + x] = 1;
        }

        return new TerrainEditSet("test", "baseline", Resolution, mask);
    }

    // ---- Reference: NetworkRoadbed.BuildLocalPatch and Prepare as they were (2026-11-16) ----

    private static NetworkRoadbed.Patch ReferenceBuildLocalPatch(
        PreparedImport prepared,
        float[] current,
        IReadOnlyList<PreparedSegment> plan,
        int start,
        IReadOnlyList<PreparedSegment> revision, ImportLog log, CancellationToken token)
    {
        var baseline = prepared.RoadbedBaseline;
        var resolution = TerrainSampling.Resolution(baseline);

        if (current.Length != baseline.Length || resolution < 2)
            throw new InvalidOperationException("Local roadbed baseline is unavailable.");

        var otherLow = new float2(float.PositiveInfinity);
        var otherHigh = new float2(float.NegativeInfinity);

        foreach (var segment in plan.Skip(start).Take(revision.Count).Concat(revision))
            foreach (var point in new[] {
                segment.CourseStart,
                segment.CourseControl1,
                segment.CourseControl2,
                segment.CourseEnd
                })
            {
                otherLow = math.min(otherLow, point.xz);
                otherHigh = math.max(otherHigh, point.xz);
            }

        var width = prepared.Request.MapSize.WidthMeters;
        var cell = width / (resolution - 1);
        var margin = 240 + revision.Max(segment => segment.NativeWidth) + 2 * cell;
        otherLow -= new float2((float)margin);
        otherHigh += new float2((float)margin);

        int Pixel(double x) => (int)math.clamp(Math.Floor((x / width + .5) * (resolution - 1)), 0, resolution - 1);

        var region =
            new int4(
                Pixel(otherLow.x),
                Pixel(otherLow.y),
                Math.Min(resolution - 1, Pixel(otherHigh.x) + 1),
                Math.Min(resolution - 1, Pixel(otherHigh.y) + 1));

        var all = plan.ToArray();

        for (var i = 0; i < revision.Count; i++)
            all[start + i] = revision[i];

        bool Nearby(PreparedSegment segment)
        {
            var min =
                math.min(
                    math.min(segment.CourseStart.xz, segment.CourseControl1.xz),
                    math.min(segment.CourseControl2.xz, segment.CourseEnd.xz));

            var max =
                math.max(
                    math.max(segment.CourseStart.xz, segment.CourseControl1.xz),
                    math.max(segment.CourseControl2.xz, segment.CourseEnd.xz));

            var reach = 240 + Math.Max(0, segment.NativeWidth) + 2 * cell;

            return math.all(max >= otherLow - (float)reach) && math.all(min <= otherHigh + (float)reach);
        }

        var changedBoxes =
            plan.Skip(start)
                .Take(revision.Count)
                .Concat(revision)
                .Select(
                    segment =>
                    {
                        var min =
                            math.min(
                                math.min(segment.CourseStart.xz, segment.CourseControl1.xz),
                                math.min(segment.CourseControl2.xz, segment.CourseEnd.xz));

                        var max =
                            math.max(
                                math.max(segment.CourseStart.xz, segment.CourseControl1.xz),
                                math.max(segment.CourseControl2.xz, segment.CourseEnd.xz));

                        return (Low: min - new float2((float)margin), High: max + new float2((float)margin));
                    })
                .ToArray();

        bool NearChange(PreparedSegment segment)
        {
            if (!Nearby(segment))
                return false;

            var min =
                math.min(
                    math.min(segment.CourseStart.xz, segment.CourseControl1.xz),
                    math.min(segment.CourseControl2.xz, segment.CourseEnd.xz));

            var max =
                math.max(
                    math.max(segment.CourseStart.xz, segment.CourseControl1.xz),
                    math.max(segment.CourseControl2.xz, segment.CourseEnd.xz));

            var reach = (float)(240 + Math.Max(0, segment.NativeWidth) + 2 * cell);

            foreach (var box in changedBoxes)
                if (math.all(max >= box.Low - reach) && math.all(min <= box.High + reach))
                    return true;

            return false;
        }

        var neighbours = all.Where(NearChange).ToArray();
        var rebuilt =
            ReferencePrepare(
                baseline,
                prepared.HeightScaleMeters,
                width,
                neighbours,
                prepared.RoadbedProtectedAreas,
                prepared.RoadbedProtectedLines,
                log,
                token,
                region);

        if (prepared.DefenceCrests?.Length == rebuilt.Length)
            log.Count("Local refit flood defence cells restored", FloodDefences.Restore(rebuilt, prepared.DefenceCrests));

        var issued =
            all.Take(start).Where(segment => !NetworkStructure.IsStructure(segment) && NearChange(segment)).ToArray();

        var w = region.z - region.x + 1;
        var h = region.w - region.y + 1;
        var samples = new float[w * h];
        var fromRebuilt = new bool[w * h];
        var preserved = 0;
        var protectedCores = new bool[w * h];
        var changeable = new bool[w * h];

        foreach (var box in changedBoxes)
            for (var z = Math.Max(region.y, Pixel(box.Low.y)); z <= Math.Min(region.w, Pixel(box.High.y) + 1); z++)
                for (var x = Math.Max(region.x, Pixel(box.Low.x)); x <= Math.Min(region.z, Pixel(box.High.x) + 1); x++)
                    changeable[(z - region.y) * w + x - region.x] = true;

        foreach (var segment in issued)
        {
            var steps =
                Math.Max(1, (int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(segment) / Math.Max(1, cell * .5)));

            var radius = segment.NativeWidth / 2 + cell;
            var previous = NetworkNormalizer.Evaluate(segment, 0);

            for (var step = 1; step <= steps; step++)
            {
                token.ThrowIfCancellationRequested();
                var next = NetworkNormalizer.Evaluate(segment, step / (float)steps);
                var low = math.min(previous.xz, next.xz) - (float)radius;
                var high = math.max(previous.xz, next.xz) + (float)radius;

                for (var z = Math.Max(region.y, Pixel(low.y)); z <= Math.Min(region.w, Pixel(high.y) + 1); z++)
                    for (var x = Math.Max(region.x, Pixel(low.x)); x <= Math.Min(region.z, Pixel(high.x) + 1); x++)
                    {
                        var point =
                            new float3(
                                (float)((x / (double)(resolution - 1) - .5) * width),
                                0,
                                (float)((z / (double)(resolution - 1) - .5) * width));

                        if (NetworkNormalizer.DistanceToSegment(point, previous, next) <= radius)
                            protectedCores[(z - region.y) * w + x - region.x] = true;
                    }

                previous = next;
            }
        }

        for (var z = 0; z < h; z++)
            for (var x = 0; x < w; x++)
            {
                token.ThrowIfCancellationRequested();
                var gx = region.x + x;
                var gz = region.y + z;
                var id = gz * resolution + gx;
                var edits = prepared.RoadbedLaterEdits;
                var protectedCore =
                    !changeable[z * w + x]
                    || protectedCores[z * w + x]
                    || edits?.Mask.Length == current.Length && edits.Mask[id] != 0;

                samples[z * w + x] = protectedCore ? current[id] : rebuilt[id];
                fromRebuilt[z * w + x] = !protectedCore;

                if (protectedCore && changeable[z * w + x])
                    preserved++;
            }

        log.Diagnostic(
            $"LOCAL ROADBED staged samples={samples.Length}; neighbourSections={neighbours.Length}; issuedCoreSamplesPreserved={preserved}; baselineImmutable=true; nativeMutation=false");

        float[]? crests = null;

        if (prepared.DefenceCrests?.Length == rebuilt.Length)
        {
            crests = new float[w * h];

            for (var z = 0; z < h; z++)
                Array.Copy(prepared.DefenceCrests, (region.y + z) * resolution + region.x, crests, z * w, w);
        }

        var published = new List<int>();

        for (var z = 0; z < h; z++)
            for (var x = 0; x < w; x++)
            {
                var near = false;

                for (var dz = Math.Max(0, z - 1); dz <= Math.Min(h - 1, z + 1) && !near; dz++)
                    for (var dx = Math.Max(0, x - 1); dx <= Math.Min(w - 1, x + 1) && !near; dx++)
                        near = fromRebuilt[dz * w + dx];

                if (near)
                    published.Add(z * w + x);
            }

        var patch =
            new NetworkRoadbed.Patch(region.x, region.y, w, h, samples, resolution, width)
            {
                Crests = crests,
                Published = published.ToArray()
            };
        patch.CopyTo(rebuilt);

        if (log.DiagnosticsEnabled)
            NetworkTerrainSupport.Validate(
                revision,
                rebuilt,
                prepared.HeightScaleMeters,
                width,
                log,
                token,
                scope: $"native-unit-{revision[0].NativeUnitId}");

        return patch;
    }

    private static float[] ReferencePrepare(float[] source, float scale, double width, IReadOnlyList<PreparedSegment> segments,
        IEnumerable<float3[]> protectedAreas,
            IEnumerable<float3[]> protectedLines,
            ImportLog log,
            CancellationToken token,
            int4? dirtyRegion = null,
            float waterSurface = float.NaN)
    {
        var n = (int)Math.Sqrt(source.Length);

        if (n * n != source.Length || n < 2 || width <= 0 || scale <= 0)
            throw new InvalidOperationException("Invalid roadbed terrain.");

        var cell = (float)(width / (n - 1));
        var protectedMask = new bool[source.Length];
        var lineGuard = new bool[source.Length];
        var region = dirtyRegion ?? new int4(0, 0, n - 1, n - 1);
        var areaMask = new bool[source.Length];
        var coordinates = new float[n];

        for (var i = 0; i < n; i++)
            coordinates[i] = (i / (float)(n - 1) - .5f) * (float)width;

        int Pixel(float value) => (int)Math.Round((value / width + .5) * (n - 1));

        void Guard(float3 vectorA, float3 vectorB, float margin, bool[]? mask = null)
        {
            var low = math.min(vectorA.xz, vectorB.xz) - margin;
            var high = math.max(vectorA.xz, vectorB.xz) + margin;
            var delta = vectorB.xz - vectorA.xz;
            var denominator = Math.Max(.001f, math.lengthsq(delta));

            for (var z = Math.Max(region.y, Pixel(low.y)); z <= Math.Min(region.w, Pixel(high.y)); z++)
                for (var x = Math.Max(region.x, Pixel(low.x)); x <= Math.Min(region.z, Pixel(high.x)); x++)
                {
                    var point = new float2(coordinates[x], coordinates[z]);
                    var t = math.clamp(math.dot(point - vectorA.xz, delta) / denominator, 0, 1);

                    if (math.distance(point, math.lerp(vectorA.xz, vectorB.xz, t)) <= margin)
                        (mask ?? protectedMask)[z * n + x] = true;
                }
        }

        foreach (var ring in protectedAreas)
        {
            token.ThrowIfCancellationRequested();

            if (ring.Length < 3)
                continue;

            var crossings = new List<float>(ring.Length);

            for (var z =
                Math.Max(
                    region.y,
                    Pixel(ring.Min(point => point.z))); z <= Math.Min(region.w, Pixel(ring.Max(point => point.z))); z++)
            {
                token.ThrowIfCancellationRequested();
                var y = coordinates[z];
                crossings.Clear();

                for (var i = 0; i < ring.Length; i++)
                {
                    var vectorA = ring[i];
                    var vectorB = ring[(i + 1) % ring.Length];

                    if ((vectorA.z > y) == (vectorB.z > y))
                        continue;

                    crossings.Add(vectorA.x + (y - vectorA.z) * (vectorB.x - vectorA.x) / (vectorB.z - vectorA.z));
                }

                crossings.Sort();

                for (var i = 1; i < crossings.Count; i += 2)
                    for (var x =
                        Math.Max(region.x, Pixel(crossings[i - 1])); x <= Math.Min(region.z, Pixel(crossings[i])); x++)
                        protectedMask[z * n + x] = areaMask[z * n + x] = true;
            }

            for (var i = 0; i < ring.Length; i++)
                Guard(ring[i], ring[(i + 1) % ring.Length], cell);
        }

        foreach (var line in protectedLines)
            for (var i = 1; i < line.Length; i++)
            {
                token.ThrowIfCancellationRequested();
                Guard(line[i - 1], line[i], cell * 2, lineGuard);
            }

        const float PortalOpen = 40, PortalReach = 100;
        var tunnelArms = new Dictionary<(NetworkKind, long), (int Tunnel, int Open, float2 At)>();

        foreach (var segment in segments)
            foreach (var (id, at) in new[] {
                (segment.StartNodeId, segment.CourseStart.xz),
                (segment.EndNodeId, segment.CourseEnd.xz)
                })
            {
                tunnelArms.TryGetValue((segment.Kind, id), out var arms);
                tunnelArms[(segment.Kind, id)] =
                    segment.Tunnel ? (arms.Tunnel + 1, arms.Open, at) : (arms.Tunnel, arms.Open + 1, at);
            }

        var portals = new Dictionary<(int, int), List<float2>>();

        foreach (var arms in tunnelArms.Values)
            if (arms.Tunnel > 0 && arms.Open > 0)
            {
                var key = ((int)Math.Floor(arms.At.x / PortalReach), (int)Math.Floor(arms.At.y / PortalReach));

                if (!portals.TryGetValue(key, out var list))
                    portals[key] = list = new();

                list.Add(arms.At);
            }

        float PortalDistance(float2 point)
        {
            var best = float.PositiveInfinity;
            int cx = (int)Math.Floor(point.x / PortalReach), cz = (int)Math.Floor(point.y / PortalReach);

            for (var dz = -1; dz <= 1; dz++)
                for (var dx = -1; dx <= 1; dx++)
                    if (portals.TryGetValue((cx + dx, cz + dz), out var list))
                        foreach (var at in list)
                            best = Math.Min(best, math.distance(point, at));

            return best;
        }

        foreach (var tunnel in segments.Where(segment => segment.Tunnel))
        {
            var steps =
                Math.Max(1, (int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(tunnel) / Math.Max(1, cell * .5f)));

            var margin = (tunnel.NativeWidth > 0 ? tunnel.NativeWidth * .5f : 6) + cell * 2;
            var previous = NetworkNormalizer.Evaluate(tunnel, 0);

            for (var i = 1; i <= steps; i++)
            {
                token.ThrowIfCancellationRequested();
                var next = NetworkNormalizer.Evaluate(tunnel, i / (float)steps);

                if (PortalDistance(previous.xz) >= PortalOpen || PortalDistance(next.xz) >= PortalOpen)
                    Guard(previous, next, margin);

                previous = next;
            }
        }

        var targets = new Dictionary<int, (float target, float budget, float blend, float distance)>();
        var protectedSamples = 0;
        var beyondBudget = 0;
        var clock = System.Diagnostics.Stopwatch.StartNew();
        var nextProgress = 5d;
        var processed = 0;
        var unsupportedSections = 0;
        var maximumSupportDeficit = 0f;
        var formations = new List<PreparedSegment>();
        var consumed = new HashSet<PreparedSegment>();
        var sharedFormations = 0;
        var mixedFormations = 0;
        var incompatibleFormations = 0;

        foreach (var group in segments
            .Where(segment => segment.PairedProfileId != 0)
            .GroupBy(segment => (segment.PairedProfileId,
            Begin: Math.Round(Math.Min(segment.PairedReferenceBegin, segment.PairedReferenceEnd), 6),
            End: Math.Round(Math.Max(segment.PairedReferenceBegin, segment.PairedReferenceEnd), 6))))
        {
            token.ThrowIfCancellationRequested();
            var pair = group.ToArray();

            if (pair.Length != 2 || pair[0].PairedReversed == pair[1].PairedReversed)
                continue;

            if (pair.Any(segment => segment.ExplicitWallSides != 0 || segment.QuaySides != 0))
                continue;

            var grounds = pair.Where(segment => !NetworkStructure.IsStructure(segment)).ToArray();

            if (grounds.Length == 0)
                continue;

            var mixed = grounds.Length == 1;
            var firstSegment = mixed ? grounds[0] : pair[0];
            var secondSegment = ReferenceEquals(firstSegment, pair[0]) ? pair[1] : pair[0];
            var aa =
                new[] {
                    firstSegment.CourseStart,
                    firstSegment.CourseControl1,
                    firstSegment.CourseControl2,
                    firstSegment.CourseEnd
                    };

            var bb =
                new[] {
                    secondSegment.CourseEnd,
                    secondSegment.CourseControl2,
                    secondSegment.CourseControl1,
                    secondSegment.CourseStart
                    };

            if (Enumerable.Range(0, 4).Any(i => Math.Abs(aa[i].y - bb[i].y) > .01f))
            {
                incompatibleFormations++;
                continue;
            }

            var gap = Enumerable.Range(0, 4).Max(i => math.distance(aa[i].xz, bb[i].xz));
            var center = Enumerable.Range(0, 4).Select(i =>
            {
                if (!mixed)
                    return (aa[i] + bb[i]) * .5f;

                var separation = math.distance(aa[i].xz, bb[i].xz);
                var fraction =
                    separation > firstSegment.NativeWidth ? (1 - firstSegment.NativeWidth / separation) / 4 : 0;

                return math.lerp(aa[i], bb[i], fraction);
            }).ToArray();

            formations.Add(firstSegment with
            {
                Start = center[0],
                Control1 = center[1],
                Control2 = center[2],
                End = center[3],
                StartCurveOffset = default,
                EndCurveOffset = default,
                FinalDeckCurve = new(center[0], center[1], center[2], center[3]),
                HorizontalParent = null,
                NativeWidth =
                    mixed
                        ? Math.Max(firstSegment.NativeWidth, (gap + firstSegment.NativeWidth) / 2)
                        : gap + Math.Max(firstSegment.NativeWidth, secondSegment.NativeWidth)
            });

            consumed.Add(firstSegment);

            if (!mixed)
            {
                consumed.Add(secondSegment);
                sharedFormations++;
            }
            else
            {
                mixedFormations++;
                var steps =
                    Math.Max(
                        1,
                        (int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(secondSegment) / Math.Max(1, cell * .5)));

                var previous = NetworkNormalizer.Evaluate(secondSegment, 0);

                for (var step = 1; step <= steps; step++)
                {
                    var next = NetworkNormalizer.Evaluate(secondSegment, step / (float)steps);
                    Guard(previous, next, secondSegment.NativeWidth / 2 + cell);
                    previous = next;
                }
            }
        }

        formations.AddRange(
            segments.Where(segment => !consumed.Contains(segment) && !NetworkStructure.IsStructure(segment)));

        const float ApproachReach = 100;
        var narrowedApproachSamples = 0;
        var structureEnds = new Dictionary<(int, int), List<float2>>();

        foreach (var segment in segments.Where(NetworkStructure.IsStructure))
            foreach (var end in new[] { segment.CourseStart.xz, segment.CourseEnd.xz })
            {
                var key = ((int)Math.Floor(end.x / ApproachReach), (int)Math.Floor(end.y / ApproachReach));

                if (!structureEnds.TryGetValue(key, out var list))
                    structureEnds[key] = list = new();

                list.Add(end);
            }

        var bridgeEnds = new Dictionary<(int, int), List<float2>>();

        foreach (var segment in segments.Where(otherSegment => otherSegment.Bridge && !otherSegment.Tunnel))
            foreach (var end in new[] { segment.CourseStart.xz, segment.CourseEnd.xz })
            {
                var key = ((int)Math.Floor(end.x / ApproachReach), (int)Math.Floor(end.y / ApproachReach));

                if (!bridgeEnds.TryGetValue(key, out var list))
                    bridgeEnds[key] = list = new();

                list.Add(end);
            }

        float EndDistance(Dictionary<(int, int), List<float2>> ends, float2 point)
        {
            var best = float.PositiveInfinity;
            int cx = (int)Math.Floor(point.x / ApproachReach), cz = (int)Math.Floor(point.y / ApproachReach);

            for (var dz = -1; dz <= 1; dz++)
                for (var dx = -1; dx <= 1; dx++)
                    if (ends.TryGetValue((cx + dx, cz + dz), out var list))
                        foreach (var end in list)
                            best = Math.Min(best, math.distance(point, end));

            return best;
        }

        float StructureEndDistance(float2 point) => EndDistance(structureEnds, point);

        log.Quality(
            new(
                "planned",
                "mixed-mode-formation",
                "roadbed",
                mixedFormations + incompatibleFormations,
                0,
                incompatibleFormations,
                0,
                0,
            "Trusted common profile; ground/median half-formation only; structural interpolation support protected; actual raster support checked separately"));

        log.Diagnostic(
            $"ROADBED FORMATIONS sharedGroundProfiles={sharedFormations}; sharedMixedModeHalfFormations={mixedFormations}; incompatibleProfiles={incompatibleFormations}; structuralFootprintsProtected=true; explicitWallsRetainedSeparately=true; profileHeightsNotReoptimized=true");

        var narrowApproaches = !DesignSwitches.On(DesignSwitches.WideApproaches);

        foreach (var segment in formations)
        {
            token.ThrowIfCancellationRequested();
            processed++;
            var budget = NetworkVerticalProfiles.CutFillBudget(segment);
            var half =
                segment.NativeWidth > 0 ? segment.NativeWidth * .5f : Math.Min(12, Math.Max(4, segment.Lanes * 1.75f));

            var unsupported = false;
            var length = NetworkVerticalProfiles.ArcLength(segment);
            var steps = Math.Max(1, (int)Math.Ceiling(length / (cell * .5f)));
            var extendStart = EndDistance(bridgeEnds, segment.CourseStart.xz) < .5f ? 1 : 0;
            var extendEnd = EndDistance(bridgeEnds, segment.CourseEnd.xz) < .5f ? 1 : 0;
            var startPoint = NetworkNormalizer.Evaluate(segment, 0);
            var endPoint = NetworkNormalizer.Evaluate(segment, 1);
            var outStart = math.normalizesafe(startPoint - NetworkNormalizer.Evaluate(segment, .001f));
            var outEnd = math.normalizesafe(endPoint - NetworkNormalizer.Evaluate(segment, .999f));

            float3 Station(int i)
                => i < 0
                    ? startPoint + outStart * (-i * length / steps)
                    : i > steps
                        ? endPoint + outEnd * ((i - steps) * length / steps)
                        : NetworkNormalizer.Evaluate(segment, i / (float)steps);

            for (var i = -extendStart; i <= steps + extendEnd; i++)
            {
                token.ThrowIfCancellationRequested();

                if (clock.Elapsed.TotalSeconds >= nextProgress)
                {
                    log.Diagnostic(
                        $"PROCESS roadbed sections={processed}; sectionSample={i}/{steps}; affectedSamples={targets.Count}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.0}");

                    nextProgress = clock.Elapsed.TotalSeconds + 5;
                }

                var t = math.clamp(i / (float)steps, 0, 1);
                var point = Station(i);
                var px = Pixel(point.x);
                var pz = Pixel(point.z);
                var previous = Station(Math.Max(-extendStart, i - 1));
                var span = point.xz - previous.xz;
                var spanLengthSquared = math.lengthsq(span);
                var originalCenter = NetworkProcessor.Sample(point.x, point.z, source, width);
                var deficit = Math.Abs(originalCenter - point.y) - budget;

                if (deficit > Math.Max(.05f, 2 * scale / 65535f))
                {
                    unsupported = true;
                    maximumSupportDeficit = Math.Max(maximumSupportDeficit, deficit);
                }

                var earthBlend =
                    Math.Min(240, Math.Max(cell * 3f, Math.Min(budget, Math.Abs(originalCenter - point.y)) * 6f));

                if (narrowApproaches && structureEnds.Count > 0)
                {
                    var approach = StructureEndDistance(point.xz);

                    if (approach < ApproachReach)
                    {
                        earthBlend = math.lerp(cell * 1.5f, earthBlend, approach / ApproachReach);
                        narrowedApproachSamples++;
                    }
                }

                var radius = half + earthBlend;
                var reach = (int)Math.Ceiling(radius / cell);
                var tangent =
                    math.normalizesafe(
                        (NetworkNormalizer.Evaluate(segment, Math.Min(1, t + .01f))
                            - NetworkNormalizer.Evaluate(segment, Math.Max(0, t - .01f))).xz);

                var leftNormal = new float2(-tangent.y, tangent.x);

                for (var z = Math.Max(region.y, pz - reach); z <= Math.Min(region.w, pz + reach); z++)
                    for (var x = Math.Max(region.x, px - reach); x <= Math.Min(region.z, px + reach); x++)
                    {
                        var sample = new float2(coordinates[x], coordinates[z]);
                        var station =
                            spanLengthSquared > .00001f
                                ? math.clamp(math.dot(sample - previous.xz, span) / spanLengthSquared, 0, 1)
                                : 1;

                        var nearest = math.lerp(previous, point, station);
                        var delta = sample - nearest.xz;
                        var distance = math.length(delta);
                        var side = math.dot(delta, leftNormal) >= 0 ? NetworkSides.Left : NetworkSides.Right;
                        var wall = ((segment.ExplicitWallSides | segment.QuaySides) & side) != 0;
                        var sideRadius = half + (wall ? cell * .75f : earthBlend);
                        var full = wall ? half : half + cell;

                        if (distance > sideRadius)
                            continue;

                        var id = z * n + x;

                        if (protectedMask[id] || lineGuard[id] && distance > half + cell)
                        {
                            protectedSamples++;
                            continue;
                        }

                        var original = source[id];

                        if (Math.Abs(nearest.y - original) > budget + .05f)
                        {
                            if (distance <= cell * .5f)
                                beyondBudget++;
                        }

                        var supportedTarget =
                            math.clamp(nearest.y, Math.Max(0, original - budget), Math.Min(scale, original + budget));

                        var blendT = math.clamp((sideRadius - distance) / Math.Max(.01f, sideRadius - full), 0, 1);
                        var weight = distance <= full ? 1 : blendT * blendT * (3 - 2 * blendT);

                        if (weight <= 0)
                            continue;

                        targets.TryGetValue(id, out var old);

                        if (old.blend == 0 || distance < old.distance)
                        {
                            if (old.blend > 0
                                && distance > half
                                && old.distance > half
                                && Math.Abs(old.target - supportedTarget) < cell)
                            {
                                var combined = old.blend + weight;
                                targets[id] =
                                    (
                                        (old.target * old.blend + supportedTarget * weight) / combined,
                                        Math.Min(old.budget, budget),
                                        Math.Max(old.blend, weight),
                                        distance);
                            }
                            else
                                targets[id] = (supportedTarget, budget, weight, distance);
                        }
                    }
            }

            if (unsupported)
                unsupportedSections++;

            if (clock.Elapsed.TotalSeconds >= nextProgress)
            {
                log.Diagnostic(
                    $"PROCESS roadbed sections={processed}; affectedSamples={targets.Count}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.0}");

                nextProgress = clock.Elapsed.TotalSeconds + 5;
            }
        }

        const float BridgeUnder = 2.5f, BridgeCut = 10, BridgeUnderRamp = 40;
        var under = new Dictionary<int, float>();

        foreach (var bridge in segments.Where(segment => segment.Bridge && !segment.Tunnel))
        {
            token.ThrowIfCancellationRequested();
            var length = NetworkVerticalProfiles.ArcLength(bridge);
            var steps = Math.Max(1, (int)Math.Ceiling(length / Math.Max(1, cell * .5f)));
            var core = bridge.NativeWidth > 0 ? bridge.NativeWidth * .5f : 6;
            var edge = core + cell * 2;
            var r = (int)Math.Ceiling(edge / cell);

            for (var i = 0; i <= steps; i++)
            {
                var t = i / (float)steps;
                var clearance = BridgeUnder * Math.Min(1, Math.Min(t, 1 - t) * length / BridgeUnderRamp);

                if (clearance <= 0)
                    continue;

                var point = NetworkNormalizer.Evaluate(bridge, t);
                var px = Pixel(point.x);
                var pz = Pixel(point.z);

                for (var z = Math.Max(region.y, pz - r); z <= Math.Min(region.w, pz + r); z++)
                    for (var x = Math.Max(region.x, px - r); x <= Math.Min(region.z, px + r); x++)
                    {
                        var id = z * n + x;
                        var distance = math.distance(new float2(coordinates[x], coordinates[z]), point.xz);

                        if (distance > edge || protectedMask[id] || lineGuard[id] || targets.ContainsKey(id))
                            continue;

                        var original = source[id];
                        var target = Math.Max(point.y - clearance, original - BridgeCut);

                        if (target >= original)
                            continue;

                        var blend = distance <= core ? 1 : Numeric.SmoothStep(1 - (distance - core) / (edge - core));
                        target = original + (target - original) * blend;
                        under[id] = under.TryGetValue(id, out var old) ? Math.Min(old, target) : target;
                    }
            }
        }

        var result = (float[])source.Clone();
        var changed = 0;
        var maximum = 0f;

        foreach (var entry in targets)
        {
            token.ThrowIfCancellationRequested();
            var original = source[entry.Key];
            var target = entry.Value.target;
            var blend = entry.Value.blend;
            target = original + (target - original) * blend;
            target =
                math.clamp(
                    target,
                    Math.Max(0, original - entry.Value.budget),
                    Math.Min(scale, original + entry.Value.budget));

            result[entry.Key] = target;

            if (result[entry.Key] != source[entry.Key])
                changed++;

            maximum = Math.Max(maximum, Math.Abs(target - original));
        }

        foreach (var (id, target) in under)
        {
            var value = math.clamp(target, 0, scale);

            if (value < result[id])
            {
                result[id] = value;
                changed++;
            }
        }

        log.Count("Roadbed samples lowered under bridge decks", under.Count);
        const float RoofHeight = 12, RoofFill = 10, RoofSlope = .5f, RoofRamp = 60;
        const int RoofShoulder = 3;
        var roofTargets = new Dictionary<int, float>();

        foreach (var tunnel in segments.Where(segment => segment.Tunnel))
        {
            token.ThrowIfCancellationRequested();
            var core = (tunnel.NativeWidth > 0 ? tunnel.NativeWidth * .5f : 6) + cell;
            var reach = core + RoofShoulder * cell;
            var steps =
                Math.Max(1, (int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(tunnel) / Math.Max(1, cell * .5f)));

            for (var i = 0; i <= steps; i++)
            {
                var point = NetworkNormalizer.Evaluate(tunnel, i / (float)steps);
                var r = (int)Math.Ceiling(reach / cell);
                var px = Pixel(point.x);
                var pz = Pixel(point.z);
                var fill = RoofFill * Numeric.SmoothStep((PortalDistance(point.xz) - PortalOpen) / RoofRamp);

                if (fill <= 0)
                    continue;

                for (var z = Math.Max(region.y, pz - r); z <= Math.Min(region.w, pz + r); z++)
                    for (var x = Math.Max(region.x, px - r); x <= Math.Min(region.z, px + r); x++)
                    {
                        var id = z * n + x;
                        var distance = math.distance(new float2(coordinates[x], coordinates[z]), point.xz);

                        if (areaMask[id] || distance > reach)
                            continue;

                        var original = source[id];
                        var current = result[id];
                        var target =
                            Math.Min(point.y + RoofHeight - Math.Max(0, distance - core) * RoofSlope, original + fill);

                        if (math.isfinite(waterSurface) && original < waterSurface)
                            target = Math.Min(target, waterSurface - 1);

                        if (targets.TryGetValue(id, out var formation) && formation.blend >= 1)
                            target = Math.Min(target, formation.target);

                        if (target <= current)
                            continue;

                        roofTargets[id] =
                            roofTargets.TryGetValue(id, out var higher) ? Math.Max(higher, target) : target;
                    }
            }
        }

        if (roofTargets.Count > 0)
        {
            const int Tile = 16;
            var tiles = new HashSet<(int, int)>();

            foreach (var id in roofTargets.Keys)
                tiles.Add((id % n / Tile, id / n / Tile));

            foreach (var segment in segments.Where(otherSegment => !NetworkStructure.IsStructure(otherSegment)))
            {
                token.ThrowIfCancellationRequested();
                var half = segment.NativeWidth > 0 ? segment.NativeWidth * .5f : 6;
                var reach = half + cell + RoofFill / RoofSlope;
                var r = (int)Math.Ceiling(reach / cell);
                var steps = Math.Max(1, (int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(segment) / (cell * .5f)));

                for (var i = 0; i <= steps; i++)
                {
                    var point = NetworkNormalizer.Evaluate(segment, i / (float)steps);
                    var px = Pixel(point.x);
                    var pz = Pixel(point.z);
                    var near = false;

                    for (var dz = -1; dz <= 1 && !near; dz++)
                        for (var dx = -1; dx <= 1 && !near; dx++)
                            near = tiles.Contains((px / Tile + dx, pz / Tile + dz));

                    if (!near)
                        continue;

                    for (var z = Math.Max(region.y, pz - r); z <= Math.Min(region.w, pz + r); z++)
                        for (var x = Math.Max(region.x, px - r); x <= Math.Min(region.z, px + r); x++)
                        {
                            var id = z * n + x;

                            if (!roofTargets.TryGetValue(id, out var target))
                                continue;

                            var ceiling =
                                point.y
                                    + Math.Max(
                                        0,
                                        math.distance(new float2(coordinates[x], coordinates[z]), point.xz)
                                            - half
                                            - cell)
                                        * RoofSlope;

                            if (ceiling < target)
                                roofTargets[id] = ceiling;
                        }
                }
            }
        }

        var roofSamples = 0;

        foreach (var (id, target) in roofTargets)
        {
            if (target <= result[id])
                continue;

            result[id] = math.clamp(target, 0, scale);
            roofSamples++;
        }

        log.Count("Roadbed tunnel roof samples filled", roofSamples);
        const int RefineSweeps = 12;
        var refined = new Dictionary<int, float>();

        float Height(int id) => refined.TryGetValue(id, out var h) ? h : result[id];

        var deckPoints = new List<float3>();

        foreach (var segment in segments.Where(otherSegment => !NetworkStructure.IsStructure(otherSegment)))
        {
            var steps = Math.Max(1, (int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(segment) / (cell * .5f)));

            for (var i = 0; i <= steps; i++)
                deckPoints.Add(NetworkNormalizer.Evaluate(segment, i / (float)steps));
        }

        var ids = new int[4];
        var weights = new float[4];

        for (var sweep = 0; sweep < RefineSweeps; sweep++)
            foreach (var point in deckPoints)
            {
                token.ThrowIfCancellationRequested();
                var gx = math.clamp(point.x / width + .5, 0, 1) * (n - 1);
                var gz = math.clamp(point.z / width + .5, 0, 1) * (n - 1);
                int x0 = (int)gx, z0 = (int)gz, x1 = Math.Min(x0 + 1, n - 1), z1 = Math.Min(z0 + 1, n - 1);
                var tx = (float)(gx - x0);
                var tz = (float)(gz - z0);
                ids[0] = z0 * n + x0;
                ids[1] = z0 * n + x1;
                ids[2] = z1 * n + x0;
                ids[3] = z1 * n + x1;
                weights[0] = (1 - tx) * (1 - tz);
                weights[1] = tx * (1 - tz);
                weights[2] = (1 - tx) * tz;
                weights[3] = tx * tz;
                var read = 0f;
                var norm = 0f;
                var movable = 0f;

                for (var k = 0; k < 4; k++)
                {
                    read += weights[k] * Height(ids[k]);
                    var x = k % 2 == 0 ? x0 : x1;
                    var z = k < 2 ? z0 : z1;

                    if (x < region.x
                        || x > region.z
                        || z < region.y
                        || z > region.w
                        || protectedMask[ids[k]]
                        || !targets.ContainsKey(ids[k]))
                        weights[k] = 0;

                    norm += weights[k] * weights[k];
                    movable += weights[k];
                }

                var residual = point.y - read;

                if (Math.Abs(residual) < .05f || movable < .5f)
                    continue;

                for (var k = 0; k < 4; k++)
                {
                    if (weights[k] == 0)
                        continue;

                    var id = ids[k];
                    var original = source[id];
                    var budget = targets[id].budget;
                    var current = Height(id);
                    var next =
                        math.clamp(
                            current + residual * weights[k] / norm,
                            Math.Min(current, point.y) - 1,
                            Math.Max(current, point.y) + 1);

                    refined[id] = math.clamp(next, Math.Max(0, original - budget), Math.Min(scale, original + budget));
                }
            }

        foreach (var (id, height) in refined)
        {
            var value = math.clamp(height, 0, scale);

            if (value != result[id])
            {
                if (result[id] == source[id])
                    changed++;

                result[id] = value;
            }
        }

        log.Count("Roadbed samples refined to the decks that read them", refined.Count);
        log.Count("Roadbed terrain samples changed", changed);
        log.Count("Roadbed protected samples avoided", protectedSamples);
        log.Count("Roadbed samples beyond cut/fill budget", beyondBudget);
        log.Count("Roadbed ground formations beyond support budget", unsupportedSections);
        log.Count("Roadbed approach samples kept to the footprint", narrowedApproachSamples);
        log.Diagnostic(
            $"ROADBED SUPPORT sampledUnsupportedFormations={unsupportedSections}; maximumCenterlineDeficitMeters={maximumSupportDeficit:0.###}; boundedEarthworkNotSupportAcceptance=true; continuousFinalSupportAuditStillRequired=true");

        log.Diagnostic(
            $"PROCESS roadbed complete; changedSamples={changed}; maximumCutFillMeters={maximum:0.###}; target=nearest-curve-station; shoulderMetersPerFillMeter=6; maximumShoulderMeters=240; zeroWeightAtToe=true; targetAndShoulderBoundedBeforeFeather=true; protectedWaterCliffsQuarries=true; structuresExcluded=true; elapsedSeconds={clock.Elapsed.TotalSeconds:0.0}");

        return result;
    }
}
