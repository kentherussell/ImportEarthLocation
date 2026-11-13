using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using Unity.Mathematics;

// The whole road pipeline from a capture's OSM, through the code the game runs,
// measured by RoadQualityReport. Water, terrain and native contracts stay as
// captured, so this is not a fresh import.
static partial class VisualMetricsReplay
{
    public static void RoadReplay(
        string folder,
        string output,
        IEnumerable<string> skip,
        bool strict,
        IEnumerable<string>? with = null)
    {
        var otherCapture = Load(folder);
        var request = otherCapture.Input["request"]!.ToObject<ImportRequest>()!;

        // The switches the capture was designed with, plus any asked for (report 112, 11.4).
        DesignSwitches.Set(string.Join(",", (with ?? Array.Empty<string>()).Prepend(request.DesignSwitches)));
        Console.WriteLine($"design switches: {(DesignSwitches.Active.Length > 0 ? DesignSwitches.Active : "none")}");

        Dictionary<long, T> Read<T>(string name)
            => CorrectiveCapturedReplay
                .ReadManifest<KeyValuePair<long, T>>(folder, "offline-osm-" + name + "-chunks.json")
                .ToDictionary(entry => entry.Key, entry => entry.Value);

        var metadata = JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder, "offline-osm-metadata.json")));
        var raw = new OsmDataset
        {
            Nodes = Read<OsmNode>("nodes"),
            Ways = Read<OsmWay>("ways"),
            NodeTags = Read<IReadOnlyDictionary<string, string>>("node-tags"),
            RawWayTags = Read<IReadOnlyDictionary<string, string>>("raw-way-tags"),
            Relations = CorrectiveCapturedReplay.ReadManifest<OsmRelation>(folder, "offline-osm-relations-chunks.json"),
            SupportWayIds = metadata["SupportWayIds"]!.ToObject<long[]>()!,
            MalformedFeaturesSkipped = (int)metadata["MalformedFeaturesSkipped"]!
        };

        var palette = JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder, "offline-palette-input.json")));
        var widths = palette["widths"]!.ToObject<Dictionary<string, float>>()!;
        var offsets = palette["laneOffsets"]!.ToObject<Dictionary<string, float[]>>()!;
        var log = new ImportLog { StrictRecovery = strict };
        var dataset = NetworkEvidence.Normalize(raw, request, log, default);
        var stages = new List<object>();
        SourceTopologyManifest? cleanupSoFar = null;
        var source = NetworkSourcePipeline.Prepare(dataset, request, widths, offsets, log, default, new SourceOptions
        {
            Skip = new HashSet<string>(skip),
            AfterStage = (name, sections, seconds) =>
        {
            // Topology after each step, against the reference of the lane-smoothing step.
            if (name == "lane-smoothing")
                cleanupSoFar = SourceTopologyObligations.Capture(sections);

            var audit = new ImportLog();

            if (cleanupSoFar != null)
                SourceTopologyObligations.Validate(cleanupSoFar, sections, audit, default);

            stages.Add(new
            {
                name,
                seconds,
                sections = sections.Count,
                missingPorts = audit.Metric("Missing mandatory selected-source ports"),
                unresolvedRoutes = audit.Metric("Unresolved selected-source reachability obligations"),
                highwayWays = sections
                    .Where(segment => NativeAssetKeys.IsHighway(segment))
                    .SelectMany(segment => segment.SourceWayIds.Append(segment.WayId))
                    .Distinct()
                    .Count()
            });
        }
        });

        // REPLAY_LOG_FILTER="TEXT": print the source pipeline's log lines containing TEXT.
        var logFilter = Environment.GetEnvironmentVariable("REPLAY_LOG_FILTER");

        if (!string.IsNullOrEmpty(logFilter))
            foreach (var line in log.Lines.Where(text => text.Contains(logFilter, StringComparison.Ordinal)))
                Console.WriteLine(line);

        var structures = NetworkStructureIntervals.UnmappedCrossingBridges(source.Sections, log, default);
        var bridgeSteps =
            new List<(string Step, double Kilometres)>
            {
                ("source", RoadBridgeKilometres(source.Sections)),
                ("unmapped crossings", RoadBridgeKilometres(structures))
            };

        var reaches =
            CorrectiveCapturedReplay.ReadManifest<WaterReachStage>(
                folder,
                (string)otherCapture.Input["waterReachesManifest"]!);

        structures =
            NetworkStructureIntervals.CulvertBridges(
                structures,
                reaches
                    .Where(reach => reach.PolygonId == null)
                    .Select(reach => (new[] { reach.Start, reach.End }, reach.HalfWidth))
                    .ToArray(),
                (int)Math.Sqrt(otherCapture.Terrain.Length),
                otherCapture.Width,
                log,
                default);

        bridgeSteps.Add(("over water channels", RoadBridgeKilometres(structures)));

        Console.WriteLine(
            $"channel crossings bridged: {log.Metric("Sections bridged over water channel footprints")} sections; "
                + "channel pieces by half width: "
                + string.Join(
                    ", ",
                    reaches
                        .Where(reach => reach.PolygonId == null)
                        .GroupBy(reach => (int)Math.Round(reach.HalfWidth / 10) * 10)
                        .OrderBy(group => group.Key)
                        .Select(group => $"{group.Key} m x{group.Count()}")));

        structures =
            NetworkStructureIntervals.StructureIntervals(
                structures,
                otherCapture.Terrain,
                otherCapture.Width,
                log,
                default,
                otherCapture.Water,
                tunnelIntervalsResolved: true);

        bridgeSteps.Add(("structure intervals", RoadBridgeKilometres(structures)));

        var withContracts =
            structures
                .Select(
                    segment => NativeAssetKeys.ApplyMetadata(
                        segment,
                        widths,
                        offsets) with
                    { NativeAssetKey = NativeAssetKeys.Sized(segment) })
                .ToArray();

        // As in the import: cleaned after the structure intervals, from the captured
        // design entry network, before the design reads the grid.
        var designCapture =
            DesignSwitches.On(DesignSwitches.SourceCleaning)
                ? otherCapture with
                {
                    Terrain =
                        TerrainCleaning.Apply(
                            otherCapture.Terrain,
                            otherCapture.Width,
                            otherCapture.Source,
                            otherCapture.Areas,
                            otherCapture.Lines,
                            log,
                            default,
                            CleaningWindow())
                }
                : otherCapture;

        // The import designs with the effective sea level, which lifts roads on ground
        // below it.
        var flood = ReadFloodInputs(otherCapture);
        var sea = flood.SeaSurface;
        Console.WriteLine($"sea level: {(float.IsNaN(sea) ? "not recorded" : sea.ToString("0.0") + " m")}");

        // As in the import: flood defences, then terrain extremes, before design.
        bool[]? defences = null;
        float[]? crests = null;

        if (DesignSwitches.On(DesignSwitches.FloodDefences) && !float.IsNaN(sea))
        {
            var built =
                FloodDefenceData.Apply(
                    designCapture.Terrain,
                    designCapture.Scale,
                    designCapture.Width,
                    sea,
                    flood.Ocean,
                    designCapture.Areas,
                    flood.Lines,
                    log,
                    default,
                    flood.Water,
                    flood.Polygons);

            designCapture = designCapture with { Terrain = built.Heights };
            defences = built.Defence;
            crests = built.Crest;
        }

        if (DesignSwitches.On(DesignSwitches.SmoothExtremes))
            designCapture = designCapture with
            {
                Terrain =
                    NetworkTerrainExtremes.Prepare(
                        designCapture.Terrain,
                        designCapture.Width,
                        Array.Empty<PreparedSegment>(),
                        designCapture.Areas,
                        designCapture.Lines,
                        log,
                        default,
                        defences,
                        float.IsNaN(sea) ? float.NegativeInfinity : sea)
            };

        foreach (var line in log.Lines.Where(
            text => text.Contains("Source cleaning", StringComparison.Ordinal)
                || text.Contains("Flood defences", StringComparison.Ordinal)
                || text.Contains("Terrain extremes", StringComparison.Ordinal)))
            Console.WriteLine(line);

        IReadOnlyList<PreparedSegment> designed = Array.Empty<PreparedSegment>();
        var contracts = otherCapture.Contracts.ToDictionary(contract => contract.PaletteKey);
        Measure(
            designCapture with { Source = withContracts },
            output,
            "road-replay-visual-compiler",
            (
                capture,
                otherLog) =>
            {
                designed =
                    NativeAssetKeys.Refresh(
                        VisualNetworkCompiler.Prepare(
                            capture.Source,
                            capture.Terrain,
                            capture.Width,
                            otherLog,
                            default,
                            capture.Water,
                            sea,
                            Environment.GetEnvironmentVariable("ROAD_DYKE_FLOOR") == "off" ? null : crests),
                        contracts);

                foreach (var name in new[] { "High fills built as bridge", "Deep cuttings built as tunnel" })
                    log.Count(name, (int)otherLog.Metric(name));

                foreach (var line in otherLog.Lines.Where(text => text.Contains("sea from edge", StringComparison.Ordinal)))
                    Console.WriteLine(line);

                foreach (var metric in otherLog.Metrics().Where(metric => metric.Key.StartsWith("Smooth roads")))
                    Console.WriteLine($"  {metric.Key}: {metric.Value:0.##}");

                foreach (var reason in new[] { "railway", "roundabout", "major road" })
                    Console.WriteLine(
                        $"short underpasses kept, {reason} over: {otherLog.Metric("Short underpasses kept, " + reason + " over")}");

                return designed;
            });

        bridgeSteps.Add(("design", RoadBridgeKilometres(designed)));

        WriteRoadQuality(
            designCapture,
            otherCapture.Terrain,
            defences,
            crests,
            sea,
            output,
            designed,
            dataset,
            request,
            source,
            stages,
            bridgeSteps,
            log);
    }

    // The cleaning window for the B1 sweep (TERRAIN_CLEANING_WINDOW, metres).
    private static float CleaningWindow()
        => float.TryParse(
            Environment.GetEnvironmentVariable("TERRAIN_CLEANING_WINDOW"),
            System.Globalization.NumberStyles.Float,
            System.Globalization.CultureInfo.InvariantCulture,
            out var metres)
            ? metres
            : TerrainCleaning.WindowMeters;

    private static void WriteRoadQuality(
        Capture otherCapture,
        float[] sourceTerrain,
        bool[]? defences,
        float[]? crests,
        float sea,
        string output,
        IReadOnlyList<PreparedSegment> designed,
        OsmDataset dataset,
        ImportRequest request,
        SourceResult source,
        List<object> stages,
        List<(string Step, double Kilometres)> bridgeSteps,
        ImportLog log)
    {
        var final =
            NativeDesignUnits.Prepare(
                StarterNetworkPolicy.WithoutSideStructures(designed),
                otherCapture.Contracts,
                new ImportLog(),
                default,
                segment => VisualCompilerTable.For(segment).MinimumSegment);

        foreach (var metric in log.Metrics().Where(metric => metric.Key.StartsWith("Smooth roads")))
            Console.WriteLine($"  {metric.Key}: {metric.Value:0.##}");

        var drawnLog = new ImportLog();

        final =
            NetworkDrawnStructures.Prepare(
                final,
                otherCapture.Terrain,
                otherCapture.Width,
                drawnLog);

        foreach (var metric in drawnLog.Metrics().Where(metric => metric.Key.Contains("unnel")))
            Console.WriteLine($"  {metric.Key}: {metric.Value:0.##}");

        bridgeSteps.Add(("beside a bridge, before earthworks", RoadBridgeKilometres(final)));

        final =
            NativeEndpointPolicy.Canonicalize(
                NativeEndpointPolicy.SnapToBorder(final, otherCapture.Width, new ImportLog()),
                new ImportLog());

        var quality = RoadQualityReport.Measure(final, dataset, request, default);
        var roadbed =
            NetworkRoadbed.Prepare(
                otherCapture.Terrain,
                otherCapture.Scale,
                otherCapture.Width,
                final,
                otherCapture.Areas,
                otherCapture.Lines,
                log,
                default);

        if (crests != null)
        {
            log.Count(
                "Flood defence cells restored after earthworks",
                FloodDefences.Restore(roadbed, crests));

            // Ground road points over a dyke whose deck stands more than 0.25 m below the
            // highest crest under the road's footprint (as the compiler reads it): the
            // game levels the ground to the road there and cuts a notch.
            int over = 0, below = 0;

            var groundRoads =
                final.Where(segment => segment.Kind == NetworkKind.Road && !segment.Bridge && !segment.Tunnel);

            // DYKE_WAYS="id,id": each of those ways' ground points with the crest under it.
            var traced =
                (Environment.GetEnvironmentVariable("DYKE_WAYS") ?? "")
                    .Split(',', StringSplitOptions.RemoveEmptyEntries)
                    .Select(long.Parse)
                    .ToHashSet();

            foreach (var segment in final.Where(segment => traced.Contains(segment.WayId)))
                for (var k = 0; k <= 8; k++)
                {
                    var point = NetworkNormalizer.Evaluate(segment, k / 8f);
                    var crest =
                        VisualNetworkCompiler.HighestCrest(
                            crests,
                            point,
                            Math.Max(4, segment.NativeWidth / 2),
                            otherCapture.Width);

                    Console.WriteLine(
                        $"DYKE WAY {segment.WayId} x {point.x:F0} z {point.z:F0} deck {point.y:F2} crest {crest:F2} ground {NetworkProcessor.Sample(point.x, point.z, roadbed, otherCapture.Width):F2} bridge {segment.Bridge}");
                }

            foreach (var segment in groundRoads)
                for (var k = 0; k <= 16; k++)
                {
                    var point = NetworkNormalizer.Evaluate(segment, k / 16f);
                    var halfWidth = Math.Max(4, segment.NativeWidth / 2);
                    var highest = VisualNetworkCompiler.HighestCrest(crests, point, halfWidth, otherCapture.Width);

                    if (float.IsNaN(highest))
                        continue;

                    over++;
                    below += point.y < highest - .25f ? 1 : 0;
                }

            log.Count("Road points over a dyke", over);
            log.Count("Road points over a dyke below its crest", below);
            Console.WriteLine($"DYKE ROADS points over a dyke {over}; below its crest by over 0.25 m {below}");
        }

        if (DesignSwitches.On(DesignSwitches.SmoothExtremes))
            roadbed =
                NetworkTerrainExtremes.Prepare(
                    roadbed,
                    otherCapture.Width,
                    final,
                    otherCapture.Areas,
                    otherCapture.Lines,
                    log,
                    default,
                    defences,
                    float.IsNaN(sea) ? float.NegativeInfinity : sea);

        var finishLog = new ImportLog();
        var beforeEarthworks = final;
        final = NetworkDrawnStructures.Finish(final, roadbed, otherCapture.Width, finishLog);
        bridgeSteps.Add(("after earthworks", RoadBridgeKilometres(final)));
        var drawn = NetworkDrawnStructures.Predict(final, roadbed, otherCapture.Width);

        Console.WriteLine(
            $"tunnel mouths behind the one beside them: {finishLog.Metric("Tunnel mouths behind the one beside them")}; "
                + $"moved in beside it: {finishLog.Metric("Tunnel mouths moved in beside the one next to them")}; "
                + $"portal cuts: {finishLog.Metric("Portal sections cut so only the mouth draws open")}");

        foreach (var reason in new[] { "far end is a junction", "past the reach", "part of it is low" })
            Console.WriteLine(
                $"bridge approach still on the ground, {reason}: {finishLog.Metric("Bridge approach on the ground after earthworks, " + reason)} sections, {finishLog.Metric("Bridge approach metres after earthworks, " + reason)} m");

        Console.WriteLine(
            $"viaducts: {finishLog.Metric("Sections clear of the finished ground drawn as viaduct")}; approaches joined after earthworks: {finishLog.Metric("Ground sections still clear after earthworks drawn as elevated")}; wall km {drawn.WallKm:F2}; cut km {drawn.CutKm:F2}");

        float Middle(PreparedSegment segment)
        {
            var point = NetworkNormalizer.Evaluate(segment, .5f);
            return NetworkProcessor.Sample(point.x, point.z, roadbed, otherCapture.Width) - point.y;
        }

        var openArm =
            new HashSet<long>(
                final
                    .Where(segment => segment.Kind == NetworkKind.Road && !segment.Tunnel)
                    .SelectMany(segment => new[] { segment.StartNodeId, segment.EndNodeId }));

        var openBores =
            final
                .Where(
                    segment => segment.Kind == NetworkKind.Road
                        && segment.Tunnel
                        && !segment.Bridge
                        && ((segment.StartElevation ?? -8) > -10
                            || (segment.EndElevation ?? -8) > -10
                            || Middle(segment) < 10.5))
                .ToArray();

        File.WriteAllLines(
            Path.Combine(output, "open-bores.txt"),
            openBores
                .Select(segment =>
        {
            var cover = Enumerable.Range(0, 9).Select(i =>
{
    var point = NetworkNormalizer.Evaluate(segment, i / 8f);
    return NetworkProcessor.Sample(point.x, point.z, roadbed, otherCapture.Width) - point.y;
}).ToArray();

            return $"{NetworkVerticalProfiles.ArcLength(segment),6:F0} {segment.Classification,-13} way {segment.WayId} open {(openArm.Contains(segment.StartNodeId) ? "S" : "-")}{(openArm.Contains(segment.EndNodeId) ? "E" : "-")} el {segment.StartElevation,6:F1} {segment.EndElevation,6:F1} cover {string.Join(" ", cover.Select(v => v.ToString("F0")))} at {segment.CourseStart.x:F0},{segment.CourseStart.z:F0}";
        }).OrderByDescending(text => text));

        var measured = VisualCompilerMetrics.Measure(final, roadbed, otherCapture.Width, default);
        File.WriteAllLines(
            Path.Combine(output, "omissions.txt"),
            final
                .Where(
                    segment => segment.StartNodeId == segment.EndNodeId
                        || !NetworkDesignChecks.HasHorizontalExtent(
                            new(
                                segment.CourseStart,
                                segment.CourseControl1,
                                segment.CourseControl2,
                                segment.CourseEnd)))
                .Select(
                    segment => $"way {segment.WayId} {segment.Classification} nodes {segment.StartNodeId},{segment.EndNodeId} at {segment.CourseStart.x:F1},{segment.CourseStart.z:F1} to {segment.CourseEnd.x:F1},{segment.CourseEnd.z:F1} tunnel={segment.Tunnel} bridge={segment.Bridge}"));

        var issues =
            VisualCompilerMetrics
                .Cluster(measured.Failures, default)
                .GroupBy(issue => issue.Category)
                .ToDictionary(group => group.Key, group => group.Count());

        var raw = new ImportLog();
        SourceTopologyObligations.Validate(source.RawTopology, final, raw, default);
        var cleanup = new ImportLog();
        SourceTopologyObligations.Validate(source.CleanupTopology, final, cleanup, default);
        var omissions =
            final.Count(
                segment => segment.StartNodeId == segment.EndNodeId
                    || !NetworkDesignChecks.HasHorizontalExtent(
                        new(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd)));

        // Report 112's measures of the plan, as the analysis tools compute them on a game run.
        var roadEdges =
            final
                .Where(segment => segment.Kind == NetworkKind.Road)
                .Select(
                    segment => new MeasureEdge(
                        segment.StartNodeId.ToString(),
                        segment.EndNodeId.ToString(),
                        Enumerable.Range(0, 9).Select(i => NetworkNormalizer.Evaluate(segment, i / 8f)).ToArray(),
                        segment.Tunnel ? "tunnel" : segment.Bridge ? "bridge" : "ground",
                        segment.NativeAssetKey,
                        segment.OneWay))
                .ToArray();

        var plan = PlanMeasures.Roads(roadEdges);
        plan["terrain"] =
            PlanMeasures.Terrain(
                sourceTerrain,
                roadbed,
                otherCapture.Width,
                roadEdges.Where(edge => edge.Structure == "ground").Select(edge => edge.Points),
                designBaseline: otherCapture.Terrain);

        // Where the steps already in the grid before design are: beside mapped water,
        // waterways, cliffs and coast (water planning and real edges), or elsewhere.
        var size = (int)Math.Sqrt(sourceTerrain.Length);
        var water = TerrainExtremes.Protected(size, otherCapture.Width, otherCapture.Areas, otherCapture.Lines);
        var sourceSteps = PlanMeasures.Steps(sourceTerrain, size);
        var besideWater =
            sourceSteps.Count(
                step => Enumerable.Range(-2, 5).Any(
                    dz => Enumerable.Range(-2, 5).Any(
                        dx => water[(step.Z + dz) * size + step.X + dx] != 0)));

        Console.WriteLine(
            $"steps before design: {sourceSteps.Count}; within 2 cells of mapped water, waterways, cliffs or coast: {besideWater}");

        // Why ground roads stand more than 2 m off the designed ground: in mapped water or
        // quarries, beside waterway, cliff or coast lines (the roadbed guards both), deeper
        // than the class budget, or none of these.
        var areas = TerrainExtremes.Protected(size, otherCapture.Width, otherCapture.Areas, Array.Empty<float3[]>());
        var lines = TerrainExtremes.Protected(size, otherCapture.Width, Array.Empty<float3[]>(), otherCapture.Lines);
        var stoppedShort = new Dictionary<string, double>();

        foreach (var segment in final.Where(segment => segment.Kind == NetworkKind.Road && !NetworkStructure.IsStructure(segment)))
        {
            for (var i = 1; i <= 8; i++)
            {
                var point = NetworkNormalizer.Evaluate(segment, i / 8f);
                var ground = NetworkProcessor.Sample(point.x, point.z, roadbed, otherCapture.Width);

                if (Math.Abs(point.y - ground) <= 2)
                    continue;

                var reason = OffGroundReason(segment, point, areas, lines, otherCapture.Terrain, otherCapture.Width);

                stoppedShort[reason] = stoppedShort.GetValueOrDefault(reason) + NetworkVerticalProfiles.ArcLength(segment) / 8 / 1000;
            }
        }

        Console.WriteLine(
            "stopped short by reason (km): "
                + string.Join("; ", stoppedShort.OrderByDescending(entry => entry.Value).Select(entry => $"{entry.Key} {entry.Value:0.0}")));

        var causes =
            Causes(
                beforeEarthworks,
                final,
                openBores,
                bridgeSteps,
                roadbed,
                otherCapture.Terrain,
                otherCapture.Width,
                areas,
                lines,
                sea);

        // Earthwork volumes against the grid the design started from.
        var cellArea = Math.Pow(otherCapture.Width / (Math.Sqrt(roadbed.Length) - 1), 2);
        double fill = 0;
        double cut = 0;

        for (var i = 0; i < roadbed.Length; i++)
        {
            var change = (roadbed[i] - otherCapture.Terrain[i]) * cellArea;

            if (change > 0)
                fill += change;
            else
                cut -= change;
        }

        plan["terrain"]!["fillMillionM3"] = Math.Round(fill / 1e6, 1);
        plan["terrain"]!["cutMillionM3"] = Math.Round(cut / 1e6, 1);

        foreach (var name in new[] {
            "Terrain spikes and pits removed",
            "High fills built as bridge",
            "Deep cuttings built as tunnel"
            })
            plan["terrain"]![name] = log.Metric(name);

        File.WriteAllText(Path.Combine(output, "road-quality.json"), JsonConvert.SerializeObject(new
        {
            schema = 1,
            plan = JsonConvert.DeserializeObject(plan.ToString()),
            capture = Path.GetFileName(otherCapture.Folder),
            quality.Classes,
            issues,
            drawn,
            causes,
            topology = new
            {
                raw = new
                {
                    ports = raw.Metric("Missing mandatory selected-source ports"),
                    routes = raw.Metric("Unresolved selected-source reachability obligations")
                },
                cleanup = new
                {
                    ports = cleanup.Metric("Missing mandatory selected-source ports"),
                    routes = cleanup.Metric("Unresolved selected-source reachability obligations")
                }
            },
            recoveries = log.Metric("OSM recovery stages"),
            predictedOmissions = omissions,
            stages
        }, Formatting.Indented));

        File.WriteAllText(
            Path.Combine(output, "plan-hash.json"),
            JsonConvert.SerializeObject(
                new
                {
                    source = PlanHash.Segments(source.Sections),
                    designed = PlanHash.Segments(designed),
                    final = PlanHash.Segments(final),
                    designTerrain = PlanHash.Heights(otherCapture.Terrain),
                    roadbed = PlanHash.Heights(roadbed)
                },
                Formatting.Indented));

        WriteRoadSites(output, final, source, dataset, request);
        Console.WriteLine("Road quality saved: " + Path.Combine(output, "road-quality.json"));
    }

    // Sorts each topology failure: a real break has no drivable path in the whole
    // network, the others are identity or provenance limits of the check itself.
    private static void WriteRoadSites(
        string output,
        IReadOnlyList<PreparedSegment> final,
        SourceResult source,
        OsmDataset dataset,
        ImportRequest request)
    {
        var check = new ImportLog { CaptureRecords = true };
        SourceTopologyObligations.Validate(source.CleanupTopology, final, check, default);
        var forward = new Dictionary<(NetworkKind, long), List<long>>();
        var both = new Dictionary<(NetworkKind, long), List<long>>();

        void Arc(Dictionary<(NetworkKind, long), List<long>> lookup, NetworkKind kind, long a, long b)
        {
            if (!lookup.TryGetValue((kind, a), out var l))
                lookup[(kind, a)] = l = new();

            l.Add(b);
        }

        foreach (var segment in final)
        {
            Arc(forward, segment.Kind, segment.StartNodeId, segment.EndNodeId);

            if (!segment.OneWay)
                Arc(forward, segment.Kind, segment.EndNodeId, segment.StartNodeId);

            Arc(both, segment.Kind, segment.StartNodeId, segment.EndNodeId);
            Arc(both, segment.Kind, segment.EndNodeId, segment.StartNodeId);
        }

        bool Reach(Dictionary<(NetworkKind, long), List<long>> lookup, NetworkKind kind, long a, long b)
        {
            var seen = new HashSet<long> { a };
            var q = new Queue<long>();
            q.Enqueue(a);

            while (q.Count > 0 && seen.Count < 20000)
            {
                var nodeId = q.Dequeue();

                if (nodeId == b)
                    return true;

                if (lookup.TryGetValue((kind, nodeId), out var l))
                    foreach (var m in l)
                        if (seen.Add(m))
                            q.Enqueue(m);
            }

            return false;
        }

        var sites = new List<JObject>();
        var aliases = new Dictionary<SourceNodeKey, HashSet<long>>();
        var nodes = new HashSet<SourceNodeKey>();

        foreach (var segment in final)
        {
            nodes.Add(new(segment.Kind, segment.StartNodeId));
            nodes.Add(new(segment.Kind, segment.EndNodeId));

            foreach (var alias in segment.SourceNodeAliases)
            {
                var nodeKey = new SourceNodeKey(segment.Kind, alias.SourceNodeId);

                if (!aliases.TryGetValue(nodeKey, out var t))
                    aliases[nodeKey] = t = new();

                t.Add(alias.CanonicalNodeId);
            }
        }

        long? Canonical(SourceNodeKey nodeKey)
        {
            if (aliases.TryGetValue(nodeKey, out var t))
            {
                if (t.Count != 1)
                    return null;

                var otherNodeKey = new SourceNodeKey(nodeKey.Kind, t.First());

                return nodes.Contains(otherNodeKey) ? otherNodeKey.Id : null;
            }

            return nodes.Contains(nodeKey) ? nodeKey.Id : null;
        }

        var claimed = new HashSet<long>(final.SelectMany(segment => segment.SourceWayIds.Append(segment.WayId)));
        var records = check.Records["source-topology-obligations.ndjson"].Select(JObject.FromObject).ToArray();
        var connections = source.CleanupTopology.Connections;

        for (var i = 0; i < connections.Count; i++)
        {
            if ((bool)records[i]["reachable"]!)
                continue;

            var connection = connections[i];
            var kind = connection.From.Kind;
            var from = Canonical(connection.From);
            var to = Canonical(connection.To);
            var verdict =
                from == null || to == null
                    ? (connection.Ways.All(claimed.Contains)
                        ? "identity-lost-road-present"
                        : "identity-lost-road-removed")
                    : Reach(forward, kind, from.Value, to.Value)
                        ? "connected-by-other-roads"
                        : Reach(both, kind, from.Value, to.Value) ? "connected-against-one-way" : "real-break";

            var way =
                connection.Ways.Where(dataset.Ways.ContainsKey).Select(wayId => dataset.Ways[wayId]).FirstOrDefault();

            var node =
                way?.NodeIds.Where(dataset.Nodes.ContainsKey).Select(nodeId => dataset.Nodes[nodeId]).FirstOrDefault();

            var classification = "";

            if (way != null)
            {
                way.Tags.TryGetValue("highway", out var text);
                way.Tags.TryGetValue("railway", out var rail);
                classification = text ?? rail ?? "";
            }

            sites.Add(
                new JObject
                {
                    ["verdict"] = verdict,
                    ["class"] = classification,
                    ["kind"] = kind.ToString(),
                    ["latitude"] = node?.Latitude,
                    ["longitude"] = node?.Longitude,
                    ["ways"] = new JArray(connection.Ways.Take(4))
                });
        }

        File.WriteAllText(
            Path.Combine(output, "road-sites.json"),
            JsonConvert.SerializeObject(sites, Formatting.Indented));

        Console.WriteLine(
            "ROAD SITES "
                + string.Join(
                    "; ",
                    sites
                        .GroupBy(row => (string)row["verdict"]! + "/" + (string)row["kind"]!)
                        .OrderByDescending(group => group.Count())
                        .Select(group => group.Key + "=" + group.Count())));
    }

    // Fails when a report is worse than its stored baseline. Limits allow small
    // churn, so a baseline only moves with a recorded reason.
    public static int RoadCompare(string reportPath, string baselinePath)
    {
        var report = JObject.Parse(File.ReadAllText(reportPath));
        var baseline = JObject.Parse(File.ReadAllText(baselinePath));
        var failures = new List<string>();

        void Cap(string name, double value, double was, double allowance)
        {
            if (value > was + allowance)
                failures.Add($"{name}: {value:0.##} against baseline {was:0.##}");
        }

        double Count(double was) => Math.Max(3, was * .01);

        var old = baseline["Classes"]!.ToDictionary(token => (string)token["Class"]!);

        foreach (var row in report["Classes"]!)
        {
            var name = (string)row["Class"]!;

            if (!old.TryGetValue(name, out var was))
                continue;

            double ReadNumber(JToken token, string key) => (double)token[key]!;

            Cap(name + " lateral p95", ReadNumber(row, "LateralP95"), ReadNumber(was, "LateralP95"), .25);
            Cap(name + " lateral max", ReadNumber(row, "LateralMaximum"), ReadNumber(was, "LateralMaximum"), 1);
            Cap(name + " km over 10 m", ReadNumber(row, "KilometresOver10"), ReadNumber(was, "KilometresOver10"), .5);

            foreach (var key in new[] {
                "Jogs",
                "KinksAtPlainNodes",
                "KinksThroughJunctions",
                "UnderRadiusFloor",
                "UnderMinimumLength"
                })
                Cap(name + " " + key, ReadNumber(row, key), ReadNumber(was, key), Count(ReadNumber(was, key)));

            Cap(
                name + " bridge km",
                ReadNumber(row, "BridgeKilometres"),
                ReadNumber(was, "BridgeKilometres"),
                Math.Max(.5, ReadNumber(was, "BridgeKilometres") * .02));

            Cap(
                name + " missing km",
                ReadNumber(row, "MissingSourceKilometres"),
                ReadNumber(was, "MissingSourceKilometres"),
                Math.Max(.1, ReadNumber(was, "MissingSourceKilometres") * .01));
        }

        // What the game should draw: long open trenches and walls are what shows in game.
        if (baseline["drawn"] is JObject drawn)
            foreach (var (key, allowance) in new[] {
                ("OpenBoreKm", .3),
                ("DeepOpenBoreKm", .2),
                ("LongestOpenBore", 15),
                ("CutKm", .3),
                ("WallKm", .3),
                ("ForcedShallowKm", .2)
                })
                Cap("drawn " + key, (double)report["drawn"]![key]!, (double)drawn[key]!, allowance);

        // Report 112's plan measures, once a baseline carries them.
        if (baseline["plan"] is JObject plan && report["plan"] is JObject now)
        {
            double Read(JObject root, string path) => (double?)root.SelectToken(path) ?? 0;

            void Rate(string path, double allowance) => Cap("plan " + path, Read(now, path), Read(plan, path), allowance);

            Rate("junctionsInClosePairShare", .002);
            Rate("edgesUnder10mPer100Km", Math.Max(.5, Read(plan, "edgesUnder10mPer100Km") * .02));
            Rate("throughNodesPer100Km", Math.Max(5, Read(plan, "throughNodesPer100Km") * .01));
            Rate("terrain.steps.design", Count(Read(plan, "terrain.steps.design")));
            Rate("terrain.spikes.count", 3);
            Rate("terrain.groundKmStoppedShort", Math.Max(.5, Read(plan, "terrain.groundKmStoppedShort") * .02));
            Rate("terrain.groundKmNeedingOver12m", Math.Max(.5, Read(plan, "terrain.groundKmNeedingOver12m") * .02));
        }

        foreach (var issue in ((JObject)report["issues"]!).Properties())
        {
            var was = (double?)baseline["issues"]?[issue.Name] ?? 0;
            Cap("issues " + issue.Name, (double)issue.Value, was, Count(was));
        }

        foreach (var scope in new[] { "raw", "cleanup" })
            foreach (var key in new[] { "ports", "routes" })
            {
                var was = (double)baseline["topology"]![scope]![key]!;
                Cap($"topology {scope} {key}", (double)report["topology"]![scope]![key]!, was, 0);
            }

        Cap("recoveries", (double)report["recoveries"]!, 0, 0);
        Cap("predicted omissions", (double)report["predictedOmissions"]!, (double)baseline["predictedOmissions"]!, 0);

        foreach (var text in failures)
            Console.WriteLine("WORSE: " + text);

        Console.WriteLine(
            failures.Count == 0 ? "Road quality within baseline." : $"{failures.Count} measures worse than baseline.");

        return failures.Count == 0 ? 0 : 1;
    }
}
