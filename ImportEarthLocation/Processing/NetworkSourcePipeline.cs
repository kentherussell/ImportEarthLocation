using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;

namespace ImportEarthLocation.Processing;

public sealed class SourceOptions
{
    // Stage names to leave out, for measuring what a stage contributes.
    public ISet<string> Skip { get; init; } = new HashSet<string>(StringComparer.Ordinal);
    public Action<string, int>? BeforeStage { get; init; }
    public Action<string, IReadOnlyList<PreparedSegment>, double>? AfterStage { get; init; }
}

public sealed record SourceResult(
    IReadOnlyList<PreparedSegment> Sections,
    SourceTopologyManifest RawTopology,
    SourceTopologyManifest CleanupTopology);

// The one order of source writers, from parsed OSM to the graph the water and
// structure steps receive. The game and --road-replay both run this, so a replay
// measures the code the game runs. RawTopology is taken right after the tunnel
// intervals; CleanupTopology after lane smoothing, the game's reference.
public static class NetworkSourcePipeline
{
    private readonly record struct Stage(
        string Name,
        bool Recoverable,
        bool Required,
        Func<IReadOnlyList<PreparedSegment>, IReadOnlyList<PreparedSegment>> Run);

    // A source port merged away, including one lane smoothing has since made plain, keeps its identity as an alias of the merged section's start.
    private static IReadOnlyList<PreparedSegment> Normalized(
        IReadOnlyList<PreparedSegment> input,
        IReadOnlyCollection<SourceNodeKey> earlierPorts,
        ImportLog log,
        CancellationToken token,
        double width)
    {
        var merged = new Dictionary<SourceNodeKey, long>();
        IReadOnlyList<PreparedSegment> result = NetworkNormalizer.Normalize(input, log, token, width, merged);
        var transferClock = System.Diagnostics.Stopwatch.StartNew();

        var kept = new HashSet<SourceNodeKey>();

        foreach (var segment in result)
        {
            kept.Add(new SourceNodeKey(segment.Kind, segment.StartNodeId));
            kept.Add(new SourceNodeKey(segment.Kind, segment.EndNodeId));
        }

        // The ports merged away, by kind, each mapped to the node it was merged into. Transfer
        // sorts every section's aliases by source node, so the order they are found in is not seen.
        var inputPorts = SourceTopologyObligations.Ports(input);
        var lost = new Dictionary<NetworkKind, Dictionary<long, long>>();

        void Consider(SourceNodeKey nodeKey)
        {
            if (!merged.TryGetValue(nodeKey, out var target) || kept.Contains(nodeKey))
                return;

            if (!lost.TryGetValue(nodeKey.Kind, out var remap))
                lost[nodeKey.Kind] = remap = new();

            remap[nodeKey.Id] = target;
        }

        foreach (var nodeKey in inputPorts)
            Consider(nodeKey);

        foreach (var nodeKey in earlierPorts)
            Consider(nodeKey);

        foreach (var pair in lost)
            result = SourceTopologyObligations.Transfer(input, result, pair.Key, pair.Value, inputPorts);

        log.AddDuration("Normalization port transfer", transferClock.Elapsed.TotalSeconds);

        return result;
    }

    public static SourceResult Prepare(
        OsmDataset dataset,
        ImportRequest request,
        IReadOnlyDictionary<string, float> widths,
        IReadOnlyDictionary<string, float[]> laneOffsets,
        ImportLog log, CancellationToken token, SourceOptions? options = null)
    {
        options ??= new SourceOptions();
        var width = request.MapSize.WidthMeters;
        var earlierPorts = new List<SourceNodeKey>();
        var stages = new[]
        {
            new Stage("limited-access",false,false,segments=>NetworkLimitedAccess.Apply(segments,dataset,log)),
            new Stage("regular-links",false,false,segments=>NetworkLimitedAccess.RegularLinks(segments,dataset,log)),
            new Stage(
                "tunnel-intervals",
                false,
                true,
                segments=>NetworkStructureIntervals.TunnelIntervals(segments,log,token)),
            new Stage("stubs",false,false,segments=>NetworkStubs.Prune(segments,request,log)),
            new Stage("lane-smoothing",false,false,segments=>NetworkLaneRuns.Smooth(segments,log)),
            new Stage("median-links",true,false,segments=>MedianLinks.Prepare(segments,log,token)),
            new Stage("avenue-consolidation",true,false,segments=>AvenueConsolidator.Prepare(segments,log,token)),
            new Stage("street-widths",false,false,segments=>NetworkLaneRuns.NormalizeStreets(segments,log)),
            new Stage("junction-clusters",true,false,segments=>StreetJunctionClusters.Prepare(segments,log,token)),
            new Stage("split-roads",true,false,segments=>NetworkSplitRoads.Prepare(segments,log,token)),
            new Stage("roundabouts",true,false,segments=>RoundaboutPlanner.Prepare(segments,log,token)),
            new Stage("teardrops",false,false,segments=>NetworkDeadEnds.RemoveTeardrops(segments,log)),
            new Stage(
                "rail-consolidation",
                true,
                false,
                segments=>request.ImportRail?RailCorridorConsolidator.Prepare(segments,log,token):segments),
            new Stage("normalization",true,false,segments=>Normalized(segments,earlierPorts,log,token,width)),
            new Stage(
                "network-hardening",
                true,
                false,
                segments=>NetworkHardener.Prepare(segments,log,token,request.ImportRail)),
            new Stage("repair-geometry",false,true,segments=>OsmRecovery.RepairGeometry(segments,width,log,token)),
            new Stage(
                "width-changes",
                false,
                false,
                segments=>DesignSwitches.On(DesignSwitches.SmoothRoads)
                    ? NetworkLaneRuns.MoveWidthChanges(segments,log)
                    : segments),
            new Stage(
                "native-metadata",
                false,
                true,
                segments=>segments
                    .Select(segment=>NativeAssetKeys.ApplyMetadata(segment,widths,laneOffsets))
                    .ToArray()),
            new Stage("native-junctions",false,false,segments=>NativeJunctionSimplifier.Prepare(segments,log,token)),
            new Stage(
                "coincident-courses",
                false,
                false,
                segments=>NetworkNormalizer.RemoveCoincidentCourses(segments,log,token)),
            new Stage("late-teardrops",false,false,segments=>NetworkDeadEnds.RemoveTeardrops(segments,log)),
            new Stage("late-stubs",true,false,segments=>NetworkDeadEnds.PruneStubs(segments,request,log)),
            new Stage(
                "street-connectors",
                true,
                false,
                segments=>NetworkDeadEnds.ConnectThroughStreets(segments,dataset,request,widths,laneOffsets,log,token)),
            new Stage("dead-ends",true,false,segments=>NetworkDeadEnds.Close(segments,width,log,token)),
            new Stage("map-edge",true,false,segments=>NetworkDeadEnds.ReachBorder(segments,width,log)),
            new Stage("blind-one-ways",true,false,segments=>NetworkDeadEnds.RemoveBlindOneWays(segments,width,log)),
            new Stage("loose-pieces",true,false,segments=>NetworkDeadEnds.RemoveLoosePieces(segments,width,log)),
        };

        foreach (var name in options.Skip)
            if (!stages.Any(stage => stage.Name == name && !stage.Required))
                throw new ArgumentException("Stage cannot be skipped: " + name);

        var buildClock = System.Diagnostics.Stopwatch.StartNew();
        var sections = NetworkProcessor.BuildSegments(dataset, request, log, token);
        log.Time("Source stage build-segments", buildClock.Elapsed.TotalSeconds);
        SourceTopologyManifest? raw = null;
        SourceTopologyManifest? cleanup = null;

        foreach (var stage in stages)
        {
            options.BeforeStage?.Invoke(stage.Name, sections.Count);

            if (!options.Skip.Contains(stage.Name))
            {
                var clock = System.Diagnostics.Stopwatch.StartNew();
                var input = sections;
                sections =
                    stage.Recoverable
                        ? OsmRecovery.Run(stage.Name, () => stage.Run(input), input, log, token)
                        : stage.Run(input);

                options.AfterStage?.Invoke(stage.Name, sections, clock.Elapsed.TotalSeconds);
            }

            if (stage.Name == "tunnel-intervals")
            {
                raw = SourceTopologyObligations.Capture(sections);
                earlierPorts.AddRange(raw.Ports);
            }

            if (stage.Name == "lane-smoothing")
            {
                cleanup = SourceTopologyObligations.Capture(sections);
                earlierPorts.AddRange(cleanup.Ports);
            }
        }

        return new(sections, raw!, cleanup!);
    }
}
