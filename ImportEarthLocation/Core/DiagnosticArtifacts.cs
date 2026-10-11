using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Mathematics;

namespace ImportEarthLocation.Core;

// Explicit DTOs avoid serializing Unity swizzles or recursive HorizontalParent
// graphs. Local evidence only; no endpoint credentials, uploads or game mutation.
public static class DiagnosticArtifacts
{
    public static object Point(float3 p) => new { x = p.x, y = p.y, z = p.z };
    private static object? Point(float3? p) => p.HasValue ? Point(p.Value) : null;
    private static bool EvidenceTag(string key) => key == "name" || key == "ref" || key == "highway" || key == "railway" ||
        key == "oneway" || key == "junction" || key == "layer" || key == "bridge" || key == "tunnel" || key == "covered" ||
        key == "location" || key == "service" || key == "tracks" || key == "gauge" || key == "width" || key == "incline" ||
        key == "embankment" || key == "cutting" || key == "lanes" || key.StartsWith("lanes:", StringComparison.Ordinal) ||
        key.StartsWith("turn:lanes", StringComparison.Ordinal) || key.StartsWith("maxspeed", StringComparison.Ordinal);
    public static IEnumerable<object> NetworkRecords(IReadOnlyList<PreparedSegment> sections)
    {
        for (var index = 0; index < sections.Count; index++)
        {
            var s = sections[index];
            yield return new
            {
                Index = index, s.WayId, s.StartNodeId, s.EndNodeId, Start = Point(s.Start), End = Point(s.End),
                Control1 = Point(s.Control1), Control2 = Point(s.Control2), Kind = s.Kind.ToString(), s.Classification,
                s.Lanes, s.OneWay, s.Bridge, s.Tunnel, s.Layer, s.Name, s.Reference, s.RailService, s.SourceWayIds,
                s.StartTerminal, s.EndTerminal, s.PhysicalTracks, s.BothWaysLanes, s.ForwardLanes, s.BackwardLanes,
                s.ExplicitLanes, s.SourceWidthMeters, s.SourceSpeedKph, s.ExplicitIncline, s.CorridorId, s.ConsolidatedAvenue,
                s.NativeWidth, s.MaximumNativeWidth, s.NativeLaneOffsets, s.StartElevation, s.EndElevation,
                StartCurveOffset = Point(s.StartCurveOffset), EndCurveOffset = Point(s.EndCurveOffset),
                s.StartMiniRoundabout, s.EndMiniRoundabout, s.StartRoundaboutRadius, s.EndRoundaboutRadius, s.Roundabout,
                ExplicitCutSides = (int)s.ExplicitCutSides, ExplicitWallSides = (int)s.ExplicitWallSides, RetainingWallSides = (int)s.RetainingWallSides,
                SourceTags = s.SourceTags.Where(p => EvidenceTag(p.Key)).ToDictionary(p => p.Key, p => p.Value),
                RawSourceTags = s.RawSourceTags.Where(p => EvidenceTag(p.Key)).ToDictionary(p => p.Key, p => p.Value),
                SourceGeometry = s.SourceGeometry.Take(512).Select(Point).ToArray(), OriginalGeometry = s.OriginalGeometry.Take(512).Select(Point).ToArray(),
                SourcePointCount = s.SourceGeometry.Count, OriginalPointCount = s.OriginalGeometry.Count,
                SourcePointsTruncated = s.SourceGeometry.Count > 512 || s.OriginalGeometry.Count > 512,
                HorizontalParentOmitted = s.HorizontalParent != null, s.ParentStartT, s.ParentEndT
            };
        }
    }

    public static RunArchive.ArtifactResult CaptureGeometry(RunArchive archive, string stage, IReadOnlyList<PreparedSegment> sections, ushort[] heights, float scale, double width)
    {
        var result = archive.WriteRecords(stage + "-network.ndjson", NetworkRecords(sections));
        archive.WriteHeights(stage + "-heightmap.u16", heights);
        archive.WriteJson(stage + "-geometry.json", new
        {
            schemaVersion = 1, stage, sectionsRequested = sections.Count, network = result,
            terrainFile = stage + "-heightmap.u16", heightSamples = heights.Length, heightResolution = (int)Math.Sqrt(heights.Length),
            heightScaleMeters = scale, mapWidthMeters = width, encoding = "little-endian uint16, row-major z/x; height=value/65535*heightScaleMeters",
            coordinateFrame = "local metres, origin at imported map centre; request latitude/longitude in run.json",
            sourceGeometryPointLimitPerSection = 512, horizontalParentGraphsIncluded = false,
            evidence = "Prepared geometry/terrain, not live native rendering. Check truncation flags before replay."
        });
        return result;
    }

    public static void CapturePrepared(RunArchive archive, PreparedImport prepared)
    {
        CaptureGeometry(archive, "native-plan", prepared.Segments, prepared.Heightmap, prepared.HeightScaleMeters, prepared.Request.MapSize.WidthMeters);
        // Full geometry can hit its byte budget on dense cities. Preserve a much
        // smaller independent incidence graph so truncated shape evidence cannot
        // make a real junction appear to be degree two during later diagnosis.
        archive.WriteRecords("native-plan-topology.ndjson",prepared.Segments.Select((s,i)=>(object)new
        {Index=i,s.WayId,s.StartNodeId,s.EndNodeId,Kind=s.Kind.ToString(),s.Classification,s.Bridge,s.Tunnel,s.OneWay,s.Lanes,s.Layer}));
        archive.WriteRecords("water-plan.ndjson", (prepared.Water?.Sources ?? Array.Empty<RiverSourcePlan>()).Select(s => (object)new
        {
            s.BodyId, s.BodyKind, s.WayId, s.Name, s.Origin, Position = Point(s.Position), s.Radius, s.Depth, s.BankLevel,
            s.TargetSurfaceLevel, s.BoundaryDriven, ChannelProbes = s.ChannelProbes.Select(Point).ToArray(), BankProbes = s.BankProbes.Select(Point).ToArray(),
            DownstreamProbe = Point(s.DownstreamProbe), bedRepairCellCount = s.BedRepairCells.Count
        }), maximumBytes: 8L * 1024 * 1024);
        archive.WriteRecords("vegetation-plan.ndjson", prepared.Vegetation.Select((v, i) => (object)new
        { index = i, position = Point(v.Position), form = v.Form.ToString(), v.Seed, v.SourceId, v.Evidence, v.Taxon }), maximumBytes: 8L * 1024 * 1024);
        archive.WriteJson("environment-plan.json", new
        {
            schemaVersion = 1, prepared.Request, prepared.Climate, prepared.SeaLevelMeters,
            vegetationCandidates = prepared.Vegetation.Count,
            vegetationDecisions = prepared.Vegetation.GroupBy(v => (v.Form, v.Evidence)).Select(g => new { form = g.Key.Form.ToString(), evidence = g.Key.Evidence, count = g.Count() }).ToArray(),
            waterControls = prepared.Water?.Sources.Count ?? 0, waterCellSize = prepared.Water?.WaterCellSize,
            surfaceRequested = prepared.Request.ApplyGroundSurfaces, surfaceAvailable = prepared.SurfaceMask != null,
            surfaceResolution = prepared.SurfaceMask?.Resolution, surfaceReport = prepared.SurfaceMask?.Report,
            evidence = "Planned environment, not observed asset placement or water simulation."
        });
        if (prepared.SurfaceMask != null)
        {
            archive.WriteBytes("surface-material.u8", prepared.SurfaceMask.Material);
            archive.WriteBytes("surface-intensity.u8", prepared.SurfaceMask.Intensity);
            archive.WriteBytes("surface-confidence.u8", prepared.SurfaceMask.Confidence);
        }
    }
}
