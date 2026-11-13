using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Unity.Mathematics;

namespace ImportEarthLocation.Core;

// Explicit DTOs avoid serializing Unity swizzles or recursive HorizontalParent
// graphs. Local evidence only; no endpoint credentials, uploads or game mutation.
public static class DiagnosticArtifacts
{
    public static IEnumerable<object> ExactNetworkRecords(IReadOnlyList<PreparedSegment> sections)
    {
        foreach (var segment in sections)
            // FinalDeckCurve also indicates frozen geometry to preprocessing.
            // Preserve its presence/absence: manufacturing it changes replay
            // behavior even when the four evaluated controls are identical.
            yield return segment with { HorizontalParent = null };
    }

    public static string SectionIdentity(PreparedSegment segment)
        => FormattableString.Invariant($"{segment.Kind}:{segment.WayId}:{segment.StartNodeId}:{segment.EndNodeId}");

    public static IEnumerable<object> SourceRecords(SourceManifest source)
    {
        foreach (var chain in source.Chains)
        {
            yield return new
            {
                record = "chain",
                chainId = chain.Id,
                points = chain.OrderedPoints.Count,
                tags = chain.Tags
                    .Where(entry => EvidenceTag(entry.Key))
                    .ToDictionary(entry => entry.Key, entry => entry.Value)
            };

            for (var i = 0; i < chain.OrderedPoints.Count; i++)
            {
                var point = chain.OrderedPoints[i];
                yield return new
                {
                    record = "point",
                    chainId = chain.Id,
                    ordinal = i,
                    kind = point.Node.Kind.ToString(),
                    nodeId = point.Node.Id,
                    x = point.Position.x,
                    y = point.Position.y,
                    z = point.Position.z,
                    incomingSpan = i > 0 ? (SourceSpanKey?)chain.OrderedSpans[i - 1] : null
                };
            }
        }
    }

    public static object Point(float3 point) => new { x = point.x, y = point.y, z = point.z };

    public static object Point(double3 point) => new { x = point.x, y = point.y, z = point.z };

    public static object? Curve(Cubic3d? curve) => curve.HasValue ? new
    { a = Point(curve.Value.P0), b = Point(curve.Value.P1), c = Point(curve.Value.P2), d = Point(curve.Value.P3) }
        : null;

    private static object? Point(float3? point) => point.HasValue ? Point(point.Value) : null;

    private static bool EvidenceTag(string key)
        => key == "name" || key == "ref" || key == "highway" || key == "railway"
            || key == "oneway"
            || key == "junction"
            || key == "layer"
            || key == "bridge"
            || key == "tunnel"
            || key == "covered"
            || key == "location"
            || key == "service"
            || key == "tracks"
            || key == "gauge"
            || key == "width"
            || key == "incline"
            || key == "embankment"
            || key == "cutting"
            || key == "lanes"
            || key.StartsWith("lanes:", StringComparison.Ordinal)
            || key.StartsWith("turn:lanes", StringComparison.Ordinal)
                || key.StartsWith("maxspeed", StringComparison.Ordinal);

    public static IEnumerable<object> NetworkRecords(IReadOnlyList<PreparedSegment> sections)
    {
        for (var index = 0; index < sections.Count; index++)
        {
            var segment = sections[index];
            yield return new
            {
                Index = index,
                segment.WayId,
                segment.StartNodeId,
                segment.EndNodeId,
                Start = Point(segment.Start),
                End = Point(segment.End),
                Control1 = Point(segment.Control1),
                Control2 = Point(segment.Control2),
                Kind = segment.Kind.ToString(),
                segment.Classification,
                DeckStart = Point(segment.CourseStart),
                DeckControl1 = Point(segment.CourseControl1),
                DeckControl2 = Point(segment.CourseControl2),
                DeckEnd = Point(segment.CourseEnd),
                ExactFinalDeckStored = segment.FinalDeckCurve.HasValue,
                HorizontalEmissionReference = Curve(segment.HorizontalEmissionReference),
                GeometryEvidence = "prepared-course-not-native-readback",
                segment.Lanes,
                segment.OneWay,
                segment.Bridge,
                segment.Tunnel,
                segment.Layer,
                segment.Name,
                segment.Reference,
                segment.RailService,
                segment.SourceWayIds,
                segment.SourceNodeAliases,
                segment.StartTerminal,
                segment.EndTerminal,
                segment.PhysicalTracks,
                segment.BothWaysLanes,
                segment.ForwardLanes,
                segment.BackwardLanes,
                segment.ExplicitLanes,
                segment.SourceWidthMeters,
                segment.SourceSpeedKph,
                segment.ExplicitIncline,
                segment.SignedInclinePriorAccepted,
                segment.CorridorId,
                segment.ConsolidatedAvenue,
                segment.NativeAssetKey,
                segment.NativeUnitId,
                segment.NativeCenterlineCurvatureLimit,
                segment.NativeWidth,
                segment.MaximumNativeWidth,
                segment.NativeLaneOffsets,
                segment.StartElevation,
                segment.EndElevation,
                segment.PairedCorridorId,
                segment.PairedSpanId,
                segment.StartHeightFamily,
                segment.EndHeightFamily,
                segment.PairedReversed,
                segment.ReferenceSpan,
                segment.StartReferenceSpeed,
                segment.EndReferenceSpeed,
                segment.PairedProfileId,
                segment.PairedProfileOnly,
                segment.PairedReferenceBegin,
                segment.PairedReferenceEnd,
                segment.RailTerminalFormationId,
                StartCurveOffset = Point(segment.StartCurveOffset),
                EndCurveOffset = Point(segment.EndCurveOffset),
                segment.StartMiniRoundabout,
                segment.EndMiniRoundabout,
                segment.StartRoundaboutRadius,
                segment.EndRoundaboutRadius,
                segment.Roundabout,
                ExplicitCutSides = (int)segment.ExplicitCutSides,
                ExplicitWallSides = (int)segment.ExplicitWallSides,
                RetainingWallSides = (int)segment.RetainingWallSides,
                SourceTags = segment.SourceTags
                    .Where(entry => EvidenceTag(entry.Key))
                    .ToDictionary(entry => entry.Key, entry => entry.Value),
                RawSourceTags = segment.RawSourceTags
                    .Where(entry => EvidenceTag(entry.Key))
                    .ToDictionary(entry => entry.Key, entry => entry.Value),
                SourceGeometry = segment.SourceGeometry.Take(512).Select(Point).ToArray(),
                OriginalGeometry = segment.OriginalGeometry.Take(512).Select(Point).ToArray(),
                SourcePointCount = segment.SourceGeometry.Count,
                OriginalPointCount = segment.OriginalGeometry.Count,
                SourcePointsTruncated = segment.SourceGeometry.Count > 512 || segment.OriginalGeometry.Count > 512,
                HorizontalParentOmitted = segment.HorizontalParent != null,
                segment.ParentStartT,
                segment.ParentEndT
            };
        }
    }

    public static RunArchive.ArtifactResult CaptureGeometry(
        RunArchive archive,
        string stage,
        IReadOnlyList<PreparedSegment> sections,
        float[] heights,
        float scale,
        double width)
    {
        var result = archive.WriteRecords(stage + "-network.ndjson", NetworkRecords(sections));
        archive.WriteHeights(stage + "-heightmap.u16", heights, scale);
        archive.WriteJson(stage + "-geometry.json", new
        {
            schemaVersion = 1,
            stage,
            sectionsRequested = sections.Count,
            network = result,
            terrainFile = stage + "-heightmap.u16",
            heightSamples = heights.Length,
            heightResolution = (int)Math.Sqrt(heights.Length),
            heightScaleMeters = scale,
            mapWidthMeters = width,
            encoding = "little-endian uint16, row-major z/x; height=value/65535*heightScaleMeters",
            coordinateFrame = "local metres, origin at imported map centre; request latitude/longitude in run.json",
            sourceGeometryPointLimitPerSection = 512,
            horizontalParentGraphsIncluded = false,
            evidence = "Prepared geometry/terrain, not live native rendering. Check truncation flags before replay."
        });

        return result;
    }

    public static void CapturePrepared(RunArchive archive, PreparedImport prepared)
    {
        CaptureGeometry(
            archive,
            "native-plan",
            prepared.Segments,
            prepared.Heights,
            prepared.HeightScaleMeters,
            prepared.Request.MapSize.WidthMeters);

        // Full geometry can hit its byte budget on dense cities. Preserve a much
        // smaller independent incidence graph so truncated shape evidence cannot
        // make a real junction appear to be degree two during later diagnosis.
        archive.WriteRecords("native-plan-topology.ndjson", prepared.Segments.Select((segment, i) => (object)new
        {
            Index = i,
            segment.WayId,
            segment.StartNodeId,
            segment.EndNodeId,
            Kind = segment.Kind.ToString(),
            segment.Classification,
            segment.Bridge,
            segment.Tunnel,
            segment.OneWay,
            segment.Lanes,
            segment.Layer
        }));

        archive.WriteRecords(
            "water-plan.ndjson",
            (prepared.Water?.Sources ?? Array.Empty<RiverSourcePlan>()).Select(river => (object)new
            {
                river.BodyId,
                river.BodyKind,
                river.WayId,
                river.Name,
                river.Origin,
                Position = Point(river.Position),
                river.Radius,
                river.Depth,
                river.BankLevel,
                river.TargetSurfaceLevel,
                river.BoundaryDriven,
                ChannelProbes = river.ChannelProbes.Select(Point).ToArray(),
                BankProbes = river.BankProbes.Select(Point).ToArray(),
                DownstreamProbe = Point(river.DownstreamProbe),
                bedRepairCellCount = river.BedRepairCells.Count
            }), maximumBytes: 8L * 1024 * 1024);

        archive.WriteRecords("vegetation-plan.ndjson", prepared.Vegetation.Select((candidate, i) => (object)new
        {
            index = i,
            position = Point(candidate.Position),
            form = candidate.Form.ToString(),
            candidate.Seed,
            sourceId = candidate.SourceId.ToString(System.Globalization.CultureInfo.InvariantCulture),
            candidate.MetadataIndex,
            metadata = candidate.MetadataIndex >= 0 && candidate.MetadataIndex < prepared.VegetationMetadata.Count
                ? prepared.VegetationMetadata[candidate.MetadataIndex]
                : default
        }), maximumBytes: 8L * 1024 * 1024);

        archive.WriteJson("environment-plan.json", new
        {
            schemaVersion = 1,
            prepared.Request,
            prepared.Climate,
            prepared.SeaLevelMeters,
            vegetationCandidates = prepared.Vegetation.Count,
            vegetationMetadataEntries = prepared.VegetationMetadata.Count,
            waterControls = prepared.Water?.Sources.Count ?? 0,
            waterCellSize = prepared.Water?.WaterCellSize,
            surfaceRequested = prepared.Request.ApplyGroundSurfaces,
            surfaceAvailable = prepared.SurfaceMask != null,
            surfaceResolution = prepared.SurfaceMask?.Resolution,
            surfaceReport = prepared.SurfaceMask?.Report,
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
