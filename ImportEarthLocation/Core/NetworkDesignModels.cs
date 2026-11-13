using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using Unity.Mathematics;

namespace ImportEarthLocation.Core;

public readonly record struct SourceNodeKey(NetworkKind Kind, long Id)
{
    public static SourceNodeKey Start(PreparedSegment segment) => new(segment.Kind, segment.StartNodeId);

    public static SourceNodeKey End(PreparedSegment segment) => new(segment.Kind, segment.EndNodeId);
}

public readonly record struct SourceSpanKey(long WayId, int RawIndex, bool Reversed);

public readonly record struct SourcePoint(SourceNodeKey Node, double3 Position);

public readonly record struct Cubic3d(double3 P0, double3 P1, double3 P2, double3 P3);

public readonly record struct Cubic3f(float3 P0, float3 P1, float3 P2, float3 P3);

public readonly record struct SourceNodeAlias(long SourceNodeId, long CanonicalNodeId, bool Mandatory);

public sealed record SourceConnection(SourceNodeKey From, SourceNodeKey To, IReadOnlyList<long> Ways);

public sealed record SourceTopologyManifest(
    IReadOnlyList<SourceNodeKey> Ports,
    IReadOnlyList<SourceConnection> Connections);

public sealed class SourceChain
{
    private readonly SourcePoint[] _points;
    private readonly SourceSpanKey[] _spans;

    public long Id { get; }

    public SourceNodeKey Start => _points[0].Node;

    public SourceNodeKey End => _points[_points.Length - 1].Node;

    public IReadOnlyList<SourcePoint> OrderedPoints { get; }
    public IReadOnlyList<SourceSpanKey> OrderedSpans { get; }
    public IReadOnlyDictionary<string, string> Tags { get; }

    public SourceChain(long id, SourcePoint[] points, SourceSpanKey[] spans, IReadOnlyDictionary<string, string> tags)
    {
        if (points.Length < 2 || spans.Length != points.Length - 1)
            throw new ArgumentException("A source chain must retain ordered span lineage.");

        Id = id;
        _points = (SourcePoint[])points.Clone();
        _spans = (SourceSpanKey[])spans.Clone();
        OrderedPoints = Array.AsReadOnly(_points);
        OrderedSpans = Array.AsReadOnly(_spans);
        var copy = new Dictionary<string, string>(StringComparer.Ordinal);

        foreach (var pair in tags)
            copy.Add(pair.Key, pair.Value);

        Tags = new System.Collections.ObjectModel.ReadOnlyDictionary<string, string>(copy);
    }
}

public sealed class SourceManifest
{
    public IReadOnlyList<SourceChain> Chains { get; }
    public SourceTopologyManifest? Topology { get; }

    public SourceManifest(SourceChain[] chains, SourceTopologyManifest? topology = null)
    {
        Chains = Array.AsReadOnly((SourceChain[])chains.Clone());
        Topology =
            topology == null
                ? null
                : new(Array.AsReadOnly(topology.Ports.ToArray()), Array.AsReadOnly(topology.Connections
            .Select(connection => connection with { Ways = Array.AsReadOnly(connection.Ways.ToArray()) }).ToArray()));
    }
}

public enum DesignQuality
{
    Certified,
    RequiresReview
}

public readonly record struct CurveCertificate(
    DesignQuality Quality,
    double MaximumCurvature,
    double MaximumGrade,
    int Leaves,
    bool NonFinite);

// Owns the preparation result. Submission order and native observations are
// separate products; neither may mutate these anchors or exact float curves.
public sealed class FinalNetworkDesign
{
    public IReadOnlyList<PreparedSegment> Sections { get; }
    public IReadOnlyList<NativeAssetContract> Assets { get; }
    public SourceManifest? Source { get; }
    public string Fingerprint { get; }

    public DesignQuality Quality => DesignQuality.RequiresReview;

    public FinalNetworkDesign(
        IReadOnlyList<PreparedSegment> sections,
        IReadOnlyList<NativeAssetContract> assets,
        SourceManifest? source = null)
    {
        Source = source;
        Sections = Array.AsReadOnly(sections.Select(segment => segment with
        {
            FinalDeckCurve =
                new(segment.CourseStart, segment.CourseControl1, segment.CourseControl2, segment.CourseEnd),
            HorizontalParent = null,
            NativeLaneOffsets = Array.AsReadOnly(segment.NativeLaneOffsets.ToArray()),
            SourceGeometry = Array.AsReadOnly(segment.SourceGeometry.ToArray()),
            OriginalGeometry = Array.AsReadOnly(segment.OriginalGeometry.ToArray()),
            SourceWayIds = Array.AsReadOnly(segment.SourceWayIds.ToArray()),
            SourceNodeAliases = Array.AsReadOnly(segment.SourceNodeAliases.ToArray()),
            SourceTags = Copy(segment.SourceTags),
            RawSourceTags = Copy(segment.RawSourceTags)
        }).ToArray());

        Assets = Array.AsReadOnly(assets.ToArray());
        using var bytes = new MemoryStream();
        using var writer = new BinaryWriter(bytes);
        writer.Write(Sections.Count);

        void Point(float3 point)
        {
            writer.Write(point.x);
            writer.Write(point.y);
            writer.Write(point.z);
        }

        foreach (var segment in Sections)
        {
            writer.Write((int)segment.Kind);
            writer.Write(segment.WayId);
            writer.Write(segment.StartNodeId);
            writer.Write(segment.EndNodeId);
            writer.Write(segment.SourceWayIds.Count);

            foreach (var id in segment.SourceWayIds)
                writer.Write(id);

            writer.Write(segment.SourceNodeAliases.Count);

            foreach (var alias in segment.SourceNodeAliases)
            {
                writer.Write(alias.SourceNodeId);
                writer.Write(alias.CanonicalNodeId);
                writer.Write(alias.Mandatory);
            }

            Point(segment.Start);
            Point(segment.End);
            Point(segment.CourseStart);
            Point(segment.CourseControl1);
            Point(segment.CourseControl2);
            Point(segment.CourseEnd);
            writer.Write(segment.NativeAssetKey);
            writer.Write(segment.NativeUnitId);
            writer.Write(segment.NativeWidth);
            writer.Write(segment.NativeCenterlineCurvatureLimit ?? double.NaN);
            writer.Write(segment.Bridge);
            writer.Write(segment.Tunnel);
            writer.Write(segment.Layer);
            writer.Write(segment.PairedCorridorId);
            writer.Write(segment.PairedSpanId);
            writer.Write(segment.StartHeightFamily);
            writer.Write(segment.EndHeightFamily);
            writer.Write(segment.PairedReversed);
            writer.Write(segment.ReferenceSpan);
            writer.Write(segment.StartReferenceSpeed);
            writer.Write(segment.EndReferenceSpeed);
            writer.Write(segment.PairedProfileId);
            writer.Write(segment.PairedReferenceBegin);
            writer.Write(segment.PairedReferenceEnd);
            writer.Write(segment.PairedProfileOnly);
            writer.Write(segment.RailTerminalFormationId);
            writer.Write(segment.NativeLaneOffsets.Count);

            foreach (var lane in segment.NativeLaneOffsets)
                writer.Write(lane);
        }

        writer.Write(Assets.Count);

        foreach (var asset in Assets.OrderBy(contract => contract.PaletteKey, StringComparer.Ordinal))
        {
            writer.Write(asset.PaletteKey);
            writer.Write(asset.TrainCurvatureLimit ?? float.NaN);
            writer.Write(asset.Envelopes.Count);

            foreach (var envelope in asset.Envelopes)
            {
                writer.Write((byte)envelope.Mode);
                writer.Write(envelope.SideMask);
                writer.Write(envelope.Available);
                writer.Write(envelope.Width);
                writer.Write(envelope.RangeMin);
                writer.Write(envelope.RangeMax);
                writer.Write(envelope.SurfaceMin);
                writer.Write(envelope.SurfaceMax);
                writer.Write(envelope.NodeOffset);
            }
        }

        writer.Flush();
        using var hash = SHA256.Create();
        Fingerprint = BitConverter.ToString(hash.ComputeHash(bytes.ToArray())).Replace("-", "");
    }

    private static IReadOnlyDictionary<string, string> Copy(IReadOnlyDictionary<string, string> source)
    {
        var copy = new Dictionary<string, string>(StringComparer.Ordinal);

        foreach (var pair in source)
            copy.Add(pair.Key, pair.Value);

        return new System.Collections.ObjectModel.ReadOnlyDictionary<string, string>(copy);
    }
}
