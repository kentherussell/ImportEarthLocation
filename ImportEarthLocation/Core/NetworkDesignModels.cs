using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using Unity.Mathematics;

namespace ImportEarthLocation.Core;

public readonly record struct SourceNodeKey(NetworkKind Kind, long Id);
public readonly record struct StationKey(long ChainId, int Ordinal);
public readonly record struct SourceSpanKey(long WayId, int RawIndex, bool Reversed);
public readonly record struct SourcePoint(SourceNodeKey Node, double3 Position);
public readonly record struct Cubic3d(double3 P0, double3 P1, double3 P2, double3 P3);
public readonly record struct Cubic3f(float3 P0, float3 P1, float3 P2, float3 P3);
public readonly record struct SourceNodeAlias(long SourceNodeId,long CanonicalNodeId,bool Mandatory);
public sealed record SourceConnection(SourceNodeKey From,SourceNodeKey To,IReadOnlyList<long> Ways);
public sealed record SourceTopologyManifest(IReadOnlyList<SourceNodeKey> Ports,IReadOnlyList<SourceConnection> Connections);

public sealed class SourceChain
{
    private readonly SourcePoint[] _points;
    private readonly SourceSpanKey[] _spans;
    public long Id { get; }
    public SourceNodeKey Start => _points[0].Node;
    public SourceNodeKey End => _points[_points.Length-1].Node;
    public IReadOnlyList<SourcePoint> OrderedPoints { get; }
    public IReadOnlyList<SourceSpanKey> OrderedSpans { get; }
    public IReadOnlyDictionary<string,string> Tags { get; }
    public SourceChain(long id, SourcePoint[] points, SourceSpanKey[] spans, IReadOnlyDictionary<string,string> tags)
    {
        if(points.Length<2||spans.Length!=points.Length-1)throw new ArgumentException("A source chain must retain ordered span lineage.");
        Id=id;_points=(SourcePoint[])points.Clone();_spans=(SourceSpanKey[])spans.Clone();
        OrderedPoints=Array.AsReadOnly(_points);OrderedSpans=Array.AsReadOnly(_spans);
        var copy=new Dictionary<string,string>(StringComparer.Ordinal);foreach(var pair in tags)copy.Add(pair.Key,pair.Value);
        Tags=new System.Collections.ObjectModel.ReadOnlyDictionary<string,string>(copy);
    }
}

public sealed class SourceManifest
{
    public IReadOnlyList<SourceChain> Chains { get; }
    public SourceTopologyManifest? Topology { get; }
    public SourceManifest(SourceChain[] chains,SourceTopologyManifest? topology=null)
    {
        Chains=Array.AsReadOnly((SourceChain[])chains.Clone());
        Topology=topology==null?null:new(Array.AsReadOnly(topology.Ports.ToArray()),Array.AsReadOnly(topology.Connections
            .Select(c=>c with{Ways=Array.AsReadOnly(c.Ways.ToArray())}).ToArray()));
    }
}

public enum DesignQuality { Certified, RequiresReview }
public readonly record struct CurveCertificate(DesignQuality Quality, double MaximumCurvature, double MaximumGrade, int Leaves, bool NonFinite);

// Owns the preparation result. Submission order and native observations are
// separate products; neither may mutate these anchors or exact float curves.
public sealed class FinalNetworkDesign
{
    public IReadOnlyList<PreparedSegment> Sections { get; }
    public IReadOnlyList<NativeAssetContract> Assets { get; }
    public SourceManifest? Source { get; }
    public string Fingerprint { get; }
    public DesignQuality Quality => DesignQuality.RequiresReview;
    public FinalNetworkDesign(IReadOnlyList<PreparedSegment> sections,IReadOnlyList<NativeAssetContract> assets,SourceManifest? source=null)
    {
        Source=source;
        Sections=Array.AsReadOnly(sections.Select(s=>s with{
            FinalDeckCurve=new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd),HorizontalParent=null,
            NativeLaneOffsets=Array.AsReadOnly(s.NativeLaneOffsets.ToArray()),SourceGeometry=Array.AsReadOnly(s.SourceGeometry.ToArray()),
            OriginalGeometry=Array.AsReadOnly(s.OriginalGeometry.ToArray()),SourceWayIds=Array.AsReadOnly(s.SourceWayIds.ToArray()),
            SourceNodeAliases=Array.AsReadOnly(s.SourceNodeAliases.ToArray()),
            SourceTags=Copy(s.SourceTags),RawSourceTags=Copy(s.RawSourceTags)}).ToArray());
        Assets=Array.AsReadOnly(assets.ToArray());
        using var bytes=new MemoryStream();using var writer=new BinaryWriter(bytes);
        writer.Write(Sections.Count);
        void Point(float3 p){writer.Write(p.x);writer.Write(p.y);writer.Write(p.z);}
        foreach(var s in Sections)
        {
            writer.Write((int)s.Kind);writer.Write(s.WayId);writer.Write(s.StartNodeId);writer.Write(s.EndNodeId);
            writer.Write(s.SourceWayIds.Count);foreach(var id in s.SourceWayIds)writer.Write(id);
            writer.Write(s.SourceNodeAliases.Count);foreach(var alias in s.SourceNodeAliases){writer.Write(alias.SourceNodeId);writer.Write(alias.CanonicalNodeId);writer.Write(alias.Mandatory);}
            Point(s.Start);Point(s.End);Point(s.CourseStart);Point(s.CourseControl1);Point(s.CourseControl2);Point(s.CourseEnd);
            writer.Write(s.NativeAssetKey);writer.Write(s.NativeUnitId);writer.Write(s.NativeWidth);writer.Write(s.NativeCenterlineCurvatureLimit??double.NaN);writer.Write(s.Bridge);writer.Write(s.Tunnel);writer.Write(s.Layer);
            writer.Write(s.PairedCorridorId);writer.Write(s.PairedSpanId);writer.Write(s.StartHeightFamily);writer.Write(s.EndHeightFamily);writer.Write(s.PairedReversed);writer.Write(s.ReferenceSpan);writer.Write(s.StartReferenceSpeed);writer.Write(s.EndReferenceSpeed);
            writer.Write(s.PairedProfileId);writer.Write(s.PairedReferenceBegin);writer.Write(s.PairedReferenceEnd);
            writer.Write(s.PairedProfileOnly);writer.Write(s.RailTerminalFormationId);
            writer.Write(s.NativeLaneOffsets.Count);foreach(var lane in s.NativeLaneOffsets)writer.Write(lane);
        }
        writer.Write(Assets.Count);
        foreach(var asset in Assets.OrderBy(a=>a.PaletteKey,StringComparer.Ordinal))
        {
            writer.Write(asset.PaletteKey);writer.Write(asset.TrainCurvatureLimit??float.NaN);writer.Write(asset.Envelopes.Count);
            foreach(var e in asset.Envelopes){writer.Write((byte)e.Mode);writer.Write(e.SideMask);writer.Write(e.Available);writer.Write(e.Width);
                writer.Write(e.RangeMin);writer.Write(e.RangeMax);writer.Write(e.SurfaceMin);writer.Write(e.SurfaceMax);writer.Write(e.NodeOffset);}
        }
        writer.Flush();using var hash=SHA256.Create();Fingerprint=BitConverter.ToString(hash.ComputeHash(bytes.ToArray())).Replace("-","");
    }
    private static IReadOnlyDictionary<string,string> Copy(IReadOnlyDictionary<string,string> source)
    {var copy=new Dictionary<string,string>(StringComparer.Ordinal);foreach(var pair in source)copy.Add(pair.Key,pair.Value);return new System.Collections.ObjectModel.ReadOnlyDictionary<string,string>(copy);}
}
