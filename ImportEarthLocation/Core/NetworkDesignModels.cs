using System;
using System.Collections.Generic;
using Unity.Mathematics;

namespace ImportEarthLocation.Core;

public readonly record struct SourceNodeKey(NetworkKind Kind, long Id);
public readonly record struct StationKey(long ChainId, int Ordinal);
public readonly record struct SourceSpanKey(long WayId, int RawIndex, bool Reversed);
public readonly record struct SourcePoint(SourceNodeKey Node, double3 Position);
public readonly record struct Cubic3d(double3 P0, double3 P1, double3 P2, double3 P3);
public readonly record struct Cubic3f(float3 P0, float3 P1, float3 P2, float3 P3);

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
    public SourceManifest(SourceChain[] chains) => Chains=Array.AsReadOnly((SourceChain[])chains.Clone());
}

public enum DesignQuality { Certified, RequiresReview }
public readonly record struct CurveCertificate(DesignQuality Quality, double MaximumCurvature, double MaximumGrade, int Leaves, bool NonFinite);
