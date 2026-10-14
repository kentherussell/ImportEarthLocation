using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Unity.Mathematics;

namespace ImportEarthLocation.Core;

[StructLayout(LayoutKind.Sequential)]
public readonly record struct VegetationCandidate(float3 Position,VegetationForm Form,long SourceId,uint Seed,int MetadataIndex);
public readonly record struct VegetationMetadata(string Evidence,string Taxon);

public sealed class VegetationPlan : IReadOnlyList<VegetationCandidate>
{
    public static readonly VegetationPlan Empty=new(Array.Empty<VegetationCandidate>(),Array.Empty<VegetationMetadata>());
    private readonly VegetationCandidate[] _candidates;
    public IReadOnlyList<VegetationMetadata> Metadata { get; }
    public int Count=>_candidates.Length;
    public VegetationCandidate this[int index]=>_candidates[index];
    // Ownership of these freshly built arrays transfers to the immutable plan.
    public VegetationPlan(VegetationCandidate[] candidates,VegetationMetadata[] metadata)
    { _candidates=candidates;Metadata=Array.AsReadOnly(metadata); }
    public IEnumerator<VegetationCandidate> GetEnumerator()=>((IEnumerable<VegetationCandidate>)_candidates).GetEnumerator();
    IEnumerator IEnumerable.GetEnumerator()=>GetEnumerator();
}
