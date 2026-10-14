using System;
using System.Collections.Generic;

namespace ImportEarthLocation.Core;

public enum NativeStructureMode : byte { Ground, Elevated, Tunnel }
// Only scalars cross the worker boundary. No Entity, native buffers or lookups.
public readonly record struct NativeCompositionEnvelope(NativeStructureMode Mode,byte SideMask,
    float Width,float SurfaceMin,float SurfaceMax,float RangeMin,float RangeMax,float NodeOffset,bool Available);
public sealed class NativeAssetContract
{
    public string PaletteKey { get; }
    public IReadOnlyList<NativeCompositionEnvelope> Envelopes { get; }
    public IReadOnlyList<float> LaneOffsets { get; }
    public float? TrainCurvatureLimit { get; }
    public NativeAssetContract(string paletteKey,NativeCompositionEnvelope[] envelopes,IReadOnlyList<float> laneOffsets,float? trainCurvatureLimit=null)
    {
        TrainCurvatureLimit=trainCurvatureLimit;
        PaletteKey=paletteKey;Envelopes=Array.AsReadOnly((NativeCompositionEnvelope[])envelopes.Clone());
        var lanes=new float[laneOffsets.Count];for(var i=0;i<lanes.Length;i++)lanes[i]=laneOffsets[i];
        LaneOffsets=Array.AsReadOnly(lanes);
    }
}
