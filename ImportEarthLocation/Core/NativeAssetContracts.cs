using System;
using System.Collections.Generic;
using System.Linq;

namespace ImportEarthLocation.Core;

public enum NativeStructureMode : byte { Ground, Elevated, Tunnel }
// Only scalars cross the worker boundary. No Entity, native buffers or lookups.
public readonly record struct NativeCompositionEnvelope(NativeStructureMode Mode,byte SideMask,
    float Width,float SurfaceMin,float SurfaceMax,float RangeMin,float RangeMax,float NodeOffset,bool Available);
public sealed class NativeAssetContract
{
    private readonly NativeCompositionEnvelope?[] _modeEnvelopes=new NativeCompositionEnvelope?[3];
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
        foreach(var mode in new[]{NativeStructureMode.Ground,NativeStructureMode.Elevated,NativeStructureMode.Tunnel})
        {
            var available=Envelopes.Where(e=>e.Mode==mode&&e.Available).ToArray();if(available.Length==0)continue;
            _modeEnvelopes[(int)mode]=new(mode,0,available.Max(e=>e.Width),available.Min(e=>e.SurfaceMin),available.Max(e=>e.SurfaceMax),
                available.Min(e=>e.RangeMin),available.Max(e=>e.RangeMax),available.Max(e=>e.NodeOffset),true);
        }
    }

    // Earthwork sides are selected after the profile. Design against the union
    // of available side compositions in this mode, not the narrower default.
    public bool TryEnvelope(NativeStructureMode mode,out NativeCompositionEnvelope envelope)
    {
        var index=(int)mode;
        var cached=index>=0&&index<_modeEnvelopes.Length?_modeEnvelopes[index]:null;
        envelope=cached??default;return cached.HasValue;
    }

    public static NativeStructureMode Mode(PreparedSegment segment)=>segment.Tunnel?NativeStructureMode.Tunnel:
        segment.Bridge?NativeStructureMode.Elevated:NativeStructureMode.Ground;

    public double? CenterlineCurvatureLimit(double width)
    {
        if(!TrainCurvatureLimit.HasValue)return null;
        var k=(double)TrainCurvatureLimit.Value;
        if(double.IsNaN(k)||double.IsInfinity(k)||k<=0)return null;
        var distance=LaneOffsets.Count>0?LaneOffsets.Max(v=>Math.Abs((double)v)):width/2;
        return k/(1+k*distance);
    }
}
