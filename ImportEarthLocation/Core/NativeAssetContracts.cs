using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using System.Security.Cryptography;

namespace ImportEarthLocation.Core;

public static class NativeAssetKeys
{
    public static bool IsHighway(PreparedSegment s)=>s.Classification.StartsWith("motorway",StringComparison.OrdinalIgnoreCase)||s.Classification.StartsWith("trunk",StringComparison.OrdinalIgnoreCase);
    public static string Family(PreparedSegment s)=>$"{(IsHighway(s)?"highway":"road")}:{s.OneWay}:{s.Bridge}:{s.ConsolidatedAvenue}";
    public static string Sized(PreparedSegment s)=>s.Kind==NetworkKind.Rail?$"rail:{s.Bridge}:{Numeric.Clamp(s.PhysicalTracks>0?s.PhysicalTracks:s.Lanes,1,2)}":
        Family(s)+":"+Numeric.Clamp(s.Lanes,1,IsHighway(s)?(s.OneWay?5:8):(s.OneWay?3:6));
}

public enum NativeStructureMode : byte { Ground, Elevated, Tunnel }
// Only scalars cross the worker boundary. No Entity, native buffers or lookups.
public readonly record struct NativeCompositionEnvelope(NativeStructureMode Mode,byte SideMask,
    float Width,float SurfaceMin,float SurfaceMax,float RangeMin,float RangeMax,float NodeOffset,bool Available);
public sealed class NativeAssetContract
{
    // Effective immutable design data only. Enumeration order and live ECS
    // handles cannot invalidate an already completed design in the same epoch.
    public static string Fingerprint(IEnumerable<NativeAssetContract> assets)
    {
        using var bytes=new MemoryStream();using var writer=new BinaryWriter(bytes);
        foreach(var asset in assets.OrderBy(a=>a.PaletteKey,StringComparer.Ordinal))
        {
            writer.Write(asset.PaletteKey);writer.Write(asset.TrainCurvatureLimit??float.NaN);
            var lanes=asset.LaneOffsets.OrderBy(x=>x).ToArray();writer.Write(lanes.Length);
            foreach(var lane in lanes)writer.Write(lane);
            var envelopes=asset.Envelopes.OrderBy(e=>e.Mode).ThenBy(e=>e.SideMask).ThenBy(e=>e.Available).ThenBy(e=>e.Width)
                .ThenBy(e=>e.SurfaceMin).ThenBy(e=>e.SurfaceMax).ThenBy(e=>e.RangeMin).ThenBy(e=>e.RangeMax).ThenBy(e=>e.NodeOffset).ToArray();writer.Write(envelopes.Length);
            foreach(var e in envelopes)
            {writer.Write((byte)e.Mode);writer.Write(e.SideMask);writer.Write(e.Available);writer.Write(e.Width);writer.Write(e.SurfaceMin);writer.Write(e.SurfaceMax);writer.Write(e.RangeMin);writer.Write(e.RangeMax);writer.Write(e.NodeOffset);}
        }
        writer.Flush();using var hash=SHA256.Create();return BitConverter.ToString(hash.ComputeHash(bytes.ToArray())).Replace("-","");
    }
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
