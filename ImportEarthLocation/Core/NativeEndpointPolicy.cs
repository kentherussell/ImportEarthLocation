using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Mathematics;

namespace ImportEarthLocation.Core;

// The native node map compares float3 positions exactly (not within a tolerance).
// Canonicalize only already-shared topology; never snap nearby unrelated roads.
public static class NativeEndpointPolicy
{
    public static string Identity(PreparedSegment s,long node) => $"{s.Kind}:{node}";
    public static IReadOnlyList<PreparedSegment> Canonicalize(IReadOnlyList<PreparedSegment> source,ImportLog log)
    {
        var positions=new Dictionary<string,float3>(StringComparer.Ordinal);
        var changed=0;var conflicts=0;var result=new List<PreparedSegment>(source.Count);
        float3 Point(PreparedSegment s,long node,float3 point)
        {
            var key=Identity(s,node);
            if(!positions.TryGetValue(key,out var canonical)){positions[key]=point;return point;}
            if(math.distance(canonical,point)>.25f)
            {if(conflicts++<25)log.Warn($"Native endpoint conflict {key}: {math.distance(canonical,point):0.###} m; not snapping unrelated/conflicting geometry.");return point;}
            if(!math.all(canonical==point))changed++;
            return canonical;
        }
        foreach(var s in source)
        {
            var a=Point(s,s.StartNodeId,s.Start);var b=Point(s,s.EndNodeId,s.End);
            result.Add(s with{Start=a,End=b,Control1=s.Control1.HasValue?s.Control1+(a-s.Start):null,Control2=s.Control2.HasValue?s.Control2+(b-s.End):null});
        }
        log.Count("Native endpoints canonicalized",changed);log.Count("Native endpoint position conflicts",conflicts);
        log.Diagnostic($"NATIVE ENDPOINTS canonicalized={changed}; conflicts={conflicts}; toleranceMeters=0.25; exactNativeKeys=true; proximityConnectionsCreated=false; sectionsDeleted=0");
        return result;
    }

    public static Dictionary<string,float> Elevations(IReadOnlyList<PreparedSegment> source)
    {
        var values=new Dictionary<string,List<float>>(StringComparer.Ordinal);
        foreach(var s in source)foreach(var start in new[]{true,false})
        {
            var key=Identity(s,start?s.StartNodeId:s.EndNodeId);
            if(!values.TryGetValue(key,out var list))values[key]=list=new List<float>();
            var elevation=NetworkStructure.IsStructure(s)?(start?s.StartElevation:s.EndElevation)??NetworkStructure.Offset(s):0;
            // A rail bridge abutment can be ground-supported, never a negative
            // cutting composition. Tunnel elevations remain genuinely negative.
            if(s.Kind==NetworkKind.Rail&&s.Bridge&&!s.Tunnel)elevation=Math.Max(0,elevation);
            list.Add(elevation);
        }
        // Mixed ground/structure portals use one node composition on both courses;
        // edge structure flags remain per-edge. Native collection otherwise keeps
        // the first endpoint's flags/elevation, making mixed portals order-dependent.
        return values.ToDictionary(p=>p.Key,p=>p.Value.Any(v=>v>0)?p.Value.Max():p.Value.Min(),StringComparer.Ordinal);
    }
}
