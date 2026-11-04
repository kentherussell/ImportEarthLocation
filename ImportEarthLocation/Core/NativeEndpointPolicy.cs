using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Mathematics;

namespace ImportEarthLocation.Core;

// The native node map compares float3 positions exactly (not within a tolerance).
// Canonicalize only already-shared topology; never snap nearby unrelated roads.
public static class NativeEndpointPolicy
{
    public static string Identity(PreparedSegment s,long node) =>
        node==s.StartNodeId&&s.StartLevelCrossing||node==s.EndNodeId&&s.EndLevelCrossing?$"Crossing:{node}":$"{s.Kind}:{node}";

    // A shared OSM node is evidence; mere geometric intersection is not. Reject
    // mixed structures/layers and conflicting horizontal positions as a whole.
    public static HashSet<long> LevelCrossings(IReadOnlyList<PreparedSegment> source)
    {
        var ports=source.SelectMany(s=>new[]{(Section:s,Node:s.StartNodeId,Point:s.CourseStart,Anchor:s.Start),(Section:s,Node:s.EndNodeId,Point:s.CourseEnd,Anchor:s.End)});
        return ports.GroupBy(p=>p.Node).Where(g=>g.Any(p=>p.Section.Kind==NetworkKind.Road)&&g.Any(p=>p.Section.Kind==NetworkKind.Rail)&&
            g.All(p=>!NetworkStructure.IsStructure(p.Section)&&p.Section.Layer==g.First().Section.Layer&&math.all(p.Point.xz==g.First().Point.xz)&&
                math.all(p.Anchor.xz==g.First().Anchor.xz)&&math.all(math.isfinite(p.Point))&&math.all(math.isfinite(p.Anchor))))
            .Select(g=>g.Key).ToHashSet();
    }

    public static IReadOnlyList<PreparedSegment> PrepareLevelCrossings(IReadOnlyList<PreparedSegment> source,ImportLog log)
    {
        var crossings=LevelCrossings(source);
        var heights=source.Where(s=>s.Kind==NetworkKind.Rail).SelectMany(s=>new[]{(Node:s.StartNodeId,Point:s.CourseStart),(Node:s.EndNodeId,Point:s.CourseEnd)})
            .Where(p=>crossings.Contains(p.Node)).GroupBy(p=>p.Node).ToDictionary(g=>g.Key,g=>g.First().Point.y);
        // Only rounding-size residuals may be canonicalized here. Larger profile
        // failures remain separate, reported crossings rather than warped decks.
        foreach(var s in source)foreach(var p in new[]{(Node:s.StartNodeId,Point:s.CourseStart),(Node:s.EndNodeId,Point:s.CourseEnd)})
            if(heights.TryGetValue(p.Node,out var y)&&Math.Abs(p.Point.y-y)>.25f)crossings.Remove(p.Node);
        var result=source.Select(s=>
        {
            var a=crossings.Contains(s.StartNodeId);var b=crossings.Contains(s.EndNodeId);
            if(!a&&!b)return s.StartLevelCrossing||s.EndLevelCrossing?s with{StartLevelCrossing=false,EndLevelCrossing=false}:s;
            var da=a?new float3(0,heights[s.StartNodeId]-s.CourseStart.y,0):float3.zero;
            var db=b?new float3(0,heights[s.EndNodeId]-s.CourseEnd.y,0):float3.zero;
            return s with{Start=s.Start+da,End=s.End+db,Control1=a?s.CourseControl1-s.StartCurveOffset+da:s.Control1,Control2=b?s.CourseControl2-s.EndCurveOffset+db:s.Control2,
                FinalDeckCurve=s.FinalDeckCurve.HasValue?new(s.CourseStart+da,s.CourseControl1+da,s.CourseControl2+db,s.CourseEnd+db):null,
                StartLevelCrossing=a,EndLevelCrossing=b};
        }).ToArray();
        log.Count("Native level crossings prepared",crossings.Count);
        log.Count("Native level crossings rejected for height mismatch",heights.Count-crossings.Count);
        return result;
    }
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
