using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Consolidation can project the two ends of ONE median junction onto different
// centerline stations. Repair that explicit connection, never nearest neighbours.
// Run before curve fitting/grading/roadbed generation so all downstream geometry
// is solved against the same canonical topology.
public static class AvenueJunctionRepair
{
    public static IReadOnlyList<PreparedSegment> Prepare(IReadOnlyList<PreparedSegment> source, ImportLog log, CancellationToken token)
    {
        token.ThrowIfCancellationRequested();
        var incident=new Dictionary<long,List<PreparedSegment>>();
        foreach(var s in source)
        foreach(var id in new[]{s.StartNodeId,s.EndNodeId})
        {if(!incident.TryGetValue(id,out var list))incident[id]=list=new();list.Add(s);}
        var parent=new Dictionary<long,long>();var points=new Dictionary<long,float3>();
        long Root(long id){while(parent.TryGetValue(id,out var next)&&next!=id)id=next;return id;}
        bool Ground(PreparedSegment s)=>s.Kind==NetworkKind.Road&&!s.Bridge&&!s.Tunnel&&!s.Roundabout&&!s.StartMiniRoundabout&&!s.EndMiniRoundabout;
        float Width(PreparedSegment s)=>math.isfinite(s.NativeWidth)&&s.NativeWidth>0?s.NativeWidth:s.Lanes>4?32:24;
        var collapsed=new HashSet<PreparedSegment>();
        foreach(var link in source.OrderBy(s=>s.WayId).ThenBy(s=>s.StartNodeId).ThenBy(s=>s.EndNodeId))
        {
            token.ThrowIfCancellationRequested();
            if(link.ConsolidatedAvenue||!Ground(link)||link.StartNodeId==link.EndNodeId)continue;
            var arms=incident[link.StartNodeId].Concat(incident[link.EndNodeId]).Distinct().ToArray();
            // A structure, rail crossing, roundabout or another layer is not a
            // recoverable median junction merely because it is close by.
            if(arms.Any(s=>!Ground(s)||s.Layer!=link.Layer))continue;
            var a=incident[link.StartNodeId].Where(s=>s.ConsolidatedAvenue).ToArray();
            var b=incident[link.EndNodeId].Where(s=>s.ConsolidatedAvenue).ToArray();
            if(a.Length==0||b.Length==0)continue;
            var distance=math.distance(link.Start.xz,link.End.xz);
            var bound=Math.Min(32,Math.Min(a.Min(Width),b.Min(Width)));
            if(!math.isfinite(distance)||distance>bound)continue;
            var duplicate=arms.Any(s=>s.ConsolidatedAvenue&&
                ((s.StartNodeId==link.StartNodeId&&s.EndNodeId==link.EndNodeId)||(s.EndNodeId==link.StartNodeId&&s.StartNodeId==link.EndNodeId)));
            var original=link.OriginalGeometry;
            var transverse=original.Count>=2&&distance>.01f&&
                Math.Abs(math.dot(math.normalizesafe(original[original.Count-1].xz-original[0].xz),math.normalizesafe(link.End.xz-link.Start.xz)))<.5f;
            var sourceEvidence=arms.Any(s=>s.ConsolidatedAvenue&&s.SourceWayIds.Contains(link.WayId));
            // Palm Springs: a transverse median link became a duplicate
            // longitudinal avenue interval. Oman: an explicit ~1 m tie-in
            // between remapped junctions retained its own carriageway evidence.
            if(!(duplicate&&transverse)&&!(distance<=2&&sourceEvidence))continue;
            var first=Root(link.StartNodeId);var last=Root(link.EndNodeId);
            if(first==last){collapsed.Add(link);continue;}
            var members=points.Keys.Concat(new[]{link.StartNodeId,link.EndNodeId}).Distinct()
                .Where(id=>Root(id)==first||Root(id)==last).OrderBy(id=>id).ToArray();
            float3 Point(long id)=>points.TryGetValue(id,out var p)?p:incident[id][0].StartNodeId==id?incident[id][0].Start:incident[id][0].End;
            var origin=Point(members[0]);var center=origin;
            var sum=float3.zero;foreach(var id in members)sum+=Point(id)-origin;center+=sum/members.Length;
            if(members.Any(id=>math.distance(Point(id).xz,center.xz)>bound*.5f))continue;
            var set=new HashSet<long>(members);
            // Do not accidentally consume a real third street inside a cluster.
            if(members.SelectMany(id=>incident[id]).Any(s=>set.Contains(s.StartNodeId)&&set.Contains(s.EndNodeId)&&!s.ConsolidatedAvenue&&!collapsed.Contains(s)&&s!=link))continue;
            var canonical=Math.Min(first,last);parent[first]=canonical;parent[last]=canonical;
            foreach(var id in members)points[id]=Point(id);
            collapsed.Add(link);
            log.Diagnostic($"AVENUE JUNCTION RECOVERY connectorWay={link.WayId}; sourceNodes={string.Join(",",members)}; canonical={canonical}; lengthMeters={distance:R}; duplicateInterval={duplicate}; transverseOriginal={transverse}; sourceEvidence={sourceEvidence}; center=({center.x:R},{center.y:R},{center.z:R}); explicitConnectionOnly=true; requiresDownstreamReaudit=true");
        }
        if(collapsed.Count==0)return source;
        var centers=points.Keys.GroupBy(Root).ToDictionary(g=>g.Key,g=>
        {var p=points[g.First()];var sum=float3.zero;foreach(var id in g)sum+=points[id]-p;return p+sum/g.Count();});
        var result=new List<PreparedSegment>(source.Count);var internalIntervals=0;var reconnected=0;
        foreach(var s in source)
        {
            token.ThrowIfCancellationRequested();if(collapsed.Contains(s))continue;
            var a=Root(s.StartNodeId);var b=Root(s.EndNodeId);
            if(a==b&&s.ConsolidatedAvenue){internalIntervals++;continue;}
            if(!points.ContainsKey(s.StartNodeId)&&!points.ContainsKey(s.EndNodeId)){result.Add(s);continue;}
            var start=points.ContainsKey(s.StartNodeId)?centers[a]:s.Start;
            var end=points.ContainsKey(s.EndNodeId)?centers[b]:s.End;
            result.Add(s with{StartNodeId=a,EndNodeId=b,Start=start,End=end,SourceGeometry=new[]{start,end},
                Control1=null,Control2=null,HorizontalParent=null,ParentStartT=0,ParentEndT=1,StartCurveOffset=float3.zero,EndCurveOffset=float3.zero});
            reconnected++;
        }
        log.Count("Avenue median junction connectors canonicalized",collapsed.Count);
        log.Count("Avenue internal duplicate intervals collapsed",internalIntervals);
        log.Diagnostic($"AVENUE JUNCTION RECOVERY complete; connectors={collapsed.Count}; internalIntervals={internalIntervals}; armsReconnected={reconnected}; outputSections={result.Count}; nearbyUnconnectedRoadsUntouched=true");
        return result;
    }
}
