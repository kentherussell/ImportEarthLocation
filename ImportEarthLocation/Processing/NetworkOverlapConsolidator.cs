using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Consolidate genuinely coincident intervals before curves/widths are fitted.
// Opposite carriageways and nearby parallel collector roads are not duplicates.
public static class NetworkOverlapConsolidator
{
    public static IReadOnlyList<PreparedSegment> Prepare(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token)
    {
        var removed=new HashSet<int>();var tails=new Dictionary<int,List<PreparedSegment>>();
        var knots=new Dictionary<int,List<(float t,long id,float3 point)>>();var remap=new Dictionary<long,(long id,float3 point)>();
        var ids=new HashSet<long>(source.SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}));var next=long.MinValue+7000000;
        long Id(){while(ids.Contains(next))next++;var id=next++;ids.Add(id);return id;}
        (long id,float3 point) Knot(int index,float t)
        {
            var s=source[index];if(t<=.001f)return(s.StartNodeId,s.Start);if(t>=.999f)return(s.EndNodeId,s.End);
            if(!knots.TryGetValue(index,out var list))knots[index]=list=new();var point=math.lerp(s.Start,s.End,t);
            foreach(var p in list)if(math.distance(p.point.xz,point.xz)<1)return(p.id,p.point);
            var id=Id();list.Add((t,id,point));return(id,point);
        }
        var groups=source.Select((s,i)=>(s,i)).Where(p=>p.s.Kind==NetworkKind.Road&&!p.s.ProtectedLink&&p.s.OneWay&&p.s.Classification is "motorway" or "trunk")
            .GroupBy(p=>(p.s.Classification,p.s.Layer,p.s.Bridge,p.s.Tunnel,route:p.s.Reference.Length>0?p.s.Reference:p.s.Name.Length>0?p.s.Name:"way:"+p.s.WayId));
        var overlaps=0;
        foreach(var group in groups)
        {
            var retained=new List<int>();
            foreach(var entry in group.OrderByDescending(p=>math.distance(p.s.Start.xz,p.s.End.xz)).ThenBy(p=>p.s.WayId))
            {
                token.ThrowIfCancellationRequested();var s=entry.s;var handled=false;
                foreach(var index in retained)
                {
                    var r=source[index];var d=r.End.xz-r.Start.xz;var e=s.End.xz-s.Start.xz;var square=math.lengthsq(d);if(square<16)continue;
                    if(math.dot(math.normalizesafe(d),math.normalizesafe(e))<.999f)continue;
                    // No arbitrary prefab-width merging: raw surveyed centers
                    // must agree within 1.5m on the shared interval.
                    var a=math.dot(s.Start.xz-r.Start.xz,d)/square;var b=math.dot(s.End.xz-r.Start.xz,d)/square;
                    if(b<=a)continue;var lo=Math.Max(0,a);var hi=Math.Min(1,b);if((hi-lo)*Math.Sqrt(square)<8)continue;
                    var startT=(lo-a)/(b-a);var endT=(hi-a)/(b-a);
                    if(math.distance(math.lerp(s.Start.xz,s.End.xz,startT),math.lerp(r.Start.xz,r.End.xz,lo))>1.5f||
                       math.distance(math.lerp(s.Start.xz,s.End.xz,endT),math.lerp(r.Start.xz,r.End.xz,hi))>1.5f)continue;
                    var first=Knot(index,lo);var last=Knot(index,hi);var unique=new List<PreparedSegment>();
                    if(startT>.001f)unique.Add(s with{EndNodeId=first.id,End=first.point,Control1=null,Control2=null,SourceGeometry=new[]{s.Start,first.point}});
                    else remap[s.StartNodeId]=first;
                    if(endT<.999f)unique.Add(s with{StartNodeId=last.id,Start=last.point,Control1=null,Control2=null,SourceGeometry=new[]{last.point,s.End}});
                    else remap[s.EndNodeId]=last;
                    tails[entry.i]=unique;removed.Add(entry.i);overlaps++;handled=true;break;
                }
                if(!handled)retained.Add(entry.i);
            }
        }
        var output=new List<PreparedSegment>();
        for(var i=0;i<source.Count;i++)
        {
            token.ThrowIfCancellationRequested();var s=source[i];
            IEnumerable<PreparedSegment> pieces;
            if(removed.Contains(i))pieces=tails[i];
            else if(knots.TryGetValue(i,out var list))
            {
                var stations=list.Concat(new[]{(t:0f,id:s.StartNodeId,point:s.Start),(t:1f,id:s.EndNodeId,point:s.End)}).OrderBy(p=>p.t).ToArray();
                var splits=new List<PreparedSegment>();for(var j=1;j<stations.Length;j++)
                    splits.Add(s with{StartNodeId=stations[j-1].id,Start=stations[j-1].point,EndNodeId=stations[j].id,End=stations[j].point,Control1=null,Control2=null,SourceGeometry=new[]{stations[j-1].point,stations[j].point}});
                pieces=splits;
            }
            else pieces=new[]{s};
            foreach(var piece in pieces)
            {
                var p=piece;
                if(p.Kind==NetworkKind.Road)
                {
                    if(remap.TryGetValue(p.StartNodeId,out var a))p=p with{StartNodeId=a.id,Start=a.point,Control1=null};
                    if(remap.TryGetValue(p.EndNodeId,out var b))p=p with{EndNodeId=b.id,End=b.point,Control2=null};
                }
                if(p.StartNodeId!=p.EndNodeId)output.Add(p);
            }
        }
        log.Count("Highway coincident intervals consolidated",overlaps);
        log.Diagnostic($"HIGHWAY INTERVALS overlappingIntervalsConsolidated={overlaps}; uniqueExtensionsRetained=true; incidentNodesTransferred={remap.Count}; oppositeCarriagewaysPreserved=true; rawCenterToleranceMeters=1.5; generationGates=false");
        return output;
    }
}
