using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// A divided surface crossing is sometimes mapped as a small directed box.
// Recover its single junction before pairing the approaching carriageways.
public static class StreetIntersectionConsolidator
{
    public static IReadOnlyList<PreparedSegment> Prepare(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token)
    {
        bool Street(PreparedSegment s)=>s.Kind==NetworkKind.Road&&!NativeAssetKeys.IsHighway(s)&&!s.Bridge&&!s.Tunnel&&!s.Roundabout&&
            !s.StartMiniRoundabout&&!s.EndMiniRoundabout;
        string Identity(PreparedSegment s)=>s.Name.Trim().Length>0?s.Name.Trim().ToUpperInvariant():s.Reference.Trim().ToUpperInvariant();
        var incident=source.SelectMany(s=>new[]{(Id:s.StartNodeId,Edge:s),(Id:s.EndNodeId,Edge:s)})
            .GroupBy(p=>p.Id).ToDictionary(g=>g.Key,g=>g.Select(p=>p.Edge).ToArray());
        var small=source.Where(s=>Street(s)&&s.OneWay&&math.distance(s.Start.xz,s.End.xz)<=48&&math.distance(s.Start.xz,s.End.xz)>=2).ToArray();
        var outgoing=small.GroupBy(s=>s.StartNodeId).ToDictionary(g=>g.Key,g=>g.ToArray());
        var remap=new Dictionary<long,long>();var positions=new Dictionary<long,float3>();var removed=new HashSet<PreparedSegment>();var count=0;
        foreach(var a in small.OrderBy(s=>s.StartNodeId).ThenBy(s=>s.EndNodeId))
        {
            token.ThrowIfCancellationRequested();
            if(remap.ContainsKey(a.StartNodeId)||!outgoing.TryGetValue(a.EndNodeId,out var second))continue;
            foreach(var b in second)foreach(var c in outgoing.TryGetValue(b.EndNodeId,out var third)?third:Array.Empty<PreparedSegment>())
            foreach(var d in outgoing.TryGetValue(c.EndNodeId,out var fourth)?fourth:Array.Empty<PreparedSegment>())
            {
                if(d.EndNodeId!=a.StartNodeId)continue;
                var box=new[]{a,b,c,d};var nodes=box.Select(s=>s.StartNodeId).ToArray();
                if(nodes.Distinct().Count()!=4||nodes.Any(remap.ContainsKey)||nodes.Min()!=a.StartNodeId)continue;
                var all=nodes.SelectMany(id=>incident[id]).Distinct().ToArray();
                if(all.Any(s=>!Street(s)||s.Layer!=a.Layer))continue;
                var center=box.Aggregate(float3.zero,(p,s)=>p+s.Start)/4;
                if(box.Any(s=>math.distance(s.Start.xz,center.xz)>32))continue;
                var rectangular=true;
                for(var k=0;k<4;k++)if(Math.Abs(math.dot(math.normalizesafe((box[k].End-box[k].Start).xz),
                    math.normalizesafe((box[(k+1)%4].End-box[(k+1)%4].Start).xz)))>.5f)rectangular=false;
                if(!rectangular)continue;
                var internalEdges=all.Where(s=>nodes.Contains(s.StartNodeId)&&nodes.Contains(s.EndNodeId)).ToArray();
                if(internalEdges.Any(s=>!box.Contains(s)))continue;
                var external=all.Except(box).ToArray();if(external.Length<6)continue;
                var axis=math.normalizesafe((a.End-a.Start).xz);var normal=new float2(-axis.y,axis.x);
                var groups=new List<(PreparedSegment Edge,bool Outgoing)>[4];for(var k=0;k<4;k++)groups[k]=new();
                var valid=true;
                foreach(var arm in external)
                {
                    var atStart=nodes.Contains(arm.StartNodeId);var away=math.normalizesafe((atStart?arm.End-arm.Start:arm.Start-arm.End).xz);
                    var x=math.dot(away,axis);var y=math.dot(away,normal);var parallel=Math.Abs(x)>=Math.Abs(y);
                    if(!arm.OneWay||Identity(arm).Length==0||Math.Max(Math.Abs(x),Math.Abs(y))<Math.Cos(20*Math.PI/180)){valid=false;break;}
                    groups[parallel?(x>0?0:2):(y>0?1:3)].Add((arm,atStart));
                }
                if(!valid||groups.Count(g=>g.Count>0)<3)continue;
                if(groups.Where(g=>g.Count>0).Any(g=>g.Count!=2||g[0].Outgoing==g[1].Outgoing||Identity(g[0].Edge)!=Identity(g[1].Edge)))continue;
                for(var k=0;k<2;k++)if(groups[k].Count>0&&groups[k+2].Count>0&&Identity(groups[k][0].Edge)!=Identity(groups[k+2][0].Edge))valid=false;
                if(!valid)continue;
                foreach(var id in nodes){remap[id]=a.StartNodeId;positions[id]=center;}
                foreach(var edge in box)removed.Add(edge);count++;
            }
        }
        if(count==0)return source;
        var result=new List<PreparedSegment>(source.Count-removed.Count);
        foreach(var s in source)
        {
            if(removed.Contains(s))continue;
            if(!remap.ContainsKey(s.StartNodeId)&&!remap.ContainsKey(s.EndNodeId)){result.Add(s);continue;}
            var p=positions.TryGetValue(s.StartNodeId,out var start)?start:s.Start;var q=positions.TryGetValue(s.EndNodeId,out var end)?end:s.End;
            result.Add(s with{StartNodeId=remap.TryGetValue(s.StartNodeId,out var first)?first:s.StartNodeId,
                EndNodeId=remap.TryGetValue(s.EndNodeId,out var last)?last:s.EndNodeId,Start=p,End=q,SourceGeometry=new[]{p,q},
                Control1=null,Control2=null,HorizontalParent=null,ParentStartT=0,ParentEndT=1,StartCurveOffset=float3.zero,EndCurveOffset=float3.zero});
        }
        log.Count("Divided street intersection boxes consolidated",count);
        log.Diagnostic($"STREET INTERSECTIONS boxes={count}; explicitDirectedCycle=true; opposingNamedApproaches=true; majorRoadsAndStructuresProtected=true");
        return SourceTopologyObligations.Transfer(source,result,NetworkKind.Road,remap);
    }
}
