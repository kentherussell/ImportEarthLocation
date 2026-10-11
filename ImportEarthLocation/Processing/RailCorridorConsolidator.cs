using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

/// <summary>Fold parallel raw track spans onto a retained rail axis, splitting it at every transferred junction.</summary>
public static class RailCorridorConsolidator
{
    public const float CorridorDistance = 7.5f;
    private sealed class Axis
    {
        public PreparedSegment[] Edges = Array.Empty<PreparedSegment>();
        public float[] LengthPrefix = Array.Empty<float>();
        public int[] BreakPrefix = Array.Empty<int>();
        public readonly Dictionary<(int,int),List<int>> Segments = new();
        public readonly Dictionary<int,List<(float t,long node,float3 point)>> Knots = new();
        public readonly List<(float lo,float hi)> PairedIntervals = new();
        public readonly List<(float lo,float hi,PreparedSegment source)> PairedSources = new();
    }
    private readonly struct Projection
    {
        public readonly int Edge;
        public readonly float T,Distance;
        public readonly float3 Point;
        public Projection(int edge,float t,float distance,float3 point){Edge=edge;T=t;Distance=distance;Point=point;}
    }
    public static IReadOnlyList<PreparedSegment> Prepare(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token)
    {
        // Transferring switches can expose a short redundant triangle: the
        // direct axis and an identical two-edge approach. Re-consolidate the
        // canonical raw geometry before fitting curves, preserving all tails.
        var result=ExactPhysicalDemands(source,log,token);
        for(var pass=0;pass<3;pass++)
        {
            var next=PreparePass(result,log,token);
            if(next.SequenceEqual(result))return next;
            result=next;
        }
        return result;
    }
    // Exact raw evidence only. Endpoint/prefab equality cannot establish that
    // separately mapped tracks represent the same physical infrastructure.
    private static IReadOnlyList<PreparedSegment> ExactPhysicalDemands(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token)
    {
        var result=new List<PreparedSegment>(source.Count);
        var buckets=new Dictionary<(long,long,int,bool,bool),List<int>>();var merged=0;
        foreach(var s in source)
        {
            token.ThrowIfCancellationRequested();var retained=-1;
            var key=(s.StartNodeId,s.EndNodeId,s.Layer,s.Bridge,s.Tunnel);
            if(s.Kind==NetworkKind.Rail&&s.RawSourceTags.Count>0&&s.OriginalGeometry.Count>=2&&s.Control1==null&&s.Control2==null)
            {
                if(buckets.TryGetValue(key,out var candidates))
                    foreach(var index in candidates)
                    {
                        var prior=result[index];
                        if(prior.Classification!=s.Classification||prior.OneWay!=s.OneWay||prior.Lanes!=s.Lanes||prior.PhysicalTracks!=s.PhysicalTracks||prior.RailService!=s.RailService||prior.Reference!=s.Reference)continue;
                        if(prior.RawSourceTags.Count!=s.RawSourceTags.Count||s.RawSourceTags.Any(p=>!prior.RawSourceTags.TryGetValue(p.Key,out var value)||value!=p.Value))continue;
                        if(prior.OriginalGeometry.Count!=s.OriginalGeometry.Count||!prior.OriginalGeometry.SequenceEqual(s.OriginalGeometry)||!math.all(prior.Start==s.Start)||!math.all(prior.End==s.End))continue;
                        retained=index;break;
                    }
                if(retained<0)
                {if(!buckets.TryGetValue(key,out candidates))buckets[key]=candidates=new();candidates.Add(result.Count);}
            }
            if(retained<0){result.Add(s);continue;}
            var kept=result[retained];
            var provenance=kept.SourceWayIds.Concat(s.SourceWayIds).Concat(new[]{kept.WayId,s.WayId}).Distinct().OrderBy(id=>id).ToArray();
            result[retained]=kept with{SourceWayIds=provenance};merged++;
            if(merged<=64)log.Diagnostic($"PROCESS exact-rail-demand retainedWay={kept.WayId}; equivalentWay={s.WayId}; rawTagsIdentical=true; originalGeometryIdentical=true; sourceNodesIdentical=true; provenanceCount={provenance.Length}; proximityMerge=false");
        }
        log.Count("Exact duplicate rail physical demands consolidated",merged);
        return merged==0?source:result;
    }
    private static IReadOnlyList<PreparedSegment> PreparePass(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token)
    {
        token.ThrowIfCancellationRequested();var clock=Stopwatch.StartNew();
        var groups=source.Where(s=>s.Kind==NetworkKind.Rail&&s.Control1==null&&s.Control2==null)
            .GroupBy(s=>(s.WayId,s.Layer,s.Bridge,s.Tunnel)).Select(g=>g.ToArray())
            .OrderBy(g=>g.All(s=>s.RailService=="yard"||s.RailService=="siding"||s.RailService=="spur")?1:0)
            .ThenByDescending(g=>g.Sum(s=>math.distance(s.Start.xz,s.End.xz))).ThenBy(g=>g[0].WayId).ToArray();
        if(groups.Length==0)return source;
        var axes=new List<Axis>();var grid=new Dictionary<(int,int),HashSet<int>>();
        var removed=new HashSet<PreparedSegment>();var map=new Dictionary<(int,long),(long node,float3 point)>();
        var protectedNodes=new HashSet<long>(source.Where(s=>s.Kind==NetworkKind.Road).SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}));
        var ids=new HashSet<long>(source.SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}));long next=-7_800_000_000_000_000_000;
        long NewNode(){while(ids.Contains(next))next++;var id=next++;ids.Add(id);return id;}
        Projection Project(Axis axis,float3 point)
        {
            var best=new Projection(-1,0,float.PositiveInfinity,default);
            // Only projections within CorridorDistance are consumed. Expanded segment
            // bounds guarantee that every acceptable exact nearest point is in this cell.
            if(!axis.Segments.TryGetValue(((int)Math.Floor(point.x/128),(int)Math.Floor(point.z/128)),out var nearby))return best;
            foreach(var i in nearby)
            {
                var e=axis.Edges[i];var delta=e.End.xz-e.Start.xz;var t=math.lengthsq(delta)<.0001f?0:math.clamp(math.dot(point.xz-e.Start.xz,delta)/math.lengthsq(delta),0,1);
                var p=math.lerp(e.Start,e.End,t);var distance=math.distance(point.xz,p.xz);
                if(distance<best.Distance||distance==best.Distance&&i<best.Edge)best=new Projection(i,t,distance,p);
            }
            return best;
        }
        IEnumerable<(int,int)> Cells(PreparedSegment edge)
        {
            var lo=math.min(edge.Start.xz,edge.End.xz)-CorridorDistance;var hi=math.max(edge.Start.xz,edge.End.xz)+CorridorDistance;
            for(var z=(int)Math.Floor(lo.y/128);z<=(int)Math.Floor(hi.y/128);z++)
                for(var x=(int)Math.Floor(lo.x/128);x<=(int)Math.Floor(hi.x/128);x++)yield return(x,z);
        }
        void AddAxis(PreparedSegment[] edges)
        {
            var id=axes.Count;var axis=new Axis{Edges=edges,LengthPrefix=new float[edges.Length+1],BreakPrefix=new int[edges.Length+1]};axes.Add(axis);
            for(var i=0;i<edges.Length;i++)
            {
                var edge=edges[i];axis.LengthPrefix[i+1]=axis.LengthPrefix[i]+math.distance(edge.Start.xz,edge.End.xz);
                axis.BreakPrefix[i+1]=axis.BreakPrefix[i]+(i>0&&edges[i-1].EndNodeId!=edge.StartNodeId?1:0);
                foreach(var cell in Cells(edge))
                {
                    if(!grid.TryGetValue(cell,out var list))grid[cell]=list=new HashSet<int>();list.Add(id);
                    if(!axis.Segments.TryGetValue(cell,out var segments))axis.Segments[cell]=segments=new();segments.Add(i);
                }
            }
        }
        var merged=0;var nextProgress=5d;
        foreach(var group in groups)
        {
            token.ThrowIfCancellationRequested();
            if(clock.Elapsed.TotalSeconds>=nextProgress){log.Diagnostic($"PROCESS rail-corridors axes={axes.Count}; spansConsolidated={removed.Count}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.0}");nextProgress=clock.Elapsed.TotalSeconds+5;}
            var candidates=new HashSet<int>();foreach(var edge in group)foreach(var cell in Cells(edge))if(grid.TryGetValue(cell,out var entries))candidates.UnionWith(entries);
            var options=new List<(int axis,List<(PreparedSegment edge,Projection a,Projection b)> spans,float length,float distance)>();
            foreach(var index in candidates.OrderBy(i=>i))
            {
                var axis=axes[index];var retainedSource=axis.Edges[0];var candidateSource=group[0];
                if(retainedSource.Bridge!=candidateSource.Bridge||retainedSource.Tunnel!=candidateSource.Tunnel||retainedSource.Layer!=candidateSource.Layer)continue;
                // Yard ladders, sidings, and separately mapped double corridors are not duplicates.
                if(retainedSource.Lanes>1||candidateSource.Lanes>1||retainedSource.RailService.Length>0||candidateSource.RailService.Length>0)continue;
                if(retainedSource.Reference.Length>0&&candidateSource.Reference.Length>0&&retainedSource.Reference!=candidateSource.Reference)continue;
                var spans=new List<(PreparedSegment,Projection,Projection)>();var length=0f;var distance=0f;
                foreach(var edge in group)
                {
                    token.ThrowIfCancellationRequested();if(protectedNodes.Contains(edge.StartNodeId)||protectedNodes.Contains(edge.EndNodeId))continue;
                    var a=Project(axis,edge.Start);var b=Project(axis,edge.End);var middle=Project(axis,(edge.Start+edge.End)*.5f);
                    var originalLength=math.distance(edge.Start.xz,edge.End.xz);var projectedLength=math.distance(a.Point.xz,b.Point.xz);
                    if(a.Distance>CorridorDistance||b.Distance>CorridorDistance||middle.Distance>CorridorDistance||originalLength<.5f||projectedLength<.5f||projectedLength/originalLength<.8f||projectedLength/originalLength>1.2f)continue;
                    if(Math.Abs(math.dot(math.normalizesafe(edge.End.xz-edge.Start.xz),math.normalizesafe(axis.Edges[middle.Edge].End.xz-axis.Edges[middle.Edge].Start.xz)))<.97f)continue;
                    // Different OSM ways often have very different vertex spacing.
                    // Allow any contiguous retained interval, but never bridge a gap
                    // in its source topology or substitute a long looping detour.
                    var lo=Math.Min(a.Edge,b.Edge);var hi=Math.Max(a.Edge,b.Edge);
                    var connected=axis.BreakPrefix[hi+1]==axis.BreakPrefix[lo+1];
                    var aDistance=axis.LengthPrefix[a.Edge]+a.T*(axis.LengthPrefix[a.Edge+1]-axis.LengthPrefix[a.Edge]);
                    var bDistance=axis.LengthPrefix[b.Edge]+b.T*(axis.LengthPrefix[b.Edge+1]-axis.LengthPrefix[b.Edge]);
                    var arc=Math.Abs(aDistance-bDistance);
                    if(!connected||arc/originalLength<.8f||arc/originalLength>1.2f)continue;
                    // One native double-track axis represents two mapped tracks,
                    // not an arbitrary number of parallel routes. Allow disjoint
                    // continuation partners, but never fold a third overlapping track.
                    var intervalLo=Math.Min(aDistance,bDistance);var intervalHi=Math.Max(aDistance,bDistance);
                    if(axis.PairedIntervals.Any(p=>Math.Min(p.hi,intervalHi)-Math.Max(p.lo,intervalLo)>2))continue;
                    if(map.TryGetValue((edge.Layer,edge.StartNodeId),out var oldA)&&math.distance(oldA.point.xz,a.Point.xz)>.5f ||
                       map.TryGetValue((edge.Layer,edge.EndNodeId),out var oldB)&&math.distance(oldB.point.xz,b.Point.xz)>.5f)continue;
                    spans.Add((edge,a,b));length+=originalLength;distance+=(a.Distance+b.Distance)*.5f*originalLength;
                }
                // Fold even a short overlapping interval. Unique extensions are
                // retained and reattached, not dropped because the whole way differs.
                if(length>=40 && distance/length>=1.5f)options.Add((index,spans,length,distance/length));
            }
            var ranked=options.OrderByDescending(o=>o.length).ThenBy(o=>o.distance).ThenBy(o=>o.axis).ToArray();
            var best=ranked.FirstOrDefault();
            if(ranked.Length>1&&ranked[1].length>=best.length*.9f&&Math.Abs(ranked[1].distance-best.distance)<1f)
            {log.Count("Ambiguous parallel rail corridors preserved");AddAxis(group);continue;}
            if(best.spans==null){AddAxis(group);continue;}
            var retained=axes[best.axis];
            (long node,float3 point) Junction(Projection p)
            {
                var edge=retained.Edges[p.Edge];
                // Junctions closer than the native track footprint cannot form
                // separate useful edges. Snap transferred knots to the existing
                // junction rather than creating centimetre-long graded tracks.
                if(math.distance(p.Point.xz,edge.Start.xz)<4f)return(edge.StartNodeId,edge.Start);
                if(math.distance(p.Point.xz,edge.End.xz)<4f)return(edge.EndNodeId,edge.End);
                if(!retained.Knots.TryGetValue(p.Edge,out var knots))retained.Knots[p.Edge]=knots=new List<(float,long,float3)>();
                foreach(var knot in knots)if(math.distance(knot.point.xz,p.Point.xz)<4f)return(knot.node,knot.point);
                var node=NewNode();knots.Add((p.T,node,p.Point));return(node,p.Point);
            }
            foreach(var span in best.spans)
            {
                map[(span.edge.Layer,span.edge.StartNodeId)]=Junction(span.a);map[(span.edge.Layer,span.edge.EndNodeId)]=Junction(span.b);removed.Add(span.edge);
                float Station(Projection p)=>retained.LengthPrefix[p.Edge]+p.T*(retained.LengthPrefix[p.Edge+1]-retained.LengthPrefix[p.Edge]);
                var first=Station(span.a);var last=Station(span.b);retained.PairedIntervals.Add((Math.Min(first,last),Math.Max(first,last)));
                retained.PairedSources.Add((Math.Min(first,last),Math.Max(first,last),span.edge));
            }
            // Unique extensions remain and are later reattached to the retained axis.
            var tails=group.Where(e=>!removed.Contains(e)).ToArray();merged++;
            log.Diagnostic($"RAIL MERGE sourceWay={group[0].WayId}; retainedWay={retained.Edges[0].WayId}; consolidatedSpans={best.spans.Count}; retainedTailSpans={tails.Length}; meanSeparationMeters={best.distance:0.##}; transferredJunctions=true");
        }
        var replacements=new Dictionary<PreparedSegment,List<PreparedSegment>>();
        foreach(var axis in axes)foreach(var entry in axis.Knots)
        {
            var e=axis.Edges[entry.Key];var knots=entry.Value.Concat(new[]{(t:0f,node:e.StartNodeId,point:e.Start),(t:1f,node:e.EndNodeId,point:e.End)}).OrderBy(k=>k.t).ToArray();
            var pieces=new List<PreparedSegment>();for(var i=1;i<knots.Length;i++)pieces.Add(e with{StartNodeId=knots[i-1].node,EndNodeId=knots[i].node,Start=knots[i-1].point,End=knots[i].point,SourceGeometry=new[]{knots[i-1].point,knots[i].point}});
            replacements[e]=pieces;
        }
        var output=new List<PreparedSegment>();var reconnected=0;
        var pairedAxes=axes.Where(a=>a.PairedIntervals.Count>0).SelectMany(a=>a.Edges.Select(e=>(e.WayId,axis:a))).GroupBy(p=>p.WayId).ToDictionary(g=>g.Key,g=>g.Select(p=>p.axis).Distinct().ToArray());
        // Portal ways commonly change layer tags. A source node shared across that
        // transition must follow its unique canonical junction, not the old axis.
        var portalMap=map.GroupBy(p=>p.Key.Item2).Where(g=>g.Select(p=>p.Value.node).Distinct().Count()==1)
            .ToDictionary(g=>g.Key,g=>g.First().Value);
        foreach(var edge in source)
        {
            token.ThrowIfCancellationRequested();if(removed.Contains(edge))continue;
            foreach(var original in replacements.TryGetValue(edge,out var pieces)?(IEnumerable<PreparedSegment>)pieces:new[]{edge})
            {
                var paired=original.Kind==NetworkKind.Rail&&axisFor(original);
                var s=original.Kind==NetworkKind.Rail?original with{Lanes=paired?2:original.Lanes,PhysicalTracks=paired?2:original.PhysicalTracks,OneWay=false}:original;
                if(paired&&pairedAxes.TryGetValue(original.WayId,out var owners))
                {
                    var absorbed=owners.SelectMany(axis=>axis.PairedSources.Where(p=>NetworkNormalizer.DistanceToSegment((s.Start+s.End)*.5f,p.source.Start,p.source.End)<=CorridorDistance+1).Select(p=>p.source)).ToArray();
                    s=s with{SourceWayIds=new[]{s}.Concat(absorbed).SelectMany(p=>p.SourceWayIds.Count>0?p.SourceWayIds:new[]{p.WayId}).Distinct().ToArray(),
                        OriginalGeometry=new[]{s}.Concat(absorbed).SelectMany(p=>p.OriginalGeometry).ToArray()};
                }
                if(s.Kind==NetworkKind.Rail)
                {
                    if(map.TryGetValue((s.Layer,s.StartNodeId),out var a)||portalMap.TryGetValue(s.StartNodeId,out a)){s=s with{StartNodeId=a.node,Start=a.point};reconnected++;}
                    if(map.TryGetValue((s.Layer,s.EndNodeId),out var b)||portalMap.TryGetValue(s.EndNodeId,out b)){s=s with{EndNodeId=b.node,End=b.point};reconnected++;}
                    if(s.StartNodeId==s.EndNodeId){log.Count("Rail crossovers consolidated into junctions");continue;}
                    if(s!=original)s=s with{SourceGeometry=new[]{s.Start,s.End}};
                }
                output.Add(s);
            }
        }
        log.Count("Rail parallel spans consolidated",removed.Count);
        log.Diagnostic($"RAIL TOTAL waysConsolidated={merged}; spansConsolidated={removed.Count}; endpointsReconnected={reconnected}; corridorDistanceMeters={CorridorDistance}; outputRailSpans={output.Count(s=>s.Kind==NetworkKind.Rail)}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.###}; bridgeTunnelSeparationPreserved=true; differentLayersPreserved=true; pairedTracksLimitedToTwo=true; yardsAndSidingsPreserved=true");
        return RailJunctionRepair.Prepare(output,log,token);

        bool axisFor(PreparedSegment section)
        {
            // Only intervals which actually absorbed a second physical track are doubled.
            if(!pairedAxes.TryGetValue(section.WayId,out var owners))return false;
            foreach(var axis in owners)
            {
                if(axis.PairedIntervals.Count==0)continue;
                for(var i=0;i<axis.Edges.Length;i++)
                {
                    var e=axis.Edges[i];if(e.WayId!=section.WayId)continue;
                    var delta=e.End.xz-e.Start.xz;var sq=math.lengthsq(delta);if(sq<.001f)continue;
                    var mid=(section.Start.xz+section.End.xz)*.5f;
                    var t=math.dot(mid-e.Start.xz,delta)/sq;if(t<0||t>1||math.distance(mid,math.lerp(e.Start.xz,e.End.xz,t))>.1f)continue;
                    var station=math.lerp(axis.LengthPrefix[i],axis.LengthPrefix[i+1],t);
                    if(axis.PairedIntervals.Any(p=>station>=p.lo&&station<=p.hi))return true;
                }
            }
            return false;
        }
    }
}
