using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

/// <summary>Consolidate ordinary divided streets while retaining attached source junctions.</summary>
public static class AvenueConsolidator
{
    private sealed class Chain
    {
        public PreparedSegment[] Edges = Array.Empty<PreparedSegment>();
        public float[] Distances = Array.Empty<float>();
        public float Length => Distances[Distances.Length - 1];
        public float3 Start => Edges[0].Start;
        public float3 End => Edges[Edges.Length - 1].End;
        public float3 Point(float fraction)
        {
            var d=math.clamp(fraction,0,1)*Length;
            var i=Array.BinarySearch(Distances,d); if(i<0)i=~i-1;
            i=Math.Max(0,Math.Min(Edges.Length-1,i));
            var edge=Edges[i]; var length=Distances[i+1]-Distances[i];
            return math.lerp(edge.Start,edge.End,length<=.001f?0:(d-Distances[i])/length);
        }
        public PreparedSegment SourceAt(float fraction)
        {
            var i=Array.BinarySearch(Distances,math.clamp(fraction,0,1)*Length); if(i<0)i=~i-1;
            return Edges[Math.Max(0,Math.Min(Edges.Length-1,i))];
        }
        public int Lanes(float fraction) => SourceAt(fraction).Lanes;
        public PreparedSegment[] SourcesBetween(float begin,float end)
        {
            var lo=Math.Min(begin,end)*Length;var hi=Math.Max(begin,end)*Length;
            return Enumerable.Range(0,Edges.Length).Where(i=>Distances[i]<hi&&Distances[i+1]>lo).Select(i=>Edges[i]).ToArray();
        }
        public float Project(float3 point)
        {
            var best=float.PositiveInfinity;var station=0f;
            for(var i=0;i<Edges.Length;i++)
            {
                var s=Edges[i];var delta=s.End.xz-s.Start.xz;var t=math.clamp(math.dot(point.xz-s.Start.xz,delta)/Math.Max(.001f,math.lengthsq(delta)),0,1);
                var gap=math.distancesq(point.xz,math.lerp(s.Start.xz,s.End.xz,t));if(gap>=best)continue;
                best=gap;station=math.lerp(Distances[i],Distances[i+1],t);
            }
            return station/Length;
        }
    }
    private static bool Eligible(PreparedSegment s) => s.Kind==NetworkKind.Road&&s.OneWay&&
        !s.Classification.EndsWith("_link",StringComparison.OrdinalIgnoreCase)&&
        (s.Classification=="primary"||s.Classification=="secondary"||s.Classification=="tertiary"||s.Classification=="residential"||s.Classification=="unclassified")&&
        !s.Roundabout&&s.Control1==null&&s.Control2==null;
    private static bool TrunkEligible(PreparedSegment s) => s.Kind==NetworkKind.Road&&s.OneWay&&s.Classification=="trunk"&&
        !s.Bridge&&!s.Tunnel&&!s.Roundabout&&s.Control1==null&&s.Control2==null;
    // Prefer whole named corridors before the local geometric completion pass.
    private static string Identity(PreparedSegment s) => "street:"+s.Layer+":"+s.Bridge+":"+s.Tunnel+":"+
        (s.Name.Trim().Length>0?"name:"+s.Name.Trim().ToUpperInvariant():s.Reference.Trim().Length>0?"ref:"+s.Reference.Trim().ToUpperInvariant():"way:"+s.WayId);
    private const string TrunkPairs="Divided trunk pairs consolidated";
    // Carriageways stay divided this far from an interchange ramp or a trunk
    // structure, so each merged stretch ends with room for its transition
    // (up to 200 m, below).
    private const float InterchangeClearance=200,StructureClearance=200,MinimumTrunkStretch=200;
    // A retained carriageway leaves its half of the merged end at no more than
    // this angle, then bends back onto its own alignment.
    private const float TransitionDegrees=8;

    /// <summary>
    /// Merge divided surface trunks along stretches with at-grade intersections.
    /// Interchange stretches keep their separate carriageways. A pair with more
    /// than two lanes on either carriageway becomes a six-lane street with a
    /// median (Large Road); otherwise it stays a four-lane two-way highway.
    /// </summary>
    public static IReadOnlyList<PreparedSegment> PrepareTrunks(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token)
    {
        while(true)
        {
            var before=log.Metric(TrunkPairs);
            source=PreparePass(source,log,token,trunk:true);
            if(log.Metric(TrunkPairs)==before)return source;
        }
    }

    // Points around which trunk carriageways are not merged, with their radii.
    // A link belongs to an interchange when its connected links touch a
    // motorway or a structure, or pass near a road structure; median openings
    // and at-grade slip lanes do not.
    private static List<(float2 Point,float Radius)> TrunkClearances(IReadOnlyList<PreparedSegment> source)
    {
        var roads=source.Where(s=>s.Kind==NetworkKind.Road).ToArray();
        var incident=roads.SelectMany(s=>new[]{(Id:s.StartNodeId,Section:s),(Id:s.EndNodeId,Section:s)})
            .GroupBy(p=>p.Id).ToDictionary(g=>g.Key,g=>g.Select(p=>p.Section).ToArray());
        var structures=roads.Where(s=>(s.Bridge||s.Tunnel)&&!s.ProtectedLink).SelectMany(s=>new[]{s.Start.xz,s.End.xz,math.lerp(s.Start.xz,s.End.xz,.5f)}).ToArray();
        var result=new List<(float2,float)>();
        var seen=new HashSet<PreparedSegment>();
        foreach(var seed in roads.Where(s=>s.ProtectedLink))
        {
            if(!seen.Add(seed))continue;
            var component=new List<PreparedSegment>{seed};
            for(var k=0;k<component.Count;k++)foreach(var node in new[]{component[k].StartNodeId,component[k].EndNodeId})
                foreach(var next in incident[node])if(next.ProtectedLink&&seen.Add(next))component.Add(next);
            var nodes=component.SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}).Distinct().ToArray();
            var interchange=component.Any(s=>s.Classification.StartsWith("motorway",StringComparison.Ordinal)||s.Bridge||s.Tunnel)||
                nodes.Any(n=>incident[n].Any(s=>s.Classification=="motorway"))||
                nodes.Select(n=>incident[n][0].StartNodeId==n?incident[n][0].Start.xz:incident[n][0].End.xz)
                    .Any(p=>structures.Any(q=>math.distancesq(p,q)<=100*100));
            if(!interchange)continue;
            foreach(var n in nodes)if(incident[n].Any(s=>s.Classification=="trunk"))
                result.Add((incident[n][0].StartNodeId==n?incident[n][0].Start.xz:incident[n][0].End.xz,InterchangeClearance));
        }
        foreach(var s in roads.Where(s=>s.Classification=="trunk"&&(s.Bridge||s.Tunnel)))
        {result.Add((s.Start.xz,StructureClearance));result.Add((s.End.xz,StructureClearance));}
        return result;
    }

    // A retained carriageway's end node moved onto the merged centreline, but
    // its lanes continue the merged road's lanes for its own direction, beside
    // the median. Bring the carriageway onto that half: its vertices move by
    // (T-C)·(1-d/L)², where C is its original end, T the point beside the merged
    // end at the half's lane centre, and d the distance from the end. It leaves
    // the half at no more than TransitionDegrees and meets its own alignment
    // tangentially at d=L, with vertices at most 10 m apart for curve fitting.
    // Only the first piece reaches the shared node; NetworkLaneSlots moves that
    // end onto the half. Any junction passed on the way moves with it; the walk
    // stops at links, structures and other moved ends.
    private static void Transitions(List<PreparedSegment> sections,List<(int Index,bool AtStart,float3 Delta)> shifted,Func<long> newNode,ImportLog log,CancellationToken token)
    {
        const float spacing=10,laneSpacing=4;
        var incident=new Dictionary<long,List<int>>();
        for(var i=0;i<sections.Count;i++)if(sections[i].Kind==NetworkKind.Road)foreach(var id in new[]{sections[i].StartNodeId,sections[i].EndNodeId})
        {if(!incident.TryGetValue(id,out var list))incident[id]=list=new();list.Add(i);}
        var ends=new HashSet<long>(shifted.Select(p=>p.AtStart?sections[p.Index].StartNodeId:sections[p.Index].EndNodeId));
        var replaced=new Dictionary<int,PreparedSegment[]>();var moved=new Dictionary<long,float3>();var shortTransitions=0;
        static bool Carriageway(PreparedSegment s)=>s.Kind==NetworkKind.Road&&s.OneWay&&s.Classification=="trunk"&&!s.Bridge&&!s.Tunnel;
        foreach(var (index,atStart,delta) in shifted)
        {
            token.ThrowIfCancellationRequested();
            var offset=math.length(delta.xz);if(offset<1||replaced.ContainsKey(index))continue;
            var node=atStart?sections[index].StartNodeId:sections[index].EndNodeId;
            // Two-way lanes are laneSpacing apart about the axis, so one
            // direction's half of an n-lane road is centred n·laneSpacing/4 out.
            var merged=incident[node].Select(j=>sections[j]).FirstOrDefault(s=>!s.OneWay&&s.ConsolidatedAvenue);
            var side=merged==null?0:laneSpacing*(merged.Lanes/2)/2f;
            var target=delta*(1-side/offset);if(math.length(target.xz)<1)continue;
            var reach=math.clamp(2*math.length(target.xz)/math.tan(math.radians(TransitionDegrees)),50,200);
            // Sections from the moved end outward; Away: the section's start is the near node.
            var run=new List<(int Index,bool Away)>();
            var current=index;var length=0f;var forward=sections[index].StartNodeId==node;var stop="";var heading=float2.zero;
            while(true)
            {
                var s=sections[current];var away=s.StartNodeId==node;run.Add((current,away));length+=math.distance(s.Start.xz,s.End.xz);
                // Slicing can leave a sub-metre piece with no direction of its own.
                if(math.distance(s.Start.xz,s.End.xz)>=1)heading=math.normalize((s.End-s.Start).xz);
                var far=away?s.EndNodeId:s.StartNodeId;
                stop=length>=reach?"":ends.Contains(far)?"merged-end":incident[far].Any(j=>sections[j].ProtectedLink)?"link":
                    incident[far].Any(j=>NetworkStructure.IsStructure(sections[j]))?"structure":"";
                if(length>=reach||stop.Length>0)break;
                // A one-way carriageway continues in its own travel direction.
                var next=incident[far].Where(j=>j!=current&&!replaced.ContainsKey(j)&&Carriageway(sections[j])&&(sections[j].StartNodeId==far)==forward&&
                    (heading.Equals(float2.zero)||math.dot(heading,math.normalizesafe((sections[j].End-sections[j].Start).xz))>.7f)).ToArray();
                if(next.Length!=1){stop="branch";break;}
                current=next[0];node=far;
            }
            if(length<reach)
            {
                var end=atStart?sections[index].Start:sections[index].End;
                log.Diagnostic($"TRUNK AVENUE TRANSITION limited way={sections[index].WayId}; at={end.x:0},{end.z:0}; offset={offset:0.0}; needed={reach:0}; available={length:0}; stop={stop}");
                reach=length;shortTransitions++;
            }
            var d=0f;
            foreach(var (j,away) in run)
            {
                var s=sections[j];
                var near=away?s.Start:s.End;var far=away?s.End:s.Start;var farNode=away?s.EndNodeId:s.StartNodeId;
                var node0=near;if(j==index)near-=delta;
                var span=math.distance(near.xz,far.xz);
                float3 Shifted(float3 p,float t){var w=t>=reach?0:(1-t/reach)*(1-t/reach);return p+target*w;}
                var points=new List<float3>{j==index?node0:Shifted(near,d)};
                for(var t=spacing;t<span&&d+t<reach;t+=spacing)points.Add(Shifted(math.lerp(near,far,t/span),d+t));
                points.Add(Shifted(far,d+span));
                if(d+span<reach)moved[farNode]=points[^1];
                var ids=new long[points.Count];ids[0]=away?s.StartNodeId:s.EndNodeId;ids[^1]=farNode;
                for(var k=1;k<ids.Length-1;k++)ids[k]=newNode();
                if(!away){points.Reverse();Array.Reverse(ids);}
                replaced[j]=Enumerable.Range(1,points.Count-1).Select(k=>s with{Start=points[k-1],End=points[k],StartNodeId=ids[k-1],EndNodeId=ids[k],
                    SourceGeometry=new[]{points[k-1],points[k]},Control1=null,Control2=null,HorizontalParent=null,
                    StartTerminal=k==1&&s.StartTerminal,EndTerminal=k==points.Count-1&&s.EndTerminal,
                    StartRoundaboutRadius=k==1?s.StartRoundaboutRadius:0,EndRoundaboutRadius=k==points.Count-1?s.EndRoundaboutRadius:0}).ToArray();
                d+=span;
            }
        }
        if(replaced.Count==0)return;
        var output=new List<PreparedSegment>(sections.Count+replaced.Values.Sum(p=>p.Length));
        for(var i=0;i<sections.Count;i++)
        {
            if(replaced.TryGetValue(i,out var pieces)){output.AddRange(pieces);continue;}
            var s=sections[i];
            if(moved.TryGetValue(s.StartNodeId,out var p))s=s with{Start=p,SourceGeometry=new[]{p}.Concat(s.SourceGeometry.Skip(1)).ToArray()};
            if(moved.TryGetValue(s.EndNodeId,out var q))s=s with{End=q,SourceGeometry=s.SourceGeometry.Take(Math.Max(1,s.SourceGeometry.Count-1)).Concat(new[]{q}).ToArray()};
            output.Add(s);
        }
        sections.Clear();sections.AddRange(output);
        log.Count("Divided trunk carriageway sections reshaped",replaced.Count);
        log.Count("Divided trunk transitions limited by nearby junctions or structures",shortTransitions);
    }

    private static float DistanceToSegment(float2 p,float2 a,float2 b)
    {
        var d=b-a;var t=math.clamp(math.dot(p-a,d)/Math.Max(.001f,math.lengthsq(d)),0,1);
        return math.distance(p,a+t*d);
    }

    public static IReadOnlyList<PreparedSegment> Prepare(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token)
    {
        source=StreetIntersectionConsolidator.Prepare(source,log,token);
        // One long carriageway may face several disconnected opposing portions.
        // Revisit retained tails after each pass; every successful pass consumes
        // one-way source edges, so a completed pair cannot be merged again.
        while(true)
        {
            var before=log.Metric("Opposing road pairs consolidated into avenues");
            source=PreparePass(source,log,token);
            if(log.Metric("Opposing road pairs consolidated into avenues")==before)break;
        }
        // Finish locally: an imperfect bend or unequal intersection approach
        // must not veto the otherwise parallel portions of a divided street.
        // Geometry, rather than naming, owns these remaining short matches.
        while(true)
        {
            var before=log.Metric("Opposing road pairs consolidated into avenues");
            var junctions=log.Metric("Avenue median junction connectors canonicalized");
            source=PreparePass(source,log,token,localMatching:true);
            if(log.Metric("Opposing road pairs consolidated into avenues")==before&&
                log.Metric("Avenue median junction connectors canonicalized")==junctions)break;
        }
        return source.Select(s=>s.ConsolidatedAvenue&&!s.OneWay&&s.Lanes<=2?s with{ConsolidatedAvenue=false}:s).ToArray();
    }

    private static IReadOnlyList<PreparedSegment> PreparePass(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token,bool localMatching=false,bool trunk=false)
    {
        token.ThrowIfCancellationRequested(); var clock=Stopwatch.StartNew();
        // Merging trunks moves their own junctions; motorways stay fixed.
        bool Fixed(PreparedSegment s)=>s.Kind==NetworkKind.Road&&(s.Classification=="motorway"||!trunk&&s.Classification=="trunk"||localMatching&&NativeAssetKeys.IsHighway(s));
        var fixedPorts=new HashSet<long>(source.Where(Fixed).SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}));
        var clearances=trunk?TrunkClearances(source):new List<(float2 Point,float Radius)>();
        bool Clear(PreparedSegment s)=>clearances.All(c=>DistanceToSegment(c.Point,s.Start.xz,s.End.xz)>c.Radius);
        var structureNodes=new HashSet<long>(source.Where(s=>s.Kind==NetworkKind.Rail).SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}));
        var railPorts=source.Where(s=>s.Kind==NetworkKind.Rail).SelectMany(s=>new[]{(Node:s.StartNodeId,Edge:s),(Node:s.EndNodeId,Edge:s)})
            .GroupBy(p=>p.Node).ToDictionary(g=>g.Key,g=>g.Select(p=>p.Edge).ToArray());
        bool RailPort(long node)
        {
            if(!localMatching||!railPorts.TryGetValue(node,out var edges)||edges.Length!=2||edges.Any(NetworkStructure.IsStructure)||edges[0].Layer!=edges[1].Layer)return false;
            if(edges[0].Classification!=edges[1].Classification||edges[0].PhysicalTracks!=edges[1].PhysicalTracks||edges[0].OneWay!=edges[1].OneWay||edges[0].ExplicitIncline!=edges[1].ExplicitIncline||
                edges.Any(s=>s.RailService.Length>0||(s.StartNodeId==node?s.StartTerminal:s.EndTerminal)))return false;
            if(edges[0].OneWay&&(edges[0].StartNodeId==node)==(edges[1].StartNodeId==node))return false;
            float2 Away(PreparedSegment s)=>math.normalizesafe((s.StartNodeId==node?s.End-s.Start:s.Start-s.End).xz);
            return math.dot(Away(edges[0]),Away(edges[1]))<-.995f;
        }
        var twoWayPorts=new HashSet<(long Node,string Identity)>(source.Where(s=>s.Kind==NetworkKind.Road&&!s.OneWay)
            .SelectMany(s=>new[]{(s.StartNodeId,Identity(s)),(s.EndNodeId,Identity(s))}));
        // Keep the actual structural tie-in edge, not an entire avenue just because
        // a distant end meets a bridge. The retained edge becomes the transition.
        var eligible=source.Where(s=>(trunk?TrunkEligible(s)&&Clear(s):Eligible(s))&&(!localMatching||!NetworkStructure.IsStructure(s))&&
            (!structureNodes.Contains(s.StartNodeId)||RailPort(s.StartNodeId))&&(!structureNodes.Contains(s.EndNodeId)||RailPort(s.EndNodeId)))
            .OrderBy(Identity,StringComparer.Ordinal).ThenBy(s=>s.WayId).ThenBy(s=>s.StartNodeId).ToArray();
        if(trunk)log.Diagnostic($"TRUNK AVENUE CLEARANCES points={clearances.Count}; carriageways={source.Count(TrunkEligible)}; clear={eligible.Length}");
        if(eligible.Length==0)return source;
        var chains=new List<Chain>();
        if(localMatching)foreach(var edge in eligible)
        {
            var length=math.distance(edge.Start.xz,edge.End.xz);
            if(math.isfinite(length)&&length>=8)chains.Add(new Chain{Edges=new[]{edge},Distances=new[]{0f,length}});
        }
        else foreach(var group in eligible.GroupBy(Identity))
        {
            token.ThrowIfCancellationRequested(); var edges=group.ToArray();
            var outgoing=edges.GroupBy(e=>e.StartNodeId).ToDictionary(g=>g.Key,g=>g.ToArray());
            var incoming=edges.GroupBy(e=>e.EndNodeId).ToDictionary(g=>g.Key,g=>g.ToArray());
            var visited=new HashSet<PreparedSegment>();
            foreach(var seed in edges.Where(e=>twoWayPorts.Contains((e.StartNodeId,group.Key))||!incoming.TryGetValue(e.StartNodeId,out var before)||before.Length!=1||outgoing[e.StartNodeId].Length!=1).Concat(edges))
            {
                if(visited.Contains(seed))continue;
                var list=new List<PreparedSegment>(); var current=seed;
                while(visited.Add(current))
                {
                    token.ThrowIfCancellationRequested(); list.Add(current);
                    if(twoWayPorts.Contains((current.EndNodeId,group.Key))||!outgoing.TryGetValue(current.EndNodeId,out var next))break;
                    var direction=math.normalizesafe(current.End.xz-current.Start.xz);
                    if(next.Length==1&&incoming[current.EndNodeId].Length==1)
                    {
                        if(math.dot(direction,math.normalizesafe(next[0].End.xz-next[0].Start.xz))<.5f)break;
                        current=next[0];continue;
                    }
                    // Keep a clearly aligned carriageway as one matching
                    // corridor across side-street junctions. Junction stations
                    // remain in SourcesBetween/Stations and are remapped later.
                    var ranked=next.Where(e=>!visited.Contains(e)&&e.Bridge==current.Bridge&&e.Tunnel==current.Tunnel)
                        .Select(e=>(edge:e,dot:math.dot(direction,math.normalizesafe(e.End.xz-e.Start.xz))))
                        .OrderByDescending(e=>e.dot).ToArray();
                    if(ranked.Length==0||ranked[0].dot<.95f||ranked.Length>1&&ranked[0].dot-ranked[1].dot<.05f)break;
                    current=ranked[0].edge;
                }
                if(list.Count==0||list[0].StartNodeId==list[list.Count-1].EndNodeId)continue;
                var distances=new float[list.Count+1];
                for(var i=0;i<list.Count;i++)distances[i+1]=distances[i]+math.distance(list[i].Start.xz,list[i].End.xz);
                if(distances[distances.Length-1]>=20&&distances.All(math.isfinite))chains.Add(new Chain{Edges=list.ToArray(),Distances=distances});
            }
        }
        // Search near the opposite chain's end, not all pairs or only their midpoints.
        var grid=new Dictionary<(int,int),List<int>>();
        for(var i=0;i<chains.Count;i++)
        {
            var steps=Math.Max(1,(int)Math.Ceiling(chains[i].Length/48));
            for(var k=0;k<=steps;k++){var point=chains[i].Point(k/(float)steps);var key=((int)Math.Floor(point.x/64),(int)Math.Floor(point.z/64));
                if(!grid.TryGetValue(key,out var bucket))grid[key]=bucket=new List<int>();if(!bucket.Contains(i))bucket.Add(i);}
        }
        var matches=new List<(int other,float score,float a0,float a1,float b0,float b1,bool tapered)>[chains.Count];
        var comparisons=0;var nextProgress=5d;var rejectedSamples=0;
        var rejections=new Dictionary<string,int>();
        void Reject(Chain a,Chain b,string reason,float value=0)
        {
            rejections[reason]=rejections.TryGetValue(reason,out var count)?count+1:1;
            if(rejectedSamples++<60)log.Diagnostic($"AVENUE REJECT name={a.Edges[0].Name}; sourceWay={a.Edges[0].WayId}; candidateWay={b.Edges[0].WayId}; reason={reason}; value={value:0.###}; lengths={a.Length:0.##}/{b.Length:0.##}");
        }
        for(var i=0;i<chains.Count;i++)
        {
            token.ThrowIfCancellationRequested(); matches[i]=new(); var a=chains[i];
            if(clock.Elapsed.TotalSeconds>=nextProgress){log.Diagnostic($"PROCESS avenue-matching chains={i}/{chains.Count}; comparisons={comparisons}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.0}");nextProgress=clock.Elapsed.TotalSeconds+5;}
            var nearby=new HashSet<int>();var searchSteps=Math.Max(1,(int)Math.Ceiling(a.Length/48));
            for(var k=0;k<=searchSteps;k++)
            {
                var point=a.Point(k/(float)searchSteps);var x=(int)Math.Floor(point.x/64);var z=(int)Math.Floor(point.z/64);
                for(var dz=-1;dz<=1;dz++)for(var dx=-1;dx<=1;dx++)if(grid.TryGetValue((x+dx,z+dz),out var bucket))nearby.UnionWith(bucket);
            }
            foreach(var j in nearby.OrderBy(v=>v))
                {
                    if(i==j)continue; var b=chains[j]; comparisons++;
                    var first=a.Edges[0];var second=b.Edges[0];
                    var nameA=first.Name.Trim();var nameB=second.Name.Trim();
                    var sameIdentity=first.Layer==second.Layer&&(nameA.Length>0&&nameB.Length>0
                        ?nameA.Equals(nameB,StringComparison.OrdinalIgnoreCase)
                        :first.Reference.Trim().Length>0&&first.Reference.Trim().Equals(second.Reference.Trim(),StringComparison.OrdinalIgnoreCase));
                    if(first.Layer!=second.Layer||!sameIdentity&&!localMatching)continue;
                    if(a.Edges[0].Bridge!=b.Edges[0].Bridge||a.Edges[0].Tunnel!=b.Edges[0].Tunnel)continue;
                    var a0=a.Project(b.End);var a1=a.Project(b.Start);var b0=b.Project(a.End);var b1=b.Project(a.Start);
                    if(localMatching)
                    {
                        // Both straight portions use A's longitudinal axis.
                        // Alternating perpendicular projections trims the ends
                        // twice on angled approaches and leaves parallel tails.
                        var axis=math.normalizesafe((a.End-a.Start).xz);
                        var start=math.dot((b.Start-a.Start).xz,axis);var end=math.dot((b.End-a.Start).xz,axis);
                        var lo=Math.Max(0,Math.Min(start,end));var hi=Math.Min(a.Length,Math.Max(start,end));
                        if(hi<=lo||Math.Abs(end-start)<.001f)continue;
                        a0=lo/a.Length;a1=hi/a.Length;
                        b0=math.clamp((hi-start)/(end-start),0,1);b1=math.clamp((lo-start)/(end-start),0,1);
                    }
                    else
                    {
                        // Clip the partner to the common interval, not to the ends
                        // of a longer curved road outside that interval. Then bring
                        // the first interval back onto those clipped partner ends.
                        // If B's endpoint projects inside A, B is already covered
                        // to that endpoint. Projecting its perpendicular foot back
                        // again shaves off a tiny tail on non-identical tangents;
                        // moving that tail to the median then creates a sharp stub.
                        b0=a1<1?0:b.Project(a.Point(a1));b1=a0>0?1:b.Project(a.Point(a0));
                        if(b1<1)a0=a.Project(b.Point(b1));if(b0>0)a1=a.Project(b.Point(b0));
                    }
                    // Keep actual endpoints when projection only shaved a
                    // sub-metre tail, preserving their junction ownership.
                    if(a0*a.Length<1)a0=0;if((1-a1)*a.Length<1)a1=1;
                    if(b0*b.Length<1)b0=0;if((1-b1)*b.Length<1)b1=1;
                    // Distinct ports on a major road keep their short turning
                    // approaches. Merge the divided body behind them instead
                    // of moving the major road or rejecting the entire street.
                    const float transition=24;
                    if(a0==0&&b1==1&&a.Edges[0].StartNodeId!=b.Edges[^1].EndNodeId&&
                        fixedPorts.Contains(a.Edges[0].StartNodeId)&&fixedPorts.Contains(b.Edges[^1].EndNodeId))
                    {a0=transition/a.Length;b1=1-transition/b.Length;}
                    if(a1==1&&b0==0&&a.Edges[^1].EndNodeId!=b.Edges[0].StartNodeId&&
                        fixedPorts.Contains(a.Edges[^1].EndNodeId)&&fixedPorts.Contains(b.Edges[0].StartNodeId))
                    {a1=1-transition/a.Length;b0=transition/b.Length;}
                    var overlap=(a1-a0)*a.Length;var reverseOverlap=(b1-b0)*b.Length;
                    // A short merged trunk between two divided stretches is two
                    // transitions with no avenue between them.
                    var minimumOverlap=localMatching?8:trunk?MinimumTrunkStretch:30;
                    if(overlap<minimumOverlap||reverseOverlap<minimumOverlap||overlap/reverseOverlap<.8f||overlap/reverseOverlap>1.25f){Reject(a,b,"overlap",overlap);continue;}
                    var valid=true;var separation=0f;var sign=0f;var tapered=false;
                    var samples=Math.Max(32,(int)Math.Ceiling(Math.Max(overlap,reverseOverlap)/24));
                    for(var sample=0;sample<=samples;sample++)
                    {
                        token.ThrowIfCancellationRequested(); var fraction=sample/(float)samples;var t=math.lerp(a0,a1,fraction);var pa=a.Point(t);
                        var u=localMatching?math.lerp(b1,b0,fraction):math.clamp(b.Project(pa),b0,b1);var pb=b.Point(u);
                        var distance=math.distance(pa.xz,pb.xz); if(distance>(localMatching?40:32)){Reject(a,b,"separation",distance);valid=false;break;} separation+=distance;
                        var tangent=math.normalizesafe(a.Point(Math.Min(1,t+.01f)).xz-a.Point(Math.Max(0,t-.01f)).xz);
                        var reverse=math.normalizesafe(b.Point(Math.Max(0,u-.01f)).xz-b.Point(Math.Min(1,u+.01f)).xz);
                        // At a shared split/join the carriageways converge, so
                        // their endpoint tangents need not be parallel. Preserve
                        // that exact connection while testing the divided body.
                        var sharedStart=a.Edges[0].StartNodeId==b.Edges[b.Edges.Length-1].EndNodeId;
                        var sharedEnd=a.Edges[a.Edges.Length-1].EndNodeId==b.Edges[0].StartNodeId;
                        var taper=sharedStart&&t*a.Length<32&&(1-u)*b.Length<32||
                            sharedEnd&&(1-t)*a.Length<32&&u*b.Length<32;
                        var endpoint=Math.Min(fraction,1-fraction)*Math.Min(overlap,reverseOverlap)<Math.Min(32,.2f*Math.Min(overlap,reverseOverlap));
                        var alignment=math.dot(tangent,reverse);
                        if(!taper&&alignment<(localMatching?.92f:endpoint ? .82f : .97f)){Reject(a,b,"tangent",alignment);valid=false;break;}
                        if(!taper&&alignment<.97f)tapered=true;
                        var delta=pb.xz-pa.xz;var side=tangent.x*delta.y-tangent.y*delta.x;
                        if(Math.Abs(side)>1){if(sign!=0&&Math.Sign(side)!=Math.Sign(sign)){valid=false;break;}sign=side;}
                    }
                    separation/=samples+1;
                    var unnamed=a.Edges[0].Name.Length==0&&a.Edges[0].Reference.Length==0;
                    // Rank coverage as well as separation. A nearby tiny fragment
                    // must not displace a slightly farther full corridor match.
                    var coverage=Math.Min(overlap/a.Length,reverseOverlap/b.Length);
                    if(valid&&(localMatching||separation>=2)&&(localMatching||!unnamed||overlap>=120&&separation<=18))matches[i].Add((j,separation+12*(1-coverage),a0,a1,b0,b1,tapered));
                }
            // A newly tolerated end taper must not displace an existing fully
            // parallel counterpart merely because its partial interval is closer.
            matches[i]=matches[i].OrderBy(m=>m.tapered).ThenBy(m=>m.score).ThenBy(m=>m.other).ToList();
        }
        bool ClearBest(int i)
        {
            if(matches[i].Count==0)return false;var best=matches[i][0];
            // Disconnected opposing portions occupy different intervals of a
            // long street; they are not competing explanations for one median.
            return localMatching||matches[i].Skip(1).All(m=>m.tapered!=best.tapered||Math.Min(m.a1,best.a1)-Math.Max(m.a0,best.a0)<=.00001f||m.score-best.score>=3);
        }
        var removed=new HashSet<PreparedSegment>(); var remap=new Dictionary<long,(long node,float3 point)>();
        var created=new List<PreparedSegment>();var tails=new List<PreparedSegment>(); var used=new HashSet<int>(); var pairs=0;
        var createdStations=new List<(int Pair,int Index)>();var stations=new Dictionary<long,(int Pair,int Index)>();
        var incident=source.Where(s=>s.Kind==NetworkKind.Road).SelectMany(s=>new[]{(Id:s.StartNodeId,Section:s),(Id:s.EndNodeId,Section:s)})
            .GroupBy(p=>p.Id).ToDictionary(g=>g.Key,g=>g.Select(p=>p.Section).ToArray());
        var degree=incident.ToDictionary(p=>p.Key,p=>p.Value.Length);
        var junctionAnchors=new Dictionary<long,float3>();
        // Keep mainline junctions fixed. Ramp ends still follow the ordinary
        // road's canonical junction, as other attached arms do; fixing both
        // sides of a divided street at separate ramp ports would prevent its
        // median from having a single junction owner.
        foreach(var s in source.Where(s=>Fixed(s)||s.Kind==NetworkKind.Road&&s.Roundabout))
        {junctionAnchors[s.StartNodeId]=s.Start;junctionAnchors[s.EndNodeId]=s.End;}
        // Retired junction IDs remain live provenance through aliases. Reusing
        // one on a later pass makes that new junction resolve to the old one.
        var ids=new HashSet<long>(source.SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}
            .Concat(s.SourceNodeAliases.SelectMany(a=>new[]{a.SourceNodeId,a.CanonicalNodeId}))));
        long nextNode=-8_000_000_000_000_000_000;long nextWay=-7_900_000_000_000_000_000;
        var wayIds=new HashSet<long>(source.Select(s=>s.WayId));
        long NewNode(){while(ids.Contains(nextNode))nextNode++;var n=nextNode++;ids.Add(n);return n;}
        Chain Slice(Chain chain,float begin,float end,List<PreparedSegment> retainedTails)
        {
            var list=new List<PreparedSegment>();var lo=begin*chain.Length;var hi=end*chain.Length;
            var startNode=begin==0?chain.Edges[0].StartNodeId:NewNode();var endNode=end==1?chain.Edges[chain.Edges.Length-1].EndNodeId:NewNode();
            for(var k=0;k<chain.Edges.Length;k++)
            {if(Math.Abs(chain.Distances[k]-lo)<.01f)startNode=chain.Edges[k].StartNodeId;if(Math.Abs(chain.Distances[k]-hi)<.01f)endNode=chain.Edges[k].StartNodeId;}
            for(var k=0;k<chain.Edges.Length;k++)
            {
                var e=chain.Edges[k];var a=chain.Distances[k];var b=chain.Distances[k+1];
                if(b<=lo||a>=hi){retainedTails.Add(e);continue;}
                var start=Math.Max(a,lo);var finish=Math.Min(b,hi);var p=math.lerp(e.Start,e.End,(start-a)/(b-a));var q=math.lerp(e.Start,e.End,(finish-a)/(b-a));
                var first=start==lo?startNode:e.StartNodeId;var last=finish==hi?endNode:e.EndNodeId;
                if(start>a)retainedTails.Add(e with{End=p,EndNodeId=first});
                list.Add(e with{Start=p,End=q,StartNodeId=first,EndNodeId=last});
                if(finish<b)retainedTails.Add(e with{Start=q,StartNodeId=last});
            }
            var distances=new float[list.Count+1];for(var k=0;k<list.Count;k++)distances[k+1]=distances[k]+math.distance(list[k].Start.xz,list[k].End.xz);
            return new Chain{Edges=list.ToArray(),Distances=distances};
        }
        var attempts=Enumerable.Range(0,chains.Count).SelectMany(i=>matches[i].Select((match,rank)=>(Index:i,Rank:rank,Match:match)))
            .Where(a=>localMatching||a.Rank==0).OrderBy(a=>localMatching?a.Match.score:a.Index).ThenBy(a=>a.Index).ThenBy(a=>a.Match.other);
        foreach(var attempt in attempts)
        {
            token.ThrowIfCancellationRequested();var i=attempt.Index;if(used.Contains(i)||!localMatching&&!ClearBest(i))continue;
            var match=attempt.Match;var j=match.other;
            if(used.Contains(j)||(localMatching?!matches[j].Any(m=>m.other==i):!ClearBest(j)||matches[j][0].other!=i))continue;
            var originalA=chains[i];var originalB=chains[j];var pairTails=new List<PreparedSegment>();
            var a=Slice(originalA,match.a0,match.a1,pairTails);var b=Slice(originalB,match.b0,match.b1,pairTails);var layer=a.Edges[0].Layer;
            if(a.Edges.Concat(b.Edges).Any(e=>structureNodes.Contains(e.StartNodeId)&&!RailPort(e.StartNodeId)||structureNodes.Contains(e.EndNodeId)&&!RailPort(e.EndNodeId)))
            {log.Count("Avenue pairs retained at structural transitions");continue;}
            var large=false;
            if(trunk)
            {
                // Only a stretch with an at-grade intersection behaves as an
                // avenue; a divided trunk without one stays a divided highway.
                var body=new HashSet<PreparedSegment>(originalA.Edges.Concat(originalB.Edges));
                bool Crossing(long node)=>incident.TryGetValue(node,out var arms)&&arms.Any(s=>!body.Contains(s)&&!s.ProtectedLink);
                if(!a.Edges.Skip(1).Any(e=>Crossing(e.StartNodeId))&&!b.Edges.Skip(1).Any(e=>Crossing(e.StartNodeId)))
                {log.Count("Divided trunk stretches kept without intersections");Reject(originalA,originalB,"no-intersection");continue;}
                // Lane pockets at intersections do not decide the cross section.
                static int Predominant(Chain c)=>Enumerable.Range(0,c.Edges.Length).GroupBy(k=>c.Edges[k].Lanes)
                    .OrderByDescending(g=>g.Sum(k=>c.Distances[k+1]-c.Distances[k])).ThenByDescending(g=>g.Key).First().Key;
                large=Predominant(a)>2||Predominant(b)>2;
            }
            // Use one longitudinal station system. Equal fractions of unequal
            // carriageway lengths put opposite junctions at different places
            // on bends and pull the median sideways between those stations.
            float BAt(float t)=>t<=0?1:t>=1?0:localMatching?1-t:b.Project(a.Point(t));
            float AAt(float t)=>t<=0?1:t>=1?0:localMatching?1-t:a.Project(b.Point(t));
            var knots=new List<float>{0,1};
            void Stations(Chain chain,bool reverse)
            {for(var k=1;k<chain.Edges.Length;k++)if(degree.TryGetValue(chain.Edges[k].StartNodeId,out var d)&&d>2||chain.Edges[k-1].Lanes!=chain.Edges[k].Lanes){var t=chain.Distances[k]/chain.Length;knots.Add(reverse?AAt(t):t);}}
            Stations(a,false);Stations(b,true);
            var steps=Math.Max(1,(int)Math.Ceiling(Math.Max(a.Length,b.Length)/30));
            var fractions=knots.OrderBy(t=>t).Aggregate(new List<float>(),(list,t)=>{if(list.Count==0||t-list[list.Count-1]>.00001f)list.Add(t);return list;});
            // Opposing OSM node grids rarely line up. Coalesce near-identical
            // stations BEFORE remapping junctions instead of rejecting the avenue.
            for(var k=fractions.Count-2;k>0;k--)
                if((fractions[k]-fractions[k-1])*Math.Min(a.Length,b.Length)<12||
                   (fractions[k+1]-fractions[k])*Math.Min(a.Length,b.Length)<12)fractions.RemoveAt(k);
            // Sampling stations must not displace real side-street junctions.
            // Add regular shape samples only after the protected stations have
            // been reconciled, leaving at least 12 m to either neighbouring pin.
            for(var k=1;k<steps;k++)
            {var t=k/(float)steps;if(fractions.All(f=>Math.Abs(f-t)*Math.Min(a.Length,b.Length)>=12))fractions.Add(t);}
            fractions.Sort();
            var positions=fractions.Select(t=>(a.Point(t)+b.Point(BAt(t)))*.5f).ToArray();
            var nodes=positions.Select(_=>NewNode()).ToArray();
            var anchored=new Dictionary<int,(long Node,float3 Point)>();
            var local=new Dictionary<long,(long node,float3 point)>();var localStations=new Dictionary<long,int>();var conflict=false;
            void Map(Chain chain,bool reversed)
            {
                for(var k=0;k<=chain.Edges.Length;k++)
                {
                    var original=k==chain.Edges.Length?chain.Edges[k-1].EndNodeId:chain.Edges[k].StartNodeId;
                    var t=chain.Distances[k]/chain.Length;if(reversed)t=AAt(t);
                    var closest=0;for(var n=1;n<fractions.Count;n++)if(Math.Abs(fractions[n]-t)<Math.Abs(fractions[closest]-t))closest=n;
                    if(junctionAnchors.TryGetValue(original,out var anchor))
                    {
                        if(anchored.TryGetValue(closest,out var priorAnchor)&&math.distance(priorAnchor.Point.xz,anchor.xz)>.1f)
                        {conflict=true;return;}
                        anchored[closest]=(original,anchor);positions[closest]=anchor;nodes[closest]=original;
                    }
                    var key=original;var value=(node:nodes[closest],point:positions[closest]);
                    if(remap.TryGetValue(key,out var old))
                    {
                        if(localMatching)
                        {
                            // A second avenue meeting the same source junction
                            // inherits its established centre, rather than
                            // demanding a second independently averaged node.
                            if(math.distance(old.point.xz,value.point.xz)>32||anchored.TryGetValue(closest,out var owner)&&owner.Node!=old.node)
                            {conflict=true;return;}
                            anchored[closest]=(old.node,old.point);positions[closest]=old.point;nodes[closest]=old.node;value=(old.node,old.point);
                        }
                        else
                        {if(math.distance(old.point.xz,value.point.xz)>.1f){conflict=true;return;}nodes[closest]=old.node;value.node=old.node;}
                    }
                    if(local.TryGetValue(key,out var prior)&&math.distance(prior.point.xz,value.point.xz)>.1f){conflict=true;return;}
                    local[key]=value;
                    localStations[key]=closest;
                }
            }
            Map(a,false);Map(b,true);
            if(conflict){log.Count("Avenue pairs retained for junction conflicts");Reject(originalA,originalB,"junction-conflict");continue;}
            if(trunk)
            {
                // Turn pockets and offset junction nodes on the two mapped
                // carriageways make their midpoint wander sideways. Relax each
                // free station toward the chord of its neighbours; this removes
                // wander shorter than about 100 m and keeps the corridor's
                // bends. Ends, anchors and junctions shared with an earlier
                // merge stay where they are.
                var pinned=new HashSet<int>(anchored.Keys){0,positions.Length-1};
                foreach(var key in local.Keys)if(remap.ContainsKey(key))pinned.Add(localStations[key]);
                for(var pass=0;pass<12;pass++)for(var k=1;k<positions.Length-1;k++)if(!pinned.Contains(k))
                {
                    var p=positions[k-1];var d=positions[k+1]-p;
                    var t=math.clamp(math.dot((positions[k]-p).xz,d.xz)/Math.Max(.001f,math.lengthsq(d.xz)),0,1);
                    positions[k]=math.lerp(positions[k],p+t*d,.5f);
                }
            }
            // A later arm can reuse an existing canonical node at a station.
            // Every earlier source alias at that station follows the final owner.
            foreach(var key in local.Keys.ToArray())
            {var station=localStations[key];local[key]=(nodes[station],positions[station]);}
            // Plain track nodes between two carriageway crossings are part of
            // the same crossing. Retire those interior stations too, otherwise
            // moving only the outer ports would reverse a short track edge.
            foreach(var group in local.Where(p=>structureNodes.Contains(p.Key)).GroupBy(p=>p.Value.node).Where(g=>g.Count()>1).ToArray())
            {
                var gates=new HashSet<long>(group.Select(p=>p.Key));var first=group.First();
                foreach(var gate in gates)foreach(var seed in railPorts[gate])
                {
                    var path=new List<long>();var seen=new HashSet<long>{gate};var edge=seed;var current=gate;var distance=0f;
                    while(true)
                    {
                        distance+=math.distance(edge.Start.xz,edge.End.xz);var nextRail=edge.StartNodeId==current?edge.EndNodeId:edge.StartNodeId;
                        if(distance>40||!seen.Add(nextRail))break;
                        if(gates.Contains(nextRail))
                        {
                            foreach(var node in path){local[node]=first.Value;localStations[node]=localStations[first.Key];}
                            break;
                        }
                        if(incident.ContainsKey(nextRail)||!RailPort(nextRail))break;
                        path.Add(nextRail);var following=railPorts[nextRail].First(s=>s!=edge);current=nextRail;edge=following;
                    }
                }
            }
            var railConflict=false;
            foreach(var entry in local.Where(p=>structureNodes.Contains(p.Key)))
            {
                var edge=railPorts[entry.Key][0];var p=edge.StartNodeId==entry.Key?edge.Start:edge.End;
                var railDirection=math.normalizesafe((edge.End-edge.Start).xz);var delta=entry.Value.point.xz-p.xz;
                if(edge.Layer!=layer||math.length(delta)>40||Math.Abs(railDirection.x*delta.y-railDirection.y*delta.x)>.5f){railConflict=true;break;}
            }
            foreach(var edge in local.Keys.Where(railPorts.ContainsKey).SelectMany(n=>railPorts[n]).Distinct())
            {
                var p=local.TryGetValue(edge.StartNodeId,out var start)?start.point:edge.Start;
                var q=local.TryGetValue(edge.EndNodeId,out var end)?end.point:edge.End;
                if(math.dot((q-p).xz,(edge.End-edge.Start).xz)<-.01f){railConflict=true;break;}
            }
            if(railConflict){log.Count("Avenue pairs retained for rail alignment conflicts");Reject(originalA,originalB,"rail-alignment");continue;}
            // A taper may explain non-parallel carriageway ends, but cannot
            // justify turning a previously sound side-street connection into
            // an acute junction when both ends move onto the median.
            if(match.tapered||localMatching)
            {
                var body=new HashSet<PreparedSegment>(originalA.Edges.Concat(originalB.Edges));var acute=false;
                double Angle(float2 u,float2 v)=>Math.Acos(math.clamp(math.dot(math.normalizesafe(u),math.normalizesafe(v)),-1,1))*180/Math.PI;
                foreach(var entry in local)
                {
                    if(!incident.TryGetValue(entry.Key,out var attached))continue;var station=localStations[entry.Key];var p=positions[station];
                    var directions=new List<float2>();if(station>0)directions.Add((positions[station-1]-p).xz);if(station+1<positions.Length)directions.Add((positions[station+1]-p).xz);
                    var external=new List<(float2 Before,float2 After)>();
                    foreach(var arm in attached.Where(s=>!body.Contains(s)))
                    {
                        var start=arm.StartNodeId==entry.Key;var old=(start?arm.End-arm.Start:arm.Start-arm.End).xz;
                        var far=start?arm.EndNodeId:arm.StartNodeId;var q=local.TryGetValue(far,out var moved)?moved.point:remap.TryGetValue(far,out moved)?moved.point:start?arm.End:arm.Start;
                        var next=(q-p).xz;if(math.lengthsq(next)<.01f)continue;
                        external.Add((old,next));
                        var prior=attached.Where(body.Contains).Select(s=>Angle(old,(s.StartNodeId==entry.Key?s.End-s.Start:s.Start-s.End).xz)).DefaultIfEmpty(0).Min();
                        if(prior>=25&&directions.Min(d=>Angle(next,d))<25){acute=true;break;}
                    }
                    // Moving one shared anchor also changes the angle between
                    // two attached arms, even when neither is the matched body.
                    for(var x=0;x<external.Count&&!acute;x++)for(var y=x+1;y<external.Count;y++)
                        if(Angle(external[x].Before,external[y].Before)>=25&&Angle(external[x].After,external[y].After)<25){acute=true;break;}
                    if(acute)break;
                }
                if(acute){log.Count("Avenue taper matches retained for acute attachments");Reject(originalA,originalB,"acute-attachment");continue;}
            }
            var mappedEvidence=Enumerable.Range(1,positions.Length).Select(_=>new List<PreparedSegment>()).ToArray();
            foreach(var edge in a.Edges.Concat(b.Edges))
            {
                var first=localStations[edge.StartNodeId];var last=localStations[edge.EndNodeId];
                for(var k=Math.Min(first,last);k<Math.Max(first,last);k++)mappedEvidence[k].Add(edge);
            }
            long Mapped(long id)=>local.TryGetValue(id,out var mapped)?mapped.node:remap.TryGetValue(id,out mapped)?mapped.node:id;
            if(local.Keys.Where(incident.ContainsKey).SelectMany(id=>incident[id]).Distinct()
                .Any(s=>(s.Bridge||s.Tunnel)&&Mapped(s.StartNodeId)==Mapped(s.EndNodeId)))
            {log.Count("Avenue pairs retained for collapsed structure connections");Reject(originalA,originalB,"collapsed-structure");continue;}
            // Reject unstable/tiny centerline pieces rather than removing either original carriageway.
            if(positions.Zip(positions.Skip(1),(p,q)=>math.distance(p.xz,q.xz)).Any(d=>d<.5f))
            {log.Count("Avenue pairs retained for tiny junction spacing");Reject(originalA,originalB,"tiny-junction-spacing");continue;}
            while(wayIds.Contains(nextWay))nextWay++;var way=nextWay++;wayIds.Add(way);
            var direction=math.normalizesafe((a.End-a.Start).xz);var median=(b.Point(BAt(.5f))-a.Point(.5f)).xz;
            var bOnLeft=direction.x*median.y-direction.y*median.x>0;var outside=bOnLeft?NetworkSides.Right:NetworkSides.Left;
            NetworkSides Outer(PreparedSegment[] sideA,PreparedSegment[] sideB,Func<PreparedSegment,NetworkSides> selector)
            {
                var aWall=sideA.Any(s=>(selector(s)&outside)!=0);var bWall=sideB.Any(s=>(selector(s)&outside)!=0);
                return (aWall?(bOnLeft?NetworkSides.Right:NetworkSides.Left):0)|(bWall?(bOnLeft?NetworkSides.Left:NetworkSides.Right):0);
            }
            for(var k=1;k<positions.Length;k++)
            {
                var middle=(fractions[k-1]+fractions[k])*.5f;
                var sideA=a.SourcesBetween(fractions[k-1],fractions[k]);var sideB=b.SourcesBetween(BAt(fractions[k]),BAt(fractions[k-1]));
                // Coalescing nearby junction stations may move a source port
                // across a sample boundary. Its way owns the intervals between
                // its actual mapped ports as well as its geometric overlap.
                var observations=sideA.Concat(sideB).Concat(mappedEvidence[k-1]).Distinct().ToArray();
                int Rank(string classification)=>classification=="primary"?4:classification=="secondary"?3:classification=="tertiary"?2:1;
                var classification=trunk?large?"primary":"trunk":observations.OrderByDescending(s=>Rank(s.Classification)).First().Classification;
                var forward=a.Lanes(middle);var backward=b.Lanes(BAt(middle));var requested=forward+backward;
                var lanes=trunk?large?6:4:NetworkSelection.MatchedLanes(requested,false,false);
                var speeds=observations.Where(s=>s.SourceSpeedKph.HasValue).Select(s=>s.SourceSpeedKph!.Value).ToArray();
                // Each output interval owns only overlapping evidence. Assigning
                // every corridor way to every piece creates false native turns.
                createdStations.Add((pairs,k-1));
                created.Add(a.SourceAt(middle) with{WayId=way,StartNodeId=nodes[k-1],EndNodeId=nodes[k],Start=positions[k-1],End=positions[k],
                    SourceWayIds=observations.SelectMany(s=>s.SourceWayIds.Count>0?s.SourceWayIds:new[]{s.WayId}).Distinct().ToArray(),
                    OriginalGeometry=observations.SelectMany(s=>s.OriginalGeometry.Count>0?s.OriginalGeometry:new[]{s.Start,s.End}).ToArray(),
                    Classification=classification,OneWay=false,Lanes=lanes,ForwardLanes=lanes/2,BackwardLanes=lanes/2,BothWaysLanes=0,
                    SourceWidthMeters=null,SourceSpeedKph=speeds.Length>0?speeds.Min():null,
                    ExplicitLanes=observations.Any(s=>s.ExplicitLanes),ConsolidatedAvenue=true,CorridorId=0,MaximumNativeWidth=float.PositiveInfinity,
                    ExplicitCutSides=Outer(sideA,sideB,s=>s.ExplicitCutSides),ExplicitWallSides=Outer(sideA,sideB,s=>s.ExplicitWallSides),
                    SourceGeometry=new[]{positions[k-1],positions[k]},Control1=null,Control2=null,HorizontalParent=null,ExplicitIncline=null});
                if(requested>6||forward!=backward)log.Diagnostic($"AVENUE CROSS SECTION sourceWays={string.Join(",",observations.Select(s=>s.WayId).Distinct())}; requestedForward={forward}; requestedBackward={backward}; requestedTotal={requested}; supportedTotal={lanes}; directionalApproximationRequired={forward!=backward}; intervalEvidenceOnly=true");
            }
            foreach(var entry in local)remap[entry.Key]=entry.Value;
            for(var k=0;k<nodes.Length;k++)stations[nodes[k]]=(pairs,k);
            foreach(var edge in originalA.Edges.Concat(originalB.Edges))removed.Add(edge);tails.AddRange(pairTails);
            used.Add(i);used.Add(j);pairs++;
            log.Diagnostic($"{(trunk?"TRUNK AVENUE MERGE":"AVENUE MERGE")} name={a.Edges[0].Name}; reference={a.Edges[0].Reference}; sourceWays={string.Join(",",a.Edges.Concat(b.Edges).Select(e=>e.WayId).Distinct().OrderBy(id=>id))}; separationMeters={match.score:0.##}; lengthMeters={a.Length:0}; junctionNodesRemapped={local.Count}; outputSections={positions.Length-1}; vanillaPalette={(trunk?large?"Large-Road":"Highway-Twoway-4":"Medium-or-Large-Road")}; highwaysAndRampsNotConverted={!trunk}");
        }
        var result=new List<PreparedSegment>();var reconnected=0;
        var shifted=new List<(int Index,bool AtStart,float3 Delta)>();var folded=new List<(int Pair,int From,int To,IReadOnlyList<long> Ways)>();
        // An unmatched tail can be an unchanged original edge. Filter removed
        // carriageways before appending tails, otherwise value equality removes
        // those retained edges a second time and severs their exterior routes.
        foreach(var edge in source.Where(s=>!removed.Contains(s)).Concat(tails))
        {
            token.ThrowIfCancellationRequested();
            var s=edge;
            // One OSM road node denotes one connection, including a bridge or
            // tunnel portal whose layer differs from the approach. Remap every
            // incident road arm together; never leave a structural arm on the
            // replaced node. Different source node IDs remain independent.
            if(s.Kind==NetworkKind.Road||localMatching&&s.Kind==NetworkKind.Rail)
            {
                var carriageway=trunk&&s.Kind==NetworkKind.Road&&s.OneWay&&s.Classification=="trunk";
                if(remap.TryGetValue(s.StartNodeId,out var start))
                {if(carriageway)shifted.Add((result.Count,true,start.point-s.Start));s=s with{StartNodeId=start.node,Start=start.point};reconnected++;}
                if(remap.TryGetValue(s.EndNodeId,out var end))
                {if(carriageway)shifted.Add((result.Count,false,end.point-s.End));s=s with{EndNodeId=end.node,End=end.point};reconnected++;}
                if(s.StartNodeId==s.EndNodeId)
                { // A median crossover has become an internal connection at one avenue junction.
                    log.Count(s.Kind==NetworkKind.Rail?"Internal rail crossing spans consolidated":"Avenue median crossovers consolidated");
                    shifted.RemoveAll(p=>p.Index==result.Count);continue;
                }
                // A median opening or U-turn between two stations of one merged
                // trunk would retrace it; its route is carried by the avenue.
                if(trunk&&s.ProtectedLink&&stations.TryGetValue(s.StartNodeId,out var from)&&stations.TryGetValue(s.EndNodeId,out var to)&&from.Pair==to.Pair)
                {
                    folded.Add((from.Pair,Math.Min(from.Index,to.Index),Math.Max(from.Index,to.Index),s.SourceWayIds.Count>0?s.SourceWayIds:new[]{s.WayId}));
                    continue;
                }
                if(s!=edge)s=s with{SourceGeometry=new[]{s.Start,s.End},Control1=null,Control2=null,HorizontalParent=null};
            }
            result.Add(s);
        }
        for(var k=0;k<created.Count;k++)
        {
            var (pair,index)=createdStations[k];
            var ways=folded.Where(f=>f.Pair==pair&&f.From<=index&&index<f.To).SelectMany(f=>f.Ways).ToArray();
            if(ways.Length>0)created[k]=created[k] with{SourceWayIds=created[k].SourceWayIds.Concat(ways).Distinct().ToArray()};
        }
        result.AddRange(created);
        if(trunk)Transitions(result,shifted,NewNode,log,token);
        log.Count(trunk?TrunkPairs:"Opposing road pairs consolidated into avenues",pairs);log.Count("Avenue attachment endpoints reconnected",reconnected);
        if(trunk)log.Count("Median links folded into merged trunks",folded.Count);
        foreach(var i in Enumerable.Range(0,chains.Count).Where(i=>!used.Contains(i)&&matches[i].Count>0).Take(40))
            log.Diagnostic($"AVENUE UNRESOLVED opposingCandidate=true; sourceWay={chains[i].Edges[0].WayId}; candidateWay={chains[matches[i][0].other].Edges[0].WayId}; separationMeters={matches[i][0].score:0.###}; lengthMeters={chains[i].Length:0.###}; reason={(ClearBest(i)?"non-reciprocal-or-attachment-transition":"multiple-near-equal-candidates")}; noBlindDirectionConversion=true");
        log.Diagnostic($"AVENUE TOTAL eligibleSections={eligible.Length}; chains={chains.Count}; comparisons={comparisons}; pairsMerged={pairs}; sourceSectionsReplaced={removed.Count}; outputAvenueSections={created.Count}; attachmentEndpointsReconnected={reconnected}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.###}; ambiguousOrUnmatchedChainsPreserved={chains.Count-used.Count}; structuresPreserved=true");
        log.Diagnostic($"AVENUE REJECTIONS {string.Join("; ",rejections.OrderBy(p=>p.Key).Select(p=>p.Key+"="+p.Value))}; sampleLimit=60");
        var aliased=SourceTopologyObligations.Transfer(source,result,NetworkKind.Road,
            remap.Where(p=>incident.ContainsKey(p.Key)).ToDictionary(p=>p.Key,p=>p.Value.node));
        if(localMatching)aliased=SourceTopologyObligations.Transfer(source,aliased,NetworkKind.Rail,
            remap.Where(p=>structureNodes.Contains(p.Key)).ToDictionary(p=>p.Key,p=>p.Value.node));
        return AvenueJunctionRepair.Prepare(aliased,log,token,completeStreetJunctions:localMatching);
    }
}
