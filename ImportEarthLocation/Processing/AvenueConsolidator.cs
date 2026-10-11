using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

/// <summary>Replace confidently matched ordinary divided roads, preserving every attached source junction.</summary>
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
    private static bool Eligible(PreparedSegment s) => s.Kind==NetworkKind.Road&&s.OneWay&&!s.Bridge&&!s.Tunnel&&
        !s.Classification.EndsWith("_link",StringComparison.OrdinalIgnoreCase)&&
        (s.Classification=="primary"||s.Classification=="secondary"||s.Classification=="tertiary"||s.Classification=="residential"||s.Classification=="unclassified")&&
        !s.Roundabout&&s.Control1==null&&s.Control2==null;
    // Names often change at administrative/OSM splits. They do not delimit a
    // physical carriageway. Conflicting explicit references remain separate.
    private static string Identity(PreparedSegment s) => s.Classification+":"+s.Layer+":"+s.Reference.Trim().ToUpperInvariant();

    public static IReadOnlyList<PreparedSegment> Prepare(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token)
    {
        token.ThrowIfCancellationRequested(); var clock=Stopwatch.StartNew();
        var structureNodes=new HashSet<long>(source.Where(s=>s.Bridge||s.Tunnel||s.Kind==NetworkKind.Rail).SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}));
        // Keep the actual structural tie-in edge, not an entire avenue just because
        // a distant end meets a bridge. The retained edge becomes the transition.
        var eligible=source.Where(s=>Eligible(s)&&!structureNodes.Contains(s.StartNodeId)&&!structureNodes.Contains(s.EndNodeId)).OrderBy(Identity,StringComparer.Ordinal).ThenBy(s=>s.WayId).ThenBy(s=>s.StartNodeId).ToArray();
        if(eligible.Length==0)return source;
        var chains=new List<Chain>();
        foreach(var group in eligible.GroupBy(Identity))
        {
            token.ThrowIfCancellationRequested(); var edges=group.ToArray();
            var outgoing=edges.GroupBy(e=>e.StartNodeId).ToDictionary(g=>g.Key,g=>g.ToArray());
            var incoming=edges.GroupBy(e=>e.EndNodeId).ToDictionary(g=>g.Key,g=>g.ToArray());
            var visited=new HashSet<PreparedSegment>();
            foreach(var seed in edges.Where(e=>!incoming.TryGetValue(e.StartNodeId,out var before)||before.Length!=1||outgoing[e.StartNodeId].Length!=1))
            {
                if(visited.Contains(seed))continue;
                var list=new List<PreparedSegment>(); var current=seed;
                while(visited.Add(current))
                {
                    token.ThrowIfCancellationRequested(); list.Add(current);
                    if(!outgoing.TryGetValue(current.EndNodeId,out var next)||next.Length!=1||incoming[current.EndNodeId].Length!=1)break;
                    current=next[0];
                }
                if(list.Count==0||list[0].StartNodeId==list[list.Count-1].EndNodeId)continue;
                var distances=new float[list.Count+1];
                for(var i=0;i<list.Count;i++)distances[i+1]=distances[i]+math.distance(list[i].Start.xz,list[i].End.xz);
                if(distances[distances.Length-1]>=60&&distances.All(math.isfinite))chains.Add(new Chain{Edges=list.ToArray(),Distances=distances});
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
        var matches=new List<(int other,float score,float a0,float a1,float b0,float b1)>[chains.Count];
        var comparisons=0;var nextProgress=5d;
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
                    var sameIdentity=Identity(a.Edges[0])==Identity(b.Edges[0])&&
                        (a.Edges[0].Name.Equals(b.Edges[0].Name,StringComparison.OrdinalIgnoreCase)||
                         a.Edges[0].Reference.Length>0);
                    if(!sameIdentity&&(a.Edges[0].Classification!=b.Edges[0].Classification||a.Edges[0].Layer!=b.Edges[0].Layer||
                        a.Edges[0].Reference.Length>0&&b.Edges[0].Reference.Length>0&&a.Edges[0].Reference!=b.Edges[0].Reference))continue;
                    var direction=math.normalizesafe(a.End.xz-a.Start.xz);
                    if(math.dot(direction,math.normalizesafe(b.End.xz-b.Start.xz))>-.95f)continue;
                    var a0=a.Project(b.End);var a1=a.Project(b.Start);var b0=b.Project(a.End);var b1=b.Project(a.Start);
                    var overlap=(a1-a0)*a.Length;var reverseOverlap=(b1-b0)*b.Length;
                    if(overlap<60||reverseOverlap<60||overlap/reverseOverlap<.8f||overlap/reverseOverlap>1.25f)continue;
                    var valid=true;var separation=0f;var sign=0f;
                    var samples=Math.Max(32,(int)Math.Ceiling(Math.Max(overlap,reverseOverlap)/24));
                    for(var sample=0;sample<=samples;sample++)
                    {
                        token.ThrowIfCancellationRequested(); var fraction=sample/(float)samples;var t=math.lerp(a0,a1,fraction);var u=math.lerp(b1,b0,fraction); var pa=a.Point(t);var pb=b.Point(u);
                        var distance=math.distance(pa.xz,pb.xz); if(distance>40){valid=false;break;} separation+=distance;
                        var tangent=math.normalizesafe(a.Point(Math.Min(1,t+.01f)).xz-a.Point(Math.Max(0,t-.01f)).xz);
                        var reverse=math.normalizesafe(b.Point(Math.Max(0,u-.01f)).xz-b.Point(Math.Min(1,u+.01f)).xz);
                        if(math.dot(tangent,reverse)<.9f){valid=false;break;}
                        var delta=pb.xz-pa.xz;var side=tangent.x*delta.y-tangent.y*delta.x;
                        if(Math.Abs(side)>1){if(sign!=0&&Math.Sign(side)!=Math.Sign(sign)){valid=false;break;}sign=side;}
                    }
                    separation/=samples+1;
                    var unnamed=a.Edges[0].Name.Length==0&&a.Edges[0].Reference.Length==0;
                    // Different source names are not proof of different corridors.
                    // Require long, narrow, reciprocal geometry when name evidence differs.
                    if(valid&&separation>=2&&(!unnamed||overlap>=120&&separation<=18)&&(sameIdentity||overlap>=160&&separation<=18))matches[i].Add((j,separation,a0,a1,b0,b1));
                }
            matches[i]=matches[i].OrderBy(m=>m.score).ThenBy(m=>m.other).ToList();
        }
        bool ClearBest(int i) => matches[i].Count>0&&(matches[i].Count==1||matches[i][1].score-matches[i][0].score>=3);
        var removed=new HashSet<PreparedSegment>(); var remap=new Dictionary<(int,long),(long node,float3 point)>();
        var created=new List<PreparedSegment>();var tails=new List<PreparedSegment>(); var used=new HashSet<int>(); var pairs=0;
        var degree=source.Where(s=>s.Kind==NetworkKind.Road).SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}).GroupBy(id=>id).ToDictionary(g=>g.Key,g=>g.Count());
        var ids=new HashSet<long>(source.SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}));
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
        foreach(var i in Enumerable.Range(0,chains.Count))
        {
            token.ThrowIfCancellationRequested(); if(used.Contains(i)||!ClearBest(i))continue;
            var j=matches[i][0].other;if(used.Contains(j)||!ClearBest(j)||matches[j][0].other!=i)continue;
            var originalA=chains[i];var originalB=chains[j];var match=matches[i][0];var pairTails=new List<PreparedSegment>();
            var a=Slice(originalA,match.a0,match.a1,pairTails);var b=Slice(originalB,match.b0,match.b1,pairTails);var layer=a.Edges[0].Layer;
            if(a.Edges.Concat(b.Edges).Any(e=>structureNodes.Contains(e.StartNodeId)||structureNodes.Contains(e.EndNodeId)))
            {log.Count("Avenue pairs retained at structural transitions");continue;}
            var knots=new List<float>{0,1};
            void Stations(Chain chain,bool reverse)
            {for(var k=1;k<chain.Edges.Length;k++)if(degree.TryGetValue(chain.Edges[k].StartNodeId,out var d)&&d>2||chain.Edges[k-1].Lanes!=chain.Edges[k].Lanes){var t=chain.Distances[k]/chain.Length;knots.Add(reverse?1-t:t);}}
            Stations(a,false);Stations(b,true);
            var steps=Math.Max(1,(int)Math.Ceiling(Math.Max(a.Length,b.Length)/30));
            for(var k=1;k<steps;k++)knots.Add(k/(float)steps);
            var fractions=knots.OrderBy(t=>t).Aggregate(new List<float>(),(list,t)=>{if(list.Count==0||t-list[list.Count-1]>.00001f)list.Add(t);return list;});
            // Opposing OSM node grids rarely line up. Coalesce near-identical
            // stations BEFORE remapping junctions instead of rejecting the avenue.
            for(var k=fractions.Count-2;k>0;k--)
                if((fractions[k]-fractions[k-1])*Math.Min(a.Length,b.Length)<12||
                   (fractions[k+1]-fractions[k])*Math.Min(a.Length,b.Length)<12)fractions.RemoveAt(k);
            var positions=fractions.Select(t=>(a.Point(t)+b.Point(1-t))*.5f).ToArray();
            var nodes=positions.Select(_=>NewNode()).ToArray();
            var local=new Dictionary<(int,long),(long node,float3 point)>(); var conflict=false;
            void Map(Chain chain,bool reversed)
            {
                for(var k=0;k<=chain.Edges.Length;k++)
                {
                    var original=k==chain.Edges.Length?chain.Edges[k-1].EndNodeId:chain.Edges[k].StartNodeId;
                    var t=chain.Distances[k]/chain.Length;if(reversed)t=1-t;
                    var closest=0;for(var n=1;n<fractions.Count;n++)if(Math.Abs(fractions[n]-t)<Math.Abs(fractions[closest]-t))closest=n;
                    var key=(layer,original);var value=(node:nodes[closest],point:positions[closest]);
                    if(remap.TryGetValue(key,out var old))
                    { if(math.distance(old.point.xz,value.point.xz)>.1f){conflict=true;return;}nodes[closest]=old.node;value.node=old.node; }
                    if(local.TryGetValue(key,out var prior)&&math.distance(prior.point.xz,value.point.xz)>.1f){conflict=true;return;}
                    local[key]=value;
                }
            }
            Map(a,false);Map(b,true);
            if(conflict){log.Count("Avenue pairs retained for junction conflicts");continue;}
            // Reject unstable/tiny centerline pieces rather than removing either original carriageway.
            if(positions.Zip(positions.Skip(1),(p,q)=>math.distance(p.xz,q.xz)).Any(d=>d<.5f))
            {log.Count("Avenue pairs retained for tiny junction spacing");continue;}
            while(wayIds.Contains(nextWay))nextWay++;var way=nextWay++;wayIds.Add(way);
            var direction=math.normalizesafe((a.End-a.Start).xz);var median=(b.Point(.5f)-a.Point(.5f)).xz;
            var bOnLeft=direction.x*median.y-direction.y*median.x>0;var outside=bOnLeft?NetworkSides.Right:NetworkSides.Left;
            NetworkSides Outer(PreparedSegment[] sideA,PreparedSegment[] sideB,Func<PreparedSegment,NetworkSides> selector)
            {
                var aWall=sideA.Any(s=>(selector(s)&outside)!=0);var bWall=sideB.Any(s=>(selector(s)&outside)!=0);
                return (aWall?(bOnLeft?NetworkSides.Right:NetworkSides.Left):0)|(bWall?(bOnLeft?NetworkSides.Left:NetworkSides.Right):0);
            }
            for(var k=1;k<positions.Length;k++)
            {
                var middle=(fractions[k-1]+fractions[k])*.5f;
                var sideA=a.SourcesBetween(fractions[k-1],fractions[k]);var sideB=b.SourcesBetween(1-fractions[k],1-fractions[k-1]);
                var observations=sideA.Concat(sideB).ToArray();
                var forward=a.Lanes(middle);var backward=b.Lanes(1-middle);var requested=forward+backward;
                var lanes=requested<=4?4:6;
                var speeds=observations.Where(s=>s.SourceSpeedKph.HasValue).Select(s=>s.SourceSpeedKph!.Value).ToArray();
                // Each output interval owns only overlapping evidence. Assigning
                // every corridor way to every piece creates false native turns.
                created.Add(a.SourceAt(middle) with{WayId=way,StartNodeId=nodes[k-1],EndNodeId=nodes[k],Start=positions[k-1],End=positions[k],
                    SourceWayIds=observations.SelectMany(s=>s.SourceWayIds.Count>0?s.SourceWayIds:new[]{s.WayId}).Distinct().ToArray(),
                    OriginalGeometry=observations.SelectMany(s=>s.OriginalGeometry.Count>0?s.OriginalGeometry:new[]{s.Start,s.End}).ToArray(),
                    OneWay=false,Lanes=lanes,ForwardLanes=lanes/2,BackwardLanes=lanes/2,BothWaysLanes=0,
                    SourceWidthMeters=null,SourceSpeedKph=speeds.Length>0?speeds.Min():null,
                    ExplicitLanes=observations.Any(s=>s.ExplicitLanes),ConsolidatedAvenue=true,CorridorId=0,MaximumNativeWidth=float.PositiveInfinity,
                    ExplicitCutSides=Outer(sideA,sideB,s=>s.ExplicitCutSides),ExplicitWallSides=Outer(sideA,sideB,s=>s.ExplicitWallSides),
                    SourceGeometry=new[]{positions[k-1],positions[k]},Control1=null,Control2=null,HorizontalParent=null,ExplicitIncline=null});
                if(requested>6||forward!=backward)log.Diagnostic($"AVENUE CROSS SECTION sourceWays={string.Join(",",observations.Select(s=>s.WayId).Distinct())}; requestedForward={forward}; requestedBackward={backward}; requestedTotal={requested}; supportedTotal={lanes}; directionalApproximationRequired={forward!=backward}; intervalEvidenceOnly=true");
            }
            foreach(var entry in local)remap[entry.Key]=entry.Value;
            foreach(var edge in originalA.Edges.Concat(originalB.Edges))removed.Add(edge);tails.AddRange(pairTails);
            used.Add(i);used.Add(j);pairs++;
            log.Diagnostic($"AVENUE MERGE name={a.Edges[0].Name}; reference={a.Edges[0].Reference}; sourceWays={string.Join(",",a.Edges.Concat(b.Edges).Select(e=>e.WayId).Distinct().OrderBy(id=>id))}; separationMeters={matches[i][0].score:0.##}; lengthMeters={a.Length:0}; junctionNodesRemapped={local.Count}; outputSections={positions.Length-1}; vanillaPalette=Medium-or-Large-Road; highwaysAndRampsNotConverted=true");
        }
        var result=new List<PreparedSegment>();var reconnected=0;
        foreach(var edge in source.Concat(tails))
        {
            token.ThrowIfCancellationRequested();if(removed.Contains(edge))continue;
            var s=edge;
            if(s.Kind==NetworkKind.Road&&!s.Bridge&&!s.Tunnel)
            {
                if(remap.TryGetValue((s.Layer,s.StartNodeId),out var start)){s=s with{StartNodeId=start.node,Start=start.point};reconnected++;}
                if(remap.TryGetValue((s.Layer,s.EndNodeId),out var end)){s=s with{EndNodeId=end.node,End=end.point};reconnected++;}
                if(s.StartNodeId==s.EndNodeId)
                { // A median crossover has become an internal connection at one avenue junction.
                    log.Count("Avenue median crossovers consolidated");continue;
                }
                if(s!=edge)s=s with{SourceGeometry=new[]{s.Start,s.End},Control1=null,Control2=null,HorizontalParent=null};
            }
            result.Add(s);
        }
        result.AddRange(created);
        log.Count("Opposing road pairs consolidated into avenues",pairs);log.Count("Avenue attachment endpoints reconnected",reconnected);
        log.Diagnostic($"AVENUE TOTAL eligibleSections={eligible.Length}; chains={chains.Count}; comparisons={comparisons}; pairsMerged={pairs}; sourceSectionsReplaced={removed.Count}; outputAvenueSections={created.Count}; attachmentEndpointsReconnected={reconnected}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.###}; ambiguousOrUnmatchedChainsPreserved={chains.Count-used.Count}; structuresPreserved=true");
        return AvenueJunctionRepair.Prepare(result,log,token);
    }
}
