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
    public const float CorridorDistance = 40f;
    private sealed class Axis
    {
        public PreparedSegment[] Edges = Array.Empty<PreparedSegment>();
        public float[] LengthPrefix = Array.Empty<float>();
        public int[] BreakPrefix = Array.Empty<int>();
        public readonly Dictionary<(int,int),List<int>> Segments = new();
        public readonly Dictionary<int,List<(float t,long node,float3 point)>> Knots = new();
        public readonly List<(float lo,float hi)> PairedIntervals = new();
        public readonly List<(float lo,float hi,PreparedSegment source)> PairedSources = new();
        public readonly HashSet<long> Nodes=new();
        public readonly Dictionary<long,float> NodeStations=new();
    }
    private readonly struct Projection
    {
        public readonly int Edge;
        public readonly float T,Distance;
        public readonly float3 Point;
        public Projection(int edge,float t,float distance,float3 point){Edge=edge;T=t;Distance=distance;Point=point;}
    }
    internal static void ResolveJunctions(Dictionary<(int Layer,long Node),(long node,float3 point)> map)
    {
        var resolved=new Dictionary<(int,long),(long node,float3 point)>();
        foreach(var key in map.Keys)
        {
            if(resolved.ContainsKey(key))continue;
            var path=new List<long>();var positions=new Dictionary<long,int>();var current=key.Node;
            (long node,float3 point) terminal;
            while(true)
            {
                if(resolved.TryGetValue((key.Layer,current),out terminal))break;
                if(positions.TryGetValue(current,out var cycleStart))
                {
                    // These identities have already been declared equivalent by
                    // exact transfer operations. Pick one stable representative
                    // and its existing point; never leave a different one per arm.
                    var canonical=path.Skip(cycleStart).Min();
                    terminal=map[(key.Layer,path.Skip(cycleStart).First(n=>map[(key.Layer,n)].node==canonical))];
                    break;
                }
                positions[current]=path.Count;path.Add(current);
                var next=map[(key.Layer,current)];
                if(next.node==current||!map.ContainsKey((key.Layer,next.node))){terminal=next;break;}
                current=next.node;
            }
            foreach(var node in path)resolved[(key.Layer,node)]=terminal;
        }
        foreach(var pair in resolved)map[pair.Key]=pair.Value;
    }
    public static IReadOnlyList<PreparedSegment> Prepare(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token)
    {
        // Transferring switches can expose a short redundant triangle: the
        // direct axis and an identical two-edge approach. Re-consolidate the
        // canonical raw geometry before fitting curves, preserving all tails.
        // Two tracks inside ONE native corridor segment, never two generated axes.
        source=source.Select(s=>s.Kind==NetworkKind.Rail?s with{Lanes=2,PhysicalTracks=2,OneWay=false,ExplicitWallSides=NetworkSides.None,RetainingWallSides=NetworkSides.None}:s).ToArray();
        var result=ExactPhysicalDemands(CollapseShortConnections(source,log,token),log,token);
        for(var pass=0;pass<8;pass++)
        {
            var next=PreparePass(result,log,token);
            if(next.SequenceEqual(result))return next;
            result=next;
        }
        return result;
    }
    public static IReadOnlyList<PreparedSegment> NativeJunctionFootprints(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token)
    {
        var widths=source.Where(s=>s.Kind==NetworkKind.Rail&&s.NativeWidth>0&&math.isfinite(s.NativeWidth)).Select(s=>s.NativeWidth).ToArray();
        if(widths.Length==0)return source;
        return RedundantOverlappingRoutes(RedundantParallelSpans(CollapseShortConnections(source,log,token,Math.Min(12,widths.Min())),log,token),log,token,35);
    }

    // Two rail routes joining the same two junctions inside one native track
    // footprint are one corridor drawn twice: a parallel track left beside its
    // axis, a link collapse turning a triangle into a pair, or two consolidated
    // double tracks between the same switches. The game cannot build both: they
    // overlap, and unit splitting gives each its own node at the same point so
    // neither line connects. A route is a chain of sections through degree-two
    // nodes. One route is kept and carries the other's provenance; the dropped
    // route's interior nodes become aliases of its nearer end. Routes farther
    // apart than the wider native footprint are a real loop and stay, as does a
    // route sharing a node with a road (a level crossing).
    internal static IReadOnlyList<PreparedSegment> RedundantParallelSpans(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token)
    {
        var chains=Chains(source);
        var groups=chains.Where(c=>c.A!=c.B).GroupBy(c=>(Math.Min(c.A,c.B),Math.Max(c.A,c.B))).Where(g=>g.Count()>1).ToArray();
        if(groups.Length==0)return source;
        var roadNodes=new HashSet<long>(source.Where(s=>s.Kind==NetworkKind.Road).SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}));
        var aliased=new HashSet<long>(source.Where(s=>s.Kind==NetworkKind.Rail).SelectMany(s=>s.SourceNodeAliases).Select(a=>a.SourceNodeId));
        var dropped=new HashSet<int>();var additions=new Dictionary<int,(List<long> Ways,List<SourceNodeAlias> Aliases,List<float3> Geometry)>();
        var retarget=new Dictionary<long,long>();var routes=0;
        foreach(var group in groups)
        {
            token.ThrowIfCancellationRequested();
            // A bridge or tunnel is explicit evidence of what the track passes
            // over or under; then the most consolidated provenance.
            var ranked=group.OrderByDescending(c=>c.Sections.Any(i=>source[i].Bridge||source[i].Tunnel))
                .ThenByDescending(c=>c.Sections.Sum(i=>source[i].SourceWayIds.Count)).ThenBy(c=>c.Sections.Min()).ToArray();
            var kept=ranked[0];var keptLine=Polyline(source,kept.Sections);var changed=false;
            foreach(var other in ranked.Skip(1))
            {
                if(other.Interior.Any(roadNodes.Contains))continue;
                var footprint=Math.Max(kept.Sections.Concat(other.Sections).Max(i=>source[i].NativeWidth),8f);
                var otherLine=Polyline(source,other.Sections);
                if(!(Separation(otherLine,keptLine)<=footprint&&Separation(keptLine,otherLine)<=footprint))continue;
                var ways=other.Sections.SelectMany(i=>source[i].SourceWayIds.Append(source[i].WayId)).ToList();
                var aliases=other.Sections.SelectMany(i=>source[i].SourceNodeAliases).ToList();
                // Interior nodes follow the dropped route to its nearer end.
                var station=0d;var total=other.Sections.Sum(i=>Length(source[i]));
                for(var k=0;k<other.Interior.Count;k++)
                {
                    station+=Length(source[other.Sections[k]]);var end=station<=total/2?other.A:other.B;
                    retarget[other.Interior[k]]=end;aliases.Add(new SourceNodeAlias(other.Interior[k],end,true));
                }
                foreach(var i in kept.Sections)
                {
                    if(!additions.TryGetValue(i,out var add))additions[i]=add=(new(),new(),new());
                    add.Ways.AddRange(ways);add.Geometry.AddRange(other.Sections.SelectMany(j=>source[j].OriginalGeometry));
                }
                additions[kept.Sections[0]].Aliases.AddRange(aliases);
                foreach(var i in other.Sections)dropped.Add(i);
                changed=true;routes++;
            }
            // An end may have been a junction only because of the copy; declaring
            // it its own alias target keeps it as a node and a port. An end already
            // aliased elsewhere keeps that single declaration.
            if(changed)additions[kept.Sections[0]].Aliases.AddRange(new[]{kept.A,kept.B}.Where(id=>!aliased.Contains(id)).Select(id=>new SourceNodeAlias(id,id,true)));
        }
        if(dropped.Count==0)return source;
        long Target(long id){var seen=0;while(retarget.TryGetValue(id,out var next)&&seen++<64)id=next;return id;}
        var output=new List<PreparedSegment>(source.Count-dropped.Count);
        for(var i=0;i<source.Count;i++)
        {
            if(dropped.Contains(i))continue;var s=source[i];
            // A dropped interior node's new declaration supersedes any earlier
            // one, so no source node names two targets.
            if(s.Kind==NetworkKind.Rail)s=s with{SourceNodeAliases=s.SourceNodeAliases.Where(a=>!retarget.ContainsKey(a.SourceNodeId)).ToArray()};
            if(additions.TryGetValue(i,out var add))s=s with{SourceWayIds=s.SourceWayIds.Append(s.WayId).Concat(add.Ways).Distinct().OrderBy(id=>id).ToArray(),
                OriginalGeometry=s.OriginalGeometry.Concat(add.Geometry).ToArray(),
                SourceNodeAliases=add.Aliases.Where(a=>retarget.ContainsKey(a.SourceNodeId)).Concat(s.SourceNodeAliases).Concat(add.Aliases.Where(a=>!retarget.ContainsKey(a.SourceNodeId))).ToArray()};
            if(s.Kind==NetworkKind.Rail&&s.SourceNodeAliases.Count>0)
                s=s with{SourceNodeAliases=s.SourceNodeAliases.Select(a=>a with{CanonicalNodeId=Target(a.CanonicalNodeId)})
                    .GroupBy(a=>a.SourceNodeId).Select(g=>g.First()).OrderBy(a=>a.SourceNodeId).ToArray()};
            output.Add(s);
        }
        log.Count("Redundant parallel rail routes removed",routes);log.Count("Redundant parallel rail spans removed",dropped.Count);
        log.Diagnostic($"RAIL PARALLEL ROUTES removed={routes}; sections={dropped.Count}; sameEndJunctions=true; withinNativeFootprint=true; levelCrossingsKept=true; provenanceRetained=true");
        return output;
    }

    // Rail routes: chains of sections through degree-two nodes, each ordered
    // from end A to end B, with the nodes between them.
    private static List<(List<int> Sections,long A,long B,List<long> Interior)> Chains(IReadOnlyList<PreparedSegment> source)
    {
        var rails=source.Select((s,i)=>(s,i)).Where(p=>p.s.Kind==NetworkKind.Rail&&p.s.StartNodeId!=p.s.EndNodeId).Select(p=>p.i).ToArray();
        var incident=new Dictionary<long,List<int>>();
        foreach(var i in rails)foreach(var id in new[]{source[i].StartNodeId,source[i].EndNodeId})
        {if(!incident.TryGetValue(id,out var list))incident[id]=list=new();list.Add(i);}
        var chains=new List<(List<int> Sections,long A,long B,List<long> Interior)>();var visited=new HashSet<int>();
        foreach(var seed in rails)
        {
            if(!visited.Add(seed))continue;
            var ends=new long[2];var sides=new List<int>[2];
            for(var side=0;side<2;side++)
            {
                var walk=new List<int>();var index=seed;var node=side==0?source[seed].StartNodeId:source[seed].EndNodeId;
                while(incident[node].Count==2)
                {
                    var next=incident[node][0]==index?incident[node][1]:incident[node][0];
                    if(next==seed||!visited.Add(next))break;
                    walk.Add(next);index=next;node=source[next].StartNodeId==node?source[next].EndNodeId:source[next].StartNodeId;
                }
                ends[side]=node;sides[side]=walk;
            }
            var sections=Enumerable.Reverse(sides[0]).Append(seed).Concat(sides[1]).ToList();
            var interior=new List<long>();var at=ends[0];
            foreach(var i in sections){at=source[i].StartNodeId==at?source[i].EndNodeId:source[i].StartNodeId;interior.Add(at);}
            interior.RemoveAt(interior.Count-1);
            chains.Add((sections,ends[0],ends[1],interior));
        }
        return chains;
    }

    // A rail route lying mostly inside another route's track footprint is a
    // crossover, ladder or connector that consolidated double track already
    // provides: the two would overlap in game. It is removed when its two ends
    // stay connected by a nearby alternative path (which then carries its ways)
    // or it is a dead-end spur. Crossovers, sidings and yard tracks go first,
    // then shorter routes; a level crossing is never removed. Interior nodes
    // become aliases of the nearer end.
    internal static IReadOnlyList<PreparedSegment> RedundantOverlappingRoutes(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token,double maximumBend)
    {
        var chains=Chains(source);if(chains.Count<2)return source;
        var aliased=new HashSet<long>(source.Where(s=>s.Kind==NetworkKind.Rail).SelectMany(s=>s.SourceNodeAliases).Select(a=>a.SourceNodeId));
        var roadNodes=new HashSet<long>(source.Where(s=>s.Kind==NetworkKind.Road).SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}));
        var lengths=chains.Select(c=>c.Sections.Sum(i=>Length(source[i]))).ToArray();
        var widths=chains.Select(c=>(double)Math.Max(c.Sections.Max(i=>source[i].NativeWidth),4f)).ToArray();
        // Samples every 2 m with their route, height and structure.
        const double step=2;const double cell=16;
        var samples=new List<(int Chain,double2 P,double Y,int Mode)>();
        for(var c=0;c<chains.Count;c++)foreach(var i in chains[c].Sections)
        {
            var s=source[i];var curve=new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);
            var n=Math.Max(1,(int)Math.Ceiling(Length(s)/step));var mode=s.Tunnel?-1:s.Bridge?1:0;
            for(var k=0;k<n;k++){var p=NetworkDesignChecks.Evaluate(curve,(k+.5)/n);samples.Add((c,p.xz,p.y,mode));}
        }
        var grid=new Dictionary<(int,int),List<int>>();
        for(var k=0;k<samples.Count;k++)
        {var key=((int)Math.Floor(samples[k].P.x/cell),(int)Math.Floor(samples[k].P.y/cell));if(!grid.TryGetValue(key,out var list))grid[key]=list=new();list.Add(k);}
        var covered=new Dictionary<(int C,int D),double>();
        foreach(var a in samples)
        {
            token.ThrowIfCancellationRequested();var cx=(int)Math.Floor(a.P.x/cell);var cz=(int)Math.Floor(a.P.y/cell);var seen=new HashSet<int>();
            for(var dz=-1;dz<=1;dz++)for(var dx=-1;dx<=1;dx++)if(grid.TryGetValue((cx+dx,cz+dz),out var near))foreach(var k in near)
            {
                var b=samples[k];if(b.Chain==a.Chain||b.Mode!=a.Mode||Math.Abs(b.Y-a.Y)>=7||seen.Contains(b.Chain))continue;
                if(math.distance(a.P,b.P)>=(widths[a.Chain]+widths[b.Chain])/2)continue;
                seen.Add(b.Chain);covered[(a.Chain,b.Chain)]=covered.TryGetValue((a.Chain,b.Chain),out var m)?m+step:step;
            }
        }
        bool Minor(int c)=>chains[c].Sections.Any(i=>source[i].RailService.Length>0);
        // A route sharing both ends with another that turns more than 45°
        // somewhere along its own course, where the other does not, is a lens
        // left by consolidation (a second track's ends merged onto the first, now
        // a 97° detour beside it); it goes whatever its overlap.
        double Turn(int c)
        {
            var at=chains[c].A;var line=new List<double2>();
            foreach(var i in chains[c].Sections)
            {
                var s=source[i];var forward=s.StartNodeId==at;at=forward?s.EndNodeId:s.StartNodeId;
                var part=Polyline(source,new List<int>{i});if(!forward)part.Reverse();line.AddRange(line.Count==0?part:part.Skip(1));
            }
            var worst=0d;
            for(var k=1;k<line.Count-1;k++)
            {
                var u=line[k]-line[k-1];var v=line[k+1]-line[k];if(math.lengthsq(u)<1e-4||math.lengthsq(v)<1e-4)continue;
                worst=Math.Max(worst,math.degrees(Math.Acos(math.clamp(math.dot(math.normalize(u),math.normalize(v)),-1,1))));
            }
            return worst;
        }
        var turns=chains.Select((_,c)=>Turn(c)).ToArray();
        bool SameEnds(int c,int d)=>chains[c].A==chains[d].A&&chains[c].B==chains[d].B||chains[c].A==chains[d].B&&chains[c].B==chains[d].A;
        var lens=new HashSet<int>(Enumerable.Range(0,chains.Count).Where(c=>turns[c]>45&&Enumerable.Range(0,chains.Count).Any(d=>d!=c&&SameEnds(c,d)&&turns[d]<=45)));
        // A spur of at most StubLength from a junction to nowhere is the remnant of
        // a crossover or ladder whose other tracks were consolidated (DC Union
        // Station throat: 7 m and 19 m stubs); it goes whatever its overlap.
        const double StubLength=25;var degree=new Dictionary<long,int>();
        foreach(var ch in chains)foreach(var id in new[]{ch.A,ch.B})degree[id]=degree.TryGetValue(id,out var d)?d+1:1;
        var stubs=new HashSet<int>(Enumerable.Range(0,chains.Count).Where(c=>lengths[c]<=StubLength&&
            (degree[chains[c].A]==1&&degree[chains[c].B]>=3||degree[chains[c].B]==1&&degree[chains[c].A]>=3)));
        var candidates=covered.Where(p=>p.Value>=.5*lengths[p.Key.C]).Select(p=>p.Key.C).Concat(lens).Concat(stubs).Distinct()
            .Where(c=>chains[c].A!=chains[c].B&&lengths[c]<=400&&!chains[c].Interior.Any(roadNodes.Contains))
            .OrderByDescending(Minor).ThenBy(c=>lengths[c]).ThenBy(c=>chains[c].Sections.Min()).ToList();
        if(candidates.Count==0)return source;
        var where=new Dictionary<long,double2>();
        foreach(var s in source.Where(s=>s.Kind==NetworkKind.Rail)){where[s.StartNodeId]=((double3)s.CourseStart).xz;where[s.EndNodeId]=((double3)s.CourseEnd).xz;}
        var adjacency=new Dictionary<long,List<int>>();
        for(var c=0;c<chains.Count;c++)foreach(var id in new[]{chains[c].A,chains[c].B})
        {if(!adjacency.TryGetValue(id,out var list))adjacency[id]=list=new();list.Add(c);}
        // Plan direction in which a route leaves one of its end nodes.
        double2 Away(int x,long node)
        {
            var first=chains[x].A==node;var s=source[first?chains[x].Sections[0]:chains[x].Sections[^1]];
            var atStart=s.StartNodeId==node;
            return math.normalizesafe(atStart?((double3)s.CourseControl1-(double3)s.CourseStart).xz:((double3)s.CourseControl2-(double3)s.CourseEnd).xz);
        }
        var removed=new HashSet<int>();var extraWays=new Dictionary<int,List<long>>();var aliases=new List<SourceNodeAlias>();var retarget=new Dictionary<long,long>();
        int Live(long id)=>adjacency[id].Count(x=>!removed.Contains(x));
        // Removes route c. Its ways go onto the alternative path, or for a spur
        // onto every route still at its junction; its interior nodes become
        // aliases of the nearer end, and a spur's dead end follows it to the
        // junction it left.
        void Drop(int c,List<int>? path)
        {
            var chain=chains[c];removed.Add(c);
            // Ways this route took over from routes removed before it travel on too.
            var ways=chain.Sections.SelectMany(i=>source[i].SourceWayIds.Append(source[i].WayId).Concat(extraWays.TryGetValue(i,out var inherited)?inherited:Enumerable.Empty<long>())).Distinct().ToList();
            if(path==null)path=adjacency[Live(chain.A)>0?chain.A:chain.B].Where(x=>!removed.Contains(x)).ToList();
            var spur=!(Live(chain.A)>0&&Live(chain.B)>0);
            foreach(var x in path)foreach(var i in chains[x].Sections){if(!extraWays.TryGetValue(i,out var list))extraWays[i]=list=new();list.AddRange(ways);}
            aliases.AddRange(chain.Sections.SelectMany(i=>source[i].SourceNodeAliases));
            var station=0d;
            for(var k=0;k<chain.Interior.Count;k++)
            {
                station+=Length(source[chain.Sections[k]]);var end=spur?(Live(chain.A)>0?chain.A:chain.B):station<=lengths[c]/2?chain.A:chain.B;
                retarget[chain.Interior[k]]=end;aliases.Add(new SourceNodeAlias(chain.Interior[k],end,true));
            }
            if(log.DiagnosticsEnabled)log.Diagnostic($"RAIL OVERLAPPING ROUTE removed ends={chain.A},{chain.B}; liveAfter={Live(chain.A)},{Live(chain.B)}; interior={string.Join(",",chain.Interior)}; spur={spur}; lengthMeters={lengths[c]:0.#}; alternativeRoutes={path.Count}");
            // A surviving end may have been a junction only because of this route;
            // declaring it its own alias target keeps it as a node and a port.
            foreach(var id in new[]{chain.A,chain.B})if(Live(id)>0&&!aliased.Contains(id)&&!retarget.ContainsKey(id))aliases.Add(new SourceNodeAlias(id,id,true));
            if(spur)foreach(var id in new[]{chain.A,chain.B})if(Live(id)==0)
            {var end=id==chain.A?chain.B:chain.A;retarget[id]=end;aliases.Add(new SourceNodeAlias(id,end,true));}
        }
        // A connector becomes a spur only once a route beside it has gone (a
        // ladder's last rung: Marseille 9208479591, DC Union Station throat), so
        // candidates are revisited until a pass removes nothing.
        for(var pass=0;pass<4;pass++)
        {
        var before=removed.Count;
        foreach(var c in candidates)
        {
            token.ThrowIfCancellationRequested();if(removed.Contains(c))continue;
            // It must overlap a route that is still there.
            if(!stubs.Contains(c)&&!covered.Any(p=>p.Key.C==c&&!removed.Contains(p.Key.D)&&p.Value>=.5*lengths[c])&&
                !(lens.Contains(c)&&Enumerable.Range(0,chains.Count).Any(d=>d!=c&&!removed.Contains(d)&&SameEnds(c,d)&&turns[d]<=45)))continue;
            var chain=chains[c];var live=new Func<long,int>(id=>adjacency[id].Count(x=>!removed.Contains(x)));
            List<int>? path=null;
            if(live(chain.A)>1&&live(chain.B)>1)
            {
                // Shortest alternative path between the two ends, bounded.
                var limit=3*lengths[c]+300;var best=new Dictionary<long,double>{[chain.A]=0};var via=new Dictionary<long,int>();
                var queue=new SortedSet<(double D,long Node)>{(0,chain.A)};
                while(queue.Count>0)
                {
                    var (d,node)=queue.Min;queue.Remove(queue.Min);if(node==chain.B||d>limit)break;
                    foreach(var x in adjacency[node])
                    {
                        if(x==c||removed.Contains(x))continue;var other=chains[x].A==node?chains[x].B:chains[x].A;var nd=d+lengths[x];
                        if(best.TryGetValue(other,out var old)&&old<=nd)continue;
                        if(best.ContainsKey(other))queue.Remove((old,other));best[other]=nd;via[other]=x;queue.Add((nd,other));
                    }
                }
                if(!best.TryGetValue(chain.B,out var found)||found>limit)continue;
                path=new List<int>();for(var node=chain.B;node!=chain.A;){var x=via[node];path.Add(x);node=chains[x].A==node?chains[x].B:chains[x].A;}
                // A straight main-line track is not removed in favour of a short
                // detour (at most twice its length) through a junction more than
                // 8 m off it where the detour turns more than 45° (the tent at
                // 43.3633, 5.3332). A yard ladder's alternative along the parallel
                // track 4–5 m away, crossovers and sidings are not protected.
                var bent=false;
                if(!Minor(c)&&turns[c]<=10&&found<=2*lengths[c])
                {
                    var a=where[chain.A];var ab=where[chain.B]-a;
                    double Off(long id){var t=math.clamp(math.dot(where[id]-a,ab)/Math.Max(1e-9,math.lengthsq(ab)),0,1);return math.distance(where[id],a+ab*t);}
                    for(var (k,node)=(1,chains[path[0]].A==chain.B?chains[path[0]].B:chains[path[0]].A);k<path.Count&&!bent;k++)
                    {
                        bent=Off(node)>8&&math.dot(Away(path[k-1],node),Away(path[k],node))>-Math.Cos(math.radians(45));
                        node=chains[path[k]].A==node?chains[path[k]].B:chains[path[k]].A;
                    }
                }
                if(bent)
                {
                    if(log.DiagnosticsEnabled)log.Diagnostic($"RAIL OVERLAPPING ROUTE kept straight ends={chain.A},{chain.B}; lengthMeters={lengths[c]:0.#}; detourRoutes={path.Count}");
                    continue;
                }
            }
            else if(live(chain.A)>1==live(chain.B)>1)continue;
            // An end left with two routes becomes a plain track node; it must not
            // bend more than maximumBend there (before design a fillet smooths a
            // moderate bend; afterwards any bend is a kink). A stub's junction keeps
            // the through track it already had, so its bend is not new.
            bool Bends(long id,params int[] going)
            {
                var rest=adjacency[id].Where(x=>!going.Contains(x)&&!removed.Contains(x)).ToArray();if(rest.Length!=2)return false;
                var a=Away(rest[0],id);var b=Away(rest[1],id);return math.dot(a,b)>-Math.Cos(math.radians(maximumBend));
            }
            if(!stubs.Contains(c)&&new[]{chain.A,chain.B}.Any(id=>Bends(id,c)))continue;
            // An end that stays with one other route would become a dead end of
            // it, once an earlier removal has taken a third route off that switch
            // (Marseille Saint-Charles, DC Union Station: seven new dead ends,
            // each a former switch). That route goes too if it is a spur of at
            // most SpurLength to a junction that keeps two routes without a new
            // bend; otherwise this route stays.
            const double SpurLength=60;var spurs=new List<int>();var refused=false;
            foreach(var id in new[]{chain.A,chain.B}.Where(id=>live(id)==2))
            {
                var r=adjacency[id].First(x=>x!=c&&!removed.Contains(x));var o=chains[r].A==id?chains[r].B:chains[r].A;
                if(chains[r].A==chains[r].B||lengths[r]>SpurLength||live(o)<3||spurs.Contains(r)||chains[r].Interior.Any(roadNodes.Contains)||Bends(o,c,r))
                {refused=true;break;}
                spurs.Add(r);
            }
            if(refused)continue;
            Drop(c,path);
            foreach(var r in spurs)Drop(r,null);
        }
        if(removed.Count==before)break;
        }
        if(removed.Count==0)return source;
        long Target(long id){var guard=0;while(retarget.TryGetValue(id,out var next)&&guard++<64)id=next;return id;}
        var dropped=new HashSet<int>(removed.SelectMany(c=>chains[c].Sections));
        var output=new List<PreparedSegment>(source.Count-dropped.Count);var carrier=-1;
        for(var i=0;i<source.Count;i++)
        {
            if(dropped.Contains(i))continue;var s=source[i];
            if(extraWays.TryGetValue(i,out var ways))s=s with{SourceWayIds=s.SourceWayIds.Append(s.WayId).Concat(ways).Distinct().OrderBy(id=>id).ToArray()};
            if(s.Kind==NetworkKind.Rail)
            {
                // A removed node's new declaration supersedes any earlier one, so
                // no source node names two targets.
                var own=s.SourceNodeAliases.Where(a=>!retarget.ContainsKey(a.SourceNodeId));
                if(carrier<0){carrier=output.Count;own=aliases.Where(a=>retarget.ContainsKey(a.SourceNodeId)).Concat(own).Concat(aliases.Where(a=>!retarget.ContainsKey(a.SourceNodeId)));}
                s=s with{SourceNodeAliases=own.Select(a=>a with{CanonicalNodeId=Target(a.CanonicalNodeId)})
                    .GroupBy(a=>a.SourceNodeId).Select(g=>g.First()).OrderBy(a=>a.SourceNodeId).ToArray()};
            }
            output.Add(s);
        }
        log.Count("Overlapping rail routes removed",removed.Count);
        log.Diagnostic($"RAIL OVERLAPPING ROUTES removed={removed.Count}; sections={dropped.Count}; insideOtherFootprint>=50%; endsStayConnected=true; levelCrossingsKept=true; provenanceOnAlternativePath=true");
        return output;
    }

    private static double Length(PreparedSegment s)=>NetworkDesignChecks.HorizontalArcLength(new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd));

    // Plan polyline through a route's courses, sampled every few metres.
    private static List<double2> Polyline(IReadOnlyList<PreparedSegment> source,List<int> sections)
    {
        var line=new List<double2>();
        foreach(var i in sections)
        {
            var s=source[i];var c=new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);
            var n=Math.Max(4,Math.Min(64,(int)Math.Ceiling(Length(s)/5)));
            for(var k=0;k<=n;k++)line.Add(NetworkDesignChecks.Evaluate(c,k/(double)n).xz);
        }
        return line;
    }

    // Largest horizontal distance from a point of one polyline to the other.
    private static double Separation(List<double2> from,List<double2> to)
    {
        var worst=0d;
        foreach(var p in from)
        {
            var best=double.MaxValue;
            for(var j=1;j<to.Count;j++)
            {
                var delta=to[j]-to[j-1];var sq=math.lengthsq(delta);
                var t=sq>0?math.clamp(math.dot(p-to[j-1],delta)/sq,0,1):0;best=Math.Min(best,math.distance(p,to[j-1]+t*delta));
            }
            worst=Math.Max(worst,best);
        }
        return worst;
    }

    private static IReadOnlyList<PreparedSegment> CollapseShortConnections(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token,double maximumDiameter=3)
    {
        var incident=new Dictionary<long,List<PreparedSegment>>();var points=new Dictionary<long,float3>();
        var protectedNodes=new HashSet<long>(source.Where(s=>s.Kind==NetworkKind.Road).SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}));
        foreach(var s in source.Where(s=>s.Kind==NetworkKind.Rail))foreach(var start in new[]{true,false})
        {
            var id=start?s.StartNodeId:s.EndNodeId;
            if(!incident.TryGetValue(id,out var arms))incident[id]=arms=new();arms.Add(s);points[id]=start?s.Start:s.End;
            var curved=(s.Control1.HasValue||s.Control2.HasValue)&&(maximumDiameter<=3||
                NetworkNormalizer.DistanceToSegment(s.CourseControl1,s.Start,s.End)>.05||NetworkNormalizer.DistanceToSegment(s.CourseControl2,s.Start,s.End)>.05);
            if((start?s.StartTerminal:s.EndTerminal)||curved||s.FinalDeckCurve.HasValue||
                math.any(s.StartCurveOffset!=0)||math.any(s.EndCurveOffset!=0))protectedNodes.Add(id);
        }
        foreach(var node in incident)
            if(node.Value.Count==1||node.Value.Select(s=>(s.Layer,s.Bridge,s.Tunnel,s.Classification,s.RailService)).Distinct().Count()!=1)protectedNodes.Add(node.Key);
        var parents=points.Keys.ToDictionary(id=>id,id=>id);
        var low=points.ToDictionary(p=>p.Key,p=>(double2)p.Value.xz);var high=new Dictionary<long,double2>(low);
        long Root(long id){var root=id;while(parents[root]!=root)root=parents[root];while(parents[id]!=id){var next=parents[id];parents[id]=root;id=next;}return root;}
        var maximum=maximumDiameter;
        foreach(var s in source.Where(s=>s.Kind==NetworkKind.Rail).OrderBy(s=>math.distancesq(s.Start.xz,s.End.xz)).ThenBy(s=>s.WayId).ThenBy(s=>s.StartNodeId))
        {
            token.ThrowIfCancellationRequested();if(protectedNodes.Contains(s.StartNodeId)||protectedNodes.Contains(s.EndNodeId))continue;
            var a=Root(s.StartNodeId);var b=Root(s.EndNodeId);if(a==b)continue;
            var min=math.min(low[a],low[b]);var max=math.max(high[a],high[b]);
            // Bound the whole cluster, not each successive link: a long run of
            // short edges must never collapse into one drifting junction.
            if(math.distance(min,max)>maximum)continue;
            if(incident[a].Count<incident[b].Count||incident[a].Count==incident[b].Count&&a>b)(a,b)=(b,a);
            parents[b]=a;low[a]=min;high[a]=max;
        }
        var survivingRoots=new HashSet<long>(source.Where(s=>s.Kind==NetworkKind.Rail&&Root(s.StartNodeId)!=Root(s.EndNodeId))
            .SelectMany(s=>new[]{Root(s.StartNodeId),Root(s.EndNodeId)}));
        // Keep an isolated short component: without a surviving incident span
        // there is nowhere to carry its source identities or visible geometry.
        var isolated=points.Keys.Where(id=>!survivingRoots.Contains(Root(id))).ToArray();
        foreach(var id in isolated)parents[id]=id;
        var remap=points.Keys.Where(id=>Root(id)!=id).ToDictionary(id=>id,id=>Root(id));
        if(remap.Count==0)return source;
        var output=new List<PreparedSegment>(source.Count);var removed=0;
        foreach(var s in source)
        {
            token.ThrowIfCancellationRequested();if(s.Kind!=NetworkKind.Rail){output.Add(s);continue;}
            var a=Root(s.StartNodeId);var b=Root(s.EndNodeId);if(a==b){removed++;continue;}
            if(a==s.StartNodeId&&b==s.EndNodeId){output.Add(s);continue;}
            output.Add(s with{StartNodeId=a,EndNodeId=b,Start=points[a],End=points[b],
                Control1=s.Control1.HasValue?math.lerp(points[a],points[b],1f/3):null,
                Control2=s.Control2.HasValue?math.lerp(points[a],points[b],2f/3):null,SourceGeometry=new[]{points[a],points[b]}});
        }
        log.Count("Sub-footprint connected rail links consolidated",removed);
        log.Diagnostic($"RAIL SHORT LINKS removed={removed}; aliasedSourceNodes={remap.Count}; maximumClusterDiameterMeters={maximum}; nativeFootprintPolicy={maximum>3}; connectedSourceEdgesOnly=true; modeBoundariesPreserved=true; proximityWeld=false");
        return SourceTopologyObligations.Transfer(source,output,NetworkKind.Rail,remap);
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
            result[retained]=kept with{SourceWayIds=provenance,SourceNodeAliases=kept.SourceNodeAliases.Concat(s.SourceNodeAliases).Distinct().ToArray()};merged++;
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
        var sourceJunctions=new Dictionary<long,(long node,float3 point)>();
        var protectedNodes=new HashSet<long>(source.Where(s=>s.Kind==NetworkKind.Road).SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}));
        // Earlier consolidation passes can retire a generated node while its
        // identity remains in an alias. It must never name a new rail junction.
        var ids=new HashSet<long>(source.SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}
            .Concat(s.SourceNodeAliases.SelectMany(a=>new[]{a.SourceNodeId,a.CanonicalNodeId}))));long next=-7_800_000_000_000_000_000;
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
                axis.Nodes.Add(edge.StartNodeId);axis.Nodes.Add(edge.EndNodeId);
                axis.NodeStations.TryAdd(edge.StartNodeId,axis.LengthPrefix[i]);axis.NodeStations.TryAdd(edge.EndNodeId,axis.LengthPrefix[i+1]);
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
                // Import policy deliberately folds yard ladders and multiple
                // parallel routes onto one usable bidirectional rail corridor.
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
                    if(axis.PairedIntervals.Any(p=>intervalLo<p.hi-.05f&&intervalHi>p.lo+.05f))continue;
                    if(map.TryGetValue((edge.Layer,edge.StartNodeId),out var oldA)&&math.distance(oldA.point.xz,a.Point.xz)>.5f ||
                       map.TryGetValue((edge.Layer,edge.EndNodeId),out var oldB)&&math.distance(oldB.point.xz,b.Point.xz)>.5f)continue;
                    // Layer changes do not give a shared OSM junction two
                    // identities. Reuse its retained axis, or keep this span
                    // distinct rather than publishing an ambiguous portal.
                    if(sourceJunctions.TryGetValue(edge.StartNodeId,out var sharedA)&&
                        (!axis.Nodes.Contains(sharedA.node)||math.distance(sharedA.point.xz,a.Point.xz)>.5f)||
                       sourceJunctions.TryGetValue(edge.EndNodeId,out var sharedB)&&
                        (!axis.Nodes.Contains(sharedB.node)||math.distance(sharedB.point.xz,b.Point.xz)>.5f))continue;
                    spans.Add((edge,a,b));length+=originalLength;distance+=(a.Distance+b.Distance)*.5f*originalLength;
                }
                // A connected source run must have one continuous, monotone
                // station map. Independent nearest-edge projections can jump
                // between nearby switch/loop legs even when every edge fits.
                var ports=new Dictionary<long,List<(float Here,float Away)>>();
                foreach(var span in spans)
                {
                    float Station(Projection p)=>axis.LengthPrefix[p.Edge]+p.T*(axis.LengthPrefix[p.Edge+1]-axis.LengthPrefix[p.Edge]);
                    var a=Station(span.Item2);var b=Station(span.Item3);
                    void Port(long node,float here,float away)
                    {if(!ports.TryGetValue(node,out var list))ports[node]=list=new();list.Add((here,away));}
                    Port(span.Item1.StartNodeId,a,b);Port(span.Item1.EndNodeId,b,a);
                }
                if(ports.Values.Any(p=>p.Max(v=>v.Here)-p.Min(v=>v.Here)>.5f||
                    p.Count==2&&(p[0].Away-p[0].Here)*(p[1].Away-p[1].Here)>=0))continue;
                // Fold even a short overlapping interval. Unique extensions are
                // retained and reattached, not dropped because the whole way differs.
                // An abutment/tag boundary can split an otherwise identified
                // rail route into a short pair. The usual 40 m overlap cannot
                // be required there: it leaves two overlapping native double
                // tracks at a shared junction. A shared canonical source junction and
                // the same nonempty route ref supplement (never replace) all
                // geometric, layer/mode and monotone projection checks above.
                var connectedRoute=length>0&&retainedSource.Reference.Length>0&&retainedSource.Reference==candidateSource.Reference&&
                    retainedSource.Classification==candidateSource.Classification&&retainedSource.RailService==candidateSource.RailService&&
                    spans.Any(p=>axis.Nodes.Contains(p.Item1.StartNodeId)||axis.Nodes.Contains(p.Item1.EndNodeId));
                if((length>=40||connectedRoute)&&length>0&&distance/length>=1.5f)options.Add((index,spans,length,distance/length));
            }
            var ranked=options.OrderByDescending(o=>o.length).ThenBy(o=>o.distance).ThenBy(o=>o.axis).ToArray();
            var best=ranked.FirstOrDefault();
            if(ranked.Length>1&&ranked[1].length>=best.length*.9f&&Math.Abs(ranked[1].distance-best.distance)<1f)
            {
                // Candidate ordering is not source evidence. Keep the distinct
                // route rather than folding it into an arbitrary neighbouring fan.
                log.Count("Rail ambiguous axes retained for source review");AddAxis(group);continue;
            }
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
                var node=NewNode();knots.Add((p.T,node,p.Point));retained.Nodes.Add(node);
                retained.NodeStations[node]=retained.LengthPrefix[p.Edge]+p.T*(retained.LengthPrefix[p.Edge+1]-retained.LengthPrefix[p.Edge]);
                return(node,p.Point);
            }
            foreach(var span in best.spans)
            {
                var a=sourceJunctions.TryGetValue(span.edge.StartNodeId,out var existingA)?existingA:Junction(span.a);
                var b=sourceJunctions.TryGetValue(span.edge.EndNodeId,out var existingB)?existingB:Junction(span.b);
                sourceJunctions[span.edge.StartNodeId]=a;sourceJunctions[span.edge.EndNodeId]=b;
                map[(span.edge.Layer,span.edge.StartNodeId)]=a;map[(span.edge.Layer,span.edge.EndNodeId)]=b;removed.Add(span.edge);
                float Station(Projection p)=>retained.LengthPrefix[p.Edge]+p.T*(retained.LengthPrefix[p.Edge+1]-retained.LengthPrefix[p.Edge]);
                var first=retained.NodeStations.TryGetValue(a.node,out var stationA)?stationA:Station(span.a);
                var last=retained.NodeStations.TryGetValue(b.node,out var stationB)?stationB:Station(span.b);
                retained.PairedIntervals.Add((Math.Min(first,last),Math.Max(first,last)));
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
        // A transferred junction may itself be transferred by a later source
        // group in this pass. Geometry and provenance must consume the same
        // terminal identity, rather than geometry taking one hop while aliases
        // follow the complete remapping chain.
        ResolveJunctions(map);
        var pairedAxes=axes.Where(a=>a.PairedIntervals.Count>0).SelectMany(a=>a.Edges.Select(e=>(e.WayId,axis:a))).GroupBy(p=>p.WayId).ToDictionary(g=>g.Key,g=>g.Select(p=>p.axis).Distinct().ToArray());
        // Portal ways commonly change layer tags. A source node shared across that
        // transition must follow its unique canonical junction, not the old axis.
        var portalMap=map.GroupBy(p=>p.Key.Item2).Where(g=>g.Select(p=>p.Value.node).Distinct().Count()==1)
            .ToDictionary(g=>g.Key,g=>g.First().Value);
        // OSM source identity is shared across a layer/tag boundary. Resolve
        // unique portal transfers together too: two individually acyclic layer
        // maps can otherwise form an A->B, B->A cycle at that boundary.
        var portalTransfers=portalMap.ToDictionary(p=>(0,p.Key),p=>p.Value);
        ResolveJunctions(portalTransfers);
        foreach(var entry in portalTransfers)portalMap[entry.Key.Item2]=entry.Value;
        foreach(var key in map.Keys.ToArray())
        {
            if(portalMap.TryGetValue(key.Item2,out var canonical))map[key]=canonical;
            else if(portalMap.TryGetValue(map[key].node,out canonical))map[key]=canonical;
        }
        foreach(var edge in source)
        {
            token.ThrowIfCancellationRequested();if(removed.Contains(edge))continue;
            foreach(var original in replacements.TryGetValue(edge,out var pieces)?(IEnumerable<PreparedSegment>)pieces:new[]{edge})
            {
                var absorbed=original.Kind==NetworkKind.Rail?AbsorbedSources(original).Distinct().ToArray():Array.Empty<PreparedSegment>();
                var s=original.Kind==NetworkKind.Rail?original with{Lanes=2,PhysicalTracks=2,OneWay=false,ExplicitWallSides=NetworkSides.None,RetainingWallSides=NetworkSides.None}:original;
                if(absorbed.Length>0)
                {
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
        log.Diagnostic($"RAIL TOTAL waysConsolidated={merged}; spansConsolidated={removed.Count}; endpointsReconnected={reconnected}; corridorDistanceMeters={CorridorDistance}; outputRailSpans={output.Count(s=>s.Kind==NetworkKind.Rail)}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.###}; bridgeTunnelSeparationPreserved=true; differentLayersPreserved=true; oneSegmentPerAxis=true; yardBranchesPrunedByBackbone=true");
        var aliased=SourceTopologyObligations.Transfer(source,output,NetworkKind.Rail,portalMap.ToDictionary(p=>p.Key,p=>p.Value.node));
        return RailJunctionRepair.Prepare(aliased,log,token);

        IEnumerable<PreparedSegment> AbsorbedSources(PreparedSegment section)
        {
            // Only intervals which actually absorbed a second physical track are doubled.
            if(!pairedAxes.TryGetValue(section.WayId,out var owners))yield break;
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
                    foreach(var interval in axis.PairedSources)
                        if(station>=interval.lo&&station<=interval.hi)yield return interval.source;
                }
            }
        }
    }
}
