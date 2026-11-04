using System;
using System.Collections.Generic;
using System.Linq;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// OSM lane counts change wherever a mapper split a way: a turn pocket, a short
// auxiliary lane, or simply a different survey. Imported, each change is a
// visible width step (New York Avenue: 3 → 4 → 3 lanes within a few hundred
// metres). Looking consistent matters more here than matching every pocket, so
// along each continuous road a run of one lane count shorter than ShortRun,
// between two runs that agree with each other, takes their count; a run under
// half that at either end of the road takes its only neighbour's. Shortest runs
// go first, so nested steps settle from the inside out.
//
// A road continues through a node into the section with the same class,
// direction and name or ref (or, with neither, the only candidate), heading
// within 45°. Ramps never continue a mainline, so real lane adds and drops at
// interchanges are kept unless they are themselves shorter than ShortRun.
public static class NetworkLaneRuns
{
    private const double ShortRun = 300;

    // Ordinary two-way streets are traced in either OSM digitisation direction.
    // Only short width bumps supported by a longer matching road are removed;
    // sustained changes and ambiguous junctions keep their source lane counts.
    public static IReadOnlyList<PreparedSegment> NormalizeStreets(IReadOnlyList<PreparedSegment> source,ImportLog log)
    {
        bool Eligible(PreparedSegment s)=>s.Kind==NetworkKind.Road&&!s.OneWay&&!NativeAssetKeys.IsHighway(s)&&!s.ProtectedLink&&
            !s.Roundabout&&!NetworkStructure.IsStructure(s)&&!NetworkBoundaryPolicy.EndpointMarker(s,true)&&!NetworkBoundaryPolicy.EndpointMarker(s,false);
        var incident=new Dictionary<long,List<int>>();
        for(var i=0;i<source.Count;i++)foreach(var node in new[]{source[i].StartNodeId,source[i].EndNodeId})
        {if(!incident.TryGetValue(node,out var list))incident[node]=list=new();list.Add(i);}
        bool SameRoad(PreparedSegment a,PreparedSegment b)=>a.Layer==b.Layer&&
            (a.Reference.Length>0&&b.Reference.Length>0?string.Equals(a.Reference,b.Reference,StringComparison.OrdinalIgnoreCase):
             a.Name.Length==0||b.Name.Length==0||string.Equals(a.Name,b.Name,StringComparison.OrdinalIgnoreCase));
        double2 Away(int i,long node)=>math.normalizesafe((double2)(source[i].StartNodeId==node?source[i].End-source[i].Start:source[i].Start-source[i].End).xz);
        bool Protected(long node)=>incident[node].Any(j=>source[j].Kind!=NetworkKind.Road||NativeAssetKeys.IsHighway(source[j])||source[j].ProtectedLink||source[j].Roundabout||NetworkStructure.IsStructure(source[j]));
        int Next(int i,long node)
        {
            var arms=incident[node];
            if(Protected(node))return -1;
            var found=-1;
            foreach(var j in arms)
            {
                if(i==j||!Eligible(source[j])||!SameRoad(source[i],source[j])||math.dot(Away(i,node),Away(j,node))>-.906307787)continue;
                if(found>=0)return -1;found=j;
            }
            return found;
        }
        var links=new Dictionary<(int,long),int>();
        for(var i=0;i<source.Count;i++)if(Eligible(source[i]))foreach(var node in new[]{source[i].StartNodeId,source[i].EndNodeId})
        {var j=Next(i,node);if(j>=0&&Next(j,node)==i)links[(i,node)]=j;}
        var used=new bool[source.Count];PreparedSegment[]? result=null;var changed=0;
        foreach(var first in Enumerable.Range(0,source.Count).OrderBy(i=>links.ContainsKey((i,source[i].StartNodeId))&&links.ContainsKey((i,source[i].EndNodeId))?1:0))
        {
            if(used[first]||!Eligible(source[first]))continue;
            var chain=new List<(int Index,bool Forward)>();var node=links.ContainsKey((first,source[first].StartNodeId))?source[first].EndNodeId:source[first].StartNodeId;
            var current=first;
            while(!used[current])
            {
                used[current]=true;var s=source[current];var forward=s.StartNodeId==node;chain.Add((current,forward));node=forward?s.EndNodeId:s.StartNodeId;
                if(!links.TryGetValue((current,node),out current)){current=-1;break;}
            }
            if(current>=0||chain.Count<2)continue; // Closed loops have no established road body.
            var lanes=chain.Select(p=>source[p.Index].Lanes).ToArray();
            var lengths=chain.Select(p=>(double)math.distance(source[p.Index].Start.xz,source[p.Index].End.xz)).ToArray();
            while(true)
            {
                var runs=new List<(int From,int To,double Length)>();
                for(var k=0;k<chain.Count;)
                {var end=k;var length=lengths[k];while(end+1<chain.Count&&lanes[end+1]==lanes[k])length+=lengths[++end];runs.Add((k,end,length));k=end+1;}
                var pick=-1;var donor=-1;
                for(var r=0;r<runs.Count;r++)
                {
                    var run=runs[r];var before=r>0?runs[r-1]:((int From,int To,double Length)?)null;
                    var after=r+1<runs.Count?runs[r+1]:((int From,int To,double Length)?)null;
                    var target=-1;
                    if(before.HasValue&&after.HasValue&&run.Length<=180&&before.Value.Length>=30&&after.Value.Length>=30&&
                        before.Value.Length+after.Value.Length>=Math.Max(120,run.Length*2)&&lanes[before.Value.To]==lanes[after.Value.From])target=before.Value.To;
                    else if(!before.HasValue||!after.HasValue)
                    {var neighbour=before??after;if(neighbour.HasValue&&run.Length<=60&&neighbour.Value.Length>=Math.Max(120,run.Length*3))target=neighbour.Value.From;}
                    // This pass removes widening; it does not expand a narrow street
                    // to match neighbouring avenues or reduce a persistent lane add.
                    if(target<0||lanes[run.From]<=lanes[target]||pick>=0&&run.Length>=runs[pick].Length)continue;
                    if(Enumerable.Range(run.From,run.To-run.From+1).Any(k=>Protected(source[chain[k].Index].StartNodeId)||Protected(source[chain[k].Index].EndNodeId)))continue;
                    pick=r;donor=target;
                }
                if(pick<0)break;
                var template=(result??source)[chain[donor].Index];var count=lanes[donor];
                for(var k=runs[pick].From;k<=runs[pick].To;k++)
                {
                    result??=source.ToArray();var index=chain[k].Index;var s=result[index];var aligned=chain[k].Forward==chain[donor].Forward;
                    result[index]=s with{Lanes=count,ForwardLanes=aligned?template.ForwardLanes:template.BackwardLanes,
                        BackwardLanes=aligned?template.BackwardLanes:template.ForwardLanes,BothWaysLanes=template.BothWaysLanes,
                        ConsolidatedAvenue=count>2&&template.ConsolidatedAvenue};lanes[k]=count;changed++;
                }
            }
        }
        log.Count("Street intersection width steps normalized",changed);
        log.Evidence($"Street widths: {changed} short widening steps normalized; matching-body pockets up to 180 m, terminal pockets up to 60 m; junction identities and travel directions retained; major-road approaches protected.");
        return result??source;
    }

    public static IReadOnlyList<PreparedSegment> Smooth(IReadOnlyList<PreparedSegment> source, ImportLog log)
    {
        var result = source.ToArray();
        var starts = new Dictionary<SourceNodeKey, List<int>>(); var ends = new Dictionary<SourceNodeKey, List<int>>();
        for (var i = 0; i < source.Count; i++)
        {
            var s = source[i]; if (s.Kind != NetworkKind.Road) continue;
            Add(starts, new(s.Kind, s.StartNodeId), i); Add(ends, new(s.Kind, s.EndNodeId), i);
        }
        static void Add(Dictionary<SourceNodeKey, List<int>> map, SourceNodeKey key, int i) { if (!map.TryGetValue(key, out var list)) map[key] = list = new(); list.Add(i); }
        double2 Heading(PreparedSegment s) => math.normalizesafe((double2)(s.End - s.Start).xz);
        bool Same(PreparedSegment a, PreparedSegment b) => a.Classification == b.Classification && a.OneWay == b.OneWay &&
            (a.Name.Length > 0 ? a.Name == b.Name : a.Reference.Length > 0 ? a.Reference == b.Reference : b.Name.Length == 0 && b.Reference.Length == 0);
        // Forward continuation only (end node → a section starting there), so
        // two-way roads digitised against each other simply form shorter roads.
        int Next(int i)
        {
            var s = source[i]; if (!starts.TryGetValue(new(s.Kind, s.EndNodeId), out var list)) return -1;
            var found = -1;
            foreach (var j in list)
            {
                if (j == i || !Same(s, source[j]) || math.dot(Heading(s), Heading(source[j])) < .7) continue;
                if (found >= 0) return -1; found = j;
            }
            return found;
        }
        var next = Enumerable.Range(0, source.Count).Select(i => source[i].Kind == NetworkKind.Road ? Next(i) : -1).ToArray();
        var hasPrevious = new bool[source.Count]; foreach (var j in next) if (j >= 0) hasPrevious[j] = true;
        var used = new bool[source.Count]; var changed = 0; var roads = 0;
        // Roads start where nothing continues into them; loops start anywhere.
        foreach (var first in Enumerable.Range(0, source.Count).OrderBy(i => hasPrevious[i] ? 1 : 0))
        {
            if (used[first] || source[first].Kind != NetworkKind.Road) continue;
            var chain = new List<int>();
            for (var i = first; i >= 0 && !used[i]; i = next[i]) { used[i] = true; chain.Add(i); }
            if (chain.Count < 3) continue;
            var length = chain.Select(i => (double)math.distance(source[i].Start.xz, source[i].End.xz)).ToArray();
            var lanes = chain.Select(i => result[i].Lanes).ToArray();
            var touched = false;
            while (true)
            {
                // Runs of equal lane count: (first index, last index, length).
                var runs = new List<(int From, int To, double Length)>();
                for (var k = 0; k < chain.Count;)
                {
                    var e = k; var sum = length[k]; while (e + 1 < chain.Count && lanes[e + 1] == lanes[k]) sum += length[++e];
                    runs.Add((k, e, sum)); k = e + 1;
                }
                var pick = -1; var target = 0;
                for (var r = 0; r < runs.Count; r++)
                {
                    var (from, to, run) = runs[r];
                    int? before = r > 0 ? lanes[runs[r - 1].To] : null; int? after = r + 1 < runs.Count ? lanes[runs[r + 1].From] : null;
                    int? replacement = before.HasValue && after.HasValue ? before == after && run < ShortRun ? before : null :
                        (before ?? after).HasValue && run < ShortRun / 2 && runs.Count > 1 ? before ?? after : null;
                    if (replacement.HasValue && (pick < 0 || run < runs[pick].Length)) { pick = r; target = replacement.Value; }
                }
                if (pick < 0) break;
                for (var k = runs[pick].From; k <= runs[pick].To; k++) lanes[k] = target;
                touched = true;
            }
            if (!touched) continue;
            roads++;
            for (var k = 0; k < chain.Count; k++)
            {
                var s = result[chain[k]]; if (s.Lanes == lanes[k]) continue;
                // Take the whole lane description of a neighbour with that count.
                var donor = chain.Select(i => source[i]).FirstOrDefault(d => d.Lanes == lanes[k]) ?? s;
                result[chain[k]] = s with { Lanes = lanes[k], ForwardLanes = donor.ForwardLanes, BackwardLanes = donor.BackwardLanes, BothWaysLanes = donor.BothWaysLanes };
                changed++;
            }
        }
        log.Count("Lane count steps smoothed", changed);
        log.Evidence($"Lane counts: {changed:N0} sections on {roads:N0} roads took their neighbours' lane count (runs under {ShortRun:0} m between matching counts, or under {ShortRun / 2:0} m at a road's end).");
        return changed == 0 ? source : result;
    }
}
