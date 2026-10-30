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
