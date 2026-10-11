using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;

namespace ImportEarthLocation.Processing;

public static class ConnectedRoadGrades
{
    // Shared-node constraint projection: a repair can move a bridge portal and
    // propagate its rise through adjoining sections, rather than pinning each
    // tiny structure independently. Retain every section and horizontal curve.
    public static void Repair(IReadOnlyList<PreparedSegment> roads, Dictionary<long,float> heights,
        IReadOnlyDictionary<PreparedSegment,float> lengths, ImportLog log, CancellationToken token,
        IReadOnlyDictionary<long,float>? originalHeights = null)
    {
        var before = new Dictionary<long,float>(heights);
        var bounds = heights.ToDictionary(p=>p.Key,p=>(lo:float.NegativeInfinity,hi:float.PositiveInfinity));
        foreach(var road in roads)
        {
            var budget=NetworkVerticalProfiles.CutFillBudget(road);
            foreach(var id in new[]{road.StartNodeId,road.EndNodeId})
            {
                var bound=bounds[id];
                // Keep tunnel depths fixed. Do not lower an existing bridge deck
                // to obtain a nicer grade at the expense of crossing clearance.
                var baseline=originalHeights!=null?originalHeights[id]:before[id];
                var lo=road.Tunnel?before[id]:road.Bridge?before[id]:Math.Min(before[id],baseline-budget);
                var hi=road.Tunnel?before[id]:Math.Max(before[id],baseline+budget);
                bounds[id]=(Math.Max(bound.lo,lo),Math.Min(bound.hi,hi));
            }
        }
        var ordered=roads.OrderBy(s=>lengths[s]).ThenBy(s=>s.WayId).ThenBy(s=>s.StartNodeId).ToArray();
        int Violations()=>ordered.Count(s=>Math.Abs(heights[s.EndNodeId]-heights[s.StartNodeId])>lengths[s]*NetworkVerticalProfiles.DesiredGrade(s)+.01f);
        var initial=Violations();var clock=System.Diagnostics.Stopwatch.StartNew();var nextProgress=5d;var passes=0;
        // Bounded alternating projections operate over actual topology, including
        // branches and structure approaches; proximity never joins unrelated roads.
        for(;passes<128;passes++)
        {
            var largest=0f;
            foreach(var s in ordered)
            {
                token.ThrowIfCancellationRequested();
                var a=s.StartNodeId;var b=s.EndNodeId;if(a==b)continue;
                var delta=heights[b]-heights[a];var excess=Math.Abs(delta)-lengths[s]*NetworkVerticalProfiles.DesiredGrade(s);
                if(excess<=.001f)continue;
                var sign=delta>0?1f:-1f;
                var roomA=Math.Max(0,sign>0?bounds[a].hi-heights[a]:heights[a]-bounds[a].lo);
                var roomB=Math.Max(0,sign>0?heights[b]-bounds[b].lo:bounds[b].hi-heights[b]);
                var moveA=Math.Min(roomA,excess*.5f);var moveB=Math.Min(roomB,excess-moveA);
                moveA=Math.Min(roomA,excess-moveB);
                heights[a]+=sign*moveA;heights[b]-=sign*moveB;
                largest=Math.Max(largest,moveA+moveB);
            }
            if(clock.Elapsed.TotalSeconds>=nextProgress)
            {log.Diagnostic($"PROCESS connected-road-grades pass={passes+1}/128; elapsedSeconds={clock.Elapsed.TotalSeconds:0.0}; remainingEndpointViolations={Violations()}");nextProgress=clock.Elapsed.TotalSeconds+5;}
            if(largest<.001f){passes++;break;}
        }
        var remaining=Violations();var moved=heights.Count(p=>Math.Abs(p.Value-before[p.Key])>.01f);
        log.Count("Connected road grade nodes adjusted",moved);
        log.Count("Connected road endpoint grade violations remaining",remaining);
        log.Diagnostic($"PROCESS connected-road-grades complete; sections={roads.Count}; adjustedNodes={moved}; endpointViolationsBefore={initial}; endpointViolationsAfter={remaining}; passes={passes}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.0}; bridgeDecksNotLowered=true; tunnelDepthsFixed=true; horizontalGeometryUnchanged=true; sectionsDeleted=0; curveGradesRequireFinalValidation=true");
        if(remaining>0)log.Warn($"Connected road grading: {remaining} endpoint constraints remain infeasible within bounded adjustments. Roads retained; this is not a guarantee of acceptable final grades.");
    }
}
