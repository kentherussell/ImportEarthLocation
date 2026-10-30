using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

public static class NetworkContinuity
{
    public static void Validate(IReadOnlyList<PreparedSegment> before,IReadOnlyList<PreparedSegment> after,ImportLog log,CancellationToken token,bool allowQualityWarnings=false)
    {
        var graph=new Dictionary<(NetworkKind,long),List<long>>();var positions=new Dictionary<(NetworkKind,long),float3>();var conflicts=0;var lost=0;var overlaps=0;
        foreach(var s in after)
        {
            token.ThrowIfCancellationRequested();
            foreach(var end in new[]{(id:s.StartNodeId,next:s.EndNodeId,point:s.Start),(id:s.EndNodeId,next:s.StartNodeId,point:s.End)})
            {
                var key=(s.Kind,end.id);
                if(positions.TryGetValue(key,out var p)&&math.distance(p,end.point)>.02f)
                {conflicts++;if(conflicts<=25)log.Warn($"Network portal/junction {s.Kind}:{end.id} has conflicting positions; {(allowQualityWarnings?"RequiresReview; safe placement continues":"strict validation fails")}.");}
                positions[key]=end.point;if(!graph.TryGetValue(key,out var list))graph[key]=list=new List<long>();list.Add(end.next);
            }
        }
        // Label components once; no repeated searches for every source section.
        var components=new Dictionary<(NetworkKind,long),int>();var index=0;
        foreach(var key in graph.Keys)
        {
            if(components.ContainsKey(key))continue;index++;var queue=new List<(NetworkKind,long)>{key};components[key]=index;
            for(var cursor=0;cursor<queue.Count;cursor++){token.ThrowIfCancellationRequested();var node=queue[cursor];foreach(var id in graph[node]){var next=(node.Item1,id);if(components.ContainsKey(next))continue;components[next]=index;queue.Add(next);}}
        }
        // A source node may survive as a declared alias, or be absorbed into a
        // longer section that still carries its way; only a way with no
        // surviving section, or two surviving ends in different pieces, is lost.
        var aliases=new Dictionary<(NetworkKind,long),long>();var ways=new HashSet<(NetworkKind,long)>();
        foreach(var s in after)
        {
            foreach(var alias in s.SourceNodeAliases)aliases[(s.Kind,alias.SourceNodeId)]=alias.CanonicalNodeId;
            ways.Add((s.Kind,s.WayId));foreach(var way in s.SourceWayIds)ways.Add((s.Kind,way));
        }
        int? Component(NetworkKind kind,long id)
        {
            if(components.TryGetValue((kind,id),out var c))return c;
            return aliases.TryGetValue((kind,id),out var canonical)&&components.TryGetValue((kind,canonical),out c)?c:null;
        }
        foreach(var s in before)
        {
            token.ThrowIfCancellationRequested();
            var a=Component(s.Kind,s.StartNodeId);var b=Component(s.Kind,s.EndNodeId);
            if(a.HasValue&&b.HasValue?a!=b:!ways.Contains((s.Kind,s.WayId)))
            {lost++;if(lost<=25)log.Warn($"Network continuity not verified for {s.Kind} way {s.WayId}; {(allowQualityWarnings?"RequiresReview; safe placement continues":"strict validation fails")}.");}
        }
        var samples=new Dictionary<(int,int),List<(int edge,float3 point,float2 heading)>>();var rails=after.Where(s=>s.Kind==NetworkKind.Rail).ToArray();
        for(var edge=0;edge<rails.Length;edge++)
        {
            token.ThrowIfCancellationRequested();var s=rails[edge];var steps=Math.Max(2,(int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(s)/4));
            for(var i=1;i<steps;i++)
            {
                var t=i/(float)steps;var p=NetworkNormalizer.Evaluate(s,t);var heading=math.normalizesafe((NetworkNormalizer.Evaluate(s,Math.Min(1,t+.001f))-NetworkNormalizer.Evaluate(s,Math.Max(0,t-.001f))).xz);
                var x=(int)Math.Floor(p.x/4);var z=(int)Math.Floor(p.z/4);
                for(var dz=-1;dz<=1;dz++)for(var dx=-1;dx<=1;dx++)if(samples.TryGetValue((x+dx,z+dz),out var candidates))foreach(var other in candidates)
                {
                    if(other.edge==edge)continue;
                    var rail=rails[other.edge];if(rail.Bridge!=s.Bridge||rail.Tunnel!=s.Tunnel)continue;
                    // Two covered/elevated routes can be parallel in plan but
                    // occupy safely separated levels. That is not duplicate rail.
                    if(Math.Abs(p.y-other.point.y)>=NetworkVerticalProfiles.MinimumBridgeClearance+2)continue;
                    if(math.distance(p.xz,other.point.xz)>=3||Math.Abs(math.dot(heading,other.heading))<.97f)continue;
                    // A branch may approach its shared junction. An extended parallel
                    // overlap away from that junction must not pass as an intersection.
                    var shared=new[]{s.StartNodeId,s.EndNodeId}.Intersect(new[]{rail.StartNodeId,rail.EndNodeId}).ToArray();
                    if(shared.Any(id=>math.distance(p.xz,positions[(NetworkKind.Rail,id)].xz)<12))continue;
                    overlaps++;if(overlaps<=25)log.Warn($"Rail overlap remains for ways {s.WayId}/{rail.WayId}; {(allowQualityWarnings?"iteration quality warning":"application blocked")}.");
                }
                if(!samples.TryGetValue((x,z),out var bucket))samples[(x,z)]=bucket=new List<(int,float3,float2)>();bucket.Add((edge,p,heading));
            }
        }
        log.Count("Network continuity unresolved sections",lost);log.Count("Network junction position conflicts",conflicts);log.Count("Rail overlap observations",overlaps);
        log.Diagnostic($"PROCESS continuity checked; verified={conflicts+lost+overlaps==0}; sourceSections={before.Count}; finalSections={after.Count}; components={index}; sharedPortalConflicts={conflicts}; unresolvedSections={lost}; railOverlapObservations={overlaps}; failClosed={!allowQualityWarnings}; topologyFailClosed={!allowQualityWarnings}; nativeAcceptanceNotYetTested=true");
        if(conflicts+lost>0){log.Count("Network repair quality observations",conflicts+lost);log.Warn($"Network continuity observations: {conflicts} conflicting junctions, {lost} unresolved sections. Generation continues for visual diagnosis.");}
        if(overlaps>0)
        {log.Count("Network repair quality observations",overlaps);log.Warn($"Iteration retains {overlaps} rail overlap observations for testing. Known endpoint connections remain intact.");}
        // Explicit strict offline validation remains available. Production
        // callers opt into inspectable RequiresReview geometry; they do not
        // abort an otherwise native-safe import for geometric observations.
        if(!allowQualityWarnings&&conflicts+lost+overlaps>0)
            throw new InvalidOperationException("Strict continuity validation failed: geometric RequiresReview observations remain.");
    }
}
