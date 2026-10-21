using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;

namespace ImportEarthLocation.Core;

// Selected, clipped source topology, captured BEFORE consolidation. Deliberate
// acquisition/policy omissions are outside this graph, not unexplained losses.
public static class SourceTopologyObligations
{
    public static SourceTopologyManifest Capture(IReadOnlyList<PreparedSegment> sections)
    {
        var incident=new Dictionary<SourceNodeKey,List<int>>();
        SourceNodeKey A(PreparedSegment s)=>new(s.Kind,s.StartNodeId);
        SourceNodeKey B(PreparedSegment s)=>new(s.Kind,s.EndNodeId);
        for(var i=0;i<sections.Count;i++)foreach(var key in new[]{A(sections[i]),B(sections[i])})
        {if(!incident.TryGetValue(key,out var list))incident[key]=list=new();list.Add(i);}
        var ports=new HashSet<SourceNodeKey>();
        foreach(var pair in incident)
        {
            var first=sections[pair.Value[0]];
            if(pair.Value.Count!=2||pair.Value.Any(i=>
                first.Kind==NetworkKind.Road&&sections[i].Lanes!=first.Lanes||sections[i].OneWay!=first.OneWay||sections[i].Bridge!=first.Bridge||sections[i].Tunnel!=first.Tunnel||
                sections[i].Layer!=first.Layer||sections[i].StartTerminal&&A(sections[i])==pair.Key||sections[i].EndTerminal&&B(sections[i])==pair.Key||
                sections[i].StartMiniRoundabout&&A(sections[i])==pair.Key||sections[i].EndMiniRoundabout&&B(sections[i])==pair.Key))ports.Add(pair.Key);
            else if(first.OneWay&&(A(first)==pair.Key)==(A(sections[pair.Value[1]])==pair.Key))ports.Add(pair.Key);
        }
        var visited=new bool[sections.Count];var connections=new List<SourceConnection>();
        void Trace(SourceNodeKey start,int seed)
        {
            var node=start;var index=seed;var forward=true;var backward=true;var ways=new HashSet<long>();
            while(!visited[index])
            {
                visited[index]=true;var s=sections[index];var along=A(s)==node;
                if(s.OneWay){forward&=along;backward&=!along;}
                ways.UnionWith(s.SourceWayIds.Count>0?s.SourceWayIds:new[]{s.WayId});node=along?B(s):A(s);
                if(ports.Contains(node)||node==start)break;
                var arms=incident[node];index=arms[0]==index?arms[1]:arms[0];
            }
            var provenance=Array.AsReadOnly(ways.OrderBy(w=>w).ToArray());
            if(forward)connections.Add(new(start,node,provenance));if(backward)connections.Add(new(node,start,provenance));
        }
        foreach(var key in ports.OrderBy(k=>k.Kind).ThenBy(k=>k.Id))foreach(var i in incident[key])if(!visited[i])Trace(key,i);
        for(var i=0;i<sections.Count;i++)if(!visited[i]){var seam=A(sections[i]);ports.Add(seam);Trace(seam,i);}
        return new(Array.AsReadOnly(ports.OrderBy(k=>k.Kind).ThenBy(k=>k.Id).ToArray()),Array.AsReadOnly(connections
            .OrderBy(c=>c.From.Kind).ThenBy(c=>c.From.Id).ThenBy(c=>c.To.Id).ThenBy(c=>c.Ways.FirstOrDefault()).ToArray()));
    }

    // Consume the consolidator's exact remapping, never recover aliases by
    // proximity or a repaired height. Previously declared aliases follow it.
    public static IReadOnlyList<PreparedSegment> Transfer(IReadOnlyList<PreparedSegment> before,IReadOnlyList<PreparedSegment> after,
        NetworkKind kind,IReadOnlyDictionary<long,long> remap)
    {
        if(remap.Count==0)return after;
        var required=new HashSet<long>(Capture(before).Ports.Where(k=>k.Kind==kind).Select(k=>k.Id));
        long Resolve(long id)
        {
            var seen=new HashSet<long>();
            while(remap.TryGetValue(id,out var next)&&next!=id&&seen.Add(id))id=next;
            return id;
        }
        var aliases=new Dictionary<long,SourceNodeAlias>();
        foreach(var s in before.Where(s=>s.Kind==kind))foreach(var alias in s.SourceNodeAliases)
            aliases[alias.SourceNodeId]=alias with{CanonicalNodeId=Resolve(alias.CanonicalNodeId)};
        foreach(var pair in remap)aliases[pair.Key]=new(pair.Key,Resolve(pair.Value),required.Contains(pair.Key)||aliases.TryGetValue(pair.Key,out var old)&&old.Mandatory);
        var byNode=aliases.Values.GroupBy(a=>a.CanonicalNodeId).ToDictionary(g=>g.Key,g=>g.ToArray());
        return after.Select(s=>s.Kind!=kind?s:s with{SourceNodeAliases=s.SourceNodeAliases.Select(alias=>alias with{
                // A new intentional contraction of this exact source ID
                // supersedes its old alias everywhere, including surviving
                // exterior sections. Otherwise stale targets make one source
                // port appear to name two different canonical nodes.
                CanonicalNodeId=Resolve(remap.ContainsKey(alias.SourceNodeId)?alias.SourceNodeId:alias.CanonicalNodeId)})
            .Concat(byNode.TryGetValue(s.StartNodeId,out var a)?a:Array.Empty<SourceNodeAlias>())
            .Concat(byNode.TryGetValue(s.EndNodeId,out var b)?b:Array.Empty<SourceNodeAlias>())
            .GroupBy(a=>a.SourceNodeId).Select(g=>g.Last()).OrderBy(a=>a.SourceNodeId).ToArray()}).ToArray();
    }

    public static void Validate(SourceTopologyManifest? expected,IReadOnlyList<PreparedSegment> sections,ImportLog log,CancellationToken token)
    {
        if(expected==null){log.Diagnostic("SOURCE OBLIGATIONS unavailable; selected topology snapshot missing; quality=RequiresReview");return;}
        var aliases=new Dictionary<SourceNodeKey,HashSet<long>>();
        var nodes=new HashSet<SourceNodeKey>();var byWay=new Dictionary<(NetworkKind,long),List<PreparedSegment>>();
        foreach(var s in sections)
        {
            nodes.Add(new(s.Kind,s.StartNodeId));nodes.Add(new(s.Kind,s.EndNodeId));
            foreach(var alias in s.SourceNodeAliases)
            {var key=new SourceNodeKey(s.Kind,alias.SourceNodeId);if(!aliases.TryGetValue(key,out var targets))aliases[key]=targets=new();targets.Add(alias.CanonicalNodeId);}
            foreach(var way in s.SourceWayIds.Count>0?s.SourceWayIds:new[]{s.WayId})
            {var key=(s.Kind,way);if(!byWay.TryGetValue(key,out var list))byWay[key]=list=new();list.Add(s);}
        }
        SourceNodeKey? Resolve(SourceNodeKey key)
        {
            if(aliases.TryGetValue(key,out var targets))
            {
                if(targets.Count!=1)return null;
                var canonical=new SourceNodeKey(key.Kind,targets.First());
                return nodes.Contains(canonical)?canonical:null;
            }
            return nodes.Contains(key)?key:null;
        }
        var missingPorts=expected.Ports.Count(p=>!Resolve(p).HasValue);var failed=0;var records=new List<object>();
        foreach(var connection in expected.Connections)
        {
            token.ThrowIfCancellationRequested();var a=Resolve(connection.From);var b=Resolve(connection.To);var reachable=false;
            if(a.HasValue&&b.HasValue)
            {
                var graph=new Dictionary<long,List<long>>();
                void Arc(long from,long to){if(!graph.TryGetValue(from,out var list))graph[from]=list=new();list.Add(to);}
                foreach(var s in connection.Ways.Where(w=>byWay.ContainsKey((connection.From.Kind,w))).SelectMany(w=>byWay[(connection.From.Kind,w)]).Distinct())
                {Arc(s.StartNodeId,s.EndNodeId);if(!s.OneWay)Arc(s.EndNodeId,s.StartNodeId);}
                var seen=new HashSet<long>{a.Value.Id};var pending=new List<long>{a.Value.Id};
                for(var i=0;i<pending.Count&&!seen.Contains(b.Value.Id);i++)if(graph.TryGetValue(pending[i],out var next))
                    foreach(var id in next)if(seen.Add(id))pending.Add(id);
                reachable=seen.Contains(b.Value.Id);
            }
            if(!reachable)failed++;
            if(log.DiagnosticsEnabled)records.Add(new{connection.From,connection.To,connection.Ways,canonicalFrom=a,canonicalTo=b,reachable,
                identityMapping="exact-source-aliases-only",nativeLaneAcceptanceVerified=false});
        }
        log.Count("Missing mandatory selected-source ports",missingPorts);log.Count("Unresolved selected-source reachability obligations",failed);
        log.Count("Selected-source topology validations");
        var expectedChecks=expected.Ports.Count+expected.Connections.Count;
        log.Quality(new("planned","source-topology","selected-source-obligations",expectedChecks,missingPorts+failed,0,0,0,
            "Exact source port identities and directional reachability; intentional aliases are checked obligations, not discarded failures"){Expected=expectedChecks});
        if(log.DiagnosticsEnabled)log.DiagnosticRecordChunks("source-topology-obligations.ndjson",records);
        log.Diagnostic($"SOURCE OBLIGATIONS ports={expected.Ports.Count}; missingPorts={missingPorts}; directionalConnections={expected.Connections.Count}; unresolvedConnections={failed}; provenanceRestrictedSearch=true; sourceProximityMatching=false; quality=RequiresReview; placementBlocked=false");
    }
}
