using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Axis consolidation splits at transferred source junctions. Proximity is not a switch.
public static class RailJunctionRepair
{
    public static IReadOnlyList<PreparedSegment> Prepare(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token)
    {
        var result=source.ToArray();var nodes=new Dictionary<long,List<(int index,bool start)>>();
        for(var i=0;i<result.Length;i++)if(result[i].Kind==NetworkKind.Rail)
            foreach(var start in new[]{true,false})
            {var id=start?result[i].StartNodeId:result[i].EndNodeId;if(!nodes.TryGetValue(id,out var entries))nodes[id]=entries=new();entries.Add((i,start));}
        var repaired=0;
        foreach(var entries in nodes.Values)
        {
            token.ThrowIfCancellationRequested();if(entries.Count<2)continue;
            var first=result[entries[0].index];var anchor=entries[0].start?first.Start:first.End;
            foreach(var e in entries)
            {
                var s=result[e.index];var old=e.start?s.Start:s.End;
                if(math.distance(old,anchor)<.001f)continue;
                result[e.index]=e.start?s with{Start=anchor}:s with{End=anchor};repaired++;
            }
        }
        log.Diagnostic($"RAIL SWITCH REPAIR canonicalEndpoints={repaired}; sourceNodeJunctions={nodes.Count(p=>p.Value.Count>2)}; proximityInventedConnections=0; validTerminalsPreserved=true");
        return result;
    }
}
