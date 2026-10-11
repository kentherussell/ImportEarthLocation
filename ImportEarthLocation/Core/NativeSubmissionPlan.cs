using System;
using System.Collections.Generic;

namespace ImportEarthLocation.Core;

// Schedule by SOURCE identity, never by spatial proximity. Preserve every
// audited curve/prefab/elevation. A junction's pending arms are generated
// together when possible instead of attaching them far apart in file order.
public sealed class NativeSubmissionPlan
{
    private readonly List<PreparedSegment> _sections;
    public IReadOnlyList<PreparedSegment> Sections => _sections;

    public NativeSubmissionPlan(IReadOnlyList<PreparedSegment> source)
    {
        var incident=new Dictionary<string,List<int>>(StringComparer.Ordinal);
        for(var i=0;i<source.Count;i++)foreach(var node in new[]{source[i].StartNodeId,source[i].EndNodeId})
        {
            var key=NativeEndpointPolicy.Identity(source[i],node);
            if(!incident.TryGetValue(key,out var arms))incident[key]=arms=new List<int>();
            arms.Add(i);
        }
        _sections=new List<PreparedSegment>(source.Count);
        var queued=new bool[source.Count];var frontier=new List<int>(source.Count);
        for(var root=0;root<source.Count;root++)
        {
            if(queued[root])continue;
            frontier.Clear();frontier.Add(root);queued[root]=true;
            for(var j=0;j<frontier.Count;j++)
            {
                var i=frontier[j];var s=source[i];_sections.Add(s);
                foreach(var node in new[]{s.StartNodeId,s.EndNodeId})
                    foreach(var next in incident[NativeEndpointPolicy.Identity(s,node)])
                        if(!queued[next]){queued[next]=true;frontier.Add(next);}
            }
        }
    }

    // Recovery changes the batch's junction context without editing geometry,
    // removing a section, or replaying an already-committed edge. Reorder only
    // the uncommitted suffix and only if an additional incident arm exists.
    public int GatherPendingJunction(int start,int count,int maximum,IReadOnlyList<string> sources)
    {
        if(start<0||count<1||start>_sections.Count-count||maximum<count)return 0;
        var junctions=new HashSet<string>(sources,StringComparer.Ordinal);
        if(junctions.Count==0)return 0;
        var indices=new List<int>();
        for(var i=start;i<_sections.Count;i++)
        {
            var s=_sections[i];
            if(junctions.Contains(NativeEndpointPolicy.Identity(s,s.StartNodeId))||junctions.Contains(NativeEndpointPolicy.Identity(s,s.EndNodeId)))indices.Add(i);
        }
        if(indices.Count<=count||indices.Count>maximum)return 0;
        // Including unrelated current-batch members would be a blind replay.
        // Require the failing batch to be entirely contained in the repair star.
        for(var i=start;i<start+count;i++)if(!indices.Contains(i))return 0;
        var selected=new HashSet<int>(indices);var suffix=new List<PreparedSegment>(_sections.Count-start);
        foreach(var i in indices)suffix.Add(_sections[i]);
        for(var i=start;i<_sections.Count;i++)if(!selected.Contains(i))suffix.Add(_sections[i]);
        for(var i=0;i<suffix.Count;i++)_sections[start+i]=suffix[i];
        return indices.Count;
    }
}
