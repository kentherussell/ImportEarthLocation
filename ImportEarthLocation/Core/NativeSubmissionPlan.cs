using System;
using System.Collections.Generic;

namespace ImportEarthLocation.Core;

// Schedule by SOURCE identity, never by spatial proximity. Preserve every
// audited curve/prefab/elevation. A junction's pending arms are generated
// together. Prepared unit boundaries, when present, are indivisible; the batch
// target controls packing, not an engine safety ceiling.
public sealed class NativeSubmissionPlan
{
    private readonly List<PreparedSegment> _sections;
    private readonly List<int> _unitEnds=new();
    public IReadOnlyList<PreparedSegment> Sections => _sections;
    public bool HasDesignUnits { get; }
    public int UnitCount { get; private set; }

    public NativeSubmissionPlan(IReadOnlyList<PreparedSegment> source)
    {
        HasDesignUnits=System.Linq.Enumerable.Any(source,s=>s.NativeUnitId>=0);
        var groups=new Dictionary<long,List<int>>();
        long Unit(int i)=>source[i].NativeUnitId>=0?source[i].NativeUnitId:-(long)i-1;
        for(var i=0;i<source.Count;i++)
        {var unit=Unit(i);if(!groups.TryGetValue(unit,out var list))groups[unit]=list=new();list.Add(i);}
        var incident=new Dictionary<string,List<int>>(StringComparer.Ordinal);
        for(var i=0;i<source.Count;i++)foreach(var node in new[]{source[i].StartNodeId,source[i].EndNodeId})
        {
            var key=NativeEndpointPolicy.Identity(source[i],node);
            if(!incident.TryGetValue(key,out var arms))incident[key]=arms=new List<int>();
            arms.Add(i);
        }
        _sections=new List<PreparedSegment>(source.Count);
        var queued=new bool[source.Count];var frontier=new List<int>(source.Count);var emitted=new HashSet<long>();
        for(var root=0;root<source.Count;root++)
        {
            if(queued[root])continue;
            frontier.Clear();frontier.Add(root);queued[root]=true;
            for(var j=0;j<frontier.Count;j++)
            {
                var i=frontier[j];var s=source[i];var unit=Unit(i);
                if(emitted.Add(unit))
                {
                    var members=groups[unit];var end=_sections.Count+members.Count;UnitCount++;
                    foreach(var member in members)
                    {
                        _sections.Add(source[member]);_unitEnds.Add(end);
                        if(!queued[member]){queued[member]=true;frontier.Add(member);}
                    }
                }
                foreach(var node in new[]{s.StartNodeId,s.EndNodeId})
                    foreach(var next in incident[NativeEndpointPolicy.Identity(s,node)])
                        if(!queued[next]){queued[next]=true;frontier.Add(next);}
            }
        }
    }

    public IReadOnlyList<PreparedSegment> Batch(int start,int target)
    {
        if(start<0||start>_sections.Count||target<1)throw new ArgumentOutOfRangeException();
        if(start>0&&start<_sections.Count&&_unitEnds[start-1]>start)
            throw new InvalidOperationException("Native batch starts inside an indivisible design unit.");
        var end=start;
        while(end<_sections.Count)
        {
            var next=_unitEnds[end];if(end>start&&next-start>target)break;end=next;
            if(end-start>=target)break;
        }
        return _sections.GetRange(start,end-start);
    }

    public void ReplaceUnissued(int start,IReadOnlyList<PreparedSegment> revision)
    {
        ValidateUnissued(start,revision);
        for(var i=0;i<revision.Count;i++)_sections[start+i]=revision[i];
    }

    public void ValidateUnissued(int start,IReadOnlyList<PreparedSegment> revision)
    {
        if(start<0||start>_sections.Count-revision.Count||start>0&&_unitEnds[start-1]>start||
            revision.Count>0&&_unitEnds[start+revision.Count-1]!=start+revision.Count)
            throw new InvalidOperationException("A local revision must replace complete unissued units.");
        for(var i=0;i<revision.Count;i++)
        {
            var old=_sections[start+i];var next=revision[i];
            if(old.Kind!=next.Kind||old.WayId!=next.WayId||old.StartNodeId!=next.StartNodeId||old.EndNodeId!=next.EndNodeId||old.NativeUnitId!=next.NativeUnitId)
                throw new InvalidOperationException("A local revision cannot change unit coverage or source ownership.");
        }
    }

    // Recovery changes the batch's junction context without editing geometry,
    // removing a section, or replaying an already-committed edge. Reorder only
    // the uncommitted suffix and only if an additional incident arm exists.
    public int GatherPendingJunction(int start,int count,int maximum,IReadOnlyList<string> sources)
    {
        // Atomic design units already contain their complete cores. The legacy
        // arm-gathering recovery may not cut or reorder those unit boundaries.
        if(HasDesignUnits)return 0;
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

    public List<IReadOnlyList<PreparedSegment>> RiskPreflight(int maximum)
    {
        var result=new List<IReadOnlyList<PreparedSegment>>();
        if(_sections.Count==0)return result;
        result.Add(_sections.GetRange(0,Math.Min(maximum,_sections.Count)));
        var incident=new Dictionary<string,List<PreparedSegment>>(StringComparer.Ordinal);
        foreach(var section in _sections)foreach(var id in new[]{section.StartNodeId,section.EndNodeId})
        {
            var key=NativeEndpointPolicy.Identity(section,id);
            if(!incident.TryGetValue(key,out var arms))incident[key]=arms=new();arms.Add(section);
        }
        var ranked=new List<KeyValuePair<string,List<PreparedSegment>>>(incident);
        ranked.Sort((a,b)=>Score(b.Value).CompareTo(Score(a.Value)));
        int Score(List<PreparedSegment> arms)
        {
            var score=arms.Count*4;
            foreach(var s in arms){if(s.Bridge||s.Tunnel)score+=3;if(Unity.Mathematics.math.distance(s.Start,s.End)<24)score+=4;if(s.StartNodeId<0||s.EndNodeId<0)score+=2;}
            return score;
        }
        var covered=new HashSet<PreparedSegment>(result[0]);
        foreach(var pair in ranked)
        {
            if(result.Count>=9)break;
            if(pair.Value.Count>maximum||!System.Linq.Enumerable.Any(pair.Value,s=>!covered.Contains(s)))continue;
            result.Add(pair.Value.ToArray());foreach(var s in pair.Value)covered.Add(s);
        }
        return result;
    }
}
