using ImportEarthLocation.Core;
using Unity.Mathematics;

// Invocation-owned offline evidence. Way membership follows merged fragments;
// it does not assert an exact correspondence between source curve parameters.
sealed class NetworkWriterTrace
{
    private readonly HashSet<long> _ways;
    public List<object> Stages { get; }=new();
    public NetworkWriterTrace(IEnumerable<long> ways)
    {
        _ways=ways.ToHashSet();
        if(_ways.Count==0||_ways.Count>32)throw new ArgumentException("Select between one and 32 exact source way IDs");
    }
    public void Observe(string stage,IReadOnlyList<PreparedSegment> sections)
    {
        var observed=new HashSet<long>();
        double[][] Points(params float3[] values)=>values.Select(p=>new[]{(double)p.x,p.y,p.z}).ToArray();
        var members=new List<object>();
        for(var i=0;i<sections.Count;i++)
        {
            var s=sections[i];var matches=s.SourceWayIds.Append(s.WayId).Where(_ways.Contains).Distinct().OrderBy(v=>v).ToArray();
            if(matches.Length==0)continue;
            observed.UnionWith(matches);
            members.Add(new{ordinal=i,s.Kind,s.WayId,s.SourceWayIds,s.StartNodeId,s.EndNodeId,s.SourceNodeAliases,
                selectedSourceWays=matches,s.PairedProfileId,s.PairedReferenceBegin,s.PairedReferenceEnd,
                s.RailTerminalFormationId,s.NativeAssetKey,s.NativeCenterlineCurvatureLimit,
                anchors=Points(s.Start,s.End),offsets=Points(s.StartCurveOffset,s.EndCurveOffset),
                course=Points(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd),
                hasFinalDeckCurve=s.FinalDeckCurve.HasValue,hasHorizontalParent=s.HorizontalParent!=null});
        }
        Stages.Add(new{stage,selectedWays=_ways.OrderBy(v=>v).ToArray(),
            absentSelectedWays=_ways.Except(observed).OrderBy(v=>v).ToArray(),members,
            coverage="All matching fragments at this observation boundary; no clipping; source way membership only",
            absentIsNotPassed=true});
    }
    public static void Test()
    {
        var s=new PreparedSegment(1,10,11,new(0,2,0),new(10,2,0),NetworkKind.Road,"primary",2,true,false,false,0)
            {SourceWayIds=new[]{2L},Control1=new(3,2,0),Control2=new(7,2,0)};
        var trace=new NetworkWriterTrace(new[]{2L,3L});trace.Observe("before",new[]{s});
        trace.Observe("after",new[]{s with{Control1=new(3,2,1)}});
        var json=Newtonsoft.Json.Linq.JArray.FromObject(trace.Stages);
        if(json[0]["members"]!.Count()!=1||(long)json[0]["absentSelectedWays"]![0]!=3||
            (double)json[0]["members"]![0]!["course"]![1]![2]!=0||
            (double)json[1]["members"]![0]!["course"]![1]![2]!=1)
            throw new Exception("Writer trace lost source membership, unavailable selection or immutable stage controls");
        Console.WriteLine("PASS: selected-source writer snapshots retain primitive controls, merged membership and explicit absence");
    }
}
