using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

static class PairedPublicationTraceTests
{
    sealed record Port(string Stage,float3 Endpoint,float3 Handle);
    static void Check(bool value,string message){if(!value)throw new Exception(message);}
    static string? FirstRestoration(float3 original,IReadOnlyList<Port> states)
    {
        for(var i=1;i<states.Count;i++)
            if(!math.all(states[i-1].Endpoint==original)&&math.all(states[i].Endpoint==original))return states[i].Stage;
        return null;
    }
    public static void Run()
    {
        var original=new float3(1,2,3);var moved=original+new float3(0,0,5);
        Check(FirstRestoration(original,new[]{new Port("construct",moved,moved),new Port("publish",original,original),new Port("compact",original,original)})=="publish",
            "Publication trace did not attribute the first restoring writer.");
        Check(FirstRestoration(original,new[]{new Port("construct",moved,moved),new Port("publish",moved,moved)})==null,
            "Publication trace invented a restoration on an unchanged port.");
        PreparedSegment Line(long way,long a,long b,float3 start,float3 end)=>new(way,a,b,start,end,NetworkKind.Road,"motorway",2,true,false,false,0)
        {Control1=math.lerp(start,end,1f/3),Control2=math.lerp(start,end,2f/3),NativeWidth=16,CorridorId=10,Reference="Fixture",
            SourceGeometry=new[]{start,end},OriginalGeometry=new[]{start,end},SourceWayIds=new[]{way}};
        var left=Line(1,1,2,new(-200,100,4),new(200,100,4));
        var partial=Line(2,3,4,new(200,100,-4),new(0,100,-4));
        var source=new[]{left,partial};var immutable=source.Select(s=>(s.Start,s.End,s.Control1,s.Control2)).ToArray();
        var immutableEvidence=source.Select(s=>s.SourceGeometry.ToArray()).ToArray();
        var states=new List<Port>();var counts=new List<int>();
        var observed=NetworkPairedCorridors.Prepare(source,new ImportLog(),default,inspectPublication:(stage,sections)=>
        {
            var tail=sections.Single(s=>s.WayId==left.WayId&&s.StartNodeId==left.StartNodeId);
            states.Add(new(stage,tail.CourseStart,tail.CourseControl1));counts.Add(sections.Count);
        });
        Check(states.Select(s=>s.Stage).SequenceEqual(new[]{"pre-canonical","post-canonical-precompaction","postcompaction"}),
            "Paired owner omitted or reordered a publication observation boundary.");
        Check(source.Select(s=>(s.Start,s.End,s.Control1,s.Control2)).SequenceEqual(immutable),"Writer tracing changed immutable source geometry.");
        Check(source.Select((s,i)=>s.SourceGeometry.SequenceEqual(immutableEvidence[i])).All(value=>value),"Writer tracing changed original source evidence.");
        var ordinary=NetworkPairedCorridors.Prepare(source,new ImportLog(),default);
        object Geometry(PreparedSegment s)=>new{s.WayId,s.StartNodeId,s.EndNodeId,s.Start,s.End,s.Control1,s.Control2,
            s.StartCurveOffset,s.EndCurveOffset,s.PairedSpanId,s.PairedProfileId};
        Check(observed.Select(Geometry).SequenceEqual(ordinary.Select(Geometry)),"Enabling writer tracing changed the published result.");
        Check(counts.All(c=>c>0),"A nonempty partial pair produced empty writer evidence.");
        // This is an observation, not an assertion that restoring the source
        // terminal is the desired geometric contract. The existing quality
        // fixture remains responsible for the unresolved constant-datum gate.
        Console.WriteLine("PAIRED PUBLICATION TRACE firstSourceTerminalRestorer="+(FirstRestoration(left.CourseStart,states)??"none")+
            "; snapshots="+states.Count+"; sourceImmutable=true; desiredGeometryNotAsserted=true");
    }
}
