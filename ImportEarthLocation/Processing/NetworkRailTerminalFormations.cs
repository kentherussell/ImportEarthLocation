using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// A terminal fan is a functional contraction, not a proximity weld. Only a
// connected undirected rail interior is changed. Exterior ports retain their
// identities and modes; several simple turnouts replace a multi-arm fan.
public static class NetworkRailTerminalFormations
{
    public static IReadOnlyList<PreparedSegment> Prepare(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token)
    {
        var incident=new Dictionary<long,List<int>>();var points=new Dictionary<long,float3>();
        var protectedNodes=new HashSet<long>(source.Where(s=>s.Kind==NetworkKind.Road).SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}));
        for(var i=0;i<source.Count;i++)if(source[i].Kind==NetworkKind.Rail)
            foreach(var entry in new[]{(source[i].StartNodeId,source[i].Start),(source[i].EndNodeId,source[i].End)})
            {if(!incident.TryGetValue(entry.Item1,out var list))incident[entry.Item1]=list=new();list.Add(i);points[entry.Item1]=entry.Item2;}
        var terminals=incident.Where(p=>p.Value.Count==1&&!protectedNodes.Contains(p.Key)).Select(p=>p.Key).OrderBy(x=>x).ToArray();
        var grid=new Dictionary<(int,int),List<long>>();
        (int,int) Cell(float3 p)=>((int)Math.Floor(p.x/250),(int)Math.Floor(p.z/250));
        foreach(var id in terminals){var key=Cell(points[id]);if(!grid.TryGetValue(key,out var list))grid[key]=list=new();list.Add(id);}
        var consumed=new HashSet<int>();var examinedTerminals=new HashSet<long>();var remap=new Dictionary<long,long>();var additions=new List<PreparedSegment>();var records=new List<object>();
        var checkedFans=0;var accepted=0;var unavailable=0;
        foreach(var seed in terminals)
        {
            token.ThrowIfCancellationRequested();if(examinedTerminals.Contains(seed)||consumed.Contains(incident[seed][0]))continue;
            var first=source[incident[seed][0]];if(first.OneWay)continue;
            var cell=Cell(points[seed]);var nearby=new HashSet<long>();
            for(var x=-1;x<=1;x++)for(var z=-1;z<=1;z++)if(grid.TryGetValue((cell.Item1+x,cell.Item2+z),out var list))
                foreach(var id in list)if(math.distance(points[id].xz,points[seed].xz)<=250)nearby.Add(id);
            if(nearby.Count<4)continue;checkedFans++;
            string Gauge(PreparedSegment s)=>s.RawSourceTags.TryGetValue("gauge",out var gauge)?gauge:s.SourceTags.TryGetValue("gauge",out gauge)?gauge:"";
            bool Compatible(int i)=>!consumed.Contains(i)&&!source[i].OneWay&&source[i].Layer==first.Layer&&source[i].Bridge==first.Bridge&&
                source[i].Tunnel==first.Tunnel&&Gauge(source[i])==Gauge(first);
            var distance=new Dictionary<long,double>{{seed,0}};var queue=new List<long>{seed};var head=0;
            while(head<queue.Count)
            {
                token.ThrowIfCancellationRequested();var node=queue[head++];if(protectedNodes.Contains(node))continue;
                foreach(var i in incident[node])
                {
                    if(!Compatible(i))continue;var s=source[i];var other=s.StartNodeId==node?s.EndNodeId:s.StartNodeId;
                    var next=distance[node]+math.distance(s.Start.xz,s.End.xz);
                    if(next>2000||distance.TryGetValue(other,out var old)&&old<=next)continue;
                    distance[other]=next;queue.Add(other);
                }
            }
            var fanEnds=nearby.Where(distance.ContainsKey).ToArray();if(fanEnds.Length<4){unavailable++;continue;}
            examinedTerminals.UnionWith(fanEnds);
            var inside=new HashSet<int>(distance.Keys.SelectMany(n=>incident[n]).Where(i=>Compatible(i)&&distance.ContainsKey(source[i].StartNodeId)&&distance.ContainsKey(source[i].EndNodeId)));
            var gates=distance.Keys.Where(n=>protectedNodes.Contains(n)||incident[n].Any(i=>!inside.Contains(i))||incident[n].Count==1&&!nearby.Contains(n)).ToArray();
            // Incompatible mode/gauge and at-grade road contacts are independent
            // ports, never members of a merged exterior group.
            if(gates.Any(n=>protectedNodes.Contains(n)))
            {
                unavailable++;records.Add(new{seed,decision="retained-source",reason="protected-or-incompatible-exterior-port",
                    gates=gates.Select(n=>new{node=n,point=points[n],modes=incident[n].Select(i=>new{source[i].WayId,source[i].Layer,source[i].Bridge,source[i].Tunnel}).ToArray()}).ToArray()});continue;
            }
            var groups=new List<List<long>>();
            foreach(var gate in gates.OrderBy(n=>n))
            {
                var group=groups.FirstOrDefault(g=>g.All(n=>math.distance(points[n].xz,points[gate].xz)<=48));
                if(group==null)groups.Add(new(){gate});else group.Add(gate);
            }
            if(groups.Count<2)
            {
                unavailable++;records.Add(new{seed,decision="retained-source",reason="exterior-group-count-not-two",exteriorGroups=groups,
                    connectedTerminalCount=fanEnds.Length,tracebackLimitMeters=2000});continue;
            }
            // All group members must have matching exterior directions, not
            // merely nearby coordinates at two unrelated crossings.
            float2 Exterior(long node)
            {
                var edge=incident[node].Where(i=>!inside.Contains(i)).DefaultIfEmpty(-1).First();
                if(edge>=0){var s=source[edge];return math.normalizesafe((s.StartNodeId==node?s.End:s.Start).xz-points[node].xz);}
                // A retained interior spur is also a functional port. Its
                // approach points outward along its last original rail edge.
                var interior=source[incident[node][0]];
                return math.normalizesafe(points[node].xz-(interior.StartNodeId==node?interior.End:interior.Start).xz);
            }
            // Never combine unrelated routes just because they are nearby.
            // Only the original two-gate case can merge aligned parallel ports.
            if(groups.Count!=2||groups.Any(g=>g.Any(n=>incident[n].Count(i=>!inside.Contains(i))!=1||
                math.lengthsq(Exterior(n))<.5||math.dot(Exterior(n),Exterior(g[0]))<Math.Cos(Math.PI/12))))
                groups=gates.OrderBy(n=>n).Select(n=>new List<long>{n}).ToList();
            var terminal=fanEnds.Min();var terminalPoint=fanEnds.Select(n=>points[n]).Aggregate(float3.zero,(p,q)=>p+q)/fanEnds.Length;
            var gatePoints=groups.Select(g=>g.Select(n=>points[n]).Aggregate(float3.zero,(p,q)=>p+q)/g.Count).ToArray();
            if(groups.Where((g,k)=>g.Any(n=>math.distance(points[n].xz,gatePoints[k].xz)>24)).Any()){unavailable++;continue;}
            var mean=gatePoints.Aggregate(float3.zero,(p,q)=>p+q)/gatePoints.Length;var junctionPoint=math.lerp(mean,terminalPoint,.35f);
            var axis=math.normalizesafe(terminalPoint.xz-junctionPoint.xz);
            if(math.distance(terminalPoint.xz,junctionPoint.xz)<80||math.lengthsq(axis)<.5){unavailable++;continue;}
            var internalNodes=inside.SelectMany(i=>new[]{source[i].StartNodeId,source[i].EndNodeId}).Distinct().ToArray();
            var junctionIds=internalNodes.Where(n=>!gates.Contains(n)&&!fanEnds.Contains(n)).OrderBy(n=>n).ToArray();
            var junctionCount=Math.Max(1,groups.Count-1);
            if(junctionIds.Length<junctionCount){unavailable++;continue;}
            var junction=junctionIds[0];
            var ways=inside.SelectMany(i=>source[i].SourceWayIds.Append(source[i].WayId)).Distinct().OrderBy(x=>x).ToArray();
            var template=inside.Select(i=>source[i]).Where(s=>s.Lanes==2&&(s.PhysicalTracks==0||s.PhysicalTracks==2)&&s.NativeWidth>0&&
                s.NativeCenterlineCurvatureLimit.HasValue&&SparseProfileRows.Finite(s.NativeCenterlineCurvatureLimit.Value)&&s.NativeCenterlineCurvatureLimit.Value>0)
                .OrderBy(s=>s.RailService is "yard" or "siding" or "spur"?1:0).ThenBy(s=>s.WayId).FirstOrDefault();
            if(template==null)
            {
                unavailable++;records.Add(new{seed,decision="retained-source",reason="native-two-track-curvature-contract-unavailable"});continue;
            }
            PreparedSegment Span(long a,long b,float3 p,float3 q,float2 ta,float2 tb)
            {
                var length=math.distance(p.xz,q.xz);var c1=p+new float3(ta.x,0,ta.y)*(length/3);var c2=q-new float3(tb.x,0,tb.y)*(length/3);
                c1.y=math.lerp(p.y,q.y,1f/3);c2.y=math.lerp(p.y,q.y,2f/3);
                var geometry=Enumerable.Range(0,33).Select(j=>(float3)NetworkDesignChecks.Evaluate(new(p,c1,c2,q),j/32d)).ToArray();
                return template with{StartNodeId=a,EndNodeId=b,Start=p,End=q,Control1=c1,Control2=c2,Lanes=2,PhysicalTracks=2,OneWay=false,
                    SourceWayIds=ways,SourceGeometry=geometry,OriginalGeometry=geometry,SourceFitToleranceMeters=24,StartTerminal=false,EndTerminal=b==terminal,
                    StartCurveOffset=default,EndCurveOffset=default,FinalDeckCurve=null,HorizontalParent=null,RailService="",RailTerminalFormationId=junction};
            }
            var proposed=new List<PreparedSegment>();
            var ordered=Enumerable.Range(0,groups.Count).OrderByDescending(k=>math.distance(gatePoints[k].xz,terminalPoint.xz)).ToArray();
            var trunkLength=math.distance(mean.xz,terminalPoint.xz)*.65f;
            if(junctionCount>1&&trunkLength/(junctionCount+1)<80){unavailable++;continue;}
            var junctionPoints=Enumerable.Range(0,junctionCount).Select(k=>math.lerp(mean,terminalPoint,
                .2f+.6f*k/junctionCount)).ToArray();
            if(junctionCount==1)junctionPoints[0]=junctionPoint;
            // The farthest two exterior routes meet at the first simple Y;
            // each subsequent route gets its own degree-three turnout.
            for(var k=0;k<ordered.Length;k++)
            {
                var group=ordered[k];var slot=Math.Max(0,k-1);
                proposed.Add(Span(groups[group].Min(),junctionIds[slot],gatePoints[group],junctionPoints[slot],-Exterior(groups[group][0]),axis));
            }
            for(var k=0;k<junctionCount-1;k++)proposed.Add(Span(junctionIds[k],junctionIds[k+1],junctionPoints[k],junctionPoints[k+1],axis,axis));
            proposed.Add(Span(junctionIds[junctionCount-1],terminal,junctionPoints[junctionCount-1],terminalPoint,axis,axis));
            if(proposed.Any(s=>NetworkDesignChecks.Certify(new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd),
                s.NativeCenterlineCurvatureLimit??double.PositiveInfinity,double.PositiveInfinity,token:token,maximumLeaves:4096).Quality!=DesignQuality.Certified))
            {unavailable++;continue;}
            additions.AddRange(proposed);
            foreach(var node in internalNodes)remap[node]=fanEnds.Contains(node)?terminal:junction;
            for(var k=0;k<junctionCount;k++)remap[junctionIds[k]]=junctionIds[k];
            foreach(var group in groups)foreach(var node in group)remap[node]=group.Min();
            foreach(var i in inside)consumed.Add(i);accepted++;
            records.Add(new{formation=junction,terminal,externalGroups=groups,sourceWays=ways,sourceSections=inside.Count,
                nodeMap=internalNodes.Select(n=>new{source=n,canonical=remap[n]}).ToArray(),policy="connected-multi-gate-terminal-contraction",nativeAcceptanceVerified=false});
        }
        if(accepted==0)
        {
            log.DiagnosticRecordChunks("rail-terminal-contractions.ndjson",records);
            log.Quality(new("planned","rail-terminal-contraction","source",0,0,unavailable,0,0,"No ambiguous terminal fan is silently simplified"){Expected=checkedFans});return source;
        }
        var output=source.Where((s,i)=>!consumed.Contains(i)).Select(s=>s.Kind!=NetworkKind.Rail?s:s with{
            StartNodeId=remap.TryGetValue(s.StartNodeId,out var a)?a:s.StartNodeId,EndNodeId=remap.TryGetValue(s.EndNodeId,out var b)?b:s.EndNodeId}).Concat(additions).ToArray();
        // Keep the external source port coordinates: the curve offset, not an
        // unrelated node identity, carries an allowed approach displacement.
        var anchors=output.Where(s=>s.Kind==NetworkKind.Rail).SelectMany(s=>new[]{(s.StartNodeId,s.Start),(s.EndNodeId,s.End)})
            .GroupBy(p=>p.Item1).ToDictionary(g=>g.Key,g=>g.First().Item2);
        output=output.Select(s=>s.Kind!=NetworkKind.Rail?s:s with{Start=anchors[s.StartNodeId],End=anchors[s.EndNodeId],
            Control1=(s.Control1??math.lerp(s.Start,s.End,1f/3))-(s.Start-anchors[s.StartNodeId]),
            Control2=(s.Control2??math.lerp(s.Start,s.End,2f/3))-(s.End-anchors[s.EndNodeId]),
            StartCurveOffset=s.Start-anchors[s.StartNodeId],EndCurveOffset=s.End-anchors[s.EndNodeId]}).ToArray();
        log.DiagnosticRecordChunks("rail-terminal-contractions.ndjson",records);log.Count("Functional rail terminal formations",accepted);log.Count("Rail terminal micrograph sections removed",consumed.Count);
        log.Quality(new("planned","rail-terminal-contraction","source",accepted,0,unavailable,0,0,"All exterior routes retained through simple turnouts; original internal ports retained as intentional aliases"){Expected=checkedFans});
        return SourceTopologyObligations.Transfer(source,output,NetworkKind.Rail,remap);
    }
}
