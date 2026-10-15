using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Topology -> horizontal shape -> lane ports -> one vertical solve. The caller
// builds terrain support afterwards; validators must not rewrite this design.
public static class CoordinatedNetworkDesign
{
    private readonly record struct Key(NetworkKind Kind,long Node);
    private sealed class Route
    {
        public readonly List<int> Edges=new();
        public readonly List<Key> Nodes=new();
        public readonly List<float> Station=new();
    }
    private static bool IsMainline(PreparedSegment s)=>s.Kind==NetworkKind.Rail||s.Classification is "motorway" or "trunk";
    private static bool Same(PreparedSegment a,PreparedSegment b)=>a.Kind==b.Kind&&a.OneWay==b.OneWay&&a.ProtectedLink==b.ProtectedLink&&
        (a.Kind==NetworkKind.Rail||a.WayId==b.WayId||a.Reference.Length>0&&a.Reference==b.Reference||a.Name.Length>0&&a.Name==b.Name||a.CorridorId!=0&&a.CorridorId==b.CorridorId);

    public static IReadOnlyList<PreparedSegment> TunnelIntervals(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token)
    {
        var result=source.ToArray();var nodes=new Dictionary<(NetworkKind,long),List<int>>();
        for(var i=0;i<result.Length;i++)foreach(var id in new[]{result[i].StartNodeId,result[i].EndNodeId})
        {var key=(result[i].Kind,id);if(!nodes.TryGetValue(key,out var list))nodes[key]=list=new();list.Add(i);}
        var filled=0;var visited=new HashSet<int>();
        for(var seed=0;seed<result.Length;seed++)
        {
            var first=result[seed];if(first.Tunnel||first.Bridge||first.ProtectedLink||!IsMainline(first)||visited.Contains(seed))continue;
            token.ThrowIfCancellationRequested();var run=new HashSet<int>{seed};var boundary=new List<int>();var length=NetworkVerticalProfiles.ArcLength(first);
            foreach(var end in new[]{first.StartNodeId,first.EndNodeId})
            {
                var current=seed;var node=end;
                while(length<=220)
                {
                    var options=nodes[(first.Kind,node)].Where(i=>i!=current&&!result[i].ProtectedLink&&result[i].OneWay==first.OneWay&&result[i].Classification==first.Classification).ToArray();
                    if(options.Length!=1)break;var next=options[0];if(run.Contains(next))break;
                    if(result[next].Tunnel){boundary.Add(next);break;}if(result[next].Bridge)break;
                    run.Add(next);length+=NetworkVerticalProfiles.ArcLength(result[next]);current=next;node=result[next].StartNodeId==node?result[next].EndNodeId:result[next].StartNodeId;
                }
            }
            visited.UnionWith(run);if(length>220||boundary.Count!=2)continue;
            foreach(var index in run){result[index]=result[index] with{Tunnel=true,Bridge=false,Layer=Math.Min(result[boundary[0]].Layer,result[boundary[1]].Layer)};filled++;}
        }
        var underground=new HashSet<long>(result.Where(s=>s.Kind==NetworkKind.Road&&s.Tunnel).SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}));
        var excluded=new HashSet<int>();var pending=new List<int>();
        for(var i=0;i<result.Length;i++)
            if(result[i].ProtectedLink&&(underground.Contains(result[i].StartNodeId)||underground.Contains(result[i].EndNodeId)))
                if(excluded.Add(i))pending.Add(i);
        for(var k=0;k<pending.Count;k++)
        {
            token.ThrowIfCancellationRequested();var current=result[pending[k]];
            foreach(var id in new[]{current.StartNodeId,current.EndNodeId})
            {
                var incident=nodes[(NetworkKind.Road,id)];if(incident.Count!=2)continue;
                foreach(var next in incident)if(result[next].ProtectedLink&&excluded.Add(next))pending.Add(next);
            }
        }
        log.Count("Short tunnel surface interruptions absorbed",filled);log.Count("Underground degree-two link sections intentionally omitted",excluded.Count);
        log.Diagnostic($"TUNNEL INTERVALS surfaceSectionsAbsorbed={filled}; maximumInterruptionMeters=220; undergroundLinkSectionsExcluded={excluded.Count}; branchesStopOmission=true; geographicOverrides=false");
        return result.Where((s,i)=>!excluded.Contains(i)).ToArray();
    }

    public static IReadOnlyList<PreparedSegment> Prepare(IReadOnlyList<PreparedSegment> source,ushort[] raster,float scale,double width,
        ImportLog log,CancellationToken token,Func<float3,float?>? water=null)
    {
        var edges=TunnelIntervals(source,log,token).ToArray();
        if(water!=null)for(var i=0;i<edges.Length;i++)
        {
            var s=edges[i];if(s.Tunnel||s.Bridge)continue;
            for(var j=0;j<=8;j++)
            {var p=NetworkNormalizer.Evaluate(s,j/8f);var stage=water(p);if(!stage.HasValue||!math.isfinite(stage.Value)||NetworkProcessor.Sample(p.x,p.z,raster,scale,width)>stage.Value+.5f)continue;edges[i]=s with{Bridge=true};break;}
        }
        var graph=new Dictionary<Key,List<int>>();var positions=new Dictionary<Key,float3>();
        Key A(PreparedSegment s)=>new(s.Kind,s.StartNodeId);Key B(PreparedSegment s)=>new(s.Kind,s.EndNodeId);
        float Ground(float3 p)=>NetworkProcessor.Sample(p.x,p.z,raster,scale,width);
        for(var i=0;i<edges.Length;i++)foreach(var entry in new[]{(A(edges[i]),edges[i].Start),(B(edges[i]),edges[i].End)})
        {if(!graph.TryGetValue(entry.Item1,out var list))graph[entry.Item1]=list=new();list.Add(i);positions[entry.Item1]=entry.Item2;}
        // The reference datum is terrain, never an earlier repair's raised deck.
        foreach(var key in positions.Keys.ToArray())
        {
            var p=positions[key];var incident=graph[key].Select(i=>edges[i]).ToArray();
            var offset=incident.All(s=>s.Tunnel)?-15:incident.Any(s=>s.Tunnel)?-2:0;
            p.y=math.clamp(Ground(p)+offset,0,scale);positions[key]=p;
        }
        int Next(int index,Key node)
        {
            var s=edges[index];var start=A(s)==node;var arrival=math.normalizesafe(start?s.Start.xz-s.End.xz:s.End.xz-s.Start.xz);
            var best=-1;var score=.8f;
            foreach(var j in graph[node])
            {
                var t=edges[j];if(j==index||!Same(s,t)||s.OneWay&&(A(t)==node)==start)continue;
                var departure=math.normalizesafe(A(t)==node?t.End.xz-t.Start.xz:t.Start.xz-t.End.xz);
                var dot=math.dot(arrival,departure);if(dot>score){best=j;score=dot;}
            }
            return best;
        }
        var routes=new List<Route>();var seen=new HashSet<int>();
        foreach(var seed in Enumerable.Range(0,edges.Length).OrderBy(i=>edges[i].ProtectedLink?1:0).ThenBy(i=>edges[i].WayId).ThenBy(i=>i))
        {
            if(seen.Contains(seed))continue;token.ThrowIfCancellationRequested();
            var first=A(edges[seed]);var reverse=seed;var backwards=new HashSet<int>{seed};
            while(true){var next=Next(reverse,first);if(next<0||seen.Contains(next)||!backwards.Add(next))break;reverse=next;first=A(edges[next])==first?B(edges[next]):A(edges[next]);}
            var route=new Route();route.Nodes.Add(first);route.Station.Add(0);var current=reverse;var node=first;
            while(seen.Add(current))
            {
                route.Edges.Add(current);var e=edges[current];node=A(e)==node?B(e):A(e);route.Nodes.Add(node);
                route.Station.Add(route.Station[route.Station.Count-1]+Math.Max(1,NetworkVerticalProfiles.ArcLength(e)));
                var next=Next(current,node);if(next<0||seen.Contains(next))break;current=next;
            }
            routes.Add(route);
        }
        var reference=new Dictionary<Key,float3>(positions);var smoothed=0;
        foreach(var route in routes)
        {
            var first=edges[route.Edges[0]];var span=IsMainline(first)?500f:100f;
            for(var j=1;j<route.Nodes.Count-1;j++)
            {
                token.ThrowIfCancellationRequested();var key=route.Nodes[j];if(graph[key].Count!=2)continue;
                var p=reference[key];var left=j;while(left>0&&route.Station[j]-route.Station[left-1]<=span)left--;
                var right=j;while(right+1<route.Nodes.Count&&route.Station[right+1]-route.Station[j]<=span)right++;
                double sw=0,sx=0,sxx=0,sy=0,sxy=0;
                for(var k=left;k<=right;k++)
                {
                    var x=route.Station[k]-route.Station[j];var support=(k>0?route.Station[k]-route.Station[k-1]:0)+(k+1<route.Nodes.Count?route.Station[k+1]-route.Station[k]:0);
                    var w=Math.Max(.01,1-Math.Abs(x)/(span+1))*Math.Max(1,support);
                    sw+=w;sx+=w*x;sxx+=w*x*x;sy+=w*reference[route.Nodes[k]].y;sxy+=w*x*reference[route.Nodes[k]].y;
                }
                var denominator=sw*sxx-sx*sx;if(denominator>1&&!graph[key].Any(i=>edges[i].Tunnel))p.y=(float)((sy*sxx-sxy*sx)/denominator);
                // Straighten only an entire simple run; do not repeatedly project
                // individual rail knots onto a moving window chord.
                var a=reference[route.Nodes[left]].xz;var b=reference[route.Nodes[right]].xz;var delta=b-a;
                var tolerance=first.Kind==NetworkKind.Rail?6f:4f;
                bool NearLine(float2 q)=>math.distance(q,a+delta*math.clamp(math.dot(q-a,delta)/Math.Max(1,math.lengthsq(delta)),0,1))<=tolerance;
                if(!first.Roundabout&&Enumerable.Range(left,right-left+1).All(k=>NearLine(reference[route.Nodes[k]].xz)))
                    p.xz=a+delta*math.clamp(math.dot(p.xz-a,delta)/Math.Max(1,math.lengthsq(delta)),0,1);
                positions[key]=p;smoothed++;
            }
        }
        for(var i=0;i<edges.Length;i++)
        {
            var s=edges[i];var a=positions[A(s)];var b=positions[B(s)];var c=s.CourseControl1+(a-s.Start);var d=s.CourseControl2+(b-s.End);
            c.y=math.lerp(a.y,b.y,1f/3);d.y=math.lerp(a.y,b.y,2f/3);
            edges[i]=s with{Start=a,End=b,Control1=c-s.StartCurveOffset,Control2=d-s.EndCurveOffset,FinalDeckCurve=null,HorizontalParent=null};
        }
        foreach(var node in graph.Where(p=>p.Value.Count==2))
        {
            var x=node.Value[0];var y=node.Value[1];var a=edges[x];var b=edges[y];if(!Same(a,b)||a.Roundabout||b.Roundabout)continue;
            var p=positions[node.Key];var xa=A(a)==node.Key;var yb=A(b)==node.Key;
            var awayA=(xa?a.End:a.Start)-p;var awayB=(yb?b.End:b.Start)-p;
            var da=math.normalizesafe(awayA.xz);var db=math.normalizesafe(awayB.xz);if(math.dot(da,db)>-.5f)continue;
            var axis=math.normalizesafe(da-db);var ca=p+new float3(axis.x,0,axis.y)*(math.length(awayA.xz)/3);
            var cb=p-new float3(axis.x,0,axis.y)*(math.length(awayB.xz)/3);
            ca.y=p.y+awayA.y/3;cb.y=p.y+awayB.y/3;
            edges[x]=xa?a with{Control1=ca}:a with{Control2=ca};edges[y]=yb?b with{Control1=cb}:b with{Control2=cb};
        }
        foreach(var node in graph.Where(p=>p.Key.Kind==NetworkKind.Rail&&p.Value.Count>=3))
        {
            var p=positions[node.Key];var ports=node.Value.Select(i=>(Index:i,Start:A(edges[i])==node.Key,
                Away:math.normalizesafe(((A(edges[i])==node.Key?edges[i].End:edges[i].Start)-p).xz))).ToArray();
            var best=1f;var axis=ports[0].Away;
            for(var i=0;i<ports.Length;i++)for(var j=i+1;j<ports.Length;j++)
            {var dot=math.dot(ports[i].Away,ports[j].Away);if(dot<best){best=dot;axis=math.normalizesafe(ports[i].Away-ports[j].Away);}}
            if(best>-.5f)continue; // No proven through track: preserve the source junction.
            foreach(var port in ports)
            {
                var s=edges[port.Index];var tangent=math.dot(port.Away,axis)>=0?axis:-axis;
                var length=math.distance(s.Start.xz,s.End.xz);var reach=Math.Min(80,length/3);
                var control=p+new float3(tangent.x,0,tangent.y)*reach;
                control.y=math.lerp(p.y,(port.Start?s.End:s.Start).y,1f/3);
                edges[port.Index]=port.Start?s with{Control1=control}:s with{Control2=control};
            }
        }
        IReadOnlyList<PreparedSegment> horizontal=NativeChainCompactor.Prepare(edges,log,token);
        horizontal=CarriagewaySeparation.Repair(horizontal,log,token);
        horizontal=NetworkLaneAlignment.Prepare(horizontal,log,token,width);
        horizontal=PortalSpans(horizontal,token);
        var cover=NetworkGeometrySafety.TunnelCoverSamples(horizontal,raster,scale,width,token).ToArray();
        var result=NetworkTunnelProfile.Solve(horizontal,scale,cover,log,token,design:new NetworkTunnelProfile.TerrainReference(raster,width,water));
        log.Diagnostic($"COORDINATED DESIGN routes={routes.Count}; smoothedOptionalStations={smoothed}; inputSections={source.Count}; outputSections={result.Count}; verticalSolves=1; endpointEscalation=false; lateHorizontalWriters=false; sourceReferenceImmutable=true; railSegmentTracks=2; qualityFailureAction=RequiresReview");
        return result;
    }

    private static IReadOnlyList<PreparedSegment> PortalSpans(IReadOnlyList<PreparedSegment> source,CancellationToken token)
    {
        var surface=new HashSet<Key>(source.Where(s=>!s.Tunnel).SelectMany(s=>new[]{new Key(s.Kind,s.StartNodeId),new Key(s.Kind,s.EndNodeId)}));
        var occupied=new HashSet<long>(source.SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}));var next=long.MinValue+6000000;
        var result=new List<PreparedSegment>();
        foreach(var s in source)
        {
            token.ThrowIfCancellationRequested();
            if(!s.Tunnel||NetworkVerticalProfiles.ArcLength(s)<40||!surface.Contains(new(s.Kind,s.StartNodeId))||!surface.Contains(new(s.Kind,s.EndNodeId)))
            {result.Add(s);continue;}
            // A portal-to-portal native edge needs an interior composition anchor,
            // not a new node every 80m or a second subdivision by another planner.
            while(occupied.Contains(next))next++;var id=next++;occupied.Add(id);
            var curve=new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);var split=NetworkDesignChecks.Split(curve,.5);
            var middle=(float3)split.Left.P3;
            result.Add(s with{End=middle,EndNodeId=id,Control1=(float3)split.Left.P1-s.StartCurveOffset,Control2=(float3)split.Left.P2,
                EndCurveOffset=default,EndTerminal=false,EndMiniRoundabout=false,EndRoundaboutRadius=0,FinalDeckCurve=null});
            result.Add(s with{Start=middle,StartNodeId=id,Control1=(float3)split.Right.P1,Control2=(float3)split.Right.P2-s.EndCurveOffset,
                StartCurveOffset=default,StartTerminal=false,StartMiniRoundabout=false,StartRoundaboutRadius=0,FinalDeckCurve=null});
        }
        return result;
    }
}
