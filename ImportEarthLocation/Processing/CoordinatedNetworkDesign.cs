using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// A single final geometry writer: connected route stations own the profile;
// terrain is an observation, not an instruction to follow every DEM pixel.
public static class CoordinatedNetworkDesign
{
    private readonly record struct Key(NetworkKind Kind,long Node);
    private sealed class Route
    {
        public readonly List<int> Edges=new();
        public readonly List<Key> Nodes=new();
        public readonly List<float> Station=new();
    }
    private static bool Main(PreparedSegment s)=>s.Kind==NetworkKind.Rail||s.Classification is "motorway" or "trunk";
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
            var first=result[seed];if(first.Tunnel||first.Bridge||first.ProtectedLink||!Main(first)||visited.Contains(seed))continue;
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
        var excluded=new HashSet<long>(result.Where(s=>s.ProtectedLink&&(underground.Contains(s.StartNodeId)||underground.Contains(s.EndNodeId))).Select(s=>s.WayId));
        log.Count("Short tunnel surface interruptions absorbed",filled);
        log.Diagnostic($"TUNNEL INTERVALS surfaceSectionsAbsorbed={filled}; maximumInterruptionMeters=220; undergroundLinkWaysExcluded={excluded.Count}; geographicOverrides=false");
        return result.Where(s=>!s.ProtectedLink||!excluded.Contains(s.WayId)).ToArray();
    }

    public static IReadOnlyList<PreparedSegment> Prepare(IReadOnlyList<PreparedSegment> source,ushort[] raster,float scale,double width,
        ImportLog log,CancellationToken token,Func<float3,float?>? water=null)
    {
        var edges=source.ToArray();var waterCrossingsPromoted=0;
        if(water!=null)for(var i=0;i<edges.Length;i++)
        {
            var s=edges[i];if(s.Tunnel||s.Bridge)continue;
            for(var j=0;j<=8;j++)
            {var p=NetworkNormalizer.Evaluate(s,j/8f);var level=water(p);if(!level.HasValue||!math.isfinite(level.Value)||p.y>=level.Value+7)continue;edges[i]=s with{Bridge=true};waterCrossingsPromoted++;break;}
        }
        var graph=new Dictionary<Key,List<int>>();var positions=new Dictionary<Key,float3>();
        var observed=new Dictionary<Key,float3>();var floor=new Dictionary<Key,float>();var ceiling=new Dictionary<Key,float>();
        Key A(PreparedSegment s)=>new(s.Kind,s.StartNodeId);Key B(PreparedSegment s)=>new(s.Kind,s.EndNodeId);
        float Ground(float3 p)=>NetworkProcessor.Sample(p.x,p.z,raster,scale,width);
        for(var i=0;i<edges.Length;i++)
        {
            var s=edges[i];foreach(var p in new[]{(A(s),s.Start),(B(s),s.End)})
            {if(!graph.TryGetValue(p.Item1,out var list))graph[p.Item1]=list=new();list.Add(i);positions[p.Item1]=p.Item2;}
        }
        foreach(var pair in positions)observed[pair.Key]=pair.Value;
        int Next(int index,Key node)
        {
            var s=edges[index];var start=A(s)==node;var arrival=math.normalizesafe(start?s.Start.xz-s.End.xz:s.End.xz-s.Start.xz);
            var best=-1;var score=.6f;
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
            var s=edges[seed];var first=A(s);var reverse=seed;var backwards=new HashSet<int>{seed};
            while(true){var next=Next(reverse,first);if(next<0||seen.Contains(next)||!backwards.Add(next))break;reverse=next;first=A(edges[next])==first?B(edges[next]):A(edges[next]);}
            var route=new Route();route.Nodes.Add(first);route.Station.Add(0);var current=reverse;var node=first;
            while(seen.Add(current))
            {
                route.Edges.Add(current);var e=edges[current];node=A(e)==node?B(e):A(e);route.Nodes.Add(node);
                route.Station.Add(route.Station[route.Station.Count-1]+Math.Max(1,math.distance(e.Start.xz,e.End.xz)));
                var next=Next(current,node);if(next<0||seen.Contains(next))break;current=next;
            }
            routes.Add(route);
        }
        var fitted=0;var lateral=0;
        foreach(var route in routes)
        {
            token.ThrowIfCancellationRequested();if(route.Edges.Count<2)continue;
            var first=edges[route.Edges[0]];var span=first.Kind==NetworkKind.Rail?500f:Main(first)?500f:100f;
            var targets=new float3[route.Nodes.Count];
            for(var j=0;j<route.Nodes.Count;j++)
            {
                var key=route.Nodes[j];var p=observed[key];targets[j]=positions[key];
                if(j==0||j==route.Nodes.Count-1||graph[key].Any(i=>edges[i].Tunnel))continue;
                // Distance-weighted local linear regression avoids dense OSM
                // node clusters receiving more influence than long intervals.
                double sw=0,sx=0,sxx=0,sy=0,sxy=0;
                var left=j;while(left>0&&route.Station[j]-route.Station[left-1]<=span)left--;
                var right=j;while(right+1<route.Nodes.Count&&route.Station[right+1]-route.Station[j]<=span)right++;
                for(var k=left;k<=right;k++)
                {
                    var x=route.Station[k]-route.Station[j];var support=(k>0?route.Station[k]-route.Station[k-1]:0)+(k+1<route.Nodes.Count?route.Station[k+1]-route.Station[k]:0);
                    var w=Math.Max(.01,1-Math.Abs(x)/(span+1))*Math.Max(1,support);var y=observed[route.Nodes[k]].y;
                    sw+=w;sx+=w*x;sxx+=w*x*x;sy+=w*y;sxy+=w*x*y;
                }
                var denominator=sw*sxx-sx*sx;if(denominator>1)targets[j].y=(float)((sy*sxx-sxy*sx)/denominator);
                // Only degree-two optional nodes move horizontally. Straight
                // route runs lose accidental knots; junctions keep one identity.
                if(first.Kind==NetworkKind.Rail&&graph[key].Count==2&&left<j&&right>j)
                {
                    var a=observed[route.Nodes[left]];var b=observed[route.Nodes[right]];var delta=b.xz-a.xz;
                    var t=math.clamp(math.dot(p.xz-a.xz,delta)/Math.Max(1,math.lengthsq(delta)),0,1);var q=math.lerp(a.xz,b.xz,t);
                    var limit=first.Kind==NetworkKind.Rail?32f:8f;
                    if(math.distance(q,p.xz)<=limit){targets[j].xz=q;lateral++;}
                }
                fitted++;
            }
            for(var j=0;j<targets.Length;j++)positions[route.Nodes[j]]=targets[j];
        }
        // Constraints come from final terrain and real structure requirements,
        // not previous independently raised endpoint heights.
        foreach(var node in graph)
        {
            var p=positions[node.Key];var ground=Ground(p);var incident=node.Value.Select(i=>edges[i]).ToArray();
            var budget=incident.Max(NetworkVerticalProfiles.CutFillBudget);
            floor[node.Key]=Math.Max(0,ground-budget);ceiling[node.Key]=Math.Min(scale,ground+budget);
            if(incident.Any(s=>s.Bridge)){floor[node.Key]=Math.Max(0,ground);ceiling[node.Key]=scale;}
            if(incident.Any(s=>s.Tunnel)){floor[node.Key]=0;ceiling[node.Key]=Math.Max(0,ground-(incident.Any(s=>!s.Tunnel)?2:15));}
            var other=new Key(node.Key.Kind==NetworkKind.Rail?NetworkKind.Road:NetworkKind.Rail,node.Key.Node);
            if(graph.TryGetValue(other,out var crossing)&&incident.All(s=>!s.Bridge&&!s.Tunnel)&&crossing.All(i=>!edges[i].Bridge&&!edges[i].Tunnel))floor[node.Key]=ceiling[node.Key]=ground;
            p.y=math.clamp(p.y,floor[node.Key],ceiling[node.Key]);positions[node.Key]=p;
        }
        foreach(var s in edges.Where(s=>s.Bridge&&!s.Tunnel))
        {
            var required=0f;for(var j=0;j<=16;j++)
            {var p=NetworkNormalizer.Evaluate(s,j/16f);var stage=water?.Invoke(p);if(stage.HasValue)required=Math.Max(required,stage.Value+7);}
            if(required<=0)continue;
            foreach(var key in new[]{A(s),B(s)}){floor[key]=Math.Max(floor[key],required);ceiling[key]=Math.Max(ceiling[key],floor[key]);}
        }
        foreach(var s in edges.Where(s=>s.Tunnel))
        {
            var roof=float.PositiveInfinity;var direction=math.normalizesafe((s.End-s.Start).xz);var side=new float3(-direction.y,0,direction.x)*Math.Max(3,s.NativeWidth*.5f);
            var count=Math.Max(8,(int)Math.Ceiling(math.distance(s.Start.xz,s.End.xz)/12));
            for(var j=0;j<=count;j++)
            {token.ThrowIfCancellationRequested();var p=math.lerp(positions[A(s)],positions[B(s)],j/(float)count);roof=Math.Min(roof,Math.Min(Ground(p),Math.Min(Ground(p-side),Ground(p+side))));}
            // One interval ceiling prevents DEM dips producing exposed slivers.
            foreach(var key in new[]{A(s),B(s)})if(graph[key].All(i=>edges[i].Tunnel))ceiling[key]=Math.Min(ceiling[key],Math.Max(0,roof-15));
        }
        var crossingGrid=new Dictionary<(int,int),HashSet<int>>();
        for(var i=0;i<edges.Length;i++)
        {
            var s=edges[i];var count=Math.Max(1,(int)Math.Ceiling(math.distance(s.Start.xz,s.End.xz)/32));
            for(var j=0;j<=count;j++){var p=math.lerp(positions[A(s)],positions[B(s)],j/(float)count);var key=((int)Math.Floor(p.x/64),(int)Math.Floor(p.z/64));if(!crossingGrid.TryGetValue(key,out var set))crossingGrid[key]=set=new();set.Add(i);}
        }
        var crossings=new List<(int upper,int lower,float t)>();var crossingPairs=new HashSet<(int,int)>();
        for(var i=0;i<edges.Length;i++)
        {
            var s=edges[i];if(!s.Bridge||s.Tunnel)continue;var a=positions[A(s)];var b=positions[B(s)];var u=b.xz-a.xz;
            var candidates=new HashSet<int>();var count=Math.Max(1,(int)Math.Ceiling(math.length(u)/32));
            for(var j=0;j<=count;j++)
            {var p=math.lerp(a,b,j/(float)count);var gx=(int)Math.Floor(p.x/64);var gz=(int)Math.Floor(p.z/64);for(var x=gx-1;x<=gx+1;x++)for(var z=gz-1;z<=gz+1;z++)if(crossingGrid.TryGetValue((x,z),out var set))candidates.UnionWith(set);}
            foreach(var index in candidates)
            {
                var t=edges[index];if(index==i||t.Tunnel||t.Bridge&&t.Layer>=s.Layer||A(t)==A(s)||A(t)==B(s)||B(t)==A(s)||B(t)==B(s))continue;
                var c=positions[A(t)];var d=positions[B(t)];var v=d.xz-c.xz;
                var determinant=u.x*v.y-u.y*v.x;if(Math.Abs(determinant)<.01f)continue;
                var delta=c.xz-a.xz;var upperT=(delta.x*v.y-delta.y*v.x)/determinant;var lowerT=(delta.x*u.y-delta.y*u.x)/determinant;
                if(upperT<0||upperT>1||lowerT<0||lowerT>1||!crossingPairs.Add((i,index)))continue;
                crossings.Add((i,index,lowerT));
            }
        }
        // Opposing nearest-station height constraints are deliberately absent:
        // reciprocal updates previously escalated deck heights without solving
        // horizontal overlap. Full swept width is repaired after curve fitting.
        foreach(var key in positions.Keys.ToArray())
        {var p=positions[key];p.y=math.clamp(p.y,floor[key],Math.Max(floor[key],ceiling[key]));positions[key]=p;}
        // Solve shared grades, keeping ramp attachment anchors on the through
        // profile. Constraints never become exceptions or deletion gates.
        var passes=0;for(;passes<512;passes++)
        {
            var motion=0f;
            foreach(var crossing in crossings)
            {
                var upper=edges[crossing.upper];var lower=edges[crossing.lower];var required=math.lerp(positions[A(lower)].y,positions[B(lower)].y,crossing.t)+7;
                foreach(var key in new[]{A(upper),B(upper)})
                {floor[key]=Math.Max(floor[key],required);ceiling[key]=Math.Max(ceiling[key],floor[key]);var p=positions[key];if(p.y<floor[key]){motion=Math.Max(motion,floor[key]-p.y);p.y=floor[key];positions[key]=p;}}
            }
            foreach(var s in edges)
            {
                token.ThrowIfCancellationRequested();var a=A(s);var b=B(s);var p=positions[a];var q=positions[b];
                var allowed=Math.Max(1,math.distance(p.xz,q.xz))*NetworkVerticalProfiles.DesiredGrade(s);var delta=q.y-p.y;
                if(Math.Abs(delta)<=allowed+.002f)continue;var sign=Math.Sign(delta);var excess=Math.Abs(delta)-allowed;
                var roomA=sign>0?ceiling[a]-p.y:p.y-floor[a];var roomB=sign>0?q.y-floor[b]:ceiling[b]-q.y;
                if(s.ProtectedLink){if(graph[a].Any(i=>Main(edges[i])))roomA=0;if(graph[b].Any(i=>Main(edges[i])))roomB=0;}
                var ma=Math.Min(Math.Max(0,roomA),excess*.5f);var mb=Math.Min(Math.Max(0,roomB),excess-ma);ma=Math.Min(Math.Max(0,roomA),excess-mb);
                p.y+=sign*ma;q.y-=sign*mb;positions[a]=p;positions[b]=q;motion=Math.Max(motion,ma+mb);
            }
            if(motion<.002f)break;
        }
        for(var i=0;i<edges.Length;i++)
        {
            var s=edges[i];var a=positions[A(s)];var b=positions[B(s)];var c1=s.CourseControl1+(a-s.Start);var c2=s.CourseControl2+(b-s.End);
            // Vertical handles start on the same linear profile; route joins
            // below supply shared, shape-preserving derivatives.
            var chord=b.xz-a.xz;var square=Math.Max(1,math.lengthsq(chord));
            c1.y=a.y+(b.y-a.y)*math.clamp(math.dot(c1.xz-(a+s.StartCurveOffset).xz,chord)/square,0,1);
            c2.y=a.y+(b.y-a.y)*math.clamp(math.dot(c2.xz-(a+s.StartCurveOffset).xz,chord)/square,0,1);
            edges[i]=s with{Start=a,End=b,Control1=c1-s.StartCurveOffset,Control2=c2-s.EndCurveOffset,StartElevation=a.y-Ground(a),EndElevation=b.y-Ground(b),HorizontalParent=null};
        }
        foreach(var route in routes)for(var j=1;j<route.Nodes.Count-1;j++)
        {
            var key=route.Nodes[j];var p=positions[key];var prev=positions[route.Nodes[j-1]];var next=positions[route.Nodes[j+1]];
            var incoming=math.normalizesafe(p.xz-prev.xz);var outgoing=math.normalizesafe(next.xz-p.xz);
            var tangent=math.normalizesafe(incoming+outgoing,outgoing);if(math.dot(incoming,outgoing)<-.2f)continue;
            var sa=(p.y-prev.y)/Math.Max(1,math.distance(prev.xz,p.xz));var sb=(next.y-p.y)/Math.Max(1,math.distance(next.xz,p.xz));
            var slope=sa*sb>0?2*sa*sb/(sa+sb):0;
            foreach(var index in new[]{route.Edges[j-1],route.Edges[j]})
            {
                var s=edges[index];var start=A(s)==key;var forward=math.dot((s.End-s.Start).xz,tangent)>=0?tangent:-tangent;
                var reach=Math.Max(.1f,math.distance(s.Start.xz,s.End.xz)/3);var control=p+new float3(forward.x,0,forward.y)*reach*(start?1:-1);
                control.y=p.y+slope*reach*(start?1:-1)*(math.dot(forward,tangent)>=0?1:-1);
                edges[index]=start?s with{Control1=control}:s with{Control2=control};
            }
        }
        // Bridge/tunnel metadata boundaries do not exempt a connected railway
        // from a shared horizontal tangent. Fit every degree-two rail join.
        foreach(var pair in graph.Where(p=>p.Key.Kind==NetworkKind.Rail&&p.Value.Count==2))
        {
            var x=pair.Value[0];var y=pair.Value[1];var a=edges[x];var b=edges[y];var p=positions[pair.Key];
            var xa=A(a)==pair.Key;var yb=A(b)==pair.Key;var farA=xa?a.End:a.Start;var farB=yb?b.End:b.Start;
            var direction=math.normalizesafe(math.normalizesafe(p.xz-farA.xz)+math.normalizesafe(farB.xz-p.xz));
            if(math.lengthsq(direction)<.01f)direction=math.normalizesafe(farB.xz-farA.xz);
            var ra=math.distance(p.xz,farA.xz)/3;var rb=math.distance(p.xz,farB.xz)/3;
            var ca=p-new float3(direction.x,0,direction.y)*ra;var cb=p+new float3(direction.x,0,direction.y)*rb;
            ca.y=math.lerp(p.y,farA.y,1f/3);cb.y=math.lerp(p.y,farB.y,1f/3);
            edges[x]=xa?a with{Control1=ca}:a with{Control2=ca};edges[y]=yb?b with{Control1=cb}:b with{Control2=cb};
        }
        var railJoins=0;
        foreach(var pair in graph.Where(p=>p.Key.Kind==NetworkKind.Rail&&p.Value.Count>=3))
        {
            var selected=pair.Value.OrderByDescending(i=>math.distance(edges[i].Start.xz,edges[i].End.xz)).ToArray();
            var axis=math.normalizesafe((edges[selected[0]].End-edges[selected[0]].Start).xz);
            foreach(var index in selected)
            {
                var s=edges[index];var start=A(s)==pair.Key;var p=start?s.Start:s.End;var away=math.normalizesafe((start?s.End-s.Start:s.Start-s.End).xz);
                var tangent=math.dot(axis,away)>=0?axis:-axis;var normal=new float2(-tangent.y,tangent.x);
                var side=Math.Sign(math.dot(away,normal));var approach=math.normalizesafe(tangent*.94f+normal*side*.342f);
                var length=math.distance(s.Start.xz,s.End.xz);var control=p+new float3(approach.x,0,approach.y)*(length/3);
                control.y=p.y+((start?s.End.y:s.Start.y)-p.y)/3;
                edges[index]=start?s with{Control1=control}:s with{Control2=control};railJoins++;
            }
        }
        var residual=edges.Count(s=>Math.Abs(s.End.y-s.Start.y)>Math.Max(1,math.distance(s.Start.xz,s.End.xz))*NetworkVerticalProfiles.DesiredGrade(s)+.01f);
        log.Count("Coordinated design routes",routes.Count);log.Count("Coordinated design grade residuals",residual);
        log.Diagnostic($"COORDINATED DESIGN routes={routes.Count}; stationFits={fitted}; optionalHorizontalKnotsRefitted={lateral}; reciprocalParallelHeightCoupling=false; actualCrossings={crossings.Count}; mappedWaterCrossingsPromoted={waterCrossingsPromoted}; railJunctionApproaches={railJoins}; gradePasses={passes}; gradeResidualSections={residual}; generationGates=false; railSegmentTracks=2");
        return CarriagewaySeparation.Repair(edges,log,token);
    }
}
