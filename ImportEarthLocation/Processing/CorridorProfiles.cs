using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Engineering-distance smoothing belongs to connected corridors, not OSM way
// boundaries. This pass runs after horizontal placement and before earthworks.
// It changes no node identity, horizontal alignment, direction or structure type.
public static class CorridorProfiles
{
    public static IReadOnlyList<PreparedSegment> Prepare(IReadOnlyList<PreparedSegment> source,
        ImportLog log,CancellationToken token,Func<float3,float?>? waterLevel=null,bool finishOnly=false)
    {
        if(source.Count==0)return source;
        var graph=new Dictionary<(NetworkKind,long),List<int>>();
        var nodes=new Dictionary<(NetworkKind,long),int>();var original=new List<float>();
        int Node(NetworkKind kind,long id,float y)
        {var key=(kind,id);if(nodes.TryGetValue(key,out var found))return found;var index=nodes.Count;nodes[key]=index;original.Add(y);graph[key]=new();return index;}
        var a=new int[source.Count];var b=new int[source.Count];var length=new float[source.Count];
        for(var i=0;i<source.Count;i++)
        {
            var s=source[i];a[i]=Node(s.Kind,s.StartNodeId,s.Start.y);b[i]=Node(s.Kind,s.EndNodeId,s.End.y);
            graph[(s.Kind,s.StartNodeId)].Add(i);graph[(s.Kind,s.EndNodeId)].Add(i);
            length[i]=NetworkVerticalProfiles.ArcLength(s);
        }
        var h=original.ToArray();var lo=h.Select(y=>Math.Max(0,y-60)).ToArray();var hi=h.Select(y=>y+60).ToArray();
        foreach(var pair in graph)
        {
            var id=nodes[pair.Key];var edges=pair.Value.Select(i=>source[i]).ToArray();
            var budget=edges.Min(NetworkVerticalProfiles.CutFillBudget);
            lo[id]=Math.Max(0,h[id]-budget);hi[id]=h[id]+budget;
            if(edges.Any(s=>s.Bridge)){lo[id]=h[id];hi[id]=h[id]+60;}
            if(edges.Any(s=>s.Tunnel)){lo[id]=Math.Max(0,h[id]-120);hi[id]=h[id];}
            // An explicit level crossing is an anchor, not two independent
            // profiles that may separate vertically during final finishing.
            if(graph.ContainsKey((pair.Key.Item1==NetworkKind.Road?NetworkKind.Rail:NetworkKind.Road,pair.Key.Item2))&&edges.All(s=>!s.Bridge&&!s.Tunnel))lo[id]=hi[id]=h[id];
        }
        var visited=new bool[source.Count];var corridors=0;var macroNodes=0;
        bool Compatible(int x,int y)
        {var s=source[x];var t=source[y];return s.Kind==t.Kind&&s.Classification==t.Classification&&s.Layer==t.Layer&&s.OneWay==t.OneWay&&s.Bridge==t.Bridge&&s.Tunnel==t.Tunnel&&!s.ExplicitIncline.HasValue&&!t.ExplicitIncline.HasValue;}
        foreach(var entry in graph.OrderBy(p=>p.Key.Item1).ThenBy(p=>p.Key.Item2))
        foreach(var seed in entry.Value)
        {
            token.ThrowIfCancellationRequested();if(visited[seed])continue;
            // Start at an actual boundary; isolated closed rings intentionally
            // keep their elevation rather than inventing a break in the loop.
            if(entry.Value.Count==2&&Compatible(entry.Value[0],entry.Value[1]))continue;
            var ids=new List<int>{nodes[entry.Key]};var edges=new List<int>();var station=new List<float>{0};
            var key=entry.Key;var current=seed;
            while(!visited[current])
            {
                visited[current]=true;edges.Add(current);var s=source[current];
                key=(s.Kind,key.Item2==s.StartNodeId?s.EndNodeId:s.StartNodeId);
                ids.Add(nodes[key]);station.Add(station[station.Count-1]+length[current]);
                var incident=graph[key];if(incident.Count!=2)break;
                var next=incident[0]==current?incident[1]:incident[0];if(!Compatible(current,next))break;current=next;
            }
            var first=source[seed];if(finishOnly||first.Tunnel||first.Bridge||edges.Count<3||station[station.Count-1]<160)continue;
            var window=first.Kind==NetworkKind.Rail||first.Classification=="motorway"||first.Classification=="trunk"?400f:160f;
            var left=0;var right=0;corridors++;
            for(var j=1;j<ids.Count-1;j++)
            {
                while(left<j&&station[j]-station[left]>window)left++;
                if(right<j)right=j;while(right+1<ids.Count&&station[right+1]-station[j]<=window)right++;
                double sum=0,weight=0;
                for(var k=left;k<=right;k++)
                {
                    var support=(k==0?0:station[k]-station[k-1])+(k==ids.Count-1?0:station[k+1]-station[k]);
                    var w=Math.Max(.001,1-Math.Abs(station[k]-station[j])/window)*support;
                    sum+=original[ids[k]]*w;weight+=w;
                }
                var id=ids[j];if(weight<=0)continue;var value=math.clamp((float)(sum/weight),lo[id],hi[id]);
                if(Math.Abs(value-h[id])>.01f){h[id]=value;macroNodes++;}
            }
        }
        // Only genuine bridges crossing evidenced water are lifted. Treat each
        // source deck route separately, never one plane for a whole interchange.
        var waterDecks=0;var maximumLift=0f;
        if(waterLevel!=null)
        foreach(var group in Enumerable.Range(0,source.Count).Where(i=>source[i].Bridge&&!source[i].Tunnel).GroupBy(i=>(source[i].Kind,source[i].WayId)))
        {
            var lift=0f;
            foreach(var i in group)
            {
                var steps=Math.Max(8,(int)Math.Ceiling(length[i]/16));
                for(var j=0;j<=steps;j++)
                {token.ThrowIfCancellationRequested();var p=NetworkNormalizer.Evaluate(source[i],j/(float)steps);var stage=waterLevel(p);if(stage.HasValue&&math.isfinite(stage.Value))lift=Math.Max(lift,stage.Value+7-p.y);}
            }
            if(lift<.01f)continue;
            if(lift>60)throw new PreparationDataException(PreparationDataIssue.GeometryUnavailable,$"Water/deck datum mismatch for bridge {group.Key.Item2}: {lift:0.##}m lift required. Unsafe native geometry was not submitted.");
            foreach(var id in group.SelectMany(i=>new[]{a[i],b[i]}).Distinct()){lo[id]=Math.Max(lo[id],original[id]+lift);hi[id]=Math.Max(hi[id],lo[id]);h[id]=Math.Max(h[id],lo[id]);}
            waterDecks++;maximumLift=Math.Max(maximumLift,lift);
        }
        // Reconcile shared anchors after late spacing / portal changes. Bounded
        // projections retain every source edge and never connect nearby roads.
        var order=Enumerable.Range(0,source.Count).OrderBy(i=>length[i]).ThenBy(i=>source[i].WayId).ToArray();
        var minimumDelta=new float[source.Count];var maximumDelta=new float[source.Count];var gradeCeilings=new float[source.Count];
        for(var i=0;i<source.Count;i++)
        {
            token.ThrowIfCancellationRequested();var s=source[i];
            var desired=length[i]*NetworkVerticalProfiles.DesiredGrade(s);
            minimumDelta[i]=-desired;maximumDelta[i]=desired;
            if(!finishOnly)continue;
            // Endpoint-only grading can steepen a short handle even while its
            // average grade improves. The affine endpoint adjustment adds one
            // constant to dy/dt; bound that change against the actual derivative.
            var ceiling=Math.Max(NetworkGeometrySafety.GradeLimit(s),NetworkVerticalProfiles.MaximumGrade(s))+.0001f;
            gradeCeilings[i]=ceiling;
            if(!math.isfinite(ceiling))continue;
            var lower=float.NegativeInfinity;var upper=float.PositiveInfinity;
            for(var j=0;j<=256;j++)
            {
                var t=j/256f;var u=1-t;
                var derivative=3*u*u*(s.CourseControl1-s.CourseStart)+6*u*t*(s.CourseControl2-s.CourseControl1)+3*t*t*(s.CourseEnd-s.CourseControl2);
                var allowance=ceiling*math.length(derivative.xz);
                lower=Math.Max(lower,-allowance-derivative.y);upper=Math.Min(upper,allowance-derivative.y);
            }
            var before=s.End.y-s.Start.y;lower+=before;upper+=before;
            if(lower<=maximumDelta[i]&&upper>=minimumDelta[i])
            {minimumDelta[i]=Math.Max(minimumDelta[i],lower);maximumDelta[i]=Math.Min(maximumDelta[i],upper);}
            else
            {
                // Preserve an already constrained cubic rather than enforcing a
                // nicer endpoint chord at the cost of a worse local steep spike.
                minimumDelta[i]=lower;maximumDelta[i]=upper;
            }
        }
        var passes=0;
        for(;passes<256;passes++)
        {
            var change=0f;
            foreach(var i in order)
            {
                token.ThrowIfCancellationRequested();if(a[i]==b[i])continue;
                var delta=h[b[i]]-h[a[i]];var target=math.clamp(delta,minimumDelta[i],maximumDelta[i]);var excess=Math.Abs(delta-target);if(excess<.001f)continue;
                var sign=Math.Sign(delta-target);var roomA=Math.Max(0,sign>0?hi[a[i]]-h[a[i]]:h[a[i]]-lo[a[i]]);var roomB=Math.Max(0,sign>0?h[b[i]]-lo[b[i]]:hi[b[i]]-h[b[i]]);
                var ma=Math.Min(roomA,excess*.5f);var mb=Math.Min(roomB,excess-ma);ma=Math.Min(roomA,excess-mb);
                h[a[i]]+=sign*ma;h[b[i]]-=sign*mb;change=Math.Max(change,ma+mb);
            }
            if(change<.001f)break;
        }
        var revertedAnchors=0;
        if(finishOnly)
        {
            // Bounded projections may stop with mutually constrained junctions
            // still unsatisfied. Restore OPTIONAL offending anchor motions and
            // recheck every dependent edge; never silently emit a steeper curve.
            var incident=Enumerable.Range(0,nodes.Count).Select(_=>new List<int>()).ToArray();
            for(var i=0;i<source.Count;i++){incident[a[i]].Add(i);incident[b[i]].Add(i);}
            var pending=Enumerable.Range(0,source.Count).ToList();var queued=Enumerable.Repeat(true,source.Count).ToArray();
            for(var cursor=0;cursor<pending.Count;cursor++)
            {
                token.ThrowIfCancellationRequested();var i=pending[cursor];queued[i]=false;var s=source[i];
                var da=h[a[i]]-original[a[i]];var db=h[b[i]]-original[b[i]];
                if(Math.Abs(da-db)<.0001f||!math.isfinite(gradeCeilings[i]))continue;
                var start=s.Start;var end=s.End;start.y=h[a[i]];end.y=h[b[i]];
                var c1=s.Control1??math.lerp(s.Start,s.End,1f/3);var c2=s.Control2??math.lerp(s.Start,s.End,2f/3);
                c1.y+=math.lerp(da,db,1f/3);c2.y+=math.lerp(da,db,2f/3);
                if(NetworkVerticalProfiles.MaximumGrade(s with{Start=start,End=end,Control1=c1,Control2=c2})<=gradeCeilings[i]+.0001f)continue;
                foreach(var id in new[]{a[i],b[i]})
                {
                    if(Math.Abs(h[id]-original[id])<.0001f||original[id]<lo[id]||original[id]>hi[id])continue;
                    h[id]=original[id];revertedAnchors++;
                    foreach(var other in incident[id])if(!queued[other]){queued[other]=true;pending.Add(other);}
                }
            }
        }
        var result=source.ToArray();var replaced=0;var flattened=0;
        for(var i=0;i<result.Length;i++)
        {
            token.ThrowIfCancellationRequested();var s=source[i];var start=s.Start;var end=s.End;start.y=h[a[i]];end.y=h[b[i]];
            var c1=s.Control1??math.lerp(s.Start,s.End,1f/3);var c2=s.Control2??math.lerp(s.Start,s.End,2f/3);
            c1.y+=math.lerp(start.y-s.Start.y,end.y-s.End.y,1f/3);c2.y+=math.lerp(start.y-s.Start.y,end.y-s.End.y,2f/3);
            var revised=s with{Start=start,End=end,Control1=c1,Control2=c2,StartElevation=s.StartElevation+(start.y-s.Start.y),EndElevation=s.EndElevation+(end.y-s.End.y)};
            // Remove per-span terrain waviness on ordinary roads, not the roof
            // or deck shape which protects a tunnel/grade-separated crossing.
            if(!s.Tunnel&&!s.Bridge&&!s.ExplicitIncline.HasValue&&(!finishOnly||!NetworkVerticalProfiles.WithinGradeCeiling(revised,NetworkGeometrySafety.GradeLimit(s))))
            {
                var delta=(revised.CourseEnd-revised.CourseStart).xz;var square=math.lengthsq(delta);
                if(square>.01f)
                {
                    var y=revised.CourseStart.y;var rise=revised.CourseEnd.y-y;
                    var x=c1;var z=c2;x.y=y+rise*math.dot(revised.CourseControl1.xz-revised.CourseStart.xz,delta)/square-revised.StartCurveOffset.y;
                    z.y=y+rise*math.dot(revised.CourseControl2.xz-revised.CourseStart.xz,delta)/square-revised.EndCurveOffset.y;
                    var plane=revised with{Control1=x,Control2=z};var budget=NetworkVerticalProfiles.CutFillBudget(s);
                    if(Enumerable.Range(0,17).All(j=>Math.Abs(NetworkNormalizer.Evaluate(plane,j/16f).y-NetworkNormalizer.Evaluate(revised,j/16f).y)<=budget)&&
                        NetworkVerticalProfiles.MaximumGrade(plane)<=Math.Max(NetworkGeometrySafety.GradeLimit(s),NetworkVerticalProfiles.MaximumGrade(revised)))
                    {revised=plane;flattened++;}
                }
            }
            result[i]=revised;if(revised!=s)replaced++;
        }
        var joins=0;
        foreach(var pair in graph.Where(p=>p.Value.Count==2))
        {
            token.ThrowIfCancellationRequested();var x=pair.Value[0];var y=pair.Value[1];if(x==y||!Compatible(x,y))continue;
            var left=result[x];var right=result[y];if(left.Bridge||left.Tunnel||right.Bridge||right.Tunnel)continue;
            var leftStart=left.StartNodeId==pair.Key.Item2;var rightStart=right.StartNodeId==pair.Key.Item2;
            var p=leftStart?left.Start:left.End;var ca=leftStart?left.Control1!.Value:left.Control2!.Value;var cb=rightStart?right.Control1!.Value:right.Control2!.Value;
            if(math.dot(math.normalizesafe(p.xz-ca.xz),math.normalizesafe(cb.xz-p.xz))<.866f)continue;
            var sa=(p.y-(leftStart?left.End.y:left.Start.y))/Math.Max(.01f,length[x]);var sb=((rightStart?right.End.y:right.Start.y)-p.y)/Math.Max(.01f,length[y]);
            // A shape-preserving harmonic tangent is zero at a crest/sag and
            // cannot overshoot an otherwise monotone longitudinal profile.
            var slope=sa*sb>0?2*sa*sb/(sa+sb):0;
            ca.y=p.y-slope*math.distance(ca.xz,p.xz);cb.y=p.y+slope*math.distance(cb.xz,p.xz);
            var nx=leftStart?left with{Control1=ca}:left with{Control2=ca};var ny=rightStart?right with{Control1=cb}:right with{Control2=cb};
            if(NetworkVerticalProfiles.WithinGradeCeiling(nx,NetworkGeometrySafety.GradeLimit(left))&&NetworkVerticalProfiles.WithinGradeCeiling(ny,NetworkGeometrySafety.GradeLimit(right)))
            {result[x]=nx;result[y]=ny;joins++;}
        }
        log.Diagnostic($"PROCESS corridor-profiles finishOnly={finishOnly}; corridors={corridors}; macroNodes={macroNodes}; sectionsUpdated={replaced}; localWavesRemoved={flattened}; monotoneVerticalJoins={joins}; waterDecksRaised={waterDecks}; maximumWaterLiftMeters={maximumLift:R}; passes={passes}; optionalAnchorsRestoredForDerivativeSafety={revertedAnchors}; sharedTopologyPreserved=true; structuresNotInvented=true; bridgeFloorsAndTunnelCeilingsPreserved=true; remainingDesiredEndpointGrades={Enumerable.Range(0,result.Length).Count(i=>Math.Abs(h[a[i]]-h[b[i]])>length[i]*NetworkVerticalProfiles.DesiredGrade(source[i])+.01f)}");
        return result;
    }
}
