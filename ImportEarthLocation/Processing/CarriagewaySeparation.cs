using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Repair the swept widths of opposing carriageways without stacking their decks.
// Shared anchors move once; attached ramps retain the same graph node identity.
public static class CarriagewaySeparation
{
    private readonly record struct Sample(int Edge,float T,float3 Point,float2 Tangent);
    private static bool Eligible(PreparedSegment s)=>s.Kind==NetworkKind.Road&&s.OneWay&&!s.ProtectedLink&&
        (s.Classification is "motorway" or "trunk")&&math.isfinite(s.NativeWidth)&&s.NativeWidth>0&&
        math.all(math.isfinite(s.CourseStart))&&math.all(math.isfinite(s.CourseEnd))&&
        math.all(math.isfinite(s.CourseControl1))&&math.all(math.isfinite(s.CourseControl2));
    private static bool Related(PreparedSegment a,PreparedSegment b)=>a.WayId!=b.WayId&&a.Layer==b.Layer&&a.Tunnel==b.Tunnel&&
        (a.Reference.Length>0&&a.Reference==b.Reference||a.Name.Length>0&&a.Name==b.Name||a.CorridorId!=0&&a.CorridorId==b.CorridorId);
    private static float2 Limit(float2 p,float maximum)=>math.lengthsq(p)>maximum*maximum?math.normalizesafe(p)*maximum:p;
    private static float2 OriginalPoint(PreparedSegment s,float2 p)
    {
        var geometry=s.OriginalGeometry;var result=p;var best=float.PositiveInfinity;
        for(var i=1;i<geometry.Count;i++)
        {var a=geometry[i-1].xz;var d=geometry[i].xz-a;var q=a+d*math.clamp(math.dot(p-a,d)/Math.Max(.001f,math.lengthsq(d)),0,1);var distance=math.lengthsq(q-p);if(distance<best){best=distance;result=q;}}
        return result;
    }

    public static IReadOnlyList<PreparedSegment> Repair(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token)
    {
        var edges=source.ToArray();var indices=Enumerable.Range(0,edges.Length).Where(i=>Eligible(edges[i])).ToArray();
        var anchors=new Dictionary<long,List<(int edge,bool start)>>();
        for(var i=0;i<edges.Length;i++)if(edges[i].Kind==NetworkKind.Road)
            foreach(var item in new[]{(edges[i].StartNodeId,true),(edges[i].EndNodeId,false)})
            {if(!anchors.TryGetValue(item.Item1,out var list))anchors[item.Item1]=list=new();list.Add((i,item.Item2));}
        var initial=0;var residual=0;var passes=0;var moved=0;
        for(;passes<=32;passes++)
        {
            token.ThrowIfCancellationRequested();
            var grid=new Dictionary<(int,int),List<Sample>>();var samples=new List<Sample>();
            foreach(var index in indices)
            {
                var s=edges[index];var count=Math.Max(8,Math.Min(256,(int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(s)/6)));
                for(var k=0;k<=count;k++)
                {
                    var t=k/(float)count;var p=NetworkNormalizer.Evaluate(s,t);
                    var tangent=math.normalizesafe((NetworkNormalizer.Evaluate(s,Math.Min(1,t+.002f))-NetworkNormalizer.Evaluate(s,Math.Max(0,t-.002f))).xz);
                    var sample=new Sample(index,t,p,tangent);samples.Add(sample);
                    var cell=((int)Math.Floor(p.x/32),(int)Math.Floor(p.z/32));
                    if(!grid.TryGetValue(cell,out var list))grid[cell]=list=new();list.Add(sample);
                }
            }
            var changes=new Dictionary<(int,int),(float2 sum,int count)>();residual=0;
            void Add(int edge,int control,float2 delta)
            {var key=(edge,control);changes.TryGetValue(key,out var old);changes[key]=(old.sum+delta,old.count+1);}
            foreach(var sample in samples)
            {
                token.ThrowIfCancellationRequested();
                var s=edges[sample.Edge];var gx=(int)Math.Floor(sample.Point.x/32);var gz=(int)Math.Floor(sample.Point.z/32);
                Sample? nearest=null;var best=float.PositiveInfinity;
                for(var x=gx-1;x<=gx+1;x++)for(var z=gz-1;z<=gz+1;z++)if(grid.TryGetValue((x,z),out var list))foreach(var other in list)
                {
                    var b=edges[other.Edge];if(other.Edge==sample.Edge||!Related(s,b)||math.dot(sample.Tangent,other.Tangent)>-.7f||
                        s.StartNodeId==b.StartNodeId||s.StartNodeId==b.EndNodeId||s.EndNodeId==b.StartNodeId||s.EndNodeId==b.EndNodeId||
                        Math.Abs(sample.Point.y-other.Point.y)>3)continue;
                    var delta=sample.Point.xz-other.Point.xz;
                    // Compare cross-section spacing, not longitudinal sample spacing.
                    var longitudinal=Math.Abs(math.dot(delta,sample.Tangent));if(longitudinal>7)continue;
                    var distance=math.length(delta);if(distance<best){best=distance;nearest=other;}
                }
                if(!nearest.HasValue)continue;
                var partner=edges[nearest.Value.Edge];var required=(s.NativeWidth+partner.NativeWidth)*.5f+2;
                var normal=new float2(-sample.Tangent.y,sample.Tangent.x);
                var signed=math.dot(sample.Point.xz-nearest.Value.Point.xz,normal);
                var original=OriginalPoint(s,sample.Point.xz);var originalPartner=OriginalPoint(partner,original);
                var originalSide=math.dot(original-originalPartner,normal);
                var side=originalSide<-.1f?-1f:originalSide>.1f?1f:signed<0?-1f:1f;
                var spacing=signed*side;
                if(spacing>=required-.1f)continue;
                residual++;
                // Keep the observed side. An exact coincident point uses a stable
                // left-of-travel normal, which reverses for the opposing road.
                var direction=normal*side;var deltaMove=direction*Math.Min(1,(required-spacing)*.5f);
                var t=sample.T;var u=1-t;var weights=new[]{u*u*u,3*u*u*t,3*u*t*t,t*t*t};
                var denominator=weights.Sum(w=>w*w);
                for(var c=0;c<4;c++)if(weights[c]>.001f)Add(sample.Edge,c,deltaMove*weights[c]/denominator);
            }
            if(passes==0)initial=residual;if(residual==0||passes==32)break;
            var nodeMoves=new Dictionary<long,(float2 sum,int count)>();
            foreach(var change in changes.Where(p=>p.Key.Item2==0||p.Key.Item2==3))
            {
                var s=edges[change.Key.Item1];var node=change.Key.Item2==0?s.StartNodeId:s.EndNodeId;
                var delta=Limit(change.Value.sum/change.Value.count,1);nodeMoves.TryGetValue(node,out var old);nodeMoves[node]=(old.sum+delta,old.count+1);
            }
            foreach(var pair in nodeMoves)
            {
                var delta=Limit(pair.Value.sum/pair.Value.count,1);if(math.lengthsq(delta)<.000001f)continue;
                var translation=new float3(delta.x,0,delta.y);
                foreach(var entry in anchors[pair.Key])
                {var s=edges[entry.edge];edges[entry.edge]=entry.start?s with{Start=s.Start+translation,Control1=s.CourseControl1-s.StartCurveOffset+translation}:s with{End=s.End+translation,Control2=s.CourseControl2-s.EndCurveOffset+translation};}
                moved++;
            }
            foreach(var change in changes.Where(p=>p.Key.Item2==1||p.Key.Item2==2))
            {
                var s=edges[change.Key.Item1];var delta=Limit(change.Value.sum/change.Value.count,1);var translation=new float3(delta.x,0,delta.y);
                edges[change.Key.Item1]=change.Key.Item2==1?s with{Control1=s.CourseControl1-s.StartCurveOffset+translation}:s with{Control2=s.CourseControl2-s.EndCurveOffset+translation};
            }
            // Reconcile continuing mainline handles after each lateral update.
            // Otherwise independently pushed controls can introduce a join kink.
            foreach(var node in anchors.Values)
            {
                var through=node.Where(p=>Eligible(edges[p.edge])).ToArray();if(through.Length!=2||through[0].start==through[1].start)continue;
                var a=edges[through[0].edge];var b=edges[through[1].edge];
                if(a.Classification!=b.Classification)continue;
                var pa=through[0].start?a.CourseStart:a.CourseEnd;var pb=through[1].start?b.CourseStart:b.CourseEnd;
                var ca=through[0].start?a.CourseControl1:a.CourseControl2;var cb=through[1].start?b.CourseControl1:b.CourseControl2;
                var da=ca.xz-pa.xz;var db=cb.xz-pb.xz;var axis=math.normalizesafe(math.normalizesafe(da)-math.normalizesafe(db));
                if(math.lengthsq(axis)<.01f||math.dot(math.normalizesafe(da),math.normalizesafe(db))>-.5f)continue;
                ca.xz=pa.xz+axis*math.length(da);cb.xz=pb.xz-axis*math.length(db);
                edges[through[0].edge]=through[0].start?a with{Control1=ca-a.StartCurveOffset}:a with{Control2=ca-a.EndCurveOffset};
                edges[through[1].edge]=through[1].start?b with{Control1=cb-b.StartCurveOffset}:b with{Control2=cb-b.EndCurveOffset};
            }
        }
        log.Count("Carriageway initial sampled contacts",initial);
        log.Count("Carriageway remaining sampled contacts",residual);
        log.Diagnostic($"CARRIAGEWAY SWEPT WIDTH eligible={indices.Length}; initialSampleContacts={initial}; lastSampleContacts={residual}; passes={passes}; sharedAnchorMoves={moved}; heightChanges=0; medianMeters=2; maximumStepMeters=1; residualsAdvisory=true; nativeMeshesNotVerified=true");
        return edges;
    }
}
