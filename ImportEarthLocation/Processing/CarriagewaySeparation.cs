using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// One immutable corridor-width fit, not iterative repulsion of neighboring
// Bezier handles. Both sides use the same midpoint and required median width.
public static class CarriagewaySeparation
{
    private readonly record struct Sample(int Edge,float3 Point,float2 Tangent);
    private static bool Eligible(PreparedSegment s)=>s.Kind==NetworkKind.Road&&s.OneWay&&!s.ProtectedLink&&
        (s.Classification is "motorway" or "trunk")&&math.isfinite(s.NativeWidth)&&s.NativeWidth>0;
    private static bool Related(PreparedSegment a,PreparedSegment b)=>a.WayId!=b.WayId&&a.Layer==b.Layer&&a.Tunnel==b.Tunnel&&
        (a.CorridorId!=0&&a.CorridorId==b.CorridorId||a.Reference.Length>0&&a.Reference==b.Reference||a.Name.Length>0&&a.Name==b.Name);

    public static IReadOnlyList<PreparedSegment> Repair(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token)
    {
        var result=source.ToArray();var grid=new Dictionary<(int,int),List<Sample>>();
        var endpoints=new Dictionary<long,(float2 Sum,int Count)>();var requested=new Dictionary<int,float2[]>();
        (int,int) Cell(float3 p)=>((int)Math.Floor(p.x/64),(int)Math.Floor(p.z/64));
        float2 Tangent(PreparedSegment s,float t)=>math.normalizesafe((NetworkNormalizer.Evaluate(s,Math.Min(1,t+.002f))-NetworkNormalizer.Evaluate(s,Math.Max(0,t-.002f))).xz);
        for(var i=0;i<source.Count;i++)if(Eligible(source[i]))
        {
            var s=source[i];var count=Math.Max(8,(int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(s)/6));
            for(var j=0;j<=count;j++)
            {var t=j/(float)count;var p=NetworkNormalizer.Evaluate(s,t);var key=Cell(p);if(!grid.TryGetValue(key,out var list))grid[key]=list=new();list.Add(new(i,p,Tangent(s,t)));}
        }
        var contacts=0;var limited=0;
        for(var i=0;i<source.Count;i++)
        {
            token.ThrowIfCancellationRequested();var s=source[i];if(!Eligible(s))continue;
            var shifts=new float2[4];var paired=false;
            for(var j=0;j<4;j++)
            {
                var t=j/3f;var p=NetworkNormalizer.Evaluate(s,t);var tangent=Tangent(s,t);var normal=new float2(-tangent.y,tangent.x);
                var cell=Cell(p);Sample? partner=null;var best=float.PositiveInfinity;
                for(var x=cell.Item1-1;x<=cell.Item1+1;x++)for(var z=cell.Item2-1;z<=cell.Item2+1;z++)if(grid.TryGetValue((x,z),out var candidates))foreach(var candidate in candidates)
                {
                    var other=source[candidate.Edge];
                    if(candidate.Edge==i||!Related(s,other)||math.dot(tangent,candidate.Tangent)>-.9f||
                       s.StartNodeId==other.StartNodeId||s.StartNodeId==other.EndNodeId||s.EndNodeId==other.StartNodeId||s.EndNodeId==other.EndNodeId)continue;
                    var delta=p.xz-candidate.Point.xz;var longitudinal=Math.Abs(math.dot(delta,tangent));
                    var distance=math.lengthsq(delta);if(longitudinal>7||distance>=best||distance>80*80)continue;
                    best=distance;partner=candidate;
                }
                if(!partner.HasValue)continue;
                var q=partner.Value;var signed=math.dot(p.xz-q.Point.xz,normal);
                var required=(s.NativeWidth+source[q.Edge].NativeWidth)*.5f+2;
                var deficit=required-Math.Abs(signed);if(deficit<=.1f)continue;
                var shift=deficit*.5f;if(shift>24){shift=24;limited++;}
                shifts[j]=normal*(signed<0?-shift:shift);paired=true;contacts++;
            }
            if(!paired)continue;requested[i]=shifts;
            void Anchor(long node,float2 delta){endpoints.TryGetValue(node,out var old);endpoints[node]=(old.Sum+delta,old.Count+1);}
            Anchor(s.StartNodeId,shifts[0]);Anchor(s.EndNodeId,shifts[3]);
        }
        float3 Move(long node)=>endpoints.TryGetValue(node,out var v)?new float3(v.Sum.x/v.Count,0,v.Sum.y/v.Count):float3.zero;
        for(var i=0;i<result.Length;i++)
        {
            var s=source[i];if(s.Kind!=NetworkKind.Road)continue;var da=Move(s.StartNodeId);var db=Move(s.EndNodeId);
            var c1=s.CourseControl1+da;var c2=s.CourseControl2+db;
            if(requested.TryGetValue(i,out var shifts))
            {
                // Interpolate the immutable paired station displacement at 1/3
                // and 2/3. Straight, equally spaced corridors stay straight.
                var q1=27*shifts[1]-8*da.xz-db.xz;var q2=27*shifts[2]-da.xz-8*db.xz;
                var d1=(2*q1-q2)/18;var d2=(2*q2-q1)/18;
                // An unmatched terminal must not make a long handle swing out.
                if(math.length(d1)>24||math.length(d2)>24)
                {d1=math.lerp(da.xz,db.xz,1f/3);d2=math.lerp(da.xz,db.xz,2f/3);limited++;}
                c1=s.CourseControl1+new float3(d1.x,0,d1.y);c2=s.CourseControl2+new float3(d2.x,0,d2.y);
            }
            result[i]=s with{Start=s.Start+da,End=s.End+db,Control1=c1-s.StartCurveOffset,Control2=c2-s.EndCurveOffset,FinalDeckCurve=null};
        }
        log.Count("Carriageway paired station corrections",contacts);
        log.Diagnostic($"CARRIAGEWAY CORRIDOR FIT pairedSections={requested.Count}; sharedAnchors={endpoints.Count}; stationCorrections={contacts}; boundedCorrections={limited}; passes=1; cumulativeRepulsion=false; maximumDisplacementMeters=24; medianMeters=2; residualsAdvisory=true; nativeMeshesNotVerified=true");
        return result;
    }
}
