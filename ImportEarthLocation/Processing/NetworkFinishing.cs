using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

public static class RoundaboutPlanner
{
    public static IReadOnlyList<PreparedSegment> Prepare(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token)
    {
        var rings=source.Where(s=>s.Roundabout&&s.Kind==NetworkKind.Road&&!s.Bridge&&!s.Tunnel).ToArray();
        var incident=new Dictionary<long,List<PreparedSegment>>();
        foreach(var s in source)foreach(var id in new[]{s.StartNodeId,s.EndNodeId})
        {if(!incident.TryGetValue(id,out var list))incident[id]=list=new();list.Add(s);}
        var byNode=new Dictionary<long,List<PreparedSegment>>();
        foreach(var s in rings)foreach(var id in new[]{s.StartNodeId,s.EndNodeId})
        {if(!byNode.TryGetValue(id,out var list))byNode[id]=list=new();list.Add(s);}
        var visited=new HashSet<PreparedSegment>();var replacements=new Dictionary<PreparedSegment,PreparedSegment>();
        var removed=new HashSet<PreparedSegment>();var remap=new Dictionary<long,(long id,float3 point,float radius)>();
        var allIds=new HashSet<long>(source.SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}));var next=long.MinValue+2000000;
        var smooth=0;var compact=0;
        foreach(var seed in rings)
        {
            token.ThrowIfCancellationRequested();if(visited.Contains(seed))continue;
            var group=new HashSet<PreparedSegment>();var queue=new List<PreparedSegment>{seed};
            for(var q=0;q<queue.Count;q++){var s=queue[q];if(!group.Add(s))continue;visited.Add(s);foreach(var id in new[]{s.StartNodeId,s.EndNodeId})foreach(var e in byNode[id])if(!group.Contains(e))queue.Add(e);}
            var edges=group.OrderBy(s=>s.WayId).ThenBy(s=>s.StartNodeId).ThenBy(s=>s.EndNodeId).ToArray();var ids=new HashSet<long>(edges.SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}));
            if(ids.Any(id=>byNode[id].Count!=2)||edges.Length<3)continue; // Incomplete rings must not collapse.
            var length=edges.Sum(s=>math.distance(s.Start.xz,s.End.xz));if(length<10)continue;
            var center=default(float3);
            foreach(var s in edges)center+=(s.Start+s.End)*.5f*math.distance(s.Start.xz,s.End.xz)/length;
            var radius=edges.Sum(s=>(math.distance(s.Start.xz,center.xz)+math.distance(s.End.xz,center.xz))*.5f*math.distance(s.Start.xz,s.End.xz))/length;
            if(radius<3||edges.Any(s=>Math.Abs(math.distance(s.Start.xz,center.xz)-radius)>Math.Max(1.5f,radius*.12f)))continue;
            var approaches=ids.SelectMany(id=>incident[id]).Where(s=>!group.Contains(s)).Distinct().ToArray();
            if(radius<=18 && approaches.Length>=3 && approaches.Length<=6 && approaches.All(s=>s.Kind==NetworkKind.Road&&!s.OneWay&&!s.Bridge&&!s.Tunnel&&!s.ProtectedLink&&s.Layer==edges[0].Layer&&!(ids.Contains(s.StartNodeId)&&ids.Contains(s.EndNodeId))&&math.distance(s.Start.xz,s.End.xz)>radius+12))
            {
                while(allIds.Contains(next))next++;var node=next++;allIds.Add(node);
                foreach(var id in ids)remap[id]=(node,center,radius);
                removed.UnionWith(group);compact++;
                log.Diagnostic($"ROUNDABOUT native-node planned; sourceWays={string.Join(",",edges.Select(s=>s.WayId).Distinct())}; sourceRadiusMeters={radius:0.##}; approaches={approaches.Length}; nativeRadiusDerivedFromComposition=true");
                continue;
            }
            foreach(var s in edges)
            {
                var a=s.Start.xz-center.xz;var b=s.End.xz-center.xz;var cross=a.x*b.y-a.y*b.x;
                var angle=(float)Math.Acos(math.clamp(math.dot(math.normalizesafe(a),math.normalizesafe(b)),-1,1));
                if(angle>.8f||angle<.001f)continue;
                var sign=cross>=0?1f:-1f;var ta=math.normalizesafe(new float2(-a.y,a.x))*sign;var tb=math.normalizesafe(new float2(-b.y,b.x))*sign;
                var reach=4f/3*radius*(float)Math.Tan(angle/4);
                var c1=s.Start+new float3(ta.x,0,ta.y)*reach;var c2=s.End-new float3(tb.x,0,tb.y)*reach;
                var candidate=s with{Control1=c1,Control2=c2,SourceGeometry=new[]{s.Start,s.End}};
                if(!NetworkNormalizer.WithinTolerance(candidate))continue;
                replacements[s]=candidate;smooth++;
            }
        }
        var result=new List<PreparedSegment>();
        foreach(var original in source)
        {
            if(removed.Contains(original))continue;
            var s=replacements.TryGetValue(original,out var fitted)?fitted:original;
            if(remap.TryGetValue(s.StartNodeId,out var a))s=s with{StartNodeId=a.id,Start=a.point,StartRoundaboutRadius=a.radius,Control1=null,Control2=null,SourceGeometry=Array.Empty<float3>()};
            if(remap.TryGetValue(s.EndNodeId,out var b))s=s with{EndNodeId=b.id,End=b.point,EndRoundaboutRadius=b.radius,Control1=null,Control2=null,SourceGeometry=Array.Empty<float3>()};
            if(s.Kind==NetworkKind.Road&&!s.Bridge&&!s.Tunnel)
            {
                if(s.StartMiniRoundabout&&incident.TryGetValue(s.StartNodeId,out var startArms)&&startArms.Count>=3)s=s with{StartRoundaboutRadius=Math.Max(s.StartRoundaboutRadius,6)};
                if(s.EndMiniRoundabout&&incident.TryGetValue(s.EndNodeId,out var endArms)&&endArms.Count>=3)s=s with{EndRoundaboutRadius=Math.Max(s.EndRoundaboutRadius,6)};
            }
            result.Add(s);
        }
        log.Diagnostic($"ROUNDABOUT PREPARATION curvedRingSections={smooth}; nativeSmallJunctions={compact}; taggedMiniApproaches={result.Count(s=>s.StartMiniRoundabout||s.EndMiniRoundabout)}; incompleteAndNonCircularRingsPreserved=true");
        return result;
    }
}

public static class NetworkCutPlanner
{
    private readonly record struct Barrier(float3 A,float3 B,float HalfWidth);
    public static IReadOnlyList<PreparedSegment> Prepare(IReadOnlyList<PreparedSegment> source,ushort[] raster,float scale,double width,
        IEnumerable<float3[]> protectedAreas,IEnumerable<float3[]> protectedLines,IEnumerable<float3[]> explicitWalls,ImportLog log,CancellationToken token)
    {
        var n=(int)Math.Sqrt(raster.Length);
        if(n<2||n*n!=raster.Length||!math.isfinite(scale)||scale<=0||double.IsNaN(width)||double.IsInfinity(width)||width<=0)throw new ArgumentException("Cut planning requires a finite map and square terrain raster.");
        var cell=(float)(width/(n-1));var halfMap=(float)(width/2);
        float Ground(float3 p){var x=math.clamp((p.x/(float)width+.5f)*(n-1),0,n-1);var z=math.clamp((p.z/(float)width+.5f)*(n-1),0,n-1);var ix=(int)x;var iz=(int)z;return math.lerp(math.lerp(raster[iz*n+ix],raster[iz*n+Math.Min(ix+1,n-1)],x-ix),math.lerp(raster[Math.Min(iz+1,n-1)*n+ix],raster[Math.Min(iz+1,n-1)*n+Math.Min(ix+1,n-1)],x-ix),z-iz)/65535f*scale;}
        var grid=new Dictionary<(int,int),List<(int owner,Barrier barrier,bool wall)>>();
        void Add(int owner,Barrier b,bool wall)
        {
            token.ThrowIfCancellationRequested();
            var margin=96+Math.Max(0,b.HalfWidth);
            var lo=math.max(math.min(b.A.xz,b.B.xz)-margin,new float2(-halfMap-128));var hi=math.min(math.max(b.A.xz,b.B.xz)+margin,new float2(halfMap+128));
            if(math.any(lo>hi))return;
            for(var z=(int)Math.Floor(lo.y/128);z<=(int)Math.Floor(hi.y/128);z++)for(var x=(int)Math.Floor(lo.x/128);x<=(int)Math.Floor(hi.x/128);x++)
            {if(!grid.TryGetValue((x,z),out var list))grid[(x,z)]=list=new();list.Add((owner,b,wall));}
        }
        for(var index=0;index<source.Count;index++)
        {
            var s=source[index];if(s.Tunnel)continue;var steps=Math.Max(1,(int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(s)/24));var previous=NetworkNormalizer.Evaluate(s,0);
            for(var j=1;j<=steps;j++){token.ThrowIfCancellationRequested();var p=NetworkNormalizer.Evaluate(s,j/(float)steps);Add(index,new(previous,p,Math.Max(4,s.NativeWidth*.5f)),false);previous=p;}
        }
        foreach(var line in protectedLines.Concat(protectedAreas))for(var j=1;j<line.Length;j++)Add(-1,new(line[j-1],line[j],cell),false);
        foreach(var line in explicitWalls)for(var j=1;j<line.Length;j++)Add(-1,new(line[j-1],line[j],1),true);
        var result=source.ToArray();var inferred=0;var explicitCount=0;var taggedCuts=0;
        for(var index=0;index<result.Length;index++)
        {
            token.ThrowIfCancellationRequested();var s=result[index];if(s.Bridge||s.Tunnel)continue;
            var sides=s.ExplicitWallSides;var half=Math.Max(4,s.NativeWidth*.5f);var tests=9;
            foreach(var left in new[]{true,false})
            {
                var side=left?NetworkSides.Left:NetworkSides.Right;var blocked=0;var tagged=0;var cuts=0;
                var cutEvidence=(s.ExplicitCutSides&side)!=0;if(cutEvidence)taggedCuts++;
                for(var j=1;j<=tests;j++)
                {
                    var t=j/(float)(tests+1);var p=NetworkNormalizer.Evaluate(s,t);var tangent=math.normalizesafe((NetworkNormalizer.Evaluate(s,Math.Min(1,t+.01f))-NetworkNormalizer.Evaluate(s,Math.Max(0,t-.01f))).xz);
                    var normal=new float3(-tangent.y,0,tangent.x)*(left?1:-1);var bank=p+normal*(half+cell);var depth=Ground(bank)-p.y;
                    // cutting=yes is evidence of an open cut, not a demand for
                    // walls. It permits a shallower DEM cut to be classified;
                    // the room/conflict test still decides the side treatment.
                    if(depth<(cutEvidence?.5f:1.5f))continue;cuts++;
                    var reach=Math.Min(48,Math.Max(cell*1.5f,depth*2.5f));
                    if(!grid.TryGetValue(((int)Math.Floor(p.x/128),(int)Math.Floor(p.z/128)),out var nearby))continue;
                    var hit=false;var wallHit=false;
                    foreach(var item in nearby)
                    {
                        if(item.owner==index)continue;
                        var b=item.barrier;var delta=b.B.xz-b.A.xz;var u=math.clamp(math.dot(bank.xz-b.A.xz,delta)/Math.Max(.001f,math.lengthsq(delta)),0,1);var q=math.lerp(b.A,b.B,u);var lateral=math.dot(q-p,normal);
                        if(lateral<half-1||lateral>half+reach+b.HalfWidth||math.distance(bank.xz,q.xz)>reach+b.HalfWidth)continue;
                        if(item.wall){if(lateral<=half+12)wallHit=true;continue;}
                        if(item.owner>=0)
                        {
                            var other=source[item.owner];
                            if(other.StartNodeId==s.StartNodeId||other.EndNodeId==s.StartNodeId||other.StartNodeId==s.EndNodeId||other.EndNodeId==s.EndNodeId)continue;
                            if(Math.Abs(q.y-p.y)>depth+3)continue;
                        }
                        hit=true;
                    }
                    if(hit)blocked++;if(wallHit)tagged++;
                }
                if(tagged>=2){sides|=side;explicitCount++;}
                else if(cuts>=3&&blocked>=Math.Max(2,cuts/2)){sides|=side;inferred++;}
            }
            result[index]=s with{RetainingWallSides=sides};
        }
        log.Diagnostic($"CUT SIDES explicitCutSides={taggedCuts}; explicitWallMatches={explicitCount}; inferredConstrainedSides={inferred}; wallSections={result.Count(s=>s.RetainingWallSides!=NetworkSides.None)}; roomyCutsUseEarthSlopes=true; sidesIndependent=true; tunnelsNeverConvertedToCuts=true; nativeWallTransitionsRequireGameReview=true");
        return result;
    }
}
