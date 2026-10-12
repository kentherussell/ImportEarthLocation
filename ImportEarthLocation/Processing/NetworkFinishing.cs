using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

public static class RoundaboutPlanner
{
    private static float[] _stockRadii=Array.Empty<float>();
    public static void ConfigureStockRadii(IEnumerable<float> radii) => _stockRadii=radii.Distinct().OrderBy(r=>r).Take(3).ToArray();
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
        var extraArcs=new Dictionary<PreparedSegment,List<PreparedSegment>>();
        var removed=new HashSet<PreparedSegment>();var remap=new Dictionary<long,(long id,float3 point,float radius)>();
        var ringPositions=new Dictionary<long,float3>();
        var circles=new Dictionary<long,(float3 center,float radius)>();
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
            if(radius<3)continue;
            var circular=edges.All(s=>Math.Abs(math.distance(s.Start.xz,center.xz)-radius)<=Math.Max(2f,radius*.2f));
            var approaches=ids.SelectMany(id=>incident[id]).Where(s=>!group.Contains(s)).Distinct().ToArray();
            var stockRadii=_stockRadii;
            var stockRadius=stockRadii.Length==0?0:stockRadii.OrderBy(r=>Math.Abs(r-radius)).ThenByDescending(r=>r).First();
            if(circular&&stockRadius>0&&radius<=stockRadii[stockRadii.Length-1] && approaches.Length>=3 && approaches.Length<=6 && approaches.All(s=>s.Kind==NetworkKind.Road&&!s.OneWay&&!s.Bridge&&!s.Tunnel&&!s.ProtectedLink&&s.Layer==edges[0].Layer&&!(ids.Contains(s.StartNodeId)&&ids.Contains(s.EndNodeId))&&math.distance(s.Start.xz,s.End.xz)>stockRadius+12))
            {
                while(allIds.Contains(next))next++;var node=next++;allIds.Add(node);
                foreach(var id in ids)remap[id]=(node,center,stockRadius);
                removed.UnionWith(group);compact++;
                log.Diagnostic($"ROUNDABOUT native-node planned; sourceWays={string.Join(",",edges.Select(s=>s.WayId).Distinct())}; sourceRadiusMeters={radius:0.##}; selectedStockCenterlineRadiusMeters={stockRadius}; approaches={approaches.Length}; nativeRadiusDerivedFromStockGeometry=true");
                continue;
            }
            foreach(var s in edges)
            {
                replacements[s]=s with{Classification="primary",OneWay=true,Lanes=2,Control1=math.lerp(s.Start,s.End,1f/3),Control2=math.lerp(s.Start,s.End,2f/3)};
                if(!circular)
                {
                    var before=byNode[s.StartNodeId].First(e=>e!=s);var after=byNode[s.EndNodeId].First(e=>e!=s);
                    var p=before.StartNodeId==s.StartNodeId?before.End:before.Start;var q=after.StartNodeId==s.EndNodeId?after.End:after.Start;
                    var irregularReach=math.distance(s.Start.xz,s.End.xz)/3;
                    var irregularStart=math.normalizesafe((s.End-p).xz);var irregularEnd=math.normalizesafe((q-s.Start).xz);
                    replacements[s]=s with{Classification="primary",OneWay=true,Lanes=2,Control1=s.Start+new float3(irregularStart.x,0,irregularStart.y)*irregularReach,Control2=s.End-new float3(irregularEnd.x,0,irregularEnd.y)*irregularReach,SourceGeometry=new[]{s.Start,s.End}};smooth++;
                    continue;
                }
                var a=s.Start.xz-center.xz;var b=s.End.xz-center.xz;var cross=a.x*b.y-a.y*b.x;
                var angle=(float)Math.Acos(math.clamp(math.dot(math.normalizesafe(a),math.normalizesafe(b)),-1,1));
                var start=center+new float3(math.normalizesafe(a).x*radius,s.Start.y-center.y,math.normalizesafe(a).y*radius);
                var end=center+new float3(math.normalizesafe(b).x*radius,s.End.y-center.y,math.normalizesafe(b).y*radius);
                ringPositions[s.StartNodeId]=start;ringPositions[s.EndNodeId]=end;
                circles[s.StartNodeId]=(center,radius);circles[s.EndNodeId]=(center,radius);
                if(angle<.001f)
                {replacements[s]=replacements[s] with{Start=start,End=end,Control1=math.lerp(start,end,1f/3),Control2=math.lerp(start,end,2f/3)};continue;}
                if(angle>(float)Math.PI/2+.001f)
                {
                    var ringSign=edges.Sum(e=>(e.Start.x-center.x)*(e.End.z-center.z)-(e.Start.z-center.z)*(e.End.x-center.x))>=0?1f:-1f;
                    var arcSign=Math.Abs(cross)<.01f?ringSign:cross>=0?1f:-1f;
                    var beginAngle=(float)Math.Atan2(a.y,a.x);var arcCount=(int)Math.Ceiling(angle/((float)Math.PI/2));
                    var arcStart=start;var arcStartId=s.StartNodeId;var extras=new List<PreparedSegment>();
                    for(var k=1;k<=arcCount;k++)
                    {
                        var theta=beginAngle+arcSign*angle*k/arcCount;
                        var arcEnd=k==arcCount?end:center+new float3((float)Math.Cos(theta)*radius,math.lerp(start.y,end.y,k/(float)arcCount)-center.y,(float)Math.Sin(theta)*radius);
                        var arcEndId=s.EndNodeId;if(k<arcCount){while(allIds.Contains(next))next++;arcEndId=next++;allIds.Add(arcEndId);circles[arcEndId]=(center,radius);}
                        var va=arcStart.xz-center.xz;var vb=arcEnd.xz-center.xz;
                        var at=math.normalizesafe(new float2(-va.y,va.x))*arcSign;var bt=math.normalizesafe(new float2(-vb.y,vb.x))*arcSign;
                        var arcReach=4f/3*radius*(float)Math.Tan(angle/arcCount/4);
                        var part=s with{Classification="primary",OneWay=true,Lanes=2,Start=arcStart,End=arcEnd,StartNodeId=arcStartId,EndNodeId=arcEndId,
                            Control1=arcStart+new float3(at.x,0,at.y)*arcReach,Control2=arcEnd-new float3(bt.x,0,bt.y)*arcReach,SourceGeometry=new[]{arcStart,arcEnd}};
                        if(k==1)replacements[s]=part;else extras.Add(part);arcStart=arcEnd;arcStartId=arcEndId;smooth++;
                    }
                    extraArcs[s]=extras;continue;
                }
                var sign=cross>=0?1f:-1f;var ta=math.normalizesafe(new float2(-a.y,a.x))*sign;var tb=math.normalizesafe(new float2(-b.y,b.x))*sign;
                var reach=4f/3*radius*(float)Math.Tan(angle/4);
                var c1=start+new float3(ta.x,0,ta.y)*reach;var c2=end-new float3(tb.x,0,tb.y)*reach;
                var candidate=s with{Classification="primary",OneWay=true,Lanes=2,Start=start,End=end,Control1=c1,Control2=c2,SourceGeometry=new[]{start,end}};
                replacements[s]=candidate;smooth++;
            }
        }
        var result=new List<PreparedSegment>();
        foreach(var original in source)
        {
            if(removed.Contains(original))continue;
            var s=replacements.TryGetValue(original,out var fitted)?fitted:original;
            if(!original.Roundabout){if(ringPositions.TryGetValue(s.StartNodeId,out var ringStart))s=s with{Start=ringStart,Control1=null,Control2=null};if(ringPositions.TryGetValue(s.EndNodeId,out var ringEnd))s=s with{End=ringEnd,Control1=null,Control2=null};}
            if(remap.TryGetValue(s.StartNodeId,out var a))s=s with{StartNodeId=a.id,Start=a.point,StartRoundaboutRadius=a.radius,Control1=null,Control2=null,SourceGeometry=Array.Empty<float3>()};
            if(remap.TryGetValue(s.EndNodeId,out var b))s=s with{EndNodeId=b.id,End=b.point,EndRoundaboutRadius=b.radius,Control1=null,Control2=null,SourceGeometry=Array.Empty<float3>()};
            if(s.Kind==NetworkKind.Road&&!s.Bridge&&!s.Tunnel)
            {
                if(s.StartMiniRoundabout&&incident.TryGetValue(s.StartNodeId,out var startArms)&&startArms.Count>=3)s=s with{StartRoundaboutRadius=Math.Max(s.StartRoundaboutRadius,6)};
                if(s.EndMiniRoundabout&&incident.TryGetValue(s.EndNodeId,out var endArms)&&endArms.Count>=3)s=s with{EndRoundaboutRadius=Math.Max(s.EndRoundaboutRadius,6)};
            }
            result.Add(s);
            if(extraArcs.TryGetValue(original,out var extras))result.AddRange(extras);
        }
        // Compact only degree-two circle nodes; never remove an approach attachment.
        var compactedArcs=0;var compactedIrregular=0;
        var arcRemoved=new HashSet<PreparedSegment>();
        var degree=result.SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}).GroupBy(n=>n).ToDictionary(g=>g.Key,g=>g.Count());
        var outgoing=result.Where(s=>s.Roundabout).GroupBy(s=>s.StartNodeId).ToDictionary(g=>g.Key,g=>g.ToArray());
        for(var i=0;i<result.Count;i++)
        {
            token.ThrowIfCancellationRequested();if(arcRemoved.Contains(result[i]))continue;
            while(true)
            {
                var a=result[i];if(!a.Roundabout||degree[a.EndNodeId]!=2||!outgoing.TryGetValue(a.EndNodeId,out var nextArcs)||nextArcs.Length!=1)break;
                var b=nextArcs[0];if(a==b||arcRemoved.Contains(b)||a.StartNodeId==b.EndNodeId)break;
                if(!circles.TryGetValue(a.EndNodeId,out var circle))
                {
                    if(!a.Control1.HasValue||!b.Control2.HasValue||math.distance(a.Start.xz,b.End.xz)>160)break;
                    var taIrregular=math.normalizesafe((a.Control1.Value-a.Start).xz);var tbIrregular=math.normalizesafe((b.End-b.Control2.Value).xz);
                    if(math.dot(taIrregular,tbIrregular)<.6f)break;
                    var irregularReach=math.distance(a.Start.xz,b.End.xz)/3;
                    var originalPoints=a.SourceGeometry.Concat(b.SourceGeometry.Skip(1)).ToArray();
                    var irregular=a with{End=b.End,EndNodeId=b.EndNodeId,Control1=a.Start+new float3(taIrregular.x,0,taIrregular.y)*irregularReach,Control2=b.End-new float3(tbIrregular.x,0,tbIrregular.y)*irregularReach,
                        SourceWayIds=a.SourceWayIds.Concat(b.SourceWayIds).Distinct().ToArray(),OriginalGeometry=a.OriginalGeometry.Concat(b.OriginalGeometry).ToArray(),SourceGeometry=originalPoints};
                    if(!NetworkNormalizer.WithinTolerance(irregular,3f))break;
                    result[i]=irregular;outgoing[a.StartNodeId]=new[]{irregular};outgoing.Remove(b.StartNodeId);arcRemoved.Add(b);compactedIrregular++;continue;
                }
                var startVector=a.Start.xz-circle.center.xz;var endVector=b.End.xz-circle.center.xz;
                var startAngle=(float)Math.Atan2(startVector.y,startVector.x);var endAngle=(float)Math.Atan2(endVector.y,endVector.x);
                var firstDelta=a.End.xz-a.Start.xz;var sign=startVector.x*firstDelta.y-startVector.y*firstDelta.x>=0?1f:-1f;
                var sweep=(endAngle-startAngle)*sign;while(sweep<0)sweep+=2*(float)Math.PI;
                if(sweep>(float)Math.PI/2+.001f)break;
                var reach=4f/3*circle.radius*(float)Math.Tan(sweep/4);
                var ta=math.normalizesafe(new float2(-startVector.y,startVector.x))*sign;var tb=math.normalizesafe(new float2(-endVector.y,endVector.x))*sign;
                var merged=a with{End=b.End,EndNodeId=b.EndNodeId,Control1=a.Start+new float3(ta.x,0,ta.y)*reach,Control2=b.End-new float3(tb.x,0,tb.y)*reach,
                    SourceWayIds=a.SourceWayIds.Concat(b.SourceWayIds).Distinct().ToArray(),OriginalGeometry=a.OriginalGeometry.Concat(b.OriginalGeometry).ToArray(),SourceGeometry=new[]{a.Start,b.End}};
                result[i]=merged;outgoing[a.StartNodeId]=new[]{merged};outgoing.Remove(b.StartNodeId);arcRemoved.Add(b);compactedArcs++;
            }
        }
        result.RemoveAll(arcRemoved.Contains);
        log.Count("Roundabout degree-two nodes simplified",compactedArcs);
        log.Count("Irregular roundabout degree-two nodes simplified",compactedIrregular);
        log.Diagnostic($"ROUNDABOUT PREPARATION curvedRingSections={smooth}; nativeSmallJunctions={compact}; circularizedRingNodes={ringPositions.Count}; taggedMiniApproaches={result.Count(s=>s.StartMiniRoundabout||s.EndMiniRoundabout)}; incompleteRingsPreserved=true; oversizedRingPalette=regular-oneway-two-lanes");
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
            token.ThrowIfCancellationRequested();var s=result[index];if(s.Kind==NetworkKind.Rail){result[index]=s with{ExplicitWallSides=NetworkSides.None,RetainingWallSides=NetworkSides.None,QuaySides=NetworkSides.None};continue;}if(s.Bridge||s.Tunnel)continue;
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
