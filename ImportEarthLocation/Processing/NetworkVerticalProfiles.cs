using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

public static class NetworkVerticalProfiles
{
    public const float AbsoluteRoadGrade = .348f;
    public static float MaximumGrade(PreparedSegment segment)
    {
        var a=segment.Start;var b=segment.Control1??math.lerp(a,segment.End,1f/3);var c=segment.Control2??math.lerp(a,segment.End,2f/3);var d=segment.End;
        var maximum=0f;
        for(var i=0;i<=256;i++)
        {var t=i/256f;var u=1-t;var derivative=3*u*u*(b-a)+6*u*t*(c-b)+3*t*t*(d-c);var horizontal=math.length(derivative.xz);var grade=horizontal<.00001f?(Math.Abs(derivative.y)<.00001f?0:float.PositiveInfinity):Math.Abs(derivative.y)/horizontal;maximum=Math.Max(maximum,grade);}
        return maximum;
    }

    // Quadratic Bezier derivative controls bound every derivative in an interval.
    // Subdivision tightens the bounds; an uncertain interval is rejected, never
    // accepted on the strength of discrete samples that might miss a sharp peak.
    public static bool WithinGradeCeiling(PreparedSegment segment)
    {
        var a=segment.Start;var b=segment.Control1??math.lerp(a,segment.End,1f/3);var c=segment.Control2??math.lerp(a,segment.End,2f/3);var d=segment.End;
        return Bound(3*(b-a),3*(c-b),3*(d-c),0);
        bool Bound(float3 p,float3 q,float3 r,int depth)
        {
            if(!math.all(math.isfinite(p))||!math.all(math.isfinite(q))||!math.all(math.isfinite(r)))return false;
            var min=math.min(p,math.min(q,r));var max=math.max(p,math.max(q,r));
            var vertical=Math.Max(Math.Abs(min.y),Math.Abs(max.y));
            if(vertical<.000001f)return true;
            var dx=min.x>0?min.x:max.x<0?-max.x:0;var dz=min.z>0?min.z:max.z<0?-max.z:0;
            var horizontal=(float)Math.Sqrt(dx*dx+dz*dz);
            if(vertical <= AbsoluteRoadGrade*horizontal+.000001f)return true;
            var pq=(p+q)*.5f;var qr=(q+r)*.5f;var middle=(pq+qr)*.5f;
            foreach(var derivative in new[]{p,middle,r})
                if(Math.Abs(derivative.y)>AbsoluteRoadGrade*math.length(derivative.xz)+.000001f)return false;
            if(depth>=14)return false;
            return Bound(p,pq,middle,depth+1)&&Bound(middle,qr,r,depth+1);
        }
    }

    public static IReadOnlyList<PreparedSegment> Prepare(IReadOnlyList<PreparedSegment> source, ushort[] heightmap, float scale, double width, ImportLog log, CancellationToken token)
    {
        if(source.Count==0)return source;
        var roads=source.Where(s=>s.Kind==NetworkKind.Road).ToArray();
        var heights=new Dictionary<long,float>();var samples=new Dictionary<long,List<float>>();
        float Ground(float3 p)=>Bilinear(p.x,p.z,heightmap,scale,width);
        foreach(var s in roads) foreach(var pair in new[]{(s.StartNodeId,s.Start),(s.EndNodeId,s.End)})
        {if(!samples.TryGetValue(pair.Item1,out var values))samples[pair.Item1]=values=new List<float>();values.Add(Ground(pair.Item2)+(s.Bridge?Math.Max(8,s.Layer*6):s.Tunnel?0:s.Layer*6));}
        foreach(var pair in samples){pair.Value.Sort();heights[pair.Key]=pair.Value[pair.Value.Count/2];}
        var structureHeights=new Dictionary<long,float>();
        foreach(var tunnel in new[]{false,true})
        {
            var structures=roads.Where(s=>tunnel?s.Tunnel:s.Bridge).ToArray();var adjacency=new Dictionary<long,List<PreparedSegment>>();
            foreach(var s in structures)foreach(var id in new[]{s.StartNodeId,s.EndNodeId}){if(!adjacency.TryGetValue(id,out var list))adjacency[id]=list=new List<PreparedSegment>();list.Add(s);}
            var visited=new HashSet<PreparedSegment>();
            foreach(var seed in structures)
            {
                token.ThrowIfCancellationRequested();if(!visited.Add(seed))continue;
                var group=new List<PreparedSegment>();var queue=new List<PreparedSegment>{seed};var identities=new HashSet<PreparedSegment>();
                for(var index=0;index<queue.Count;index++){var s=queue[index];if(!identities.Add(s))continue;group.Add(s);visited.Add(s);foreach(var id in new[]{s.StartNodeId,s.EndNodeId})foreach(var next in adjacency[id])if(!identities.Contains(next))queue.Add(next);}
                var degree=new Dictionary<long,int>();var positions=new Dictionary<long,float3>();foreach(var s in group){foreach(var id in new[]{s.StartNodeId,s.EndNodeId}){degree.TryGetValue(id,out var count);degree[id]=count+1;}positions[s.StartNodeId]=s.Start;positions[s.EndNodeId]=s.End;}
                var portals=degree.Where(p=>p.Value==1).Select(p=>p.Key).OrderBy(n=>n).ToArray();
                if(portals.Length!=2||degree.Values.Any(d=>d>2)){log.Count(tunnel?"Tunnel profiles unresolved":"Bridge profiles unresolved");continue;}
                var distances=new Dictionary<long,float>{{portals[0],0}};var ordered=new List<long>{portals[0]};
                for(var index=0;index<ordered.Count;index++)foreach(var s in adjacency[ordered[index]])
                {if(!identities.Contains(s))continue;var next=s.StartNodeId==ordered[index]?s.EndNodeId:s.StartNodeId;if(distances.ContainsKey(next))continue;distances[next]=distances[ordered[index]]+ArcLength(s);ordered.Add(next);}
                var total=distances[portals[1]];if(total<.01f)continue;
                var offset=tunnel?0:group.Max(s=>Math.Max(8,s.Layer*6));var start=Ground(positions[portals[0]])+offset;var end=Ground(positions[portals[1]])+offset;
                foreach(var point in distances)structureHeights[point.Key]=math.lerp(start,end,point.Value/total);
            }
        }
        foreach(var pair in structureHeights)heights[pair.Key]=pair.Value;
        // Spread necessary bridge rise along actual connected approach topology, not a nearby road.
        var approaches=new Dictionary<long,List<PreparedSegment>>();foreach(var s in roads.Where(s=>!s.Bridge&&!s.Tunnel))foreach(var id in new[]{s.StartNodeId,s.EndNodeId}){if(!approaches.TryGetValue(id,out var list))approaches[id]=list=new List<PreparedSegment>();list.Add(s);}
        var riseQueue=structureHeights.Keys.ToList();var rises=0;
        for(var cursor=0;cursor<riseQueue.Count;cursor++)
        {
            token.ThrowIfCancellationRequested();var id=riseQueue[cursor];if(!approaches.TryGetValue(id,out var edges))continue;
            foreach(var s in edges)
            {var other=s.StartNodeId==id?s.EndNodeId:s.StartNodeId;var required=heights[id]-ArcLength(s)*.10f;
             if(required>heights[other]+.01f&&required<=samples[other].Min()+30){heights[other]=required;riseQueue.Add(other);rises++;}}
            if(riseQueue.Count>roads.Length*4+16)throw new InvalidOperationException("Bridge approach profile constraints failed to converge; preparation stopped.");
        }
        if(rises>0)log.Count("Bridge grade tie-in nodes adjusted",rises);
        var result=new List<PreparedSegment>();var rejected=0;float maximum=0;
        foreach(var original in source)
        {
            token.ThrowIfCancellationRequested();
            if(original.Kind!=NetworkKind.Road){result.Add(original);continue;}
            var s=original;var a=s.Start;var d=s.End;a.y=heights[s.StartNodeId];d.y=heights[s.EndNodeId];
            var values=new float[33];var path=new float[33];
            var startLift=a.y-Ground(a);var endLift=d.y-Ground(d);
            for(var i=0;i<values.Length;i++)
            {var p=NetworkNormalizer.Evaluate(s,i/32f);if(i>0)path[i]=path[i-1]+math.distance(p.xz,NetworkNormalizer.Evaluate(s,(i-1)/32f).xz);values[i]=Ground(p)+math.lerp(startLift,endLift,i/32f);}
            if(path[32]<.01f){rejected++;continue;}
            if(s.Bridge||s.Tunnel)for(var i=0;i<values.Length;i++)values[i]=math.lerp(a.y,d.y,path[i]/path[32]);
            else
            {
                if(s.ExplicitIncline.HasValue && Math.Abs(a.y+s.ExplicitIncline.Value*path[32]-d.y)<=3)
                {for(var i=1;i<32;i++)values[i]=a.y+s.ExplicitIncline.Value*path[i];log.Count("Explicit incline profiles used");}
                var raw=(float[])values.Clone();for(var i=1;i<32;i++){var lo=Math.Min(raw[i-1],raw[i+1]);var hi=Math.Max(raw[i-1],raw[i+1]);if(raw[i]<lo-1||raw[i]>hi+1)values[i]=(raw[i-1]+raw[i+1])*.5f;}
            }
            values[0]=a.y;values[32]=d.y;
            var pieces = new List<PreparedSegment>();
            var fitted = FitPiece(0,1,0);
            var grade = pieces.Count == 0 ? float.PositiveInfinity : pieces.Max(MaximumGrade);
            if(!fitted || !math.isfinite(grade)||pieces.Any(piece=>!WithinGradeCeiling(piece)))
            {rejected++;log.Count("Roads rejected for impossible grade/profile");if(rejected<=25)log.Warn($"Road way {s.WayId} rejected: unsafe centerline grade/profile (peak {grade*100:0.0}%, ceiling 34.8%); affected bridge/ramp tie-in remains unresolved.");continue;}
            if((s.Classification.StartsWith("motorway",StringComparison.Ordinal)||s.Classification.StartsWith("trunk",StringComparison.Ordinal))&&grade>.12f)log.Count("Highway profiles requiring source review");
            maximum=Math.Max(maximum,grade);result.AddRange(pieces);log.Count(s.Bridge?"Bridge profiles fitted":s.ProtectedLink?"Ramp profiles fitted":"Road profiles fitted");

            float Target(float t){var p=t*32;var i=Math.Min(31,(int)p);return math.lerp(values[i],values[i+1],p-i);}
            long Node(float t){if(t==0)return s.StartNodeId;if(t==1)return s.EndNodeId;unchecked{return -1-(long)(((ulong)s.WayId*397UL ^ (ulong)s.StartNodeId*104729UL ^ (ulong)s.EndNodeId*1000003UL ^ math.hash(NetworkNormalizer.Evaluate(s,t).xz))%(ulong)long.MaxValue);}}
            bool FitPiece(float first,float last,int depth)
            {
                token.ThrowIfCancellationRequested();
                var curve=Subcurve(s,first,last);var start=curve.Start;var end=curve.End;start.y=Target(first);end.y=Target(last);
                double aa=0,ab=0,bb=0,ar=0,br=0;
                for(var i=1;i<32;i++){var t=i/32d;var u=1-t;var b1=3*u*u*t;var b2=3*u*t*t;var residual=Target(math.lerp(first,last,(float)t))-u*u*u*start.y-t*t*t*end.y;aa+=b1*b1;ab+=b1*b2;bb+=b2*b2;ar+=b1*residual;br+=b2*residual;}
                var determinant=aa*bb-ab*ab;var c1=curve.Control1!.Value;var c2=curve.Control2!.Value;
                c1.y=(float)((ar*bb-br*ab)/determinant);c2.y=(float)((br*aa-ar*ab)/determinant);
                var candidate=curve with{Start=start,End=end,Control1=c1,Control2=c2,StartNodeId=Node(first),EndNodeId=Node(last),StartElevation=start.y-Ground(start),EndElevation=end.y-Ground(end),HorizontalParent=first==0&&last==1?null:s,ParentStartT=first,ParentEndT=last};
                var error=0f;for(var i=0;i<=32;i++)error=Math.Max(error,Math.Abs(NetworkNormalizer.Evaluate(candidate,i/32f).y-Target(math.lerp(first,last,i/32f))));
                // Fall back to less aggressive local fitting rather than flatten broad relief.
                if((error>2 || !WithinGradeCeiling(candidate)) && depth<3 && ArcLength(candidate)>8)
                {log.Count("Vertical profiles locally refined");var middle=(first+last)*.5f;return FitPiece(first,middle,depth+1)&&FitPiece(middle,last,depth+1);}
                if(error>3)return false;pieces.Add(candidate);return true;
            }
        }
        if(roads.Length>0&&(rejected==roads.Length||roads.Length>20&&rejected>roads.Length/2))throw new InvalidOperationException($"Major vertical-profile failure: {rejected}/{roads.Length} roads exceed valid grade/profile limits. Current map unchanged.");
        if(roads.Length>0)log.Info($"Road vertical profiles: retained {roads.Length-rejected}, rejected {rejected}; maximum retained centerline grade {maximum*100:0.0}%; absolute ceiling 34.8%.");
        return result;
    }
    public static float ArcLength(PreparedSegment segment)
    {float length=0;var previous=NetworkNormalizer.Evaluate(segment,0);for(var i=1;i<=64;i++){var next=NetworkNormalizer.Evaluate(segment,i/64f);length+=math.distance(previous.xz,next.xz);previous=next;}return length;}
    private static PreparedSegment Subcurve(PreparedSegment s,float first,float last)
    {
        var a=s.Start;var b=s.Control1??math.lerp(s.Start,s.End,1f/3);var c=s.Control2??math.lerp(s.Start,s.End,2f/3);var d=s.End;
        var ab=math.lerp(a,b,last);var bc=math.lerp(b,c,last);var cd=math.lerp(c,d,last);var abc=math.lerp(ab,bc,last);var bcd=math.lerp(bc,cd,last);var end=math.lerp(abc,bcd,last);
        var ratio=first/last;var leftAB=math.lerp(a,ab,ratio);var leftBC=math.lerp(ab,abc,ratio);var leftCD=math.lerp(abc,end,ratio);var leftABC=math.lerp(leftAB,leftBC,ratio);var leftBCD=math.lerp(leftBC,leftCD,ratio);var start=math.lerp(leftABC,leftBCD,ratio);
        return s with{Start=start,End=end,Control1=leftBCD,Control2=leftCD};
    }
    private static float Bilinear(float x,float z,ushort[] raster,float scale,double width)
    {
        if(width<=0||!math.isfinite(scale)||scale<=0||raster.Length!=TerrainBuilder.Resolution*TerrainBuilder.Resolution)throw new InvalidOperationException("Road profile: invalid terrain buffer/scale.");
        var px=Numeric.Clamp((x/width+.5)*(TerrainBuilder.Resolution-1),0,TerrainBuilder.Resolution-1);var py=Numeric.Clamp((z/width+.5)*(TerrainBuilder.Resolution-1),0,TerrainBuilder.Resolution-1);
        var ix=(int)Math.Floor(px);var iy=(int)Math.Floor(py);var jx=Math.Min(ix+1,TerrainBuilder.Resolution-1);var jy=Math.Min(iy+1,TerrainBuilder.Resolution-1);var fx=(float)(px-ix);var fy=(float)(py-iy);
        return math.lerp(math.lerp(raster[iy*TerrainBuilder.Resolution+ix],raster[iy*TerrainBuilder.Resolution+jx],fx),math.lerp(raster[jy*TerrainBuilder.Resolution+ix],raster[jy*TerrainBuilder.Resolution+jx],fx),fy)/ushort.MaxValue*scale;
    }
}
