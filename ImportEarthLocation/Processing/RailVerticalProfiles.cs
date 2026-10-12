using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

public static class RailVerticalProfiles
{
    public static IReadOnlyList<PreparedSegment> Prepare(IReadOnlyList<PreparedSegment> source,ushort[] raster,float scale,double width,ImportLog log,CancellationToken token)
    {
        var rails=source.Where(s=>s.Kind==NetworkKind.Rail).ToArray();if(rails.Length==0)return source;
        var lengths=new Dictionary<PreparedSegment,float>();
        foreach(var s in rails)if(!lengths.ContainsKey(s))lengths[s]=NetworkVerticalProfiles.ArcLength(s);
        var n=(int)Math.Sqrt(raster.Length);
        float Ground(float3 p)
        {var x=math.clamp((p.x/(float)width+.5f)*(n-1),0,n-1);var z=math.clamp((p.z/(float)width+.5f)*(n-1),0,n-1);var ix=(int)x;var iz=(int)z;var jx=Math.Min(n-1,ix+1);var jz=Math.Min(n-1,iz+1);return math.lerp(math.lerp(raster[iz*n+ix],raster[iz*n+jx],x-ix),math.lerp(raster[jz*n+ix],raster[jz*n+jx],x-ix),z-iz)/65535f*scale;}
        var positions=new Dictionary<long,float3>();var joins=new Dictionary<long,List<PreparedSegment>>();
        foreach(var s in rails)foreach(var pair in new[]{(s.StartNodeId,s.Start),(s.EndNodeId,s.End)})
        {positions[pair.Item1]=pair.Item2;if(!joins.TryGetValue(pair.Item1,out var edges))joins[pair.Item1]=edges=new List<PreparedSegment>();edges.Add(s);}
        var heights=positions.ToDictionary(p=>p.Key,p=>Ground(p.Value));
        foreach(var node in joins)
        {
            // Bridge abutments may be ground-supported. A blanket +8 m at
            // every bridge endpoint produces unnecessary pillars and steps;
            // only observed independent crossings create clearance floors.
            if(node.Value.All(s=>s.Tunnel))heights[node.Key]-=15;
        }
        // Share heights through portals and branches; do not stack independently
        // sampled pieces. Propagate necessary bridge approaches along actual rails.
        var bridgeFloors=positions.ToDictionary(p=>p.Key,p=>Ground(p.Value));
        // Road/rail crossings share the clear-space minimum; bounded correction
        // never stacks a fresh offset on the previous solver pass.
        var roadGrid=new Dictionary<(int,int),HashSet<PreparedSegment>>();
        foreach(var road in source.Where(e=>!e.Tunnel))
        {
            var steps=Math.Max(1,(int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(road)/32));
            for(var i=0;i<=steps;i++){var p=NetworkNormalizer.Evaluate(road,i/(float)steps);var x=(int)Math.Floor(p.x/64);var z=(int)Math.Floor(p.z/64);
                for(var dx=-1;dx<=1;dx++)for(var dz=-1;dz<=1;dz++){var key=(x+dx,z+dz);if(!roadGrid.TryGetValue(key,out var set))roadGrid[key]=set=new HashSet<PreparedSegment>();set.Add(road);}}
        }
        foreach(var rail in rails.Where(e=>e.Bridge&&!e.Tunnel))
        {
            token.ThrowIfCancellationRequested();var steps=Math.Max(1,(int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(rail)/16));var floor=float.NegativeInfinity;
            for(var i=0;i<=steps;i++)
            {
                var p=NetworkNormalizer.Evaluate(rail,i/(float)steps);
                if(!roadGrid.TryGetValue(((int)Math.Floor(p.x/64),(int)Math.Floor(p.z/64)),out var nearby))continue;
                foreach(var road in nearby)
                {
                    if(road.Kind==NetworkKind.Rail&&(road.StartNodeId==rail.StartNodeId||road.StartNodeId==rail.EndNodeId||road.EndNodeId==rail.StartNodeId||road.EndNodeId==rail.EndNodeId))continue;
                    if(road.Bridge&&road.Layer>=rail.Layer)continue;
                    for(var j=0;j<16;j++)
                    {
                        var a=NetworkNormalizer.Evaluate(road,j/16f);var b=NetworkNormalizer.Evaluate(road,(j+1)/16f);var delta=b.xz-a.xz;
                        var t=math.clamp(math.dot(p.xz-a.xz,delta)/Math.Max(.001f,math.lengthsq(delta)),0,1);
                        if(math.distance(p.xz,math.lerp(a.xz,b.xz,t))>2)continue;
                        if(NetworkContactPolicy.Classify(rail,road,p,math.lerp(a,b,t),(rail.End-rail.Start).xz,delta,false)!=NetworkContactKind.IndependentLevels)continue;
                        floor=Math.Max(floor,math.lerp(a.y,b.y,t)+NetworkVerticalProfiles.MinimumBridgeClearance+2);
                    }
                }
            }
            if(float.IsNegativeInfinity(floor))continue;
            foreach(var id in new[]{rail.StartNodeId,rail.EndNodeId})
            {var bounded=Math.Min(Ground(positions[id])+30,floor);bridgeFloors[id]=Math.Max(bridgeFloors[id],bounded);heights[id]=Math.Max(heights[id],bounded);}
            if(new[]{rail.StartNodeId,rail.EndNodeId}.Any(id=>floor>Ground(positions[id])+30))log.Warn($"Rail way {rail.WayId}: network clearance exceeds the bounded lift budget; retaining the corridor and reporting unresolved clearance rather than raising it indefinitely.");
            log.Diagnostic($"RAIL NETWORK CLEARANCE way={rail.WayId}; targetClearSpaceMeters=5; deckAllowanceMeters=2; requestedDeckHeight={floor}; maximumTerrainLiftMeters=30; renderedClearanceUnverified=true");
        }
        float Clamp(long node,float value)
        {
            var ground=Ground(positions[node]);
            var ceiling=joins[node].All(e=>e.Tunnel)?ground-15:ground+30;
            var floor=joins[node].Any(e=>e.Bridge)?bridgeFloors[node]:ground-20;
            return math.clamp(value,floor,ceiling);
        }
        for(var pass=0;pass<128;pass++)
        {
            token.ThrowIfCancellationRequested();var changed=false;
            foreach(var s in rails)
            {var allowed=lengths[s]*NetworkVerticalProfiles.DesiredGrade(s);var a=heights[s.StartNodeId];var b=heights[s.EndNodeId];
             var excess=Math.Abs(a-b)-allowed;
             if(excess>.001f){var sign=a>b?1f:-1f;var nextA=Clamp(s.StartNodeId,a-sign*excess*.5f);var nextB=Clamp(s.EndNodeId,b+sign*excess*.5f);
                 heights[s.StartNodeId]=nextA;heights[s.EndNodeId]=nextB;changed|=Math.Abs(nextA-a)+Math.Abs(nextB-b)>.001f;}}
            if(!changed)break;
        }
        // Smooth connected station heights without lifting tunnels through their cover.
        for(var pass=0;pass<24;pass++)
        {
            token.ThrowIfCancellationRequested();var snapshot=new Dictionary<long,float>(heights);
            foreach(var node in joins.Where(p=>p.Value.Count==2))
            {
                var a=node.Value[0];var b=node.Value[1];
                var left=a.StartNodeId==node.Key?a.EndNodeId:a.StartNodeId;var right=b.StartNodeId==node.Key?b.EndNodeId:b.StartNodeId;
                if(a.Tunnel!=b.Tunnel)continue;
                var la=Math.Max(1,lengths[a]);var lb=Math.Max(1,lengths[b]);
                var target=(snapshot[left]*lb+snapshot[right]*la)/(la+lb);
                var value=Clamp(node.Key,math.lerp(snapshot[node.Key],target,.35f));
                if(Math.Abs(value-snapshot[left])<=la*.02f&&Math.Abs(value-snapshot[right])<=lb*.02f)heights[node.Key]=value;
            }
        }
        float Slope(long node,PreparedSegment edge)
        {
            var other=edge.StartNodeId==node?edge.EndNodeId:edge.StartNodeId;
            var own=(heights[other]-heights[node])/Math.Max(1,lengths[edge]);
            if(joins[node].Count!=2)return own;
            var adjacent=joins[node].First(e=>!ReferenceEquals(e,edge));
            var far=adjacent.StartNodeId==node?adjacent.EndNodeId:adjacent.StartNodeId;
            var back=(heights[node]-heights[far])/Math.Max(1,lengths[adjacent]);
            return own*back<=0?0:math.clamp((own+back)*.5f,-.02f,.02f);
        }
        var result=new List<PreparedSegment>();
        foreach(var s in source)
        {
            token.ThrowIfCancellationRequested();if(s.Kind!=NetworkKind.Rail){result.Add(s);continue;}
            var a=s.Start;var d=s.End;a.y=heights[s.StartNodeId];d.y=heights[s.EndNodeId];
            var c1=s.Control1??math.lerp(a,d,1f/3);var c2=s.Control2??math.lerp(a,d,2f/3);var delta=d.xz-a.xz;var square=Math.Max(.001f,math.lengthsq(delta));
            c1.y=a.y+Slope(s.StartNodeId,s)*math.distance(c1.xz,a.xz);c2.y=d.y+Slope(s.EndNodeId,s)*math.distance(c2.xz,d.xz);
            var tunnelDip=0f;
            if(s.Tunnel)
            {
                // Tunnel approaches enter at the shared portal, then stay below
                // sampled terrain. Raising a tunnel to a bridge is never a fallback.
                var ceiling=float.PositiveInfinity;
                var length=NetworkVerticalProfiles.ArcLength(s);
                var baseCurve=s with{Start=a,End=d,Control1=c1,Control2=c2};
                float PortalCover(float distance){var t=math.clamp(distance/160,0,1);return 15*t*t*(3-2*t);}
                for(var i=1;i<64;i++){var t=i/64f;var p=NetworkNormalizer.Evaluate(baseCurve,t);var line=p.y;
                    var clearance=15f;
                    if(joins[s.StartNodeId].Any(e=>!e.Tunnel))clearance=Math.Min(clearance,PortalCover(length*t));
                    if(joins[s.EndNodeId].Any(e=>!e.Tunnel))clearance=Math.Min(clearance,PortalCover(length*(1-t)));
                    ceiling=Math.Min(ceiling,(Ground(p)-clearance-line)/(16*t*t*(1-t)*(1-t)));}
                tunnelDip=Math.Min(0,ceiling);
            }
            var revised=s with{Start=a,End=d,Control1=c1,Control2=c2,OneWay=false,StartElevation=a.y-Ground(a),EndElevation=d.y-Ground(d)};
            if(!NetworkVerticalProfiles.WithinGradeCeiling(revised,s.Tunnel?.12f:.06f))
            {log.Count("Rail profiles retained with relaxed limits");log.Warn($"Rail way {s.WayId}: preferred {(s.Tunnel?"tunnel":"corridor")} grade infeasible; retaining connected profile rather than deleting it or blocking preparation. Grade quality unverified.");}
            if(s.Tunnel)
            {
                // Native composition reads endpoint elevations. A portal-to-portal
                // curve with both elevations zero can otherwise render above ground
                // despite its dipped middle. Give interior spans negative elevations.
                var parts=Math.Min(64,Math.Max(lengths[s]>=40?2:1,(int)Math.Ceiling(lengths[s]/80)));
                if(parts>1&&(parts&1)!=0)parts++; // Include the deepest middle station without tiny quarter-spans.
                var ids=new long[parts+1];ids[0]=s.StartNodeId;ids[parts]=s.EndNodeId;
                for(var i=1;i<parts;i++){unchecked{ids[i]=-1-(long)(((ulong)s.WayId*397UL^(ulong)s.StartNodeId*104729UL^(ulong)s.EndNodeId*1000003UL^math.hash(NetworkNormalizer.Evaluate(revised,i/(float)parts).xz))%(ulong)long.MaxValue);}}
                float3 Point(float t){var p=NetworkNormalizer.Evaluate(revised,t);p.y+=tunnelDip*16*t*t*(1-t)*(1-t);return p;}
                float3 Derivative(float t){var u=1-t;var derivative=3*u*u*(c1-a)+6*u*t*(c2-c1)+3*t*t*(d-c2);derivative.y+=tunnelDip*32*t*u*(1-2*t);return derivative;}
                var steep=false;
                for(var i=0;i<parts;i++)
                {
                    var first=i/(float)parts;var last=(i+1)/(float)parts;var start=Point(first);var end=Point(last);
                    var piece=revised with{Start=start,End=end,Control1=start+Derivative(first)*(last-first)/3,Control2=end-Derivative(last)*(last-first)/3,
                        StartNodeId=ids[i],EndNodeId=ids[i+1],StartElevation=start.y-Ground(start),EndElevation=end.y-Ground(end),HorizontalParent=s,ParentStartT=first,ParentEndT=last};
                    steep|=!NetworkVerticalProfiles.WithinGradeCeiling(piece,.12f);result.Add(piece);
                }
                if(steep)log.Warn($"Rail tunnel way {s.WayId}: fitted portal/cover profile exceeds the preferred grade; retained continuous underground geometry, grade quality requires game review.");
            }
            else result.Add(revised);
            log.Count(s.Tunnel?"Rail tunnel profiles fitted":s.Bridge?"Rail bridge profiles fitted":"Rail ground profiles fitted");
        }
        log.Diagnostic($"PROCESS rail-grading complete; sections={rails.Length}; sharedNodes={heights.Count}; deletedSections=0; physicalTrackCounts={string.Join(",",rails.Select(s=>s.PhysicalTracks>0?s.PhysicalTracks:s.Lanes).Distinct().OrderBy(n=>n))}; tunnelDepthCheckedFromTerrain=true; blanketBridgePortalLift=false; desiredGrade=2%; relaxedLimitsReportedSeparately=true");
        return result;
    }
}
