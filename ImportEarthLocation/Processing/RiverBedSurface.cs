using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Polygon-wide bed shaping. Cuts are blended toward actual banks and islands,
// not toward each source's circular footprint, which produced a beaded trench.
public static class RiverBedSurface
{
    public readonly record struct Profile(float3 A,float3 B,float FloorA,float FloorB);
    public static void Shape(ushort[] original,float scale,double width,IReadOnlyList<GeographicPolygon> rivers,
        IReadOnlyList<Profile> profiles,Dictionary<int,ushort> edits,float waterCell,ImportLog log,CancellationToken token)
    {
        if(rivers.Count==0||profiles.Count==0)return;
        var n=(int)Math.Sqrt(original.Length);
        if(n<2||n*n!=original.Length||scale<=0||width<=0||!math.isfinite(scale)||double.IsNaN(width)||double.IsInfinity(width))throw new ArgumentException("Invalid riverbed terrain.");
        var spacing=(float)width/(n-1);var half=(float)width/2;
        var mask=new bool[original.Length];var holes=new bool[original.Length];
        void Ring(float3[] ring,bool[] target,bool value)
        {
            if(ring.Length<3)return;
            var first=Math.Max(0,(int)Math.Ceiling((ring.Min(p=>p.z)+half)/spacing));var last=Math.Min(n-1,(int)Math.Floor((ring.Max(p=>p.z)+half)/spacing));
            for(var z=first;z<=last;z++)
            {
                token.ThrowIfCancellationRequested();var y=z*spacing-half;var crossings=new List<float>();
                for(var i=0;i<ring.Length;i++){var a=ring[i];var b=ring[(i+1)%ring.Length];if((a.z>y)==(b.z>y))continue;crossings.Add(a.x+(y-a.z)*(b.x-a.x)/(b.z-a.z));}
                crossings.Sort();for(var i=1;i<crossings.Count;i+=2)
                    for(var x=Math.Max(0,(int)Math.Ceiling((crossings[i-1]+half)/spacing));x<=Math.Min(n-1,(int)Math.Floor((crossings[i]+half)/spacing));x++)target[z*n+x]=value;
            }
        }
        foreach(var river in rivers){foreach(var ring in river.Outer)Ring(ring,mask,true);foreach(var ring in river.Inner)Ring(ring,holes,true);}
        for(var i=0;i<mask.Length;i++)mask[i]&=!holes[i];
        // Distance-to-bank field, including inner rings. Two chamfer sweeps are
        // sufficient for smooth material-scale feathering without circular seams.
        var distance=new float[mask.Length];for(var i=0;i<mask.Length;i++)distance[i]=mask[i]?float.PositiveInfinity:0;
        for(var z=0;z<n;z++)for(var x=0;x<n;x++)
        {
            var id=z*n+x;if(!mask[id])continue;
            if(x==0||z==0){distance[id]=0;continue;}
            distance[id]=Math.Min(distance[id],Math.Min(distance[id-1]+spacing,Math.Min(distance[id-n]+spacing,distance[id-n-1]+spacing*1.414214f)));
            if(x<n-1)distance[id]=Math.Min(distance[id],distance[id-n+1]+spacing*1.414214f);
        }
        for(var z=n-1;z>=0;z--)for(var x=n-1;x>=0;x--)
        {
            var id=z*n+x;if(!mask[id])continue;
            if(x==n-1||z==n-1){distance[id]=0;continue;}
            distance[id]=Math.Min(distance[id],Math.Min(distance[id+1]+spacing,Math.Min(distance[id+n]+spacing,distance[id+n+1]+spacing*1.414214f)));
            if(x>0)distance[id]=Math.Min(distance[id],distance[id+n-1]+spacing*1.414214f);
        }
        var grid=new Dictionary<(int,int),List<Profile>>();
        foreach(var profile in profiles)
        {
            var lo=math.min(profile.A.xz,profile.B.xz);var hi=math.max(profile.A.xz,profile.B.xz);
            for(var z=(int)Math.Floor(lo.y/256);z<=(int)Math.Floor(hi.y/256);z++)for(var x=(int)Math.Floor(lo.x/256);x<=(int)Math.Floor(hi.x/256);x++)
            {if(!grid.TryGetValue((x,z),out var list))grid[(x,z)]=list=new();list.Add(profile);}
        }
        var changed=0;var limited=0;
        for(var z=0;z<n;z++)
        {
            token.ThrowIfCancellationRequested();
            for(var x=0;x<n;x++)
            {
                var id=z*n+x;if(!mask[id])continue;var p=new float2(x*spacing-half,z*spacing-half);
                var gx=(int)Math.Floor(p.x/256);var gz=(int)Math.Floor(p.y/256);var best=float.PositiveInfinity;var floor=0f;
                // Search enough neighborhood to cover wide rivers, without
                // borrowing an arbitrary distant river profile across a watershed.
                for(var dz=-4;dz<=4;dz++)for(var dx=-4;dx<=4;dx++)if(grid.TryGetValue((gx+dx,gz+dz),out var list))foreach(var profile in list)
                {
                    var delta=profile.B.xz-profile.A.xz;var t=math.clamp(math.dot(p-profile.A.xz,delta)/Math.Max(.001f,math.lengthsq(delta)),0,1);
                    var d=math.distancesq(p,math.lerp(profile.A.xz,profile.B.xz,t));if(d>=best)continue;best=d;floor=math.lerp(profile.FloorA,profile.FloorB,t);
                }
                if(best>1024*1024)continue;
                var blend=math.clamp(distance[id]/Math.Max(spacing,waterCell*1.5f),0,1);blend=blend*blend*(3-2*blend);
                var height=original[id]/65535f*scale;var cut=Math.Max(0,(height-floor)*blend);if(cut>12)limited++;
                var target=(ushort)Math.Ceiling(Math.Max(0,height-Math.Min(12,cut))/scale*65535);
                // Replace circular legacy cuts with the bank-distance profile.
                // No-data, banks, islands and the caller's DEM remain untouched.
                if(target<original[id]){edits[id]=target;changed++;}else edits.Remove(id);
            }
        }
        for(var i=0;i<mask.Length;i++)if(holes[i])edits.Remove(i);
        log.Diagnostic($"WATER BED SURFACE samples={changed}; cutLimitedSamples={limited}; maximumCutMeters=12; polygonWide=true; bankAndIslandFeathering=true; sourceCirclesDoNotShapeBed=true");
    }
}
