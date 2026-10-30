using System;
using System.Collections.Generic;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Feather the cut-depth field, rather than the DEM itself. Only owned cut
// rims may rise; channels, source hollows, levees and original cliffs stay put.
public static class WaterCutSmoothing
{
    public static int Apply(ushort[] original,ushort[] shaped,byte[] owned,byte[] channels,HashSet<int> controls,
        float scale,double width,Func<float3,float?> water,IReadOnlyList<float3[]> cliffs,ImportLog log,CancellationToken token,Action? pulse=null)
    {
        token.ThrowIfCancellationRequested();
        var n=(int)Math.Sqrt(original.Length);
        if(n<3||n*n!=original.Length||shaped.Length!=original.Length||owned.Length!=original.Length||channels.Length!=original.Length)
            throw new ArgumentException("Water-bank smoothing requires matching square rasters.");
        if(!math.isfinite(scale)||scale<=0||double.IsNaN(width)||double.IsInfinity(width)||width<=0)
            throw new ArgumentOutOfRangeException(nameof(scale),"Water-bank smoothing requires positive finite height scale and map width.");
        var spacing=width/(n-1);var metres=scale/65535d;var epsilon=Math.Max(1,(int)Math.Ceiling(.05/metres));
        var protectedCells=new HashSet<int>();
        foreach(var line in cliffs)for(var k=1;k<line.Length;k++)
        {
            token.ThrowIfCancellationRequested();
            var a=line[k-1].xz;var b=line[k].xz;var steps=Math.Max(1,(int)Math.Ceiling(math.cmax(math.abs(b-a))/spacing));
            for(var step=0;step<=steps;step++)
            {
                var p=math.lerp(a,b,step/(float)steps);
                var x=(int)Math.Round((p.x/width+.5)*(n-1));var z=(int)Math.Round((p.y/width+.5)*(n-1));
                for(var dz=-2;dz<=2;dz++)for(var dx=-2;dx<=2;dx++)
                    if(x+dx>=0&&x+dx<n&&z+dz>=0&&z+dz<n)protectedCells.Add((z+dz)*n+x+dx);
            }
        }
        var changes=new Dictionary<int,ushort>();var updates=0;var largest=0d;
        for(var pass=0;pass<2;pass++)
        {
            changes.Clear();
            for(var z=1;z<n-1;z++)for(var x=1;x<n-1;x++)
            {
                var i=z*n+x;if((i&16383)==0){token.ThrowIfCancellationRequested();pulse?.Invoke();}
                if(owned[i]==0||channels[i]!=0||controls.Contains(i)||original[i]-shaped[i]<=epsilon||protectedCells.Contains(i))continue;
                var steep=false;
                for(var dz=-1;dz<=1;dz++)for(var dx=-1;dx<=1;dx++)
                {
                    if(dx==0&&dz==0)continue;var j=i+dz*n+dx;
                    if(Math.Abs(original[j]-original[i])*metres>.65*spacing*Math.Sqrt(dx*dx+dz*dz))steep=true;
                }
                if(steep)continue;
                var rim=false;
                for(var dz=-2;dz<=2&&!rim;dz++)for(var dx=-2;dx<=2;dx++)
                {
                    if(x+dx<0||x+dx>=n||z+dz<0||z+dz>=n)continue;var j=i+dz*n+dx;
                    if(owned[j]==0||original[j]-shaped[j]<=epsilon){rim=true;break;}
                }
                if(!rim)continue;
                var depth=0d;var weight=0d;
                for(var dz=-1;dz<=1;dz++)for(var dx=-1;dx<=1;dx++)
                {
                    var j=i+dz*n+dx;var w=(dx==0?2:1)*(dz==0?2:1);
                    depth+=w*Math.Max(0,original[j]-shaped[j]);weight+=w;
                }
                var target=original[i]-depth/weight;
                var p=new float3((float)(x*spacing-width/2),0,(float)(z*spacing-width/2));var level=water(p);
                if(level.HasValue&&math.isfinite(level.Value))target=Math.Min(target,Math.Max(shaped[i],(level.Value-.5)/metres));
                var encoded=(ushort)Math.Max(shaped[i],Math.Min(original[i],Math.Floor(target)));
                if(encoded<=shaped[i])continue;
                changes[i]=encoded;largest=Math.Max(largest,(encoded-shaped[i])*metres);
            }
            // Apply together so shore orientation and traversal order do not
            // bias the filter. The work set covers edited rims, not another DEM.
            foreach(var change in changes)shaped[change.Key]=change.Value;
            updates+=changes.Count;
        }
        log.Count("Water cut-bank smoothing updates",updates);
        log.Diagnostic($"WATER CUT BANKS updates={updates}; passes=2; maximumPassLiftMeters={largest:0.###}; originalReliefPreserved=true; untouchedSamplesUnchanged=true; lowersBanks=false; wetCoreAndControlsProtected=true; mappedCliffCells={protectedCells.Count}");
        return updates;
    }
}
