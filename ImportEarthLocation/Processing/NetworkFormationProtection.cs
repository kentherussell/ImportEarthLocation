using System;
using System.Collections.Generic;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// A directional candidate certificate, not a relaxation of protected geography.
// The broad source envelope includes every possible 24 m move; the emitted
// candidate need only prove its actual native-width footprint stays clear.
internal sealed class NetworkFormationProtection
{
    private readonly record struct Area(float3[] Ring,double2 Min,double2 Max);
    private readonly List<Area> _areas=new();
    private readonly List<(double2 A,double2 B)> _lines=new();
    private readonly Dictionary<(int,int),List<int>> _grid=new();
    private readonly int[] _visited;
    private readonly double _halfWidth;
    private readonly CancellationToken _token;
    private int _query;
    public NetworkFormationProtection(double width,IReadOnlyList<float3[]> areas,IReadOnlyList<float3[]> lines,CancellationToken token)
    {
        _halfWidth=width/2;_token=token;
        void Line(float3[] points,bool closed)
        {
            for(var j=0;j<(closed?points.Length:points.Length-1);j++)
            {
                token.ThrowIfCancellationRequested();var a=(double2)points[j].xz;var b=(double2)points[(j+1)%points.Length].xz;
                var id=_lines.Count;_lines.Add((a,b));var low=math.min(a,b);var high=math.max(a,b);
                for(var z=(int)Math.Floor(low.y/128);z<=(int)Math.Floor(high.y/128);z++)
                    for(var x=(int)Math.Floor(low.x/128);x<=(int)Math.Floor(high.x/128);x++)
                    {if(!_grid.TryGetValue((x,z),out var list))_grid[(x,z)]=list=new();list.Add(id);}
            }
        }
        foreach(var line in lines)Line(line,false);
        foreach(var ring in areas)
        {
            if(ring.Length<3){Line(ring,false);continue;}var low=(double2)ring[0].xz;var high=low;
            foreach(var p in ring){low=math.min(low,p.xz);high=math.max(high,p.xz);}
            _areas.Add(new(ring,low,high));Line(ring,true);
        }
        _visited=new int[_lines.Count];
    }
    public bool Clear(Cubic3d curve,double nativeHalfWidth)
    {
        _token.ThrowIfCancellationRequested();if(!NetworkDesignChecks.Finite(curve))return false;
        var low=math.min(math.min(curve.P0.xz,curve.P1.xz),math.min(curve.P2.xz,curve.P3.xz));
        var high=math.max(math.max(curve.P0.xz,curve.P1.xz),math.max(curve.P2.xz,curve.P3.xz));
        if(math.cmax(math.abs(low))>_halfWidth-nativeHalfWidth||math.cmax(math.abs(high))>_halfWidth-nativeHalfWidth)return false;
        foreach(var area in _areas)
        {
            var p=curve.P0.xz;if(math.any(p<area.Min)||math.any(p>area.Max))continue;var inside=false;
            for(var i=0;i<area.Ring.Length;i++)
            {var a=area.Ring[i];var b=area.Ring[(i+1)%area.Ring.Length];if((a.z>p.y)!=(b.z>p.y)&&p.x<a.x+(p.y-a.z)*(b.x-a.x)/(b.z-a.z))inside=!inside;}
            if(inside)return false;
        }
        var margin=nativeHalfWidth+12;low-=margin;high+=margin;
        if(_query==int.MaxValue){Array.Clear(_visited,0,_visited.Length);_query=0;}_query++;
        for(var z=(int)Math.Floor(low.y/128);z<=(int)Math.Floor(high.y/128);z++)
            for(var x=(int)Math.Floor(low.x/128);x<=(int)Math.Floor(high.x/128);x++)
                if(_grid.TryGetValue((x,z),out var candidates))foreach(var id in candidates)
                {if(_visited[id]==_query)continue;_visited[id]=_query;if(Near(curve,_lines[id],margin,_token))return false;}
        return true;
    }
    internal static bool Near(Cubic3d c,(double2 A,double2 B) line,double margin,CancellationToken token,int depth=0)
    {
        token.ThrowIfCancellationRequested();var center=(c.P0.xz+c.P1.xz+c.P2.xz+c.P3.xz)/4;
        var radius=Math.Max(Math.Max(math.distance(center,c.P0.xz),math.distance(center,c.P1.xz)),
            Math.Max(math.distance(center,c.P2.xz),math.distance(center,c.P3.xz)));
        var chord=line.B-line.A;var squared=math.lengthsq(chord);
        var distance=math.distance(center,line.A+chord*(squared>0?math.clamp(math.dot(center-line.A,chord)/squared,0,1):0));
        if(distance-radius>margin)return false;
        if(distance+radius<=margin||depth==16)return true;
        var split=NetworkDesignChecks.Split(c,.5);return Near(split.Left,line,margin,token,depth+1)||Near(split.Right,line,margin,token,depth+1);
    }
}
