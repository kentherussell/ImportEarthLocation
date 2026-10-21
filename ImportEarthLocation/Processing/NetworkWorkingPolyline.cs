using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Select fitting observations, never source/native nodes. Original stations and
// every original interval remain available to the independent fit certificate.
public static class NetworkWorkingPolyline
{
    public static int[]? Select(IReadOnlyList<float3> points,double[] stations,double tolerance,CancellationToken token)
    {
        if(points.Count<32)return null;
        var keep=new bool[points.Count];keep[0]=keep[points.Count-1]=true;
        var pending=new List<(int Begin,int End)>{(0,points.Count-1)};var visits=0;
        while(pending.Count>0)
        {
            token.ThrowIfCancellationRequested();var (begin,end)=pending[pending.Count-1];pending.RemoveAt(pending.Count-1);var worst=tolerance;var split=-1;
            var span=stations[end]-stations[begin];
            for(var i=begin+1;i<end;i++)
            {
                // Bound worst-case work on adversarial zigzags. Falling back
                // retains the original observations, not a weaker certificate.
                if(++visits>points.Count*16L)return null;
                var t=span>0?(stations[i]-stations[begin])/span:0;
                var error=math.distance((double2)points[i].xz,math.lerp((double2)points[begin].xz,(double2)points[end].xz,t));
                if(error>worst){worst=error;split=i;}
            }
            if(split<0)continue;
            keep[split]=true;pending.Add((split,end));pending.Add((begin,split));
        }
        var selected=Enumerable.Range(0,points.Count).Where(i=>keep[i]).ToArray();
        return selected.Length<points.Count?selected:null;
    }
}
