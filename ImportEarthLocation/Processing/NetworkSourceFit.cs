using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// The same bounded two-handle fit is used for ordinary source spans and rebuilt
// approaches. Source correspondence is physical XZ arc station, never terrain Y.
public static class NetworkSourceFit
{
    public static bool TryFit(IReadOnlyList<float3> points,double3 start,double3 end,double2 startTangent,double2 endTangent,
        double tolerance,CancellationToken token,out Cubic3d curve,out int split)
    {
        curve=default;split=Math.Max(1,(points.Count-1)/2);
        if(points.Count<2||!(tolerance>0))return false;
        var stations=new double[points.Count];var support=new double[points.Count];var observations=new double3[points.Count];
        for(var i=0;i<points.Count;i++)
        {
            token.ThrowIfCancellationRequested();observations[i]=points[i];
            if(i>0)stations[i]=stations[i-1]+math.distance(points[i-1].xz,points[i].xz);
        }
        var length=stations[stations.Length-1];if(!(length>0))return false;
        for(var i=0;i<points.Count;i++)
        {
            support[i]=((i>0?stations[i]-stations[i-1]:0)+(i+1<points.Count?stations[i+1]-stations[i]:0))*.5;
        }
        for(var i=0;i<points.Count;i++)stations[i]/=length;
        try{curve=NetworkDesignChecks.FitHandles(start,end,startTangent,endTangent,observations,stations,support,1e-5);}
        catch(ArgumentException){return false;}
        var axis=math.normalizesafe(end.xz-start.xz);
        if(math.lengthsq(axis)==0||Math.Min(math.dot((curve.P1-curve.P0).xz,axis),
            Math.Min(math.dot((curve.P2-curve.P1).xz,axis),math.dot((curve.P3-curve.P2).xz,axis)))<=1e-10)return false;
        var fitted=curve;var worst=-1d;
        for(var i=1;i<points.Count-1;i++)
        {
            var error=math.distance(NetworkDesignChecks.Evaluate(fitted,stations[i]).xz,observations[i].xz);
            if(error>worst){worst=error;split=i;}
        }
        bool Check(double a,double b,double2 p,double2 q,int depth)
        {
            token.ThrowIfCancellationRequested();
            var left=NetworkDesignChecks.Evaluate(fitted,a).xz;var right=NetworkDesignChecks.Evaluate(fitted,b).xz;
            var error=Math.Max(math.distance(left,p),math.distance(right,q));
            if(error>tolerance)return false;
            var acceleration=Math.Max(math.length(NetworkDesignChecks.SecondDerivative(fitted,a).xz),
                math.length(NetworkDesignChecks.SecondDerivative(fitted,b).xz));
            if(error+acceleration*(b-a)*(b-a)/8<=tolerance)return true;
            if(depth==20)return false;
            var middle=(a+b)*.5;var point=(p+q)*.5;
            return Check(a,middle,p,point,depth+1)&&Check(middle,b,point,q,depth+1);
        }
        // Float conversion is part of emission. Reserve its convex-hull error
        // instead of certifying only a double curve that the engine never sees.
        var rounding=new[]{fitted.P0,fitted.P1,fitted.P2,fitted.P3}.Max(p=>math.distance(p.xz,((double3)(float3)p).xz));
        tolerance-=rounding;if(!(tolerance>0))return false;
        for(var i=1;i<points.Count;i++)if(stations[i]>stations[i-1]&&
            !Check(stations[i-1],stations[i],observations[i-1].xz,observations[i].xz,0))return false;
        return true;
    }
}
