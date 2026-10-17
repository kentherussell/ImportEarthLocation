using System;
using System.Collections.Generic;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Optional approximation cuts must survive the final float coordinate grid.
// This never removes a source junction: the enclosing curve's endpoints remain.
public static class NetworkEmissionStations
{
    public static double MinimumReach(Cubic3d c)
    {
        var magnitude=Math.Max(Math.Max(math.cmax(math.abs(c.P0.xz)),math.cmax(math.abs(c.P3.xz))),1);
        var value=(float)magnitude;
        var ulp=(double)(math.asfloat(math.asint(value)+1)-value);
        return Math.Max(.05,8*ulp);
    }

    public static bool CanSplit(Cubic3d c,double t)
    {
        if(!(t>0&&t<1))return false;
        var p=(double3)(float3)NetworkDesignChecks.Evaluate(c,t);
        var minimum=MinimumReach(c);
        return math.distance(p.xz,((double3)(float3)c.P0).xz)>=minimum&&
            math.distance(p.xz,((double3)(float3)c.P3).xz)>=minimum;
    }

    public static double[] Cuts(Cubic3d c,IEnumerable<double> candidates)
    {
        var ordered=new SortedSet<double>(candidates);var result=new List<double>{0};
        var previous=((double3)(float3)c.P0).xz;var end=((double3)(float3)c.P3).xz;var minimum=MinimumReach(c);
        foreach(var t in ordered)
        {
            if(!(t>0&&t<1))continue;
            var p=((double3)(float3)NetworkDesignChecks.Evaluate(c,t)).xz;
            if(math.distance(p,previous)<minimum||math.distance(p,end)<minimum)continue;
            result.Add(t);previous=p;
        }
        result.Add(1);return result.ToArray();
    }
}
