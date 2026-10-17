using System;
using System.Collections.Generic;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Clamped C2 spline in a fixed physical reference station. Y may carry a
// scalar offset; it is not necessarily a world elevation.
public sealed class NetworkReferenceSpline
{
    private readonly double[] _station;
    private readonly double3[] _point,_second;
    public int Pieces=>_station.Length-1;
    public double Station(int i)=>_station[i];
    public NetworkReferenceSpline(IReadOnlyList<double> station,IReadOnlyList<double3> points,double3 first,double3 last)
    {
        if(station.Count<2||station.Count!=points.Count)throw new ArgumentException("Spline needs distinct ordered stations.");
        _station=new double[station.Count];_point=new double3[station.Count];_second=new double3[station.Count];
        var diagonal=new double[station.Count];var upper=new double[station.Count];var rhs=new double3[station.Count];
        for(var i=0;i<station.Count;i++){_station[i]=station[i];_point[i]=points[i];if(!math.all(math.isfinite(points[i]))||!SparseProfileRows.Finite(station[i])||i>0&&station[i]<=station[i-1])throw new ArgumentException("Invalid spline datum.");}
        var h=_station[1]-_station[0];diagonal[0]=2*h;upper[0]=h;rhs[0]=6*((_point[1]-_point[0])/h-first);
        for(var i=1;i<station.Count;i++)
        {
            var left=_station[i]-_station[i-1];double lower;
            if(i==station.Count-1){lower=left;diagonal[i]=2*left;rhs[i]=6*(last-(_point[i]-_point[i-1])/left);}
            else{var right=_station[i+1]-_station[i];lower=left;diagonal[i]=2*(left+right);upper[i]=right;rhs[i]=6*((_point[i+1]-_point[i])/right-(_point[i]-_point[i-1])/left);}
            var factor=lower/diagonal[i-1];diagonal[i]-=factor*upper[i-1];rhs[i]-=factor*rhs[i-1];
        }
        _second[^1]=rhs[^1]/diagonal[^1];
        for(var i=station.Count-2;i>=0;i--)_second[i]=(rhs[i]-upper[i]*_second[i+1])/diagonal[i];
    }
    public Cubic3d Curve(int piece)
    {
        var h=_station[piece+1]-_station[piece];var a=_point[piece];var b=_point[piece+1];
        var left=(b-a)/h-h*(2*_second[piece]+_second[piece+1])/6;
        var right=(b-a)/h+h*(_second[piece]+2*_second[piece+1])/6;
        return NetworkDesignChecks.Hermite(a,b,left,right,h);
    }
    public (double3 Point,double3 Derivative) Evaluate(double station)
    {
        var piece=Array.BinarySearch(_station,station);if(piece<0)piece=~piece-1;piece=Math.Max(0,Math.Min(Pieces-1,piece));
        var h=_station[piece+1]-_station[piece];var t=math.clamp((station-_station[piece])/h,0,1);var c=Curve(piece);
        return(NetworkDesignChecks.Evaluate(c,t),NetworkDesignChecks.Derivative(c,t)/h);
    }
}
