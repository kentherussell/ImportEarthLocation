using System;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

/// <summary>Double geometry and continuous certificates. Quality is not native safety.</summary>
public static class NetworkDesignChecks
{
    public static double3 Evaluate(Cubic3d c,double t)
    {var u=1-t;return u*u*u*c.P0+3*u*u*t*c.P1+3*u*t*t*c.P2+t*t*t*c.P3;}
    public static double3 Derivative(Cubic3d c,double t)
    {var u=1-t;return 3*(u*u*(c.P1-c.P0)+2*u*t*(c.P2-c.P1)+t*t*(c.P3-c.P2));}
    public static double3 SecondDerivative(Cubic3d c,double t)
        => 6*((1-t)*(c.P2-2*c.P1+c.P0)+t*(c.P3-2*c.P2+c.P1));
    public static Cubic3d Hermite(double3 start,double3 end,double3 startDerivative,double3 endDerivative,double referenceSpan)
        => new(start,start+startDerivative*referenceSpan/3,end-endDerivative*referenceSpan/3,end);
    public static (Cubic3d Left,Cubic3d Right) Split(Cubic3d c,double t)
    {
        var a=math.lerp(c.P0,c.P1,t);var b=math.lerp(c.P1,c.P2,t);var d=math.lerp(c.P2,c.P3,t);
        var e=math.lerp(a,b,t);var f=math.lerp(b,d,t);var p=math.lerp(e,f,t);
        return (new(c.P0,a,e,p),new(p,f,d,c.P3));
    }
    public static double Curvature(Cubic3d c,double t)
    {
        var d=Derivative(c,t).xz;var a=SecondDerivative(c,t).xz;var speed=math.length(d);
        return speed>0?Cross(d,a)/(speed*speed*speed):double.NaN;
    }
    public static double Cross(double2 a,double2 b)=>a.x*b.y-a.y*b.x;
    // Exact two-variable box-constrained least squares. Stable candidate order,
    // no iterative fitter or unconstrained handle that reverses progression.
    public static Cubic3d FitHandles(double3 start,double3 end,double2 startTangent,double2 endTangent,
        double3[] observations,double[] stations,double[] support,double epsWorld)
    {
        if(observations.Length!=stations.Length||stations.Length!=support.Length)throw new ArgumentException("Source observation dimensions differ.");
        var chord=math.distance(start.xz,end.xz);var lower=Math.Max(.05*chord,10*epsWorld);var upper=.45*chord;
        if(!(chord>0)||!(lower<=upper)||math.lengthsq(startTangent)==0||math.lengthsq(endTangent)==0)
            throw new ArgumentException("Source cluster has no representable bounded handles.");
        startTangent=math.normalize(startTangent);endTangent=math.normalize(endTangent);
        double a11=0,a12=0,a22=0,c1=0,c2=0;
        for(var i=0;i<observations.Length;i++)
        {
            var t=stations[i];var u=1-t;var b0=u*u*u;var b1=3*u*u*t;var b2=3*u*t*t;var b3=t*t*t;
            var v1=b1*startTangent;var v2=-b2*endTangent;var residual=observations[i].xz-((b0+b1)*start.xz+(b2+b3)*end.xz);
            var w=support[i];if(!SparseProfileRows.Finite(w)||w<0||t<0||t>1)throw new ArgumentException("Invalid physical observation support.");
            a11+=w*math.dot(v1,v1);a12+=w*math.dot(v1,v2);a22+=w*math.dot(v2,v2);
            c1+=w*math.dot(v1,residual);c2+=w*math.dot(v2,residual);
        }
        var bestA=math.clamp(chord/3,lower,upper);var bestB=bestA;
        double Cost(double a,double b)=>a11*a*a+2*a12*a*b+a22*b*b-2*c1*a-2*c2*b;
        var best=Cost(bestA,bestB);
        void Candidate(double a,double b)
        {a=math.clamp(a,lower,upper);b=math.clamp(b,lower,upper);var f=Cost(a,b);if(f<best){best=f;bestA=a;bestB=b;}}
        var determinant=a11*a22-a12*a12;var trace=a11+a22;
        if(determinant>1e-12*trace*trace)
        {var a=(c1*a22-c2*a12)/determinant;var b=(c2*a11-c1*a12)/determinant;if(a>=lower&&a<=upper&&b>=lower&&b<=upper)Candidate(a,b);}
        Candidate(lower,a22>0?(c2-a12*lower)/a22:bestB);
        Candidate(upper,a22>0?(c2-a12*upper)/a22:bestB);
        Candidate(a11>0?(c1-a12*lower)/a11:bestA,lower);
        Candidate(a11>0?(c1-a12*upper)/a11:bestA,upper);
        return new(start,new(start.x+bestA*startTangent.x,math.lerp(start.y,end.y,1/3d),start.z+bestA*startTangent.y),
            new(end.x-bestB*endTangent.x,math.lerp(start.y,end.y,2/3d),end.z-bestB*endTangent.y),end);
    }
    public static bool Finite(Cubic3d c)=>math.all(math.isfinite(c.P0))&&math.all(math.isfinite(c.P1))&&math.all(math.isfinite(c.P2))&&math.all(math.isfinite(c.P3));
    public static CurveCertificate Certify(Cubic3d curve,double maximumCurvature,double maximumGrade,double numericalSpeed=1e-10,CancellationToken token=default)
    {
        if(!Finite(curve))return new(DesignQuality.RequiresReview,double.PositiveInfinity,double.PositiveInfinity,0,true);
        var curvature=0d;var grade=0d;var leaves=0;var certified=true;var visits=0;
        void Visit(Cubic3d c,int depth)
        {
            if((visits++&1023)==0)token.ThrowIfCancellationRequested();
            var d0=3*(c.P1-c.P0);var d1=3*(c.P2-c.P1);var d2=3*(c.P3-c.P2);
            if(math.lengthsq(d0.xz)+math.lengthsq(d1.xz)+math.lengthsq(d2.xz)==0)
            {certified=false;leaves++;curvature=grade=double.PositiveInfinity;return;}
            var chord=c.P3.xz-c.P0.xz;var length=math.length(chord);
            var direction=length>0?chord/length:double2.zero;
            var lower=Math.Min(math.dot(d0.xz,direction),Math.Min(math.dot(d1.xz,direction),math.dot(d2.xz,direction)));
            var boundK=double.PositiveInfinity;var boundG=double.PositiveInfinity;
            if(lower>numericalSpeed)
            {
                var a0=2*(d1.xz-d0.xz);var a1=2*(d2.xz-d1.xz);
                var n0=Cross(d0.xz,a0);var n1=(Cross(d0.xz,a1)+2*Cross(d1.xz,a0))/3;
                var n2=(2*Cross(d1.xz,a1)+Cross(d2.xz,a0))/3;var n3=Cross(d2.xz,a1);
                boundK=Math.Max(Math.Max(Math.Abs(n0),Math.Abs(n1)),Math.Max(Math.Abs(n2),Math.Abs(n3)))/(lower*lower*lower);
                boundG=Math.Max(Math.Abs(d0.y),Math.Max(Math.Abs(d1.y),Math.Abs(d2.y)))/lower;
            }
            if(lower>numericalSpeed&&boundK<=maximumCurvature&&boundG<=maximumGrade)
            {leaves++;curvature=Math.Max(curvature,boundK);grade=Math.Max(grade,boundG);return;}
            if(depth==16)
            {certified=false;leaves++;curvature=Math.Max(curvature,boundK);grade=Math.Max(grade,boundG);return;}
            var split=Split(c,.5);Visit(split.Left,depth+1);Visit(split.Right,depth+1);
        }
        Visit(curve,0);
        return new(certified?DesignQuality.Certified:DesignQuality.RequiresReview,curvature,grade,leaves,false);
    }
    public static (double Value,double First,double Second) QuinticShift(double delta,double station,double length)
    {
        if(!(length>0))throw new ArgumentOutOfRangeException(nameof(length));
        var u=math.clamp(station/length,0d,1d);var u2=u*u;var u3=u2*u;var u4=u3*u;var u5=u4*u;
        return (delta*(1-10*u3+15*u4-6*u5),delta*(-30*u2+60*u3-30*u4)/length,
            delta*(-60*u+180*u2-120*u3)/(length*length));
    }
}
