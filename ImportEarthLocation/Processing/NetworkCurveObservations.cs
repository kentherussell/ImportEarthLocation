using System;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Bounded observations for comparing planned and native curves. These samples
// expose gross defects; continuous certificates remain a separate quality state.
public static class NetworkCurveObservations
{
    public readonly record struct Observation(bool Finite,double SampledLengthMeters,double SampledMaximumGrade,
        double SampledMaximumCurvature,double MinimumControlProgressMeters,int Samples,bool ContinuousCertificate)
    {
        public double SampledSignedGradeVariation { get; init; }
        public double SampledMaximumGradeChangePerMeter { get; init; }
        public int SampledGradeSignChanges { get; init; }
        public int StationaryGradeSamples { get; init; }
    }

    public static Observation Measure(Cubic3d curve)
    {
        if(!NetworkDesignChecks.Finite(curve))return new(false,0,double.PositiveInfinity,double.PositiveInfinity,0,0,false);
        var axis=math.normalizesafe(curve.P3.xz-curve.P0.xz);
        var progress=Math.Min(math.dot((curve.P1-curve.P0).xz,axis),
            Math.Min(math.dot((curve.P2-curve.P1).xz,axis),math.dot((curve.P3-curve.P2).xz,axis)));
        var length=0d;var grade=0d;var curvature=0d;var previous=curve.P0;
        var variation=0d;var gradeChange=0d;double? previousGrade=null;var sign=0;var signChanges=0;var stationary=0;
        for(var i=0;i<=64;i++)
        {
            var t=i/64d;var p=NetworkDesignChecks.Evaluate(curve,t);
            var step=i>0?math.distance(previous.xz,p.xz):0;
            length+=step;previous=p;
            var derivative=NetworkDesignChecks.Derivative(curve,t);var speed=math.length(derivative.xz);
            grade=Math.Max(grade,speed>1e-10?Math.Abs(derivative.y)/speed:Math.Abs(derivative.y)<=1e-10?0:double.PositiveInfinity);
            if(speed>1e-10)
            {
                var signed=derivative.y/speed;
                if(previousGrade.HasValue)
                {var change=Math.Abs(signed-previousGrade.Value);variation+=change;if(step>1e-10)gradeChange=Math.Max(gradeChange,change/step);}
                previousGrade=signed;
                // Report reversals above the existing continuation-grade noise
                // tolerance. A reversal alone is not a failed physical policy.
                var nextSign=Math.Abs(signed)>1e-5?Math.Sign(signed):0;
                if(nextSign!=0){if(sign!=0&&sign!=nextSign)signChanges++;sign=nextSign;}
            }
            else{stationary++;previousGrade=null;sign=0;}
            var k=speed>0?NetworkDesignChecks.Cross(derivative.xz,NetworkDesignChecks.SecondDerivative(curve,t).xz)/(speed*speed*speed):double.NaN;
            curvature=double.IsNaN(k)?double.PositiveInfinity:Math.Max(curvature,Math.Abs(k));
        }
        return new(true,length,grade,curvature,progress,65,false){SampledSignedGradeVariation=variation,
            SampledMaximumGradeChangePerMeter=gradeChange,SampledGradeSignChanges=signChanges,StationaryGradeSamples=stationary};
    }
}
