using System;
using System.Collections.Generic;

namespace ImportEarthLocation.Processing;

public sealed class ProfileStar
{
    public int Height { get; }
    internal readonly int[] Controls, Groups;
    internal readonly double[] Alpha;
    internal readonly double[] Trial, GroupA, GroupB, GroupLower, GroupUpper, GroupSlope;
    internal readonly double[] ActiveA, ActiveB, LowerA, LowerB, UpperA, UpperB;
    internal readonly double[] Breakpoints;
    internal readonly double[] LineA, LineB;
    public double? FixedHeight { get; }
    internal readonly double?[] FixedSlopes;
    internal int GroupCount { get; }
    public ProfileStar(int height,int[] controls,double[] alpha,int[] groups,double? fixedHeight=null,double?[]? fixedSlopes=null)
    {
        if(controls.Length!=alpha.Length||controls.Length!=groups.Length)throw new ArgumentException("Star metadata mismatch.");
        Height=height;Controls=(int[])controls.Clone();Alpha=(double[])alpha.Clone();Groups=(int[])groups.Clone();FixedHeight=fixedHeight;
        var groupCount=0;for(var i=0;i<groups.Length;i++){if(groups[i]<0||alpha[i]==0||!SparseProfileRows.Finite(alpha[i]))throw new ArgumentException("Star requires finite nonzero reaches and nonnegative group IDs.");groupCount=Math.Max(groupCount,checked(groups[i]+1));}
        GroupCount=groupCount;FixedSlopes=fixedSlopes==null?new double?[groupCount]:(double?[])fixedSlopes.Clone();
        if(FixedSlopes.Length!=groupCount)throw new ArgumentException("Fixed grade group count mismatch.");
        Trial=new double[controls.Length];GroupA=new double[groupCount];GroupB=new double[groupCount];GroupLower=new double[groupCount];GroupUpper=new double[groupCount];GroupSlope=new double[groupCount];
        ActiveA=new double[groupCount];ActiveB=new double[groupCount];LowerA=new double[groupCount];LowerB=new double[groupCount];UpperA=new double[groupCount];UpperB=new double[groupCount];
        // Exact enumeration only for <=8 ports. High-degree stars use scalar bisection.
        Breakpoints=new double[controls.Length<=8?140:0];LineA=new double[controls.Length<=8?17:0];LineB=new double[LineA.Length];
    }
}

/// <summary>Disjoint exact D-metric node-star projection and fixed height boxes.</summary>
public interface IProfileProjection
{
    int Variables { get; }
    double HeightScale { get; }
    bool Project(double[] values,double[] diagonal);
    bool TryLinearMinimum(double[] gradient,out double lowerBound);
    void Fallback(double[] prior,double[] target);
}

public sealed class ProfileHardProjector : IProfileProjection
{
    private readonly ProfileStar[] _stars;
    private readonly bool[] _owned;
    public int Variables { get; }
    public double HeightScale { get; }
    internal IReadOnlyList<ProfileStar> Stars=>_stars;
    public void Fallback(double[] prior,double[] target)
    {for(var j=0;j<Variables;j++)target[j]=SparseProfileRows.Finite(prior[j])?Clamp(prior[j],0,HeightScale):0;}
    public ProfileHardProjector(int variables,double heightScale,ProfileStar[] stars)
    {
        if(variables<0||!SparseProfileRows.Finite(heightScale)||heightScale<=0)throw new ArgumentException("Invalid hard-set dimensions.");
        Variables=variables;HeightScale=heightScale;_stars=(ProfileStar[])stars.Clone();_owned=new bool[variables];
        void Claim(int v){if(v<0||v>=variables||_owned[v])throw new ArgumentException("Star blocks must have disjoint variable ownership.");_owned[v]=true;}
        foreach(var star in _stars){Claim(star.Height);foreach(var control in star.Controls)Claim(control);}
    }
    public bool Project(double[] values,double[] d)
    {
        if(values.Length!=Variables||d.Length!=Variables)throw new ArgumentException("Projection vector size mismatch.");
        for(var j=0;j<Variables;j++)if(!SparseProfileRows.Finite(values[j])||!SparseProfileRows.Finite(d[j])||d[j]<=0)return false;
        foreach(var star in _stars)if(!ProjectStar(star,values,d))return false;
        for(var j=0;j<Variables;j++)if(!_owned[j])values[j]=Clamp(values[j],0,HeightScale);
        return true;
    }
    /// <summary>Minimum of a linear functional over the SAME bounded star set.
    /// Returns a lower bound, including the unresolved scalar bracket's error.
    /// This is a stopping certificate, not another optimization solver.</summary>
    public bool TryLinearMinimum(double[] gradient,out double lowerBound)
    {
        lowerBound=0;
        if(gradient.Length!=Variables)throw new ArgumentException("Gradient size mismatch.");
        for(var j=0;j<Variables;j++)
        {
            if(!SparseProfileRows.Finite(gradient[j]))return false;
            if(!_owned[j])lowerBound+=Math.Min(0,gradient[j]*HeightScale);
        }
        foreach(var star in _stars)
        {
            var lo=0d;var hi=HeightScale;var heightCost=gradient[star.Height];
            if(star.FixedHeight.HasValue)lo=hi=star.FixedHeight.Value;
            Array.Clear(star.GroupB,0,star.GroupCount);
            for(var j=0;j<star.Controls.Length;j++)
            {
                var a=star.Alpha[j];var g=star.Groups[j];var cost=gradient[star.Controls[j]];
                heightCost+=cost;star.GroupB[g]+=a*cost;
                if(star.FixedSlopes[g].HasValue)
                {var m=star.FixedSlopes[g]!.Value;lo=Math.Max(lo,-a*m);hi=Math.Min(hi,HeightScale-a*m);}
            }
            if(!SparseProfileRows.Finite(lo)||!SparseProfileRows.Finite(hi)||lo<0||hi>HeightScale||lo>hi)return false;
            // All port bounds are h+alpha*m in [0,S]. For each sign of alpha,
            // the largest magnitude determines the tight lower/upper envelope.
            Array.Clear(star.GroupLower,0,star.GroupCount);Array.Clear(star.GroupUpper,0,star.GroupCount);
            for(var j=0;j<star.Controls.Length;j++)
            {var a=star.Alpha[j];var g=star.Groups[j];if(a>0)star.GroupLower[g]=Math.Max(star.GroupLower[g],a);else star.GroupUpper[g]=Math.Max(star.GroupUpper[g],-a);}
            var slopeBound=Math.Abs(heightCost);
            for(var g=0;g<star.GroupCount;g++)
            {
                if(!SparseProfileRows.Finite(star.GroupB[g]))return false;
                if(star.FixedSlopes[g].HasValue)continue;
                var p=star.GroupLower[g];var q=star.GroupUpper[g];
                var reciprocal=Math.Max(p>0?1/p:0,q>0?1/q:0);
                slopeBound+=Math.Abs(star.GroupB[g])*reciprocal;
            }
            double Linear(double h,out double slope)
            {
                var value=heightCost*h;slope=heightCost;
                for(var g=0;g<star.GroupCount;g++)
                {
                    var cost=star.GroupB[g];double m,derivative;
                    if(star.FixedSlopes[g].HasValue){m=star.FixedSlopes[g]!.Value;derivative=0;}
                    else
                    {
                        var positive=star.GroupLower[g];var negative=star.GroupUpper[g];
                        if(cost>=0)
                        {
                            var p=positive>0?-h/positive:double.NegativeInfinity;
                            var q=negative>0?-(HeightScale-h)/negative:double.NegativeInfinity;
                            if(p>=q){m=p;derivative=-1/positive;}else{m=q;derivative=1/negative;}
                        }
                        else
                        {
                            var p=positive>0?(HeightScale-h)/positive:double.PositiveInfinity;
                            var q=negative>0?h/negative:double.PositiveInfinity;
                            if(p<=q){m=p;derivative=-1/positive;}else{m=q;derivative=1/negative;}
                        }
                    }
                    value+=cost*m;slope+=cost*derivative;
                }
                return value;
            }
            var lower=lo;var upper=hi;
            for(var iteration=0;iteration<64&&lower<upper;iteration++)
            {
                var mid=lower+(upper-lower)*.5;if(mid==lower||mid==upper)break;
                Linear(mid,out var slope);if(slope<0)lower=mid;else upper=mid;
            }
            var minimum=Math.Min(Linear(lower,out _),Linear(upper,out _));
            var bound=minimum-slopeBound*(upper-lower);
            if(!SparseProfileRows.Finite(bound))return false;
            lowerBound+=bound;
        }
        return SparseProfileRows.Finite(lowerBound);
    }
    private bool ProjectStar(ProfileStar star,double[] values,double[] d)
    {
        var originalHeight=values[star.Height];var lo=0d;var hi=HeightScale;
        if(star.FixedHeight.HasValue){lo=hi=star.FixedHeight.Value;if(!SparseProfileRows.Finite(lo)||lo<0||hi>HeightScale)return false;}
        Array.Clear(star.GroupA,0,star.GroupCount);Array.Clear(star.GroupB,0,star.GroupCount);Array.Clear(star.GroupSlope,0,star.GroupCount);
        for(var j=0;j<star.Controls.Length;j++)
        {
            var v=star.Controls[j];var a=star.Alpha[j];var group=star.Groups[j];star.Trial[j]=values[v];
            star.GroupA[group]-=d[v]*a;star.GroupB[group]+=d[v]*a*values[v];star.GroupSlope[group]+=d[v]*a*a;
            if(star.FixedSlopes[group].HasValue)
            {var m=star.FixedSlopes[group]!.Value;if(!SparseProfileRows.Finite(m))return false;lo=Math.Max(lo,-a*m);hi=Math.Min(hi,HeightScale-a*m);}
        }
        if(lo>hi)return false;
        for(var g=0;g<star.GroupCount;g++)
        {if(!(star.GroupSlope[g]>0))return false;star.GroupA[g]/=star.GroupSlope[g];star.GroupB[g]/=star.GroupSlope[g];}
        void Slopes(double h)
        {
            for(var g=0;g<star.GroupCount;g++){star.GroupLower[g]=double.NegativeInfinity;star.GroupUpper[g]=double.PositiveInfinity;}
            for(var j=0;j<star.Controls.Length;j++)
            {
                var a=star.Alpha[j];var g=star.Groups[j];var slope=-1/a;
                var lowerB=a>0?0:HeightScale/a;var upperB=a>0?HeightScale/a:0;
                var lower=slope*h+lowerB;var upper=slope*h+upperB;
                if(lower>star.GroupLower[g]){star.GroupLower[g]=lower;star.LowerA[g]=slope;star.LowerB[g]=lowerB;}
                if(upper<star.GroupUpper[g]){star.GroupUpper[g]=upper;star.UpperA[g]=slope;star.UpperB[g]=upperB;}
            }
            for(var g=0;g<star.GroupCount;g++)
            {
                var unbounded=star.GroupA[g]*h+star.GroupB[g];
                if(star.FixedSlopes[g].HasValue){star.ActiveA[g]=0;star.ActiveB[g]=star.FixedSlopes[g]!.Value;}
                else if(unbounded<=star.GroupLower[g]){star.ActiveA[g]=star.LowerA[g];star.ActiveB[g]=star.LowerB[g];}
                else if(unbounded>=star.GroupUpper[g]){star.ActiveA[g]=star.UpperA[g];star.ActiveB[g]=star.UpperB[g];}
                else {star.ActiveA[g]=star.GroupA[g];star.ActiveB[g]=star.GroupB[g];}
                star.GroupSlope[g]=star.ActiveA[g]*h+star.ActiveB[g];
            }
        }
        double Objective(double h)
        {
            Slopes(h);var delta=h-originalHeight;var sum=.5*d[star.Height]*delta*delta;
            for(var j=0;j<star.Controls.Length;j++){delta=h+star.Alpha[j]*star.GroupSlope[star.Groups[j]]-star.Trial[j];sum+=.5*d[star.Controls[j]]*delta*delta;}
            return sum;
        }
        void Quadratic(out double q,out double r)
        {
            q=d[star.Height];r=d[star.Height]*originalHeight;
            for(var j=0;j<star.Controls.Length;j++)
            {
                var a=star.Alpha[j];var group=star.Groups[j];var factor=1+a*star.ActiveA[group];var weight=d[star.Controls[j]];
                q+=weight*factor*factor;r-=weight*factor*(a*star.ActiveB[group]-star.Trial[j]);
            }
        }
        double Derivative(double h)
        {Slopes(h);Quadratic(out var q,out var r);return q*h-r;}
        var bestH=lo;var bestObjective=Objective(lo);
        void Consider(double h)
        {var objective=Objective(h);if(objective<bestObjective||(objective==bestObjective&&h<bestH)){bestObjective=objective;bestH=h;}}
        Consider(hi);
        if(lo<hi&&star.Controls.Length<=8)
        {
            var count=2;star.Breakpoints[0]=lo;star.Breakpoints[1]=hi;
            for(var g=0;g<star.GroupCount;g++)
            {
                if(star.FixedSlopes[g].HasValue)continue;
                var lines=1;star.LineA[0]=star.GroupA[g];star.LineB[0]=star.GroupB[g];
                for(var j=0;j<star.Controls.Length;j++)if(star.Groups[j]==g)
                {star.LineA[lines]=-1/star.Alpha[j];star.LineB[lines++]=0;star.LineA[lines]=-1/star.Alpha[j];star.LineB[lines++]=HeightScale/star.Alpha[j];}
                for(var i=0;i<lines;i++)for(var j=i+1;j<lines;j++)
                {var divisor=star.LineA[i]-star.LineA[j];if(divisor==0)continue;var h=(star.LineB[j]-star.LineB[i])/divisor;if(h>lo&&h<hi)star.Breakpoints[count++]=h;}
            }
            Array.Sort(star.Breakpoints,0,count);
            for(var i=0;i<count-1;i++)
            {
                var left=star.Breakpoints[i];var right=star.Breakpoints[i+1];if(left==right)continue;
                Slopes(left+(right-left)*.5);Quadratic(out var q,out var r);
                var h=Clamp(r/q,left,right);Consider(h);Consider(left);Consider(right);
            }
        }
        else if(lo<hi)
        {
            var lower=lo;var upper=hi;
            for(var iteration=0;iteration<64&&upper-lower>1e-8;iteration++)
            {var mid=lower+(upper-lower)*.5;if(Derivative(mid)<0)lower=mid;else upper=mid;}
            Consider(lower);Consider(upper);Consider(lower+(upper-lower)*.5);
        }
        if(!SparseProfileRows.Finite(bestObjective))return false;
        Slopes(bestH);values[star.Height]=bestH;
        for(var j=0;j<star.Controls.Length;j++)values[star.Controls[j]]=bestH+star.Alpha[j]*star.GroupSlope[star.Groups[j]];
        return true;
    }
    private static double Clamp(double value,double lo,double hi)=>Math.Max(lo,Math.Min(hi,value));
}
