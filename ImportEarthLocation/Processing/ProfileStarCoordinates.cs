using System;
using System.Collections.Generic;
using System.Threading;

namespace ImportEarthLocation.Processing;

// Exact null-space coordinates for free node stars. A control is h + beta*q,
// where h and q are in metres and max |beta| is one in each grade group. This
// separates translation from stiff derivative costs without changing any row
// or the original [0,S] bounds on node/control heights. Fixed native contexts
// deliberately retain the existing projector and coordinates.
internal sealed class ProfileStarCoordinates
{
    internal readonly record struct Binding(int Height,int Relative,double Beta);
    internal sealed class Star
    {
        public int Height;
        public int[] Relative=Array.Empty<int>();
        public double[] Positive=Array.Empty<double>(),Negative=Array.Empty<double>(),Trial=Array.Empty<double>(),Breaks=Array.Empty<double>();
    }
    public SparseProfileRows Rows { get; }
    public double[] Prior { get; }
    public Binding[] Bindings { get; }
    public IProfileProjection Projection { get; }
    private ProfileStarCoordinates(SparseProfileRows rows,double[] prior,Binding[] bindings,IProfileProjection projection)
    {Rows=rows;Prior=prior;Bindings=bindings;Projection=projection;}
    public double[] Expand(double[] values)
    {
        var result=new double[Bindings.Length];
        for(var i=0;i<result.Length;i++){var b=Bindings[i];result[i]=values[b.Height]+(b.Relative<0?0:b.Beta*values[b.Relative]);}
        return result;
    }
    public static ProfileStarCoordinates? Create(SparseProfileRows rows,double[] prior,ProfileHardProjector original,CancellationToken token,Action<string>? diagnostic=null)
    {
        if(original.Stars.Count==0)return null;
        foreach(var star in original.Stars)
        {if(star.FixedHeight.HasValue)return null;foreach(var slope in star.FixedSlopes)if(slope.HasValue)return null;}
        var clock=System.Diagnostics.Stopwatch.StartNew();var nextProgress=5d;
        diagnostic?.Invoke($"PROFILE WORK stage=star-coordinate-transform-start; variables={rows.Variables}; rows={rows.Count}");
        var bindings=new Binding[prior.Length];var assigned=new bool[prior.Length];var seed=new List<double>();var stars=new List<Star>();
        foreach(var source in original.Stars)
        {
            var scales=new double[source.GroupCount];
            for(var j=0;j<source.Controls.Length;j++)scales[source.Groups[j]]=Math.Max(scales[source.Groups[j]],Math.Abs(source.Alpha[j]));
            foreach(var scale in scales)if(!(scale>0)||!SparseProfileRows.Finite(scale))return null;
            var h=seed.Count;seed.Add(prior[source.Height]);bindings[source.Height]=new(h,-1,0);assigned[source.Height]=true;
            var star=new Star{Height=h,Relative=new int[scales.Length],Positive=new double[scales.Length],Negative=new double[scales.Length],
                Trial=new double[scales.Length],Breaks=new double[6*scales.Length+2]};
            var numerator=new double[scales.Length];var denominator=new double[scales.Length];
            for(var g=0;g<scales.Length;g++){star.Relative[g]=seed.Count;seed.Add(0);}
            for(var j=0;j<source.Controls.Length;j++)
            {
                var group=source.Groups[j];var beta=source.Alpha[j]/scales[group];var control=source.Controls[j];
                if(beta==0||!SparseProfileRows.Finite(beta))return null;
                bindings[control]=new(h,star.Relative[group],beta);assigned[control]=true;
                numerator[group]+=beta*(prior[control]-prior[source.Height]);denominator[group]+=beta*beta;
                if(beta>0)star.Positive[group]=Math.Max(star.Positive[group],beta);else star.Negative[group]=Math.Max(star.Negative[group],-beta);
            }
            for(var g=0;g<scales.Length;g++)seed[star.Relative[g]]=numerator[g]/denominator[g];
            stars.Add(star);
        }
        for(var i=0;i<prior.Length;i++)if(!assigned[i]){bindings[i]=new(seed.Count,-1,0);seed.Add(prior[i]);}
        if(seed.Count>=prior.Length)return null;
        var reduced=new SparseProfileRows(seed.Count);var indices=new int[16];var coefficients=new double[16];
        for(long r=0;r<rows.Count;r++)
        {
            if((r&1023)==0)
            {
                token.ThrowIfCancellationRequested();
                if(diagnostic!=null&&clock.Elapsed.TotalSeconds>=nextProgress)
                {nextProgress=clock.Elapsed.TotalSeconds+5;diagnostic($"PROFILE WORK stage=star-coordinate-transform; rowsProcessed={r}; totalRows={rows.Count}; elapsedSeconds={clock.Elapsed.TotalSeconds:R}");}
            }
            var row=rows.Row(r);var count=0;
            void Add(int variable,double value)
            {var slot=0;while(slot<count&&indices[slot]!=variable)slot++;if(slot==count){indices[count]=variable;coefficients[count++]=0;}coefficients[slot]+=value;}
            for(var j=0;j<row.Count;j++)
            {var b=bindings[rows.Index(row.Offset+j)];var a=rows.Coefficient(row.Offset+j);Add(b.Height,a);if(b.Relative>=0)Add(b.Relative,a*b.Beta);}
            var actual=0;for(var j=0;j<count;j++)
            {if(!SparseProfileRows.Finite(coefficients[j]))return null;if(coefficients[j]!=0){indices[actual]=indices[j];coefficients[actual++]=coefficients[j];}}
            if(actual>8)
            {diagnostic?.Invoke("PROFILE COORDINATES unavailable=transformed-row-arity; originalProblemRetained=true");return null;}
            reduced.Append(indices,coefficients,actual,row.Target,row.Weight,row.Kind);
        }
        reduced.Freeze();diagnostic?.Invoke($"PROFILE WORK stage=star-coordinate-transform-complete; variables={seed.Count}; rows={reduced.Count}; terms={reduced.Terms}; elapsedSeconds={clock.Elapsed.TotalSeconds:R}");
        return new(reduced,seed.ToArray(),bindings,new ProjectionSet(seed.Count,original.HeightScale,stars.ToArray()));
    }

    private sealed class ProjectionSet : IProfileProjection
    {
        private readonly Star[] _stars;private readonly bool[] _owned;
        public int Variables { get; }
        public double HeightScale { get; }
        public ProjectionSet(int variables,double scale,Star[] stars)
        {Variables=variables;HeightScale=scale;_stars=stars;_owned=new bool[variables];foreach(var s in stars){_owned[s.Height]=true;foreach(var q in s.Relative)_owned[q]=true;}}
        private static double Clamp(double v,double lo,double hi)=>Math.Max(lo,Math.Min(hi,v));
        private void Bounds(Star s,int g,double h,out double lo,out double hi)
        {
            var p=s.Positive[g];var n=s.Negative[g];
            lo=Math.Max(p>0?-h/p:double.NegativeInfinity,n>0?(h-HeightScale)/n:double.NegativeInfinity);
            hi=Math.Min(p>0?(HeightScale-h)/p:double.PositiveInfinity,n>0?h/n:double.PositiveInfinity);
        }
        private int Breakpoints(Star s,bool projection)
        {
            var count=0;s.Breaks[count++]=0;s.Breaks[count++]=HeightScale;
            void Add(double h){if(h>0&&h<HeightScale)s.Breaks[count++]=h;}
            for(var g=0;g<s.Relative.Length;g++)
            {
                var p=s.Positive[g];var n=s.Negative[g];
                if(p>0&&n>0){Add(HeightScale*p/(p+n));Add(HeightScale*n/(p+n));}
                if(!projection)continue;var q=s.Trial[g];
                if(p>0){Add(-p*q);Add(HeightScale-p*q);}if(n>0){Add(HeightScale+n*q);Add(n*q);}
            }
            Array.Sort(s.Breaks,0,count);return count;
        }
        public void Fallback(double[] prior,double[] target)
        {
            for(var i=0;i<Variables;i++)target[i]=SparseProfileRows.Finite(prior[i])?Clamp(prior[i],0,HeightScale):0;
            foreach(var star in _stars)foreach(var q in star.Relative)target[q]=0;
        }
        public bool Project(double[] values,double[] d)
        {
            if(values.Length!=Variables||d.Length!=Variables)throw new ArgumentException("Projection vector size mismatch.");
            for(var i=0;i<Variables;i++)if(!SparseProfileRows.Finite(values[i])||!SparseProfileRows.Finite(d[i])||d[i]<=0)return false;
            foreach(var s in _stars)
            {
                var trial=values[s.Height];for(var g=0;g<s.Relative.Length;g++)s.Trial[g]=values[s.Relative[g]];
                var count=Breakpoints(s,true);var best=double.PositiveInfinity;var bestH=0d;
                for(var k=1;k<count;k++)
                {
                    var lo=s.Breaks[k-1];var hi=s.Breaks[k];var mid=(lo+hi)/2;var a=d[s.Height];var b=a*trial;
                    for(var g=0;g<s.Relative.Length;g++)
                    {
                        Bounds(s,g,mid,out var lower,out var upper);var q=s.Trial[g];if(q>=lower&&q<=upper)continue;
                        var p=s.Positive[g];var n=s.Negative[g];double slope,intercept;
                        if(q<lower)
                        {if(p>0&&(n==0||-mid/p>=(mid-HeightScale)/n)){slope=-1/p;intercept=0;}else {slope=1/n;intercept=-HeightScale/n;}}
                        else
                        {if(p>0&&(n==0||(HeightScale-mid)/p<=mid/n)){slope=-1/p;intercept=HeightScale/p;}else {slope=1/n;intercept=0;}}
                        var weight=d[s.Relative[g]];a+=weight*slope*slope;b+=weight*slope*(q-intercept);
                    }
                    var h=Clamp(b/a,lo,hi);var cost=d[s.Height]*(h-trial)*(h-trial);
                    for(var g=0;g<s.Relative.Length;g++)
                    {Bounds(s,g,h,out var lower,out var upper);var delta=Clamp(s.Trial[g],lower,upper)-s.Trial[g];cost+=d[s.Relative[g]]*delta*delta;}
                    if(cost<best){best=cost;bestH=h;}
                }
                if(!SparseProfileRows.Finite(best))return false;values[s.Height]=bestH;
                for(var g=0;g<s.Relative.Length;g++){Bounds(s,g,bestH,out var lower,out var upper);values[s.Relative[g]]=Clamp(s.Trial[g],lower,upper);}
            }
            for(var i=0;i<Variables;i++)if(!_owned[i])values[i]=Clamp(values[i],0,HeightScale);
            return true;
        }
        public bool TryLinearMinimum(double[] gradient,out double lowerBound)
        {
            if(gradient.Length!=Variables)throw new ArgumentException("Gradient size mismatch.");
            lowerBound=0;var magnitude=0d;
            for(var i=0;i<Variables;i++)
            {if(!SparseProfileRows.Finite(gradient[i]))return false;if(!_owned[i]){var value=Math.Min(0,gradient[i]*HeightScale);lowerBound+=value;magnitude+=Math.Abs(value);}}
            foreach(var s in _stars)
            {
                var count=Breakpoints(s,false);var best=double.PositiveInfinity;var starMagnitude=0d;
                for(var k=0;k<count;k++)
                {
                    var h=s.Breaks[k];var value=gradient[s.Height]*h;var absolute=Math.Abs(value);
                    for(var g=0;g<s.Relative.Length;g++)
                    {Bounds(s,g,h,out var lo,out var hi);var term=gradient[s.Relative[g]]*(gradient[s.Relative[g]]>=0?lo:hi);value+=term;absolute+=Math.Abs(term);}
                    best=Math.Min(best,value);starMagnitude=Math.Max(starMagnitude,absolute);
                }
                lowerBound+=best;magnitude+=starMagnitude;
            }
            lowerBound-=64*2.2204460492503131e-16*Math.Max(1,Variables)*magnitude;
            return SparseProfileRows.Finite(lowerBound);
        }
    }
}
