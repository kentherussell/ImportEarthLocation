using System;
using System.Collections.Generic;
using System.Threading;

namespace ImportEarthLocation.Processing;

public enum ProfileRowKind : byte { EqualityCost, UpperHingeCost }
public readonly record struct ProfileRow(long Offset, byte Count, double Target, double Weight, ProfileRowKind Kind);

/// <summary>Frozen primitive sparse rows. No dense matrix or per-row term object.</summary>
public sealed class SparseProfileRows
{
    public const int ChunkSize=4096;
    private readonly List<ProfileRow[]> _rows=new();
    private readonly List<int[]> _indices=new();
    private readonly List<double[]> _coefficients=new();
    private readonly int[] _indexScratch=new int[8];
    private readonly double[] _coefficientScratch=new double[8];
    private bool _sealed;
    public long Count { get; private set; }
    public long Terms { get; private set; }
    public int Variables { get; }
    public long CapacityBytes => checked((long)_rows.Count*ChunkSize*40+(long)_indices.Count*ChunkSize*12);
    public SparseProfileRows(int variables) { if(variables<0)throw new ArgumentOutOfRangeException(nameof(variables));Variables=variables; }
    public void Append(int[] indices,double[] coefficients,int count,double target,double weight,ProfileRowKind kind)
    {
        if(_sealed)throw new InvalidOperationException("Profile rows are frozen.");
        if(count<0||count>8||indices.Length<count||coefficients.Length<count||!Finite(target)||!Finite(weight)||weight<=0)
            throw new ArgumentException("Invalid sparse profile row.");
        var used=0;
        for(var i=0;i<count;i++)
        {
            if(indices[i]<0||indices[i]>=Variables||!Finite(coefficients[i]))throw new ArgumentException("Invalid profile coefficient.");
            var slot=0;while(slot<used&&_indexScratch[slot]!=indices[i])slot++;
            if(slot==used){_indexScratch[used]=indices[i];_coefficientScratch[used++]=coefficients[i];}
            else _coefficientScratch[slot]+=coefficients[i];
        }
        var actual=0;
        for(var i=0;i<used;i++)if(_coefficientScratch[i]!=0)
        { _indexScratch[actual]=_indexScratch[i];_coefficientScratch[actual++]=_coefficientScratch[i]; }
        // At most eight elements: stable index insertion sort, not a global new solver/order policy.
        for(var i=1;i<actual;i++)
        {var index=_indexScratch[i];var coefficient=_coefficientScratch[i];var j=i;while(j>0&&_indexScratch[j-1]>index){_indexScratch[j]=_indexScratch[j-1];_coefficientScratch[j]=_coefficientScratch[j-1];j--;}_indexScratch[j]=index;_coefficientScratch[j]=coefficient;}
        var offset=Terms;
        for(var i=0;i<actual;i++)
        {
            var chunk=checked((int)(Terms/ChunkSize));var slot=(int)(Terms%ChunkSize);
            if(chunk==_indices.Count){_indices.Add(new int[ChunkSize]);_coefficients.Add(new double[ChunkSize]);}
            _indices[chunk][slot]=_indexScratch[i];_coefficients[chunk][slot]=_coefficientScratch[i];Terms=checked(Terms+1);
        }
        var rowChunk=checked((int)(Count/ChunkSize));var rowSlot=(int)(Count%ChunkSize);
        if(rowChunk==_rows.Count)_rows.Add(new ProfileRow[ChunkSize]);
        _rows[rowChunk][rowSlot]=new ProfileRow(offset,(byte)actual,target,weight,kind);Count=checked(Count+1);
    }
    public void Freeze() => _sealed=true;
    public ProfileRow Row(long index)=>_rows[checked((int)(index/ChunkSize))][(int)(index%ChunkSize)];
    public int Index(long index)=>_indices[checked((int)(index/ChunkSize))][(int)(index%ChunkSize)];
    public double Coefficient(long index)=>_coefficients[checked((int)(index/ChunkSize))][(int)(index%ChunkSize)];
    public double Evaluate(double[] x,double[]? gradient,CancellationToken token,bool objective=true)
    {
        if(!_sealed||x.Length!=Variables||(gradient!=null&&gradient.Length!=Variables))throw new InvalidOperationException("Profile evaluation requires frozen rows and matching vectors.");
        if(gradient!=null)Array.Clear(gradient,0,gradient.Length);
        var sum=0d;
        for(long r=0;r<Count;r++)
        {
            if((r&1023)==0)token.ThrowIfCancellationRequested();
            var row=Row(r);var residual=-row.Target;
            for(var j=0;j<row.Count;j++)residual+=Coefficient(row.Offset+j)*x[Index(row.Offset+j)];
            if(row.Kind==ProfileRowKind.UpperHingeCost)residual=Math.Max(0,residual);
            if(objective)sum+=.5*row.Weight*residual*residual;
            if(gradient!=null)for(var j=0;j<row.Count;j++)gradient[Index(row.Offset+j)]+=row.Weight*residual*Coefficient(row.Offset+j);
        }
        return sum;
    }
    public (double[] D,double L) Conditioning(CancellationToken token)
    {
        var diagonal=new double[Variables];var inverseRoot=new double[Variables];var sums=new double[Variables];
        for(long r=0;r<Count;r++)
        {if((r&1023)==0)token.ThrowIfCancellationRequested();var row=Row(r);for(var j=0;j<row.Count;j++){var a=Coefficient(row.Offset+j);diagonal[Index(row.Offset+j)]+=row.Weight*a*a;}}
        for(var j=0;j<Variables;j++){diagonal[j]=Math.Max(1e-9,diagonal[j]);inverseRoot[j]=1/Math.Sqrt(diagonal[j]);}
        for(long r=0;r<Count;r++)
        {
            if((r&1023)==0)token.ThrowIfCancellationRequested();var row=Row(r);var norm=0d;
            for(var j=0;j<row.Count;j++)norm+=Math.Abs(Coefficient(row.Offset+j))*inverseRoot[Index(row.Offset+j)];
            for(var j=0;j<row.Count;j++)sums[Index(row.Offset+j)]+=row.Weight*Math.Abs(Coefficient(row.Offset+j))*inverseRoot[Index(row.Offset+j)]*norm;
        }
        var lipschitz=0d;for(var j=0;j<Variables;j++)lipschitz=Math.Max(lipschitz,sums[j]);
        return(diagonal,1.01*lipschitz);
    }
    internal static bool Finite(double x)=>!double.IsNaN(x)&&!double.IsInfinity(x);
    public double DirectionalLipschitz(double[] gradient,double[] d,double globalL,CancellationToken token)
    {
        var denominator=0d;
        for(var j=0;j<Variables;j++)denominator+=gradient[j]*(gradient[j]/d[j]);
        if(!(denominator>0)||!Finite(denominator))return globalL;
        var numerator=0d;
        for(long r=0;r<Count;r++)
        {
            if((r&1023)==0)token.ThrowIfCancellationRequested();var row=Row(r);var directional=0d;
            for(var j=0;j<row.Count;j++){var v=Index(row.Offset+j);directional+=Coefficient(row.Offset+j)*(gradient[v]/d[v]);}
            numerator+=row.Weight*directional*directional;
        }
        var local=1.01*(numerator/denominator);
        return Finite(local)&&local>0?Math.Min(globalL,local):globalL;
    }
}

public enum ProfileSolveOutcome { Converged, IterationCap, NumericFallback, HardSetUnavailable }
public sealed record ProfileSolveReport(ProfileSolveOutcome Outcome,int AcceptedIterations,int Trials,int Restarts,int GradientSweeps,int ObjectiveSweeps,int CheckSweeps,int Projections,double Objective,double ProjectedStep,long VectorCapacityBytes,double OptimalityGap,int OptimalityChecks,int CurvatureSweeps,int Backtracks)
{ public int Components { get; init; } = 1; public string FailureReason { get; init; } = ""; }
public sealed record ProfileSolveResult(double[] Heights,ProfileSolveReport Report);

/// <summary>The single amendment-selected solver. Geometric acceptance is separate.</summary>
public static class NetworkProfileSolve
{
    // Row/star connectivity is the exact separability criterion. Roads joined by
    // a crossing or parallel-profile row stay in the same solve component.
    public static ProfileSolveResult SolveComponents(SparseProfileRows rows,double[] prior,double scale,ProfileStar[] stars,CancellationToken token=default,Action<int,int,ProfileSolveReport>? componentReport=null)
    {
        var parent=new int[rows.Variables];for(var i=0;i<parent.Length;i++)parent[i]=i;
        int Root(int i){while(parent[i]!=i){parent[i]=parent[parent[i]];i=parent[i];}return i;}
        void Join(int a,int b){a=Root(a);b=Root(b);if(a!=b)parent[Math.Max(a,b)]=Math.Min(a,b);}
        for(long i=0;i<rows.Count;i++)
        {if((i&1023)==0)token.ThrowIfCancellationRequested();var row=rows.Row(i);for(var j=1;j<row.Count;j++)Join(rows.Index(row.Offset),rows.Index(row.Offset+j));}
        foreach(var star in stars)foreach(var control in star.Controls)Join(star.Height,control);
        var groups=new Dictionary<int,List<int>>();var local=new int[parent.Length];
        for(var i=0;i<parent.Length;i++){var root=Root(i);if(!groups.TryGetValue(root,out var group))groups[root]=group=new();local[i]=group.Count;group.Add(i);}
        if(groups.Count<=1){var single=SolveComponent(rows,prior,new ProfileHardProjector(rows.Variables,scale,stars),token);componentReport?.Invoke(0,prior.Length,single.Report);return single;}
        var builders=new Dictionary<int,SparseProfileRows>();var blocks=new Dictionary<int,List<ProfileStar>>();
        foreach(var group in groups){builders[group.Key]=new(group.Value.Count);blocks[group.Key]=new();}
        var indices=new int[8];var coefficients=new double[8];var constantObjective=0d;
        for(long i=0;i<rows.Count;i++)
        {
            if((i&1023)==0)token.ThrowIfCancellationRequested();var row=rows.Row(i);
            if(row.Count==0){var residual=row.Kind==ProfileRowKind.UpperHingeCost?Math.Max(0,-row.Target):-row.Target;constantObjective+=.5*row.Weight*residual*residual;continue;}
            for(var j=0;j<row.Count;j++){indices[j]=local[rows.Index(row.Offset+j)];coefficients[j]=rows.Coefficient(row.Offset+j);}
            builders[Root(rows.Index(row.Offset))].Append(indices,coefficients,row.Count,row.Target,row.Weight,row.Kind);
        }
        foreach(var star in stars)
        {var controls=new int[star.Controls.Length];for(var j=0;j<controls.Length;j++)controls[j]=local[star.Controls[j]];blocks[Root(star.Height)].Add(new(local[star.Height],controls,star.Alpha,star.Groups,star.FixedHeight,star.FixedSlopes));}
        var output=new double[prior.Length];ProfileSolveReport? total=null;
        foreach(var group in groups)
        {
            token.ThrowIfCancellationRequested();var seed=new double[group.Value.Count];for(var j=0;j<seed.Length;j++)seed[j]=prior[group.Value[j]];
            var problem=builders[group.Key];problem.Freeze();var result=SolveComponent(problem,seed,new ProfileHardProjector(seed.Length,scale,blocks[group.Key].ToArray()),token);
            componentReport?.Invoke(group.Key,seed.Length,result.Report);
            for(var j=0;j<seed.Length;j++)output[group.Value[j]]=result.Heights[j];
            var report=result.Report;
            if(total==null)total=report;else total=new(
                (ProfileSolveOutcome)Math.Max((int)total.Outcome,(int)report.Outcome),Math.Max(total.AcceptedIterations,report.AcceptedIterations),total.Trials+report.Trials,total.Restarts+report.Restarts,
                total.GradientSweeps+report.GradientSweeps,total.ObjectiveSweeps+report.ObjectiveSweeps,total.CheckSweeps+report.CheckSweeps,total.Projections+report.Projections,
                total.Objective+report.Objective,Math.Max(total.ProjectedStep,report.ProjectedStep),Math.Max(total.VectorCapacityBytes,report.VectorCapacityBytes),
                total.OptimalityGap+report.OptimalityGap,total.OptimalityChecks+report.OptimalityChecks,total.CurvatureSweeps+report.CurvatureSweeps,total.Backtracks+report.Backtracks);
        }
        return new(output,total! with{Components=groups.Count,Objective=total.Objective+constantObjective});
    }

    public static ProfileSolveResult SolveComponent(SparseProfileRows rows,double[] immutablePrior,ProfileHardProjector projector,CancellationToken token=default)
    {
        var n=rows.Variables;if(immutablePrior.Length!=n||projector.Variables!=n)throw new ArgumentException("Profile component dimension mismatch.");
        var x=(double[])immutablePrior.Clone();var y=new double[n];var candidate=new double[n];var gradient=new double[n];var best=new double[n];
        var conditioned=rows.Conditioning(token);var d=conditioned.D;var l=conditioned.L;
        var accepted=0;var trials=0;var restarts=0;var gradients=0;var objectives=0;var checks=0;var projections=0;var projectedStep=double.PositiveInfinity;
        var fx=double.NaN;var bestF=double.NaN;
        var optimalityGap=double.PositiveInfinity;var optimalityChecks=0;
        var curvatureSweeps=0;var backtracks=0;var previousL=l;var acceptedL=l;var trialObjective=double.NaN;
        ProfileSolveResult Finish(ProfileSolveOutcome outcome,string reason="")
            =>new(best,new ProfileSolveReport(outcome,accepted,trials,restarts,gradients,objectives,checks,projections,bestF,projectedStep,checked((long)n*8*8),optimalityGap,optimalityChecks,curvatureSweeps,backtracks){FailureReason=reason});
        // Even a numerical failure must return a finite bounded diagnostic profile.
        // This fallback is not convergence or proof of a satisfied hard set.
        for(var j=0;j<n;j++)best[j]=SparseProfileRows.Finite(x[j])?Math.Max(0,Math.Min(projector.HeightScale,x[j])):0;
        if(!AllFinite(x)||!AllFinite(d)||(n>0&&(!SparseProfileRows.Finite(l)||l<=0)))return Finish(ProfileSolveOutcome.NumericFallback);
        if(!projector.Project(x,d))return Finish(ProfileSolveOutcome.HardSetUnavailable);
        projections++;Array.Copy(x,best,n);Array.Copy(x,y,n);fx=bestF=rows.Evaluate(x,null,token);objectives++;
        if(!SparseProfileRows.Finite(fx))return Finish(ProfileSolveOutcome.NumericFallback);
        if(n==0)return Finish(ProfileSolveOutcome.Converged);
        var history=new double[4];history[0]=fx;var t=1d;var successes=0;
        bool Trial()
        {
            var fy=rows.Evaluate(y,gradient,token);gradients++;
            if(!SparseProfileRows.Finite(fy)||!AllFinite(gradient))return false;
            var local=rows.DirectionalLipschitz(gradient,d,l,token);curvatureSweeps++;
            // At most sixteen local probes, then the proven global bound.
            // Projection and majorization validate each probe; Rayleigh alone
            // is never treated as a bound on every feasible direction.
            for(var attempt=0;attempt<=16;attempt++)
            {
                token.ThrowIfCancellationRequested();trials++;
                acceptedL=attempt==16?l:local;
                for(var j=0;j<n;j++)candidate[j]=y[j]-gradient[j]/(acceptedL*d[j]);
                var finite=AllFinite(candidate);
                if(finite&&projector.Project(candidate,d)&&AllFinite(candidate))
                {
                    projections++;trialObjective=rows.Evaluate(candidate,null,token);objectives++;
                    var linear=0d;var quadratic=0d;
                    for(var j=0;j<n;j++){var delta=candidate[j]-y[j];linear+=gradient[j]*delta;quadratic+=d[j]*delta*delta;}
                    var rhs=fy+linear+.5*acceptedL*quadratic;
                    var margin=1e-12*Math.Max(1,Math.Max(Math.Abs(fy),Math.Abs(rhs)));
                    if(SparseProfileRows.Finite(trialObjective)&&SparseProfileRows.Finite(rhs)&&trialObjective<=rhs+margin)return true;
                }
                if(attempt==16||acceptedL==l)return false;
                backtracks++;local=Math.Min(l,local*2);
            }
            return false;
        }
        for(var iteration=1;iteration<=2000;iteration++)
        {
            token.ThrowIfCancellationRequested();
            if(!AllFinite(y)||!Trial())
            {
                // A failed extrapolated probe is not evidence that a step from
                // the last accepted point is unavailable. Restart acceleration
                // and retry the same safeguarded FISTA trial before falling back.
                restarts++;t=1;Array.Copy(x,y,n);
                if(!Trial())return Finish(ProfileSolveOutcome.NumericFallback,"restarted_projected_majorization_trial_failed");
            }
            var fc=trialObjective;
            if(!SparseProfileRows.Finite(fc))return Finish(ProfileSolveOutcome.NumericFallback);
            var tolerance=1e-12*Math.Max(1,Math.Abs(fx));
            if(fc>fx+tolerance)
            {
                restarts++;t=1;Array.Copy(x,y,n);token.ThrowIfCancellationRequested();
                if(!Trial())return Finish(ProfileSolveOutcome.NumericFallback,"restart_majorization_trial_failed");
                fc=trialObjective;
                if(!SparseProfileRows.Finite(fc)||fc>fx+tolerance)return Finish(ProfileSolveOutcome.NumericFallback,"restart_objective_increased");
            }
            // Equal rounded objectives can still have substantially different
            // first-order gaps near an optimum with a large constant residual.
            // Keep the latest tied minimizer; the independent gap guard still
            // certifies the returned vector, never the objective tie itself.
            if(fc<=bestF){bestF=fc;Array.Copy(candidate,best,n);}
            var tNext=(1+Math.Sqrt(1+4*t*t*(acceptedL/previousL)))/2;var momentum=(t-1)/tNext;
            for(var j=0;j<n;j++)y[j]=candidate[j]+momentum*(candidate[j]-x[j]);
            var old=x;x=candidate;candidate=old;fx=fc;t=tNext;previousL=acceptedL;accepted=iteration;
            if(iteration%10!=0)continue;
            rows.Evaluate(x,gradient,token,objective:false);checks++;
            if(!AllFinite(gradient))return Finish(ProfileSolveOutcome.NumericFallback);
            for(var j=0;j<n;j++)candidate[j]=x[j]-gradient[j]/(l*d[j]);
            if(!AllFinite(candidate)||!projector.Project(candidate,d)||!AllFinite(candidate))return Finish(ProfileSolveOutcome.NumericFallback);
            projections++;projectedStep=0;for(var j=0;j<n;j++)projectedStep=Math.Max(projectedStep,Math.Abs(candidate[j]-x[j]));
            var check=iteration/10;var slot=check%4;var oldObjective=check>=3?history[(check-3)%4]:double.NaN;
            history[slot]=fx;
            var relative=check>=3?Math.Abs(fx-oldObjective)/Math.Max(1,Math.Abs(oldObjective)):double.PositiveInfinity;
            optimalityGap=double.PositiveInfinity;
            if(projectedStep<=.0005&&relative<=1e-9)
            {
                optimalityChecks++;
                // Certification belongs to the vector actually returned. Tiny
                // permitted objective excursions can leave x less accurate than
                // the retained best vector despite identical printed objectives.
                var same=true;for(var j=0;j<n;j++)if(best[j]!=x[j]){same=false;break;}
                if(!same)
                {
                    rows.Evaluate(best,gradient,token,objective:false);checks++;
                    if(!AllFinite(gradient))return Finish(ProfileSolveOutcome.NumericFallback);
                    for(var j=0;j<n;j++)candidate[j]=best[j]-gradient[j]/(l*d[j]);
                    if(!AllFinite(candidate)||!projector.Project(candidate,d)||!AllFinite(candidate))return Finish(ProfileSolveOutcome.NumericFallback);
                    projections++;projectedStep=0;for(var j=0;j<n;j++)projectedStep=Math.Max(projectedStep,Math.Abs(candidate[j]-best[j]));
                }
                if(projector.TryLinearMinimum(gradient,out var support))
                {
                    var atX=0d;var magnitude=Math.Abs(support);
                    for(var j=0;j<n;j++){var term=gradient[j]*best[j];atX+=term;magnitude+=Math.Abs(term);}
                    // Conservative summation margin, not an exact-arithmetic
                    // interval guarantee for the preceding gradient evaluation.
                    var roundoff=64*2.2204460492503131e-16*Math.Max(1,n)*magnitude;
                    optimalityGap=Math.Max(0,atX-support)+roundoff;
                }
            }
            successes=projectedStep<=.0005&&optimalityGap<=1e-9*Math.Max(1,Math.Abs(bestF))?successes+1:0;
            if(successes==3)return Finish(ProfileSolveOutcome.Converged);
        }
        return Finish(ProfileSolveOutcome.IterationCap);
    }
    private static bool AllFinite(double[] values)
    {for(var i=0;i<values.Length;i++)if(!SparseProfileRows.Finite(values[i]))return false;return true;}
}
