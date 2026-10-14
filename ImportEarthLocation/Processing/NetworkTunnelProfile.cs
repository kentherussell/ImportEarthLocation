using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

/// <summary>Joint fixed-row cover/grade solve. Never performs a cover-only rewrite after grading.</summary>
public static class NetworkTunnelProfile
{
    public static IReadOnlyList<PreparedSegment> Solve(IReadOnlyList<PreparedSegment> source,float scale,
        IReadOnlyList<(int index,float t,float ceiling)> cover,ImportLog log,CancellationToken token,
        IReadOnlyDictionary<SourceNodeKey,double>? verifiedBoundaryHeights=null)
    {
        if(cover.Count==0)return source;
        var ordered=Enumerable.Range(0,source.Count).OrderBy(i=>source[i].Kind).ThenBy(i=>source[i].StartNodeId)
            .ThenBy(i=>source[i].EndNodeId).ThenBy(i=>source[i].WayId).ToArray();
        var ids=new Dictionary<SourceNodeKey,int>();var keys=new Dictionary<int,SourceNodeKey>();var prior=new List<double>();
        var from=new int[source.Count];var to=new int[source.Count];var c1=new int[source.Count];var c2=new int[source.Count];
        int Node(PreparedSegment s,long id,float height)
        {var key=new SourceNodeKey(s.Kind,id);if(ids.TryGetValue(key,out var v))return v;v=prior.Count;ids.Add(key,v);keys.Add(v,key);prior.Add(math.clamp((double)height,0,scale));return v;}
        foreach(var i in ordered)
        {
            token.ThrowIfCancellationRequested();var s=source[i];
            from[i]=Node(s,s.StartNodeId,s.CourseStart.y);to[i]=Node(s,s.EndNodeId,s.CourseEnd.y);
            c1[i]=prior.Count;prior.Add(math.clamp((double)s.CourseControl1.y,0,scale));
            c2[i]=prior.Count;prior.Add(math.clamp((double)s.CourseControl2.y,0,scale));
        }
        var ports=new Dictionary<int,List<(int Variable,double Reach,double2 Direction)>>();
        void Port(int node,int variable,double2 direction,bool start)
        {
            var reach=math.length(direction);
            // A finite stationary handle has no longitudinal derivative model.
            // Leave its control boxed and report quality, rather than inventing
            // a reach or treating the source corner as a native safety exception.
            if(!(reach>1e-8)){log.Count("Profile degenerate horizontal ports");return;}
            if(!ports.TryGetValue(node,out var list))ports[node]=list=new();
            list.Add((variable,start?reach:-reach,direction/reach));
        }
        foreach(var i in ordered)
        {var s=source[i];Port(from[i],c1[i],(s.CourseControl1-s.CourseStart).xz,true);Port(to[i],c2[i],(s.CourseControl2-s.CourseEnd).xz,false);}
        var stars=new List<ProfileStar>();
        foreach(var pair in ports.OrderBy(p=>p.Key))
        {
            var p=pair.Value;var groups=new int[p.Count];var alpha=new double[p.Count];var controls=new int[p.Count];
            for(var j=0;j<p.Count;j++){controls[j]=p[j].Variable;alpha[j]=p[j].Reach;groups[j]=j;}
            // A through join has opposing OUTWARD tangents and coherent travel
            // orientation. Do not tie unrelated intersection branches together.
            if(p.Count==2&&math.dot(p[0].Direction,p[1].Direction)<-.9)
            {groups[1]=0;alpha[1]=-Math.Sign(alpha[0])*Math.Abs(alpha[1]);}
            double? fixedHeight=null;
            if(verifiedBoundaryHeights!=null&&verifiedBoundaryHeights.TryGetValue(keys[pair.Key],out var fixedValue))
            {
                if(!SparseProfileRows.Finite(fixedValue)||fixedValue<0||fixedValue>scale)
                    throw new InvalidOperationException("Verified native boundary is not representable in the fixed map-height box.");
                fixedHeight=fixedValue;
            }
            stars.Add(new ProfileStar(pair.Key,controls,alpha,groups,fixedHeight));
        }
        if(verifiedBoundaryHeights!=null)foreach(var pair in ids)
            if(!ports.ContainsKey(pair.Value)&&verifiedBoundaryHeights.TryGetValue(pair.Key,out var fixedValue))
                stars.Add(new ProfileStar(pair.Value,Array.Empty<int>(),Array.Empty<double>(),Array.Empty<int>(),fixedValue));
        var rows=new SparseProfileRows(prior.Count);var indices=new int[8];var coefficients=new double[8];
        void Row(int count,double target,double weight,ProfileRowKind kind)
            =>rows.Append(indices,coefficients,count,target,weight,kind);
        var eps=Math.Max(.05,2*scale/65535d);
        var coverByEdge=new List<(float T,float Ceiling)>[source.Count];
        foreach(var sample in cover)
        {if(coverByEdge[sample.index]==null)coverByEdge[sample.index]=new();coverByEdge[sample.index].Add((sample.t,sample.ceiling));}
        foreach(var i in ordered)
        {
            token.ThrowIfCancellationRequested();var s=source[i];var budget=NetworkVerticalProfiles.CutFillBudget(s);
            var length=NetworkVerticalProfiles.ArcLength(s);var support=Math.Max(1e-8,length);
            indices[0]=from[i];indices[1]=c1[i];indices[2]=c2[i];indices[3]=to[i];
            for(var sample=0;sample<=8;sample++)
            {
                var t=sample/8d;var u=1-t;
                coefficients[0]=u*u*u/budget;coefficients[1]=3*u*u*t/budget;
                coefficients[2]=3*u*t*t/budget;coefficients[3]=t*t*t/budget;
                var weight=support/8*(sample==0||sample==8?.5:1);
                Row(4,NetworkNormalizer.Evaluate(s,(float)t).y/budget,weight,ProfileRowKind.EqualityCost);
            }
            // Freeze derivative leaves first; the row parameter and horizontal
            // lower bound both refer to that leaf's own parameterization.
            var curve=new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);
            var desired=NetworkVerticalProfiles.DesiredGrade(s);var exceptional=NetworkGeometrySafety.GradeLimit(s);
            var basis0=new double[4];var basis1=new double[4];
            void Derivatives(Cubic3d leaf,double begin,double end,int depth)
            {
                token.ThrowIfCancellationRequested();var chord=leaf.P3.xz-leaf.P0.xz;var size=math.length(chord);
                var direction=size>0?chord/size:double2.zero;
                var d0=3*(leaf.P1-leaf.P0).xz;var d1=3*(leaf.P2-leaf.P1).xz;var d2=3*(leaf.P3-leaf.P2).xz;
                var minimum=Math.Min(math.dot(d0,direction),Math.Min(math.dot(d1,direction),math.dot(d2,direction)));
                var maximum=Math.Max(math.length(d0),Math.Max(math.length(d1),math.length(d2)));
                if((minimum<=1e-8||minimum<maximum*.95)&&depth<16&&maximum>1e-8)
                {var split=NetworkDesignChecks.Split(leaf,.5);var middle=(begin+end)/2;Derivatives(split.Left,begin,middle,depth+1);Derivatives(split.Right,middle,end,depth+1);return;}
                if(!(minimum>1e-8)){log.Count("Profile uncertified derivative leaves");return;}
                var leafWeight=support*(end-begin)/3;
                for(var k=0;k<3;k++)
                {
                    void Basis(int control,double[] target)
                    {
                        switch(control)
                        {case 0:NetworkFootprintMath.Blossom(begin,begin,begin,target);break;case 1:NetworkFootprintMath.Blossom(begin,begin,end,target);break;
                        case 2:NetworkFootprintMath.Blossom(begin,end,end,target);break;default:NetworkFootprintMath.Blossom(end,end,end,target);break;}
                    }
                    Basis(k,basis0);Basis(k+1,basis1);
                    for(var policy=0;policy<2;policy++)
                    {
                        var limit=policy==0?desired:exceptional;
                        for(var j=0;j<4;j++)coefficients[j]=3*(basis1[j]-basis0[j])/(minimum*limit);
                        Row(4,1,leafWeight*(limit==exceptional?1000:1),ProfileRowKind.UpperHingeCost);
                        for(var j=0;j<4;j++)coefficients[j]=-coefficients[j];
                        Row(4,1,leafWeight*(limit==exceptional?1000:1),ProfileRowKind.UpperHingeCost);
                    }
                }
            }
            Derivatives(curve,0,1,0);
            var samples=coverByEdge[i];
            if(samples!=null)foreach(var sample in samples)
            {
                var t=(double)sample.T;var u=1-t;
                coefficients[0]=u*u*u/eps;coefficients[1]=3*u*u*t/eps;coefficients[2]=3*u*t*t/eps;coefficients[3]=t*t*t/eps;
                // The existing finite cover observations are frozen together
                // with grades. Continuous terrain-triangle certification is
                // tracked separately, never claimed from these observations.
                Row(4,(sample.Ceiling-.25)/eps,10000d/samples.Count,ProfileRowKind.UpperHingeCost);
            }
        }
        for(var i=0;i<prior.Count;i++)
        {indices[0]=i;coefficients[0]=1;Row(1,prior[i],1e-8,ProfileRowKind.EqualityCost);}
        rows.Freeze();var clock=System.Diagnostics.Stopwatch.StartNew();
        var solution=NetworkProfileSolve.SolveComponent(rows,prior.ToArray(),new ProfileHardProjector(prior.Count,scale,stars.ToArray()),token);
        var result=source.ToArray();var maximumGrade=0f;var exposed=0;var maxExcess=0d;var gradeFailures=0;
        foreach(var i in ordered)
        {
            var s=source[i];var a=s.CourseStart;var b=s.CourseControl1;var c=s.CourseControl2;var d=s.CourseEnd;
            a.y=(float)solution.Heights[from[i]];b.y=(float)solution.Heights[c1[i]];c.y=(float)solution.Heights[c2[i]];d.y=(float)solution.Heights[to[i]];
            var start=s.Start;var end=s.End;start.y=a.y;end.y=d.y;
            result[i]=s with{Start=start,End=end,Control1=b-s.StartCurveOffset,Control2=c-s.EndCurveOffset,
                FinalDeckCurve=null,HorizontalParent=null,StartElevation=(s.StartElevation??0)+a.y-s.CourseStart.y,EndElevation=(s.EndElevation??0)+d.y-s.CourseEnd.y};
            maximumGrade=Math.Max(maximumGrade,NetworkVerticalProfiles.MaximumGrade(result[i]));
            if(!NetworkVerticalProfiles.WithinGradeCeiling(result[i],NetworkGeometrySafety.GradeLimit(s)))gradeFailures++;
        }
        foreach(var sample in cover)
        {var excess=NetworkNormalizer.Evaluate(result[sample.index],sample.t).y-sample.ceiling;if(excess>.05){exposed++;maxExcess=Math.Max(maxExcess,excess);}}
        var report=solution.Report;
        log.AddDuration("Joint tunnel profile solve",clock.Elapsed.TotalSeconds);
        log.Count("Joint profile variables",prior.Count);log.AddCounter("Joint profile rows",rows.Count);
        // Observed cover is not the complete continuous terrain/composition
        // certificate. Keep that review requirement visible until the exact
        // triangle builder is integrated; never label a sampled pass Certified.
        log.Count("Network repair quality observations");
        log.Count("Joint profile exposed cover samples",exposed);log.Count("Joint profile exceptional-grade failures",gradeFailures);
        log.Info($"Joint tunnel profile: {report.Outcome}; {report.AcceptedIterations} iterations; exposed samples={exposed}; maximum grade={maximumGrade:0.#####}. Native-safe geometry retained for inspection.");
        if(log.DiagnosticsEnabled)
        {
            log.Diagnostic($"V-PROFILE JOINT outcome={report.Outcome}; variables={prior.Count}; rows={rows.Count}; terms={rows.Terms}; packedCapacityBytes={rows.CapacityBytes}; vectorBytes={report.VectorCapacityBytes}; iterations={report.AcceptedIterations}; restarts={report.Restarts}; backtracks={report.Backtracks}; gap={report.OptimalityGap:R}; objective={report.Objective:R}; maximumGrade={maximumGrade:R}; coverResidual={maxExcess:R}; observedCoverOnly=true; continuousCoverCertificate=false; coverOnlyPostRewrite=false; fixedHeightBox=true; soleOptimizer=FISTA");
            log.DiagnosticRecords("joint-tunnel-profile-input.ndjson",DiagnosticArtifacts.NetworkRecords(source));
            log.DiagnosticRecords("joint-tunnel-profile.ndjson",DiagnosticArtifacts.NetworkRecords(result));
            var failures=Enumerable.Range(0,result.Length).Where(i=>!NetworkVerticalProfiles.WithinGradeCeiling(result[i],NetworkGeometrySafety.GradeLimit(result[i])));
            var coverFailures=cover.Where(p=>NetworkNormalizer.Evaluate(result[p.index],p.t).y>p.ceiling+.05).Select(p=>p.index);
            log.DiagnosticRecords("joint-tunnel-profile-failures.ndjson",DiagnosticArtifacts.NetworkRecords(failures.Concat(coverFailures).Distinct().Take(32).Select(i=>result[i]).ToArray()));
            log.DiagnosticRecords("joint-tunnel-cover-observations.ndjson",cover
                .OrderByDescending(p=>NetworkNormalizer.Evaluate(result[p.index],p.t).y-p.ceiling).Take(64)
                .Select(p=>(object)new
                {
                    constraint="frozen-tunnel-cover-observation",policyStatus="implementation-assumption-awaiting-game-validation",
                    evidence="prepared-deck-versus-final-raster-not-native-readback",nativeRenderingVerified=false,
                    result[p.index].WayId,result[p.index].StartNodeId,result[p.index].EndNodeId,t=p.t,
                    beforeDeckY=NetworkNormalizer.Evaluate(source[p.index],p.t).y,
                    afterDeckY=NetworkNormalizer.Evaluate(result[p.index],p.t).y,ceilingY=p.ceiling,
                    residualMeters=NetworkNormalizer.Evaluate(result[p.index],p.t).y-p.ceiling,
                    solveMarginMeters=.25,observationToleranceMeters=.05,normalizationMeters=eps,
                    continuousCoverCertificate=false,requestedObservations=cover.Count,
                    sampleSelection="largest-residual-first-at-most-64-not-complete-coverage",
                    solverOutcome=report.Outcome.ToString(),qualityFailureAction="RequiresReview-not-import-abort"
                }));
        }
        return result;
    }
}
