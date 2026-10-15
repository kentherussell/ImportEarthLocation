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
    public sealed record TerrainReference(ushort[] Heights,double Width,Func<float3,float?>? Water=null);
    private readonly record struct Crossing(int Upper,float T,int Lower,float U);
    private readonly record struct Span(int Edge,float T,float U,float3 A,float3 B);

    private static Crossing[] Crossings(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token)
    {
        var spans=new List<Span>();var grid=new Dictionary<(int,int),List<int>>();
        var nodes=new Dictionary<SourceNodeKey,int>();var arcs=new List<List<ProfileGraphArc>>();
        int Node(PreparedSegment s,long id){var key=new SourceNodeKey(s.Kind,id);if(nodes.TryGetValue(key,out var n))return n;n=nodes.Count;nodes[key]=n;arcs.Add(new());return n;}
        foreach(var s in source)
        {var a=Node(s,s.StartNodeId);var b=Node(s,s.EndNodeId);var cost=Math.Max(1,NetworkVerticalProfiles.ArcLength(s))*NetworkVerticalProfiles.DesiredGrade(s);arcs[a].Add(new(b,cost));arcs[b].Add(new(a,cost));}
        var graph=new NetworkProfileGraph(arcs.Select(a=>a.ToArray()).ToArray());var distances=new Dictionary<int,double[]>();
        double Distance(int a,int b)
        {if(!distances.TryGetValue(a,out var row)){if(distances.Count>=64)distances.Clear();row=graph.Distances(new[]{a},8,token);distances[a]=row;}return row[b];}
        double Blend(double a,double b,double t)=>t<=1e-8?a:t>=1-1e-8?b:(1-t)*a+t*b;
        for(var i=0;i<source.Count;i++)
        {
            var s=source[i];if(s.Tunnel)continue;var count=Math.Max(8,Math.Min(256,(int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(s)/24)));
            var previous=NetworkNormalizer.Evaluate(s,0);
            for(var j=1;j<=count;j++)
            {
                token.ThrowIfCancellationRequested();var next=NetworkNormalizer.Evaluate(s,j/(float)count);var id=spans.Count;
                spans.Add(new(i,(j-1)/(float)count,j/(float)count,previous,next));
                var low=math.min(previous.xz,next.xz);var high=math.max(previous.xz,next.xz);
                for(var x=(int)Math.Floor(low.x/64);x<=(int)Math.Floor(high.x/64);x++)for(var z=(int)Math.Floor(low.y/64);z<=(int)Math.Floor(high.y/64);z++)
                {if(!grid.TryGetValue((x,z),out var list))grid[(x,z)]=list=new();list.Add(id);}
                previous=next;
            }
        }
        var pairs=new HashSet<(int,int)>();var result=new List<Crossing>();var contradictory=0;
        foreach(var cell in grid.Values)for(var i=0;i<cell.Count;i++)for(var j=i+1;j<cell.Count;j++)
        {
            token.ThrowIfCancellationRequested();var a=spans[cell[i]];var b=spans[cell[j]];if(a.Edge==b.Edge)continue;
            var s=source[a.Edge];var t=source[b.Edge];if(!s.Bridge&&!t.Bridge||s.Layer==t.Layer&&s.Bridge==t.Bridge)continue;
            if(s.Kind==t.Kind&&(s.StartNodeId==t.StartNodeId||s.StartNodeId==t.EndNodeId||s.EndNodeId==t.StartNodeId||s.EndNodeId==t.EndNodeId))continue;
            var u=a.B.xz-a.A.xz;var v=b.B.xz-b.A.xz;var determinant=u.x*v.y-u.y*v.x;
            if(Math.Abs(determinant)<1e-6||Math.Abs(math.dot(math.normalizesafe(u),math.normalizesafe(v)))>.95f)continue;
            var delta=b.A.xz-a.A.xz;var pa=(delta.x*v.y-delta.y*v.x)/determinant;var pb=(delta.x*u.y-delta.y*u.x)/determinant;
            if(pa<0||pa>1||pb<0||pb>1)continue;
            var ta=math.lerp(a.T,a.U,pa);var tb=math.lerp(b.T,b.U,pb);
            var upper=s.Layer!=t.Layer?s.Layer>t.Layer:s.Bridge;
            var crossing=upper?new Crossing(a.Edge,ta,b.Edge,tb):new Crossing(b.Edge,tb,a.Edge,ta);
            var key=(Math.Min(a.Edge,b.Edge),Math.Max(a.Edge,b.Edge));if(pairs.Contains(key))continue;
            var hi=source[crossing.Upper];var lo=source[crossing.Lower];
            if(hi.Kind==lo.Kind)
            {
                var aa=nodes[new(hi.Kind,hi.StartNodeId)];var ab=nodes[new(hi.Kind,hi.EndNodeId)];var ba=nodes[new(lo.Kind,lo.StartNodeId)];var bb=nodes[new(lo.Kind,lo.EndNodeId)];
                var allowance=Blend(Blend(Distance(aa,ba),Distance(aa,bb),crossing.U),Blend(Distance(ab,ba),Distance(ab,bb),crossing.U),crossing.T);
                if(allowance<7-.01)
                {pairs.Add(key);contradictory++;if(contradictory<=16)log.Diagnostic($"CROSSING INTERPRETATION REVIEW upperWay={hi.WayId}; lowerWay={lo.WayId}; requestedGapMeters=7; connectedGradeAllowanceMeters={allowance:R}; inferredObstacleOmitted=true; realConnectivityPreserved=true");continue;}
            }
            pairs.Add(key);result.Add(crossing);
        }
        log.Diagnostic($"PROFILE CROSSINGS curvePolylineStations=true; crossingConstraints={result.Count}; contradictoryInferredObstacles={contradictory}; endpointChordIntersections=false; ceilingExpansion=false; requiresReview={contradictory>0}");
        return result.ToArray();
    }

    private static void AddParallelRows(IReadOnlyList<PreparedSegment> source,int[] from,int[] to,int[] c1,int[] c2,SparseProfileRows rows,CancellationToken token)
    {
        var grid=new Dictionary<(int,int),List<(int Edge,float T,float3 Point)>>();
        bool Eligible(PreparedSegment s)=>s.Kind==NetworkKind.Road&&s.OneWay&&!s.ProtectedLink&&s.CorridorId!=0;
        for(var i=0;i<source.Count;i++)if(Eligible(source[i]))
        {
            var count=Math.Max(8,(int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(source[i])/12));
            for(var j=0;j<=count;j++){var t=j/(float)count;var p=NetworkNormalizer.Evaluate(source[i],t);var key=((int)Math.Floor(p.x/64),(int)Math.Floor(p.z/64));if(!grid.TryGetValue(key,out var list))grid[key]=list=new();list.Add((i,t,p));}
        }
        var indices=new int[8];var coefficients=new double[8];
        void Basis(int slot,int edge,double t,double sign)
        {var u=1-t;indices[slot]=from[edge];indices[slot+1]=c1[edge];indices[slot+2]=c2[edge];indices[slot+3]=to[edge];coefficients[slot]=sign*u*u*u;coefficients[slot+1]=sign*3*u*u*t;coefficients[slot+2]=sign*3*u*t*t;coefficients[slot+3]=sign*t*t*t;}
        for(var i=0;i<source.Count;i++)if(Eligible(source[i]))for(var j=0;j<=2;j++)
        {
            token.ThrowIfCancellationRequested();var s=source[i];var t=j*.5f;var p=NetworkNormalizer.Evaluate(s,t);var gx=(int)Math.Floor(p.x/64);var gz=(int)Math.Floor(p.z/64);
            var best=80f*80;var edge=-1;var parameter=0f;
            var heading=math.normalizesafe((s.End-s.Start).xz);
            for(var x=gx-1;x<=gx+1;x++)for(var z=gz-1;z<=gz+1;z++)if(grid.TryGetValue((x,z),out var list))foreach(var candidate in list)
            {
                var other=source[candidate.Edge];if(candidate.Edge<=i||other.CorridorId!=s.CorridorId||other.Layer!=s.Layer||other.Bridge!=s.Bridge||other.Tunnel!=s.Tunnel||math.dot(heading,math.normalizesafe((other.End-other.Start).xz))>-.9f)continue;
                var d=math.lengthsq(p.xz-candidate.Point.xz);if(d>=best)continue;best=d;edge=candidate.Edge;parameter=candidate.T;
            }
            if(edge<0)continue;Basis(0,i,t,1);Basis(4,edge,parameter,-1);rows.Append(indices,coefficients,8,0,1000,ProfileRowKind.EqualityCost);
        }
    }
    public static IReadOnlyList<PreparedSegment> Solve(IReadOnlyList<PreparedSegment> source,float scale,
        IReadOnlyList<(int index,float t,float ceiling)> cover,ImportLog log,CancellationToken token,
        IReadOnlyDictionary<SourceNodeKey,double>? verifiedBoundaryHeights=null,TerrainReference? design=null)
    {
        if(source.Count==0||cover.Count==0&&design==null)return source;
        float Ground(float3 point)=>NetworkProcessor.Sample(point.x,point.z,design!.Heights,scale,design.Width);
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
                if(design!=null)
                {
                    var p=NetworkNormalizer.Evaluate(s,(float)t);var ground=Ground(p);
                    var lower=ground-(s.Tunnel?120:s.Bridge?0:budget);var upper=ground+(s.Bridge?(s.Kind==NetworkKind.Rail?30:60):budget);
                    // Fixed terrain-relative observations: never increase these
                    // limits in response to another road's current iteration.
                    Row(4,upper/budget,weight*1000,ProfileRowKind.UpperHingeCost);
                    for(var k=0;k<4;k++)coefficients[k]=-coefficients[k];
                    Row(4,-lower/budget,weight*1000,ProfileRowKind.UpperHingeCost);
                    var level=design.Water?.Invoke(p);
                    if(s.Bridge&&!s.Tunnel&&level.HasValue&&math.isfinite(level.Value))
                        Row(4,-(level.Value+7)/budget,weight*1000,ProfileRowKind.UpperHingeCost);
                }
            }
            if(design!=null)
            {
                // Penalize change of longitudinal grade, not just maximum slope.
                // z_ss = z_tt/|h_t|^2 - z_t dot(h_t,h_tt)/|h_t|^4.
                var transition=s.Kind==NetworkKind.Rail?250d:s.ProtectedLink?60d:150d;
                for(var sample=0;sample<3;sample++)
                {
                    var t=(sample+.5)/3;var u=1-t;
                    var h1=3*u*u*(double2)(s.CourseControl1-s.CourseStart).xz+6*u*t*(double2)(s.CourseControl2-s.CourseControl1).xz+3*t*t*(double2)(s.CourseEnd-s.CourseControl2).xz;
                    var h2=6*u*(double2)(s.CourseControl2-2*s.CourseControl1+s.CourseStart).xz+6*t*(double2)(s.CourseEnd-2*s.CourseControl2+s.CourseControl1).xz;
                    var square=(double)math.lengthsq(h1);if(square<1e-8)continue;
                    var correction=math.dot(h1,h2)/(square*square);var normalization=transition/NetworkVerticalProfiles.DesiredGrade(s);
                    var first=new[]{-3*u*u,3*u*u-6*u*t,6*u*t-3*t*t,3*t*t};
                    var second=new[]{6*u,-12*u+6*t,6*u-12*t,6*t};
                    for(var k=0;k<4;k++)coefficients[k]=normalization*(second[k]/square-first[k]*correction);
                    Row(4,0,support/3,ProfileRowKind.EqualityCost);
                }
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
        var crossings=design==null?Array.Empty<Crossing>():Crossings(source,log,token);
        foreach(var crossing in crossings)
        {
            void Basis(int offset,int edge,double t,double sign)
            {
                var u=1-t;indices[offset]=from[edge];indices[offset+1]=c1[edge];indices[offset+2]=c2[edge];indices[offset+3]=to[edge];
                coefficients[offset]=sign*u*u*u/eps;coefficients[offset+1]=sign*3*u*u*t/eps;
                coefficients[offset+2]=sign*3*u*t*t/eps;coefficients[offset+3]=sign*t*t*t/eps;
            }
            Basis(0,crossing.Upper,crossing.T,-1);Basis(4,crossing.Lower,crossing.U,1);
            Row(8,-7/eps,1000,ProfileRowKind.UpperHingeCost);
        }
        if(design!=null)AddParallelRows(source,from,to,c1,c2,rows,token);
        for(var i=0;i<prior.Count;i++)
        {indices[0]=i;coefficients[0]=1;Row(1,prior[i],1e-8,ProfileRowKind.EqualityCost);}
        rows.Freeze();var clock=System.Diagnostics.Stopwatch.StartNew();
        var outcomes=new int[4];var componentFailures=0;
        var solution=NetworkProfileSolve.SolveComponents(rows,prior.ToArray(),scale,stars.ToArray(),token,(id,count,report)=>
        {
            outcomes[(int)report.Outcome]++;
            if(report.Outcome>=ProfileSolveOutcome.NumericFallback&&componentFailures++<16)
                log.Diagnostic($"PROFILE COMPONENT root={id}; variables={count}; outcome={report.Outcome}; reason={report.FailureReason}; iterations={report.AcceptedIterations}; objective={report.Objective:R}; bestFiniteProfileRetained=true");
        });
        var result=source.ToArray();var maximumGrade=0f;var exposed=0;var maxExcess=0d;var gradeFailures=0;
        foreach(var i in ordered)
        {
            var s=source[i];var a=s.CourseStart;var b=s.CourseControl1;var c=s.CourseControl2;var d=s.CourseEnd;
            a.y=(float)solution.Heights[from[i]];b.y=(float)solution.Heights[c1[i]];c.y=(float)solution.Heights[c2[i]];d.y=(float)solution.Heights[to[i]];
            var start=s.Start;var end=s.End;start.y=a.y;end.y=d.y;
            result[i]=s with{Start=start,End=end,Control1=b-s.StartCurveOffset,Control2=c-s.EndCurveOffset,
                FinalDeckCurve=null,HorizontalParent=null,StartElevation=design==null?(s.StartElevation??0)+a.y-s.CourseStart.y:a.y-Ground(a),EndElevation=design==null?(s.EndElevation??0)+d.y-s.CourseEnd.y:d.y-Ground(d)};
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
            log.Diagnostic($"PROFILE DESIGN fixedTerrainReference={design!=null}; crossingStations={crossings.Length}; independentComponents={report.Components}; convergedComponents={outcomes[0]}; cappedComponents={outcomes[1]}; numericFallbackComponents={outcomes[2]}; unavailableComponents={outcomes[3]}; perComponentIterationCap=2000; longitudinalSmoothness={design!=null}; repeatedEndpointRaising=false; clearanceDatumMeters=7; nativeCompositionClearanceStillRequiresReadback=true");
            log.DiagnosticRecords("joint-tunnel-profile-input.ndjson",DiagnosticArtifacts.NetworkRecords(source.Take(32).ToArray()));
            log.DiagnosticRecords("joint-tunnel-profile.ndjson",DiagnosticArtifacts.NetworkRecords(result.Take(32).ToArray()));
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
