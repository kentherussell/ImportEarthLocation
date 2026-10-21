using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Native cuts are observations of a functional profile, not independent bends.
// Substitute exact cubic restriction coefficients into the existing objective;
// the optimizer, physical rows and hard boundary contracts remain unchanged.
public static class NetworkProfileParents
{
    public sealed record Binding(int[] Variables,double[] Coefficients,double Offset=0)
    {
        public double Evaluate(double[] values)
        {var result=Offset;for(var i=0;i<Variables.Length;i++)result+=Coefficients[i]*values[Variables[i]];return result;}
    }
    public sealed record Reduction(SparseProfileRows Rows,double[] Prior,ProfileStar[] Stars,Binding[] Bindings)
    {
        public string Encoding { get; init; }="exact-cubic-parent-restriction";
        public double[] Expand(double[] values)=>Bindings.Select(b=>b.Evaluate(values)).ToArray();
    }
    private sealed record Parent(int[] Sections,double Begin,double End,Dictionary<int,(double Begin,double End)>? Stations=null);

    private static double MaximumSpan(PreparedSegment s)=>s.Kind==NetworkKind.Rail||s.Classification is "motorway" or "trunk"?500:100;

    public static ProfileSolveResult CertifyFixedResult(Reduction reduction,SparseProfileRows original,ProfileStar[] stars,double scale,
        ProfileSolveResult result,CancellationToken token)
    {
        var expanded=reduction.Expand(result.Heights);var report=result.Report with{OriginalVariables=reduction.Bindings.Length};
        if(report.Outcome!=ProfileSolveOutcome.Converged)return new(expanded,report);
        var gradient=new double[expanded.Length];var variableCost=0d;
        for(long r=0;r<original.Count;r++)
        {
            if((r&1023)==0)token.ThrowIfCancellationRequested();var row=original.Row(r);var active=false;var residual=-row.Target;
            for(var j=0;j<row.Count;j++)
            {var id=original.Index(row.Offset+j);residual+=original.Coefficient(row.Offset+j)*expanded[id];active|=reduction.Bindings[id].Variables.Length>0;}
            // Frozen-only penalties are constants, not a looser stopping scale.
            if(!active)continue;if(row.Kind==ProfileRowKind.UpperHingeCost)residual=Math.Max(0,residual);
            variableCost+=.5*row.Weight*residual*residual;
            for(var j=0;j<row.Count;j++)
            {var id=original.Index(row.Offset+j);if(reduction.Bindings[id].Variables.Length>0)gradient[id]+=row.Weight*residual*original.Coefficient(row.Offset+j);}
        }
        var projector=new ProfileHardProjector(expanded.Length,scale,stars);var gap=double.PositiveInfinity;
        if(projector.TryLinearMinimum(gradient,out var lower))
        {
            var at=0d;var magnitude=Math.Abs(lower);
            for(var i=0;i<expanded.Length;i++){var term=expanded[i]*gradient[i];at+=term;magnitude+=Math.Abs(term);}
            gap=Math.Max(0,at-lower)+64*2.2204460492503131e-16*Math.Max(1,expanded.Length)*magnitude;
        }
        var conditioning=original.Conditioning(token);var step=double.PositiveInfinity;var check=new double[expanded.Length];var bound=conditioning.L==0?1:conditioning.L;
        for(var i=0;i<check.Length;i++)check[i]=expanded[i]-gradient[i]/(bound*conditioning.D[i]);
        if(projector.Project(check,conditioning.D))
        {step=0;for(var i=0;i<check.Length;i++)step=Math.Max(step,Math.Abs(check[i]-expanded[i]));}
        var reducedCost=reduction.Rows.Evaluate(result.Heights,null,token,includeConstantObjective:false);
        var tolerance=1e-9*Math.Max(1,Math.Abs(reducedCost));
        var agrees=SparseProfileRows.Finite(variableCost)&&SparseProfileRows.Finite(reducedCost)&&Math.Abs(variableCost-reducedCost)<=tolerance&&gap<=tolerance&&step<=.0005;
        return new(expanded,report with{Outcome=agrees?report.Outcome:ProfileSolveOutcome.NumericFallback,
            FailureReason=agrees?report.FailureReason:"fixed_substitution_original_coordinate_certificate_failed",
            OptimalityGap=Math.Max(report.OptimalityGap,gap),ProjectedStep=Math.Max(report.ProjectedStep,step),
            CheckSweeps=report.CheckSweeps+1,OptimalityChecks=report.OptimalityChecks+1,Projections=report.Projections+1});
    }

    // Fully fixed stars have no remaining freedom. Substitute their exact
    // values before connectivity partitioning; a frozen obstacle must not
    // connect otherwise independent free islands through optimizer variables.
    // Partially fixed stars retain the original projector without approximation.
    public static Reduction? EliminateFixed(SparseProfileRows rows,double[] prior,ProfileStar[] stars,double scale,ImportLog log,CancellationToken token)
    {
        if(rows.Variables!=prior.Length)throw new ArgumentException("Fixed substitution vector size mismatch.");
        // Validate ownership before any star disappears from the derived set.
        _=new ProfileHardProjector(prior.Length,scale,stars);
        var fixedValues=new double?[prior.Length];var removed=new HashSet<int>();var eliminated=0;
        foreach(var star in stars)
        {
            if(!star.FixedHeight.HasValue||star.FixedSlopes.Any(v=>!v.HasValue))continue;
            var h=star.FixedHeight.Value;if(!SparseProfileRows.Finite(h)||h<0||h>scale)return null;
            fixedValues[star.Height]=h;eliminated++;removed.Add(star.Height);
            for(var j=0;j<star.Controls.Length;j++)
            {
                var value=h+star.Alpha[j]*star.FixedSlopes[star.Groups[j]]!.Value;
                if(!SparseProfileRows.Finite(value)||value<0||value>scale)return null;
                fixedValues[star.Controls[j]]=value;eliminated++;
            }
        }
        if(eliminated==0)return null;
        var free=Enumerable.Range(0,prior.Length).Where(i=>!fixedValues[i].HasValue).ToArray();
        // Keep the established solver's all-fixed path rather than inventing
        // a new termination report for an empty optimizer problem.
        if(free.Length==0)return null;
        var map=Enumerable.Repeat(-1,prior.Length).ToArray();for(var i=0;i<free.Length;i++)map[free[i]]=i;
        var bindings=Enumerable.Range(0,prior.Length).Select(i=>fixedValues[i].HasValue?
            new Binding(Array.Empty<int>(),Array.Empty<double>(),fixedValues[i]!.Value):new Binding(new[]{map[i]},new[]{1d})).ToArray();
        var retained=stars.Where(s=>!removed.Contains(s.Height)).Select(s=>new ProfileStar(map[s.Height],s.Controls.Select(v=>map[v]).ToArray(),s.Alpha,s.Groups,s.FixedHeight,s.FixedSlopes)).ToArray();
        var reduced=new SparseProfileRows(free.Length,coalesceInterleaved:true);var indices=new int[8];var coefficients=new double[8];
        for(long r=0;r<rows.Count;r++)
        {
            if((r&1023)==0)token.ThrowIfCancellationRequested();var row=rows.Row(r);var count=0;var target=row.Target;
            for(var j=0;j<row.Count;j++)
            {
                var id=rows.Index(row.Offset+j);var coefficient=rows.Coefficient(row.Offset+j);
                if(fixedValues[id].HasValue)target-=coefficient*fixedValues[id]!.Value;
                else {indices[count]=map[id];coefficients[count++]=coefficient;}
            }
            reduced.Append(indices,coefficients,count,target,row.Weight,row.Kind);
        }
        reduced.Freeze();log.Count("Profile immutable variables substituted",eliminated);
        log.Diagnostic($"PROFILE FIXED SUBSTITUTION originalVariables={prior.Length}; freeVariables={free.Length}; immutableVariables={eliminated}; originalRows={rows.Count}; derivedRows={reduced.Count}; constantObjective={reduced.ConstantObjective:R}; fullPhysicalContextRetained=true; frozenObstaclesDoNotCoupleFreeComponents=true; partialStarsUnchanged=true");
        return new(reduced,free.Select(i=>prior[i]).ToArray(),retained,bindings){Encoding="fixed-node-star-substitution-v1"};
    }

    public static Reduction? Reduce(IReadOnlyList<PreparedSegment> source,int[] from,int[] first,int[] second,int[] to,
        SparseProfileRows rows,double[] prior,ProfileStar[] stars,double scale,ImportLog log,CancellationToken token,IReadOnlyDictionary<string,NativeAssetContract>? assets=null)
    {
        // Bounded scalar evidence distinguishes actual refusal populations
        // from an assumed opportunity to enlarge the feasible parent family.
        var reasons=new[]{"Parent cuts: paired maximum length","Parent cuts: paired station discontinuity",
            "Parent cuts: paired semantic fixed or external-control boundary","Parent refusals: paired parameter metric",
            "Parent cuts: degenerate or nonfinite parameter metric","Parent cuts: maximum parent length",
            "Parent cuts: incompatible parameter metric","Parent refusals: shared control ownership",
            "Parent refusals: fixed internal observation","Parent refusals: internal grade representation",
            "Parent refusals: endpoint star representation","Parent profile hard-boundary reductions unavailable",
            "Parent profile projection ownership unavailable","Parent profile row arity unavailable"};
        var before=reasons.Select(log.Metric).ToArray();
        try{return ReduceCore(source,from,first,second,to,rows,prior,stars,scale,log,token,assets);}
        finally
        {
            var bindings=source.Count(s=>s.HorizontalParent!=null);
            var orphanIntervals=source.Count(s=>s.HorizontalParent==null&&(s.ParentStartT!=0||s.ParentEndT!=1));
            log.Diagnostic($"PROFILE PARENT CENSUS sections={source.Count}; retainedHorizontalBindings={bindings}; intervalMetadataWithoutParent={orphanIntervals}; pairedSections={source.Count(s=>s.PairedSpanId!=0)}; lostParentIdentityNotInferred=true; refusalCountsAreAttempts=true; refusalCounters="+
                string.Join("|",reasons.Select((reason,i)=>reason+":"+(log.Metric(reason)-before[i]))));
        }
    }

    private static Reduction? ReduceCore(IReadOnlyList<PreparedSegment> source,int[] from,int[] first,int[] second,int[] to,
        SparseProfileRows rows,double[] prior,ProfileStar[] stars,double scale,ImportLog log,CancellationToken token,IReadOnlyDictionary<string,NativeAssetContract>? assets)
    {
        var parents=new List<Parent>();
        var nodeStars=stars.ToDictionary(s=>s.Height);
        log.Diagnostic($"PROFILE PARENT INPUT stars={stars.Length}; fixedHeightStars={stars.Count(s=>s.FixedHeight.HasValue)}; fixedSlopeStars={stars.Count(s=>s.FixedSlopes.Any(v=>v.HasValue))}; independentGradeGroupStars={stars.Count(s=>s.Groups.Distinct().Count()>1)}; emptyStars={stars.Count(s=>s.Controls.Length==0)}; nativePortsNotAutomaticallyProfileKnots=true");
        log.Count("Parent input: fixed height stars",stars.Count(s=>s.FixedHeight.HasValue));
        log.Count("Parent input: fixed slope stars",stars.Count(s=>s.FixedSlopes.Any(v=>v.HasValue)));
        log.Count("Parent input: independent grade group stars",stars.Count(s=>s.Groups.Distinct().Count()>1));
        bool FreeInterior(int node)
        {
            if(!nodeStars.TryGetValue(node,out var star)||star.FixedHeight.HasValue||star.Groups.Length==0)return false;
            foreach(var slope in star.FixedSlopes)if(slope.HasValue)return false;
            foreach(var group in star.Groups)if(group!=star.Groups[0])return false;
            return true;
        }
        double Reach(int node,int control)
        {if(!nodeStars.TryGetValue(node,out var star))return 0;var k=Array.IndexOf(star.Controls,control);return k<0?0:Math.Abs(star.Alpha[k]);}
        foreach(var corridor in Enumerable.Range(0,source.Count).Where(i=>source[i].PairedCorridorId!=0).GroupBy(i=>source[i].PairedCorridorId))
        {
            token.ThrowIfCancellationRequested();
            var spans=corridor.GroupBy(i=>source[i].PairedSpanId).Select(g=>g.ToArray()).ToArray();
            if(spans.Any(g=>g.Length!=2||source[g[0]].PairedReversed==source[g[1]].PairedReversed))continue;
            var ordered=spans.OrderBy(g=>Math.Min(source[g[0]].PairedReferenceBegin,source[g[0]].PairedReferenceEnd)).ToArray();
            var start=0;
            while(start<ordered.Length)
            {
                var begin=Math.Min(source[ordered[start][0]].PairedReferenceBegin,source[ordered[start][0]].PairedReferenceEnd);
                var finish=start;
                while(finish+1<ordered.Length)
                {
                    var a=source[ordered[finish][0]];var next=ordered[finish+1];
                    var end=Math.Max(a.PairedReferenceBegin,a.PairedReferenceEnd);
                    var nextEnd=Math.Max(source[next[0]].PairedReferenceBegin,source[next[0]].PairedReferenceEnd);
                    if(nextEnd-begin>MaximumSpan(a)){log.Count("Parent cuts: paired maximum length");break;}
                    if(Math.Abs(end-Math.Min(source[next[0]].PairedReferenceBegin,source[next[0]].PairedReferenceEnd))>1e-7)
                    {log.Count("Parent cuts: paired station discontinuity");break;}
                    bool Mode(bool reversed)
                    {
                        var p=source[ordered[finish].First(i=>source[i].PairedReversed==reversed)];
                        var q=source[next.First(i=>source[i].PairedReversed==reversed)];
                        var pIndex=ordered[finish].First(i=>source[i].PairedReversed==reversed);
                        var qIndex=next.First(i=>source[i].PairedReversed==reversed);
                        var pStart=p.PairedReferenceBegin>p.PairedReferenceEnd;
                        var qStart=q.PairedReferenceBegin<q.PairedReferenceEnd;
                        var node=pStart?from[pIndex]:to[pIndex];
                        if(node!=(qStart?from[qIndex]:to[qIndex])||!FreeInterior(node)||
                            NetworkBoundaryPolicy.RequiresProfileBoundary(NetworkBoundaryPolicy.Classify(p,pStart,q,qStart,assets)))return false;
                        foreach(var control in nodeStars[node].Controls)
                        {
                            var owned=false;
                            foreach(var i in ordered[finish])if(first[i]==control||second[i]==control)owned=true;
                            foreach(var i in next)if(first[i]==control||second[i]==control)owned=true;
                            if(!owned)return false;
                        }
                        return true;
                    }
                    if(!Mode(false)||!Mode(true)){log.Count("Parent cuts: paired semantic fixed or external-control boundary");break;}finish++;
                }
                var last=source[ordered[finish][0]];var endStation=Math.Max(last.PairedReferenceBegin,last.PairedReferenceEnd);
                if(finish>start&&endStation-begin>1e-6)
                {
                    var members=ordered.Skip(start).Take(finish-start+1).ToArray();
                    var coordinates=new Dictionary<int,(double Begin,double End)>();var position=0d;var previousStep=0d;var previousReach=0d;var valid=true;
                    foreach(var span in members)
                    {
                        var forward=span.First(i=>source[i].PairedReferenceBegin<source[i].PairedReferenceEnd);
                        var original=source[forward].PairedReferenceEnd-source[forward].PairedReferenceBegin;
                        var a=Reach(from[forward],first[forward]);var b=Reach(to[forward],second[forward]);
                        var step=position==0?original:previousStep*a/previousReach;
                        if(!(a>1e-8&&b>1e-8&&SparseProfileRows.Finite(step))||step<original*.25||step>original*4)
                        {log.Count("Parent refusals: paired parameter metric");valid=false;break;}
                        foreach(var i in span)coordinates[i]=source[i].PairedReferenceBegin<source[i].PairedReferenceEnd?(position,position+step):(position+step,position);
                        position+=step;previousStep=step;previousReach=b;
                    }
                    if(valid)parents.Add(new(members.SelectMany(g=>g).ToArray(),0,position,coordinates));
                }
                start=finish+1;
            }
        }
        // Ordinary degree-two chains use the existing physical G1 metric to
        // assign a common parent parameter. At a join, dt/handleReach is equal
        // on both sides. Exact Y restriction consequently preserves physical
        // grade even when the two horizontal cubics have different speeds.
        var incidence=new Dictionary<int,List<int>>();
        for(var i=0;i<source.Count;i++)foreach(var node in new[]{from[i],to[i]})
        {if(!incidence.TryGetValue(node,out var edges))incidence[node]=edges=new();edges.Add(i);}
        var sourcePorts=0;var shareablePorts=0;var designBoundaries=0;var bookkeepingCandidates=0;
        foreach(var pair in incidence)
        {
            if(pair.Value.Count!=2)continue;
            var ai=pair.Value[0];var bi=pair.Value[1];
            var kind=NetworkBoundaryPolicy.Classify(source[ai],from[ai]==pair.Key,source[bi],from[bi]==pair.Key,assets);
            if(source[ai].Lanes!=source[bi].Lanes&&NetworkBoundaryPolicy.SameCapturedComposition(source[ai],source[bi],assets))
                log.Count("Profile nominal lane boundaries with identical captured composition");
            var port=(kind&NetworkBoundaryKind.SourcePort)!=0;if(port)sourcePorts++;
            var boundary=NetworkBoundaryPolicy.RequiresProfileBoundary(kind)||!FreeInterior(pair.Key)||
                source[ai].OneWay&&(from[ai]==pair.Key)==(from[bi]==pair.Key);
            if(boundary)designBoundaries++;else {bookkeepingCandidates++;if(port)shareablePorts++;}
        }
        log.Diagnostic($"PROFILE BOUNDARY CLASSIFICATION degreeTwoSourcePorts={sourcePorts}; shareableSourcePorts={shareablePorts}; designBoundaries={designBoundaries}; bookkeepingCandidates={bookkeepingCandidates}; countsAreCandidates=true; nativeNodesRemoved=false");
        var visited=new HashSet<int>();
        bool Eligible(int i)=>source[i].PairedCorridorId==0&&source[i].PairedSpanId==0;
        int Next(int edge,int node)
        {
            if(!incidence.TryGetValue(node,out var entries)||entries.Count!=2||!nodeStars.TryGetValue(node,out var star)||
                star.Controls.Length!=2||!FreeInterior(node))return -1;
            var next=entries[0]==edge?entries[1]:entries[0];var a=source[edge];var b=source[next];
            if(!Eligible(next)||NetworkBoundaryPolicy.RequiresProfileBoundary(NetworkBoundaryPolicy.Classify(a,from[edge]==node,b,from[next]==node,assets)))return -1;
            if(a.OneWay&&(from[edge]==node)==(from[next]==node))return -1;
            return next;
        }
        foreach(var seedIndex in Enumerable.Range(0,source.Count))
        {
            token.ThrowIfCancellationRequested();if(!Eligible(seedIndex)||visited.Contains(seedIndex))continue;
            var firstEdge=seedIndex;var firstNode=from[firstEdge];var back=new HashSet<int>{firstEdge};
            while(true)
            {
                var next=Next(firstEdge,firstNode);if(next<0||visited.Contains(next)||!back.Add(next))break;
                firstNode=from[next]==firstNode?to[next]:from[next];firstEdge=next;
            }
            var route=new List<(int Index,bool Forward)>();var at=firstNode;var current=firstEdge;
            while(visited.Add(current))
            {
                var forward=from[current]==at;route.Add((current,forward));at=forward?to[current]:from[current];
                var next=Next(current,at);if(next<0||visited.Contains(next))break;current=next;
            }
            var begin=0;
            while(begin<route.Count)
            {
                var members=new List<int>();var stations=new Dictionary<int,(double Begin,double End)>();var position=0d;var arc=0d;
                var previousReach=0d;var previousStep=0d;var end=begin;
                var window=MaximumSpan(source[route[begin].Index]);
                while(end<route.Count)
                {
                    var item=route[end];var s=source[item.Index];
                    var length=NetworkVerticalProfiles.ArcLength(s);
                    var startReach=math.length((double2)(item.Forward?s.CourseControl1-s.CourseStart:s.CourseControl2-s.CourseEnd).xz);
                    var endReach=math.length((double2)(item.Forward?s.CourseEnd-s.CourseControl2:s.CourseStart-s.CourseControl1).xz);
                    var step=end==begin?length:previousStep*startReach/previousReach;
                    if(!(length>1e-8&&startReach>1e-8&&endReach>1e-8&&SparseProfileRows.Finite(step)))
                    {log.Count("Parent cuts: degenerate or nonfinite parameter metric");break;}
                    if(end>begin&&arc+length>window){log.Count("Parent cuts: maximum parent length");break;}
                    if(end>begin&&(step<length*.25||step>length*4)){log.Count("Parent cuts: incompatible parameter metric");break;}
                    members.Add(item.Index);stations[item.Index]=item.Forward?(position,position+step):(position+step,position);
                    position+=step;arc+=length;previousReach=endReach;previousStep=step;end++;
                }
                if(members.Count>1)parents.Add(new(members.ToArray(),0,position,stations));
                begin=Math.Max(begin+1,end);
            }
        }
        if(parents.Count==0)return null;
        var disabled=new HashSet<int>();
        // A genuine boundary that cannot be represented by a disjoint parent
        // endpoint star retains its original variables, not an approximate hard
        // constraint. At most one retry; this is not a geometry repair loop.
        for(var attempt=0;attempt<2;attempt++)
        {
            var bindings=Enumerable.Range(0,prior.Length).Select(i=>new Binding(new[]{i},new[]{1d})).ToArray();
            var assigned=new bool[prior.Length];var owners=Enumerable.Repeat(-1,prior.Length).ToArray();var seed=prior.ToList();var invalid=new HashSet<int>();
            for(var p=0;p<parents.Count;p++)
            {
                if(disabled.Contains(p))continue;var parent=parents[p];
                (double Begin,double End) Station(int i)=>parent.Stations!=null?parent.Stations[i]:(source[i].PairedReferenceBegin,source[i].PairedReferenceEnd);
                var forward=parent.Sections.Where(i=>Station(i).Begin<Station(i).End).OrderBy(i=>Station(i).Begin).ToArray();
                var ordered=parent.Sections.OrderBy(i=>Math.Min(Station(i).Begin,Station(i).End)).ToArray();
                var firstIndex=forward.Length>0&&parent.Stations==null?forward[0]:ordered[0];
                var lastIndex=forward.Length>0&&parent.Stations==null?forward[forward.Length-1]:ordered[ordered.Length-1];
                var a=Station(firstIndex).Begin<Station(firstIndex).End?from[firstIndex]:to[firstIndex];
                var b=Station(lastIndex).Begin<Station(lastIndex).End?to[lastIndex]:from[lastIndex];
                var c=seed.Count;seed.Add((2*prior[a]+prior[b])/3);var d=seed.Count;seed.Add((prior[a]+2*prior[b])/3);
                var variables=new[]{a,c,d,b};
                foreach(var i in parent.Sections)
                {
                    var station=Station(i);var lo=math.clamp((station.Begin-parent.Begin)/(parent.End-parent.Begin),0,1);
                    var hi=math.clamp((station.End-parent.Begin)/(parent.End-parent.Begin),0,1);
                    var local=new[]{from[i],first[i],second[i],to[i]};
                    for(var control=0;control<4;control++)
                    {
                        var coefficients=new double[4];
                        NetworkFootprintMath.Blossom(control<3?lo:hi,control<2?lo:hi,control<1?lo:hi,coefficients);
                        var terms=Enumerable.Range(0,4).Where(k=>coefficients[k]!=0).ToArray();
                        var binding=new Binding(terms.Select(k=>variables[k]).ToArray(),terms.Select(k=>coefficients[k]).ToArray());
                        var id=local[control];
                        if(assigned[id]&&!Same(bindings[id],binding)){log.Count("Parent refusals: shared control ownership");invalid.Add(p);if(owners[id]>=0)invalid.Add(owners[id]);}
                        else {bindings[id]=binding;assigned[id]=true;owners[id]=p;}
                    }
                }
            }
            var transformed=new List<ProfileStar>();
            foreach(var star in stars)
            {
                var height=bindings[star.Height];
                if(height.Variables.Length!=1||height.Coefficients[0]!=1)
                {
                    if(star.FixedHeight.HasValue||star.FixedSlopes.Any(s=>s.HasValue)){log.Count("Parent refusals: fixed internal observation");Reject(star);continue;}
                    // Internal native knots already satisfy each G1 group
                    // identically under exact restriction of the same parent.
                    var slopes=new Dictionary<int,Binding>();var valid=true;
                    for(var k=0;k<star.Controls.Length;k++)
                    {
                        var slope=Difference(bindings[star.Controls[k]],height,star.Alpha[k]);
                        if(slopes.TryGetValue(star.Groups[k],out var old)&&!Same(old,slope))valid=false;else slopes[star.Groups[k]]=slope;
                    }
                    if(!valid){log.Count("Parent refusals: internal grade representation");Reject(star);}continue;
                }
                var controls=new List<int>();var alpha=new List<double>();var groups=new List<int>();var validStar=true;
                for(var k=0;k<star.Controls.Length;k++)
                {
                    var binding=bindings[star.Controls[k]];var terms=Enumerable.Range(0,binding.Variables.Length).Where(j=>binding.Variables[j]!=height.Variables[0]).ToArray();
                    if(terms.Length!=1){validStar=false;break;}
                    var term=terms[0];var coefficient=binding.Coefficients[term];
                    var h=Enumerable.Range(0,binding.Variables.Length).Where(j=>binding.Variables[j]==height.Variables[0]).Sum(j=>binding.Coefficients[j]);
                    if(!(coefficient>0)||Math.Abs(coefficient+h-1)>1e-10){validStar=false;break;}
                    var id=binding.Variables[term];var reach=star.Alpha[k]/coefficient;var duplicate=controls.IndexOf(id);
                    if(duplicate>=0)
                    {if(groups[duplicate]!=star.Groups[k]||Math.Abs(alpha[duplicate]-reach)>1e-8*Math.Max(1,Math.Abs(reach)))validStar=false;}
                    else {controls.Add(id);alpha.Add(reach);groups.Add(star.Groups[k]);}
                }
                if(!validStar){log.Count("Parent refusals: endpoint star representation");Reject(star);continue;}
                transformed.Add(new(height.Variables[0],controls.ToArray(),alpha.ToArray(),groups.ToArray(),star.FixedHeight,star.FixedSlopes));
            }
            void Reject(ProfileStar star)
            {foreach(var variable in star.Controls.Concat(new[]{star.Height}))if(owners[variable]>=0)invalid.Add(owners[variable]);}
            if(invalid.Count>0)
            {disabled.UnionWith(invalid);if(attempt==0)continue;log.Count("Parent profile hard-boundary reductions unavailable",invalid.Count);return null;}
            var used=new SortedSet<int>(bindings.SelectMany(b=>b.Variables));var dense=used.Select((v,i)=>(v,i)).ToDictionary(p=>p.v,p=>p.i);
            if(used.Count>=prior.Length)return null;
            var mapped=bindings.Select(b=>new Binding(b.Variables.Select(v=>dense[v]).ToArray(),b.Coefficients)).ToArray();
            var projected=transformed.Select(s=>new ProfileStar(dense[s.Height],s.Controls.Select(v=>dense[v]).ToArray(),s.Alpha,s.Groups,s.FixedHeight,s.FixedSlopes)).ToArray();
            try {new ProfileHardProjector(used.Count,scale,projected);}
            catch(ArgumentException){log.Count("Parent profile projection ownership unavailable");return null;}
            var reduced=new SparseProfileRows(used.Count,coalesceInterleaved:true);var ii=new int[8];var aa=new double[8];
            for(long r=0;r<rows.Count;r++)
            {
                if((r&1023)==0)token.ThrowIfCancellationRequested();var row=rows.Row(r);var count=0;
                for(var j=0;j<row.Count;j++)
                {
                    var binding=mapped[rows.Index(row.Offset+j)];var coefficient=rows.Coefficient(row.Offset+j);
                    for(var k=0;k<binding.Variables.Length;k++)
                    {
                        var id=binding.Variables[k];var slot=0;while(slot<count&&ii[slot]!=id)slot++;
                        if(slot==count){if(count==8){log.Count("Parent profile row arity unavailable");return null;}ii[count]=id;aa[count++]=0;}
                        aa[slot]+=coefficient*binding.Coefficients[k];
                    }
                }
                reduced.Append(ii,aa,count,row.Target,row.Weight,row.Kind);
            }
            reduced.Freeze();log.Diagnostic($"PROFILE PARENTS originalVariables={prior.Length}; parentVariables={used.Count}; functionalParents={parents.Count-disabled.Count}; maximumMainlineParentMeters=500; fixedAndSemanticBoundariesPreserved=true; emissionCutsAreNotUnknowns=true; exactObjectiveSubstitution=true; originalPhysicalRowsRetained=true");
            return new(reduced,used.Select(v=>seed[v]).ToArray(),projected,mapped);
        }
        return null;
    }
    private static Binding Difference(Binding a,Binding b,double divisor)
    {
        var terms=new SortedDictionary<int,double>();
        foreach(var item in new[]{(a,1d),(b,-1d)})for(var i=0;i<item.Item1.Variables.Length;i++)
        {var id=item.Item1.Variables[i];terms.TryGetValue(id,out var old);terms[id]=old+item.Item2*item.Item1.Coefficients[i]/divisor;}
        return new(terms.Keys.ToArray(),terms.Values.ToArray());
    }
    private static bool Same(Binding a,Binding b)
    {
        var delta=Difference(a,b,1);
        return delta.Coefficients.All(v=>Math.Abs(v)<=1e-12*Math.Max(1,a.Coefficients.Concat(b.Coefficients).Select(Math.Abs).DefaultIfEmpty().Max()));
    }
}
