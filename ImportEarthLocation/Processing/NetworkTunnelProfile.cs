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
    public sealed record PhysicalQuality(long ClearanceFailures,double MaximumClearanceResidual,long CoverFailures,
        double MaximumCoverResidual,int UncertifiedCover,int ExceptionalGradeFailures)
    {
        public long ClearanceRows { get; init; }
        public long CoverRows { get; init; }
        public int UncertifiedClearance { get; init; }
        public long HorizontalConflictPairs { get; init; }
        public double ClearanceAffectedLength { get; init; }
        public double CoverAffectedLength { get; init; }
    }
    public static PhysicalQuality MeasurePhysical(IReadOnlyList<PreparedSegment> source,ushort[] terrain,float scale,double width,
        IReadOnlyDictionary<string,NativeAssetContract>? assets,CancellationToken token,NetworkPhysicalContacts.GeometryCache? geometry=null)
    {
        var from=new int[source.Count];var c1=new int[source.Count];
        var c2=new int[source.Count];var to=new int[source.Count];var heights=new double[source.Count*4];
        for(var i=0;i<source.Count;i++)
        {
            var s=source[i];from[i]=i*4;c1[i]=i*4+1;c2[i]=i*4+2;to[i]=i*4+3;
            heights[i*4]=s.CourseStart.y;heights[i*4+1]=s.CourseControl1.y;heights[i*4+2]=s.CourseControl2.y;heights[i*4+3]=s.CourseEnd.y;
        }
        var epsilon=Math.Max(.05,2*scale/65535d);long contacts=0;var uncertified=0;var uncertifiedClearance=0;var horizontalConflicts=0L;
        long clearanceFailures=0,coverFailures=0;var clearance=0d;var cover=0d;var coverPhase=false;
        var failedClearanceSections=new HashSet<int>();var failedCoverSections=new HashSet<int>();
        var rows=SparseProfileRows.ForMeasurement(checked(source.Count*4),(indices,coefficients,count,target)=>
        {
            token.ThrowIfCancellationRequested();var residual=-target;
            for(var j=0;j<count;j++)residual+=coefficients[j]*heights[indices[j]];
            residual*=epsilon;
            if(residual<=(coverPhase?epsilon:.05))return;
            if(coverPhase){coverFailures++;cover=Math.Max(cover,residual);}else{clearanceFailures++;clearance=Math.Max(clearance,residual);}
            var affected=coverPhase?failedCoverSections:failedClearanceSections;
            for(var j=0;j<count;j++)affected.Add(indices[j]/4);
        });
        if(assets!=null)
        {
            var contactReport=NetworkPhysicalContacts.Append(source,assets,from,c1,c2,to,rows,epsilon,token,geometry:geometry);
            contacts=contactReport.Rows;uncertifiedClearance=contactReport.UncertifiedLeaves+contactReport.MissingAssets;horizontalConflicts=contactReport.HorizontalConflictPairs;
            coverPhase=true;
            var tunnelReport=NetworkTunnelContacts.Append(source,terrain,scale,width,assets,from,c1,c2,to,rows,epsilon,token);
            uncertified=tunnelReport.Uncertified+tunnelReport.MissingAssets;
        }
        else {uncertifiedClearance=source.Count;uncertified=source.Count(s=>s.Tunnel);}
        var grades=source.Count(s=>!NetworkVerticalProfiles.WithinGradeCeiling(s,NetworkGeometrySafety.GradeLimit(s)));
        return new(clearanceFailures,clearance,coverFailures,cover,uncertified,grades)
            {ClearanceRows=contacts,CoverRows=rows.Count-contacts,UncertifiedClearance=uncertifiedClearance,HorizontalConflictPairs=horizontalConflicts,
                ClearanceAffectedLength=failedClearanceSections.Sum(i=>NetworkCurveObservations.Measure(new(source[i].CourseStart,source[i].CourseControl1,source[i].CourseControl2,source[i].CourseEnd)).SampledLengthMeters),
                CoverAffectedLength=failedCoverSections.Sum(i=>NetworkCurveObservations.Measure(new(source[i].CourseStart,source[i].CourseControl1,source[i].CourseControl2,source[i].CourseEnd)).SampledLengthMeters)};
    }
    public sealed record TerrainReference(ushort[] Heights,double Width,Func<float3,float?>? Water=null,
        IReadOnlyDictionary<string,NativeAssetContract>? Assets=null,Action<string>? Progress=null,bool CaptureReplay=true,string EvidenceScope="local")
    {public NetworkPhysicalContacts.GeometryCache? ContactGeometry { get; init; }}
    public static IReadOnlyList<PreparedSegment> Solve(IReadOnlyList<PreparedSegment> source,float scale,
        IReadOnlyList<(int index,float t,float ceiling)> cover,ImportLog log,CancellationToken token,
        IReadOnlyDictionary<SourceNodeKey,double>? verifiedBoundaryHeights=null,TerrainReference? design=null,
        IReadOnlyDictionary<SourceNodeKey,double>? verifiedBoundarySlopes=null,ISet<int>? frozenSections=null,
        Action<PhysicalQuality>? physicalQuality=null)
    {
        if(source.Count==0||cover.Count==0&&design==null)return source;
        if(design!=null)
        {
            var witnesses=new List<object>();var feasibility=NetworkGroundFeasibility.Check(source,design.Heights,scale,design.Width,token,
                log.DiagnosticsEnabled?value=>{if(witnesses.Count<64)witnesses.Add(value);}:null);
            var scope=design.CaptureReplay?"initial-design":design.EvidenceScope+"-unit-"+source[0].NativeUnitId;
            log.Quality(new("planned","ground-grade-budget-feasibility",scope,feasibility.Checked,feasibility.Conflicts,feasibility.Unavailable,
                feasibility.MaximumDeficitMeters,0,"Necessary fixed-XZ ground endpoint condition; failed means grade/support budgets are incompatible; passed is not a full feasibility certificate"));
            if(log.DiagnosticsEnabled&&witnesses.Count>0)log.DiagnosticRecords("ground-feasibility-"+scope+".ndjson",witnesses,critical:true);
        }
        float Ground(float3 point)=>NetworkProcessor.Sample(point.x,point.z,design!.Heights,scale,design.Width);
        var ordered=Enumerable.Range(0,source.Count).OrderBy(i=>source[i].Kind).ThenBy(i=>source[i].StartNodeId)
            .ThenBy(i=>source[i].EndNodeId).ThenBy(i=>source[i].WayId).ToArray();
        var ids=new Dictionary<SourceNodeKey,int>();var keys=new Dictionary<int,SourceNodeKey>();var prior=new List<double>();
        var heightFamilies=new Dictionary<long,int>();var controlFamilies=new Dictionary<long,(int First,int Second)>();
        var referencePorts=new Dictionary<SourceNodeKey,(double Speed,bool Reverse)>();
        var nodeFamilies=new Dictionary<SourceNodeKey,long>();
        var sharedNodes=new HashSet<int>();var priorCount=new List<int>();
        foreach(var s in source)if(s.PairedSpanId!=0)
        {
            referencePorts[new(s.Kind,s.StartNodeId)]=(s.StartReferenceSpeed,s.PairedReversed);referencePorts[new(s.Kind,s.EndNodeId)]=(s.EndReferenceSpeed,s.PairedReversed);
            nodeFamilies[new(s.Kind,s.StartNodeId)]=s.StartHeightFamily;nodeFamilies[new(s.Kind,s.EndNodeId)]=s.EndHeightFamily;
        }
        var heightOnlyKeys=new HashSet<SourceNodeKey>();var familyParents=new Dictionary<long,long>();var nextCrossingFamily=long.MinValue;
        var usedHeightFamilies=new HashSet<long>(nodeFamilies.Values);
        long FamilyRoot(long value)
        {while(familyParents.TryGetValue(value,out var parent)&&parent!=value)value=parent;return value;}
        var crossingPorts=source.SelectMany(s=>new[]{(Section:s,Node:s.StartNodeId,Point:s.CourseStart),(Section:s,Node:s.EndNodeId,Point:s.CourseEnd)})
            .GroupBy(p=>p.Node);
        foreach(var group in crossingPorts)
        {
            var crossing=group.ToArray();if(!crossing.Any(p=>p.Section.Kind==NetworkKind.Road)||!crossing.Any(p=>p.Section.Kind==NetworkKind.Rail))continue;
            var first=crossing[0];
            if(crossing.Any(p=>NetworkStructure.IsStructure(p.Section)||p.Section.Layer!=first.Section.Layer||!math.all(p.Point.xz==first.Point.xz)))continue;
            var keysAtCrossing=crossing.Select(p=>new SourceNodeKey(p.Section.Kind,p.Node)).Distinct().ToArray();
            if(verifiedBoundaryHeights!=null&&keysAtCrossing.Where(verifiedBoundaryHeights.ContainsKey).Select(k=>verifiedBoundaryHeights[k]).Distinct().Count()>1)
            {
                // These are already committed ports. Retain their separate
                // heights rather than recursively trying the same impossible merge.
                log.Count("Conflicting fixed ground road rail crossing heights");
                log.Diagnostic($"PROFILE GROUND CROSSING node={group.Key}; incompatibleFixedHeights=true; quality=RequiresReview; committedBoundariesPreserved=true");
                continue;
            }
            var families=keysAtCrossing.Where(nodeFamilies.ContainsKey).Select(k=>FamilyRoot(nodeFamilies[k])).Where(f=>f!=0).Distinct().ToArray();
            while(usedHeightFamilies.Contains(nextCrossingFamily))nextCrossingFamily++;
            var family=families.Length>0?families.Min():nextCrossingFamily++;
            usedHeightFamilies.Add(family);
            foreach(var existing in families)familyParents[existing]=family;
            foreach(var key in keysAtCrossing){nodeFamilies[key]=family;heightOnlyKeys.Add(key);}
        }
        foreach(var key in nodeFamilies.Keys.ToArray())nodeFamilies[key]=FamilyRoot(nodeFamilies[key]);
        if(verifiedBoundaryHeights!=null&&nodeFamilies.GroupBy(p=>p.Value).Any(g=>g.Where(p=>verifiedBoundaryHeights.ContainsKey(p.Key)).Select(p=>verifiedBoundaryHeights[p.Key]).Distinct().Count()>1))
        {
            log.Diagnostic("PROFILE SHARED BOUNDARY RequiresReview; incompatible committed family heights; independent profiles retain every fixed native boundary.");
            var independent=source.Select(s=>s with{PairedSpanId=0,PairedCorridorId=0,StartHeightFamily=0,EndHeightFamily=0,ReferenceSpan=0}).ToArray();
            return Solve(independent,scale,cover,log,token,verifiedBoundaryHeights,design,verifiedBoundarySlopes,frozenSections,physicalQuality);
        }
        var from=new int[source.Count];var to=new int[source.Count];var c1=new int[source.Count];var c2=new int[source.Count];
        int Node(PreparedSegment s,long id,float height)
        {
            var key=new SourceNodeKey(s.Kind,id);if(ids.TryGetValue(key,out var v))return v;
            nodeFamilies.TryGetValue(key,out var family);
            if(family!=0&&heightFamilies.TryGetValue(family,out v))
            {ids.Add(key,v);prior[v]=(prior[v]*priorCount[v]+math.clamp((double)height,0,scale))/(++priorCount[v]);sharedNodes.Add(v);return v;}
            v=prior.Count;ids.Add(key,v);keys.Add(v,key);prior.Add(math.clamp((double)height,0,scale));priorCount.Add(1);
            if(family!=0){heightFamilies[family]=v;sharedNodes.Add(v);}return v;
        }
        foreach(var i in ordered)
        {
            token.ThrowIfCancellationRequested();var s=source[i];
            from[i]=Node(s,s.StartNodeId,s.CourseStart.y);to[i]=Node(s,s.EndNodeId,s.CourseEnd.y);
            if(s.PairedSpanId!=0&&controlFamilies.TryGetValue(s.PairedSpanId,out var family))
            {
                c1[i]=s.PairedReversed?family.Second:family.First;c2[i]=s.PairedReversed?family.First:family.Second;
                foreach(var entry in new[]{(c1[i],s.CourseControl1.y),(c2[i],s.CourseControl2.y)})
                    prior[entry.Item1]=(prior[entry.Item1]*priorCount[entry.Item1]+math.clamp((double)entry.Item2,0,scale))/(++priorCount[entry.Item1]);
            }
            else
            {
                int Control(float3 point,float3 endpoint,int node)
                {
                    if(s.PairedSpanId==0&&math.lengthsq((point-endpoint).xz)==0)
                    {log.Count("Stationary profile controls share endpoint height");return node;}
                    var v=prior.Count;prior.Add(math.clamp((double)point.y,0,scale));priorCount.Add(1);return v;
                }
                c1[i]=Control(s.CourseControl1,s.CourseStart,from[i]);
                c2[i]=Control(s.CourseControl2,s.CourseEnd,to[i]);
                if(s.PairedSpanId!=0)controlFamilies[s.PairedSpanId]=s.PairedReversed?(c2[i],c1[i]):(c1[i],c2[i]);
            }
        }
        var heightOnlyVariables=new HashSet<int>(heightOnlyKeys.Where(ids.ContainsKey).Select(k=>ids[k]));
        var frozen=new Dictionary<int,double>();
        if(frozenSections!=null)foreach(var i in frozenSections)
        {
            var s=source[i];
            foreach(var value in new[]{(from[i],(double)s.CourseStart.y),(c1[i],(double)s.CourseControl1.y),
                (c2[i],(double)s.CourseControl2.y),(to[i],(double)s.CourseEnd.y)})
            {
                if(frozen.TryGetValue(value.Item1,out var existing)&&Math.Abs(existing-value.Item2)>1e-6)
                    throw new InvalidOperationException("Local reconstruction has incompatible frozen shared controls; original geometry retained.");
                frozen[value.Item1]=value.Item2;
            }
        }
        var ports=new Dictionary<int,List<(int Variable,double Reach,double2 Direction)>>();
        void Port(int node,int variable,double2 direction,bool start,PreparedSegment s)
        {
            var reach=math.length(direction);
            // A finite stationary handle has no longitudinal derivative model.
            // A stationary unpaired control already shares its endpoint height.
            // Do not invent a longitudinal reach or a vertical-only tangent.
            if(!(reach>1e-8)){log.Count("Profile degenerate horizontal ports");return;}
            if(!ports.TryGetValue(node,out var list))ports[node]=list=new();
            if(list.Any(p=>p.Variable==variable))return;
            var alpha=start?reach:-reach;
            if(s.PairedSpanId!=0)alpha=(start?1:-1)*(s.PairedReversed?-1:1)*s.ReferenceSpan/3;
            else if(referencePorts.TryGetValue(new(s.Kind,start?s.StartNodeId:s.EndNodeId),out var reference)&&reference.Speed>1e-8)
                alpha=alpha/reference.Speed*(reference.Reverse?-1:1);
            list.Add((variable,alpha,direction/reach));
        }
        foreach(var i in ordered)
        {var s=source[i];Port(from[i],c1[i],(s.CourseControl1-s.CourseStart).xz,true,s);Port(to[i],c2[i],(s.CourseControl2-s.CourseEnd).xz,false,s);}
        var stars=new List<ProfileStar>();
        foreach(var pair in ports.OrderBy(p=>p.Key))
        {
            var p=pair.Value;var groups=new int[p.Count];var alpha=new double[p.Count];var controls=new int[p.Count];
            for(var j=0;j<p.Count;j++){controls[j]=p[j].Variable;alpha[j]=p[j].Reach;groups[j]=j;}
            // A through join has opposing OUTWARD tangents and coherent travel
            // orientation. Do not tie unrelated intersection branches together.
            if(heightOnlyVariables.Contains(pair.Key))
            {
                // Explicit road/rail source crossings share elevation, not a
                // generic tangent/grade axis across perpendicular networks.
                for(var j=0;j<p.Count;j++)for(var k=0;k<j;k++)if(Math.Abs(math.dot(p[j].Direction,p[k].Direction))>1-1e-5)
                {
                    groups[j]=groups[k];alpha[j]=Math.Sign(math.dot(p[j].Direction,p[k].Direction))*Math.Sign(alpha[k])*Math.Abs(alpha[j]);break;
                }
            }
            else if(sharedNodes.Contains(pair.Key))Array.Clear(groups,0,groups.Length);
            else if(p.Count==2&&math.dot(p[0].Direction,p[1].Direction)<-.9)
            {groups[1]=0;alpha[1]=-Math.Sign(alpha[0])*Math.Abs(alpha[1]);}
            else if(p.Count>2&&p.All(port=>Math.Abs(math.dot(port.Direction,p[0].Direction))>1-1e-5))
            {
                // A canonical collinear bundle/rail turnout has one physical
                // grade axis, not an independent slope on every branch.
                // Signs are outward in the same frozen horizontal frame.
                Array.Clear(groups,0,groups.Length);
                for(var j=0;j<p.Count;j++)alpha[j]=Math.Sign(math.dot(p[j].Direction,p[0].Direction))*Math.Abs(p[j].Reach);
                log.Count("Canonical multi-arm grade groups");
            }
            var denseGroups=new Dictionary<int,int>();
            for(var j=0;j<groups.Length;j++)
            {if(!denseGroups.TryGetValue(groups[j],out var dense))denseGroups[groups[j]]=dense=denseGroups.Count;groups[j]=dense;}
            double? fixedHeight=null;
            if(frozen.TryGetValue(pair.Key,out var frozenHeight))fixedHeight=frozenHeight;
            var fixedValues=verifiedBoundaryHeights==null?Array.Empty<double>():ids.Where(p=>p.Value==pair.Key&&verifiedBoundaryHeights.ContainsKey(p.Key)).Select(p=>verifiedBoundaryHeights[p.Key]).ToArray();
            if(fixedValues.Length>0)
            {
                var fixedValue=fixedValues[0];
                if(!SparseProfileRows.Finite(fixedValue)||fixedValue<0||fixedValue>scale)
                    throw new InvalidOperationException("Verified native boundary is not representable in the fixed map-height box.");
                if(fixedHeight.HasValue&&Math.Abs(fixedHeight.Value-fixedValue)>1e-6)
                    throw new InvalidOperationException("Verified boundary conflicts with the frozen neighbouring profile.");
                fixedHeight=fixedValue;
            }
            double?[]? fixedSlopes=null;
            if(verifiedBoundarySlopes!=null)
            {
                var values=ids.Where(k=>k.Value==pair.Key&&verifiedBoundarySlopes.ContainsKey(k.Key)).Select(k=>verifiedBoundarySlopes[k.Key]).ToArray();
                if(values.Length>0)
                {
                    if(groups.Distinct().Count()!=1||values.Any(v=>!SparseProfileRows.Finite(v)||Math.Abs(v-values[0])>1e-8))
                        throw new InvalidOperationException("Observed fixed boundary slopes do not define one representable native port.");
                    var key=ids.First(k=>k.Value==pair.Key&&verifiedBoundarySlopes.ContainsKey(k.Key)).Key;
                    var slope=values[0];if(referencePorts.TryGetValue(key,out var reference))slope*=reference.Speed*(reference.Reverse?-1:1);
                    fixedSlopes=new double?[]{slope};
                }
            }
            if(fixedHeight.HasValue)
            {
                if(fixedSlopes==null&&controls.All(frozen.ContainsKey))
                {
                    // Unaffected float context is an observation, not a new
                    // exact-G1 design request. Its controls are all immutable;
                    // tying their slightly different observed slopes would
                    // make an unrelated local reconstruction infeasible.
                    for(var j=0;j<groups.Length;j++)groups[j]=j;
                    log.Count("Fully frozen observed node stars retained independently");
                }
                fixedSlopes??=new double?[groups.Max()+1];
                for(var j=0;j<controls.Length;j++)if(frozen.TryGetValue(controls[j],out var value))
                {
                    var slope=(value-fixedHeight.Value)/alpha[j];var group=groups[j];
                    if(fixedSlopes[group].HasValue&&Math.Abs(fixedSlopes[group]!.Value-slope)>1e-6)
                        throw new InvalidOperationException("Frozen native/profile context conflicts with a shared grade group; local reconstruction unavailable.");
                    fixedSlopes[group]=slope;
                }
            }
            stars.Add(new ProfileStar(pair.Key,controls,alpha,groups,fixedHeight,fixedSlopes));
        }
        log.Diagnostic($"PROFILE LEVEL CROSSINGS sharedHeightVariables={heightOnlyVariables.Count}; exactSourceNodeAndGroundModeRequired=true; roadRailTopologyNotWelded=true; perpendicularGradeGroupsSeparate=true");
        var nodeVariables=new HashSet<int>(ids.Values);
        foreach(var pair in frozen)if(nodeVariables.Contains(pair.Key)&&!ports.ContainsKey(pair.Key))
            stars.Add(new ProfileStar(pair.Key,Array.Empty<int>(),Array.Empty<double>(),Array.Empty<int>(),pair.Value));
        if(verifiedBoundaryHeights!=null)foreach(var pair in ids)
            if(!ports.ContainsKey(pair.Value)&&!frozen.ContainsKey(pair.Value)&&verifiedBoundaryHeights.TryGetValue(pair.Key,out var fixedValue))
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
            if(s.Kind==NetworkKind.Road&&s.ExplicitIncline.HasValue)
            {
                // Parsing/reversal already converts the raw tag into this
                // section's orientation. Do not reverse it a second time.
                var incline=(double)s.ExplicitIncline.Value;
                var consistent=math.isfinite(incline)&&(s.SignedInclinePriorAccepted??Math.Abs(incline*length-(s.CourseEnd.y-s.CourseStart.y))<=3);
                if(consistent)
                {
                    for(var sample=0;sample<3;sample++)
                    {
                        var t=(sample+.5)/3;var u=1-t;
                        var speed=math.length(NetworkDesignChecks.Derivative(new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd),t).xz);
                        if(!(speed>1e-8)){log.Count("Signed incline observations unavailable");continue;}
                        coefficients[0]=-3*u*u/(speed*.005);coefficients[1]=(3*u*u-6*u*t)/(speed*.005);
                        coefficients[2]=(6*u*t-3*t*t)/(speed*.005);coefficients[3]=3*t*t/(speed*.005);
                        Row(4,incline/.005,100*support/3,ProfileRowKind.EqualityCost);
                    }
                    log.Count("Signed incline priors accepted");
                }
                else
                {
                    log.Count("Contradictory signed incline priors");
                    log.Diagnostic($"PROFILE INCLINE review way={s.WayId}; nodes={s.StartNodeId},{s.EndNodeId}; signedGrade={incline:R}; immutableArcMeters={length:R}; immutableHeightDifference={s.CourseEnd.y-s.CourseStart.y:R}; toleranceMeters=3; enforced=false; rawTagRetained=true");
                }
            }
            // Observations are virtual physical stations, not a reason to add
            // native nodes. The coordinated route already supplies a smoothed
            // height intent; immutable terrain still defines cut/fill bounds.
            var observationCount=design==null?8:Math.Min(64,Math.Max(2,(int)Math.Ceiling(length/40)));
            for(var sample=0;sample<=observationCount;sample++)
            {
                var t=sample/(double)observationCount;var u=1-t;
                coefficients[0]=u*u*u/budget;coefficients[1]=3*u*u*t/budget;
                coefficients[2]=3*u*t*t/budget;coefficients[3]=t*t*t/budget;
                var weight=support/observationCount*(sample==0||sample==observationCount?.5:1);
                Row(4,NetworkNormalizer.Evaluate(s,(float)t).y/budget,weight,ProfileRowKind.EqualityCost);
                if(design!=null)
                {
                    var p=NetworkNormalizer.Evaluate(s,(float)t);var ground=Ground(p);
                    // Cut/fill is terrain-relative only for ground roads. A
                    // bridge/tunnel has a structural alignment datum; pulling
                    // it toward every valley/crest sample recreates the very
                    // earthwork it was selected to avoid. Physical deck/roof
                    // clearance rows remain independent and unchanged.
                    var datum=NetworkStructure.IsStructure(s)?NetworkNormalizer.Evaluate(s,(float)t).y:ground;
                    var lower=datum-budget;var upper=datum+budget;
                    // Fixed terrain-relative observations: never increase these
                    // limits in response to another road's current iteration.
                    Row(4,upper/budget,weight,ProfileRowKind.UpperHingeCost);
                    for(var k=0;k<4;k++)coefficients[k]=-coefficients[k];
                    Row(4,-lower/budget,weight,ProfileRowKind.UpperHingeCost);
                    var level=design.Water?.Invoke(p);
                    if(s.Bridge&&!s.Tunnel&&level.HasValue&&math.isfinite(level.Value))
                    {
                        // Clearance uses eps once, not the cut/fill budget or a
                        // second inverse-variance factor.
                        for(var k=0;k<4;k++)coefficients[k]*=budget/eps;
                        Row(4,-(level.Value+7)/eps,10000*weight/support,ProfileRowKind.UpperHingeCost);
                    }
                }
            }
            if(design!=null)
            {
                // Physical grade differences, normalized once by Gd*ds/window.
                // This replaces the former ad-hoc curvature penalty.
                var window=s.Kind==NetworkKind.Rail||!s.ProtectedLink&&(s.Classification is "motorway" or "trunk")?500d:100d;
                for(var sample=0;sample<3;sample++)
                {
                    var first=sample/3d;var last=(sample+1)/3d;var referenceCurve=new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);
                    var v0=math.length(NetworkDesignChecks.Derivative(referenceCurve,first).xz);var v1=math.length(NetworkDesignChecks.Derivative(referenceCurve,last).xz);
                    if(!(v0>1e-8&&v1>1e-8))continue;
                    var spacing=Math.Max(1e-8,NetworkDesignChecks.HorizontalArcLength(referenceCurve,first,last));var sigma=NetworkVerticalProfiles.DesiredGrade(s)*spacing/window;
                    double Derivative(int control,double t)=>control switch{0=>-3*(1-t)*(1-t),1=>3*(1-t)*(1-t)-6*(1-t)*t,2=>6*(1-t)*t-3*t*t,_=>3*t*t};
                    for(var k=0;k<4;k++)coefficients[k]=(Derivative(k,last)/v1-Derivative(k,first)/v0)/sigma;
                    Row(4,0,spacing,ProfileRowKind.EqualityCost);
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
            if(design?.Assets==null&&samples!=null)foreach(var sample in samples)
            {
                var t=(double)sample.T;var u=1-t;
                coefficients[0]=u*u*u/eps;coefficients[1]=3*u*u*t/eps;coefficients[2]=3*u*t*t/eps;coefficients[3]=t*t*t/eps;
                // The existing finite cover observations are frozen together
                // with grades. Continuous terrain-triangle certification is
                // tracked separately, never claimed from these observations.
                Row(4,(sample.Ceiling-.25)/eps,10000d/samples.Count,ProfileRowKind.UpperHingeCost);
            }
        }
        var contactStart=rows.Count;long contactRows=0;
        if(design?.Assets!=null)
        {
            var constantCount=0;var constantWitnesses=new List<object>();var horizontalWitnesses=new List<object>();
            // Compact row ranges only, not a retained contact triangle graph.
            // Replay rows and source section maps supply coefficients/provenance.
            var contactEvidence=log.DiagnosticsEnabled?new List<NetworkPhysicalContacts.ContactEvidence>():null;
            var contacts=NetworkPhysicalContacts.Append(source,design.Assets,from,c1,c2,to,rows,eps,token,design.Progress,
                log.DiagnosticsEnabled?witness=>{constantCount++;if(constantWitnesses.Count<128)constantWitnesses.Add(witness);}:null,
                log.DiagnosticsEnabled?witness=>{if(horizontalWitnesses.Count<128)horizontalWitnesses.Add(witness);}:null,
                contactEvidence==null?null:contactEvidence.Add,design.ContactGeometry);
            if(log.DiagnosticsEnabled)
            {
                var prefix=design.CaptureReplay?"":$"{design.EvidenceScope}-unit-{source[0].NativeUnitId}-";
                log.DiagnosticCompressedRecords(prefix+"physical-contact-provenance.ndjson.gz",contactEvidence!.Select(p=>(object)new
                    {p.FirstLeaf,p.SecondLeaf,p.FirstSection,p.SecondSection,p.FirstBegin,p.FirstEnd,p.SecondBegin,p.SecondEnd,p.Classification,
                        firstRow=p.FirstRow<0?-1:p.FirstRow-contactStart,p.RowCount,rowBase=contactStart,
                        identityScope="this-frozen-physical-builder-leaf-pair",coefficientsSource=design.CaptureReplay?"profile-problem.ndjson.gz":null,
                        localCoefficientsOnlyCapturedForFailure=!design.CaptureReplay}));
                log.DiagnosticRecords(prefix+"physical-constant-demands.ndjson",constantWitnesses,critical:true);
                log.DiagnosticRecords(prefix+"horizontal-conflicts.ndjson",horizontalWitnesses,critical:true);
                log.Diagnostic($"PROFILE CONSTANT DEMANDS rows={constantCount}; captured={constantWitnesses.Count}; completeWitnessCoverage={constantCount<=128}; retainedInObjective=true; placementBlocked=false");
            }
            contactRows=contacts.Rows;
            log.Diagnostic($"PROFILE PHYSICAL CONTACTS leaves={contacts.Leaves}; uncertifiedLeaves={contacts.UncertifiedLeaves}; missingAssetContracts={contacts.MissingAssets}; regions={contacts.Regions}; rows={contacts.Rows}; candidateTests={contacts.CandidateTests}; localTransitionLeafPairs={contacts.LocalTransitionPairs}; pairedFormationLeafPairs={contacts.PairedFormationPairs}; connectedGradeContradictions={contacts.GradeContradictionPairs}; contradictionAloneRemovesRows=false; fullWidth=true; centrelineCrossingsDisabled=true; sideEnvelopeUnion=true; qualityFailureAction=RequiresReview");
        }
        var coverStart=rows.Count;var tunnelRows=0L;NetworkTunnelContacts.Report tunnelReport=default;
        if(design?.Assets!=null)
        {
            tunnelReport=NetworkTunnelContacts.Append(source,design.Heights,scale,design.Width,design.Assets,from,c1,c2,to,rows,eps,token,design.Progress);
            tunnelRows=tunnelReport.Rows;
            log.Diagnostic($"PROFILE CONTINUOUS COVER leaves={tunnelReport.Leaves}; rows={tunnelRows}; triangles={tunnelReport.Triangles}; uncertified={tunnelReport.Uncertified}; missingAssets={tunnelReport.MissingAssets}; sampledRowsReplaced=true; portalDistanceUpperBounds=true; roofMetadataFaded=false; qualityFailureBlocksPlacement=false");
        }
        log.Diagnostic($"PROFILE SHARED FAMILIES nodes={heightFamilies.Count}; pairedSpans={controlFamilies.Count}; nearestSampleParallelPenalties=false; topologyIdsMerged=false");
        for(var i=0;i<prior.Count;i++)
        {indices[0]=i;coefficients[0]=1;Row(1,prior[i],1e-8,ProfileRowKind.EqualityCost);}
        rows.Freeze();var clock=System.Diagnostics.Stopwatch.StartNew();
        var initial=prior.ToArray();var blocks=stars.ToArray();
        var globalReplay=log.DiagnosticsEnabled&&design?.CaptureReplay==true;
        IEnumerable<object> Evidence(int root,int[]? global,SparseProfileRows problem,double[] seed,ProfileStar[] componentStars,
            ProfileSolveResult? retained,bool includeRows,string? rowSource)
        {
            yield return new{record="header",root,variables=seed.Length,rows=includeRows?problem.Count:0,fullProblemRows=problem.Count,scale,
                problemScope=includeRows?(root<0?"global-problem":"component-problem"):"result-only",rowSource,
                solver="safeguarded-FISTA",iterationCap=2000,retainedVectorAvailable=retained!=null,report=retained?.Report};
            for(var i=0;i<seed.Length;i++)yield return new{record="variable",local=i,global=global==null?i:global[i],prior=seed[i],retained=retained?.Heights[i]??seed[i],
                sourceNode=keys.TryGetValue(global==null?i:global[i],out var key)?(SourceNodeKey?)key:null};
            foreach(var star in componentStars)yield return new{record="star",star.Height,star.Controls,star.Alpha,star.Groups,star.FixedHeight,star.FixedSlopes};
            if(includeRows)for(long i=0;i<problem.Count;i++)
            {
                if((i&16383)==0){token.ThrowIfCancellationRequested();design?.Progress?.Invoke($"Saving replayable road/rail profile equations {i:N0}/{problem.Count:N0}");}
                var row=problem.Row(i);var ii=new int[row.Count];var aa=new double[row.Count];
                for(var j=0;j<ii.Length;j++){ii[j]=problem.Index(row.Offset+j);aa[j]=problem.Coefficient(row.Offset+j);}
                yield return new{record="row",ordinal=i,indices=ii,coefficients=aa,row.Target,row.Weight,kind=row.Kind.ToString()};
            }
            yield return new{record="complete",variables=seed.Length,stars=componentStars.Length,rows=includeRows?problem.Count:0,terms=includeRows?problem.Terms:0};
        }
        if(globalReplay)
        {
            log.DiagnosticRecordChunks("profile-variable-map.ndjson",ordered.Select(i=>(object)new{section=i,source[i].WayId,source[i].StartNodeId,source[i].EndNodeId,
                source[i].SourceWayIds,source[i].NativeAssetKey,kind=source[i].Kind.ToString(),from=from[i],firstControl=c1[i],secondControl=c2[i],to=to[i],
                source[i].PairedProfileId,source[i].PairedReferenceBegin,source[i].PairedReferenceEnd}));
            log.DiagnosticCompressedRecords("profile-problem.ndjson.gz",Evidence(-1,null,rows,initial,blocks,null,true,null));
        }
        var outcomes=new int[4];var componentFailures=0;var failureCaptures=0;var cappedCaptures=0;
        var solution=NetworkProfileSolve.SolveComponents(rows,initial,scale,blocks,token,(id,count,report)=>
        {
            outcomes[(int)report.Outcome]++;
            if(report.Outcome>=ProfileSolveOutcome.NumericFallback&&componentFailures++<16)
                log.Diagnostic($"PROFILE COMPONENT root={id}; variables={count}; outcome={report.Outcome}; reason={report.FailureReason}; iterations={report.AcceptedIterations}; objective={report.Objective:R}; bestFiniteProfileRetained=true");
        },failureEvidence:log.DiagnosticsEnabled?(root,global,problem,seed,componentStars,failed)=>
        {
            // Capture the actual packed local problem, not a nearby road sample.
            // The archive declares truncation if its reserved budget is spent.
            if(failed.Report.Outcome==ProfileSolveOutcome.IterationCap){if(cappedCaptures++>=1)return;}
            else {if(failureCaptures++>=4)return;}
            var prefix=design?.CaptureReplay==false?$"profile-{design.EvidenceScope}-unit-{source[0].NativeUnitId}":"profile";
            log.DiagnosticCompressedRecords($"{prefix}-component-{root}-failure.ndjson.gz",Evidence(root,global,problem,seed,componentStars,failed,!globalReplay,
                globalReplay?"profile-problem.ndjson.gz":null));
        }:null,progress:design?.Progress==null?null:(component,total,iteration,variables)=>
            design.Progress($"Smoothing road/rail profile component {component}/{total}; {variables:N0} height variables; iteration {iteration}/2000"),
            diagnostic:log.DiagnosticsEnabled?log.Diagnostic:null);
        if(globalReplay)log.DiagnosticCompressedRecords("profile-result.ndjson.gz",Evidence(-1,null,rows,initial,blocks,solution,false,"profile-problem.ndjson.gz"));
        var physicalFailures=0L;var physicalMaximum=0d;var worstPhysical=new SortedSet<(double Residual,long Row)>();
        for(var r=contactStart;r<contactStart+contactRows;r++)
        {
            if((r&1023)==0)token.ThrowIfCancellationRequested();var row=rows.Row(r);var residual=-row.Target;
            for(var j=0;j<row.Count;j++)residual+=rows.Coefficient(row.Offset+j)*solution.Heights[rows.Index(row.Offset+j)];
            residual*=eps;if(residual<=.05)continue;physicalFailures++;physicalMaximum=Math.Max(physicalMaximum,residual);
            if(log.DiagnosticsEnabled){worstPhysical.Add((residual,r));if(worstPhysical.Count>64)worstPhysical.Remove(worstPhysical.Min);}
        }
        log.Count("Native footprint clearance rows requiring review",(int)Math.Min(int.MaxValue,physicalFailures));
        long failedCoverRows=0;var maximumCoverRow=0d;
        for(var r=coverStart;r<coverStart+tunnelRows;r++)
        {
            if((r&1023)==0)token.ThrowIfCancellationRequested();var row=rows.Row(r);var residual=-row.Target;
            for(var j=0;j<row.Count;j++)residual+=rows.Coefficient(row.Offset+j)*solution.Heights[rows.Index(row.Offset+j)];
            residual*=eps;if(residual>eps){failedCoverRows++;maximumCoverRow=Math.Max(maximumCoverRow,residual);}
        }
        log.Count("Continuous tunnel cover rows requiring review",(int)Math.Min(int.MaxValue,failedCoverRows));
        log.Diagnostic($"PROFILE PHYSICAL RESULT checkedRows={contactRows}; rowsRequiringReview={physicalFailures}; maximumResidualMeters={physicalMaximum:R}; continuousCoverRows={tunnelRows}; coverRowsRequiringReview={failedCoverRows}; maximumCoverResidualMeters={maximumCoverRow:R}; coverUncertified={tunnelReport.Uncertified}; nativeMeshVerified=false; placementBlockedForQuality=false");
        var evidencePrefix=design?.CaptureReplay==false?$"{design.EvidenceScope}-unit-{source[0].NativeUnitId}-":"";
        if(log.DiagnosticsEnabled)log.DiagnosticRecords(evidencePrefix+"physical-contact-residuals.ndjson",worstPhysical.Reverse().Select(entry=>
        {
            var row=rows.Row(entry.Row);var variables=new int[row.Count];var weights=new double[row.Count];var heights=new double[row.Count];
            for(var j=0;j<variables.Length;j++){variables[j]=rows.Index(row.Offset+j);weights[j]=rows.Coefficient(row.Offset+j);heights[j]=solution.Heights[variables[j]];}
            return (object)new{row=entry.Row,residualMeters=entry.Residual,variables,coefficients=weights,retainedHeights=heights,row.Target,normalizationMeters=eps,quality="RequiresReview",sampleSelection="worst-64-rows",completeCoverage=physicalFailures<=64};
        }),critical:true);
        var result=source.ToArray();var maximumGrade=0f;var exposed=0;var maxExcess=0d;var gradeFailures=0;
        foreach(var i in ordered)
        {
            var s=source[i];var a=s.CourseStart;var b=s.CourseControl1;var c=s.CourseControl2;var d=s.CourseEnd;
            a.y=(float)solution.Heights[from[i]];b.y=(float)solution.Heights[c1[i]];c.y=(float)solution.Heights[c2[i]];d.y=(float)solution.Heights[to[i]];
            var start=s.Start;var end=s.End;start.y=a.y-s.StartCurveOffset.y;end.y=d.y-s.EndCurveOffset.y;
            result[i]=s with{Start=start,End=end,Control1=b-s.StartCurveOffset,Control2=c-s.EndCurveOffset,
                FinalDeckCurve=new(a,b,c,d),HorizontalParent=null,StartElevation=design==null?(s.StartElevation??0)+a.y-s.CourseStart.y:a.y-Ground(a),EndElevation=design==null?(s.EndElevation??0)+d.y-s.CourseEnd.y:d.y-Ground(d)};
            maximumGrade=Math.Max(maximumGrade,NetworkVerticalProfiles.MaximumGrade(result[i]));
            if(!NetworkVerticalProfiles.WithinGradeCeiling(result[i],NetworkGeometrySafety.GradeLimit(s)))gradeFailures++;
        }
        foreach(var sample in cover)
        {var excess=NetworkNormalizer.Evaluate(result[sample.index],sample.t).y-sample.ceiling;if(excess>.05){exposed++;maxExcess=Math.Max(maxExcess,excess);}}
        var report=solution.Report;
        log.Quality(new("planned","profile-convergence",design?.CaptureReplay==false?design.EvidenceScope+"-unit-"+source[0].NativeUnitId:"initial-design",
            report.Components,outcomes.Skip(1).Sum(),outcomes[3],report.OptimalityGap,0,
            "Sole safeguarded FISTA; unchanged 2000 accepted-iteration cap; returned-vector gap and physical acceptance are separate requirements"));
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
            log.Diagnostic($"V-PROFILE JOINT outcome={report.Outcome}; variables={prior.Count}; rows={rows.Count}; terms={rows.Terms}; packedCapacityBytes={rows.CapacityBytes}; vectorBytes={report.VectorCapacityBytes}; iterations={report.AcceptedIterations}; restarts={report.Restarts}; backtracks={report.Backtracks}; gap={report.OptimalityGap:R}; objective={report.Objective:R}; maximumGrade={maximumGrade:R}; sampledCoverResidual={maxExcess:R}; observedCoverOnly={design?.Assets==null}; continuousCoverRows={tunnelRows}; continuousCoverResidual={maximumCoverRow:R}; uncertifiedCoverLeaves={tunnelReport.Uncertified}; missingRoofContracts={tunnelReport.MissingAssets}; coverOnlyPostRewrite=false; fixedHeightBox=true; soleOptimizer=FISTA; nativeMeshStillRequiresReadback=true");
            log.Diagnostic($"PROFILE DESIGN fixedTerrainReference={design!=null}; physicalContactRows={contactRows}; independentComponents={report.Components}; convergedComponents={outcomes[0]}; cappedComponents={outcomes[1]}; numericFallbackComponents={outcomes[2]}; unavailableComponents={outcomes[3]}; perComponentIterationCap=2000; longitudinalSmoothness={design!=null}; repeatedEndpointRaising=false; clearSpaceMeters=5; nativeCompositionClearanceStillRequiresReadback=true");
            log.DiagnosticRecords(evidencePrefix+"joint-tunnel-profile-input.ndjson",DiagnosticArtifacts.NetworkRecords(source.Take(32).ToArray()));
            log.DiagnosticRecords(evidencePrefix+"joint-tunnel-profile.ndjson",DiagnosticArtifacts.NetworkRecords(result.Take(32).ToArray()));
            var failures=Enumerable.Range(0,result.Length).Where(i=>!NetworkVerticalProfiles.WithinGradeCeiling(result[i],NetworkGeometrySafety.GradeLimit(result[i])));
            var coverFailures=cover.Where(p=>NetworkNormalizer.Evaluate(result[p.index],p.t).y>p.ceiling+.05).Select(p=>p.index);
            log.DiagnosticRecords(evidencePrefix+"joint-tunnel-profile-failures.ndjson",DiagnosticArtifacts.NetworkRecords(failures.Concat(coverFailures).Distinct().Take(32).Select(i=>result[i]).ToArray()));
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
        physicalQuality?.Invoke(new(physicalFailures,physicalMaximum,failedCoverRows,maximumCoverRow,tunnelReport.Uncertified,gradeFailures));
        return result;
    }
}
