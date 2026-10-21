using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// One pre-freeze repair request, not a repeated geometry/terrain feedback loop.
// Rebuild complete local approaches, retain fixed outer poses, and include
// immutable neighbouring decks in the same physical rows as frozen context.
public static class NetworkPhysicalReconstruction
{
    public static IReadOnlyList<PreparedSegment> Once(IReadOnlyList<PreparedSegment> horizontal,IReadOnlyList<PreparedSegment> solved,
        ushort[] terrain,float scale,double width,IReadOnlyDictionary<string,NativeAssetContract>? assets,Func<float3,float?>? water,
        ImportLog log,CancellationToken token,Action<string>? progress=null,NetworkTunnelProfile.PhysicalQuality? initialQuality=null,
        Action<string,SparseProfileRows>? inspectProfileRows=null,Action<object>? inspectProfileSeam=null)
    {
        var failed=new HashSet<int>();
        for(var i=0;i<solved.Count;i++)
        {token.ThrowIfCancellationRequested();if(!NetworkVerticalProfiles.WithinGradeCeiling(solved[i],NetworkGeometrySafety.GradeLimit(solved[i])))failed.Add(i);}
        if(failed.Count==0)return solved;
        var incident=new Dictionary<SourceNodeKey,List<int>>();
        SourceNodeKey Key(PreparedSegment s,bool start)=>new(s.Kind,start?s.StartNodeId:s.EndNodeId);
        for(var i=0;i<solved.Count;i++)foreach(var start in new[]{true,false})
        {var key=Key(solved[i],start);if(!incident.TryGetValue(key,out var list))incident[key]=list=new();list.Add(i);}
        var affected=new HashSet<int>(failed);var queue=failed.Select(i=>(Index:i,Distance:0d)).ToList();
        var distances=queue.ToDictionary(p=>p.Index,p=>p.Distance);
        for(var k=0;k<queue.Count;k++)
        {
            token.ThrowIfCancellationRequested();var current=queue[k];var s=solved[current.Index];
            var reach=s.Kind==NetworkKind.Rail||s.Classification is "motorway" or "trunk"?500:220;
            foreach(var start in new[]{true,false})
            {
                var arms=incident[Key(s,start)];if(arms.Count!=2)continue;
                foreach(var next in arms)
                {
                    if(next==current.Index)continue;var t=solved[next];
                    if(t.Kind!=s.Kind||t.OneWay!=s.OneWay||t.ProtectedLink!=s.ProtectedLink)continue;
                    var distance=current.Distance+NetworkDesignChecks.HorizontalArcLength(new(t.CourseStart,t.CourseControl1,t.CourseControl2,t.CourseEnd));
                    if(distance>reach||distances.TryGetValue(next,out var old)&&old<=distance)continue;
                    distances[next]=distance;affected.Add(next);queue.Add((next,distance));
                }
            }
        }
        // Shared profile controls are indivisible even when one carriageway's
        // exceptional-grade witness occurs between different sample stations.
        var families=new HashSet<long>(affected.Select(i=>solved[i].PairedProfileId).Where(id=>id!=0));
        for(var i=0;i<solved.Count;i++)if(families.Contains(solved[i].PairedProfileId))affected.Add(i);
        var candidate=solved.ToArray();var visited=new HashSet<int>();var rebuilt=0;
        foreach(var seed in affected.OrderBy(i=>solved[i].Kind).ThenBy(i=>solved[i].WayId).ThenBy(i=>solved[i].StartNodeId))
        {
            if(visited.Contains(seed))continue;
            var path=new List<(int Index,bool Forward)>();var first=Key(solved[seed],true);var edge=seed;
            // Walk back to the fixed boundary of this affected degree-two run.
            var back=new HashSet<int>{seed};
            while(incident[first].Count==2)
            {
                var next=incident[first].First(i=>i!=edge);
                if(!affected.Contains(next)||!back.Add(next)||solved[next].PairedProfileId!=solved[seed].PairedProfileId)break;
                edge=next;first=Key(solved[next],true)==first?Key(solved[next],false):Key(solved[next],true);
            }
            var at=first;
            while(affected.Contains(edge)&&visited.Add(edge))
            {
                var forward=Key(solved[edge],true)==at;path.Add((edge,forward));at=Key(solved[edge],!forward);
                if(incident[at].Count!=2)break;var next=incident[at].First(i=>i!=edge);
                if(solved[next].PairedProfileId!=solved[seed].PairedProfileId)break;edge=next;
            }
            if(path.Count==0||first==at||solved[seed].PairedProfileId!=0)continue;
            var points=new List<float3>();var boundaries=new List<double>{0};var total=0d;
            foreach(var item in path)
            {
                var s=horizontal[item.Index];var source=s.SourceGeometry.Count>=2?s.SourceGeometry.ToArray():new[]{s.CourseStart,s.CourseEnd};
                if(!item.Forward)Array.Reverse(source);
                foreach(var point in source)
                {if(points.Count>0){var step=math.distance(points[points.Count-1].xz,point.xz);if(step<1e-8)continue;total+=step;}points.Add(point);}
                boundaries.Add(total);
            }
            if(!(total>0)||points.Count<2)continue;
            var startSection=solved[path[0].Index];var endSection=solved[path[path.Count-1].Index];
            var a=path[0].Forward?startSection.CourseStart:startSection.CourseEnd;
            var b=path[path.Count-1].Forward?endSection.CourseEnd:endSection.CourseStart;
            var ta=path[0].Forward?startSection.CourseControl1-a:a-startSection.CourseControl2;
            var tb=path[path.Count-1].Forward?b-endSection.CourseControl2:endSection.CourseControl1-b;
            var budget=solved[seed].ProtectedLink?48:NetworkNormalizer.Tolerance(solved[seed].Kind);
            if(!NetworkSourceFit.TryFit(points,a,b,ta.xz,tb.xz,budget,token,out var fitted,out _))continue;
            if(solved[seed].Kind==NetworkKind.Rail)
            {
                var limits=path.Select(p=>solved[p.Index].NativeCenterlineCurvatureLimit).ToArray();
                if(limits.Any(limit=>!limit.HasValue)||NetworkDesignChecks.Certify(fitted,limits.Min(limit=>limit!.Value),
                    double.PositiveInfinity,token:token,maximumLeaves:4096).Quality!=DesignQuality.Certified)continue;
            }
            for(var j=0;j<path.Count;j++)
            {
                var item=path[j];var s=solved[item.Index];var lo=boundaries[j]/total;var hi=boundaries[j+1]/total;
                if(!(hi>lo))continue;var part=NetworkDesignChecks.Split(fitted,hi).Left;
                if(lo>0)part=NetworkDesignChecks.Split(part,lo/hi).Right;
                if(!item.Forward)part=new(part.P3,part.P2,part.P1,part.P0);
                candidate[item.Index]=s with{Start=(float3)part.P0-s.StartCurveOffset,Control1=(float3)part.P1-s.StartCurveOffset,
                    Control2=(float3)part.P2-s.EndCurveOffset,End=(float3)part.P3-s.EndCurveOffset,FinalDeckCurve=null,HorizontalParent=null,HorizontalEmissionReference=part};
                rebuilt++;
            }
        }
        // Preserve global portal distances, formation classification and family
        // area weights. Exact fixed-star substitution below makes optimizer
        // dependencies local without an unsafe spatial cut of physical evidence.
        var indices=new List<int>();var frozen=new HashSet<int>();
        for(var i=0;i<solved.Count;i++)
        {
            if(!affected.Contains(i))frozen.Add(indices.Count);indices.Add(i);
        }
        var local=indices.Select(i=>candidate[i]).ToArray();var fixedHeights=new Dictionary<SourceNodeKey,double>();var fixedSlopes=new Dictionary<SourceNodeKey,double>();
        // Spend the existing single corrective solve on a supported structural
        // alternative when ground-chain budgets conflict. This is not another
        // optimizer pass. Full surrounding portal/contact context stays present,
        // and neither mode selection nor anchor publication may move a frozen
        // native neighbour. The existing final physical gate still owns adoption.
        var modeLog=new ImportLog();
        var structural=NetworkVisualDesign.StructureIntent(local,terrain,scale,width,assets,modeLog,token,
            allowedSections:affected,propagateChainConflicts:true);
        var changedModes=Enumerable.Range(0,local.Length).Count(i=>NativeAssetContract.Mode(local[i])!=NativeAssetContract.Mode(structural[i]));
        var frozenPreserved=frozen.All(i=>local[i]==structural[i]);
        var curvatureCertified=Enumerable.Range(0,local.Length).Where(i=>local[i].Kind==NetworkKind.Rail&&
            NativeAssetContract.Mode(local[i])!=NativeAssetContract.Mode(structural[i])).All(i=>
                structural[i].NativeCenterlineCurvatureLimit.HasValue&&NetworkDesignChecks.Certify(
                    new(structural[i].CourseStart,structural[i].CourseControl1,structural[i].CourseControl2,structural[i].CourseEnd),
                    structural[i].NativeCenterlineCurvatureLimit!.Value,double.PositiveInfinity,token:token,maximumLeaves:4096).Quality==DesignQuality.Certified);
        if(changedModes>0&&frozenPreserved&&curvatureCertified)local=structural.ToArray();
        log.Diagnostic($"PHYSICAL STRUCTURAL CANDIDATE changedModes={changedModes}; frozenContextPreserved={frozenPreserved}; nativeCurvatureCertified={curvatureCertified}; submittedToExistingSolve={changedModes>0&&frozenPreserved&&curvatureCertified}; additionalSolves=0; finalPhysicalCertificateRequired=true");
        foreach(var i in affected)foreach(var start in new[]{true,false})
        {
            var s=solved[i];var key=Key(s,start);
            if(incident[key].Count==2&&incident[key].All(affected.Contains))continue;
            var point=start?s.CourseStart:s.CourseEnd;var delta=start?s.CourseControl1-point:point-s.CourseControl2;
            fixedHeights[key]=point.y;
            var reach=math.length((double2)delta.xz);
            if(incident[key].Count==1)fixedSlopes[key]=reach>1e-8?delta.y/reach:0;
        }
        try
        {
            progress?.Invoke($"Reconstructing exceptional-grade approaches once; {affected.Count:N0} affected sections; {frozen.Count:N0} frozen neighbours");
            var baselineQuality=NetworkTunnelProfile.MeasurePhysical(indices.Select(i=>solved[i]).ToArray(),terrain,scale,width,assets,token);
            NetworkTunnelProfile.PhysicalQuality? quality=null;
            var repaired=NetworkTunnelProfile.Solve(local,scale,Array.Empty<(int,float,float)>(),log,token,fixedHeights,
                new(terrain,width,water,assets,progress,CaptureReplay:false,EvidenceScope:"physical-reconstruction")
                    {InspectRows=inspectProfileRows==null?null:(scope,matrix)=>inspectProfileRows("corrective/"+scope,matrix),InspectSeam=inspectProfileSeam},fixedSlopes,frozen,value=>quality=value);
            var remaining=indices.Select((id,i)=>(id,i)).Count(p=>affected.Contains(p.id)&&
                !NetworkVerticalProfiles.WithinGradeCeiling(repaired[p.i],NetworkGeometrySafety.GradeLimit(repaired[p.i])));
            // A failed attempt is evidence, not permission to replace a better
            // visible profile or to start another 2,000-iteration repair loop.
            var actualQuality=NetworkTunnelProfile.MeasurePhysical(repaired,terrain,scale,width,assets,token);
            var accepted=remaining<failed.Count&&quality!=null&&
                actualQuality.ClearanceFailures<=baselineQuality.ClearanceFailures&&actualQuality.CoverFailures<=baselineQuality.CoverFailures&&
                actualQuality.MaximumClearanceResidual<=baselineQuality.MaximumClearanceResidual+1e-6&&
                actualQuality.MaximumCoverResidual<=baselineQuality.MaximumCoverResidual+1e-6&&
                actualQuality.UncertifiedCover<=baselineQuality.UncertifiedCover&&
                actualQuality.UncertifiedClearance<=baselineQuality.UncertifiedClearance&&
                actualQuality.HorizontalConflictPairs<=baselineQuality.HorizontalConflictPairs;
            if(accepted)for(var i=0;i<indices.Count;i++)if(affected.Contains(indices[i]))candidate[indices[i]]=repaired[i];
            log.Diagnostic($"PHYSICAL RECONSTRUCTION attempts=1; initialExceptionalSections={failed.Count}; affectedSections={affected.Count}; rebuiltHorizontalSections={rebuilt}; frozenNeighbours={frozen.Count}; remainingExceptionalSections={remaining}; accepted={accepted}; fixedOuterHeights={fixedHeights.Count}; productionPhysicalAcceptanceNotImplied=true; repeatedRepair=false");
            return accepted?candidate:solved;
        }
        catch(Exception error) when(error is ArgumentException||error is InvalidOperationException)
        {log.Diagnostic("PHYSICAL RECONSTRUCTION RequiresReview; fixed context unavailable; original finite profile retained; attempt not repeated: "+error.Message);return solved;}
    }
}
