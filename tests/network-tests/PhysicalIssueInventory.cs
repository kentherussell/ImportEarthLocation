using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

// Offline inventory. Leaf parameters refer to final emitted sections; source
// way membership alone is not an exact source-parameter correspondence.
static class PhysicalIssueInventory
{
    public static object Capture(IReadOnlyList<PreparedSegment> original,IReadOnlyList<PreparedSegment> final,
        ushort[] terrain,float scale,double width,IReadOnlyDictionary<string,NativeAssetContract> assets)
    {
        var observations=new List<NetworkTunnelProfile.PhysicalObservation>();
        var quality=NetworkTunnelProfile.MeasurePhysical(final,terrain,scale,width,assets,default,observe:observations.Add);
        var sourceWays=original.SelectMany(s=>s.SourceWayIds.Append(s.WayId)).ToHashSet();
        object? Section(int index,double begin,double end)
        {
            if(index<0)return null;var s=final[index];
            var ways=s.SourceWayIds.Append(s.WayId).Distinct().OrderBy(v=>v).ToArray();
            return new{ordinal=index,s.Kind,s.WayId,s.StartNodeId,s.EndNodeId,sourceWays=ways,
                finalParameterBegin=begin,finalParameterEnd=end,s.PairedCorridorId,s.PairedProfileId,
                s.PairedReferenceBegin,s.PairedReferenceEnd,
                sourceMembership=ways.All(sourceWays.Contains)?"KnownSelectedSourceWays":"UnmappedSourceWays",
                sourceParameterMapping="Unavailable"};
        }
        var clearance=observations.Where(o=>o.Category=="full-width-clearance"&&!o.Unavailable).Sum(o=>o.FailedRows);
        var cover=observations.Where(o=>o.Category=="continuous-tunnel-cover"&&!o.Unavailable).Sum(o=>o.FailedRows);
        var grades=observations.Where(o=>o.Category=="exceptional-grade"&&!o.Unavailable).Sum(o=>o.FailedRows);
        var unavailableCoverEvents=observations.Count(o=>o.Category=="continuous-tunnel-cover"&&o.Unavailable);
        if(clearance!=quality.ClearanceFailures||cover!=quality.CoverFailures||grades!=quality.ExceptionalGradeFailures)
            throw new InvalidDataException("Physical interval inventory lost failed certification rows.");
        if(unavailableCoverEvents!=quality.UncertifiedCover)
            throw new InvalidDataException("Physical interval inventory lost unavailable cover events.");
        var rail=RailWitness(original,final);
        return new{quality,rail,failedClearanceAndCoverRowsMapped=true,failedExceptionalGradeSectionsMapped=true,
            unavailableCoverEventsMapped=true,sourceParameterMappingComplete=false,
            coverageCounts=new{failedClearanceRows=clearance,failedCoverRows=cover,exceptionalGradeSections=grades,
                unavailableCoverEvents,
                missingDeckContractEvents=observations.Count(o=>o.Unavailable&&o.Classification=="missing-deck-contract"),
                unavailableClearanceLeafEvents=observations.Count(o=>o.Unavailable&&o.Classification=="unavailable-footprint-leaf"),
                unavailableClearancePairEvents=observations.Count(o=>o.Unavailable&&o.Classification=="unavailable-footprint"),
                interpretation="Unavailable events are not deduplicated intervals or independent map issues; clearance pair events can repeat an unavailable leaf"},
            coverage="All failing clearance/cover certification rows and exceptional-grade sections; reported unavailable intervals; no clipping; not an independent map issue total",
            observations=observations.Select(o=>new{o.Category,o.Classification,o.Unavailable,o.FailedRows,o.MaximumResidual,
                first=Section(o.FirstSection,o.FirstBegin,o.FirstEnd),second=Section(o.SecondSection,o.SecondBegin,o.SecondEnd)}).ToArray()};
    }

    private static object RailWitness(IReadOnlyList<PreparedSegment> original,IReadOnlyList<PreparedSegment> final)
    {
        var incident=new Dictionary<long,List<int>>();var failed=new List<int>();var unavailable=new List<int>();
        for(var i=0;i<final.Count;i++)
        {
            var s=final[i];if(s.Kind!=NetworkKind.Rail)continue;
            foreach(var node in new[]{s.StartNodeId,s.EndNodeId})
            {if(!incident.TryGetValue(node,out var edges))incident[node]=edges=new();edges.Add(i);}
            if(!s.NativeCenterlineCurvatureLimit.HasValue||!double.IsFinite(s.NativeCenterlineCurvatureLimit.Value)||s.NativeCenterlineCurvatureLimit.Value<=0)
            {unavailable.Add(i);continue;}
            var curve=new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);
            if(!NetworkDesignChecks.Finite(curve)||NetworkDesignChecks.Certify(curve,s.NativeCenterlineCurvatureLimit.Value,
                double.PositiveInfinity,maximumLeaves:4096).Quality!=DesignQuality.Certified)failed.Add(i);
        }
        var component=new HashSet<int>();var pending=new Queue<int>();
        var seed=failed.Count>0?failed[0]:unavailable.Count>0?unavailable[0]:-1;
        var selectedWitnessReason=failed.Count>0?"failed-curvature-certificate":unavailable.Count>0?"unavailable-curvature-contract":"no-failed-or-unavailable-rail";
        if(seed>=0){component.Add(seed);pending.Enqueue(seed);}
        while(pending.Count>0)
        {
            var s=final[pending.Dequeue()];
            foreach(var node in new[]{s.StartNodeId,s.EndNodeId})foreach(var next in incident[node])
                if(component.Add(next))pending.Enqueue(next);
        }
        var sourceWays=component.SelectMany(i=>final[i].SourceWayIds.Append(final[i].WayId)).ToHashSet();
        double[][] Points(PreparedSegment s)=>new[]{s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd}
            .Select(p=>new[]{(double)p.x,p.y,p.z}).ToArray();
        return new{failedCertificateSections=failed,missingCurvatureContractSections=unavailable,
            unavailableCurvatureContracts=unavailable.Select(i=>new{ordinal=i,final[i].NativeCenterlineCurvatureLimit,
                reason=!final[i].NativeCenterlineCurvatureLimit.HasValue?"missing":
                    !double.IsFinite(final[i].NativeCenterlineCurvatureLimit!.Value)?"nonfinite":"nonpositive"}).ToArray(),
            selectedWitnessReason,
            selectedWitnessCoverage=seed>=0?"Complete selected rail component at exact final source nodes; no proximity-inferred topology":"No rail component selected",
            members=component.OrderBy(i=>i).Select(i=>new{ordinal=i,final[i].WayId,final[i].SourceWayIds,
                final[i].StartNodeId,final[i].EndNodeId,final[i].NativeAssetKey,final[i].NativeWidth,
                final[i].NativeCenterlineCurvatureLimit,final[i].RailTerminalFormationId,
                final[i].Layer,final[i].Bridge,final[i].Tunnel,final[i].OneWay,final[i].PhysicalTracks,final[i].RailService,
                final[i].SourceFitToleranceMeters,final[i].MaximumNativeWidth,final[i].NativeLaneOffsets,
                startDegree=incident[final[i].StartNodeId].Count,endDegree=incident[final[i].EndNodeId].Count,
                controls=Points(final[i])}).ToArray(),
            immutableSelectedSource=original.Where(s=>s.Kind==NetworkKind.Rail&&s.SourceWayIds.Append(s.WayId).Any(sourceWays.Contains))
                .Select(s=>new{s.WayId,s.SourceWayIds,s.StartNodeId,s.EndNodeId,
                    sourceGeometry=s.SourceGeometry.Select(p=>new[]{(double)p.x,p.y,p.z}).ToArray(),
                    originalGeometry=s.OriginalGeometry.Select(p=>new[]{(double)p.x,p.y,p.z}).ToArray(),
                    s.NativeAssetKey,s.NativeWidth,s.NativeCenterlineCurvatureLimit,controls=Points(s)}).ToArray(),
            newRecipeCertified=false};
    }
}
