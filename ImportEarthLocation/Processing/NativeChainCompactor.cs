using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// OSM shape points and fitter boundaries are not mandatory game junctions.
// Combine compatible degree-two curves before grading/roadbed/clearance work.
// Never contract a branch, mode/structure/lane transition, portal or marker.
public static class NativeChainCompactor
{
    public static IReadOnlyList<PreparedSegment> Prepare(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token)
    {
        var work=source.Cast<PreparedSegment?>().ToArray();
        var nodes=new Dictionary<string,HashSet<int>>(StringComparer.Ordinal);
        string Key(PreparedSegment s,long id)=>NativeEndpointPolicy.Identity(s,id);
        void Add(int index,PreparedSegment s){foreach(var id in new[]{s.StartNodeId,s.EndNodeId}){var key=Key(s,id);if(!nodes.TryGetValue(key,out var set))nodes[key]=set=new HashSet<int>();set.Add(index);}}
        void Remove(int index,PreparedSegment s){nodes[Key(s,s.StartNodeId)].Remove(index);nodes[Key(s,s.EndNodeId)].Remove(index);}
        for(var i=0;i<work.Length;i++)Add(i,work[i]!);
        var protectedAliases=new HashSet<SourceNodeKey>(source.SelectMany(s=>s.SourceNodeAliases.Where(a=>a.Mandatory).Select(a=>new SourceNodeKey(s.Kind,a.CanonicalNodeId))));
        // Process short joins first: insertion order can otherwise absorb a
        // short span into a long one and strand its other artificial junction
        // against the unchanged 400m fit bound.
        var pending=new SortedSet<(float Length,string Key,int Version)>();
        var versions=new Dictionary<string,int>(StringComparer.Ordinal);
        void Queue(string key)
        {
            var version=versions.TryGetValue(key,out var previous)?previous+1:1;
            versions[key]=version;
            if(nodes[key].Count!=2)return;
            var length=0f;
            foreach(var index in nodes[key])
            {
                var segment=work[index]!;var point=NetworkNormalizer.Evaluate(segment,0);
                for(var step=1;step<=32;step++){var next=NetworkNormalizer.Evaluate(segment,step/32f);length+=math.distance(point.xz,next.xz);point=next;}
            }
            pending.Add((length,key,version));
        }
        foreach(var key in nodes.Keys)Queue(key);
        var merged=0;var fitRejections=0;
        var rejected=new Dictionary<string,int>(StringComparer.Ordinal);
        void Reject(string reason){rejected[reason]=rejected.TryGetValue(reason,out var count)?count+1:1;}
        while(pending.Count>0)
        {
            token.ThrowIfCancellationRequested();var next=pending.Min;pending.Remove(next);
            var key=next.Key;if(versions[key]!=next.Version)continue;var incident=nodes[key];
            if(incident.Count!=2)continue;
            var ids=incident.OrderBy(i=>i).ToArray();var a=work[ids[0]]!;var b=work[ids[1]]!;
            if(Key(a,a.EndNodeId)!=key){if(a.OneWay) { (a,b)=(b,a); (ids[0],ids[1])=(ids[1],ids[0]); } else a=NetworkNormalizer.Reverse(a);}
            if(Key(a,a.EndNodeId)!=key)continue;
            if(Key(b,b.StartNodeId)!=key){if(b.OneWay)continue;b=NetworkNormalizer.Reverse(b);}
            if(!Compatible(a,b)){Reject("semantic-or-structure-boundary");continue;}
            if(a.StartNodeId==b.EndNodeId||!math.all(a.End==b.Start)||a.EndMiniRoundabout||b.StartMiniRoundabout||
                a.EndTerminal||b.StartTerminal||a.EndRoundaboutRadius>0||b.StartRoundaboutRadius>0||protectedAliases.Contains(new(a.Kind,a.EndNodeId))){Reject("loop-anchor-or-marker");continue;}
            var candidate=Fit(a,b,token,out var reason);
            if(candidate!=null&&candidate.Kind==NetworkKind.Rail&&candidate.NativeCenterlineCurvatureLimit.HasValue&&
                NetworkDesignChecks.Certify(new(candidate.CourseStart,candidate.CourseControl1,candidate.CourseControl2,candidate.CourseEnd),
                    candidate.NativeCenterlineCurvatureLimit.Value,double.PositiveInfinity,token:token,maximumLeaves:4096).Quality!=DesignQuality.Certified)
            {candidate=null;reason="native-train-curvature-contract";}
            if(candidate==null)
            {
                Reject(reason);
                if(++fitRejections<=10)log.Diagnostic($"NATIVE SHAPE NODE retained={key}; ways={a.WayId},{b.WayId}; reason={reason}; toleranceRelaxed=false; sourceUnchanged=true");
                continue;
            }
            Remove(ids[0],work[ids[0]]!);Remove(ids[1],work[ids[1]]!);
            work[ids[0]]=candidate;work[ids[1]]=null;Add(ids[0],candidate);merged++;
            Queue(Key(candidate,candidate.StartNodeId));Queue(Key(candidate,candidate.EndNodeId));
            if(merged<=25)log.Diagnostic($"NATIVE SHAPE NODE compacted={key}; sourceWays={string.Join(",",candidate.SourceWayIds)}; retainedEnds={candidate.StartNodeId},{candidate.EndNodeId}; branchesRemoved=0; fitToleranceMeters={NetworkNormalizer.Tolerance(candidate.Kind)}; gradingStillPending=true");
        }
        log.Count("Native artificial junctions compacted",merged);
        log.Diagnostic($"NATIVE CHAIN COMPACTION input={source.Count}; output={source.Count-merged}; artificialNodesRemoved={merged}; realBranchesPreserved=true; curveEvidenceRetained=true");
        log.Diagnostic("NATIVE CHAIN REJECTIONS "+string.Join("; ",rejected.OrderBy(p=>p.Key).Select(p=>$"{p.Key}={p.Value}"))+"; countsAreCandidateVisits=true; boundedTwoHandleFit=true; toleranceRelaxed=false");
        return work.Where(s=>s!=null).Select(s=>s!).ToArray();
    }

    private static bool Compatible(PreparedSegment a,PreparedSegment b)=>
        a.Bridge==b.Bridge&&a.Tunnel==b.Tunnel&&a.Kind==b.Kind&&a.Classification==b.Classification&&a.Roundabout==b.Roundabout&&
        a.OneWay==b.OneWay&&a.Lanes==b.Lanes&&a.Layer==b.Layer&&a.RailService==b.RailService&&a.PhysicalTracks==b.PhysicalTracks&&
        a.ForwardLanes==b.ForwardLanes&&a.BackwardLanes==b.BackwardLanes&&a.BothWaysLanes==b.BothWaysLanes&&
        a.ExplicitIncline==b.ExplicitIncline&&a.SignedInclinePriorAccepted==b.SignedInclinePriorAccepted&&a.SourceSpeedKph==b.SourceSpeedKph&&a.SourceWidthMeters==b.SourceWidthMeters&&
        a.NativeWidth==b.NativeWidth&&a.NativeAssetKey==b.NativeAssetKey&&a.NativeCenterlineCurvatureLimit==b.NativeCenterlineCurvatureLimit&&a.NativeLaneOffsets.SequenceEqual(b.NativeLaneOffsets)&&
        a.CorridorId==b.CorridorId&&a.ConsolidatedAvenue==b.ConsolidatedAvenue&&a.ExplicitCutSides==b.ExplicitCutSides&&
        a.ExplicitWallSides==b.ExplicitWallSides&&a.RetainingWallSides==b.RetainingWallSides&&a.Name==b.Name&&a.Reference==b.Reference&&
        math.all(a.StartCurveOffset==0)&&math.all(a.EndCurveOffset==0)&&math.all(b.StartCurveOffset==0)&&math.all(b.EndCurveOffset==0)&&
        TagsCompatible(a.SourceTags,b.SourceTags)&&TagsCompatible(a.RawSourceTags,b.RawSourceTags);

    private static bool TagsCompatible(IReadOnlyDictionary<string,string> a,IReadOnlyDictionary<string,string> b)
    {
        // Preserve permissions, lane directives, surfaces and unknown semantics.
        // Only metadata already compared as normalized fields or provenance-only
        // annotations may differ; never discard a second piece's routing rules.
        bool Annotation(string key)=>key=="name"||key=="ref"||key=="lanes"||key=="maxspeed"||key=="width"||
            key=="source"||key.StartsWith("source:",StringComparison.Ordinal)||key=="note"||key=="fixme"||key=="created_by";
        foreach(var key in a.Keys.Concat(b.Keys).Distinct())
            if(!Annotation(key)&&(!a.TryGetValue(key,out var av)||!b.TryGetValue(key,out var bv)||av!=bv))return false;
        return true;
    }

    private static PreparedSegment? Fit(PreparedSegment a,PreparedSegment b,CancellationToken token,out string reason)
    {
        reason="curve-span-bound";
        var points=new List<float3>();
        foreach(var s in new[]{a,b})for(var i=points.Count==0?0:1;i<=32;i++)points.Add(NetworkNormalizer.Evaluate(s,i/32f));
        var lengths=new float[points.Count];for(var i=1;i<points.Count;i++)lengths[i]=lengths[i-1]+math.distance(points[i-1].xz,points[i].xz);
        var total=lengths[lengths.Length-1];if(total<.01f||total>400)return null;
        var da=math.normalizesafe((a.CourseControl1-a.CourseStart).xz);var db=math.normalizesafe((b.CourseEnd-b.CourseControl2).xz);
        reason="outer-tangent-bound";
        if(math.lengthsq(da)<.5f||math.lengthsq(db)<.5f||math.dot(da,db)<0)return null; // at most a quarter-turn
        var original=a.OriginalGeometry.Concat(b.OriginalGeometry).ToArray();
        var reference=(a.SourceGeometry.Count>=2?a.SourceGeometry:points.Take(33).ToArray())
            .Concat(b.SourceGeometry.Count>=2?b.SourceGeometry:points.Skip(32).ToArray()).ToArray();
        reason="reference-point-budget";if(reference.Length>4096)return null;
        var template=a with{EndNodeId=b.EndNodeId,End=b.End,EndElevation=b.EndElevation,ExplicitLanes=a.ExplicitLanes||b.ExplicitLanes,
            MaximumNativeWidth=Math.Min(a.MaximumNativeWidth,b.MaximumNativeWidth),
            EndMiniRoundabout=b.EndMiniRoundabout,EndTerminal=b.EndTerminal,EndRoundaboutRadius=b.EndRoundaboutRadius,
            SourceWayIds=a.SourceWayIds.Concat(b.SourceWayIds).Concat(new[]{a.WayId,b.WayId}).Distinct().ToArray(),
            SourceNodeAliases=a.SourceNodeAliases.Concat(b.SourceNodeAliases).Distinct().ToArray(),
            SourceGeometry=reference,OriginalGeometry=original,FinalDeckCurve=null,HorizontalParent=null,ParentStartT=0,ParentEndT=1};
        // Use the same physical-arc support and box-constrained reach policy as
        // source spans and whole approaches. Compaction may not accumulate drift
        // away from either the immutable source or the current pair of curves.
        reason="bounded-source-fit";
        if(!NetworkSourceFit.TryFit(points,a.CourseStart,b.CourseEnd,da,db,NetworkNormalizer.Tolerance(a.Kind),token,out var curve,out _))return null;
        var candidate=template with{Control1=(float3)curve.P1,Control2=(float3)curve.P2,HorizontalEmissionReference=curve};
        var rawValid=NetworkNormalizer.WithinTolerance(candidate);
        var pairValid=NetworkNormalizer.WithinTolerance(candidate with{SourceGeometry=points});
        reason=rawValid&&pairValid?"accepted":!rawValid&&!pairValid?"raw-reference-and-pair-tolerance":!rawValid?"raw-reference-tolerance":"current-pair-tolerance";
        return rawValid&&pairValid?candidate:null;
    }
}
