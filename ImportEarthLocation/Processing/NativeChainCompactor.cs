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
    public static IReadOnlyList<PreparedSegment> Prepare(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token,SourceTopologyManifest? obligations=null)
    {
        var nodes=new Dictionary<string,HashSet<int>>(StringComparer.Ordinal);
        string Key(PreparedSegment s,long id)=>NativeEndpointPolicy.Identity(s,id);
        void Add(int index,PreparedSegment s){foreach(var id in new[]{s.StartNodeId,s.EndNodeId}){var key=Key(s,id);if(!nodes.TryGetValue(key,out var set))nodes[key]=set=new HashSet<int>();set.Add(index);}}
        for(var i=0;i<source.Count;i++)Add(i,source[i]);
        var protectedAliases=new HashSet<SourceNodeKey>(source.SelectMany(s=>s.SourceNodeAliases.Where(a=>a.Mandatory).Select(a=>new SourceNodeKey(s.Kind,a.CanonicalNodeId))));
        protectedAliases.UnionWith(SourceTopologyObligations.Capture(source).Ports);
        if(obligations!=null)
        {
            var required=new HashSet<SourceNodeKey>(obligations.Ports);protectedAliases.UnionWith(required);
            foreach(var s in source)foreach(var alias in s.SourceNodeAliases)
                if(required.Contains(new(s.Kind,alias.SourceNodeId)))protectedAliases.Add(new(s.Kind,alias.CanonicalNodeId));
        }
        var lengths=source.Select(s=>NetworkDesignChecks.HorizontalArcLength(new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd))).ToArray();
        var visited=new bool[source.Count];var output=new List<PreparedSegment>(source.Count);var merged=0;var fitRejections=0;
        var rejected=new Dictionary<string,int>(StringComparer.Ordinal);
        void Reject(string reason){rejected[reason]=rejected.TryGetValue(reason,out var count)?count+1:1;}
        bool Join(PreparedSegment a,PreparedSegment b,string key)=>Compatible(a,b)&&a.EndNodeId==b.StartNodeId&&
            a.StartNodeId!=b.EndNodeId&&math.all(a.End==b.Start)&&!NetworkBoundaryPolicy.EndpointMarker(a,false)&&!NetworkBoundaryPolicy.EndpointMarker(b,true)&&
            !protectedAliases.Contains(new(a.Kind,a.EndNodeId))&&nodes[key].Count==2;
        foreach(var seed in Enumerable.Range(0,source.Count).OrderBy(i=>source[i].WayId).ThenBy(i=>source[i].StartNodeId).ThenBy(i=>source[i].EndNodeId))
        {
            token.ThrowIfCancellationRequested();if(visited[seed])continue;
            var first=seed;var head=source[seed];var back=new HashSet<int>{seed};
            while(true)
            {
                var key=Key(head,head.StartNodeId);if(nodes[key].Count!=2)break;
                var next=nodes[key].First(i=>i!=first);if(visited[next]||!back.Add(next))break;
                var previous=source[next];if(previous.EndNodeId!=head.StartNodeId){if(previous.OneWay)break;previous=NetworkNormalizer.Reverse(previous);}
                if(!Join(previous,head,key))break;first=next;head=previous;
            }
            var path=new List<PreparedSegment>();var prefix=new List<double>{0};var index=first;var current=head;
            while(!visited[index])
            {
                visited[index]=true;path.Add(current);prefix.Add(prefix[^1]+lengths[index]);
                var key=Key(current,current.EndNodeId);if(nodes[key].Count!=2)break;var next=nodes[key].First(i=>i!=index);
                if(visited[next])break;var other=source[next];
                if(other.StartNodeId!=current.EndNodeId){if(other.OneWay)break;other=NetworkNormalizer.Reverse(other);}
                if(!Join(current,other,key))break;index=next;current=other;
            }
            void Emit(int begin,int end)
            {
                token.ThrowIfCancellationRequested();if(end-begin==1){output.Add(path[begin]);return;}
                PreparedSegment? candidate=null;var reason="curve-span-bound";var preferredSplit=-1;
                if(prefix[end]-prefix[begin]<=400)candidate=Fit(path.Skip(begin).Take(end-begin).ToArray(),token,out reason,out preferredSplit);
                if(candidate!=null){output.Add(candidate);merged+=end-begin-1;return;}
                Reject(reason);fitRejections++;
                var middle=begin+(end-begin)/2;var target=(prefix[begin]+prefix[end])/2;
                for(var k=begin+1;k<end;k++)if(Math.Abs(prefix[k]-target)<Math.Abs(prefix[middle]-target))middle=k;
                // Follow the shared fitter's error witness, but retain bounded
                // recursion even when a pathological endpoint is always worst.
                if(preferredSplit>0)middle=begin+Math.Max(Math.Max(1,(end-begin)/4),Math.Min(end-begin-Math.Max(1,(end-begin)/4),preferredSplit));
                Emit(begin,middle);Emit(middle,end);
            }
            for(var begin=0;begin<path.Count;)
            {
                var end=begin+1;
                while(end<path.Count&&prefix[end+1]-prefix[begin]<=400)end++;
                Emit(begin,end);begin=end;
            }
        }
        log.Count("Native artificial junctions compacted",merged);
        log.Diagnostic($"NATIVE CHAIN COMPACTION input={source.Count}; output={source.Count-merged}; artificialNodesRemoved={merged}; realBranchesPreserved=true; curveEvidenceRetained=true");
        log.Diagnostic("NATIVE CHAIN REJECTIONS "+string.Join("; ",rejected.OrderBy(p=>p.Key).Select(p=>$"{p.Key}={p.Value}"))+"; boundedAdaptiveWindows=true; rejectedFitAttempts="+fitRejections+"; maximumEmittedSpanMeters=400");
        return output;
    }

    internal static bool Compatible(PreparedSegment a,PreparedSegment b,bool requireZeroOffsets=true,bool ignoreNominalLaneCount=false)=>
        a.Bridge==b.Bridge&&a.Tunnel==b.Tunnel&&a.Kind==b.Kind&&a.Classification==b.Classification&&a.Roundabout==b.Roundabout&&
        a.OneWay==b.OneWay&&(a.Lanes==b.Lanes||ignoreNominalLaneCount)&&a.Layer==b.Layer&&a.RailService==b.RailService&&a.PhysicalTracks==b.PhysicalTracks&&
        a.ForwardLanes==b.ForwardLanes&&a.BackwardLanes==b.BackwardLanes&&a.BothWaysLanes==b.BothWaysLanes&&
        a.ExplicitIncline==b.ExplicitIncline&&a.SignedInclinePriorAccepted==b.SignedInclinePriorAccepted&&a.SourceSpeedKph==b.SourceSpeedKph&&a.SourceWidthMeters==b.SourceWidthMeters&&
        a.NativeWidth==b.NativeWidth&&a.NativeAssetKey==b.NativeAssetKey&&a.NativeCenterlineCurvatureLimit==b.NativeCenterlineCurvatureLimit&&a.NativeLaneOffsets.SequenceEqual(b.NativeLaneOffsets)&&
        a.ConsolidatedAvenue==b.ConsolidatedAvenue&&a.ExplicitCutSides==b.ExplicitCutSides&&
        a.ExplicitWallSides==b.ExplicitWallSides&&a.RetainingWallSides==b.RetainingWallSides&&a.Reference==b.Reference&&
        (!requireZeroOffsets||math.all(a.StartCurveOffset==0)&&math.all(a.EndCurveOffset==0)&&math.all(b.StartCurveOffset==0)&&math.all(b.EndCurveOffset==0))&&
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

    private static PreparedSegment? Fit(IReadOnlyList<PreparedSegment> spans,CancellationToken token,out string reason,out int preferredSplit)
    {
        var a=spans[0];var b=spans[spans.Count-1];
        reason="curve-span-bound";preferredSplit=-1;
        var points=new List<float3>();
        foreach(var s in spans)for(var i=points.Count==0?0:1;i<=16;i++)points.Add(NetworkNormalizer.Evaluate(s,i/16f));
        var lengths=new float[points.Count];for(var i=1;i<points.Count;i++)lengths[i]=lengths[i-1]+math.distance(points[i-1].xz,points[i].xz);
        var total=lengths[lengths.Length-1];if(total<.01f||total>400)return null;
        var da=math.normalizesafe((a.CourseControl1-a.CourseStart).xz);var db=math.normalizesafe((b.CourseEnd-b.CourseControl2).xz);
        reason="outer-tangent-bound";
        if(math.lengthsq(da)<.5f||math.lengthsq(db)<.5f||math.dot(da,db)<0)return null; // at most a quarter-turn
        var original=spans.SelectMany(s=>s.OriginalGeometry).ToArray();
        var reference=spans.SelectMany(s=>s.SourceGeometry.Count>=2?s.SourceGeometry:new[]{s.CourseStart,s.CourseEnd}).ToArray();
        var template=a with{EndNodeId=b.EndNodeId,End=b.End,EndElevation=b.EndElevation,ExplicitLanes=spans.Any(s=>s.ExplicitLanes),
            SourceFitToleranceMeters=spans.Min(s=>s.SourceFitToleranceMeters??NetworkNormalizer.Tolerance(s.Kind)),
            MaximumNativeWidth=spans.Min(s=>s.MaximumNativeWidth),
            EndMiniRoundabout=b.EndMiniRoundabout,EndTerminal=b.EndTerminal,EndRoundaboutRadius=b.EndRoundaboutRadius,
            EndLevelCrossing=b.EndLevelCrossing,
            SourceWayIds=spans.SelectMany(s=>s.SourceWayIds.Append(s.WayId)).Distinct().ToArray(),
            SourceNodeAliases=spans.SelectMany(s=>s.SourceNodeAliases).Distinct().ToArray(),
            SourceGeometry=reference,OriginalGeometry=original,FinalDeckCurve=null,HorizontalParent=null,ParentStartT=0,ParentEndT=1};
        // Use the same physical-arc support and box-constrained reach policy as
        // source spans and whole approaches. Compaction may not accumulate drift
        // away from either the immutable source or the current pair of curves.
        reason="bounded-source-fit";
        if(!NetworkSourceFit.TryFit(reference,a.CourseStart,b.CourseEnd,da,db,template.SourceFitToleranceMeters.Value,token,out var curve,out var witness))
        {
            var cursor=0;
            for(var i=0;i<spans.Count;i++)
            {
                cursor+=spans[i].SourceGeometry.Count>=2?spans[i].SourceGeometry.Count:2;
                if(witness<cursor){preferredSplit=i+1;break;}
            }
            return null;
        }
        var candidate=template with{Control1=(float3)curve.P1,Control2=(float3)curve.P2,HorizontalEmissionReference=curve};
        var rawValid=NetworkNormalizer.WithinTolerance(candidate);
        var pairValid=NetworkNormalizer.WithinTolerance(candidate with{SourceGeometry=points});
        reason=rawValid&&pairValid?"accepted":!rawValid&&!pairValid?"raw-reference-and-pair-tolerance":!rawValid?"raw-reference-tolerance":"current-pair-tolerance";
        if(!rawValid||!pairValid)return null;
        if(candidate.Kind==NetworkKind.Rail&&candidate.NativeCenterlineCurvatureLimit.HasValue&&
            NetworkDesignChecks.Certify(new(candidate.CourseStart,candidate.CourseControl1,candidate.CourseControl2,candidate.CourseEnd),
                candidate.NativeCenterlineCurvatureLimit.Value,double.PositiveInfinity,token:token,maximumLeaves:4096).Quality!=DesignQuality.Certified)
        {reason="native-train-curvature-contract";return null;}
        return candidate;
    }
}
