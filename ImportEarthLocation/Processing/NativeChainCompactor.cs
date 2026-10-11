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
                a.EndTerminal||b.StartTerminal||a.EndRoundaboutRadius>0||b.StartRoundaboutRadius>0){Reject("loop-anchor-or-marker");continue;}
            var candidate=Fit(a,b,out var reason);
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
        log.Diagnostic("NATIVE CHAIN REJECTIONS "+string.Join("; ",rejected.OrderBy(p=>p.Key).Select(p=>$"{p.Key}={p.Value}"))+"; countsAreCandidateVisits=true; boundedFitIterations=5; toleranceRelaxed=false");
        return work.Where(s=>s!=null).Select(s=>s!).ToArray();
    }

    private static bool Compatible(PreparedSegment a,PreparedSegment b)=>
        !a.Bridge&&!b.Bridge&&!a.Tunnel&&!b.Tunnel&&a.Kind==b.Kind&&a.Classification==b.Classification&&a.Roundabout==b.Roundabout&&
        a.OneWay==b.OneWay&&a.Lanes==b.Lanes&&a.Layer==b.Layer&&a.RailService==b.RailService&&a.PhysicalTracks==b.PhysicalTracks&&
        a.ForwardLanes==b.ForwardLanes&&a.BackwardLanes==b.BackwardLanes&&a.BothWaysLanes==b.BothWaysLanes&&
        a.ExplicitIncline==b.ExplicitIncline&&a.SourceSpeedKph==b.SourceSpeedKph&&a.SourceWidthMeters==b.SourceWidthMeters&&
        a.NativeWidth==b.NativeWidth&&a.NativeLaneOffsets.SequenceEqual(b.NativeLaneOffsets)&&
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

    private static PreparedSegment? Fit(PreparedSegment a,PreparedSegment b,out string reason)
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
            SourceGeometry=reference,OriginalGeometry=original,HorizontalParent=null,ParentStartT=0,ParentEndT=1};
        var parameters=lengths.Select(length=>length/total).ToArray();
        // Arc length is only an initial parameter guess, not cubic time. Refine
        // that guess rather than relax the geographic or old-curve tolerances.
        // A fixed iteration count bounds work; outer tangent directions stay fixed.
        for(var iteration=0;iteration<5;iteration++)
        {
            double aa=0,ab=0,bb=0,ar=0,br=0;
            for(var i=1;i<points.Count-1;i++)
            {
                var t=parameters[i];var u=1-t;var x=3*u*u*t;var y=3*u*t*t;
                var p=points[i].xz-((u*u*u+x)*a.Start.xz+(y+t*t*t)*b.End.xz);
                var va=x*da;var vb=-y*db;aa+=math.dot(va,va);ab+=math.dot(va,vb);bb+=math.dot(vb,vb);ar+=math.dot(va,p);br+=math.dot(vb,p);
            }
            var determinant=aa*bb-ab*ab;reason="singular-handle-fit";
            if(determinant<1e-9)
            {
                // Collinear tangents make the two-handle least-squares system
                // singular, not the road invalid. Try the bounded chord fit.
                var straight=template with{Control1=math.lerp(a.Start,b.End,1f/3),Control2=math.lerp(a.Start,b.End,2f/3)};
                if(NetworkNormalizer.WithinTolerance(straight)&&NetworkNormalizer.WithinTolerance(straight with{SourceGeometry=points}))
                { reason="accepted-collinear";return straight; }
                return null;
            }
            var ha=(float)((ar*bb-br*ab)/determinant);var hb=(float)((br*aa-ar*ab)/determinant);
            reason="handle-length-bound";
            if(!math.isfinite(ha)||!math.isfinite(hb)||ha<0||hb<0||ha>total||hb>total)return null;
            var c1=new float3(a.Start.x+da.x*ha,math.lerp(a.Start.y,b.End.y,1f/3),a.Start.z+da.y*ha);
            var c2=new float3(b.End.x-db.x*hb,math.lerp(a.Start.y,b.End.y,2f/3),b.End.z-db.y*hb);
            var candidate=template with{Control1=c1,Control2=c2};
            // Accumulated reference prevents progressive fitting drift.
            var rawValid=NetworkNormalizer.WithinTolerance(candidate);
            var pairValid=NetworkNormalizer.WithinTolerance(candidate with{SourceGeometry=points});
            if(rawValid&&pairValid){reason="accepted";return candidate;}
            reason=!rawValid&&!pairValid?"raw-reference-and-pair-tolerance":!rawValid?"raw-reference-tolerance":"current-pair-tolerance";
            for(var i=1;i<points.Count-1;i++)
            {
                var t=parameters[i];var u=1-t;var delta=(NetworkNormalizer.Evaluate(candidate,t)-points[i]).xz;
                var derivative=(3*u*u*(c1-a.Start)+6*u*t*(c2-c1)+3*t*t*(b.End-c2)).xz;
                var second=(6*u*(c2-2*c1+a.Start)+6*t*(b.End-2*c2+c1)).xz;
                var denominator=math.dot(derivative,derivative)+math.dot(delta,second);
                if(denominator>1e-6f)
                {
                    var next=math.clamp(t-math.dot(delta,derivative)/denominator,0,1);
                    if(math.isfinite(next))parameters[i]=next;
                }
            }
            for(var i=1;i<parameters.Length;i++)parameters[i]=math.max(parameters[i],parameters[i-1]);
        }
        return null;
    }
}
