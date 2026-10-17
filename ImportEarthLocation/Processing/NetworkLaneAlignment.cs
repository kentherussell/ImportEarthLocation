using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Shared CoursePos anchors preserve native nodes; the independently supplied
// NetCourse curve supplies the lateral lane alignment. Do not offset node IDs.
public static class NetworkLaneAlignment
{
    public static IReadOnlyList<PreparedSegment> Prepare(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token,double width=double.PositiveInfinity)
    {
        // Reconciliation may run this a second time with the selected native
        // palette. Recompute offsets, rather than adding another lane-width shift.
        var result=source.Select(s=>s.Kind==NetworkKind.Road&&s.OneWay?s with{StartCurveOffset=default,EndCurveOffset=default}:s).ToArray();var nodes=new Dictionary<long,List<(int index,bool start)>>();
        for(var i=0;i<result.Length;i++)if(result[i].Kind==NetworkKind.Road&&result[i].OneWay)
            foreach(var start in new[]{true,false})
            {var id=start?result[i].StartNodeId:result[i].EndNodeId;if(!nodes.TryGetValue(id,out var list))nodes[id]=list=new();list.Add((i,start));}
        var ramps=0;var transitions=0;var tapered=0;var limited=0;
        var directions=new List<NetworkApproachTapers.DirectionRequest>();
        var laneCache=new Dictionary<PreparedSegment,float[]>();
        float[] Lanes(PreparedSegment s)
        {
            if(laneCache.TryGetValue(s,out var cached))return cached;
            var count=Math.Min(6,Math.Max(1,s.Lanes));
            if(s.NativeLaneOffsets.Count>0)return laneCache[s]=s.NativeLaneOffsets.OrderBy(x=>x).ToArray();
            // Section-local readback is not a composed lane palette. Use a
            // bounded vanilla lane-spacing estimate instead of a false centre lane.
            var spacing=4f;return laneCache[s]=Enumerable.Range(0,count).Select(i=>(i-(count-1)*.5f)*spacing).ToArray();
        }
        float2 Heading(PreparedSegment s,bool start)=>math.normalizesafe((start?s.CourseControl1-s.CourseStart:s.CourseEnd-s.CourseControl2).xz,math.normalizesafe((s.End-s.Start).xz));
        void Offset((int index,bool start) entry,float3 value)
        {var s=result[entry.index];result[entry.index]=entry.start?s with{StartCurveOffset=value}:s with{EndCurveOffset=value};}
        // Build immutable joins before applying any correction. This removes
        // source enumeration order from the lane-placement decision.
        var joins=new List<((int index,bool start) incoming,(int index,bool start) outgoing,float3 normal,(int index,bool start)[] ramps,float delta)>();
        foreach(var node in nodes.OrderBy(p=>p.Key))
        {
            var entries=node.Value;
            token.ThrowIfCancellationRequested();var main=entries.Where(e=>!result[e.index].ProtectedLink&&(result[e.index].Classification=="motorway"||result[e.index].Classification=="trunk")).ToArray();
            if(main.Length!=2)continue;
            var inputs=main.Where(e=>!e.start).ToArray();var outputs=main.Where(e=>e.start).ToArray();
            if(inputs.Length!=1||outputs.Length!=1)continue;
            var incoming=inputs[0];var outgoing=outputs[0];
            var a=result[incoming.index];var b=result[outgoing.index];var heading=Heading(a,false);
            if(math.dot(heading,Heading(b,true))<.9f||a.Tunnel!=b.Tunnel)continue;
            heading=math.normalizesafe(heading+Heading(b,true));
            var normal=new float3(heading.y,0,-heading.x);var anchor=a.End;
            var attached=entries.Where(e=>result[e.index].ProtectedLink&&result[e.index].OneWay&&result[e.index].Tunnel==a.Tunnel).ToArray();
            var side=1f;
            if(attached.Length>0)
            {
                var r=result[attached[0].index];var far=attached[0].start?r.End:r.Start;
                side=math.dot(far-anchor,normal)>=0?1:-1;
            }
            var al=Lanes(a);var bl=Lanes(b);
            // Keep the opposite through-lane edge fixed when a lane is added or
            // removed on the ramp side. Never centre a wider road on a narrower one.
            var aAnchor=side>0?al[0]:al[al.Length-1];var bAnchor=side>0?bl[0]:bl[bl.Length-1];
            if(attached.Length>0||Math.Abs(aAnchor-bAnchor)>.01)
            {directions.Add(new(incoming.index,false,-(double2)heading));directions.Add(new(outgoing.index,true,(double2)heading));}
            // Fix the continuing lane edge, not the centers of different-width
            // roads. Propagate this deck offset over the approach, never node IDs.
            joins.Add((incoming,outgoing,normal,attached,aAnchor-bAnchor));
        }
        var continuation=joins.ToDictionary(j=>j.incoming.index,j=>j);
        const float taperLength=220;
        var referenceLengths=result.Select(s=>NetworkVerticalProfiles.ArcLength(s)).ToArray();
        var stationShifts=new List<NetworkApproachTapers.Shift>();
        foreach(var join in joins)
        {
            token.ThrowIfCancellationRequested();if(Math.Abs(join.delta)<.01f)continue;
            transitions++;var index=join.outgoing.index;var distance=0f;var startNormal=join.normal;var visited=new HashSet<int>();
            var path=new List<(int index,float length)>();var available=0f;
            var requestedReach=(float)Math.Max(taperLength,1.875*Math.Abs(join.delta)/Math.Tan(Math.PI/90));
            while(available<requestedReach&&visited.Add(index))
            {
                token.ThrowIfCancellationRequested();var length=referenceLengths[index];path.Add((index,length));available+=length;
                if(!continuation.TryGetValue(index,out var next))break;index=next.outgoing.index;
            }
            var reach=Math.Max(.01f,Math.Min(requestedReach,available));
            float Weight(float d)=>(float)NetworkDesignChecks.QuinticShift(1,d,reach).Value;
            foreach(var step in path)
            {
                index=step.index;var s=result[index];var length=step.length;var endHeading=Heading(s,false);var endNormal=new float3(endHeading.y,0,-endHeading.x);
                stationShifts.Add(new(index,distance,length,reach,join.delta,result[join.incoming.index].EndNodeId));
                Offset((index,true),s.StartCurveOffset+startNormal*(join.delta*Weight(distance)));
                Offset((index,false),s.EndCurveOffset+endNormal*(join.delta*Weight(distance+length)));
                distance+=length;tapered++;
                // A valid terminal or an ambiguous fork is a stop, not an error
                // or a reason to search endlessly for a continuous highway.
                startNormal=endNormal;
            }
        }
        // A large correction is review evidence, not permission to independently
        // clamp adjacent ports and introduce another kink. Reach grows with shift.
        for(var i=0;i<result.Length;i++)
        {
            var s=result[i];var maximum=Math.Max(math.length(s.StartCurveOffset),math.length(s.EndCurveOffset));
            if(maximum>12)limited++;
        }
        foreach(var join in joins)
        {
                var incoming=join.incoming;var outgoing=join.outgoing;var a=result[incoming.index];var b=result[outgoing.index];var normal=join.normal;var anchor=a.End;var heading=new float2(-normal.z,normal.x);
            foreach(var entry in join.ramps)
            {
                var ramp=result[entry.index];var far=entry.start?ramp.End:ramp.Start;var rampSide=math.dot(far-anchor,normal)>=0?1:-1;
                var target=entry.start?a:b;var targetOffsets=Lanes(target);var lane=rampSide>0?targetOffsets[targetOffsets.Length-1]:targetOffsets[0];
                var targetOffset=entry.start?a.EndCurveOffset:result[outgoing.index].StartCurveOffset;
                var rampOffsets=Lanes(ramp);var rampLane=rampSide>0?rampOffsets[0]:rampOffsets[rampOffsets.Length-1];
                // Ramp local lane coordinates rotate with its own approach.
                // Subtracting both offsets in the mainline frame picks the wrong
                // receiving lane when the ramp enters at a substantial angle.
                var addedLane=entry.start?a.Lanes>b.Lanes:b.Lanes>a.Lanes;
                // Without an added receiving lane, use a real merging junction,
                // not almost coincident decks with crossing lane markings.
                var approach=heading;
                if(!addedLane)
                {
                    // Preserve the selected 20-degree no-added-lane contract,
                    // but rotate the whole approach rather than one short handle.
                    var angle=20f*(float)Math.PI/180;
                    var sign=entry.start?rampSide:-rampSide;
                    approach=heading*(float)Math.Cos(angle)+new float2(normal.x,normal.z)*sign*(float)Math.Sin(angle);
                }
                var rampNormal=new float3(approach.y,0,-approach.x);
                var offset=addedLane?targetOffset+normal*lane-rampNormal*rampLane:targetOffset;
                Offset(entry,offset);directions.Add(new(entry.index,entry.start,(double2)approach*(entry.start?1:-1)));ramps++;
            }
        }
        var bundles=0;
        foreach(var node in nodes)
        {
            token.ThrowIfCancellationRequested();
            var main=node.Value.Where(e=>!result[e.index].ProtectedLink&&result[e.index].Classification is "motorway" or "trunk").ToArray();
            var inputs=main.Where(e=>!e.start).ToArray();var outputs=main.Where(e=>e.start).ToArray();
            var merging=inputs.Length>=2&&outputs.Length==1;var splitting=outputs.Length>=2&&inputs.Length==1;
            if(!merging&&!splitting)continue;
            var receiver=merging?outputs[0]:inputs[0];var target=result[receiver.index];
            var heading=Heading(target,merging);var normal=new float3(heading.y,0,-heading.x);
            var branchPorts=merging?inputs:outputs;var anchor=merging?target.Start:target.End;
            var reach=Math.Max(30,2*node.Value.Max(e=>result[e.index].NativeWidth));
            float Across((int index,bool start) port,float distance)
            {
                var index=port.index;var start=port.start;var seen=new HashSet<int>();var point=start?source[index].Start:source[index].End;
                while(seen.Add(index))
                {
                    var section=source[index];var previous=start?section.CourseStart:section.CourseEnd;
                    for(var j=1;j<=32;j++)
                    {
                        var next=NetworkNormalizer.Evaluate(section,start?j/32f:1-j/32f);var span=math.distance(previous.xz,next.xz);
                        if(span>=distance){point=math.lerp(previous,next,span>0?distance/span:0);return math.dot(point-anchor,normal);}
                        distance-=span;previous=next;point=next;
                    }
                    var id=start?section.EndNodeId:section.StartNodeId;
                    if(!nodes.TryGetValue(id,out var arms)||arms.Count!=2)break;
                    var continuation=arms.First(e=>e.index!=index);
                    if(continuation.start!=start||source[continuation.index].ProtectedLink!=section.ProtectedLink)break;
                    index=continuation.index;start=continuation.start;
                }
                return math.dot(point-anchor,normal);
            }
            var ordered=branchPorts.Select(e=>(Port:e,Half:Across(e,reach*.5f),Full:Across(e,reach),Far:Across(e,reach*2)))
                .OrderBy(p=>p.Full).ThenBy(p=>source[p.Port.index].WayId).ToArray();
            var ambiguous=false;
            for(var j=1;j<ordered.Length;j++)
                if(ordered[j].Full-ordered[j-1].Full<=.001f||ordered[j].Half-ordered[j-1].Half<=.001f||ordered[j].Far-ordered[j-1].Far<=.001f)ambiguous=true;
            if(ambiguous){log.Count("Lane bundles with ambiguous immutable branch order");continue;}
            var branches=ordered.Select(p=>p.Port).ToArray();
            var slots=Lanes(target);var total=branches.Sum(e=>Math.Max(1,result[e.index].Lanes));var cursor=0;
            if(total!=slots.Length){log.Count("Lane bundles requiring native lane merge");continue;}
            directions.Add(new(receiver.index,receiver.start,(double2)heading*(receiver.start?1:-1)));
            foreach(var branch in branches)
            {
                var s=result[branch.index];var count=Math.Max(1,s.Lanes);var laneCenter=slots.Skip(cursor).Take(count).Average();cursor+=count;
                var branchCenter=Lanes(s).Average();var offset=normal*(laneCenter-branchCenter)+(merging?target.StartCurveOffset:target.EndCurveOffset);
                Offset(branch,offset);directions.Add(new(branch.index,branch.start,(double2)heading*(branch.start?1:-1)));
            }
            bundles++;
        }
        log.Count("Highway lane bundles aligned",bundles);
        log.Diagnostic($"LANE BUNDLES junctions={bundles}; laneOrder=travel-frame-left-to-right; separateNodeIdentitiesCreated=false; noAddedLaneMergeAngleDegrees=20; appliedBeforeVerticalSolve=true");
        // Offset handles and ramp tangents cannot extend past a clipped map end.
        // Roll back the whole affected join, not one incident curve endpoint:
        // otherwise the guard itself could introduce another misaligned join.
        var half=(float)(width/2);var boundaryNodes=new HashSet<long>();
        bool Inside(float3 p)=>math.all(math.isfinite(p))&&Math.Abs(p.x)<=half&&Math.Abs(p.z)<=half;
        for(var i=0;i<result.Length;i++)
        {
            var s=result[i];if(s.Kind!=NetworkKind.Road||!s.OneWay)continue;
            if(!Inside(s.CourseStart)||!Inside(s.CourseControl1))boundaryNodes.Add(s.StartNodeId);
            if(!Inside(s.CourseEnd)||!Inside(s.CourseControl2))boundaryNodes.Add(s.EndNodeId);
        }
        for(var i=0;i<result.Length;i++)
        {
            var s=result[i];if(s.Kind!=NetworkKind.Road||!s.OneWay)continue;
            if(boundaryNodes.Contains(s.StartNodeId))s=s with{StartCurveOffset=default,Control1=source[i].Control1};
            if(boundaryNodes.Contains(s.EndNodeId))s=s with{EndCurveOffset=default,Control2=source[i].Control2};
            result[i]=s;
        }
        if(boundaryNodes.Count>0)log.Warn($"Lane alignment constrained at {boundaryNodes.Count} map-boundary joins. Shared connections preserved; the affected lane correction was rolled back, not the road.");
        if(boundaryNodes.Count>0)stationShifts.RemoveAll(s=>boundaryNodes.Contains(result[s.Index].StartNodeId)||boundaryNodes.Contains(result[s.Index].EndNodeId));
        log.Diagnostic($"LANE ALIGNMENT ramps={ramps}; widthTransitions={transitions}; taperedSections={tapered}; minimumTaperMeters={taperLength}; largeOffsetsRequiringReview={limited}; boundaryJoinsRolledBack={boundaryNodes.Count}; curveEndpointsOffset=true; exactSharedNodeAnchorsPreserved=true; nativeMeshRequiresVerification=true");
        directions.RemoveAll(r=>boundaryNodes.Contains(r.Start?result[r.Index].StartNodeId:result[r.Index].EndNodeId));
        return NetworkApproachTapers.Materialize(result,stationShifts,log,token,directions,fallback:source);
    }
}
