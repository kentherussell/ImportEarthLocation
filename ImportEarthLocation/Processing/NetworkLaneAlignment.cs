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
        float2 Heading(PreparedSegment s,bool start)=>math.normalizesafe((start?s.CourseControl1-s.CourseStart:s.CourseEnd-s.CourseControl2).xz);
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
            // Fix the continuing lane edge, not the centers of different-width
            // roads. Propagate this deck offset over the approach, never node IDs.
            joins.Add((incoming,outgoing,normal,attached,aAnchor-bAnchor));
        }
        var continuation=joins.ToDictionary(j=>j.incoming.index,j=>j);
        const float taperLength=220;
        foreach(var join in joins)
        {
            token.ThrowIfCancellationRequested();if(Math.Abs(join.delta)<.01f)continue;
            transitions++;var index=join.outgoing.index;var distance=0f;var startNormal=join.normal;var visited=new HashSet<int>();
            var path=new List<(int index,float length)>();var available=0f;
            while(available<taperLength&&visited.Add(index)&&path.Count<64)
            {
                token.ThrowIfCancellationRequested();var length=NetworkVerticalProfiles.ArcLength(result[index]);path.Add((index,length));available+=length;
                if(!continuation.TryGetValue(index,out var next))break;index=next.outgoing.index;
            }
            var reach=Math.Max(.01f,Math.Min(taperLength,available));
            float Weight(float d){var t=math.clamp(d/reach,0,1);return 1-t*t*(3-2*t);}
            foreach(var step in path)
            {
                index=step.index;var s=result[index];var length=step.length;var endHeading=Heading(s,false);var endNormal=new float3(endHeading.y,0,-endHeading.x);
                Offset((index,true),s.StartCurveOffset+startNormal*(join.delta*Weight(distance)));
                Offset((index,false),s.EndCurveOffset+endNormal*(join.delta*Weight(distance+length)));
                distance+=length;tapered++;
                // A valid terminal or an ambiguous fork is a stop, not an error
                // or a reason to search endlessly for a continuous highway.
                startNormal=endNormal;
            }
        }
        // Guard only pathological stacks of width changes, preserving shared
        // corrections wherever normal native lane geometry stays within budget.
        for(var i=0;i<result.Length;i++)
        {
            var s=result[i];var maximum=Math.Max(math.length(s.StartCurveOffset),math.length(s.EndCurveOffset));
            float3 Bounded(float3 offset)=>math.length(offset)>12?offset*(12/math.length(offset)):offset;
            if(maximum>12){limited++;result[i]=s with{StartCurveOffset=Bounded(s.StartCurveOffset),EndCurveOffset=Bounded(s.EndCurveOffset)};}
        }
        foreach(var join in joins)
        {
            var incoming=join.incoming;var outgoing=join.outgoing;var a=result[incoming.index];var b=result[outgoing.index];var normal=join.normal;var anchor=a.End;var heading=Heading(a,false);
            foreach(var entry in join.ramps)
            {
                var ramp=result[entry.index];var far=entry.start?ramp.End:ramp.Start;var rampSide=math.dot(far-anchor,normal)>=0?1:-1;
                var target=entry.start?a:b;var targetOffsets=Lanes(target);var lane=rampSide>0?targetOffsets[targetOffsets.Length-1]:targetOffsets[0];
                var targetOffset=entry.start?a.EndCurveOffset:result[outgoing.index].StartCurveOffset;
                var rampOffsets=Lanes(ramp);var rampLane=rampSide>0?rampOffsets[0]:rampOffsets[rampOffsets.Length-1];
                // Ramp local lane coordinates rotate with its own approach.
                // Subtracting both offsets in the mainline frame picks the wrong
                // receiving lane when the ramp enters at a substantial angle.
                var rampHeading=Heading(ramp,entry.start);var rampNormal=new float3(rampHeading.y,0,-rampHeading.x);
                var addedLane=entry.start?a.Lanes>b.Lanes:b.Lanes>a.Lanes;
                // Without an added receiving lane, use a real merging junction,
                // not almost coincident decks with crossing lane markings.
                var offset=addedLane?targetOffset+normal*lane-rampNormal*rampLane:targetOffset;
                if(math.length(offset)>12)offset=math.normalizesafe(offset)*12;
                var control=entry.start?ramp.Control1??math.lerp(ramp.Start,ramp.End,1f/3):ramp.Control2??math.lerp(ramp.Start,ramp.End,2f/3);
                var approach=heading;
                if(!addedLane)
                {
                    const float angle=.34906585f; // 20-degree terminal, curved upstream.
                    var sign=entry.start?rampSide:-rampSide;
                    approach=heading*(float)Math.Cos(angle)+new float2(normal.x,normal.z)*sign*(float)Math.Sin(angle);
                }
                var reach=Math.Min(math.distance(ramp.Start.xz,ramp.End.xz)*.4f,120);
                var aligned=anchor+new float3(approach.x,0,approach.y)*reach*(entry.start?1:-1);
                var mainGrade=(target.End.y-target.Start.y)/Math.Max(1,math.distance(target.Start.xz,target.End.xz));
                aligned.y=anchor.y+mainGrade*reach*(entry.start?1:-1);
                var farHeading=math.normalizesafe((ramp.End-ramp.Start).xz*.75f+approach*.25f);var farReach=math.distance(ramp.Start.xz,ramp.End.xz)/3;
                var farControl=(entry.start?ramp.End:ramp.Start)+new float3(farHeading.x,0,farHeading.y)*farReach*(entry.start?-1:1);
                farControl.y=math.lerp(ramp.Start.y,ramp.End.y,entry.start?2f/3:1f/3);
                result[entry.index]=entry.start?ramp with{Control1=aligned,Control2=farControl,StartCurveOffset=offset}:ramp with{Control1=farControl,Control2=aligned,EndCurveOffset=offset};ramps++;
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
            var branches=(merging?inputs:outputs).OrderBy(e=>math.dot((e.start?result[e.index].End:result[e.index].Start)-target.Start,normal)).ToArray();
            var slots=Lanes(target);var total=branches.Sum(e=>Math.Max(1,result[e.index].Lanes));var cursor=0;
            if(total!=slots.Length){log.Count("Lane bundles requiring native lane merge");continue;}
            foreach(var branch in branches)
            {
                var s=result[branch.index];var count=Math.Max(1,s.Lanes);var laneCenter=slots.Skip(cursor).Take(count).Average();cursor+=count;
                var branchCenter=Lanes(s).Average();var offset=normal*(laneCenter-branchCenter)+(merging?target.StartCurveOffset:target.EndCurveOffset);
                Offset(branch,offset);var p=branch.start?s.Start:s.End;var reach=Math.Min(180,math.distance(s.Start.xz,s.End.xz)*.4f);
                var control=p+new float3(heading.x,0,heading.y)*reach*(branch.start?1:-1);
                var grade=(target.End.y-target.Start.y)/Math.Max(1,math.distance(target.Start.xz,target.End.xz));control.y=p.y+grade*reach*(branch.start?1:-1);
                var revised=result[branch.index];result[branch.index]=branch.start?revised with{Control1=control}:revised with{Control2=control};
            }
            bundles++;
        }
        log.Count("Highway lane bundles aligned",bundles);
        log.Diagnostic($"LANE BUNDLES junctions={bundles}; laneOrder=travel-frame-left-to-right; separateNodeIdentitiesCreated=false; noAddedLaneMergeAngleDegrees=20");
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
        // The intentional lateral correction becomes the grading reference.
        for(var i=0;i<result.Length;i++)if(math.lengthsq(result[i].StartCurveOffset)+math.lengthsq(result[i].EndCurveOffset)>0)
            result[i]=result[i] with{SourceGeometry=Enumerable.Range(0,65).Select(j=>NetworkNormalizer.Evaluate(result[i],j/64f)).ToArray(),HorizontalParent=null};
        log.Diagnostic($"LANE ALIGNMENT ramps={ramps}; widthTransitions={transitions}; taperedSections={tapered}; taperMeters={taperLength}; boundedOffsetSections={limited}; boundaryJoinsRolledBack={boundaryNodes.Count}; curveEndpointsOffset=true; exactSharedNodeAnchorsPreserved=true; nativeMeshRequiresVerification=true");
        return result;
    }
}
