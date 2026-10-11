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
            if(math.dot(heading,Heading(b,true))<.97f||a.Tunnel!=b.Tunnel)continue;
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
            joins.Add((incoming,outgoing,normal,attached,aAnchor-bAnchor));
        }
        var continuation=joins.ToDictionary(j=>j.incoming.index,j=>j);
        const float taperLength=140;
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
                var offset=targetOffset+normal*(lane-rampLane);if(math.length(offset)>12)continue;
                var control=entry.start?ramp.Control1??math.lerp(ramp.Start,ramp.End,1f/3):ramp.Control2??math.lerp(ramp.Start,ramp.End,2f/3);
                var reach=Math.Min(60,math.distance(control.xz,anchor.xz));var aligned=anchor+new float3(heading.x,0,heading.y)*reach*(entry.start?1:-1);aligned.y=control.y;
                result[entry.index]=entry.start?ramp with{Control1=aligned,StartCurveOffset=offset}:ramp with{Control2=aligned,EndCurveOffset=offset};ramps++;
            }
        }
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
