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
    public static IReadOnlyList<PreparedSegment> Prepare(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token,double width=double.PositiveInfinity,bool deferLinkApproaches=false)
    {
        // Reconciliation may run this a second time with the selected native
        // palette. Recompute offsets, rather than adding another lane-width shift.
        var result=source.Select(s=>s.Kind==NetworkKind.Road&&s.OneWay?s with{StartCurveOffset=default,EndCurveOffset=default,FinalDeckCurve=null}:s).ToArray();var nodes=new Dictionary<long,List<(int index,bool start)>>();
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
            if(s.NativeLaneOffsets.Count>0)
            {
                if(s.NativeLaneOffsets.Count!=s.Lanes)log.Count("Nominal road lane counts differing from native slots");
                return laneCache[s]=s.NativeLaneOffsets.OrderBy(x=>x).ToArray();
            }
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
            // Default right-hand-traffic policy: retain the travel-left lane
            // datum. An attached ramp alone is not evidence of a left-lane drop.
            var side=1f;
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
        var stationShifts=new List<NetworkApproachTapers.Shift>();
        var hasPredecessor=new HashSet<int>(joins.Select(j=>j.outgoing.index));
        var datum=new Dictionary<int,float>();
        foreach(var seed in continuation.Keys.OrderBy(i=>hasPredecessor.Contains(i)?1:0).ThenBy(i=>result[i].WayId).ThenBy(i=>result[i].StartNodeId))
        {
            if(datum.ContainsKey(seed))continue;var index=seed;var value=0f;
            while(!datum.ContainsKey(index))
            {
                token.ThrowIfCancellationRequested();datum[index]=value;
                if(!continuation.TryGetValue(index,out var join))break;
                if(Math.Abs(join.delta)>.01f)transitions++;
                value+=join.delta;index=join.outgoing.index;
            }
        }
        foreach(var item in datum)
        {
            if(Math.Abs(item.Value)<.001f)continue;var s=result[item.Key];
            var a=Heading(s,true);var b=Heading(s,false);
            // Zero station span is a constant lateral datum, with the curved
            // normal derivative still supplied by the shared materializer.
            stationShifts.Add(new(item.Key,0,0,1,item.Value));
            Offset((item.Key,true),new float3(a.y,0,-a.x)*item.Value);
            Offset((item.Key,false),new float3(b.y,0,-b.x)*item.Value);tapered++;
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
                var addedLane=rampSide>0&&(entry.start?Lanes(a).Length>Lanes(b).Length:Lanes(b).Length>Lanes(a).Length);
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
            var slots=Lanes(target);var total=branches.Sum(e=>Lanes(result[e.index]).Length);var cursor=0;
            if(total!=slots.Length){log.Count("Lane bundles requiring native lane merge");continue;}
            directions.Add(new(receiver.index,receiver.start,(double2)heading*(receiver.start?1:-1)));
            foreach(var branch in branches)
            {
                var s=result[branch.index];var count=Lanes(s).Length;var laneCenter=slots.Skip(cursor).Take(count).Average();cursor+=count;
                var branchCenter=Lanes(s).Average();var offset=normal*(laneCenter-branchCenter)+(merging?target.StartCurveOffset:target.EndCurveOffset);
                Offset(branch,offset);directions.Add(new(branch.index,branch.start,(double2)heading*(branch.start?1:-1)));
            }
            bundles++;
        }
        // A two-way road dividing into one-way carriageways: the native node
        // joins them at its centre. Each carriageway instead continues its own
        // direction's half, next to the median on its travel-right (right-hand
        // traffic), and leaves parallel to the two-way road (NetworkLaneSlots
        // does the same in the visual compiler).
        var twoWay=new Dictionary<long,List<int>>();
        for(var i=0;i<result.Length;i++)if(result[i].Kind==NetworkKind.Road&&!result[i].OneWay)
            foreach(var id in new[]{result[i].StartNodeId,result[i].EndNodeId}){if(!twoWay.TryGetValue(id,out var list))twoWay[id]=list=new();list.Add(i);}
        var divisions=0;
        foreach(var node in nodes.OrderBy(p=>p.Key))
        {
            token.ThrowIfCancellationRequested();
            var arms=node.Value;if(!twoWay.TryGetValue(node.Key,out var roads)||!arms.Any(e=>e.start)||!arms.Any(e=>!e.start))continue;
            foreach(var r in roads)
            {
                var road=result[r];var atStart=road.StartNodeId==node.Key;var anchor=atStart?road.Start:road.End;
                var into=atStart?Heading(road,true):-Heading(road,false);var forward=Heading(road,atStart);
                float2 Away((int index,bool start) e)=>e.start?Heading(result[e.index],true):-Heading(result[e.index],false);
                if(road.Bridge||arms.Any(e=>math.dot(Away(e),-into)<.82f||result[e.index].Tunnel!=road.Tunnel||result[e.index].Bridge))continue;
                var lanes=Lanes(road);var changes=new List<((int index,bool start) Port,float3 Offset)>();
                foreach(var arm in arms)
                {
                    var s=result[arm.index];var travel=Heading(s,arm.start);var normal=new float3(travel.y,0,-travel.x);
                    var side=(math.dot(travel,forward)>0?lanes.Where(x=>x>0):lanes.Where(x=>x<0).Select(x=>-x)).OrderBy(x=>x).ToArray();
                    if(side.Length==0){changes.Clear();break;}
                    var own=Lanes(s);var shift=side.Take(Math.Min(own.Length,side.Length)).Average()-own.Average();
                    var far=arm.start?s.End:s.Start;
                    if(Math.Abs(shift)>12||math.dot(far-anchor,normal)*Math.Sign(shift)<-.5f){changes.Clear();break;}
                    changes.Add((arm,normal*shift));
                }
                if(changes.Count==0)continue;
                foreach(var (port,offset) in changes){Offset(port,offset);directions.Add(new(port.index,port.start,-(double2)into));}
                divisions++;break;
            }
        }
        log.Count("Lane slots aligned where two-way roads divide",divisions);
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
        if(deferLinkApproaches)
        {
            // The paired owner has not selected its final native courses yet.
            // Do not fit a ramp against a frame that that owner may replace.
            for(var i=0;i<result.Length;i++)if(source[i].ProtectedLink)result[i]=source[i];
            directions.RemoveAll(r=>source[r.Index].ProtectedLink);
            stationShifts.RemoveAll(s=>source[s.Index].ProtectedLink);
        }
        return NetworkApproachTapers.Materialize(result,stationShifts,log,token,directions,fallback:source);
    }

    // Consume the already selected mainline courses without resetting or
    // refitting them. Only link approaches own geometry in this handoff.
    public static IReadOnlyList<PreparedSegment> MaterializeLinkApproaches(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token)
    {
        var result=source.ToArray();var directions=new List<NetworkApproachTapers.DirectionRequest>();
        var nodes=new Dictionary<long,List<(int Index,bool Start)>>();
        for(var i=0;i<source.Count;i++)if(source[i].Kind==NetworkKind.Road&&source[i].OneWay)
            foreach(var start in new[]{true,false})
            {var id=start?source[i].StartNodeId:source[i].EndNodeId;if(!nodes.TryGetValue(id,out var ports))nodes[id]=ports=new();ports.Add((i,start));}
        foreach(var node in nodes.OrderBy(p=>p.Key))
        {
            token.ThrowIfCancellationRequested();var main=node.Value.Where(p=>!source[p.Index].ProtectedLink&&source[p.Index].Classification is "motorway" or "trunk").ToArray();
            if(main.Length!=2||main.Count(p=>p.Start)!=1)continue;
            var a=source[main.First(p=>!p.Start).Index];var b=source[main.First(p=>p.Start).Index];
            var da=math.normalizesafe((a.CourseEnd-a.CourseControl2).xz);var db=math.normalizesafe((b.CourseControl1-b.CourseStart).xz);
            if(math.dot(da,db)<.9f||a.Tunnel!=b.Tunnel||a.NativeLaneOffsets.Count==0||b.NativeLaneOffsets.Count==0)continue;
            var heading=math.normalizesafe(da+db);var normal=new float3(heading.y,0,-heading.x);
            foreach(var port in node.Value.Where(p=>source[p.Index].ProtectedLink&&source[p.Index].Tunnel==a.Tunnel))
            {
                var ramp=source[port.Index];if(ramp.NativeLaneOffsets.Count==0)continue;
                var anchor=port.Start?ramp.Start:ramp.End;var far=port.Start?ramp.End:ramp.Start;
                var right=math.dot(far-anchor,normal)>=0;var target=port.Start?a:b;
                var added=right&&(port.Start?a.NativeLaneOffsets.Count>b.NativeLaneOffsets.Count:b.NativeLaneOffsets.Count>a.NativeLaneOffsets.Count);
                var direction=heading;
                if(!added)
                {
                    var angle=20f*(float)Math.PI/180;var sign=(port.Start?1:-1)*(right?1:-1);
                    direction=heading*(float)Math.Cos(angle)+normal.xz*sign*(float)Math.Sin(angle);
                }
                var offset=(port.Start?a.CourseEnd-a.End:b.CourseStart-b.Start);
                if(added)
                {
                    var lane=right?target.NativeLaneOffsets.Max():target.NativeLaneOffsets.Min();
                    var rampLane=right?ramp.NativeLaneOffsets.Min():ramp.NativeLaneOffsets.Max();
                    offset+=normal*lane-new float3(direction.y,0,-direction.x)*rampLane;
                }
                result[port.Index]=port.Start?result[port.Index] with{StartCurveOffset=offset}:result[port.Index] with{EndCurveOffset=offset};
                directions.Add(new(port.Index,port.Start,(double2)direction*(port.Start?1:-1)));
            }
        }
        if(directions.Count==0)return source;
        log.Count("Final mainline frame link requests",directions.Count);
        // Selected course offsets are boundary poses, not a second displacement
        // field to apply after fitting. One fit owns both heading and position.
        var failed=new HashSet<int>();
        var fitted=NetworkApproachFrames.Prepare(result,directions,log,token,failed,courseCoordinates:true).ToArray();
        foreach(var index in failed)fitted[index]=source[index];
        return fitted;
    }
}
