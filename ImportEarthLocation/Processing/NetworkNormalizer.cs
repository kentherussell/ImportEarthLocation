using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

public static class NetworkNormalizer
{
    // Run after all consolidation/roundabout fitting: distinct OSM ways can
    // become the same native course only after those transformations.
    public static IReadOnlyList<PreparedSegment> RemoveCoincidentCourses(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token)
    {
        var result=new List<PreparedSegment>();var buckets=new Dictionary<string,List<int>>();
        foreach(var s in source)
        {
            token.ThrowIfCancellationRequested();
            var a=s.OneWay?s.StartNodeId:Math.Min(s.StartNodeId,s.EndNodeId);
            var b=s.OneWay?s.EndNodeId:Math.Max(s.StartNodeId,s.EndNodeId);
            var family=s.Classification is "motorway" or "motorway_link" or "trunk" or "trunk_link"?"highway":s.Kind==NetworkKind.Rail?s.Classification:"street";
            var key=$"{s.Kind}:{family}:{s.Lanes}:{s.OneWay}:{s.Layer}:{s.Bridge}:{s.Tunnel}:{a}:{b}";
            if(!buckets.TryGetValue(key,out var candidates))buckets[key]=candidates=new();
            var match=-1;
            foreach(var index in candidates)
            {
                var other=result[index];var reversed=s.StartNodeId!=other.StartNodeId;
                if(Math.Abs(s.NativeWidth-other.NativeWidth)>.05f)continue;
                if(Enumerable.Range(0,33).All(j=>math.distance(Evaluate(s,j/32f),Evaluate(other,(reversed?32-j:j)/32f))<.5f))
                {match=index;break;}
            }
            if(match<0){candidates.Add(result.Count);result.Add(s);continue;}
            var old=result[match];var keeper=s.Roundabout&&!old.Roundabout?s:old;
            result[match]=keeper with{SourceWayIds=old.SourceWayIds.Concat(s.SourceWayIds).Append(old.WayId).Append(s.WayId).Distinct().ToArray()};
            log.Count("Final coincident courses coalesced");
            log.Diagnostic($"NETWORK COINCIDENT COURSE retainedWay={keeper.WayId}; aliasWay={s.WayId}; ports={a}/{b}; toleranceMeters=0.5; sourceAliasesRetained=true");
        }
        return result;
    }

    public static float Tolerance(NetworkKind kind) => kind == NetworkKind.Rail ? 3f : 2f;
    // Native placement shares a real source node through bridge/tunnel portals.
    // Count all incident same-kind edges, including other structure groups, so
    // merging a ground continuation cannot erase a structural branch junction.
    private static string NodeKey(PreparedSegment segment, long node) => $"{segment.Kind}:{node}";

    public static IReadOnlyList<PreparedSegment> Normalize(IReadOnlyList<PreparedSegment> input, ImportLog log, CancellationToken token, double mapWidth = double.PositiveInfinity)
    {
        input=NormalizeLaneRuns(input,log,token);
        var clock = Stopwatch.StartNew();
        log.Diagnostic($"PROCESS normalization begin; sourceSections={input.Count}; junctions/sharp-turns/levels protected=true");
        var degree = new Dictionary<string, int>();
        foreach (var segment in input)
        {
            foreach (var id in new[] { segment.StartNodeId, segment.EndNodeId })
            { var key = NodeKey(segment, id); degree.TryGetValue(key, out var count); degree[key] = count + 1; }
        }
        foreach(var segment in input)
        {
            if(segment.StartMiniRoundabout)degree[NodeKey(segment,segment.StartNodeId)]+=100;
            if(segment.EndMiniRoundabout)degree[NodeKey(segment,segment.EndNodeId)]+=100;
        }
        log.Time("Topology/cleanup", clock.Elapsed.TotalSeconds);
        clock.Restart();
        var output = new List<PreparedSegment>();
        // OSM way boundaries are metadata boundaries, not mandatory native nodes.
        // Merge compatible degree-two continuations, retaining every real junction.
        var processed = 0; var nextProgress = 5d;
        foreach (var group in input.GroupBy(segment => (Route:segment.Reference,
            segment.Kind, segment.Classification, segment.Lanes, segment.OneWay, segment.Bridge, segment.Tunnel, segment.Layer,
            segment.Roundabout,segment.ExplicitCutSides,segment.ExplicitWallSides,segment.RailService,segment.ForwardLanes,segment.BackwardLanes)))
        {
            processed += group.Count();
            // RoundaboutPlanner has already fitted approved circular arcs. Endpoint-only
            // refitting would erase their handles and turn the ring into a polygon.
            if(group.First().Roundabout&&group.All(s=>s.Control1.HasValue&&s.Control2.HasValue))
            {output.AddRange(group);continue;}
            if (clock.Elapsed.TotalSeconds >= nextProgress)
            { log.Diagnostic($"PROCESS curve-fitting sourceSections={processed}/{input.Count}; fittedSections={output.Count}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.0}"); nextProgress = clock.Elapsed.TotalSeconds + 5; }
            // A ramp is protected topology, not necessarily a sequence of straight edges.
            // Retain tiny links; longer chains use the same junction-aware bounded fit.
            if (group.First().ProtectedLink && group.Sum(s => math.distance(s.Start.xz, s.End.xz)) < 12)
            {
                foreach (var segment in group) { token.ThrowIfCancellationRequested(); output.Add(segment with { SourceGeometry = new[] { segment.Start, segment.End } }); }
                log.Count("Protected ramps preserved", group.Count());
                continue;
            }
            var chain = new List<PreparedSegment>();
            void Flush()
            {
                if (chain.Count == 0) return;
                var piece = new List<PreparedSegment>(); var length = 0f;
                void Emit()
                {
                    if (piece.Count == 0) return;
                    var points = new List<float3> { piece[0].Start };
                    points.AddRange(piece.Select(segment => segment.End));
                    Fit(points, piece, output, token, (float)(mapWidth / 2)); piece.Clear(); length = 0;
                }
                foreach (var segment in chain)
                {
                    if (piece.Count > 0)
                    {
                        var previous = piece[piece.Count - 1];
                        if (degree[NodeKey(segment, segment.StartNodeId)] != 2 ||
                            math.dot(math.normalizesafe(previous.End - previous.Start), math.normalizesafe(segment.End - segment.Start)) < (segment.Kind==NetworkKind.Rail||segment.Classification=="motorway"||segment.Classification=="trunk"?.5f:.707f) ||
                            length > (segment.Kind==NetworkKind.Rail||segment.Classification=="motorway"||segment.Classification=="trunk"?400f:200f) || piece.Count >= 256) Emit();
                    }
                    piece.Add(segment); length += math.distance(segment.Start, segment.End);
                }
                Emit();
                chain.Clear();
            }
            foreach (var segment in OrderSections(group,degree))
            {
                token.ThrowIfCancellationRequested();
                if (chain.Count > 0 && chain[0].StartNodeId == segment.EndNodeId && segment.OneWay)
                    chain.Insert(0, segment);
                else if (chain.Count == 0 || chain[chain.Count - 1].EndNodeId == segment.StartNodeId)
                    chain.Add(segment);
                else { Flush(); chain.Add(segment); }
            }
            Flush();
        }
        log.Count("Segments merged", Math.Max(0, input.Count - output.Count));
        var unique = new List<PreparedSegment>();
        var equivalents = new Dictionary<string, List<PreparedSegment>>(StringComparer.Ordinal);
        foreach (var segment in output)
        {
            token.ThrowIfCancellationRequested();
            var a = segment.OneWay ? segment.StartNodeId : Math.Min(segment.StartNodeId, segment.EndNodeId);
            var b = segment.OneWay ? segment.EndNodeId : Math.Max(segment.StartNodeId, segment.EndNodeId);
            var key = $"{segment.Kind}:{segment.Classification}:{segment.Lanes}:{segment.OneWay}:{segment.Bridge}:{segment.Tunnel}:{segment.Layer}:{a}:{b}";
            if (!equivalents.TryGetValue(key, out var candidates)) equivalents[key] = candidates = new List<PreparedSegment>();
            // Only equivalent topology endpoints AND nearly coincident curves qualify.
            // Parallel tracks/carriageways with different OSM node IDs never qualify.
            var duplicate = candidates.Any(other => Enumerable.Range(0, 33).All(i =>
                math.distance(Evaluate(segment, i / 32f).xz, Evaluate(other, (segment.StartNodeId == other.StartNodeId ? i : 32 - i) / 32f).xz) < 0.05f));
            if (duplicate) { log.Count("Duplicates/artifacts removed"); continue; }
            candidates.Add(segment); unique.Add(segment);
        }
        log.Time("Smoothing", clock.Elapsed.TotalSeconds);
        SmoothJoins(unique, log, token, (float)(mapWidth / 2));
        log.Diagnostic($"PROCESS normalization complete; sourceSections={input.Count}; outputSections={unique.Count}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.0}");
        return unique;
    }

    private static IEnumerable<PreparedSegment> OrderSections(IEnumerable<PreparedSegment> source,Dictionary<string,int> degree)
    {
        var items=source.ToArray();var remaining=new HashSet<int>(Enumerable.Range(0,items.Length));
        var nodes=new Dictionary<string,List<int>>();
        for(var i=0;i<items.Length;i++)foreach(var id in new[]{items[i].StartNodeId,items[i].EndNodeId})
        {var key=NodeKey(items[i],id);if(!nodes.TryGetValue(key,out var list))nodes[key]=list=new();list.Add(i);}
        while(remaining.Count>0)
        {
            var seed=remaining.Min();remaining.Remove(seed);var chain=new LinkedList<PreparedSegment>();chain.AddLast(items[seed]);
            foreach(var prepend in new[]{false,true})while(true)
            {
                var tip=prepend?chain.First!.Value:chain.Last!.Value;var id=prepend?tip.StartNodeId:tip.EndNodeId;var key=NodeKey(tip,id);
                if(degree[key]!=2)break;
                var candidates=nodes[key].Where(remaining.Contains).ToArray();if(candidates.Length!=1)break;
                var index=candidates[0];var s=items[index];
                var matches=prepend?s.EndNodeId==id:s.StartNodeId==id;
                if(!matches){if(s.OneWay)break;s=Reverse(s);}
                remaining.Remove(index);if(prepend)chain.AddFirst(s);else chain.AddLast(s);
            }
            foreach(var item in chain)yield return item;
        }
    }

    public static PreparedSegment Reverse(PreparedSegment s)
    {
        NetworkSides Swap(NetworkSides value)=>((value&NetworkSides.Left)!=0?NetworkSides.Right:0)|((value&NetworkSides.Right)!=0?NetworkSides.Left:0);
        return s with{StartNodeId=s.EndNodeId,EndNodeId=s.StartNodeId,Start=s.End,End=s.Start,Control1=s.Control2,Control2=s.Control1,
            StartCurveOffset=s.EndCurveOffset,EndCurveOffset=s.StartCurveOffset,StartElevation=s.EndElevation,EndElevation=s.StartElevation,
            ForwardLanes=s.BackwardLanes,BackwardLanes=s.ForwardLanes,ExplicitIncline=s.ExplicitIncline.HasValue?-s.ExplicitIncline.Value:null,
            ExplicitCutSides=Swap(s.ExplicitCutSides),ExplicitWallSides=Swap(s.ExplicitWallSides),RetainingWallSides=Swap(s.RetainingWallSides),
            StartRoundaboutRadius=s.EndRoundaboutRadius,EndRoundaboutRadius=s.StartRoundaboutRadius,
            StartMiniRoundabout=s.EndMiniRoundabout,EndMiniRoundabout=s.StartMiniRoundabout,StartTerminal=s.EndTerminal,EndTerminal=s.StartTerminal,
            OriginalGeometry=s.OriginalGeometry.Reverse().ToArray(),SourceGeometry=s.SourceGeometry.Reverse().ToArray(),HorizontalParent=null};
    }

    private static void SmoothJoins(List<PreparedSegment> sections, ImportLog log, CancellationToken token, float halfWidth)
    {
        var endpoints = new Dictionary<string, List<(int index, bool start)>>();
        for(var i=0;i<sections.Count;i++)
        {
            var s=sections[i];
            foreach(var start in new[]{true,false})
            {
                var node=start?s.StartNodeId:s.EndNodeId;
                var key=NodeKey(s,node);if(!endpoints.TryGetValue(key,out var entries))endpoints[key]=entries=new List<(int,bool)>();entries.Add((i,start));
            }
        }
        var clock=Stopwatch.StartNew();var nextProgress=5d;var examined=0;var smoothed=0;
        foreach(var entries in endpoints.Values)
        {
            token.ThrowIfCancellationRequested();examined++;
            if(clock.Elapsed.TotalSeconds>=nextProgress){log.Diagnostic($"PROCESS tangent-continuity nodes={examined}/{endpoints.Count}; smoothed={smoothed}");nextProgress=clock.Elapsed.TotalSeconds+5;}
            if(entries.Count!=2||entries[0].index==entries[1].index)continue;
            var left=entries[0];var right=entries[1];var a=sections[left.index];var b=sections[right.index];
            var sameRoute=a.WayId==b.WayId || a.Reference.Length>0&&a.Reference.Equals(b.Reference,StringComparison.OrdinalIgnoreCase) || a.Name.Length>0&&a.Name.Equals(b.Name,StringComparison.OrdinalIgnoreCase);
            // A name boundary is not a sharp bend. Degree-two continuations with
            // compatible roles/widths may share a tangent across source ways.
            if(a.Classification!=b.Classification||a.Lanes!=b.Lanes||a.OneWay!=b.OneWay||
                !sameRoute&&(a.ProtectedLink!=b.ProtectedLink||a.Reference.Length>0&&b.Reference.Length>0&&!a.Reference.Equals(b.Reference,StringComparison.OrdinalIgnoreCase)))continue;
            if(a.OneWay&&left.start==right.start)continue;
            // A curve joining a straight section must not bow that straight section.
            bool Straight(PreparedSegment s)=>s.SourceGeometry.Count>=2&&s.SourceGeometry.All(p=>DistanceToSegment(p,s.Start,s.End)<.05f);
            var point=left.start?a.Start:a.End;var other=right.start?b.Start:b.End;
            if(math.distance(point,other)>.01f)continue;
            var ca=left.start?a.Control1??math.lerp(a.Start,a.End,1f/3):a.Control2??math.lerp(a.Start,a.End,2f/3);
            var cb=right.start?b.Control1??math.lerp(b.Start,b.End,1f/3):b.Control2??math.lerp(b.Start,b.End,2f/3);
            var incoming=math.normalizesafe(point-ca);var outgoing=math.normalizesafe(cb-point);
            if(math.dot(incoming,outgoing)<.866f)continue; // Intentional corners stay sharp.
            // An individual source chord is not necessarily an intentionally
            // straight road. Only long straight runs anchor their own direction.
            var anchorA=Straight(a)&&math.distance(a.Start.xz,a.End.xz)>150;
            var anchorB=Straight(b)&&math.distance(b.Start.xz,b.End.xz)>150;
            if(anchorA&&anchorB)continue;
            var tangent=anchorA?incoming:anchorB?outgoing:math.normalizesafe(incoming+outgoing);
            var newA=point-tangent*math.distance(point,ca);var newB=point+tangent*math.distance(point,cb);
            if(Math.Abs(newA.x)>halfWidth||Math.Abs(newA.z)>halfWidth||Math.Abs(newB.x)>halfWidth||Math.Abs(newB.z)>halfWidth)continue;
            var candidateA=left.start?a with{Control1=newA}:a with{Control2=newA};
            var candidateB=right.start?b with{Control1=newB}:b with{Control2=newB};
            if(!WithinTolerance(candidateA)||!WithinTolerance(candidateB))continue;
            sections[left.index]=candidateA;sections[right.index]=candidateB;smoothed++;
        }
        log.Count("Road curve joins smoothed",smoothed);
        log.Diagnostic($"PROCESS tangent-continuity complete; joinsSmoothed={smoothed}; junctions/sharp-corners unchanged=true");
        SmoothRampAttachments(sections,log,token,halfWidth);
    }

    public static IReadOnlyList<PreparedSegment> RefineFinalRoadJoins(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token,double width)
    {
        var result=source.ToList();SmoothJoins(result,log,token,(float)(width/2));return result;
    }

    private static IReadOnlyList<PreparedSegment> NormalizeLaneRuns(IReadOnlyList<PreparedSegment> input,ImportLog log,CancellationToken token)
    {
        var result=input.ToArray();var nodes=new Dictionary<long,List<int>>();
        for(var i=0;i<input.Count;i++)if(input[i].Kind==NetworkKind.Road)
            foreach(var id in new[]{input[i].StartNodeId,input[i].EndNodeId}){if(!nodes.TryGetValue(id,out var list))nodes[id]=list=new List<int>();list.Add(i);}
        var changed=0;
        for(var i=0;i<input.Count;i++)
        {
            token.ThrowIfCancellationRequested();var s=input[i];
            if(s.Kind!=NetworkKind.Road||s.ExplicitLanes||s.ProtectedLink||!s.OneWay||!(s.Classification=="motorway"||s.Classification=="trunk")||math.distance(s.Start.xz,s.End.xz)>180)continue;
            int Neighbor(long id)
            {
                if(nodes[id].Count!=2)return -1;return nodes[id][0]==i?nodes[id][1]:nodes[id][0];
            }
            var ai=Neighbor(s.StartNodeId);var bi=Neighbor(s.EndNodeId);if(ai<0||bi<0||ai==bi)continue;
            var a=input[ai];var b=input[bi];
            bool Compatible(PreparedSegment p)=>!p.ProtectedLink&&p.Classification==s.Classification&&p.OneWay==s.OneWay&&p.Layer==s.Layer&&p.Tunnel==s.Tunnel;
            if(!Compatible(a)||!Compatible(b)||a.Lanes!=b.Lanes||a.Lanes==s.Lanes)continue;
            result[i]=s with{Lanes=Numeric.Clamp(a.Lanes,1,5)};changed++;
        }
        log.Count("Short highway lane fluctuations normalized",changed);
        // Remove short lane-count runs bounded on both sides by the same size.
        // Follow route-compatible continuations through ramp junctions, but never
        // normalize a ramp or erase a sustained lane-count change.
        var runChanged=0;var seenRuns=new HashSet<int>();
        bool SameRoute(PreparedSegment a,PreparedSegment b)=>!b.ProtectedLink&&a.Kind==b.Kind&&a.Classification==b.Classification&&a.OneWay==b.OneWay&&a.Layer==b.Layer&&a.Tunnel==b.Tunnel&&
            (a.WayId==b.WayId||a.CorridorId!=0&&a.CorridorId==b.CorridorId||a.Reference.Length>0&&a.Reference==b.Reference||a.Name.Length>0&&a.Name==b.Name);
        int Continuation(int index,long node)
        {
            if(nodes[node].Any(i=>result[i].ProtectedLink))return -1;
            var candidates=nodes[node].Where(i=>i!=index&&SameRoute(result[index],result[i])).ToArray();
            return candidates.Length==1?candidates[0]:-1;
        }
        for(var seed=0;seed<result.Length;seed++)
        {
            token.ThrowIfCancellationRequested();var s=result[seed];
            if(s.Kind!=NetworkKind.Road||s.ExplicitLanes||s.ProtectedLink||seenRuns.Contains(seed))continue;
            var run=new HashSet<int>{seed};var ends=new List<int>();var length=math.distance(s.Start.xz,s.End.xz);
            foreach(var startNode in new[]{s.StartNodeId,s.EndNodeId})
            {
                var current=seed;var node=startNode;
                while(run.Count<64&&length<=350)
                {
                    var next=Continuation(current,node);if(next<0||run.Contains(next))break;
                    var road=result[next];if(road.Lanes!=s.Lanes){ends.Add(next);break;}
                    run.Add(next);length+=math.distance(road.Start.xz,road.End.xz);current=next;node=road.StartNodeId==node?road.EndNodeId:road.StartNodeId;
                }
            }
            foreach(var index in run)seenRuns.Add(index);
            if(length>350||ends.Count!=2||run.Any(index=>result[index].ExplicitLanes)||result[ends[0]].Lanes!=result[ends[1]].Lanes)continue;
            foreach(var index in run){result[index]=result[index] with{Lanes=result[ends[0]].Lanes};runChanged++;}
        }
        log.Count("Corridor lane fluctuation sections normalized",runChanged);
        log.Diagnostic("LANE POLICY syntheticAuxiliaryWidening=false; sustainedSourceLaneChangesPreserved=true; maximumNormalizedRunMeters=350; rampsNotWidened=true");
        return result;
    }

    private static void SmoothRampAttachments(List<PreparedSegment> sections,ImportLog log,CancellationToken token,float halfWidth)
    {
        var nodes=new Dictionary<long,List<(int index,bool start)>>();
        for(var i=0;i<sections.Count;i++)if(sections[i].Kind==NetworkKind.Road)
            foreach(var start in new[]{true,false}){var id=start?sections[i].StartNodeId:sections[i].EndNodeId;if(!nodes.TryGetValue(id,out var list))nodes[id]=list=new List<(int,bool)>();list.Add((i,start));}
        var changed=0;
        foreach(var entries in nodes.Values.Where(e=>e.Count>=3))foreach(var rampEnd in entries)
        {
            token.ThrowIfCancellationRequested();var ramp=sections[rampEnd.index];if(!ramp.ProtectedLink||!ramp.OneWay)continue;
            var point=rampEnd.start?ramp.Start:ramp.End;
            var control=rampEnd.start?ramp.Control1??math.lerp(ramp.Start,ramp.End,1f/3):ramp.Control2??math.lerp(ramp.Start,ramp.End,2f/3);
            var direction=math.normalizesafe((rampEnd.start?control-point:point-control).xz);
            var best=direction;var score=.5f;PreparedSegment? bestMain=null;
            foreach(var entry in entries)
            {
                var main=sections[entry.index];if(main.ProtectedLink||!main.OneWay||main.Tunnel!=ramp.Tunnel)continue;
                var handle=entry.start?main.Control1??math.lerp(main.Start,main.End,1f/3):main.Control2??math.lerp(main.Start,main.End,2f/3);
                var heading=math.normalizesafe((entry.start?handle-main.Start:main.End-handle).xz);
                var dot=math.dot(direction,heading);if(dot>score){score=dot;best=heading;bestMain=main;}
            }
            if(score<=.5f)continue;
            var length=Math.Min(60,math.distance(point.xz,control.xz));
            var revised=control;revised.x=point.x+best.x*length*(rampEnd.start?1:-1);revised.z=point.z+best.y*length*(rampEnd.start?1:-1);
            // Lateral lane alignment belongs to NetworkLaneAlignment only.
            // Applying it here too doubled offsets before the final width pass.
            if(math.distance(revised.xz,control.xz)>12||Math.Abs(revised.x)>halfWidth||Math.Abs(revised.z)>halfWidth)continue;
            if(bestMain!=null&&bestMain.NativeLaneOffsets.Count>0&&ramp.NativeLaneOffsets.Count>0)log.Count("Ramp outer-lane approaches aligned");
            var candidate=rampEnd.start?ramp with{Control1=revised}:ramp with{Control2=revised};
            // Direction-only smoothing must obey the existing source envelope.
            // Lane-aware geometry is an intentional bounded lateral correction;
            // retain its approved alignment as the subsequent grading reference.
            if(!WithinTolerance(candidate))continue;
            sections[rampEnd.index]=candidate;changed++;
        }
        log.Count("Ramp mainline tangents aligned",changed);
    }

    public static IReadOnlyList<PreparedSegment> Refit(PreparedSegment section,CancellationToken token,float tolerance=.5f)
    {
        var points=section.SourceGeometry.ToList();if(points.Count<3)return new[]{section};
        var edges=new List<PreparedSegment>();
        long Id(float3 point){unchecked{return -1-(long)(((ulong)section.WayId*104729UL^math.hash(point.xz))%(ulong)long.MaxValue);}}
        for(var i=1;i<points.Count;i++)
            edges.Add(section with{Start=points[i-1],End=points[i],StartNodeId=i==1?section.StartNodeId:Id(points[i-1]),EndNodeId=i==points.Count-1?section.EndNodeId:Id(points[i]),Control1=null,Control2=null,HorizontalParent=null,
                StartRoundaboutRadius=i==1?section.StartRoundaboutRadius:0,EndRoundaboutRadius=i==points.Count-1?section.EndRoundaboutRadius:0});
        var output=new List<PreparedSegment>();Fit(points,edges,output,token,float.PositiveInfinity,tolerance);return output;
    }

    private static void Fit(List<float3> points, IReadOnlyList<PreparedSegment> source, List<PreparedSegment> output, CancellationToken token, float halfWidth,float? tolerance=null)
    {
        token.ThrowIfCancellationRequested();
        if (source.Count == 1) { output.Add(source[0] with { SourceGeometry = points.ToArray() }); return; }
        var first = source[0]; var last = source[source.Count - 1];
        var distance = math.distance(points[0], points[points.Count - 1]);
        // Estimate direction over a short arc, rather than trusting one noisy OSM edge.
        var reach = Math.Min(first.Kind==NetworkKind.Rail?80f:first.Classification=="motorway"||first.Classification=="trunk"?60f:35f, distance * .2f);
        float3 Tangent(bool end)
        {
            var origin = end ? points[points.Count - 1] : points[0];
            var length = 0f; var previous = origin;
            for (var i = 1; i < points.Count; i++)
            {
                var point = points[end ? points.Count - 1 - i : i];
                length += math.distance(previous.xz, point.xz); previous = point;
                if (length >= reach || i == points.Count - 1)
                    return math.normalizesafe(end ? origin - point : point - origin);
            }
            return math.normalizesafe(last.End - first.Start);
        }
        var c1 = points[0] + Tangent(false) * distance / 3f;
        var c2 = points[points.Count - 1] - Tangent(true) * distance / 3f;
        var candidate = first with { End = last.End, EndNodeId = last.EndNodeId, EndRoundaboutRadius=last.EndRoundaboutRadius,
            EndMiniRoundabout=last.EndMiniRoundabout,EndTerminal=last.EndTerminal,
            SourceWayIds=source.SelectMany(s=>s.SourceWayIds.Count>0?s.SourceWayIds:new[]{s.WayId}).Distinct().ToArray(),
            OriginalGeometry=source.SelectMany(s=>s.OriginalGeometry.Count>0?s.OriginalGeometry:new[]{s.Start,s.End}).ToArray(),
            ExplicitLanes=source.Any(s=>s.ExplicitLanes),Control1 = c1, Control2 = c2, SourceGeometry = points.ToArray() };
        if (distance > 0.05f && Math.Abs(c1.x) <= halfWidth && Math.Abs(c1.z) <= halfWidth &&
            Math.Abs(c2.x) <= halfWidth && Math.Abs(c2.z) <= halfWidth && WithinTolerance(candidate,tolerance)) { output.Add(candidate); return; }
        // A rejected fit increases the section count, never deletes source geometry.
        var split = source.Count / 2;
        Fit(points.Take(split + 1).ToList(), source.Take(split).ToArray(), output, token, halfWidth,tolerance);
        Fit(points.Skip(split).ToList(), source.Skip(split).ToArray(), output, token, halfWidth,tolerance);
    }

    private static List<float3> Simplify(IReadOnlyList<float3> points, float tolerance)
    {
        var keep = new SortedSet<int> { 0, points.Count - 1 };
        void Visit(int start, int end)
        {
            var farthest = tolerance; var split = -1;
            for (var i = start + 1; i < end; i++)
            { var distance = DistanceToSegment(points[i], points[start], points[end]); if (distance > farthest) { farthest = distance; split = i; } }
            if (split < 0) return;
            keep.Add(split); Visit(start, split); Visit(split, end);
        }
        Visit(0, points.Count - 1); return keep.Select(i => points[i]).ToList();
    }

    public static float3 Evaluate(PreparedSegment segment, float t)
    {
        var c1 = segment.CourseControl1;
        var c2 = segment.CourseControl2;
        var u = 1 - t;
        return u * u * u * segment.CourseStart + 3 * u * u * t * c1 + 3 * u * t * t * c2 + t * t * t * segment.CourseEnd;
    }

    public static float DistanceToSegment(float3 point, float3 start, float3 end)
    {
        point.y = start.y = end.y = 0;
        var delta = end - start;
        var t = math.lengthsq(delta) > 0 ? math.clamp(math.dot(point - start, delta) / math.lengthsq(delta), 0, 1) : 0;
        return math.distance(point, start + t * delta);
    }

    public static bool WithinTolerance(PreparedSegment segment,float? requestedTolerance=null)
    {
        if (segment.SourceGeometry.Count < 2) return false;
        // Exact straight horizontal geometry remains exact even when only its vertical
        // controls change. Do not reject long straight sections on a sampling allowance.
        if (segment.SourceGeometry.All(p => DistanceToSegment(p, segment.Start, segment.End) < .01f) &&
            DistanceToSegment(segment.Control1 ?? segment.Start, segment.Start, segment.End) < .01f &&
            DistanceToSegment(segment.Control2 ?? segment.End, segment.Start, segment.End) < .01f) return true;
        var sourceIndex=new LineDistanceIndex(segment.SourceGeometry);
        var c1 = segment.Control1 ?? math.lerp(segment.Start, segment.End, 1f / 3f);
        var c2 = segment.Control2 ?? math.lerp(segment.Start, segment.End, 2f / 3f);
        var a = segment.Start; var d = segment.End; a.y = c1.y = c2.y = d.y = 0;
        var arcBound = math.distance(a, c1) + math.distance(c1, c2) + math.distance(c2, d);
        var tolerance = requestedTolerance??Tolerance(segment.Kind);
        // Adapt sampling to curve length while preserving the same continuous-error
        // reserve. Short curves no longer need 256 samples; long curves get more.
        var sampleCount = Math.Min(2048, Math.Max(32, (int)Math.Ceiling(6 * arcBound / tolerance)));
        var samples = new float3[sampleCount + 1];
        for (var i = 0; i <= sampleCount; i++) samples[i] = Evaluate(segment, i / (float)sampleCount);
        // Control-polygon length bounds between-sample movement; reserve that error
        // instead of trusting vertex-only comparisons which can cut a hairpin.
        var allowance = tolerance - 3 * arcBound / (2 * sampleCount);
        if (allowance <= 0) return false;
        foreach (var point in samples) if (sourceIndex.Distance(point) > allowance) return false;
        var curveIndex=new LineDistanceIndex(samples);
        for (var i = 1; i < segment.SourceGeometry.Count; i++)
        {
            var start = segment.SourceGeometry[i - 1]; var end = segment.SourceGeometry[i];
            var steps = Math.Max(1, (int)Math.Ceiling(math.distance(start, end) / (tolerance / 2)));
            for (var j = 0; j <= steps; j++)
                if (curveIndex.Distance(math.lerp(start, end, j / (float)steps)) > tolerance * 0.75f) return false;
        }
        return true;
    }

    // Exact nearest-segment queries with bounding-box pruning. This retains the
    // same two-sided tolerance test without scanning thousands of samples per query.
    private sealed class LineDistanceIndex
    {
        private sealed class Node
        {
            public float2 Min,Max;
            public int First,Count;
            public Node? Left,Right;
        }
        private readonly IReadOnlyList<float3> points;
        private readonly int[] indices;
        private readonly Node root;
        public LineDistanceIndex(IReadOnlyList<float3> line)
        {
            points=line;indices=Enumerable.Range(0,line.Count-1).ToArray();root=Build(0,indices.Length);
        }
        private Node Build(int first,int count)
        {
            var node=new Node{First=first,Count=count,Min=new float2(float.PositiveInfinity),Max=new float2(float.NegativeInfinity)};
            for(var i=first;i<first+count;i++){var index=indices[i];node.Min=math.min(node.Min,math.min(points[index].xz,points[index+1].xz));node.Max=math.max(node.Max,math.max(points[index].xz,points[index+1].xz));}
            if(count<=8)return node;
            var x=node.Max.x-node.Min.x>=node.Max.y-node.Min.y;
            Array.Sort(indices,first,count,Comparer<int>.Create((a,b)=>
                (x?points[a].x+points[a+1].x:points[a].z+points[a+1].z).CompareTo(x?points[b].x+points[b+1].x:points[b].z+points[b+1].z)));
            var half=count/2;node.Left=Build(first,half);node.Right=Build(first+half,count-half);return node;
        }
        public float Distance(float3 point)
        {
            var best=float.PositiveInfinity;
            float Lower(Node node)=>math.distance(point.xz,math.clamp(point.xz,node.Min,node.Max));
            void Search(Node node)
            {
                if(Lower(node)>best)return;
                if(node.Left==null){for(var i=node.First;i<node.First+node.Count;i++){var index=indices[i];best=Math.Min(best,DistanceToSegment(point,points[index],points[index+1]));}return;}
                var left=node.Left!;var right=node.Right!;
                if(Lower(left)<=Lower(right)){Search(left);Search(right);}else{Search(right);Search(left);}
            }
            Search(root);return best;
        }
    }

    public static void Validate(IReadOnlyList<PreparedSegment> segments, double width, CancellationToken token, ImportLog? log = null)
    {
        var clock = Stopwatch.StartNew(); var nextProgress = 5d; var processed = 0;
        var parents = new HashSet<PreparedSegment>();
        foreach (var segment in segments)
        {
            token.ThrowIfCancellationRequested();
            processed++;
            if (clock.Elapsed.TotalSeconds >= nextProgress)
            { log?.Diagnostic($"PROCESS geometry-validation sections={processed}/{segments.Count}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.0}"); nextProgress = clock.Elapsed.TotalSeconds + 5; }
            if (!math.all(math.isfinite(segment.Start)) || !math.all(math.isfinite(segment.End)) || math.distance(segment.Start, segment.End) < 0.01f)
                log?.Warn($"Normalized way {segment.WayId} has invalid geometry; retained for local repair, not an import gate.");
            for (var i = 0; i <= 32; i++)
            {
                var p = Evaluate(segment, i / 32f);
                if (!math.all(math.isfinite(p)) || Math.Abs(p.x) > width / 2 + 0.02 || Math.Abs(p.z) > width / 2 + 0.02)
                    log?.Count("Normalized network envelope observations");
            }
            if (segment.HorizontalParent != null)
            {
                // Profile refinement subdivides an already validated horizontal curve exactly.
                if (parents.Add(segment.HorizontalParent) && !WithinTolerance(segment.HorizontalParent)) log?.Count("Profile parent geographic tolerance observations");
                for(var i=0;i<=32;i++) if(math.distance(Evaluate(segment,i/32f).xz,Evaluate(segment.HorizontalParent,math.lerp(segment.ParentStartT,segment.ParentEndT,i/32f)).xz)>.02f)
                    log?.Count("Vertical refinement horizontal alignment observations");
            }
            else if (segment.Control1.HasValue && !WithinTolerance(segment))
                log?.Count("Fitted network geographic tolerance observations");
        }
    }
}
