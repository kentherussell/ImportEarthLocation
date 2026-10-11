using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

public static class NetworkVerticalProfiles
{
    public const float MinimumBridgeClearance = 5f;
    private const float DeckSeparation = MinimumBridgeClearance + 4f;
    private const float MaximumClearanceLift = 30f;
    public const float AbsoluteRoadGrade = .348f;
    // Practical import limits, not claims about the maximum grade of real roads.
    public static float GradeLimit(PreparedSegment segment) => segment.SourceSpeedKph>=80&&!segment.ProtectedLink?.09f:segment.ProtectedLink ? .13f :
        segment.Classification == "motorway" ? .09f : segment.Classification == "trunk" ? .11f : .16f;
    public static float DesiredGrade(PreparedSegment s)=>s.Kind==NetworkKind.Rail?.02f:s.ProtectedLink?.06f:
        s.Classification=="motorway"||s.Classification=="trunk"||s.SourceSpeedKph>=80?.035f:
        s.Classification=="primary"||s.Classification=="secondary"?.06f:.08f;
    public static float CutFillBudget(PreparedSegment segment) => segment.Kind==NetworkKind.Rail?6f:
        segment.Classification=="motorway"||segment.Classification=="trunk"||segment.ProtectedLink?20f:10f;
    public static float MaximumGrade(PreparedSegment segment)
    {
        var a=segment.CourseStart;var b=segment.CourseControl1;var c=segment.CourseControl2;var d=segment.CourseEnd;
        var maximum=0f;
        for(var i=0;i<=256;i++)
        {var t=i/256f;var u=1-t;var derivative=3*u*u*(b-a)+6*u*t*(c-b)+3*t*t*(d-c);var horizontal=math.length(derivative.xz);var grade=horizontal<.00001f?(Math.Abs(derivative.y)<.00001f?0:float.PositiveInfinity):Math.Abs(derivative.y)/horizontal;maximum=Math.Max(maximum,grade);}
        return maximum;
    }

    // Quadratic Bezier derivative controls bound every derivative in an interval.
    // Subdivision tightens the bounds; an uncertain interval is rejected, never
    // accepted on the strength of discrete samples that might miss a sharp peak.
    public static bool WithinGradeCeiling(PreparedSegment segment, float ceiling = AbsoluteRoadGrade)
    {
        var a=segment.CourseStart;var b=segment.CourseControl1;var c=segment.CourseControl2;var d=segment.CourseEnd;
        return Bound(3*(b-a),3*(c-b),3*(d-c),0);
        bool Bound(float3 p,float3 q,float3 r,int depth)
        {
            if(!math.all(math.isfinite(p))||!math.all(math.isfinite(q))||!math.all(math.isfinite(r)))return false;
            var min=math.min(p,math.min(q,r));var max=math.max(p,math.max(q,r));
            var vertical=Math.Max(Math.Abs(min.y),Math.Abs(max.y));
            if(vertical<.000001f)return true;
            var dx=min.x>0?min.x:max.x<0?-max.x:0;var dz=min.z>0?min.z:max.z<0?-max.z:0;
            var horizontal=(float)Math.Sqrt(dx*dx+dz*dz);
            if(vertical <= ceiling*horizontal+.000001f)return true;
            var pq=(p+q)*.5f;var qr=(q+r)*.5f;var middle=(pq+qr)*.5f;
            foreach(var derivative in new[]{p,middle,r})
                if(Math.Abs(derivative.y)>ceiling*math.length(derivative.xz)+.000001f)return false;
            if(depth>=14)return false;
            return Bound(p,pq,middle,depth+1)&&Bound(middle,qr,r,depth+1);
        }
    }

    public static IReadOnlyList<PreparedSegment> Prepare(IReadOnlyList<PreparedSegment> source, ushort[] heightmap, float scale, double width, ImportLog log, CancellationToken token)
    {
        if(source.Count==0)return source;
        var roads=source.Where(s=>s.Kind==NetworkKind.Road).ToArray();
        var clock=System.Diagnostics.Stopwatch.StartNew();var nextProgress=5d;var processed=0;
        log.Diagnostic($"PROCESS road-grading begin; roads={roads.Length}; desiredMainlineGrade=3.5%; desiredRampGrade=6%; desiredMajorRoadGrade=6%; motorwayExceptionalLimit=9%; trunkExceptionalLimit=11%; rampExceptionalLimit=13%; streetExceptionalLimit=16%; terrainUnchanged=true; railLimitsUnchanged=true");
        var lengths=new Dictionary<PreparedSegment,float>();foreach(var road in roads){token.ThrowIfCancellationRequested();lengths[road]=ArcLength(road);}
        var heights=new Dictionary<long,float>();var samples=new Dictionary<long,List<float>>();
        var surfaceNodeIds=new HashSet<long>(roads.Where(s=>!s.Tunnel).SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}));
        float Ground(float3 p)=>Bilinear(p.x,p.z,heightmap,scale,width);
        foreach(var s in roads) foreach(var pair in new[]{(s.StartNodeId,s.Start),(s.EndNodeId,s.End)})
        {if(!samples.TryGetValue(pair.Item1,out var values))samples[pair.Item1]=values=new List<float>();values.Add(Ground(pair.Item2));}
        foreach(var pair in samples){pair.Value.Sort();heights[pair.Key]=pair.Value[pair.Value.Count/2];}
        // Rails are fixed crossing obstacles, not members of the road height graph.
        // Give them private node IDs so an OSM road/rail crossing cannot accidentally
        // become a shared elevation constraint or a clearance feedback loop.
        var nextRailNode=long.MinValue;
        var railObstacles=source.Where(s=>s.Kind==NetworkKind.Rail&&!s.Tunnel).Select(s=>
        {
            while(heights.ContainsKey(nextRailNode))nextRailNode++;
            var a=nextRailNode++;while(heights.ContainsKey(nextRailNode))nextRailNode++;
            var b=nextRailNode++;heights[a]=s.Start.y;heights[b]=s.End.y;
            samples[a]=new List<float>{s.Start.y};samples[b]=new List<float>{s.End.y};
            var obstacle=s with{StartNodeId=a,EndNodeId=b};lengths[obstacle]=ArcLength(obstacle);return obstacle;
        }).ToArray();
        // Index crossings once, rather than scanning every road for every bridge.
        var crossingGrid=new Dictionary<(int,int),HashSet<PreparedSegment>>();
        foreach(var road in roads.Where(r=>!r.Tunnel).Concat(railObstacles))
        {
            var count=Math.Max(1,(int)Math.Ceiling(lengths[road]/32));
            for(var j=0;j<=count;j++)
            {var p=NetworkNormalizer.Evaluate(road,j/(float)count);var x=(int)Math.Floor(p.x/64);var z=(int)Math.Floor(p.z/64);
             for(var dx=-1;dx<=1;dx++)for(var dz=-1;dz<=1;dz++){var key=(x+dx,z+dz);if(!crossingGrid.TryGetValue(key,out var set))crossingGrid[key]=set=new HashSet<PreparedSegment>();set.Add(road);}}
        }
        var structureHeights=new Dictionary<long,float>();
        var clearanceConstraints=new List<(long[] span,PreparedSegment upper,float t,PreparedSegment lower,float u)>();
        foreach(var tunnel in new[]{false,true})
        {
            var structures=roads.Where(s=>tunnel?s.Tunnel:s.Bridge).ToArray();var adjacency=new Dictionary<long,List<PreparedSegment>>();
            foreach(var s in structures)foreach(var id in new[]{s.StartNodeId,s.EndNodeId}){if(!adjacency.TryGetValue(id,out var list))adjacency[id]=list=new List<PreparedSegment>();list.Add(s);}
            var visited=new HashSet<PreparedSegment>();
            foreach(var seed in structures)
            {
                token.ThrowIfCancellationRequested();if(!visited.Add(seed))continue;
                var group=new List<PreparedSegment>();var queue=new List<PreparedSegment>{seed};var identities=new HashSet<PreparedSegment>();
                for(var index=0;index<queue.Count;index++){var s=queue[index];if(!identities.Add(s))continue;group.Add(s);visited.Add(s);foreach(var id in new[]{s.StartNodeId,s.EndNodeId})foreach(var next in adjacency[id])if(!identities.Contains(next))queue.Add(next);}
                var degree=new Dictionary<long,int>();var positions=new Dictionary<long,float3>();foreach(var s in group){foreach(var id in new[]{s.StartNodeId,s.EndNodeId}){degree.TryGetValue(id,out var count);degree[id]=count+1;}positions[s.StartNodeId]=s.Start;positions[s.EndNodeId]=s.End;}
                var portals=degree.Where(p=>p.Value==1).Select(p=>p.Key).OrderBy(n=>n).ToArray();
                var branched=portals.Length!=2||degree.Values.Any(d=>d>2);
                if(branched)
                {
                    if(tunnel)
                    {
                        // Solve every branch on the actual graph. Ground/tunnel
                        // tie-ins remain portals, interior nodes remain underground.
                        foreach(var node in positions.Keys)
                        {var surface=surfaceNodeIds.Contains(node);structureHeights[node]=Ground(positions[node])-(surface?0:8);}
                        for(var pass=0;pass<64;pass++)foreach(var edge in group)
                        {
                            var a=edge.StartNodeId;var b=edge.EndNodeId;var allowed=lengths[edge]*GradeLimit(edge);
                            var excess=Math.Abs(structureHeights[a]-structureHeights[b])-allowed;if(excess<=.001f)continue;
                            var sign=structureHeights[a]>structureHeights[b]?1f:-1f;
                            if(!surfaceNodeIds.Contains(a))structureHeights[a]=Math.Min(Ground(positions[a])-8,structureHeights[a]-sign*excess*.5f);
                            if(!surfaceNodeIds.Contains(b))structureHeights[b]=Math.Min(Ground(positions[b])-8,structureHeights[b]+sign*excess*.5f);
                        }
                        log.Count("Branched tunnel groups fitted jointly");continue;
                    }
                    // A connected interchange is not one bridge deck. Choosing
                    // two farthest nodes and projecting every branch onto that
                    // plane couples independent flyovers and creates tall ramps.
                    // Keep shared local anchors here. The final contact solver
                    // handles actual crossings and shared grades jointly.
                    foreach(var node in positions.Keys)
                    {
                        var local=Ground(positions[node]);
                        structureHeights[node]=structureHeights.TryGetValue(node,out var prior)?Math.Max(prior,local):local;
                    }
                    log.Count("Branched bridge groups deferred to coupled contacts");
                    log.Diagnostic($"BRIDGE GRAPH local-profile seedWay={seed.WayId}; sections={group.Count}; nodes={positions.Count}; branches={degree.Count(p=>p.Value>2)}; wholeInterchangePlane=false; authoritativeClearanceSolve=final-coupled-contacts");
                    continue;
                }
                var distances=new Dictionary<long,float>{{portals[0],0}};var ordered=new List<long>{portals[0]};
                for(var index=0;index<ordered.Count;index++)foreach(var s in adjacency[ordered[index]])
                {if(!identities.Contains(s))continue;var next=s.StartNodeId==ordered[index]?s.EndNodeId:s.StartNodeId;if(distances.ContainsKey(next))continue;distances[next]=distances[ordered[index]]+ArcLength(s);ordered.Add(next);}
                var total=distances[portals[1]];if(total<.01f)continue;
                var start=Ground(positions[portals[0]]);var end=Ground(positions[portals[1]]);
                // Ground-supported abutments are allowed; do not blanket-add 8 m
                // at both portals. Lift the whole bridge profile only where a
                // real lower road crossing needs clearance, then grade approaches.
                var lift=0f;var crossings=0;
                var crossingPairs=new HashSet<(PreparedSegment,PreparedSegment)>();
                if(!tunnel)foreach(var bridge in group)
                {
                    var count=Math.Max(8,(int)Math.Ceiling(lengths[bridge]/8));
                    for(var j=0;j<=count;j++)
                    {
                        var t=j/(float)count;var p=NetworkNormalizer.Evaluate(bridge,t);
                        if(!crossingGrid.TryGetValue(((int)Math.Floor(p.x/64),(int)Math.Floor(p.z/64)),out var nearby))continue;
                        var baseline=math.lerp(start,end,math.clamp(math.lerp(distances[bridge.StartNodeId],distances[bridge.EndNodeId],t)/total,0,1));
                        foreach(var lower in nearby)
                        {
                            if(identities.Contains(lower)||lower.Bridge&&lower.Layer>=bridge.Layer||lower.StartNodeId==bridge.StartNodeId||lower.EndNodeId==bridge.EndNodeId||lower.EndNodeId==bridge.StartNodeId||lower.StartNodeId==bridge.EndNodeId)continue;
                            // A lower approach sharing ANY node of this component is
                            // not an independent deck. Raising both creates a cycle.
                            if(degree.ContainsKey(lower.StartNodeId)||degree.ContainsKey(lower.EndNodeId))continue;
                            var heading=math.normalizesafe(lower.End.xz-lower.Start.xz);var bh=math.normalizesafe(bridge.End.xz-bridge.Start.xz);
                            if(Math.Abs(math.dot(heading,bh))>.9f)continue;
                            var delta=lower.End.xz-lower.Start.xz;var square=math.lengthsq(delta);if(square<.01f)continue;
                            var u=math.clamp(math.dot(p.xz-lower.Start.xz,delta)/square,0,1);var q=NetworkNormalizer.Evaluate(lower,u);
                            if(math.distance(p.xz,q.xz)>Math.Max(6,lower.Lanes*1.75f+bridge.Lanes*1.75f))continue;
                            if(!crossingPairs.Add((bridge,lower))||!TryCrossing(bridge,lower,out var crossingT,out var crossingU))continue;
                            var lowerHeight=math.lerp(heights[lower.StartNodeId],heights[lower.EndNodeId],crossingU);
                            baseline=math.lerp(start,end,math.clamp(math.lerp(distances[bridge.StartNodeId],distances[bridge.EndNodeId],crossingT)/total,0,1));
                            // 5 m clear space + 2 m conservative deck allowance +
                            // 2 m reserve for later bounded tangent adjustments.
                            lift=Math.Max(lift,lowerHeight+DeckSeparation-baseline);crossings++;
                            clearanceConstraints.Add((degree.Keys.ToArray(),bridge,crossingT,lower,crossingU));
                        }
                    }
                }
                if(lift>MaximumClearanceLift){log.Count("Bridge clearance bounded fallbacks");lift=MaximumClearanceLift;}
                if(lift>0){start+=lift;end+=lift;log.Count("Bridge crossing clearance profiles raised");}
                if(crossings>0)log.Diagnostic($"BRIDGE CLEARANCE way={seed.WayId}; crossingSamples={crossings}; profileLiftMeters={lift}; minimumClearSpaceMeters=5; deckAllowanceMeters=2; smoothingReserveMeters=2; fullSpanAndConnectedApproaches=true; renderedDeckNotYetVerified=true");
                foreach(var point in distances)structureHeights[point.Key]=math.lerp(start,end,math.clamp(point.Value/total,0,1));
            }
        }
        var coordinatedBridgeNodes=0;var structureSnapshot=new Dictionary<long,float>(structureHeights);
        foreach(var bridge in roads.Where(r=>r.Bridge&&r.CorridorId!=0))foreach(var endpoint in new[]{(bridge.StartNodeId,bridge.Start),(bridge.EndNodeId,bridge.End)})
        {
            token.ThrowIfCancellationRequested();if(!structureSnapshot.TryGetValue(endpoint.Item1,out var current))continue;
            if(!crossingGrid.TryGetValue(((int)Math.Floor(endpoint.Item2.x/64),(int)Math.Floor(endpoint.Item2.z/64)),out var nearby))continue;
            foreach(var other in nearby)
            {
                if(!other.Bridge||other.CorridorId!=bridge.CorridorId||other.Layer!=bridge.Layer||other.WayId==bridge.WayId||math.dot(math.normalizesafe(bridge.End.xz-bridge.Start.xz),math.normalizesafe(other.End.xz-other.Start.xz))>-.9f)continue;
                if(!structureSnapshot.ContainsKey(other.StartNodeId)||!structureSnapshot.ContainsKey(other.EndNodeId))continue;
                var delta=other.End.xz-other.Start.xz;var square=math.lengthsq(delta);if(square<.01f)continue;
                var u=math.dot(endpoint.Item2.xz-other.Start.xz,delta)/square;if(u<0||u>1||math.distance(endpoint.Item2.xz,NetworkNormalizer.Evaluate(other,u).xz)>64)continue;
                var target=math.lerp(structureSnapshot[other.StartNodeId],structureSnapshot[other.EndNodeId],u);
                if(target>current&&target-current<=8){structureHeights[endpoint.Item1]=Math.Max(structureHeights[endpoint.Item1],target);coordinatedBridgeNodes++;}
            }
        }
        log.Count("Parallel bridge elevations coordinated",coordinatedBridgeNodes);
        foreach(var pair in structureHeights)heights[pair.Key]=pair.Value;
        // Spread necessary bridge rise along actual connected approach topology, not a nearby road.
        var approaches=new Dictionary<long,List<PreparedSegment>>();foreach(var s in roads.Where(s=>!s.Bridge&&!s.Tunnel))foreach(var id in new[]{s.StartNodeId,s.EndNodeId}){if(!approaches.TryGetValue(id,out var list))approaches[id]=list=new List<PreparedSegment>();list.Add(s);}
        var riseQueue=structureHeights.Keys.ToList();var rises=0;
        for(var cursor=0;cursor<riseQueue.Count;cursor++)
        {
            token.ThrowIfCancellationRequested();var id=riseQueue[cursor];if(!approaches.TryGetValue(id,out var edges))continue;
            foreach(var s in edges)
            {var other=s.StartNodeId==id?s.EndNodeId:s.StartNodeId;var required=heights[id]-lengths[s]*GradeLimit(s)*.8f;
             if(required>heights[other]+.01f&&required<=samples[other].Min()+30){heights[other]=required;riseQueue.Add(other);rises++;}}
            if(riseQueue.Count>roads.Length*4+16)
            {log.Warn("Bridge approach constraints reached the bounded repair limit; retaining current shared heights and continuing with local profile fallbacks.");log.Count("Bridge approach relaxation limits reached");break;}
        }
        if(rises>0)log.Count("Bridge grade tie-in nodes adjusted",rises);
        // Smooth connected road elevations over multiple sections rather than following
        // each DEM bump independently. Junctions and structural portals remain anchors.
        var smoothingBase=new Dictionary<long,float>(heights);var smoothedNodes=new HashSet<long>();
        // Membership, geometry and budgets do not change during longitudinal relaxation.
        var smoothingStencils=new List<(long Node,long A,long B,float LengthA,float LengthB,float LengthSum,float Budget)>();
        foreach(var pair in approaches)
        {
                if(pair.Value.Count!=2||structureHeights.ContainsKey(pair.Key))continue;
                var a=pair.Value[0];var b=pair.Value[1];
                if(a.Classification!=b.Classification||a.Layer!=b.Layer||a.OneWay!=b.OneWay||a.ExplicitIncline.HasValue||b.ExplicitIncline.HasValue)continue;
                var otherA=a.StartNodeId==pair.Key?a.EndNodeId:a.StartNodeId;var otherB=b.StartNodeId==pair.Key?b.EndNodeId:b.StartNodeId;
                var lengthA=lengths[a];var lengthB=lengths[b];if(lengthA+lengthB<.01f||otherA==otherB)continue;
                var point=a.StartNodeId==pair.Key?a.Start:a.End;
                var incoming=math.normalizesafe(point.xz-(a.StartNodeId==pair.Key?a.End:a.Start).xz);
                var outgoing=math.normalizesafe((b.StartNodeId==pair.Key?b.End:b.Start).xz-point.xz);
                if(math.dot(incoming,outgoing)<.866f)continue;
                smoothingStencils.Add((pair.Key,otherA,otherB,lengthA,lengthB,lengthA+lengthB,CutFillBudget(a)));
        }
        var smoothingPrevious=new Dictionary<long,float>(heights);var smoothingKeys=heights.Keys.ToArray();
        for(var pass=0;pass<96;pass++)
        {
            token.ThrowIfCancellationRequested();foreach(var id in smoothingKeys)smoothingPrevious[id]=heights[id];
            foreach(var stencil in smoothingStencils)
            {
                var target=(smoothingPrevious[stencil.A]*stencil.LengthB+smoothingPrevious[stencil.B]*stencil.LengthA)/stencil.LengthSum;
                // Prefer a smooth longitudinal profile over small DEM undulations.
                // A weak terrain anchor prevents flattening an entire long corridor.
                var value=math.clamp(.98f*target+.02f*smoothingBase[stencil.Node],smoothingBase[stencil.Node]-stencil.Budget,smoothingBase[stencil.Node]+stencil.Budget);
                if(Math.Abs(value-heights[stencil.Node])>.01f)smoothedNodes.Add(stencil.Node);
                heights[stencil.Node]=value;
            }
        }
        log.Count("Road longitudinal elevation nodes smoothed",smoothedNodes.Count);
        log.Diagnostic($"PROCESS longitudinal-road-smoothing complete; nodesSmoothed={smoothedNodes.Count}; passes=96; smoothnessWeight=0.98; terrainWeight=0.02; junctionsAndPortalsAnchored=true; boundedCutFillMeters=10-streets/20-highways; terrainUnchanged=true");
        // Use actual detected divided-corridor membership, not arbitrary proximity.
        // Project onto the companion alignment at the same longitudinal position.
        var parallelAdjusted=0;
        foreach(var corridor in roads.Where(r=>r.CorridorId!=0&&!r.ProtectedLink&&!r.Tunnel&&!r.Bridge).GroupBy(r=>r.CorridorId))
        {
            var members=corridor.ToArray();var grid=new Dictionary<(int,int),HashSet<PreparedSegment>>();
            foreach(var road in members){var n=Math.Max(1,(int)Math.Ceiling(lengths[road]/64));for(var j=0;j<=n;j++){var p=NetworkNormalizer.Evaluate(road,j/(float)n);var key=((int)Math.Floor(p.x/128),(int)Math.Floor(p.z/128));if(!grid.TryGetValue(key,out var set))grid[key]=set=new HashSet<PreparedSegment>();set.Add(road);}}
            var targets=new Dictionary<long,List<float>>();
            foreach(var road in members)foreach(var endpoint in new[]{(road.StartNodeId,road.Start),(road.EndNodeId,road.End)})
            {
                token.ThrowIfCancellationRequested();if(structureHeights.ContainsKey(endpoint.Item1)||approaches[endpoint.Item1].Count!=2)continue;
                PreparedSegment? best=null;var bestU=0f;var distance=64f;
                var x=(int)Math.Floor(endpoint.Item2.x/128);var z=(int)Math.Floor(endpoint.Item2.z/128);
                for(var dx=-1;dx<=1;dx++)for(var dz=-1;dz<=1;dz++)if(grid.TryGetValue((x+dx,z+dz),out var set))foreach(var other in set)
                {
                    if(other.WayId==road.WayId||other.Layer!=road.Layer||math.dot(math.normalizesafe(road.End.xz-road.Start.xz),math.normalizesafe(other.End.xz-other.Start.xz))>-.9f)continue;
                    var delta=other.End.xz-other.Start.xz;var square=math.lengthsq(delta);if(square<.01f)continue;
                    var u=math.dot(endpoint.Item2.xz-other.Start.xz,delta)/square;if(u<0||u>1)continue;
                    var d=math.distance(endpoint.Item2.xz,NetworkNormalizer.Evaluate(other,u).xz);if(d<distance){distance=d;best=other;bestU=u;}
                }
                if(best==null)continue;
                var target=(heights[endpoint.Item1]+math.lerp(heights[best.StartNodeId],heights[best.EndNodeId],bestU))*.5f;
                if(Math.Abs(target-heights[endpoint.Item1])>8)continue;
                if(!targets.TryGetValue(endpoint.Item1,out var values))targets[endpoint.Item1]=values=new List<float>();values.Add(target);
            }
            foreach(var pair in targets){heights[pair.Key]=pair.Value.Average();parallelAdjusted++;}
        }
        log.Count("Parallel carriageway elevations coordinated",parallelAdjusted);
        // Fit a shared endpoint profile before fitting individual curves. Every road
        // using a junction receives the same height; bridge/tunnel portal heights stay fixed.
        var initialHeights=smoothingBase;var adjusted=new HashSet<long>();
        for(var pass=0;pass<32;pass++)
        {
            var changed=false;
            foreach(var edge in roads)
            {
                token.ThrowIfCancellationRequested();if(edge.Bridge||edge.Tunnel)continue;
                var a=edge.StartNodeId;var b=edge.EndNodeId;var delta=heights[b]-heights[a];
                var limit=lengths[edge]*DesiredGrade(edge);var excess=Math.Abs(delta)-limit;if(excess<=.01f)continue;
                var sign=delta>0?1f:-1f;
                var budget=CutFillBudget(edge);
                var moveA=structureHeights.ContainsKey(a)?0:Math.Max(0,Math.Min(excess*.5f,budget-sign*(heights[a]-initialHeights[a])));
                var moveB=structureHeights.ContainsKey(b)?0:Math.Max(0,Math.Min(excess-moveA,budget+sign*(heights[b]-initialHeights[b])));
                if(moveA+moveB<=.001f)continue;
                heights[a]+=sign*moveA;heights[b]-=sign*moveB;
                if(moveA>0)adjusted.Add(a);if(moveB>0)adjusted.Add(b);changed=true;
            }
            if(clock.Elapsed.TotalSeconds>=nextProgress){log.Diagnostic($"PROCESS junction-grade-relaxation pass={pass+1}/32; adjustedNodes={adjusted.Count}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.0}");nextProgress=clock.Elapsed.TotalSeconds+5;}
            if(!changed)break;
        }
        log.Count("Road grade junction nodes adjusted",adjusted.Count);
        log.Diagnostic($"PROCESS junction-grade-relaxation complete; adjustedNodes={adjusted.Count}; bridge/tunnelPortalsFixed=true; maximumLocalAdjustmentMeters=20");
        ConnectedRoadGrades.Repair(roads,heights,lengths,log,token,smoothingBase);
        // Grade projection may raise the road below a bridge. Recheck crossings
        // against those solved heights and propagate any whole-span correction.
        var clearanceAnchor=new Dictionary<long,float>(heights);
        int EndpointGradeIssues()=>roads.Count(s=>Math.Abs(heights[s.EndNodeId]-heights[s.StartNodeId])>lengths[s]*GradeLimit(s)*.8f+.01f);
        for(var pass=0;pass<3;pass++)
        {
            var lifted=0;
            var snapshot=new Dictionary<long,float>(heights);
            var gradeIssuesBefore=EndpointGradeIssues();
            var corrections=new Dictionary<long[],float>();
            foreach(var crossing in clearanceConstraints)
            {
                token.ThrowIfCancellationRequested();
                var upper=math.lerp(snapshot[crossing.upper.StartNodeId],snapshot[crossing.upper.EndNodeId],crossing.t);
                var lower=math.lerp(snapshot[crossing.lower.StartNodeId],snapshot[crossing.lower.EndNodeId],crossing.u);
                var deficit=lower+DeckSeparation-upper;if(deficit<=.01f)continue;
                var room=crossing.span.Min(id=>Math.Max(0,Math.Min(clearanceAnchor[id]+10,samples[id].Min()+MaximumClearanceLift)-snapshot[id]));
                var correction=Math.Min(deficit,room);
                if(correction<=.01f)continue;
                // Combine demands on shared nodes using MAX, never summation.
                // All lower heights are read from the same iteration snapshot.
                corrections[crossing.span]=correction;
            }
            var targets=new Dictionary<long,float>();
            foreach(var correction in corrections)foreach(var id in correction.Key)
                targets[id]=Math.Max(targets.TryGetValue(id,out var target)?target:snapshot[id],snapshot[id]+correction.Value);
            foreach(var target in targets){heights[target.Key]=target.Value;lifted++;}
            if(lifted==0)break;
            log.Count("Bridge clearance bounded nodes adjusted",lifted);
            ConnectedRoadGrades.Repair(roads,heights,lengths,log,token,smoothingBase);
            if(EndpointGradeIssues()>gradeIssuesBefore)
            {
                foreach(var pair in snapshot)heights[pair.Key]=pair.Value;
                log.Count("Bridge clearance grade-safe rollbacks");
                log.Warn("Bridge clearance refinement worsened connected approach grades; restored the last bounded continuous profile. The 5 m clearance target remains, unresolved crossings will be logged; generation continues.");
                break;
            }
        }
        var unresolvedClearances=clearanceConstraints.Count(c=>math.lerp(heights[c.upper.StartNodeId],heights[c.upper.EndNodeId],c.t)-math.lerp(heights[c.lower.StartNodeId],heights[c.lower.EndNodeId],c.u)<DeckSeparation-.05f);
        if(unresolvedClearances>0)log.Warn($"Bridge clearance: {unresolvedClearances} constraints remain unresolved within bounded elevations. Target clear space remains 5 m; continuous roads retained, no repeated unbounded lifting.");
        log.Diagnostic($"BRIDGE CLEARANCE RECHECK constraints={clearanceConstraints.Count}; unresolved={unresolvedClearances}; deckAllowanceMeters=2; clearSpaceMeters=5; renderVerificationPending=true");
        var result=new List<PreparedSegment>();var rejected=0;float maximum=0;
        foreach(var original in source)
        {
            token.ThrowIfCancellationRequested();
            if(original.Kind!=NetworkKind.Road){result.Add(original);continue;}
            processed++;
            if(clock.Elapsed.TotalSeconds>=nextProgress){log.Diagnostic($"PROCESS vertical-curve-fitting roads={processed}/{roads.Length}; outputSections={result.Count}; rejected={rejected}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.0}");nextProgress=clock.Elapsed.TotalSeconds+5;}
            var s=original;var a=s.Start;var d=s.End;a.y=heights[s.StartNodeId];d.y=heights[s.EndNodeId];
            var values=new float[33];var path=new float[33];
            var startLift=a.y-Ground(a);var endLift=d.y-Ground(d);
            for(var i=0;i<values.Length;i++)
            {var p=NetworkNormalizer.Evaluate(s,i/32f);if(i>0)path[i]=path[i-1]+math.distance(p.xz,NetworkNormalizer.Evaluate(s,(i-1)/32f).xz);values[i]=Ground(p)+math.lerp(startLift,endLift,i/32f);}
            if(path[32]<.01f){rejected++;result.Add(s with{Start=a,End=d});continue;}
            if(s.Bridge||s.Tunnel)for(var i=0;i<values.Length;i++)values[i]=math.lerp(a.y,d.y,path[i]/path[32]);
            else
            {
                // Make the interior agree with the corridor's solved endpoint profile,
                // not a second independent terrain-following solution per segment.
                for(var i=1;i<32;i++)values[i]=math.lerp(values[i],math.lerp(a.y,d.y,path[i]/path[32]),.97f);
                if(s.ExplicitIncline.HasValue && Math.Abs(a.y+s.ExplicitIncline.Value*path[32]-d.y)<=3)
                {for(var i=1;i<32;i++)values[i]=a.y+s.ExplicitIncline.Value*path[i];log.Count("Explicit incline profiles used");}
                var raw=(float[])values.Clone();for(var i=1;i<32;i++){var lo=Math.Min(raw[i-1],raw[i+1]);var hi=Math.Max(raw[i-1],raw[i+1]);if(raw[i]<lo-1||raw[i]>hi+1)values[i]=(raw[i-1]+raw[i+1])*.5f;}
            }
            values[0]=a.y;values[32]=d.y;
            var practicalLimit=GradeLimit(s);
            if(!s.Bridge&&!s.Tunnel && Math.Abs(d.y-a.y)<=path[32]*practicalLimit*.8f)
            {
                // Bounded cuts/fills smooth terrain noise without flattening the map.
                // Feasible endpoint envelopes avoid spikes at shared junctions.
                var gradeBudget=practicalLimit*.8f;var terrainBudget=CutFillBudget(s);
                var raw=(float[])values.Clone();
                for(var pass=0;pass<4;pass++)
                {
                    var previous=(float[])values.Clone();
                    for(var i=1;i<32;i++)
                    {
                        var lo=Math.Max(a.y-gradeBudget*path[i],d.y-gradeBudget*(path[32]-path[i]));
                        var hi=Math.Min(a.y+gradeBudget*path[i],d.y+gradeBudget*(path[32]-path[i]));
                        var localLo=Math.Max(lo,raw[i]-terrainBudget);var localHi=Math.Min(hi,raw[i]+terrainBudget);
                        if(localLo>localHi)continue; // Keep unrepairable relief visible to validation.
                        values[i]=math.clamp((previous[i-1]+2*previous[i]+previous[i+1])*.25f,localLo,localHi);
                    }
                }
            }
            var pieces = new List<PreparedSegment>();
            var fitted = FitPiece(0,1,0);
            var grade = pieces.Count == 0 ? float.PositiveInfinity : pieces.Max(MaximumGrade);
            if(!fitted || !math.isfinite(grade)||pieces.Any(piece=>!WithinGradeCeiling(piece,practicalLimit)))
            {
                // Preserve the horizontal alignment and shared endpoints. A linear
                // elevation plane avoids local least-squares spikes. If preferred
                // limits remain infeasible, preserve connectivity and warn, not abort.
                var delta=d.xz-a.xz;var square=math.lengthsq(delta);
                var c1=s.Control1??math.lerp(a,d,1f/3);var c2=s.Control2??math.lerp(a,d,2f/3);
                c1.y=square>.0001f?a.y+(d.y-a.y)*math.dot(c1.xz-a.xz,delta)/square:a.y;
                c2.y=square>.0001f?a.y+(d.y-a.y)*math.dot(c2.xz-a.xz,delta)/square:d.y;
                var fallback=s with{Start=a,End=d,Control1=c1,Control2=c2,StartElevation=a.y-Ground(a),EndElevation=d.y-Ground(d)};
                var budget=CutFillBudget(s);
                var terrainSafe=s.Bridge||s.Tunnel||Enumerable.Range(0,65).All(i=>{var p=NetworkNormalizer.Evaluate(fallback,i/64f);return Math.Abs(p.y-Ground(p))<=budget+.05f;});
                if(!terrainSafe||!WithinGradeCeiling(fallback,practicalLimit))
                {
                    rejected++;log.Count("Road profiles requiring repair");
                    // Do not solve a difficult ground road by creating an artificial bridge.
                    // Prefer the straight elevation plane with shared endpoints;
                    // report any roadbed mismatch instead of restoring terrain spikes.
                    // Retain the smooth elevation plane even when local terrain
                    // shaping cannot fully achieve it. Do not put DEM spikes back
                    // into the road or silently create another bridge. Roadbed
                    // protections/budgets still apply, and this limitation is logged.
                    pieces.Clear();pieces.Add(fallback);grade=MaximumGrade(fallback);
                    log.Count("Road profiles retained with relaxed limits");
                    if(rejected<=25)log.Warn($"Road way {s.WayId}: preferred grade/terrain budget infeasible; retained connected smooth-elevation fallback, grade={grade*100:0.0}%, target={practicalLimit*100:0.0}%. Terrain budget satisfied={terrainSafe}; roadbed may need review. No road deleted or converted to bridge; preparation continues.");
                    log.Diagnostic($"ROAD PROFILE RECOVERY way={s.WayId}; startNode={s.StartNodeId}; endNode={s.EndNodeId}; classification={s.Classification}; bridge={s.Bridge}; tunnel={s.Tunnel}; lengthMeters={path[32]:0.###}; startHeight={a.y:0.###}; endHeight={d.y:0.###}; targetGrade={practicalLimit:0.###}; retainedGrade={grade:0.###}; linearTerrainBudgetSatisfied={terrainSafe}; fallback=smooth-elevation; sharedEndpointsRetained=true; omitted=false");
                }
                else
                {
                    pieces.Clear();pieces.Add(fallback);grade=MaximumGrade(fallback);log.Count("Road profiles repaired without deletion");
                }
            }
            if((s.Classification.StartsWith("motorway",StringComparison.Ordinal)||s.Classification.StartsWith("trunk",StringComparison.Ordinal))&&grade>.12f)log.Count("Highway profiles requiring source review");
            maximum=Math.Max(maximum,grade);result.AddRange(pieces);log.Count(s.Bridge?"Bridge profiles fitted":s.ProtectedLink?"Ramp profiles fitted":"Road profiles fitted");

            float Target(float t){var p=t*32;var i=Math.Min(31,(int)p);return math.lerp(values[i],values[i+1],p-i);}
            long Node(float t){if(t==0)return s.StartNodeId;if(t==1)return s.EndNodeId;unchecked{return -1-(long)(((ulong)s.WayId*397UL ^ (ulong)s.StartNodeId*104729UL ^ (ulong)s.EndNodeId*1000003UL ^ math.hash(NetworkNormalizer.Evaluate(s,t).xz))%(ulong)long.MaxValue);}}
            bool FitPiece(float first,float last,int depth)
            {
                token.ThrowIfCancellationRequested();
                var curve=Subcurve(s,first,last);var start=curve.Start;var end=curve.End;start.y=Target(first);end.y=Target(last);
                double aa=0,ab=0,bb=0,ar=0,br=0;
                for(var i=1;i<32;i++){var t=i/32d;var u=1-t;var b1=3*u*u*t;var b2=3*u*t*t;var residual=Target(math.lerp(first,last,(float)t))-u*u*u*start.y-t*t*t*end.y;aa+=b1*b1;ab+=b1*b2;bb+=b2*b2;ar+=b1*residual;br+=b2*residual;}
                var determinant=aa*bb-ab*ab;var c1=curve.Control1!.Value;var c2=curve.Control2!.Value;
                c1.y=(float)((ar*bb-br*ab)/determinant);c2.y=(float)((br*aa-ar*ab)/determinant);
                var candidate=curve with{Start=start,End=end,Control1=c1,Control2=c2,StartNodeId=Node(first),EndNodeId=Node(last),StartRoundaboutRadius=first==0?s.StartRoundaboutRadius:0,EndRoundaboutRadius=last==1?s.EndRoundaboutRadius:0,StartElevation=start.y-Ground(start),EndElevation=end.y-Ground(end),HorizontalParent=first==0&&last==1?null:s,ParentStartT=first,ParentEndT=last};
                var error=0f;for(var i=0;i<=32;i++)error=Math.Max(error,Math.Abs(NetworkNormalizer.Evaluate(candidate,i/32f).y-Target(math.lerp(first,last,i/32f))));
                // Fall back to less aggressive local fitting rather than flatten broad relief.
                if((error>2 || !WithinGradeCeiling(candidate,practicalLimit)) && depth<3 && ArcLength(candidate)>40)
                {log.Count("Vertical profiles locally refined");var middle=(first+last)*.5f;return FitPiece(first,middle,depth+1)&&FitPiece(middle,last,depth+1);}
                if(error>3)return false;pieces.Add(candidate);return true;
            }
        }
        if(rejected>0)log.Warn($"Road profile recovery: {rejected}/{roads.Length} sections retained with relaxed quality limits. Preparation continues; grade/terrain quality is not fully verified.");
        SmoothVerticalJoins(result,log,token);
        var fittedByRoot=result.Where(s=>s.Kind==NetworkKind.Road).GroupBy(s=>
            (s.WayId,(s.HorizontalParent??s).StartNodeId,(s.HorizontalParent??s).EndNodeId)).ToDictionary(g=>g.Key,g=>g.ToArray());
        float FittedHeight(PreparedSegment original,float t)
        {
            if(original.Kind==NetworkKind.Rail)return NetworkNormalizer.Evaluate(original,t).y;
            if(!fittedByRoot.TryGetValue((original.WayId,original.StartNodeId,original.EndNodeId),out var pieces))return float.NaN;
            foreach(var piece in pieces)
            {
                var first=piece.HorizontalParent==null?0:piece.ParentStartT;var last=piece.HorizontalParent==null?1:piece.ParentEndT;
                if(t>=first-.0001f&&t<=last+.0001f)return NetworkNormalizer.Evaluate(piece,math.clamp((t-first)/(last-first),0,1)).y;
            }
            return float.NaN;
        }
        var fittedClearanceIssues=clearanceConstraints.Count(c=>
        {var gap=FittedHeight(c.upper,c.t)-FittedHeight(c.lower,c.u);return !math.isfinite(gap)||gap<MinimumBridgeClearance+2-.05f;});
        log.Count("Final fitted bridge clearance issues",fittedClearanceIssues);
        log.Diagnostic($"ROAD FINAL AUDIT crossingConstraints={clearanceConstraints.Count}; fittedDeckClearanceIssues={fittedClearanceIssues}; targetClearSpaceMeters=5; conservativeDeckAllowanceMeters=2; curveHeightsChecked=true; repeatedUnboundedLifting=false; renderedClearanceRequiresGameCheck=true");
        maximum=result.Where(s=>s.Kind==NetworkKind.Road).Select(MaximumGrade).DefaultIfEmpty(0).Max();
        var sourceDegree=new Dictionary<long,int>();var retainedDegree=new Dictionary<long,int>();
        void Degrees(IEnumerable<PreparedSegment> segments,Dictionary<long,int> counts)
        {foreach(var road in segments.Where(s=>s.Kind==NetworkKind.Road))foreach(var node in new[]{road.StartNodeId,road.EndNodeId}){counts.TryGetValue(node,out var count);counts[node]=count+1;}}
        Degrees(source,sourceDegree);Degrees(result,retainedDegree);
        var exposed=retainedDegree.Count(p=>p.Value==1&&sourceDegree.TryGetValue(p.Key,out var before)&&before>1);
        log.Count("Road endpoints exposed by profile rejection",exposed);
        if(exposed>0)log.Warn($"Road grading left {exposed} formerly connected source endpoints open after unsafe sections were omitted; network continuity is partial, not verified success.");
        if(roads.Length>0)log.Diagnostic($"PROCESS road-grading complete; retained={roads.Length}; rejected=0; relaxedProfiles={rejected}; maximumRetainedGrade={maximum*100:0.0}%; elapsedSeconds={clock.Elapsed.TotalSeconds:0.0}; practicalLimitsEnforced={rejected==0}; generationContinues=true");
        return result;
    }
    private static void SmoothVerticalJoins(List<PreparedSegment> sections,ImportLog log,CancellationToken token)
    {
        var joins=new Dictionary<long,List<(int index,bool start)>>();
        for(var i=0;i<sections.Count;i++)if(sections[i].Kind==NetworkKind.Road)
            foreach(var start in new[]{true,false}){var id=start?sections[i].StartNodeId:sections[i].EndNodeId;if(!joins.TryGetValue(id,out var entries))joins[id]=entries=new List<(int,bool)>();entries.Add((i,start));}
        var smoothed=0;
        foreach(var entries in joins.Values)
        {
            token.ThrowIfCancellationRequested();if(entries.Count!=2||entries[0].index==entries[1].index)continue;
            var left=entries[0];var right=entries[1];var a=sections[left.index];var b=sections[right.index];
            if(a.Bridge||b.Bridge)continue; // Final deck profile must not acquire a sag.
            if(a.ExplicitIncline.HasValue||b.ExplicitIncline.HasValue||a.Classification!=b.Classification||a.OneWay!=b.OneWay)continue;
            var point=left.start?a.Start:a.End;if(math.distance(point,right.start?b.Start:b.End)>.01f)continue;
            var ca=left.start?a.Control1??math.lerp(a.Start,a.End,1f/3):a.Control2??math.lerp(a.Start,a.End,2f/3);
            var cb=right.start?b.Control1??math.lerp(b.Start,b.End,1f/3):b.Control2??math.lerp(b.Start,b.End,2f/3);
            if(math.dot(math.normalizesafe(point.xz-ca.xz),math.normalizesafe(cb.xz-point.xz))<.866f)continue;
            var lengthA=ArcLength(a);var lengthB=ArcLength(b);if(lengthA+lengthB<.01f)continue;
            var slope=((right.start?b.End.y:b.Start.y)-(left.start?a.End.y:a.Start.y))/(lengthA+lengthB);
            var ceiling=Math.Min(GradeLimit(a),GradeLimit(b));slope=math.clamp(slope,-ceiling*.8f,ceiling*.8f);
            ca.y=point.y-slope*math.distance(ca.xz,point.xz);cb.y=point.y+slope*math.distance(cb.xz,point.xz);
            var newA=left.start?a with{Control1=ca}:a with{Control2=ca};var newB=right.start?b with{Control1=cb}:b with{Control2=cb};
            if(!WithinGradeCeiling(newA,GradeLimit(a))||!WithinGradeCeiling(newB,GradeLimit(b)))continue;
            bool Bounded(PreparedSegment old,PreparedSegment revised)=>Enumerable.Range(0,33).All(i=>Math.Abs(NetworkNormalizer.Evaluate(old,i/32f).y-NetworkNormalizer.Evaluate(revised,i/32f).y)<=2);
            if(!Bounded(a,newA)||!Bounded(b,newB))continue;
            sections[left.index]=newA;sections[right.index]=newB;smoothed++;
        }
        log.Count("Road vertical curve joins smoothed",smoothed);
        log.Diagnostic($"PROCESS vertical-tangent-continuity complete; joinsSmoothed={smoothed}; gradeCeilingsEnforced=true; includesSharedStructurePortals=true; endpointsAndStructureTypesUnchanged=true");
    }
    public static float ArcLength(PreparedSegment segment)
    {float length=0;var previous=NetworkNormalizer.Evaluate(segment,0);for(var i=1;i<=64;i++){var next=NetworkNormalizer.Evaluate(segment,i/64f);length+=math.distance(previous.xz,next.xz);previous=next;}return length;}
    private static bool TryCrossing(PreparedSegment a,PreparedSegment b,out float t,out float u)
    {
        t=u=0;
        float Cross(float2 x,float2 y)=>x.x*y.y-x.y*y.x;
        for(var i=0;i<16;i++)for(var j=0;j<16;j++)
        {
            var p=NetworkNormalizer.Evaluate(a,i/16f).xz;var q=NetworkNormalizer.Evaluate(b,j/16f).xz;
            var r=NetworkNormalizer.Evaluate(a,(i+1)/16f).xz-p;var s=NetworkNormalizer.Evaluate(b,(j+1)/16f).xz-q;
            var denominator=Cross(r,s);if(Math.Abs(denominator)<.0001f)continue;
            var x=Cross(q-p,s)/denominator;var y=Cross(q-p,r)/denominator;
            if(x<0||x>1||y<0||y>1)continue;
            t=(i+x)/16;u=(j+y)/16;
            if(t>.005f&&t<.995f&&u>.005f&&u<.995f)return true;
        }
        return false;
    }
    private static PreparedSegment Subcurve(PreparedSegment s,float first,float last)
    {
        var a=s.Start;var b=s.Control1??math.lerp(s.Start,s.End,1f/3);var c=s.Control2??math.lerp(s.Start,s.End,2f/3);var d=s.End;
        var ab=math.lerp(a,b,last);var bc=math.lerp(b,c,last);var cd=math.lerp(c,d,last);var abc=math.lerp(ab,bc,last);var bcd=math.lerp(bc,cd,last);var end=math.lerp(abc,bcd,last);
        var ratio=first/last;var leftAB=math.lerp(a,ab,ratio);var leftBC=math.lerp(ab,abc,ratio);var leftCD=math.lerp(abc,end,ratio);var leftABC=math.lerp(leftAB,leftBC,ratio);var leftBCD=math.lerp(leftBC,leftCD,ratio);var start=math.lerp(leftABC,leftBCD,ratio);
        return s with{Start=start,End=end,Control1=leftBCD,Control2=leftCD};
    }
    private static float Bilinear(float x,float z,ushort[] raster,float scale,double width)
    {
        if(width<=0||!math.isfinite(scale)||scale<=0||raster.Length!=TerrainBuilder.Resolution*TerrainBuilder.Resolution)throw new InvalidOperationException("Road profile: invalid terrain buffer/scale.");
        var px=Numeric.Clamp((x/width+.5)*(TerrainBuilder.Resolution-1),0,TerrainBuilder.Resolution-1);var py=Numeric.Clamp((z/width+.5)*(TerrainBuilder.Resolution-1),0,TerrainBuilder.Resolution-1);
        var ix=(int)Math.Floor(px);var iy=(int)Math.Floor(py);var jx=Math.Min(ix+1,TerrainBuilder.Resolution-1);var jy=Math.Min(iy+1,TerrainBuilder.Resolution-1);var fx=(float)(px-ix);var fy=(float)(py-iy);
        return math.lerp(math.lerp(raster[iy*TerrainBuilder.Resolution+ix],raster[iy*TerrainBuilder.Resolution+jx],fx),math.lerp(raster[jy*TerrainBuilder.Resolution+ix],raster[jy*TerrainBuilder.Resolution+jx],fx),fy)/ushort.MaxValue*scale;
    }
}
