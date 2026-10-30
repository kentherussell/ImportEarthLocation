using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Topology -> horizontal shape -> lane ports -> one vertical solve. The caller
// builds terrain support afterwards; validators must not rewrite this design.
public static class CoordinatedNetworkDesign
{
    private readonly record struct Key(NetworkKind Kind,long Node);
    private sealed class Route
    {
        public readonly List<int> Edges=new();
        public readonly List<Key> Nodes=new();
        public readonly List<float> Station=new();
    }
    private static bool IsMainline(PreparedSegment s)=>s.Kind==NetworkKind.Rail||s.Classification is "motorway" or "trunk";
    private static bool Same(PreparedSegment a,PreparedSegment b)=>a.Kind==b.Kind&&a.OneWay==b.OneWay&&a.ProtectedLink==b.ProtectedLink&&
        (a.Kind==NetworkKind.Rail||a.WayId==b.WayId||a.Reference.Length>0&&a.Reference==b.Reference||a.Name.Length>0&&a.Name==b.Name||a.CorridorId!=0&&a.CorridorId==b.CorridorId);
    private static bool ExplicitStructure(PreparedSegment s)=>s.RawSourceTags.ContainsKey("bridge")||s.RawSourceTags.ContainsKey("tunnel")||
        s.RawSourceTags.ContainsKey("layer")||s.SourceTags.ContainsKey("bridge")||s.SourceTags.ContainsKey("tunnel")||s.SourceTags.ContainsKey("layer");

    public static IReadOnlyList<PreparedSegment> TunnelIntervals(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token)
    {
        var result=source.ToArray();var nodes=new Dictionary<(NetworkKind,long),List<int>>();
        for(var i=0;i<result.Length;i++)foreach(var id in new[]{result[i].StartNodeId,result[i].EndNodeId})
        {var key=(result[i].Kind,id);if(!nodes.TryGetValue(key,out var list))nodes[key]=list=new();list.Add(i);}
        var filled=0;var visited=new HashSet<int>();
        for(var seed=0;seed<result.Length;seed++)
        {
            var first=result[seed];if(first.Tunnel||first.Bridge||first.ProtectedLink||!IsMainline(first)||ExplicitStructure(first)||visited.Contains(seed))continue;
            token.ThrowIfCancellationRequested();var run=new HashSet<int>{seed};var boundary=new List<int>();var length=NetworkVerticalProfiles.ArcLength(first);
            foreach(var end in new[]{first.StartNodeId,first.EndNodeId})
            {
                var current=seed;var node=end;
                while(length<=220)
                {
                    if(nodes[(first.Kind,node)].Count!=2)break;
                    var options=nodes[(first.Kind,node)].Where(i=>i!=current&&!result[i].ProtectedLink&&result[i].OneWay==first.OneWay&&result[i].Classification==first.Classification).ToArray();
                    if(options.Length!=1)break;var next=options[0];if(run.Contains(next))break;
                    var a=result[current];var b=result[next];
                    if(a.OneWay&&(a.StartNodeId==node)==(b.StartNodeId==node)||
                        NetworkBoundaryPolicy.EndpointMarker(a,a.StartNodeId==node)||NetworkBoundaryPolicy.EndpointMarker(b,b.StartNodeId==node))break;
                    if(result[next].Tunnel){boundary.Add(next);break;}if(result[next].Bridge)break;
                    if(ExplicitStructure(result[next]))break;
                    run.Add(next);length+=NetworkVerticalProfiles.ArcLength(result[next]);current=next;node=result[next].StartNodeId==node?result[next].EndNodeId:result[next].StartNodeId;
                }
            }
            visited.UnionWith(run);if(length>220||boundary.Count!=2||result[boundary[0]].Layer!=result[boundary[1]].Layer)continue;
            foreach(var index in run){result[index]=result[index] with{Tunnel=true,Bridge=false,Layer=Math.Min(result[boundary[0]].Layer,result[boundary[1]].Layer)};filled++;}
        }
        var underground=new HashSet<long>(result.Where(s=>s.Kind==NetworkKind.Road&&s.Tunnel).SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}));
        var excluded=new HashSet<int>();var pending=new List<int>();
        for(var i=0;i<result.Length;i++)
            if(result[i].ProtectedLink&&(underground.Contains(result[i].StartNodeId)||underground.Contains(result[i].EndNodeId)))
                if(excluded.Add(i))pending.Add(i);
        for(var k=0;k<pending.Count;k++)
        {
            token.ThrowIfCancellationRequested();var current=result[pending[k]];
            foreach(var id in new[]{current.StartNodeId,current.EndNodeId})
            {
                var incident=nodes[(NetworkKind.Road,id)];if(incident.Count!=2)continue;
                foreach(var next in incident)if(result[next].ProtectedLink&&excluded.Add(next))pending.Add(next);
            }
        }
        log.Count("Short tunnel surface interruptions absorbed",filled);log.Count("Underground degree-two link sections intentionally omitted",excluded.Count);
        log.Diagnostic($"TUNNEL INTERVALS surfaceSectionsAbsorbed={filled}; maximumInterruptionMeters=220; undergroundLinkSectionsExcluded={excluded.Count}; branchesStopOmission=true; geographicOverrides=false");
        return result.Where((s,i)=>!excluded.Contains(i)).ToArray();
    }

    public static IReadOnlyList<PreparedSegment> StructureIntervals(IReadOnlyList<PreparedSegment> source,ushort[] raster,float scale,double width,
        ImportLog log,CancellationToken token,Func<float3,float?>? water=null,bool tunnelIntervalsResolved=false)
    {
        var edges=(tunnelIntervalsResolved?source:TunnelIntervals(source,log,token)).ToArray();
        var waterBridges=0;
        if(water!=null)for(var i=0;i<edges.Length;i++)
        {
            var s=edges[i];if(s.Tunnel||s.Bridge||ExplicitStructure(s))continue;
            var samples=s.Kind==NetworkKind.Road?Math.Max(8,(int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(s)/10)):8;
            for(var j=0;j<=samples;j++)
            {
                var t=j/(float)samples;var p=NetworkNormalizer.Evaluate(s,t);
                var direction=NetworkNormalizer.Evaluate(s,Math.Min(1,t+.001f))-NetworkNormalizer.Evaluate(s,Math.Max(0,t-.001f));
                var normal=math.normalizesafe(new float3(-direction.z,0,direction.x))*Math.Max(4,s.NativeWidth*.5f);
                var crossing=false;
                for(var side=s.Kind==NetworkKind.Road?-1:0;side<=(s.Kind==NetworkKind.Road?1:0);side++)
                {
                    var q=p+normal*side;var stage=water(q);if(!stage.HasValue||!math.isfinite(stage.Value))continue;
                    var ground=NetworkProcessor.Sample(q.x,q.z,raster,scale,width);
                    if(s.Kind==NetworkKind.Road?stage.Value-ground<WaterLevels.RoadBridgeDepthMeters:ground>stage.Value+.5f)continue;
                    crossing=true;break;
                }
                if(crossing){edges[i]=s with{Bridge=true};if(s.Kind==NetworkKind.Road)waterBridges++;break;}
            }
        }
        if(water!=null)
        {
            log.Count("Road bridges inferred from planned water depth",waterBridges);
            log.Diagnostic($"ROAD WATER POLICY bridgeDepthMeters={WaterLevels.RoadBridgeDepthMeters}; groundFreeboardMeters={WaterLevels.RoadFreeboardMeters}; inferredBridges={waterBridges}; roadEdgesSampled=true; explicitStructuresRetained=true; nativeWaterObserved=false");
        }
        return edges;
    }

    public static IReadOnlyList<PreparedSegment> Prepare(IReadOnlyList<PreparedSegment> source,ushort[] raster,float scale,double width,
        ImportLog log,CancellationToken token,Func<float3,float?>? water=null,bool structureIntervalsResolved=false,
        IReadOnlyDictionary<string,NativeAssetContract>? assets=null,Action<string>? progress=null,
        IReadOnlyList<float3[]>? protectedAreas=null,IReadOnlyList<float3[]>? protectedLines=null,SourceTopologyManifest? sourceTopology=null,
        Action<string,IReadOnlyList<PreparedSegment>>? inspectStage=null,
        Action<string,SparseProfileRows>? inspectProfileRows=null,Action<object>? inspectProfileSeam=null,
        Action<string,IReadOnlyList<PreparedSegment>>? inspectWriter=null)
    {
        var invalid=source.Count(s=>!NetworkDesignChecks.Finite(new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd))||!math.all(math.isfinite(s.Start))||!math.all(math.isfinite(s.End)));
        if(invalid>0)
        {
            log.Count("Nonfinite source sections excluded for native safety",invalid);
            log.Warn($"Native safety: {invalid} nonfinite source sections cannot be represented; continuing the remaining network.");
            var finite=source.Where(s=>NetworkDesignChecks.Finite(new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd))&&math.all(math.isfinite(s.Start))&&math.all(math.isfinite(s.End))).ToArray();
            return Prepare(finite,raster,scale,width,log,token,water,structureIntervalsResolved,assets,progress,
                protectedAreas,protectedLines,sourceTopology,inspectStage,inspectProfileRows,inspectProfileSeam,inspectWriter);
        }
        if(source.Count==0)return source;
        var stageClock=System.Diagnostics.Stopwatch.StartNew();
        void Stage(string name,IReadOnlyList<PreparedSegment> sections)
        {
            var seconds=stageClock.Elapsed.TotalSeconds;
            log.AddDuration("Coordinated "+name,seconds);
            log.Diagnostic($"DESIGN STAGE name={name}; elapsedSeconds={seconds:F6}; sections={sections.Count}; scope=coordinated-design; includesDiagnosticOverhead=true; boundaryCensusTimedSeparately=true");
            NetworkBoundaryPolicy.Survey(sections,sourceTopology,log,name,assets);
            inspectWriter?.Invoke(name,sections);
            inspectStage?.Invoke(name,sections);
            stageClock.Restart();
        }
        inspectWriter?.Invoke("coordinated-input",source);
        source=NetworkRailTerminalFormations.Prepare(source,log,token);
        inspectWriter?.Invoke("rail-terminal-formations",source);
        source=RailCorridorConsolidator.NativeJunctionFootprints(source,log,token);
        inspectWriter?.Invoke("rail-junction-footprints",source);
        var edges=(structureIntervalsResolved?source:StructureIntervals(source,raster,scale,width,log,token,water))
            .OrderBy(s=>s.Kind).ThenBy(s=>s.WayId).ThenBy(s=>s.StartNodeId).ThenBy(s=>s.EndNodeId).ToArray();
        var graph=new Dictionary<Key,List<int>>();var positions=new Dictionary<Key,float3>();
        Key A(PreparedSegment s)=>new(s.Kind,s.StartNodeId);Key B(PreparedSegment s)=>new(s.Kind,s.EndNodeId);
        float Ground(float3 p)=>NetworkProcessor.Sample(p.x,p.z,raster,scale,width);
        // Certify tag consistency against the immutable terrain/reference span
        // before smoothing or taper subdivision changes its endpoints/length.
        for(var i=0;i<edges.Length;i++)
        {
            var s=edges[i];if(s.Kind!=NetworkKind.Road||!s.ExplicitIncline.HasValue)continue;
            edges[i]=s with{SignedInclinePriorAccepted=math.isfinite(s.ExplicitIncline.Value)&&
                Math.Abs(s.ExplicitIncline.Value*NetworkVerticalProfiles.ArcLength(s)-(Ground(s.End)-Ground(s.Start)))<=3};
        }
        for(var i=0;i<edges.Length;i++)foreach(var entry in new[]{(A(edges[i]),edges[i].Start),(B(edges[i]),edges[i].End)})
        {if(!graph.TryGetValue(entry.Item1,out var list))graph[entry.Item1]=list=new();list.Add(i);positions[entry.Item1]=entry.Item2;}
        // The reference datum is terrain, never an earlier repair's raised deck.
        foreach(var key in positions.Keys.ToArray())
        {
            var p=positions[key];var incident=graph[key].Select(i=>edges[i]).ToArray();
            var offset=incident.All(s=>s.Tunnel)?-15:incident.Any(s=>s.Tunnel)?-2:0;
            p.y=math.clamp(Ground(p)+offset,0,scale);positions[key]=p;
        }
        int Next(int index,Key node)
        {
            var s=edges[index];var start=A(s)==node;var arrival=math.normalizesafe(start?s.Start.xz-s.End.xz:s.End.xz-s.Start.xz);
            var best=-1;var score=.8f;
            foreach(var j in graph[node])
            {
                var t=edges[j];if(j==index||!Continues(s,t,node)||s.OneWay&&(A(t)==node)==start)continue;
                var departure=math.normalizesafe(A(t)==node?t.End.xz-t.Start.xz:t.Start.xz-t.End.xz);
                var dot=math.dot(arrival,departure);if(dot>score){best=j;score=dot;}
            }
            return best;
        }
        bool Continues(PreparedSegment a,PreparedSegment b,Key node)=>
            !NetworkBoundaryPolicy.EndpointMarker(a,A(a)==node)&&!NetworkBoundaryPolicy.EndpointMarker(b,A(b)==node)&&
            (Same(a,b)||graph[node].Count==2&&NativeChainCompactor.Compatible(a,b));
        var routes=new List<Route>();var seen=new HashSet<int>();
        foreach(var seed in Enumerable.Range(0,edges.Length).OrderBy(i=>edges[i].ProtectedLink?1:0).ThenBy(i=>edges[i].WayId).ThenBy(i=>i))
        {
            if(seen.Contains(seed))continue;token.ThrowIfCancellationRequested();
            var first=A(edges[seed]);var reverse=seed;var backwards=new HashSet<int>{seed};
            while(true){var next=Next(reverse,first);if(next<0||seen.Contains(next)||!backwards.Add(next))break;reverse=next;first=A(edges[next])==first?B(edges[next]):A(edges[next]);}
            var route=new Route();route.Nodes.Add(first);route.Station.Add(0);var current=reverse;var node=first;
            while(seen.Add(current))
            {
                route.Edges.Add(current);var e=edges[current];node=A(e)==node?B(e):A(e);route.Nodes.Add(node);
                route.Station.Add(route.Station[route.Station.Count-1]+Math.Max(1,NetworkVerticalProfiles.ArcLength(e)));
                var next=Next(current,node);if(next<0||seen.Contains(next))break;current=next;
            }
            routes.Add(route);
        }
        var reference=new Dictionary<Key,float3>(positions);var smoothed=0;
        foreach(var route in routes)
        {
            var first=edges[route.Edges[0]];var span=IsMainline(first)?500f:100f;
            var lineA=reference[route.Nodes[0]].xz;var lineB=reference[route.Nodes[route.Nodes.Count-1]].xz;var line=lineB-lineA;
            // This is a source-geometry writer, so it must use the same budget
            // as the downstream shared fitter. The former doubled allowance
            // moved knots outside that fitter's admissible source envelope,
            // then left later reconstruction/compaction rejecting the damage.
            var lineSquare=math.lengthsq(line);var tolerance=NetworkNormalizer.Tolerance(first.Kind);
            bool NearRouteLine(float2 q)=>lineSquare>1&&math.distance(q,lineA+line*math.clamp(math.dot(q-lineA,line)/lineSquare,0,1))<=tolerance;
            var straighten=!first.Roundabout&&route.Nodes.All(k=>NearRouteLine(reference[k].xz));
            for(var j=1;j<route.Nodes.Count-1;j++)
            {
                token.ThrowIfCancellationRequested();var key=route.Nodes[j];if(graph[key].Count!=2)continue;
                var p=reference[key];var left=j;while(left>0&&route.Station[j]-route.Station[left-1]<=span)left--;
                var right=j;while(right+1<route.Nodes.Count&&route.Station[right+1]-route.Station[j]<=span)right++;
                double sw=0,sx=0,sxx=0,sy=0,sxy=0;
                for(var k=left;k<=right;k++)
                {
                    var x=route.Station[k]-route.Station[j];var support=(k>0?route.Station[k]-route.Station[k-1]:0)+(k+1<route.Nodes.Count?route.Station[k+1]-route.Station[k]:0);
                    var w=Math.Max(.01,1-Math.Abs(x)/(span+1))*Math.Max(1,support);
                    sw+=w;sx+=w*x;sxx+=w*x*x;sy+=w*reference[route.Nodes[k]].y;sxy+=w*x*reference[route.Nodes[k]].y;
                }
                var denominator=sw*sxx-sx*sx;if(denominator>1&&!graph[key].Any(i=>edges[i].Tunnel))p.y=(float)((sy*sxx-sxy*sx)/denominator);
                // Straighten only an entire simple run; do not repeatedly project
                // individual rail knots onto a moving window chord.
                if(straighten)p.xz=lineA+line*math.clamp(math.dot(p.xz-lineA,line)/lineSquare,0,1);
                positions[key]=p;smoothed++;
            }
        }
        for(var i=0;i<edges.Length;i++)
        {
            var s=edges[i];var a=positions[A(s)];var b=positions[B(s)];var c=s.CourseControl1+(a-s.Start);var d=s.CourseControl2+(b-s.End);
            c.y=math.lerp(a.y,b.y,1f/3);d.y=math.lerp(a.y,b.y,2f/3);
            edges[i]=s with{Start=a,End=b,Control1=c-s.StartCurveOffset,Control2=d-s.EndCurveOffset,FinalDeckCurve=null,HorizontalParent=null};
        }
        foreach(var node in graph.Where(p=>p.Value.Count==2))
        {
            var x=node.Value[0];var y=node.Value[1];var a=edges[x];var b=edges[y];if(!Continues(a,b,node.Key)||a.Roundabout||b.Roundabout)continue;
            var p=positions[node.Key];var xa=A(a)==node.Key;var yb=A(b)==node.Key;
            var awayA=(xa?a.End:a.Start)-p;var awayB=(yb?b.End:b.Start)-p;
            var da=math.normalizesafe(awayA.xz);var db=math.normalizesafe(awayB.xz);if(math.dot(da,db)>-.5f)continue;
            var axis=math.normalizesafe(da-db);var ca=p+new float3(axis.x,0,axis.y)*(math.length(awayA.xz)/3);
            var cb=p-new float3(axis.x,0,axis.y)*(math.length(awayB.xz)/3);
            ca.y=p.y+awayA.y/3;cb.y=p.y+awayB.y/3;
            edges[x]=xa?a with{Control1=ca}:a with{Control2=ca};edges[y]=yb?b with{Control1=cb}:b with{Control2=cb};
        }
        var railDirections=new List<NetworkApproachTapers.DirectionRequest>();
        if(protectedAreas!=null&&protectedLines!=null)
            edges=NetworkVisualDesign.FitBudgets(edges,width,protectedAreas,protectedLines,water,log,token).ToArray();
        foreach(var node in graph.Where(p=>p.Key.Kind==NetworkKind.Rail&&p.Value.Count>=3))
        {
            var p=positions[node.Key];var ports=node.Value.OrderBy(i=>edges[i].WayId).ThenBy(i=>edges[i].StartNodeId).ThenBy(i=>edges[i].EndNodeId).Select(i=>(Index:i,Start:A(edges[i])==node.Key,
                Away:math.normalizesafe(((A(edges[i])==node.Key?edges[i].End:edges[i].Start)-p).xz))).ToArray();
            var best=1f;var bestPriority=int.MaxValue;var axis=ports[0].Away;var ambiguous=false;
            for(var i=0;i<ports.Length;i++)for(var j=i+1;j<ports.Length;j++)
            {
                var dot=math.dot(ports[i].Away,ports[j].Away);var candidate=math.normalizesafe(ports[i].Away-ports[j].Away);
                if(dot>-.5f)continue;
                var a=edges[ports[i].Index];var b=edges[ports[j].Index];
                var continuation=a.WayId==b.WayId||a.SourceWayIds.Contains(b.WayId)||b.SourceWayIds.Contains(a.WayId)||a.SourceWayIds.Intersect(b.SourceWayIds).Any();
                int Service(PreparedSegment s)=>s.RailService is "yard" or "siding" or "spur"?1:0;
                var priority=(continuation?0:4)+Service(a)+Service(b);
                if(priority<bestPriority||priority==bestPriority&&dot<best-1e-6)
                {bestPriority=priority;best=dot;axis=candidate;ambiguous=false;}
                else if(priority==bestPriority&&Math.Abs(dot-best)<=1e-6&&Math.Abs(math.dot(axis,candidate))<.999f)ambiguous=true;
            }
            if(ambiguous){log.Count("Rail junction axes requiring source review");continue;}
            if(best>-.5f)continue; // No proven through track: preserve the source junction.
            foreach(var port in ports)
            {
                var s=edges[port.Index];var tangent=math.dot(port.Away,axis)>=0?axis:-axis;
                railDirections.Add(new(port.Index,port.Start,tangent));
            }
        }
        progress?.Invoke($"Simplifying {edges.Length:N0} road and railway spans");
        log.Diagnostic($"DESIGN INPUT routes={routes.Count}; sections={edges.Length}; sourceReferencePoints={source.Sum(s=>s.SourceGeometry.Count)}; rawEvidenceRetained=true; compatibleUnnamedWaysContinuous=true; nativeContractCount={assets?.Count??0}; formationOwnerUnchanged=true");
        Stage("route-and-junction-frames",edges);
        var approaches=NetworkApproachTapers.Materialize(edges,Array.Empty<NetworkApproachTapers.Shift>(),log,token,railDirections,includeLaneOffsets:false);
        Stage("turnout-approach-fit",approaches);
        if(protectedAreas!=null&&protectedLines!=null)
            approaches=NetworkVisualDesign.FitBudgets(approaches,width,protectedAreas,protectedLines,water,log,token);
        IReadOnlyList<PreparedSegment> horizontal=NativeChainCompactor.Prepare(approaches,log,token,sourceTopology);
        Stage("chain-compaction",horizontal);
        horizontal=NetworkHorizontalRegularity.Prepare(horizontal,log,token);
        horizontal=NetworkLaneAlignment.Prepare(horizontal,log,token,width,deferLinkApproaches:true);
        horizontal=PortalSpans(horizontal,token);
        Stage("regularity-lane-alignment-portals",horizontal);
        horizontal=NetworkPairedCorridors.Prepare(horizontal,log,token,progress,width,protectedAreas,protectedLines,assets,
            inspectPublication:inspectWriter);
        horizontal=NetworkLaneAlignment.MaterializeLinkApproaches(horizontal,log,token);
        Stage("paired-corridors",horizontal);
        horizontal=PortalSpans(NetworkVisualDesign.StructureIntent(horizontal,raster,scale,width,assets,log,token),token);
        horizontal=NetworkHorizontalRegularity.Prepare(horizontal,log,token);
        log.CaptureProfileInput(horizontal,raster,scale,width,assets);
        var cover=NetworkGeometrySafety.TunnelCoverSamples(horizontal,raster,scale,width,token,assets).ToArray();
        Stage("profile-input-and-cover",horizontal);
        progress?.Invoke("Building fixed slope, structure-clearance and tunnel-cover equations");
        NetworkTunnelProfile.PhysicalQuality? physicalQuality=null;
        var contactGeometry=new NetworkPhysicalContacts.GeometryCache();
        var result=NetworkTunnelProfile.Solve(horizontal,scale,cover,log,token,design:new NetworkTunnelProfile.TerrainReference(raster,width,water,assets,progress)
            {ContactGeometry=contactGeometry,InspectRows=inspectProfileRows==null?null:(scope,matrix)=>inspectProfileRows("initial/"+scope,matrix),InspectSeam=inspectProfileSeam},
            physicalQuality:value=>physicalQuality=value);
        Stage("profile-formulation-and-solve",result);
        result=NetworkPhysicalReconstruction.Once(horizontal,result,raster,scale,width,assets,water,log,token,progress,physicalQuality,inspectProfileRows,inspectProfileSeam);
        Stage("bounded-physical-reconstruction",result);
        progress?.Invoke("Certifying final road/rail physical outcomes against the immutable terrain");
        var finalPhysical=NetworkTunnelProfile.MeasurePhysical(result,raster,scale,width,assets,token,contactGeometry);
        log.Diagnostic($"CONTACT GEOMETRY CACHE builds={contactGeometry.Builds}; reuses={contactGeometry.Reuses}; scope=one-design; xzAndContractsChecked=true; heightResidualsAlwaysRecomputed=true");
        Stage("final-physical-certification",result);
        log.Quality(new("planned","full-width-clearance","final-design",(int)Math.Min(int.MaxValue,finalPhysical.ClearanceRows),
            (int)Math.Min(int.MaxValue,finalPhysical.ClearanceFailures),assets==null?1:finalPhysical.UncertifiedClearance,
            finalPhysical.MaximumClearanceResidual,finalPhysical.ClearanceAffectedLength,"Correlated native full-width footprint inequalities evaluated on final float controls; affected length counts each involved section once"));
        log.Quality(new("planned","continuous-tunnel-cover","final-design",(int)Math.Min(int.MaxValue,finalPhysical.CoverRows),
            (int)Math.Min(int.MaxValue,finalPhysical.CoverFailures),assets==null?1:finalPhysical.UncertifiedCover,
            finalPhysical.MaximumCoverResidual,finalPhysical.CoverAffectedLength,"Continuous encoded-terrain roof/portal rows; missing or uncertified coverage unavailable"));
        log.Quality(new("planned","same-level-overlap","final-design",result.Count,(int)Math.Min(int.MaxValue,finalPhysical.HorizontalConflictPairs),
            finalPhysical.UncertifiedClearance,finalPhysical.HorizontalConflictPairs,0,"Conservative overlap outside classified formations; never repaired by vertical stacking"));
        log.Diagnostic($"COORDINATED DESIGN routes={routes.Count}; smoothedOptionalStations={smoothed}; inputSections={source.Count}; outputSections={result.Count}; initialGlobalProfileSolves=1; exceptionalLocalReconstructionLimit=1; endpointEscalation=false; lateHorizontalWriters=false; sourceReferenceImmutable=true; railSegmentTracks=2; qualityFailureAction=RequiresReview");
        return result;
    }

    // A ground road or railway over a short culvert (WaterBodyPlanner.ShortCulverts)
    // or an untagged crossing of an open channel (WaterBodyPlanner.OpenChannels)
    // becomes a bridge across it. Otherwise water revalidation, which never
    // carves beside a ground road, leaves the river dammed by the embankment.
    // The span reaches past the channel (widened for an oblique crossing) by
    // the road's half width plus the two raster cells revalidation protects
    // beside a ground road, at most 50 m each side; a span ending within
    // 10 m of a section end takes the rest of the section.
    public static IReadOnlyList<PreparedSegment> CulvertBridges(IReadOnlyList<PreparedSegment> source,IReadOnlyList<(float3[] Line,float HalfWidth)> culverts,
        int resolution,double width,ImportLog log,CancellationToken token)
    {
        if(culverts.Count==0)return source;
        var cell=width/(resolution-1);
        // Channel pieces indexed by the 128 m grid cells their bounds touch, so
        // thousands of open waterways cost a lookup per section, not a scan.
        const double GridCell=128;
        var pieces=new List<(int Culvert,double2 A,double2 B)>();var grid=new Dictionary<(int,int),List<int>>();
        void Cells(double2 a,double2 b,Action<(int,int)> visit)
        {
            for(var x=(int)Math.Floor(Math.Min(a.x,b.x)/GridCell);x<=(int)Math.Floor(Math.Max(a.x,b.x)/GridCell);x++)
            for(var z=(int)Math.Floor(Math.Min(a.y,b.y)/GridCell);z<=(int)Math.Floor(Math.Max(a.y,b.y)/GridCell);z++)visit((x,z));
        }
        for(var c=0;c<culverts.Count;c++)for(var m=1;m<culverts[c].Line.Length;m++)
        {
            var a=(double2)culverts[c].Line[m-1].xz;var b=(double2)culverts[c].Line[m].xz;var index=pieces.Count;pieces.Add((c,a,b));
            Cells(a,b,key=>{if(!grid.TryGetValue(key,out var list))grid[key]=list=new List<int>();list.Add(index);});
        }
        var occupied=new HashSet<long>(source.SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}));var next=long.MinValue+40000000;
        long Id(){while(occupied.Contains(next))next++;occupied.Add(next);return next++;}
        var result=new List<PreparedSegment>();var bridged=0;var candidates=new HashSet<int>();
        foreach(var s in source)
        {
            token.ThrowIfCancellationRequested();
            if(s.Bridge||s.Tunnel){result.Add(s);continue;}
            var (points,at)=Polyline(s);var n=points.Length-1;
            candidates.Clear();for(var k=1;k<=n;k++)Cells(points[k-1],points[k],key=>{if(grid.TryGetValue(key,out var list))candidates.UnionWith(list);});
            if(candidates.Count==0){result.Add(s);continue;}
            var spans=new List<(double From,double To)>();
            foreach(var piece in candidates.OrderBy(i=>i))
            {
                var (c,c0,c1)=pieces[piece];
                for(var k=1;k<=n;k++)
                {
                    if(!Cross(points[k-1],points[k],c0,c1,out var u,out _))continue;
                    var sine=Math.Abs(NetworkDesignChecks.Cross(math.normalizesafe(points[k]-points[k-1]),math.normalizesafe(c1-c0)));
                    var reach=Math.Min(50,culverts[c].HalfWidth/Math.Max(.5,sine)+Math.Max(4,s.NativeWidth*.5)+2*cell);
                    var station=math.lerp(at[k-1],at[k],u);spans.Add((station-reach,station+reach));
                }            }
            if(spans.Count==0){result.Add(s);continue;}
            Bridged(s,at,spans,s.Layer,Id,result);bridged++;
        }
        log.Count("Sections bridged over short culverts",bridged);
        return result;
    }

    // Two ground highway-class roads (motorway, trunk or their links) crossing at
    // more than 10° without a shared node are grade separated; the map left
    // the upper one's bridge untagged (the I-495 express ramps over the
    // highway). At grade the game draws one through the other. The minor road
    // (a link before a mainline, then fewer lanes, then the higher way id)
    // becomes a bridge one layer above the other, over the other's half width
    // plus ClimbRoom either side, so the profile solver can lift it clear.
    public static IReadOnlyList<PreparedSegment> UnmappedCrossingBridges(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token)
    {
        const double ClimbRoom=30,Cell=64;
        static bool Highway(PreparedSegment s)=>s.Kind==NetworkKind.Road&&!s.Bridge&&!s.Tunnel&&(s.ProtectedLink||s.Classification is "motorway" or "trunk" or "motorway_link" or "trunk_link");
        var candidates=Enumerable.Range(0,source.Count).Where(i=>Highway(source[i])).ToArray();if(candidates.Length<2)return source;
        var lines=new Dictionary<int,(double2[] Points,double[] At)>();foreach(var i in candidates)lines[i]=Polyline(source[i]);
        var grid=new Dictionary<(int,int),List<(int Section,int K)>>();
        foreach(var i in candidates)for(var k=1;k<lines[i].Points.Length;k++)
        {var p=lines[i].Points[k];var key=((int)Math.Floor(p.x/Cell),(int)Math.Floor(p.y/Cell));if(!grid.TryGetValue(key,out var list))grid[key]=list=new();list.Add((i,k));}
        var spans=new Dictionary<int,List<(double From,double To)>>();var layers=new Dictionary<int,int>();var crossings=0;
        foreach(var cell in grid)foreach(var (i,k) in cell.Value)
        {
            token.ThrowIfCancellationRequested();
            for(var dz=-1;dz<=1;dz++)for(var dx=-1;dx<=1;dx++)if(grid.TryGetValue((cell.Key.Item1+dx,cell.Key.Item2+dz),out var near))foreach(var (j,m) in near)
            {
                // Each sample belongs to one cell, so a pair of samples is met once,
                // from the lower section; a set of visited pairs grew to hundreds of
                // millions in DC and Houston and ran out of memory.
                if(j<=i)continue;var a=source[i];var b=source[j];
                if(a.Layer!=b.Layer||a.StartNodeId==b.StartNodeId||a.StartNodeId==b.EndNodeId||a.EndNodeId==b.StartNodeId||a.EndNodeId==b.EndNodeId)continue;
                var (pa,sa)=lines[i];var (pb,sb)=lines[j];
                if(!Cross(pa[k-1],pa[k],pb[m-1],pb[m],out var u,out var v))continue;
                var da=math.normalizesafe(pa[k]-pa[k-1]);var db=math.normalizesafe(pb[m]-pb[m-1]);
                if(Math.Abs(NetworkDesignChecks.Cross(da,db))<Math.Sin(math.radians(10)))continue;
                // A crossing within 5 m of a section end is a junction mapped with separate nodes.
                var station=math.lerp(sa[k-1],sa[k],u);var t=math.lerp(sb[m-1],sb[m],v);
                if(station<5||station>sa[sa.Length-1]-5||t<5||t>sb[sb.Length-1]-5)continue;
                var minorIsA=a.ProtectedLink!=b.ProtectedLink?a.ProtectedLink:a.Lanes!=b.Lanes?a.Lanes<b.Lanes:a.WayId>b.WayId;
                var (minor,major,at)=minorIsA?(i,b,station):(j,a,t);
                var reach=Math.Max(4,major.NativeWidth*.5)/Math.Max(.5,Math.Abs(NetworkDesignChecks.Cross(da,db)))+ClimbRoom;
                if(!spans.TryGetValue(minor,out var list))spans[minor]=list=new();list.Add((at-reach,at+reach));
                layers[minor]=Math.Max(layers.TryGetValue(minor,out var l)?l:source[minor].Layer,major.Layer+1);crossings++;
            }
        }
        if(crossings==0)return source;
        var occupied=new HashSet<long>(source.SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}));var next=long.MinValue+42000000;
        long Id(){while(occupied.Contains(next))next++;occupied.Add(next);return next++;}
        var result=new List<PreparedSegment>();
        for(var i=0;i<source.Count;i++)
            if(spans.TryGetValue(i,out var list))Bridged(source[i],lines[i].At,list,layers[i],Id,result);else result.Add(source[i]);
        log.Count("Unmapped highway crossings bridged",crossings);
        return result;
    }

    // Segments ab and cd intersect at a + u(b − a) = c + v(d − c).
    private static bool Cross(double2 a,double2 b,double2 c,double2 d,out double u,out double v)
    {
        var r=b-a;var q=d-c;var den=r.x*q.y-r.y*q.x;u=v=0;if(Math.Abs(den)<1e-12)return false;
        var w=c-a;u=(w.x*q.y-w.y*q.x)/den;v=(w.x*r.y-w.y*r.x)/den;return u>=0&&u<=1&&v>=0&&v<=1;
    }

    // A section's course sampled every 2 m, with arc-length stations.
    private static (double2[] Points,double[] At) Polyline(PreparedSegment s)
    {
        var curve=new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);
        var n=Math.Max(2,(int)Math.Ceiling(NetworkDesignChecks.HorizontalArcLength(curve)/2));
        var points=new double2[n+1];var at=new double[n+1];
        for(var k=0;k<=n;k++){points[k]=NetworkDesignChecks.Evaluate(curve,k/(double)n).xz;if(k>0)at[k]=at[k-1]+math.distance(points[k-1],points[k]);}
        return (points,at);
    }

    // Splits a section (stations from Polyline) so the spans become bridge
    // pieces at the given layer; spans are merged when within 10 m, and a span
    // ending within 10 m of a section end takes the rest of the section.
    private static void Bridged(PreparedSegment s,double[] at,List<(double From,double To)> spans,int layer,Func<long> id,List<PreparedSegment> result)
    {
        var n=at.Length-1;var total=at[n];var merged=new List<(double From,double To)>();
        foreach(var span in spans.OrderBy(x=>x.From))
        {
            var from=span.From<10?0:span.From;var to=span.To>total-10?total:span.To;
            if(merged.Count>0&&from<=merged[merged.Count-1].To+10)merged[merged.Count-1]=(merged[merged.Count-1].From,Math.Max(merged[merged.Count-1].To,to));
            else merged.Add((from,to));
        }
        double T(double station){var k=Array.BinarySearch(at,station);if(k>=0)return k/(double)n;k=~k;return (k-1+(station-at[k-1])/Math.Max(1e-9,at[k]-at[k-1]))/n;}
        var stations=new[]{0d}.Concat(merged.SelectMany(x=>new[]{x.From,x.To}).Where(x=>x>0&&x<total)).Append(total).ToArray();
        bool Over(int piece)=>merged.Any(x=>x.From<=stations[piece]+1e-6&&x.To>=stations[piece+1]-1e-6);
        PreparedSegment Mark(PreparedSegment p,int piece)=>Over(piece)?p with{Bridge=true,Layer=layer}:p;
        var rest=s;var consumed=0d;
        for(var piece=0;piece<stations.Length-2;piece++)
        {
            var t=T(stations[piece+1]);
            var (left,right)=NativeDesignUnits.Split(rest,(t-consumed)/(1-consumed),id());
            result.Add(Mark(left with{FinalDeckCurve=null},piece));rest=right with{FinalDeckCurve=null};consumed=t;
        }
        result.Add(Mark(rest,stations.Length-2));
    }

    private static IReadOnlyList<PreparedSegment> PortalSpans(IReadOnlyList<PreparedSegment> source,CancellationToken token)
    {
        var surface=new HashSet<Key>(source.Where(s=>!s.Tunnel).SelectMany(s=>new[]{new Key(s.Kind,s.StartNodeId),new Key(s.Kind,s.EndNodeId)}));
        var occupied=new HashSet<long>(source.SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}));var next=long.MinValue+6000000;
        var result=new List<PreparedSegment>();
        foreach(var s in source)
        {
            token.ThrowIfCancellationRequested();
            if(!s.Tunnel||NetworkVerticalProfiles.ArcLength(s)<40||!surface.Contains(new(s.Kind,s.StartNodeId))||!surface.Contains(new(s.Kind,s.EndNodeId)))
            {result.Add(s);continue;}
            // A portal-to-portal native edge needs an interior composition anchor,
            // not a new node every 80m or a second subdivision by another planner.
            while(occupied.Contains(next))next++;var id=next++;occupied.Add(id);
            var curve=new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);var split=NetworkDesignChecks.Split(curve,.5);
            var middle=(float3)split.Left.P3;
            result.Add(s with{End=middle,EndNodeId=id,Control1=(float3)split.Left.P1-s.StartCurveOffset,Control2=(float3)split.Left.P2,
                EndCurveOffset=default,EndTerminal=false,EndMiniRoundabout=false,EndRoundaboutRadius=0,FinalDeckCurve=null});
            result.Add(s with{Start=middle,StartNodeId=id,Control1=(float3)split.Right.P1,Control2=(float3)split.Right.P2-s.EndCurveOffset,
                StartCurveOffset=default,StartTerminal=false,StartMiniRoundabout=false,StartRoundaboutRadius=0,FinalDeckCurve=null});
        }
        return result;
    }
}
