using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using ImportEarthLocation.Geo;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

public static class WaterPlanner
{
    private sealed record Edge(long From, long To, OsmWay Way, float3 A, float3 B);
    private sealed record Candidate(Edge Edge, float3 Start, string Origin);
    private sealed record RiverArea(GeographicPolygon Polygon, float MinX, float MaxX, float MinZ, float MaxZ)
    {
        public bool Contains(float3 point)=>point.x>=MinX&&point.x<=MaxX&&point.z>=MinZ&&point.z<=MaxZ&&Polygon.Contains(point);
    }

    public static WaterPlan Prepare(OsmDataset? data, IReadOnlyList<GeographicPolygon> polygons, ImportRequest request, TerrainBuilder.Result terrain, ImportLog log, CancellationToken token, float waterCellSize = 7,Action<string>? progress=null)
        => WaterBodyPlanner.Prepare(data,polygons,request,terrain,log,token,waterCellSize,progress);

    // Retained for comparison only. The former test flood bypass is not reachable
    // from the importer; production planning uses a bounded body/stage contract.
    private static WaterPlan PrepareLegacy(OsmDataset? data, IReadOnlyList<GeographicPolygon> polygons, ImportRequest request, TerrainBuilder.Result terrain, ImportLog log, CancellationToken token, float waterCellSize = 7)
    {
        if (data == null) return new WaterPlan(false, 0, 0, 0, "Water geography unavailable; inland sources skipped.");
        var half = (float)(request.MapSize.WidthMeters / 2);
        if(!math.isfinite(waterCellSize)||waterCellSize<=0)
            return new WaterPlan(false,0,0,0,"River sources skipped: active water-grid size unavailable.");
        var repairs=new Dictionary<int,ushort>();
        Dictionary<int,ushort>? pending=null;
        var repairReasons=new Dictionary<string,int>();var repairDetails=0;
        long repairCandidateWay=0,repairActiveWay=0;
        var riverAreas=polygons.Where(p=>p.Outer.Any(r=>r.Length>0)&&(p.Tags.TryGetValue("water",out var water)&&water=="river"||p.Tags.TryGetValue("waterway",out var type)&&type=="riverbank"))
            .Select(p=>{var points=p.Outer.SelectMany(r=>r).ToArray();return new RiverArea(p,points.Min(v=>v.x),points.Max(v=>v.x),points.Min(v=>v.z),points.Max(v=>v.z));}).ToArray();
        int Pixel(float3 p)=>(int)Math.Round(Numeric.Clamp(p.z/request.MapSize.WidthMeters+.5,0,1)*(TerrainBuilder.Resolution-1))*TerrainBuilder.Resolution+(int)Math.Round(Numeric.Clamp(p.x/request.MapSize.WidthMeters+.5,0,1)*(TerrainBuilder.Resolution-1));
        bool Inside(float3 p) => Math.Abs(p.x) <= half && Math.Abs(p.z) <= half;
        float Height(float3 p) {var index=Pixel(p);return (pending!=null&&pending.TryGetValue(index,out var local)?local:repairs.TryGetValue(index,out var saved)?saved:terrain.Heightmap[index])/(float)ushort.MaxValue*terrain.HeightScaleMeters;}
        float OriginalHeight(float3 p)=>terrain.Heightmap[Pixel(p)]/(float)ushort.MaxValue*terrain.HeightScaleMeters;
        bool Repair(float3 center,float target)
        {
            bool Refuse(string reason)
            {
                repairReasons.TryGetValue(reason,out var count);repairReasons[reason]=count+1;
                if(repairDetails++<50)log.Diagnostic($"WATER REPAIR REJECT candidateWay={repairCandidateWay}; activeWay={repairActiveWay}; reason={reason}; position={center}; currentHeight={Height(center):R}; targetHeight={target:R}; requiredCutMeters={Height(center)-target:R}; mappedRiverAreas={riverAreas.Length}; pendingSamples={pending?.Count??0}; acceptedSamples={repairs.Count}; waterCellSize={waterCellSize:R}");
                return false;
            }
            // Correct only small DEM obstructions inside an explicitly mapped riverbed.
            // Never raise terrain, modify lakes, or cut through an island/bank to force an outlet.
            var area=riverAreas.FirstOrDefault(p=>p.Contains(center));if(area==null)return Refuse("sample-outside-mapped-river-area");
            var spacing=(float)request.MapSize.WidthMeters/(TerrainBuilder.Resolution-1);
            var reach=Math.Max(waterCellSize,spacing*2);
            var core=Math.Max(waterCellSize*.5f,spacing*.75f);
            var pixel=Pixel(center);var cx=pixel%TerrainBuilder.Resolution;var cz=pixel/TerrainBuilder.Resolution;
            var cells=(int)Math.Ceiling(reach/spacing);var edits=new Dictionary<int,ushort>();
            for(var z=Math.Max(0,cz-cells);z<=Math.Min(TerrainBuilder.Resolution-1,cz+cells);z++)
            for(var x=Math.Max(0,cx-cells);x<=Math.Min(TerrainBuilder.Resolution-1,cx+cells);x++)
            {
                token.ThrowIfCancellationRequested();
                var p=new float3(x*spacing-half,0,z*spacing-half);
                var distance=math.distance(p.xz,center.xz);
                if(distance>reach||!area.Contains(p))continue;
                var index=z*TerrainBuilder.Resolution+x;var original=terrain.Heightmap[index]/(float)ushort.MaxValue*terrain.HeightScaleMeters;
                if(original<=target)continue;
                // A hydraulic core must be passable; surrounding banks do not have
                // to be flattened to its bed level. Taper cuts toward the existing
                // terrain, preserving polygon holes and never exceeding 3 m.
                if(distance<=core&&original-target>3)
                {
                    log.Diagnostic($"WATER REPAIR BLOCKING SAMPLE candidateWay={repairCandidateWay}; activeWay={repairActiveWay}; position={p}; originalHeight={original:R}; targetHeight={target:R}; requiredCutMeters={original-target:R}; distanceFromChannelMeters={distance:R}; coreRadiusMeters={core:R}; footprintRadiusMeters={reach:R}");
                    return Refuse("channel-core-cut-exceeds-3m");
                }
                var blend=distance<=core?1f:1f-(distance-core)/(reach-core);
                blend=blend*blend*(3-2*blend);
                var cut=Math.Min(3f,(original-target)*blend);
                if(cut<terrain.HeightScaleMeters/ushort.MaxValue)continue;
                edits[index]=(ushort)Math.Ceiling((original-cut)/terrain.HeightScaleMeters*ushort.MaxValue);
            }
            if(!edits.ContainsKey(pixel))return Refuse("center-pixel-not-correctable-inside-river");
            if(pending!.Count+edits.Count>50000)return Refuse("candidate-repair-sample-budget");
            if(repairs.Count+pending.Count+edits.Count>250000)return Refuse("map-repair-sample-budget");
            foreach(var edit in edits)if(!pending!.TryGetValue(edit.Key,out var old)||edit.Value<old)pending[edit.Key]=edit.Value;
            return true;
        }
        bool Tag(OsmWay w, string key, string value) => w.Tags.TryGetValue(key, out var v) && v == value;
        float3 Local(long id) { var n = data.Nodes[id]; return GeoMath.ToLocalMeters(n.Latitude, n.Longitude, request.Latitude, request.Longitude); }
        var coastal = data.Ways.Values.Any(w => Tag(w, "natural", "coastline") && w.NodeIds.Any(id => data.Nodes.ContainsKey(id) && Inside(Local(id))));
        var ways = data.Ways.Values.Where(w => Tag(w, "waterway", "river") || Tag(w, "waterway", "stream")).OrderBy(w => w.Id).ToArray();
        var incoming = new HashSet<long>();
        // A rejected upstream piece must not turn the next segment into an apparent spring.
        foreach (var way in ways) foreach (var id in way.NodeIds.Skip(1)) incoming.Add(id);
        var outgoing = new Dictionary<long, List<Edge>>();
        var edges = new List<Edge>(); var seen = new HashSet<(long, long)>(); var skipped = 0;
        var reasons=new Dictionary<string,int>();var detailedRiverSkips=0;
        void Skip(long id, string reason)
        {
            skipped++;reasons.TryGetValue(reason,out var count);reasons[reason]=count+1;
            if (skipped <= 50) log.Warn($"Waterway {id} source skipped: {reason}.");
            if(data.Ways.TryGetValue(id,out var way)&&Tag(way,"waterway","river")&&detailedRiverSkips++<100)
                log.Diagnostic($"WATER RIVER SKIP way={id}; name={(way.Tags.TryGetValue("name",out var name)?name:"unnamed")}; reason={reason}");
        }
        foreach (var way in ways)
        {
            token.ThrowIfCancellationRequested();
            if (way.NodeIds.Length < 2 || way.NodeIds.Any(id => !data.Nodes.ContainsKey(id)) || Tag(way, "intermittent", "yes") || Tag(way, "tunnel", "yes") || Tag(way, "tunnel", "culvert"))
            { Skip(way.Id, "intermittent, enclosed or incomplete geometry"); continue; }
            for (var i = 1; i < way.NodeIds.Length; i++)
            {
                var from = way.NodeIds[i - 1]; var to = way.NodeIds[i];
                if (from == to || !seen.Add((from, to))) continue;
                var edge = new Edge(from, to, way, Local(from), Local(to));
                if (!math.all(math.isfinite(edge.A)) || !math.all(math.isfinite(edge.B))) continue;
                edges.Add(edge);
                if (!outgoing.TryGetValue(from, out var list)) outgoing[from] = list = new List<Edge>();
                list.Add(edge);
            }
        }
        var candidates = new List<Candidate>();
        foreach (var edge in edges)
        {
            token.ThrowIfCancellationRequested();
            if (!Inside(edge.A) && Clip(edge.A, edge.B, half, out var entry, out _) && entry > 0)
                candidates.Add(new Candidate(edge, math.lerp(edge.A, edge.B, entry), "map-entry"));
            else if (Inside(edge.A) && !incoming.Contains(edge.From))
                candidates.Add(new Candidate(edge, edge.A, "mapped-inland-headwater"));
        }
        log.Info($"Planning river water: {ways.Length:N0} mapped ways, {candidates.Count:N0} upstream/entry candidates; checking connected channels and banks.");
        var sources = new List<RiverSourcePlan>();
        var barrierDetails=0;
        foreach (var candidate in candidates.OrderBy(c => Tag(c.Edge.Way, "waterway", "river") ? 0 : 1).ThenBy(c => c.Edge.Way.Id).ThenBy(c => c.Edge.From))
        {
            token.ThrowIfCancellationRequested();
            if(Tag(candidate.Edge.Way,"waterway","river")&&sources.Any(s=>s.WayId==candidate.Edge.Way.Id||math.distance(s.Position.xz,candidate.Start.xz)<2000))continue;
            if (sources.Count >= 64) { Skip(candidate.Edge.Way.Id, "bounded source budget reached (64)"); continue; }
            pending=new Dictionary<int,ushort>();
            var edge = candidate.Edge; var point = candidate.Start; var path = new List<float3> { point };
            var visited = new HashSet<(long, long)>(); var outlet = false; var reason = "no verified downstream outlet";
            var lowest = Height(point); var distance = 0f;
            while (visited.Add((edge.From, edge.To)) && visited.Count <= 20000)
            {
                token.ThrowIfCancellationRequested();
                if (!Clip(point, edge.B, half, out _, out var exit)) break;
                var end = math.lerp(point, edge.B, exit); var length = math.distance(point, end);
                // Repair footprints must overlap at the active simulation scale;
                // fixed 32 m spacing could leave untouched barriers on finer grids.
                var steps = Math.Max(1, (int)Math.Ceiling(length / Math.Min(32,waterCellSize*.5f)));
                var blocked = false;
                for (var i = 1; i <= steps; i++)
                {
                    token.ThrowIfCancellationRequested();
                    var sample = math.lerp(point, end, (float)i / steps); var h = Height(sample);
                    if(Tag(candidate.Edge.Way,"waterway","river")&&h>lowest+.15f&&h<=lowest+3)
                    {repairCandidateWay=candidate.Edge.Way.Id;repairActiveWay=edge.Way.Id;if(Repair(sample,lowest))h=Height(sample);}
                    if (!TerrainConditioner.Finite(h) || h > lowest + 1.5f)
                    {
                        blocked = true; reason = "downstream terrain barrier/rising channel exceeds 1.5 m";
                        if(Tag(candidate.Edge.Way,"waterway","river")&&barrierDetails++<50)
                            log.Diagnostic($"WATER CHANNEL BARRIER candidateWay={candidate.Edge.Way.Id}; activeWay={edge.Way.Id}; origin={candidate.Origin}; position={sample}; observedHeight={h:R}; upstreamLow={lowest:R}; riseMeters={h-lowest:R}; distanceFromStartMeters={distance:0.##}; allowableUncorrectedRiseMeters=1.5; maxMappedRiverCutMeters=3; correctionRequiresMappedRiverAndBoundedFootprint=true; sourceNotCreated=true");
                        break;
                    }
                    lowest = Math.Min(lowest, h); distance += length / steps; path.Add(sample);
                    if (h < WaterLevels.Effective(terrain.SeaLevelMeters) - .25f) { outlet = true; break; }
                }
                if (blocked || outlet) break;
                if (exit < 1 || !Inside(edge.B)) { outlet = true; break; }
                if (!outgoing.TryGetValue(edge.To, out var next) || next.Count != 1)
                { reason = next?.Count > 1 ? "ambiguous downstream fork" : "channel ends inland without a verified outlet"; break; }
                point = edge.B; edge = next[0];
            }
            if (!outlet || distance < 100) { Skip(candidate.Edge.Way.Id, outlet ? "channel too short" : reason); continue; }
            // Inset along the actual channel; never place directly on the map boundary.
            var sourceIndex = 0; var walked = 0f;
            while (sourceIndex + 1 < path.Count && walked < 96) { walked += math.distance(path[sourceIndex], path[sourceIndex + 1]); sourceIndex++; }
            if (sourceIndex + 1 >= path.Count) { Skip(candidate.Edge.Way.Id, "no channel remains after source inset"); continue; }
            var position = path[sourceIndex]; position.y = Height(position);
            if (position.y <= WaterLevels.Effective(terrain.SeaLevelMeters)) { Skip(candidate.Edge.Way.Id, "source already at/below sea level"); continue; }
            var tangent = math.normalizesafe(path[sourceIndex + 1] - path[sourceIndex]); var normal = new float3(-tangent.z, 0, tangent.x);
            float Bank(int side, out float bankDistance)
            {
                bankDistance = 0;
                for (var d = Math.Max(16f,waterCellSize); d <= 2048; d *= 2)
                {
                    var p = position + normal * (side * d); if (!Inside(p)) break;
                    var h = Height(p);
                    if (TerrainConditioner.Finite(h) && h > position.y + .5f) { bankDistance = d; return h; }
                }
                return float.NaN;
            }
            var left = Bank(-1, out var leftDistance); var right = Bank(1, out var rightDistance);
            var bank = Math.Min(left, right); var depth = Math.Min(1f, bank - position.y - .35f);
            var radius = Math.Min(64f, Math.Min(leftDistance, rightDistance) / 4);
            if (!TerrainConditioner.Finite(depth) || depth < .2f || radius < waterCellSize || Math.Max(Math.Abs(position.x), Math.Abs(position.z)) + radius + waterCellSize > half)
            { Skip(candidate.Edge.Way.Id, "banks/source footprint cannot be safely resolved at water-grid scale"); continue; }
            if (sources.Any(s => math.distance(s.Position.xz, position.xz) < Math.Max(100, s.Radius + radius)))
            { Skip(candidate.Edge.Way.Id, "nearby upstream source already supplies this area"); continue; }
            var probe = path[Math.Min(path.Count - 1, sourceIndex + 16)]; probe.y = Height(probe);
            var probes=new List<float3>();
            for(var i=1;i<=4;i++){var p=path[sourceIndex+(path.Count-1-sourceIndex)*i/5];p.y=Height(p);probes.Add(p);}
            candidate.Edge.Way.Tags.TryGetValue("name", out var name);
            sources.Add(new RiverSourcePlan(candidate.Edge.Way.Id, name ?? "unnamed waterway", candidate.Origin, position, radius, depth, bank, probe){ChannelProbes=probes});
            foreach(var edit in pending)if(!repairs.TryGetValue(edit.Key,out var old)||edit.Value<old)repairs[edit.Key]=edit.Value;
            log.Diagnostic($"WATER CHANNEL REPAIR way={candidate.Edge.Way.Id}; changedSamples={pending.Count}; maxCutMeters=3; mappedRiverOnly=true; taperedFootprint=true; coreDiameterMeters={waterCellSize:R}; waterCellSize={waterCellSize:R}; downstreamProbes={probes.Count}; nativeFlowNotYetVerified=true");
            log.Diagnostic($"WATER PLAN way={candidate.Edge.Way.Id}; origin={candidate.Origin}; position={position}; radius={radius:0.##}; depth={depth:0.##}; bankLevel={bank:0.##}; checkedChannelMeters={distance:0}; downstreamProbe={probe}; supplementalSources=0; springPhysicallyVerified=false");
        }
        pending=null; // Never rank fallback sites using an uncommitted repair batch.
        // A DEM obstruction or an incomplete downstream graph is not permission to
        // suppress all water. For mapped river polygons, attempt bounded target-depth
        // sources directly in the river, including elevated reaches and inland heads.
        // Polygon holes still exclude islands; no lake or dry-land carving is implied.
        var mappedSites=new List<(Edge edge,float3 point,RiverArea area,int priority)>();
        foreach(var e in edges.Where(e=>Tag(e.Way,"waterway","river")))
        {
            token.ThrowIfCancellationRequested();
            if(!Clip(e.A,e.B,half,out var first,out var last))continue;
            var a=math.lerp(e.A,e.B,first);var b=math.lerp(e.A,e.B,last);var length=math.distance(a.xz,b.xz);
            var count=Math.Max(1,(int)Math.Ceiling(length/128));
            for(var i=0;i<=count;i++)
            {
                var p=math.lerp(a,b,(i+.5f)/(count+1));
                if(Math.Max(Math.Abs(p.x),Math.Abs(p.z))+waterCellSize*2>half)continue;
                var area=riverAreas.FirstOrDefault(r=>r.Contains(p));if(area==null)continue;
                p.y=Height(p);if(!math.isfinite(p.y))continue;
                mappedSites.Add((e,p,area,!Inside(e.A)?3:!incoming.Contains(e.From)?2:1));
            }
        }
        // Boundary-driven sources need a connected hydraulic bed, not just a
        // relaxed source veto. Lower only the mapped river core, taper to banks,
        // preserve holes/islands, and never mutate the original DEM.
        var channelVisited=new HashSet<Edge>();var channelCuts=0;var limitedCuts=0;
        var bedProfiles=new List<RiverBedSurface.Profile>();
        var spacing=(float)request.MapSize.WidthMeters/(TerrainBuilder.Resolution-1);
        foreach(var seed in edges.Where(e=>Tag(e.Way,"waterway","river")).OrderBy(e=>incoming.Contains(e.From)?1:0).ThenByDescending(e=>Height(e.A)).ThenBy(e=>e.Way.Id))
        {
            var edge=seed;var floor=OriginalHeight(edge.A)-3f;
            while(channelVisited.Add(edge))
            {
                token.ThrowIfCancellationRequested();
                if(Clip(edge.A,edge.B,half,out var lo,out var hi))
                {
                    var a=math.lerp(edge.A,edge.B,lo);var b=math.lerp(edge.A,edge.B,hi);
                    var count=Math.Max(1,(int)Math.Ceiling(math.distance(a.xz,b.xz)/Math.Max(spacing,waterCellSize*.5f)));
                    for(var i=0;i<=count;i++)
                    {
                        var center=math.lerp(a,b,i/(float)count);var area=riverAreas.FirstOrDefault(r=>r.Contains(center));
                        if(area==null){floor=OriginalHeight(center)-3f;continue;}
                        var previousFloor=floor;
                        floor=Math.Min(floor,OriginalHeight(center)-3f);
                        if(i>0)bedProfiles.Add(new RiverBedSurface.Profile(math.lerp(a,b,(i-1)/(float)count),center,previousFloor,floor));
                        var pixel=Pixel(center);var cx=pixel%TerrainBuilder.Resolution;var cz=pixel/TerrainBuilder.Resolution;
                        var reach=Math.Max(waterCellSize,spacing*1.5f);var cells=(int)Math.Ceiling(reach/spacing);
                        for(var z=Math.Max(0,cz-cells);z<=Math.Min(TerrainBuilder.Resolution-1,cz+cells);z++)
                        for(var x=Math.Max(0,cx-cells);x<=Math.Min(TerrainBuilder.Resolution-1,cx+cells);x++)
                        {
                            var point=new float3(x*spacing-half,0,z*spacing-half);var distance=math.distance(point.xz,center.xz);
                            if(distance>reach||!area.Contains(point))continue;
                            var index=z*TerrainBuilder.Resolution+x;var original=terrain.Heightmap[index]/(float)ushort.MaxValue*terrain.HeightScaleMeters;
                            var blend=distance<=reach*.5f?1f:1-(distance-reach*.5f)/(reach*.5f);blend=blend*blend*(3-2*blend);
                            var requested=Math.Max(0,(original-floor)*blend);if(requested>12)limitedCuts++;
                            var target=(ushort)Math.Ceiling(Math.Max(0,original-Math.Min(12,requested))/terrain.HeightScaleMeters*ushort.MaxValue);
                            if(target>=terrain.Heightmap[index]||repairs.TryGetValue(index,out var old)&&old<=target)continue;
                            if(repairs.Count>=1500000&&!repairs.ContainsKey(index))continue;
                            repairs[index]=target;channelCuts++;
                        }
                    }
                }
                if(!outgoing.TryGetValue(edge.To,out var next))break;
                var channelNext=next.Where(e=>Tag(e.Way,"waterway","river")&&!channelVisited.Contains(e)).OrderBy(e=>Height(e.B)).ThenBy(e=>e.Way.Id).ToArray();
                if(channelNext.Length==0)break;edge=channelNext[0];
            }
        }
        RiverBedSurface.Shape(terrain.Heightmap,terrain.HeightScaleMeters,request.MapSize.WidthMeters,riverAreas.Select(r=>r.Polygon).ToArray(),bedProfiles,repairs,waterCellSize,log,token);
        log.Count("Boundary river channel edits",channelCuts);log.Count("Boundary river channel cut limits",limitedCuts);
        log.Diagnostic($"WATER CHANNEL boundariesDriveBed=true; visitedEdges={channelVisited.Count}; editedSamples={repairs.Count}; maximumCutMeters=12; limitedSamples={limitedCuts}; holesAndBanksProtected=true; originalDemUnchanged=true");
        for(var i=0;i<sources.Count;i++)
        {
            var source=sources[i];if(!riverAreas.Any(r=>r.Contains(source.Position)))continue;
            var p=source.Position;p.y=Height(p);
            var depth=source.Radius>=waterCellSize*2?7f:5f;
            sources[i]=source with{Position=p,Depth=depth,TargetSurfaceLevel=p.y+depth,BoundaryDriven=true,
                ChannelProbes=source.ChannelProbes.Select(probe=>new float3(probe.x,Height(probe),probe.z)).ToArray()};
        }
        foreach(var site in mappedSites.OrderByDescending(s=>s.priority).ThenBy(s=>s.edge.Way.Id).ThenByDescending(s=>s.point.y))
        {
            token.ThrowIfCancellationRequested();if(sources.Count>=1536)break;
            var position=site.point;position.y=Height(position);
            if(position.y<=WaterLevels.Effective(terrain.SeaLevelMeters))continue;
            if(sources.Any(s=>math.distance(s.Position.xz,position.xz)<100))continue;
            var radius=waterCellSize;
            // Prefer a circle within mapped banks. A narrow polygon still gets a
            // grid-sized attempt: the native grid cannot represent a smaller source.
            bool Fits(float r)=>Enumerable.Range(0,16).All(i=>site.area.Contains(site.point+new float3(math.cos(i*math.PI/8),0,math.sin(i*math.PI/8))*r));
            if(Fits(waterCellSize*2))radius=Math.Max(waterCellSize,Math.Min(168,waterCellSize*3));
            while(radius>waterCellSize&&!Fits(radius))radius-=waterCellSize;
            radius=Math.Min(radius,half-Math.Max(Math.Abs(position.x),Math.Abs(position.z))-waterCellSize);
            var downstream=site.edge.B;downstream.y=Height(downstream);
            site.edge.Way.Tags.TryGetValue("name",out var name);
            var probes=new List<float3>();var probeEdge=site.edge;var probeVisited=new HashSet<Edge>();
            while(probeVisited.Add(probeEdge)&&probes.Count<32)
            {
                var steps=Math.Max(1,(int)Math.Ceiling(math.distance(probeEdge.A.xz,probeEdge.B.xz)/256));
                for(var j=1;j<=steps&&probes.Count<32;j++){var p=math.lerp(probeEdge.A,probeEdge.B,j/(float)steps);if(Inside(p)){p.y=Height(p);probes.Add(p);}}
                if(!outgoing.TryGetValue(probeEdge.To,out var next))break;
                var follow=next.Where(e=>Tag(e.Way,"waterway","river")&&!probeVisited.Contains(e)).OrderBy(e=>Height(e.B)).FirstOrDefault();if(follow==null)break;probeEdge=follow;
            }
            var depth=radius>=waterCellSize*2?7f:5f;
            sources.Add(new RiverSourcePlan(site.edge.Way.Id,name??"unnamed river","osm-river-boundary",position,radius,depth,position.y+depth+1,downstream){BoundaryDriven=true,TargetSurfaceLevel=position.y+depth,ChannelProbes=probes});
            log.Diagnostic($"WATER BOUNDARY PLAN way={site.edge.Way.Id}; name={name}; position={position}; radius={radius}; targetDepth={depth}; downstreamProbes={probes.Count}; polygonFootprintFits={Fits(radius)}; priority={site.priority}; floodingAcceptedForTesting=true");
        }
        var sea=WaterLevels.Effective(terrain.SeaLevelMeters);var regulators=new List<RiverSourcePlan>();
        foreach(var site in mappedSites.OrderBy(s=>Height(s.point)).ThenBy(s=>s.edge.Way.Id))
        {
            token.ThrowIfCancellationRequested();if(regulators.Count>=12)break;
            var p=site.point;p.y=Height(p);if(p.y>=sea-.1f)continue;
            if(regulators.Any(s=>math.distance(s.Position.xz,p.xz)<3000))continue;
            var radius=Math.Max(waterCellSize,Math.Min(168,waterCellSize*3));
            while(radius>waterCellSize&&!Enumerable.Range(0,16).All(i=>site.area.Contains(p+new float3(math.cos(i*math.PI/8),0,math.sin(i*math.PI/8))*radius)))radius-=waterCellSize;
            radius=Math.Min(radius,half-Math.Max(Math.Abs(p.x),Math.Abs(p.z))-waterCellSize);
            regulators.Add(new RiverSourcePlan(site.edge.Way.Id,"Downstream river level","sea-level-regulator",p,radius,sea-p.y,sea+1,p){BoundaryDriven=true,TargetSurfaceLevel=sea,ChannelProbes=new[]{p}});
        }
        sources.AddRange(regulators);log.Count("Downstream sea-level regulators planned",regulators.Count);
        log.Count("Boundary-driven river sources planned",sources.Count(s=>s.BoundaryDriven));
        var lakes = polygons.Count(p => p.Tags.TryGetValue("natural", out var n) && n == "water" && (!p.Tags.TryGetValue("water", out var t) || t == "lake" || t == "reservoir") || p.Tags.TryGetValue("landuse", out var l) && l == "reservoir");
        ushort[]? conditioned=null;
        if(repairs.Count>0){conditioned=(ushort[])terrain.Heightmap.Clone();foreach(var edit in repairs)conditioned[edit.Key]=edit.Value;}
        log.Count("Riverbed samples corrected",repairs.Count);
        log.Diagnostic("WATER REPAIR SUMMARY rejectedAttempts="+repairReasons.Values.Sum()+"; reasons="+string.Join(" | ",repairReasons.OrderBy(p=>p.Key).Select(p=>$"{p.Key}:{p.Value}"))+"; detailedMessagesLimitedTo=50; sourceCreationNotProven=true");
        var report = $"Water: {sources.Count} river/stream sources planned, including {sources.Count(s=>s.BoundaryDriven)} boundary-driven test sources; {skipped} strict candidates skipped; {repairs.Count} riverbed samples corrected; {lakes} lake/reservoir features (unchanged). Local flooding is allowed for boundary-driven sources; downstream filling requires simulation verification.";
        log.Info(report);
        log.Diagnostic($"WATER PLAN SUMMARY mappedWays={ways.Length}; connectedEdges={edges.Count}; mapEntryCandidates={candidates.Count(c=>c.Origin=="map-entry")}; inlandHeadCandidates={candidates.Count(c=>c.Origin=="mapped-inland-headwater")}; sourcesPlanned={sources.Count}; seaPlaneLevel={WaterLevels.Effective(terrain.SeaLevelMeters):R}; skipReasons="+string.Join(" | ",reasons.OrderBy(p=>p.Key).Select(p=>$"{p.Key}:{p.Value}")));
        return new WaterPlan(coastal, ways.Length, lakes, skipped, report) { Sources = sources,WaterCellSize=waterCellSize,ConditionedHeightmap=conditioned };
    }

    // Clipping also finds crossings with both endpoints outside the map.
    internal static bool Clip(float3 a, float3 b, float half, out float entry, out float exit)
    {
        var lo = 0f; var hi = 1f;
        bool Axis(float start, float delta)
        {
            if (Math.Abs(delta) < .0001f) return Math.Abs(start) <= half;
            var t0 = (-half - start) / delta; var t1 = (half - start) / delta;
            lo = Math.Max(lo, Math.Min(t0, t1)); hi = Math.Min(hi, Math.Max(t0, t1)); return lo <= hi;
        }
        var hit = Axis(a.x, b.x - a.x) && Axis(a.z, b.z - a.z);
        entry = lo; exit = hi; return hit && hi > lo;
    }
}
