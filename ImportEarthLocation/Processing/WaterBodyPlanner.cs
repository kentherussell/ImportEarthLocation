using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using ImportEarthLocation.Geo;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Body levels, beds and banks have separate ownership. A source targets an
// absolute surface level; its depth is recomputed against the final native bed.
public static class WaterBodyPlanner
{
    // Same even/odd predicate as GeographyPolygons.Inside, but queries inspect
    // only edges crossing the point's horizontal band, not an entire river ring
    // for every DEM pixel. Long edges are stored once to bound index memory.
    private sealed class IndexedRing
    {
        private const float Band=128;
        private readonly Dictionary<int,List<(float3 A,float3 B)>> _bands=new();
        private readonly List<(float3 A,float3 B)> _long=new();
        private readonly CancellationToken _token;
        public IndexedRing(float3[] ring,CancellationToken token,Action? pulse=null)
        {
            _token=token;
            for(var i=0;i<ring.Length;i++)
            {
                token.ThrowIfCancellationRequested();if((i&1023)==0)pulse?.Invoke();var a=ring[i];var b=ring[(i+1)%ring.Length];
                if(a.z==b.z)continue;
                var lo=(int)Math.Floor(Math.Min(a.z,b.z)/Band);var hi=(int)Math.Floor(Math.Max(a.z,b.z)/Band);
                if((long)hi-lo>512){_long.Add((a,b));continue;}
                for(var z=lo;z<=hi;z++)
                {if(!_bands.TryGetValue(z,out var edges))_bands[z]=edges=new();edges.Add((a,b));}
            }
        }
        public bool Inside(float3 p)
        {
            var inside=false;var checks=0;
            void Scan(List<(float3 A,float3 B)> edges)
            {
                foreach(var edge in edges)
                {
                    if((checks++&1023)==0)_token.ThrowIfCancellationRequested();var a=edge.A;var b=edge.B;
                    if((a.z>p.z)!=(b.z>p.z)&&p.x<(b.x-a.x)*(p.z-a.z)/(b.z-a.z)+a.x)inside=!inside;
                }
            }
            if(_bands.TryGetValue((int)Math.Floor(p.z/Band),out var nearby))Scan(nearby);
            Scan(_long);return inside;
        }
    }
    private sealed class Area
    {
        public readonly GeographicPolygon Polygon;
        public readonly float2 Min,Max;
        public readonly int Order;
        private readonly IndexedRing[] _outer,_inner;
        public bool HasHoles=>_inner.Length>0;
        public Area(GeographicPolygon polygon,int order,CancellationToken token,Action? pulse=null)
        {
            Polygon=polygon;Order=order;Min=new float2(float.PositiveInfinity);Max=new float2(float.NegativeInfinity);
            foreach(var ring in polygon.Outer)foreach(var p in ring){token.ThrowIfCancellationRequested();Min=math.min(Min,p.xz);Max=math.max(Max,p.xz);}
            _outer=polygon.Outer.Select(r=>new IndexedRing(r,token,pulse)).ToArray();_inner=polygon.Inner.Select(r=>new IndexedRing(r,token,pulse)).ToArray();
        }
        public bool InBounds(float3 p)=>math.all(p.xz>=Min)&&math.all(p.xz<=Max);
        public bool InHole(float3 p)=>InBounds(p)&&_inner.Any(r=>r.Inside(p));
        public bool Contains(float3 p)=>InBounds(p)&&_outer.Any(r=>r.Inside(p))&&!_inner.Any(r=>r.Inside(p));
    }
    private sealed class AreaIndex
    {
        private const float Size=2048;
        private readonly Dictionary<(int,int),List<Area>> _cells=new();
        private readonly List<Area> _large=new();
        public AreaIndex(IEnumerable<Area> areas,CancellationToken token=default)
        {
            foreach(var area in areas)
            {
                token.ThrowIfCancellationRequested();var lo=(int2)math.floor(area.Min/Size);var hi=(int2)math.floor(area.Max/Size);
                if((hi.x-lo.x+1L)*(hi.y-lo.y+1L)>4096){_large.Add(area);continue;}
                for(var x=lo.x;x<=hi.x;x++)for(var z=lo.y;z<=hi.y;z++)
                {token.ThrowIfCancellationRequested();if(!_cells.TryGetValue((x,z),out var list))_cells[(x,z)]=list=new();list.Add(area);}
            }
        }
        public IEnumerable<Area> Nearby(float3 p)
        {
            _cells.TryGetValue(((int)Math.Floor(p.x/Size),(int)Math.Floor(p.z/Size)),out var local);
            var i=0;var j=0;var count=local?.Count??0;
            // Preserve original polygon precedence at overlaps; never use hash
            // iteration order to choose a different body or island mask.
            while(i<count||j<_large.Count)
                if(j==_large.Count||i<count&&local![i].Order<_large[j].Order)yield return local![i++];else yield return _large[j++];
        }
    }
    public static Func<float3,float?> SurfaceSampler(WaterPlan water,IReadOnlyList<GeographicPolygon> polygons)
    {
        var masks=polygons.Where(p=>p.Tags.TryGetValue("natural",out var n)&&n=="water"||p.Tags.TryGetValue("waterway",out var w)&&w=="riverbank"||p.Tags.TryGetValue("landuse",out var l)&&l=="reservoir").Where(p=>p.Outer.Any(r=>r.Length>0))
            .Select((p,i)=>new Area(p,i,default)).ToArray();
        var maskIndex=new AreaIndex(masks);
        var flat=water.AreaStages.GroupBy(s=>s.BodyId).ToDictionary(g=>g.Key,g=>g.Min(s=>s.Level));
        var reaches=new Dictionary<(int,int),List<WaterReachStage>>();
        foreach(var r in water.ReachStages)
        {
            var margin=Math.Max(r.HalfWidth,r.PolygonId.HasValue?1024:1);var lo=math.min(r.Start.xz,r.End.xz)-margin;var hi=math.max(r.Start.xz,r.End.xz)+margin;
            for(var x=(int)Math.Floor(lo.x/2048);x<=(int)Math.Floor(hi.x/2048);x++)
                for(var z=(int)Math.Floor(lo.y/2048);z<=(int)Math.Floor(hi.y/2048);z++)
                {if(!reaches.TryGetValue((x,z),out var list))reaches[(x,z)]=list=new();list.Add(r);}
        }
        return p=>
        {
            var area=maskIndex.Nearby(p).FirstOrDefault(a=>a.Contains(p));
            if(area!=null&&flat.TryGetValue(area.Polygon.Identity,out var level))return level;
            float? result=null;var best=float.PositiveInfinity;
            if(reaches.TryGetValue(((int)Math.Floor(p.x/2048),(int)Math.Floor(p.z/2048)),out var list))foreach(var r in list)
            {
                if(r.PolygonId.HasValue ? area?.Polygon.Identity!=r.PolygonId.Value : maskIndex.Nearby(p).Any(a=>a.InHole(p)))continue;
                var delta=r.End.xz-r.Start.xz;var t=math.clamp(math.dot(p.xz-r.Start.xz,delta)/Math.Max(.0001f,math.lengthsq(delta)),0,1);
                var distance=math.distance(p.xz,math.lerp(r.Start.xz,r.End.xz,t));
                if(!r.PolygonId.HasValue&&distance>r.HalfWidth||distance>=best)continue;
                best=distance;result=math.lerp(r.StartLevel,r.EndLevel,t);
            }
            return result;
        };
    }
    public static WaterPlan Prepare(OsmDataset? data,IReadOnlyList<GeographicPolygon> polygons,ImportRequest request,
        TerrainBuilder.Result terrain,ImportLog log,CancellationToken token,float cell,Action<string>? progress=null)
    {
        if(data==null||!math.isfinite(cell)||cell<=0)return new WaterPlan(false,0,0,0,"Water evidence or native grid unavailable; water preparation cannot be verified.");
        var width=(float)request.MapSize.WidthMeters;var half=width*.5f;var n=(int)Math.Sqrt(terrain.Heightmap.Length);
        var spacing=width/(n-1);var conditioned=(ushort[])terrain.Heightmap.Clone();var editMask=new byte[conditioned.Length];var editCount=0;var sources=new List<RiverSourcePlan>();
        var areaStages=new List<WaterAreaStage>();var reachStages=new List<WaterReachStage>();
        var riverSurface=new float[conditioned.Length];
        var unresolved=0;var riverCount=0;var lakeCount=0;
        var clock=Stopwatch.StartNew();var lastPulse=-5d;var phase="indexing mapped water";long activeBody=0;long bedVisits=0;long reachSamples=0;var totalWays=0;
        void Pulse(bool force=false)
        {
            token.ThrowIfCancellationRequested();if(!force&&clock.Elapsed.TotalSeconds-lastPulse<5)return;
            lastPulse=clock.Elapsed.TotalSeconds;
            progress?.Invoke($"Planning water: {phase}; {riverCount}/{totalWays} waterways; {sources.Count} controls");
            log.Diagnostic($"WATER PLAN PROGRESS phase={phase}; body={activeBody}; lakes={lakeCount}; waterways={riverCount}/{totalWays}; reachSamples={reachSamples}; bedCellsVisited={bedVisits}; bedEdits={editCount}; controls={sources.Count}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.###}; cancellationChecked=true");
        }
        Pulse(true);
        bool Tag(IReadOnlyDictionary<string,string> tags,string k,string v)=>tags.TryGetValue(k,out var value)&&value==v;
        bool Inside(float3 p)=>Math.Abs(p.x)<=half-cell*2&&Math.Abs(p.z)<=half-cell*2;
        int Pixel(float3 p)=>(int)Math.Round(math.clamp(p.z/width+.5f,0,1)*(n-1))*n+(int)Math.Round(math.clamp(p.x/width+.5f,0,1)*(n-1));
        float Height(float3 p)=>NetworkProcessor.Sample(p.x,p.z,conditioned,terrain.HeightScaleMeters,width);
        float BankHeight(float3 p)=>NetworkProcessor.Sample(p.x,p.z,terrain.Heightmap,terrain.HeightScaleMeters,width);
        float3 Local(long id){var p=data.Nodes[id];return GeoMath.ToLocalMeters(p.Latitude,p.Longitude,request.Latitude,request.Longitude);}
        var areas=polygons.Where(p=>Tag(p.Tags,"natural","water")||Tag(p.Tags,"waterway","riverbank")||Tag(p.Tags,"landuse","reservoir"))
            .Where(p=>p.Outer.Any(r=>r.Length>0)).Select((p,i)=>{Pulse();return new Area(p,i,token,()=>Pulse());}).ToArray();
        var areaIndex=new AreaIndex(areas,token);var holeIndex=new AreaIndex(areas.Where(a=>a.HasHoles),token);
        bool Coastal(Area a)=>Tag(a.Polygon.Tags,"water","sea")||Tag(a.Polygon.Tags,"water","ocean")||Tag(a.Polygon.Tags,"water","bay")||Tag(a.Polygon.Tags,"natural","bay")||Tag(a.Polygon.Tags,"tidal","yes");
        bool Flowing(Area a)=>Tag(a.Polygon.Tags,"water","river")||Tag(a.Polygon.Tags,"water","canal")||Tag(a.Polygon.Tags,"waterway","riverbank");
        bool Dry(Area a)=>Tag(a.Polygon.Tags,"intermittent","yes")||a.Polygon.Tags.TryGetValue("seasonal",out var season)&&season!="no"&&season!="false"&&season!="0";
        bool Lake(Area a)=>!Flowing(a)&&!Coastal(a)&&!Dry(a);
        bool MappedDry(float3 p)=>holeIndex.Nearby(p).Any(a=>a.InHole(p));
        bool DryFootprint(float3 q,Func<float3,bool> wet)
        {
            if(!Inside(q)||wet(q))return false;
            var margin=Math.Max(cell,spacing)*1.5f;
            return !wet(q+new float3(margin,0,margin))&&!wet(q+new float3(-margin,0,margin))&&
                !wet(q+new float3(margin,0,-margin))&&!wet(q+new float3(-margin,0,-margin));
        }
        float3? DryProbe(float3 p,float3 direction,Func<float3,bool> wet,float start)
        {
            for(var distance=Math.Max(start,Math.Max(cell,spacing)*2);distance<=1024;distance+=Math.Max(cell,spacing))
            {var q=p+direction*distance;if(DryFootprint(q,wet))return q;}
            return null;
        }
        void Cut(float3 center,float radius,float target,Func<float3,bool> mask,float budget)
        {
            var pixel=Pixel(center);var x0=pixel%n;var z0=pixel/n;var reach=(int)Math.Ceiling(radius/spacing);
            for(var z=Math.Max(0,z0-reach);z<=Math.Min(n-1,z0+reach);z++)for(var x=Math.Max(0,x0-reach);x<=Math.Min(n-1,x0+reach);x++)
            {
                if((bedVisits++&4095)==0)Pulse();var p=new float3(x*spacing-half,0,z*spacing-half);var distance=math.distance(p.xz,center.xz);
                if(distance>radius)continue;var index=z*n+x;var original=terrain.Heightmap[index]/65535f*terrain.HeightScaleMeters;
                var blend=math.clamp((radius-distance)/(radius*.3f),0,1);blend=blend*blend*(3-2*blend);
                var height=Math.Max(0,original-Math.Min(budget,Math.Max(0,original-target))*blend);
                var encoded=(ushort)Math.Ceiling(math.clamp(height/terrain.HeightScaleMeters*65535,0,65535));
                // Overlapping cuts frequently propose the same/larger height.
                // Preserve the exact min-cut result while avoiding another
                // polygon query for a raster cell that cannot change.
                if(encoded>=conditioned[index])continue;
                if(mask(p)){conditioned[index]=encoded;if(editMask[index]==0){editMask[index]=1;editCount++;}}
            }
        }
        var boundaryOffsets=new[]{new float3(spacing,0,0),new float3(-spacing,0,0),new float3(0,0,spacing),new float3(0,0,-spacing)};
        void CarveArea(Area area,Func<float3,float?> level)
        {
            var accepted=0;var blockedByBudget=0;
            var loX=Math.Max(0,(int)Math.Ceiling((area.Min.x+half)/spacing));var hiX=Math.Min(n-1,(int)Math.Floor((area.Max.x+half)/spacing));
            var loZ=Math.Max(0,(int)Math.Ceiling((area.Min.y+half)/spacing));var hiZ=Math.Min(n-1,(int)Math.Floor((area.Max.y+half)/spacing));
            for(var z=loZ;z<=hiZ;z++)for(var x=loX;x<=hiX;x++)
            {
                if((bedVisits++&4095)==0)Pulse();var p=new float3(x*spacing-half,0,z*spacing-half);
                if(!area.Contains(p)||MappedDry(p))continue;var stage=level(p);if(!stage.HasValue||!math.isfinite(stage.Value))continue;
                var interior=0;foreach(var direction in boundaryOffsets)if(area.Contains(p+direction))interior++;
                var depth=interior==4?2:Math.Max(.25f,interior*.25f);
                var index=z*n+x;var original=terrain.Heightmap[index]/65535f*terrain.HeightScaleMeters;
                var height=Math.Max(0,Lake(area)?Math.Max(original-8,Math.Min(original,stage.Value-depth)):Math.Min(original,stage.Value-depth));
                var encoded=(ushort)Math.Ceiling(math.clamp(height/terrain.HeightScaleMeters*65535,0,65535));
                accepted++;if(height>=stage.Value-.1f)blockedByBudget++;
                // The accepted wet interior owns its bed even if it was already
                // below target. Later earthworks must not introduce a new dam.
                if(!Lake(area)&&editMask[index]==0){editMask[index]=1;editCount++;}
                if(encoded>=conditioned[index])continue;conditioned[index]=encoded;if(editMask[index]==0){editMask[index]=1;editCount++;}
            }
            log.Diagnostic($"WATER AREA BED body={area.Polygon.Identity}; cellsVisitedInside={accepted}; bedAboveStageAfterBudget={blockedByBudget}; continuousMask=true; holesPreserved=true; lakeBudgetMeters=8; mappedRiverCutsConfinedToWetMask=true");
            log.Count("Water area bed cells above stage",blockedByBudget);
        }
        bool Add(long way,long body,string kind,string name,float3 p,float radius,float stage,float bank,IReadOnlyList<float3> banks,IReadOnlyList<float3> probes,Func<float3,bool> mask)
        {
            if(!Inside(p)||radius<Math.Max(2,cell*.25f)||!math.isfinite(stage))return false;
            // No circle spilling onto banks or islands, even for boundary-driven rivers.
            bool Fits(float r)=>Enumerable.Range(1,3).All(ring=>Enumerable.Range(0,16).All(i=>mask(p+new float3(math.cos(i*math.PI/8),0,math.sin(i*math.PI/8))*(r*ring/3))));
            while(radius>Math.Max(2,cell*.25f)&&!Fits(radius))radius=Math.Max(Math.Max(2,cell*.25f),radius-cell*.25f);
            if(!Fits(radius)||stage>bank-.35f){unresolved++;log.Count(stage>bank-.35f?"Water stage bank conflicts":"Water source footprint policy rejected");return false;}
            // Give every footprint ring a fully submerged core, including the
            // neighbouring terrain samples used by native bilinear interpolation.
            Cut(p,(radius+spacing*2)/.7f,stage-2.5f,mask,kind=="lake"?8:float.PositiveInfinity);
            p.y=Height(p);var depth=stage-p.y;
            if(depth<.4f){unresolved++;log.Diagnostic($"WATER BODY UNRESOLVED body={body}; reason=bed-above-bank-limited-stage; bed={p.y:R}; stage={stage:R}; noFloodBypass=true");return false;}
            var repairCells=new List<int>();var pixel=Pixel(p);var reach=(int)Math.Ceiling((radius+spacing*1.5f)/spacing);
            for(var z=Math.Max(0,pixel/n-reach);z<=Math.Min(n-1,pixel/n+reach);z++)for(var x=Math.Max(0,pixel%n-reach);x<=Math.Min(n-1,pixel%n+reach);x++)
            {var q=new float3(x*spacing-half,0,z*spacing-half);if(math.distance(q.xz,p.xz)<=radius+spacing*1.5f&&mask(q))repairCells.Add(z*n+x);}
            sources.Add(new RiverSourcePlan(way,name,"body-stage",p,radius,depth,bank,probes.Count>0?probes[probes.Count-1]:p)
                {BodyId=body,BodyKind=kind,BoundaryDriven=false,TargetSurfaceLevel=stage,SurfaceOffsetMeters=kind=="river"?1:0,BankProbes=banks,ChannelProbes=probes,BedRepairCells=repairCells});return true;
        }
        // Lakes use one flat body level, inferred conservatively from their mapped
        // shore. Internal holes stay dry. Sparse controls cover large isolated basins.
        phase="mapped lakes";Pulse(true);
        foreach(var area in areas.Where(Lake))
        {
            token.ThrowIfCancellationRequested();lakeCount++;var polygon=area.Polygon;activeBody=polygon.Id;Pulse();
            var banks=polygon.Outer.SelectMany(r=>r.Where((p,i)=>i%Math.Max(1,r.Length/64)==0)).Where(Inside)
                .Select(p=>Enumerable.Range(0,8).Select(i=>DryProbe(p,new float3(math.cos(i*math.PI/4),0,math.sin(i*math.PI/4)),area.Contains,cell*2)).FirstOrDefault(q=>q.HasValue))
                .Where(p=>p.HasValue).Select(p=>p!.Value).ToArray();
            if(banks.Length<3){unresolved++;continue;}
            var sorted=banks.Select(Height).OrderBy(h=>h).ToArray();var stage=sorted[0]-.5f;
            areaStages.Add(new WaterAreaStage(polygon.Identity,"lake",stage));
            CarveArea(area,p=>stage);
            if(NetworkEvidence.Measure(polygon.Tags,"ele") is float absolute)log.Diagnostic($"WATER ELE body={polygon.Id}; sourceAbsoluteMeters={absolute}; stageFromConsistentLocalShore=true; absoluteDatumNotAssumed=true");
            var step=Math.Max(cell*4,192);var probes=new List<float3>();
            for(var z=Math.Max(-half,area.Min.y)+cell*2;z<Math.Min(half,area.Max.y);z+=step)
            for(var x=Math.Max(-half,area.Min.x)+cell*2;x<Math.Min(half,area.Max.x);x+=step)
            {Pulse();var p=new float3(x,0,z);if(area.Contains(p)&&Inside(p))probes.Add(p);}
            if(probes.Count==0)
            {
                var center=new float3((area.Min.x+area.Max.x)*.5f,0,(area.Min.y+area.Max.y)*.5f);
                if(area.Contains(center)&&Inside(center))probes.Add(center);
            }
            foreach(var p in probes){Cut(p,step*.8f,stage-2,area.Contains,8);}
            var controls=0;
            foreach(var p in probes.Where((p,i)=>i%4==0))if(Add(polygon.Id,polygon.Identity,"lake",polygon.Tags.TryGetValue("name",out var name)?name:"lake",p,cell*2,stage,sorted[0],banks,probes.Take(24).ToArray(),area.Contains))controls++;
            log.Diagnostic($"WATER BODY id={polygon.Id}; kind=lake; stage={stage:R}; controls={controls}; shoreProbes={banks.Length}; flatStage=true; holesProtected=true");
        }
        var lines=data.Ways.Values.Where(w=>w.Tags.TryGetValue("waterway",out var type)&&(type=="river"||type=="stream"||type=="canal"||type=="drain"||type=="ditch"))
            .Where(w=>!Tag(w.Tags,"intermittent","yes")&&!Tag(w.Tags,"tunnel","yes")&&!Tag(w.Tags,"tunnel","culvert")&&w.NodeIds.Length>1&&w.NodeIds.All(data.Nodes.ContainsKey)).OrderBy(w=>w.Id).ToArray();
        totalWays=lines.Length;phase="connecting river reaches";Pulse(true);
        // Connected waterway ways share a durable body identity independent of controls.
        // Attach roots by rank, not source iteration order. A long river must
        // not build a deep parent chain before the first bed sample is processed.
        var parents=lines.ToDictionary(w=>w.Id,w=>w.Id);var ranks=lines.ToDictionary(w=>w.Id,w=>0);
        long Root(long id){while(parents[id]!=id){token.ThrowIfCancellationRequested();parents[id]=parents[parents[id]];id=parents[id];}return id;}
        var bodyIds=lines.ToDictionary(w=>w.Id,w=>w.Id);
        void Connect(long a,long b)
        {
            a=Root(a);b=Root(b);if(a==b)return;
            if(ranks[a]<ranks[b]){var swap=a;a=b;b=swap;}
            parents[b]=a;if(ranks[a]==ranks[b])ranks[a]++;
            bodyIds[a]=Math.Min(bodyIds[a],bodyIds[b]);
        }
        var ends=new Dictionary<long,long>();foreach(var w in lines)foreach(var id in w.NodeIds){Pulse();if(ends.TryGetValue(id,out var other))Connect(w.Id,other);else ends[id]=w.Id;}
        var positions=new Dictionary<long,float3>();var nodeStage=new Dictionary<long,float>();
        foreach(var w in lines)foreach(var id in w.NodeIds)if(!positions.ContainsKey(id)){Pulse();var p=Local(id);positions[id]=p;nodeStage[id]=BankHeight(p)+2.8f;}
        var bankTargets=new Dictionary<long,float>();
        var bankLimitedNodes=0;
        foreach(var w in lines)for(var i=0;i<w.NodeIds.Length;i++)
        {
            Pulse();var id=w.NodeIds[i];var p=positions[id];var neighbor=positions[w.NodeIds[i==w.NodeIds.Length-1?i-1:i+1]];
            if(math.distancesq(neighbor.xz,p.xz)<.0001f){log.Count("Water duplicate node positions skipped");continue;}
            var normal=math.normalizesafe(new float3(-(neighbor-p).z,0,(neighbor-p).x));
            var area=areaIndex.Nearby(p).FirstOrDefault(r=>Flowing(r)&&!Coastal(r)&&!Dry(r)&&r.Contains(p));
            var halfWidth=Math.Max(1,(NetworkEvidence.Measure(w.Tags,"width")??(Tag(w.Tags,"waterway","river")?32:12))*.5f);
            var bank=float.PositiveInfinity;
            foreach(var side in new[]{-1,1})
            {
                var distance=Math.Max(cell,halfWidth);
                while(distance<512&&area!=null&&area.Contains(p+normal*(distance*side)))distance+=Math.Max(cell,halfWidth*.5f);
                var q=DryProbe(p,normal*side,area!=null?area.Contains:r=>NetworkNormalizer.DistanceToSegment(r,p,neighbor)<=halfWidth,distance);
                if(q.HasValue)bank=Math.Min(bank,BankHeight(q.Value));
            }
            if(math.isfinite(bank))
            {
                var target=bank-.35f;
                if(bankTargets.TryGetValue(id,out var prior))target=Math.Min(prior,target);
                bankTargets[id]=target;
                if(nodeStage[id]>bank-.35f)bankLimitedNodes++;
            }
        }
        foreach(var p in bankTargets)nodeStage[p.Key]=p.Value;
        var desiredStage=new Dictionary<long,float>(nodeStage);
        log.Diagnostic($"WATER OFFSET CONTAINMENT requestedOffsetMeters=1; bankLimitedNodeUpdates={bankLimitedNodes}; effectiveStageCappedAtBank=true; fullOffsetNotGuaranteed=true; reason=offset-and-containment-can-conflict");
        log.Count("Water offset bank limited nodes",bankLimitedNodes);
        var baseBed=positions.ToDictionary(p=>p.Key,p=>Height(p.Value));
        var unsupportedBodies=new HashSet<long>();var sampledBankUpdates=0;var missingBankSamples=0;
        var lowerStage=positions.ToDictionary(p=>p.Key,p=>
        {
            var polygon=areaIndex.Nearby(p.Value).FirstOrDefault(a=>Flowing(a)&&!Coastal(a)&&!Dry(a)&&a.Contains(p.Value));
            return polygon!=null?.4f:Math.Max(.4f,baseBed[p.Key]-8+.4f);
        });
        phase="sampling contained river stage bounds";Pulse(true);
        // Constrain the shared endpoint solve BEFORE carving or creating any
        // regulator. Rejecting a local regulator alone cannot stop an upstream
        // regulator of the same body flooding a low bank halfway along a reach.
        foreach(var way in lines)for(var i=1;i<way.NodeIds.Length;i++)
        {
            Pulse();var aid=way.NodeIds[i-1];var bid=way.NodeIds[i];var a=positions[aid];var b=positions[bid];
            if(!WaterPlanner.Clip(a,b,half-cell*2,out var lo,out var hi)||math.distancesq(a.xz,b.xz)<.0001f)continue;
            var halfWidth=Math.Max(.5f,(NetworkEvidence.Measure(way.Tags,"width")??(Tag(way.Tags,"waterway","river")?32:12))*.5f);
            var length=math.distance(a.xz,b.xz)*(hi-lo);var samples=Math.Max(1,(int)Math.Ceiling(length/Math.Max(spacing*.5f,Math.Min(cell*2,halfWidth))));
            var normal=math.normalizesafe(new float3(-(b-a).z,0,(b-a).x));
            for(var j=0;j<=samples;j++)
            {
                Pulse();var t=math.lerp(lo,hi,j/(float)samples);var p=math.lerp(a,b,t);if(MappedDry(p))continue;
                var polygon=areaIndex.Nearby(p).FirstOrDefault(r=>Flowing(r)&&!Coastal(r)&&!Dry(r)&&r.Contains(p));
                var bank=float.PositiveInfinity;var banks=0;
                foreach(var side in new[]{-1,1})
                {
                    var distance=Math.Max(cell,halfWidth);
                    while(distance<512&&polygon!=null&&polygon.Contains(p+normal*(distance*side)))distance+=Math.Max(cell,halfWidth*.5f);
                    var q=DryProbe(p,normal*side,polygon!=null?polygon.Contains:r=>NetworkNormalizer.DistanceToSegment(r,a,b)<=halfWidth,distance);
                    if(q.HasValue){bank=Math.Min(bank,BankHeight(q.Value));banks++;}
                }
                if(banks<2||!math.isfinite(bank)){missingBankSamples++;continue;}
                var proposed=math.lerp(nodeStage[aid],nodeStage[bid],t);var excess=proposed-(bank-.35f);
                if(excess>.005f)
                {
                    // Lower both endpoints by the same excess. This preserves
                    // the reach gradient and satisfies its sampled linear bound;
                    // all later adjustments are downward, so prior bounds hold.
                    // Weighted projection cannot cumulatively lower a whole river
                    // below its recoverable bed floor. Unsatisfied local constraints
                    // remain local diagnostics, not whole-body isolation.
                    // Absolute bounds reference the immutable desired profile.
                    // Dense repeated samples cannot cumulatively subtract the
                    // same bank constraint until a whole river becomes puddles.
                    var slope=desiredStage[bid]-desiredStage[aid];var limit=bank-.35f;
                    nodeStage[aid]=Math.Min(nodeStage[aid],Math.Max(lowerStage[aid],limit-slope*t));
                    nodeStage[bid]=Math.Min(nodeStage[bid],Math.Max(lowerStage[bid],limit+slope*(1-t)));sampledBankUpdates++;
                }
                var minimum=Height(p)-(polygon!=null?12:4)+.4f;
                if(math.lerp(nodeStage[aid],nodeStage[bid],t)<minimum-.005f)unsupportedBodies.Add(bodyIds[Root(way.Id)]);
            }
        }
        foreach(var id in positions.Keys)nodeStage[id]=Math.Max(nodeStage[id],lowerStage[id]);
        foreach(var way in lines)
            if(way.NodeIds.Any(id=>nodeStage[id]<lowerStage[id]-.005f))unsupportedBodies.Add(bodyIds[Root(way.Id)]);
        log.Count("Water sampled bank bound updates",sampledBankUpdates);
        log.Diagnostic($"WATER REACH BOUNDS sampledBankUpdates={sampledBankUpdates}; missingBankSamples={missingBankSamples}; infeasibleBodiesBeforeSolve={unsupportedBodies.Count}; endpointBoundsShared=true; boundedBedRecovery=true; noLocalRegulatorFloodBypass=true");
        // OSM waterways are directed downstream. Shared source nodes must not
        // reset the river stage at every way boundary. Bound propagation by the
        // reach bed-edit budget so a bad elevation cannot flatten an entire river.
        var outgoing=positions.Keys.ToDictionary(id=>id,id=>new HashSet<long>());
        var incoming=positions.Keys.ToDictionary(id=>id,id=>0);
        foreach(var w in lines)for(var i=1;i<w.NodeIds.Length;i++)
            if(outgoing[w.NodeIds[i-1]].Add(w.NodeIds[i]))incoming[w.NodeIds[i]]++;
        var ready=incoming.Where(p=>p.Value==0).OrderBy(p=>p.Key).Select(p=>p.Key).ToList();var readyIndex=0;
        var solved=new HashSet<long>();var infeasible=0;
        phase="solving directed water stages";Pulse(true);
        void Propagate(long a,long b)
        {
            var minimum=lowerStage[b];
            if(nodeStage[a]<minimum-.005f)
            {
                infeasible++;
                if(ends.TryGetValue(a,out var way))unsupportedBodies.Add(bodyIds[Root(way)]);
                return; // Never drag the downstream DAG below recoverable beds.
            }
            nodeStage[b]=Math.Min(nodeStage[b],nodeStage[a]);
        }
        while(readyIndex<ready.Count)
        {Pulse();var a=ready[readyIndex++];solved.Add(a);foreach(var b in outgoing[a]){Propagate(a,b);if(--incoming[b]==0)ready.Add(b);}}
        // Cyclic/tidal or incorrectly directed evidence is not silently treated as
        // an ordinary river. Retain conservative local levels and expose the residual.
        var cyclic=positions.Count-solved.Count;
        var residual=outgoing.Sum(p=>p.Value.Count(b=>nodeStage[b]>nodeStage[p.Key]+.005f));
        foreach(var link in outgoing)foreach(var downstream in link.Value)
            if(nodeStage[downstream]>nodeStage[link.Key]+.005f&&ends.TryGetValue(link.Key,out var way))unsupportedBodies.Add(bodyIds[Root(way)]);
        log.Diagnostic($"WATER STAGE SOLVE algorithm=topological-v2; nodes={positions.Count}; solvedNodes={solved.Count}; cycleOrCycleDependentNodes={cyclic}; infeasibleBudgetEdges={infeasible}; uphillResidualEdges={residual}; offsetMeters=1; controlsIndependent=true; cyclicLevelsAreLocalApproximation=true");
        log.Count("Water stage cycle dependent nodes",cyclic);log.Count("Water stage uphill residual edges",residual);
        var controlGrid=new Dictionary<(int,int),List<RiverSourcePlan>>();
        bool NearControl(long body,float3 p)
        {var x=(int)Math.Floor(p.x/192);var z=(int)Math.Floor(p.z/192);for(var dx=-1;dx<=1;dx++)for(var dz=-1;dz<=1;dz++)if(controlGrid.TryGetValue((x+dx,z+dz),out var list)&&list.Any(s=>s.BodyId==body&&math.distance(s.Position.xz,p.xz)<192))return true;return false;}
        phase="conditioning river beds and placing controls";Pulse(true);
        foreach(var way in lines)
        {
            token.ThrowIfCancellationRequested();riverCount++;var body=bodyIds[Root(way.Id)];activeBody=body;Pulse();var previousStage=nodeStage[way.NodeIds[0]];
            if(unsupportedBodies.Contains(body))log.Count("Water bodies using local feasible reaches");
            for(var i=1;i<way.NodeIds.Length;i++)
            {
                var a=positions[way.NodeIds[i-1]];var b=positions[way.NodeIds[i]];if(!WaterPlanner.Clip(a,b,half-cell*2,out var lo,out var hi))continue;
                var originalA=a;var originalB=b;a=math.lerp(originalA,originalB,lo);b=originalA+(originalB-originalA)*hi;
                var sourceWidth=NetworkEvidence.Measure(way.Tags,"width")??(Tag(way.Tags,"waterway","river")?32:12);
                if(!math.isfinite(sourceWidth)||sourceWidth<=0||sourceWidth>2048){log.Count("Water width repaired");sourceWidth=Tag(way.Tags,"waterway","river")?32:12;}
                var reach=sourceWidth*.5f;
                var length=math.distance(a.xz,b.xz);var steps=Math.Max(1,(int)Math.Ceiling(length/Math.Max(spacing*.5f,Math.Min(cell*2,reach))));
                if(length<.01f){log.Count("Water zero length reaches skipped");continue;}
                var normal=math.normalizesafe(new float3(-(b-a).z,0,(b-a).x));
                for(var j=0;j<=steps;j++)
                {
                    reachSamples++;Pulse();var p=math.lerp(a,b,j/(float)steps);if(MappedDry(p))continue;var area=areaIndex.Nearby(p).FirstOrDefault(r=>Flowing(r)&&!Coastal(r)&&!Dry(r)&&r.Contains(p));
                    // Do not silently inflate a known 4m canal to the native
                    // 56m source diameter, or reinterpret an island as unmapped water.
                    Func<float3,bool> mask=area!=null?q=>area.Contains(q)&&!MappedDry(q):q=>!MappedDry(q)&&NetworkNormalizer.DistanceToSegment(q,a,b)<=reach;
                    var banks=new List<float3>();
                    foreach(var side in new[]{-1,1})
                    {
                        var distance=Math.Max(cell,reach);
                        while(distance<512&&mask(p+normal*(distance*side)))distance+=Math.Max(cell,reach*.5f);
                        var bankPoint=DryProbe(p,normal*side,mask,distance);if(bankPoint.HasValue)banks.Add(bankPoint.Value);
                    }
                    if(banks.Count<2){unresolved++;continue;}
                    var bank=banks.Min(BankHeight);var stage=Math.Min(bank-.35f,math.lerp(nodeStage[way.NodeIds[i-1]],nodeStage[way.NodeIds[i]],math.lerp(lo,hi,j/(float)steps)));
                    var floor=area!=null?.4f:Math.Max(.4f,Height(p)-8+.4f);
                    if(stage<floor){unresolved++;log.Count("Water locally infeasible reaches");continue;}
                    if(stage>bank-.25f)
                    {unresolved++;log.Count("Water sampled final bank bound violations");continue;}
                    var stagePixel=Pixel(p);riverSurface[stagePixel]=riverSurface[stagePixel]>0?Math.Min(riverSurface[stagePixel],stage):stage;
                    // Publish stage evidence even when a native source is subgrid.
                    // Keep the shared-node solve authoritative rather than independently
                    // lowering each way at confluences. Bank conflicts reject controls.
                    if(j<steps)
                    {
                        var next=math.lerp(a,b,(j+1)/(float)steps);
                        var nextLevel=Math.Min(stage,math.lerp(nodeStage[way.NodeIds[i-1]],nodeStage[way.NodeIds[i]],math.lerp(lo,hi,(j+1)/(float)steps)));
                        var candidate=new WaterReachStage(body,way.Id,p,next,stage,nextLevel,reach,area?.Polygon.Identity);
                        var last=reachStages.Count>0?reachStages[reachStages.Count-1]:null;
                        if(last!=null&&last.WayId==candidate.WayId&&last.PolygonId==candidate.PolygonId&&math.distance(last.End.xz,p.xz)<.01f&&
                            math.dot(math.normalizesafe((last.End-last.Start).xz),math.normalizesafe((next-p).xz))>.99999f&&
                            Math.Abs((last.EndLevel-last.StartLevel)/Math.Max(.1f,math.distance(last.Start.xz,last.End.xz))-(nextLevel-stage)/Math.Max(.1f,math.distance(p.xz,next.xz)))<.0001f)
                            reachStages[reachStages.Count-1]=last with{End=next,EndLevel=nextLevel};
                        else reachStages.Add(candidate);
                    }
                    // Bound cross-way inconsistencies in later body diagnostics rather
                    // than inventing an absolute river elevation from OSM layer tags.
                    // Width tags/defaults describe narrow centerline-only reaches.
                    // When a river polygon supplies actual banks, condition that
                    // mapped wet cross-section instead of carving a 32m ribbon
                    // down the middle of a several-hundred-metre-wide river.
                    var wetRadius=area!=null?Math.Min(512,banks.Min(q=>math.distance(q.xz,p.xz)))*.95f:reach;
                    previousStage=stage;Cut(p,wetRadius/.7f,stage-2.5f,mask,area!=null?float.PositiveInfinity:8);
                    if(area==null&&sourceWidth<cell*2)log.Count("Water mapped subgrid sites using contained controls");
                    if(NearControl(body,p))continue;
                    var radius=area!=null?Math.Max(cell,Math.Min(192,wetRadius*.55f)):Math.Max(2,Math.Min(cell,reach*.7f));if(Add(way.Id,body,"river",way.Tags.TryGetValue("name",out var name)?name:"waterway",p,radius,stage,bank,banks,new[]{p,math.lerp(p,b,.5f),b},mask))
                    {var key=((int)Math.Floor(p.x/192),(int)Math.Floor(p.z/192));if(!controlGrid.TryGetValue(key,out var list))controlGrid[key]=list=new();list.Add(sources[sources.Count-1]);}
                }
            }
        }
        phase="publishing conditioned water terrain";Pulse(true);
        // Multi-source propagation gives every mapped channel cell the nearby
        // solved reach level in linear raster work, not a per-pixel scan of all
        // river segments. Islands and disconnected polygons cannot be crossed.
        var wet=new byte[conditioned.Length];
        foreach(var area in areas.Where(a=>Flowing(a)&&!Coastal(a)&&!Dry(a)))
        {
            var loX=Math.Max(0,(int)Math.Ceiling((area.Min.x+half)/spacing));var hiX=Math.Min(n-1,(int)Math.Floor((area.Max.x+half)/spacing));
            var loZ=Math.Max(0,(int)Math.Ceiling((area.Min.y+half)/spacing));var hiZ=Math.Min(n-1,(int)Math.Floor((area.Max.y+half)/spacing));
            for(var z=loZ;z<=hiZ;z++)for(var x=loX;x<=hiX;x++)
            {if((bedVisits++&4095)==0)Pulse();var p=new float3(x*spacing-half,0,z*spacing-half);if(area.Contains(p)&&!MappedDry(p))wet[z*n+x]=1;}
        }
        var riverQueue=new int[conditioned.Length];var head=0;var tail=0;
        for(var i=0;i<wet.Length;i++){if((i&16383)==0)Pulse();if(wet[i]!=0&&riverSurface[i]>0)riverQueue[tail++]=i;else riverSurface[i]=0;}
        while(head<tail)
        {
            if((head&4095)==0)Pulse();var i=riverQueue[head++];var x=i%n;var z=i/n;
            void Visit(int next){if(next<0||wet[next]==0||riverSurface[next]>0)return;riverSurface[next]=riverSurface[i];riverQueue[tail++]=next;}
            Visit(x>0?i-1:-1);Visit(x+1<n?i+1:-1);Visit(z>0?i-n:-1);Visit(z+1<n?i+n:-1);
        }
        foreach(var area in areas.Where(a=>Flowing(a)&&!Coastal(a)&&!Dry(a)))CarveArea(area,p=>riverSurface[Pixel(p)]>0?riverSurface[Pixel(p)]:(float?)null);
        log.Diagnostic($"WATER CHANNEL CONTINUITY mappedWetCells={wet.Count(v=>v!=0)}; solvedWetCells={tail}; disconnectedUnseededCells={wet.Count(v=>v!=0)-tail}; islandsPreserved=true; sourceLevelPropagation=nearestConnectedRasterSeed; quadraticReachSearch=false");
        log.Diagnostic($"WATER BANKFULL PROFILE targetBankFreeboardMeters=.35; immutableBankTerrain=true; nonCumulativeSampleBounds=true; initialCenterlineOffsetMeters=2.8; fullWidthDepthMeters=2.5; widthAwareSourceRadius=true; mappedRiverBedBudget=maskOwned; finalNodes={nodeStage.Count}; minimumFinalStage={nodeStage.Values.DefaultIfEmpty(0).Min():R}; maximumFinalStage={nodeStage.Values.DefaultIfEmpty(0).Max():R}");
        // River cross-sections were already stamped at <= half DEM spacing or
        // half mapped width. Do not perform a second polygon-wide nearest-reach
        // search per raster cell: it can turn dense hydrology into quadratic work.
        log.Diagnostic($"WATER RIVER OFFSET requestedMeters=1; controls={sources.Count(s=>s.BodyKind=="river")}; appliedOnceBeforeSafetyChecks=true; lakeAndSeaOffsets=0; bankLimitedNodeUpdates={bankLimitedNodes}; fullOffsetNotGuaranteed=true; bankConflictsRejectControls=true");
        TerrainArtifactRepair.TraceEncoded(conditioned, terrain.HeightScaleMeters, terrain.SeaLevelMeters, "after-water-bed-planning", log.Diagnostic);
        var coastal=data.Ways.Values.Any(w=>Tag(w.Tags,"natural","coastline"));
        var report=$"Water: {sources.Count} bank-limited river/lake controls; {lakeCount} mapped lakes; {unresolved} unresolved narrow/height sites. No flooding bypass. Coverage requires simulation verification.";
        log.Info(report);log.Diagnostic($"WATER BODY SUMMARY bodies={sources.Select(s=>s.BodyId).Distinct().Count()}; controls={sources.Count}; bedEdits={editCount}; cellMeters={cell:R}; mappedRiverBankWidthUsed=true; polygonBedCutsMaskOwned=true; fixedDepthFallback=false; floodingAccepted=false; unresolved={unresolved}; indexedPolygonPredicates=true; bedCellsVisited={bedVisits}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.###}; canonicalAreas={areaStages.Count}; canonicalReaches={reachStages.Count}; maskBytes={editMask.Length}; geometryVersion={WaterPlan.GeometryVersion}");
        return new WaterPlan(coastal,riverCount,lakeCount,unresolved,report){Sources=sources,ConditionedHeightmap=editCount>0?conditioned:null,PlannedBedHeightmap=conditioned,WaterCellSize=cell,AreaStages=areaStages,ReachStages=reachStages,BedEdits=new TerrainEditSet("water-bed","pre-water-terrain",n,editMask)};
    }

    public static WaterPlan Revalidate(WaterPlan plan,TerrainBuilder.Result terrain,double width,ImportLog log,CancellationToken token,IReadOnlyList<PreparedSegment>? networks=null,Action<string>? progress=null)
    {
        token.ThrowIfCancellationRequested();
        if(plan.Sources.Count==0&&plan.BedEdits==null)
        {
            // No controls can request a new cut. The preparation raster is
            // already applied upstream; never reuse it over newer roadbeds.
            log.Diagnostic("WATER FINAL REVALIDATION controls=0; noAcceptedControls=true; networkProtectionRasterizationSkipped=true; finalTerrainRetained=true");
            return plan with{ConditionedHeightmap=null};
        }
        var clock=Stopwatch.StartNew();var lastPulse=-5d;var networkCount=0;var sourceCount=0;long protectionVisits=0;
        void Pulse(bool force=false)
        {
            token.ThrowIfCancellationRequested();if(!force&&clock.Elapsed.TotalSeconds-lastPulse<5)return;
            lastPulse=clock.Elapsed.TotalSeconds;
            progress?.Invoke($"Checking final water terrain: {networkCount}/{networks?.Count??0} network sections; {sourceCount}/{plan.Sources.Count} controls");
            log.Diagnostic($"WATER FINAL PROGRESS networks={networkCount}/{networks?.Count??0}; controls={sourceCount}/{plan.Sources.Count}; protectionCellsVisited={protectionVisits}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.###}; cancellationChecked=true");
        }
        Pulse(true);
        var raster=(ushort[])terrain.Heightmap.Clone();var n=(int)Math.Sqrt(raster.Length);var cell=(float)width/(n-1);var half=(float)width*.5f;
        float Height(float3 p)=>NetworkProcessor.Sample(p.x,p.z,raster,terrain.HeightScaleMeters,width);
        int Pixel(float value)=>(int)Math.Round((value/width+.5)*(n-1));
        // Recovery is confined to previously accepted control footprints. Never
        // carve through a road bed, a bridge support envelope or a tunnel roof.
        var protectedCells=new byte[raster.Length];var finalEdits=new byte[raster.Length];
        foreach(var s in networks??Array.Empty<PreparedSegment>())
        {
            networkCount++;Pulse();
            var count=Math.Max(1,(int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(s)/Math.Max(1,cell*.5f)));var radius=Math.Max(4,s.NativeWidth*.5f)+cell*2;var reach=(int)Math.Ceiling(radius/cell);
            for(var i=0;i<=count;i++)
            {
                Pulse();var p=NetworkNormalizer.Evaluate(s,i/(float)count);var px=Pixel(p.x);var pz=Pixel(p.z);
                for(var z=Math.Max(0,pz-reach);z<=Math.Min(n-1,pz+reach);z++)for(var x=Math.Max(0,px-reach);x<=Math.Min(n-1,px+reach);x++)
                {
                    if((protectionVisits++&4095)==0)Pulse();
                    if(math.distance(new float2(x*cell-half,z*cell-half),p.xz)<=radius)protectedCells[z*n+x]=1;
                }
            }
        }
        var revised=new List<RiverSourcePlan>();var adjusted=0;var repaired=0;var blocked=0;
        var owned=plan.BedEdits;var plannedBed=plan.PlannedBedHeightmap;var supportBlocked=0;
        if(owned!=null&&plannedBed!=null&&owned.Mask.Length==raster.Length&&plannedBed.Length==raster.Length)
            for(var index=0;index<raster.Length;index++)
            {
                if((index&4095)==0)Pulse();if(owned.Mask[index]==0||raster[index]<=plannedBed[index])continue;
                if(protectedCells[index]!=0){supportBlocked++;continue;}
                var cut=(raster[index]-plannedBed[index])/65535f*terrain.HeightScaleMeters;
                // This restores the exact previously accepted water-owned bed,
                // not a new unbounded cut or a change to islands/infrastructure.
                raster[index]=plannedBed[index];finalEdits[index]=1;repaired++;
            }
        foreach(var source in plan.Sources)
        {
            sourceCount++;Pulse();var p=source.Position;p.y=Height(p);var bank=source.BankProbes.Count>0?source.BankProbes.Min(Height):source.BankLevel;
            var stage=source.TargetSurfaceLevel??source.Position.y+source.Depth;
            if(stage>bank-.25f)
            {log.Count("Water final bank conflicts");log.Diagnostic($"WATER FINAL UNRESOLVED body={source.BodyId}; way={source.WayId}; stage={stage:R}; bank={bank:R}; reason=final-bank-lowered; canonicalStageRetained=true; controlRejected=true");continue;}
            // Repair only modest new obstructions. Larger or infrastructure-owned
            // changes need a combined regional redesign, not a flooding source.
            foreach(var index in source.BedRepairCells)
            {
                token.ThrowIfCancellationRequested();if(index<0||index>=raster.Length||protectedCells[index]!=0)continue;
                var q=new float3((index%n)*cell-half,0,(index/n)*cell-half);
                if(source.BankProbes.Any(b=>math.distance(b.xz,q.xz)<cell*2))continue;
                var old=raster[index]/65535f*terrain.HeightScaleMeters;var target=stage-.6f;if(old<=target)continue;
                if(owned==null||owned.Mask.Length!=raster.Length||owned.Mask[index]==0){blocked++;continue;}
                raster[index]=(ushort)Math.Ceiling(math.clamp(target/terrain.HeightScaleMeters*65535,0,65535));finalEdits[index]=1;repaired++;
            }
            p.y=Height(p);
            if(stage-p.y<.2f){log.Diagnostic($"WATER FINAL UNRESOLVED body={source.BodyId}; way={source.WayId}; bed={p.y:R}; stage={stage:R}; reason=earthworks-blocked-control; noFloodBypass=true");continue;}
            if(Math.Abs(stage-(source.TargetSurfaceLevel??stage))>.01f||Math.Abs(p.y-source.Position.y)>.01f)adjusted++;
            revised.Add(source with{Position=p,Depth=stage-p.y,TargetSurfaceLevel=stage,BankLevel=bank,ChannelProbes=source.ChannelProbes.Select(q=>new float3(q.x,Height(q),q.z)).ToArray()});
        }
        log.Diagnostic($"WATER FINAL REVALIDATION controls={revised.Count}/{plan.Sources.Count}; adjusted={adjusted}; repairedBedSamples={repaired}; nonOwnedObstructionsBlocked={blocked}; supportBlockedSamples={supportBlocked}; restoresAcceptedOwnedBedWithoutTwoMeterCap=true; infrastructureAndBanksProtected=true; lakesKeepOneStage=true; afterEarthworks=true; canonicalStagePreserved=true; recoveryUsesCompleteOwnedWetMask=true; protectionMaskBytes={protectedCells.Length}; producerMaskBytes={finalEdits.Length}");
        TerrainArtifactRepair.TraceEncoded(raster, terrain.HeightScaleMeters, terrain.SeaLevelMeters, "after-water-final-revalidation", log.Diagnostic);
        return plan with{Sources=revised,ConditionedHeightmap=repaired>0?raster:null,FinalBedEdits=new TerrainEditSet("water-final-bed","post-roadbed-terrain",n,finalEdits),Skipped=plan.Skipped+plan.Sources.Count-revised.Count};
    }
}
