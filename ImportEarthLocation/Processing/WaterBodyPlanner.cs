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
        var grid=water.Sources.Where(s=>s.TargetSurfaceLevel.HasValue).GroupBy(s=>((int)Math.Floor(s.Position.x/2048),(int)Math.Floor(s.Position.z/2048))).ToDictionary(g=>g.Key,g=>g.ToArray());
        return p=>
        {
            var area=maskIndex.Nearby(p).FirstOrDefault(a=>a.Contains(p));
            if(area==null)return null;
            var river=area.Polygon.Tags.TryGetValue("water",out var type)&&type=="river"||area.Polygon.Tags.ContainsKey("waterway");
            RiverSourcePlan? nearest=null;var best=2048f*2048;
            var x=(int)Math.Floor(p.x/2048);var z=(int)Math.Floor(p.z/2048);
            for(var dx=-1;dx<=1;dx++)for(var dz=-1;dz<=1;dz++)if(grid.TryGetValue((x+dx,z+dz),out var list))foreach(var s in list)
            {
                if(river?s.BodyKind=="lake"||!area.Contains(s.Position):s.BodyId!=area.Polygon.Id)continue;
                var distance=math.distancesq(p.xz,s.Position.xz);if(distance>=best)continue;best=distance;nearest=s;
            }
            return nearest?.TargetSurfaceLevel;
        };
    }
    public static WaterPlan Prepare(OsmDataset? data,IReadOnlyList<GeographicPolygon> polygons,ImportRequest request,
        TerrainBuilder.Result terrain,ImportLog log,CancellationToken token,float cell,Action<string>? progress=null)
    {
        if(data==null||!math.isfinite(cell)||cell<=0)return new WaterPlan(false,0,0,0,"Water evidence or native grid unavailable; water preparation cannot be verified.");
        var width=(float)request.MapSize.WidthMeters;var half=width*.5f;var n=(int)Math.Sqrt(terrain.Heightmap.Length);
        var spacing=width/(n-1);var edits=new Dictionary<int,ushort>();var sources=new List<RiverSourcePlan>();
        var unresolved=0;var riverCount=0;var lakeCount=0;
        var clock=Stopwatch.StartNew();var lastPulse=-5d;var phase="indexing mapped water";long activeBody=0;long bedVisits=0;long reachSamples=0;var totalWays=0;
        void Pulse(bool force=false)
        {
            token.ThrowIfCancellationRequested();if(!force&&clock.Elapsed.TotalSeconds-lastPulse<5)return;
            lastPulse=clock.Elapsed.TotalSeconds;
            progress?.Invoke($"Planning water: {phase}; {riverCount}/{totalWays} waterways; {sources.Count} controls");
            log.Diagnostic($"WATER PLAN PROGRESS phase={phase}; body={activeBody}; lakes={lakeCount}; waterways={riverCount}/{totalWays}; reachSamples={reachSamples}; bedCellsVisited={bedVisits}; bedEdits={edits.Count}; controls={sources.Count}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.###}; cancellationChecked=true");
        }
        Pulse(true);
        bool Tag(IReadOnlyDictionary<string,string> tags,string k,string v)=>tags.TryGetValue(k,out var value)&&value==v;
        bool Inside(float3 p)=>Math.Abs(p.x)<=half-cell*2&&Math.Abs(p.z)<=half-cell*2;
        int Pixel(float3 p)=>(int)Math.Round(math.clamp(p.z/width+.5f,0,1)*(n-1))*n+(int)Math.Round(math.clamp(p.x/width+.5f,0,1)*(n-1));
        float Height(float3 p){var index=Pixel(p);return (edits.TryGetValue(index,out var h)?h:terrain.Heightmap[index])/65535f*terrain.HeightScaleMeters;}
        float3 Local(long id){var p=data.Nodes[id];return GeoMath.ToLocalMeters(p.Latitude,p.Longitude,request.Latitude,request.Longitude);}
        var areas=polygons.Where(p=>Tag(p.Tags,"natural","water")||Tag(p.Tags,"waterway","riverbank")||Tag(p.Tags,"landuse","reservoir"))
            .Where(p=>p.Outer.Any(r=>r.Length>0)).Select((p,i)=>{Pulse();return new Area(p,i,token,()=>Pulse());}).ToArray();
        var areaIndex=new AreaIndex(areas,token);var holeIndex=new AreaIndex(areas.Where(a=>a.HasHoles),token);
        bool Lake(Area a)=>!Tag(a.Polygon.Tags,"water","river")&&!Tag(a.Polygon.Tags,"waterway","riverbank");
        bool MappedDry(float3 p)=>holeIndex.Nearby(p).Any(a=>a.InHole(p));
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
                if(encoded>=terrain.Heightmap[index]||edits.TryGetValue(index,out var old)&&encoded>=old)continue;
                if(mask(p))edits[index]=encoded;
            }
        }
        bool Add(long way,long body,string kind,string name,float3 p,float radius,float stage,float bank,IReadOnlyList<float3> banks,IReadOnlyList<float3> probes,Func<float3,bool> mask)
        {
            if(!Inside(p)||radius<cell||stage<=WaterLevels.Effective(terrain.SeaLevelMeters))return false;
            // No circle spilling onto banks or islands, even for boundary-driven rivers.
            bool Fits(float r)=>Enumerable.Range(0,16).All(i=>mask(p+new float3(math.cos(i*math.PI/8),0,math.sin(i*math.PI/8))*r));
            while(radius>cell&&!Fits(radius))radius-=cell*.5f;
            if(!Fits(radius)||stage>bank-.35f){unresolved++;return false;}
            Cut(p,radius+spacing*2,stage-1.5f,mask,kind=="lake"?8:12);
            p.y=Height(p);var depth=stage-p.y;
            if(depth<.4f){unresolved++;log.Diagnostic($"WATER BODY UNRESOLVED body={body}; reason=bed-above-bank-limited-stage; bed={p.y:R}; stage={stage:R}; noFloodBypass=true");return false;}
            var repairCells=new List<int>();var pixel=Pixel(p);var reach=(int)Math.Ceiling((radius+spacing*1.5f)/spacing);
            for(var z=Math.Max(0,pixel/n-reach);z<=Math.Min(n-1,pixel/n+reach);z++)for(var x=Math.Max(0,pixel%n-reach);x<=Math.Min(n-1,pixel%n+reach);x++)
            {var q=new float3(x*spacing-half,0,z*spacing-half);if(math.distance(q.xz,p.xz)<=radius+spacing*1.5f&&mask(q))repairCells.Add(z*n+x);}
            sources.Add(new RiverSourcePlan(way,name,"body-stage",p,radius,depth,bank,probes.Count>0?probes[probes.Count-1]:p)
                {BodyId=body,BodyKind=kind,BoundaryDriven=false,TargetSurfaceLevel=stage,BankProbes=banks,ChannelProbes=probes,BedRepairCells=repairCells});return true;
        }
        // Lakes use one flat body level, inferred conservatively from their mapped
        // shore. Internal holes stay dry. Sparse controls cover large isolated basins.
        phase="mapped lakes";Pulse(true);
        foreach(var area in areas.Where(Lake))
        {
            token.ThrowIfCancellationRequested();lakeCount++;var polygon=area.Polygon;activeBody=polygon.Id;Pulse();
            var banks=polygon.Outer.SelectMany(r=>r.Where((p,i)=>i%Math.Max(1,r.Length/64)==0)).Where(Inside).ToArray();
            if(banks.Length<3){unresolved++;continue;}
            var sorted=banks.Select(Height).OrderBy(h=>h).ToArray();var stage=sorted[0]-.5f;
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
            foreach(var p in probes.Where((p,i)=>i%4==0))if(Add(polygon.Id,polygon.Id,"lake",polygon.Tags.TryGetValue("name",out var name)?name:"lake",p,cell*2,stage,sorted[0],banks,probes.Take(24).ToArray(),area.Contains))controls++;
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
        var ends=new Dictionary<long,long>();foreach(var w in lines)foreach(var id in new[]{w.NodeIds[0],w.NodeIds[w.NodeIds.Length-1]}){Pulse();if(ends.TryGetValue(id,out var other))Connect(w.Id,other);else ends[id]=w.Id;}
        var positions=new Dictionary<long,float3>();var nodeStage=new Dictionary<long,float>();
        foreach(var w in lines)foreach(var id in w.NodeIds)if(!positions.ContainsKey(id)){Pulse();var p=Local(id);positions[id]=p;nodeStage[id]=Height(p)+.8f;}
        var baseBed=positions.ToDictionary(p=>p.Key,p=>Height(p.Value));
        // OSM waterways are directed downstream. Shared source nodes must not
        // reset the river stage at every way boundary. Bound propagation by the
        // reach bed-edit budget so a bad elevation cannot flatten an entire river.
        for(var pass=0;pass<24;pass++)
        {
            phase=$"solving river levels pass {pass+1}/24";Pulse(true);
            var changed=false;foreach(var w in lines)for(var i=1;i<w.NodeIds.Length;i++)
            {Pulse();var a=w.NodeIds[i-1];var b=w.NodeIds[i];var target=Math.Max(baseBed[b]-3,Math.Min(nodeStage[b],nodeStage[a]));if(target<nodeStage[b]-.005f){nodeStage[b]=target;changed=true;}}
            if(!changed)break;
        }
        var controlGrid=new Dictionary<(int,int),List<RiverSourcePlan>>();
        bool NearControl(long body,float3 p)
        {var x=(int)Math.Floor(p.x/192);var z=(int)Math.Floor(p.z/192);for(var dx=-1;dx<=1;dx++)for(var dz=-1;dz<=1;dz++)if(controlGrid.TryGetValue((x+dx,z+dz),out var list)&&list.Any(s=>s.BodyId==body&&math.distance(s.Position.xz,p.xz)<192))return true;return false;}
        phase="conditioning river beds and placing controls";Pulse(true);
        foreach(var way in lines)
        {
            token.ThrowIfCancellationRequested();riverCount++;var body=bodyIds[Root(way.Id)];activeBody=body;Pulse();var previousStage=nodeStage[way.NodeIds[0]];
            for(var i=1;i<way.NodeIds.Length;i++)
            {
                var a=positions[way.NodeIds[i-1]];var b=positions[way.NodeIds[i]];if(!WaterPlanner.Clip(a,b,half-cell*2,out var lo,out var hi))continue;
                var originalA=a;var originalB=b;a=math.lerp(originalA,originalB,lo);b=originalA+(originalB-originalA)*hi;
                var length=math.distance(a.xz,b.xz);var steps=Math.Max(1,(int)Math.Ceiling(length/Math.Max(cell*2,16)));
                var normal=math.normalizesafe(new float3(-(b-a).z,0,(b-a).x));
                for(var j=0;j<=steps;j++)
                {
                    reachSamples++;Pulse();var p=math.lerp(a,b,j/(float)steps);if(MappedDry(p))continue;var area=areaIndex.Nearby(p).FirstOrDefault(r=>!Lake(r)&&r.Contains(p));
                    var sourceWidth=NetworkEvidence.Measure(way.Tags,"width")??(Tag(way.Tags,"waterway","river")?32:12);
                    // Do not silently inflate a known 4m canal to the native
                    // 56m source diameter, or reinterpret an island as unmapped water.
                    if(area==null&&sourceWidth<cell*2){unresolved++;continue;}
                    var reach=Math.Max(cell,sourceWidth*.5f);Func<float3,bool> mask=area!=null?area.Contains:q=>!MappedDry(q)&&NetworkNormalizer.DistanceToSegment(q,a,b)<=reach;
                    var banks=new List<float3>();
                    foreach(var side in new[]{-1,1})
                    {
                        var distance=Math.Max(cell,reach);
                        while(distance<512&&mask(p+normal*(distance*side)))distance+=Math.Max(cell,reach*.5f);
                        var bankPoint=p+normal*(distance*side);if(Math.Abs(bankPoint.x)<=half&&Math.Abs(bankPoint.z)<=half)banks.Add(bankPoint);
                    }
                    if(banks.Count<2){unresolved++;continue;}
                    var bank=banks.Min(Height);var stage=Math.Min(previousStage,Math.Min(bank-.5f,math.lerp(nodeStage[way.NodeIds[i-1]],nodeStage[way.NodeIds[i]],math.lerp(lo,hi,j/(float)steps))));
                    // Bound cross-way inconsistencies in later body diagnostics rather
                    // than inventing an absolute river elevation from OSM layer tags.
                    // Width tags/defaults describe narrow centerline-only reaches.
                    // When a river polygon supplies actual banks, condition that
                    // mapped wet cross-section instead of carving a 32m ribbon
                    // down the middle of a several-hundred-metre-wide river.
                    var wetRadius=area!=null?Math.Max(cell*1.5f,Math.Min(512,banks.Min(q=>math.distance(q.xz,p.xz)))*.95f):Math.Max(cell*1.5f,reach);
                    previousStage=stage;Cut(p,wetRadius,stage-1.5f,mask,area!=null?12:4);
                    if(NearControl(body,p))continue;
                    var radius=cell;if(Add(way.Id,body,"river",way.Tags.TryGetValue("name",out var name)?name:"waterway",p,radius,stage,bank,banks,new[]{p,math.lerp(p,b,.5f),b},mask))
                    {var key=((int)Math.Floor(p.x/192),(int)Math.Floor(p.z/192));if(!controlGrid.TryGetValue(key,out var list))controlGrid[key]=list=new();list.Add(sources[sources.Count-1]);}
                }
            }
        }
        phase="publishing conditioned water terrain";Pulse(true);
        for (var i = 0; i < sources.Count; i++)
        {
            var source = sources[i]; if (source.BodyKind != "river") continue;
            sources[i] = source with { Depth = source.Depth + 1, TargetSurfaceLevel = (source.TargetSurfaceLevel ?? source.Position.y + source.Depth) + 1, SurfaceOffsetMeters = 1 };
        }
        log.Diagnostic($"WATER RIVER OFFSET meters=1; controls={sources.Count(s=>s.BodyKind=="river")}; appliedOnceAtPlanning=true; lakeAndSeaOffsets=0; shallowBankOvertoppingRisk=true");
        var conditioned=(ushort[])terrain.Heightmap.Clone();foreach(var edit in edits)conditioned[edit.Key]=edit.Value;
        TerrainArtifactRepair.TraceEncoded(conditioned, terrain.HeightScaleMeters, terrain.SeaLevelMeters, "after-water-bed-planning", log.Diagnostic);
        var coastal=data.Ways.Values.Any(w=>Tag(w.Tags,"natural","coastline"));
        var report=$"Water: {sources.Count} bank-limited river/lake controls; {lakeCount} mapped lakes; {unresolved} unresolved narrow/height sites. No flooding bypass. Coverage requires simulation verification.";
        log.Info(report);log.Diagnostic($"WATER BODY SUMMARY bodies={sources.Select(s=>s.BodyId).Distinct().Count()}; controls={sources.Count}; bedEdits={edits.Count}; cellMeters={cell:R}; mappedRiverBankWidthUsed=true; maximumPolygonBedCutMeters=12; fixedDepthFallback=false; floodingAccepted=false; unresolved={unresolved}; indexedPolygonPredicates=true; bedCellsVisited={bedVisits}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.###}");
        return new WaterPlan(coastal,riverCount,lakeCount,unresolved,report){Sources=sources,ConditionedHeightmap=edits.Count>0?conditioned:null,WaterCellSize=cell};
    }

    public static WaterPlan Revalidate(WaterPlan plan,TerrainBuilder.Result terrain,double width,ImportLog log,CancellationToken token,IReadOnlyList<PreparedSegment>? networks=null,Action<string>? progress=null)
    {
        token.ThrowIfCancellationRequested();
        if(plan.Sources.Count==0)
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
        var protectedCells=new HashSet<int>();
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
                    if(math.distance(new float2(x*cell-half,z*cell-half),p.xz)<=radius)protectedCells.Add(z*n+x);
                }
            }
        }
        var lakeStages=plan.Sources.Where(s=>s.BodyKind=="lake").GroupBy(s=>s.BodyId).ToDictionary(g=>g.Key,g=>g.Min(s=>Math.Min(s.TargetSurfaceLevel??s.Position.y+s.Depth,s.BankProbes.Count>0?s.BankProbes.Min(Height)-.35f:s.BankLevel-.35f)));
        var revised=new List<RiverSourcePlan>();var adjusted=0;var repaired=0;var blocked=0;
        foreach(var source in plan.Sources)
        {
            sourceCount++;Pulse();var p=source.Position;p.y=Height(p);var bank=source.BankProbes.Count>0?source.BankProbes.Min(Height):source.BankLevel;
            var stage=source.BodyKind=="lake"?lakeStages[source.BodyId]:Math.Min(source.TargetSurfaceLevel??source.Position.y+source.Depth,bank-.35f+math.clamp(source.SurfaceOffsetMeters,0,1));
            // Repair only modest new obstructions. Larger or infrastructure-owned
            // changes need a combined regional redesign, not a flooding source.
            foreach(var index in source.BedRepairCells)
            {
                token.ThrowIfCancellationRequested();if(index<0||index>=raster.Length||protectedCells.Contains(index))continue;
                var q=new float3((index%n)*cell-half,0,(index/n)*cell-half);
                if(source.BankProbes.Any(b=>math.distance(b.xz,q.xz)<cell*2))continue;
                var old=raster[index]/65535f*terrain.HeightScaleMeters;var target=stage-.6f;if(old<=target)continue;
                if(old-target>2){blocked++;continue;}
                raster[index]=(ushort)Math.Ceiling(math.clamp(target/terrain.HeightScaleMeters*65535,0,65535));repaired++;
            }
            p.y=Height(p);
            if(stage-p.y<.2f){log.Diagnostic($"WATER FINAL UNRESOLVED body={source.BodyId}; way={source.WayId}; bed={p.y:R}; stage={stage:R}; reason=earthworks-blocked-control; noFloodBypass=true");continue;}
            if(Math.Abs(stage-(source.TargetSurfaceLevel??stage))>.01f||Math.Abs(p.y-source.Position.y)>.01f)adjusted++;
            revised.Add(source with{Position=p,Depth=stage-p.y,TargetSurfaceLevel=stage,BankLevel=bank,ChannelProbes=source.ChannelProbes.Select(q=>new float3(q.x,Height(q),q.z)).ToArray()});
        }
        log.Diagnostic($"WATER FINAL REVALIDATION controls={revised.Count}/{plan.Sources.Count}; adjusted={adjusted}; repairedBedSamples={repaired}; blockedLargeObstructions={blocked}; maximumNewCutMeters=2; infrastructureAndBanksProtected=true; lakesKeepOneStage=true; afterEarthworks=true; absoluteStagePreservedUnlessBankLowered=true");
        TerrainArtifactRepair.TraceEncoded(raster, terrain.HeightScaleMeters, terrain.SeaLevelMeters, "after-water-final-revalidation", log.Diagnostic);
        return plan with{Sources=revised,ConditionedHeightmap=repaired>0?raster:null,Skipped=plan.Skipped+plan.Sources.Count-revised.Count};
    }
}
