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

        // Exact distance to the nearest outer or inner edge, or +∞ beyond limit.
        // Edges sit in 64 m cells, built on first use; a long river outline has
        // thousands of vertices, so a per-sample scan of every edge is too slow.
        private const float EdgeCell=64;
        private Dictionary<(int,int),List<(float2 A,float2 B)>>? _edges;
        public float EdgeDistance(float3 p,float limit,out float2 closest)
        {
            if(_edges==null)
            {
                _edges=new();
                foreach(var ring in Polygon.Outer.Concat(Polygon.Inner))for(var i=0;i<ring.Length;i++)
                {
                    var a=ring[i].xz;var b=ring[(i+1)%ring.Length].xz;var lo=math.floor(math.min(a,b)/EdgeCell);var hi=math.floor(math.max(a,b)/EdgeCell);
                    if((hi.x-lo.x+1)*(hi.y-lo.y+1)>4096)continue;
                    for(var x=(int)lo.x;x<=(int)hi.x;x++)for(var z=(int)lo.y;z<=(int)hi.y;z++)
                    {if(!_edges.TryGetValue((x,z),out var list))_edges[(x,z)]=list=new();list.Add((a,b));}
                }
            }
            var best=float.PositiveInfinity;closest=p.xz;var reach=(int)Math.Ceiling(limit/EdgeCell);
            var cx=(int)Math.Floor(p.x/EdgeCell);var cz=(int)Math.Floor(p.z/EdgeCell);
            for(var x=cx-reach;x<=cx+reach;x++)for(var z=cz-reach;z<=cz+reach;z++)if(_edges.TryGetValue((x,z),out var list))
                foreach(var (a,b) in list)
                {
                    var ab=b-a;var t=math.clamp(math.dot(p.xz-a,ab)/Math.Max(1e-9f,math.lengthsq(ab)),0,1);var q=a+ab*t;var d=math.distance(p.xz,q);
                    if(d<best){best=d;closest=q;}
                }
            return best<=limit?best:float.PositiveInfinity;
        }
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
    public static Func<float3,float?> SurfaceSampler(WaterPlan water,IReadOnlyList<GeographicPolygon> polygons)=>SurfaceSampler(water,polygons,false);
    // Include the continuous channel cut for the native water grid as well as
    // mapped banks. Source circles alone leave dry-looking gaps between controls.
    public static Func<float3,float?> ConstructedSurfaceSampler(WaterPlan water,IReadOnlyList<GeographicPolygon> polygons)
    {
        if(water.ReachStages.Count==0)return SurfaceSampler(water,polygons);
        var reaches=water.ReachStages.Where(r=>r.PolygonId.HasValue).Concat(water.ReachStages.Select(r=>r with{
            HalfWidth=Math.Max(r.HalfWidth,water.WaterCellSize),PolygonId=null})).ToArray();
        return SurfaceSampler(water with{ReachStages=reaches},polygons);
    }
    // reference=true answers with the original single-grid scan and no
    // prefilter, so tests can prove the indexed sampler gives the same answers.
    internal static Func<float3,float?> SurfaceSampler(WaterPlan water,IReadOnlyList<GeographicPolygon> polygons,bool reference)
    {
        var masks=polygons.Where(p=>!Canal(p.Tags)).Where(p=>p.Tags.TryGetValue("natural",out var n)&&n=="water"||p.Tags.TryGetValue("waterway",out var w)&&w=="riverbank"||p.Tags.TryGetValue("landuse",out var l)&&l=="reservoir").Where(p=>p.Outer.Any(r=>r.Length>0))
            .Select((p,i)=>new Area(p,i,default)).ToArray();
        var maskIndex=new AreaIndex(masks);
        var flat=water.AreaStages.GroupBy(s=>s.BodyId).ToDictionary(g=>g.Key,g=>g.Min(s=>s.Level));
        // Reaches bound to a mapped polygon apply anywhere inside it, so they are
        // registered 1 km around their line in 2 km cells; free reaches apply only
        // within their half width and sit in 256 m cells. A point outside every
        // polygon then never scans the polygon-bound ones, and a free reach is
        // scanned only near its own line. Both lists keep ReachStages order and
        // are merged by that index, so ties resolve exactly as in one list.
        const float BoundCell=2048,FreeCell=256;
        var bound=new Dictionary<(int,int),List<(int Index,WaterReachStage Reach)>>();var free=new Dictionary<(int,int),List<(int Index,WaterReachStage Reach)>>();
        for(var i=0;i<water.ReachStages.Count;i++)
        {
            var r=water.ReachStages[i];var polygonBound=r.PolygonId.HasValue;var grid=polygonBound?bound:free;var size=polygonBound?BoundCell:FreeCell;
            var margin=Math.Max(r.HalfWidth,polygonBound?1024:1);var lo=math.min(r.Start.xz,r.End.xz)-margin;var hi=math.max(r.Start.xz,r.End.xz)+margin;
            for(var x=(int)Math.Floor(lo.x/size);x<=(int)Math.Floor(hi.x/size);x++)
                for(var z=(int)Math.Floor(lo.y/size);z<=(int)Math.Floor(hi.y/size);z++)
                {if(!grid.TryGetValue((x,z),out var list))grid[(x,z)]=list=new();list.Add((i,r));}
        }
        var none=new List<(int Index,WaterReachStage Reach)>();
        if(reference)
        {
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
        // Most queries are far from any water. The game's runtime makes the
        // per-query allocations below expensive (DC: 47.7 s in game against
        // 0.7 s offline for the same 207,000 queries), so a 128 m grid of cells
        // that can hold water answers "none" first. It is conservative: every
        // cell within one cell of a mask edge, inside a mask (by scanline at cell
        // centres), or within a free reach's half width is marked, so a point the
        // full test would find is never cut off.
        const float Cell=128;var occupied=new HashSet<long>();
        long Key(int x,int z)=>((long)x<<32)^(uint)z;
        void Mark(float2 a,float2 b,float margin)
        {
            var reach=(int)Math.Ceiling(margin/Cell)+1;var steps=Math.Max(1,(int)Math.Ceiling(math.distance(a,b)/(Cell/2)));
            for(var k=0;k<=steps;k++)
            {
                var q=math.lerp(a,b,k/(float)steps);var cx=(int)Math.Floor(q.x/Cell);var cz=(int)Math.Floor(q.y/Cell);
                for(var dz=-reach;dz<=reach;dz++)for(var dx=-reach;dx<=reach;dx++)occupied.Add(Key(cx+dx,cz+dz));
            }
        }
        foreach(var area in masks)foreach(var ring in area.Polygon.Outer)
        {
            for(var i=0;i<ring.Length;i++)Mark(ring[i].xz,ring[(i+1)%ring.Length].xz,0);
            var crossings=new List<float>();
            for(var cz=(int)Math.Floor(area.Min.y/Cell);cz<=(int)Math.Floor(area.Max.y/Cell);cz++)
            {
                var z=(cz+.5f)*Cell;crossings.Clear();
                for(var i=0;i<ring.Length;i++){var a=ring[i];var b=ring[(i+1)%ring.Length];if((a.z>z)!=(b.z>z))crossings.Add(a.x+(z-a.z)*(b.x-a.x)/(b.z-a.z));}
                crossings.Sort();
                for(var i=1;i<crossings.Count;i+=2)for(var cx=(int)Math.Floor(crossings[i-1]/Cell);cx<=(int)Math.Floor(crossings[i]/Cell);cx++)occupied.Add(Key(cx,cz));
            }
        }
        foreach(var r in water.ReachStages)if(!r.PolygonId.HasValue)Mark(r.Start.xz,r.End.xz,r.HalfWidth);
        return p=>
        {
            if(!occupied.Contains(Key((int)Math.Floor(p.x/Cell),(int)Math.Floor(p.z/Cell))))return null;
            Area? area=null;foreach(var a in maskIndex.Nearby(p))if(a.Contains(p)){area=a;break;}
            if(area!=null&&flat.TryGetValue(area.Polygon.Identity,out var level))return level;
            float? result=null;var best=float.PositiveInfinity;
            var polygonReaches=area!=null&&bound.TryGetValue(((int)Math.Floor(p.x/BoundCell),(int)Math.Floor(p.z/BoundCell)),out var b)?b:none;
            // Whether p is in an island hole depends only on p: tested once, not per reach.
            if(!free.TryGetValue(((int)Math.Floor(p.x/FreeCell),(int)Math.Floor(p.z/FreeCell)),out var lineReaches))lineReaches=none;
            else foreach(var a in maskIndex.Nearby(p))if(a.InHole(p)){lineReaches=none;break;}
            for(int i=0,j=0;i<polygonReaches.Count||j<lineReaches.Count;)
            {
                WaterReachStage r;
                if(j==lineReaches.Count||i<polygonReaches.Count&&polygonReaches[i].Index<lineReaches[j].Index){r=polygonReaches[i++].Reach;if(area!.Polygon.Identity!=r.PolygonId!.Value)continue;}
                else r=lineReaches[j++].Reach;
                var delta=r.End.xz-r.Start.xz;var t=math.clamp(math.dot(p.xz-r.Start.xz,delta)/Math.Max(.0001f,math.lengthsq(delta)),0,1);
                var distance=math.distance(p.xz,math.lerp(r.Start.xz,r.End.xz,t));
                if(!r.PolygonId.HasValue&&distance>r.HalfWidth||distance>=best)continue;
                best=distance;result=math.lerp(r.StartLevel,r.EndLevel,t);
            }
            return result;
        };
    }
    // A waterway culvert or tunnel shorter than this (metres) carries a river
    // under a road, embankment, interchange or rail yard; it stays in the river
    // network so the river is not cut in two there. At 60 m, streams passing
    // under motorway interchanges were cut into dry, disconnected reaches.
    // Longer ones are buried rivers (Marseille's Jarret runs covered for
    // kilometres, under the boulevard built on it) and must not become open
    // channels.
    public const float CulvertLength=250;
    private static readonly string[] Waterways={"river","stream","drain","ditch"};
    private static bool Canal(IReadOnlyDictionary<string,string> tags)=>
        tags.TryGetValue("waterway",out var way)&&way=="canal"||tags.TryGetValue("water",out var water)&&water=="canal";
    // A dry, abandoned channel keeps its waterway tag in OSM; Marseille's
    // abandoned branches of its canal were carved as trenches across hillsides.
    private static bool Abandoned(IReadOnlyDictionary<string,string> tags)=>
        tags.TryGetValue("abandoned",out var a)&&a=="yes"||tags.TryGetValue("disused",out var d)&&d=="yes";

    // One sample along a waterway's line: its place on the clipped edge A–B
    // (and the next sample on that edge, if any), its distance along the way,
    // the node it stands on (0 between nodes),
    // its dry bank probes with the lower bank's height, and the mapped
    // river polygon it lies in.
    private readonly record struct ChannelSample(float3 P,float3 A,float3 B,float3? Next,float Station,long Node,
        float Bank,IReadOnlyList<float3> Banks,Area? Area,float Reach,float Channel,float SourceWidth);

    // The waterways that make up the river network: rivers and streams,
    // and every ditch or drain connected to one through shared nodes, whatever
    // its width, so a river carried for a stretch by a narrow drain stays one
    // river. A ditch network touching none (field drainage) is left out unless
    // mapped at least a water cell (28 m) wide: alone it never holds a source
    // and was carved as a row of dry pits. Abandoned, disused and intermittent
    // waterways and canals are excluded; the culvert length rule is the caller's.
    private static HashSet<long> RiverNetwork(OsmDataset data)
    {
        var candidates=data.Ways.Values.Where(w=>w.Tags.TryGetValue("waterway",out var type)&&Waterways.Contains(type)&&!Canal(w.Tags)&&!Abandoned(w.Tags)&&
            !(w.Tags.TryGetValue("intermittent",out var intermittent)&&intermittent=="yes")&&w.NodeIds.Length>1&&w.NodeIds.All(data.Nodes.ContainsKey)).ToArray();
        var parent=candidates.ToDictionary(w=>w.Id,w=>w.Id);
        long Root(long id){while(parent[id]!=id)id=parent[id]=parent[parent[id]];return id;}
        var owner=new Dictionary<long,long>();
        foreach(var w in candidates)foreach(var node in w.NodeIds)
            if(owner.TryGetValue(node,out var other)){var a=Root(w.Id);var b=Root(other);if(a!=b)parent[a]=b;}else owner[node]=w.Id;
        var main=new HashSet<long>(candidates.Where(w=>w.Tags["waterway"] is "river" or "stream").Select(w=>Root(w.Id)));
        return new HashSet<long>(candidates.Where(w=>main.Contains(Root(w.Id))||NetworkEvidence.Measure(w.Tags,"width")>=28).Select(w=>w.Id));
    }

    // Short culverts as local polylines with their channel half width (metres):
    // the width tag, or 16 m for a river and 6 m otherwise, as for open reaches.
    public static IReadOnlyList<(float3[] Line,float HalfWidth)> ShortCulverts(OsmDataset? data,ImportRequest request)
    {
        if(data==null)return Array.Empty<(float3[],float)>();
        var result=new List<(float3[],float)>();var network=RiverNetwork(data);
        foreach(var w in data.Ways.Values.OrderBy(w=>w.Id))
        {
            if(!network.Contains(w.Id)||!w.Tags.TryGetValue("tunnel",out var tunnel)||tunnel!="yes"&&tunnel!="culvert")continue;
            var type=w.Tags["waterway"];
            var line=w.NodeIds.Select(id=>{var p=data.Nodes[id];return GeoMath.ToLocalMeters(p.Latitude,p.Longitude,request.Latitude,request.Longitude);}).ToArray();
            var length=0f;for(var i=1;i<line.Length;i++)length+=math.distance(line[i-1].xz,line[i].xz);
            if(length>=CulvertLength)continue;
            var width=NetworkEvidence.Measure(w.Tags,"width")??(type=="river"?32:12);
            result.Add((line,Math.Max(1,(float)width*.5f)));
        }
        return result;
    }

    // Open waterways of the river network as local polylines with their channel
    // half width, as for ShortCulverts. A ground road crossing one without a
    // bridge or culvert tag is an omission in the map, and its embankment would
    // dam the carved channel; CoordinatedNetworkDesign.CulvertBridges spans it.
    public static IReadOnlyList<(float3[] Line,float HalfWidth)> OpenChannels(OsmDataset? data,ImportRequest request)
    {
        if(data==null)return Array.Empty<(float3[],float)>();
        var result=new List<(float3[],float)>();var network=RiverNetwork(data);
        foreach(var w in data.Ways.Values.OrderBy(w=>w.Id))
        {
            if(!network.Contains(w.Id)||w.Tags.TryGetValue("tunnel",out var tunnel)&&tunnel!="no")continue;
            var type=w.Tags["waterway"];
            var line=w.NodeIds.Select(id=>{var p=data.Nodes[id];return GeoMath.ToLocalMeters(p.Latitude,p.Longitude,request.Latitude,request.Longitude);}).ToArray();
            var width=NetworkEvidence.Measure(w.Tags,"width")??(type=="river"?32:12);
            result.Add((line,Math.Max(1,(float)width*.5f)));
        }
        return result;
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
        var areas=polygons.Where(p=>!Canal(p.Tags)).Where(p=>Tag(p.Tags,"natural","water")||Tag(p.Tags,"waterway","riverbank")||Tag(p.Tags,"landuse","reservoir"))
            .Where(p=>p.Outer.Any(r=>r.Length>0)).Select((p,i)=>{Pulse();return new Area(p,i,token,()=>Pulse());}).ToArray();
        var areaIndex=new AreaIndex(areas,token);var holeIndex=new AreaIndex(areas.Where(a=>a.HasHoles),token);
        bool Coastal(Area a)=>Tag(a.Polygon.Tags,"water","sea")||Tag(a.Polygon.Tags,"water","ocean")||Tag(a.Polygon.Tags,"water","bay")||Tag(a.Polygon.Tags,"natural","bay")||Tag(a.Polygon.Tags,"tidal","yes");
        bool Flowing(Area a)=>Tag(a.Polygon.Tags,"water","river")||Tag(a.Polygon.Tags,"waterway","riverbank");
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
        // Cells on the line of a carved waterway channel. Shore shaping of mapped
        // water areas, which runs later, must not raise them back into a dam.
        var channelCells=new byte[n*n];
        void Cut(float3 center,float radius,float target,Func<float3,bool> mask,float budget,bool channel=false)
        {
            var pixel=Pixel(center);var x0=pixel%n;var z0=pixel/n;var reach=(int)Math.Ceiling(radius/spacing);
            for(var z=Math.Max(0,z0-reach);z<=Math.Min(n-1,z0+reach);z++)for(var x=Math.Max(0,x0-reach);x<=Math.Min(n-1,x0+reach);x++)
            {
                if((bedVisits++&4095)==0)Pulse();var p=new float3(x*spacing-half,0,z*spacing-half);var distance=math.distance(p.xz,center.xz);
                if(distance>radius)continue;var index=z*n+x;var original=terrain.Heightmap[index]/65535f*terrain.HeightScaleMeters;
                var blend=math.clamp((radius-distance)/(radius*.4f),0,1);blend=blend*blend*(3-2*blend);
                if(channel&&blend>.5f&&channelCells[index]==0&&mask(p))channelCells[index]=1;
                var height=Math.Max(0,original-Math.Min(budget,Math.Max(0,original-target))*blend);
                var encoded=(ushort)Math.Ceiling(math.clamp(height/terrain.HeightScaleMeters*65535,0,65535));
                // Overlapping cuts frequently propose the same/larger height.
                // Preserve the exact min-cut result while avoiding another
                // polygon query for a raster cell that cannot change.
                if(encoded>=conditioned[index])continue;
                if(mask(p)){conditioned[index]=encoded;if(editMask[index]==0){editMask[index]=1;editCount++;}}
            }
        }
        // Shore band: near the mapped edge, heights follow the exact distance to
        // it rather than which grid points fall inside. The bed deepens from the
        // stage at the edge to full depth, and the bank rises from the stage at
        // the edge back to the original land, each along a smoothstep over two
        // grid spacings, so slope is continuous through the edge and into the
        // untouched terrain. Land beyond the band is untouched; bank land is
        // lowered by at most bankCut and never below the levee, so it still
        // contains the water, and low bank land is raised to the levee.
        // Within one grid spacing of the edge both sides follow the same slope
        // EdgeSlope: the ground is stage + EdgeSlope·s for the signed distance
        // s, a plane along a straight shore, which the grid's bilinear
        // interpolation reproduces exactly in every cell the edge crosses, so
        // the waterline lies on the mapped edge whatever its angle to the grid.
        // A step at the edge (a lip 0.5 m up on the bank and 0.5 m down in the
        // bed) put each grid point on one side of the step or the other and
        // drew a slanted shore as teeth (East Potomac Park). The slope still
        // rises 0.8 m in half a 28 m water cell, so a dry cell next to the
        // shore stands clearly above the stage; flat profiles (zero slope at
        // the edge) let the water wet whole cells into a tooth per cell. Beyond
        // that spacing the levee and the bed follow a cubic to LeveeFreeboard
        // above, and the full depth below, the stage at the band's end, flat
        // there. Low bank land raised to the levee eases back down to its own
        // height (at least ShoreRim above the stage) across the band.
        var bedBand=2*spacing;var shoreBand=2*spacing;const float bankCut=3;var shapedBank=0;var raisedBank=0;
        const float LeveeFreeboard=1.2f,EdgeSlope=.06f,ShoreRim=.3f;
        // Shore-profile bed height of each wet pixel within the bed band (NaN elsewhere).
        var shoreFloor=new float[n*n];Array.Fill(shoreFloor,float.NaN);
        static float Ramp(float x){x=Math.Min(1,x);return x*x*(3-2*x);}
        // EdgeSlope·e up to one spacing, then a cubic Hermite to height at the
        // band's end with slope 0, kept monotonic by limiting its start slope.
        float Profile(float e,float height,float band)
        {
            if(e<=spacing)return EdgeSlope*e;
            var t=Math.Min(1,(e-spacing)/(band-spacing));var h0=EdgeSlope*spacing;var m=Math.Min(EdgeSlope*(band-spacing),3*Math.Max(0,height-h0));
            return h0*(2*t*t*t-3*t*t+1)+m*(t*t*t-2*t*t+t)+height*(3*t*t-2*t*t*t);
        }
        void CarveArea(Area area,Func<float3,float?> level)
        {
            var accepted=0;var blockedByBudget=0;var pad=(int)Math.Ceiling(shoreBand/spacing);
            var loX=Math.Max(0,(int)Math.Ceiling((area.Min.x+half)/spacing)-pad);var hiX=Math.Min(n-1,(int)Math.Floor((area.Max.x+half)/spacing)+pad);
            var loZ=Math.Max(0,(int)Math.Ceiling((area.Min.y+half)/spacing)-pad);var hiZ=Math.Min(n-1,(int)Math.Floor((area.Max.y+half)/spacing)+pad);
            for(var z=loZ;z<=hiZ;z++)for(var x=loX;x<=hiX;x++)
            {
                if((bedVisits++&4095)==0)Pulse();var p=new float3(x*spacing-half,0,z*spacing-half);
                var edge=area.EdgeDistance(p,shoreBand,out var closest);
                if(!area.Contains(p)||MappedDry(p))
                {
                    if(float.IsPositiveInfinity(edge)||channelCells[z*n+x]!=0)continue;
                    // The bank meets the stage of the water just across the edge.
                    var inward=math.normalizesafe(closest-p.xz)*.5f;var shore=level(new float3(closest.x+inward.x,0,closest.y+inward.y));
                    if(!shore.HasValue||!math.isfinite(shore.Value))continue;
                    // Low banks: source noise leaves pixels of a low shore (East
                    // Potomac Park) under the stage, and water filled them one by
                    // one into a spiky edge. Bank land up to bankCut below the
                    // stage rises to the lip, unless it lies in other mapped
                    // water, where the levee would dam that body. Higher bank land
                    // is lowered toward the stage but never below the lip.
                    var levee=shore.Value+Profile(edge,LeveeFreeboard,shoreBand);
                    var bankIndex=z*n+x;var land=terrain.Heightmap[bankIndex]/65535f*terrain.HeightScaleMeters;
                    var bank=Math.Max(levee,shore.Value+(land-shore.Value)*Ramp(edge/shoreBand));
                    if(land<levee&&land>shore.Value-bankCut&&!areaIndex.Nearby(p).Any(a=>a.Contains(p)))
                    {
                        var rim=math.lerp(levee,Math.Max(land,shore.Value+ShoreRim),Ramp(Math.Max(0,edge-spacing)/(shoreBand-spacing)));
                        var raised=(ushort)Math.Ceiling(math.clamp(rim/terrain.HeightScaleMeters*65535,0,65535));
                        if(raised>conditioned[bankIndex]){conditioned[bankIndex]=raised;raisedBank++;if(editMask[bankIndex]==0){editMask[bankIndex]=1;editCount++;}}
                        continue;
                    }
                    if(land<=bank)continue;
                    var cut=(ushort)Math.Ceiling(math.clamp(Math.Max(land-bankCut,bank)/terrain.HeightScaleMeters*65535,0,65535));
                    if(cut>=conditioned[bankIndex])continue;
                    conditioned[bankIndex]=cut;shapedBank++;if(editMask[bankIndex]==0){editMask[bankIndex]=1;editCount++;}
                    continue;
                }
                var stage=level(p);if(!stage.HasValue||!math.isfinite(stage.Value))continue;
                // An edge shared with other mapped water (a river drawn in
                // pieces) is not a shore: the bed runs through it at full depth.
                if(edge<bedBand){var outward=math.normalizesafe(closest-p.xz);var across=new float3(closest.x+outward.x,0,closest.y+outward.y);
                    if(areaIndex.Nearby(across).Any(a=>a!=area&&a.Contains(across)))edge=bedBand;}
                // A flowing polygon is a metre deeper than a lake (rivers read as cut channels).
                var depth=Profile(edge,Lake(area)?2:3,bedBand);
                var index=z*n+x;var original=terrain.Heightmap[index]/65535f*terrain.HeightScaleMeters;
                var height=Math.Max(0,Lake(area)?Math.Max(original-8,Math.Min(original,stage.Value-depth)):Math.Min(original,stage.Value-depth));
                var encoded=(ushort)Math.Ceiling(math.clamp(height/terrain.HeightScaleMeters*65535,0,65535));
                // Within the band the bed is later raised to the shore profile
                // itself where the source lies deeper, so a deep channel beside
                // its bank does not reopen the step at the edge.
                if(edge<bedBand){var floor=Math.Max(0,stage.Value-depth);shoreFloor[index]=float.IsNaN(shoreFloor[index])?floor:Math.Max(shoreFloor[index],floor);}
                // The shore profile is shallow by design within a few metres of the edge.
                accepted++;if(depth>=.1f&&height>=stage.Value-.1f)blockedByBudget++;
                // The accepted wet interior owns its bed even if it was already
                // below target. Later earthworks must not introduce a new dam.
                if(!Lake(area)&&editMask[index]==0){editMask[index]=1;editCount++;}
                if(encoded>=conditioned[index])continue;conditioned[index]=encoded;if(editMask[index]==0){editMask[index]=1;editCount++;}
            }
            log.Diagnostic($"WATER AREA BED body={area.Polygon.Identity}; cellsVisitedInside={accepted}; bedAboveStageAfterBudget={blockedByBudget}; continuousMask=true; holesPreserved=true; lakeBudgetMeters=8; bedCutsConfinedToWetMask=true; shoreBandMeters={shoreBand:0.#}; bankShapingOnlyLowersWithin={bankCut}m");
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
        // A salt lagoon on a coast (water=lagoon, salt=yes) is open to the sea,
        // so its water stands no lower than the sea's surface; its low shores
        // flood as the sea's own would, so they are not bank probes. Planned
        // from its lowest shore probe, the Étang de Berre (water=pond, salt=yes)
        // stood 12 m below the sea, and so did every river draining into it.
        var seaSurface=WaterLevels.Effective(terrain.SeaLevelMeters);var coastalMap=data.Ways.Values.Any(w=>Tag(w.Tags,"natural","coastline"));
        bool Lagoon(Area a)=>coastalMap&&(Tag(a.Polygon.Tags,"water","lagoon")||Tag(a.Polygon.Tags,"salt","yes"));
        foreach(var area in areas.Where(Lake))
        {
            token.ThrowIfCancellationRequested();lakeCount++;var polygon=area.Polygon;activeBody=polygon.Id;Pulse();
            var banks=polygon.Outer.SelectMany(r=>r.Where((p,i)=>i%Math.Max(1,r.Length/64)==0)).Where(Inside)
                .Select(p=>Enumerable.Range(0,8).Select(i=>DryProbe(p,new float3(math.cos(i*math.PI/4),0,math.sin(i*math.PI/4)),area.Contains,cell*2)).FirstOrDefault(q=>q.HasValue))
                .Where(p=>p.HasValue).Select(p=>p!.Value).ToArray();
            if(banks.Length<3){unresolved++;continue;}
            var sorted=banks.Select(Height).OrderBy(h=>h).ToArray();var stage=sorted[0]-.5f;
            if(Lagoon(area)&&stage<seaSurface){stage=seaSurface;sorted[0]=stage+.5f;banks=Array.Empty<float3>();log.Count("Water lagoons held at the sea's surface");}
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
        float Length(OsmWay w){var length=0f;for(var i=1;i<w.NodeIds.Length;i++)length+=math.distance(Local(w.NodeIds[i-1]).xz,Local(w.NodeIds[i]).xz);return length;}
        var network=RiverNetwork(data);
        var lines=data.Ways.Values.Where(w=>network.Contains(w.Id)&&
                (!Tag(w.Tags,"tunnel","yes")&&!Tag(w.Tags,"tunnel","culvert")||Length(w)<CulvertLength)).OrderBy(w=>w.Id).ToArray();
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
        // Sources of one body stand at least spacing metres apart: 192 m, or 384 m
        // along a waterway narrower than two water cells without a polygon, whose
        // water flows on to the next one (DC's streams otherwise took 9,600).
        bool NearControl(long body,float3 p,float spacing=192)
        {var x=(int)Math.Floor(p.x/192);var z=(int)Math.Floor(p.z/192);var r=(int)Math.Ceiling(spacing/192);for(var dx=-r;dx<=r;dx++)for(var dz=-r;dz<=r;dz++)if(controlGrid.TryGetValue((x+dx,z+dz),out var list)&&list.Any(s=>s.BodyId==body&&math.distance(s.Position.xz,p.xz)<spacing))return true;return false;}
        phase="conditioning river beds and placing controls";Pulse(true);
        // A channel without a mapped river polygon may be cut this far into the
        // source ground. Narrow urban rivers (Marseille's Huveaune) sit in DEM
        // cells that average in their banks, so 8 m skipped whole stretches,
        // which then had neither bed nor water source and broke the river up;
        // 12 m still skipped 3,097 Marseille samples.
        // A river or stream without a polygon is carved at least one water
        // simulation cell either side of its line, ChannelDepth below its stage,
        // with banks sloping off over the outer 40% of the cut. Cut to its 6 m
        // half width, a stream was a row of pits narrower than the game's 28 m
        // water cells, and the water could not flow along it.
        // Each sample's stage stands RiverFreeboard below its lower bank: at
        // the solve's .35 m, and 3.5 m of channel, the water ran level with the
        // land and Marseille's rivers read as shallow paint rather than a cut.
        // Every sample is carved, including one with no dry bank found on both
        // sides (a map edge, a wide polygon) or one too far below ground: such a
        // sample only gets no water source. Skipping it left a dry plug that
        // split the river into disconnected pools.
        const float ChannelCut=20,ChannelDepth=5,RiverFreeboard=1;var placedSamples=0;var skippedTooDeep=0;var skippedAboveBank=0;var skippedNoBanks=0;
        // Each waterway gets one water surface before anything is cut: its
        // samples are collected first, then solved, then carved.
        //  - Each sample may stand RiverFreeboard below its lower bank. A dip
        //    in that bound narrower than StageBridge and at most LeveeMax deep
        //    is bridged instead of followed: the water keeps its level and a
        //    levee is raised beside the channel there. Following each dip
        //    (the node solve's interpolated level did, along a whole edge)
        //    dropped every later sample with it and cut trenches downstream:
        //    Marseille's median cut was 7 m, 3.2 m without it.
        //  - The surface never rises downstream: a running minimum along the
        //    way, starting from the lowest surface of the ways ending at its
        //    first node and capped at every node where a tributary ended at its
        //    own surface. Ways run in the order of their last node in the stage
        //    solve, so tributaries and upstream ways come first. Per-sample
        //    stages before rose and fell with each bank, and a source held
        //    higher downstream pushed its water back upstream over lower banks.
        //  - A waterway that runs to the sea never stands below the sea's
        //    surface: the Huveaune's mouth was planned 8 m below it, so the sea
        //    would have run up the channel.
        // The bed never rises downstream either: each cut reaches no higher than
        // the lowest bed cut upstream on the way, carried on through way ends.
        // Cut to stage − ChannelDepth alone, a quarter of Marseille's open
        // waterway length stood behind a dam. Only the channel's own target is
        // carried, never a deeper lake bed it passes through.
        // A canal runs level along a hillside on a bench with an embankment on
        // the downhill side: its bound is the higher of that bank and the ground
        // on its line, and wider, higher dips are bridged. Bounded by its
        // downhill bank alone, the Canal de Marseille sank into trenches more
        // than 15 m deep along 1,950 of its reaches.
        const float StageBridge=150,LeveeMax=3,CanalBridge=400,CanalLeveeMax=6;
        var coastline=new Dictionary<(int,int),List<(float3 A,float3 B)>>();
        foreach(var w in data.Ways.Values.Where(w=>Tag(w.Tags,"natural","coastline")))
            for(var i=1;i<w.NodeIds.Length;i++)
            {
                if(!data.Nodes.ContainsKey(w.NodeIds[i-1])||!data.Nodes.ContainsKey(w.NodeIds[i]))continue;
                var a=Local(w.NodeIds[i-1]);var b=Local(w.NodeIds[i]);var key=((int)Math.Floor((a.x+b.x)/2/512),(int)Math.Floor((a.z+b.z)/2/512));
                if(math.distance(a.xz,b.xz)>1024)continue;if(!coastline.TryGetValue(key,out var list))coastline[key]=list=new();list.Add((a,b));
            }
        // A way's end reaches the sea when it lies in mapped sea, within 150 m of a lagoon, or within
        // 150 m of the coastline.
        bool AtSea(float3 p)
        {
            if(areaIndex.Nearby(p).Any(a=>Coastal(a)&&a.Contains(p)||Lagoon(a)&&(a.Contains(p)||a.EdgeDistance(p,150,out _)<=150)))return true;
            var cx=(int)Math.Floor(p.x/512);var cz=(int)Math.Floor(p.z/512);
            for(var dx=-2;dx<=2;dx++)for(var dz=-2;dz<=2;dz++)if(coastline.TryGetValue((cx+dx,cz+dz),out var list)&&list.Any(s=>NetworkNormalizer.DistanceToSegment(p,s.A,s.B)<=150))return true;
            return false;
        }
        var upstreamOf=positions.Keys.ToDictionary(id=>id,id=>new List<long>());
        foreach(var link in outgoing)foreach(var b in link.Value)upstreamOf[b].Add(link.Key);
        var toSea=new HashSet<long>();var seaQueue=new List<long>();
        foreach(var id in positions.Keys)if(outgoing[id].Count==0&&AtSea(positions[id])&&toSea.Add(id))seaQueue.Add(id);
        for(var q=0;q<seaQueue.Count;q++)foreach(var u in upstreamOf[seaQueue[q]])if(toSea.Add(u))seaQueue.Add(u);
        var upstreamOrder=new Dictionary<long,int>();for(var k=0;k<ready.Count;k++)upstreamOrder[ready[k]]=k;
        var nodeSurface=new Dictionary<long,float>();var nodeBed=new Dictionary<long,float>();
        var levees=new List<(float3 A,float3 B,float3 P,float Channel,float Height)>();var bridgedSamples=0;var seaRaisedSamples=0;
        foreach(var way in lines.OrderBy(w=>upstreamOrder.TryGetValue(w.NodeIds[w.NodeIds.Length-1],out var k)?k:int.MaxValue).ThenBy(w=>w.Id))
        {
            token.ThrowIfCancellationRequested();riverCount++;var body=bodyIds[Root(way.Id)];activeBody=body;Pulse();
            var bedLimit=nodeBed.TryGetValue(way.NodeIds[0],out var carried)?carried:float.PositiveInfinity;
            if(unsupportedBodies.Contains(body))log.Count("Water bodies using local feasible reaches");
            var sourceWidth=NetworkEvidence.Measure(way.Tags,"width")??(Tag(way.Tags,"waterway","river")?32:12);
            if(!math.isfinite(sourceWidth)||sourceWidth<=0||sourceWidth>2048){log.Count("Water width repaired");sourceWidth=Tag(way.Tags,"waterway","river")?32:12;}
            var reach=sourceWidth*.5f;
            // Every waterway of the network is carved at least a water cell
            // either side of its line: a ditch or a canal cut to its own 6 m
            // half width on a 14 m grid read as a row of square pits (Cairo's
            // canals) that the water could not flow along.
            var channel=Math.Max(reach,cell);
            var samples=new List<ChannelSample>();var station=0f;
            for(var i=1;i<way.NodeIds.Length;i++)
            {
                var a=positions[way.NodeIds[i-1]];var b=positions[way.NodeIds[i]];if(!WaterPlanner.Clip(a,b,half-cell*2,out var lo,out var hi))continue;
                var originalA=a;var originalB=b;a=math.lerp(originalA,originalB,lo);b=originalA+(originalB-originalA)*hi;
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
                    var node=j==0&&lo<=0?way.NodeIds[i-1]:j==steps&&hi>=1?way.NodeIds[i]:0;
                    samples.Add(new(p,a,b,j<steps?math.lerp(a,b,(j+1)/(float)steps):null,station+length*j/steps,node,
                        banks.Count<2?BankHeight(p):banks.Min(BankHeight),banks,area,reach,channel,sourceWidth));
                }
                station+=length;
            }
            if(samples.Count==0)continue;
            // The surface (bridged bound, running minimum, sea floor).
            var canal=Tag(way.Tags,"waterway","canal");var bridge=canal?CanalBridge:StageBridge;var leveeMax=canal?CanalLeveeMax:LeveeMax;
            var count=samples.Count;var bound=samples.Select(s=>(canal?Math.Max(s.Bank,BankHeight(s.P)):s.Bank)-RiverFreeboard).ToArray();
            var dilated=new float[count];var surface=new float[count];
            for(int k=0,lo=0,hi=0;k<count;k++)
            {
                while(samples[k].Station-samples[lo].Station>bridge/2)lo++;
                while(hi+1<count&&samples[hi+1].Station-samples[k].Station<=bridge/2)hi++;
                var value=float.NegativeInfinity;for(var q=lo;q<=hi;q++)value=Math.Max(value,bound[q]);dilated[k]=value;
            }
            for(int k=0,lo=0,hi=0;k<count;k++)
            {
                while(samples[k].Station-samples[lo].Station>bridge/2)lo++;
                while(hi+1<count&&samples[hi+1].Station-samples[k].Station<=bridge/2)hi++;
                var value=float.PositiveInfinity;for(var q=lo;q<=hi;q++)value=Math.Min(value,dilated[q]);
                surface[k]=Math.Min(value,bound[k]+leveeMax);
                if(samples[k].Node!=0&&nodeSurface.TryGetValue(samples[k].Node,out var arriving))surface[k]=Math.Min(surface[k],arriving);
                if(k>0)surface[k]=Math.Min(surface[k],surface[k-1]);
            }
            var seaBound=toSea.Contains(way.NodeIds[way.NodeIds.Length-1]);
            for(var k=0;k<count;k++)
            {
                if(seaBound&&surface[k]<seaSurface){surface[k]=seaSurface;seaRaisedSamples++;}
                if(surface[k]>bound[k]+.01f)bridgedSamples++;
            }
            for(var k=0;k<count;k++)
            {
                var sample=samples[k];var p=sample.P;var a=sample.A;var b=sample.B;var area=sample.Area;var stage=surface[k];var banks=sample.Banks;var bankless=banks.Count<2;
                Func<float3,bool> mask=area!=null?q=>area.Contains(q)&&!MappedDry(q):q=>!MappedDry(q)&&NetworkNormalizer.DistanceToSegment(q,a,b)<=reach;
                // The carved channel and its sloping banks (ChannelDepth note).
                // A mapped river polygon narrower than that channel gets the
                // channel too: the Huveaune's, under 20 m wide on a 14 m
                // grid, kept only the few grid points inside it, and its bed
                // stood near ground level between them.
                Func<float3,bool> channelMask=area!=null?q=>mask(q)||!MappedDry(q)&&NetworkNormalizer.DistanceToSegment(q,a,b)<=channel/.6f:q=>!MappedDry(q)&&NetworkNormalizer.DistanceToSegment(q,a,b)<=channel/.6f;
                var floor=area!=null?.4f:Math.Max(.4f,Height(p)-ChannelCut+.4f);
                if(stage<floor)
                {
                    unresolved++;skippedTooDeep++;log.Count("Water locally infeasible reaches");
                    var target=Math.Min(stage-ChannelDepth,bedLimit);Cut(p,channel/.6f,target,channelMask,area!=null?float.PositiveInfinity:ChannelCut,true);bedLimit=Math.Max(Height(p),target);continue;
                }
                // A bridged dip, or a sea-level floor above low land, gets a
                // levee RiverFreeboard above the water; its crest stands in
                // for the bank probes.
                var bank=sample.Bank;
                if(!bankless&&stage>bank-RiverFreeboard)
                {
                    bank=stage+RiverFreeboard;levees.Add((a,b,p,channel,bank));
                    var across=math.normalizesafe(new float3(-(b-a).z,0,(b-a).x))*(channel/.6f+cell*.5f);banks=new[]{p+across,p-across};bankless=false;
                }
                placedSamples++;
                var stagePixel=Pixel(p);riverSurface[stagePixel]=riverSurface[stagePixel]>0?Math.Min(riverSurface[stagePixel],stage):stage;
                // Publish stage evidence even when a native source is subgrid.
                if(sample.Next is float3 next)
                {
                    var nextLevel=k+1<count?Math.Min(stage,surface[k+1]):stage;
                    var candidate=new WaterReachStage(body,way.Id,p,next,stage,nextLevel,reach,area?.Polygon.Identity);
                    var last=reachStages.Count>0?reachStages[reachStages.Count-1]:null;
                    if(last!=null&&last.WayId==candidate.WayId&&last.PolygonId==candidate.PolygonId&&math.distance(last.End.xz,p.xz)<.01f&&
                        math.dot(math.normalizesafe((last.End-last.Start).xz),math.normalizesafe((next-p).xz))>.99999f&&
                        Math.Abs((last.EndLevel-last.StartLevel)/Math.Max(.1f,math.distance(last.Start.xz,last.End.xz))-(nextLevel-stage)/Math.Max(.1f,math.distance(p.xz,next.xz)))<.0001f)
                        reachStages[reachStages.Count-1]=last with{End=next,EndLevel=nextLevel};
                    else reachStages.Add(candidate);
                }
                // When a river polygon supplies actual banks, condition that
                // mapped wet cross-section instead of carving a 32m ribbon
                // down the middle of a several-hundred-metre-wide river.
                var wetRadius=area!=null&&!bankless?Math.Min(512,banks.Min(q=>math.distance(q.xz,p.xz)))*.95f:channel;
                var bedTarget=Math.Min(stage-ChannelDepth,bedLimit);Cut(p,Math.Max(wetRadius,channel)/.6f,bedTarget,channelMask,area!=null?float.PositiveInfinity:ChannelCut,true);bedLimit=Math.Max(Height(p),bedTarget);
                if(bankless){unresolved++;skippedNoBanks++;continue;}
                if(NearControl(body,p,area==null&&sample.SourceWidth<cell*2?384:192))continue;
                // A source's footprint is the carved channel's flat bed, not the
                // mapped width: a stream's 6 m half width held no source at all,
                // so every stream stayed a dry trench.
                Func<float3,bool> footprint=area!=null?mask:q=>!MappedDry(q)&&NetworkNormalizer.DistanceToSegment(q,a,b)<=channel;
                var radius=area!=null?Math.Max(cell,Math.Min(192,wetRadius*.55f)):Math.Max(2,Math.Min(cell,channel*.7f));
                if(Add(way.Id,body,"river",way.Tags.TryGetValue("name",out var name)?name:"waterway",p,radius,stage,bank,banks,new[]{p,math.lerp(p,b,.5f),b},footprint))
                {var key=((int)Math.Floor(p.x/192),(int)Math.Floor(p.z/192));if(!controlGrid.TryGetValue(key,out var list))controlGrid[key]=list=new();list.Add(sources[sources.Count-1]);}
            }
            var end=way.NodeIds[way.NodeIds.Length-1];
            nodeSurface[end]=nodeSurface.TryGetValue(end,out var arrivingSurface)?Math.Min(arrivingSurface,surface[count-1]):surface[count-1];
            if(math.isfinite(bedLimit))nodeBed[end]=nodeBed.TryGetValue(end,out var downstream)?Math.Min(downstream,bedLimit):bedLimit;
        }
        // Levees go up after every channel is cut, never across a channel or
        // into mapped water: full height from the channel's cut out to a water
        // cell beyond it, then easing back to the ground over two more cells.
        var leveeCells=0;
        foreach(var (a,b,p,channel,height) in levees)
        {
            var crest=channel/.6f+cell;var outer=crest+2*cell;var pixel=Pixel(p);var span=(int)Math.Ceiling(outer/spacing);
            for(var z=Math.Max(0,pixel/n-span);z<=Math.Min(n-1,pixel/n+span);z++)for(var x=Math.Max(0,pixel%n-span);x<=Math.Min(n-1,pixel%n+span);x++)
            {
                var index=z*n+x;if(channelCells[index]!=0)continue;var q=new float3(x*spacing-half,0,z*spacing-half);
                var d=NetworkNormalizer.DistanceToSegment(q,a,b);if(d<channel||d>outer||math.distance(q.xz,p.xz)>outer)continue;
                if(areaIndex.Nearby(q).Any(r=>!Dry(r)&&r.Contains(q)))continue;
                var current=conditioned[index]/65535f*terrain.HeightScaleMeters;var t=Ramp(Math.Max(0,d-crest)/(outer-crest));
                var target=(ushort)Math.Ceiling(math.clamp(math.lerp(height,current,t)/terrain.HeightScaleMeters*65535,0,65535));
                if(target<=conditioned[index])continue;conditioned[index]=target;leveeCells++;if(editMask[index]==0){editMask[index]=1;editCount++;}
            }
        }
        log.Count("Water surface samples bridging a bank dip",bridgedSamples);log.Count("Water surface samples held at sea level",seaRaisedSamples);log.Count("Water levee cells raised",leveeCells);
        log.Diagnostic($"WATER SURFACE PROFILE monotoneDownstream=true; bridgeMeters={StageBridge}; leveeMaxMeters={LeveeMax}; bridgedSamples={bridgedSamples}; seaBoundNodes={toSea.Count}; seaSurface={seaSurface:R}; seaRaisedSamples={seaRaisedSamples}; leveeCells={leveeCells}");
        log.Evidence($"Rivers: {riverCount:N0} waterways, {placedSamples:N0} channel samples accepted, {sources.Count:N0} water sources; carved without a source: {skippedTooDeep:N0} more than {ChannelCut:0} m below ground, {skippedNoBanks:N0} without two banks; skipped: {skippedAboveBank:N0} above their banks.",
            warning:skippedTooDeep+skippedAboveBank>placedSamples/10);
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
            // A diagonal raster contact is still a mapped channel contact.
            if(x>0&&z>0)Visit(i-n-1);if(x+1<n&&z>0)Visit(i-n+1);
            if(x>0&&z+1<n)Visit(i+n-1);if(x+1<n&&z+1<n)Visit(i+n+1);
        }
        // Seed disconnected raster fragments from their SAME geographic river
        // polygon's solved reach. Do not jump across islands or use another river.
        var reachesByPolygon=reachStages.Where(r=>r.PolygonId.HasValue).GroupBy(r=>r.PolygonId!.Value).ToDictionary(g=>g.Key,g=>g.ToArray());
        var additionalSeeds=0;
        for(var i=0;i<wet.Length;i++)
        {
            if((i&16383)==0)Pulse();if(wet[i]==0||riverSurface[i]>0)continue;
            var p=new float3(i%n*spacing-half,0,i/n*spacing-half);
            var area=areaIndex.Nearby(p).FirstOrDefault(a=>Flowing(a)&&!Coastal(a)&&!Dry(a)&&a.Contains(p));
            if(area==null||!reachesByPolygon.TryGetValue(area.Polygon.Identity,out var reaches))continue;
            var best=float.PositiveInfinity;var level=0f;
            foreach(var reach in reaches)
            {
                var d=reach.End.xz-reach.Start.xz;var t=math.clamp(math.dot(p.xz-reach.Start.xz,d)/Math.Max(1,math.lengthsq(d)),0,1);
                var distance=math.distancesq(p.xz,math.lerp(reach.Start.xz,reach.End.xz,t));if(distance>=best)continue;
                best=distance;level=math.lerp(reach.StartLevel,reach.EndLevel,t);
            }
            if(level<=0)continue;riverSurface[i]=level;riverQueue[tail++]=i;additionalSeeds++;
            while(head<tail)
            {
                var current=riverQueue[head++];var x=current%n;var z=current/n;
                void Visit(int next){if(next<0||wet[next]==0||riverSurface[next]>0)return;riverSurface[next]=riverSurface[current];riverQueue[tail++]=next;}
                Visit(x>0?current-1:-1);Visit(x+1<n?current+1:-1);Visit(z>0?current-n:-1);Visit(z+1<n?current+n:-1);
            }
        }
        log.Count("River raster components reseeded",additionalSeeds);
        foreach(var area in areas.Where(a=>Flowing(a)&&!Coastal(a)&&!Dry(a)))CarveArea(area,p=>riverSurface[Pixel(p)]>0?riverSurface[Pixel(p)]:(float?)null);
        log.Diagnostic($"WATER CHANNEL CONTINUITY mappedWetCells={wet.Count(v=>v!=0)}; solvedWetCells={tail}; disconnectedUnseededCells={wet.Count(v=>v!=0)-tail}; islandsPreserved=true; sourceLevelPropagation=nearestConnectedRasterSeed; quadraticReachSearch=false");
        log.Count("Water shore bank samples shaped",shapedBank);
        log.Diagnostic($"WATER BANKFULL PROFILE targetBankFreeboardMeters=.35; bankTerrainShapedOnlyInShoreBand=true; shoreBankSamplesShaped={shapedBank}; nonCumulativeSampleBounds=true; initialCenterlineOffsetMeters=2.8; fullWidthDepthMeters=2.5; widthAwareSourceRadius=true; mappedRiverBedBudget=maskOwned; finalNodes={nodeStage.Count}; minimumFinalStage={nodeStage.Values.DefaultIfEmpty(0).Min():R}; maximumFinalStage={nodeStage.Values.DefaultIfEmpty(0).Max():R}");
        // River cross-sections were already stamped at <= half DEM spacing or
        // half mapped width. Do not perform a second polygon-wide nearest-reach
        // search per raster cell: it can turn dense hydrology into quadratic work.
        log.Diagnostic($"WATER RIVER OFFSET requestedMeters=1; controls={sources.Count(s=>s.BodyKind=="river")}; appliedOnceBeforeSafetyChecks=true; lakeAndSeaOffsets=0; bankLimitedNodeUpdates={bankLimitedNodes}; fullOffsetNotGuaranteed=true; bankConflictsRejectControls=true");
        // Control hollows are clipped by the wet mask, which recreates a one-pixel
        // cliff at the shore, and a source DEM deeper than the profile near its
        // bank leaves one too. Within the bed band, the bed is raised to the
        // shore profile, except on the bilinear neighbourhood of each control,
        // whose planned depth must hold.
        var controlCells=new HashSet<int>();
        foreach(var s in sources){var c=Pixel(s.Position);for(var dz=-1;dz<=1;dz++)for(var dx=-1;dx<=1;dx++){var x=c%n+dx;var z=c/n+dz;if(x>=0&&x<n&&z>=0&&z<n)controlCells.Add(z*n+x);}}
        var easedShore=0;
        for(var i=0;i<shoreFloor.Length;i++)
        {
            if(float.IsNaN(shoreFloor[i])||controlCells.Contains(i)||channelCells[i]!=0)continue;
            var floor=(ushort)Math.Ceiling(math.clamp(shoreFloor[i]/terrain.HeightScaleMeters*65535,0,65535));
            if(floor>conditioned[i]){conditioned[i]=floor;easedShore++;}
        }
        log.Count("Water shore bed samples eased",easedShore);
        phase="smoothing newly carved banks";Pulse(true);
        var smoothingPlan=new WaterPlan(false,riverCount,lakeCount,unresolved,"")
            {Sources=sources,AreaStages=areaStages,ReachStages=reachStages,WaterCellSize=cell};
        var cliffs=data.Ways.Values.Where(w=>Tag(w.Tags,"natural","cliff"))
            .Select(w=>w.NodeIds.Where(data.Nodes.ContainsKey).Select(Local).ToArray()).Where(p=>p.Length>1).ToArray();
        WaterCutSmoothing.Apply(terrain.Heightmap,conditioned,editMask,channelCells,controlCells,terrain.HeightScaleMeters,width,
            ConstructedSurfaceSampler(smoothingPlan,polygons),cliffs,log,token,()=>Pulse());
        // A bank probe that this planning lowered (into a carved channel such
        // as a lake's outlet stream, another body's bed, or a shaped shore) or
        // that lies inside other mapped water reads planned water, not the
        // ground that holds this body in; water reaching it stays in a planned
        // channel. Revalidate then judges only ground that later earthworks
        // lowered. Marseille's and DC's lakes lost their controls to probes in
        // the trench of a stream running past or out of them. A control left
        // with no probe keeps its planning bank level.
        bool PlannedWater(float3 q)=>Height(q)<BankHeight(q)-.25f||areaIndex.Nearby(q).Any(a=>!Dry(a)&&a.Contains(q));
        for(var i=0;i<sources.Count;i++)
        {
            var kept=sources[i].BankProbes.Where(q=>!PlannedWater(q)).ToArray();
            if(kept.Length<sources[i].BankProbes.Count){sources[i]=sources[i] with{BankProbes=kept};log.Count("Water controls with bank probes on planned water");}
        }
        log.Diagnostic($"WATER SHORE PROFILE bedBandMeters={bedBand:0.#}; bankBandMeters={shoreBand:0.#}; ramp=smoothstep; bankSamplesShaped={shapedBank}; lowBankSamplesRaised={raisedBank};bedSamplesEasedFromControlHollows={easedShore}; controlNeighbourhoodsKept={controlCells.Count}");
        TerrainArtifactRepair.TraceEncoded(conditioned, terrain.HeightScaleMeters, terrain.SeaLevelMeters, "after-water-bed-planning", log.Diagnostic);
        var coastal=data.Ways.Values.Any(w=>Tag(w.Tags,"natural","coastline"));
        var report=$"Water: {sources.Count} bank-limited river/lake controls; {lakeCount} mapped lakes; {unresolved} unresolved narrow/height sites. No flooding bypass. Coverage requires simulation verification.";
        log.Info(report);log.Diagnostic($"WATER BODY SUMMARY bodies={sources.Select(s=>s.BodyId).Distinct().Count()}; controls={sources.Count}; bedEdits={editCount}; cellMeters={cell:R}; mappedRiverBankWidthUsed=true; polygonBedCutsMaskOwned=true; fixedDepthFallback=false; floodingAccepted=false; unresolved={unresolved}; indexedPolygonPredicates=true; bedCellsVisited={bedVisits}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.###}; canonicalAreas={areaStages.Count}; canonicalReaches={reachStages.Count}; maskBytes={editMask.Length}; geometryVersion={WaterPlan.GeometryVersion}");
        var plan=new WaterPlan(coastal,riverCount,lakeCount,unresolved,report){Sources=sources,ConditionedHeightmap=editCount>0?conditioned:null,PlannedBedHeightmap=conditioned,WaterCellSize=cell,AreaStages=areaStages,ReachStages=reachStages,BedEdits=new TerrainEditSet("water-bed","pre-water-terrain",n,editMask)};
        return WaterContainment.Validate(plan,terrain with{Heightmap=conditioned},width,polygons,log,token);
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
        foreach(var s in (networks??Array.Empty<PreparedSegment>()).Where(s=>!s.Bridge&&!s.Tunnel))
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
        // A bank probe may stand within .25 m of the stage on the planned terrain
        // itself, where the shore was shaped for that stage (a levee rising from
        // the waterline). Only a bank that later earthworks lowered, or one
        // never above the stage, is a conflict. Such a control then targets
        // .35 m below that bank when that lowers it by at most StageDrop and
        // still leaves water over its bed: rejected outright, a lake whose
        // controls all share one low shore probe stayed a dry carved pit
        // (93 of Marseille's 311 sourced lakes). The canonical stage is kept
        // for bridge clearance, which a lower surface only makes safer.
        const float StageDrop=1;
        var plannedBanks=plan.PlannedBedHeightmap!=null&&plan.PlannedBedHeightmap.Length==raster.Length?plan.PlannedBedHeightmap:null;
        float Planned(float3 q)=>NetworkProcessor.Sample(q.x,q.z,plannedBanks!,terrain.HeightScaleMeters,width);
        var lowered=0;
        foreach(var source in plan.Sources)
        {
            sourceCount++;Pulse();var p=source.Position;p.y=Height(p);var bank=source.BankProbes.Count>0?source.BankProbes.Min(Height):source.BankLevel;
            var stage=source.TargetSurfaceLevel??source.Position.y+source.Depth;
            var planned=plannedBanks!=null&&source.BankProbes.Count>0?source.BankProbes.Min(Planned):float.NegativeInfinity;
            if(stage>bank-.25f&&(planned<stage+.05f||bank<planned-.05f))
            {
                if(stage-(bank-.35f)<=StageDrop&&bank-.35f-p.y>=.4f){stage=bank-.35f;lowered++;log.Count("Water final stages lowered to their banks");}
                else{log.Count("Water final bank conflicts");log.Diagnostic($"WATER FINAL UNRESOLVED body={source.BodyId}; way={source.WayId}; stage={stage:R}; bank={bank:R}; reason=final-bank-lowered; canonicalStageRetained=true; controlRejected=true");continue;}
            }
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
        log.Diagnostic($"WATER FINAL REVALIDATION controls={revised.Count}/{plan.Sources.Count}; adjusted={adjusted}; loweredToBank={lowered}; repairedBedSamples={repaired}; nonOwnedObstructionsBlocked={blocked}; supportBlockedSamples={supportBlocked}; restoresAcceptedOwnedBedWithoutTwoMeterCap=true; infrastructureAndBanksProtected=true; lakesKeepOneStage=true; afterEarthworks=true; canonicalStagePreserved=true; recoveryUsesCompleteOwnedWetMask=true; protectionMaskBytes={protectedCells.Length}; producerMaskBytes={finalEdits.Length}");
        TerrainArtifactRepair.TraceEncoded(raster, terrain.HeightScaleMeters, terrain.SeaLevelMeters, "after-water-final-revalidation", log.Diagnostic);
        return plan with{Sources=revised,ConditionedHeightmap=repaired>0?raster:null,FinalBedEdits=new TerrainEditSet("water-final-bed","post-roadbed-terrain",n,finalEdits),Skipped=plan.Skipped+plan.Sources.Count-revised.Count};
    }
}
