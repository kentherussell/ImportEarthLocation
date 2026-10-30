using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using ImportEarthLocation.Geo;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Rock and boulder sites from OSM rock evidence and final terrain (report 69).
// Vanilla boulders are big (4–6 m) and vanilla rocks small (about 1 m), so
// RockLarge means a boulder and RockMedium/RockSmall the larger and smaller
// rocks (NativeVegetationPlacement.ResolveRocks, which also drops any piece
// that would sit inside a boulder's real footprint).
//
// Rocks collect in clumps, not a scatter. A rockiness map on a 10 m grid adds:
//   natural=cliff line    nearest cliff's closeness (1 − d/60 m, ×0.35 uphill),
//                         scaled from 0.6 for a lone cliff to 1 for four or more
//                         cliffs within 60 m (the Calanques' dense networks)
//   natural=scree         0.8 base
//   natural=rock/stone    1.0 base (areas)
//   natural=bare_rock     0.6 base
//   scrub/heath/grassland only where steeper than 31°, rising with slope
// all weighted by steepness. A clump is a pile of five to ten overlapping
// boulders, with no small rocks. Clumps settle where rocks come to rest: at the
// foot of cliffs and rocky slopes, or at the waterline; hilltops, upper slopes
// and flat rock get none (scattered rocks there looked artificial and cost most
// of the preparation and placement time). Mapped single boulders (natural=stone)
// and outcrops (natural=rock nodes) keep their own small clumps. Above
// MaximumRocks the least rocky slopes are left out.
// Positions are local metres (x east, z north); y is final terrain height.
public static class RockPlanner
{
    public const int MaximumRocks=160_000;
    private const float Cell=10;
    private const int Sub=3;
    private const float CliffReach=60;
    // tan 31°: steeper natural slopes rarely hold continuous soil.
    private const float OutcropSlope=.6f;
    private const float Spacing=.8f;
    // Water broad-phase squares (metres), and the most squares one entry may fill.
    private const float WaterCell=256;private const int WaterCellLimit=256;
    // Candidate MetadataIndex is an index into Reasons; callers merging with a
    // vegetation plan offset it by their own metadata count.
    public static readonly string[] Reasons={"stone_node","rock_node","rock_area","bare_rock","scree","cliff_talus","steep_slope","rockfall"};
    // Cliff clumps: one every CliffClumpStep metres of cliff, found by walking
    // downhill up to RockfallReach metres to where the slope eases below RockfallRest.
    private const float CliffClumpStep=60,RockfallReach=48,RockfallRest=.35f;
    // Rocky-slope clumps: at most one per SlopeBlock-metre square, walked up to
    // SlopeReach metres downhill to the slope's foot.
    private const float SlopeBlock=100,SlopeReach=120;
    // Vanilla boulders are 4.4–5.6 m across: neighbours BoulderStep apart
    // overlap a little, so a clump reads as one pile.
    private const float BoulderStep=2.6f;
    // Boulders per clump: ClumpBoulders to twice that; fewer than MinimumClump
    // placed and the clump is left out.
    private const int ClumpBoulders=5,MinimumClump=3;
    // Smooth 0–1 value noise with features about 250 m across, from hashed lattice values.
    private static float Patch(float2 p)
    {
        var q=p/250f;var i=(int2)math.floor(q);var f=q-math.floor(q);f=f*f*(3-2*f);
        float V(int2 c)=>(math.hash(new int3(c,0x9A7C))&0xFFFF)/65535f;
        return math.lerp(math.lerp(V(i),V(i+new int2(1,0)),f.x),math.lerp(V(i+new int2(0,1)),V(i+1),f.x),f.y);
    }

    // Closeness: nearest cliff's 1 − d/CliffReach (×0.35 uphill); Cliffs: distinct
    // cliff ways within reach (LastWay counts each way once per cell).
    private sealed class Site{public float Rockiness;public float Closeness,Area,Slope;public int Cliffs;public long LastWay;public int AreaReason;
        public float Cliff=>Closeness*Math.Min(1,.45f+.15f*Cliffs);}

    public static IReadOnlyList<VegetationCandidate> Prepare(OsmDataset? data,ImportRequest request,TerrainBuilder.Result terrain,
        IReadOnlyList<PreparedSegment> networks,IReadOnlyList<GeographicPolygon> geography,WaterPlan water,ImportLog log,CancellationToken token)
    {
        if(data==null||!request.ImportRocks)return Array.Empty<VegetationCandidate>();
        var timer=Stopwatch.StartNew();var half=(float)(request.MapSize.WidthMeters/2);var width=request.MapSize.WidthMeters;
        float Height(float x,float z)=>NetworkProcessor.Sample(x,z,terrain.Heightmap,terrain.HeightScaleMeters,width);
        float Slope(float x,float z)
        {
            var y=Height(x,z);var dx=Height(x+6,z)-y;var dz=Height(x,z+6)-y;
            return math.length(new float2(dx,dz))/6;
        }
        bool Tag(IReadOnlyDictionary<string,string> tags,string key,string value)=>tags.TryGetValue(key,out var actual)&&actual==value;

        var corridors=new ForestProcessor.CorridorIndex();
        foreach(var segment in networks)
        {
            token.ThrowIfCancellationRequested();if(segment.Tunnel)continue;
            var radius=(segment.NativeWidth>0?segment.NativeWidth*.5f:Math.Max(4,segment.Lanes*1.75f))+3;
            var steps=Math.Max(1,(int)Math.Ceiling(math.distance(segment.Start,segment.End)/10f));
            for(var i=0;i<steps;i++)corridors.Add(NetworkNormalizer.Evaluate(segment,i/(float)steps),NetworkNormalizer.Evaluate(segment,(i+1)/(float)steps),radius,token);
        }
        var wet=geography.Where(p=>Tag(p.Tags,"natural","water")||p.Tags.ContainsKey("water")||Tag(p.Tags,"waterway","riverbank")).ToArray();
        var wetBounds=wet.Select(p=>(Min:p.Outer.SelectMany(r=>r).Select(q=>q.xz).Aggregate(math.min),Max:p.Outer.SelectMany(r=>r).Select(q=>q.xz).Aggregate(math.max))).ToArray();
        var sea=WaterLevels.Effective(terrain.SeaLevelMeters)+.5f;
        // Broad phase only: each water polygon (by its bounds) and source circle
        // (radius + 2 m) is listed in the WaterCell squares it touches, or among
        // the large entries checked everywhere; the exact tests below decide.
        var wetCells=new Dictionary<(int,int),List<int>>();var wetLarge=new List<int>();
        var sourceCells=new Dictionary<(int,int),List<int>>();var sourceLarge=new List<int>();
        void Index(Dictionary<(int,int),List<int>> cells,List<int> large,int i,float2 min,float2 max)
        {
            int x0=(int)Math.Floor(min.x/WaterCell),x1=(int)Math.Floor(max.x/WaterCell),z0=(int)Math.Floor(min.y/WaterCell),z1=(int)Math.Floor(max.y/WaterCell);
            if((long)(x1-x0+1)*(z1-z0+1)>WaterCellLimit){large.Add(i);return;}
            for(var x=x0;x<=x1;x++)for(var z=z0;z<=z1;z++){if(!cells.TryGetValue((x,z),out var list))cells[(x,z)]=list=new();list.Add(i);}
        }
        for(var i=0;i<wet.Length;i++)Index(wetCells,wetLarge,i,wetBounds[i].Min,wetBounds[i].Max);
        for(var i=0;i<water.Sources.Count;i++){var s=water.Sources[i];var r=s.Radius+2;Index(sourceCells,sourceLarge,i,s.Position.xz-r,s.Position.xz+r);}
        (int,int) WaterKey(float2 at)=>((int)Math.Floor(at.x/WaterCell),(int)Math.Floor(at.y/WaterCell));
        bool WetPolygon(int i,float2 at)=>at.x>=wetBounds[i].Min.x&&at.x<=wetBounds[i].Max.x&&at.y>=wetBounds[i].Min.y&&at.y<=wetBounds[i].Max.y&&wet[i].Contains(new float3(at.x,0,at.y));
        bool Wet(float2 at)
        {
            if(wetCells.TryGetValue(WaterKey(at),out var near))foreach(var i in near)if(WetPolygon(i,at))return true;
            foreach(var i in wetLarge)if(WetPolygon(i,at))return true;
            return false;
        }
        bool NearSource(float2 at)
        {
            bool Inside(int i)=>math.distance(water.Sources[i].Position.xz,at)<water.Sources[i].Radius+2;
            if(sourceCells.TryGetValue(WaterKey(at),out var near))foreach(var i in near)if(Inside(i))return true;
            foreach(var i in sourceLarge)if(Inside(i))return true;
            return false;
        }

        var output=new List<VegetationCandidate>();var counts=new Dictionary<string,int>(StringComparer.Ordinal);
        void Count(string key,int n=1){counts.TryGetValue(key,out var c);counts[key]=c+n;}
        var occupied=new Dictionary<(int,int),List<float2>>();
        bool Accept(float2 at,VegetationForm form,long source,uint seed,string reason)
        {
            token.ThrowIfCancellationRequested();
            if(!math.all(math.isfinite(at))||Math.Abs(at.x)>half||Math.Abs(at.y)>half){Count("outside");return false;}
            var p=new float3(at.x,Height(at.x,at.y),at.y);
            if(!math.isfinite(p.y)||p.y<=sea){Count("water");return false;}
            if(corridors.Contains(p)){Count("roads");return false;}
            if(Wet(at)){Count("water");return false;}
            if(NearSource(at)){Count("water");return false;}
            var bucket=((int)Math.Floor(at.x/Spacing),(int)Math.Floor(at.y/Spacing));
            for(var x=-1;x<=1;x++)for(var z=-1;z<=1;z++)
                if(occupied.TryGetValue((bucket.Item1+x,bucket.Item2+z),out var near)&&near.Any(q=>math.distance(q,at)<Spacing)){Count("spacing");return false;}
            if(!occupied.TryGetValue(bucket,out var list))occupied[bucket]=list=new();list.Add(at);
            output.Add(new(p,form,source,seed,Array.IndexOf(Reasons,reason)));Count(reason);return true;
        }
        float2 Direction(uint h){var angle=(h&0xFFFF)/65536f*math.PI*2;return new float2(math.cos(angle),math.sin(angle));}
        // A clump: a pile of ClumpBoulders boulders, each placed against one
        // already in it so they overlap slightly. Small rocks were too small to
        // read and cost placement, so none are placed. A clump that cannot stand
        // MinimumClump boulders (road, water, map edge) is removed whole rather
        // than leaving a lone boulder.
        void Clump(float2 center,int boulders,int rocks,long source,uint seed,string reason)
        {
            if(output.Count>=MaximumRocks)return;
            // The density slider keeps whole clumps, so the kept ones stay dense.
            if(request.RockDensity<1&&(math.hash(new uint2(seed,0xD5u))&0xFFFF)/65536f>=request.RockDensity)return;
            var placed=new List<float2>();var first=output.Count;var count=ClumpBoulders+(int)(seed>>12)%(ClumpBoulders+1);
            for(var k=0;k<count*2&&placed.Count<count;k++)
            {
                var h=math.hash(new uint3(seed,(uint)k,0xB0u));
                var at=placed.Count==0?center:placed[(int)(h>>20)%placed.Count]+BoulderStep*Direction(h);
                if(Accept(at,VegetationForm.RockLarge,source,h,reason))placed.Add(at);
                else if(placed.Count==0)return;
            }
            if(placed.Count>=MinimumClump)return;
            output.RemoveRange(first,output.Count-first);Count("clumps too small",1);
            foreach(var at in placed)occupied[((int)Math.Floor(at.x/Spacing),(int)Math.Floor(at.y/Spacing))].Remove(at);
        }
        // Downhill from a start in 4 m steps until the slope eases below
        // RockfallRest or the ground meets the sea or mapped water: where fallen
        // rocks settle, the last dry step on a shoreline.
        (float2 Foot,bool Shore) Settle(float2 start,float2 down,float reach)
        {
            var foot=start;
            for(var walked=4f;walked<=reach;walked+=4)
            {
                var next=start+down*walked;
                if(Height(next.x,next.y)<=sea||Wet(next))return(foot,true);
                foot=next;if(walked>=8&&Slope(next.x,next.y)<RockfallRest)break;
            }
            return(foot,false);
        }
        float3 Local(long id){var n=data.Nodes[id];return GeoMath.ToLocalMeters(n.Latitude,n.Longitude,request.Latitude,request.Longitude);}

        // Mapped single boulders and outcrops first: the strongest evidence wins spacing.
        foreach(var node in data.NodeTags.OrderBy(p=>p.Key))
        {
            if(!data.Nodes.ContainsKey(node.Key))continue;
            var stone=Tag(node.Value,"natural","stone");if(!stone&&!Tag(node.Value,"natural","rock"))continue;
            var p=Local(node.Key);var seed=math.hash(new int2(unchecked((int)node.Key),unchecked((int)(node.Key>>32))));
            Clump(p.xz,stone?1:1+(int)(seed>>4&1),stone?4+(int)(seed>>8)%3:6+(int)(seed>>8)%5,node.Key,seed,stone?"stone_node":"rock_node");
        }

        // Rockiness field.
        var field=new Dictionary<(int,int),Site>();
        Site At(int x,int z){if(!field.TryGetValue((x,z),out var s))field[(x,z)]=s=new Site();return s;}
        // A cheap per-cell test (slope) runs before the polygon test, so large
        // gentle scrub areas cost one height sample per cell and create nothing.
        void Fill(GeographicPolygon polygon,Func<float2,float> value,Action<Site,float> apply)
        {
            var points=polygon.Outer.SelectMany(r=>r).ToArray();if(points.Length==0)return;
            var min=math.max(points.Select(p=>p.xz).Aggregate(math.min),-half);var max=math.min(points.Select(p=>p.xz).Aggregate(math.max),half);
            for(var z=(int)Math.Floor(min.y/Cell);z<=(int)Math.Floor(max.y/Cell);z++)
            for(var x=(int)Math.Floor(min.x/Cell);x<=(int)Math.Floor(max.x/Cell);x++)
            {
                token.ThrowIfCancellationRequested();var c=new float2((x+.5f)*Cell,(z+.5f)*Cell);
                var v=value(c);if(v>0&&polygon.Contains(new float3(c.x,0,c.y)))apply(At(x,z),v);
            }
        }
        var polygons=GeographyPolygons.Build(data,request,log,token);
        foreach(var polygon in polygons.OrderBy(p=>p.Identity))
        {
            if(Tag(polygon.Tags,"natural","rock")||Tag(polygon.Tags,"natural","stone"))Fill(polygon,_=>1f,(s,v)=>{if(s.Area<v){s.Area=v;s.AreaReason=2;}});
            else if(Tag(polygon.Tags,"natural","scree"))Fill(polygon,_=>.8f,(s,v)=>{if(s.Area<v){s.Area=v;s.AreaReason=4;}});
            else if(Tag(polygon.Tags,"natural","bare_rock"))Fill(polygon,_=>.6f,(s,v)=>{if(s.Area<v){s.Area=v;s.AreaReason=3;}});
            else if(Tag(polygon.Tags,"natural","scrub")||Tag(polygon.Tags,"natural","heath")||Tag(polygon.Tags,"natural","grassland"))
                Fill(polygon,c=>Math.Min(.7f,(Slope(c.x,c.y)-OutcropSlope)*1.5f),(s,v)=>s.Slope=Math.Max(s.Slope,v));
        }
        // Cliff lines: each segment raises cells within CliffReach, fuller on its
        // lower side where talus collects; many nearby cliffs add up to a field.
        foreach(var way in data.Ways.Values.Where(w=>Tag(w.Tags,"natural","cliff")).OrderBy(w=>w.Id))
        {
            if(way.NodeIds.Any(id=>!data.Nodes.ContainsKey(id)))continue;var line=way.NodeIds.Select(id=>Local(id).xz).ToArray();
            for(var i=1;i<line.Length;i++)
            {
                token.ThrowIfCancellationRequested();var a=line[i-1];var b=line[i];var length=math.distance(a,b);if(length<.01f)continue;
                var along=(b-a)/length;var normal=new float2(-along.y,along.x);var mid=(a+b)/2;
                var down=Height(mid.x+normal.x*8,mid.y+normal.y*8)<Height(mid.x-normal.x*8,mid.y-normal.y*8)?normal:-normal;
                var lo=math.max(math.min(a,b)-CliffReach,-half);var hi=math.min(math.max(a,b)+CliffReach,half);
                for(var z=(int)Math.Floor(lo.y/Cell);z<=(int)Math.Floor(hi.y/Cell);z++)
                for(var x=(int)Math.Floor(lo.x/Cell);x<=(int)Math.Floor(hi.x/Cell);x++)
                {
                    var c=new float2((x+.5f)*Cell,(z+.5f)*Cell);var t=math.clamp(math.dot(c-a,along),0,length);var offset=c-(a+along*t);var d=math.length(offset);
                    if(d>CliffReach)continue;var weight=(1-d/CliffReach)*(math.dot(offset,down)>=0?1f:.35f);
                    var s=At(x,z);s.Closeness=Math.Max(s.Closeness,weight);if(s.LastWay!=way.Id){s.LastWay=way.Id;s.Cliffs++;}
                }
            }
        }
        foreach(var pair in field)
        {
            var s=pair.Value;var c=new float2((pair.Key.Item1+.5f)*Cell,(pair.Key.Item2+.5f)*Cell);
            var raw=Math.Max(Math.Max(s.Cliff,s.Area),s.Slope);if(raw<=0)continue;
            // Steepness weighting: flat rocky ground 0.35, at 40° and steeper 1.
            s.Rockiness=Math.Min(1,raw*(.35f+.65f*Math.Min(1,Slope(c.x,c.y)/.84f)));
        }

        // Rockfall: rocks that fell from a cliff lie where the slope below it
        // levels out, or at the waterline when the cliff meets water. Every
        // CliffClumpStep metres along each cliff one clump settles there.
        var piles=0;var shoreClumps=0;
        foreach(var way in data.Ways.Values.Where(w=>Tag(w.Tags,"natural","cliff")).OrderBy(w=>w.Id))
        {
            if(way.NodeIds.Any(id=>!data.Nodes.ContainsKey(id)))continue;var line=way.NodeIds.Select(id=>Local(id).xz).ToArray();
            // Steps run along the whole cliff, so a short-segment way still gets
            // one clump per CliffClumpStep rather than one per segment.
            var travelled=0f;var nextClump=CliffClumpStep*.5f;
            for(var i=1;i<line.Length;i++)
            {
                var a=line[i-1];var b=line[i];var length=math.distance(a,b);if(length<.01f)continue;
                var along=(b-a)/length;var normal=new float2(-along.y,along.x);
                for(;nextClump<travelled+length;nextClump+=CliffClumpStep)
                {
                    if(output.Count>=MaximumRocks)break;
                    var seed=math.hash(new uint4((uint)way.Id,(uint)(way.Id>>32),(uint)i,(uint)(nextClump*10)));
                    var start=a+along*(nextClump-travelled);
                    var down=Height(start.x+normal.x*6,start.y+normal.y*6)<Height(start.x-normal.x*6,start.y-normal.y*6)?normal:-normal;
                    var (foot,shore)=Settle(start,down,RockfallReach);
                    Clump(foot,1+(int)(seed>>4)%3,8+(int)((seed>>8)%9),way.Id,seed,"rockfall");
                    piles++;if(shore)shoreClumps++;
                }
                travelled+=length;
            }
        }

        // Rocky slopes (rock, scree and bare-rock areas, cliff surroundings and
        // steep scrub): one clump per SlopeBlock square at most, from the block's
        // rockiest steep cell, settled at the foot of its slope or the shoreline.
        // Tops, upper slopes and flat rock get none. A low-frequency patch value
        // (250 m features) leaves some rocky slopes bare and others collecting.
        var blocks=new Dictionary<(int,int),(float Rockiness,(int,int) Cell)>();
        foreach(var pair in field)
        {
            var s=pair.Value;if(s.Rockiness<.3f)continue;
            var c=new float2((pair.Key.Item1+.5f)*Cell,(pair.Key.Item2+.5f)*Cell);if(Slope(c.x,c.y)<OutcropSlope*.8f)continue;
            var block=((int)Math.Floor(c.x/SlopeBlock),(int)Math.Floor(c.y/SlopeBlock));
            if(!blocks.TryGetValue(block,out var best)||s.Rockiness>best.Rockiness||s.Rockiness==best.Rockiness&&pair.Key.CompareTo(best.Cell)<0)
                blocks[block]=(s.Rockiness,pair.Key);
        }
        var capped=false;var slopeClumps=0;
        foreach(var pair in blocks.OrderByDescending(p=>p.Value.Rockiness).ThenBy(p=>p.Key.Item1).ThenBy(p=>p.Key.Item2))
        {
            if(output.Count>=MaximumRocks){capped=true;break;}
            var key=pair.Value.Cell;var s=field[key];var center=new float2((key.Item1+.5f)*Cell,(key.Item2+.5f)*Cell);
            var patch=Patch(center);if(patch<.4f)continue;
            var reason=s.Cliff>=Math.Max(s.Area,s.Slope)?"cliff_talus":s.Area>=s.Slope?Reasons[s.AreaReason]:"steep_slope";
            var gradient=new float2(Height(center.x+4,center.y)-Height(center.x-4,center.y),Height(center.x,center.y+4)-Height(center.x,center.y-4));
            if(math.lengthsq(gradient)<1e-6f)continue;
            var seed=math.hash(new int3(key.Item1,key.Item2,0x40C5));
            var (foot,shore)=Settle(center,-math.normalize(gradient),SlopeReach);
            Clump(foot,1+(int)(seed>>4&1),6+(int)((seed>>8)%7),0,seed,reason);
            slopeClumps++;if(shore)shoreClumps++;
        }

        string N(string key)=>counts.TryGetValue(key,out var c)?c.ToString("N0"):"0";
        foreach(var form in new[]{VegetationForm.RockSmall,VegetationForm.RockMedium,VegetationForm.RockLarge})
        {var n=output.Count(c=>c.Form==form);if(n>0)log.Count("Rocks planned "+form,n);}
        log.Count("Rocks prepared",output.Count);log.Time("Rock preparation",timer.Elapsed.TotalSeconds);
        log.Evidence($"Rocks prepared: {output.Count:N0} ({output.Count(c=>c.Form==VegetationForm.RockLarge):N0} boulders) in {timer.Elapsed.TotalSeconds:0.0} s from {field.Count(p=>p.Value.Rockiness>.05f):N0} rocky 10 m cells{(capped?$", capped at {MaximumRocks:N0} keeping the rockiest":"")}; clumps: {piles:N0} below cliffs, {slopeClumps:N0} at the foot of rocky slopes, {shoreClumps:N0} of them on the shoreline; mapped boulders {N("stone_node")}, outcrops {N("rock_node")}; rocks by slope source: rock areas {N("rock_area")}, bare rock {N("bare_rock")}, scree {N("scree")}, cliffs {N("cliff_talus")}, steep slopes {N("steep_slope")}, rockfall {N("rockfall")}; rejected near roads {N("roads")}, in water {N("water")}, spacing {N("spacing")}.");
        return output;
    }
}
