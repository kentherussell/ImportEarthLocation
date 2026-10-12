using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using ImportEarthLocation.Geo;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Existing OSM/climate evidence and final terrain, independent of surface paint.
public static class VegetationPlanner
{
    private sealed record Habitat(GeographicPolygon Polygon,float2 Min,float2 Max);
    private sealed record Evidence(long Id,float3 A,float3 B,IReadOnlyDictionary<string,string> Tags);
    public static IReadOnlyList<VegetationCandidate> Prepare(OsmDataset? data,IReadOnlyList<float3> trees,
        ImportRequest request,TerrainBuilder.Result terrain,IReadOnlyList<PreparedSegment> networks,
        IReadOnlyList<GeographicPolygon> geography,WaterPlan water,ClimateNormals? climate,ImportLog log,CancellationToken token)
    {
        if(data==null)return Array.Empty<VegetationCandidate>();
        var half=(float)request.MapSize.WidthMeters*.5f;
        var habitats=GeographyPolygons.Build(data,request,log,token).Concat(geography).GroupBy(p=>(p.Id,string.Join("|",p.Tags.OrderBy(p=>p.Key).Select(p=>p.Key+"="+p.Value)))).Select(g=>g.First()).Where(p=>p.Outer.Any(r=>r.Length>0))
            .Select(p=>new Habitat(p,p.Outer.SelectMany(r=>r).Select(p=>p.xz).Aggregate(math.min),p.Outer.SelectMany(r=>r).Select(p=>p.xz).Aggregate(math.max))).ToArray();
        var habitatGrid=new Dictionary<(int,int),List<Habitat>>();
        foreach(var h in habitats)for(var z=(int)Math.Floor(Math.Max(-half,h.Min.y)/256);z<=(int)Math.Floor(Math.Min(half,h.Max.y)/256);z++)for(var x=(int)Math.Floor(Math.Max(-half,h.Min.x)/256);x<=(int)Math.Floor(Math.Min(half,h.Max.x)/256);x++)
        {token.ThrowIfCancellationRequested();if(!habitatGrid.TryGetValue((x,z),out var list))habitatGrid[(x,z)]=list=new();list.Add(h);}
        IReadOnlyList<Habitat> At(float3 p)=>habitatGrid.TryGetValue(((int)Math.Floor(p.x/256),(int)Math.Floor(p.z/256)),out var hs)?hs.Where(h=>p.x>=h.Min.x&&p.x<=h.Max.x&&p.z>=h.Min.y&&p.z<=h.Max.y&&h.Polygon.Contains(p)).ToArray():Array.Empty<Habitat>();
        bool Tag(IReadOnlyDictionary<string,string> tags,string k,string v)=>tags.TryGetValue(k,out var actual)&&actual.Equals(v,StringComparison.OrdinalIgnoreCase);
        var evidence=new Dictionary<(int,int),List<Evidence>>();
        float3 Local(long id){var n=data.Nodes[id];return GeoMath.ToLocalMeters(n.Latitude,n.Longitude,request.Latitude,request.Longitude);}
        void Index(Evidence e,float margin)
        {
            var lo=math.min(e.A.xz,e.B.xz)-margin;var hi=math.max(e.A.xz,e.B.xz)+margin;
            for(var z=(int)Math.Floor(lo.y/64);z<=(int)Math.Floor(hi.y/64);z++)for(var x=(int)Math.Floor(lo.x/64);x<=(int)Math.Floor(hi.x/64);x++)
            {if(!evidence.TryGetValue((x,z),out var list))evidence[(x,z)]=list=new();list.Add(e);}
        }
        foreach(var node in data.NodeTags.Where(p=>Tag(p.Value,"natural","tree")||Tag(p.Value,"natural","shrub")))if(data.Nodes.ContainsKey(node.Key))Index(new(node.Key,Local(node.Key),Local(node.Key),node.Value),1);
        foreach(var row in data.Ways.Values.Where(w=>Tag(w.Tags,"natural","tree_row")))for(var i=1;i<row.NodeIds.Length;i++)if(data.Nodes.ContainsKey(row.NodeIds[i-1])&&data.Nodes.ContainsKey(row.NodeIds[i]))Index(new(row.Id,Local(row.NodeIds[i-1]),Local(row.NodeIds[i]),row.Tags),1);
        var clear=new ForestProcessor.CorridorIndex();foreach(var s in networks.Where(s=>!s.Tunnel))
        {var steps=Math.Max(1,(int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(s)/12));for(var i=0;i<steps;i++)clear.Add(NetworkNormalizer.Evaluate(s,i/(float)steps),NetworkNormalizer.Evaluate(s,(i+1)/(float)steps),Math.Max(4,s.NativeWidth*.5f)+3,token);}
        var output=new List<VegetationCandidate>();var seen=new HashSet<(int,int)>();var rejected=0;
        var exclusions=new Dictionary<string,int>(StringComparer.Ordinal);
        void Exclusion(string reason){exclusions.TryGetValue(reason,out var count);exclusions[reason]=count+1;}
        void Reject(string reason){rejected++;Exclusion(reason);}
        var waterGrid=new Dictionary<(int,int),List<RiverSourcePlan>>();
        foreach(var s in water.Sources)
        for(var z=(int)Math.Floor((s.Position.z-s.Radius-2)/64);z<=(int)Math.Floor((s.Position.z+s.Radius+2)/64);z++)
        for(var x=(int)Math.Floor((s.Position.x-s.Radius-2)/64);x<=(int)Math.Floor((s.Position.x+s.Radius+2)/64);x++)
        {if(!waterGrid.TryGetValue((x,z),out var list))waterGrid[(x,z)]=list=new();list.Add(s);}
        var warm=climate!=null&&climate.Temperatures.Length>0&&climate.Temperatures.Min()>14;
        var annualRain=climate==null?float.NaN:ClimateMath.AnnualPrecipitation(climate.Precipitation);
        var dry=annualRain<150;
        var warmArid=dry&&climate!=null&&climate.Temperatures.Length==12&&climate.Temperatures.Average()>18;
        var cold=climate!=null&&climate.Temperatures.Length>0&&climate.Temperatures.Average()<8;
        var shrubGrid=new Dictionary<(int,int),List<float3>>();
        string Value(IReadOnlyDictionary<string,string>? tags,string key)=>tags!=null&&tags.TryGetValue(key,out var v)?v.ToLowerInvariant():"";
        void Add(float3 p,bool shrub,long habitatId=0,IReadOnlyDictionary<string,string>? habitatTags=null)
        {
            token.ThrowIfCancellationRequested();if(Math.Abs(p.x)>half||Math.Abs(p.z)>half){Exclusion("outside-map");return;}
            p.y=NetworkProcessor.Sample(p.x,p.z,terrain.Heightmap,terrain.HeightScaleMeters,request.MapSize.WidthMeters);
            if(!math.all(math.isfinite(p))){Reject("non-finite-position");return;}
            if(clear.Contains(p)){Reject("network-clearance");return;}
            var hs=At(p);if(hs.Any(h=>Tag(h.Polygon.Tags,"natural","water")||h.Polygon.Tags.ContainsKey("water")||Tag(h.Polygon.Tags,"waterway","riverbank"))){Reject("mapped-water");return;}
            if(waterGrid.TryGetValue(((int)Math.Floor(p.x/64),(int)Math.Floor(p.z/64)),out var waters)&&waters.Any(s=>math.distance(s.Position.xz,p.xz)<s.Radius+2&&p.y<(s.TargetSurfaceLevel??s.Position.y+s.Depth)+.5f)){Reject("planned-water-footprint");return;}
            var dx=NetworkProcessor.Sample(p.x+6,p.z,terrain.Heightmap,terrain.HeightScaleMeters,request.MapSize.WidthMeters)-p.y;
            var dz=NetworkProcessor.Sample(p.x,p.z+6,terrain.Heightmap,terrain.HeightScaleMeters,request.MapSize.WidthMeters)-p.y;
            if(math.length(new float2(dx,dz))/6>(shrub?1.2f:.7f)){Reject("slope-limit");return;}
            Evidence? explicitSource=null;
            if(evidence.TryGetValue(((int)Math.Floor(p.x/64),(int)Math.Floor(p.z/64)),out var local))explicitSource=local.Where(e=>NetworkNormalizer.DistanceToSegment(p,e.A,e.B)<.75f).OrderBy(e=>math.distance(p.xz,e.A.xz)).FirstOrDefault();
            var habitat=hs.FirstOrDefault(h=>Tag(h.Polygon.Tags,"natural","wood")||Tag(h.Polygon.Tags,"landuse","forest"));
            var tags=explicitSource?.Tags??habitatTags??habitat?.Polygon.Tags;
            var id=explicitSource?.Id??(habitatId!=0?habitatId:habitat?.Polygon.Id??0);
            var taxon=Value(tags,"taxon")+" "+Value(tags,"species")+" "+Value(tags,"genus");
            var explicitPalm=taxon.Contains("phoenix")||taxon.Contains("washingtonia")||taxon.Contains("cocos")||taxon.Contains("arecaceae")||taxon.Contains("palm")||Value(tags,"leaf_type")=="palm";
            var conifer=Value(tags,"leaf_type")=="needleleaved"||taxon.Contains("pinus")||taxon.Contains("picea")||taxon.Contains("abies")||taxon.Contains("cedrus")||taxon.Contains("pine")||taxon.Contains("spruce");
            // Palms are broadleaved too. That generic leaf tag is not a species
            // override of a warm-arid prior; an actual oak/maple taxon still is.
            var explicitBroadleaf=(!warmArid&&Value(tags,"leaf_type")=="broadleaved")||taxon.Contains("quercus")||taxon.Contains("acer")||taxon.Contains("betula")||taxon.Contains("populus")||taxon.Contains("fagus")||taxon.Contains("oak")||taxon.Contains("maple");
            var form=shrub||explicitSource!=null&&Tag(explicitSource.Tags,"natural","shrub")?VegetationForm.Shrub:explicitPalm?VegetationForm.Palm:conifer?VegetationForm.Conifer:VegetationForm.Broadleaf;
            var beach=hs.Any(h=>Tag(h.Polygon.Tags,"natural","beach"));
            var reason=explicitSource!=null?"mapped-node/row":tags!=null?"mapped-habitat":"climate-prior";
            // Evergreen is not synonymous with conifer. Beaches are not palm
            // plantations; only already-evidenced tree sites receive this prior.
            var key=((int)Math.Round(p.x*2),(int)Math.Round(p.z*2));
            var seed=math.hash(new uint3(unchecked((uint)key.Item1),unchecked((uint)key.Item2),unchecked((uint)id)^4u));
            if(!explicitPalm&&!conifer&&!explicitBroadleaf&&form!=VegetationForm.Shrub)
            {
                if(warmArid){form=VegetationForm.Palm;reason+="/warm-arid-existing-tree-site";}
                else if(Value(tags,"leaf_type")=="mixed"){form=seed%2==0?VegetationForm.Conifer:VegetationForm.Broadleaf;reason+="/mixed-leaf";}
                else if(warm&&beach&&!dry){form=VegetationForm.Palm;reason+="/warm-beach-prior";}
                else if(cold){form=VegetationForm.Conifer;reason+="/cold-climate-prior";}
            }
            if(explicitSource==null&&beach&&form!=VegetationForm.Palm){Reject("unmapped-non-palm-beach-site");return;}
            if(!seen.Add(key)){Exclusion("duplicate-position");return;}
            if(form==VegetationForm.Shrub)
            {
                var sx=(int)Math.Floor(p.x/4);var sz=(int)Math.Floor(p.z/4);
                for(var x=sx-1;x<=sx+1;x++)for(var z=sz-1;z<=sz+1;z++)if(shrubGrid.TryGetValue((x,z),out var nearby)&&nearby.Any(q=>math.distance(p.xz,q.xz)<4)){Exclusion("shrub-spacing");return;}
                if(!shrubGrid.TryGetValue((sx,sz),out var bushSites))shrubGrid[(sx,sz)]=bushSites=new();bushSites.Add(p);
            }
            output.Add(new(p,form,seed,id,reason,taxon.Trim()));
        }
        foreach(var p in trees)Add(p,false);
        // A separate sparse shrub layer; never scatter bushes across unclassified desert.
        foreach(var h in habitats.Where(h=>Tag(h.Polygon.Tags,"natural","scrub")||Tag(h.Polygon.Tags,"natural","heath")))
        for(var z=(int)Math.Floor(Math.Max(-half,h.Min.y)/8);z<=(int)Math.Floor(Math.Min(half,h.Max.y)/8);z++)
        for(var x=(int)Math.Floor(Math.Max(-half,h.Min.x)/8);x<=(int)Math.Floor(Math.Min(half,h.Max.x)/8);x++)
        {token.ThrowIfCancellationRequested();var seed=math.hash(new int3(x,z,(int)h.Polygon.Id));if(seed%12!=0)continue;var p=new float3((x+.25f+(seed&255)/512f)*8,0,(z+.25f+((seed>>8)&255)/512f)*8);if(h.Polygon.Contains(p))Add(p,true,h.Polygon.Id,h.Polygon.Tags);}
        foreach(var node in data.NodeTags.Where(p=>Tag(p.Value,"natural","shrub")))if(data.Nodes.ContainsKey(node.Key))Add(Local(node.Key),true,node.Key,node.Value);
        // Distributed canopy first, then infill, then bushes. Stopping early
        // leaves useful woodland across the whole map, not a completed centre.
        output=output.OrderBy(p=>p.Form==VegetationForm.Shrub?2:p.Seed%4==0?0:1).ThenBy(p=>p.Seed).ToList();
        log.Diagnostic("VEGETATION WORKLOAD canopyDensityRetained=true; inferredShrubFractionOfPrevious=0.25; explicitShrubsRetained=true; order=distributed-canopy-infill-shrubs");
        foreach(var group in output.GroupBy(p=>p.Form))log.Count("Vegetation planned "+group.Key,group.Count());
        foreach(var group in output.GroupBy(p=>(p.Form,p.Evidence)))log.Diagnostic($"VEGETATION DECISION form={group.Key.Form}; evidence={group.Key.Evidence}; count={group.Count()}; exampleSourceIds={string.Join(",",group.Take(3).Select(p=>p.SourceId))}; plannedNotNativeAcceptance=true");
        foreach(var reason in exclusions.OrderBy(p=>p.Key))log.Diagnostic($"VEGETATION EXCLUSION reason={reason.Key}; count={reason.Value}; generationPolicyUnchanged=true");
        log.Diagnostic($"VEGETATION POLICY version=6; candidates={output.Count}; rejectedFinalMasks={rejected}; habitats={habitats.Length}; vanillaOnly=true; climateAvailable={climate!=null}; annualPrecipitationMm={annualRain:R}; warmAridPrior={warmArid}; uncapped=true; order=distributed-canopy-infill-shrubs; seedExcludesHeight=true; newDesertTreeSitesInvented=false; explicitTaxonOverridesClimate=true; speciesNotGuaranteed=true");
        return output;
    }
}
