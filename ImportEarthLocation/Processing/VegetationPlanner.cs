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
    private sealed record TaxonEvidence(string Raw,string Trimmed,string LeafType);
    public static VegetationPlan Prepare(OsmDataset? data,IReadOnlyList<float3> trees,
        ImportRequest request,TerrainBuilder.Result terrain,IReadOnlyList<PreparedSegment> networks,
        IReadOnlyList<GeographicPolygon> geography,WaterPlan water,ClimateNormals? climate,ImportLog log,CancellationToken token,
        Func<float3,float?>? waterSurface=null)
    {
        if(data==null)return VegetationPlan.Empty;
        var half=(float)request.MapSize.WidthMeters*.5f;
        var habitats=GeographyPolygons.Build(data,request,log,token).Concat(geography).GroupBy(p=>(p.Id,string.Join("|",p.Tags.OrderBy(p=>p.Key).Select(p=>p.Key+"="+p.Value)))).Select(g=>g.First()).Where(p=>p.Outer.Any(r=>r.Length>0))
            .Select(p=>new Habitat(p,p.Outer.SelectMany(r=>r).Select(p=>p.xz).Aggregate(math.min),p.Outer.SelectMany(r=>r).Select(p=>p.xz).Aggregate(math.max))).ToArray();
        var habitatGrid=new Dictionary<(int,int),List<Habitat>>();
        foreach(var h in habitats)for(var z=(int)Math.Floor(Math.Max(-half,h.Min.y)/256);z<=(int)Math.Floor(Math.Min(half,h.Max.y)/256);z++)for(var x=(int)Math.Floor(Math.Max(-half,h.Min.x)/256);x<=(int)Math.Floor(Math.Min(half,h.Max.x)/256);x++)
        {token.ThrowIfCancellationRequested();if(!habitatGrid.TryGetValue((x,z),out var list))habitatGrid[(x,z)]=list=new();list.Add(h);}
        bool Tag(IReadOnlyDictionary<string,string> tags,string k,string v)=>tags.TryGetValue(k,out var actual)&&actual.Equals(v,StringComparison.OrdinalIgnoreCase);
        (bool Water,bool Beach,Habitat? Forest,bool Built) At(float3 p)
        {
            var wet=false;var beach=false;var built=false;Habitat? forest=null;
            if(habitatGrid.TryGetValue(((int)Math.Floor(p.x/256),(int)Math.Floor(p.z/256)),out var bucket))
            foreach(var h in bucket)
            {
                if(p.x<h.Min.x||p.x>h.Max.x||p.z<h.Min.y||p.z>h.Max.y||!h.Polygon.Contains(p))continue;
                var tags=h.Polygon.Tags;
                wet|=Tag(tags,"natural","water")||tags.ContainsKey("water")||Tag(tags,"waterway","riverbank");
                beach|=Tag(tags,"natural","beach");
                built|=tags.ContainsKey("building")||Tag(tags,"amenity","parking");
                if(forest==null&&ForestProcessor.Kind(tags)==ForestProcessor.AreaKind.Woodland)forest=h;
            }
            return(wet,beach,forest,built);
        }
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
        var metadata=new List<VegetationMetadata>();var metadataIndex=new Dictionary<(string,string),int>();
        var taxonCache=new Dictionary<IReadOnlyDictionary<string,string>,TaxonEvidence>();
        var noTaxon=new TaxonEvidence("  ","","");
        var reasonVariants=new string[4,5];var bases=new[]{"mapped-node/row","mapped-habitat","climate-prior","riparian-bank"};
        var suffixes=new[]{"","/warm-arid-existing-tree-site","/mixed-leaf","/warm-beach-prior","/cold-climate-prior"};
        for(var i=0;i<4;i++)for(var j=0;j<5;j++)reasonVariants[i,j]=bases[i]+suffixes[j];
        var formCounts=new int[4];
        TaxonEvidence Taxon(IReadOnlyDictionary<string,string>? tags)
        {
            if(tags==null)return noTaxon;
            if(taxonCache.TryGetValue(tags,out var taxon))return taxon;
            var raw=Value(tags,"taxon")+" "+Value(tags,"species")+" "+Value(tags,"genus");
            return taxonCache[tags]=new TaxonEvidence(raw,raw.Trim(),Value(tags,"leaf_type"));
        }
        int Metadata(string reason,string taxon)
        {
            var key=(reason,taxon);
            if(metadataIndex.TryGetValue(key,out var index))return index;
            index=metadata.Count;metadataIndex.Add(key,index);metadata.Add(new(reason,taxon));return index;
        }
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
        var continuousWater=waterSurface??WaterBodyPlanner.ConstructedSurfaceSampler(water,geography);
        var bankSites=new Dictionary<(int,int),List<(float3 Point,bool Shrub)>>();var bankTrees=0;var bankShrubs=0;
        var bankCells=new HashSet<(int,int)>();
        void MarkBank(float3 a,float3 b,float margin)
        {
            var steps=Math.Max(1,(int)Math.Ceiling(math.distance(a.xz,b.xz)/64));var radius=(int)Math.Ceiling(margin/64);
            for(var k=0;k<=steps;k++)
            {
                var p=math.lerp(a,b,k/(float)steps);var cx=(int)Math.Floor(p.x/64);var cz=(int)Math.Floor(p.z/64);
                for(var z=cz-radius;z<=cz+radius;z++)for(var x=cx-radius;x<=cx+radius;x++)bankCells.Add((x,z));
            }
        }
        foreach(var r in water.ReachStages)MarkBank(r.Start,r.End,128+Math.Max(r.HalfWidth,water.WaterCellSize));
        var bankPolygons=new HashSet<long>(water.ReachStages.Where(r=>r.PolygonId.HasValue).Select(r=>r.PolygonId!.Value));
        foreach(var polygon in geography.Where(p=>bankPolygons.Contains(p.Identity)))foreach(var ring in polygon.Outer.Concat(polygon.Inner))
            for(var k=0;k<ring.Length;k++)MarkBank(ring[k],ring[(k+1)%ring.Length],128);
        var shrubGrid=new Dictionary<(int,int),List<float3>>();
        string Value(IReadOnlyDictionary<string,string>? tags,string key)=>tags!=null&&tags.TryGetValue(key,out var v)?v.ToLowerInvariant():"";
        void Add(float3 p,bool shrub,long habitatId=0,IReadOnlyDictionary<string,string>? habitatTags=null,bool riparian=false)
        {
            token.ThrowIfCancellationRequested();if(Math.Abs(p.x)>half||Math.Abs(p.z)>half){Exclusion("outside-map");return;}
            p.y=NetworkProcessor.Sample(p.x,p.z,terrain.Heightmap,terrain.HeightScaleMeters,request.MapSize.WidthMeters);
            if(!math.all(math.isfinite(p))){Reject("non-finite-position");return;}
            if(clear.Contains(p)){Reject("network-clearance");return;}
            var hs=At(p);if(hs.Water){Reject("mapped-water");return;}
            if(continuousWater(p) is float waterLevel&&p.y<=waterLevel+.5f){Reject("planned-water-channel");return;}
            if(riparian&&hs.Built){Reject("riparian-built-land");return;}
            if(waterGrid.TryGetValue(((int)Math.Floor(p.x/64),(int)Math.Floor(p.z/64)),out var waters))
                foreach(var s in waters)if(math.distance(s.Position.xz,p.xz)<s.Radius+2&&p.y<(s.TargetSurfaceLevel??s.Position.y+s.Depth)+.5f){Reject("planned-water-footprint");return;}
            Evidence? explicitSource=null;
            if(evidence.TryGetValue(((int)Math.Floor(p.x/64),(int)Math.Floor(p.z/64)),out var local))
            {
                var nearest=float.PositiveInfinity;
                foreach(var e in local)
                {
                    if(NetworkNormalizer.DistanceToSegment(p,e.A,e.B)>=.75f)continue;
                    var distance=math.distance(p.xz,e.A.xz);
                    if(distance<nearest){nearest=distance;explicitSource=e;}
                }
            }
            var habitat=hs.Forest;
            // Central differences avoid treating a one-sided bump as the whole
            // hillside grade. Ratios are rise/run: 1.4 is about 54 degrees,
            // 1.7 about 60, and 2 about 63.
            var east=NetworkProcessor.Sample(p.x+6,p.z,terrain.Heightmap,terrain.HeightScaleMeters,request.MapSize.WidthMeters);
            var west=NetworkProcessor.Sample(p.x-6,p.z,terrain.Heightmap,terrain.HeightScaleMeters,request.MapSize.WidthMeters);
            var north=NetworkProcessor.Sample(p.x,p.z+6,terrain.Heightmap,terrain.HeightScaleMeters,request.MapSize.WidthMeters);
            var south=NetworkProcessor.Sample(p.x,p.z-6,terrain.Heightmap,terrain.HeightScaleMeters,request.MapSize.WidthMeters);
            var bush=shrub||explicitSource!=null&&Tag(explicitSource.Tags,"natural","shrub");
            var slopeLimit=bush?2f:explicitSource!=null||habitat!=null?1.7f:riparian?1.2f:1.4f;
            if(math.length(new float2(east-west,north-south))/12>slopeLimit){Reject("slope-limit");return;}
            // Allow smooth steep planes, but reject large local ledges that can
            // leave the root footprint hanging in space.
            if(Math.Abs(east+west-2*p.y)+Math.Abs(north+south-2*p.y)>(bush?16:12)){Reject("terrain-break");return;}
            var tags=explicitSource?.Tags??habitatTags??habitat?.Polygon.Tags;
            var id=explicitSource?.Id??(habitatId!=0?habitatId:habitat?.Polygon.Id??0);
            var taxonInfo=Taxon(tags);var taxon=taxonInfo.Raw;
            var explicitPalm=taxon.Contains("phoenix")||taxon.Contains("washingtonia")||taxon.Contains("cocos")||taxon.Contains("arecaceae")||taxon.Contains("palm")||taxonInfo.LeafType=="palm";
            var conifer=taxonInfo.LeafType=="needleleaved"||taxon.Contains("pinus")||taxon.Contains("picea")||taxon.Contains("abies")||taxon.Contains("cedrus")||taxon.Contains("pine")||taxon.Contains("spruce");
            // Palms are broadleaved too. That generic leaf tag is not a species
            // override of a warm-arid prior; an actual oak/maple taxon still is.
            var explicitBroadleaf=(!warmArid&&taxonInfo.LeafType=="broadleaved")||taxon.Contains("quercus")||taxon.Contains("acer")||taxon.Contains("betula")||taxon.Contains("populus")||taxon.Contains("fagus")||taxon.Contains("oak")||taxon.Contains("maple");
            var form=shrub||explicitSource!=null&&Tag(explicitSource.Tags,"natural","shrub")?VegetationForm.Shrub:explicitPalm?VegetationForm.Palm:conifer?VegetationForm.Conifer:VegetationForm.Broadleaf;
            var beach=hs.Beach;
            var reasonBase=riparian?3:explicitSource!=null?0:tags!=null?1:2;var reasonVariant=0;
            // Evergreen is not synonymous with conifer. Beaches are not palm
            // plantations; only already-evidenced tree sites receive this prior.
            var key=((int)Math.Round(p.x*2),(int)Math.Round(p.z*2));
            var seed=math.hash(new uint3(unchecked((uint)key.Item1),unchecked((uint)key.Item2),unchecked((uint)id)^4u));
            if(!explicitPalm&&!conifer&&!explicitBroadleaf&&form!=VegetationForm.Shrub)
            {
                if(warmArid&&(!riparian||climate!.Temperatures.Min()>5)){form=VegetationForm.Palm;reasonVariant=1;}
                else if(taxonInfo.LeafType=="mixed"){form=seed%2==0?VegetationForm.Conifer:VegetationForm.Broadleaf;reasonVariant=2;}
                else if(warm&&beach&&!dry){form=VegetationForm.Palm;reasonVariant=3;}
                else if(cold){form=VegetationForm.Conifer;reasonVariant=4;}
            }
            if(explicitSource==null&&beach&&form!=VegetationForm.Palm){Reject("unmapped-non-palm-beach-site");return;}
            var bx=(int)Math.Floor(p.x/8);var bz=(int)Math.Floor(p.z/8);
            if(riparian)
                for(var x=bx-1;x<=bx+1;x++)for(var z=bz-1;z<=bz+1;z++)if(bankSites.TryGetValue((x,z),out var sites))
                    foreach(var site in sites)if(math.distance(p.xz,site.Point.xz)<(shrub&&site.Shrub?4:shrub||site.Shrub?5:8)){Exclusion("riparian-spacing");return;}
            if(!seen.Add(key)){Exclusion("duplicate-position");return;}
            if(form==VegetationForm.Shrub)
            {
                var sx=(int)Math.Floor(p.x/4);var sz=(int)Math.Floor(p.z/4);
                for(var x=sx-1;x<=sx+1;x++)for(var z=sz-1;z<=sz+1;z++)if(shrubGrid.TryGetValue((x,z),out var nearby))
                    foreach(var q in nearby)if(math.distance(p.xz,q.xz)<4){Exclusion("shrub-spacing");return;}
                if(!shrubGrid.TryGetValue((sx,sz),out var bushSites))shrubGrid[(sx,sz)]=bushSites=new();bushSites.Add(p);
            }
            output.Add(new(p,form,id,seed,Metadata(reasonVariants[reasonBase,reasonVariant],taxonInfo.Trimmed)));formCounts[(int)form]++;
            if(bankCells.Contains(((int)Math.Floor(p.x/64),(int)Math.Floor(p.z/64))))
            {if(!bankSites.TryGetValue((bx,bz),out var plants))bankSites[(bx,bz)]=plants=new();plants.Add((p,form==VegetationForm.Shrub));}
            if(riparian){if(form==VegetationForm.Shrub)bankShrubs++;else bankTrees++;}
        }
        foreach(var p in trees)Add(p,false);
        // A separate sparse shrub layer; never scatter bushes across unclassified desert.
        foreach(var h in habitats.Where(h=>Tag(h.Polygon.Tags,"natural","scrub")||Tag(h.Polygon.Tags,"natural","heath")))
        for(var z=(int)Math.Floor(Math.Max(-half,h.Min.y)/8);z<=(int)Math.Floor(Math.Min(half,h.Max.y)/8);z++)
        for(var x=(int)Math.Floor(Math.Max(-half,h.Min.x)/8);x<=(int)Math.Floor(Math.Min(half,h.Max.x)/8);x++)
        {token.ThrowIfCancellationRequested();var seed=math.hash(new int3(x,z,(int)h.Polygon.Id));if(seed%12!=0)continue;var p=new float3((x+.25f+(seed&255)/512f)*8,0,(z+.25f+((seed>>8)&255)/512f)*8);if(h.Polygon.Contains(p))Add(p,true,h.Polygon.Id,h.Polygon.Tags);}
        foreach(var node in data.NodeTags.Where(p=>Tag(p.Value,"natural","shrub")))if(data.Nodes.ContainsKey(node.Key))Add(Local(node.Key),true,node.Key,node.Value);
        // New growth is confined to a narrow dry-bank strip. A stable coarse
        // hash selects clumps; finer hashes place shrubs and occasional trees
        // within each clump rather than forming an uninterrupted planted row.
        void Bank(float3 point,float3 outward,float level,long body)
        {
            var length=math.length(outward.xz);if(length<.001f)return;outward/=length;
            var shore=point;var found=false;
            for(var d=0f;d<=96;d+=3)
            {
                shore=point+outward*d;shore.y=NetworkProcessor.Sample(shore.x,shore.z,terrain.Heightmap,terrain.HeightScaleMeters,request.MapSize.WidthMeters);
                if(shore.y>level+.5f&&!At(shore).Water){found=true;break;}
            }
            if(!found||shore.y>level+8)return;
            var habitat=At(shore);var uphill=shore+outward*120;
            var relief=Math.Abs(NetworkProcessor.Sample(uphill.x,uphill.z,terrain.Heightmap,terrain.HeightScaleMeters,request.MapSize.WidthMeters)-shore.y);
            var moist=habitat.Forest!=null||annualRain>=600||annualRain>=350&&relief>35;var arid=annualRain<350;
            var patch=math.hash(new int3((int)Math.Floor(shore.x/64),(int)Math.Floor(shore.z/64),unchecked((int)body)));
            if(patch%100>=(moist?80:arid?35:55))return;
            var seed=math.hash(new int3((int)Math.Round(shore.x/3),(int)Math.Round(shore.z/3),unchecked((int)body)^0x4B17));
            var count=moist?4:arid?1:2;var band=moist?18f:arid?7f:12f;
            for(var k=0;k<count;k++)
            {
                var hash=math.hash(new uint2(seed,(uint)k));var along=((hash&255)/255f-.5f)*10;var across=2+((hash>>8)&255)/255f*band;
                var site=shore+outward*across+new float3(-outward.z,0,outward.x)*along;
                var height=NetworkProcessor.Sample(site.x,site.z,terrain.Heightmap,terrain.HeightScaleMeters,request.MapSize.WidthMeters);
                if(height<=level+.5f||height>level+(moist?8:4))continue;
                Add(site,true,body,riparian:true);
            }
            if(seed%100<(moist?30:arid?10:20))
            {
                var site=shore+outward*(4+(seed%11));var height=NetworkProcessor.Sample(site.x,site.z,terrain.Heightmap,terrain.HeightScaleMeters,request.MapSize.WidthMeters);
                if(height>level+.5f&&height<=level+(moist?8:4))Add(site,false,body,riparian:true);
            }
        }
        var mappedBanks=new HashSet<long>();
        foreach(var reach in water.ReachStages.OrderBy(r=>r.BodyId).ThenBy(r=>r.WayId).ThenBy(r=>r.Start.x).ThenBy(r=>r.Start.z))
        {
            token.ThrowIfCancellationRequested();
            if(reach.PolygonId is long polygonId)
            {
                if(!mappedBanks.Add(polygonId))continue;
                var polygon=geography.FirstOrDefault(p=>p.Identity==polygonId);if(polygon==null)continue;
                foreach(var ring in polygon.Outer.Concat(polygon.Inner))for(var k=0;k<ring.Length;k++)
                {
                    var a=ring[k];var b=ring[(k+1)%ring.Length];var delta=b-a;var steps=Math.Max(1,(int)Math.Ceiling(math.length(delta.xz)/12));
                    var normal=math.normalizesafe(new float3(-delta.z,0,delta.x));
                    for(var j=0;j<steps;j++)
                    {
                        var p=math.lerp(a,b,(j+.5f)/steps);var side=polygon.Contains(p+normal*2)?-normal:normal;
                        var level=continuousWater(p-normal*.5f)??continuousWater(p+normal*.5f)??math.lerp(reach.StartLevel,reach.EndLevel,.5f);
                        Bank(p,side,level,reach.BodyId);
                    }
                }
            }
            else
            {
                var delta=reach.End-reach.Start;var normal=math.normalizesafe(new float3(-delta.z,0,delta.x));
                var steps=Math.Max(1,(int)Math.Ceiling(math.length(delta.xz)/12));var halfWidth=Math.Max(reach.HalfWidth,water.WaterCellSize);
                for(var j=0;j<steps;j++)for(var side=-1;side<=1;side+=2)
                {var t=(j+.5f)/steps;Bank(math.lerp(reach.Start,reach.End,t)+normal*side*halfWidth,normal*side,math.lerp(reach.StartLevel,reach.EndLevel,t),reach.BodyId);}
            }
        }
        log.Count("Riparian trees planned",bankTrees);log.Count("Riparian shrubs planned",bankShrubs);
        // Distributed canopy first, then infill, then bushes. Stopping early
        // leaves useful woodland across the whole map, not a completed centre.
        var ordered=output.OrderBy(p=>p.Form==VegetationForm.Shrub?2:p.Seed%4==0?0:1).ThenBy(p=>p.Seed).ToArray();
        log.Diagnostic("VEGETATION WORKLOAD canopyDensityRetained=true; inferredShrubFractionOfPrevious=0.25; explicitShrubsRetained=true; order=distributed-canopy-infill-shrubs");
        for(var form=0;form<formCounts.Length;form++)if(formCounts[form]>0)log.Count("Vegetation planned "+(VegetationForm)form,formCounts[form]);
        if(log.DiagnosticsEnabled)log.Diagnostic($"VEGETATION METADATA entries={metadata.Count}; compactCandidates={output.Count}; continuousChannelsExcluded=true; riparianSitesInferred=true");
        foreach(var reason in exclusions.OrderBy(p=>p.Key))log.Diagnostic($"VEGETATION EXCLUSION reason={reason.Key}; count={reason.Value}; policyVersion=8");
        log.Diagnostic($"VEGETATION POLICY version=8; candidates={output.Count}; rejectedFinalMasks={rejected}; habitats={habitats.Length}; vanillaOnly=true; climateAvailable={climate!=null}; annualPrecipitationMm={annualRain:R}; warmAridPrior={warmArid}; uncapped=true; order=distributed-canopy-infill-shrubs; seedExcludesHeight=true; riparianTrees={bankTrees}; riparianShrubs={bankShrubs}; dryLandScatterAdded=false; explicitTaxonOverridesClimate=true; speciesNotGuaranteed=true");
        return new VegetationPlan(ordered,metadata.ToArray());
    }

}
