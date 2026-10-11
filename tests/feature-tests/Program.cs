using ImportEarthLocation.Core;
using ImportEarthLocation.Geo;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

static class Program
{
    static void Check(bool value, string message) { if(!value) throw new Exception(message); }
    static int Main()
    {
        try { Run(); return 0; }
        catch(Exception e) { Console.Error.WriteLine(e); return 1; }
    }
    static void Run()
    {
#if WATER_SMOKE_ONLY
        WaterTests.Run();
        TerrainAcquisitionChecks();
#else
        var log = new ImportLog(); var request = new ImportRequest(43.72,15.82,MapSizeProfile.MapExt57,false);
#if !SURFACE_SMOKE_ONLY
        ArchiveTests.Run();
        UiPackageTests.Run();
        Check(RailPrefabPolicy.CanUse("Twoway Train Track", 1), "Verified vanilla bidirectional single track rejected");
        Check(!RailPrefabPolicy.CanUse("Oneway Train Track", -1), "Unknown lane count must permit explicit vanilla fallback");
        Check(RailPrefabPolicy.CanUse("Twoway Train Track", 2)&&RailPrefabPolicy.CanUse("Double Train Track",2), "Verified base double-track corridor rejected");
        Check(!RailPrefabPolicy.CanUse("Twoway Train Track", 0), "Verified empty track accepted");
        foreach(var name in new[]{"Twoway Train Track - Station Side", "LiftBridge01", "Custom Train Track"})
            Check(!RailPrefabPolicy.CanUse(name, 1) && !RailPrefabPolicy.CanUse(name, -1), "Special/custom track accepted: " + name);
        var baseline=Newtonsoft.Json.Linq.JObject.Parse("{\"prefab\":\"Climate\",\"windX\":0.5}");
        Check(DiagnosticEvidence.Compare(true,baseline,baseline.DeepClone())=="matching-native-state","Matching native readbacks not recognized");
        Check(DiagnosticEvidence.Compare(false,baseline,baseline)=="unverified","Skipped operation claimed as applied persistence");
        Check(DiagnosticEvidence.Compare(true,baseline,null)=="unverified"&&DiagnosticEvidence.Compare(true,null,baseline)=="unverified","Missing readback claimed as persistence");
        Check(DiagnosticEvidence.Compare(true,baseline,Newtonsoft.Json.Linq.JValue.CreateNull())=="unverified","Null native readback accepted");
        Check(DiagnosticEvidence.Compare(true,baseline,Newtonsoft.Json.Linq.JObject.Parse("{\"prefab\":\"Other\",\"windX\":0.5}"))=="native-state-differs","Changed native prefab not detected");
        var expected=new Newtonsoft.Json.Linq.JArray(baseline.DeepClone());
        Check(DiagnosticEvidence.AllExpectedPresent(expected,new Newtonsoft.Json.Linq.JArray(baseline.DeepClone())),"Created connection readback not recognized");
        Check(!DiagnosticEvidence.AllExpectedPresent(expected,new Newtonsoft.Json.Linq.JArray())&&!DiagnosticEvidence.AllExpectedPresent(new Newtonsoft.Json.Linq.JArray(),new Newtonsoft.Json.Linq.JArray()),"Empty air state claimed as successful creation");
        Check(!DiagnosticEvidence.AllExpectedPresent(expected,null),"Unavailable air snapshot claimed as successful creation");
        var cities = AirPlanner.Select(request,log);
        Check(cities.Count == 20 && cities.SequenceEqual(AirPlanner.Select(request,log)), "Bundled destinations not deterministic/bounded");
        Check(cities.All(c=>c.DistanceKm>0 && c.Bearing>=0 && c.Bearing<360 && !string.IsNullOrWhiteSpace(c.Name)), "Invalid destination record");
        Check(cities.Zip(cities.Skip(1),(a,b)=>a.DistanceKm<=b.DistanceKm).All(x=>x), "Cities not nearest-first");
        var directions = Enumerable.Range(0,8).Select(i=>new AirDestination("City"+i,100+i,i*45,float3.zero)).ToArray();
        var placed = AirPlanner.Place(directions,1000,2000);
        Check(placed.Count==8 && placed[0].Position.z==1000 && placed[2].Position.x==1000 && placed[4].Position.z==-1000 && placed[6].Position.x==-1000, "Native north-up perimeter mapping wrong");
        Check(placed.All(p=>Math.Abs(p.Position.x)==1000 || Math.Abs(p.Position.z)==1000), "Connection not on continuous square perimeter");
        var congested = new[]{new AirDestination("nearest",100,44,float3.zero),new AirDestination("crowded",101,46,float3.zero),new AirDestination("other",200,180,float3.zero)};
        var reduced = AirPlanner.Place(congested,1000,2000);
        Check(reduced.Count==2 && reduced[0].Name=="nearest", "Corner congestion moved/kept wrong destination");
        Check(AirPlanner.Place(new[]{directions[0] with{Bearing=double.NaN}},1000,2000).Count==0, "Invalid bearing accepted");
        var speed=Enumerable.Repeat(5f,12).ToArray();
        var north=ClimateMath.PrevailingWind(new float[12],speed);
        Check(Math.Abs(north.east)<.001 && north.north==-5 && north.speed==5, "North FROM wind not converted southward");
        var east=ClimateMath.PrevailingWind(Enumerable.Repeat(90f,12).ToArray(),speed);
        Check(east.east==-5 && Math.Abs(east.north)<.001, "East FROM wind not converted westward");
        var wrapped=ClimateMath.PrevailingWind(Enumerable.Range(0,12).Select(i=>i%2==0?359f:1f).ToArray(),speed);
        Check(wrapped.north < -4.99 && Math.Abs(wrapped.east)<.01, "Circular wind average wraps incorrectly");
        try { ClimateMath.PrevailingWind(new float[12],Enumerable.Repeat(float.NaN,12).ToArray()); throw new Exception("NaN wind accepted"); } catch(ArgumentException){}
#endif
        float3[] Ring(float left,float bottom,float right,float top)=>new[]{new float3(left,0,bottom),new float3(right,0,bottom),new float3(right,0,top),new float3(left,0,top),new float3(left,0,bottom)};
        var grass=new GeographicPolygon(1,new Dictionary<string,string>{{"natural","grassland"}},new(){Ring(-200,-200,200,200)},new(){Ring(-30,-30,30,30)});
        var sand=new GeographicPolygon(2,new Dictionary<string,string>{{"natural","sand"}},new(){Ring(100,100,180,180)},new());
        Check(grass.Contains(new float3(100,0,0)) && !grass.Contains(float3.zero), "Surface polygon hole filled");
        var small=request with{Latitude=0,Longitude=0,MapSize=new("test","test",600,false)};
        var terrain=new TerrainBuilder.Result(Enumerable.Repeat((ushort)32768,256).ToArray(),100,40,0,100);
        var mask=SurfacePlanner.Prepare(new[]{grass,sand},small,terrain,log,default);
        int Sample(float x,float z)=>(int)((z+300)/600*2048)*2048+(int)((x+300)/600*2048);
        Check(mask.Material[Sample(140,140)]==(byte)GroundMaterial.Sand && mask.Material[Sample(80,0)]==0 && mask.Material[Sample(0,0)]==0, "Explicit sand/vegetated Default/hole failed");
        Check(mask.Intensity[Sample(179,140)]<mask.Intensity[Sample(140,140)], "Geographic feathering absent");
        var flooded=SurfacePlanner.Prepare(new[]{grass,sand},small,terrain with{SeaLevelMeters=60},log,default);
        Check(flooded.Material[Sample(140,140)]==(byte)GroundMaterial.Sand&&flooded.Material[Sample(80,0)]==0, "Submerged sand lost or underwater vegetation inferred");
        // Update the existing surface smoke for exclusive brush semantics and richer evidence.
        var reversed=SurfacePlanner.Prepare(new[]{sand,grass},small,terrain,log,default);
        Check(mask.Material.SequenceEqual(reversed.Material)&&mask.Intensity.SequenceEqual(reversed.Intensity),"Surface order changed classification/intensity");
        var conflicting=sand with{Tags=new Dictionary<string,string>{{"natural","beach"},{"surface","gravel"}}};
        var explicitSand=sand with{Tags=new Dictionary<string,string>{{"natural","beach"},{"surface","sand"}}};
        var sameId=SurfacePlanner.Prepare(new[]{explicitSand,conflicting},small,terrain,log,default,resolution:256);
        var sameIdReversed=SurfacePlanner.Prepare(new[]{conflicting,explicitSand},small,terrain,log,default,resolution:256);
        Check(sameId.Material.SequenceEqual(sameIdReversed.Material)&&sameId.Intensity.SequenceEqual(sameIdReversed.Intensity),"Coincident way/relation IDs make physical cover depend on paint order");
        Check(SurfacePaint.VanillaMaterial("Terrain Material Extra 1",4)==GroundMaterial.DryGrass&&SurfacePaint.VanillaMaterial("Terrain Material Extra 2",5)==GroundMaterial.Grass&&SurfacePaint.VanillaMaterial("Terrain Material Extra 3",6)==GroundMaterial.Dirt&&SurfacePaint.VanillaMaterial("Terrain Material Extra 4",7)==GroundMaterial.Sand,"Vanilla paint identities/channels wrong");
        Check(SurfacePaint.VanillaMaterial("Terrain Material Extra 4",4)==GroundMaterial.Base,"Mismatched native brush type accepted");
        SurfacePaint.Validate(mask);
        var paintSample=SurfacePaint.Sample(mask,140,140);
        Check(paintSample.material==GroundMaterial.Sand&&paintSample.intensity>0,"Exclusive paint sampling lost mapped brush");
        GeographicPolygon Cover(long id,params (string key,string value)[] tags)=>new(id,tags.ToDictionary(t=>t.key,t=>t.value),new(){Ring(-200,-200,200,200)},new());
        var scrub=SurfacePlanner.Prepare(new[]{Cover(3,("natural","scrub"))},small,terrain,log,default,resolution:256);
        var woods=SurfacePlanner.Prepare(new[]{Cover(3,("natural","wood"))},small,terrain,log,default,resolution:256);
        var rocks=SurfacePlanner.Prepare(new[]{Cover(3,("natural","beach"),("surface","gravel"))},small,terrain with{SeaLevelMeters=60},log,default,resolution:256);
        var track=SurfacePlanner.Prepare(new[]{Cover(3,("highway","track"),("surface","sand"),("area","yes"))},small,terrain,log,default,resolution:256);
        Check(scrub.Material[128*256+128]==(byte)GroundMaterial.DryGrass&&woods.Material.All(m=>m==0),"Scrub not Dry or woodland automatically Lush");
        Check(rocks.Material[128*256+128]==(byte)GroundMaterial.Dirt&&track.Material.All(m=>m==0),"Rocky beach lost underwater or track surface painted enclosing landscape");
        // Existing surface smoke: a broad, coherent sand ecotone must neither
        // invent sand outside its footprint nor erase small beaches/map edges.
        var transitionRequest=small with{MapSize=new("transition","transition",2000,false)};
        var sandHalf=new GeographicPolygon(30,new Dictionary<string,string>{{"natural","sand"}},new(){Ring(-1200,-1200,0,1200)},new());
        var grassHalf=Cover(31,("natural","grassland")) with{Outer=new(){Ring(0,-1200,1200,1200)}};
        int TransitionSample(float x,float z)=>(int)((z+1000)/2000*256)*256+(int)((x+1000)/2000*256);
        var naturalEdge=SurfacePlanner.Prepare(new[]{sandHalf,grassHalf},transitionRequest,terrain,log,default,resolution:256);
        Check(naturalEdge.Intensity[TransitionSample(-240,0)]>220&&naturalEdge.Intensity[TransitionSample(-36,0)]<190&&naturalEdge.Intensity[TransitionSample(-36,0)]>0,"Sand transition stayed hard or lost solid interior");
        var edgeVariation=Enumerable.Range(0,17).Select(k=>naturalEdge.Intensity[TransitionSample(-36,-600+k*75)]).ToArray();
        Check(edgeVariation.Max()-edgeVariation.Min()>15,"Natural sand edge is a uniform-width contour");
        Check(naturalEdge.Material[TransitionSample(20,0)]==0&&naturalEdge.Intensity[TransitionSample(20,0)]==0,"Ecotone expanded sand into non-sand ground");
        var allSand=SurfacePlanner.Prepare(new[]{sandHalf with{Outer=new(){Ring(-1200,-1200,1200,1200)}}},transitionRequest,terrain,log,default,resolution:256);
        Check(allSand.Intensity[0]>220&&allSand.Intensity[255]>220&&allSand.Intensity[^1]>220,"Map crop created an artificial grass frame inside sand");
        var narrowBeach=SurfacePlanner.Prepare(new[]{sandHalf with{Tags=new Dictionary<string,string>{{"natural","beach"}},Outer=new(){Ring(-10,-700,10,700)}}},transitionRequest,terrain,log,default,resolution:256);
        Check(narrowBeach.Material[TransitionSample(0,0)]==(byte)GroundMaterial.Sand&&narrowBeach.Intensity[TransitionSample(0,0)]>220&&narrowBeach.Material[TransitionSample(20,0)]==0,"Size-aware transition erased or expanded a narrow beach");
        // Extend this existing surface smoke: sand-backed flat desert expands,
        // but a beach or flatness alone does not establish a regional desert.
        var desertRequest=small with{MapSize=new("desert","desert",6000,false)};
        var inland=new GeographicPolygon(20,new Dictionary<string,string>{{"natural","sand"}},new(){Ring(-2600,-700,-1500,700)},new());
        int DesertSample(float x,float z)=>(int)((z+3000)/6000*256)*256+(int)((x+3000)/6000*256);
        var protectedCover=new[]{Cover(21,("natural","wood")),Cover(22,("natural","bare_rock")) with{Outer=new(){Ring(600,-200,1000,200)}},Cover(23,("natural","water")) with{Outer=new(){Ring(1400,-200,1800,200)}},Cover(24,("landuse","residential")) with{Outer=new(){Ring(2200,-200,2600,200)}}};
        var desert=SurfacePlanner.Prepare(new[]{inland}.Concat(protectedCover).ToArray(),desertRequest,terrain,log,default,resolution:256);
        Check(desert.Material[DesertSample(0,1800)]==(byte)GroundMaterial.Sand&&desert.Intensity[DesertSample(0,1800)]>200,"Flat inland-sand-backed desert remained grassy Default");
        Check(desert.Material[DesertSample(0,0)]!=(byte)GroundMaterial.Sand&&desert.Material[DesertSample(800,0)]==(byte)GroundMaterial.Dirt&&desert.Material[DesertSample(1600,0)]==0&&desert.Material[DesertSample(2400,0)]==0,"Desert overwrote woodland, rock, water or built-up cover");
        var beachOnly=SurfacePlanner.Prepare(new[]{inland with{Tags=new Dictionary<string,string>{{"natural","beach"},{"surface","sand"}}}},desertRequest,terrain,log,default,resolution:256);
        var noSand=SurfacePlanner.Prepare(Array.Empty<GeographicPolygon>(),desertRequest,terrain,log,default,resolution:256);
        var floodedDesert=SurfacePlanner.Prepare(new[]{inland},desertRequest,terrain with{SeaLevelMeters=60},log,default,resolution:256);
        Check(beachOnly.Material[DesertSample(0,1800)]==0&&noSand.Material.All(m=>m==0)&&floodedDesert.Material[DesertSample(0,1800)]==0,"Beach-only, no-sand or submerged terrain triggered desert expansion");
        var riverBounds=GeoMath.BoundsFromCenter(0,0,500);
        var riverData=new OsmDataset{Nodes=new Dictionary<long,OsmNode>{{11,new(11,riverBounds.South,0)},{12,new(12,riverBounds.North,0)}},Ways=new Dictionary<long,OsmWay>{{10,new(10,new long[]{11,12},new Dictionary<string,string>{{"waterway","river"}})}}};
        var wet=SurfacePlanner.Prepare(new[]{grass},small,terrain,log,default,riverData,resolution:256);
        var seasonalWay=riverData.Ways[10] with{Tags=new Dictionary<string,string>{{"waterway","river"},{"intermittent","yes"}}};
        var intermittent=SurfacePlanner.Prepare(new[]{grass},small,terrain,log,default,new OsmDataset{Nodes=riverData.Nodes,Ways=new Dictionary<long,OsmWay>{{10,seasonalWay}}},resolution:256);
        Check(wet.Material[128*256+128]==(byte)GroundMaterial.Grass&&intermittent.Material.All(m=>m!=(byte)GroundMaterial.Grass),"Permanent/seasonal moisture semantics failed");
        var moistEdge=SurfacePlanner.Prepare(new[]{sandHalf,grassHalf},transitionRequest,terrain,log,default,riverData,resolution:256);
        Check(moistEdge.Intensity[TransitionSample(-36,0)]<naturalEdge.Intensity[TransitionSample(-36,0)],"Moist sand edge did not gain a broader transition");
        var seasonal=SurfacePlanner.Prepare(new[]{grass},small,terrain,log,default,new OsmDataset{Nodes=riverData.Nodes,Ways=new Dictionary<long,OsmWay>{{10,seasonalWay with{Tags=new Dictionary<string,string>{{"waterway","river"},{"seasonal","wet_season"}}}}}},resolution:256);
        Check(seasonal.Material.SequenceEqual(intermittent.Material)&&seasonal.Intensity.SequenceEqual(intermittent.Intensity),"Named seasonal drainage treated as permanent water");
        foreach(var surfaceMask in new[]{mask,flooded,scrub,wet,intermittent})Check(surfaceMask.Material.Zip(surfaceMask.Intensity,(m,s)=>m!=0||s==0).All(v=>v),"Default has a painted intensity");
        var heights=Enumerable.Range(0,256).Select(i=>(ushort)Math.Round((50+70*Math.Exp(-Math.Pow((i%16-7.5)/2.5,2)))/200*65535)).ToArray();
        var ridgeTerrain=terrain with{Heightmap=heights,HeightScaleMeters=200,SeaLevelMeters=0};
        var ridge=SurfacePlanner.Prepare(Array.Empty<GeographicPolygon>(),small,ridgeTerrain,log,default,resolution:256);
        var shifted=SurfacePlanner.Prepare(Array.Empty<GeographicPolygon>(),small,ridgeTerrain with{Heightmap=heights.Select(h=>(ushort)(h+16384)).ToArray(),SeaLevelMeters=50},log,default,resolution:256);
        Check(ridge.Material.Contains((byte)GroundMaterial.DryGrass)&&ridge.Material.SequenceEqual(shifted.Material),"Relative terrain exposure depends on absolute elevation");
        using(var cancelled=new CancellationTokenSource()){cancelled.Cancel();try{SurfacePlanner.Prepare(Array.Empty<GeographicPolygon>(),small,terrain,log,cancelled.Token,resolution:256);throw new Exception("Surface cancellation ignored");}catch(OperationCanceledException){}}
        const int zoom=12; var tile=(int)GeoMath.LongitudeToTileX(15.82,zoom); var tileY=(int)GeoMath.LatitudeToTileY(43.72,zoom);
        var data=new OsmDataset{Nodes=new Dictionary<long,OsmNode>{{1,new(1,43.72,15.82)},{2,new(2,43.721,15.821)}},Ways=new Dictionary<long,OsmWay>{{1,new(1,new long[]{1,2},new Dictionary<string,string>{{"natural","coastline"}})}}};
        var guard=TerrainGeographyGuard.Build(data,zoom,tile,tileY,256,256,default);
        Check(guard.Any(x=>x) && guard.Any(x=>!x), "Mapped coast guard not localized");
#if !SURFACE_SMOKE_ONLY
        WaterTests.Run();
        TerrainAcquisitionChecks();
        EnvironmentChecks.Run();
        Console.WriteLine("PASS: diagnostic evidence match/mismatch/missing/skipped/null guards, offline deterministic cities, nearest-first/perimeter/corner spacing, north-up bearings, circular FROM wind, exclusive surface/native brush mapping, priority/ID ties/holes/underwater support/seasonal moisture/relative exposure/feathering, sand-backed flat desert and mapped non-sand protection, terrain-aware irregular sand ecotones, narrow beach cores and crop-edge preservation.");
#else
        Console.WriteLine("PASS: existing surface smoke subset: native brush identities/exclusive sampling, deterministic priority/ID ties, holes/underwater/seasonal moisture/relative exposure, sand-backed desert/non-sand protection, irregular terrain-aware ecotones, narrow beach cores and crop-edge preservation. No game rendering or unrelated feature-suite acceptance claimed.");
#endif
#endif
    }

    // Exercise the real async planner up to its first cancelled download gate.
    // No duplicate footprint algorithm, network access, game build or deployment.
    static void TerrainAcquisitionChecks()
    {
        var root=Path.Combine(Path.GetTempPath(),"iel-dem-plan-"+Guid.NewGuid().ToString("N"));
        try
        {
            var log=new ImportLog();
            var client=new ImportEarthLocation.Data.TerrainTileClient(new ImportEarthLocation.Data.CacheStore(root,log),log);
            using var cancelled=new CancellationTokenSource();cancelled.Cancel();
            var bounds=GeoMath.BoundsFromCenter(54.235921,-4.514767,57344);
            try { client.DownloadAsync(bounds,57344,cancelled.Token).GetAwaiter().GetResult();throw new Exception("Cancelled DEM request completed"); }
            catch(OperationCanceledException) { }
            var diagnostic=log.Lines.Single(s=>s.StartsWith("DEM PLAN ",StringComparison.Ordinal));
            Check(diagnostic.Contains("preferredTiles=121;")&&diagnostic.Contains("selectedTiles=121;")&&diagnostic.Contains("zoomReduced=False;"),"Valid Isle of Man footprint was rejected or unnecessarily downsampled: "+diagnostic);
            Check(!Directory.Exists(root)||!Directory.EnumerateFiles(root,"*",SearchOption.AllDirectories).Any(),"Cancelled footprint check acquired or cached tile bytes");
            try { client.DownloadAsync(new GeoBounds(1,2,0,3),57344,cancelled.Token).GetAwaiter().GetResult();throw new Exception("Inverted DEM bounds accepted"); }
            catch(PreparationDataException error) { Check(error.Issue==PreparationDataIssue.ElevationUnavailable,"Bad DEM bounds lost typed data outcome"); }
            Console.WriteLine("PASS: real DEM planner retains the captured 121-tile footprint, honors cancellation before acquisition and rejects inverted bounds; no HTTP or elevation decoding exercised.");
        }
        finally { if(Directory.Exists(root))Directory.Delete(root,true); }
    }
}
namespace ImportEarthLocation { public static class Mod { public const string Id="FeatureSmoke";public const string Version="test"; } }
namespace ImportEarthLocation.Core { public sealed class ImportLog { public readonly List<string> Lines=new(); public void Info(string s)=>Lines.Add(s); public void Warn(string s)=>Lines.Add(s); public void Diagnostic(string s)=>Lines.Add(s); public void Count(string s,int count=1){} public void Time(string s,double seconds){} } }
namespace ImportEarthLocation.Data { public static class LocationServices { public static bool Valid(double lat,double lon)=>double.IsFinite(lat)&&double.IsFinite(lon)&&Math.Abs(lat)<=90&&Math.Abs(lon)<=180; } }
namespace ImportEarthLocation.Processing { public static class TerrainBuilder { public const int Resolution=16; public sealed record Result(ushort[] Heightmap,float HeightScaleMeters,float SeaLevelMeters,float MinimumElevation,float MaximumElevation); } }
