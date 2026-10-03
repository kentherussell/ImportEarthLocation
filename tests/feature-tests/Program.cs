using ImportEarthLocation.Core;
using ImportEarthLocation.Geo;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

static class Program
{
    static void Check(bool value, string message) { if(!value) throw new Exception(message); }
    static void Main()
    {
        var log = new ImportLog(); var request = new ImportRequest(43.72,15.82,MapSizeProfile.MapExt57,false);
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
        float3[] Ring(float left,float bottom,float right,float top)=>new[]{new float3(left,0,bottom),new float3(right,0,bottom),new float3(right,0,top),new float3(left,0,top),new float3(left,0,bottom)};
        var grass=new GeographicPolygon(1,new Dictionary<string,string>{{"natural","grassland"}},new(){Ring(-200,-200,200,200)},new(){Ring(-30,-30,30,30)});
        var sand=new GeographicPolygon(2,new Dictionary<string,string>{{"natural","sand"}},new(){Ring(100,100,180,180)},new());
        Check(grass.Contains(new float3(100,0,0)) && !grass.Contains(float3.zero), "Surface polygon hole filled");
        var small=request with{Latitude=0,Longitude=0,MapSize=new("test","test",600,false)};
        var terrain=new TerrainBuilder.Result(Enumerable.Repeat((ushort)32768,256).ToArray(),100,40,0,100);
        var mask=SurfacePlanner.Prepare(new[]{grass,sand},small,terrain,log,default);
        int Sample(float x,float z)=>(int)((z+300)/600*2048)*2048+(int)((x+300)/600*2048);
        Check(mask.Material[Sample(140,140)]==(byte)GroundMaterial.Sand && mask.Material[Sample(80,0)]==(byte)GroundMaterial.Grass && mask.Material[Sample(0,0)]==0, "Explicit cover priority/hole/base failed");
        Check(mask.Intensity[Sample(199,80)]<mask.Intensity[Sample(80,80)], "Geographic feathering absent");
        var flooded=SurfacePlanner.Prepare(new[]{grass},small,terrain with{SeaLevelMeters=60},log,default);
        Check(flooded.Material.All(m=>m==0), "Underwater surface painted");
        const int zoom=12; var tile=(int)GeoMath.LongitudeToTileX(15.82,zoom); var tileY=(int)GeoMath.LatitudeToTileY(43.72,zoom);
        var data=new OsmDataset{Nodes=new Dictionary<long,OsmNode>{{1,new(1,43.72,15.82)},{2,new(2,43.721,15.821)}},Ways=new Dictionary<long,OsmWay>{{1,new(1,new long[]{1,2},new Dictionary<string,string>{{"natural","coastline"}})}}};
        var guard=TerrainGeographyGuard.Build(data,zoom,tile,tileY,256,256,default);
        Check(guard.Any(x=>x) && guard.Any(x=>!x), "Mapped coast guard not localized");
        Console.WriteLine("PASS: offline deterministic cities, nearest-first/perimeter/corner spacing, north-up bearings, circular FROM wind, surface priority/holes/base/water/feathering, mapped terrain protection.");
    }
}
namespace ImportEarthLocation.Core { public sealed class ImportLog { public void Info(string s){} public void Warn(string s){} public void Count(string s,int count=1){} } }
namespace ImportEarthLocation.Data { public static class LocationServices { public static bool Valid(double lat,double lon)=>double.IsFinite(lat)&&double.IsFinite(lon)&&Math.Abs(lat)<=90&&Math.Abs(lon)<=180; } }
namespace ImportEarthLocation.Processing { public static class TerrainBuilder { public const int Resolution=16; public sealed record Result(ushort[] Heightmap,float HeightScaleMeters,float SeaLevelMeters,float MinimumElevation,float MaximumElevation); } }
