#if !SURFACE_SMOKE_ONLY
using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Newtonsoft.Json.Linq;
using Unity.Mathematics;

static class ClimateLandscapeTests
{
    static void Check(bool value,string message){if(!value)throw new Exception(message);}
    public static ClimateNormals Climate(string city)
    {
        var json=JObject.Parse(File.ReadAllText(Path.Combine(AppContext.BaseDirectory,"fixtures","climate-"+city+".json")))["properties"]!["parameter"]!;
        var months=new[]{"JAN","FEB","MAR","APR","MAY","JUN","JUL","AUG","SEP","OCT","NOV","DEC"};
        return new(months.Select(m=>(float)json["T2M"]![m]!).ToArray(),months.Select(m=>(float)json["PRECTOTCORR"]![m]!).ToArray(),new float[12],new float[12],0,"captured NASA normals");
    }
    public static void Run()
    {
        var request=new ImportRequest(0,0,new("climate","climate",4000,false),false);
        var terrain=new TerrainBuilder.Result(Enumerable.Repeat((ushort)32768,129*129).ToArray(),200,0,0,200);
        GeographicPolygon Box(long id,float x,float z,params (string,string)[] tags)=>new(id,tags.ToDictionary(t=>t.Item1,t=>t.Item2),
            new(){new[]{new float3(x,0,z),new float3(x+700,0,z),new float3(x+700,0,z+700),new float3(x,0,z+700)}},new());
        var masks=new[]{Box(1,900,700,("natural","wood")),Box(2,-1700,700,("leisure","park")),Box(3,900,-1500,("surface","asphalt"))};
        var data=new OsmDataset{Nodes=new Dictionary<long,OsmNode>{{1,new(1,-2000/6378137d*180/Math.PI,0)},{2,new(2,2000/6378137d*180/Math.PI,0)}},
            Ways=new Dictionary<long,OsmWay>{{10,new(10,new long[]{1,2},new Dictionary<string,string>{{"waterway","river"}})}}};
        SurfaceMask Plan(ClimateNormals? climate)=>SurfacePlanner.Prepare(masks,request,terrain,new ImportLog(),default,data,128,climate);
        var baseline=Plan(null);var boise=Plan(Climate("boise"));
        foreach(var city in new[]{"dc","marseille"})
        {var p=Plan(Climate(city));Check(p.Material.SequenceEqual(baseline.Material)&&p.Intensity.SequenceEqual(baseline.Intensity),city+" changed outside the new climate gate");}
        int Pixel(float x,float z)=>(int)((z/4000+.5)*128)*128+(int)((x/4000+.5)*128);
        Check(boise.Material[Pixel(1500,0)]==(byte)GroundMaterial.DryGrass&&boise.Intensity[Pixel(1500,0)]>50,"Boise's dry open land did not receive a substantial dry-grass treatment");
        Check(boise.Material[Pixel(0,0)]==(byte)GroundMaterial.Grass,"Permanent freshwater lost its lush near-bank surface");
        foreach(var p in new[]{new float2(1250,1050),new float2(-1350,1050),new float2(1250,-1150)})
        {var i=Pixel(p.x,p.y);Check(boise.Material[i]==baseline.Material[i]&&boise.Intensity[i]==baseline.Intensity[i],"Woodland, park or paving was overridden by the regional climate bias");}
        foreach(var invalid in new[]{Climate("boise") with{Precipitation=new float[0]},Climate("boise") with{Temperatures=Enumerable.Repeat(float.NaN,12).ToArray()}})
        {var p=Plan(invalid);Check(p.Material.SequenceEqual(baseline.Material)&&p.Intensity.SequenceEqual(baseline.Intensity),"Invalid climate inferred a regional dry-grass treatment");}
        var humid=Climate("boise") with{Precipitation=Enumerable.Repeat(3f,12).ToArray()};
        Check(Plan(humid).Material.SequenceEqual(baseline.Material),"A wet east-coast-like climate received the dry regional treatment");
        var treeRequest=request with{MapSize=new("slope","slope",400,false)};
        TerrainBuilder.Result Plane(float slope,bool ledge=false)
        {
            var h=new ushort[129*129];for(var z=0;z<129;z++)for(var x=0;x<129;x++)
            {var px=(x/128f-.5f)*400;var y=500+slope*px+(ledge&&px<0?-14:0);h[z*129+x]=(ushort)Math.Round(y/1000*65535);}
            return new(h,1000,0,0,1000);
        }
        var point=new float3(0,0,0);var empty=new WaterPlan(false,0,0,0,"");
        VegetationPlan Trees(OsmDataset source,TerrainBuilder.Result dem)=>VegetationPlanner.Prepare(source,new[]{point},treeRequest,dem,Array.Empty<PreparedSegment>(),Array.Empty<GeographicPolygon>(),empty,null,new ImportLog(),default);
        var explicitTree=new OsmDataset{Nodes=new Dictionary<long,OsmNode>{{1,new(1,0,0)}},NodeTags=new Dictionary<long,IReadOnlyDictionary<string,string>>{{1,new Dictionary<string,string>{{"natural","tree"}}}}};
        Check(Trees(new(),Plane(1.2f)).Count==1,"A smooth 50-degree hillside still rejected an ordinary tree");
        Check(Trees(explicitTree,Plane(1.65f)).Count==1,"A mapped tree on a smooth near-59-degree hillside was rejected");
        Check(Trees(explicitTree,Plane(1.9f)).Count==0,"The slope policy admitted a tree beyond its cliff limit");
        Check(Trees(new(),Plane(0,true)).Count==0,"Central differences concealed a large local terrain ledge");
        Console.WriteLine("PASS: captured Boise dry-grass climate, byte-identical DC/Marseille gate, freshwater/woodland/park/paving protection, missing-climate fallback, liberal hillside trees and cliff/ledge guards.");
    }
}
#endif
