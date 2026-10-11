using ImportEarthLocation.Core;
using ImportEarthLocation.Geo;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

static class EnvironmentChecks
{
    static void Check(bool b,string message){if(!b)throw new Exception(message);}
    public static void Run()
    {
        var dry=new ClimateNormals(Enumerable.Repeat(25f,12).ToArray(),new float[12],new float[12],new float[12],0,"existing-fixture");
        Check(ClimateMath.AnnualPrecipitation(Enumerable.Repeat(2f,12).ToArray())==730,"Mean daily precipitation was mistaken for monthly totals");
        Check(float.IsNaN(ClimateMath.AnnualPrecipitation(new[]{float.NaN})),"Missing rainfall should not infer aridity");
        var request=new ImportRequest(25.7,32.65,new("large","large",57344,false),false);
        var terrain=new TerrainBuilder.Result(Enumerable.Repeat((ushort)32768,256).ToArray(),100,0,0,100);
        float3[] Ring(float x,float z,float width)=>new[]{new float3(x,0,z),new float3(x+width,0,z),new float3(x+width,0,z+width),new float3(x,0,z+width)};
        var sand=Enumerable.Range(0,6).Select(i=>new GeographicPolygon(i+1,new Dictionary<string,string>{{"natural","sand"}},new(){Ring(-16000+i*6000,-500,1000)},new())).ToArray();
        var woods=new GeographicPolygon(20,new Dictionary<string,string>{{"natural","wood"}},new(){Ring(-500,-500,1000)},new());
        var masks=sand.Concat(new[]{woods}).ToArray();var log=new ImportLog();
        var arid=SurfacePlanner.Prepare(masks,request,terrain,log,default,resolution:256,climate:dry);
        var rainy=SurfacePlanner.Prepare(masks,request,terrain,log,default,resolution:256,climate:dry with{Precipitation=Enumerable.Repeat(3f,12).ToArray()});
        int Pixel(float x,float z)=>(int)((z/57344+.5)*256)*256+(int)((x/57344+.5)*256);
        Check(arid.Material[Pixel(22000,10000)]==(byte)GroundMaterial.Sand&&arid.Intensity[Pixel(22000,10000)]>200,"Large-map sand evidence was diluted and flat arid desert stayed grassy");
        Check(arid.Material[Pixel(0,0)]!=(byte)GroundMaterial.Sand&&rainy.Material[Pixel(22000,10000)]==0,"Climate overwrote mapped woodland or invented a wet-region desert");
        var beaches=SurfacePlanner.Prepare(sand.Select(p=>p with{Tags=new Dictionary<string,string>{{"natural","beach"}}}).ToArray(),request,terrain,log,default,resolution:256,climate:dry);
        Check(beaches.Material[Pixel(22000,10000)]==0,"Beach evidence became an arid-map desert trigger");
        request=request with{Latitude=0,Longitude=0,MapSize=new("vegetation","vegetation",1000,false)};
        var positions=new[]{new float3(-100,0,0),new float3(100,0,0),new float3(0,0,150)};
        var nodes=positions.Select((p,i)=>new OsmNode(i+1,p.z/6378137d*180/Math.PI,p.x/6378137d*180/Math.PI)).ToDictionary(n=>n.Id);
        nodes[4]=new OsmNode(4,-150/6378137d*180/Math.PI,0);
        var tags=new Dictionary<long,IReadOnlyDictionary<string,string>>{
            {1,new Dictionary<string,string>{{"natural","tree"},{"leaf_type","broadleaved"}}},
            {2,new Dictionary<string,string>{{"natural","tree"},{"genus","Quercus"}}},
            {3,new Dictionary<string,string>{{"natural","tree"},{"genus","Pinus"}}},
            {4,new Dictionary<string,string>{{"natural","shrub"}}}};
        var data=new OsmDataset{Nodes=nodes,NodeTags=tags};var emptyWater=new WaterPlan(false,0,0,0,"");
        IReadOnlyList<VegetationCandidate> Vegetation(ClimateNormals c,TerrainBuilder.Result dem)=>VegetationPlanner.Prepare(data,positions,request,dem,Array.Empty<PreparedSegment>(),Array.Empty<GeographicPolygon>(),emptyWater,c,log,default);
        var trees=Vegetation(dry,terrain);
        Check(trees.Count==4&&trees.Single(t=>t.SourceId==1).Form==VegetationForm.Palm&&trees.Single(t=>t.SourceId==2).Form==VegetationForm.Broadleaf&&trees.Single(t=>t.SourceId==3).Form==VegetationForm.Conifer&&trees.Single(t=>t.SourceId==4).Form==VegetationForm.Shrub,"Warm-arid palette ignored explicit taxon/shrub evidence or fabricated tree sites");
        Check(trees.Zip(trees.Skip(1),(a,b)=>math.lengthsq(a.Position.xz)<=math.lengthsq(b.Position.xz)).All(b=>b),"Uncapped center-out vegetation ordering changed");
        var raised=Vegetation(dry,terrain with{Heightmap=Enumerable.Repeat((ushort)40000,256).ToArray()});
        Check(trees.Select(t=>(t.SourceId,t.Seed,t.Form)).SequenceEqual(raised.Select(t=>(t.SourceId,t.Seed,t.Form))),"A terrain height change changed vegetation seed/palette");
        var wetTrees=Vegetation(dry with{Precipitation=Enumerable.Repeat(3f,12).ToArray()},terrain);
        Check(wetTrees.Single(t=>t.SourceId==1).Form==VegetationForm.Broadleaf,"Mean daily rainfall wrongly classified a wet climate as arid");
        Console.WriteLine("PASS: large-map corroborated desert, mapped non-sand/beach protection, rainfall units, arid existing-site palms, explicit oak/pine/shrub overrides, uncapped center-out ordering and height-independent seeds.");
    }
}
