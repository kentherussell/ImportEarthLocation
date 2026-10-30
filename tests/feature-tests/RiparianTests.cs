#if !SURFACE_SMOKE_ONLY
using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

static class RiparianTests
{
    static void Check(bool value,string message){if(!value)throw new Exception(message);}
    public static void Run()
    {
        const int n=129;const float scale=100;const double width=1600;
        var heights=new ushort[n*n];
        for(var z=0;z<n;z++)for(var x=0;x<n;x++)
        {var metres=(x/(double)(n-1)-.5)*width;heights[z*n+x]=(ushort)Math.Round((Math.Abs(metres)<=30?47:52)/scale*65535);}
        var terrain=new TerrainBuilder.Result(heights,scale,0,0,100);var request=new ImportRequest(0,0,new("bank","bank",width,false),false);
        var water=new WaterPlan(false,1,0,0,"fixture"){WaterCellSize=28,
            ReachStages=new[]{new WaterReachStage(10,20,new(0,0,-650),new(0,0,650),50,50,2,null)},
            Sources=new[]{new RiverSourcePlan(20,"stream","fixture",new(0,47,-600),10,3,52,new(0,0,-550)){BodyId=10,TargetSurfaceLevel=50}}};
        var mapped=WaterBodyPlanner.SurfaceSampler(water,Array.Empty<GeographicPolygon>());
        var expanded=WaterBodyPlanner.ConstructedSurfaceSampler(water,Array.Empty<GeographicPolygon>());
        Check(mapped(new(20,0,250))==null&&expanded(new(20,0,250))==50,"The continuous simulated channel footprint still follows only the thin mapped line");
        var data=new OsmDataset();var input=new[]{new float3(0,0,250),new float3(20,0,250),new float3(300,0,0)};
        ClimateNormals Climate(float rain,float temperature)=>new(Enumerable.Repeat(temperature,12).ToArray(),Enumerable.Repeat(rain,12).ToArray(),new float[12],new float[12],0,"fixture");
        VegetationPlan Plan(ClimateNormals climate)=>VegetationPlanner.Prepare(data,input,request,terrain,Array.Empty<PreparedSegment>(),Array.Empty<GeographicPolygon>(),water,climate,new ImportLog(),default);
        var wet=Plan(Climate(3,12));var arid=Plan(Climate(.2f,25));var coldArid=Plan(Climate(.2f,2));
        Check(wet.Any(p=>p.Position.x==300)&&wet.All(p=>Math.Abs(p.Position.x)>28),"Trees remained in the channel between water sources, or a dry inland tree disappeared");
        int Banks(VegetationPlan p)=>p.Count(c=>p.Metadata[c.MetadataIndex].Evidence.StartsWith("riparian-bank",StringComparison.Ordinal));
        Check(Banks(wet)>Banks(arid)&&Banks(arid)>0,"Bank planting did not become sparser in an arid climate");
        Check(wet.Count(p=>p.Form==VegetationForm.Shrub)>wet.Count(p=>p.Form!=VegetationForm.Shrub),"Riparian growth is not predominantly bushes with occasional trees");
        Check(coldArid.All(p=>p.Form!=VegetationForm.Palm),"Cold dry banks received palms from aridity alone");
        Check(arid.Any(p=>p.Form==VegetationForm.Palm),"A warm frost-free arid bank never received an appropriate palm prior");
        Check(wet.SequenceEqual(Plan(Climate(3,12))),"Riparian clumps are not deterministic");
        var road=new PreparedSegment(99,1,2,new(-100,52,0),new(100,52,0),NetworkKind.Road,"residential",2,false,false,false,0){NativeWidth=12};
        var protectedPlan=VegetationPlanner.Prepare(data,input,request,terrain,new[]{road},Array.Empty<GeographicPolygon>(),water,Climate(3,12),new ImportLog(),default);
        Check(protectedPlan.All(p=>NetworkNormalizer.DistanceToSegment(p.Position,road.Start,road.End)>9),"Inferred bank vegetation entered a road corridor");
        var building=new GeographicPolygon(77,new Dictionary<string,string>{{"building","yes"}},new(){new[]{new float3(30,0,-700),new float3(100,0,-700),new float3(100,0,700),new float3(30,0,700)}},new());
        var built=VegetationPlanner.Prepare(data,Array.Empty<float3>(),request,terrain,Array.Empty<PreparedSegment>(),new[]{building},water,Climate(3,12),new ImportLog(),default);
        Check(built.All(p=>!building.Contains(p.Position)),"New bank vegetation entered a mapped building");
        Console.WriteLine("PASS: continuous river-channel exclusion, clustered shrubs and trees, wet/arid density, cold-palm guard, road clearance and deterministic bank planting.");
    }
}
#endif
