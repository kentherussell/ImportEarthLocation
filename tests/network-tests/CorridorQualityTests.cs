using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

internal static class CorridorQualityTests
{
    private static void Check(bool value,string message){if(!value)throw new Exception(message);}
    private static PreparedSegment Line(long way,long a,long b,float3 start,float3 end,NetworkKind kind=NetworkKind.Road)=>
        new(way,a,b,start,end,kind,kind==NetworkKind.Rail?"rail":"motorway",2,kind==NetworkKind.Road,false,false,0)
        {Control1=math.lerp(start,end,1f/3),Control2=math.lerp(start,end,2f/3),NativeWidth=kind==NetworkKind.Rail?12:16,
            SourceGeometry=new[]{start,end},OriginalGeometry=new[]{start,end},SourceWayIds=new[]{way},Reference="Fixture"};
    public static void Run()
    {
        var left=Line(1,1,2,new(-200,100,4),new(200,100,4)) with{CorridorId=10};
        var right=Line(2,3,4,new(200,100,-4),new(-200,100,-4)) with{CorridorId=10};
        var paired=CarriagewaySeparation.Repair(new[]{left,right},new ImportLog(),default);
        for(var j=0;j<=32;j++)
        {
            var a=NetworkNormalizer.Evaluate(paired[0],j/32f);var b=NetworkNormalizer.Evaluate(paired[1],1-j/32f);
            Check(Math.Abs(a.z-9)<.02&&Math.Abs(b.z+9)<.02,"Straight paired carriageway acquired a bulge or inconsistent median");
            Check(Math.Abs(a.y-100)<.001&&Math.Abs(b.y-100)<.001,"Horizontal corridor fit changed elevations");
        }
        Check(left.Start.z==4&&right.Start.z==-4,"Corridor fit mutated its immutable reference");
        var a0=Line(3,10,11,new(-100,0,0),new(0,0,0)) with{Bridge=true};
        var a1=Line(3,11,12,new(0,0,0),new(100,0,0)) with{Bridge=true};
        Check(NativeChainCompactor.Prepare(new[]{a0,a1},new ImportLog(),default).Count==1,"A compatible bridge interval retained a meaningless internal node");
        Check(NativeChainCompactor.Prepare(new[]{a0,a1 with{Bridge=false}},new ImportLog(),default).Count==2,"An actual abutment was contracted");

        var terrain=Enumerable.Repeat((ushort)32768,32*32).ToArray();const float scale=200;
        var sky=Line(9,90,91,new(-100,4000,0),new(100,4100,0),NetworkKind.Rail) with{Bridge=true,OneWay=false};
        var log=new ImportLog();var lowered=CoordinatedNetworkDesign.Prepare(new[]{sky},terrain,scale,4000,log,default);
        Check(lowered.Count==1&&new[]{lowered[0].CourseStart,lowered[0].CourseControl1,lowered[0].CourseControl2,lowered[0].CourseEnd}.All(p=>math.all(math.isfinite(p))&&p.y<131&&p.y>99),"Rail inherited escalating deck heights instead of the fixed terrain datum");
        Check(log.Lines.Count(l=>l.StartsWith("Joint tunnel profile:"))==1,"A design invoked multiple joint vertical solves");

        var road=Line(20,20,21,new(-200,0,20),new(200,0,20));
        var bridge=Line(21,22,23,new(0,0,-200),new(0,0,200),NetworkKind.Rail) with{Bridge=true,Layer=1,OneWay=false};
        var crossing=CoordinatedNetworkDesign.Prepare(new[]{road,bridge},terrain,scale,4000,new ImportLog(),default);
        var upper=NetworkNormalizer.Evaluate(crossing.Single(s=>s.Kind==NetworkKind.Rail),.55f);
        var lower=NetworkNormalizer.Evaluate(crossing.Single(s=>s.Kind==NetworkKind.Road),.5f);
        Check(upper.y-lower.y>6.5f,"Real crossing station failed to receive clearance");
        Check(crossing.All(s=>s.Start.y<161&&s.End.y<161),"Crossing correction escalated entire decks beyond terrain-relative bounds");

        var before=Line(30,30,31,new(-1000,0,0),new(-600,0,0));
        var tunnel=Line(30,31,32,new(-600,0,0),new(600,0,0)) with{Tunnel=true,Layer=-1};
        var after=Line(30,32,33,new(600,0,0),new(1000,0,0));
        var buried=CoordinatedNetworkDesign.Prepare(new[]{before,tunnel,after},terrain,scale,4000,new ImportLog(),default);
        var underground=buried.Where(s=>s.Tunnel).ToArray();
        Check(underground.Length==2,"Portal-to-portal tunnel needs one interior anchor, not repeated 80m subdivisions");
        Check(underground.All(s=>s.StartElevation<0&&s.EndElevation<0),"Tunnel composition elevations are not below the final terrain datum");
        Check(buried.Any(s=>s.StartNodeId==31)&&buried.Any(s=>s.EndNodeId==32),"Tunnel profiling lost portal identities");

        var waves=new ushort[128*128];
        for(var z=0;z<128;z++)for(var x=0;x<128;x++)waves[z*128+x]=(ushort)Math.Round((100+.4*Math.Sin(2*Math.PI*((x/127d-.5)*4000)/80))/200*65535);
        var noisy=Enumerable.Range(0,40).Select(i=>Line(40,100+i,101+i,new(-600+i*30,0,500),new(-570+i*30,0,500),NetworkKind.Rail)).ToArray();
        var smooth=CoordinatedNetworkDesign.Prepare(noisy,waves,200,4000,new ImportLog(),default);
        Check(smooth.Count<noisy.Length/2,"Straight railway retained dense sampling nodes");
        var samples=smooth.SelectMany(s=>Enumerable.Range(0,33).Select(i=>NetworkNormalizer.Evaluate(s,i/32f).y)).ToArray();
        Check(samples.Max()-samples.Min()<.4,"Sub-grade-limit DEM ripples were preserved instead of smoothing the longitudinal profile");

        var rows=new SparseProfileRows(2);rows.Append(new[]{0},new[]{1d},1,3,1,ProfileRowKind.EqualityCost);rows.Append(new[]{1},new[]{1d},1,7,1,ProfileRowKind.EqualityCost);rows.Freeze();
        var independent=NetworkProfileSolve.SolveComponents(rows,new[]{0d,0d},20,Array.Empty<ProfileStar>());
        Check(independent.Report.Components==2&&Math.Abs(independent.Heights[0]-3)<1e-5&&Math.Abs(independent.Heights[1]-7)<1e-5,"Separable FISTA changed independent optima");
        Console.WriteLine("PASS: immutable straight paired corridors, structure compaction/abutments, terrain-relative rail recovery, crossing-station clearance, single-solve tunnel portals and independent FISTA components. Native rendering is not exercised.");
    }

    public static void Replay(string folder)
    {
        var description=Newtonsoft.Json.Linq.JObject.Parse(File.ReadAllText(Path.Combine(folder,"before-cover-geometry.json")));
        var source=File.ReadLines(Path.Combine(folder,"before-cover-network.ndjson")).Select(line=>Newtonsoft.Json.Linq.JObject.Parse(line).ToObject<PreparedSegment>()!).ToArray();
        var bytes=File.ReadAllBytes(Path.Combine(folder,"before-cover-heightmap.u16"));var raster=new ushort[bytes.Length/2];Buffer.BlockCopy(bytes,0,raster,0,bytes.Length);
        var scale=(float)description["heightScaleMeters"]!;var width=(double)description["mapWidthMeters"]!;
        var watch=System.Diagnostics.Stopwatch.StartNew();var log=new ImportLog();
        var result=CoordinatedNetworkDesign.Prepare(source,raster,scale,width,log,default);
        Check(result.All(s=>new[]{s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd}.All(p=>math.all(math.isfinite(p)))),"Archived corridor recovery returned nonfinite geometry");
        var lift=result.Where(s=>s.Kind==NetworkKind.Rail).SelectMany(s=>Enumerable.Range(0,17).Select(i=>{var p=NetworkNormalizer.Evaluate(s,i/16f);return p.y-NetworkProcessor.Sample(p.x,p.z,raster,scale,width);})).Max();
        Check(lift<100,"Archived sky-rail fixture still has extreme terrain-relative elevation");
        Console.WriteLine($"ARCHIVED CORRIDOR REPLAY source={source.Length}; output={result.Count}; seconds={watch.Elapsed.TotalSeconds:0.###}; maximumRailLiftMeters={lift:R}; notRawOsmReplay=true; nativeRenderingNotExercised=true");
        foreach(var line in log.Lines.Where(l=>l.Contains("PROFILE CROSSINGS")||l.Contains("PROFILE COMPONENT")||l.Contains("PROFILE DESIGN")||l.Contains("COORDINATED DESIGN")||l.StartsWith("Joint tunnel profile")))Console.WriteLine(line);
    }
}
