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
        TunnelContactTests.Run();
        PairedCorridorTests.Run();
        LaneObligationTests.Run();
        LocalRefitTests.Run();
        AcceptancePipelineTests.Run();
        var quotas=NativeDiagnosticSampling.Quotas(new[]{1,1000,3,2000},256);
        Check(quotas.Sum()==256&&quotas[0]==1&&quotas[2]==3&&quotas[1]>100&&quotas[3]>100,"Small strata wasted native diagnostic sampling capacity");
        Check(NativeDiagnosticSampling.Quotas(new[]{1,2},int.MaxValue).SequenceEqual(new[]{1,2})&&NativeDiagnosticSampling.Quotas(Array.Empty<int>(),256).Length==0,"Complete/empty native sampling quotas incorrect");
        var left=Line(1,1,2,new(-200,100,4),new(200,100,4)) with{CorridorId=10};
        var right=Line(2,3,4,new(200,100,-4),new(-200,100,-4)) with{CorridorId=10};
        var paired=CarriagewaySeparation.Repair(new[]{left,right},new ImportLog(),default);
        for(var j=0;j<=32;j++)
        {
            var a=NetworkNormalizer.Evaluate(paired[0],j/32f);var b=NetworkNormalizer.Evaluate(paired[1],1-j/32f);
            Check(Math.Abs(a.z-9.05)<.02&&Math.Abs(b.z+9.05)<.02,"Straight paired carriageway acquired a bulge or inconsistent median including the two fit-error allowances");
            Check(Math.Abs(a.y-100)<.001&&Math.Abs(b.y-100)<.001,"Horizontal corridor fit changed elevations");
        }
        Check(left.Start.z==4&&right.Start.z==-4,"Corridor fit mutated its immutable reference");
        var partial=right with{End=new float3(0,100,-4),Control1=new float3(400f/3,100,-4),Control2=new float3(200f/3,100,-4)};
        var partialPair=CarriagewaySeparation.Repair(new[]{left,partial},new ImportLog(),default);
        // Report 28 AUD-001: the unpaired part keeps its source anchor and eases
        // outward to the paired offset, never bending in toward the partner
        // (the defect was a 1.26 m inward hook before widening).
        var leftPieces=partialPair.Where(s=>s.WayId==left.WayId).OrderBy(s=>s.Start.x).ToArray();
        var leftPoints=leftPieces.SelectMany(s=>Enumerable.Range(0,201).Select(i=>NetworkNormalizer.Evaluate(s,i/200f))).OrderBy(p=>p.x).ToArray();
        Check(Math.Abs(leftPieces[0].Start.z-4)<.001f&&leftPoints.All(p=>p.z>4-.02f&&p.z<9.05f+.02f),"Partial parallel overlap introduced an inward hook or moved the unpaired anchor");
        Check(leftPoints.Zip(leftPoints.Skip(1)).All(p=>p.Second.z>=p.First.z-.01f),"Partial parallel overlap eased back and forth instead of once outward");
        Check(leftPoints.Where(p=>p.x>=0).All(p=>Math.Abs(p.z-9.05)<.02f),"Partial parallel overlap lost its paired separation");
        NativePreviewSafety.RequireEdge(true,true,true,true);
        foreach(var unsafeContract in new[]{(false,true,true,true),(true,false,true,true),(true,true,false,true),(true,true,true,false)})
        {
            var rejected=false;try{NativePreviewSafety.RequireEdge(unsafeContract.Item1,unsafeContract.Item2,unsafeContract.Item3,unsafeContract.Item4);}catch(NativeSafetyException){rejected=true;}
            Check(rejected,"Structural native failure was downgraded to geometric quality");
        }
        var a0=Line(3,10,11,new(-100,0,0),new(0,0,0)) with{Bridge=true};
        var a1=Line(3,11,12,new(0,0,0),new(100,0,0)) with{Bridge=true};
        Check(NativeChainCompactor.Prepare(new[]{a0,a1},new ImportLog(),default).Count==1,"A compatible bridge interval retained a meaningless internal node");
        Check(NativeChainCompactor.Prepare(new[]{a0,a1 with{Bridge=false}},new ImportLog(),default).Count==2,"An actual abutment was contracted");

        var terrain=Enumerable.Repeat((ushort)32768,32*32).ToArray();const float scale=200;
        var sky=Line(9,90,91,new(-100,4000,0),new(100,4100,0),NetworkKind.Rail) with{Bridge=true,OneWay=false};
        var log=new ImportLog();var lowered=CoordinatedNetworkDesign.Prepare(new[]{sky},terrain,scale,4000,log,default);
        Check(lowered.Count==1&&new[]{lowered[0].CourseStart,lowered[0].CourseControl1,lowered[0].CourseControl2,lowered[0].CourseEnd}.All(p=>math.all(math.isfinite(p))&&p.y<131&&p.y>99),"Rail inherited escalating deck heights instead of the fixed terrain datum");
        Check(log.Lines.Count(l=>l.StartsWith("Joint tunnel profile:"))==1,"A design invoked multiple joint vertical solves");
        foreach(var sign in new[]{-1,1})
        {
            var incline=Line(109,1090,1091,new(0,100,0),new(100,100+sign*2.1f,0)) with{ExplicitIncline=sign*.05f};
            var inclineLog=new ImportLog();var tagged=NetworkTunnelProfile.Solve(new[]{incline},200,Array.Empty<(int index,float t,float ceiling)>(),inclineLog,default,design:new(terrain,4000));
            var untagged=NetworkTunnelProfile.Solve(new[]{incline with{ExplicitIncline=null}},200,Array.Empty<(int index,float t,float ceiling)>(),new ImportLog(),default,design:new(terrain,4000));
            Check(inclineLog.Metric("Signed incline priors accepted")==1&&sign*(tagged[0].CourseEnd.y-tagged[0].CourseStart.y)>sign*(untagged[0].CourseEnd.y-untagged[0].CourseStart.y)+1,
                "Consistent signed incline lost its oriented soft prior");
            var contradictory=new ImportLog();NetworkTunnelProfile.Solve(new[]{incline with{ExplicitIncline=-sign*.05f}},200,Array.Empty<(int index,float t,float ceiling)>(),contradictory,default,design:new(terrain,4000));
            Check(contradictory.Metric("Signed incline priors accepted")==0&&contradictory.Metric("Contradictory signed incline priors")==1,"Contradictory incline was enforced instead of recorded");
        }

        var road=Line(20,20,21,new(-200,0,20),new(200,0,20));
        var bridge=Line(21,22,23,new(0,0,-200),new(0,0,200),NetworkKind.Rail) with{Bridge=true,Layer=1,OneWay=false};
        var envelopes=Enumerable.Range(0,48).Select(i=>new NativeCompositionEnvelope((NativeStructureMode)(i/16),(byte)(i%16),16,0,1,-2,3,4,true)).ToArray();
        var asset=new NativeAssetContract("fixture",envelopes,Array.Empty<float>(),.05f);
        var assets=new Dictionary<string,NativeAssetContract>{{"fixture",asset}};
        road=road with{NativeAssetKey="fixture"};bridge=bridge with{NativeAssetKey="fixture"};
        Check(asset.TryEnvelope(NativeStructureMode.Elevated,out var selected)&&selected.RangeMin==-2&&selected.SurfaceMax==1,"Captured structure datum was not selected");
        Check(Math.Abs(asset.CenterlineCurvatureLimit(16)!.Value-.05/(1+.05*8))<1e-8,"Native train curvature/track-offset conversion incorrect");
        var mutableLanes=new[]{-2f,2f};var freezeInput=new[]{road with{NativeLaneOffsets=mutableLanes}};
        var frozen=new FinalNetworkDesign(freezeInput,new[]{asset});var fingerprint=frozen.Fingerprint;
        mutableLanes[0]=-99;freezeInput[0]=bridge;
        Check(frozen.Sections[0].WayId==road.WayId&&frozen.Sections[0].NativeLaneOffsets[0]==-2&&frozen.Sections[0].FinalDeckCurve.HasValue&&frozen.Fingerprint==fingerprint,"Final design was changed by a mutable preparation input");
        var crossing=CoordinatedNetworkDesign.Prepare(new[]{road,bridge},terrain,scale,4000,new ImportLog(),default,assets:assets);
        var upper=NetworkNormalizer.Evaluate(crossing.Single(s=>s.Kind==NetworkKind.Rail),.55f);
        var lower=NetworkNormalizer.Evaluate(crossing.Single(s=>s.Kind==NetworkKind.Road),.5f);
        Check(upper.y-lower.y>6.5f,"Real crossing station failed to receive clearance");
        Check(crossing.All(s=>s.Start.y<161&&s.End.y<161),"Crossing correction escalated entire decks beyond terrain-relative bounds");

        var parallelUpper=road with{WayId=23,StartNodeId=24,EndNodeId=25,Bridge=true,Layer=1,Start=road.Start+new float3(0,9,10),End=road.End+new float3(0,9,10),Control1=road.CourseControl1+new float3(0,9,10),Control2=road.CourseControl2+new float3(0,9,10)};
        var physical=new SparseProfileRows(8);
        var contacts=NetworkPhysicalContacts.Append(new[]{road,parallelUpper},assets,new[]{0,4},new[]{1,5},new[]{2,6},new[]{3,7},physical,.05,default);
        physical.Freeze();
        Check(contacts.Rows>0&&contacts.MissingAssets==0,"Parallel full-width overlap was missed because centre lines do not cross");
        CloseContact(new[]{0d,0d,0d,0d,9d,9d,9d,9d},true);
        CloseContact(new[]{0d,0d,0d,0d,7d,7d,7d,7d},false);
        void CloseContact(double[] heights,bool clears)=>Check((physical.Evaluate(heights,null,default)<1e-9)==clears,"Full-width rows ignored native underside/surface datum");
        var shared=parallelUpper with{StartNodeId=road.StartNodeId};
        var localRows=new SparseProfileRows(8);
        var local=NetworkPhysicalContacts.Append(new[]{road,shared},assets,new[]{0,4},new[]{1,5},new[]{2,6},new[]{3,7},localRows,.05,default);
        Check(local.Rows>0,"One shared node incorrectly exempted the entire long footprint from clearance");
        foreach(var longitudinalShift in new[]{0f,400f})
        {
            var boundary=parallelUpper with{Start=road.Start+new float3(longitudinalShift,9,16),End=road.End+new float3(longitudinalShift,9,16),
                Control1=road.CourseControl1+new float3(longitudinalShift,9,16),Control2=road.CourseControl2+new float3(longitudinalShift,9,16)};
            var boundaryRows=new SparseProfileRows(8);var boundaryContact=NetworkPhysicalContacts.Append(new[]{road,boundary},assets,new[]{0,4},new[]{1,5},new[]{2,6},new[]{3,7},boundaryRows,.05,default);
            boundaryRows.Freeze();
            Check(boundaryContact.Rows>0&&boundaryRows.Evaluate(new[]{0d,0d,0d,0d,7d,7d,7d,7d},null,default)>0,"Zero-area line/point deck contact was discarded");
        }

        var before=Line(30,30,31,new(-1000,0,0),new(-600,0,0));
        var tunnel=Line(30,31,32,new(-600,0,0),new(600,0,0)) with{Tunnel=true,Layer=-1};
        var after=Line(30,32,33,new(600,0,0),new(1000,0,0));
        var buried=CoordinatedNetworkDesign.Prepare(new[]{before,tunnel,after},terrain,scale,4000,new ImportLog(),default);
        var nativeCover=NetworkGeometrySafety.TunnelCoverSamples(new[]{before,tunnel with{NativeAssetKey="fixture"},after},terrain,scale,4000,default,assets).ToArray();
        var portalCover=nativeCover.First(p=>p.t==0);
        Check(Math.Abs(portalCover.ceiling-(NetworkProcessor.Sample(tunnel.Start.x,tunnel.Start.z,terrain,scale,4000)-3))<.01,"Actual native tunnel roof height was faded or omitted at the portal");
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
        var illRepresented=new SparseProfileRows(1);illRepresented.Append(new[]{0},new[]{1e200},1,0,1,ProfileRowKind.EqualityCost);illRepresented.Freeze();
        var captured=false;
        var fallback=NetworkProfileSolve.SolveComponents(illRepresented,new[]{5d},20,Array.Empty<ProfileStar>(),failureEvidence:(root,global,problem,seed,stars,failed)=>
        {captured=root==0&&global.SequenceEqual(new[]{0})&&ReferenceEquals(problem,illRepresented)&&seed[0]==5&&stars.Length==0&&failed.Report.Outcome==ProfileSolveOutcome.NumericFallback;});
        Check(captured&&fallback.Heights[0]==5,"Numerical fallback did not expose the exact failing local problem or retain its finite prior");
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
