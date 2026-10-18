using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

static class VisualDesignTests
{
    static void Check(bool value,string message){if(!value)throw new Exception(message);}
    static PreparedSegment Road(long way,long a,long b,float3 p,float3 q)=>
        new(way,a,b,p,q,NetworkKind.Road,"motorway",2,true,false,false,0)
        {NativeWidth=12,NativeAssetKey="visual",Reference="V1",SourceGeometry=new[]{p,q},OriginalGeometry=new[]{p,q}};
    static NativeAssetContract Asset()=>new("visual",new[]{
        new NativeCompositionEnvelope(NativeStructureMode.Ground,0,12,0,0,-1,1,8,true),
        new NativeCompositionEnvelope(NativeStructureMode.Elevated,0,12,0,0,-1,1,8,true),
        new NativeCompositionEnvelope(NativeStructureMode.Tunnel,0,12,0,0,-1,1,8,true)},new[]{-2f,2f},.03f);
    public static void Run()
    {
        var route=Enumerable.Range(0,24).Select(i=>Road(100+i,i+1,i+2,new(i*10,50,(float)Math.Sin(i*.25)),new((i+1)*10,50,(float)Math.Sin((i+1)*.25)))).ToArray();
        var compact=NativeChainCompactor.Prepare(route,new ImportLog(),default);
        var shuffled=NativeChainCompactor.Prepare(route.Reverse().ToArray(),new ImportLog(),default);
        Check(compact.Count<route.Length/2,"Whole-run fitting retained source-scale segmentation");
        Check(compact.SelectMany(s=>s.SourceWayIds).Distinct().Count()==route.Length,"Whole-run fitting lost source identities");
        Check(compact.Count==shuffled.Count&&compact.Zip(shuffled).All(p=>p.First.StartNodeId==p.Second.StartNodeId&&p.First.EndNodeId==p.Second.EndNodeId&&
            math.all(p.First.CourseControl1==p.Second.CourseControl1)&&math.all(p.First.CourseControl2==p.Second.CourseControl2)),"Whole-run fitting depends on source enumeration order");
        var aliased=route.Select(s=>s with{SourceNodeAliases=new[]{new SourceNodeAlias(999,12,false)}}).ToArray();
        var required=new SourceTopologyManifest(new[]{new SourceNodeKey(NetworkKind.Road,999)},Array.Empty<SourceConnection>());
        var protectedPort=NativeChainCompactor.Prepare(aliased,new ImportLog(),default,required);
        Check(protectedPort.Any(s=>s.StartNodeId==12||s.EndNodeId==12),"Compaction lost an immutable source port hidden behind a nonmandatory local alias");
        var run=Enumerable.Range(0,5).Select(i=>Road(200+i,100+i,101+i,new(i*100,50,0),new((i+1)*100,50,0))).ToArray();
        var budgets=NetworkVisualDesign.FitBudgets(run,2000,Array.Empty<float3[]>(),Array.Empty<float3[]>(),null,new ImportLog(),default);
        Check(budgets[2].SourceFitToleranceMeters==6&&budgets[0].SourceFitToleranceMeters==2,"Open-space policy did not distinguish a terminal from a simple interior");
        var protectedBudgets=NetworkVisualDesign.FitBudgets(run,2000,new[]{new[]{new float3(220,0,-20),new float3(280,0,20)}},Array.Empty<float3[]>(),null,new ImportLog(),default);
        Check(protectedBudgets[2].SourceFitToleranceMeters==2,"Geographic allowance ignored protected geography");
        var mainA=Road(250,2500,2501,new(-600,50,4),new(0,50,4));var mainB=Road(251,2501,2502,new(0,50,4),new(600,50,4));
        var oppositeA=Road(252,2503,2504,new(600,50,-4),new(0,50,-4));var oppositeB=Road(253,2504,2505,new(0,50,-4),new(-600,50,-4));
        var ramp=Road(254,2501,2506,new(0,50,4),new(100,50,150)) with{Classification="motorway_link",NativeWidth=8};
        var junctionPair=NetworkPairedCorridors.Prepare(new[]{mainA,mainB,oppositeA,oppositeB,ramp},new ImportLog(),default);
        Check(junctionPair.Any(s=>s.PairedProfileId!=0)&&junctionPair.Any(s=>s.StartNodeId==2501)&&junctionPair.Any(s=>s.EndNodeId==2501),
            "Mainline matching across a ramp branch lost pairing or the real junction port");
        var junctionAnchors=junctionPair.SelectMany(s=>new[]{(Id:s.StartNodeId,Point:s.Start),(Id:s.EndNodeId,Point:s.End)}).Where(p=>p.Id==2501).ToArray();
        Check(junctionAnchors.All(p=>math.all(p.Point==junctionAnchors[0].Point)),"Paired formation left mismatched anchors at a ramp junction");
        var assets=new Dictionary<string,NativeAssetContract>{{"visual",Asset()}};
        var valley=Enumerable.Range(0,17*17).Select(i=>(ushort)Math.Round((50-25*Math.Sin(Math.PI*(i%17)/16))/100*65535)).ToArray();
        var crossing=Road(300,3000,3001,new(-200,50,0),new(200,50,0));
        var bridge=NetworkVisualDesign.StructureIntent(new[]{crossing},valley,100,400,assets,new ImportLog(),default);
        Check(bridge[0].Bridge&&math.all(bridge[0].Start==crossing.Start)&&math.all(bridge[0].End==crossing.End),"Valley formation did not select a coherent bridge with fixed outer anchors");
        var bridgeProfile=NetworkTunnelProfile.Solve(bridge,100,Array.Empty<(int,float,float)>(),new ImportLog(),default,design:new(valley,400,Assets:assets));
        Check(Enumerable.Range(0,33).All(i=>Math.Abs(NetworkNormalizer.Evaluate(bridgeProfile[0],i/32f).y-50)<1e-4),"Ground cut/fill costs pulled a coherent bridge back into its valley");
        var explicitGround=crossing with{RawSourceTags=new Dictionary<string,string>{{"bridge","no"}}};
        Check(!NetworkVisualDesign.StructureIntent(new[]{explicitGround},valley,100,400,assets,new ImportLog(),default)[0].Bridge,"Structure preflight overrode an explicit physical policy");
        var ridge=Enumerable.Range(0,17*17).Select(i=>(ushort)Math.Round((50+25*Math.Sin(Math.PI*(i%17)/16))/100*65535)).ToArray();
        Check(NetworkVisualDesign.StructureIntent(new[]{crossing},ridge,100,400,assets,new ImportLog(),default)[0].Tunnel,"Crest formation did not select a coherent tunnel");
        PreparedSegment Rail(long way,long a,long b,float3 p,float3 q)=>Road(way,a,b,p,q) with{Kind=NetworkKind.Rail,Classification="rail",OneWay=false};
        var fan=new[]{Rail(401,1,2,new(-100,50,0),new(0,50,0)),Rail(402,2,3,new(0,50,0),new(8,50,0)),
            Rail(403,3,4,new(8,50,0),new(100,50,0)),Rail(404,2,5,new(0,50,0),new(-100,50,100)),Rail(405,3,6,new(8,50,0),new(100,50,-100))};
        var nativeFan=RailCorridorConsolidator.NativeJunctionFootprints(fan,new ImportLog(),default);
        var fanLog=new ImportLog();SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(fan),nativeFan,fanLog,default);
        Check(nativeFan.Count==4&&fanLog.Metric("Missing mandatory selected-source ports")==0&&fanLog.Metric("Unresolved selected-source reachability obligations")==0,
            "Native-width rail footprint reconstruction lost external ports or movements");
        var noBridgeAsset=new Dictionary<string,NativeAssetContract>{{"visual",new("visual",new[]{new NativeCompositionEnvelope(NativeStructureMode.Ground,0,12,0,0,-1,1,8,true)},Array.Empty<float>())}};
        Check(!NetworkVisualDesign.StructureIntent(new[]{crossing},valley,100,400,noBridgeAsset,new ImportLog(),default)[0].Bridge,"Structure selection invented an unavailable native composition");
        var wideBridge=new Dictionary<string,NativeAssetContract>{{"visual",new("visual",new[]{
            new NativeCompositionEnvelope(NativeStructureMode.Ground,0,12,0,0,-1,1,8,true),
            new NativeCompositionEnvelope(NativeStructureMode.Elevated,0,24,0,0,-1,1,8,true)},Array.Empty<float>())}};
        Check(!NetworkVisualDesign.StructureIntent(new[]{crossing with{MaximumNativeWidth=20}},valley,100,400,wideBridge,new ImportLog(),default)[0].Bridge,
            "Structure preflight selected a composition wider than the native placement contract");
        var noisyTerrain=Enumerable.Range(0,33*33).Select(i=>(ushort)Math.Round((100+(i%2==0?4:-4))/200d*65535)).ToArray();
        var intent=Road(450,4500,4501,new(-200,100,0),new(200,100,0));
        var smooth=NetworkTunnelProfile.Solve(new[]{intent},200,Array.Empty<(int,float,float)>(),new ImportLog(),default,design:new(noisyTerrain,400));
        Check(NetworkCurveObservations.Measure(new(smooth[0].CourseStart,smooth[0].CourseControl1,smooth[0].CourseControl2,smooth[0].CourseEnd)).SampledSignedGradeVariation<1e-5,
            "Raw terrain noise overrode a feasible flat route intent");
        var cache=new NetworkPhysicalContacts.GeometryCache();var flat=Enumerable.Repeat((ushort)32768,17*17).ToArray();
        NetworkTunnelProfile.MeasurePhysical(new[]{crossing},flat,100,1000,assets,default,cache);
        var raised=crossing with{Start=crossing.Start+new float3(0,10,0),End=crossing.End+new float3(0,10,0)};
        NetworkTunnelProfile.MeasurePhysical(new[]{raised},flat,100,1000,assets,default,cache);
        Check(cache.Builds==1&&cache.Reuses==1,"Height-only readback rebuilt immutable XZ proofs");
        var moved=raised with{End=raised.End+new float3(0,0,1)};
        NetworkTunnelProfile.MeasurePhysical(new[]{moved},flat,100,1000,assets,default,cache);
        Check(cache.Builds==2,"Changed geometry reused stale footprint proofs");
        var oscillating=crossing with{Control1=new(-65,80,0),Control2=new(65,20,0)};
        var visualLog=new ImportLog{CaptureRecords=true};NetworkQualityAudit.Planned(new[]{oscillating},visualLog,default);
        Check(visualLog.Findings.Any(f=>f.Category=="visual-oscillation"&&f.Failed>0),"Visual ledger missed a full hump-and-dip profile");
        var svg=(string)visualLog.Artifacts["visual-quality-worst.svg"];
        Check(svg.Contains("<polyline")&&svg.Contains("sampled, not rendered")&&System.Text.Encoding.UTF8.GetByteCount(svg)<1024*1024,"Visual evidence diagram missing, misleading, or unbounded");
        var document=new System.Xml.XmlDocument();document.LoadXml(svg);
        Console.WriteLine($"PASS: whole-route fit {route.Length}->{compact.Count}; guarded budgets; bridge/tunnel intent; footprint cache invalidation; visual ledger and SVG.");
    }
}
