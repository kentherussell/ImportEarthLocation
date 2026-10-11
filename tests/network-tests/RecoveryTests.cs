using System.Text;
using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

internal static class RecoveryTests
{
    private static void Check(bool pass,string message){if(!pass)throw new Exception(message);}
    public static void Run()
    {
        var log=new ImportLog();
        Check(OsmRecovery.Run<int>("test",()=>throw new InvalidOperationException("bad OSM"),17,log,default)==17,"Data-stage fallback failed");
        try{OsmRecovery.Run<int>("test",()=>throw new OutOfMemoryException(),17,log,default);throw new Exception("Memory failure swallowed");}catch(OutOfMemoryException){}
        using(var cancellation=new CancellationTokenSource())
        {cancellation.Cancel();try{OsmRecovery.Run("test",()=>1,17,log,cancellation.Token);throw new Exception("Cancellation swallowed");}catch(OperationCanceledException){}}
        PreparedSegment S(long way,long a,long b,float3 start,float3 end)=>new(way,a,b,start,end,NetworkKind.Road,"primary",2,false,false,false,0);
        var original=new[]{S(1,1,2,new(-600,50,0),new(0,50,0)) with{Control1=new float3(float.NaN,0,0)},S(2,2,3,new(0,55,0),new(100,50,0)),S(3,4,5,new(float.NaN,0,0),new(10,50,100)) with{SourceGeometry=new[]{new float3(-50,50,100),new float3(10,50,100)}}};
        var repaired=OsmRecovery.RepairGeometry(original,1000,log,default);
        Check(repaired.Count==3&&repaired[0].Start.x==-500&&!repaired[0].Control1.HasValue&&repaired[2].Start.x==-50,"Local geometry repair omitted a recoverable original anchor");
        Check(math.distance(repaired[0].End,repaired[1].Start)<.001f&&original[1].Start.y==55,"Shared endpoint repair broke topology/mutated source");
        var missing=S(4,2,6,new(float.NaN,0,0),new(40,50,-100));
        Check(OsmRecovery.RepairGeometry(new[]{missing,original[0]},1000,log,default).Count==2,"Healthy later incident node cannot recover a missing shared anchor");
        try{OsmRecovery.RepairGeometry(new[]{missing with{StartNodeId=99}},1000,log,default);throw new Exception("Unknown malformed anchor silently omitted");}catch(InvalidOperationException){}
        var root=Path.Combine(Path.GetTempPath(),"iel-recovery-"+Guid.NewGuid().ToString("N"));
        var valid=Encoding.UTF8.GetBytes("<osm><node id='1' lat='38' lon='-77'/><node id='2' lat='38.001' lon='-77'/><way id='4'><nd ref='1'/><nd ref='2'/><tag k='highway' v='primary'/></way></osm>");
        var partial=OsmRecovery.Parse(new[]{valid,Encoding.UTF8.GetBytes("<osm><remark>timeout</remark></osm>")},root,"test",log,default);
        Check(partial.Nodes.Count==2&&partial.Ways.Count==1&&log.Lines.Any(s=>s.Contains("coverageComplete=false")),"Unreadable tile discarded healthy OSM coverage");
        Check(!Directory.Exists(root),"Partial parsing should not persist an incomplete full-coverage cache");
        var cliff=new ushort[128*128];for(var z=0;z<128;z++)for(var x=0;x<128;x++)cliff[z*128+x]=(ushort)(x<64?0:65535);
        var roads=new[]{S(10,10,11,new(-20,0,0),new(0,0,0)),S(11,11,12,new(0,0,0),new(20,0,0)),S(12,11,13,new(0,0,0),new(0,0,50))};
        var profiles=NetworkVerticalProfiles.Prepare(roads,cliff,1000,1000,log,default);
        Check(profiles.Count>=3&&profiles.Any(s=>s.WayId==10)&&profiles.Any(s=>s.WayId==11),"Unrepairable connected roads were deleted");
        Check(profiles.SelectMany(s=>new[]{(id:s.StartNodeId,p:s.Start),(id:s.EndNodeId,p:s.End)}).Where(p=>p.id==11).Select(p=>p.p.y).Distinct().Count()==1,"Relaxed profiles disconnected shared junction");
        var chain=Enumerable.Range(0,8).Select(i=>S(100+i,i,i+1,new(i*20,0,0),new((i+1)*20,0,0))).ToList();
        chain.Add(S(108,8,9,new(160,4,0),new(163.9f,8,0)) with{Bridge=true});
        var heights=Enumerable.Range(0,10).ToDictionary(i=>(long)i,i=>i==9?8f:i==8?4f:0f);
        var lengths=chain.ToDictionary(s=>s,s=>math.distance(s.Start.xz,s.End.xz));
        ConnectedRoadGrades.Repair(chain,heights,lengths,log,default);
        Check(heights[8]>=4&&heights[9]>=8,"Connected repair lowered bridge clearance");
        Check((heights[9]-heights[8])/3.9f<.15f,"Short bridge endpoint spike not repaired");
        Check(heights[6]>.1f&&heights[7]>.1f,"Bridge repair did not propagate through multiple approach segments");
        Check(chain.All(s=>Math.Abs(heights[s.EndNodeId]-heights[s.StartNodeId])<=lengths[s]*NetworkVerticalProfiles.GradeLimit(s)*.8f+.02f),"Feasible connected approach violates endpoint grade");
        var pinned=chain[^1] with{Tunnel=true,Bridge=false};var fixedHeights=new Dictionary<long,float>{{8,4},{9,8}};
        ConnectedRoadGrades.Repair(new[]{pinned},fixedHeights,new Dictionary<PreparedSegment,float>{{pinned,3.9f}},log,default);
        Check(fixedHeights[8]==4&&fixedHeights[9]==8,"Unrepairable tunnel anchor moved");
        Check(log.Lines.Any(s=>s.Contains("Connected road grading:")&&s.Contains("infeasible")),"Infeasible connected constraints falsely reported as success");
        using(var cancelled=new CancellationTokenSource())
        {cancelled.Cancel();try{ConnectedRoadGrades.Repair(chain,heights,lengths,log,cancelled.Token);throw new Exception("Connected repair ignored cancellation");}catch(OperationCanceledException){}}
        Console.WriteLine("PASS: OSM recovery, partial XML coverage, immutable bounded geometry/shared joins, difficult connected grades, cancellation and memory failure propagation.");
    }
}
