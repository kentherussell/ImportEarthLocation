using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

static class SafetyTests
{
    static void Check(bool value,string message){if(!value)throw new Exception(message);}
    static PreparedSegment S(long way,long a,long b,float3 start,float3 end,bool bridge=false,int layer=0,NetworkKind kind=NetworkKind.Road)=>
        new(way,a,b,start,end,kind,kind==NetworkKind.Rail?"rail":"motorway",kind==NetworkKind.Rail?2:3,kind==NetworkKind.Road,bridge,false,layer){NativeWidth=12,SourceGeometry=new[]{start,end}};
    public static void Run()
    {
        LaneChecks();
        CorridorRecoveryChecks();
        var raster=Enumerable.Repeat((ushort)(100f/200*65535),256).ToArray();
        var input=new[]{S(1,1,2,new(-200,100,0),new(0,120,0),true,1),S(1,2,3,new(0,120,0),new(200,100,0),true,1),
            S(2,4,1,new(-400,100,0),new(-200,100,0)),S(2,3,5,new(200,100,0),new(400,100,0)),S(3,6,7,new(0,100,-400),new(0,100,400))};
        var result=NetworkGeometrySafety.Repair(input,raster,200,1000,new ImportLog(),default);
        Check(result.Count==input.Length,"Safety solver deleted road sections");
        Check(result.Where(s=>s.Bridge).All(s=>s.Start.y<=120.01f&&s.End.y<=120.01f),"Local bridge crest became a sky-high whole-interchange plane");
        Check(result.All(s=>NetworkVerticalProfiles.WithinGradeCeiling(s,NetworkGeometrySafety.GradeLimit(s))),"Final fitted curve exceeds preferred grade");
        Check(input[0].Start.y==100&&input[0].End.y==120,"Safety repair mutated caller input");
        var offset=new[]{S(10,10,11,new(-200,100,0),new(200,100,0),true,2),S(11,12,13,new(-200,100,10),new(200,100,10),true,1)};
        var parallel=NetworkGeometrySafety.Repair(offset,raster,200,1000,new ImportLog(),default);
        Check(parallel[0].Start.y-parallel[1].Start.y>=7,"Parallel overlapping deck footprints were ignored");
        var rail=new[]{S(20,1,2,new(-200,100,0),new(200,100,0),true,1,NetworkKind.Rail),S(21,1,2,new(0,100,-200),new(0,100,200))};
        var mixed=NetworkGeometrySafety.Repair(rail,raster,200,1000,new ImportLog(),default);
        Check(mixed[0].Start.y-mixed[1].Start.y>=7,"Road/rail OSM IDs accidentally share elevation or bypass clearance");
        var log=new ImportLog();
        try{NetworkGeometrySafety.Validate(offset,log,default);throw new Exception("Colliding prepared map passed final audit");}catch(InvalidOperationException e){Check(e.Message.Contains("prevented application"),"Safety gate lost useful failure message");}
        var sky=new[]{S(30,30,31,new(-10,100,0),new(10,190,0))};
        try{NetworkGeometrySafety.Validate(sky,new ImportLog(),default);throw new Exception("Steep profile passed audit");}catch(InvalidOperationException){}
        var tied=new[]{S(40,40,41,new(-20,100,0),new(0,100,0),true),S(41,41,42,new(0,100,0),new(20,100,0))};
        NetworkGeometrySafety.Validate(tied,new ImportLog(),default);
        var loop=S(45,45,46,new(0,100,0),new(100,100,0),true,1) with{Control1=new(500,100,500),Control2=new(500,100,-500)};
        var joinedStraight=S(46,45,47,new(0,100,0),new(800,100,0),true,1);
        try{NetworkGeometrySafety.Validate(new[]{loop,joinedStraight},new ImportLog(),default);throw new Exception("Remote crossing bypassed clearance because the curves share a far-away endpoint");}catch(InvalidOperationException){}
        var tunnels=new[]{S(60,60,61,new(-200,100,0),new(0,100,0),kind:NetworkKind.Rail) with{Tunnel=true},S(61,61,62,new(0,100,0),new(200,100,0),kind:NetworkKind.Rail) with{Tunnel=true}};
        var buried=NetworkGeometrySafety.Repair(tunnels,raster,200,1000,new ImportLog(),default);
        NetworkGeometrySafety.ValidateTunnelCover(buried,raster,200,1000,default);
        try{NetworkGeometrySafety.ValidateTunnelCover(tunnels,raster,200,1000,default);throw new Exception("Surface-level tunnel passed final terrain audit");}catch(InvalidOperationException){}
        var surfaced=(ushort[])raster.Clone();Array.Fill(surfaced,(ushort)(85f/200*65535));
        try{NetworkGeometrySafety.ValidateTunnelCover(buried,surfaced,200,1000,default);throw new Exception("Later terrain carving exposed a tunnel without detection");}catch(InvalidOperationException){}
        var outside=new[]{S(70,70,71,new(450,100,0),new(480,100,0)) with{EndCurveOffset=new(25,0,0)}};
        try{NetworkGeometrySafety.ValidateEnvelope(outside,1000,200,default);throw new Exception("Offset native curve outside the map passed audit");}catch(InvalidOperationException){}
        Console.WriteLine("PASS: coupled width-aware road/rail safety, flat bridge groups, parallel deck overlap, native junction exclusions, preferred fitted grades, immutable input and atomic rejection.");
    }
    static void CorridorRecoveryChecks()
    {
        var waves=Enumerable.Range(0,32).Select(i=>S(100+i,i,i+1,new(i*40,100+3*MathF.Sin(i*MathF.PI/2),0),new((i+1)*40,100+3*MathF.Sin((i+1)*MathF.PI/2),0))).ToArray();
        var finished=CorridorProfiles.Prepare(waves,new ImportLog(),default);
        Check(finished.Count==waves.Length&&finished.Select(s=>(s.StartNodeId,s.EndNodeId,s.Start.xz,s.End.xz)).SequenceEqual(waves.Select(s=>(s.StartNodeId,s.EndNodeId,s.Start.xz,s.End.xz))),"Macro profile changed horizontal geometry or source topology");
        Check(finished.Skip(8).Take(16).Max(s=>s.Start.y)-finished.Skip(8).Take(16).Min(s=>s.Start.y)<1,"Corridor still follows short-period DEM oscillations");
        Check(finished.All(s=>NetworkVerticalProfiles.WithinGradeCeiling(s,.08f)),"Corridor wave removal introduced excessive curve grades");
        Check(waves[1].Start.y==103,"Corridor finishing mutated the source");
        var deck=new[]{S(201,1,2,new(-100,100,0),new(100,100,0),true,1),S(202,0,1,new(-500,100,0),new(-100,100,0)),S(203,2,3,new(100,100,0),new(500,100,0))};
        var dryDeck=CorridorProfiles.Prepare(deck,new ImportLog(),default,waterLevel:p=>null);
        Check(dryDeck[0].Start.y==100,"Dry bridge acquired an invented water clearance");
        var wetDeck=CorridorProfiles.Prepare(deck,new ImportLog(),default,waterLevel:p=>Math.Abs(p.x)<50?110:null);
        Check(wetDeck[0].Start.y>=117&&wetDeck[0].End.y>=117&&wetDeck[0].Start.Equals(wetDeck[1].End)&&wetDeck[0].End.Equals(wetDeck[2].Start),"Evidenced-water bridge remains submerged or detached its approaches");
        Check(wetDeck.All(s=>NetworkVerticalProfiles.WithinGradeCeiling(s,.08f)),"Water bridge lift created unsafe approaches");
        var safetyRaster=Enumerable.Repeat((ushort)(100f/200*65535),256).ToArray();
        var safeWetDeck=NetworkGeometrySafety.Repair(wetDeck,safetyRaster,200,1000,new ImportLog(),default,allowQualityWarnings:true,waterLevel:p=>Math.Abs(p.x)<50?110:null);
        Check(safeWetDeck[0].Start.y>=116.99f&&safeWetDeck[0].End.y>=116.99f,"The later coupled clearance solve submerged an already water-safe bridge");
        var curvedDeck=S(211,11,12,new(0,100,0),new(100,106,0),true,1) with{Control1=new(3,100,0),Control2=new(97,106,0)};
        var constrained=new[]{curvedDeck,S(212,12,13,new(100,106,0),new(160,100,0))};
        var constrainedFinish=CorridorProfiles.Prepare(constrained,new ImportLog(),default,finishOnly:true);
        Check(NetworkVerticalProfiles.MaximumGrade(constrainedFinish[0])<=Math.Max(.08f,NetworkVerticalProfiles.MaximumGrade(curvedDeck))+.001f,"Final shared-anchor grading steepened a short curved bridge handle");
        Check(constrainedFinish[0].End.Equals(constrainedFinish[1].Start),"Derivative-bound finishing detached a shared bridge approach");
        var raster=Enumerable.Repeat((ushort)(90f/200*65535),256).ToArray();
        var corridor=new[]{S(301,0,1,new(-400,100,0),new(-200,100,0)),S(302,1,2,new(-200,100,0),new(200,100,0)) with{Tunnel=true},S(303,2,3,new(200,100,0),new(400,100,0))};
        var buried=NetworkGeometrySafety.RepairFinalTunnelCover(corridor,raster,200,1000,new ImportLog(),default);
        NetworkGeometrySafety.ValidateTunnelCover(buried,raster,200,1000,default);
        Check(buried[0].End.Equals(buried[1].Start)&&buried[1].End.Equals(buried[2].Start)&&buried.Count==3,"Final tunnel recovery disconnected a mixed surface portal");
        var shortPortal=new[]{S(311,20,21,new(-210,100,0),new(-10,100,0)),S(312,21,22,new(-10,100,0),new(10,100,0)) with{Tunnel=true},S(313,22,23,new(10,100,0),new(210,100,0))};
        var shortRecovered=NetworkGeometrySafety.RepairFinalTunnelCover(shortPortal,raster,200,1000,new ImportLog(),default);
        Check(shortRecovered.All(s=>NetworkVerticalProfiles.MaximumGrade(s)<=.081f),"Final roof repair recovered cover but left a steep short portal handle/approach");
        NetworkGeometrySafety.ValidateTunnelCover(shortRecovered,raster,200,1000,default);
        var impossiblePortal=new[]{S(321,30,31,new(-200,100,0),new(-10,100,0),true,1),S(322,31,32,new(-10,100,0),new(10,100,0)) with{Tunnel=true}};
        try{NetworkGeometrySafety.RepairFinalTunnelCover(impossiblePortal,raster,200,1000,new ImportLog(),default);throw new Exception("Irrecoverable fixed deck/roof conflict was accepted");}
        catch(PreparationDataException error){Check(error.Issue==PreparationDataIssue.GeometryUnavailable&&error.UserMessage.Contains("unchanged"),"Exhausted geometric recovery was not a graceful typed data outcome");}
        using var stop=new CancellationTokenSource();stop.Cancel();
        try{CorridorProfiles.Prepare(waves,new ImportLog(),stop.Token);throw new Exception("Corridor cancellation ignored");}catch(OperationCanceledException){}
        Console.WriteLine("PASS: macro corridor waves, immutable shared topology, water deck floors/graded approaches, final tunnel-cover recovery with connected mixed portals, cancellation.");
    }
    static void LaneChecks()
    {
        foreach(var side in new[]{-1,1})
        {
            var a=S(51,1,2,new(-300,100,0),new(0,100,0)) with{NativeLaneOffsets=new[]{-3.5f,0,3.5f}};
            var b=S(52,2,3,new(0,100,0),new(300,100,0)) with{Lanes=4,NativeLaneOffsets=new[]{-5.25f,-1.75f,1.75f,5.25f}};
            var r=S(53,4,2,new(-150,100,side*80),new(0,100,0)) with{Classification="motorway_link",Lanes=1,NativeLaneOffsets=new[]{0f}};
            var result=NetworkLaneAlignment.Prepare(new[]{a,b,r},new ImportLog(),default);
            var normal=new float3(0,0,-1);var rampSide=-side;
            var oppositeLaneBefore=rampSide>0?-3.5f:3.5f;var oppositeLaneAfter=rampSide>0?-5.25f:5.25f;
            Check(math.distance(result[0].CourseEnd+normal*oppositeLaneBefore,result[1].CourseStart+normal*oppositeLaneAfter)<.001f,"Width change did not keep the through lanes aligned");
            var outerAfter=rampSide>0?5.25f:-5.25f;
            Check(math.distance(result[2].CourseEnd,result[1].CourseStart+normal*outerAfter)<.001f,"Ramp lane centre still joins the highway centre instead of its closest lane");
            Check(math.all(result[0].End==result[1].Start)&&math.all(result[2].End==result[1].Start),"Lane offset broke exact shared native anchors");
            Check(math.distance(result[2].CourseEnd,result[2].End)>3,"Test did not exercise an actually offset curve endpoint");
        }
    }
}
