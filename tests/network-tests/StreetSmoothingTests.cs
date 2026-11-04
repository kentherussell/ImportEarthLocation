using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

static class StreetSmoothingTests
{
    static void Check(bool value,string message){if(!value)throw new Exception(message);}
    static PreparedSegment Road(long way,long a,long b,float x,float z,float xx,float zz)=>
        new(way,a,b,new(x,50,z),new(xx,50,zz),NetworkKind.Road,"tertiary",2,false,false,false,0)
        {NativeWidth=8,NativeAssetKey="street",SourceGeometry=new[]{new float3(x,50,z),new float3(xx,50,zz)}};

    public static void Run()
    {
        var source=new[]{Road(1,1,2,-160,0,0,4),Road(1,2,3,0,4,160,0),Road(2,2,4,0,4,0,160)};
        var terrain=Enumerable.Repeat((ushort)32768,65*65).ToArray();var log=new ImportLog();
        var output=VisualNetworkCompiler.Prepare(source,terrain,100,2000,log,default);
        var through=output.Where(s=>s.WayId==1||s.SourceWayIds.Contains(1)).ToArray();
        var incoming=through.Single(s=>s.EndNodeId==2);var outgoing=through.Single(s=>s.StartNodeId==2);
        var a=math.normalizesafe((incoming.CourseEnd-incoming.CourseControl2).xz);var b=math.normalizesafe((outgoing.CourseControl1-outgoing.CourseStart).xz);
        Check(math.dot(a,b)>.99999f,"A gently continuing street retained a kink at its real junction");
        Check(math.distance(incoming.CourseEnd.xz,new float2(0,4))<.001f&&math.all(incoming.CourseEnd==outgoing.CourseStart),"Street smoothing moved or split the junction anchor");
        SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(source),output,log,default);
        Check(log.Metric("Missing mandatory selected-source ports")==0&&log.Metric("Unresolved selected-source reachability obligations")==0,"Street smoothing lost a connected side street");
        UnitCuts();
        ShortStreetStations(terrain);
        GentleMajorRoad(terrain);
        NativeRoundabouts(terrain);
        IntersectionWidths();
        JunctionClusters();
        OverlapFootprints(terrain);
        MedianTurnLinks();
        DividedTrunks();
        DividedLaneHalves();
        StackedTunnels(terrain);
        Console.WriteLine("PASS: smooth anchored street continuation, connected side streets and no short optional street cuts beside a crowded native handoff.");
    }

    static void IntersectionWidths()
    {
        var source=new[]{Road(1200,1,2,-500,0,-40,0) with{Name="Main",ForwardLanes=1,BackwardLanes=1},
            Road(1201,3,2,40,0,-40,0) with{Name="Main",Lanes=4,ConsolidatedAvenue=true},
            Road(1202,3,4,40,0,500,0) with{Name="Main",ForwardLanes=1,BackwardLanes=1},
            Road(1203,2,5,-40,0,-40,200) with{Name="Cross Street"}};
        var log=new ImportLog();var result=NetworkLaneRuns.NormalizeStreets(source,log);
        Check(result[1].Lanes==2&&!result[1].ConsolidatedAvenue&&source[1].Lanes==4,
            "A reversed intersection pocket did not match its regular two-lane road or mutated the source");
        Check(math.all(result[1].Start==source[1].Start)&&math.all(result[1].End==source[1].End)&&result[1].StartNodeId==3&&result[1].EndNodeId==2&&result[3]==source[3],
            "Width normalization changed a source junction or connecting street");
        SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(source),result,log,default);
        Check(log.Metric("Missing mandatory selected-source ports")==0&&log.Metric("Unresolved selected-source reachability obligations")==0,
            "Width normalization lost intersection routes");
        var reversedDonors=source.Select((s,i)=>i is 0 or 2?s with{Lanes=3,ForwardLanes=2,BackwardLanes=1}:s).ToArray();
        var asymmetric=NetworkLaneRuns.NormalizeStreets(reversedDonors,new ImportLog());
        Check(asymmetric[1].ForwardLanes==1&&asymmetric[1].BackwardLanes==2,"Reversed digitisation reversed the adopted lane allocation");
        foreach(var exception in new[]{source[1] with{Bridge=true},source[1] with{Roundabout=true},source[1] with{StartRoundaboutRadius=12},
            source[1] with{OneWay=true},source[1] with{Classification="motorway"},source[1] with{Classification="trunk"},source[1] with{Name="Other Road"}})
        {
            var fixture=source.ToArray();fixture[1]=exception;
            Check(NetworkLaneRuns.NormalizeStreets(fixture,new ImportLog())[1]==exception,"A protected road or junction width was changed");
        }
        var sustained=source.ToArray();sustained[1]=source[1] with{Start=new(200,50,0)};sustained[2]=source[2] with{Start=new(200,50,0)};
        Check(NetworkLaneRuns.NormalizeStreets(sustained,new ImportLog())[1].Lanes==4,"A sustained avenue was narrowed");
        var narrow=source.Select((s,i)=>s with{Lanes=i==1?2:4}).ToArray();
        Check(NetworkLaneRuns.NormalizeStreets(narrow,new ImportLog())[1].Lanes==2,"A narrow connector was expanded to an avenue");
        var terminal=new[]{Road(1210,10,11,-400,0,0,0),Road(1211,11,12,0,0,35,0) with{Lanes=4}};
        Check(NetworkLaneRuns.NormalizeStreets(terminal,new ImportLog())[1].Lanes==2,"A short terminal pocket was not normalized against its longer road body");
        var major=source.Append(Road(1220,2,6,-40,0,-40,-200) with{Classification="trunk"}).ToArray();
        Check(NetworkLaneRuns.NormalizeStreets(major,new ImportLog())[1].Lanes==4,"A major-road junction approach was narrowed");
        var ambiguous=source.Append(Road(1221,2,7,-40,0,300,30) with{Name="Main"}).ToArray();
        Check(NetworkLaneRuns.NormalizeStreets(ambiguous,new ImportLog())[1].Lanes==4,"An ambiguous fork was treated as a simple through road");
    }

    static void JunctionClusters()
    {
        void Routes(PreparedSegment[] source,IReadOnlyList<PreparedSegment> result,string message)
        {
            var log=new ImportLog();SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(source),result,log,default);
            Check(log.Metric("Missing mandatory selected-source ports")==0&&log.Metric("Unresolved selected-source reachability obligations")==0,message);
        }
        PreparedSegment OneWay(long way,long a,long b,float x,float z,float xx,float zz)=>Road(way,a,b,x,z,xx,zz) with{OneWay=true,Lanes=1};
        // A two-way street splitting into a short one-way pair that rejoins at the next junction.
        var split=new[]{Road(1300,1,2,-300,0,0,0) with{Lanes=4,ConsolidatedAvenue=true},
            OneWay(1301,2,4,0,0,10,4),OneWay(1301,4,3,10,4,20,0),OneWay(1302,3,5,20,0,10,-4),OneWay(1302,5,2,10,-4,0,0),
            Road(1303,3,6,20,0,20,200),Road(1304,7,3,20,-200,20,0)};
        var closed=StreetJunctionClusters.Prepare(split,new ImportLog(),default);
        var joined=closed.SingleOrDefault(s=>s.StartNodeId==2&&s.EndNodeId==3);
        Check(joined!=null&&!joined.OneWay&&joined.Lanes==4&&closed.Count(s=>s.OneWay)==0&&joined.SourceWayIds.Contains(1301)&&joined.SourceWayIds.Contains(1302),
            "A short one-way split between a two-way street and its next junction was not closed");
        Routes(split,closed,"Closing a short one-way split lost a route");
        PreparedSegment[] Pair(float length,float offset)=>new[]{split[0],OneWay(1301,2,4,0,0,length/2,offset),OneWay(1301,4,3,length/2,offset,length,0),
            OneWay(1302,3,5,length,0,length/2,-offset),OneWay(1302,5,2,length/2,-offset,0,0),Road(1303,3,6,length,0,length,200),Road(1304,7,3,length,-200,length,0)};
        var curved=StreetJunctionClusters.Prepare(Pair(50,4),new ImportLog(),default).Single(s=>s.StartNodeId==2&&s.EndNodeId==3);
        Check(curved.SourceGeometry.Count>2&&curved.SourceGeometry.All(p=>Math.Abs(p.z)<.001f),"A longer close split did not follow the carriageways' midline");
        Check(StreetJunctionClusters.Prepare(Pair(80,4),new ImportLog(),default).Count(s=>s.OneWay)==4,"A one-way pair longer than the split reach was closed");
        Check(StreetJunctionClusters.Prepare(Pair(50,12),new ImportLog(),default).Count(s=>s.OneWay)==4,"A one-way pair around a real median was closed");
        var highway=split.Select(s=>s.OneWay?s with{Classification="trunk"}:s).ToArray();
        Check(StreetJunctionClusters.Prepare(highway,new ImportLog(),default).Count(s=>s.OneWay)==4,"A highway carriageway pair was closed as a street split");
        // Two T-junctions 4 m apart on one street become one crossing.
        var offset=new[]{Road(1310,10,11,-300,0,0,0) with{Classification="primary"},Road(1310,11,12,0,0,4,0) with{Classification="primary"},
            Road(1310,12,13,4,0,300,0) with{Classification="primary"},Road(1311,11,14,0,0,0,200),Road(1312,12,15,4,0,4,-200)};
        var merged=StreetJunctionClusters.Prepare(offset,new ImportLog(),default);
        Check(merged.Count==4&&merged.SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}).GroupBy(id=>id).Any(g=>g.Count()==4),
            "Two street junctions 4 m apart were not merged into one");
        Check(merged.Where(s=>s.WayId==1312).All(s=>math.distance(s.End.xz,new float2(4,-200))<.001f),"Merging junctions moved a far end");
        Routes(offset,merged,"Merging nearby junctions lost a route");
        var divided=new[]{OneWay(1320,20,21,-300,-6,0,-6),OneWay(1320,21,22,0,-6,300,-6),OneWay(1321,23,24,300,6,0,6),OneWay(1321,24,25,0,6,-300,6),
            Road(1322,21,24,0,-6,0,6),Road(1323,26,21,0,-200,0,-6),Road(1324,24,27,0,6,0,200)};
        Check(StreetJunctionClusters.Prepare(divided,new ImportLog(),default).Count==divided.Length,"A street crossing both carriageways of a divided road was merged into one node");
        var triangle=offset.Append(Road(1313,11,15,0,0,4,-200)).ToArray();
        Check(StreetJunctionClusters.Prepare(triangle,new ImportLog(),default).Count==triangle.Length,"Merging junctions would have created a duplicate connection");
        // A divided street meeting a roundabout: it leaves the ring at 31 and
        // joins it at 32, 12 m downstream. Two-way streets meet 33 and 34.
        PreparedSegment Ring(long a,long b,float x,float z,float xx,float zz)=>OneWay(1330,a,b,x,z,xx,zz) with{Roundabout=true,Lanes=2};
        var roundabout=new[]{Ring(31,32,-6,-19,6,-19),Ring(32,33,6,-19,19,6),Ring(33,34,19,6,-19,6),Ring(34,31,-19,6,-6,-19),
            Road(1331,35,36,0,-300,0,-60) with{Lanes=4,ConsolidatedAvenue=true},OneWay(1332,36,37,0,-60,3,-40),OneWay(1332,37,32,3,-40,6,-19),
            OneWay(1333,31,38,-6,-19,-3,-40),OneWay(1333,38,36,-3,-40,0,-60),Road(1334,33,39,19,6,300,6),Road(1335,34,40,-19,6,-300,6)};
        var approached=StreetJunctionClusters.Prepare(roundabout,new ImportLog(),default);
        var approach=approached.SingleOrDefault(s=>s.StartNodeId==36&&s.EndNodeId is 31 or 32);
        Check(approach!=null&&!approach.OneWay&&approached.Count(s=>s.OneWay&&!s.Roundabout)==0&&approach.SourceWayIds.Contains(1332)&&approach.SourceWayIds.Contains(1333),
            "A divided street's entry and exit at a roundabout were not joined into one two-way approach");
        Check(approached.Count(s=>s.Roundabout)==4&&approached.Where(s=>s.Roundabout).All(s=>s.StartNodeId!=36&&s.EndNodeId!=36),"Joining a roundabout approach changed the ring");
        Routes(roundabout,approached,"Joining a roundabout approach lost the entry or exit route");
        var spread=roundabout.Select(s=>s.StartNodeId==32?Road(s.WayId,31,32,40,-19,s.End.x,s.End.z) with{StartNodeId=32,EndNodeId=s.EndNodeId,OneWay=true,Roundabout=s.Roundabout}:
            s.EndNodeId==32?Road(s.WayId,s.StartNodeId,32,s.Start.x,s.Start.z,40,-19) with{OneWay=true,Roundabout=s.Roundabout}:s).ToArray();
        Check(StreetJunctionClusters.Prepare(spread,new ImportLog(),default).Count(s=>s.OneWay&&!s.Roundabout)==4,"Carriageways meeting a roundabout far apart were joined");
    }

    static void MedianTurnLinks()
    {
        void Routes(PreparedSegment[] source,IReadOnlyList<PreparedSegment> result,string message)
        {
            var log=new ImportLog();SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(source),result,log,default);
            Check(log.Metric("Missing mandatory selected-source ports")==0&&log.Metric("Unresolved selected-source reachability obligations")==0,message);
        }
        PreparedSegment Trunk(long way,long a,long b,float x,float z,float xx,float zz)=>Road(way,a,b,x,z,xx,zz) with{Classification="trunk",OneWay=true,Lanes=2};
        PreparedSegment Link(long way,long a,long b,float x,float z,float xx,float zz)=>Road(way,a,b,x,z,xx,zz) with{Classification="trunk_link",OneWay=true,Lanes=1};
        // A divided trunk 12 m apart: southbound at x=-6, northbound at x=+6.
        var corridor=new[]{Trunk(1500,1,2,-6,400,-6,100),Trunk(1500,2,3,-6,100,-6,0),Trunk(1500,3,4,-6,0,-6,-400),
            Trunk(1501,5,6,6,-400,6,0),Trunk(1501,6,7,6,0,6,400)};
        // U-turn link from southbound at z=100 down the median onto northbound at z=0.
        var uturn=corridor.Concat(new[]{Link(1502,2,8,-6,100,0,80),Link(1502,8,6,0,80,6,0)}).ToArray();
        var opened=MedianLinks.Prepare(uturn,new ImportLog(),default);
        var opening=opened.Where(s=>s.Classification=="trunk_link").ToArray();
        Check(opening.Length==1&&opening[0].EndNodeId==6&&math.distance(opening[0].Start.xz,opening[0].End.xz)<13,
            "A median U-turn link that is the only crossover was not shortened to an opening at its far end");
        Routes(uturn,opened,"A median opening lost the U-turn route");
        // With a cross street over both carriageways the link is folded into the crossing.
        var crossing=uturn.Concat(new[]{Road(1503,3,6,-6,0,6,0),Road(1504,9,3,-200,0,-6,0),Road(1505,6,10,6,0,200,0)}).ToArray();
        var folded=MedianLinks.Prepare(crossing,new ImportLog(),default);
        Check(folded.All(s=>s.Classification!="trunk_link")&&folded.Count==crossing.Length-2,"A median turn link beside a median crossing was not folded into it");
        Routes(crossing,folded,"Folding a median turn link lost its route");
        // A right-turn slip lane outside the corridor is not in the median.
        var slip=corridor.Concat(new[]{Link(1506,2,11,-6,100,-14,80),Link(1506,11,3,-14,80,-6,0)}).ToArray();
        Check(MedianLinks.Prepare(slip,new ImportLog(),default).Count==slip.Length,"A slip lane outside a divided road was treated as a median link");
        // An unnamed right-turn slip hugging the corner of two streets is folded
        // into the turn through their junction.
        var corner=new[]{Road(1510,20,21,-300,0,-60,0),Road(1510,21,22,-60,0,0,0),Road(1510,22,23,0,0,300,0),
            Road(1511,24,22,0,300,0,0),Road(1511,22,25,0,0,0,-60),Road(1511,25,26,0,-60,0,-300)};
        PreparedSegment Turn(long a,long b,float x,float z,float xx,float zz)=>Road(1512,a,b,x,z,xx,zz) with{OneWay=true,Lanes=1};
        var turning=corner.Concat(new[]{Turn(21,27,-60,0,-12,-6),Turn(27,28,-12,-6,-6,-12),Turn(28,25,-6,-12,0,-60)}).ToArray();
        var turned=MedianLinks.Prepare(turning,new ImportLog(),default);
        Check(turned.Count==corner.Length&&turned.Where(s=>s.WayId==1510&&s.StartNodeId==21||s.WayId==1511&&s.StartNodeId==22).All(s=>s.SourceWayIds.Contains(1512)),
            "A turn slip beside two streets was not folded into their junction");
        Routes(turning,turned,"Folding a turn slip lost its route");
        Check(MedianLinks.Prepare(turning.Select(s=>s.WayId==1512?s with{Name="Slip Street"}:s).ToArray(),new ImportLog(),default).Count==turning.Length,
            "A named one-way street was folded as a turn slip");
        // Opposing carriageways 16 m apart, joined at both ends, with streets beyond.
        var opposed=new[]{Turn(30,31,-100,-8,100,-8),Turn(32,33,100,8,-100,8),Road(1513,31,32,100,-8,100,8),Road(1514,33,30,-100,8,-100,-8),
            Road(1515,30,40,-100,-8,-300,-8),Road(1516,31,41,100,-8,300,-8),Road(1517,32,42,100,8,300,8),Road(1518,33,43,-100,8,-300,8)};
        Check(MedianLinks.Prepare(opposed,new ImportLog(),default).Count==opposed.Length,"A carriageway beside its opposing carriageway was folded as a turn slip");
    }

    static void DividedTrunks()
    {
        void Routes(PreparedSegment[] source,IReadOnlyList<PreparedSegment> result,string message)
        {
            var log=new ImportLog();SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(source),result,log,default);
            Check(log.Metric("Missing mandatory selected-source ports")==0&&log.Metric("Unresolved selected-source reachability obligations")==0,message);
        }
        // Southbound at x=-h and northbound at x=+h in 100 m pieces from z=-1000
        // to 1000, sharing node rows so a cross street at z=0 meets both.
        PreparedSegment[] Corridor(float h,int lanes)
        {
            PreparedSegment Trunk(long way,long a,long b,float x,float z,float zz)=>Road(way,a,b,x,z,x,zz) with{Classification="trunk",OneWay=true,Lanes=lanes,Name="Main Trunk"};
            return Enumerable.Range(0,20).SelectMany(k=>new[]{Trunk(1600,100+k+1,100+k,-h,-1000+100*(k+1),-1000+100*k),Trunk(1601,200+k,200+k+1,h,-1000+100*k,-1000+100*(k+1))}).ToArray();
        }
        PreparedSegment[] Cross(float h)=>new[]{Road(1602,300,110,-200,0,-h,0),Road(1602,110,210,-h,0,h,0),Road(1602,210,301,h,0,200,0)};
        var large=Corridor(12,3).Concat(Cross(12)).ToArray();var merged=AvenueConsolidator.PrepareTrunks(large,new ImportLog(),default);
        Check(merged.All(s=>!(s.OneWay&&s.Classification=="trunk"))&&merged.Where(s=>s.ConsolidatedAvenue).All(s=>!s.OneWay&&s.Classification=="primary"&&s.Lanes==6),
            "A divided trunk with three-lane carriageways and an intersection did not become a six-lane avenue");
        Routes(large,merged,"Merging a divided trunk lost a route");
        var narrow=AvenueConsolidator.PrepareTrunks(Corridor(12,2).Concat(Cross(12)).ToArray(),new ImportLog(),default);
        Check(narrow.Where(s=>s.ConsolidatedAvenue).All(s=>!s.OneWay&&s.Classification=="trunk"&&s.Lanes==4)&&narrow.Any(s=>s.ConsolidatedAvenue),
            "A divided trunk with two-lane carriageways did not stay a four-lane highway");
        var plain=Corridor(12,3);
        Check(AvenueConsolidator.PrepareTrunks(plain,new ImportLog(),default).Count(s=>s.OneWay)==plain.Length,"A divided trunk without intersections was merged");
        // An interchange ramp at z=900 keeps that end divided. The retained
        // carriageways come in on their own half of the avenue (6 m out), not
        // its centre, and bend back onto their 15 m alignment without overshoot;
        // NetworkLaneSlots then moves the shared end onto that half.
        var ramp=new[]{Road(1603,219,400,15,900,300,900) with{Classification="motorway_link",OneWay=true,Lanes=1},
            Road(1604,400,401,300,900,300,1200) with{Classification="motorway",OneWay=true}};
        var interchange=Corridor(15,3).Concat(Cross(15)).Concat(ramp).ToArray();
        var divided=AvenueConsolidator.PrepareTrunks(interchange,new ImportLog(),default);
        var end=divided.Where(s=>s.ConsolidatedAvenue).SelectMany(s=>new[]{s.Start,s.End}).Max(p=>p.z);
        Check(end>500&&end<=700.01f,"The merged trunk did not stop clear of the interchange");
        var northbound=divided.Where(s=>s.OneWay&&s.Classification=="trunk"&&s.End.z>s.Start.z).SelectMany(s=>new[]{s.Start,s.End}).Where(p=>p.z>end+1).ToArray();
        var profile=northbound.OrderBy(p=>p.z).ToArray();
        Check(profile.Zip(profile.Skip(1),(p,q)=>q.x>=p.x-.001f).All(b=>b)&&profile.All(p=>p.x>=5.5f&&p.x<=15.01f)&&profile.Where(p=>p.z<end+15).All(p=>p.x<7.5f)&&
            profile.Where(p=>p.z>end+200).All(p=>Math.Abs(p.x-15)<.01f),"A retained carriageway did not run from its half of the avenue back to its own alignment");
        Routes(interchange,divided,"Keeping an interchange divided lost a route");
    }

    // Two tunnels on different layers whose bores lie on top of each other for
    // 600 m: the lower layer must stay a full clearance below the upper along
    // the whole overlap, not only at one point.
    static void StackedTunnels(ushort[] terrain)
    {
        PreparedSegment Bore(long way,long a,long b,float x,float z,float xx,float zz,int layer)=>Road(way,a,b,x,z,xx,zz) with{Classification="trunk",OneWay=true,Tunnel=true,Layer=layer};
        var source=new[]{Road(1600,1,2,-900,0,-300,0) with{Classification="trunk",OneWay=true},Bore(1600,2,3,-300,0,300,0,-1),Road(1600,3,4,300,0,900,0) with{Classification="trunk",OneWay=true},
            Road(1601,5,6,-900,300,-300,4) with{Classification="trunk",OneWay=true},Bore(1601,6,7,-300,4,300,-4,-2),Road(1601,7,8,300,-4,900,-300) with{Classification="trunk",OneWay=true}};
        var output=VisualNetworkCompiler.Prepare(source,terrain,100,2000,new ImportLog(),default);
        double Height(long way,float x)
        {
            var s=output.Where(s=>s.Tunnel&&(s.WayId==way||s.SourceWayIds.Contains(way))).First(s=>Math.Min(s.CourseStart.x,s.CourseEnd.x)<=x&&Math.Max(s.CourseStart.x,s.CourseEnd.x)>=x);
            var t=Enumerable.Range(0,201).Select(k=>k/200f).OrderBy(t=>Math.Abs(NetworkNormalizer.Evaluate(s,t).x-x)).First();
            return NetworkNormalizer.Evaluate(s,t).y;
        }
        foreach(var x in new[]{-200f,-100,0,100,200})
            Check(Height(1600,x)-Height(1601,x)>4,$"Stacked tunnels on layers -1 and -2 are {Height(1600,x)-Height(1601,x):F1} m apart at x={x}");
    }

    static void DividedLaneHalves()
    {
        // A six-lane avenue ending at the origin from the south divides into a
        // northbound carriageway on the east (right-hand traffic) and a
        // southbound one on the west. Each end lands on its own half, next to
        // the median, easing back to its own line by its far end.
        PreparedSegment Oneway(long way,long a,long b,float x,float z,float xx,float zz)=>Road(way,a,b,x,z,xx,zz) with{Classification="trunk",OneWay=true,Lanes=3,NativeLaneOffsets=new[]{-4f,0,4}};
        var source=new[]{Road(1700,2,1,0,-200,0,0) with{Lanes=6,ConsolidatedAvenue=true,NativeLaneOffsets=new[]{-10f,-6,-2,2,6,10}},
            Oneway(1701,1,3,0,0,8,200),Oneway(1702,4,1,-8,200,0,0)};
        var result=NetworkLaneSlots.Align(source,new ImportLog());
        var north=result[1];var south=result[2];
        Check(math.distance(north.CourseStart.xz,new float2(6,0))<.01f&&math.distance(south.CourseEnd.xz,new float2(-6,0))<.01f,
            "Divided carriageways did not meet their own halves of the two-way road");
        var northX=Enumerable.Range(0,21).Select(k=>NetworkNormalizer.Evaluate(north,k/20f).x).ToArray();var southX=Enumerable.Range(0,21).Select(k=>-NetworkNormalizer.Evaluate(south,1-k/20f).x).ToArray();
        // One long cubic carries the whole ease here, so allow a slight (0.3 m) ripple.
        bool Easing(float[] x)=>x.All(v=>v>=5.99f&&v<=8.01f)&&x.Select((v,k)=>x.Skip(k).Min()>=v-.3f).All(b=>b)&&Math.Abs(x[0]-6)<.01f&&Math.Abs(x[^1]-8)<.01f;
        Check(math.all(north.CourseEnd==source[1].End)&&Easing(northX)&&Easing(southX),
            "Divided carriageways did not ease from their halves back to their own lines");
        Check(math.all(north.Start==source[1].Start)&&north.StartNodeId==1,"Lane alignment moved the shared node");
    }

    static void OverlapFootprints(ushort[] terrain)
    {
        PreparedSegment Built(long way,long a,long b,float x,float z,float xx,float zz,float width)=>Road(way,a,b,x,z,xx,zz) with{NativeWidth=width};
        int Overlaps(PreparedSegment[] sections)=>VisualCompilerMetrics.Measure(sections,terrain,100,2000,default).Failures.Count(f=>f.Category=="same-level-overlap");
        // Wide perpendicular arms split 16 m and 24 m from their junction: the
        // continuations' flat ends meet only at the junction corner.
        var crossing=new[]{Built(1400,1,2,0,0,16,0,32),Built(1400,2,3,16,0,200,0,32),Built(1401,1,4,0,0,0,24,32),Built(1401,4,5,0,24,0,200,32),
            Built(1402,6,1,-200,0,0,0,32),Built(1403,7,1,0,-200,0,0,32)};
        Check(Overlaps(crossing)==0,"Flat section ends beside a junction were measured as overlapping");
        var stacked=new[]{Built(1410,10,11,-100,0,100,0,16),Built(1411,12,13,-100,10,100,10,16)};
        Check(Overlaps(stacked)>0,"Two unconnected carriageways drawn over each other were not measured as overlapping");
    }

    static void NativeRoundabouts(ushort[] terrain)
    {
        PreparedSegment[] Ring(float radius,float zScale=1)
        {
            float2 P(int i)=>new(radius*(float)Math.Cos(i*Math.PI/4),radius*zScale*(float)Math.Sin(i*Math.PI/4));
            var ring=Enumerable.Range(0,8).Select(i=>Road(1000+i,1000+i,1000+(i+1)%8,P(i).x,P(i).y,P(i+1).x,P(i+1).y)
                with{Roundabout=true,OneWay=true});
            var arms=Enumerable.Range(0,4).Select(i=>
            {
                var p=P(i*2);var far=p*8;
                return i%2==0?Road(1100+i,1100+i,1000+i*2,far.x,far.y,p.x,p.y) with{OneWay=true}:
                    Road(1100+i,1000+i*2,1100+i,p.x,p.y,far.x,far.y) with{OneWay=true};
            });
            return ring.Concat(arms).ToArray();
        }
        try
        {
            RoundaboutPlanner.ConfigureStockRadii(new[]{float.NaN,-1,6,12,20,32});
            var source=Ring(30);var log=new ImportLog();var native=RoundaboutPlanner.Prepare(source,log,default);
            Check(native.Count==4&&!native.Any(s=>s.Roundabout),"A stock-sized roundabout with directional approaches retained its road ring");
            var nodes=native.SelectMany(s=>new[]{(s.StartNodeId,s.StartRoundaboutRadius),(s.EndNodeId,s.EndRoundaboutRadius)}).Where(p=>p.Item2>0).ToArray();
            Check(nodes.Length==4&&nodes.All(p=>p.Item2==32)&&nodes.Select(p=>p.Item1).Distinct().Count()==1,
                "Native roundabout did not select the nearest available size including the fourth catalog size");
            var node=nodes[0].Item1;
            var output=VisualNetworkCompiler.Prepare(native,terrain,100,2000,log,default);
            Check(output.Count(s=>s.StartRoundaboutRadius>0||s.EndRoundaboutRadius>0)==4&&output.All(s=>
                (s.StartRoundaboutRadius==0||s.StartNodeId==node)&&(s.EndRoundaboutRadius==0||s.EndNodeId==node)),
                "Native roundabout markers were lost or copied onto unrelated emitted nodes");
            SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(source),output,log,default);
            Check(log.Metric("Missing mandatory selected-source ports")==0&&log.Metric("Unresolved selected-source reachability obligations")==0,
                "Native roundabout replacement lost directional approaches");
            Check(RoundaboutPlanner.Prepare(Ring(45),new ImportLog(),default).Any(s=>s.Roundabout),"An oversized circle was collapsed");
            Check(RoundaboutPlanner.Prepare(Ring(30,.4f),new ImportLog(),default).Any(s=>s.Roundabout),"An irregular circle was collapsed");
            foreach(var size in new[]{(Source:10f,Native:12f),(Source:18f,Native:20f)})
            {
                var sized=RoundaboutPlanner.Prepare(Ring(size.Source),new ImportLog(),default);
                Check(sized.All(s=>!s.Roundabout)&&sized.Any(s=>s.StartRoundaboutRadius==size.Native||s.EndRoundaboutRadius==size.Native),
                    "Different source roundabout sizes did not select different native sizes");
            }
            var structured=Ring(18).Select(s=>s.Roundabout?s with{Bridge=true,Layer=1}:s).ToArray();
            Check(RoundaboutPlanner.Prepare(structured,new ImportLog(),default).Any(s=>s.Roundabout),"An elevated ring was replaced by a ground junction");
            RoundaboutPlanner.ConfigureStockRadii(Array.Empty<float>());
            Check(RoundaboutPlanner.Prepare(source,new ImportLog(),default).Any(s=>s.Roundabout),"A ring was removed without an available native palette");
        }
        finally{RoundaboutPlanner.ConfigureStockRadii(Array.Empty<float>());}
    }

    static void GentleMajorRoad(ushort[] terrain)
    {
        var points=new[]{new float2(-200,0),new float2(0,1.4f),new float2(12,0),new float2(200,0)};
        var source=Enumerable.Range(1,points.Length-1).Select(i=>
            Road(970,970+i-1,970+i,points[i-1].x,points[i-1].y,points[i].x,points[i].y)
                with{Classification="motorway",OneWay=true}).ToArray();
        var log=new ImportLog();var output=VisualNetworkCompiler.Prepare(source,terrain,100,2000,log,default);
        var curves=output.Select(s=>new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd)).ToArray();
        Check(curves.All(c=>Enumerable.Range(0,33).All(k=>Math.Abs(NetworkDesignChecks.Evaluate(c,k/32d).z)<=1.5)),
            "Removing a tight optional bend exceeded the divided road alignment budget");
        Check(output.Count<=2,"A small survey bend fragmented a long major-road alignment");
        SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(source),output,log,default);
        Check(log.Metric("Missing mandatory selected-source ports")==0&&log.Metric("Unresolved selected-source reachability obligations")==0,
            "Smoothing a major-road survey bend lost its directed route");
    }

    static void ShortStreetStations(ushort[] terrain)
    {
        var source=new[]{Road(950,1,2,-120,0,0,0),Road(951,2,3,0,0,6,0),Road(952,3,4,6,0,120,0),Road(953,3,5,6,0,6,120)};
        source[1]=source[1] with{WayId=-951,SourceWayIds=new long[]{951}};
        var log=new ImportLog();var result=VisualNetworkCompiler.Prepare(source,terrain,100,2000,log,default);
        Check(log.Metric("Short plain street stations absorbed")==1,"A short plain street station was not absorbed");
        Check(result.Any(s=>(s.StartNodeId==3&&math.distance(s.CourseStart.xz,new float2(6,0))<.001f)||
            (s.EndNodeId==3&&math.distance(s.CourseEnd.xz,new float2(6,0))<.001f)),"Absorbing a tag station moved the real street junction");
        Check(result.Any(s=>s.SourceWayIds.Contains(951)&&s.SourceWayIds.Contains(-951)),"Absorbing a street station discarded original or prepared way evidence");
        SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(source),result,log,default);
        Check(log.Metric("Missing mandatory selected-source ports")==0&&log.Metric("Unresolved selected-source reachability obligations")==0,
            "Absorbing a plain street station lost a connected route");
    }

    static void UnitCuts()
    {
        // A chain between two junctions; other nodes crowd every position the
        // start core's cut may slide to, except the core's own sub-minimum one.
        var source=new List<PreparedSegment>{Road(10,10,11,0,0,150,0),Road(10,11,12,150,0,300,0),Road(20,10,20,0,0,0,200),Road(21,10,21,0,0,0,-200),Road(22,12,22,300,0,300,200),Road(23,12,23,300,0,300,-200)};
        for(var i=0;i<4;i++)source.Add(Road(30+i,30+i*2,31+i*2,40+i*3,1,40+i*3,120));
        var contracts=new[]{new NativeAssetContract("street",new[]{new NativeCompositionEnvelope(NativeStructureMode.Ground,0,8,0,0,0,0,4,true)},Array.Empty<float>())};
        // Highways, bridges and track take the same class minimum as streets.
        foreach(var (name,variant) in new (string,Func<PreparedSegment,PreparedSegment>)[]{("street",s=>s),
            ("motorway bridge",s=>s with{Classification="motorway",Bridge=true,Layer=1}),("rail",s=>s with{Kind=NetworkKind.Rail,Classification="rail"})})
        {
            var input=source.Select(variant).ToList();var result=NativeDesignUnits.Prepare(input,contracts,new ImportLog(),default,_=>40);            Check(result.Where(s=>s.WayId==10).All(s=>NetworkDesignChecks.HorizontalArcLength(new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd))>=40),
                $"A crowded optional unit cut fell back to a short {name} sliver");
            var log=new ImportLog();SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(input),result,log,default);
            Check(log.Metric("Missing mandatory selected-source ports")==0&&log.Metric("Unresolved selected-source reachability obligations")==0,$"Omitting a {name} sliver cut lost source ports or routes");
        }
    }
}
