using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

static class RockTests
{
    static void Check(bool value,string message) { if(!value)throw new Exception(message); }
    public static void Run()
    {
        double Degrees(float m)=>m/6378137d*180/Math.PI;
        OsmNode Node(long id,float x,float z)=>new(id,Degrees(z),Degrees(x));
        var request=new ImportRequest(0,0,new("test","test",600,false),false){ImportRocks=true};
        var flat=new TerrainBuilder.Result(Enumerable.Repeat((ushort)32768,256).ToArray(),100,0,0,100);
        // 16×16 raster over 600 m: 40 m per pixel. Rising east by 2,100 units of
        // 1,000/65,535 m per pixel is a slope of about 0.8 (39°).
        var steep=new TerrainBuilder.Result(Enumerable.Range(0,256).Select(i=>(ushort)(1000+i%16*2100)).ToArray(),1000,0,0,1000);
        IReadOnlyDictionary<string,string> Tags(string natural)=>new Dictionary<string,string>{{"natural",natural}};
        OsmDataset Area(string natural,float half)
        {
            var nodes=new Dictionary<long,OsmNode>{{1,Node(1,-half,-half)},{2,Node(2,half,-half)},{3,Node(3,half,half)},{4,Node(4,-half,half)}};
            return new OsmDataset{Nodes=nodes,Ways=new Dictionary<long,OsmWay>{{7,new OsmWay(7,new long[]{1,2,3,4,1},Tags(natural))}}};
        }
        var none=Array.Empty<PreparedSegment>();var noGeography=Array.Empty<GeographicPolygon>();var water=new WaterPlan(false,0,0,0,"test");
        IReadOnlyList<VegetationCandidate> Plan(OsmDataset data,TerrainBuilder.Result terrain,IReadOnlyList<PreparedSegment>? roads=null,ImportRequest? r=null)=>
            RockPlanner.Prepare(data,r??request,terrain,roads??none,noGeography,water,new ImportLog(),default);
        bool Rock(VegetationCandidate c)=>c.Form>=VegetationForm.RockSmall;
        // Only boulders, in piles: every boulder overlaps another (2.6 m apart
        // against 4.4 m and up across), and no small rocks.
        void Clumped(IReadOnlyList<VegetationCandidate> rocks,string what)
        {
            Check(rocks.Count>=3,what+" has fewer than three boulders");
            Check(rocks.All(c=>c.Form==VegetationForm.RockLarge),what+" placed small rocks");
            foreach(var b in rocks)
                Check(rocks.Where(o=>!o.Equals(b)).Min(o=>math.distance(o.Position.xz,b.Position.xz))<=2.61f,what+" has a boulder not touching another");
            for(var i=0;i<rocks.Count;i++)for(var j=i+1;j<rocks.Count;j++)
                Check(math.distance(rocks[i].Position.xz,rocks[j].Position.xz)>=.8f,what+" has rocks closer than the 0.8 m spacing");
        }

        var boulder=new OsmDataset{Nodes=new Dictionary<long,OsmNode>{{1,Node(1,10,20)}},NodeTags=new Dictionary<long,IReadOnlyDictionary<string,string>>{{1,Tags("stone")}}};
        var single=Plan(boulder,flat);
        Check(single.Count is >=5 and <=10&&math.distance(single[0].Position.xz,new float2(10,20))<.5f,"Mapped boulder is not a pile of boulders at its node");
        Clumped(single,"Mapped boulder");
        Check(Plan(boulder,flat,r:request with{ImportRocks=false}).Count==0,"Rocks planned with the option off");
        // Water sources reject rocks within radius + 2 m: a small source in its own
        // broad-phase square, a huge one listed for every square, and one far away.
        RiverSourcePlan Source(float x,float z,float radius)=>new(1,"","test",new(x,0,z),radius,1,0,default);
        int Near(params RiverSourcePlan[] sources)=>RockPlanner.Prepare(boulder,request,flat,none,noGeography,water with{Sources=sources},new ImportLog(),default).Count;
        Check(Near(Source(12,21,4))==0&&Near(Source(-90000,20,90100))==0,"A rock was placed inside a water source");
        Check(Near(Source(250,250,5))==single.Count,"A distant water source rejected rocks");

        // No scatter on flat rock or gentle scrub, hilltops included.
        Check(Plan(Area("bare_rock",100),flat).Count==0,"Rocks scattered across flat bare rock");
        Check(Plan(Area("scree",100),flat).Count==0,"Rocks scattered across flat scree");
        Check(Plan(Area("scrub",100),flat).Count==0,"Rocks scattered across gentle scrub");

        // A rocky slope collects clumps at its foot, downhill (west).
        var slope=Plan(Area("bare_rock",150),steep);
        Check(slope.Count>0&&slope.All(Rock),"A rocky slope collected no clumps");
        Clumped(slope,"Rocky slope");
        // The test slope never levels out, so clumps settle the full 120 m downhill.
        Check(slope.Average(c=>c.Position.x)<-60,"Rocky-slope clumps did not settle downhill of the rock");
        Check(slope.SequenceEqual(Plan(Area("bare_rock",150),steep)),"Rock placement is nondeterministic");
        // The density slider keeps whole clumps: fewer rocks, each still touching a boulder.
        var half=Plan(Area("bare_rock",150),steep,r:request with{RockDensity=.5f});
        Check(half.Count>0&&half.Count<slope.Count&&half.All(c=>slope.Contains(c)),"Half rock density did not keep a subset of the full plan");
        Clumped(half,"Half-density slope");
        Check(Plan(Area("bare_rock",150),steep,r:request with{RockDensity=0}).Count==0,"Zero rock density still placed rocks");

        var road=new PreparedSegment(10,1,2,new float3(-300,0,-120),new float3(300,0,-120),NetworkKind.Road,"primary",2,false,false,false,0);
        Check(Plan(Area("bare_rock",150),steep,new[]{road}).All(c=>Math.Abs(c.Position.z+120)>4),"Rock placed on a road");
        Check(Plan(Area("bare_rock",150),steep with{SeaLevelMeters=2000}).Count==0,"Rocks placed under the sea");

        // A north–south cliff on terrain rising east: clumps lie west, downhill,
        // a few per 200 m of cliff rather than a talus band.
        var cliffNodes=new Dictionary<long,OsmNode>{{1,Node(1,0,-100)},{2,Node(2,0,100)}};
        var cliff=new OsmDataset{Nodes=cliffNodes,Ways=new Dictionary<long,OsmWay>{{9,new OsmWay(9,new long[]{1,2},Tags("cliff"))}}};
        var talus=Plan(cliff,steep);
        Check(talus.Count>5&&talus.Count(c=>c.Position.x<0)>=.75*talus.Count,"Cliff clumps are not mostly on the downhill side");
        Check(talus.Count<=8*21,"A 200 m cliff collected more than a handful of clumps");
        Clumped(talus,"Cliff foot");
    }
}
