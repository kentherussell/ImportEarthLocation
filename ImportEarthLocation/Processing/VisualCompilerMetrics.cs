using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Visual screening of a finished network against the per-class tolerances in
// research report 64 (§5.6, §7). Units are metres and grade fractions in local
// map XZ; heights are absolute. This is a comparison metric for pipelines, not
// a certificate or a placement veto. Each section contributes at most one
// failure per category, located at its worst sample, so long and short
// sections weigh equally and clustering decides what one visible issue is.
public static class VisualCompilerMetrics
{
    public const double IssueRadius=100;
    private const double Spacing=5;
    private const double TerrainTolerance=.3;
    private const double PairedTolerance=.3;

    public readonly record struct Tolerance(double RadiusFloor,double K,double MinimumSegment,double PairSeparation);
    public readonly record struct Failure(string Category,int Section,long WayId,long StartNodeId,long EndNodeId,
        double3 Position,double Value,double Limit,double Severity);
    public sealed record Issue(string Category,int Failures,double Severity,double Value,double Limit,double3 Position,long[] Ways);
    public readonly record struct PairedCoverage(double EligibleMeters,double MatchedMeters,double FailedMeters);
    public sealed record Measurement(int Sections,double Kilometres,PairedCoverage Paired,IReadOnlyList<Failure> Failures);
    private readonly record struct Sample(int Section,double3 Position,double2 Direction,double Grade,double Step);

    public static Tolerance For(PreparedSegment s)
    {var c=VisualCompilerTable.For(s);return new(c.RadiusFloor,c.K,c.MinimumSegment,c.PairSeparation);}

    public static Measurement Measure(IReadOnlyList<PreparedSegment> sections,ushort[] terrain,float scale,double width,CancellationToken token)
    {
        var failures=new List<Failure>();var samples=new List<Sample>();var first=new int[sections.Count+1];var kilometres=0d;
        void Fail(string category,int section,double3 position,double value,double limit,double severity)
        {var s=sections[section];failures.Add(new(category,section,s.WayId,s.StartNodeId,s.EndNodeId,position,value,limit,severity));}
        for(var i=0;i<sections.Count;i++)
        {
            token.ThrowIfCancellationRequested();first[i]=samples.Count;var s=sections[i];var curve=Curve(s);
            if(!NetworkDesignChecks.Finite(curve))continue;
            var length=NetworkDesignChecks.HorizontalArcLength(curve);if(!(length>0))continue;kilometres+=length/1000;
            var tolerance=For(s);var count=Math.Max(2,(int)Math.Ceiling(length/Spacing)+1);var previous=curve.P0;
            for(var k=0;k<count;k++)
            {
                var t=k/(count-1d);var p=NetworkDesignChecks.Evaluate(curve,t);var d=NetworkDesignChecks.Derivative(curve,t);var speed=math.length(d.xz);
                samples.Add(new(i,p,speed>1e-9?d.xz/speed:default,speed>1e-9?d.y/speed:0,math.distance(previous.xz,p.xz)));previous=p;
            }
            var own=samples.GetRange(first[i],count);
            var steepest=own.OrderByDescending(p=>Math.Abs(p.Grade)).First();var gradeLimit=NetworkGeometrySafety.GradeLimit(s);
            if(Math.Abs(steepest.Grade)>gradeLimit)Fail("steep-grade",i,steepest.Position,Math.Abs(steepest.Grade),gradeLimit,Math.Abs(steepest.Grade)/gradeLimit);
            // Grade change reported in percentage points per 10 m; the limit is 10/K.
            var rateLimit=10/(tolerance.K*(s.Tunnel?VisualCompilerTable.TunnelKShare:1));var worstRate=0d;var rateAt=own[0].Position;
            for(var k=1;k<own.Count;k++)if(own[k].Step>=1)
            {var rate=Math.Abs(own[k].Grade-own[k-1].Grade)/own[k].Step*1000;if(rate>worstRate){worstRate=rate;rateAt=own[k].Position;}}
            if(worstRate>rateLimit)Fail("grade-change",i,rateAt,worstRate,rateLimit,worstRate/rateLimit);
            if(!s.Roundabout)
            {
                var curvature=0d;var curvatureAt=own[0].Position;
                for(var k=0;k<count;k++)
                {var value=Math.Abs(NetworkDesignChecks.Curvature(curve,k/(count-1d)));if(value>curvature){curvature=value;curvatureAt=own[k].Position;}}
                if(curvature*tolerance.RadiusFloor>1)Fail("tight-radius",i,curvatureAt,1/curvature,tolerance.RadiusFloor,curvature*tolerance.RadiusFloor);
                if(length<tolerance.MinimumSegment)Fail("short-segment",i,own[count/2].Position,length,tolerance.MinimumSegment,tolerance.MinimumSegment/Math.Max(1,length));
            }
            if(!NetworkStructure.IsStructure(s))
            {
                var worst=own.Select(p=>(p.Position,Delta:Math.Abs(p.Position.y-NetworkProcessor.Sample((float)p.Position.x,(float)p.Position.z,terrain,scale,width))))
                    .OrderByDescending(p=>p.Delta).First();
                if(worst.Delta>TerrainTolerance)Fail("terrain-mismatch",i,worst.Position,worst.Delta,TerrainTolerance,worst.Delta/TerrainTolerance);
            }
        }
        first[sections.Count]=samples.Count;
        Nodes(sections,Fail,token);
        var grid=new Dictionary<(int,int),List<int>>();
        (int,int) Cell(double3 p)=>((int)Math.Floor(p.x/32),(int)Math.Floor(p.z/32));
        for(var k=0;k<samples.Count;k++){var cell=Cell(samples[k].Position);if(!grid.TryGetValue(cell,out var list))grid[cell]=list=new();list.Add(k);}
        IEnumerable<int> Near(double3 p,double radius)
        {
            var r=(int)Math.Ceiling(radius/32);var (cx,cz)=Cell(p);
            for(var x=cx-r;x<=cx+r;x++)for(var z=cz-r;z<=cz+r;z++)if(grid.TryGetValue((x,z),out var list))foreach(var k in list)yield return k;
        }
        // Height of b's centreline at a's position, extrapolated along b's own grade.
        double HeightAt(Sample b,double3 p)=>b.Position.y+b.Grade*math.dot(p.xz-b.Position.xz,b.Direction);
        var paired=Paired(sections,samples,first,Near,HeightAt,Fail,token);
        Clearance(sections,samples,first,Near,HeightAt,Fail,token);
        return new(sections.Count,kilometres,paired,failures);
    }

    // Groups failures of one category into visible issues. Candidates are already
    // the same category and at most IssueRadius apart; SameIssue decides the rest.
    public static IReadOnlyList<Issue> Cluster(IReadOnlyList<Failure> failures,CancellationToken token)
    {
        var parent=Enumerable.Range(0,failures.Count).ToArray();
        int Root(int i){while(parent[i]!=i)i=parent[i]=parent[parent[i]];return i;}
        var grid=new Dictionary<(string,int,int),List<int>>();
        for(var i=0;i<failures.Count;i++)
        {
            token.ThrowIfCancellationRequested();var f=failures[i];var cx=(int)Math.Floor(f.Position.x/IssueRadius);var cz=(int)Math.Floor(f.Position.z/IssueRadius);
            for(var x=cx-1;x<=cx+1;x++)for(var z=cz-1;z<=cz+1;z++)if(grid.TryGetValue((f.Category,x,z),out var list))
                foreach(var j in list)if(math.distance(failures[j].Position.xz,f.Position.xz)<=IssueRadius&&SameIssue(failures[j],f))parent[Root(i)]=Root(j);
            if(!grid.TryGetValue((f.Category,cx,cz),out var own))grid[(f.Category,cx,cz)]=own=new();own.Add(i);
        }
        return Enumerable.Range(0,failures.Count).GroupBy(Root).Select(g=>
        {
            var members=g.Select(i=>failures[i]).ToArray();var worst=members.OrderByDescending(f=>f.Severity).ThenBy(f=>f.Section).First();
            return new Issue(worst.Category,members.Length,worst.Severity,worst.Value,worst.Limit,worst.Position,members.Select(f=>f.WayId).Distinct().OrderBy(w=>w).ToArray());
        }).OrderByDescending(i=>i.Severity).ThenBy(i=>i.Category,StringComparer.Ordinal).ToArray();
    }

    // One visible issue follows one road: the same OSM way or a section touching
    // it end to end. Mere proximity would chain across dense grids and hide
    // distinct defects. Paired heights are the exception: the opposite
    // carriageway is another way, side by side, and shows the same defect.
    private static bool SameIssue(in Failure a,in Failure b)=>
        a.WayId==b.WayId||a.StartNodeId==b.StartNodeId||a.StartNodeId==b.EndNodeId||a.EndNodeId==b.StartNodeId||a.EndNodeId==b.EndNodeId||
        a.Category=="paired-height"&&math.distance(a.Position.xz,b.Position.xz)<=60;

    private static void Nodes(IReadOnlyList<PreparedSegment> sections,Action<string,int,double3,double,double,double> fail,CancellationToken token)
    {
        var arms=new Dictionary<SourceNodeKey,List<(int Section,bool Start)>>();
        for(var i=0;i<sections.Count;i++)foreach(var start in new[]{true,false})
        {
            var key=new SourceNodeKey(sections[i].Kind,start?sections[i].StartNodeId:sections[i].EndNodeId);
            if(!arms.TryGetValue(key,out var list))arms[key]=list=new();list.Add((i,start));
        }
        foreach(var node in arms.OrderBy(p=>p.Key.Kind).ThenBy(p=>p.Key.Id))
        {
            token.ThrowIfCancellationRequested();var list=node.Value;
            var away=list.Select(a=>Away(sections[a.Section],a.Start)).ToArray();if(away.Any(a=>!a.Valid))continue;
            var position=(double3)(list[0].Start?sections[list[0].Section].CourseStart:sections[list[0].Section].CourseEnd);
            if(list.Count==2)
            {
                // A continuous road leaves a degree-two node opposite to how it arrived.
                var deflection=math.degrees(Math.Acos(math.clamp(-math.dot(away[0].Direction,away[1].Direction),-1,1)));
                if(deflection>2)fail("tangent-break",list[1].Section,position,deflection,2,deflection/2);
                var kink=Math.Abs(away[0].Grade+away[1].Grade)*100;var limit=Math.Min(10/For(sections[list[0].Section]).K,10/For(sections[list[1].Section]).K);
                if(sections[list[0].Section].Tunnel&&sections[list[1].Section].Tunnel)limit/=VisualCompilerTable.TunnelKShare;
                if(kink>limit)fail("grade-change",list[1].Section,position,kink,limit,kink/limit);
                continue;
            }
            if(node.Key.Kind!=NetworkKind.Road)continue;
            for(var a=0;a<list.Count;a++)for(var b=a+1;b<list.Count;b++)
            {
                var x=sections[list[a].Section];var y=sections[list[b].Section];if(x.ProtectedLink||y.ProtectedLink)continue;
                // A two-way road splitting into its one-way carriageways diverges at a small angle by design.
                if(x.OneWay&&y.OneWay&&list[a].Start!=list[b].Start)continue;
                var angle=math.degrees(Math.Acos(math.clamp(math.dot(away[a].Direction,away[b].Direction),-1,1)));
                if(angle<25)fail("acute-junction",list[b].Section,position,angle,25,25/Math.Max(1,angle));
            }
        }
    }

    // Opposite one-way carriageways of one class within that class's separation
    // band, regardless of whether the design paired them (report 43: 4% coverage).
    private static PairedCoverage Paired(IReadOnlyList<PreparedSegment> sections,List<Sample> samples,int[] first,
        Func<double3,double,IEnumerable<int>> near,Func<Sample,double3,double> heightAt,Action<string,int,double3,double,double,double> fail,CancellationToken token)
    {
        bool Candidate(PreparedSegment s)=>s.Kind==NetworkKind.Road&&s.OneWay&&!s.ProtectedLink&&!s.Roundabout&&For(s).PairSeparation>0;
        var eligible=0d;var matched=0d;var failed=0d;
        for(var i=0;i<sections.Count;i++)
        {
            var s=sections[i];if(!Candidate(s))continue;token.ThrowIfCancellationRequested();var band=For(s).PairSeparation;
            var worst=0d;var worstAt=default(double3);
            for(var k=first[i];k<first[i+1];k++)
            {
                var a=samples[k];eligible+=a.Step;var best=double.PositiveInfinity;var delta=0d;
                foreach(var j in near(a.Position,band))
                {
                    var b=samples[j];var o=sections[b.Section];
                    if(b.Section==i||!Candidate(o)||o.Classification!=s.Classification||o.Layer!=s.Layer||math.dot(a.Direction,b.Direction)>-.966)continue;
                    var offset=b.Position.xz-a.Position.xz;var along=math.dot(offset,a.Direction);var lateral=Math.Abs(NetworkDesignChecks.Cross(a.Direction,offset));
                    if(Math.Abs(along)>Spacing||lateral<5||lateral>band||lateral>=best)continue;
                    best=lateral;delta=Math.Abs(a.Position.y-heightAt(b,a.Position));
                }
                if(double.IsPositiveInfinity(best))continue;matched+=a.Step;
                if(delta>PairedTolerance){failed+=a.Step;if(delta>worst){worst=delta;worstAt=a.Position;}}
            }
            if(worst>0)fail("paired-height",i,worstAt,worst,PairedTolerance,worst/PairedTolerance);
        }
        return new(eligible,matched,failed);
    }

    // Deck-to-deck height over a lower-layer or tunnel section whose footprint the
    // bridge overlaps. Clearance includes deck depth: 5.5 m over road, 7 m over rail.
    private static void Clearance(IReadOnlyList<PreparedSegment> sections,List<Sample> samples,int[] first,
        Func<double3,double,IEnumerable<int>> near,Func<Sample,double3,double> heightAt,Action<string,int,double3,double,double,double> fail,CancellationToken token)
    {
        static double Half(PreparedSegment s)=>(s.NativeWidth>0?s.NativeWidth:8)/2;
        // Sections within two graph steps of the bridge connect to it (a ramp
        // leaving at its abutment); their overlap is a junction, not a crossing.
        var incident=new Dictionary<SourceNodeKey,List<int>>();
        for(var i=0;i<sections.Count;i++)foreach(var key in new SourceNodeKey[]{new(sections[i].Kind,sections[i].StartNodeId),new(sections[i].Kind,sections[i].EndNodeId)})
        {if(!incident.TryGetValue(key,out var list))incident[key]=list=new();list.Add(i);}
        IEnumerable<int> Touching(int i)=>new SourceNodeKey[]{new(sections[i].Kind,sections[i].StartNodeId),new(sections[i].Kind,sections[i].EndNodeId)}.SelectMany(k=>incident[k]);
        for(var i=0;i<sections.Count;i++)
        {
            var s=sections[i];if(!s.Bridge||s.Tunnel)continue;token.ThrowIfCancellationRequested();
            var adjacent=new HashSet<int>(Touching(i).SelectMany(Touching));
            var ends=new HashSet<SourceNodeKey>{new(s.Kind,s.StartNodeId),new(s.Kind,s.EndNodeId)};var worst=double.PositiveInfinity;var worstAt=default(double3);var worstLimit=0d;
            for(var k=first[i];k<first[i+1];k++)
            {
                var a=samples[k];
                foreach(var j in near(a.Position,Half(s)+15))
                {
                    var b=samples[j];var o=sections[b.Section];if(b.Section==i||adjacent.Contains(b.Section))continue;
                    if(!(o.Layer<s.Layer||o.Tunnel)||ends.Contains(new(o.Kind,o.StartNodeId))||ends.Contains(new(o.Kind,o.EndNodeId)))continue;
                    if(math.distance(a.Position.xz,b.Position.xz)>Half(s)+Half(o)+Spacing/2)continue;
                    var gap=a.Position.y-heightAt(b,a.Position);var limit=(o.Kind==NetworkKind.Rail?7:5.5)-.5;
                    if(gap<limit&&gap<worst){worst=gap;worstAt=a.Position;worstLimit=limit;}
                }
            }
            if(!double.IsPositiveInfinity(worst))fail("crossing-clearance",i,worstAt,worst,worstLimit,worstLimit/Math.Max(.5,worst));
        }
        // Same-level overlap: road footprints of unconnected sections on one layer
        // overlapping by more than 0.5 m within 3 m of height, e.g. carriageways
        // drawn on top of each other. Rail is excluded: parallel tracks touch.
        // Each sample is a disc of half the road width, except that a section ends
        // flat where it continues within 30° into exactly one other road: there
        // the disc is clipped at the end line. Junction, dead-end and sharp-bend
        // discs stay whole, standing in for node pavement, end caps and mitres.
        var along=new double[samples.Count];
        for(var i=0;i<sections.Count;i++)for(var k=first[i]+1;k<first[i+1];k++)along[k]=along[k-1]+samples[k].Step;
        double2 Outward(int section,long id)=>sections[section].StartNodeId==id?-samples[first[section]].Direction:samples[first[section+1]-1].Direction;
        bool Plain(int section,long id)
        {
            var arms=incident[new(sections[section].Kind,id)];if(arms.Count!=2||arms[0]==arms[1]||first[arms[0]]==first[arms[0]+1]||first[arms[1]]==first[arms[1]+1])return false;
            var other=arms[0]==section?arms[1]:arms[0];
            return math.dot(Outward(section,id),-Outward(other,id))>Math.Cos(Math.PI/6);
        }
        var plainStart=new bool[sections.Count];var plainEnd=new bool[sections.Count];
        for(var i=0;i<sections.Count;i++)if(sections[i].Kind==NetworkKind.Road&&first[i]<first[i+1])
        {plainStart[i]=Plain(i,sections[i].StartNodeId);plainEnd[i]=Plain(i,sections[i].EndNodeId);}
        double Reach(int k,double2 u)
        {
            var a=samples[k];var s=sections[a.Section];var h=Half(s);var reach=h;
            void Clip(double e,double2 outward)
            {
                var c=math.dot(u,outward);
                if(e<h&&h*c>e)reach=Math.Min(reach,e*c+Math.Sqrt(h*h-e*e)*Math.Sqrt(Math.Max(0,1-c*c)));
            }
            if(plainStart[a.Section])Clip(along[k],Outward(a.Section,s.StartNodeId));
            if(plainEnd[a.Section])Clip(along[first[a.Section+1]-1]-along[k],Outward(a.Section,s.EndNodeId));
            return reach;
        }
        for(var i=0;i<sections.Count;i++)
        {
            var s=sections[i];if(s.Kind!=NetworkKind.Road)continue;token.ThrowIfCancellationRequested();
            var adjacent=new HashSet<int>(Touching(i).SelectMany(Touching));var worst=0d;var worstAt=default(double3);
            for(var k=first[i];k<first[i+1];k++)
            {
                var a=samples[k];
                foreach(var j in near(a.Position,Half(s)+15))
                {
                    var b=samples[j];var o=sections[b.Section];if(b.Section<=i||o.Kind!=NetworkKind.Road||o.Layer!=s.Layer||adjacent.Contains(b.Section))continue;
                    var distance=math.distance(a.Position.xz,b.Position.xz);
                    if(Half(s)+Half(o)-distance<=.5)continue;
                    var u=distance>1e-6?(b.Position.xz-a.Position.xz)/distance:default;
                    var overlap=distance>1e-6?Reach(k,u)+Reach(j,-u)-distance:Half(s)+Half(o);
                    if(overlap>.5&&Math.Abs(a.Position.y-heightAt(b,a.Position))<3&&overlap>worst){worst=overlap;worstAt=a.Position;}
                }
            }
            if(worst>0)fail("same-level-overlap",i,worstAt,worst,.5,worst/.5);
        }
    }

    private static Cubic3d Curve(PreparedSegment s)=>new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);

    private static (bool Valid,double2 Direction,double Grade) Away(PreparedSegment s,bool start)
    {
        var c=Curve(s);var d=start?(c.P1-c.P0):(c.P2-c.P3);
        if(math.lengthsq(d.xz)<1e-6)d=start?(c.P2-c.P0):(c.P1-c.P3);
        var horizontal=math.length(d.xz);
        return horizontal<1e-6?(false,default,0):(true,d.xz/horizontal,d.y/horizontal);
    }
}
