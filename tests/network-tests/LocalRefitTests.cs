using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

static class LocalRefitTests
{
    static void Check(bool value,string message){if(!value)throw new Exception(message);}
    public static void Run()
    {
        Check(NetworkPersistenceHash.Digest(new[]{"edge-b","edge-a"})==NetworkPersistenceHash.Digest(new[]{"edge-a","edge-b"}),"Network digest depends on native enumeration order");
        Check(NetworkPersistenceHash.Digest(new[]{"a|b","c"})!=NetworkPersistenceHash.Digest(new[]{"a","b|c"})&&NetworkPersistenceHash.Digest(new[]{"edge-a"})!=NetworkPersistenceHash.Digest(new[]{"edge-a","edge-a"}),"Persistence hash lost record boundaries or duplicate edge coverage");
        var source=new PreparedSegment(233491784,1,2,new(-100,100,0),new(100,100,0),NetworkKind.Road,"motorway",2,true,true,false,1)
            {NativeWidth=16,NativeUnitId=1};
        var terrain=Enumerable.Repeat((ushort)32768,17*17).ToArray();var assets=new Dictionary<string,NativeAssetContract>();
        var poses=new Dictionary<SourceNodeKey,NativeJunctionReconciliation.Port>{{new(NetworkKind.Road,1),new(new(-92,101,0),new(1,0),.01,true)}};
        var refit=NativeJunctionReconciliation.Refit(new[]{source},poses,terrain,200,2000,assets,new ImportLog(),default);
        Check(refit.NativeSafe&&!refit.NeedsTerrainRestamp&&refit.ChangedSections==1,"Eight-metre owned structural port did not receive one safe local refit");
        var curve=refit.Sections.Single();
        var originalAnchor=new NativeBatchBindings.Anchor<string>("owned",source.Start,curve.Start){CenterLimit=10};
        var revisionAnchor=NativeBatchBindings.LocalRevisionAnchor(originalAnchor,curve.Start);
        Check(math.all(NativeBatchBindings.RebaseBound(curve,revisionAnchor,null).Start==curve.Start)&&math.all(originalAnchor.Planned==source.Start),"Revised attachment lost original provenance or rejected its new course datum");
        var excessive=false;try{NativeBatchBindings.LocalRevisionAnchor(originalAnchor with{Generated=source.Start+new float3(11,0,0)},source.Start+new float3(11,0,0));}catch(NativeSafetyException){excessive=true;}
        Check(excessive,"Local refit silently moved the original source footprint budget");
        Check(math.all(curve.Start==poses[new(NetworkKind.Road,1)].Position)&&math.all(curve.End==source.End),"Local refit moved a fixed native or future-neighbour boundary");
        var slope=(curve.CourseControl1.y-curve.CourseStart.y)/math.length((curve.CourseControl1-curve.CourseStart).xz);
        Check(Math.Abs(slope-.01)<1e-6,"Actual native receiver grade was penalized rather than fixed");
        var gradeOnly=NativeJunctionReconciliation.Refit(new[]{source},new Dictionary<SourceNodeKey,NativeJunctionReconciliation.Port>{{new(NetworkKind.Road,1),new(source.Start,new(1,0),.02,true)}},terrain,200,2000,assets,new ImportLog(),default);
        var g=gradeOnly.Sections[0];
        Check(gradeOnly.ChangedSections==1&&Math.Abs((g.CourseControl1.y-g.CourseStart.y)/math.length((g.CourseControl1-g.CourseStart).xz)-.02)<1e-6,"An unchanged node position hid a changed native receiver grade");
        Check(source.Start.x==-100&&source.Start.y==100&&source.Control1==null,"Local refit mutated its frozen source");
        var plan=new NativeSubmissionPlan(new[]{source});plan.ReplaceUnissued(0,refit.Sections);
        Check(plan.Batch(0,32).Single().Start.x==-92,"Unissued unit revision was not published to submission");
        var rejected=false;try{plan.ReplaceUnissued(0,new[]{curve with{StartNodeId=999}});}catch(InvalidOperationException){rejected=true;}
        Check(rejected&&plan.Sections[0].StartNodeId==1,"Revision changed source ownership or partially published invalid input");
        var ground=NativeJunctionReconciliation.Refit(new[]{source with{Bridge=false}},poses,terrain,200,2000,assets,new ImportLog(),default);
        Check(ground.NeedsTerrainRestamp,"Ground geometry was declared publishable without owned terrain restamping");
        var invalid=NativeJunctionReconciliation.Refit(new[]{source},new Dictionary<SourceNodeKey,NativeJunctionReconciliation.Port>{{new(NetworkKind.Road,1),new(new(float.NaN),new(1,0),0,true)}},terrain,200,2000,assets,new ImportLog(),default);
        Check(!invalid.NativeSafe&&ReferenceEquals(invalid.Sections[0],source),"Nonfinite observed pose escaped safety or destroyed original safe geometry");
        Console.WriteLine("PASS: eight-metre unissued structural refit, exact outer position/grade, immutable source, atomic submission revision, nonfinite safety and explicit ground-restamp requirement.");
    }
}
