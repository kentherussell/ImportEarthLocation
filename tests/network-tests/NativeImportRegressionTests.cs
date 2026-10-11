using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Newtonsoft.Json.Linq;
using Unity.Mathematics;

// Shared production decisions. These do not execute the game's network generator.
internal static class NativeImportRegressionTests
{
    private static void Check(bool ok,string message){if(!ok)throw new Exception(message);}
    private static PreparedSegment Line(long way,long a,long b,float x,float end)=>
        new(way,a,b,new(x,0,0),new(end,0,0),NetworkKind.Road,"secondary",2,false,false,false,0)
        {Name="Route",SourceWayIds=new[]{way},SourceGeometry=new[]{new float3(x,0,0),new float3(end,0,0)}};
    private static IReadOnlyList<PreparedSegment> Compact(params PreparedSegment[] source)=>NativeChainCompactor.Prepare(source,new ImportLog(),default);
    public static void Run()
    {
        var a=Line(10,1,2,0,40);var b=Line(11,2,3,40,41.6f) with{ExplicitLanes=true};
        var joined=Compact(a,b);
        Check(joined.Count==1&&joined[0].StartNodeId==1&&joined[0].EndNodeId==3,"Compatible 1.6m artificial boundary was not compacted");
        Check(joined[0].ExplicitLanes&&joined[0].SourceWayIds.Order().SequenceEqual(new long[]{10,11}),"Compaction lost explicit lane evidence or source ways");
        Check(a.EndNodeId==2&&b.StartNodeId==2,"Compaction mutated source records");
        Check(Compact(joined.ToArray()).SequenceEqual(joined),"Compaction is not idempotent");
        Check(NetworkNormalizer.WithinTolerance(joined[0]),"Accumulated geographic error exceeds approved tolerance");
        var branch=Line(12,2,4,40,40) with{End=new float3(40,0,30),SourceGeometry=Array.Empty<float3>()};
        Check(Compact(a,b,branch).Count==3,"Real third arm was contracted");
        var boundaries=new[]{b with{Lanes=4},b with{SourceSpeedKph=60},b with{Bridge=true},b with{Tunnel=true},
            b with{StartTerminal=true},b with{StartMiniRoundabout=true},b with{StartRoundaboutRadius=10},
            b with{StartCurveOffset=new float3(1,0,0)},b with{NativeLaneOffsets=new[]{2f}},b with{RailService="yard"},
            b with{SourceTags=new Dictionary<string,string>{{"access","private"}}},
            b with{RawSourceTags=new Dictionary<string,string>{{"surface","gravel"}}},
            b with{RawSourceTags=new Dictionary<string,string>{{"turn:lanes","left|through"}}}};
        foreach(var boundary in boundaries)Check(Compact(a,boundary).Count==2,"Structure, marker, lane or source-semantic boundary lost");
        Check(Compact(a with{OneWay=true},NetworkNormalizer.Reverse(b) with{OneWay=true}).Count==2,"Opposing one-way directions combined");
        Check(Compact(a,NetworkNormalizer.Reverse(b)).Count==1,"Reversed two-way source chain lost connectivity");
        var rail=b with{Kind=NetworkKind.Rail,Classification="rail"};
        Check(Compact(a,rail).Count==2,"Coincident road and rail source identities conflated");
        PreparedSegment Arc(long start,long end,float from,float to)
        {
            float3 P(float t)=>new(20000+20*math.cos(t),0,-20000+20*math.sin(t));
            var h=20*4f/3*math.tan((to-from)/4);
            var s=Line(20,start,end,0,0) with{Start=P(from),End=P(to),OneWay=true,Roundabout=true,
                Control1=P(from)+new float3(-math.sin(from)*h,0,math.cos(from)*h),
                Control2=P(to)-new float3(-math.sin(to)*h,0,math.cos(to)*h),SourceGeometry=Array.Empty<float3>()};
            return s with{SourceGeometry=Enumerable.Range(0,33).Select(i=>NetworkNormalizer.Evaluate(s,i/32f)).ToArray()};
        }
        var arcs=Compact(Arc(101,102,0,.08f),Arc(102,103,.08f,.4f));
        Check(arcs.Count==1&&!arcs.Any(s=>s.StartNodeId==102||s.EndNodeId==102),"Dense roundabout curve boundary retained at map-scale coordinates");
        Check(Compact(a,a with{StartNodeId=2,EndNodeId=1,Start=a.End,End=a.Start,SourceGeometry=a.SourceGeometry.Reverse().ToArray()}).Count==2,"Closed loop collapsed");

        var disconnected=Line(99,90,91,900,950);var c=Line(13,3,5,41.6f,80);
        var plan=new NativeSubmissionPlan(new[]{a,disconnected,c,b,branch});
        Check(plan.Sections.Count==5&&plan.Sections[0]==a&&plan.Sections[1]==b&&plan.Sections[2]==branch,"Connected submission order lost identity frontier");
        Check(plan.Sections.ToHashSet().SetEquals(new[]{a,b,c,branch,disconnected}),"Submission scheduling removed/changed geometry");
        Check(plan.GatherPendingJunction(1,1,128,new[]{"Road:2"})==2,"Recovery did not gather pending incident arms");
        Check(plan.Sections[0]==a&&plan.Sections.Count==5,"Recovery replayed committed prefix or dropped a section");
        Check(plan.GatherPendingJunction(-1,1,128,new[]{"Road:2"})==0&&plan.GatherPendingJunction(1,1,1,new[]{"Road:2"})==0,
            "Invalid or oversized recovery star accepted");
        Console.WriteLine("PASS: artificial curve compaction, source provenance, semantic/structure/branch guards, direction, loop preservation and source-identity pending-only recovery.");
    }

    public static void ReplayCompaction(string folder)
    {
        var path=Path.Combine(folder,"native-plan-network.ndjson");var meta=JObject.Parse(File.ReadAllText(path+".meta.json"));
        Check((bool?)meta["Truncated"]==false,"Complete archived plan required");
        // The capture is after grading. Reconstruct horizontal pregrading inputs,
        // preserving all routing/width/structure/tag evidence; not a native replay.
        float3 Flat(float3 p)=>new(p.x,0,p.z);
        var source=File.ReadLines(path).Select(line=>
        {
            var row=JObject.Parse(line);Check((bool?)row["SourcePointsTruncated"]!=true,"Truncated source geometry");
            var s=row.ToObject<PreparedSegment>()??throw new Exception("Invalid archived section");
            return s with{Start=Flat(s.Start),End=Flat(s.End),Control1=s.Control1.HasValue?Flat(s.Control1.Value):null,
                Control2=s.Control2.HasValue?Flat(s.Control2.Value):null,SourceGeometry=s.SourceGeometry.Select(Flat).ToArray(),
                OriginalGeometry=s.OriginalGeometry.Select(Flat).ToArray()};
        }).ToArray();
        Check(source.Length==(int?)meta["Records"],"Archive record count mismatch");
        var folderName=Path.GetFileName(folder);
        long target=folderName.StartsWith("20261010T003348")?12362042:
            folderName.StartsWith("20261010T005122")?11113948351:
            folderName.StartsWith("20261010T004853")?283481966:throw new Exception("Unrecognized diagnosed fixture");
        string Key(PreparedSegment s,long id)=>NativeEndpointPolicy.Identity(s,id);
        var incidence=source.SelectMany(s=>new[]{Key(s,s.StartNodeId),Key(s,s.EndNodeId)}).GroupBy(k=>k).ToDictionary(g=>g.Key,g=>g.Count());
        Check(incidence["Road:"+target]==2,"Fixture must reproduce artificial degree-two boundary");
        var repaired=NativeChainCompactor.Prepare(source,new ImportLog(),default);
        var retained=repaired.SelectMany(s=>new[]{Key(s,s.StartNodeId),Key(s,s.EndNodeId)}).ToHashSet();
        if(retained.Contains("Road:"+target))
        {
            var originalPair=source.Where(s=>s.Kind==NetworkKind.Road&&(s.StartNodeId==target||s.EndNodeId==target)).ToArray();
            Console.WriteLine($"Diagnostic: original-pair compaction {originalPair.Length} -> {Compact(originalPair).Count}");
            foreach(var s in repaired.Where(s=>s.Kind==NetworkKind.Road&&(s.StartNodeId==target||s.EndNodeId==target)))
                Console.WriteLine(Newtonsoft.Json.JsonConvert.SerializeObject(DiagnosticArtifacts.NetworkRecords(new[]{s}).First()));
        }
        Check(!retained.Contains("Road:"+target),"Diagnosed artificial source junction remains: "+target);
        foreach(var real in incidence.Where(p=>p.Value>=3))Check(retained.Contains(real.Key),"True archived branch removed: "+real.Key);
        var originalWays=source.SelectMany(s=>s.SourceWayIds.Append(s.WayId)).ToHashSet();
        Check(originalWays.SetEquals(repaired.SelectMany(s=>s.SourceWayIds.Append(s.WayId))),"Archived compaction lost source-way provenance");
        foreach(var structure in source.Where(s=>s.Bridge||s.Tunnel))Check(repaired.Contains(structure),"Archived structure/portal edited");
        Check(NativeChainCompactor.Prepare(repaired,new ImportLog(),default).SequenceEqual(repaired),"Archived compaction is not idempotent");
        Console.WriteLine($"PASS: complete {folderName} horizontal archive {source.Length} -> {repaired.Count} curves; diagnosed node {target} removed; all true branches, structures and source ways retained. Offline pregrading evidence, NOT native engine acceptance.");
    }

    public static void ReplayLocalCompaction(string folder)
    {
        // Marseille's old 64MB geometry capture is truncated. Do not process it
        // as a complete graph or claim preservation of unseen incident branches.
        const long target=12362042;
        var path=Path.Combine(folder,"native-plan-network.ndjson");
        Check((bool?)JObject.Parse(File.ReadAllText(path+".meta.json"))["Truncated"]==true,"Local fixture expects explicitly incomplete evidence");
        float3 Flat(float3 p)=>new(p.x,0,p.z);
        var pair=File.ReadLines(path).Select(JObject.Parse).Where(r=>(long)r["StartNodeId"]! ==target||(long)r["EndNodeId"]! ==target)
            .Select(row=>row.ToObject<PreparedSegment>()!).Select(s=>s with{Start=Flat(s.Start),End=Flat(s.End),
                Control1=s.Control1.HasValue?Flat(s.Control1.Value):null,Control2=s.Control2.HasValue?Flat(s.Control2.Value):null,
                SourceGeometry=s.SourceGeometry.Select(Flat).ToArray(),OriginalGeometry=s.OriginalGeometry.Select(Flat).ToArray()}).ToArray();
        Check(pair.Length==2&&pair.All(s=>s.WayId==3617372),"Missing diagnosed Marseille pieces");
        var repaired=Compact(pair);
        Check(repaired.Count==1&&!repaired.Any(s=>s.StartNodeId==target||s.EndNodeId==target),"Marseille isolated diagnosed boundary not compacted");
        Check(NetworkNormalizer.WithinTolerance(repaired[0]),"Marseille local fit exceeded geographic bounds");
        Console.WriteLine("PASS: Marseille diagnosed curve-pair local fit removes node12362042 within unchanged tolerance. INCOMPLETE graph evidence: unseen branches and whole-map/native success are NOT verified.");
    }
}
