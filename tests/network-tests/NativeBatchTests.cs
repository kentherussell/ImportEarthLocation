using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Newtonsoft.Json.Linq;
using Unity.Mathematics;

// Exercise the production matcher/rebaser, not a duplicate test-only algorithm.
internal static class NativeBatchTests
{
    private static readonly Dictionary<string,string> Empty = new(StringComparer.Ordinal);
    private static PreparedSegment Section(long a=1,long b=2,NetworkKind kind=NetworkKind.Road) =>
        new(10,a,b,new float3(0,182.917435f,0),new float3(40,181.47049f,0),kind,
            kind==NetworkKind.Road?"primary":"rail",2,false,false,false,0)
        { Control1=new float3(10,182.5f,0),Control2=new float3(30,182,0) };
    private static NativeBatchBindings.Edge<string> Edge(PreparedSegment s,string entity="edge",string a="a",string b="b",string prefab="stock") =>
        new(entity,a,b,s.Start-new float3(0,.83197f,0),s.End-new float3(0,.32787f,0),s.Kind,prefab);
    private static NativeBatchBindings.Batch<string> Match(PreparedSegment s,IReadOnlyList<NativeBatchBindings.Edge<string>> edges,
        IReadOnlyDictionary<string,string>? committed=null) =>
        NativeBatchBindings.Match(new[]{new NativeBatchBindings.Expected<string>(s,"stock")},edges,committed??Empty);
    private static void Check(bool condition,string message){if(!condition)throw new Exception(message);}
    private static void Refuses(Action action,string expected,string message)
    {
        try{action();}catch(InvalidOperationException e){Check(e.Message.Contains(expected,StringComparison.Ordinal),message+": wrong rejection: "+e.Message);return;}
        throw new Exception(message+": unsafe graph accepted");
    }

    public static void Run()
    {
        Check(NativeBatchBindings.Target("preview","null",false,false)=="preview","New native entity should survive as itself");
        Check(NativeBatchBindings.Target("preview","original",true,false)=="original","Ordinary native update must bind Original");
        Check(NativeBatchBindings.Target("preview","original",true,true)=="preview","Native replacement must not bind the deleted Original");
        var s=Section();var edge=Edge(s);var result=Match(s,new[]{edge});
        Check(result.Edges.Count==1&&result.Anchors.Count==2,"Valid native height adjustment lost coverage");
        Check(result.Anchors["Road:1"].Entity=="a"&&math.all(result.Anchors["Road:1"].Generated==edge.Start),"Source identity did not retain actual native anchor");
        Check(!math.all(edge.Start==s.Start)&&math.all(edge.Start.xz==s.Start.xz),"Regression fixture must reproduce the old XYZ readback failure");
        var reversed=edge with{StartNode=edge.EndNode,EndNode=edge.StartNode,Start=edge.End,End=edge.Start};
        result=Match(s,new[]{reversed});
        Check(!result.Edges[0].Forward&&result.Anchors["Road:1"].Entity=="a","Reversed native edge changes source direction");
        Check(Match(s,new[]{edge,edge with{Entity="parallel",StartNode="p",EndNode="q",Start=edge.Start+new float3(0,0,.1f),End=edge.End+new float3(0,0,.1f)}}).Edges.Count==1,"Nearby parallel road confused exact source pair");
        Check(Match(s,new[]{edge,edge with{Entity="rail",Kind=NetworkKind.Rail},edge with{Entity="asset",Prefab="wrong-stock"}}).Edges.Count==1,"Rail or unselected asset confused road binding");
        Refuses(()=>Match(s,Array.Empty<NativeBatchBindings.Edge<string>>()),"0 qualified","Missing generated edge");
        Refuses(()=>Match(s,new[]{edge with{Prefab="wrong-stock"}}),"0 qualified","Wrong selected native asset");
        Refuses(()=>Match(s,new[]{edge with{Kind=NetworkKind.Rail}}),"0 qualified","Rail crossing is not a road junction");
        Refuses(()=>Match(s,new[]{edge with{Start=edge.Start+new float3(.001f,0,0)}}),"0 qualified","Horizontal drift must not snap to nearest road");
        Refuses(()=>Match(s,new[]{edge,edge with{Entity="ambiguous",StartNode="other-a",EndNode="other-b"}}),"2 qualified","Ambiguous coincident asset pair");
        Refuses(()=>Match(s,new[]{edge with{Start=new float3(float.NaN,0,0)}}),"non-finite","Non-finite generated endpoint");
        Refuses(()=>Match(s,new[]{edge with{End=new float3(40,float.PositiveInfinity,0)}}),"non-finite","Non-finite native height");
        Refuses(()=>Match(s,new[]{edge},new Dictionary<string,string>{{"Road:1","wrong-node"}}),"0 qualified","Known source identity must not migrate to another entity");
        Refuses(()=>Match(s,new[]{edge},new Dictionary<string,string>{{"Road:99","a"}}),"merged unrelated","A generated node cannot steal a different committed identity");
        Refuses(()=>Match(s,new[]{edge},new Dictionary<string,string>{{"Road:99","old"},{"Road:100","old"}}),"conflicting source","Previously aliased committed identities");
        Refuses(()=>NativeBatchBindings.Match(new[]{new NativeBatchBindings.Expected<string>(s,"stock"),new NativeBatchBindings.Expected<string>(s,"stock")},new[]{edge},Empty),"same native edge","Duplicate edge coverage");

        var next=s with{WayId=11,StartNodeId=2,EndNodeId=3,Start=s.End,End=new float3(80,181,0)};
        var nextEdge=Edge(next,"next","b","c") with{Start=edge.End};
        var connected=NativeBatchBindings.Match(new[]{new NativeBatchBindings.Expected<string>(s,"stock"),new NativeBatchBindings.Expected<string>(next,"stock")},new[]{edge,nextEdge},Empty);
        Check(connected.Anchors.Count==3&&connected.Anchors["Road:2"].Entity=="b","Shared junction split within one batch");
        Refuses(()=>NativeBatchBindings.Match(new[]{new NativeBatchBindings.Expected<string>(s,"stock"),new NativeBatchBindings.Expected<string>(next,"stock")},new[]{edge,nextEdge with{StartNode="duplicate-b"}},Empty),"0 qualified","Shared source junction generated as two nodes");
        Refuses(()=>NativeBatchBindings.Match(new[]{new NativeBatchBindings.Expected<string>(s,"stock"),new NativeBatchBindings.Expected<string>(next,"stock")},new[]{edge,nextEdge with{Start=nextEdge.Start+new float3(0,.1f,0)}},Empty),"conflicting anchors","One native entity at conflicting positions");
        var unrelated=next with{StartNodeId=99};
        Refuses(()=>NativeBatchBindings.Match(new[]{new NativeBatchBindings.Expected<string>(s,"stock"),new NativeBatchBindings.Expected<string>(unrelated,"stock")},new[]{edge,nextEdge},Empty),"merged unrelated","Distinct source nodes cannot become a false connection");

        var committed=connected.Anchors.ToDictionary(p=>p.Key,p=>p.Value.Entity,StringComparer.Ordinal);
        Check(Match(next,new[]{nextEdge},committed).Anchors["Road:2"].Entity=="b","Cross-batch junction lost explicit identity");
        var replacement=NativeBatchBindings.RemapCommitted(committed,new Dictionary<string,string>{{"b","replacement-b"}});
        Check(Match(next,new[]{nextEdge with{StartNode="replacement-b"}},replacement).Anchors["Road:2"].Entity=="replacement-b"&&committed["Road:2"]=="b","Native replacement source remap mutated old bindings or lost lineage");
        var portal=next with{Bridge=true,Layer=1,StartElevation=0,EndElevation=8,RetainingWallSides=NetworkSides.Left,StartCurveOffset=new float3(2,0,0)};
        Check(Match(portal,new[]{nextEdge},committed).Edges.Count==1,"Shared ground-to-bridge portal blocked by structure identity");
        Check(Match(portal with{Bridge=false,Tunnel=true,Layer=-1},new[]{nextEdge},committed).Edges.Count==1,"Shared ground-to-tunnel portal blocked");
        var rail=next with{Kind=NetworkKind.Rail};
        Check(Match(rail,new[]{nextEdge with{Kind=NetworkKind.Rail,StartNode="rail-b",EndNode="rail-c"}},committed).Anchors["Rail:2"].Entity=="rail-b","Same road/rail numeric source ID must not join modes");

        var rebased=NativeBatchBindings.Rebase(portal,edge.End,nextEdge.End);
        Check(math.all(rebased.Start==edge.End)&&math.all(rebased.Control1!.Value==portal.Control1!.Value+(edge.End-portal.Start)),"Committed anchor adjustment did not translate adjacent handle");
        Check(math.all(rebased.Control2!.Value==portal.Control2!.Value+(nextEdge.End-portal.End)),"End handle not translated");
        Check(math.all(rebased.StartCurveOffset==portal.StartCurveOffset)&&rebased.Bridge&&rebased.Layer==1&&rebased.RetainingWallSides==NetworkSides.Left&&rebased.StartElevation==portal.StartElevation&&rebased.Lanes==portal.Lanes,"Rebasing changed unrelated native selection or structure evidence");
        Check(math.all(portal.Start==s.End)&&!ReferenceEquals(portal,rebased),"Rebase mutated archived plan");
        Check(NativeBatchBindings.Rebase(portal,null,null)==portal,"No committed anchor should leave plan unchanged");
        Refuses(()=>NativeBatchBindings.Rebase(portal,portal.Start+new float3(.01f,0,0),null),"moved horizontally","Committed horizontal drift");
        Refuses(()=>NativeBatchBindings.Rebase(portal,null,new float3(80,float.NaN,0)),"non-finite","Committed invalid height");
        var straight=s with{Control1=null,Control2=null};
        var straightRebase=NativeBatchBindings.Rebase(straight,edge.Start,edge.End);
        Check(straightRebase.Control1==null&&straightRebase.Control2==null&&math.all(math.isfinite(straightRebase.CourseControl1)),"Implicit straight control handles lost during rebase");
        var midpoint=new float3(20,182,0);
        var splitA=edge with{Entity="split-a",EndNode="subdivision",End=midpoint};
        var splitB=edge with{Entity="split-b",StartNode="subdivision",Start=midpoint};
        var split=Match(s,new[]{splitB,splitA});
        Check(split.SectionCount==1&&split.Edges.Count==2&&split.Nodes.Count==3&&split.Anchors.Count==2,"Native subdivision not verified as one complete submitted span");
        Refuses(()=>Match(s,new[]{splitA}),"0 qualified","Incomplete subdivided span");
        Refuses(()=>Match(s,new[]{splitA,splitB with{StartNode="disconnected"}}),"0 qualified","Spatially touching split edges with different node entities are not connected");
        Refuses(()=>Match(s,new[]{edge,splitA,splitB}),"2 qualified","Direct edge and split path both present are ambiguous duplicate coverage");
        Refuses(()=>Match(s,new[]{splitA with{End=midpoint+new float3(0,0,1)},splitB with{Start=midpoint+new float3(0,0,1)}}),"0 qualified","Alternate route is not an anonymous native subdivision of submitted course");
        var bent=s with{Control1=new float3(10,182.5f,30),Control2=new float3(30,182,30)};
        var curvedMidpoint=new float3(20,182,22.5f);
        Check(Match(bent,new[]{splitA with{End=curvedMidpoint},splitB with{Start=curvedMidpoint}}).Edges.Count==2,"Curved-course subdivision mistaken for a straight-line detour");
        Refuses(()=>Match(s,new[]{splitA,splitB},new Dictionary<string,string>{{"Road:999","subdivision"}}),"0 qualified","Cannot route a submitted span through another committed source junction");
        var longSource=s with{Start=new float3(0,182,0),End=new float3(40960,182,0),Control1=null,Control2=null};
        var longChain=Enumerable.Range(0,2048).Select(i=>new NativeBatchBindings.Edge<string>("long-edge-"+i,"long-node-"+i,"long-node-"+(i+1),
            new float3(i*20,181,0),new float3((i+1)*20,181,0),NetworkKind.Road,"stock")).ToArray();
        Check(Match(longSource,longChain).Edges.Count==2048,"Long legitimate native chain did not complete without recursive stack growth");
        SimulateBatches(53060);SimulateBatches(17999);
        LaterBatchRegressions();
        OverlapCoalescenceRegressions();
        ManifestRoundTrip();
        Console.WriteLine("PASS: production native batch bindings recover height-only drift, preserve portals/mode/source identity and control handles, reject missing/ambiguous/parallel/aliased/non-finite graphs; 71,059 synthetic sections complete across 556 batches (not a native-game import).");
    }

    private static void LaterBatchRegressions()
    {
        var s=Section() with{Start=new float3(15099.667f,229.35846f,-11503.515f),End=new float3(15083.043f,229.27856f,-11503.26f),Control1=null,Control2=null,NativeWidth=32};
        var center=new float3(15094.125f,229.31899f,-11503.43f);
        var old=new NativeBatchBindings.Anchor<string>("committed",s.Start,s.Start-new float3(0,.03947f,0));
        var bound=new Dictionary<string,string>{{"Road:1","committed"}};
        var edge=new NativeBatchBindings.Edge<string>("new-edge","committed","new-end",center,s.End,s.Kind,"stock"){CurveStart=s.CourseStart,CurveEnd=s.CourseEnd};
        var valid=Match(s,new[]{edge},bound);
        Check(math.all(valid.Anchors["Road:1"].Generated==center),"Native center regeneration lost exact submitted curve-port identity");
        Refuses(()=>Match(s,new[]{edge with{CurveStart=null,CurveEnd=null}},bound),"0 qualified","Center displacement without native curve evidence");
        Refuses(()=>Match(s,new[]{edge with{StartNode="wrong-owner"}},bound),"0 qualified","Curve evidence must not bypass committed ownership");
        Refuses(()=>Match(s,new[]{edge with{Start=center+new float3(50,0,0)}},bound),"0 qualified","Center moved beyond native footprint");
        Refuses(()=>Match(s,new[]{edge with{CurveStart=s.Start+new float3(.01f,0,0)}},bound),"0 qualified","Nearby curve port is not exact submitted port");
        Refuses(()=>Match(s,new[]{edge with{CurveEnd=null}},bound),"incomplete curve","Partial native curve evidence");
        var narrow=s with{NativeWidth=12};var widerCenter=center-new float3(5,0,0);
        var wideJunctionEdge=edge with{Start=widerCenter};
        Refuses(()=>Match(narrow,new[]{wideJunctionEdge},bound),"0 qualified","Narrow road may not authorize unproven large center displacement");
        var wideJunction=NativeBatchBindings.Match(new[]{new NativeBatchBindings.Expected<string>(narrow,"stock"){StartCenterBudget=18}},new[]{wideJunctionEdge},bound);
        var sharedAnchor=wideJunction.Anchors["Road:1"] with{CenterLimit=18};
        Check(math.all(NativeBatchBindings.RebaseBound(narrow,sharedAnchor,null).CourseStart.xz==narrow.CourseStart.xz),"Widest incident source footprint must consistently authorize continuation onto a narrower arm");
        var anchor=valid.Anchors["Road:1"];
        var continuation=NativeBatchBindings.RebaseBound(s,anchor,null);
        Check(math.all(continuation.Start==center)&&math.all(continuation.CourseStart.xz==s.CourseStart.xz)&&math.all(continuation.CourseControl1.xz==s.CourseControl1.xz),"Continuation must distinguish native center from fixed source curve ports/handles");
        Check(math.all(continuation.CourseControl2.xz==s.CourseControl2.xz)&&math.all(continuation.CourseEnd.xz==s.CourseEnd.xz),"Rebasing one junction moved the other port");
        Check(Match(continuation,new[]{edge},bound).Edges.Count==1,"Actual rebased course manifest cannot match generated curve ports");
        Refuses(()=>NativeBatchBindings.RebaseBound(s,anchor with{Planned=anchor.Planned+new float3(1,0,0)},null),"provenance","Cannot authorize rebase with unrelated source point");
        var next=s with{WayId=11,StartNodeId=2,EndNodeId=3,Start=s.End,End=new float3(14863.046f,229.17683f,-11500.373f),Control1=null,Control2=null};
        var bypass=new NativeBatchBindings.Edge<string>("bypass","committed","next-end",center,next.End,s.Kind,"stock"){CurveStart=next.CourseStart,CurveEnd=next.CourseEnd};
        Refuses(()=>NativeBatchBindings.Match(new[]{new NativeBatchBindings.Expected<string>(s,"stock"),new NativeBatchBindings.Expected<string>(next,"stock")},new[]{edge,bypass},bound),"0 qualified","Captured Palm Springs bypass still must be rejected before Apply");
        var joined=bypass with{StartNode="new-end",Start=s.End};
        Check(NativeBatchBindings.Match(new[]{new NativeBatchBindings.Expected<string>(s,"stock"),new NativeBatchBindings.Expected<string>(next,"stock")},new[]{edge,joined},bound).Anchors.Count==3,"Corrected later-batch incidence failed");
        Check(NativePreviewRecovery.NextBatchSize(128,0,false,true)==32&&NativePreviewRecovery.NextBatchSize(32,1,false,true)==1&&NativePreviewRecovery.NextBatchSize(1,2,false,true)==0,"Recovery must change context; repeating an identical single-section preview cannot repair it");
        Check(NativePreviewRecovery.NextBatchSize(128,0,true,true)==0&&NativePreviewRecovery.NextBatchSize(128,0,false,false)==0&&NativePreviewRecovery.NextBatchSize(1,3,false,true)==0,"Applied, unclassified or exhausted failures may not be replayed");
        Console.WriteLine("PASS: later-batch 5.54 m center/curve-port separation, bound continuation, correct selected entity/asset, captured bypass rejection and bounded pre-commit-only retries.");
    }

    private static void OverlapCoalescenceRegressions()
    {
        // 2026-10-09 log-derived failure shapes. Correct source flags must
        // prevent these previews; the matcher must NOT start accepting them
        // just to make old captured batches or an import report success.
        var section=Section() with{NativeWidth=32};
        var owner=new Dictionary<string,string>{{"Road:1","owned-start"}};
        var unchanged=Edge(section,"continuation","owned-start","new-end") with{CurveStart=section.CourseStart,CurveEnd=section.CourseEnd};
        Check(Match(section,new[]{unchanged},owner).SectionCount==1,"Protected continuation onto an unchanged committed owner rejected");
        foreach(var displacement in new[]{new float3(-7.59277f,0,-2.5205f),new float3(-15.81836f,0,2.4023f)})
        {
            var recentered=unchanged with{Start=unchanged.Start+displacement,CurveStart=section.CourseStart+displacement};
            Refuses(()=>Match(section,new[]{recentered},owner),"0 qualified","Moved owner AND moved curve port may not be accepted as valid continuation");
        }
        var shortSection=section with{Start=new float3(14622.9277f,182,-6361.80225f),End=new float3(14541.6309f,182,-6362.53027f),Control1=null,Control2=null};
        var mergedEnd=new float3(14420.4307f,181,-6363.607f);
        var merged=new NativeBatchBindings.Edge<string>("merged-202m","owned-start","different-owned-junction",shortSection.Start,mergedEnd,NetworkKind.Road,"stock")
            {CurveStart=shortSection.CourseStart,CurveEnd=mergedEnd};
        var owners=new Dictionary<string,string>{{"Road:1","owned-start"},{"Road:99","different-owned-junction"}};
        Refuses(()=>Match(shortSection,new[]{merged},owners),"0 qualified","Native 81m-to-202m extension deleting an intermediate source junction must remain unsafe");
        Console.WriteLine("PASS: unchanged explicit continuation accepted; log-derived owner/port movement and overextended known-junction bypass rejected. This validates matcher safety, not native merge prevention or whole-import completion.");
    }

    private static void SimulateBatches(int sectionCount)
    {
        var committed=new Dictionary<string,string>(StringComparer.Ordinal);var live=new Dictionary<string,float3>(StringComparer.Ordinal);
        var total=0;var batches=0;
        float3 Position(int id) => new(-24000+(id%800)*60,200,-24000+(id/800)*60);
        string Node(NetworkKind kind,int id)=>$"{kind}-entity-{id}";
        float3 Generated(float3 point,int id)=>point+new float3(0,-.832f+(id%17)*.05f,0);
        for(var first=0;first<sectionCount;first+=128)
        {
            var expected=new List<NativeBatchBindings.Expected<string>>();var actual=new List<NativeBatchBindings.Edge<string>>();
            for(var i=first;i<Math.Min(first+128,sectionCount);i++)
            {
                var kind=i%2000<1000?NetworkKind.Road:NetworkKind.Rail;
                var source=Section(i,i+1,kind) with{WayId=i+1,Start=Position(i),End=Position(i+1),Control1=null,Control2=null};
                var aKey=NativeEndpointPolicy.Identity(source,source.StartNodeId);var bKey=NativeEndpointPolicy.Identity(source,source.EndNodeId);
                var submitted=NativeBatchBindings.Rebase(source,live.TryGetValue(aKey,out var oldA)?oldA:null,live.TryGetValue(bKey,out var oldB)?oldB:null);
                if(committed.ContainsKey(aKey))Check(math.all(submitted.Start==live[aKey]),"Later course did not reuse committed height");
                var a=Generated(source.Start,i);var b=Generated(source.End,i+1);
                expected.Add(new(source,"stock"));actual.Add(new("edge-"+i,Node(kind,i),Node(kind,i+1),a,b,kind,"stock"));
            }
            // Native archetype enumeration is not submission order.
            actual.Reverse();var result=NativeBatchBindings.Match(expected,actual,committed);
            Check(result.Edges.Count==expected.Count,"Many-batch import lost section coverage");
            foreach(var p in result.Anchors)
            {if(committed.TryGetValue(p.Key,out var prior))Check(prior==p.Value.Entity,"Batch boundary spawned duplicate node");committed[p.Key]=p.Value.Entity;live[p.Key]=p.Value.Generated;}
            total+=result.Edges.Count;batches++;
        }
        Check(total==sectionCount&&batches==(sectionCount+127)/128,"Whole many-batch handoff model did not complete");
    }

    public static void Replay(string folder)
    {
        var manifests=Directory.GetFiles(folder,"native-batch-*-expected.ndjson").OrderBy(p=>p,StringComparer.Ordinal).ToArray();
        if(manifests.Length>0){ReplayManifests(folder,manifests);return;}
        var source=File.ReadLines(Path.Combine(folder,"native-plan-network.ndjson")).Take(128).Select(ParseSection).ToArray();
        Check(source.Length==128,"Captured first batch is incomplete");
        var generated=new List<NativeBatchBindings.Edge<string>>();
        foreach(var line in File.ReadLines(Path.Combine(folder,"last-native-temp-batch.ndjson")))
        {
            var row=JObject.Parse(line);if(row["edge"] is not JObject e)continue;
            Check((string?)row["original"]=="0:0","Replay is first-create-batch only; cannot infer unrecorded node originals for later batches");
            var kind=((string?)row["prefabName"]??"").Contains("Track",StringComparison.OrdinalIgnoreCase)?NetworkKind.Rail:NetworkKind.Road;
            generated.Add(new((string)row["entity"]!,(string)e["start"]!,(string)e["end"]!,Point(e["startPosition"]!),Point(e["endPosition"]!),kind,(string)row["prefab"]!));
        }
        // The old archive records generated assets, not the resolver's expected
        // prefab IDs. Qualify replay geometry first; production additionally
        // requires the actual resolver result (wrong-asset fixtures above).
        var expected=new List<NativeBatchBindings.Expected<string>>();
        foreach(var s in source)
        {
            var candidates=generated.Where(e=>e.Kind==s.Kind&&(math.all(e.Start.xz==s.Start.xz)||math.all(e.End.xz==s.Start.xz))).Select(e=>e.Prefab).Distinct().ToArray();
            var qualified=new List<string>();
            foreach(var prefab in candidates)
            {
                try{NativeBatchBindings.Match(new[]{new NativeBatchBindings.Expected<string>(s,prefab)},generated,Empty);qualified.Add(prefab);}
                catch(InvalidOperationException){/* No unique complete chain with this recorded asset. */}
            }
            Check(qualified.Count==1,$"Replay lacks uniquely recorded chain asset: way={s.WayId}; candidates={candidates.Length}; qualified={qualified.Count}");
            expected.Add(new(s,qualified[0]));
        }
        var bindings=NativeBatchBindings.Match(expected,generated,Empty);
        var changed=bindings.Anchors.Values.Count(a=>!math.all(a.Planned==a.Generated));
        Check(bindings.SectionCount==128&&bindings.Edges.Count>=128&&changed>0,"Real regression did not recover height-adjusted first-batch graph");
        Console.WriteLine($"PASS: captured {Path.GetFileName(folder)} first batch: {bindings.SectionCount}/128 source sections, {bindings.Edges.Count} native edges; {bindings.Anchors.Count} source anchors, {changed} differ from prepared XYZ, {bindings.Nodes.Count-bindings.Anchors.Count} native subdivision nodes. Native simulation and resolver asset selection are not replayed.");
    }

    private static float3 Point(JToken token)=>new((float)token["x"]!,(float)token["y"]!,(float)token["z"]!);
    private static PreparedSegment ParseSection(string line)
    {
        var s=JObject.Parse(line);return new((long)s["WayId"]!,(long)s["StartNodeId"]!,(long)s["EndNodeId"]!,Point(s["Start"]!),Point(s["End"]!),
            Enum.Parse<NetworkKind>((string)s["Kind"]!),(string)s["Classification"]!,(int)s["Lanes"]!,(bool)s["OneWay"]!,(bool)s["Bridge"]!,(bool)s["Tunnel"]!,(int)s["Layer"]!)
        {Control1=s["Control1"] is JObject c1?Point(c1):null,Control2=s["Control2"] is JObject c2?Point(c2):null,
            StartCurveOffset=s["StartCurveOffset"] is JObject a?Point(a):float3.zero,EndCurveOffset=s["EndCurveOffset"] is JObject b?Point(b):float3.zero,
            NativeWidth=(float?)s["NativeWidth"]??0,ConsolidatedAvenue=(bool?)s["ConsolidatedAvenue"]??false,
            Name=(string?)s["Name"]??"",Reference=(string?)s["Reference"]??"",Roundabout=(bool?)s["Roundabout"]??false,
            StartMiniRoundabout=(bool?)s["StartMiniRoundabout"]??false,EndMiniRoundabout=(bool?)s["EndMiniRoundabout"]??false,
            SourceWayIds=s["SourceWayIds"]?.Values<long>().ToArray()??Array.Empty<long>(),
            SourceGeometry=s["SourceGeometry"]?.Select(Point).ToArray()??Array.Empty<float3>(),OriginalGeometry=s["OriginalGeometry"]?.Select(Point).ToArray()??Array.Empty<float3>()};
    }

    public static void ReplayJunctionPlan(string folder)
    {
        var network=Path.Combine(folder,"native-plan-network.ndjson");
        var meta=JObject.Parse(File.ReadAllText(network+".meta.json"));
        if((bool?)meta["Truncated"]??true)throw new InvalidOperationException("Incomplete junction-plan artifact: "+network);
        var source=File.ReadLines(network).Select(line=>
        {
            if((bool?)JObject.Parse(line)["SourcePointsTruncated"]??false)throw new InvalidOperationException("Incomplete junction source geometry: "+network);
            return ParseSection(line);
        }).ToArray();
        Check(source.Length==(int)meta["Records"]!,"Junction-plan metadata count differs from recorded sections");
        var palm=source.FirstOrDefault(s=>s.WayId==1220190819&&s.StartNodeId==-7999999999999999208&&s.EndNodeId==-7999999999999999209);
        var oman=source.FirstOrDefault(s=>s.WayId==183177670&&s.StartNodeId==-7999999999999998872&&s.EndNodeId==-7999999999999999839);
        var link=palm??oman??throw new Exception("Archive does not contain either diagnosed later-batch median junction.");
        var arms=source.Where(s=>s.ConsolidatedAvenue&&(s.StartNodeId==link.StartNodeId||s.EndNodeId==link.StartNodeId||s.StartNodeId==link.EndNodeId||s.EndNodeId==link.EndNodeId)).ToArray();
        var log=new ImportLog();var repaired=AvenueJunctionRepair.Prepare(source,log,default);
        Check(!repaired.Any(s=>s.WayId==link.WayId&&s.StartNodeId==link.StartNodeId&&s.EndNodeId==link.EndNodeId),"Real archived connector was not recovered");
        var canonical=Math.Min(link.StartNodeId,link.EndNodeId);
        Check(repaired.Any(s=>s.ConsolidatedAvenue&&(s.StartNodeId==canonical||s.EndNodeId==canonical)),"Repaired archived junction lost its avenue arms");
        foreach(var arm in arms.Where(s=>s.StartNodeId!=link.StartNodeId&&s.StartNodeId!=link.EndNodeId||s.EndNodeId!=link.StartNodeId&&s.EndNodeId!=link.EndNodeId))
            Check(repaired.Any(s=>s.WayId==arm.WayId&&(s.StartNodeId==canonical||s.EndNodeId==canonical)),"An archived approach lost its canonical junction");
        Check(!repaired.Any(s=>s.StartNodeId==s.EndNodeId),"Archived topology repair created a self-loop");
        Check(AvenueJunctionRepair.Prepare(repaired,new ImportLog(),default).SequenceEqual(repaired),"Real archived repair is not idempotent");
        Console.WriteLine($"PASS: {Path.GetFileName(folder)} real complete archived plan ({source.Length} sections) recovers diagnosed way {link.WayId}; repaired sections={repaired.Count}. Offline topology evidence only; archived native preview is not regenerated and terrain must be reaudited in production.");
        foreach(var line in log.Lines.Where(s=>s.Contains("connectorWay="+link.WayId+";",StringComparison.Ordinal)))Console.WriteLine(line);
    }

    private static void ReplayManifests(string folder,IReadOnlyList<string> manifests)
    {
        var accepted=0;var rejected=0;var pending=0;
        foreach(var manifest in manifests)
        {
            var stem=Path.GetFileName(manifest).Replace("-expected.ndjson","");
            var graphPath=Path.Combine(folder,stem+"-graph.ndjson");var ownersPath=Path.Combine(folder,stem+"-owners.ndjson");
            if(!File.Exists(graphPath)||!File.Exists(ownersPath)){pending++;continue;}
            foreach(var path in new[]{manifest,graphPath,ownersPath})
            {
                var meta=path+".meta.json";
                if(File.Exists(meta)&&((bool?)JObject.Parse(File.ReadAllText(meta))["Truncated"]??false))throw new InvalidOperationException("Incomplete batch artifact: "+path);
            }
            var expected=File.ReadLines(manifest).Select(line=>{var row=JObject.Parse(line);return new NativeBatchBindings.Expected<string>(ParseSection(row["section"]!.ToString()),(string)row["prefab"]!)
                {StartCenterBudget=(float?)row["StartCenterBudget"]??0,EndCenterBudget=(float?)row["EndCenterBudget"]??0};}).ToArray();
            var graph=File.ReadLines(graphPath).Select(line=>{var row=JObject.Parse(line);return new NativeBatchBindings.Edge<string>((string)row["entity"]!,(string)row["startNode"]!,(string)row["endNode"]!,Point(row["start"]!),Point(row["end"]!),Enum.Parse<NetworkKind>((string)row["mode"]!),(string)row["prefab"]!)
                {CurveStart=row["curveStart"] is JObject a?Point(a):null,CurveEnd=row["curveEnd"] is JObject b?Point(b):null};}).ToArray();
            var owners=File.ReadLines(ownersPath).Select(JObject.Parse).ToDictionary(row=>(string)row["source"]!,row=>(string)row["entity"]!,StringComparer.Ordinal);
            var wasRejected=File.ReadLines(Path.Combine(folder,"checkpoints.ndjson")).Select(JObject.Parse).Any(row=>(string?)row["phase"]=="native-batch/preview-rejected"&&(string?)row["details"]?["artifactStem"]==stem);
            try
            {
                var result=NativeBatchBindings.Match(expected,graph,owners);
                Check(!wasRejected,"Previously rejected captured preview now accepted without regeneration: "+stem);
                accepted++;Check(result.SectionCount==expected.Length,"Captured batch source coverage incomplete");
            }
            catch(NativePreviewException){Check(wasRejected,"Captured accepted preview unexpectedly rejected: "+stem);rejected++;}
        }
        Check(accepted+rejected>0,"No complete batch manifests to replay");
        Console.WriteLine($"PASS: selected-asset/submitted-course/owner replay: accepted={accepted}; correctly rejected previews={rejected}; incomplete/pending manifests={pending}. Not a native-game import.");
    }

    private static void ManifestRoundTrip()
    {
        var root=Path.Combine(Path.GetTempPath(),"iel-native-manifest-"+Guid.NewGuid().ToString("N"));
        try
        {
            var archive=new RunArchive(root,new{fixture=true},typeof(NativeBatchTests).Assembly.Location);
            var s=Section() with{NativeWidth=32};
            void Write(string stem,bool valid)
            {
                archive.WriteRecords(stem+"-expected.ndjson",new object[]{new{kind="expected-course",prefab="stock",StartCenterBudget=18,EndCenterBudget=18,section=DiagnosticArtifacts.NetworkRecords(new[]{s}).First()}});
                archive.WriteRecords(stem+"-owners.ndjson",new object[]{new{source="Road:1",entity="committed"}});
                archive.WriteRecords(stem+"-graph.ndjson",valid?new object[]{new{entity="edge",startNode="committed",endNode="new-end",start=DiagnosticArtifacts.Point(s.Start+new float3(2,-.5f,0)),end=DiagnosticArtifacts.Point(s.End),
                    mode="Road",prefab="stock",curveStart=DiagnosticArtifacts.Point(s.CourseStart),curveEnd=DiagnosticArtifacts.Point(s.CourseEnd)}}:Array.Empty<object>());
                if(!valid)archive.Checkpoint("native-batch/preview-rejected",new{artifactStem=stem,applied=false});
            }
            Write("native-batch-000128-128-attempt-00",false);
            Write("native-batch-000128-032-attempt-01",true);
            Replay(archive.DirectoryPath);
            Check(Directory.GetFiles(archive.DirectoryPath,"*-graph.ndjson").Length==2,"Successful retry overwrote failed graph evidence");
            var path=Path.Combine(archive.DirectoryPath,"native-batch-000128-032-attempt-01-graph.ndjson");
            archive.WriteRecords(Path.GetFileName(path),new object[]{new{oversized=true}},maximumBytes:1);
            Refuses(()=>Replay(archive.DirectoryPath),"Incomplete batch artifact","Truncated captured graph may not establish coverage");
        }
        finally{if(Directory.Exists(root))Directory.Delete(root,true);}
        Console.WriteLine("PASS: synthetic diagnostic manifest/owner/graph roundtrip retains rejected and accepted attempts independently; truncated evidence cannot claim coverage.");
    }
}
