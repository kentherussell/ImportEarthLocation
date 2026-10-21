using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

static class TunnelContactTests
{
    private static void Check(bool value,string message){if(!value)throw new Exception(message);}
    public static void Run()
    {
        var terrain=Enumerable.Repeat((ushort)32768,17*17).ToArray();const float scale=200;const double width=2000;
        var tunnel=new PreparedSegment(1,1,2,new(-600,80,0),new(600,80,0),NetworkKind.Road,"motorway",2,true,false,true,-1)
            {NativeWidth=16,NativeAssetKey="tunnel"};
        var envelope=new NativeCompositionEnvelope(NativeStructureMode.Tunnel,0,16,0,1,-2,3,4,true);
        var assets=new Dictionary<string,NativeAssetContract>{{"tunnel",new("tunnel",new[]{envelope},Array.Empty<float>())}};
        (SparseProfileRows Rows,NetworkTunnelContacts.Report Report) Build(IReadOnlyList<PreparedSegment> sections,ushort[]? raster=null,
            Action<NetworkTunnelContacts.CoverEvidence>? evidence=null)
        {
            var rows=new SparseProfileRows(sections.Count*4);var a=Enumerable.Range(0,sections.Count).Select(i=>4*i).ToArray();
            var report=NetworkTunnelContacts.Append(sections,raster??terrain,scale,width,assets,a,a.Select(i=>i+1).ToArray(),a.Select(i=>i+2).ToArray(),a.Select(i=>i+3).ToArray(),rows,.05,default,evidence:evidence);
            rows.Freeze();return(rows,report);
        }
        var interior=Build(new[]{tunnel});
        Check(interior.Report.Rows>0&&interior.Report.Uncertified==0&&interior.Report.MissingAssets==0,"Closed tunnel lost its continuous roof rows");
        Check(interior.Rows.Evaluate(Enumerable.Repeat(81d,4).ToArray(),null,default)==0&&interior.Rows.Evaluate(Enumerable.Repeat(83d,4).ToArray(),null,default)>0,"Continuous cover ignored actual three-metre roof or interior15m policy");
        var weight=Enumerable.Range(0,(int)interior.Rows.Count).Sum(i=>interior.Rows.Row(i).Weight);
        Check(weight>=9999.999&&weight<=10100.001,"Tunnel represented-area family weights were applied more than once");
        var before=tunnel with{WayId=2,StartNodeId=3,EndNodeId=1,Start=new(-800,100,0),End=tunnel.Start,Tunnel=false};
        var after=tunnel with{WayId=3,StartNodeId=2,EndNodeId=4,Start=tunnel.End,End=new(800,100,0),Tunnel=false};
        var portal=Build(new[]{before,tunnel,after});
        Check(portal.Report.Uncertified==0,"Straight portal distances failed certification");
        var portalCeilings=new List<double>();
        for(long r=0;r<portal.Rows.Count;r++)
        {
            var row=portal.Rows.Row(r);
            if(row.Count==1&&portal.Rows.Index(row.Offset)==4)portalCeilings.Add(row.Target*.05);
        }
        Check(portalCeilings.Count>0&&portalCeilings.Min()>96.8&&portalCeilings.Max()<97.1,"Portal was treated as interior15m cover, or actual roof metadata was faded");
        var uneven=tunnel with{Control1=new(-590,80,0),Control2=new(-300,80,0)};
        var varying=Build(new[]{before,uneven,after});
        Check(varying.Report.Rows>0&&varying.Report.Uncertified==0,"Nonuniform-speed portal refinement did not converge to its distance bound");
        // A side-wall depression is invisible to centreline samples but must
        // lower the full-width roof allowance. Keep the centreline row unchanged.
        var depressed=(ushort[])terrain.Clone();depressed[9*17+8]=20000;
        var wide=tunnel with{NativeWidth=300};var side=Build(new[]{wide},depressed);
        Check(side.Rows.Evaluate(Enumerable.Repeat(81d,4).ToArray(),null,default)>0,"Off-centre terrain depression escaped full-width cover");
        var missing=Build(new[]{tunnel with{NativeAssetKey="absent"}});
        Check(missing.Report.MissingAssets==1&&missing.Report.Rows==0,"Missing native roof metadata was silently invented");
        void CompareStreaming(PreparedSegment[] sections,ushort[] raster,string context,bool independentFamilies=false)
        {
            var storedEvidence=new List<NetworkTunnelContacts.CoverEvidence>();var measuredEvidence=new List<NetworkTunnelContacts.CoverEvidence>();
            var stored=Build(sections,raster,storedEvidence.Add);var captured=new List<(int[] Indices,double[] Coefficients,double Target)>();
            var sink=SparseProfileRows.ForMeasurement(sections.Length*4,(indices,coefficients,count,target)=>
                captured.Add((indices.Take(count).ToArray(),coefficients.Take(count).ToArray(),target)));
            var starts=Enumerable.Range(0,sections.Length).Select(i=>4*i).ToArray();
            var streamed=NetworkTunnelContacts.Append(sections,raster,scale,width,assets,starts,starts.Select(i=>i+1).ToArray(),
                starts.Select(i=>i+2).ToArray(),starts.Select(i=>i+3).ToArray(),sink,.05,default,evidence:measuredEvidence.Add);
            Check(streamed==stored.Report&&captured.Count==stored.Rows.Count&&sink.CapacityBytes==0,
                context+": streaming roof certification changed row/coverage counts or retained a packed matrix");
            for(var r=0;r<captured.Count;r++)
            {
                var row=stored.Rows.Row(r);var actual=captured[r];
                Check(actual.Indices.Length==row.Count&&BitConverter.DoubleToInt64Bits(actual.Target)==BitConverter.DoubleToInt64Bits(row.Target),
                    context+": streaming roof certification changed a target");
                for(var j=0;j<row.Count;j++)Check(actual.Indices[j]==stored.Rows.Index(row.Offset+j)&&
                    BitConverter.DoubleToInt64Bits(actual.Coefficients[j])==BitConverter.DoubleToInt64Bits(stored.Rows.Coefficient(row.Offset+j)),
                    context+": streaming roof certification changed a canonical coefficient");
            }
            // Stored costs classify unavailable regions in pass zero, while the
            // measurement sink interleaves them with its only emission pass.
            // Their coverage and row ranges must match, including multiplicity.
            var expectedEvidence=storedEvidence.GroupBy(e=>e).ToDictionary(g=>g.Key,g=>g.Count());
            var actualEvidence=measuredEvidence.GroupBy(e=>e).ToDictionary(g=>g.Key,g=>g.Count());
            Check(expectedEvidence.Count==actualEvidence.Count&&expectedEvidence.All(p=>actualEvidence.TryGetValue(p.Key,out var count)&&count==p.Value),
                context+": streaming roof evidence changed interval coverage, classification or canonical row ranges");
            if(independentFamilies)
            {
                // Unit measurement weights are deliberately not optimizer cost
                // weights. Each disconnected stored family retains its own
                // represented-area normalization, independently of the sink.
                var weights=new Dictionary<int,double>();
                for(long r=0;r<stored.Rows.Count;r++)
                {
                    var row=stored.Rows.Row(r);Check(row.Count>0,context+": family fixture unexpectedly lost its variables");
                    var section=stored.Rows.Index(row.Offset)/4;
                    for(var j=1;j<row.Count;j++)Check(stored.Rows.Index(row.Offset+j)/4==section,context+": a roof row crossed independent families");
                    weights[section]=weights.TryGetValue(section,out var previous)?previous+row.Weight:row.Weight;
                }
                Check(weights.Count==sections.Length&&weights.Values.All(value=>value>=9999.999&&value<=10100.001),
                    context+": disconnected tunnel families shared or lost their optimizer normalization");
            }
        }
        foreach(var section in new[]{tunnel,wide,tunnel with{Start=new(-1100,80,0)},uneven,tunnel with{NativeAssetKey="absent"}})
            CompareStreaming(new[]{section},terrain,"single-section roof");
        CompareStreaming(new[]{before,uneven,after},depressed,"surface-connected nonuniform portal graph");
        var distant=tunnel with{WayId=10,StartNodeId=11,EndNodeId=12,NativeWidth=32,Start=new(-600,80,400),End=new(600,80,400)};
        CompareStreaming(new[]{wide,distant},depressed,"disconnected tunnel families",independentFamilies:true);
        var collapsed=tunnel with{End=tunnel.Start};
        CompareStreaming(new[]{collapsed},terrain,"uncertified stationary footprint");
        foreach(var section in new[]{tunnel with{Start=new(-600,90,0),End=new(600,90,0)},
            wide with{Start=new(-1100,90,0),End=new(600,90,0)},tunnel with{NativeAssetKey="absent"}})
        {
            var observations=new List<NetworkTunnelProfile.PhysicalObservation>();
            var baseline=NetworkTunnelProfile.MeasurePhysical(new[]{section},terrain,scale,width,assets,default);
            var measured=NetworkTunnelProfile.MeasurePhysical(new[]{section},terrain,scale,width,assets,default,observe:observations.Add);
            Check(baseline==measured,"Physical inventory changed certification counts or residuals");
            Check(observations.Where(o=>o.Category=="continuous-tunnel-cover"&&!o.Unavailable).Sum(o=>o.FailedRows)==measured.CoverFailures,
                "Physical inventory omitted failed cover rows");
            Check(observations.All(o=>o.FirstSection==0&&o.FirstBegin>=0&&o.FirstEnd<=1&&o.FirstBegin<=o.FirstEnd),
                "Physical inventory lost final-section parameter correspondence");
            if(section.NativeAssetKey=="absent")Check(observations.Any(o=>o.Unavailable&&o.Classification=="missing-roof-contract"),
                "Missing roof contract disappeared from physical inventory");
        }
        var allModes=Enum.GetValues<NativeStructureMode>().Select(mode=>envelope with{Mode=mode}).ToArray();
        var physicalAssets=new Dictionary<string,NativeAssetContract>{{"all",new("all",allModes,Array.Empty<float>())}};
        var lower=tunnel with{NativeAssetKey="all",Tunnel=false,Layer=0,Start=new(-600,100,0),End=new(600,100,0)};
        var upper=lower with{WayId=2,StartNodeId=3,EndNodeId=4,Bridge=true,Layer=1,Start=new(-600,102,0),End=new(600,102,0)};
        var contactObservations=new List<NetworkTunnelProfile.PhysicalObservation>();
        var physical=NetworkTunnelProfile.MeasurePhysical(new[]{lower,upper},terrain,scale,width,physicalAssets,default,observe:contactObservations.Add);
        Check(physical.ClearanceFailures>0&&contactObservations.Where(o=>o.Category=="full-width-clearance"&&!o.Unavailable).Sum(o=>o.FailedRows)==physical.ClearanceFailures,
            "Physical inventory omitted failed full-width clearance rows");
        var rail=lower with{Kind=NetworkKind.Rail,NativeCenterlineCurvatureLimit=.001,
            Start=new(-50,100,0),Control1=new(-50,100,50),Control2=new(50,100,-50),End=new(50,100,0)};
        var continuation=rail with{WayId=3,StartNodeId=rail.EndNodeId,EndNodeId=5,
            Start=rail.End,Control1=null,Control2=null,End=new(150,100,0)};
        var independent=continuation with{WayId=4,StartNodeId=6,EndNodeId=7,Start=new(-50,100,100),End=new(150,100,100),NativeCenterlineCurvatureLimit=null};
        var railSections=new[]{rail,continuation,independent};
        var inventory=Newtonsoft.Json.Linq.JObject.FromObject(PhysicalIssueInventory.Capture(railSections,railSections,terrain,scale,width,physicalAssets));
        Check(inventory["rail"]!["failedCertificateSections"]!.Count()>0&&inventory["rail"]!["members"]!.Count()==2&&
            inventory["rail"]!["missingCurvatureContractSections"]!.Count()==1,
            "Rail witness lost its connected formation, invented proximity topology, or omitted unavailable contracts");
        var unavailableRails=new[]{rail with{NativeCenterlineCurvatureLimit=0},continuation with{NativeCenterlineCurvatureLimit=double.NaN},
            independent with{NativeCenterlineCurvatureLimit=-1},independent with{WayId=5,StartNodeId=8,EndNodeId=9,NativeCenterlineCurvatureLimit=null}};
        var unavailableInventory=Newtonsoft.Json.Linq.JObject.FromObject(
            PhysicalIssueInventory.Capture(unavailableRails,unavailableRails,terrain,scale,width,physicalAssets));
        var unavailableRail=unavailableInventory["rail"]!;
        Check(unavailableRail["failedCertificateSections"]!.Count()==0&&unavailableRail["missingCurvatureContractSections"]!.Count()==4&&
            (string?)unavailableRail["selectedWitnessReason"]=="unavailable-curvature-contract"&&unavailableRail["members"]!.Count()==2&&
            unavailableRail["immutableSelectedSource"]!.Count()==2,
            "All-unavailable rail contracts lost their complete source-connected witness or gained a proximity connection");
        var reasons=unavailableRail["unavailableCurvatureContracts"]!.Select(entry=>(string?)entry["reason"]).ToArray();
        Check(reasons.SequenceEqual(new[]{"nonpositive","nonfinite","nonpositive","missing"}),
            "Rail inventory confused missing, nonfinite and nonpositive curvature contracts");
        Check((string?)inventory["rail"]!["selectedWitnessReason"]=="failed-curvature-certificate",
            "Unavailable rail contracts superseded a failing-certificate witness");
        var noIssues=Newtonsoft.Json.Linq.JObject.FromObject(PhysicalIssueInventory.Capture(
            Array.Empty<PreparedSegment>(),Array.Empty<PreparedSegment>(),terrain,scale,width,physicalAssets));
        Check((string?)noIssues["rail"]!["selectedWitnessReason"]=="no-failed-or-unavailable-rail"&&noIssues["rail"]!["members"]!.Count()==0,
            "Empty rail coverage claimed a selected witness");
        foreach(var sections in new[]{new[]{tunnel with{NativeAssetKey="absent"}},new[]{collapsed},
            new[]{wide with{Start=new(-1100,80,0)}},new[]{collapsed,tunnel with{WayId=8,StartNodeId=8,EndNodeId=9,NativeAssetKey="absent"}}})
        {
            var coverInventory=Newtonsoft.Json.Linq.JObject.FromObject(PhysicalIssueInventory.Capture(sections,sections,terrain,scale,width,assets));
            Check((int)coverInventory["quality"]!["UncertifiedCover"]!>0&&
                (long)coverInventory["coverageCounts"]!["unavailableCoverEvents"]! == (int)coverInventory["quality"]!["UncertifiedCover"]!&&
                (bool)coverInventory["unavailableCoverEventsMapped"]!,
                "Unavailable cover events escaped the inventory ledger");
        }
        Console.WriteLine("PASS: continuous full-width tunnel roof, exact terrain cells, interior/portal policy, nonuniform-speed distance bounds, area normalization and missing-contract review.");
    }
}
