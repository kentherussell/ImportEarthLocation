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
        (SparseProfileRows Rows,NetworkTunnelContacts.Report Report) Build(IReadOnlyList<PreparedSegment> sections,ushort[]? raster=null)
        {
            var rows=new SparseProfileRows(sections.Count*4);var a=Enumerable.Range(0,sections.Count).Select(i=>4*i).ToArray();
            var report=NetworkTunnelContacts.Append(sections,raster??terrain,scale,width,assets,a,a.Select(i=>i+1).ToArray(),a.Select(i=>i+2).ToArray(),a.Select(i=>i+3).ToArray(),rows,.05,default);
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
        Console.WriteLine("PASS: continuous full-width tunnel roof, exact terrain cells, interior/portal policy, nonuniform-speed distance bounds, area normalization and missing-contract review.");
    }
}
