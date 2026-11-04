using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Newtonsoft.Json;
using Unity.Mathematics;

static partial class VisualMetricsReplay
{
    public static void StreetSmoothing(string folder,string output,string? baselineAssembly=null)
    {
        var c=Load(folder);var log=new ImportLog();
        var compiled=VisualNetworkCompiler.Prepare(c.Source,c.Terrain,c.Scale,c.Width,log,default,c.Water);
        var sections=NativeDesignUnits.Prepare(StarterNetworkPolicy.WithoutSideStructures(compiled),c.Contracts,log,default,s=>VisualCompilerTable.For(s).MinimumSegment);
        sections=NativeEndpointPolicy.PrepareLevelCrossings(sections,log);
        bool Street(PreparedSegment s)=>s.Kind==NetworkKind.Road&&!NativeAssetKeys.IsHighway(s)&&!s.ProtectedLink&&!s.Bridge&&!s.Tunnel&&!s.Roundabout;
        float[] Point(float3 p)=>new[]{p.x,p.y,p.z};
        var ports=sections.SelectMany((s,i)=>new[]{(Index:i,Section:s,Node:s.StartNodeId,Start:true),(Index:i,Section:s,Node:s.EndNodeId,Start:false)})
            .GroupBy(p=>new SourceNodeKey(p.Section.Kind,p.Node)).ToDictionary(g=>g.Key,g=>g.ToArray());
        var rows=sections.Select((s,i)=>(s,i)).Where(p=>Street(p.s)).Select(p=>
        {
            var s=p.s;var length=NetworkDesignChecks.HorizontalArcLength(new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd));
            return new{index=p.i,s.WayId,s.StartNodeId,s.EndNodeId,s.Name,s.Classification,length,minimum=VisualCompilerTable.For(s).MinimumSegment,
                startDegree=ports[new(s.Kind,s.StartNodeId)].Length,endDegree=ports[new(s.Kind,s.EndNodeId)].Length,
                start=new[]{s.CourseStart.x,s.CourseStart.y,s.CourseStart.z},end=new[]{s.CourseEnd.x,s.CourseEnd.y,s.CourseEnd.z}};
        }).ToArray();
        var kinks=ports.Where(p=>p.Value.Length==2&&p.Value.All(q=>Street(q.Section))).Select(p=>
        {
            var arms=p.Value.Select(q=>
            {
                var s=q.Section;var d=q.Start?s.CourseControl1-s.CourseStart:s.CourseControl2-s.CourseEnd;
                return (Direction:math.normalizesafe(d.xz),Grade:d.y/Math.Max(.001,math.length(d.xz)));
            }).ToArray();
            return new{node=p.Key.Id,ways=p.Value.Select(q=>q.Section.WayId).ToArray(),
                angle=math.degrees(Math.Acos(math.clamp(-math.dot(arms[0].Direction,arms[1].Direction),-1,1))),
                gradeJump=Math.Abs(arms[0].Grade+arms[1].Grade),position=Point(p.Value[0].Start?p.Value[0].Section.CourseStart:p.Value[0].Section.CourseEnd)};
        }).ToArray();
        var topology=new ImportLog{CaptureRecords=true};
        SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(c.Source),sections,topology,default);
        string GeometryHash(IEnumerable<PreparedSegment> values)
        {
            var lines=values.Select(s=>JsonConvert.SerializeObject(new{s.Kind,s.WayId,s.Classification,s.Lanes,s.OneWay,s.Bridge,s.Tunnel,s.Layer,
                a=Point(s.CourseStart),b=Point(s.CourseControl1),c=Point(s.CourseControl2),d=Point(s.CourseEnd)})).OrderBy(s=>s,StringComparer.Ordinal);
            return Convert.ToHexString(System.Security.Cryptography.SHA256.HashData(System.Text.Encoding.UTF8.GetBytes(string.Join("\n",lines))));
        }
        string? baselineHighways=null;string? baselineRail=null;
        if(baselineAssembly!=null)
        {
            // Load the frozen pre-edit harness in its own context. Its private
            // capture loader and compiler supply the actual baseline geometry;
            // no candidate code reconstructs or approximates that baseline.
            var assembly=System.Reflection.Assembly.LoadFile(Path.GetFullPath(baselineAssembly));
            var capture=assembly.GetType("VisualMetricsReplay")!.GetMethod("Load",System.Reflection.BindingFlags.NonPublic|System.Reflection.BindingFlags.Static)!.Invoke(null,new object[]{folder})!;
            object Get(string name)=>capture.GetType().GetProperty(name)!.GetValue(capture)!;
            var baselineLog=Activator.CreateInstance(assembly.GetType("ImportEarthLocation.Core.ImportLog")!)!;
            var result=(System.Collections.IEnumerable)assembly.GetType("ImportEarthLocation.Processing.VisualNetworkCompiler")!.GetMethod("Prepare")!
                .Invoke(null,new[]{Get("Source"),Get("Terrain"),Get("Scale"),Get("Width"),baselineLog,(object)default(System.Threading.CancellationToken),Get("Water"),(object)float.NaN})!;
            var converted=result.Cast<object>().Select(s=>
            {
                object Value(string name)=>s.GetType().GetProperty(name)!.GetValue(s)!;
                float3 Position(string name){var p=Value(name);float Field(string key)=>Convert.ToSingle(p.GetType().GetField(key)!.GetValue(p));return new(Field("x"),Field("y"),Field("z"));}
                return new PreparedSegment(Convert.ToInt64(Value("WayId")),0,0,Position("CourseStart"),Position("CourseEnd"),
                    Enum.Parse<NetworkKind>(Value("Kind").ToString()!),(string)Value("Classification"),Convert.ToInt32(Value("Lanes")),
                    (bool)Value("OneWay"),(bool)Value("Bridge"),(bool)Value("Tunnel"),Convert.ToInt32(Value("Layer")))
                    {Control1=Position("CourseControl1"),Control2=Position("CourseControl2")};
            }).ToArray();
            baselineHighways=GeometryHash(converted.Where(s=>s.Kind==NetworkKind.Road&&NativeAssetKeys.IsHighway(s)));
            baselineRail=GeometryHash(converted.Where(s=>s.Kind==NetworkKind.Rail));
        }
        var report=new{capture=Path.GetFileName(folder),streetSections=rows.Length,shortSections=rows.Count(r=>r.length<r.minimum),
            artificialShortSections=rows.Count(r=>r.length<r.minimum&&(r.startDegree==2||r.endDegree==2)),below30Metres=rows.Count(r=>r.length<30),
            plainNodeKinksOver1Degree=kinks.Count(k=>k.angle>1),plainNodeKinksOver5Degrees=kinks.Count(k=>k.angle>5),
            plainNodeGradeJumpsOver1Percent=kinks.Count(k=>k.gradeJump>.01),
            highwayGeometryHash=GeometryHash(sections.Where(s=>s.Kind==NetworkKind.Road&&NativeAssetKeys.IsHighway(s))),
            railGeometryHash=GeometryHash(sections.Where(s=>s.Kind==NetworkKind.Rail)),
            compiledHighwayGeometryHash=GeometryHash(compiled.Where(s=>s.Kind==NetworkKind.Road&&NativeAssetKeys.IsHighway(s))),
            compiledRailGeometryHash=GeometryHash(compiled.Where(s=>s.Kind==NetworkKind.Rail)),baselineHighwayGeometryHash=baselineHighways,baselineRailGeometryHash=baselineRail,
            missingPorts=topology.Metric("Missing mandatory selected-source ports"),unresolvedRoutes=topology.Metric("Unresolved selected-source reachability obligations"),
            rows,kinks=kinks.OrderByDescending(k=>k.angle).ToArray(),diagnostics=log.Lines.Where(l=>l.StartsWith("VISUAL COMPILER horizontal",StringComparison.Ordinal)||l.StartsWith("NATIVE UNITS",StringComparison.Ordinal)).ToArray()};
        File.WriteAllText(output,JsonConvert.SerializeObject(report,Formatting.Indented));
        Console.WriteLine($"Street smoothing {report.capture}: sections={rows.Length}; short={report.shortSections}; artificialShort={report.artificialShortSections}; kinks>1deg={report.plainNodeKinksOver1Degree}; report={output}");
    }
}
