using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Newtonsoft.Json;
using Unity.Mathematics;

static partial class VisualMetricsReplay
{
    public static void RailCrossings(string folder,string output)
    {
        var c=Load(folder);var log=new ImportLog();
        var expected=NativeEndpointPolicy.LevelCrossings(c.Source);
        var compiled=VisualNetworkCompiler.Prepare(c.Source,c.Terrain,c.Scale,c.Width,log,default,c.Water);
        var sections=NativeEndpointPolicy.PrepareLevelCrossings(compiled,log);
        var actual=NativeEndpointPolicy.LevelCrossings(sections);
        var rows=sections.SelectMany(s=>new[]{(Section:s,Node:s.StartNodeId,Point:s.CourseStart),(Section:s,Node:s.EndNodeId,Point:s.CourseEnd)})
            .Where(p=>expected.Contains(p.Node)).GroupBy(p=>p.Node).Select(g=>
            {
                var ports=g.ToArray();var road=ports.Where(p=>p.Section.Kind==NetworkKind.Road).ToArray();var rail=ports.Where(p=>p.Section.Kind==NetworkKind.Rail).ToArray();
                var horizontal=road.Length==0||rail.Length==0?double.PositiveInfinity:road.Max(a=>rail.Max(b=>(double)math.distance(a.Point.xz,b.Point.xz)));
                var vertical=road.Length==0||rail.Length==0?double.PositiveInfinity:road.Max(a=>rail.Max(b=>(double)Math.Abs(a.Point.y-b.Point.y)));
                var grades=ports.Select(p=>
                {
                    var s=p.Section;var d=p.Node==s.StartNodeId?s.CourseControl1-s.CourseStart:s.CourseEnd-s.CourseControl2;
                    return Math.Abs(d.y)/Math.Max(.001,math.length(d.xz));
                }).ToArray();
                return new{node=g.Key,roadPorts=road.Length,railPorts=rail.Length,horizontalGap=horizontal,verticalGap=vertical,
                    maxEntryGrade=grades.Max(),sharedNativeOwner=ports.Select(p=>NativeEndpointPolicy.Identity(p.Section,p.Node)).Distinct().Count()==1,
                    structures=ports.Count(p=>NetworkStructure.IsStructure(p.Section))};
            }).ToArray();
        File.WriteAllText(output,JsonConvert.SerializeObject(new{capture=Path.GetFileName(folder),sourceCrossings=expected.Count,finalCrossings=actual.Count,
            missingNodes=expected.Where(n=>!rows.Any(r=>r.node==n)).ToArray(),rows,
            metrics=log.Lines.Where(l=>l.Contains("crossing",StringComparison.OrdinalIgnoreCase)||l.Contains("junction height mismatch",StringComparison.Ordinal)).ToArray()},Formatting.Indented));
        Console.WriteLine($"Rail crossings: source={expected.Count}; final={actual.Count}; exactSharedNativeOwners={rows.Count(r=>r.sharedNativeOwner)}; report={output}");
    }
}
