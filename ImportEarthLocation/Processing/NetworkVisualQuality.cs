using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Complete sampled screening, not a mesh certificate or a placement veto.
// Only the diagnostic drawings/witness examples are bounded to the worst cases.
public static class NetworkVisualQuality
{
    public static void Audit(IReadOnlyList<PreparedSegment> sections,NetworkCurveObservations.Observation[] observations,
        ImportLog log,CancellationToken token)
    {
        var rates=0;var oscillations=0;var shortSpans=0;var backward=0;var candidates=0;var maximum=0d;
        var worst=new List<(int Index,double Score)>();
        for(var i=0;i<sections.Count;i++)
        {
            token.ThrowIfCancellationRequested();var s=sections[i];var o=observations[i];
            var window=s.Kind==NetworkKind.Rail||!s.ProtectedLink&&(s.Classification is "motorway" or "trunk")?500d:100d;
            var threshold=4*NetworkVerticalProfiles.DesiredGrade(s)/window;
            var rate=o.SampledMaximumGradeChangePerMeter>threshold;
            var oscillating=o.SampledGradeSignChanges>=2&&o.SampledSignedGradeVariation>2*NetworkVerticalProfiles.DesiredGrade(s)&&o.SampledLengthMeters<2*window;
            var shortSpan=o.SampledLengthMeters<Math.Max(8,s.NativeWidth*.75);
            var nonregular=o.MinimumControlProgressMeters<=0&&!s.Roundabout;
            if(rate)rates++;if(oscillating)oscillations++;if(shortSpan)shortSpans++;if(nonregular)backward++;
            if(rate||oscillating||shortSpan||nonregular)candidates++;
            maximum=Math.Max(maximum,o.SampledMaximumGradeChangePerMeter);
            if(log.DiagnosticsEnabled&&(rate||oscillating||shortSpan||nonregular))
            {
                var score=(rate?Math.Min(100,o.SampledMaximumGradeChangePerMeter/threshold):0)+(oscillating?10:0)+(shortSpan?2:0)+(nonregular?20:0);
                worst.Add((i,score));if(worst.Count>48)worst=worst.OrderByDescending(p=>p.Score).ThenBy(p=>sections[p.Index].WayId).Take(24).ToList();
            }
        }
        void Put(string name,int failed,double peak,string meaning)=>log.Quality(new("planned",name,"visual-screen",sections.Count,failed,0,peak,0,meaning+"; sampled screening; RequiresReview only; native mesh acceptance unknown"));
        Put("visual-grade-rate",rates,maximum,"Grade-change/meter above 4*desiredGrade/window; windows 500m mainline/rail, 100m others");
        Put("visual-oscillation",oscillations,oscillations,"At least two grade reversals and total variation above 2*desiredGrade within two windows");
        Put("short-native-span",shortSpans,shortSpans,"Length below max(8m,0.75*nativeWidth); may be a necessary junction/portal, not automatically removable");
        Put("backward-control-progress",backward,backward,"Nonpositive control-polygon progress outside roundabouts; conservative screening, not proof of a cusp");
        if(!log.DiagnosticsEnabled)return;
        var selected=worst.OrderByDescending(p=>p.Score).ThenBy(p=>sections[p.Index].WayId).Take(24).ToArray();
        var distribution=observations.Where(o=>o.Finite&&SparseProfileRows.Finite(o.SampledMaximumGradeChangePerMeter))
            .Select(o=>o.SampledMaximumGradeChangePerMeter).OrderBy(v=>v).ToArray();
        double? Percentile(double p)=>distribution.Length==0?null:distribution[(int)Math.Ceiling(p*(distribution.Length-1))];
        log.DiagnosticJson("visual-quality-summary.json",new{sections=sections.Count,rates,oscillations,shortSpans,backward,
            examples=selected.Length,examplesComplete=selected.Length==candidates,
            gradeChangePerMeter=new{finiteObservations=distribution.Length,p50=Percentile(.5),p95=Percentile(.95),p99=Percentile(.99)},
            nativeOutcome="Unknown",screeningOnly=true});
        log.DiagnosticRecords("visual-quality-worst.ndjson",selected.Select(p=>(object)new{section=p.Index,score=p.Score,
            sections[p.Index].WayId,sections[p.Index].StartNodeId,sections[p.Index].EndNodeId,observation=observations[p.Index]}));
        var neighbours=new Dictionary<SourceNodeKey,List<int>>();
        for(var i=0;i<sections.Count;i++)foreach(var id in new[]{sections[i].StartNodeId,sections[i].EndNodeId})
        {var key=new SourceNodeKey(sections[i].Kind,id);if(!neighbours.TryGetValue(key,out var list))neighbours[key]=list=new();list.Add(i);}
        var svg=new StringBuilder("<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"900\" height=\"")
            .Append(Math.Max(1,selected.Length)*360).Append("\" viewBox=\"0 0 900 ").Append(Math.Max(1,selected.Length)*360).Append("\"><rect width=\"100%\" height=\"100%\" fill=\"#20292e\"/>");
        string N(double value)=>value.ToString("0.##",CultureInfo.InvariantCulture);
        for(var row=0;row<selected.Length;row++)
        {
            token.ThrowIfCancellationRequested();var chosen=selected[row].Index;var target=sections[chosen];var ids=new HashSet<int>{chosen};var frontier=new[]{chosen};
            for(var depth=0;depth<3;depth++)
            {
                var next=new List<int>();foreach(var i in frontier)foreach(var node in new[]{sections[i].StartNodeId,sections[i].EndNodeId})
                    foreach(var j in neighbours[new(target.Kind,node)])if(ids.Count<16&&ids.Add(j))next.Add(j);
                frontier=next.ToArray();
            }
            var curves=ids.OrderBy(i=>i).Select(i=>(Index:i,Points:Enumerable.Range(0,33).Select(k=>NetworkDesignChecks.Evaluate(new(sections[i].CourseStart,sections[i].CourseControl1,sections[i].CourseControl2,sections[i].CourseEnd),k/32d)).ToArray())).ToArray();
            if(curves.Any(c=>c.Points.Any(p=>!math.all(math.isfinite(p)))))continue;
            var all=curves.SelectMany(c=>c.Points).ToArray();var low=new double2(all.Min(p=>p.x),all.Min(p=>p.z));var high=new double2(all.Max(p=>p.x),all.Max(p=>p.z));
            var halfWidth=ids.Max(i=>sections[i].NativeWidth)/2;low-=halfWidth;high+=halfWidth;
            var range=high-low;var scale=Math.Min(820/Math.Max(1,range.x),160/Math.Max(1,range.y));var axis=math.normalizesafe((double2)(target.End-target.Start).xz,new double2(1,0));
            var station=all.Select(p=>math.dot(p.xz,axis)).ToArray();var xmin=station.Min();var xmax=station.Max();var ymin=all.Min(p=>p.y);var ymax=all.Max(p=>p.y);
            svg.Append("<text x=\"20\" y=\"").Append(row*360+22).Append("\" fill=\"#d7e1e7\" font-size=\"13\">Way ").Append(target.WayId).Append(" — plan and chord-projected elevation; sampled, not rendered</text>");
            foreach(var c in curves)
            {
                var color=c.Index==chosen?"#ddb484":"#8baebb";
                var stroke=Math.Max(1,sections[c.Index].NativeWidth*scale);
                svg.Append("<polyline fill=\"none\" stroke=\"").Append(color).Append("\" stroke-width=\"").Append(N(stroke)).Append("\" points=\"");
                foreach(var p in c.Points)svg.Append(N(40+(p.x-low.x)*scale)).Append(',').Append(N(row*360+195-(p.z-low.y)*scale)).Append(' ');
                svg.Append("\"/><polyline fill=\"none\" stroke=\"").Append(color).Append("\" stroke-width=\"2\" points=\"");
                foreach(var p in c.Points)svg.Append(N(40+820*(math.dot(p.xz,axis)-xmin)/Math.Max(1,xmax-xmin))).Append(',').Append(N(row*360+330-90*(p.y-ymin)/Math.Max(1,ymax-ymin))).Append(' ');
                svg.Append("\"/>");
            }
            svg.Append("<text x=\"40\" y=\"").Append(row*360+350).Append("\" fill=\"#aebdc6\" font-size=\"11\">Elevation range ").Append(N(ymin)).Append("–").Append(N(ymax)).Append(" m; vertical scale exaggerated; amber = selected defect</text>");
        }
        svg.Append("</svg>");log.DiagnosticSvg("visual-quality-worst.svg",svg.ToString());
    }
}
