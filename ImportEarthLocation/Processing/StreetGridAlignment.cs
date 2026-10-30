using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Local plan alignment in map metres. Only the compiler's ordinary-road
// strokes enter this pass; it retains node identities and never changes lanes.
public static class StreetGridAlignment
{
    public sealed record Street(int Id,SourceNodeKey[] Nodes,int[] At,double2[] Points,double Width);
    public readonly record struct Line(double2 Direction,double Offset)
    {
        public double2 Normal=>new(-Direction.y,Direction.x);
        public double2 Project(double2 p)=>p+Normal*(Offset-math.dot(p,Normal));
    }
    public sealed record Result(IReadOnlyDictionary<int,Line> Lines,IReadOnlyDictionary<SourceNodeKey,double2> Nodes);
    private sealed record Fit(Street Street,Line Line,double Length,double Angle,double Residual);
    private const double QuarterTurn=Math.PI/2;
    private const double Cell=400;
    private const double Neighbourhood=450;
    private const double AngleTolerance=2.5*Math.PI/180;

    public static Result Prepare(IReadOnlyList<Street> streets,HashSet<SourceNodeKey> fixedNodes,ImportLog log,CancellationToken token)
    {
        var original=new Dictionary<SourceNodeKey,double2>();var budget=new Dictionary<SourceNodeKey,double>();
        foreach(var street in streets)for(var k=0;k<street.Nodes.Length;k++)
        {
            var node=street.Nodes[k];original[node]=street.Points[street.At[k]];
            var limit=Math.Min(12,Math.Max(4,street.Width/2+2));
            budget[node]=budget.TryGetValue(node,out var old)?Math.Min(old,limit):limit;
        }
        var fits=new Dictionary<int,Fit>();
        foreach(var street in streets)
        {
            token.ThrowIfCancellationRequested();var p=street.Points;if(p.Length<2)continue;
            var chord=p[^1]-p[0];var span=math.length(chord);if(span<80)continue;
            var length=0d;var direction=math.normalizesafe(chord);var sum=default(double2);
            for(var k=1;k<p.Length;k++)
            {
                var d=p[k]-p[k-1];var l=math.length(d);length+=l;
                if(l>1e-6&&math.dot(d/l,direction)>Math.Cos(10*Math.PI/180))sum+=d;
            }
            if(length<=0||span/length<.96||math.lengthsq(sum)<1)continue;
            direction=math.normalizesafe(sum);var normal=new double2(-direction.y,direction.x);
            var offsets=p.Select(q=>math.dot(q,normal)).OrderBy(v=>v).ToArray();var offset=offsets[offsets.Length/2];
            var residual=p.Select(q=>Math.Abs(math.dot(q,normal)-offset)).OrderBy(v=>v).ToArray();
            if(residual[(int)(.9*(residual.Length-1))]>3||residual[^1]>Math.Min(12,street.Width/2+2))continue;
            fits[street.Id]=new(street,new(direction,offset),length,Math.Atan2(direction.y,direction.x),residual[^1]);
        }
        var cells=new Dictionary<(int,int),HashSet<int>>();
        (int,int) Key(double2 p)=>((int)Math.Floor(p.x/Cell),(int)Math.Floor(p.y/Cell));
        foreach(var fit in fits.Values)foreach(var p in fit.Street.Points)
        {var key=Key(p);if(!cells.TryGetValue(key,out var ids))cells[key]=ids=new();ids.Add(fit.Street.Id);}
        double Difference(double a,double b){var d=a-b;return d-Math.Round(d/QuarterTurn)*QuarterTurn;}
        IEnumerable<int> Near(Fit fit)
        {
            var p=(fit.Street.Points[0]+fit.Street.Points[^1])/2;var (cx,cz)=Key(p);var seen=new HashSet<int>();
            for(var dz=-1;dz<=1;dz++)for(var dx=-1;dx<=1;dx++)if(cells.TryGetValue((cx+dx,cz+dz),out var ids))foreach(var id in ids)
            {
                if(!seen.Add(id))continue;var q=fits[id].Street.Points;var ab=q[^1]-q[0];
                var t=math.clamp(math.dot(p-q[0],ab)/Math.Max(1e-9,math.lengthsq(ab)),0,1);
                if(math.distance(p,q[0]+ab*t)<=Neighbourhood)yield return id;
            }
        }
        var neighbours=fits.Values.ToDictionary(f=>f.Street.Id,f=>Near(f).OrderBy(id=>id).ToArray());
        var axes=new Dictionary<int,(double Angle,double Support)>();
        foreach(var fit in fits.Values)
        {
            var nearby=neighbours[fit.Street.Id].Select(id=>fits[id]).Where(f=>Math.Abs(Difference(f.Angle,fit.Angle))<=AngleTolerance).ToArray();
            bool Parallel(Fit f)=>Math.Abs(math.dot(f.Line.Direction,fit.Line.Direction))>.707;
            int Lines(bool parallel)
            {
                var normal=parallel?fit.Line.Normal:fit.Line.Direction;
                var offsets=nearby.Where(f=>Parallel(f)==parallel).Select(f=>math.dot((f.Street.Points[0]+f.Street.Points[^1])/2,normal)).OrderBy(v=>v).ToArray();
                var count=0;var last=double.NegativeInfinity;
                foreach(var offset in offsets)if(offset-last>=Math.Max(20,fit.Street.Width+2)){last=offset;count++;}
                return count;
            }
            // Two distinct streets on each axis distinguish a block grid from
            // several collinear pieces of one road or an isolated crossroads.
            if(Lines(true)<2||Lines(false)<2)continue;
            var x=0d;var y=0d;var support=0d;
            foreach(var f in nearby){var w=Math.Min(300,f.Length);x+=w*Math.Cos(4*f.Angle);y+=w*Math.Sin(4*f.Angle);support+=w;}
            axes[fit.Street.Id]=(Math.Atan2(y,x)/4,support);
        }
        var snapped=new Dictionary<int,double>();
        foreach(var seed in axes.OrderByDescending(p=>p.Value.Support).ThenBy(p=>p.Key))
        {
            if(snapped.ContainsKey(seed.Key))continue;var queue=new List<int>{seed.Key};snapped[seed.Key]=seed.Value.Angle;
            for(var head=0;head<queue.Count;head++)foreach(var id in neighbours[queue[head]])
                if(!snapped.ContainsKey(id)&&axes.ContainsKey(id)&&Math.Abs(Difference(fits[id].Angle,seed.Value.Angle))<=AngleTolerance)
                {snapped[id]=seed.Value.Angle;queue.Add(id);}
        }
        var lines=new Dictionary<int,Line>();
        foreach(var fit in fits.Values)
        {
            var line=fit.Line;
            if(snapped.TryGetValue(fit.Street.Id,out var axis))
            {
                var angle=axis+Math.Round((fit.Angle-axis)/QuarterTurn)*QuarterTurn;var direction=new double2(Math.Cos(angle),Math.Sin(angle));
                var n=new double2(-direction.y,direction.x);var values=fit.Street.Points.Select(p=>math.dot(p,n)).OrderBy(v=>v).ToArray();
                line=new(direction,values[values.Length/2]);
            }
            else if(fit.Residual>1.5)continue;
            var pinned=fit.Street.Nodes.Where(fixedNodes.Contains).Select(n=>math.dot(original[n],line.Normal)).ToArray();
            if(pinned.Length>0)line=line with{Offset=pinned[0]};
            if(pinned.Any(v=>Math.Abs(v-line.Offset)>.01)||fit.Street.Points.Any(p=>math.distance(p,line.Project(p))>Math.Min(12,fit.Street.Width/2+2)))continue;
            lines[fit.Street.Id]=line;
        }
        var atNode=new Dictionary<SourceNodeKey,List<int>>();
        foreach(var fit in fits.Values)foreach(var node in fit.Street.Nodes)
        {if(!atNode.TryGetValue(node,out var list))atNode[node]=list=new();list.Add(fit.Street.Id);}
        var proposed=new Dictionary<SourceNodeKey,double2>();
        while(lines.Count>0)
        {
            token.ThrowIfCancellationRequested();proposed.Clear();
            foreach(var entry in atNode)
            {
                var p=original[entry.Key];if(fixedNodes.Contains(entry.Key)){proposed[entry.Key]=p;continue;}
                var ids=entry.Value.Where(lines.ContainsKey).Distinct().ToArray();if(ids.Length==0)continue;
                var target=lines[ids[0]].Project(p);var cross=0d;
                for(var a=0;a<ids.Length;a++)for(var b=a+1;b<ids.Length;b++)
                {
                    var x=lines[ids[a]];var y=lines[ids[b]];var d=NetworkDesignChecks.Cross(x.Normal,y.Normal);if(Math.Abs(d)<=Math.Max(.5,cross))continue;
                    cross=Math.Abs(d);target=new((x.Offset*y.Normal.y-x.Normal.y*y.Offset)/d,(x.Normal.x*y.Offset-x.Offset*y.Normal.x)/d);
                }
                if(math.distance(target,p)>budget[entry.Key]||ids.Any(id=>Math.Abs(math.dot(target,lines[id].Normal)-lines[id].Offset)>.01))target=p;
                proposed[entry.Key]=target;
            }
            var rejected=lines.Keys.Where(id=>fits[id].Street.Nodes.Any(n=>!proposed.TryGetValue(n,out var p)||Math.Abs(math.dot(p,lines[id].Normal)-lines[id].Offset)>.01)).ToArray();
            if(rejected.Length==0)break;foreach(var id in rejected)lines.Remove(id);
        }
        if(lines.Count==0)proposed.Clear();
        var moved=proposed.Where(p=>math.distancesq(p.Value,original[p.Key])>1e-12).ToDictionary(p=>p.Key,p=>p.Value);
        log.Diagnostic($"VISUAL COMPILER street alignment eligibleStrokes={streets.Count}; straightCandidates={fits.Count}; gridCandidates={snapped.Count}; alignedStrokes={lines.Count}; movedNodes={moved.Count}; localAxes=true; highwayAndStructureNodesFixed=true");
        return new(lines,moved);
    }
}
