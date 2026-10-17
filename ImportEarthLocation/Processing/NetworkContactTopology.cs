using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Immutable topology witnesses do not use heights produced by the optimizer.
// A short connected path diagnoses a clearance/grade contradiction; it never
// exempts a genuine remote crossing merely because its roads connect elsewhere.
public sealed class NetworkContactTopology
{
    private readonly IReadOnlyList<PreparedSegment> _source;
    private readonly Cubic3d[] _curves;
    private readonly int[] _start,_end;
    private readonly double[] _lengths;
    private readonly NetworkProfileGraph _graph;
    private readonly double _maximumGap;
    private readonly Dictionary<(int,int),double> _paths=new();
    private readonly Dictionary<int,List<(int Transition,double Begin,double End)>> _local=new();

    public NetworkContactTopology(IReadOnlyList<PreparedSegment> source,NativeCompositionEnvelope[] envelopes,CancellationToken token,double epsilon=.05)
    {
        _source=source;_curves=new Cubic3d[source.Count];_start=new int[source.Count];_end=new int[source.Count];_lengths=new double[source.Count];
        var nodes=new Dictionary<SourceNodeKey,int>();var incidence=new List<List<int>>();
        int Node(SourceNodeKey key){if(!nodes.TryGetValue(key,out var id)){nodes[key]=id=nodes.Count;incidence.Add(new());}return id;}
        for(var i=0;i<source.Count;i++)
        {
            token.ThrowIfCancellationRequested();var s=source[i];_start[i]=Node(new(s.Kind,s.StartNodeId));_end[i]=Node(new(s.Kind,s.EndNodeId));
            incidence[_start[i]].Add(i);incidence[_end[i]].Add(i);_curves[i]=new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);
            _lengths[i]=UpperLength(_curves[i]);
        }
        var adjacency=incidence.Select(_=>new List<ProfileGraphArc>()).ToArray();
        for(var i=0;i<source.Count;i++)
        {
            var cost=_lengths[i]*NetworkGeometrySafety.GradeLimit(source[i]);
            adjacency[_start[i]].Add(new(_end[i],cost));adjacency[_end[i]].Add(new(_start[i],cost));
        }
        _graph=new(adjacency.Select(a=>a.ToArray()).ToArray());
        _maximumGap=Math.Max(0,5+envelopes.Max(e=>(double)e.SurfaceMax)-envelopes.Min(e=>(double)e.RangeMin)+epsilon);
        for(var node=0;node<incidence.Count;node++)
        {
            var arms=incidence[node];if(arms.Count<2)continue;var a=source[arms[0]];
            if(arms.All(i=>a.Layer==source[i].Layer&&NativeAssetContract.Mode(a)==NativeAssetContract.Mode(source[i])))continue;
            var pa=_start[arms[0]]==node?a.Start:a.End;
            if(arms.Any(i=>!math.all((_start[i]==node?source[i].Start:source[i].End).xz==pa.xz)))continue;
            // A canonical portal/abutment connects even when its source layer
            // changes (bridge tags also create inferred layer defaults). Only
            // this unique continuation's actual local formation is exempt.
            double radius=arms.Max(i=>envelopes[i].NodeOffset+Math.Max(source[i].NativeWidth,envelopes[i].Width)/2);
            // Outward branches at an acute merge overlap beyond a node-centred
            // half-width square. Bound their local fan by strip separation,
            // stopping traversal at the next branch/mode boundary below.
            foreach(var i in arms)foreach(var j in arms)
            {
                if(i>=j)continue;
                var di=math.normalizesafe((_start[i]==node?_curves[i].P1-_curves[i].P0:_curves[i].P2-_curves[i].P3).xz);
                var dj=math.normalizesafe((_start[j]==node?_curves[j].P1-_curves[j].P0:_curves[j].P2-_curves[j].P3).xz);
                if(math.dot(di,dj)<=0)continue;
                var sine=Math.Abs(di.x*dj.y-di.y*dj.x);
                if(sine<=1e-6)continue; // Coincident branches need horizontal reconstruction, not an unbounded exemption.
                radius=Math.Max(radius,(Math.Max(source[i].NativeWidth,envelopes[i].Width)+Math.Max(source[j].NativeWidth,envelopes[j].Width))/(2*sine));
            }
            foreach(var seed in arms)
            {
                var current=seed;var at=node;var distance=0d;var seen=new HashSet<int>();
                while(distance<radius&&seen.Add(current))
                {
                    token.ThrowIfCancellationRequested();var forward=_start[current]==at;var length=_lengths[current];
                    double Parameter(double allowance)
                    {
                        var lo=0d;var hi=1d;
                        for(var iteration=0;iteration<32;iteration++)
                        {
                            var t=(lo+hi)/2;var partial=forward?Subcurve(_curves[current],0,t):Subcurve(_curves[current],1-t,1);
                            if(UpperLength(partial)<=allowance)lo=t;else hi=t;
                        }
                        return lo;
                    }
                    var fraction=length<=radius-distance?1:Parameter(radius-distance);
                    if(!_local.TryGetValue(current,out var intervals))_local[current]=intervals=new();
                    intervals.Add((node,forward?0:1-fraction,forward?fraction:1));
                    distance+=length;at=forward?_end[current]:_start[current];
                    if(incidence[at].Count!=2)break;
                    var next=incidence[at][0]==current?incidence[at][1]:incidence[at][0];
                    if(NativeAssetContract.Mode(source[next])!=NativeAssetContract.Mode(source[seed]))break;current=next;
                }
            }
        }
    }

    public List<FootprintLeaf> SplitFormationLeaves(IReadOnlyList<FootprintLeaf> leaves,NativeCompositionEnvelope[] envelopes,CancellationToken token)
    {
        var result=new List<FootprintLeaf>(leaves.Count);
        foreach(var leaf in leaves)
        {
            if(!_local.TryGetValue(leaf.Curve,out var intervals)){result.Add(leaf with{Ordinal=result.Count});continue;}
            var cuts=new SortedSet<double>{leaf.Begin,leaf.End};
            foreach(var interval in intervals)foreach(var t in new[]{interval.Begin,interval.End})if(t>leaf.Begin&&t<leaf.End)cuts.Add(t);
            var values=cuts.ToArray();
            for(var i=1;i<values.Length;i++)
            {
                var first=values[i-1];var last=values[i];var parts=new List<FootprintLeaf>();
                var width=Math.Max(_source[leaf.Curve].NativeWidth,envelopes[leaf.Curve].Width)/2;
                NetworkFootprintMath.BuildLeaves(Subcurve(_curves[leaf.Curve],first,last),width,leaf.Curve,parts,token);
                foreach(var part in parts)result.Add(part with{Ordinal=result.Count,Begin=first+(last-first)*part.Begin,End=first+(last-first)*part.End});
            }
        }
        return result;
    }

    public bool LocalTransition(FootprintLeaf a,FootprintLeaf b)
    {
        if(!_local.TryGetValue(a.Curve,out var aa)||!_local.TryGetValue(b.Curve,out var bb))return false;
        foreach(var x in aa)if(a.Begin>=x.Begin&&a.End<=x.End)
            foreach(var y in bb)if(x.Transition==y.Transition&&b.Begin>=y.Begin&&b.End<=y.End)return true;
        return false;
    }

    public double RiseBudget(int a,double t,int b,double u,CancellationToken token)
    {
        var ca=NetworkGeometrySafety.GradeLimit(_source[a]);var cb=NetworkGeometrySafety.GradeLimit(_source[b]);
        var ap=UpperLength(Subcurve(_curves[a],0,t))*ca;var aq=UpperLength(Subcurve(_curves[a],t,1))*ca;
        var bp=UpperLength(Subcurve(_curves[b],0,u))*cb;var bq=UpperLength(Subcurve(_curves[b],u,1))*cb;
        double Path(int from,int to)
        {
            var key=from<to?(from,to):(to,from);
            if(!_paths.TryGetValue(key,out var value))
            {
                // Cache eviction limits memory only; every query is still
                // evaluated and no physical contact/row is omitted.
                if(_paths.Count>=65536)_paths.Clear();
                _paths[key]=value=_graph.Distance(from,to,_maximumGap,token);
            }
            return value;
        }
        return Math.Min(Math.Min(ap+Path(_start[a],_start[b])+bp,ap+Path(_start[a],_end[b])+bq),
            Math.Min(aq+Path(_end[a],_start[b])+bp,aq+Path(_end[a],_end[b])+bq));
    }

    private static Cubic3d Subcurve(Cubic3d c,double begin,double end)
    {
        if(begin==end){var p=NetworkDesignChecks.Evaluate(c,begin);return new(p,p,p,p);}
        if(end<1)c=NetworkDesignChecks.Split(c,end).Left;if(begin>0)c=NetworkDesignChecks.Split(c,begin/end).Right;return c;
    }
    private static double UpperLength(Cubic3d c)
        =>math.distance(c.P0.xz,c.P1.xz)+math.distance(c.P1.xz,c.P2.xz)+math.distance(c.P2.xz,c.P3.xz);
}
