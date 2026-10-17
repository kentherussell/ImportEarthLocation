using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// One station function owns a taper across all of its spans. Hermite handles
// include the derivative of the shift (and of the curved reference normal).
public static class NetworkApproachTapers
{
    public readonly record struct Shift(int Index,double Distance,double Span,double Reach,double Delta,long Group=0);
    public readonly record struct DirectionRequest(int Index,bool Start,double2 Outward);
    private readonly record struct Field(double Distance,double Span,double Reach,double Delta,double3 Vector,bool Normal);

    public static IReadOnlyList<PreparedSegment> Materialize(IReadOnlyList<PreparedSegment> source,IReadOnlyList<Shift> planned,
        ImportLog log,CancellationToken token,IReadOnlyList<DirectionRequest>? directions=null,bool includeLaneOffsets=true,IReadOnlyList<PreparedSegment>? fallback=null)
    {
        var directionRequestCount=directions?.Count??0;
        if(directions!=null&&directions.Count>0)
        {
            var failed=new HashSet<int>();source=NetworkApproachFrames.Prepare(source,directions,log,token,failed);directions=null;
            if(failed.Count>0&&fallback!=null&&fallback.Count==source.Count)
            {
                var groups=new HashSet<long>(planned.Where(s=>failed.Contains(s.Index)&&s.Group!=0).Select(s=>s.Group));
                foreach(var shift in planned)if(shift.Group!=0&&groups.Contains(shift.Group))failed.Add(shift.Index);
                var retained=source.ToArray();foreach(var index in failed)retained[index]=fallback[index];source=retained;
                planned=planned.Where(s=>!failed.Contains(s.Index)).ToArray();
                log.Diagnostic($"JOINT FRAME rollbackSections={failed.Count}; lateralAndHeadingRequestsRetiredTogether=true; originalSafeGeometryRetained=true");
            }
        }
        var fields=new Dictionary<int,List<Field>>();var nodes=new Dictionary<SourceNodeKey,List<(int Index,bool Start)>>();
        void Add(int index,Field field){if(!fields.TryGetValue(index,out var list))fields[index]=list=new();list.Add(field);}
        foreach(var shift in planned)Add(shift.Index,new(shift.Distance,shift.Span,shift.Reach,shift.Delta,default,true));
        for(var i=0;i<source.Count;i++)if(source[i].Kind==NetworkKind.Rail||source[i].Kind==NetworkKind.Road&&source[i].OneWay)
            foreach(var start in new[]{true,false})
            {var id=new SourceNodeKey(source[i].Kind,start?source[i].StartNodeId:source[i].EndNodeId);if(!nodes.TryGetValue(id,out var list))nodes[id]=list=new();list.Add((i,start));}
        Cubic3d Base(PreparedSegment s)=>s.Kind==NetworkKind.Rail?new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd):
            new(s.Start,s.Control1??math.lerp(s.Start,s.End,1f/3),s.Control2??math.lerp(s.Start,s.End,2f/3),s.End);
        double3 Planned(int index,double t)
        {
            var curve=Base(source[index]);var value=double3.zero;
            if(fields.TryGetValue(index,out var list))foreach(var field in list)value+=Offset(curve,field,t).Value;
            return value;
        }
        // Freeze residual port requests before propagation; a contribution from
        // one port must not recursively become another request at a later knot.
        var ports=new List<(int Index,bool Start,double3 Delta)>();
        for(var i=0;i<source.Count;i++)if(includeLaneOffsets&&source[i].Kind==NetworkKind.Road&&source[i].OneWay)
            foreach(var start in new[]{true,false})
            {
                var delta=(double3)(start?source[i].StartCurveOffset:source[i].EndCurveOffset)-Planned(i,start?0:1);
                if(math.length(delta)>.001)ports.Add((i,start,delta));
            }
        var constrained=0;
        foreach(var port in ports)
        {
            token.ThrowIfCancellationRequested();var desired=Math.Max(220,1.875*math.length(port.Delta)/Math.Tan(Math.PI/90));
            var path=new List<(int Index,bool Start,double Length)>();var seen=new HashSet<int>();
            var index=port.Index;var start=port.Start;var available=0d;
            while(seen.Add(index))
            {
                token.ThrowIfCancellationRequested();var s=source[index];var length=(double)NetworkVerticalProfiles.ArcLength(s with{StartCurveOffset=default,EndCurveOffset=default,FinalDeckCurve=null});
                path.Add((index,start,length));available+=length;if(available>=desired)break;
                var node=start?s.EndNodeId:s.StartNodeId;
                if(!nodes.TryGetValue(new(s.Kind,node),out var arms)||arms.Count!=2)break;
                var next=arms[0].Index==index?arms[1]:arms[0];var other=source[next.Index];
                if(other.Tunnel!=s.Tunnel||other.Bridge!=s.Bridge||other.NativeAssetKey!=s.NativeAssetKey||
                    other.ProtectedLink!=s.ProtectedLink||next.Start!=start)break;
                index=next.Index;start=next.Start;
            }
            var reach=Math.Min(desired,available);if(reach<desired-.01)constrained++;
            if(!(reach>0))continue;var distance=0d;
            foreach(var step in path)
            {
                Add(step.Index,new(step.Start?distance:distance+step.Length,step.Start?step.Length:-step.Length,reach,1,port.Delta,false));
                distance+=step.Length;
            }
        }
        (double3 Value,double3 Derivative) Desired(int index,double t)
        {
            var c=Base(source[index]);var value=NetworkDesignChecks.Evaluate(c,t);var derivative=NetworkDesignChecks.Derivative(c,t);
            if(fields.TryGetValue(index,out var terms))foreach(var f in terms)
            {var offset=Offset(c,f,t);value+=offset.Value;derivative+=offset.Derivative;}
            return (value,derivative);
        }
        var sharedDerivatives=new Dictionary<(int Index,bool Start),double3>();
        foreach(var node in nodes.Values)
        {
            if(node.Count!=2)continue;
            if(source[node[0].Index].Kind==NetworkKind.Rail)
            {
                var p=node[0];var q=node[1];
                if(!fields.ContainsKey(p.Index)&&!fields.ContainsKey(q.Index))continue;
                var aa=Desired(p.Index,p.Start?0:1);var bb=Desired(q.Index,q.Start?0:1);
                var spanA=NetworkDesignChecks.HorizontalArcLength(Base(source[p.Index]));var spanB=NetworkDesignChecks.HorizontalArcLength(Base(source[q.Index]));
                if(!(spanA>0&&spanB>0)||math.distance(aa.Value,bb.Value)>.01)continue;
                var da=aa.Derivative*(p.Start?1:-1)/spanA;var db=bb.Derivative*(q.Start?1:-1)/spanB;
                if(math.dot(math.normalizesafe(da.xz),math.normalizesafe(db.xz))>-.9)continue;
                var commonRail=!fields.ContainsKey(p.Index)?da:!fields.ContainsKey(q.Index)?-db:(da-db)/2;
                sharedDerivatives[(p.Index,p.Start)]=commonRail*spanA*(p.Start?1:-1);
                sharedDerivatives[(q.Index,q.Start)]=-commonRail*spanB*(q.Start?1:-1);
                continue;
            }
            if(node[0].Start==node[1].Start)continue;
            var incoming=node[0].Start?node[1]:node[0];var outgoing=node[0].Start?node[0]:node[1];
            if(!fields.ContainsKey(incoming.Index)||!fields.ContainsKey(outgoing.Index))continue;
            var a=source[incoming.Index];var b=source[outgoing.Index];
            if(a.NativeAssetKey!=b.NativeAssetKey||a.Lanes!=b.Lanes||a.Tunnel!=b.Tunnel||a.Bridge!=b.Bridge)continue;
            var end=Desired(incoming.Index,1);var start=Desired(outgoing.Index,0);
            if(math.distance(end.Value,start.Value)>.01||math.dot(math.normalizesafe(end.Derivative.xz),math.normalizesafe(start.Derivative.xz))<.9)continue;
            var la=(double)NetworkVerticalProfiles.ArcLength(a with{StartCurveOffset=default,EndCurveOffset=default,FinalDeckCurve=null});
            var lb=(double)NetworkVerticalProfiles.ArcLength(b with{StartCurveOffset=default,EndCurveOffset=default,FinalDeckCurve=null});
            if(!(la>0&&lb>0))continue;
            // C1 source cubics need not have equal curvature. An offset of
            // those cubics otherwise gains a tangent jump through delta*N'.
            // Publish one station derivative and include its fitting error.
            var common=(end.Derivative/la+start.Derivative/lb)/2;
            sharedDerivatives[(incoming.Index,false)]=common*la;sharedDerivatives[(outgoing.Index,true)]=common*lb;
        }
        var occupied=new HashSet<long>(source.SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}));var nextNode=long.MinValue+20000000;
        long Node(){while(occupied.Contains(nextNode))nextNode++;var value=nextNode++;occupied.Add(value);return value;}
        var result=new List<PreparedSegment>(source.Count);var added=0;var review=0;var boundMaximum=0d;
        for(var index=0;index<source.Count;index++)
        {
            token.ThrowIfCancellationRequested();var s=source[index];
            if(!fields.TryGetValue(index,out var list)){result.Add(s);continue;}
            var curve=Base(s);
            // Preserve invalid input for the native safety guard. A NaN error
            // bound is neither a certified fit nor a reason to subdivide it.
            if(!NetworkDesignChecks.Finite(curve)){result.Add(s);review++;continue;}
            var cuts=new SortedSet<double>{0,1};
            foreach(var field in list)if(field.Span!=0)
            {var end=(field.Reach-field.Distance)/field.Span;if(end>0&&end<1)cuts.Add(end);}
            var coordinates=new Dictionary<double,(long Id,float3 Anchor)>{{0,(s.StartNodeId,s.Start)},{1,(s.EndNodeId,s.End)}};
            (long Id,float3 Anchor) Port(double t)
            {if(!coordinates.TryGetValue(t,out var p))coordinates[t]=p=(Node(),(float3)NetworkDesignChecks.Evaluate(curve,t));return p;}
            (double3 Value,double3 Derivative) Evaluate(double t)
            {
                var value=Desired(index,t);
                if(t==0&&sharedDerivatives.TryGetValue((index,true),out var a))value.Derivative=a;
                if(t==1&&sharedDerivatives.TryGetValue((index,false),out var b))value.Derivative=b;
                return value;
            }
            var leaves=0;
            void Fit(double begin,double end,int depth)
            {
                token.ThrowIfCancellationRequested();var a=Evaluate(begin);var b=Evaluate(end);
                var fitted=NetworkDesignChecks.Hermite(a.Value,b.Value,a.Derivative,b.Derivative,end-begin);
                var sub=NetworkDesignChecks.Split(curve,end).Left;
                if(begin>0)sub=NetworkDesignChecks.Split(sub,begin/end).Right;
                var error=0d;foreach(var field in list)error+=FourthBound(sub,field,begin,end)/384;
                // Hermite's derivative basis has maximum magnitude 4/27.
                // Account for the explicitly shared source-knot derivative;
                // do not claim the unmodified-function error theorem alone.
                error+=4d/27*(end-begin)*(math.distance(a.Derivative,Desired(index,begin).Derivative)+math.distance(b.Derivative,Desired(index,end).Derivative));
                if(error>.045&&depth<12&&leaves<127&&NetworkEmissionStations.CanSplit(fitted,.5))
                {var middle=(begin+end)/2;Fit(begin,middle,depth+1);Fit(middle,end,depth+1);return;}
                var rounded=new Cubic3f((float3)fitted.P0,(float3)fitted.P1,(float3)fitted.P2,(float3)fitted.P3);
                var rounding=Math.Max(Math.Max(math.distance(fitted.P0,rounded.P0),math.distance(fitted.P1,rounded.P1)),
                    Math.Max(math.distance(fitted.P2,rounded.P2),math.distance(fitted.P3,rounded.P3)));
                leaves++;if(error+rounding>.05)review++;else boundMaximum=Math.Max(boundMaximum,error);
                var p=Port(begin);var q=Port(end);var p0=(float3)fitted.P0;var p3=(float3)fitted.P3;
                var startOffset=p0-p.Anchor;var endOffset=p3-q.Anchor;
                var materialized=s with{Start=p.Anchor,End=q.Anchor,StartNodeId=p.Id,EndNodeId=q.Id,
                    StartCurveOffset=startOffset,EndCurveOffset=endOffset,Control1=(float3)fitted.P1-startOffset,Control2=(float3)fitted.P2-endOffset,
                    FinalDeckCurve=rounded,HorizontalParent=null,HorizontalEmissionReference=fitted,
                    StartTerminal=begin==0&&s.StartTerminal,EndTerminal=end==1&&s.EndTerminal,
                    StartMiniRoundabout=begin==0&&s.StartMiniRoundabout,EndMiniRoundabout=end==1&&s.EndMiniRoundabout,
                    StartRoundaboutRadius=begin==0?s.StartRoundaboutRadius:0,EndRoundaboutRadius=end==1?s.EndRoundaboutRadius:0,
                    StartElevation=Elevation(begin),EndElevation=Elevation(end)};
                result.Add(materialized with{SourceGeometry=Enumerable.Range(0,33).Select(j=>NetworkNormalizer.Evaluate(materialized,j/32f)).ToArray()});
                float? Elevation(double t)=>s.StartElevation.HasValue&&s.EndElevation.HasValue?
                    math.lerp(s.StartElevation.Value,s.EndElevation.Value,(float)t):t==0?s.StartElevation:t==1?s.EndElevation:null;
            }
            var stations=NetworkEmissionStations.Cuts(curve,cuts);var before=result.Count;
            for(var i=1;i<stations.Length;i++)Fit(stations[i-1],stations[i],0);
            added+=result.Count-before-1;
        }
        log.Count("Whole-approach taper extra spans",added);log.Count("Taper fits requiring review",review);
        log.Diagnostic($"APPROACH TAPERS stationFunctions={planned.Count}; directionRequests={directionRequestCount}; residualPorts={ports.Count}; constrainedApproaches={constrained}; addedSpans={added}; uncertifiedFits={review}; maximumCertifiedDoubleFitErrorMeters={boundMaximum:R}; doubleFitBudgetMeters=0.045; floatBudgetMeters=0.005; derivativeAware=true; curvedNormalDerivative=true; independentEndpointSmoothsteps=false; qualityFailureBlocksPlacement=false");
        return result;
    }

    private static (double3 Value,double3 Derivative) Offset(Cubic3d c,Field f,double t)
    {
        var shift=NetworkDesignChecks.QuinticShift(f.Delta,f.Distance+f.Span*t,f.Reach);
        if(!f.Normal)return (f.Vector*shift.Value,f.Vector*(shift.First*f.Span));
        var v=NetworkDesignChecks.Derivative(c,t).xz;var a=NetworkDesignChecks.SecondDerivative(c,t).xz;var speed=math.length(v);
        if(!(speed>1e-12))return (default,default);
        var n=new double3(v.y/speed,0,-v.x/speed);
        var dv=a/speed-v*(math.dot(v,a)/(speed*speed*speed));var dn=new double3(dv.y,0,-dv.x);
        return (n*shift.Value,n*(shift.First*f.Span)+dn*shift.Value);
    }

    private static double FourthBound(Cubic3d c,Field f,double begin,double end)
    {
        var lo=Math.Min(f.Distance+f.Span*begin,f.Distance+f.Span*end);
        if(lo>=f.Reach)return 0;
        var span=Math.Abs(f.Span*(end-begin))/f.Reach;var d=Math.Abs(f.Delta);
        var d1=1.875*d*span;var d2=5.773502692*d*span*span;var d3=60*d*span*span*span;var d4=360*d*Math.Pow(span,4);
        if(!f.Normal)return math.length(f.Vector)*d4;
        var v0=3*(c.P1-c.P0).xz;var v1=3*(c.P2-c.P1).xz;var v2=3*(c.P3-c.P2).xz;
        var direction=math.normalizesafe(c.P3.xz-c.P0.xz);var lower=Math.Min(math.dot(v0,direction),Math.Min(math.dot(v1,direction),math.dot(v2,direction)));
        if(!(lower>1e-12))return double.PositiveInfinity;
        var v=Math.Max(math.length(v0),Math.Max(math.length(v1),math.length(v2)));
        var a=Math.Max(math.length(2*(v1-v0)),math.length(2*(v2-v1)));var j=math.length(2*(v2-2*v1+v0));
        var r=lower*lower;var r1=2*v*a;var r2=2*(a*a+v*j);var r3=6*a*j;var r4=6*j*j;
        var q=1/lower;var q1=.5*Math.Pow(r,-1.5)*r1;
        var q2=.75*Math.Pow(r,-2.5)*r1*r1+.5*Math.Pow(r,-1.5)*r2;
        var q3=1.875*Math.Pow(r,-3.5)*Math.Pow(r1,3)+2.25*Math.Pow(r,-2.5)*r1*r2+.5*Math.Pow(r,-1.5)*r3;
        var q4=6.5625*Math.Pow(r,-4.5)*Math.Pow(r1,4)+11.25*Math.Pow(r,-3.5)*r1*r1*r2+
            2.25*Math.Pow(r,-2.5)*r2*r2+3*Math.Pow(r,-2.5)*r1*r3+.5*Math.Pow(r,-1.5)*r4;
        var n1=a*q+v*q1;var n2=j*q+2*a*q1+v*q2;var n3=3*j*q1+3*a*q2+v*q3;var n4=6*j*q2+4*a*q3+v*q4;
        // Cubic Hermite value error <= sup ||F''''|| / 384 on [0,1].
        return n4*d+4*n3*d1+6*n2*d2+4*n1*d3+d4;
    }
}
