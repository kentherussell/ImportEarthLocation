using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// One source-based matcher/model feeds both horizontal emission and shared
// profile families. Ambiguous or uncertified intervals retain independent curves.
public static class NetworkPairedCorridors
{
    // Internal spline-to-native approximation, not the source-fit, clearance or
    // paired-height tolerance. Actual emitted separation is certified below.
    private const double EmissionFitTolerance=.25;
    internal static bool Separated(Cubic3d a,Cubic3d b,double minimum,CancellationToken token)
    {
        var visits=0;
        bool Prove(Cubic3d first,Cubic3d second,int firstDepth,int secondDepth)
        {
            token.ThrowIfCancellationRequested();if(++visits>32768)return false;
            var lowA=math.min(math.min(first.P0.xz,first.P1.xz),math.min(first.P2.xz,first.P3.xz));
            var highA=math.max(math.max(first.P0.xz,first.P1.xz),math.max(first.P2.xz,first.P3.xz));
            var lowB=math.min(math.min(second.P0.xz,second.P1.xz),math.min(second.P2.xz,second.P3.xz));
            var highB=math.max(math.max(second.P0.xz,second.P1.xz),math.max(second.P2.xz,second.P3.xz));
            if(math.length(math.max(double2.zero,math.max(lowA-highB,lowB-highA)))>=minimum-1e-7)return true;
            var delta=NetworkDesignChecks.Evaluate(first,.5).xz-NetworkDesignChecks.Evaluate(second,.5).xz;
            var distance=math.length(delta);if(!(distance>=minimum-1e-7))return false;var axis=delta/distance;
            var lowerA=Math.Min(Math.Min(math.dot(first.P0.xz,axis),math.dot(first.P1.xz,axis)),
                Math.Min(math.dot(first.P2.xz,axis),math.dot(first.P3.xz,axis)));
            var upperB=Math.Max(Math.Max(math.dot(second.P0.xz,axis),math.dot(second.P1.xz,axis)),
                Math.Max(math.dot(second.P2.xz,axis),math.dot(second.P3.xz,axis)));
            if(lowerA-upperB>=minimum-1e-7)return true;
            // Independent curve parameters: equal-station distance alone can
            // miss a diagonal overlap. Split one hull and cover both products,
            // including cross-boundary parameter combinations.
            if(firstDepth==16&&secondDepth==16)return false;
            if(secondDepth==16||firstDepth<16&&math.lengthsq(highA-lowA)>=math.lengthsq(highB-lowB))
            {var split=NetworkDesignChecks.Split(first,.5);return Prove(split.Left,second,firstDepth+1,secondDepth)&&Prove(split.Right,second,firstDepth+1,secondDepth);}
            var other=NetworkDesignChecks.Split(second,.5);return Prove(first,other.Left,firstDepth,secondDepth+1)&&Prove(first,other.Right,firstDepth,secondDepth+1);
        }
        return NetworkDesignChecks.Finite(a)&&NetworkDesignChecks.Finite(b)&&minimum>0&&Prove(a,b,0,0);
    }
    private sealed class Chain
    {
        public int Id;public readonly List<int> Edges=new();public readonly List<Piece> Pieces=new();
        public readonly Dictionary<int,(int First,int Last)> Ranges=new();
        public readonly List<(double Begin,double End)> JunctionCores=new();
        public double Length,StartCore,EndCore;
    }
    private sealed record Piece(int Chain,int Edge,double Begin,double End,double Station,double Length,double2 A,double2 B,double2 Minimum,double2 Maximum,int Ordinal);
    private readonly record struct Projection(int Chain,int Edge,double Parameter,double Station,double2 Point,double2 Heading,double Distance);
    private readonly record struct Match(double A,double B,double2 PA,double2 PB,int Side);
    private sealed record Interval(Chain A,Chain B,Match[] Samples);
    private static bool Eligible(PreparedSegment s)=>s.Kind==NetworkKind.Road&&s.OneWay&&!s.ProtectedLink&&!s.Roundabout&&
        s.Classification is "motorway" or "trunk"&&s.NativeWidth>0&&math.isfinite(s.NativeWidth)&&
        !s.SourceTags.ContainsKey("reversible")&&(!s.SourceTags.TryGetValue("oneway",out var way)||way!="reversible");
    internal static bool Related(PreparedSegment a,PreparedSegment b)=>a.Tunnel==b.Tunnel&&
        (a.Layer==b.Layer||!a.RawSourceTags.ContainsKey("layer")&&!b.RawSourceTags.ContainsKey("layer")&&a.Bridge!=b.Bridge)&&a.Classification==b.Classification&&
        (a.CorridorId!=0&&a.CorridorId==b.CorridorId||a.Reference.Length>0&&a.Reference==b.Reference||a.Name.Length>0&&a.Name==b.Name);
    private static Cubic3d Curve(PreparedSegment s)=>new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);

    public static IReadOnlyList<PreparedSegment> Prepare(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token,Action<string>? progress=null,
        double width=double.PositiveInfinity,IReadOnlyList<float3[]>? protectedAreas=null,IReadOnlyList<float3[]>? protectedLines=null,
        IReadOnlyDictionary<string,NativeAssetContract>? assets=null,Action<string,IReadOnlyList<PreparedSegment>>? inspectPublication=null)
    {
        var protection=protectedAreas!=null&&protectedLines!=null?new NetworkFormationProtection(width,protectedAreas,protectedLines,token):null;
        var incident=new Dictionary<long,List<int>>();
        for(var i=0;i<source.Count;i++)if(source[i].Kind==NetworkKind.Road)foreach(var id in new[]{source[i].StartNodeId,source[i].EndNodeId})
        {if(!incident.TryGetValue(id,out var list))incident[id]=list=new();list.Add(i);}
        var chains=new List<Chain>();var visited=new bool[source.Count];var pieceCount=0;
        // Correspondence survives an abutment or lane-width boundary. Emission
        // cuts at every source edge below retain each side's actual asset/mode.
        bool Continue(int a,int b)=>Eligible(source[b])&&(source[a].WayId==source[b].WayId||Related(source[a],source[b]))&&
            source[a].Tunnel==source[b].Tunnel;
        int Next(int edge,bool backwards)
        {
            var s=source[edge];var node=backwards?s.StartNodeId:s.EndNodeId;
            // A ramp branch does not end the mainline corridor identity. Its
            // footprint is excluded below, while source edge cuts keep the port.
            var main=incident[node].Where(i=>!source[i].ProtectedLink).ToArray();
            if(main.Length!=2||incident[node].Any(i=>i!=edge&&!source[i].ProtectedLink&&!Eligible(source[i])))return -1;
            var next=main[0]==edge?main[1]:main[0];
            // A retained port remains an emission event, not a new corridor.
            // Stopping here hid most carriageways behind endpoint cores.
            return Continue(edge,next)&&(backwards?source[next].EndNodeId:source[next].StartNodeId)==node?next:-1;
        }
        foreach(var seed in Enumerable.Range(0,source.Count).OrderBy(i=>source[i].WayId).ThenBy(i=>source[i].StartNodeId))
        {
            token.ThrowIfCancellationRequested();if(visited[seed]||!Eligible(source[seed]))continue;
            var first=seed;var seen=new HashSet<int>{seed};
            while(true){var previous=Next(first,true);if(previous<0||visited[previous]||!seen.Add(previous))break;first=previous;}
            var chain=new Chain{Id=chains.Count};var index=first;
            while(!visited[index])
            {
                visited[index]=true;chain.Edges.Add(index);var c=Curve(source[index]);var firstPiece=chain.Pieces.Count;
                if(incident[source[index].StartNodeId].Count>2)
                {var radius=2*source[index].NativeWidth;chain.JunctionCores.Add((chain.Length-radius,chain.Length+radius));}
                if(NetworkDesignChecks.Finite(c))Flatten(c,0,1,0);
                void Flatten(Cubic3d part,double begin,double end,int depth)
                {
                    token.ThrowIfCancellationRequested();var a=part.P0.xz;var b=part.P3.xz;
                    var error=Math.Max(math.distance(part.P1.xz,(2*a+b)/3),math.distance(part.P2.xz,(a+2*b)/3));
                    if((math.distance(a,b)>10||error>.01)&&depth<18)
                    {var split=NetworkDesignChecks.Split(part,.5);var middle=(begin+end)/2;Flatten(split.Left,begin,middle,depth+1);Flatten(split.Right,middle,end,depth+1);return;}
                    var length=math.distance(a,b);if(length<=1e-8)return;
                    var minimum=math.min(math.min(part.P0.xz,part.P1.xz),math.min(part.P2.xz,part.P3.xz));
                    var maximum=math.max(math.max(part.P0.xz,part.P1.xz),math.max(part.P2.xz,part.P3.xz));
                    chain.Pieces.Add(new(chain.Id,index,begin,end,chain.Length,length,a,b,minimum,maximum,pieceCount++));chain.Length+=length;
                }
                if(chain.Pieces.Count>firstPiece)chain.Ranges[index]=(firstPiece,chain.Pieces.Count-1);
                var next=Next(index,false);if(next<0||visited[next])break;index=next;
            }
            var head=source[chain.Edges[0]];var tail=source[chain.Edges[^1]];
            // Only real junction/semantic boundaries own exclusion cores. A
            // legitimate external terminal does not hide a complete straight pair.
            chain.StartCore=incident[head.StartNodeId].Count>2?2*head.NativeWidth:0;
            chain.EndCore=incident[tail.EndNodeId].Count>2?2*tail.NativeWidth:0;chains.Add(chain);
        }
        var grid=new Dictionary<(int,int),List<Piece>>();
        foreach(var chain in chains)foreach(var p in chain.Pieces)
        {
            var min=p.Minimum;var max=p.Maximum;
            for(var z=(int)Math.Floor(min.y/64);z<=(int)Math.Floor(max.y/64);z++)for(var x=(int)Math.Floor(min.x/64);x<=(int)Math.Floor(max.x/64);x++)
            {if(!grid.TryGetValue((x,z),out var list))grid[(x,z)]=list=new();list.Add(p);}
        }
        bool Interior(Projection p)=>p.Station>=chains[p.Chain].StartCore&&p.Station<=chains[p.Chain].Length-chains[p.Chain].EndCore&&
            !chains[p.Chain].JunctionCores.Any(c=>p.Station>=c.Begin&&p.Station<=c.End);
        var projected=new int[pieceCount];var query=0;
        long projectionCandidates=0,projectionBoundsRejected=0;
        Projection? Closest(int own,double2 point,double2 heading)
        {
            if(query==int.MaxValue){Array.Clear(projected,0,projected.Length);query=0;}query++;
            var candidates=new Dictionary<int,Projection>();var gx=(int)Math.Floor(point.x/64);var gz=(int)Math.Floor(point.y/64);
            var reference=source[chains[own].Edges[0]];
            for(var z=gz-2;z<=gz+2;z++)for(var x=gx-2;x<=gx+2;x++)if(grid.TryGetValue((x,z),out var pieces))foreach(var p in pieces)
            {
                if(projected[p.Ordinal]==query)continue;projected[p.Ordinal]=query;
                if(p.Chain==own||!Related(reference,source[p.Edge]))continue;
                projectionCandidates++;
                // Precomputed exact control-hull bounds reject only pieces
                // outside the radius or unable to improve this chain's best
                // observed projection. This is candidate discovery, not proof.
                var lower=math.distance(point,math.clamp(point,p.Minimum,p.Maximum));
                if(lower>96||candidates.TryGetValue(p.Chain,out var nearest)&&lower>nearest.Distance)
                {projectionBoundsRejected++;continue;}
                var u=math.clamp(math.dot(point-p.A,p.B-p.A)/math.lengthsq(p.B-p.A),0,1);var t=p.Begin+(p.End-p.Begin)*u;
                var c=Curve(source[p.Edge]);
                // Discovery only: flattening bounds the initial projection
                // error. Continuous proof belongs to the selected emission,
                // not to every one of millions of discarded candidate roots.
                for(var probe=0;probe<4;probe++)
                {
                    var position=NetworkDesignChecks.Evaluate(c,t).xz;var velocity=NetworkDesignChecks.Derivative(c,t).xz;
                    var denominator=math.dot(velocity,velocity)+math.dot(position-point,NetworkDesignChecks.SecondDerivative(c,t).xz);
                    if(!(denominator>1e-12))break;
                    t=math.clamp(t-math.dot(position-point,velocity)/denominator,p.Begin,p.End);
                }
                var q0=NetworkDesignChecks.Evaluate(c,t).xz;var direction=math.normalizesafe(NetworkDesignChecks.Derivative(c,t).xz);
                var distance=math.distance(point,q0);if(distance>96||math.dot(heading,direction)>-Math.Cos(Math.PI/12))continue;
                var candidate=new Projection(p.Chain,p.Edge,t,p.Station+p.Length*(t-p.Begin)/(p.End-p.Begin),q0,direction,distance);
                if(!Interior(candidate))continue;
                if(!candidates.TryGetValue(p.Chain,out var old)||distance<old.Distance)candidates[p.Chain]=candidate;
            }
            var sorted=candidates.Values.OrderBy(p=>p.Distance).ThenBy(p=>p.Chain).ToArray();if(sorted.Length==0)return null;
            if(sorted.Length>1&&sorted[1].Distance-sorted[0].Distance<Math.Max(2,.25*sorted[0].Distance))return null;
            return sorted[0];
        }
        Projection At(Chain chain,double station)
        {
            var lo=0;var hi=chain.Pieces.Count-1;
            while(lo<hi){var mid=(lo+hi)/2;if(chain.Pieces[mid].Station+chain.Pieces[mid].Length<station)lo=mid+1;else hi=mid;}
            var p=chain.Pieces[lo];var t=p.Begin+(p.End-p.Begin)*math.clamp((station-p.Station)/p.Length,0,1);var c=Curve(source[p.Edge]);
            return new(chain.Id,p.Edge,t,station,NetworkDesignChecks.Evaluate(c,t).xz,math.normalizesafe(NetworkDesignChecks.Derivative(c,t).xz),0);
        }
        double ParameterOnEdge(Chain chain,int edge,double station)
        {
            var range=chain.Ranges[edge];var lo=range.First;var hi=range.Last;
            while(lo<hi){var mid=(lo+hi)/2;if(chain.Pieces[mid].Station+chain.Pieces[mid].Length<station)lo=mid+1;else hi=mid;}
            var piece=chain.Pieces[lo];return piece.Begin+(piece.End-piece.Begin)*math.clamp((station-piece.Station)/piece.Length,0,1);
        }
        var intervals=new List<Interval>();var ambiguous=0;
        foreach(var chain in chains)
        {
            progress?.Invoke($"Matching opposing mainline chain {chain.Id+1}/{chains.Count}");
            if(chain.Pieces.Count==0)continue;var samples=new List<Match>();var partner=-1;
            void Flush()
            {if(samples.Count>1&&samples[^1].A-samples[0].A>=80)intervals.Add(new(chain,chains[partner],samples.ToArray()));samples.Clear();partner=-1;}
            var stations=new SortedSet<double>{0,chain.Length};
            for(var s=20d;s<chain.Length;s+=20)stations.Add(s);
            foreach(var core in chain.JunctionCores){stations.Add(math.clamp(core.Begin,0,chain.Length));stations.Add(math.clamp(core.End,0,chain.Length));}
            foreach(var station in stations)
            {
                token.ThrowIfCancellationRequested();var a=At(chain,station);
                if(!Interior(a)){Flush();continue;}
                var b=Closest(chain.Id,a.Point,a.Heading);
                if(!b.HasValue||b.Value.Chain<chain.Id){Flush();continue;}
                var back=Closest(b.Value.Chain,b.Value.Point,b.Value.Heading);
                // Nearest maps on curved variable-width offsets need not be
                // involutions. Require chain agreement and monotone stations.
                if(!back.HasValue||back.Value.Chain!=chain.Id){ambiguous++;Flush();continue;}
                var normal=new double2(-a.Heading.y,a.Heading.x);var sign=math.dot(a.Point-b.Value.Point,normal)<0?-1:1;
                if(partner!=b.Value.Chain||samples.Count>0&&(b.Value.Station>=samples[^1].B-1e-6||samples[^1].Side!=sign))Flush();
                partner=b.Value.Chain;samples.Add(new(station,b.Value.Station,a.Point,b.Value.Point,sign));
            }
            Flush();
        }
        var occupied=new HashSet<long>(source.SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}));var nextNode=long.MinValue+30000000;
        long NewNode(){while(occupied.Contains(nextNode))nextNode++;var node=nextNode++;occupied.Add(node);return node;}
        long familyId=0,spanId=0,corridorId=0;var profileOnlyIntervals=0;var straightMetrics=0;var replaced=new Dictionary<int,List<(double Begin,double End,PreparedSegment Segment)>>();
        var sourceCuts=new Dictionary<int,List<(double Parameter,long Node)>>();
        long SourceCut(Projection p)
        {
            if(p.Parameter<1e-7)return source[p.Edge].StartNodeId;
            if(p.Parameter>1-1e-7)return source[p.Edge].EndNodeId;
            if(!sourceCuts.TryGetValue(p.Edge,out var cuts))sourceCuts[p.Edge]=cuts=new();
            // Same original curve and parameter, not a spatial weld. This is
            // the identical numeric boundary tolerance used by Original below.
            foreach(var cut in cuts)if(Math.Abs(cut.Parameter-p.Parameter)<1e-7)return cut.Node;
            var node=NewNode();cuts.Add((p.Parameter,node));return node;
        }
        var movements=new Dictionary<long,float3>();var directions=new Dictionary<long,double2>();var rejected=0;var accepted=0;
        var originalAnchors=new Dictionary<long,float3>();var publishedAnchors=new Dictionary<long,float3>();
        foreach(var s in source.Where(s=>s.Kind==NetworkKind.Road))
        {if(!originalAnchors.ContainsKey(s.StartNodeId))originalAnchors[s.StartNodeId]=s.Start;if(!originalAnchors.ContainsKey(s.EndNodeId))originalAnchors[s.EndNodeId]=s.End;}
        var construction=log.DiagnosticsEnabled?new List<object>():null;var sourceReview=0;
        var designed=log.DiagnosticsEnabled?new Dictionary<long,(Cubic3d A,Cubic3d B,double WidthA,double WidthB)>():null;
        foreach(var interval in intervals)
        {
            progress?.Invoke($"Fitting trusted paired corridor {accepted+rejected+1}/{intervals.Count}");
            token.ThrowIfCancellationRequested();var sample=interval.Samples;var station=new List<double>{0};var points=new List<double3>();var gaps=new List<double>();
            var templates=new Dictionary<(int,int),PreparedSegment>();
            PreparedSegment Template(int own,int other)
            {
                if(templates.TryGetValue((own,other),out var template))return template;template=source[own];
                if(assets!=null&&assets.TryGetValue(template.NativeAssetKey,out var native)&&native.TryEnvelope(NativeAssetContract.Mode(template),out var envelope))
                {
                    var actualWidth=Math.Max(template.NativeWidth,envelope.Width);
                    template=template with{NativeWidth=actualWidth,NativeLaneOffsets=native.LaneOffsets,
                        NativeCenterlineCurvatureLimit=native.CenterlineCurvatureLimit(actualWidth)};
                }
                if(assets!=null&&NetworkVisualDesign.TryPairedBridge(template,source[other],assets,out var resolved))template=resolved;
                templates[(own,other)]=template;return template;
            }
            for(var i=0;i<sample.Length;i++)
            {
                var a=At(interval.A,sample[i].A);var b=At(interval.B,sample[i].B);
                var wa=Template(a.Edge,b.Edge).NativeWidth;var wb=Template(b.Edge,a.Edge).NativeWidth;
                var normal=new double2(-a.Heading.y,a.Heading.x);
                // Fit the median, not the midpoint of two changing-width
                // centre courses. Width is a native composition event, not a
                // reason to bend the shared reference at every lane change.
                var mid=(sample[i].PA+sample[i].PB)/2+sample[i].Side*normal*(wb-wa)/4;
                if(i>0){var delta=math.distance(mid,points[^1].xz);if(delta<=1e-6)continue;station.Add(station[^1]+delta);}
                gaps.Add(Math.Max(1.05,(math.distance(sample[i].PA,sample[i].PB)-(wa+wb)/2)/2));
                points.Add(new(mid.x,0,mid.y));
            }
            if(points.Count!=sample.Length||station[^1]<80){rejected++;continue;}
            var halfMedian=gaps.OrderBy(v=>v).ElementAt(gaps.Count/2);
            var rawPoints=points.ToArray();
            for(var i=0;i<points.Count;i++)
            {
                var sum=double2.zero;var moment=double2.zero;var weight=0d;var firstMoment=0d;var secondMoment=0d;
                void Add(int j)
                {
                    var x=station[j]-station[i];var w=1-Math.Abs(x)/140;
                    sum+=rawPoints[j].xz*w;moment+=rawPoints[j].xz*(w*x);weight+=w;firstMoment+=w*x;secondMoment+=w*x*x;
                }
                for(var j=i;j>=0&&station[i]-station[j]<=120;j--)
                    Add(j);
                for(var j=i+1;j<points.Count&&station[j]-station[i]<=120;j++)
                    Add(j);
                // Preserve the interval's two endpoint poses for the existing
                // boundary correction. Interior smoothing remains subject to
                // the unchanged continuous source/protection certificate.
                var determinant=weight*secondMoment-firstMoment*firstMoment;
                var mid=i==0||i==points.Count-1||determinant<=1e-12?rawPoints[i].xz:(sum*secondMoment-moment*firstMoment)/determinant;
                var t=Math.Min(1,Math.Min(station[i],station[station.Count-1]-station[i])/120);
                var gain=t*t*t*(10+t*(-15+6*t));
                mid=math.lerp(rawPoints[i].xz,mid,gain);
                points[i]=new(mid.x,halfMedian,mid.y);
            }
            var va=math.normalizesafe(points[1].xz-points[0].xz);var vb=math.normalizesafe(points[^1].xz-points[^2].xz);
            var spline=new NetworkReferenceSpline(station,points,new(va.x,0,va.y),new(vb.x,0,vb.y));var side=sample[0].Side;
            var needsRepair=sample.Any(p=>math.distance(p.PA,p.PB)<
                (Template(At(interval.A,p.A).Edge,At(interval.B,p.B).Edge).NativeWidth+Template(At(interval.B,p.B).Edge,At(interval.A,p.A).Edge).NativeWidth)/2+2.1);
            var coordinated=protection!=null&&needsRepair;
            var fitWidthA=0d;var fitWidthB=0d;var fitOffsetA=0d;var fitOffsetB=0d;
            var cuts=new SortedSet<double>{0,station[^1]};
            foreach(var (chain,a) in new[]{(interval.A,true),(interval.B,false)})foreach(var edge in chain.Edges)
            {
                if(!chain.Ranges.TryGetValue(edge,out var range))continue;
                var p=chain.Pieces[range.First];if(p.Begin!=0)continue;
                var s=Common(p.Station,a);if(s>0&&s<station[^1])cuts.Add(s);
            }
            var boundaries=cuts.ToArray();var datumA=0d;var datumB=0d;
            if(coordinated)for(var k=1;k<boundaries.Length;k++)
            {
                // Native composition is constant between the union of both
                // sides' source events. A distant width or bridge on either
                // chain must not move this interval's continuing-lane datum.
                var middle=(boundaries[k-1]+boundaries[k])/2;
                var a=TemplateAt(middle,true);var b=TemplateAt(middle,false);
                if(a.NativeLaneOffsets.Count>0)datumA=Math.Max(datumA,a.NativeWidth/2-side*a.NativeLaneOffsets.Min());
                if(b.NativeLaneOffsets.Count>0)datumB=Math.Max(datumB,b.NativeWidth/2-side*b.NativeLaneOffsets.Min());
            }
            double Offset(PreparedSegment template,bool isA)=>!coordinated||template.NativeLaneOffsets.Count==0?template.NativeWidth/2:
                (isA?datumA:datumB)+side*template.NativeLaneOffsets.Min();
            (double2 Point,double2 Derivative) RawDesired(double s,int sign,double nativeOffset)
            {
                var value=spline.Evaluate(s);var v=value.Derivative.xz;var speed=math.length(v);
                if(!(speed>1e-10))return(new(double.NaN),new(double.NaN));
                // The spline second derivative is exact from its cubic controls.
                var piece=0;while(piece+1<spline.Pieces&&s>spline.Station(piece+1))piece++;
                var c=spline.Curve(piece);var reach=spline.Station(piece+1)-spline.Station(piece);
                var acceleration=NetworkDesignChecks.SecondDerivative(c,(s-spline.Station(piece))/reach).xz/(reach*reach);
                var tangent=v/speed;var normal=new double2(-tangent.y,tangent.x);
                var dn=new double2(-(acceleration-tangent*math.dot(tangent,acceleration)).y,(acceleration-tangent*math.dot(tangent,acceleration)).x)/speed;
                var offset=value.Point.y+nativeOffset;
                return(value.Point.xz+sign*offset*normal,v+sign*(value.Derivative.y*normal+offset*dn));
            }
            // Unrepaired intervals retain their selected boundary frames.
            // Generated repairs select one shared reference below instead of
            // applying two incompatible corrections to that reference.
            Cubic3d BoundaryCorrection(bool isA)
            {
                var chain=isA?interval.A:interval.B;var sign=isA?side:-side;
                var first=At(chain,isA?sample[0].A:sample[0].B);var last=At(chain,isA?sample[^1].A:sample[^1].B);
                var opposite=isA?interval.B:interval.A;
                var otherFirst=At(opposite,isA?sample[0].B:sample[0].A);var otherLast=At(opposite,isA?sample[^1].B:sample[^1].A);
                var start=RawDesired(0,sign,Offset(Template(first.Edge,otherFirst.Edge),isA));var end=RawDesired(station[^1],sign,Offset(Template(last.Edge,otherLast.Edge),isA));
                var orientation=isA?1:-1;
                var startDerivative=first.Heading*orientation*math.length(start.Derivative);
                var endDerivative=last.Heading*orientation*math.length(end.Derivative);
                var lockStart=isA?chain.StartCore>0:chain.EndCore>0;
                var lockEnd=isA?chain.EndCore>0:chain.StartCore>0;
                double3 V(double2 p)=>new(p.x,0,p.y);
                return NetworkDesignChecks.Hermite(lockStart?V(first.Point-start.Point):default,lockEnd?V(last.Point-end.Point):default,
                    lockStart?V(startDerivative-start.Derivative):default,lockEnd?V(endDerivative-end.Derivative):default,station[^1]);
            }
            // Repair owns the generated courses, not frozen native poses.
            // Averaging two old boundary corrections after choosing compatible
            // lateral poses changes that choice again. The shared reference is
            // authoritative here; Original below carries its pose and tangent
            // into each attached source remainder before final certification.
            var correctionA=coordinated?default:BoundaryCorrection(true);
            var correctionB=coordinated?default:BoundaryCorrection(false);
            (double2 Point,double2 Derivative) Desired(double s,int sign)
            {
                var raw=RawDesired(s,sign,sign==side?fitOffsetA:fitOffsetB);var correction=sign==side?correctionA:correctionB;
                var t=s/station[^1];
                return(raw.Point+NetworkDesignChecks.Evaluate(correction,t).xz,
                    raw.Derivative+NetworkDesignChecks.Derivative(correction,t).xz/station[^1]);
            }
            double Correspondence(double s,bool a)
            {
                var k=station.BinarySearch(s);if(k<0)k=~k-1;k=Math.Max(0,Math.Min(sample.Length-2,k));var t=(s-station[k])/(station[k+1]-station[k]);
                return math.lerp(a?sample[k].A:sample[k].B,a?sample[k+1].A:sample[k+1].B,t);
            }
            PreparedSegment TemplateAt(double s,bool isA)
            {var a=At(interval.A,Correspondence(s,true)).Edge;var b=At(interval.B,Correspondence(s,false)).Edge;return isA?Template(a,b):Template(b,a);}
            double RequiredAt(double s)=>(TemplateAt(s,true).NativeWidth+TemplateAt(s,false).NativeWidth)/2+2.1;
            double Allowance(PreparedSegment s)=>Math.Min(24,s.SourceFitToleranceMeters??24);
            double Common(double sourceStation,bool a)
            {
                var first=a?sample[0].A:sample[0].B;var last=a?sample[^1].A:sample[^1].B;
                if(sourceStation<Math.Min(first,last)-1e-7||sourceStation>Math.Max(first,last)+1e-7)return double.NaN;
                var lo=0;var hi=sample.Length-2;
                while(lo<hi){var mid=(lo+hi)/2;var end=a?sample[mid+1].A:sample[mid+1].B;if(a?sourceStation>end:sourceStation<end)lo=mid+1;else hi=mid;}
                var p=a?sample[lo].A:sample[lo].B;var q=a?sample[lo+1].A:sample[lo+1].B;
                return math.lerp(station[lo],station[lo+1],(sourceStation-p)/(q-p));
            }
            var emitted=new List<(double Begin,double End,Cubic3d A,Cubic3d B)>();
            // Width events offset each side of the one median reference.
            // They do not deform the reference or add another taper writer.
            var invalid=false;var fallbackReason="";object? budgetFailure=null;var sourceFidelityFailed=false;
            // In coordinated design, source/protection certification is
            // quality evidence, not a frozen native placement boundary. Keep
            // those failures visible rather than retaining a known overlap.
            // The 24 m construction cap is independent and remains mandatory.
            double LocalAllowance(PreparedSegment s)=>Math.Max(Allowance(s),coordinated?24:0);
            double fitBegin=0,fitEnd=0;var fitFailure="";var endpointError=0d;var offsetRegularity=0d;
            bool Fits(double begin,double end,Cubic3d fitted,int sign,int depth,double tolerance=EmissionFitTolerance-.005)
            {
                // Independent check subdivision: the native curve is not split
                // merely because a proof needs a tighter reference interval.
                var a=Desired(begin,sign);var b=Desired(end,sign);var ta=(begin-fitBegin)/(fitEnd-fitBegin);var tb=(end-fitBegin)/(fitEnd-fitBegin);
                var ea=math.distance(a.Point,NetworkDesignChecks.Evaluate(fitted,ta).xz);var eb=math.distance(b.Point,NetworkDesignChecks.Evaluate(fitted,tb).xz);
                if(!SparseProfileRows.Finite(ea+eb)||ea>tolerance||eb>tolerance){fitFailure="endpoint-error";endpointError=Math.Max(ea,eb);return false;}
                var piece=0;while(piece+1<spline.Pieces&&begin>=spline.Station(piece+1))piece++;
                if(end>spline.Station(piece+1)+1e-8)
                {var mid=Math.Min(end,spline.Station(piece+1));return Fits(begin,mid,fitted,sign,depth,tolerance)&&Fits(mid,end,fitted,sign,depth,tolerance);}
                // Bound derivatives on the containing physical spline piece.
                // Cutting nearly coincident source/width stations and dividing
                // their rounded controls by a tiny span falsely creates a
                // stationary reference. Whole-piece hull bounds remain valid
                // on every proof subinterval without that cancellation.
                var c=spline.Curve(piece);var h=spline.Station(piece+1)-spline.Station(piece);
                var span=end-begin;var d0=3*(c.P1-c.P0).xz/h;var d1=3*(c.P2-c.P1).xz/h;var d2=3*(c.P3-c.P2).xz/h;
                var direction=math.normalizesafe(c.P3.xz-c.P0.xz);var minimum=Math.Min(math.dot(d0,direction),Math.Min(math.dot(d1,direction),math.dot(d2,direction)));
                if(!(minimum>1e-10)){fitFailure="reference-derivative";return false;}
                var acceleration=Math.Max(math.length(6*(c.P2-2*c.P1+c.P0).xz),math.length(6*(c.P3-2*c.P2+c.P1).xz))/(h*h);
                var jerk=math.length(6*(c.P3-3*c.P2+3*c.P1-c.P0).xz)/(h*h*h);
                var offset=Math.Max(Math.Max(Math.Abs(c.P0.y),Math.Abs(c.P1.y)),Math.Max(Math.Abs(c.P2.y),Math.Abs(c.P3.y)))+(sign==side?fitOffsetA:fitOffsetB);
                var offsetFirst=Math.Max(Math.Abs(3*(c.P1.y-c.P0.y)),Math.Max(Math.Abs(3*(c.P2.y-c.P1.y)),Math.Abs(3*(c.P3.y-c.P2.y))))/h;
                var offsetSecond=6*Math.Max(Math.Abs(c.P2.y-2*c.P1.y+c.P0.y),Math.Abs(c.P3.y-2*c.P2.y+c.P1.y))/(h*h);
                if(offset*acceleration/(minimum*minimum)>.5||Math.Min(Math.Min(c.P0.y,c.P1.y),Math.Min(c.P2.y,c.P3.y))<1.05-.001)
                {fitFailure="offset-regularity";offsetRegularity=offset*acceleration/(minimum*minimum);return false;}
                var correction=sign==side?correctionA:correctionB;
                var correctionBound=Math.Max(math.length(NetworkDesignChecks.SecondDerivative(correction,begin/station[^1]).xz),
                    math.length(NetworkDesignChecks.SecondDerivative(correction,end/station[^1]).xz))/(station[^1]*station[^1]);
                var bound=acceleration+offsetSecond+2*offsetFirst*acceleration/minimum+offset*(2*jerk/minimum+5*acceleration*acceleration/(minimum*minimum))+correctionBound;
                var fitSpan=fitEnd-fitBegin;var candidateBound=6*Math.Max(math.length((fitted.P2-2*fitted.P1+fitted.P0).xz),
                    math.length((fitted.P3-2*fitted.P2+fitted.P1).xz))/(fitSpan*fitSpan);
                if(Math.Max(ea,eb)+(bound+candidateBound)*span*span/8<=tolerance)return true;
                if(depth==16){fitFailure="proof-depth";return false;}var middle=(begin+end)/2;return Fits(begin,middle,fitted,sign,depth+1,tolerance)&&Fits(middle,end,fitted,sign,depth+1,tolerance);
            }
            void Emit(double begin,double end,int depth)
            {
                fitFailure="";
                fitWidthA=TemplateAt((begin+end)/2,true).NativeWidth;fitWidthB=TemplateAt((begin+end)/2,false).NativeWidth;
                fitOffsetA=Offset(TemplateAt((begin+end)/2,true),true);fitOffsetB=Offset(TemplateAt((begin+end)/2,false),false);
                token.ThrowIfCancellationRequested();var a0=Desired(begin,side);var a1=Desired(end,side);var b0=Desired(begin,-side);var b1=Desired(end,-side);
                Cubic3d Fit((double2 Point,double2 Derivative) a,(double2 Point,double2 Derivative) b)=>NetworkDesignChecks.Hermite(new(a.Point.x,0,a.Point.y),new(b.Point.x,0,b.Point.y),new(a.Derivative.x,0,a.Derivative.y),new(b.Derivative.x,0,b.Derivative.y),end-begin);
                var a=Fit(a0,a1);var b=Fit(b0,b1);fitBegin=begin;fitEnd=end;
                bool Inside(Cubic3d c)=>math.cmax(math.abs(c.P0.xz))<=width/2&&math.cmax(math.abs(c.P1.xz))<=width/2&&
                    math.cmax(math.abs(c.P2.xz))<=width/2&&math.cmax(math.abs(c.P3.xz))<=width/2;
                if(!Inside(a)||!Inside(b)){invalid=true;fallbackReason="map-boundary";return;}
                if(math.distance(((float3)a.P0).xz,((float3)a.P3).xz)<NetworkEmissionStations.MinimumReach(a)||
                    math.distance(((float3)b.P0).xz,((float3)b.P3).xz)<NetworkEmissionStations.MinimumReach(b))
                {invalid=true;fallbackReason="native-reach";log.Count("Paired emission intervals requiring representability refit");return;}
                double Rounding(Cubic3d c)=>new[]{c.P0,c.P1,c.P2,c.P3}.Max(p=>math.distance(p,(double3)(float3)p));
                if(Rounding(a)>.005||Rounding(b)>.005){invalid=true;fallbackReason="float-rounding";return;}
                var valid=end-begin<=400+1e-7&&Fits(begin,end,a,side,0)&&Fits(begin,end,b,-side,0)&&
                    // The reference includes a 0.1 m median reserve. Certify
                    // the actual emitted 2 m median independently of the
                    // 0.25 m internal spline approximation allowance.
                    Separated(a,b,RequiredAt((begin+end)/2)-.1,token);
                for(var j=0;j<=16;j++)
                {
                    var s=math.lerp(begin,end,j/16d);var pa=At(interval.A,Correspondence(s,true));var pb=At(interval.B,Correspondence(s,false));
                    if(math.distance(pa.Point,Desired(s,side).Point)>LocalAllowance(source[pa.Edge])||math.distance(pb.Point,Desired(s,-side).Point)>LocalAllowance(source[pb.Edge]))
                    {invalid=true;fallbackReason="source-reference-budget";budgetFailure=new{station=s,firstWay=source[pa.Edge].WayId,secondWay=source[pb.Edge].WayId,
                        firstDisplacement=math.distance(pa.Point,Desired(s,side).Point),secondDisplacement=math.distance(pb.Point,Desired(s,-side).Point),
                        firstAllowance=LocalAllowance(source[pa.Edge]),secondAllowance=LocalAllowance(source[pb.Edge])};return;} // Subdividing cannot repair a rejected reference model.
                }
                if(!valid&&depth<12&&end-begin>1){var middle=(begin+end)/2;Emit(begin,middle,depth+1);Emit(middle,end,depth+1);return;}
                if(!valid){invalid=true;fallbackReason="continuous-emission-"+(fitFailure.Length==0?"separation":fitFailure);return;}
                emitted.Add((begin,end,a,b));
            }
            for(var k=1;!invalid&&k<boundaries.Length;k++)Emit(boundaries[k-1],boundaries[k],0);
            // Certify source fidelity independently of native emission cuts.
            // Splitting at both correspondence/affine-source partitions makes
            // each source parameter map affine within a proof interval.
            var proofCuts=new SortedSet<double>(station);
            foreach(var (chain,a) in new[]{(interval.A,true),(interval.B,false)})foreach(var p in chain.Pieces)
            {var common=Common(p.Station,a);if(common>0&&common<station[^1])proofCuts.Add(common);}
            var proof=proofCuts.ToArray();
            for(var k=1;!invalid&&k<proof.Length;k++)
            {
                var begin=proof[k-1];var end=proof[k];
                Cubic3d SourceCurve(Chain chain,bool a)
                {
                    var edge=At(chain,Correspondence((begin+end)/2,a)).Edge;
                    var p=ParameterOnEdge(chain,edge,Correspondence(begin,a));var q=ParameterOnEdge(chain,edge,Correspondence(end,a));
                    var c=Subcurve(Curve(source[edge]),Math.Min(p,q),Math.Max(p,q));return p<=q?c:new(c.P3,c.P2,c.P1,c.P0);
                }
                var ca=SourceCurve(interval.A,true);var cb=SourceCurve(interval.B,false);
                var templateA=source[At(interval.A,Correspondence((begin+end)/2,true)).Edge];
                var templateB=source[At(interval.B,Correspondence((begin+end)/2,false)).Edge];
                var allowanceA=LocalAllowance(templateA);var allowanceB=LocalAllowance(templateB);fitBegin=begin;fitEnd=end;
                fitWidthA=TemplateAt((begin+end)/2,true).NativeWidth;fitWidthB=TemplateAt((begin+end)/2,false).NativeWidth;
                fitOffsetA=Offset(TemplateAt((begin+end)/2,true),true);fitOffsetB=Offset(TemplateAt((begin+end)/2,false),false);
                invalid=!Fits(begin,end,ca,side,0,allowanceA)||!Fits(begin,end,cb,-side,0,allowanceB);
                if(invalid)fallbackReason="continuous-source-fit";
                if(!invalid)
                {
                    if(!sourceFidelityFailed)sourceFidelityFailed=!Fits(begin,end,ca,side,0,templateA.SourceFitToleranceMeters??24)||
                        !Fits(begin,end,cb,-side,0,templateB.SourceFitToleranceMeters??24);
                }
            }
            for(var k=1;!invalid&&k<emitted.Count;k++)foreach(var isA in new[]{true,false})
            {
                var previous=emitted[k-1];var next=emitted[k];var chain=isA?interval.A:interval.B;
                var a=TemplateAt((previous.Begin+previous.End)/2,isA);var b=TemplateAt((next.Begin+next.End)/2,isA);
                if(a.NativeLaneOffsets.Count==0||b.NativeLaneOffsets.Count==0)continue;
                var ca=isA?previous.A:previous.B;var cb=isA?next.A:next.B;
                var da=math.normalizesafe((ca.P3-ca.P2).xz)*(isA?1:-1);var db=math.normalizesafe((cb.P1-cb.P0).xz)*(isA?1:-1);
                var pa=ca.P3.xz+new double2(da.y,-da.x)*a.NativeLaneOffsets.Min();
                var pb=cb.P0.xz+new double2(db.y,-db.x)*b.NativeLaneOffsets.Min();
                var normal=math.normalizesafe(new double2(da.y+db.y,-da.x-db.x));
                if(Math.Abs(math.dot(pa-pb,normal))>.05)
                {invalid=true;fallbackReason="continuing-lane-datum";log.Count("Paired candidates retaining source for continuing-lane mismatch");break;}
            }
            var profileOnly=invalid||emitted.Count==0;
            construction?.Add(new{firstWays=interval.A.Edges.SelectMany(i=>source[i].SourceWayIds.Append(source[i].WayId)).Distinct().ToArray(),
                secondWays=interval.B.Edges.SelectMany(i=>source[i].SourceWayIds.Append(source[i].WayId)).Distinct().ToArray(),
                firstBegin=sample[0].A,firstEnd=sample[^1].A,secondBegin=sample[0].B,secondEnd=sample[^1].B,
                lengthMeters=station[^1],fallbackReason,profileOnly,endpointError,offsetRegularity,
                budgetFailure,
                sourceFidelityFailed,
                coordinated,needsRepair,constructionEnvelopeMeters=coordinated?24:0,
                sections=interval.A.Edges.Concat(interval.B.Edges).Select(i=>new{source[i].WayId,source[i].NativeWidth,
                    startOffsetMeters=math.length(source[i].StartCurveOffset),endOffsetMeters=math.length(source[i].EndCurveOffset)}).ToArray(),
                firstBudget=interval.A.Edges.Min(i=>source[i].SourceFitToleranceMeters??24),secondBudget=interval.B.Edges.Min(i=>source[i].SourceFitToleranceMeters??24)});
            if(profileOnly)
            {
                log.Count("Paired profile-only: "+fallbackReason);
                if(profileOnlyIntervals<16)log.Diagnostic($"PAIR FALLBACK firstWay={source[interval.A.Edges[0]].WayId}; secondWay={source[interval.B.Edges[0]].WayId}; reason={fallbackReason}; endpointErrorMeters={endpointError:R}; offsetRegularityBound={offsetRegularity:R}; confirmedHeightRelationshipRetained=true; separationCertified=false");
                // Retain the confirmed relationship without asserting spacing.
                // Corresponding source subcurves share Y variables. Straight
                // courses may normalize their parameter speed without moving
                // their centerline locus or source endpoints.
                emitted.Clear();
                Cubic3d OriginalInterval(Chain chain,bool a,double begin,double end)
                {
                    var edge=At(chain,Correspondence((begin+end)/2,a)).Edge;
                    var p=ParameterOnEdge(chain,edge,Correspondence(begin,a));var q=ParameterOnEdge(chain,edge,Correspondence(end,a));
                    var c=Subcurve(Curve(source[edge]),Math.Min(p,q),Math.Max(p,q));return a?c:new(c.P3,c.P2,c.P1,c.P0);
                }
                bool Straight(Cubic3d c)
                {
                    var chord=c.P3.xz-c.P0.xz;var length=math.length(chord);if(!(length>1e-5))return false;var axis=chord/length;
                    var a=c.P1.xz-c.P0.xz;var b=c.P2.xz-c.P0.xz;
                    return Math.Abs(NetworkDesignChecks.Cross(a,axis))<=1e-8&&Math.Abs(NetworkDesignChecks.Cross(b,axis))<=1e-8&&
                        math.dot(a,axis)>=0&&math.dot(b,axis)>=math.dot(a,axis)&&math.dot(b,axis)<=length;
                }
                for(var k=1;k<boundaries.Length;k++)
                {
                    var a=OriginalInterval(interval.A,true,boundaries[k-1],boundaries[k]);var b=OriginalInterval(interval.B,false,boundaries[k-1],boundaries[k]);
                    if(Straight(a)&&Straight(b))
                    {
                        var reach=Math.Min(math.distance(a.P0.xz,a.P3.xz),math.distance(b.P0.xz,b.P3.xz))/3;
                        Cubic3d Normalize(Cubic3d c)
                        {var axis=math.normalizesafe(c.P3.xz-c.P0.xz);return c with{
                            P1=c.P1 with{x=c.P0.x+axis.x*reach,z=c.P0.z+axis.y*reach},
                            P2=c.P2 with{x=c.P3.x-axis.x*reach,z=c.P3.z-axis.y*reach}};}
                        a=Normalize(a);b=Normalize(b);straightMetrics++;log.Count("Profile-only straight station metrics normalized");
                    }
                    emitted.Add((boundaries[k-1],boundaries[k],a,b));
                }
            }
            // Do not claim overlapping intervals twice when a chain's partner
            // changes. The matcher ends intervals at ambiguity, not a sort tie.
            var additions=new Dictionary<int,List<(double Begin,double End,PreparedSegment Segment)>>();var localMoves=new Dictionary<long,float3>();var localDirections=new Dictionary<long,double2>();
            var localAnchors=new Dictionary<long,float3>();
            float3 Anchor(long node,float3 course,float3 offset)
            {
                if(localAnchors.TryGetValue(node,out var existing))return existing;
                var anchor=publishedAnchors.TryGetValue(node,out existing)?existing:course-offset;
                localAnchors[node]=anchor;return anchor;
            }
            var portsA=new Dictionary<double,long>();var portsB=new Dictionary<double,long>();var families=new Dictionary<double,long>();var corridor=++corridorId;
            foreach(var e in emitted)
            {
                var span=++spanId;
                if(designed!=null)designed[span]=(e.A,e.B,TemplateAt((e.Begin+e.End)/2,true).NativeWidth,TemplateAt((e.Begin+e.End)/2,false).NativeWidth);
                foreach(var isA in new[]{true,false})
                {
                    var chain=isA?interval.A:interval.B;var lo=At(chain,Correspondence(e.Begin,isA));var hi=At(chain,Correspondence(e.End,isA));
                    var middle=At(chain,Correspondence((e.Begin+e.End)/2,isA));var template=TemplateAt((e.Begin+e.End)/2,isA);
                    var start=isA?lo:hi;var end=isA?hi:lo;
                    Projection OnEdge(Projection point)
                    {
                        var t=ParameterOnEdge(chain,middle.Edge,point.Station);
                        return point with{Edge=middle.Edge,Parameter=t};
                    }
                    start=OnEdge(start);end=OnEdge(end);
                    var curve=isA?e.A:new Cubic3d(e.B.P3,e.B.P2,e.B.P1,e.B.P0);
                    var ports=isA?portsA:portsB;
                    long Port(double s,Projection p)
                    {if(!ports.TryGetValue(s,out var id))ports[s]=id=SourceCut(p);return id;}
                    long Family(double s){if(!families.TryGetValue(s,out var id))families[s]=id=++familyId;return id;}
                    var startStation=isA?e.Begin:e.End;var endStation=isA?e.End:e.Begin;
                    var startNode=Port(startStation,start);var endNode=Port(endStation,end);
                    var y0=NetworkDesignChecks.Evaluate(Curve(source[start.Edge]),start.Parameter).y;var y1=NetworkDesignChecks.Evaluate(Curve(source[end.Edge]),end.Parameter).y;
                    curve=curve with{P0=curve.P0 with{y=y0},P1=curve.P1 with{y=math.lerp(y0,y1,1d/3)},P2=curve.P2 with{y=math.lerp(y0,y1,2d/3)},P3=curve.P3 with{y=y1}};
                    var p0=(float3)curve.P0;var p1=(float3)curve.P1;var p2=(float3)curve.P2;var p3=(float3)curve.P3;
                    var startOffset=math.lerp(template.StartCurveOffset,template.EndCurveOffset,(float)start.Parameter);
                    var endOffset=math.lerp(template.StartCurveOffset,template.EndCurveOffset,(float)end.Parameter);
                    // One node anchor, separate native course offsets at a
                    // width event. Welding different centre courses would
                    // destroy the continuing-lane datum and median clearance.
                    var startAnchor=Anchor(startNode,p0,startOffset);var endAnchor=Anchor(endNode,p3,endOffset);
                    startOffset=p0-startAnchor;endOffset=p3-endAnchor;
                    var segment=template with{StartNodeId=startNode,EndNodeId=endNode,Start=p0-startOffset,End=p3-endOffset,Control1=p1-startOffset,Control2=p2-endOffset,FinalDeckCurve=new(p0,p1,p2,p3),HorizontalParent=null,HorizontalEmissionReference=curve,
                        StartCurveOffset=startOffset,EndCurveOffset=endOffset,
                        PairedCorridorId=corridor,PairedSpanId=span,PairedReversed=!isA,StartHeightFamily=Family(startStation),EndHeightFamily=Family(endStation),ReferenceSpan=e.End-e.Begin,
                        PairedProfileId=span,PairedProfileOnly=profileOnly,PairedReferenceBegin=startStation,PairedReferenceEnd=endStation,
                        StartReferenceSpeed=math.length((curve.P1-curve.P0).xz)*3/(e.End-e.Begin),EndReferenceSpeed=math.length((curve.P3-curve.P2).xz)*3/(e.End-e.Begin),
                        StartTerminal=start.Parameter<1e-7&&source[start.Edge].StartTerminal,EndTerminal=end.Parameter>1-1e-7&&source[end.Edge].EndTerminal};
                    if(!additions.TryGetValue(middle.Edge,out var list))additions[middle.Edge]=list=new();
                    list.Add((start.Parameter,end.Parameter,segment));
                    foreach(var port in new[]{(Node:startNode,Anchor:startAnchor,Projection:start),(Node:endNode,Anchor:endAnchor,Projection:end)})
                    {
                        if(!originalAnchors.TryGetValue(port.Node,out var original))
                        {
                            // A generated cut also joins an independent source
                            // remainder. Publish its displacement, otherwise
                            // that remainder reinstates the old overlapping pose.
                            var previous=source[port.Projection.Edge];var t=(float)port.Projection.Parameter;
                            original=(float3)NetworkDesignChecks.Evaluate(Curve(previous),port.Projection.Parameter)-
                                math.lerp(previous.StartCurveOffset,previous.EndCurveOffset,t);
                        }
                        localMoves[port.Node]=port.Anchor-original;
                    }
                    localDirections[startNode]=math.normalizesafe((double2)(p1-p0).xz);localDirections[endNode]=math.normalizesafe((double2)(p3-p2).xz);
                }
            }
            if(additions.Any(p=>replaced.TryGetValue(p.Key,out var old)&&p.Value.Any(a=>old.Any(b=>a.Begin<b.End-1e-7&&a.End>b.Begin+1e-7)))){rejected++;continue;}
            // A correspondence boundary is an optional approximation cut, not
            // permission to leave a millimetre source remainder. Such tails
            // amplify grade-difference coefficients by inverse arc length
            // squared and add native nodes without useful geometry. Reject the
            // whole proposed pair; never delete or weld the original endpoint.
            var unrepresentable=additions.Any(p=>
            {
                var original=Curve(source[p.Key]);
                return p.Value.Any(part=>
                    part.Begin>1e-7&&part.Begin<1-1e-7&&!NetworkEmissionStations.CanSplit(original,part.Begin)||
                    part.End>1e-7&&part.End<1-1e-7&&!NetworkEmissionStations.CanSplit(original,part.End)||
                    math.distance(part.Segment.CourseStart.xz,part.Segment.CourseEnd.xz)<NetworkEmissionStations.MinimumReach(original));
            });
            if(unrepresentable)
            {rejected++;log.Count("Paired candidates rejected before sub-grid emission");continue;}
            foreach(var p in additions){if(!replaced.TryGetValue(p.Key,out var list))replaced[p.Key]=list=new();list.AddRange(p.Value);}
            foreach(var move in localMoves)movements[move.Key]=move.Value;accepted++;if(profileOnly)profileOnlyIntervals++;
            if(!profileOnly&&sourceFidelityFailed)sourceReview++;
            foreach(var direction in localDirections)directions[direction.Key]=direction.Value;
            foreach(var anchor in localAnchors)publishedAnchors[anchor.Key]=anchor.Value;
        }
        var output=new List<PreparedSegment>();
        for(var i=0;i<source.Count;i++)
        {
            var s=source[i];var cursor=0d;
            void Original(double begin,double end)
            {
                if(end-begin<1e-7)return;var c=Subcurve(Curve(s),begin,end);var a=begin==0?s.StartNodeId:NewNode();var b=end==1?s.EndNodeId:NewNode();
                if(replaced.TryGetValue(i,out var pieces))foreach(var piece in pieces)
                {if(Math.Abs(piece.End-begin)<1e-7)a=piece.Segment.EndNodeId;if(Math.Abs(piece.Begin-end)<1e-7)b=piece.Segment.StartNodeId;}
                var da=movements.TryGetValue(a,out var x)?x:float3.zero;var db=movements.TryGetValue(b,out var y)?y:float3.zero;
                if(begin==0&&incident.TryGetValue(a,out var ai)&&ai.Count==1&&math.lengthsq(da)==0)da=db;
                if(end==1&&incident.TryGetValue(b,out var bi)&&bi.Count==1&&math.lengthsq(db)==0)db=da;
                var startOffset=math.lerp(s.StartCurveOffset,s.EndCurveOffset,(float)begin);
                var endOffset=math.lerp(s.StartCurveOffset,s.EndCurveOffset,(float)end);
                var p0=(float3)c.P0+da;var p3=(float3)c.P3+db;var p1=(float3)c.P1+da;var p2=(float3)c.P2+db;
                // The paired owner also publishes the boundary tangent to its
                // independent remainder. Do not leave a second owner to repair
                // that join after the profile has already been solved.
                // The shared node position belongs to every attached arm,
                // but the paired mainline tangent does not. In particular a
                // link's selected merge angle must not be reset to zero here.
                if(Eligible(s)&&directions.TryGetValue(a,out var startDirection)){var reach=math.distance(c.P0.xz,c.P1.xz);p1.xz=p0.xz+(float2)(startDirection*reach);}
                if(Eligible(s)&&directions.TryGetValue(b,out var endDirection)){var reach=math.distance(c.P2.xz,c.P3.xz);p2.xz=p3.xz-(float2)(endDirection*reach);}
                output.Add(s with{StartNodeId=a,EndNodeId=b,Start=p0-startOffset,End=p3-endOffset,Control1=p1-startOffset,Control2=p2-endOffset,
                    StartCurveOffset=startOffset,EndCurveOffset=endOffset,FinalDeckCurve=null,HorizontalParent=null,HorizontalEmissionReference=null,StartTerminal=begin==0&&s.StartTerminal,EndTerminal=end==1&&s.EndTerminal});
            }
            if(s.Kind!=NetworkKind.Road||!replaced.ContainsKey(i)&&!movements.ContainsKey(s.StartNodeId)&&!movements.ContainsKey(s.EndNodeId)){output.Add(s);continue;}
            if(replaced.TryGetValue(i,out var pieces))foreach(var piece in pieces.OrderBy(p=>p.Begin))
            {Original(cursor,piece.Begin);output.Add(piece.Segment);cursor=piece.End;}
            Original(cursor,1);
        }
        inspectPublication?.Invoke("pre-canonical",output);
        // Multiple confirmed intervals can meet one exact source port. Publish
        // one canonical anchor for that port, including its attached remainder,
        // rather than leaving interval-local floating point poses in conflict.
        var anchors=new Dictionary<long,float3>();
        foreach(var s in source.Where(s=>s.Kind==NetworkKind.Road))foreach(var port in new[]{(s.StartNodeId,s.Start),(s.EndNodeId,s.End)})
            if(!anchors.ContainsKey(port.Item1))anchors[port.Item1]=port.Item2+(movements.TryGetValue(port.Item1,out var delta)?delta:default);
        // The paired owner selected these anchors before publication. An
        // untouched remainder must consume them, not become a second writer.
        foreach(var anchor in publishedAnchors)anchors[anchor.Key]=anchor.Value;
        foreach(var s in output.Where(s=>s.Kind==NetworkKind.Road))foreach(var port in new[]{(s.StartNodeId,s.Start),(s.EndNodeId,s.End)})
            if(!anchors.ContainsKey(port.Item1))anchors[port.Item1]=port.Item2;
        var revisedProfiles=new HashSet<long>();var publications=0;
        for(var i=0;i<output.Count;i++)
        {
            var s=output[i];if(s.Kind!=NetworkKind.Road)continue;
            var a=anchors.TryGetValue(s.StartNodeId,out var start)?start:s.Start;
            var b=anchors.TryGetValue(s.EndNodeId,out var end)?end:s.End;
            if(math.all(a==s.Start)&&math.all(b==s.End))continue;
            var da=a-s.Start;var db=b-s.End;var c=new Cubic3f(s.CourseStart+da,s.CourseControl1+da,s.CourseControl2+db,s.CourseEnd+db);
            if(s.PairedProfileId!=0&&construction!=null)
            {
                double[] P(float3 p)=>new[]{(double)p.x,p.z};
                foreach(var port in new[]{(Node:s.StartNodeId,Before:s.Start,After:a),(Node:s.EndNodeId,Before:s.End,After:b)})
                    if(!math.all(port.Before==port.After))construction.Add(new{stage="anchor-publication",profile=s.PairedProfileId,s.WayId,port.Node,
                        before=P(port.Before),after=P(port.After),published=publishedAnchors.TryGetValue(port.Node,out var p)?P(p):null,
                        original=originalAnchors.TryGetValue(port.Node,out var o)?P(o):null,movement=movements.TryGetValue(port.Node,out var m)?P(m):null});
            }
            output[i]=s with{Start=a,End=b,Control1=s.CourseControl1+da-s.StartCurveOffset,Control2=s.CourseControl2+db-s.EndCurveOffset,
                FinalDeckCurve=c,HorizontalEmissionReference=null};publications++;
            if(s.PairedProfileId!=0)revisedProfiles.Add(s.PairedProfileId);
        }
        inspectPublication?.Invoke("post-canonical-precompaction",output);
        // Certify the published float curves, not a pre-anchor double model.
        // Canonical publication may either preserve or invalidate spacing.
        var certified=new HashSet<long>();var protectionChecked=0;var protectionReview=0;
        output=PairedEmissionCompaction.Prepare(output,source,log,token,out var compactedProfiles,width);
        inspectPublication?.Invoke("postcompaction",output);
        foreach(var group in output.Where(s=>s.PairedProfileId!=0).GroupBy(s=>s.PairedProfileId))
        {
            var pair=group.ToArray();if(pair.Length!=2)continue;var a=Curve(pair[0]);var b=Curve(pair[1]);
            if(pair[0].PairedReversed!=pair[1].PairedReversed)b=new(b.P3,b.P2,b.P1,b.P0);
            if(Separated(a,b,(pair[0].NativeWidth+pair[1].NativeWidth)/2+2,token))certified.Add(group.Key);
            else if(designed!=null&&designed.TryGetValue(group.Key,out var before))
            {
                double[] Points(Cubic3d c)=>new[]{c.P0.x,c.P0.z,c.P1.x,c.P1.z,c.P2.x,c.P2.z,c.P3.x,c.P3.z};
                var forward=pair.Single(s=>!s.PairedReversed);var reverse=pair.Single(s=>s.PairedReversed);
                construction?.Add(new{stage="published-spacing-failure",profile=group.Key,firstWays=forward.SourceWayIds.Append(forward.WayId).Distinct().ToArray(),
                    secondWays=reverse.SourceWayIds.Append(reverse.WayId).Distinct().ToArray(),
                    plannedWidthA=before.WidthA,plannedWidthB=before.WidthB,publishedWidthA=forward.NativeWidth,publishedWidthB=reverse.NativeWidth,
                    plannedA=Points(before.A),plannedB=Points(before.B),publishedA=Points(Curve(forward)),publishedB=Points(new(reverse.CourseEnd,reverse.CourseControl2,reverse.CourseControl1,reverse.CourseStart)),
                    anchorRevised=revisedProfiles.Contains(group.Key)});
            }
            if(protection!=null)
            {protectionChecked++;if(!protection.Clear(a,pair[0].NativeWidth/2)||!protection.Clear(b,pair[1].NativeWidth/2))protectionReview++;}
        }
        // Source coverage still refers to the pre-coalescence intervals. Each
        // removed profile inherits only its final owner's actual certificate.
        foreach(var profile in compactedProfiles)if(certified.Contains(profile.Value))certified.Add(profile.Key);
        for(var i=0;i<output.Count;i++)if(output[i].PairedProfileId!=0)output[i]=output[i] with{PairedProfileOnly=!certified.Contains(output[i].PairedProfileId)};
        profileOnlyIntervals=output.Where(s=>s.PairedProfileOnly&&s.PairedCorridorId!=0).Select(s=>s.PairedCorridorId).Distinct().Count();
        if(publications>0)log.Diagnostic($"PAIR PORT PUBLICATION revisedSections={publications}; uncertifiedProfileSpans={revisedProfiles.Count(id=>!certified.Contains(id))}; exactSourceIdentityOnly=true; finalFloatSeparationRecertified=true; nativeOwnershipUnchanged=true");
        log.Count("Trusted paired intervals",accepted);log.Count("Paired intervals requiring independent fit",rejected);
        if(construction!=null)log.DiagnosticRecordChunks("paired-construction",construction);
        log.Quality(new("planned","paired-source-fidelity","formations",accepted,sourceReview,0,0,0,
            "Original source tolerance retained independently; bounded generated formation repair is not geographic certification"));
        log.Quality(new("planned","paired-protected-footprint","published-spans",protectionChecked,protectionReview,protection==null?output.Count(s=>s.PairedProfileId!=0)/2:0,0,0,
            "Actual native-width footprint plus 12 m against immutable protected geography; failures remain RequiresReview and do not waive terrain protection"));
        log.Count("Confirmed paired intervals without certified separation",profileOnlyIntervals);
        log.Diagnostic($"PAIRED PROFILE METRICS straightSpansNormalized={straightMetrics}; endpointsPreserved=true; centerlineLocusDeviationBoundMeters=1e-8; curvedSourceRetained=true; separationCertified=false");
        log.Quality(new("planned","paired-separation","formations",accepted,profileOnlyIntervals,rejected,profileOnlyIntervals,0,"Shared profiles do not imply certified horizontal separation; rejected emissions remain visible"));
        log.AddCounter("Paired continuous projection candidates",projectionCandidates);
        log.AddCounter("Paired continuous projections excluded by hull bounds",projectionBoundsRejected);
        var candidateEdges=new HashSet<int>(intervals.SelectMany(p=>p.A.Edges.Concat(p.B.Edges)));
        var eligibleLength=0d;var pairedLength=0d;var independentLength=0d;var certifiedLength=0d;var unknownLength=0d;var coverage=log.DiagnosticsEnabled?new List<object>():null;
        for(var i=0;i<source.Count;i++)
        {
            if(!Eligible(source[i]))continue;token.ThrowIfCancellationRequested();var curve=Curve(source[i]);
            var length=NetworkDesignChecks.HorizontalArcLength(curve);var matched=0d;var uncertified=0d;
            if(replaced.TryGetValue(i,out var ranges))foreach(var range in ranges)
            {
                var span=NetworkDesignChecks.HorizontalArcLength(curve,range.Begin,range.End);matched+=span;
                if(!certified.Contains(range.Segment.PairedProfileId))uncertified+=span;
            }
            matched=Math.Min(length,matched);var independent=Math.Max(0,length-matched);
            eligibleLength+=length;pairedLength+=matched;independentLength+=independent;
            certifiedLength+=Math.Max(0,matched-uncertified);if(candidateEdges.Contains(i))unknownLength+=independent;
            coverage?.Add(new{section=i,source[i].WayId,source[i].StartNodeId,source[i].EndNodeId,eligibleLengthMeters=length,
                pairedSourceLengthMeters=matched,independentSourceLengthMeters=independent,
                confirmedPairLengthMeters=matched,sharedProfileLengthMeters=matched,unknownCandidateLengthMeters=candidateEdges.Contains(i)?independent:0,
                certifiedSeparationSourceLengthMeters=Math.Max(0,matched-uncertified),uncertifiedSeparationSourceLengthMeters=uncertified,
                reason=uncertified>0?"confirmed-shared-profile-separation-requires-review":independent<.01?"fully-certified-paired-interval":matched>0?"independent-boundary-or-unmatched-interval":
                    candidateEdges.Contains(i)?"reciprocal-candidate-not-certified":"no-reciprocal-candidate",
                unmatchedDoesNotProveMissingRequiredPartner=true});
        }
        if(coverage!=null)log.DiagnosticCompressedRecords("paired-correspondence-coverage.ndjson.gz",coverage);
        log.Diagnostic($"PAIR COVERAGE eligibleSourceLengthMeters={eligibleLength:R}; confirmedPairLengthMeters={pairedLength:R}; sharedProfileLengthMeters={pairedLength:R}; certifiedSeparationLengthMeters={certifiedLength:R}; uncertifiedSeparationLengthMeters={pairedLength-certifiedLength:R}; unknownCandidateLengthMeters={unknownLength:R}; independentSourceLengthMeters={independentLength:R}; bothCarriagewaysCounted=true; sourceStationsNotSubdivisionCounts=true; unmatchedIsNotAutomaticallyAnError=true");
        log.Diagnostic($"TRUSTED PAIRS chains={chains.Count}; candidateIntervals={intervals.Count}; accepted={accepted}; independentFallbacks={rejected}; nonreciprocalOrAmbiguousStations={ambiguous}; sharedHeightSpans={spanId}; projectionCandidates={projectionCandidates}; projectionBoundsRejected={projectionBoundsRejected}; sourceBased=true; radiusMeters=96; minimumOverlapMeters=80; continuousProjection=true; synchronizedEmission=true; qualityFailureBlocksPlacement=false");
        return output;
    }
    private static Cubic3d Subcurve(Cubic3d c,double begin,double end)
    {if(end<1)c=NetworkDesignChecks.Split(c,end).Left;if(begin>0)c=NetworkDesignChecks.Split(c,begin/end).Right;return c;}

    private static bool TryProject(Cubic3d curve,double2 point,double begin,double end,double seed,out double parameter)
    {
        var bestT=seed;var best=math.distance(NetworkDesignChecks.Evaluate(curve,seed).xz,point);
        void Candidate(double t)
        {var distance=math.distance(NetworkDesignChecks.Evaluate(curve,t).xz,point);if(distance<best){best=distance;bestT=t;}}
        Candidate(begin);Candidate(end);
        // Newton supplies a cheap upper bound, not a proof of the nearest root.
        var t=seed;
        for(var i=0;i<6;i++)
        {
            var q=NetworkDesignChecks.Evaluate(curve,t).xz;var d=NetworkDesignChecks.Derivative(curve,t).xz;var a=NetworkDesignChecks.SecondDerivative(curve,t).xz;
            var denominator=math.dot(d,d)+math.dot(q-point,a);if(!(denominator>1e-12))break;
            t=math.clamp(t-math.dot(q-point,d)/denominator,begin,end);Candidate(t);
        }
        var visits=0;var complete=true;
        void Bound(Cubic3d c,double first,double last,int depth)
        {
            if(++visits>4096){complete=false;return;}
            var lo=math.min(math.min(c.P0.xz,c.P1.xz),math.min(c.P2.xz,c.P3.xz));var hi=math.max(math.max(c.P0.xz,c.P1.xz),math.max(c.P2.xz,c.P3.xz));
            var lower=math.distance(point,math.clamp(point,lo,hi));if(best-lower<=.0005)return;
            var middle=(first+last)/2;Candidate(middle);
            if(depth==20){complete=false;return;}
            var pair=NetworkDesignChecks.Split(c,.5);Bound(pair.Left,first,middle,depth+1);Bound(pair.Right,middle,last,depth+1);
        }
        Bound(Subcurve(curve,begin,end),begin,end,0);parameter=bestT;return complete;
    }
}
