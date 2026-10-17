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
    private sealed class Chain
    {
        public int Id;public readonly List<int> Edges=new();public readonly List<Piece> Pieces=new();
        public readonly Dictionary<int,(int First,int Last)> Ranges=new();
        public double Length,StartCore,EndCore;
    }
    private sealed record Piece(int Chain,int Edge,double Begin,double End,double Station,double Length,double2 A,double2 B);
    private readonly record struct Projection(int Chain,int Edge,double Parameter,double Station,double2 Point,double2 Heading,double Distance);
    private readonly record struct Match(double A,double B,double2 PA,double2 PB,int Side);
    private sealed record Interval(Chain A,Chain B,Match[] Samples);
    private static bool Eligible(PreparedSegment s)=>s.Kind==NetworkKind.Road&&s.OneWay&&!s.ProtectedLink&&!s.Roundabout&&
        s.Classification is "motorway" or "trunk"&&s.NativeWidth>0&&math.isfinite(s.NativeWidth)&&
        !s.SourceTags.ContainsKey("reversible")&&(!s.SourceTags.TryGetValue("oneway",out var way)||way!="reversible");
    private static bool Related(PreparedSegment a,PreparedSegment b)=>a.Tunnel==b.Tunnel&&
        (a.Layer==b.Layer||!a.RawSourceTags.ContainsKey("layer")&&!b.RawSourceTags.ContainsKey("layer")&&a.Bridge!=b.Bridge)&&a.Classification==b.Classification&&
        (a.CorridorId!=0&&a.CorridorId==b.CorridorId||a.Reference.Length>0&&a.Reference==b.Reference||a.Name.Length>0&&a.Name==b.Name);
    private static Cubic3d Curve(PreparedSegment s)=>new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);

    public static IReadOnlyList<PreparedSegment> Prepare(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token,Action<string>? progress=null)
    {
        var incident=new Dictionary<long,List<int>>();
        for(var i=0;i<source.Count;i++)if(source[i].Kind==NetworkKind.Road)foreach(var id in new[]{source[i].StartNodeId,source[i].EndNodeId})
        {if(!incident.TryGetValue(id,out var list))incident[id]=list=new();list.Add(i);}
        var chains=new List<Chain>();var visited=new bool[source.Count];
        var requiredAliases=new HashSet<long>(source.Where(s=>s.Kind==NetworkKind.Road).SelectMany(s=>s.SourceNodeAliases).Where(a=>a.Mandatory).Select(a=>a.CanonicalNodeId));
        // Correspondence survives an abutment or lane-width boundary. Emission
        // cuts at every source edge below retain each side's actual asset/mode.
        bool Continue(int a,int b)=>Eligible(source[b])&&(source[a].WayId==source[b].WayId||Related(source[a],source[b]))&&
            source[a].Tunnel==source[b].Tunnel;
        int Next(int edge,bool backwards)
        {
            var s=source[edge];var node=backwards?s.StartNodeId:s.EndNodeId;
            if(incident[node].Count!=2||requiredAliases.Contains(node))return -1;var next=incident[node][0]==edge?incident[node][1]:incident[node][0];
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
                if(NetworkDesignChecks.Finite(c))Flatten(c,0,1,0);
                void Flatten(Cubic3d part,double begin,double end,int depth)
                {
                    token.ThrowIfCancellationRequested();var a=part.P0.xz;var b=part.P3.xz;
                    var error=Math.Max(math.distance(part.P1.xz,(2*a+b)/3),math.distance(part.P2.xz,(a+2*b)/3));
                    if((math.distance(a,b)>10||error>.01)&&depth<18)
                    {var split=NetworkDesignChecks.Split(part,.5);var middle=(begin+end)/2;Flatten(split.Left,begin,middle,depth+1);Flatten(split.Right,middle,end,depth+1);return;}
                    var length=math.distance(a,b);if(length<=1e-8)return;
                    chain.Pieces.Add(new(chain.Id,index,begin,end,chain.Length,length,a,b));chain.Length+=length;
                }
                if(chain.Pieces.Count>firstPiece)chain.Ranges[index]=(firstPiece,chain.Pieces.Count-1);
                var next=Next(index,false);if(next<0||visited[next])break;index=next;
            }
            var head=source[chain.Edges[0]];var tail=source[chain.Edges[^1]];
            // Only real junction/semantic boundaries own exclusion cores. A
            // legitimate external terminal does not hide a complete straight pair.
            chain.StartCore=incident[head.StartNodeId].Count>1?2*head.NativeWidth:0;
            chain.EndCore=incident[tail.EndNodeId].Count>1?2*tail.NativeWidth:0;chains.Add(chain);
        }
        var grid=new Dictionary<(int,int),List<Piece>>();
        foreach(var chain in chains)foreach(var p in chain.Pieces)
        {
            var min=math.min(p.A,p.B);var max=math.max(p.A,p.B);
            for(var z=(int)Math.Floor(min.y/64);z<=(int)Math.Floor(max.y/64);z++)for(var x=(int)Math.Floor(min.x/64);x<=(int)Math.Floor(max.x/64);x++)
            {if(!grid.TryGetValue((x,z),out var list))grid[(x,z)]=list=new();list.Add(p);}
        }
        bool Interior(Projection p)=>p.Station>=chains[p.Chain].StartCore&&p.Station<=chains[p.Chain].Length-chains[p.Chain].EndCore;
        Projection? Closest(int own,double2 point,double2 heading)
        {
            var candidates=new Dictionary<int,Projection>();var gx=(int)Math.Floor(point.x/64);var gz=(int)Math.Floor(point.y/64);
            var reference=source[chains[own].Edges[0]];
            for(var z=gz-2;z<=gz+2;z++)for(var x=gx-2;x<=gx+2;x++)if(grid.TryGetValue((x,z),out var pieces))foreach(var p in pieces)
            {
                if(p.Chain==own||!Related(reference,source[p.Edge]))continue;
                var u=math.clamp(math.dot(point-p.A,p.B-p.A)/math.lengthsq(p.B-p.A),0,1);var t=p.Begin+(p.End-p.Begin)*u;
                var c=Curve(source[p.Edge]);
                if(!TryProject(c,point,p.Begin,p.End,t,out t))continue;
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
            var stations=new SortedSet<double>(chain.Pieces.Select(p=>p.Station)){chain.Length};
            foreach(var station in stations)
            {
                token.ThrowIfCancellationRequested();var a=At(chain,station);
                if(!Interior(a)){Flush();continue;}
                var b=Closest(chain.Id,a.Point,a.Heading);
                if(!b.HasValue||b.Value.Chain<chain.Id){Flush();continue;}
                var back=Closest(b.Value.Chain,b.Value.Point,b.Value.Heading);
                if(!back.HasValue||back.Value.Chain!=chain.Id||math.distance(back.Value.Point,a.Point)>.1){ambiguous++;Flush();continue;}
                var normal=new double2(-a.Heading.y,a.Heading.x);var sign=math.dot(a.Point-b.Value.Point,normal)<0?-1:1;
                if(partner!=b.Value.Chain||samples.Count>0&&(b.Value.Station>=samples[^1].B-1e-6||samples[^1].Side!=sign))Flush();
                partner=b.Value.Chain;samples.Add(new(station,b.Value.Station,a.Point,b.Value.Point,sign));
            }
            Flush();
        }
        var occupied=new HashSet<long>(source.SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}));var nextNode=long.MinValue+30000000;
        long NewNode(){while(occupied.Contains(nextNode))nextNode++;var node=nextNode++;occupied.Add(node);return node;}
        long familyId=0,spanId=0,corridorId=0;var replaced=new Dictionary<int,List<(double Begin,double End,PreparedSegment Segment)>>();
        var movements=new Dictionary<long,float3>();var directions=new Dictionary<long,double2>();var rejected=0;var accepted=0;
        foreach(var interval in intervals)
        {
            progress?.Invoke($"Fitting trusted paired corridor {accepted+rejected+1}/{intervals.Count}");
            token.ThrowIfCancellationRequested();var sample=interval.Samples;var station=new List<double>{0};var points=new List<double3>();
            var required=(interval.A.Edges.Max(i=>source[i].NativeWidth)+interval.B.Edges.Max(i=>source[i].NativeWidth))/2+2.1;
            for(var i=0;i<sample.Length;i++)
            {
                var mid=(sample[i].PA+sample[i].PB)/2;
                if(i>0){var delta=math.distance(mid,points[^1].xz);if(delta<=1e-6)continue;station.Add(station[^1]+delta);}
                var localRequired=(source[At(interval.A,sample[i].A).Edge].NativeWidth+source[At(interval.B,sample[i].B).Edge].NativeWidth)/2+2.1;
                points.Add(new(mid.x,Math.Max(math.distance(sample[i].PA,sample[i].PB),localRequired)/2,mid.y));
            }
            if(points.Count!=sample.Length||station[^1]<80){rejected++;continue;}
            var va=math.normalizesafe(points[1].xz-points[0].xz);var vb=math.normalizesafe(points[^1].xz-points[^2].xz);
            var spline=new NetworkReferenceSpline(station,points,new(va.x,0,va.y),new(vb.x,0,vb.y));var side=sample[0].Side;
            (double2 Point,double2 Derivative) RawDesired(double s,int sign)
            {
                var value=spline.Evaluate(s);var v=value.Derivative.xz;var speed=math.length(v);
                if(!(speed>1e-10))return(new(double.NaN),new(double.NaN));
                // The spline second derivative is exact from its cubic controls.
                var piece=0;while(piece+1<spline.Pieces&&s>spline.Station(piece+1))piece++;
                var c=spline.Curve(piece);var reach=spline.Station(piece+1)-spline.Station(piece);
                var acceleration=NetworkDesignChecks.SecondDerivative(c,(s-spline.Station(piece))/reach).xz/(reach*reach);
                var tangent=v/speed;var normal=new double2(-tangent.y,tangent.x);
                var dn=new double2(-(acceleration-tangent*math.dot(tangent,acceleration)).y,(acceleration-tangent*math.dot(tangent,acceleration)).x)/speed;
                return(value.Point.xz+sign*value.Point.y*normal,v+sign*(value.Derivative.y*normal+value.Point.y*dn));
            }
            // Pairing consumes already selected junction/lane frames. Match
            // both boundary poses inside this same reference model, rather
            // than moving a terminal and asking another writer to repair it.
            Cubic3d BoundaryCorrection(bool isA)
            {
                var chain=isA?interval.A:interval.B;var sign=isA?side:-side;
                var first=At(chain,isA?sample[0].A:sample[0].B);var last=At(chain,isA?sample[^1].A:sample[^1].B);
                var start=RawDesired(0,sign);var end=RawDesired(station[^1],sign);
                var orientation=isA?1:-1;
                var startDerivative=first.Heading*orientation*math.length(start.Derivative);
                var endDerivative=last.Heading*orientation*math.length(end.Derivative);
                var lockStart=isA?chain.StartCore>0:chain.EndCore>0;
                var lockEnd=isA?chain.EndCore>0:chain.StartCore>0;
                double3 V(double2 p)=>new(p.x,0,p.y);
                return NetworkDesignChecks.Hermite(lockStart?V(first.Point-start.Point):default,lockEnd?V(last.Point-end.Point):default,
                    lockStart?V(startDerivative-start.Derivative):default,lockEnd?V(endDerivative-end.Derivative):default,station[^1]);
            }
            var correctionA=BoundaryCorrection(true);var correctionB=BoundaryCorrection(false);
            (double2 Point,double2 Derivative) Desired(double s,int sign)
            {
                var raw=RawDesired(s,sign);var correction=sign==side?correctionA:correctionB;
                var t=s/station[^1];
                return(raw.Point+NetworkDesignChecks.Evaluate(correction,t).xz,
                    raw.Derivative+NetworkDesignChecks.Derivative(correction,t).xz/station[^1]);
            }
            double Correspondence(double s,bool a)
            {
                var k=station.BinarySearch(s);if(k<0)k=~k-1;k=Math.Max(0,Math.Min(sample.Length-2,k));var t=(s-station[k])/(station[k+1]-station[k]);
                return math.lerp(a?sample[k].A:sample[k].B,a?sample[k+1].A:sample[k+1].B,t);
            }
            double RequiredAt(double s)=>(source[At(interval.A,Correspondence(s,true)).Edge].NativeWidth+
                source[At(interval.B,Correspondence(s,false)).Edge].NativeWidth)/2+2.1;
            double Common(double sourceStation,bool a)
            {
                var first=a?sample[0].A:sample[0].B;var last=a?sample[^1].A:sample[^1].B;
                if(sourceStation<Math.Min(first,last)-1e-7||sourceStation>Math.Max(first,last)+1e-7)return double.NaN;
                var lo=0;var hi=sample.Length-2;
                while(lo<hi){var mid=(lo+hi)/2;var end=a?sample[mid+1].A:sample[mid+1].B;if(a?sourceStation>end:sourceStation<end)lo=mid+1;else hi=mid;}
                var p=a?sample[lo].A:sample[lo].B;var q=a?sample[lo+1].A:sample[lo+1].B;
                return math.lerp(station[lo],station[lo+1],(sourceStation-p)/(q-p));
            }
            var cuts=new SortedSet<double>{0,station[^1]};
            foreach(var (chain,a) in new[]{(interval.A,true),(interval.B,false)})foreach(var edge in chain.Edges)
                foreach(var p in chain.Pieces.Where(p=>p.Edge==edge&&p.Begin==0))
                {var s=Common(p.Station,a);if(s>0&&s<station[^1])cuts.Add(s);}
            var emitted=new List<(double Begin,double End,Cubic3d A,Cubic3d B)>();var invalid=false;
            bool Separated(Cubic3d a,Cubic3d b,double minimum,int depth)
            {
                var delta=new Cubic3d(a.P0-b.P0,a.P1-b.P1,a.P2-b.P2,a.P3-b.P3);
                var normal=math.normalizesafe(NetworkDesignChecks.Evaluate(delta,.5).xz);
                if(math.lengthsq(normal)==0)return false;
                var lower=Math.Min(Math.Min(math.dot(delta.P0.xz,normal),math.dot(delta.P1.xz,normal)),
                    Math.Min(math.dot(delta.P2.xz,normal),math.dot(delta.P3.xz,normal)));
                if(lower>=minimum-1e-7)return true;
                if(depth==12)return false;
                var left=NetworkDesignChecks.Split(a,.5);var right=NetworkDesignChecks.Split(b,.5);
                return Separated(left.Left,right.Left,minimum,depth+1)&&Separated(left.Right,right.Right,minimum,depth+1);
            }
            double fitBegin=0,fitEnd=0;
            bool Fits(double begin,double end,Cubic3d fitted,int sign,int depth,double tolerance=.045)
            {
                // Independent check subdivision: the native curve is not split
                // merely because a proof needs a tighter reference interval.
                var a=Desired(begin,sign);var b=Desired(end,sign);var ta=(begin-fitBegin)/(fitEnd-fitBegin);var tb=(end-fitBegin)/(fitEnd-fitBegin);
                var ea=math.distance(a.Point,NetworkDesignChecks.Evaluate(fitted,ta).xz);var eb=math.distance(b.Point,NetworkDesignChecks.Evaluate(fitted,tb).xz);
                if(!SparseProfileRows.Finite(ea+eb)||ea>tolerance||eb>tolerance)return false;
                var piece=0;while(piece+1<spline.Pieces&&begin>=spline.Station(piece+1))piece++;
                if(end>spline.Station(piece+1)+1e-8)
                {var mid=Math.Min(end,spline.Station(piece+1));return Fits(begin,mid,fitted,sign,depth,tolerance)&&Fits(mid,end,fitted,sign,depth,tolerance);}
                var c=spline.Curve(piece);var h=spline.Station(piece+1)-spline.Station(piece);c=Subcurve(c,(begin-spline.Station(piece))/h,(end-spline.Station(piece))/h);
                var span=end-begin;var d0=3*(c.P1-c.P0).xz/span;var d1=3*(c.P2-c.P1).xz/span;var d2=3*(c.P3-c.P2).xz/span;
                var direction=math.normalizesafe(c.P3.xz-c.P0.xz);var minimum=Math.Min(math.dot(d0,direction),Math.Min(math.dot(d1,direction),math.dot(d2,direction)));
                if(!(minimum>1e-10))return false;
                var acceleration=Math.Max(math.length(6*(c.P2-2*c.P1+c.P0).xz),math.length(6*(c.P3-2*c.P2+c.P1).xz))/(span*span);
                var jerk=math.length(6*(c.P3-3*c.P2+3*c.P1-c.P0).xz)/(span*span*span);
                var offset=Math.Max(Math.Max(Math.Abs(c.P0.y),Math.Abs(c.P1.y)),Math.Max(Math.Abs(c.P2.y),Math.Abs(c.P3.y)));
                var offsetFirst=Math.Max(Math.Abs(3*(c.P1.y-c.P0.y)),Math.Max(Math.Abs(3*(c.P2.y-c.P1.y)),Math.Abs(3*(c.P3.y-c.P2.y))))/span;
                var offsetSecond=6*Math.Max(Math.Abs(c.P2.y-2*c.P1.y+c.P0.y),Math.Abs(c.P3.y-2*c.P2.y+c.P1.y))/(span*span);
                if(offset*acceleration/(minimum*minimum)>.5||Math.Min(Math.Min(c.P0.y,c.P1.y),Math.Min(c.P2.y,c.P3.y))<RequiredAt((begin+end)/2)/2-.001)return false;
                var correction=sign==side?correctionA:correctionB;
                var correctionBound=Math.Max(math.length(NetworkDesignChecks.SecondDerivative(correction,begin/station[^1]).xz),
                    math.length(NetworkDesignChecks.SecondDerivative(correction,end/station[^1]).xz))/(station[^1]*station[^1]);
                var bound=acceleration+offsetSecond+2*offsetFirst*acceleration/minimum+offset*(2*jerk/minimum+5*acceleration*acceleration/(minimum*minimum))+correctionBound;
                var sub=Subcurve(fitted,ta,tb);var candidateBound=6*Math.Max(math.length((sub.P2-2*sub.P1+sub.P0).xz),math.length((sub.P3-2*sub.P2+sub.P1).xz))/(span*span);
                if(Math.Max(ea,eb)+(bound+candidateBound)*span*span/8<=tolerance)return true;
                if(depth==16)return false;var middle=(begin+end)/2;return Fits(begin,middle,fitted,sign,depth+1,tolerance)&&Fits(middle,end,fitted,sign,depth+1,tolerance);
            }
            void Emit(double begin,double end,int depth)
            {
                token.ThrowIfCancellationRequested();var a0=Desired(begin,side);var a1=Desired(end,side);var b0=Desired(begin,-side);var b1=Desired(end,-side);
                Cubic3d Fit((double2 Point,double2 Derivative) a,(double2 Point,double2 Derivative) b)=>NetworkDesignChecks.Hermite(new(a.Point.x,0,a.Point.y),new(b.Point.x,0,b.Point.y),new(a.Derivative.x,0,a.Derivative.y),new(b.Derivative.x,0,b.Derivative.y),end-begin);
                var a=Fit(a0,a1);var b=Fit(b0,b1);fitBegin=begin;fitEnd=end;
                if(math.distance(((float3)a.P0).xz,((float3)a.P3).xz)<NetworkEmissionStations.MinimumReach(a)||
                    math.distance(((float3)b.P0).xz,((float3)b.P3).xz)<NetworkEmissionStations.MinimumReach(b))
                {invalid=true;log.Count("Paired emission intervals requiring representability refit");return;}
                double Rounding(Cubic3d c)=>new[]{c.P0,c.P1,c.P2,c.P3}.Max(p=>math.distance(p,(double3)(float3)p));
                if(Rounding(a)>.005||Rounding(b)>.005){invalid=true;return;}
                var valid=end-begin<=400+1e-7&&Fits(begin,end,a,side,0)&&Fits(begin,end,b,-side,0)&&
                    // The reference reserves 2*0.05 m fitting error above the
                    // physical 2 m median. Check the emitted pair against that
                    // physical minimum; do not spend the reserve twice.
                    Separated(a,b,RequiredAt((begin+end)/2)-.1,0);
                for(var j=0;j<=16;j++)
                {
                    var s=math.lerp(begin,end,j/16d);var pa=At(interval.A,Correspondence(s,true));var pb=At(interval.B,Correspondence(s,false));
                    var allowance=Math.Max(4,(RequiredAt(s)-math.distance(pa.Point,pb.Point))/2+4);
                    if(math.distance(pa.Point,Desired(s,side).Point)>allowance||math.distance(pb.Point,Desired(s,-side).Point)>allowance)
                    {invalid=true;return;} // Subdividing the fit cannot repair a rejected reference model.
                }
                if(!valid&&depth<12&&end-begin>1){var middle=(begin+end)/2;Emit(begin,middle,depth+1);Emit(middle,end,depth+1);return;}
                if(!valid){invalid=true;return;}emitted.Add((begin,end,a,b));
            }
            var boundaries=cuts.ToArray();for(var k=1;k<boundaries.Length;k++)Emit(boundaries[k-1],boundaries[k],0);
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
                var gapBound=Math.Max(Math.Max(math.distance(ca.P0.xz,cb.P0.xz),math.distance(ca.P1.xz,cb.P1.xz)),Math.Max(math.distance(ca.P2.xz,cb.P2.xz),math.distance(ca.P3.xz,cb.P3.xz)));
                var allowance=4+Math.Max(0,(required-gapBound)/2);fitBegin=begin;fitEnd=end;
                invalid=!Fits(begin,end,ca,side,0,allowance)||!Fits(begin,end,cb,-side,0,allowance);
            }
            if(invalid||emitted.Count==0){rejected++;continue;}
            // Do not claim overlapping intervals twice when a chain's partner
            // changes. The matcher ends intervals at ambiguity, not a sort tie.
            var additions=new Dictionary<int,List<(double Begin,double End,PreparedSegment Segment)>>();var localMoves=new Dictionary<long,float3>();var localDirections=new Dictionary<long,double2>();
            var portsA=new Dictionary<double,long>();var portsB=new Dictionary<double,long>();var families=new Dictionary<double,long>();var corridor=++corridorId;
            foreach(var e in emitted)
            {
                var span=++spanId;
                foreach(var isA in new[]{true,false})
                {
                    var chain=isA?interval.A:interval.B;var lo=At(chain,Correspondence(e.Begin,isA));var hi=At(chain,Correspondence(e.End,isA));
                    var middle=At(chain,Correspondence((e.Begin+e.End)/2,isA));var template=source[middle.Edge];
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
                    {if(!ports.TryGetValue(s,out var id))ports[s]=id=p.Parameter<1e-7?source[p.Edge].StartNodeId:p.Parameter>1-1e-7?source[p.Edge].EndNodeId:NewNode();return id;}
                    long Family(double s){if(!families.TryGetValue(s,out var id))families[s]=id=++familyId;return id;}
                    var startStation=isA?e.Begin:e.End;var endStation=isA?e.End:e.Begin;
                    var startNode=Port(startStation,start);var endNode=Port(endStation,end);
                    var y0=NetworkDesignChecks.Evaluate(Curve(source[start.Edge]),start.Parameter).y;var y1=NetworkDesignChecks.Evaluate(Curve(source[end.Edge]),end.Parameter).y;
                    curve=curve with{P0=curve.P0 with{y=y0},P1=curve.P1 with{y=math.lerp(y0,y1,1d/3)},P2=curve.P2 with{y=math.lerp(y0,y1,2d/3)},P3=curve.P3 with{y=y1}};
                    var p0=(float3)curve.P0;var p1=(float3)curve.P1;var p2=(float3)curve.P2;var p3=(float3)curve.P3;
                    var segment=template with{StartNodeId=startNode,EndNodeId=endNode,Start=p0,End=p3,Control1=p1,Control2=p2,FinalDeckCurve=new(p0,p1,p2,p3),HorizontalParent=null,HorizontalEmissionReference=curve,
                        StartCurveOffset=default,EndCurveOffset=default,
                        PairedCorridorId=corridor,PairedSpanId=span,PairedReversed=!isA,StartHeightFamily=Family(startStation),EndHeightFamily=Family(endStation),ReferenceSpan=e.End-e.Begin,
                        PairedProfileId=span,PairedReferenceBegin=startStation,PairedReferenceEnd=endStation,
                        StartReferenceSpeed=math.length((curve.P1-curve.P0).xz)*3/(e.End-e.Begin),EndReferenceSpeed=math.length((curve.P3-curve.P2).xz)*3/(e.End-e.Begin),
                        StartTerminal=start.Parameter<1e-7&&source[start.Edge].StartTerminal,EndTerminal=end.Parameter>1-1e-7&&source[end.Edge].EndTerminal};
                    if(!additions.TryGetValue(middle.Edge,out var list))additions[middle.Edge]=list=new();
                    list.Add((start.Parameter,end.Parameter,segment));
                    foreach(var port in new[]{(startNode,p0,start),(endNode,p3,end)})
                        if(occupied.Contains(port.Item1))localMoves[port.Item1]=port.Item2-(float3)NetworkDesignChecks.Evaluate(Curve(source[port.Item3.Edge]),port.Item3.Parameter);
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
            foreach(var move in localMoves)movements[move.Key]=move.Value;accepted++;
            foreach(var direction in localDirections)directions[direction.Key]=direction.Value;
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
                var startOffset=begin==0?s.StartCurveOffset:default;var endOffset=end==1?s.EndCurveOffset:default;
                var p0=(float3)c.P0+da;var p3=(float3)c.P3+db;var p1=(float3)c.P1+da;var p2=(float3)c.P2+db;
                // The paired owner also publishes the boundary tangent to its
                // independent remainder. Do not leave a second owner to repair
                // that join after the profile has already been solved.
                if(directions.TryGetValue(a,out var startDirection)){var reach=math.distance(c.P0.xz,c.P1.xz);p1.xz=p0.xz+(float2)(startDirection*reach);}
                if(directions.TryGetValue(b,out var endDirection)){var reach=math.distance(c.P2.xz,c.P3.xz);p2.xz=p3.xz-(float2)(endDirection*reach);}
                output.Add(s with{StartNodeId=a,EndNodeId=b,Start=p0-startOffset,End=p3-endOffset,Control1=p1-startOffset,Control2=p2-endOffset,
                    StartCurveOffset=startOffset,EndCurveOffset=endOffset,FinalDeckCurve=null,HorizontalParent=null,HorizontalEmissionReference=null,StartTerminal=begin==0&&s.StartTerminal,EndTerminal=end==1&&s.EndTerminal});
            }
            if(!replaced.ContainsKey(i)&&!movements.ContainsKey(s.StartNodeId)&&!movements.ContainsKey(s.EndNodeId)){output.Add(s);continue;}
            if(replaced.TryGetValue(i,out var pieces))foreach(var piece in pieces.OrderBy(p=>p.Begin))
            {Original(cursor,piece.Begin);output.Add(piece.Segment);cursor=piece.End;}
            Original(cursor,1);
        }
        log.Count("Trusted paired intervals",accepted);log.Count("Paired intervals requiring independent fit",rejected);
        var candidateEdges=new HashSet<int>(intervals.SelectMany(p=>p.A.Edges.Concat(p.B.Edges)));
        var eligibleLength=0d;var pairedLength=0d;var independentLength=0d;var coverage=log.DiagnosticsEnabled?new List<object>():null;
        for(var i=0;i<source.Count;i++)
        {
            if(!Eligible(source[i]))continue;token.ThrowIfCancellationRequested();var curve=Curve(source[i]);
            var length=NetworkDesignChecks.HorizontalArcLength(curve);var matched=0d;
            if(replaced.TryGetValue(i,out var ranges))foreach(var range in ranges)matched+=NetworkDesignChecks.HorizontalArcLength(curve,range.Begin,range.End);
            matched=Math.Min(length,matched);var independent=Math.Max(0,length-matched);
            eligibleLength+=length;pairedLength+=matched;independentLength+=independent;
            coverage?.Add(new{section=i,source[i].WayId,source[i].StartNodeId,source[i].EndNodeId,eligibleLengthMeters=length,
                pairedSourceLengthMeters=matched,independentSourceLengthMeters=independent,
                reason=independent<.01?"fully-certified-paired-interval":matched>0?"independent-boundary-or-unmatched-interval":
                    candidateEdges.Contains(i)?"reciprocal-candidate-not-certified":"no-reciprocal-candidate",
                unmatchedDoesNotProveMissingRequiredPartner=true});
        }
        if(coverage!=null)log.DiagnosticCompressedRecords("paired-correspondence-coverage.ndjson.gz",coverage);
        log.Diagnostic($"PAIR COVERAGE eligibleSourceLengthMeters={eligibleLength:R}; pairedSourceLengthMeters={pairedLength:R}; independentSourceLengthMeters={independentLength:R}; bothCarriagewaysCounted=true; sourceStationsNotSubdivisionCounts=true; unmatchedIsNotAutomaticallyAnError=true");
        log.Diagnostic($"TRUSTED PAIRS chains={chains.Count}; candidateIntervals={intervals.Count}; accepted={accepted}; independentFallbacks={rejected}; nonreciprocalOrAmbiguousStations={ambiguous}; sharedHeightSpans={spanId}; sourceBased=true; radiusMeters=96; minimumOverlapMeters=80; continuousProjection=true; synchronizedEmission=true; qualityFailureBlocksPlacement=false");
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
