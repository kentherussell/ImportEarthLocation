using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Partition the finished deck, not the taper influence. Only overlapping
// junction cores union; a long connecting corridor is never an atomic city.
public static class NativeDesignUnits
{
    // Shortest piece an optional unit cut may leave beside an existing node
    // (metres): under a lane width there is no usable native edge.
    private const double MinimumPiece=3;
    // No unit cut within this distance (metres) of another node of its kind.
    private const float CutClearance=3;
    // Added to a class minimum (metres): a cut placed exactly at the minimum
    // measures a few millimetres short once the pieces are refitted.
    private const double MinimumMargin=.5;
    private sealed class Chain
    {
        public SourceNodeKey Start, End;
        public readonly List<(int Index,bool Forward,double Length)> Spans=new();
        public double Length, StartCore, EndCore, Guard;
        public double RequiredStartCore,RequiredEndCore;
        public bool Indivisible;
    }

    // minimumPiece (visual compiler only) gives each chain's shortest acceptable
    // piece. Core boundaries first settle onto an existing node or at least that
    // far from one (Settle); a remaining cut that would leave a sliver is skipped.
    // A core cut is skipped only when the short side lies outside the core and
    // the inside part is longer, so the whole span still classifies into the
    // core's unit by its midpoint; corridor cuts carry no ownership.
    public static IReadOnlyList<PreparedSegment> Prepare(IReadOnlyList<PreparedSegment> source,
        IReadOnlyList<NativeAssetContract> contracts,ImportLog log,CancellationToken token,Func<PreparedSegment,double>? minimumPiece=null)
    {
        if(source.Count==0)return source;
        var assets=contracts.ToDictionary(a=>a.PaletteKey,StringComparer.Ordinal);
        var incident=new Dictionary<SourceNodeKey,List<int>>();
        SourceNodeKey A(PreparedSegment s)=>new(s.Kind,s.StartNodeId);
        SourceNodeKey B(PreparedSegment s)=>new(s.Kind,s.EndNodeId);
        for(var i=0;i<source.Count;i++)foreach(var key in new[]{A(source[i]),B(source[i])})
        {if(!incident.TryGetValue(key,out var list))incident[key]=list=new();list.Add(i);}
        var critical=new HashSet<SourceNodeKey>();var cores=new Dictionary<SourceNodeKey,double>();
        var guards=new Dictionary<SourceNodeKey,double>();
        var formations=new Dictionary<SourceNodeKey,List<double2>>();
        foreach(var pair in incident)
        {
            var first=source[pair.Value[0]];
            if(pair.Value.Count!=2||pair.Value.Any(i=>source[i].NativeAssetKey!=first.NativeAssetKey||
                source[i].Tunnel!=first.Tunnel||source[i].Bridge!=first.Bridge||source[i].Roundabout||
                source[i].Classification!=first.Classification||source[i].OneWay!=first.OneWay))critical.Add(pair.Key);
            var core=0d;var guard=.01d;
            var footprint=new List<double2>();
            foreach(var index in pair.Value)
            {
                var s=source[index];var width=(double)s.NativeWidth;var offset=0d;
                if(assets.TryGetValue(s.NativeAssetKey,out var asset)&&asset.TryEnvelope(NativeAssetContract.Mode(s),out var e))
                {width=Math.Max(width,e.Width);offset=e.NodeOffset;}
                var deckOffset=math.length(A(s)==pair.Key?s.StartCurveOffset:s.EndCurveOffset);
                core=Math.Max(core,Math.Max(offset,2*width+deckOffset));guard=Math.Max(guard,offset);
                var start=A(s)==pair.Key;var anchor=start?s.Start:s.End;
                var point=start?s.CourseStart:s.CourseEnd;var control=start?s.CourseControl1:s.CourseControl2;
                var tangent=math.normalizesafe((double2)(control-point).xz);var normal=new double2(-tangent.y,tangent.x);
                var center=(double2)(point-anchor).xz;
                foreach(var advance in new[]{0d,offset})foreach(var side in new[]{-1d,1d})
                    footprint.Add(center+tangent*advance+normal*(side*width/2));
            }
            cores[pair.Key]=core;guards[pair.Key]=guard;formations[pair.Key]=footprint;
        }
        var visited=new bool[source.Count];var chains=new List<Chain>();
        void Trace(SourceNodeKey start,int seed)
        {
            var chain=new Chain{Start=start};var node=start;var index=seed;
            while(!visited[index])
            {
                token.ThrowIfCancellationRequested();visited[index]=true;var s=source[index];var forward=A(s)==node;
                var length=Length(new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd));
                chain.Spans.Add((index,forward,length));chain.Length+=length;node=forward?B(s):A(s);
                if(critical.Contains(node)||node==start)break;
                var arms=incident[node];index=arms[0]==index?arms[1]:arms[0];
            }
            chain.End=node;chain.StartCore=cores[start];chain.EndCore=cores[node];
            chain.Guard=Math.Max(guards[start],guards[node]);
            double Required(SourceNodeKey key,int edge)
            {
                var s=source[edge];var first=A(s)==key;
                var point=first?s.CourseStart:s.CourseEnd;var control=first?s.CourseControl1:s.CourseControl2;
                var direction=math.normalizesafe((double2)(control-point).xz);
                var projection=guards[key];foreach(var vertex in formations[key])projection=Math.Max(projection,math.dot(vertex,direction));
                return projection;
            }
            chain.RequiredStartCore=Required(start,chain.Spans[0].Index);
            chain.RequiredEndCore=Required(node,chain.Spans[chain.Spans.Count-1].Index);
            chain.StartCore=Math.Max(chain.StartCore,chain.RequiredStartCore);chain.EndCore=Math.Max(chain.EndCore,chain.RequiredEndCore);
            // 2*width is a candidate cut envelope, not proof that two cores
            // overlap. Before union, use the projected actual central native
            // port rectangles and minimum node offset, excluding long tapers.
            if(chain.Length<=chain.StartCore+chain.EndCore+2*chain.Guard)
            {chain.StartCore=chain.RequiredStartCore;chain.EndCore=chain.RequiredEndCore;}
            chain.Indivisible=chain.Length<=chain.StartCore+chain.EndCore+2*chain.Guard;
            chains.Add(chain);
        }
        foreach(var key in critical.OrderBy(k=>k.Kind).ThenBy(k=>k.Id))
            foreach(var index in incident[key])if(!visited[index])Trace(key,index);
        // A source cycle has no degree != 2 node. Its deterministic existing seam
        // is retained; prefix and suffix belong to the same seam core.
        for(var i=0;i<source.Count;i++)if(!visited[i])Trace(A(source[i]),i);

        var parents=new Dictionary<SourceNodeKey,SourceNodeKey>();
        SourceNodeKey Root(SourceNodeKey key)
        {
            if(!parents.ContainsKey(key)){parents[key]=key;return key;}
            var root=key;while(parents[root]!=root)root=parents[root];
            while(parents[key]!=key){var next=parents[key];parents[key]=root;key=next;}return root;
        }
        foreach(var chain in chains)if(chain.Indivisible)
        {var a=Root(chain.Start);var b=Root(chain.End);if(a!=b)parents[b]=a;}
        var units=new Dictionary<SourceNodeKey,int>();var nextUnit=0;
        int Core(SourceNodeKey key)
        {key=Root(key);if(!units.TryGetValue(key,out var value))units[key]=value=nextUnit++;return value;}
        var occupied=new HashSet<long>(source.SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}));var nextNode=long.MinValue+12000000;
        long Node(){while(occupied.Contains(nextNode))nextNode++;var id=nextNode++;occupied.Add(id);return id;}
        var result=new List<PreparedSegment>(source.Count);var splits=0;var indivisible=0;var slivers=0;var settled=0;var moved=0;var crowded=0;
        // Node positions by kind in CutClearance cells. The game joins nodes at
        // one position: two tracks cut where they cross (Marseille, 21 October:
        // a surface line over a tunnel line) got four unit nodes at one point,
        // which it wired together into dead ends.
        var nodes=new Dictionary<(NetworkKind,int,int),List<float2>>();
        (NetworkKind,int,int) NodeCell(NetworkKind kind,float2 p)=>(kind,(int)Math.Floor(p.x/CutClearance),(int)Math.Floor(p.y/CutClearance));
        void AddNode(NetworkKind kind,float2 p){var key=NodeCell(kind,p);if(!nodes.TryGetValue(key,out var list))nodes[key]=list=new();list.Add(p);}
        bool Clear(NetworkKind kind,float2 p)
        {
            var (_,cx,cz)=NodeCell(kind,p);
            for(var dz=-1;dz<=1;dz++)for(var dx=-1;dx<=1;dx++)if(nodes.TryGetValue((kind,cx+dx,cz+dz),out var list)&&list.Any(q=>math.distance(q,p)<CutClearance))return false;
            return true;
        }
        foreach(var s in source){AddNode(s.Kind,s.CourseStart.xz);AddNode(s.Kind,s.CourseEnd.xz);}
        foreach(var chain in chains)
        {
            token.ThrowIfCancellationRequested();var startUnit=Core(chain.Start);var endUnit=Core(chain.End);
            var minimum=minimumPiece==null?0:minimumPiece(source[chain.Spans[0].Index])+MinimumMargin;
            var boundaries=new SortedSet<double>();var corridorUnits=new List<int>();
            // One already-fitted span needs one safe handoff, not two cuts
            // enclosing a third, otherwise empty corridor batch. The boundary
            // remains inside neutral space beyond both native cores/guards.
            var singleHandoff=!chain.Indivisible&&chain.Spans.Count==1&&chain.Start!=chain.End;
            if(chain.Indivisible)indivisible++;
            else if(singleHandoff)boundaries.Add((chain.StartCore+chain.Length-chain.EndCore)/2);
            else
            {
                var prefix=new double[chain.Spans.Count+1];
                for(var k=0;k<chain.Spans.Count;k++)prefix[k+1]=prefix[k]+chain.Spans[k].Length;
                if(minimum>0)
                {
                    var inner=prefix.Skip(1).Take(chain.Spans.Count-1).ToArray();
                    var startCore=Settle(chain.RequiredStartCore,chain.StartCore,inner,minimum);
                    var endCore=Settle(chain.RequiredEndCore,chain.EndCore,inner.Select(e=>chain.Length-e).ToArray(),minimum);
                    if(startCore+endCore+2*chain.Guard<chain.Length&&(startCore!=chain.StartCore||endCore!=chain.EndCore))
                    {chain.StartCore=startCore;chain.EndCore=endCore;settled++;}
                }
                boundaries.Add(chain.StartCore);boundaries.Add(chain.Length-chain.EndCore);
                void Partition(double begin,double end)
                {
                    token.ThrowIfCancellationRequested();var count=0;
                    for(var k=0;k<chain.Spans.Count;k++)if(prefix[k]<end&&prefix[k+1]>begin)count++;
                    // Optional corridor cuts are physical midpoints, away from
                    // both cores. Never obtain a small batch by cutting a core.
                    if(count<=32||end-begin<=2*chain.Guard+.02)return;
                    var middle=(begin+end)/2;boundaries.Add(middle);Partition(begin,middle);Partition(middle,end);
                }
                Partition(chain.StartCore,chain.Length-chain.EndCore);
                for(var k=1;k<boundaries.Count;k++)corridorUnits.Add(nextUnit++);
            }
            var stations=boundaries.ToArray();
            var distance=0d;
            foreach(var span in chain.Spans)
            {
                var s=source[span.Index];var cuts=new List<double>();var joined=new List<(double T,int Unit)>();
                // Every piece but a roundabout ring keeps the class minimum on both sides.
                var bounded=minimum>0&&!s.Roundabout;
                if(!chain.Indivisible)
                {
                    foreach(var boundary in stations)
                    {
                        var local=boundary-distance;
                        if(local>.01&&local<span.Length-.01)
                        {
                            var before=local;var after=span.Length-local;
                            var startCore=!singleHandoff&&Math.Abs(boundary-chain.StartCore)<1e-6;var endCore=!singleHandoff&&Math.Abs(boundary-(chain.Length-chain.EndCore))<1e-6;
                            var curve=new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);
                            var t=Parameter(curve,span.Forward?local:span.Length-local);
                            if(Math.Min(before,after)<MinimumPiece)
                            {
                                // No sub-lane piece beside a node. A core boundary whose
                                // short side lies inside the core takes the whole span
                                // into that core, so the junction footprint stays whole.
                                if(startCore&&before<after)joined.Add((t,startUnit));
                                else if(endCore&&after<before)joined.Add((t,endUnit));
                                slivers++;continue;
                            }
                            if(minimum>0)
                            {
                                var skip=startCore?after<minimum&&before>=after:endCore?before<minimum&&after>=before:Math.Min(before,after)<minimum;
                                if(skip){slivers++;continue;}
                            }
                            // A cut on another node of its kind slides along the
                            // span (3, 6, 9 m; a core boundary only away from its
                            // core, which must stay whole) or is left out.
                            var chosen=double.NaN;
                            var shifts=startCore?new[]{0d,3,6,9}:endCore?new[]{0d,-3,-6,-9}:new[]{0d,3,-3,6,-6,9,-9};
                            // A core boundary that would leave its core side shorter
                            // than the class's shortest section first moves out
                            // until it does not, when the other side stays as long.
                            if(minimum>0&&startCore&&before<minimum&&after-(minimum-before)>=minimum)shifts=new[]{minimum-before}.Concat(shifts).ToArray();
                            if(minimum>0&&endCore&&after<minimum&&before-(minimum-after)>=minimum)shifts=new[]{-(minimum-after)}.Concat(shifts).ToArray();
                            if(!span.Forward&&(startCore||endCore))shifts=shifts.Select(x=>-x).ToArray();
                            foreach(var shift in shifts)
                            {
                                var at=local+shift;if(at<MinimumPiece||span.Length-at<MinimumPiece)continue;
                                if(bounded&&Math.Min(at,span.Length-at)<minimum)continue;
                                var u=Parameter(curve,span.Forward?at:span.Length-at);
                                if(bounded&&cuts.Any(c=>Length(curve,Math.Min(c,u),Math.Max(c,u))<minimum))continue;
                                if(u<=0||u>=1||!CanSplit(s,u)||!Clear(s.Kind,((float3)NetworkDesignChecks.Evaluate(curve,u)).xz))continue;
                                chosen=u;if(shift!=0)moved++;break;
                            }
                            if(double.IsNaN(chosen))
                            {
                                if(bounded&&(startCore||endCore))
                                {joined.Add((t,startCore?startUnit:endUnit));slivers++;}
                                else crowded++;
                                continue;
                            }
                            cuts.Add(chosen);AddNode(s.Kind,((float3)NetworkDesignChecks.Evaluate(curve,chosen)).xz);
                        }
                    }
                }
                cuts.Sort();cuts.Add(1);var begin=0d;var piece=s;
                foreach(var end in cuts)
                {
                    PreparedSegment child;
                    if(end<1)
                    {
                        var pair=Split(piece,(end-begin)/(1-begin),Node());child=pair.Left;piece=pair.Right;splits++;
                    }
                    else child=piece;
                    // Classify by reference-chain station, never by XZ proximity.
                    var middle=Length(new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd),0,(begin+end)/2);
                    var station=distance+(span.Forward?middle:span.Length-middle);
                    var unit=startUnit;
                    if(!chain.Indivisible)
                    {
                        if(singleHandoff)unit=station<stations[0]?startUnit:endUnit;
                        else if(station>chain.Length-chain.EndCore)unit=endUnit;
                        else if(station>=chain.StartCore)
                        {var bucket=Array.BinarySearch(stations,station);if(bucket<0)bucket=~bucket-1;unit=corridorUnits[Math.Min(bucket,corridorUnits.Count-1)];}
                        foreach(var j in joined)if(j.T>begin&&j.T<end)unit=j.Unit;
                    }
                    result.Add(child with{NativeUnitId=unit});begin=end;
                }
                distance+=span.Length;
            }
        }
        // Union final unit cuts in each shared reference profile. The opposite
        // carriageway keeps a distinct node, but gets the identical station cut.
        var sharedCuts=result.Where(s=>s.PairedProfileId!=0).GroupBy(s=>s.PairedProfileId).ToDictionary(g=>g.Key,
            g=>g.SelectMany(s=>new[]{s.PairedReferenceBegin,s.PairedReferenceEnd}).Distinct().OrderBy(t=>t).ToArray());
        var synchronized=new List<PreparedSegment>(result.Count);
        foreach(var s in result)
        {
            token.ThrowIfCancellationRequested();
            if(s.PairedProfileId==0){synchronized.Add(s);continue;}
            var parameters=sharedCuts[s.PairedProfileId].Select(station=>(station-s.PairedReferenceBegin)/(s.PairedReferenceEnd-s.PairedReferenceBegin))
                .Where(t=>t>1e-8&&t<1-1e-8).OrderBy(t=>t).ToArray();
            var remaining=s;var begin=0d;
            foreach(var t in parameters)
            {
                var local=(t-begin)/(1-begin);
                if(!CanSplit(remaining,local)){log.Count("Unrepresentable optional unit cuts avoided");continue;}
                // The opposite carriageway's cut can fall beside a node of this one.
                var piece=new Cubic3d(remaining.CourseStart,remaining.CourseControl1,remaining.CourseControl2,remaining.CourseEnd);
                var cutMinimum=!s.Roundabout?
                    Math.Max(MinimumPiece,minimumPiece==null?0:minimumPiece(s)+MinimumMargin):MinimumPiece;
                if(Math.Min(Length(piece,0,local),Length(piece,local,1))<cutMinimum){log.Count("Paired unit cuts beside a node skipped");continue;}                var cutAt=((float3)NetworkDesignChecks.Evaluate(piece,local)).xz;
                if(!Clear(remaining.Kind,cutAt)){log.Count("Paired unit cuts on another node skipped");continue;}
                AddNode(remaining.Kind,cutAt);
                var pair=Split(remaining,local,Node());synchronized.Add(pair.Left);remaining=pair.Right;begin=t;splits++;
            }
            synchronized.Add(remaining);
        }
        result=synchronized;
        var sizes=result.GroupBy(s=>s.NativeUnitId).Select(g=>g.Count()).ToArray();
        if(log.DiagnosticsEnabled)log.DiagnosticRecords("native-unit-core-extents.ndjson",chains.Select(c=>(object)new
        {c.Start,c.End,c.Length,c.StartCore,c.EndCore,c.RequiredStartCore,c.RequiredEndCore,c.Guard,c.Indivisible,
            candidateStartCore=cores[c.Start],candidateEndCore=cores[c.End],footprint="projected-central-port-rectangles-not-taper-influence",nativePreviewLegalityUnverified=true}));
        log.Count("Native design units",sizes.Length);log.Count("Native unit exact boundary splits",splits);
        log.Count("Native unit cuts moved off another node",moved);log.Count("Native unit cuts left out beside another node",crowded);
        if(minimumPiece!=null){log.Count("Native unit sliver cuts skipped",slivers);log.Count("Native unit core boundaries settled",settled);}
        if(log.DiagnosticsEnabled)log.DiagnosticJson("native-segmentation-reasons.json",new
        {
            inputSections=source.Count,emittedSections=result.Count,ownershipHandoffSplits=splits,
            pairedSynchronizationSplits=result.Count-source.Count-splits,
            branchNodes=incident.Count(p=>p.Value.Count>2),terminalNodes=incident.Count(p=>p.Value.Count==1),
            semanticOrModeBoundaryNodes=critical.Count(p=>incident[p].Count==2),
            retainedDegreeTwoNodes=incident.Count(p=>p.Value.Count==2&&!critical.Contains(p.Key)),
            sampledStationsCreateNativeNodes=false,countsAreNotMutuallyExclusive=true,
            nativeSafetyAndAtomicCoreBoundariesPreserved=true
        });
        log.Diagnostic($"NATIVE DESIGN UNITS units={sizes.Length}; sections={result.Count}; exactBoundarySplits={splits}; indivisibleChains={indivisible}; largestUnit={sizes.Max()}; overlappingCoresOnly=true; taperInfluenceDoesNotUnion=true; heuristicAloneUnions=false; centralFootprintProjected=true; nativeAcceptance=RequiresReview");
        return result;
    }

    // Distance of a core boundary from its junction. The existing node nearest
    // the wanted extent, between the projected native footprint (required) and
    // one longest compiled section (400 m) beyond it, becomes the boundary, so no
    // cut is made at all; otherwise the cut lies at least one piece beyond both
    // the core extent and the node behind it. Replay showed unchanged unit
    // counts and sizes (largest 32 sections) with fewer sections overall.
    private static double Settle(double required,double core,double[] nodes,double minimum)
    {
        const double reach=400;
        var wanted=Math.Max(core,minimum);var best=double.NaN;
        foreach(var e in nodes)if(e>=required&&e<=wanted+reach&&(double.IsNaN(best)||Math.Abs(e-wanted)<Math.Abs(best-wanted)))best=e;
        if(!double.IsNaN(best))return best;
        var behind=0d;foreach(var e in nodes)if(e<wanted)behind=Math.Max(behind,e);
        return Math.Max(wanted,behind+minimum);
    }

    private static double Length(Cubic3d c,double begin=0,double end=1)
    {
        // Invalid geometry is retained for the typed native safety decision,
        // not recursively subdivided into hundreds of thousands of NaN leaves.
        if(!NetworkDesignChecks.Finite(c))return 0;
        if(end<1)c=NetworkDesignChecks.Split(c,end).Left;
        if(begin>0)c=NetworkDesignChecks.Split(c,begin/end).Right;
        double Visit(Cubic3d curve,double tolerance,int depth)
        {
            var lower=math.distance(curve.P0.xz,curve.P3.xz);
            var upper=math.distance(curve.P0.xz,curve.P1.xz)+math.distance(curve.P1.xz,curve.P2.xz)+math.distance(curve.P2.xz,curve.P3.xz);
            if(upper-lower<=tolerance||depth==18)return (upper+lower)/2;
            var pair=NetworkDesignChecks.Split(curve,.5);return Visit(pair.Left,tolerance/2,depth+1)+Visit(pair.Right,tolerance/2,depth+1);
        }
        return Visit(c,.001,0);
    }

    private static double Parameter(Cubic3d curve,double distance)
    {
        var lo=0d;var hi=1d;
        // Opposite parameterizations must publish the same paired station to
        // the existing 1e-7 m agreement, even for a 400 m source span.
        for(var i=0;i<36;i++){var t=(lo+hi)/2;if(Length(curve,0,t)<distance)lo=t;else hi=t;}
        return (lo+hi)/2;
    }

    private static bool CanSplit(PreparedSegment s,double t)
    {
        var curve=new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);
        var p=(float3)NetworkDesignChecks.Evaluate(curve,t);
        // Optional partition cuts must survive the final float coordinate grid.
        // This does not remove a source junction or impose a road-length policy.
        var magnitude=Math.Max(math.cmax(math.abs(s.CourseStart.xz)),math.cmax(math.abs(s.CourseEnd.xz)));
        var margin=Math.Max(.05,magnitude/1048576d);
        return math.distance(p.xz,s.CourseStart.xz)>margin&&math.distance(p.xz,s.CourseEnd.xz)>margin;
    }

    public static (PreparedSegment Left,PreparedSegment Right) Split(PreparedSegment s,double t,long node)
    {
        var pair=NetworkDesignChecks.Split(new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd),t);
        var reference=s.HorizontalEmissionReference.HasValue?NetworkDesignChecks.Split(s.HorizontalEmissionReference.Value,t):
            ((Cubic3d Left,Cubic3d Right)?)null;
        var middle=(float3)pair.Left.P3;var offset=math.lerp(s.StartCurveOffset,s.EndCurveOffset,(float)t);var anchor=middle-offset;
        var station=math.lerp(s.PairedReferenceBegin,s.PairedReferenceEnd,t);
        float? elevation=s.StartElevation.HasValue&&s.EndElevation.HasValue?math.lerp(s.StartElevation.Value,s.EndElevation.Value,(float)t):null;
        var left=s with{EndNodeId=node,End=anchor,EndCurveOffset=offset,EndElevation=elevation,
            PairedSpanId=0,ReferenceSpan=0,EndHeightFamily=0,
            PairedReferenceEnd=station,
            Control1=(float3)pair.Left.P1-s.StartCurveOffset,Control2=(float3)pair.Left.P2-offset,
            EndTerminal=false,EndLevelCrossing=false,EndMiniRoundabout=false,EndRoundaboutRadius=0,FinalDeckCurve=null,HorizontalParent=null,HorizontalEmissionReference=reference?.Left};
        var right=s with{StartNodeId=node,Start=anchor,StartCurveOffset=offset,StartElevation=elevation,
            PairedSpanId=0,ReferenceSpan=0,StartHeightFamily=0,
            PairedReferenceBegin=station,
            Control1=(float3)pair.Right.P1-offset,Control2=(float3)pair.Right.P2-s.EndCurveOffset,
            StartTerminal=false,StartLevelCrossing=false,StartMiniRoundabout=false,StartRoundaboutRadius=0,FinalDeckCurve=null,HorizontalParent=null,HorizontalEmissionReference=reference?.Right};
        left=left with{FinalDeckCurve=new((float3)pair.Left.P0,(float3)pair.Left.P1,(float3)pair.Left.P2,middle)};
        right=right with{FinalDeckCurve=new(middle,(float3)pair.Right.P1,(float3)pair.Right.P2,(float3)pair.Right.P3)};
        // Keep the rounded split port identical on both sides, including its
        // deck/node decomposition. No independent resampling or snapping.
        return (left,right);
    }
}
