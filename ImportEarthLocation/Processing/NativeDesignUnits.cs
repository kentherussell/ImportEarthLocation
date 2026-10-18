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
    private sealed class Chain
    {
        public SourceNodeKey Start, End;
        public readonly List<(int Index,bool Forward,double Length)> Spans=new();
        public double Length, StartCore, EndCore, Guard;
        public double RequiredStartCore,RequiredEndCore;
        public bool Indivisible;
    }

    public static IReadOnlyList<PreparedSegment> Prepare(IReadOnlyList<PreparedSegment> source,
        IReadOnlyList<NativeAssetContract> contracts,ImportLog log,CancellationToken token)
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
        var result=new List<PreparedSegment>(source.Count);var splits=0;var indivisible=0;
        foreach(var chain in chains)
        {
            token.ThrowIfCancellationRequested();var startUnit=Core(chain.Start);var endUnit=Core(chain.End);
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
                var s=source[span.Index];var cuts=new List<double>();
                if(!chain.Indivisible)
                {
                    foreach(var boundary in stations)
                    {
                        var local=boundary-distance;
                        if(local>.01&&local<span.Length-.01)
                        {
                            var curve=new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);
                            var t=Parameter(curve,span.Forward?local:span.Length-local);
                            if(t>0&&t<1&&CanSplit(s,t))cuts.Add(t);
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
            EndTerminal=false,EndMiniRoundabout=false,EndRoundaboutRadius=0,FinalDeckCurve=null,HorizontalParent=null,HorizontalEmissionReference=reference?.Left};
        var right=s with{StartNodeId=node,Start=anchor,StartCurveOffset=offset,StartElevation=elevation,
            PairedSpanId=0,ReferenceSpan=0,StartHeightFamily=0,
            PairedReferenceBegin=station,
            Control1=(float3)pair.Right.P1-offset,Control2=(float3)pair.Right.P2-s.EndCurveOffset,
            StartTerminal=false,StartMiniRoundabout=false,StartRoundaboutRadius=0,FinalDeckCurve=null,HorizontalParent=null,HorizontalEmissionReference=reference?.Right};
        left=left with{FinalDeckCurve=new((float3)pair.Left.P0,(float3)pair.Left.P1,(float3)pair.Left.P2,middle)};
        right=right with{FinalDeckCurve=new(middle,(float3)pair.Right.P1,(float3)pair.Right.P2,(float3)pair.Right.P3)};
        // Keep the rounded split port identical on both sides, including its
        // deck/node decomposition. No independent resampling or snapping.
        return (left,right);
    }
}
