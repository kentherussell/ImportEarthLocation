using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Geographic freedom is granted before fitting, never by relaxing a failed
// physical certificate. Bounds deliberately overprotect nearby infrastructure.
public static class NetworkVisualDesign
{
    public static IReadOnlyList<PreparedSegment> StructureIntent(IReadOnlyList<PreparedSegment> source,ushort[] terrain,float scale,double width,
        IReadOnlyDictionary<string,NativeAssetContract>? assets,ImportLog log,CancellationToken token,
        ISet<int>? allowedSections=null,bool propagateChainConflicts=false)
    {
        if(assets==null)
        {log.Quality(new("planned","structure-intent","pre-profile",0,0,1,0,0,"Native mode contracts unavailable; no inferred structural changes"));return source;}
        var original=source;
        source=PairedStructureModes(source,assets,log,token);
        var orientation=OrientRailRuns(source,token);source=orientation.Source;
        var incident=new Dictionary<SourceNodeKey,List<int>>();
        for(var i=0;i<source.Count;i++)foreach(var id in new[]{source[i].StartNodeId,source[i].EndNodeId})
        {var key=new SourceNodeKey(source[i].Kind,id);if(!incident.TryGetValue(key,out var arms))incident[key]=arms=new();arms.Add(i);}
        bool Eligible(PreparedSegment s)=>!NetworkStructure.IsStructure(s)&&!s.ProtectedLink&&!s.Roundabout&&
            (s.Kind==NetworkKind.Rail||s.Classification is "motorway" or "trunk")&&!s.RawSourceTags.ContainsKey("layer")&&
            !s.RawSourceTags.ContainsKey("bridge")&&!s.RawSourceTags.ContainsKey("tunnel")&&s.PairedProfileId==0;
        bool EligibleAt(int i)=>(allowedSections==null||allowedSections.Contains(i))&&Eligible(source[i]);
        var output=source.ToArray();var visited=new bool[source.Count];var bridges=0;var tunnels=0;var unresolved=0;var unavailable=0;var checkedIntervals=0;
        var groundConflicts=new HashSet<int>();
        // Accumulated chain conflicts remain diagnostic until coordinated mode
        // selection can resolve them without worsening independent cover/grade.
        NetworkGroundFeasibility.Check(source,terrain,scale,width,token,conflictSection:i=>groundConflicts.Add(i),propagateChains:propagateChainConflicts);
        foreach(var seed in Enumerable.Range(0,source.Count).OrderBy(i=>source[i].WayId).ThenBy(i=>source[i].StartNodeId))
        {
            token.ThrowIfCancellationRequested();if(visited[seed]||!EligibleAt(seed))continue;
            var path=new LinkedList<int>();path.AddLast(seed);visited[seed]=true;
            void Extend(bool backwards)
            {
                var edge=backwards?path.First!.Value:path.Last!.Value;
                while(true)
                {
                    var s=source[edge];var node=backwards?s.StartNodeId:s.EndNodeId;var arms=incident[new(s.Kind,node)];
                    if(arms.Count!=2)break;var next=arms.First(i=>i!=edge);var n=source[next];
                    if(visited[next]||!EligibleAt(next)||n.NativeAssetKey!=s.NativeAssetKey||n.Layer!=s.Layer||n.OneWay!=s.OneWay||
                        (backwards?n.EndNodeId:n.StartNodeId)!=node)break;
                    visited[next]=true;if(backwards)path.AddFirst(next);else path.AddLast(next);edge=next;
                }
            }
            Extend(true);Extend(false);var indices=path.ToArray();var first=source[indices[0]];var last=source[indices[^1]];
            var lengths=indices.Select(i=>NetworkDesignChecks.HorizontalArcLength(new(source[i].CourseStart,source[i].CourseControl1,source[i].CourseControl2,source[i].CourseEnd))).ToArray();
            var windows=new List<(int Begin,int End)>();
            if(lengths.Sum()<=3000)windows.Add((0,indices.Length));
            else foreach(var bad in Enumerable.Range(0,indices.Length).Where(k=>groundConflicts.Contains(indices[k])))
            {
                var begin=bad;var end=bad+1;var reach=lengths[bad];
                while(true)
                {
                    var a=source[indices[begin]].CourseStart;var b=source[indices[end-1]].CourseEnd;
                    if(reach>=500&&Math.Abs(a.y-b.y)<=NetworkVerticalProfiles.DesiredGrade(first)*reach)break;
                    var left=begin>0?lengths[begin-1]:double.PositiveInfinity;var right=end<indices.Length?lengths[end]:double.PositiveInfinity;
                    var next=Math.Min(left,right);if(!SparseProfileRows.Finite(next)||reach+next>3000)break;
                    reach+=next;if(left<=right)begin--;else end++;
                }
                if(windows.Count>0&&begin<windows[windows.Count-1].End)
                    windows[windows.Count-1]=(windows[windows.Count-1].Begin,Math.Max(end,windows[windows.Count-1].End));
                else windows.Add((begin,end));
            }
            foreach(var window in windows)
            {
                var run=indices.Skip(window.Begin).Take(window.End-window.Begin).ToArray();
                var runLengths=lengths.Skip(window.Begin).Take(window.End-window.Begin).ToArray();
                var necessarilyIncompatible=run.Any(groundConflicts.Contains);
                var total=runLengths.Sum();if(total>3000||total<250&&!necessarilyIncompatible)continue;
                first=source[run[0]];last=source[run[run.Length-1]];
                checkedIntervals++;
                var y0=first.Start.y;var y1=last.End.y;
                var intentGrade=necessarilyIncompatible?NetworkGeometrySafety.GradeLimit(first):NetworkVerticalProfiles.DesiredGrade(first);
                if(Math.Abs(y1-y0)>intentGrade*total)
                {
                    if(!necessarilyIncompatible){unresolved++;continue;}
                    // These heights are a terrain-derived prior, not verified
                    // native boundary poses. A proven impossible ground run
                    // must not be vetoed by fixing that same prior. Evaluate
                    // structural intent against a grade-limited line; the
                    // joint solve still owns endpoint heights and all physical
                    // cover, clearance and neighbouring-ground obligations.
                    var middle=((double)y0+y1)/2;var rise=Math.Sign(y1-y0)*intentGrade*total/2;
                    y0=(float)(middle-rise);y1=(float)(middle+rise);
                }
                var low=0d;var high=0d;var distance=0d;
                for(var k=0;k<run.Length;k++)
                {
                    var s=source[run[k]];
                    for(var j=0;j<=16;j++)
                    {
                        var t=j/16d;var p=NetworkNormalizer.Evaluate(s,(float)t);var target=math.lerp(y0,y1,(distance+t*runLengths[k])/total);
                        var delta=target-NetworkProcessor.Sample(p.x,p.z,terrain,scale,width);low=Math.Min(low,delta);high=Math.Max(high,delta);
                    }
                    distance+=runLengths[k];
                }
                // Ordinary ground routes require a clear valley or crest.
                // Proven impossible ground intervals make one bounded mode
                // decision even for mixed terrain; the unchanged continuous
                // cover/grade rows still decide physical feasibility.
                var threshold=first.Kind==NetworkKind.Rail?4:8;NativeStructureMode? mode=null;
                if(high>threshold&&low>=-1)mode=NativeStructureMode.Elevated;
                else if(low<-threshold&&high<=1)mode=NativeStructureMode.Tunnel;
                else if(necessarilyIncompatible)
                {if(low<-threshold)mode=NativeStructureMode.Tunnel;else if(high>threshold)mode=NativeStructureMode.Elevated;}
                if(!mode.HasValue){if(high>threshold||low<-threshold||necessarilyIncompatible)unresolved++;continue;}
                if(!TryModeAsset(first,mode.Value,assets,out var asset)||!asset.TryEnvelope(mode.Value,out var envelope))
                {unavailable++;continue;}
                if(run.Any(i=>envelope.Width>source[i].MaximumNativeWidth)){unresolved++;continue;}
                distance=0;
                for(var k=0;k<run.Length;k++)
                {
                    var i=run[k];var s=source[i];var length=runLengths[k];var a=(float)math.lerp(y0,y1,distance/total);var b=(float)math.lerp(y0,y1,(distance+length)/total);
                    var start=s.Start;var end=s.End;start.y=a;end.y=b;var c1=s.CourseControl1;var c2=s.CourseControl2;c1.y=math.lerp(a,b,1f/3);c2.y=math.lerp(a,b,2f/3);
                    output[i]=s with{Start=start,End=end,Control1=c1-s.StartCurveOffset,Control2=c2-s.EndCurveOffset,FinalDeckCurve=null,
                        Bridge=mode==NativeStructureMode.Elevated,Tunnel=mode==NativeStructureMode.Tunnel,NativeAssetKey=asset.PaletteKey,NativeLaneOffsets=asset.LaneOffsets,NativeWidth=Math.Max(s.NativeWidth,envelope.Width),
                        NativeCenterlineCurvatureLimit=asset.CenterlineCurvatureLimit(Math.Max(s.NativeWidth,envelope.Width))};
                    distance+=length;if(mode==NativeStructureMode.Elevated)bridges++;else tunnels++;
                }
            }
        }
        log.Diagnostic($"STRUCTURE INTENT bridgeSections={bridges}; tunnelSections={tunnels}; unresolvedIntervals={unresolved}; minimumIntervalMeters=250; maximumIntervalMeters=3000; explicitModesPreserved=true; pairedModesPreserved=true; gradeAndCoverStillValidated=true");
        log.Quality(new("planned","structure-intent","pre-profile",checkedIntervals-unavailable,unresolved,unavailable,0,0,
            "Bounded valley/crest selection; mixed/steep/unavailable or over-width cases retain geometry for review; not a feasibility certificate or native acceptance"){Expected=checkedIntervals});
        var result=StructuralProfiles(output,log,token).ToArray();
        foreach(var i in orientation.Reversed)result[i]=NetworkNormalizer.Reverse(result[i]);
        if(allowedSections!=null)for(var i=0;i<result.Length;i++)if(!allowedSections.Contains(i))result[i]=original[i];
        Dictionary<SourceNodeKey,float>? fixedAnchors=null;
        if(allowedSections!=null)
        {
            fixedAnchors=new();
            for(var i=0;i<original.Count;i++)if(!allowedSections.Contains(i))
            {
                var s=original[i];fixedAnchors[new(s.Kind,s.StartNodeId)]=s.Start.y;fixedAnchors[new(s.Kind,s.EndNodeId)]=s.End.y;
            }
        }
        PublishIntentAnchors(original,result,log,fixedAnchors);
        log.Count("Undirected rail sections oriented for structural intent",orientation.Reversed.Count);
        return result;
    }

    private static void PublishIntentAnchors(IReadOnlyList<PreparedSegment> original,PreparedSegment[] result,ImportLog log,
        IReadOnlyDictionary<SourceNodeKey,float>? fixedAnchors=null)
    {
        var proposals=new Dictionary<SourceNodeKey,(double Sum,int Count)>();
        for(var i=0;i<result.Length;i++)foreach(var start in new[]{true,false})
        {
            var before=start?original[i].Start:original[i].End;var after=start?result[i].Start:result[i].End;
            if(before.y==after.y)continue;var key=new SourceNodeKey(result[i].Kind,start?result[i].StartNodeId:result[i].EndNodeId);
            proposals.TryGetValue(key,out var value);proposals[key]=(value.Sum+after.y,value.Count+1);
        }
        var published=0;
        for(var i=0;i<result.Length;i++)
        {
            var s=result[i];var a=s.Start;var b=s.End;
            if(proposals.TryGetValue(new(s.Kind,s.StartNodeId),out var start))a.y=(float)(start.Sum/start.Count);
            if(proposals.TryGetValue(new(s.Kind,s.EndNodeId),out var end))b.y=(float)(end.Sum/end.Count);
            // A corrective prior consumes committed boundary heights; it must
            // not average them with an inferred structural prior. The joint
            // solver retains the corresponding native controls independently.
            if(fixedAnchors!=null&&fixedAnchors.TryGetValue(new(s.Kind,s.StartNodeId),out var fixedStart))a.y=fixedStart;
            if(fixedAnchors!=null&&fixedAnchors.TryGetValue(new(s.Kind,s.EndNodeId),out var fixedEnd))b.y=fixedEnd;
            var da=a-s.Start;var db=b-s.End;if(da.y==0&&db.y==0)continue;
            // This publishes one pre-solve prior at an exact source identity,
            // not a spatial weld or a verified native boundary mutation. Move
            // its attached handle equally so the prior's end slope is retained.
            result[i]=s with{Start=a,End=b,Control1=s.CourseControl1+da-s.StartCurveOffset,
                Control2=s.CourseControl2+db-s.EndCurveOffset,FinalDeckCurve=null};published++;
        }
        log.Count("Structural intent incident sections synchronized",published);
    }

    private static (PreparedSegment[] Source,HashSet<int> Reversed) OrientRailRuns(IReadOnlyList<PreparedSegment> source,CancellationToken token)
    {
        var result=source.ToArray();var reversed=new HashSet<int>();var visited=new HashSet<int>();
        var incident=new Dictionary<SourceNodeKey,List<int>>();
        for(var i=0;i<source.Count;i++)foreach(var node in new[]{source[i].StartNodeId,source[i].EndNodeId})
        {var key=new SourceNodeKey(source[i].Kind,node);if(!incident.TryGetValue(key,out var arms))incident[key]=arms=new();arms.Add(i);}
        bool Eligible(PreparedSegment s)=>s.Kind==NetworkKind.Rail&&!s.OneWay&&s.PairedSpanId==0&&s.PairedProfileId==0;
        foreach(var seed in Enumerable.Range(0,source.Count).OrderBy(i=>source[i].WayId).ThenBy(i=>source[i].StartNodeId))
        {
            if(!Eligible(result[seed])||!visited.Add(seed))continue;
            void Extend(bool backwards)
            {
                var edge=seed;
                while(true)
                {
                    token.ThrowIfCancellationRequested();var current=result[edge];var node=backwards?current.StartNodeId:current.EndNodeId;
                    var arms=incident[new(current.Kind,node)];if(arms.Count!=2||arms[0]==arms[1])break;var next=arms.First(i=>i!=edge);var candidate=result[next];
                    if(visited.Contains(next)||!Eligible(candidate)||candidate.NativeAssetKey!=current.NativeAssetKey||candidate.Layer!=current.Layer||
                        candidate.Bridge!=current.Bridge||candidate.Tunnel!=current.Tunnel||candidate.Classification!=current.Classification)break;
                    // OSM way direction is not a travel restriction on a
                    // bidirectional railway. Normalize only this design view;
                    // restore every source orientation before returning.
                    if((backwards?candidate.EndNodeId:candidate.StartNodeId)!=node)
                    {result[next]=NetworkNormalizer.Reverse(candidate);reversed.Add(next);}
                    visited.Add(next);edge=next;
                }
            }
            Extend(true);Extend(false);
        }
        return(result,reversed);
    }

    private static IReadOnlyList<PreparedSegment> PairedStructureModes(IReadOnlyList<PreparedSegment> source,
        IReadOnlyDictionary<string,NativeAssetContract> assets,ImportLog log,CancellationToken token)
    {
        var output=source.ToArray();var expected=0;var changed=0;var unavailable=0;
        foreach(var group in Enumerable.Range(0,source.Count).Where(i=>source[i].PairedProfileId!=0).GroupBy(i=>source[i].PairedProfileId))
        {
            token.ThrowIfCancellationRequested();var indices=group.ToArray();if(indices.Length!=2)continue;
            var a=source[indices[0]];var b=source[indices[1]];
            if(a.PairedReversed==b.PairedReversed||a.Tunnel||b.Tunnel||a.Bridge==b.Bridge)continue;expected++;
            var index=a.Bridge?indices[1]:indices[0];var ground=source[index];var bridge=a.Bridge?a:b;
            // An absent tag may be inferred within a confirmed formation, but
            // an explicit ground/level policy is not silently rewritten.
            if(!TryPairedBridge(ground,bridge,assets,out var resolved))
            {unavailable++;continue;}
            output[index]=resolved;changed++;
        }
        log.Count("Confirmed paired bridge modes reconciled",changed);
        log.Quality(new("planned","paired-structure-intent","pre-profile",expected-unavailable,0,unavailable,0,0,
            "Matching common stations share bridge intent only with an available native envelope; explicit bridge/level tags remain authoritative"){Expected=expected});
        return output;
    }

    internal static bool TryPairedBridge(PreparedSegment ground,PreparedSegment bridge,
        IReadOnlyDictionary<string,NativeAssetContract> assets,out PreparedSegment resolved)
    {
        resolved=ground;
        if(ground.Bridge||ground.Tunnel||!bridge.Bridge||bridge.Tunnel||ground.RawSourceTags.ContainsKey("bridge")||ground.RawSourceTags.ContainsKey("layer")||
            !TryModeAsset(ground,NativeStructureMode.Elevated,assets,out var asset)||!asset.TryEnvelope(NativeStructureMode.Elevated,out var envelope)||
            Math.Max(ground.NativeWidth,envelope.Width)>ground.MaximumNativeWidth)return false;
        var width=Math.Max(ground.NativeWidth,envelope.Width);
        resolved=ground with{Bridge=true,Layer=bridge.Layer,NativeAssetKey=asset.PaletteKey,NativeLaneOffsets=asset.LaneOffsets,NativeWidth=width,
            NativeCenterlineCurvatureLimit=asset.CenterlineCurvatureLimit(width)};
        return true;
    }

    private static bool TryModeAsset(PreparedSegment segment,NativeStructureMode mode,
        IReadOnlyDictionary<string,NativeAssetContract> assets,out NativeAssetContract asset)
    {
        // Opaque caller-provided keys describe their own mode union. Stock
        // palette keys must follow the same selector used by native placement.
        var ground=NativeAssetKeys.Sized(segment with{Bridge=false});
        var elevated=NativeAssetKeys.Sized(segment with{Bridge=true});
        var key=segment.NativeAssetKey==ground||segment.NativeAssetKey==elevated
            ?(mode==NativeStructureMode.Elevated?elevated:ground):segment.NativeAssetKey;
        return assets.TryGetValue(key,out asset!);
    }

    // This writes only the pre-solve intent. Physical clearance, portal cover
    // and exceptional grades remain obligations of the one joint solver.
    private static IReadOnlyList<PreparedSegment> StructuralProfiles(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token)
    {
        var incident=new Dictionary<SourceNodeKey,List<int>>();
        for(var i=0;i<source.Count;i++)foreach(var id in new[]{source[i].StartNodeId,source[i].EndNodeId})
        {var key=new SourceNodeKey(source[i].Kind,id);if(!incident.TryGetValue(key,out var arms))incident[key]=arms=new();arms.Add(i);}
        var result=source.ToArray();var visited=new HashSet<int>();var runs=0;var steep=0;
        foreach(var seed in Enumerable.Range(0,source.Count).OrderBy(i=>source[i].WayId).ThenBy(i=>source[i].StartNodeId))
        {
            token.ThrowIfCancellationRequested();if(visited.Contains(seed)||!NetworkStructure.IsStructure(source[seed]))continue;
            var path=new LinkedList<int>();path.AddLast(seed);visited.Add(seed);
            void Extend(bool backwards)
            {
                var current=backwards?path.First!.Value:path.Last!.Value;
                while(true)
                {
                    var s=source[current];var node=backwards?s.StartNodeId:s.EndNodeId;var arms=incident[new(s.Kind,node)];
                    if(arms.Count!=2)break;var next=arms.First(i=>i!=current);var n=source[next];
                    if(visited.Contains(next)||n.Bridge!=s.Bridge||n.Tunnel!=s.Tunnel||n.Layer!=s.Layer||n.OneWay!=s.OneWay||
                        n.NativeAssetKey!=s.NativeAssetKey||n.PairedCorridorId!=s.PairedCorridorId||(backwards?n.EndNodeId:n.StartNodeId)!=node)break;
                    visited.Add(next);if(backwards)path.AddFirst(next);else path.AddLast(next);current=next;
                }
            }
            Extend(true);Extend(false);var indices=path.ToArray();var lengths=indices.Select(i=>NetworkVerticalProfiles.ArcLength(source[i])).ToArray();
            var total=lengths.Sum();if(total<1)continue;var y0=source[indices[0]].CourseStart.y;var y1=source[indices[^1]].CourseEnd.y;
            if(Math.Abs(y1-y0)/total>NetworkVerticalProfiles.DesiredGrade(source[seed]))steep++;
            var station=0d;
            for(var k=0;k<indices.Length;k++)
            {
                var i=indices[k];var s=source[i];var a=s.Start;var b=s.End;var c=s.CourseControl1;var d=s.CourseControl2;
                var begin=math.lerp(y0,y1,station/total);var end=math.lerp(y0,y1,(station+lengths[k])/total);
                a.y=(float)begin-s.StartCurveOffset.y;b.y=(float)end-s.EndCurveOffset.y;
                c.y=(float)math.lerp(begin,end,1d/3);d.y=(float)math.lerp(begin,end,2d/3);
                result[i]=s with{Start=a,End=b,Control1=c-s.StartCurveOffset,Control2=d-s.EndCurveOffset,FinalDeckCurve=null};station+=lengths[k];
            }
            runs++;
        }
        log.Count("Whole structural profile intents",runs);
        log.Quality(new("planned","structural-intent-grade","pre-profile",runs,steep,0,0,0,
            "Whole compatible bridge/tunnel runs prefer one constant grade; outer poses fixed; physical solve remains authoritative"));
        return result;
    }

    public static IReadOnlyList<PreparedSegment> FitBudgets(IReadOnlyList<PreparedSegment> source,double width,
        IEnumerable<float3[]> protectedAreas,IEnumerable<float3[]> protectedLines,Func<float3,float?>? water,
        ImportLog log,CancellationToken token)
    {
        var boxes=new List<FootprintLeaf>();var guarded=new HashSet<int>();var blocked=new HashSet<int>();var formations=new HashSet<int>();
        var degree=new Dictionary<SourceNodeKey,int>();
        foreach(var s in source)foreach(var id in new[]{s.StartNodeId,s.EndNodeId})
        {var key=new SourceNodeKey(s.Kind,id);degree[key]=degree.TryGetValue(key,out var count)?count+1:1;}
        var parents=Enumerable.Range(0,source.Count).ToArray();var mainlinePorts=new Dictionary<long,List<int>>();
        int Root(int i){while(parents[i]!=i){parents[i]=parents[parents[i]];i=parents[i];}return i;}
        for(var i=0;i<source.Count;i++)
        {
            var s=source[i];if(s.Kind!=NetworkKind.Road||!s.OneWay||s.ProtectedLink||s.Classification is not ("motorway" or "trunk"))continue;
            foreach(var node in new[]{s.StartNodeId,s.EndNodeId})
            {if(!mainlinePorts.TryGetValue(node,out var entries))mainlinePorts[node]=entries=new();entries.Add(i);}
        }
        foreach(var port in mainlinePorts)
        {
            if(port.Value.Count!=2)continue;var i=port.Value[0];var j=port.Value[1];var a=source[i];var b=source[j];
            var sameRoute=a.WayId==b.WayId||a.CorridorId!=0&&a.CorridorId==b.CorridorId||
                a.Reference.Length>0&&a.Reference==b.Reference||a.Name.Length>0&&a.Name==b.Name;
            if(a.Classification!=b.Classification||!sameRoute||
                (a.StartNodeId==port.Key)==(b.StartNodeId==port.Key))continue;
            var x=Root(i);var y=Root(j);if(x!=y)parents[Math.Max(x,y)]=Math.Min(x,y);
        }
        FootprintLeaf Box(int curve,double2 low,double2 high)=>new(curve,boxes.Count,0,1,low,high,default,default,0,0,0,low,high,true);
        for(var i=0;i<source.Count;i++)
        {
            token.ThrowIfCancellationRequested();var s=source[i];var c=new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);
            var low=math.min(math.min(c.P0.xz,c.P1.xz),math.min(c.P2.xz,c.P3.xz));
            var high=math.max(math.max(c.P0.xz,c.P1.xz),math.max(c.P2.xz,c.P3.xz));
            // Include the complete possible formation displacement in the
            // obstacle broad phase, not merely the old centreline footprint.
            var margin=Math.Max(24,s.NativeWidth/2+24);boxes.Add(Box(i,low-margin,high+margin));
            if(NetworkStructure.IsStructure(s)||s.ProtectedLink||s.Roundabout||s.StartTerminal||s.EndTerminal||
                degree[new(s.Kind,s.StartNodeId)]!=2||degree[new(s.Kind,s.EndNodeId)]!=2||
                math.cmax(math.abs(low))>width/2-16||math.cmax(math.abs(high))>width/2-16)guarded.Add(i);
            if(water!=null)for(var k=0;k<=8;k++)if(water(NetworkNormalizer.Evaluate(s,k/8f)).HasValue){guarded.Add(i);break;}
        }
        foreach(var line in protectedAreas.Concat(protectedLines))
        {
            token.ThrowIfCancellationRequested();if(line.Length==0)continue;
            var low=(double2)line[0].xz;var high=low;
            foreach(var p in line){low=math.min(low,p.xz);high=math.max(high,p.xz);}
            boxes.Add(Box(-1,low-12,high+12));
        }
        var index=new NetworkFootprintIndex(boxes,token);
        var partnerRoots=new Dictionary<int,HashSet<int>>();
        void Partner(int a,int b)
        {a=Root(a);b=Root(b);if(!partnerRoots.TryGetValue(a,out var candidates))partnerRoots[a]=candidates=new();candidates.Add(b);}
        foreach(var pair in index.CandidatePairs(token))
        {
            var ai=boxes[pair.A].Curve;var bi=boxes[pair.B].Curve;
            if(ai<0&&bi<0)continue;
            if(ai>=0&&bi>=0)
            {
                var a=source[ai];var b=source[bi];
                if(a.Kind==b.Kind&&(a.StartNodeId==b.StartNodeId||a.StartNodeId==b.EndNodeId||a.EndNodeId==b.StartNodeId||a.EndNodeId==b.EndNodeId))continue;
                if(Root(ai)==Root(bi))continue;
                if(a.Kind==NetworkKind.Road&&b.Kind==NetworkKind.Road&&a.OneWay&&b.OneWay&&!a.ProtectedLink&&!b.ProtectedLink&&
                    a.Classification is "motorway" or "trunk"&&NetworkPairedCorridors.Related(a,b)&&
                    math.dot(math.normalizesafe(a.End.xz-a.Start.xz),math.normalizesafe(b.End.xz-b.Start.xz))<-.8)
                {formations.Add(ai);formations.Add(bi);Partner(ai,bi);Partner(bi,ai);continue;}
                if(a.Layer!=b.Layer||a.Bridge!=b.Bridge||a.Tunnel!=b.Tunnel)continue;
            }
            if(ai>=0){guarded.Add(ai);blocked.Add(ai);}if(bi>=0){guarded.Add(bi);blocked.Add(bi);}
        }
        var relaxed=0;var output=new PreparedSegment[source.Count];
        foreach(var i in formations)
        {
            var root=Root(i);
            if(!partnerRoots.TryGetValue(root,out var candidates)||candidates.Count!=1||
                !partnerRoots.TryGetValue(candidates.First(),out var reciprocal)||reciprocal.Count!=1||!reciprocal.Contains(root))
                blocked.Add(i);
        }
        for(var i=0;i<source.Count;i++)
        {
            var s=source[i];var budget=NetworkNormalizer.Tolerance(s.Kind);
            if(s.RailTerminalFormationId!=0&&!blocked.Contains(i)||formations.Contains(i)&&!blocked.Contains(i))budget=24;
            else if(!guarded.Contains(i)&&(s.Kind==NetworkKind.Rail||s.Classification is "motorway" or "trunk"))
            {budget=s.Kind==NetworkKind.Rail?4:6;relaxed++;}
            output[i]=s with{SourceFitToleranceMeters=budget};
        }
        log.Diagnostic($"VISUAL FIT POLICY sections={source.Count}; relaxedOpenSections={relaxed}; formationCandidates={formations.Count}; formationBudgetMeters=24; openRoadMeters=6; openRailMeters=4; guardedSections={guarded.Count}; protectedBoundsConservative=true; physicalLimitsUnchanged=true");
        return output;
    }
}
