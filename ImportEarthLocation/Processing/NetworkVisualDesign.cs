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
        IReadOnlyDictionary<string,NativeAssetContract>? assets,ImportLog log,CancellationToken token)
    {
        if(assets==null)
        {log.Quality(new("planned","structure-intent","pre-profile",0,0,1,0,0,"Native mode contracts unavailable; no inferred structural changes"));return source;}
        var incident=new Dictionary<SourceNodeKey,List<int>>();
        for(var i=0;i<source.Count;i++)foreach(var id in new[]{source[i].StartNodeId,source[i].EndNodeId})
        {var key=new SourceNodeKey(source[i].Kind,id);if(!incident.TryGetValue(key,out var arms))incident[key]=arms=new();arms.Add(i);}
        bool Eligible(PreparedSegment s)=>!NetworkStructure.IsStructure(s)&&!s.ProtectedLink&&!s.Roundabout&&
            (s.Kind==NetworkKind.Rail||s.Classification is "motorway" or "trunk")&&!s.RawSourceTags.ContainsKey("layer")&&
            !s.RawSourceTags.ContainsKey("bridge")&&!s.RawSourceTags.ContainsKey("tunnel")&&s.PairedProfileId==0;
        var output=source.ToArray();var visited=new bool[source.Count];var bridges=0;var tunnels=0;var unresolved=0;var checkedIntervals=0;
        foreach(var seed in Enumerable.Range(0,source.Count).OrderBy(i=>source[i].WayId).ThenBy(i=>source[i].StartNodeId))
        {
            token.ThrowIfCancellationRequested();if(visited[seed]||!Eligible(source[seed]))continue;
            var path=new LinkedList<int>();path.AddLast(seed);visited[seed]=true;
            void Extend(bool backwards)
            {
                var edge=backwards?path.First!.Value:path.Last!.Value;
                while(true)
                {
                    var s=source[edge];var node=backwards?s.StartNodeId:s.EndNodeId;var arms=incident[new(s.Kind,node)];
                    if(arms.Count!=2)break;var next=arms.First(i=>i!=edge);var n=source[next];
                    if(visited[next]||!Eligible(n)||n.NativeAssetKey!=s.NativeAssetKey||n.Layer!=s.Layer||n.OneWay!=s.OneWay||
                        (backwards?n.EndNodeId:n.StartNodeId)!=node)break;
                    visited[next]=true;if(backwards)path.AddFirst(next);else path.AddLast(next);edge=next;
                }
            }
            Extend(true);Extend(false);var indices=path.ToArray();var first=source[indices[0]];var last=source[indices[^1]];
            var lengths=indices.Select(i=>NetworkDesignChecks.HorizontalArcLength(new(source[i].CourseStart,source[i].CourseControl1,source[i].CourseControl2,source[i].CourseEnd))).ToArray();
            var total=lengths.Sum();if(total<250||total>3000)continue;
            checkedIntervals++;
            var y0=first.Start.y;var y1=last.End.y;
            if(Math.Abs(y1-y0)>NetworkVerticalProfiles.DesiredGrade(first)*total){unresolved++;continue;}
            var low=0d;var high=0d;var distance=0d;
            for(var k=0;k<indices.Length;k++)
            {
                var s=source[indices[k]];
                for(var j=0;j<=16;j++)
                {
                    var t=j/16d;var p=NetworkNormalizer.Evaluate(s,(float)t);var target=math.lerp(y0,y1,(distance+t*lengths[k])/total);
                    var delta=target-NetworkProcessor.Sample(p.x,p.z,terrain,scale,width);low=Math.Min(low,delta);high=Math.Max(high,delta);
                }
                distance+=lengths[k];
            }
            // Select only a clear valley or crest; mixed terrain remains a
            // reviewed ground design, not a sequence of toggled compositions.
            var threshold=first.Kind==NetworkKind.Rail?4:8;NativeStructureMode? mode=null;
            if(high>threshold&&low>=-1)mode=NativeStructureMode.Elevated;
            else if(low<-threshold&&high<=1)mode=NativeStructureMode.Tunnel;
            if(!mode.HasValue){if(high>threshold||low<-threshold)unresolved++;continue;}
            if(!assets.TryGetValue(first.NativeAssetKey,out var asset)||!asset.TryEnvelope(mode.Value,out var envelope)||
                indices.Any(i=>envelope.Width>source[i].MaximumNativeWidth)){unresolved++;continue;}
            distance=0;
            for(var k=0;k<indices.Length;k++)
            {
                var i=indices[k];var s=source[i];var length=lengths[k];var a=(float)math.lerp(y0,y1,distance/total);var b=(float)math.lerp(y0,y1,(distance+length)/total);
                var start=s.Start;var end=s.End;start.y=a;end.y=b;var c1=s.CourseControl1;var c2=s.CourseControl2;c1.y=math.lerp(a,b,1f/3);c2.y=math.lerp(a,b,2f/3);
                output[i]=s with{Start=start,End=end,Control1=c1-s.StartCurveOffset,Control2=c2-s.EndCurveOffset,FinalDeckCurve=null,
                    Bridge=mode==NativeStructureMode.Elevated,Tunnel=mode==NativeStructureMode.Tunnel,NativeWidth=Math.Max(s.NativeWidth,envelope.Width),
                    NativeCenterlineCurvatureLimit=asset.CenterlineCurvatureLimit(Math.Max(s.NativeWidth,envelope.Width))};
                distance+=length;if(mode==NativeStructureMode.Elevated)bridges++;else tunnels++;
            }
        }
        log.Diagnostic($"STRUCTURE INTENT bridgeSections={bridges}; tunnelSections={tunnels}; unresolvedIntervals={unresolved}; minimumIntervalMeters=250; maximumIntervalMeters=3000; explicitModesPreserved=true; pairedModesPreserved=true; gradeAndCoverStillValidated=true");
        log.Quality(new("planned","structure-intent","pre-profile",checkedIntervals,unresolved,0,0,0,
            "Bounded valley/crest selection; mixed/steep/unavailable or over-width cases retain geometry for review; not a feasibility certificate or native acceptance"));
        return output;
    }

    public static IReadOnlyList<PreparedSegment> FitBudgets(IReadOnlyList<PreparedSegment> source,double width,
        IEnumerable<float3[]> protectedAreas,IEnumerable<float3[]> protectedLines,Func<float3,float?>? water,
        ImportLog log,CancellationToken token)
    {
        var boxes=new List<FootprintLeaf>();var guarded=new HashSet<int>();
        var degree=new Dictionary<SourceNodeKey,int>();
        foreach(var s in source)foreach(var id in new[]{s.StartNodeId,s.EndNodeId})
        {var key=new SourceNodeKey(s.Kind,id);degree[key]=degree.TryGetValue(key,out var count)?count+1:1;}
        FootprintLeaf Box(int curve,double2 low,double2 high)=>new(curve,boxes.Count,0,1,low,high,default,default,0,0,0,low,high,true);
        for(var i=0;i<source.Count;i++)
        {
            token.ThrowIfCancellationRequested();var s=source[i];var c=new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);
            var low=math.min(math.min(c.P0.xz,c.P1.xz),math.min(c.P2.xz,c.P3.xz));
            var high=math.max(math.max(c.P0.xz,c.P1.xz),math.max(c.P2.xz,c.P3.xz));
            var margin=Math.Max(12,s.NativeWidth/2+6);boxes.Add(Box(i,low-margin,high+margin));
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
        foreach(var pair in index.CandidatePairs(token))
        {
            var ai=boxes[pair.A].Curve;var bi=boxes[pair.B].Curve;
            if(ai<0&&bi<0)continue;
            if(ai>=0&&bi>=0)
            {
                var a=source[ai];var b=source[bi];
                if(a.Kind==b.Kind&&(a.StartNodeId==b.StartNodeId||a.StartNodeId==b.EndNodeId||a.EndNodeId==b.StartNodeId||a.EndNodeId==b.EndNodeId))continue;
            }
            if(ai>=0)guarded.Add(ai);if(bi>=0)guarded.Add(bi);
        }
        var relaxed=0;var output=new PreparedSegment[source.Count];
        for(var i=0;i<source.Count;i++)
        {
            var s=source[i];var budget=NetworkNormalizer.Tolerance(s.Kind);
            if(!guarded.Contains(i)&&(s.Kind==NetworkKind.Rail||s.Classification is "motorway" or "trunk"))
            {budget=s.Kind==NetworkKind.Rail?4:6;relaxed++;}
            output[i]=s with{SourceFitToleranceMeters=budget};
        }
        log.Diagnostic($"VISUAL FIT POLICY sections={source.Count}; relaxedOpenSections={relaxed}; openRoadMeters=6; openRailMeters=4; guardedSections={guarded.Count}; protectedBoundsConservative=true; physicalLimitsUnchanged=true");
        return output;
    }
}
