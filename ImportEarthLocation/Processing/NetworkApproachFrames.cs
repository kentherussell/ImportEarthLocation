using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Resolve both ends of a degree-two approach together. This replaces direction
// displacement fields; lateral lane tapers retain their one station function.
public static class NetworkApproachFrames
{
    public static IReadOnlyList<PreparedSegment> Prepare(IReadOnlyList<PreparedSegment> source,
        IReadOnlyList<NetworkApproachTapers.DirectionRequest> requests,ImportLog log,CancellationToken token,ISet<int>? failedSections=null)
    {
        if(requests.Count==0)return source;
        var result=source.ToArray();var incident=new Dictionary<SourceNodeKey,List<(int Index,bool Start)>>();
        var directions=new Dictionary<(int Index,bool Start),double2>();var conflicts=new HashSet<(int,bool)>();
        var parents=Enumerable.Range(0,source.Count).ToArray();
        int Root(int i){while(parents[i]!=i){parents[i]=parents[parents[i]];i=parents[i];}return i;}
        void Union(int a,int b){a=Root(a);b=Root(b);if(a!=b)parents[Math.Max(a,b)]=Math.Min(a,b);}
        var completed=new HashSet<(int Index,bool Start)>();var affected=new HashSet<int>();
        foreach(var request in requests)
        {
            var key=(request.Index,request.Start);var direction=math.normalizesafe(request.Outward);
            if(math.lengthsq(direction)<.5)continue;
            if(directions.TryGetValue(key,out var old)&&math.dot(old,direction)<1-1e-6)conflicts.Add(key);
            else directions[key]=direction;
        }
        foreach(var key in conflicts)directions.Remove(key);
        for(var i=0;i<source.Count;i++)foreach(var start in new[]{true,false})
        {
            var s=source[i];var key=new SourceNodeKey(s.Kind,start?s.StartNodeId:s.EndNodeId);
            if(!incident.TryGetValue(key,out var entries))incident[key]=entries=new();entries.Add((i,start));
        }
        // Every branch sharing a requested junction is one publication unit.
        // A failed turnout arm may not leave its successful siblings warped.
        foreach(var group in requests.GroupBy(r=>new SourceNodeKey(source[r.Index].Kind,r.Start?source[r.Index].StartNodeId:source[r.Index].EndNodeId)))
        {var first=group.First().Index;foreach(var request in group)Union(first,request.Index);}
        var mandatory=new HashSet<SourceNodeKey>(source.SelectMany(s=>s.SourceNodeAliases.Where(a=>a.Mandatory).Select(a=>new SourceNodeKey(s.Kind,a.CanonicalNodeId))));
        Cubic3d Curve(PreparedSegment s)=>s.Kind==NetworkKind.Rail?new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd):
            new(s.Start,s.Control1??math.lerp(s.Start,s.End,1f/3),s.Control2??math.lerp(s.Start,s.End,2f/3),s.End);
        var lengths=source.Select(s=>NetworkDesignChecks.HorizontalArcLength(Curve(s))).ToArray();
        var used=new HashSet<int>();var rebuilt=0;var unresolved=conflicts.Count;
        foreach(var request in requests.OrderBy(r=>source[r.Index].Kind).ThenBy(r=>source[r.Index].WayId).ThenBy(r=>r.Start?0:1))
        {
            if(used.Contains(request.Index)||!directions.ContainsKey((request.Index,request.Start)))continue;
            token.ThrowIfCancellationRequested();var path=new List<(int Index,bool Forward)>();var seen=new HashSet<int>();
            var edge=request.Index;var forward=request.Start;var distance=0d;
            while(seen.Add(edge)&&!used.Contains(edge))
            {
                var s=source[edge];path.Add((edge,forward));distance+=lengths[edge];
                affected.Add(edge);Union(request.Index,edge);
                var key=new SourceNodeKey(s.Kind,forward?s.EndNodeId:s.StartNodeId);var arms=incident[key];
                if(directions.ContainsKey((edge,!forward))||arms.Count!=2||mandatory.Contains(key)||distance>=500||s.PairedProfileId!=0)break;
                var next=arms.First(p=>p.Index!=edge);var other=source[next.Index];
                if(other.Kind!=s.Kind||other.Roundabout||other.OneWay!=s.OneWay||other.ProtectedLink!=s.ProtectedLink||
                    other.PairedProfileId!=0||other.Tunnel!=s.Tunnel||other.Bridge!=s.Bridge||other.NativeAssetKey!=s.NativeAssetKey||s.OneWay&&next.Start!=forward)break;
                edge=next.Index;forward=next.Start;
            }
            if(path.Count==0)continue;
            // A request from the opposite junction is included when the paths
            // meet. It is not later added as a competing deformation field.
            var first=path[0];var last=path[path.Count-1];var ca=Curve(source[first.Index]);var cb=Curve(source[last.Index]);
            var a=first.Forward?ca.P0:ca.P3;var b=last.Forward?cb.P3:cb.P0;
            var ta=directions.TryGetValue((first.Index,first.Forward),out var ad)?ad:
                math.normalizesafe((first.Forward?ca.P1-ca.P0:ca.P2-ca.P3).xz);
            var tb=directions.TryGetValue((last.Index,!last.Forward),out var bd)?-bd:
                math.normalizesafe((last.Forward?cb.P3-cb.P2:cb.P0-cb.P1).xz);
            var budget=source[first.Index].ProtectedLink?48:NetworkNormalizer.Tolerance(source[first.Index].Kind);
            var parts=new Cubic3d[path.Count];
            bool FitRange(int begin,int end,double3 start,double3 finish,double2 incoming,double2 outgoing)
            {
                token.ThrowIfCancellationRequested();var observations=new List<float3>();var stations=new List<double>{0};var length=0d;
                for(var k=begin;k<end;k++)
                {
                    var item=path[k];var curve=Curve(source[item.Index]);
                    for(var j=0;j<=32;j++)
                    {
                        var point=(float3)NetworkDesignChecks.Evaluate(curve,item.Forward?j/32d:1-j/32d);
                        if(observations.Count>0){var step=math.distance(observations[observations.Count-1].xz,point.xz);if(step<1e-8)continue;length+=step;}
                        observations.Add(point);
                    }
                    stations.Add(length);
                }
                var valid=NetworkSourceFit.TryFit(observations,start,finish,incoming,outgoing,budget,token,out var fitted,out _);
                if(valid&&source[first.Index].Kind==NetworkKind.Rail)
                {
                    var limits=path.Skip(begin).Take(end-begin).Select(p=>source[p.Index].NativeCenterlineCurvatureLimit).ToArray();
                    if(limits.All(k=>k.HasValue))valid=NetworkDesignChecks.Certify(fitted,limits.Min(k=>k!.Value),double.PositiveInfinity,token:token,maximumLeaves:4096).Quality==DesignQuality.Certified;
                    else unresolved++;
                }
                if(valid&&length>0)
                {
                    for(var k=begin;k<end;k++)
                    {
                        var lo=stations[k-begin]/length;var hi=stations[k-begin+1]/length;if(!(hi>lo))return false;
                        var part=NetworkDesignChecks.Split(fitted,hi).Left;if(lo>0)part=NetworkDesignChecks.Split(part,lo/hi).Right;
                        parts[k]=part;
                    }
                    return true;
                }
                if(end-begin<2)return false;
                // A genuine bend can require more than one simple cubic. Split
                // only at existing optional stations, with one shared pose;
                // never subdivide a bad fit while claiming it became accurate.
                var middle=begin+1;for(var k=begin+2;k<end;k++)if(Math.Abs(stations[k-begin]-length/2)<Math.Abs(stations[middle-begin]-length/2))middle=k;
                var left=path[middle-1];var right=path[middle];var lc=Curve(source[left.Index]);var rc=Curve(source[right.Index]);
                var join=left.Forward?lc.P3:lc.P0;
                var ld=left.Forward?lc.P3-lc.P2:lc.P0-lc.P1;var rd=right.Forward?rc.P1-rc.P0:rc.P2-rc.P3;
                var direction=math.normalizesafe(math.normalizesafe(ld.xz)+math.normalizesafe(rd.xz));
                if(math.lengthsq(direction)==0)return false;
                return FitRange(begin,middle,start,join,incoming,direction)&&FitRange(middle,end,join,finish,direction,outgoing);
            }
            var valid=FitRange(0,path.Count,a,b,ta,tb);
            if(!valid){unresolved++;continue;}
            var children=new List<(int Index,PreparedSegment Section)>();
            for(var j=0;j<path.Count;j++)
            {
                var item=path[j];var s=source[item.Index];var part=parts[j];
                if(!item.Forward)part=new(part.P3,part.P2,part.P1,part.P0);
                var startOffset=s.Kind==NetworkKind.Rail?s.StartCurveOffset:default;
                var endOffset=s.Kind==NetworkKind.Rail?s.EndCurveOffset:default;
                var revised=s with{Start=(float3)part.P0-startOffset,Control1=(float3)part.P1-startOffset,
                    Control2=(float3)part.P2-endOffset,End=(float3)part.P3-endOffset,FinalDeckCurve=null,HorizontalParent=null,HorizontalEmissionReference=part};
                // The original geographic observations remain the authority.
                if(s.SourceGeometry.Count>=2&&!NetworkNormalizer.WithinTolerance(revised,(float)budget)){valid=false;break;}
                children.Add((item.Index,revised));
            }
            if(!valid){unresolved++;continue;}
            foreach(var child in children){result[child.Index]=child.Section;used.Add(child.Index);}
            foreach(var item in path)foreach(var start in new[]{true,false})if(directions.ContainsKey((item.Index,start)))completed.Add((item.Index,start));
            rebuilt++;
        }
        var failedRoots=new HashSet<int>(requests.Where(r=>!completed.Contains((r.Index,r.Start))).Select(r=>Root(r.Index)));
        var rolledBack=0;
        foreach(var index in affected)if(failedRoots.Contains(Root(index))){result[index]=source[index];rolledBack++;}
        foreach(var index in affected.Concat(requests.Select(r=>r.Index)).Distinct())if(failedRoots.Contains(Root(index)))failedSections?.Add(index);
        log.Count("Whole approach frames reconstructed",rebuilt);log.Count("Whole approach frames requiring review",unresolved);
        log.Quality(new("planned","joint-approach-frame","approaches-"+source[requests[0].Index].Kind,requests.Count,
            requests.Count(r=>failedRoots.Contains(Root(r.Index))),0,failedRoots.Count,0,"Connected requested junction fans publish atomically; source ports and actual rail curvature contracts retained"));
        log.Diagnostic($"APPROACH FRAMES rebuilt={rebuilt}; unresolved={unresolved}; failedJointGroups={failedRoots.Count}; rolledBackSections={rolledBack}; jointPublication=true; fixedOuterFrames=true; sourceBudgetRoad=2; sourceBudgetRail=3; protectedLinkBudget=48; directionFieldsAdded=false; nativeSafetyNotGeometryCertification=true");
        return result;
    }
}
