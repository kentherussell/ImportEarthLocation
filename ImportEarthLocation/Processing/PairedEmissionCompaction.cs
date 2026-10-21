using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Used only by the paired owner before its final publication certificates.
// Both carriageways accept or retain a boundary together; no independent side
// simplifier is allowed to change the paired station/family correspondence.
internal static class PairedEmissionCompaction
{
    public static List<PreparedSegment> Prepare(List<PreparedSegment> source,IReadOnlyList<PreparedSegment> original,
        ImportLog log,CancellationToken token,out Dictionary<long,long> profiles,double width=double.PositiveInfinity)
    {
        profiles=new();var incidence=new Dictionary<SourceNodeKey,int>();
        foreach(var s in source)foreach(var id in new[]{s.StartNodeId,s.EndNodeId})
        {var key=new SourceNodeKey(s.Kind,id);incidence[key]=incidence.TryGetValue(key,out var n)?n+1:1;}
        var required=new HashSet<SourceNodeKey>(SourceTopologyObligations.Capture(original).Ports);
        foreach(var s in original)foreach(var alias in s.SourceNodeAliases)if(alias.Mandatory)required.Add(new(s.Kind,alias.CanonicalNodeId));
        var output=source.ToArray();var removed=new bool[source.Count];var deviations=new double[source.Count];var attempts=0;var merged=0;
        bool Stop(string reason){log.Count("Paired emission precondition: "+reason);return false;}
        bool Join(PreparedSegment a,PreparedSegment b)
        {
            if(a.EndNodeId!=b.StartNodeId||!math.all(a.End==b.Start)||!math.all(a.CourseEnd==b.CourseStart))return Stop("anchor discontinuity");
            if(incidence[new(a.Kind,a.EndNodeId)]!=2)return Stop("branch");
            if(required.Contains(new(a.Kind,a.EndNodeId)))return Stop("required port");
            if(a.NativeUnitId>=0||b.NativeUnitId>=0)return Stop("existing native ownership");
            if(NetworkBoundaryPolicy.EndpointMarker(a,false)||NetworkBoundaryPolicy.EndpointMarker(b,true))return Stop("endpoint marker");
            if(!NativeChainCompactor.Compatible(a,b,requireZeroOffsets:false)||a.PairedProfileOnly!=b.PairedProfileOnly)return Stop("semantic composition or pairing");
            return true;
        }
        foreach(var corridor in Enumerable.Range(0,source.Count).Where(i=>source[i].PairedCorridorId!=0).GroupBy(i=>source[i].PairedCorridorId))
        {
            var pairs=corridor.GroupBy(i=>source[i].PairedSpanId).Select(g=>g.ToArray())
                .OrderBy(g=>Math.Min(source[g[0]].PairedReferenceBegin,source[g[0]].PairedReferenceEnd)).ToArray();
            if(pairs.Any(p=>p.Length!=2||source[p[0]].PairedReversed==source[p[1]].PairedReversed))continue;
            var left=-1;var right=-1;
            foreach(var pair in pairs)
            {
                token.ThrowIfCancellationRequested();var a=pair.First(i=>!source[i].PairedReversed);var b=pair.First(i=>source[i].PairedReversed);
                if(left>=0&&Math.Abs(output[left].PairedReferenceEnd-source[a].PairedReferenceBegin)<1e-7&&
                    Join(output[left],source[a])&&Join(source[b],output[right]))
                {
                    attempts++;
                    if(TryMerge(output[left],source[a],deviations[left],0,width,token,out var forward,out var forwardError)&&
                        TryMerge(source[b],output[right],0,deviations[right],width,token,out var reverse,out var reverseError)&&
                        NetworkPairedCorridors.Separated(Curve(forward),Curve(reverse),(forward.NativeWidth+reverse.NativeWidth)/2+2,token))
                    {
                        var profile=output[left].PairedProfileId;profiles[source[a].PairedProfileId]=profile;
                        output[left]=forward with{PairedSpanId=output[left].PairedSpanId,PairedProfileId=profile};
                        output[right]=reverse with{PairedSpanId=output[left].PairedSpanId,PairedProfileId=profile};
                        deviations[left]=forwardError;deviations[right]=reverseError;
                        removed[a]=removed[b]=true;merged++;continue;
                    }
                }
                left=a;right=b;
            }
        }
        log.Count("Paired emission parent candidates rejected",attempts-merged);
        log.Diagnostic($"PAIRED EMISSION COMPACTION inputSections={source.Count}; outputSections={source.Count-2*merged}; pairedBoundariesRemoved={merged}; candidateAttempts={attempts}; rejectedParentFitOrGeometry={attempts-merged}; maximumSpanMeters=400; maximumCurveDeviationMeters=0.25; synchronized=true; mandatoryPortsPreserved=true; finalFloatSeparationCertified=true");
        return output.Where((_,i)=>!removed[i]).ToList();
    }

    private static Cubic3d Curve(PreparedSegment s)=>new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);
    private static bool TryMerge(PreparedSegment a,PreparedSegment b,double previousA,double previousB,double width,CancellationToken token,
        out PreparedSegment merged,out double deviation)
    {
        merged=a;deviation=0;var first=Math.Abs(a.PairedReferenceEnd-a.PairedReferenceBegin);var second=Math.Abs(b.PairedReferenceEnd-b.PairedReferenceBegin);
        var length=first+second;if(!(first>0&&second>0&&length<=400))return false;
        var ca=Curve(a);var cb=Curve(b);var t=first/length;
        var intended=new Cubic3d(ca.P0,ca.P0+(ca.P1-ca.P0)/t,cb.P3+(cb.P2-cb.P3)/(1-t),cb.P3);
        var c=new Cubic3d((double3)(float3)intended.P0,(double3)(float3)intended.P1,(double3)(float3)intended.P2,(double3)(float3)intended.P3);
        if(!NetworkDesignChecks.Finite(c))return false;
        if(math.cmax(math.abs(c.P0.xz))>width/2||math.cmax(math.abs(c.P1.xz))>width/2||
            math.cmax(math.abs(c.P2.xz))>width/2||math.cmax(math.abs(c.P3.xz))>width/2)return false;
        var axis=math.normalizesafe(c.P3.xz-c.P0.xz);
        if(Math.Min(math.dot((c.P1-c.P0).xz,axis),Math.Min(math.dot((c.P2-c.P1).xz,axis),math.dot((c.P3-c.P2).xz,axis)))<=1e-10)return false;
        var parts=NetworkDesignChecks.Split(c,t);
        static double Error(Cubic3d x,Cubic3d y)=>Math.Max(Math.Max(math.distance(x.P0.xz,y.P0.xz),math.distance(x.P1.xz,y.P1.xz)),
            Math.Max(math.distance(x.P2.xz,y.P2.xz),math.distance(x.P3.xz,y.P3.xz)));
        // Convex-hull bound on the difference of cubics in the original common
        // station parameter. It includes float conversion, not just samples.
        deviation=Math.Max(Error(parts.Left,ca)+previousA,Error(parts.Right,cb)+previousB);
        if(deviation>.25)return false;
        var references=a.SourceGeometry.Concat(b.SourceGeometry).ToArray();
        merged=a with{EndNodeId=b.EndNodeId,End=b.End,EndElevation=b.EndElevation,EndTerminal=b.EndTerminal,
            EndMiniRoundabout=b.EndMiniRoundabout,EndRoundaboutRadius=b.EndRoundaboutRadius,EndCurveOffset=b.EndCurveOffset,
            Control1=(float3)c.P1-a.StartCurveOffset,Control2=(float3)c.P2-b.EndCurveOffset,
            FinalDeckCurve=new((float3)c.P0,(float3)c.P1,(float3)c.P2,(float3)c.P3),HorizontalEmissionReference=intended,
            HorizontalParent=null,ParentStartT=0,ParentEndT=1,
            SourceGeometry=references,OriginalGeometry=a.OriginalGeometry.Concat(b.OriginalGeometry).ToArray(),
            SourceWayIds=a.SourceWayIds.Append(a.WayId).Concat(b.SourceWayIds.Append(b.WayId)).Distinct().ToArray(),
            SourceNodeAliases=a.SourceNodeAliases.Concat(b.SourceNodeAliases).Distinct().ToArray(),
            SourceFitToleranceMeters=Math.Min(a.SourceFitToleranceMeters??24,b.SourceFitToleranceMeters??24),
            MaximumNativeWidth=Math.Min(a.MaximumNativeWidth,b.MaximumNativeWidth),ExplicitLanes=a.ExplicitLanes||b.ExplicitLanes,
            PairedReferenceEnd=b.PairedReferenceEnd,ReferenceSpan=length,EndHeightFamily=b.EndHeightFamily,
            StartReferenceSpeed=3*math.distance(c.P0.xz,c.P1.xz)/length,EndReferenceSpeed=3*math.distance(c.P2.xz,c.P3.xz)/length};
        token.ThrowIfCancellationRequested();
        return references.Length>=2&&NetworkNormalizer.WithinTolerance(merged)&&
            NetworkDesignChecks.HorizontalArcLength(c)<=400&&(!merged.NativeCenterlineCurvatureLimit.HasValue||
            NetworkDesignChecks.Certify(c,merged.NativeCenterlineCurvatureLimit.Value,double.PositiveInfinity,token:token,maximumLeaves:4096).Quality==DesignQuality.Certified);
    }
}
