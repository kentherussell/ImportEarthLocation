using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Pure numeric work only. Callers must first Clear the owned, unapplied preview
// and observe exact retirement. This method cannot mutate an applied ECS owner.
public static class NativeJunctionReconciliation
{
    public readonly record struct Port(float3 Position,float2 TravelTangent,double Grade,bool AppliedBoundary);
    public sealed record Result(IReadOnlyList<PreparedSegment> Sections,bool NativeSafe,bool NeedsTerrainRestamp,int ChangedSections,DesignQuality Quality)
    { public NetworkRoadbed.Patch? TerrainPatch { get; init; } }
    public static Result Refit(IReadOnlyList<PreparedSegment> immutableUnit,IReadOnlyDictionary<SourceNodeKey,Port> observed,
        ushort[] terrain,float scale,double width,IReadOnlyDictionary<string,NativeAssetContract> assets,ImportLog log,CancellationToken token,
        Func<float3,float?>? water=null,IReadOnlyList<PreparedSegment>? context=null)
    {
        var result=immutableUnit.ToArray();var fixedHeights=new Dictionary<SourceNodeKey,double>();var fixedSlopes=new Dictionary<SourceNodeKey,double>();var changed=0;var restamp=false;
        var degrees=new Dictionary<SourceNodeKey,int>();
        foreach(var s in immutableUnit)foreach(var id in new[]{s.StartNodeId,s.EndNodeId})
        {var key=new SourceNodeKey(s.Kind,id);degrees[key]=degrees.TryGetValue(key,out var old)?old+1:1;}
        foreach(var pair in observed)
        {
            if(!math.all(math.isfinite(pair.Value.Position))||!math.all(math.isfinite(pair.Value.TravelTangent))||!SparseProfileRows.Finite(pair.Value.Grade))
                return new(immutableUnit,false,false,0,DesignQuality.RequiresReview);
        }
        for(var i=0;i<result.Length;i++)
        {
            token.ThrowIfCancellationRequested();var s=immutableUnit[i];var a=s.CourseStart;var b=s.CourseEnd;
            var ta=math.normalizesafe((s.CourseControl1-a).xz);var tb=math.normalizesafe((b-s.CourseControl2).xz);
            var aKey=new SourceNodeKey(s.Kind,s.StartNodeId);var bKey=new SourceNodeKey(s.Kind,s.EndNodeId);
            var aPose=observed.TryGetValue(aKey,out var ap);var bPose=observed.TryGetValue(bKey,out var bp);
            void Fixed(SourceNodeKey key,float3 position,float2 tangent,double grade)
            {
                // A unit's outer approach is a real boundary contract. Interior
                // multi-arm frames need per-branch semantics, not one invented grade.
                if(degrees[key]!=1)return;fixedHeights[key]=position.y;fixedSlopes[key]=grade;
            }
            var slopeA=math.length((s.CourseControl1-a).xz)>1e-8?(s.CourseControl1.y-a.y)/math.length((s.CourseControl1-a).xz):0;
            var slopeB=math.length((b-s.CourseControl2).xz)>1e-8?(b.y-s.CourseControl2.y)/math.length((b-s.CourseControl2).xz):0;
            if(aPose){a=ap.Position+s.StartCurveOffset;ta=ap.TravelTangent;Fixed(aKey,a,ta,ap.Grade);}else Fixed(aKey,a,ta,slopeA);
            if(bPose){b=bp.Position+s.EndCurveOffset;tb=bp.TravelTangent;Fixed(bKey,b,tb,bp.Grade);}else Fixed(bKey,b,tb,slopeB);
            var da=a-s.CourseStart;var db=b-s.CourseEnd;
            var tangentChanged=math.dot(ta,math.normalizesafe((s.CourseControl1-s.CourseStart).xz))<1-1e-8||math.dot(tb,math.normalizesafe((s.CourseEnd-s.CourseControl2).xz))<1-1e-8;
            var gradeChanged=aPose&&Math.Abs(ap.Grade-slopeA)>1e-6||bPose&&Math.Abs(bp.Grade-slopeB)>1e-6;
            var sourceCurve=new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);Cubic3d curve;
            if(Math.Max(math.length(da),math.length(db))<=.1&&!tangentChanged)
                curve=new(a,(double3)s.CourseControl1+da,(double3)s.CourseControl2+db,b);
            else
            {
                var raw=s.SourceGeometry.Count>=2?s.SourceGeometry.ToArray():Enumerable.Range(0,33).Select(j=>(float3)NetworkDesignChecks.Evaluate(sourceCurve,j/32d)).ToArray();
                var observations=new double3[raw.Length];var stations=new double[raw.Length];var support=new double[raw.Length];
                for(var j=0;j<raw.Length;j++)
                {
                    observations[j]=raw[j];if(j>0)stations[j]=stations[j-1]+math.distance(raw[j-1].xz,raw[j].xz);
                }
                var total=stations[stations.Length-1];
                for(var j=0;j<raw.Length;j++)support[j]=((j>0?stations[j]-stations[j-1]:0)+(j+1<raw.Length?stations[j+1]-stations[j]:0))*.5;
                for(var j=0;j<raw.Length;j++)stations[j]=total>0?stations[j]/total:j/(double)(raw.Length-1);
                try{curve=NetworkDesignChecks.FitHandles(a,b,ta,tb,observations,stations,support,1e-5);}
                catch(ArgumentException){log.Diagnostic($"LOCAL REFIT unavailable way={s.WayId}; unrepresentable fixed endpoint/tangent frame; unchanged unit retained");return new(immutableUnit,true,false,0,DesignQuality.RequiresReview);}
            }
            if(!NetworkDesignChecks.Finite(curve))return new(immutableUnit,false,false,0,DesignQuality.RequiresReview);
            var ca=(float3)curve.P1;var cb=(float3)curve.P2;
            result[i]=s with{Start=a-s.StartCurveOffset,End=b-s.EndCurveOffset,Control1=ca-s.StartCurveOffset,Control2=cb-s.EndCurveOffset,
                FinalDeckCurve=null,HorizontalParent=null,HorizontalEmissionReference=curve,PairedSpanId=0,StartHeightFamily=0,EndHeightFamily=0,ReferenceSpan=0};
            if(gradeChanged||tangentChanged||math.distance(a,s.CourseStart)>.001||math.distance(b,s.CourseEnd)>.001||math.distance(ca,s.CourseControl1)>.001||math.distance(cb,s.CourseControl2)>.001)
            {changed++;if(!NetworkStructure.IsStructure(s))restamp=true;}
        }
        if(changed==0)return new(immutableUnit,true,false,0,DesignQuality.RequiresReview);
        // Exactly one local solve. Outer committed and future-neighbour port
        // heights/grades are hard projector contracts, not large penalties.
        var local=new List<PreparedSegment>(result);var frozen=new HashSet<int>();
        if(context!=null&&result.Length>0)
        {
            var own=new HashSet<(NetworkKind,long,long,long)>(immutableUnit.Select(s=>(s.Kind,s.WayId,s.StartNodeId,s.EndNodeId)));
            var low=new float2(float.PositiveInfinity);var high=new float2(float.NegativeInfinity);
            foreach(var s in immutableUnit.Concat(result))foreach(var p in new[]{s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd})
            {low=math.min(low,p.xz);high=math.max(high,p.xz);}
            foreach(var s in context)
            {
                if(own.Contains((s.Kind,s.WayId,s.StartNodeId,s.EndNodeId)))continue;
                var radius=Math.Max(32,s.NativeWidth+16);
                var lo=math.min(math.min(s.CourseStart.xz,s.CourseControl1.xz),math.min(s.CourseControl2.xz,s.CourseEnd.xz));
                var hi=math.max(math.max(s.CourseStart.xz,s.CourseControl1.xz),math.max(s.CourseControl2.xz,s.CourseEnd.xz));
                if(!math.all(hi>=low-radius)||!math.all(lo<=high+radius))continue;
                frozen.Add(local.Count);local.Add(s);
            }
        }
        var coupled=NetworkTunnelProfile.Solve(local,scale,Array.Empty<(int,float,float)>(),log,token,fixedHeights,new(terrain,width,water,assets,CaptureReplay:false,EvidenceScope:"native-refit"),fixedSlopes,frozen);
        var solved=coupled.Take(result.Length).ToArray();
        changed=0;restamp=false;
        for(var i=0;i<solved.Length;i++)
        {
            var s=solved[i];var old=immutableUnit[i];
            var controls=new[]{s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd};
            if(controls.Any(p=>!math.all(math.isfinite(p))||math.any(math.abs(p.xz)>(float)(width/2+.02))||p.y<0||p.y>scale))
                return new(immutableUnit,false,false,0,DesignQuality.RequiresReview);
            if(math.distance(s.CourseStart,old.CourseStart)>.001||math.distance(s.CourseControl1,old.CourseControl1)>.001||
                math.distance(s.CourseControl2,old.CourseControl2)>.001||math.distance(s.CourseEnd,old.CourseEnd)>.001)
            {changed++;if(!NetworkStructure.IsStructure(s))restamp=true;}
        }
        log.Diagnostic($"LOCAL REFIT designed sections={solved.Length}; frozenContextSections={frozen.Count}; changed={changed}; needsOwnedTerrainRestamp={restamp}; fixedOuterHeights={fixedHeights.Count}; fixedOuterGrades={fixedSlopes.Count}; sourceSnapshotUnchanged=true; nativeApplyInvocations=0; quality=RequiresReview");
        return new(solved,true,restamp,changed,DesignQuality.RequiresReview);
    }
}
