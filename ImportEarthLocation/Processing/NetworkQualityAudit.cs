using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

public static class NetworkQualityAudit
{
    public static double? DirectionRoundingBudget(PreparedSegment section,bool start)
    {
        var reference=section.HorizontalEmissionReference;if(!reference.HasValue)return null;
        var intended=reference.Value;var p=start?intended.P0:intended.P3;var h=start?intended.P1:intended.P2;
        var actual=start?section.CourseStart:section.CourseEnd;var actualHandle=start?section.CourseControl1:section.CourseControl2;
        var error=math.distance(p.xz,((double3)actual).xz)+math.distance(h.xz,((double3)actualHandle).xz);
        var reach=math.distance(p.xz,h.xz);
        if(!SparseProfileRows.Finite(error)||!SparseProfileRows.Finite(reach)||reach<=0)return null;
        return Math.Asin(Math.Min(1,error/Math.Max(reach-error,1e-5)));
    }

    public static void Planned(IReadOnlyList<PreparedSegment> sections,ImportLog log,CancellationToken token,bool sourceTopologyAvailable=true)
    {
        var gradeFailures=0;var curvatureFailures=0;var curvatureUnknown=0;var finiteFailures=0;var regularityUnknown=0;
        var gradeLength=0d;var curvatureLength=0d;var maximumGrade=0d;var maximumCurvature=0d;
        var incident=new Dictionary<SourceNodeKey,List<(PreparedSegment Section,bool Start)>>();
        foreach(var s in sections)
        {
            token.ThrowIfCancellationRequested();var curve=new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);
            var observation=NetworkCurveObservations.Measure(curve);var finite=observation.Finite;
            if(!finite)finiteFailures++;
            maximumGrade=Math.Max(maximumGrade,observation.SampledMaximumGrade);
            if(s.Kind==NetworkKind.Rail)maximumCurvature=Math.Max(maximumCurvature,observation.SampledMaximumCurvature);
            if(!NetworkVerticalProfiles.WithinGradeCeiling(s,NetworkGeometrySafety.GradeLimit(s))){gradeFailures++;gradeLength+=observation.SampledLengthMeters;}
            var limit=s.NativeCenterlineCurvatureLimit;
            if(s.Kind==NetworkKind.Rail&&!limit.HasValue)curvatureUnknown++;
            if(finite)
            {
                var certificate=NetworkDesignChecks.Certify(curve,limit??double.PositiveInfinity,double.PositiveInfinity,token:token,maximumLeaves:4096);
                if(certificate.Quality!=DesignQuality.Certified)
                {regularityUnknown++;if(s.Kind==NetworkKind.Rail&&limit.HasValue){curvatureFailures++;curvatureLength+=observation.SampledLengthMeters;}}
            }
            else regularityUnknown++;
            foreach(var start in new[]{true,false})
            {
                var key=new SourceNodeKey(s.Kind,start?s.StartNodeId:s.EndNodeId);
                if(!incident.TryGetValue(key,out var list))incident[key]=list=new();list.Add((s,start));
            }
        }
        var anchorFailures=0;var headingFailures=0;var headingUnknown=0;var gradeJoinFailures=0;var gradeJoinUnknown=0;
        var joins=0;var headingWorst=0d;var gradeWorst=0d;var roundingFailures=0;var roundingUnknown=0;var roundingWorst=0d;
        var joinWitnesses=new List<object>();
        static double Angle(double2 a,double2 b)=>Math.Atan2(Math.Abs(NetworkDesignChecks.Cross(a,b)),math.dot(a,b));
        foreach(var node in incident.Values)
        {
            token.ThrowIfCancellationRequested();var point=node[0].Start?node[0].Section.Start:node[0].Section.End;
            if(node.Any(p=>!math.all((p.Start?p.Section.Start:p.Section.End)==point)))anchorFailures++;
            if(node.Count!=2||node.Any(p=>p.Section.Roundabout))continue;
            var a=node[0];var b=node[1];
            double3 Away((PreparedSegment Section,bool Start) p)=>p.Start?(double3)(p.Section.CourseControl1-p.Section.CourseStart):(double3)(p.Section.CourseControl2-p.Section.CourseEnd);
            var da=Away(a);var db=Away(b);var la=math.length(da.xz);var lb=math.length(db.xz);
            if(!(la>0&&lb>0)){joins++;headingUnknown++;gradeJoinUnknown++;roundingUnknown++;continue;}
            // Only designed through continuations have this contract. A real
            // corner is not assigned an invented common grade or travel axis.
            var dot=math.dot(da.xz/la,db.xz/lb);if(dot>-.9)continue;joins++;
            var angle=Angle(da.xz,-db.xz);var jump=Math.Abs(da.y/la+db.y/lb);
            headingWorst=Math.Max(headingWorst,angle);gradeWorst=Math.Max(gradeWorst,jump);
            var betaA=DirectionRoundingBudget(a.Section,a.Start);var betaB=DirectionRoundingBudget(b.Section,b.Start);double? allowance=null;
            if(betaA.HasValue&&betaB.HasValue)
            {
                allowance=Math.Max(1e-5,betaA.Value+betaB.Value);roundingWorst=Math.Max(roundingWorst,allowance.Value);
                if(allowance.Value>Math.PI/180*.1)roundingFailures++;
                // An excessive budget is a representability failure, not
                // permission to accept any sharp join it could explain.
                if(angle>Math.Min(allowance.Value,Math.PI/180*.1))headingFailures++;
            }
            else
            {
                roundingUnknown++;
                if(angle>Math.PI/180*.1)headingFailures++;
                else if(angle>1e-5)headingUnknown++;
            }
            if(jump>1e-5)gradeJoinFailures++;
            if(log.DiagnosticsEnabled&&joinWitnesses.Count<128&&(angle>1e-5||jump>1e-5||!allowance.HasValue))
                joinWitnesses.Add(new{a.Section.Kind,node=a.Start?a.Section.StartNodeId:a.Section.EndNodeId,
                    wayA=a.Section.WayId,wayB=b.Section.WayId,headingRadians=angle,gradeJump=jump,betaA,betaB,allowance,
                    emittedA=DiagnosticArtifacts.Point(da),emittedB=DiagnosticArtifacts.Point(db),
                    referenceA=DiagnosticArtifacts.Curve(a.Section.HorizontalEmissionReference),referenceB=DiagnosticArtifacts.Curve(b.Section.HorizontalEmissionReference)});
        }
        var paired=0;var pairedFailures=0;var pairedUnknown=0;var pairedWorst=0d;
        foreach(var group in sections.Where(s=>s.PairedProfileId!=0).GroupBy(s=>(s.PairedProfileId,
            Begin:Math.Round(Math.Min(s.PairedReferenceBegin,s.PairedReferenceEnd),6),End:Math.Round(Math.Max(s.PairedReferenceBegin,s.PairedReferenceEnd),6))))
        {
            var pair=group.ToArray();if(pair.Length!=2){pairedUnknown++;continue;}paired++;
            var a=pair[0];var b=pair[1];var ah=new[]{a.CourseStart.y,a.CourseControl1.y,a.CourseControl2.y,a.CourseEnd.y};
            var bh=new[]{b.CourseStart.y,b.CourseControl1.y,b.CourseControl2.y,b.CourseEnd.y};
            if(Math.Sign(a.PairedReferenceEnd-a.PairedReferenceBegin)!=Math.Sign(b.PairedReferenceEnd-b.PairedReferenceBegin))Array.Reverse(bh);
            var difference=Enumerable.Range(0,4).Max(i=>Math.Abs((double)ah[i]-bh[i]));pairedWorst=Math.Max(pairedWorst,difference);
            if(difference>.00001)pairedFailures++;
        }
        void Put(string category,int count,int failed,int unknown,double worst,double length,string evidence)
            =>log.Quality(new("planned",category,"final-design",count,failed,unknown,worst,length,evidence));
        Put("native-safety-finite",sections.Count,finiteFailures,0,finiteFailures,0,"Exact finite controls; ECS safety remains a separate native observation");
        Put("regularity",sections.Count,0,regularityUnknown,0,0,"Bounded continuous derivative certificate; exhausted/uncertified is unavailable, not passed");
        Put("exceptional-grade",sections.Count,gradeFailures,0,maximumGrade,gradeLength,"Continuous grade ceiling; worst value and affected length are sampled observations");
        Put("rail-curvature",sections.Count(s=>s.Kind==NetworkKind.Rail),curvatureFailures,curvatureUnknown,maximumCurvature,curvatureLength,"Actual selected rail contract; absent contracts unavailable");
        Put("canonical-anchors",incident.Count,anchorFailures,0,anchorFailures,0,"Bit-identical source node anchors; deck offsets are separate");
        Put("continuation-heading",joins,headingFailures,headingUnknown,headingWorst,0,"Final float through-join radians; max(1e-5,betaA+betaB); absent references cannot justify the native 0.1-degree allowance");
        Put("float-direction-representability",joins,roundingFailures,roundingUnknown,roundingWorst,0,"Actual intended/emitted XZ endpoint and handle errors; budget above 0.1 degree requires review, never a relaxed heading limit");
        Put("continuation-grade",joins,gradeJoinFailures,gradeJoinUnknown,gradeWorst,0,"Outward physical-grade sum; through continuations only; stationary ports unavailable");
        Put("paired-height",paired,pairedFailures,pairedUnknown,pairedWorst,0,"Maximum corresponding height-control difference; exact common reference interval");
        Put("source-topology",sections.Count,(int)(log.Metric("Missing mandatory selected-source ports")+log.Metric("Unresolved selected-source reachability obligations")),sourceTopologyAvailable?0:1,0,0,"Exact selected-source aliases and directional reachability, not proximity");
        if(log.DiagnosticsEnabled)log.DiagnosticRecords("planned-continuation-witnesses.ndjson",joinWitnesses,critical:true);
    }
}
