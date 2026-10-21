using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// One set of numeric junction checks, used for final design and qualified
// native readback. Observations never grant ownership or rewrite a junction.
public static class NetworkJunctionQuality
{
    public sealed record Pose(double3 Point,double2 Travel,IReadOnlyList<float> LaneOffsets);
    public readonly record struct Observation(long Node,long ApproachWay,string Category,double? Value,double Limit);
    public static void Audit(IReadOnlyList<PreparedSegment> sections,Func<PreparedSegment,bool,Pose?> observe,
        ImportLog log,CancellationToken token,string stage,string scope,Action<Observation>? observation=null)
    {
        var nodes=sections.Where(s=>s.Kind==NetworkKind.Road&&s.OneWay)
            .SelectMany(s=>new[]{(Section:s,Start:true),(Section:s,Start:false)})
            .GroupBy(p=>p.Start?p.Section.StartNodeId:p.Section.EndNodeId);
        var expected=0;var checkedJoins=0;var missing=0;var datumFailed=0;var headingFailed=0;var rampExpected=0;var rampChecked=0;var rampMissing=0;var rampFailed=0;
        var worstDatum=0d;var worstHeading=0d;var worstAngleError=0d;var records=log.DiagnosticsEnabled?new List<object>():null;
        var positionTolerance=stage=="planned"?.05:.25;
        foreach(var node in nodes)
        {
            token.ThrowIfCancellationRequested();var entries=node.ToArray();
            var main=entries.Where(p=>!p.Section.ProtectedLink&&(p.Section.Classification is "motorway" or "trunk")).ToArray();
            if(main.Length!=2||main.Count(p=>p.Start)!=1)continue;expected++;
            var incoming=main.First(p=>!p.Start);var outgoing=main.First(p=>p.Start);
            var a=observe(incoming.Section,false);var b=observe(outgoing.Section,true);
            var available=a!=null&&b!=null&&a.LaneOffsets.Count>0&&b.LaneOffsets.Count>0&&math.lengthsq(a.Travel)>.5&&math.lengthsq(b.Travel)>.5;
            var heading=available?math.normalizesafe(a!.Travel+b!.Travel):default;
            available=available&&math.lengthsq(heading)>.5;
            if(!available)
            {missing++;observation?.Invoke(new(node.Key,0,"continuing-lane-datum",null,positionTolerance));observation?.Invoke(new(node.Key,0,"mainline-junction-heading",null,1));}
            else
            {
                checkedJoins++;var normal=new double2(heading.y,-heading.x);
                var leftA=a!.Point.xz+new double2(a.Travel.y,-a.Travel.x)*a.LaneOffsets.Min();
                var leftB=b!.Point.xz+new double2(b.Travel.y,-b.Travel.x)*b.LaneOffsets.Min();
                var mismatch=Math.Max(Math.Abs(math.dot(leftA-leftB,normal)),Math.Abs(a.Point.y-b.Point.y));
                var angle=Math.Acos(math.clamp(math.dot(a.Travel,b.Travel),-1,1))*180/Math.PI;
                worstDatum=Math.Max(worstDatum,mismatch);worstHeading=Math.Max(worstHeading,angle);
                observation?.Invoke(new(node.Key,0,"continuing-lane-datum",mismatch,positionTolerance));
                observation?.Invoke(new(node.Key,0,"mainline-junction-heading",angle,1));
                if(mismatch>positionTolerance)datumFailed++;if(angle>1)headingFailed++;
                records?.Add(new{node=node.Key,incoming=incoming.Section.WayId,outgoing=outgoing.Section.WayId,
                    continuingLaneMismatchMeters=mismatch,throughHeadingDegrees=angle,nativeSlots=true});
            }
            foreach(var ramp in entries.Where(p=>p.Section.ProtectedLink))
            {
                // Without actual slot counts, it is unknown whether a new lane
                // exists. Do not infer the merge contract from nominal lanes.
                if(!available){rampExpected++;rampMissing++;observation?.Invoke(new(node.Key,ramp.Section.WayId,"no-added-lane-approach",null,1));continue;}
                var far=ramp.Start?ramp.Section.End:ramp.Section.Start;
                var anchor=ramp.Start?ramp.Section.Start:ramp.Section.End;
                var rightSide=math.dot((double2)(far-anchor).xz,new double2(heading.y,-heading.x))>0;
                var added=rightSide&&(ramp.Start?a!.LaneOffsets.Count>b!.LaneOffsets.Count:b!.LaneOffsets.Count>a!.LaneOffsets.Count);
                if(added)continue;rampExpected++;var pose=observe(ramp.Section,ramp.Start);
                if(pose==null||math.lengthsq(pose.Travel)<.5){rampMissing++;observation?.Invoke(new(node.Key,ramp.Section.WayId,"no-added-lane-approach",null,1));continue;}rampChecked++;
                var angle=Math.Acos(math.clamp(math.dot(heading,pose.Travel),-1,1))*180/Math.PI;var error=Math.Abs(angle-20);
                worstAngleError=Math.Max(worstAngleError,error);if(error>1)rampFailed++;
                observation?.Invoke(new(node.Key,ramp.Section.WayId,"no-added-lane-approach",error,1));
                records?.Add(new{node=node.Key,ramp=ramp.Section.WayId,noAddedLane=true,approachDegrees=angle,errorDegrees=error});
            }
        }
        void Put(string category,int count,int failures,int unavailable,int total,double worst,string meaning)
            =>log.Quality(new(stage,category,scope,count,failures,unavailable,worst,0,meaning){Expected=total});
        Put("continuing-lane-datum",checkedJoins,datumFailed,missing,expected,worstDatum,"Travel-left continuing native slot; lateral and height mismatch; longitudinal native junction trimming excluded");
        Put("mainline-junction-heading",checkedJoins,headingFailed,missing,expected,worstHeading,"Through mainline heading difference <=1 degree");
        Put("no-added-lane-approach",rampChecked,rampFailed,rampMissing,rampExpected,worstAngleError,"Actual slot counts select no-added-lane 20 +/-1 degree contract; missing lineage/palette unavailable");
        if(records!=null)log.DiagnosticCompressedRecords(scope+"-junction-quality.ndjson.gz",records);
    }
}
