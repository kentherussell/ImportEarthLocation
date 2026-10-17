using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Run before vertical variables are allocated. A stationary horizontal endpoint
// cannot support a free vertical handle without creating a vertical tangent.
public static class NetworkHorizontalRegularity
{
    public static IReadOnlyList<PreparedSegment> Prepare(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token)
    {
        var result=source.ToArray();var corrected=0;var collapsed=0;var reconstructed=0;var unresolved=0;
        for(var i=0;i<result.Length;i++)
        {
            token.ThrowIfCancellationRequested();var s=result[i];
            var a=s.CourseStart;var b=s.CourseControl1;var c=s.CourseControl2;var d=s.CourseEnd;
            var chord=math.distance(a.xz,d.xz);
            if(chord<=1e-6)
            {
                collapsed++;
                // Do not weld distinct mandatory source nodes or fabricate a
                // direction. Native safety decides representability separately.
                continue;
            }
            var changed=false;
            if(math.distance(a.xz,b.xz)<=1e-6){b.xz=math.lerp(a.xz,d.xz,1f/3);changed=true;}
            if(math.distance(c.xz,d.xz)<=1e-6){c.xz=math.lerp(a.xz,d.xz,2f/3);changed=true;}
            var delta=(double2)(d-a).xz;var direction=math.normalizesafe(delta);
            var progress=Math.Min(math.dot((double2)(b-a).xz,direction),Math.Min(math.dot((double2)(c-b).xz,direction),math.dot((double2)(d-c).xz,direction)));
            if(progress<=1e-8&&!s.Roundabout)
            {
                // A straight candidate is preferred only when the complete
                // retained reference supports it. Approximation failure never
                // justifies deleting a real turn or a mandatory source port.
                var straight=s with{Control1=math.lerp(a,d,1f/3)-s.StartCurveOffset,Control2=math.lerp(a,d,2f/3)-s.EndCurveOffset,
                    FinalDeckCurve=new(a,math.lerp(a,d,1f/3),math.lerp(a,d,2f/3),d),HorizontalParent=null,
                    HorizontalEmissionReference=new(a,math.lerp((double3)a,d,1d/3),math.lerp((double3)a,d,2d/3),d)};
                if(NetworkNormalizer.WithinTolerance(straight))
                {result[i]=straight;reconstructed++;continue;}
                unresolved++;
            }
            if(!changed)continue;
            result[i]=s with{Control1=b-s.StartCurveOffset,Control2=c-s.EndCurveOffset,FinalDeckCurve=new(a,b,c,d),HorizontalParent=null,HorizontalEmissionReference=null};
            corrected++;
        }
        log.Count("Stationary horizontal endpoint handles regularized",corrected);
        log.Count("Collapsed horizontal sections requiring native representability review",collapsed);
        log.Count("Nonregular horizontal spans reconstructed",reconstructed);log.Count("Nonregular horizontal spans unresolved",unresolved);
        log.Diagnostic($"HORIZONTAL REGULARITY correctedEndpointHandles={corrected}; collapsedEndpointSections={collapsed}; mandatoryNodesRemoved=0; verticalSolveNotYetRun=true; qualityFailureBlocksPlacement=false");
        return result;
    }
}
