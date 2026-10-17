using System;
using System.Collections.Generic;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Necessary endpoint condition only: no optimizer, terrain edits or mode changes.
// If it fails, no profile on this fixed XZ curve can satisfy both the ground
// cut/fill/support budget and the physical grade limit. A pass is not sufficiency.
public static class NetworkGroundFeasibility
{
    public readonly record struct Report(int Checked,int Conflicts,int Unavailable,double MaximumDeficitMeters);
    public static Report Check(IReadOnlyList<PreparedSegment> sections,ushort[] terrain,float scale,double width,
        CancellationToken token,Action<object>? witness=null)
    {
        var checkedCount=0;var conflicts=0;var unavailable=0;var maximum=0d;
        var epsilon=Math.Max(.05,2*scale/65535d);
        for(var i=0;i<sections.Count;i++)
        {
            token.ThrowIfCancellationRequested();var s=sections[i];if(NetworkStructure.IsStructure(s))continue;checkedCount++;
            var c=new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);
            if(!NetworkDesignChecks.Finite(c)){unavailable++;continue;}
            var a=NetworkProcessor.Sample((float)c.P0.x,(float)c.P0.z,terrain,scale,width);
            var b=NetworkProcessor.Sample((float)c.P3.x,(float)c.P3.z,terrain,scale,width);
            // The control polygon bounds physical horizontal arc length above.
            // Using it makes a negative finding conservative even for curves.
            var upperLength=math.distance(c.P0.xz,c.P1.xz)+math.distance(c.P1.xz,c.P2.xz)+math.distance(c.P2.xz,c.P3.xz);
            var budget=NetworkVerticalProfiles.CutFillBudget(s);var grade=NetworkGeometrySafety.GradeLimit(s);
            var allowed=2*(budget+epsilon)+grade*upperLength;var deficit=Math.Abs((double)b-a)-allowed;
            if(!SparseProfileRows.Finite(deficit)){unavailable++;continue;}
            if(deficit<=1e-5)continue;conflicts++;maximum=Math.Max(maximum,deficit);
            witness?.Invoke(new{section=i,s.WayId,s.StartNodeId,s.EndNodeId,groundStartMeters=a,groundEndMeters=b,
                horizontalArcUpperBoundMeters=upperLength,cutFillBudgetMeters=budget,supportToleranceMeters=epsilon,
                gradeLimit=grade,deficitMeters=deficit,classification="fixed-ground-mode-endpoint-incompatibility",
                solverConvergenceCannotResolve=true,placementBlocked=false});
        }
        return new(checkedCount,conflicts,unavailable,maximum);
    }
}
