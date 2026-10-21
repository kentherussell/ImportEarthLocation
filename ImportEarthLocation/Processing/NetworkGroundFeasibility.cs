using System;
using System.Collections.Generic;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Necessary ground endpoint/chain conditions only: no optimizer or terrain edits.
// If it fails, no profile on this fixed XZ curve can satisfy both the ground
// cut/fill/support budget and the physical grade limit. A pass is not sufficiency.
public static class NetworkGroundFeasibility
{
    public readonly record struct Report(int Checked,int Conflicts,int Unavailable,double MaximumDeficitMeters);
    public static Report Check(IReadOnlyList<PreparedSegment> sections,ushort[] terrain,float scale,double width,
        CancellationToken token,Action<object>? witness=null,Action<int>? conflictSection=null,bool propagateChains=true)
    {
        var checkedCount=0;var conflicts=0;var unavailable=0;var maximum=0d;
        var bounds=propagateChains?new (double LowA,double HighA,double LowB,double HighB,double Reach)?[sections.Count]:
            Array.Empty<(double LowA,double HighA,double LowB,double HighB,double Reach)?>();
        var incidence=new Dictionary<SourceNodeKey,List<int>>();var conflicted=new HashSet<int>();
        for(var i=0;propagateChains&&i<sections.Count;i++)foreach(var node in new[]{sections[i].StartNodeId,sections[i].EndNodeId})
        {var key=new SourceNodeKey(sections[i].Kind,node);if(!incidence.TryGetValue(key,out var edges))incidence[key]=edges=new();edges.Add(i);}
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
            if(propagateChains)bounds[i]=(a-budget-epsilon-s.StartCurveOffset.y,a+budget+epsilon-s.StartCurveOffset.y,
                b-budget-epsilon-s.EndCurveOffset.y,b+budget+epsilon-s.EndCurveOffset.y,grade*upperLength);
            if(deficit<=1e-5)continue;conflicts++;maximum=Math.Max(maximum,deficit);
            conflicted.Add(i);
            conflictSection?.Invoke(i);
            witness?.Invoke(new{section=i,s.WayId,s.StartNodeId,s.EndNodeId,groundStartMeters=a,groundEndMeters=b,
                kind=s.Kind.ToString(),sourceClassification=s.Classification,s.RawSourceTags,s.NativeAssetKey,
                horizontalArcUpperBoundMeters=upperLength,cutFillBudgetMeters=budget,supportToleranceMeters=epsilon,
                gradeLimit=grade,deficitMeters=deficit,classification="fixed-ground-mode-endpoint-incompatibility",
                solverConvergenceCannotResolve=true,placementBlocked=false});
        }
        if(!propagateChains)return new(checkedCount,conflicts,unavailable,maximum);
        // Two bounded passes on compatible degree-two ground runs. Intervals
        // are node-anchor heights: course Y offsets are accounted for exactly.
        // The control polygon is an upper arc bound, so these are necessary
        // conditions, not terrain coverage, cover or clearance certificates.
        var visited=new bool[sections.Count];
        int Next(int edge,long node)
        {
            var a=sections[edge];if(!incidence.TryGetValue(new(a.Kind,node),out var edges)||edges.Count!=2)return -1;
            var next=edges[0]==edge?edges[1]:edges[0];var b=sections[next];
            if(!bounds[next].HasValue||NetworkBoundaryPolicy.RequiresProfileBoundary(NetworkBoundaryPolicy.Classify(a,a.StartNodeId==node,b,b.StartNodeId==node))||
                a.OneWay&&(a.StartNodeId==node)==(b.StartNodeId==node))return -1;
            return next;
        }
        for(var seed=0;seed<sections.Count;seed++)
        {
            if(visited[seed]||!bounds[seed].HasValue)continue;token.ThrowIfCancellationRequested();
            var first=seed;var node=sections[first].StartNodeId;var back=new HashSet<int>{first};
            while(true)
            {
                var next=Next(first,node);if(next<0||visited[next]||!back.Add(next))break;
                node=sections[next].StartNodeId==node?sections[next].EndNodeId:sections[next].StartNodeId;first=next;
            }
            var path=new List<(int Index,bool Forward)>();var current=first;var startNode=node;
            while(!visited[current])
            {
                visited[current]=true;var s=sections[current];var forward=s.StartNodeId==node;path.Add((current,forward));
                node=forward?s.EndNodeId:s.StartNodeId;var next=Next(current,node);if(next<0)break;current=next;
            }
            if(node==startNode){unavailable+=path.Count;continue;}
            if(path.Count<2)continue;
            foreach(var reverse in new[]{false,true})
            {
                var low=double.NegativeInfinity;var high=double.PositiveInfinity;
                for(var k=0;k<path.Count;k++)
                {
                    token.ThrowIfCancellationRequested();var item=path[reverse?path.Count-1-k:k];var s=sections[item.Index];var v=bounds[item.Index]!.Value;
                    var forward=item.Forward!=reverse;var startLow=forward?v.LowA:v.LowB;var startHigh=forward?v.HighA:v.HighB;
                    low=Math.Max(low,startLow);high=Math.Min(high,startHigh);
                    var shift=forward?s.StartCurveOffset.y-s.EndCurveOffset.y:s.EndCurveOffset.y-s.StartCurveOffset.y;
                    var endLow=forward?v.LowB:v.LowA;var endHigh=forward?v.HighB:v.HighA;
                    if(low<=high+1e-5)
                    {low=Math.Max(endLow,low+shift-v.Reach);high=Math.Min(endHigh,high+shift+v.Reach);}
                    if(low<=high+1e-5)continue;
                    var deficit=low-high;maximum=Math.Max(maximum,deficit);
                    if(conflicted.Add(item.Index))
                    {
                        conflicts++;conflictSection?.Invoke(item.Index);
                        witness?.Invoke(new{section=item.Index,s.WayId,s.StartNodeId,s.EndNodeId,deficitMeters=deficit,
                            classification="fixed-ground-mode-chain-incompatibility",compatibleSections=path.Count,
                            necessaryConditionOnly=true,solverConvergenceCannotResolve=true,placementBlocked=false});
                    }
                    break;
                }
            }
        }
        return new(checkedCount,conflicts,unavailable,maximum);
    }
}
