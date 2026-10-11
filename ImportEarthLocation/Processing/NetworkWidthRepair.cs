using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

/// <summary>Incremental, bounded shared-node spacing. No endpoint is invented or removed.</summary>
public static class NetworkWidthRepair
{
    private readonly record struct NodeKey(NetworkKind Kind,long Id);
    private sealed class Curve
    {
        public float2[] Pieces=Array.Empty<float2>(), Samples=Array.Empty<float2>(), Headings=Array.Empty<float2>(), Pushes=Array.Empty<float2>();
        public float[] Overlaps=Array.Empty<float>();
        public float2 Low,High;
        public readonly List<(int,int)> Cells=new();
        public readonly HashSet<int> Neighbors=new();
    }
    public static IReadOnlyList<PreparedSegment> Prepare(IReadOnlyList<PreparedSegment> source,
        Func<PreparedSegment,float> minimumWidth,double mapWidth,ImportLog log,CancellationToken token)
    {
        token.ThrowIfCancellationRequested();if(source.Count==0)return source;
        var timer=Stopwatch.StartNew();var sections=source.ToArray();var half=(float)(mapWidth/2);
        var nodeMap=new Dictionary<NodeKey,int>();var original=new List<float2>();var incident=new List<List<int>>();
        var from=new int[sections.Length];var to=new int[sections.Length];
        int Node(PreparedSegment s,long id,float2 p)
        {
            var key=new NodeKey(s.Kind,id);if(nodeMap.TryGetValue(key,out var index))return index;
            index=original.Count;nodeMap[key]=index;original.Add(p);incident.Add(new List<int>());return index;
        }
        var widths=new float[sections.Length];var curves=new Curve[sections.Length];var eligible=new bool[sections.Length];var maxWidth=0f;
        for(var i=0;i<sections.Length;i++)
        {
            token.ThrowIfCancellationRequested();var s=sections[i];from[i]=Node(s,s.StartNodeId,s.Start.xz);to[i]=Node(s,s.EndNodeId,s.End.xz);
            incident[from[i]].Add(i);incident[to[i]].Add(i);widths[i]=minimumWidth(s);
            eligible[i]=math.isfinite(widths[i])&&widths[i]>0;
            if(eligible[i])maxWidth=Math.Max(maxWidth,widths[i]);curves[i]=new Curve();
        }
        var grid=new Dictionary<(int,int),HashSet<int>>();var active=new HashSet<int>();var candidates=new HashSet<int>();var changed=new HashSet<int>();
        var pairs=new HashSet<(int,int)>();
        var blocked=new bool[original.Count];var shifts=new float2[original.Count];var sums=new float2[original.Count];var counts=new int[original.Count];
        var previous=new float2[original.Count];var totals=new float2[original.Count];var scores=new double[sections.Length];
        long pairChecks=0;var refreshes=0;
        void Refresh(int index)
        {
            var c=curves[index];foreach(var cell in c.Cells)if(grid.TryGetValue(cell,out var bucket)){bucket.Remove(index);if(bucket.Count==0)grid.Remove(cell);}
            c.Cells.Clear();if(!eligible[index])return;var s=sections[index];refreshes++;
            if(c.Samples.Length==0)
            {
                var stations=Math.Max(8,Math.Min(512,(int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(s)/32)));
                c.Samples=new float2[stations+1];c.Headings=new float2[stations+1];c.Overlaps=new float[stations+1];c.Pushes=new float2[stations+1];c.Pieces=new float2[9];
            }
            for(var j=0;j<9;j++)c.Pieces[j]=NetworkNormalizer.Evaluate(s,j/8f).xz;
            for(var j=0;j<c.Samples.Length;j++)
            {
                var t=j/(float)(c.Samples.Length-1);c.Samples[j]=NetworkNormalizer.Evaluate(s,t).xz;
                c.Headings[j]=math.normalizesafe((NetworkNormalizer.Evaluate(s,Math.Min(1,t+.001f))-NetworkNormalizer.Evaluate(s,Math.Max(0,t-.001f))).xz);
            }
            c.Low=math.min(math.min(s.CourseStart.xz,s.CourseControl1.xz),math.min(s.CourseControl2.xz,s.CourseEnd.xz));
            c.High=math.max(math.max(s.CourseStart.xz,s.CourseControl1.xz),math.max(s.CourseControl2.xz,s.CourseEnd.xz));
            for(var z=(int)Math.Floor(c.Low.y/128);z<=(int)Math.Floor(c.High.y/128);z++)for(var x=(int)Math.Floor(c.Low.x/128);x<=(int)Math.Floor(c.High.x/128);x++)
            {var cell=(x,z);if(!grid.TryGetValue(cell,out var bucket))grid[cell]=bucket=new();bucket.Add(index);c.Cells.Add(cell);}
        }
        void Nearby(int index,HashSet<int> output)
        {
            if(!eligible[index])return;var c=curves[index];var radius=(widths[index]+maxWidth)*.5f+1;
            for(var z=(int)Math.Floor((c.Low.y-radius)/128);z<=(int)Math.Floor((c.High.y+radius)/128);z++)
            for(var x=(int)Math.Floor((c.Low.x-radius)/128);x<=(int)Math.Floor((c.High.x+radius)/128);x++)
                if(grid.TryGetValue((x,z),out var bucket))output.UnionWith(bucket);
        }
        for(var i=0;i<sections.Length;i++){Refresh(i);if(eligible[i])active.Add(i);}
        var best=sections.ToArray();var bestScore=double.PositiveInfinity;var referenceScore=double.PositiveInfinity;
        var stalled=0;var fallback=false;var fastPasses=0;var clusterPasses=0;var passes=0;var bestWorst=0f;
        for(;passes<36;passes++)
        {
            token.ThrowIfCancellationRequested();var passTimer=Stopwatch.StartNew();var activeCount=active.Count;
            // Old/new neighbors are dirty. Unchanged curves retain their residuals and contributions.
            pairs.Clear();
            foreach(var i in active.OrderBy(v=>v))
            {
                var c=curves[i];Array.Clear(c.Overlaps,0,c.Overlaps.Length);Array.Clear(c.Pushes,0,c.Pushes.Length);
                c.Neighbors.Clear();candidates.Clear();Nearby(i,candidates);var s=sections[i];
                foreach(var k in candidates.OrderBy(v=>v))
                {
                    var o=sections[k];if(k==i)continue;
                    var oc=curves[k];var separation=(widths[i]+widths[k])*.5f+1;
                    if(math.any(c.High+separation<oc.Low)||math.any(oc.High+separation<c.Low))continue;
                    c.Neighbors.Add(k);pairs.Add((Math.Min(i,k),Math.Max(i,k)));
                }
            }
            void Inspect(int i,int k)
            {
                    var c=curves[i];var oc=curves[k];var s=sections[i];var separation=(widths[i]+widths[k])*.5f+1;
                    for(var j=0;j<c.Samples.Length;j++)
                    {
                        var center=c.Samples[j];var heading=c.Headings[j];var reach=(widths[i]+widths[k])*1.5f+1;
                        var ownPoint=NetworkNormalizer.Evaluate(s,j/(float)(c.Samples.Length-1));
                        if((from[i]==from[k]||from[i]==to[k])&&math.distance(center,s.Start.xz)<reach||
                           (to[i]==from[k]||to[i]==to[k])&&math.distance(center,s.End.xz)<reach)continue;
                        for(var p=0;p<8;p++)
                        {
                            var a=oc.Pieces[p];var delta=oc.Pieces[p+1]-a;
                            if(Math.Abs(math.dot(heading,math.normalizesafe(delta)))<.9f)continue;
                            var t=math.clamp(math.dot(center-a,delta)/Math.Max(.0001f,math.lengthsq(delta)),0,1);
                            var otherPoint=NetworkNormalizer.Evaluate(sections[k],(p+t)/8f);
                            var kind=NetworkContactPolicy.Classify(s,sections[k],ownPoint,otherPoint,heading,delta,false);
                            if(kind!=NetworkContactKind.HorizontalConflict)continue;
                            var difference=center-(a+delta*t);var distance=math.length(difference);var overlap=separation-distance;
                            if(overlap<=c.Overlaps[j])continue;c.Overlaps[j]=overlap;
                            var basis=curves[Math.Min(i,k)].Headings[0];
                            var direction=distance>.1f?difference/distance:new float2(basis.y,-basis.x)*(i<k?1:-1);
                            c.Pushes[j]=direction*Math.Min(12,overlap*.5f);
                        }
                    }
            }
            foreach(var pair in pairs.OrderBy(p=>p.Item1).ThenBy(p=>p.Item2))
            {pairChecks++;if(active.Contains(pair.Item1))Inspect(pair.Item1,pair.Item2);if(active.Contains(pair.Item2))Inspect(pair.Item2,pair.Item1);}
            foreach(var i in active){scores[i]=0;foreach(var overlap in curves[i].Overlaps)scores[i]+=overlap*overlap;}
            var score=scores.Sum();var worst=0f;var failures=0;
            foreach(var c in curves)foreach(var overlap in c.Overlaps){worst=Math.Max(worst,overlap);if(overlap>.05f)failures++;}
            if(score<bestScore){bestScore=score;bestWorst=worst;Array.Copy(sections,best,sections.Length);}
            if(double.IsPositiveInfinity(referenceScore)||score<referenceScore-Math.Max(.01,referenceScore*.002)){stalled=0;referenceScore=score;}else stalled++;
            log.Diagnostic($"PROCESS vanilla-width-progress pass={passes+1}; mode={(fallback?"cluster":"fast")}; activeSections={activeCount}; overlapSamples={failures}; worstOverlapMeters={worst:0.###}; residual={score:0.###}; bestResidual={bestScore:0.###}; stalledPasses={stalled}; elapsedSeconds={timer.Elapsed.TotalSeconds:0.###}");
            if(worst<=.002f)break;
            if(!fallback&&(stalled>=4||fastPasses>=12||active.Count==0))
            {
                fallback=true;stalled=0;referenceScore=double.PositiveInfinity;Array.Copy(best,sections,sections.Length);
                // Restoring a snapshot also restores the cumulative budget origin.
                Array.Clear(totals,0,totals.Length);for(var i=0;i<sections.Length;i++){totals[from[i]]=sections[i].Start.xz-original[from[i]];totals[to[i]]=sections[i].End.xz-original[to[i]];}
                active.Clear();for(var i=0;i<sections.Length;i++){Refresh(i);if(eligible[i])active.Add(i);}
                log.Diagnostic("PROCESS vanilla-width-escalation coordinatedSharedNodes=true; expandedApproaches=true; movementBudgetMeters=48; noConnectionInference=true");continue;
            }
            if(fallback&&(stalled>=6||clusterPasses>=22||active.Count==0))break;
            Array.Clear(sums,0,sums.Length);Array.Clear(counts,0,counts.Length);Array.Clear(shifts,0,shifts.Length);
            void Add(int id,float2 push,float weight){if(blocked[id]||weight<.01f)return;sums[id]+=push*weight*2;counts[id]++;}
            for(var i=0;i<curves.Length;i++){var c=curves[i];for(var j=0;j<c.Pushes.Length;j++)if(c.Overlaps[j]>.002f){var t=j/(float)(c.Pushes.Length-1);Add(from[i],c.Pushes[j],1-t);Add(to[i],c.Pushes[j],t);}}
            for(var id=0;id<shifts.Length;id++)if(counts[id]>0)shifts[id]=sums[id]/counts[id]*(fallback?.55f:1f);
            if(fallback)
            {
                // Simultaneous spring relaxation expands repair into existing approaches.
                // Degree-one ends are legitimate; no continuation is inferred from proximity.
                Array.Copy(shifts,sums,shifts.Length);
                for(var id=0;id<shifts.Length;id++)
                {
                    if(blocked[id])continue;var spring=float2.zero;var neighbors=0;
                    foreach(var i in incident[id])
                    {var other=from[i]==id?to[i]:from[i];if(other==id)continue;var d=math.distance(original[id],original[other]);if(d>300)continue;spring+=(totals[other]-totals[id]+sums[other])*(.15f*(1-d/300));neighbors++;}
                    if(neighbors>0)shifts[id]+=spring/neighbors;
                }
            }
            var limited=0;var rolled=0;var moved=0;var reversed=0;var maxMove=0f;var budget=fallback?48f:24f;
            for(var id=0;id<shifts.Length;id++)
            {
                var shift=shifts[id];if(math.lengthsq(shift)<.00000001f){shifts[id]=float2.zero;continue;}
                var p=original[id]+totals[id];
                float Limit(float value,float delta)=>delta>0?math.clamp((half-value)/delta,0,1):delta<0?math.clamp((-half-value)/delta,0,1):1;
                var factor=Math.Min(Limit(p.x,shift.x),Limit(p.y,shift.y));
                var aa=math.lengthsq(shift);var bb=2*math.dot(totals[id],shift);var cc=math.lengthsq(totals[id])-budget*budget;
                factor=Math.Min(factor,math.clamp((-bb+math.sqrt(Math.Max(0,bb*bb-4*aa*cc)))/(2*aa),0,1));
                if(factor<1){limited++;shift*=Math.Max(0,factor-.0001f);}if(math.dot(previous[id],shift)<-.0001f)reversed++;shifts[id]=shift;
            }
            PreparedSegment Revised(int i)
            {
                var s=sections[i];var a=new float3(shifts[from[i]].x,0,shifts[from[i]].y);var b=new float3(shifts[to[i]].x,0,shifts[to[i]].y);
                return s with{Start=s.Start+a,End=s.End+b,Control1=(s.Control1??math.lerp(s.Start,s.End,1f/3))+math.lerp(a,b,1f/3),Control2=(s.Control2??math.lerp(s.Start,s.End,2f/3))+math.lerp(a,b,2f/3),HorizontalParent=null};
            }
            var check=new List<int>();changed.Clear();for(var id=0;id<shifts.Length;id++)if(math.lengthsq(shifts[id])>0)foreach(var i in incident[id])if(changed.Add(i))check.Add(i);
            for(var cursor=0;cursor<check.Count;cursor++)
            {
                token.ThrowIfCancellationRequested();var i=check[cursor];var candidate=Revised(i);
                bool Inside(float3 p)=>math.all(math.isfinite(p))&&Math.Abs(p.x)<=half&&Math.Abs(p.z)<=half;
                var oldChord=source[i].End.xz-source[i].Start.xz;
                var newChord=candidate.End.xz-candidate.Start.xz;
                var oldLength=math.length(oldChord);var newLength=math.length(newChord);
                // Collision reduction must never accept a reversed/collapsed
                // edge or sharp deformation of a short owning junction span.
                var shapeSafe=newLength>=Math.Max(.05f,oldLength*.7f)&&newLength<=oldLength*1.4f+.1f&&
                    math.dot(math.normalizesafe(oldChord),math.normalizesafe(newChord))>=.866f;
                if(Inside(candidate.CourseStart)&&Inside(candidate.CourseEnd)&&Inside(candidate.CourseControl1)&&Inside(candidate.CourseControl2)&&shapeSafe)continue;
                foreach(var id in new[]{from[i],to[i]})if(math.lengthsq(shifts[id])>0){shifts[id]=float2.zero;blocked[id]=true;rolled++;foreach(var neighbor in incident[id])check.Add(neighbor);}
            }
            active.Clear();foreach(var i in changed.OrderBy(v=>v))
            {
                if(math.lengthsq(shifts[from[i]])+math.lengthsq(shifts[to[i]])==0)continue;
                active.Add(i);active.UnionWith(curves[i].Neighbors);sections[i]=Revised(i);Refresh(i);
            }
            foreach(var i in changed)if(eligible[i])Nearby(i,active);
            for(var id=0;id<shifts.Length;id++){if(math.lengthsq(shifts[id])>0){moved++;maxMove=Math.Max(maxMove,math.length(shifts[id]));totals[id]+=shifts[id];}previous[id]=shifts[id];}
            if(limited>0||rolled>0)log.Warn($"Network-spacing adjustments reached a movement or map boundary limit: {limited} shared nodes constrained, {rolled} rolled back. Connections retained; final safety audit is still required.");
            log.Count("Road width boundary-limited nodes",limited);log.Count("Road width rolled-back shared nodes",rolled);log.Time("Width repair last pass",passTimer.Elapsed.TotalSeconds);
            log.Diagnostic($"PROCESS vanilla-width-boundary requestedNodes={moved+limited}; limitedNodes={limited}; rolledBackNodes={rolled}; movedSharedNodes={moved}; reversingNodes={reversed}; maximumOffsetMeters={maxMove:0.###}; deletedSections=0; preparationContinues={moved>0}");
            if(fallback)clusterPasses++;else fastPasses++;
        }
        // Never accept the last unevaluated move. Return the best measured complete layout.
        sections=best;var revisedCount=0;
        for(var i=0;i<sections.Length;i++)if(!ReferenceEquals(sections[i],source[i]))
        {token.ThrowIfCancellationRequested();var s=sections[i];var geometry=new float3[65];for(var j=0;j<65;j++)geometry[j]=NetworkNormalizer.Evaluate(s,j/64f);sections[i]=s with{SourceGeometry=geometry};revisedCount++;}
        var roads=NetworkHardener.ConstrainWidths(sections.Where(s=>s.Kind==NetworkKind.Road).Select(s=>s with{MaximumNativeWidth=float.PositiveInfinity}).ToList(),token);
        var roadIndex=0;for(var i=0;i<sections.Length;i++)if(sections[i].Kind==NetworkKind.Road)sections[i]=roads[roadIndex++];
        log.Count("Road sections separated for vanilla width",revisedCount);log.Time("Vanilla width repair",timer.Elapsed.TotalSeconds);
        if(bestWorst>.05f){log.Count("Network repair quality observations");log.Warn($"Iteration width repair retained its best measured layout with up to {bestWorst:0.##} m overlap. No sections removed; inspect interchanges in the test map.");}
        log.Diagnostic($"PROCESS vanilla-width-complete passes={Math.Min(passes+1,36)}; fastPasses={fastPasses}; clusterPasses={clusterPasses}; candidatePairs={pairChecks}; curveRefreshes={refreshes}; bestResidual={bestScore:0.###}; worstOverlapMeters={bestWorst:0.###}; remainingOverlapIsQualityWarning=true; deletedSections=0; elapsedSeconds={timer.Elapsed.TotalSeconds:0.###}");
        return sections;
    }
}
