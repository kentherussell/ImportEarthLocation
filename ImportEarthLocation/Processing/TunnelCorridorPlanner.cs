using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Solve buried chains and their connected approaches together, not each OSM
// way independently. Layer orders crossings; it never determines depth.
public static class TunnelCorridorPlanner
{
    private readonly record struct Key(NetworkKind Kind,long Id);
    public static IReadOnlyList<PreparedSegment> Prepare(IReadOnlyList<PreparedSegment> source,ushort[] raster,float scale,double width,ImportLog log,CancellationToken token)
    {
        if(!source.Any(s=>s.Tunnel))return source;
        var sections=new List<PreparedSegment>();var occupied=new HashSet<Key>(source.SelectMany(s=>new[]{new Key(s.Kind,s.StartNodeId),new Key(s.Kind,s.EndNodeId)}));var next=long.MinValue+4000000;
        float Ground(float3 p)=>NetworkProcessor.Sample(p.x,p.z,raster,scale,width);
        foreach(var s in source)
        {
            token.ThrowIfCancellationRequested();if(!s.Tunnel){sections.Add(s);continue;}
            var length=NetworkVerticalProfiles.ArcLength(s);var parts=Math.Max(length>=40?2:1,(int)Math.Ceiling(length/80));
            var ids=new long[parts+1];ids[0]=s.StartNodeId;ids[parts]=s.EndNodeId;
            for(var i=1;i<parts;i++){while(occupied.Contains(new(s.Kind,next)))next++;ids[i]=next++;occupied.Add(new(s.Kind,ids[i]));}
            for(var i=0;i<parts;i++)
            {
                var lo=i/(float)parts;var hi=(i+1)/(float)parts;var a=NetworkNormalizer.Evaluate(s,lo);var d=NetworkNormalizer.Evaluate(s,hi);
                float3 Derivative(float t){var u=1-t;return 3*u*u*(s.CourseControl1-s.CourseStart)+6*u*t*(s.CourseControl2-s.CourseControl1)+3*t*t*(s.CourseEnd-s.CourseControl2);}
                sections.Add(s with{Start=a,End=d,Control1=a+Derivative(lo)*(hi-lo)/3,Control2=d-Derivative(hi)*(hi-lo)/3,
                    StartCurveOffset=default,EndCurveOffset=default,StartNodeId=ids[i],EndNodeId=ids[i+1],
                    StartRoundaboutRadius=i==0?s.StartRoundaboutRadius:0,EndRoundaboutRadius=i==parts-1?s.EndRoundaboutRadius:0,
                    StartMiniRoundabout=i==0&&s.StartMiniRoundabout,EndMiniRoundabout=i==parts-1&&s.EndMiniRoundabout,
                    HorizontalParent=null,ParentStartT=0,ParentEndT=1});
            }
        }
        var joins=new Dictionary<Key,List<int>>();var points=new Dictionary<Key,float3>();var heights=new Dictionary<Key,float>();var lengths=new float[sections.Count];
        for(var i=0;i<sections.Count;i++)
        {
            var s=sections[i];lengths[i]=NetworkVerticalProfiles.ArcLength(s);
            foreach(var end in new[]{(s.StartNodeId,s.Start),(s.EndNodeId,s.End)})
            {var key=new Key(s.Kind,end.Item1);points[key]=end.Item2;if(!joins.TryGetValue(key,out var list))joins[key]=list=new();list.Add(i);heights[key]=end.Item2.y;}
        }
        var floors=new Dictionary<Key,float>();var ceilings=new Dictionary<Key,float>();var active=new HashSet<Key>();
        foreach(var p in joins)
        {
            var ground=Ground(points[p.Key]);var tunnel=p.Value.Any(i=>sections[i].Tunnel);var portal=tunnel&&p.Value.Any(i=>!sections[i].Tunnel);
            var budget=p.Key.Kind==NetworkKind.Rail?20:p.Value.Max(i=>NetworkVerticalProfiles.CutFillBudget(sections[i]));
            floors[p.Key]=Math.Max(0,ground-(tunnel?120:budget));ceilings[p.Key]=ground+budget;if(!tunnel)continue;
            var bank=ground;
            foreach(var i in p.Value.Where(i=>sections[i].Tunnel))
            {var s=sections[i];var tangent=math.normalizesafe((s.End-s.Start).xz);var side=new float3(-tangent.y,0,tangent.x)*Math.Max(3,s.NativeWidth*.5f);bank=Math.Min(bank,Math.Min(Ground(points[p.Key]+side),Ground(points[p.Key]-side)));}
            ceilings[p.Key]=Math.Max(floors[p.Key],bank-(portal?2:15));heights[p.Key]=Math.Min(heights[p.Key],ceilings[p.Key]);active.Add(p.Key);
        }
        // Bound the repair envelope by actual graph distance, not proximity.
        var distance=active.ToDictionary(k=>k,k=>0f);var queue=active.ToList();
        for(var cursor=0;cursor<queue.Count;cursor++)foreach(var i in joins[queue[cursor]])
        {
            token.ThrowIfCancellationRequested();var s=sections[i];
            var far=new Key(s.Kind,s.StartNodeId==queue[cursor].Id?s.EndNodeId:s.StartNodeId);var d=distance[queue[cursor]]+lengths[i];
            if(d>1000||distance.TryGetValue(far,out var old)&&old<=d)continue;distance[far]=d;active.Add(far);queue.Add(far);
        }
        // Include frontier edges with a fixed far anchor, and actual bridge
        // approaches. Otherwise one changed endpoint can leave an unbounded kink.
        foreach(var key in joins.Keys.Where(k=>!active.Contains(k)))floors[key]=ceilings[key]=heights[key];
        var affected=Enumerable.Range(0,sections.Count).Where(i=>active.Contains(new(sections[i].Kind,sections[i].StartNodeId))||active.Contains(new(sections[i].Kind,sections[i].EndNodeId))).ToArray();
        var passes=0;var residual=0f;
        for(;passes<128;passes++)
        {
            token.ThrowIfCancellationRequested();var movement=0f;residual=0;
            foreach(var i in affected)
            {
                var s=sections[i];var a=new Key(s.Kind,s.StartNodeId);var b=new Key(s.Kind,s.EndNodeId);var grade=NetworkVerticalProfiles.DesiredGrade(s);
                var excess=Math.Abs(heights[a]-heights[b])-lengths[i]*grade;if(excess<=.005f)continue;residual=Math.Max(residual,excess);
                var upper=heights[a]>heights[b]?a:b;var lower=upper==a?b:a;
                var lowered=Math.Max(floors[upper],heights[upper]-excess);movement=Math.Max(movement,heights[upper]-lowered);heights[upper]=lowered;
                excess=Math.Abs(heights[a]-heights[b])-lengths[i]*grade;
                if(excess>.005f){var raised=Math.Min(ceilings[lower],heights[lower]+excess);movement=Math.Max(movement,raised-heights[lower]);heights[lower]=raised;}
            }
            if(movement<.001f)break;
        }
        var changed=0;
        for(var i=0;i<sections.Count;i++)
        {
            var s=sections[i];var ka=new Key(s.Kind,s.StartNodeId);var kb=new Key(s.Kind,s.EndNodeId);if(!active.Contains(ka)&&!active.Contains(kb))continue;
            var a=s.Start;var d=s.End;a.y=heights[ka];d.y=heights[kb];var c1=s.Control1??math.lerp(a,d,1f/3);var c2=s.Control2??math.lerp(a,d,2f/3);
            if(!s.Tunnel)
            {
                if(Math.Abs(a.y-s.Start.y)+Math.Abs(d.y-s.End.y)<.001f)continue;
                c1.y+=math.lerp(a.y-s.Start.y,d.y-s.End.y,1f/3);c2.y+=math.lerp(a.y-s.Start.y,d.y-s.End.y,2f/3);
                sections[i]=s with{Start=a,End=d,Control1=c1,Control2=c2,StartElevation=a.y-Ground(a),EndElevation=d.y-Ground(d)};changed++;continue;
            }
            var delta=d.xz-a.xz;var square=Math.Max(.001f,math.lengthsq(delta));c1.y=a.y+(d.y-a.y)*math.dot(c1.xz-a.xz,delta)/square;c2.y=a.y+(d.y-a.y)*math.dot(c2.xz-a.xz,delta)/square;
            sections[i]=s with{Start=a,End=d,Control1=c1,Control2=c2,StartElevation=a.y-Ground(a),EndElevation=d.y-Ground(d)};changed++;
        }
        residual=affected.Select(i=>Math.Max(0,Math.Abs(heights[new(sections[i].Kind,sections[i].StartNodeId)]-heights[new(sections[i].Kind,sections[i].EndNodeId)])-lengths[i]*NetworkVerticalProfiles.DesiredGrade(sections[i]))).DefaultIfEmpty(0).Max();
        log.Diagnostic($"TUNNEL CORRIDOR sections={sections.Count(s=>s.Tunnel)}; approachNodes={active.Count}; revisedSections={changed}; passes={Math.Min(passes+1,128)}; endpointGradeResidualMeters={residual:R}; portalCutMeters=2; interiorCoverMeters=15; approachEnvelopeMeters=1000; frontierAnchorsFixed=true; bridgeApproachesIncluded=true; proximityJoins=false; sourceSectionsDeleted=0; nativeUndergroundCompositionRequiresReadback=true");
        return sections;
    }
}
