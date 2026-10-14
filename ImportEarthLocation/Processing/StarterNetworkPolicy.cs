using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Gameplay-first topology. Never join unrelated levels or substitute nearby
// roads for actual source connections.
public static class StarterNetworkPolicy
{
    public static IReadOnlyList<PreparedSegment> Quays(IReadOnlyList<PreparedSegment> source,ushort[] raster,float scale,double width,ImportLog log,CancellationToken token)
    {
        var incident=new Dictionary<long,List<PreparedSegment>>();foreach(var s in source.Where(s=>s.Kind==NetworkKind.Road))foreach(var id in new[]{s.StartNodeId,s.EndNodeId})
        {if(!incident.TryGetValue(id,out var list))incident[id]=list=new();list.Add(s);}
        var distance=new Dictionary<long,float>();var queue=new List<long>();
        foreach(var p in incident.Where(p=>p.Value.Count!=2||p.Value.Any(s=>s.ProtectedLink||s.Roundabout||s.StartMiniRoundabout||s.EndMiniRoundabout||s.Bridge||s.Tunnel)))
        {distance[p.Key]=0;queue.Add(p.Key);}
        for(var k=0;k<queue.Count;k++)foreach(var s in incident[queue[k]])
        {token.ThrowIfCancellationRequested();var id=s.StartNodeId==queue[k]?s.EndNodeId:s.StartNodeId;var d=distance[queue[k]]+NetworkVerticalProfiles.ArcLength(s);if(d>120||distance.TryGetValue(id,out var old)&&old<=d)continue;distance[id]=d;queue.Add(id);}
        var ids=new HashSet<long>(source.SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}));var next=long.MinValue+9000000;
        long Node(){while(ids.Contains(next))next++;var id=next++;ids.Add(id);return id;}
        var output=new List<PreparedSegment>();var placed=0;var both=0;var setbackSkipped=0;
        foreach(var original in source)
        {
            token.ThrowIfCancellationRequested();var s=original with{QuaySides=NetworkSides.None};
            if(s.Kind!=NetworkKind.Road||s.Bridge||s.Tunnel||s.ProtectedLink||s.Roundabout){output.Add(s);continue;}
            var length=NetworkVerticalProfiles.ArcLength(s);var setback=Math.Max(30,s.NativeWidth*2);
            var begin=Math.Max(0,setback-(distance.TryGetValue(s.StartNodeId,out var a)?a:setback));var end=length-Math.Max(0,setback-(distance.TryGetValue(s.EndNodeId,out var b)?b:setback));
            if(end-begin<30){setbackSkipped++;output.Add(s);continue;}
            var left=0;var right=0;
            for(var j=0;j<7;j++)
            {var t=math.lerp(begin/length,end/length,(j+.5f)/7);var p=NetworkNormalizer.Evaluate(s,t);var tangent=math.normalizesafe((NetworkNormalizer.Evaluate(s,Math.Min(1,t+.01f))-NetworkNormalizer.Evaluate(s,Math.Max(0,t-.01f))).xz);var normal=new float2(-tangent.y,tangent.x)*(Math.Max(4,s.NativeWidth*.5f)+12);
                if(p.y-NetworkProcessor.Sample(p.x+normal.x,p.z+normal.y,raster,scale,width)>3)left++;
                if(p.y-NetworkProcessor.Sample(p.x-normal.x,p.z-normal.y,raster,scale,width)>3)right++;}
            var side=left>=5&&right<3?NetworkSides.Left:right>=5&&left<3?NetworkSides.Right:NetworkSides.None;
            if(left>=3&&right>=3)both++;if(side==NetworkSides.None){output.Add(s);continue;}
            // Split at setbacks, preserving the exact existing cubic and real
            // endpoint identity. Wall upgrades only cover the central interval.
            var cuts=new List<float>{0};if(begin>1)cuts.Add(begin/length);if(end<length-1)cuts.Add(end/length);cuts.Add(1);
            var node=s.StartNodeId;
            float3 Derivative(float t){var u=1-t;return 3*u*u*(s.CourseControl1-s.CourseStart)+6*u*t*(s.CourseControl2-s.CourseControl1)+3*t*t*(s.CourseEnd-s.CourseControl2);}
            for(var j=1;j<cuts.Count;j++)
            {var lo=cuts[j-1];var hi=cuts[j];var id=j==cuts.Count-1?s.EndNodeId:Node();var offsetA=math.lerp(s.StartCurveOffset,s.EndCurveOffset,lo);var offsetB=math.lerp(s.StartCurveOffset,s.EndCurveOffset,hi);
                var p=NetworkNormalizer.Evaluate(s,lo);var q=NetworkNormalizer.Evaluate(s,hi);var active=(lo+hi)*.5f>=begin/length&&(lo+hi)*.5f<=end/length;
                output.Add(s with{Start=p-offsetA,End=q-offsetB,StartNodeId=node,EndNodeId=id,Control1=p+Derivative(lo)*(hi-lo)/3-offsetA,Control2=q-Derivative(hi)*(hi-lo)/3-offsetB,StartCurveOffset=offsetA,EndCurveOffset=offsetB,StartElevation=p.y-NetworkProcessor.Sample(p.x,p.z,raster,scale,width),EndElevation=q.y-NetworkProcessor.Sample(q.x,q.z,raster,scale,width),StartMiniRoundabout=lo==0&&s.StartMiniRoundabout,EndMiniRoundabout=hi==1&&s.EndMiniRoundabout,StartTerminal=lo==0&&s.StartTerminal,EndTerminal=hi==1&&s.EndTerminal,QuaySides=active?side:NetworkSides.None,RetainingWallSides=active?NetworkSides.None:s.RetainingWallSides,HorizontalParent=null});node=id;}
            placed++;log.Diagnostic($"ROAD QUAY way={s.WayId}; side={side}; beginMeters={begin:R}; endMeters={end:R}; intersectionSetbackMeters={setback:R}; bothSidesAllowed=false; nativeRaisedSideRenderingRequiresGameReview=true");
        }
        log.Diagnostic($"ROAD QUAY PLAN eligibleSections={placed}; bothSidesExcluded={both}; shortSetbackIntervalsExcluded={setbackSkipped}; railExcluded=true");return output;
    }
    public static IReadOnlyList<PreparedSegment> Rail(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token)
    {
        var output=new List<PreparedSegment>(source.Count);var omitted=0;
        foreach(var segment in source)
        {
            token.ThrowIfCancellationRequested();
            if(segment.Kind!=NetworkKind.Rail){output.Add(segment);continue;}
            // Density, short tails and cycles cannot prove yard membership.
            // Legitimate loops and terminal obligations retain source identity.
            var service=segment.RailService;
            if(segment.RawSourceTags.TryGetValue("service",out var raw))service=raw;
            else if(segment.SourceTags.TryGetValue("service",out var normalized))service=normalized;
            if(service is "yard" or "siding" or "spur"){omitted++;continue;}
            output.Add(segment with{PhysicalTracks=2,Lanes=2,OneWay=false});
        }
        log.Count("Source-evidenced yard sections intentionally omitted",omitted);
        if(log.DiagnosticsEnabled)log.Diagnostic($"RAIL BACKBONE yardServiceOmissions={omitted}; densityPruning=false; proximityWelding=false; legitimateTerminalsAndLoopsRetained=true; oneDoubleTrackCourse=true");
        return output;
    }

    public static IReadOnlyList<PreparedSegment> Coordinate(IReadOnlyList<PreparedSegment> source,ushort[] raster,float scale,double width,ImportLog log,CancellationToken token,Func<float3,float?>? waterLevel=null)
    {
        source=RebaseBridgeSupports(source,raster,scale,width,waterLevel,log,token);
        var result=source.ToArray();var grid=new Dictionary<(int,int),HashSet<int>>();var targets=new Dictionary<long,List<float>>();
        var frozen=new HashSet<long>(source.Where(s=>s.Kind==NetworkKind.Road&&s.Tunnel).SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}));
        var bridgeFloors=new Dictionary<long,float>();foreach(var s in source.Where(s=>s.Bridge))foreach(var start in new[]{true,false})
        {var id=start?s.StartNodeId:s.EndNodeId;var y=start?s.Start.y:s.End.y;bridgeFloors[id]=bridgeFloors.TryGetValue(id,out var old)?Math.Max(old,y):y;}
        bool Main(PreparedSegment s)=>s.Kind==NetworkKind.Road&&s.OneWay&&!s.Tunnel&&s.Classification is "motorway" or "trunk";
        for(var i=0;i<source.Count;i++)if(Main(source[i]))
        {var s=source[i];var steps=Math.Max(1,(int)Math.Ceiling(math.distance(s.Start.xz,s.End.xz)/48));for(var k=0;k<=steps;k++){var p=math.lerp(s.Start,s.End,k/(float)steps);var key=((int)Math.Floor(p.x/64),(int)Math.Floor(p.z/64));if(!grid.TryGetValue(key,out var set))grid[key]=set=new();set.Add(i);}}
        var pairs=0;
        foreach(var s in source.Where(Main))foreach(var start in new[]{true,false})
        {
            token.ThrowIfCancellationRequested();var id=start?s.StartNodeId:s.EndNodeId;if(frozen.Contains(id))continue;var p=start?s.Start:s.End;var heading=math.normalizesafe((s.End-s.Start).xz);var gap=64f;float? value=null;
            var gx=(int)Math.Floor(p.x/64);var gz=(int)Math.Floor(p.z/64);
            for(var x=gx-1;x<=gx+1;x++)for(var z=gz-1;z<=gz+1;z++)if(grid.TryGetValue((x,z),out var set))foreach(var i in set)
            {var t=source[i];if(t.WayId==s.WayId||t.Layer!=s.Layer||math.dot(heading,math.normalizesafe((t.End-t.Start).xz))>-.95f)continue;
                if(!(s.Reference.Length>0&&s.Reference.Equals(t.Reference,StringComparison.OrdinalIgnoreCase)||s.Name.Length>0&&s.Name.Equals(t.Name,StringComparison.OrdinalIgnoreCase)||s.CorridorId!=0&&s.CorridorId==t.CorridorId))continue;
                var delta=t.End.xz-t.Start.xz;var square=math.lengthsq(delta);if(square<1)continue;var u=math.dot(p.xz-t.Start.xz,delta)/square;if(u<0||u>1)continue;
                var q=NetworkNormalizer.Evaluate(t,u);var d=math.distance(p.xz,q.xz);if(d<2||d>=gap)continue;gap=d;value=s.Bridge||t.Bridge?Math.Max(p.y,q.y):(p.y+q.y)*.5f;}
            if(value.HasValue){if(!targets.TryGetValue(id,out var list))targets[id]=list=new();list.Add(value.Value);pairs++;}
        }
        var heights=new Dictionary<long,float>();foreach(var p in targets)heights[p.Key]=p.Value.Average();var groundAdjusted=0;
        // Each real junction has one shared height; budgets reference the DEM,
        // not a previously edited roadbed or an accumulated elevation offset.
        foreach(var s in source.Where(s=>!s.Tunnel&&!s.Bridge))foreach(var start in new[]{true,false})
        {var id=start?s.StartNodeId:s.EndNodeId;if(frozen.Contains(id))continue;var p=start?s.Start:s.End;var ground=NetworkProcessor.Sample(p.x,p.z,raster,scale,width);var budget=NetworkVerticalProfiles.CutFillBudget(s);
            var proposed=heights.TryGetValue(id,out var h)?h:p.y;var bounded=math.clamp(proposed,Math.Max(0,ground-budget),ground+budget);if(bridgeFloors.TryGetValue(id,out var floor))bounded=Math.Max(bounded,floor);if(Math.Abs(bounded-proposed)>.01f)groundAdjusted++;heights[id]=bounded;}
        for(var i=0;i<result.Length;i++)
        {var s=result[i];if(s.Tunnel)continue;var a=s.Start;var b=s.End;if(heights.TryGetValue(s.StartNodeId,out var h))a.y=h;if(heights.TryGetValue(s.EndNodeId,out h))b.y=h;
            var da=a.y-s.Start.y;var db=b.y-s.End.y;var c=s.Control1??math.lerp(s.Start,s.End,1f/3);var d=s.Control2??math.lerp(s.Start,s.End,2f/3);c.y+=math.lerp(da,db,1f/3);d.y+=math.lerp(da,db,2f/3);
            var revised=s with{Start=a,End=b,Control1=c,Control2=d,StartElevation=a.y-NetworkProcessor.Sample(a.x,a.z,raster,scale,width),EndElevation=b.y-NetworkProcessor.Sample(b.x,b.z,raster,scale,width)};
            // Only ground-supported bridge tails may become ground. Do not
            // lower decks or convert genuinely unsupported water crossings.
            if(s.Bridge&&Enumerable.Range(0,9).All(j=>Math.Abs(NetworkNormalizer.Evaluate(revised,j/8f).y-NetworkProcessor.Sample(NetworkNormalizer.Evaluate(revised,j/8f).x,NetworkNormalizer.Evaluate(revised,j/8f).z,raster,scale,width))<1.5f))
            {revised=revised with{Bridge=false};log.Count("Ground supported bridge tails reclassified");}
            result[i]=revised;}
        log.Diagnostic($"CORRIDOR GAMEPLAY pairedStations={pairs}; groundBudgetClamps={groundAdjusted}; referenceAcrossWayBoundaries=true; tunnelAnchorsPreserved=true; baseline=immutableDEM");
        // Pair coordination must not be the last writer of arbitrary handle
        // heights. Reconcile shared grades and tangents before roadbed refresh.
        return CorridorProfiles.Prepare(result,log,token,waterLevel,finishOnly:true);
    }

    private static IReadOnlyList<PreparedSegment> RebaseBridgeSupports(IReadOnlyList<PreparedSegment> source,ushort[] raster,float scale,double width,Func<float3,float?>? waterLevel,ImportLog log,CancellationToken token)
    {
        float Ground(float3 p)=>NetworkProcessor.Sample(p.x,p.z,raster,scale,width);
        var grid=new Dictionary<(int,int),HashSet<int>>();
        for(var i=0;i<source.Count;i++)if(!source[i].Tunnel)
        {
            var s=source[i];var count=Math.Max(1,(int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(s)/48));
            for(var j=0;j<=count;j++){var p=NetworkNormalizer.Evaluate(s,j/(float)count);var key=((int)Math.Floor(p.x/128),(int)Math.Floor(p.z/128));if(!grid.TryGetValue(key,out var list))grid[key]=list=new();list.Add(i);}
        }
        var floors=new Dictionary<(NetworkKind,long),float>();var bridges=0;var crossingSamples=0;
        void Floor(PreparedSegment s,long id,float height){var key=(s.Kind,id);floors[key]=floors.TryGetValue(key,out var h)?Math.Max(h,height):height;}
        for(var i=0;i<source.Count;i++)
        {
            token.ThrowIfCancellationRequested();var s=source[i];if(!s.Bridge||s.Tunnel)continue;bridges++;
            Floor(s,s.StartNodeId,Ground(s.Start));Floor(s,s.EndNodeId,Ground(s.End));
            var minimum=float.NegativeInfinity;var count=Math.Max(8,(int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(s)/24));
            for(var j=0;j<=count;j++)
            {
                var p=NetworkNormalizer.Evaluate(s,j/(float)count);var wet=waterLevel?.Invoke(p);if(wet.HasValue&&math.isfinite(wet.Value))minimum=Math.Max(minimum,wet.Value+7);
                var cell=((int)Math.Floor(p.x/128),(int)Math.Floor(p.z/128));var nearby=new HashSet<int>();
                for(var x=-1;x<=1;x++)for(var z=-1;z<=1;z++)if(grid.TryGetValue((cell.Item1+x,cell.Item2+z),out var candidates))nearby.UnionWith(candidates);
                foreach(var k in nearby)
                {
                    var t=source[k];if(k==i||t.Layer>s.Layer||s.Kind==t.Kind&&(s.StartNodeId==t.StartNodeId||s.StartNodeId==t.EndNodeId||s.EndNodeId==t.StartNodeId||s.EndNodeId==t.EndNodeId))continue;
                    if(Math.Abs(math.dot(math.normalizesafe((s.End-s.Start).xz),math.normalizesafe((t.End-t.Start).xz)))>.9f)continue;
                    for(var q=0;q<8;q++)
                    {
                        var a=NetworkNormalizer.Evaluate(t,q/8f);var b=NetworkNormalizer.Evaluate(t,(q+1)/8f);var delta=b.xz-a.xz;
                        var u=math.clamp(math.dot(p.xz-a.xz,delta)/Math.Max(.001f,math.lengthsq(delta)),0,1);var point=math.lerp(a,b,u);
                        if(math.distance(p.xz,point.xz)>Math.Max(4,(s.NativeWidth+t.NativeWidth)*.5f))continue;
                        // Equal-layer independent bridge crossings lack a safe
                        // ordering witness. Preserve their old clearance plane
                        // rather than collapsing both decks onto the DEM.
                        if(t.Bridge&&t.Layer==s.Layer){minimum=Math.Max(minimum,p.y);crossingSamples++;continue;}
                        var level=t.Bridge?point.y:math.clamp(point.y,Ground(point)-NetworkVerticalProfiles.CutFillBudget(t),Ground(point)+NetworkVerticalProfiles.CutFillBudget(t));
                        minimum=Math.Max(minimum,level+NetworkVerticalProfiles.MinimumBridgeClearance+2);crossingSamples++;
                    }
                }
            }
            if(math.isfinite(minimum)){Floor(s,s.StartNodeId,minimum);Floor(s,s.EndNodeId,minimum);}
        }
        var frozen=new HashSet<(NetworkKind,long)>(source.Where(s=>s.Tunnel).SelectMany(s=>new[]{(s.Kind,s.StartNodeId),(s.Kind,s.EndNodeId)}));
        var heights=new Dictionary<(NetworkKind,long),float>(floors);
        foreach(var s in source.Where(s=>!s.Bridge&&!s.Tunnel))foreach(var pair in new[]{(s.StartNodeId,s.Start),(s.EndNodeId,s.End)})
        {var key=(s.Kind,pair.Item1);if(heights.TryGetValue(key,out var h))heights[key]=Math.Max(h,Ground(pair.Item2));}
        var result=source.ToArray();var adjusted=0;
        for(var i=0;i<result.Length;i++)
        {
            var s=result[i];if(s.Tunnel)continue;var a=s.Start;var b=s.End;
            if(!frozen.Contains((s.Kind,s.StartNodeId))&&heights.TryGetValue((s.Kind,s.StartNodeId),out var h))a.y=h;
            if(!frozen.Contains((s.Kind,s.EndNodeId))&&heights.TryGetValue((s.Kind,s.EndNodeId),out h))b.y=h;
            var da=a.y-s.Start.y;var db=b.y-s.End.y;if(Math.Abs(da)+Math.Abs(db)<.01f&&!s.Bridge)continue;
            var c=s.Control1??math.lerp(s.Start,s.End,1f/3);var d=s.Control2??math.lerp(s.Start,s.End,2f/3);c.y+=math.lerp(da,db,1f/3);d.y+=math.lerp(da,db,2f/3);
            if(s.Bridge)
            {
                // Old interior handle humps are not structural requirements.
                // Every evidenced crossing established a floor at BOTH ends
                // above; a monotone deck preserves that floor without waves.
                var delta=b.xz-a.xz;var square=Math.Max(.001f,math.lengthsq(delta));
                c.y=math.lerp(a.y,b.y,math.clamp(math.dot(c.xz-a.xz,delta)/square,0,1));
                d.y=math.lerp(a.y,b.y,math.clamp(math.dot(d.xz-a.xz,delta)/square,0,1));
            }
            result[i]=s with{Start=a,End=b,Control1=c,Control2=d,StartElevation=a.y-Ground(a),EndElevation=b.y-Ground(b)};adjusted++;
        }
        log.Diagnostic($"BRIDGE SUPPORT PROFILE bridges={bridges}; adjustedSections={adjusted}; crossingSamples={crossingSamples}; floorsFromTerrainAndObservedCrossings=true; oldDeckHumpsNotFloors=true; clearSpaceMeters=5; deckAllowanceMeters=2; waterFloorsRetained=true; tunnelNodesFrozen=true");
        return result;
    }
}
