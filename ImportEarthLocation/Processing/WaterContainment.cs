using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// A terrain-only spill screen, in local map metres and absolute encoded heights.
// It does not certify native fluid behaviour. No terrain is raised or cut here.
public static class WaterContainment
{
    private const int MaximumCells=1048576;
    private const float Freeboard=.35f;

    public static WaterPlan Validate(WaterPlan plan,TerrainBuilder.Result terrain,double width,
        IReadOnlyList<GeographicPolygon> polygons,ImportLog log,CancellationToken token)
    {
        token.ThrowIfCancellationRequested();
        var lakes=plan.Sources.Where(s=>s.BodyKind=="lake").GroupBy(s=>s.BodyId).ToArray();
        if(lakes.Length==0)return plan;
        var outlines=polygons.GroupBy(p=>p.Identity).ToDictionary(g=>g.Key,g=>g.First());
        // A neighbouring standing-water body may itself fail this screen. Only
        // mapped flowing reaches can establish an intended outlet here.
        var surface=WaterBodyPlanner.ConstructedSurfaceSampler(plan with{AreaStages=Array.Empty<WaterAreaStage>()},polygons);
        var levels=new Dictionary<long,float>();var rejected=new HashSet<long>();var checkedBodies=0;var lowered=0;
        foreach(var lake in lakes)
        {
            token.ThrowIfCancellationRequested();
            if(!outlines.TryGetValue(lake.Key,out var polygon))
            {rejected.Add(lake.Key);log.Warn($"Lake {lake.Key}: containment outline unavailable; its controls were omitted.");continue;}
            if(plan.Coastal&&(polygon.Tags.TryGetValue("water",out var kind)&&kind=="lagoon"||polygon.Tags.TryGetValue("salt",out var salt)&&salt=="yes"))continue;
            var controls=lake.ToArray();var stage=controls.Min(s=>s.TargetSurfaceLevel??s.Position.y+s.Depth);
            var spill=Spill(polygon,controls,stage,terrain,width,plan.WaterCellSize,surface,token);checkedBodies++;
            if(!spill.HasValue)
            {rejected.Add(lake.Key);log.Warn($"Lake {lake.Key}: containment coverage exceeded its bounded terrain screen; its controls were omitted.");continue;}
            var safe=Math.Min(stage,spill.Value-Freeboard);
            var wet=controls.Any(s=>safe-NetworkProcessor.Sample(s.Position.x,s.Position.z,terrain.Heightmap,terrain.HeightScaleMeters,width)>=.4f);
            if(!wet||!math.isfinite(safe))
            {rejected.Add(lake.Key);log.Warn($"Lake {lake.Key}: its planned level {stage:0.##} m has an uncontained escape at {spill.Value:0.##} m; its controls were omitted.");}
            else
            {levels[lake.Key]=safe;if(safe<stage-.01f)lowered++;}
            log.Diagnostic($"WATER CONTAINMENT body={lake.Key}; plannedLevel={stage:R}; lowestSpill={spill.Value:R}; acceptedLevel={(rejected.Contains(lake.Key)?float.NaN:safe):R}; sourceRejected={rejected.Contains(lake.Key)}; terrainOnly=true; nativeFlowVerified=false; bankRaising=false");
        }
        var sources=new List<RiverSourcePlan>();
        foreach(var source in plan.Sources)
        {
            if(rejected.Contains(source.BodyId))continue;
            if(source.BodyKind!="lake"||!levels.TryGetValue(source.BodyId,out var level)){sources.Add(source);continue;}
            var position=source.Position;position.y=NetworkProcessor.Sample(position.x,position.z,terrain.Heightmap,terrain.HeightScaleMeters,width);
            if(level-position.y<.4f)continue;
            sources.Add(source with{Position=position,TargetSurfaceLevel=level,Depth=level-position.y});
        }
        log.Count("Lake containment bodies checked",checkedBodies);log.Count("Lake containment levels lowered",lowered);log.Count("Lake containment bodies omitted",rejected.Count);
        return plan with{Sources=sources,AreaStages=plan.AreaStages.Where(s=>!rejected.Contains(s.BodyId))
            .Select(s=>levels.TryGetValue(s.BodyId,out var level)?s with{Level=level}:s).ToArray(),
            Skipped=plan.Skipped+plan.Sources.Count-sources.Count,
            Report=plan.Report+$" Lake containment: {checkedBodies} checked, {lowered} lowered, {rejected.Count} omitted; native flow unverified."};
    }

    // Minimax Dijkstra: a path costs its highest terrain sample. The first dry
    // exterior point popped identifies the lowest escape saddle. Planned flowing
    // outlets terminate their own branch; other dry exterior land remains tested.
    private static float? Spill(GeographicPolygon polygon,IReadOnlyList<RiverSourcePlan> controls,float stage,
        TerrainBuilder.Result terrain,double width,float waterCell,Func<float3,float?> surface,CancellationToken token)
    {
        var n=TerrainSampling.Resolution(terrain.Heightmap);var step=(float)width/(n-1);var half=(float)width*.5f;
        var vertices=polygon.Outer.SelectMany(r=>r).ToArray();if(vertices.Length<3)return null;
        var margin=Math.Max(step*3,waterCell*2);var lo=vertices.Select(p=>p.xz).Aggregate(math.min)-margin;
        var hi=vertices.Select(p=>p.xz).Aggregate(math.max)+margin;
        int Pixel(float value)=>(int)Math.Floor((value+half)/step);
        var x0=Math.Max(0,Pixel(lo.x));var z0=Math.Max(0,Pixel(lo.y));var x1=Math.Min(n-1,Pixel(hi.x)+1);var z1=Math.Min(n-1,Pixel(hi.y)+1);
        var nx=x1-x0+1;var nz=z1-z0+1;var count=(long)nx*nz;
        if(nx<2||nz<2||count>MaximumCells)return null;
        var inside=new byte[(int)count];var crossings=new List<float>();
        void Fill(float3[] ring,byte value)
        {
            if(ring.Length<3)return;
            for(var z=0;z<nz;z++)
            {
                token.ThrowIfCancellationRequested();var worldZ=(z+z0)*step-half;crossings.Clear();
                for(var k=0;k<ring.Length;k++)
                {var a=ring[k];var b=ring[(k+1)%ring.Length];if((a.z>worldZ)!=(b.z>worldZ))crossings.Add(a.x+(worldZ-a.z)*(b.x-a.x)/(b.z-a.z));}
                crossings.Sort();
                for(var k=1;k<crossings.Count;k+=2)
                {
                    var first=Math.Max(0,(int)Math.Ceiling((crossings[k-1]+half)/step)-x0);var last=Math.Min(nx-1,Pixel(crossings[k])-x0);
                    for(var x=first;x<=last;x++)inside[z*nx+x]=value;
                }
            }
        }
        foreach(var ring in polygon.Outer)Fill(ring,1);foreach(var ring in polygon.Inner)Fill(ring,0);
        // A one-water-cell shoreline collar accommodates the sloping bank, but
        // cannot conceal a path continuing into surrounding downhill terrain.
        var allowed=(byte[])inside.Clone();var collar=Math.Max(1,(int)Math.Ceiling(waterCell/step));
        for(var z=0;z<nz;z++)for(var x=0;x<nx;x++)if(inside[z*nx+x]!=0)
        {
            if(x>0&&x<nx-1&&z>0&&z<nz-1&&inside[z*nx+x-1]!=0&&inside[z*nx+x+1]!=0&&inside[(z-1)*nx+x]!=0&&inside[(z+1)*nx+x]!=0)continue;
            for(var dz=-collar;dz<=collar;dz++)for(var dx=-collar;dx<=collar;dx++)
                if(x+dx>=0&&x+dx<nx&&z+dz>=0&&z+dz<nz)allowed[(z+dz)*nx+x+dx]=1;
        }
        var best=new float[(int)count];for(var k=0;k<best.Length;k++)best[k]=float.PositiveInfinity;
        var heap=new List<(int Index,float Cost)>();
        float Height(int index)=>terrain.Heightmap[(index/nx+z0)*n+index%nx+x0]/65535f*terrain.HeightScaleMeters;
        void Push(int index,float cost)
        {
            if(cost>=best[index])return;best[index]=cost;heap.Add((index,cost));var k=heap.Count-1;
            while(k>0){var parent=(k-1)/2;if(heap[parent].Cost<=cost)break;heap[k]=heap[parent];k=parent;}heap[k]=(index,cost);
        }
        (int Index,float Cost) Pop()
        {
            var first=heap[0];var last=heap[heap.Count-1];heap.RemoveAt(heap.Count-1);if(heap.Count==0)return first;
            var k=0;while(k*2+1<heap.Count){var child=k*2+1;if(child+1<heap.Count&&heap[child+1].Cost<heap[child].Cost)child++;if(heap[child].Cost>=last.Cost)break;heap[k]=heap[child];k=child;}heap[k]=last;return first;
        }
        foreach(var source in controls)
        {
            var x=(int)Math.Round((source.Position.x+half)/step)-x0;var z=(int)Math.Round((source.Position.z+half)/step)-z0;
            if(x>=0&&x<nx&&z>=0&&z<nz)Push(z*nx+x,Height(z*nx+x));
        }
        if(heap.Count==0)return null;
        while(heap.Count>0)
        {
            token.ThrowIfCancellationRequested();var (index,cost)=Pop();if(cost!=best[index])continue;
            var x=index%nx;var z=index/nx;
            if(allowed[index]==0||x+x0==0||z+z0==0||x+x0==n-1||z+z0==n-1)
            {
                var p=new float3((x+x0)*step-half,Height(index),(z+z0)*step-half);var outlet=surface(p);
                if(allowed[index]==0&&outlet.HasValue&&outlet.Value<=stage+.1f&&p.y<outlet.Value+.1f)continue;
                return cost;
            }
            for(var dz=-1;dz<=1;dz++)for(var dx=-1;dx<=1;dx++)
            {
                if(dx==0&&dz==0||x+dx<0||x+dx>=nx||z+dz<0||z+dz>=nz)continue;
                var next=(z+dz)*nx+x+dx;Push(next,Math.Max(cost,Height(next)));
            }
        }
        // Every exit was an evidenced planned water connection.
        return float.MaxValue;
    }
}
