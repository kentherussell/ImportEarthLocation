using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

public static class NetworkRoadbed
{
    public static ushort[] Refresh(ushort[] baseline,ushort[] previousRoadbed,ushort[] finalRaster,float scale,double width,
        IReadOnlyList<PreparedSegment> previousProfiles,IReadOnlyList<PreparedSegment> finalProfiles,
        IEnumerable<float3[]> protectedAreas,IEnumerable<float3[]> protectedLines,ImportLog log,CancellationToken token)
    {
        if(baseline.Length!=previousRoadbed.Length||baseline.Length!=finalRaster.Length||previousProfiles.Count!=finalProfiles.Count)
            throw new InvalidOperationException("Final roadbed refresh requires the same terrain and complete source network.");
        var changed=0;
        for(var i=0;i<finalProfiles.Count;i++)
        {
            token.ThrowIfCancellationRequested();var a=previousProfiles[i];var b=finalProfiles[i];if(NetworkStructure.IsStructure(b))continue;
            if(Math.Abs(a.CourseStart.y-b.CourseStart.y)>.01f||Math.Abs(a.CourseControl1.y-b.CourseControl1.y)>.01f||
               Math.Abs(a.CourseControl2.y-b.CourseControl2.y)>.01f||Math.Abs(a.CourseEnd.y-b.CourseEnd.y)>.01f)changed++;
        }
        if(changed==0)return finalRaster;
        // Recompute against the immutable pre-earthworks raster. A late profile
        // change must not leave a floating/cut road, nor accumulate another cut
        // budget on top of the first pass. Preserve subsequent water edits exactly.
        var refreshed=Prepare(baseline,scale,width,finalProfiles,protectedAreas,protectedLines,log,token);
        var waterEdits=0;
        for(var i=0;i<refreshed.Length;i++)
        {if((i&4095)==0)token.ThrowIfCancellationRequested();if(finalRaster[i]==previousRoadbed[i])continue;refreshed[i]=finalRaster[i];waterEdits++;}
        log.Diagnostic($"PROCESS final-roadbed refreshedGroundProfiles={changed}; laterWaterSamplesRetained={waterEdits}; immutableEarthworkBaseline=true; cutFillBudgetNotAccumulated=true; protectedRoofsRetained=true");
        return refreshed;
    }

    // Work on a copy. Structural spans never become broad terrain embankments.
    public static ushort[] Prepare(ushort[] source,float scale,double width,IReadOnlyList<PreparedSegment> segments,
        IEnumerable<float3[]> protectedAreas,IEnumerable<float3[]> protectedLines,ImportLog log,CancellationToken token)
    {
        var n=(int)Math.Sqrt(source.Length);if(n*n!=source.Length||n<2||width<=0||scale<=0)throw new InvalidOperationException("Invalid roadbed terrain.");
        var cell=(float)(width/(n-1));var protectedMask=new bool[source.Length];
        var coordinates=new float[n];for(var i=0;i<n;i++)coordinates[i]=(i/(float)(n-1)-.5f)*(float)width;
        int Pixel(float value)=>(int)Math.Round((value/width+.5)*(n-1));
        void Guard(float3 a,float3 b,float margin)
        {
            var lo=math.min(a.xz,b.xz)-margin;var hi=math.max(a.xz,b.xz)+margin;
            var delta=b.xz-a.xz;var denominator=Math.Max(.001f,math.lengthsq(delta));
            for(var z=Math.Max(0,Pixel(lo.y));z<=Math.Min(n-1,Pixel(hi.y));z++)
                for(var x=Math.Max(0,Pixel(lo.x));x<=Math.Min(n-1,Pixel(hi.x));x++)
                {
                    var p=new float2(coordinates[x],coordinates[z]);
                    var t=math.clamp(math.dot(p-a.xz,delta)/denominator,0,1);
                    if(math.distance(p,math.lerp(a.xz,b.xz,t))<=margin)protectedMask[z*n+x]=true;
                }
        }
        // Protect the actual polygon, not its rectangular envelope. A winding
        // river's bounding box can cover kilometres of dry roads and prevent
        // their entire cut/fill pass from being applied.
        foreach(var ring in protectedAreas)
        {
            token.ThrowIfCancellationRequested();if(ring.Length<3)continue;
            var crossings=new List<float>(ring.Length);
            for(var z=Math.Max(0,Pixel(ring.Min(p=>p.z)));z<=Math.Min(n-1,Pixel(ring.Max(p=>p.z)));z++)
            {
                token.ThrowIfCancellationRequested();var y=coordinates[z];crossings.Clear();
                for(var i=0;i<ring.Length;i++){var a=ring[i];var b=ring[(i+1)%ring.Length];if((a.z>y)==(b.z>y))continue;crossings.Add(a.x+(y-a.z)*(b.x-a.x)/(b.z-a.z));}
                crossings.Sort();for(var i=1;i<crossings.Count;i+=2)for(var x=Math.Max(0,Pixel(crossings[i-1]));x<=Math.Min(n-1,Pixel(crossings[i]));x++)protectedMask[z*n+x]=true;
            }
            for(var i=0;i<ring.Length;i++)Guard(ring[i],ring[(i+1)%ring.Length],cell);
        }
        foreach(var line in protectedLines)for(var i=1;i<line.Length;i++){token.ThrowIfCancellationRequested();Guard(line[i-1],line[i],cell*2);}
        // Surface-road cuts must not remove a tunnel's roof. Include the terrain
        // interpolation footprint, not just the narrow track/road centreline.
        foreach(var tunnel in segments.Where(s=>s.Tunnel))
        {
            var steps=Math.Max(1,(int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(tunnel)/Math.Max(1,cell*.5f)));
            var margin=(tunnel.NativeWidth>0?tunnel.NativeWidth*.5f:6)+cell*2;
            var previous=NetworkNormalizer.Evaluate(tunnel,0);
            for(var i=1;i<=steps;i++){token.ThrowIfCancellationRequested();var next=NetworkNormalizer.Evaluate(tunnel,i/(float)steps);Guard(previous,next,margin);previous=next;}
        }
        var targets=new Dictionary<int,(float target,float budget,float blend,float distance)>();var protectedSamples=0;var beyondBudget=0;var clock=System.Diagnostics.Stopwatch.StartNew();var nextProgress=5d;var processed=0;
        foreach(var s in segments.Where(s=>!NetworkStructure.IsStructure(s)))
        {
            token.ThrowIfCancellationRequested();processed++;
            var budget=Math.Min(60,Math.Max(NetworkVerticalProfiles.CutFillBudget(s),Math.Max(Math.Abs(s.StartElevation??0),Math.Abs(s.EndElevation??0))+2));
            var half=s.NativeWidth>0?s.NativeWidth*.5f:Math.Min(12,Math.Max(4,s.Lanes*1.75f));
            var steps=Math.Max(1,(int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(s)/(cell*.5f)));
            for(var i=0;i<=steps;i++)
            {
                token.ThrowIfCancellationRequested();
                if(clock.Elapsed.TotalSeconds>=nextProgress)
                {log.Diagnostic($"PROCESS roadbed sections={processed}; sectionSample={i}/{steps}; affectedSamples={targets.Count}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.0}");nextProgress=clock.Elapsed.TotalSeconds+5;}
                var t=i/(float)steps;var p=NetworkNormalizer.Evaluate(s,t);var px=Pixel(p.x);var pz=Pixel(p.z);
                var previous=NetworkNormalizer.Evaluate(s,Math.Max(0,(i-1)/(float)steps));
                var span=p.xz-previous.xz;var spanLengthSquared=math.lengthsq(span);
                var originalCenter=source[math.clamp(pz,0,n-1)*n+math.clamp(px,0,n-1)]/65535f*scale;
                // A large embankment needs proportionally more lateral room.
                // Smoothstep's peak derivative is 1.5: six metres of shoulder
                // per metre of fill keeps the added bank slope near 1:4.
                var earthBlend=Math.Min(240,Math.Max(cell*3f,Math.Abs(originalCenter-p.y)*6f));
                var radius=half+earthBlend;var reach=(int)Math.Ceiling(radius/cell);
                var tangent=math.normalizesafe((NetworkNormalizer.Evaluate(s,Math.Min(1,t+.01f))-NetworkNormalizer.Evaluate(s,Math.Max(0,t-.01f))).xz);
                var leftNormal=new float2(-tangent.y,tangent.x);
                for(var z=Math.Max(0,pz-reach);z<=Math.Min(n-1,pz+reach);z++)for(var x=Math.Max(0,px-reach);x<=Math.Min(n-1,px+reach);x++)
                {
                    var sample=new float2(coordinates[x],coordinates[z]);
                    var station=spanLengthSquared>.00001f?math.clamp(math.dot(sample-previous.xz,span)/spanLengthSquared,0,1):1;
                    var nearest=math.lerp(previous,p,station);
                    var delta=sample-nearest.xz;var distance=math.length(delta);
                    var side=math.dot(delta,leftNormal)>=0?NetworkSides.Left:NetworkSides.Right;
                    // Walls hold the outer bank; earth slopes occupy the available
                    // room. Use the same road height and a smooth bank transition.
                    var wall=(s.RetainingWallSides&side)!=0;var sideRadius=half+(wall?cell*.75f:earthBlend);
                    if(distance>sideRadius)continue;
                    var id=z*n+x;if(protectedMask[id]){protectedSamples++;continue;}
                    var original=source[id]/65535f*scale;if(Math.Abs(nearest.y-original)>budget+.05f)
                    {
                        // At DEM resolution a pixel can be outside a narrow road on
                        // steep terrain. Do not force the entire pixel to the road grade.
                        if(distance<=cell*.5f)beyondBudget++;
                        // Do not truncate the feather: the OUTER blended edit
                        // can be small even when full road height is far away.
                    }
                    var blendT=math.clamp((sideRadius-distance)/Math.Max(.01f,sideRadius-half),0,1);
                    var weight=distance<=half?1:blendT*blendT*(3-2*blendT);
                    if(weight<=0)continue;
                    targets.TryGetValue(id,out var old);
                    // Follow the nearest curve station, not an average of many
                    // uphill/downhill radial stamps. Shared junction profiles
                    // already have one solved elevation; no far stamp can pull
                    // the road core toward a different longitudinal height.
                    if(old.blend==0||distance<old.distance)
                        targets[id]=(nearest.y,budget,weight,distance);
                }
            }
            if(clock.Elapsed.TotalSeconds>=nextProgress){log.Diagnostic($"PROCESS roadbed sections={processed}; affectedSamples={targets.Count}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.0}");nextProgress=clock.Elapsed.TotalSeconds+5;}
        }
        var result=(ushort[])source.Clone();var changed=0;var maximum=0f;
        foreach(var entry in targets)
        {
            token.ThrowIfCancellationRequested();var original=source[entry.Key]/65535f*scale;var target=entry.Value.target;
            var blend=entry.Value.blend;target=original+(target-original)*blend;
            target=math.clamp(target,Math.Max(0,original-entry.Value.budget),Math.Min(scale,original+entry.Value.budget));
            result[entry.Key]=(ushort)Math.Round(target/scale*65535);if(result[entry.Key]!=source[entry.Key])changed++;
            maximum=Math.Max(maximum,Math.Abs(target-original));
        }
        log.Count("Roadbed terrain samples changed",changed);log.Count("Roadbed protected samples avoided",protectedSamples);
        log.Count("Roadbed samples beyond cut/fill budget",beyondBudget);
        log.Diagnostic($"PROCESS roadbed complete; changedSamples={changed}; maximumCutFillMeters={maximum:0.###}; target=nearest-curve-station; shoulderMetersPerFillMeter=6; maximumShoulderMeters=240; zeroWeightAtToe=true; budgetAppliedToBlendedDisplacement=true; protectedWaterCliffsQuarries=true; structuresExcluded=true; elapsedSeconds={clock.Elapsed.TotalSeconds:0.0}");
        return result;
    }
}
