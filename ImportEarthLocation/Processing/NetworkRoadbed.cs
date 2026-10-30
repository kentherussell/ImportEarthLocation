using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

public static class NetworkRoadbed
{
    public sealed record Patch(int X,int Z,int Width,int Height,ushort[] Samples,int Resolution,double MapWidth)
    {
        public float2 Minimum=>new((float)((X/(double)(Resolution-1)-.5)*MapWidth),(float)((Z/(double)(Resolution-1)-.5)*MapWidth));
        public float2 Maximum=>new((float)(((X+Width-1)/(double)(Resolution-1)-.5)*MapWidth),(float)(((Z+Height-1)/(double)(Resolution-1)-.5)*MapWidth));
        public double Encoded(double x,double z)=>TerrainSampling.Encoded(Samples,Width,Height,
            (x-Minimum.x)/Math.Max(1e-8,Maximum.x-Minimum.x),(z-Minimum.y)/Math.Max(1e-8,Maximum.y-Minimum.y));
        public void CopyTo(ushort[] destination)
        {for(var z=0;z<Height;z++)Array.Copy(Samples,z*Width,destination,(Z+z)*Resolution+X,Width);}
    }

    public static Patch BuildLocalPatch(PreparedImport prepared,ushort[] current,IReadOnlyList<PreparedSegment> plan,int start,
        IReadOnlyList<PreparedSegment> revision,ImportLog log,CancellationToken token)
    {
        var baseline=prepared.RoadbedBaseline;var n=TerrainSampling.Resolution(baseline);
        if(current.Length!=baseline.Length||n<2)throw new InvalidOperationException("Local roadbed baseline is unavailable.");
        var lo=new float2(float.PositiveInfinity);var hi=new float2(float.NegativeInfinity);
        foreach(var s in plan.Skip(start).Take(revision.Count).Concat(revision))
            foreach(var p in new[]{s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd}){lo=math.min(lo,p.xz);hi=math.max(hi,p.xz);}
        var width=prepared.Request.MapSize.WidthMeters;var cell=width/(n-1);var margin=240+revision.Max(s=>s.NativeWidth)+2*cell;
        lo-=new float2((float)margin);hi+=new float2((float)margin);
        int Pixel(double x)=>(int)math.clamp(Math.Floor((x/width+.5)*(n-1)),0,n-1);
        var region=new int4(Pixel(lo.x),Pixel(lo.y),Math.Min(n-1,Pixel(hi.x)+1),Math.Min(n-1,Pixel(hi.y)+1));
        var all=plan.ToArray();for(var i=0;i<revision.Count;i++)all[start+i]=revision[i];
        bool Nearby(PreparedSegment s)
        {
            var min=math.min(math.min(s.CourseStart.xz,s.CourseControl1.xz),math.min(s.CourseControl2.xz,s.CourseEnd.xz));
            var max=math.max(math.max(s.CourseStart.xz,s.CourseControl1.xz),math.max(s.CourseControl2.xz,s.CourseEnd.xz));
            var reach=240+Math.Max(0,s.NativeWidth)+2*cell;
            return math.all(max>=lo-(float)reach)&&math.all(min<=hi+(float)reach);
        }
        var neighbours=all.Where(Nearby).ToArray();
        var rebuilt=Prepare(baseline,prepared.HeightScaleMeters,width,neighbours,prepared.RoadbedProtectedAreas,prepared.RoadbedProtectedLines,log,token,region);
        var issued=all.Take(start).Where(s=>!NetworkStructure.IsStructure(s)&&Nearby(s)).ToArray();
        var w=region.z-region.x+1;var h=region.w-region.y+1;var samples=new ushort[w*h];var preserved=0;
        var protectedCores=new bool[w*h];
        foreach(var s in issued)
        {
            var steps=Math.Max(1,(int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(s)/Math.Max(1,cell*.5)));
            var radius=s.NativeWidth/2+cell;var previous=NetworkNormalizer.Evaluate(s,0);
            for(var step=1;step<=steps;step++)
            {
                token.ThrowIfCancellationRequested();var next=NetworkNormalizer.Evaluate(s,step/(float)steps);
                var low=math.min(previous.xz,next.xz)-(float)radius;var high=math.max(previous.xz,next.xz)+(float)radius;
                for(var z=Math.Max(region.y,Pixel(low.y));z<=Math.Min(region.w,Pixel(high.y)+1);z++)
                    for(var x=Math.Max(region.x,Pixel(low.x));x<=Math.Min(region.z,Pixel(high.x)+1);x++)
                    {
                        var point=new float3((float)((x/(double)(n-1)-.5)*width),0,(float)((z/(double)(n-1)-.5)*width));
                        if(NetworkNormalizer.DistanceToSegment(point,previous,next)<=radius)protectedCores[(z-region.y)*w+x-region.x]=true;
                    }
                previous=next;
            }
        }
        for(var z=0;z<h;z++)for(var x=0;x<w;x++)
        {
            token.ThrowIfCancellationRequested();var gx=region.x+x;var gz=region.y+z;var id=gz*n+gx;
            // Preserve already-issued ground cores, including an interpolation
            // cell of support. A local retry owns only its unissued support.
            var edits=prepared.RoadbedLaterEdits;
            var protectedCore=protectedCores[z*w+x]||edits?.Mask.Length==current.Length&&edits.Mask[id]!=0;
            samples[z*w+x]=protectedCore?current[id]:rebuilt[id];if(protectedCore)preserved++;
        }
        log.Diagnostic($"LOCAL ROADBED staged samples={samples.Length}; neighbourSections={neighbours.Length}; issuedCoreSamplesPreserved={preserved}; baselineImmutable=true; nativeMutation=false");
        var patch=new Patch(region.x,region.y,w,h,samples,n,width);
        patch.CopyTo(rebuilt);
        NetworkTerrainSupport.Validate(revision,rebuilt,prepared.HeightScaleMeters,width,log,token,scope:$"native-unit-{revision[0].NativeUnitId}");
        return patch;
    }
    public static ushort[] Refresh(ushort[] baseline,ushort[] previousRoadbed,ushort[] finalRaster,float scale,double width,
        IReadOnlyList<PreparedSegment> previousProfiles,IReadOnlyList<PreparedSegment> finalProfiles,
        IEnumerable<float3[]> protectedAreas,IEnumerable<float3[]> protectedLines,ImportLog log,CancellationToken token,TerrainEditSet? laterEdits=null)
    {
        if(baseline.Length!=previousRoadbed.Length||baseline.Length!=finalRaster.Length)
        {log.Warn("Roadbed refresh raster registration mismatch; final raster retained, no whole-import failure.");return finalRaster;}
        var changed=Math.Abs(previousProfiles.Count-finalProfiles.Count);
        if(changed>0)log.Warn("Roadbed profiles changed during local recovery; refreshing the surviving network against the immutable terrain baseline.");
        for(var i=0;i<Math.Min(previousProfiles.Count,finalProfiles.Count);i++)
        {
            token.ThrowIfCancellationRequested();var a=previousProfiles[i];var b=finalProfiles[i];if(NetworkStructure.IsStructure(b))continue;
            if(math.distance(a.CourseStart,b.CourseStart)>.01f||math.distance(a.CourseControl1,b.CourseControl1)>.01f||
               math.distance(a.CourseControl2,b.CourseControl2)>.01f||math.distance(a.CourseEnd,b.CourseEnd)>.01f||
               Math.Abs(a.NativeWidth-b.NativeWidth)>.01f||NetworkStructure.IsStructure(a)!=NetworkStructure.IsStructure(b))changed++;
        }
        if(changed==0)return finalRaster;
        // Recompute against the immutable pre-earthworks raster. A late profile
        // change must not leave a floating/cut road, nor accumulate another cut
        // budget on top of the first pass. Preserve subsequent water edits exactly.
        var refreshed=Prepare(baseline,scale,width,finalProfiles,protectedAreas,protectedLines,log,token);
        var waterEdits=0;var unclassifiedEdits=0;var conflictingSamples=0;
        for(var i=0;i<refreshed.Length;i++)
        {if((i&4095)==0)token.ThrowIfCancellationRequested();if(finalRaster[i]==previousRoadbed[i])continue;if(refreshed[i]!=baseline[i]&&refreshed[i]!=finalRaster[i])conflictingSamples++;refreshed[i]=finalRaster[i];if(laterEdits?.Mask.Length==refreshed.Length&&laterEdits.Mask[i]!=0)waterEdits++;else unclassifiedEdits++;}
        log.Diagnostic($"PROCESS final-roadbed refreshedGroundProfiles={changed}; laterWaterSamplesRetained={waterEdits}; unclassifiedLaterSamplesRetained={unclassifiedEdits}; producer={laterEdits?.Producer??"unknown"}; baseline={laterEdits?.Baseline??"unknown"}; immutableEarthworkBaseline=true; cutFillBudgetNotAccumulated=true; unknownEditsNotRelabeledWater=true");
        log.Count("Terrain water support composition conflicts",conflictingSamples);
        log.Diagnostic($"TERRAIN COMPOSITION conflictingSamples={conflictingSamples}; preservationPolicy=retain-explicit-later-edit; finalSupportAndRoofValidationRequired=true; unknownEditsPreservedNotCertified=true");
        return refreshed;
    }

    // Work on a copy. Structural spans never become broad terrain embankments.
    // waterSurface: effective sea level in the raster's frame (NaN when unknown),
    // which a tunnel roof fill under the sea never reaches.
    public static ushort[] Prepare(ushort[] source,float scale,double width,IReadOnlyList<PreparedSegment> segments,
        IEnumerable<float3[]> protectedAreas,IEnumerable<float3[]> protectedLines,ImportLog log,CancellationToken token,int4? dirtyRegion=null,float waterSurface=float.NaN)
    {
        var n=(int)Math.Sqrt(source.Length);if(n*n!=source.Length||n<2||width<=0||scale<=0)throw new InvalidOperationException("Invalid roadbed terrain.");
        var cell=(float)(width/(n-1));var protectedMask=new bool[source.Length];var lineGuard=new bool[source.Length];var region=dirtyRegion??new int4(0,0,n-1,n-1);
        // Inside mapped water and quarry polygons only, without the guard bands
        // around them and around cliff, waterway and coast lines: tunnel roof
        // cover may be built there (Marseille's tunnels pass beside cliffs and
        // under buried streams, whose bands left their bores exposed).
        var areaMask=new bool[source.Length];
        var coordinates=new float[n];for(var i=0;i<n;i++)coordinates[i]=(i/(float)(n-1)-.5f)*(float)width;
        int Pixel(float value)=>(int)Math.Round((value/width+.5)*(n-1));
        void Guard(float3 a,float3 b,float margin,bool[]? mask=null)
        {
            var lo=math.min(a.xz,b.xz)-margin;var hi=math.max(a.xz,b.xz)+margin;
            var delta=b.xz-a.xz;var denominator=Math.Max(.001f,math.lengthsq(delta));
            for(var z=Math.Max(region.y,Pixel(lo.y));z<=Math.Min(region.w,Pixel(hi.y));z++)
                for(var x=Math.Max(region.x,Pixel(lo.x));x<=Math.Min(region.z,Pixel(hi.x));x++)
                {
                    var p=new float2(coordinates[x],coordinates[z]);
                    var t=math.clamp(math.dot(p-a.xz,delta)/denominator,0,1);
                    if(math.distance(p,math.lerp(a.xz,b.xz,t))<=margin)(mask??protectedMask)[z*n+x]=true;
                }
        }
        // Protect the actual polygon, not its rectangular envelope. A winding
        // river's bounding box can cover kilometres of dry roads and prevent
        // their entire cut/fill pass from being applied.
        foreach(var ring in protectedAreas)
        {
            token.ThrowIfCancellationRequested();if(ring.Length<3)continue;
            var crossings=new List<float>(ring.Length);
            for(var z=Math.Max(region.y,Pixel(ring.Min(p=>p.z)));z<=Math.Min(region.w,Pixel(ring.Max(p=>p.z)));z++)
            {
                token.ThrowIfCancellationRequested();var y=coordinates[z];crossings.Clear();
                for(var i=0;i<ring.Length;i++){var a=ring[i];var b=ring[(i+1)%ring.Length];if((a.z>y)==(b.z>y))continue;crossings.Add(a.x+(y-a.z)*(b.x-a.x)/(b.z-a.z));}
                crossings.Sort();for(var i=1;i<crossings.Count;i+=2)for(var x=Math.Max(region.x,Pixel(crossings[i-1]));x<=Math.Min(region.z,Pixel(crossings[i]));x++)protectedMask[z*n+x]=areaMask[z*n+x]=true;
            }
            for(var i=0;i<ring.Length;i++)Guard(ring[i],ring[(i+1)%ring.Length],cell);
        }
        // Bands along waterway, cliff and coast lines keep road shoulders and fills
        // off them, but not a ground road's own core and the grid cell beside it,
        // from which the ground under its centre is read: the road stands there in
        // game whatever the ground does, and left unstamped it floated or sank
        // beside bridges over streams (300 of Marseille's 1,455 mismatches).
        foreach(var line in protectedLines)for(var i=1;i<line.Length;i++){token.ThrowIfCancellationRequested();Guard(line[i-1],line[i],cell*2,lineGuard);}
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
        var unsupportedSections=0;var maximumSupportDeficit=0f;
        // A shared ground profile owns one formation, including its
        // median. Do not let two nearest-road stamps independently select the
        // elevation across that same cross-section. Mixed structural modes and
        // explicit walls keep their separate footprints.
        var formations=new List<PreparedSegment>();var consumed=new HashSet<PreparedSegment>();var sharedFormations=0;
        var mixedFormations=0;var incompatibleFormations=0;
        // Unit subdivision retains the parent profile ID. Match its actual
        // reference interval too; the micrometre key absorbs arithmetic noise
        // in exact subdivision, without moving any geometry or terrain sample.
        foreach(var group in segments.Where(s=>s.PairedProfileId!=0).GroupBy(s=>(s.PairedProfileId,
            Begin:Math.Round(Math.Min(s.PairedReferenceBegin,s.PairedReferenceEnd),6),
            End:Math.Round(Math.Max(s.PairedReferenceBegin,s.PairedReferenceEnd),6))))
        {
            token.ThrowIfCancellationRequested();var pair=group.ToArray();
            if(pair.Length!=2||pair[0].PairedReversed==pair[1].PairedReversed)continue;
            if(pair.Any(s=>s.ExplicitWallSides!=0||s.QuaySides!=0))continue;
            var grounds=pair.Where(s=>!NetworkStructure.IsStructure(s)).ToArray();if(grounds.Length==0)continue;
            var mixed=grounds.Length==1;var a=mixed?grounds[0]:pair[0];var b=ReferenceEquals(a,pair[0])?pair[1]:pair[0];
            var aa=new[]{a.CourseStart,a.CourseControl1,a.CourseControl2,a.CourseEnd};
            var bb=new[]{b.CourseEnd,b.CourseControl2,b.CourseControl1,b.CourseStart};
            if(Enumerable.Range(0,4).Any(i=>Math.Abs(aa[i].y-bb[i].y)>.01f)){incompatibleFormations++;continue;}
            var gap=Enumerable.Range(0,4).Max(i=>math.distance(aa[i].xz,bb[i].xz));
            var center=Enumerable.Range(0,4).Select(i=>
            {
                if(!mixed)return(aa[i]+bb[i])*.5f;
                var separation=math.distance(aa[i].xz,bb[i].xz);
                // Ground outer edge to common median bisector. Do not build a
                // full ground platform underneath the structural carriageway.
                var fraction=separation>a.NativeWidth?(1-a.NativeWidth/separation)/4:0;
                return math.lerp(aa[i],bb[i],fraction);
            }).ToArray();
            formations.Add(a with{Start=center[0],Control1=center[1],Control2=center[2],End=center[3],
                StartCurveOffset=default,EndCurveOffset=default,FinalDeckCurve=new(center[0],center[1],center[2],center[3]),
                HorizontalParent=null,NativeWidth=mixed?Math.Max(a.NativeWidth,(gap+a.NativeWidth)/2):gap+Math.Max(a.NativeWidth,b.NativeWidth)});
            consumed.Add(a);
            if(!mixed){consumed.Add(b);sharedFormations++;}
            else
            {
                mixedFormations++;
                // The interpolation footprint is protected too. A coarse grid
                // may consequently fail ground support; retain that conflict
                // rather than bury a bridge or cut a tunnel roof to hide it.
                var steps=Math.Max(1,(int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(b)/Math.Max(1,cell*.5)));
                var previous=NetworkNormalizer.Evaluate(b,0);
                for(var step=1;step<=steps;step++){var next=NetworkNormalizer.Evaluate(b,step/(float)steps);Guard(previous,next,b.NativeWidth/2+cell);previous=next;}
            }
        }
        formations.AddRange(segments.Where(s=>!consumed.Contains(s)&&!NetworkStructure.IsStructure(s)));
        // Bridge abutments and tunnel portals: the approach fill or cutting stays
        // close to the road footprint (a steep, short bank) instead of the broad
        // 1:4 shoulders used elsewhere, widening back to normal over ApproachReach.
        const float ApproachReach=100;var narrowedApproachSamples=0;
        var structureEnds=new Dictionary<(int,int),List<float2>>();
        foreach(var s in segments.Where(NetworkStructure.IsStructure))foreach(var end in new[]{s.CourseStart.xz,s.CourseEnd.xz})
        {var key=((int)Math.Floor(end.x/ApproachReach),(int)Math.Floor(end.y/ApproachReach));if(!structureEnds.TryGetValue(key,out var list))structureEnds[key]=list=new();list.Add(end);}
        float StructureEndDistance(float2 p)
        {
            var best=float.PositiveInfinity;int cx=(int)Math.Floor(p.x/ApproachReach),cz=(int)Math.Floor(p.y/ApproachReach);
            for(var dz=-1;dz<=1;dz++)for(var dx=-1;dx<=1;dx++)if(structureEnds.TryGetValue((cx+dx,cz+dz),out var list))foreach(var end in list)best=Math.Min(best,math.distance(p,end));
            return best;
        }
        log.Quality(new("planned","mixed-mode-formation","roadbed",mixedFormations+incompatibleFormations,0,incompatibleFormations,0,0,
            "Trusted common profile; ground/median half-formation only; structural interpolation support protected; actual raster support checked separately"));
        log.Diagnostic($"ROADBED FORMATIONS sharedGroundProfiles={sharedFormations}; sharedMixedModeHalfFormations={mixedFormations}; incompatibleProfiles={incompatibleFormations}; structuralFootprintsProtected=true; explicitWallsRetainedSeparately=true; profileHeightsNotReoptimized=true");
        foreach(var s in formations)
        {
            token.ThrowIfCancellationRequested();processed++;
            var budget=NetworkVerticalProfiles.CutFillBudget(s);
            var half=s.NativeWidth>0?s.NativeWidth*.5f:Math.Min(12,Math.Max(4,s.Lanes*1.75f));
            var unsupported=false;
            var steps=Math.Max(1,(int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(s)/(cell*.5f)));
            for(var i=0;i<=steps;i++)
            {
                token.ThrowIfCancellationRequested();
                if(clock.Elapsed.TotalSeconds>=nextProgress)
                {log.Diagnostic($"PROCESS roadbed sections={processed}; sectionSample={i}/{steps}; affectedSamples={targets.Count}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.0}");nextProgress=clock.Elapsed.TotalSeconds+5;}
                var t=i/(float)steps;var p=NetworkNormalizer.Evaluate(s,t);var px=Pixel(p.x);var pz=Pixel(p.z);
                var previous=NetworkNormalizer.Evaluate(s,Math.Max(0,(i-1)/(float)steps));
                var span=p.xz-previous.xz;var spanLengthSquared=math.lengthsq(span);
                var originalCenter=NetworkProcessor.Sample(p.x,p.z,source,scale,width);
                var deficit=Math.Abs(originalCenter-p.y)-budget;
                if(deficit>Math.Max(.05f,2*scale/65535f)){unsupported=true;maximumSupportDeficit=Math.Max(maximumSupportDeficit,deficit);}
                // A large embankment needs proportionally more lateral room.
                // Smoothstep's peak derivative is 1.5: six metres of shoulder
                // per metre of fill keeps the added bank slope near 1:4.
                // An unsupported profile must not enlarge its earthworks beyond
                // the displacement the terrain policy actually permits.
                var earthBlend=Math.Min(240,Math.Max(cell*3f,Math.Min(budget,Math.Abs(originalCenter-p.y))*6f));
                if(structureEnds.Count>0)
                {
                    var approach=StructureEndDistance(p.xz);
                    if(approach<ApproachReach){earthBlend=math.lerp(cell*1.5f,earthBlend,approach/ApproachReach);narrowedApproachSamples++;}
                }
                var radius=half+earthBlend;var reach=(int)Math.Ceiling(radius/cell);
                var tangent=math.normalizesafe((NetworkNormalizer.Evaluate(s,Math.Min(1,t+.01f))-NetworkNormalizer.Evaluate(s,Math.Max(0,t-.01f))).xz);
                var leftNormal=new float2(-tangent.y,tangent.x);
                for(var z=Math.Max(region.y,pz-reach);z<=Math.Min(region.w,pz+reach);z++)for(var x=Math.Max(region.x,px-reach);x<=Math.Min(region.z,px+reach);x++)
                {
                    var sample=new float2(coordinates[x],coordinates[z]);
                    var station=spanLengthSquared>.00001f?math.clamp(math.dot(sample-previous.xz,span)/spanLengthSquared,0,1):1;
                    var nearest=math.lerp(previous,p,station);
                    var delta=sample-nearest.xz;var distance=math.length(delta);
                    var side=math.dot(delta,leftNormal)>=0?NetworkSides.Left:NetworkSides.Right;
                    // Walls hold the outer bank; earth slopes occupy the available
                    // room. Use the same road height and a smooth bank transition.
                    // Only evidenced walls justify a narrow, cliff-like shoulder.
                    // Automatically inferred walls are not ground truth and must
                    // not turn otherwise natural earthworks into vertical stamps.
                    var wall=((s.ExplicitWallSides|s.QuaySides)&side)!=0;var sideRadius=half+(wall?cell*.75f:earthBlend);
                    if(distance>sideRadius)continue;
                    var id=z*n+x;if(protectedMask[id]||lineGuard[id]&&distance>half+cell){protectedSamples++;continue;}
                    var original=source[id]/65535f*scale;if(Math.Abs(nearest.y-original)>budget+.05f)
                    {
                        // At DEM resolution a pixel can be outside a narrow road on
                        // steep terrain. Do not force the entire pixel to the road grade.
                        if(distance<=cell*.5f)beyondBudget++;
                        // Retain a bounded, smoothly feathered edit, but report
                        // that it cannot support the requested road height.
                    }
                    var supportedTarget=math.clamp(nearest.y,Math.Max(0,original-budget),Math.Min(scale,original+budget));
                    var blendT=math.clamp((sideRadius-distance)/Math.Max(.01f,sideRadius-half),0,1);
                    var weight=distance<=half?1:blendT*blendT*(3-2*blendT);
                    if(weight<=0)continue;
                    targets.TryGetValue(id,out var old);
                    // Follow the nearest curve station, not an average of many
                    // uphill/downhill radial stamps. Shared junction profiles
                    // already have one solved elevation; no far stamp can pull
                    // the road core toward a different longitudinal height.
                    if(old.blend==0||distance<old.distance)
                    {
                        // Compose the outer edit, not just its raw centre height.
                        // Switching nearest corridors must not discard a stronger
                        // compatible feather and leave a seam between earthworks.
                        if(old.blend>0&&distance>half&&old.distance>half&&Math.Abs(old.target-supportedTarget)<cell)
                        {
                            var combined=old.blend+weight;
                            targets[id]=((old.target*old.blend+supportedTarget*weight)/combined,Math.Min(old.budget,budget),Math.Max(old.blend,weight),distance);
                        }
                        else targets[id]=(supportedTarget,budget,weight,distance);
                    }
                }
            }
            if(unsupported)unsupportedSections++;
            if(clock.Elapsed.TotalSeconds>=nextProgress){log.Diagnostic($"PROCESS roadbed sections={processed}; affectedSamples={targets.Count}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.0}");nextProgress=clock.Elapsed.TotalSeconds+5;}
        }
        // Under a bridge deck: city elevation data carries viaducts and their
        // embankments (the A7 into Marseille), and a deck sat in the ground it
        // should stand over. Away from its ends ground reaching within BridgeUnder
        // of a deck (its structure depth) is lowered to that, by at most BridgeCut, across the
        // deck and feathered over two cells beyond it. The clearance falls to
        // zero over BridgeUnderRamp metres toward each end so the abutment meets
        // the deck. Cells of a ground formation or protected feature are kept.
        const float BridgeUnder=2.5f,BridgeCut=10,BridgeUnderRamp=40;var under=new Dictionary<int,float>();
        static float Ramp(float x){x=math.clamp(x,0,1);return x*x*(3-2*x);}
        foreach(var bridge in segments.Where(s=>s.Bridge&&!s.Tunnel))
        {
            token.ThrowIfCancellationRequested();
            var length=NetworkVerticalProfiles.ArcLength(bridge);var steps=Math.Max(1,(int)Math.Ceiling(length/Math.Max(1,cell*.5f)));
            var core=bridge.NativeWidth>0?bridge.NativeWidth*.5f:6;var edge=core+cell*2;var r=(int)Math.Ceiling(edge/cell);
            for(var i=0;i<=steps;i++)
            {
                var t=i/(float)steps;var clearance=BridgeUnder*Math.Min(1,Math.Min(t,1-t)*length/BridgeUnderRamp);if(clearance<=0)continue;
                var p=NetworkNormalizer.Evaluate(bridge,t);var px=Pixel(p.x);var pz=Pixel(p.z);
                for(var z=Math.Max(region.y,pz-r);z<=Math.Min(region.w,pz+r);z++)for(var x=Math.Max(region.x,px-r);x<=Math.Min(region.z,px+r);x++)
                {
                    var id=z*n+x;var distance=math.distance(new float2(coordinates[x],coordinates[z]),p.xz);
                    if(distance>edge||protectedMask[id]||lineGuard[id]||targets.ContainsKey(id))continue;
                    var original=source[id]/65535f*scale;var target=Math.Max(p.y-clearance,original-BridgeCut);if(target>=original)continue;
                    var blend=distance<=core?1:Ramp(1-(distance-core)/(edge-core));target=original+(target-original)*blend;
                    under[id]=under.TryGetValue(id,out var old)?Math.Min(old,target):target;
                }
            }
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
        foreach(var (id,target) in under)
        {var value=(ushort)Math.Round(math.clamp(target/scale,0,1)*65535);if(value<result[id]){result[id]=value;changed++;}}
        log.Count("Roadbed samples lowered under bridge decks",under.Count);
        // Tunnel roofs: where the ground (or the sea floor) lies below a tunnel's
        // roof the bore shows (the A55 by Marseille's Old Port, whose tunnels
        // cannot dive deep enough before reaching the water). Cover is built up to
        // the roof over the tunnel's footprint, by at most RoofFill above the
        // original ground, never above 1 m under the sea surface, and never in
        // mapped water, cliffs or quarries. RoofFill covers a cut-and-cover
        // tunnel whose open trench the elevation data shows (the A507 by the A50):
        // its deck lies above the trench floor, so 4 m left the roof exposed.
        // The cover stands full height out to one grid cell beyond the bore's
        // edge, so bilinear terrain across the bore never dips below its roof,
        // then falls 1 m per 2 m over RoofShoulder cells instead of stopping in
        // a step. Only mapped water and quarry interiors (areaMask) are kept;
        // the guard bands around cliffs, streams and shores are not.
        // RoofHeight matches the profile's TunnelDepth: the game draws a tunnel
        // as an open cut unless the ground stands about 12 m above its deck, and
        // 8 m of cover left 2.8 km of the A507's covered trenches drawn open.
        // Cover never rises above the formation of a ground road over the
        // tunnel, which would bury that road.
        const float RoofHeight=13,RoofFill=10,RoofSlope=.5f;const int RoofShoulder=3;var roofSamples=0;
        foreach(var tunnel in segments.Where(s=>s.Tunnel))
        {
            token.ThrowIfCancellationRequested();
            var core=(tunnel.NativeWidth>0?tunnel.NativeWidth*.5f:6)+cell;var reach=core+RoofShoulder*cell;
            var steps=Math.Max(1,(int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(tunnel)/Math.Max(1,cell*.5f)));
            for(var i=0;i<=steps;i++)
            {
                var p=NetworkNormalizer.Evaluate(tunnel,i/(float)steps);var r=(int)Math.Ceiling(reach/cell);var px=Pixel(p.x);var pz=Pixel(p.z);
                for(var z=Math.Max(region.y,pz-r);z<=Math.Min(region.w,pz+r);z++)for(var x=Math.Max(region.x,px-r);x<=Math.Min(region.z,px+r);x++)
                {
                    var id=z*n+x;var distance=math.distance(new float2(coordinates[x],coordinates[z]),p.xz);if(areaMask[id]||distance>reach)continue;
                    var original=source[id]/65535f*scale;var current=result[id]/65535f*scale;
                    var target=Math.Min(p.y+RoofHeight-Math.Max(0,distance-core)*RoofSlope,original+RoofFill);
                    if(math.isfinite(waterSurface)&&original<waterSurface)target=Math.Min(target,waterSurface-1);
                    if(targets.TryGetValue(id,out var formation)&&formation.blend>=1)target=Math.Min(target,formation.target);
                    if(target<=current)continue;
                    result[id]=(ushort)Math.Round(math.clamp(target/scale,0,1)*65535);roofSamples++;
                }
            }
        }
        log.Count("Roadbed tunnel roof samples filled",roofSamples);
        log.Count("Roadbed terrain samples changed",changed);log.Count("Roadbed protected samples avoided",protectedSamples);
        log.Count("Roadbed samples beyond cut/fill budget",beyondBudget);
        log.Count("Roadbed ground formations beyond support budget",unsupportedSections);
        log.Count("Roadbed approach samples kept to the footprint",narrowedApproachSamples);
        log.Diagnostic($"ROADBED SUPPORT sampledUnsupportedFormations={unsupportedSections}; maximumCenterlineDeficitMeters={maximumSupportDeficit:0.###}; boundedEarthworkNotSupportAcceptance=true; continuousFinalSupportAuditStillRequired=true");
        log.Diagnostic($"PROCESS roadbed complete; changedSamples={changed}; maximumCutFillMeters={maximum:0.###}; target=nearest-curve-station; shoulderMetersPerFillMeter=6; maximumShoulderMeters=240; zeroWeightAtToe=true; targetAndShoulderBoundedBeforeFeather=true; protectedWaterCliffsQuarries=true; structuresExcluded=true; elapsedSeconds={clock.Elapsed.TotalSeconds:0.0}");
        return result;
    }
}
