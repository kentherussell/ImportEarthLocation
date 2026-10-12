using System;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using ImportEarthLocation.Geo;

namespace ImportEarthLocation.Processing;

public static class CoastalTerrain
{
    public sealed record CoastCoverage(int Width,int Height,int Zoom,int OriginX,int OriginY,
        bool[] Ocean,bool[] Ambiguous,bool[] Unseeded,float[] ShoreDistance,int AcceptedComponents,int ConflictingComponents);
    // OSM coastline direction: land on left, sea on right. Missing or contradictory
    // coast coverage is not permission to flood-fill land from negative DEM height.
    public static void Shape(float[] heights, int width, int height, int zoom, int originX, int originY,
        float spacing, OsmDataset? geography, CancellationToken token, Action<string>? report, Action<CoastCoverage>? capture = null)
    {
        if (geography == null) return;
        var barrier = new bool[heights.Length]; var seeds = new byte[heights.Length];
        var seedDistance = new double[heights.Length];
        for (var i = 0; i < seedDistance.Length; i++) seedDistance[i] = double.PositiveInfinity;
        var coastEdges = 0;
        var oriented=new System.Collections.Generic.List<(double ax,double ay,double bx,double by)>();
        foreach (var way in geography.Ways.Values.Where(w => w.Tags.TryGetValue("natural", out var value) && value == "coastline"))
        {
            for (var k = 1; k < way.NodeIds.Length; k++)
            {
                token.ThrowIfCancellationRequested();
                if (!geography.Nodes.TryGetValue(way.NodeIds[k - 1], out var a) || !geography.Nodes.TryGetValue(way.NodeIds[k], out var b)) continue;
                var ax = GeoMath.LongitudeToTileX(a.Longitude, zoom) * 256 - originX;
                var ay = GeoMath.LatitudeToTileY(a.Latitude, zoom) * 256 - originY;
                var bx = GeoMath.LongitudeToTileX(b.Longitude, zoom) * 256 - originX;
                var by = GeoMath.LatitudeToTileY(b.Latitude, zoom) * 256 - originY;
                var dx = bx - ax; var dy = by - ay; var length = Math.Sqrt(dx * dx + dy * dy);
                if(TerrainConditioner.Finite((float)length)&&length>.001&&length<=100000)
                    oriented.Add((ax,ay,bx,by));
                if (!TerrainConditioner.Finite((float)length) || length < .001 || Math.Max(ax, bx) < -5 || Math.Min(ax, bx) > width + 5 || Math.Max(ay, by) < -5 || Math.Min(ay, by) > height + 5) continue;
                if (length > 100000) continue; // Malformed geography cannot create an unbounded loop.
                coastEdges++;
                var count = (int)Math.Ceiling(length * 2);
                for (var j = 0; j <= count; j++)
                {
                    if ((j & 1023) == 0) token.ThrowIfCancellationRequested();
                    var x = ax + dx * j / count; var y = ay + dy * j / count;
                    var ix = (int)Math.Round(x); var iy = (int)Math.Round(y);
                    for (var yy = iy - 1; yy <= iy + 1; yy++) for (var xx = ix - 1; xx <= ix + 1; xx++)
                        if (xx >= 0 && xx < width && yy >= 0 && yy < height) barrier[yy * width + xx] = true;
                    Mark(x - dy / length * 4, y + dx / length * 4, 1, ax, ay, bx, by); // Sea, right in image coordinates.
                    Mark(x + dy / length * 4, y - dx / length * 4, 2, ax, ay, bx, by); // Land.
                }
            }
        }
        if (coastEdges == 0) return;
        var excluded = new bool[heights.Length];
        foreach (var way in geography.Ways.Values)
        {
            token.ThrowIfCancellationRequested();
            if (!(way.Tags.TryGetValue("natural", out var natural) && natural == "water" || way.Tags.TryGetValue("landuse", out var landuse) && landuse == "reservoir")) continue;
            if (way.Tags.TryGetValue("water", out var water) && (water == "sea" || water == "bay")) continue;
            if (way.NodeIds.Length < 4 || way.NodeIds[0] != way.NodeIds[way.NodeIds.Length - 1]) continue;
            ExcludeEnvelope(way.NodeIds);
        }
        var protectedRelations = 0;
        foreach (var relation in geography.Relations)
        {
            token.ThrowIfCancellationRequested();
            if (!(relation.Tags.TryGetValue("natural", out var natural) && natural == "water" || relation.Tags.TryGetValue("landuse", out var landuse) && landuse == "reservoir")) continue;
            if (relation.Tags.TryGetValue("water", out var water) && (water == "sea" || water == "bay")) continue;
            // Join relation rings by source-node identity, never by proximity.
            // A failed relation retains the conservative envelope guard; only
            // fully assembled rings can replace it with an exact hole-aware mask.
            var outer=Join(relation.Members.Where(m=>!m.inner).Select(m=>m.wayId));
            var inner=Join(relation.Members.Where(m=>m.inner).Select(m=>m.wayId));
            if(outer!=null&&outer.Count>0&&inner!=null)
            {
                var outerPoints=outer.Select(Project).ToArray(); var innerPoints=inner.Select(Project).ToArray();
                var all=outerPoints.SelectMany(p=>p).ToArray();
                RasterExclude(all,(x,y)=>outerPoints.Any(p=>TerrainGeographyGuard.Inside(x,y,p))&&!innerPoints.Any(p=>TerrainGeographyGuard.Inside(x,y,p)));
            }
            else ExcludeEnvelope(relation.Members.Where(m=>!m.inner&&geography.Ways.ContainsKey(m.wayId)).SelectMany(m=>geography.Ways[m.wayId].NodeIds),false);
            protectedRelations++;
        }
        System.Collections.Generic.List<long[]>? Join(System.Collections.Generic.IEnumerable<long> ids)
        {
            var remaining=new System.Collections.Generic.List<System.Collections.Generic.List<long>>();
            foreach(var id in ids) { if(!geography.Ways.TryGetValue(id,out var way)||way.NodeIds.Length<2||way.NodeIds.Any(n=>!geography.Nodes.ContainsKey(n)))return null;remaining.Add(way.NodeIds.ToList()); }
            var rings=new System.Collections.Generic.List<long[]>();
            while(remaining.Count>0)
            {
                token.ThrowIfCancellationRequested();
                var chain=remaining[0];remaining.RemoveAt(0);
                while(chain[0]!=chain[chain.Count-1])
                {
                    token.ThrowIfCancellationRequested();
                    var matches=remaining.Where(r=>r[0]==chain[chain.Count-1]||r[r.Count-1]==chain[chain.Count-1]).ToArray();
                    if(matches.Length!=1)return null;
                    var next=matches[0];remaining.Remove(next);if(next[0]!=chain[chain.Count-1])next.Reverse();chain.AddRange(next.Skip(1));
                }
                if(chain.Count<4)return null;rings.Add(chain.ToArray());
            }
            return rings;
        }
        (double x,double y)[] Project(System.Collections.Generic.IEnumerable<long> nodeIds)=>nodeIds.Where(geography.Nodes.ContainsKey).Select(id=>geography.Nodes[id])
            .Select(p=>(x:GeoMath.LongitudeToTileX(p.Longitude,zoom)*256-originX,y:GeoMath.LatitudeToTileY(p.Latitude,zoom)*256-originY)).ToArray();
        void ExcludeEnvelope(System.Collections.Generic.IEnumerable<long> nodeIds,bool exact=true)
        {
            var points=Project(nodeIds).Where(p=>!double.IsNaN(p.x)&&!double.IsInfinity(p.x)&&!double.IsNaN(p.y)&&!double.IsInfinity(p.y)).ToArray();
            if (points.Length < 3) return;
            RasterExclude(points,(x,y)=>!exact||TerrainGeographyGuard.Inside(x,y,points));
        }
        void RasterExclude((double x,double y)[] points,Func<double,double,bool> contains)
        {
            if(points.Length<3||points.Any(p=>double.IsNaN(p.x)||double.IsInfinity(p.x)||double.IsNaN(p.y)||double.IsInfinity(p.y)))return;
            var left=Math.Max(0,(int)Math.Floor(points.Min(p=>p.x)));var right=Math.Min(width-1,(int)Math.Ceiling(points.Max(p=>p.x)));
            var top=Math.Max(0,(int)Math.Floor(points.Min(p=>p.y)));var bottom=Math.Min(height-1,(int)Math.Ceiling(points.Max(p=>p.y)));
            for(var y=top;y<=bottom;y++){token.ThrowIfCancellationRequested();for(var x=left;x<=right;x++)if(contains(x,y))excluded[y*width+x]=true;}
        }
        var visited = new bool[heights.Length]; var ocean = new bool[heights.Length]; var queue = new int[heights.Length];
        var accepted = 0; var ambiguous = 0; var unseeded = 0;
        long oceanCells=0,ambiguousCells=0,unseededCells=0,landCells=0;
        var ambiguousMask=capture!=null ? new bool[heights.Length] : Array.Empty<bool>();
        var unseededMask=capture!=null ? new bool[heights.Length] : Array.Empty<bool>();
        var barrierSeeds=0;
        // All barriers must be finished before seeds are accepted. Tight bends can
        // place a valid side seed inside a later segment's barrier; it is not a
        // trustworthy land/sea vote and must not poison component classification.
        for(var i=0;i<seeds.Length;i++) if(barrier[i]&&seeds[i]!=0){seeds[i]=0;barrierSeeds++;}
        for (var seed = 0; seed < heights.Length; seed++)
        {
            if ((seed & 16383) == 0) token.ThrowIfCancellationRequested();
            if (visited[seed] || barrier[seed]) continue;
            var head = 0; var count = 1; byte labels = 0; queue[0] = seed; visited[seed] = true;
            while (head < count)
            {
                if ((head & 4095) == 0) token.ThrowIfCancellationRequested();
                var i = queue[head++]; labels |= seeds[i]; var x = i % width; var y = i / width;
                Add(x > 0 ? i - 1 : -1); Add(x + 1 < width ? i + 1 : -1); Add(y > 0 ? i - width : -1); Add(y + 1 < height ? i + width : -1);
                void Add(int next) { if (next < 0 || visited[next] || barrier[next]) return; visited[next] = true; queue[count++] = next; }
            }
            if (labels == 3) { ambiguous++; ambiguousCells+=count; }
            if(labels==0){unseeded++;unseededCells+=count;}
            if(labels==2)landCells+=count;
            if(capture!=null && (labels==3||labels==0)) for(var k=0;k<count;k++)
                if(labels==3)ambiguousMask[queue[k]]=true;else unseededMask[queue[k]]=true;
            if (labels != 1) continue;
            accepted++; for (var k = 0; k < count; k++) if(!excluded[queue[k]]) {ocean[queue[k]] = true;oceanCells++;}
        }
        // Classify scanlines directly from oriented coastline crossings. A single
        // missing raster barrier or wrong-side seed must not poison an entire sea
        // component. Crossings outside the crop still determine the interior side;
        // no DEM sign or nearest-coast heuristic is used to declare land ocean.
        var scanlineRepairs=0L;var classifiedRows=0;
        var classified=new bool[heights.Length];var provenLand=new bool[heights.Length];
        var bands=new System.Collections.Generic.Dictionary<int,System.Collections.Generic.List<(double ax,double ay,double bx,double by)>>();
        foreach(var e in oriented)
            for(var y=Math.Max(0,(int)Math.Ceiling(Math.Min(e.ay,e.by)));y<Math.Min(height,Math.Ceiling(Math.Max(e.ay,e.by)));y++)
            {if(!bands.TryGetValue(y,out var band))bands[y]=band=new();band.Add(e);}
        for(var y=0;y<height;y++)
        {
            token.ThrowIfCancellationRequested();if(!bands.TryGetValue(y,out var band))continue;
            var crossings=band.Select(e=>(x:e.ax+(y-e.ay)*(e.bx-e.ax)/(e.by-e.ay),rightSea:e.by<e.ay)).OrderBy(e=>e.x).ToArray();
            if(crossings.Length==0)continue;classifiedRows++;
            var next=0;var sea=!crossings[0].rightSea;
            for(var x=0;x<width;x++)
            {
                while(next<crossings.Length&&crossings[next].x<=x){sea=crossings[next].rightSea;next++;}
                var i=y*width+x;if(barrier[i]||excluded[i])continue;
                if(ocean[i]!=sea)scanlineRepairs++;ocean[i]=sea;classified[i]=true;provenLand[i]=!sea;
            }
        }
        // The orthogonal sweep supplies evidence beyond the latitude extent of
        // a cropped coastline. A row with no crossings is not an artificial shore.
        var columns=new System.Collections.Generic.Dictionary<int,System.Collections.Generic.List<(double ax,double ay,double bx,double by)>>();
        foreach(var e in oriented)if(e.ax!=e.bx)
            for(var x=Math.Max(0,(int)Math.Ceiling(Math.Min(e.ax,e.bx)));x<Math.Min(width,Math.Ceiling(Math.Max(e.ax,e.bx)));x++)
            {if(!columns.TryGetValue(x,out var band))columns[x]=band=new();band.Add(e);}
        var verticalRepairs=0;
        for(var x=0;x<width;x++)
        {
            token.ThrowIfCancellationRequested();if(!columns.TryGetValue(x,out var band))continue;
            var crossings=band.Select(e=>(y:e.ay+(x-e.ax)*(e.by-e.ay)/(e.bx-e.ax),belowSea:e.bx>e.ax)).OrderBy(e=>e.y).ToArray();
            var next=0;var sea=!crossings[0].belowSea;
            for(var y=0;y<height;y++)
            {
                while(next<crossings.Length&&crossings[next].y<=y){sea=crossings[next].belowSea;next++;}
                var i=y*width+x;if(barrier[i]||excluded[i]||classified[i])continue;
                ocean[i]=sea;classified[i]=true;provenLand[i]=!sea;verticalRepairs++;
            }
        }
        // Resolve remaining connected regions only from unanimous directed
        // coastline witnesses. No DEM sign or nearest-coast flood heuristic.
        Array.Clear(visited,0,visited.Length);var propagated=0;
        for(var seed=0;seed<heights.Length;seed++)
        {
            if((seed&16383)==0)token.ThrowIfCancellationRequested();
            if(visited[seed]||barrier[seed]||excluded[seed])continue;
            var head=0;var count=1;byte votes=0;queue[0]=seed;visited[seed]=true;
            while(head<count)
            {
                if((head&4095)==0)token.ThrowIfCancellationRequested();var i=queue[head++];var x=i%width;var y=i/width;
                if(classified[i])votes|=(byte)(ocean[i]?1:2);
                void Add(int k){if(k<0||visited[k]||barrier[k]||excluded[k])return;visited[k]=true;queue[count++]=k;}
                Add(x>0?i-1:-1);Add(x+1<width?i+1:-1);Add(y>0?i-width:-1);Add(y+1<height?i+width:-1);
            }
            if(votes!=1&&votes!=2)continue;
            for(var k=0;k<count;k++){var i=queue[k];if(classified[i])continue;ocean[i]=votes==1;provenLand[i]=votes==2;classified[i]=true;propagated++;}
        }
        if(capture!=null)for(var i=0;i<heights.Length;i++)if(classified[i])ambiguousMask[i]=unseededMask[i]=false;
        oceanCells=ocean.LongCount(v=>v);
        report?.Invoke($"DEM COAST RECONCILIATION classifiedRows={classifiedRows}/{height}; orthogonalCells={verticalRepairs}; propagatedWitnessCells={propagated}; orientedEdges={oriented.Count}; changedClassificationCells={scanlineRepairs}; acceptedOceanCells={oceanCells}; negativeHeightFloodFill=false; syntheticOffshoreDepth=true");
        // Two-pass shoreline distance. No artificial slope toward the map's crop edge.
        var distance = new float[heights.Length];
        for (var i = 0; i < distance.Length; i++) distance[i] = barrier[i]||provenLand[i] ? 0 : float.PositiveInfinity;
        var diagonal = spacing * 1.41421356f;
        for (var y = 0; y < height; y++)
        {
            token.ThrowIfCancellationRequested();
            for (var x = 0; x < width; x++)
            {
                var i = y * width + x;
                if (x > 0) distance[i] = Math.Min(distance[i], distance[i - 1] + spacing);
                if (y > 0)
                {
                    distance[i] = Math.Min(distance[i], distance[i - width] + spacing);
                    if (x > 0) distance[i] = Math.Min(distance[i], distance[i - width - 1] + diagonal);
                    if (x + 1 < width) distance[i] = Math.Min(distance[i], distance[i - width + 1] + diagonal);
                }
            }
        }
        var changed = 0;
        for (var y = height - 1; y >= 0; y--)
        {
            token.ThrowIfCancellationRequested();
            for (var x = width - 1; x >= 0; x--)
            {
                var i = y * width + x;
                if (x + 1 < width) distance[i] = Math.Min(distance[i], distance[i + 1] + spacing);
                if (y + 1 < height) distance[i] = Math.Min(distance[i], distance[i + width] + spacing);
                if (y + 1 < height && x > 0) distance[i] = Math.Min(distance[i], distance[i + width - 1] + diagonal);
                if (y + 1 < height && x + 1 < width) distance[i] = Math.Min(distance[i], distance[i + width + 1] + diagonal);
                if (!ocean[i] || excluded[i] || !TerrainConditioner.Finite(distance[i])) continue;
                var target = -Math.Min(180, .5f + distance[i] * .015f);
                // Terrarium is not verified bathymetry here. A modeled ocean must
                // not retain a fraction of an 11km corrupt floor at its coastline.
                heights[i] = target; changed++;
            }
        }
        var coastalRimRepairs = 0;
        for (var y = 3; y < height - 3; y++)
        {
            token.ThrowIfCancellationRequested();
            for (var x = 3; x < width - 3; x++)
            {
                var i = y * width + x;
                if (!barrier[i] || excluded[i] || heights[i] >= -.5f) continue;
                var seaNeighbor = false;
                for (var dy = -3; dy <= 3 && !seaNeighbor; dy++) for (var dx = -3; dx <= 3; dx++)
                    if (ocean[(y + dy) * width + x + dx]) { seaNeighbor = true; break; }
                if (!seaNeighbor) continue;
                heights[i] = -.5f; coastalRimRepairs++;
            }
        }
        report?.Invoke($"DEM COAST model=oriented-coastline-offshore-profile; edges={coastEdges}; oceanComponents={accepted}; conflictingComponentsSkipped={ambiguous}; unseededComponents={unseeded}; acceptedOceanCells={oceanCells}; conflictingCells={ambiguousCells}; unseededCells={unseededCells}; provenLandCells={landCells}; discardedBarrierSeeds={barrierSeeds}; totalCells={heights.Length}; modeledSamples={changed}; coastalRimSamples={coastalRimRepairs}; maximumModeledDepthMeters=180; depthSlope=.015; distance=8-neighbor-chamfer; seedPolicy=nearest-shore-segment; mappedInlandWaterEnvelopesProtected=true; inlandWaterRelations={protectedRelations}; nonOceanLandPreserved=true; incompleteCoastCoverageMayLimitRepair=true; realBathymetryClaimed=false");
        capture?.Invoke(new CoastCoverage(width,height,zoom,originX,originY,ocean,ambiguousMask,unseededMask,distance,accepted,ambiguous));
        void Mark(double x, double y, byte label, double ax, double ay, double bx, double by)
        {
            var ix = (int)Math.Round(x); var iy = (int)Math.Round(y);
            if (ix < 0 || ix >= width || iy < 0 || iy >= height) return;
            var dx = bx - ax; var dy = by - ay;
            var t = Math.Max(0, Math.Min(1, ((ix - ax) * dx + (iy - ay) * dy) / (dx * dx + dy * dy)));
            var ex = ix - ax - t * dx; var ey = iy - ay - t * dy; var squared = ex * ex + ey * ey;
            var i = iy * width + ix;
            // Offset seeds can overlap on tight bends/inlets. A more distant
            // opposite shoreline must not poison an entire connected ocean.
            if (squared + 1e-6 < seedDistance[i]) { seedDistance[i] = squared; seeds[i] = label; }
            else if (Math.Abs(squared - seedDistance[i]) <= 1e-6) seeds[i] |= label;
        }
    }
}
