using System;
using System.Collections.Generic;
using System.Threading;

namespace ImportEarthLocation.Processing;

/// <summary>Bounded source-grid repairs across tile boundaries, before encoding.</summary>
public static class TerrainArtifactRepair
{
    public static void Trace(float[] raster, int width, string phase, Action<string>? report)
    {
        if (raster.Length == 0 || width <= 0)
        { report?.Invoke($"DEM PHASE phase={phase}; extremaAvailable=false; reason=empty-or-invalid-grid"); return; }
        float minimum = float.PositiveInfinity, maximum = float.NegativeInfinity; var minCell = -1; var maxCell = -1; var invalid = 0;
        for (var i = 0; i < raster.Length; i++)
        {
            var h = raster[i]; if (!TerrainConditioner.ValidEarthElevation(h)) invalid++;
            if (!TerrainConditioner.Finite(h)) continue;
            if (h < minimum) { minimum = h; minCell = i; } if (h > maximum) { maximum = h; maxCell = i; }
        }
        report?.Invoke($"DEM PHASE phase={phase}; minimum={minimum:R}; minimumCell={minCell % width},{minCell / width}; maximum={maximum:R}; maximumCell={maxCell % width},{maxCell / width}; invalidSamples={invalid}; coordinates=required-source-grid");
    }
    public static void TraceEncoded(ushort[] raster, float scale, float datum, string phase, Action<string>? report)
    {
        if (raster.Length == 0)
        { report?.Invoke($"DEM PHASE phase={phase}; extremaAvailable=false; reason=empty-grid"); return; }
        var width = (int)Math.Sqrt(raster.Length); var minCell = 0; var maxCell = 0;
        for (var i = 1; i < raster.Length; i++)
        { if (raster[i] < raster[minCell]) minCell = i; if (raster[i] > raster[maxCell]) maxCell = i; }
        report?.Invoke($"DEM PHASE phase={phase}; minimumEarthMeters={raster[minCell] / 65535f * scale - datum:R}; minimumCell={minCell % width},{minCell / width}; maximumEarthMeters={raster[maxCell] / 65535f * scale - datum:R}; maximumCell={maxCell % width},{maxCell / width}; coordinates=encoded-output-grid; resolution={width}");
    }
    public static void Repair(float[] raster, int width, int height, float spacing, CancellationToken token, Action<string>? report, Func<int, bool>? protectedSample = null, float supportMeters = 180)
    {
        var invalid = new HashSet<int>();
        for (var i = 0; i < raster.Length; i++)
        { if ((i & 16383) == 0) token.ThrowIfCancellationRequested(); if (!TerrainConditioner.ValidEarthElevation(raster[i])) invalid.Add(i); }
        var repaired = 0;
        if (invalid.Count > Math.Max(64, raster.Length / 1000))
            throw new InvalidOperationException("DEM recovery: invalid samples exceed the bounded source-area budget.");
        while (invalid.Count > 0)
        {
            token.ThrowIfCancellationRequested(); var seed = -1; foreach (var i in invalid) { seed = i; break; }
            invalid.Remove(seed); var cluster = new List<int> { seed };
            var x0 = seed % width; var x1 = x0; var y0 = seed / width; var y1 = y0;
            for (var k = 0; k < cluster.Count; k++)
            {
                token.ThrowIfCancellationRequested(); var i = cluster[k]; var x = i % width; var y = i / width;
                x0 = Math.Min(x0, x); x1 = Math.Max(x1, x); y0 = Math.Min(y0, y); y1 = Math.Max(y1, y);
                void Add(int next) { if (next >= 0 && invalid.Remove(next)) cluster.Add(next); }
                Add(x > 0 ? i - 1 : -1); Add(x + 1 < width ? i + 1 : -1); Add(y > 0 ? i - width : -1); Add(y + 1 < height ? i + width : -1);
                if (cluster.Count > 64) throw new InvalidOperationException("DEM recovery: invalid contiguous source region exceeds 64 cells; no trustworthy bounded interpolation exists.");
            }
            if (x0 == 0 || y0 == 0 || x1 + 1 == width || y1 + 1 == height || x1 - x0 > 16 || y1 - y0 > 16)
                throw new InvalidOperationException("DEM recovery: corrupt source region lacks a complete bounded surrounding ring.");
            var ring = new List<(int X, int Y, float H)>();
            for (var y = y0 - 1; y <= y1 + 1; y++) for (var x = x0 - 1; x <= x1 + 1; x++)
                if (x == x0 - 1 || x == x1 + 1 || y == y0 - 1 || y == y1 + 1)
                { var h = raster[y * width + x]; if (TerrainConditioner.ValidEarthElevation(h)) ring.Add((x, y, h)); }
            if (ring.Count < 8) throw new InvalidOperationException("DEM recovery: insufficient valid neighboring tile support.");
            float lo = float.PositiveInfinity, hi = float.NegativeInfinity;
            var sides = 0;
            foreach (var p in ring)
            { lo = Math.Min(lo, p.H); hi = Math.Max(hi, p.H); if (p.X == x0 - 1) sides |= 1; if (p.X == x1 + 1) sides |= 2; if (p.Y == y0 - 1) sides |= 4; if (p.Y == y1 + 1) sides |= 8; }
            if (sides != 15 || hi - lo > Math.Max(30, spacing * 2))
                throw new InvalidOperationException("DEM recovery: source ring lacks coherent support on all four sides.");
            // Invalid values cannot be retained even inside protected features. Restore
            // only this invalid footprint from its valid rim; finite protected samples
            // remain untouched. A local plane prevents a constant-height slope stamp.
            var plane = FitPlane(ring); var protectedInvalid = 0;
            foreach (var i in cluster)
            {
                double sum = 0, weights = 0;
                foreach (var p in ring) { var dx = p.X - i % width; var dy = p.Y - i / width; var weight = 1d / (dx * dx + dy * dy); sum += (p.H - PlaneHeight(plane,p.X,p.Y)) * weight; weights += weight; }
                raster[i] = (float)Math.Max(lo,Math.Min(hi,PlaneHeight(plane,i%width,i/width)+sum/weights)); repaired++;
                if(protectedSample?.Invoke(i)==true) protectedInvalid++;
            }
            report?.Invoke($"DEM MOSAIC REPAIR clusterCells={cluster.Count}; bounds={x0},{y0}..{x1},{y1}; ringSamples={ring.Count}; protectedInvalidSamples={protectedInvalid}; finiteProtectedSamplesPreserved=true; tileEdgesSupported=true; method=bounded-plane-residual-interpolation");
        }
        // Multi-scale isolated extrema: all four opposing directions must return
        // to a consistent neighborhood. Long ridges/cliffs fail this criterion.
        var baseline = (float[])raster.Clone(); var spikes = 0; var pits = 0; var details = 0;
        var changed = new bool[raster.Length];
        var neighbors = new float[16];
        var directions = new[] { (-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1) };
        var radius = Math.Max(3, Math.Min(24, (int)Math.Ceiling(supportMeters / spacing)));
        for (var y = radius; y < height - radius; y++)
        {
            token.ThrowIfCancellationRequested();
            for (var x = radius; x < width - radius; x++)
            {
                var i = y * width + x; var value = baseline[i]; var k = 0;
                if (changed[i] || protectedSample?.Invoke(i) == true) continue;
                var nearLow = Math.Min(Math.Min(baseline[i - 1], baseline[i + 1]), Math.Min(baseline[i - width], baseline[i + width]));
                var nearHigh = Math.Max(Math.Max(baseline[i - 1], baseline[i + 1]), Math.Max(baseline[i - width], baseline[i + width]));
                // A resampled multi-cell artifact need not jump abruptly in one
                // cell. Require a local extremum plus coherent enclosing support.
                if (value < nearHigh && value > nearLow) continue;
                var a = baseline[i - radius]; var b = baseline[i + radius];
                var c = baseline[i - radius * width]; var d0 = baseline[i + radius * width];
                var low = Math.Min(Math.Min(a, b), Math.Min(c, d0)); var high = Math.Max(Math.Max(a, b), Math.Max(c, d0));
                var threshold = Math.Max(40, spacing * 2);
                if (value >= low - threshold && value <= high + threshold) continue;
                for (var r = radius - 1; r <= radius; r++)
                    foreach (var d in directions)
                        neighbors[k++] = baseline[(y + d.Item2 * r) * width + x + d.Item1 * r];
                var rim = new List<(int X,int Y,float H)>(16); k=0;
                for(var r=radius-1;r<=radius;r++) foreach(var direction in directions)
                    rim.Add((x+direction.Item1*r,y+direction.Item2*r,neighbors[k++]));
                var plane=FitPlane(rim);
                for(k=0;k<rim.Count;k++) neighbors[k]=(float)(rim[k].H-PlaneHeight(plane,rim[k].X,rim[k].Y));
                Array.Sort(neighbors); var median=(float)PlaneHeight(plane,x,y)+(neighbors[7]+neighbors[8])*.5f;
                var spread = neighbors[15] - neighbors[0]; var residual = value - median;
                if (spread > Math.Max(12, spacing * .4f) || Math.Abs(residual) < Math.Max(threshold, spread * 6)) continue;
                var component = new List<int> { i }; var componentSet = new HashSet<int> { i }; var supported = true;
                var sign = Math.Sign(residual); var cutoff = Math.Max(20, spread * 2);
                for (var p = 0; p < component.Count && supported; p++)
                {
                    var current = component[p]; var xx = current % width; var yy = current / width;
                    if (Math.Abs(xx - x) >= radius || Math.Abs(yy - y) >= radius || component.Count > 512 || protectedSample?.Invoke(current) == true)
                    { supported = false; break; }
                    foreach (var next in new[] { current - 1, current + 1, current - width, current + width })
                    {
                        if (sign * (baseline[next] - PlaneHeight(plane,next%width,next/width)) <= cutoff || !componentSet.Add(next)) continue;
                        component.Add(next);
                    }
                }
                if (!supported) continue;
                // Enclosing flat ground is not itself corruption: retain real
                // rounded hills/cones. Inspect the whole component so resampled
                // plateaus can qualify even when the seed has no immediate jump.
                var maximumJump = 0f;
                foreach (var index in component)
                    foreach (var next in new[] { index - 1, index + 1, index - width, index + width })
                        maximumJump = Math.Max(maximumJump, Math.Abs(baseline[index] - baseline[next]));
                if (maximumJump < Math.Max(30, spacing)) continue;
                var rimMinimum=float.PositiveInfinity; var rimMaximum=float.NegativeInfinity;
                foreach(var p in rim) { rimMinimum=Math.Min(rimMinimum,p.H); rimMaximum=Math.Max(rimMaximum,p.H); }
                foreach (var index in component) { raster[index] = (float)Math.Max(rimMinimum,Math.Min(rimMaximum,PlaneHeight(plane,index%width,index/width))); changed[index] = true; }
                if (residual > 0) spikes += component.Count; else pits += component.Count;
                if (details++ < 25) report?.Invoke($"DEM ARTIFACT kind={(residual > 0 ? "spike" : "pit")}; sourceCell={x},{y}; sourceHeight={value:R}; surroundingPlaneHeight={median:R}; ringResidualSpread={spread:R}; correctedClusterCells={component.Count}; maximumComponentJumpMeters={maximumJump:R}; method=constrained-local-plane; localExtremumAndEnclosingSupport=true; suppliedProtectionRespected=true");
            }
        }
        report?.Invoke($"DEM ARTIFACT REPAIR invalidSamples={repaired}; isolatedSpikes={spikes}; isolatedPits={pits}; supportRadiusCells={radius}; ridgesRequireSurroundingSupport=true; noGlobalBiasRollback=true");
    }
    private static (double X,double Y,double H,double Dx,double Dy) FitPlane(List<(int X,int Y,float H)> points)
    {
        double x=0,y=0,h=0; foreach(var p in points){x+=p.X;y+=p.Y;h+=p.H;} x/=points.Count;y/=points.Count;h/=points.Count;
        double xx=0,yy=0,xy=0,xh=0,yh=0;
        foreach(var p in points){var dx=p.X-x;var dy=p.Y-y;var dh=p.H-h;xx+=dx*dx;yy+=dy*dy;xy+=dx*dy;xh+=dx*dh;yh+=dy*dh;}
        var determinant=xx*yy-xy*xy;
        return Math.Abs(determinant)<1e-9 ? (x,y,h,0,0) : (x,y,h,(xh*yy-yh*xy)/determinant,(yh*xx-xh*xy)/determinant);
    }
    private static double PlaneHeight((double X,double Y,double H,double Dx,double Dy) plane,int x,int y)=>plane.H+(x-plane.X)*plane.Dx+(y-plane.Y)*plane.Dy;
}
