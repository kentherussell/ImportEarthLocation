using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

public static class NetworkRoadbed
{
    // Samples: heights in metres over the patch's cells, encoded where the bridge applies it.
    public sealed record Patch(int X, int Z, int Width, int Height, float[] Samples, int Resolution, double MapWidth)
    {
        public float2 Minimum
            => new(
                (float)((X / (double)(Resolution - 1) - .5) * MapWidth),
                (float)((Z / (double)(Resolution - 1) - .5) * MapWidth));

        public float2 Maximum
            => new(
                (float)(((X + Width - 1) / (double)(Resolution - 1) - .5) * MapWidth),
                (float)(((Z + Height - 1) / (double)(Resolution - 1) - .5) * MapWidth));

        // Flood-defence crests and barriers over the patch (NaN where none), kept whole
        // on the game's grid as the full terrain is (TerrainSampling.HeightKeepingCrests).
        public float[]? Crests { get; init; }

        // Indices of the cells the bridge must publish: those that took the rebuilt ground
        // and their eight neighbours, which a bilinear sample between rebuilt and kept ground
        // can reach. Every other cell holds `current`, the ground already published, so the
        // bridge leaves those pixels as the game has them (report 118). Listed on the refit's
        // background task, so the game's thread visits only these. Null: publish every cell.
        public int[]? Published { get; init; }

        // Height in metres at a map point inside the patch.
        public double HeightAt(double x, double z)
            => TerrainSampling.HeightKeepingCrests(
                Samples,
                Crests,
                Width,
                Height,
                (x - Minimum.x) / Math.Max(1e-8, Maximum.x - Minimum.x),
                (z - Minimum.y) / Math.Max(1e-8, Maximum.y - Minimum.y));

        // The patch cell nearest a map point inside the patch.
        public int CellAt(double x, double z)
        {
            var u = (x - Minimum.x) / Math.Max(1e-8, Maximum.x - Minimum.x);
            var v = (z - Minimum.y) / Math.Max(1e-8, Maximum.y - Minimum.y);
            var column = (int)Math.Round(Math.Max(0, Math.Min(1, u)) * (Width - 1));
            var row = (int)Math.Round(Math.Max(0, Math.Min(1, v)) * (Height - 1));

            return row * Width + column;
        }

        public void CopyTo(float[] destination)
        {
            for (var z = 0; z < Height; z++)
                Array.Copy(Samples, z * Width, destination, (Z + z) * Resolution + X, Width);
        }
    }

    // The map's count of baseline cells below their defence crest, kept for the baseline and
    // crests it was counted on: a design's refits share both arrays, and neither is written
    // after design (refits only read them). Replaced as a whole, so a refit on another
    // thread reads a consistent entry.
    private sealed class BelowCrestCount
    {
        public BelowCrestCount(float[] baseline, float[] crests, int count)
        {
            Baseline = baseline;
            Crests = crests;
            Count = count;
        }

        public float[] Baseline { get; }

        public float[] Crests { get; }

        public int Count { get; }
    }

    private static volatile BelowCrestCount? s_belowCrestCount;

    // The baseline cells outside the region that lie below their crest, the cells a restore
    // of the whole map would raise there.
    private static int BaselineCellsBelowCrestOutside(float[] baseline, float[] crests, int4 region, int resolution)
    {
        var total = s_belowCrestCount;

        if (total == null || total.Baseline != baseline || total.Crests != crests)
        {
            var count = 0;

            for (var id = 0; id < baseline.Length; id++)
                if (baseline[id] < crests[id])
                    count++;

            total = new BelowCrestCount(baseline, crests, count);
            s_belowCrestCount = total;
        }

        var inside = 0;

        for (var z = region.y; z <= region.w; z++)
            for (var id = z * resolution + region.x; id <= z * resolution + region.z; id++)
                if (baseline[id] < crests[id])
                    inside++;

        return total.Count - inside;
    }

    // What one import's local refits share and would otherwise rebuild on every call over the
    // whole plan or map: each planned section's box, the extent of each protected ring and
    // line, and, with diagnostics on, the whole-map ground the support check reads. Each part
    // is checked on every use against what it was built from: the plan's sections by
    // reference, so a section the plan replaced after a refit is measured again, and the
    // design's arrays by identity (they are not written after design). ImportApplySystem
    // starts a refit only after the last one finished and drops its cache with the plan when
    // an import ends, so one refit uses a cache at a time: a refit still unwinding after a
    // reset keeps the cache it captured, and the next import makes its own. With diagnostics
    // on, the ground copy (64 MiB on a 4,096-square map) is held until the import ends.
    public sealed class LocalPatchCache
    {
        private PreparedSegment?[] _sections = Array.Empty<PreparedSegment?>();
        private float2[] _lows = Array.Empty<float2>();
        private float2[] _highs = Array.Empty<float2>();
        private float[] _reaches = Array.Empty<float>();
        private double _cell = double.NaN;
        private IEnumerable<float3[]>? _ringSource;
        private float[]? _ringRanges;
        private IEnumerable<float3[]>? _lineSource;
        private float[]? _lineBoxes;
        private float[]? _ground;
        private float[]? _groundBaseline;
        private float[]? _groundCrests;

        // The plan's sections' boxes and reaches, index for index (see Measure).
        public void Bounds(
            IReadOnlyList<PreparedSegment> plan,
            double cell,
            out float2[] lows,
            out float2[] highs,
            out float[] reaches)
        {
            if (_sections.Length != plan.Count || _cell != cell)
            {
                _sections = new PreparedSegment?[plan.Count];
                _lows = new float2[plan.Count];
                _highs = new float2[plan.Count];
                _reaches = new float[plan.Count];
                _cell = cell;
            }

            for (var index = 0; index < plan.Count; index++)
            {
                var segment = plan[index];

                if (ReferenceEquals(_sections[index], segment))
                    continue;

                _sections[index] = segment;
                Measure(segment, cell, out _lows[index], out _highs[index], out _reaches[index]);
            }

            lows = _lows;
            highs = _highs;
            reaches = _reaches;
        }

        public float[] RingRanges(IEnumerable<float3[]> rings)
        {
            if (_ringRanges == null || !ReferenceEquals(_ringSource, rings))
            {
                _ringRanges = NetworkRoadbed.RingRanges(rings);
                _ringSource = rings;
            }

            return _ringRanges;
        }

        public float[] LineBoxes(IEnumerable<float3[]> lines)
        {
            if (_lineBoxes == null || !ReferenceEquals(_lineSource, lines))
            {
                _lineBoxes = NetworkRoadbed.LineBoxes(lines);
                _lineSource = lines;
            }

            return _lineBoxes;
        }

        // The baseline with its defences restored (crests null: none). The caller may write
        // into it but must put back what it wrote before the next use.
        public float[] Ground(float[] baseline, float[]? crests)
        {
            if (_ground == null || _groundBaseline != baseline || _groundCrests != crests)
            {
                var ground = (float[])baseline.Clone();

                if (crests != null)
                    FloodDefences.Restore(ground, crests);

                _ground = ground;
                _groundBaseline = baseline;
                _groundCrests = crests;
            }

            return _ground;
        }
    }

    // A section's box over its four course points (lowest and highest x and z) and the
    // distance, in metres, within which its earthworks can reach a cell.
    private static void Measure(PreparedSegment segment, double cell, out float2 low, out float2 high, out float reach)
    {
        low =
            math.min(
                math.min(segment.CourseStart.xz, segment.CourseControl1.xz),
                math.min(segment.CourseControl2.xz, segment.CourseEnd.xz));

        high =
            math.max(
                math.max(segment.CourseStart.xz, segment.CourseControl1.xz),
                math.max(segment.CourseControl2.xz, segment.CourseEnd.xz));

        reach = (float)(240 + Math.Max(0, segment.NativeWidth) + 2 * cell);
    }

    // Each ring's lowest and highest x and z, four per ring in order, found as PrepareRegion
    // finds them (NaN z for a ring of fewer than three points, which it passes over).
    private static float[] RingRanges(IEnumerable<float3[]> rings)
    {
        var ranges = new List<float>();

        foreach (var ring in rings)
        {
            var lowX = float.PositiveInfinity;
            var highX = float.NegativeInfinity;

            foreach (var point in ring)
            {
                lowX = Math.Min(lowX, point.x);
                highX = Math.Max(highX, point.x);
            }

            ranges.Add(lowX);
            ranges.Add(highX);
            ranges.Add(ring.Length < 3 ? float.NaN : ring.Min(point => point.z));
            ranges.Add(ring.Length < 3 ? float.NaN : ring.Max(point => point.z));
        }

        return ranges.ToArray();
    }

    // Each line's lowest x and z and highest x and z, four per line in order (NaN for a line
    // with a coordinate that is not finite, which is never skipped).
    private static float[] LineBoxes(IEnumerable<float3[]> lines)
    {
        var boxes = new List<float>();

        foreach (var line in lines)
        {
            var low = new float2(float.PositiveInfinity);
            var high = new float2(float.NegativeInfinity);
            var finite = true;

            foreach (var point in line)
            {
                finite &= math.all(math.isfinite(point.xz));
                low = math.min(low, point.xz);
                high = math.max(high, point.xz);
            }

            if (!finite)
                low = high = new float2(float.NaN);

            boxes.Add(low.x);
            boxes.Add(low.y);
            boxes.Add(high.x);
            boxes.Add(high.y);
        }

        return boxes.ToArray();
    }

    // cache: what the import's earlier refits measured (null: measured afresh for this call).
    public static Patch BuildLocalPatch(
        PreparedImport prepared,
        float[] current,
        IReadOnlyList<PreparedSegment> plan,
        int start,
        IReadOnlyList<PreparedSegment> revision, ImportLog log, CancellationToken token,
        LocalPatchCache? cache = null)
    {
        var partClock = System.Diagnostics.Stopwatch.StartNew();
        cache ??= new LocalPatchCache();
        var baseline = prepared.RoadbedBaseline;
        var resolution = TerrainSampling.Resolution(baseline);

        if (current.Length != baseline.Length || resolution < 2)
            throw new InvalidOperationException("Local roadbed baseline is unavailable.");

        if (start < 0 || start + revision.Count > plan.Count)
            throw new ArgumentOutOfRangeException(nameof(start), "A local revision must lie within the plan.");

        var otherLow = new float2(float.PositiveInfinity);
        var otherHigh = new float2(float.NegativeInfinity);

        // The old sections the revision replaces, then the revision, each one's course points
        // in order.
        for (var index = 0; index < 2 * revision.Count; index++)
        {
            var segment = index < revision.Count ? plan[start + index] : revision[index - revision.Count];
            otherLow = math.min(otherLow, segment.CourseStart.xz);
            otherHigh = math.max(otherHigh, segment.CourseStart.xz);
            otherLow = math.min(otherLow, segment.CourseControl1.xz);
            otherHigh = math.max(otherHigh, segment.CourseControl1.xz);
            otherLow = math.min(otherLow, segment.CourseControl2.xz);
            otherHigh = math.max(otherHigh, segment.CourseControl2.xz);
            otherLow = math.min(otherLow, segment.CourseEnd.xz);
            otherHigh = math.max(otherHigh, segment.CourseEnd.xz);
        }

        var width = prepared.Request.MapSize.WidthMeters;
        var cell = width / (resolution - 1);
        var margin = 240 + revision.Max(segment => segment.NativeWidth) + 2 * cell;
        otherLow -= new float2((float)margin);
        otherHigh += new float2((float)margin);

        int Pixel(double x) => (int)math.clamp(Math.Floor((x / width + .5) * (resolution - 1)), 0, resolution - 1);

        var region =
            new int4(
                Pixel(otherLow.x),
                Pixel(otherLow.y),
                Math.Min(resolution - 1, Pixel(otherHigh.x) + 1),
                Math.Min(resolution - 1, Pixel(otherHigh.y) + 1));

        // The plan's sections' boxes, measured once per import and again only where the plan
        // replaced a section, and the revision's, which stand in for the sections it replaces.
        cache.Bounds(plan, cell, out var lows, out var highs, out var reaches);
        var revisedLows = new float2[revision.Count];
        var revisedHighs = new float2[revision.Count];
        var revisedReaches = new float[revision.Count];

        for (var index = 0; index < revision.Count; index++)
            Measure(revision[index], cell, out revisedLows[index], out revisedHighs[index], out revisedReaches[index]);

        // A unit can run for kilometres, so its bounding box holds far more ground than it
        // changes. Earthworks reach at most `margin` from a section, so only cells within
        // that distance of an old or revised section can differ, and only sections within a
        // second `margin` of those can shape such a cell. Everything else keeps `current`.
        var changedBoxes = new (float2 Low, float2 High)[2 * revision.Count];

        for (var index = 0; index < revision.Count; index++)
        {
            changedBoxes[index] =
                (lows[start + index] - new float2((float)margin), highs[start + index] + new float2((float)margin));

            changedBoxes[revision.Count + index] =
                (revisedLows[index] - new float2((float)margin), revisedHighs[index] + new float2((float)margin));
        }

        bool NearChange(float2 min, float2 max, float reach)
        {
            if (!(math.all(max >= otherLow - reach) && math.all(min <= otherHigh + reach)))
                return false;

            foreach (var box in changedBoxes)
                if (math.all(max >= box.Low - reach) && math.all(min <= box.High + reach))
                    return true;

            return false;
        }

        // The sections near the change, in plan order with the revision in place of the
        // sections it replaces, and those of them already issued that stand on the ground.
        var neighbourList = new List<PreparedSegment>();
        var issued = new List<PreparedSegment>();

        for (var index = 0; index < plan.Count; index++)
        {
            var revised = index - start;
            var replaced = revised >= 0 && revised < revision.Count;
            var near =
                replaced
                    ? NearChange(revisedLows[revised], revisedHighs[revised], revisedReaches[revised])
                    : NearChange(lows[index], highs[index], reaches[index]);

            if (!near)
                continue;

            var segment = replaced ? revision[revised] : plan[index];
            neighbourList.Add(segment);

            if (index < start && !NetworkStructure.IsStructure(segment))
                issued.Add(segment);
        }

        var neighbours = neighbourList.ToArray();
        log.AddDuration("Local refit patch neighbours", partClock.Elapsed.TotalSeconds);
        partClock.Restart();
        var w = region.z - region.x + 1;
        var h = region.w - region.y + 1;
        // The rebuilt ground of the region alone, row by row, cell for cell as the patch.
        var rebuilt =
            PrepareRegion(
                baseline,
                prepared.HeightScaleMeters,
                width,
                neighbours,
                prepared.RoadbedProtectedAreas,
                prepared.RoadbedProtectedLines,
                log,
                token,
                region,
                float.NaN,
                cache);

        partClock.Restart();

        // Earthworks may cut a defence where a road crosses it; it goes back up, as in design.
        // The count covers the whole map, as it did when the whole map was rebuilt: outside the
        // region that is the baseline's cells below their crest.
        if (prepared.DefenceCrests?.Length == baseline.Length)
            log.Count(
                "Local refit flood defence cells restored",
                BaselineCellsBelowCrestOutside(baseline, prepared.DefenceCrests, region, resolution)
                    + FloodDefences.Restore(rebuilt, prepared.DefenceCrests, region.x, region.y, w, resolution));

        log.AddDuration("Local refit patch flood defences", partClock.Elapsed.TotalSeconds);
        partClock.Restart();
        var samples = new float[w * h];
        var preserved = 0;
        var protectedCores = new bool[w * h];
        var changeable = new bool[w * h];
        // Each changeable cell once: only these can take the rebuilt ground, so the passes
        // below visit them rather than the whole rectangle.
        var changeableCells = new List<int>();

        foreach (var box in changedBoxes)
            for (var z = Math.Max(region.y, Pixel(box.Low.y)); z <= Math.Min(region.w, Pixel(box.High.y) + 1); z++)
                for (var x = Math.Max(region.x, Pixel(box.Low.x)); x <= Math.Min(region.z, Pixel(box.High.x) + 1); x++)
                {
                    var index = (z - region.y) * w + x - region.x;

                    if (changeable[index])
                        continue;

                    changeable[index] = true;
                    changeableCells.Add(index);
                }

        foreach (var segment in issued)
        {
            var steps =
                Math.Max(1, (int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(segment) / Math.Max(1, cell * .5)));

            var radius = segment.NativeWidth / 2 + cell;
            var previous = NetworkNormalizer.Evaluate(segment, 0);

            for (var step = 1; step <= steps; step++)
            {
                token.ThrowIfCancellationRequested();
                var next = NetworkNormalizer.Evaluate(segment, step / (float)steps);
                var low = math.min(previous.xz, next.xz) - (float)radius;
                var high = math.max(previous.xz, next.xz) + (float)radius;

                for (var z = Math.Max(region.y, Pixel(low.y)); z <= Math.Min(region.w, Pixel(high.y) + 1); z++)
                    for (var x = Math.Max(region.x, Pixel(low.x)); x <= Math.Min(region.z, Pixel(high.x) + 1); x++)
                    {
                        var point =
                            new float3(
                                (float)((x / (double)(resolution - 1) - .5) * width),
                                0,
                                (float)((z / (double)(resolution - 1) - .5) * width));

                        if (NetworkNormalizer.DistanceToSegment(point, previous, next) <= radius)
                            protectedCores[(z - region.y) * w + x - region.x] = true;
                    }

                previous = next;
            }
        }

        // Every cell holds the ground already published unless it can change and is free.
        for (var z = 0; z < h; z++)
            Array.Copy(current, (region.y + z) * resolution + region.x, samples, z * w, w);

        var edits = prepared.RoadbedLaterEdits;
        var rebuiltCells = new List<int>();

        foreach (var index in changeableCells)
        {
            token.ThrowIfCancellationRequested();
            var id = (region.y + index / w) * resolution + region.x + index % w;
            // Preserve already-issued ground cores, including an interpolation
            // cell of support. A local retry owns only its unissued support.
            var protectedCore = protectedCores[index] || edits?.Mask.Length == current.Length && edits.Mask[id] != 0;

            if (protectedCore)
            {
                preserved++;
                continue;
            }

            samples[index] = rebuilt[index];
            rebuiltCells.Add(index);
        }

        log.Diagnostic(
            $"LOCAL ROADBED staged samples={samples.Length}; neighbourSections={neighbours.Length}; issuedCoreSamplesPreserved={preserved}; baselineImmutable=true; nativeMutation=false");

        float[]? crests = null;

        if (prepared.DefenceCrests?.Length == baseline.Length)
        {
            crests = new float[w * h];

            for (var z = 0; z < h; z++)
                Array.Copy(prepared.DefenceCrests, (region.y + z) * resolution + region.x, crests, z * w, w);
        }

        // A cell is published when it or one of its eight neighbours took the rebuilt ground:
        // the rebuilt cells' neighbourhoods, listed once each and sorted into row order.
        var published = new List<int>();
        var listed = new bool[w * h];

        foreach (var index in rebuiltCells)
        {
            var x = index % w;
            var z = index / w;

            for (var dz = Math.Max(0, z - 1); dz <= Math.Min(h - 1, z + 1); dz++)
                for (var dx = Math.Max(0, x - 1); dx <= Math.Min(w - 1, x + 1); dx++)
                {
                    var neighbour = dz * w + dx;

                    if (listed[neighbour])
                        continue;

                    listed[neighbour] = true;
                    published.Add(neighbour);
                }
        }

        published.Sort();

        var patch =
            new Patch(region.x, region.y, w, h, samples, resolution, width)
            {
                Crests = crests,
                Published = published.ToArray()
            };

        log.AddDuration("Local refit patch assembly", partClock.Elapsed.TotalSeconds);

        // The ground-support certificate only reports (report 118), and placement waits for
        // this refit, so it runs only with diagnostics on; only it reads the patched ground.
        // The whole map it reads is the baseline, with its defences restored, under the patch:
        // the cache's copy, with the patch written in for the check and taken out after it.
        if (log.DiagnosticsEnabled)
        {
            partClock.Restart();
            var ground =
                cache.Ground(
                    baseline,
                    prepared.DefenceCrests?.Length == baseline.Length ? prepared.DefenceCrests : null);

            var underPatch = new float[w * h];

            for (var z = 0; z < h; z++)
                Array.Copy(ground, (region.y + z) * resolution + region.x, underPatch, z * w, w);

            try
            {
                patch.CopyTo(ground);
                NetworkTerrainSupport.Validate(
                    revision,
                    ground,
                    prepared.HeightScaleMeters,
                    width,
                    log,
                    token,
                    scope: $"native-unit-{revision[0].NativeUnitId}");
            }
            finally
            {
                for (var z = 0; z < h; z++)
                    Array.Copy(underPatch, z * w, ground, (region.y + z) * resolution + region.x, w);
            }

            log.AddDuration("Local refit patch support check", partClock.Elapsed.TotalSeconds);
        }

        return patch;
    }

    // Work on a copy. Structural spans never become broad terrain embankments.
    // waterSurface: effective sea level in the raster's frame (NaN when unknown),
    // which a tunnel roof fill under the sea never reaches.
    public static float[] Prepare(float[] source, float scale, double width, IReadOnlyList<PreparedSegment> segments,
        IEnumerable<float3[]> protectedAreas,
            IEnumerable<float3[]> protectedLines,
            ImportLog log,
            CancellationToken token,
            int4? dirtyRegion = null,
            float waterSurface = float.NaN)
    {
        var n = (int)Math.Sqrt(source.Length);

        if (n * n != source.Length || n < 2 || width <= 0 || scale <= 0)
            throw new InvalidOperationException("Invalid roadbed terrain.");

        var window =
            PrepareRegion(
                source,
                scale,
                width,
                segments,
                protectedAreas,
                protectedLines,
                log,
                token,
                dirtyRegion,
                waterSurface,
                null);

        if (dirtyRegion is not int4 region)
            return window;

        // Nothing outside the region changes: the whole grid is the source with the window in it.
        var result = (float[])source.Clone();
        var regionWidth = region.z - region.x + 1;

        for (var z = region.y; z <= region.w; z++)
            Array.Copy(window, (z - region.y) * regionWidth, result, z * n + region.x, regionWidth);

        return result;
    }

    // Prepare's work, returning only the region (the whole grid when there is none), row by
    // row: every edit lies inside it, so a local refit need not copy the rest of the map.
    // localRefit: a local refit's cache, which holds the protected rings' and lines' extents;
    // the refit's parts are timed. Null for a design, which measures them afresh.
    private static float[] PrepareRegion(
        float[] source,
        float scale,
        double width,
        IReadOnlyList<PreparedSegment> segments,
        IEnumerable<float3[]> protectedAreas,
        IEnumerable<float3[]> protectedLines,
        ImportLog log,
        CancellationToken token,
        int4? dirtyRegion,
        float waterSurface,
        LocalPatchCache? localRefit)
    {
        var partClock = System.Diagnostics.Stopwatch.StartNew();
        var n = (int)Math.Sqrt(source.Length);
        var cell = (float)(width / (n - 1));
        var region = dirtyRegion ?? new int4(0, 0, n - 1, n - 1);
        // The masks and the result cover the region only, row by row (the whole grid when
        // there is none): nothing outside it is marked, read or changed. A local refit's
        // region is a part of the map.
        var regionWidth = region.z - region.x + 1;
        var regionCells = regionWidth * (region.w - region.y + 1);
        var protectedMask = new bool[regionCells];
        var lineGuard = new bool[regionCells];
        // Inside mapped water and quarry polygons only, without the guard bands
        // around them and around cliff, waterway and coast lines: tunnel roof
        // cover may be built there (Marseille's tunnels pass beside cliffs and
        // under buried streams, whose bands left their bores exposed).
        var areaMask = new bool[regionCells];
        var coordinates = new float[n];

        for (var i = 0; i < n; i++)
            coordinates[i] = (i / (float)(n - 1) - .5f) * (float)width;

        int Pixel(float value) => (int)Math.Round((value / width + .5) * (n - 1));

        int MaskIndex(int x, int z) => (z - region.y) * regionWidth + x - region.x;

        int ResultIndex(int id) => MaskIndex(id % n, id / n);

        void Guard(float3 vectorA, float3 vectorB, float margin, bool[]? mask = null)
        {
            var low = math.min(vectorA.xz, vectorB.xz) - margin;
            var high = math.max(vectorA.xz, vectorB.xz) + margin;
            var delta = vectorB.xz - vectorA.xz;
            var denominator = Math.Max(.001f, math.lengthsq(delta));

            for (var z = Math.Max(region.y, Pixel(low.y)); z <= Math.Min(region.w, Pixel(high.y)); z++)
                for (var x = Math.Max(region.x, Pixel(low.x)); x <= Math.Min(region.z, Pixel(high.x)); x++)
                {
                    var point = new float2(coordinates[x], coordinates[z]);
                    var t = math.clamp(math.dot(point - vectorA.xz, delta) / denominator, 0, 1);

                    if (math.distance(point, math.lerp(vectorA.xz, vectorB.xz, t)) <= margin)
                        (mask ?? protectedMask)[MaskIndex(x, z)] = true;
                }
        }

        // A refit takes each ring's x extent and each line's box from its cache, measured once.
        var ringRanges = localRefit?.RingRanges(protectedAreas);
        var lineBoxes = localRefit?.LineBoxes(protectedLines);
        var ringIndex = -1;

        // Protect the actual polygon, not its rectangular envelope. A winding
        // river's bounding box can cover kilometres of dry roads and prevent
        // their entire cut/fill pass from being applied.
        foreach (var ring in protectedAreas)
        {
            ringIndex++;
            token.ThrowIfCancellationRequested();

            if (ring.Length < 3)
                continue;

            var lowX = float.PositiveInfinity;
            var highX = float.NegativeInfinity;

            if (ringRanges != null)
            {
                lowX = ringRanges[4 * ringIndex];
                highX = ringRanges[4 * ringIndex + 1];
            }
            else
                foreach (var point in ring)
                {
                    lowX = Math.Min(lowX, point.x);
                    highX = Math.Max(highX, point.x);
                }

            // A ring that reaches no column of the region marks nothing: its interior lies
            // between its lowest and highest x, and its edge guards one cell beyond them
            // (rounding moves a crossing by far less than a cell). A ring with a coordinate
            // that is not finite is left to the passes below.
            if (math.isfinite(lowX)
                && math.isfinite(highX)
                && (Pixel(highX + cell) < region.x || Pixel(lowX - cell) > region.z))
                continue;

            var lowZ = ringRanges != null ? ringRanges[4 * ringIndex + 2] : ring.Min(point => point.z);
            var highZ = ringRanges != null ? ringRanges[4 * ringIndex + 3] : ring.Max(point => point.z);

            // Nor does a ring that reaches no row of the region, for the same reasons.
            if (math.isfinite(lowZ)
                && math.isfinite(highZ)
                && (Pixel(highZ + cell) < region.y || Pixel(lowZ - cell) > region.w))
                continue;

            var crossings = new List<float>(ring.Length);

            for (var z = Math.Max(region.y, Pixel(lowZ)); z <= Math.Min(region.w, Pixel(highZ)); z++)
            {
                token.ThrowIfCancellationRequested();
                var y = coordinates[z];
                crossings.Clear();

                for (var i = 0; i < ring.Length; i++)
                {
                    var vectorA = ring[i];
                    var vectorB = ring[(i + 1) % ring.Length];

                    if ((vectorA.z > y) == (vectorB.z > y))
                        continue;

                    crossings.Add(vectorA.x + (y - vectorA.z) * (vectorB.x - vectorA.x) / (vectorB.z - vectorA.z));
                }

                crossings.Sort();

                for (var i = 1; i < crossings.Count; i += 2)
                    for (var x =
                        Math.Max(region.x, Pixel(crossings[i - 1])); x <= Math.Min(region.z, Pixel(crossings[i])); x++)
                        protectedMask[MaskIndex(x, z)] = areaMask[MaskIndex(x, z)] = true;
            }

            for (var i = 0; i < ring.Length; i++)
                Guard(ring[i], ring[(i + 1) % ring.Length], cell);
        }

        // Bands along waterway, cliff and coast lines keep road shoulders and fills
        // off them, but not a ground road's own core and the grid cell beside it,
        // from which the ground under its centre is read: the road stands there in
        // game whatever the ground does, and left unstamped it floated or sank
        // beside bridges over streams (300 of Marseille's 1,455 mismatches).
        var lineIndex = -1;

        foreach (var line in protectedLines)
        {
            lineIndex++;

            // A refit passes over a line that reaches no cell of the region: each of its bands
            // lies within its box widened by two cells, and a third covers rounding.
            if (lineBoxes != null
                && math.isfinite(lineBoxes[4 * lineIndex])
                && (Pixel(lineBoxes[4 * lineIndex + 2] + cell * 3) < region.x
                    || Pixel(lineBoxes[4 * lineIndex] - cell * 3) > region.z
                    || Pixel(lineBoxes[4 * lineIndex + 3] + cell * 3) < region.y
                    || Pixel(lineBoxes[4 * lineIndex + 1] - cell * 3) > region.w))
                continue;

            for (var i = 1; i < line.Length; i++)
            {
                token.ThrowIfCancellationRequested();
                Guard(line[i - 1], line[i], cell * 2, lineGuard);
            }
        }

        if (localRefit != null)
            log.AddDuration("Local refit patch protection", partClock.Elapsed.TotalSeconds);

        partClock.Restart();

        // Tunnel nodes that also meet an open section. The game draws the first
        // PortalOpen metres of a bore as an open cut, so there is no roof to keep:
        // guarding it left the approach's cutting unstamped, and the road ran
        // between walls of original ground up to the portal.
        const float PortalOpen = 40, PortalReach = 100;
        var tunnelArms = new Dictionary<(NetworkKind, long), (int Tunnel, int Open, float2 At)>();

        foreach (var segment in segments)
            foreach (var (id, at) in new[] {
                (segment.StartNodeId, segment.CourseStart.xz),
                (segment.EndNodeId, segment.CourseEnd.xz)
                })
            {
                tunnelArms.TryGetValue((segment.Kind, id), out var arms);
                tunnelArms[(segment.Kind, id)] =
                    segment.Tunnel ? (arms.Tunnel + 1, arms.Open, at) : (arms.Tunnel, arms.Open + 1, at);
            }

        var portals = new Dictionary<(int, int), List<float2>>();

        foreach (var arms in tunnelArms.Values)
            if (arms.Tunnel > 0 && arms.Open > 0)
            {
                var key = ((int)Math.Floor(arms.At.x / PortalReach), (int)Math.Floor(arms.At.y / PortalReach));

                if (!portals.TryGetValue(key, out var list))
                    portals[key] = list = new();

                list.Add(arms.At);
            }

        float PortalDistance(float2 point)
        {
            var best = float.PositiveInfinity;
            int cx = (int)Math.Floor(point.x / PortalReach), cz = (int)Math.Floor(point.y / PortalReach);

            for (var dz = -1; dz <= 1; dz++)
                for (var dx = -1; dx <= 1; dx++)
                    if (portals.TryGetValue((cx + dx, cz + dz), out var list))
                        foreach (var at in list)
                            best = Math.Min(best, math.distance(point, at));

            return best;
        }

        // Surface-road cuts must not remove a tunnel's roof. Include the terrain
        // interpolation footprint, not just the narrow track/road centreline.
        foreach (var tunnel in segments.Where(segment => segment.Tunnel))
        {
            var steps =
                Math.Max(1, (int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(tunnel) / Math.Max(1, cell * .5f)));

            var margin = (tunnel.NativeWidth > 0 ? tunnel.NativeWidth * .5f : 6) + cell * 2;
            var previous = NetworkNormalizer.Evaluate(tunnel, 0);

            for (var i = 1; i <= steps; i++)
            {
                token.ThrowIfCancellationRequested();
                var next = NetworkNormalizer.Evaluate(tunnel, i / (float)steps);

                if (PortalDistance(previous.xz) >= PortalOpen || PortalDistance(next.xz) >= PortalOpen)
                    Guard(previous, next, margin);

                previous = next;
            }
        }

        var targets = new Dictionary<int, (float target, float budget, float blend, float distance)>();
        var protectedSamples = 0;
        var beyondBudget = 0;
        var clock = System.Diagnostics.Stopwatch.StartNew();
        var nextProgress = 5d;
        var processed = 0;
        var unsupportedSections = 0;
        var maximumSupportDeficit = 0f;
        // A shared ground profile owns one formation, including its
        // median. Do not let two nearest-road stamps independently select the
        // elevation across that same cross-section. Mixed structural modes and
        // explicit walls keep their separate footprints.
        var formations = new List<PreparedSegment>();
        var consumed = new HashSet<PreparedSegment>();
        var sharedFormations = 0;
        var mixedFormations = 0;
        var incompatibleFormations = 0;

        // Unit subdivision retains the parent profile ID. Match its actual
        // reference interval too; the micrometre key absorbs arithmetic noise
        // in exact subdivision, without moving any geometry or terrain sample.
        foreach (var group in segments
            .Where(segment => segment.PairedProfileId != 0)
            .GroupBy(segment => (segment.PairedProfileId,
            Begin: Math.Round(Math.Min(segment.PairedReferenceBegin, segment.PairedReferenceEnd), 6),
            End: Math.Round(Math.Max(segment.PairedReferenceBegin, segment.PairedReferenceEnd), 6))))
        {
            token.ThrowIfCancellationRequested();
            var pair = group.ToArray();

            if (pair.Length != 2 || pair[0].PairedReversed == pair[1].PairedReversed)
                continue;

            if (pair.Any(segment => segment.ExplicitWallSides != 0 || segment.QuaySides != 0))
                continue;

            var grounds = pair.Where(segment => !NetworkStructure.IsStructure(segment)).ToArray();

            if (grounds.Length == 0)
                continue;

            var mixed = grounds.Length == 1;
            var firstSegment = mixed ? grounds[0] : pair[0];
            var secondSegment = ReferenceEquals(firstSegment, pair[0]) ? pair[1] : pair[0];
            var aa =
                new[] {
                    firstSegment.CourseStart,
                    firstSegment.CourseControl1,
                    firstSegment.CourseControl2,
                    firstSegment.CourseEnd
                    };

            var bb =
                new[] {
                    secondSegment.CourseEnd,
                    secondSegment.CourseControl2,
                    secondSegment.CourseControl1,
                    secondSegment.CourseStart
                    };

            if (Enumerable.Range(0, 4).Any(i => Math.Abs(aa[i].y - bb[i].y) > .01f))
            {
                incompatibleFormations++;
                continue;
            }

            var gap = Enumerable.Range(0, 4).Max(i => math.distance(aa[i].xz, bb[i].xz));
            var center = Enumerable.Range(0, 4).Select(i =>
            {
                if (!mixed)
                    return (aa[i] + bb[i]) * .5f;

                var separation = math.distance(aa[i].xz, bb[i].xz);
                // Ground outer edge to common median bisector. Do not build a
                // full ground platform underneath the structural carriageway.
                var fraction =
                    separation > firstSegment.NativeWidth ? (1 - firstSegment.NativeWidth / separation) / 4 : 0;

                return math.lerp(aa[i], bb[i], fraction);
            }).ToArray();

            formations.Add(firstSegment with
            {
                Start = center[0],
                Control1 = center[1],
                Control2 = center[2],
                End = center[3],
                StartCurveOffset = default,
                EndCurveOffset = default,
                FinalDeckCurve = new(center[0], center[1], center[2], center[3]),
                HorizontalParent = null,
                NativeWidth =
                    mixed
                        ? Math.Max(firstSegment.NativeWidth, (gap + firstSegment.NativeWidth) / 2)
                        : gap + Math.Max(firstSegment.NativeWidth, secondSegment.NativeWidth)
            });

            consumed.Add(firstSegment);

            if (!mixed)
            {
                consumed.Add(secondSegment);
                sharedFormations++;
            }
            else
            {
                mixedFormations++;
                // The interpolation footprint is protected too. A coarse grid
                // may consequently fail ground support; retain that conflict
                // rather than bury a bridge or cut a tunnel roof to hide it.
                var steps =
                    Math.Max(
                        1,
                        (int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(secondSegment) / Math.Max(1, cell * .5)));

                var previous = NetworkNormalizer.Evaluate(secondSegment, 0);

                for (var step = 1; step <= steps; step++)
                {
                    var next = NetworkNormalizer.Evaluate(secondSegment, step / (float)steps);
                    Guard(previous, next, secondSegment.NativeWidth / 2 + cell);
                    previous = next;
                }
            }
        }

        formations.AddRange(
            segments.Where(segment => !consumed.Contains(segment) && !NetworkStructure.IsStructure(segment)));

        // Bridge abutments and tunnel portals: the approach fill or cutting stays
        // close to the road footprint (a steep, short bank) instead of the broad
        // 1:4 shoulders used elsewhere, widening back to normal over ApproachReach.
        const float ApproachReach = 100;
        var narrowedApproachSamples = 0;
        var structureEnds = new Dictionary<(int, int), List<float2>>();

        foreach (var segment in segments.Where(NetworkStructure.IsStructure))
            foreach (var end in new[] { segment.CourseStart.xz, segment.CourseEnd.xz })
            {
                var key = ((int)Math.Floor(end.x / ApproachReach), (int)Math.Floor(end.y / ApproachReach));

                if (!structureEnds.TryGetValue(key, out var list))
                    structureEnds[key] = list = new();

                list.Add(end);
            }

        var bridgeEnds = new Dictionary<(int, int), List<float2>>();

        foreach (var segment in segments.Where(otherSegment => otherSegment.Bridge && !otherSegment.Tunnel))
            foreach (var end in new[] { segment.CourseStart.xz, segment.CourseEnd.xz })
            {
                var key = ((int)Math.Floor(end.x / ApproachReach), (int)Math.Floor(end.y / ApproachReach));

                if (!bridgeEnds.TryGetValue(key, out var list))
                    bridgeEnds[key] = list = new();

                list.Add(end);
            }

        float EndDistance(Dictionary<(int, int), List<float2>> ends, float2 point)
        {
            var best = float.PositiveInfinity;
            int cx = (int)Math.Floor(point.x / ApproachReach), cz = (int)Math.Floor(point.y / ApproachReach);

            for (var dz = -1; dz <= 1; dz++)
                for (var dx = -1; dx <= 1; dx++)
                    if (ends.TryGetValue((cx + dx, cz + dz), out var list))
                        foreach (var end in list)
                            best = Math.Min(best, math.distance(point, end));

            return best;
        }

        float StructureEndDistance(float2 point) => EndDistance(structureEnds, point);

        log.Quality(
            new(
                "planned",
                "mixed-mode-formation",
                "roadbed",
                mixedFormations + incompatibleFormations,
                0,
                incompatibleFormations,
                0,
                0,
            "Trusted common profile; ground/median half-formation only; structural interpolation support protected; actual raster support checked separately"));

        log.Diagnostic(
            $"ROADBED FORMATIONS sharedGroundProfiles={sharedFormations}; sharedMixedModeHalfFormations={mixedFormations}; incompatibleProfiles={incompatibleFormations}; structuralFootprintsProtected=true; explicitWallsRetainedSeparately=true; profileHeightsNotReoptimized=true");

        // With the wide-approaches switch on, approaches keep their full banks
        // (report 112, 11.4 B2): a bank 1.5 cells wide is a terrace step.
        var narrowApproaches = !DesignSwitches.On(DesignSwitches.WideApproaches);

        foreach (var segment in formations)
        {
            token.ThrowIfCancellationRequested();
            processed++;
            var budget = NetworkVerticalProfiles.CutFillBudget(segment);
            var half =
                segment.NativeWidth > 0 ? segment.NativeWidth * .5f : Math.Min(12, Math.Max(4, segment.Lanes * 1.75f));

            var unsupported = false;
            var length = NetworkVerticalProfiles.ArcLength(segment);
            var steps = Math.Max(1, (int)Math.Ceiling(length / (cell * .5f)));
            // A ground road ending at a bridge carries its formation one sample
            // step (half a cell) on along its end tangent and grade: the game
            // reads the ground under the road's last metres from grid samples
            // under the bridge, which no other formation stamps (DC's motorway
            // approaches stood up to 3 m above the ground there). A full cell
            // raised more ground under roads passing beneath the bridge end.
            var extendStart = EndDistance(bridgeEnds, segment.CourseStart.xz) < .5f ? 1 : 0;
            var extendEnd = EndDistance(bridgeEnds, segment.CourseEnd.xz) < .5f ? 1 : 0;
            var startPoint = NetworkNormalizer.Evaluate(segment, 0);
            var endPoint = NetworkNormalizer.Evaluate(segment, 1);
            var outStart = math.normalizesafe(startPoint - NetworkNormalizer.Evaluate(segment, .001f));
            var outEnd = math.normalizesafe(endPoint - NetworkNormalizer.Evaluate(segment, .999f));

            float3 Station(int i)
                => i < 0
                    ? startPoint + outStart * (-i * length / steps)
                    : i > steps
                        ? endPoint + outEnd * ((i - steps) * length / steps)
                        : NetworkNormalizer.Evaluate(segment, i / (float)steps);

            var point = Station(-extendStart);

            for (var i = -extendStart; i <= steps + extendEnd; i++)
            {
                token.ThrowIfCancellationRequested();

                if (clock.Elapsed.TotalSeconds >= nextProgress)
                {
                    log.Diagnostic(
                        $"PROCESS roadbed sections={processed}; sectionSample={i}/{steps}; affectedSamples={targets.Count}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.0}");

                    nextProgress = clock.Elapsed.TotalSeconds + 5;
                }

                var t = math.clamp(i / (float)steps, 0, 1);
                // The span runs from the last station to this one (the first is its own).
                var previous = point;

                if (i > -extendStart)
                    point = Station(i);

                var px = Pixel(point.x);
                var pz = Pixel(point.z);
                var span = point.xz - previous.xz;
                var spanLengthSquared = math.lengthsq(span);
                var originalCenter = NetworkProcessor.Sample(point.x, point.z, source, width);
                var deficit = Math.Abs(originalCenter - point.y) - budget;

                // Two steps of the game's height encoding, which the ground is rounded to.
                if (deficit > Math.Max(.05f, 2 * scale / 65535f))
                {
                    unsupported = true;
                    maximumSupportDeficit = Math.Max(maximumSupportDeficit, deficit);
                }

                // A large embankment needs proportionally more lateral room.
                // Smoothstep's peak derivative is 1.5: six metres of shoulder
                // per metre of fill keeps the added bank slope near 1:4.
                // An unsupported profile must not enlarge its earthworks beyond
                // the displacement the terrain policy actually permits.
                var earthBlend =
                    Math.Min(240, Math.Max(cell * 3f, Math.Min(budget, Math.Abs(originalCenter - point.y)) * 6f));

                if (narrowApproaches && structureEnds.Count > 0)
                {
                    var approach = StructureEndDistance(point.xz);

                    if (approach < ApproachReach)
                    {
                        earthBlend = math.lerp(cell * 1.5f, earthBlend, approach / ApproachReach);
                        narrowedApproachSamples++;
                    }
                }

                var radius = half + earthBlend;
                var reach = (int)Math.Ceiling(radius / cell);
                var tangent =
                    math.normalizesafe(
                        (NetworkNormalizer.Evaluate(segment, Math.Min(1, t + .01f))
                            - NetworkNormalizer.Evaluate(segment, Math.Max(0, t - .01f))).xz);

                var leftNormal = new float2(-tangent.y, tangent.x);
                // No side reaches further than `radius` from the span (a wall side's shoulder
                // is narrower than any earth bank), so a cell more than a metre beyond that
                // from the span's bounding box fails the distance test below: rounding moves
                // that distance by millimetres. Rows and columns of such cells are skipped,
                // except about a span whose position is not finite.
                var pruned = math.all(math.isfinite(previous.xz)) && math.all(math.isfinite(point.xz));
                var limit = radius + 1d;
                var spanLowX = Math.Min(previous.x, point.x);
                var spanHighX = Math.Max(previous.x, point.x);
                var spanLowZ = Math.Min(previous.z, point.z);
                var spanHighZ = Math.Max(previous.z, point.z);

                for (var z = Math.Max(region.y, pz - reach); z <= Math.Min(region.w, pz + reach); z++)
                {
                    var firstX = Math.Max(region.x, px - reach);
                    var lastX = Math.Min(region.z, px + reach);

                    if (pruned)
                    {
                        var rowGap = Math.Max(0, Math.Max(spanLowZ - coordinates[z], coordinates[z] - spanHighZ));

                        if (rowGap > limit)
                            continue;

                        var chord = Math.Sqrt(limit * limit - (double)rowGap * rowGap);
                        firstX = Math.Max(firstX, (int)Math.Floor(((spanLowX - chord) / width + .5) * (n - 1)));
                        lastX = Math.Min(lastX, (int)Math.Ceiling(((spanHighX + chord) / width + .5) * (n - 1)));
                    }

                    for (var x = firstX; x <= lastX; x++)
                    {
                        var sample = new float2(coordinates[x], coordinates[z]);
                        var station =
                            spanLengthSquared > .00001f
                                ? math.clamp(math.dot(sample - previous.xz, span) / spanLengthSquared, 0, 1)
                                : 1;

                        var nearest = math.lerp(previous, point, station);
                        var delta = sample - nearest.xz;
                        var distance = math.length(delta);
                        var side = math.dot(delta, leftNormal) >= 0 ? NetworkSides.Left : NetworkSides.Right;
                        // Walls hold the outer bank; earth slopes occupy the available
                        // room. Use the same road height and a smooth bank transition.
                        // Only evidenced walls justify a narrow, cliff-like shoulder.
                        // Automatically inferred walls are not ground truth and must
                        // not turn otherwise natural earthworks into vertical stamps.
                        var wall = ((segment.ExplicitWallSides | segment.QuaySides) & side) != 0;
                        var sideRadius = half + (wall ? cell * .75f : earthBlend);
                        // The ground under the centreline is read from the four
                        // surrounding grid samples, so the formation stands at full
                        // height out to one cell beyond the deck before feathering.
                        var full = wall ? half : half + cell;

                        if (distance > sideRadius)
                            continue;

                        var id = z * n + x;

                        if (protectedMask[MaskIndex(x, z)] || lineGuard[MaskIndex(x, z)] && distance > half + cell)
                        {
                            protectedSamples++;
                            continue;
                        }

                        var original = source[id];

                        if (Math.Abs(nearest.y - original) > budget + .05f)
                        {
                            // At DEM resolution a pixel can be outside a narrow road on
                            // steep terrain. Do not force the entire pixel to the road grade.
                            if (distance <= cell * .5f)
                                beyondBudget++;
                            // Retain a bounded, smoothly feathered edit, but report
                            // that it cannot support the requested road height.
                        }

                        var supportedTarget =
                            math.clamp(nearest.y, Math.Max(0, original - budget), Math.Min(scale, original + budget));

                        var blendT = math.clamp((sideRadius - distance) / Math.Max(.01f, sideRadius - full), 0, 1);
                        var weight = distance <= full ? 1 : blendT * blendT * (3 - 2 * blendT);

                        if (weight <= 0)
                            continue;

                        targets.TryGetValue(id, out var old);

                        // Follow the nearest curve station, not an average of many
                        // uphill/downhill radial stamps. Shared junction profiles
                        // already have one solved elevation; no far stamp can pull
                        // the road core toward a different longitudinal height.
                        if (old.blend == 0 || distance < old.distance)
                        {
                            // Compose the outer edit, not just its raw centre height.
                            // Switching nearest corridors must not discard a stronger
                            // compatible feather and leave a seam between earthworks.
                            if (old.blend > 0
                                && distance > half
                                && old.distance > half
                                && Math.Abs(old.target - supportedTarget) < cell)
                            {
                                var combined = old.blend + weight;
                                targets[id] =
                                    (
                                        (old.target * old.blend + supportedTarget * weight) / combined,
                                        Math.Min(old.budget, budget),
                                        Math.Max(old.blend, weight),
                                        distance);
                            }
                            else
                                targets[id] = (supportedTarget, budget, weight, distance);
                        }
                    }
                }
            }

            if (unsupported)
                unsupportedSections++;

            if (clock.Elapsed.TotalSeconds >= nextProgress)
            {
                log.Diagnostic(
                    $"PROCESS roadbed sections={processed}; affectedSamples={targets.Count}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.0}");

                nextProgress = clock.Elapsed.TotalSeconds + 5;
            }
        }

        if (localRefit != null)
            log.AddDuration("Local refit patch stamping", partClock.Elapsed.TotalSeconds);

        partClock.Restart();

        // Under a bridge deck: city elevation data carries viaducts and their
        // embankments (the A7 into Marseille), and a deck sat in the ground it
        // should stand over. Away from its ends ground reaching within BridgeUnder
        // of a deck (its structure depth) is lowered to that, by at most BridgeCut, across the
        // deck and feathered over two cells beyond it. The clearance falls to
        // zero over BridgeUnderRamp metres toward each end so the abutment meets
        // the deck. Cells of a ground formation or protected feature are kept.
        const float BridgeUnder = 2.5f, BridgeCut = 10, BridgeUnderRamp = 40;
        var under = new Dictionary<int, float>();

        foreach (var bridge in segments.Where(segment => segment.Bridge && !segment.Tunnel))
        {
            token.ThrowIfCancellationRequested();
            var length = NetworkVerticalProfiles.ArcLength(bridge);
            var steps = Math.Max(1, (int)Math.Ceiling(length / Math.Max(1, cell * .5f)));
            var core = bridge.NativeWidth > 0 ? bridge.NativeWidth * .5f : 6;
            var edge = core + cell * 2;
            var r = (int)Math.Ceiling(edge / cell);

            for (var i = 0; i <= steps; i++)
            {
                var t = i / (float)steps;
                var clearance = BridgeUnder * Math.Min(1, Math.Min(t, 1 - t) * length / BridgeUnderRamp);

                if (clearance <= 0)
                    continue;

                var point = NetworkNormalizer.Evaluate(bridge, t);
                var px = Pixel(point.x);
                var pz = Pixel(point.z);

                for (var z = Math.Max(region.y, pz - r); z <= Math.Min(region.w, pz + r); z++)
                    for (var x = Math.Max(region.x, px - r); x <= Math.Min(region.z, px + r); x++)
                    {
                        var id = z * n + x;
                        var distance = math.distance(new float2(coordinates[x], coordinates[z]), point.xz);

                        if (distance > edge
                            || protectedMask[MaskIndex(x, z)]
                            || lineGuard[MaskIndex(x, z)]
                            || targets.ContainsKey(id))
                            continue;

                        var original = source[id];
                        var target = Math.Max(point.y - clearance, original - BridgeCut);

                        if (target >= original)
                            continue;

                        var blend = distance <= core ? 1 : Numeric.SmoothStep(1 - (distance - core) / (edge - core));
                        target = original + (target - original) * blend;
                        under[id] = under.TryGetValue(id, out var old) ? Math.Min(old, target) : target;
                    }
            }
        }

        var result = new float[regionCells];

        for (var z = region.y; z <= region.w; z++)
            Array.Copy(source, z * n + region.x, result, (z - region.y) * regionWidth, regionWidth);

        var changed = 0;
        var maximum = 0f;

        foreach (var entry in targets)
        {
            token.ThrowIfCancellationRequested();
            var original = source[entry.Key];
            var target = entry.Value.target;
            var blend = entry.Value.blend;
            target = original + (target - original) * blend;
            target =
                math.clamp(
                    target,
                    Math.Max(0, original - entry.Value.budget),
                    Math.Min(scale, original + entry.Value.budget));

            result[ResultIndex(entry.Key)] = target;

            if (result[ResultIndex(entry.Key)] != source[entry.Key])
                changed++;

            maximum = Math.Max(maximum, Math.Abs(target - original));
        }

        foreach (var (id, target) in under)
        {
            var value = math.clamp(target, 0, scale);

            if (value < result[ResultIndex(id)])
            {
                result[ResultIndex(id)] = value;
                changed++;
            }
        }

        log.Count("Roadbed samples lowered under bridge decks", under.Count);
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
        // RoofHeight matches the profile's TunnelDepth.
        // No cover is built over a portal's open cut, and the fill rises to full
        // over the next RoofRamp metres: full cover at the portal stood as a block
        // over the road going in. Cover never rises above any ground road nearby,
        // and falls to it at the shoulder slope: most such roads are guarded beside
        // the bore, so they own no formation here, and cover buried them up to
        // 10 m (at Rosslyn and at the Old Port links).
        const float RoofHeight = 12, RoofFill = 10, RoofSlope = .5f, RoofRamp = 60;
        const int RoofShoulder = 3;
        var roofTargets = new Dictionary<int, float>();

        foreach (var tunnel in segments.Where(segment => segment.Tunnel))
        {
            token.ThrowIfCancellationRequested();
            var core = (tunnel.NativeWidth > 0 ? tunnel.NativeWidth * .5f : 6) + cell;
            var reach = core + RoofShoulder * cell;
            var steps =
                Math.Max(1, (int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(tunnel) / Math.Max(1, cell * .5f)));

            for (var i = 0; i <= steps; i++)
            {
                var point = NetworkNormalizer.Evaluate(tunnel, i / (float)steps);
                var r = (int)Math.Ceiling(reach / cell);
                var px = Pixel(point.x);
                var pz = Pixel(point.z);
                var fill = RoofFill * Numeric.SmoothStep((PortalDistance(point.xz) - PortalOpen) / RoofRamp);

                if (fill <= 0)
                    continue;

                for (var z = Math.Max(region.y, pz - r); z <= Math.Min(region.w, pz + r); z++)
                    for (var x = Math.Max(region.x, px - r); x <= Math.Min(region.z, px + r); x++)
                    {
                        var id = z * n + x;
                        var distance = math.distance(new float2(coordinates[x], coordinates[z]), point.xz);

                        if (areaMask[MaskIndex(x, z)] || distance > reach)
                            continue;

                        var original = source[id];
                        var current = result[ResultIndex(id)];
                        var target =
                            Math.Min(point.y + RoofHeight - Math.Max(0, distance - core) * RoofSlope, original + fill);

                        if (math.isfinite(waterSurface) && original < waterSurface)
                            target = Math.Min(target, waterSurface - 1);

                        if (targets.TryGetValue(id, out var formation) && formation.blend >= 1)
                            target = Math.Min(target, formation.target);

                        if (target <= current)
                            continue;

                        roofTargets[id] =
                            roofTargets.TryGetValue(id, out var higher) ? Math.Max(higher, target) : target;
                    }
            }
        }

        // Every ground section's points at half-cell spacing, start to end, section after
        // section: the roof ceilings below and the refinement sweeps after them both walk
        // them. A section's points run from its start index to the next section's.
        var groundSections = segments.Where(otherSegment => !NetworkStructure.IsStructure(otherSegment)).ToArray();
        var deckPoints = new List<float3>();
        var sectionStarts = new int[groundSections.Length + 1];

        for (var section = 0; section < groundSections.Length; section++)
        {
            var segment = groundSections[section];
            var steps = Math.Max(1, (int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(segment) / (cell * .5f)));
            sectionStarts[section] = deckPoints.Count;

            for (var i = 0; i <= steps; i++)
                deckPoints.Add(NetworkNormalizer.Evaluate(segment, i / (float)steps));
        }

        sectionStarts[groundSections.Length] = deckPoints.Count;

        if (roofTargets.Count > 0)
        {
            // Tiles of Tile x Tile cells holding a roof target, by tile column and row.
            const int Tile = 16;
            var tilesPerRow = (n + Tile - 1) / Tile;
            var tiles = new bool[tilesPerRow * tilesPerRow];
            var sectionPoints = deckPoints.ToArray();

            foreach (var id in roofTargets.Keys)
                tiles[id / n / Tile * tilesPerRow + id % n / Tile] = true;

            bool RoofTile(int column, int row)
                => column >= 0
                    && column < tilesPerRow
                    && row >= 0
                    && row < tilesPerRow
                    && tiles[row * tilesPerRow + column];

            for (var section = 0; section < groundSections.Length; section++)
            {
                token.ThrowIfCancellationRequested();
                var segment = groundSections[section];
                var half = segment.NativeWidth > 0 ? segment.NativeWidth * .5f : 6;
                var reach = half + cell + RoofFill / RoofSlope;
                var r = (int)Math.Ceiling(reach / cell);

                for (var pointIndex = sectionStarts[section]; pointIndex < sectionStarts[section + 1]; pointIndex++)
                {
                    var point = sectionPoints[pointIndex];
                    var px = Pixel(point.x);
                    var pz = Pixel(point.z);
                    var near = false;

                    for (var dz = -1; dz <= 1 && !near; dz++)
                        for (var dx = -1; dx <= 1 && !near; dx++)
                            near = RoofTile(px / Tile + dx, pz / Tile + dz);

                    if (!near)
                        continue;

                    for (var z = Math.Max(region.y, pz - r); z <= Math.Min(region.w, pz + r); z++)
                        for (var x = Math.Max(region.x, px - r); x <= Math.Min(region.z, px + r); x++)
                        {
                            var id = z * n + x;

                            if (!roofTargets.TryGetValue(id, out var target))
                                continue;

                            var ceiling =
                                point.y
                                    + Math.Max(
                                        0,
                                        math.distance(new float2(coordinates[x], coordinates[z]), point.xz)
                                            - half
                                            - cell)
                                        * RoofSlope;

                            if (ceiling < target)
                                roofTargets[id] = ceiling;
                        }
                }
            }
        }

        var roofSamples = 0;

        foreach (var (id, target) in roofTargets)
        {
            if (target <= result[ResultIndex(id)])
                continue;

            result[ResultIndex(id)] = math.clamp(target, 0, scale);
            roofSamples++;
        }

        log.Count("Roadbed tunnel roof samples filled", roofSamples);
        // Each formation sample takes its nearest road's height, but the game
        // reads the ground under a road bilinearly from the four surrounding
        // samples, and where two roads at different heights share those samples
        // (a frontage road beside a raised main road, the arms of a sloping
        // junction) one of them read a blend. Kaczmarz sweeps over every ground
        // deck sample, at the formation's half-cell spacing, move only samples a
        // formation already owns, within their cut/fill budget, toward the
        // least-squares compromise of all the decks that read them. Twelve sweeps
        // halved the replayed terrain mismatches for about 0.3 s; three gave a
        // third, and the gain per sweep was still falling at twelve.
        const int RefineSweeps = 12;

        // A deck point's four samples, their bilinear weights and which of them a sweep may
        // move (inside the region, unprotected, owned by a formation) do not change between
        // sweeps, so they are found once. A point whose movable weight is under one half is
        // never stepped and is dropped. Each movable sample gets a slot holding its refined
        // height, which starts at its built height; a sample that is not movable always
        // reads its built height (the source outside the region, which is never built).
        // Corners are stored four to a point, in the order (x0, z0), (x1, z0), (x0, z1), (x1, z1).
        var cornerHeights = new float[deckPoints.Count * 4];
        var cornerSlots = new int[deckPoints.Count * 4];
        var readWeights = new float[deckPoints.Count * 4];
        var stepWeights = new float[deckPoints.Count * 4];
        var stepNorms = new float[deckPoints.Count];
        var deckHeights = new float[deckPoints.Count];
        var steppedPoints = 0;
        var slotOfCell = new Dictionary<int, int>();
        var slotCells = new List<int>();
        var ids = new int[4];
        var weights = new float[4];

        foreach (var point in deckPoints)
        {
            token.ThrowIfCancellationRequested();
            var gx = math.clamp(point.x / width + .5, 0, 1) * (n - 1);
            var gz = math.clamp(point.z / width + .5, 0, 1) * (n - 1);
            int x0 = (int)gx, z0 = (int)gz, x1 = Math.Min(x0 + 1, n - 1), z1 = Math.Min(z0 + 1, n - 1);
            var tx = (float)(gx - x0);
            var tz = (float)(gz - z0);
            ids[0] = z0 * n + x0;
            ids[1] = z0 * n + x1;
            ids[2] = z1 * n + x0;
            ids[3] = z1 * n + x1;
            weights[0] = (1 - tx) * (1 - tz);
            weights[1] = tx * (1 - tz);
            weights[2] = (1 - tx) * tz;
            weights[3] = tx * tz;
            var first = steppedPoints * 4;
            var norm = 0f;
            var movable = 0f;

            for (var k = 0; k < 4; k++)
            {
                var x = k % 2 == 0 ? x0 : x1;
                var z = k < 2 ? z0 : z1;
                var outside = x < region.x || x > region.z || z < region.y || z > region.w;
                var fixedSample = outside || protectedMask[MaskIndex(x, z)] || !targets.ContainsKey(ids[k]);

                cornerHeights[first + k] = outside ? source[ids[k]] : result[MaskIndex(x, z)];
                readWeights[first + k] = weights[k];
                cornerSlots[first + k] = -1;

                if (fixedSample)
                    weights[k] = 0;
                else if (!slotOfCell.TryGetValue(ids[k], out cornerSlots[first + k]))
                {
                    cornerSlots[first + k] = slotCells.Count;
                    slotOfCell[ids[k]] = slotCells.Count;
                    slotCells.Add(ids[k]);
                }

                stepWeights[first + k] = weights[k];
                norm += weights[k] * weights[k];
                movable += weights[k];
            }

            if (movable < .5f)
                continue;

            stepNorms[steppedPoints] = norm;
            deckHeights[steppedPoints] = point.y;
            steppedPoints++;
        }

        var slotHeights = new float[slotCells.Count];
        var slotOriginals = new float[slotCells.Count];
        var slotBudgets = new float[slotCells.Count];
        var slotRefined = new bool[slotCells.Count];

        for (var slot = 0; slot < slotCells.Count; slot++)
        {
            slotHeights[slot] = result[ResultIndex(slotCells[slot])];
            slotOriginals[slot] = source[slotCells[slot]];
            slotBudgets[slot] = targets[slotCells[slot]].budget;
        }

        float Height(int corner) => cornerSlots[corner] < 0 ? cornerHeights[corner] : slotHeights[cornerSlots[corner]];

        for (var sweep = 0; sweep < RefineSweeps; sweep++)
            for (var pointIndex = 0; pointIndex < steppedPoints; pointIndex++)
            {
                token.ThrowIfCancellationRequested();
                var first = pointIndex * 4;
                var deckHeight = deckHeights[pointIndex];
                var norm = stepNorms[pointIndex];
                var read = 0f;

                for (var k = 0; k < 4; k++)
                    read += readWeights[first + k] * Height(first + k);

                // Where most of the read is fixed ground the step would be amplified
                // into a spike; a sample may pass the deck height by at most 1 m.
                var residual = deckHeight - read;

                if (Math.Abs(residual) < .05f)
                    continue;

                for (var k = 0; k < 4; k++)
                {
                    var weight = stepWeights[first + k];

                    if (weight == 0)
                        continue;

                    var slot = cornerSlots[first + k];
                    var original = slotOriginals[slot];
                    var budget = slotBudgets[slot];
                    var current = slotHeights[slot];
                    var next =
                        math.clamp(
                            current + residual * weight / norm,
                            Math.Min(current, deckHeight) - 1,
                            Math.Max(current, deckHeight) + 1);

                    slotHeights[slot] =
                        math.clamp(next, Math.Max(0, original - budget), Math.Min(scale, original + budget));

                    slotRefined[slot] = true;
                }
            }

        var refinedSamples = 0;

        for (var slot = 0; slot < slotCells.Count; slot++)
        {
            if (!slotRefined[slot])
                continue;

            refinedSamples++;
            var index = ResultIndex(slotCells[slot]);
            var value = math.clamp(slotHeights[slot], 0, scale);

            if (value != result[index])
            {
                if (result[index] == slotOriginals[slot])
                    changed++;

                result[index] = value;
            }
        }

        log.Count("Roadbed samples refined to the decks that read them", refinedSamples);
        log.Count("Roadbed terrain samples changed", changed);
        log.Count("Roadbed protected samples avoided", protectedSamples);
        log.Count("Roadbed samples beyond cut/fill budget", beyondBudget);
        log.Count("Roadbed ground formations beyond support budget", unsupportedSections);
        log.Count("Roadbed approach samples kept to the footprint", narrowedApproachSamples);
        log.Diagnostic(
            $"ROADBED SUPPORT sampledUnsupportedFormations={unsupportedSections}; maximumCenterlineDeficitMeters={maximumSupportDeficit:0.###}; boundedEarthworkNotSupportAcceptance=true; continuousFinalSupportAuditStillRequired=true");

        log.Diagnostic(
            $"PROCESS roadbed complete; changedSamples={changed}; maximumCutFillMeters={maximum:0.###}; target=nearest-curve-station; shoulderMetersPerFillMeter=6; maximumShoulderMeters=240; zeroWeightAtToe=true; targetAndShoulderBoundedBeforeFeather=true; protectedWaterCliffsQuarries=true; structuresExcluded=true; elapsedSeconds={clock.Elapsed.TotalSeconds:0.0}");

        if (localRefit != null)
            log.AddDuration("Local refit patch refinement", partClock.Elapsed.TotalSeconds);

        return result;
    }
}
