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

        // Height in metres at a map point inside the patch.
        public double HeightAt(double x, double z)
            => TerrainSampling.HeightKeepingCrests(
                Samples,
                Crests,
                Width,
                Height,
                (x - Minimum.x) / Math.Max(1e-8, Maximum.x - Minimum.x),
                (z - Minimum.y) / Math.Max(1e-8, Maximum.y - Minimum.y));

        public void CopyTo(float[] destination)
        {
            for (var z = 0; z < Height; z++)
                Array.Copy(Samples, z * Width, destination, (Z + z) * Resolution + X, Width);
        }
    }

    public static Patch BuildLocalPatch(
        PreparedImport prepared,
        float[] current,
        IReadOnlyList<PreparedSegment> plan,
        int start,
        IReadOnlyList<PreparedSegment> revision, ImportLog log, CancellationToken token)
    {
        var baseline = prepared.RoadbedBaseline;
        var resolution = TerrainSampling.Resolution(baseline);

        if (current.Length != baseline.Length || resolution < 2)
            throw new InvalidOperationException("Local roadbed baseline is unavailable.");

        var otherLow = new float2(float.PositiveInfinity);
        var otherHigh = new float2(float.NegativeInfinity);

        foreach (var segment in plan.Skip(start).Take(revision.Count).Concat(revision))
            foreach (var point in new[] {
                segment.CourseStart,
                segment.CourseControl1,
                segment.CourseControl2,
                segment.CourseEnd
                })
            {
                otherLow = math.min(otherLow, point.xz);
                otherHigh = math.max(otherHigh, point.xz);
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

        var all = plan.ToArray();

        for (var i = 0; i < revision.Count; i++)
            all[start + i] = revision[i];

        bool Nearby(PreparedSegment segment)
        {
            var min =
                math.min(
                    math.min(segment.CourseStart.xz, segment.CourseControl1.xz),
                    math.min(segment.CourseControl2.xz, segment.CourseEnd.xz));

            var max =
                math.max(
                    math.max(segment.CourseStart.xz, segment.CourseControl1.xz),
                    math.max(segment.CourseControl2.xz, segment.CourseEnd.xz));

            var reach = 240 + Math.Max(0, segment.NativeWidth) + 2 * cell;

            return math.all(max >= otherLow - (float)reach) && math.all(min <= otherHigh + (float)reach);
        }

        // A unit can run for kilometres, so its bounding box holds far more ground than it
        // changes. Earthworks reach at most `margin` from a section, so only cells within
        // that distance of an old or revised section can differ, and only sections within a
        // second `margin` of those can shape such a cell. Everything else keeps `current`.
        var changedBoxes =
            plan.Skip(start)
                .Take(revision.Count)
                .Concat(revision)
                .Select(
                    segment =>
                    {
                        var min =
                            math.min(
                                math.min(segment.CourseStart.xz, segment.CourseControl1.xz),
                                math.min(segment.CourseControl2.xz, segment.CourseEnd.xz));

                        var max =
                            math.max(
                                math.max(segment.CourseStart.xz, segment.CourseControl1.xz),
                                math.max(segment.CourseControl2.xz, segment.CourseEnd.xz));

                        return (Low: min - new float2((float)margin), High: max + new float2((float)margin));
                    })
                .ToArray();

        bool NearChange(PreparedSegment segment)
        {
            if (!Nearby(segment))
                return false;

            var min =
                math.min(
                    math.min(segment.CourseStart.xz, segment.CourseControl1.xz),
                    math.min(segment.CourseControl2.xz, segment.CourseEnd.xz));

            var max =
                math.max(
                    math.max(segment.CourseStart.xz, segment.CourseControl1.xz),
                    math.max(segment.CourseControl2.xz, segment.CourseEnd.xz));

            var reach = (float)(240 + Math.Max(0, segment.NativeWidth) + 2 * cell);

            foreach (var box in changedBoxes)
                if (math.all(max >= box.Low - reach) && math.all(min <= box.High + reach))
                    return true;

            return false;
        }

        var neighbours = all.Where(NearChange).ToArray();
        var rebuilt =
            Prepare(
                baseline,
                prepared.HeightScaleMeters,
                width,
                neighbours,
                prepared.RoadbedProtectedAreas,
                prepared.RoadbedProtectedLines,
                log,
                token,
                region);

        // Earthworks may cut a defence where a road crosses it; it goes back up, as in design.
        if (prepared.DefenceCrests?.Length == rebuilt.Length)
            log.Count("Local refit flood defence cells restored", FloodDefences.Restore(rebuilt, prepared.DefenceCrests));

        var issued =
            all.Take(start).Where(segment => !NetworkStructure.IsStructure(segment) && NearChange(segment)).ToArray();

        var w = region.z - region.x + 1;
        var h = region.w - region.y + 1;
        var samples = new float[w * h];
        var preserved = 0;
        var protectedCores = new bool[w * h];
        var changeable = new bool[w * h];

        foreach (var box in changedBoxes)
            for (var z = Math.Max(region.y, Pixel(box.Low.y)); z <= Math.Min(region.w, Pixel(box.High.y) + 1); z++)
                for (var x = Math.Max(region.x, Pixel(box.Low.x)); x <= Math.Min(region.z, Pixel(box.High.x) + 1); x++)
                    changeable[(z - region.y) * w + x - region.x] = true;

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

        for (var z = 0; z < h; z++)
            for (var x = 0; x < w; x++)
            {
                token.ThrowIfCancellationRequested();
                var gx = region.x + x;
                var gz = region.y + z;
                var id = gz * resolution + gx;
                // Preserve already-issued ground cores, including an interpolation
                // cell of support. A local retry owns only its unissued support.
                var edits = prepared.RoadbedLaterEdits;
                var protectedCore =
                    !changeable[z * w + x]
                    || protectedCores[z * w + x]
                    || edits?.Mask.Length == current.Length && edits.Mask[id] != 0;

                samples[z * w + x] = protectedCore ? current[id] : rebuilt[id];

                if (protectedCore && changeable[z * w + x])
                    preserved++;
            }

        log.Diagnostic(
            $"LOCAL ROADBED staged samples={samples.Length}; neighbourSections={neighbours.Length}; issuedCoreSamplesPreserved={preserved}; baselineImmutable=true; nativeMutation=false");

        float[]? crests = null;

        if (prepared.DefenceCrests?.Length == rebuilt.Length)
        {
            crests = new float[w * h];

            for (var z = 0; z < h; z++)
                Array.Copy(prepared.DefenceCrests, (region.y + z) * resolution + region.x, crests, z * w, w);
        }

        var patch = new Patch(region.x, region.y, w, h, samples, resolution, width) { Crests = crests };
        patch.CopyTo(rebuilt);
        NetworkTerrainSupport.Validate(
            revision,
            rebuilt,
            prepared.HeightScaleMeters,
            width,
            log,
            token,
            scope: $"native-unit-{revision[0].NativeUnitId}");

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

        var cell = (float)(width / (n - 1));
        var protectedMask = new bool[source.Length];
        var lineGuard = new bool[source.Length];
        var region = dirtyRegion ?? new int4(0, 0, n - 1, n - 1);
        // Inside mapped water and quarry polygons only, without the guard bands
        // around them and around cliff, waterway and coast lines: tunnel roof
        // cover may be built there (Marseille's tunnels pass beside cliffs and
        // under buried streams, whose bands left their bores exposed).
        var areaMask = new bool[source.Length];
        var coordinates = new float[n];

        for (var i = 0; i < n; i++)
            coordinates[i] = (i / (float)(n - 1) - .5f) * (float)width;

        int Pixel(float value) => (int)Math.Round((value / width + .5) * (n - 1));

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
                        (mask ?? protectedMask)[z * n + x] = true;
                }
        }

        // Protect the actual polygon, not its rectangular envelope. A winding
        // river's bounding box can cover kilometres of dry roads and prevent
        // their entire cut/fill pass from being applied.
        foreach (var ring in protectedAreas)
        {
            token.ThrowIfCancellationRequested();

            if (ring.Length < 3)
                continue;

            var crossings = new List<float>(ring.Length);

            for (var z =
                Math.Max(
                    region.y,
                    Pixel(ring.Min(point => point.z))); z <= Math.Min(region.w, Pixel(ring.Max(point => point.z))); z++)
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
                        protectedMask[z * n + x] = areaMask[z * n + x] = true;
            }

            for (var i = 0; i < ring.Length; i++)
                Guard(ring[i], ring[(i + 1) % ring.Length], cell);
        }

        // Bands along waterway, cliff and coast lines keep road shoulders and fills
        // off them, but not a ground road's own core and the grid cell beside it,
        // from which the ground under its centre is read: the road stands there in
        // game whatever the ground does, and left unstamped it floated or sank
        // beside bridges over streams (300 of Marseille's 1,455 mismatches).
        foreach (var line in protectedLines)
            for (var i = 1; i < line.Length; i++)
            {
                token.ThrowIfCancellationRequested();
                Guard(line[i - 1], line[i], cell * 2, lineGuard);
            }

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
                var point = Station(i);
                var px = Pixel(point.x);
                var pz = Pixel(point.z);
                var previous = Station(Math.Max(-extendStart, i - 1));
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

                for (var z = Math.Max(region.y, pz - reach); z <= Math.Min(region.w, pz + reach); z++)
                    for (var x = Math.Max(region.x, px - reach); x <= Math.Min(region.z, px + reach); x++)
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

                        if (protectedMask[id] || lineGuard[id] && distance > half + cell)
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

            if (unsupported)
                unsupportedSections++;

            if (clock.Elapsed.TotalSeconds >= nextProgress)
            {
                log.Diagnostic(
                    $"PROCESS roadbed sections={processed}; affectedSamples={targets.Count}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.0}");

                nextProgress = clock.Elapsed.TotalSeconds + 5;
            }
        }

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

                        if (distance > edge || protectedMask[id] || lineGuard[id] || targets.ContainsKey(id))
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

        var result = (float[])source.Clone();
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

            result[entry.Key] = target;

            if (result[entry.Key] != source[entry.Key])
                changed++;

            maximum = Math.Max(maximum, Math.Abs(target - original));
        }

        foreach (var (id, target) in under)
        {
            var value = math.clamp(target, 0, scale);

            if (value < result[id])
            {
                result[id] = value;
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

                        if (areaMask[id] || distance > reach)
                            continue;

                        var original = source[id];
                        var current = result[id];
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

        if (roofTargets.Count > 0)
        {
            const int Tile = 16;
            var tiles = new HashSet<(int, int)>();

            foreach (var id in roofTargets.Keys)
                tiles.Add((id % n / Tile, id / n / Tile));

            foreach (var segment in segments.Where(otherSegment => !NetworkStructure.IsStructure(otherSegment)))
            {
                token.ThrowIfCancellationRequested();
                var half = segment.NativeWidth > 0 ? segment.NativeWidth * .5f : 6;
                var reach = half + cell + RoofFill / RoofSlope;
                var r = (int)Math.Ceiling(reach / cell);
                var steps = Math.Max(1, (int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(segment) / (cell * .5f)));

                for (var i = 0; i <= steps; i++)
                {
                    var point = NetworkNormalizer.Evaluate(segment, i / (float)steps);
                    var px = Pixel(point.x);
                    var pz = Pixel(point.z);
                    var near = false;

                    for (var dz = -1; dz <= 1 && !near; dz++)
                        for (var dx = -1; dx <= 1 && !near; dx++)
                            near = tiles.Contains((px / Tile + dx, pz / Tile + dz));

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
            if (target <= result[id])
                continue;

            result[id] = math.clamp(target, 0, scale);
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
        var refined = new Dictionary<int, float>();

        float Height(int id) => refined.TryGetValue(id, out var h) ? h : result[id];

        var deckPoints = new List<float3>();

        foreach (var segment in segments.Where(otherSegment => !NetworkStructure.IsStructure(otherSegment)))
        {
            var steps = Math.Max(1, (int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(segment) / (cell * .5f)));

            for (var i = 0; i <= steps; i++)
                deckPoints.Add(NetworkNormalizer.Evaluate(segment, i / (float)steps));
        }

        var ids = new int[4];
        var weights = new float[4];

        for (var sweep = 0; sweep < RefineSweeps; sweep++)
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
                var read = 0f;
                var norm = 0f;
                var movable = 0f;

                for (var k = 0; k < 4; k++)
                {
                    read += weights[k] * Height(ids[k]);
                    var x = k % 2 == 0 ? x0 : x1;
                    var z = k < 2 ? z0 : z1;

                    if (x < region.x
                        || x > region.z
                        || z < region.y
                        || z > region.w
                        || protectedMask[ids[k]]
                        || !targets.ContainsKey(ids[k]))
                        weights[k] = 0;

                    norm += weights[k] * weights[k];
                    movable += weights[k];
                }

                // Where most of the read is fixed ground the step would be amplified
                // into a spike; a sample may pass the deck height by at most 1 m.
                var residual = point.y - read;

                if (Math.Abs(residual) < .05f || movable < .5f)
                    continue;

                for (var k = 0; k < 4; k++)
                {
                    if (weights[k] == 0)
                        continue;

                    var id = ids[k];
                    var original = source[id];
                    var budget = targets[id].budget;
                    var current = Height(id);
                    var next =
                        math.clamp(
                            current + residual * weights[k] / norm,
                            Math.Min(current, point.y) - 1,
                            Math.Max(current, point.y) + 1);

                    refined[id] = math.clamp(next, Math.Max(0, original - budget), Math.Min(scale, original + budget));
                }
            }

        foreach (var (id, height) in refined)
        {
            var value = math.clamp(height, 0, scale);

            if (value != result[id])
            {
                if (result[id] == source[id])
                    changed++;

                result[id] = value;
            }
        }

        log.Count("Roadbed samples refined to the decks that read them", refined.Count);
        log.Count("Roadbed terrain samples changed", changed);
        log.Count("Roadbed protected samples avoided", protectedSamples);
        log.Count("Roadbed samples beyond cut/fill budget", beyondBudget);
        log.Count("Roadbed ground formations beyond support budget", unsupportedSections);
        log.Count("Roadbed approach samples kept to the footprint", narrowedApproachSamples);
        log.Diagnostic(
            $"ROADBED SUPPORT sampledUnsupportedFormations={unsupportedSections}; maximumCenterlineDeficitMeters={maximumSupportDeficit:0.###}; boundedEarthworkNotSupportAcceptance=true; continuousFinalSupportAuditStillRequired=true");

        log.Diagnostic(
            $"PROCESS roadbed complete; changedSamples={changed}; maximumCutFillMeters={maximum:0.###}; target=nearest-curve-station; shoulderMetersPerFillMeter=6; maximumShoulderMeters=240; zeroWeightAtToe=true; targetAndShoulderBoundedBeforeFeather=true; protectedWaterCliffsQuarries=true; structuresExcluded=true; elapsedSeconds={clock.Elapsed.TotalSeconds:0.0}");

        return result;
    }
}
