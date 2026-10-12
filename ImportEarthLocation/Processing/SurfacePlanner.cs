using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using ImportEarthLocation.Geo;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

/// <summary>Resolve OSM, morphology and terrain-following moisture into one brush, not paint layers.</summary>
public static class SurfacePlanner
{
    private enum Cover : byte { Unknown, Vegetation, Scrub, Heath, Wetland, Dry, Sand, Rock, Earth, NonSand }
    private readonly record struct Evidence(Cover Cover, byte Priority);

    public static SurfaceMask Prepare(IReadOnlyList<GeographicPolygon> polygons, ImportRequest request,
        TerrainBuilder.Result terrain, ImportLog log, CancellationToken token, OsmDataset? geography = null, int resolution = 2048, ClimateNormals? climate = null)
    {
        var clock = Stopwatch.StartNew(); var width = request.MapSize.WidthMeters;
        var hn = (int)Math.Sqrt(terrain.Heightmap.Length);
        if (resolution < 16 || resolution > 2048 || hn < 2 || (long)hn * hn != terrain.Heightmap.Length ||
            !Finite(width) || width <= 0 || !Finite(terrain.HeightScaleMeters) || terrain.HeightScaleMeters <= 0 || !Finite(terrain.SeaLevelMeters))
            throw new ArgumentException("Surface preparation requires finite bounds and a square valid terrain raster.");
        token.ThrowIfCancellationRequested();
        var size = resolution * resolution; var cell = (float)(width / resolution);
        var cover = new byte[size]; var priority = new byte[size]; var beachCoverage = new byte[size]; var urbanContext = new bool[size];
        byte[]? beachSubsamples = null;
        var mapped = 0; var beachPolygons = 0; var beachSamples = 0;
        var provenance = new List<object>();
        foreach (var polygon in polygons.OrderBy(p => p.Id))
        {
            // Land use is context, not proof that every square metre is paved.
            var use = Tag(polygon.Tags, "landuse");
            if (use == "residential" || use == "commercial" || use == "industrial" || use == "retail" || use == "construction" || use == "brownfield" || use == "cemetery")
                Rasterize(polygon, resolution, width, (a, b) => { for (var i = a; i < b; i++) urbanContext[i] = true; }, token);
            var evidence = Classify(polygon.Tags); if (evidence.Cover == Cover.Unknown) continue;
            if (provenance.Count < 32) provenance.Add(new { polygon.Id, physicalCover = evidence.Cover.ToString(), priority = evidence.Priority,
                useContext = use, natural = Tag(polygon.Tags, "natural"), surface = Tag(polygon.Tags, "surface"), landcover = Tag(polygon.Tags, "landcover"), outerRings = polygon.Outer.Count, innerRings = polygon.Inner.Count });
            var beach = Tag(polygon.Tags, "natural") == "beach" && evidence.Cover == Cover.Sand;
            if (beach) beachPolygons++;
            void Fill(int start, int end)
            {
                // OSM way and relation IDs can coincide. Resolve equal-priority
                // conflicting physical covers explicitly, not by input order.
                for (var i = start; i < end; i++) if (evidence.Priority > priority[i] ||
                    evidence.Priority == priority[i] &&
                    (evidence.Cover == Cover.NonSand || evidence.Cover == Cover.Wetland && cover[i] != (byte)Cover.NonSand ||
                     cover[i] != (byte)Cover.Wetland && (byte)evidence.Cover > cover[i]))
                { if (beach && cover[i] != (byte)Cover.Sand) beachSamples++; cover[i] = (byte)evidence.Cover; priority[i] = evidence.Priority; }
            }
            if (beach)
            {
                beachSubsamples ??= new byte[size]; var occupied = new List<int>();
                // Sample narrow mapped beaches at quarter-cell spacing, then
                // retain their footprint in the available native paint cell.
                // No generic beach is invented along rocky or urban coastlines.
                Rasterize(polygon, resolution * 4, width, (start, end) =>
                {
                    for (var sample = start; sample < end; sample++)
                    {
                        var i = sample / (resolution * 4) / 4 * resolution + sample % (resolution * 4) / 4;
                        if (beachSubsamples[i] == 0) occupied.Add(i);
                        beachSubsamples[i] = (byte)Math.Min(16, beachSubsamples[i] + 1);
                    }
                }, token);
                foreach (var i in occupied)
                {
                    Fill(i, i + 1);
                    if (cover[i] == (byte)Cover.Sand) beachCoverage[i] = Math.Max(beachCoverage[i], (byte)(beachSubsamples[i] * 255 / 16));
                    beachSubsamples[i] = 0;
                }
            }
            else Rasterize(polygon, resolution, width, Fill, token);
            mapped++;
        }
        // Neighborhood statistics and drainage are computed once on a bounded
        // context grid, then smoothly interpolated to the full paint resolution.
        var context = new Context(Math.Min(resolution, 1024), width, terrain, token);
        context.Moisture(polygons, geography, request, token);
        var desert = context.Desert(polygons, cover, resolution, WaterLevels.Effective(terrain.SeaLevelMeters), log, token, climate);
        var edgeDistance = BoundaryDistance(cover, resolution, cell, token);
        var material = new byte[size]; var intensity = new byte[size]; var confidence = new byte[size];
        var sea = WaterLevels.Effective(terrain.SeaLevelMeters); var underwater = 0; var inferredSand = 0;
        var desertDecisions = new int[7]; // diagnostics only; never used to choose a brush
        for (var z = 0; z < resolution; z++)
        {
            token.ThrowIfCancellationRequested(); var worldZ = (z + .5) * cell - width * .5;
            for (var x = 0; x < resolution; x++)
            {
                var i = z * resolution + x; var worldX = (x + .5) * cell - width * .5; var p = context.At(worldX, worldZ);
                var submerged = Height(terrain.Heightmap, hn, terrain.HeightScaleMeters, width, worldX, worldZ) < sea;
                if (submerged) underwater++;
                var evidence = (Cover)cover[i];
                desertDecisions[evidence!=Cover.Unknown?0:submerged?1:!desert?2:p.Flat<=.08f?3:p.Rough>=.35f?4:Math.Max(p.Wet,p.WeakWet)>=.35f?5:6]++;
                var transition = evidence == Cover.Rock || evidence == Cover.Earth ? 45f : 140f;
                var edge = Smooth((edgeDistance[i] + cell * .5f) / Math.Max(transition, cell * 1.5f));
                // Deterministic low-frequency intensity variation, never random material switches.
                var variation = .88 + .24 * ValueNoise(worldX / 380, worldZ / 380, 0x31415926u);
                GroundMaterial winner = GroundMaterial.Base; var strength = 0f; byte certainty = 0;
                if (evidence == Cover.Sand || evidence == Cover.Rock || evidence == Cover.Earth)
                {
                    winner = evidence == Cover.Sand ? GroundMaterial.Sand : GroundMaterial.Dirt;
                    // Sand's footprint is resolved first. A single terrain-aware
                    // ecotone below handles explicit and inferred sand together,
                    // rather than giving trusted polygons a narrow hard outline.
                    strength = evidence == Cover.Sand ? .96f : edge * (evidence == Cover.Rock ? .88f : .92f); certainty = priority[i];
                }
                else if (evidence != Cover.NonSand)
                {
                    var moisture = p.Wet * (1 - .72f * p.Steep) * (1 + .2f * p.Valley);
                    var exposure = .5f * p.Ridge + .38f * p.Steep + .08f * p.Exposure;
                    var dryness = evidence == Cover.Scrub ? .64f * edge : evidence == Cover.Heath ? .38f * edge : 0;
                    var wetland = evidence == Cover.Wetland ? .88f * edge * (1 - .45f * p.Steep) : 0;
                    // An intermittent drain can moderate Dry, but cannot create Lush alone.
                    var vegetation = Math.Max(moisture, wetland) - exposure - dryness + .24f * p.WeakWet;
                    if (evidence == Cover.Dry)
                    { winner = GroundMaterial.DryGrass; strength = edge * (.72f - .25f * Math.Max(p.Wet, p.WeakWet)); certainty = priority[i]; }
                    else if (vegetation > .24f)
                    { winner = GroundMaterial.Grass; strength = Math.Min(.86f, (vegetation - .24f) * 1.2f); certainty = (byte)Math.Max(100, Math.Min(210, 100 + 110 * vegetation)); }
                    else if (vegetation < -.24f)
                    { winner = GroundMaterial.DryGrass; strength = Math.Min(.82f, (-vegetation - .24f) * 1.35f); certainty = (byte)Math.Max(85, Math.Min(190, 85 - 100 * vegetation)); }
                    else if (evidence == Cover.Vegetation || evidence == Cover.Unknown && !urbanContext[i])
                    {
                        // Subtle continuous grass variation, not isolated random brush pixels.
                        winner = exposure > .12f ? GroundMaterial.DryGrass : GroundMaterial.Grass;
                        strength = exposure > .12f ? .08f + .2f * exposure : .06f + .16f * (float)ValueNoise(worldX / 550, worldZ / 550, 0xa24baed5u);
                        certainty = 105;
                    }
                    // Dryness or slope alone is not barren ground. Only unmapped,
                    // steep AND rough AND locally exposed terrain admits weak Dirt inference.
                    if (evidence == Cover.Unknown && p.Steep > .35f && p.Rough > .2f && p.Ridge > .15f && p.Wet < .3f)
                    { winner = GroundMaterial.Dirt; strength = Math.Min(.32f, (p.Steep - .35f) * .35f + (p.Rough - .2f) * .2f) * (.4f + .6f * p.Ridge); certainty = 110; }
                    if (desert && evidence == Cover.Unknown && Math.Max(p.Wet, p.WeakWet) < .35f)
                    {
                        // Arid rocky hills are desert too, but they are not dunes.
                        // Do not leave their unclassified background lush green.
                        winner = GroundMaterial.Dirt;
                        strength = .38f + .3f * Math.Max(p.Steep, p.Rough); certainty = 165;
                    }
                    // Flatness alone never establishes sand. Regional sand/dune or
                    // existing hot/arid climate permits inference on unmapped, dry, flat
                    // land. Every mapped non-sand cover takes precedence.
                    if (desert && evidence == Cover.Unknown && p.Flat > .08f && p.Rough < .35f && Math.Max(p.Wet, p.WeakWet) < .35f)
                    {
                        winner = GroundMaterial.Sand;
                        strength = (.7f + .26f * Smooth(p.Flat/.6f)) * (1 - .45f * Smooth(Math.Max(p.Wet, p.WeakWet) / .35f)) * (1 - .3f * p.Rough);
                        certainty = 200; inferredSand++;
                    }
                }
                // User policy: a confirmed desert remains sandy on hills and
                // mountains too. This final precedence intentionally overrides
                // vegetation/rock/dirt painting, but never water or paved/urban
                // exclusions. Regional gating prevents unrelated grassy maps
                // from becoming deserts merely because they contain a beach.
                if (desert && evidence != Cover.NonSand && evidence != Cover.Wetland)
                {
                    strength = winner == GroundMaterial.Sand ? Math.Max(strength, .88f) : .88f;
                    winner = GroundMaterial.Sand;
                    certainty = Math.Max(certainty, (byte)200);
                }
                if (winner == GroundMaterial.Base) continue;
                var value = (byte)Math.Round(255 * Clamp(strength * variation)); if (value == 0) continue;
                material[i] = (byte)winner; intensity[i] = value; confidence[i] = certainty;
            }
        }
        // Inferred patches also feather to Default at their own resolved boundary.
        // No competing brush is introduced by this smoothing operation.
        var desertGaps = desert ? BlendNarrowDesertGaps(material, intensity, cover, resolution, cell, width, context, sea, token) : 0;
        log.Diagnostic($"SURFACE DESERT GAP filledSamples={desertGaps}; maximumBracketReachMeters=100; explicitCoversProtected=true; wetGapsRetained=true");
        var finalDistance = BoundaryDistance(material, resolution, cell, token);
        // Feather the original sandy footprint before adding exterior density.
        // Otherwise the inward pass attenuates the already-faded outward band
        // a second time, making compatible exterior sand effectively invisible.
        var sandTransition = NaturalSandTransition(material, intensity, finalDistance, resolution, cell, width, context, sea, token);
        var outward = ExpandCompatibleSand(material, intensity, confidence, cover, urbanContext, resolution, cell, width, context, sea, token);
        if (outward > 0) finalDistance = BoundaryDistance(material, resolution, cell, token);
        for (var i = 0; i < size; i++)
        {
            if ((i & 65535) == 0) token.ThrowIfCancellationRequested();
            if (beachCoverage[i] > 0 && material[i] == (byte)GroundMaterial.Sand)
                intensity[i] = (byte)Math.Max(1, Math.Round(intensity[i] * beachCoverage[i] / 255d));
        }
        log.DiagnosticRecords("surface-evidence.ndjson", provenance);
        log.Count("Surface fractional beach samples", beachCoverage.Count(v => v > 0 && v < 255));
        log.Count("Surface outward ecotone samples", outward);
        log.Count("Surface urban context samples", urbanContext.Count(v => v));
        log.Diagnostic($"SURFACE PROVENANCE version=physical-use-policy-v2; urbanContextSamples={urbanContext.Count(v => v)}; fractionalBeachSamples={beachCoverage.Count(v => v > 0 && v < 255)}; beachEquivalentFullSamples={beachCoverage.Sum(v => (long)v) / 255d:R}; outwardEcotoneSamples={outward}; actualPavingProtected=true; landUseIsNotSubstrate=true");
        for (var i = 0; i < size; i++)
        {
            if ((i & 65535) == 0) token.ThrowIfCancellationRequested();
            if (material[i] != 0 && material[i] != (byte)GroundMaterial.Sand && confidence[i] < 225)
                intensity[i] = (byte)Math.Round(intensity[i] * Smooth((finalDistance[i] + cell * .5f) / Math.Max(material[i] == (byte)GroundMaterial.Dirt ? 45 : 100, cell * 1.5f)));
        }
        SmoothExclusive(material, intensity, resolution, token);
        var counts = new int[5]; var totalStrength = new long[5]; var transitions = new SurfaceTransition[size];
        for (var i = 0; i < size; i++)
        {
            if (intensity[i] == 0) { material[i] = 0; confidence[i] = 0; }
            counts[material[i]]++; totalStrength[material[i]] += intensity[i];
            transitions[i] = material[i] == (byte)GroundMaterial.Dirt ? SurfaceTransition.Medium : SurfaceTransition.Soft;
        }
        var report = "Ground surfaces: exclusive Default/Dry/Lush/Dirt/Sand; OSM + terrain-following moisture; " +
            string.Join(", ", Enumerable.Range(0, 5).Select(m => $"{Label((GroundMaterial)m)} {100d * counts[m] / size:0.#}%")) + ".";
        log.Info(report);
        log.Diagnostic($"SURFACE DESERT DECISIONS mappedCoverProtected={desertDecisions[0]}; submergedUnknown={desertDecisions[1]}; regionalGateClosedUnknown={desertDecisions[2]}; insufficientFlatness={desertDecisions[3]}; excessiveRoughness={desertDecisions[4]}; wetOrDrainageInfluenced={desertDecisions[5]}; eligibleUnknownSandSites={desertDecisions[6]}; beforeEcotone=true; exclusivePriorityCounts=true");
        log.Diagnostic($"SURFACE ECOTONE model=terrain-aware-correlated-density; sandTransitionSamples={sandTransition.Samples}; transitionWidthMeters={sandTransition.MinWidth:0.#}..{sandTransition.MaxWidth:0.#}; sizeLimitedSandRegions={sandTransition.NarrowRegions}; mappedNonSandExpanded=false; cropEdgeFade=false; exteriorFadeAppliedOnce=true; nativeExclusiveBrushes=true; appearanceVerified=false");
        log.Diagnostic($"SURFACE BEACH sandyPolygons={beachPolygons}; footprintSamples={beachSamples}; samplingFactor=4; maximumFootprintExpansionMeters={cell:R}; genericCoastSandInvented=false");
        log.Diagnostic($"SURFACE DESERT PRECEDENCE enabled={desert}; slopeAndLandCoverPaintOverridden=true; mappedWaterWetlandActualPavingProtected=true; urbanUseAloneIsNotPaving=true; mappedVegetationMayHaveSandUnderlay=true");
        log.Diagnostic($"SURFACE PLAN resolution={resolution}; contextResolution={context.N}; mappedPolygons={mapped}; permanentWaterSeeds={context.PermanentSeeds}; intermittentWaterSeeds={context.WeakSeeds}; underwaterSamples={underwater}; desertSupported={desert}; inferredSandSamples={inferredSand}; paintOrderIndependent=true; climateInputs={climate!=null}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.###}; meanIntensity=" +
            string.Join(",", Enumerable.Range(1, 4).Select(m => $"{Label((GroundMaterial)m)}:{(counts[m] == 0 ? 0 : 100d * totalStrength[m] / counts[m] / 255):0.#}%")));
        var mask = new SurfaceMask(resolution, width, material, intensity, report) { Confidence = confidence, Transitions = transitions };
        SurfacePaint.Validate(mask); return mask;
    }

    private static Evidence Classify(IReadOnlyDictionary<string, string> tags)
    {
        if (tags.ContainsKey("highway") || tags.ContainsKey("railway") || tags.ContainsKey("building")) return default;
        var natural = Tag(tags, "natural"); var landuse = Tag(tags, "landuse"); var surface = Tag(tags, "surface");
        if (natural.Length == 0 && landuse.Length == 0 && Tag(tags, "landcover").Length == 0 && surface.Length == 0 && Tag(tags, "area") != "yes" && !tags.ContainsKey("water") && !tags.ContainsKey("wetland") && Tag(tags, "waterway") != "riverbank") return default;
        // A water mask describes simulation, not the underlying ground material.
        // Keep substrate rules active in case the player later raises the bed.
        if (natural == "water" || natural == "bay" || tags.ContainsKey("water") || Tag(tags, "waterway") == "riverbank" || landuse == "reservoir") return default;
        // Water/wetland and actual paving remain hard exclusions. District land
        // use cannot suppress observed physical cover within that district.
        if (natural == "wetland" || tags.ContainsKey("wetland")) return new(Cover.Wetland, 255);
        if (surface == "asphalt" || surface == "concrete" || surface == "paved" || surface == "paving_stones" || surface == "cobblestone") return new(Cover.NonSand, 255);
        if (surface == "sand") return new(Cover.Sand, 255);
        if (surface == "rock" || surface == "stone" || surface == "gravel" || surface == "pebblestone" || surface == "shingle") return new(Cover.Rock, 255);
        if (surface == "earth" || surface == "ground" || surface == "dirt" || surface == "mud") return new(Cover.Earth, 250);
        if (surface == "dry_grass") return new(Cover.Dry, 245);
        if (surface == "grass") return new(Cover.Vegetation, 245);
        if (natural == "sand" || natural == "dune" || Tag(tags, "landcover") == "sand") return new(Cover.Sand, 245);
        // Preserve the existing unspecified sandy-beach fallback, not a generic coastline buffer.
        if (natural == "beach") return surface.Length == 0 ? new(Cover.Sand, 235) : default;
        if (natural == "bare_rock" || natural == "scree" || natural == "shingle" || natural == "rock") return new(Cover.Rock, 240);
        if (natural == "mud" || natural == "bare_earth" || landuse == "quarry") return new(Cover.Earth, 240);
        if (natural == "scrub" || Tag(tags, "landcover") == "scrub") return new(Cover.Scrub, 200);
        if (natural == "heath") return new(Cover.Heath, 190);
        if (Tag(tags, "landcover") == "grass" || Tag(tags, "landcover") == "trees" || natural == "grassland" || natural == "wood" || landuse == "grass" || landuse == "meadow" || landuse == "forest" ||
            landuse == "farmland" || landuse == "orchard" || landuse == "vineyard" || surface == "grass") return new(Cover.Vegetation, 170);
        return default;
    }

    // Union outer rings minus holes using scanline intervals. Cost follows covered
    // spans instead of multiplying pixel-count by every polygon vertex.
    private static void Rasterize(GeographicPolygon polygon, int n, double width, Action<int, int> fill, CancellationToken token)
    {
        var edges = new List<(float3 A, float3 B, int Ring, bool Inner)>(); var ringNumber = 0;
        foreach (var rings in new[] { polygon.Outer, polygon.Inner }) foreach (var ring in rings)
        {
            token.ThrowIfCancellationRequested();
            for (var i = 0; i < ring.Length; i++)
            {
                var a = ring[i]; var b = ring[(i + 1) % ring.Length];
                if (!math.all(math.isfinite(a)) || !math.all(math.isfinite(b))) throw new ArgumentException("Non-finite surface polygon.");
                if (a.z != b.z) edges.Add(a.z < b.z ? (a, b, ringNumber, rings == polygon.Inner) : (b, a, ringNumber, rings == polygon.Inner));
            }
            ringNumber++;
        }
        if (edges.Count == 0) return;
        edges.Sort((a, b) => a.A.z.CompareTo(b.A.z)); var cell = width / n; var half = width * .5; var next = 0;
        var active = new List<(float3 A, float3 B, int Ring, bool Inner)>(); var crossings = new List<(double X, int Ring, bool Inner)>(); var events = new List<(double X, int Outer, int Inner)>();
        var minRow = Pixel(edges[0].A.z, n, half, cell); var maxRow = Pixel(edges.Max(e => e.B.z), n, half, cell);
        for (var z = minRow; z < maxRow; z++)
        {
            token.ThrowIfCancellationRequested(); var y = (z + .5) * cell - half;
            while (next < edges.Count && edges[next].A.z <= y) active.Add(edges[next++]);
            crossings.Clear(); events.Clear();
            for (var j = active.Count - 1; j >= 0; j--)
            {
                var e = active[j]; if (e.B.z <= y) { active.RemoveAt(j); continue; }
                crossings.Add((e.A.x + (y - e.A.z) / (e.B.z - e.A.z) * (e.B.x - e.A.x), e.Ring, e.Inner));
            }
            crossings.Sort((a, b) => a.Ring != b.Ring ? a.Ring.CompareTo(b.Ring) : a.X.CompareTo(b.X));
            for (var j = 0; j + 1 < crossings.Count; j += 2)
            {
                var a = crossings[j]; var b = crossings[j + 1]; if (a.Ring != b.Ring) throw new ArgumentException("Invalid surface polygon crossings.");
                events.Add((a.X, a.Inner ? 0 : 1, a.Inner ? 1 : 0)); events.Add((b.X, a.Inner ? 0 : -1, a.Inner ? -1 : 0));
            }
            events.Sort((a, b) => a.X.CompareTo(b.X)); var outer = 0; var inner = 0; var previous = 0d;
            foreach (var e in events)
            {
                if (outer > 0 && inner == 0)
                { var start = Pixel(previous, n, half, cell); var end = Pixel(e.X, n, half, cell); if (end > start) fill(z * n + start, z * n + end); }
                outer += e.Outer; inner += e.Inner; previous = e.X;
            }
        }
    }
    private static int Pixel(double coordinate, int n, double half, double cell) => (int)Math.Max(0, Math.Min(n, Math.Ceiling((coordinate + half) / cell - .5)));

    private sealed class Context
    {
        public readonly int N; private readonly double _width; private readonly float _cell;
        private readonly float[] _height, _steep, _ridge, _valley, _rough, _exposure, _flat;
        private float[] _wet = Array.Empty<float>(), _weak = Array.Empty<float>(); private double _latitude;
        public int PermanentSeeds { get; private set; } public int WeakSeeds { get; private set; }
        public Context(int n, double width, TerrainBuilder.Result terrain, CancellationToken token)
        {
            N = n; _width = width; _cell = (float)(width / n); var size = n * n;
            _height = new float[size]; _steep = new float[size]; _ridge = new float[size]; _valley = new float[size]; _rough = new float[size]; _exposure = new float[size]; _flat = new float[size];
            var hn = (int)Math.Sqrt(terrain.Heightmap.Length);
            for (var z = 0; z < n; z++)
            { token.ThrowIfCancellationRequested(); for (var x = 0; x < n; x++) _height[z * n + x] = Height(terrain.Heightmap, hn, terrain.HeightScaleMeters, width, (x + .5) * _cell - width * .5, (z + .5) * _cell - width * .5); }
            var local = Box(_height, n, Math.Max(1, (int)Math.Round(120 / _cell)), token); var broad = Box(_height, n, Math.Max(2, (int)Math.Round(650 / _cell)), token);
            for (var z = 0; z < n; z++)
            {
                token.ThrowIfCancellationRequested();
                for (var x = 0; x < n; x++)
                {
                    var i = z * n + x; var dx = (H(x + 1, z) - H(x - 1, z)) / (_cell * 2); var dz = (H(x, z + 1) - H(x, z - 1)) / (_cell * 2); var slope = Math.Sqrt(dx * dx + dz * dz);
                    _steep[i] = Smooth((float)(slope - .12) / .95f); _ridge[i] = Smooth((_height[i] - broad[i] - 3) / 55); _valley[i] = Smooth((broad[i] - _height[i] - 3) / 45);
                    _rough[i] = Smooth((Math.Abs(_height[i] - local[i]) - 2) / 25); _exposure[i] = (float)(dz / Math.Max(.001, slope));
                    _flat[i] = Smooth((.16f - (float)slope) / .12f);
                }
            }
        }
        private float H(int x, int z) => _height[Math.Max(0, Math.Min(N - 1, z)) * N + Math.Max(0, Math.Min(N - 1, x))];
        public (float Wet, float WeakWet, float Steep, float Ridge, float Valley, float Rough, float Exposure, float Flat, float Height) At(double x, double z) =>
            (Sample(_wet, x, z), Sample(_weak, x, z), Sample(_steep, x, z), Sample(_ridge, x, z), Sample(_valley, x, z), Sample(_rough, x, z),
                Math.Abs(_latitude) < 12 ? 0 : Clamp(Sample(_exposure, x, z) * (_latitude < 0 ? -1 : 1)), Sample(_flat, x, z), Sample(_height, x, z));
        private float Sample(float[] field, double x, double z) => Bilinear(field, N, (x / _width + .5) * N - .5, (z / _width + .5) * N - .5);
        public bool Desert(IReadOnlyList<GeographicPolygon> polygons, byte[] cover, int resolution, float sea, ImportLog log, CancellationToken token, ClimateNormals? climate)
        {
            var inlandSand = new bool[N * N];
            foreach (var polygon in polygons)
            {
                if (Classify(polygon.Tags).Cover != Cover.Sand || Tag(polygon.Tags, "natural") == "beach") continue;
                Rasterize(polygon, N, _width, (start, end) => { for (var i = start; i < end; i++) inlandSand[i] = true; }, token);
            }
            var land = 0; var flat = 0; var sand = 0; var vegetated = 0; var minX = N; var maxX = -1; var minZ = N; var maxZ = -1;
            for (var z = 0; z < N; z++)
            {
                token.ThrowIfCancellationRequested();
                for (var x = 0; x < N; x++)
                {
                    var i = z * N + x; if (_height[i] <= sea) continue;
                    land++; if (_flat[i] > .5f && _rough[i] < .25f) flat++;
                    var mapped = (Cover)cover[Math.Min(resolution - 1, (int)((z + .5) * resolution / N)) * resolution + Math.Min(resolution - 1, (int)((x + .5) * resolution / N))];
                    if (mapped == Cover.Vegetation || mapped == Cover.Wetland) vegetated++;
                    if (!inlandSand[i] || mapped != Cover.Sand) continue;
                    sand++; minX = Math.Min(minX, x); maxX = Math.Max(maxX, x); minZ = Math.Min(minZ, z); maxZ = Math.Max(maxZ, z);
                }
            }
            var sandShare = sand / (double)Math.Max(1, land); var flatShare = flat / (double)Math.Max(1, land); var vegetationShare = vegetated / (double)Math.Max(1, land);
            var area = sand * (double)_cell * _cell; var span = sand == 0 ? 0 : Math.Max(maxX - minX + 1, maxZ - minZ + 1) * (double)_cell;
            // Avoid treating a small beach, isolated sand pit, or predominantly
            // mapped vegetated landscape as a regional desert. Existing hot/arid
            // normals independently permit local dry-flat inference; no new request.
            var annual=climate==null?float.NaN:ClimateMath.AnnualPrecipitation(climate.Precipitation);
            var arid=annual<150&&climate!.Temperatures.Length==12&&climate.Temperatures.All(math.isfinite)&&climate.Temperatures.Average()>15;
            // Fixed physical evidence prevents a larger map diluting the same
            // regional sand observations. Aridity opens inference but explicit
            // physical non-sand cover still wins at every individual paint site.
            var supported = arid || sand >= 6 && sandShare >= .003 && area >= Math.Min(100_000, _width * _width * .01) &&
                span >= Math.Min(_width * .08, 2000) && flatShare >= .5 && vegetationShare < .3;
            log.Diagnostic($"SURFACE DESERT supported={supported}; inlandSandAreaM2={area:0}; inlandSandShare={sandShare:0.####}; inlandSandSpanMeters={span:0}; flatLandShare={flatShare:0.####}; mappedVegetationShare={vegetationShare:0.####}; beachTriggersExpansion=false; climateInputs={climate!=null}; annualPrecipitationMm={annual:R}; aridCorroboration={arid}; mappedNonSandProtected=true");
            return supported;
        }
        public void Moisture(IReadOnlyList<GeographicPolygon> polygons, OsmDataset? geography, ImportRequest request, CancellationToken token)
        {
            _latitude = request.Latitude; var wet = new Dictionary<(int Index, float Range), float>(); var weak = new Dictionary<(int Index, float Range), float>();
            void Seed(int i, float range, float strength, bool intermittent)
            { var seeds = intermittent ? weak : wet; var key=(i,range); if (!seeds.TryGetValue(key, out var old) || strength > old) seeds[key] = strength; }
            foreach (var polygon in polygons.OrderBy(p => p.Id))
            {
                var tags = polygon.Tags; var natural = Tag(tags, "natural");
                if (natural != "water" && natural != "bay" && natural != "wetland" && !tags.ContainsKey("wetland") && !tags.ContainsKey("water") && Tag(tags, "waterway") != "riverbank" && Tag(tags, "landuse") != "reservoir") continue;
                var salt = Tag(tags, "water") == "sea" || Tag(tags, "water") == "bay" || natural == "bay" || Tag(tags, "salt") == "yes" || Tag(tags, "tidal") == "yes";
                var river = Tag(tags, "water") == "river" || tags.ContainsKey("waterway"); var intermittent = Intermittent(tags); var range = salt ? 300f : natural == "wetland" || tags.ContainsKey("wetland") ? 240f : river ? 650f : 450f;
                Rasterize(polygon, N, _width, (start, end) => { for (var i = start; i < end; i++) Seed(i, intermittent ? range * .4f : range, intermittent ? .28f : salt ? .18f : .95f, intermittent); }, token);
            }
            if (geography != null) foreach (var way in geography.Ways.Values.OrderBy(w => w.Id))
            {
                token.ThrowIfCancellationRequested(); var waterway = Tag(way.Tags, "waterway"); var coast = Tag(way.Tags, "natural") == "coastline";
                if (!coast && waterway != "river" && waterway != "stream" && waterway != "canal" && waterway != "drain" && waterway != "ditch") continue;
                var intermittent = Intermittent(way.Tags); var range = coast ? 300f : waterway == "river" ? 650f : waterway == "stream" ? 230f : waterway == "canal" ? 200f : 110f;
                for (var j = 1; j < way.NodeIds.Length; j++)
                {
                    // Do not join across missing geometry to invent drainage.
                    if (!geography.Nodes.TryGetValue(way.NodeIds[j - 1], out var a) || !geography.Nodes.TryGetValue(way.NodeIds[j], out var b)) continue;
                    Line(GeoMath.ToLocalMeters(a.Latitude, a.Longitude, request.Latitude, request.Longitude), GeoMath.ToLocalMeters(b.Latitude, b.Longitude, request.Latitude, request.Longitude),
                        // Salt water is not freshwater support for a lush strip
                        // on an otherwise arid coast. Regional desert gating is
                        // still required before any sand inference.
                        i => Seed(i, intermittent ? range * .4f : range, intermittent ? .28f : coast ? .18f : .9f, intermittent));
                }
            }
            PermanentSeeds = wet.Count; WeakSeeds = weak.Count; _wet = Influence(wet, token); _weak = Influence(weak, token);
        }
        private void Line(float3 p, float3 q, Action<int> seed)
        {
            if (!math.all(math.isfinite(p)) || !math.all(math.isfinite(q))) return;
            var half = _width * .5; var dx = (double)q.x - p.x; var dz = (double)q.z - p.z; var lo = 0d; var hi = 1d;
            bool Clip(double d, double v) { if (d == 0) return v >= 0; var t = v / d; if (d < 0) lo = Math.Max(lo, t); else hi = Math.Min(hi, t); return lo <= hi; }
            if (!Clip(-dx, p.x + half) || !Clip(dx, half - p.x) || !Clip(-dz, p.z + half) || !Clip(dz, half - p.z)) return;
            var steps = Math.Max(1, (int)Math.Ceiling(Math.Max(Math.Abs(dx), Math.Abs(dz)) * (hi - lo) / _cell * 2));
            for (var k = 0; k <= steps; k++)
            { var t = lo + (hi - lo) * k / steps; var x = Math.Max(0, Math.Min(N - 1, (int)((p.x + t * dx + half) / _cell))); var z = Math.Max(0, Math.Min(N - 1, (int)((p.z + t * dz + half) / _cell))); seed(z * N + x); }
        }
        private float[] Influence(Dictionary<(int Index, float Range), float> seeds, CancellationToken token)
        {
            var result = new float[N * N];
            if (seeds.Count == 0) return result;
            var costs = new float[result.Length];
            // Separate significance scales avoid a nearby narrow drain winning a
            // node and incorrectly blocking a broader river's farther influence.
            foreach (var group in seeds.GroupBy(p=>p.Key.Range).OrderBy(g=>g.Key))
            {
            token.ThrowIfCancellationRequested(); for(var i=0;i<costs.Length;i++)costs[i]=float.PositiveInfinity;
            var heap = new MinHeap();
            foreach (var pair in group) { costs[pair.Key.Index] = (float)-Math.Log(pair.Value); heap.Push(pair.Key.Index, costs[pair.Key.Index]); }
            var count = 0;
            while (heap.Count > 0)
            {
                if ((count++ & 4095) == 0) token.ThrowIfCancellationRequested();
                var entry = heap.Pop(); var i = entry.Index; if (entry.Cost > costs[i]) continue; var x = i % N; var z = i / N;
                for (var dz = -1; dz <= 1; dz++) for (var dx = -1; dx <= 1; dx++)
                {
                    if (dx == 0 && dz == 0 || x + dx < 0 || x + dx >= N || z + dz < 0 || z + dz >= N) continue;
                    var next = (z + dz) * N + x + dx; var distance = _cell * (dx == 0 || dz == 0 ? 1 : 1.414214f);
                    // Prefer low/concave paths; climbing hills/ridges attenuates spread.
                    // This models visual moisture, not runoff or water simulation.
                    var uphill = Math.Max(0, (_height[next] - _height[i]) / distance); var penalty = (1 + 10 * uphill + 3 * _ridge[next]) * (1 - .3f * _valley[next]);
                    var cost = entry.Cost + distance / group.Key * penalty;
                    if (cost >= 4.5f || cost >= costs[next]) continue;
                    costs[next] = cost; heap.Push(next, cost);
                }
            }
            for (var i = 0; i < costs.Length; i++) if(!float.IsPositiveInfinity(costs[i]))result[i] = Math.Max(result[i],(float)Math.Exp(-costs[i]));
            }
            return Box(result, N, 1, token);
        }
    }
    // The game's netstandard runtime has no PriorityQueue<T>.
    private sealed class MinHeap
    {
        private readonly List<(int Index, float Cost)> _items = new(); public int Count => _items.Count;
        public void Push(int index, float cost)
        { var i = _items.Count; _items.Add((index, cost)); while (i > 0) { var parent = (i - 1) / 2; if (_items[parent].Cost <= cost) break; _items[i] = _items[parent]; i = parent; } _items[i] = (index, cost); }
        public (int Index, float Cost) Pop()
        {
            var root = _items[0]; var last = _items[_items.Count - 1]; _items.RemoveAt(_items.Count - 1); if (_items.Count == 0) return root; var i = 0;
            while (i * 2 + 1 < _items.Count) { var child = i * 2 + 1; if (child + 1 < _items.Count && _items[child + 1].Cost < _items[child].Cost) child++; if (_items[child].Cost >= last.Cost) break; _items[i] = _items[child]; i = child; }
            _items[i] = last; return root;
        }
    }
    private static int BlendNarrowDesertGaps(byte[] material, byte[] intensity, byte[] cover, int n, float cell,
        double width, Context context, float sea, CancellationToken token)
    {
        // Only fill unmapped dry gaps bracketed by sand. Physical vegetation,
        // water, rock and other explicit OSM covers are never overwritten.
        var snapshot = (byte[])material.Clone(); var reach = Math.Min(32, Math.Max(1, (int)(100 / cell)));
        var filled = 0;
        for (var z = 0; z < n; z++)
        {
            token.ThrowIfCancellationRequested();
            for (var x = 0; x < n; x++)
            {
                var i = z * n + x; if (cover[i] != 0 || snapshot[i] == (byte)GroundMaterial.Sand) continue;
                bool Sand(int dx, int dz)
                {
                    for (var k = 1; k <= reach; k++)
                    {
                        var xx = x + dx * k; var zz = z + dz * k;
                        if (xx < 0 || xx >= n || zz < 0 || zz >= n) return false;
                        if (snapshot[zz * n + xx] == (byte)GroundMaterial.Sand) return true;
                    }
                    return false;
                }
                if (!(Sand(-1, 0) && Sand(1, 0) || Sand(0, -1) && Sand(0, 1))) continue;
                var wx = (x + .5) * cell - width * .5; var wz = (z + .5) * cell - width * .5;
                var p = context.At(wx, wz);
                if (p.Height <= sea || p.Wet > .2f || p.WeakWet > .25f || p.Rough > .25f || p.Flat < .2f) continue;
                material[i] = (byte)GroundMaterial.Sand;
                intensity[i] = (byte)(150 + 70 * ValueNoise(wx / 75, wz / 75, 0x712de321u)); filled++;
            }
        }
        return filled;
    }

    private static int ExpandCompatibleSand(byte[] material, byte[] intensity, byte[] confidence, byte[] cover,
        bool[] urban, int n, float cell, double width, Context context, float sea, CancellationToken token)
    {
        // Exclusive native brushes remain unchanged. Outward density approaches
        // Base; this is not a claim of a continuous Sand/Grass weight blend.
        var distance = new float[material.Length]; var heap = new MinHeap(); var sandy = 0;
        for (var i = 0; i < distance.Length; i++)
        {
            distance[i] = float.PositiveInfinity;
            if (material[i] != (byte)GroundMaterial.Sand) continue;
            sandy++; var x = i % n; var z = i / n;
            if (x > 0 && material[i - 1] != material[i] || x + 1 < n && material[i + 1] != material[i] ||
                z > 0 && material[i - n] != material[i] || z + 1 < n && material[i + n] != material[i])
            { distance[i] = 0; heap.Push(i, 0); }
        }
        var added = 0; var visits = 0; var budget = (int)Math.Min(material.Length * .01, sandy * .12);
        while (heap.Count > 0)
        {
            if ((visits++ & 4095) == 0) token.ThrowIfCancellationRequested();
            var entry = heap.Pop(); if (entry.Cost != distance[entry.Index]) continue;
            var x = entry.Index % n; var z = entry.Index / n;
            if (entry.Cost > 0)
            {
                var wx = (x + .5) * cell - width * .5; var wz = (z + .5) * cell - width * .5; var p = context.At(wx, wz);
                var band = 35 + 45 * ValueNoise(wx / 180, wz / 180, 0x1e35a7u);
                if (entry.Cost > band || p.Height <= sea || p.Wet > .35f || p.WeakWet > .35f || added >= budget) continue;
                material[entry.Index] = (byte)GroundMaterial.Sand;
                intensity[entry.Index] = (byte)Math.Round(120 * Smooth(1 - entry.Cost / (float)band));
                confidence[entry.Index] = 120; added++;
            }
            for (var dz = -1; dz <= 1; dz++) for (var dx = -1; dx <= 1; dx++)
            {
                if (dx == 0 && dz == 0 || x + dx < 0 || x + dx >= n || z + dz < 0 || z + dz >= n) continue;
                var next = (z + dz) * n + x + dx;
                if (cover[next] != (byte)Cover.Unknown || urban[next] || confidence[next] >= 225 ||
                    material[next] == (byte)GroundMaterial.Sand || material[next] == (byte)GroundMaterial.Dirt) continue;
                var cost = entry.Cost + cell * (dx == 0 || dz == 0 ? 1 : 1.414214f);
                if (cost > 80 || cost >= distance[next]) continue;
                distance[next] = cost; heap.Push(next, cost);
            }
        }
        return added;
    }

    private static (int Samples, float MinWidth, float MaxWidth, int NarrowRegions) NaturalSandTransition(
        byte[] material, byte[] intensity, float[] distance, int n, float cell, double width,
        Context context, float sea, CancellationToken token)
    {
        if (Array.IndexOf(material, (byte)GroundMaterial.Sand) < 0) return default;
        var visited = new bool[material.Length]; var queue = new int[material.Length];
        var samples = 0; var minWidth = float.PositiveInfinity; var maxWidth = 0f; var narrowRegions = 0;
        // Component depth caps the ecotone: a small beach/sand patch must retain
        // a solid sandy core instead of disappearing beneath a map-scale blur.
        // Four-neighbor membership does not join separate patches at one corner.
        for (var seed = 0; seed < material.Length; seed++)
        {
            if ((seed & 65535) == 0) token.ThrowIfCancellationRequested();
            if (visited[seed] || material[seed] != (byte)GroundMaterial.Sand) continue;
            var head = 0; var count = 1; var depth = cell * .5f; queue[0] = seed; visited[seed] = true;
            while (head < count)
            {
                if ((head & 4095) == 0) token.ThrowIfCancellationRequested();
                var i = queue[head++]; var x = i % n; var z = i / n;
                depth = Math.Max(depth, Math.Min(1440f, distance[i] + cell * .5f));
                Add(x > 0 ? i - 1 : -1); Add(x + 1 < n ? i + 1 : -1);
                Add(z > 0 ? i - n : -1); Add(z + 1 < n ? i + n : -1);
                void Add(int next)
                {
                    if (next < 0 || visited[next] || material[next] != (byte)GroundMaterial.Sand) return;
                    visited[next] = true; queue[count++] = next;
                }
            }
            var sizeLimit = Math.Max(cell * .5f, depth * .65f);
            if (sizeLimit < 70) narrowRegions++;
            for (var k = 0; k < count; k++)
            {
                if ((k & 4095) == 0) token.ThrowIfCancellationRequested();
                var i = queue[k]; var inward = distance[i] + cell * .5f;
                if (inward >= sizeLimit) continue; // Solid interiors do not receive edge noise.
                var worldX = (i % n + .5) * cell - width * .5;
                var worldZ = (i / n + .5) * cell - width * .5;
                var terrain = context.At(worldX, worldZ);
                if (terrain.Height < sea) continue; // A seabed must not feather into grass.
                var moisture = Math.Max(terrain.Wet, terrain.WeakWet * .5f);
                var broad = ValueNoise(worldX / 180, worldZ / 180, 0x51f15e5u);
                var patches = ValueNoise(worldX / 63, worldZ / 63, 0x9e3779b9u);
                var detail = ValueNoise(worldX / 19, worldZ / 19, 0x85ebca6bu);
                // Broader on moist/concave flats, narrower on dry exposed slopes.
                // World-meter, coherent noise avoids constant-width halos, stripes,
                // independent pixel speckles and changes with input polygon order.
                var desiredWidth = 4 * (90 + 100 * moisture + 35 * terrain.Valley) *
                    (1 - .4f * terrain.Steep) * (.8f + .4f * broad);
                var band = Math.Min(sizeLimit, Math.Max(cell * .5f, desiredWidth));
                if (inward >= band) continue;
                var t = Clamp(inward / band);
                var clumps = 2 * (.72f * patches + .28f * detail) - 1;
                // Perturb density only within the band. Both its outer fade and
                // its solid interior remain continuous, with no new material mix.
                t = Clamp(t + .24f * clumps * 4 * t * (1 - t));
                var density = t * t * t * (t * (t * 6 - 15) + 10);
                intensity[i] = (byte)Math.Round(intensity[i] * density);
                samples++; minWidth = Math.Min(minWidth, band); maxWidth = Math.Max(maxWidth, band);
            }
        }
        return (samples, samples == 0 ? 0 : minWidth, maxWidth, narrowRegions);
    }

    private static float ValueNoise(double x, double z, uint seed)
    {
        var x0 = (int)Math.Floor(x); var z0 = (int)Math.Floor(z);
        var fx = Smooth((float)(x - x0)); var fz = Smooth((float)(z - z0));
        float Lattice(int a, int b)
        {
            unchecked
            {
                var hash = (uint)a * 0x8da6b343u ^ (uint)b * 0xd8163841u ^ seed;
                hash ^= hash >> 16; hash *= 0x7feb352du; hash ^= hash >> 15;
                hash *= 0x846ca68bu; hash ^= hash >> 16;
                return (hash & 0x00ffffffu) / 16777215f;
            }
        }
        return (Lattice(x0, z0) * (1 - fx) + Lattice(x0 + 1, z0) * fx) * (1 - fz) +
            (Lattice(x0, z0 + 1) * (1 - fx) + Lattice(x0 + 1, z0 + 1) * fx) * fz;
    }

    private static float[] BoundaryDistance(byte[] cover, int n, float cell, CancellationToken token)
    {
        var distance = new float[cover.Length];
        for (var z = 0; z < n; z++)
        {
            token.ThrowIfCancellationRequested();
            for (var x = 0; x < n; x++)
            {
                var i = z * n + x;
                // Cropping the imported map is not evidence of a grass boundary.
                distance[i] = cover[i] == 0 || x > 0 && cover[i - 1] != cover[i] ||
                    x + 1 < n && cover[i + 1] != cover[i] || z > 0 && cover[i - n] != cover[i] ||
                    z + 1 < n && cover[i + n] != cover[i] ? 0 : float.PositiveInfinity;
            }
        }
        for (var z = 0; z < n; z++)
        {
            token.ThrowIfCancellationRequested();
            for (var x = 0; x < n; x++)
            {
                var i = z * n + x;
                if (x > 0) distance[i] = Math.Min(distance[i], distance[i - 1] + cell);
                if (z == 0) continue;
                distance[i] = Math.Min(distance[i], distance[i - n] + cell);
                if (x > 0) distance[i] = Math.Min(distance[i], distance[i - n - 1] + cell * 1.414214f);
                if (x + 1 < n) distance[i] = Math.Min(distance[i], distance[i - n + 1] + cell * 1.414214f);
            }
        }
        for (var z = n - 1; z >= 0; z--)
        {
            token.ThrowIfCancellationRequested();
            for (var x = n - 1; x >= 0; x--)
            {
                var i = z * n + x;
                if (x + 1 < n) distance[i] = Math.Min(distance[i], distance[i + 1] + cell);
                if (z + 1 == n) continue;
                distance[i] = Math.Min(distance[i], distance[i + n] + cell);
                if (x + 1 < n) distance[i] = Math.Min(distance[i], distance[i + n + 1] + cell * 1.414214f);
                if (x > 0) distance[i] = Math.Min(distance[i], distance[i + n - 1] + cell * 1.414214f);
            }
        }
        return distance;
    }
    private static float[] Box(float[] source, int n, int radius, CancellationToken token)
    {
        radius = Math.Min(n - 1, radius); var horizontal = new float[source.Length]; var output = new float[source.Length];
        for (var z = 0; z < n; z++)
        {
            token.ThrowIfCancellationRequested(); double sum = 0; var left = 0; var right = -1;
            for (var x = 0; x < n; x++) { var a = Math.Max(0, x - radius); var b = Math.Min(n - 1, x + radius); while (right < b) sum += source[z * n + ++right]; while (left < a) sum -= source[z * n + left++]; horizontal[z * n + x] = (float)(sum / (right - left + 1)); }
        }
        for (var x = 0; x < n; x++)
        {
            token.ThrowIfCancellationRequested(); double sum = 0; var bottom = 0; var top = -1;
            for (var z = 0; z < n; z++) { var a = Math.Max(0, z - radius); var b = Math.Min(n - 1, z + radius); while (top < b) sum += horizontal[++top * n + x]; while (bottom < a) sum -= horizontal[bottom++ * n + x]; output[z * n + x] = (float)(sum / (top - bottom + 1)); }
        }
        return output;
    }
    private static void SmoothExclusive(byte[] material, byte[] intensity, int n, CancellationToken token)
    {
        var result = new byte[intensity.Length];
        for (var z = 0; z < n; z++)
        {
            token.ThrowIfCancellationRequested();
            for (var x = 0; x < n; x++)
            {
                var i = z * n + x; if (material[i] == 0) continue; var sum = 0; var weight = 0;
                for (var dz = -1; dz <= 1; dz++) for (var dx = -1; dx <= 1; dx++)
                {
                    var j = Math.Max(0, Math.Min(n - 1, z + dz)) * n + Math.Max(0, Math.Min(n - 1, x + dx));
                    var w = (dx == 0 ? 2 : 1) * (dz == 0 ? 2 : 1);
                    if (material[j] == material[i]) { sum += intensity[j] * w; weight += w; }
                    // Sand already has its physical, size-aware edge fade. Do
                    // not erode a one/two-cell beach again with a pixel-size blur.
                    else if (material[i] != (byte)GroundMaterial.Sand) weight += w;
                }
                result[i] = (byte)((sum + weight / 2) / weight);
            }
        }
        Array.Copy(result, intensity, result.Length);
    }
    private static float Height(ushort[] map, int n, float scale, double width, double x, double z)
    {
        var px = Math.Max(0, Math.Min(n - 1, (x / width + .5) * (n - 1))); var pz = Math.Max(0, Math.Min(n - 1, (z / width + .5) * (n - 1)));
        var x0 = (int)px; var z0 = (int)pz; var x1 = Math.Min(n - 1, x0 + 1); var z1 = Math.Min(n - 1, z0 + 1); var fx = px - x0; var fz = pz - z0;
        return (float)((map[z0 * n + x0] * (1 - fx) + map[z0 * n + x1] * fx) * (1 - fz) + (map[z1 * n + x0] * (1 - fx) + map[z1 * n + x1] * fx) * fz) / ushort.MaxValue * scale;
    }
    private static float Bilinear(float[] map, int n, double x, double z)
    {
        x = Math.Max(0, Math.Min(n - 1, x)); z = Math.Max(0, Math.Min(n - 1, z)); var x0 = (int)x; var z0 = (int)z; var x1 = Math.Min(n - 1, x0 + 1); var z1 = Math.Min(n - 1, z0 + 1); var fx = x - x0; var fz = z - z0;
        return (float)((map[z0 * n + x0] * (1 - fx) + map[z0 * n + x1] * fx) * (1 - fz) + (map[z1 * n + x0] * (1 - fx) + map[z1 * n + x1] * fx) * fz);
    }
    private static string Tag(IReadOnlyDictionary<string, string> tags, string key) => tags.TryGetValue(key, out var value) ? value.Trim().ToLowerInvariant() : "";
    private static bool Intermittent(IReadOnlyDictionary<string, string> tags)
    {
        var seasonal = Tag(tags, "seasonal");
        return Tag(tags, "intermittent") == "yes" ||
            seasonal.Length > 0 && seasonal != "no" && seasonal != "false" && seasonal != "0";
    }
    private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    private static float Clamp(double value) => (float)Math.Max(0, Math.Min(1, value));
    private static float Smooth(float value) { value = Clamp(value); return value * value * (3 - 2 * value); }
    private static string Label(GroundMaterial material) => material == GroundMaterial.Base ? "Default" : material == GroundMaterial.Grass ? "Lush" : material == GroundMaterial.DryGrass ? "Dry" : material.ToString();
}
