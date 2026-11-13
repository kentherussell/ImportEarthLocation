using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using ImportEarthLocation.Geo;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Body levels, beds and banks have separate ownership. A source targets an
// absolute surface level; its depth is recomputed against the final native bed.
public static class WaterBodyPlanner
{
    // Same even/odd predicate as GeographyPolygons.Inside, but queries inspect
    // only edges crossing the point's horizontal band, not an entire river ring
    // for every DEM pixel. Long edges are stored once to bound index memory.
    private sealed class IndexedRing
    {
        private const float Band = 128;
        private readonly Dictionary<int, List<(float3 A, float3 B)>> _bands = new();
        private readonly List<(float3 A, float3 B)> _long = new();
        private readonly CancellationToken _token;

        public IndexedRing(float3[] ring, CancellationToken token, Action? pulse = null)
        {
            _token = token;

            for (var i = 0; i < ring.Length; i++)
            {
                token.ThrowIfCancellationRequested();

                if ((i & 1023) == 0)
                    pulse?.Invoke();

                var vectorA = ring[i];
                var vectorB = ring[(i + 1) % ring.Length];

                if (vectorA.z == vectorB.z)
                    continue;

                var low = (int)Math.Floor(Math.Min(vectorA.z, vectorB.z) / Band);
                var high = (int)Math.Floor(Math.Max(vectorA.z, vectorB.z) / Band);

                if ((long)high - low > 512)
                {
                    _long.Add((vectorA, vectorB));
                    continue;
                }

                for (var z = low; z <= high; z++)
                {
                    if (!_bands.TryGetValue(z, out var edges))
                        _bands[z] = edges = new();

                    edges.Add((vectorA, vectorB));
                }
            }
        }

        public bool Inside(float3 point)
        {
            var inside = false;
            var checks = 0;

            void Scan(List<(float3 A, float3 B)> edges)
            {
                foreach (var edge in edges)
                {
                    if ((checks++ & 1023) == 0)
                        _token.ThrowIfCancellationRequested();

                    var vectorA = edge.A;
                    var vectorB = edge.B;

                    if ((vectorA.z > point.z) != (vectorB.z > point.z)
                        && point.x
                            < (vectorB.x - vectorA.x) * (point.z - vectorA.z) / (vectorB.z - vectorA.z) + vectorA.x)
                        inside = !inside;
                }
            }

            if (_bands.TryGetValue((int)Math.Floor(point.z / Band), out var nearby))
                Scan(nearby);

            Scan(_long);

            return inside;
        }
    }

    private sealed class Area
    {
        public readonly GeographicPolygon Polygon;
        public readonly float2 Min, Max;
        public readonly int Order;
        private readonly IndexedRing[] _outer, _inner;

        public bool HasHoles => _inner.Length > 0;

        public Area(GeographicPolygon polygon, int order, CancellationToken token, Action? pulse = null)
        {
            Polygon = polygon;
            Order = order;
            Min = new float2(float.PositiveInfinity);
            Max = new float2(float.NegativeInfinity);

            foreach (var ring in polygon.Outer)
                foreach (var point in ring)
                {
                    token.ThrowIfCancellationRequested();
                    Min = math.min(Min, point.xz);
                    Max = math.max(Max, point.xz);
                }

            _outer = polygon.Outer.Select(points => new IndexedRing(points, token, pulse)).ToArray();
            _inner = polygon.Inner.Select(points => new IndexedRing(points, token, pulse)).ToArray();
        }

        public bool InBounds(float3 point) => math.all(point.xz >= Min) && math.all(point.xz <= Max);

        public bool InHole(float3 point) => InBounds(point) && _inner.Any(ring => ring.Inside(point));

        public bool Contains(float3 point)
            => InBounds(point) && _outer.Any(ring => ring.Inside(point)) && !_inner.Any(ring => ring.Inside(point));

        // Exact distance to the nearest outer or inner edge, or +∞ beyond limit.
        // Edges sit in 64 m cells, built on first use; a long river outline has
        // thousands of vertices, so a per-sample scan of every edge is too slow.
        private const float EdgeCell = 64;
        private Dictionary<(int, int), List<(float2 A, float2 B)>>? _edges;

        public float EdgeDistance(float3 point, float limit, out float2 closest)
        {
            if (_edges == null)
            {
                _edges = new();

                foreach (var ring in Polygon.Outer.Concat(Polygon.Inner))
                    for (var i = 0; i < ring.Length; i++)
                    {
                        var vectorA = ring[i].xz;
                        var vectorB = ring[(i + 1) % ring.Length].xz;
                        var low = math.floor(math.min(vectorA, vectorB) / EdgeCell);
                        var high = math.floor(math.max(vectorA, vectorB) / EdgeCell);

                        if ((high.x - low.x + 1) * (high.y - low.y + 1) > 4096)
                            continue;

                        for (var x = (int)low.x; x <= (int)high.x; x++)
                            for (var z = (int)low.y; z <= (int)high.y; z++)
                            {
                                if (!_edges.TryGetValue((x, z), out var list))
                                    _edges[(x, z)] = list = new();

                                list.Add((vectorA, vectorB));
                            }
                    }
            }

            var best = float.PositiveInfinity;
            closest = point.xz;
            var reach = (int)Math.Ceiling(limit / EdgeCell);
            var cx = (int)Math.Floor(point.x / EdgeCell);
            var cz = (int)Math.Floor(point.z / EdgeCell);

            for (var x = cx - reach; x <= cx + reach; x++)
                for (var z = cz - reach; z <= cz + reach; z++)
                    if (_edges.TryGetValue((x, z), out var list))
                        foreach (var (vectorA, vectorB) in list)
                        {
                            var ab = vectorB - vectorA;
                            var t =
                                math.clamp(math.dot(point.xz - vectorA, ab) / Math.Max(1e-9f, math.lengthsq(ab)), 0, 1);

                            var otherPoint = vectorA + ab * t;
                            var otherDistance = math.distance(point.xz, otherPoint);

                            if (otherDistance < best)
                            {
                                best = otherDistance;
                                closest = otherPoint;
                            }
                        }

            return best <= limit ? best : float.PositiveInfinity;
        }
    }

    private sealed class AreaIndex
    {
        private const float Size = 2048;
        private readonly Dictionary<(int, int), List<Area>> _cells = new();
        private readonly List<Area> _large = new();

        public AreaIndex(IEnumerable<Area> areas, CancellationToken token = default)
        {
            foreach (var area in areas)
            {
                token.ThrowIfCancellationRequested();
                var low = (int2)math.floor(area.Min / Size);
                var high = (int2)math.floor(area.Max / Size);

                if ((high.x - low.x + 1L) * (high.y - low.y + 1L) > 4096)
                {
                    _large.Add(area);
                    continue;
                }

                for (var x = low.x; x <= high.x; x++)
                    for (var z = low.y; z <= high.y; z++)
                    {
                        token.ThrowIfCancellationRequested();

                        if (!_cells.TryGetValue((x, z), out var list))
                            _cells[(x, z)] = list = new();

                        list.Add(area);
                    }
            }
        }

        public IEnumerable<Area> Nearby(float3 point)
        {
            _cells.TryGetValue(((int)Math.Floor(point.x / Size), (int)Math.Floor(point.z / Size)), out var local);
            var i = 0;
            var j = 0;
            var count = local?.Count ?? 0;

            // Preserve original polygon precedence at overlaps; never use hash
            // iteration order to choose a different body or island mask.
            while (i < count || j < _large.Count)
                if (j == _large.Count || i < count && local![i].Order < _large[j].Order)
                    yield return local![i++];
                else
                    yield return _large[j++];
        }
    }

    public static Func<float3, float?> SurfaceSampler(WaterPlan water, IReadOnlyList<GeographicPolygon> polygons)
        => SurfaceSampler(water, polygons, false);

    // Include the continuous channel cut for the native water grid as well as
    // mapped banks. Source circles alone leave dry-looking gaps between controls.
    public static Func<float3, float?> ConstructedSurfaceSampler(
        WaterPlan water,
        IReadOnlyList<GeographicPolygon> polygons)
    {
        if (water.ReachStages.Count == 0)
            return SurfaceSampler(water, polygons);

        var reaches =
            water.ReachStages
                .Where(reach => reach.PolygonId.HasValue)
                .Concat(water.ReachStages.Select(reach => reach with
                {
                    HalfWidth = Math.Max(reach.HalfWidth, water.WaterCellSize) / .6f,
                    PolygonId = null
                })).ToArray();

        return SurfaceSampler(water with { ReachStages = reaches }, polygons);
    }

    // reference=true answers with the original single-grid scan and no
    // prefilter, so tests can prove the indexed sampler gives the same answers.
    internal static Func<float3, float?> SurfaceSampler(
        WaterPlan water,
        IReadOnlyList<GeographicPolygon> polygons,
        bool reference)
    {
        var masks =
            polygons
                .Where(polygon => !ExcludedChannel(polygon.Tags))
                .Where(
                    polygon => polygon.Tags.TryGetValue("natural", out var text) && text == "water"
                        || polygon.Tags.TryGetValue("waterway", out var otherText) && otherText == "riverbank"
                        || polygon.Tags.TryGetValue("landuse", out var secondText) && secondText == "reservoir")
                .Where(polygon => polygon.Outer.Any(points => points.Length > 0))
            .Select((polygon, i) => new Area(polygon, i, default)).ToArray();

        var maskIndex = new AreaIndex(masks);
        var flat =
            water.AreaStages
                .GroupBy(stage => stage.BodyId)
                .ToDictionary(group => group.Key, group => group.Min(stage => stage.Level));

        // Reaches bound to a mapped polygon apply anywhere inside it, so they are
        // registered 1 km around their line in 2 km cells; free reaches apply only
        // within their half width and sit in 256 m cells. A point outside every
        // polygon then never scans the polygon-bound ones, and a free reach is
        // scanned only near its own line. Both lists keep ReachStages order and
        // are merged by that index, so ties resolve exactly as in one list.
        const float BoundCell = 2048, FreeCell = 256;
        var bound = new Dictionary<(int, int), List<(int Index, WaterReachStage Reach)>>();
        var free = new Dictionary<(int, int), List<(int Index, WaterReachStage Reach)>>();

        for (var i = 0; i < water.ReachStages.Count; i++)
        {
            var reach = water.ReachStages[i];
            var polygonBound = reach.PolygonId.HasValue;
            var grid = polygonBound ? bound : free;
            var size = polygonBound ? BoundCell : FreeCell;
            var margin = Math.Max(reach.HalfWidth, polygonBound ? 1024 : 1);
            var low = math.min(reach.Start.xz, reach.End.xz) - margin;
            var high = math.max(reach.Start.xz, reach.End.xz) + margin;

            for (var x = (int)Math.Floor(low.x / size); x <= (int)Math.Floor(high.x / size); x++)
                for (var z = (int)Math.Floor(low.y / size); z <= (int)Math.Floor(high.y / size); z++)
                {
                    if (!grid.TryGetValue((x, z), out var list))
                        grid[(x, z)] = list = new();

                    list.Add((i, reach));
                }
        }

        var none = new List<(int Index, WaterReachStage Reach)>();

        if (reference)
        {
            var reaches = new Dictionary<(int, int), List<WaterReachStage>>();

            foreach (var reach in water.ReachStages)
            {
                var margin = Math.Max(reach.HalfWidth, reach.PolygonId.HasValue ? 1024 : 1);
                var low = math.min(reach.Start.xz, reach.End.xz) - margin;
                var high = math.max(reach.Start.xz, reach.End.xz) + margin;

                for (var x = (int)Math.Floor(low.x / 2048); x <= (int)Math.Floor(high.x / 2048); x++)
                    for (var z = (int)Math.Floor(low.y / 2048); z <= (int)Math.Floor(high.y / 2048); z++)
                    {
                        if (!reaches.TryGetValue((x, z), out var list))
                            reaches[(x, z)] = list = new();

                        list.Add(reach);
                    }
            }

            return point =>
            {
                var area = maskIndex.Nearby(point).FirstOrDefault(otherArea => otherArea.Contains(point));

                if (area != null && flat.TryGetValue(area.Polygon.Identity, out var level))
                    return level;

                float? result = null;
                var best = float.PositiveInfinity;

                if (reaches.TryGetValue(
                    ((int)Math.Floor(point.x / 2048), (int)Math.Floor(point.z / 2048)),
                    out var list))
                    foreach (var reach in list)
                    {
                        if (reach.PolygonId.HasValue
                            ? area?.Polygon.Identity != reach.PolygonId.Value
                            : maskIndex.Nearby(point).Any(otherArea => otherArea.InHole(point)))
                            continue;

                        var delta = reach.End.xz - reach.Start.xz;
                        var t =
                            math.clamp(
                                math.dot(point.xz - reach.Start.xz, delta) / Math.Max(.0001f, math.lengthsq(delta)),
                                0,
                                1);

                        var distance = math.distance(point.xz, math.lerp(reach.Start.xz, reach.End.xz, t));

                        if (!reach.PolygonId.HasValue && distance > reach.HalfWidth || distance >= best)
                            continue;

                        best = distance;
                        result = math.lerp(reach.StartLevel, reach.EndLevel, t);
                    }

                return result;
            };
        }

        // Most queries are far from any water. The game's runtime makes the
        // per-query allocations below expensive (DC: 47.7 s in game against
        // 0.7 s offline for the same 207,000 queries), so a 128 m grid of cells
        // that can hold water answers "none" first. It is conservative: every
        // cell within one cell of a mask edge, inside a mask (by scanline at cell
        // centres), or within a free reach's half width is marked, so a point the
        // full test would find is never cut off.
        const float Cell = 128;
        var occupied = new HashSet<long>();

        long Key(int x, int z) => ((long)x << 32) ^ (uint)z;

        void Mark(float2 vectorA, float2 vectorB, float margin)
        {
            var reach = (int)Math.Ceiling(margin / Cell) + 1;
            var steps = Math.Max(1, (int)Math.Ceiling(math.distance(vectorA, vectorB) / (Cell / 2)));

            for (var k = 0; k <= steps; k++)
            {
                var point = math.lerp(vectorA, vectorB, k / (float)steps);
                var cx = (int)Math.Floor(point.x / Cell);
                var cz = (int)Math.Floor(point.y / Cell);

                for (var dz = -reach; dz <= reach; dz++)
                    for (var dx = -reach; dx <= reach; dx++)
                        occupied.Add(Key(cx + dx, cz + dz));
            }
        }

        foreach (var area in masks)
            foreach (var ring in area.Polygon.Outer)
            {
                for (var i = 0; i < ring.Length; i++)
                    Mark(ring[i].xz, ring[(i + 1) % ring.Length].xz, 0);

                var crossings = new List<float>();

                for (var cz = (int)Math.Floor(area.Min.y / Cell); cz <= (int)Math.Floor(area.Max.y / Cell); cz++)
                {
                    var z = (cz + .5f) * Cell;
                    crossings.Clear();

                    for (var i = 0; i < ring.Length; i++)
                    {
                        var vectorA = ring[i];
                        var vectorB = ring[(i + 1) % ring.Length];

                        if ((vectorA.z > z) != (vectorB.z > z))
                            crossings.Add(
                                vectorA.x + (z - vectorA.z) * (vectorB.x - vectorA.x) / (vectorB.z - vectorA.z));
                    }

                    crossings.Sort();

                    for (var i = 1; i < crossings.Count; i += 2)
                        for (var cx =
                            (int)Math.Floor(crossings[i - 1] / Cell); cx <= (int)Math.Floor(crossings[i] / Cell); cx++)
                            occupied.Add(Key(cx, cz));
                }
            }

        foreach (var reach in water.ReachStages)
            if (!reach.PolygonId.HasValue)
                Mark(reach.Start.xz, reach.End.xz, reach.HalfWidth);

        return point =>
        {
            if (!occupied.Contains(Key((int)Math.Floor(point.x / Cell), (int)Math.Floor(point.z / Cell))))
                return null;

            Area? area = null;

            foreach (var firstArea in maskIndex.Nearby(point))
                if (firstArea.Contains(point))
                {
                    area = firstArea;
                    break;
                }

            if (area != null && flat.TryGetValue(area.Polygon.Identity, out var level))
                return level;

            float? result = null;
            var best = float.PositiveInfinity;
            var polygonReaches =
                area != null
                    && bound.TryGetValue(
                        ((int)Math.Floor(point.x / BoundCell), (int)Math.Floor(point.z / BoundCell)),
                        out var secondItems)
                    ? secondItems
                    : none;

            // Whether p is in an island hole depends only on p: tested once, not per reach.
            if (!free.TryGetValue(
                ((int)Math.Floor(point.x / FreeCell), (int)Math.Floor(point.z / FreeCell)),
                out var lineReaches))
                lineReaches = none;
            else
                foreach (var firstArea in maskIndex.Nearby(point))
                    if (firstArea.InHole(point))
                    {
                        lineReaches = none;
                        break;
                    }

            for (int i = 0, j = 0; i < polygonReaches.Count || j < lineReaches.Count;)
            {
                WaterReachStage reach;

                if (j == lineReaches.Count
                    || i < polygonReaches.Count && polygonReaches[i].Index < lineReaches[j].Index)
                {
                    reach = polygonReaches[i++].Reach;

                    if (area!.Polygon.Identity != reach.PolygonId!.Value)
                        continue;
                }
                else
                    reach = lineReaches[j++].Reach;

                var delta = reach.End.xz - reach.Start.xz;
                var t =
                    math.clamp(
                        math.dot(point.xz - reach.Start.xz, delta) / Math.Max(.0001f, math.lengthsq(delta)),
                        0,
                        1);

                var distance = math.distance(point.xz, math.lerp(reach.Start.xz, reach.End.xz, t));

                if (!reach.PolygonId.HasValue && distance > reach.HalfWidth || distance >= best)
                    continue;

                best = distance;
                result = math.lerp(reach.StartLevel, reach.EndLevel, t);
            }

            return result;
        };
    }

    // The import represents mapped buried rivers as surface channels. Crossings
    // must span the constructed cut, including its feathered banks, rather than
    // the often much narrower mapped waterway width.
    private static readonly string[] Waterways = { "river", "drain", "ditch" };

    internal static bool ExcludedChannel(IReadOnlyDictionary<string, string> tags) =>
        tags.TryGetValue("waterway", out var way) && way is "canal" or "stream"
            || tags.TryGetValue("water", out var water) && water is "canal" or "stream";

    // A dry, abandoned channel keeps its waterway tag in OSM; Marseille's
    // abandoned branches of its canal were carved as trenches across hillsides.
    private static bool Abandoned(IReadOnlyDictionary<string, string> tags) =>
        tags.TryGetValue("abandoned", out var text) && text == "yes"
            || tags.TryGetValue("disused", out var otherText) && otherText == "yes";

    private static bool Buried(IReadOnlyDictionary<string, string> tags)
        => tags.TryGetValue("tunnel", out var tunnel) && tunnel != "no";

    private static float ChannelExtent(IReadOnlyDictionary<string, string> tags, float cell)
    {
        var fallback = tags["waterway"] == "river" ? 32 : 12;
        var width = NetworkEvidence.Measure(tags, "width") ?? fallback;

        if (!math.isfinite(width) || width <= 0 || width > 2048)
            width = fallback;

        return Math.Max(width * .5f, cell) / .6f;
    }

    // Repair only small breaks supported by both identity and downstream
    // direction. Never jump across a mapped canal, intermittent reach or tunnel.
    // The input dataset is shared with other planners and remains immutable.
    private static OsmDataset ConnectRiverEndpoints(
        OsmDataset data,
        ImportRequest request,
        ImportLog? log = null,
        CancellationToken token = default)
    {
        const float maximumGap = 28, alignment = .8660254f;
        var ways =
            data.Ways.Values
                .Where(
                    way => way.Tags.TryGetValue("waterway", out var kind)
                        && Waterways.Contains(kind)
                        && !ExcludedChannel(way.Tags)
                        && !Abandoned(way.Tags)
                        && !(way.Tags.TryGetValue("intermittent", out var dry) && dry == "yes")
                && way.NodeIds.Length > 1
                && way.NodeIds.All(data.Nodes.ContainsKey))
                .OrderBy(way => way.Id)
                .ToArray();

        var incoming = new HashSet<long>();
        var outgoing = new HashSet<long>();

        // Excluded evidence is still a real connection: do not invent a bypass.
        foreach (var way in data.Ways.Values.Where(otherWay => otherWay.Tags.ContainsKey("waterway")))
            for (var i = 1; i < way.NodeIds.Length; i++)
            {
                outgoing.Add(way.NodeIds[i - 1]);
                incoming.Add(way.NodeIds[i]);
            }

        float3 Local(long id)
        {
            var node = data.Nodes[id];
            return GeoMath.ToLocalMeters(node.Latitude, node.Longitude, request.Latitude, request.Longitude);
        }

        var heads = new Dictionary<(int, int), List<OsmWay>>();

        (int, int) Key(float3 point) => ((int)Math.Floor(point.x / maximumGap), (int)Math.Floor(point.z / maximumGap));

        foreach (var way in ways)
        {
            token.ThrowIfCancellationRequested();

            if (incoming.Contains(way.NodeIds[0]))
                continue;

            var key = Key(Local(way.NodeIds[0]));

            if (!heads.TryGetValue(key, out var list))
                heads[key] = list = new();

            list.Add(way);
        }

        var matches = new List<(OsmWay From, OsmWay To)>();

        foreach (var way in ways)
        {
            token.ThrowIfCancellationRequested();
            var end = way.NodeIds[way.NodeIds.Length - 1];

            if (outgoing.Contains(end))
                continue;

            var vectorA = Local(end);
            var direction = math.normalizesafe((vectorA - Local(way.NodeIds[way.NodeIds.Length - 2])).xz);
            var (cx, cz) = Key(vectorA);
            var candidates = new List<OsmWay>();

            for (var dx = -1; dx <= 1; dx++)
                for (var dz = -1; dz <= 1; dz++)
                    if (heads.TryGetValue((cx + dx, cz + dz), out var list))
                        foreach (var other in list)
                        {
                            if (other.Id == way.Id || way.NodeIds.Contains(other.NodeIds[0]))
                                continue;

                            var vectorB = Local(other.NodeIds[0]);
                            var distance = math.distance(vectorA.xz, vectorB.xz);

                            if (distance > maximumGap)
                                continue;

                            if (distance > 2
                                && (!way.Tags.TryGetValue("name", out var name) || string.IsNullOrWhiteSpace(name)
                                    || !other.Tags.TryGetValue("name", out var otherName)
                                    || !string.Equals(name, otherName, StringComparison.OrdinalIgnoreCase)))
                                continue;

                            if (way.Tags.TryGetValue("name", out var left)
                                && other.Tags.TryGetValue("name", out var right)
                                && !string.Equals(left, right, StringComparison.OrdinalIgnoreCase))
                                continue;

                            var next = math.normalizesafe((Local(other.NodeIds[1]) - vectorB).xz);

                            if (math.dot(direction, next) < alignment)
                                continue;

                            if (distance > .1f
                                && (math.dot(direction, (vectorB - vectorA).xz / distance) < alignment
                                    || math.dot(next, (vectorB - vectorA).xz / distance) < alignment))
                                continue;

                            candidates.Add(other);
                        }

            if (candidates.Count == 1)
                matches.Add((way, candidates[0]));
        }

        var unique =
            matches
                .GroupBy(item => item.To.Id)
                .Where(group => group.Count() == 1)
                .Select(group => group.Single())
                .ToArray();

        if (unique.Length == 0)
            return data;

        var revised = data.Ways.ToDictionary(entry => entry.Key, entry => entry.Value);
        var parents = ways.ToDictionary(way => way.Id, way => way.Id);
        var nodeOwners = new Dictionary<long, long>();

        long Root(long id)
        {
            while (parents[id] != id)
                id = parents[id] = parents[parents[id]];

            return id;
        }

        foreach (var way in ways)
            foreach (var id in way.NodeIds)
                if (nodeOwners.TryGetValue(id, out var other))
                    parents[Root(way.Id)] = Root(other);
                else
                    nodeOwners[id] = way.Id;

        var connected = 0;

        foreach (var (upstream, downstream) in unique)
        {
            token.ThrowIfCancellationRequested();
            var root = Root(upstream.Id);
            var otherRoot = Root(downstream.Id);

            if (root == otherRoot)
                continue;

            parents[root] = otherRoot;
            revised[upstream.Id] =
                upstream with { NodeIds = upstream.NodeIds.Concat(new[] { downstream.NodeIds[0] }).ToArray() };

            connected++;
            log?.Diagnostic(
                $"WATER ENDPOINT CONNECTION upstreamWay={upstream.Id}; downstreamWay={downstream.Id}; gapMeters={math.distance(Local(upstream.NodeIds[upstream.NodeIds.Length - 1]).xz, Local(downstream.NodeIds[0]).xz):R}; maximumGapMeters={maximumGap}; identityAndDirectionChecked=true; originalEvidenceUnchanged=true");
        }

        log?.Count("Water endpoint gaps connected", connected);

        return new OsmDataset
        {
            Nodes = data.Nodes,
            NodeTags = data.NodeTags,
            Ways = revised,
            Relations = data.Relations,
            SupportWayIds = data.SupportWayIds,
            RawWayTags = data.RawWayTags,
            MalformedFeaturesSkipped = data.MalformedFeaturesSkipped
        };
    }

    // One sample along a waterway's line: its place on the clipped edge A–B
    // (and the next sample on that edge, if any), its distance along the way,
    // the node it stands on (0 between nodes),
    // its dry bank probes with the lower bank's height, and the mapped
    // river polygon it lies in.
    private readonly record struct ChannelSample(float3 P, float3 A, float3 B, float3? Next, float Station, long Node,
        float Bank,
            IReadOnlyList<float3> Banks,
            Area? Area,
            float Reach,
            float Channel,
            float SourceWidth,
            bool Bankless);

    // The waterways that make up the river network: rivers,
    // and every ditch or drain connected to one through shared nodes, whatever
    // its width, so a river carried for a stretch by a narrow drain stays one
    // river. A ditch network touching none (field drainage) is left out unless
    // mapped at least a water cell (28 m) wide: alone it never holds a source
    // and was carved as a row of dry pits. Abandoned, disused and intermittent
    // waterways and canals are excluded. Buried reaches retain connectivity.
    // Long covered rivers stay covered; digging them open forced bridges on every road above.
    private const double MaximumOpenedBuried = 250;

    private static double Metres(OsmDataset data, OsmWay way)
    {
        var total = 0d;

        for (var i = 1; i < way.NodeIds.Length; i++)
        {
            var firstNode = data.Nodes[way.NodeIds[i - 1]];
            var secondNode = data.Nodes[way.NodeIds[i]];
            var dz = (secondNode.Latitude - firstNode.Latitude) * 111320;
            var dx =
                (secondNode.Longitude - firstNode.Longitude) * 111320 * Math.Cos(firstNode.Latitude * Math.PI / 180);

            total += Math.Sqrt(dx * dx + dz * dz);
        }

        return total;
    }

    private static HashSet<long> RiverNetwork(OsmDataset data)
    {
        var candidates =
            data.Ways.Values
                .Where(
                    way => way.Tags.TryGetValue("waterway", out var type)
                        && Waterways.Contains(type)
                        && !ExcludedChannel(way.Tags)
                        && !Abandoned(way.Tags)
                        && !(way.Tags.TryGetValue("intermittent", out var intermittent) && intermittent == "yes")
                && way.NodeIds.Length > 1
                && way.NodeIds.All(data.Nodes.ContainsKey)
                && !(Buried(way.Tags) && Metres(data, way) > MaximumOpenedBuried))
                .ToArray();

        var parent = candidates.ToDictionary(way => way.Id, way => way.Id);

        long Root(long id)
        {
            while (parent[id] != id)
                id = parent[id] = parent[parent[id]];

            return id;
        }

        var owner = new Dictionary<long, long>();

        foreach (var way in candidates)
            foreach (var node in way.NodeIds)
                if (owner.TryGetValue(node, out var other))
                {
                    var root = Root(way.Id);
                    var otherRoot = Root(other);

                    if (root != otherRoot)
                        parent[root] = otherRoot;
                }
                else
                    owner[node] = way.Id;

        var main =
            new HashSet<long>(candidates.Where(way => way.Tags["waterway"] == "river").Select(way => Root(way.Id)));

        return new HashSet<long>(
            candidates
                .Where(way => main.Contains(Root(way.Id)) || NetworkEvidence.Measure(way.Tags, "width") >= 28)
                .Select(way => way.Id));
    }

    // Mapped buried routes up to MaximumOpenedBuried opened at the surface.
    // HalfWidth is the constructed cut's outer radius, in local map metres.
    public static IReadOnlyList<(float3[] Line, float HalfWidth)> BuriedChannels(
        OsmDataset? data,
        ImportRequest request,
        float waterCellSize = 28)
    {
        if (data == null || !math.isfinite(waterCellSize) || waterCellSize <= 0)
            return Array.Empty<(float3[], float)>();

        data = ConnectRiverEndpoints(data, request);
        var result = new List<(float3[], float)>();
        var network = RiverNetwork(data);

        foreach (var way in data.Ways.Values.OrderBy(otherWay => otherWay.Id))
        {
            if (!network.Contains(way.Id) || !Buried(way.Tags))
                continue;

            var line = way.NodeIds.Select(id =>
{
    var node = data.Nodes[id];
    return GeoMath.ToLocalMeters(node.Latitude, node.Longitude, request.Latitude, request.Longitude);
}).ToArray();

            result.Add((line, ChannelExtent(way.Tags, waterCellSize)));
        }

        return result;
    }

    // Open waterways of the river network as local polylines with their channel
    // cut half width, as for BuriedChannels. A ground road crossing one without a
    // bridge or culvert tag is an omission in the map, and its embankment would
    // dam the carved channel; NetworkStructureIntervals.CulvertBridges spans it.
    public static IReadOnlyList<(float3[] Line, float HalfWidth)> OpenChannels(
        OsmDataset? data,
        ImportRequest request,
        float waterCellSize = 28)
    {
        if (data == null || !math.isfinite(waterCellSize) || waterCellSize <= 0)
            return Array.Empty<(float3[], float)>();

        data = ConnectRiverEndpoints(data, request);
        var result = new List<(float3[], float)>();
        var network = RiverNetwork(data);

        foreach (var way in data.Ways.Values.OrderBy(otherWay => otherWay.Id))
        {
            if (!network.Contains(way.Id) || Buried(way.Tags))
                continue;

            var line = way.NodeIds.Select(id =>
{
    var node = data.Nodes[id];
    return GeoMath.ToLocalMeters(node.Latitude, node.Longitude, request.Latitude, request.Longitude);
}).ToArray();

            result.Add((line, ChannelExtent(way.Tags, waterCellSize)));
        }

        return result;
    }

    public static WaterPlan Prepare(OsmDataset? data, IReadOnlyList<GeographicPolygon> polygons, ImportRequest request,
        TerrainBuilder.Result terrain,
            ImportLog log,
            CancellationToken token,
            float cell,
            Action<string>? progress = null)
    {
        if (data == null || !math.isfinite(cell) || cell <= 0)
            return new WaterPlan(
                false,
                0,
                0,
                0,
                "Water evidence or native grid unavailable; water preparation cannot be verified.");

        data = ConnectRiverEndpoints(data, request, log, token);
        var width = (float)request.MapSize.WidthMeters;
        var half = width * .5f;
        var n = (int)Math.Sqrt(terrain.Heights.Length);
        var spacing = width / (n - 1);
        var conditioned = (float[])terrain.Heights.Clone();
        var editMask = new byte[conditioned.Length];
        var editCount = 0;
        var sources = new List<RiverSourcePlan>();
        var areaStages = new List<WaterAreaStage>();
        var reachStages = new List<WaterReachStage>();
        var riverSurface = new float[conditioned.Length];
        var unresolved = 0;
        var riverCount = 0;
        var lakeCount = 0;
        var clock = Stopwatch.StartNew();
        var lastPulse = -5d;
        var phase = "indexing mapped water";
        long activeBody = 0;
        long bedVisits = 0;
        long reachSamples = 0;
        var totalWays = 0;

        void Pulse(bool force = false)
        {
            token.ThrowIfCancellationRequested();

            if (!force && clock.Elapsed.TotalSeconds - lastPulse < 5)
                return;

            lastPulse = clock.Elapsed.TotalSeconds;
            progress?.Invoke($"Planning water: {phase}; {riverCount}/{totalWays} waterways; {sources.Count} controls");
            log.Diagnostic(
                $"WATER PLAN PROGRESS phase={phase}; body={activeBody}; lakes={lakeCount}; waterways={riverCount}/{totalWays}; reachSamples={reachSamples}; bedCellsVisited={bedVisits}; bedEdits={editCount}; controls={sources.Count}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.###}; cancellationChecked=true");
        }

        Pulse(true);

        bool Inside(float3 point) => Math.Abs(point.x) <= half - cell * 2 && Math.Abs(point.z) <= half - cell * 2;

        int Pixel(float3 point)
            => (int)Math.Round(math.clamp(point.z / width + .5f, 0, 1) * (n - 1)) * n
                + (int)Math.Round(math.clamp(point.x / width + .5f, 0, 1) * (n - 1));

        float Height(float3 point)
            => NetworkProcessor.Sample(point.x, point.z, conditioned, width);

        float BankHeight(float3 point)
            => NetworkProcessor.Sample(point.x, point.z, terrain.Heights, width);

        float3 Local(long id)
        {
            var node = data.Nodes[id];
            return GeoMath.ToLocalMeters(node.Latitude, node.Longitude, request.Latitude, request.Longitude);
        }

        var areas =
            polygons
                .Where(polygon => !ExcludedChannel(polygon.Tags))
                .Where(
                    polygon => NetworkSelection.HasTag(polygon.Tags, "natural", "water")
                        || NetworkSelection.HasTag(polygon.Tags, "waterway", "riverbank")
                        || NetworkSelection.HasTag(polygon.Tags, "landuse", "reservoir"))
            .Where(polygon => polygon.Outer.Any(points => points.Length > 0)).Select((polygon, i) =>
{
    Pulse();
    return new Area(polygon, i, token, () => Pulse());
}).ToArray();

        var areaIndex = new AreaIndex(areas, token);
        var holeIndex = new AreaIndex(areas.Where(area => area.HasHoles), token);

        bool Coastal(Area firstArea) => WaterLevels.SeaTagged(firstArea.Polygon.Tags);

        bool Flowing(Area firstArea)
            => NetworkSelection.HasTag(firstArea.Polygon.Tags, "water", "river")
                || NetworkSelection.HasTag(firstArea.Polygon.Tags, "waterway", "riverbank");

        bool Dry(Area firstArea)
            => NetworkSelection.HasTag(firstArea.Polygon.Tags, "intermittent", "yes")
                || firstArea.Polygon.Tags.TryGetValue("seasonal", out var season)
                    && season != "no"
                    && season != "false"
                    && season != "0";

        bool Lake(Area firstArea) => !Flowing(firstArea) && !Coastal(firstArea) && !Dry(firstArea);

        bool MappedDry(float3 point) => holeIndex.Nearby(point).Any(area => area.InHole(point));

        bool DryFootprint(float3 point, Func<float3, bool> wet)
        {
            if (!Inside(point) || wet(point))
                return false;

            var margin = Math.Max(cell, spacing) * 1.5f;

            return !wet(point + new float3(margin, 0, margin)) && !wet(point + new float3(-margin, 0, margin))
                && !wet(point + new float3(margin, 0, -margin)) && !wet(point + new float3(-margin, 0, -margin));
        }

        float3? DryProbe(float3 point, float3 direction, Func<float3, bool> wet, float start)
        {
            for (var distance =
                Math.Max(start, Math.Max(cell, spacing) * 2); distance <= 1024; distance += Math.Max(cell, spacing))
            {
                var otherPoint = point + direction * distance;

                if (DryFootprint(otherPoint, wet))
                    return otherPoint;
            }

            return null;
        }

        // Cells on the line of a carved waterway channel. Shore shaping of mapped
        // water areas, which runs later, must not raise them back into a dam.
        var channelCells = new byte[n * n];

        void Cut(float3 center, float radius, float target, Func<float3, bool> mask, float budget, bool channel = false)
        {
            var pixel = Pixel(center);
            var x0 = pixel % n;
            var z0 = pixel / n;
            var reach = (int)Math.Ceiling(radius / spacing);

            for (var z = Math.Max(0, z0 - reach); z <= Math.Min(n - 1, z0 + reach); z++)
                for (var x = Math.Max(0, x0 - reach); x <= Math.Min(n - 1, x0 + reach); x++)
                {
                    if ((bedVisits++ & 4095) == 0)
                        Pulse();

                    var point = new float3(x * spacing - half, 0, z * spacing - half);
                    var distance = math.distance(point.xz, center.xz);

                    if (distance > radius)
                        continue;

                    var index = z * n + x;
                    var original = terrain.Heights[index];
                    var blend = math.clamp((radius - distance) / (radius * .4f), 0, 1);
                    blend = blend * blend * (3 - 2 * blend);

                    if (channel && blend > .5f && channelCells[index] == 0 && mask(point))
                        channelCells[index] = 1;

                    var height = Math.Max(0, original - Math.Min(budget, Math.Max(0, original - target)) * blend);
                    var bed = Math.Min(height, terrain.HeightScaleMeters);

                    // Overlapping cuts frequently propose the same/larger height.
                    // Preserve the exact min-cut result while avoiding another
                    // polygon query for a raster cell that cannot change.
                    if (bed >= conditioned[index])
                        continue;

                    if (mask(point))
                    {
                        conditioned[index] = bed;

                        if (editMask[index] == 0)
                        {
                            editMask[index] = 1;
                            editCount++;
                        }
                    }
                }
        }

        // A waterway's cross-section (report 116): a flat bed out to bedRadius; a lower bank
        // rising smoothly from the bed to a shelf (RiverFreeboard above the water), ending at
        // 1/0.6 of bedRadius, where the earlier cut ended; then an upper bank rising
        // smoothly from the shelf back to the ground at about BankSlope, at most
        // MaximumBankWidth wide. The earlier cut rose from the bed straight to the ground
        // over 40% of its radius: 6–20 m in under 20 m, which the game drew as rock cliffs.
        // Ground is only ever lowered, by at most budget. Ground below the shelf stays as it
        // is, and ground beyond the lower bank never drops below the shelf, so the bank
        // probes just outside it still stand above the water (they stage it and gate its
        // sources).
        const float BankSlope = .3f, MaximumBankWidth = 80;

        void CutChannel(
            float3 center,
            float bedRadius,
            float bedTarget,
            float shelf,
            float upperBankHeight,
            Func<float3, bool> mask,
            float budget)
        {
            var lowerEdge = bedRadius / .6f;
            var bankWidth = math.clamp((upperBankHeight - shelf) / BankSlope, cell, MaximumBankWidth);
            var outer = lowerEdge + bankWidth;
            var pixel = Pixel(center);
            var x0 = pixel % n;
            var z0 = pixel / n;
            var reach = (int)Math.Ceiling(outer / spacing);

            static float Smooth(float t)
            {
                t = math.saturate(t);
                return t * t * (3 - 2 * t);
            }

            for (var z = Math.Max(0, z0 - reach); z <= Math.Min(n - 1, z0 + reach); z++)
                for (var x = Math.Max(0, x0 - reach); x <= Math.Min(n - 1, x0 + reach); x++)
                {
                    if ((bedVisits++ & 4095) == 0)
                        Pulse();

                    var point = new float3(x * spacing - half, 0, z * spacing - half);
                    var distance = math.distance(point.xz, center.xz);

                    if (distance > outer)
                        continue;

                    var index = z * n + x;
                    var original = terrain.Heights[index];
                    var floor = Math.Min(original, shelf);

                    var height =
                        distance <= bedRadius
                            ? bedTarget
                            : distance <= lowerEdge
                                ? math.lerp(bedTarget, floor, Smooth((distance - bedRadius) / (lowerEdge - bedRadius)))
                                : math.lerp(floor, original, Smooth((distance - lowerEdge) / bankWidth));

                    height = math.clamp(Math.Max(height, original - budget), 0, Math.Min(original, terrain.HeightScaleMeters));

                    if (distance <= bedRadius + (lowerEdge - bedRadius) * .5f
                        && channelCells[index] == 0
                        && mask(point))
                        channelCells[index] = 1;

                    if (height >= conditioned[index] || !mask(point))
                        continue;

                    conditioned[index] = height;

                    if (editMask[index] == 0)
                    {
                        editMask[index] = 1;
                        editCount++;
                    }
                }
        }

        // Shore band: near the mapped edge, heights follow the exact distance to
        // it rather than which grid points fall inside. The bed deepens from the
        // stage at the edge to full depth, and the bank rises from the stage at
        // the edge back to the original land, each along a smoothstep over two
        // grid spacings, so slope is continuous through the edge and into the
        // untouched terrain. Land beyond the band is untouched; bank land is
        // lowered by at most bankCut and never below the levee, so it still
        // contains the water, and low bank land is raised to the levee.
        // Within one grid spacing of the edge both sides follow the same slope
        // EdgeSlope: the ground is stage + EdgeSlope·s for the signed distance
        // s, a plane along a straight shore, which the grid's bilinear
        // interpolation reproduces exactly in every cell the edge crosses, so
        // the waterline lies on the mapped edge whatever its angle to the grid.
        // A step at the edge (a lip 0.5 m up on the bank and 0.5 m down in the
        // bed) put each grid point on one side of the step or the other and
        // drew a slanted shore as teeth (East Potomac Park). The slope still
        // rises 0.8 m in half a 28 m water cell, so a dry cell next to the
        // shore stands clearly above the stage; flat profiles (zero slope at
        // the edge) let the water wet whole cells into a tooth per cell. Beyond
        // that spacing the levee and the bed follow a cubic to LeveeFreeboard
        // above, and the full depth below, the stage at the band's end, flat
        // there. Low bank land raised to the levee eases back down to its own
        // height (at least ShoreRim above the stage) across the band.
        var bedBand = 2 * spacing;
        var shoreBand = 2 * spacing;
        const float bankCut = 3;
        var shapedBank = 0;
        var raisedBank = 0;
        const float LeveeFreeboard = 1.2f, EdgeSlope = .06f, ShoreRim = .3f;
        // Shore-profile bed height of each wet pixel within the bed band (NaN elsewhere).
        var shoreFloor = new float[n * n];
        Array.Fill(shoreFloor, float.NaN);

        static float Ramp(float x)
        {
            x = Math.Min(1, x);
            return x * x * (3 - 2 * x);
        }

        // EdgeSlope·e up to one spacing, then a cubic Hermite to height at the
        // band's end with slope 0, kept monotonic by limiting its start slope.
        float Profile(float e, float height, float band)
        {
            if (e <= spacing)
                return EdgeSlope * e;

            var t = Math.Min(1, (e - spacing) / (band - spacing));
            var h0 = EdgeSlope * spacing;
            var m = Math.Min(EdgeSlope * (band - spacing), 3 * Math.Max(0, height - h0));

            return h0 * (2 * t * t * t - 3 * t * t + 1)
                + m * (t * t * t - 2 * t * t + t)
                + height * (3 * t * t - 2 * t * t * t);
        }

        void CarveArea(Area area, Func<float3, float?> level)
        {
            var accepted = 0;
            var blockedByBudget = 0;
            var pad = (int)Math.Ceiling(shoreBand / spacing);
            var loX = Math.Max(0, (int)Math.Ceiling((area.Min.x + half) / spacing) - pad);
            var hiX = Math.Min(n - 1, (int)Math.Floor((area.Max.x + half) / spacing) + pad);
            var loZ = Math.Max(0, (int)Math.Ceiling((area.Min.y + half) / spacing) - pad);
            var hiZ = Math.Min(n - 1, (int)Math.Floor((area.Max.y + half) / spacing) + pad);

            for (var z = loZ; z <= hiZ; z++)
                for (var x = loX; x <= hiX; x++)
                {
                    if ((bedVisits++ & 4095) == 0)
                        Pulse();

                    var point = new float3(x * spacing - half, 0, z * spacing - half);
                    var edge = area.EdgeDistance(point, shoreBand, out var closest);

                    if (!area.Contains(point) || MappedDry(point))
                    {
                        if (float.IsPositiveInfinity(edge) || channelCells[z * n + x] != 0)
                            continue;

                        // The bank meets the stage of the water just across the edge.
                        var inward = math.normalizesafe(closest - point.xz) * .5f;
                        var shore = level(new float3(closest.x + inward.x, 0, closest.y + inward.y));

                        if (!shore.HasValue || !math.isfinite(shore.Value))
                            continue;

                        // Low banks: source noise leaves pixels of a low shore (East
                        // Potomac Park) under the stage, and water filled them one by
                        // one into a spiky edge. Bank land up to bankCut below the
                        // stage rises to the lip, unless it lies in other mapped
                        // water, where the levee would dam that body. Higher bank land
                        // is lowered toward the stage but never below the lip.
                        var levee = shore.Value + Profile(edge, LeveeFreeboard, shoreBand);
                        var bankIndex = z * n + x;
                        var land = terrain.Heights[bankIndex];
                        var bank = Math.Max(levee, shore.Value + (land - shore.Value) * Ramp(edge / shoreBand));

                        if (land < levee
                            && land > shore.Value - bankCut
                            && !areaIndex.Nearby(point).Any(otherArea => otherArea.Contains(point)))
                        {
                            var rim =
                                math.lerp(
                                    levee,
                                    Math.Max(land, shore.Value + ShoreRim),
                                    Ramp(Math.Max(0, edge - spacing) / (shoreBand - spacing)));

                            var raised =
                                math.clamp(rim, 0, terrain.HeightScaleMeters);

                            if (raised > conditioned[bankIndex])
                            {
                                conditioned[bankIndex] = raised;
                                raisedBank++;

                                if (editMask[bankIndex] == 0)
                                {
                                    editMask[bankIndex] = 1;
                                    editCount++;
                                }
                            }

                            continue;
                        }

                        if (land <= bank)
                            continue;

                        var cut =
                            math.clamp(Math.Max(land - bankCut, bank), 0, terrain.HeightScaleMeters);

                        if (cut >= conditioned[bankIndex])
                            continue;

                        conditioned[bankIndex] = cut;
                        shapedBank++;

                        if (editMask[bankIndex] == 0)
                        {
                            editMask[bankIndex] = 1;
                            editCount++;
                        }

                        continue;
                    }

                    var stage = level(point);

                    if (!stage.HasValue || !math.isfinite(stage.Value))
                        continue;

                    // An edge shared with other mapped water (a river drawn in
                    // pieces) is not a shore: the bed runs through it at full depth.
                    if (edge < bedBand)
                    {
                        var outward = math.normalizesafe(closest - point.xz);
                        var across = new float3(closest.x + outward.x, 0, closest.y + outward.y);

                        if (areaIndex.Nearby(across).Any(otherArea => otherArea != area && otherArea.Contains(across)))
                            edge = bedBand;
                    }

                    // A flowing polygon is a metre deeper than a lake (rivers read as cut channels).
                    var depth = Profile(edge, Lake(area) ? 2 : 3, bedBand);
                    var index = z * n + x;
                    var original = terrain.Heights[index];
                    var height =
                        Math.Max(
                            0,
                            Lake(area)
                                ? Math.Max(original - 8, Math.Min(original, stage.Value - depth))
                                : Math.Min(original, stage.Value - depth));

                    var bed = Math.Min(height, terrain.HeightScaleMeters);

                    // Within the band the bed is later raised to the shore profile
                    // itself where the source lies deeper, so a deep channel beside
                    // its bank does not reopen the step at the edge.
                    if (edge < bedBand)
                    {
                        var floor = Math.Max(0, stage.Value - depth);
                        shoreFloor[index] = float.IsNaN(shoreFloor[index]) ? floor : Math.Max(shoreFloor[index], floor);
                    }

                    // The shore profile is shallow by design within a few metres of the edge.
                    accepted++;

                    if (depth >= .1f && height >= stage.Value - .1f)
                        blockedByBudget++;

                    // The accepted wet interior owns its bed even if it was already
                    // below target. Later earthworks must not introduce a new dam.
                    if (!Lake(area) && editMask[index] == 0)
                    {
                        editMask[index] = 1;
                        editCount++;
                    }

                    if (bed >= conditioned[index])
                        continue;

                    conditioned[index] = bed;

                    if (editMask[index] == 0)
                    {
                        editMask[index] = 1;
                        editCount++;
                    }
                }

            log.Diagnostic(
                $"WATER AREA BED body={area.Polygon.Identity}; cellsVisitedInside={accepted}; bedAboveStageAfterBudget={blockedByBudget}; continuousMask=true; holesPreserved=true; lakeBudgetMeters=8; bedCutsConfinedToWetMask=true; shoreBandMeters={shoreBand:0.#}; bankShapingOnlyLowersWithin={bankCut}m");

            log.Count("Water area bed cells above stage", blockedByBudget);
        }

        bool Add(
            long way,
            long body,
            string kind,
            string name,
            float3 point,
            float radius,
            float stage,
            float bank,
            IReadOnlyList<float3> banks,
            IReadOnlyList<float3> probes,
            Func<float3, bool> mask,
            bool gridSource = false)
        {
            if (!Inside(point) || radius < Math.Max(2, cell * .25f) || !math.isfinite(stage))
                return false;

            // No circle spilling onto banks or islands, even for boundary-driven rivers. A
            // grid source reaches only the water-grid nodes within its radius, so those are
            // what must lie in the channel.
            bool Fits(float r)
                => gridSource
                    ? GridNodes(point, r).All(mask)
                    : Enumerable
                    .Range(1, 3)
                    .All(
                        ring => Enumerable
                            .Range(0, 16)
                            .All(
                                i => mask(
                                    point
                                        + new float3(math.cos(i * math.PI / 8), 0, math.sin(i * math.PI / 8))
                                            * (r * ring / 3))));

            // A grid source keeps its radius: smaller, it reaches no node and adds no water.
            while (!gridSource && radius > Math.Max(2, cell * .25f) && !Fits(radius))
                radius = Math.Max(Math.Max(2, cell * .25f), radius - cell * .25f);

            // A source that had to shrink below 0.9 cell (a pond, a narrow lake) can miss every
            // grid node and stay dry: 464 of DC's 1,055 such sources were dry on 11-13. It
            // becomes a grid source on the nearest node inside its water instead.
            if (!gridSource && radius < cell * .9f && mask(GridNode(point)))
            {
                point = GridNode(point);
                radius = cell * .9f;
                gridSource = true;
            }

            if (!Fits(radius) || stage > bank - .35f)
            {
                unresolved++;
                log.Count(
                    stage > bank - .35f ? "Water stage bank conflicts" : "Water source footprint policy rejected");

                return false;
            }

            // Give every footprint ring a fully submerged core, including the
            // neighbouring terrain samples used by native bilinear interpolation.
            Cut(point, (radius + spacing * 2) / .7f, stage - 2.5f, mask, kind == "lake" ? 8 : float.PositiveInfinity);
            point.y = Height(point);
            var depth = stage - point.y;

            if (depth < .4f)
            {
                unresolved++;
                log.Diagnostic(
                    $"WATER BODY UNRESOLVED body={body}; reason=bed-above-bank-limited-stage; bed={point.y:R}; stage={stage:R}; noFloodBypass=true");

                return false;
            }

            var repairCells = new List<int>();
            var pixel = Pixel(point);
            var reach = (int)Math.Ceiling((radius + spacing * 1.5f) / spacing);

            for (var z = Math.Max(0, pixel / n - reach); z <= Math.Min(n - 1, pixel / n + reach); z++)
                for (var x = Math.Max(0, pixel % n - reach); x <= Math.Min(n - 1, pixel % n + reach); x++)
                {
                    var otherPoint = new float3(x * spacing - half, 0, z * spacing - half);

                    if (math.distance(otherPoint.xz, point.xz) <= radius + spacing * 1.5f && mask(otherPoint))
                        repairCells.Add(z * n + x);
                }

            sources.Add(
                new RiverSourcePlan(
                    way,
                    name,
                    "body-stage",
                    point,
                    radius,
                    depth,
                    bank,
                    probes.Count > 0 ? probes[probes.Count - 1] : point)
                {
                    BodyId = body,
                    BodyKind = kind,
                    BoundaryDriven = false,
                    TargetSurfaceLevel = stage,
                    SurfaceOffsetMeters = kind == "river" ? 1 : 0,
                    BankProbes = banks,
                    ChannelProbes = probes,
                    BedRepairCells = repairCells,
                    GridSnapped = gridSource
                });

            return true;
        }

        // The water-grid nodes within radius of a point: multiples of the water cell size,
        // as the game reads its grid (WaterSystem.CalculateSourceMultiplier).
        IEnumerable<float3> GridNodes(float3 center, float radius)
        {
            var reach = (int)Math.Ceiling(radius / cell);
            var x0 = (int)Math.Round(center.x / cell);
            var z0 = (int)Math.Round(center.z / cell);

            for (var dz = -reach; dz <= reach; dz++)
                for (var dx = -reach; dx <= reach; dx++)
                {
                    var node = new float3((x0 + dx) * cell, center.y, (z0 + dz) * cell);

                    if (math.distance(node.xz, center.xz) < radius)
                        yield return node;
                }
        }

        // The water-grid node nearest a point.
        float3 GridNode(float3 point)
            => new((float)Math.Round(point.x / cell) * cell, point.y, (float)Math.Round(point.z / cell) * cell);

        // Lakes use one flat body level, inferred conservatively from their mapped
        // shore. Internal holes stay dry. Sparse controls cover large isolated basins.
        phase = "mapped lakes";
        Pulse(true);
        // A salt lagoon on a coast (water=lagoon, salt=yes) is open to the sea,
        // so its water stands no lower than the sea's surface; its low shores
        // flood as the sea's own would, so they are not bank probes. Planned
        // from its lowest shore probe, the Étang de Berre (water=pond, salt=yes)
        // stood 12 m below the sea, and so did every river draining into it.
        var seaSurface = WaterLevels.Effective(terrain.SeaLevelMeters);
        // With RaiseBanks a lake's level is the elevation data over its water (the data
        // reads the surface there, not the bed): the median over the lake, so a single
        // low shore point no longer sets it. It rises at most LakeLeveeMax above the
        // shore rule's level (lowest bank − 0.5 m), and every shore lower than the new
        // level plus LakeFreeboard gets a levee (shoreLevees, raised after the rivers).
        const float LakeLeveeMax = 5, LakeFreeboard = 1;
        var raiseBanks = DesignSwitches.On(DesignSwitches.RaiseBanks);
        var shoreLevees = new List<(float3 A, float3 B, float Height)>();
        var raisedLakes = 0;
        var coastalMap = data.Ways.Values.Any(way => NetworkSelection.HasTag(way.Tags, "natural", "coastline"));

        bool Lagoon(Area firstArea)
            => coastalMap
                && (NetworkSelection.HasTag(firstArea.Polygon.Tags, "water", "lagoon")
                    || NetworkSelection.HasTag(firstArea.Polygon.Tags, "salt", "yes"));

        // Basins touching the coast at about sea level are open to the sea.
        var coastCells = new HashSet<(int, int)>();

        foreach (var way in data.Ways.Values.Where(
            otherWay => NetworkSelection.HasTag(otherWay.Tags, "natural", "coastline")))
            foreach (var id in way.NodeIds)
                if (data.Nodes.ContainsKey(id))
                {
                    var point = Local(id);
                    coastCells.Add(((int)Math.Floor(point.x / 64), (int)Math.Floor(point.z / 64)));
                }

        bool NearCoast(float3 point)
        {
            var cx = (int)Math.Floor(point.x / 64);
            var cz = (int)Math.Floor(point.z / 64);

            for (var dx = -1; dx <= 1; dx++)
                for (var dz = -1; dz <= 1; dz++)
                    if (coastCells.Contains((cx + dx, cz + dz)))
                        return true;

            return false;
        }

        bool TouchesCoast(Area firstArea) => firstArea.Polygon.Outer.Any(points => points.Any(NearCoast));

        foreach (var area in areas.Where(Lake))
        {
            token.ThrowIfCancellationRequested();
            lakeCount++;
            var polygon = area.Polygon;
            activeBody = polygon.Id;
            Pulse();
            var banks =
                polygon.Outer
                    .SelectMany(points => points.Where((point, i) => i % Math.Max(1, points.Length / 64) == 0))
                    .Where(Inside)
                .Select(
                    point => Enumerable
                        .Range(0, 8)
                        .Select(
                            i => DryProbe(
                                point,
                                new float3(math.cos(i * math.PI / 4), 0, math.sin(i * math.PI / 4)),
                                area.Contains,
                                cell * 2))
                        .FirstOrDefault(otherPoint => otherPoint.HasValue))
                .Where(point => point.HasValue).Select(point => point!.Value).ToArray();

            if (banks.Length < 3)
            {
                unresolved++;
                continue;
            }

            var sorted = banks.Select(Height).OrderBy(h => h).ToArray();
            var stage = sorted[0] - .5f;

            if (!Lagoon(area)
                && stage <= seaSurface + 1
                && !NetworkSelection.HasTag(area.Polygon.Tags, "water", "reservoir")
                && !NetworkSelection.HasTag(area.Polygon.Tags, "landuse", "reservoir")
                && TouchesCoast(area))
            {
                log.Count("Water basins open to the sea");
                continue;
            }

            if (Lagoon(area) && stage < seaSurface)
            {
                stage = seaSurface;
                sorted[0] = stage + .5f;
                banks = Array.Empty<float3>();
                log.Count("Water lagoons held at the sea's surface");
            }

            if (raiseBanks && !Lagoon(area))
            {
                var readings = new List<float>();
                var sampleStep = Math.Max(cell, Math.Max(area.Max.x - area.Min.x, area.Max.y - area.Min.y) / 24);

                for (var z = area.Min.y + sampleStep / 2; z < area.Max.y; z += sampleStep)
                    for (var x = area.Min.x + sampleStep / 2; x < area.Max.x; x += sampleStep)
                    {
                        var point = new float3(x, 0, z);

                        if (area.Contains(point) && Inside(point))
                            readings.Add(Height(point));
                    }

                readings.Sort();

                // Only a flat reading is a water surface: over narrow ditches the data
                // averages in the land beside them, and over a valley reservoir it may
                // hold the old ground. Then the shore rule stands.
                if (readings.Count >= 9
                    && readings[readings.Count * 9 / 10] - readings[readings.Count / 10] <= 2)
                {
                    var level = Math.Min(readings[readings.Count / 2], stage + LakeLeveeMax);

                    if (level > stage)
                    {
                        stage = level;
                        sorted[0] = stage + LakeFreeboard;
                        raisedLakes++;

                        // Rings may be left open (the first point is not repeated), so
                        // each point is joined to the next one round, the last to the first.
                        foreach (var ring in polygon.Outer.Concat(polygon.Inner))
                            for (var i = 0; i < ring.Length; i++)
                            {
                                var start = ring[i];
                                var end = ring[(i + 1) % ring.Length];

                                if (Inside(start) || Inside(end))
                                    shoreLevees.Add((start, end, stage + LakeFreeboard));
                            }
                    }
                }
            }

            areaStages.Add(new WaterAreaStage(polygon.Identity, "lake", stage));
            CarveArea(area, point => stage);

            if (NetworkEvidence.Measure(polygon.Tags, "ele") is float absolute)
                log.Diagnostic(
                    $"WATER ELE body={polygon.Id}; sourceAbsoluteMeters={absolute}; stageFromConsistentLocalShore=true; absoluteDatumNotAssumed=true");

            var step = Math.Max(cell * 4, 192);
            var probes = new List<float3>();

            for (var z = Math.Max(-half, area.Min.y) + cell * 2; z < Math.Min(half, area.Max.y); z += step)
                for (var x = Math.Max(-half, area.Min.x) + cell * 2; x < Math.Min(half, area.Max.x); x += step)
                {
                    Pulse();
                    var point = new float3(x, 0, z);

                    if (area.Contains(point) && Inside(point))
                        probes.Add(point);
                }

            if (probes.Count == 0)
            {
                var center = new float3((area.Min.x + area.Max.x) * .5f, 0, (area.Min.y + area.Max.y) * .5f);

                if (area.Contains(center) && Inside(center))
                    probes.Add(center);
            }

            foreach (var point in probes)
            {
                Cut(point, step * .8f, stage - 2, area.Contains, 8);
            }

            var controls = 0;

            foreach (var point in probes.Where((otherPoint, i) => i % 4 == 0))
                if (Add(
                    polygon.Id,
                    polygon.Identity,
                    "lake",
                    polygon.Tags.TryGetValue("name", out var name) ? name : "lake",
                    point,
                    cell * 2,
                    stage,
                    sorted[0],
                    banks,
                    probes.Take(24).ToArray(),
                    area.Contains))
                    controls++;

            log.Diagnostic(
                $"WATER BODY id={polygon.Id}; kind=lake; stage={stage:R}; controls={controls}; shoreProbes={banks.Length}; flatStage=true; holesProtected=true");
        }

        float Length(OsmWay way)
        {
            var length = 0f;

            for (var i = 1; i < way.NodeIds.Length; i++)
                length += math.distance(Local(way.NodeIds[i - 1]).xz, Local(way.NodeIds[i]).xz);

            return length;
        }

        var network = RiverNetwork(data);
        var lines = data.Ways.Values.Where(way => network.Contains(way.Id)).OrderBy(way => way.Id).ToArray();
        var buried = lines.Where(way => Buried(way.Tags)).ToArray();
        log.Count("Water buried waterways opened", buried.Length);
        log.Count(
            "Water buried channel metres",
            (int)Math.Min(int.MaxValue, Math.Round(buried.Sum(way => (double)Length(way)))));

        log.Diagnostic(
            $"WATER BURIED CHANNELS selectedWays={buried.Length}; openedAtSurface=true; lengthCutoff=false; crossingBridgesRequired=true; canalsAndIntermittentWaysExcluded=true; nativeFlowVerified=false");

        totalWays = lines.Length;
        phase = "connecting river reaches";
        Pulse(true);
        // Connected waterway ways share a durable body identity independent of controls.
        // Attach roots by rank, not source iteration order. A long river must
        // not build a deep parent chain before the first bed sample is processed.
        var parents = lines.ToDictionary(way => way.Id, way => way.Id);
        var ranks = lines.ToDictionary(way => way.Id, way => 0);

        long Root(long id)
        {
            while (parents[id] != id)
            {
                token.ThrowIfCancellationRequested();
                parents[id] = parents[parents[id]];
                id = parents[id];
            }

            return id;
        }

        var bodyIds = lines.ToDictionary(way => way.Id, way => way.Id);

        void Connect(long a, long b)
        {
            a = Root(a);
            b = Root(b);

            if (a == b)
                return;

            if (ranks[a] < ranks[b])
            {
                var swap = a;
                a = b;
                b = swap;
            }

            parents[b] = a;

            if (ranks[a] == ranks[b])
                ranks[a]++;

            bodyIds[a] = Math.Min(bodyIds[a], bodyIds[b]);
        }

        var ends = new Dictionary<long, long>();

        foreach (var way in lines)
            foreach (var id in way.NodeIds)
            {
                Pulse();

                if (ends.TryGetValue(id, out var other))
                    Connect(way.Id, other);
                else
                    ends[id] = way.Id;
            }

        var positions = new Dictionary<long, float3>();
        var nodeStage = new Dictionary<long, float>();

        foreach (var way in lines)
            foreach (var id in way.NodeIds)
                if (!positions.ContainsKey(id))
                {
                    Pulse();
                    var point = Local(id);
                    positions[id] = point;
                    nodeStage[id] = BankHeight(point) + 2.8f;
                }

        var bankTargets = new Dictionary<long, float>();
        var bankLimitedNodes = 0;

        foreach (var way in lines)
            for (var i = 0; i < way.NodeIds.Length; i++)
            {
                Pulse();
                var id = way.NodeIds[i];
                var point = positions[id];
                var neighbor = positions[way.NodeIds[i == way.NodeIds.Length - 1 ? i - 1 : i + 1]];

                if (math.distancesq(neighbor.xz, point.xz) < .0001f)
                {
                    log.Count("Water duplicate node positions skipped");
                    continue;
                }

                var normal = math.normalizesafe(new float3(-(neighbor - point).z, 0, (neighbor - point).x));
                var area =
                    areaIndex
                        .Nearby(point)
                        .FirstOrDefault(
                            otherArea => Flowing(otherArea)
                                && !Coastal(otherArea)
                                && !Dry(otherArea)
                                && otherArea.Contains(point));

                var halfWidth =
                    Math.Max(
                        1,
                        (NetworkEvidence.Measure(way.Tags, "width")
                            ?? (NetworkSelection.HasTag(way.Tags, "waterway", "river") ? 32 : 12))
                            * .5f);

                var bank = float.PositiveInfinity;

                foreach (var side in new[] { -1, 1 })
                {
                    var distance = Math.Max(cell, halfWidth);

                    while (distance < 512 && area != null && area.Contains(point + normal * (distance * side)))
                        distance += Math.Max(cell, halfWidth * .5f);

                    var otherPoint =
                        DryProbe(
                            point,
                            normal * side,
                            area != null
                                ? area.Contains
                                : r => NetworkNormalizer.DistanceToSegment(r, point, neighbor) <= halfWidth,
                            distance);

                    if (otherPoint.HasValue)
                        bank = Math.Min(bank, BankHeight(otherPoint.Value));
                }

                if (math.isfinite(bank))
                {
                    var target = bank - .35f;

                    if (bankTargets.TryGetValue(id, out var prior))
                        target = Math.Min(prior, target);

                    bankTargets[id] = target;

                    if (nodeStage[id] > bank - .35f)
                        bankLimitedNodes++;
                }
            }

        foreach (var entry in bankTargets)
            nodeStage[entry.Key] = entry.Value;

        var desiredStage = new Dictionary<long, float>(nodeStage);
        log.Diagnostic(
            $"WATER OFFSET CONTAINMENT requestedOffsetMeters=1; bankLimitedNodeUpdates={bankLimitedNodes}; effectiveStageCappedAtBank=true; fullOffsetNotGuaranteed=true; reason=offset-and-containment-can-conflict");

        log.Count("Water offset bank limited nodes", bankLimitedNodes);
        var baseBed = positions.ToDictionary(entry => entry.Key, entry => Height(entry.Value));
        var unsupportedBodies = new HashSet<long>();
        var sampledBankUpdates = 0;
        var missingBankSamples = 0;
        var lowerStage = positions.ToDictionary(entry => entry.Key, entry =>
        {
            var polygon =
                areaIndex
                    .Nearby(entry.Value)
                    .FirstOrDefault(
                        area => Flowing(area) && !Coastal(area) && !Dry(area) && area.Contains(entry.Value));

            return polygon != null ? .4f : Math.Max(.4f, baseBed[entry.Key] - 8 + .4f);
        });

        phase = "sampling contained river stage bounds";
        Pulse(true);

        // Constrain the shared endpoint solve BEFORE carving or creating any
        // regulator. Rejecting a local regulator alone cannot stop an upstream
        // regulator of the same body flooding a low bank halfway along a reach.
        foreach (var way in lines)
            for (var i = 1; i < way.NodeIds.Length; i++)
            {
                Pulse();
                var aid = way.NodeIds[i - 1];
                var bid = way.NodeIds[i];
                var vectorA = positions[aid];
                var vectorB = positions[bid];

                if (!WaterPlanner.Clip(vectorA, vectorB, half - cell * 2, out var low, out var high)
                    || math.distancesq(vectorA.xz, vectorB.xz) < .0001f)
                    continue;

                var halfWidth =
                    Math.Max(
                        .5f,
                        (NetworkEvidence.Measure(way.Tags, "width")
                            ?? (NetworkSelection.HasTag(way.Tags, "waterway", "river") ? 32 : 12))
                            * .5f);

                var length = math.distance(vectorA.xz, vectorB.xz) * (high - low);
                var samples =
                    Math.Max(1, (int)Math.Ceiling(length / Math.Max(spacing * .5f, Math.Min(cell * 2, halfWidth))));

                var normal = math.normalizesafe(new float3(-(vectorB - vectorA).z, 0, (vectorB - vectorA).x));

                for (var j = 0; j <= samples; j++)
                {
                    Pulse();
                    var t = math.lerp(low, high, j / (float)samples);
                    var point = math.lerp(vectorA, vectorB, t);

                    if (MappedDry(point))
                        continue;

                    var polygon =
                        areaIndex
                            .Nearby(point)
                            .FirstOrDefault(
                                area => Flowing(area) && !Coastal(area) && !Dry(area) && area.Contains(point));

                    var bank = float.PositiveInfinity;
                    var banks = 0;

                    foreach (var side in new[] { -1, 1 })
                    {
                        var distance = Math.Max(cell, halfWidth);

                        while (distance < 512
                            && polygon != null
                            && polygon.Contains(point + normal * (distance * side)))
                            distance += Math.Max(cell, halfWidth * .5f);

                        var otherPoint =
                            DryProbe(
                                point,
                                normal * side,
                                polygon != null
                                    ? polygon.Contains
                                    : r => NetworkNormalizer.DistanceToSegment(r, vectorA, vectorB) <= halfWidth,
                                distance);

                        if (otherPoint.HasValue)
                        {
                            bank = Math.Min(bank, BankHeight(otherPoint.Value));
                            banks++;
                        }
                    }

                    if (banks < 2 || !math.isfinite(bank))
                    {
                        missingBankSamples++;
                        continue;
                    }

                    var proposed = math.lerp(nodeStage[aid], nodeStage[bid], t);
                    var excess = proposed - (bank - .35f);

                    if (excess > .005f)
                    {
                        // Lower both endpoints by the same excess. This preserves
                        // the reach gradient and satisfies its sampled linear bound;
                        // all later adjustments are downward, so prior bounds hold.
                        // Weighted projection cannot cumulatively lower a whole river
                        // below its recoverable bed floor. Unsatisfied local constraints
                        // remain local diagnostics, not whole-body isolation.
                        // Absolute bounds reference the immutable desired profile.
                        // Dense repeated samples cannot cumulatively subtract the
                        // same bank constraint until a whole river becomes puddles.
                        var slope = desiredStage[bid] - desiredStage[aid];
                        var limit = bank - .35f;
                        nodeStage[aid] = Math.Min(nodeStage[aid], Math.Max(lowerStage[aid], limit - slope * t));
                        nodeStage[bid] = Math.Min(nodeStage[bid], Math.Max(lowerStage[bid], limit + slope * (1 - t)));
                        sampledBankUpdates++;
                    }

                    var minimum = Height(point) - (polygon != null ? 12 : 4) + .4f;

                    if (math.lerp(nodeStage[aid], nodeStage[bid], t) < minimum - .005f)
                        unsupportedBodies.Add(bodyIds[Root(way.Id)]);
                }
            }

        foreach (var id in positions.Keys)
            nodeStage[id] = Math.Max(nodeStage[id], lowerStage[id]);

        foreach (var way in lines)
            if (way.NodeIds.Any(id => nodeStage[id] < lowerStage[id] - .005f))
                unsupportedBodies.Add(bodyIds[Root(way.Id)]);

        log.Count("Water sampled bank bound updates", sampledBankUpdates);
        log.Diagnostic(
            $"WATER REACH BOUNDS sampledBankUpdates={sampledBankUpdates}; missingBankSamples={missingBankSamples}; infeasibleBodiesBeforeSolve={unsupportedBodies.Count}; endpointBoundsShared=true; boundedBedRecovery=true; noLocalRegulatorFloodBypass=true");

        // OSM waterways are directed downstream. Shared source nodes must not
        // reset the river stage at every way boundary. Bound propagation by the
        // reach bed-edit budget so a bad elevation cannot flatten an entire river.
        var outgoing = positions.Keys.ToDictionary(id => id, id => new HashSet<long>());
        var incoming = positions.Keys.ToDictionary(id => id, id => 0);

        foreach (var way in lines)
            for (var i = 1; i < way.NodeIds.Length; i++)
                if (outgoing[way.NodeIds[i - 1]].Add(way.NodeIds[i]))
                    incoming[way.NodeIds[i]]++;

        var ready =
            incoming.Where(entry => entry.Value == 0).OrderBy(entry => entry.Key).Select(entry => entry.Key).ToList();

        var readyIndex = 0;
        var solved = new HashSet<long>();
        var infeasible = 0;
        phase = "solving directed water stages";
        Pulse(true);

        void Propagate(long a, long b)
        {
            var minimum = lowerStage[b];

            if (nodeStage[a] < minimum - .005f)
            {
                infeasible++;

                if (ends.TryGetValue(a, out var way))
                    unsupportedBodies.Add(bodyIds[Root(way)]);

                return; // Never drag the downstream DAG below recoverable beds.
            }

            nodeStage[b] = Math.Min(nodeStage[b], nodeStage[a]);
        }

        while (readyIndex < ready.Count)
        {
            Pulse();
            var a = ready[readyIndex++];
            solved.Add(a);

            foreach (var b in outgoing[a])
            {
                Propagate(a, b);

                if (--incoming[b] == 0)
                    ready.Add(b);
            }
        }

        // Cyclic/tidal or incorrectly directed evidence is not silently treated as
        // an ordinary river. Retain conservative local levels and expose the residual.
        var cyclic = positions.Count - solved.Count;
        var residual = outgoing.Sum(entry => entry.Value.Count(b => nodeStage[b] > nodeStage[entry.Key] + .005f));

        foreach (var link in outgoing)
            foreach (var downstream in link.Value)
                if (nodeStage[downstream] > nodeStage[link.Key] + .005f && ends.TryGetValue(link.Key, out var way))
                    unsupportedBodies.Add(bodyIds[Root(way)]);

        log.Diagnostic(
            $"WATER STAGE SOLVE algorithm=topological-v2; nodes={positions.Count}; solvedNodes={solved.Count}; cycleOrCycleDependentNodes={cyclic}; infeasibleBudgetEdges={infeasible}; uphillResidualEdges={residual}; offsetMeters=1; controlsIndependent=true; cyclicLevelsAreLocalApproximation=true");

        log.Count("Water stage cycle dependent nodes", cyclic);
        log.Count("Water stage uphill residual edges", residual);
        var controlGrid = new Dictionary<(int, int), List<RiverSourcePlan>>();

        // Sources of one body stand at least spacing metres apart: 192 m, or 384 m
        // along a waterway narrower than two water cells without a polygon, whose
        // water flows on to the next one (DC's streams otherwise took 9,600).
        bool NearControl(long body, float3 point, float spacing = 192)
        {
            var x = (int)Math.Floor(point.x / 192);
            var z = (int)Math.Floor(point.z / 192);
            var r = (int)Math.Ceiling(spacing / 192);

            for (var dx = -r; dx <= r; dx++)
                for (var dz = -r; dz <= r; dz++)
                    if (controlGrid.TryGetValue((x + dx, z + dz), out var list)
                        && list.Any(
                            river => river.BodyId == body && math.distance(river.Position.xz, point.xz) < spacing))
                        return true;

            return false;
        }

        phase = "conditioning river beds and placing controls";
        Pulse(true);
        // A channel without a mapped river polygon may be cut this far into the
        // source ground. Narrow urban rivers (Marseille's Huveaune) sit in DEM
        // cells that average in their banks, so 8 m skipped whole stretches,
        // which then had neither bed nor water source and broke the river up;
        // 12 m still skipped 3,097 Marseille samples.
        // A river or stream without a polygon is carved at least one water
        // simulation cell either side of its line, ChannelDepth below its stage,
        // with banks sloping off over the outer 40% of the cut. Cut to its 6 m
        // half width, a stream was a row of pits narrower than the game's 28 m
        // water cells, and the water could not flow along it.
        // Each sample's stage stands RiverFreeboard below its lower bank: at
        // the solve's .35 m, and 3.5 m of channel, the water ran level with the
        // land and Marseille's rivers read as shallow paint rather than a cut.
        // Every sample is carved, including one with no dry bank found on both
        // sides (a map edge, a wide polygon) or one too far below ground: such a
        // sample only gets no water source. Skipping it left a dry plug that
        // split the river into disconnected pools.
        const float ChannelCut = 20, ChannelDepth = 5, RiverFreeboard = 1;
        var placedSamples = 0;
        var skippedTooDeep = 0;
        var skippedAboveBank = 0;
        var skippedNoBanks = 0;
        var seaLevelSamples = 0;
        // Each waterway gets one water surface before anything is cut: its
        // samples are collected first, then solved, then carved.
        //  - Each sample may stand RiverFreeboard below its lower bank. A dip
        //    in that bound narrower than riverBridge and at most riverLeveeMax deep
        //    is bridged instead of followed: the water keeps its level and a
        //    levee is raised beside the channel there. Following each dip
        //    (the node solve's interpolated level did, along a whole edge)
        //    dropped every later sample with it and cut trenches downstream:
        //    Marseille's median cut was 7 m, 3.2 m without it.
        //  - The surface never rises downstream: a running minimum along the
        //    way, starting from the lowest surface of the ways ending at its
        //    first node and capped at every node where a tributary ended at its
        //    own surface. Ways run in the order of their last node in the stage
        //    solve, so tributaries and upstream ways come first. Per-sample
        //    stages before rose and fell with each bank, and a source held
        //    higher downstream pushed its water back upstream over lower banks.
        //  - A waterway that runs to the sea never stands below the sea's
        //    surface: the Huveaune's mouth was planned 8 m below it, so the sea
        //    would have run up the channel.
        // The bed never rises downstream either: each cut reaches no higher than
        // the lowest bed cut upstream on the way, carried on through way ends.
        // Cut to stage − ChannelDepth alone, a quarter of Marseille's open
        // waterway length stood behind a dam. Only the channel's own target is
        // carried, never a deeper lake bed it passes through.
        // A canal runs level along a hillside on a bench with an embankment on
        // the downhill side: its bound is the higher of that bank and the ground
        // on its line, and wider, higher dips are bridged. Bounded by its
        // downhill bank alone, the Canal de Marseille sank into trenches more
        // than 15 m deep along 1,950 of its reaches.
        // With RaiseBanks a river bridges dips up to 600 m long and 5 m deep, as a
        // canal nearly does: the water keeps its level and a levee holds it, instead
        // of the level dropping to a low bank and staying down for the rest of the way.
        var riverBridge = raiseBanks ? 600f : 150f;
        var riverLeveeMax = raiseBanks ? 5f : 3f;
        const float CanalBridge = 400, CanalLeveeMax = 6;
        var coastline = new Dictionary<(int, int), List<(float3 A, float3 B)>>();

        foreach (var way in data.Ways.Values.Where(
            otherWay => NetworkSelection.HasTag(otherWay.Tags, "natural", "coastline")))
            for (var i = 1; i < way.NodeIds.Length; i++)
            {
                if (!data.Nodes.ContainsKey(way.NodeIds[i - 1]) || !data.Nodes.ContainsKey(way.NodeIds[i]))
                    continue;

                var vectorA = Local(way.NodeIds[i - 1]);
                var vectorB = Local(way.NodeIds[i]);
                var key =
                    (
                        (int)Math.Floor((vectorA.x + vectorB.x) / 2 / 512),
                        (int)Math.Floor((vectorA.z + vectorB.z) / 2 / 512));

                if (math.distance(vectorA.xz, vectorB.xz) > 1024)
                    continue;

                if (!coastline.TryGetValue(key, out var list))
                    coastline[key] = list = new();

                list.Add((vectorA, vectorB));
            }

        // A way's end reaches the sea when it lies in mapped sea, within 150 m of a lagoon, or within
        // 150 m of the coastline.
        bool AtSea(float3 point)
        {
            if (areaIndex
                .Nearby(point)
                .Any(
                    area => Coastal(area) && area.Contains(point)
                        || Lagoon(area) && (area.Contains(point) || area.EdgeDistance(point, 150, out _) <= 150)))
                return true;

            var cx = (int)Math.Floor(point.x / 512);
            var cz = (int)Math.Floor(point.z / 512);

            for (var dx = -2; dx <= 2; dx++)
                for (var dz = -2; dz <= 2; dz++)
                    if (coastline.TryGetValue((cx + dx, cz + dz), out var list)
                        && list.Any(item => NetworkNormalizer.DistanceToSegment(point, item.A, item.B) <= 150))
                        return true;

            return false;
        }

        var upstreamOf = positions.Keys.ToDictionary(id => id, id => new List<long>());

        foreach (var link in outgoing)
            foreach (var b in link.Value)
                upstreamOf[b].Add(link.Key);

        var toSea = new HashSet<long>();
        var seaQueue = new List<long>();

        foreach (var id in positions.Keys)
            if (outgoing[id].Count == 0 && AtSea(positions[id]) && toSea.Add(id))
                seaQueue.Add(id);

        for (var q = 0; q < seaQueue.Count; q++)
            foreach (var u in upstreamOf[seaQueue[q]])
                if (toSea.Add(u))
                    seaQueue.Add(u);

        var upstreamOrder = new Dictionary<long, int>();

        for (var k = 0; k < ready.Count; k++)
            upstreamOrder[ready[k]] = k;

        var nodeSurface = new Dictionary<long, float>();
        var nodeBed = new Dictionary<long, float>();
        var waySurface = new Dictionary<long, float>();
        var wayBed = new Dictionary<long, float>();
        var levees = new List<(float3 A, float3 B, float3 P, float Channel, float Height)>();
        var bridgedSamples = 0;
        var seaRaisedSamples = 0;
        // A mapped way can continue through a confluence or branch. Condition
        // its junction-to-junction pieces in node order, retaining the source
        // way identity. Sorting whole ways by their final node can process a
        // branch before the level/bed reaches its interior starting junction.
        var junctionPieces = 0;

        IEnumerable<OsmWay> Pieces()
        {
            foreach (var line in lines)
            {
                token.ThrowIfCancellationRequested();
                var start = 0;

                for (var i = 1; i < line.NodeIds.Length; i++)
                {
                    var id = line.NodeIds[i];

                    if (i < line.NodeIds.Length - 1 && upstreamOf[id].Count == 1 && outgoing[id].Count == 1)
                        continue;

                    junctionPieces++;
                    yield return start == 0 && i == line.NodeIds.Length - 1
                        ? line
                        : line with { NodeIds = line.NodeIds.Skip(start).Take(i - start + 1).ToArray() };

                    start = i;
                }
            }
        }

        var conditionedWays = new HashSet<long>();

        foreach (var way in Pieces()
            .OrderBy(
                way => upstreamOrder.TryGetValue(way.NodeIds[way.NodeIds.Length - 1], out var k) ? k : int.MaxValue)
            .ThenBy(way => way.Id))
        {
            token.ThrowIfCancellationRequested();

            if (conditionedWays.Add(way.Id))
                riverCount++;

            var body = bodyIds[Root(way.Id)];
            activeBody = body;
            Pulse();
            var bedLimit = nodeBed.TryGetValue(way.NodeIds[0], out var carried) ? carried : float.PositiveInfinity;

            if (wayBed.TryGetValue(way.Id, out var previousBed))
                bedLimit = Math.Min(bedLimit, previousBed);

            if (unsupportedBodies.Contains(body))
                log.Count("Water bodies using local feasible reaches");

            var sourceWidth =
                NetworkEvidence.Measure(way.Tags, "width")
                    ?? (NetworkSelection.HasTag(way.Tags, "waterway", "river") ? 32 : 12);

            if (!math.isfinite(sourceWidth) || sourceWidth <= 0 || sourceWidth > 2048)
            {
                log.Count("Water width repaired");
                sourceWidth = NetworkSelection.HasTag(way.Tags, "waterway", "river") ? 32 : 12;
            }

            var reach = sourceWidth * .5f;
            // Every waterway of the network is carved at least a water cell
            // either side of its line: a ditch or a canal cut to its own 6 m
            // half width on a 14 m grid read as a row of square pits (Cairo's
            // canals) that the water could not flow along.
            var channel = Math.Max(reach, cell);
            var samples = new List<ChannelSample>();
            var station = 0f;

            for (var i = 1; i < way.NodeIds.Length; i++)
            {
                var vectorA = positions[way.NodeIds[i - 1]];
                var vectorB = positions[way.NodeIds[i]];

                if (!WaterPlanner.Clip(vectorA, vectorB, half - cell * 2, out var low, out var high))
                    continue;

                var originalA = vectorA;
                var originalB = vectorB;
                vectorA = math.lerp(originalA, originalB, low);
                vectorB = originalA + (originalB - originalA) * high;
                var length = math.distance(vectorA.xz, vectorB.xz);
                var steps = Math.Max(1, (int)Math.Ceiling(length / Math.Max(spacing * .5f, Math.Min(cell * 2, reach))));

                if (length < .01f)
                {
                    log.Count("Water zero length reaches skipped");
                    continue;
                }

                var normal = math.normalizesafe(new float3(-(vectorB - vectorA).z, 0, (vectorB - vectorA).x));

                for (var j = 0; j <= steps; j++)
                {
                    reachSamples++;
                    Pulse();
                    var point = math.lerp(vectorA, vectorB, j / (float)steps);

                    if (MappedDry(point))
                        continue;

                    var area =
                        areaIndex
                            .Nearby(point)
                            .FirstOrDefault(
                                otherArea => Flowing(otherArea)
                                    && !Coastal(otherArea)
                                    && !Dry(otherArea)
                                    && otherArea.Contains(point));

                    // Do not silently inflate a known 4m canal to the native
                    // 56m source diameter, or reinterpret an island as unmapped water.
                    Func<float3, bool> mask =
                        area != null
                            ? otherPoint => area.Contains(otherPoint) && !MappedDry(otherPoint)
                            : otherPoint => !MappedDry(otherPoint)
                                && NetworkNormalizer.DistanceToSegment(otherPoint, vectorA, vectorB) <= reach;

                    var banks = new List<float3>();
                    var bankSides = 0;

                    foreach (var side in new[] { -1, 1 })
                    {
                        var distance = Math.Max(cell, reach);

                        while (distance < 512 && mask(point + normal * (distance * side)))
                            distance += Math.Max(cell, reach * .5f);

                        var bankPoint = DryProbe(point, normal * side, mask, distance);

                        if (bankPoint.HasValue)
                        {
                            banks.Add(bankPoint.Value);
                            bankSides++;
                        }

                        // DryProbe deliberately clears a full native footprint.
                        // On a narrow urban stream it can land on a high terrace
                        // beyond a low near bank. That terrace cannot contain the
                        // water: constrain the stage at the actual cut's rim too.
                        if (area == null)
                        {
                            var rim = point + normal * (side * (channel / .6f + cell * .5f));

                            if (Inside(rim)
                                && !areaIndex.Nearby(rim).Any(otherArea => !Dry(otherArea) && otherArea.Contains(rim)))
                                banks.Add(rim);
                        }
                    }

                    var node = j == 0 && low <= 0 ? way.NodeIds[i - 1] : j == steps && high >= 1 ? way.NodeIds[i] : 0;
                    samples.Add(
                        new(
                            point,
                            vectorA,
                            vectorB,
                            j < steps ? math.lerp(vectorA, vectorB, (j + 1) / (float)steps) : null,
                            station + length * j / steps,
                            node,
                        bankSides < 2 ? BankHeight(point) : banks.Min(BankHeight),
                            banks,
                            area,
                            reach,
                            channel,
                            sourceWidth,
                            bankSides < 2));
                }

                station += length;
            }

            if (samples.Count == 0)
                continue;

            // The surface (bridged bound, running minimum, sea floor).
            var canal = NetworkSelection.HasTag(way.Tags, "waterway", "canal");
            var bridge = canal ? CanalBridge : riverBridge;
            var leveeMax = canal ? CanalLeveeMax : riverLeveeMax;
            var count = samples.Count;
            var bound =
                samples
                    .Select(
                        sample => (canal ? Math.Max(sample.Bank, BankHeight(sample.P)) : sample.Bank) - RiverFreeboard)
                    .ToArray();

            var dilated = new float[count];
            var surface = new float[count];

            for (int k = 0, low = 0, high = 0; k < count; k++)
            {
                while (samples[k].Station - samples[low].Station > bridge / 2)
                    low++;

                while (high + 1 < count && samples[high + 1].Station - samples[k].Station <= bridge / 2)
                    high++;

                var value = float.NegativeInfinity;

                for (var q = low; q <= high; q++)
                    value = Math.Max(value, bound[q]);

                dilated[k] = value;
            }

            for (int k = 0, low = 0, high = 0; k < count; k++)
            {
                while (samples[k].Station - samples[low].Station > bridge / 2)
                    low++;

                while (high + 1 < count && samples[high + 1].Station - samples[k].Station <= bridge / 2)
                    high++;

                var value = float.PositiveInfinity;

                for (var q = low; q <= high; q++)
                    value = Math.Min(value, dilated[q]);

                surface[k] = Math.Min(value, bound[k] + leveeMax);

                // Preserve the original whole-way running minimum even when
                // clipped/cyclic evidence leaves a junction piece disconnected
                // from the last sampled endpoint of the preceding piece.
                if (waySurface.TryGetValue(way.Id, out var previousSurface))
                    surface[k] = Math.Min(surface[k], previousSurface);

                if (samples[k].Node != 0 && nodeSurface.TryGetValue(samples[k].Node, out var arriving))
                    surface[k] = Math.Min(surface[k], arriving);

                if (k > 0)
                    surface[k] = Math.Min(surface[k], surface[k - 1]);
            }

            var seaBound = toSea.Contains(way.NodeIds[way.NodeIds.Length - 1]);

            for (var k = 0; k < count; k++)
            {
                if (seaBound && surface[k] < seaSurface)
                {
                    surface[k] = seaSurface;
                    seaRaisedSamples++;
                }

                if (surface[k] > bound[k] + .01f)
                    bridgedSamples++;
            }

            for (var k = 0; k < count; k++)
            {
                var sample = samples[k];
                var point = sample.P;
                var vectorA = sample.A;
                var vectorB = sample.B;
                var area = sample.Area;
                var stage = surface[k];
                var banks = sample.Banks;
                var bankless = sample.Bankless;
                Func<float3, bool> mask =
                    area != null
                        ? otherPoint => area.Contains(otherPoint) && !MappedDry(otherPoint)
                        : otherPoint => !MappedDry(otherPoint)
                            && NetworkNormalizer.DistanceToSegment(otherPoint, vectorA, vectorB) <= reach;

                // The carved channel and its sloping banks (ChannelDepth note).
                // A mapped river polygon narrower than that channel gets the
                // channel too: the Huveaune's, under 20 m wide on a 14 m
                // grid, kept only the few grid points inside it, and its bed
                // stood near ground level between them.
                // When a river polygon supplies actual banks, condition that
                // mapped wet cross-section instead of carving a 32m ribbon
                // down the middle of a several-hundred-metre-wide river.
                var wetRadius =
                    area != null && !bankless
                        ? Math.Min(512, banks.Min(otherPoint => math.distance(otherPoint.xz, point.xz))) * .95f
                        : channel;

                // The banks reach out to the upper bank's outer edge (CutChannel), measured from
                // the carved channel: a wide mapped polygon keeps its own shore shaping.
                // Sized for the highest ground it has to meet: the bank probes, and the ground
                // where a bank of full width would end on either side (a hill keeps rising).
                var sideways = math.normalizesafe(new float3(-(vectorB - vectorA).z, 0, (vectorB - vectorA).x));
                var upperBank =
                    banks
                        .Append(point + sideways * (channel / .6f + MaximumBankWidth))
                        .Append(point - sideways * (channel / .6f + MaximumBankWidth))
                        .Where(Inside)
                        .Select(BankHeight)
                        .DefaultIfEmpty(sample.Bank)
                        .Max();
                var bankReach =
                    channel / .6f
                        + math.clamp((upperBank - stage - RiverFreeboard) / BankSlope, cell, MaximumBankWidth);

                Func<float3, bool> channelMask =
                    area != null
                        ? otherPoint => mask(otherPoint)
                            || !MappedDry(otherPoint)
                                && NetworkNormalizer.DistanceToSegment(otherPoint, vectorA, vectorB) <= bankReach
                        : otherPoint => !MappedDry(otherPoint)
                            && NetworkNormalizer.DistanceToSegment(otherPoint, vectorA, vectorB) <= bankReach;

                var floor = area != null ? .4f : Math.Max(.4f, Height(point) - ChannelCut + .4f);

                if (stage < floor)
                {
                    unresolved++;
                    skippedTooDeep++;
                    log.Count("Water locally infeasible reaches");
                    var target = Math.Min(stage - ChannelDepth, bedLimit);
                    CutChannel(
                        point,
                        channel,
                        target,
                        stage + RiverFreeboard,
                        upperBank,
                        channelMask,
                        area != null ? float.PositiveInfinity : ChannelCut);

                    bedLimit = Math.Max(Height(point), target);
                    continue;
                }

                // A bridged dip, or a sea-level floor above low land, gets a
                // levee RiverFreeboard above the water; its crest stands in
                // for the bank probes.
                var bank = sample.Bank;

                if (!bankless && stage > bank - RiverFreeboard)
                {
                    bank = stage + RiverFreeboard;
                    levees.Add((vectorA, vectorB, point, channel, bank));
                    var across =
                        math.normalizesafe(new float3(-(vectorB - vectorA).z, 0, (vectorB - vectorA).x))
                            * (channel / .6f + cell * .5f);

                    banks = new[] { point + across, point - across };
                    bankless = false;
                }

                placedSamples++;
                var stagePixel = Pixel(point);
                riverSurface[stagePixel] =
                    riverSurface[stagePixel] > 0 ? Math.Min(riverSurface[stagePixel], stage) : stage;

                // Publish stage evidence even when a native source is subgrid.
                if (sample.Next is float3 next)
                {
                    var nextLevel = k + 1 < count ? Math.Min(stage, surface[k + 1]) : stage;
                    var candidate =
                        new WaterReachStage(body, way.Id, point, next, stage, nextLevel, reach, area?.Polygon.Identity);

                    var last = reachStages.Count > 0 ? reachStages[reachStages.Count - 1] : null;

                    if (last != null
                        && last.WayId == candidate.WayId
                        && last.PolygonId == candidate.PolygonId
                        && last.BlockedByBuriedInlet == candidate.BlockedByBuriedInlet
                        && math.distance(last.End.xz, point.xz) < .01f
                        && math.dot(
                            math.normalizesafe((last.End - last.Start).xz),
                            math.normalizesafe((next - point).xz))
                            > .99999f
                            && Math.Abs(
                            (last.EndLevel - last.StartLevel) / Math.Max(.1f, math.distance(last.Start.xz, last.End.xz))
                                - (nextLevel - stage) / Math.Max(.1f, math.distance(point.xz, next.xz)))
                            < .0001f)
                        reachStages[reachStages.Count - 1] = last with { End = next, EndLevel = nextLevel };
                    else
                        reachStages.Add(candidate);
                }

                var bedTarget = Math.Min(stage - ChannelDepth, bedLimit);
                CutChannel(
                    point,
                    Math.Max(wetRadius, channel),
                    bedTarget,
                    stage + RiverFreeboard,
                    upperBank,
                    channelMask,
                    area != null ? float.PositiveInfinity : ChannelCut);

                bedLimit = Math.Max(Height(point), bedTarget);

                if (bankless)
                {
                    unresolved++;
                    skippedNoBanks++;
                    continue;
                }

                // A stretch at the sea's level is the sea's: the sea fills it, and a
                // source there only adds work for the simulation.
                if (seaBound && stage <= seaSurface + .05f)
                {
                    seaLevelSamples++;
                    continue;
                }

                // Sources stand 192 m apart on every waterway. Narrow streams had 384 m, and
                // the game's water sags between level sources: on 11-13 a stretch 230 m from
                // the nearest source was dry in Marseille's Arc (report 115).
                if (NearControl(body, point))
                    continue;

                // A source's footprint is the carved channel's flat bed, not the
                // mapped width: a stream's 6 m half width held no source at all,
                // so every stream stayed a dry trench.
                Func<float3, bool> channelFootprint =
                    otherPoint => !MappedDry(otherPoint)
                        && NetworkNormalizer.DistanceToSegment(otherPoint, vectorA, vectorB) <= channel;

                var name = way.Tags.TryGetValue("name", out var mapped) ? mapped : "waterway";
                var probes = new[] { point, math.lerp(point, vectorB, .5f), vectorB };

                // A mapped river polygon holds a wide source where one fits inside it.
                // Otherwise, and on every waterway without a polygon, the source stands on the
                // water-grid node nearest the line, inside the carved channel, with a radius
                // of 0.9 cell (GridSnapped). The former channel sources (0.7 of the channel,
                // under 20 m) often reached no grid node and stayed dry: 605 of DC's 1,632 on
                // 11-13. A polygon river too narrow for its own source (Marseille's, 77% dry)
                // takes one in its carved channel instead of none.
                var placed =
                    area != null
                    && Add(
                        way.Id,
                        body,
                        "river",
                        name,
                        point,
                        Math.Max(cell, Math.Min(192, wetRadius * .55f)),
                        stage,
                        bank,
                        banks,
                        probes,
                        mask);

                if (!placed)
                {
                    var node = GridNode(point);

                    placed =
                        channelFootprint(node)
                        && Add(
                            way.Id,
                            body,
                            "river",
                            name,
                            node,
                            cell * .9f,
                            stage,
                            bank,
                            banks,
                            probes,
                            otherPoint => channelFootprint(otherPoint) || area != null && mask(otherPoint),
                            gridSource: true);
                }

                if (placed)
                {
                    var key = ((int)Math.Floor(point.x / 192), (int)Math.Floor(point.z / 192));

                    if (!controlGrid.TryGetValue(key, out var list))
                        controlGrid[key] = list = new();

                    list.Add(sources[sources.Count - 1]);
                }
            }

            var end = way.NodeIds[way.NodeIds.Length - 1];
            nodeSurface[end] =
                nodeSurface.TryGetValue(end, out var arrivingSurface)
                    ? Math.Min(arrivingSurface, surface[count - 1])
                    : surface[count - 1];

            waySurface[way.Id] = surface[count - 1];

            if (math.isfinite(bedLimit))
                wayBed[way.Id] = bedLimit;

            if (math.isfinite(bedLimit))
                nodeBed[end] = nodeBed.TryGetValue(end, out var downstream) ? Math.Min(downstream, bedLimit) : bedLimit;
        }

        log.Count("Water interior junction splits", junctionPieces - lines.Length);
        log.Diagnostic(
            $"WATER JUNCTION CONTINUITY mappedWays={lines.Length}; conditioningPieces={junctionPieces}; interiorJunctionsCarryStageAndBed=true; sourceWayIdsPreserved=true");

        // Levees go up after every channel is cut, never across a channel or
        // into mapped water: full height from the channel's cut out to a water
        // cell beyond it, then easing back to the ground over two more cells.
        var leveeCells = 0;

        foreach (var (vectorA, vectorB, point, channel, height) in levees)
        {
            var crest = channel / .6f + cell;
            var outer = crest + 2 * cell;
            var pixel = Pixel(point);
            var span = (int)Math.Ceiling(outer / spacing);

            for (var z = Math.Max(0, pixel / n - span); z <= Math.Min(n - 1, pixel / n + span); z++)
                for (var x = Math.Max(0, pixel % n - span); x <= Math.Min(n - 1, pixel % n + span); x++)
                {
                    var index = z * n + x;

                    if (channelCells[index] != 0)
                        continue;

                    var otherPoint = new float3(x * spacing - half, 0, z * spacing - half);
                    var distanceToSegment = NetworkNormalizer.DistanceToSegment(otherPoint, vectorA, vectorB);

                    if (distanceToSegment < channel
                        || distanceToSegment > outer
                        || math.distance(otherPoint.xz, point.xz) > outer)
                        continue;

                    if (areaIndex.Nearby(otherPoint).Any(area => !Dry(area) && area.Contains(otherPoint)))
                        continue;

                    var current = conditioned[index];
                    var t = Ramp(Math.Max(0, distanceToSegment - crest) / (outer - crest));
                    var target =
                        math.clamp(math.lerp(height, current, t), 0, terrain.HeightScaleMeters);

                    if (target <= conditioned[index])
                        continue;

                    conditioned[index] = target;
                    leveeCells++;

                    if (editMask[index] == 0)
                    {
                        editMask[index] = 1;
                        editCount++;
                    }
                }
        }

        // Lake shore levees (RaiseBanks): full height from the shore out past the shore
        // probes a source checks (two water cells), then easing back to the ground over
        // two more cells. Only dry ground outside every water body is raised.
        var shoreCells = 0;

        foreach (var (vectorA, vectorB, height) in shoreLevees)
        {
            var crest = 2.5f * cell;
            var outer = crest + 2 * cell;
            var low = math.min(vectorA.xz, vectorB.xz) - outer;
            var high = math.max(vectorA.xz, vectorB.xz) + outer;
            var x0 = Math.Max(0, (int)Math.Floor((low.x + half) / spacing));
            var x1 = Math.Min(n - 1, (int)Math.Ceiling((high.x + half) / spacing));
            var z0 = Math.Max(0, (int)Math.Floor((low.y + half) / spacing));
            var z1 = Math.Min(n - 1, (int)Math.Ceiling((high.y + half) / spacing));

            for (var z = z0; z <= z1; z++)
                for (var x = x0; x <= x1; x++)
                {
                    var index = z * n + x;
                    var otherPoint = new float3(x * spacing - half, 0, z * spacing - half);
                    var distanceToSegment = NetworkNormalizer.DistanceToSegment(otherPoint, vectorA, vectorB);

                    if (distanceToSegment > outer || channelCells[index] != 0)
                        continue;

                    var current = conditioned[index];
                    var t = Ramp(Math.Max(0, distanceToSegment - crest) / (outer - crest));
                    var target =
                        math.clamp(math.lerp(height, current, t), 0, terrain.HeightScaleMeters);

                    if (target <= conditioned[index]
                        || areaIndex.Nearby(otherPoint).Any(area => !Dry(area) && area.Contains(otherPoint)))
                        continue;

                    conditioned[index] = target;
                    shoreCells++;

                    if (editMask[index] == 0)
                    {
                        editMask[index] = 1;
                        editCount++;
                    }
                }
        }

        log.Count("Water lakes held at their surface by shore levees", raisedLakes);
        log.Count("Water lake shore levee cells raised", shoreCells);
        log.Count("Water surface samples bridging a bank dip", bridgedSamples);
        log.Count("Water surface samples held at sea level", seaRaisedSamples);
        log.Count("Water sea-level samples left to the sea without a source", seaLevelSamples);
        log.Count("Water levee cells raised", leveeCells);
        log.Diagnostic(
            $"WATER SURFACE PROFILE monotoneDownstream=true; bridgeMeters={riverBridge}; leveeMaxMeters={riverLeveeMax}; bridgedSamples={bridgedSamples}; seaBoundNodes={toSea.Count}; seaSurface={seaSurface:R}; seaRaisedSamples={seaRaisedSamples}; leveeCells={leveeCells}");

        log.Evidence(
            $"Rivers: {riverCount:N0} waterways, {placedSamples:N0} channel samples accepted, {sources.Count:N0} water sources; carved without a source: {skippedTooDeep:N0} more than {ChannelCut:0} m below ground, {skippedNoBanks:N0} without two banks; skipped: {skippedAboveBank:N0} above their banks.",
            warning: skippedTooDeep + skippedAboveBank > placedSamples / 10);

        phase = "publishing conditioned water terrain";
        Pulse(true);
        // Multi-source propagation gives every mapped channel cell the nearby
        // solved reach level in linear raster work, not a per-pixel scan of all
        // river segments. Islands and disconnected polygons cannot be crossed.
        var wet = new byte[conditioned.Length];

        foreach (var area in areas.Where(area => Flowing(area) && !Coastal(area) && !Dry(area)))
        {
            var loX = Math.Max(0, (int)Math.Ceiling((area.Min.x + half) / spacing));
            var hiX = Math.Min(n - 1, (int)Math.Floor((area.Max.x + half) / spacing));
            var loZ = Math.Max(0, (int)Math.Ceiling((area.Min.y + half) / spacing));
            var hiZ = Math.Min(n - 1, (int)Math.Floor((area.Max.y + half) / spacing));

            for (var z = loZ; z <= hiZ; z++)
                for (var x = loX; x <= hiX; x++)
                {
                    if ((bedVisits++ & 4095) == 0)
                        Pulse();

                    var point = new float3(x * spacing - half, 0, z * spacing - half);

                    if (area.Contains(point) && !MappedDry(point))
                        wet[z * n + x] = 1;
                }
        }

        var riverQueue = new int[conditioned.Length];
        var head = 0;
        var tail = 0;

        for (var i = 0; i < wet.Length; i++)
        {
            if ((i & 16383) == 0)
                Pulse();

            if (wet[i] != 0 && riverSurface[i] > 0)
                riverQueue[tail++] = i;
            else
                riverSurface[i] = 0;
        }

        while (head < tail)
        {
            if ((head & 4095) == 0)
                Pulse();

            var i = riverQueue[head++];
            var x = i % n;
            var z = i / n;

            void Visit(int next)
            {
                if (next < 0 || wet[next] == 0 || riverSurface[next] > 0)
                    return;

                riverSurface[next] = riverSurface[i];
                riverQueue[tail++] = next;
            }

            Visit(x > 0 ? i - 1 : -1);
            Visit(x + 1 < n ? i + 1 : -1);
            Visit(z > 0 ? i - n : -1);
            Visit(z + 1 < n ? i + n : -1);

            // A diagonal raster contact is still a mapped channel contact.
            if (x > 0 && z > 0)
                Visit(i - n - 1);

            if (x + 1 < n && z > 0)
                Visit(i - n + 1);

            if (x > 0 && z + 1 < n)
                Visit(i + n - 1);

            if (x + 1 < n && z + 1 < n)
                Visit(i + n + 1);
        }

        // Seed disconnected raster fragments from their SAME geographic river
        // polygon's solved reach. Do not jump across islands or use another river.
        var reachesByPolygon =
            reachStages
                .Where(reach => reach.PolygonId.HasValue)
                .GroupBy(reach => reach.PolygonId!.Value)
                .ToDictionary(group => group.Key, group => group.ToArray());

        var additionalSeeds = 0;

        for (var i = 0; i < wet.Length; i++)
        {
            if ((i & 16383) == 0)
                Pulse();

            if (wet[i] == 0 || riverSurface[i] > 0)
                continue;

            var point = new float3(i % n * spacing - half, 0, i / n * spacing - half);
            var area =
                areaIndex
                    .Nearby(point)
                    .FirstOrDefault(
                        otherArea => Flowing(otherArea)
                            && !Coastal(otherArea)
                            && !Dry(otherArea)
                            && otherArea.Contains(point));

            if (area == null || !reachesByPolygon.TryGetValue(area.Polygon.Identity, out var reaches))
                continue;

            var best = float.PositiveInfinity;
            var level = 0f;

            foreach (var reach in reaches)
            {
                var d = reach.End.xz - reach.Start.xz;
                var t = math.clamp(math.dot(point.xz - reach.Start.xz, d) / Math.Max(1, math.lengthsq(d)), 0, 1);
                var distance = math.distancesq(point.xz, math.lerp(reach.Start.xz, reach.End.xz, t));

                if (distance >= best)
                    continue;

                best = distance;
                level = math.lerp(reach.StartLevel, reach.EndLevel, t);
            }

            if (level <= 0)
                continue;

            riverSurface[i] = level;
            riverQueue[tail++] = i;
            additionalSeeds++;

            while (head < tail)
            {
                var current = riverQueue[head++];
                var x = current % n;
                var z = current / n;

                void Visit(int next)
                {
                    if (next < 0 || wet[next] == 0 || riverSurface[next] > 0)
                        return;

                    riverSurface[next] = riverSurface[current];
                    riverQueue[tail++] = next;
                }

                Visit(x > 0 ? current - 1 : -1);
                Visit(x + 1 < n ? current + 1 : -1);
                Visit(z > 0 ? current - n : -1);
                Visit(z + 1 < n ? current + n : -1);
            }
        }

        log.Count("River raster components reseeded", additionalSeeds);

        foreach (var area in areas.Where(area => Flowing(area) && !Coastal(area) && !Dry(area)))
            CarveArea(area, point => riverSurface[Pixel(point)] > 0 ? riverSurface[Pixel(point)] : (float?)null);

        log.Diagnostic(
            $"WATER CHANNEL CONTINUITY mappedWetCells={wet.Count(v => v != 0)}; solvedWetCells={tail}; disconnectedUnseededCells={wet.Count(v => v != 0) - tail}; islandsPreserved=true; sourceLevelPropagation=nearestConnectedRasterSeed; quadraticReachSearch=false");

        log.Count("Water shore bank samples shaped", shapedBank);
        log.Diagnostic(
            $"WATER BANKFULL PROFILE targetBankFreeboardMeters=.35; bankTerrainShapedOnlyInShoreBand=true; shoreBankSamplesShaped={shapedBank}; nonCumulativeSampleBounds=true; initialCenterlineOffsetMeters=2.8; fullWidthDepthMeters=2.5; widthAwareSourceRadius=true; mappedRiverBedBudget=maskOwned; finalNodes={nodeStage.Count}; minimumFinalStage={nodeStage.Values.DefaultIfEmpty(0).Min():R}; maximumFinalStage={nodeStage.Values.DefaultIfEmpty(0).Max():R}");

        // River cross-sections were already stamped at <= half DEM spacing or
        // half mapped width. Do not perform a second polygon-wide nearest-reach
        // search per raster cell: it can turn dense hydrology into quadratic work.
        log.Diagnostic(
            $"WATER RIVER OFFSET requestedMeters=1; controls={sources.Count(river => river.BodyKind == "river")}; appliedOnceBeforeSafetyChecks=true; lakeAndSeaOffsets=0; bankLimitedNodeUpdates={bankLimitedNodes}; fullOffsetNotGuaranteed=true; bankConflictsRejectControls=true");

        // Control hollows are clipped by the wet mask, which recreates a one-pixel
        // cliff at the shore, and a source DEM deeper than the profile near its
        // bank leaves one too. Within the bed band, the bed is raised to the
        // shore profile, except on the bilinear neighbourhood of each control,
        // whose planned depth must hold.
        var controlCells = new HashSet<int>();

        foreach (var river in sources)
        {
            var pixel = Pixel(river.Position);

            for (var dz = -1; dz <= 1; dz++)
                for (var dx = -1; dx <= 1; dx++)
                {
                    var x = pixel % n + dx;
                    var z = pixel / n + dz;

                    if (x >= 0 && x < n && z >= 0 && z < n)
                        controlCells.Add(z * n + x);
                }
        }

        var easedShore = 0;

        for (var i = 0; i < shoreFloor.Length; i++)
        {
            if (float.IsNaN(shoreFloor[i]) || controlCells.Contains(i) || channelCells[i] != 0)
                continue;

            var floor = math.clamp(shoreFloor[i], 0, terrain.HeightScaleMeters);

            if (floor > conditioned[i])
            {
                conditioned[i] = floor;
                easedShore++;
            }
        }

        log.Count("Water shore bed samples eased", easedShore);
        phase = "smoothing newly carved banks";
        Pulse(true);
        var smoothingPlan = new WaterPlan(false, riverCount, lakeCount, unresolved, "")
        { Sources = sources, AreaStages = areaStages, ReachStages = reachStages, WaterCellSize = cell };

        var cliffs = data.Ways.Values.Where(way => NetworkSelection.HasTag(way.Tags, "natural", "cliff"))
            .Select(way => way.NodeIds.Where(data.Nodes.ContainsKey).Select(Local).ToArray())
                .Where(points => points.Length > 1)
                .ToArray();

        WaterCutSmoothing.Apply(
            terrain.Heights,
            conditioned,
            editMask,
            channelCells,
            controlCells,
            width,
            ConstructedSurfaceSampler(smoothingPlan, polygons), cliffs, log, token, () => Pulse());

        // A bank probe that this planning lowered (into a carved channel such
        // as a lake's outlet stream, another body's bed, or a shaped shore) or
        // that lies inside other mapped water reads planned water, not the
        // ground that holds this body in; water reaching it stays in a planned
        // channel. Revalidate then judges only ground that later earthworks
        // lowered. Marseille's and DC's lakes lost their controls to probes in
        // the trench of a stream running past or out of them. A control left
        // with no probe keeps its planning bank level.
        bool PlannedWater(float3 point)
            => Height(point) < BankHeight(point) - .25f
                || areaIndex.Nearby(point).Any(area => !Dry(area) && area.Contains(point));

        for (var i = 0; i < sources.Count; i++)
        {
            var kept = sources[i].BankProbes.Where(point => !PlannedWater(point)).ToArray();

            if (kept.Length < sources[i].BankProbes.Count)
            {
                sources[i] = sources[i] with { BankProbes = kept };
                log.Count("Water controls with bank probes on planned water");
            }
        }

        log.Diagnostic(
            $"WATER SHORE PROFILE bedBandMeters={bedBand:0.#}; bankBandMeters={shoreBand:0.#}; ramp=smoothstep; bankSamplesShaped={shapedBank}; lowBankSamplesRaised={raisedBank};bedSamplesEasedFromControlHollows={easedShore}; controlNeighbourhoodsKept={controlCells.Count}");

        TerrainArtifactRepair.TraceHeights(
            conditioned,
            terrain.SeaLevelMeters,
            "after-water-bed-planning",
            log.Diagnostic);

        var coastal = data.Ways.Values.Any(way => NetworkSelection.HasTag(way.Tags, "natural", "coastline"));
        var report =
            $"Water: {sources.Count} bank-limited river/lake controls; {lakeCount} mapped lakes; {unresolved} unresolved narrow/height sites; {buried.Length} mapped buried waterways opened as surface channels. No flooding bypass. Coverage requires simulation verification.";

        log.Info(report);
        log.Diagnostic(
            $"WATER BODY SUMMARY bodies={sources.Select(river => river.BodyId).Distinct().Count()}; controls={sources.Count}; bedEdits={editCount}; cellMeters={cell:R}; mappedRiverBankWidthUsed=true; polygonBedCutsMaskOwned=true; fixedDepthFallback=false; floodingAccepted=false; unresolved={unresolved}; indexedPolygonPredicates=true; bedCellsVisited={bedVisits}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.###}; canonicalAreas={areaStages.Count}; canonicalReaches={reachStages.Count}; maskBytes={editMask.Length}; geometryVersion={WaterPlan.GeometryVersion}");

        var plan =
            new WaterPlan(
                coastal,
                riverCount,
                lakeCount,
                unresolved,
                report)
            {
                Sources = sources,
                ConditionedHeights = editCount > 0 ? conditioned : null,
                PlannedBedHeights = conditioned,
                WaterCellSize = cell,
                AreaStages = areaStages,
                ReachStages = reachStages,
                BedEdits = new TerrainEditSet("water-bed", "pre-water-terrain", n, editMask)
            };

        var contained =
            WaterContainment.Validate(plan, terrain with { Heights = conditioned }, width, polygons, log, token);

        // A rejected lake whose low rim is short is held by an earth dam: the real dam
        // is usually too narrow for the elevation data. Any other rejected lake gives
        // its terrain back, so it leaves neither a dry pit nor a wall.
        IEnumerable<Area> Rejected(WaterPlan checkedPlan)
        {
            var kept = checkedPlan.AreaStages.Select(stage => stage.BodyId).ToHashSet();
            return areaStages
                .Where(stage => stage.Kind == "lake" && !kept.Contains(stage.BodyId))
                .SelectMany(stage => areas.Where(area => area.Polygon.Identity == stage.BodyId));
        }

        var dams = 0;

        foreach (var area in Rejected(contained).ToArray())
        {
            token.ThrowIfCancellationRequested();
            var stage = areaStages.First(otherStage => otherStage.BodyId == area.Polygon.Identity).Level;
            var pad = (int)Math.Ceiling((cell * 2 + spacing) / spacing);
            var rim = new List<int>();

            for (var z =
                Math.Max(
                    0,
                    (int)((area.Min.y + half) / spacing)
                        - pad); z <= Math.Min(n - 1, (int)((area.Max.y + half) / spacing) + pad); z++)
                for (var x =
                    Math.Max(
                        0,
                        (int)((area.Min.x + half) / spacing)
                            - pad); x <= Math.Min(n - 1, (int)((area.Max.x + half) / spacing) + pad); x++)
                {
                    var point = new float3(x * spacing - half, 0, z * spacing - half);

                    if (area.Contains(point))
                        continue;

                    var edge = area.EdgeDistance(point, cell * 2, out _);

                    if (edge <= cell || float.IsPositiveInfinity(edge) || edge > cell * 2)
                        continue;

                    if (conditioned[z * n + x] < stage
                        && channelCells[z * n + x] == 0)
                        rim.Add(z * n + x);
                }

            if (rim.Count == 0 || rim.Count * spacing > 2 * MaximumDamLength)
                continue;

            var crest =
                math.clamp(stage + LeveeFreeboard, 0, terrain.HeightScaleMeters);

            foreach (var i in rim)
            {
                conditioned[i] = crest;
                editMask[i] = 1;
            }

            dams++;
        }

        log.Count("Water earth dams built", dams);

        if (dams > 0)
            contained =
                WaterContainment.Validate(plan, terrain with { Heights = conditioned }, width, polygons, log, token);

        var restored = 0;

        foreach (var area in Rejected(contained))
        {
            var pad = (int)Math.Ceiling((cell * 2 + shoreBand) / spacing);

            for (var z =
                Math.Max(
                    0,
                    (int)((area.Min.y + half) / spacing)
                        - pad); z <= Math.Min(n - 1, (int)((area.Max.y + half) / spacing) + pad); z++)
                for (var x =
                    Math.Max(
                        0,
                        (int)((area.Min.x + half) / spacing)
                            - pad); x <= Math.Min(n - 1, (int)((area.Max.x + half) / spacing) + pad); x++)
                {
                    var i = z * n + x;

                    if (channelCells[i] != 0 || conditioned[i] == terrain.Heights[i])
                        continue;

                    var point = new float3(x * spacing - half, 0, z * spacing - half);

                    if (!area.Contains(point)
                        && float.IsPositiveInfinity(area.EdgeDistance(point, cell * 2 + shoreBand, out _)))
                        continue;

                    if (areaIndex.Nearby(point).Any(otherArea => otherArea != area && otherArea.Contains(point)))
                        continue;

                    conditioned[i] = terrain.Heights[i];
                    restored++;
                }
        }

        log.Count("Water terrain cells restored from rejected lakes", restored);

        return contained;
    }

    private const float MaximumDamLength = 250;

    public static WaterPlan Revalidate(
        WaterPlan plan,
        TerrainBuilder.Result terrain,
        double width,
        ImportLog log,
        CancellationToken token,
        IReadOnlyList<PreparedSegment>? networks = null,
        Action<string>? progress = null)
    {
        token.ThrowIfCancellationRequested();

        if (plan.Sources.Count == 0 && plan.BedEdits == null)
        {
            // No controls can request a new cut. The preparation raster is
            // already applied upstream; never reuse it over newer roadbeds.
            log.Diagnostic(
                "WATER FINAL REVALIDATION controls=0; noAcceptedControls=true; networkProtectionRasterizationSkipped=true; finalTerrainRetained=true");

            return plan with { ConditionedHeights = null };
        }

        var clock = Stopwatch.StartNew();
        var lastPulse = -5d;
        var networkCount = 0;
        var sourceCount = 0;
        long protectionVisits = 0;

        void Pulse(bool force = false)
        {
            token.ThrowIfCancellationRequested();

            if (!force && clock.Elapsed.TotalSeconds - lastPulse < 5)
                return;

            lastPulse = clock.Elapsed.TotalSeconds;
            progress?.Invoke(
                $"Checking final water terrain: {networkCount}/{networks?.Count ?? 0} network sections; {sourceCount}/{plan.Sources.Count} controls");

            log.Diagnostic(
                $"WATER FINAL PROGRESS networks={networkCount}/{networks?.Count ?? 0}; controls={sourceCount}/{plan.Sources.Count}; protectionCellsVisited={protectionVisits}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.###}; cancellationChecked=true");
        }

        Pulse(true);
        var raster = (float[])terrain.Heights.Clone();
        var n = (int)Math.Sqrt(raster.Length);
        var cell = (float)width / (n - 1);
        var half = (float)width * .5f;

        float Height(float3 point)
            => NetworkProcessor.Sample(point.x, point.z, raster, width);

        int Pixel(float value) => (int)Math.Round((value / width + .5) * (n - 1));

        // Recovery is confined to previously accepted control footprints. Never
        // carve through a road bed, a bridge support envelope or a tunnel roof.
        var protectedCells = new byte[raster.Length];
        var finalEdits = new byte[raster.Length];

        foreach (var segment in (networks
            ?? Array.Empty<PreparedSegment>()).Where(otherSegment => !otherSegment.Bridge && !otherSegment.Tunnel))
        {
            networkCount++;
            Pulse();
            var count =
                Math.Max(1, (int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(segment) / Math.Max(1, cell * .5f)));

            var radius = Math.Max(4, segment.NativeWidth * .5f) + cell * 2;
            var reach = (int)Math.Ceiling(radius / cell);

            for (var i = 0; i <= count; i++)
            {
                Pulse();
                var point = NetworkNormalizer.Evaluate(segment, i / (float)count);
                var px = Pixel(point.x);
                var pz = Pixel(point.z);

                for (var z = Math.Max(0, pz - reach); z <= Math.Min(n - 1, pz + reach); z++)
                    for (var x = Math.Max(0, px - reach); x <= Math.Min(n - 1, px + reach); x++)
                    {
                        if ((protectionVisits++ & 4095) == 0)
                            Pulse();

                        if (math.distance(new float2(x * cell - half, z * cell - half), point.xz) <= radius)
                            protectedCells[z * n + x] = 1;
                    }
            }
        }

        var revised = new List<RiverSourcePlan>();
        var adjusted = 0;
        var repaired = 0;
        var blocked = 0;
        var owned = plan.BedEdits;
        var plannedBed = plan.PlannedBedHeights;
        var supportBlocked = 0;

        if (owned != null
            && plannedBed != null
            && owned.Mask.Length == raster.Length
            && plannedBed.Length == raster.Length)
            for (var index = 0; index < raster.Length; index++)
            {
                if ((index & 4095) == 0)
                    Pulse();

                if (owned.Mask[index] == 0 || raster[index] <= plannedBed[index])
                    continue;

                if (protectedCells[index] != 0)
                {
                    supportBlocked++;
                    continue;
                }

                var cut = raster[index] - plannedBed[index];
                // This restores the exact previously accepted water-owned bed,
                // not a new unbounded cut or a change to islands/infrastructure.
                raster[index] = plannedBed[index];
                finalEdits[index] = 1;
                repaired++;
            }

        // A bank probe may stand within .25 m of the stage on the planned terrain
        // itself, where the shore was shaped for that stage (a levee rising from
        // the waterline). Only a bank that later earthworks lowered, or one
        // never above the stage, is a conflict. Such a control then targets
        // .35 m below that bank when that lowers it by at most StageDrop and
        // still leaves water over its bed: rejected outright, a lake whose
        // controls all share one low shore probe stayed a dry carved pit
        // (93 of Marseille's 311 sourced lakes). The canonical stage is kept
        // for bridge clearance, which a lower surface only makes safer.
        const float StageDrop = 1;
        var plannedBanks =
            plan.PlannedBedHeights != null && plan.PlannedBedHeights.Length == raster.Length
                ? plan.PlannedBedHeights
                : null;

        float Planned(float3 point)
            => NetworkProcessor.Sample(point.x, point.z, plannedBanks!, width);

        var lowered = 0;

        foreach (var source in plan.Sources)
        {
            sourceCount++;
            Pulse();
            var point = source.Position;
            point.y = Height(point);
            var bank = source.BankProbes.Count > 0 ? source.BankProbes.Min(Height) : source.BankLevel;
            var stage = source.TargetSurfaceLevel ?? source.Position.y + source.Depth;
            var planned =
                plannedBanks != null && source.BankProbes.Count > 0
                    ? source.BankProbes.Min(Planned)
                    : float.NegativeInfinity;

            if (stage > bank - .25f && (planned < stage + .05f || bank < planned - .05f))
            {
                if (stage - (bank - .35f) <= StageDrop && bank - .35f - point.y >= .4f)
                {
                    stage = bank - .35f;
                    lowered++;
                    log.Count("Water final stages lowered to their banks");
                }
                else
                {
                    log.Count("Water final bank conflicts");
                    log.Diagnostic(
                        $"WATER FINAL UNRESOLVED body={source.BodyId}; way={source.WayId}; stage={stage:R}; bank={bank:R}; reason=final-bank-lowered; canonicalStageRetained=true; controlRejected=true");

                    continue;
                }
            }

            // Repair only modest new obstructions. Larger or infrastructure-owned
            // changes need a combined regional redesign, not a flooding source.
            foreach (var index in source.BedRepairCells)
            {
                token.ThrowIfCancellationRequested();

                if (index < 0 || index >= raster.Length || protectedCells[index] != 0)
                    continue;

                var otherPoint = new float3((index % n) * cell - half, 0, (index / n) * cell - half);

                if (source.BankProbes.Any(b => math.distance(b.xz, otherPoint.xz) < cell * 2))
                    continue;

                var old = raster[index];
                var target = stage - .6f;

                if (old <= target)
                    continue;

                if (owned == null || owned.Mask.Length != raster.Length || owned.Mask[index] == 0)
                {
                    blocked++;
                    continue;
                }

                raster[index] = math.clamp(target, 0, terrain.HeightScaleMeters);
                finalEdits[index] = 1;
                repaired++;
            }

            point.y = Height(point);

            if (stage - point.y < .2f)
            {
                log.Diagnostic(
                    $"WATER FINAL UNRESOLVED body={source.BodyId}; way={source.WayId}; bed={point.y:R}; stage={stage:R}; reason=earthworks-blocked-control; noFloodBypass=true");

                continue;
            }

            if (Math.Abs(stage - (source.TargetSurfaceLevel ?? stage)) > .01f
                || Math.Abs(point.y - source.Position.y) > .01f)
                adjusted++;

            revised.Add(
                source with
                {
                    Position = point,
                    Depth = stage - point.y,
                    TargetSurfaceLevel = stage,
                    BankLevel = bank,
                    ChannelProbes =
                        source.ChannelProbes
                            .Select(otherPoint => new float3(otherPoint.x, Height(otherPoint), otherPoint.z))
                            .ToArray()
                });
        }

        log.Diagnostic(
            $"WATER FINAL REVALIDATION controls={revised.Count}/{plan.Sources.Count}; adjusted={adjusted}; loweredToBank={lowered}; repairedBedSamples={repaired}; nonOwnedObstructionsBlocked={blocked}; supportBlockedSamples={supportBlocked}; restoresAcceptedOwnedBedWithoutTwoMeterCap=true; infrastructureAndBanksProtected=true; lakesKeepOneStage=true; afterEarthworks=true; canonicalStagePreserved=true; recoveryUsesCompleteOwnedWetMask=true; protectionMaskBytes={protectedCells.Length}; producerMaskBytes={finalEdits.Length}");

        TerrainArtifactRepair.TraceHeights(
            raster,
            terrain.SeaLevelMeters,
            "after-water-final-revalidation",
            log.Diagnostic);

        return plan with
        {
            Sources = revised,
            ConditionedHeights = repaired > 0 ? raster : null,
            FinalBedEdits = new TerrainEditSet("water-final-bed", "post-roadbed-terrain", n, finalEdits),
            Skipped = plan.Skipped + plan.Sources.Count - revised.Count
        };
    }
}
