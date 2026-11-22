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

public static class ForestProcessor
{
    // Spatial bucket width for the spacing scan; any spacing up to this is covered.
    public const float MinimumSpacing = 12f;
    // Planting spacing for woodland, mapped street trees and tree rows. Woodland
    // density is otherwise set by one jittered candidate per WoodlandCellSize
    // cell (15 m: 1.44× the former 18 m cells), then thinned for character:
    // feathered edges in some places and patchy gaps (WoodlandKeep), about a
    // fifth fewer, so the total stays near the former count.
    public const float MappedSpacing = 6f;
    public const float WoodlandSpacing = 6f;
    public const float WoodlandCellSize = 15f;
    public const double WoodlandTimeLimitSeconds = 300;
    // Orchard rows run along the polygon's longest edge, MappedSpacing apart
    // within a row. Street trees stand StreetSpacing apart, StreetSetback
    // beyond the road's half width, where the vegetation clearance is 3 m.
    public const float OrchardRowSpacing = 7f;
    public const float StreetSpacing = 10f;
    public const float StreetSetback = 4f;
    // Guessed (inferred) trees, by habitat (report 121 §16.1). All together they added
    // 1.64M of DC's 4.1M trees and the editor crashed near 3.86M, so they come back one
    // habitat at a time, where the evidence is strongest:
    //   PlantOrchards   rows in mapped orchards (someone mapped trees standing there);
    //   PlantParkEdges  a row of trees along park edges (within one cell, 15 m); park
    //                   interiors can be plazas or playing fields, so they stay bare
    //                   until a survey shows where trees can stand;
    //   PlantInferredTrees  the rest: park interiors, golf, cemeteries, grounds,
    //                   gardens, avenues and streets, still off.
    // Switches, not constants, so the code they turn off still compiles without warnings.
    public static readonly bool PlantOrchards = true;
    public static readonly bool PlantParkEdges = true;
    public static readonly bool PlantInferredTrees = false;

    // Woodland infill (report 121 §16.2): inside a wood, at least EdgeCells cells from
    // its edge and outside a clearing, the keep threshold rises by this much on the same
    // draw, so a sparse wood fills in while its edges and clearings keep their character.
    public const float InteriorInfill = .15f;

    // Areas in ascending precedence where polygons overlap. Orchards and open
    // areas get no cell trees: orchards are planted in rows, and pitches,
    // parking and golf playing surfaces stay clear.
    internal enum AreaKind : byte
    {
        None,
        Residential,
        Grounds,
        Golf,
        Park,
        Cemetery,
        Woodland,
        Orchard,
        Open
    }

    private sealed record Mask(List<float3[]> Outer, List<float3[]> Inner, AreaKind Kind);

    private sealed record WoodlandRing(Mask Mask, RingIndex Points, float MinX, float MaxX, float MinZ, float MaxZ);

    private sealed record Hole(RingIndex Points, float MinX, float MaxX, float MinZ, float MaxZ);

    // A point-in-ring test that only visits the edges near the point's z. The ray cast in
    // Inside toggles on edges that straddle the point's z, so edges filed under other
    // bands can never toggle and skipping them gives the same answer. Large mapped
    // forests (Mexico City has one of 166,000 points) otherwise cost a full walk per tree.
    internal sealed class RingIndex
    {
        private const int DirectLimit = 256;
        private const float BandHeight = 32f;
        private readonly float3[] _points;
        private readonly float _minimumZ;
        private readonly int[][]? _bands;

        public RingIndex(float3[] points)
            : this(points, DirectLimit)
        {
        }

        // Rings of up to directLimit points are walked whole. Vegetation planning tests
        // most points against several habitats of a hundred or more points each, so it
        // bands smaller rings than tree selection does.
        public RingIndex(float3[] points, int directLimit)
        {
            _points = points;

            if (points.Length <= directLimit)
                return;

            _minimumZ = points.Min(point => point.z);
            var count = Band(points.Max(point => point.z)) + 1;
            var bands = new List<int>[count];

            for (var i = 0; i < points.Length; i++)
            {
                var vectorA = points[i];
                var vectorB = points[(i + 1) % points.Length];
                var last = Band(Math.Max(vectorA.z, vectorB.z));

                for (var band = Band(Math.Min(vectorA.z, vectorB.z)); band <= last; band++)
                    (bands[band] ??= new List<int>()).Add(i);
            }

            _bands = bands.Select(edges => edges?.ToArray() ?? Array.Empty<int>()).ToArray();
        }

        private int Band(float z) => (int)Math.Floor((z - _minimumZ) / BandHeight);

        public bool Contains(float3 point)
        {
            var inside = false;

            // Inside's walk, over the array directly rather than through IReadOnlyList,
            // which costs an interface call per point (far more on Mono).
            if (_bands == null)
            {
                for (var i = 0; i < _points.Length; i++)
                {
                    var vectorA = _points[i];
                    var vectorB = _points[(i + 1) % _points.Length];

                    if ((vectorA.z > point.z) != (vectorB.z > point.z)
                        && point.x
                            < (vectorB.x - vectorA.x) * (point.z - vectorA.z) / (vectorB.z - vectorA.z) + vectorA.x)
                        inside = !inside;
                }

                return inside;
            }

            var band = Band(point.z);

            if (band < 0 || band >= _bands.Length)
                return false;

            foreach (var i in _bands[band])
            {
                var vectorA = _points[i];
                var vectorB = _points[(i + 1) % _points.Length];

                if ((vectorA.z > point.z) != (vectorB.z > point.z)
                    && point.x < (vectorB.x - vectorA.x) * (point.z - vectorA.z) / (vectorB.z - vectorA.z) + vectorA.x)
                    inside = !inside;
            }

            return inside;
        }
    }

    // A tree category with its own counters, so the per-tree path neither builds string keys
    // nor looks them up; the counters join the named counts before reporting.
    private sealed class TreeCategory
    {
        public readonly string Name;
        public readonly int Priority;
        public int Candidates;
        public int Accepted;

        public TreeCategory(string name, int priority)
        {
            Name = name;
            Priority = priority;
        }
    }

    // A tree waiting for its turn in centre-out order. The squared distance is computed
    // once, exactly as math.lengthsq(point.xz), rather than on every comparison.
    private readonly struct PendingTree
    {
        public readonly float3 Point;
        public readonly TreeCategory Category;
        public readonly float DistanceSquared;
        public readonly int Priority;
        public readonly long Serial;

        public PendingTree(float3 point, TreeCategory category, long serial)
        {
            Point = point;
            Category = category;
            DistanceSquared = math.lengthsq(point.xz);
            Priority = category.Priority;
            Serial = serial;
        }

        public int CompareTo(in PendingTree other)
        {
            var compare = DistanceSquared.CompareTo(other.DistanceSquared);

            if (compare != 0)
                return compare;

            compare = Priority.CompareTo(other.Priority);

            return compare != 0 ? compare : Serial.CompareTo(other.Serial);
        }
    }

    // A binary min-heap of pending trees.
    private sealed class PendingTrees
    {
        private PendingTree[] _items = new PendingTree[1024];

        public int Count { get; private set; }

        public PendingTree Min => _items[0];

        public void Add(PendingTree item)
        {
            if (Count == _items.Length)
                Array.Resize(ref _items, _items.Length * 2);

            var child = Count++;

            while (child > 0)
            {
                var parent = (child - 1) / 2;

                if (_items[parent].CompareTo(item) <= 0)
                    break;

                _items[child] = _items[parent];
                child = parent;
            }

            _items[child] = item;
        }

        public PendingTree RemoveMin()
        {
            var minimum = _items[0];
            var last = _items[--Count];
            _items[Count] = default;
            var parent = 0;

            while (true)
            {
                var child = 2 * parent + 1;

                if (child >= Count)
                    break;

                if (child + 1 < Count && _items[child + 1].CompareTo(_items[child]) < 0)
                    child++;

                if (last.CompareTo(_items[child]) <= 0)
                    break;

                _items[parent] = _items[child];
                parent = child;
            }

            if (Count > 0)
                _items[parent] = last;

            return minimum;
        }
    }

    private sealed class WoodlandBlock
    {
        public int X, Z;
        public float MinimumDistanceSquared;
        public readonly List<WoodlandRing> Rings = new();
    }

    private readonly record struct Corridor(float3 A, float3 B, float Radius);

    internal sealed class CorridorIndex
    {
        private const float CellSize = 50f;
        private readonly Dictionary<(int, int), List<Corridor>> _cells = new();

        public long CandidateChecks { get; private set; }
        public long ExactChecks { get; private set; }

        public int CellCount => _cells.Count;

        public void Add(float3 vectorA, float3 vectorB, float radius, CancellationToken token)
        {
            var corridor = new Corridor(vectorA, vectorB, radius);

            for (var x =
                (int)Math.Floor(
                    (Math.Min(vectorA.x, vectorB.x) - radius)
                        / CellSize); x <= (int)Math.Floor((Math.Max(vectorA.x, vectorB.x) + radius) / CellSize); x++)
                for (var z =
                    (int)Math.Floor(
                        (Math.Min(vectorA.z, vectorB.z) - radius)
                            / CellSize); z
                                <= (int)Math.Floor((Math.Max(vectorA.z, vectorB.z) + radius) / CellSize); z++)
                {
                    token.ThrowIfCancellationRequested();

                    if (!_cells.TryGetValue((x, z), out var list))
                        _cells[(x, z)] = list = new List<Corridor>();

                    list.Add(corridor);
                }
        }

        public bool Contains(float3 point)
        {
            if (!_cells.TryGetValue(
                ((int)Math.Floor(point.x / CellSize), (int)Math.Floor(point.z / CellSize)),
                out var nearby))
                return false;

            foreach (var corridor in nearby)
            {
                CandidateChecks++;

                if (point.x < Math.Min(corridor.A.x, corridor.B.x) - corridor.Radius
                    || point.x > Math.Max(corridor.A.x, corridor.B.x) + corridor.Radius
                    || point.z < Math.Min(corridor.A.z, corridor.B.z) - corridor.Radius
                        || point.z > Math.Max(corridor.A.z, corridor.B.z) + corridor.Radius)
                    continue;

                ExactChecks++;

                if (NetworkNormalizer.DistanceToSegment(point, corridor.A, corridor.B) <= corridor.Radius)
                    return true;
            }

            return false;
        }
    }

    internal static AreaKind Kind(IReadOnlyDictionary<string, string> tags)
    {
        string Tag(string key) => tags.TryGetValue(key, out var value) ? value : "";

        var landuse = Tag("landuse");
        var leisure = Tag("leisure");
        var amenity = Tag("amenity");

        if (leisure is "pitch" or "playground" or "track" or "swimming_pool" or "stadium" || amenity == "parking"
            || Tag(
                "golf") is "fairway" or "green" or "tee" or "bunker" or "water_hazard" or "lateral_water_hazard" or "driving_range")
            return AreaKind.Open;

        if (landuse == "orchard")
            return AreaKind.Orchard;

        // OSM wetland=swamp and mangrove are wooded wetlands.
        if (Tag("natural") == "wood"
            || landuse == "forest"
            || Tag("landcover") == "trees"
            || Tag("wetland") is "swamp" or "mangrove")
            return AreaKind.Woodland;

        if (landuse == "cemetery" || amenity == "grave_yard")
            return AreaKind.Cemetery;

        if (leisure is "park" or "garden" || landuse is "recreation_ground" or "village_green")
            return AreaKind.Park;

        if (leisure == "golf_course")
            return AreaKind.Golf;

        if (amenity is "school" or "college" or "university" or "kindergarten" || landuse == "religious")
            return AreaKind.Grounds;

        return landuse == "residential" ? AreaKind.Residential : AreaKind.None;
    }

    public static IReadOnlyList<float3> Prepare(OsmDataset data, ImportRequest request, TerrainBuilder.Result terrain,
        IReadOnlyList<PreparedSegment> networks,
            ImportLog log,
            CancellationToken token,
            Func<float3, bool>? excludeWater = null,
            ClimateNormals? climate = null)
    {
        token.ThrowIfCancellationRequested();
        var timer = Stopwatch.StartNew();

        // Per-part times for the run report, recorded between parts, never per tree.
        var partTimer = Stopwatch.StartNew();

        void EndPart(string name)
        {
            log.AddDuration(name, partTimer.Elapsed.TotalSeconds);
            partTimer.Restart();
        }

        var masks = new List<Mask>();
        var members = new HashSet<(long, AreaKind)>();
        var incompleteAreas = 0;

        foreach (var relation in data.Relations
            .Select(relation => (relation, kind: Kind(relation.Tags)))
            .Where(item => item.kind != AreaKind.None)
            .OrderBy(item => item.relation.Id))
        {
            token.ThrowIfCancellationRequested();

            // A member way repeating its relation's kind is the same area.
            foreach (var member in relation.relation.Members)
                members.Add((member.wayId, relation.kind));

            try
            {
                masks.Add(
                    new Mask(
                        Rings(relation.relation.Members.Where(member => !member.inner).Select(member => member.wayId)),
                    Rings(relation.relation.Members.Where(member => member.inner).Select(member => member.wayId)),
                        relation.kind));
            }
            catch (InvalidOperationException exception)
            {
                if (relation.kind == AreaKind.Woodland)
                    log.Warn($"Skipped forest relation {relation.relation.Id}: {exception.Message}");
                else
                    incompleteAreas++;
            }
        }

        foreach (var (way, kind) in data.Ways.Values
            .Select(way => (way, kind: Kind(way.Tags)))
            .Where(item => item.kind != AreaKind.None && !members.Contains((item.way.Id, item.kind)))
            .OrderBy(item => item.way.Id))
        {
            token.ThrowIfCancellationRequested();

            // Pitches, tracks and golf holes are often mapped as open lines.
            if (kind != AreaKind.Woodland
                && (way.NodeIds.Length < 4 || way.NodeIds[0] != way.NodeIds[way.NodeIds.Length - 1]))
                continue;

            try
            {
                masks.Add(new Mask(Rings(new[] { way.Id }), new List<float3[]>(), kind));
            }
            catch (InvalidOperationException exception)
            {
                if (kind == AreaKind.Woodland)
                    log.Warn($"Skipped forest way {way.Id}: {exception.Message}");
                else
                    incompleteAreas++;
            }
        }

        List<float3[]> Rings(IEnumerable<long> ids)
        {
            var remaining = new SortedDictionary<long, long[]>();

            foreach (var id in ids)
            {
                if (!data.Ways.TryGetValue(id, out var way) || way.NodeIds.Any(node => !data.Nodes.ContainsKey(node)))
                    throw new InvalidOperationException("Incomplete forest polygon membership.");

                remaining[id] = way.NodeIds;
            }

            var rings = new List<float3[]>();

            while (remaining.Count > 0)
            {
                var first = remaining.First();
                remaining.Remove(first.Key);
                var ring = first.Value.ToList();

                while (ring.Count > 1 && ring[0] != ring[ring.Count - 1])
                {
                    token.ThrowIfCancellationRequested();
                    var end = ring[ring.Count - 1];
                    var choices =
                        remaining
                            .Where(item => item.Value[0] == end || item.Value[item.Value.Length - 1] == end)
                            .ToArray();

                    if (choices.Length != 1)
                        throw new InvalidOperationException("Open or ambiguous forest ring.");

                    var next = choices[0];
                    remaining.Remove(next.Key);
                    ring.AddRange((next.Value[0] == end ? next.Value : next.Value.Reverse()).Skip(1));
                }

                if (ring.Count < 4 || ring[0] != ring[ring.Count - 1])
                    throw new InvalidOperationException("Invalid forest ring.");

                var points = ring.Select(id =>
{
    var node = data.Nodes[id];
    return GeoMath.ToLocalMeters(node.Latitude, node.Longitude, request.Latitude, request.Longitude);
}).ToArray();

                if (points.Any(point => !math.all(math.isfinite(point))))
                    throw new InvalidOperationException("Invalid forest coordinates.");

                rings.Add(points);
            }

            return rings;
        }

        EndPart("Forest masks");
        var half = (float)(request.MapSize.WidthMeters / 2);
        // Woodland and other area trees keep the wide corridor. Trees that stand
        // beside a road (mapped, rows and street trees) need only the 3 m verge
        // clearance VegetationPlanner applies to every candidate.
        var corridors = new CorridorIndex();
        var verges = new CorridorIndex();

        foreach (var segment in networks)
        {
            token.ThrowIfCancellationRequested();

            if (segment.Tunnel)
                continue; // Surface vegetation above covered tunnels is legitimate.

            var radius =
                segment.NativeWidth > 0
                    ? segment.NativeWidth * .5f + 6
                    : segment.Kind == NetworkKind.Rail ? 12f : Math.Max(15f, segment.Lanes * 1.75f + 7f);

            var sections = Math.Max(1, (int)Math.Ceiling(math.distance(segment.Start, segment.End) / 10f));

            for (var i = 0; i < sections; i++)
            {
                var vectorA = NetworkNormalizer.Evaluate(segment, i / (float)sections);
                var vectorB = NetworkNormalizer.Evaluate(segment, (i + 1) / (float)sections);
                corridors.Add(vectorA, vectorB, radius + 2f, token);
                verges.Add(vectorA, vectorB, Math.Max(4, segment.NativeWidth * .5f) + 3, token);
            }
        }

        EndPart("Forest clearance index");

        var output = new List<float3>();
        var candidates = new List<(float3 point, string category)>();
        var spacing = new Dictionary<(int, int), List<float3>>();
        var explicitTrees = new Dictionary<(int, int), List<float3>>();
        var counts = new Dictionary<string, int>();
        var categories = new Dictionary<string, TreeCategory>();

        // Per-tree outcomes are counted here and join the named counts before reporting.
        var outsideOrInvalid = 0;
        var waterRejected = 0;
        var networkRejected = 0;
        var spacingRejected = 0;

        void Count(string key, int amount = 1)
        {
            counts.TryGetValue(key, out var value);
            counts[key] = value + amount;
        }

        TreeCategory CategoryNamed(string name)
        {
            if (!categories.TryGetValue(name, out var category))
                categories[name] = category = new TreeCategory(name, Priority(name));

            return category;
        }

        void Add(float3 point, string category) => candidates.Add((point, category));

        bool Accept(float3 point, TreeCategory category)
        {
            category.Candidates++;

            if (!math.all(math.isfinite(point)) || Math.Abs(point.x) > half || Math.Abs(point.z) > half)
            {
                outsideOrInvalid++;
                return false;
            }

            point.y =
                NetworkProcessor.Sample(
                    point.x,
                    point.z,
                    terrain.Heights,
                    request.MapSize.WidthMeters);

            if (!math.isfinite(point.y) || point.y <= WaterLevels.Effective(terrain.SeaLevelMeters) + 0.5f)
            {
                waterRejected++;
                return false;
            }

            if (excludeWater?.Invoke(point) == true)
            {
                waterRejected++;
                return false;
            }

            var name = category.Name;

            if ((name is "mappedTree" or "rowTree" or "streetTree" or "avenueTree"
                ? verges
                : corridors).Contains(point))
            {
                networkRejected++;
                return false;
            }

            // Buckets stay MinimumSpacing wide, so the 3×3 scan covers either radius.
            var minimum = name == "woodlandTree" ? WoodlandSpacing : MappedSpacing;
            var bucketX = (int)Math.Floor(point.x / MinimumSpacing);
            var bucketZ = (int)Math.Floor(point.z / MinimumSpacing);

            // Inferred trees give way to mapped trees and rows wherever they would
            // crowd one, even if they come first in the center-out order.
            if (name is not ("mappedTree" or "rowTree" or "woodlandTree"))
                for (var dx = -1; dx <= 1; dx++)
                    for (var dz = -1; dz <= 1; dz++)
                        if (explicitTrees.TryGetValue(BucketKey(bucketX + dx, bucketZ + dz), out var mapped)
                            && AnyCloser(mapped, point, MappedSpacing))
                        {
                            spacingRejected++;
                            return false;
                        }

            for (var dx = -1; dx <= 1; dx++)
                for (var dz = -1; dz <= 1; dz++)
                    if (spacing.TryGetValue(BucketKey(bucketX + dx, bucketZ + dz), out var neighbors)
                        && AnyCloser(neighbors, point, minimum))
                    {
                        spacingRejected++;
                        return false;
                    }

            var bucket = BucketKey(bucketX, bucketZ);

            if (!spacing.TryGetValue(bucket, out var bucketTrees))
                spacing[bucket] = bucketTrees = new List<float3>();

            bucketTrees.Add(point);
            output.Add(point);
            category.Accepted++;

            return true;
        }

        foreach (var tagged in data.NodeTags.OrderBy(pair => pair.Key))
        {
            token.ThrowIfCancellationRequested();

            if (!tagged.Value.TryGetValue("natural", out var natural)
                || natural != "tree"
                || !data.Nodes.TryGetValue(tagged.Key, out var node))
                continue;

            Add(
                GeoMath.ToLocalMeters(node.Latitude, node.Longitude, request.Latitude, request.Longitude),
                "mappedTree");
        }

        var rows =
            data.Ways.Values
                .Where(way => way.Tags.TryGetValue("natural", out var natural) && natural == "tree_row")
                .OrderBy(way => way.Id)
                .ToArray();

        foreach (var row in rows)
        {
            token.ThrowIfCancellationRequested();

            if (row.NodeIds.Length < 2 || row.NodeIds.Any(id => !data.Nodes.ContainsKey(id)))
            {
                Count("invalidRows");
                log.Warn($"Skipped tree row {row.Id}: incomplete geometry.");
                continue;
            }

            var points = row.NodeIds.Select(id =>
{
    var node = data.Nodes[id];
    return GeoMath.ToLocalMeters(node.Latitude, node.Longitude, request.Latitude, request.Longitude);
}).ToArray();

            if (points.Any(point => !math.all(math.isfinite(point))))
            {
                Count("invalidRows");
                continue;
            }

            // Continuous spacing along a polyline, not a fresh placement at every OSM vertex.
            var nextDistance = 0f;

            for (var i = 1; i < points.Length; i++)
            {
                var vectorA = points[i - 1];
                var vectorB = points[i];
                var length = math.distance(vectorA, vectorB);

                if (length < .001f)
                    continue;

                if (!ClipRow(vectorA, vectorB, half, out var entry, out var exit))
                {
                    nextDistance = (nextDistance - length) % MappedSpacing;

                    if (nextDistance < 0)
                        nextDistance += MappedSpacing;

                    continue;
                }

                var d =
                    nextDistance
                        + Math.Max(0, (float)Math.Ceiling((entry * length - nextDistance) / MappedSpacing))
                            * MappedSpacing;

                for (; d <= exit * length; d += MappedSpacing)
                {
                    token.ThrowIfCancellationRequested();
                    Add(math.lerp(vectorA, vectorB, d / length), "rowTree");
                }

                nextDistance = (nextDistance - length) % MappedSpacing;

                if (nextDistance < 0)
                    nextDistance += MappedSpacing;
            }
        }

        EndPart("Forest mapped trees");

        // Fixed density. Retain lazy center-out ordering even without a count cap,
        // so a future cap can reuse the same deterministic placement policy.
        // Block distance is a lower bound, so no unvisited block can hide a closer eligible tree.
        const int blockCells = 32;
        var cell = WoodlandCellSize;
        var blockWidth = cell * blockCells;
        var index = new Dictionary<(int, int), WoodlandBlock>();
        var holes = new Dictionary<Mask, Hole[]>();

        foreach (var mask in masks)
            holes[mask] =
                mask.Inner
                    .Select(
                        points => new Hole(
                            new RingIndex(points),
                            points.Min(point => point.x),
                            points.Max(point => point.x),
                            points.Min(point => point.z),
                            points.Max(point => point.z)))
                    .ToArray();

        // With inferred trees off, only woodland (and park edges) yield cell trees, and a tree
        // only needs to know that a cell is not one of them, so the other areas ranked below
        // woodland need no tests at all.
        foreach (var mask in masks.Where(
            mask => PlantInferredTrees
                || mask.Kind >= AreaKind.Woodland
                || PlantParkEdges && mask.Kind == AreaKind.Park))
            foreach (var ring in mask.Outer)
            {
                var bounds =
                    new WoodlandRing(
                        mask,
                        new RingIndex(ring),
                        ring.Min(point => point.x),
                        ring.Max(point => point.x),
                        ring.Min(point => point.z),
                        ring.Max(point => point.z));

                if (bounds.MaxX < -half || bounds.MinX > half || bounds.MaxZ < -half || bounds.MinZ > half)
                    continue;

                var minX = (int)Math.Floor(Math.Max(-half, bounds.MinX) / blockWidth);
                var maxX = (int)Math.Floor(Math.Min(half, bounds.MaxX) / blockWidth);
                var minZ = (int)Math.Floor(Math.Max(-half, bounds.MinZ) / blockWidth);
                var maxZ = (int)Math.Floor(Math.Min(half, bounds.MaxZ) / blockWidth);

                for (var z = minZ; z <= maxZ; z++)
                    for (var x = minX; x <= maxX; x++)
                    {
                        token.ThrowIfCancellationRequested();

                        if (!index.TryGetValue((x, z), out var block))
                        {
                            float AxisDistance(int axis) => axis >= 0 ? axis * blockWidth : -(axis + 1) * blockWidth;

                            var dx = AxisDistance(x);
                            var dz = AxisDistance(z);
                            index[(x, z)] =
                                block = new WoodlandBlock { X = x, Z = z, MinimumDistanceSquared = dx * dx + dz * dz };
                        }

                        block.Rings.Add(bounds);
                    }
            }

        var blocks =
            index.Values
                .Where(
                    block => PlantInferredTrees
                        || block.Rings.Any(
                            ring => ring.Mask.Kind == AreaKind.Woodland
                                || PlantParkEdges && ring.Mask.Kind == AreaKind.Park))
                .OrderBy(block => block.MinimumDistanceSquared)
                .ThenBy(block => block.X)
                .ThenBy(block => block.Z)
                .ToArray();

        bool Covers(WoodlandRing ring, float3 point)
        {
            if (point.x < ring.MinX
                || point.x > ring.MaxX
                || point.z < ring.MinZ
                || point.z > ring.MaxZ
                || !ring.Points.Contains(point))
                return false;

            foreach (var hole in holes[ring.Mask])
                if (point.x >= hole.MinX
                    && point.x <= hole.MaxX
                    && point.z >= hole.MinZ
                    && point.z <= hole.MaxZ
                    && hole.Points.Contains(point))
                    return false;

            return true;
        }

        AreaKind KindAt(float3 point, IEnumerable<WoodlandRing> rings)
        {
            var kind = AreaKind.None;

            foreach (var ring in rings)
            {
                if (ring.Mask.Kind <= kind || !Covers(ring, point))
                    continue;

                kind = ring.Mask.Kind;

                if (kind == AreaKind.Open)
                    break;
            }

            return kind;
        }

        AreaKind AreaAt(float3 point)
            => index.TryGetValue(
                ((int)Math.Floor(point.x / blockWidth), (int)Math.Floor(point.z / blockWidth)),
                out var block)
                ? KindAt(point, block.Rings)
                : AreaKind.None;

        EndPart("Forest woodland index");

        // Garden and street trees are watered, so a dry climate keeps at least
        // half of them. Unknown climate takes a middle value.
        var rain = climate == null ? float.NaN : ClimateMath.AnnualPrecipitation(climate.Precipitation);
        var residential = float.IsNaN(rain) ? .8f : Math.Clamp(.5f + (rain - 200) / 800, .5f, 1f);

        foreach (var mask in masks.Where(mask => mask.Kind == AreaKind.Orchard))
        {
            token.ThrowIfCancellationRequested();
            var outline = mask.Outer.OrderByDescending(points => points.Length).First();
            var u = new float2(1, 0);
            var longest = 0f;

            for (var i = 0; i < outline.Length; i++)
            {
                var edge = outline[(i + 1) % outline.Length].xz - outline[i].xz;

                if (math.lengthsq(edge) > longest)
                {
                    longest = math.lengthsq(edge);
                    u = math.normalizesafe(edge, u);
                }
            }

            var v = new float2(-u.y, u.x);
            var origin = outline[0].xz;
            var low = new float2(float.MaxValue);
            var high = new float2(float.MinValue);

            foreach (var point in mask.Outer.SelectMany(points => points))
            {
                var otherPoint = new float2(math.dot(point.xz - origin, u), math.dot(point.xz - origin, v));
                low = math.min(low, otherPoint);
                high = math.max(high, otherPoint);
            }

            var rings =
                mask.Outer
                    .Select(
                        points => new WoodlandRing(
                            mask,
                            new RingIndex(points),
                            points.Min(point => point.x),
                            points.Max(point => point.x),
                            points.Min(point => point.z),
                            points.Max(point => point.z)))
                    .ToArray();

            for (var b = low.y + OrchardRowSpacing * .5f; b <= high.y; b += OrchardRowSpacing)
                for (var a = low.x + MappedSpacing * .5f; a <= high.x; a += MappedSpacing)
                {
                    var otherPoint = origin + u * a + v * b;
                    var point = new float3(otherPoint.x, 0, otherPoint.y);

                    if (Math.Abs(point.x) <= half
                        && Math.Abs(point.z) <= half
                        && rings.Any(ring => Covers(ring, point)))
                        Add(point, "orchardTree");
                }
        }

        // Road courses resampled about every 10 m, for street trees and for
        // gaps between mapped trees along a road. Offsets are measured from
        // the course, positive to the left of travel.
        (float3[] Points, float[] Along) Course(PreparedSegment segment)
        {
            var estimate = 0f;

            for (var i = 1; i <= 8; i++)
                estimate +=
                    math.distance(
                        NetworkNormalizer.Evaluate(segment, (i - 1) / 8f).xz,
                        NetworkNormalizer.Evaluate(segment, i / 8f).xz);

            var steps = Math.Max(1, (int)Math.Ceiling(estimate / 10));
            var points = new float3[steps + 1];
            var along = new float[steps + 1];

            for (var i = 0; i <= steps; i++)
            {
                points[i] = NetworkNormalizer.Evaluate(segment, i / (float)steps);

                if (i > 0)
                    along[i] = along[i - 1] + math.distance(points[i - 1].xz, points[i].xz);
            }

            return (points, along);
        }

        float3 Beside((float3[] Points, float[] Along) course, float along, float offset)
        {
            var i = 1;

            while (i < course.Along.Length - 1 && course.Along[i] < along)
                i++;

            var vectorA = course.Points[i - 1];
            var vectorB = course.Points[i];
            var span = course.Along[i] - course.Along[i - 1];
            var point =
                math.lerp(vectorA, vectorB, span > 1e-4f ? math.saturate((along - course.Along[i - 1]) / span) : 0);

            var d = math.normalizesafe(vectorB.xz - vectorA.xz);

            return point + new float3(-d.y, 0, d.x) * offset;
        }

        foreach (var item in candidates.Where(
            otherItem => otherItem.category is "mappedTree" or "rowTree" && math.all(math.isfinite(otherItem.point))))
        {
            var key =
                BucketKey((int)Math.Floor(item.point.x / MinimumSpacing), (int)Math.Floor(item.point.z / MinimumSpacing));

            if (!explicitTrees.TryGetValue(key, out var list))
                explicitTrees[key] = list = new();

            list.Add(item.point);
        }

        var mappedGrid = new Dictionary<(int, int), List<float3>>();

        foreach (var item in candidates.Where(
            otherItem => otherItem.category == "mappedTree" && math.all(math.isfinite(otherItem.point))))
        {
            var key = ((int)Math.Floor(item.point.x / 32), (int)Math.Floor(item.point.z / 32));

            if (!mappedGrid.TryGetValue(key, out var list))
                mappedGrid[key] = list = new();

            list.Add(item.point);
        }

        var avenueRoads = 0;

        // Avenue and street trees are inferred, and dropped below when those are off.
        foreach (var segment in networks.Where(
            otherSegment => PlantInferredTrees
                && otherSegment.Kind == NetworkKind.Road
                && !otherSegment.Tunnel
                && !otherSegment.Bridge
                && !otherSegment.Classification.StartsWith("motorway", StringComparison.Ordinal)))
        {
            token.ThrowIfCancellationRequested();
            var course = Course(segment);
            var width = Math.Max(4, segment.NativeWidth * .5f);
            var length = course.Along[course.Along.Length - 1];

            if (length < 20)
                continue;

            // Mapped trees along this road: 1–12 m beyond its half width, beside it rather than past its ends.
            var sides = new[] { new List<float2>(), new List<float2>() };
            var low =
                course.Points.Aggregate(new float2(float.MaxValue), (m, point) => math.min(m, point.xz)) - width - 12;

            var high =
                course.Points.Aggregate(new float2(float.MinValue), (m, point) => math.max(m, point.xz)) + width + 12;

            for (var gz = (int)Math.Floor(low.y / 32); gz <= (int)Math.Floor(high.y / 32); gz++)
                for (var gx = (int)Math.Floor(low.x / 32); gx <= (int)Math.Floor(high.x / 32); gx++)
                {
                    if (!mappedGrid.TryGetValue((gx, gz), out var nearby))
                        continue;

                    foreach (var point in nearby)
                    {
                        var best = float.PositiveInfinity;
                        var along = 0f;
                        var offset = 0f;

                        for (var i = 1; i < course.Points.Length; i++)
                        {
                            var a = course.Points[i - 1].xz;
                            var ab = course.Points[i].xz - a;
                            var t = math.dot(point.xz - a, ab) / math.max(1e-6f, math.lengthsq(ab));

                            if (t < 0 && i > 1 || t > 1 && i < course.Points.Length - 1)
                                t = math.saturate(t);

                            if (t < 0 || t > 1)
                                continue;

                            var distance = math.distance(point.xz, a + ab * t);

                            if (distance >= best)
                                continue;

                            var d = math.normalizesafe(ab);
                            best = distance;
                            along = course.Along[i - 1] + t * math.length(ab);
                            offset = math.dot(point.xz - a, new float2(-d.y, d.x));
                        }

                        if (best < width + 1 || best > width + 12)
                            continue;

                        sides[offset > 0 ? 0 : 1].Add(new float2(along, offset));
                    }
                }

            var filled = false;

            foreach (var side in sides.Where(points => points.Count >= 2))
            {
                side.Sort((vectorA, vectorB) => vectorA.x.CompareTo(vectorB.x));
                // The street's own planting interval: the median of its short gaps.
                var gaps =
                    side
                        .Zip(side.Skip(1), (vectorA, vectorB) => vectorB.x - vectorA.x)
                        .Where(g => g >= 4 && g <= 30)
                        .OrderBy(g => g)
                        .ToArray();

                var interval = gaps.Length == 0 ? StreetSpacing : Math.Clamp(gaps[gaps.Length / 2], MappedSpacing, 15f);

                for (var i = 1; i < side.Count; i++)
                {
                    var gap = side[i].x - side[i - 1].x;

                    if (gap <= 1.5f * interval || gap > 80)
                        continue;

                    var n = (int)Math.Round(gap / interval) - 1;

                    for (var k = 1; k <= n; k++)
                    {
                        var f = k / (float)(n + 1);
                        Add(
                            Beside(
                                course,
                                math.lerp(side[i - 1].x, side[i].x, f),
                                math.lerp(side[i - 1].y, side[i].y, f)),
                            "avenueTree");

                        filled = true;
                    }
                }
            }

            if (filled)
                avenueRoads++;

            // Street trees on residential land: both verges, 12 m clear of
            // junctions, with about one place in four left open for driveways.
            if (segment.Roundabout || segment.Classification.EndsWith("_link", StringComparison.Ordinal)
                || segment.Classification is not ("residential" or "living_street" or "unclassified" or "tertiary" or "secondary" or "primary"))
                continue;

            var phase =
                (Hash(unchecked((uint)segment.WayId * 2654435761u ^ (uint)segment.StartNodeId)) & 1023) / 1024f
                    * StreetSpacing;

            for (var d = 12 + phase; d <= length - 12; d += StreetSpacing)
                for (var side = -1; side <= 1; side += 2)
                {
                    var point = Beside(course, d, side * (width + StreetSetback));

                    if (Roll(point) >= .75f)
                        continue;

                    if (AreaAt(point) is AreaKind.Residential or AreaKind.Grounds or AreaKind.Park)
                        Add(point, "streetTree");
                }
        }

        int Priority(string category)
            => category == "mappedTree"
                ? 0
                : category == "rowTree" ? 1 : category is "avenueTree" or "streetTree" or "orchardTree" ? 2 : 3;

        // Centre-out order: distance from the map centre, then category priority, then
        // discovery order. The serial makes the order total, so a binary heap pops exactly
        // as the earlier sorted set did, without a tree node per candidate (report 114 S5).
        var pending = new PendingTrees();
        long serial = 0;

        foreach (var candidate in candidates)
        {
            token.ThrowIfCancellationRequested();

            if (!PlantInferredTrees
                && candidate.category is not ("mappedTree" or "rowTree")
                && !(PlantOrchards && candidate.category == "orchardTree"))
                continue;

            if (math.all(math.isfinite(candidate.point)))
                pending.Add(new PendingTree(candidate.point, CategoryNamed(candidate.category), serial++));
            else
            {
                Count(candidate.category + "Candidates");
                Count("outsideOrInvalid");
            }
        }

        EndPart("Forest candidate grids");
        var surveySeconds = 0.0;
        var nextBlock = 0;
        var woodlandCandidates = 0;
        var woodlandCellsVisited = 0;
        var nextProgress = timer.Elapsed.TotalSeconds + 5;
        log.Info(
            $"Woodland sampling: fixed {cell:0} m cells; lazy center-out placement; no tree-count limit. Global density reduction disabled.");

        // Each cell takes the highest-precedence area covering it; the map border counts
        // as the same area, so it is not an edge.
        const byte border = byte.MaxValue;
        var span = blockCells + 2 * EdgeCells;

        // The pure part of a block: its cells' jittered points, area kinds and distance in
        // cells from the area's edge. It reads only the rings, so blocks are surveyed ahead in
        // parallel; their trees are still added one block at a time in the same order.
        (float3[] Points, byte[] Kinds, int[] Edges) Survey(WoodlandBlock block)
        {
            // Coverage of the block plus an EdgeCells margin (cells' own jittered
            // points), so each tree knows how many cells it is from the woodland edge.
            // The margin also tests the neighbouring blocks' rings, so a boundary
            // shared with an adjacent wood is not mistaken for an open edge.
            var rings = new List<WoodlandRing>(block.Rings);

            for (var nz = block.Z - 1; nz <= block.Z + 1; nz++)
                for (var nx = block.X - 1; nx <= block.X + 1; nx++)
                    if ((nx != block.X || nz != block.Z) && index.TryGetValue((nx, nz), out var neighbour))
                        foreach (var ring in neighbour.Rings)
                            if (!rings.Contains(ring))
                                rings.Add(ring);

            var kinds = new byte[span * span];
            var points = new float3[span * span];
            var edges = new int[span * span];

            for (var k = 0; k < span; k++)
                for (var j = 0; j < span; j++)
                {
                    var x = block.X * blockCells - EdgeCells + j;
                    var z = block.Z * blockCells - EdgeCells + k;
                    var seed = Hash(unchecked((uint)x * 73856093u ^ (uint)z * 19349663u));
                    var point =
                        new float3((x + (seed & 65535) / 65536f) * cell, 0, (z + (Hash(seed) & 65535) / 65536f) * cell);

                    points[k * span + j] = point;

                    if (Math.Abs(point.x) > half || Math.Abs(point.z) > half)
                    {
                        kinds[k * span + j] = border;
                        continue;
                    }

                    var margin =
                        j < EdgeCells || k < EdgeCells || j >= EdgeCells + blockCells || k >= EdgeCells + blockCells;

                    kinds[k * span + j] = (byte)KindAt(point, margin ? rings : block.Rings);
                }

            for (var k = EdgeCells; k < EdgeCells + blockCells; k++)
                for (var j = EdgeCells; j < EdgeCells + blockCells; j++)
                {
                    var here = kinds[k * span + j];
                    var kind = (AreaKind)here;

                    if (here == border || kind is AreaKind.None or AreaKind.Orchard or AreaKind.Open)
                        continue;

                    var edge = EdgeCells + 1;

                    for (var dz = -EdgeCells; dz <= EdgeCells; dz++)
                        for (var dx = -EdgeCells; dx <= EdgeCells; dx++)
                        {
                            var other = kinds[(k + dz) * span + j + dx];

                            if (other != here && other != border)
                                edge = Math.Min(edge, Math.Max(Math.Abs(dx), Math.Abs(dz)));
                        }

                    edges[k * span + j] = edge;
                }

            return (points, kinds, edges);
        }

        var surveys = new (float3[] Points, byte[] Kinds, int[] Edges)[blocks.Length];
        var surveyAhead = Math.Max(1, Environment.ProcessorCount) * 4;

        // Count keys per category, built once rather than for every cell.
        var categoryKeys = new Dictionary<AreaKind, (string Category, string Cells, string Thinned)>();
        var woodlandCategory = CategoryNamed("woodlandTree");

        void Generate(int number)
        {
            if (surveys[number].Points == null)
            {
                var surveyStart = timer.Elapsed.TotalSeconds;
                System.Threading.Tasks.Parallel.For(
                    number,
                    Math.Min(blocks.Length, number + surveyAhead),
                    new System.Threading.Tasks.ParallelOptions { CancellationToken = token },
                    next => surveys[next] = Survey(blocks[next]));

                surveySeconds += timer.Elapsed.TotalSeconds - surveyStart;
            }

            var (points, kinds, edges) = surveys[number];
            surveys[number] = default;

            for (var k = EdgeCells; k < EdgeCells + blockCells; k++)
                for (var j = EdgeCells; j < EdgeCells + blockCells; j++)
                {
                    woodlandCellsVisited++;
                    var point = points[k * span + j];
                    var here = kinds[k * span + j];
                    var kind = (AreaKind)here;

                    if (here == border || kind is AreaKind.None or AreaKind.Orchard or AreaKind.Open)
                        continue;

                    var edge = edges[k * span + j];

                    if (kind == AreaKind.Woodland)
                    {
                        woodlandCandidates++;

                        if (!WoodlandKeep(point, edge * cell))
                        {
                            Count("thinned");
                            continue;
                        }

                        pending.Add(new PendingTree(point, woodlandCategory, serial++));
                        continue;
                    }

                    // Park edges alone: a park cell more than one cell from the edge stays bare.
                    if (!PlantInferredTrees && !(PlantParkEdges && kind == AreaKind.Park && edge <= 1))
                        continue;

                    if (!categoryKeys.TryGetValue(kind, out var keys))
                    {
                        var name = Category(kind);
                        categoryKeys[kind] = keys = (name, name + "Cells", name + "Thinned");
                    }

                    Count(keys.Cells);

                    if (!InferredKeep(kind, point, edge, residential))
                    {
                        Count(keys.Thinned);
                        continue;
                    }

                    pending.Add(new PendingTree(point, CategoryNamed(keys.Category), serial++));
                }
        }

        // Blocks are visited center-out, so stopping early keeps the trees nearest the
        // center and only leaves outer woodland bare; an import never stalls on this step.
        var woodlandStopped = false;

        while (pending.Count > 0 || (!woodlandStopped && nextBlock < blocks.Length))
        {
            token.ThrowIfCancellationRequested();

            if (!woodlandStopped
                && nextBlock < blocks.Length
                && timer.Elapsed.TotalSeconds >= WoodlandTimeLimitSeconds)
            {
                woodlandStopped = true;
                log.Warn(
                    $"Tree selection passed {WoodlandTimeLimitSeconds:0} s; {blocks.Length - nextBlock:N0} of {blocks.Length:N0} outer woodland blocks were left without trees.");
            }

            if (timer.Elapsed.TotalSeconds >= nextProgress)
            {
                log.Info(
                    $"Preparing center-out trees: {output.Count:N0} accepted (uncapped); woodland blocks {nextBlock:N0}/{blocks.Length:N0} visited.");

                nextProgress = timer.Elapsed.TotalSeconds + 5;
            }

            if (!woodlandStopped
                && nextBlock < blocks.Length
                && (pending.Count == 0
                    || blocks[nextBlock].MinimumDistanceSquared <= pending.Min.DistanceSquared))
            {
                Generate(nextBlock++);
                continue;
            }

            var candidate = pending.RemoveMin();
            Accept(candidate.Point, candidate.Category);
        }

        // The survey runs inside the placement loop; its parallel batches are timed apart.
        log.AddDuration("Forest woodland survey", surveySeconds);
        log.AddDuration("Forest placement", partTimer.Elapsed.TotalSeconds - surveySeconds);
        partTimer.Restart();

        log.Info(
            $"Forest clearance index: {corridors.CellCount:N0} cells (50m); {corridors.CandidateChecks:N0} nearby checks; {corridors.ExactChecks:N0} exact distance checks; original clearance distances retained.");

        log.Diagnostic(
            $"FOREST INDEX cellMeters=50; cells={corridors.CellCount}; nearbyChecks={corridors.CandidateChecks}; exactChecks={corridors.ExactChecks}; trees={output.Count}; elapsedSeconds={timer.Elapsed.TotalSeconds:0.###}");

        // The per-tree counters join the named counts. A name is added only when its count is
        // above zero, as the per-tree Count calls did, so the same names exist with the same values.
        void Join(string key, int amount)
        {
            if (amount > 0)
                Count(key, amount);
        }

        foreach (var category in categories.Values)
        {
            Join(category.Name + "Candidates", category.Candidates);
            Join(category.Name + "Accepted", category.Accepted);
        }

        Join("outsideOrInvalid", outsideOrInvalid);
        Join("waterRejected", waterRejected);
        Join("networkRejected", networkRejected);
        Join("spacingRejected", spacingRejected);

        int Total(string key) => counts.TryGetValue(key, out var value) ? value : 0;

        log.Diagnostic(
            $"FOREST DISTRIBUTION order=center-out; globalDensityReduction=false; woodlandCellMeters={cell:0.###}; woodlandSpacingMeters={WoodlandSpacing}; mappedSpacingMeters={MappedSpacing}; totalDiscoveredCandidates={candidates.Count + woodlandCandidates}; mappedTreeCandidates={candidates.Count(item => item.category == "mappedTree")}; mappedTreesAccepted={Total("mappedTreeAccepted")}; mappedRows={rows.Length}; rowCandidates={candidates.Count(item => item.category == "rowTree")}; rowTreesAccepted={Total("rowTreeAccepted")}; woodlandCandidates={woodlandCandidates}; woodlandThinned={Total("thinned")}; woodlandTreesAccepted={Total("woodlandTreeAccepted")}; woodlandBlocksVisited={nextBlock}; woodlandBlocksTotal={blocks.Length}; woodlandCellsVisited={woodlandCellsVisited}; woodlandFullyEnumerated={nextBlock == blocks.Length}; outsideOrInvalid={Total("outsideOrInvalid")}; waterRejected={Total("waterRejected")}; networkRejected={Total("networkRejected")}; spacingRejected={Total("spacingRejected")}; unexaminedDiscoveredCandidates={pending.Count}; invalidRows={Total("invalidRows")}; totalPrepared={output.Count}; limit=none; limitReached=false; outermostAcceptedRadiusMeters={(output.Count == 0 ? 0 : math.length(output[output.Count - 1].xz)):0.##}; finalNativeTreeCountVerified=false");

        var inferred =
            new[] {
                "orchardTree",
                "avenueTree",
                "streetTree",
                "parkTree",
                "cemeteryTree",
                "golfTree",
                "groundsTree",
                "gardenTree"
                };

        log.Diagnostic(
            $"FOREST INFERENCE residentialFactor={residential:0.###}; annualPrecipitationMm={rain:0.#}; masks={string.Join(",", masks.GroupBy(mask => mask.Kind).OrderBy(group => group.Key).Select(group => $"{group.Key}:{group.Count()}"))}; incompleteAreas={incompleteAreas}; mappedStreetsFilled={avenueRoads}; "
                +
            string.Join(
                "; ",
                inferred.Select(
                    text => $"{text}Candidates={Total(text + "Candidates")}; {text}Cells={Total(text + "Cells")}; {text}Thinned={Total(text + "Thinned")}; {text}Accepted={Total(text + "Accepted")}")));

        log.Evidence(
            $"Trees prepared: {Total("mappedTreeAccepted"):N0} mapped individual trees, {Total("rowTreeAccepted"):N0} row trees, {Total("woodlandTreeAccepted"):N0} woodland trees ({Total("thinned"):N0} thinned for natural edges and gaps), {inferred.Sum(text => Total(text + "Accepted")):N0} inferred ({string.Join(", ", inferred.Select(text => $"{Total(text + "Accepted"):N0} {text}"))}); {output.Count:N0} total. Spacing woodland {WoodlandSpacing:0} m, mapped/rows {MappedSpacing:0} m; {Total("spacingRejected"):N0} spacing rejections.");

        log.Count("Trees prepared", output.Count);
        EndPart("Forest summary");
        log.Time("Vegetation preparation", timer.Elapsed.TotalSeconds);

        foreach (var text in inferred)
            if (Total(text + "Accepted") > 0)
                log.Count("Inferred trees prepared: " + text, Total(text + "Accepted"));

        return output;
    }

    // Woodland cells within this many cells of an open cell count as edge.
    private const int EdgeCells = 3;

    // Natural thinning, deterministic per position. Edges feather out over three
    // cells (keep ~45% at the rim), but only where a slow 150 m pattern says so,
    // so some edges stay crisp; a 250 m pattern varies interior density by up
    // to 25%; and a 90 m pattern opens occasional small clearings. The interior,
    // away from edges and clearings, keeps InteriorInfill more.
    internal static bool WoodlandKeep(float3 point, float edgeDistance)
    {
        var x = point.x;
        var z = point.z;
        var featherHere = Smooth((Noise(x / 150, z / 150, 0x9e3779b9u) - .35f) / .3f);
        var rim = Smooth(edgeDistance / (EdgeCells * WoodlandCellSize));
        var edge = 1 - featherHere * (1 - (.45f + .55f * rim));
        var density = .75f + .25f * Noise(x / 250, z / 250, 0x85ebca6bu);
        var clearing = Noise(x / 90, z / 90, 0xc2b2ae35u) > .82f ? .3f : 1f;
        var keep = edge * density * clearing;

        if (rim >= 1 && clearing >= 1)
            keep = Math.Min(1, keep + InteriorInfill);

        return Roll(point) < keep;
    }

    private static string Category(AreaKind kind) => kind switch
    {
        AreaKind.Residential => "gardenTree",
        AreaKind.Grounds => "groundsTree",
        AreaKind.Golf => "golfTree",
        AreaKind.Park => "parkTree",
        AreaKind.Cemetery => "cemeteryTree",
        _ => "woodlandTree"
    };

    // Open-grown trees on land not mapped as woodland, as the share of 15 m
    // cells kept (every cell would be about 44 trees/ha). Parks and golf
    // roughs form clumps on a 70 m pattern, with trees along their edges
    // (within one cell). Residential land varies by up to ±45% over 250 m,
    // scaled by the climate factor.
    internal static bool InferredKeep(AreaKind kind, float3 point, int edgeCells, float residential)
    {
        var x = point.x;
        var z = point.z;
        var clump = Noise(x / 70, z / 70, 0x27d4eb2fu);
        var keep = kind switch
        {
            AreaKind.Park => edgeCells <= 1 ? .55f : clump > .55f ? .8f : clump > .4f ? .35f : .08f,
            AreaKind.Golf => edgeCells <= 1 ? .6f : clump > .5f ? .7f : .15f,
            AreaKind.Cemetery => .35f * (.7f + .6f * Noise(x / 80, z / 80, 0x165667b1u)),
            AreaKind.Grounds => .15f,
            AreaKind.Residential => residential * .32f * (.55f + .9f * Noise(x / 250, z / 250, 0xd3a2646cu)),
            _ => 0f
        };

        return Roll(point) < keep;
    }

    // Deterministic per-position uniform value in [0,1).
    private static float Roll(float3 point) =>
        (Hash(
            unchecked((uint)(int)Math.Floor(point.x * 4) * 2654435761u
                ^ (uint)(int)Math.Floor(point.z * 4) * 2246822519u))
            & 65535)
            / 65536f;

    private static float Smooth(float t)
    {
        t = Math.Clamp(t, 0, 1);
        return t * t * (3 - 2 * t);
    }

    // Bilinear value noise on an integer lattice, in [0,1].
    private static float Noise(float x, float z, uint salt)
    {
        var x0 = (int)Math.Floor(x);
        var z0 = (int)Math.Floor(z);
        var fx = x - x0;
        var fz = z - z0;

        float Lattice(int i, int k)
            => (Hash(unchecked((uint)i * 73856093u ^ (uint)k * 19349663u ^ salt)) & 65535) / 65535f;

        var u = fx * fx * (3 - 2 * fx);
        var v = fz * fz * (3 - 2 * fz);

        return math.lerp(
            math.lerp(Lattice(x0, z0), Lattice(x0 + 1, z0), u),
            math.lerp(Lattice(x0, z0 + 1), Lattice(x0 + 1, z0 + 1), u),
            v);
    }

    private static uint Hash(uint value)
    {
        unchecked
        {
            value ^= value >> 16;
            value *= 0x7feb352d;
            value ^= value >> 15;
            value *= 0x846ca68b;

            return value ^ (value >> 16);
        }
    }

    // One dictionary key per spacing bucket. A tuple, not two ints packed into a long: a long's
    // hash folds its halves together with XOR, so x and z collided and the forest ran three
    // times slower (report 119).
    private static (int, int) BucketKey(int x, int z) => (x, z);

    // Whether any of the points stands closer than distance to point in the plane: the
    // spacing test as a plain loop, without a closure, delegate and enumerator per bucket.
    private static bool AnyCloser(List<float3> points, float3 point, float distance)
    {
        for (var i = 0; i < points.Count; i++)
            if (math.distance(points[i].xz, point.xz) < distance)
                return true;

        return false;
    }

    private static bool ClipRow(float3 vectorA, float3 vectorB, float half, out float entry, out float exit)
    {
        var low = 0f;
        var high = 1f;

        bool Axis(float start, float delta)
        {
            if (Math.Abs(delta) < .0001f)
                return Math.Abs(start) <= half;

            var t0 = (-half - start) / delta;
            var t1 = (half - start) / delta;
            low = Math.Max(low, Math.Min(t0, t1));
            high = Math.Min(high, Math.Max(t0, t1));

            return low <= high;
        }

        var hit = Axis(vectorA.x, vectorB.x - vectorA.x) && Axis(vectorA.z, vectorB.z - vectorA.z);
        entry = low;
        exit = high;

        return hit;
    }

    internal static bool Inside(float3 point, IReadOnlyList<float3> ring)
    {
        var inside = false;

        for (var i = 0; i < ring.Count; i++)
        {
            var vectorA = ring[i];
            var vectorB = ring[(i + 1) % ring.Count];

            if ((vectorA.z > point.z) != (vectorB.z > point.z)
                && point.x < (vectorB.x - vectorA.x) * (point.z - vectorA.z) / (vectorB.z - vectorA.z) + vectorA.x)
                inside = !inside;
        }

        return inside;
    }
}
