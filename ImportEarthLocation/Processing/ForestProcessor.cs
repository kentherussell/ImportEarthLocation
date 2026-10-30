using System;
using System.Collections.Generic;
using System.Diagnostics;
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
    private sealed record Mask(List<float3[]> Outer, List<float3[]> Inner);
    private sealed record WoodlandRing(Mask Mask, float3[] Points, float MinX, float MaxX, float MinZ, float MaxZ);
    private sealed record Hole(float3[] Points,float MinX,float MaxX,float MinZ,float MaxZ);
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
        public void Add(float3 a, float3 b, float radius, CancellationToken token)
        {
            var corridor = new Corridor(a, b, radius);
            for (var x = (int)Math.Floor((Math.Min(a.x, b.x) - radius) / CellSize); x <= (int)Math.Floor((Math.Max(a.x, b.x) + radius) / CellSize); x++)
            for (var z = (int)Math.Floor((Math.Min(a.z, b.z) - radius) / CellSize); z <= (int)Math.Floor((Math.Max(a.z, b.z) + radius) / CellSize); z++)
            {
                token.ThrowIfCancellationRequested();
                if (!_cells.TryGetValue((x, z), out var list)) _cells[(x, z)] = list = new List<Corridor>();
                list.Add(corridor);
            }
        }
        public bool Contains(float3 point)
        {
            if (!_cells.TryGetValue(((int)Math.Floor(point.x / CellSize), (int)Math.Floor(point.z / CellSize)), out var nearby)) return false;
            foreach (var corridor in nearby)
            {
                CandidateChecks++;
                if (point.x < Math.Min(corridor.A.x, corridor.B.x) - corridor.Radius || point.x > Math.Max(corridor.A.x, corridor.B.x) + corridor.Radius ||
                    point.z < Math.Min(corridor.A.z, corridor.B.z) - corridor.Radius || point.z > Math.Max(corridor.A.z, corridor.B.z) + corridor.Radius) continue;
                ExactChecks++;
                if (NetworkNormalizer.DistanceToSegment(point, corridor.A, corridor.B) <= corridor.Radius) return true;
            }
            return false;
        }
    }
    private static bool Forest(IReadOnlyDictionary<string, string> tags) =>
        tags.TryGetValue("natural", out var natural) && natural == "wood" || tags.TryGetValue("landuse", out var landuse) && landuse == "forest";

    public static IReadOnlyList<float3> Prepare(OsmDataset data, ImportRequest request, TerrainBuilder.Result terrain,
        IReadOnlyList<PreparedSegment> networks, ImportLog log, CancellationToken token,Func<float3,bool>? excludeWater=null)
    {
        token.ThrowIfCancellationRequested();
        var timer = Stopwatch.StartNew(); var masks = new List<Mask>(); var members = new HashSet<long>();
        foreach (var relation in data.Relations.Where(relation => Forest(relation.Tags)).OrderBy(relation => relation.Id))
        {
            token.ThrowIfCancellationRequested();
            foreach (var member in relation.Members) members.Add(member.wayId);
            try
            {
                masks.Add(new Mask(Rings(relation.Members.Where(member => !member.inner).Select(member => member.wayId)),
                    Rings(relation.Members.Where(member => member.inner).Select(member => member.wayId))));
            }
            catch (InvalidOperationException exception) { log.Warn($"Skipped forest relation {relation.Id}: {exception.Message}"); }
        }
        foreach (var way in data.Ways.Values.Where(way => Forest(way.Tags) && !members.Contains(way.Id)).OrderBy(way => way.Id))
        {
            token.ThrowIfCancellationRequested();
            try { masks.Add(new Mask(Rings(new[] { way.Id }), new List<float3[]>())); }
            catch (InvalidOperationException exception) { log.Warn($"Skipped forest way {way.Id}: {exception.Message}"); }
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
                var first = remaining.First(); remaining.Remove(first.Key); var ring = first.Value.ToList();
                while (ring.Count > 1 && ring[0] != ring[ring.Count - 1])
                {
                    token.ThrowIfCancellationRequested();
                    var end = ring[ring.Count - 1];
                    var choices = remaining.Where(item => item.Value[0] == end || item.Value[item.Value.Length - 1] == end).ToArray();
                    if (choices.Length != 1) throw new InvalidOperationException("Open or ambiguous forest ring.");
                    var next = choices[0]; remaining.Remove(next.Key);
                    ring.AddRange((next.Value[0] == end ? next.Value : next.Value.Reverse()).Skip(1));
                }
                if (ring.Count < 4 || ring[0] != ring[ring.Count - 1]) throw new InvalidOperationException("Invalid forest ring.");
                var points = ring.Select(id => { var node = data.Nodes[id]; return GeoMath.ToLocalMeters(node.Latitude, node.Longitude, request.Latitude, request.Longitude); }).ToArray();
                if (points.Any(point => !math.all(math.isfinite(point)))) throw new InvalidOperationException("Invalid forest coordinates.");
                rings.Add(points);
            }
            return rings;
        }
        var half = (float)(request.MapSize.WidthMeters / 2);
        var corridors = new CorridorIndex();
        foreach (var segment in networks)
        {
            token.ThrowIfCancellationRequested();
            if(segment.Tunnel)continue; // Surface vegetation above covered tunnels is legitimate.
            var radius = segment.NativeWidth>0?segment.NativeWidth*.5f+6:segment.Kind == NetworkKind.Rail ? 12f : Math.Max(15f, segment.Lanes * 1.75f + 7f);
            var sections = Math.Max(1, (int)Math.Ceiling(math.distance(segment.Start, segment.End) / 10f));
            for (var i = 0; i < sections; i++)
            {
                var a = NetworkNormalizer.Evaluate(segment, i / (float)sections); var b = NetworkNormalizer.Evaluate(segment, (i + 1) / (float)sections);
                corridors.Add(a, b, radius + 2f, token);
            }
        }
        var output = new List<float3>();
        var candidates = new List<(float3 point, string category)>();
        var spacing = new Dictionary<(int, int), List<float3>>();
        var counts = new Dictionary<string, int>();
        void Count(string key) { counts.TryGetValue(key, out var value); counts[key] = value + 1; }
        void Add(float3 point, string category) => candidates.Add((point, category));
        bool Accept(float3 point, string category)
        {
            Count(category + "Candidates");
            if (!math.all(math.isfinite(point)) || Math.Abs(point.x) > half || Math.Abs(point.z) > half) { Count("outsideOrInvalid"); return false; }
            point.y = NetworkProcessor.Sample(point.x, point.z, terrain.Heightmap, terrain.HeightScaleMeters, request.MapSize.WidthMeters);
            if (!math.isfinite(point.y) || point.y <= WaterLevels.Effective(terrain.SeaLevelMeters) + 0.5f) { Count("waterRejected"); return false; }
            if(excludeWater?.Invoke(point)==true){Count("waterRejected");return false;}
            if (corridors.Contains(point)) { Count("networkRejected"); return false; }
            // Buckets stay MinimumSpacing wide, so the 3×3 scan covers either radius.
            var minimum = category == "woodlandTree" ? WoodlandSpacing : MappedSpacing;
            var bucket = ((int)Math.Floor(point.x / MinimumSpacing), (int)Math.Floor(point.z / MinimumSpacing));
            for (var dx = -1; dx <= 1; dx++) for (var dz = -1; dz <= 1; dz++)
                if (spacing.TryGetValue((bucket.Item1 + dx, bucket.Item2 + dz), out var neighbors) && neighbors.Any(other => math.distance(other.xz, point.xz) < minimum))
                { Count("spacingRejected"); return false; }
            if (!spacing.TryGetValue(bucket, out var bucketTrees)) spacing[bucket] = bucketTrees = new List<float3>();
            bucketTrees.Add(point); output.Add(point); Count(category + "Accepted"); return true;
        }
        foreach (var tagged in data.NodeTags.OrderBy(pair => pair.Key))
        {
            token.ThrowIfCancellationRequested();
            if (!tagged.Value.TryGetValue("natural", out var natural) || natural != "tree" || !data.Nodes.TryGetValue(tagged.Key, out var node)) continue;
            Add(GeoMath.ToLocalMeters(node.Latitude, node.Longitude, request.Latitude, request.Longitude), "mappedTree");
        }
        var rows = data.Ways.Values.Where(way => way.Tags.TryGetValue("natural", out var natural) && natural == "tree_row").OrderBy(way => way.Id).ToArray();
        foreach (var row in rows)
        {
            token.ThrowIfCancellationRequested();
            if (row.NodeIds.Length < 2 || row.NodeIds.Any(id => !data.Nodes.ContainsKey(id)))
            { Count("invalidRows"); log.Warn($"Skipped tree row {row.Id}: incomplete geometry."); continue; }
            var points = row.NodeIds.Select(id => { var n = data.Nodes[id]; return GeoMath.ToLocalMeters(n.Latitude, n.Longitude, request.Latitude, request.Longitude); }).ToArray();
            if (points.Any(p => !math.all(math.isfinite(p)))) { Count("invalidRows"); continue; }
            // Continuous spacing along a polyline, not a fresh placement at every OSM vertex.
            var nextDistance = 0f;
            for (var i = 1; i < points.Length; i++)
            {
                var a = points[i - 1]; var b = points[i]; var length = math.distance(a, b);
                if (length < .001f) continue;
                if (!ClipRow(a, b, half, out var entry, out var exit))
                { nextDistance = (nextDistance - length) % MappedSpacing; if (nextDistance < 0) nextDistance += MappedSpacing; continue; }
                var d = nextDistance + Math.Max(0, (float)Math.Ceiling((entry * length - nextDistance) / MappedSpacing)) * MappedSpacing;
                for (; d <= exit * length; d += MappedSpacing)
                { token.ThrowIfCancellationRequested(); Add(math.lerp(a, b, d / length), "rowTree"); }
                nextDistance = (nextDistance - length) % MappedSpacing; if (nextDistance < 0) nextDistance += MappedSpacing;
            }
        }
        // Fixed density. Retain lazy center-out ordering even without a count cap,
        // so a future cap can reuse the same deterministic placement policy.
        // Block distance is a lower bound, so no unvisited block can hide a closer eligible tree.
        const int blockCells = 32;
        var cell = WoodlandCellSize;
        var blockWidth = cell * blockCells;
        var index = new Dictionary<(int, int), WoodlandBlock>();
        var holes=new Dictionary<Mask,Hole[]>();
        foreach(var mask in masks)holes[mask]=mask.Inner.Select(h=>new Hole(h,h.Min(p=>p.x),h.Max(p=>p.x),h.Min(p=>p.z),h.Max(p=>p.z))).ToArray();
        foreach (var mask in masks)
        foreach (var ring in mask.Outer)
        {
            var bounds = new WoodlandRing(mask, ring, ring.Min(p=>p.x), ring.Max(p=>p.x), ring.Min(p=>p.z), ring.Max(p=>p.z));
            if (bounds.MaxX < -half || bounds.MinX > half || bounds.MaxZ < -half || bounds.MinZ > half) continue;
            var minX = (int)Math.Floor(Math.Max(-half, bounds.MinX) / blockWidth); var maxX = (int)Math.Floor(Math.Min(half, bounds.MaxX) / blockWidth);
            var minZ = (int)Math.Floor(Math.Max(-half, bounds.MinZ) / blockWidth); var maxZ = (int)Math.Floor(Math.Min(half, bounds.MaxZ) / blockWidth);
            for (var z = minZ; z <= maxZ; z++) for (var x = minX; x <= maxX; x++)
            {
                token.ThrowIfCancellationRequested();
                if (!index.TryGetValue((x,z),out var block))
                {
                    float AxisDistance(int axis) => axis >= 0 ? axis * blockWidth : -(axis + 1) * blockWidth;
                    var dx=AxisDistance(x); var dz=AxisDistance(z);
                    index[(x,z)] = block = new WoodlandBlock { X=x, Z=z, MinimumDistanceSquared=dx*dx+dz*dz };
                }
                block.Rings.Add(bounds);
            }
        }
        var blocks=index.Values.OrderBy(b=>b.MinimumDistanceSquared).ThenBy(b=>b.X).ThenBy(b=>b.Z).ToArray();
        int Priority(string category) => category == "mappedTree" ? 0 : category == "rowTree" ? 1 : 2;
        var pending=new SortedSet<(float3 point,string category,long serial)>(Comparer<(float3 point,string category,long serial)>.Create((a,b)=>
        {
            var compare=math.lengthsq(a.point.xz).CompareTo(math.lengthsq(b.point.xz));
            if(compare!=0)return compare;
            compare=Priority(a.category).CompareTo(Priority(b.category));
            return compare!=0?compare:a.serial.CompareTo(b.serial);
        }));
        long serial=0;
        foreach(var candidate in candidates)
        {
            token.ThrowIfCancellationRequested();
            if(math.all(math.isfinite(candidate.point)))pending.Add((candidate.point,candidate.category,serial++));
            else { Count(candidate.category+"Candidates"); Count("outsideOrInvalid"); }
        }
        var nextBlock=0; var woodlandCandidates=0; var woodlandCellsVisited=0; var nextProgress=timer.Elapsed.TotalSeconds+5;
        log.Info($"Woodland sampling: fixed {cell:0} m cells; lazy center-out placement; no tree-count limit. Global density reduction disabled.");
        void Generate(WoodlandBlock block)
        {
            // Coverage of the block plus an EdgeCells margin (cells' own jittered
            // points), so each tree knows how many cells it is from the woodland edge.
            // The margin also tests the neighbouring blocks' rings, so a boundary
            // shared with an adjacent wood is not mistaken for an open edge.
            var rings=new List<WoodlandRing>(block.Rings);
            for(var nz=block.Z-1;nz<=block.Z+1;nz++)for(var nx=block.X-1;nx<=block.X+1;nx++)
                if((nx!=block.X||nz!=block.Z)&&index.TryGetValue((nx,nz),out var neighbour))foreach(var r in neighbour.Rings)if(!rings.Contains(r))rings.Add(r);
            var span=blockCells+2*EdgeCells;var inside=new bool[span*span];var points=new float3[span*span];
            for(var k=0;k<span;k++)for(var j=0;j<span;j++)
            {
                token.ThrowIfCancellationRequested();
                var x=block.X*blockCells-EdgeCells+j;var z=block.Z*blockCells-EdgeCells+k;
                var seed=Hash(unchecked((uint)x*73856093u^(uint)z*19349663u));
                var point=new float3((x+(seed&65535)/65536f)*cell,0,(z+(Hash(seed)&65535)/65536f)*cell);points[k*span+j]=point;
                // The map border is not a forest edge.
                if(Math.Abs(point.x)>half||Math.Abs(point.z)>half){inside[k*span+j]=true;continue;}
                var margin=j<EdgeCells||k<EdgeCells||j>=EdgeCells+blockCells||k>=EdgeCells+blockCells;
                foreach(var ring in margin?rings:block.Rings)
                {
                    if(point.x<ring.MinX||point.x>ring.MaxX||point.z<ring.MinZ||point.z>ring.MaxZ||!Inside(point,ring.Points))continue;
                    var excluded=false;
                    foreach(var hole in holes[ring.Mask])
                        if(point.x>=hole.MinX&&point.x<=hole.MaxX&&point.z>=hole.MinZ&&point.z<=hole.MaxZ&&Inside(point,hole.Points)){excluded=true;break;}
                    if(!excluded){inside[k*span+j]=true;break;}
                }
            }
            for(var k=EdgeCells;k<EdgeCells+blockCells;k++)for(var j=EdgeCells;j<EdgeCells+blockCells;j++)
            {
                woodlandCellsVisited++;var point=points[k*span+j];
                if(Math.Abs(point.x)>half||Math.Abs(point.z)>half||!inside[k*span+j])continue;
                var edge=EdgeCells+1;
                for(var dz=-EdgeCells;dz<=EdgeCells;dz++)for(var dx=-EdgeCells;dx<=EdgeCells;dx++)
                    if(!inside[(k+dz)*span+j+dx])edge=Math.Min(edge,Math.Max(Math.Abs(dx),Math.Abs(dz)));
                woodlandCandidates++;
                if(!WoodlandKeep(point,edge*cell)){Count("thinned");continue;}
                pending.Add((point,"woodlandTree",serial++));
            }
        }
        while(pending.Count>0||nextBlock<blocks.Length)
        {
            token.ThrowIfCancellationRequested();
            if(timer.Elapsed.TotalSeconds>=nextProgress)
            { log.Info($"Preparing center-out trees: {output.Count:N0} accepted (uncapped); woodland blocks {nextBlock:N0}/{blocks.Length:N0} visited."); nextProgress=timer.Elapsed.TotalSeconds+5; }
            if(nextBlock<blocks.Length&&(pending.Count==0||blocks[nextBlock].MinimumDistanceSquared<=math.lengthsq(pending.Min.point.xz)))
            { Generate(blocks[nextBlock++]); continue; }
            var candidate=pending.Min; pending.Remove(candidate); Accept(candidate.point,candidate.category);
        }
        log.Info($"Forest clearance index: {corridors.CellCount:N0} cells (50m); {corridors.CandidateChecks:N0} nearby checks; {corridors.ExactChecks:N0} exact distance checks; original clearance distances retained.");
        log.Diagnostic($"FOREST INDEX cellMeters=50; cells={corridors.CellCount}; nearbyChecks={corridors.CandidateChecks}; exactChecks={corridors.ExactChecks}; trees={output.Count}; elapsedSeconds={timer.Elapsed.TotalSeconds:0.###}");
        int Total(string key) => counts.TryGetValue(key, out var value) ? value : 0;
        log.Diagnostic($"FOREST DISTRIBUTION order=center-out; globalDensityReduction=false; woodlandCellMeters={cell:0.###}; woodlandSpacingMeters={WoodlandSpacing}; mappedSpacingMeters={MappedSpacing}; totalDiscoveredCandidates={candidates.Count+woodlandCandidates}; mappedTreeCandidates={candidates.Count(c=>c.category=="mappedTree")}; mappedTreesAccepted={Total("mappedTreeAccepted")}; mappedRows={rows.Length}; rowCandidates={candidates.Count(c=>c.category=="rowTree")}; rowTreesAccepted={Total("rowTreeAccepted")}; woodlandCandidates={woodlandCandidates}; woodlandThinned={Total("thinned")}; woodlandTreesAccepted={Total("woodlandTreeAccepted")}; woodlandBlocksVisited={nextBlock}; woodlandBlocksTotal={blocks.Length}; woodlandCellsVisited={woodlandCellsVisited}; woodlandFullyEnumerated={nextBlock==blocks.Length}; outsideOrInvalid={Total("outsideOrInvalid")}; waterRejected={Total("waterRejected")}; networkRejected={Total("networkRejected")}; spacingRejected={Total("spacingRejected")}; unexaminedDiscoveredCandidates={pending.Count}; invalidRows={Total("invalidRows")}; totalPrepared={output.Count}; limit=none; limitReached=false; outermostAcceptedRadiusMeters={(output.Count==0?0:math.length(output[output.Count-1].xz)):0.##}; finalNativeTreeCountVerified=false");
        log.Evidence($"Trees prepared: {Total("mappedTreeAccepted"):N0} mapped individual trees, {Total("rowTreeAccepted"):N0} row trees, {Total("woodlandTreeAccepted"):N0} woodland trees ({Total("thinned"):N0} thinned for natural edges and gaps); {output.Count:N0} total. Spacing woodland {WoodlandSpacing:0} m, mapped/rows {MappedSpacing:0} m; {Total("spacingRejected"):N0} spacing rejections.");
        log.Count("Trees prepared", output.Count); log.Time("Vegetation preparation", timer.Elapsed.TotalSeconds);
        return output;
    }

    // Woodland cells within this many cells of an open cell count as edge.
    private const int EdgeCells = 3;

    // Natural thinning, deterministic per position. Edges feather out over three
    // cells (keep ~45% at the rim), but only where a slow 150 m pattern says so,
    // so some edges stay crisp; a 250 m pattern varies interior density by up
    // to 25%; and a 90 m pattern opens occasional small clearings.
    internal static bool WoodlandKeep(float3 point,float edgeDistance)
    {
        var x=point.x;var z=point.z;
        var featherHere=Smooth((Noise(x/150,z/150,0x9e3779b9u)-.35f)/.3f);
        var rim=Smooth(edgeDistance/(EdgeCells*WoodlandCellSize));
        var edge=1-featherHere*(1-(.45f+.55f*rim));
        var density=.75f+.25f*Noise(x/250,z/250,0x85ebca6bu);
        var clearing=Noise(x/90,z/90,0xc2b2ae35u)>.82f?.3f:1f;
        var keep=edge*density*clearing;
        var roll=(Hash(unchecked((uint)(int)Math.Floor(x*4)*2654435761u^(uint)(int)Math.Floor(z*4)*2246822519u))&65535)/65536f;
        return roll<keep;
    }
    private static float Smooth(float t){t=Math.Clamp(t,0,1);return t*t*(3-2*t);}
    // Bilinear value noise on an integer lattice, in [0,1].
    private static float Noise(float x,float z,uint salt)
    {
        var x0=(int)Math.Floor(x);var z0=(int)Math.Floor(z);var fx=x-x0;var fz=z-z0;
        float Lattice(int i,int k)=>(Hash(unchecked((uint)i*73856093u^(uint)k*19349663u^salt))&65535)/65535f;
        var u=fx*fx*(3-2*fx);var v=fz*fz*(3-2*fz);
        return math.lerp(math.lerp(Lattice(x0,z0),Lattice(x0+1,z0),u),math.lerp(Lattice(x0,z0+1),Lattice(x0+1,z0+1),u),v);
    }

    private static uint Hash(uint value) { unchecked { value ^= value >> 16; value *= 0x7feb352d; value ^= value >> 15; value *= 0x846ca68b; return value ^ (value >> 16); } }
    private static bool ClipRow(float3 a, float3 b, float half, out float entry, out float exit)
    {
        var lo = 0f; var hi = 1f;
        bool Axis(float start, float delta)
        {
            if (Math.Abs(delta) < .0001f) return Math.Abs(start) <= half;
            var t0 = (-half - start) / delta; var t1 = (half - start) / delta;
            lo = Math.Max(lo, Math.Min(t0, t1)); hi = Math.Min(hi, Math.Max(t0, t1)); return lo <= hi;
        }
        var hit = Axis(a.x, b.x - a.x) && Axis(a.z, b.z - a.z); entry = lo; exit = hi; return hit;
    }
    private static bool Inside(float3 point, IReadOnlyList<float3> ring)
    {
        var inside = false;
        for (var i = 0; i < ring.Count; i++)
        {
            var a = ring[i]; var b = ring[(i + 1) % ring.Count];
            if ((a.z > point.z) != (b.z > point.z) && point.x < (b.x - a.x) * (point.z - a.z) / (b.z - a.z) + a.x) inside = !inside;
        }
        return inside;
    }
}
