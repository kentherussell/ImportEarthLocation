using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

public static class NetworkNormalizer
{
    public static float Tolerance(NetworkKind kind) => kind == NetworkKind.Rail ? 0.75f : 2f;
    private static string NodeKey(PreparedSegment segment, long node) => $"{segment.Kind}:{segment.Bridge}:{segment.Tunnel}:{segment.Layer}:{node}";

    public static IReadOnlyList<PreparedSegment> Normalize(IReadOnlyList<PreparedSegment> input, ImportLog log, CancellationToken token, double mapWidth = double.PositiveInfinity)
    {
        var clock = Stopwatch.StartNew();
        var degree = new Dictionary<string, int>();
        foreach (var segment in input)
        {
            foreach (var id in new[] { segment.StartNodeId, segment.EndNodeId })
            { var key = NodeKey(segment, id); degree.TryGetValue(key, out var count); degree[key] = count + 1; }
        }
        log.Time("Topology/cleanup", clock.Elapsed.TotalSeconds);
        clock.Restart();
        var output = new List<PreparedSegment>();
        // Never join across OSM way endpoints or tag changes. Degree is measured globally,
        // so intersections within a way are protected too, including small real spurs.
        foreach (var group in input.GroupBy(segment => (segment.WayId, segment.Kind, segment.Classification, segment.Lanes, segment.OneWay, segment.Bridge, segment.Tunnel, segment.Layer)))
        {
            if (group.First().ProtectedLink)
            {
                foreach (var segment in group) { token.ThrowIfCancellationRequested(); output.Add(segment with { SourceGeometry = new[] { segment.Start, segment.End } }); }
                log.Count("Protected ramps preserved", group.Count());
                continue;
            }
            var chain = new List<PreparedSegment>();
            void Flush()
            {
                if (chain.Count == 0) return;
                var piece = new List<PreparedSegment>(); var length = 0f;
                void Emit()
                {
                    if (piece.Count == 0) return;
                    var points = new List<float3> { piece[0].Start };
                    points.AddRange(piece.Select(segment => segment.End));
                    Fit(points, piece, output, token, (float)(mapWidth / 2)); piece.Clear(); length = 0;
                }
                foreach (var segment in chain)
                {
                    if (piece.Count > 0)
                    {
                        var previous = piece[piece.Count - 1];
                        if (degree[NodeKey(segment, segment.StartNodeId)] != 2 || segment.StartNodeId < 0 ||
                            math.dot(math.normalizesafe(previous.End - previous.Start), math.normalizesafe(segment.End - segment.Start)) < 0.866f ||
                            length > 200f || piece.Count >= 64) Emit();
                    }
                    piece.Add(segment); length += math.distance(segment.Start, segment.End);
                }
                Emit();
                chain.Clear();
            }
            foreach (var segment in group)
            {
                token.ThrowIfCancellationRequested();
                if (chain.Count > 0 && chain[0].StartNodeId == segment.EndNodeId && segment.OneWay)
                    chain.Insert(0, segment);
                else if (chain.Count == 0 || chain[chain.Count - 1].EndNodeId == segment.StartNodeId)
                    chain.Add(segment);
                else { Flush(); chain.Add(segment); }
            }
            Flush();
        }
        log.Count("Segments merged", Math.Max(0, input.Count - output.Count));
        var unique = new List<PreparedSegment>();
        var equivalents = new Dictionary<string, List<PreparedSegment>>(StringComparer.Ordinal);
        foreach (var segment in output)
        {
            token.ThrowIfCancellationRequested();
            var a = segment.OneWay ? segment.StartNodeId : Math.Min(segment.StartNodeId, segment.EndNodeId);
            var b = segment.OneWay ? segment.EndNodeId : Math.Max(segment.StartNodeId, segment.EndNodeId);
            var key = $"{segment.Kind}:{segment.Classification}:{segment.Lanes}:{segment.OneWay}:{segment.Bridge}:{segment.Tunnel}:{segment.Layer}:{a}:{b}";
            if (!equivalents.TryGetValue(key, out var candidates)) equivalents[key] = candidates = new List<PreparedSegment>();
            // Only equivalent topology endpoints AND nearly coincident curves qualify.
            // Parallel tracks/carriageways with different OSM node IDs never qualify.
            var duplicate = candidates.Any(other => Enumerable.Range(0, 33).All(i =>
                math.distance(Evaluate(segment, i / 32f).xz, Evaluate(other, (segment.StartNodeId == other.StartNodeId ? i : 32 - i) / 32f).xz) < 0.05f));
            if (duplicate) { log.Count("Duplicates/artifacts removed"); continue; }
            candidates.Add(segment); unique.Add(segment);
        }
        log.Time("Smoothing", clock.Elapsed.TotalSeconds);
        return unique;
    }

    private static void Fit(List<float3> points, IReadOnlyList<PreparedSegment> source, List<PreparedSegment> output, CancellationToken token, float halfWidth)
    {
        token.ThrowIfCancellationRequested();
        if (source.Count == 1) { output.Add(source[0] with { SourceGeometry = points.ToArray() }); return; }
        var first = source[0]; var last = source[source.Count - 1];
        var distance = math.distance(points[0], points[points.Count - 1]);
        var simplified = Simplify(points, Tolerance(first.Kind) * 0.1f);
        var c1 = points[0] + math.normalizesafe(simplified[1] - simplified[0]) * distance / 3f;
        var c2 = points[points.Count - 1] - math.normalizesafe(simplified[simplified.Count - 1] - simplified[simplified.Count - 2]) * distance / 3f;
        var candidate = first with { End = last.End, EndNodeId = last.EndNodeId, Control1 = c1, Control2 = c2, SourceGeometry = points.ToArray() };
        if (distance > 0.05f && Math.Abs(c1.x) <= halfWidth && Math.Abs(c1.z) <= halfWidth &&
            Math.Abs(c2.x) <= halfWidth && Math.Abs(c2.z) <= halfWidth && WithinTolerance(candidate)) { output.Add(candidate); return; }
        // A rejected fit increases the section count, never deletes source geometry.
        var split = source.Count / 2;
        Fit(points.Take(split + 1).ToList(), source.Take(split).ToArray(), output, token, halfWidth);
        Fit(points.Skip(split).ToList(), source.Skip(split).ToArray(), output, token, halfWidth);
    }

    private static List<float3> Simplify(IReadOnlyList<float3> points, float tolerance)
    {
        var keep = new SortedSet<int> { 0, points.Count - 1 };
        void Visit(int start, int end)
        {
            var farthest = tolerance; var split = -1;
            for (var i = start + 1; i < end; i++)
            { var distance = DistanceToSegment(points[i], points[start], points[end]); if (distance > farthest) { farthest = distance; split = i; } }
            if (split < 0) return;
            keep.Add(split); Visit(start, split); Visit(split, end);
        }
        Visit(0, points.Count - 1); return keep.Select(i => points[i]).ToList();
    }

    public static float3 Evaluate(PreparedSegment segment, float t)
    {
        var c1 = segment.Control1 ?? math.lerp(segment.Start, segment.End, 1f / 3f);
        var c2 = segment.Control2 ?? math.lerp(segment.Start, segment.End, 2f / 3f);
        var u = 1 - t;
        return u * u * u * segment.Start + 3 * u * u * t * c1 + 3 * u * t * t * c2 + t * t * t * segment.End;
    }

    public static float DistanceToSegment(float3 point, float3 start, float3 end)
    {
        point.y = start.y = end.y = 0;
        var delta = end - start;
        var t = math.lengthsq(delta) > 0 ? math.clamp(math.dot(point - start, delta) / math.lengthsq(delta), 0, 1) : 0;
        return math.distance(point, start + t * delta);
    }

    public static bool WithinTolerance(PreparedSegment segment)
    {
        if (segment.SourceGeometry.Count < 2) return false;
        // Exact straight horizontal geometry remains exact even when only its vertical
        // controls change. Do not reject long straight sections on a sampling allowance.
        if (segment.SourceGeometry.All(p => DistanceToSegment(p, segment.Start, segment.End) < .01f) &&
            DistanceToSegment(segment.Control1 ?? segment.Start, segment.Start, segment.End) < .01f &&
            DistanceToSegment(segment.Control2 ?? segment.End, segment.Start, segment.End) < .01f) return true;
        var samples = new float3[257];
        for (var i = 0; i < samples.Length; i++) samples[i] = Evaluate(segment, i / 256f);
        float Distance(float3 point, IReadOnlyList<float3> line)
        { var closest = float.MaxValue; for (var i = 1; i < line.Count; i++) closest = Math.Min(closest, DistanceToSegment(point, line[i - 1], line[i])); return closest; }
        var c1 = segment.Control1 ?? math.lerp(segment.Start, segment.End, 1f / 3f);
        var c2 = segment.Control2 ?? math.lerp(segment.Start, segment.End, 2f / 3f);
        var a = segment.Start; var d = segment.End; a.y = c1.y = c2.y = d.y = 0;
        var arcBound = math.distance(a, c1) + math.distance(c1, c2) + math.distance(c2, d);
        var tolerance = Tolerance(segment.Kind);
        // Control-polygon length bounds between-sample movement; reserve that error
        // instead of trusting vertex-only comparisons which can cut a hairpin.
        var allowance = tolerance - 3 * arcBound / 512f;
        if (allowance <= 0) return false;
        foreach (var point in samples) if (Distance(point, segment.SourceGeometry) > allowance) return false;
        for (var i = 1; i < segment.SourceGeometry.Count; i++)
        {
            var start = segment.SourceGeometry[i - 1]; var end = segment.SourceGeometry[i];
            var steps = Math.Max(1, (int)Math.Ceiling(math.distance(start, end) / (tolerance / 2)));
            for (var j = 0; j <= steps; j++)
                if (Distance(math.lerp(start, end, j / (float)steps), samples) > tolerance * 0.75f) return false;
        }
        return true;
    }

    public static void Validate(IReadOnlyList<PreparedSegment> segments, double width, CancellationToken token)
    {
        foreach (var segment in segments)
        {
            token.ThrowIfCancellationRequested();
            if (!math.all(math.isfinite(segment.Start)) || !math.all(math.isfinite(segment.End)) || math.distance(segment.Start, segment.End) < 0.01f)
                throw new InvalidOperationException("Normalized network contains invalid geometry.");
            for (var i = 0; i <= 32; i++)
            {
                var p = Evaluate(segment, i / 32f);
                if (!math.all(math.isfinite(p)) || Math.Abs(p.x) > width / 2 + 0.02 || Math.Abs(p.z) > width / 2 + 0.02)
                    throw new InvalidOperationException("Normalized network is outside map bounds or has invalid controls.");
            }
            if (segment.HorizontalParent != null)
            {
                // Profile refinement subdivides an already validated horizontal curve exactly.
                if (!WithinTolerance(segment.HorizontalParent)) throw new InvalidOperationException("Profile parent exceeded geographic tolerance.");
                for(var i=0;i<=32;i++) if(math.distance(Evaluate(segment,i/32f).xz,Evaluate(segment.HorizontalParent,math.lerp(segment.ParentStartT,segment.ParentEndT,i/32f)).xz)>.02f)
                    throw new InvalidOperationException("Vertical refinement changed the original horizontal alignment.");
            }
            else if (segment.Control1.HasValue && !WithinTolerance(segment))
                throw new InvalidOperationException("A fitted network exceeded the geographic error tolerance.");
        }
    }
}
