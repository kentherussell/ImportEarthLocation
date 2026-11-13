using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Local plan alignment in map metres. Only the compiler's ordinary-road
// strokes enter this pass; it retains node identities and never changes lanes.
public static class StreetGridAlignment
{
    public sealed record Street(int Id, SourceNodeKey[] Nodes, int[] At, double2[] Points, double Width);

    public readonly record struct Line(double2 Direction, double Offset)
    {
        public double2 Normal => new(-Direction.y, Direction.x);

        public double2 Project(double2 point) => point + Normal * (Offset - math.dot(point, Normal));
    }

    public sealed record Result(
        IReadOnlyDictionary<int, Line> Lines,
        IReadOnlyDictionary<SourceNodeKey, double2> Nodes);

    private sealed record Fit(Street Street, Line Line, double Length, double Angle, double Residual);

    private const double QuarterTurn = Math.PI / 2;
    private const double Cell = 400;
    private const double Neighbourhood = 450;
    private const double AngleTolerance = 2.5 * Math.PI / 180;

    public static Result Prepare(
        IReadOnlyList<Street> streets,
        HashSet<SourceNodeKey> fixedNodes,
        ImportLog log,
        CancellationToken token)
    {
        var original = new Dictionary<SourceNodeKey, double2>();
        var budget = new Dictionary<SourceNodeKey, double>();

        foreach (var street in streets)
            for (var k = 0; k < street.Nodes.Length; k++)
            {
                var node = street.Nodes[k];
                original[node] = street.Points[street.At[k]];
                var limit = Math.Min(12, Math.Max(4, street.Width / 2 + 2));
                budget[node] = budget.TryGetValue(node, out var old) ? Math.Min(old, limit) : limit;
            }

        var fits = new Dictionary<int, Fit>();

        foreach (var street in streets)
        {
            token.ThrowIfCancellationRequested();
            var points = street.Points;

            if (points.Length < 2)
                continue;

            var chord = points[^1] - points[0];
            var span = math.length(chord);

            if (span < 80)
                continue;

            var length = 0d;
            var direction = math.normalizesafe(chord);
            var sum = default(double2);

            for (var k = 1; k < points.Length; k++)
            {
                var d = points[k] - points[k - 1];
                var otherLength = math.length(d);
                length += otherLength;

                if (otherLength > 1e-6 && math.dot(d / otherLength, direction) > Math.Cos(10 * Math.PI / 180))
                    sum += d;
            }

            if (length <= 0 || span / length < .96 || math.lengthsq(sum) < 1)
                continue;

            direction = math.normalizesafe(sum);
            var normal = new double2(-direction.y, direction.x);
            var offsets = points.Select(point => math.dot(point, normal)).OrderBy(v => v).ToArray();
            var offset = offsets[offsets.Length / 2];
            var residual = points.Select(point => Math.Abs(math.dot(point, normal) - offset)).OrderBy(v => v).ToArray();

            if (residual[(int)(.9 * (residual.Length - 1))] > 3 || residual[^1] > Math.Min(12, street.Width / 2 + 2))
                continue;

            fits[street.Id] =
                new(street, new(direction, offset), length, Math.Atan2(direction.y, direction.x), residual[^1]);
        }

        var cells = new Dictionary<(int, int), HashSet<int>>();

        (int, int) Key(double2 point) => ((int)Math.Floor(point.x / Cell), (int)Math.Floor(point.y / Cell));

        foreach (var fit in fits.Values)
            foreach (var point in fit.Street.Points)
            {
                var key = Key(point);

                if (!cells.TryGetValue(key, out var ids))
                    cells[key] = ids = new();

                ids.Add(fit.Street.Id);
            }

        double Difference(double a, double b)
        {
            var d = a - b;
            return d - Math.Round(d / QuarterTurn) * QuarterTurn;
        }

        IEnumerable<int> Near(Fit fit)
        {
            var point = (fit.Street.Points[0] + fit.Street.Points[^1]) / 2;
            var (cx, cz) = Key(point);
            var seen = new HashSet<int>();

            for (var dz = -1; dz <= 1; dz++)
                for (var dx = -1; dx <= 1; dx++)
                    if (cells.TryGetValue((cx + dx, cz + dz), out var ids))
                        foreach (var id in ids)
                        {
                            if (!seen.Add(id))
                                continue;

                            var points = fits[id].Street.Points;
                            var ab = points[^1] - points[0];
                            var t =
                                math.clamp(math.dot(point - points[0], ab) / Math.Max(1e-9, math.lengthsq(ab)), 0, 1);

                            if (math.distance(point, points[0] + ab * t) <= Neighbourhood)
                                yield return id;
                        }
        }

        var neighbours = fits.Values.ToDictionary(fit => fit.Street.Id, fit => Near(fit).OrderBy(id => id).ToArray());
        var axes = new Dictionary<int, (double Angle, double Support)>();

        foreach (var fit in fits.Values)
        {
            var nearby =
                neighbours[fit.Street.Id]
                    .Select(id => fits[id])
                    .Where(otherFit => Math.Abs(Difference(otherFit.Angle, fit.Angle)) <= AngleTolerance)
                    .ToArray();

            bool Parallel(Fit otherFit) => Math.Abs(math.dot(otherFit.Line.Direction, fit.Line.Direction)) > .707;

            int Lines(bool parallel)
            {
                var normal = parallel ? fit.Line.Normal : fit.Line.Direction;
                var offsets =
                    nearby
                        .Where(otherFit => Parallel(otherFit) == parallel)
                        .Select(
                            otherFit => math.dot((otherFit.Street.Points[0] + otherFit.Street.Points[^1]) / 2, normal))
                        .OrderBy(v => v)
                        .ToArray();

                var count = 0;
                var last = double.NegativeInfinity;

                foreach (var offset in offsets)
                    if (offset - last >= Math.Max(20, fit.Street.Width + 2))
                    {
                        last = offset;
                        count++;
                    }

                return count;
            }

            // Two distinct streets on each axis distinguish a block grid from
            // several collinear pieces of one road or an isolated crossroads.
            if (Lines(true) < 2 || Lines(false) < 2)
                continue;

            var x = 0d;
            var y = 0d;
            var support = 0d;

            foreach (var otherFit in nearby)
            {
                var w = Math.Min(300, otherFit.Length);
                x += w * Math.Cos(4 * otherFit.Angle);
                y += w * Math.Sin(4 * otherFit.Angle);
                support += w;
            }

            axes[fit.Street.Id] = (Math.Atan2(y, x) / 4, support);
        }

        var snapped = new Dictionary<int, double>();

        foreach (var seed in axes.OrderByDescending(entry => entry.Value.Support).ThenBy(entry => entry.Key))
        {
            if (snapped.ContainsKey(seed.Key))
                continue;

            var queue = new List<int> { seed.Key };
            snapped[seed.Key] = seed.Value.Angle;

            for (var head = 0; head < queue.Count; head++)
                foreach (var id in neighbours[queue[head]])
                    if (!snapped.ContainsKey(id)
                        && axes.ContainsKey(id)
                        && Math.Abs(Difference(fits[id].Angle, seed.Value.Angle)) <= AngleTolerance)
                    {
                        snapped[id] = seed.Value.Angle;
                        queue.Add(id);
                    }
        }

        var lines = new Dictionary<int, Line>();

        foreach (var fit in fits.Values)
        {
            var line = fit.Line;

            if (snapped.TryGetValue(fit.Street.Id, out var axis))
            {
                var angle = axis + Math.Round((fit.Angle - axis) / QuarterTurn) * QuarterTurn;
                var direction = new double2(Math.Cos(angle), Math.Sin(angle));
                var n = new double2(-direction.y, direction.x);
                var values = fit.Street.Points.Select(point => math.dot(point, n)).OrderBy(v => v).ToArray();
                line = new(direction, values[values.Length / 2]);
            }
            else if (fit.Residual > 1.5)
                continue;

            var pinned =
                fit.Street.Nodes
                    .Where(fixedNodes.Contains)
                    .Select(nodeKey => math.dot(original[nodeKey], line.Normal))
                    .ToArray();

            if (pinned.Length > 0)
                line = line with { Offset = pinned[0] };

            if (pinned.Any(v => Math.Abs(v - line.Offset) > .01)
                || fit.Street.Points.Any(
                    point => math.distance(point, line.Project(point)) > Math.Min(12, fit.Street.Width / 2 + 2)))
                continue;

            lines[fit.Street.Id] = line;
        }

        var atNode = new Dictionary<SourceNodeKey, List<int>>();

        foreach (var fit in fits.Values)
            foreach (var node in fit.Street.Nodes)
            {
                if (!atNode.TryGetValue(node, out var list))
                    atNode[node] = list = new();

                list.Add(fit.Street.Id);
            }

        var proposed = new Dictionary<SourceNodeKey, double2>();

        while (lines.Count > 0)
        {
            token.ThrowIfCancellationRequested();
            proposed.Clear();

            foreach (var entry in atNode)
            {
                var point = original[entry.Key];

                if (fixedNodes.Contains(entry.Key))
                {
                    proposed[entry.Key] = point;
                    continue;
                }

                var ids = entry.Value.Where(lines.ContainsKey).Distinct().ToArray();

                if (ids.Length == 0)
                    continue;

                var target = lines[ids[0]].Project(point);
                var cross = 0d;

                for (var a = 0; a < ids.Length; a++)
                    for (var b = a + 1; b < ids.Length; b++)
                    {
                        var firstLine = lines[ids[a]];
                        var secondLine = lines[ids[b]];
                        var otherCross = NetworkDesignChecks.Cross(firstLine.Normal, secondLine.Normal);

                        if (Math.Abs(otherCross) <= Math.Max(.5, cross))
                            continue;

                        cross = Math.Abs(otherCross);
                        target =
                            new(
                                (firstLine.Offset * secondLine.Normal.y - firstLine.Normal.y * secondLine.Offset)
                                    / otherCross,
                                (firstLine.Normal.x * secondLine.Offset - firstLine.Offset * secondLine.Normal.x)
                                    / otherCross);
                    }

                if (math.distance(target, point) > budget[entry.Key]
                    || ids.Any(id => Math.Abs(math.dot(target, lines[id].Normal) - lines[id].Offset) > .01))
                    target = point;

                proposed[entry.Key] = target;
            }

            var rejected =
                lines.Keys
                    .Where(
                        id => fits[id].Street.Nodes.Any(
                            nodeKey => !proposed.TryGetValue(nodeKey, out var otherPoint)
                                || Math.Abs(math.dot(otherPoint, lines[id].Normal) - lines[id].Offset) > .01))
                    .ToArray();

            if (rejected.Length == 0)
                break;

            foreach (var id in rejected)
                lines.Remove(id);
        }

        if (lines.Count == 0)
            proposed.Clear();

        var moved =
            proposed
                .Where(entry => math.distancesq(entry.Value, original[entry.Key]) > 1e-12)
                .ToDictionary(entry => entry.Key, entry => entry.Value);

        log.Diagnostic(
            $"VISUAL COMPILER street alignment eligibleStrokes={streets.Count}; straightCandidates={fits.Count}; gridCandidates={snapped.Count}; alignedStrokes={lines.Count}; movedNodes={moved.Count}; localAxes=true; highwayAndStructureNodesFixed=true");

        return new(lines, moved);
    }
}
