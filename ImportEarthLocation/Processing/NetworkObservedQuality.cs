using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Source-qualified paths are supplied by the native observer. No proximity
// search may substitute a different road for a missing lineage path.
public static class NetworkObservedQuality
{
    public static void Paired(
        IReadOnlyList<PreparedSegment> sections,
        Func<PreparedSegment, IReadOnlyList<Cubic3d>?> observe,
        ImportLog log, CancellationToken token, string scope)
    {
        var checkedPairs = 0;
        var expected = 0;
        var failed = 0;
        var unknown = 0;
        var worst = 0d;
        var affected = 0d;
        var witnesses = new List<object>();

        foreach (var group in sections
            .Where(segment => segment.PairedProfileId != 0)
            .GroupBy(segment => (segment.PairedProfileId,
            Begin: Math.Round(Math.Min(segment.PairedReferenceBegin, segment.PairedReferenceEnd), 6),
                End: Math.Round(Math.Max(segment.PairedReferenceBegin, segment.PairedReferenceEnd), 6))))
        {
            token.ThrowIfCancellationRequested();
            expected++;
            var pair = group.ToArray();

            if (pair.Length != 2)
            {
                unknown++;
                continue;
            }

            var firstSegment = pair[0];
            var secondSegment = pair[1];
            var ac = observe(firstSegment);
            var bc = observe(secondSegment);

            if (ac == null || bc == null || ac.Count == 0 || bc.Count == 0)
            {
                unknown++;
                continue;
            }

            var sameDirection =
                Math.Sign(firstSegment.PairedReferenceEnd - firstSegment.PairedReferenceBegin)
                    == Math.Sign(secondSegment.PairedReferenceEnd - secondSegment.PairedReferenceBegin);

            var pairWorst = 0d;
            var unavailable = false;

            for (var station = 0; station <= 16; station++)
            {
                var t = station / 16d;
                var pa =
                    NetworkDesignChecks.Evaluate(
                        new(
                            firstSegment.CourseStart,
                            firstSegment.CourseControl1,
                            firstSegment.CourseControl2,
                            firstSegment.CourseEnd),
                        t);

                var pb =
                    NetworkDesignChecks.Evaluate(
                        new(
                            secondSegment.CourseStart,
                            secondSegment.CourseControl1,
                            secondSegment.CourseControl2,
                            secondSegment.CourseEnd),
                        sameDirection ? t : 1 - t);

                var ya = Height(ac, pa.xz);
                var yb = Height(bc, pb.xz);

                if (!ya.HasValue || !yb.HasValue)
                {
                    unavailable = true;
                    continue;
                }

                pairWorst = Math.Max(pairWorst, Math.Abs(ya.Value - yb.Value));
            }

            worst = Math.Max(worst, pairWorst);

            if (pairWorst > .25)
            {
                failed++;
                affected +=
                    NetworkDesignChecks.HorizontalArcLength(
                        new(
                            firstSegment.CourseStart,
                            firstSegment.CourseControl1,
                            firstSegment.CourseControl2,
                            firstSegment.CourseEnd));
            }

            if (unavailable)
                unknown++;
            else
                checkedPairs++;

            if ((unavailable || pairWorst > .25) && witnesses.Count < 128)
                witnesses.Add(new
                {
                    firstSegment.PairedProfileId,
                    firstSegment.WayId,
                    otherWay = secondSegment.WayId,
                    firstSegment.StartNodeId,
                    firstSegment.EndNodeId,
                    maximumObservedDifferenceMeters = pairWorst,
                    unavailableStations = unavailable,
                    stations = 17,
                    projectionToleranceMeters = .05,
                    continuousCertificate = false
                });
        }

        log.Quality(new("native-paired", "paired-height", scope, checkedPairs, failed, unknown, worst, affected,
            "17 common planned-reference stations on exact qualified native paths; <=0.25m observed height difference; XZ mismatch/ambiguous height unavailable; sampled, not continuous")
        { Expected = expected });

        if (log.DiagnosticsEnabled)
            log.DiagnosticRecords(scope + "-paired-height-witnesses.ndjson", witnesses, critical: true);
    }

    private static double? Height(IReadOnlyList<Cubic3d> curves, double2 point)
    {
        var best = double.PositiveInfinity;
        var height = 0d;
        var ambiguous = false;

        foreach (var curve in curves)
        {
            if (!NetworkDesignChecks.Finite(curve))
                continue;

            for (var i = 0; i < 32; i++)
            {
                var low = i / 32d;
                var high = (i + 1) / 32d;
                var otherPoint = NetworkDesignChecks.Evaluate(curve, low);
                var secondPoint = NetworkDesignChecks.Evaluate(curve, high);
                var span = secondPoint.xz - otherPoint.xz;
                var fraction =
                    math.clamp(math.dot(point - otherPoint.xz, span) / Math.Max(1e-20, math.lengthsq(span)), 0, 1);

                var t = low + (high - low) * fraction;

                for (var step = 0; step < 6; step++)
                {
                    var x = NetworkDesignChecks.Evaluate(curve, t);
                    var d = NetworkDesignChecks.Derivative(curve, t).xz;
                    var denominator =
                        math.dot(d, d) + math.dot(x.xz - point, NetworkDesignChecks.SecondDerivative(curve, t).xz);

                    if (!(denominator > 1e-15))
                        break;

                    t = math.clamp(t - math.dot(x.xz - point, d) / denominator, low, high);
                }

                var value = NetworkDesignChecks.Evaluate(curve, t);
                var distance = math.distance(value.xz, point);

                if (distance < best - 1e-6)
                {
                    best = distance;
                    height = value.y;
                    ambiguous = false;
                }
                else if (Math.Abs(distance - best) <= 1e-6 && Math.Abs(height - value.y) > .05)
                    ambiguous = true;
            }
        }

        return best <= .05 && !ambiguous ? height : null;
    }

    public static void Ports(SourceTopologyManifest? selected, IReadOnlyList<PreparedSegment> sections,
        Func<SourceNodeKey, int?> degree, ImportLog log, CancellationToken token, string stage, string scope)
    {
        var expected = new Dictionary<SourceNodeKey, HashSet<SourceNodeKey>>();

        if (selected != null)
            foreach (var connection in selected.Connections)
            {
                void Add(SourceNodeKey firstNodeKey, SourceNodeKey secondNodeKey)
                {
                    if (!expected.TryGetValue(firstNodeKey, out var peers))
                        expected[firstNodeKey] = peers = new();

                    peers.Add(secondNodeKey);
                }

                Add(connection.From, connection.To);
                Add(connection.To, connection.From);
            }

        var actual =
            sections
                .SelectMany(
                    segment => new[] {
                        new SourceNodeKey(segment.Kind, segment.StartNodeId),
                        new SourceNodeKey(segment.Kind, segment.EndNodeId)
                        })
                .Distinct()
                .ToArray();

        var aliases =
            sections
                .SelectMany(
                    segment => segment.SourceNodeAliases.Select(
                        alias => (
                            Source: new SourceNodeKey(segment.Kind, alias.SourceNodeId),
                            Canonical: new SourceNodeKey(segment.Kind, alias.CanonicalNodeId))))
            .GroupBy(item => item.Canonical)
                .ToDictionary(group => group.Key, group => group.Select(item => item.Source).Distinct().ToArray());

        var canonical =
            sections
                .SelectMany(
                    segment => segment.SourceNodeAliases.Select(
                        alias => (
                            Source: new SourceNodeKey(segment.Kind, alias.SourceNodeId),
                            Canonical: new SourceNodeKey(segment.Kind, alias.CanonicalNodeId))))
            .GroupBy(item => item.Source)
                .Where(group => group.Select(item => item.Canonical).Distinct().Count() == 1)
                .ToDictionary(group => group.Key, group => group.First().Canonical);

        var declaredTerminals =
            new HashSet<SourceNodeKey>(
                sections
                    .SelectMany(
                        segment => new[]{
                            (Node:new SourceNodeKey(segment.Kind,segment.StartNodeId),Terminal:segment.StartTerminal),
            (Node:new SourceNodeKey(segment.Kind,segment.EndNodeId),Terminal:segment.EndTerminal)})
                .Where(item => item.Terminal)
                .Select(item => item.Node));

        var failed = 0;
        var unknown = 0;
        var terminals = 0;
        var selectionBoundaries = 0;
        var witnesses = new List<object>();

        foreach (var node in actual)
        {
            token.ThrowIfCancellationRequested();
            var observed = degree(node);

            if (!observed.HasValue)
            {
                unknown++;
                continue;
            }

            if (observed.Value > 1)
                continue;

            if (observed.Value == 0)
            {
                failed++;

                if (witnesses.Count < 128)
                    witnesses.Add(
                        new { node.Kind, node.Id, observedDegree = 0, classification = "no-live-incident-edge" });

                continue;
            }

            var sourceKeys = aliases.TryGetValue(node, out var mapped) ? mapped.Concat(new[] { node }) : new[] { node };
            var required =
                sourceKeys
                    .Where(expected.ContainsKey)
                    .SelectMany(nodeKey => expected[nodeKey])
                    .Select(nodeKey => canonical.TryGetValue(nodeKey, out var otherNodeKey) ? otherNodeKey : nodeKey)
                    .Where(nodeKey => nodeKey != node)
                    .Distinct()
                    .Count();

            var declaredTerminal = declaredTerminals.Contains(node);

            if (required > 1)
            {
                failed++;

                if (witnesses.Count < 128)
                    witnesses.Add(
                        new
                        {
                            node.Kind,
                            node.Id,
                            observedDegree = observed,
                            selectedPeers = required,
                            declaredTerminal,
                            classification = "nonterminal-orphan"
                        });
            }
            else if (declaredTerminal)
                terminals++;
            else if (selected != null && required == 1)
                selectionBoundaries++;
            else
                unknown++;
        }

        log.Quality(new(stage, "nonterminal-orphan-ports", scope, actual.Length, failed, unknown, failed, 0,
            "Selected-source connectivity and exact aliases; degree one is not automatically a broken road"));

        log.Quality(
            new(stage, "declared-terminal-ports", scope, terminals, 0, 0, 0, 0, "Explicit prepared terminal markers"));

        log.Quality(new(stage, "selected-graph-boundary-ports", scope, selectionBoundaries, 0, 0, 0, 0,
            "One-neighbour boundary in the selected/clipped source graph; omission reason is not inferred from proximity"));

        if (log.DiagnosticsEnabled)
            log.DiagnosticRecords(scope + "-orphan-witnesses.ndjson", witnesses, critical: true);
    }
}
