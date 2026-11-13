using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Without streets, OSM's short non-motorway connectors lose what they joined:
// a median U-turn between a divided road's two carriageways, or a stub that
// led to a street that is not imported. Both render as odd little roads. Only
// when streets are off, chains of non-motorway road (connected through plain
// degree-two nodes) are removed when they are
//   a crossover: under 200 m, both ends on opposite one-way carriageways of
//                one road (same name or ref);
//   a stub:      under 150 m, one end attached, the other a dead end away
//                from the map border;
//   a fragment:  under 150 m with both ends dead.
// Motorways and motorway links, including dead-end ramps, are never removed.
// With streets on, only dead-end main-road stubs under 80 m go: they read as
// nubs on the network and connect nothing the import keeps.
public static class NetworkStubs
{
    private const double CrossoverLength = 200, StubLength = 150, StreetStubLength = 80, BorderMargin = 50;

    public static IReadOnlyList<PreparedSegment> Prune(
        IReadOnlyList<PreparedSegment> source,
        ImportRequest request,
        ImportLog log)
    {
        var streets = (request.RoadLevel & RoadImportLevel.Streets) != 0;
        var half = request.MapSize.WidthMeters / 2 - BorderMargin;

        var incident = new Dictionary<SourceNodeKey, List<int>>();

        for (var i = 0; i < source.Count; i++)
            foreach (var key in new[] { SourceNodeKey.Start(source[i]), SourceNodeKey.End(source[i]) })
            {
                if (!incident.TryGetValue(key, out var list))
                    incident[key] = list = new();

                list.Add(i);
            }

        bool Candidate(PreparedSegment segment)
            => segment.Kind == NetworkKind.Road
                && !segment.Classification.StartsWith("motorway", StringComparison.Ordinal)
                && (!streets
                    || !NativeAssetKeys.IsHighway(segment)
                        && !segment.ProtectedLink
                        && !segment.Roundabout
                        && !NetworkStructure.IsStructure(segment));

        var chainOf = new int[source.Count];
        Array.Fill(chainOf, -1);
        var chains = new List<List<int>>();

        for (var i = 0; i < source.Count; i++)
        {
            if (chainOf[i] >= 0 || !Candidate(source[i]))
                continue;

            var chain = new List<int>();
            var pending = new List<int> { i };
            chainOf[i] = chains.Count;

            for (var k = 0; k < pending.Count; k++)
            {
                var segment = source[pending[k]];
                chain.Add(pending[k]);

                foreach (var key in new[] { SourceNodeKey.Start(segment), SourceNodeKey.End(segment) })
                {
                    var arms = incident[key];

                    if (arms.Count != 2)
                        continue;

                    foreach (var j in arms)
                        if (chainOf[j] < 0 && Candidate(source[j]))
                        {
                            chainOf[j] = chains.Count;
                            pending.Add(j);
                        }
                }
            }

            chains.Add(chain);
        }

        var removed = new bool[source.Count];
        int crossovers = 0, stubs = 0, fragments = 0;

        foreach (var chain in chains)
        {
            var members = new HashSet<int>(chain);
            var length =
                chain.Sum(
                    i => NetworkDesignChecks.HorizontalArcLength(
                        new(
                            source[i].CourseStart,
                            source[i].CourseControl1,
                            source[i].CourseControl2,
                            source[i].CourseEnd)));

            if (length >= CrossoverLength)
                continue;

            // End nodes: chain nodes not shared by two chain members.
            var ends =
                chain
                    .SelectMany(i => new[] { SourceNodeKey.Start(source[i]), SourceNodeKey.End(source[i]) })
                    .GroupBy(nodeKey => nodeKey)
                    .Where(group => incident[group.Key].Count(j => members.Contains(j)) == 1)
                    .Select(group => group.Key)
                    .ToArray();

            var attached =
                ends.Select(nodeKey => incident[nodeKey].Where(j => !members.Contains(j)).ToArray()).ToArray();

            bool Dead(int e)
                => attached[e].Length == 0
                    && Math.Abs(source[incident[ends[e]][0]].Start.x) < half
                    && Math.Abs(source[incident[ends[e]][0]].Start.z) < half
                    && Math.Abs(source[incident[ends[e]][0]].End.x) < half
                    && Math.Abs(source[incident[ends[e]][0]].End.z) < half;

            var dead = Enumerable.Range(0, ends.Length).Count(Dead);
            string kind = "";

            if (streets)
            {
                if (ends.Length == 2 && dead == 1 && length < StreetStubLength)
                    kind = "stub";
            }
            else if (ends.Length == 2 && dead == 2 && length < StubLength)
                kind = "fragment";
            else if (ends.Length == 2 && dead == 1 && length < StubLength)
                kind = "stub";
            else if (ends.Length == 2 && dead == 0 && Crossover(attached[0], attached[1]))
                kind = "crossover";

            if (kind.Length == 0)
                continue;

            foreach (var i in chain)
                removed[i] = true;

            if (kind == "crossover")
                crossovers++;
            else if (kind == "stub")
                stubs++;
            else
                fragments++;
        }

        // Two attachments on opposite one-way carriageways of one named road.
        bool Crossover(int[] left, int[] right)
        {
            foreach (var i in left)
                foreach (var j in right)
                {
                    var firstSegment = source[i];
                    var secondSegment = source[j];

                    if (!firstSegment.OneWay
                        || !secondSegment.OneWay
                        || firstSegment.Kind != NetworkKind.Road
                        || secondSegment.Kind != NetworkKind.Road)
                        continue;

                    var same =
                        firstSegment.Name.Length > 0 && firstSegment.Name == secondSegment.Name
                            || firstSegment.Reference.Length > 0 && firstSegment.Reference == secondSegment.Reference;

                    if (same
                        && math.dot(
                            math.normalizesafe((firstSegment.End - firstSegment.Start).xz),
                            math.normalizesafe((secondSegment.End - secondSegment.Start).xz))
                            < -.5f)
                        return true;
                }

            return false;
        }

        var total = crossovers + stubs + fragments;
        log.Count("Odd connector roads removed", total);
        log.Evidence(
            $"Odd connector roads removed: {total:N0}; median U-turn crossovers {crossovers:N0}, dead-end stubs {stubs:N0}, fragments {fragments:N0}; motorways and ramps kept.");

        return total == 0 ? source : source.Where((_, i) => !removed[i]).ToArray();
    }
}
