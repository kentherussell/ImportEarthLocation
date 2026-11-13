using System.Collections.Generic;
using System.IO;
using System.Linq;
using ImportEarthLocation.Core;

namespace ImportEarthLocation.Processing;

// OSM's trunk means "major road", not limited access: Rt 50 in Virginia is trunk,
// with 48 signals and 109 side roads over 34 km. No tag separates them
// (control_of_access is unused on the test maps, and expressway=yes is on US 50),
// so the road itself decides. Trunk is cut at every signal, crossing and at-grade
// side road. A stretch stays a highway if it has interchanges or is mostly bridge
// or tunnel; length alone doesn't count, since signals on an arterial can be
// kilometres apart. The rest is drawn as a regular road. Ramps stay highways, and
// these roads keep their carriageways: see AvenueConsolidator.
public static class NetworkLimitedAccess
{
    private const double InterchangeStretch = 1000, BoundedStretch = 4000, StructureShare = .5;
    private const int MinimumInterchanges = 2;

    public static bool Converted(PreparedSegment segment)
        => segment.Classification == "primary"
            && segment.RawSourceTags.TryGetValue("highway", out var text)
            && text == "trunk";

    public static IReadOnlyList<PreparedSegment> Apply(
        IReadOnlyList<PreparedSegment> source,
        OsmDataset dataset,
        ImportLog log)
    {
        var trunk = new List<int>();
        var armsAt = new Dictionary<long, List<int>>();

        for (var i = 0; i < source.Count; i++)
        {
            var segment = source[i];

            if (segment.Kind != NetworkKind.Road || segment.Classification != "trunk")
                continue;

            trunk.Add(i);

            foreach (var id in new[] { segment.StartNodeId, segment.EndNodeId })
            {
                if (!armsAt.TryGetValue(id, out var list))
                    armsAt[id] = list = new();

                list.Add(i);
            }
        }

        if (trunk.Count == 0)
            return source;

        static bool AtGradeClass(string text)
            => text is "primary" or "secondary" or "tertiary" or "unclassified" or "residential" or "living_street" or "road";

        var crossing = new HashSet<long>();

        foreach (var way in dataset.Ways.Values)
        {
            if (!way.Tags.TryGetValue("highway", out var text) || !AtGradeClass(text) || NetworkEvidenceStructure(way))
                continue;

            foreach (var id in way.NodeIds)
                if (armsAt.ContainsKey(id))
                    crossing.Add(id);
        }

        bool Cut(long id) => (crossing.Contains(id) && armsAt[id].All(a => !NetworkStructure.IsStructure(source[a])))
            || NetworkEvidence.NodeIs(dataset, id, "highway", "traffic_signals")
                || NetworkEvidence.NodeIs(dataset, id, "highway", "crossing")
                || NetworkEvidence.NodeIs(dataset, id, "highway", "stop");

        var ramps = new Dictionary<long, int>();

        foreach (var segment in source)
            if (segment.Kind == NetworkKind.Road && segment.ProtectedLink)
                foreach (var id in new[] { segment.StartNodeId, segment.EndNodeId })
                    if (armsAt.ContainsKey(id))
                        ramps[id] = ramps.GetValueOrDefault(id) + 1;

        var parent = trunk.ToDictionary(i => i, i => i);

        int Root(int i)
        {
            while (parent[i] != i)
                i = parent[i] = parent[parent[i]];

            return i;
        }

        foreach (var (id, arms) in armsAt)
            if (!Cut(id))
                for (var k = 1; k < arms.Count; k++)
                    parent[Root(arms[k])] = Root(arms[0]);

        var result = source.ToArray();
        var kept = 0;
        var converted = 0;

        foreach (var stretch in trunk.GroupBy(Root))
        {
            double Length(int i) => NetworkVerticalProfiles.ArcLength(source[i]);

            var length = stretch.Sum(Length);
            var structure = stretch.Where(i => NetworkStructure.IsStructure(source[i])).Sum(Length);
            var interchanges =
                stretch
                    .SelectMany(i => new[] { source[i].StartNodeId, source[i].EndNodeId })
                    .Distinct()
                    .Count(ramps.ContainsKey);

            // Signals at both ends of a short run with interchanges make it an arterial
            // with a grade separation or two, not a freeway. Left as highway, Arlington
            // Boulevard flipped between assets every couple of kilometres.
            var cuts =
                stretch.SelectMany(i => new[] { source[i].StartNodeId, source[i].EndNodeId }).Distinct().Count(Cut);

            var freeway =
                length >= InterchangeStretch
                    && interchanges >= MinimumInterchanges
                    && (cuts < 2 || length >= BoundedStretch);

            if (freeway || structure >= StructureShare * length)
            {
                kept += stretch.Count();
                continue;
            }

            foreach (var i in stretch)
            {
                result[i] = source[i] with { Classification = "primary" };
                converted++;
            }
        }

        log.Count("Trunk sections kept as highway", kept);
        log.Count("Trunk sections drawn as regular road", converted);
        log.Evidence(
            $"Limited access: {kept:N0} trunk sections stay highways; {converted:N0} without limited access are drawn as regular roads.");

        return result;
    }

    private static bool NetworkEvidenceStructure(OsmWay way)
        => way.Tags.TryGetValue("bridge", out var text) && text != "no"
            || way.Tags.TryGetValue("tunnel", out var otherText) && otherText != "no";
}
