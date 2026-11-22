using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// OSM's trunk means "major road", not limited access: Rt 50 in Virginia is trunk,
// with 48 signals and 109 side roads over 34 km. No tag separates them
// (control_of_access is unused on the test maps, and expressway=yes is on US 50),
// so the road itself decides. Trunk is cut at every signal, crossing and at-grade
// side road. A stretch stays a highway if it has interchanges, has no at-grade
// junction at all, or is mostly bridge or tunnel; length alone doesn't count, since
// signals on an arterial can be kilometres apart. The rest is drawn as a regular road. Ramps stay highways, and
// these roads keep their carriageways: see AvenueConsolidator.
public static class NetworkLimitedAccess
{
    private const double InterchangeStretch = 1000, BoundedStretch = 1800, StructureShare = .5;
    private const int MinimumInterchanges = 2;

    // A regular-road link is removed when its roads stay connected by a path no longer than
    // this many times the link, and at least the minimum (metres).
    private const double DetourFactor = 4, MinimumDetour = 400;

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
        var crossingClasses = new Dictionary<long, List<string>>();

        foreach (var way in dataset.Ways.Values)
        {
            if (!way.Tags.TryGetValue("highway", out var text) || !AtGradeClass(text) || NetworkEvidenceStructure(way))
                continue;

            for (var k = 0; k < way.NodeIds.Length; k++)
            {
                var id = way.NodeIds[k];

                if (!armsAt.ContainsKey(id))
                    continue;

                crossing.Add(id);

                if (!log.DiagnosticsEnabled)
                    continue;

                if (!crossingClasses.TryGetValue(id, out var classes))
                    crossingClasses[id] = classes = new();

                var position = k == 0 || k == way.NodeIds.Length - 1 ? "end" : "through";
                classes.Add(text + ":" + position + (way.Tags.TryGetValue("oneway", out var oneWay) ? ":oneway=" + oneWay : ""));
            }
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

        var motorwayNodes = new HashSet<long>();

        foreach (var segment in source)
            if (segment.Kind == NetworkKind.Road && segment.Classification == "motorway")
            {
                motorwayNodes.Add(segment.StartNodeId);
                motorwayNodes.Add(segment.EndNodeId);
            }

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
        var keptStretches = new List<int[]>();

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
            // Boulevard flipped between assets every couple of kilometres: its runs between
            // at-grade ends are 0.2-1.6 km. Expressways that end at a roundabout or signal
            // run longer between them: Suitland Parkway 1.9-4 km, Paris's D 170 and N 19
            // 2.8-3.4 km, Cairo's 26 July Corridor 3.4-3.9 km (report 120).
            var cuts =
                stretch.SelectMany(i => new[] { source[i].StartNodeId, source[i].EndNodeId }).Distinct().Count(Cut);

            // A stretch with no at-grade junction at either end or along it is reached only
            // by ramps or from a motorway, so it is limited access whatever its ramp count:
            // Cairo's desert roads leave a motorway and run 6 km with one ramp.
            var freeway =
                length >= InterchangeStretch
                    && (cuts == 0
                        || interchanges >= MinimumInterchanges && (cuts < 2 || length >= BoundedStretch));

            if (log.DiagnosticsEnabled)
            {
                var first = source[stretch.First()];
                var ends =
                    stretch
                        .SelectMany(i => new[] { source[i].StartNodeId, source[i].EndNodeId })
                        .GroupBy(id => id)
                        .Where(group => group.Count() == 1)
                        .Select(group => group.Key)
                        .ToArray();

                string CutReason(long id)
                    => NetworkEvidence.NodeIs(dataset, id, "highway", "traffic_signals") ? "signals"
                        : NetworkEvidence.NodeIs(dataset, id, "highway", "crossing") ? "crossing"
                        : NetworkEvidence.NodeIs(dataset, id, "highway", "stop") ? "stop"
                        : "side-road(" + string.Join("+", crossingClasses.GetValueOrDefault(id) ?? new List<string>()) + ")";

                var cutReasons =
                    stretch
                        .SelectMany(i => new[] { source[i].StartNodeId, source[i].EndNodeId })
                        .Distinct()
                        .Where(Cut)
                        .Select(id => CutReason(id) + "@" + id)
                        .ToArray();

                log.Diagnostic(
                    $"LIMITED ACCESS stretch way={first.WayId}; name={first.Name}; ref={first.Reference}; lengthMeters={length:0}; interchanges={interchanges}; cuts={cuts}; structureShare={structure / Math.Max(1, length):0.00}; motorwayEnds={ends.Count(motorwayNodes.Contains)}; kept={freeway || structure >= StructureShare * length}; at={first.Start.x:0},{first.Start.z:0}; cutAt={string.Join(",", cutReasons)}");
            }

            if (freeway || structure >= StructureShare * length)
            {
                keptStretches.Add(stretch.ToArray());
                continue;
            }

            foreach (var i in stretch)
            {
                result[i] = source[i] with { Classification = "primary" };
                converted++;
            }
        }

        var agreed = AgreeCarriageways(result, keptStretches);

        foreach (var stretch in keptStretches)
            if (agreed.Contains(stretch))
                foreach (var i in stretch)
                {
                    result[i] = source[i] with { Classification = "primary" };
                    converted++;
                }
            else
                kept += stretch.Length;

        log.Count("Trunk stretches drawn as regular road beside their regular carriageway", agreed.Count);
        log.Count("Trunk sections kept as highway", kept);
        log.Count("Trunk sections drawn as regular road", converted);
        log.Evidence(
            $"Limited access: {kept:N0} trunk sections stay highways; {converted:N0} without limited access are drawn as regular roads.");

        return result;
    }

    // The two carriageways of a divided road are one road, but each is decided on its own
    // above, and OSM often tags them apart. On Montreal's Route 132 one carriageway is trunk
    // and the other primary, and a carriageway whose side roads all join its partner reads
    // as limited access. The partner is then a regular road and merges into an avenue, while
    // this one stays a highway beside it. A kept stretch is drawn as a regular road when at
    // least half its length runs opposite a regular one-way road (primary, after the
    // conversion above) of its own name or ref, at most CarriagewaySeparation metres away.
    private const float CarriagewaySeparation = 40, BesideShare = .5f;

    private static HashSet<int[]> AgreeCarriageways(PreparedSegment[] sections, List<int[]> keptStretches)
    {
        const float Cell = 64;

        static string Identity(PreparedSegment segment)
            => segment.Name.Trim().Length > 0
                ? "name:" + segment.Name.Trim().ToUpperInvariant()
                : segment.Reference.Trim().Length > 0
                    ? "ref:" + segment.Reference.Trim().ToUpperInvariant()
                    : "";

        static (int, int) Key(float3 point) => ((int)Math.Floor(point.x / Cell), (int)Math.Floor(point.z / Cell));

        var regular = new Dictionary<(int, int), List<int>>();

        for (var i = 0; i < sections.Length; i++)
        {
            var segment = sections[i];

            if (segment.Kind != NetworkKind.Road
                || !segment.OneWay
                || segment.Classification != "primary"
                || Identity(segment).Length == 0)
                continue;

            var length = math.distance(segment.Start.xz, segment.End.xz);
            var steps = Math.Max(1, (int)Math.Ceiling(length / (Cell / 2)));

            for (var k = 0; k <= steps; k++)
            {
                var key = Key(math.lerp(segment.Start, segment.End, k / (float)steps));

                if (!regular.TryGetValue(key, out var list))
                    regular[key] = list = new List<int>();

                if (list.Count == 0 || list[^1] != i)
                    list.Add(i);
            }
        }

        bool Beside(PreparedSegment segment)
        {
            var identity = Identity(segment);
            var direction = math.normalizesafe(segment.End.xz - segment.Start.xz);
            var middle = math.lerp(segment.Start, segment.End, .5f);
            var (x, z) = Key(middle);

            if (identity.Length == 0)
                return false;

            for (var dx = -1; dx <= 1; dx++)
                for (var dz = -1; dz <= 1; dz++)
                {
                    if (!regular.TryGetValue((x + dx, z + dz), out var list))
                        continue;

                    foreach (var j in list)
                    {
                        var other = sections[j];
                        var otherDirection = math.normalizesafe(other.End.xz - other.Start.xz);

                        if (math.dot(direction, otherDirection) < -.85f
                            && Identity(other) == identity
                            && Distance(middle.xz, other.Start.xz, other.End.xz) <= CarriagewaySeparation)
                            return true;
                    }
                }

            return false;
        }

        var agreed = new HashSet<int[]>();

        foreach (var stretch in keptStretches)
        {
            var length = 0f;
            var beside = 0f;

            foreach (var i in stretch)
            {
                var segment = sections[i];
                var segmentLength = math.distance(segment.Start.xz, segment.End.xz);
                length += segmentLength;

                if (segment.OneWay && !NetworkStructure.IsStructure(segment) && Beside(segment))
                    beside += segmentLength;
            }

            if (length > 0 && beside >= BesideShare * length)
                agreed.Add(stretch);
        }

        return agreed;
    }

    private static float Distance(float2 point, float2 start, float2 end)
    {
        var d = end - start;
        var t = math.clamp(math.dot(point - start, d) / Math.Max(.001f, math.lengthsq(d)), 0, 1);

        return math.distance(point, start + t * d);
    }

    // A link whose roads are all drawn as regular roads is the slip lane or turning road of
    // an ordinary junction, and the game's own junction already makes that turn. Such links
    // are taken as connected groups. A group touching a motorway, a motorway link or a trunk
    // kept as highway is an interchange and stays. Any other group is removed when the roads
    // it joins stay connected without it within a short detour; otherwise it is their only
    // connection (a ramp down to a road passing underneath) and becomes a regular road of its
    // own class. Runs after Apply, so a trunk drawn as a regular road no longer counts.
    //
    // Connected is not enough (report 121 §11–14): a car must still be able to make every
    // movement the group made. Each ordered pair of attachments the group joined, driving in
    // its legal direction and obeying OSM turn restrictions, needs a replacement route of the
    // same direction and turns within the detour limit, on the network left after *every*
    // removal, so two groups cannot each count on the other. A group with a movement that
    // fails is kept and drawn as a regular road. Adding a kept group back only adds roads, so
    // the routes found for the groups still removed stay valid.
    public static IReadOnlyList<PreparedSegment> RegularLinks(
        IReadOnlyList<PreparedSegment> source,
        OsmDataset dataset,
        ImportLog log)
    {
        static bool Highway(PreparedSegment segment)
            => segment.Classification.StartsWith("motorway", StringComparison.Ordinal)
                || segment.Classification == "trunk";

        bool Link(int i)
            => source[i].ProtectedLink && !source[i].Classification.StartsWith("motorway", StringComparison.Ordinal);

        var incident = new Dictionary<SourceNodeKey, List<int>>();

        for (var i = 0; i < source.Count; i++)
        {
            if (source[i].Kind != NetworkKind.Road)
                continue;

            foreach (var key in new[] { SourceNodeKey.Start(source[i]), SourceNodeKey.End(source[i]) })
            {
                if (!incident.TryGetValue(key, out var list))
                    incident[key] = list = new();

                list.Add(i);
            }
        }

        var groupOf = new int[source.Count];
        Array.Fill(groupOf, -1);
        var groups = new List<List<int>>();

        for (var i = 0; i < source.Count; i++)
        {
            if (groupOf[i] >= 0 || !Link(i))
                continue;

            var group = new List<int> { i };
            groupOf[i] = groups.Count;

            for (var k = 0; k < group.Count; k++)
            {
                var member = source[group[k]];

                foreach (var key in new[] { SourceNodeKey.Start(member), SourceNodeKey.End(member) })
                    foreach (var j in incident[key])
                        if (groupOf[j] < 0 && Link(j))
                        {
                            groupOf[j] = groups.Count;
                            group.Add(j);
                        }
            }

            groups.Add(group);
        }

        var lengths = new double[source.Count];
        Array.Fill(lengths, double.NaN);

        // Each section's arc length is read many times by the searches; it is computed once.
        double Length(int i)
        {
            if (double.IsNaN(lengths[i]))
                lengths[i] = NetworkVerticalProfiles.ArcLength(source[i]);

            return lengths[i];
        }

        var removed = new bool[source.Count];
        var result = source.ToArray();
        var tentative = new List<(int Group, List<SourceNodeKey> Attachments, double Limit)>();
        int interchanges = 0, regular = 0;

        for (var g = 0; g < groups.Count; g++)
        {
            var group = groups[g];
            var attachments = new List<SourceNodeKey>();
            var interchange = false;

            foreach (var key in group
                .SelectMany(i => new[] { SourceNodeKey.Start(source[i]), SourceNodeKey.End(source[i]) })
                .Distinct())
            {
                var others = incident[key].Where(j => groupOf[j] != g).ToArray();

                if (others.Length == 0)
                    continue;

                attachments.Add(key);
                interchange |= others.Any(j => Highway(source[j]));
            }

            if (interchange)
            {
                interchanges++;
                continue;
            }

            // A loose link piece is left to the dead-end stages.
            if (attachments.Count < 2)
                continue;

            var groupLength = group.Sum(i => Length(i));
            var limit = Math.Max(MinimumDetour, DetourFactor * groupLength);

            // A quick undirected test first: a group whose roads are not even connected
            // without it is their only connection.
            if (Reachable(attachments, limit, g))
            {
                tentative.Add((g, attachments, limit));
                continue;
            }

            DrawAsRegularRoad(group);
            regular++;
        }

        void DrawAsRegularRoad(List<int> group)
        {
            foreach (var i in group)
            {
                var parent = source[i].Classification.Substring(0, source[i].Classification.Length - "_link".Length);
                result[i] = source[i] with { Classification = parent == "trunk" ? "primary" : parent };
            }
        }

        // Every tentative removal is applied before any movement is checked.
        var withdrawn = new bool[groups.Count];

        foreach (var candidate in tentative)
            withdrawn[candidate.Group] = true;

        var rules = TurnRules.Read(dataset);
        int slipLanes = 0, kept = 0, movements = 0;

        foreach (var (g, attachments, limit) in tentative)
        {
            var failure = "";

            foreach (var origin in attachments)
            {
                var targets = Search(origin, attachments, double.PositiveInfinity, i => groupOf[i] == g, proving: false);
                movements += targets.Count;

                if (targets.Count == 0)
                    continue;

                var replaced = Search(
                    origin,
                    targets.Keys.ToList(),
                    limit,
                    i => groupOf[i] < 0 || !withdrawn[groupOf[i]],
                    proving: true);

                foreach (var target in targets.Keys)
                    if (failure.Length == 0 && !replaced.ContainsKey(target))
                        failure = $"{origin.Id}->{target.Id}";

                if (failure.Length > 0)
                    break;
            }

            if (failure.Length == 0)
            {
                foreach (var i in groups[g])
                    removed[i] = true;

                slipLanes++;
                continue;
            }

            DrawAsRegularRoad(groups[g]);
            kept++;

            if (log.DiagnosticsEnabled)
            {
                var first = source[groups[g][0]];
                log.Diagnostic(
                    $"REGULAR LINK kept way={first.WayId}; sections={groups[g].Count}; lostMovement={failure}; limitMeters={limit:0}; at={first.Start.x:0},{first.Start.z:0}");
            }
        }

        // Shortest legal driving distance from the origin to each target, through sections
        // the filter allows. A car follows a section from start to end, or either way when it
        // is two-way (BuildSegments has already reversed oneway=-1 ways); it may not turn back
        // on the section it came along, and each turn at a node must be one the OSM turn
        // restrictions allow. The first section leaving the origin is free, as is the
        // arrival: the movement is the pair of attachments, not the lanes around them.
        // When proving a replacement route, unresolved restrictions also forbid their turn.
        Dictionary<SourceNodeKey, double> Search(
            SourceNodeKey origin,
            IReadOnlyCollection<SourceNodeKey> targets,
            double limitMeters,
            Func<int, bool> allowed,
            bool proving)
        {
            var wanted = new HashSet<SourceNodeKey>(targets);
            wanted.Remove(origin);
            var found = new Dictionary<SourceNodeKey, double>();
            var distance = new Dictionary<int, double>();
            var queue = new SortedSet<(double Distance, int Arc)>();

            // Arc 2i drives section i from start to end; arc 2i + 1 drives it backwards.
            SourceNodeKey Head(int arc)
                => arc % 2 == 0 ? SourceNodeKey.End(source[arc / 2]) : SourceNodeKey.Start(source[arc / 2]);

            IEnumerable<int> Leaving(SourceNodeKey node)
            {
                foreach (var i in incident[node])
                {
                    if (!allowed(i))
                        continue;

                    if (SourceNodeKey.Start(source[i]) == node)
                        yield return 2 * i;

                    if (!source[i].OneWay && SourceNodeKey.End(source[i]) == node)
                        yield return 2 * i + 1;
                }
            }

            void Offer(int arc, double reached)
            {
                if (reached > limitMeters || distance.TryGetValue(arc, out var known) && known <= reached)
                    return;

                distance[arc] = reached;
                queue.Add((reached, arc));
            }

            foreach (var arc in Leaving(origin))
                Offer(arc, Length(arc / 2));

            while (queue.Count > 0 && found.Count < wanted.Count)
            {
                var (reached, arc) = queue.Min;
                queue.Remove(queue.Min);

                if (reached > distance[arc])
                    continue;

                var node = Head(arc);

                if (wanted.Contains(node) && !found.ContainsKey(node))
                    found[node] = reached;

                var fromWay = source[arc / 2].WayId;

                foreach (var next in Leaving(node))
                {
                    if (next / 2 == arc / 2)
                        continue;

                    if (!rules.Allowed(fromWay, node.Id, source[next / 2].WayId, proving))
                        continue;

                    Offer(next, reached + Length(next / 2));
                }
            }

            return found;
        }

        // Shortest road distance from the first attachment, without this group, until every other
        // attachment is reached or the limit is passed. Distances are section arc lengths.
        bool Reachable(List<SourceNodeKey> attachments, double limit, int group)
        {
            var targets = new HashSet<SourceNodeKey>(attachments.Skip(1));
            var distance = new Dictionary<SourceNodeKey, double> { [attachments[0]] = 0 };
            var queue = new SortedSet<(double Distance, long Node, int Kind)>
            {
                (0, attachments[0].Id, (int)attachments[0].Kind)
            };

            while (queue.Count > 0 && targets.Count > 0)
            {
                var (reached, nodeId, kind) = queue.Min;
                queue.Remove(queue.Min);
                var key = new SourceNodeKey((NetworkKind)kind, nodeId);

                if (reached > distance[key])
                    continue;

                targets.Remove(key);

                foreach (var i in incident[key])
                {
                    if (groupOf[i] == group)
                        continue;

                    var other =
                        SourceNodeKey.Start(source[i]) == key
                            ? SourceNodeKey.End(source[i])
                            : SourceNodeKey.Start(source[i]);

                    var next = reached + Length(i);

                    if (next > limit || distance.TryGetValue(other, out var known) && known <= next)
                        continue;

                    distance[other] = next;
                    queue.Add((next, other.Id, (int)other.Kind));
                }
            }

            return targets.Count == 0;
        }

        log.Count("Regular-road links removed", slipLanes);
        log.Count("Regular-road links drawn as regular roads", regular + kept);
        log.Count("Regular-road links kept for a movement", kept);
        log.Evidence(
            $"Links: {interchanges:N0} groups at highways stay ramps; {slipLanes:N0} slip-lane groups at regular junctions removed; {regular:N0} drawn as regular roads as the only connection; {kept:N0} kept as regular roads because a driving movement ({movements:N0} checked) had no legal detour without them.");

        return slipLanes == 0 ? result : result.Where((_, i) => !removed[i]).ToArray();
    }

    // OSM turn restrictions for cars, as rules on the turn from one way onto another at a node.
    // A rule with a single via node, a plain from and to way, and a restriction that applies
    // to cars at all times is enforced everywhere. Anything else (a via way, a time condition)
    // is unresolved: it does not take a movement away from the original links, but a
    // replacement route may not use its turn, so a doubtful detour keeps the link.
    private sealed class TurnRules
    {
        private sealed class Rule
        {
            public readonly HashSet<long> Forbidden = new();
            public HashSet<long>? Only;
        }

        private readonly Dictionary<(long Way, long Node), Rule> enforced = new();
        private readonly Dictionary<(long Way, long Node), Rule> unresolved = new();

        public bool Allowed(long fromWay, long node, long toWay, bool proving)
            => Obeys(enforced, fromWay, node, toWay) && (!proving || Obeys(unresolved, fromWay, node, toWay));

        private static bool Obeys(Dictionary<(long Way, long Node), Rule> rules, long fromWay, long node, long toWay)
        {
            if (!rules.TryGetValue((fromWay, node), out var rule))
                return true;

            return !rule.Forbidden.Contains(toWay) && (rule.Only == null || rule.Only.Contains(toWay));
        }

        public static TurnRules Read(OsmDataset dataset)
        {
            var rules = new TurnRules();

            foreach (var relation in dataset.Relations)
            {
                if (!relation.Tags.TryGetValue("type", out var type) || type != "restriction")
                    continue;

                if (relation.Tags.TryGetValue("except", out var except)
                    && except.Split(';').Any(mode => mode.Trim() is "motorcar" or "motor_vehicle" or "vehicle"))
                    continue;

                var conditional = false;
                var restriction = CarRestriction(relation.Tags);

                if (restriction == null)
                {
                    restriction = ConditionalCarRestriction(relation.Tags);
                    conditional = restriction != null;
                }

                if (restriction == null)
                    continue;

                var only = restriction.StartsWith("only_", StringComparison.Ordinal);

                if (!only && !restriction.StartsWith("no_", StringComparison.Ordinal))
                    continue;

                var from = relation.OrderedMembers.Where(member => member.Role == "from" && member.Type == "way").ToArray();
                var to = relation.OrderedMembers.Where(member => member.Role == "to" && member.Type == "way").ToArray();
                var via = relation.OrderedMembers.Where(member => member.Role == "via").ToArray();

                if (from.Length != 1 || to.Length != 1 || via.Length == 0)
                    continue;

                if (via.Length == 1 && via[0].Type == "node")
                {
                    // Turning back on the same way is already ruled out by the searches, and
                    // forbidding the way itself would also forbid driving straight on.
                    if (!only && from[0].Id == to[0].Id)
                        continue;

                    Add(conditional ? rules.unresolved : rules.enforced, from[0].Id, via[0].Id, to[0].Id, only);
                    continue;
                }

                // A via way: only the first turn, from the from way onto the first via way, is
                // known for certain, so it carries the rule as an unresolved one.
                if (via[0].Type != "way"
                    || !dataset.Ways.TryGetValue(from[0].Id, out var fromWay)
                    || !dataset.Ways.TryGetValue(via[0].Id, out var viaWay))
                    continue;

                var shared = fromWay.NodeIds.Intersect(viaWay.NodeIds).ToArray();

                if (shared.Length == 1)
                    Add(rules.unresolved, from[0].Id, shared[0], via[0].Id, only);
            }

            return rules;
        }

        private static void Add(
            Dictionary<(long Way, long Node), Rule> rules,
            long fromWay,
            long node,
            long toWay,
            bool only)
        {
            if (!rules.TryGetValue((fromWay, node), out var rule))
                rules[(fromWay, node)] = rule = new Rule();

            if (only)
                (rule.Only ??= new HashSet<long>()).Add(toWay);
            else
                rule.Forbidden.Add(toWay);
        }

        private static string? CarRestriction(IReadOnlyDictionary<string, string> tags)
        {
            foreach (var key in new[] { "restriction:motorcar", "restriction:motor_vehicle", "restriction" })
                if (tags.TryGetValue(key, out var value))
                    return value.Trim();

            return null;
        }

        // A conditional value reads "no_left_turn @ (Mo-Fr 07:00-09:00)".
        private static string? ConditionalCarRestriction(IReadOnlyDictionary<string, string> tags)
        {
            foreach (var key in new[]
                {
                    "restriction:motorcar:conditional",
                    "restriction:motor_vehicle:conditional",
                    "restriction:conditional"
                })
                if (tags.TryGetValue(key, out var value))
                    return value.Split('@')[0].Trim();

            return null;
        }
    }

    private static bool NetworkEvidenceStructure(OsmWay way)
        => way.Tags.TryGetValue("bridge", out var text) && text != "no"
            || way.Tags.TryGetValue("tunnel", out var otherText) && otherText != "no";
}
