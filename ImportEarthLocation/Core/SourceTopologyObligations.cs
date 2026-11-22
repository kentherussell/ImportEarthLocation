using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;

namespace ImportEarthLocation.Core;

// Selected, clipped source topology, captured BEFORE consolidation. Deliberate
// acquisition/policy omissions are outside this graph, not unexplained losses.
public static class SourceTopologyObligations
{
    public static SourceTopologyManifest Capture(IReadOnlyList<PreparedSegment> sections)
    {
        var connections = new List<SourceConnection>();
        var ports = Sorted(Ports(sections, connections));
        var firstWays = new long[connections.Count];
        var order = new int[connections.Count];

        for (var i = 0; i < connections.Count; i++)
        {
            firstWays[i] = connections[i].Ways.Count > 0 ? connections[i].Ways[0] : 0;
            order[i] = i;
        }

        // Ordered by start, end and first way; connections alike in all three keep the order
        // they were traced in, as a stable sort would leave them.
        int Compare(int first, int second)
        {
            var firstConnection = connections[first];
            var secondConnection = connections[second];
            var result = ((int)firstConnection.From.Kind).CompareTo((int)secondConnection.From.Kind);

            if (result == 0)
                result = firstConnection.From.Id.CompareTo(secondConnection.From.Id);

            if (result == 0)
                result = firstConnection.To.Id.CompareTo(secondConnection.To.Id);

            if (result == 0)
                result = firstWays[first].CompareTo(firstWays[second]);

            return result != 0 ? result : first.CompareTo(second);
        }

        Array.Sort(order, Compare);
        var sortedConnections = new SourceConnection[order.Length];

        for (var i = 0; i < order.Length; i++)
            sortedConnections[i] = connections[order[i]];

        return new(Array.AsReadOnly(ports), Array.AsReadOnly(sortedConnections));
    }

    // The ports by kind, then identifier; no two are alike, so any sort gives this one order.
    private static SourceNodeKey[] Sorted(HashSet<SourceNodeKey> ports)
    {
        var sorted = new SourceNodeKey[ports.Count];
        ports.CopyTo(sorted);
        Array.Sort(sorted, CompareNodeKeys);

        return sorted;
    }

    private static int CompareNodeKeys(SourceNodeKey first, SourceNodeKey second)
    {
        var result = ((int)first.Kind).CompareTo((int)second.Kind);

        return result != 0 ? result : first.Id.CompareTo(second.Id);
    }

    // The ports of the sections of every kind, without their connections.
    public static HashSet<SourceNodeKey> Ports(IReadOnlyList<PreparedSegment> sections)
        => Ports(sections, null);

    // The ports of the sections, and their connections when a list is given for them.
    // Transfer needs only the ports, on every consolidation pass.
    private static HashSet<SourceNodeKey> Ports(
        IReadOnlyList<PreparedSegment> sections,
        List<SourceConnection>? connections)
    {
        var incident = new Dictionary<SourceNodeKey, List<int>>();

        void Touch(SourceNodeKey key, int index)
        {
            if (!incident.TryGetValue(key, out var list))
                incident[key] = list = new();

            list.Add(index);
        }

        for (var i = 0; i < sections.Count; i++)
        {
            Touch(SourceNodeKey.Start(sections[i]), i);
            Touch(SourceNodeKey.End(sections[i]), i);
        }

        var ports = new HashSet<SourceNodeKey>();

        foreach (var pair in incident)
        {
            var arms = pair.Value;
            var first = sections[arms[0]];
            var port = arms.Count != 2;

            for (var arm = 0; !port && arm < arms.Count; arm++)
                port = BreaksRun(first, sections[arms[arm]], pair.Key);

            if (port)
                ports.Add(pair.Key);
            else if (first.OneWay
                && (SourceNodeKey.Start(first) == pair.Key)
                    == (SourceNodeKey.Start(sections[arms[1]]) == pair.Key))
                ports.Add(pair.Key);
        }

        var visited = new bool[sections.Count];

        void Trace(SourceNodeKey start, int seed)
        {
            var node = start;
            var index = seed;
            var forward = true;
            var backward = true;
            var ways = connections != null ? new HashSet<long>() : null;

            while (!visited[index])
            {
                visited[index] = true;
                var segment = sections[index];
                var along = SourceNodeKey.Start(segment) == node;

                if (segment.OneWay)
                {
                    forward &= along;
                    backward &= !along;
                }

                if (ways != null)
                {
                    if (segment.SourceWayIds.Count > 0)
                        ways.UnionWith(segment.SourceWayIds);
                    else
                        ways.Add(segment.WayId);
                }

                node = along ? SourceNodeKey.End(segment) : SourceNodeKey.Start(segment);

                if (ports.Contains(node) || node == start)
                    break;

                var arms = incident[node];
                index = arms[0] == index ? arms[1] : arms[0];
            }

            if (connections == null || ways == null)
                return;

            var sortedWays = new long[ways.Count];
            ways.CopyTo(sortedWays);
            Array.Sort(sortedWays);
            var provenance = Array.AsReadOnly(sortedWays);

            if (forward)
                connections.Add(new(start, node, provenance));

            if (backward)
                connections.Add(new(node, start, provenance));
        }

        // Every node inside a run between ports has exactly two arms, so a trace from either
        // end of a run visits all of it. Which runs are visited therefore does not depend on
        // the order of the ports; that order matters only to the connections recorded.
        IEnumerable<SourceNodeKey> starts = connections != null ? Sorted(ports) : ports;

        foreach (var key in starts)
            foreach (var i in incident[key])
                if (!visited[i])
                    Trace(key, i);

        for (var i = 0; i < sections.Count; i++)
            if (!visited[i])
            {
                var seam = SourceNodeKey.Start(sections[i]);
                ports.Add(seam);
                Trace(seam, i);
            }

        return ports;
    }

    // Whether this arm of a two-armed node ends the run through it, compared with the first arm.
    private static bool BreaksRun(PreparedSegment first, PreparedSegment section, SourceNodeKey key)
        => first.Kind == NetworkKind.Road && section.Lanes != first.Lanes
            || section.OneWay != first.OneWay
            || section.Bridge != first.Bridge
            || section.Tunnel != first.Tunnel
            || section.Layer != first.Layer
            || section.StartTerminal && SourceNodeKey.Start(section) == key
            || section.EndTerminal && SourceNodeKey.End(section) == key
            || section.StartMiniRoundabout && SourceNodeKey.Start(section) == key
            || section.EndMiniRoundabout && SourceNodeKey.End(section) == key;

    // Consume the consolidator's exact remapping, never recover aliases by
    // proximity or a repaired height. Previously declared aliases follow it.
    public static IReadOnlyList<PreparedSegment> Transfer(
        IReadOnlyList<PreparedSegment> before,
        IReadOnlyList<PreparedSegment> after,
        NetworkKind kind, IReadOnlyDictionary<long, long> remap)
    {
        if (remap.Count == 0)
            return after;

        // Node keys carry their kind, so the ports of one kind are found from its sections alone.
        var sectionsOfKind = new List<PreparedSegment>();

        foreach (var segment in before)
            if (segment.Kind == kind)
                sectionsOfKind.Add(segment);

        return Transfer(before, after, kind, remap, Ports(sectionsOfKind, null));
    }

    // As above, with the ports of the sections before already known (of this kind or of every kind).
    public static IReadOnlyList<PreparedSegment> Transfer(
        IReadOnlyList<PreparedSegment> before,
        IReadOnlyList<PreparedSegment> after,
        NetworkKind kind,
        IReadOnlyDictionary<long, long> remap,
        HashSet<SourceNodeKey> beforePorts)
    {
        if (remap.Count == 0)
            return after;

        long Resolve(long id)
        {
            // Most identifiers are not remapped at all, so the cycle guard is made only when needed.
            HashSet<long>? seen = null;

            while (remap.TryGetValue(id, out var next) && next != id)
            {
                seen ??= new HashSet<long>();

                if (!seen.Add(id))
                    break;

                id = next;
            }

            return id;
        }

        var aliases = new Dictionary<long, SourceNodeAlias>();

        foreach (var segment in before)
        {
            if (segment.Kind != kind)
                continue;

            foreach (var alias in segment.SourceNodeAliases)
                aliases[alias.SourceNodeId] = alias with { CanonicalNodeId = Resolve(alias.CanonicalNodeId) };
        }

        foreach (var pair in remap)
            aliases[pair.Key] =
                new(
                    pair.Key,
                    Resolve(pair.Value),
                    beforePorts.Contains(new SourceNodeKey(kind, pair.Key))
                        || aliases.TryGetValue(pair.Key, out var old) && old.Mandatory);

        var byNode = new Dictionary<long, List<SourceNodeAlias>>();

        foreach (var alias in aliases.Values)
        {
            if (!byNode.TryGetValue(alias.CanonicalNodeId, out var group))
                byNode[alias.CanonicalNodeId] = group = new();

            group.Add(alias);
        }

        // A section with no aliases of its own and none arriving at its nodes keeps an
        // empty list, so it is passed on as it is rather than copied.
        bool Untouched(PreparedSegment segment)
            => segment.SourceNodeAliases.Count == 0
                && !byNode.ContainsKey(segment.StartNodeId)
                && !byNode.ContainsKey(segment.EndNodeId);

        var result = new PreparedSegment[after.Count];

        // The latest alias of each source node: its own aliases first, then those arriving at its start
        // and its end, a later one replacing an earlier one with the same source node.
        var latest = new Dictionary<long, SourceNodeAlias>();

        for (var i = 0; i < after.Count; i++)
        {
            var segment = after[i];

            if (segment.Kind != kind || Untouched(segment))
            {
                result[i] = segment;

                continue;
            }

            latest.Clear();

            foreach (var alias in segment.SourceNodeAliases)
                latest[alias.SourceNodeId] = alias with
                {
                    // A new intentional contraction of this exact source ID
                    // supersedes its old alias everywhere, including surviving
                    // exterior sections. Otherwise stale targets make one source
                    // port appear to name two different canonical nodes.
                    CanonicalNodeId =
                        Resolve(remap.ContainsKey(alias.SourceNodeId) ? alias.SourceNodeId : alias.CanonicalNodeId)
                };

            if (byNode.TryGetValue(segment.StartNodeId, out var startAliases))
                foreach (var alias in startAliases)
                    latest[alias.SourceNodeId] = alias;

            if (byNode.TryGetValue(segment.EndNodeId, out var endAliases))
                foreach (var alias in endAliases)
                    latest[alias.SourceNodeId] = alias;

            var sourceNodeIds = new long[latest.Count];
            var merged = new SourceNodeAlias[latest.Count];
            var count = 0;

            foreach (var pair in latest)
            {
                sourceNodeIds[count] = pair.Key;
                merged[count] = pair.Value;
                count++;
            }

            // The source node identifiers are distinct, so the order is the same as a stable sort's.
            Array.Sort(sourceNodeIds, merged);
            result[i] = segment with { SourceNodeAliases = merged };
        }

        return result;
    }

    public static void Validate(
        SourceTopologyManifest? expected,
        IReadOnlyList<PreparedSegment> sections,
        ImportLog log,
        CancellationToken token)
    {
        if (expected == null)
        {
            log.Diagnostic(
                "SOURCE OBLIGATIONS unavailable; selected topology snapshot missing; quality=RequiresReview");

            return;
        }

        var aliases = new Dictionary<SourceNodeKey, HashSet<long>>();
        var nodes = new HashSet<SourceNodeKey>();
        var byWay = new Dictionary<(NetworkKind, long), List<PreparedSegment>>();

        foreach (var segment in sections)
        {
            nodes.Add(new(segment.Kind, segment.StartNodeId));
            nodes.Add(new(segment.Kind, segment.EndNodeId));

            foreach (var alias in segment.SourceNodeAliases)
            {
                var key = new SourceNodeKey(segment.Kind, alias.SourceNodeId);

                if (!aliases.TryGetValue(key, out var targets))
                    aliases[key] = targets = new();

                targets.Add(alias.CanonicalNodeId);
            }

            foreach (var way in segment.SourceWayIds.Count > 0 ? segment.SourceWayIds : new[] { segment.WayId })
            {
                var key = (segment.Kind, way);

                if (!byWay.TryGetValue(key, out var list))
                    byWay[key] = list = new();

                list.Add(segment);
            }
        }

        SourceNodeKey? Resolve(SourceNodeKey key)
        {
            if (aliases.TryGetValue(key, out var targets))
            {
                if (targets.Count != 1)
                    return null;

                var canonical = new SourceNodeKey(key.Kind, targets.First());

                return nodes.Contains(canonical) ? canonical : null;
            }

            return nodes.Contains(key) ? key : null;
        }

        var missingPorts = expected.Ports.Count(nodeKey => !Resolve(nodeKey).HasValue);
        var failed = 0;
        var records = new List<object>();

        foreach (var connection in expected.Connections)
        {
            token.ThrowIfCancellationRequested();
            var firstNodeKey = Resolve(connection.From);
            var secondNodeKey = Resolve(connection.To);
            var reachable = false;

            if (firstNodeKey.HasValue && secondNodeKey.HasValue)
            {
                var graph = new Dictionary<long, List<long>>();

                void Arc(long from, long to)
                {
                    if (!graph.TryGetValue(from, out var list))
                        graph[from] = list = new();

                    list.Add(to);
                }

                foreach (var segment in connection.Ways
                    .Where(wayId => byWay.ContainsKey((connection.From.Kind, wayId)))
                    .SelectMany(wayId => byWay[(connection.From.Kind, wayId)])
                    .Distinct())
                {
                    Arc(segment.StartNodeId, segment.EndNodeId);

                    if (!segment.OneWay)
                        Arc(segment.EndNodeId, segment.StartNodeId);
                }

                var seen = new HashSet<long> { firstNodeKey.Value.Id };
                var pending = new List<long> { firstNodeKey.Value.Id };

                for (var i = 0; i < pending.Count && !seen.Contains(secondNodeKey.Value.Id); i++)
                    if (graph.TryGetValue(pending[i], out var next))
                        foreach (var id in next)
                            if (seen.Add(id))
                                pending.Add(id);

                reachable = seen.Contains(secondNodeKey.Value.Id);
            }

            if (!reachable)
                failed++;

            if (log.DiagnosticsEnabled)
                records.Add(new
                {
                    connection.From,
                    connection.To,
                    connection.Ways,
                    canonicalFrom = firstNodeKey,
                    canonicalTo = secondNodeKey,
                    reachable,
                    identityMapping = "exact-source-aliases-only",
                    nativeLaneAcceptanceVerified = false
                });
        }

        log.Count("Missing mandatory selected-source ports", missingPorts);
        log.Count("Unresolved selected-source reachability obligations", failed);
        log.Count("Selected-source topology validations");
        var expectedChecks = expected.Ports.Count + expected.Connections.Count;
        log.Quality(
            new(
                "planned",
                "source-topology",
                "selected-source-obligations",
                expectedChecks,
                missingPorts + failed,
                0,
                0,
                0,
            "Exact source port identities and directional reachability; intentional aliases are checked obligations, not discarded failures")
            { Expected = expectedChecks });

        if (log.DiagnosticsEnabled)
            log.DiagnosticRecordChunks("source-topology-obligations.ndjson", records);

        log.Diagnostic(
            $"SOURCE OBLIGATIONS ports={expected.Ports.Count}; missingPorts={missingPorts}; directionalConnections={expected.Connections.Count}; unresolvedConnections={failed}; provenanceRestrictedSearch=true; sourceProximityMatching=false; quality=RequiresReview; placementBlocked=false");
    }
}
