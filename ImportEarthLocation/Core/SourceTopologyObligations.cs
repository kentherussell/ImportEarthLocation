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
        var ports = Ports(sections, connections);

        return new(
            Array.AsReadOnly(ports.OrderBy(nodeKey => nodeKey.Kind).ThenBy(nodeKey => nodeKey.Id).ToArray()),
            Array.AsReadOnly(connections
            .OrderBy(connection => connection.From.Kind)
                .ThenBy(connection => connection.From.Id)
                .ThenBy(connection => connection.To.Id)
                .ThenBy(connection => connection.Ways.FirstOrDefault())
                .ToArray()));
    }

    // The ports of the sections, and their connections when a list is given for them.
    // Transfer needs only the ports, on every consolidation pass.
    private static HashSet<SourceNodeKey> Ports(
        IReadOnlyList<PreparedSegment> sections,
        List<SourceConnection>? connections)
    {
        var incident = new Dictionary<SourceNodeKey, List<int>>();

        for (var i = 0; i < sections.Count; i++)
            foreach (var key in new[] { SourceNodeKey.Start(sections[i]), SourceNodeKey.End(sections[i]) })
            {
                if (!incident.TryGetValue(key, out var list))
                    incident[key] = list = new();

                list.Add(i);
            }

        var ports = new HashSet<SourceNodeKey>();

        foreach (var pair in incident)
        {
            var first = sections[pair.Value[0]];

            if (pair.Value.Count != 2 || pair.Value.Any(i =>
                first.Kind == NetworkKind.Road && sections[i].Lanes != first.Lanes
                    || sections[i].OneWay != first.OneWay
                    || sections[i].Bridge != first.Bridge
                    || sections[i].Tunnel != first.Tunnel
                    || sections[i].Layer != first.Layer
                    || sections[i].StartTerminal && SourceNodeKey.Start(sections[i]) == pair.Key
                    || sections[i].EndTerminal && SourceNodeKey.End(sections[i]) == pair.Key
                    || sections[i].StartMiniRoundabout && SourceNodeKey.Start(sections[i]) == pair.Key
                    || sections[i].EndMiniRoundabout && SourceNodeKey.End(sections[i]) == pair.Key))
                ports.Add(pair.Key);
            else if (first.OneWay
                && (SourceNodeKey.Start(first) == pair.Key)
                    == (SourceNodeKey.Start(sections[pair.Value[1]]) == pair.Key))
                ports.Add(pair.Key);
        }

        var visited = new bool[sections.Count];

        void Trace(SourceNodeKey start, int seed)
        {
            var node = start;
            var index = seed;
            var forward = true;
            var backward = true;
            var ways = new HashSet<long>();

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

                if (connections != null)
                    ways.UnionWith(segment.SourceWayIds.Count > 0 ? segment.SourceWayIds : new[] { segment.WayId });

                node = along ? SourceNodeKey.End(segment) : SourceNodeKey.Start(segment);

                if (ports.Contains(node) || node == start)
                    break;

                var arms = incident[node];
                index = arms[0] == index ? arms[1] : arms[0];
            }

            if (connections == null)
                return;

            var provenance = Array.AsReadOnly(ways.OrderBy(wayId => wayId).ToArray());

            if (forward)
                connections.Add(new(start, node, provenance));

            if (backward)
                connections.Add(new(node, start, provenance));
        }

        foreach (var key in ports.OrderBy(nodeKey => nodeKey.Kind).ThenBy(nodeKey => nodeKey.Id))
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

    // Consume the consolidator's exact remapping, never recover aliases by
    // proximity or a repaired height. Previously declared aliases follow it.
    public static IReadOnlyList<PreparedSegment> Transfer(
        IReadOnlyList<PreparedSegment> before,
        IReadOnlyList<PreparedSegment> after,
        NetworkKind kind, IReadOnlyDictionary<long, long> remap)
    {
        if (remap.Count == 0)
            return after;

        var required =
            new HashSet<long>(
                Ports(before, null).Where(nodeKey => nodeKey.Kind == kind).Select(nodeKey => nodeKey.Id));

        long Resolve(long id)
        {
            var seen = new HashSet<long>();

            while (remap.TryGetValue(id, out var next) && next != id && seen.Add(id))
                id = next;

            return id;
        }

        var aliases = new Dictionary<long, SourceNodeAlias>();

        foreach (var segment in before.Where(otherSegment => otherSegment.Kind == kind))
            foreach (var alias in segment.SourceNodeAliases)
                aliases[alias.SourceNodeId] = alias with { CanonicalNodeId = Resolve(alias.CanonicalNodeId) };

        foreach (var pair in remap)
            aliases[pair.Key] =
                new(
                    pair.Key,
                    Resolve(pair.Value),
                    required.Contains(pair.Key) || aliases.TryGetValue(pair.Key, out var old) && old.Mandatory);

        var byNode =
            aliases.Values
                .GroupBy(alias => alias.CanonicalNodeId)
                .ToDictionary(group => group.Key, group => group.ToArray());

        // A section with no aliases of its own and none arriving at its nodes keeps an
        // empty list, so it is passed on as it is rather than copied.
        bool Untouched(PreparedSegment segment)
            => segment.SourceNodeAliases.Count == 0
                && !byNode.ContainsKey(segment.StartNodeId)
                && !byNode.ContainsKey(segment.EndNodeId);

        return after.Select(segment => segment.Kind != kind || Untouched(segment) ? segment : segment with
        {
            SourceNodeAliases = segment.SourceNodeAliases.Select(alias => alias with
            {
                // A new intentional contraction of this exact source ID
                // supersedes its old alias everywhere, including surviving
                // exterior sections. Otherwise stale targets make one source
                // port appear to name two different canonical nodes.
                CanonicalNodeId =
                    Resolve(remap.ContainsKey(alias.SourceNodeId) ? alias.SourceNodeId : alias.CanonicalNodeId)
            })
            .Concat(
                byNode.TryGetValue(segment.StartNodeId, out var firstAlias)
                    ? firstAlias
                    : Array.Empty<SourceNodeAlias>())
            .Concat(
                byNode.TryGetValue(segment.EndNodeId, out var secondAlias)
                    ? secondAlias
                    : Array.Empty<SourceNodeAlias>())
            .GroupBy(alias => alias.SourceNodeId)
                .Select(group => group.Last())
                .OrderBy(alias => alias.SourceNodeId)
                .ToArray()
        }).ToArray();
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
