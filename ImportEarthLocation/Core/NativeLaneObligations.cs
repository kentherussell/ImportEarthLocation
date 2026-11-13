using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Unity.Mathematics;

namespace ImportEarthLocation.Core;

public readonly record struct SourceLanePort(int Section, bool Start, int Rank);

public readonly record struct LaneSlotObligation(
    SourceNodeKey Node,
    SourceLanePort From,
    SourceLanePort To,
    double PlannedLateralError)
{
    public long? SourceRelation { get; init; }
}

public sealed record LaneObligationPlan(IReadOnlyList<LaneSlotObligation> Movements, int UnavailableNodes);

// Compile from the immutable lane frames, never from whichever native movement
// happens to exist. General turn semantics remain explicitly unavailable unless
// the source specifies them; do not fabricate a complete-turns certificate.
public static class NativeLaneObligations
{
    private readonly record struct Slot(SourceLanePort Port, double Across);

    public static LaneObligationPlan Build(
        IReadOnlyList<PreparedSegment> sections,
        IReadOnlyList<OsmRelation>? relations = null)
    {
        var incident = new Dictionary<SourceNodeKey, List<(int Section, bool Start)>>();

        for (var i = 0; i < sections.Count; i++)
            foreach (var start in new[] { true, false })
            {
                var segment = sections[i];
                var key = new SourceNodeKey(segment.Kind, start ? segment.StartNodeId : segment.EndNodeId);

                if (!incident.TryGetValue(key, out var list))
                    incident[key] = list = new();

                list.Add((i, start));
            }

        var result = new List<LaneSlotObligation>();
        var unavailable = 0;

        foreach (var node in incident.OrderBy(entry => entry.Key.Kind).ThenBy(entry => entry.Key.Id))
        {
            if (node.Value.Count < 2)
                continue;

            if (node.Key.Kind == NetworkKind.Rail)
            {
                // A retained two-track axis has one train lane in each travel
                // direction. Opposite sides of the canonical turnout frame
                // connect; same-side arms do not imply a reverse turn.
                foreach (var from in node.Value)
                    foreach (var to in node.Value)
                    {
                        if (from.Section == to.Section)
                            continue;

                        double2 Away((int Section, bool Start) item)
                        {
                            var segment = sections[item.Section];
                            return math.normalizesafe(
                                (double2)(item.Start
                                    ? segment.CourseControl1 - segment.CourseStart
                                    : segment.CourseControl2 - segment.CourseEnd).xz);
                        }

                        if (math.dot(Away(from), Away(to)) >= 0)
                            continue;

                        result.Add(new(node.Key, new(from.Section, from.Start, 0), new(to.Section, to.Start, 0), 0));
                    }

                continue;
            }

            if (node.Key.Kind != NetworkKind.Road
                || node.Value.Any(
                    item => !sections[item.Section].OneWay || sections[item.Section].NativeLaneOffsets.Count == 0))
            {
                unavailable++;
                continue;
            }

            var receiver =
                node.Value
                    .OrderByDescending(item => sections[item.Section].NativeLaneOffsets.Count)
                    .ThenBy(item => sections[item.Section].WayId)
                    .First();

            double2 Heading((int Section, bool Start) item)
            {
                var segment = sections[item.Section];
                return math.normalizesafe(
                    (double2)(item.Start
                        ? segment.CourseControl1 - segment.CourseStart
                        : segment.CourseEnd - segment.CourseControl2).xz);
            }

            var heading = Heading(receiver);
            var normal = new double2(heading.y, -heading.x);

            if (math.lengthsq(heading) < .5 || node.Value.Any(item => math.dot(Heading(item), heading) < .9))
            {
                unavailable++;
                continue;
            }

            var incoming = new List<Slot>();
            var outgoing = new List<Slot>();

            foreach (var port in node.Value)
            {
                var segment = sections[port.Section];
                var point = port.Start ? segment.CourseStart : segment.CourseEnd;
                var direction = Heading(port);
                var side = new double2(direction.y, -direction.x);
                var offsets = segment.NativeLaneOffsets.OrderBy(x => x).ToArray();

                for (var rank = 0; rank < offsets.Length; rank++)
                {
                    var lateral = math.dot((double2)point.xz + side * offsets[rank], normal);
                    (port.Start ? outgoing : incoming).Add(new(new(port.Section, port.Start, rank), lateral));
                }
            }

            incoming.Sort((firstSlot, secondSlot) => firstSlot.Across.CompareTo(secondSlot.Across));
            outgoing.Sort((firstSlot, secondSlot) => firstSlot.Across.CompareTo(secondSlot.Across));

            if (incoming.Count == 0 || outgoing.Count == 0)
            {
                unavailable++;
                continue;
            }

            var added = new HashSet<(SourceLanePort, SourceLanePort)>();

            void Add(Slot firstSlot, Slot secondSlot)
            {
                if (added.Add((firstSlot.Port, secondSlot.Port)))
                    result.Add(
                        new(node.Key, firstSlot.Port, secondSlot.Port, Math.Abs(firstSlot.Across - secondSlot.Across)));
            }

            if (incoming.Count == outgoing.Count)
            {
                for (var i = 0; i < incoming.Count; i++)
                    Add(incoming[i], outgoing[i]);
            }
            else
            {
                // A lane addition or drop can legitimately have a many-to-one
                // connection. Cover each physical input/output in lateral order.
                foreach (var firstSlot in incoming)
                    Add(
                        firstSlot,
                        outgoing
                            .OrderBy(secondSlot => Math.Abs(firstSlot.Across - secondSlot.Across))
                            .ThenBy(slot => slot.Port.Section)
                            .ThenBy(slot => slot.Port.Rank)
                            .First());

                foreach (var secondSlot in outgoing)
                    Add(
                        incoming
                            .OrderBy(firstSlot => Math.Abs(firstSlot.Across - secondSlot.Across))
                            .ThenBy(slot => slot.Port.Section)
                            .ThenBy(slot => slot.Port.Rank)
                            .First(),
                        secondSlot);
            }
        }

        // Explicit simple connectivity relations also describe non-collinear
        // turns. Never infer their lane numbers from whichever lanes CS2 made.
        // Conditional, excepted and via-way semantics remain unavailable.
        if (relations != null)
            foreach (var relation in relations.OrderBy(relation => relation.Id))
            {
                if (!relation.Tags.TryGetValue("type", out var type) || type != "connectivity")
                    continue;

                var from =
                    relation.OrderedMembers.Where(member => member.Type == "way" && member.Role == "from").ToArray();

                var to = relation.OrderedMembers.Where(member => member.Type == "way" && member.Role == "to").ToArray();
                var via =
                    relation.OrderedMembers.Where(member => member.Type == "node" && member.Role == "via").ToArray();

                if (from.Length != 1
                    || to.Length != 1
                    || via.Length != 1
                    || relation.OrderedMembers.Any(member => member.Role == "via" && member.Type == "way")
                    || relation.Tags.Keys.Any(otherText => otherText.Contains("conditional") || otherText == "except")
                        || !relation.Tags.TryGetValue("connectivity", out var text))
                {
                    unavailable++;
                    continue;
                }

                bool Represents(PreparedSegment segment, long way)
                    => segment.WayId == way || segment.SourceWayIds.Contains(way);

                bool AtVia(PreparedSegment segment, bool start)
                {
                    var id = start ? segment.StartNodeId : segment.EndNodeId;
                    return id == via[0].Id
                        || segment.SourceNodeAliases.Any(
                            alias => alias.SourceNodeId == via[0].Id && alias.CanonicalNodeId == id);
                }

                var inputs =
                    Enumerable
                        .Range(0, sections.Count)
                        .Where(
                            i => sections[i].OneWay && Represents(sections[i], from[0].Id) && AtVia(sections[i], false))
                        .ToArray();

                var outputs =
                    Enumerable
                        .Range(0, sections.Count)
                        .Where(i => sections[i].OneWay && Represents(sections[i], to[0].Id) && AtVia(sections[i], true))
                        .ToArray();

                if (inputs.Length != 1
                    || outputs.Length != 1
                    || sections[inputs[0]].EndNodeId != sections[outputs[0]].StartNodeId)
                {
                    unavailable++;
                    continue;
                }

                var pairs = new List<(int From, int To)>();
                var valid = true;

                foreach (var item in text.Split('|'))
                {
                    var sides = item.Split(':');

                    if (sides.Length != 2 || !int.TryParse(sides[0], out var sourceLane) || sourceLane < 1)
                    {
                        valid = false;
                        break;
                    }

                    foreach (var target in sides[1].Split(','))
                    {
                        if (!int.TryParse(target, out var targetLane)
                            || targetLane < 1
                            || sourceLane > sections[inputs[0]].NativeLaneOffsets.Count
                            || targetLane > sections[outputs[0]].NativeLaneOffsets.Count)
                        {
                            valid = false;
                            break;
                        }

                        pairs.Add((sourceLane - 1, targetLane - 1));
                    }

                    if (!valid)
                        break;
                }

                if (!valid || pairs.Count == 0)
                {
                    unavailable++;
                    continue;
                }

                foreach (var pair in pairs.Distinct())
                    result.Add(new(new(NetworkKind.Road, sections[inputs[0]].EndNodeId),
                                    new(inputs[0], false, pair.From), new(outputs[0], true, pair.To), double.NaN)
                    { SourceRelation = relation.Id });
            }

        return new(result.AsReadOnly(), unavailable);
    }
}
