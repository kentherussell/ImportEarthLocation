using System;
using System.Collections.Generic;
using System.Linq;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Lane alignment where one-way roads merge into or split from one road. The
// native node joins every arm at the node's centre, so two 2-lane roads joining
// a 3-lane road each met its middle, half a lane off (DC: over half of all such
// movements 2 m out). Each branch's end instead moves sideways, through its curve
// offset, onto the receiving road's lanes: the leftmost branch takes the
// leftmost lanes, the rightmost the rightmost, and any overlap (a lane drop) or
// gap (an added lane) falls between them. The node itself and the receiving
// road do not move. Offsets are in the right-hand normal of the receiving
// road's travel direction, the frame NativeLaneObligations measures in.
public static class NetworkLaneSlots
{
    // Branches within this angle of the receiving road join side by side.
    private const double ParallelCosine = .82; // 35°
    // Shortest section over which a width change's sideways step is bent out (metres).
    private static double Taper(double shift) => Math.Max(60, 30 * Math.Abs(shift));
    // Larger shifts would bend the branch end visibly; leave those as they are.
    // A ramp becoming the fifth lane of a motorway moves 8 m, so the bound is wider.
    private const float MaximumShift = 12;
    // Length of branch over which a sideways shift is bent in (metres).
    private const double BendLength = 80;

    public static IReadOnlyList<PreparedSegment> Align(IReadOnlyList<PreparedSegment> source, ImportLog log)
    {
        var result = source.ToArray();
        var ports = new Dictionary<long, List<(int Index, bool Start)>>();
        for (var i = 0; i < source.Count; i++)
        {
            var s = source[i]; if (s.Kind != NetworkKind.Road || !s.OneWay) continue;
            foreach (var start in new[] { true, false })
            { var id = start ? s.StartNodeId : s.EndNodeId; if (!ports.TryGetValue(id, out var list)) ports[id] = list = new(); list.Add((i, start)); }
        }
        var degree = source.SelectMany(s => new[] { s.StartNodeId, s.EndNodeId }).GroupBy(id => id).ToDictionary(g => g.Key, g => g.Count());
        float[] Lanes(PreparedSegment s) => s.NativeLaneOffsets.Count > 0 ? s.NativeLaneOffsets.OrderBy(x => x).ToArray() :
            Enumerable.Range(0, Math.Max(1, s.Lanes)).Select(k => (k - (Math.Max(1, s.Lanes) - 1) * .5f) * 4f).ToArray();
        // Travel direction at the port, pointing along the road's travel.
        float2 Travel(PreparedSegment s, bool start) => math.normalizesafe(start ? (s.CourseControl1 - s.CourseStart).xz : (s.CourseEnd - s.CourseControl2).xz);
        var aligned = 0; var shifted = 0; var skipped = 0; var widths = 0;
        void Bend((int Index, bool Start) port, float2 shift)
        {
            var offset = new float3(shift.x, 0, shift.y);
            // The sideways bend needs room: sections closer than BendLength to
            // the node move whole (both ends), back through plain two-arm
            // nodes, and the bend sits on the section where that length is
            // reached. A short end section bent alone made tight S-curves.
            var index = port.Index; var atNode = port.Start; var covered = 0d; var seen = new HashSet<int>();
            while (seen.Add(index))
            {
                var s = result[index];
                result[index] = atNode ? s with { StartCurveOffset = s.StartCurveOffset + offset } : s with { EndCurveOffset = s.EndCurveOffset + offset };
                shifted++;
                covered += NetworkDesignChecks.HorizontalArcLength(new(s.CourseStart, s.CourseControl1, s.CourseControl2, s.CourseEnd));
                if (covered >= BendLength) break;
                var farId = atNode ? s.EndNodeId : s.StartNodeId;
                if (!ports.TryGetValue(farId, out var far) || far.Count != 2) break;
                var next = far.First(p => p.Index != index);
                if (next.Start == !atNode || source[next.Index].Tunnel != s.Tunnel) break;
                // Move this section's far end too, then continue on the next one.
                var moved = result[index];
                result[index] = atNode ? moved with { EndCurveOffset = moved.EndCurveOffset + offset } : moved with { StartCurveOffset = moved.StartCurveOffset + offset };
                index = next.Index; atNode = next.Start;
            }
        }
        foreach (var node in ports.OrderBy(p => p.Key))
        {
            var list = node.Value;
            // A one-way road changing width at a plain node keeps its left edge
            // (right-hand traffic): a lane opens or closes on the right. The
            // native node centres both widths on one point, so the downstream
            // side moves sideways by half the width difference at the node and
            // returns to its mapped line within the first section at least Taper
            // long, as one S whose end tangents are unchanged (a 2 m step over
            // 60 m keeps a radius of about 300 m); shorter sections before it
            // move whole. Spreading the step over several short sections bent
            // each into an S of its own. Without such a section before a junction
            // or the map edge, whose ends cannot move, the node is left as it is.
            if (list.Count == 2 && list[0].Start != list[1].Start && degree[node.Key] == 2)
            {
                var down = list.First(p => p.Start); var up = list.First(p => !p.Start); var d = source[down.Index]; var u = source[up.Index];
                if (Math.Abs(d.NativeWidth - u.NativeWidth) <= .5f || d.Tunnel != u.Tunnel || d.Roundabout || u.Roundabout || d.ProtectedLink != u.ProtectedLink) continue;
                var travel = Travel(d, true);
                if (math.dot(travel, Travel(u, false)) < ParallelCosine) continue;
                var shift = (d.NativeWidth - u.NativeWidth) / 2;
                if (Math.Abs(shift) > MaximumShift) continue;
                // Sections shorter than the bend move whole; the first long enough
                // takes the step as one S, its start moved and its end not.
                var bend = Taper(shift); var chain = new List<int>(); var bent = -1; var index = down.Index; var seen = new HashSet<int>();
                while (seen.Add(index))
                {
                    var s = result[index];
                    if (NetworkDesignChecks.HorizontalArcLength(new(s.CourseStart, s.CourseControl1, s.CourseControl2, s.CourseEnd)) >= bend) { bent = index; break; }
                    chain.Add(index);
                    if (!ports.TryGetValue(s.EndNodeId, out var far) || far.Count != 2 || degree[s.EndNodeId] != 2) break;
                    var next = far.First(p => p.Index != index);
                    if (!next.Start || source[next.Index].Tunnel != s.Tunnel) break;
                    index = next.Index;
                }
                if (bent < 0) continue;
                var offset = new float3(travel.y, 0, -travel.x) * shift;
                foreach (var i in chain) { result[i] = result[i] with { StartCurveOffset = result[i].StartCurveOffset + offset, EndCurveOffset = result[i].EndCurveOffset + offset }; shifted++; }
                result[bent] = result[bent] with { StartCurveOffset = result[bent].StartCurveOffset + offset }; shifted++;
                widths++;
                continue;
            }
            if (list.Count < 3) continue;
            var inputs = list.Where(p => !p.Start).ToArray(); var outputs = list.Where(p => p.Start).ToArray();
            var merging = outputs.Length == 1 && inputs.Length >= 2; var splitting = inputs.Length == 1 && outputs.Length >= 2;
            if (!merging && !splitting) continue;
            var receiver = merging ? outputs[0] : inputs[0]; var target = source[receiver.Index];
            var heading = Travel(target, receiver.Start); var normal = new float2(heading.y, -heading.x);
            var branches = merging ? inputs : outputs;
            if (branches.Any(b => math.dot(Travel(source[b.Index], b.Start), heading) < ParallelCosine) || branches.Any(b => source[b.Index].Tunnel != target.Tunnel)) { skipped++; continue; }
            var anchor = receiver.Start ? target.CourseStart.xz : target.CourseEnd.xz;
            // Order by where each branch is 30 m from the node, left to right.
            float Across((int Index, bool Start) b)
            {
                var s = source[b.Index]; var length = Math.Max(1, NetworkDesignChecks.HorizontalArcLength(new(s.CourseStart, s.CourseControl1, s.CourseControl2, s.CourseEnd)));
                var t = (float)Math.Min(1, 30 / length); var p = NetworkNormalizer.Evaluate(s, b.Start ? t : 1 - t);
                return math.dot(p.xz - anchor, normal);
            }
            var ordered = branches.Select(b => (Port: b, Across: Across(b))).OrderBy(b => b.Across).ToArray();
            if (ordered.Zip(ordered.Skip(1), (a, b) => b.Across - a.Across).Any(d => d < .5f)) { skipped++; continue; }
            var slots = Lanes(target); var cursor = 0; var changes = new List<((int Index, bool Start) Port, float2 Shift)>();
            for (var k = 0; k < ordered.Length; k++)
            {
                var branch = source[ordered[k].Port.Index]; var own = Lanes(branch); var count = Math.Min(own.Length, slots.Length);
                // Left branches take slots from the left; the last one from the right.
                var first = k == ordered.Length - 1 ? slots.Length - count : Math.Min(cursor, slots.Length - count); cursor = first + count;
                var shift = slots.Skip(first).Take(count).Average() - own.Average();
                // Own lane offsets are in the branch's own frame; at a near-parallel
                // join that is the receiving frame to within cos 35°.
                if (Math.Abs(shift) > MaximumShift) { changes.Clear(); break; }
                changes.Add((ordered[k].Port, normal * shift));
            }
            if (changes.Count == 0) { skipped++; continue; }
            foreach (var (port, shift) in changes)
                if (math.lengthsq(shift) >= 1e-4f) Bend(port, shift);
            aligned++;
        }
        log.Count("Lane slots aligned at merges and splits", aligned);
        log.Count("Lane slots aligned where lanes are added or dropped", widths);
        log.Diagnostic($"LANE SLOTS mergesAndSplits={aligned}; laneCountChanges={widths}; branchEndsShifted={shifted}; skippedNotParallelOrAmbiguous={skipped}; maximumShiftMeters={MaximumShift}");
        return result;
    }
}
