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
        var twoWay = new Dictionary<long, List<int>>();
        for (var i = 0; i < source.Count; i++)
        {
            var s = source[i]; if (s.Kind != NetworkKind.Road || s.OneWay) continue;
            foreach (var id in new[] { s.StartNodeId, s.EndNodeId }) { if (!twoWay.TryGetValue(id, out var list)) twoWay[id] = list = new(); list.Add(i); }
        }
        float[] Lanes(PreparedSegment s) => s.NativeLaneOffsets.Count > 0 ? s.NativeLaneOffsets.OrderBy(x => x).ToArray() :
            Enumerable.Range(0, Math.Max(1, s.Lanes)).Select(k => (k - (Math.Max(1, s.Lanes) - 1) * .5f) * 4f).ToArray();
        // Travel direction at the port, pointing along the road's travel.
        float2 Travel(PreparedSegment s, bool start) => math.normalizesafe(start ? (s.CourseControl1 - s.CourseStart).xz : (s.CourseEnd - s.CourseControl2).xz);
        var aligned = 0; var shifted = 0; var skipped = 0; var widths = 0; var divided = 0; var undivided = 0; var wideDivisions = 0; var centred = new SortedDictionary<string, int>();
        int Centred(string reason) { centred[reason] = centred.TryGetValue(reason, out var n) ? n + 1 : 1; return -1; }
        // A two-way road dividing into one-way carriageways (an avenue or a
        // divided highway beginning). The native node joins them at its centre,
        // so each carriageway met the middle of the two-way road and the two
        // overlapped there. Each instead continues its own direction's half: its
        // end moves onto that half's lanes next to the median, on its travel-right
        // (right-hand traffic), across the two-way road's axis. Along the
        // carriageway its course blends from that lane position back to its own:
        // a point d along it, at lateral position y, moves to h·w + y·(1 - w),
        // w = 1 - 3x² + 2x³, x = d/Taper(h), where h is the lane position. The
        // blend never leaves the range between the two, so a course already near
        // the half (AvenueConsolidator's merged-trunk transitions, or the
        // compiler's fork clearance) is not pushed further out; w reaches 0
        // early at the first node that is not a plain continuation. Returns 1
        // when aligned, -1 when the arms continue the road but cannot be placed,
        // 0 when this is not a division.
        int Divide(List<(int Index, bool Start)> arms, PreparedSegment road, long id)
        {
            var atStart = road.StartNodeId == id; var anchor = atStart ? road.CourseStart : road.CourseEnd;
            // Pointing from the node into the two-way road.
            var into = math.normalizesafe(atStart ? (road.CourseControl1 - road.CourseStart).xz : (road.CourseControl2 - road.CourseEnd).xz);
            var forward = math.normalizesafe(atStart ? (road.CourseControl1 - road.CourseStart).xz : (road.CourseEnd - road.CourseControl2).xz);
            float2 Away((int Index, bool Start) p) { var s = source[p.Index]; return math.normalizesafe(p.Start ? (s.CourseControl1 - s.CourseStart).xz : (s.CourseControl2 - s.CourseEnd).xz); }
            if (!arms.Any(p => p.Start) || !arms.Any(p => !p.Start) || arms.Any(p => math.dot(Away(p), -into) < ParallelCosine || source[p.Index].Tunnel != road.Tunnel)) return 0;
            if (road.FinalDeckCurve != null || arms.Any(p => source[p.Index].FinalDeckCurve != null)) return Centred("deck");
            // Bridge decks and abutments keep their native alignment.
            if (road.Bridge || arms.Any(p => source[p.Index].Bridge)) return Centred("bridge");
            var lanes = Lanes(road); var changes = new List<(float3 Offset, double Shift, List<(int Index, bool AtNode, double From, double Length)> Chain, double Reach)>();
            foreach (var arm in arms)
            {
                var s = source[arm.Index];
                // Travel along the two-way road's axis, and its right-hand side.
                var travel = math.dot(Travel(s, arm.Start), forward) > 0 ? forward : -forward; var normal = new float2(travel.y, -travel.x);
                // Two-way lane offsets are in the road's own right-hand frame;
                // the lanes on this travel's right are the other half when it
                // runs against the road's direction.
                var half = (math.dot(travel, forward) > 0 ? lanes.Where(x => x > 0) : lanes.Where(x => x < 0).Select(x => -x)).OrderBy(x => x).ToArray();
                var own = Lanes(s); if (half.Length == 0) return Centred("lanes");
                var shift = half.Take(Math.Min(own.Length, half.Length)).Average() - own.Average();
                // A carriageway on the other side (left-hand traffic, or one
                // crossing over) is never pulled across; one leaving along the
                // axis is placed on its half.
                var length = Math.Max(1, NetworkDesignChecks.HorizontalArcLength(new(s.CourseStart, s.CourseControl1, s.CourseControl2, s.CourseEnd)));
                var probe = NetworkNormalizer.Evaluate(s, arm.Start ? (float)Math.Min(1, 30 / length) : 1 - (float)Math.Min(1, 30 / length));
                if (Math.Abs(shift) > MaximumShift) return Centred("shift");
                if (math.dot(probe.xz - anchor.xz, normal) * Math.Sign(shift) < -.5f) return Centred("side");
                // Plain continuations from the node: (index, entered at its start, distance at entry, length).
                var chain = new List<(int Index, bool AtNode, double From, double Length)>(); var index = arm.Index; var atNode = arm.Start; var covered = 0d; var seen = new HashSet<int>();
                var reach = Taper(shift);
                while (seen.Add(index))
                {
                    var section = result[index]; var span = NetworkDesignChecks.HorizontalArcLength(new(section.CourseStart, section.CourseControl1, section.CourseControl2, section.CourseEnd));
                    chain.Add((index, atNode, covered, span)); covered += span;
                    if (covered >= reach) break;
                    var farId = atNode ? section.EndNodeId : section.StartNodeId;
                    if (!ports.TryGetValue(farId, out var far) || far.Count != 2 || degree[farId] != 2) { reach = covered; break; }
                    var next = far.First(p => p.Index != index);
                    if (next.Start != atNode || source[next.Index].Tunnel != section.Tunnel) { reach = covered; break; }
                    index = next.Index; atNode = next.Start;
                }
                // Easing a shift h out over L curves the course at a radius of
                // about L²/6h; below 40 m a short split (a roundabout flare, or
                // one ending at a junction or structure) keeps meeting the centre.
                if (reach < Math.Max(20, Math.Sqrt(240 * Math.Abs(shift)))) return Centred("short");
                changes.Add((new float3(normal.x, 0, normal.y) * shift, shift, chain, reach));
            }
            foreach (var (offset, shift, chain, reach) in changes)
            {
                float Weight(double d) { var x = Math.Min(1, d / reach); return (float)(1 - 3 * x * x + 2 * x * x * x); }
                var across = offset / (float)shift;
                float3 Blend(float3 p, double d) => across * (float)(shift - math.dot((p - anchor).xz, across.xz)) * Weight(d);
                foreach (var (i, near, from, span) in chain)
                {
                    var s = result[i];
                    var a = Blend(near ? s.CourseStart : s.CourseEnd, from); var b = Blend(near ? s.CourseEnd : s.CourseStart, from + span);
                    result[i] = near ? s with { StartCurveOffset = s.StartCurveOffset + a, EndCurveOffset = s.EndCurveOffset + b }
                        : s with { EndCurveOffset = s.EndCurveOffset + a, StartCurveOffset = s.StartCurveOffset + b };
                    shifted++;
                }
            }
            return 1;
        }
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
            if (twoWay.TryGetValue(node.Key, out var receivers))
            {
                // Side streets may share the node; the receiver is the two-way
                // road that every one-way arm continues.
                var outcome = receivers.Select(r => Divide(list, source[r], node.Key)).FirstOrDefault(o => o != 0);
                if (outcome > 0) { divided++; continue; }
                if (outcome < 0) { skipped++; undivided++; continue; }
                if (list.Any(p => p.Start) && list.Any(p => !p.Start)) wideDivisions++;
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
        log.Count("Lane slots aligned where two-way roads divide", divided);
        log.Diagnostic($"LANE SLOTS mergesAndSplits={aligned}; laneCountChanges={widths}; twoWayDivisions={divided}; twoWayDivisionsLeftCentred={undivided}; oneWayPairsAtTwoWayNotParallel={wideDivisions}; leftCentredBecause={string.Join(",", centred.Select(p => p.Key + ":" + p.Value))}; branchEndsShifted={shifted}; skippedNotParallelOrAmbiguous={skipped}; maximumShiftMeters={MaximumShift}");
        return result;
    }
}
