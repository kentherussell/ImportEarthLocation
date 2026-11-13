using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// The game picks one composition per built edge, from the elevation we submit and
// the ground around it. This last step on the final plan makes the pieces say what
// they should draw as; it moves no deck.
//
// - A ground section standing clear of the terrain beside a bridge is walled by the
//   game (5 Nov DC: 2.4 km at 6-8 m, unstamped). It joins the bridge instead.
// - A ground section running deep under the terrain from a bore is walled as an open
//   cut. It joins the bore (the unmapped links between the Old Port tunnels).
// - A bore's section touching open road draws as one open cut, whatever its length
//   (E Street: 193 m with 15 m of ground over one end). It is cut where the ground
//   first covers it, so only that first piece draws open.
public static class NetworkDrawnStructures
{
    // The game draws a tunnel when both submitted ends are low enough and the real
    // depth at the edge's middle, at the road's two sides, is too: CourseSplitSystem
    // computes the middle from the terrain. On 5 Nov, with both ends forced, edges under
    // 10 m of ground there were nearly all open cuts and those over 12 m nearly all
    // tunnels (report 109). On 6 Nov the 10-14 m band split about 2 to 1 for tunnels, but
    // cutting at 13 m added about 2 km of open bore per map, so 11 m stays (report 110).
    // TunnelCover is the least an end is forced at, DrawnCover what a piece's middle is
    // cut to have, and DrawsAs the line used to predict. Ends count too: in the 10-14 m band on 6 Nov,
    // edges with both ends at 10 m or more drew as tunnel 49 of 63 times and those with
    // an end at 7.5-10 m only 21 of 66, which fits the game's -10 m rule for a tunnel's
    // highest point. PortalEndCover is what a cut needs at the piece's end.
    public const float TunnelCover = 7, DrawnCover = 11, PortalEndCover = 10;

    private const float DrawsAs = 10.5f,
        ElevatedClear = 4.5f,
        ElevatedLowest = 4,
        ElevatedReach = 100,
        FinishedReach = 300,
        MinimumPiece = 20,
        ExposedCover = 5;

    private const int Samples = 8, PerKind = 15;

    // Two tunnel mouths are a pair when their bores run within about 25 degrees of each
    // other, MouthBeside to MouthReach metres apart sideways, and one is MouthStagger to
    // MouthReach metres further in than the other.
    private const float MouthParallel = .9f, MouthBeside = 4, MouthReach = 60, MouthStagger = 5;

    // Before the roadbed, which treats these sections by what they now are.
    public static IReadOnlyList<PreparedSegment> Prepare(
        IReadOnlyList<PreparedSegment> source,
        float[] terrain,
        double width,
        ImportLog log)
    {
        // A short end can close on itself in design; the game builds nothing from that.
        var sections = source.Where(segment => segment.StartNodeId != segment.EndNodeId).ToList();
        log.Count("Ground sections beside a bridge drawn as elevated", JoinBridges(sections, terrain, width, log, ElevatedReach, "before earthworks"));

        log.Count("Ground sections under the terrain drawn as tunnel", JoinBores(sections, terrain, width));

        return sections;
    }

    // On the final terrain, whose fill over bores decides what ground covers them.
    public static IReadOnlyList<PreparedSegment> Finish(
        IReadOnlyList<PreparedSegment> source,
        float[] terrain,
        double width,
        ImportLog log)
    {
        var sections = source.ToList();
        log.Count("Sections clear of the finished ground drawn as viaduct", JoinViaducts(sections, terrain, width));

        // What the earthworks could not fill would stand on walls, so the reach is longer here.
        log.Count(
            "Ground sections still clear after earthworks drawn as elevated",
            JoinBridges(sections, terrain, width, log, FinishedReach, "after earthworks"));

        log.Count("Portal sections cut so only the mouth draws open", CutPortals(sections, terrain, width, log));

        return NativeEndpointPolicy.InteriorTunnelElevations(
            NativeEndpointPolicy.Canonicalize(sections, log),
            terrain,
            width,
            log);
    }

    private static float Ground(float3 point, float[] terrain, double width)
        => NetworkProcessor.Sample(point.x, point.z, terrain, width);

    private static float[] Cover(PreparedSegment segment, float[] terrain, double width)
    {
        var cover = new float[Samples + 1];

        for (var i = 0; i <= Samples; i++)
        {
            var point = NetworkNormalizer.Evaluate(segment, i / (float)Samples);
            cover[i] = Ground(point, terrain, width) - point.y;
        }

        return cover;
    }

    private static bool Candidate(PreparedSegment segment)
        => segment.Kind == NetworkKind.Road
            && !NetworkStructure.IsStructure(segment)
            && segment.StartNodeId != segment.EndNodeId;

    // Road standing clear of the finished ground along its whole length has no earth under
    // it, and the game walls it unless it is told it is a structure, bridge beside it or not.
    private static int JoinViaducts(List<PreparedSegment> sections, float[] terrain, double width)
    {
        var changed = 0;

        for (var i = 0; i < sections.Count; i++)
        {
            var segment = sections[i];

            if (!Candidate(segment) || NetworkVerticalProfiles.ArcLength(segment) < MinimumPiece)
                continue;

            var clear = Cover(segment, terrain, width);

            if (clear.Max() > -ElevatedLowest)
                continue;

            sections[i] =
                segment with
                {
                    Bridge = true,
                    Layer = Math.Max(segment.Layer, 1),
                    StartElevation = -clear[0],
                    EndElevation = -clear[Samples]
                };

            changed++;
        }

        return changed;
    }

    private static int JoinBridges(
        List<PreparedSegment> sections,
        float[] terrain,
        double width,
        ImportLog log,
        float maximumReach,
        string stage)
    {
        var degree = Degrees(sections);
        var bridge = new Dictionary<long, float>();
        var rejected = new Dictionary<int, (string Reason, float Length)>();
        var changed = 0;

        foreach (var segment in sections)
            if (segment.Kind == NetworkKind.Road && segment.Bridge && !segment.Tunnel)
            {
                bridge[segment.StartNodeId] = 0;
                bridge[segment.EndNodeId] = 0;
            }

        // Outward from each bridge, so a chain of pieces all standing clear joins it, up
        // to ElevatedReach: past that an approach is an embankment's job, not a viaduct's.
        for (var again = true; again;)
        {
            again = false;

            for (var i = 0; i < sections.Count; i++)
            {
                var segment = sections[i];

                if (!Candidate(segment)
                    || !bridge.ContainsKey(segment.StartNodeId) && !bridge.ContainsKey(segment.EndNodeId))
                    continue;

                // A junction on the far end stays at ground level with its side roads. A piece
                // between two bridges has a bridge at its far end too, and reads from the nearer.
                var fromStart =
                    bridge.ContainsKey(segment.StartNodeId)
                        && (!bridge.ContainsKey(segment.EndNodeId)
                            || bridge[segment.StartNodeId] <= bridge[segment.EndNodeId]);

                var far = fromStart ? segment.EndNodeId : segment.StartNodeId;
                var between = bridge.ContainsKey(far);
                var clear = Cover(segment, terrain, width);
                var near = fromStart ? clear[0] : clear[Samples];
                var length = NetworkVerticalProfiles.ArcLength(segment);
                var standsClear = clear.Count(c => c <= -WallHeight) * 2 > clear.Length;

                if (degree[far] > 2 && !between)
                {
                    if (standsClear)
                        rejected[i] = ("far end is a junction", length);

                    continue;
                }

                var reach = bridge[fromStart ? segment.StartNodeId : segment.EndNodeId] + length;

                if (reach > maximumReach)
                {
                    if (standsClear)
                        rejected[i] = ("past the reach", length);

                    continue;
                }

                if (clear.Max() > -ElevatedLowest || -near < ElevatedClear)
                {
                    if (standsClear)
                        rejected[i] = ("part of it is low", length);

                    continue;
                }

                sections[i] =
                    segment with
                    {
                        Bridge = true,
                        Layer = Math.Max(segment.Layer, 1),
                        StartElevation = -clear[0],
                        EndElevation = -clear[Samples]
                    };

                if (!between)
                    bridge[far] = reach;

                rejected.Remove(i);
                changed++;
                again = true;
            }
        }

        foreach (var reason in rejected.Values.GroupBy(entry => entry.Reason))
        {
            log.Count($"Bridge approach on the ground {stage}, {reason.Key}", reason.Count());
            log.Count($"Bridge approach metres {stage}, {reason.Key}", (int)reason.Sum(entry => entry.Length));
        }

        return changed;
    }

    private static int JoinBores(List<PreparedSegment> sections, float[] terrain, double width)
    {
        var bore = new HashSet<long>();
        var changed = 0;

        foreach (var segment in sections)
            if (segment.Kind == NetworkKind.Road && segment.Tunnel)
            {
                bore.Add(segment.StartNodeId);
                bore.Add(segment.EndNodeId);
            }

        var occupied =
            new HashSet<long>(sections.SelectMany(segment => new[] { segment.StartNodeId, segment.EndNodeId }));

        var next = long.MinValue + 14000000;

        long Node()
        {
            while (occupied.Contains(next))
                next++;

            occupied.Add(next);

            return next++;
        }

        for (var again = true; again;)
        {
            again = false;

            for (var i = 0; i < sections.Count; i++)
            {
                var segment = sections[i];

                if (!Candidate(segment) || !bore.Contains(segment.StartNodeId) && !bore.Contains(segment.EndNodeId))
                    continue;

                var profile = Profile(segment, terrain, width);
                var fromStart = bore.Contains(segment.StartNodeId);
                var steps = profile.Length - 1;
                // How far from the bore the ground keeps covering the section.
                var run = 0;

                while (run < steps && profile[fromStart ? run + 1 : steps - run - 1] >= TunnelCover)
                    run++;

                if (profile[fromStart ? 0 : steps] < TunnelCover)
                    continue;

                if (run == steps)
                {
                    sections[i] =
                        segment with
                        {
                            Tunnel = true,
                            Layer = Math.Min(segment.Layer, -1),
                            StartElevation = -profile[0],
                            EndElevation = -profile[steps]
                        };

                    bore.Add(segment.StartNodeId);
                    bore.Add(segment.EndNodeId);
                    changed++;
                    again = true;
                    continue;
                }

                // The covered part joins the bore and the rest stays open, so the open
                // length is only what the ground leaves open.
                var length = NetworkVerticalProfiles.ArcLength(segment);
                var along = run * length / steps;

                if (along < MinimumPiece || length - along < MinimumPiece)
                    continue;

                var t = fromStart ? along / length : 1 - along / length;
                var node = Node();
                var pair = NativeDesignUnits.Split(segment, t, node);
                var covered = fromStart ? pair.Left : pair.Right;
                var left = fromStart ? pair.Right : pair.Left;
                var at = (int)Math.Round(t * steps);
                covered =
                    covered with
                    {
                        Tunnel = true,
                        Layer = Math.Min(segment.Layer, -1),
                        StartElevation = fromStart ? -profile[0] : -profile[at],
                        EndElevation = fromStart ? -profile[at] : -profile[steps]
                    };

                sections[i] = covered;
                sections.Add(left);
                bore.Add(node);
                changed++;
                again = true;
            }
        }

        return changed;
    }

    private static int CutPortals(List<PreparedSegment> sections, float[] terrain, double width, ImportLog log)
    {
        var open = new HashSet<long>();

        foreach (var segment in sections)
            if (segment.Kind == NetworkKind.Road && !segment.Tunnel)
            {
                open.Add(segment.StartNodeId);
                open.Add(segment.EndNodeId);
            }

        var occupied =
            new HashSet<long>(sections.SelectMany(segment => new[] { segment.StartNodeId, segment.EndNodeId }));

        var next = long.MinValue + 13000000;

        long Node()
        {
            while (occupied.Contains(next))
                next++;

            occupied.Add(next);

            return next++;
        }

        var cuts = 0;
        var planned = new List<PortalCut>();

        for (var i = sections.Count - 1; i >= 0; i--)
        {
            var segment = sections[i];

            if (segment.Kind != NetworkKind.Road || !segment.Tunnel)
                continue;

            // An end draws open beside open road, and where it has too little ground to force.
            var cover = Profile(segment, terrain, width);
            var atStart = open.Contains(segment.StartNodeId) || cover[0] < TunnelCover;
            var atEnd = open.Contains(segment.EndNodeId) || cover[^1] < TunnelCover;

            if (!atStart && !atEnd)
                continue;

            var length = NetworkVerticalProfiles.ArcLength(segment);

            if (length < MinimumPiece * 2)
                continue;

            var steps = cover.Length - 1;

            float Middle(int from, int to) => SideCover(segment, (from + to) * .5f / steps, terrain, width);

            // The nearest cuts that leave the piece between them drawing as tunnel:
            // an end the game can be told is low, a middle it reads as deep.
            var first = 0;

            if (atStart)
            {
                first = -1;

                for (var k = 0; k <= steps && first < 0; k++)
                    if (cover[k] >= PortalEndCover && Middle(k, steps) >= DrawnCover)
                        first = k;
            }

            if (first < 0)
                continue;

            var last = steps;

            if (atEnd)
            {
                last = -1;

                for (var k = steps; k > first && last < 0; k--)
                    if (cover[k] >= PortalEndCover && Middle(first, k) >= DrawnCover)
                        last = k;
            }

            // Ground this close to the deck between the two would show the tube.
            if (last < first || cover.Skip(first).Take(last - first + 1).Min() < ExposedCover)
                continue;

            var a = Math.Max(first / (double)steps, MinimumPiece / length);
            var b = Math.Min(last / (double)steps, 1 - MinimumPiece / length);

            if (atStart && atEnd && (b - a) * length < MinimumPiece
                || atStart && (a <= 0 || a >= 1)
                || atEnd && (b <= 0 || b >= 1))
                continue;

            planned.Add(new(i, atStart ? a : null, atEnd ? b : null, cover, length));
        }

        if (DesignSwitches.On(DesignSwitches.SmoothRoads))
            AlignMouths(sections, planned, log);

        // From the last section back, so the indexes of those still to cut stay valid.
        foreach (var plan in planned)
        {
            var i = plan.Section;
            var segment = sections[i];
            var cut = new List<double>();

            if (plan.Start is { } a)
                cut.Add(a);

            if (plan.End is { } b)
                cut.Add(b);

            var pieces = new List<PreparedSegment>();
            var remaining = segment;
            var begin = 0d;

            foreach (var t in cut.OrderBy(t => t))
            {
                var pair = NativeDesignUnits.Split(remaining, (t - begin) / (1 - begin), Node());
                pieces.Add(pair.Left);
                remaining = pair.Right;
                begin = t;
                cuts++;
            }

            if (pieces.Count == 0)
                continue;

            pieces.Add(remaining);
            sections.RemoveAt(i);
            sections.InsertRange(i, pieces);
        }

        return cuts;
    }

    // Where one bore is cut: Start and End are the cut parameters on the section, null
    // where that end is not cut. Cover is the ground over the deck along it.
    private sealed record PortalCut(int Section, double? Start, double? End, float[] Cover, float Length)
    {
        public double? Start { get; set; } = Start;

        public double? End { get; set; } = End;
    }

    // Each bore is cut where its own ground first covers it, so the two bores of a
    // divided road opened at different points along the road. The shallower mouth of
    // two side by side moves in to stand beside the deeper one, where its own cover
    // still allows a portal there.
    private static void AlignMouths(List<PreparedSegment> sections, List<PortalCut> planned, ImportLog log)
    {
        var mouths = new List<(PortalCut Cut, bool Start, float2 Point, float2 Into)>();

        foreach (var cut in planned)
            foreach (var start in new[] { true, false })
            {
                if ((start ? cut.Start : cut.End) is not { } t)
                    continue;

                var segment = sections[cut.Section];
                var point = NetworkNormalizer.Evaluate(segment, (float)t).xz;

                var ahead =
                    NetworkNormalizer.Evaluate(segment, (float)Math.Min(1, t + .01)).xz
                        - NetworkNormalizer.Evaluate(segment, (float)Math.Max(0, t - .01)).xz;

                mouths.Add((cut, start, point, math.normalizesafe(ahead) * (start ? 1 : -1)));
            }

        int staggered = 0, moved = 0;

        foreach (var mouth in mouths)
        {
            var segment = sections[mouth.Cut.Section];
            var steps = mouth.Cut.Cover.Length - 1;
            var deepest = -1;

            foreach (var other in mouths)
            {
                var across = other.Point - mouth.Point;
                var along = math.dot(across, mouth.Into);
                var beside = Math.Abs(across.x * mouth.Into.y - across.y * mouth.Into.x);

                // The other mouth is further in, on a bore running the same way beside this one.
                if (other.Cut == mouth.Cut
                    || math.dot(other.Into, mouth.Into) < MouthParallel
                    || beside < MouthBeside
                    || beside > MouthReach
                    || along < MouthStagger
                    || along > MouthReach)
                    continue;

                staggered++;

                // The sample of this bore nearest the other mouth.
                var nearest = 0;
                var nearestDistance = float.MaxValue;

                for (var k = 0; k <= steps; k++)
                {
                    var distance =
                        math.distance(NetworkNormalizer.Evaluate(segment, k / (float)steps).xz, other.Point);

                    if (distance < nearestDistance)
                    {
                        nearestDistance = distance;
                        nearest = k;
                    }
                }

                if (mouth.Cut.Cover[nearest] < PortalEndCover)
                    continue;

                deepest = deepest < 0 ? nearest : mouth.Start ? Math.Max(deepest, nearest) : Math.Min(deepest, nearest);
            }

            if (deepest < 0)
                continue;

            var t = deepest / (double)steps;
            var margin = MinimumPiece / mouth.Cut.Length;

            if (mouth.Start)
            {
                var limit = (mouth.Cut.End ?? 1) - margin;

                if (t > mouth.Cut.Start && t <= limit)
                {
                    mouth.Cut.Start = t;
                    moved++;
                }
            }
            else
            {
                var limit = (mouth.Cut.Start ?? 0) + margin;

                if (t < mouth.Cut.End && t >= limit)
                {
                    mouth.Cut.End = t;
                    moved++;
                }
            }
        }

        log.Count("Tunnel mouths behind the one beside them", staggered);
        log.Count("Tunnel mouths moved in beside the one next to them", moved);
    }

    // Ground over the deck every 5 m or so.
    private static float[] Profile(PreparedSegment segment, float[] terrain, double width)
    {
        var steps = Math.Max(4, (int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(segment) / 5));
        var cover = new float[steps + 1];

        for (var k = 0; k <= steps; k++)
        {
            var point = NetworkNormalizer.Evaluate(segment, k / (float)steps);
            cover[k] = Ground(point, terrain, width) - point.y;
        }

        return cover;
    }

    // Ground over the deck at a point, the lower of the road's two sides.
    private static float SideCover(PreparedSegment segment, float t, float[] terrain, double width)
    {
        var point = NetworkNormalizer.Evaluate(segment, t);
        var d =
            math.normalizesafe(
                NetworkNormalizer.Evaluate(segment, Math.Min(1, t + .01f)).xz
                    - NetworkNormalizer.Evaluate(segment, Math.Max(0, t - .01f)).xz);

        var side = new float2(-d.y, d.x) * (segment.NativeWidth > 0 ? segment.NativeWidth * .5f : 6);

        return Math.Min(
            Ground(new float3(point.x + side.x, 0, point.z + side.y), terrain, width),
            Ground(new float3(point.x - side.x, 0, point.z - side.y), terrain, width))
            - point.y;
    }

    private static Dictionary<long, int> Degrees(IEnumerable<PreparedSegment> sections)
    {
        var degree = new Dictionary<long, int>();

        foreach (var segment in sections)
            if (segment.Kind == NetworkKind.Road)
            {
                degree[segment.StartNodeId] = degree.GetValueOrDefault(segment.StartNodeId) + 1;
                degree[segment.EndNodeId] = degree.GetValueOrDefault(segment.EndNodeId) + 1;
            }

        return degree;
    }

    // The plan's worst-looking places, for a look in game: long open bores, and the
    // longest ground sections held off the terrain or sunk into it. Span is the length
    // of the piece in metres, so the view can be sized to it.
    public static IEnumerable<(string Name, float3 Point, float Span)> Worst(
        IReadOnlyList<PreparedSegment> sections,
        float[] terrain,
        double width)
    {
        var rows = new List<(string Kind, float Score, float3 Point, float Span)>();

        foreach (var segment in sections.Where(
            otherSegment => otherSegment.Kind == NetworkKind.Road && !otherSegment.Bridge))
        {
            var length = NetworkVerticalProfiles.ArcLength(segment);
            var point = NetworkNormalizer.Evaluate(segment, .5f);
            var cover = Cover(segment, terrain, width);

            if (segment.Tunnel)
            {
                if ((segment.StartElevation ?? -8) > -10
                    || (segment.EndElevation ?? -8) > -10
                    || Math.Min(cover[0], cover[^1]) < PortalEndCover
                    || SideCover(segment, .5f, terrain, width) < DrawsAs)
                {
                    // A bore is under the ground at its middle; its mouth is what shows.
                    var mouth = NetworkNormalizer.Evaluate(segment, cover[0] <= cover[^1] ? 0 : 1);
                    rows.Add(("open bore", length, mouth, length));
                }

                continue;
            }

            if (cover.Count(c => c >= WallHeight) * 2 > cover.Length)
                rows.Add(("cut", length * cover.Max(), point, length));

            if (cover.Count(c => c <= -WallHeight) * 2 > cover.Length)
                rows.Add(("wall", length * -cover.Min(), point, length));
        }

        foreach (var group in rows.GroupBy(item => item.Kind).OrderBy(group => group.Key))
        {
            var chosen = new List<float3>();

            foreach (var row in group.OrderByDescending(item => item.Score))
            {
                // Two views of one spot show nothing more.
                if (chosen.Any(point => math.distance(point.xz, row.Point.xz) < 300))
                    continue;

                chosen.Add(row.Point);

                if (chosen.Count > PerKind)
                    break;

                yield return ($"worst {row.Kind} {chosen.Count}", row.Point, row.Span);
            }
        }
    }

    // Places the plan crowds or strains, for the same look: where many junctions sit
    // close together, dead ends, and the steepest sections.
    public static IEnumerable<(string Name, float3 Point)> Crowded(
        IReadOnlyList<PreparedSegment> sections,
        double width)
    {
        var roads = sections.Where(segment => segment.Kind == NetworkKind.Road).ToArray();
        var at = new Dictionary<long, (int Degree, float3 Point)>();

        foreach (var segment in roads)
            foreach (var (id, point) in new[] {
                (segment.StartNodeId, segment.CourseStart),
                (segment.EndNodeId, segment.CourseEnd)
                })
            {
                at.TryGetValue(id, out var item);
                at[id] = (item.Degree + 1, point);
            }

        const float Cell = 120;
        var half = (float)width / 2 - 300;
        var cells =
            at.Values
                .Where(item => item.Degree >= 3)
                .GroupBy(item => ((int)Math.Floor(item.Point.x / Cell), (int)Math.Floor(item.Point.z / Cell)))
                .OrderByDescending(group => group.Count());

        IEnumerable<(string Name, float3 Point)> Spaced(string name, IEnumerable<float3> points)
        {
            var chosen = new List<float3>();

            foreach (var point in points)
            {
                if (chosen.Any(otherPoint => math.distance(otherPoint.xz, point.xz) < 500))
                    continue;

                chosen.Add(point);

                if (chosen.Count > PerKind)
                    yield break;

                yield return ($"{name} {chosen.Count}", point);
            }
        }

        foreach (var site in Spaced("crowded junctions", cells.Select(group => group.First().Point)))
            yield return site;

        foreach (var site in Spaced(
            "dead end",
            at.Values
                .Where(item => item.Degree == 1 && Math.Abs(item.Point.x) < half && Math.Abs(item.Point.z) < half)
                .OrderBy(item => item.Point.x * 7919 + item.Point.z)
                .Select(item => item.Point)))
            yield return site;

        float Grade(PreparedSegment segment)
            => Math.Abs(segment.CourseEnd.y - segment.CourseStart.y)
                / Math.Max(1, math.distance(segment.CourseStart.xz, segment.CourseEnd.xz));

        foreach (var site in Spaced(
            "steep",
            roads
                .Where(segment => math.distance(segment.CourseStart.xz, segment.CourseEnd.xz) > 15)
                .OrderByDescending(Grade)
                .Select(segment => NetworkNormalizer.Evaluate(segment, .5f))))
            yield return site;
    }

    // Length the game should draw each way, read from the final plan and terrain, so a
    // replay or the import's own log can show it without a game run. Walls and cuts
    // are ground sections whose deck stays WallHeight or more off the ground over half
    // their length; open bores are tunnel sections with an end above the -10 m limit.
    public const float WallHeight = 4;

    public sealed record Drawn(
        double TunnelKm,
        double OpenBoreKm,
        double LongestOpenBore,
        double DeepOpenBoreKm,
        double CutKm,
        double WallKm,
        double ForcedShallowKm);

    public static Drawn Predict(IReadOnlyList<PreparedSegment> sections, float[] terrain, double width)
    {
        var open = new HashSet<long>();

        foreach (var segment in sections)
            if (segment.Kind == NetworkKind.Road && !segment.Tunnel)
            {
                open.Add(segment.StartNodeId);
                open.Add(segment.EndNodeId);
            }

        double tunnel = 0, bore = 0, longest = 0, deep = 0, cut = 0, wall = 0, shallow = 0;

        foreach (var segment in sections.Where(
            otherSegment => otherSegment.Kind == NetworkKind.Road && !otherSegment.Bridge))
        {
            var length = NetworkVerticalProfiles.ArcLength(segment);
            var cover = Cover(segment, terrain, width);

            if (segment.Tunnel)
            {
                var low = Math.Min(segment.StartElevation ?? -8, segment.EndElevation ?? -8);
                var high = Math.Max(segment.StartElevation ?? -8, segment.EndElevation ?? -8);
                var drawsOpen =
                    high > -10
                        || low > -15
                        || Math.Min(cover[0], cover[^1]) < PortalEndCover
                        || SideCover(segment, .5f, terrain, width) < DrawsAs;

                if (drawsOpen)
                {
                    bore += length;
                    longest = Math.Max(longest, length);

                    if (cover.Count(c => c >= TunnelCover) * 2 > cover.Length)
                        deep += length;
                }
                else
                {
                    tunnel += length;

                    if (cover.Min() < TunnelCover)
                        shallow += length;
                }

                continue;
            }

            if (cover.Count(c => c >= WallHeight) * 2 > cover.Length)
                cut += length;

            if (cover.Count(c => c <= -WallHeight) * 2 > cover.Length)
                wall += length;
        }

        return new(tunnel / 1000, bore / 1000, longest, deep / 1000, cut / 1000, wall / 1000, shallow / 1000);
    }
}
