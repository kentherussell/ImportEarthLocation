using Newtonsoft.Json.Linq;
using Unity.Mathematics;

// What the game changes between the sections we submit and the edges it builds: how
// sections are split, how far curves and nodes move, which structure each section gets,
// and how dense the resulting junctions are. Reads one capture; prints one JSON line.
static partial class VisualMetricsReplay
{
    private sealed record PlannedPiece(
        int Index,
        string Kind,
        string Class,
        bool Bridge,
        bool Tunnel,
        bool OneWay,
        string StartNode,
        string EndNode,
        float3[] Points,
        float Length);

    private sealed record LiveEdge(
        string StartNode,
        string EndNode,
        string Kind,
        string Composition,
        float3[] Points,
        float Length);

    private static float3[] SampleCubic(float3 a, float3 b, float3 c, float3 d, int count)
    {
        var points = new float3[count];

        for (var i = 0; i < count; i++)
        {
            var t = i / (float)(count - 1);
            var u = 1 - t;
            points[i] = u * u * u * a + 3 * u * u * t * b + 3 * u * t * t * c + t * t * t * d;
        }

        return points;
    }

    private static float PolylineLength(float3[] points)
    {
        var length = 0f;

        for (var i = 1; i < points.Length; i++)
            length += math.distance(points[i - 1].xz, points[i].xz);

        return length;
    }

    // Horizontal distance from a point to a polyline, with the polyline's height at that place.
    private static (float Distance, float Height) ToPolyline(float3 point, float3[] line)
    {
        var best = (Distance: float.MaxValue, Height: 0f);

        for (var i = 1; i < line.Length; i++)
        {
            var segment = line[i].xz - line[i - 1].xz;
            var lengthSquared = math.lengthsq(segment);
            var t = lengthSquared < 1e-6f ? 0 : math.saturate(math.dot(point.xz - line[i - 1].xz, segment) / lengthSquared);
            var closest = math.lerp(line[i - 1], line[i], t);
            var distance = math.distance(closest.xz, point.xz);

            if (distance < best.Distance)
                best = (distance, closest.y);
        }

        return best;
    }

    private static string Composition(JObject row)
    {
        var flags = $"{(string?)row["general"]},{(string?)row["left"]},{(string?)row["right"]}";

        if (flags.Contains("Tunnel"))
            return "tunnel";

        if (flags.Contains("Elevated"))
            return "elevated";

        if (flags.Contains("Lowered"))
            return "lowered";

        if (flags.Contains("Raised"))
            return "raised";

        return "ground";
    }

    private static double Quantile(IReadOnlyList<float> sorted, double q) =>
        sorted.Count == 0 ? double.NaN : sorted[(int)(q * (sorted.Count - 1))];

    public static void GameAdjustments(string folder, string name)
    {
        const float CellSize = 32;
        const float MatchTolerance = 6;

        var planned =
            CorrectiveCapturedReplay
                .ReadManifest<JObject>(folder, "final-prepared-network-chunks.json")
                .Select((row, index) =>
                {
                    var points = SampleCubic(
                        ReadPoint(row["a"]!),
                        ReadPoint(row["b"]!),
                        ReadPoint(row["c"]!),
                        ReadPoint(row["d"]!),
                        17);

                    return new PlannedPiece(
                        index,
                        (string?)row["kind"] ?? "",
                        (string?)row["Classification"] ?? "",
                        (bool?)row["Bridge"] ?? false,
                        (bool?)row["Tunnel"] ?? false,
                        (bool?)row["OneWay"] ?? false,
                        (string?)row["StartNodeId"] ?? "",
                        (string?)row["EndNodeId"] ?? "",
                        points,
                        PolylineLength(points));
                })
                .ToArray();

        var live =
            CapturedRunFiles
                .Names(folder, "native-owned-final-", ".ndjson.gz")
                .Where(text => System.Text.RegularExpressions.Regex.IsMatch(text, @"^native-owned-final-\d+\.ndjson\.gz$"))
                .SelectMany(text => CapturedRunFiles.Records(CapturedRunFiles.Resolve(folder, text)))
                .Where(row => row["a"] != null && row["d"] != null)
                .Select(row =>
                {
                    var points = SampleCubic(
                        ReadPoint(row["a"]!),
                        ReadPoint(row["b"]!),
                        ReadPoint(row["c"]!),
                        ReadPoint(row["d"]!),
                        9);

                    return new LiveEdge(
                        (string)row["startNode"]!,
                        (string)row["endNode"]!,
                        (string?)row["kind"] ?? "",
                        Composition(row),
                        points,
                        PolylineLength(points));
                })
                .GroupBy(edge => edge.StartNode + "|" + edge.EndNode)
                .Select(group => group.First())
                .ToArray();

        var cells = new Dictionary<(int, int), List<PlannedPiece>>();

        foreach (var piece in planned)
            foreach (var key in piece.Points
                .Select(point => ((int)Math.Floor(point.x / CellSize), (int)Math.Floor(point.z / CellSize)))
                .Distinct())
            {
                if (!cells.TryGetValue(key, out var list))
                    cells[key] = list = new List<PlannedPiece>();

                list.Add(piece);
            }

        // Each live edge goes to the planned section of the same kind it follows most
        // closely, judged by its worst sample; beyond the tolerance it is unmatched.
        var pieces = new Dictionary<int, List<(LiveEdge Edge, float Lateral, float Vertical)>>();
        var unmatched = new List<LiveEdge>();

        foreach (var edge in live)
        {
            var middle = edge.Points[edge.Points.Length / 2];
            var candidates = new HashSet<PlannedPiece>();
            var cx = (int)Math.Floor(middle.x / CellSize);
            var cz = (int)Math.Floor(middle.z / CellSize);

            for (var dz = -1; dz <= 1; dz++)
                for (var dx = -1; dx <= 1; dx++)
                    if (cells.TryGetValue((cx + dx, cz + dz), out var list))
                        candidates.UnionWith(list.Where(piece => piece.Kind == edge.Kind));

            var best = (Piece: (PlannedPiece?)null, Lateral: float.MaxValue, Vertical: 0f);

            foreach (var piece in candidates)
            {
                var lateral = 0f;
                var vertical = 0f;

                foreach (var point in edge.Points)
                {
                    var (distance, height) = ToPolyline(point, piece.Points);
                    lateral = Math.Max(lateral, distance);
                    vertical = Math.Max(vertical, Math.Abs(point.y - height));
                }

                if (lateral < best.Lateral)
                    best = (piece, lateral, vertical);
            }

            if (best.Piece == null || best.Lateral > MatchTolerance)
            {
                unmatched.Add(edge);
                continue;
            }

            if (!pieces.TryGetValue(best.Piece.Index, out var assigned))
                pieces[best.Piece.Index] = assigned = new();

            assigned.Add((edge, best.Lateral, best.Vertical));
        }

        var roads = planned.Where(piece => piece.Kind == "Road").ToArray();
        var liveRoads = live.Where(edge => edge.Kind == "Road").ToArray();

        // Splits: a planned section built as several edges. When the pieces carry
        // different compositions the split is the game's elevation-type split.
        var split = 0;
        var compositionSplit = 0;
        var missing = 0;
        var lateralErrors = new List<float>();
        var verticalErrors = new List<float>();
        var outcome = new Dictionary<string, int>();
        var splitByClass = new Dictionary<string, (int Sections, int Split)>();

        foreach (var piece in roads)
        {
            var key = piece.Bridge ? "bridge" : piece.Tunnel ? "tunnel" : "ground";
            var entry = splitByClass.GetValueOrDefault(piece.Class);

            if (!pieces.TryGetValue(piece.Index, out var assigned))
            {
                missing++;
                splitByClass[piece.Class] = (entry.Sections + 1, entry.Split);
                continue;
            }

            if (assigned.Count > 1)
            {
                split++;

                if (assigned.Select(item => item.Edge.Composition).Distinct().Count() > 1)
                    compositionSplit++;
            }

            splitByClass[piece.Class] = (entry.Sections + 1, entry.Split + (assigned.Count > 1 ? 1 : 0));

            foreach (var item in assigned)
            {
                lateralErrors.Add(item.Lateral);
                verticalErrors.Add(item.Vertical);
                var result = key + "->" + item.Edge.Composition;
                outcome[result] = outcome.GetValueOrDefault(result) + 1;
            }
        }

        lateralErrors.Sort();
        verticalErrors.Sort();

        // Live junctions: nodes where three or more road edges meet, and how many sit
        // close to another junction, which the game draws as back-to-back crossings.
        var ends =
            liveRoads
                .SelectMany(edge => new[] { (Node: edge.StartNode, P: edge.Points[0]), (Node: edge.EndNode, P: edge.Points[^1]) })
                .ToArray();

        var degree = ends.GroupBy(item => item.Node).ToDictionary(group => group.Key, group => group.Count());
        var position = ends.GroupBy(item => item.Node).ToDictionary(group => group.Key, group => group.First().P);
        var junctions = degree.Where(entry => entry.Value >= 3).Select(entry => (entry.Key, P: position[entry.Key])).ToArray();
        var junctionCells =
            junctions
                .GroupBy(item => ((int)Math.Floor(item.P.x / 16), (int)Math.Floor(item.P.z / 16)))
                .ToDictionary(group => group.Key, group => group.ToArray());

        int CloseJunctions(float radius) =>
            junctions.Count(
                item =>
                {
                    var cx = (int)Math.Floor(item.P.x / 16);
                    var cz = (int)Math.Floor(item.P.z / 16);

                    for (var dz = -1; dz <= 1; dz++)
                        for (var dx = -1; dx <= 1; dx++)
                            if (junctionCells.TryGetValue((cx + dx, cz + dz), out var near)
                                && near.Any(other => other.Key != item.Key && math.distance(other.P.xz, item.P.xz) < radius))
                                return true;

                    return false;
                });

        // The report 112 measures on the built network, computed as for a plan.
        var measures =
            PlanMeasures.Roads(
                liveRoads
                    .Select(
                        edge => new MeasureEdge(
                            edge.StartNode,
                            edge.EndNode,
                            edge.Points,
                            edge.Composition switch { "elevated" => "bridge", "tunnel" => "tunnel", _ => "ground" },
                            "",
                            false))
                    .ToArray());

        // Planned junction nodes against the nearest live node: how far the game moved them.
        var plannedEnds =
            roads
                .SelectMany(piece => new[] { (Node: piece.StartNode, P: piece.Points[0]), (Node: piece.EndNode, P: piece.Points[^1]) })
                .ToArray();

        var plannedJunctions =
            plannedEnds
                .GroupBy(item => item.Node)
                .Where(group => group.Count() >= 3)
                .Select(group => group.First().P)
                .ToArray();

        var liveNodeCells =
            position
                .Values
                .GroupBy(point => ((int)Math.Floor(point.x / 16), (int)Math.Floor(point.z / 16)))
                .ToDictionary(group => group.Key, group => group.ToArray());

        var nodeMoves = new List<float>();

        foreach (var point in plannedJunctions)
        {
            var best = float.MaxValue;
            var cx = (int)Math.Floor(point.x / 16);
            var cz = (int)Math.Floor(point.z / 16);

            for (var dz = -1; dz <= 1; dz++)
                for (var dx = -1; dx <= 1; dx++)
                    if (liveNodeCells.TryGetValue((cx + dx, cz + dz), out var near))
                        foreach (var other in near)
                            best = Math.Min(best, math.distance(other.xz, point.xz));

            nodeMoves.Add(best);
        }

        nodeMoves.Sort();

        // Planned through nodes (two section ends) with no live node within half a metre:
        // the game combined the two sections into one edge and dropped the node.
        bool LiveNodeNear(float3 point, float radius)
        {
            var cx = (int)Math.Floor(point.x / 16);
            var cz = (int)Math.Floor(point.z / 16);

            for (var dz = -1; dz <= 1; dz++)
                for (var dx = -1; dx <= 1; dx++)
                    if (liveNodeCells.TryGetValue((cx + dx, cz + dz), out var near)
                        && near.Any(other => math.distance(other.xz, point.xz) < radius))
                        return true;

            return false;
        }

        var throughNodes =
            plannedEnds
                .GroupBy(item => item.Node)
                .Where(group => group.Count() == 2)
                .Select(group => group.First().P)
                .ToArray();

        var droppedThroughNodes = throughNodes.Count(point => !LiveNodeNear(point, .5f));

        // Control: a dropped node is only a combine when a built edge still runs through it;
        // otherwise the road there wasn't built at all.
        var liveCells = new Dictionary<(int, int), List<LiveEdge>>();

        foreach (var edge in live)
            foreach (var key in edge.Points
                .Select(point => ((int)Math.Floor(point.x / CellSize), (int)Math.Floor(point.z / CellSize)))
                .Distinct())
            {
                if (!liveCells.TryGetValue(key, out var list))
                    liveCells[key] = list = new List<LiveEdge>();

                list.Add(edge);
            }

        bool LiveRoadThrough(float3 point)
        {
            var cx = (int)Math.Floor(point.x / CellSize);
            var cz = (int)Math.Floor(point.z / CellSize);

            for (var dz = -1; dz <= 1; dz++)
                for (var dx = -1; dx <= 1; dx++)
                    if (liveCells.TryGetValue((cx + dx, cz + dz), out var near)
                        && near.Any(edge => ToPolyline(point, edge.Points).Distance < 1))
                        return true;

            return false;
        }

        var droppedOnLiveRoad = throughNodes.Count(point => !LiveNodeNear(point, .5f) && LiveRoadThrough(point));

        // Unmatched live edges that still lie on planned road: combined sections.
        var plannedRoadCells =
            roads
                .SelectMany(piece => piece.Points.Select(point => (Key: ((int)Math.Floor(point.x / CellSize), (int)Math.Floor(point.z / CellSize)), Piece: piece)))
                .GroupBy(item => item.Key)
                .ToDictionary(group => group.Key, group => group.Select(item => item.Piece).Distinct().ToArray());

        bool OnPlannedRoad(float3 point)
        {
            var cx = (int)Math.Floor(point.x / CellSize);
            var cz = (int)Math.Floor(point.z / CellSize);

            for (var dz = -1; dz <= 1; dz++)
                for (var dx = -1; dx <= 1; dx++)
                    if (plannedRoadCells.TryGetValue((cx + dx, cz + dz), out var near)
                        && near.Any(piece => ToPolyline(point, piece.Points).Distance < 1))
                        return true;

            return false;
        }

        var unmatchedRoads = unmatched.Where(edge => edge.Kind == "Road").ToArray();
        var combined = unmatchedRoads.Where(edge => edge.Points.All(OnPlannedRoad)).ToArray();

        // Why sections split: by planned structure, with the planned length and the
        // longest built piece, to tell length limits from elevation-type splits.
        var splitDetail = new Dictionary<string, (int Sections, int Split, List<float> Planned, List<float> Pieces)>();

        foreach (var piece in roads)
        {
            var key = piece.Bridge ? "bridge" : piece.Tunnel ? "tunnel" : "ground";
            if (!splitDetail.TryGetValue(key, out var entry))
                entry = (0, 0, new List<float>(), new List<float>());

            entry.Sections++;

            if (pieces.TryGetValue(piece.Index, out var assigned) && assigned.Count > 1)
            {
                entry.Split++;
                entry.Planned.Add(piece.Length);
                entry.Pieces.Add(assigned.Max(item => item.Edge.Length));
            }

            splitDetail[key] = entry;
        }

        // Live degree at each planned junction node.
        var liveDegreeAtPlannedJunction = new Dictionary<string, int>();
        var nodeKeyCells =
            position
                .GroupBy(entry => ((int)Math.Floor(entry.Value.x / 16), (int)Math.Floor(entry.Value.z / 16)))
                .ToDictionary(group => group.Key, group => group.Select(entry => entry.Key).ToArray());

        foreach (var point in plannedJunctions)
        {
            var cx = (int)Math.Floor(point.x / 16);
            var cz = (int)Math.Floor(point.z / 16);
            var found = "none";

            for (var dz = -1; dz <= 1 && found == "none"; dz++)
                for (var dx = -1; dx <= 1 && found == "none"; dx++)
                    if (nodeKeyCells.TryGetValue((cx + dx, cz + dz), out var near))
                        foreach (var node in near)
                            if (math.distance(position[node].xz, point.xz) < .5f)
                            {
                                found = degree[node] >= 3 ? "3+" : degree[node].ToString();
                                break;
                            }

            liveDegreeAtPlannedJunction[found] = liveDegreeAtPlannedJunction.GetValueOrDefault(found) + 1;
        }

        // Where lateral and vertical changes happen, by the live composition.
        var lateralByComposition = new Dictionary<string, (int Edges, int Lateral, int Vertical)>();

        foreach (var assigned in pieces.Values)
            foreach (var item in assigned.Where(item => item.Edge.Kind == "Road"))
            {
                var entry = lateralByComposition.GetValueOrDefault(item.Edge.Composition);
                lateralByComposition[item.Edge.Composition] =
                    (entry.Edges + 1, entry.Lateral + (item.Lateral > 1 ? 1 : 0), entry.Vertical + (item.Vertical > 1 ? 1 : 0));
            }

        if (Environment.GetEnvironmentVariable("ADJUST_CONTROLS") == "1")
        {
            // Planned against live control points on reshaped single-piece roads: does
            // the game keep our end tangents and only change the control lengths?
            var plannedRows = CorrectiveCapturedReplay.ReadManifest<JObject>(folder, "final-prepared-network-chunks.json");
            var liveRows =
                CapturedRunFiles
                    .Names(folder, "native-owned-final-", ".ndjson.gz")
                    .Where(text => System.Text.RegularExpressions.Regex.IsMatch(text, @"^native-owned-final-\d+\.ndjson\.gz$"))
                    .SelectMany(text => CapturedRunFiles.Records(CapturedRunFiles.Resolve(folder, text)))
                    .Where(row => row["a"] != null)
                    .GroupBy(row => (string)row["startNode"]! + "|" + (string)row["endNode"]!)
                    .ToDictionary(group => group.Key, group => group.First());

            var byEnds = liveRows.Values.GroupBy(row => ((int)Math.Round((float)row["a"]!["x"]! * 10), (int)Math.Round((float)row["a"]!["z"]! * 10))).ToDictionary(group => group.Key, group => group.ToArray());
            var shown = 0;

            foreach (var (index, assigned) in pieces.Where(pair => pair.Value.Count == 1 && pair.Value[0].Lateral > 1 && planned[pair.Key].Kind == "Road"))
            {
                var row = plannedRows[index];
                var a = ReadPoint(row["a"]!);
                var b = ReadPoint(row["b"]!);
                var c = ReadPoint(row["c"]!);
                var d = ReadPoint(row["d"]!);
                var key = ((int)Math.Round(a.x * 10), (int)Math.Round(a.z * 10));
                var keyReverse = ((int)Math.Round(d.x * 10), (int)Math.Round(d.z * 10));
                var match = (byEnds.GetValueOrDefault(key) ?? byEnds.GetValueOrDefault(keyReverse) ?? Array.Empty<JObject>()).FirstOrDefault();

                if (match == null)
                    continue;

                var la = ReadPoint(match["a"]!);
                var lb = ReadPoint(match["b"]!);
                var lc = ReadPoint(match["c"]!);
                var ld = ReadPoint(match["d"]!);

                if (math.distance(la.xz, a.xz) > .01f)
                    (la, lb, lc, ld) = (ld, lc, lb, la);

                float Angle(float3 from, float3 to, float3 otherFrom, float3 otherTo) =>
                    math.degrees(math.acos(math.clamp(math.dot(math.normalizesafe((to - from).xz), math.normalizesafe((otherTo - otherFrom).xz)), -1, 1)));

                Console.WriteLine(
                    $"  chord {math.distance(a.xz, d.xz):F0} m; planned handles {math.distance(a.xz, b.xz):F1}/{math.distance(d.xz, c.xz):F1} live {math.distance(la.xz, lb.xz):F1}/{math.distance(ld.xz, lc.xz):F1}; tangent change {Angle(a, b, la, lb):F1}/{Angle(d, c, ld, lc):F1} deg; turn {Angle(a, b, c, d):F0} deg");

                if (++shown == 12)
                    break;
            }
        }

        if (Environment.GetEnvironmentVariable("ADJUST_EXAMPLES") == "1")
            foreach (var (index, assigned) in pieces.Where(pair => pair.Value.Any(item => item.Lateral > 1)).Take(12))
            {
                var piece = planned[index];
                var grade =
                    Enumerable.Range(1, piece.Points.Length - 1)
                        .Max(i => Math.Abs(piece.Points[i].y - piece.Points[i - 1].y) / Math.Max(.01f, math.distance(piece.Points[i].xz, piece.Points[i - 1].xz)));

                Console.WriteLine(
                    $"  {piece.Class} bridge={piece.Bridge} length {piece.Length:F0} m grade {grade:P0} live pieces {assigned.Count}: "
                        + string.Join("; ", assigned.Select(item => $"{item.Edge.Composition} {item.Edge.Length:F0} m lateral {item.Lateral:F1} vertical {item.Vertical:F1} ends moved {Math.Min(math.distance(item.Edge.Points[0].xz, piece.Points[0].xz), math.distance(item.Edge.Points[0].xz, piece.Points[^1].xz)):F2}/{Math.Min(math.distance(item.Edge.Points[^1].xz, piece.Points[0].xz), math.distance(item.Edge.Points[^1].xz, piece.Points[^1].xz)):F2} m")));
            }

        var plannedLengths = roads.Select(piece => piece.Length).OrderBy(value => value).ToArray();
        var liveLengths = liveRoads.Select(edge => edge.Length).OrderBy(value => value).ToArray();
        var plannedKm = roads.Sum(piece => piece.Length) / 1000;

        var report = new JObject
        {
            ["map"] = name,
            ["plannedRoadSections"] = roads.Length,
            ["liveRoadEdges"] = liveRoads.Length,
            ["plannedRoadKm"] = Math.Round(plannedKm, 1),
            ["liveRoadKm"] = Math.Round(liveRoads.Sum(edge => edge.Length) / 1000, 1),
            ["sectionsSplit"] = split,
            ["sectionsSplitAcrossCompositions"] = compositionSplit,
            ["sectionsWithoutLiveEdge"] = missing,
            ["liveRoadEdgesUnmatched"] = unmatchedRoads.Length,
            ["liveRoadEdgesCombined"] = combined.Length,
            ["combinedLengthP50"] = Math.Round(Quantile(combined.Select(edge => edge.Length).OrderBy(value => value).ToArray(), .5), 1),
            ["plannedThroughNodes"] = throughNodes.Length,
            ["droppedThroughNodes"] = droppedThroughNodes,
            ["droppedThroughNodesOnBuiltRoad"] = droppedOnLiveRoad,
            ["lateralP50"] = Math.Round(Quantile(lateralErrors, .5), 2),
            ["lateralP95"] = Math.Round(Quantile(lateralErrors, .95), 2),
            ["lateralOver1m"] = lateralErrors.Count(value => value > 1),
            ["verticalP95"] = Math.Round(Quantile(verticalErrors, .95), 2),
            ["verticalOver1m"] = verticalErrors.Count(value => value > 1),
            ["plannedJunctions"] = plannedJunctions.Length,
            ["junctionMoveP50"] = Math.Round(Quantile(nodeMoves, .5), 3),
            ["junctionMoveP95"] = Math.Round(Quantile(nodeMoves, .95), 2),
            ["junctionsMovedOver1m"] = nodeMoves.Count(value => value > 1),
            ["liveJunctions"] = junctions.Length,
            ["liveJunctionsWithin15m"] = CloseJunctions(15),
            ["measures"] = measures,
            ["liveJunctionsWithin30m"] = CloseJunctions(30),
            ["liveDeadEnds"] = degree.Count(entry => entry.Value == 1),
            ["plannedUnder10m"] = plannedLengths.Count(value => value < 10),
            ["liveUnder10m"] = liveLengths.Count(value => value < 10),
            ["plannedLengthP50"] = Math.Round(Quantile(plannedLengths, .5), 1),
            ["liveLengthP50"] = Math.Round(Quantile(liveLengths, .5), 1),
            ["splitDetail"] = JObject.FromObject(
                splitDetail.ToDictionary(
                    entry => entry.Key,
                    entry => $"{entry.Value.Split}/{entry.Value.Sections}; planned P50 {Quantile(entry.Value.Planned.OrderBy(value => value).ToArray(), .5):F0} m; longest piece P50 {Quantile(entry.Value.Pieces.OrderBy(value => value).ToArray(), .5):F0} m, max {(entry.Value.Pieces.Count == 0 ? 0 : entry.Value.Pieces.Max()):F0} m")),
            ["liveDegreeAtPlannedJunction"] = JObject.FromObject(liveDegreeAtPlannedJunction),
            ["changesByComposition"] = JObject.FromObject(
                lateralByComposition.ToDictionary(
                    entry => entry.Key,
                    entry => $"{entry.Value.Edges} edges, lateral>1m {entry.Value.Lateral}, vertical>1m {entry.Value.Vertical}")),
            ["structureOutcome"] = JObject.FromObject(outcome.OrderByDescending(entry => entry.Value).ToDictionary(entry => entry.Key, entry => entry.Value)),
            ["splitByClass"] = JObject.FromObject(
                splitByClass
                    .Where(entry => entry.Value.Sections >= 50)
                    .OrderByDescending(entry => entry.Value.Sections)
                    .ToDictionary(entry => entry.Key, entry => $"{entry.Value.Split}/{entry.Value.Sections}")),
        };

        // HILL_CHECK=1 (report 112, A2): does the game bend a ground section's curve where it
        // changed the terrain under it? Compares the game-side terrain change along planned
        // ground roads that moved over 1 m with those that didn't.
        if (Environment.GetEnvironmentVariable("HILL_CHECK") == "1")
            report["hillCheck"] = HillCheck(folder, roads, pieces);

        Console.WriteLine(report.ToString(Newtonsoft.Json.Formatting.None));
    }

    private static JObject HillCheck(
        string folder,
        PlannedPiece[] roads,
        Dictionary<int, List<(LiveEdge Edge, float Lateral, float Vertical)>> pieces)
    {
        var capture = Load(folder);
        var designed = ReadHeights(folder, "final-design-heightmap.u16", capture.Scale);
        var observed = ReadHeights(folder, "native-observed-heightmap.u16", capture.Scale);
        var size = (int)Math.Sqrt(designed.Length);
        // The game holds its grid at its own width (report 112, section 1).
        var gameWidth =
            (double?)JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder, "native-observed-terrain.json")))["width"]
            ?? capture.Width;

        float Bilinear(float[] grid, double mapWidth, float x, float z)
        {
            var u = (x / mapWidth + .5) * (size - 1);
            var v = (z / mapWidth + .5) * (size - 1);
            var x0 = Math.Clamp((int)Math.Floor(u), 0, size - 2);
            var z0 = Math.Clamp((int)Math.Floor(v), 0, size - 2);
            var fx = (float)Math.Clamp(u - x0, 0, 1);
            var fz = (float)Math.Clamp(v - z0, 0, 1);

            return math.lerp(
                math.lerp(grid[z0 * size + x0], grid[z0 * size + x0 + 1], fx),
                math.lerp(grid[(z0 + 1) * size + x0], grid[(z0 + 1) * size + x0 + 1], fx),
                fz);
        }

        var moved = new List<float>();
        var still = new List<float>();

        foreach (var piece in roads)
        {
            if (piece.Bridge || piece.Tunnel || !pieces.TryGetValue(piece.Index, out var assigned))
                continue;

            var change = piece.Points.Max(point =>
                Math.Abs(Bilinear(observed, gameWidth, point.x, point.z) - Bilinear(designed, capture.Width, point.x, point.z)));

            (assigned.Max(item => item.Lateral) > 1 ? moved : still).Add(change);
        }

        JObject Summary(List<float> values)
        {
            values.Sort();

            return new JObject
            {
                ["sections"] = values.Count,
                ["changeP50"] = Math.Round(Quantile(values, .5), 2),
                ["changeP90"] = Math.Round(Quantile(values, .9), 2),
                ["changedOver0.5mShare"] = Math.Round(values.Count(value => value > .5f) / (double)Math.Max(1, values.Count), 3),
            };
        }

        return new JObject { ["moved"] = Summary(moved), ["unmoved"] = Summary(still) };
    }
}
