using ImportEarthLocation.Core;
using Unity.Mathematics;

// Prototype: penalised least-squares alignment of OSM highway chains (no code in the mod).
// Minimise sum (x_i-p_i)^2 + lambda * sum (third difference of x / h^3)^2 h, ends pinned.
// Then measure lateral error against the OSM polyline and the radius of the result.
public static class Fit
{
    sealed record Chain(string Cls, List<double2> Points);

    public static void Run(string map)
    {
        var capture = new Cap(map);
        var data = capture.LoadOsm();
        var ways =
            data.Ways.Values
                .Where(
                    way => way.Tags.TryGetValue("highway", out var text)
                        && text is "motorway" or "trunk" or "motorway_link" or "trunk_link"
                        && way.NodeIds.All(data.Nodes.ContainsKey)
                        && way.NodeIds.Length >= 2)
                .ToList();

        double2 NodePoint(long id)
        {
            var point = capture.Local(data.Nodes[id].Latitude, data.Nodes[id].Longitude);
            return new(point.x, point.z);
        }

        string HighwayClass(OsmWay way) => way.Tags["highway"];

        bool Mainline(OsmWay way) => HighwayClass(way) is "motorway" or "trunk";

        // Chains: follow one-way direction through nodes where exactly one same-family way continues within 35 degrees.
        var outgoing = ways.GroupBy(way => way.NodeIds[0]).ToDictionary(group => group.Key, group => group.ToList());
        var incoming = ways.GroupBy(way => way.NodeIds[^1]).ToDictionary(group => group.Key, group => group.ToList());
        var used = new HashSet<long>();
        var chains = new List<Chain>();

        double2 EndDir(OsmWay way)
        {
            var length = way.NodeIds.Length;
            return math.normalizesafe(NodePoint(way.NodeIds[length - 1]) - NodePoint(way.NodeIds[length - 2]));
        }

        double2 StartDir(OsmWay way) => math.normalizesafe(NodePoint(way.NodeIds[1]) - NodePoint(way.NodeIds[0]));

        OsmWay? Next(OsmWay way)
        {
            if (!outgoing.TryGetValue(way.NodeIds[^1], out var list))
                return null;

            var d = EndDir(way);
            var cand =
                list
                    .Where(
                        otherWay => !used.Contains(otherWay.Id)
                            && Mainline(otherWay) == Mainline(way)
                            && math.dot(d, StartDir(otherWay)) > Math.Cos(math.radians(35)))
                    .OrderByDescending(otherWay => math.dot(d, StartDir(otherWay)))
                    .ToList();

            return cand.FirstOrDefault();
        }

        OsmWay? Prev(OsmWay way)
        {
            if (!incoming.TryGetValue(way.NodeIds[0], out var list))
                return null;

            var d = StartDir(way);
            var cand =
                list
                    .Where(
                        otherWay => !used.Contains(otherWay.Id)
                            && Mainline(otherWay) == Mainline(way)
                            && math.dot(d, EndDir(otherWay)) > Math.Cos(math.radians(35)))
                    .OrderByDescending(otherWay => math.dot(d, EndDir(otherWay)))
                    .ToList();

            return cand.FirstOrDefault();
        }

        foreach (var seed in ways.OrderByDescending(way => Mainline(way) ? 1 : 0).ThenBy(way => way.Id))
        {
            if (!used.Add(seed.Id))
                continue;

            var seq = new LinkedList<OsmWay>();
            seq.AddFirst(seed);

            for (var way = Next(seed); way != null; way = Next(way))
            {
                used.Add(way.Id);
                seq.AddLast(way);
            }

            for (var way = Prev(seed); way != null; way = Prev(way))
            {
                used.Add(way.Id);
                seq.AddFirst(way);
            }

            var points = new List<double2>();

            foreach (var way in seq)
                foreach (var id in points.Count == 0 ? way.NodeIds : way.NodeIds.Skip(1))
                    points.Add(NodePoint(id));

            chains.Add(new(HighwayClass(seed), points));
        }

        Console.WriteLine($"{map}: {chains.Count} chains from {ways.Count} ways");

        foreach (var (group, floor) in new[] {
            ("motorway", 250d),
            ("trunk", 150d),
            ("motorway_link", 50d),
            ("trunk_link", 50d)
            })
        {
            var mine = chains.Where(chain => chain.Cls == group).ToList();

            foreach (var wave in group.EndsWith("link") ? new[] { 40d, 60d, 90d } : new[] { 120d, 200d, 300d })
            {
                var lat = new List<double>();
                double km = 0, tightKm = 0, turn = 0;
                var sites = 0;

                foreach (var chain in mine)
                {
                    var (points, _) = Resample(chain.Points, 5);

                    if (points.Count < 8)
                        continue;

                    var fit = Smooth(points, 5, wave);
                    var segment = chain.Points;

                    for (var i = 0; i < fit.Length; i++)
                    {
                        var best = double.MaxValue;
                        var low = Math.Max(1, (int)(i * (double)segment.Count / fit.Length) - 40);

                        for (var j = 1; j < segment.Count; j++)
                        {
                            var d = Hw.DistSeg(fit[i], segment[j - 1], segment[j]);

                            if (d < best)
                                best = d;
                        }

                        lat.Add(best);
                    }

                    var inTight = false;

                    for (var i = 2; i + 2 < fit.Length; i++)
                    {
                        var vectorA = math.normalizesafe(fit[i] - fit[i - 2]);
                        var vectorB = math.normalizesafe(fit[i + 2] - fit[i]);
                        var ang =
                            Math.Abs(
                                Math.Atan2(vectorA.x * vectorB.y - vectorA.y * vectorB.x, math.dot(vectorA, vectorB)));

                        var ds = math.distance(fit[i - 2], fit[i + 2]) / 2;
                        var curv = ang / Math.Max(.1, ds);
                        km += 5 / 1000d;
                        turn += ang / 2;
                        var tight = curv * floor > 1;

                        if (tight)
                        {
                            tightKm += 5 / 1000d;

                            if (!inTight)
                                sites++;
                        }

                        inTight = tight;
                    }
                }

                lat.Sort();

                if (lat.Count > 0)
                    Console.WriteLine(
                        $"  {group,-14} wavelength {wave,4:F0} m: {km,7:F0} km; lateral p50 {lat[lat.Count / 2],5:F2} p95 {lat[(int)(.95 * lat.Count)],5:F2} p99 {lat[(int)(.99 * lat.Count)],5:F2} max {lat[^1],6:F1}; under floor {floor:F0} m: {sites,5} stretches, {tightKm,6:F1} km ({100 * tightKm / km:F1}%)");
            }
        }
    }

    static (List<double2>, double) Resample(List<double2> points, double h)
    {
        var result = new List<double2> { points[0] };
        var left = h;

        for (var i = 1; i < points.Count; i++)
        {
            var vectorA = points[i - 1];
            var vectorB = points[i];
            var otherDistance = math.distance(vectorA, vectorB);
            var t = 0d;

            while (otherDistance - t >= left)
            {
                t += left;
                result.Add(math.lerp(vectorA, vectorB, t / otherDistance));
                left = h;
            }

            left -= otherDistance - t;
        }

        if (math.distance(result[^1], points[^1]) > .5)
            result.Add(points[^1]);

        return (result, h);
    }

    // Banded Cholesky on (I + mu*D3'D3), with the two ends pinned by heavy weights. Cutoff wavelength L: mu=(L/(2pi h))^6.
    public static double2[] Smooth(List<double2> points, double h, double wavelength)
    {
        var count = points.Count;
        var mu = Math.Pow(wavelength / (2 * Math.PI * h), 6);
        const int b = 3;
        var A = new double[count, b + 1];
        var w = new double[count];

        for (var i = 0; i < count; i++)
            w[i] = i == 0 || i == count - 1 ? 1e8 : 1;

        var d = new[] { -1d, 3, -3, 1 };

        for (var i = 0; i < count; i++)
            A[i, 0] += w[i];

        for (var k = 0; k + 3 < count; k++)
            for (var r = 0; r < 4; r++)
                for (var s = r; s < 4; s++)
                    A[k + r, s - r] += mu * d[r] * d[s];

        // Cholesky
        for (var i = 0; i < count; i++)
        {
            for (var j = 0; j <= b && i + j < count; j++)
            {
                var sum = A[i, j];

                for (var k = 1; k <= b - j && i - k >= 0; k++)
                    sum -= A[i - k, k] * A[i - k, k + j];

                if (j == 0)
                    A[i, 0] = Math.Sqrt(Math.Max(sum, 1e-12));
                else
                    A[i, j] = sum / A[i, 0];
            }
        }

        var otherPoints = new double2[count];

        for (var i = 0; i < count; i++)
            otherPoints[i] = points[i] * w[i];

        for (var i = 0; i < count; i++)
        {
            for (var k = 1; k <= b && i - k >= 0; k++)
                otherPoints[i] -= A[i - k, k] * otherPoints[i - k];

            otherPoints[i] /= A[i, 0];
        }

        for (var i = count - 1; i >= 0; i--)
        {
            for (var k = 1; k <= b && i + k < count; k++)
                otherPoints[i] -= A[i, k] * otherPoints[i + k];

            otherPoints[i] /= A[i, 0];
        }

        return otherPoints;
    }
}
