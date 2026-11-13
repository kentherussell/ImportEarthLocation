using Unity.Mathematics;

// Roundabout rings in a plan: how big and how round each is, how many pieces it is
// built from, and whether it sits on one level. Reads a replay's plan when PLAN_DIR
// is set, else what the game received.
public static class Roundabouts
{
    private const double ShortPiece = 8, SmallRadius = 9;

    public static void Run(string map, int examples)
    {
        var capture = new Cap(map);
        var data = capture.LoadOsm();

        bool Ring(Sec section) =>
            section.Ways
                .Append(section.Way)
                .Any(
                    way => data.Ways.TryGetValue(way, out var found)
                        && found.Tags.TryGetValue("junction", out var text)
                        && text is "roundabout" or "circular");

        var plan = Triage.PlanOf(capture).Where(section => section.Kind == "Road").ToList();

        // A replay's plan flags its own ring sections; the tags are the fallback, and
        // they also catch an arm that was joined with a ring way.
        var flagged = plan.Any(section => section.Roundabout);
        var ring = plan.Where(section => flagged ? section.Roundabout : Ring(section)).ToList();
        var ringSections = ring.ToHashSet();
        var parent = new Dictionary<long, long>();

        long Find(long id)
        {
            if (!parent.TryGetValue(id, out var up))
                return parent[id] = id;

            return up == id ? id : parent[id] = Find(up);
        }

        foreach (var section in ring)
            parent[Find(section.A)] = Find(section.B);

        var armsAt = new Dictionary<long, int>();

        foreach (var section in plan.Where(section => !ringSections.Contains(section)))
        {
            armsAt[section.A] = armsAt.GetValueOrDefault(section.A) + 1;
            armsAt[section.B] = armsAt.GetValueOrDefault(section.B) + 1;
        }

        var rows = new List<(string Text, double Radius, int Pieces, int Arms, int Short, double Off, double Relief, bool Mixed, bool Closed)>();

        foreach (var group in ring.GroupBy(section => Find(section.A)))
        {
            var sections = group.ToList();
            var points = new List<double3>();

            foreach (var section in sections)
                for (var k = 0; k <= 8; k++)
                    points.Add(section.At(k / 8d));

            // Least-squares circle (Kasa) about the mean, so uneven piece lengths do not
            // pull the centre the way a plain average of the points would.
            var mean = new double2(points.Average(point => point.x), points.Average(point => point.z));
            double sxx = 0, sxz = 0, szz = 0, sxr = 0, szr = 0, sr = 0;

            foreach (var point in points)
            {
                var d = point.xz - mean;
                var r = math.lengthsq(d);
                sxx += d.x * d.x;
                sxz += d.x * d.y;
                szz += d.y * d.y;
                sxr += d.x * r;
                szr += d.y * r;
                sr += r;
            }

            var determinant = sxx * szz - sxz * sxz;
            var centre = mean;

            if (Math.Abs(determinant) > 1e-6)
                centre += new double2(sxr * szz - szr * sxz, szr * sxx - sxr * sxz) / determinant / 2;

            var distances = points.Select(point => math.distance(point.xz, centre)).ToList();
            var radius = distances.Average();
            var closed = sections
                .SelectMany(section => new[] { section.A, section.B })
                .GroupBy(node => node)
                .All(node => node.Count() == 2);
            var off = distances.Max(distance => Math.Abs(distance - radius));
            var nodes = sections.SelectMany(section => new[] { section.A, section.B }).Distinct().ToList();
            var arms = nodes.Count(node => armsAt.GetValueOrDefault(node) > 0);
            var relief = points.Max(point => point.y) - points.Min(point => point.y);

            var mixed =
                sections.Select(section => (section.Bridge, section.Tunnel, section.Layer)).Distinct().Count() > 1;

            var levels =
                string.Join(
                    ",",
                    sections
                        .GroupBy(
                            section => (section.Bridge ? "bridge" : section.Tunnel ? "tunnel" : "ground") + section.Layer)
                        .Select(level => $"{level.Key}x{level.Count()}"));

            var geographic = capture.ToGeographic(centre.x, centre.y);

            rows.Add(
                ($"radius {radius,5:F1} m off-circle {off,4:F1} m pieces {sections.Count,2} arms {arms,2} "
                    + $"under {ShortPiece} m {sections.Count(section => section.ArcLength() < ShortPiece),2} "
                    + $"relief {relief,4:F1} m{(mixed ? " MIXED LEVELS " + levels : "")}{(closed ? "" : " OPEN")} "
                    + $"{geographic.lat:F5},{geographic.lon:F5} ({centre.x:F0},{centre.y:F0})",
                    radius,
                    sections.Count,
                    arms,
                    sections.Count(section => section.ArcLength() < ShortPiece),
                    off,
                    relief,
                    mixed,
                    closed));
        }

        Console.WriteLine(
            $"{map}: rings {rows.Count}; radius under {SmallRadius} m {rows.Count(row => row.Radius < SmallRadius)}; "
                + $"more pieces than arms {rows.Count(row => row.Pieces > Math.Max(2, row.Arms))} "
                + $"(extra pieces {rows.Sum(row => Math.Max(0, row.Pieces - Math.Max(2, row.Arms)))}); "
                + $"with a piece under {ShortPiece} m {rows.Count(row => row.Short > 0)} (pieces {rows.Sum(row => row.Short)}); "
                + $"closed {rows.Count(row => row.Closed)}; "
                + $"closed off-circle over 0.5 m {rows.Count(row => row.Closed && row.Off > .5)}, "
                + $"over 2 m {rows.Count(row => row.Closed && row.Off > 2)}; "
                + $"open off-circle over 2 m {rows.Count(row => !row.Closed && row.Off > 2)}; "
                + $"relief over 1 m {rows.Count(row => row.Relief > 1)}; mixed levels {rows.Count(row => row.Mixed)}");

        foreach (var row in rows.Where(row => row.Mixed).Take(examples))
            Console.WriteLine("  mixed  " + row.Text);

        foreach (var row in rows.OrderBy(row => row.Radius).Take(examples))
            Console.WriteLine("  small  " + row.Text);

        foreach (var row in rows.OrderByDescending(row => row.Short).Take(examples))
            Console.WriteLine("  pieces " + row.Text);

        foreach (var row in rows.OrderByDescending(row => row.Off).Take(examples))
            Console.WriteLine("  shape  " + row.Text);
    }
}
