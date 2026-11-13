// Roundabout rings: how far their nodes differ in height, and how steep that makes them.
public static class Rings
{
    public static void Run(string map)
    {
        var capture = new Cap(map);
        var data = capture.LoadOsm();
        bool Ring(Sec section) => section.Ways.Append(section.Way).Any(way => data.Ways.TryGetValue(way, out var found) && found.Tags.TryGetValue("junction", out var text) && text is "roundabout" or "circular");
        var plan = Compare.Final(capture).Where(section => section.Kind == "Road").ToList();
        var ring = plan.Where(Ring).ToList();
        var parent = new Dictionary<long, long>();
        long Find(long id) { if (!parent.TryGetValue(id, out var p)) return parent[id] = id; return p == id ? id : parent[id] = Find(p); }
        foreach (var section in ring) parent[Find(section.A)] = Find(section.B);
        var groups = ring.GroupBy(section => Find(section.A)).Select(group => (Sections: group.ToList(), Relief: group.SelectMany(s => new[] { s.P0.y, s.P3.y }).Max() - group.SelectMany(s => new[] { s.P0.y, s.P3.y }).Min(), Grade: group.Max(s => Math.Abs(s.P3.y - s.P0.y) / Math.Max(1, s.ArcLength())), Length: group.Sum(s => s.ArcLength()))).ToList();
        Console.WriteLine($"{map}: rings {groups.Count}; relief >1 m {groups.Count(g => g.Relief > 1)}, >2 m {groups.Count(g => g.Relief > 2)}, >4 m {groups.Count(g => g.Relief > 4)}; piece over 8% {groups.Count(g => g.Grade > .08)}, over 15% {groups.Count(g => g.Grade > .15)}");
        var steep = plan.Where(s => s.ArcLength() > 15 && Math.Abs(s.P3.y - s.P0.y) / s.ArcLength() > .1).ToList();
        var ringSections = ring.ToHashSet();
        var ringNodes = ring.SelectMany(s => new[] { s.A, s.B }).ToHashSet();
        Console.WriteLine($"  sections over 10% (15+ m): {steep.Count}; ring pieces {steep.Count(ringSections.Contains)}, touching a ring {steep.Count(s => !ringSections.Contains(s) && (ringNodes.Contains(s.A) || ringNodes.Contains(s.B)))}, bridge {steep.Count(s => s.Bridge)}, tunnel {steep.Count(s => s.Tunnel)}");
        foreach (var g in groups.OrderByDescending(g => g.Relief).Take(6)) { var at = capture.ToGeographic(g.Sections[0].P0.x, g.Sections[0].P0.z); Console.WriteLine($"  relief {g.Relief,5:F1} m grade {g.Grade:P0} length {g.Length,4:F0} m {at.lat:F4},{at.lon:F4}"); }
    }
}