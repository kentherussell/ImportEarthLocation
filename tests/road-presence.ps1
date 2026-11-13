# Most sections never get a recorded path in the game (35,251 of 36,173 in Marseille on
# 6 Nov), so "unresolved" says little about the road. This checks each such section against
# the edges actually built: an end counts as on a road within about 8 m (two 4 m cells).
param(
    [Parameter(Mandatory)][string]$RunFolder,
    [switch]$Summary
)

Add-Type -TypeDefinition @'
using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Compression;
using System.Linq;
using System.Text.RegularExpressions;

public static class RoadPresence
{
    private const double Cell = 4;

    private static readonly Regex Identity =
        new Regex("\"WayId\":\"?(-?\\d+)\"?,\"StartNodeId\":\"?(-?\\d+)\"?,\"EndNodeId\":\"?(-?\\d+)\"?");

    private static Regex Point(string name) =>
        new Regex("\"" + name + "\":\\{\"x\":(-?[\\d.eE+-]+),\"y\":(-?[\\d.eE+-]+),\"z\":(-?[\\d.eE+-]+)\\}");

    private static double[] Read(Regex pattern, string line)
    {
        var match = pattern.Match(line);

        return match.Success
            ? new[] { double.Parse(match.Groups[1].Value), double.Parse(match.Groups[3].Value) }
            : null;
    }

    public static int[] Measure(string folder, List<string> samples)
    {
        Regex a = Point("a"), b = Point("b"), c = Point("c"), d = Point("d"), start = Point("start"), end = Point("end");
        var entityPattern = new Regex("\"entity\":\"([^\"]+)\"");
        var grid = new HashSet<(long, long)>();
        var entities = new HashSet<string>();
        var sections = new Dictionary<string, (double[] Start, double[] End)>();
        var recorded = new HashSet<string>();

        foreach (var file in Directory.GetFiles(folder, "native-unit-final-*.ndjson.gz"))
        {
            if (!Regex.IsMatch(Path.GetFileName(file), @"^native-unit-final-\d+\.ndjson\.gz$") || new FileInfo(file).Length == 0)
                continue;

            using var stream = File.OpenRead(file);
            using var unzip = new GZipStream(stream, CompressionMode.Decompress);
            using var reader = new StreamReader(unzip);
            string line;

            while ((line = reader.ReadLine()) != null)
            {
                if (line.Contains("\"outcome\":\"unresolved\""))
                {
                    var id = Identity.Match(line);
                    var from = Read(start, line);
                    var to = Read(end, line);

                    if (id.Success && from != null && to != null)
                        sections[id.Value] = (from, to);

                    continue;
                }

                if (line.Contains("\"outcome\":\"section-lineage\""))
                {
                    var id = Identity.Match(line);

                    if (id.Success)
                        recorded.Add(id.Value);
                }

                double[] p0 = Read(a, line), p1 = Read(b, line), p2 = Read(c, line), p3 = Read(d, line);

                if (p0 == null || p1 == null || p2 == null || p3 == null)
                    continue;

                var entity = entityPattern.Match(line).Groups[1].Value;

                if (entity.Length > 0 && !entities.Add(entity))
                    continue;

                var length = Math.Sqrt(Math.Pow(p0[0] - p3[0], 2) + Math.Pow(p0[1] - p3[1], 2)) + 1;
                var steps = Math.Max(4, (int)(length / 2));

                for (var i = 0; i <= steps; i++)
                {
                    double t = (double)i / steps, u = 1 - t;
                    var x = u * u * u * p0[0] + 3 * u * u * t * p1[0] + 3 * u * t * t * p2[0] + t * t * t * p3[0];
                    var z = u * u * u * p0[1] + 3 * u * u * t * p1[1] + 3 * u * t * t * p2[1] + t * t * t * p3[1];
                    grid.Add(((long)Math.Floor(x / Cell), (long)Math.Floor(z / Cell)));
                }
            }
        }

        bool OnRoad(double[] point)
        {
            long x = (long)Math.Floor(point[0] / Cell), z = (long)Math.Floor(point[1] / Cell);

            for (var dx = -1; dx <= 1; dx++)
                for (var dz = -1; dz <= 1; dz++)
                    if (grid.Contains((x + dx, z + dz)))
                        return true;

            return false;
        }

        int checkedCount = 0, both = 0, one = 0, neither = 0;

        foreach (var section in sections.Where(entry => !recorded.Contains(entry.Key)))
        {
            checkedCount++;
            var count = (OnRoad(section.Value.Start) ? 1 : 0) + (OnRoad(section.Value.End) ? 1 : 0);

            if (count == 2)
            {
                both++;
                continue;
            }

            if (count == 1)
                one++;
            else
                neither++;

            if (samples.Count < 40)
                samples.Add(section.Key + " at " + section.Value.Start[0] + "," + section.Value.Start[1]);
        }

        return new[] { checkedCount, both, one, neither };
    }
}
'@ -Language CSharp

$samples = [System.Collections.Generic.List[string]]::new()
$result = [RoadPresence]::Measure($RunFolder, $samples)

if ($result[0] -eq 0) {
    'no unit records to check'
    return
}

"$($result[1]) of $($result[0]) unrecorded sections are on built road; $($result[2]) have one end off it, $($result[3]) both ends"

if (-not $Summary) {
    $samples
}
