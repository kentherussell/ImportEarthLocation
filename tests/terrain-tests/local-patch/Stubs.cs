// Stand-ins for the game-side types the road design sources refer to, as in tests/network-tests.
namespace ImportEarthLocation
{
    public static class Mod
    {
        public const string Id = "ImportEarthLocation";
    }
}

namespace ImportEarthLocation.Core
{
    // Keeps every line, count and finding, so two runs can be compared.
    public sealed class ImportLog
    {
        public bool StrictRecovery { get; init; }

        public bool CaptureRecords { get; init; }

        public bool ScalarDiagnostics { get; init; }

        public bool DiagnosticsEnabled => CaptureRecords || ScalarDiagnostics;

        public readonly List<string> Lines = new();

        public readonly List<NetworkQualityFinding> Findings = new();

        public readonly List<(string Name, double Value)> Counts = new();

        public void Info(string text) => Lines.Add(text);

        public void Warn(string text) => Lines.Add(text);

        public void Diagnostic(string text) => Lines.Add(text);

        public void Evidence(string text, bool warning = false) => Lines.Add(text);

        public void Quality(NetworkQualityFinding finding) => Findings.Add(finding);

        public void Count(string text, int count = 1) => Counts.Add((text, count));

        public double Metric(string text) => Counts.Where(entry => entry.Name == text).Sum(entry => entry.Value);

        public IEnumerable<KeyValuePair<string, double>> Metrics()
            => Counts.GroupBy(entry => entry.Name)
                .Select(group => new KeyValuePair<string, double>(group.Key, group.Sum(entry => entry.Value)));

        public void DiagnosticJson(string name, object value) { }

        public void DiagnosticSvg(string name, string svg) { }

        public void DiagnosticRecords(
            string text,
            IEnumerable<object> records,
            long maximumBytes = 8L * 1024 * 1024,
            bool critical = false)
        { }

        public void DiagnosticCompressedRecords(string text, IEnumerable<object> records) { }

        public void DiagnosticRecordChunks(string text, IEnumerable<object> records) { }

        public void Time(string text, double seconds) { }

        // Seconds by name, summed as the game's log sums them.
        public readonly Dictionary<string, double> Durations = new();

        public void AddDuration(string text, double seconds)
        {
            Durations.TryGetValue(text, out var old);
            Durations[text] = old + seconds;
        }
    }
}

namespace ImportEarthLocation.Processing
{
    public static class TerrainBuilder
    {
        public sealed record Result(
            float[] Heights,
            float HeightScaleMeters,
            float SeaLevelMeters,
            float MinimumElevation,
            float MaximumElevation);
    }
}

namespace ImportEarthLocation.Data
{
    public static class PublicHttpClient
    {
        public static HttpClient Instance = new();
    }

    public sealed class CacheStore
    {
        public Task<byte[]?> ReadValidAsync(
            string group,
            string key,
            string extension,
            Func<byte[], bool> valid,
            CancellationToken token) => Task.FromResult<byte[]?>(null);

        public Task WriteAsync(string group, string key, string extension, byte[] bytes, CancellationToken token)
            => Task.CompletedTask;
    }
}
