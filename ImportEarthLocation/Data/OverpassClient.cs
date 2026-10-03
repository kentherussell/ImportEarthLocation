using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Net.Http;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;
using System.Threading.Tasks;
using System.Xml.Linq;
using ImportEarthLocation.Core;
using ImportEarthLocation.Geo;

namespace ImportEarthLocation.Data;

public sealed class OverpassClient
{
    private static readonly string[] Endpoints =
    {
        "https://overpass-api.de/api/interpreter",
        "https://overpass.private.coffee/api/interpreter",
        "https://maps.mail.ru/osm/tools/overpass/api/interpreter"
    };

    private readonly CacheStore _cache;
    private readonly ImportLog _log;
    private readonly HttpClient _http;
    private readonly Func<TimeSpan, CancellationToken, Task> _delay;
    private readonly Func<DateTimeOffset> _now;
    private readonly DateTimeOffset[] _availableAt = new DateTimeOffset[Endpoints.Length];
    private readonly object _scheduler = new();
    private int _active;
    private int _concurrency = 3;
    private readonly bool _geography;
    private readonly RoadImportLevel? _roadLevel;
    private readonly bool _importRail = true;

    private async Task AcquireAsync(CancellationToken token)
    {
        while (true)
        {
            token.ThrowIfCancellationRequested();
            lock (_scheduler) { if (_active < _concurrency) { _active++; return; } }
            await Task.Delay(50, token).ConfigureAwait(false);
        }
    }

    private void ReduceConcurrency()
    {
        lock (_scheduler)
        {
            if (_concurrency <= 1) return;
            _concurrency--;
            _log.Warn($"Overpass rate limiting: reducing concurrency to {_concurrency}. Existing requests will finish; new requests obey the reduced limit.");
        }
    }

    public OverpassClient(CacheStore cache, ImportLog log, bool geography = false, RoadImportLevel? roadLevel = null, bool importRail = true)
        : this(cache, log, PublicHttpClient.Instance, Task.Delay, () => DateTimeOffset.UtcNow) { _geography = geography; _roadLevel = roadLevel; _importRail = importRail; }

    internal OverpassClient(CacheStore cache, ImportLog log, HttpClient http,
        Func<TimeSpan, CancellationToken, Task> delay, Func<DateTimeOffset> now)
    {
        _cache = cache;
        _log = log;
        _http = http;
        _delay = delay;
        _now = now;
    }

    public async Task<IReadOnlyList<byte[]>> Download16Async(GeoBounds bounds, bool includeSecondary, CancellationToken token)
    {
        var tiles = GeoMath.Split4x4(bounds);
        var results = new byte[tiles.Count][];
        var next = -1;
        Exception? failure = null;
        var elapsed = Stopwatch.StartNew();
        using var cancellation = CancellationTokenSource.CreateLinkedTokenSource(token);
        async Task Worker()
        {
            try
            {
                while (true)
                {
                    cancellation.Token.ThrowIfCancellationRequested();
                    var index = Interlocked.Increment(ref next);
                    if (index >= tiles.Count) return;
                    _log.Info($"Downloading OSM tile {index + 1}/16.");
                    results[index] = await DownloadOneAsync(tiles[index], includeSecondary, index, cancellation.Token).ConfigureAwait(false);
                }
            }
            catch (Exception exception)
            {
                if (!(exception is OperationCanceledException) || !cancellation.IsCancellationRequested)
                    Interlocked.CompareExchange(ref failure, exception, null);
                cancellation.Cancel();
            }
        }
        await Task.WhenAll(Enumerable.Range(0, 3).Select(_ => Task.Run(Worker))).ConfigureAwait(false);
        _log.Time("OSM download", elapsed.Elapsed.TotalSeconds);
        if (failure != null) throw failure;
        token.ThrowIfCancellationRequested();
        if (results.Any(bytes => bytes == null)) throw new InvalidDataException("Incomplete OSM tile coverage; preparation aborted.");
        return results;
    }

    private async Task<byte[]> DownloadOneAsync(GeoBounds bounds, bool includeSecondary, int index, CancellationToken token)
    {
        var query = _geography ? BuildGeographyQuery(bounds) : _roadLevel.HasValue ? BuildNetworkQuery(bounds, _roadLevel.Value, _importRail) : BuildQuery(bounds, includeSecondary);
        var key = $"overpass-v3:{index}:{includeSecondary}:{query}";
        var cached = await _cache.ReadValidAsync("overpass", key, "osm", IsValidOsm, token).ConfigureAwait(false);
        if (cached != null)
        {
            _log.Info($"OSM tile {index + 1}/16 loaded from validated cache ({cached.Length:N0} bytes).");
            _log.Count("OSM cache hits");
            return cached;
        }

        Exception? last = null;
        // Six attempts at EACH independent provider, with short pauses on switching
        // and a separate cooldown before returning to a failed provider.
        for (var round = 0; round < 6; round++)
        for (var provider = 0; provider < Endpoints.Length; provider++)
        {
            token.ThrowIfCancellationRequested();
            var providerIndex = (index + provider) % Endpoints.Length;
            var endpoint = Endpoints[providerIndex];
            TimeSpan wait;
            lock (_scheduler) wait = _availableAt[providerIndex] - _now();
            if (wait > TimeSpan.Zero)
            {
                _log.Info($"OSM tile {index + 1}/16: waiting {wait.TotalSeconds:0}s before retrying {endpoint}.");
                await _delay(wait, token).ConfigureAwait(false);
            }
            var elapsed = Stopwatch.StartNew();
            var cooldown = TimeSpan.FromSeconds(Math.Min(120, 30 * Math.Pow(2, round)));
            _log.Info($"OSM tile {index + 1}/16: {endpoint}, provider attempt {round + 1}/6, bounds {F(bounds.South)},{F(bounds.West)},{F(bounds.North)},{F(bounds.East)}.");
            try
            {
                using var content = new FormUrlEncodedContent(new Dictionary<string, string> { ["data"] = query });
                await AcquireAsync(token).ConfigureAwait(false);
                elapsed.Restart();
                HttpResponseMessage received;
                try { received = await _http.PostAsync(endpoint, content, token).ConfigureAwait(false); }
                finally { lock (_scheduler) _active--; }
                using var response = received;
                var bytes = await response.Content.ReadAsByteArrayAsync().ConfigureAwait(false);
                token.ThrowIfCancellationRequested();
                var retryAfter = response.Headers.RetryAfter;
                var advisedWait = retryAfter?.Delta ?? (retryAfter?.Date - _now());
                if (advisedWait.HasValue && advisedWait.Value > TimeSpan.FromMinutes(5))
                    throw new InvalidOperationException($"OSM provider {endpoint} requested Retry-After beyond the bounded 5-minute retry window. Preparation stopped; try again later.");
                if (advisedWait.HasValue && advisedWait.Value > cooldown) cooldown = advisedWait.Value;
                if (!response.IsSuccessStatusCode)
                {
                    var code = (int)response.StatusCode;
                    if (code == 429) ReduceConcurrency();
                    if (code != 429 && code != 500 && code != 502 && code != 503 && code != 504 && code != 408)
                        throw new InvalidOperationException($"OSM tile {index + 1}/16: non-retryable HTTP {code} at {endpoint}; {ResponseSummary(bytes)}");
                    throw new HttpRequestException($"HTTP {(int)response.StatusCode} ({response.ReasonPhrase}); server response: {ResponseSummary(bytes)}");
                }
                if (!IsValidOsm(bytes)) throw new InvalidDataException($"HTTP 200 contained invalid/incomplete OSM data; server response: {ResponseSummary(bytes)}");
                await _cache.WriteAsync("overpass", key, "osm", bytes, token).ConfigureAwait(false);
                _log.Info($"OSM tile {index + 1}/16 succeeded at {endpoint} in {elapsed.Elapsed.TotalSeconds:0.0}s ({bytes.Length:N0} bytes).");
                _log.Count("OSM downloaded tiles");
                return bytes;
            }
            catch (Exception exception) when (!token.IsCancellationRequested &&
                (exception is HttpRequestException || exception is OperationCanceledException || exception is InvalidDataException))
            {
                last = exception;
                var reason = exception is OperationCanceledException ? "HTTP client timed out (90s limit)." : exception.Message +
                    (exception.InnerException == null ? "" : $" ({exception.GetBaseException().GetType().Name}: {exception.GetBaseException().Message})");
                _log.Warn($"OSM tile {index + 1}/16 failed at {endpoint}, provider attempt {round + 1}/6, elapsed {elapsed.Elapsed.TotalSeconds:0.0}s: {reason}");
                _log.Count("OSM retries");
                lock (_scheduler) { var until = _now() + cooldown; if (until > _availableAt[providerIndex]) _availableAt[providerIndex] = until; }
                if (round != 5 || provider != Endpoints.Length - 1)
                    await _delay(TimeSpan.FromSeconds(1), token).ConfigureAwait(false);
            }
        }
        throw new HttpRequestException($"OSM tile {index + 1}/16 failed after 6 attempts per provider ({Endpoints.Length * 6} attempts total). Preparation stopped; no map contents changed. Last failure: {last?.Message}");
    }

    private static string BuildQuery(GeoBounds b, bool includeSecondary)
        => BuildNetworkQuery(b, includeSecondary ? RoadImportLevel.Secondary : RoadImportLevel.Primary, true);

    public static string BuildNetworkQuery(GeoBounds b, RoadImportLevel roadLevel, bool rail)
    {
        var classes = NetworkSelection.Classes(roadLevel);
        var bbox = string.Join(",", new[] { F(b.South), F(b.West), F(b.North), F(b.East) });
        var roads = classes.Length == 0 ? "" : $"way[\"highway\"~\"^({classes})$\"]({bbox});";
        var tracks = rail ? $"way[\"railway\"=\"rail\"]({bbox});" : "";
        // Independent forest acquisition remains in the existing shared query.
        return $"[out:xml][timeout:60];({roads}{tracks}way[\"natural\"=\"wood\"]({bbox});way[\"landuse\"=\"forest\"]({bbox});relation[\"natural\"=\"wood\"]({bbox});relation[\"landuse\"=\"forest\"]({bbox}););(._;>>;);out body;";
    }

    private static string F(double value) => value.ToString("0.########", CultureInfo.InvariantCulture);

    private static string BuildGeographyQuery(GeoBounds b)
    {
        var bbox = string.Join(",", new[] { F(b.South), F(b.West), F(b.North), F(b.East) });
        return $"[out:xml][timeout:60];(way[\"natural\"~\"^(water|coastline|beach|sand|grassland|scrub|bare_rock)$\"]({bbox});relation[\"natural\"~\"^(water|beach|sand|grassland|scrub)$\"]({bbox});way[\"waterway\"~\"^(river|stream|riverbank)$\"]({bbox});relation[\"waterway\"=\"riverbank\"]({bbox});way[\"landuse\"~\"^(reservoir|grass|meadow|farmland|quarry)$\"]({bbox});relation[\"landuse\"~\"^(reservoir|grass|meadow|farmland|quarry)$\"]({bbox}););(._;>>;);out body;";
    }

    private static bool IsValidOsm(byte[] bytes)
    {
        if (bytes.Length < 30) return false;
        try
        {
            using var stream = new System.IO.MemoryStream(bytes, writable: false);
            var document = XDocument.Load(stream);
            return document.Root?.Name.LocalName == "osm" && !document.Root.Elements("remark").Any();
        }
        catch { return false; }
    }

    private static string ResponseSummary(byte[] bytes)
    {
        if (bytes.Length == 0) return "(empty response)";
        var text = Encoding.UTF8.GetString(bytes, 0, Math.Min(bytes.Length, 8192));
        text = Regex.Replace(Regex.Replace(text, "<[^>]*>", " "), @"\s+", " ").Trim();
        return text.Length > 600 ? text.Substring(0, 600) + "…" : text;
    }
}
