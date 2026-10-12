using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Net.Http;
using System.Security.Cryptography;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;
using System.Threading.Tasks;
using System.Xml.Linq;
using ImportEarthLocation.Core;
using ImportEarthLocation.Geo;

namespace ImportEarthLocation.Data;

internal static class OverpassProviders
{
    internal static readonly string[] Endpoints = BuildEndpoints(null);
    internal static int? PreferredIndex => null;

    internal static string[] BuildEndpoints(string? primary)
    {
        var fallback = new[] {
        "https://overpass-api.de/api/interpreter",
        "https://overpass.private.coffee/api/interpreter",
        "https://maps.mail.ru/osm/tools/overpass/api/interpreter"
        };
        return string.IsNullOrWhiteSpace(primary) ? fallback : new[] { primary! }.Concat(fallback).ToArray();
    }
}

public sealed class OverpassClient
{
    private readonly string[] Endpoints;
    private static readonly object RoutingGate = new();
    private static string _routingKey = "";
    private static OverpassRequestScheduler _routingScheduler = new();

    private readonly CacheStore _cache;
    private readonly ImportLog _log;
    private readonly HttpClient _http;
    private readonly Func<TimeSpan, CancellationToken, Task> _delay;
    private readonly Func<DateTimeOffset> _now;
    private readonly OverpassRequestScheduler _scheduler;
    private readonly bool _geography;
    private readonly bool _forestOnly;
    private readonly bool _includeGroundSurfaces = true;
    private readonly RoadImportLevel? _roadLevel;
    private readonly bool _importRail = true;

    private string Category => _forestOnly ? "forests" : _geography ? "geography" : "networks";
    private double _activeSeconds;
    private double _activeBudgetSeconds = double.PositiveInfinity;
    private readonly object _budgetGate = new();
    private bool _allowPartial;

    private string Query(GeoBounds bounds, bool secondary) => _forestOnly ? BuildForestQuery(bounds) : _geography ? BuildGeographyQuery(bounds, _includeGroundSurfaces) : _roadLevel.HasValue ? BuildNetworkQuery(bounds, _roadLevel.Value, _importRail) : BuildQuery(bounds, secondary);
    private string TileKey(string query, int index, bool secondary) => _geography ? $"overpass-v3:{index}:{secondary}:{query}" : $"overpass-v4:{Category}:{index}:{query}";

    public Task<IReadOnlyList<byte[]>> DownloadMapAsync(GeoBounds bounds, bool includeSecondary, CancellationToken token, double activeBudgetSeconds = double.PositiveInfinity, bool allowPartial = false)
    {
        _activeBudgetSeconds = activeBudgetSeconds;
        _allowPartial = allowPartial;
        token.ThrowIfCancellationRequested();
        _log.Diagnostic($"OSM MODE category={Category}; coverage=16-tiles; wholeMapAttempt=false; reason=stability-rollback; cachedTilesRetained=true");
        return Download16Async(bounds, includeSecondary, token);
    }

    public OverpassClient(CacheStore cache, ImportLog log, bool geography = false, RoadImportLevel? roadLevel = null, bool importRail = true, bool forestOnly = false, bool includeGroundSurfaces = true)
        : this(cache, log, PublicHttpClient.Instance, Task.Delay, () => DateTimeOffset.UtcNow)
    {
        var primary = ServicePreferences.Load(log);
        lock (RoutingGate)
        {
            Endpoints = OverpassProviders.BuildEndpoints(primary);
            if (_routingKey != primary)
            {
                _routingKey = primary;
                _routingScheduler = new OverpassRequestScheduler(Endpoints.Length, primary.Length == 0 ? null : 0);
            }
            _scheduler = _routingScheduler;
        }
        _geography = geography; _roadLevel = roadLevel; _importRail = importRail; _forestOnly = forestOnly; _includeGroundSurfaces = includeGroundSurfaces;
        log.Diagnostic($"OSM SERVICE configured={primary.Length > 0}; providers={Endpoints.Length}; credentialLogged=false");
    }

    internal OverpassClient(CacheStore cache, ImportLog log, HttpClient http,
        Func<TimeSpan, CancellationToken, Task> delay, Func<DateTimeOffset> now, OverpassRequestScheduler? scheduler = null)
    {
        _cache = cache;
        _log = log;
        _http = http;
        _delay = delay;
        _now = now;
        Endpoints = OverpassProviders.Endpoints;
        _scheduler = scheduler ?? new OverpassRequestScheduler();
    }

    public async Task<IReadOnlyList<byte[]>> Download16Async(GeoBounds bounds, bool includeSecondary, CancellationToken token)
    {
        token.ThrowIfCancellationRequested();
        if(!_geography&&!_forestOnly&&_roadLevel==RoadImportLevel.None&&!_importRail)
        { _log.Diagnostic("OSM category=networks; skipped=true; reason=roads-and-rail-disabled; httpRequests=0"); return Array.Empty<byte[]>(); }
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
                    _log.Info($"Downloading OSM {Category} tile {index + 1}/16.");
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
        await Task.WhenAll(Enumerable.Range(0, 2).Select(_ => Task.Run(Worker))).ConfigureAwait(false);
        _log.Time($"OSM {Category} download", elapsed.Elapsed.TotalSeconds);
        if (failure != null || token.IsCancellationRequested)
            _log.Diagnostic($"OSM ABORT category={Category}; completedTiles={results.Count(b=>b!=null)}/16; missingTiles={string.Join(",",Enumerable.Range(0,results.Length).Where(i=>results[i]==null).Select(i=>i+1))}; completedCacheRetained=true; activeSeconds={_activeSeconds:0.###}; elapsedSeconds={elapsed.Elapsed.TotalSeconds:0.###}; reason={failure?.Message ?? "cancelled/deadline"}");
        token.ThrowIfCancellationRequested();
        if (failure != null && _allowPartial && (failure is HttpRequestException || failure is InvalidDataException || failure is InvalidOperationException || failure is TimeoutException))
        {
            // Retry exhaustion ends remote work, not generation. Recover exact-query
            // cache entries for tiles that workers did not reach; never use stale bounds.
            for(var i=0;i<results.Length;i++)if(results[i]==null)
            {
                token.ThrowIfCancellationRequested();
                var key=TileKey(Query(tiles[i],includeSecondary),i,includeSecondary);
                var cached=await _cache.ReadValidAsync("overpass",key,"osm",IsValidOsm,token).ConfigureAwait(false);
                if(cached!=null)results[i]=cached;
            }
            var available=results.Where(b=>b!=null).ToArray();
            _log.Count("OSM unavailable download tiles",results.Length-available.Length);
            _log.Warn($"OSM {Category} download incomplete: {available.Length}/16 readable/cached tiles retained after retry exhaustion. Preparation continues; coverage is partial, not a complete-data success.");
            _log.Diagnostic($"OSM RECOVERY stage=download-{Category}; retainedTiles={available.Length}; missingTiles={string.Join(",",Enumerable.Range(0,results.Length).Where(i=>results[i]==null).Select(i=>i+1))}; generationContinues=true; coverageComplete=false");
            return available;
        }
        if (failure != null) throw failure;
        if (results.Any(bytes => bytes == null)) throw new InvalidDataException("Incomplete OSM tile coverage; preparation aborted.");
        _log.Diagnostic($"OSM TOTAL category={Category}; tiles={results.Length}; bytes={results.Sum(b=>(long)b.Length)}; largestTileBytes={results.Max(b=>b.Length)}; elapsedSeconds={elapsed.Elapsed.TotalSeconds:0.###}; sharedConcurrency={_scheduler.Concurrency}; perProviderConcurrency=1; subdivisions=4x4");
        return results;
    }

    public async Task<ImportEarthLocation.Processing.OsmDataset> RecoverDependenciesAsync(ImportEarthLocation.Processing.OsmDataset source,GeoBounds bounds,CancellationToken token)
    {
        var nodes=source.Nodes.ToDictionary(p=>p.Key,p=>p.Value);var tags=source.NodeTags.ToDictionary(p=>p.Key,p=>p.Value);var ways=source.Ways.ToDictionary(p=>p.Key,p=>p.Value);
        var requestIndex=0;var requestedIds=0;
        for(var pass=0;pass<2;pass++)
        {
            token.ThrowIfCancellationRequested();
            var missingNodes=ways.Values.SelectMany(w=>w.NodeIds).Concat(source.Relations.SelectMany(r=>r.OrderedMembers.Where(m=>m.Type=="node").Select(m=>m.Id))).Where(id=>!nodes.ContainsKey(id)).Distinct().OrderBy(id=>id).ToArray();
            var missingWays=source.Relations.SelectMany(r=>r.OrderedMembers.Where(m=>m.Type=="way").Select(m=>m.Id)).Where(id=>!ways.ContainsKey(id))
                .Concat(ways.Values.Where(w=>w.NodeIds.Any(id=>!nodes.ContainsKey(id))).Select(w=>w.Id)).Distinct().OrderBy(id=>id).ToArray();
            if(missingNodes.Length==0&&missingWays.Length==0)break;
            if(missingNodes.Length>4096||missingWays.Length>128)
                throw new InvalidDataException("Network dependencies exceed the bounded recovery budget. Preparation can be retried; the current map is unchanged.");
            var batches=Math.Max((missingNodes.Length+511)/512,(missingWays.Length+63)/64);
            for(var batch=0;batch<batches;batch++)
            {
                var nodeIds=missingNodes.Skip(batch*512).Take(512).ToArray();var wayIds=missingWays.Skip(batch*64).Take(64).ToArray();
                var query=BuildDependencyQuery(nodeIds,wayIds);
                var bytes=await DownloadOneAsync(bounds,false,requestIndex++,token,queryOverride:query,bypassCache:pass>0).ConfigureAwait(false);
                var recovered=ImportEarthLocation.Processing.OsmParser.Merge(new[]{bytes},_log,token,wholeMap:true);
                foreach(var p in recovered.Nodes)nodes[p.Key]=p.Value;foreach(var p in recovered.NodeTags)tags[p.Key]=p.Value;foreach(var p in recovered.Ways)ways[p.Key]=p.Value;
                requestedIds+=nodeIds.Length+wayIds.Length;
                _log.Diagnostic($"OSM DEPENDENCY RECOVERY pass={pass+1}; requestedNodes={nodeIds.Length}; requestedWays={wayIds.Length}; returnedNodes={recovered.Nodes.Count}; returnedWays={recovered.Ways.Count}; bytes={bytes.Length}; existingProvidersOnly=true; broadBboxRefetch=false");
            }
        }
        _log.Diagnostic($"OSM DEPENDENCY CLOSURE requests={requestIndex}; requestedIds={requestedIds}; remainingNodeReferences={ways.Values.SelectMany(w=>w.NodeIds).Distinct().Count(id=>!nodes.ContainsKey(id))}; requiredSelectionValidatedNext=true");
        return new ImportEarthLocation.Processing.OsmDataset{Nodes=nodes,NodeTags=tags,Ways=ways,Relations=source.Relations,MalformedFeaturesSkipped=source.MalformedFeaturesSkipped};
    }

    private async Task<byte[]> DownloadOneAsync(GeoBounds bounds, bool includeSecondary, int index, CancellationToken token, bool wholeMap = false,string? queryOverride=null,bool bypassCache=false)
    {
        var query = queryOverride??Query(bounds, includeSecondary);
        var queryId = QueryId(query);
        var tileLabel = queryOverride!=null?$"dependency-{index+1}":wholeMap ? "whole" : $"{index+1}/16";
        // Preserve the existing geography cache; new forest entries do not depend on network preferences.
        var key = queryOverride!=null?$"overpass-dependencies-v1:{query}":wholeMap ? $"overpass-whole-v1:{Category}:{query}" : TileKey(query, index, includeSecondary);
        var cached = bypassCache?null:await _cache.ReadValidAsync("overpass", key, "osm", IsValidOsm, token).ConfigureAwait(false);
        if (cached != null)
        {
            _log.Info($"OSM tile {index + 1}/16 loaded from validated cache ({cached.Length:N0} bytes).");
            _log.Count($"OSM {Category} cache hits");
            _log.Diagnostic($"OSM TILE category={Category}; tile={index+1}/16; cacheHit=true; bytes={cached.Length}; bounds={F(bounds.South)},{F(bounds.West)},{F(bounds.North)},{F(bounds.East)}");
            return cached;
        }

        Exception? last = null;
        // Six attempts at EACH independent provider, with short pauses on switching
        // and a separate cooldown before returning to a failed provider.
        var attempts = new int[Endpoints.Length];
        var limit = wholeMap ? 1 : queryOverride!=null?2:6;
        while (attempts.Any(a => a < limit))
        {
            token.ThrowIfCancellationRequested();
            lock (_budgetGate) if (_activeSeconds >= _activeBudgetSeconds) throw new TimeoutException($"OSM {Category} active HTTP budget exhausted ({_activeSeconds:0.0}/{_activeBudgetSeconds:0.0}s); queue time excluded.");
            var queued = Stopwatch.StartNew();
            var providerIndex = await _scheduler.AcquireAnyAsync(Category, p => attempts[p] < limit, Math.Max(0,index), _now, _delay, token).ConfigureAwait(false);
            var round = attempts[providerIndex]++;
            var endpoint = Endpoints[providerIndex];
            // The temporary provider URL contains an access key; do not print it.
            var endpointLabel = new Uri(endpoint).GetLeftPart(UriPartial.Authority);
            var elapsed = Stopwatch.StartNew();
            var cooldown = TimeSpan.FromSeconds(Math.Min(120, 30 * Math.Pow(2, round)));
            _log.Info($"OSM {Category} tile {tileLabel}: {endpointLabel}, provider attempt {round + 1}/{limit}, bounds {F(bounds.South)},{F(bounds.West)},{F(bounds.North)},{F(bounds.East)}.");
            _log.Diagnostic($"OSM REQUEST category={Category}; tile={tileLabel}; queryId={queryId}; queryChars={query.Length}; provider={endpointLabel}; providerAttempt={round+1}/{limit}");
            var acquired=true;var queuedSeconds=queued.Elapsed.TotalSeconds;var responseBytes=0;var status=0;
            try
            {
                using var content = new FormUrlEncodedContent(new Dictionary<string, string> { ["data"] = query });
                using var requestTimeout = CancellationTokenSource.CreateLinkedTokenSource(token);
                requestTimeout.CancelAfter(TimeSpan.FromSeconds(90));
                using var response = await _http.PostAsync(endpoint, content, requestTimeout.Token).ConfigureAwait(false);
                var bytes = await response.Content.ReadAsByteArrayAsync().ConfigureAwait(false);
                responseBytes=bytes.Length;status=(int)response.StatusCode;
                token.ThrowIfCancellationRequested();
                var retryAfter = response.Headers.RetryAfter;
                var advisedWait = retryAfter?.Delta ?? (retryAfter?.Date - _now());
                if (!response.IsSuccessStatusCode)
                {
                    var code = (int)response.StatusCode;
                    if (code == 429) _scheduler.ReduceConcurrency(_log,_now());
                    if (advisedWait.HasValue && advisedWait.Value > cooldown) cooldown = advisedWait.Value;
                    if (code == 400 || code == 422)
                        // Identical bad syntax must not be sprayed at every
                        // provider; this is an application query bug, not OSM loss.
                        throw new InvalidOperationException($"OSM {Category} tile {tileLabel}: non-retryable HTTP {code} at {endpointLabel}; queryId={queryId}; {ResponseSummary(bytes)}");
                    var retryable = code == 429 || code == 500 || code == 502 || code == 503 || code == 504 || code == 408;
                    if (!retryable || advisedWait.HasValue && advisedWait.Value > TimeSpan.FromMinutes(5))
                    {
                        // Authentication/retired route/provider-specific limits
                        // cannot be repaired by retrying this provider. Respect
                        // its cooldown but try the already configured fallbacks.
                        attempts[providerIndex] = limit;
                        if (cooldown < TimeSpan.FromMinutes(5)) cooldown = TimeSpan.FromMinutes(5);
                        _log.Diagnostic($"OSM PROVIDER unavailable provider={endpointLabel}; http={code}; thisTileProviderExhausted=true; existingFallbacksAllowed=true; cooldownSeconds={cooldown.TotalSeconds:0.###}");
                    }
                    throw new HttpRequestException($"HTTP {(int)response.StatusCode} ({response.ReasonPhrase}); server response: {ResponseSummary(bytes)}");
                }
                if (!IsValidOsm(bytes)) throw new InvalidDataException($"HTTP 200 contained invalid/incomplete OSM data; server response: {ResponseSummary(bytes)}");
                await _cache.WriteAsync("overpass", key, "osm", bytes, token).ConfigureAwait(false);
                _scheduler.RecordSuccess(providerIndex, _now(), elapsed.Elapsed.TotalSeconds);
                _log.Info($"OSM tile {index + 1}/16 succeeded at {endpointLabel} in {elapsed.Elapsed.TotalSeconds:0.0}s ({bytes.Length:N0} bytes).");
                _log.Count($"OSM {Category} downloaded tiles");
                _log.Diagnostic($"OSM TILE category={Category}; tile={tileLabel}; queryId={queryId}; cacheHit=false; provider={endpointLabel}; providerAttempt={round+1}/{limit}; http={status}; bytes={responseBytes}; queueSeconds={queuedSeconds:0.###}; elapsedSeconds={elapsed.Elapsed.TotalSeconds:0.###}; bounds={F(bounds.South)},{F(bounds.West)},{F(bounds.North)},{F(bounds.East)}");
                if (wholeMap) _log.Diagnostic($"OSM WHOLE category={Category}; cacheHit=false; provider={endpointLabel}; bytes={bytes.Length}; complete=true; elapsedSeconds={elapsed.Elapsed.TotalSeconds:0.###}");
                return bytes;
            }
            catch (Exception exception) when (!token.IsCancellationRequested &&
                (exception is HttpRequestException || exception is OperationCanceledException || exception is InvalidDataException))
            {
                last = exception;
                elapsed.Stop();
                var reason = exception is OperationCanceledException ? "HTTP client timed out (90s limit)." : Sanitize(exception.Message +
                    (exception.InnerException == null ? "" : $" ({exception.GetBaseException().GetType().Name}: {exception.GetBaseException().Message})"));
                _log.Warn($"OSM {Category} tile {tileLabel} failed at {endpointLabel}, provider attempt {round + 1}/{limit}, elapsed {elapsed.Elapsed.TotalSeconds:0.0}s: {reason}");
                _log.Count($"OSM {Category} retries");
                _log.Diagnostic($"OSM FAILURE category={Category}; tile={tileLabel}; queryId={queryId}; provider={endpointLabel}; http={status}; bytes={responseBytes}; queueSeconds={queuedSeconds:0.###}; elapsedSeconds={elapsed.Elapsed.TotalSeconds:0.###}; reason={reason}");
                _scheduler.CoolDown(providerIndex,_now()+cooldown);
                _scheduler.RecordFailure(providerIndex, _now(), _log);
                if(acquired){_scheduler.Release(providerIndex);acquired=false;}
                // A genuine query execution failure benefits from smaller bounds; admission/rate-limit failures do not.
                if (wholeMap && (reason.IndexOf("Query timed out", StringComparison.OrdinalIgnoreCase) >= 0 || exception is InvalidDataException))
                    throw new InvalidDataException("Whole-map query execution/validation failed: " + reason, exception);
                if (attempts.Any(a => a < limit)) await _delay(TimeSpan.FromSeconds(1), token).ConfigureAwait(false);
            }
            finally { lock (_budgetGate) _activeSeconds += elapsed.Elapsed.TotalSeconds; if(acquired)_scheduler.Release(providerIndex); }
        }
        throw new HttpRequestException($"OSM {Category} {(wholeMap?"whole-map attempt":$"tile {tileLabel}")} exhausted up to {limit} attempts per provider ({Endpoints.Length * limit} attempts total maximum); queryId={queryId}. No map contents changed. Last failure: {Sanitize(last?.Message ?? "unknown")}");
    }

    private static string BuildQuery(GeoBounds b, bool includeSecondary)
        => BuildNetworkQuery(b, includeSecondary ? RoadImportLevel.Secondary : RoadImportLevel.Primary, true);

    internal static string BuildDependencyQuery(IReadOnlyList<long> nodes, IReadOnlyList<long> ways)
        => "[out:xml][timeout:60];("+(nodes.Count>0?$"node(id:{string.Join(",",nodes)});":"")+(ways.Count>0?$"way(id:{string.Join(",",ways)});":"")+");out body qt;>;out skel qt;";

    private static string QueryId(string query)
    {
        using var sha=SHA256.Create();
        return BitConverter.ToString(sha.ComputeHash(Encoding.UTF8.GetBytes(query)),0,6).Replace("-", "").ToLowerInvariant();
    }

    private static string Sanitize(string text)
        => Regex.Replace(text,@"https?://overpass\.nextgis\.com/[^/\s\""<>]+","https://overpass.nextgis.com/[redacted]",RegexOptions.IgnoreCase);

    public static string BuildNetworkQuery(GeoBounds b, RoadImportLevel roadLevel, bool rail)
    {
        var classes = NetworkSelection.Classes(roadLevel);
        var bbox = string.Join(",", new[] { F(b.South), F(b.West), F(b.North), F(b.East) });
        var roads = classes.Length == 0 ? "" : $"way[\"highway\"~\"^({classes})$\"]({bbox});";
        var tracks = rail ? $"way[\"railway\"=\"rail\"]({bbox});" : "";
        var walls = roads.Length>0||tracks.Length>0 ? $"way[\"barrier\"=\"retaining_wall\"]({bbox});" : "";
        if(roads.Length==0&&tracks.Length==0)return "[out:xml][timeout:60];out body;";
        // Bounded enrichment uses this same provider/cache/scheduler. One-hop
        // support candidates are filtered locally; only roundabout dependencies
        // receive a second expansion, never an unbounded city-wide network walk.
        return $"[out:xml][timeout:60];({roads}{tracks})->.network;node(w.network)->.junctions;"+
            "way(bn.junctions)[\"highway\"~\"^(unclassified|residential|service)$\"]->.support;"+
            "(.network;.support;)->.arms;node(w.arms)->.armnodes;way(bn.armnodes)[\"junction\"=\"roundabout\"]->.rings;"+
            "node(w.rings)->.ringnodes;way(bn.ringnodes)[\"junction\"=\"roundabout\"]->.rings;node(w.rings)->.ringnodes;way(bn.ringnodes)[\"junction\"=\"roundabout\"]->.rings;"+
            "(.network;.support;.rings;)->.routes;rel(bw.routes)[\"type\"~\"^(restriction|connectivity)$\"]->.movements;"+
            $"({walls}way[\"man_made\"=\"bridge\"]({bbox});way[\"area:highway\"]({bbox});)->.footprints;"+
            "(.routes;way(r.movements);.footprints;)->.ways;(.ways;.movements;);out body qt;"+
            "(node(w.ways);node(r.movements);)->.nodes;"+
            "(node.nodes[\"highway\"];node.nodes[\"railway\"];node.nodes[\"barrier\"];node.nodes[\"ele\"];)->.tagged;"+
            // Overpass tokenizes '-.' as a type unless the difference operator
            // is separated from the named set. Preserve selected node tags.
            ".tagged out body qt;(.nodes; - .tagged;);out skel qt;";
    }

    public static string BuildForestQuery(GeoBounds b)
    {
        var bbox=string.Join(",",new[]{F(b.South),F(b.West),F(b.North),F(b.East)});
        return $"[out:xml][timeout:60];(node[\"natural\"~\"^(tree|shrub)$\"]({bbox});way[\"natural\"~\"^(tree_row|wood|scrub|heath|wetland|beach)$\"]({bbox});way[\"landuse\"~\"^(forest|orchard)$\"]({bbox});relation[\"natural\"~\"^(wood|scrub|heath|wetland|beach)$\"]({bbox});relation[\"landuse\"~\"^(forest|orchard)$\"]({bbox}););" + CompleteAreaOutput(mappedTrees: true);
    }

    private static string F(double value) => value.ToString("0.########", CultureInfo.InvariantCulture);

    public static string BuildGeographyQuery(GeoBounds b, bool includeGroundSurfaces = true)
    {
        var bbox = string.Join(",", new[] { F(b.South), F(b.West), F(b.North), F(b.East) });
        // Beaches and quarries protect terrain even when painting is disabled.
        // Painting consumes land-cover and drainage tags, never climate data.
        // Supporting nodes remain skeleton-only; selected/member ways retain all tags.
        var cover = "sand|dune|bare_rock|scree|shingle|rock|mud|bare_earth|scrub|heath|grassland|wetland|wood";
        var naturalWays = includeGroundSurfaces ? "water|coastline|beach|" + cover : "water|coastline|beach";
        var naturalRelations = includeGroundSurfaces ? "water|beach|" + cover : "water|beach";
        var landuse = includeGroundSurfaces ? "reservoir|quarry|grass|meadow|forest|farmland|orchard|vineyard|residential|commercial|industrial|retail|construction|brownfield|cemetery" : "reservoir|quarry";
        var waterways = "river|stream|riverbank|canal|drain|ditch";
        var surfaceAreas = includeGroundSurfaces ? $"way[\"wetland\"]({bbox});relation[\"wetland\"]({bbox});way[\"landcover\"=\"sand\"]({bbox});relation[\"landcover\"=\"sand\"]({bbox});" +
            $"way[\"area\"=\"yes\"][\"surface\"~\"^(sand|rock|stone|gravel|pebblestone|shingle|earth|ground|dirt|mud|grass|dry_grass|asphalt|concrete|paved|paving_stones|cobblestone)$\"][!\"highway\"][!\"railway\"][!\"building\"]({bbox});" : "";
        return $"[out:xml][timeout:60];(way[\"natural\"~\"^({naturalWays})$\"]({bbox});relation[\"natural\"~\"^({naturalRelations})$\"]({bbox});way[\"waterway\"~\"^({waterways})$\"]({bbox});relation[\"waterway\"=\"riverbank\"]({bbox});way[\"landuse\"~\"^({landuse})$\"]({bbox});relation[\"landuse\"~\"^({landuse})$\"]({bbox});{surfaceAreas});" + CompleteAreaOutput(mappedTrees: false);
    }

    private static string CompleteAreaOutput(bool mappedTrees)
    {
        // Keep the existing recursive expansion, all member ways and their tags,
        // including inner rings. Only change the detail of supporting nodes.
        const string expand = "(._;>>;)->.features;";
        if (mappedTrees)
            // Preserve trees discovered on member geometry too, not only the
            // explicitly selected tree nodes. Avoid emitting each tree twice.
            return expand + "node.features[\"natural\"~\"^(tree|shrub)$\"]->.trees;(way.features;relation.features;.trees;);out body qt;(node.features; - .trees;);out skel qt;";
        return expand + "(way.features;relation.features;);out body qt;node.features;out skel qt;";
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
        text = Sanitize(System.Net.WebUtility.HtmlDecode(Regex.Replace(Regex.Replace(text, "<[^>]*>", " "), @"\s+", " ").Trim()));
        return text.Length > 600 ? text.Substring(0, 600) + "…" : text;
    }
}

/// <summary>One budget across network, forest and geography requests, including provider cooldowns.</summary>
public sealed class OverpassRequestScheduler
{
    private readonly object _gate=new();
    private readonly DateTimeOffset[] _availableAt;
    private readonly bool[] _providerActive;
    private int _active;
    private int _concurrency=2;
    private DateTimeOffset _recoverAfter;
    private readonly DateTimeOffset[] _lastSuccess;
    private readonly double[] _responseSeconds;
    private readonly int[] _failures;
    private readonly int? _preferredIndex;
    public OverpassRequestScheduler() : this(OverpassProviders.Endpoints.Length, OverpassProviders.PreferredIndex) { }
    internal OverpassRequestScheduler(int providerCount, int? preferredIndex)
    {
        if(providerCount < 1 || preferredIndex.HasValue && (preferredIndex.Value < 0 || preferredIndex.Value >= providerCount)) throw new ArgumentOutOfRangeException(nameof(providerCount));
        _preferredIndex=preferredIndex;
        _availableAt=new DateTimeOffset[providerCount];_providerActive=new bool[providerCount];
        _lastSuccess=new DateTimeOffset[providerCount];_responseSeconds=new double[providerCount];_failures=new int[providerCount];
    }
    private sealed record Waiting(string Category, Func<int, bool> Eligible, int Tile);
    private readonly List<Waiting> _waiting = new();
    private string _lastCategory = "";
    public int Concurrency { get { lock(_gate)return _concurrency; } }

    // Every round still visits each provider exactly once; preference cannot shrink the retry budget.
    public int[] ProviderOrder(int tile, DateTimeOffset now)
    {
        lock (_gate)
            return Enumerable.Range(0, _availableAt.Length).OrderBy(p => _availableAt[p] > now)
                .ThenBy(p => p == _preferredIndex ? 0 : 1)
                .ThenBy(p => _failures[p])
                .ThenBy(p => _providerActive[p])
                .ThenByDescending(p => _lastSuccess[p] != default && now - _lastSuccess[p] < TimeSpan.FromMinutes(10))
                .ThenBy(p => _lastSuccess[p] == default ? double.MaxValue : _responseSeconds[p])
                .ThenBy(p => (p - tile % _availableAt.Length + _availableAt.Length) % _availableAt.Length).ToArray();
    }

    public void RecordSuccess(int provider, DateTimeOffset now, double seconds)
    {
        lock (_gate) { _lastSuccess[provider] = now; _responseSeconds[provider] = seconds; _failures[provider] = 0; }
    }

    public void RecordFailure(int provider)
    {
        lock (_gate) _failures[provider] = Math.Min(6, _failures[provider] + 1);
    }

    public void RecordFailure(int provider, DateTimeOffset now, ImportLog log)
    {
        lock (_gate)
        {
            RecordFailure(provider);
            if (_failures[provider] < 2) return;
            var until = now.AddMinutes(Math.Min(10, Math.Pow(2, _failures[provider] - 1)));
            CoolDown(provider, until);
            log.Diagnostic($"OSM PROVIDER CIRCUIT providerIndex={provider}; consecutiveFailures={_failures[provider]}; unavailableUntil={_availableAt[provider]:O}; recoveryProbeLimit=1");
        }
    }

    public async Task<int> AcquireAnyAsync(string category, Func<int, bool> eligible, int tile,
        Func<DateTimeOffset> now, Func<TimeSpan, CancellationToken, Task> delay, CancellationToken token)
    {
        var request = new Waiting(category, eligible, tile);
        lock (_gate) _waiting.Add(request);
        try
        {
            while (true)
            {
                token.ThrowIfCancellationRequested(); var wait = TimeSpan.FromMilliseconds(10);
                lock (_gate)
                {
                    var time = now();
                    if (_concurrency == 1 && _recoverAfter != default && time >= _recoverAfter) { _concurrency = 2; _recoverAfter = default; }
                    if (_active < _concurrency)
                    {
                        // Alternate categories when both have dispatchable work, FIFO within each category.
                        foreach (var queued in _waiting.OrderBy(w => w.Category == _lastCategory))
                        {
                            // A busy primary is not a failed primary: queue for it
                            // instead of sending a fresh tile straight to a fallback.
                            // Cooldown/exhaustion still permits public-provider fallback.
                            var prefer = _preferredIndex.HasValue && queued.Eligible(_preferredIndex.Value) && _availableAt[_preferredIndex.Value] <= time;
                            var available = ProviderOrder(queued.Tile, time).Where(p => queued.Eligible(p) && (!prefer || p == _preferredIndex) && !_providerActive[p] && _availableAt[p] <= time).ToArray();
                            if (available.Length == 0) continue;
                            if (!ReferenceEquals(queued, request)) break;
                            var provider = available[0]; _active++; _providerActive[provider] = true; _lastCategory = category;
                            _waiting.Remove(request); return provider;
                        }
                    }
                    var times = Enumerable.Range(0, _availableAt.Length).Where(p => eligible(p)).Select(p => _availableAt[p] - time).ToArray();
                    if (times.Length > 0 && times.All(t => t > TimeSpan.Zero)) wait = times.Min() > TimeSpan.FromSeconds(10) ? TimeSpan.FromSeconds(10) : times.Min();
                }
                if (wait > TimeSpan.FromMilliseconds(10)) await delay(wait, token).ConfigureAwait(false);
                else await Task.Delay(10, token).ConfigureAwait(false);
            }
        }
        finally { lock (_gate) _waiting.Remove(request); }
    }

    public async Task AcquireAsync(int provider,Func<DateTimeOffset> now,Func<TimeSpan,CancellationToken,Task> delay,CancellationToken token)
    {
        while(true)
        {
            token.ThrowIfCancellationRequested();TimeSpan cooldown;
            lock(_gate)
            {
                if(_concurrency==1&&_recoverAfter!=default&&now()>=_recoverAfter){_concurrency=2;_recoverAfter=default;}
                cooldown=_availableAt[provider]-now();
                if(cooldown<=TimeSpan.Zero&&_active<_concurrency&&!_providerActive[provider]){_active++;_providerActive[provider]=true;return;}
            }
            if(cooldown>TimeSpan.Zero)await delay(cooldown>TimeSpan.FromSeconds(10)?TimeSpan.FromSeconds(10):cooldown,token).ConfigureAwait(false);
            else await Task.Delay(50,token).ConfigureAwait(false);
        }
    }
    public void Release(int provider){lock(_gate){_providerActive[provider]=false;_active--;}}
    public void CoolDown(int provider,DateTimeOffset until){lock(_gate){if(until>_availableAt[provider])_availableAt[provider]=until;}}
    public void ReduceConcurrency(ImportLog log,DateTimeOffset now)
    {
        lock(_gate)
        {
            _recoverAfter=now.AddMinutes(1);
            if(_concurrency<=1)return;_concurrency--;
            log.Warn($"Overpass rate limiting: reducing shared concurrency to {_concurrency}. Every category obeys the reduced limit; recovery is deferred until 60 seconds without another 429, and provider Retry-After/cooldowns still apply.");
        }
    }
}
